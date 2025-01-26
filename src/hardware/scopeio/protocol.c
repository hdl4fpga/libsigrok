/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2010 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2011 Olivier Fauchon <olivier@aixmarseille.com>
 * Copyright (C) 2012 Alexandru Gagniuc <mr.nuke.me@gmail.com>
 * Copyright (C) 2015 Bartosz Golaszewski <bgolaszewski@baylibre.com>
 * Copyright (C) 2019 Frank Stettner <frank-stettner@gmx.net>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include <config.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"
#include "protocol.h"

#define ANALOG_SAMPLES_PER_PERIOD 20

SR_PRIV const char *scopeio_analog_pattern_str[] = {
	"GN14",
	"GP14",
	"GN15",
	"GP15",
	"GN16",
	"GP16",
	"GN17",
	"GP17",
};

SR_PRIV int scopeio_sockfd;
SR_PRIV struct sockaddr_in scopeio_server_addr;

// SR_PRIV void scopeio_free_analog_pattern(struct dev_context *devc)
// {
	// g_free(devc->analog_patterns[GN14]);
	// g_free(devc->analog_patterns[GP14]);
	// g_free(devc->analog_patterns[GN15]);
	// g_free(devc->analog_patterns[GP15]);
	// g_free(devc->analog_patterns[GN16]);
	// g_free(devc->analog_patterns[GP16]);
	// g_free(devc->analog_patterns[GN17]);
	// g_free(devc->analog_patterns[GP17]);
// }

/*
 * Fixup a memory image of generated logic data before it gets sent to
 * the session's datafeed. Mask out content from disabled channels.
 *
 * TODO: Need we apply a channel map, and enforce a dense representation
 * of the enabled channels' data?
 */
static void logic_fixup_feed(struct dev_context *devc,
		struct sr_datafeed_logic *logic)
{
	size_t fp_off;
	uint8_t fp_mask;
	size_t off, idx;
	uint8_t *sample;

	fp_off = devc->first_partial_logic_index;
	fp_mask = devc->first_partial_logic_mask;
	if (fp_off == logic->unitsize)
		return;

	for (off = 0; off < logic->length; off += logic->unitsize) {
		sample = logic->data + off;
		sample[fp_off] &= fp_mask;
		for (idx = fp_off + 1; idx < logic->unitsize; idx++)
			sample[idx] = 0x00;
	}
}

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <unistd.h>

#define CHAR_WIDTH    8
#define CHANNELS      8
#define SAMPLE_WIDTH 13

float *decode (float *samples, const unsigned char *block, size_t length);
static int acc  = 0;
static int data = 0;
static int j = 0;

float *decode (float *samples, const unsigned char *block, size_t length)
{
	for (size_t i = 0; i < length; i++) {
		unsigned int sample;

		switch(*block++) {
		case 0x18:
			int length;
			length = *block++;

			for (int i = 0; i <= length; i++) {
				data <<= CHAR_WIDTH;
				data |= *block++;
				acc += CHAR_WIDTH;
				data &= ((1 << (SAMPLE_WIDTH+CHAR_WIDTH-1))-1);
				if (acc >= SAMPLE_WIDTH) {
					acc %= SAMPLE_WIDTH;
					sample = data;
					sample >>= acc;
					sample &= (1 << SAMPLE_WIDTH)-1;
					if (j < 1) {
						// fprintf(stderr,"---> %d\n", sample);
						*samples++ = sample;
					}
					j = (j+1) % 8;
				}
			}
			break;
		default:
			block += (*block + 2);
		}
	}
	return samples;
}

#define BLOCK 1024
static float values[1024];

static void send_analog_packet(
	struct analog_gen *ag,
	struct sr_dev_inst *sdi, 
	uint64_t *analog_sent)
{
	acc  = 0;
	data = 0;
	j = 0;
	float *xxx = values;
	for(int i = 0; i < 16; i++) {
		static union { char byte[4]; int word; } hton;
		static unsigned char buff[256];
		static unsigned char *ptr;

		ptr = buff+sizeof(short);
    	*ptr++ = 0x17;
    	*ptr++ = 0x02;
    	*ptr++ = 0x00;
    	*ptr++ = (BLOCK-1)/256; //0x03;
    	*ptr++ = (BLOCK-1)%256; //0xff;
    	*ptr++ = 0x16;
    	*ptr++ = 0x03;
		hton.word = htonl((i << 10));
    	*ptr++ = hton.byte[0] | 0x80;
    	*ptr++ = hton.byte[1];
    	*ptr++ = hton.byte[2];
    	*ptr++ = hton.byte[3];
		*(short *)buff = ptr-buff-sizeof(short);

		sendto(scopeio_sockfd, buff, ptr-buff, 0, (const struct sockaddr *)&scopeio_server_addr, sizeof(scopeio_server_addr));
		socklen_t addr_len = sizeof(scopeio_server_addr);
		static char unsigned buffer[6+BLOCK+2*((BLOCK+256-1)/256)];

		int n;
		for (ptr = buffer; (size_t) (ptr-buffer) < sizeof(buffer); ptr += n) {
			n = recvfrom(scopeio_sockfd, ptr, sizeof(buffer)-(ptr-buffer), 0, (struct sockaddr *)&scopeio_server_addr, &addr_len);
		}
		xxx = decode(xxx, buffer, sizeof(buffer));
	}

	struct sr_datafeed_packet packet;
	struct dev_context *devc;
	uint64_t sending_now, to_avg;
	int ag_pattern_pos;
	unsigned int i;
	float amplitude, offset, value;

	if (!ag->ch || !ag->ch->enabled)
		return;

	devc = sdi->priv;
	packet.type = SR_DF_ANALOG;
	packet.payload = &ag->packet;

	ag->packet.meaning->channels = g_slist_append(NULL, ag->ch);
	ag->packet.meaning->mq = ag->mq;
	ag->packet.meaning->mqflags = ag->mq_flags;

	/* Set a unit for the given quantity. */
	if (ag->mq == SR_MQ_VOLTAGE)
		ag->packet.meaning->unit = SR_UNIT_VOLT;
	else if (ag->mq == SR_MQ_CURRENT)
		ag->packet.meaning->unit = SR_UNIT_AMPERE;
	else if (ag->mq == SR_MQ_RESISTANCE)
		ag->packet.meaning->unit = SR_UNIT_OHM;
	else if (ag->mq == SR_MQ_CAPACITANCE)
		ag->packet.meaning->unit = SR_UNIT_FARAD;
	else if (ag->mq == SR_MQ_TEMPERATURE)
		ag->packet.meaning->unit = SR_UNIT_CELSIUS;
	else if (ag->mq == SR_MQ_FREQUENCY)
		ag->packet.meaning->unit = SR_UNIT_HERTZ;
	else if (ag->mq == SR_MQ_DUTY_CYCLE)
		ag->packet.meaning->unit = SR_UNIT_PERCENTAGE;
	else if (ag->mq == SR_MQ_CONTINUITY)
		ag->packet.meaning->unit = SR_UNIT_OHM;
	else if (ag->mq == SR_MQ_PULSE_WIDTH)
		ag->packet.meaning->unit = SR_UNIT_PERCENTAGE;
	else if (ag->mq == SR_MQ_CONDUCTANCE)
		ag->packet.meaning->unit = SR_UNIT_SIEMENS;
	else if (ag->mq == SR_MQ_POWER)
		ag->packet.meaning->unit = SR_UNIT_WATT;
	else if (ag->mq == SR_MQ_GAIN)
		ag->packet.meaning->unit = SR_UNIT_UNITLESS;
	else if (ag->mq == SR_MQ_SOUND_PRESSURE_LEVEL)
		ag->packet.meaning->unit = SR_UNIT_DECIBEL_SPL;
	else if (ag->mq == SR_MQ_CARBON_MONOXIDE)
		ag->packet.meaning->unit = SR_UNIT_CONCENTRATION;
	else if (ag->mq == SR_MQ_RELATIVE_HUMIDITY)
		ag->packet.meaning->unit = SR_UNIT_HUMIDITY_293K;
	else if (ag->mq == SR_MQ_TIME)
		ag->packet.meaning->unit = SR_UNIT_SECOND;
	else if (ag->mq == SR_MQ_WIND_SPEED)
		ag->packet.meaning->unit = SR_UNIT_METER_SECOND;
	else if (ag->mq == SR_MQ_PRESSURE)
		ag->packet.meaning->unit = SR_UNIT_HECTOPASCAL;
	else if (ag->mq == SR_MQ_PARALLEL_INDUCTANCE)
		ag->packet.meaning->unit = SR_UNIT_HENRY;
	else if (ag->mq == SR_MQ_PARALLEL_CAPACITANCE)
		ag->packet.meaning->unit = SR_UNIT_FARAD;
	else if (ag->mq == SR_MQ_PARALLEL_RESISTANCE)
		ag->packet.meaning->unit = SR_UNIT_OHM;
	else if (ag->mq == SR_MQ_SERIES_INDUCTANCE)
		ag->packet.meaning->unit = SR_UNIT_HENRY;
	else if (ag->mq == SR_MQ_SERIES_CAPACITANCE)
		ag->packet.meaning->unit = SR_UNIT_FARAD;
	else if (ag->mq == SR_MQ_SERIES_RESISTANCE)
		ag->packet.meaning->unit = SR_UNIT_OHM;
	else if (ag->mq == SR_MQ_DISSIPATION_FACTOR)
		ag->packet.meaning->unit = SR_UNIT_UNITLESS;
	else if (ag->mq == SR_MQ_QUALITY_FACTOR)
		ag->packet.meaning->unit = SR_UNIT_UNITLESS;
	else if (ag->mq == SR_MQ_PHASE_ANGLE)
		ag->packet.meaning->unit = SR_UNIT_DEGREE;
	else if (ag->mq == SR_MQ_DIFFERENCE)
		ag->packet.meaning->unit = SR_UNIT_UNITLESS;
	else if (ag->mq == SR_MQ_COUNT)
		ag->packet.meaning->unit = SR_UNIT_PIECE;
	else if (ag->mq == SR_MQ_POWER_FACTOR)
		ag->packet.meaning->unit = SR_UNIT_UNITLESS;
	else if (ag->mq == SR_MQ_APPARENT_POWER)
		ag->packet.meaning->unit = SR_UNIT_VOLT_AMPERE;
	else if (ag->mq == SR_MQ_MASS)
		ag->packet.meaning->unit = SR_UNIT_GRAM;
	else if (ag->mq == SR_MQ_HARMONIC_RATIO)
		ag->packet.meaning->unit = SR_UNIT_UNITLESS;
	else
		ag->packet.meaning->unit = SR_UNIT_UNITLESS;

	if (!devc->avg) {
		// ag_pattern_pos = analog_pos % pattern->num_samples;
		// sending_now = MIN(analog_todo, pattern->num_samples - ag_pattern_pos);
		// ag->packet.num_samples = sending_now;

		ag->packet.data = values;

		ag->packet.num_samples = xxx-values; //630;
		sr_session_send(sdi, &packet);

		/* Whichever channel group gets there first. */
		*analog_sent = MAX(*analog_sent, sending_now);
	} 
}

/* Callback handling data */
SR_PRIV int scopeio_prepare_data(int fd, int revents, void *cb_data)
{
	struct sr_dev_inst *sdi;
	struct dev_context *devc;
	struct sr_datafeed_packet packet;
	struct sr_datafeed_logic logic;
	struct analog_gen *ag;
	GHashTableIter iter;
	void *value;
	uint64_t analog_sent, sending_now;
	int64_t elapsed_us, limit_us, todo_us;
	int64_t trigger_offset;
	int pre_trigger_samples;

	(void)fd;
	(void)revents;

	sdi = cb_data;
	devc = sdi->priv;

	/* Just in case. */
	if (devc->cur_samplerate <= 0
			&& devc->num_analog_channels <= 0) {
		sr_dev_acquisition_stop(sdi);
		return G_SOURCE_CONTINUE;
	}

	/* What time span should we send samples for? */
	elapsed_us = g_get_monotonic_time() - devc->start_us;
	limit_us = 1000 * devc->limit_msec;
	if (limit_us > 0 && limit_us < elapsed_us)
		todo_us = MAX(0, limit_us - devc->spent_us);
	else
		todo_us = MAX(0, elapsed_us - devc->spent_us);

	if (devc->limit_samples > 0) {
		// if (devc->limit_samples < devc->sent_samples)
			// samples_todo = 0;
		// else if (devc->limit_samples - devc->sent_samples < samples_todo)
			// samples_todo = devc->limit_samples - devc->sent_samples;
	}

	// if (samples_todo == 0)
		// return G_SOURCE_CONTINUE;

	if (devc->limit_frames) {
		/* Never send more samples than a frame can fit... */
		// samples_todo = MIN(samples_todo, SAMPLES_PER_FRAME);
		/* ...or than we need to finish the current frame. */
		// samples_todo = MIN(samples_todo,
			// SAMPLES_PER_FRAME - devc->sent_frame_samples);
	}

	/* Calculate the actual time covered by this run back from the sample
	 * count, rounded towards zero. This avoids getting stuck on a too-low
	 * time delta with no samples being sent due to round-off.
	 */
	// todo_us = samples_todo * G_USEC_PER_SEC / devc->cur_samplerate;

	if (!devc->enabled_analog_channels)

		/* Logic */

		/* Analog, one channel at a time */
		analog_sent = 0;

	g_hash_table_iter_init(&iter, devc->ch_ag);
	while (g_hash_table_iter_next(&iter, NULL, &value)) {
		send_analog_packet(value, sdi, &analog_sent);
		fprintf(stderr,"*************\n");
	}

	devc->spent_us += todo_us;

	if (devc->limit_frames ) {
		std_session_send_df_frame_end(sdi);
		devc->limit_frames--;
		if (!devc->limit_frames) {
			sr_dbg("Requested number of frames reached.");
			sr_dev_acquisition_stop(sdi);
		}
	}

	if (devc->limit_frames) {
		std_session_send_df_frame_begin(sdi);
	}

	return G_SOURCE_CONTINUE;
}
