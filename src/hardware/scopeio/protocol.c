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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <unistd.h>

#define CHAR_WIDTH    8
#define CHANNELS      8
#define SAMPLE_WIDTH 13

float *decode (float *samples, int id, const unsigned char *block, size_t length);
static int acc  = 0;
static int data = 0;
static int j    = 0;

float *decode (float *samples, int id, const unsigned char *block, size_t length)
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
					if (j == id) {
						*samples++ = 3.3*(float)sample/4096.0;
						// *samples++ = sample;
						// fprintf(stderr,"================ %d\n", sample);
					}
					j = (j+1) % (ARRAY_SIZE(scopeio_analog_pattern_str));
				}
			}
			break;
		default:
			block += (*block + 2);
		}
	}
	return samples;
}

#define BLOCK  1024
#define BLOCKS 16
static float values[BLOCKS*BLOCK];
#define XXX ((6+BLOCK+2*BLOCK/256))
static char unsigned data_buffer[BLOCKS*XXX];
 

static void scopeio_xx(void);
static void scopeio_xx(void)
{
	unsigned char *data_ptr;
	data_ptr = data_buffer;
	for(int i = 0; i < BLOCKS; i++) {
		static union { char byte[4]; int word; } hton;
		static unsigned char rqst_buff[256];
		static unsigned char *rqst_ptr;

		rqst_ptr = rqst_buff+sizeof(short);
    	*rqst_ptr++ = 0x17;
    	*rqst_ptr++ = 0x02;
    	*rqst_ptr++ = 0x00;
    	*rqst_ptr++ = (BLOCK-1)/256;
    	*rqst_ptr++ = (BLOCK-1)%256;
    	*rqst_ptr++ = 0x16;
    	*rqst_ptr++ = 0x03;
		hton.word = htonl((i << 10));
    	*rqst_ptr++ = hton.byte[0] | 0x80;
    	*rqst_ptr++ = hton.byte[1];
    	*rqst_ptr++ = hton.byte[2];
    	*rqst_ptr++ = hton.byte[3];
		*(short *)rqst_buff = rqst_ptr-rqst_buff-sizeof(short);

		sendto(scopeio_sockfd, rqst_buff, rqst_ptr-rqst_buff, 0, (const struct sockaddr *)&scopeio_server_addr, sizeof(scopeio_server_addr));
		socklen_t addr_len = sizeof(scopeio_server_addr);

		int n;
		for (int j = 0 ;  j < XXX; j += n) {
			n = recvfrom(scopeio_sockfd, data_ptr, XXX-j, 0, (struct sockaddr *)&scopeio_server_addr, &addr_len);
			data_ptr += n;
		}
	}
}

static void send_analog_packet(
	struct analog_gen *ag,
	struct sr_dev_inst *sdi)
{
	float *last = values;
	acc  = 0;
	data = 0;
	j    = 0;

	last = decode(values, ag->id, data_buffer, sizeof(data_buffer));
		// fprintf(stderr,"================ %d\n", last-values);

	struct sr_datafeed_packet packet;
	struct dev_context *devc;

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

		ag->packet.data = values;

		ag->packet.num_samples = last-values; //630;
		sr_session_send(sdi, &packet);

		/* Whichever channel group gets there first. */
	} 
}

/* Callback handling data */
SR_PRIV int scopeio_prepare_data(int fd, int revents, void *cb_data)
{
	struct sr_dev_inst *sdi;
	struct dev_context *devc;
	GHashTableIter iter;
	void *value;

	(void)fd;
	(void)revents;

	sdi = cb_data;
	devc = sdi->priv;

	scopeio_xx();
	g_hash_table_iter_init(&iter, devc->ch_ag);
	while (g_hash_table_iter_next(&iter, NULL, &value)) {
		send_analog_packet(value, sdi);
	}
	std_session_send_df_frame_begin(sdi);
	sr_dev_acquisition_stop(sdi);
	return G_SOURCE_CONTINUE;

}
