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

#define DEFAULT_LOGIC_PATTERN			PATTERN_SIGROK

#define DEFAULT_NUM_ANALOG_CHANNELS		8


static const uint32_t scanopts[] = {
	SR_CONF_NUM_ANALOG_CHANNELS,
	SR_CONF_LIMIT_FRAMES,
};

static const uint32_t drvopts[] = {
	SR_CONF_SCOPEIO_DEV,
	SR_CONF_OSCILLOSCOPE,
};

static const uint32_t devopts[] = {
	SR_CONF_CONTINUOUS,
	SR_CONF_LIMIT_SAMPLES  | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_LIMIT_MSEC     | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_LIMIT_FRAMES   | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_SAMPLERATE     | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_TRIGGER_SOURCE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_TRIGGER_SLOPE  | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_TRIGGER_LEVEL  | SR_CONF_GET | SR_CONF_SET,
};

static const uint32_t devopts_cg_analog_group[] = {
	// SR_CONF_AMPLITUDE | SR_CONF_GET | SR_CONF_SET,
	// SR_CONF_OFFSET | SR_CONF_GET | SR_CONF_SET,
};

static const uint32_t devopts_cg_analog_channel[] = {
	SR_CONF_MEASURED_QUANTITY | SR_CONF_GET | SR_CONF_SET,
};

static const uint64_t samplerates[] = {
	SR_HZ(1024000/1),
	SR_HZ(1024000/2),
	SR_HZ(1024000/4),
	SR_HZ(1024000/5),
	SR_HZ(1024000/8),
};

static const char *trigger_slopes[] = {
	"POS", "NEG",
};

static GSList *scan(struct sr_dev_driver *di, GSList *options)
{
	struct dev_context *devc;
	struct sr_dev_inst *sdi;
	struct sr_channel *ch;
	struct sr_channel_group *cg, *acg;
	struct sr_config *src;
	struct analog_gen *ag;
	GSList *l;
	int num_analog_channels, i;
	uint64_t limit_frames;
	char channel_name[16];

	num_analog_channels = DEFAULT_NUM_ANALOG_CHANNELS;
	limit_frames = DEFAULT_LIMIT_FRAMES;
	for (l = options; l; l = l->next) {
		src = l->data;
		switch (src->key) {
		case SR_CONF_NUM_ANALOG_CHANNELS:
			num_analog_channels = g_variant_get_int32(src->data);
			break;
		case SR_CONF_LIMIT_FRAMES:
			limit_frames = g_variant_get_uint64(src->data);
			break;
		}
	}

	sdi = g_malloc0(sizeof(struct sr_dev_inst));
	sdi->status = SR_ST_INACTIVE;
	sdi->model = g_strdup("ScopeIO device");

	devc = g_malloc0(sizeof(struct dev_context));
	devc->cur_samplerate = samplerates[0];
	devc->num_analog_channels = num_analog_channels;
	devc->limit_frames = limit_frames;
	devc->capture_ratio = 20;
	devc->stl = NULL;
	strcpy(devc->trigger_slope, "POS");

	/* Analog channels, channel groups and pattern generators. */
	devc->ch_ag = g_hash_table_new(g_direct_hash, g_direct_equal);
	if (num_analog_channels > 0) {
		int id = 0;
		/* An "Analog" channel group with all analog channels in it. */
		acg = sr_channel_group_new(sdi, "Analog", NULL);

		for (i = 0; i < num_analog_channels; i++) {
			snprintf(channel_name, 16, scopeio_analog_pattern_str[i]);
			ch = sr_channel_new(sdi, i, SR_CHANNEL_ANALOG,
					TRUE, channel_name);
			acg->channels = g_slist_append(acg->channels, ch);

			/* Every analog channel gets its own channel group as well. */
			cg = sr_channel_group_new(sdi, channel_name, NULL);
			cg->channels = g_slist_append(NULL, ch);

			/* Every channel gets a generator struct. */
			ag = g_malloc(sizeof(struct analog_gen));
			ag->ch = ch;
			ag->mq = SR_MQ_VOLTAGE;
			ag->mq_flags = SR_MQFLAG_DC;
			ag->unit = SR_UNIT_VOLT;
			ag->amplitude = DEFAULT_ANALOG_AMPLITUDE;
			ag->offset = DEFAULT_ANALOG_OFFSET;
			sr_analog_init(&ag->packet, &ag->encoding, &ag->meaning, &ag->spec, 2);
			ag->packet.meaning->channels = cg->channels;
			ag->packet.meaning->mq = ag->mq;
			ag->packet.meaning->mqflags = ag->mq_flags;
			ag->packet.meaning->unit = ag->unit;
			ag->packet.encoding->digits = DEFAULT_ANALOG_ENCODING_DIGITS;
			ag->packet.spec->spec_digits = DEFAULT_ANALOG_SPEC_DIGITS;
			ag->id = id;
			ag->avg_val = 0.0f;
			ag->num_avgs = 0;
			g_hash_table_insert(devc->ch_ag, ch, ag);

			id = (id+1) % ARRAY_SIZE(scopeio_analog_pattern_str);
		}
	}

	sdi->priv = devc;

	return std_scan_complete(di, g_slist_append(NULL, sdi));
}

static void clear_helper(struct dev_context *devc)
{
	GHashTableIter iter;
	void *value;

	// scopeio_free_analog_pattern(devc);

	/* Analog generators. */
	g_hash_table_iter_init(&iter, devc->ch_ag);
	while (g_hash_table_iter_next(&iter, NULL, &value))
		g_free(value);
	g_hash_table_unref(devc->ch_ag);
}

static int dev_clear(const struct sr_dev_driver *di)
{
	return std_dev_clear_with_callback(di, (std_dev_clear_callback)clear_helper);
}

static int config_get(
	uint32_t key, 
	GVariant **data,
	const struct sr_dev_inst *sdi, 
	const struct sr_channel_group *cg)
{
	struct dev_context *devc;
	struct sr_channel *ch;
	struct analog_gen *ag;
	GVariant *mq_arr[2];

	if (!sdi)
		return SR_ERR_ARG;

	devc = sdi->priv;
	switch (key) {
	case SR_CONF_SAMPLERATE:
		*data = g_variant_new_uint64(devc->cur_samplerate);
		break;
	case SR_CONF_LIMIT_SAMPLES:
		*data = g_variant_new_uint64(devc->limit_samples);
		break;
	case SR_CONF_LIMIT_MSEC:
		*data = g_variant_new_uint64(devc->limit_msec);
		break;
	case SR_CONF_LIMIT_FRAMES:
		*data = g_variant_new_uint64(devc->limit_frames);
		break;
	case SR_CONF_MEASURED_QUANTITY:
		/* Any channel in the group will do. */
		ch = cg->channels->data;
		ag = g_hash_table_lookup(devc->ch_ag, ch);
		mq_arr[0] = g_variant_new_uint32(ag->mq);
		mq_arr[1] = g_variant_new_uint64(ag->mq_flags);
		*data = g_variant_new_tuple(mq_arr, 2);
		break;
	case SR_CONF_TRIGGER_SOURCE:
		*data = g_variant_new_string("GN14");
		break;
	case SR_CONF_TRIGGER_SLOPE:
		if (!strncmp(devc->trigger_slope, "POS", 3)) {
			*data = g_variant_new_string("POS");
		} else if (!strncmp(devc->trigger_slope, "NEG", 3)) {
			*data = g_variant_new_string("NEG");
		} else {
			sr_dbg("Unknown trigger slope: '%s'.", devc->trigger_slope);
			return SR_ERR_NA;
		}
		break;
	case SR_CONF_TRIGGER_LEVEL:
		*data = g_variant_new_double(devc->trigger_level);
		break;
	default:
		return SR_ERR_NA;
	}

	return SR_OK;
}

static int config_set(
	uint32_t key,
	GVariant *data,
	const struct sr_dev_inst *sdi,
	const struct sr_channel_group *cg)
{
	struct dev_context *devc;
	struct analog_gen *ag;
	struct sr_channel *ch;
	GVariant *mq_tuple_child;
	GSList *l;

	devc = sdi->priv;

	switch (key) {
	case SR_CONF_SAMPLERATE:
		devc->cur_samplerate = g_variant_get_uint64(data);
		break;
	case SR_CONF_LIMIT_SAMPLES:
		devc->limit_msec = 0;
		devc->limit_samples = g_variant_get_uint64(data);
		break;
	case SR_CONF_LIMIT_MSEC:
		devc->limit_msec = g_variant_get_uint64(data);
		devc->limit_samples = 0;
		break;
	case SR_CONF_LIMIT_FRAMES:
		devc->limit_frames = g_variant_get_uint64(data);
		break;
	case SR_CONF_MEASURED_QUANTITY:
		for (l = cg->channels; l; l = l->next) {
			ch = l->data;
			ag = g_hash_table_lookup(devc->ch_ag, ch);
			mq_tuple_child = g_variant_get_child_value(data, 0);
			ag->mq = g_variant_get_uint32(mq_tuple_child);
			mq_tuple_child = g_variant_get_child_value(data, 1);
			ag->mq_flags = g_variant_get_uint64(mq_tuple_child);
			g_variant_unref(mq_tuple_child);
		}
		break;
	case SR_CONF_TRIGGER_SOURCE:
		break;
	case SR_CONF_TRIGGER_SLOPE:
		break;
	case SR_CONF_TRIGGER_LEVEL:
		devc->trigger_level = g_variant_get_double(data);
		break;
	default:
		return SR_ERR_NA;
	}

	return SR_OK;
}

static int config_list(
	uint32_t key, 
	GVariant **data, 
	const struct sr_dev_inst *sdi, 
	const struct sr_channel_group *cg)
{
	(void) sdi;

	switch (key) {
	case SR_CONF_SCAN_OPTIONS:
		if (cg != NULL)
			return SR_ERR_NA;
	case SR_CONF_DEVICE_OPTIONS:
		if (cg == NULL)
			return STD_CONFIG_LIST(key, data, sdi, cg, scanopts, drvopts, devopts);
		else { 
			struct sr_channel *ch = cg->channels->data;

			if (ch->type == SR_CHANNEL_ANALOG) {
				// if (strcmp(cg->name, "Analog") == 0)
					// *data = std_gvar_array_u32(ARRAY_AND_SIZE(devopts_cg_analog_group));
				// else
				*data = std_gvar_array_u32(ARRAY_AND_SIZE(devopts_cg_analog_channel));
			} else
				return SR_ERR_BUG;
		}
		break;
	case SR_CONF_SAMPLERATE:
		if (cg == NULL)
			*data = std_gvar_samplerates_steps(ARRAY_AND_SIZE(samplerates));
		else
			return SR_ERR_NA;
		break;
	case SR_CONF_TRIGGER_SOURCE:
		*data = g_variant_new_strv(scopeio_analog_pattern_str, GP17+1);
		break;
	case SR_CONF_TRIGGER_SLOPE:
		*data = g_variant_new_strv(ARRAY_AND_SIZE(trigger_slopes));
		break;
	default:
		return SR_ERR_NA;
	}

	return SR_OK;
}

static int dev_acquisition_start(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	GSList *l;
	struct sr_channel *ch;
	// struct sr_trigger *trigger;

	devc = sdi->priv;

	/* Setup triggers */
	// if ((trigger = sr_session_trigger_get(sdi->session))) {
		// devc->stl = soft_trigger_logic_new(sdi, trigger, pre_trigger_samples);
		// if (!devc->stl)
			// return SR_ERR_MALLOC;
// 
		// /* Disable all analog channels since using them when there are logic
		//  * triggers set up would require having pre-trigger sample buffers
		//  * for analog sample data.
		//  */
		// for (l = sdi->channels; l; l = l->next) {
			// ch = l->data;
			// if (ch->type == SR_CHANNEL_ANALOG)
				// ch->enabled = FALSE;
		// }
	// }

	/*
	 * Determine the numbers of logic and analog channels that are
	 * involved in the acquisition. Determine an offset and a mask to
	 * remove excess logic data content before datafeed submission.
	 */
	devc->enabled_analog_channels = 0;
	for (l = sdi->channels; l; l = l->next) {
		ch = l->data;
		if (!ch->enabled)
			continue;
		if (ch->type == SR_CHANNEL_ANALOG) {
			devc->enabled_analog_channels++;
			continue;
		}
		if (ch->type != SR_CHANNEL_LOGIC)
			continue;
		/*
		 * TODO: Need we create a channel map here, such that the
		 * session datafeed packets will have a dense representation
		 * of the enabled channels' data? For example store channels
		 * D3 and D5 in bit positions 0 and 1 respectively, when all
		 * other channels are disabled? The current implementation
		 * generates a sparse layout, might provide data for logic
		 * channels that are disabled while it might suppress data
		 * from enabled channels at the same time.
		 */
	}

	sr_session_source_add(sdi->session, -1, 0, 100,
			scopeio_prepare_data, (struct sr_dev_inst *)sdi);

	std_session_send_df_header(sdi);

	if (devc->limit_frames > 0)
		std_session_send_df_frame_begin(sdi);

	/* We use this timestamp to decide how many more samples to send. */
	devc->start_us = g_get_monotonic_time();
	devc->spent_us = 0;
	devc->step = 0;

	return SR_OK;
}

static int dev_acquisition_stop(struct sr_dev_inst *sdi)
{
	struct dev_context *devc;

	sr_session_source_remove(sdi->session, -1);

	devc = sdi->priv;
	if (devc->limit_frames > 0)
		std_session_send_df_frame_end(sdi);

	std_session_send_df_end(sdi);


	if (devc->stl) {
		soft_trigger_logic_free(devc->stl);
		devc->stl = NULL;
	}

	return SR_OK;
}

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <unistd.h>

#define PORT 8080

static int dev_open(struct sr_dev_inst *sdi)
{
	(void) sdi;
	if ((scopeio_sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		sr_err("Socket creation failed");
		return SR_ERR;
	}
	memset(&scopeio_server_addr, 0, sizeof(scopeio_server_addr));
	scopeio_server_addr.sin_family = AF_INET; // IPv4
	scopeio_server_addr.sin_port = htons(PORT);
	scopeio_server_addr.sin_addr.s_addr = INADDR_ANY;

	return SR_OK;
}

static int dev_close(struct sr_dev_inst *sdi)
{
	(void) sdi;
	return SR_OK;
}

static struct sr_dev_driver scopeio_driver_info = {
	.name = "ScopeIO",
	.longname = "ScopeIO driver",
	.api_version = 1,
	.init = std_init,
	.cleanup = std_cleanup,
	.scan = scan,
	.dev_list = std_dev_list,
	.dev_clear = dev_clear,
	.config_get = config_get,
	.config_set = config_set,
	.config_list = config_list,
	.dev_open = dev_open,
	.dev_close = dev_close,
	.dev_acquisition_start = dev_acquisition_start,
	.dev_acquisition_stop = dev_acquisition_stop,
	.context = NULL,
};
SR_REGISTER_DEV_DRIVER(scopeio_driver_info);
