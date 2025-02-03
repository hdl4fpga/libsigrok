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

#ifndef LIBSIGROK_HARDWARE_SCOPEIO_PROTOCOL_H
#define LIBSIGROK_HARDWARE_SCOPEIO_PROTOCOL_H

#include <stdint.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"

#define LOG_PREFIX "ScopeIO"

/* The size in bytes of chunks to send through the session bus. */
#define LOGIC_BUFSIZE			4096
/* Size of the analog pattern space per channel. */
#define ANALOG_BUFSIZE			4096
/* This is a development feature: it starts a new frame every n samples. */
#define SAMPLES_PER_FRAME		1000UL
#define DEFAULT_LIMIT_FRAMES		0

#define DEFAULT_ANALOG_ENCODING_DIGITS	4
#define DEFAULT_ANALOG_SPEC_DIGITS		4
#define DEFAULT_ANALOG_AMPLITUDE		10
#define DEFAULT_ANALOG_OFFSET			0.

enum analog_channel { GN14, GP14, GN15, GP15, GN16, GP16, GN17, GP17 };
extern SR_PRIV const char *scopeio_analog_pattern_str[8];

struct analog_pattern {
	float data[ANALOG_BUFSIZE];
	unsigned int num_samples;
};

struct scopeio_devcontext {
};

struct dev_context {
	uint64_t cur_samplerate;
	uint64_t limit_samples;
	uint64_t limit_msec;
	uint64_t limit_frames;
	int64_t start_us;
	int64_t spent_us;
	uint64_t step;
	uint8_t logic_data[LOGIC_BUFSIZE];
	/* Analog */
	// struct analog_pattern *analog_patterns[ARRAY_SIZE(analog_pattern_str)];
	char trigger_slope[16];
	float trigger_level;
	int32_t num_analog_channels;
	GHashTable *ch_ag;
	gboolean avg; /* True if averaging is enabled */
	uint64_t avg_samples;
	size_t enabled_analog_channels;
	/* Triggers */
	uint64_t capture_ratio;
	gboolean trigger_fired;
	struct soft_trigger_logic *stl;
};

struct analog_gen {
	struct sr_channel *ch;
	enum sr_mq mq;
	enum sr_mqflag mq_flags;
	enum sr_unit unit;
	enum analog_channel id;
	float amplitude;
	float offset;
	struct sr_datafeed_analog packet;
	struct sr_analog_encoding encoding;
	struct sr_analog_meaning meaning;
	struct sr_analog_spec spec;
	float avg_val; /* Average value */
	unsigned int num_avgs; /* Number of samples averaged */
};

SR_PRIV int scopeio_prepare_data(int fd, int revents, void *cb_data);
extern SR_PRIV int scopeio_sockfd;
extern SR_PRIV struct sockaddr_in scopeio_server_addr;

#endif
