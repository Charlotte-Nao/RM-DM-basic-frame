//
// Created by ASUS on 2026/9/10.
//

#ifndef DM_REMOTE_H
#define DM_REMOTE_H

#include <stdint.h>

#include "../application/global_data.h"

#define REMOTE_FRAME_LENGTH      18U

#define REMOTE_CHANNEL_MIN       364U
#define REMOTE_CHANNEL_CENTER    1024U
#define REMOTE_CHANNEL_MAX       1684U

/* Parsed switch values, ordered from top to bottom. */
#define REMOTE_SWITCH_UP         1U
#define REMOTE_SWITCH_MID        2U
#define REMOTE_SWITCH_DOWN       3U

int remote_parse(const uint8_t *data,
                 uint16_t length,
                 volatile remote_data_t *remote_data);

#endif // DM_REMOTE_H