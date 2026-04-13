/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef XD58C_H_
#define XD58C_H_

#include <zephyr/drivers/adc.h>

#define XD58C_BUFFER_SIZE_MAX 16

int xd58c_init(void);
int xd58c_start(void);
void xd58c_callback_set(adc_sequence_callback callback, void *user_data);

#endif /* XD58C_H_ */