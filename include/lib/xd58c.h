/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef XD58C_H_
#define XD58C_H_

#include <zephyr/drivers/adc.h>

#if defined(CONFIG_XD58C)
int xd58c_init(void);
int xd58c_start(void);
void xd58c_callback_set(adc_sequence_callback callback, void *user_data);
#else
static inline int xd58c_init(void) { return -ENOTSUP; }
static inline int xd58c_start(void) { return -ENOTSUP; }
static inline void xd58c_callback_set(adc_sequence_callback callback,
                                      void *user_data) {}
#endif /* CONFIG_XD58C */

#endif /* XD58C_H_ */