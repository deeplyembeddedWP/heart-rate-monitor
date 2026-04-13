/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <app_version.h>
#include <lib/xd58c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

static struct k_sem _complete_sem;

static enum adc_action _callback(const struct device *dev,
                                 const struct adc_sequence *sequence,
                                 uint16_t sampling_index) {
  LOG_INF("ADC callback (index %d) value: %d", sampling_index,
          ((int16_t *)sequence->buffer)[sampling_index]);

  if (sampling_index >= XD58C_BUFFER_SIZE_MAX - 1) {
    k_sem_give(&_complete_sem);
  }

  return ADC_ACTION_CONTINUE;
}

int main(void) {

  LOG_INF("Heart Rate Monitor %s", APP_VERSION_STRING);

  int err = xd58c_init();
  if (err) {
    LOG_ERR("init XD58C (err %d)", err);
    return err;
  }

  xd58c_callback_set(_callback, NULL);

  k_sem_init(&_complete_sem, 0, 1);

  err = xd58c_start();
  if (err) {
    LOG_ERR("start XD58C (err %d)", err);
    return err;
  }

  while (1) {
    err = k_sem_take(&_complete_sem, K_FOREVER);
    if (!err) {
      xd58c_start();
    }
  }

  return 0;
}