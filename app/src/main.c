/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <app_version.h>
#include <lib/xd58c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/**
 * @brief Callback function for ADC sequence completion.
 * @param dev Pointer to the ADC device.
 * @param sequence Pointer to the ADC sequence.
 * @param sampling_index Index of the current sampling.
 * @return The action to take after the callback.
 */
static enum adc_action _callback(const struct device *dev,
                                 const struct adc_sequence *sequence,
                                 uint16_t sampling_index) {
  int16_t raw = *(int16_t *)sequence->buffer;

  static int32_t dc = 0;
  dc = dc - (dc >> 5) + raw;
  int16_t ac = raw - (int16_t)(dc >> 5);

  LOG_INF("%d", ac);

  return ADC_ACTION_REPEAT;
}

int main(void) {

  LOG_INF("Heart Rate Monitor %s", APP_VERSION_STRING);

  int err = xd58c_init();
  if (err) {
    LOG_ERR("init XD58C (err %d)", err);
    return err;
  }

  xd58c_callback_set(_callback, NULL);

  err = xd58c_start();
  if (err) {
    LOG_ERR("start XD58C (err %d)", err);
    return err;
  }

  while (1) {
    k_sleep(K_SECONDS(1));
  }

  return 0;
}