/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#include <app_version.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

static const struct adc_dt_spec adc_chan =
    ADC_DT_SPEC_GET(DT_PATH(zephyr_user));

static int16_t _buffer[100] = {};

static struct adc_sequence_options _options = {
    .interval_us = 10000 /* 10 ms */,
    .extra_samplings = 99 /* 99 extra + 1 = 100 total */,
    .callback = NULL,
    .user_data = NULL,
};

static struct adc_sequence _config = {
    .buffer = _buffer,
    .buffer_size = sizeof(_buffer),
    .options = &_options,
    .calibrate = false,
};

int main(void) {
  LOG_INF("Heart Rate Monitor App version: %s", APP_VERSION_STRING);

  int err = adc_is_ready_dt(&adc_chan);
  if (!err) {
    LOG_ERR("ADC device not ready");
    return err;
  }

  err = adc_channel_setup_dt(&adc_chan);
  if (err < 0) {
    LOG_ERR("ADC channel setup failed: %d", err);
    return err;
  }

  err = adc_sequence_init_dt(&adc_chan, &_config);
  if (err < 0) {
    LOG_ERR("ADC sequence initialization failed: %d", err);
    return err;
  }

  while (1) {
  }

  return 0;
}
