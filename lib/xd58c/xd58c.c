/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <autoconf.h>
#include <lib/xd58c.h>

LOG_MODULE_REGISTER(xd58c, CONFIG_XD58C_LOG_LEVEL);

static struct {
  const struct adc_dt_spec adc_chan;
  struct adc_sequence_options adc_seq_opts;
  struct adc_sequence adc_seq;
  int16_t adc_buf;
} _this = {
    .adc_chan = ADC_DT_SPEC_GET(DT_PATH(zephyr_user)),
};

/**
 * @brief Initialize the XD58C heart rate monitor.
 *
 * @return 0 on success, negative error code on failure.
 */
int xd58c_init(void) {

  int err = adc_is_ready_dt(&_this.adc_chan);
  if (!err) {
    LOG_ERR("ADC device not ready");
    return -ENODEV;
  }

  err = adc_channel_setup_dt(&_this.adc_chan);
  if (err < 0) {
    LOG_ERR("setup ADC channel (err %d)", err);
    return err;
  }

  _this.adc_seq_opts.interval_us = 5000 /* 5 ms */;
  _this.adc_seq_opts.extra_samplings = 0;
  _this.adc_seq_opts.callback = NULL;
  _this.adc_seq_opts.user_data = NULL;

  _this.adc_seq.options = &_this.adc_seq_opts;
  _this.adc_seq.channels = BIT(_this.adc_chan.channel_id);
  _this.adc_seq.buffer = &_this.adc_buf;
  _this.adc_seq.buffer_size = sizeof(_this.adc_buf);
  _this.adc_seq.resolution = _this.adc_chan.resolution;
  _this.adc_seq.oversampling = _this.adc_chan.oversampling;
  _this.adc_seq.calibrate = false;

  err = adc_sequence_init_dt(&_this.adc_chan, &_this.adc_seq);
  if (err < 0) {
    LOG_ERR("init ADC sequence (err %d)", err);
    return err;
  }

  return 0;
}

/**
 * @brief Set the callback function to be called after each ADC sampling is
 * done.
 */
void xd58c_callback_set(adc_sequence_callback callback, void *user_data) {
  _this.adc_seq_opts.callback = callback;
  _this.adc_seq_opts.user_data = user_data;
}

/**
 * @brief Start the heart rate monitor.
 *
 * @return 0 on success, negative error code on failure.
 */
int xd58c_start(void) {
  return adc_read_async(_this.adc_chan.dev, &_this.adc_seq, NULL);
}
