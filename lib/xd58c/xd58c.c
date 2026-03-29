/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <autoconf.h>
#include <lib/xd58c.h>

LOG_MODULE_REGISTER(xd58c, CONFIG_XD58C_LOG_LEVEL);

/*---------------------------------------------------------------------------
 * Internal state — not exposed in xd58c.h
 * Keeps output callback and user_data alongside driver data
 *--------------------------------------------------------------------------*/
struct xd58c_internal {
  struct xd58c_data *data;
  xd58c_output_cb_t output_cb;
  void *output_user_data;
};

/* One internal state per driver instance */
static struct xd58c_internal _internal;

/* Processing thread stack — statically allocated */
K_THREAD_STACK_DEFINE(_proc_stack, CONFIG_XD58C_THREAD_STACK_SIZE);

/*---------------------------------------------------------------------------
 * Low-pass filter
 *--------------------------------------------------------------------------*/
#define XD58C_LPF_SIZE_MAX 16

static int32_t lpf(int32_t new_sample, uint8_t lpf_size, int32_t *buf, int *idx,
                   int32_t *sum) {
  *sum -= buf[*idx];
  buf[*idx] = new_sample;
  *sum += new_sample;
  *idx = (*idx + 1) % lpf_size;

  return *sum / lpf_size;
}

/*---------------------------------------------------------------------------
 * Peak detector
 *--------------------------------------------------------------------------*/
static void detect_bpm(int32_t mv, const struct xd58c_config *config,
                       bool *above_threshold, int64_t *last_peak_ms,
                       int32_t *bpm) {
  int64_t now_ms = k_uptime_get();

  /* Guard — finger not present, bypass detector */
  if (mv < config->finger_present_threshold_mv) {
    *above_threshold = false;
    return;
  }

  /* Timeout — reset BPM if no peak detected within max interval */
  if (*last_peak_ms != 0 &&
      (now_ms - *last_peak_ms) > config->max_peak_interval_ms) {
    *bpm = 0;
    *last_peak_ms = 0; /* reset so next detection starts fresh */
    *above_threshold = false;
    LOG_DBG("BPM timeout — no peak for %dms", config->max_peak_interval_ms);
  }

  if (!(*above_threshold) && mv > config->peak_threshold_mv) {
    *above_threshold = true;

    if (*last_peak_ms != 0) {
      int64_t interval_ms = now_ms - *last_peak_ms;

      if (interval_ms >= config->min_peak_interval_ms &&
          interval_ms <= config->max_peak_interval_ms) {
        *bpm = 60000 / interval_ms;
        LOG_DBG("Peak — interval=%lldms bpm=%d", interval_ms, (int)*bpm);
      }
    }

    *last_peak_ms = now_ms;

  } else if (*above_threshold && mv < config->peak_threshold_mv) {
    *above_threshold = false;
  }
}

/*---------------------------------------------------------------------------
 * ADC callback — ISR context
 *--------------------------------------------------------------------------*/
static enum adc_action adc_callback(const struct device *dev,
                                    const struct adc_sequence *seq,
                                    uint16_t sampling_index) {
  ARG_UNUSED(dev);
  ARG_UNUSED(sampling_index);

  struct xd58c_data *data = seq->options->user_data;

  if (k_msgq_put(data->msgq, &data->buffer, K_NO_WAIT) != 0) {
    LOG_DBG("pulse_msgq full — sample dropped");
  }

  return data->running ? ADC_ACTION_REPEAT : ADC_ACTION_FINISH;
}

/*---------------------------------------------------------------------------
 * Processing thread
 *
 * Runs at CONFIG_XD58C_THREAD_PRIORITY. Blocks on msgq waiting for
 * samples from the ADC callback. For each sample:
 *   1. Converts raw to mV
 *   2. Applies LPF
 *   3. Runs peak detection → BPM
 *   4. Calls output_cb with xd58c_sample (for graph/logging)
 *--------------------------------------------------------------------------*/
static void processing_thread(void *p1, void *p2, void *p3) {
  ARG_UNUSED(p2);
  ARG_UNUSED(p3);

  struct xd58c_internal *internal = (struct xd58c_internal *)p1;
  struct xd58c_data *data = internal->data;
  int16_t raw;

  LOG_INF("XD-58C processing thread started");

  while (1) {
    /* Block until ADC callback delivers a sample */
    if (k_msgq_get(data->msgq, &raw, K_FOREVER) != 0) {
      continue;
    }

    /* 1. Convert raw to millivolts */
    int32_t mv = raw;
    int err = adc_raw_to_millivolts_dt(data->adc_chan, &mv);
    if (err < 0) {
      LOG_WRN("mv conversion failed: %d", err);
      continue;
    }

    /* 2. Low-pass filter */
    data->filtered_mv = lpf(mv, data->config->lpf_size, data->lpf_buf,
                            &data->lpf_idx, &data->lpf_sum);

    /* 3. Peak detection → BPM */
    detect_bpm(mv, data->config, &data->above_threshold, &data->last_peak_ms,
               &data->bpm);

    /* 4. Build output record and fire callback */
    if (internal->output_cb != NULL) {
      struct xd58c_sample sample = {
          .timestamp_ms = k_uptime_get(),
          .raw_mv = mv,
          .filtered_mv = data->filtered_mv,
          .bpm = data->bpm,
      };
      internal->output_cb(&sample, internal->output_user_data);
    }
  }
}

/*---------------------------------------------------------------------------
 * Public API
 *--------------------------------------------------------------------------*/

int xd58c_init(struct xd58c_data *data, const struct xd58c_config *config,
               const struct adc_dt_spec *chan, struct k_msgq *msgq,
               xd58c_output_cb_t output_cb, void *output_user_data) {
  int err;

  if (data == NULL || config == NULL || chan == NULL || msgq == NULL) {
    return -EINVAL;
  }

  if (config->lpf_size == 0 || config->lpf_size > XD58C_LPF_SIZE_MAX) {
    LOG_ERR("lpf_size must be 1..%d, got %d", XD58C_LPF_SIZE_MAX,
            config->lpf_size);
    return -EINVAL;
  }

  /* Validate ADC */
  if (!adc_is_ready_dt(chan)) {
    LOG_ERR("ADC device not ready");
    return -ENODEV;
  }

  /* Initialise data */
  memset(data, 0, sizeof(*data));
  data->adc_chan = chan;
  data->msgq = msgq;
  data->config = config;
  data->running = false;

  /* Store internal state */
  _internal.data = data;
  _internal.output_cb = output_cb;
  _internal.output_user_data = output_user_data;

  /* Configure ADC channel */
  err = adc_channel_setup_dt(chan);
  if (err < 0) {
    LOG_ERR("ADC channel setup failed: %d", err);
    return err;
  }

  /* Build sequence options — user_data recovers data in callback */
  data->options = (struct adc_sequence_options){
      .interval_us = config->interval_us,
      .extra_samplings = 0,
      .callback = adc_callback,
      .user_data = data,
  };

  /* Build ADC sequence */
  data->sequence = (struct adc_sequence){
      .buffer = &data->buffer,
      .buffer_size = sizeof(data->buffer),
      .options = &data->options,
      .calibrate = false,
  };

  /* Fill channels, resolution, oversampling from DT */
  err = adc_sequence_init_dt(chan, &data->sequence);
  if (err < 0) {
    LOG_ERR("ADC sequence init failed: %d", err);
    return err;
  }

  /* Spawn processing thread */
  data->stack = _proc_stack;
  k_thread_create(&data->thread, data->stack, CONFIG_XD58C_THREAD_STACK_SIZE,
                  processing_thread, &_internal, NULL, NULL,
                  CONFIG_XD58C_THREAD_PRIORITY, 0, K_NO_WAIT);

  k_thread_name_set(&data->thread, "xd58c_proc");

  LOG_INF("XD-58C initialised — %uHz threshold=%dmV lpf=%d",
          1000000U / config->interval_us, (int)config->peak_threshold_mv,
          config->lpf_size);

  return 0;
}

int xd58c_start(struct xd58c_data *data) {
  if (data == NULL) {
    return -EINVAL;
  }

  data->running = true;

  int err = adc_read_async(data->adc_chan->dev, &data->sequence, NULL);
  if (err < 0) {
    LOG_ERR("ADC async read failed: %d", err);
    data->running = false;
    return err;
  }

  LOG_INF("XD-58C sampling started");
  return 0;
}

int xd58c_stop(struct xd58c_data *data) {
  if (data == NULL) {
    return -EINVAL;
  }

  data->running = false;
  LOG_INF("XD-58C sampling stopped");
  return 0;
}

int xd58c_get_bpm(const struct xd58c_data *data, int32_t *bpm) {
  if (data == NULL || bpm == NULL) {
    return -EINVAL;
  }

  *bpm = data->bpm;
  return 0;
}

int xd58c_get_mv(const struct xd58c_data *data, int32_t *mv) {
  if (data == NULL || mv == NULL) {
    return -EINVAL;
  }

  *mv = data->filtered_mv;
  return 0;
}