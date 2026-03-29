/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <app_version.h>
#include <lib/xd58c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/*---------------------------------------------------------------------------
 * ADC channel from devicetree
 *--------------------------------------------------------------------------*/
static const struct adc_dt_spec adc_chan =
    ADC_DT_SPEC_GET(DT_PATH(zephyr_user));

/*---------------------------------------------------------------------------
 * Message queue — ADC ISR → processing thread
 *--------------------------------------------------------------------------*/
K_MSGQ_DEFINE(pulse_msgq, sizeof(int16_t), 20, 2);

/*---------------------------------------------------------------------------
 * Driver instance
 *--------------------------------------------------------------------------*/
static struct xd58c_data sensor_data;
static const struct xd58c_config sensor_config = XD58C_DEFAULT_CONFIG;

/*---------------------------------------------------------------------------
 * Real-time output callback
 *
 * Called from processing thread for every sample — 100 times per second.
 * Format: "ts=<ms> raw=<mV> filt=<mV> bpm=<bpm>"
 * Parse this on the host PC to plot the waveform and BPM in real time.
 *--------------------------------------------------------------------------*/
static void on_sample(const struct xd58c_sample *sample, void *user_data) {
  ARG_UNUSED(user_data);

  /* Tab-separated for easy Python parsing */
  LOG_INF("%lld\t%d\t%d\t%d", sample->timestamp_ms, (int)sample->raw_mv,
          (int)sample->filtered_mv, (int)sample->bpm);
}

/*---------------------------------------------------------------------------
 * Main
 *--------------------------------------------------------------------------*/
int main(void) {
  int err;

  LOG_INF("Heart Rate Monitor %s", APP_VERSION_STRING);

  err = xd58c_init(&sensor_data, &sensor_config, &adc_chan, &pulse_msgq,
                   on_sample, NULL);
  if (err < 0) {
    LOG_ERR("XD-58C init failed: %d", err);
    return err;
  }

  err = xd58c_start(&sensor_data);
  if (err < 0) {
    LOG_ERR("XD-58C start failed: %d", err);
    return err;
  }

  /* Processing thread drives everything — main has nothing left to do */
  return 0;
}