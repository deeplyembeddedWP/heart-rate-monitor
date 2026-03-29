/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef XD58C_H_
#define XD58C_H_

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief XD-58C optical pulse sensor driver
 * @defgroup xd58c XD-58C Pulse Sensor
 * @{
 */

/*---------------------------------------------------------------------------
 * Constants
 *--------------------------------------------------------------------------*/

/** Maximum LPF buffer size — lpf_size in config must not exceed this */
#define XD58C_LPF_SIZE_MAX 16

/*---------------------------------------------------------------------------
 * Output record — one per sample, passed to output callback
 *--------------------------------------------------------------------------*/

/**
 * @brief Real-time output record streamed once per ADC sample (100Hz)
 *
 * Passed to the user-supplied output callback from the processing thread.
 * All fields are valid on every call — bpm is 0 until the first valid
 * peak pair is detected.
 */
struct xd58c_sample {
  /** Timestamp in milliseconds since boot (from k_uptime_get()) */
  int64_t timestamp_ms;

  /** Raw ADC reading converted to millivolts (before LPF) */
  int32_t raw_mv;

  /** LPF-smoothed millivolt reading (used for peak detection) */
  int32_t filtered_mv;

  /** Most recently calculated BPM (0 if not yet detected) */
  int32_t bpm;
};

/*---------------------------------------------------------------------------
 * Output callback type
 *--------------------------------------------------------------------------*/

/**
 * @brief Real-time output callback — called from processing thread
 *
 * Invoked once per sample (at interval_us rate) from the processing
 * thread context. Safe to call LOG_INF, k_msgq_put, etc.
 * Must NOT block indefinitely.
 *
 * @param sample    Pointer to output record — valid only during this call
 * @param user_data User data pointer passed to xd58c_init()
 */
typedef void (*xd58c_output_cb_t)(const struct xd58c_sample *sample,
                                  void *user_data);

/*---------------------------------------------------------------------------
 * Configuration — const, set once at init, lives in flash
 *--------------------------------------------------------------------------*/

/**
 * @brief XD-58C driver configuration
 *
 * Pass a pointer to xd58c_init(). All fields must be set before calling
 * init. Use XD58C_DEFAULT_CONFIG as a starting point.
 */
struct xd58c_config {
  /**
   * ADC sampling interval in microseconds.
   * Default: 10000 (10ms = 100Hz)
   * Must match a valid nRF SAADC interval supported by the hardware timer.
   */
  uint32_t interval_us;

  /**
   * Peak detection threshold in millivolts.
   * Default: 1650mV (midpoint of 0-3300mV range)
   * Tune this based on the actual signal amplitude from the sensor.
   * A rising edge crossing this threshold counts as a heartbeat peak.
   */
  int32_t peak_threshold_mv;

  /**
   * Minimum time between two peaks in milliseconds.
   * Default: 300ms (equivalent to 200 BPM maximum)
   * Peaks detected faster than this are ignored as noise.
   */
  int32_t min_peak_interval_ms;

  /**
   * Maximum time between two peaks in milliseconds.
   * Default: 1500ms (equivalent to 40 BPM minimum)
   * Intervals longer than this reset the peak detector.
   */
  int32_t max_peak_interval_ms;

  /**
   * Moving average low-pass filter size.
   * Default: 5
   * Cutoff frequency = f_sample / (2 * lpf_size)
   * At 100Hz with lpf_size=5: cutoff = 10Hz
   * Must be in range [1, XD58C_LPF_SIZE_MAX].
   */
  uint8_t lpf_size;

  int32_t finger_present_threshold_mv;
};

/**
 * @brief Default configuration for the XD-58C driver
 *
 * Suitable for most use cases with the nRF52840 DK.
 * Copy and modify fields as needed before passing to xd58c_init().
 *
 * Example:
 * @code
 * static const struct xd58c_config config = XD58C_DEFAULT_CONFIG;
 * @endcode
 */
#define XD58C_DEFAULT_CONFIG                                                   \
  {                                                                            \
      .interval_us = 10000,                                                    \
      .peak_threshold_mv = 1700,                                               \
      .min_peak_interval_ms = 300,                                             \
      .max_peak_interval_ms = 1500,                                            \
      .lpf_size = 10,                                                          \
      .finger_present_threshold_mv = 1600,                                     \
  }

/*---------------------------------------------------------------------------
 * Driver data — mutable, lives in RAM, one instance per sensor
 *--------------------------------------------------------------------------*/

/**
 * @brief XD-58C driver runtime data
 *
 * Declare one instance per sensor. Initialised by xd58c_init().
 * Do not access fields directly — use the public API functions.
 */
struct xd58c_data {
  /** ADC channel spec from devicetree — set by xd58c_init() */
  const struct adc_dt_spec *adc_chan;

  /** Driver config — set by xd58c_init() */
  const struct xd58c_config *config;

  /** Single sample buffer — oversampling averaging done in HW */
  int16_t buffer;

  /** ADC sequence options — interval, callback, user_data */
  struct adc_sequence_options options;

  /** ADC sequence — buffer, resolution, channels, oversampling */
  struct adc_sequence sequence;

  /** Message queue — ADC ISR hands samples to processing thread */
  struct k_msgq *msgq;

  /** LPF state — circular buffer */
  int32_t lpf_buf[XD58C_LPF_SIZE_MAX];

  /** LPF state — current write index */
  int lpf_idx;

  /** LPF state — running sum for efficient average */
  int32_t lpf_sum;

  /** Peak detector state — true when signal is above threshold */
  bool above_threshold;

  /** Peak detector state — timestamp of last detected peak */
  int64_t last_peak_ms;

  /** Set true by xd58c_start(), false by xd58c_stop() */
  bool running;

  /** Most recently calculated BPM (0 until first valid detection) */
  int32_t bpm;

  /** Most recently LPF-smoothed millivolt reading */
  int32_t filtered_mv;

  /** Processing thread control block */
  struct k_thread thread;

  /** Processing thread stack pointer */
  k_thread_stack_t *stack;
};

/*---------------------------------------------------------------------------
 * Public API
 *--------------------------------------------------------------------------*/

/**
 * @brief Initialise the XD-58C driver
 *
 * Validates the ADC device, configures the ADC channel from the
 * devicetree spec, initialises the ADC sequence and internal state,
 * and spawns the processing thread.
 *
 * Must be called before xd58c_start(). May be called from main()
 * before the kernel scheduler is fully running.
 *
 * @param data             Pointer to driver data struct (caller allocated)
 * @param config           Pointer to driver config struct
 * @param chan             Pointer to ADC channel spec from devicetree
 * @param msgq             Pointer to message queue for ISR→thread handoff
 * @param output_cb        Callback invoked once per sample with results.
 *                         Pass NULL to disable real-time output.
 * @param output_user_data Passed as-is to output_cb. May be NULL.
 *
 * @retval 0        Success
 * @retval -EINVAL  NULL argument or invalid config field
 * @retval -ENODEV  ADC device not ready
 * @retval <0       ADC channel or sequence setup error
 */
int xd58c_init(struct xd58c_data *data, const struct xd58c_config *config,
               const struct adc_dt_spec *chan, struct k_msgq *msgq,
               xd58c_output_cb_t output_cb, void *output_user_data);

/**
 * @brief Start continuous async ADC sampling
 *
 * Kicks off the ADC peripheral. From this point the ADC callback
 * fires every interval_us microseconds, feeding samples into the
 * message queue which the processing thread consumes.
 *
 * Must be called after xd58c_init().
 *
 * @param data Pointer to driver data struct
 *
 * @retval 0        Success
 * @retval -EINVAL  NULL argument
 * @retval <0       ADC read error
 */
int xd58c_start(struct xd58c_data *data);

/**
 * @brief Stop continuous async ADC sampling
 *
 * Sets the running flag to false. The ADC callback returns
 * ADC_ACTION_FINISH on the next fire, cleanly stopping the peripheral.
 * The processing thread continues to drain any queued samples then
 * blocks waiting for new ones.
 *
 * @param data Pointer to driver data struct
 *
 * @retval 0        Success
 * @retval -EINVAL  NULL argument
 */
int xd58c_stop(struct xd58c_data *data);

/**
 * @brief Get the most recently calculated BPM
 *
 * Thread-safe read of the BPM value updated by the processing thread.
 * Returns 0 if no valid heartbeat has been detected yet.
 *
 * @param data Pointer to driver data struct
 * @param bpm  Output — set to current BPM value
 *
 * @retval 0        Success
 * @retval -EINVAL  NULL argument
 */
int xd58c_get_bpm(const struct xd58c_data *data, int32_t *bpm);

/**
 * @brief Get the most recently LPF-smoothed millivolt reading
 *
 * Useful for polling the current waveform value outside the output
 * callback — for example from a display update loop.
 *
 * @param data Pointer to driver data struct
 * @param mv   Output — set to filtered millivolt value
 *
 * @retval 0        Success
 * @retval -EINVAL  NULL argument
 */
int xd58c_get_mv(const struct xd58c_data *data, int32_t *mv);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* XD58C_H_ */