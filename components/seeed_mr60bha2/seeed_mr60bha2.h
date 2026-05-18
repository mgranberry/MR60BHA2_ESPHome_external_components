#pragma once
#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif
#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif
#include "esphome/components/uart/uart.h"
#include "esphome/core/automation.h"
#include "esphome/core/helpers.h"

#include <cstring>

namespace esphome {
namespace seeed_mr60bha2 {
static const uint8_t FRAME_SOF = 0x01;
static const uint16_t BREATH_RATE_TYPE_BUFFER = 0x0A14;
static const uint16_t PEOPLE_EXIST_TYPE_BUFFER = 0x0F09;
static const uint16_t HEART_RATE_TYPE_BUFFER = 0x0A15;
static const uint16_t DISTANCE_TYPE_BUFFER = 0x0A16;
static const uint16_t PRINT_CLOUD_BUFFER = 0x0A04;

// Frame structure: SOF(1) + ID(2) + LEN(2) + TYPE(2) + HEAD_CKSUM(1) + DATA(LEN) + DATA_CKSUM(1)
static const size_t FRAME_HEADER_SIZE = 8;        // SOF + ID + LEN + TYPE + HEAD_CKSUM
static const size_t FRAME_MAX_DATA_LENGTH = 64;   // Maximum expected data payload
static const size_t FRAME_BUFFER_SIZE = 256;      // Ring buffer capacity

class MR60BHA2Component : public Component,
                          public uart::UARTDevice {
#ifdef USE_BINARY_SENSOR
  SUB_BINARY_SENSOR(has_target);
#endif
#ifdef USE_SENSOR
  SUB_SENSOR(breath_rate);
  SUB_SENSOR(heart_rate);
  SUB_SENSOR(distance);
  SUB_SENSOR(num_targets);
#endif
#ifdef USE_TEXT_SENSOR
  SUB_TEXT_SENSOR(version);
#endif

 public:
  float get_setup_priority() const override { return esphome::setup_priority::LATE; }
  void setup() override;
  void dump_config() override;
  void loop() override;

 protected:
  void process_frame_(uint16_t frame_id, uint16_t frame_type, const uint8_t *data, size_t length);
  bool parse_frame_();
  void discard_until_sof_();

  void check_staleness_();

  uint8_t rx_buf_[FRAME_BUFFER_SIZE];
  size_t rx_count_{0};

  static const uint32_t STALE_TIMEOUT_MS = 30000;
  uint32_t last_breath_rate_ms_{0};
  uint32_t last_heart_rate_ms_{0};
  uint32_t last_distance_ms_{0};
  uint32_t last_num_targets_ms_{0};
  uint32_t last_has_target_ms_{0};
};

}  // namespace seeed_mr60bha2
}  // namespace esphome
