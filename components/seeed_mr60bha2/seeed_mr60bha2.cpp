#include "seeed_mr60bha2.h"
#include "esphome/core/log.h"

#include <cinttypes>
#include <cmath>
#include <utility>

namespace esphome {
namespace seeed_mr60bha2 {

static const char *const TAG = "seeed_mr60bha2";

#ifndef MR60BHA2_VERSION
#define MR60BHA2_VERSION "unknown"
#endif

void MR60BHA2Component::setup() {
#ifdef USE_TEXT_SENSOR
  if (this->version_text_sensor_ != nullptr) {
    this->version_text_sensor_->publish_state(MR60BHA2_VERSION);
  }
#endif
}

// Prints the component's configuration data. dump_config() prints all of the component's configuration
// items in an easy-to-read format, including the configuration key-value pairs.
void MR60BHA2Component::dump_config() {
  ESP_LOGCONFIG(TAG, "MR60BHA2:");
#ifdef USE_BINARY_SENSOR
  LOG_BINARY_SENSOR(" ", "People Exist Binary Sensor", this->has_target_binary_sensor_);
#endif
#ifdef USE_SENSOR
  LOG_SENSOR(" ", "Breath Rate Sensor", this->breath_rate_sensor_);
  LOG_SENSOR(" ", "Heart Rate Sensor", this->heart_rate_sensor_);
  LOG_SENSOR(" ", "Distance Sensor", this->distance_sensor_);
  LOG_SENSOR(" ", "Target Number Sensor", this->num_targets_sensor_);
#endif
}

static uint8_t calculate_checksum(const uint8_t *data, size_t len) {
  uint8_t checksum = 0;
  for (size_t i = 0; i < len; i++) {
    checksum ^= data[i];
  }
  return ~checksum;
}

void MR60BHA2Component::discard_until_sof_() {
  size_t shift = 1;
  while (shift < this->rx_count_ && this->rx_buf_[shift] != FRAME_SOF) {
    shift++;
  }
  if (shift < this->rx_count_) {
    this->rx_count_ -= shift;
    memmove(this->rx_buf_, this->rx_buf_ + shift, this->rx_count_);
  } else {
    this->rx_count_ = 0;
  }
}

void MR60BHA2Component::loop() {
  this->check_staleness_();

  size_t available = this->available();
  if (available == 0)
    return;

  size_t space = FRAME_BUFFER_SIZE - this->rx_count_;
  if (space == 0) {
    ESP_LOGW(TAG, "RX buffer full, resetting");
    this->rx_count_ = 0;
    space = FRAME_BUFFER_SIZE;
  }

  size_t to_read = std::min(available, space);
  this->read_array(this->rx_buf_ + this->rx_count_, to_read);
  this->rx_count_ += to_read;

  while (this->rx_count_ > 0) {
    if (this->rx_buf_[0] != FRAME_SOF) {
      this->discard_until_sof_();
      continue;
    }
    if (!this->parse_frame_())
      break;
  }
}

bool MR60BHA2Component::parse_frame_() {
  uint8_t *data = this->rx_buf_;
  size_t count = this->rx_count_;

  if (count < FRAME_HEADER_SIZE)
    return false;

  if (calculate_checksum(data, 7) != data[7]) {
    ESP_LOGD(TAG, "Header checksum failed, resyncing");
    this->discard_until_sof_();
    return true;
  }

  uint16_t length = encode_uint16(data[3], data[4]);
  if (length > FRAME_MAX_DATA_LENGTH) {
    ESP_LOGD(TAG, "Frame length %u exceeds max, resyncing", length);
    this->discard_until_sof_();
    return true;
  }

  size_t total_frame_size = FRAME_HEADER_SIZE + length + 1;
  if (count < total_frame_size)
    return false;

  if (calculate_checksum(data + FRAME_HEADER_SIZE, length) != data[FRAME_HEADER_SIZE + length]) {
    ESP_LOGD(TAG, "Data checksum failed, resyncing");
    this->discard_until_sof_();
    return true;
  }

  uint16_t frame_id = encode_uint16(data[1], data[2]);
  uint16_t frame_type = encode_uint16(data[5], data[6]);

  ESP_LOGV(TAG, "Frame OK: ID=0x%04x Type=0x%04x Len=%u", frame_id, frame_type, length);
  this->process_frame_(frame_id, frame_type, data + FRAME_HEADER_SIZE, length);

  this->rx_count_ -= total_frame_size;
  if (this->rx_count_ > 0) {
    memmove(this->rx_buf_, this->rx_buf_ + total_frame_size, this->rx_count_);
  }
  return true;
}

void MR60BHA2Component::process_frame_(uint16_t frame_id, uint16_t frame_type, const uint8_t *data, size_t length) {
  if (this->has_target_binary_sensor_ != nullptr && !this->has_target_binary_sensor_->state &&
      frame_type != PEOPLE_EXIST_TYPE_BUFFER) {
    // Do not process other frames while people exists sensor is still false
    return;
  }
  switch (frame_type) {
    case BREATH_RATE_TYPE_BUFFER:
      if (this->breath_rate_sensor_ != nullptr && length >= 4) {
        this->last_breath_rate_ms_ = millis();
        uint32_t current_breath_rate_int = encode_uint32(data[3], data[2], data[1], data[0]);
        if (current_breath_rate_int != 0) {
          float breath_rate_float;
          memcpy(&breath_rate_float, &current_breath_rate_int, sizeof(float));
          if (this->breath_rate_sensor_->state != breath_rate_float) {
            this->breath_rate_sensor_->publish_state(breath_rate_float);
          }
        }
      }
      break;
    case PEOPLE_EXIST_TYPE_BUFFER:
      if (length >= 2) {
        uint16_t has_target_int = encode_uint16(data[1], data[0]);
        this->last_has_target_ms_ = millis();
        bool has_target_bool = has_target_int != 0;
        if (this->has_target_binary_sensor_ != nullptr &&
            this->has_target_binary_sensor_->state != has_target_bool) {
          this->has_target_binary_sensor_->publish_state(has_target_bool);
        }
        if (!has_target_bool) {
          if (this->breath_rate_sensor_ != nullptr && !std::isnan(this->breath_rate_sensor_->state)) {
            this->breath_rate_sensor_->publish_state(NAN);
          }
          if (this->heart_rate_sensor_ != nullptr && !std::isnan(this->heart_rate_sensor_->state)) {
            this->heart_rate_sensor_->publish_state(NAN);
          }
          if (this->distance_sensor_ != nullptr && !std::isnan(this->distance_sensor_->state)) {
            this->distance_sensor_->publish_state(NAN);
          }
          if (this->num_targets_sensor_ != nullptr && !std::isnan(this->num_targets_sensor_->state)) {
            this->num_targets_sensor_->publish_state(NAN);
          }
        }
      }
      break;
    case HEART_RATE_TYPE_BUFFER:
      if (this->heart_rate_sensor_ != nullptr && length >= 4) {
        this->last_heart_rate_ms_ = millis();
        uint32_t current_heart_rate_int = encode_uint32(data[3], data[2], data[1], data[0]);
        if (current_heart_rate_int != 0) {
          float heart_rate_float;
          memcpy(&heart_rate_float, &current_heart_rate_int, sizeof(float));
          if (this->heart_rate_sensor_->state != heart_rate_float) {
            this->heart_rate_sensor_->publish_state(heart_rate_float);
          }
        }
      }
      break;
    case DISTANCE_TYPE_BUFFER:
      if (this->distance_sensor_ != nullptr && length >= 8) {
        this->last_distance_ms_ = millis();
        uint32_t flag = encode_uint32(data[3], data[2], data[1], data[0]);
        if (flag == 0) {
          if (!std::isnan(this->distance_sensor_->state)) {
            this->distance_sensor_->publish_state(NAN);
          }
        } else {
          uint32_t current_distance_int = encode_uint32(data[7], data[6], data[5], data[4]);
          float distance_float;
          memcpy(&distance_float, &current_distance_int, sizeof(float));
          if (this->distance_sensor_->state != distance_float) {
            this->distance_sensor_->publish_state(distance_float);
          }
        }
      }
      break;
    case PRINT_CLOUD_BUFFER:
      if (this->num_targets_sensor_ != nullptr && length >= 4) {
        uint32_t current_num_targets_int = encode_uint32(data[3], data[2], data[1], data[0]);
        this->last_num_targets_ms_ = millis();
        if (static_cast<uint32_t>(this->num_targets_sensor_->state) != current_num_targets_int) {
          this->num_targets_sensor_->publish_state(current_num_targets_int);
        }
      }
      break;
    default:
      break;
  }
}

void MR60BHA2Component::check_staleness_() {
  uint32_t now = millis();
  if (this->breath_rate_sensor_ != nullptr && this->last_breath_rate_ms_ != 0 &&
      now - this->last_breath_rate_ms_ > STALE_TIMEOUT_MS && !std::isnan(this->breath_rate_sensor_->state)) {
    this->breath_rate_sensor_->publish_state(NAN);
  }
  if (this->heart_rate_sensor_ != nullptr && this->last_heart_rate_ms_ != 0 &&
      now - this->last_heart_rate_ms_ > STALE_TIMEOUT_MS && !std::isnan(this->heart_rate_sensor_->state)) {
    this->heart_rate_sensor_->publish_state(NAN);
  }
  if (this->distance_sensor_ != nullptr && this->last_distance_ms_ != 0 &&
      now - this->last_distance_ms_ > STALE_TIMEOUT_MS && !std::isnan(this->distance_sensor_->state)) {
    this->distance_sensor_->publish_state(NAN);
  }
  if (this->num_targets_sensor_ != nullptr && this->last_num_targets_ms_ != 0 &&
      now - this->last_num_targets_ms_ > STALE_TIMEOUT_MS && !std::isnan(this->num_targets_sensor_->state)) {
    this->num_targets_sensor_->publish_state(NAN);
  }
#ifdef USE_BINARY_SENSOR
  if (this->has_target_binary_sensor_ != nullptr && this->last_has_target_ms_ != 0 &&
      now - this->last_has_target_ms_ > STALE_TIMEOUT_MS && this->has_target_binary_sensor_->state) {
    this->has_target_binary_sensor_->publish_state(false);
  }
#endif
}

}  // namespace seeed_mr60bha2
}  // namespace esphome
