#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/sensor/sensor.h"

#include <SPI.h>
#include <vector>
#include <string>

namespace esphome {
namespace bresser_weather {

class BresserWeather;

struct RadioPreset {
  const char *name;
  uint32_t freq_hz;
  float bitrate_kbps;
  float deviation_khz;
  float rxbw_khz;
  bool sync_enabled;
  uint16_t sync_word;
  uint8_t pkt_len;
};

class BresserWeatherSensor : public Component {
 public:
  void setup() override {}
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_temperature_sensor(sensor::Sensor *s) { temperature_ = s; }
  void set_humidity_sensor(sensor::Sensor *s) { humidity_ = s; }
  void set_wind_speed_sensor(sensor::Sensor *s) { wind_speed_ = s; }
  void set_wind_direction_sensor(sensor::Sensor *s) { wind_direction_ = s; }
  void set_rain_total_sensor(sensor::Sensor *s) { rain_total_ = s; }
  void set_uv_index_sensor(sensor::Sensor *s) { uv_index_ = s; }
  void set_light_lux_sensor(sensor::Sensor *s) { light_lux_ = s; }
  void set_pressure_sensor(sensor::Sensor *s) { pressure_ = s; }
  void set_rssi_sensor(sensor::Sensor *s) { rssi_ = s; }

  sensor::Sensor *temperature_{nullptr};
  sensor::Sensor *humidity_{nullptr};
  sensor::Sensor *wind_speed_{nullptr};
  sensor::Sensor *wind_direction_{nullptr};
  sensor::Sensor *rain_total_{nullptr};
  sensor::Sensor *uv_index_{nullptr};
  sensor::Sensor *light_lux_{nullptr};
  sensor::Sensor *pressure_{nullptr};
  sensor::Sensor *rssi_{nullptr};
};

class BresserWeather : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::BUS; }

  void set_pins(int mosi, int miso, int clk, int cs, int gdo0, int gdo2) {
    mosi_pin_ = mosi;
    miso_pin_ = miso;
    clk_pin_ = clk;
    cs_pin_ = cs;
    gdo0_pin_ = gdo0;
    gdo2_pin_ = gdo2;
  }
  void set_frequency_hz(uint32_t hz) { configured_freq_hz_ = hz; }
  void set_log_unknown(bool v) { log_unknown_ = v; }
  void set_scan_mode(bool v) { scan_mode_ = v; }
  void set_scan_interval_ms(uint32_t v) { scan_interval_ms_ = v; }
  void set_status_interval_ms(uint32_t v) { status_interval_ms_ = v; }
  void set_raw_dump_topic(const std::string &t) { raw_dump_topic_ = t; }

  void register_sensor(BresserWeatherSensor *s) { sensors_.push_back(s); }

  static constexpr size_t PRESET_COUNT = 8;
  static const RadioPreset PRESETS[PRESET_COUNT];

 protected:
  // ---- SPI / CC1101 low-level ----
  void cc1101_select_();
  void cc1101_deselect_();
  bool cc1101_wait_miso_low_();
  void cc1101_write_reg_(uint8_t addr, uint8_t value);
  bool cc1101_write_verify_(uint8_t addr, uint8_t value, const char *name);
  uint8_t cc1101_read_reg_(uint8_t addr);
  uint8_t cc1101_read_status_(uint8_t addr);
  void cc1101_strobe_(uint8_t cmd);
  void cc1101_read_burst_(uint8_t addr, uint8_t *buf, uint8_t len);
  void cc1101_write_burst_(uint8_t addr, const uint8_t *buf, uint8_t len);
  void cc1101_reset_();
  bool cc1101_probe_();
  void cc1101_enter_rx_();
  void cc1101_flush_rx_();

  // ---- High-level radio config ----
  void apply_preset_(const RadioPreset &p);
  void log_register_dump_();

  static void calc_drate_(float kbps, uint8_t &drate_e, uint8_t &drate_m);
  static void calc_dev_(float khz, uint8_t &dev_e, uint8_t &dev_m);
  static uint8_t calc_chanbw_(float khz);

  // ---- Frame handling ----
  void handle_frame_(uint8_t *raw, uint8_t length, int16_t rssi_dbm);
  bool decode_bresser_7in1_(const uint8_t *raw, uint8_t length,
                            int16_t rssi_dbm, bool from_scan);
  void log_packet_diff_(const uint8_t *raw, uint8_t length);
  void publish_raw_(const uint8_t *raw, uint8_t length, int16_t rssi_dbm,
                    int sync_offset_bit);
  void publish_decoded_(float t, float h, float wkmh, float wdir,
                        float rmm, float uv, float lux, float p,
                        int16_t rssi_dbm);

  static int16_t rssi_raw_to_dbm_(uint8_t raw);
  static uint16_t lfsr_digest16_(const uint8_t *message, unsigned bytes,
                                 uint16_t gen, uint16_t key);
  static int find_sync_bit_(const uint8_t *raw, uint8_t length,
                            uint16_t sync_word);

  // ---- State ----
  int mosi_pin_{-1};
  int miso_pin_{-1};
  int clk_pin_{-1};
  int cs_pin_{-1};
  int gdo0_pin_{-1};
  int gdo2_pin_{-1};

  uint32_t configured_freq_hz_{868300000};
  bool log_unknown_{true};
  bool scan_mode_{false};
  uint32_t scan_interval_ms_{30000};
  uint32_t status_interval_ms_{10000};
  std::string raw_dump_topic_{""};

  bool radio_ready_{false};
  SPIClass *spi_{nullptr};
  // Default 1 MHz — well within CC1101 datasheet limits, far more tolerant
  // of long jumper wires and breadboard parasitics than 4 MHz.
  SPISettings spi_settings_{1000000, MSBFIRST, SPI_MODE0};

  std::vector<BresserWeatherSensor *> sensors_;

  size_t current_preset_idx_{0};
  uint32_t scan_started_ms_{0};
  uint32_t total_packets_{0};
  uint32_t valid_packets_{0};
  uint32_t last_packet_ms_{0};
  uint32_t last_status_log_ms_{0};

  uint8_t prev_payload_[64]{};
  uint8_t prev_payload_len_{0};
};

}  // namespace bresser_weather
}  // namespace esphome
