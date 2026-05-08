#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/sensor/sensor.h"

#include <SPI.h>
#include <vector>

namespace esphome {
namespace bresser_weather {

class BresserWeather;

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

struct BresserFrame {
  uint8_t data[27];
  uint8_t length;
  int16_t rssi_dbm;
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
  void set_frequency_hz(uint32_t hz) { frequency_hz_ = hz; }
  void set_log_unknown(bool v) { log_unknown_ = v; }

  void register_sensor(BresserWeatherSensor *s) { sensors_.push_back(s); }

 protected:
  void cc1101_select_();
  void cc1101_deselect_();
  bool cc1101_wait_miso_low_();
  void cc1101_write_reg_(uint8_t addr, uint8_t value);
  uint8_t cc1101_read_reg_(uint8_t addr);
  uint8_t cc1101_read_status_(uint8_t addr);
  void cc1101_strobe_(uint8_t cmd);
  void cc1101_read_burst_(uint8_t addr, uint8_t *buf, uint8_t len);
  void cc1101_write_burst_(uint8_t addr, const uint8_t *buf, uint8_t len);
  void cc1101_reset_();
  bool cc1101_init_();
  void cc1101_program_freq_(uint32_t hz);
  void cc1101_enter_rx_();
  void cc1101_flush_rx_();

  bool process_frame_(const BresserFrame &frame);
  bool decode_bresser_7in1_(uint8_t *msg, uint8_t length, int16_t rssi_dbm);
  void publish_(float temperature, float humidity, float wind_speed_kmh,
                float wind_dir_deg, float rain_mm, float uv_index,
                float light_lux, float pressure_hpa, int16_t rssi_dbm);

  static int16_t rssi_raw_to_dbm_(uint8_t raw);
  static uint16_t lfsr_digest16_(const uint8_t *message, unsigned bytes,
                                 uint16_t gen, uint16_t key);

  int mosi_pin_{-1};
  int miso_pin_{-1};
  int clk_pin_{-1};
  int cs_pin_{-1};
  int gdo0_pin_{-1};
  int gdo2_pin_{-1};

  uint32_t frequency_hz_{868300000};
  bool log_unknown_{false};
  bool radio_ready_{false};

  SPIClass *spi_{nullptr};
  SPISettings spi_settings_{4000000, MSBFIRST, SPI_MODE0};

  std::vector<BresserWeatherSensor *> sensors_;
  uint32_t last_packet_ms_{0};
};

}  // namespace bresser_weather
}  // namespace esphome
