#include "bresser_weather.h"

#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace bresser_weather {

static const char *const TAG = "bresser_weather";

// ---------------------------------------------------------------------------
// CC1101 register addresses (subset used by the driver)
// ---------------------------------------------------------------------------
static constexpr uint8_t CC1101_IOCFG2 = 0x00;
static constexpr uint8_t CC1101_IOCFG0 = 0x02;
static constexpr uint8_t CC1101_FIFOTHR = 0x03;
static constexpr uint8_t CC1101_SYNC1 = 0x04;
static constexpr uint8_t CC1101_SYNC0 = 0x05;
static constexpr uint8_t CC1101_PKTLEN = 0x06;
static constexpr uint8_t CC1101_PKTCTRL1 = 0x07;
static constexpr uint8_t CC1101_PKTCTRL0 = 0x08;
static constexpr uint8_t CC1101_CHANNR = 0x0A;
static constexpr uint8_t CC1101_FSCTRL1 = 0x0B;
static constexpr uint8_t CC1101_FSCTRL0 = 0x0C;
static constexpr uint8_t CC1101_FREQ2 = 0x0D;
static constexpr uint8_t CC1101_FREQ1 = 0x0E;
static constexpr uint8_t CC1101_FREQ0 = 0x0F;
static constexpr uint8_t CC1101_MDMCFG4 = 0x10;
static constexpr uint8_t CC1101_MDMCFG3 = 0x11;
static constexpr uint8_t CC1101_MDMCFG2 = 0x12;
static constexpr uint8_t CC1101_MDMCFG1 = 0x13;
static constexpr uint8_t CC1101_MDMCFG0 = 0x14;
static constexpr uint8_t CC1101_DEVIATN = 0x15;
static constexpr uint8_t CC1101_MCSM2 = 0x16;
static constexpr uint8_t CC1101_MCSM1 = 0x17;
static constexpr uint8_t CC1101_MCSM0 = 0x18;
static constexpr uint8_t CC1101_FOCCFG = 0x19;
static constexpr uint8_t CC1101_BSCFG = 0x1A;
static constexpr uint8_t CC1101_AGCCTRL2 = 0x1B;
static constexpr uint8_t CC1101_AGCCTRL1 = 0x1C;
static constexpr uint8_t CC1101_AGCCTRL0 = 0x1D;
static constexpr uint8_t CC1101_FREND1 = 0x21;
static constexpr uint8_t CC1101_FREND0 = 0x22;
static constexpr uint8_t CC1101_FSCAL3 = 0x23;
static constexpr uint8_t CC1101_FSCAL2 = 0x24;
static constexpr uint8_t CC1101_FSCAL1 = 0x25;
static constexpr uint8_t CC1101_FSCAL0 = 0x26;
static constexpr uint8_t CC1101_TEST2 = 0x2C;
static constexpr uint8_t CC1101_TEST1 = 0x2D;
static constexpr uint8_t CC1101_TEST0 = 0x2E;
static constexpr uint8_t CC1101_PARTNUM = 0x30;
static constexpr uint8_t CC1101_VERSION = 0x31;
static constexpr uint8_t CC1101_RSSI = 0x34;
static constexpr uint8_t CC1101_MARCSTATE = 0x35;
static constexpr uint8_t CC1101_PKTSTATUS = 0x38;
static constexpr uint8_t CC1101_RXBYTES = 0x3B;

static constexpr uint8_t CC1101_PATABLE = 0x3E;
static constexpr uint8_t CC1101_TXFIFO = 0x3F;
static constexpr uint8_t CC1101_RXFIFO = 0x3F;

// Strobes
static constexpr uint8_t CC1101_SRES = 0x30;
static constexpr uint8_t CC1101_SCAL = 0x33;
static constexpr uint8_t CC1101_SRX = 0x34;
static constexpr uint8_t CC1101_SIDLE = 0x36;
static constexpr uint8_t CC1101_SFRX = 0x3A;
static constexpr uint8_t CC1101_SNOP = 0x3D;

// Header bits
static constexpr uint8_t CC1101_READ_SINGLE = 0x80;
static constexpr uint8_t CC1101_READ_BURST = 0xC0;
static constexpr uint8_t CC1101_WRITE_BURST = 0x40;

// Bresser 7-in-1 packet length (bytes after sync word)
static constexpr uint8_t BRESSER_PAYLOAD_LEN = 26;

// ---------------------------------------------------------------------------
// SPI low-level helpers
// ---------------------------------------------------------------------------
void BresserWeather::cc1101_select_() {
  digitalWrite(this->cs_pin_, LOW);
}

void BresserWeather::cc1101_deselect_() {
  digitalWrite(this->cs_pin_, HIGH);
}

bool BresserWeather::cc1101_wait_miso_low_() {
  // CC1101 holds MISO high until the crystal is stable after CS goes low.
  uint32_t start = micros();
  while (digitalRead(this->miso_pin_) == HIGH) {
    if ((uint32_t) (micros() - start) > 5000) {
      return false;
    }
  }
  return true;
}

void BresserWeather::cc1101_write_reg_(uint8_t addr, uint8_t value) {
  this->spi_->beginTransaction(this->spi_settings_);
  this->cc1101_select_();
  this->cc1101_wait_miso_low_();
  this->spi_->transfer(addr);
  this->spi_->transfer(value);
  this->cc1101_deselect_();
  this->spi_->endTransaction();
}

uint8_t BresserWeather::cc1101_read_reg_(uint8_t addr) {
  this->spi_->beginTransaction(this->spi_settings_);
  this->cc1101_select_();
  this->cc1101_wait_miso_low_();
  this->spi_->transfer(addr | CC1101_READ_SINGLE);
  uint8_t v = this->spi_->transfer(0);
  this->cc1101_deselect_();
  this->spi_->endTransaction();
  return v;
}

uint8_t BresserWeather::cc1101_read_status_(uint8_t addr) {
  // Status registers (0x30..0x3D) require burst-read bit set.
  this->spi_->beginTransaction(this->spi_settings_);
  this->cc1101_select_();
  this->cc1101_wait_miso_low_();
  this->spi_->transfer(addr | CC1101_READ_BURST);
  uint8_t v = this->spi_->transfer(0);
  this->cc1101_deselect_();
  this->spi_->endTransaction();
  return v;
}

void BresserWeather::cc1101_strobe_(uint8_t cmd) {
  this->spi_->beginTransaction(this->spi_settings_);
  this->cc1101_select_();
  this->cc1101_wait_miso_low_();
  this->spi_->transfer(cmd);
  this->cc1101_deselect_();
  this->spi_->endTransaction();
}

void BresserWeather::cc1101_read_burst_(uint8_t addr, uint8_t *buf, uint8_t len) {
  this->spi_->beginTransaction(this->spi_settings_);
  this->cc1101_select_();
  this->cc1101_wait_miso_low_();
  this->spi_->transfer(addr | CC1101_READ_BURST);
  for (uint8_t i = 0; i < len; ++i) {
    buf[i] = this->spi_->transfer(0);
  }
  this->cc1101_deselect_();
  this->spi_->endTransaction();
}

void BresserWeather::cc1101_write_burst_(uint8_t addr, const uint8_t *buf, uint8_t len) {
  this->spi_->beginTransaction(this->spi_settings_);
  this->cc1101_select_();
  this->cc1101_wait_miso_low_();
  this->spi_->transfer(addr | CC1101_WRITE_BURST);
  for (uint8_t i = 0; i < len; ++i) {
    this->spi_->transfer(buf[i]);
  }
  this->cc1101_deselect_();
  this->spi_->endTransaction();
}

void BresserWeather::cc1101_reset_() {
  // Manual SRES with the timing required by the datasheet.
  digitalWrite(this->cs_pin_, HIGH);
  delayMicroseconds(5);
  digitalWrite(this->cs_pin_, LOW);
  delayMicroseconds(5);
  digitalWrite(this->cs_pin_, HIGH);
  delayMicroseconds(45);
  this->cc1101_strobe_(CC1101_SRES);
  delay(2);
}

void BresserWeather::cc1101_program_freq_(uint32_t hz) {
  // FREQ = hz * 2^16 / 26 MHz, rounded.
  uint64_t freq_word = ((uint64_t) hz << 16) / 26000000ULL;
  uint8_t f2 = (freq_word >> 16) & 0xFF;
  uint8_t f1 = (freq_word >> 8) & 0xFF;
  uint8_t f0 = freq_word & 0xFF;
  this->cc1101_write_reg_(CC1101_FREQ2, f2);
  this->cc1101_write_reg_(CC1101_FREQ1, f1);
  this->cc1101_write_reg_(CC1101_FREQ0, f0);
  ESP_LOGD(TAG, "FREQ programmed: %u Hz -> %02X %02X %02X", hz, f2, f1, f0);
}

bool BresserWeather::cc1101_init_() {
  this->cc1101_reset_();

  uint8_t part = this->cc1101_read_status_(CC1101_PARTNUM);
  uint8_t version = this->cc1101_read_status_(CC1101_VERSION);
  ESP_LOGI(TAG, "CC1101 PARTNUM=0x%02X VERSION=0x%02X", part, version);
  if (version == 0x00 || version == 0xFF) {
    ESP_LOGE(TAG, "CC1101 not detected on SPI bus");
    return false;
  }

  // GDO2: asserts on sync detect, deasserts at end of packet (IOCFG=0x06).
  this->cc1101_write_reg_(CC1101_IOCFG2, 0x06);
  // GDO0: same (we drive both interrupt and status read off it).
  this->cc1101_write_reg_(CC1101_IOCFG0, 0x06);
  // RX FIFO threshold = 33 bytes (full-payload trigger after sync).
  this->cc1101_write_reg_(CC1101_FIFOTHR, 0x47);

  // Sync word 0x2DD4 (Fine Offset / Bresser).
  this->cc1101_write_reg_(CC1101_SYNC1, 0x2D);
  this->cc1101_write_reg_(CC1101_SYNC0, 0xD4);

  // Fixed packet length: 26 payload bytes captured after sync.
  this->cc1101_write_reg_(CC1101_PKTLEN, BRESSER_PAYLOAD_LEN);
  // No address check, no auto-flush, no status appended.
  this->cc1101_write_reg_(CC1101_PKTCTRL1, 0x00);
  // Fixed length, no whitening, no CRC by hardware (we validate ourselves).
  this->cc1101_write_reg_(CC1101_PKTCTRL0, 0x00);
  this->cc1101_write_reg_(CC1101_CHANNR, 0x00);

  // IF = 152 kHz @ 26 MHz xtal.
  this->cc1101_write_reg_(CC1101_FSCTRL1, 0x06);
  this->cc1101_write_reg_(CC1101_FSCTRL0, 0x00);

  // Frequency programmed from configuration.
  this->cc1101_program_freq_(this->frequency_hz_);

  // Modem: CHANBW=270 kHz (CHANBW_E=1, CHANBW_M=2), DRATE_E=9.
  // DRATE_M = 124 -> 18.868 kbps, the rate Bresser 7-in-1 actually uses.
  this->cc1101_write_reg_(CC1101_MDMCFG4, 0x69);
  this->cc1101_write_reg_(CC1101_MDMCFG3, 0x7C);
  // 2-FSK, no Manchester, 16/16 sync word.
  this->cc1101_write_reg_(CC1101_MDMCFG2, 0x03);
  // 4 preamble bytes, default channel spacing exponent.
  this->cc1101_write_reg_(CC1101_MDMCFG1, 0x22);
  this->cc1101_write_reg_(CC1101_MDMCFG0, 0xF8);
  // Deviation ~41 kHz (DEV_E=4, DEV_M=5) -> close to the spec'd 40 kHz.
  this->cc1101_write_reg_(CC1101_DEVIATN, 0x45);

  // MCSM: stay in RX after a packet, auto-cal on IDLE->RX.
  this->cc1101_write_reg_(CC1101_MCSM2, 0x07);
  this->cc1101_write_reg_(CC1101_MCSM1, 0x3F);
  this->cc1101_write_reg_(CC1101_MCSM0, 0x18);

  // Frequency offset compensation (TI recommended for low data rates).
  this->cc1101_write_reg_(CC1101_FOCCFG, 0x16);
  this->cc1101_write_reg_(CC1101_BSCFG, 0x6C);
  // AGC tuned for low-rate FSK reception.
  this->cc1101_write_reg_(CC1101_AGCCTRL2, 0x43);
  this->cc1101_write_reg_(CC1101_AGCCTRL1, 0x40);
  this->cc1101_write_reg_(CC1101_AGCCTRL0, 0x91);

  this->cc1101_write_reg_(CC1101_FREND1, 0x56);
  this->cc1101_write_reg_(CC1101_FREND0, 0x10);
  this->cc1101_write_reg_(CC1101_FSCAL3, 0xE9);
  this->cc1101_write_reg_(CC1101_FSCAL2, 0x2A);
  this->cc1101_write_reg_(CC1101_FSCAL1, 0x00);
  this->cc1101_write_reg_(CC1101_FSCAL0, 0x1F);
  this->cc1101_write_reg_(CC1101_TEST2, 0x81);
  this->cc1101_write_reg_(CC1101_TEST1, 0x35);
  this->cc1101_write_reg_(CC1101_TEST0, 0x09);

  // Receive-only profile - PATABLE not strictly required.
  uint8_t patable[] = {0xC0};
  this->cc1101_write_burst_(CC1101_PATABLE, patable, sizeof(patable));

  return true;
}

void BresserWeather::cc1101_flush_rx_() {
  this->cc1101_strobe_(CC1101_SIDLE);
  delayMicroseconds(100);
  this->cc1101_strobe_(CC1101_SFRX);
  delayMicroseconds(100);
}

void BresserWeather::cc1101_enter_rx_() {
  this->cc1101_flush_rx_();
  this->cc1101_strobe_(CC1101_SRX);
}

// ---------------------------------------------------------------------------
// Component lifecycle
// ---------------------------------------------------------------------------
void BresserWeather::setup() {
  ESP_LOGI(TAG, "Setting up bresser_weather (CC1101 SPI driver)...");
  pinMode(this->cs_pin_, OUTPUT);
  digitalWrite(this->cs_pin_, HIGH);
  pinMode(this->gdo0_pin_, INPUT);
  if (this->gdo2_pin_ >= 0) {
    pinMode(this->gdo2_pin_, INPUT);
  }

  this->spi_ = new SPIClass(VSPI);
  this->spi_->begin(this->clk_pin_, this->miso_pin_, this->mosi_pin_, this->cs_pin_);
  // ESPHome owns the CS pin via digitalWrite; tell the bus library not to touch it.
  this->spi_->setHwCs(false);

  delay(10);

  this->radio_ready_ = this->cc1101_init_();
  if (!this->radio_ready_) {
    this->mark_failed();
    return;
  }
  this->cc1101_enter_rx_();
  ESP_LOGI(TAG, "CC1101 ready, listening at %u Hz", this->frequency_hz_);
}

void BresserWeather::loop() {
  if (!this->radio_ready_) {
    return;
  }

  // GDO0 with IOCFG=0x06 is high while RX is in flight from sync detect to
  // packet end. We poll for the falling edge by watching MARCSTATE/RXBYTES.
  uint8_t rxbytes = this->cc1101_read_status_(CC1101_RXBYTES);
  bool overflow = (rxbytes & 0x80) != 0;
  uint8_t available = rxbytes & 0x7F;

  if (overflow) {
    ESP_LOGW(TAG, "CC1101 RX FIFO overflow, flushing");
    this->cc1101_enter_rx_();
    return;
  }

  if (available < BRESSER_PAYLOAD_LEN) {
    return;
  }

  BresserFrame frame{};
  frame.length = BRESSER_PAYLOAD_LEN;

  this->cc1101_read_burst_(CC1101_RXFIFO, frame.data, BRESSER_PAYLOAD_LEN);
  uint8_t rssi_raw = this->cc1101_read_status_(CC1101_RSSI);
  frame.rssi_dbm = BresserWeather::rssi_raw_to_dbm_(rssi_raw);

  this->cc1101_enter_rx_();

  uint32_t now = millis();
  if (now - this->last_packet_ms_ < 250) {
    // Same transmission burst (Bresser sends the frame multiple times).
    return;
  }
  this->last_packet_ms_ = now;

  ESP_LOGD(TAG, "Got %u-byte frame, RSSI=%d dBm", frame.length, frame.rssi_dbm);
  if (!this->process_frame_(frame) && this->log_unknown_) {
    char buf[3 * 27 + 1] = {0};
    for (uint8_t i = 0; i < frame.length; ++i) {
      snprintf(buf + i * 3, 4, "%02X ", frame.data[i]);
    }
    ESP_LOGW(TAG, "Undecoded frame: %s", buf);
  }
}

void BresserWeather::dump_config() {
  ESP_LOGCONFIG(TAG, "Bresser Weather (CC1101):");
  ESP_LOGCONFIG(TAG, "  MOSI: %d  MISO: %d  CLK: %d  CS: %d", this->mosi_pin_,
                this->miso_pin_, this->clk_pin_, this->cs_pin_);
  ESP_LOGCONFIG(TAG, "  GDO0: %d  GDO2: %d", this->gdo0_pin_, this->gdo2_pin_);
  ESP_LOGCONFIG(TAG, "  Frequency: %u Hz", this->frequency_hz_);
  ESP_LOGCONFIG(TAG, "  Log unknown frames: %s", YESNO(this->log_unknown_));
  ESP_LOGCONFIG(TAG, "  Radio ready: %s", YESNO(this->radio_ready_));
  ESP_LOGCONFIG(TAG, "  Registered listeners: %u",
                (unsigned) this->sensors_.size());
}

void BresserWeatherSensor::dump_config() {
  ESP_LOGCONFIG(TAG, "Bresser Weather Sensor:");
  LOG_SENSOR("  ", "Temperature", this->temperature_);
  LOG_SENSOR("  ", "Humidity", this->humidity_);
  LOG_SENSOR("  ", "Wind speed", this->wind_speed_);
  LOG_SENSOR("  ", "Wind direction", this->wind_direction_);
  LOG_SENSOR("  ", "Rain total", this->rain_total_);
  LOG_SENSOR("  ", "UV index", this->uv_index_);
  LOG_SENSOR("  ", "Light", this->light_lux_);
  LOG_SENSOR("  ", "Pressure", this->pressure_);
  LOG_SENSOR("  ", "RSSI", this->rssi_);
}

// ---------------------------------------------------------------------------
// Bresser 7-in-1 decoder, ported from rtl_433/src/devices/bresser_7in1.c
// ---------------------------------------------------------------------------
uint16_t BresserWeather::lfsr_digest16_(const uint8_t *message, unsigned bytes,
                                        uint16_t gen, uint16_t key) {
  uint16_t sum = 0;
  for (unsigned k = 0; k < bytes; ++k) {
    uint8_t data = message[k];
    for (int i = 7; i >= 0; --i) {
      if ((data >> i) & 1) {
        sum ^= key;
      }
      if (key & 1) {
        key = (key >> 1) ^ gen;
      } else {
        key = (key >> 1);
      }
    }
  }
  return sum;
}

int16_t BresserWeather::rssi_raw_to_dbm_(uint8_t raw) {
  // CC1101 datasheet RSSI conversion. Offset of 74 dB at default settings.
  int16_t v = (raw >= 128) ? ((int16_t) raw - 256) : (int16_t) raw;
  return v / 2 - 74;
}

bool BresserWeather::process_frame_(const BresserFrame &frame) {
  uint8_t buf[27];
  if (frame.length < 25) {
    return false;
  }
  uint8_t msg_len = frame.length;
  if (msg_len > sizeof(buf)) {
    msg_len = sizeof(buf);
  }
  memcpy(buf, frame.data, msg_len);
  return this->decode_bresser_7in1_(buf, msg_len, frame.rssi_dbm);
}

bool BresserWeather::decode_bresser_7in1_(uint8_t *msg, uint8_t length,
                                          int16_t rssi_dbm) {
  // The first 25 bytes carry the protocol payload; trailing bytes (if any)
  // are ignored. Whitening is a uniform XOR with 0xAA.
  if (length < 25) {
    return false;
  }
  for (uint8_t i = 0; i < 25; ++i) {
    msg[i] ^= 0xAA;
  }

  uint16_t chk = (uint16_t) (msg[0] << 8) | msg[1];
  uint16_t digest = BresserWeather::lfsr_digest16_(&msg[2], 23, 0x8810, 0xBA95);
  if ((uint16_t) (chk ^ digest) != 0x6DF1) {
    return false;
  }

  uint8_t s_type = msg[6] >> 4;
  uint8_t channel = msg[6] & 0x07;
  uint16_t sensor_id = ((uint16_t) msg[2] << 8) | msg[3];

  // Weather variants we know how to decode.
  bool is_weather =
      (s_type == 0x01 || s_type == 0x03 || s_type == 0x04 || s_type == 0x08);
  if (!is_weather) {
    ESP_LOGD(TAG, "Bresser frame: id=0x%04X type=0x%X channel=%u (not a weather variant)",
             sensor_id, s_type, channel);
    return false;
  }

  // Wind direction: 3 BCD digits in bytes 4..5 (high nibble, low nibble, high nibble).
  int wdir_raw = (msg[4] >> 4) * 100 + (msg[4] & 0x0F) * 10 + (msg[5] >> 4);
  // Wind average: 3 BCD digits across bytes 8..9 (low nibble, high nibble, low nibble), m/s.
  int wavg_raw = (msg[8] & 0x0F) * 100 + (msg[9] >> 4) * 10 + (msg[9] & 0x0F);
  // Temperature: 3 BCD digits in bytes 14..15 (with low-nibble flags).
  int temp_raw = (msg[14] >> 4) * 100 + (msg[14] & 0x0F) * 10 + (msg[15] >> 4);
  // Humidity: 2 BCD digits in byte 16.
  int humidity = (msg[16] >> 4) * 10 + (msg[16] & 0x0F);
  // Rain: 6 BCD digits in bytes 10..12, x0.1 mm.
  int rain_raw = (msg[10] >> 4) * 100000 + (msg[10] & 0x0F) * 10000 +
                 (msg[11] >> 4) * 1000 + (msg[11] & 0x0F) * 100 +
                 (msg[12] >> 4) * 10 + (msg[12] & 0x0F);
  // Light: 6 BCD digits in bytes 17..19 in lux.
  int light_raw = (msg[17] >> 4) * 100000 + (msg[17] & 0x0F) * 10000 +
                  (msg[18] >> 4) * 1000 + (msg[18] & 0x0F) * 100 +
                  (msg[19] >> 4) * 10 + (msg[19] & 0x0F);
  // UV: 3 BCD digits in bytes 20..21, x0.1.
  int uv_raw = (msg[20] >> 4) * 100 + (msg[20] & 0x0F) * 10 + (msg[21] >> 4);

  float temperature_c = temp_raw * 0.1f;
  if (temp_raw > 600) {
    // Bresser encodes negative temperatures as 1000 - |T*10|.
    temperature_c = (temp_raw - 1000) * 0.1f;
  }
  float wind_avg_kmh = wavg_raw * 0.1f * 3.6f;
  float wind_dir_deg = (float) wdir_raw;
  float rain_mm = rain_raw * 0.1f;
  float uv_index = uv_raw * 0.1f;
  float light_lux = (float) light_raw;
  float humidity_pct = (float) humidity;
  // Pressure is not carried by the standard 7-in-1 frame; leave NaN so
  // listeners that don't bind a pressure sensor stay clean.
  float pressure_hpa = NAN;

  ESP_LOGI(TAG,
           "Bresser 7-in-1 id=0x%04X ch=%u T=%.1f°C RH=%d%% Wavg=%.1fkm/h "
           "Wdir=%d° Rain=%.1fmm UV=%.1f Lux=%.0f RSSI=%ddBm",
           sensor_id, channel, temperature_c, humidity, wind_avg_kmh, wdir_raw,
           rain_mm, uv_index, light_lux, rssi_dbm);

  this->publish_(temperature_c, humidity_pct, wind_avg_kmh, wind_dir_deg,
                 rain_mm, uv_index, light_lux, pressure_hpa, rssi_dbm);
  return true;
}

void BresserWeather::publish_(float temperature, float humidity,
                              float wind_speed_kmh, float wind_dir_deg,
                              float rain_mm, float uv_index, float light_lux,
                              float pressure_hpa, int16_t rssi_dbm) {
  for (auto *s : this->sensors_) {
    if (s->temperature_ != nullptr && !std::isnan(temperature)) {
      s->temperature_->publish_state(temperature);
    }
    if (s->humidity_ != nullptr && !std::isnan(humidity)) {
      s->humidity_->publish_state(humidity);
    }
    if (s->wind_speed_ != nullptr && !std::isnan(wind_speed_kmh)) {
      s->wind_speed_->publish_state(wind_speed_kmh);
    }
    if (s->wind_direction_ != nullptr && !std::isnan(wind_dir_deg)) {
      s->wind_direction_->publish_state(wind_dir_deg);
    }
    if (s->rain_total_ != nullptr && !std::isnan(rain_mm)) {
      s->rain_total_->publish_state(rain_mm);
    }
    if (s->uv_index_ != nullptr && !std::isnan(uv_index)) {
      s->uv_index_->publish_state(uv_index);
    }
    if (s->light_lux_ != nullptr && !std::isnan(light_lux)) {
      s->light_lux_->publish_state(light_lux);
    }
    if (s->pressure_ != nullptr && !std::isnan(pressure_hpa)) {
      s->pressure_->publish_state(pressure_hpa);
    }
    if (s->rssi_ != nullptr) {
      s->rssi_->publish_state(rssi_dbm);
    }
  }
}

}  // namespace bresser_weather
}  // namespace esphome
