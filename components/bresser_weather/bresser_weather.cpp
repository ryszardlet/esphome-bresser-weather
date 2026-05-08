#include "bresser_weather.h"

#include "esphome/core/log.h"
#include "esphome/core/hal.h"

#ifdef USE_MQTT
#include "esphome/components/mqtt/mqtt_client.h"
#endif

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace esphome {
namespace bresser_weather {

static const char *const TAG = "bresser_weather";

// ---------------------------------------------------------------------------
// CC1101 register addresses
// ---------------------------------------------------------------------------
namespace reg {
constexpr uint8_t IOCFG2 = 0x00;
constexpr uint8_t IOCFG0 = 0x02;
constexpr uint8_t FIFOTHR = 0x03;
constexpr uint8_t SYNC1 = 0x04;
constexpr uint8_t SYNC0 = 0x05;
constexpr uint8_t PKTLEN = 0x06;
constexpr uint8_t PKTCTRL1 = 0x07;
constexpr uint8_t PKTCTRL0 = 0x08;
constexpr uint8_t CHANNR = 0x0A;
constexpr uint8_t FSCTRL1 = 0x0B;
constexpr uint8_t FSCTRL0 = 0x0C;
constexpr uint8_t FREQ2 = 0x0D;
constexpr uint8_t FREQ1 = 0x0E;
constexpr uint8_t FREQ0 = 0x0F;
constexpr uint8_t MDMCFG4 = 0x10;
constexpr uint8_t MDMCFG3 = 0x11;
constexpr uint8_t MDMCFG2 = 0x12;
constexpr uint8_t MDMCFG1 = 0x13;
constexpr uint8_t MDMCFG0 = 0x14;
constexpr uint8_t DEVIATN = 0x15;
constexpr uint8_t MCSM2 = 0x16;
constexpr uint8_t MCSM1 = 0x17;
constexpr uint8_t MCSM0 = 0x18;
constexpr uint8_t FOCCFG = 0x19;
constexpr uint8_t BSCFG = 0x1A;
constexpr uint8_t AGCCTRL2 = 0x1B;
constexpr uint8_t AGCCTRL1 = 0x1C;
constexpr uint8_t AGCCTRL0 = 0x1D;
constexpr uint8_t FREND1 = 0x21;
constexpr uint8_t FREND0 = 0x22;
constexpr uint8_t FSCAL3 = 0x23;
constexpr uint8_t FSCAL2 = 0x24;
constexpr uint8_t FSCAL1 = 0x25;
constexpr uint8_t FSCAL0 = 0x26;
constexpr uint8_t TEST2 = 0x2C;
constexpr uint8_t TEST1 = 0x2D;
constexpr uint8_t TEST0 = 0x2E;
constexpr uint8_t PARTNUM = 0x30;
constexpr uint8_t VERSION = 0x31;
constexpr uint8_t RSSI = 0x34;
constexpr uint8_t MARCSTATE = 0x35;
constexpr uint8_t PKTSTATUS = 0x38;
constexpr uint8_t RXBYTES = 0x3B;
constexpr uint8_t PATABLE = 0x3E;
constexpr uint8_t FIFO = 0x3F;

constexpr uint8_t SRES = 0x30;
constexpr uint8_t SCAL = 0x33;
constexpr uint8_t SRX = 0x34;
constexpr uint8_t SIDLE = 0x36;
constexpr uint8_t SFRX = 0x3A;

constexpr uint8_t READ_SINGLE = 0x80;
constexpr uint8_t READ_BURST = 0xC0;
constexpr uint8_t WRITE_BURST = 0x40;
}  // namespace reg

static constexpr uint32_t CC1101_XTAL_HZ = 26000000UL;

// ---------------------------------------------------------------------------
// Radio presets used in scan mode. Index 0 is also the default profile when
// scan_mode is disabled.
//
// CC1101 only fills the RX FIFO after sync detection, so even the "fuzz"
// presets enable sync — they just use 0xAAAA which matches the preamble
// bytes the sensor sends. find_sync_bit_() then locates the real sync
// inside the captured bytes.
// ---------------------------------------------------------------------------
const RadioPreset BresserWeather::PRESETS[BresserWeather::PRESET_COUNT] = {
    {"A_canonical",   868300000,  8.21f,  57.136f, 270.0f, true, 0x2DD4, 26},
    {"B_aa_preamble", 868300000,  8.21f,  57.136f, 270.0f, true, 0xAAAA, 64},
    {"C_17_2kbps",    868300000, 17.241f, 40.0f,   203.0f, true, 0x2DD4, 26},
    {"D_18_8kbps",    868300000, 18.868f, 40.0f,   203.0f, true, 0x2DD4, 26},
    {"E_aa2d_sync",   868300000,  8.21f,  57.136f, 270.0f, true, 0xAA2D, 27},
    {"F_868_35MHz",   868350000,  8.21f,  57.136f, 270.0f, true, 0x2DD4, 26},
    {"G_4_8kbps_aa",  868300000,  4.8f,    9.6f,    58.0f, true, 0xAAAA, 64},
    {"H_9_6kbps_aa",  868300000,  9.6f,   40.0f,   203.0f, true, 0xAAAA, 64},
};

// ---------------------------------------------------------------------------
// SPI plumbing
//
// Mirrors the wiring used by ryszardlet/esphome-wmbus-cc1101's CC1101Driver,
// which is known-good on the same ESP32 + CC1101 hardware:
//   1) cs_low_()
//   2) wait_miso_low_()         <-- chip ready (CHIP_RDYn)
//   3) spi.beginTransaction(...)
//   4) spi.transfer(...)
//   5) spi.endTransaction()
//   6) cs_high_()
// CS is toggled OUTSIDE the transaction wrapper; that's what the working
// driver does and what fixes the silent setup() failure we saw in v0.2.0.
// ---------------------------------------------------------------------------
void BresserWeather::cc1101_select_() { digitalWrite(this->cs_pin_, LOW); }
void BresserWeather::cc1101_deselect_() { digitalWrite(this->cs_pin_, HIGH); }

// Software SPI (mode 0, MSB first). One bit per loop iteration:
//   1. set MOSI to outgoing bit
//   2. CLK rising edge
//   3. half-period delay
//   4. sample MISO
//   5. CLK falling edge
//   6. half-period delay
// At 1 us half-period this gives ~500 kHz, plenty for CC1101.
uint8_t BresserWeather::bitbang_xfer_(uint8_t out) {
  uint8_t in = 0;
  for (int i = 7; i >= 0; --i) {
    digitalWrite(this->mosi_pin_, (out >> i) & 1 ? HIGH : LOW);
    digitalWrite(this->clk_pin_, HIGH);
    if (this->bitbang_half_period_us_) delayMicroseconds(this->bitbang_half_period_us_);
    in = (in << 1) | (digitalRead(this->miso_pin_) ? 1 : 0);
    digitalWrite(this->clk_pin_, LOW);
    if (this->bitbang_half_period_us_) delayMicroseconds(this->bitbang_half_period_us_);
  }
  return in;
}

bool BresserWeather::cc1101_wait_miso_low_() {
  // CC1101 holds CHIP_RDYn (= MISO) high until its crystal stabilises after
  // CS is asserted. Match wmbus driver's 20 ms deadline; if MISO never goes
  // low we still return so the caller can probe registers and decide.
  uint32_t deadline = millis() + 20;
  while (digitalRead(this->miso_pin_) == HIGH) {
    if ((int32_t) (millis() - deadline) >= 0) return false;
  }
  return true;
}

void BresserWeather::cc1101_write_reg_(uint8_t addr, uint8_t value) {
  this->cc1101_select_();
  this->cc1101_wait_miso_low_();
  if (this->bitbang_spi_) {
    this->bitbang_xfer_(addr);
    this->bitbang_xfer_(value);
  } else {
    this->spi_->beginTransaction(this->spi_settings_);
    this->spi_->transfer(addr);
    this->spi_->transfer(value);
    this->spi_->endTransaction();
  }
  this->cc1101_deselect_();
  ESP_LOGV(TAG, "WR 0x%02X = 0x%02X", addr, value);
}

bool BresserWeather::cc1101_write_verify_(uint8_t addr, uint8_t value,
                                          const char *name) {
  this->cc1101_write_reg_(addr, value);
  uint8_t got = this->cc1101_read_reg_(addr);
  if (got != value) {
    ESP_LOGE(TAG, "Register %s (0x%02X) write failed: wrote 0x%02X read 0x%02X",
             name, addr, value, got);
    return false;
  }
  return true;
}

uint8_t BresserWeather::cc1101_read_reg_(uint8_t addr) {
  this->cc1101_select_();
  this->cc1101_wait_miso_low_();
  uint8_t v;
  if (this->bitbang_spi_) {
    this->bitbang_xfer_(addr | reg::READ_SINGLE);
    v = this->bitbang_xfer_(0);
  } else {
    this->spi_->beginTransaction(this->spi_settings_);
    this->spi_->transfer(addr | reg::READ_SINGLE);
    v = this->spi_->transfer(0);
    this->spi_->endTransaction();
  }
  this->cc1101_deselect_();
  return v;
}

uint8_t BresserWeather::cc1101_read_status_(uint8_t addr) {
  this->cc1101_select_();
  this->cc1101_wait_miso_low_();
  uint8_t v;
  if (this->bitbang_spi_) {
    this->bitbang_xfer_(addr | reg::READ_BURST);
    v = this->bitbang_xfer_(0);
  } else {
    this->spi_->beginTransaction(this->spi_settings_);
    this->spi_->transfer(addr | reg::READ_BURST);
    v = this->spi_->transfer(0);
    this->spi_->endTransaction();
  }
  this->cc1101_deselect_();
  return v;
}

void BresserWeather::cc1101_strobe_(uint8_t cmd) {
  this->cc1101_select_();
  this->cc1101_wait_miso_low_();
  if (this->bitbang_spi_) {
    this->bitbang_xfer_(cmd);
  } else {
    this->spi_->beginTransaction(this->spi_settings_);
    this->spi_->transfer(cmd);
    this->spi_->endTransaction();
  }
  this->cc1101_deselect_();
}

void BresserWeather::cc1101_read_burst_(uint8_t addr, uint8_t *buf,
                                        uint8_t len) {
  this->cc1101_select_();
  this->cc1101_wait_miso_low_();
  if (this->bitbang_spi_) {
    this->bitbang_xfer_(addr | reg::READ_BURST);
    for (uint8_t i = 0; i < len; ++i) buf[i] = this->bitbang_xfer_(0);
  } else {
    this->spi_->beginTransaction(this->spi_settings_);
    this->spi_->transfer(addr | reg::READ_BURST);
    for (uint8_t i = 0; i < len; ++i) buf[i] = this->spi_->transfer(0);
    this->spi_->endTransaction();
  }
  this->cc1101_deselect_();
}

void BresserWeather::cc1101_write_burst_(uint8_t addr, const uint8_t *buf,
                                         uint8_t len) {
  this->cc1101_select_();
  this->cc1101_wait_miso_low_();
  if (this->bitbang_spi_) {
    this->bitbang_xfer_(addr | reg::WRITE_BURST);
    for (uint8_t i = 0; i < len; ++i) this->bitbang_xfer_(buf[i]);
  } else {
    this->spi_->beginTransaction(this->spi_settings_);
    this->spi_->transfer(addr | reg::WRITE_BURST);
    for (uint8_t i = 0; i < len; ++i) this->spi_->transfer(buf[i]);
    this->spi_->endTransaction();
  }
  this->cc1101_deselect_();
}

// Manual reset per datasheet §19.1.2.
void BresserWeather::cc1101_reset_() {
  ESP_LOGE(TAG, "  >>> CC1101 reset: CS dance + SRES strobe");
  digitalWrite(this->cs_pin_, HIGH);
  delayMicroseconds(40);
  digitalWrite(this->cs_pin_, LOW);
  delayMicroseconds(40);
  digitalWrite(this->cs_pin_, HIGH);
  delayMicroseconds(40);
  digitalWrite(this->cs_pin_, LOW);
  bool miso_ok_pre = this->cc1101_wait_miso_low_();
  uint8_t reset_status;
  if (this->bitbang_spi_) {
    reset_status = this->bitbang_xfer_(reg::SRES);
  } else {
    this->spi_->beginTransaction(this->spi_settings_);
    reset_status = this->spi_->transfer(reg::SRES);
    this->spi_->endTransaction();
  }
  bool miso_ok_post = this->cc1101_wait_miso_low_();
  digitalWrite(this->cs_pin_, HIGH);
  delay(10);
  ESP_LOGE(TAG, "      SRES status byte=0x%02X miso_pre=%s miso_post=%s",
           reset_status, miso_ok_pre ? "LOW" : "HIGH/timeout",
           miso_ok_post ? "LOW" : "HIGH/timeout");
}

bool BresserWeather::cc1101_probe_() {
  uint8_t part = this->cc1101_read_status_(reg::PARTNUM);
  uint8_t version = this->cc1101_read_status_(reg::VERSION);
  ESP_LOGI(TAG, "  CC1101 probe: PARTNUM=0x%02X VERSION=0x%02X", part, version);

  // We do NOT mark_failed on any of these — the user wants the component to
  // keep running so they can see follow-on logs / try scan_mode / etc.
  if (version == 0x00 || version == 0xFF) {
    ESP_LOGE(TAG,
             "  CC1101 not responding (VERSION=0x%02X). Check 3.3V power, "
             "MOSI/MISO swap, CS active-low, SCK frequency. Continuing anyway.",
             version);
    return false;
  }
  if (part != 0x00) {
    ESP_LOGW(TAG, "  CC1101 PARTNUM=0x%02X unexpected (TI sells only 0x00). "
                  "Some clones report different — continuing.", part);
  }
  if (version != 0x04 && version != 0x14) {
    ESP_LOGW(TAG, "  CC1101 VERSION=0x%02X unusual (expected 0x04 or 0x14). "
                  "Continuing.", version);
  }
  return true;
}

void BresserWeather::cc1101_flush_rx_() {
  this->cc1101_strobe_(reg::SIDLE);
  delayMicroseconds(100);
  this->cc1101_strobe_(reg::SFRX);
  delayMicroseconds(100);
}

void BresserWeather::cc1101_enter_rx_() {
  this->cc1101_flush_rx_();
  this->cc1101_strobe_(reg::SRX);
}

// ---------------------------------------------------------------------------
// Modulation register math
// ---------------------------------------------------------------------------
void BresserWeather::calc_drate_(float kbps, uint8_t &drate_e, uint8_t &drate_m) {
  // bitrate = (256 + DRATE_M) * 2^DRATE_E * Fxosc / 2^28
  double target = (double) kbps * 1000.0 * (double) (1ULL << 28) /
                  (double) CC1101_XTAL_HZ;
  for (int e = 15; e >= 0; --e) {
    double base = (double) (1ULL << e);
    double m = (target / base) - 256.0;
    if (m >= 0.0 && m <= 255.0) {
      drate_e = (uint8_t) e;
      drate_m = (uint8_t) std::lround(m);
      return;
    }
  }
  drate_e = 0;
  drate_m = 0;
}

void BresserWeather::calc_dev_(float khz, uint8_t &dev_e, uint8_t &dev_m) {
  // f_dev = Fxosc / 2^17 * (8 + DEV_M) * 2^DEV_E
  double target = (double) khz * 1000.0 * (double) (1ULL << 17) /
                  (double) CC1101_XTAL_HZ;
  double best_err = 1e9;
  uint8_t best_e = 0, best_m = 0;
  for (int e = 0; e < 8; ++e) {
    for (int m = 0; m < 8; ++m) {
      double v = (double) (8 + m) * (double) (1ULL << e);
      double err = std::abs(v - target);
      if (err < best_err) {
        best_err = err;
        best_e = (uint8_t) e;
        best_m = (uint8_t) m;
      }
    }
  }
  dev_e = best_e;
  dev_m = best_m;
}

uint8_t BresserWeather::calc_chanbw_(float khz) {
  // BW = Fxosc / (8 * (4+CHANBW_M) * 2^CHANBW_E)
  double target = (double) CC1101_XTAL_HZ / (8.0 * (double) khz * 1000.0);
  double best_err = 1e9;
  uint8_t best_e = 0, best_m = 0;
  for (int e = 0; e < 4; ++e) {
    for (int m = 0; m < 4; ++m) {
      double v = (double) (4 + m) * (double) (1ULL << e);
      double err = std::abs(v - target);
      if (err < best_err) {
        best_err = err;
        best_e = (uint8_t) e;
        best_m = (uint8_t) m;
      }
    }
  }
  return (uint8_t) ((best_e << 6) | (best_m << 4));
}

// ---------------------------------------------------------------------------
// Apply a high-level RadioPreset to the chip
// ---------------------------------------------------------------------------
void BresserWeather::apply_preset_(const RadioPreset &p) {
  ESP_LOGI(TAG, "==> Applying preset '%s': freq=%.3fMHz br=%.3fkbps dev=%.1fkHz "
                "bw=%.0fkHz sync=%s pkt=%u",
           p.name, p.freq_hz / 1e6f, p.bitrate_kbps, p.deviation_khz,
           p.rxbw_khz, p.sync_enabled ? "ON" : "OFF", p.pkt_len);

  this->cc1101_strobe_(reg::SIDLE);
  delayMicroseconds(200);

  // GDO2: assert on sync detect, deassert at end of packet (or after
  //       fixed-length burst when sync is disabled).
  this->cc1101_write_verify_(reg::IOCFG2, 0x06, "IOCFG2");
  this->cc1101_write_verify_(reg::IOCFG0, 0x06, "IOCFG0");
  this->cc1101_write_verify_(reg::FIFOTHR, 0x47, "FIFOTHR");

  // Sync word
  this->cc1101_write_verify_(reg::SYNC1, (p.sync_word >> 8) & 0xFF, "SYNC1");
  this->cc1101_write_verify_(reg::SYNC0, p.sync_word & 0xFF, "SYNC0");

  this->cc1101_write_verify_(reg::PKTLEN, p.pkt_len, "PKTLEN");
  this->cc1101_write_verify_(reg::PKTCTRL1, 0x00, "PKTCTRL1");
  this->cc1101_write_verify_(reg::PKTCTRL0, 0x00, "PKTCTRL0");
  this->cc1101_write_verify_(reg::CHANNR, 0x00, "CHANNR");
  this->cc1101_write_verify_(reg::FSCTRL1, 0x06, "FSCTRL1");
  this->cc1101_write_verify_(reg::FSCTRL0, 0x00, "FSCTRL0");

  // Frequency
  uint64_t freq_word = ((uint64_t) p.freq_hz << 16) / (uint64_t) CC1101_XTAL_HZ;
  uint8_t f2 = (freq_word >> 16) & 0xFF;
  uint8_t f1 = (freq_word >> 8) & 0xFF;
  uint8_t f0 = freq_word & 0xFF;
  this->cc1101_write_verify_(reg::FREQ2, f2, "FREQ2");
  this->cc1101_write_verify_(reg::FREQ1, f1, "FREQ1");
  this->cc1101_write_verify_(reg::FREQ0, f0, "FREQ0");

  // Modem
  uint8_t drate_e, drate_m;
  calc_drate_(p.bitrate_kbps, drate_e, drate_m);
  uint8_t chanbw = calc_chanbw_(p.rxbw_khz);
  uint8_t mdmcfg4 = chanbw | (drate_e & 0x0F);
  this->cc1101_write_verify_(reg::MDMCFG4, mdmcfg4, "MDMCFG4");
  this->cc1101_write_verify_(reg::MDMCFG3, drate_m, "MDMCFG3");

  // MDMCFG2:
  //   bit7=DEM_DCFILT_OFF (0=enabled), bits6-4=MOD_FORMAT (0=2-FSK),
  //   bit3=MANCHESTER (0=off), bits2-0=SYNC_MODE.
  // SYNC_MODE 0b010 = 16/16 sync match, 0b000 = no preamble/sync.
  uint8_t mdmcfg2 = p.sync_enabled ? 0x02 : 0x00;
  this->cc1101_write_verify_(reg::MDMCFG2, mdmcfg2, "MDMCFG2");

  // 4-byte preamble, default channel-spacing exponent.
  this->cc1101_write_verify_(reg::MDMCFG1, 0x22, "MDMCFG1");
  this->cc1101_write_verify_(reg::MDMCFG0, 0xF8, "MDMCFG0");

  uint8_t dev_e, dev_m;
  calc_dev_(p.deviation_khz, dev_e, dev_m);
  uint8_t deviatn = ((dev_e & 0x07) << 4) | (dev_m & 0x07);
  this->cc1101_write_verify_(reg::DEVIATN, deviatn, "DEVIATN");

  // MCSM: stay in RX after a packet, auto-cal IDLE->RX.
  this->cc1101_write_verify_(reg::MCSM2, 0x07, "MCSM2");
  this->cc1101_write_verify_(reg::MCSM1, 0x3F, "MCSM1");
  this->cc1101_write_verify_(reg::MCSM0, 0x18, "MCSM0");

  // Frequency offset comp + bit sync (TI-recommended for low-rate FSK).
  this->cc1101_write_verify_(reg::FOCCFG, 0x16, "FOCCFG");
  this->cc1101_write_verify_(reg::BSCFG, 0x6C, "BSCFG");

  // AGC tuned for weak-signal reception (matthias-bs values).
  this->cc1101_write_verify_(reg::AGCCTRL2, 0xC7, "AGCCTRL2");
  this->cc1101_write_verify_(reg::AGCCTRL1, 0x00, "AGCCTRL1");
  this->cc1101_write_verify_(reg::AGCCTRL0, 0xB2, "AGCCTRL0");

  this->cc1101_write_verify_(reg::FREND1, 0xB6, "FREND1");
  this->cc1101_write_verify_(reg::FREND0, 0x10, "FREND0");
  this->cc1101_write_verify_(reg::FSCAL3, 0xEA, "FSCAL3");
  this->cc1101_write_verify_(reg::FSCAL2, 0x2A, "FSCAL2");
  this->cc1101_write_verify_(reg::FSCAL1, 0x00, "FSCAL1");
  this->cc1101_write_verify_(reg::FSCAL0, 0x1F, "FSCAL0");
  this->cc1101_write_verify_(reg::TEST2, 0x81, "TEST2");
  this->cc1101_write_verify_(reg::TEST1, 0x35, "TEST1");
  this->cc1101_write_verify_(reg::TEST0, 0x09, "TEST0");

  uint8_t patable[] = {0xC0};
  this->cc1101_write_burst_(reg::PATABLE, patable, sizeof(patable));

  // Manual frequency calibration after register changes.
  this->cc1101_strobe_(reg::SCAL);
  delay(2);

  ESP_LOGI(TAG, "    -> drate_e=%u drate_m=%u dev_e=%u dev_m=%u "
                "MDMCFG4=0x%02X DEVIATN=0x%02X FREQ=%02X%02X%02X",
           drate_e, drate_m, dev_e, dev_m, mdmcfg4, deviatn, f2, f1, f0);

  this->cc1101_enter_rx_();
}

void BresserWeather::log_register_dump_() {
  uint8_t marc = this->cc1101_read_status_(reg::MARCSTATE);
  ESP_LOGE(TAG, "  >>> Register dump after init:");
  ESP_LOGE(TAG, "      PARTNUM=0x%02X VERSION=0x%02X MARCSTATE=0x%02X",
           this->cc1101_read_status_(reg::PARTNUM),
           this->cc1101_read_status_(reg::VERSION), marc);
  ESP_LOGE(TAG, "      FREQ=%02X%02X%02X", this->cc1101_read_reg_(reg::FREQ2),
           this->cc1101_read_reg_(reg::FREQ1),
           this->cc1101_read_reg_(reg::FREQ0));
  ESP_LOGE(TAG, "      MDMCFG4=0x%02X MDMCFG3=0x%02X MDMCFG2=0x%02X DEVIATN=0x%02X",
           this->cc1101_read_reg_(reg::MDMCFG4),
           this->cc1101_read_reg_(reg::MDMCFG3),
           this->cc1101_read_reg_(reg::MDMCFG2),
           this->cc1101_read_reg_(reg::DEVIATN));
  ESP_LOGE(TAG, "      SYNC=%02X%02X PKTLEN=0x%02X PKTCTRL0=0x%02X PKTCTRL1=0x%02X",
           this->cc1101_read_reg_(reg::SYNC1),
           this->cc1101_read_reg_(reg::SYNC0),
           this->cc1101_read_reg_(reg::PKTLEN),
           this->cc1101_read_reg_(reg::PKTCTRL0),
           this->cc1101_read_reg_(reg::PKTCTRL1));
  ESP_LOGE(TAG, "      AGCCTRL2/1/0=0x%02X/0x%02X/0x%02X FREND1=0x%02X",
           this->cc1101_read_reg_(reg::AGCCTRL2),
           this->cc1101_read_reg_(reg::AGCCTRL1),
           this->cc1101_read_reg_(reg::AGCCTRL0),
           this->cc1101_read_reg_(reg::FREND1));
  if (marc != 0x0D) {
    ESP_LOGW(TAG, "MARCSTATE=0x%02X is not RX(0x0D) - radio is not listening!",
             marc);
  }
}

// ---------------------------------------------------------------------------
// Component lifecycle
// ---------------------------------------------------------------------------
void BresserWeather::setup() {
  // setup() runs BEFORE the ESPHome API/OTA log stream is connected, so any
  // ESP_LOGE here only reaches a serial console. We capture all diagnostic
  // results into member fields so the early loop() iterations can re-emit
  // them — that way the user sees them over OTA too.
  ESP_LOGE(TAG, "=== bresser_weather setup() entry (v0.2.4) ===");
  ESP_LOGE(TAG, "  pins MOSI=%d MISO=%d CLK=%d CS=%d GDO0=%d GDO2=%d",
           mosi_pin_, miso_pin_, clk_pin_, cs_pin_, gdo0_pin_, gdo2_pin_);
  ESP_LOGE(TAG, "  SPI mode: %s @ %u Hz",
           this->bitbang_spi_ ? "BIT-BANG (software)" : "HARDWARE (Arduino SPI)",
           (unsigned) this->spi_clock_hz_);

  // GPIO config. In bit-bang mode we own MOSI/CLK as outputs and MISO as
  // input; in hardware mode the Arduino SPI.begin() owns them.
  pinMode(this->cs_pin_, OUTPUT);
  digitalWrite(this->cs_pin_, HIGH);
  pinMode(this->gdo0_pin_, INPUT);
  if (this->gdo2_pin_ >= 0) pinMode(this->gdo2_pin_, INPUT);

  if (this->bitbang_spi_) {
    pinMode(this->mosi_pin_, OUTPUT);
    digitalWrite(this->mosi_pin_, LOW);
    pinMode(this->clk_pin_, OUTPUT);
    digitalWrite(this->clk_pin_, LOW);
    pinMode(this->miso_pin_, INPUT);
    // Compute half-period from configured clock rate. Cap at 100 kHz
    // (=5 us half-period) for absolute reliability.
    uint32_t hz = this->spi_clock_hz_;
    if (hz < 50000) hz = 50000;
    if (hz > 4000000) hz = 4000000;
    uint32_t half_period_us = 500000UL / hz;  // half of (1/hz) seconds in us
    if (half_period_us == 0) half_period_us = 1;
    this->bitbang_half_period_us_ = half_period_us;
    ESP_LOGE(TAG, "  bit-bang half-period: %u us (≈%u Hz)",
             (unsigned) half_period_us,
             (unsigned) (500000UL / half_period_us));
  } else {
    pinMode(this->miso_pin_, INPUT);
  }

  this->diag_miso_pre_spi_high_ = digitalRead(this->miso_pin_) == HIGH;
  ESP_LOGE(TAG, "  pre-SPI MISO digital read=%s",
           this->diag_miso_pre_spi_high_ ? "HIGH" : "LOW");

  if (!this->bitbang_spi_) {
    ESP_LOGE(TAG, "  creating SPIClass and calling begin() at %u Hz...",
             (unsigned) this->spi_clock_hz_);
    this->spi_settings_ = SPISettings(this->spi_clock_hz_, MSBFIRST, SPI_MODE0);
    this->spi_ = new SPIClass();
    this->spi_->begin(this->clk_pin_, this->miso_pin_, this->mosi_pin_,
                      this->cs_pin_);
    this->spi_->setHwCs(false);
    delay(10);
  }
  this->diag_miso_post_spi_high_ = digitalRead(this->miso_pin_) == HIGH;
  ESP_LOGE(TAG, "  SPI ready; post-begin MISO=%s, CS pin reads=%s",
           this->diag_miso_post_spi_high_ ? "HIGH" : "LOW",
           digitalRead(this->cs_pin_) ? "HIGH" : "LOW");

  // ---- SNOP loopback ----
  this->cc1101_select_();
  this->diag_miso_after_cs_low_ = this->cc1101_wait_miso_low_();
  if (this->bitbang_spi_) {
    this->diag_snop_status_ = this->bitbang_xfer_(0x3D);
  } else {
    this->spi_->beginTransaction(this->spi_settings_);
    this->diag_snop_status_ = this->spi_->transfer(0x3D);
    this->spi_->endTransaction();
  }
  this->cc1101_deselect_();
  ESP_LOGE(TAG, "  SNOP: after CS LOW miso=%s, status byte=0x%02X",
           this->diag_miso_after_cs_low_ ? "LOW(ready)" : "HIGH/timeout",
           this->diag_snop_status_);

  this->cc1101_reset_();

  this->diag_partnum_ = this->cc1101_read_status_(reg::PARTNUM);
  this->diag_version_ = this->cc1101_read_status_(reg::VERSION);
  ESP_LOGE(TAG, "  PARTNUM=0x%02X VERSION=0x%02X", this->diag_partnum_,
           this->diag_version_);

  // ---- Write-then-read echo test on a benign writable register ----
  ESP_LOGE(TAG, "  echo test on FSCTRL0 (write→read three patterns):");
  for (int i = 0; i < 3; ++i) {
    uint8_t pattern = this->diag_echo_written_[i];
    this->cc1101_write_reg_(reg::FSCTRL0, pattern);
    this->diag_echo_read_[i] = this->cc1101_read_reg_(reg::FSCTRL0);
    ESP_LOGE(TAG, "    wrote 0x%02X, read 0x%02X — %s", pattern,
             this->diag_echo_read_[i],
             (this->diag_echo_read_[i] == pattern) ? "OK" : "MISMATCH");
  }
  this->cc1101_write_reg_(reg::FSCTRL0, 0x00);

  bool probe_ok = (this->diag_version_ != 0x00 && this->diag_version_ != 0xFF);
  if (probe_ok) {
    ESP_LOGE(TAG, "  *** CC1101 detected OK ***");
  } else {
    ESP_LOGE(TAG, "  *** CC1101 probe FAILED — keeping component active ***");
  }

  RadioPreset live = PRESETS[0];
  live.freq_hz = this->configured_freq_hz_;
  this->apply_preset_(live);
  this->log_register_dump_();
  this->radio_ready_ = probe_ok;
  this->diag_setup_done_ = true;

  this->scan_started_ms_ = millis();
  ESP_LOGE(TAG, "=== setup() complete (radio_ready=%s) ===",
           YESNO(this->radio_ready_));
}

void BresserWeather::loop() {
  uint32_t now = millis();
  // Note: we deliberately keep the loop running even when radio_ready_ is
  // false — the heartbeat below shows what the chip is responding with so
  // a wiring/SPI bug can be diagnosed instead of silently disabled.

  // ---- Re-emit setup() diagnostics over the OTA log stream ----
  // setup() finishes before the ESPHome API/OTA log connects, so the [E]
  // lines from setup() never reach the dashboard. We re-emit them three
  // times: at first loop, then 5 s later, then 15 s later. After that we
  // stop spamming.
  if (this->diag_setup_done_ && this->diag_dumps_emitted_ < 3) {
    bool fire = false;
    if (this->diag_dumps_emitted_ == 0) {
      this->diag_first_dump_ms_ = now;
      fire = true;
    } else if (this->diag_dumps_emitted_ == 1 &&
               (now - this->diag_first_dump_ms_) >= 5000) {
      fire = true;
    } else if (this->diag_dumps_emitted_ == 2 &&
               (now - this->diag_first_dump_ms_) >= 15000) {
      fire = true;
    }
    if (fire) {
      this->diag_dumps_emitted_++;
      ESP_LOGE(TAG, "[BOOT-DIAG #%d] === setup() captures (re-emit for OTA) ===",
               this->diag_dumps_emitted_);
      ESP_LOGE(TAG, "[BOOT-DIAG] pins MOSI=%d MISO=%d CLK=%d CS=%d GDO0=%d GDO2=%d",
               mosi_pin_, miso_pin_, clk_pin_, cs_pin_, gdo0_pin_, gdo2_pin_);
      ESP_LOGE(TAG, "[BOOT-DIAG] pre-SPI MISO=%s, post-begin MISO=%s",
               this->diag_miso_pre_spi_high_ ? "HIGH" : "LOW",
               this->diag_miso_post_spi_high_ ? "HIGH" : "LOW");
      ESP_LOGE(TAG, "[BOOT-DIAG] after CS LOW MISO went LOW: %s",
               this->diag_miso_after_cs_low_ ? "YES" : "NO/timeout");
      ESP_LOGE(TAG, "[BOOT-DIAG] SNOP status byte=0x%02X (expect 0x0F when OK; "
                    "0x00 = MISO stuck LOW; 0xFF = MISO stuck HIGH)",
               this->diag_snop_status_);
      ESP_LOGE(TAG, "[BOOT-DIAG] PARTNUM=0x%02X VERSION=0x%02X (expect 0x00 / 0x14)",
               this->diag_partnum_, this->diag_version_);
      for (int i = 0; i < 3; ++i) {
        ESP_LOGE(TAG, "[BOOT-DIAG] echo wrote=0x%02X read=0x%02X — %s",
                 this->diag_echo_written_[i], this->diag_echo_read_[i],
                 (this->diag_echo_written_[i] == this->diag_echo_read_[i])
                     ? "OK"
                     : "MISMATCH (SPI broken)");
      }
      // Verdict
      bool any_echo_ok = false;
      bool all_echo_ok = true;
      for (int i = 0; i < 3; ++i) {
        if (this->diag_echo_written_[i] == this->diag_echo_read_[i])
          any_echo_ok = true;
        else
          all_echo_ok = false;
      }
      if (this->diag_snop_status_ == 0x00 && this->diag_partnum_ == 0x00 &&
          this->diag_version_ == 0x00 && !any_echo_ok) {
        ESP_LOGE(TAG, "[BOOT-DIAG] verdict: MISO line is dead/floating LOW. "
                      "Most likely: chip not powered, MISO not connected, or "
                      "wrong MISO pin. Compare to wmbus_cc1101 wiring.");
      } else if (this->diag_snop_status_ == 0xFF &&
                 this->diag_version_ == 0xFF && !any_echo_ok) {
        ESP_LOGE(TAG, "[BOOT-DIAG] verdict: MISO line is floating HIGH. "
                      "Most likely: CS pin not actually reaching chip, chip "
                      "not powered, or hardware-CS interfering with manual.");
      } else if (all_echo_ok) {
        ESP_LOGE(TAG, "[BOOT-DIAG] verdict: SPI bidirectional WORKING. "
                      "Bug must be downstream (register config / RX state).");
      } else {
        ESP_LOGE(TAG, "[BOOT-DIAG] verdict: SPI partially working. Bus "
                      "integrity issue — try lower SPI clock or shorter wires.");
      }
      ESP_LOGE(TAG, "[BOOT-DIAG] === end ===");
    }
  }

  // ---- scan mode cycling ----
  if (this->scan_mode_ &&
      (now - this->scan_started_ms_) >= this->scan_interval_ms_) {
    this->current_preset_idx_ = (this->current_preset_idx_ + 1) % PRESET_COUNT;
    RadioPreset live = PRESETS[this->current_preset_idx_];
    if (this->current_preset_idx_ == 0) live.freq_hz = this->configured_freq_hz_;
    this->apply_preset_(live);
    this->scan_started_ms_ = now;
  }

  // ---- periodic status log ----
  if ((now - this->last_status_log_ms_) >= this->status_interval_ms_) {
    this->last_status_log_ms_ = now;
    uint8_t marc = this->cc1101_read_status_(reg::MARCSTATE);
    uint8_t rxb = this->cc1101_read_status_(reg::RXBYTES);
    uint8_t pktstatus = this->cc1101_read_status_(reg::PKTSTATUS);
    int rssi_dbm = rssi_raw_to_dbm_(this->cc1101_read_status_(reg::RSSI));
    bool gdo0 = digitalRead(this->gdo0_pin_) == HIGH;
    const char *cfg = this->scan_mode_
                          ? PRESETS[this->current_preset_idx_].name
                          : "default";
    ESP_LOGI(TAG, "Waiting... uptime=%us cfg=%s rx=%u/%u marc=0x%02X "
                  "rxbytes=%u pktstatus=0x%02X rssi=%ddBm gdo0=%s",
             now / 1000, cfg, this->valid_packets_, this->total_packets_,
             marc, rxb & 0x7F, pktstatus, rssi_dbm,
             gdo0 ? "HIGH" : "LOW");
    if (marc != 0x0D) {
      ESP_LOGW(TAG, "MARCSTATE=0x%02X != RX(0x0D) - re-entering RX", marc);
      this->cc1101_enter_rx_();
    }
  }

  // ---- packet reception ----
  uint8_t rxb_raw = this->cc1101_read_status_(reg::RXBYTES);
  bool overflow = (rxb_raw & 0x80) != 0;
  uint8_t available = rxb_raw & 0x7F;

  if (overflow) {
    // Throttle the spam — when SPI is broken every iteration sees overflow.
    static uint32_t last_overflow_log = 0;
    if ((now - last_overflow_log) >= 5000) {
      last_overflow_log = now;
      ESP_LOGW(TAG, "RX FIFO overflow (rxbytes=0x%02X) - flushing", rxb_raw);
    }
    this->cc1101_enter_rx_();
    return;
  }

  uint8_t want = PRESETS[this->current_preset_idx_].pkt_len;
  if (this->scan_mode_ == false) want = PRESETS[0].pkt_len;
  if (available < want) return;

  uint8_t buf[64];
  if (want > sizeof(buf)) want = sizeof(buf);
  this->cc1101_read_burst_(reg::FIFO, buf, want);
  uint8_t rssi_raw = this->cc1101_read_status_(reg::RSSI);
  int16_t rssi_dbm = rssi_raw_to_dbm_(rssi_raw);

  this->cc1101_enter_rx_();

  if ((now - this->last_packet_ms_) < 250) {
    // Bresser repeats the same frame within a single TX burst — drop dupes.
    return;
  }
  this->last_packet_ms_ = now;
  this->total_packets_++;
  this->handle_frame_(buf, want, rssi_dbm);
}

void BresserWeather::handle_frame_(uint8_t *raw, uint8_t length,
                                   int16_t rssi_dbm) {
  const RadioPreset &cur = PRESETS[this->current_preset_idx_];

  // Hex dump
  char hex[3 * 64 + 1] = {0};
  for (uint8_t i = 0; i < length; ++i) snprintf(hex + i * 3, 4, "%02X ", raw[i]);
  ESP_LOGI(TAG, "[PKT cfg=%s n=%u rssi=%ddBm] HEX: %s", cur.name,
           this->total_packets_, rssi_dbm, hex);

  // Hunt for a 0x2DD4 sync word inside the payload (helps when scan ran
  // without sync detection — we capture raw bits and locate the frame).
  int sync_bit = find_sync_bit_(raw, length, 0x2DD4);
  if (sync_bit >= 0) {
    ESP_LOGI(TAG, "[PKT cfg=%s] Sync 0x2DD4 found at bit=%d (byte=%d.%d)",
             cur.name, sync_bit, sync_bit / 8, sync_bit % 8);
  }

  // Diff vs previous packet
  this->log_packet_diff_(raw, length);

  // Try standard 7-in-1 decode against the captured payload directly. If the
  // frame was received with sync detection, this is the canonical path.
  bool decoded = this->decode_bresser_7in1_(raw, length, rssi_dbm,
                                            this->scan_mode_);

  // If sync was at a non-byte-aligned offset (typical when capturing raw
  // bits), also try the bit-shifted variant.
  if (!decoded && sync_bit > 0 && (sync_bit + 16 + 25 * 8) <= (int) (length * 8)) {
    uint8_t shifted[27] = {0};
    int data_bit = sync_bit + 16;
    for (int i = 0; i < 25 && (data_bit / 8) < length; ++i) {
      uint16_t hi = raw[data_bit / 8] << 8;
      if ((data_bit / 8 + 1) < length) hi |= raw[data_bit / 8 + 1];
      shifted[i] = (hi >> (8 - (data_bit % 8))) & 0xFF;
      data_bit += 8;
    }
    ESP_LOGD(TAG, "[PKT cfg=%s] Trying bit-shifted decode at offset %d",
             cur.name, sync_bit + 16);
    decoded = this->decode_bresser_7in1_(shifted, 25, rssi_dbm, true);
  }

  this->publish_raw_(raw, length, rssi_dbm, sync_bit);

  if (!decoded && this->log_unknown_) {
    ESP_LOGW(TAG, "[PKT cfg=%s n=%u] Undecoded frame", cur.name,
             this->total_packets_);
  }
}

bool BresserWeather::decode_bresser_7in1_(const uint8_t *raw, uint8_t length,
                                          int16_t rssi_dbm, bool from_scan) {
  if (length < 25) return false;

  uint8_t msg[27];
  memcpy(msg, raw, std::min<uint8_t>(length, 27));
  for (uint8_t i = 0; i < 25; ++i) msg[i] ^= 0xAA;

  uint16_t chk = ((uint16_t) msg[0] << 8) | msg[1];
  uint16_t digest = lfsr_digest16_(&msg[2], 23, 0x8810, 0xBA95);
  uint16_t xor_result = chk ^ digest;
  bool crc_ok = (xor_result == 0x6DF1);

  ESP_LOGI(TAG, "[PKT] CRC: %s (chk=0x%04X digest=0x%04X xor=0x%04X expected=0x6DF1)",
           crc_ok ? "OK" : "FAIL", chk, digest, xor_result);

  if (!crc_ok) {
    if (from_scan) {
      // In scan mode: log decoded fields anyway — useful for tuning.
      ESP_LOGD(TAG, "[PKT] CRC failed but logging would-be fields below for inspection");
    } else {
      return false;
    }
  }

  uint8_t s_type = msg[6] >> 4;
  uint8_t channel = msg[6] & 0x07;
  uint16_t sensor_id = ((uint16_t) msg[2] << 8) | msg[3];

  int wdir_raw = (msg[4] >> 4) * 100 + (msg[4] & 0x0F) * 10 + (msg[5] >> 4);
  int wavg_raw = (msg[8] & 0x0F) * 100 + (msg[9] >> 4) * 10 + (msg[9] & 0x0F);
  int temp_raw = (msg[14] >> 4) * 100 + (msg[14] & 0x0F) * 10 + (msg[15] >> 4);
  int humidity = (msg[16] >> 4) * 10 + (msg[16] & 0x0F);
  int rain_raw = (msg[10] >> 4) * 100000 + (msg[10] & 0x0F) * 10000 +
                 (msg[11] >> 4) * 1000 + (msg[11] & 0x0F) * 100 +
                 (msg[12] >> 4) * 10 + (msg[12] & 0x0F);
  int light_raw = (msg[17] >> 4) * 100000 + (msg[17] & 0x0F) * 10000 +
                  (msg[18] >> 4) * 1000 + (msg[18] & 0x0F) * 100 +
                  (msg[19] >> 4) * 10 + (msg[19] & 0x0F);
  int uv_raw = (msg[20] >> 4) * 100 + (msg[20] & 0x0F) * 10 + (msg[21] >> 4);

  float temp_c = (temp_raw > 600) ? (temp_raw - 1000) * 0.1f : temp_raw * 0.1f;
  float wavg_kmh = wavg_raw * 0.1f * 3.6f;
  float rain_mm = rain_raw * 0.1f;
  float uv_idx = uv_raw * 0.1f;

  bool plaus_temp = (temp_c >= -50.0f && temp_c <= 80.0f);
  bool plaus_hum = (humidity >= 0 && humidity <= 100);
  bool plaus_wdir = (wdir_raw >= 0 && wdir_raw <= 360);
  bool plaus_wavg = (wavg_kmh >= 0.0f && wavg_kmh <= 200.0f);

  ESP_LOGI(TAG,
           "[PKT] decode id=0x%04X ch=%u s_type=0x%X T=%.1f°C[%s] RH=%d%%[%s] "
           "Wavg=%.1fkm/h[%s] Wdir=%d°[%s] Rain=%.1fmm UV=%.1f Lux=%d",
           sensor_id, channel, s_type, temp_c, plaus_temp ? "OK" : "OOR",
           humidity, plaus_hum ? "OK" : "OOR", wavg_kmh,
           plaus_wavg ? "OK" : "OOR", wdir_raw, plaus_wdir ? "OK" : "OOR",
           rain_mm, uv_idx, light_raw);

  bool is_weather =
      (s_type == 0x01 || s_type == 0x03 || s_type == 0x04 || s_type == 0x08);
  if (!crc_ok || !is_weather) return false;
  if (!plaus_temp || !plaus_hum) {
    ESP_LOGW(TAG, "[PKT] CRC OK but values implausible - rejecting");
    return false;
  }

  this->valid_packets_++;
  this->publish_decoded_(temp_c, (float) humidity, wavg_kmh, (float) wdir_raw,
                         rain_mm, uv_idx, (float) light_raw, NAN, rssi_dbm);
  return true;
}

void BresserWeather::log_packet_diff_(const uint8_t *raw, uint8_t length) {
  if (this->prev_payload_len_ > 0 && this->prev_payload_len_ == length) {
    int identical = 0;
    int diff_pos[8] = {0};
    int diff_n = 0;
    for (uint8_t i = 0; i < length; ++i) {
      if (raw[i] == this->prev_payload_[i]) {
        identical++;
      } else if (diff_n < 8) {
        diff_pos[diff_n++] = i;
      }
    }
    char dpos[64] = {0};
    for (int i = 0; i < diff_n; ++i) {
      char tmp[6];
      snprintf(tmp, sizeof(tmp), "%d%s", diff_pos[i], i + 1 < diff_n ? "," : "");
      strncat(dpos, tmp, sizeof(dpos) - strlen(dpos) - 1);
    }
    ESP_LOGD(TAG, "[PKT] diff vs prev: %d/%d identical, changed at [%s]",
             identical, length, dpos);
  }
  memcpy(this->prev_payload_, raw, std::min<uint8_t>(length, 64));
  this->prev_payload_len_ = length;
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
int16_t BresserWeather::rssi_raw_to_dbm_(uint8_t raw) {
  int16_t v = (raw >= 128) ? ((int16_t) raw - 256) : (int16_t) raw;
  return v / 2 - 74;
}

uint16_t BresserWeather::lfsr_digest16_(const uint8_t *message, unsigned bytes,
                                        uint16_t gen, uint16_t key) {
  uint16_t sum = 0;
  for (unsigned k = 0; k < bytes; ++k) {
    uint8_t data = message[k];
    for (int i = 7; i >= 0; --i) {
      if ((data >> i) & 1) sum ^= key;
      if (key & 1) {
        key = (key >> 1) ^ gen;
      } else {
        key = (key >> 1);
      }
    }
  }
  return sum;
}

int BresserWeather::find_sync_bit_(const uint8_t *raw, uint8_t length,
                                   uint16_t sync_word) {
  if (length < 2) return -1;
  uint32_t window = 0;
  int total_bits = length * 8;
  for (int b = 0; b < total_bits; ++b) {
    uint8_t bit = (raw[b / 8] >> (7 - (b % 8))) & 1;
    window = ((window << 1) | bit) & 0xFFFF;
    if (b >= 15 && (uint16_t) window == sync_word) {
      return b - 15;
    }
  }
  return -1;
}

void BresserWeather::publish_raw_(const uint8_t *raw, uint8_t length,
                                  int16_t rssi_dbm, int sync_offset_bit) {
  if (this->raw_dump_topic_.empty()) return;
#ifdef USE_MQTT
  if (mqtt::global_mqtt_client == nullptr) return;
  char hex[3 * 64 + 1] = {0};
  for (uint8_t i = 0; i < length; ++i) snprintf(hex + i * 2, 3, "%02X", raw[i]);
  char payload[512];
  const RadioPreset &cur = PRESETS[this->current_preset_idx_];
  snprintf(payload, sizeof(payload),
           "{\"cfg\":\"%s\",\"freq_hz\":%u,\"rssi_dbm\":%d,\"ts\":%u,"
           "\"len\":%u,\"sync_bit\":%d,\"hex\":\"%s\"}",
           cur.name, (unsigned) cur.freq_hz, rssi_dbm, (unsigned) (millis() / 1000),
           length, sync_offset_bit, hex);
  mqtt::global_mqtt_client->publish(this->raw_dump_topic_,
                                    std::string(payload));
#else
  (void) raw;
  (void) length;
  (void) rssi_dbm;
  (void) sync_offset_bit;
#endif
}

void BresserWeather::publish_decoded_(float t, float h, float wkmh, float wdir,
                                      float rmm, float uv, float lux, float p,
                                      int16_t rssi_dbm) {
  for (auto *s : this->sensors_) {
    if (s->temperature_ && !std::isnan(t)) s->temperature_->publish_state(t);
    if (s->humidity_ && !std::isnan(h)) s->humidity_->publish_state(h);
    if (s->wind_speed_ && !std::isnan(wkmh)) s->wind_speed_->publish_state(wkmh);
    if (s->wind_direction_ && !std::isnan(wdir)) s->wind_direction_->publish_state(wdir);
    if (s->rain_total_ && !std::isnan(rmm)) s->rain_total_->publish_state(rmm);
    if (s->uv_index_ && !std::isnan(uv)) s->uv_index_->publish_state(uv);
    if (s->light_lux_ && !std::isnan(lux)) s->light_lux_->publish_state(lux);
    if (s->pressure_ && !std::isnan(p)) s->pressure_->publish_state(p);
    if (s->rssi_) s->rssi_->publish_state(rssi_dbm);
  }
}

// ---------------------------------------------------------------------------
// dump_config
// ---------------------------------------------------------------------------
void BresserWeather::dump_config() {
  ESP_LOGCONFIG(TAG, "Bresser Weather (CC1101):");
  ESP_LOGCONFIG(TAG, "  MOSI=%d MISO=%d CLK=%d CS=%d GDO0=%d GDO2=%d",
                mosi_pin_, miso_pin_, clk_pin_, cs_pin_, gdo0_pin_, gdo2_pin_);
  ESP_LOGCONFIG(TAG, "  Frequency: %.3f MHz", configured_freq_hz_ / 1e6f);
  ESP_LOGCONFIG(TAG, "  SPI mode: %s @ %u Hz",
                bitbang_spi_ ? "bit-bang" : "hardware",
                (unsigned) spi_clock_hz_);
  ESP_LOGCONFIG(TAG, "  Log unknown frames: %s", YESNO(log_unknown_));
  ESP_LOGCONFIG(TAG, "  Scan mode: %s (interval=%u ms)", YESNO(scan_mode_),
                (unsigned) scan_interval_ms_);
  ESP_LOGCONFIG(TAG, "  Status interval: %u ms", (unsigned) status_interval_ms_);
  if (!raw_dump_topic_.empty())
    ESP_LOGCONFIG(TAG, "  Raw dump MQTT topic: %s", raw_dump_topic_.c_str());
  ESP_LOGCONFIG(TAG, "  Radio ready: %s", YESNO(radio_ready_));
  ESP_LOGCONFIG(TAG, "  Sensor listeners: %u", (unsigned) sensors_.size());
}

void BresserWeatherSensor::dump_config() {
  ESP_LOGCONFIG(TAG, "Bresser Weather Sensor:");
  LOG_SENSOR("  ", "Temperature", temperature_);
  LOG_SENSOR("  ", "Humidity", humidity_);
  LOG_SENSOR("  ", "Wind speed", wind_speed_);
  LOG_SENSOR("  ", "Wind direction", wind_direction_);
  LOG_SENSOR("  ", "Rain total", rain_total_);
  LOG_SENSOR("  ", "UV index", uv_index_);
  LOG_SENSOR("  ", "Light", light_lux_);
  LOG_SENSOR("  ", "Pressure", pressure_);
  LOG_SENSOR("  ", "RSSI", rssi_);
}

}  // namespace bresser_weather
}  // namespace esphome
