# esphome-bresser-weather

ESPHome external component that receives **Bresser 7-in-1** / **VEVOR YT60231 / SucceBuy** weather-station packets at **868.30 MHz** using a **TI CC1101** sub-GHz radio attached to an ESP32 over SPI, decodes them in firmware, and publishes the measurements as ESPHome sensors.

The decoder is a port of [`rtl_433`'s `bresser_7in1.c`](https://github.com/merbanan/rtl_433/blob/master/src/devices/bresser_7in1.c). The CC1101 modulation parameters match what [`matthias-bs/BresserWeatherSensorReceiver`](https://github.com/matthias-bs/BresserWeatherSensorReceiver) uses against real Bresser hardware.

## Radio profile (canonical)

| Parameter         | Value                                           |
| ----------------- | ----------------------------------------------- |
| Frequency         | 868.30 MHz                                      |
| Modulation        | 2-FSK                                           |
| Bit rate          | **8.21 kbps** (NOT 17.241 — common mistake)     |
| Frequency dev.    | **57.136 kHz** (NOT 40 kHz)                     |
| Channel BW        | 270 kHz                                         |
| Sync word         | `0x2DD4` (16/16 match)                          |
| Whitening         | XOR 0xAA over the 25-byte payload               |
| Integrity check   | LFSR-16 digest, gen 0x8810, key 0xBA95, xor 0x6DF1 |
| TX interval       | every 20 s                                      |

## Supported hardware

| Side        | Hardware                                                |
| ----------- | ------------------------------------------------------- |
| MCU         | ESP32 (DevKit, WROOM-32, WROVER, …)                     |
| Radio       | CC1101 module (E07-M1101D, CMT-CC1101, generic 868 MHz) |
| Sensor (TX) | VEVOR YT60231 / SucceBuy 7-in-1 / Bresser 7-in-1        |

## Wiring

| CC1101 pin | ESP32 GPIO | YAML key   | Note                            |
| ---------- | ---------- | ---------- | ------------------------------- |
| VCC        | 3V3        | —          | **3.3 V only — do NOT use 5 V** |
| GND        | GND        | —          |                                 |
| SI / MOSI  | GPIO23     | `mosi_pin` |                                 |
| SO / MISO  | GPIO19     | `miso_pin` |                                 |
| SCK        | GPIO18     | `clk_pin`  |                                 |
| CSN        | GPIO5      | `cs_pin`   |                                 |
| GDO0       | GPIO4      | `gdo0_pin` | sync-detect status              |
| GDO2       | GPIO27     | `gdo2_pin` | optional, mirrored status       |

## Configuration

```yaml
external_components:
  - source: github://ryszardlet/esphome-bresser-weather@main
    components: [ bresser_weather ]

bresser_weather:
  mosi_pin: GPIO23
  miso_pin: GPIO19
  clk_pin: GPIO18
  cs_pin: GPIO5
  gdo0_pin: GPIO4
  gdo2_pin: GPIO27
  frequency: 868.30
  log_unknown: true       # log undecoded frames
  scan_mode: false        # cycle through radio presets (diagnostic)
  scan_interval: 30s
  status_interval: 10s    # heartbeat log line
  raw_dump_topic: ""      # publish every packet to MQTT (optional)

sensor:
  - platform: bresser_weather
    temperature:
      name: "Outdoor temperature"
    humidity:
      name: "Outdoor humidity"
    wind_speed:
      name: "Wind speed"
    wind_direction:
      name: "Wind direction"
    rain_total:
      name: "Rainfall total"
    uv_index:
      name: "UV index"
    light_lux:
      name: "Light intensity"
    rssi:
      name: "Outdoor RSSI"
```

A complete example is at [`example/weather_station.yaml`](example/weather_station.yaml).

## Sensors exposed

| Key             | Unit       | Accuracy | Notes                                         |
| --------------- | ---------- | -------- | --------------------------------------------- |
| `temperature`   | °C         | 0.1      | Negative values handled (`raw - 1000`)        |
| `humidity`      | %          | 0        | 2-digit BCD                                   |
| `wind_speed`    | km/h       | 0.1      | Converted from m/s                            |
| `wind_direction`| °          | 0        | 0-360                                         |
| `rain_total`    | mm         | 0.1      | Cumulative — `state_class: total_increasing`  |
| `uv_index`      | UV index   | 0.1      |                                               |
| `light_lux`     | lx         | 0        | 6-digit BCD                                   |
| `pressure`      | hPa        | 0.1      | **Not transmitted by the outdoor sensor**     |
| `rssi`          | dBm        | 0        | Diagnostic                                    |

> **Pressure note.** The Bresser/VEVOR outdoor sensor does NOT transmit barometric pressure — it's measured by the indoor base station's internal BMP. The `pressure:` schema entry exists for forward compatibility but the decoder will never populate it.

## Diagnostic features

### `log_unknown: true`

Every received frame that fails CRC or sensor-type validation is logged with a hex dump and the CRC math (`chk` vs `digest` vs xor result). Set to `false` after first successful packet.

### `scan_mode: true` — find the right radio profile

When you can't get any frames, enable this. The component cycles through 8 radio profiles every `scan_interval` and logs which one (if any) yields valid CRCs:

| Preset            | Bitrate     | Dev.     | BW      | Sync     | Use case                       |
| ----------------- | ----------- | -------- | ------- | -------- | ------------------------------ |
| `A_canonical`     | 8.21 kbps   | 57 kHz   | 270 kHz | 0x2DD4   | Default — real Bresser HW      |
| `B_aa_preamble`   | 8.21 kbps   | 57 kHz   | 270 kHz | 0xAAAA   | Lock on preamble, dump 64 B    |
| `C_17_2kbps`      | 17.241 kbps | 40 kHz   | 203 kHz | 0x2DD4   | Alt rate often quoted online   |
| `D_18_8kbps`      | 18.868 kbps | 40 kHz   | 203 kHz | 0x2DD4   | Alt rate from older specs      |
| `E_aa2d_sync`     | 8.21 kbps   | 57 kHz   | 270 kHz | 0xAA2D   | Alt sync (preamble→sync edge)  |
| `F_868_35MHz`     | 8.21 kbps   | 57 kHz   | 270 kHz | 0x2DD4   | Slight freq offset             |
| `G_4_8kbps_aa`    | 4.8 kbps    | 9.6 kHz  | 58 kHz  | 0xAAAA   | Cheap-clone alternative        |
| `H_9_6kbps_aa`    | 9.6 kbps    | 40 kHz   | 203 kHz | 0xAAAA   | Generic FSK probe              |

The `_aa*` and `B_aa_preamble` presets use sync `0xAAAA` which matches the preamble bytes the sensor transmits. The component then runs `find_sync_bit_()` on the captured FIFO to locate the real `0x2DD4` and bit-shift the payload before decoding.

Watch the log for `[PKT cfg=A_canonical n=N] HEX: ...` followed by `[PKT] CRC: OK` — that's the working preset.

### `status_interval: 10s` — heartbeat

Every interval the component logs:

```
[bresser_weather] Waiting... uptime=42s cfg=default rx=0/0 marc=0x0D rxbytes=0 pktstatus=0x00 rssi=-117dBm gdo0=LOW
```

`marc=0x0D` is RX state (correct). `rxbytes>0` means data is in the FIFO. `gdo0=HIGH` would mean a sync was detected.

### `raw_dump_topic: "bresser/raw"` — MQTT off-device analysis

Requires the `mqtt:` ESPHome component to be configured. Every received frame publishes a JSON blob:

```json
{"cfg":"A_canonical","freq_hz":868300000,"rssi_dbm":-114,"ts":120,"len":26,"sync_bit":-1,"hex":"7234D63E92B94C3F94D8...."}
```

You can subscribe with `mosquitto_sub -t bresser/raw` and feed the hex into Python for offline decoding.

## Troubleshooting

**No `CC1101 PARTNUM=0x00 VERSION=0x14` line at boot.** SPI wiring problem. The component will mark itself failed and stop. Check 3.3 V power, MISO not swapped with MOSI, and CS active-low.

**`PARTNUM`/`VERSION` line appears but `MARCSTATE != 0x0D` in heartbeat.** The chip isn't entering RX. Usually a frequency calibration issue — try `scan_mode: true` so the loop re-applies presets and re-strobes RX every interval.

**Heartbeat shows `rxbytes=0` and `gdo0=LOW` for minutes.** No sync word matched. Try `scan_mode: true`. If preset `B_raw_dump` collects bytes (rxbytes goes up), the radio is working but the sync word is wrong — check the hex dumps for a 0x2DD4 pattern at any bit offset.

**`[PKT] CRC: FAIL` repeatedly.** The frames are being captured but the bit alignment or whitening is off. The component already tries a bit-shifted re-decode; if that still fails, the variant is non-standard. Send a few `raw_dump_topic` JSON captures to a maintainer.

**RSSI very low (-115 dBm or weaker).** Check the antenna — most CC1101 modules ship without one. A 17 cm wire on the ANT pin works for 868 MHz. Concrete walls/metal roofing kill the signal.

## Project structure

```
esphome-bresser-weather/
├── README.md
├── components/
│   └── bresser_weather/
│       ├── __init__.py
│       ├── sensor.py
│       ├── bresser_weather.h
│       └── bresser_weather.cpp
└── example/
    └── weather_station.yaml
```

## Credits

- [`rtl_433`](https://github.com/merbanan/rtl_433) — reference 7-in-1 decoder
- [`matthias-bs/BresserWeatherSensorReceiver`](https://github.com/matthias-bs/BresserWeatherSensorReceiver) — verified CC1101 modulation parameters
- TI CC1101 datasheet (SWRS061I) — register definitions

## License

MIT
