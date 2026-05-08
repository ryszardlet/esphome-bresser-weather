# esphome-bresser-weather

ESPHome external component that receives **Bresser 7-in-1** / **VEVOR YT60231**
weather-station packets at **868.30 MHz** using a **TI CC1101** sub-GHz radio
attached to an ESP32 over SPI, decodes them in firmware, and publishes the
measurements as ESPHome sensors.

The decoder is a port of [`rtl_433`'s `bresser_7in1.c`](https://github.com/merbanan/rtl_433/blob/master/src/devices/bresser_7in1.c) —
same XOR descrambling, same LFSR-16 digest (`gen=0x8810`, `key=0xBA95`,
final XOR `0x6DF1`), same BCD field layout.

## Supported hardware

| Side              | Hardware                                                |
| ----------------- | ------------------------------------------------------- |
| MCU               | ESP32 (DevKit, WROOM-32, WROVER, …)                     |
| Radio             | CC1101 module (E07-M1101D, CMT-CC1101, generic 868 MHz) |
| Sensor (TX)       | VEVOR YT60231 / Bresser 7-in-1 outdoor unit             |

## Wiring

Default SPI assignment used in the example. Any free GPIOs work — set them
via the YAML.

| CC1101 pin | ESP32 GPIO | Note                                |
| ---------- | ---------- | ----------------------------------- |
| VCC        | 3V3        | **3.3 V only — do not use 5 V**     |
| GND        | GND        |                                     |
| SI / MOSI  | GPIO23     | `mosi_pin`                          |
| SO / MISO  | GPIO19     | `miso_pin`                          |
| SCK        | GPIO18     | `clk_pin`                           |
| CSN        | GPIO5      | `cs_pin`                            |
| GDO0       | GPIO4      | `gdo0_pin` — sync-detect status     |
| GDO2       | GPIO27     | `gdo2_pin` — optional, also status  |

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
  frequency: 868.30        # MHz, optional (default 868.30)
  log_unknown: true        # log frames with bad CRC / wrong sensor type

sensor:
  - platform: bresser_weather
    temperature:
      name: "Temperatura zewnętrzna"
    humidity:
      name: "Wilgotność zewnętrzna"
    wind_speed:
      name: "Prędkość wiatru"
    wind_direction:
      name: "Kierunek wiatru"
    rain_total:
      name: "Suma opadów"
    uv_index:
      name: "Indeks UV"
    light_lux:
      name: "Natężenie światła"
    pressure:
      name: "Ciśnienie"      # only published if your variant carries it
    rssi:
      name: "RSSI stacji"
```

A complete configuration is provided in
[`example/weather_station.yaml`](example/weather_station.yaml).

## Sensors exposed

| Key             | Unit       | Accuracy | Notes                                              |
| --------------- | ---------- | -------- | -------------------------------------------------- |
| `temperature`   | °C         | 0.1      | Negative values handled (`raw - 1000`)             |
| `humidity`      | %          | 0        | Direct BCD from byte 16                            |
| `wind_speed`    | km/h       | 0.1      | Converted from m/s                                 |
| `wind_direction`| °          | 0        | 0 – 360                                            |
| `rain_total`    | mm         | 0.1      | Cumulative — `state_class: total_increasing`       |
| `uv_index`      | UV index   | 0.1      |                                                    |
| `light_lux`     | lx         | 0        | 6-digit BCD                                        |
| `pressure`      | hPa        | 0.1      | Not transmitted by 7-in-1 outdoor unit (see below) |
| `rssi`          | dBm        | 0        | Diagnostic; from CC1101 RSSI register              |

> **Note on pressure.** The standard Bresser 7-in-1 outdoor sensor does not
> transmit barometric pressure — it's measured by the indoor base station.
> The schema accepts `pressure:` so future variants can use the same plumbing,
> but the current decoder leaves it unpublished.

## Radio configuration

| Parameter         | Value                                |
| ----------------- | ------------------------------------ |
| Modulation        | 2-FSK                                |
| Bit rate          | 18.868 kbps (Bresser 7-in-1 native)  |
| Frequency dev.    | ~41 kHz                              |
| Channel BW        | ~270 kHz                             |
| Sync word         | `0x2DD4` (16/16 match)               |
| Whitening         | XOR 0xAA over the 25-byte payload    |
| Integrity check   | LFSR-16 digest (poly 0x8810)         |

## Troubleshooting

**No frames at all.** Confirm the CC1101 is detected at boot — the log line
`CC1101 PARTNUM=0x00 VERSION=0x14` (or similar non-zero version) must appear.
If `VERSION=0x00` or `0xFF`, your wiring is wrong: check 3.3 V power, SCK,
MOSI/MISO swap, and CS.

**Frames received but always undecoded.** Enable `log_unknown: true`. If the
hex dump always starts with `0xAA`-ish noise rather than recognisable BCD
after XOR, the sync word is being missed — check the antenna and the
868.30 MHz frequency setting (some EU variants use 868.35).

**Frames decoded but values look wrong.** Verify the sensor variant — this
component decodes weather variants `s_type ∈ {0x01, 0x03, 0x04, 0x08}`.
Other variants (CO₂, PM2.5, pool thermometer) share the same framing but
have different field layouts; they'll be reported as `not a weather variant`
in the log.

**RX FIFO overflow warnings.** Harmless — they happen when the ESP32 misses
a packet edge under load. The driver re-enters RX automatically.

## Project structure

```
esphome-bresser-weather/
├── README.md
├── components/
│   └── bresser_weather/
│       ├── __init__.py          # ESPHome hub component schema
│       ├── sensor.py            # ESPHome sensor platform schema
│       ├── bresser_weather.h    # C++ class declarations
│       └── bresser_weather.cpp  # CC1101 driver + 7-in-1 decoder
└── example/
    └── weather_station.yaml     # Drop-in ESPHome config
```

## Credits

* [`rtl_433`](https://github.com/merbanan/rtl_433) — reference decoder for
  the Bresser 7-in-1 protocol; this firmware ports the C decoder verbatim.
* [`mathieu-mp/esphome-bresser`](https://github.com/mathieu-mp/esphome-bresser)
  — sibling project; useful cross-reference if you're hacking on the radio
  config.
* TI CC1101 datasheet (SWRS061I) — register values.

## License

MIT
