"""ESPHome external component: bresser_weather.

Hub-style component that drives a TI CC1101 radio over SPI for FSK reception
of Bresser 7-in-1 / VEVOR YT60231 weather station packets at 868.30 MHz.
Sensor entities are registered separately under ``sensor:`` (see sensor.py).

The default radio profile uses the canonical Bresser-7in1 modulation:
8.21 kbps / 57 kHz deviation / 270 kHz RX bandwidth / sync 0x2DD4. These
match what matthias-bs/BresserWeatherSensorReceiver uses against real
Bresser hardware. ``scan_mode`` cycles through a fallback list of profiles
so weak/unknown variants can still be diagnosed in the field.
"""
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.const import CONF_FREQUENCY, CONF_ID

CODEOWNERS = ["@ryszardlet"]
DEPENDENCIES = ["esp32"]
AUTO_LOAD = ["sensor"]
MULTI_CONF = False

CONF_MOSI_PIN = "mosi_pin"
CONF_MISO_PIN = "miso_pin"
CONF_CLK_PIN = "clk_pin"
CONF_CS_PIN = "cs_pin"
CONF_GDO0_PIN = "gdo0_pin"
CONF_GDO2_PIN = "gdo2_pin"
CONF_LOG_UNKNOWN = "log_unknown"
CONF_SCAN_MODE = "scan_mode"
CONF_SCAN_INTERVAL = "scan_interval"
CONF_RAW_DUMP_TOPIC = "raw_dump_topic"
CONF_STATUS_INTERVAL = "status_interval"
CONF_SPI_MODE = "spi_mode"
CONF_SPI_CLOCK_HZ = "spi_clock_hz"

bresser_weather_ns = cg.esphome_ns.namespace("bresser_weather")
BresserWeather = bresser_weather_ns.class_("BresserWeather", cg.Component)


def _frequency_mhz(value):
    if isinstance(value, str):
        s = value.strip().lower().replace("mhz", "").strip()
        try:
            mhz = float(s)
        except ValueError as err:
            raise cv.Invalid(f"Invalid frequency: {value!r}") from err
    else:
        mhz = float(value)
    if not 300.0 <= mhz <= 928.0:
        raise cv.Invalid("frequency must be a CC1101 RF band value in MHz")
    return int(round(mhz * 1_000_000))


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(BresserWeather),
        cv.Required(CONF_MOSI_PIN): pins.internal_gpio_output_pin_number,
        cv.Required(CONF_MISO_PIN): pins.internal_gpio_input_pin_number,
        cv.Required(CONF_CLK_PIN): pins.internal_gpio_output_pin_number,
        cv.Required(CONF_CS_PIN): pins.internal_gpio_output_pin_number,
        cv.Required(CONF_GDO0_PIN): pins.internal_gpio_input_pin_number,
        cv.Optional(CONF_GDO2_PIN): pins.internal_gpio_input_pin_number,
        cv.Optional(CONF_FREQUENCY, default="868.300"): _frequency_mhz,
        cv.Optional(CONF_LOG_UNKNOWN, default=True): cv.boolean,
        cv.Optional(CONF_SCAN_MODE, default=False): cv.boolean,
        cv.Optional(
            CONF_SCAN_INTERVAL, default="30s"
        ): cv.positive_time_period_milliseconds,
        cv.Optional(
            CONF_STATUS_INTERVAL, default="10s"
        ): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_RAW_DUMP_TOPIC, default=""): cv.string,
        cv.Optional(CONF_SPI_MODE, default="bitbang"): cv.one_of(
            "hardware", "bitbang", lower=True
        ),
        cv.Optional(CONF_SPI_CLOCK_HZ, default=100_000): cv.positive_int,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    cg.add_library("SPI", None)

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_pins(
        config[CONF_MOSI_PIN],
        config[CONF_MISO_PIN],
        config[CONF_CLK_PIN],
        config[CONF_CS_PIN],
        config[CONF_GDO0_PIN],
        config.get(CONF_GDO2_PIN, -1),
    ))
    cg.add(var.set_frequency_hz(config[CONF_FREQUENCY]))
    cg.add(var.set_log_unknown(config[CONF_LOG_UNKNOWN]))
    cg.add(var.set_scan_mode(config[CONF_SCAN_MODE]))
    cg.add(var.set_scan_interval_ms(int(config[CONF_SCAN_INTERVAL].total_milliseconds)))
    cg.add(var.set_status_interval_ms(int(config[CONF_STATUS_INTERVAL].total_milliseconds)))
    cg.add(var.set_raw_dump_topic(config[CONF_RAW_DUMP_TOPIC]))
    cg.add(var.set_bitbang_spi(config[CONF_SPI_MODE] == "bitbang"))
    cg.add(var.set_spi_clock_hz(config[CONF_SPI_CLOCK_HZ]))
