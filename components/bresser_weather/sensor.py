"""Sensor platform for bresser_weather.

Registers up to nine ESPHome sensor entities populated from a decoded
Bresser 7-in-1 telegram: temperature, humidity, wind speed, wind direction,
rain total, UV index, light level, pressure (when carried by the variant)
and the radio RSSI of the last frame.
"""
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_HUMIDITY,
    CONF_ID,
    CONF_PRESSURE,
    CONF_TEMPERATURE,
    DEVICE_CLASS_ATMOSPHERIC_PRESSURE,
    DEVICE_CLASS_HUMIDITY,
    DEVICE_CLASS_ILLUMINANCE,
    DEVICE_CLASS_PRECIPITATION,
    DEVICE_CLASS_SIGNAL_STRENGTH,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_WIND_SPEED,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_CELSIUS,
    UNIT_DECIBEL_MILLIWATT,
    UNIT_DEGREES,
    UNIT_HECTOPASCAL,
    UNIT_KILOMETER_PER_HOUR,
    UNIT_LUX,
    UNIT_MILLIMETER,
    UNIT_PERCENT,
)

from . import BresserWeather, bresser_weather_ns

CONF_BRESSER_WEATHER_ID = "bresser_weather_id"
CONF_WIND_SPEED = "wind_speed"
CONF_WIND_DIRECTION = "wind_direction"
CONF_RAIN_TOTAL = "rain_total"
CONF_UV_INDEX = "uv_index"
CONF_LIGHT_LUX = "light_lux"
CONF_RSSI = "rssi"

DEPENDENCIES = ["bresser_weather"]

UNIT_UV_INDEX = "UV index"

BresserWeatherSensor = bresser_weather_ns.class_(
    "BresserWeatherSensor", cg.Component
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(BresserWeatherSensor),
        cv.GenerateID(CONF_BRESSER_WEATHER_ID): cv.use_id(BresserWeather),
        cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_HUMIDITY): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_HUMIDITY,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_WIND_SPEED): sensor.sensor_schema(
            unit_of_measurement=UNIT_KILOMETER_PER_HOUR,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_WIND_SPEED,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_WIND_DIRECTION): sensor.sensor_schema(
            unit_of_measurement=UNIT_DEGREES,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_RAIN_TOTAL): sensor.sensor_schema(
            unit_of_measurement=UNIT_MILLIMETER,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_PRECIPITATION,
            state_class=STATE_CLASS_TOTAL_INCREASING,
        ),
        cv.Optional(CONF_UV_INDEX): sensor.sensor_schema(
            unit_of_measurement=UNIT_UV_INDEX,
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_LIGHT_LUX): sensor.sensor_schema(
            unit_of_measurement=UNIT_LUX,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_ILLUMINANCE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_PRESSURE): sensor.sensor_schema(
            unit_of_measurement=UNIT_HECTOPASCAL,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_ATMOSPHERIC_PRESSURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_RSSI): sensor.sensor_schema(
            unit_of_measurement=UNIT_DECIBEL_MILLIWATT,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_SIGNAL_STRENGTH,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    parent = await cg.get_variable(config[CONF_BRESSER_WEATHER_ID])
    cg.add(parent.register_sensor(var))

    if CONF_TEMPERATURE in config:
        s = await sensor.new_sensor(config[CONF_TEMPERATURE])
        cg.add(var.set_temperature_sensor(s))
    if CONF_HUMIDITY in config:
        s = await sensor.new_sensor(config[CONF_HUMIDITY])
        cg.add(var.set_humidity_sensor(s))
    if CONF_WIND_SPEED in config:
        s = await sensor.new_sensor(config[CONF_WIND_SPEED])
        cg.add(var.set_wind_speed_sensor(s))
    if CONF_WIND_DIRECTION in config:
        s = await sensor.new_sensor(config[CONF_WIND_DIRECTION])
        cg.add(var.set_wind_direction_sensor(s))
    if CONF_RAIN_TOTAL in config:
        s = await sensor.new_sensor(config[CONF_RAIN_TOTAL])
        cg.add(var.set_rain_total_sensor(s))
    if CONF_UV_INDEX in config:
        s = await sensor.new_sensor(config[CONF_UV_INDEX])
        cg.add(var.set_uv_index_sensor(s))
    if CONF_LIGHT_LUX in config:
        s = await sensor.new_sensor(config[CONF_LIGHT_LUX])
        cg.add(var.set_light_lux_sensor(s))
    if CONF_PRESSURE in config:
        s = await sensor.new_sensor(config[CONF_PRESSURE])
        cg.add(var.set_pressure_sensor(s))
    if CONF_RSSI in config:
        s = await sensor.new_sensor(config[CONF_RSSI])
        cg.add(var.set_rssi_sensor(s))
