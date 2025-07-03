#include "configuration.h"

#if !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR

#include "../mesh/generated/meshtastic/telemetry.pb.h"
#include "RS485WindSensor.h"
#include "TelemetrySensor.h"

#define WIND_TX_PIN 45
#define WIND_RX_PIN 46
#define WIND_DE_RE_PIN 4
#define WIND_SLAVE_ID 2
#define WIND_BAUD 9600

static HardwareSerial windSerial(2);

static void RS485WindSensorPreTransmission() { digitalWrite(WIND_DE_RE_PIN, HIGH); }
static void RS485WindSensorPostTransmission() { digitalWrite(WIND_DE_RE_PIN, LOW); }

RS485WindSensor::RS485WindSensor()
    : TelemetrySensor(meshtastic_TelemetrySensorType_RS485_WIND, "RS485_WIND")
{
    // mark sensor as present so hasSensor() returns true
    nodeTelemetrySensorsMap[sensorType].first = 1;
}

int32_t RS485WindSensor::runOnce()
{
    LOG_INFO("Init sensor: %s", sensorName);

    pinMode(WIND_DE_RE_PIN, OUTPUT);
    digitalWrite(WIND_DE_RE_PIN, LOW);

    windSerial.begin(WIND_BAUD, SERIAL_8N1, WIND_RX_PIN, WIND_TX_PIN);

    node.begin(WIND_SLAVE_ID, windSerial);

    node.preTransmission(RS485WindSensorPreTransmission);
    node.postTransmission(RS485WindSensorPostTransmission);
    status = 1;

    return DEFAULT_SENSOR_MINIMUM_WAIT_TIME_BETWEEN_READS;
}

bool RS485WindSensor::getMetrics(meshtastic_Telemetry *measurement)
{
    measurement->variant.environment_metrics.has_wind_direction = true;

    uint8_t res = node.readHoldingRegisters(0x0000, 1);
    if (res == node.ku8MBSuccess) {
        uint16_t raw = node.getResponseBuffer(0);
        float angle = raw / 10.0f;
        measurement->variant.environment_metrics.wind_direction = static_cast<uint16_t>(angle);
        LOG_INFO("Wind angle: %0.1f deg", angle);
    } else {
        LOG_WARN("Wind reg0 error: %u", res);
    }

    res = node.readHoldingRegisters(0x0001, 1);
    if (res == node.ku8MBSuccess) {
        uint8_t idx = node.getResponseBuffer(0) & 0xFF;
        LOG_INFO("Wind dir index: %u", idx);
    } else {
        LOG_WARN("Wind reg1 error: %u", res);
    }

    return true;
}

#endif
