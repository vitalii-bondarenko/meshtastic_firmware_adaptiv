#pragma once

#include "configuration.h"

#if !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR

#include "../mesh/generated/meshtastic/telemetry.pb.h"
#include "TelemetrySensor.h"
#include <ModbusMaster.h>
#include <HardwareSerial.h>

class RS485WindSensor : public TelemetrySensor
{
  private:
    ModbusMaster node;

    static void preTransmission();
    static void postTransmission();

  public:
    RS485WindSensor();
    virtual int32_t runOnce() override;
    virtual bool getMetrics(meshtastic_Telemetry *measurement) override;
};

#endif
