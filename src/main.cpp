#include <Adafruit_AHTX0.h>
#include <MideaHeatpumpIR.h>

#include "HomeSpan.h"

Adafruit_AHTX0 aht;
IRSenderPWM irSender(14);

class DevThermostat : public Service::Thermostat
{
    sensors_event_t temp_sensor;
    sensors_event_t humidity_sensor;
    HeatpumpIR *heatpumpIR;

    SpanCharacteristic *currentState;
    SpanCharacteristic *targetState;
    SpanCharacteristic *currentTemp;
    SpanCharacteristic *targetTemp;
    SpanCharacteristic *currentHumidity;
    SpanCharacteristic *unit;

  public:
    DevThermostat()
    {
        aht.getEvent(&humidity_sensor, &temp_sensor);
        heatpumpIR = new MideaHeatpumpIR();

        currentState = new Characteristic::CurrentHeatingCoolingState(0);
        targetState = new Characteristic::TargetHeatingCoolingState();

        currentTemp = new Characteristic::CurrentTemperature(temp_sensor.temperature);
        currentTemp->setRange(-50, 100);
        targetTemp = new Characteristic::TargetTemperature();
        targetTemp->setRange(16, 30, 0.5);
        currentHumidity = new Characteristic::CurrentRelativeHumidity(humidity_sensor.relative_humidity);

        unit = new Characteristic::TemperatureDisplayUnits(0);
    }

    void loop() override
    {
        if (currentTemp->timeVal() > 5000) {
            aht.getEvent(&humidity_sensor, &temp_sensor);

            currentTemp->setVal(temp_sensor.temperature);
            currentHumidity->setVal(humidity_sensor.relative_humidity);
        }
    }

    boolean update() override
    {
        int state = targetState->getNewVal();
        float temp = targetTemp->getNewVal();

        // // Save to EEPROM.
        // ACState.put(0, state);
        // ACState.commit();
        // ACTemp.put(0, temp);
        // ACTemp.commit();

        // if (state == 0) {
        //     IRACSwitch.put(0, 0);
        //     ac.off();
        // } else {
        //     IRACSwitch.put(0, 1);
        //     ac.on();
        //     previousACState = state;
        // }

        // switch (state) {
        //     case 0:
        //         break;
        //     case 1:
        //         state = kPanasonicAcHeat;
        //         currentState->setVal(state);
        //         break;
        //     case 2:
        //         state = kPanasonicAcCool;
        //         currentState->setVal(state);
        //         break;
        //     default:
        //         state = kPanasonicAcAuto;
        //         // Setting the current state to auto cause the device to stop responding.
        //         currentState->setVal(0);
        //         break;
        // }

        // ac.setMode(state);
        // IRACState.put(0, state);

        // ac.setFan(kPanasonicAcFanAuto);
        // ac.setSwingVertical(kPanasonicAcSwingVAuto);
        // ac.setSwingHorizontal(kPanasonicAcSwingHAuto);
        // ac.setTemp(temp);
        // ac.send();
        // printState();
        return (true);
    }
};

String getID()
{
#if defined(ESP8266)
    String id(ESP.getChipId());
#elif defined(ESP32)
    String id((uint32_t)ESP.getEfuseMac(), HEX);
#endif
    id.toUpperCase();
    return id;
}

void setup()
{
    Serial.begin(115200);
    Wire.begin(16, 17);

    if (!aht.begin()) {
        Serial.println("Could not find AHT? Check wiring");
        while (1)
            delay(10);
    }

    homeSpan.setPairingCode("97654321");
    homeSpan.setApSSID("MothHomeSetup");
    homeSpan.setApPassword("97654321");
    homeSpan.enableOTA("97654321");

    homeSpan.begin(Category::AirConditioners, "Moth AirConditioner");

    new SpanAccessory();
    new Service::AccessoryInformation();
    new Characteristic::Manufacturer("MOTH");
    new Characteristic::SerialNumber(getID().c_str());
    new Characteristic::Model("A2");
    new Characteristic::FirmwareRevision(FIRMWARE_VERSION);
    new Characteristic::HardwareRevision(HARDWARE_VERSION);
    new Characteristic::Identify();

    new DevThermostat();
}

void loop()
{
    homeSpan.poll();
}
