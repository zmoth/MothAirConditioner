#include <Adafruit_AHTX0.h>
// #include <ir_Midea.h>
// #include <IRremoteESP8266.h>
#include <IRsend.h>

#include "HomeSpan.h"
#include "IRsendMeidi.h"

Adafruit_AHTX0 aht;

IRsendMeidi irsendmeidi(12);

class DevThermostat : public Service::Thermostat
{
    const uint16_t kIrLed = 12;
    // IRMideaAC *ac;
    sensors_event_t temp_sensor;
    sensors_event_t humidity_sensor;

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

        pinMode(kIrLed, OUTPUT);
        // ac = new IRMideaAC(kIrLed);

        irsendmeidi.begin_2();
        irsendmeidi.setZBPL(38);
        irsendmeidi.setCodeTime(500, 1600, 550, 4400, 4400, 5220);

        currentState = new Characteristic::CurrentHeatingCoolingState();
        targetState = new Characteristic::TargetHeatingCoolingState();

        currentTemp = new Characteristic::CurrentTemperature(temp_sensor.temperature);
        currentTemp->setRange(-50, 100);
        targetTemp = new Characteristic::TargetTemperature(26.0, true);
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

        switch (state) {
            case Characteristic::TargetHeatingCoolingState::OFF:
                // ac->off();
                // ac->send();
                irsendmeidi.setPowers(0);
                return (true);
            case Characteristic::TargetHeatingCoolingState::HEAT:
                // ac->on();
                // ac->setMode(kMideaACHeat);
                irsendmeidi.setPowers(1);
                delay(100);
                irsendmeidi.setModes(2);
                currentState->setVal(state);
                break;
            case Characteristic::TargetHeatingCoolingState::COOL:
                // ac->on();
                // ac->setMode(kMideaACCool);
                irsendmeidi.setPowers(1);
                delay(100);
                irsendmeidi.setModes(1);
                currentState->setVal(state);
                break;
            default:
                // ac->on();
                // ac->setMode(kMideaACAuto);
                irsendmeidi.setPowers(1);
                delay(100);
                irsendmeidi.setModes(0);
                currentState->setVal(0);
                break;
        }

        // ac->setTemp(temp);
        // ac->send();
        irsendmeidi.setTemps(temp);
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
