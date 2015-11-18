#include "Adafruit_Sensor.h"
#include "Adafruit_BME280.h"
#include "elapsedMillis.h"

#define SEALEVELPRESSURE_HPA (1013.25)

// control led
const int led = D7;

// main status
int lastSensorStatus = 0, currentSensorStatus = 0;
char publishString[128];
char eventString[10];

// BME280 sensor
Adafruit_BME280 bme;
double temperature = 0;
double pressure = 0;
double altitude = 0;
double humidity = 0;
const double TEMPERATURE_THRESHOLD_LOW = 0; // used for alerts if temp is blow this value

// moisture sensor
const int SOIL_THRESHOLD_LOW = 500;
const int SOIL_THRESHOLD_HIGH = 3500;
struct SoilSensor {
  int sensor;
  int power;
};
SoilSensor soilSensors[3] = {{ A0, D4 }, { A1, D5 }, { A2, D6 }};
int soil1 = 2048, soil2 = 2048, soil3 = 2048;

// interval counters for measurement and post
const unsigned int MEASUREMENT_RATE = 60000; // read sensor data every 60 sec
const unsigned int POST_RATE = 60 * 60000; // post data every 60 minutes
elapsedMillis lastMeasurement;
elapsedMillis lastPost;

const unsigned int PUBLISHEVENT_TIME_HOUR = 15; // send events ~ 3pm
boolean particleEventPublished = true;

void setup() {
  Serial.begin(9600);
  Time.zone(1);

  // declare GPIO pins
  pinMode(led, OUTPUT);
  for (unsigned int i = 0; i < sizeof(soilSensors); i++) {
    pinMode(soilSensors[i].sensor, INPUT);
    pinMode(soilSensors[i].power, OUTPUT);
  }

  // init BME280 sensor
  if (!bme.begin()) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  // declare Particle cloud variables
  Particle.variable("temperature", &temperature, DOUBLE);
  Particle.variable("pressure", &pressure, DOUBLE);
  Particle.variable("humidity", &humidity, DOUBLE);
  Particle.variable("soil1", &soil1, INT);
  Particle.variable("soil2", &soil2, INT);
  Particle.variable("soil3", &soil3, INT);
  Particle.variable("status", &currentSensorStatus, INT);
}

void loop() {
    // read sensor data
    if (lastMeasurement > MEASUREMENT_RATE) {
      int bmeSensorStatus = readBMESensor();
      delay(200);
      int soil1SensorStatus = readSoilSensor(soilSensors[0].sensor, soilSensors[0].power, soil1);
      delay(200);
      int soil2SensorStatus = readSoilSensor(soilSensors[1].sensor, soilSensors[1].power, soil2);
      delay(200);
      int soil3SensorStatus = readSoilSensor(soilSensors[2].sensor, soilSensors[2].power, soil3);

      // calc overall sensor status
      currentSensorStatus = (bmeSensorStatus << 6) + (soil1SensorStatus << 4) +
        (soil2SensorStatus << 2) + soil3SensorStatus;

      dumpSerial();
      postToParticle("sensor");

      // send events to particle cloud used to trigger IFTTT push notification
      if (Time.hour() == PUBLISHEVENT_TIME_HOUR && particleEventPublished == true) {
        postSoilEventToParticle();
        particleEventPublished = false; // only publish once
      }
      if (Time.hour() > PUBLISHEVENT_TIME_HOUR) {
        particleEventPublished = true; // reset publish status
      }
      lastMeasurement = 0;
    }

    // update cloud on timer or sensor data changes
    if (lastPost > POST_RATE || currentSensorStatus != lastSensorStatus) {
      postBMEEventToParticle();
      postToParticle("parse");
      lastPost = 0;
    }

    lastSensorStatus = currentSensorStatus;
}

// post full sensor status to Particle cloud used for Parse.com data storage
void postToParticle(String eventName) {
  sprintf(publishString,"{\"status\": %d, \"temp\": %0.2f, \"pressure\": %0.2f, \"humidity\": %0.2f, \"soil1\": %u, \"soil2\": %u, \"soil3\": %u}",
    currentSensorStatus, temperature, pressure, humidity, soil1, soil2, soil3);
  Particle.publish(eventName, publishString);
}

// post soil sensor low status to Particle cloud used for IFTTT push
void postSoilEventToParticle() {
  if (soil1 < SOIL_THRESHOLD_LOW) {
    Particle.publish("plant_alert", "1");
  }
  if (soil2 < SOIL_THRESHOLD_LOW) {
    Particle.publish("plant_alert", "2");
  }
  if (soil3 < SOIL_THRESHOLD_LOW) {
    Particle.publish("plant_alert", "3");
  }
}

// post BMP temperature sensor low status to Particle cloud used for IFTTT push
void postBMEEventToParticle() {
  if (temperature < TEMPERATURE_THRESHOLD_LOW) {
    sprintf(eventString, "%0.2f °C", temperature);
    Particle.publish("temp_alert", eventString);
  }
}

// read BME280 sensor data
int readBMESensor() {
  double currentTemperature = 0;
  digitalWrite(led, HIGH);
  currentTemperature = bme.readTemperature();
  pressure = bme.readPressure() / 100.0F;
  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  humidity = bme.readHumidity();
  delay(200);
  digitalWrite(led, LOW);

  // calc temp diff for changes at .x level
  int tempDiff = (currentTemperature - temperature)*10;
  temperature = currentTemperature;

  if (tempDiff > 0) {
    return 1; // indicate temp increase above threshold
  } else if (tempDiff < 0){
    return 2; // indicate temp decrease above threshold
  }
  return 0; // indicate normal reading
}

// read soil sensor data
int readSoilSensor(const int soilSensor, const int soilSensorPower, int &soilValue) {
    digitalWrite(led, HIGH);
    digitalWrite(soilSensorPower, HIGH); //turn sensor power on
    delay(10); // give some time to settle
    soilValue = analogRead(soilSensor);
    digitalWrite(soilSensorPower, LOW); //turn sensor power off
    delay(190);
    digitalWrite(led, LOW);

    if (soilValue < SOIL_THRESHOLD_LOW) {
      return 1; // indicate soil is to low
    } else if (soilValue > SOIL_THRESHOLD_HIGH) {
      return 2; // indicate soil is to high
    }
    return 0; // indicate normal reading
}

void dumpSerial() {
  Serial.print("Temperature = ");
  Serial.print(temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(pressure);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(altitude);
  Serial.println(" m");

  Serial.print("Humidity = ");
  Serial.print(humidity);
  Serial.println(" %");

  Serial.print("Soil 1 = ");
  Serial.println(soil1);

  Serial.print("Soil 2 = ");
  Serial.println(soil2);

  Serial.print("Soil 3 = ");
  Serial.println(soil3);
}
