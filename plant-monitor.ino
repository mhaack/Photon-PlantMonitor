#include "Adafruit_Sensor.h"
#include "Adafruit_BME280.h"
#include "elapsedMillis.h"
#include "CircularBuffer.h"
#include "MQTT.h"

#define SEALEVELPRESSURE_HPA (1013.25)

// control led
const int led = D7;

char publishString[128];
const String sensorName = "plantmonitor";

// BME280 sensor
Adafruit_BME280 bme;
int currentSensorStatus = 0;
double temperature = 0;
double temperatureOffset = 0;
double pressure = 0;
double altitude = 0;
double humidity = 0;
const double TEMPERATURE_THRESHOLD_LOW = 0; // used for alerts if temp is blow this value
int setTemperatureOffset(String command); // function to configure the temperate offset

// moisture sensor
struct SoilSensor {
  int sensor;
  int power;
};
SoilSensor soilSensors[3] = {{ A0, D4 }, { A1, D5 }, { A2, D6 }};
CircularBuffer<int, 15> soilBuffer[3];
int soil1 = 2048, soil2 = 2048, soil3 = 2048;

// interval counters for measurement and post
unsigned int measurementInterval = 60; // read sensor data every 60 sec
elapsedMillis lastMeasurement;

// MQTT
void callback(char* topic, byte* payload, unsigned int length);
byte mqttServer[] = { 192,168,1,20 };
MQTT mqttClient(mqttServer, 1883, callback);

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

  // declare Particle cloud variables & functions
  Particle.variable("temperature", &temperature, DOUBLE);
  Particle.variable("pressure", &pressure, DOUBLE);
  Particle.variable("humidity", &humidity, DOUBLE);
  Particle.variable("soil1", &soil1, INT);
  Particle.variable("soil2", &soil2, INT);
  Particle.variable("soil3", &soil3, INT);
  Particle.variable("status", &currentSensorStatus, INT);

  Particle.function("tempOffset", setTemperatureOffset);
  Particle.function("tempInterval", setTemperatureInterval);
}

void loop() {
    // MQTT loop
    if (!mqttClient.isConnected()) {
      reconnect();
    }
    mqttClient.loop();

    // read sensor data
    if (lastMeasurement > (measurementInterval * 1000)) {
      currentSensorStatus = readBMESensor();
      delay(200);
      readSoilSensor1();
      delay(200);
      readSoilSensor2();
      delay(200);
      readSoilSensor3();

      dumpSerial();
      postToParticle();
      postToMQTT();

      lastMeasurement = 0;
    }
}

// MQTT recieve message
void callback(char* topic, byte* payload, unsigned int length) {
    //char p[length + 1];
    //memcpy(p, payload, length);
    //p[length] = NULL;
    //String message(p);
}

// MQTT connect
void reconnect() {
  while (!mqttClient.isConnected()) {
    Serial.print("Attempting MQTT connection...");
    if (mqttClient.connect(sensorName)) {
      Serial.println("connected");
      mqttClient.subscribe("inTopic");
    } else {
      Serial.println("failed, try again in 5 seconds");
      delay(5000);
    }
  }
}

int setTemperatureOffset(String command) {
  temperatureOffset = command.toFloat();
  Serial.printlnf("Temperature offset set to %.2f", temperatureOffset);
  return 1;
}

int setTemperatureInterval(String command) {
  if(command.toInt() > 0) {
    measurementInterval = command.toInt();
      Serial.printlnf("Measurement interval set to %d", measurementInterval);
    return 1;
  }
  else return -1;


  return 1;
}

void postToParticle() {
  sprintf(publishString,"{\"status\": %d, \"temp\": %0.2f, \"pressure\": %0.2f, \"humidity\": %0.2f, \"soil1\": %u, \"soil2\": %u, \"soil3\": %u}",
    currentSensorStatus, temperature, pressure, humidity, soil1, soil2, soil3);
  Particle.publish(sensorName, publishString);
}

void postToMQTT() {
  sprintf(publishString, "%0.2f", temperature);
  mqttClient.publish(sensorName + "/temperature", publishString);
  sprintf(publishString, "%0.2f", pressure);
  mqttClient.publish(sensorName + "/pressure", publishString);
  sprintf(publishString, "%0.2f", humidity);
  mqttClient.publish(sensorName + "/humidity", publishString);
  sprintf(publishString, "%u", soil1);
  mqttClient.publish(sensorName + "/soil1", publishString);
  sprintf(publishString, "%u", soil2);
  mqttClient.publish(sensorName + "/soil2", publishString);
  sprintf(publishString, "%u", soil3);
  mqttClient.publish(sensorName + "/soil3", publishString);

}

// read BME280 sensor data
int readBMESensor() {
  double currentTemperature = 0;
  digitalWrite(led, HIGH);
  currentTemperature = bme.readTemperature() - temperatureOffset;
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

void readSoilSensor1() {
  readSoilSensor(soilSensors[0].sensor, soilSensors[0].power, soil1);
  soilBuffer[0].push(soil1);
}

void readSoilSensor2() {
  readSoilSensor(soilSensors[1].sensor, soilSensors[1].power, soil2);
  soilBuffer[1].push(soil2);
}

void readSoilSensor3() {
  readSoilSensor(soilSensors[2].sensor, soilSensors[2].power, soil3);
  soilBuffer[2].push(soil3);
}

// read soil sensor data
void readSoilSensor(const int soilSensor, const int soilSensorPower, int &soilValue) {
    digitalWrite(led, HIGH);
    digitalWrite(soilSensorPower, HIGH); //turn sensor power on
    delay(100); // give some time to settle
    soilValue = analogRead(soilSensor);
    digitalWrite(soilSensorPower, LOW); //turn sensor power off
    delay(100);
    digitalWrite(led, LOW);
}

void dumpSerial() {
  Serial.printlnf("Temperature = %.2f °C", temperature);
  Serial.printlnf("Pressure = %.2f hPa", pressure);
  Serial.printlnf("Approx. Altitude = %.2f m", altitude);
  Serial.printlnf("Humidity = %.2f %%", humidity);
  Serial.printlnf("Soil 1 = %d", soil1);
  Serial.printlnf("Soil 2 = %d", soil2);
  Serial.printlnf("Soil 3 = %d", soil3);
}
