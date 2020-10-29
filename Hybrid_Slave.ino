#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define SEALEVELPRESSURE_HPA (1013.25)
#define NODE_ADDRESS 1  // Change this unique address for each I2C slave node
#define PAYLOAD_SIZE 2 // Number of bytes  expected to be received by the master I2C node +3 for the try
byte nodePayload[PAYLOAD_SIZE];
//char c = "Hello";
//float nodePayload;
float TEMP;
float PRESSURE;
float ALTITUDE;
float HUMIDITY;
Adafruit_BME280 bme;


void setup()
{
Serial.begin(9600);
  Serial.println("SLAVE SENDER NODE");
  Serial.print("Node address: ");
  Serial.println(NODE_ADDRESS);
  Serial.print("Payload size: ");
  Serial.println(PAYLOAD_SIZE);
  Serial.println("***********************");
  Wire.begin(NODE_ADDRESS);  // Activate I2C network
  Wire.begin();             //For env. measurement
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  Wire.onRequest(requestEvent); // Request attention of master node
}
void loop()
{
  delay(1000);
  TEMP = bme.readTemperature();
  PRESSURE = bme.readPressure() / 100.0F ;
  ALTITUDE = bme.readAltitude(SEALEVELPRESSURE_HPA);
  HUMIDITY = bme.readHumidity();
  nodePayload[0] = NODE_ADDRESS;
  nodePayload[1] = TEMP;
  nodePayload[2] = PRESSURE;
  nodePayload[3] = ALTITUDE;
  nodePayload[4] = HUMIDITY;

  delay(10000);
}
  


void requestEvent()
{
  Wire.write(nodePayload, PAYLOAD_SIZE);//nodePayload, PAYLOAD_SIZE.
  Serial.print("Sensor value: ");  // for debugging purposes
  Serial.println(nodePayload[0]);
  Serial.print("Temperature: ");
  Serial.print(nodePayload[1]);
 // Serial.println(nodePayload[2]);
 // Serial.println(nodePayload[3]);

}