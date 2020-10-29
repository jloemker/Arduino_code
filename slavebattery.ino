/*
 For voltages higher than 9.99, increase PAYLOAD_SIZE by 1 & change (increase) the number for every ********************[INCREASE] in the slave and master
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define SEALEVELPRESSURE_HPA (1013.25)
#define NODE_ADDRESS 1  // Change this unique address for each I2C slave node
#define PAYLOAD_SIZE 11 // Number of bytes  expected to be received by the master *************************[INCREASE]
byte nodePayload[PAYLOAD_SIZE];

//char c = "Hello";
//float nodePayload;
float Temp;
float PRESSURE;
float ALTITUDE;
float HUMIDITY;
Adafruit_BME280 bme;

int PIN_HEATING= 2;
int PIN_COOLING =4;
int T_L =15;
int T_H= 30;

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
 
union Float {
    float    m_float;
    uint8_t  m_bytes[sizeof(float)];
};

float        TEMP;
uint8_t      bytes[sizeof(float)];
Float        myFloat;

union Double {
    double    m_double;
    uint8_t  m_bytesV[sizeof(double)];
};
 
double VOLT;
uint8_t bytesV[sizeof(double)];
Double        myDouble;

void loop()
{
  delay(1000);
  Temp = bme.readTemperature();
  PRESSURE = bme.readPressure() / 100.0F ;
  ALTITUDE = bme.readAltitude(SEALEVELPRESSURE_HPA);
  HUMIDITY = bme.readHumidity();
  //1 byte = 8 bit -> 1 float = 32 bit -> 1 float = 4 byte
  TEMP = Temp;
   *(float*)(bytes) = TEMP;  // convert float to bytes
    printf("bytes = [ 0x%.2x, 0x%.2x, 0x%.2x, 0x%.2x ]\r\n", bytes[0], bytes[1], bytes[2], bytes[3], bytes[4]);
 nodePayload[0] = NODE_ADDRESS;  
  for(int i = 1; i <= 4; i++){
      nodePayload[i] = bytes[i-1];
  }

  // float T = read_Temperature();
//for temperature control

 if (Temp>T_H){
    activate_cooler(); 
    Serial.println("Cooling.");}
 
 else if (Temp<T_L){
    activate_heater();
    Serial.println("Heating.");
    }
 else{ deactivate();}

    double R1 = 19.902; // resistance of resistor in voltage divider in kOhms
    double R2 = 4.717;
  double factor = (R1+R2)/R2; //ratio to go from measured voltage to actual voltage of battery
  // read the input on analog pin 0:
  int sensorValue = analogRead(A0);
  double voltage = factor * sensorValue * 3.3 /1024;// 3.3/1024 is to convert to a voltage, where 3.3 is the max voltage the arduino can measure. (3.3 for Due, 5.0 for most others)
    VOLT = voltage;
    Serial.println(VOLT);
     *(double*)(bytesV) = VOLT; 
 for(int j = 5; j<=9; j++){       //                                                                      *************************[INCREASE]
     nodePayload[j] = bytesV[j-6];
 }
  delay(100);
}

void requestEvent()
{
  Wire.write(nodePayload, PAYLOAD_SIZE);//nodePayload, PAYLOAD_SIZE.
  Serial.print("Sensor value: ");  // for debugging purposes
}



void activate_heater(){
  digitalWrite(PIN_COOLING,LOW);
  digitalWrite(PIN_HEATING,HIGH); }

void activate_cooler(){
  digitalWrite(PIN_HEATING,LOW);
  digitalWrite(PIN_COOLING,HIGH);}

void deactivate(){
  digitalWrite(PIN_COOLING,LOW);
  digitalWrite(PIN_HEATING,LOW);}
 
