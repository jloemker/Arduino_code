#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define SEALEVELPRESSURE_HPA (1013.25)
#define NODE_ADDRESS 1  // Change this unique address for each I2C slave node
#define PAYLOAD_SIZE 5 // Number of bytes  expected to be received by the master I2C node +3 for the try
byte nodePayload[PAYLOAD_SIZE];
int data_len; // data[] = {1,1,1,1,1};
byte  fb;

//char c = "Hello";
//float nodePayload;
float Temp;
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
/*
void float2Bytes(float val,byte* bytes_array){
  // Create union of shared memory space
  union {
    float float_variable;
    byte temp_array[5];
  } u;
  // Overite bytes of union with float variable
  u.float_variable = val;
  // Assign bytes to input array
  memcpy(bytes_array, u.temp_array, 5);
}
*/

 
union Float {
    float    m_float;
    uint8_t  m_bytes[sizeof(float)];
};

float        TEMP;
uint8_t      bytes[sizeof(float)];
Float        myFloat;
void loop()
{
  delay(1000);
  Temp = bme.readTemperature();
  PRESSURE = bme.readPressure() / 100.0F ;
  ALTITUDE = bme.readAltitude(SEALEVELPRESSURE_HPA);
  HUMIDITY = bme.readHumidity();
  //1 byte = 8 bit -> 1 float = 32 bit -> 1 float = 4 byte
// int val = (TEMP*100);
 
  TEMP = Temp;
 
   *(float*)(bytes) = TEMP;  // convert float to bytes
    printf("bytes = [ 0x%.2x, 0x%.2x, 0x%.2x, 0x%.2x ]\r\n", bytes[0], bytes[1], bytes[2], bytes[3], bytes[4]);
    
    // send bytes over data channel ..

/*
  byte nodePayload[] = {1,2,3,4,5};
  nodePayload[0] = bytes[4];
  nodePayload[1] = bytes[3];
  nodePayload[2] = bytes[2];
  nodePayload[3] = bytes[1];
  nodePayload[4] = bytes[0];

 
  for(int i = 0; i < len(TEMP); i++)
  {
  data[] = i;
  }
  float TEMP;
  byte nodePayload[5];
  float2Bytes(TEMP,&nodePayload[0]);

*/
  nodePayload[0] = NODE_ADDRESS;
  
  for(int i = 1; i <= 4; i++){
      nodePayload[i] = bytes[i-1];
  }

  delay(10000);
}
  

void requestEvent()
{
  Wire.write(nodePayload, PAYLOAD_SIZE);//nodePayload, PAYLOAD_SIZE.
  Serial.print("Sensor value: ");  // for debugging purposes
  Serial.println(bytes[0]);
  Serial.print("Temperature: ");
  Serial.println(bytes[1]);
  Serial.println(bytes[2]);
/*    Serial.println(fb[0]);
      Serial.println(fb[1]);
        Serial.println(fb[2]);
          Serial.println(fb[3]);
       //   Serial.println(nodePayload[5]);

*/


//  Serial.println(payload);
  //Serial.println(TEMP);
/*
   Serial.print(u.temp_byte[0]);
   Serial.print(u.temp_byte[1]);
   Serial.print(u.temp_byte[2]);
  /*for(int i=0; i<4;i++){
  Serial.print(u.temp_byte[i]);
  }
  */
 // Serial.println(nodePayload[2]);
 // Serial.println(nodePayload[3]);

}
