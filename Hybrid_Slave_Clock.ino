#include <Wire.h>
#include <ds3231.h>
 
struct ts t;

#define NODE_ADDRESS 2  // Change this unique address for each I2C slave node
#define PAYLOAD_SIZE 14 // Number of bytes  expected to be received by the master I2C node +3 for the try
byte nodePayload[PAYLOAD_SIZE];
//char c = "Hello";
//float nodePayload;
int YEAR;
float MONTH;
float DAY;
float HOUR;
float MINUTE;
float SECOND;

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  DS3231_init(DS3231_INTCN);

//  float f = 23.56;
//  byte* b = (byte*) &f;
  
  /*----------------------------------------------------------------------------
  In order to synchronise your clock module, insert timetable values below !
  ----------------------------------------------------------------------------*/
  t.hour=23; 
  t.min=59;
  t.sec=30;
  t.mday=31;
  t.mon=12;
  t.year=2020;
 
  DS3231_set(t);
   
  Serial.println("SLAVE SENDER NODE");
  Serial.print("Node address: ");
  Serial.println(NODE_ADDRESS);
  Serial.print("Payload size: ");
  Serial.println(PAYLOAD_SIZE);
  Serial.println("***********************");
  Wire.begin(NODE_ADDRESS);  // Activate I2C network
  Wire.onRequest(requestEvent); // Request attention of master node
}
void loop()
{
  DS3231_get(&t);
  DAY = t.mday;
  MONTH = t.mon;
  YEAR = t.year;
  HOUR = t.hour;
  MINUTE = t.min;
  SECOND = t.sec;

  int YEAR2 = YEAR/100;
  Serial.println(YEAR2);

  int YEAR3 = YEAR - YEAR2*100;
  Serial.println(YEAR3);
  
  nodePayload[0] = NODE_ADDRESS;
  nodePayload[1] = DAY;
  nodePayload[2] = MONTH;
  nodePayload[3] = YEAR2;
  nodePayload[4] = YEAR3;
  nodePayload[5] = HOUR;
  nodePayload[6] = MINUTE;
  nodePayload[7] = SECOND;
  delay(1000);
  //Serial.println(t.year);
}

void requestEvent()
{
  Wire.write(nodePayload, PAYLOAD_SIZE);//nodePayload, PAYLOAD_SIZE.
//  Serial.print("Sensor value: ");  // for debugging purposes
//  Serial.println(nodePayload[0]);
//  Serial.print("Temperature: ");
//  Serial.print(YEAR);
//  Serial.print("Pressure: ");
//  Serial.println(nodePayload[3]);
 // Serial.println(nodePayload[3]);

}
