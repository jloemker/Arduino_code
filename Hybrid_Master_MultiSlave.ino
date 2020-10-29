// To understand the wiring with clock and base line -> https://medium.com/@pkl9231/arduino-master-slave-control-using-i2c-protocol-56504e348538
#include <Wire.h>
#define PAYLOAD_SIZE 7 // how many bytes to expect from each I2C salve node
#define NODE_MAX 2 // maximum number of slave nodes (I2C addresses) to probe --- just 1
#define START_NODE 1 // The starting I2C address of slave nodes
#define NODE_READ_DELAY 1000 // Some delay between I2C node reads
byte nodePayload[PAYLOAD_SIZE]; // it was int
//char c;
float TEMP;
float PRESSURE;
float ALTITUDE;
float HUMIDITY;

int const YEAR;
float MONTH;
float DAY;
float HOUR;
float MINUTE;
float SECOND;


void setup()
{
  Serial.begin(9600);
  Serial.println("MASTER READER NODE");
  Serial.print("Maximum Slave Nodes: ");
  Serial.println(NODE_MAX);
  Serial.print("Payload size: ");
  Serial.println(PAYLOAD_SIZE);
  Serial.println("***********************");
Wire.begin(1);        // Activate I2C link - you could give it number 6 f.e. as argment
}
void loop()
{
  for (int nodeAddress = START_NODE; nodeAddress <= NODE_MAX; nodeAddress++) { // we are starting from Node address 0
    Wire.requestFrom(nodeAddress, PAYLOAD_SIZE);    // request data from node#
    if (Wire.available() == PAYLOAD_SIZE) { // if data size is avaliable from nodes
      // For multiple slaves
      for (int i = 0; i < PAYLOAD_SIZE; i++) nodePayload[i] = Wire.read();  // get nodes data
      for (int j = 0; j < PAYLOAD_SIZE; j++) Serial.println(nodePayload[j]);   // print nodes data
      
     //for(int i=0; i<=7; i++){
       // nodePayload[i]=Wire.read();
        //Serial.println(nodePayload[i]);
    //  }
      
      Serial.println("*************************");
    }
  }
  delay(NODE_READ_DELAY);
}
