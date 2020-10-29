// To understand the wiring with clock and base line -> https://medium.com/@pkl9231/arduino-master-slave-control-using-i2c-protocol-56504e348538
#include <Wire.h>
#define PAYLOAD_SIZE 5 // how many bytes to expect from each I2C salve node ---- defenetly more than this
#define NODE_MAX 1 // maximum number of slave nodes (I2C addresses) to probe --- just 1
#define START_NODE 0 // The starting I2C address of slave nodes
#define NODE_READ_DELAY 10000 // Some delay between I2C node reads
byte nodePayload[PAYLOAD_SIZE]; // it was int
//char c;

float PRESSURE;
float ALTITUDE;
float HUMIDITY;

union Float {
    float    m_float;
    uint8_t  m_bytes[sizeof(float)];
};
 
float        TEMP;
uint8_t      bytes[sizeof(float)];
Float        myFloat;

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
  for (int nodeAddress = START_NODE; nodeAddress <= NODE_MAX; nodeAddress++) { // we are starting from Node address 2
    Wire.requestFrom(nodeAddress, PAYLOAD_SIZE);    // request data from node#
    if (Wire.available() == PAYLOAD_SIZE) { // if data size is avaliable from nodes
      /* For multiple slaves
      for (int i = 0; i < PAYLOAD_SIZE; i++) nodePayload[i] = Wire.read();  // get nodes data
      for (int j = 0; j < PAYLOAD_SIZE; j++) Serial.println(nodePayload[j]);   // print nodes data
      */
        nodePayload[0]=Wire.read();
        Serial.println(nodePayload[0]);
        
      for(int i=1; i<5; i++){
        nodePayload[i]=Wire.read();
        bytes[i-1]=nodePayload[i];
       // nodePayload[i] =
        TEMP = *(float*)(bytes);  // convert bytes back to float
      
      myFloat.m_float = TEMP;   // assign a float to union

    // send bytes over the data channel ..
    
    // receive bytes and assign them to the union bytes: myFloat.m_bytes[0] = received_bytes[0] ..        
    TEMP = myFloat.m_float;   // get the float back from the union
      }
    Serial.println(TEMP);    

      }
      Serial.println("*************************");
    }
  
  delay(NODE_READ_DELAY);
}
