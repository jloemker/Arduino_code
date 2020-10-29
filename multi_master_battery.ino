// To understand the wiring with clock and base line -> https://medium.com/@pkl9231/arduino-master-slave-control-using-i2c-protocol-56504e348538
/*
 For voltages higher than 9.99, increase PAYLOAD_SIZE by 1 & change (increase) the number for every ********************[INCREASE] in the slave and master
 */
#include <Wire.h>
#define PAYLOAD_SIZE 11 //                                                                      *************************[INCREASE]
#define NODE_MAX 2 // maximum number of slave nodes (I2C addresses) to probe --- just 1
#define START_NODE 1 // The starting I2C address of slave nodes
#define NODE_READ_DELAY 500 // Some delay between I2C node reads
byte nodePayload[PAYLOAD_SIZE]; // it was int
//char c;

float PRESSURE;
float ALTITUDE;
float HUMIDITY;
byte VIB;

unsigned long mstime;

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
  for (int nodeAddress = 1; nodeAddress <= NODE_MAX; nodeAddress++) { // we are starting from Node address 2
    Wire.requestFrom(nodeAddress, PAYLOAD_SIZE);    // request data from node#
    if (Wire.available() >= 11) { // if data size is avaliable from nodes  ********************************** [INCREASE]
      /* For multiple slaves
      for (int i = 0; i < PAYLOAD_SIZE; i++) nodePayload[i] = Wire.read();  // get nodes data
      for (int j = 0; j < PAYLOAD_SIZE; j++) Serial.println(nodePayload[j]);   // print nodes data
      */
        nodePayload[0]=Wire.read();
        //Serial.println(nodePayload[0]);
        
      for(int i=1; i<=4; i++){
        nodePayload[i]=Wire.read();
        bytes[i-1]=nodePayload[i];
       // nodePayload[i] =
        TEMP = *(float*)(bytes);  // convert bytes back to float
      
      myFloat.m_float = TEMP;   // assign a float to union
    // send bytes over the data channel .. 
    // receive bytes and assign them to the union bytes: myFloat.m_bytes[0] = received_bytes[0] ..        
    TEMP = myFloat.m_float;   // get the float back from the union
      }
       
      for(int i=5; i<=9; i++){                                          //        ***************************[INCREASE]
        nodePayload[i]=Wire.read();
        bytesV[i-6]=nodePayload[i];
        
        VOLT = *(double*)(bytesV);  // convert bytes back to float
     //    Serial.println(VOLT);
      myDouble.m_double = VOLT;   // assign a float to union
     //    Serial.println(VOLT);
    // send bytes over the data channel ..
    // receive bytes and assign them to the union bytes: myFloat.m_bytes[0] = received_bytes[0] ..        
    VOLT = myDouble.m_double;   // get the float back from the union
      }
      
      }else{ // if you want to extend the number of measuremtns with different numbers of bytes
      // For multiple slaves
  
        nodePayload[0]=Wire.read();
        //Serial.println(nodePayload[0]);
        
        nodePayload[0] = nodeAddress;
        for(int i; i <=4; i++){
               bytes[i-1]=nodePayload[i];
       // nodePayload[i] =
        VIB = *(float*)(bytes);  // convert bytes back to float
      myFloat.m_float = VIB;   // assign a float to union
    // send bytes over the data channel .. 
    // receive bytes and assign them to the union bytes: myFloat.m_bytes[0] = received_bytes[0] ..        
    VIB = myFloat.m_float;
 
      }
    }

   mstime = millis();
   Serial.print(mstime);
   Serial.print(',');
   Serial.print(TEMP);
   Serial.print(',');
   Serial.print(VOLT);
   Serial.print(',');
   Serial.print(VIB);
   Serial.println();
  }
  delay(NODE_READ_DELAY);
}
