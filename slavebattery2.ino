#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Servo.h>

#define SEALEVELPRESSURE_HPA (1013.25)
#define NODE_ADDRESS 2  // Change this unique address for each I2C slave node
#define PAYLOAD_SIZE 2 // Number of bytes  expected to be received by the master I2C node +3 for the try
byte nodePayload[PAYLOAD_SIZE];

 #include "I2Cdev.h"

 

#include "MPU6050_6Axis_MotionApps20.h"

 

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

#define OUTPUT_READABLE_REALACCEL


#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };


volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


Servo bigServo;                 // providing a name

int ballsDelay = 2400;          // Timelength to move balls from counterclockwise to clockwise position (and vice versa)
int incomingByte = 0; // for incoming serial data
int toServo = 0;      // to send to servo
int stateBalls = 0;   // 0 for center, 1 for fully left, -1 for fully right

void requestEvent()
{
  Wire.write(nodePayload, PAYLOAD_SIZE);//nodePayload, PAYLOAD_SIZE.
  Serial.print("Sensor value: ");  // for debugging purposes
}

void setup()
{
Serial.begin(9600);
  bigServo.attach(3);
  Serial.println("SLAVE SENDER NODE");
  Serial.print("Node address: ");
  Serial.println(NODE_ADDRESS);
  Serial.print("Payload size: ");
  Serial.println(PAYLOAD_SIZE);
  Serial.println("***********************");
  Wire.begin(NODE_ADDRESS);  // Activate I2C network
  Wire.onRequest(requestEvent); // Request attention of master node

      #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    devStatus = mpu.dmpInitialize();
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
    if (devStatus == 0) {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.setDMPEnabled(true);
        
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;

        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    } 
 
} 


 
union Float {
    float    m_float;
    uint8_t  m_bytes[sizeof(float)];
};

float        VIB;
uint8_t      bytes[sizeof(float)];
Float        myFloat;

void loop()
{
  delay(1000);
/*
   *(float*)(bytes) = TEMP;  // convert float to bytes
    printf("bytes = [ 0x%.2x, 0x%.2x, 0x%.2x, 0x%.2x ]\r\n", bytes[0], bytes[1], bytes[2], bytes[3], bytes[4]);
 nodePayload[0] = NODE_ADDRESS;  
  for(int i = 1; i <= 4; i++){
      nodePayload[i] = bytes[i-1];
  }
 */
// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() 
{
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            //Serial.print("areal\t");
            Serial.println(aaReal.x*aaReal.x);
            Vib = aaReal.x;
            //Serial.print("\t");
            //Serial.print(aaReal.y);
            //Serial.print("\t");
            //Serial.println(aaReal.z);
        #endif
        
        nodePayload[0] = NODE_ADDRESS;
        VIB = Vib;
   *(float*)(bytes) = VIB;  // convert float to bytes
    printf("bytes = [ 0x%.2x, 0x%.2x, 0x%.2x, 0x%.2x ]\r\n", bytes[0], bytes[1], bytes[2], bytes[3], bytes[4]);
        nodePayload[0] = NODE_ADDRESS;
        for(int i=1; i<=4 ; i++){
          nodePayload[i] = bytes[i-1]; 
        }
       
    }
}
