#include <SoftwareSerial.h>

#define rxPin 2
#define txPin 3

int val = 0;

SoftwareSerial mySerial = SoftwareSerial(rxPin, txPin);

void setup() {
    pinMode(rxPin, INPUT);
    pinMode(txPin, OUTPUT);
    mySerial.begin(9600);
    Serial.begin(9600);
 }
 
void loop() {
    val = digitalRead(2);
    if (val == 1){
      mySerial.write("hello there");
      int val = 0;
    }
}
