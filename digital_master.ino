#include <SoftwareSerial.h>

#define rxPin 2
#define txPin 3

SoftwareSerial mySerial = SoftwareSerial(rxPin, txPin);

void setup() {
    pinMode(rxPin, INPUT);
    pinMode(txPin, OUTPUT);
    mySerial.begin(9600);
    Serial.begin(9600);
 }

void loop() {
    digitalWrite(3, HIGH); //sort of sends a command
    mySerial.listen(); //listen to slave
    while (mySerial.available() > 0){
      char c = mySerial.read();
      Serial.print(c);
    }
    digitalWrite(3, LOW);
    Serial.println();
    delay(1000);
}
