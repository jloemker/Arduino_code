#include <Wire.h>
#include <ds3231.h>
 
struct ts t; 
 
void setup() {
  Serial.begin(9600);
  Wire.begin();
  DS3231_init(DS3231_INTCN);
  /*----------------------------------------------------------------------------
  In order to synchronise your clock module, insert timetable values below !
  ----------------------------------------------------------------------------*/
  t.hour=9; 
  t.min=55;
  t.sec=0;
  t.mday=19;
  t.mon=10;
  t.year=2020;
 
  DS3231_set(t); 
}
 
void loop() {
  DS3231_get(&t);
  Serial.print("Date : ");
  Serial.print(t.mday);
  Serial.print("/");
  Serial.print(t.mon);
  Serial.print("/");
  Serial.print(t.year);
  Serial.print("\t Hour : ");
  Serial.print(t.hour);
  Serial.print(":");
  Serial.print(t.min);
  Serial.print(".");
  Serial.println(t.sec);
  Serial.println(t.year);
  delay(1000);
}
