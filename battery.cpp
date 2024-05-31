#include "battery.hpp"



void battery_setup() {
  pinMode(VBAT_PIN, INPUT);
  pinMode(BATSELA_PIN, OUTPUT);
  pinMode(BATSELB_PIN, OUTPUT);
  pinMode(BATSELC_PIN, OUTPUT);

  digitalWrite(BATSELA_PIN, LOW);
  digitalWrite(BATSELB_PIN, LOW);
  digitalWrite(BATSELC_PIN, LOW);
}


void battery_select(uint8_t channel) {
  if(channel & 0b100) {digitalWrite(BATSELC_PIN, HIGH);}
  else {digitalWrite(BATSELC_PIN, LOW);}
  if(channel & 0b010) {digitalWrite(BATSELB_PIN, HIGH);}
  else {digitalWrite(BATSELB_PIN, LOW);}
  if(channel & 0b001) {digitalWrite(BATSELA_PIN, HIGH);}
  else {digitalWrite(BATSELA_PIN, LOW);}
}

float battery_read(uint8_t channel) {
  float voltage = 0.0;
  battery_select(channel);

  int measurement = analogRead(VBAT_PIN);

  if(channel == 5) {voltage = measurement/1024.0 * 36.3;}
  else if(channel == 0) {voltage = measurement/1024.0 * 7.6;} // because voltage divider is inverted
  else {voltage = measurement/1024.0 * 4.2;}

  return voltage;
}