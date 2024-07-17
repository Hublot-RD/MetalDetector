#include "battery.hpp"


namespace battery {

void setup() {
  pinMode(VBAT_PIN, INPUT);
  pinMode(BATSELA_PIN, OUTPUT);
  pinMode(BATSELB_PIN, OUTPUT);
  pinMode(BATSELC_PIN, OUTPUT);

  digitalWrite(BATSELA_PIN, LOW);
  digitalWrite(BATSELB_PIN, LOW);
  digitalWrite(BATSELC_PIN, LOW);
}


void select(uint8_t channel) {
  if(channel & 0b100) {digitalWrite(BATSELC_PIN, HIGH);}
  else {digitalWrite(BATSELC_PIN, LOW);}
  if(channel & 0b010) {digitalWrite(BATSELB_PIN, HIGH);}
  else {digitalWrite(BATSELB_PIN, LOW);}
  if(channel & 0b001) {digitalWrite(BATSELA_PIN, HIGH);}
  else {digitalWrite(BATSELA_PIN, LOW);}
}

float read(uint8_t channel) {
  float voltage = 0.0;
  battery::select(channel);

  int measurement = analogRead(VBAT_PIN);

  if(channel == V_TOTAL) {voltage = measurement/1024.0 * 35.8;}
  else if(channel == V_CELL_1) {voltage = measurement/1024.0 * 7.6;} // because voltage divider is inverted
  else {voltage = measurement/1024.0 * 4.2;}

  return voltage;
}

} // namespace battery