#include <EEPROM.h>

#define LEFT_OFFSET_ADDR 0
#define RIGHT_OFFSET_ADDR 2
int left_offset = -158;
int right_offset = 296;

void EEPROMWriteInt(int address, int value);
int EEPROMReadInt(int address);

void setup() {
  Serial.begin(115200);

  EEPROMWriteInt(LEFT_OFFSET_ADDR,left_offset);
  EEPROMWriteInt(RIGHT_OFFSET_ADDR,right_offset);
  
  int new_left_offset = EEPROMReadInt(LEFT_OFFSET_ADDR);
  int new_right_offset = EEPROMReadInt(RIGHT_OFFSET_ADDR);
  Serial.print("The left offset was set to: ");
  Serial.println(new_left_offset);
  Serial.print("The right offset was set to: ");
  Serial.println(new_right_offset);
}

void loop() {
  // put your main code here, to run repeatedly:

}

void EEPROMWriteInt(int address, int value)
{
  byte two = (value & 0xFF);
  byte one = ((value >> 8) & 0xFF);
  
  EEPROM.update(address, two);
  EEPROM.update(address + 1, one);
}
 
int EEPROMReadInt(int address)
{
  long two = EEPROM.read(address);
  long one = EEPROM.read(address + 1);
 
  return ((two << 0) & 0xFFFFFF) + ((one << 8) & 0xFFFFFFFF);
}
