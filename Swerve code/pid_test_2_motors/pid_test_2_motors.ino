

/*  ACE128 Basic Test 0x38
 *  
 *  This is a basic validation test for the Absolute Encoder Module with 
 *  base address 0x38. This should work out of the box with your module.
 *  
 *  It does not demonstrate all the options and variations.
 *  
 *  See ACE128test example for all the options or if something goes wrong
 *  
 *  this sketch displays the current position in the three useful forms on the serial monitor at 9600 baud
 *    pos  = this is the raw value converted to the range -64 - +63 relative to the logical zero. Logical zero is
 *           wherever it was when the sketch started, unless the value was saved in EEPROM or reset.
 *    upos = this is the raw value converted to the range 0 - 127 relative to the logical zero
 *    mpos = this is like pos, but goes multiturn -32768 - 32767
 *  
 *  The value below is the address of your module. Set this to the correct value.
 *  For purchased modules this is the base address of the module.
 */


#include <PID_v1.h>
#define ACE_ADDR_LEFT 0x38
#define ACE_ADDR_RIGHT 0x20
#define ACE128_EEPROM_AVR
#define ACE128_EEPROM_ADDR 0x50

#include <ACE128.h>  // Include the ACE128.h from the library folder
#include <ACE128map87654321.h> // mapping for pin order 87654321

ACE128 myACEleft(ACE_ADDR_LEFT, (uint8_t*)encoderMap_87654321,0x50); // I2C without using EEPROM
ACE128 myACEright(ACE_ADDR_RIGHT, (uint8_t*)encoderMap_87654321,0x50); // I2C without using EEPROM
uint8_t oldPos = 255;
uint8_t upos;
int8_t pos;
int pos_angle;
int16_t mpos;
uint8_t rawpos;

#include <math.h>
#include <PS3BT.h>
#include <usbhub.h>

#define pi 3.1415
#define addr_to_save 0

#include <Servo.h>
#define LEFT_SERVO_PIN 11 
#define RIGHT_SERVO_PIN 12
#define LEFT_MOTOR_PIN 10
#define RIGHT_MOTOR_PIN 7
Servo myservoleft;  // create servo object to control a servo (left turning motor)
Servo myservoright;  // create servo object to control a servo (right turning motor)
Servo myservoleftm;  // create servo object to control a servo (left drive motor)
Servo myservorightm;  // create servo object to control a servo (right drive motor)

//write ints to an integer
#include <EEPROM.h>

//must be 2 apart
#define LEFT_OFFSET_ADDR 0
#define RIGHT_OFFSET_ADDR 2
int left_offset;// = -158;
int right_offset;// = 296;

void EEPROMWriteInt(int address, int value);
int EEPROMReadInt(int address);
 
//Specify the links and initial tuning parameters
#define SAMPLE_RATE 20
double Ku=9, Tu=0.25;
double Kp=0.6*Ku, Ki=1.2*Ku/Tu, Kd=2*Ku*Tu/40;
//Define Variables we'll be connecting to
double Setpoint_left, Input_left, Output_left;
PID myPIDleft(&Input_left, &Output_left, &Setpoint_left, Kp, Ki, Kd, DIRECT);
//Define Variables we'll be connecting to
double Setpoint_right, Input_right, Output_right;
PID myPIDright(&Input_right, &Output_right, &Setpoint_right, Kp, Ki, Kd, DIRECT);

int turn(int x, int y){
    int z = x-y;
    if(z>180){
        z-=360;
    }else if (z<-180
    ){
        z+=360;
    }
    return z;
}

int newconnect = 0;
int leftX, leftY, rightX, rightY;
USB Usb;
USBHub Hub1(&Usb);
BTD Btd(&Usb);
PS3BT PS3(&Btd);

bool new_offset_left = false;
bool new_offset_right = false;

void setup() {
   
  Serial.begin(115200);
  
  //read values
  left_offset = EEPROMReadInt(LEFT_OFFSET_ADDR);
  right_offset = EEPROMReadInt(RIGHT_OFFSET_ADDR);
  
  Serial.print("The left offset was set to: ");
  Serial.println(left_offset);
  Serial.print("The right offset was set to: ");
  Serial.println(right_offset);
  new_offset_left = false;
  new_offset_right = false;
  
  int newconnect = 0;         // Variable(boolean) for connection to ps3, also activates rumble
  
  myACEleft.begin();    // this is required for each instance, initializes the pins
  myACEright.begin();    // this is required for each instance, initializes the pins

  if (Usb.Init() == -1)       // this is for an error message with USB connections
  {
    Serial.print(F("\r\nOSC did not start"));
    while (1);
  }
  Serial.print(F("\r\nPS3 Bluetooth Library Started"));

  
  myservoleft.attach(LEFT_SERVO_PIN, 1100, 1800);;  // attaches the servo on pin 9 to the servo 
  myservoright.attach(RIGHT_SERVO_PIN, 1100, 1800);;  // attaches the servo on pin 12 to the servo 
  myservoleftm.attach(LEFT_MOTOR_PIN, 1100, 1800);;  // attaches the servo on pin 9 to the servo 
  myservorightm.attach(RIGHT_MOTOR_PIN, 1100, 1800);;  // attaches the servo on pin 12 to the servo 

//turn the PID on
  Setpoint_left = 0;
  myPIDleft.SetOutputLimits(-180, 180);
  myPIDleft.SetSampleTime(SAMPLE_RATE);
  myPIDleft.SetMode(AUTOMATIC);
  myPIDright.SetOutputLimits(-180, 180);
  myPIDright.SetSampleTime(SAMPLE_RATE);
  myPIDright.SetMode(AUTOMATIC);
 
 Input_left = 0.0;
 Input_right = 0.0;
}



void loop() {
    Usb.Task();                           // This updates the input from the PS3 controller
        

  if (PS3.PS3Connected)                 // run if the controller is connected
  {
    if (newconnect == 0)                // this is the vibration that you feel when you first connect
    {
      PS3.moveSetRumble(64);
      PS3.setRumbleOn(100, 255, 100, 255); //VIBRATE!!!
      newconnect++;
    }
    
    if (PS3.getButtonClick(PS)) {
      PS3.disconnect();
      newconnect = 0;
    }

    /* For individual stick control of modules (debugging)
    leftX = map(PS3.getAnalogHat(LeftHatX), 0, 255, -90, 90);     // Recieves PS3
    leftY = map(PS3.getAnalogHat(LeftHatY), 0, 255, 90, -90);     // Recieves PS3
    rightX = map(PS3.getAnalogHat(RightHatX), 0, 255, -90, 90);     // Recieves PS3
    rightY = map(PS3.getAnalogHat(RightHatY), 0, 255, 90, -90);     // Recieves PS3    
    //*/
    double vx, vy, w;
    vx = map(PS3.getAnalogHat(LeftHatX), 0, 255, 180, -180);     // Recieves PS3; robot x-velocity (right positive)
    vy = map(PS3.getAnalogHat(LeftHatY), 0, 255, 180, -180);     // Recieves PS3; robot y-velocity (forward positive)
    w = map(PS3.getAnalogHat(RightHatX), 0, 255, -100, 100);     // Recieves PS3; robot rotational velocity (CCW positive)
    
    w = w/(10+sqrt(pow(vx,2) + pow(vy,2))); // scale w by speed (rotational sensitivity is a function of speed)
    
    /* Joysticks read 0 and 255 for a rather large range, not ideal for swerve;
     * in order to get a better "circular" value, the following piece of code
     * recalculates vx and vy assuming maximum velocity to normalize the magnitude
     * at all points around the joystick's limits
     */
    float theta;
    if((abs(vx)>179)||(abs(vy)>179)){ //assume max velocity:
      theta = atan2(vy,vx);
      vx = cos(theta)*180*sqrt(2);
      vy = sin(theta)*180*sqrt(2);
    }
    
    //Joystick deadzone:
    if((vx<10)&&(vx>-10)){
      vx = 0;
    }

    if((vy<10)&&(vy>-10)){
      vy = 0;
    }

    if((w<3)&&(w>-3)){
      w = 0;
    }

    //Swerve calcs:
    double vleft, dirleft, vright, dirright;
    vleft = sqrt(pow((vx - 4*w),2) + pow((vy + 4*w),2));
    dirleft = 180/pi*atan2((vx - 4*w),(vy + 4*w));
    vright = sqrt(pow((vx + 4*w),2) + pow((vy - 4*w),2));
    dirright = 180/pi*atan2((vx + 4*w),(vy - 4*w));
    //Limiting max speed output to 180:
    if((vleft>180)||(vright>180)){
      vleft = vleft/max(vleft,vright)*180;
      vright = vright/max(vleft,vright)*180;
    }

    int8_t pos_left = myACEleft.pos();                 // get logical position - signed -64 to +63
    int pos_angle_left = map(pos_left, -64, 63, -180, 180);
    pos_angle_left = pos_angle_left + left_offset;

    int8_t pos_right = myACEright.pos();                 // get logical position - signed -64 to +63
    int pos_angle_right = map(pos_right, -64, 63, -180, 180);
    pos_angle_right = pos_angle_right + right_offset;
    
    if(PS3.getButtonPress(R1)){ //For module angle correction on-the-fly:
      if(PS3.getButtonPress(L1)){
        if(PS3.getButtonClick(UP)) {
          left_offset += 10;
          new_offset_left = true;
          
        }
        else if(PS3.getButtonClick(DOWN)) {
          left_offset -= 10;
          new_offset_left = true;
        }
        if(PS3.getButtonClick(RIGHT)){
          right_offset += 10;
          new_offset_right = true;
        }
        else if(PS3.getButtonClick(LEFT)) {
          right_offset -= 10;
          new_offset_right = true;
        }
      }
      else{
        if(PS3.getButtonClick(UP)) {
          left_offset += 1;
          new_offset_left = true;
        }
        else if(PS3.getButtonClick(DOWN)) {
          left_offset -= 1;
          new_offset_left = true;
        }
        if(PS3.getButtonClick(RIGHT)) {
          right_offset += 1;
          new_offset_right = true;
        }
        else if(PS3.getButtonClick(LEFT)) {
          right_offset -= 1;
          new_offset_right = true;
        }
      }
      dirleft = 0;
      dirright = 0;
    }else {
      if(new_offset_right){
          new_offset_right = false;
          EEPROMWriteInt(RIGHT_OFFSET_ADDR,right_offset);
          
          int new_left_offset = EEPROMReadInt(LEFT_OFFSET_ADDR);
          int new_right_offset = EEPROMReadInt(RIGHT_OFFSET_ADDR);
          Serial.print("The left offset was set to: ");
          Serial.println(new_left_offset);
          Serial.print("The right offset was set to: ");
          Serial.println(new_right_offset);
      }
      if(new_offset_left){
          EEPROMWriteInt(LEFT_OFFSET_ADDR,left_offset);
          new_offset_left = false;
        int new_left_offset = EEPROMReadInt(LEFT_OFFSET_ADDR);
        int new_right_offset = EEPROMReadInt(RIGHT_OFFSET_ADDR);
        Serial.print("The left offset was set to: ");
        Serial.println(new_left_offset);
        Serial.print("The right offset was set to: ");
        Serial.println(new_right_offset);
      }
    }
    
    int to_turn_left = turn(pos_angle_left,dirleft);
    Input_left = (double) to_turn_left;
    int to_turn_right = turn(pos_angle_right,dirright);
    Input_right = (double) to_turn_right;
    
    myPIDleft.Compute();
    myPIDright.Compute();

    /*
      //Serial.print(" Output: ");
      Serial.print(Output_left);
      Serial.print(", ");
      Serial.print(Output_right);
      Serial.print(", ");
      //Serial.print(" Input: ");
      Serial.print(Input_left);
      Serial.print(", ");
      Serial.print(Input_right);
      Serial.println(", ");
      //Serial.print(" SetPoint: ");
      Serial.print(Setpoint_left);
      Serial.print(", ");
      Serial.print(pos_angle_left);
      Serial.print(", ");
      Serial.print(pos_angle_right);
      Serial.print(", ");
      Serial.println(millis());
      Serial.print(" Robot Speed: ");
      Serial.print(vx);
      Serial.print(", ");
      Serial.print(vy);
      Serial.print(", ");
      Serial.print(w);
      Serial.print(", ");
      Serial.print(" Speed: ");
      Serial.print(vleft);
      Serial.print(", ");
      Serial.print(vright);
      Serial.print(", ");
      Serial.print(" Direction: ");
      Serial.print(dirleft);
      Serial.print(", ");
      Serial.print(dirright);
      Serial.print(", ");
      */
      Serial.print(left_offset);
      Serial.print(", ");
      Serial.print(right_offset);
      Serial.print(", ");
      Serial.println(millis());
      //*/
      
      myservoleft.writeMicroseconds(map(Output_left,-180,180,1300,1700)); 
      myservoright.writeMicroseconds(map(Output_right,-180,180,1300,1700)); 
      if(PS3.getAnalogButton(R2)>127){
        myservoleftm.writeMicroseconds(map(vleft,-180,180,1000,2000)); 
        myservorightm.writeMicroseconds(map(vright,-180,180,1000,2000)); 
      }
      else{
        myservoleftm.writeMicroseconds(map(vleft,-180,180,1200,1800)); 
        myservorightm.writeMicroseconds(map(vright,-180,180,1200,1800)); 
      }
  
  }
  else terminator();
}

void terminator(){
  myservoleft.writeMicroseconds(1500); //Coasting stop value
  myservoleftm.writeMicroseconds(1500); //Coasting stop value
  myservoright.writeMicroseconds(1500); //Coasting stop value
  myservorightm.writeMicroseconds(1500); //Coasting stop value
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
