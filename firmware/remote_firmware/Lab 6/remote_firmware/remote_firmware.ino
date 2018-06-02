#include <radio.h>
#include "quad_remote.h"      // Header file with pin definitions and setup
//#include <serLCD.h>
//serLCD lcd;

//TODO: Add disarm feature

#define DEBUG 0
#define BAUDRATE 115200

struct rfPacket {
  byte throttle;
  char pitch;
  char roll;
  char yaw; 
  byte pot_L;

  byte checksum = 0xDEADBEAF;
} rfMessage;

int throttle_raw;
int pitch_raw;
int roll_raw;
int yaw_raw;

int throttle_mapped;
int pitch_mapped;
int roll_mapped;
int yaw_mapped;


int button_left;
int button_right;
int pot_left;
int pot_right;



void setup() {
  #if DEBUG
    Serial.begin(BAUDRATE);
  #endif

  pinMode(PIN_BTN1, INPUT_PULLUP);
  pinMode(PIN_BTN2, INPUT_PULLUP);
  
  rfBegin(21);
  
  rfMessage.throttle = 0;
  rfMessage.pitch = 0;
  rfMessage.roll = 0;  
  rfMessage.yaw = 0;  
  rfMessage.checksum = 0xDEADBEAF;
}

void loop() {  
  throttle_raw = analogRead(PIN_THROTTLE);
  pitch_raw = analogRead(PIN_PITCH);
  roll_raw = analogRead(PIN_ROLL);
  yaw_raw = analogRead(PIN_YAW);
  
  button_left = !digitalRead(PIN_BTN1);
  button_right = !digitalRead(PIN_BTN2);
  
  pot_left = constrain(map(analogRead(PIN_POT1),115,816,0,1023),0,100);
  rfMessage.pot_L=pot_left;
  Serial.println(rfMessage.pot_L);
  pot_right = constrain(map(analogRead(PIN_POT2),115,816,0,1023),0,100);
//  rfMessage.pot2_rf=pot_right;
  
  yaw_mapped = constrain(map(yaw_raw, 153, 816, 1023, 0),0,1023);
  throttle_mapped = constrain(map(throttle_raw, 137, 816, 0, 1023),0,1023);
  pitch_mapped = constrain(map(pitch_raw, 113, 816, 1023, 0),0,1023);
  roll_mapped = constrain(map(roll_raw, 115, 816, 0, 1023),0,1023);

  
//  rfMessage.throttle_rf = constrain(map(throttle_mapped, 0, 1023, 0, 255), 0, 255);
//  rfMessage.pitch_rf = constrain(map(pitch_mapped, 0, 1023, 0, 255), 0, 255);
//  rfMessage.roll_rf = constrain(map(roll_mapped, 0, 1023, 0, 255), 0, 255);
//  rfMessage.yaw_rf  = constrain(map(yaw_mapped, 0, 1023, 0, 255), 0, 255);

  rfMessage.throttle = constrain(map(throttle_mapped, 0, 1023, 0, 255), 0, 255);
  rfMessage.pitch = constrain(map(pitch_mapped, 0, 1023, -10, 10), -10, 10);
  if(abs(rfMessage.pitch) < 5)
    rfMessage.pitch = 0;
  else {
    if(rfMessage.pitch < 0)
      rfMessage.pitch+=5;
    else
      rfMessage.pitch-=5;
  }
  rfMessage.roll = constrain(map(roll_mapped, 0, 1023, -10, 10), -10, 10);
  if(abs(rfMessage.roll) < 5)
    rfMessage.roll = 0;
  else {
    if(rfMessage.roll < 0)
      rfMessage.roll+=5;
    else
      rfMessage.roll-=5;
  }
  rfMessage.yaw  = constrain(map(yaw_mapped, 0, 1023, -10, 10), -10, 10);
  if(abs(rfMessage.yaw) < 5)
    rfMessage.yaw = 0;
  else {
    if(rfMessage.yaw < 0)
      rfMessage.yaw+=5;
    else
      rfMessage.yaw-=5;
  }
  
  rfWrite((uint8_t *) & rfMessage, sizeof(rfMessage));  
  rfFlush();

  #if DEBUG   
    print_rfMessage(); 
//    print_raw_gimbals();
//    print_mapped_gimbals();
//    print_misc_peripherals();
  #endif

//  delay(50);  
}


void print_rfMessage() {
  Serial.print("(throttle, pitch, roll, yaw, checksum)=(");
  Serial.print(rfMessage.throttle); Serial.print(", ");
  Serial.print((int) rfMessage.pitch); Serial.print(", ");
  Serial.print((int) rfMessage.roll); Serial.print(", ");
  Serial.print((int) rfMessage.yaw); Serial.print(", ");
  Serial.print(rfMessage.checksum); Serial.println(")");
}

void print_mapped_gimbals() {
  Serial.print("(throttle, pitch, roll, yaw)=(");
  Serial.print(throttle_mapped); Serial.print(", ");
  Serial.print(pitch_mapped); Serial.print(", ");
  Serial.print(roll_mapped); Serial.print(", ");
  Serial.print(yaw_mapped); Serial.println(")");
}

void print_raw_gimbals() {
  Serial.print("(throttle, pitch, roll, yaw)=(");
  Serial.print(throttle_raw); Serial.print(", ");
  Serial.print(pitch_raw); Serial.print(", ");
  Serial.print(roll_raw); Serial.print(", ");
  Serial.print(yaw_raw); Serial.println(")");
}

void print_misc_peripherals() {
  Serial.print("(button_left, button_right, pot_left, pot_right)=(");
  Serial.print(button_left); Serial.print(", ");
  Serial.print(button_right); Serial.print(", ");
  Serial.print(pot_left); Serial.print(", ");
  Serial.print(pot_right); Serial.println(")");
}
