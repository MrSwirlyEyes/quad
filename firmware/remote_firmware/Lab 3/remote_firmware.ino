#include <radio.h>
#include "quad_remote.h"      // Header file with pin definitions and setup
#include <serLCD.h>
serLCD lcd;

struct rfPacket {
  int throttle_rf;
  int roll_rf;
  int pitch_rf;
  int yaw_rf;
  int checksum;
  
} rfMessage;

void setup() {
  Serial.begin(9600);

  pinMode(PIN_BTN1, INPUT_PULLUP);
  pinMode(PIN_BTN2, INPUT_PULLUP);
  
  rfBegin(21);  // Initialize ATmega128RFA1 radio on channel 11 (can be 11-26)
  
  rfMessage.throttle_rf = 0;
  rfMessage.roll_rf = 0;
  rfMessage.pitch_rf = 0;
  rfMessage.yaw_rf = 0;
  // Send a message to other RF boards on this channel
  rfPrint("ATmega128RFA1 Dev Board Online!\r\n");
}



void loop() {
  int yaw_raw = analogRead(PIN_YAW);
  int throttle_raw = analogRead(PIN_THROTTLE);
  int roll_raw = analogRead(PIN_ROLL);
  int pitch_raw = analogRead(PIN_PITCH);
  int red_button = !digitalRead(PIN_BTN1);
  int yellow_button = !digitalRead(PIN_BTN2);
  int potentiometer_1 = analogRead(PIN_POT1);
  int potentiometer_2 = analogRead(PIN_POT2);
  int yaw = constrain(map(yaw_raw, 153, 862, 0, 1023),0,1023);
  int throttle = constrain(map(throttle_raw, 139, 854, 0, 1023),0,1023);
  int pitch = constrain(map(pitch_raw, 94, 823, 0, 1023),0,1023);
  int roll = constrain(map(roll_raw, 104, 838, 0, 1023),0,1023);
  Serial.print("Yaw: ");
  Serial.print(yaw);
  Serial.print(" Throttle: ");
  Serial.print(throttle);
  Serial.print(" Roll: ");
  Serial.print(roll);
  Serial.print(" Pitch ");
  Serial.print(pitch); 
  Serial.print("Button 1 (Red): ");
  Serial.print(red_button);
  Serial.print(" Button 2 (Yellow): ");
  Serial.print(yellow_button);
  Serial.println();
  
  rfMessage.yaw_rf  = map(yaw, 0, 1023, 0, 255);
  rfMessage.throttle_rf = map(throttle, 0, 1023, 0, 255);
  rfMessage.pitch_rf = map(pitch, 0, 1023, 0, 255);
  rfMessage.roll_rf = map(roll, 0, 1023, 0, 255);
  rfMessage.checksum = rfMessage.throttle_rf^rfMessage.roll_rf^rfMessage.pitch_rf^rfMessage.yaw_rf;
  rfWrite((uint8_t *) & rfMessage, sizeof(rfMessage));
  //analogWrite(3, throttle_to_motor);

  delay(50);

  
}
   

                                                                                                                                                                               

