#include <radio.h>


struct rfPacket {
  int throttle_rf;
  int roll_rf;
  int pitch_rf;
  int yaw_rf;
  int checksum;
  
} rfMessage;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  rfBegin(21);  // Initialize ATmega128RFA1 radio on channel 11 (can be 11-26)

}


void loop() {
  // put your main code here, to run repeatedly:
  int len;
  if (len = rfAvailable() && rfMessage.checksum == rfMessage.throttle_rf^rfMessage.roll_rf^rfMessage.pitch_rf^rfMessage.yaw_rf)  // If serial comes in...
  {
    rfRead((uint8_t *) & rfMessage, sizeof(rfMessage));

    Serial.println(rfMessage.throttle_rf);
    analogWrite(3, rfMessage.throttle_rf);
    analogWrite(4, rfMessage.throttle_rf);
    analogWrite(8, rfMessage.throttle_rf);
    analogWrite(9, rfMessage.throttle_rf);

  }

}
