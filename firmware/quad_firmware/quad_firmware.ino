#include <radio.h>
#include <Adafruit_Simple_AHRS.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>

#define TESTSTAND 0
#define PCB 1

#define DRONE PCB

#define DEBUG 0

#define RADIO_CHANNEL 21
#define BAUDRATE 115200

#define COMP_GAIN 0.02

//#define pitch_kp 0.6 //0.5
//#define pitch_kd 0.4 //0.4
//#define pitch_ki 0.01 //0.01
//
//#define roll_kp 0.6 //0.5
//#define roll_kd 0.4 //0.4
//#define roll_ki 0.01 //0.01
//
//#define yaw_kp 0.2 //
//#define yaw_kd 0.1
//#define yaw_ki 0.0 //

//// Increasing proportional
//#define pitch_kp 0.9 //0.5
//#define pitch_kd 0.4 //0.4
//#define pitch_ki 0.015 //0.01
//
//#define roll_kp 0.6 //0.5
//#define roll_kd 0.4 //0.4
//#define roll_ki 0.01 //0.01

////// Increasing derivative
//#define pitch_kp 0.01 //0.01
//#define pitch_kd 0.3 //0.3
//#define pitch_ki 0.01 //0.01
//
//#define roll_kp 0.01 //0.01
//#define roll_kd 0.3 //0.3
//#define roll_ki 0.01 //0.01

/// Balancing KP and KI
#define pitch_kp 1.0 //0.01
#define pitch_kd 0.0 //0.3
#define pitch_ki 0.0 //0.01

#define roll_kp 1.0 //0.01 0.1
#define roll_kd 0.0 //0.3
#define roll_ki 0.0 //0.05

#define yaw_kp 0.05
#define yaw_kd 0.05
#define yaw_ki 0.0

#define integral_max 50

#define MTR_TL 3 // GREEN A, CW
#define MTR_TR 19 // GREEN B, CCW

#define MTR_BL 4 // BLACK B, CCW
#define MTR_BR 5 // BLACK A, CW

#define MIN_MOTOR_SPEED 40 // TODO: optimize for this being 0? OR maybe try to increase this a little?
#define MAX_MOTOR_SPEED 255

// LED variables
#define LED_BL 17
#define LED_BR 16
#define LED_TL 2
//#define LED_TR 9+1
// NOTE: TESTSTAND has no LEDs...

struct rfPacket {
  byte throttle;
  char pitch;
  char roll;
  char yaw; 
  byte pot_L;

  byte checksum;
} rfMessage;

//byte offset = 32; //27 // constant offset to compensate for motors being uneven

char offset_TL = -9; char offset_TR = -27;
char offset_BL = -7; char offset_BR = -5;

byte checksum=0xDEADBEAF;

Adafruit_LSM9DS1 imu = Adafruit_LSM9DS1();
Adafruit_Simple_AHRS ahrs(&imu.getAccel(), &imu.getMag(), &imu.getGyro());
sensors_vec_t orientation;
quad_data_t quad;

float imu_offset_pitch;
float imu_offset_pitch_rate;
float imu_offset_roll;
float imu_offset_roll_rate;
float imu_offset_yaw_rate;

float filtered_pitch;
float filtered_pitch_rate;
float angle_pitch; // position off center
float error_pitch; // difference between angle and setpoint
float derivative_pitch; // reading from gyroscope - degrees/second
float integral_pitch; // summation of error over time
float setpoint_pitch; // the desired angle - compared with actual position to determine error
float control_pitch = 0; // value written to motors (two added, two subtracted), process variable
float error_pitch_last;

float filtered_roll;
float filtered_roll_rate;
float angle_roll;
float error_roll;
float derivative_roll;
float integral_roll;
float setpoint_roll;
float control_roll = 0;
float error_roll_last;

float filtered_yaw_rate;
float angle_yaw = 0;
float error_yaw;
float error_yaw_prev;
float derivative_yaw;
float integral_yaw;
float setpoint_yaw;
float control_yaw = 0;

//#define box_size 10
//float box_filter_roll[box_size] = {0};
//float box_filter_pitch[box_size] = {0};
//float box_filter_yaw[box_size] = {0};


byte signal_loss_count;
#define signal_loss_max 20

float t0=0;
float dt=0;

void setup() {
  // Write motors to 0 (safety)
  analogWrite(MTR_TL, 0); analogWrite(MTR_TR, 0); analogWrite(MTR_BL, 0); analogWrite(MTR_BR, 0);

  // Initialize LED pins
  pinMode(LED_BL, OUTPUT); pinMode(LED_BR, OUTPUT); pinMode(LED_TL, OUTPUT);// pinMode(LED_TR, OUTPUT);

  test_leds();
  delay(500);

  #ifdef DEBUG
    Serial.begin(BAUDRATE);
  #endif

  rfBegin(RADIO_CHANNEL);  // Initialize ATmega128RFA1 radio on channel 11 (can be 11-26)

  if (!imu.begin()) {       
    #ifdef DEBUG
      /* There was a problem detecting the LSM9DS1 ... check your connections */
      Serial.print(F("Ooops, no LSM9DS1 detected ... Check your wiring!"));
    #endif
    while (1) {
      // TODO: ADD SOMETHING HERE TO MAKE LEDS SIGNAL BAD STUFF
    }
  }

  imu_configuration();
  calibrate_imu();
  test_leds(); 

  signal_loss_count = 0;
}



void loop() {
//  dt = abs(millis() - t0) / 1000.0;
//  t0 = millis();  

  imu.read();
  ahrs.getQuadOrientation(&quad);

//  float curr_val_roll=quad.roll_rate;
//  static float box_sum_roll=0;
//  static int i=0;
//
//  box_filter_roll[i]=curr_val_roll;
//  box_sum_roll += (curr_val_roll - box_filter_roll[(i+1) % box_size]) / box_size;
//  // box_sum_roll += cur_val_roll - box_filter_roll[(i+1) % box_size];
//  // roll_value_used_in_calculations = box_sum_roll / box_size;
//
////  quad.roll_rate=box_sum_roll;
//  filtered_roll_rate = box_sum_roll;
//
//
//  float curr_val_pitch=quad.pitch_rate;
//  static float box_sum_pitch=0;  
//
//  box_filter_pitch[i]=curr_val_pitch;
//  box_sum_pitch += (curr_val_pitch - box_filter_pitch[(i+1) % box_size]) / box_size;
//
////  quad.pitch_rate=box_sum_pitch;
//  filtered_pitch_rate = box_sum_pitch;
//
//
//  float curr_val_yaw=quad.yaw_rate;
//  static float box_sum_yaw=0;  
//
//  box_filter_yaw[i]=curr_val_yaw;
//  box_sum_yaw += (curr_val_yaw - box_filter_yaw[(i+1) % box_size]) / box_size;
//
////  quad.yaw_rate=box_sum_yaw;
//  filtered_yaw_rate = box_sum_yaw;
//
//  i=(i+1) % box_size;

  filtered_pitch = quad.pitch - imu_offset_pitch;
  filtered_pitch_rate = quad.pitch_rate - imu_offset_pitch_rate;
  
  filtered_roll = quad.roll - imu_offset_roll;
  filtered_roll_rate = quad.roll_rate - imu_offset_roll_rate;
  filtered_yaw_rate = quad.yaw_rate - imu_offset_yaw_rate;


  angle_pitch = (angle_pitch + (filtered_roll_rate)) * COMP_GAIN + (filtered_roll) * (1 - COMP_GAIN);
  
  error_pitch = (rfMessage.pitch) - (angle_pitch); // - imu_offset_roll);
  derivative_pitch = error_pitch - error_pitch_last;
  error_pitch_last = error_pitch;
  integral_pitch += (error_pitch);
  if(integral_pitch > integral_max)
    integral_pitch = integral_max;
  else if (integral_pitch < -integral_max)
    integral_pitch = -integral_max;
    
//  control_pitch = (pitch_kp * error_pitch) + (pitch_kd * filtered_roll_rate) - (pitch_ki * (integral_pitch));
  control_pitch = (pitch_kp * error_pitch) - (pitch_kd * derivative_pitch) - (pitch_ki * (integral_pitch));





  angle_roll = (angle_roll + (filtered_pitch_rate)) * COMP_GAIN + (filtered_pitch) * (1 - COMP_GAIN);

  error_roll = (-rfMessage.roll) - (angle_roll); // - imu_offset_pitch);
  derivative_roll = error_roll - error_roll_last;
  error_roll_last = error_roll;
//  integral_roll += (roll_ki + error_roll * dt);
  integral_roll += (error_roll);
  if(integral_roll > integral_max)
    integral_roll = integral_max;
  else if (integral_roll < -integral_max)
    integral_roll = -integral_max;
    
//  control_roll = (roll_kp * error_roll) + (roll_kd * filtered_pitch_rate) + (roll_ki * ((integral_roll)));
control_roll = (roll_kp * error_roll) - (roll_kd * derivative_roll) + (roll_ki * (integral_roll));

  if (abs(filtered_yaw_rate) > 1)
    angle_yaw += filtered_yaw_rate;
  error_yaw = rfMessage.yaw - (angle_yaw);  
//  error_yaw = rfMessage.yaw - (quad.yaw_rate - imu_offset_yaw_rate);  
  control_yaw = constrain((yaw_kp * error_yaw) + (yaw_kd * (error_yaw - error_yaw_prev)) + (yaw_ki * ((integral_yaw + error_yaw * dt))),-100,100);
  error_yaw_prev = error_yaw;

  if (rfAvailable()) {
    rfRead((uint8_t *) & rfMessage, sizeof(rfMessage));
//    offset=rfMessage.pot_L;
//    roll_kd = (float) rfMessage.pot_L * 0.05;
//    Serial.println(roll_kp);
//    rfFlush();
//    roll_kd = rfMessage.pot2_rf*0.01;


    #ifdef DEBUG
//      print_rfMessage();
    #endif

    if (rfMessage.throttle > MIN_MOTOR_SPEED && (rfMessage.checksum == checksum)) {      
      signal_loss_count = 0;
      
//      analogWrite(MTR_TL, constrain((int) (rfMessage.throttle*0.75 + offset_TL) + control_pitch - control_roll + control_yaw, 0, MAX_MOTOR_SPEED));
//      analogWrite(MTR_TR, constrain((int) (rfMessage.throttle*0.75 + offset_TR) + control_pitch + control_roll - control_yaw, 0, MAX_MOTOR_SPEED));
//      analogWrite(MTR_BL, constrain((int) (rfMessage.throttle*0.75 + offset_BL) - control_pitch - control_roll - control_yaw, 0, MAX_MOTOR_SPEED));
//      analogWrite(MTR_BR, constrain((int) (rfMessage.throttle*0.75 + offset_BR) - control_pitch + control_roll + control_yaw, 0, MAX_MOTOR_SPEED));
      analogWrite(MTR_TL, constrain((int) (rfMessage.throttle*0.6 + offset_TL) + control_pitch - control_roll + control_yaw, 0, MAX_MOTOR_SPEED));
      analogWrite(MTR_TR, constrain((int) (rfMessage.throttle*0.6 + offset_TR) + control_pitch + control_roll - control_yaw, 0, MAX_MOTOR_SPEED));
      analogWrite(MTR_BL, constrain((int) (rfMessage.throttle*0.6 + offset_BL) - control_pitch - control_roll - control_yaw, 0, MAX_MOTOR_SPEED));
      analogWrite(MTR_BR, constrain((int) (rfMessage.throttle*0.6 + offset_BR) - control_pitch + control_roll + control_yaw, 0, MAX_MOTOR_SPEED));
//      analogWrite(MTR_TL, constrain((int) (rfMessage.throttle*0.75 + offset_TL), 0, MAX_MOTOR_SPEED));
//      analogWrite(MTR_TR, constrain((int) (rfMessage.throttle*0.75 + offset_TR), 0, MAX_MOTOR_SPEED));
//      analogWrite(MTR_BL, constrain((int) (rfMessage.throttle*0.75 + offset_BL), 0, MAX_MOTOR_SPEED));
//      analogWrite(MTR_BR, constrain((int) (rfMessage.throttle*0.75 + offset_BR), 0, MAX_MOTOR_SPEED));

    } else {
      analogWrite(MTR_TR, 0);
      analogWrite(MTR_TL, 0);
      analogWrite(MTR_BL, 0);
      analogWrite(MTR_BR, 0);
    }
  } else {
    signal_loss_count++;
    if (signal_loss_count > signal_loss_max) {
      // TODO: implement a safe landing sequence of events?
      signal_loss_count = 0;
      analogWrite(MTR_TR, 0);
      analogWrite(MTR_TL, 0);
      analogWrite(MTR_BL, 0);
      analogWrite(MTR_BR, 0);      
    }
  } // end rf avail

//  led_flash();

  #ifdef DEBUG
//    graph_data();
    print_imu_data();
  #endif  
}














void calibrate_imu() {
  delay(50);
  imu_offset_pitch = 0;
  imu_offset_roll = 0;
  imu_offset_yaw_rate = 0;
  

  for (byte i = 0; i < 150;) {
    
    imu.read();
    if (ahrs.getQuadOrientation(&quad)) {
      if(i > 49) {
        imu_offset_pitch += quad.pitch;
        imu_offset_pitch_rate += quad.pitch_rate;
        imu_offset_roll += quad.roll;
        imu_offset_roll_rate += quad.roll_rate;
        imu_offset_yaw_rate += quad.yaw_rate;
      }
//      Serial.println(imu_offset_yaw_rate);
      i++;
    }
    delay(1);
  }
  imu_offset_pitch = imu_offset_pitch / 100.0;
  imu_offset_pitch_rate = imu_offset_pitch_rate / 100.0;
  imu_offset_roll = imu_offset_roll / 100.0;
  imu_offset_roll_rate = imu_offset_roll_rate / 100.0;
  imu_offset_yaw_rate = imu_offset_yaw_rate / 100.0;
}

void imu_configuration() {
  // Set data rate for G and XL.  Set G low-pass cut off.  (Section 7.12)
  imu.write8(XGTYPE, Adafruit_LSM9DS1::LSM9DS1_REGISTER_CTRL_REG1_G,  ODR_119 | G_BW_G_10 );  //238hz ODR + 63Hz cuttof // was 238

  // Enable the XL (Section 7.23)
  imu.write8(XGTYPE, Adafruit_LSM9DS1::LSM9DS1_REGISTER_CTRL_REG5_XL, XL_ENABLE_X | XL_ENABLE_Y | XL_ENABLE_Z);

  // Set low-pass XL filter frequency divider (Section 7.25)
  imu.write8(XGTYPE, Adafruit_LSM9DS1::LSM9DS1_REGISTER_CTRL_REG7_XL, HR_MODE | XL_LP_ODR_RATIO_400); // was 400

  // enable mag continuous (Section 8.7)
  imu.write8(MAGTYPE, Adafruit_LSM9DS1::LSM9DS1_REGISTER_CTRL_REG3_M, B00000000); // continuous mode

  // This only sets range of measurable values for each sensor.  Setting these manually (I.e., without using these functions) will cause incorrect output from the library.
  imu.setupAccel(Adafruit_LSM9DS1::LSM9DS1_ACCELRANGE_2G);
  imu.setupMag(Adafruit_LSM9DS1::LSM9DS1_MAGGAIN_4GAUSS);
  imu.setupGyro(Adafruit_LSM9DS1::LSM9DS1_GYROSCALE_500DPS);
}

void test_leds() {
  delay(25);
  digitalWrite(LED_BL,HIGH);
  digitalWrite(LED_BR,HIGH);
  digitalWrite(LED_TL,HIGH);
//  digitalWrite(LED_TR,HIGH);
  delay(25);
  digitalWrite(LED_BL,LOW);
  digitalWrite(LED_BR,LOW);
  digitalWrite(LED_TL,LOW);
//  digitalWrite(LED_TR,LOW);  
}

void led_flash() {
  static bool flashing = true;
  if(flashing) {
    digitalWrite(LED_BL,LOW);
    digitalWrite(LED_BR,LOW);
    digitalWrite(LED_TL,HIGH);
//    digitalWrite(LED_TR,HIGH);
  } else {
  digitalWrite(LED_BL,HIGH);
  digitalWrite(LED_BR,HIGH);
  digitalWrite(LED_TL,LOW);
//  digitalWrite(LED_TR,LOW);
  }
  flashing=!flashing;
}












#ifdef DEBUG
  void print_rfMessage() {
    Serial.print("(throttle, pitch, roll, yaw, pot_L, checksum)=(");
    Serial.print(rfMessage.throttle); Serial.print(", ");
    Serial.print(rfMessage.pitch); Serial.print(", ");
    Serial.print(rfMessage.roll); Serial.print(", ");
    Serial.print(rfMessage.yaw); Serial.print(", ");
    Serial.print(rfMessage.pot_L); Serial.print(", ");
    Serial.print(rfMessage.checksum); Serial.println(")");
  }

  void graph_data() {
    Serial.print(control_roll); Serial.print(" ");
    Serial.print(roll_kp * error_roll); Serial.print(" ");
    Serial.print(roll_kd * quad.pitch_rate); Serial.print(" ");
    Serial.println(roll_ki * integral_roll);
  }

  void print_imu_data() {
//    Serial.print(quad.pitch); Serial.print(" ");
//    Serial.print(quad.roll); Serial.print(" ");
    Serial.println(quad.yaw_rate);
    Serial.println(filtered_yaw_rate);
    Serial.println(imu_offset_yaw_rate);
    
  }
#endif
