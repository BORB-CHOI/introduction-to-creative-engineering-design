#include <Servo.h>

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9            // [20200000] LED핀 설정
#define PIN_SERVO 10         //[20191979] Servo 핀 설정
#define PIN_IR A0            // [20191979] 적외선 핀(아날로그핀)

// Framework setting
#define _DIST_TARGET 255    // [20213085] 정지하려는 위치 목표값
#define _DIST_MIN 100       //[20191979] 최소거리
#define _DIST_MAX 410       //[20191979] 최대거리

// Distance sensor
#define _DIST_ALPHA 0.07 //[20203118] DIST_ALPHA 값 설정

// Servo range
#define _DUTY_MIN 1170    // [20213083]서보 각도의 최솟값 설정
#define _DUTY_NEU 1550    // [20213090]서보 수평 각도 펄스 값
#define _DUTY_MAX 1950    // [20213081] 서보 각도의 최댓값

// Servo speed control
#define _SERVO_ANGLE 30.0
#define _SERVO_SPEED 40.0

// Event periods
#define _INTERVAL_DIST 20   // [20213099] 거리 센서 주기 설정
#define _INTERVAL_SERVO 20  // [20213099] 서보 주기 설정
#define _INTERVAL_SERIAL 100  // [20213099] 시리얼 표시 주기 설정

// PID parameters
#define _KP 1
#define _KD 60.0

#define a 100
#define b 330

//////////////////////
// global variables //
//////////////////////

// Servo instance
Servo myservo;

// Distance sensor
float dist_target;
float dist_raw, dist_ema;

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist, event_servo, event_serial; 

// Servo speed control
int duty_chg_per_interval;
int duty_target, duty_curr; 

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;


void setup() {
  // initialize GPIO pins for LED and attach servo 
  pinMode(PIN_LED, OUTPUT);
  
  // move servo to neutral position
  myservo.attach(PIN_SERVO);
  duty_target = duty_curr = _DUTY_NEU;
  myservo.writeMicroseconds(duty_curr);

  // initialize global variables
  duty_curr=_DUTY_MIN;
  dist_raw = dist_ema = ir_distance_filtered();
  error_curr = error_prev = dist_target - dist_ema;
  dist_target = _DIST_TARGET;

  last_sampling_time_dist = last_sampling_time_servo = last_sampling_time_serial = 0;
  event_dist = event_servo = event_serial = false;

  // initialize serial port
  Serial.begin(57600);

  // convert angle speed into duty change per interval.
  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * ((float)_SERVO_SPEED / _SERVO_ANGLE) * (_INTERVAL_SERVO / 1000.0);
}
  


void loop() {
/////////////////////
// Event generator //
/////////////////////

  unsigned long time_curr = millis();
  if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST){
    last_sampling_time_dist += _INTERVAL_DIST;
    event_dist = true;
  }

  if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO){
    last_sampling_time_servo += _INTERVAL_SERVO;
    event_servo = true;
  }

  if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL){
    last_sampling_time_serial += _INTERVAL_SERIAL;
    event_serial = true;
  }

////////////////////
// Event handlers //
////////////////////

  if(event_dist) {
    event_dist = false;
    
    // get a distance reading from the distance sensor
    dist_raw = ir_distance_filtered();
    dist_ema = (1 - _DIST_ALPHA) * dist_ema + _DIST_ALPHA * dist_raw;

    // PID control logic
    error_curr = dist_target - dist_ema;
    pterm = _KP * error_curr;  
    dterm = _KD * (error_curr - error_prev);
    control = pterm + dterm; 

    duty_target = _DUTY_NEU + control;

    // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    if(duty_target > _DUTY_MAX){duty_target = _DUTY_MAX;}
    if(duty_target < _DUTY_MIN){duty_target = _DUTY_MIN;}

    error_prev = error_curr;
  }

  
  if(event_servo) {
    event_servo=false;
    if(duty_target > duty_curr) {
      duty_curr += duty_chg_per_interval;
      if(duty_curr > duty_target) duty_curr = duty_target;
    }
    else {
      duty_curr -= duty_chg_per_interval;
      if(duty_curr < duty_target) duty_curr = duty_target;
    } 
    myservo.writeMicroseconds(duty_curr); 
  }

  
  if(event_serial) {
    event_serial = false; //[20203118]
    Serial.print("dist_ir:");
    Serial.print(dist_ema);
    Serial.print(",pterm:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",dterm:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",duty_target:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");
  }
}


float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  val = 100 + 300.0 / (b-a) * (val - a);
  return val;
}

float ir_distance_filtered(void){ // return value unit: mm
  return ir_distance(); // for now, just use ir_distance() without noise filter.
}
