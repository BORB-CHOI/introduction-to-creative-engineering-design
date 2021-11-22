#include <Servo.h>

// Arduino pin assignment
#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_TRIG 12
#define PIN_ECHO 13
#define PIN_IR A0

// configurable parameters
#define SND_VEL 346.0 // sound velocity at 24 celsius degree (unit: m/s)
#define _INTERVAL_DIST 200 // USS interval (unit: ms)
#define _INTERVAL_SERVO 20 // servo interval (unit: ms)
#define _INTERVAL_SERIAL 100 // serial interval (unit: ms)
#define _DIST_MIN 90 // minimum distance to be measured (unit: mm)
#define _DIST_MAX 360 // maximum distance to be measured (unit: mm)

#define _DUTY_MIN 1180 // servo full clockwise position (0 degree)
#define _DUTY_NEU 1476 // servo neutral position (90 degree)
#define _DUTY_MAX 1676 // servo full counterclockwise position (180 degree)

// global variables
float timeout; // unit: us
float dist_min, dist_max, dist_center, dist_raw, dist_prev, dist_cali ; // unit: mm
float scale; // used for pulse duration to distance conversion
Servo myservo;
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; // unit: ms
bool event_dist, event_servo, event_serial;

int a, b, x; // unit: mm

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  pinMode(PIN_TRIG,OUTPUT);
  digitalWrite(PIN_TRIG, LOW); 
  pinMode(PIN_ECHO,INPUT);

  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(_DUTY_NEU);

// initialize USS related variables
  dist_min = _DIST_MIN; 
  dist_max = _DIST_MAX;
  dist_center = _DIST_MIN + _DIST_MAX / 2.0;
  timeout = (_INTERVAL_DIST / 2) * 1000.0; // precalculate pulseIn() timeout value. (unit: us)
  dist_raw = dist_prev = 0.0; // raw distance output from USS (unit: mm)
  scale = 0.001 * 0.5 * SND_VEL;  
  

// initialize serial port
  Serial.begin(57600);  

// initialize event variables
  last_sampling_time_dist = last_sampling_time_servo = last_sampling_time_serial = 0;
  event_dist = event_servo = event_serial = false;

  a = 98;
  b = 392;

}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
// wait until next sampling time. 
// millis() returns the number of milliseconds since the program started. Will overflow after 50 days.
  unsigned long time_curr = millis();
  if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
        last_sampling_time_dist += _INTERVAL_DIST;
        event_dist = true;
  }
  if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO) {
        last_sampling_time_servo += _INTERVAL_SERVO;
        event_servo = true;
  }
  if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
        last_sampling_time_serial += _INTERVAL_SERIAL;
        event_serial = true;
  }
    
  if(event_dist) {
    event_dist = false;
    // get a distance reading from the USS
    dist_raw = ir_distance();
    dist_cali = 100 + 300.0 / (b - a) * (dist_raw - a);
  }
  
  if(event_servo) {
    event_servo = false;
// adjust servo position according to the USS read value
    if(dist_cali > 180) myservo.writeMicroseconds(_DUTY_MIN);
    else myservo.writeMicroseconds(_DUTY_MAX);
//    else myservo.writeMicroseconds(_DUTY_NEU);
  }
  
  if(event_serial) {
    event_serial = false;
// output the read value to the serial port
    Serial.print("Min:100,Low:180,raw:");
    Serial.print(dist_cali);
    Serial.print(",servo:");
    Serial.print(myservo.read());  
    Serial.println(",High:220,Max:300");
  }
}

//// get a distance reading from USS. return value is in millimeter.
//float USS_measure(int TRIG, int ECHO)
//{
//  float reading;
//  digitalWrite(TRIG, HIGH);
//  delayMicroseconds(10);
//  digitalWrite(TRIG, LOW);
//  reading = pulseInLong(ECHO, HIGH, timeout) * scale; // unit: mm
//  if(reading < dist_min || reading > dist_max) reading = 0.0; // return 0 when out of range.
//  if(reading == 0.0) reading = dist_prev; // range filter
//  else dist_prev = reading;
//  return reading;
//}
