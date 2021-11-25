#include <Servo.h>   

// Configurable parameters //

// Arduino pin assignment
#define PIN_LED 9          
#define PIN_SERVO 10       
#define PIN_IR A0       

// Framework setting
#define _DIST_TARGET 255 
#define _DIST_MIN 100 
#define _DIST_MAX 410   

// Distance sensor
#define _DIST_ALPHA 0.5    

// Servo range
#define _DUTY_MIN 1000   
#define _DUTY_NEU 1450   
#define _DUTY_MAX 2000

// Servo speed control
#define _SERVO_ANGLE 30       
#define _SERVO_SPEED 30   

// Event periods
#define _INTERVAL_DIST 20
#define _INTERVAL_SERVO 20      
#define _INTERVAL_SERIAL 100 

// PID parameters
#define _KP 2.5   // [20213058]비례이득

// Servo instance 
Servo myservo;    // [20213068] 서보 변수명을 myservo로 선언

// Distance sensor
float dist_target; // location to send the ball
float dist_raw, dist_ema;    // [20213077] 측정값, ema필터 적용값



// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist, event_servo, event_serial;

// Servo speed control
int duty_chg_per_interval; 
int duty_target, duty_curr; 
int a, b;

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;

void setup() {
// initialize GPIO pins for LED and attach servo 
  pinMode(PIN_LED,OUTPUT);
  myservo.attach(PIN_SERVO);

// move servo to neutral position
  myservo.writeMicroseconds(_DUTY_NEU); // [20213078] 서보 중립위치

// initialize serial port
  Serial.begin(57600); // [20213058]

// initialize global variables 
  dist_target = _DIST_TARGET;
  dist_raw = ir_distance();
  error_curr = error_prev = dist_target - dist_raw;

  last_sampling_time_dist = 0; // [20213078] last_sampling_time_dist 초기화
  last_sampling_time_servo = 0; // [20213055]
  last_sampling_time_serial = 0; // [20213080]

  event_dist = event_servo = event_serial = false;

// convert angle speed into duty change per interval.
  duty_chg_per_interval = (float)(_DUTY_MAX - _DUTY_MIN) * _SERVO_SPEED / ((float)_SERVO_ANGLE) * _INTERVAL_SERVO / 1000; // [20213078] 한 주기 당 제어할 최대 duty값 초기화
  a = 70; //70;
  b = 385; //385;
}
  


void loop() {
// Event generator //
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

// Event handlers //

  if(event_dist) {
    event_dist = false; //[20213078]
  // get a distance reading from the distance sensor
    dist_raw = ir_distance_filtered(); // [20213055]

  // PID control logic
    error_curr = dist_target - dist_raw; // [20213075]
    pterm = error_curr; // [20213078]
    control = _KP * pterm; // [20213078]

  // duty_target = f(duty_neutral, control)
    duty_target = _DUTY_NEU + control; // [20213055]

  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
  if(duty_target < _DUTY_MIN){
    duty_target = _DUTY_MIN;
  } // [20213080]
  if(duty_target > _DUTY_MAX){
    duty_target = _DUTY_MAX;
  } // [20213080]

  }
  
  if(event_servo) {
    event_servo = false; // [20213078]

    // adjust duty_curr toward duty_target by duty_chg_per_interval

    // update servo position

// adjust duty_curr toward duty_target by duty_chg_per_interval
  if(duty_target > duty_curr) {
    duty_curr += duty_chg_per_interval;
  if(duty_curr > duty_target) duty_curr = duty_target;
  }
  else {
    duty_curr -= duty_chg_per_interval;
    if(duty_curr < duty_target) duty_curr = duty_target;
  
  } // [20213075]
  
  
  // update servo position
  myservo.writeMicroseconds(duty_curr); // [20213058]
}

  if(event_serial) {
    event_serial = false; // [20213078]
    Serial.print("dist_ir:");
    Serial.print(dist_raw);
    Serial.print(",pterm:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",duty_target:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410"); // [20213071] 
// Min :측정 최소거리 Max : 최대거리 Low : 목표구역 최소거리, dist_target :기준이 되는 거리, Low : 목표구역 최소거리 값을 시리얼 모니터에 표시

  }
}

float ir_distance(void){ // return value unit: mm
  int a = 70;
  int b = 300;
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  val = 100 + 300.0 / (b - a) * (val - a);
  return val;
}
float ir_distance_filtered(void){ // return value unit: mm
  float raw_dist = ir_distance();
  float dist_ema = 100 + 300.0 / (b - a) * (raw_dist - a);// for now, just use ir_distance() without noise filter.
  return dist_ema;
}
