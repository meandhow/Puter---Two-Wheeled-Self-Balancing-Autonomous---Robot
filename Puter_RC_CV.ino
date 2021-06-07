

#include <Wire.h>                                            //Include the Wire.h library so we can communicate with the gyro
#include <PID_v1.h>

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>


#define THROTTLE_SIGNAL_IN 9 // INTERRUPT 0 = DIGITAL PIN 2 - use the interrupt number in attachInterrupt
#define THROTTLE_SIGNAL_IN_PIN 9 // INTERRUPT 0 = DIGITAL PIN 2 - use the PIN number in digitalRead

#define NEUTRAL_THROTTLE 1500 // this is the duration in microseconds of neutral throttle on an electric RC Car

#define DIR_SIGNAL_IN 10 // INTERRUPT 0 = DIGITAL PIN 2 - use the interrupt number in attachInterrupt
#define DIR_SIGNAL_IN_PIN 10 // INTERRUPT 0 = DIGITAL PIN 2 - use the PIN number in digitalRead

#define NEUTRAL_DIR 1500 // this is the duration in microseconds of neutral throttle on an electric RC Car

#define KNOB_SIGNAL_IN 12 // INTERRUPT 0 = DIGITAL PIN 2 - use the interrupt number in attachInterrupt
#define KNOB_SIGNAL_IN_PIN 12 // INTERRUPT 0 = DIGITAL PIN 2 - use the PIN number in digitalRead

#define NEUTRAL_KNOB 1500 // this is the duration in microseconds of neutral throttle on an electric RC Car

volatile int nThrottleIn = NEUTRAL_THROTTLE; // volatile, we set this in the Interrupt and read it in loop so it must be declared volatile
volatile unsigned long ulStartPeriod = 0; // set in the interrupt
volatile boolean bNewThrottleSignal = false; // set in the interrupt and read in the loop

volatile int nDirIn = NEUTRAL_DIR; // volatile, we set this in the Interrupt and read it in loop so it must be declared volatile
volatile unsigned long ulStartPeriodDir = 0; // set in the interrupt
volatile boolean bNewDirSignal = false; // set in the interrupt and read in the loop

volatile int nKnobIn = NEUTRAL_KNOB; // volatile, we set this in the Interrupt and read it in loop so it must be declared volatile
volatile unsigned long ulStartPeriodKnob = 0; // set in the interrupt
volatile boolean bNewKnobSignal = false; // set in the interrupt and read in the loop

#include <ODriveArduino.h>
float pid_signal;
// Printing with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) {
  obj.print(arg);
  return obj;
}
template<>        inline Print& operator <<(Print &obj, float arg) {
  obj.print(arg, 4);
  return obj;
}

bool dir_bool = true;
bool dist_bool = true;
bool butt1 = 0;
bool butt2 = 0;
bool butt3 = 0;
float Knob_pot;
float pot_angle;
int horP = 200;
int horI = 0;
int horD = 0;
int distP = 10;
int distI = 0;
int distD = 0;
int vepil = 0;
//String data;
int dirINT;
int distINT;
float CV_vel_setpoint = 0;
float CV_setpoint = 0;
float CV_vel = 0;

boolean distFLAG_FOR = 0;
boolean distFLAG_BACK = 0;

double Setpoint, Input, Output;
PID horPID(&Input, &Output, &Setpoint, horP, horI, horD, DIRECT);
double SetpointDist, InputDist, OutputDist;
PID distPID(&InputDist, &OutputDist, &SetpointDist, distP, distI, distD, REVERSE);
// ODrive object
ODriveArduino odrive(Serial1);

long vel0;
long vel1;
long ave_vel = 0;
long vel0past = 0;
float TF = 0.1;
int pos0;
int pos1;
float curr0;
float curr1;

int gyro_address = 0x68;                                     //MPU-6050 I2C address (0x68 or 0x69)
float yaw;
float pitch;
float roll;

RF24 radio(9, 10) ; // CE, CSN
const byte address[6] = "00001";

float pid_setpoint = -1;
int vel_setpoint = 0;
float kp = 0.5; //10000                                      //Gain setting for the P-controller (15)
float kd = 120; //1500  100                                   //Gain setting for the I-controller (1.5)
float ki = 0.03; //0.01 //100                                     //Gain setting for the D-controller (30
float vel_kp = 0.08;
float vel_kd = 0;
float vel_ki = 0; //0.001
long counter = 0;
float pid_adjustment = 0;
float lastError;
float cumError = 0;
float cumErrorUse = 0;
float previousTimePID;

float vel_lastError;
float vel_cumError = 0;
float vel_cumErrorUse = 0;

float vel_pitch = 0;

int rec_CV;
long rc_values[2];
int knob1;
int knob2;
int butt;
int buttState = 0;
int pot;

float RC_setpoint = 0;
float RC_vel_setpoint = 0;

float left_motor_signal = 0;
float right_motor_signal = 0;



//motor 0 is left, motor 1 is right


// ================================================================
// ===               MPU6050                ===
// ================================================================

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
   ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
   ========================================================================= */



// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT



#define INTERRUPT_PIN 14  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               MPU INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}



// ================================================================
// ===                      MPU INITIAL SETUP                       ===
// ================================================================

void MPUInitialize() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(115200);
  //while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
  // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));


  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(-3491);
  mpu.setYGyroOffset(-20);
  mpu.setZGyroOffset(-41);
  mpu.setZAccelOffset(1014); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
}



// ================================================================
// ===                    MPU MAIN LOOP                     ===
// ================================================================

void readAngles() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    yaw = ypr[0] * 180 / M_PI;
    pitch = ypr[1] * 180 / M_PI;
    roll = ypr[2] * 180 / M_PI;
    /* good
    Serial.print("ypr ");
    Serial.print(yaw);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.println(roll); */
#endif

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
}

// ================================================================
// ===               ODRIVE                ===
// ================================================================


void OdriveInitiation() {
  // ODrive uses 115200 baud
  Serial1.setRX(27);
  Serial1.setTX(1);
  Serial1.begin(115200);


  //while (!Serial) ; // wait for Arduino Serial Monitor to open

  //Serial.println("ODriveArduino");
  //Serial.println("Setting parameters...");

  // In this example we set the same parameters to both motors.
  // You can of course set them different if you want.
  // See the documentation or play around in odrivetool to see the available parameters
  for (int axis = 0; axis < 2; ++axis) {
    Serial1 << "w axis" << axis << ".controller.config.vel_limit " << 500000.0f << '\n';
    Serial1 << "w axis" << axis << ".motor.config.current_lim " << 50.0f << '\n';
    Serial1 << "w axis" << axis << ".motor.config.calibration_current " << 20.0f << '\n';
    //odrive_serial << "w axis" << axis <<".motor.config.pre_calibrated = True"<<'\n'; //tells odrive its motor is calibrated
    //odrive_serial << "w axis" << axis <<".encoder.config.pre_calibrated = True"<<'\n';
    // This ends up writing something like "w axis0.motor.config.current_lim 10.0\n"
  }

  //ODRIVE calibration
  for (int c = 0; c < 2; c++) {
    int motornum = c;
    int requested_state;
    
    requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
    Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
    odrive.run_state(motornum, requested_state, false);
    delay(10000);
    requested_state = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
    Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
    odrive.run_state(motornum, requested_state, false);
    delay(10000); 
    /*requested_state = ODriveArduino::AXIS_STATE_FULL_CALIBRATION_SEQUENCE;
    Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
    odrive.run_state(motornum, requested_state, false);
    delay(10000); */
    requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
    Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
    odrive.run_state(motornum, requested_state, false); // don't wait
    Serial.print("Motor: ");
    Serial.print(c);
    Serial.println(" calibrated");
  } 
}

void OdriveWrite() {
  RC_vel_setpoint /= 10000;
  CV_vel_setpoint = CV_vel/10000;
  float left_motor_curr;
  float right_motor_curr;
  if(butt3 == 1){
    left_motor_curr = (pid_signal - RC_vel_setpoint - CV_vel_setpoint); //+ RC_vel_setpoint //map(left_motor_signal, -1000, 10000, -22000, 22000);  //we can either use mapping here or tweak with PID
    right_motor_curr = -(pid_signal + RC_vel_setpoint + CV_vel_setpoint); //map(right_motor_signal, -10000, 10000, -22000, 22000); //gain values to amplify the PID output signal so
  }
  else{
    left_motor_curr = (pid_signal - RC_vel_setpoint); //+ RC_vel_setpoint //map(left_motor_signal, -1000, 10000, -22000, 22000);  //we can either use mapping here or tweak with PID
    right_motor_curr = -(pid_signal + RC_vel_setpoint); //map(right_motor_signal, -10000, 10000, -22000, 22000); //gain values to amplify the PID output signal so
  }
  //float left_motor_curr = (pid_signal - RC_vel_setpoint - CV_vel_setpoint); //+ RC_vel_setpoint //map(left_motor_signal, -1000, 10000, -22000, 22000);  //we can either use mapping here or tweak with PID
  //float right_motor_curr = -(pid_signal + RC_vel_setpoint + CV_vel_setpoint); //map(right_motor_signal, -10000, 10000, -22000, 22000); //gain values to amplify the PID output signal so
  if (abs(left_motor_curr) > 0.2) {
    odrive.SetCurrent(0, left_motor_curr);
    odrive.SetCurrent(1, right_motor_curr);                                      // it is within the velocity range of ODRIVE
  }

  vel0 = odrive.GetVelocity(0);
  vel1 = odrive.GetVelocity(1);
  ave_vel = abs(vel0 - vel1)/2;
  //Serial.print("ave_vel: ");
  //Serial.println(ave_vel);
  //Serial.print("vel");
  //Serial.print(ave_vel);

  /*good
  Serial.print(" Curr left: ");
  Serial.print(left_motor_curr);
  Serial.print(" Curr right: ");
  Serial.print(-right_motor_curr);
  Serial.print(" Vel left: ");
  Serial.print(vel0);
  Serial.print(" Vel right: ");
  Serial.println(-vel1); */

  
  /*Serial.print(" curr0: ");
  Serial.print(curr0);
  Serial.print(" curr1: ");
  Serial.println(curr1);*/
}


// ================================================================
// ===               RADIO COMMUNICATION                ===
// ================================================================

void calcInput()
{
  // if the pin is high, its the start of an interrupt
  if(digitalRead(THROTTLE_SIGNAL_IN_PIN) == HIGH)
  {
    // get the time using micros - when our code gets really busy this will become inaccurate, but for the current application its
    // easy to understand and works very well
    ulStartPeriod = micros();
  }
  else
  {
    // if the pin is low, its the falling edge of the pulse so now we can calculate the pulse duration by subtracting the
    // start time ulStartPeriod from the current time returned by micros()
    if(ulStartPeriod && (bNewThrottleSignal == false))
    {
      nThrottleIn = (int)(micros() - ulStartPeriod);
      ulStartPeriod = 0;

      // tell loop we have a new signal on the throttle channel
      // we will not update nThrottleIn until loop sets
      // bNewThrottleSignal back to false
      bNewThrottleSignal = true;
    }
  }
}

void calcInputDir()
{
  // if the pin is high, its the start of an interrupt
  if(digitalRead(DIR_SIGNAL_IN_PIN) == HIGH)
  {
    // get the time using micros - when our code gets really busy this will become inaccurate, but for the current application its
    // easy to understand and works very well
    ulStartPeriodDir = micros();
  }
  else
  {
    // if the pin is low, its the falling edge of the pulse so now we can calculate the pulse duration by subtracting the
    // start time ulStartPeriod from the current time returned by micros()
    if(ulStartPeriodDir && (bNewDirSignal == false))
    {
      nDirIn = (int)(micros() - ulStartPeriodDir);
      ulStartPeriodDir = 0;

      // tell loop we have a new signal on the throttle channel
      // we will not update nThrottleIn until loop sets
      // bNewThrottleSignal back to false
      bNewDirSignal = true;
    }
  }
}

void calcInputKnob()
{
  // if the pin is high, its the start of an interrupt
  if(digitalRead(KNOB_SIGNAL_IN_PIN) == HIGH)
  {
    // get the time using micros - when our code gets really busy this will become inaccurate, but for the current application its
    // easy to understand and works very well
    ulStartPeriodKnob = micros();
  }
  else
  {
    // if the pin is low, its the falling edge of the pulse so now we can calculate the pulse duration by subtracting the
    // start time ulStartPeriod from the current time returned by micros()
    if(ulStartPeriodKnob && (bNewKnobSignal == false))
    {
      nKnobIn = (int)(micros() - ulStartPeriodKnob);
      ulStartPeriodKnob = 0;

      // tell loop we have a new signal on the throttle channel
      // we will not update nThrottleIn until loop sets
      // bNewThrottleSignal back to false
      bNewKnobSignal = true;
    }
  }
}

void radioInitialize() {
  attachInterrupt(THROTTLE_SIGNAL_IN,calcInput,CHANGE);
  attachInterrupt(DIR_SIGNAL_IN,calcInputDir,CHANGE);
  attachInterrupt(KNOB_SIGNAL_IN,calcInputKnob,CHANGE);
  pinMode(A9, INPUT);
  Serial.begin(115200);
  pinMode(36, INPUT);
  pinMode(37, INPUT);
  pinMode(38, INPUT);
}

void radioReceive() {
  pot = analogRead(A9);
  butt1 = digitalRead(36);
  butt2 = digitalRead(37);
  butt3 = digitalRead(38);

  pot_angle = map(pot, 0, 1023, -2, 2);
  //RC_vel_setpoint = map(nDirIn, 1000, 1900, -30000, 30000);
  
   // if a new throttle signal has been measured, lets print the value to serial, if not our code could carry on with some other processing
 if(bNewThrottleSignal)
 {

   //Serial.println((String)" Throttle: " + nThrottleIn);
   

   // set this back to false when we have finished
   // with nThrottleIn, while true, calcInput will not update
   // nThrottleIn
   bNewThrottleSignal = false;
 }
 if(bNewDirSignal)
 {

   //Serial.println((String) " Dir: " + nDirIn);
   

   // set this back to false when we have finished
   // with nThrottleIn, while true, calcInput will not update
   // nThrottleIn
   bNewDirSignal = false;
 }
 
 if(bNewKnobSignal)
 {

   //Serial.println((String) " Dir: " + nDirIn);
   

   // set this back to false when we have finished
   // with nThrottleIn, while true, calcInput will not update
   // nThrottleIn
   bNewKnobSignal = false;
 }

    if (nDirIn > 1510 || nDirIn < 1490) {
      RC_vel_setpoint = map(nDirIn, 1080, 1920, -30000, 30000);
    }
    else {
      RC_vel_setpoint = 0;
    }
    
    
    if (nThrottleIn > 1520 || nThrottleIn < 1480) {
      RC_setpoint = map(nThrottleIn, 1060, 1910, -40000, 40000);
    }
    else {
      RC_setpoint = 0;
    }
    
    if (nKnobIn > 1510 || nKnobIn < 1490) {
      Knob_pot = map(nKnobIn, 1080, 1920, -150, 150);
      Knob_pot /= 100;
    }
    else {
      Knob_pot = 0;
    }
    
 

  

  //Serial.print((String)"Knob1: " + RC_setpoint + " , ");
  //Serial.print((String)"Knob2: " + RC_vel_setpoint + " , ");
  
}

// ================================================================
// ===               PID                ===
// ================================================================
String getValue(String data, char separator, int index)
{
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void AI_PIDsetup(){
  Setpoint = 0;
  horPID.SetMode(AUTOMATIC);
  horPID.SetOutputLimits(-20000, 20000);
  Serial.setTimeout(5);
  SetpointDist = 77;
  distPID.SetMode(AUTOMATIC);
  distPID.SetOutputLimits(-200, 250);
}
void AI_PIDupdate(){
  if (Serial.available() > 0) {
    String data = Serial.readString();
    String dist = getValue(data, ':', 0);
    String dir = getValue(data, ':', 1);
    //String dir = Serial.readStringUntil("/n");
    dirINT = dir.toInt();
    distINT = dist.toInt();
    //distINT = 90;
    Serial.print("DIR:");
    Serial.print(dirINT);
    Serial.print(",");
    Serial.print("DIST:");
    Serial.print(distINT);
    Serial.print(",");
    
    
    if (dir_bool == true){
    Input = dirINT;
    horPID.Compute();
    CV_vel = Output;
    Serial.print("VEL:");
    Serial.print(CV_vel);
    Serial.print(",");
    
    if (ave_vel != 0){

      if (ave_vel < 20000){
      CV_vel = CV_vel;
      }
      if (ave_vel > 20000 && ave_vel < 50000){
      CV_vel = CV_vel / 2;
      }
      if (ave_vel >50000){
      CV_vel = CV_vel / 4;
      }
    
      
    }
    
    
    //Serial.print("You sent me: ");
    //Serial.println(CV_vel);
    }
    if (distINT <= 50 ){
      distFLAG_FOR = 1;
    }
    else if (distINT >= 80){
      distFLAG_FOR = 0;

    }

    if (distINT >= 95 ){
      distFLAG_BACK = 1;
    }
    else if (distINT <= 80){
      distFLAG_BACK = 0;

    }

    if (dist_bool == true){
      InputDist = distINT;
      distPID.Compute();
      CV_setpoint = OutputDist/100;
    }
    /*
    if (dist_bool == true){
      if (distFLAG_FOR = 1 && distINT < 65){
        CV_setpoint = -2;
        //Serial.print("CV_set: ");
        //Serial.println(CV_setpoint);
        Serial.print("CV");
        Serial.print(CV_setpoint);
        Serial.print(",");
      }
      else if(distFLAG_FOR = 1 && distINT >= 65 && distINT < 75){
        CV_setpoint = -1;
        //Serial.print("CV_set: ");
        //Serial.println(CV_setpoint);
        Serial.print("CV");
        Serial.print(CV_setpoint);
        Serial.print(",");
      }

      else if(distFLAG_FOR = 1 && distINT >= 75 && distINT < 80){
        CV_setpoint = 0;
        //Serial.print("CV_set: ");
        //Serial.println(CV_setpoint);
        Serial.print("CV");
        Serial.print(CV_setpoint);
        Serial.print(",");
      }
      
      

      if (distFLAG_BACK = 1 && distINT > 85){
        CV_setpoint = 1;
        //Serial.print("CV_set: ");
        //Serial.println(CV_setpoint);
        Serial.print("CV");
        Serial.print(CV_setpoint);
        Serial.print(",");
      }
      else if (distFLAG_BACK = 1 && distINT > 80 && distINT <= 85){
        CV_setpoint = 0;
        //Serial.print("CV_set: ");
        //Serial.println(CV_setpoint);
        Serial.print("CV");
        Serial.print(CV_setpoint);
        Serial.print(",");
      }
      
    }*/
    Serial.println("END");
  }
  
}
float computePID() {
  int currentTimePID = millis();
  if (counter = 0) {
    previousTimePID = currentTimePID;
  }
  int elapsedTime = currentTimePID - previousTimePID;
  counter += 1;

  RC_setpoint /= 10000;
  float error;
  if(butt3 == 1){
    error =  pid_setpoint + RC_setpoint + CV_setpoint + pitch ; // - vel_pid_output + vel_pid_output + pid_adjustment + vel_pid_output + pid_input_MC
  }
  else{
    error =  pid_setpoint + RC_setpoint + pitch ; // - vel_pid_output + vel_pid_output + pid_adjustment + vel_pid_output + pid_input_MC
  }
  if( butt2 == 1){
    cumError += error * elapsedTime /10 ;
  }
  else{
    cumError = 0;
  }
  
  if (cumError > 1000) {
    cumErrorUse = 1000;
  }
  else if (cumError < -1000) {
    cumErrorUse = -1000;
  }
  else {
    cumErrorUse = cumError;
  }

  float rateError = (error - lastError) / elapsedTime;
  float pid_output;
  if(butt1 == 1){
    pid_output = kp * error + ki * cumErrorUse + kd * rateError;
  }
  else {
    pid_output = 0;
  }
  /*
  Serial.print(" cumError: ");
  Serial.print(cumError);
  Serial.print(" , ");
  Serial.print(" kiAct: ");
  Serial.print(ki * cumErrorUse);
  Serial.print(" , ");
  Serial.print(" kpAct: ");
  Serial.print(kp * error);
  Serial.print(" , ");
  Serial.print(" pidAdj: ");
  Serial.print(pid_adjustment);
  */
  /*Serial.print(" velError: ");
  Serial.print(vel_error);
  Serial.print(" velPID: ");
  Serial.print(vel_pid_output);*/

  /* GOOD
  Serial.print(" cumError: ");
  Serial.print(cumError);
  Serial.print(" Ki: ");
  Serial.print(cumErrorUse * ki);
  
  Serial.print(" kpAct: ");
  Serial.print(kp * error);
  Serial.print(" KD: ");
  Serial.print(kd * rateError);
  Serial.print(" PID: ");
  Serial.print(pid_output);
  Serial.print(" T: ");
  Serial.print(elapsedTime); */
  previousTimePID = currentTimePID;
  lastError = error;
  vel0past = vel0;
  //vel_lastError = vel_error;
  return pid_output;


}
void computeWheelVelocity(int pid_output) {
  //int total_input = side_input_MC; //+ side_input_CV;
  //left_motor_signal = pid_output + total_input; //map MC and CV values respectively so that the pos on the left is negative and on the right positive
  //right_motor_signal = pid_output - total_input;
}

// ================================================================
// ===               ACTUAL PROGRAM                ===
// ================================================================


void setup() {
  Serial.begin(115200);                                                       //Start the serial port at 9600 kbps
  Wire.begin();                                                             //Start the I2C bus as master
  MPUInitialize();
  radioInitialize();
  OdriveInitiation(); // initialize and calibrate
  AI_PIDsetup();
  readAngles();
}


void loop() {

  radioReceive();
  readAngles();
  pid_signal = computePID();
  AI_PIDupdate();
  //computeWheelVelocity(pid_signal);
  OdriveWrite();
  //OdriveRead();

}
