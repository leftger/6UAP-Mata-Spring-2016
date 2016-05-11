/******************************************************************************
  MIT 6.UAP Spring 2016
  Swing Feedback System
  Gerzain Mata

  The primary objective of this code is to attempt to demonstrate feedback with
  respect to a reference swing in 3D space.

  In our scenario an experienced batter records his/her swing by holding the
  rocker switch.  The bat is now in the RECORD state.  An Adafruit 9DOF IMU gets
  the acceleration, gyroscope, and magnetometer measurements and are passed to
  the Atmega328P.  The MCU then implements Madgwick's IMU algorithm which
  provides roll, pitch, and yaw values with fast convergence and no noticeable
  drift.

  The measurements are then saved to a Cypress FRAM module since the MCU doesn't
  have sufficient memory space to save all the measurements.  The Cypress FRAM
  module allows the measurements to persist even after the bat is turned off.

  When the user lets go of the rocker switch the bat then goes into the START
  state whereby it waits for user input on the GREEN button to enter the DELAY
  state.

  When the bat goes into the DELAY state the bat waits for two seconds so that
  the user can prepare his/herself to execute their own "novice" swing. After
  two seconds the bat goes into the PLAYBACK state.

  During PLAYBACK the MCU starts reading the FRAM from the beginning of
  the memory space.  The current Yaw, Pitch, and Roll values are compared to the
  Yaw, Pitch, and Roll values from the recorded values and a PID controller is
  implemented for each rotation axis.

  Once the user finishes his/her swing in PLAYBACK mode then the bat returns to
  the START state.

  RGB LED states:
  START     --> BLUE
  DELAY     --> ORANGE
  PLAYBACK  --> GREEN
  RECORD    --> RED

  Arduino Pinout:
  SCL       --> Adafruit 9DOF SCL
  SDA       --> Adafruit 9DOF SDA
  D9        --> GREEN LED (active low)
  D10       --> RED LED (active low)
  D11       --> BLUE LED (active low)
  D2        --> Rocker Switch (w/pullup)
  D3        --> Green Pushbutton (w/pullup)
******************************************************************************/
#include <math.h> // sines,cosines and all the math Euler came up with
#include "CYI2CFRAM.h" // FRAM read/write
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_9DOF.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_MotorShield.h>
#include <MadgwickAHRS.h>

// Constants for motor drive
#define MAX_PWM 150
#define MIN_PWM -150
#define GREEN_LED_PIN 9
#define RED_LED_PIN 10
#define BLUE_LED_PIN 11
#define ROCKER_PIN 3
#define GREEN_BTN_PIN 2
#define NUM_MOTORS 4

// FRAM constants
#define NUM_SAMPLES_READ 100 // try to read the first 100 gyro measurements
#define FRAM_SLAVE_SRAM_ADDR    (0x50)  // F-RAM Slave Address
#define FAIL                    (0u)    // Macro for F-RAM Read/Write PASS/FAIL
#define PASS                    (1u)
#define COM_ERR()  for(;;){}            // I2C Communication Error

// constants for Madgwick and Mahony filters
#define betaDef      3.5f      // 2 * proportional gain
#define sampleFreq  60.0f      // sample frequency in Hz
#define twoKpDef  (2.0f * 0.5f) // 2 * proportional gain
#define twoKiDef  (2.0f * 0.0f) // 2 * integral gain

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

/* Assign a unique ID to the sensors */
Adafruit_9DOF                 dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);
Adafruit_DCMotor *motors[4]; // list of pointers to the motor instances so that they can be
// referenced more easily
double motor_drives[4]; // array of floats containing the PWM values to drive motors along
// with directionality

uint8_t *toFram;
unsigned int i;
volatile uint32_t readFramAddress;
volatile uint32_t writeFramAddress;
uint8_t sts;

volatile bool btnPrev; // previous pushbutton state
volatile unsigned long timeBtnPrev;
float samplefreq;
unsigned long last_time;
unsigned long new_time;
volatile unsigned long delayStart;
float dt;
float minima;

sensors_event_t accel_event;
sensors_event_t mag_event;
sensors_event_t gyro_event;
sensors_vec_t   orientation;

enum current_state {START, RECORD, PLAYBACK, DELAY};
volatile enum current_state myState;

Madgwick myIMU(betaDef, sampleFreq);

class PIDControl
{
    float ki, kp, kd;
    float desired;
    float current_meas, prev_error;
    float error, error_deriv, error_integral, output;

  public:
    PIDControl(): PIDControl(2.0f, 0.1f, 0.5f) {}
    PIDControl(float kp, float ki, float kd) {
      this->kp = kp;
      this->ki = ki;
      this->kd = kd;
    }

    void updateState(float newDesired, float current) {
      desired = newDesired;
      prev_error = error;
      current_meas = current;
      error = desired - current_meas; // current error
      error_integral += error * dt * ki;
      if (error_integral > MAX_PWM) error_integral = MAX_PWM;
      else if (error_integral < MIN_PWM) error_integral = MIN_PWM;
      error_deriv = (error - prev_error) / dt;
      output = kp * error + error_integral + kd * error_deriv;
    }

    float getOutput() {
      return output;
    }
};

PIDControl rollPid;
PIDControl pitchPid(4.0f, 0.1f, 0.5f);
PIDControl yawPid(2.0f, 0.1f, 0.5f);

union framFloatToBytes {
  float f;
  uint8_t b[sizeof(float)];
};

float invSqrt(float x) {
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i >> 1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

// calculate the pwm drive and direction for each motor given a theta (theta should be from 0 to 360);
void getPWMForMotors(double degree, double* pwms) {
  float horizontal = cos(degreesToRadians(degree));
  float vertical = sin(degreesToRadians(degree));
  if (horizontal >= 0) {
    pwms[0] = MAX_PWM * horizontal;
    pwms[2] = 0;
  }
  else {
    pwms[2] = MAX_PWM * horizontal;
    pwms[0] = 0;
  }
  if (vertical >= 0) {
    pwms[1] = MAX_PWM * vertical;
    pwms[3] = 0;
  }
  else {
    pwms[3] = MAX_PWM * vertical;
    pwms[1] = 0;
  }
}

double degreesToRadians(double degree) {
  return (degree * 71) / 4068;
}

void initSensors()
{
  if (!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while (1);
  }
  if (!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while (1);
  }

  /* Initialise the sensor */
  if (!gyro.begin())
  {
    /* There was a problem detecting the L3GD20 ... check your connections */
    Serial.println("Ooops, no L3GD20 detected ... Check your wiring!");
    while (1);
  }

  //  if (!bme.begin()) {
  //    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
  //    while (1);
  //  }
}

void doFRAMStuff() {
  //uint8_t fram_rd_current_byte;
  uint8_t i;
  uint8_t result;
  uint32_t no_of_bytes_read;
  framFloatToBytes ff2b;

  Serial.print("\nReading first N samples from FRAM");
  Serial.print("\n-------------------------------\n");

  // Start-up Delay
  delay(100);

  result = PASS;
  Serial.print("\n\nRead Data\n");

  for (i = 0; i < NUM_SAMPLES_READ; i++) {
    if (!i) {
      no_of_bytes_read = FRAM_I2C_Random_Read(FRAM_SLAVE_SRAM_ADDR, readFramAddress, reinterpret_cast<uint8_t *>(ff2b.b), sizeof(float));
      readFramAddress += 4;
    }
    else {
      no_of_bytes_read = FRAM_I2C_Current_Read(FRAM_SLAVE_SRAM_ADDR, reinterpret_cast<uint8_t *>(ff2b.b), sizeof(float));
      readFramAddress += 4;
    }
    if (no_of_bytes_read == sizeof(float))
    {
      result = PASS;
      Serial.print("Roll: ");
      Serial.print(ff2b.f);
      readFramAddress += 4;
    }
    no_of_bytes_read = FRAM_I2C_Current_Read(FRAM_SLAVE_SRAM_ADDR, reinterpret_cast<uint8_t *>(ff2b.b), sizeof(float));
    if (no_of_bytes_read == sizeof(float))
    {
      result = PASS;
      Serial.print(" Pitch: ");
      Serial.print(ff2b.f);
      readFramAddress += 4;
    }
    no_of_bytes_read = FRAM_I2C_Current_Read(FRAM_SLAVE_SRAM_ADDR, reinterpret_cast<uint8_t *>(ff2b.b), sizeof(float));
    if (no_of_bytes_read == sizeof(float))
    {
      result = PASS;
      Serial.print(" Yaw: ");
      Serial.println(ff2b.f);
      readFramAddress += 4;
    }
  }


  Serial.print("\n\n-----------------------------");
  Serial.println("\nF-RAM Read N samples End\n");

}

void printIMUOutput() {
//  Serial.print(F("Orientation: "));
//  Serial.print(F("Roll:"));
  Serial.print(F(" "));
  Serial.print(orientation.roll);
//  Serial.print(F(" Pitch:"));
  Serial.print(F(" "));
  Serial.print(orientation.pitch);
//  Serial.print(F(" Yaw:"));
  Serial.print(F(" "));
  Serial.print(orientation.heading);
//  Serial.print(F(" SampleFreq:"));
  Serial.print(F(" "));
  Serial.print(samplefreq);
//  Serial.print(" Fram address: ");
//  Serial.print(writeFramAddress);
  Serial.println(F(""));
}

void releaseMotors(){
  for (int y = 0; y < 4; y++) {
       motors[y]->run(RELEASE);
     }
}

void updateIMUAndDoMadgwick() {
  /* Read the accelerometer and magnetometer */
  accel.getEvent(&accel_event);
  mag.getEvent(&mag_event);
  gyro.getEvent(&gyro_event);

  float  gx = gyro_event.gyro.x;
  float  gy = gyro_event.gyro.y;
  float  gz = gyro_event.gyro.z;

  /* Get a new sensor event */
  float  ax = (accel_event.acceleration.x) *101 ;
  float  ay = accel_event.acceleration.y * 101;
  float  az = accel_event.acceleration.z * 101 ;

  /* Get a new sensor event */
  float  mx = mag_event.magnetic.x;
  float  my = mag_event.magnetic.y;
  float  mz = mag_event.magnetic.z;

  myIMU.update(gx, gy, gz, ax, ay, az, mx, my, mz);

  orientation.pitch = (myIMU.getRoll() * 180 / PI) + 90;
  orientation.roll = (myIMU.getPitch() * 180 / PI) + 90;
  orientation.heading = 2* ((myIMU.getYaw() * 180 / PI) + 90) - minima;
  if (orientation.heading <0){
    minima += (orientation.heading);
  }
}

void rockerISR() {
  /* parameter NDELAY = 650000;
   parameter NBITS = 20;

   reg [NBITS-1:0] count;
   reg xnew, clean;

   always @(posedge clk)
     if (reset) begin xnew <= noisy; clean <= noisy; count <= 0; end
     else if (noisy != xnew) begin xnew <= noisy; count <= 0; end
     else if (count == NDELAY) clean <= xnew;
     else count <= count+1;*/

  bool current = digitalRead(ROCKER_PIN);
  if(millis()-timeBtnPrev >= 30){
    if(myState == RECORD){
      myState = START;
    }
    else if (myState == START){
      myState = RECORD;
      writeFramAddress = 0x0000;
    }
    timeBtnPrev = millis();
  }
}

void greenBtnISR() {
  if (myState == START) {
    myState = DELAY;
    delayStart = millis();
  }
}

void setup() {
  btnPrev = HIGH;
  i = 0;
  myState = START;
  Serial.begin(115200);           // set up Serial library at 9600 bps
  // Initialize I2C F-RAM
  FRAM_I2C_Init();
  //doFRAMStuff();
  readFramAddress = 0x0000; // last FRAM address read
  writeFramAddress = 0x0000; // last FRAM address written
  AFMS.begin(); // default frequency of 1.6KHz

  for(int x = 0; x < NUM_MOTORS; x++){
    motors[x]= AFMS.getMotor(x+1);
  }

  /* Initialise the sensors */
  initSensors();

  // turn on motors 0-255 for duty cycle PWM
  for (int x = 0; x < 4; x++) {
    motors[x]->run(RELEASE);
  }

  // sets the direction of the motors to none (i.e. don't spin)
  for (int x = 0; x < 4; x++) {
    motors[x]->setSpeed(/*(x + 1)*MAX_PWM / 4*/0);
  }
  pinMode(RED_LED_PIN, OUTPUT);           // set pin to output
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);
  pinMode(ROCKER_PIN, INPUT);             // set rocker pin to input
  pinMode(GREEN_BTN_PIN, INPUT);          // set green pushbutton pin to input
  attachInterrupt(digitalPinToInterrupt(ROCKER_PIN), rockerISR, RISING);
  attachInterrupt(digitalPinToInterrupt(GREEN_BTN_PIN), greenBtnISR, LOW);
}

void loop() {

  updateIMUAndDoMadgwick();
  switch (myState) {
    case START: // paint it blue
      digitalWrite(RED_LED_PIN, HIGH);
      digitalWrite(GREEN_LED_PIN, HIGH);
      digitalWrite(BLUE_LED_PIN, LOW);
      break;
    case DELAY: // paint it orange
      digitalWrite(RED_LED_PIN, LOW);
      digitalWrite(GREEN_LED_PIN, LOW);
      digitalWrite(BLUE_LED_PIN, HIGH);
      break;
    case RECORD: // paint it red
      digitalWrite(RED_LED_PIN, LOW);
      digitalWrite(GREEN_LED_PIN, HIGH);
      digitalWrite(BLUE_LED_PIN, HIGH);
      break;
    case PLAYBACK: // paint it green
      digitalWrite(RED_LED_PIN, HIGH);
      digitalWrite(GREEN_LED_PIN, LOW);
      digitalWrite(BLUE_LED_PIN, HIGH);
      break;
    default: // paint it white
      digitalWrite(RED_LED_PIN, LOW);
      digitalWrite(GREEN_LED_PIN, LOW);
      digitalWrite(BLUE_LED_PIN, LOW);
  }

  new_time = millis();
  dt = (new_time - last_time) * 0.001; //in seconds
  samplefreq = 1000 / (new_time - last_time);
  last_time = new_time;

  // print output to serial port
  printIMUOutput();

  if (myState == DELAY && (new_time - delayStart >= 2000)) {
    myState = PLAYBACK;
    readFramAddress = 0x0000;
  }

  else if (myState == PLAYBACK) {
    if (readFramAddress <= writeFramAddress) {
      framFloatToBytes readRoll, readPitch, readYaw;
      if (readFramAddress == 0x0000) {
        FRAM_I2C_Random_Read(FRAM_SLAVE_SRAM_ADDR, readFramAddress, reinterpret_cast<uint8_t *>(readRoll.b), sizeof(float));
        readFramAddress += 4;
      }
      else {
        FRAM_I2C_Current_Read(FRAM_SLAVE_SRAM_ADDR, reinterpret_cast<uint8_t *>(readRoll.b), sizeof(float));
        readFramAddress += 4;
      }
      FRAM_I2C_Current_Read(FRAM_SLAVE_SRAM_ADDR, reinterpret_cast<uint8_t *>(readPitch.b), sizeof(float));
      FRAM_I2C_Current_Read(FRAM_SLAVE_SRAM_ADDR, reinterpret_cast<uint8_t *>(readYaw.b), sizeof(float));
      readFramAddress += 8;

      rollPid.updateState(readRoll.f, orientation.roll);
      pitchPid.updateState(readPitch.f, orientation.pitch);
      yawPid.updateState(readYaw.f, orientation.heading);

      float rollPwm = rollPid.getOutput();
      float pitchPwm = pitchPid.getOutput();
      float yawPwm =  yawPid.getOutput();

      Serial.print(F("YawPWM: "));
      Serial.print(yawPwm);
      Serial.print(F(" PitchPWM: "));
      Serial.println(pitchPwm);

      if (pitchPwm < 0) {
        motors[2]->setSpeed(abs(pitchPwm));
        motors[2]->run(FORWARD);
        motors[0]->run(RELEASE);
      }
      else if (pitchPwm > 0) {
        motors[0]->setSpeed(abs(pitchPwm));
        motors[0]->run(FORWARD);
        motors[2]->run(RELEASE);
      }
      if (yawPwm < 0) {
        motors[3]->setSpeed(abs(yawPwm));
        motors[3]->run(FORWARD);
        motors[1]->run(RELEASE);
      }
      else if (yawPwm > 0) {
        motors[1]->setSpeed(abs(yawPwm));
        motors[1]->run(FORWARD);
        motors[3]->run(RELEASE);
      }
    }
    else {
      myState = START; // played back all the samples so we're back at our original state
      readFramAddress = 0x0000; // go back to the beginning of the read address.
      releaseMotors();
    }
  }

 else if (myState == START) {
//    for (int x = 0; x <= 360; x++) {
//      //getPWMForMotors((double)x, motor_drives);
//      for (int y = 0; y < 4; y++) {
//        motors[y]->setSpeed(x/5);
//        motors[y]->run(FORWARD);
//      }
//      //delay(10);
//    }
  }

  // Save measurements to FRAM
  else if (myState == RECORD) {
    sts =   FRAM_I2C_Write(FRAM_SLAVE_SRAM_ADDR, writeFramAddress, reinterpret_cast<uint8_t*>(&orientation.roll), sizeof(float));
    writeFramAddress += 4;
    sts =   FRAM_I2C_Write(FRAM_SLAVE_SRAM_ADDR, writeFramAddress, reinterpret_cast<uint8_t*>(&orientation.pitch), sizeof(float));
    writeFramAddress += 4;
    sts =   FRAM_I2C_Write(FRAM_SLAVE_SRAM_ADDR, writeFramAddress, reinterpret_cast<uint8_t*>(&orientation.heading), sizeof(float));
    writeFramAddress += 4;
//    Serial.print(" Fram address: ");
//    Serial.println(writeFramAddress);
  }
}
