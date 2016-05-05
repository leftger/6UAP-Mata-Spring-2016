#include <Wire.h> // I2C library
#include <math.h> // sines,cosines and all the math Euler came up with
#include "CYI2CFRAM.h" // FRAM library
#include <Adafruit_Sensor.h>
//#include <Adafruit_BMP280.h> // Pressure sensor library
#include <Adafruit_LSM303_U.h>
#include <Adafruit_9DOF.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_MotorShield.h>
#include <MadgwickAHRS.h>

// Constants for motor drive
#define MAX_PWM 150
#define MAX_ROLL 90
#define outMax 255
#define outMin 255

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

Adafruit_DCMotor *motor_uno = AFMS.getMotor(1);
Adafruit_DCMotor *motor_dos = AFMS.getMotor(2);
Adafruit_DCMotor *motor_tres = AFMS.getMotor(3);
Adafruit_DCMotor *motor_cuatro = AFMS.getMotor(4);

/* Assign a unique ID to the sensors */
Adafruit_9DOF                 dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);
Adafruit_DCMotor *motors[4]; // list of pointers to the motor instances so that they can be
// referenced more easily
double motor_drives[4]; // array of floats containing the PWM values to drive motors along
// with directionality

//Adafruit_BMP280 bme; // I2C Pressure Sensor

uint8_t *toFram;
unsigned int i;
volatile uint32_t framAddress = 0x0000;

float samplefreq;
unsigned long last_time;
unsigned long new_time;
float dt;

sensors_event_t accel_event;
sensors_event_t mag_event;
sensors_event_t gyro_event;
sensors_vec_t   orientation;

Madgwick myIMU(betaDef, sampleFreq);

class PIDControl
{
    float ki, kp, kd;
    float desired;
    float current_meas, prev_meas;
    float error, error_deriv, error_integral, output;

  public:
    PIDControl(): PIDControl(0.1f, 2.0f, 0.5f) {}
    PIDControl(float ki, float kp, float kd) {
      this->ki = ki;
      this->kp = kp;
      this->kd = kd;
    }

    void updateState(float newDesired, float current) {
      desired = newDesired;
      prev_meas = current_meas;
      current_meas = current;
      error = desired - current_meas; // current error
      error_integral += error * dt * ki;
      if (error_integral > outMax) error_integral = outMax;
      else if (error_integral < outMin) error_integral = outMin;
      error_deriv = (current_meas - prev_meas) / dt;
      output = kp * error + error_integral - kd * error_deriv;
    }

    float getOutput() {
      return output;
    }
};

PIDControl rollPid;
PIDControl PitchPid;
PIDControl headingPid;

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
  pwms[0] = MAX_PWM * cos(degreesToRadians(degree));
  pwms[1] = MAX_PWM * sin(degreesToRadians(degree));
  pwms[2] = -(pwms[0]);
  pwms[3] = -(pwms[1]);
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

  // Initialize I2C F-RAM
  FRAM_I2C_Init();

  // Start-up Delay
  delay(100);

  result = PASS;
  Serial.print("\n\nRead Data\n");

  for (i = 0; i < NUM_SAMPLES_READ; i++) {
    if (!i) {
      no_of_bytes_read = FRAM_I2C_Random_Read(FRAM_SLAVE_SRAM_ADDR, framAddress, reinterpret_cast<uint8_t *>(ff2b.b), sizeof(float));
    }
    else no_of_bytes_read = FRAM_I2C_Current_Read(FRAM_SLAVE_SRAM_ADDR, reinterpret_cast<uint8_t *>(ff2b.b), sizeof(float));
    if (no_of_bytes_read == sizeof(float))
    {
      result = PASS;
      Serial.print("Roll: ");
      Serial.print(ff2b.f);
    }
    no_of_bytes_read = FRAM_I2C_Current_Read(FRAM_SLAVE_SRAM_ADDR, reinterpret_cast<uint8_t *>(ff2b.b), sizeof(float));
    if (no_of_bytes_read == sizeof(float))
    {
      result = PASS;
      Serial.print(" Pitch: ");
      Serial.print(ff2b.f);
    }
    no_of_bytes_read = FRAM_I2C_Current_Read(FRAM_SLAVE_SRAM_ADDR, reinterpret_cast<uint8_t *>(ff2b.b), sizeof(float));
    if (no_of_bytes_read == sizeof(float))
    {
      result = PASS;
      Serial.print(" Yaw: ");
      Serial.println(ff2b.f);
    }
  }

  // reset FRAM address to beginning
  framAddress = 0x0000;


  Serial.print("\n\n-----------------------------");
  Serial.println("\nF-RAM Read N samples End");

}

void setup() {
  i = 0;
  Serial.begin(115200);           // set up Serial library at 9600 bps
  doFRAMStuff();
  //  Serial.println("Adafruit Motorshield v2 - DC Motor test!");
  AFMS.begin(); // default frequency of 1.6KHz
  motors[0] = motor_uno;
  motors[1] = motor_dos;
  motors[2] = motor_tres;
  motors[3] = motor_cuatro;

  /* Initialise the sensors */
  initSensors();

  // turn on motors 0-255 for duty cycle PWM
  for (int x = 0; x < 4; x++) {
    motors[x]->run(RELEASE);
  }

  // sets the direction of the motors to none (i.e. don't spin)
  for (int x = 0; x < 4; x++) {
    motors[x]->setSpeed((x + 1)*MAX_PWM / 4);
  }
}

void loop() {

  /* Read the accelerometer and magnetometer */
  accel.getEvent(&accel_event);
  mag.getEvent(&mag_event);
  gyro.getEvent(&gyro_event);

  float  gx = gyro_event.gyro.x;
  float  gy = gyro_event.gyro.y;
  float  gz = gyro_event.gyro.z;

  /* Get a new sensor event */
  float  ax = (accel_event.acceleration.x) * 101 ;
  float  ay = accel_event.acceleration.y * 101;
  float  az = accel_event.acceleration.z * 101 ;

  /* Get a new sensor event */
  float  mx = mag_event.magnetic.x;
  float  my = mag_event.magnetic.y;
  float  mz = mag_event.magnetic.z;

  myIMU.update(gx, gy, gz, ax, ay, az, mx, my, mz);

  orientation.pitch = myIMU.getRoll() * 180 / PI;
  orientation.roll = myIMU.getPitch() * 180 / PI;
  orientation.heading = myIMU.getYaw() * 180 / PI;

  /* 'orientation' should have valid .roll and .pitch fields */
  new_time = millis();
  dt = (new_time - last_time) * 0.001; //in seconds
  samplefreq = 1000 / (new_time - last_time);
  last_time = new_time;
  Serial.print(F("Orientation: "));
  Serial.print(F("Roll: "));
  Serial.print(orientation.roll);
  Serial.print(F(" Pitch: "));
  Serial.print(orientation.pitch);
  Serial.print(F(" Yaw: "));
  Serial.print(orientation.heading);
  Serial.print(F(" SampleFreq: "));
  Serial.print(samplefreq);
  Serial.println(F(""));
  //  if (orientation.pitch > 0) {
  //    motors[0]->setSpeed(MAX_PWM / MAX_ROLL * abs(orientation.pitch));
  //    motors[0]->run(FORWARD);
  //    motors[2]->run(RELEASE);
  //  }
  //  else if (orientation.pitch < 0) {
  //    motors[2]->setSpeed(MAX_PWM / MAX_ROLL * abs(orientation.pitch));
  //    motors[2]->run(FORWARD);
  //    motors[0]->run(RELEASE);
  //  }

  //  for (int x = 0; x <= 360; x++) {
  //    getPWMForMotors((double)x, motor_drives);
  //    for (int y = 0; y < 4; y++) {
  //      motors[y]->setSpeed(abs(motor_drives[y]));
  //      if (motor_drives[y] > 0) {
  //        motors[y]->run(FORWARD);
  //      }
  //      else {
  //        motors[y]->run(BACKWARD);
  //      }
  //    }
  //    //delay(10);
  //  }
  //  Serial.print("Temperature = ");
  //  Serial.print(bme.readTemperature());
  //  Serial.println(" *C");
  //
  //  Serial.print("Pressure = ");
  //  Serial.print(bme.readPressure());
  //  Serial.println(" Pa");
  //
  //  Serial.print("Approx altitude = ");
  //  Serial.print(bme.readAltitude(1013.25)); // this should be adjusted to your local forcase
  //  Serial.println(" m");

  uint8_t sts;
  if (i < NUM_SAMPLES_READ) {
    sts =   FRAM_I2C_Write(FRAM_SLAVE_SRAM_ADDR, framAddress, reinterpret_cast<uint8_t*>(&orientation.roll), sizeof(float));
    framAddress += 4;
    sts =   FRAM_I2C_Write(FRAM_SLAVE_SRAM_ADDR, framAddress, reinterpret_cast<uint8_t*>(&orientation.pitch), sizeof(float));
    framAddress += 4;
    sts =   FRAM_I2C_Write(FRAM_SLAVE_SRAM_ADDR, framAddress, reinterpret_cast<uint8_t*>(&orientation.heading), sizeof(float));
    framAddress += 4;
    i++;
  }

  //  if (sts != FRAM_I2C_MSTR_NO_ERROR)
  //  {
  //    // I2C Error
  //    Serial.print("\nCOM ERROR : ");
  //    Serial.print(sts, HEX);
  //    COM_ERR();
  //  }

}
