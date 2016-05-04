#include <Wire.h> // I2C library
#include <math.h> // sines,cosines and all the math Euler came up with
#include "CYI2CFRAM.h" // FRAM library
#include <Adafruit_Sensor.h> // Header file needed for BMP280
//#include <Adafruit_BMP280.h> // Pressure sensor library
#include <Adafruit_LSM303_U.h>
#include <Adafruit_9DOF.h>
#include <Adafruit_L3GD20_U.h>
// new include for MotorShield v2.0
#include <Adafruit_MotorShield.h>

//#define MAX_PWM 255
#define MAX_PWM 150
#define MAX_ROLL 90
#define NUM_SAMPLES_READ 100 // try to read the first 100 gyro measurements
#define FRAM_SLAVE_SRAM_ADDR    (0x50)  // F-RAM Slave Address

#define betaDef      3.0f      // 2 * proportional gain
#define sampleFreq  512.0f      // sample frequency in Hz
#define twoKpDef  (2.0f * 0.5f) // 2 * proportional gain
#define twoKiDef  (2.0f * 0.0f) // 2 * integral gain

// Macro for F-RAM Read/Write PASS/FAIL
#define FAIL                    (0u)
#define PASS                    (1u)

// I2C Communication Error
#define COM_ERR()  for(;;){}

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
volatile uint32_t framAddress = 0x0000;

volatile float beta = betaDef;                        // 2 * proportional gain (Kp)
volatile float twoKp = twoKpDef;                      // 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;                      // 2 * integral gain (Ki)
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f; // integral error terms scaled by Ki
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;   // quaternion of sensor frame relative to auxiliary frame
float psi, theta, phi;
float samplefreq;
unsigned long tiempo = 0.0f;

sensors_event_t accel_event;
sensors_event_t mag_event;
sensors_event_t gyro_event;
sensors_vec_t   orientation;

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
  float fromGyro;

  //  Serial.print("\nReading first N samples from FRAM");
  //  Serial.print("\n-------------------------------\n");

  // Initialize I2C F-RAM
  FRAM_I2C_Init();

  // Start-up Delay
  delay(100);

  // Read and Compare 16 bytes of F-RAM data
  result = PASS;
  //  Serial.print("\n\nRead Data\n");

  // F-RAM Random Read : Read 15 bytes of data from SRAM starting from address EXAMPLE_ADDR_2. //
  //                     Data will be stored into fram_rd_data array.                          //

  for (i = 0; i < NUM_SAMPLES_READ; i++) {
    no_of_bytes_read = FRAM_I2C_Random_Read(FRAM_SLAVE_SRAM_ADDR, framAddress, reinterpret_cast<uint8_t *>(ff2b.b), sizeof(float));
    if (no_of_bytes_read == sizeof(float))
    {
      result = PASS;
      //      Serial.print("roll:");
      //      Serial.println(ff2b.f);
      framAddress += sizeof(float);
    }
    no_of_bytes_read = FRAM_I2C_Random_Read(FRAM_SLAVE_SRAM_ADDR, framAddress, reinterpret_cast<uint8_t *>(ff2b.b), sizeof(float));
    if (no_of_bytes_read == sizeof(float))
    {
      result = PASS;
      //      Serial.print("pitch:");
      //      Serial.println(ff2b.f);
      framAddress += sizeof(float);
    }
    no_of_bytes_read = FRAM_I2C_Random_Read(FRAM_SLAVE_SRAM_ADDR, framAddress, reinterpret_cast<uint8_t *>(ff2b.b), sizeof(float));
    if (no_of_bytes_read == sizeof(float))
    {
      result = PASS;
      //      Serial.print("heading:");
      //      Serial.println(ff2b.f);
      framAddress += sizeof(float);
    }
  }

  // reset FRAM address to beginning
  framAddress = 0x0000;



  // F-RAM Read : Read the 16th byte through current location read
  //  no_of_bytes_read = FRAM_I2C_Current_Read(FRAM_SLAVE_SRAM_ADDR, &fram_rd_current_byte, 1);
  //  if (no_of_bytes_read > 0)
  //  {
  //    // Read F-RAM data
  //    Serial.print(fram_rd_current_byte, HEX);
  //
  //    if (fram_rd_current_byte != data_bytes[BUFFER_SIZE - 1])
  //      result = FAIL;
  //  }



  //  Serial.print("\n\n-----------------------------");
  //  Serial.println("\nF-RAM Read N samples End");

}

void setup() {
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

  MahonyAHRSupdate();
  psi = atan2((2 * q1 * q2) - (2 * q0 * q3), (2 * pow(q0, 2)) + (2 * pow(q1, 2)) - 1);
  theta = asin((2 * q1 * q3) + (2 * q0 * q2));
  phi = atan2((2 * q2 * q3) - (2 * q0 * q1), (2 * pow(q0, 2)) + (2 * pow(q3, 2)) - 1);

  orientation.pitch = theta * 180 / PI;
  orientation.roll = phi * 180 / PI;
  orientation.heading = psi * -180 / PI;

  /* 'orientation' should have valid .roll and .pitch fields */
  unsigned long nowTime = millis();
  Serial.print(F("Orientation: "));
  Serial.print(orientation.roll);
  Serial.print(F(" "));
  Serial.print(orientation.pitch);
  Serial.print(F(" "));
  Serial.print(orientation.heading);
  Serial.print(F(" "));
  Serial.print(samplefreq);
  Serial.println(F(""));
  
  samplefreq = 1000/(nowTime- tiempo);
  tiempo = nowTime;
  if (orientation.pitch > 0) {
    motors[0]->setSpeed(MAX_PWM / MAX_ROLL * abs(orientation.pitch));
    motors[0]->run(FORWARD);
    motors[2]->run(RELEASE);
  }
  else if (orientation.pitch < 0) {
    motors[2]->setSpeed(MAX_PWM / MAX_ROLL * abs(orientation.pitch));
    motors[2]->run(FORWARD);
    motors[0]->run(RELEASE);
  }

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
  toFram = reinterpret_cast<uint8_t*>(&orientation.roll);
  sts =   FRAM_I2C_Write(FRAM_SLAVE_SRAM_ADDR, framAddress, toFram, 4);
  framAddress += 4;
  toFram = reinterpret_cast<uint8_t*>(&orientation.pitch);
  sts =   FRAM_I2C_Write(FRAM_SLAVE_SRAM_ADDR, framAddress, toFram, 4);
  framAddress += 4;
  toFram = reinterpret_cast<uint8_t*>(&orientation.heading);
  sts =   FRAM_I2C_Write(FRAM_SLAVE_SRAM_ADDR, framAddress, toFram, 4);
  framAddress += 4;

  if (sts != FRAM_I2C_MSTR_NO_ERROR)
  {
    // I2C Error
    Serial.print("\nCOM ERROR : ");
    Serial.print(sts, HEX);
    COM_ERR();
  }

  delay(100);

  Serial.println();
}

void MadgwickAHRSupdate() {
  /* Get a new sensor event */
  float  gx = gyro_event.gyro.x;
  float  gy = gyro_event.gyro.y;
  float  gz = gyro_event.gyro.z;

  /* Get a new sensor event */
  float  ax = (accel_event.acceleration.x) * 101;
  float  ay = accel_event.acceleration.y * 101;
  float  az = accel_event.acceleration.z * 101;

  /* Get a new sensor event */
  float  mx = mag_event.magnetic.x;
  float  my = mag_event.magnetic.y;
  float  mz = mag_event.magnetic.z;
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float hx, hy;
  float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;


  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Normalise magnetometer measurement
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0mx = 2.0f * q0 * mx;
    _2q0my = 2.0f * q0 * my;
    _2q0mz = 2.0f * q0 * mz;
    _2q1mx = 2.0f * q1 * mx;
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _2q0q2 = 2.0f * q0 * q2;
    _2q2q3 = 2.0f * q2 * q3;
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;

    // Reference direction of Earth's magnetic field
    hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
    hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * (1.0f / samplefreq);
  q1 += qDot2 * (1.0f / samplefreq);
  q2 += qDot3 * (1.0f / samplefreq);
  q3 += qDot4 * (1.0f / samplefreq);

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

void MahonyAHRSupdate() {
    /* Get a new sensor event */
    float  gx = gyro_event.gyro.x;
    float  gy = gyro_event.gyro.y;
    float  gz = gyro_event.gyro.z;

    /* Get a new sensor event */
    float  ax = (accel_event.acceleration.x) * 101;
    float  ay = accel_event.acceleration.y * 101;
    float  az = accel_event.acceleration.z * 101;

    /* Get a new sensor event */
    float  mx = mag_event.magnetic.x;
    float  my = mag_event.magnetic.y;
    float  mz = mag_event.magnetic.z;
  float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;  
  float hx, hy, bx, bz;
  float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;     

    // Normalise magnetometer measurement
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;   

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;   

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

    // Estimated direction of gravity and magnetic field
    halfvx = q1q3 - q0q2;
    halfvy = q0q1 + q2q3;
    halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);  
  
    // Error is sum of cross product between estimated direction and measured direction of field vectors
    halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
    halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
    halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

    // Compute and apply integral feedback if enabled
    if(twoKi > 0.0f) {
      integralFBx += twoKi * halfex * (1.0f / sampleFreq);  // integral error scaled by Ki
      integralFBy += twoKi * halfey * (1.0f / sampleFreq);
      integralFBz += twoKi * halfez * (1.0f / sampleFreq);
      gx += integralFBx;  // apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    }
    else {
      integralFBx = 0.0f; // prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }
  
  // Integrate rate of change of quaternion
  gx *= (0.5f * (1.0f / sampleFreq));   // pre-multiply common factors
  gy *= (0.5f * (1.0f / sampleFreq));
  gz *= (0.5f * (1.0f / sampleFreq));
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx); 
  
  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}


