#include <Wire.h> // I2C library
#include <AFMotor.h> // Motor library
#include <math.h> // sines,cosines and all the math Euler came up with
#include "CYI2CFRAM.h" // FRAM library
#include <Adafruit_LSM303_U.h>
#include <Adafruit_9DOF.h>
#include <Adafruit_L3GD20_U.h>

#define MAX_PWM 255

// Data buffer size
#define BUFFER_SIZE            (16u)

// Example F-RAM Address
#define EXAMPLE_ADDR_1         (0x2000)

// Example F-RAM Address
#define EXAMPLE_ADDR_2         (0x3456)

// Example F-RAM data
#define EXAMPLE_DATA_BYTE      (0xa5)

// Example status register data
#define EXAMPLE_STS_REG_VALUE  (0x08)

// F-RAM Slave Address
#define FRAM_SLAVE_SRAM_ADDR    (0x50)

// Macro for F-RAM Read/Write PASS/FAIL
#define FAIL                    (0u)
#define PASS                    (1u)

// I2C Communication Error
#define COM_ERR()  for(;;){}

AF_DCMotor motor_uno(1);
AF_DCMotor motor_dos(2);
AF_DCMotor motor_tres(3);
AF_DCMotor motor_cuatro(4);
AF_DCMotor *motors[4]; // list of pointers to the motor instances so that they can be
// referenced more easily
double motor_drives[4]; // array of floats containing the PWM values to drive motors along
// with directionality

/* Assign a unique ID to the sensors */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

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

void doFRAMStuff() {
  uint8_t data_bytes[BUFFER_SIZE];
  uint8_t fram_rd_data[BUFFER_SIZE];
  uint8_t fram_wr_byte;
  uint8_t fram_rd_byte, fram_rd_current_byte;
  uint8_t i;
  uint8_t result;
  uint8_t sts;
  uint32_t no_of_bytes_read;

  Serial.print("\nF-RAM I2C Example Project Start");
  Serial.print("\n-------------------------------\n");

  // Initialize I2C F-RAM
  FRAM_I2C_Init();

  // Start-up Delay
  delay(100);

  // Initialization of variables
  for (i = 0; i < BUFFER_SIZE; i++)
  {
    data_bytes[i] = i + 1;
    fram_rd_data[i] = 0;
  }

  // Write F-RAM data
  Serial.print("\n1. Write Data\n");
  // F-RAM Write : Write 1 byte of data to F-RAM location EXAMPLE_ADDR_1
  fram_wr_byte = EXAMPLE_DATA_BYTE;
  sts =   FRAM_I2C_Write(FRAM_SLAVE_SRAM_ADDR, EXAMPLE_ADDR_1, &fram_wr_byte, 1);
  if (sts != FRAM_I2C_MSTR_NO_ERROR)
  {
    // I2C Error
    Serial.print("\nCOM ERROR : ");
    Serial.print(sts, HEX);
    COM_ERR();
  }

  Serial.print(fram_wr_byte, HEX);

  // Read a data byte
  Serial.print("\n\n2. Read Data");
  // F-RAM Read : Read 1 byte of data from F-RAM location EXAMPLE_ADDR_1
  no_of_bytes_read = FRAM_I2C_Random_Read(FRAM_SLAVE_SRAM_ADDR, EXAMPLE_ADDR_1, &fram_rd_byte, 1);
  if ( no_of_bytes_read > 0)
  {
    Serial.print("\nNo. of bytes Read : ");
    Serial.print(no_of_bytes_read, HEX);

    // Read F-RAM data
    Serial.print("\nData : ");
    Serial.print(fram_rd_byte, HEX);
  }
  else
  {
    // Zero bytes read
    Serial.print("\nNo. of bytes Read : ");
    Serial.print(no_of_bytes_read, HEX);
  }

  // Compare and display the result on Serial Monitor
  if (fram_rd_byte == fram_wr_byte)
  {
    Serial.print("\nWrite-Read : PASS");
  }
  else
  {
    Serial.print("\nWrite-Read : FAIL");
  }

  // Write 16 bytes of F-RAM data
  Serial.print("\n\n3. Write Data Array\n");

  // F-RAM Write : Write 16 bytes from array data_bytes to F-RAM location EXAMPLE_ADDR_2
  sts = FRAM_I2C_Write(FRAM_SLAVE_SRAM_ADDR, EXAMPLE_ADDR_2, data_bytes, BUFFER_SIZE);
  if (sts != FRAM_I2C_MSTR_NO_ERROR)
  {
    // I2C Error
    Serial.print("\nCOM ERROR : ");
    Serial.print(sts, HEX);
    COM_ERR();
  }

  for (i = 0; i < BUFFER_SIZE; i++)
  {
    Serial.print(data_bytes[i], HEX);
    Serial.print("\t");
  }

  // Read and Compare 16 bytes of F-RAM data
  result = PASS;
  Serial.print("\n\n4. Read Data Array\n");

  // F-RAM Random Read : Read 15 bytes of data from SRAM starting from address EXAMPLE_ADDR_2. //
  //                     Data will be stored into fram_rd_data array.                          //

  no_of_bytes_read = FRAM_I2C_Random_Read(FRAM_SLAVE_SRAM_ADDR, EXAMPLE_ADDR_2, fram_rd_data, BUFFER_SIZE - 1);
  if (no_of_bytes_read > 0)
  {
    result = PASS;
    for (i = 0; i < no_of_bytes_read; i++)
    {
      if (fram_rd_data[i] != data_bytes[i])
        result = FAIL;
      Serial.print(fram_rd_data[i], HEX);
      Serial.print("\t");
    }

  }
  else
  {
    // Zero bytes read
    Serial.print("\nNo. of bytes Read : ");
    Serial.print(no_of_bytes_read, HEX);
  }
  // F-RAM Read : Read the 16th byte through current location read
  no_of_bytes_read = FRAM_I2C_Current_Read(FRAM_SLAVE_SRAM_ADDR, &fram_rd_current_byte, 1);
  if (no_of_bytes_read > 0)
  {
    // Read F-RAM data
    Serial.print(fram_rd_current_byte, HEX);

    if (fram_rd_current_byte != data_bytes[BUFFER_SIZE - 1])
      result = FAIL;
  }

  // Indicate the busrt write / read result
  if (result == PASS)
  {
    Serial.print("\nWrite-Read : PASS");
  }
  else
  {
    Serial.print("\nWrite-Read : FAIL");
  }

  Serial.print("\n\n-----------------------------");
  Serial.print("\nF-RAM I2C Example Project End");

}

void do9DOFStuff(){
  Serial.println(F("Adafruit 9DOF Tester")); Serial.println("");
  
  /* Initialise the sensors */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  if(!gyro.begin())
  {
    /* There was a problem detecting the L3GD20 ... check your connections */
    Serial.print("Ooops, no L3GD20 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  /* Display some basic information on this sensor */
  displaySensorDetails();
}

void displaySensorDetails(void)
{
  sensor_t sensor;
  
  accel.getSensor(&sensor);
  Serial.println(F("----------- ACCELEROMETER ----------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" m/s^2"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  gyro.getSensor(&sensor);
  Serial.println(F("------------- GYROSCOPE -----------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" rad/s"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
  
  mag.getSensor(&sensor);
  Serial.println(F("----------- MAGNETOMETER -----------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" uT"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" uT"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" uT"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
}

void setup() {
  Serial.begin(115200);           // set up Serial library at 9600 bps
  doFRAMStuff();
  Serial.println("Motor test!");
  motors[0] = &motor_uno;
  motors[1] = &motor_dos;
  motors[2] = &motor_tres;
  motors[3] = &motor_cuatro;

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
  uint8_t i;
  /* Get a new sensor event */
  sensors_event_t event;

  for (int x = 0; x <= 360; x++) {
    getPWMForMotors((double)x, motor_drives);
    for (int y = 0; y < 4; y++) {
      motors[y]->setSpeed(abs(motor_drives[y]));
      if (motor_drives[y] > 0) {
        motors[y]->run(FORWARD);
      }
      else {
        motors[y]->run(BACKWARD);
      }
    }
    //delay(10);
  }
  accel.getEvent(&event);
  Serial.print(F("ACCEL "));
  Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");

  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  mag.getEvent(&event);
  Serial.print(F("MAG   "));
  Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");

  /* Display the results (gyrocope values in rad/s) */
  gyro.getEvent(&event);
  Serial.print(F("GYRO  "));
  Serial.print("X: "); Serial.print(event.gyro.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.gyro.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.gyro.z); Serial.print("  ");Serial.println("rad/s ");  

  Serial.println(F(""));
}
