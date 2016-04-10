// Adafruit Motor shield library
// copyright Adafruit Industries LLC, 2009
// this code is public domain, enjoy!

#include <AFMotor.h>
#include <math.h>

#define MAX_PWM 255

AF_DCMotor motor_uno(1);
AF_DCMotor motor_dos(2);
AF_DCMotor motor_tres(3);
AF_DCMotor motor_cuatro(4);
AF_DCMotor *motors[4]; // list of pointers to the motor instances so that they can be
                       // referenced more easily
double motor_drives[4]; // array of floats containing the PWM values to drive motors along
                       // with directionality

// calculate the pwm drive and direction for each motor given a theta (theta should be from 0 to 360);
void getPWMForMotors(double degree, double* pwms){
  pwms[0] = MAX_PWM * cos(degreesToRadians(degree));
  pwms[1] = MAX_PWM * sin(degreesToRadians(degree));
  pwms[2] = -(pwms[0]);
  pwms[3] = -(pwms[1]);
}

double degreesToRadians(double degree){
  return (degree * 71) / 4068;
}

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Motor test!");
  motors[0] = &motor_uno;
  motors[1] = &motor_dos;
  motors[2] = &motor_tres;
  motors[3] = &motor_cuatro;

  // turn on motors 0-255 for duty cycle PWM
  for(int x= 0; x < 4; x++){
    motors[x]->run(RELEASE);
  }

 // sets the direction of the motors to none (i.e. don't spin)
  for(int x= 0; x < 4; x++){
    motors[x]->setSpeed((x+1)*MAX_PWM/4);
  }
}

void loop() {
  uint8_t i;
  
//  Serial.print("tick");
//
//  for(int x= 0; x < 4;x++){
//    motors[x]->run(FORWARD);
//  }
//  delay(1000);
//  
//  Serial.print("tock");
//
//  for(int x=0; x<4;x++){
//    motors[x]->run(BACKWARD);
//  }
//
//  Serial.print("tech");
//  motor_cuatro.run(RELEASE);
//  delay(1000);

  for(int x = 0; x <= 360; x++){
    getPWMForMotors((double)x,motor_drives);
    for(int y = 0; y < 4; y++){
      motors[y]->setSpeed(abs(motor_drives[y]));
      if(motor_drives[y]>0){
        motors[y]->run(FORWARD);
      }
      else{
        motors[y]->run(BACKWARD);
      }
    }
    delay(10);
  }
}
