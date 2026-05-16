#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(0x40);

#define SERVOMIN  70
#define SERVOMAX  550

#define SERVOCENTER 310

void setup() {

  pca9685.begin();
  
  pca9685.setPWMFreq(50);

  /*
  pca9685.setPWM(0, 0, 220); //315
  pca9685.setPWM(1, 0, 360); //310
  pca9685.setPWM(2, 0, 215); //310
  pca9685.setPWM(3, 0, 480); //312
  pca9685.setPWM(4, 0, 307); //307
  pca9685.setPWM(5, 0, 280); //280~365
  */
  
  //Set joint angles here
  int16_t jointAngle0 = -95;
  int16_t jointAngle1 = 50;
  int16_t jointAngle2 = -95;
  int16_t jointAngle3 = 168;
  int16_t jointAngle4 = 0;
  int16_t jointAngle5 = 0; //0 is closed, 85 is open

  int16_t PWM_Command[6] = {315+jointAngle0, 310+jointAngle1, 310+jointAngle2, 312+jointAngle3, 307+jointAngle4, 280+jointAngle5};

  pca9685.setPWM(0, 0, PWM_Command[0]); //315
  pca9685.setPWM(1, 0, PWM_Command[1]); //310
  pca9685.setPWM(2, 0, PWM_Command[2]); //310
  pca9685.setPWM(3, 0, PWM_Command[3]); //312
  pca9685.setPWM(4, 0, PWM_Command[4]); //307
  pca9685.setPWM(5, 0, PWM_Command[5]); //280~365
  

  delay(10);

}

void loop() {

}