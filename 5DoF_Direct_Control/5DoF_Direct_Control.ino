#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(0x40);

#define SERVOMIN  70
#define SERVOMAX  550

#define SERVOCENTER 310

void setup() {

  pca9685.begin();
  
  pca9685.setPWMFreq(50);

  pca9685.setPWM(0, 0, 220); //315
  pca9685.setPWM(1, 0, 360); //310
  pca9685.setPWM(2, 0, 215); //310
  pca9685.setPWM(3, 0, 480); //312
  pca9685.setPWM(4, 0, 307); //307
  pca9685.setPWM(5, 0, 280); //280~365

  delay(10);

}

void loop() {

}