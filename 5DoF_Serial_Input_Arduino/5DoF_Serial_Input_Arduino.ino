// 5DoF_Serial_Input

#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Debug mode flags
#define TEST1 0 // Joint angle input
#define TEST2 0 // Task stack overflow

// Servo parameters
#define SERVOCENTER 310.0f
#define NUM_SERVOS 6
#define PWM_FREQ 50

// PCA9685 Setup
Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(0x40);

// Array for joint-space angles
volatile float targetAngles[NUM_SERVOS] = {0.0, 45.0, -90.0, -90.0, 0.0, -20.0};

// PWM tick offsets for neutral joint positions
static float tickOffset[NUM_SERVOS] = {5.0, 0.0, 0.0, 2.0, -3.0, 0.0};

// Servo rotation direction
static float servoDirection[NUM_SERVOS] = {1.0, 1.0, 1.0, -1.0, 1.0, 1.0};

// Mutex for accessing angles
SemaphoreHandle_t xAnglesMutex;

// Covert angles to PWM tick value
void anglesToTicks(const float angles[], uint16_t ticks[], size_t length) {

  for (size_t i = 0; i < length; i++) {
    
    float rawTicks = SERVOCENTER + servoDirection[i] * (1.5f * angles[i]) + tickOffset[i];
    
    ticks[i] = (uint16_t)round(rawTicks);
  }
}

// Task 1: Read serial data
void readSerialTask(void *pvParameters) {
  (void) pvParameters;

  #if TEST2
  UBaseType_t uxHighWaterMark1 = uxTaskGetStackHighWaterMark(NULL);
  #endif

  String inputString = "";

  for (;;) {
    // Check for incoming data
    if (Serial.available() > 0) {
      // Read one line
      inputString = Serial.readStringUntil('\n');
      inputString.trim();

      #if TEST1
      Serial.print(F("Serial data received: "));
      Serial.println(inputString);
      #endif

      // Expect format like: [30, 45, 90, 0, 10, 45]
      // 1) Check if it starts with '[' and ends with ']'
      if (inputString.startsWith("[") && inputString.endsWith("]")) {
        // Extract the content between [ and ]
        String contents = inputString.substring(1, inputString.length() - 1);
        contents.trim();  // remove any extra whitespace

        // Parse tokens separated by commas
        float tempAngles[NUM_SERVOS];
        int index = 0;

        // Convert to a modifiable C-string
        char buf[128];
        contents.toCharArray(buf, 128);

        // Split by comma
        char* token = strtok(buf, ",");
        while (token != NULL && index < NUM_SERVOS) {
          // Trim spaces around token and convert to float
          while (*token == ' ') token++; // skip leading spaces
          tempAngles[index++] = atof(token);
          token = strtok(NULL, ",");
        }

        if (index == NUM_SERVOS) {
          // All 6 angles were parsed successfully
          if (xSemaphoreTake(xAnglesMutex, portMAX_DELAY) == pdTRUE) {
            for (int i = 0; i < NUM_SERVOS; i++) {
              targetAngles[i] = tempAngles[i];
            }
            xSemaphoreGive(xAnglesMutex);

            // Print confirmation
            #if TEST1
            Serial.print(F("New angles set: ["));
            for (int i = 0; i < NUM_SERVOS; i++) {
              Serial.print(targetAngles[i]);
              if (i < NUM_SERVOS - 1) Serial.print(F(", "));
            }
            Serial.println(F("]"));
            #endif

          }
        } else {
          #if TEST1
          Serial.println(F("Invalid array size. Please provide 6 angles inside [ ]."));
          #endif
        }
      } else if (inputString.length() > 0) {
        #if TEST1
        Serial.println(F("Invalid format. Use: [angle1, angle2, angle3, angle4, angle5, angle6]"));
        #endif
      }
    }

    // Check for stack overflow
    #if TEST2
    uxHighWaterMark1 = uxTaskGetStackHighWaterMark(NULL);
    Serial.print(F("Task 1 stack water mark: "));
    Serial.println(uxHighWaterMark1);
    #endif

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// Task 2: Update servo outputs on PCA9685
void servoUpdateTask(void *pvParameters) {
  (void) pvParameters;

  #if TEST2
  UBaseType_t uxHighWaterMark2 = uxTaskGetStackHighWaterMark(NULL);
  #endif

  float localAngles[NUM_SERVOS];
  uint16_t localTicks[NUM_SERVOS];

  for (;;) {
    // Copy angles from shared array
    if (xSemaphoreTake(xAnglesMutex, portMAX_DELAY) == pdTRUE) {
      for (int i = 0; i < NUM_SERVOS; i++) {
        localAngles[i] = targetAngles[i];
      }
      xSemaphoreGive(xAnglesMutex);
    }

    // Convert angles to ticks
    anglesToTicks(localAngles, localTicks, NUM_SERVOS);

    // Send new tick value to each servo
    for (int i = 0; i < NUM_SERVOS; i++) {
      pca9685.setPWM(i, 0, localTicks[i]);
    }

    // Check for stack overflow
    #if TEST2
    uxHighWaterMark2 = uxTaskGetStackHighWaterMark(NULL);
    Serial.print(F("Task 2 stack water mark: "));
    Serial.println(uxHighWaterMark2);
    #endif

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void setup() {
  // Serial init
  Serial.begin(115200);
  while (!Serial) { }

  // I2C init & PCA9685 setup
  Wire.begin();
  pca9685.begin();
  pca9685.setPWMFreq(PWM_FREQ);

  xAnglesMutex = xSemaphoreCreateMutex();
  if (xAnglesMutex == NULL) {
    Serial.println(F("Error creating mutex!"));
    while (1);
  }

  // Create tasks
  xTaskCreate(readSerialTask, "ReadSerial", 280, NULL, 1, NULL);
  xTaskCreate(servoUpdateTask, "ServoUpdate", 150, NULL, 1, NULL);

  // Start FreeRTOS
  vTaskStartScheduler();
}

void loop() {

}
