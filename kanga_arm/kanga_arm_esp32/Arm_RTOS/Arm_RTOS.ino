/*  INCLUSIONS  */
#include <ESP32-TWAI-CAN.hpp>
#include <ESP32Servo.h>
#include <Arduino.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


/*  DEFINITIONS  */
#define BAUDRATE 1000   // CAN bitrate setting used by library


// PIN Definitions
#define CAN_TX 5
#define CAN_RX 4

// L9110 motor inputs
#define WRIST_IN1 27
#define WRIST_IN2 26

#define EE_PWM 25

#define LIMIT_SWITCH_J1 36
#define LIMIT_SWITCH_J2 39
#define LIMIT_SWITCH_J3 34
#define LIMIT_SWITCH_J4 35

// LEDC channels for wrist motor
#define WRIST_CH1 1
#define WRIST_CH2 2
#define WRIST_PWM_FREQ 1000
#define WRIST_PWM_RES 8


// CAN IDs
#define CAN_ID_LIMIT_SWITCH 800
#define CAN_ID_WRIST_ENCODER 801
#define CAN_ID_WRIST_CONTROL 802
#define CAN_ID_EE_CONTROL 803


/*  RTOS HANDLERS  */

// Queue Creation
QueueHandle_t xWristControlFrame;
QueueHandle_t xEEControlFrame;
QueueHandle_t xTransmitFrame;

// Task Handler Creation
TaskHandle_t xReadHandle = NULL;
TaskHandle_t xWriteHandle = NULL;
TaskHandle_t xWristServoHandle = NULL;
TaskHandle_t xEEServoHandle = NULL;

Servo EEServo;


/*  FUNCTION DECLARATIONS  */
void vCanReadLoop(void *pvParameters);
void vCanWriteLoop(void *pvParameters);
void vWristServoControl(void *pvParameters);
void vEEServoControl(void *pvParameters);
void setWristMotor(uint8_t direction, uint8_t duty);


void setup() {
  // Setup serial for debugging
  Serial.begin(115200);

  // Wrist motor setup for L9110
  ledcAttachChannel(WRIST_IN1, WRIST_PWM_FREQ, WRIST_PWM_RES, WRIST_CH1);
  ledcAttachChannel(WRIST_IN2, WRIST_PWM_FREQ, WRIST_PWM_RES, WRIST_CH2);

  // Start with motor off
  ledcWrite(WRIST_IN1, 0);
  ledcWrite(WRIST_IN2, 0);

  // EE Servo
  EEServo.attach(EE_PWM, 1000, 2210);
  EEServo.write(180);


  // Limit Switch GPIOs
  // pinMode(LIMIT_SWITCH_J1, INPUT_PULLUP);
  // pinMode(LIMIT_SWITCH_J2, INPUT_PULLUP);
  // pinMode(LIMIT_SWITCH_J3, INPUT_PULLUP);
  // pinMode(LIMIT_SWITCH_J4, INPUT_PULLUP);

  // Start CAN
  if (ESP32Can.begin(ESP32Can.convertSpeed(BAUDRATE), CAN_TX, CAN_RX, 10, 10)) {
    Serial.println("CAN bus started!");
  } else {
    Serial.println("CAN bus failed!");
  }

  // Queue Creation
  xWristControlFrame = xQueueCreate(5, sizeof(CanFrame));
  xEEControlFrame = xQueueCreate(5, sizeof(CanFrame));
  xTransmitFrame = xQueueCreate(5, sizeof(CanFrame));

  // Task Creation
  xTaskCreatePinnedToCore(
    vCanReadLoop,
    "Can Read Loop",
    8000,
    NULL,
    5,
    &xReadHandle,
    0
  );

  xTaskCreatePinnedToCore(
    vCanWriteLoop,
    "Can Write Loop",
    2000,
    NULL,
    4,
    &xWriteHandle,
    0
  );

  xTaskCreatePinnedToCore(
    vWristServoControl,
    "J5/Wrist Control Loop",
    2000,
    NULL,
    2,
    &xWristServoHandle,
    1
  );

  xTaskCreatePinnedToCore(
    vEEServoControl,
    "End Effector Servo",
    2000,
    &EEServo,
    3,
    &xEEServoHandle,
    1
  );
}

void loop() {
  // Unused
}


/*  HELPER FUNCTIONS  */

// L9110 control
// direction = 0 -> IN1 = PWM, IN2 = 0
// direction = 1 -> IN1 = 0,   IN2 = PWM
// anything else -> stop
void setWristMotor(uint8_t direction, uint8_t duty) {
  if (direction == 0) {
    ledcWrite(WRIST_IN1, duty);
    ledcWrite(WRIST_IN2, 0);
  }
  else if (direction == 1) {
    ledcWrite(WRIST_IN1, 0);
    ledcWrite(WRIST_IN2, duty);
  }
  else {
    ledcWrite(WRIST_IN1, 0);
    ledcWrite(WRIST_IN2, 0);
  }
}


/*  CORE 0 TASKS BY PRIORITY  */

// 1. Writing over CAN
void vCanWriteLoop(void *pvParameters) {
  Serial.println("Enter Write Task");

  CanFrame transmittedFrame;

  for (;;) {
    // Wait until something is queued to transmit
    if (xQueueReceive(xTransmitFrame, &transmittedFrame, portMAX_DELAY) == pdTRUE) {
      if (ESP32Can.writeFrame(transmittedFrame, 10) != 1) {
        Serial.println("Write Failed");
      }
    }
  }
}

// 2. Reading from CAN
void vCanReadLoop(void *pvParameters) {
  Serial.println("Enter Read Task");

  CanFrame receivedFrame;

  for (;;) {
    ESP32Can.readFrame(receivedFrame, portMAX_DELAY);

    switch (receivedFrame.identifier) {
      case CAN_ID_WRIST_CONTROL:
        xQueueSend(xWristControlFrame, &receivedFrame, 0);
        break;

      case CAN_ID_EE_CONTROL:
        xQueueSend(xEEControlFrame, &receivedFrame, 0);
        break;

      default:
        break;
    }
  }
}


/*  CORE 1 TASKS BY PRIORITY  */

// 1. Limit Switch Read Loop
/*
void vReadLimitSwitch(void * pvParameters) {
  CanFrame LimitSwitchFrame;

  LimitSwitchFrame.identifier = CAN_ID_LIMIT_SWITCH;
  LimitSwitchFrame.extd = 0;
  LimitSwitchFrame.data_length_code = 1;
  LimitSwitchFrame.data[0] = 0;

  for (;;) {
    LimitSwitchFrame.data[0] = 0;

    if (digitalRead(LIMIT_SWITCH_J1) == HIGH) {
      LimitSwitchFrame.data[0] |= 0b00001000;
    }
    if (digitalRead(LIMIT_SWITCH_J2) == HIGH) {
      LimitSwitchFrame.data[0] |= 0b00000100;
    }
    if (digitalRead(LIMIT_SWITCH_J3) == HIGH) {
      LimitSwitchFrame.data[0] |= 0b00000010;
    }
    if (digitalRead(LIMIT_SWITCH_J4) == HIGH) {
      LimitSwitchFrame.data[0] |= 0b00000001;
    }
  }
}
*/


// 2. End Effector Servo Control
void vEEServoControl(void *pvParameters) {
  CanFrame EEControlFrame;
  Servo *EEServoPtr = (Servo *)pvParameters;

  for (;;) {
    xQueueReceive(xEEControlFrame, &EEControlFrame, portMAX_DELAY);

    int servoAngle = constrain((int)EEControlFrame.data[0], 0, 180);
    EEServoPtr->write(servoAngle);

    Serial.println("");
    Serial.print("Servo Goal Angle: ");
    Serial.println(servoAngle);
  }
}


// 3. J5/Wrist motor control through L9110
void vWristServoControl(void *pvParameters) {
  CanFrame WristControlFrame;

  for (;;) {
    xQueueReceive(xWristControlFrame, &WristControlFrame, portMAX_DELAY);

    uint8_t direction = WristControlFrame.data[0];
    uint8_t duty = WristControlFrame.data[1];

    setWristMotor(direction, duty);

    Serial.println("");
    if (direction == 0) {
      Serial.print("Spinning Direction 1 at: ");
      Serial.println(duty);
    }
    else if (direction == 1) {
      Serial.print("Spinning Direction 2 at: ");
      Serial.println(duty);
    }
    else {
      Serial.println("Invalid direction, wrist motor stopped");
    }
  }
}

// 4. J5 Encoder Read
// void vWristEncoderRead(void *pvParameters) {}