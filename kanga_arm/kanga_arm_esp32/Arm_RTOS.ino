/*  INCLUSIONS  */
#include <ESP32-TWAI-CAN.hpp>
#include <ESP32Servo.h>
#include <Arduino.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


/*  DEFINITIONS  */
#define BAUDRATE 500 // 500000kb/s


// PIN Definitions
#define CAN_TX 5
#define CAN_RX 4

#define WRIST_SPEED 27
#define WRIST_DIRECTION 26

#define EE_PWM 25

#define LIMIT_SWITCH_J1 36
#define LIMIT_SWITCH_J2 39
#define LIMIT_SWITCH_J3 34
#define LIMIT_SWITCH_J4 35


// CAN IDs
#define CAN_ID_LIMIT_SWITCH 800
#define CAN_ID_WRIST_ENCODER 801
#define CAN_ID_WRIST_CONTROL 802
#define CAN_ID_EE_CONTROL 803


/*  RTOS HANDLERS  */

//Queue Creation
QueueHandle_t xWristControlFrame;
QueueHandle_t xEEControlFrame;
QueueHandle_t xTransmitFrame;

//Task Handler Creation
TaskHandle_t xReadHandle = NULL;
TaskHandle_t xWriteHandle = NULL;
TaskHandle_t xWristServoHandle = NULL;
TaskHandle_t xEEServoHandle = NULL;

Servo EEServo;

void setup() {
  // Setup serial for debbuging.
  Serial.begin(115200);

  //Pin Setup 
  // J5/Wrist GPIO and PWM
  pinMode(WRIST_DIRECTION, OUTPUT);
  ledcAttachChannel(WRIST_SPEED, 1000, 8, 1); // (Pin,Frequency,Resolution)

  // EE PWM
  EEServo.attach(EE_PWM);

  // Limit Switch GPIOs
  //pinMode(LIMIT_SWITCH_J1, INPUT_PULLUP);
  //pinMode(LIMIT_SWITCH_J2, INPUT_PULLUP);
  //pinMode(LIMIT_SWITCH_J3, INPUT_PULLUP);
  //pinMode(LIMIT_SWITCH_J4, INPUT_PULLUP);

  // Whatever the encoder is


  // Sets Speed, Pins and queue sizes.
  if(ESP32Can.begin(ESP32Can.convertSpeed(BAUDRATE), CAN_TX, CAN_RX, 10, 10)) {
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
                          0);

  xTaskCreatePinnedToCore(
                          vCanWriteLoop,
                          "Can Write Loop",
                          2000,
                          NULL,
                          4,
                          &xWriteHandle,
                          0);

  xTaskCreatePinnedToCore(
                          vWristServoControl,
                          "J5/Wrist Control Loop",
                          2000,
                          NULL,
                          2,
                          &xWristServoHandle,
                          1);


  xTaskCreatePinnedToCore(
                          vEEServoControl,
                          "End Effector Servo",
                          2000,
                          &EEServo,
                          3,
                          &xEEServoHandle,
                          1);


} //Setup End

void loop() {
  //Question your existence
}

/*  CORE 0 TASKS BY PRIORITY  */

// 1. Writing over CAN
void vCanWriteLoop(void *pvParameters) {
  Serial.println("Enter Write Task");
  
  CanFrame transmittedFrame;
  
  /*  Testing
  transmittedFrame.identifier = 100;
  transmittedFrame.extd = 0;
  transmittedFrame.data_length_code = 2;
  transmittedFrame.data[0] = 20;
  transmittedFrame.data[1] = 30;
  */ 

  for ( ;; )
  { 
    // Wait on Queue to transmit data
    // xQueueReceive(xTransmitFrame, &transmittedFrame, portMAX_DELAY);
    
    if (ESP32Can.writeFrame(transmittedFrame, 10)!=1) {
      Serial.println("Write Failed");
    }

    vTaskDelay(pdMS_TO_TICKS(10000));

  }
}

// 2. Reading from CAN
void vCanReadLoop(void *pvParameters) {
  Serial.println("Enter Read Task");
  
  CanFrame receivedFrame;

  for( ;; )
  { 
    ESP32Can.readFrame(receivedFrame, portMAX_DELAY);  

    // Switch based on whether LED or Servo data was sent.
    switch(receivedFrame.identifier) {
      case CAN_ID_WRIST_CONTROL:
        xQueueSend(xWristControlFrame, &receivedFrame,0);
        break;
      case CAN_ID_EE_CONTROL:
        xQueueSend(xEEControlFrame, &receivedFrame,0);
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
    //Some timing control to prevent blocking?
    //Delay = ?
    LimitSwitchFrame.data[0] = 0;

    if (digitalRead(LIMIT_SWITCH_J1) == HIGH){
      LimitSwitchFrame.data[0] = LimitSwitchFrame.data[0] | 0b00001000;
    }
    if (digitalRead(LIMIT_SWITCH_J2) == HIGH){
      LimitSwitchFrame.data[0] = LimitSwitchFrame.data[0] | 0b00000100;
    }
    if (digitalRead(LIMIT_SWITCH_J3) == HIGH){
      LimitSwitchFrame.data[0] = LimitSwitchFrame.data[0] | 0b00000010;
    }
    if (digitalRead(LIMIT_SWITCH_J4) == HIGH){
      LimitSwitchFrame.data[0] = LimitSwitchFrame.data[0] | 0b00000001;
    }
  }
}
*/

// 2. End Effector Servo Control
void vEEServoControl (void * pvParameters) {
  CanFrame EEControlFrame;
  Servo *EEServo = (Servo *)pvParameters; //Gets the Servo object that was made in setup(), and passed in task creation;

  for (;;) {
    xQueueReceive(xEEControlFrame, &EEControlFrame, portMAX_DELAY);
/*
    Serial.println("EE Control Data");
    Serial.println(EEControlFrame.identifier);
    Serial.println(EEControlFrame.data[0]);
    Serial.println("");
*/ 
    EEServo->write(EEControlFrame.data[0]); //Assumes data is in angle

    Serial.println("");
    Serial.print("Servo Goal Angle: ");
    Serial.println(EEControlFrame.data[0]);

  }
}


// 3. J5/Wrist send PWMish signal
void vWristServoControl(void * pvParameters) {
  CanFrame WristControlFrame;

  for (;;) {
    // Wait for Queued Message
    xQueueReceive(xWristControlFrame, &WristControlFrame, portMAX_DELAY);

    
    if (WristControlFrame.data[0] == 0) {
      digitalWrite(WRIST_DIRECTION, LOW); //CCW or CW
      ledcWrite(WRIST_SPEED, WristControlFrame.data[1]);

      Serial.println("");
      Serial.print("Spinning Direction 1 at: ");
      Serial.println(WristControlFrame.data[1]);
    }
    
    if (WristControlFrame.data[0] == 1) {
      digitalWrite(WRIST_DIRECTION, HIGH); //CCW or CW
      ledcWrite(WRIST_SPEED, ~(WristControlFrame.data[1]));

      Serial.println("");
      Serial.print("Spinning Direction 2 at: ");
      Serial.println(WristControlFrame.data[1]);
    } 

  }
}

// 4. J5 Encoder Read
//void vWristEncoderRead()