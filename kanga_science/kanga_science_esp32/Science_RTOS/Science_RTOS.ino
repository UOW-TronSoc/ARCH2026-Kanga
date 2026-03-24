/*  INCLUSIONS  */
#include <ESP32-TWAI-CAN.hpp>
#include <ESP32Servo.h>
#include <Arduino.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


/*  DEFINITIONS  */
#define BAUDRATE 250 // 250000kb/s


// PIN Definitions
#define CAN_TX 5
#define CAN_RX 4
#define HEATING_PIN 22
#define COOLING_PIN 23
#define ACTUATOR_PWM_PIN 25
#define ACTUATOR_DIR_PIN 26

// LEDC config for linear actuator (Cytron)
#define ACTUATOR_PWM_CH   0
#define ACTUATOR_PWM_FREQ 5000
#define ACTUATOR_PWM_RES  8

// CAN IDS
#define CAN_ID_AUGER_DRILL 820
#define CAN_ID_AUGER_ACTUATOR 821  // D0: direction (0/1), D1: PWM 0-255
#define CAN_ID_THERMISTOR_READINGS 823
#define CAN_ID_HEATING_CONTROL 824
#define CAN_ID_COOLING_CONTROL 829
#define CAN_ID_ULTRASONIC_READING 825 //??
#define CAN_ID_ILMENITE_SERVO 826
#define CAN_ID_SPECTROPHOTOMETER_PERCENT 827
#define CAN_ID_ILMENITE_LIMIT_SWITCH 828


/*  RTOS HANDLERS  */

//Queue Creation
QueueHandle_t xTransmitFrame;
QueueHandle_t xAugerDrillFrame;
QueueHandle_t xAugerActuatorFrame;
QueueHandle_t xIlmeniteServoFrame;
QueueHandle_t xHeatingFrame;
QueueHandle_t xCoolingFrame;

//Task Handler Creation
TaskHandle_t xReadHandle = NULL;
TaskHandle_t xWriteHandle = NULL;
TaskHandle_t xAugerDrillHandle = NULL;
TaskHandle_t xAugerActuatorHandle = NULL;
TaskHandle_t xAugerHeatingHandle = NULL;
TaskHandle_t xAugerCoolingHandle = NULL;
TaskHandle_t xAugerUltrasonicHandle = NULL;
TaskHandle_t xIlmeniteServoHandle = NULL;
TaskHandle_t xIlmeniteLimitSwitchHandle = NULL;
TaskHandle_t xIlmeniteSpectrophotometerHandle = NULL;

/*  FUNCTION DECLARATIONS  */
void vCanReadLoop(void *pvParameters);
void vCanWriteLoop(void *pvParameters);
void vIlmeniteLimitSwitch(void *pvParameters);
void vAugerHeatingControl(void *pvParameters);
void vAugerCoolingControl(void *pvParameters);
void vAugerDrillControl(void *pvParameters);
void vLinearActuatorControl(void *pvParameters);
void vAugerUltrasonicControl(void *pvParameters);
void vIlmeniteServoControl(void *pvParameters);
void vIlmeniteSpectrophotometer(void *pvParameters);


void setup() {
// Setup serial for debbuging.
  Serial.begin(115200);

//Pin Setup
  pinMode(HEATING_PIN, OUTPUT);
  pinMode(COOLING_PIN, OUTPUT);
  digitalWrite(HEATING_PIN, LOW);
  digitalWrite(COOLING_PIN, LOW);
  Serial.print("Heating pin ");
  Serial.print(HEATING_PIN);
  Serial.println(" -> OUTPUT, LOW");
  Serial.print("Cooling pin ");
  Serial.print(COOLING_PIN);
  Serial.println(" -> OUTPUT, LOW");

  // Linear actuator (Cytron) - direction pin + LEDC PWM, start stopped
  pinMode(ACTUATOR_DIR_PIN, OUTPUT);
  digitalWrite(ACTUATOR_DIR_PIN, LOW);
  ledcAttachChannel(ACTUATOR_PWM_PIN, ACTUATOR_PWM_FREQ, ACTUATOR_PWM_RES, ACTUATOR_PWM_CH);
  ledcWrite(ACTUATOR_PWM_PIN, 0);
  Serial.println("Linear actuator pins configured");


// CAN Setup
  if(ESP32Can.begin(ESP32Can.convertSpeed(BAUDRATE), CAN_TX, CAN_RX, 10, 10)) {
      Serial.println("CAN bus started!");
  } else {
      Serial.println("CAN bus failed!");
  }

// Queue Creation
  xAugerDrillFrame = xQueueCreate(5, sizeof(CanFrame)); // ID:820, D1: Direction, D2: Speed
  xAugerActuatorFrame = xQueueCreate(5, sizeof(CanFrame)); // ID:821, D1: Direction, D2: Speed
  xHeatingFrame = xQueueCreate(5, sizeof(CanFrame)); // ID:824, D1: (0 = Off, 1 = On)
  xCoolingFrame = xQueueCreate(5, sizeof(CanFrame)); // ID:829, D1: (0 = Off, 1 = On)
  xIlmeniteServoFrame = xQueueCreate(5, sizeof(CanFrame)); // ID:826, D1: Degrees
  xTransmitFrame = xQueueCreate(15, sizeof(CanFrame)); //ID:823, D1+D2: Thermistor1... | ID 827: D1 = Processed Ilmenite Data


// Task Creation
  xTaskCreatePinnedToCore(
                          vCanReadLoop,
                          "Can Read Loop",
                          2000,
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
                          vIlmeniteLimitSwitch,
                          "Ilmenite Limit Switch Reading",
                          2000,
                          NULL,
                          7,
                          &xIlmeniteLimitSwitchHandle,
                          1);


  xTaskCreatePinnedToCore(
                          vAugerHeatingControl,
                          "Auger Heating Control Loop",
                          4096,
                          NULL,
                          6,
                          &xAugerHeatingHandle,
                          1);


  xTaskCreatePinnedToCore(
                          vAugerCoolingControl,
                          "Auger Cooling Control Loop",
                          4096,
                          NULL,
                          6,
                          &xAugerCoolingHandle,
                          1);


  xTaskCreatePinnedToCore(
                          vAugerDrillControl,
                          "Auger Drill Control Loop",
                          2000,
                          NULL,
                          5,
                          &xAugerDrillHandle,
                          1);


  xTaskCreatePinnedToCore(
                          vLinearActuatorControl,
                          "Linear Actuator Control Loop",
                          4096,
                          NULL,
                          4,
                          &xAugerActuatorHandle,
                          1);


  xTaskCreatePinnedToCore(
                          vAugerUltrasonicControl,
                          "Auger Ultrasonic Read Loop",
                          2000,
                          NULL,
                          3,
                          &xAugerUltrasonicHandle,
                          1);


  xTaskCreatePinnedToCore(
                          vIlmeniteServoControl,
                          "Ilmenite Servo Control Loop",
                          2000,
                          NULL,
                          2,
                          &xIlmeniteServoHandle,
                          1);


  xTaskCreatePinnedToCore(
                          vIlmeniteSpectrophotometer,
                          "Ilmenite Spectrophotometer Read/Process Loop",
                          2000,
                          NULL,
                          1,
                          &xIlmeniteSpectrophotometerHandle,
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
      // Serial.println("Write Failed");
    }

    vTaskDelay(pdMS_TO_TICKS(10000));

  }
}

// 2. Reading from CAN
void vCanReadLoop(void *pvParameters) {
  Serial.println("Enter Read Task");
  
  CanFrame receivedFrame;
  Serial.println("[CAN RX] Waiting for frames...");

  for( ;; )
  { 
    ESP32Can.readFrame(receivedFrame, portMAX_DELAY);
    Serial.print("[CAN RX] ID: ");
    Serial.print(receivedFrame.identifier);
    Serial.print("  D0: ");
    Serial.println(receivedFrame.data[0]);

    // Switch on incoming CAN ID and route to the appropriate queue.
    switch(receivedFrame.identifier) {
      case CAN_ID_AUGER_ACTUATOR:
        Serial.println("[CAN RX] -> Actuator queue");
        xQueueSend(xAugerActuatorFrame, &receivedFrame, 0);
        break;
      case CAN_ID_HEATING_CONTROL:
        Serial.println("[CAN RX] -> Heating queue");
        xQueueSend(xHeatingFrame, &receivedFrame, 0);
        break;
      case CAN_ID_COOLING_CONTROL:
        Serial.println("[CAN RX] -> Cooling queue");
        xQueueSend(xCoolingFrame, &receivedFrame, 0);
        break;
      default:
        Serial.println("[CAN RX] Unknown ID, ignored");
        break;
    }
    
  }
}


/*  CORE 1 TASKS BY PRIORITY  */

// 1. Ilmenite Limit Switch

void vIlmeniteLimitSwitch(void * pvParameters) {
  CanFrame LimitSwitchFrame;

  for (;;) {
    //Read Limit Switch
    //Queue Limit Switch data on event
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// 2. Heating Control Loop

void vAugerHeatingControl(void * pvParameters) {
  Serial.println("[HEATING] Task started");
  Serial.flush();
  CanFrame HeatingCommandFrame;

  for (;;) {
    Serial.println("[HEATING] Waiting on queue...");
    Serial.flush();
    xQueueReceive(xHeatingFrame, &HeatingCommandFrame, portMAX_DELAY);
    Serial.println("[HEATING] Frame received, driving pin");
    Serial.flush();
    bool state = HeatingCommandFrame.data[0];
    digitalWrite(HEATING_PIN, state ? LOW : HIGH);
    Serial.print("[HEATING] -> ");
    Serial.println(state ? "ON" : "OFF");
    Serial.flush();
  }
}

// 3. Cooling Control Loop

void vAugerCoolingControl(void * pvParameters) {
  Serial.println("[COOLING] Task started");
  Serial.flush();
  CanFrame CoolingCommandFrame;

  for (;;) {
    Serial.println("[COOLING] Waiting on queue...");
    Serial.flush();
    xQueueReceive(xCoolingFrame, &CoolingCommandFrame, portMAX_DELAY);
    Serial.println("[COOLING] Frame received, driving pin");
    Serial.flush();
    bool state = CoolingCommandFrame.data[0];
    digitalWrite(COOLING_PIN, state ? LOW : HIGH);
    Serial.print("[COOLING] -> ");
    Serial.println(state ? "ON" : "OFF");
    Serial.flush();
  }
}

// 4. Auger Drill Control

void vAugerDrillControl(void * pvParameters) {
  CanFrame AugerDrillFrame;

  for (;;) {
    //Wait for Drill input
    //Command based on drill input
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// 5. Linear Actuator Control (Cytron, D0: direction, D1: PWM 0-255)

void vLinearActuatorControl(void * pvParameters) {
  Serial.println("[ACTUATOR] Task started");
  Serial.flush();
  CanFrame ActuatorFrame;

  for (;;) {
    Serial.println("[ACTUATOR] Waiting on queue...");
    Serial.flush();
    xQueueReceive(xAugerActuatorFrame, &ActuatorFrame, portMAX_DELAY);

    uint8_t direction = ActuatorFrame.data[0];
    uint8_t speed     = ActuatorFrame.data[1];

    digitalWrite(ACTUATOR_DIR_PIN, direction ? HIGH : LOW);
    ledcWrite(ACTUATOR_PWM_PIN, speed);

    Serial.print("[ACTUATOR] dir: ");
    Serial.print(direction);
    Serial.print("  pwm: ");
    Serial.println(speed);
    Serial.flush();
  }
}

// 6. Ultrasonic Reading Loop

void vAugerUltrasonicControl(void * pvParameters) {
  CanFrame AugerUltrasonicFrame;

  for (;;) {
    //Read Ultrasonic data
    //Put into Transmit Queue
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// 7. Ilmenite Servo Control

void vIlmeniteServoControl(void * pvParameters) {
  CanFrame IlmeniteServoFrame;

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// 8. Ilmenite Spectrophotographer Reading + Processing

void vIlmeniteSpectrophotometer(void * pvParameters) {
  CanFrame IlmenitePercentageFrame;

  for (;;) {
    //Read Ilmenite Data
    //Process it
    //Form Frame
    //Queue Frame
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}



