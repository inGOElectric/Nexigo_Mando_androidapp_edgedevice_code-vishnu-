//Working awesome getting data every 500mS once in a synchronized manner!!!!!!!!!!!!!!!!!!!!!
//Not maintaining library files in the repo since you can find in LG PC

#include <SPI.h>//SPI for CAN transceiver IC MCP2515
#include <mcp2515.h>//Library for MCP2515
#include "BluetoothSerial.h"//Library for bluetooth
#include <Wire.h>//

//Libraries for freeRTOS
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

BluetoothSerial ESP_BT; // Class name for bluetooth

struct can_frame canMsg;
MCP2515 mcp2515(5);//SPI pin(SS/CS) for ESP WROOM 32

//Battery Extended CAN ID's
int frame_01 = 0x9F10FA01;
int frame_11 = 0x9F11FA01;
int frame_51 = 0x9F15FA01;

//Motor controller's Extended CAN ID's
int frame_M0 = 0x90261022;
int frame_M1 = 0x90261023;

SemaphoreHandle_t xMutex;//Defining mutex to protect shared resources

//Defining structure attributes of both battery and motor controller
struct DataPacket {
  int SoC;
  float batteryCurrent;
  float batteryVoltage;
  int Wh;
  int temperatures[4];
  int speed;
  int controllerVoltage;
  int controllerCurrent;
  int mcuTemp;
  int motorTemp;
  int throttle;
  bool batteryUpdated;
  bool controllerUpdated;
} sharedData;

void setup() {
  Serial.begin(19200);//Baud Rate for Bluetooth serial which works at it's best
  ESP_BT.begin("ESP32_Control");
  SPI.begin();
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);//Setting CAN bitrate to 500kbps since both Battery and Motor controller are at 500kbps baud rate
  mcp2515.setNormalMode();

  xMutex = xSemaphoreCreateMutex();

  xTaskCreate(batteryTask, "Battery Task", 4096, NULL, 1, NULL);//Battery Task
  xTaskCreate(controllerTask, "Controller Task", 4096, NULL, 1, NULL);//Controller Task
  xTaskCreate(dataProcessingTask, "Data Processing Task", 4096, NULL, 1, NULL);//Task that synchronises and sends all the parameters through ble every 500mSec once.
}

//Through out the tasks you can check that Mutex has been used to protect the shared variable
void batteryTask(void *pvParameters) {
  while (1) {
    if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
      if (canMsg.can_id == frame_01) {
        xSemaphoreTake(xMutex, portMAX_DELAY);
        sharedData.SoC = canMsg.data[0];
        // Current
        long MSBC = canMsg.data[3] << 8;
        long MSBFC = MSBC | canMsg.data[2];
        sharedData.batteryCurrent = MSBFC * 0.01;
        // Voltage
        long MSBV = canMsg.data[5] << 8;
        long MSBFV = MSBV | canMsg.data[4];
        sharedData.batteryVoltage = MSBFV * 0.01;
        sharedData.batteryUpdated = true;
        xSemaphoreGive(xMutex);
      }
      if (canMsg.can_id == frame_11) {
        long MSBWh = canMsg.data[5] << 8;
        long MSBFWh = MSBWh | canMsg.data[4];
        xSemaphoreTake(xMutex, portMAX_DELAY);
        sharedData.Wh = MSBFWh;
        sharedData.batteryUpdated = true;
        xSemaphoreGive(xMutex);
      }
      if (canMsg.can_id == frame_51) {
        xSemaphoreTake(xMutex, portMAX_DELAY);
        sharedData.temperatures[0] = canMsg.data[2] - 80;
        sharedData.temperatures[1] = canMsg.data[3] - 80;
        sharedData.temperatures[2] = canMsg.data[4] - 80;
        sharedData.temperatures[3] = canMsg.data[5] - 80;
        sharedData.batteryUpdated = true;
        xSemaphoreGive(xMutex);
      }
    }
  }
}

void controllerTask(void *pvParameters) {
  while (1) {
    if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
      if (canMsg.can_id == frame_M0) {
        xSemaphoreTake(xMutex, portMAX_DELAY);
        // kmph
        long MSBrpm = canMsg.data[3] << 8;
        long MSBFrpm = MSBrpm | canMsg.data[2];
        sharedData.speed = MSBFrpm * 0.0807;
        // Voltage
        long MSBv = canMsg.data[5] << 8;
        long MSBFv = MSBv | canMsg.data[4];
        sharedData.controllerVoltage = MSBFv;
        // Current
        long MSBc = canMsg.data[7] << 8;
        long MSBFc = MSBc | canMsg.data[6];
        sharedData.controllerCurrent = MSBFc;
        sharedData.controllerUpdated = true;
        xSemaphoreGive(xMutex);
      }
      if (canMsg.can_id == frame_M1) {
        xSemaphoreTake(xMutex, portMAX_DELAY);
        sharedData.mcuTemp = canMsg.data[0];
        sharedData.motorTemp = canMsg.data[1];
        sharedData.throttle = canMsg.data[4];
        sharedData.controllerUpdated = true;
        xSemaphoreGive(xMutex);
      }
    }
  }
}

void dataProcessingTask(void *pvParameters) {
  while (1) {
    vTaskDelay(500 / portTICK_PERIOD_MS); // Used as a delay element for 500mS
    xSemaphoreTake(xMutex, portMAX_DELAY);
    if (sharedData.batteryUpdated && sharedData.controllerUpdated) {
      sharedData.batteryUpdated = false;
      sharedData.controllerUpdated = false;

      //Once we receive data from both the tasks writing data to Bluetooth every 500mS once
      //Battery SOC
      Serial.print("SoC : "); Serial.println(sharedData.SoC, DEC);
      int SoC = ESP_BT.write(sharedData.SoC); 
      
      //Battery Capacity
      Serial.print("Wh : "); Serial.println(sharedData.Wh);
      int KWh = ESP_BT.write(sharedData.Wh); 
      
      //Battery Thermisters
      Serial.print("T1"); Serial.print(" : "); Serial.println(sharedData.temperatures[0], DEC);
      int t1 = ESP_BT.write(sharedData.temperatures[0]);
      Serial.print("T2"); Serial.print(" : "); Serial.println(sharedData.temperatures[1], DEC);
      int t2 = ESP_BT.write(sharedData.temperatures[1]);
      Serial.print("T3"); Serial.print(" : "); Serial.println(sharedData.temperatures[2], DEC);
      int t3 = ESP_BT.write(sharedData.temperatures[2]);
      Serial.print("T4"); Serial.print(" : "); Serial.println(sharedData.temperatures[3], DEC);
      int t4 = ESP_BT.write(sharedData.temperatures[3]);
      
      //Motor Speed
      Serial.print("Speed : "); Serial.println(sharedData.speed);
      int Kmph = ESP_BT.write(sharedData.speed);
      
      //Controller Voltage/Battery Voltage
      Serial.print("Controller Voltage : "); Serial.println(sharedData.controllerVoltage);
      int V = ESP_BT.write(sharedData.controllerVoltage);
      
      //Controller current(Majorly battery Current)
      Serial.print("Controller Current : "); Serial.println(sharedData.controllerCurrent);
      int A = ESP_BT.write(sharedData.controllerCurrent);
      
      //Motor Controller Temperature
      Serial.print("MCU Temp : "); Serial.println(sharedData.mcuTemp, DEC);
      int CT = ESP_BT.write(sharedData.mcuTemp);

      //Motor Temperature
      Serial.print("Motor Temp : "); Serial.println(sharedData.motorTemp, DEC);
      int MT = ESP_BT.write(sharedData.motorTemp);

      //Throttle percentage
      Serial.print("Throttle % : "); Serial.println(sharedData.throttle, DEC);
      int Th = ESP_BT.write(sharedData.throttle);
      Serial.println("");
    }
    xSemaphoreGive(xMutex);
  }
}

void loop() {
  // Nothing to do here, all work is done in tasks
}
