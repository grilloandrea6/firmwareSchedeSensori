/**
 * Firmware for sensor boards
 * Paquitop
 * 
 * To be run on: STM32G0B1KBT6 on custom PCB
 * 
 * Author: Andrea Grillo S282802
 * 
 */

#include <EEPROM.h>

#define PIN_LASER     PB1   // pin of the mcu to which the laser is connected
#define PIN_SONAR     PB0   // pin of the mcu to which the sonar is connected
#define PIN_LED       PA6   // pin of the mcu to which the LED is connected
#define MAIN_ID       0x01  // can ID of the main board (receiver)
#define ALARM_TIMEOUT 2000  // minimum time between the alarms to be sent

// CAN commands
enum {
  SET_ID_CAN = 0x12,
  SET_THRESHOLD,
  DIST_REQUEST,
  ALARM_YELLOW,
  ALARM_RED,
  ALARM_LASER,
  DIST_ANS
};

// global variables to handle CANBUS
FDCAN_HandleTypeDef hfdcan1;
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
FDCAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];

// CAN ID of this board 
uint8_t myCanId;
// thresholds
uint16_t yellowThreshold, redThreshold, laserThreshold;
// flag to activate sending of distance
uint8_t distRequested = 0;
// time 
long alarmTime = -1;

// prototipi delle funzioni
static void MX_FDCAN1_Init();
static void readFromFlash();
static void writeToFlash();

void setup() {
  /* read config from FLASH memory*/
  readFromFlash();

  /* init CAN serial */
  MX_FDCAN1_Init();

  /* setting OUTPUT led */
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED,HIGH);
  delay(500);
  digitalWrite(PIN_LED,LOW);
}

void loop() {
  static int laser, sonar, alarmSend;

  /* timeout update to send alarms */
  if(alarmTime != -1 && (millis() - alarmTime) >= ALARM_TIMEOUT) alarmTime = -1;

  /* reading data from sensors */
  laser = analogRead(PIN_LASER);
  sonar = analogRead(PIN_SONAR);

  /**
   * Conditions to be met to send an alarm:
   *  - time elapsed from last alarm sent >= TIMEOUT_ALARM
   *  - RED: sonar < redThreshold
   *  - YELLOW: redThreshold < sonar < yellowThreshold
   *  - LASER: laser > laserThreshold
   */
  if(alarmTime == -1) {
    alarmSend = 1;
    TxData[1] = myCanId;
    TxHeader.DataLength = FDCAN_DLC_BYTES_2;

    if(laser > laserThreshold) /* Laser alarm */
      TxData[0] = ALARM_LASER;
    else if(sonar < redThreshold)  /* RED alarm */      
      TxData[0] = ALARM_RED;
    else if(sonar < yellowThreshold) /* YELLOW alarm */
      TxData[0] = ALARM_YELLOW;
      else alarmSend = 0;

    if(alarmSend) {
      if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK)
        Error_Handler();
      
      alarmTime = millis();
      digitalWrite(PIN_LED,HIGH);
      delay(200);
      digitalWrite(PIN_LED,LOW);
    }
  }

  /**
   * If we received a distance request 
   * send requested data
   */
  if(distRequested) {
    TxHeader.DataLength = FDCAN_DLC_BYTES_6;

    TxData[0] = DIST_ANS;
    TxData[1] = myCanId;
    TxData[2] = laser >> 8;
    TxData[3] = laser;
    TxData[4] = sonar >> 8;
    TxData[5] = sonar;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK)
      Error_Handler();

    distRequested = 0;
    digitalWrite(PIN_LED,HIGH);
    delay(200);
    digitalWrite(PIN_LED,LOW);
  }
}

/**
 * CAN init
 */
static void MX_FDCAN1_Init(void) {
  /** 
   * CAN settings for timings and clock division
   * generated with STM32CUBEIDE. To be tuned with working boards.
   */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = ENABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 16;
  hfdcan1.Init.NominalTimeSeg1 = 63;
  hfdcan1.Init.NominalTimeSeg2 = 16;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 4;
  hfdcan1.Init.DataTimeSeg1 = 5;
  hfdcan1.Init.DataTimeSeg2 = 4;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
    Error_Handler();
  
  /* Start the FDCAN module */
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    Error_Handler();

  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    Error_Handler();

  /* Prepare Tx Header */
  TxHeader.IdType = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;
  TxHeader.Identifier = MAIN_ID;
}


/**
 * Callback function to receive CAN messages
 * WARNING: do NOT change function name
*/
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == RESET)
    return;

  /* Retrieve Rx messages from RX FIFO0 */
  if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
    Error_Handler();
  
  /* We can handle only standard messages, not FD */
  if(RxHeader.IdType != FDCAN_STANDARD_ID)
    return;
  
  /* Data Parsing */
  if(RxData[0] == SET_ID_CAN && RxHeader.DataLength == FDCAN_DLC_BYTES_2) { /* Set ID CAN */
    myCanId = RxData[1];
    writeToFlash();  
  } else if(RxData[0] == SET_THRESHOLD && RxHeader.DataLength == FDCAN_DLC_BYTES_7) { /* SET_THRESHOLD */
    yellowThreshold = RxData[1] << 8 | RxData[2];
    redThreshold = RxData[3] << 8 | RxData[4];
    laserThreshold = RxData[5] << 8 | RxData[6];
    writeToFlash();
  } else if(RxHeader.StdId == myCanId && RxData[0] == DIST_REQUEST && RxHeader.DataLength == FDCAN_DLC_BYTES_1){ /* DIST_REQUEST */
    distRequested = 1;
  }
}

/* read config from FLASH */
void readFromFlash() {
  eeprom_buffer_fill();
  myCanId = eeprom_buffered_read_byte(1);
  yellowThreshold = eeprom_buffered_read_byte(2) << 8 | eeprom_buffered_read_byte(3); 
  redThreshold = eeprom_buffered_read_byte(4) << 8 | eeprom_buffered_read_byte(5);
  laserThreshold = eeprom_buffered_read_byte(6) << 8 | eeprom_buffered_read_byte(7);
}

/* write config on FLASH */
void writeToFlash() {
  eeprom_buffered_write_byte(1, myCanId);
  
  eeprom_buffered_write_byte(2, yellowThreshold >> 8);
  eeprom_buffered_write_byte(3, yellowThreshold);

  eeprom_buffered_write_byte(4, redThreshold >> 8);
  eeprom_buffered_write_byte(5, redThreshold);

  eeprom_buffered_write_byte(6, laserThreshold >> 8);
  eeprom_buffered_write_byte(7, laserThreshold);
  
  eeprom_buffer_flush();
}