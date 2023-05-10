#include <EEPROM.h>

#define PIN_LASER PB1
#define PIN_SONAR PB0
#define PIN_LED
#define MAIN_ID 0x01


enum {
  setIdCan = 0x12,
  setSoglie,
  distRequest,
  alarmGiallo,
  alarmRosso,
  ansDist
};

FDCAN_HandleTypeDef hfdcan1;
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
FDCAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];

uint8_t myCanId;
uint16_t sogliaGiallo, sogliaRosso;
uint8_t distRequested = 0;

static void MX_FDCAN1_Init(void);
static void FDCAN_Config(void);
static void readFromFlash();
static void writeToFlash();
static int convertLaser(int las) { return las;}
static int convertSonar(int son) { return son;}

void setup() {
  /* leggo configurazione dalla memoria FLASH*/
  readFromFlash();

  /* inizializzazione seriale CAN */
  MX_FDCAN1_Init();
  FDCAN_Config();
}

void loop() {
  int laser, sonar;

  /* lettura dati da sensore e riscalamento */
  laser = convertLaser(analogRead(PIN_LASER));
  sonar = convertSonar(analogRead(PIN_SONAR));

  if(laser < sogliaRosso || sonar < sogliaRosso) {
  /**
   * Condizioni per mandare un allarme:
   *  - tempo >= TIMEOUT_ALARM trascorso dall'ultimo allarme
   *  - ROSSO: laser < sogliaRosso oppure sonar < sogliaRosso
   *  - GIALLO: sogliaRosso < laser < sogliaGiallo
   */
    TxHeader.Identifier = MAIN_ID;
    TxHeader.DataLength = FDCAN_DLC_BYTES_2;
    TxData[0] = alarmRosso;
    TxData[1] = myCanId;
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK)
      Error_Handler();
  } else if(sonar < sogliaGiallo) {
    TxHeader.Identifier = MAIN_ID;
    TxHeader.DataLength = FDCAN_DLC_BYTES_2;
    TxData[0] = alarmGiallo;
    TxData[1] = myCanId;
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK)
      Error_Handler();
  }

  /**
   * Se si è ricevuta una richiesta di distanza,
   * procedo a inviare i dati richiesti
   */
  if(distRequested) {
    TxHeader.Identifier = MAIN_ID;
    TxHeader.DataLength = FDCAN_DLC_BYTES_6;

    TxData[0] = ansDist;
    TxData[1] = myCanId;
    TxData[2] = laser >> 8;
    TxData[3] = laser;
    TxData[4] = sonar >> 8;
    TxData[5] = sonar;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK)
      Error_Handler();

    distRequested = 0;
  }
}

/**
 * Inizializzazione CAN
 * ToDo check valori
 */
static void MX_FDCAN1_Init(void) {
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
}

static void FDCAN_Config(void)
{
  FDCAN_FilterTypeDef sFilterConfig;

  /* Configure Rx filter */
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x321;
  sFilterConfig.FilterID2 = 0x7FF;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
    Error_Handler();

  /* Configure global filter:
     Filter all remote frames with STD and EXT ID
     Reject non matching frames with STD ID and EXT ID */
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
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
}

/**
  * @brief  Rx FIFO 0 callback.
  * @param  hfdcan: pointer to an FDCAN_HandleTypeDef structure that contains
  *         the configuration information for the specified FDCAN.
  * @param  RxFifo0ITs: indicates which Rx FIFO 0 interrupts are signalled.
  *         This parameter can be any combination of @arg FDCAN_Rx_Fifo0_Interrupts.
  * @retval None
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
    /* Retrieve Rx messages from RX FIFO0 */
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
      Error_Handler();

    if ((RxHeader.Identifier == myCanId) && (RxHeader.IdType == FDCAN_STANDARD_ID)/* && (RxHeader.DataLength == FDCAN_DLC_BYTES_2)*/)
    {
      switch(RxData[0]) {
          case setIdCan:
            myCanId = TxData[1];
            writeToFlash();
          break;
          case setSoglie:
            sogliaGiallo = RxData[1] << 8 | RxData[2];
            sogliaRosso = RxData[3] << 8 | RxData[4];
            writeToFlash();
          break;
          case distRequest:
            distRequested = 1;
          break;
      }

    }
  }
}

/* leggo i parametri impostati dalla flash */
void readFromFlash() {
    eeprom_buffer_fill();
    myCanId = eeprom_buffered_read_byte(1);
    sogliaGiallo = eeprom_buffered_read_byte(2) << 8 | eeprom_buffered_read_byte(3); // dovrò stare attento con i tipi se le soglie sono uint16, analogRead ritorna int
    sogliaRosso = eeprom_buffered_read_byte(4) << 8 | eeprom_buffered_read_byte(5); // dovrò stare attento con i tipi se le soglie sono uint16
}

/* scrivo sulla flash i parametri */
void writeToFlash() {
    eeprom_buffered_write_byte(1, myCanId);

    eeprom_buffered_write_byte(2, sogliaGiallo >> 8);
    eeprom_buffered_write_byte(3, sogliaGiallo);

    eeprom_buffered_write_byte(4, sogliaRosso >> 8);
    eeprom_buffered_write_byte(5, sogliaRosso);
    
    eeprom_buffer_flush();
}