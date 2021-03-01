#include <SX126x.h>

#define RF_FREQUENCY                      915000000                 // Hz  center frequency
#define TX_OUTPUT_POWER                   -3                        // dBm tx output power
#define LORA_BANDWIDTH                    SX126X_LORA_BW_125_0      // Bandwidth
#define LORA_SPREADING_FACTOR             8                         // spreading factor [SF5..SF12]
#define LORA_CODINGRATE                   1                         // [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
#define LORA_PREAMBLE_LENGTH              8                         // Same for Tx and Rx
#define LORA_FIX_LENGTH_PAYLOAD_ON        false                     // variable data payload
#define LORA_PACKET_CRC_ENABLE            false                     // check payload crc
#define LORA_IQ_INVERSION_ON              false                     // use inverted IQ
#define LORA_PAYLOADLENGTH                0                         // 0: variable receive length, 1..255 payloadlength

// PINS
#define LORA_SPI_SELECT                   48
#define LORA_RESET                        49
#define LORA_BUSY                         47
#define LORA_DIO_1                        2

SX126x lora(LORA_SPI_SELECT,              //Port-Pin Output: SPI select
            LORA_RESET,                   //Port-Pin Output: Reset 
            LORA_BUSY);                   //Port-Pin Input:  Busy

void setup() {
  pinMode(LORA_DIO_1, INPUT);
  attachInterrupt(digitalPinToInterrupt(LORA_DIO_1), loraISR, RISING);
  
  Serial.begin(9600);
  delay(500);
  Serial.println("Starting Up...");
  
  uint8_t res = lora.ModuleConfig(SX126X_PACKET_TYPE_LORA, RF_FREQUENCY, TX_OUTPUT_POWER);                

  if ( res != ERR_NONE ) {
    Serial.println("Error Initializing SX126x");
    while(true) delay(10);
  }

  lora.LoRaBegin(
    LORA_SPREADING_FACTOR, 
    LORA_BANDWIDTH, 
    LORA_CODINGRATE, 
    LORA_PREAMBLE_LENGTH, 
    LORA_PAYLOADLENGTH, 
    LORA_PACKET_CRC_ENABLE,
    LORA_IQ_INVERSION_ON
  );

  Serial.println("SX126x Initialized");
}

uint8_t* pRxData = new uint8_t[128];

void loop() {
  delay(2000);
}

void loraISR() {
  lora.Dio1Interrupt(); // Update the lora driver
  
  uint8_t rxLen = lora.Receive(pRxData, 128);
  if ( rxLen > 0 ) { 
    int8_t rssi, snr;
    uint16_t val = ((uint16_t)pRxData[0]<<8)&0xFF00;
    val |= ((uint16_t)pRxData[1])&0x00FF;
    Serial.println("Received: " + String(val));
    lora.ReceiveStatus(&rssi, &snr);
    Serial.print("SNR: ");
    Serial.println(snr, DEC);
    Serial.print("RSSI: ");
    Serial.println(rssi, DEC);
  }
}
