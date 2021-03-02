/* LoraRXAsync.ino
 * 
 * A simple Async RX example.
 */

#include <SX126x.h>

#define RF_FREQUENCY                                915000000 // Hz  center frequency
#define TX_OUTPUT_POWER                             -3        // dBm tx output power
#define LORA_BANDWIDTH                              SX126X_LORA_BW_125_0         // Bandwidth
#define LORA_SPREADING_FACTOR                       8         // spreading factor [SF5..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false     // variable data payload
#define LORA_PACKET_CRC_ENABLE                      false     // check payload crc
#define LORA_IQ_INVERSION                           false     // use inverted IQ
#define LORA_PAYLOADLENGTH                          0         // 0: variable receive length, 1..255 payloadlength

// UNO PINS
#define LORA_SPI_SELECT                             10
#define LORA_RESET                                  9
#define LORA_BUSY                                   8
#define LORA_DIO_1                                  2

SX126x  lora(LORA_SPI_SELECT,              //Port-Pin Output: SPI select
             LORA_RESET,                   //Port-Pin Output: Reset 
             LORA_BUSY,                    //Port-Pin Input:  Busy
             LORA_DIO_1);                  //Port-Pin Input:  Dio1

void setup() {
  Serial.begin(9600);
  delay(500);
  Serial.println("Starting Up...");
  
  uint8_t rv = lora.ModuleConfig(SX126X_PACKET_TYPE_LORA, RF_FREQUENCY, TX_OUTPUT_POWER);

  if ( rv == ERR_NONE ) {
    lora.LoRaBegin(
      LORA_SPREADING_FACTOR, 
      LORA_BANDWIDTH, 
      LORA_CODINGRATE, 
      LORA_PREAMBLE_LENGTH, 
      LORA_PAYLOADLENGTH, 
      LORA_PACKET_CRC_ENABLE,
      LORA_IQ_INVERSION
    );
  
    // Register your RX done hook with the driver
    lora.setRxDoneHook(loraRxDone);
  
    Serial.println("SX126x Initialized");
  }
  else {
    Serial.print("Error Initializing SX126x: " );
    Serial.println(rv);
  }
}


void loop() {
  // Do work while the SX126x handles receiving!
  delay(10);
}


// Lora RX Done ISR
void loraRxDone(uint8_t rxStatus, uint8_t* pRxData, uint16_t len) {
  if ( rxStatus == ERR_NONE ) {
    int8_t rssi, snr;
    uint16_t val = ((uint16_t)pRxData[0]<<8)&0xFF00;
    val |= ((uint16_t)pRxData[1])&0x00FF;
    Serial.print("Received: ");
    Serial.println(val);
    Serial.print("Length: ");
    Serial.println(len);
    lora.ReceiveStatus(&rssi, &snr);
    Serial.print("SNR: ");
    Serial.println(snr, DEC);
    Serial.print("RSSI: ");
    Serial.println(rssi, DEC);
  }
  else {
    Serial.print("Lora RX Error: ");
    Serial.println(rxStatus);
  }
}
