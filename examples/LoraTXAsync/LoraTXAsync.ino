/* LoraTXAsync.ino
 * 
 * A simple Async TX example.
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
    
    // Register your TX done hook with the driver if you want
    lora.setTxDoneHook(loraTxDone);
    
    Serial.println("SX126x Initialized");
  }
  else {
    Serial.print("Error Initializing SX126x: ");
    Serial.println(rv);
  }
}

uint16_t i = 0;
uint8_t* data = new uint8_t[2];
unsigned long startMillis = millis();

void loop() {
  data[0] = (i >> 8) & 0x00FF;
  data[1] = i & 0x00FF;
  startMillis = millis();
  lora.SendAsync(data, 2);
  
  // Do work while SX126x sends your message!
  delay(1000);
}


// Lora TX Done ISR
void loraTxDone(uint8_t txStatus) {
  unsigned long sendTime = millis()-startMillis;
  Serial.print("Sent: ");
  Serial.print(i);
  Serial.print(" in ");
  Serial.print(sendTime);
  Serial.print(" ms ");
  Serial.println(txStatus ? "with error(s)" : "successfully");
  i++;
}
