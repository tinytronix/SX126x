
#include <SX126x.h>

#define RF_FREQUENCY                                433000000 // Hz  center frequency
#define TX_OUTPUT_POWER                             -3        // dBm tx output power
#define LORA_BANDWIDTH                              4         // bandwidth=125khz  0:250kHZ,1:125kHZ,2:62kHZ,3:20kHZ.... look for radio line 392                                                               
#define LORA_SPREADING_FACTOR                       7         // spreading factor=11 [SF5..SF12]
#define LORA_CODINGRATE                             4         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]

#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false     // variable data payload
#define LORA_IQ_INVERSION_ON                        false
#define LORA_PAYLOADLENGTH                          0         // 0: variable receive length 
                                                              // 1..255 payloadlength

SX126x  lora(PD5, //SPI select
             PD6, //reset
             PD7, //busy
             PB0  //interrupt DIO1 
             );
             
bool b = false;

void setup() 
{
  Serial.begin(9600);
  
  delay(500);
               
  lora.begin(SX126X_PACKET_TYPE_LORA,   //LoRa or FSK, FSK currently not supported
             RF_FREQUENCY,              //frequency in Hz
             TX_OUTPUT_POWER);          //tx power in dBm
             
  lora.LoRaConfig(LORA_SPREADING_FACTOR, 
                    LORA_BANDWIDTH, 
                    LORA_CODINGRATE, 
                    LORA_PREAMBLE_LENGTH, 
                    LORA_PAYLOADLENGTH, 
                    true,              //crcOn  
                    false);             //invertIrq
   
  Serial.println("Test starting");
}

uint8_t i;
uint8_t rxLen = 0;
uint8_t* pRxData = new uint8_t[128];
uint32_t errors = 0;
int count = 0;
unsigned long start;

//send data and await echo
void loop() 
{
  if ( b == false )
  {
    start = millis();
    lora.Send(&i, 1, true);
    i++;
    b = true;
  }
  else
  {
    rxLen = lora.Receive(pRxData, 128);
    if ( rxLen > 0 )
    { 
      unsigned long roundtrip = millis() - start;
      Serial.print(pRxData[0], DEC); Serial.print(" "); Serial.print(roundtrip); Serial.println("ms");
      b = false;
      count = 0;
    }
    else
    {
      count++;
    }
    if ( count == 500 )
    {
      count = 0;
      Serial.println("Retrigger");
      b = false;
    }
  }
}
