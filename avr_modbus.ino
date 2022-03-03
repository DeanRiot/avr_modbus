#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "crc.hh"
/***HARDWARE DEFINES*******************************************************************/
#define MAIN_LED 10
#define DATA_LED 9
#define BUTTON 8
#define MAX_SWITCH_MODE 7
/***USART SPEED DEFINE*****************************************************************/
//#define AVR_USART_9600_VALUE 72 //for 11.059mhz and 9600 baud
#define AVR_USART_9600_VALUE 103 //for 16mhz and 9600 baud
/*****USART CONFIGURATION *************************************************************/
volatile typedef enum {FALSE = 0, TRUE = !FALSE} bool;
volatile int i = 0;
volatile byte testBuffer[8] = {0, 0, 0, 0, 0, 0, 0, 0};
volatile bool readyToExchange = false;
volatile unsigned char numOfDataToSend = 0;
volatile unsigned char numOfDataToReceive = 8;
volatile unsigned char *sendDataPtr;
volatile unsigned char *receivedDataPtr;
volatile unsigned char numOfDataSended;
volatile unsigned char numOfDataReceived;
/*MODBUS*******************************************************************************/
const unsigned char slave_id = 222;
const unsigned char slave_name[8] = {'S', 'E', 'C', '_', 'M', 'S', 'Y', 'S'};
const unsigned char reg_len = 3;
unsigned char registers[] = {0, 0, 0};
unsigned char t_frame[14];
unsigned char t_raw[6];
char16_t crc = 0;
/*****************************************************/
void UART_SendData(uint8_t *pSendData, uint8_t nNumOfDataToSend)
{
  digitalWrite(MAX_SWITCH_MODE, HIGH);
  numOfDataToSend = nNumOfDataToSend;
  numOfDataSended = 0;
  readyToExchange = FALSE;
  UCSR0B |= (1 << TXEN0);
  for(numOfDataSended; numOfDataSended<nNumOfDataToSend; numOfDataSended++)
  {
    USART_Transmit(pSendData[numOfDataSended]);
  }
  readyToExchange = 1;
}
/***************************************************************************************/
void USART_Transmit(unsigned char data)
{
/* Wait for empty transmit buffer */
while (!(UCSR0A & (1<<UDRE0)))
;
/* Put data into buffer, sends the data */
UDR0 = data;
}
/***************************************************************************************/
void UART_ReceiveData(uint8_t* pReceivedData, uint8_t nNumOfDataToReceive)
{
  digitalWrite(MAX_SWITCH_MODE, LOW);
  receivedDataPtr = pReceivedData;
  numOfDataToReceive = nNumOfDataToReceive;
  numOfDataReceived = 0;
  readyToExchange = FALSE;
  UCSR0B |= (1 << RXCIE0) | (1 << RXEN0);
}
/***************************************************************************************/
unsigned char USART_Receive(void)
{
/* Wait for data to be received */
while (!(UCSR0A & (1<<RXC0)));
/* Get and return received data from buffer */
return UDR0;
}
/***************************************************************************************/
void USART_Init( unsigned int baud )
{
  /* Set baud rate */
  UBRR0H = (unsigned char)(baud >> 8);
  UBRR0L =  (unsigned char)baud;
  /* Enable receiver and transmitter */
  UCSR0B = (1 << RXEN0) | (1 << TXEN0);// | (1 << RXCIE0);
  /* Set frame format: 8data, 2stop bit */
  UCSR0C = (0 << USBS0) | (3 << UCSZ00);
}
/***********************************************************************************/
void flushBuffers() {
  for (byte i = 0; i < 14; i++) t_frame[i] = 0;
  for (byte i = 0; i < 6; i++) t_raw[i] = 0;
  crc = 0;
}
/*Errors***********************************************************************/
void SendIvalidData(unsigned char f) {
  t_frame[0] = slave_id;
  t_frame[1] = f + 128;
  t_frame[2] = 2;
  char16_t crc = MODBUS_CRC16_v1(t_frame, 3);
  t_frame[3] =  crc & 0xff;
  t_frame[4] = crc >> 8;
  UART_SendData(t_frame, 5);
  while (!readyToExchange);
  flushBuffers();
}

void SendIvalidCMD() {
  t_frame[0] = slave_id;
  t_frame[1] = 1 + 128;
  t_frame[2] = 2;
  char16_t crc = MODBUS_CRC16_v1(t_frame, 3);
  t_frame[3] =  crc & 0xff;
  t_frame[4] = crc >> 8;
  UART_SendData(t_frame, 5);
  while (!readyToExchange);
  flushBuffers();
}
/*WRITE BYTES FROM registers to UART*******************************************/
void ReadHoldings(void) {
  if (testBuffer[3] <= 2 && testBuffer[3] >= 0) {
    registers[testBuffer[3]] = testBuffer[5];
    UART_SendData(testBuffer, 8);
    while (!readyToExchange);
    flushBuffers();
  }
  else SendIvalidData(6);
}
/***READ BYTES FROM UART TO registers ****************************************/
void WriteHoldings(void) {
  if (testBuffer[3] >= 0 &&
      testBuffer[5] <= reg_len &&
      testBuffer[3] + testBuffer[5] <= reg_len &&
      testBuffer[3] + testBuffer[5] > 0 ) {
    t_frame[0] = testBuffer[0];
    t_frame[1] = testBuffer[1];
    t_frame[2] = testBuffer[5] * 2;
    byte index = 3;
    while (testBuffer[3] <= testBuffer[5] && testBuffer[3] < reg_len) {
      for (byte i = 0; i < 2; i++) {
        if (i % 2 != 0) {
          t_frame[index] = registers[testBuffer[3]];
        }
        else {
          t_frame[index] = 0;
        }
        index++;
      }
      testBuffer[3]++;
    }
    crc = MODBUS_CRC16_v1(t_frame, t_frame[2] + 3);
    t_frame[t_frame[2] + 3] =  crc & 0xff;
    t_frame[t_frame[2] + 4] = crc >> 8;
    UART_SendData(t_frame, t_frame[2] + 5);
    while (!readyToExchange);
    flushBuffers();
  }
  else SendIvalidData(3);
}
/***********WRITE PLC NAME FROM slave_name CONST***********************/
void WriteName(void)
{
  t_frame[0] = testBuffer[0];
  t_frame[1] = 3;
  t_frame[2] = 8;
  for (byte i = 0; i < 8; i++)
  {
    t_frame[i + 3] = slave_name[i];
  }
  crc = MODBUS_CRC16_v1(t_frame, 11);
  t_frame[11] = crc & 0xff;
  t_frame[12] = crc >> 8;
  UART_SendData(t_frame, 13);
  while (!readyToExchange);
  flushBuffers();
}
/*MODBUS FRAME ANALYZER**************************************************/
void ModbusWorker()
{ 
  if (testBuffer[0] = slave_id) {
    for (byte i = 0; i < 6; i++){
      t_raw[i] = testBuffer[i];
    }
    crc = MODBUS_CRC16_v1(t_raw, 6);
    if ((crc>>8) == testBuffer[7] and (crc&0xff) == testBuffer[6]) {
    if (testBuffer[1] == 6) {
      ReadHoldings();
    }
    else if (testBuffer[1] == 3) {
      if (testBuffer[3] == slave_id && testBuffer[5] == 4) WriteName();
      else WriteHoldings();
    }
    else 
    {
      SendIvalidCMD();
    }
    }else{
      return;
    }
   }
   else{
    return;
   }
}
/***BUISNES LOGIC************************************************************/
void blink_LED(byte led)
{
     digitalWrite(led, HIGH);
    _delay_ms(50);
    digitalWrite(led, LOW);
    _delay_ms(50);
}
void read_status(void)
{
   registers[0] = digitalRead(BUTTON)?43:0;
   if(registers[0] == 43)
   {
    registers[1] = 43;
   }
   if (registers[1]== 43)
   {
    blink_LED(MAIN_LED); 
   }
}
/***************************************************************************/
int main(void)
{
  sei(); //ACCEPT INTERRUPTS
  pinMode(MAIN_LED, OUTPUT);
  pinMode(DATA_LED, OUTPUT);
  pinMode(BUTTON, INPUT);
  USART_Init(AVR_USART_9600_VALUE);//INITIALIZE USART
  while (true)
  {
    UART_ReceiveData(testBuffer, 8);//ACCEPT RECEIVE 8 BYTES FROM UART
    read_status();
    blink_LED(DATA_LED);
  }
}
/*BYTE RECEIVED INTERRUPT******************************************************************/
ISR(USART_RX_vect)
{
 *receivedDataPtr = USART_Receive();
      receivedDataPtr++;
      numOfDataReceived++;  
  if (numOfDataReceived == numOfDataToReceive)
  {
    UCSR0B &= ~((1 << RXCIE0) | (1 << RXEN0));
    readyToExchange = 1;
    ModbusWorker();
  }
}
/***************************************************************************************/
