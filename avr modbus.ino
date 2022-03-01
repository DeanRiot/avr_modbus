#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "crc.hh"
/***************************************************************************************/
//#define AVR_USART_9600_VALUE 72
#define AVR_USART_9600_VALUE 103
//USART*****************************************************************************/
volatile typedef enum {FALSE = 0, TRUE = !FALSE} bool;
volatile int i = 0;
volatile byte testBuffer[8] = {0, 0, 0, 0, 0, 0, 0, 0};
volatile bool readyToExchange;
volatile unsigned char numOfDataToSend;
volatile unsigned char numOfDataToReceive = 8;
volatile unsigned char *sendDataPtr;
volatile unsigned char *receivedDataPtr;
volatile unsigned char numOfDataSended;
volatile unsigned char numOfDataReceived;
volatile bool firstByte = true;
volatile bool RX_ended = true;
volatile bool TX_ended = false;
/*MODBUS*******************************************************************************/
const unsigned char slave_id = 1;
unsigned char slave_name[8] = {'S', 'E', 'C', '_', 'M', 'S', 'Y', 'S'};
const unsigned char reg_len = 3;
unsigned char registers[] = {0, 0, 0};
unsigned char t_frame[14];
unsigned char t_raw[6];
char16_t crc = 0;
/*****************************************************/
void UART_SendData(uint8_t *pSendData, uint8_t nNumOfDataToSend)
{
  digitalWrite(8, HIGH);
  sendDataPtr = pSendData;
  numOfDataToSend = nNumOfDataToSend;
  numOfDataSended = 0;
  readyToExchange = FALSE;
  UCSR0B |= (1 << UDRIE0) | (1 << TXEN0);
}
/***************************************************************************************/
void UART_ReceiveData(uint8_t* pReceivedData, uint8_t nNumOfDataToReceive)
{
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
while (!(UCSR0A & (1<<RXC0)))
;
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
  UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
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
  firstByte = true;
}
/*MODB*************************************************************************/
void ReadHoldings(void) {
  if (testBuffer[3] <= 2 && testBuffer[3] >= 0) {
    registers[testBuffer[3] - 1] = testBuffer[5];
    UART_SendData(testBuffer, 8);
    while (!readyToExchange);
    flushBuffers();
  }
  else SendIvalidData(6);
  firstByte = true;
}
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
  firstByte = true;
}
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
  firstByte = true;
}

void ModbusWorker()
{
      digitalWrite(10, HIGH);
    _delay_ms(50);
    digitalWrite(10, LOW);
    _delay_ms(50);
  if (testBuffer[0] = slave_id) {
    for (byte i = 0; i < 6; i++)
    {
      t_raw[i] = testBuffer[i];
    }
    crc = MODBUS_CRC16_v1(t_raw, 6);
    //  if ((crc>>8) == testBuffer[7] and (crc&0xff) == testBuffer[6]) {

    if (testBuffer[1] == 6) {
      ReadHoldings();
    }
    else if (testBuffer[1] == 3) {
      if (testBuffer[3] == slave_id && testBuffer[5] == 4) WriteName();
      else WriteHoldings();
    }
    // else SendIvalidCMD();
    //  }
    //}
    // }
  }
}

/***************************************************************************/
int main(void)
{
  sei();
  pinMode(10, OUTPUT);
  pinMode(8, OUTPUT);
  USART_Init(AVR_USART_9600_VALUE);
  while (true)
  {
    UART_ReceiveData(testBuffer, 8);
  }
}
/***************************************************************************************/
ISR(USART_UDRE_vect)
{
  UDR0 = *sendDataPtr;
  sendDataPtr++;
  numOfDataSended++;
  if (numOfDataSended == numOfDataToSend)
  {
    UCSR0B &= ~((1 << UDRIE0));
    readyToExchange = 1;
  }
}
/***************************************************************************************/
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
