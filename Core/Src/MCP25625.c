/*
    (c) 2016 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/

#include "MCP25625.h"
#include "gpio.h"

/* SPI related variables */
extern SPI_HandleTypeDef        hspi3;
#define SPI_CAN                 &hspi3
#define SPI_TIMEOUT             10
#define MCP25625_CS_HIGH()   HAL_GPIO_WritePin(CAN_CS1_GPIO_Port, CAN_CS1_Pin, GPIO_PIN_SET)
#define MCP25625_CS_LOW()    HAL_GPIO_WritePin(CAN_CS1_GPIO_Port, CAN_CS1_Pin, GPIO_PIN_RESET)

/* Prototypes */
static void SPI_Tx(uint8_t data);
static void SPI_TxBuffer(uint8_t *buffer, uint8_t length);
static uint8_t SPI_Rx(void);
static void SPI_RxBuffer(uint8_t *buffer, uint8_t length);

/* initialize MCP25625 */
bool MCP25625_Initialize(void)
{
  MCP25625_CS_HIGH();    
  
  uint8_t loop = 10;
  
  do {
    /* check SPI Ready */
    if(HAL_SPI_GetState(SPI_CAN) == HAL_SPI_STATE_READY)
      return true;
    
    loop--;
  } while(loop > 0); 
      
  return false;
}

/* change mode as configuration mode */
bool MCP25625_SetConfigMode(void)
{
  /* configure CANCTRL Register */
  MCP25625_WriteByte(MCP25625_CANCTRL, 0x80);
  
  uint8_t loop = 10;
  
  do {    
    /* confirm mode configuration */
    if((MCP25625_ReadByte(MCP25625_CANSTAT) & 0xE0) == 0x80)
      return true;
    
    loop--;
  } while(loop > 0); 
  
  return false;
}

/* change mode as normal mode */
bool MCP25625_SetNormalMode(void)
{
  /* configure CANCTRL Register */
  MCP25625_WriteByte(MCP25625_CANCTRL, 0x00);
  
  uint8_t loop = 10;
  
  do {    
    /* confirm mode configuration */
    if((MCP25625_ReadByte(MCP25625_CANSTAT) & 0xE0) == 0x00)
      return true;
    
    loop--;
  } while(loop > 0);
  
  return false;
}

/* Entering sleep mode */
bool MCP25625_SetSleepMode(void)
{
  /* configure CANCTRL Register */
  MCP25625_WriteByte(MCP25625_CANCTRL, 0x20);
  
  uint8_t loop = 10;
  
  do {    
    /* confirm mode configuration */
    if((MCP25625_ReadByte(MCP25625_CANSTAT) & 0xE0) == 0x20)
      return true;
    
    loop--;
  } while(loop > 0);
  
  return false;
}

/* MCP25625 SPI-Reset */
void MCP25625_Reset(void)
{    
  MCP25625_CS_LOW();
      
  SPI_Tx(MCP25625_RESET);
      
  MCP25625_CS_HIGH();
}

/* read single byte */
uint8_t MCP25625_ReadByte (uint8_t address)
{
  uint8_t retVal;
  
  MCP25625_CS_LOW();
  
  SPI_Tx(MCP25625_READ);
  SPI_Tx(address);
  retVal = SPI_Rx();
      
  MCP25625_CS_HIGH();
  
  return retVal;
}

/* read buffer */
void MCP25625_ReadRxSequence(uint8_t instruction, uint8_t *data, uint8_t length)
{
  MCP25625_CS_LOW();
  
  SPI_Tx(instruction);
  SPI_RxBuffer(data, length);
    
  MCP25625_CS_HIGH();
}

/* write single byte */
void MCP25625_WriteByte(uint8_t address, uint8_t data)
{    
  MCP25625_CS_LOW();  
  
  SPI_Tx(MCP25625_WRITE);
  SPI_Tx(address);
  SPI_Tx(data);  
    
  MCP25625_CS_HIGH();
}

/* write buffer */
void MCP25625_WriteByteSequence(uint8_t startAddress, uint8_t endAddress, uint8_t *data)
{    
  MCP25625_CS_LOW();
  
  SPI_Tx(MCP25625_WRITE);
  SPI_Tx(startAddress);
  SPI_TxBuffer(data, (endAddress - startAddress + 1));
  
  MCP25625_CS_HIGH();
}

/* write to TxBuffer */
void MCP25625_LoadTxSequence(uint8_t instruction, uint8_t *idReg, uint8_t dlc, uint8_t *data)
{    
  MCP25625_CS_LOW();
  
  SPI_Tx(instruction);
  SPI_TxBuffer(idReg, 4);
  SPI_Tx(dlc);
  SPI_TxBuffer(data, dlc);
       
  MCP25625_CS_HIGH();
}

/* write to TxBuffer(1 byte) */
void MCP25625_LoadTxBuffer(uint8_t instruction, uint8_t data)
{
  MCP25625_CS_LOW();
  
  SPI_Tx(instruction);
  SPI_Tx(data);
        
  MCP25625_CS_HIGH();
}

/* request to send */
void MCP25625_RequestToSend(uint8_t instruction)
{
  MCP25625_CS_LOW();
  
  SPI_Tx(instruction);
      
  MCP25625_CS_HIGH();
}

/* read status */
uint8_t MCP25625_ReadStatus(void)
{
  uint8_t retVal;
  
  MCP25625_CS_LOW();
  
  SPI_Tx(MCP25625_READ_STATUS);
  retVal = SPI_Rx();
        
  MCP25625_CS_HIGH();
  
  return retVal;
}

/* read RX STATUS register */
uint8_t MCP25625_GetRxStatus(void)
{
  uint8_t retVal;
  
  MCP25625_CS_LOW();
  
  SPI_Tx(MCP25625_RX_STATUS);
  retVal = SPI_Rx();
        
  MCP25625_CS_HIGH();
  
  return retVal;
}

/* Use when changing register value */
void MCP25625_BitModify(uint8_t address, uint8_t mask, uint8_t data)
{    
  MCP25625_CS_LOW();
  
  SPI_Tx(MCP25625_BIT_MOD);
  SPI_Tx(address);
  SPI_Tx(mask);
  SPI_Tx(data);
        
  MCP25625_CS_HIGH();
}

/* SPI Tx wrapper function  */
static void SPI_Tx(uint8_t data)
{
  HAL_SPI_Transmit(SPI_CAN, &data, 1, SPI_TIMEOUT);    
}

/* SPI Tx wrapper function */
static void SPI_TxBuffer(uint8_t *buffer, uint8_t length)
{
  HAL_SPI_Transmit(SPI_CAN, buffer, length, SPI_TIMEOUT);    
}

/* SPI Rx wrapper function */
static uint8_t SPI_Rx(void)
{
  uint8_t retVal;
  HAL_SPI_Receive(SPI_CAN, &retVal, 1, SPI_TIMEOUT);
  return retVal;
}

/* SPI Rx wrapper function */
static void SPI_RxBuffer(uint8_t *buffer, uint8_t length)
{
  HAL_SPI_Receive(SPI_CAN, buffer, length, SPI_TIMEOUT);
}