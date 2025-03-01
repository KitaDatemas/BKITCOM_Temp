/*
 * Hardware_Interface_Layer.h
 *
 *  Created on: Jan 31, 2025
 *      Author: ASUS
 */

#ifndef INC_HARDWARE_INTERFACE_LAYER_H_
#define INC_HARDWARE_INTERFACE_LAYER_H_
#include <stdlib.h>
#include "main.h"
#include "Error_Code.h"
#include "softwareTimer.h"
#include "common.h"
/*
 * Predefined Protocols
 * uart1, uart2
 * i2c1, i2c2
 * spi1, spi2
 * */

/*
 * Predefined Timer
 * TIM2
 * */

/*
 * RS485_Configuration
 * PC11: TX
 * PC10: RX
 * PA15: Enable
 * */

#define UART_BAUDRATE 9600
#define RS485_BAUDRATE 9600

#define HEADER_SIZE 16
#define SEND_TIMEOUT 1000
#define RECEIVE_TIMEOUT 1000

typedef struct {
	BKITCOM_Role_type_t uart_role;
	UART_HandleTypeDef uart;
	uint16_t baudrate;
} UART_type_t;

typedef struct {
	BKITCOM_Role_type_t i2c_role;
	uint16_t address;
	I2C_HandleTypeDef i2c;
} I2C_type_t;

typedef struct {
	BKITCOM_Role_type_t spi_role;
	uint16_t CPOL;
	uint16_t CPHA;
	SPI_HandleTypeDef spi;
} SPI_type_t;

typedef struct {
	BKITCOM_Role_type_t RS485_role;
	UART_HandleTypeDef uart;
	uint32_t baudrate;
	uint16_t address;
} RS485_type_t;

typedef struct {
	UART_type_t uart_Handler;
	I2C_type_t i2c_Handler;
	SPI_type_t spi_Handler;
	RS485_type_t RS485_Handler;
} BKITCOM_type_t;

typedef enum {IDLE, SENDING, RECEIVING, ERROR_STATE} HIL_State;

extern BKITCOM_Role_type_t role;
extern HIL_State state;

extern uint8_t uart_en, i2c_en, spi_en, RS485_en;
extern BKITCOM_type_t Com;
extern TIM_HandleTypeDef hw_tim;

extern Protocol Com_Send_Signal;
extern Protocol Com_Receive_Signal;
extern CRC_Status_type_t CRC_CheckFlag;
extern uint8_t Received_Flag;

extern uint8_t Header_Buffer[4][HEADER_SIZE];/*The index of protcols is UART, I2C, SPI and RS485 in order*/
extern uint8_t * Payload_Buffer[4];
extern uint32_t count;

BKITCOM_Error_Code uart_config(uint8_t uart_x, uint32_t baudrate, BKITCOM_Role_type_t role);
BKITCOM_Error_Code i2c_config(uint8_t i2c_x, uint16_t address, BKITCOM_Role_type_t role);
BKITCOM_Error_Code spi_config(uint8_t spi_x, uint16_t CPOL, uint16_t CPHA, BKITCOM_Role_type_t role);
BKITCOM_Error_Code RS485_config(uint8_t uart_x, uint32_t baudrate, uint16_t address, BKITCOM_Role_type_t role);

void hw_tim_init(void);
void slave_init(void);
void i2c_init(void);
void spi_init(void);
void uart_init(void);
void RS485_init(void);
void hw_init(void);
void hw_reboot(Protocol com);

BKITCOM_Error_Code hw_send(Protocol com, uint16_t address, uint8_t * data, uint32_t data_length);
BKITCOM_Error_Code hw_receive(Protocol com, uint8_t * data, uint8_t data_length, uint32_t timeout);
BKITCOM_Error_Code ErrorCode_Handler (HAL_StatusTypeDef Error_Code);

void Hardware_Interface_Layer_FSM (void);

void Protocol_Buffer_Malloc (Protocol protocol, uint32_t length);
#endif /* INC_HARDWARE_INTERFACE_LAYER_H_ */
