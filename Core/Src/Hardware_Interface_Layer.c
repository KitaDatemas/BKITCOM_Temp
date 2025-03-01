/*
 * Hardware_Interface_Layer.c
 *
 *  Created on: Jan 31, 2025
 *      Author: ASUS
 */
#include "Hardware_Interface_Layer.h"

uint8_t i2c_en = 0, spi_en = 0, uart_en = 0, RS485_en = 0;
TIM_HandleTypeDef hw_tim;
uint8_t Received_Flag = 0;
uint16_t prev_i2c_slave_address;
uint16_t prev_spi_slave_address;

//BKITCOM_Error_Code code;
BKITCOM_type_t Com;
Protocol Com_Send_Signal = NONE;
Protocol Com_Receive_Signal = NONE;
CRC_Status_type_t CRC_CheckFlag = NONE;
uint32_t count;

uint8_t Header_Buffer[4][HEADER_SIZE],
		* Payload_Buffer[4] = {NULL};

HIL_State state = IDLE;

void hw_reboot(Protocol com){
	switch (com) {
		case UART:
			HAL_UART_DeInit(&Com.uart_Handler.uart);
			RS485_init();
			break;
		case I2C:
			HAL_I2C_DeInit(&Com.i2c_Handler.i2c);
			i2c_init();
			break;
		case SPI:
			HAL_SPI_DeInit(&Com.spi_Handler.spi);
			spi_init();
			break;
		case RS485:
			HAL_UART_DeInit(&Com.RS485_Handler.uart);
			RS485_init();
			break;
		default:
			return;
	}
}

void i2c_init(void){
	Com.i2c_Handler.i2c.Init.ClockSpeed = 100000;
	Com.i2c_Handler.i2c.Init.DutyCycle = I2C_DUTYCYCLE_2;
	Com.i2c_Handler.i2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	Com.i2c_Handler.i2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	Com.i2c_Handler.i2c.Init.OwnAddress2 = 0;
	Com.i2c_Handler.i2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	Com.i2c_Handler.i2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&Com.i2c_Handler.i2c) != HAL_OK)
	{
		Error_Handler();
	}
}


void spi_init(void){
	Com.spi_Handler.spi.Init.Direction = SPI_DIRECTION_2LINES;
	Com.spi_Handler.spi.Init.DataSize = SPI_DATASIZE_8BIT;
	Com.spi_Handler.spi.Init.CLKPolarity = Com.spi_Handler.CPOL;
	Com.spi_Handler.spi.Init.CLKPhase = Com.spi_Handler.CPHA;
	Com.spi_Handler.spi.Init.NSS = SPI_NSS_SOFT;
	Com.spi_Handler.spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	Com.spi_Handler.spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
	Com.spi_Handler.spi.Init.TIMode = SPI_TIMODE_DISABLE;
	Com.spi_Handler.spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	Com.spi_Handler.spi.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&Com.spi_Handler.spi) != HAL_OK)
	{
		Error_Handler();
	}
}

void uart_init(void){
	Com.uart_Handler.uart.Init.BaudRate = Com.uart_Handler.baudrate;
	Com.uart_Handler.uart.Init.WordLength = UART_WORDLENGTH_8B;
	Com.uart_Handler.uart.Init.StopBits = UART_STOPBITS_1;
	Com.uart_Handler.uart.Init.Parity = UART_PARITY_NONE;
	Com.uart_Handler.uart.Init.Mode = UART_MODE_TX_RX;
	Com.uart_Handler.uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	Com.uart_Handler.uart.Init.OverSampling = UART_OVERSAMPLING_16;
	HAL_StatusTypeDef error_code = HAL_UART_Init(&Com.uart_Handler.uart);
	if (error_code != HAL_OK)
	{
		Error_Handler();
	}
}

void RS485_init(void){
	Com.RS485_Handler.uart.Init.BaudRate = Com.RS485_Handler.baudrate;
	Com.RS485_Handler.uart.Init.WordLength = UART_WORDLENGTH_8B;
	Com.RS485_Handler.uart.Init.StopBits = UART_STOPBITS_1;
	Com.RS485_Handler.uart.Init.Parity = UART_PARITY_NONE;
	Com.RS485_Handler.uart.Init.Mode = UART_MODE_TX_RX;
	Com.RS485_Handler.uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	Com.RS485_Handler.uart.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&Com.RS485_Handler.uart) != HAL_OK)
	{
		Error_Handler();
	}
}

void hw_tim_init(void) {
	  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	  TIM_MasterConfigTypeDef sMasterConfig = {0};

	  hw_tim.Instance = TIM2;
	  hw_tim.Init.Prescaler = 35999;
	  hw_tim.Init.CounterMode = TIM_COUNTERMODE_UP;
	  hw_tim.Init.Period = 1;
	  hw_tim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	  hw_tim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	  if (HAL_TIM_Base_Init(&hw_tim) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE2;
	  sClockSourceConfig.ClockPolarity = TIM_CLOCKPOLARITY_NONINVERTED;
	  sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;
	  sClockSourceConfig.ClockFilter = 0;
	  if (HAL_TIM_ConfigClockSource(&hw_tim, &sClockSourceConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	  if (HAL_TIMEx_MasterConfigSynchronization(&hw_tim, &sMasterConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

void slave_init(void){
	if (uart_en)	hw_receive((Protocol)UART, Header_Buffer[(Protocol)UART], HEADER_SIZE, 0);
	if (i2c_en)		hw_receive((Protocol)I2C, Header_Buffer[(Protocol)I2C], HEADER_SIZE, 0);
	if (spi_en)		hw_receive((Protocol)SPI, Header_Buffer[(Protocol)SPI], HEADER_SIZE, 0);
	if (RS485_en){
		/*Preserved for future use*/
	}
}

/**
  * @brief	Initialize all the protocols that STM32 has (UART, I2C, SPI, ...)
  * @param	Role: Specify the protocol role
  * @retval	NONE
  */
void hw_init(void) {
	uint8_t isSlaveCongig = 0;

	hw_tim_init();

	if (i2c_en) {
		i2c_init();
		if (Com.i2c_Handler.i2c_role == BKITCOM_SLAVE)			isSlaveCongig = 1;
	}

	if (spi_en) {
		spi_init();
		if (Com.spi_Handler.spi_role == BKITCOM_SLAVE)			isSlaveCongig = 1;
	}

	if (uart_en) {
		uart_init();
		if (Com.uart_Handler.uart_role == BKITCOM_SLAVE)		isSlaveCongig = 1;
	}

	if (RS485_en) {
		RS485_init();
		if (Com.RS485_Handler.RS485_role == BKITCOM_SLAVE)		isSlaveCongig = 1;
	}

	if (isSlaveCongig)		slave_init();

	timerInit(&hw_tim);
}

BKITCOM_Error_Code i2c_config (uint8_t i2c_x, uint16_t address, BKITCOM_Role_type_t role) {
	Com.i2c_Handler.i2c_role = role;

	switch (i2c_x) {
		case 1:
			Com.i2c_Handler.i2c.Instance = I2C1;
			break;
		case 2:
			Com.i2c_Handler.i2c.Instance = I2C2;
			break;
		default:
			return BKITCOM_INVALID;
	}
	Com.i2c_Handler.address = address;
	Com.i2c_Handler.i2c.Init.OwnAddress1 = address;
	i2c_en = 1;
	return BKITCOM_SUCCESS;
}

BKITCOM_Error_Code spi_config (uint8_t spi_x, uint16_t CPOL, uint16_t CPHA, BKITCOM_Role_type_t role) {
	Com.spi_Handler.spi_role = role;

	if (role == BKITCOM_MASTER)
		Com.spi_Handler.spi.Init.Mode = SPI_MODE_MASTER;
	else if (role == BKITCOM_SLAVE)
		Com.spi_Handler.spi.Init.Mode = SPI_MODE_SLAVE;

	switch (spi_x) {
		case 1:
			Com.spi_Handler.spi.Instance = SPI1;
			break;
		case 2:
			Com.spi_Handler.spi.Instance = SPI2;
			break;
		default:
			return BKITCOM_INVALID;
	}
	Com.spi_Handler.CPOL = CPOL;
	Com.spi_Handler.CPHA = CPHA;
	spi_en = 1;
	return BKITCOM_SUCCESS;
}

BKITCOM_Error_Code uart_config (uint8_t uart_x, uint32_t baudrate, BKITCOM_Role_type_t role) {
	Com.uart_Handler.uart_role = role;

	switch (uart_x) {
		case 1:
			Com.uart_Handler.uart.Instance = USART1;
			break;
		case 2:
			Com.uart_Handler.uart.Instance = USART2;
			break;
		case 3:
			Com.uart_Handler.uart.Instance = USART3;
			break;
		default:
			return BKITCOM_INVALID;
	}
	Com.uart_Handler.baudrate = baudrate;
	uart_en = 1;
	return BKITCOM_SUCCESS;
}


BKITCOM_Error_Code RS485_config (uint8_t uart_x, uint32_t baudrate, uint16_t address, BKITCOM_Role_type_t role) {
	Com.RS485_Handler.RS485_role = role;

	switch (uart_x) {
	case 1:
		Com.RS485_Handler.uart.Instance = USART1;
		break;
	case 2:
		Com.RS485_Handler.uart.Instance = USART2;
		break;
	case 3:
		Com.RS485_Handler.uart.Instance = USART3;
		break;
	default:
		return BKITCOM_INVALID;
	}
	Com.RS485_Handler.baudrate = baudrate;
	Com.RS485_Handler.address = address;
	RS485_en = 1;
	return BKITCOM_SUCCESS;
}


BKITCOM_Error_Code ErrorCode_Handler (HAL_StatusTypeDef Error_Code) {
	switch (Error_Code) {
	case HAL_OK:
		return BKITCOM_SUCCESS;
	case HAL_BUSY:
		return BKITCOM_BUSY;
	case HAL_TIMEOUT:
		return BKITCOM_TIMEOUT;
	case HAL_ERROR:
		switch (Com_Send_Signal) {
		case UART:
			Error_Code = HAL_UART_GetError(&Com.uart_Handler.uart);
			if (Error_Code == HAL_UART_ERROR_ORE) {
				hw_reboot(UART);
				return BKITCOM_OVERRUN;
			}
			break;
		case I2C:
			Error_Code = HAL_I2C_GetError(&Com.i2c_Handler.i2c);
			if (Error_Code == HAL_I2C_ERROR_ARLO) {
				hw_reboot(I2C);
				return BKITCOM_I2C_COLLISION;
			}
			break;
		case SPI:
			Error_Code = HAL_SPI_GetError(&Com.spi_Handler.spi);
			if (Error_Code == HAL_SPI_ERROR_MODF) {
				/*Incorrect SPI clock polarity/phase*/
				hw_reboot(SPI);
				return BKITCOM_INVALID_CLOCK_PHASE_POLARITY;
			}
			if (Error_Code == HAL_SPI_ERROR_OVR) {
				hw_reboot(SPI);
				return BKITCOM_OVERRUN;
			}
			if (Error_Code == HAL_SPI_ERROR_ABORT) {
				/*Abort arise when the SPI connection is lost, reconnect required*/
				return BKITCOM_ABORT;
			}
			break;
		case RS485:
			break;
		default:
			break;
		}
	}
}

BKITCOM_Error_Code hw_send(Protocol com, uint16_t address, uint8_t * data, uint32_t data_length) {
	HAL_StatusTypeDef Error_Code;
	BKITCOM_Error_Code code;

	/*Com_Send_Signal is set for FSM to capture the sending signal*/
	Com_Send_Signal = com;
	uint8_t retry = 0;

	/*Determined the protocol and send*/
	SEND_MESSAGE:
		switch (Com_Send_Signal) {
		case UART:
				if (uart_en == 0)
					goto PROTOCOL_NOT_INIT;
				Error_Code = HAL_UART_Transmit(&Com.uart_Handler.uart, data, data_length, 150);
				break;

		case I2C:
				if (i2c_en == 0)
					goto PROTOCOL_NOT_INIT;
				if (Com.i2c_Handler.i2c_role == BKITCOM_MASTER) {
					prev_i2c_slave_address = address;
					Error_Code = HAL_I2C_Master_Transmit(&Com.i2c_Handler.i2c, address << 1, data, data_length, 150);
				} else {
					Error_Code = HAL_I2C_Slave_Transmit(&Com.i2c_Handler.i2c, data, data_length, 2000);
				}
				break;

		case SPI:
				if (spi_en == 0)
					goto PROTOCOL_NOT_INIT;
				Error_Code = HAL_SPI_Transmit(&Com.spi_Handler.spi, data, data_length, 150);
				break;

		case RS485:
				if (RS485_en == 0)
					goto PROTOCOL_NOT_INIT;
				break;

		default:
			/*Dealing with uninitialized protocol calling*/
			PROTOCOL_NOT_INIT:
				Com_Send_Signal = NONE;
				return BKITCOM_PROTOCOL_NOT_INIT;
		}

		code = ErrorCode_Handler(Error_Code);

		if (code != BKITCOM_SUCCESS && retry < 3) {
			/*Auto retry*/
			retry++;
			goto SEND_MESSAGE;
		}

		Com_Send_Signal = NONE;
		HAL_Delay(10);
		return code;
}

BKITCOM_Error_Code hw_receive(Protocol com, uint8_t * data, uint8_t data_length, uint32_t timeout){
	HAL_StatusTypeDef Error_Code;
	if (data_length < 0)			return BKITCOM_INVALID;
	timeout = (timeout == 0)? RECEIVE_TIMEOUT : timeout;

	switch (com) {
		case UART:
			/*This is for ensure that the master only call with timeout and slave can do something else while waiting for data*/
			if (Com.uart_Handler.uart_role == BKITCOM_MASTER) {
				Error_Code = HAL_UART_Receive(&Com.uart_Handler.uart, data, data_length, timeout);
			}
			else	{
				count = 404;
				Error_Code = HAL_UART_Receive_IT(&Com.uart_Handler.uart, data, data_length);
			}
			break;

		case SPI:
			if (Com.spi_Handler.spi_role == BKITCOM_MASTER) {
				uint8_t * dummyData = malloc(sizeof(uint8_t) * data_length);
				Error_Code = HAL_SPI_TransmitReceive(&Com.spi_Handler.spi, dummyData, data, data_length, timeout);
				free(dummyData);
			}
			else 	Error_Code = HAL_SPI_Receive_IT(&Com.spi_Handler.spi, data, data_length);
			break;

		case I2C:
			if (Com.i2c_Handler.i2c_role == BKITCOM_MASTER) {
				uint8_t * dummyData = malloc(sizeof(uint8_t) * data_length);
				Error_Code = HAL_I2C_Master_Receive(&Com.i2c_Handler.i2c, prev_i2c_slave_address, dummyData, data_length, 1000);
				free(dummyData);
			} else 	Error_Code = HAL_I2C_Slave_Receive_IT(&Com.i2c_Handler.i2c, data, data_length);
			break;

		case RS485:
			break;

		default:
			break;
	}
	return ErrorCode_Handler(Error_Code);
}

void Hardware_Interface_Layer_FSM (void) {
	switch (state) {
	case IDLE:
		/*If the header is received*/
		if (Com_Receive_Signal != NONE){
			Com_Receive_Signal = NONE;
			/*The payload length is store in byte idx 1 and 2*/
			Protocol_Buffer_Malloc(Com_Receive_Signal, (Header_Buffer[Com_Receive_Signal][1] << 8) | Header_Buffer[Com_Receive_Signal][0]);
			hw_receive(Com_Receive_Signal, Payload_Buffer[Com_Receive_Signal], (Header_Buffer[Com_Receive_Signal][1] << 8) | Header_Buffer[Com_Receive_Signal][0], 0);
			state = RECEIVING;
		}
		break;
	case RECEIVING:
		/*If the payload is received*/
		if (Com_Receive_Signal != NONE) {
			/*Wait for header*/
			switch (Com_Receive_Signal) {
				case UART:
					if (Com.uart_Handler.uart_role == BKITCOM_SLAVE)		hw_receive(Com_Receive_Signal, Header_Buffer[Com_Receive_Signal], HEADER_SIZE, 0);
					break;
				case I2C:
					if (Com.i2c_Handler.i2c_role == BKITCOM_SLAVE)			hw_receive(Com_Receive_Signal, Header_Buffer[Com_Receive_Signal], HEADER_SIZE, 0);
					break;
				case SPI:
					if (Com.spi_Handler.spi_role == BKITCOM_SLAVE)			hw_receive(Com_Receive_Signal, Header_Buffer[Com_Receive_Signal], HEADER_SIZE, 0);
					break;
				case RS485:
					break;
			}
			Com_Receive_Signal = NONE;
			Received_Flag = 1;
			state = IDLE;
		}
		break;
	case ERROR_STATE:
		break;
	}
}

/*Callback function definition*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	count++;
	Com_Receive_Signal = UART;
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	Com_Receive_Signal = I2C;
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
	Com_Receive_Signal = SPI;
}

void Protocol_Buffer_Malloc (Protocol protocol, uint32_t length) {
	if (Payload_Buffer[protocol] != NULL)		free(Payload_Buffer[protocol]);
	Payload_Buffer[protocol] = malloc (sizeof(uint8_t) * length);
}
