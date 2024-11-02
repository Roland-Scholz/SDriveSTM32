/*
 * DualSD.cpp
 *
 *  Created on: 1 Nov 2016
 *      Author: ralim
 */

#include <SDCard.h>
#include <uprintf.h>

SPI_HandleTypeDef* _spi;
GPIO_TypeDef *_cs1Port;
uint16_t _cs1Pin;
uint32_t _sdSize;

int initDone = false;

void SDCard(SPI_HandleTypeDef* hspi, uint16_t cs1Pin, GPIO_TypeDef* cs1Port) {
	_spi = hspi;
	_cs1Pin = cs1Pin;
	_cs1Port = cs1Port;
	_sdSize = 0;
}

int getSize(uint32_t* size, uint16_t* blklen) {
	uint8_t res;

	if (_sdSize == 0) {
		//We have not read the disk size just yet :O
		union csd_t csd;
		res = readCSD(&csd);
		if (DEBUG)
			uprintf("CMD9 getSize res: %d\n", res);

		if (res == false) {
			if (DEBUG)
				uprintf("CSD read failed\n");
			return false;
		}
		if (DEBUG)

			uprintf("CSD read OK\n");

		uint8_t* prt = (uint8_t*) &csd;
		for (int i = 0; i < 16; i++) {
			uprintf("%x ", prt[i]);
		}
		uprintf("\n");

		if (csd.v1.csd_ver == 0) {
			//uprintf("csd version 0 \n");

			uint32_t c_size = (csd.v1.c_size_hi << 10)
					+ (csd.v1.c_size_med << 2) + csd.v1.c_size_lo;
			uint32_t c_size_mult =
					1
							<< ((csd.v1.c_size_mult_hi << 1)
									+ csd.v1.c_size_mult_lo + 2);
			uint32_t factor = (1 << csd.v1.read_bl_len);

			uprintf("read_bl_len: %d\n", factor);
			uprintf("c_size: %d\n", c_size);
			uprintf("c_size_mult: %d\n", c_size_mult);

			factor /= SD_BLOCK_SIZE;

			_sdSize = (c_size + 1) * c_size_mult * factor;
			uprintf("csd size %d \n", _sdSize);

		} else if (csd.v2.csd_ver == 1) {
			uprintf("csd version 1 \n");
			return false;
			uint32_t c_size = ((uint32_t) csd.v2.c_size_high << 16)
					| (csd.v2.c_size_mid << 8) | csd.v2.c_size_low;
			c_size *= 1024;
			_sdSize = c_size;

		} else {
			uprintf("csd version unknown \n");
			return false;
		}
	}

	*blklen = SD_BLOCK_SIZE;
	*size = _sdSize;

	if (DEBUG)
		uprintf("csd size %d \n", *size, *blklen);

	return true;
}
/** read CID or CSR register */
uint8_t readRegister(uint8_t cmd, void* buf) {
	uint8_t* dst = (uint8_t*) (buf);
	uint8_t res;
	if ((res = cardCommand(cmd, 0)) != 0) {
		deselectCard();
		return false;
	}
	uint8_t temp = 0xFF;
	while (temp == 0xFF) {
		SPI_Recieve(&temp, 1);
	}
	// transfer data
	SPI_Recieve(dst, 16);
	SPI_Recieve(&temp, 1); //CRC1
	SPI_Recieve(&temp, 1); //CRC2
	deselectCard();
	return true;
}

int readBlock(uint32_t blockaddr, uint8_t* buffer) {
	uint8_t res;
	uint8_t temp = 0xFF;

	if (DEBUG)
		uprintf("readBlock block: %d, buffer: %x \n", blockaddr, buffer);

	if ((res = cardCommand(CMD17, blockaddr)) != 0) {
		/*
		 * Error
		 */

		uprintf("CMD17 read failed %x \n", res);
		deselectCard();
		return false;
	}

	while (temp == 0xFF) {
		HAL_SPI_Receive(_spi, &temp, 1, 100);
	}

	SPI_Recieve(buffer, SD_BLOCK_SIZE);
//eat the CRC
	temp = 0xFF;
	SPI_Recieve(&temp, 1);
	temp = 0xFF;
	SPI_Recieve(&temp, 1);
	deselectCard();
	return true;
}

int writeBlock(uint32_t blockaddr, uint8_t* buffer) {
//The cardCommand will select the card so we have to make sure we clean up

	if (cardCommand(CMD24, blockaddr)) {
		/*
		 * Error
		 */
		deselectCard();
		return false;
	}
	/*
	 * Write the data
	 */
	uint8_t temp = DATA_START_BLOCK;
	HAL_SPI_Transmit(_spi, &temp, 1, 100);
	HAL_SPI_Transmit(_spi, buffer, SD_BLOCK_SIZE, 100);
	temp = 0xFF;
	HAL_SPI_Transmit(_spi, &temp, 1, 100); //2 bytes CRC)
	HAL_SPI_Transmit(_spi, &temp, 1, 100);
//read response
	SPI_Recieve(&temp, 1);		       
	if ((temp & DATA_RES_MASK) != DATA_RES_ACCEPTED) {
		/*
		 * Error
		 */
		deselectCard();
		return false;
	}
// wait for flash programming to complete
	waitUntilReady();

// response is r2 so get and check two bytes for nonzero
	if (cardCommand(CMD13, 0)) {
		/*
		 * Error
		 */
		deselectCard();
		return false;
	}
	SPI_Recieve(&temp, 1);
	if (temp) {
		/*
		 * Error
		 */
		deselectCard();
		return false;
	}
	deselectCard();
	return true;
}

void waitUntilReady() {
	uint8_t ans[1] = { 0 };
	while (ans[0] != 0xFF) {
		SPI_Recieve(ans, 1);
	}
}

int initalize() {

	initialize_start:

	uprintf("SPI_HandleTypeDef* %x CS_PIN %d Port %x \n",
			_spi, _cs1Pin, _cs1Port);
	//HAL_Delay(500);
	uprintf("Init after delay 500 \n");
	_spi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256; //slow down at first
	HAL_SPI_Init(_spi); //apply the speed change
	deselectCard();
	uprintf("Init after deselect \n");
//We must supply at least 74 clocks with CS high
	uint8_t buffer[4] = { 0xFF, 0xFF, 0xFF, 0xFF };
	HAL_SPI_Transmit(_spi, buffer, 4, 100);
	HAL_SPI_Transmit(_spi, buffer, 4, 100);
	HAL_SPI_Transmit(_spi, buffer, 4, 100);
	uprintf("Init after SPI_Transmit \n");
//	HAL_Delay(5);
	uprintf("Init after Delay 5 \n");
	selectCard();
	uint8_t status;
// command to go idle in SPI mode
	uprintf("Init before while CMD0 \n");

	int cnt = 0;
	while ((status = cardCommand(CMD0, 0)) != R1_IDLE_STATE) {
		cnt++;
		if (cnt > 100)
			break;
	}

	if (cnt > 100) {
		uprintf("Init failed! \n");
		goto initialize_start;
	}

	uprintf("CMD8 start \n");
	uint8_t res;
// check SD version
	if (((res = cardCommand(CMD8, 0x1AA)) & R1_ILLEGAL_COMMAND)) {
		deselectCard();
		uprintf("CMD8 false res: %x \n", res);
//		return false; //Unsupported
	} else {
		// only need last byte of r7 response
		HAL_SPI_Receive(_spi, buffer, 4, 100);
		if (buffer[3] != 0XAA) {
			uprintf("CMD8 false2 \n");
			return false; //failed check
		}

	}
	uprintf("CMD8 end \n");

// initialize card and send host supports SDHC
	while ((status = cardAcmd(ACMD41, 0x40000000)) != R1_READY_STATE) {
		//uprintf("%02X ", status);
	}
// if SD2 read OCR register to check for SDHC card
	uprintf("CMD58 start \n");
	if (cardCommand(CMD58, 0)) {
		deselectCard();
		uprintf("CMD58 false \n");
		return false;
	}
	uprintf("CMD58 OK \n");
//discard OCR reg

	SPI_Recieve(buffer, 4);
	deselectCard();
	_spi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2; //speed back up
	HAL_SPI_Init(_spi); //apply the speed change

	return true;
}

uint8_t cardCommand(uint8_t command, uint32_t arg) {
	uint8_t res = 0xFF;
	uint8_t cnt = 0;

	selectCard();

	waitUntilReady(); //wait for card to no longer be busy

	uint8_t commandSequence[] = { (uint8_t) (command | 0x40), (uint8_t) (arg
			>> 24), (uint8_t) (arg >> 16), (uint8_t) (arg >> 8), (uint8_t) (arg
			& 0xFF), 0xFF };

	if (command == CMD0)
		commandSequence[5] = 0x95;
	else if (command == CMD8)
		commandSequence[5] = 0x87;

	if (DEBUG)
		uprintf("block: %x %x %x %x\n", commandSequence[1], commandSequence[2],
				commandSequence[3], commandSequence[4]);
	HAL_SPI_Transmit(_spi, commandSequence, 6, 100);

	//Data sent, now await Response
	while (res & 0x80) {
		SPI_Recieve(&res, 1);
		cnt++;
		if (cnt > 200) break;
		//uprintf("res: %d ", res);
	}

    if (DEBUG)
    	uprintf("CMD: %x res: %x\n", command, res);

    return res;
}

uint8_t cardAcmd(uint8_t cmd, uint32_t arg) {
	cardCommand(CMD55, 0);
	return cardCommand(cmd, arg);
}

void selectCard() {
	HAL_GPIO_WritePin(_cs1Port, _cs1Pin, GPIO_PIN_RESET);
}

uint8_t readCSD(union csd_t* csd) {
	return readRegister(CMD9, csd);
}

HAL_StatusTypeDef SPI_Recieve(uint8_t* pData, uint16_t Size) {

	HAL_StatusTypeDef errorcode = HAL_OK;

	/* Process Locked */
	__HAL_LOCK(_spi);

	/* Don't overwrite in case of HAL_SPI_STATE_BUSY_RX */
	if (_spi->State == HAL_SPI_STATE_READY) {
		_spi->State = HAL_SPI_STATE_BUSY_TX_RX;
	}

	/* Set the transaction information */
	_spi->ErrorCode = HAL_SPI_ERROR_NONE;
	_spi->pRxBuffPtr = (uint8_t *) pData;
	_spi->RxXferCount = Size;
	_spi->RxXferSize = Size;
	_spi->pTxBuffPtr = (uint8_t *) pData;
	_spi->TxXferCount = Size;
	_spi->TxXferSize = Size;

	/*Init field not used in handle to zero */
	_spi->RxISR = NULL;
	_spi->TxISR = NULL;
	/* Check if the SPI is already enabled */
	if ((_spi->Instance->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE) {
		/* Enable SPI peripheral */
		__HAL_SPI_ENABLE(_spi);
	}
	/* Transmit and Receive data in 8 Bit mode */
	while ((_spi->RxXferCount > 0U)) {
		*(__IO uint8_t *) &_spi->Instance->DR = 0xFF; //send data
		while (!(__HAL_SPI_GET_FLAG(_spi, SPI_FLAG_TXE)))
			;
		while (!(__HAL_SPI_GET_FLAG(_spi, SPI_FLAG_RXNE)))
			;
		(*(uint8_t *) pData++) = _spi->Instance->DR;
		_spi->RxXferCount--;
	}

	if (lSPI_WaitFlagStateUntilTimeout(_spi, SPI_FLAG_BSY, RESET, 100,
			HAL_GetTick()) != HAL_OK) {
		_spi->ErrorCode |= HAL_SPI_ERROR_FLAG;

		errorcode = HAL_TIMEOUT;
	}

	_spi->State = HAL_SPI_STATE_READY;
	__HAL_UNLOCK(_spi);
	return errorcode;
}

/**
 * @brief Handle SPI Communication Timeout.
 * @param hspi: pointer to a SPI_HandleTypeDef structure that contains
 *              the configuration information for SPI module.
 * @param Flag: SPI flag to check
 * @param State: flag state to check
 * @param Timeout: Timeout duration
 * @param Tickstart: tick start value
 * @retval HAL status
 */
HAL_StatusTypeDef lSPI_WaitFlagStateUntilTimeout(SPI_HandleTypeDef *hspi,
		uint32_t Flag, uint32_t State, uint32_t Timeout, uint32_t Tickstart) {
	while ((hspi->Instance->SR & Flag) != State) {
		if (Timeout != HAL_MAX_DELAY) {
			if ((Timeout == 0U) || ((HAL_GetTick() - Tickstart) >= Timeout)) {
				/* Disable the SPI and reset the CRC: the CRC value should be cleared
				 on both master and slave sides in order to resynchronize the master
				 and slave for their respective CRC calculation */

				/* Disable TXE, RXNE and ERR interrupts for the interrupt process */
				__HAL_SPI_DISABLE_IT(hspi,
						(SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));

				if ((hspi->Init.Mode == SPI_MODE_MASTER)
						&& ((hspi->Init.Direction == SPI_DIRECTION_1LINE)
								|| (hspi->Init.Direction
										== SPI_DIRECTION_2LINES_RXONLY))) {
					/* Disable SPI peripheral */
					__HAL_SPI_DISABLE(hspi);
				}

				/* Reset CRC Calculation */
				if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE) {
					SPI_RESET_CRC(hspi);
				}

				hspi->State = HAL_SPI_STATE_READY;

				/* Process Unlocked */
				__HAL_UNLOCK(hspi);

				return HAL_TIMEOUT;
			}
		}
	}

	return HAL_OK;
}
void deselectCard() {
	HAL_GPIO_WritePin(_cs1Port, _cs1Pin, GPIO_PIN_SET);
}
