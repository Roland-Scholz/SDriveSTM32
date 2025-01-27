/*
 * DualSD.h
 *
 *  Created on: 1 Nov 2016
 *      Author: ralim
 */

#ifndef SDCARD_H_
#define SDCARD_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include "stm32f103xb.h"

#define false 0
#define true 1
#define SD_BLOCK_SIZE 512

#define DEBUG false

// Based on the document:
//
// SD Specifications
// Part 1
// Physical Layer
// Simplified Specification
// Version 2.00
// September 25, 2006
//
// www.sdcard.org/developers/tech/sdcard/pls/Simplified_Physical_Layer_Spec.pdf
//------------------------------------------------------------------------------
// SD card commands
/** GO_IDLE_STATE - init card in spi mode if CS low */
#define CMD0	0x00
/** SEND_IF_COND - verify SD Memory Card interface operating condition.*/
#define CMD8  0X08
/** SEND_CSD - read the Card Specific Data (CSD register) */
#define CMD9  0X09
/** SEND_CID - read the card identification information (CID register) */
#define CMD10  0X0A
 /** END_READ - end read data */
 #define CMD12  0X0C
/** SEND_STATUS - read the card status register */
#define CMD13  0X0D
/** READ_BLOCK - read a single data block from the card */
#define CMD17  0X11
 /** READ_BLOCK_MULTIPLE - read multiple block from the card */
 #define CMD18  0X12
/** WRITE_BLOCK - write a single data block to the card */
 #define CMD24  0X18
/** WRITE_MULTIPLE_BLOCK - write blocks of data until a STOP_TRANSMISSION */
#define CMD25  0X19;
/** ERASE_WR_BLK_START - sets the address of the first block to be erased */
#define CMD32  0X20
/** ERASE_WR_BLK_END - sets the address of the last block of the continuous
 range to be erased*/
#define CMD33  0X21
/** ERASE - erase all previously selected blocks */
#define CMD38  0X26
/** APP_CMD - escape for application specific command */
#define CMD55  0X37
/** READ_OCR - read the OCR register of a card */
#define CMD58  0X3A
/** SET_WR_BLK_ERASE_COUNT - Set the number of write blocks to be
 pre-erased before writing */
#define ACMD23  0X17
/** SD_SEND_OP_COMD - Sends host capacity support information and
 activates the card's initialization process */
#define ACMD41  0X29
//------------------------------------------------------------------------------
/** status for card in the ready state */
#define R1_READY_STATE  0X00
/** status for card in the idle state */
#define R1_IDLE_STATE  0X01
/** status bit for illegal command */
#define R1_ILLEGAL_COMMAND  0X04
/** start data token for read or write single block*/
#define DATA_START_BLOCK  0XFE
/** stop token for write multiple blocks*/
#define STOP_TRAN_TOKEN  0XFD
/** start data token for write multiple blocks*/
#define WRITE_MULTIPLE_TOKEN  0XFC
/** mask for data response tokens after a write block operation */
#define DATA_RES_MASK  0X1F
/** write data accepted token */
#define DATA_RES_ACCEPTED  0X05
//------------------------------------------------------------------------------
typedef struct CID {
	// byte 0
	uint8_t mid;  // Manufacturer ID
	// byte 1-2
	char oid[2];  // OEM/Application ID
	// byte 3-7
	char pnm[5];  // Product name
	// byte 8
	unsigned prv_m :4;  // Product revision n.m
	unsigned prv_n :4;
	// byte 9-12
	uint32_t psn;  // Product serial number
	// byte 13
	unsigned mdt_year_high :4;  // Manufacturing date
	unsigned reserved :4;
	// byte 14
	unsigned mdt_month :4;
	unsigned mdt_year_low :4;
	// byte 15
	unsigned always1 :1;
	unsigned crc :7;
} cid_t;
//------------------------------------------------------------------------------
// CSD for version 1.00 cards
typedef struct CSDV1 {
	// byte 0
	unsigned reserved1 :6;
	unsigned csd_ver :2;

	uint8_t taac;
	uint8_t nsac;
	uint8_t tran_speed;

	// byte 4
	uint8_t ccc_hi :8;

	// byte 5
	unsigned read_bl_len :4;
	unsigned ccc_lo :4;

	// byte 6
	unsigned c_size_hi :2;
	unsigned dsr_imp :1;
	unsigned read_blk_misalign :1;
	unsigned write_blk_misalign :1;
	unsigned read_bl_partial :1;
	unsigned reserved2 :2;

	// byte 7
	uint8_t c_size_med;

	// byte 8
	unsigned vdd_r_curr_max :3;
	unsigned vdd_r_curr_min :3;
	unsigned c_size_lo :2;

	// byte 9
	unsigned c_size_mult_hi :2;
	unsigned vdd_w_cur_max :3;
	unsigned vdd_w_curr_min :3;

	// byte 10
	unsigned sector_size_hi :6;
	unsigned erase_blk_en :1;
	unsigned c_size_mult_lo :1;

	// byte 11
	unsigned wp_grp_size :7;
	unsigned sector_size_lo :1;

	// byte 12
	unsigned wp_grp_enable :1;
	unsigned reserved3 :2;
	unsigned r2w_factor :3;
	unsigned write_bl_len_hi :2;

	// byte 13
	unsigned reserved4 :5;
	unsigned write_partial :1;
	unsigned write_bl_len_lo :2;

	// byte 14
	unsigned reserved5 :2;
	unsigned file_format :2;
	unsigned tmp_write_protect :1;
	unsigned perm_write_protect :1;
	unsigned copy :1;
	unsigned file_format_grp :1;

	// byte 15
	unsigned always1 :1;
	unsigned crc :7;
} csd1_t;
//------------------------------------------------------------------------------
// CSD for version 2.00 cards
typedef struct CSDV2 {
	// byte 0
	unsigned reserved1 :6;
	unsigned csd_ver :2;
	// byte 1
	uint8_t taac;
	// byte 2
	uint8_t nsac;
	// byte 3
	uint8_t tran_speed;
	// byte 4
	uint8_t ccc_high;
	// byte 5
	unsigned read_bl_len :4;
	unsigned ccc_low :4;
	// byte 6
	unsigned reserved2 :4;
	unsigned dsr_imp :1;
	unsigned read_blk_misalign :1;
	unsigned write_blk_misalign :1;
	unsigned read_bl_partial :1;
	// byte 7
	unsigned reserved3 :2;
	unsigned c_size_high :6;
	// byte 8
	uint8_t c_size_mid;
	// byte 9
	uint8_t c_size_low;
	// byte 10
	unsigned sector_size_high :6;
	unsigned erase_blk_en :1;
	unsigned reserved4 :1;
	// byte 11
	unsigned wp_grp_size :7;
	unsigned sector_size_low :1;
	// byte 12
	unsigned write_bl_len_high :2;
	unsigned r2w_factor :3;
	unsigned reserved5 :2;
	unsigned wp_grp_enable :1;
	// byte 13
	unsigned reserved6 :5;
	unsigned write_partial :1;
	unsigned write_bl_len_low :2;
	// byte 14
	unsigned reserved7 :2;
	unsigned file_format :2;
	unsigned tmp_write_protect :1;
	unsigned perm_write_protect :1;
	unsigned copy :1;
	unsigned file_format_grp :1;
	// byte 15
	unsigned always1 :1;
	unsigned crc :7;
} csd2_t;
//------------------------------------------------------------------------------
// union of old and new style CSD register
union csd_t {
	csd1_t v1;
	csd2_t v2;
};

	void SDCard(SPI_HandleTypeDef* hspi, uint16_t cs1Pin, GPIO_TypeDef* cs1Port);

	int initalize(); //startup the cards
	int getSize(uint32_t* size, uint16_t* blklen); //get the total storage space size
	int readBlock(uint32_t blockaddr, uint8_t* buffer); //reads a single 512 byte block
	int writeBlock(uint32_t blockaddr, uint8_t* buffer); //writes a single 512 byte block

	void waitUntilReady(); //waits until the card is ready
	uint8_t cardCommand(uint8_t command, uint32_t arg);
	uint8_t cardAcmd(uint8_t cmd, uint32_t arg);

	HAL_StatusTypeDef SPI_Recieve(uint8_t *pData, uint16_t Size);
	HAL_StatusTypeDef lSPI_WaitFlagStateUntilTimeout(SPI_HandleTypeDef *hspi,
			uint32_t Flag, uint32_t State, uint32_t Timeout,
			uint32_t Tickstart);
	uint8_t readRegister(uint8_t cmd, void* buf);

	/**
	 * Read a cards CSD register. The CSD contains Card-Specific Data that
	 * provides information regarding access to the card's contents. */
	uint8_t readCSD(union csd_t* csd);
	void selectCard();
	void deselectCard();

	//extern SPI_HandleTypeDef* _spi;
	//extern GPIO_TypeDef *_cs1Port;
	//extern uint16_t _cs1Pin;
	//extern uint64_t _sdSize;

#ifdef __cplusplus
}
#endif


#endif /* SDCARD_H_ */
