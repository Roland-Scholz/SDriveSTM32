#include <stdlib.h>
#include "fatfs.h"
#include "stm32f1xx_hal.h"
#include "uprintf.h"
#include <string.h>
#include <math.h>
#include "sboot.h"
#include "SDCard.h"

//#include "usb_device.h"
//#include "usbd_core.h"
//#include "usbd_desc.h"
//#include "usbd_msc.h"
//#include "usbd_storage_if.h"

extern UART_HandleTypeDef huart2;

#define TRUE 1
#define FALSE 0
#define SIO_DEBUG 0

#define CLMT_SIZE 256

#define HIPOKEY 7

/*
 * Data structures
 */

typedef uint8_t byte;

typedef struct {
	byte actual_drive_number;
	byte fastsio_active;
	byte fastsio_pokeydiv;
	byte bootloader_relocation;
	byte p4, p5, p6, p7;
} SDriveParameters;

struct atrHeader {
	byte h0; /* 0x96 */
	byte h1; /* 0x02 */
	byte filesizelo;
	byte filesizemed;
	byte secsizelo;
	byte secsizehi;
	byte filesizehi;
	byte unused[9];
} atrHeader;

typedef struct {
	DWORD seek;
	uint16_t size;
} atariSeek_t;

typedef struct {
	byte dunit;
	byte dcomnd;
	byte daux1;
	byte daux2;
	byte dcheck;
} cmdFrame_t;

enum seekcodes {
	xfd, /* This is a xfd (raw sd) image */
	atr, /* This is a regular ATR image */
	atrdd3, /* This is a dd ATR image, including the first 3 sectors */
	xex
};

/*
 * Prototypes
 */
extern UART_HandleTypeDef huart3;

typedef UART_HandleTypeDef* HANDLE;

static void Sleep(int i);

static uint16_t comwrite(HANDLE hComm, byte *c, uint16_t i);
static uint16_t comread(HANDLE hComm, byte *c, uint16_t i);
static int ring();

static void err(const char *s);
static void ack();
static void ack1();
static void nack();
static void atariSeek(int disk, int sec, atariSeek_t *aSeek);
static void senddata(int disk, int sec);
static byte checkSum(byte * buf, int size);
static void sendrawdata(byte *buf, uint16_t size);
static int recvdata(int disk, int sec);
static HANDLE get_atari(byte speed);
void getcmd(cmdFrame_t *buf);
static FRESULT loaddisk(char *path, int disk);
static void decode(cmdFrame_t *buf);
static uint32_t comget(byte *);
static void toggleBaud(void);
//static void closedisks();
static void closedisk(int disk);
static void filename2atari(byte *buf, char *filename, int index);

/*
 * Macros
 */

#define ATRHEAD		16
#define MAXDISKS	5
#define BOOTDISK	0

/*
 * Global variables
 */
static uint16_t secsize[MAXDISKS];
static uint16_t seccount[MAXDISKS];
static uint8_t wrongatr[MAXDISKS];

static enum seekcodes seekcode[MAXDISKS];
static FIL *diskfd[MAXDISKS];
static int ro[MAXDISKS];
static SDriveParameters sParms;
static char imagename[MAXDISKS][13];

static byte status[] = { 0x00, 0xFF, 0x80, 0 };
static byte version[] = "SDrive01";
static char path[128];
static char filename[13];
static cmdFrame_t cmdbuf;

static uint32_t cnt;

static HANDLE atari = NULL; /* fd of the serial port hooked up to the SIO2PC cable */
static byte buf[512];
static int synced = -1;
static int lastIndex = 0;
static byte* ptr;
static DWORD* dPtr;
static WORD* wPtr;
static byte boot = TRUE;
static uint32_t sdSector;
static byte directAccess = FALSE;
static byte bAck = 'A';
static byte bComp = 'C';
static byte bNack = 'N';
static byte flag = 255;
//static char bootfile[] = "FAT16.ATR";
static char bootfile[] = "SDRIVE.ATR";
//static char bootfile[] = "SERPAR~1.ATR";

// 8 : 57600
//16 : 38400
//40 : 19200

static byte hispeed = HIPOKEY;
static byte isHiSpeed = FALSE;

/*
 * this definition of 'sleep some microseconds' does not
 * conform to the SIO-protocol, but works
 *
 */

static void Sleep(int i) {
	HAL_Delay(i);
}
/**
 * @brief Initializes DWT_Clock_Cycle_Count for DWT_Delay_us function
 * @return Error DWT counter
 * 1: clock cycle counter not started
 * 0: clock cycle counter works
 */
uint32_t DWT_Delay_Init(void) {
	/* Disable TRC */
	CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk; // ~0x01000000;
	/* Enable TRC */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // 0x01000000;
	/* Disable clock cycle counter */
	DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; //~0x00000001;
	/* Enable clock cycle counter */
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; //0x00000001;
	/* Reset the clock cycle counter value */
	DWT->CYCCNT = 0;
	/* 3 NO OPERATION instructions */
	__ASM volatile ("NOP");
	__ASM volatile ("NOP");
	__ASM volatile ("NOP");
	/* Check if clock cycle counter has started */
	if (DWT->CYCCNT) {
		return 0; /*clock cycle counter started*/
	} else {
		return 1; /*clock cycle counter not started*/
	}
}

__STATIC_INLINE void usleep(volatile uint32_t microseconds) {

	uint32_t clk_cycle_start = DWT->CYCCNT;
	/* Go to number of cycles for system */

	microseconds *= (HAL_RCC_GetHCLKFreq() / 1000000);
	/* Delay till end */

	while ((DWT->CYCCNT - clk_cycle_start) < microseconds-15);      // change value of 15 to adjust timing (microseconds-x)
}

/*
 * main()
 *
 * Read the command line, open the disk images, connect to the Atari,
 * and listen for commands.
 *
 * This never terminates.
 */

int sioMain() {
	int i;

	uprintf("DWT started (0 = OK, 1 = NotOK): %d\n", DWT_Delay_Init());

	atari = get_atari(0);

	for (i = 0; i < MAXDISKS; i++) {
		diskfd[i] = NULL;
		ro[i] = 0;
		imagename[i][0] = 0;
	}

	strcpy(imagename[0], bootfile);
	loaddisk(imagename[0], 0);
	boot = TRUE;

	/*
	 * Main control loop
	 *
	 * Read a command and deal with it
	 * The command frame is 5 bytes.
	 */
	while (TRUE) {

		getcmd(&cmdbuf);
		decode(&cmdbuf);
	}

	return TRUE;
}

static void err(const char *s) {
	uprintf("%s", s);
}

static void ack1() {
	usleep(1000);
	ack();
}

static void ack() {
	if (comwrite(atari, &bAck, 1) != HAL_OK)
		err("ack failed\n");
}

static void complete() {
	usleep(400);
	if (comwrite(atari, &bComp, 1) != HAL_OK)
		err("ack failed\n");
	//usleep(COMPLETE2);
}

static void nack() {
	usleep(800);
	if (comwrite(atari, &bNack, 1) != HAL_OK)
		err("nack failed\n");
	//uprintf("NACK sent!\n");
}

static void atariSeek(int disk, int sec, atariSeek_t *aSeek) {

	aSeek->seek = (sec - 1) * secsize[disk];
	aSeek->size = secsize[disk];

	if (secsize[disk] != 128) {
		if (sec <= 3) {
			aSeek->size = 128;
			aSeek->seek = (sec - 1) * 128;
		} else {
			if (wrongatr[disk] == FALSE) {
				aSeek->seek -= 384;
			}
		}
	}

	if (seekcode[disk] == atr)
		aSeek->seek += ATRHEAD;

	//uprintf("sector: %d, size: %d, seek: %d\n", sec, aSeek->size, aSeek->seek);
}

static void setBaud(byte speed) {

// PAL : 1.7734470 MHz
// NTSC: 1.7897725 MHz

	if (speed == 0) {
		huart3.Init.BaudRate = 124000;
	} else {
		huart3.Init.BaudRate = (1773447) / ((speed + 7) << 1);
	}

	HAL_UART_Init(&huart3);

//uprintf("set baud rate: %d \n", huart3.Init.BaudRate);
}

static void senddata(int disk, int sec) {
	atariSeek_t aSeek;
	UINT flen;
	FRESULT rc;
	UINT bytes_read;

	if (seekcode[disk] != xex) {
		atariSeek(disk, sec, &aSeek);

		if ((rc = f_lseek(diskfd[disk], aSeek.seek)) != FR_OK) {
			uprintf("Seek failed %X, %d %d\n", diskfd[disk], aSeek.seek, rc);
		}

		if ((rc = f_read(diskfd[disk], buf, aSeek.size, &flen)) != FR_OK) {
			uprintf("Read failed %X, %d %d\n", diskfd[disk], aSeek.size, rc);
		}

		/*
		 for (int i = 0; i < aSeek.size; i++) {
		 if (i % 16 == 0) {
		 uprintf("\n");
		 }
		 uprintf("%02X ", buf[i]);
		 }
		 */
		//uprintf("read seek: %d size: %d \n", aSeek.seek, aSeek.size);
		goto senddata_buf;
	}

// XEX
	aSeek.size = 128;

	if (sec <= 2) {
		sendrawdata(boot_xex_loader + (sec - 1) * 128, 128);
		return;
	}

	if (sec == 0x168) {
		buf[0] = 'S';
		buf[1] = seccount[disk] & 0xff;
		buf[2] = seccount[disk] >> 8;
		buf[3] = buf[4] = 0;
		goto senddata_buf;
	}

	if (sec == 0x169) {
		memset(buf, 0, 128);
		buf[0] = 0x42;
		buf[1] = seccount[disk] & 0xff;
		buf[2] = seccount[disk] >> 8;
		buf[3] = 0x71;
		buf[4] = 0x01;

		memcpy(buf + 5, "DUMMY   XEX", 11);
		goto senddata_buf;
	}

	if (sec >= 0x171) {

		bytes_read = (sec - 0x171) * 125;

		if ((rc = f_lseek(diskfd[disk], bytes_read)) != FR_OK) {
			uprintf("Seek failed %X, %d %d\n", diskfd[disk], bytes_read, rc);
		}
		if ((rc = f_read(diskfd[disk], buf, 125, &flen)) != FR_OK) {
			uprintf("Read failed %X, %d %d\n", diskfd[disk], 125, rc);
		}

		sec++;
		if (flen < 125) {
			sec = 0;
		}

		buf[125] = sec >> 8;
		buf[126] = sec & 0xff;
		buf[127] = flen;

	}

	senddata_buf: sendrawdata(buf, aSeek.size);
	return;
}

static byte checkSum(byte * buf, int size) {
	unsigned short i, sum;

	for (i = sum = 0; i < size; i++) {
		sum += buf[i];
		if (sum > 255)
			sum -= 255;
	}

	return (byte) sum;

}

static void sendrawdata(byte *buf, uint16_t size) {

	byte bsum = checkSum(buf, size);

	if (comwrite(atari, buf, size) != HAL_OK) {
		err("sendrawdata failed\n");
	}

	if (comwrite(atari, &bsum, 1) != HAL_OK) {
		err("sendrawdata failed\n");
	}

	/*
	 for (i = 0; i < size; i++) {
	 if (i % 16 == 0) printf("\n");
	 printf("%02X ", buf[i]);
	 }
	 printf("%02X \n", bsum);
	 printf("sent size: %d \n", size);
	 */

}

static int recvdata(int disk, int sec) {

	atariSeek_t aSeek;
	byte cSum;
	UINT flen;

	atariSeek(disk, sec, &aSeek);

	comread(atari, buf, aSeek.size + 1);

	cSum = checkSum(buf, aSeek.size);

	if (buf[aSeek.size] != cSum) {
		uprintf("[BAD SUM]");
		return 0;
	} else {
		f_lseek(diskfd[disk], aSeek.seek);
		f_write(diskfd[disk], buf, aSeek.size, &flen);
		synced = disk;
		//uprintf("sync needed %d\n", synced);
		return 1;
		//f_sync(diskfd[disk]);
	}
}

/*
 * get_atari()
 *
 * Open the serial device and return the file descriptor.
 * It assumes that it is /dev/ttyS0 unless there's a symlink
 * from /dev/mouse to that, in which case /dev/ttyS1 is used.
 */
static HANDLE get_atari(byte speed) {

	return (&huart3);
}

/*
 * getcmd()
 *
 * Read one 5-byte command
 *
 * The Atari will activate the command line while sending
 * the 5-byte command, which is detected as the ring indicator
 * by the iscmd() function.
 * What we do is read on byte, and if the command line is active
 * immediately after reading the byte, we assume that that byte
 * was the first of a 5-byte command.  Otherwise, we assume that
 * that was a data byte going to or from another device.
 *
 * The second version of this function reads bytes until it gets
 * a block of 5 that have a correct checksum, and assume that that
 * represents a command regardless of the setting of the command
 * line.
 */
void getcmd(cmdFrame_t *cmdbuf) {
	int rc;
	int cmdcnt;
	byte cSum;

	again:

	cnt = 0;
	while (ring())
		;

	for (cmdcnt = 0; cmdcnt < 5; cmdcnt++) {
		rc = comget((byte *)cmdbuf + cmdcnt);

		if (rc) {
			//uprintf("framing error \n");
			usleep(1000);
			cnt = 0;
			while (!ring())
				;

			if (atari->Instance->SR & UART_FLAG_RXNE) {
				rc = atari->Instance->DR;
			}

			toggleBaud();

			goto again;
		}

	}

	cnt = 0;
	while (!ring())
		;

	cSum = checkSum((byte *)cmdbuf, 4);

	if (cmdbuf->dcheck != cSum) {
		uprintf("Invalid Checksum: c1: %x, c2: %x\n", cmdbuf[4], cSum);
		uprintf("CMD frame: %x %x %x %x %x \n", cmdbuf->dunit, cmdbuf[1], cmdbuf->daux1,
				cmdbuf[3], cmdbuf[4]);
		toggleBaud();
		goto again;
	}

}

void toggleBaud() {
	if (isHiSpeed == TRUE) {
		setBaud(40);
		isHiSpeed = FALSE;
		//uprintf("hibaud off\n");
	} else {
		setBaud(hispeed);
		isHiSpeed = TRUE;
		//uprintf("hibaud on\n");
	}
}

/*
 * closedisk()
 *
 * Closes all disk images.
 */
/*
static void closedisks() {
	int i;
	for (i = 0; i < MAXDISKS; i++) {
		closedisk(i);
	}
}
*/

/*
 * closedisk()
 *
 * Closes a disk image.
 */
static void closedisk(int disk) {

	if (disk >= 0 && disk < MAXDISKS) {
		if (diskfd[disk] != NULL) {
			uprintf("unmounting disk %d\n", disk);
			f_close(diskfd[disk]);
			//uprintf("unmounting fclose done fin: %x\n", diskfd[disk]);
			free(diskfd[disk]->cltbl);
			//uprintf("unmounting free cltbl done\n", disk);
			free(diskfd[disk]);
			//uprintf("unmounting free fin done\n", disk);
			diskfd[disk] = NULL;
			//uprintf("unmounting done\n", disk);
			imagename[disk][0] = 0;
		}
	}

}

/*
 * loaddisk()
 *
 * Ready a disk image.
 * The type of file (xfd/atr) is determined by the file size.
 */
static FRESULT loaddisk(char *path, int disk) {
	FIL *fin = NULL;
	UINT flen;
	DWORD *clmt;
	FRESULT res;

	uprintf("trying to open disk %d, path %s \n", disk, path);

	closedisk(disk);

	if (disk >= MAXDISKS) {
		uprintf("Attempt to load invalid disk number %d\n", disk + 1);
		return FR_DENIED;
	}

	fin = malloc(sizeof(FIL));

	if ((res = f_open(fin, path, FA_READ | FA_WRITE)) != FR_OK) {
		uprintf("Can't open image for read/write %s, res: %d\n", path, res);
		if (res == FR_LOCKED) {
			if ((res = f_open(fin, path, FA_READ)) != FR_OK) {
				uprintf("Can't open image for read %s, res: %d\n", path, res);
				free(fin);
				return res;
			}
		}
		free(fin);
		return res;
	}

	if (f_read(fin, &atrHeader, sizeof(atrHeader), &flen) != FR_OK) {
		uprintf("Could not load header \n");
		return res;
	}

	if (!(clmt = malloc(CLMT_SIZE))) {
		uprintf("Could not malloc CLMT \n");
		return FR_DISK_ERR;
	}

	fin->cltbl = clmt;
	clmt[0] = CLMT_SIZE >> 2;

	if ((res = f_lseek(fin, CREATE_LINKMAP)) != FR_OK) {
		uprintf("Could not create linkmap \n");
		return res;
	}

	if (atrHeader.h0 == 0x96 && atrHeader.h1 == 0x02) {
		seekcode[disk] = atr;
		secsize[disk] = atrHeader.secsizelo + (atrHeader.secsizehi << 8);

		flen = (atrHeader.filesizelo + (atrHeader.filesizemed << 8)
				+ (atrHeader.filesizehi << 16)) << 4;

		seccount[disk] = 3 + ((flen - 384) / secsize[disk]);

		if (secsize[disk] == 256) {
			if (384 + ((seccount[disk] - 3) << 8) != flen) {
				wrongatr[disk] = TRUE;
				seccount[disk]--;
			} else {
				wrongatr[disk] = FALSE;
			}
		}

		uprintf("flen: %d, sectors: %d, size: %d, wrong: %d\n", flen,
				seccount[disk], secsize[disk], wrongatr[disk]);

	} else if (atrHeader.h0 == 0xff && atrHeader.h1 == 0xff) {
		seekcode[disk] = xex;
		secsize[disk] = 128;
		seccount[disk] = f_size(fin) / 125;
		if (f_size(fin) - seccount[disk] * 125 > 0)
			seccount[disk]++;

		uprintf("XEX size: %d, secs: %d \n", f_size(fin), seccount[disk]);
	} else {

		//uprintf("XFD size: %d \n", flen);
		seekcode[disk] = xfd;
		secsize[disk] = 128;
		seccount[disk] = f_size(fin) / 128;
	}

	diskfd[disk] = fin;

	sParms.actual_drive_number = disk;
	sParms.bootloader_relocation = 0;
	sParms.fastsio_active = FALSE;
	sParms.fastsio_pokeydiv = hispeed;

	uprintf("Disk image %s opened%s (%d %d-byte sectors)\n", path,
			ro[disk] ? " read-only" : "", seccount[disk], secsize[disk]);

	return FR_OK;
}

static void filename2atari(byte *buf, char *filename, int index) {
	int len;

	memset(buf, 32, 11);
	ptr = (byte *) strchr(filename, '.');
	if (ptr == NULL) {
		len = strlen(filename);
	} else {
		len = ptr - (byte *) filename;
	}

//uprintf("filenam: %x, ptr: %x, len: %d \n", filename, ptr, len);

	strncpy((char*) buf, filename, len);
	if (ptr != NULL) {
		strncpy((char*) buf + 8, (char *) (ptr + 1), 3);
	}

	buf[11] = 0;
	buf[12] = index & 0xff;
	buf[13] = index >> 8;

//uprintf("found: %s index: %d\n", filename, index);

}

/*
 * decode()
 *
 * Given a command frame (5-bytes), decode it and
 * do whatever needs to be done.
 */
static void decode(cmdFrame_t *cmdbuf) {
	int disk = -1;
	unsigned int sec;
	byte perbuf[13];
	int tracks, secs, density, len;
	DIR dir;
	FRESULT res;
	FILINFO finfo;

	if (SIO_DEBUG) {
		uprintf("%02x %02x %02x %02x %02x \n", cmdbuf->dunit, cmdbuf[1], cmdbuf->daux1,
				cmdbuf[3], cmdbuf[4]);
	}

	//only reply to SIO-Id 0x31 to 0x34 (D1: - D4:)
	if (cmdbuf->dunit >= 0x31 && cmdbuf->dunit < 0x34) {
		disk = cmdbuf->dunit - 0x30;
		//choose disk entry 0 if boot=TRUE and D1: is accessed
		if (boot == TRUE && disk == 1) {
			disk = 0;
		}

		//check if a file-descriptor is loaded for selected disk 0-4
		if (disk < 0 || disk >= MAXDISKS || diskfd[disk] == NULL) {
			uprintf("[drive: %d no image]\n", disk);
			return;
		}
	} else {
		return;
	}

	if (cmdbuf->dunit >= 0x71 && cmdbuf->dunit <= 0x74) {
		if (cmdbuf->dunit == 0x71) {
			disk = 0;
		} else {
			//nack();
			return;
		}
	}

	sec = cmdbuf->daux1 + (cmdbuf->daux2 << 8);

	switch (cmdbuf->dcomnd) {

	//get speed byte
	case 0x3f:
		//uprintf("get speed byte %d \n", hispeed);
		ack();
		complete();
		sendrawdata(&hispeed, 1);
		break;

	// read PERCOM
	case 0x4E:

		ack();

		tracks = 40;
		secs = 18;

		if (secsize[disk] == 128) {
			secs = seccount[disk] / 40;
			tracks = seccount[disk] / secs;
		} else {
			if (seccount[disk] > 2880) {
				tracks = 1;
				secs = seccount[disk];
			} else {
				tracks = seccount[disk] / 18;
				secs = seccount[disk] / tracks;
			}
		}

		density = 0;
		if (!(secsize[disk] == 128 && seccount[disk] == 720)) {
			density = 4;
		}

		perbuf[0] = tracks;
		perbuf[1] = 1;
		perbuf[2] = secs >> 8;
		perbuf[3] = secs & 0xff;
		perbuf[4] = 0;
		perbuf[5] = density;
		perbuf[6] = secsize[disk] >> 8;
		perbuf[7] = secsize[disk] & 0xff;
		perbuf[8] = 0xff;
		perbuf[9] = 0;
		perbuf[10] = 0;
		perbuf[11] = 0;
		/*
		 uprintf("get percom ", hispeed);
		 for (int i = 0; i < 12; i++) {
		 uprintf("%02X ", perbuf[i]);
		 }
		 uprintf("\n");
		 */
		complete();
		sendrawdata(perbuf, 12);
		break;

	//read sector
	case 0x52:
		//uprintf("read disk %d, sector %x: \r", disk, sec);
		ack();
		if (directAccess == TRUE) {
			if (SIO_DEBUG) uprintf("read  direct sector %d\n", sdSector);
			res = readBlock(sdSector << 9, buf);
			complete();
			sendrawdata(buf, 512);
			directAccess = FALSE;
		} else {
			complete();
			senddata(disk, sec);
		}

		break;

	//set percom
	case 0x4F:
		//uprintf("set percom \n");
		//usleep(ACK1);
		ack();
		comread(atari, perbuf, 13);
		ack();
		complete();
		break;

	// write Sector
	case 0x50:
	case 0x57:
		if (sec == 0) {
			if (directAccess == TRUE) {
				if (SIO_DEBUG) uprintf("write direct sector %d\n", sdSector);
				ack();
				comread(atari, buf, 512);
				comread(atari, perbuf, 1);
				ack1();

				if (checkSum(buf, 512) != *perbuf) {
					nack();
					break;
				}
				if (!writeBlock(sdSector << 9, buf)) {
					nack();
					break;
				}
				complete();
				directAccess = FALSE;
			} else {
				goto setDirectSector;
			}
			break;
		}

		if (ro[disk] || seekcode[disk] == xex) {
			nack();
			uprintf("[Read-only image or XeX]\n");
			break;
		}
		ack();
		if (recvdata(disk, sec)) {
			ack1();
			complete();
		} else {
			uprintf("[NACK] sent\n");
			nack();
		}
		break;

// Status
	case 0x53:
		ack();

		/*
		 * Bob Woolley wrote on comp.sys.atari.8bit:
		 *
		 * at your end of the process, the bytes are
		 * CMD status, H/W status, Timeout and unused.
		 * CMD is the $2EA value previously
		 * memtioned. Bit 7 indicates an ED disk.  Bits
		 * 6 and 5 ($6x) indicate DD. Bit 3 indicates
		 * write protected. Bits 0-2 indicate different
		 * error conditions.  H/W is the FDD controller
		 * chip status.  Timeout is the device timeout
		 * value for CIO to use if it wants.
		 *
		 * So, I expect you want to send a $60 as the
		 * first byte if you want the OS to think you
		 * are in DD. OK?
		 */

		status[0] = 0;

		if (secsize[disk] != 128)
			status[0] = 32;

		if (secsize[disk] == 128 && seccount[disk] == 1040)
			status[0] |= 0x80;

		if (ro[disk])
			status[0] |= 8;

		//uprintf("status: %02X %02X %02X %02X \n", status[0], status[1],
		//		status[2], status[3]);

		complete();

		sendrawdata(status, 4);
		break;

	case 0x21:
		ack();
		//uprintf("format ");
		buf[0] = buf[1] = 255;
		Sleep(100);
		complete();
		Sleep(100);
		sendrawdata(buf, secsize[disk]);
		//uprintf("... formatted\n");
		break;

	case 0x20:
		//uprintf("download ");
		break;
	case 0x54:
		//uprintf("readaddr ");
		break;
	case 0x51:
		//uprintf("readspin ");
		break;
	case 0x55:
		//uprintf("motoron ");
		break;
	case 0x56:
		//uprintf("verify ");
		break;

//	get 20 files
	case 0xC0:
		ack();
		memset(buf, 0, 241);
		res = f_opendir(&dir, path);
		len = secs = tracks = 0;
		if (res == FR_OK) {
			len = sec;
			for (; len > 0; len--) {
				f_readdir(&dir, &finfo);
			}

			while (f_readdir(&dir, &finfo) == FR_OK) {
				if (finfo.fname[0] == 0)
					break;

				if (secs >= 240) {
					*(buf + secs) = '*';
					break;
				}

				ptr = (byte *) strchr(finfo.fname, '.');
				if (ptr == NULL) {
					len = strlen(finfo.fname);
				} else {
					len = ptr - (byte *) finfo.fname;
				}
				//uprintf("%s %d %d ", finfo.fname, buf + secs, len);
				memset(buf + secs, 32, 11);
				strncpy((char*) buf + secs, finfo.fname, len);

				if (ptr != NULL) {
					ptr++;
					len = strlen((char *) ptr);
					//uprintf("%s %d %d \n", finfo.fname, buf + secs + 8, len);
					strncpy((char*) (buf + secs + 8), (char *) (ptr), len);
				}
				secs += 11;

				*(buf + secs) = finfo.fattrib;
				secs++;
			}
			f_closedir(&dir);
		}

		complete();
		sendrawdata(buf, 241);
		//uprintf("file: %s\n", buf);
		break;
//set pokey divisio
	case 0xC1:
		ack();
		hispeed = cmdbuf->daux1;
		//uprintf("Set speed byte: %d\n", hispeed);
		complete();
		break;

//set bootloader relocator
	case 0xC2:
		ack();
		//uprintf("Bootloader Relocation: %02X\n", cmdbuf->daux1);
		sParms.bootloader_relocation = cmdbuf->daux1;
		complete();
		break;

//get flags
	case 0xDB:
		ack();
		complete();
		sendrawdata(&flag, 1);
		break;

//  set SD sector Number
	case 0xDD:
setDirectSector:
		ack();
		comread(atari, (byte *) &sdSector, 4);
		comread(atari, perbuf, 1);
		ack1();

		if (checkSum((byte *) &sdSector, 4) != *perbuf) {
			nack();
			break;
		}
		directAccess = TRUE;
		complete();
		//uprintf("SD-sector: %d\n", sdSector);
		break;

//  read SD sector
	case 0xDE:
		ack();
		res = readBlock(sdSector << 9, buf);
		complete();
		sendrawdata(buf, 512);
		break;

//	get SDVersion-Info
	case 0xE0:
		ack();
		complete();
		sendrawdata(version, cmdbuf->daux1);
		break;

//  Init Drive
	case 0xE1:
		ack();
		boot = FALSE;
		complete();
		break;
//deactivate drive
	case 0xE2:
		ack();
		disk = cmdbuf->daux1;
		//uprintf("deactivate drive: %d\n", disk);
		closedisk(disk);
		complete();
		break;
//  Get Filename
	case 0xE3:
		get_filename: ack();

		disk = cmdbuf->daux1;
		filename2atari(buf, imagename[disk], 0);
		buf[12] = 0;
		buf[13] = 0;

		complete();
		sendrawdata(buf, 14);
		break;

//  Get Filename + attrib + info
	case 0xE5:
		ack();
		memset(buf, 32, 50);
		res = f_opendir(&dir, path);

		if (res == FR_OK) {
			for (len = sec; len >= 0; len--) {
				f_readdir(&dir, &finfo);
			}
			f_closedir(&dir);

			//strcpy(imagename[disk], finfo.fname);
			//filename2atari(buf, imagename[disk], sec);

			buf[11] = 0;	// attribute

			dPtr = (DWORD *) (buf + 12);
			*dPtr = finfo.fsize;

			wPtr = (WORD *) (buf + 16);
			*wPtr = finfo.fdate;

			wPtr++;
			*wPtr = finfo.ftime;

			int year = (finfo.fdate >> 9) + 1980;
			int month = (finfo.fdate >> 5) & 0xf;
			int day = finfo.fdate & 0x1f;
			int hour = finfo.ftime >> 11;
			int min = (finfo.ftime >> 5) & 0x3f;
			int seco = (finfo.ftime & 0x1f) << 1;

			sprintf((char *) (buf + 20), "%010d %04d-%02d-%02d %02d:%02d:%02d",
					(int) finfo.fsize, year, month, day, hour, min, seco);

			//1234567890 YYYY-MM-DD HH:MM:SS

			complete();
			sendrawdata(buf, 50);
		} else {
			nack();
		}

		break;

// Get number of direntries
	case 0xEA:
		ack();
		//uprintf("path: %s\n", path);
		res = f_opendir(&dir, path);
		len = 0;
		if (res == FR_OK) {
			while (f_readdir(&dir, &finfo) == FR_OK && finfo.fname[0] != 0) {
				len++;
			}
			f_closedir(&dir);
		}
		//uprintf("number of entries: %d\n", len);
		complete();
		sendrawdata((byte *) &len, 2);
		break;

//  Get Path
	case 0xEB:
		ack();
		memset(buf, 32, cmdbuf->daux1 * 12);
//		f_getcwd((TCHAR *) buf, cmdbuf->daux1);
		strcpy((char *) buf, path);
//		uprintf("Get Path: %s \n", buf);
		complete();
		sendrawdata(buf, cmdbuf->daux1 * 12);
		break;

//  Find Pattern
	case 0xEC:
		ack();
		comread(atari, buf, 11);

		len = strlen((char *) buf);
		//uprintf("pattern: %s, len: %d\n", buf, len);

		res = f_opendir(&dir, path);
		lastIndex = 0;

		if (res == FR_OK) {

			while (f_readdir(&dir, &finfo) == FR_OK) {
				if (finfo.fname[0] == 0)
					break;

				//uprintf("entry: %s\n", finfo.fname);

				if (strncmp((char *) buf, finfo.fname, len) == 0) {
					strcpy(filename, finfo.fname);
					filename2atari(buf, filename, lastIndex);
					break;
				}

				lastIndex++;
			}

			f_closedir(&dir);
		}

		complete();
		sendrawdata(buf, 14);

		break;

//
	case 0xED:
		ack();
		filename2atari(buf, filename, lastIndex);
		complete();
		sendrawdata(buf, 14);

		break;

//  Swap Drive
	case 0xEE:
		ack();
		complete();
		break;

//  Get drive params
	case 0xEF:
		ack();

		sParms.fastsio_active = isHiSpeed;
		sParms.fastsio_pokeydiv = hispeed;

		ptr = (byte *) (&sParms);
		ptr += cmdbuf->daux2;

		//uprintf("Drive parms: %02X %02X %02X %02X\n", *ptr, *(ptr+1), *(ptr+2), *(ptr+3));

		complete();
		sendrawdata(ptr, cmdbuf->daux1);

		//uprintf("Drive parms: %02X %02X\n", *ptr, *(ptr+1));
		break;

//  Set file to Dx:
	case 0xF0:
	case 0xF1:
	case 0xF2:
	case 0xF3:
	case 0xF4:
		ack();

		disk = cmdbuf->dcomnd - 0xF0;

		res = f_opendir(&dir, path);
		if (res == FR_OK) {
			for (len = sec; len >= 0; len--) {
				f_readdir(&dir, &finfo);
			}
			f_closedir(&dir);

			strcpy((char *) buf, path);
			if (path[1] != 0) {
				strcat((char *) buf, "/");
				strcat((char *) buf, finfo.fname);
			} else {
				strcpy((char *) buf, finfo.fname);
			}

//			uprintf("%s\n", buf);

			loaddisk((char *) buf, disk);
			strcpy(imagename[disk], finfo.fname);
		}

		complete();
		break;

//	dir up
	case 0xFD:
		res = f_opendir(&dir, path);

		if (res == FR_OK) {
			while (TRUE) {
				if (f_readdir(&dir, &finfo) == FR_OK && finfo.fname[0] != 0) {
					if (finfo.fname[0] == '.' && finfo.fname[1] == '.') {
						ptr = (byte *) strrchr(path, '/');
						*ptr = 0;
						strcpy(filename, (char *) (ptr + 1));
						// uprintf("%s, %d\n", filename, finfo.fattrib);
						break;
					}
				} else {
					break;
				}
			}
			f_closedir(&dir);
		}
		goto get_filename;

		break;

//  Set root dir
	case 0xFE:
		ack();
		complete();
		strcpy(path, "/");
		break;

//  Set actual dir
	case 0xFF:
		ack();
		res = f_opendir(&dir, path);

		if (res == FR_OK) {

			for (len = sec; len >= 0; len--) {
				f_readdir(&dir, &finfo);
			}
			f_closedir(&dir);

			len = strlen(finfo.fname);

			switch (len) {
			case 0:
				break;
			case 1:
				if (finfo.fname[0] == '.') {
					break;
				}
			case 2:
				if (finfo.fname[0] == '.' && finfo.fname[1] == '.') {
					ptr = (byte *) strrchr(path, '/');
					// uprintf("%x %x \n", path, ptr);

					if ((byte*) path == ptr)
						ptr++;
					*ptr = 0;
					break;
				}
			default:
				if (path[1] != 0) {
					strcat(path, "/");
				}
				strcat(path, finfo.fname);
				break;
			}
			//uprintf("%s %s\n", path, finfo.fname);
		}

		complete();
		break;

	}

}

static uint16_t comwrite(HANDLE hComm, byte *c, uint16_t i) {
	return HAL_UART_Transmit(hComm, c, i, HAL_MAX_DELAY); // send message via UART
}

static uint16_t comread(HANDLE hComm, byte *c, uint16_t i) {

	return HAL_UART_Receive(hComm, c, i, HAL_MAX_DELAY); // send message via UART

}

static uint32_t comget(byte *c) {

	uint32_t sr;

	while (!((sr = atari->Instance->SR) & UART_FLAG_RXNE)) {
		//uprintf("PA11: %d ", HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11));

	}

	*c = (byte) (atari->Instance->DR & 0xff);

	return sr & 0xf;

}

static int ring() {
	int i;

	if (huart2.Instance->SR & UART_FLAG_RXNE) {
		i = (byte) (huart2.Instance->DR & 0xff);

		switch (i) {
		case 'b':
			boot = TRUE;
			break;
		default:
			break;
		}
	}

	if (synced != -1) {
		cnt++;
		if (cnt > 100000) {
			//uprintf("syncing disk %d\n", synced);
			f_sync(diskfd[synced]);
			synced = -1;
			//uprintf("synced!\n");
			cnt = 0;
		}
	}

	return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);
}

