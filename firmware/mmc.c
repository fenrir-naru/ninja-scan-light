/*
 * Copyright (c) 2013, M.Naruoka (fenrir)
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 * 
 * - Redistributions of source code must retain the above copyright notice, 
 *   this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice, 
 *   this list of conditions and the following disclaimer in the documentation 
 *   and/or other materials provided with the distribution.
 * - Neither the name of the naruoka.org nor the names of its contributors 
 *   may be used to endorse or promote products derived from this software 
 *   without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS 
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, 
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

#include "c8051f380.h"                 // SFR declarations
#include "mmc.h"
#include "util.h"
#include "type.h"
#include "f38x_spi.h"
#include <string.h>

// Command table value definitions
// Used in the issue_command function to
// decode and execute MMC command requests
enum {NO, YES};
enum {CMD, RD, WR};
enum {R1, R1b, R2, R3, R7};

// Start and stop data tokens for single and multiple
// block MMC data operations
#define     START_SBR      0xFE
#define     START_MBR      0xFE
#define     START_SBW      0xFE
#define     START_MBW      0xFC
#define     STOP_MBW       0xFD

// Mask for data response Token after an MMC write
#define     DATA_RESP_MASK 0x1F

// Mask for busy Token in R1b response
#define     BUSY_BIT       0x80

// This structure defines entries into the command table;
typedef struct {
  unsigned char command_index;     // OpCode;
  unsigned char trans_type;        // Indicates command transfer type;
  unsigned char response;          // Indicates expected response;
} command_t;

// Command table for MMC.  This table contains all commands available in SPI
// mode;  Format of command entries is described above in command structure
// definition;
__code command_t command_list[] = {
    { 0, CMD, R1},  // CMD0;  GO_IDLE_STATE: reset card;
    { 1, CMD, R1},  // CMD1;  SEND_OP_COND: initialize card;
    { 9, RD , R1},  // CMD9;  SEND_CSD: get card specific data;
    {10, RD , R1},  // CMD10; SEND_CID: get card identifier;
    {12, CMD, R1},  // CMD12; STOP_TRANSMISSION: end read;
    {13, CMD, R2},  // CMD13; SEND_STATUS: read card status;
    {16, CMD, R1},  // CMD16; SET_BLOCKLEN: set block size; arg required;
    {17, RD , R1},  // CMD17; READ_SINGLE_BLOCK: read 1 block; arg required;
    {18, RD , R1},  // CMD18; READ_MULTIPLE_BLOCK: read > 1; arg required;
    {24, WR , R1},  // CMD24; WRITE_BLOCK: write 1 block; arg required;
    {25, WR , R1},  // CMD25; WRITE_MULTIPLE_BLOCK: write > 1; arg required;
    {27, CMD, R1},  // CMD27; PROGRAM_CSD: program CSD;
    {28, CMD, R1b}, // CMD28; SET_WRITE_PROT: set wp for group; arg required;
    {29, CMD, R1b}, // CMD29; CLR_WRITE_PROT: clear group wp; arg required;
    {30, CMD, R1},  // CMD30; SEND_WRITE_PROT: check wp status; arg required;
    {32, CMD, R1},  // CMD32; TAG_SECTOR_START: tag 1st erase; arg required;
    {33, CMD, R1},  // CMD33; TAG_SECTOR_END: tag end(single); arg required;
    {34, CMD, R1},  // CMD34; UNTAG_SECTOR: deselect for erase; arg required;
    {35, CMD, R1},  // CMD35; TAG_ERASE_GROUP_START; arg required;
    {36, CMD, R1},  // CMD36; TAG_ERASE_GROUP_END; arg required;
    {37, CMD, R1},  // CMD37; UNTAG_ERASE_GROUP; arg required;
    {38, CMD, R1b}, // CMD38; ERASE: erase all tagged sectors; arg required;
    {42, CMD, R1b}, // CMD42; LOCK_UNLOCK; arg required;
    {55, CMD, R1},  // CMD55; APP_CMD;
    {41, CMD, R1},  // For ACMD41; APP_SEND_OP_CMD; arg required;
    {58, CMD, R3},  // CMD58; READ_OCR: read OCR register;
    {59, CMD, R1},  // CMD59; CRC_ON_OFF: toggles CRC checking; arg required;
    { 8, CMD, R7},  // CMD8;  SEND_IF_COND: Sends SD Memory Card interface condition; arg required;
  };

// Command Table Index Constants:
// Definitions for each table entry in the command table.
// These allow the issue_command function to be called with a
// meaningful parameter rather than a number.
enum {
  GO_IDLE_STATE,
  SEND_OP_COND,
  SEND_CSD,
  SEND_CID,
  STOP_TRANSMISSION,
  SEND_STATUS,
  SET_BLOCKLEN,
  READ_SINGLE_BLOCK,
  READ_MULTIPLE_BLOCK,
  WRITE_BLOCK,
  WRITE_MULTIPLE_BLOCK,
  PROGRAM_CSD,
  SET_WRITE_PROT,
  CLR_WRITE_PROT,
  SEND_WRITE_PROT,
  TAG_SECTOR_START,
  TAG_SECTOR_END,
  UNTAG_SECTOR,
  TAG_ERASE_GROUP_START,
  TAG_ERASE_GROUP_END,
  UNTAG_ERASE_GROUP,
  ERASE,
  LOCK_UNLOCK,
  APP_CMD,
  APP_SEND_OP_CMD,
  READ_OCR,
  CRC_ON_OFF,
  SEND_IF_COND,
};

// MMC block length;  Set during initialization;
__xdata unsigned int mmc_block_length = 0;

// MMC block number;  Computed during initialization;
__xdata unsigned long mmc_physical_sectors = 0;

__bit mmc_initialized = FALSE;
static __bit require_busy_check = FALSE;
static __bit block_addressing = 0;
static __bit sdhc = 0;

#define select_MMC() spi_assert_cs()
#define deselect_MMC() spi_deassert_cs()

/*
 * Send buffer SPI clocks 
 * to ensure no MMC operations are pending
 */
#define prologue() { \
  spi_send_8clock(); \
  select_MMC(); \
}

/*
 * Send 8 more SPI clocks to ensure the card 
 * has finished all necessary operations 
 */
#define epilogue() { \
  deselect_MMC(); \
  spi_send_8clock(); \
}

mmc_res_t mmc_flush() {
  if(require_busy_check){
    require_busy_check = FALSE;
    
    prologue();
    /*
     * wait for end of busy signal;
     * 
     * Start SPI transfer to receive busy tokens;
     * When a non-zero Token is returned,
     * card is no longer busy;
     */
    while(spi_write_read_byte(0xFF) == 0x00);
    epilogue();
  }
  return MMC_NORMAL;
}

#define issue_command(cmd_index, argument, pchar) \
_issue_command(&command_list[cmd_index], argument, pchar)

/**
 * This function generates the necessary SPI traffic for all MMC SPI commands.
 * The three parameters are described below:
 * 
 * Cmd:      This parameter is used to index into the command table and read
 *           the desired command.  The Command Table Index Constants allow the
 *           caller to use a meaningful constant name in the Cmd parameter 
 *           instead of a simple index number.  For example, instead of calling 
 *           issue_command (0, argument, pchar) to send the MMC into idle
 *           state, the user can call 
 *           issue_command (GO_IDLE_STATE, argument, pchar);
 *
 * argument: This parameter is used for MMC commands that require an argument.
 *           MMC arguments are 32-bits long and can be values such as an
 *           an address, a block length setting, or register settings for the
 *           MMC.
 *
 * pchar:    This parameter is a pointer to the local data location for MMC 
 *           data operations.  When a read or write occurs, data will be stored
 *           or retrieved from the location pointed to by pchar.
 *
 * The issue_command function indexes the command table using the Cmd
 * parameter. It reads the command table entry into memory and uses information
 * from that entry to determine how to proceed.  Returns the 16-bit card 
 * response value;
 */
static unsigned char _issue_command(
    __code command_t *current_command,
    unsigned long argument,
    unsigned char *pchar){

  // current data block length;
  static __xdata unsigned int current_blklen = 512;
  
  unsigned char counter;
  
  // Temp variable to preserve data block length during temporary changes;
  unsigned int old_blklen;
  
  // Variable for storing card res;
  unsigned char res;
  
  if(current_command == &command_list[APP_SEND_OP_CMD]){
    issue_command(APP_CMD, 0, NULL);
  }
  
  prologue();
  
  if(require_busy_check){
    require_busy_check = FALSE;
    
    /*
     * wait for end of busy signal;
     * 
     * Start SPI transfer to receive busy tokens;
     * When a non-zero Token is returned,
     * card is no longer busy;
     */
    while(spi_write_read_byte(0xFF) == 0x00);
  }
  
  // Issue command opcode;
  spi_write_read_byte(current_command->command_index | 0x40);
  
  old_blklen = current_blklen;
  switch(current_command->command_index){
    /*
     * If current command changes block length, update block length variable
     * to keep track;
     * 
     * Command byte = 16 means that a set block length command is taking place
     * and block length variable must be set;
     */
    case 16:
      current_blklen = argument;
      break;
    
    /*
     * Command byte = 9 or 10 means that a 16-byte register value is being read
     * from the card, block length must be set to 16 bytes, and restored at the
     * end of the transfer;
     * 
     * Command is a GET_CSD or GET_CID, set block length to 16-bytes;
     */
    case 9:
    case 10:
      current_blklen = 16;
      break;
    
    /*
     * read or write block(s)
     */
    case 17:
    case 18:
    case 24:
    case 25:
      if(!block_addressing){
#if MMC_PHYSICAL_BLOCK_SIZE == 512
        argument <<= 9;
#else
        argument *= MMC_PHYSICAL_BLOCK_SIZE;
#endif
      }
      break;
  }
  
  /*
   * If an argument is required, transmit one, 
   * otherwise transmit 4 bytes of 0x00; 
   */
  counter = sizeof(DWORD_t);
  do {
    DWORD_t v;
    v.i = argument;
    spi_write_read_byte(v.c[--counter]);
  } while(counter);
  
  /*
   * Transmit CRC byte;  In all cases except for CMD0 and CMD8,
   * this will be a dummy character;
   */
  switch(current_command->command_index){
    case 0: spi_write_read_byte(0x95); break;
    case 8: spi_write_read_byte(0x87); break;
    default: spi_write_read_byte(0xFF); break;
  }
  
  // read the response
  timeout_10ms = 0;
  while((res = spi_write_read_byte(0xFF)) & BUSY_BIT){
    if(timeout_10ms > 0x80){
      epilogue();
      return res;
    }
  }
  
  /*
   * The command table entry will indicate what type of response to expect for
   * a given command;  The following conditional handles the MMC response;
   */
  switch(current_command->response){
    case R1b:
      // Read the R1b response;
      /*
       * wait for busy signal to end;
       * 
       * When byte from card is non-zero,
       * card is no longer busy;
       */
      do{
        if(spi_write_read_byte(0xff) != 0x00){break;}
      }while(1);
      break;
    case R1:
      counter = 0; // Already read the R1 response from the card;
      break;
    case R2:
      *(pchar++) = res; // copy the first byte of R2 response
      res = MMC_NORMAL;
      counter = 1; // Read R2 remaining response
      break;
    case R3:
    case R7:
      counter = 4; // Read R3, R7 response;
      break;
  }
  
  // Read additional response.
  while(counter-- > 0){ 
    *(pchar++) = spi_write_read_byte(0xFF);
  }
  
  /*
   * This conditional handles all data operations;  The command entry
   * determines what type, if any, data operations need to occur;
   * Read data from the MMC;
   */
  switch(current_command->trans_type){
    case WR: {
      unsigned char data_res;
      /*
       * Write data to the MMC;
       * Start by sending 8 SPI clocks so the MMC can prepare for the write;
       */
      spi_send_8clock();
      spi_write_read_byte(START_SBW);
      
      spi_write(pchar, current_blklen);  // Write <current_blklen> bytes to MMC;
      
      // Write CRC bytes (don't cares);
      spi_write_read_byte(0xFF);
      spi_write_read_byte(0xFF);
      
      /*
       * Read Data Response from card;
       * 
       * When bit 0 of the MMC response is clear, a valid data response
       * has been received;
       */
      data_res = spi_write_read_byte(0xFF);
      if((data_res & DATA_RESP_MASK) != 0x05){
        epilogue();
        return MMC_ERROR;
      }
      spi_send_8clock();
      if(spi_write_read_byte(0xFF) == 0x00){
        require_busy_check = TRUE;
      }
      break;
    }
    case RD: {
      unsigned char data_res;
      
      /*
       * wait for a start read Token from the MMC
       */
      timeout_10ms = 0;
      do{
        data_res = spi_write_read_byte(0xFF);
        if(data_res == START_SBR){break;}
        if((data_res != 0xFF) || (timeout_10ms >= 0x80)) { 
          epilogue();
          return MMC_ERROR;
        }
      }while(1);
      
      spi_read(pchar, current_blklen);
    
      /*
       * After all data is read, read the two CRC bytes;
       * These bytes are not used in this mode, 
       * but the placeholders must be read anyway;
       */
      spi_write_read_byte(0xFF);
      spi_write_read_byte(0xFF);
      
      break;
    }
  }
  
  epilogue();
  
  // Restore old block length if needed;
  switch(current_command->command_index){
    case 9:
    case 10:
      current_blklen = old_blklen;
      break;
  }
  return res;
}


/**
 * xdata pointer for storing MMC
 * register values;
 * Transmit at least 64 SPI clocks
 * before any bus comm occurs.
 */
static __xdata unsigned char buffer[16];

/**
 * This function initializes the flash card, configures it to operate in SPI
 * mode, and reads the operating conditions register to ensure that the device
 * has initialized correctly.  It also determines the size of the card by
 * reading the Card Specific Data Register (CSD).
 */
void mmc_init(){
  
  unsigned short loopguard;
  unsigned char counter;  // SPI byte counter;
  
  if(mmc_initialized){return;}
  
  spi_clock(200); // speed up spi, 200KHz.
  
  deselect_MMC();
  wait_ms(100);
  
  /*
   * Select the MMC with the CS pin;
   * Send 74 more SPI clocks to ensure proper startup;
   */
  counter = 10;
  while(counter--){spi_write_read_byte(0xFF);}
  
  select_MMC(); 
 	
  wait_ms(1);
  
  /*
   * Send the GO_IDLE_STATE command with CS driven low;
   * This causes the MMC to enter SPI mode;
   */
  issue_command(GO_IDLE_STATE, 0, NULL);

  loopguard = 0;
  if(issue_command(SEND_IF_COND, 0x01AA, buffer) == 1) {
    /* SDHC */
    sdhc = 1;
    if((buffer[2] == 0x01) && (buffer[3] == 0xAA)){
      /* The card can work at vdd range of 2.7-3.6V */
      /* Wait for leaving idle state (ACMD41 with HCS bit) */
      do{
        if(issue_command(APP_SEND_OP_CMD, 1UL << 30, NULL) == MMC_NORMAL){break;}
        wait_us(50);
        if((++loopguard) > 0x1000){return;}
      }while(1);
      if(issue_command(READ_OCR, 0, buffer) == MMC_NORMAL){
        /* Check CCS bit in the OCR */
        if(buffer[0] & 0x40){block_addressing = 1;}
      }
    }
  }else{
    /* SDSC or MMC */
    unsigned char cmd; 
    if(issue_command(APP_SEND_OP_CMD, 0, NULL) <= 1){
      /* SDSC */
      cmd = APP_SEND_OP_CMD;
    }else{
      /* MMC */
      cmd = SEND_OP_COND;
    }
    /* Wait for leaving idle state */
    while(issue_command(cmd, 0, NULL)){
      wait_us(50);
      if((++loopguard) > 0x1000){return;}
    }
    /* Set R/W block length to 512 */
    if(issue_command(SET_BLOCKLEN, (unsigned long)MMC_PHYSICAL_BLOCK_SIZE, NULL) != 0){
      return;
    }
  }
  
  /*
   * Get the Card Specific Data (CSD)
   * register to determine the size of the MMC;
   */
  if((issue_command(SEND_STATUS, 0, buffer) != MMC_NORMAL)
      || (issue_command(SEND_CSD, 0, buffer) != MMC_NORMAL)){
    return;
  }

  switch(buffer[0] & 0xC0){
    case (0x01 << 6):
      mmc_block_length = 512;
      mmc_physical_sectors = ((unsigned long)(buffer[7] & 0x3F) << 16)
          | (buffer[8] << 8) | buffer[9];
      mmc_physical_sectors++;
      mmc_physical_sectors <<= 10;
      break;
    case (0x00 << 6): {
      unsigned int c_size 
          = ((buffer[6] & 0x03) << 10) | (buffer[7] << 2) | ((buffer[8] & 0xc0) >> 6);
      unsigned char c_mult 
          = (((buffer[9] & 0x03) << 1) | ((buffer[10] & 0x80) >> 7));
      mmc_block_length = 1 << (buffer[5] & 0x0f);
      
      // Determine the number of MMC sectors;
      mmc_physical_sectors = (unsigned long)(c_size+1)*(1 << (c_mult+2));
      break;
    }
    default:
      return;
  }
  /* バイト-ビット対応表
   *  0 127-120
   *  1 119-112
   *  2 111-104
   *  3 110-96
   *  4 95-88
   *  5 87-80
   *  6 79-72
   *  7 71-64
   *  8 63-56
   *  9 55-48
   * 10 47-40
   * 11 39-32
   * 12 31-24
   * 13 23-16
   * 14 15-8
   * 15 7-0
   */
  
  spi_clock(12000); // speed up spi, 12MHz.
  
  mmc_initialized = TRUE;
}

/**
 * If you know beforehand that you'll read an entire 512-byte block, then
 * this function has a smaller ROM footprint than MMC_FLASH_Read.
 * 
 * @param address address of block
 * @param pchar pointer to byte
 * @return card status
 */
mmc_res_t mmc_read(
    unsigned long address, 
    unsigned char *pchar){
  return (issue_command(READ_SINGLE_BLOCK, address, pchar) == MMC_NORMAL
      ? MMC_NORMAL : MMC_ERROR);
}

/**
 * If you know beforehand that you'll write an entire 512-byte block, then
 * this function is more RAM-efficient than MMC_FLASH_Write because it
 * doesn't require a 512-byte buffer (and it's faster too, it doesn't
 * require a read operation first). And it has a smaller ROM footprint too.
 * 
 * @param address address of block
 * @param wdata pointer to data
 * @return card status
 */
mmc_res_t mmc_write(
    unsigned long address, 
    unsigned char *wdata){
  return (issue_command(WRITE_BLOCK, address, wdata) == MMC_NORMAL
      ? MMC_NORMAL : MMC_ERROR);
}

/**
 * Function returns the status of MMC card
 * 
 */
mmc_res_t mmc_get_status(){
  return (issue_command(SEND_STATUS, 0, buffer) == MMC_NORMAL
      ? MMC_NORMAL : MMC_ERROR);
}
