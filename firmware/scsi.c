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

#include "f38x_usb.h"

#include "scsi.h"

#include "usb_msd.h"
#include "util.h"

#include "diskio.h"
#include <stdio.h>
#include <string.h>

enum command_t {
  //SCSI_FORMAT_UNIT                  = 0x04,
  SCSI_INQUIRY                      = 0x12,
  SCSI_START_STOP_UNIT              = 0x1B,
  //SCSI_MODE_SELECT_10               = 0x55,
  SCSI_MODE_SENSE_6                 = 0x1A,
  //SCSI_MODE_SENSE_10                = 0x5A,
  SCSI_PREVENT_ALLOW_MEDIUM_REMOVAL = 0x1E,
  SCSI_READ_10                      = 0x28,
  //SCSI_READ_12                      = 0xA8,
  SCSI_READ_CAPACITY_10             = 0x25,
  //SCSI_READ_FORMAT_CAPACITIES       = 0x23,
  SCSI_REQUEST_SENSE                = 0x03,
  //SCSI_REZERO_UNIT                  = 0x01,
  //SCSI_SEEK_10                      = 0x2B,
  //SCSI_SEND_DIAGNOSTIC              = 0x10,
  SCSI_TEST_UNIT_READY              = 0x00,
  SCSI_VERIFY_10                    = 0x2F,
  SCSI_WRITE_10                     = 0x2A,
  //SCSI_WRITE_12                     = 0xAA,
  //SCSI_WRITE_VERIFY                 = 0x2E,
};

scsi_status_t scsi_status;
static scsi_status_t res_status;

u32 __xdata scsi_residue;
u8 __xdata scsi_lun = 0;
u16 __xdata scsi_block_size = 0;

// Buffer for read/write transfers:
static u8 __xdata scratch[512];

static void (* __xdata pending_job)();

static struct {
  u8 *buf;
  u16 size;
} __xdata tx_data;

static void scsi_tx_1packet(){
  u8 write_count;
  if((write_count = usb_write(tx_data.buf, tx_data.size, MSD_EP_IN)) > 0){
    scsi_residue -= write_count;
    if(tx_data.size -= write_count){
      tx_data.buf += write_count;
    }else{
      scsi_status = res_status;
    }
  }
}

/**
 * scsi_tx_short
 * 
 * This function sends defined numbers of bytes via USB
 * 
 * @param ptr poiter to sending bytes
 * @param count number of sending bytes
 */
static void scsi_tx_short(u8* ptr, unsigned count){
  if(scsi_residue < count){ // case (7)
    res_status = SCSI_PHASE_ERROR;
    tx_data.size = (scsi_residue & 0xFF);
  }else{
    res_status = SCSI_PASSED;
    tx_data.size = count;
  }
  
  tx_data.buf = ptr;
  
  msd_action |= MSD_DEVICE_TX;
  pending_job = scsi_tx_1packet;
  scsi_status = SCSI_PENDING;
}

/**
 * This function responses to inquiry from other USB device
 * 
 */
static void inquiry(){
  static const __code u8 inquiry_response[36] = {
    0x00, // Peripheral qualifier & peripheral device type
    0x80, // Removable medium
    0x05, // Version of the standard (2=obsolete, 5=SPC-3)
    0x02, // No NormACA, No HiSup, response data format=2
    0x20, // No extra parameters
    0x00, // No flags
    0x80, // 0x80 => BQue => Basic Task Management supported
    0x00, // No flags
    'S','i','L','a','b','s',' ',' ', // Requested by Dekimo via www.t10.org
    'M','a','s','s',' ','S','t','o','r','a','g','e',' ',' ',' ',' ',
    '0','0','0','1'
  };
  scsi_tx_short(
      inquiry_response, 
      sizeof(inquiry_response));
}

/**
 * This function responses to resply the requested sense
 * 
 */
static void request_sense(){
  static const __code u8 request_sense_response[18] = {
    (0x70 & 0xEF)     // ResponseCode = ILLEGAL_REQUEST
    | (0x00 & 0x80),  // no data in the information field
    0x00,             // Obsolete = 0
    (0x05 & 0x0F)     // SenseKey=S_ILLEGAL_REQUEST, Refer SPC-3 Section 4.5.6
    | (0x00 & 0x20)   // Incorrect Length Indicator
    | (0x00 & 0x40)   // End of Medium
    | (0x00 & 0x80),  // FileMark, for READ and SPACE commands
    0x00, 0x00, 0x00, 0x00,   // device type or command specific (SPC-33.1.18)
    0x0a,             // AddSenseLen,number of additional sense bytes that follow <=244 n-7 (n=17 (0..17))
    0x00, 0x00, 0x00, 0x00,   // CmdSpecificInfo, depends on command on which exception occured
    0x20,             // ASC, additional sense code = Invalid command opcode
    0x00,             // ASCQ, additional sense code qualifier Section 4.5.2.1 SPC-3 = Invalid command opcode
    0x00,             // FRUC, Field Replaceable Unit Code 4.5.2.5 SPC-3
    0x00, 0x00, 0x00  // SenseKeySpecific, msb is SKSV sense-key specific valied field set=> valid SKS
                      // 18-n additional sense bytes can be defined later 
  };
  scsi_tx_short(
      request_sense_response, 
      sizeof(request_sense_response));
}

/**
 * This function responses to capacity informations request
 * 
 */
static void read_capacity10(){
  
  static struct {
    DWORD_t lba;
    DWORD_t block_length;
  } __xdata read_capacity10_param;
  
  DWORD_t v;
  disk_ioctl(scsi_lun, GET_SECTOR_COUNT, (void *)&v);
  v.i--;
  
#if (defined(__SDCC) || defined(SDCC))
  // Little endian => Big endian
  read_capacity10_param.lba.c[3] = v.c[0];
  read_capacity10_param.lba.c[2] = v.c[1];
  read_capacity10_param.lba.c[1] = v.c[2];
  read_capacity10_param.lba.c[0] = v.c[3];
  v.i = scsi_block_size;
  read_capacity10_param.block_length.c[3] = v.c[0];
  read_capacity10_param.block_length.c[2] = v.c[1];
  read_capacity10_param.block_length.c[1] = v.c[2];
  read_capacity10_param.block_length.c[0] = v.c[3];
#else
  read_capacity10_param.lba.i = be_u32(v.i);
  read_capacity10_param.block_length.i = be_u32((u32)scsi_block_size);
#endif

  scsi_tx_short(
      (u8 *)&read_capacity10_param,
      sizeof(read_capacity10_param));
}

/**
 * This function responses to mode sense information request
 * 
 */
static void mode_sense6(){
  static __code const u8 mode_sense6_response[4] = {
    0x03, 0, 0, 0
  }; // No mode sense parameter
  scsi_tx_short(
      mode_sense6_response, 
      sizeof(mode_sense6_response));
}

static __xdata struct {
  unsigned int blocks;
  int byte_in_block;
  DWORD_t d_LBA;
} scsi_target;

static unsigned char __xdata disk_retry;

/**
 * This function responses to read command
 * 
 */
static void read10(){
  while((scsi_target.byte_in_block >= scsi_block_size)
      || !(scsi_residue)){
    if(scsi_target.blocks){
      if(disk_read(scsi_lun, scratch, scsi_target.d_LBA.i, 1) == RES_OK){
        scsi_target.d_LBA.i++;
        scsi_target.blocks--;
        scsi_target.byte_in_block = 0;
        disk_retry = 0;
        break;
      }else if(!(++disk_retry)){
        scsi_status = SCSI_PHASE_ERROR;
      }
    }else{
      scsi_status = res_status;
    }
    return;
  }
  
  {
    unsigned int write_count;
    write_count = min(
        (scsi_block_size - scsi_target.byte_in_block),
        scsi_residue);
    write_count = usb_write( 
        scratch + scsi_target.byte_in_block, 
        write_count,
        MSD_EP_IN);
    scsi_target.byte_in_block += write_count;
    scsi_residue -= write_count;
  }
}

/**
 * This function responses to write command
 * 
 */
static void write10(){
  while((scsi_target.byte_in_block <= 0) 
      || !(scsi_residue)){
    if(disk_write(scsi_lun, scratch, scsi_target.d_LBA.i, 1) == RES_OK){
      scsi_target.d_LBA.i++;
      if(--(scsi_target.blocks)){
        scsi_target.byte_in_block = scsi_block_size;
        disk_retry = 0;
        break;
      }else{
        scsi_status = res_status;
      }
    }else if(!(++disk_retry)){
      res_status = SCSI_PHASE_ERROR;
    }
    return;
  }
  
  {
    unsigned int read_count;
    read_count = min(
        scsi_target.byte_in_block,
        scsi_residue);
    read_count = usb_read(
        scratch + scsi_block_size - scsi_target.byte_in_block, 
        read_count,
        MSD_EP_OUT); 
    scsi_target.byte_in_block -= read_count;
    scsi_residue -= read_count;
  }
}

void setup_read_write(){
  WORD_t tf_length;
#if (defined(__SDCC) || defined(SDCC))
  // Big endian => Little endian
  tf_length.c[0] = msd_cbw.CBWCB[8];
  tf_length.c[1] = msd_cbw.CBWCB[7];
#else
  tf_length.i = msd_cbw.CBWCB[7];
  tf_length.i <<= 8; tf_length.i |= msd_cbw.CBWCB[8];
#endif
  
  // Case (2), (3)
  if(tf_length.i == 0){
    msd_action 
        = MSD_HOST_SIDE(msd_action) | MSD_DEVICE_NO_DATA;
    scsi_status = SCSI_PASSED;
    return;
  }
  
#if (defined(__SDCC) || defined(SDCC))
  // Big endian => Little endian
  scsi_target.d_LBA.c[0] = msd_cbw.CBWCB[5];
  scsi_target.d_LBA.c[1] = msd_cbw.CBWCB[4];
  scsi_target.d_LBA.c[2] = msd_cbw.CBWCB[3];
  scsi_target.d_LBA.c[3] = msd_cbw.CBWCB[2];
#else
  scsi_target.d_LBA.i = msd_cbw.CBWCB[2];
  scsi_target.d_LBA.i <<= 8; scsi_target.d_LBA.i |= msd_cbw.CBWCB[3];
  scsi_target.d_LBA.i <<= 8; scsi_target.d_LBA.i |= msd_cbw.CBWCB[4];
  scsi_target.d_LBA.i <<= 8; scsi_target.d_LBA.i |= msd_cbw.CBWCB[5];
#endif
  
  scsi_target.blocks = (scsi_residue + scsi_block_size - 1) / scsi_block_size;
  
  // Case (7), (13)
  if(scsi_target.blocks < tf_length.i){
    res_status = SCSI_PHASE_ERROR;
  }
  
  // Case (4), (5), (6), (9), (11), (12)
  else{
    scsi_target.blocks = tf_length.i;
    res_status = SCSI_PASSED;
  }
  
  scsi_target.byte_in_block = scsi_block_size;
  
  scsi_status = SCSI_PENDING;
  disk_retry = 0;
}

/**
 * This function prepares to run scsi commands
 * 
 */
void scsi_setup(){
  switch((enum command_t)(msd_cbw.CBWCB[0])) { // SCSI Operation code
    case SCSI_INQUIRY:
      inquiry();
      break;
    case SCSI_REQUEST_SENSE:
      request_sense();
      break;
    case SCSI_MODE_SENSE_6:
      mode_sense6();
      break;
    case SCSI_READ_CAPACITY_10:
      read_capacity10();
      break;
      
    case SCSI_READ_10:
      pending_job = read10;
      msd_action |= MSD_DEVICE_TX;
      setup_read_write();
      break;
      
    case SCSI_WRITE_10:
      pending_job = write10;
      msd_action |= MSD_DEVICE_RX;
      setup_read_write();
      break;
      
    case SCSI_VERIFY_10:
    case SCSI_TEST_UNIT_READY:
    case SCSI_START_STOP_UNIT:
    case SCSI_PREVENT_ALLOW_MEDIUM_REMOVAL:
      msd_action |= MSD_DEVICE_NO_DATA;
      scsi_status = SCSI_PASSED;
      break;
  }
}

/**
 * This function executes scsi commands
 * 
 */
void scsi_ex(){
  if(scsi_status == SCSI_PENDING){
    pending_job();
  }
}
