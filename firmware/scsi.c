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

#define SCSI_TEST_UNIT_READY              0x00
#define SCSI_REQUEST_SENSE                0x03
#define SCSI_FORMAT_UNIT                  0x04
#define SCSI_SEND_DIAGNOSTIC              0x10
#define SCSI_INQUIRY                      0x12
#define SCSI_MODE_SELECT_6                0x15
#define SCSI_MODE_SENSE_6                 0x1A
#define SCSI_START_STOP_UNIT              0x1B
#define SCSI_PREVENT_ALLOW_MEDIUM_REMOVAL 0x1E
#define SCSI_READ_CAPACITY_10             0x25
#define SCSI_READ_CAPACITY_16             0x9E
#define SCSI_READ_6                       0x08
#define SCSI_READ_10                      0x28
#define SCSI_READ_16                      0x88
#define SCSI_WRITE_10                     0x2A
#define SCSI_VERIFY_10                    0x2F
#define SCSI_READ_FORMAT_CAPACITIES       0x23

u8 __xdata scsi_Status;
u32 __xdata scsi_Residue;

// Buffer for read/write transfers:
static u8 __xdata scratch[512];

static __code const u8 scsi_Standard_Inquiry_Data[36]= {
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

static __code const u8 scsi_Sense_Response_Body[18] = {
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

static struct {
  DWORD_t lba;
  DWORD_t block_length;
} __xdata scsi_Read_Capacity_10;

u8 __xdata scsi_lun = 0;
u16 __xdata scsi_block_size = 0;

static __code const u8 scsi_Mode_Sense_6[4]= { 0x03,0,0,0 }; // No mode sense parameter

static __code void (* __xdata pending_job)(void);

static u8 __xdata res_status;

static u8 * __xdata tx_buf_ptr;
static __xdata unsigned tx_buf_count;

static void scsi_tx_1packet(){
  u8 write_count;
  if(write_count = usb_write(tx_buf_ptr, tx_buf_count, MSD_EP_IN)){
    scsi_Residue -= write_count;
    if(tx_buf_count -= write_count){
      tx_buf_ptr += write_count;  
    }else{
      scsi_Status = res_status;
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
  if(scsi_Residue < count){ // case (7)
    res_status = SCSI_PHASE_ERROR;
    tx_buf_count = (scsi_Residue & 0xFF);
  }else{
    res_status = SCSI_PASSED;
    tx_buf_count = count;
  }
  
  tx_buf_ptr = ptr;
  
  msd_action |= MSD_DEVICE_TX;
  pending_job = scsi_tx_1packet;
  scsi_Status = SCSI_PENDING;
}

/**
 * This function responses to inquiry from other USB device
 * 
 */
static void scsi_Inquiry(){
  scsi_tx_short(
      scsi_Standard_Inquiry_Data, 
      sizeof(scsi_Standard_Inquiry_Data));
}

/**
 * This function responses to resply the requested sense
 * 
 */
static void scsi_Request_Sense(){
  scsi_tx_short(
      scsi_Sense_Response_Body, 
      sizeof(scsi_Sense_Response_Body));
}

/**
 * This function responses to capacity informations request
 * 
 */
static void scsi_Read_Capacity10(){
	
  u32 count;
  disk_ioctl(scsi_lun, GET_SECTOR_COUNT, (void *)&count);
  count--;
  
  scsi_Read_Capacity_10.lba.i = be_u32(count);
  scsi_Read_Capacity_10.block_length.i = be_u32(scsi_block_size);

  scsi_tx_short(
      (u8 *)&scsi_Read_Capacity_10,
      sizeof(scsi_Read_Capacity_10));
}

/**
 * This function responses to mode sense information request
 * 
 */
static void scsi_Mode_Sense6(){
  scsi_tx_short(
      scsi_Mode_Sense_6, 
      sizeof(scsi_Mode_Sense_6));
}

static __xdata struct {
  unsigned int blocks;
  int byte_in_block;
  u32 d_LBA;
} scsi_target;

static unsigned char __xdata disk_retry;

/**
 * This function responses to read command
 * 
 */
static void scsi_Read10(){
  while((scsi_target.byte_in_block >= scsi_block_size)
      || !(scsi_Residue)){
    if(scsi_target.blocks){
      if(disk_read(scsi_lun, scratch, scsi_target.d_LBA, 1) == RES_OK){
        scsi_target.d_LBA++;
        scsi_target.blocks--;
        scsi_target.byte_in_block = 0;
        disk_retry = 0;
        break;
      }else if(!(++disk_retry)){
        scsi_Status = SCSI_PHASE_ERROR;
      }
    }else{
      scsi_Status = res_status;
    }
    return;
  }
  
  {
    unsigned int write_count;
    write_count = min(
        (scsi_block_size - scsi_target.byte_in_block),
        scsi_Residue);
    write_count = usb_write( 
        scratch + scsi_target.byte_in_block, 
        write_count,
        MSD_EP_IN);
    scsi_target.byte_in_block += write_count;
    scsi_Residue -= write_count;
  }
}

/**
 * This function responses to write command
 * 
 */
static void scsi_Write10(){
  while((scsi_target.byte_in_block <= 0) 
      || !(scsi_Residue)){
    if(disk_write(scsi_lun, scratch, scsi_target.d_LBA, 1) == RES_OK){
      scsi_target.d_LBA++;
      if(--(scsi_target.blocks)){
        scsi_target.byte_in_block = scsi_block_size;
        disk_retry = 0;
        break;
      }else{
        scsi_Status = res_status;
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
        scsi_Residue);
    read_count = usb_read(
        scratch + scsi_block_size - scsi_target.byte_in_block, 
        read_count,
        MSD_EP_OUT); 
    scsi_target.byte_in_block -= read_count;
    scsi_Residue -= read_count;
  }
}

void setup_read_write(){
  // Big endian
  unsigned int tf_length = msd_cbw.CBWCB[7];
  tf_length <<= 8; tf_length |= msd_cbw.CBWCB[8];
  
  // Case (2), (3)
  if(!tf_length){
  	msd_action 
        = MSD_HOST_SIDE(msd_action) | MSD_DEVICE_NO_DATA;
    scsi_Status = SCSI_PASSED;
    return;
  }
  
  // Big endian
  scsi_target.d_LBA = msd_cbw.CBWCB[2];
  scsi_target.d_LBA <<= 8; scsi_target.d_LBA |= msd_cbw.CBWCB[3];
  scsi_target.d_LBA <<= 8; scsi_target.d_LBA |= msd_cbw.CBWCB[4];
  scsi_target.d_LBA <<= 8; scsi_target.d_LBA |= msd_cbw.CBWCB[5];
  
  scsi_target.blocks = (scsi_Residue + scsi_block_size - 1) / scsi_block_size;
  
  // Case (7), (13)
  if(scsi_target.blocks < tf_length){
    res_status = SCSI_PHASE_ERROR;
  }
  
  // Case (4), (5), (6), (9), (11), (12)
  else{
    scsi_target.blocks = tf_length;
    res_status = SCSI_PASSED;
  }
  
  scsi_target.byte_in_block = scsi_block_size;
  
  scsi_Status = SCSI_PENDING;
  disk_retry = 0;
}

/**
 * This function prepares to run scsi commands
 * 
 */
void scsi_setup(){
  switch(msd_cbw.CBWCB[0]) { // SCSI Operation code
    case SCSI_INQUIRY:
      scsi_Inquiry();
      break;
    case SCSI_REQUEST_SENSE:
      scsi_Request_Sense();
      break;
    case SCSI_MODE_SENSE_6:
      scsi_Mode_Sense6();
      break;
    case SCSI_READ_CAPACITY_10:
      scsi_Read_Capacity10();
      break;
      
    case SCSI_READ_10:
      pending_job = scsi_Read10;
      msd_action |= MSD_DEVICE_TX;
      setup_read_write();
      break;
      
    case SCSI_WRITE_10:
      pending_job = scsi_Write10;
      msd_action |= MSD_DEVICE_RX;
      setup_read_write();
      break;
      
    case SCSI_VERIFY_10:
    case SCSI_TEST_UNIT_READY:
    case SCSI_START_STOP_UNIT:
    case SCSI_PREVENT_ALLOW_MEDIUM_REMOVAL:
      msd_action |= MSD_DEVICE_NO_DATA;
      scsi_Status = SCSI_PASSED;
      break;
  }
}

/**
 * This function executes scsi commands
 * 
 */
void scsi_ex(){
  if(scsi_Status == SCSI_PENDING){
    pending_job();
  }
}
