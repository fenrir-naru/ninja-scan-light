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

#include <stdio.h>

#include "main.h"
#include "f38x_usb.h"

#include "usb_msd.h"
#include "scsi.h"
#include "util.h"
#include "diskio.h"

#define DIRECTION_IN	  0x80
#define DIRECTION_OUT	0x00

#define CBW_SIGNATURE 0x43425355
#define CSW_SIGNATURE 0x53425355

typedef enum {
  MSD_READY,
  MSD_COMMAND_TRANSPORT,
  MSD_DATA,
  MSD_ZERO_PADDING,
  MSD_DO_STALL,
  MSD_STALLING,
  MSD_MAKE_CSW,
  MSD_STATUS_TRANSPORT,
  MSD_DO_RESET
} msd_state_t;

static msd_state_t __xdata msd_State;

msd_cbw_t __xdata msd_cbw;
msd_csw_t __xdata msd_csw;

static volatile __bit discard_state_transition;

static volatile __bit ep_in_stall = FALSE;
static volatile __bit ep_out_stall = FALSE;

void usb_MSD_init(){
  if(ep_in_stall){
    usb_clear_halt(DIR_IN, MSD_EP_IN);
    usb_status_unlock(DIR_IN, MSD_EP_IN);
  }
  if(ep_out_stall){
    usb_clear_halt(DIR_OUT, MSD_EP_OUT);
    usb_status_unlock(DIR_OUT, MSD_EP_OUT);
  }
  msd_State = MSD_DO_RESET;
  discard_state_transition = TRUE;
}

static void usb_MSD_reset(){
  // Parse this class-specific request
  if((usb_setup_buf.bmRequestType == 0x21) 
      && (usb_setup_buf.wLength.i == 0x00)) {
    
    usb_status_unlock(DIR_IN, MSD_EP_IN);
    usb_status_unlock(DIR_OUT, MSD_EP_OUT);
    
    msd_State = MSD_DO_RESET;
    discard_state_transition = TRUE;
    
    usb_request_completed = TRUE;
  }
}

static void usb_MSD_Get_MaxLUN(){ 
  // Parse this class-specific request
  if((usb_setup_buf.bmRequestType == 0xA1) 
      && (usb_setup_buf.wLength.i == 0x01)
      && (usb_setup_buf.wValue.i == 0x00)) {
    // Return max lun to host:
    static __code unsigned char maxlun[] = {0x00}; // Only 1 LUN supported
    regist_data(maxlun, 1);
    
    // put endpoint in transmit mode
    usb_ep0_status = EP_TX;
    usb_request_completed = TRUE;
  }
}

void usb_MSD_req(){
  switch(usb_setup_buf.bRequest){
    case MSD_GET_MAX_LUN:
      usb_MSD_Get_MaxLUN();
      break;
    case MSD_RESET:
      usb_MSD_reset();
      break;
  }
}

u8 __xdata msd_action;

/**
 * This is a polling function. 
 * It checks if something is received and calls the responding functions (USB).
 * 
 */
void msd_polling(){
  
  msd_state_t current;
  
  CRITICAL_USB0(
    current = msd_State;
    discard_state_transition = FALSE;
  );
  
  switch(current) {
    case MSD_READY: {
      unsigned int ready_bytes = usb_count_ep_out(MSD_EP_OUT); 
      if(!ready_bytes){return;}
      
      // Look for a "valid" and "meaningful" CBW, 
      // as defined in the spec: Check size
      if(ready_bytes == sizeof(msd_cbw_t)) {
        
        usb_read((BYTE*)&msd_cbw, sizeof(msd_cbw_t), MSD_EP_OUT);
        
        // Check signature, reserved bits & LUN
        if((CBW_SIGNATURE == le_u32(msd_cbw.dCBWSignature.i)) 
            && (msd_cbw.bCBWLUN <= 0x0F)
            && (msd_cbw.bCBWCBLength && (msd_cbw.bCBWCBLength <= 0x10))
            && (!(msd_cbw.bmCBWFlags & (~0x80)))){
          
          scsi_residue = le_u32(msd_cbw.dCBWDataTransferLength.i);
          scsi_status = SCSI_FAILED;
          
          scsi_lun = msd_cbw.bCBWLUN;
          disk_ioctl(scsi_lun, GET_SECTOR_SIZE, (void *)&scsi_block_size);
          
          if(scsi_residue){
            msd_action 
                = (msd_cbw.bmCBWFlags & 0x80) ? MSD_HOST_RX : MSD_HOST_TX;
          }else{
            msd_action = MSD_HOST_NO_DATA;
          }
          
          scsi_setup();
          
          current = MSD_DATA;
          ep_in_stall = FALSE;
          ep_out_stall = FALSE;
          
          switch(msd_action){
            // case(2),(3)
            case (MSD_HOST_NO_DATA | MSD_DEVICE_RX):
            case (MSD_HOST_NO_DATA | MSD_DEVICE_TX):
              current = MSD_MAKE_CSW;
              scsi_status = SCSI_PHASE_ERROR;
              break;
            
            // case(8)
            case (MSD_HOST_TX | MSD_DEVICE_TX):
              scsi_status = SCSI_PHASE_ERROR;
              current = MSD_DO_STALL;
              ep_in_stall = TRUE;
              break;
            
            // case(10)
            case (MSD_HOST_RX | MSD_DEVICE_RX):
              scsi_status = SCSI_PHASE_ERROR;
              current = MSD_DO_STALL;
              ep_out_stall = TRUE;
              break;
          }
          
          break;
        }
      }else{
        usb_flush(MSD_EP_OUT);
      }
      
      ep_in_stall = TRUE;
      ep_out_stall = TRUE;
      usb_status_lock(DIR_IN, MSD_EP_IN);
      usb_status_lock(DIR_OUT, MSD_EP_OUT);
      current = MSD_DO_STALL;
      break;
    }

    case MSD_DATA:
      scsi_ex();
      if(scsi_status != SCSI_PENDING){
        if(scsi_residue > 0){
          
          // Case (4) and Case (5)?
          if(MSD_HOST_SIDE(msd_action) == MSD_HOST_RX){
            msd_csw.dCSWDataResidue.i = le_u32(scsi_residue);
            current = MSD_ZERO_PADDING;
            break;
          }
          // Case (9) and Case (11)?
          else if(MSD_HOST_SIDE(msd_action) == MSD_HOST_TX){
            ep_out_stall = TRUE;
            current = MSD_DO_STALL;
            break;
          }
        }
        current = MSD_MAKE_CSW;
      }
      break;
    
    case MSD_ZERO_PADDING:
      {
        u32 fill_length = le_u32(msd_csw.dCSWDataResidue.i);
        fill_length -= usb_fill(0, fill_length, MSD_EP_IN);
        msd_csw.dCSWDataResidue.i = le_u32(fill_length);
        if(!fill_length){current = MSD_MAKE_CSW;}
        break;
      }
    
    case MSD_DO_STALL:
      if(ep_in_stall){usb_stall(DIR_IN, MSD_EP_IN, NULL);}
      if(ep_out_stall){usb_stall(DIR_OUT, MSD_EP_OUT, NULL);}
      current = MSD_STALLING;
    
    case MSD_STALLING:
      CRITICAL_USB0(
        ep_in_stall = (usb_ep_status(DIR_IN, MSD_EP_IN) != EP_IDLE);
        ep_out_stall = (usb_ep_status(DIR_OUT, MSD_EP_OUT) != EP_IDLE);
      );
      if(ep_in_stall || ep_out_stall){break;}
    
    case MSD_MAKE_CSW:
      // Reply with a CSW:
      msd_csw.dCSWSignature.i = le_u32(CSW_SIGNATURE);
      msd_csw.dCSWTag.i = msd_cbw.dCBWTag.i;
      msd_csw.bCSWStatus = scsi_status;
      msd_csw.dCSWDataResidue.i = le_u32(scsi_residue);
      current = MSD_STATUS_TRANSPORT;

		case MSD_STATUS_TRANSPORT:
			if(usb_write((BYTE*)&msd_csw, sizeof(msd_csw_t), MSD_EP_IN)){
        current = MSD_READY;
      }
      break;

    case MSD_DO_RESET:
    default:
      current = MSD_READY;
      break;
  }
  
  CRITICAL_USB0(
    if(!discard_state_transition){
      msd_State = current;
    }
  );
}
