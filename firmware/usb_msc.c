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

#include "usb_msc.h"
#include "scsi.h"
#include "util.h"
#include "diskio.h"

#define DIRECTION_IN   0x80
#define DIRECTION_OUT  0x00

#define CBW_SIGNATURE 0x43425355
#define CSW_SIGNATURE 0x53425355

typedef enum {
  MSC_READY,
  MSC_COMMAND_TRANSPORT,
  MSC_DATA,
  MSC_ZERO_PADDING,
  MSC_DO_STALL,
  MSC_STALLING,
  MSC_MAKE_CSW,
  MSC_STATUS_TRANSPORT,
  MSC_DO_RESET
} msc_state_t;

static msc_state_t __xdata msc_State;

msc_cbw_t __xdata msc_cbw;
msc_csw_t __xdata msc_csw;

static volatile __bit discard_state_transition;

static volatile __bit ep_in_stall = FALSE;
static volatile __bit ep_out_stall = FALSE;

void usb_MSC_init(){
  if(ep_in_stall){
    usb_clear_halt(DIR_IN, MSC_EP_IN);
    usb_status_unlock(DIR_IN, MSC_EP_IN);
  }
  if(ep_out_stall){
    usb_clear_halt(DIR_OUT, MSC_EP_OUT);
    usb_status_unlock(DIR_OUT, MSC_EP_OUT);
  }
  msc_State = MSC_DO_RESET;
  discard_state_transition = TRUE;
}

static void usb_MSC_reset(){
  // Parse this class-specific request
  if((ep0_setup.bmRequestType == 0x21) 
      && (ep0_setup.wLength.i == 0x00)) {
    
    usb_status_unlock(DIR_IN, MSC_EP_IN);
    usb_status_unlock(DIR_OUT, MSC_EP_OUT);
    
    msc_State = MSC_DO_RESET;
    discard_state_transition = TRUE;
    
    ep0_request_completed = TRUE;
  }
}

static void usb_MSC_Get_MaxLUN(){ 
  // Parse this class-specific request
  if((ep0_setup.bmRequestType == 0xA1) 
      && (ep0_setup.wLength.i == 0x01)
      && (ep0_setup.wValue.i == 0x00)) {
    // Return max lun to host:
    static __code unsigned char maxlun[] = {0x00}; // Only 1 LUN supported
    ep0_register_data(maxlun, 1);
    
    // put endpoint in transmit mode
    usb_ep0_status = EP_TX;
    ep0_request_completed = TRUE;
  }
}

void usb_MSC_req(){
  switch(ep0_setup.bRequest){
    case MSC_GET_MAX_LUN:
      usb_MSC_Get_MaxLUN();
      break;
    case MSC_RESET:
      usb_MSC_reset();
      break;
  }
}

u8 __xdata msc_action;

/**
 * This is a polling function. 
 * It checks if something is received and calls the responding functions (USB).
 * 
 */
void msc_polling(){
  
  msc_state_t current;
  
  CRITICAL_USB0(
    current = msc_State;
    discard_state_transition = FALSE;
  );
  
  switch(current) {
    case MSC_READY: {
      unsigned int ready_bytes = usb_count_ep_out(MSC_EP_OUT); 
      if(!ready_bytes){return;}
      
      // Look for a "valid" and "meaningful" CBW, 
      // as defined in the spec: Check size
      if(ready_bytes == sizeof(msc_cbw_t)) {
        
        usb_read((BYTE*)&msc_cbw, sizeof(msc_cbw_t), MSC_EP_OUT);
        
        // Check signature, reserved bits & LUN
        if((CBW_SIGNATURE == le_u32(msc_cbw.dCBWSignature.i)) 
            && (msc_cbw.bCBWLUN <= 0x0F)
            && (msc_cbw.bCBWCBLength && (msc_cbw.bCBWCBLength <= 0x10))
            && (!(msc_cbw.bmCBWFlags & (~0x80)))){
          
          scsi_residue = le_u32(msc_cbw.dCBWDataTransferLength.i);
          scsi_status = SCSI_FAILED;
          
          scsi_lun = msc_cbw.bCBWLUN;
          disk_ioctl(scsi_lun, GET_SECTOR_SIZE, (void *)&scsi_block_size);
          
          if(scsi_residue){
            msc_action 
                = (msc_cbw.bmCBWFlags & 0x80) ? MSC_HOST_RX : MSC_HOST_TX;
          }else{
            msc_action = MSC_HOST_NO_DATA;
          }
          
          scsi_setup();
          
          current = MSC_DATA;
          ep_in_stall = FALSE;
          ep_out_stall = FALSE;
          
          // @see http://www.usb.org/developers/devclass_docs/usbmassbulk_10.pdf Table 6.1
          switch(msc_action){
            // case(2),(3)
            case (MSC_HOST_NO_DATA | MSC_DEVICE_RX):
            case (MSC_HOST_NO_DATA | MSC_DEVICE_TX):
              current = MSC_MAKE_CSW;
              scsi_status = SCSI_PHASE_ERROR;
              break;
            
            // case(8)
            case (MSC_HOST_TX | MSC_DEVICE_TX):
              scsi_status = SCSI_PHASE_ERROR;
              current = MSC_DO_STALL;
              ep_in_stall = TRUE;
              break;
            
            // case(10)
            case (MSC_HOST_RX | MSC_DEVICE_RX):
              scsi_status = SCSI_PHASE_ERROR;
              current = MSC_DO_STALL;
              ep_out_stall = TRUE;
              break;
          }
          
          break;
        }
      }else{
        usb_flush(MSC_EP_OUT);
      }
      
      ep_in_stall = TRUE;
      ep_out_stall = TRUE;
      usb_status_lock(DIR_IN, MSC_EP_IN);
      usb_status_lock(DIR_OUT, MSC_EP_OUT);
      current = MSC_DO_STALL;
      break;
    }

    case MSC_DATA:
      scsi_ex();
      if(scsi_status != SCSI_PENDING){
        if(scsi_residue > 0){
          
          // Case (4) and Case (5)?
          if(MSC_HOST_SIDE(msc_action) == MSC_HOST_RX){
            msc_csw.dCSWDataResidue.i = le_u32(scsi_residue);
            current = MSC_ZERO_PADDING;
            break;
          }
          // Case (9) and Case (11)?
          else if(MSC_HOST_SIDE(msc_action) == MSC_HOST_TX){
            ep_out_stall = TRUE;
            current = MSC_DO_STALL;
            break;
          }
        }
        current = MSC_MAKE_CSW;
      }
      break;
    
    case MSC_ZERO_PADDING:
      {
        u32 fill_length = le_u32(msc_csw.dCSWDataResidue.i);
        fill_length -= usb_fill(0, fill_length, MSC_EP_IN);
        msc_csw.dCSWDataResidue.i = le_u32(fill_length);
        if(!fill_length){current = MSC_MAKE_CSW;}
        break;
      }
    
    case MSC_DO_STALL:
      if(ep_in_stall){usb_stall(DIR_IN, MSC_EP_IN, NULL);}
      if(ep_out_stall){usb_stall(DIR_OUT, MSC_EP_OUT, NULL);}
      current = MSC_STALLING;
    
    case MSC_STALLING:
      CRITICAL_USB0(
        ep_in_stall = (usb_ep_status(DIR_IN, MSC_EP_IN) != EP_IDLE);
        ep_out_stall = (usb_ep_status(DIR_OUT, MSC_EP_OUT) != EP_IDLE);
      );
      if(ep_in_stall || ep_out_stall){break;}
    
    case MSC_MAKE_CSW:
      // Reply with a CSW:
      msc_csw.dCSWSignature.i = le_u32(CSW_SIGNATURE);
      msc_csw.dCSWTag.i = msc_cbw.dCBWTag.i;
      msc_csw.bCSWStatus = scsi_status;
      msc_csw.dCSWDataResidue.i = le_u32(scsi_residue);
      current = MSC_STATUS_TRANSPORT;

    case MSC_STATUS_TRANSPORT:
      if(usb_write((BYTE*)&msc_csw, sizeof(msc_csw_t), MSC_EP_IN)){
        current = MSC_READY;
      }
      break;

    case MSC_DO_RESET:
    default:
      current = MSC_READY;
      break;
  }
  
  CRITICAL_USB0(
    if(!discard_state_transition){
      msc_State = current;
    }
  );
}
