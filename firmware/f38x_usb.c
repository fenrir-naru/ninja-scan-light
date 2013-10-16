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
#include <string.h>

#include "c8051f380.h"
#include "main.h"
#include "f38x_usb_register.h"
#include "f38x_usb.h"
#include "usb_descriptor.h"

#include "usb_std_req.h"
#include "usb_other_req.h"

#include "usb_msd.h"
#include "usb_cdc.h"

// Holds the current USB State def. in F34x_USB_Main.h
__xdata BYTE usb_state;

// Buffer for current device request information
__xdata setup_buffer_t usb_setup_buf;

// Size of data to return
__xdata unsigned int ep0_Data_Size;

// Pointer to data to return
BYTE* ep0_Data_Ptr;

volatile __bit usb_request_completed;
void __xdata (*ep0_callback)();

// Holds the status for each endpoint
BYTE __xdata usb_ep_stat[7];

static __xdata unsigned int ep_out_stored[3];
#define count_ep_out(index) ep_out_stored[index - 1]

static __code unsigned int ep_size[] = {
  PACKET_SIZE_EP0,
  PACKET_SIZE_EP1,
  PACKET_SIZE_EP2,
  PACKET_SIZE_EP3
};

static __code void(* __xdata stall_callbacks[6])();

#define EP_INDEX_2_LINER_INDEX(dir, ep_index) \
((dir == DIR_IN) ? (ep_index - 1) : (ep_index + 2))

// consider into both interrupt and user invoked cases
#ifdef USE_ASM_FOR_SFR_MANIP
#define CRITICAL_USB0_SPECIAL(func) \
{ \
  BYTE eie1_backup = EIE1; \
  {__asm anl _EIE1,SHARP ~0x02 __endasm; } \
  { \
    func; \
  } \
  if(eie1_backup & 0x02){ \
    {__asm orl _EIE1,SHARP 0x02 __endasm; } \
  } \
}
#else
#define CRITICAL_USB0_SPECIAL(func) \
{ \
  BYTE eie1_backup = EIE1; \
  EIE1 &= ~(0x02); \
  { \
    func; \
  } \
  if(eie1_backup & 0x02){EIE1 |= 0x02;} \
}
#endif

#ifndef min
#define min(x, y) (((x) < (y)) ? (x) : (y))
#endif
#ifndef max
#define max(x, y) (((x) > (y)) ? (x) : (y))
#endif

/**
 * Read from the selected endpoint FIFO
 * 
 * @param ptr_data read data destination
 * @param index target EP index
 * @param u_num_bytes number of bytes to unload
 * @return number to read
 */
static unsigned int fifo_read_C(
    BYTE* ptr_data, 
    unsigned int u_num_bytes,
    unsigned char index 
){
  
  if(index){u_num_bytes = min(u_num_bytes, count_ep_out(index));}
  
  // Check if >0 bytes requested,
  if(u_num_bytes){
    
    unsigned int read_count;
  
    // Set address
    USB0ADR = (FIFO_EP0 + index);
    
    // Set auto-read and initiate first read
    USB0ADR |= 0xC0;       
  
    // Wait for BUSY->'0' (data ready)
    while(USB0ADR & 0x80);
  
    // Unload <NumBytes> from the selected FIFO
    if(ptr_data){
      
      for(read_count = u_num_bytes - 1; read_count; read_count--){
        // Copy data byte
        *(ptr_data++) = USB0DAT;
        
        // Wait for BUSY->'0' (data ready)
        while(USB0ADR & 0x80);
      }
      
      // Clear auto-read
      USB0ADR &= ~0x40;
      
      // Copy data byte
      *(ptr_data++) = USB0DAT;
    }else{
      volatile BYTE c;
      for(read_count = u_num_bytes - 1; read_count; read_count--){
        
        // Copy data byte
        c = USB0DAT;
        
        // Wait for BUSY->'0' (data ready)
        while(USB0ADR & 0x80);              
      }
    }
    
    USB0ADR = 0;
  }
  
  return u_num_bytes;
}
#define fifo0_read_C(ptr_data, u_num_bytes) \
fifo_read_C(ptr_data, u_num_bytes, 0)

typedef struct {
  BYTE *array;
  BYTE c;
} write_target_t;

/**
 * Write to the selected endpoint FIFO
 * 
 * @param ptr_data read data destination
 * @param u_num_bytes number of bytes to unload
 * @param addr target address
 * @return number to write
 */
static unsigned int fifo_write_C(
  write_target_t *target,
  unsigned int u_num_bytes,
  unsigned char index
){
  
  unsigned int target_bytes = min(u_num_bytes, ep_size[index]);
  
  // If >0 bytes requested,
  if(target_bytes){
      
    // Wait for BUSY->'0' (register available)
    while(USB0ADR & 0x80);
    
    // Set address (mask out bits7-6)
    USB0ADR = (FIFO_EP0 + index);
  
    if(target->array){
      // Write <NumBytes> to the selected FIFO
      for(u_num_bytes = target_bytes; u_num_bytes; u_num_bytes--){
        USB0DAT = *((target->array)++);
      }
    }else{
      // Write <NumBytes> to the selected FIFO
      for(u_num_bytes = target_bytes; u_num_bytes; u_num_bytes--){
        USB0DAT = target->c;
      }
    }
  }
  
  return target_bytes;
}
#define fifo0_write_C(target, u_num_bytes) \
fifo_write_C(target, u_num_bytes, 0)

/**
 * Handle Endpoints whose dircetion is IN.
 * 
 * @param ep_index Index of Endpoint
 */
static void usb_handle_in(unsigned char ep_index){
  
  BYTE control_reg;

  // Set index to EP ep_index
  POLL_WRITE_BYTE(INDEX, ep_index);
  
  // Read contol register for EP ep_index
  POLL_READ_BYTE(EINCSR1, control_reg);

  // Clear sent stall and flush FIFO if last packet returned a stall
  if(control_reg & rbInSTSTL){
    __code void (*callback_func)()
        = stall_callbacks[EP_INDEX_2_LINER_INDEX(DIR_IN, ep_index)];
    POLL_WRITE_BYTE(EINCSR1, 
        (control_reg & (rbInCLRDT | ~rbInSTSTL)));
    if(callback_func){callback_func();}
  }
}

/**
 * Handle Endpoints whose dircetion is IN.
 * 
 * @param ep_index Index of Endpoint
 * 
 */
static void usb_handle_out(unsigned char ep_index){
  BYTE control_reg;

  // Set index
  POLL_WRITE_BYTE(INDEX, ep_index);
  POLL_READ_BYTE(EOUTCSR1, control_reg);
  
  // Clear sent stall bit if last packet was a stall
  if(control_reg & rbOutSTSTL){
    __code void (*callback_func)()
        = stall_callbacks[EP_INDEX_2_LINER_INDEX(DIR_OUT, ep_index)]; 
    POLL_WRITE_BYTE(EOUTCSR1, 
        (control_reg & (rbOutCLRDT | ~rbOutSTSTL)));
    if(callback_func){callback_func();}
  }
  
  // If endpoint is halted, send a stall
  else if(ep_strip_owner(usb_ep_status(DIR_OUT, ep_index)) == EP_HALT){
    POLL_WRITE_BYTE(EOUTCSR1, rbOutSDSTL);
  }

  // Otherwise read received packet from host
  else{
    POLL_READ_BYTE(EOUTCNTH, *(((BYTE *)&(ep_out_stored[ep_index - 1])) + 1));
    POLL_READ_BYTE(EOUTCNTL, *((BYTE *)&(ep_out_stored[ep_index - 1])));
  }
}

/**
 * Decode Incoming Setup requests
 * Load data packets on fifo while in transmit mode
 * 
 */
static void handle_setup(){
  // Temporary storage for EP control register
  BYTE control_reg;

  // Set Index to Endpoint Zero
  POLL_WRITE_BYTE(INDEX, EP0_IDX);
  
  // Read control register
  POLL_READ_BYTE(E0CSR, control_reg);

  // Handle Status Phase of Set Address command
  if(usb_ep0_status == EP_ADDRESS){
    POLL_WRITE_BYTE(FADDR, usb_setup_buf.wValue.c[LSB]);
    usb_ep0_status = EP_IDLE;
  }

  // If last packet was a sent stall, reset STSTL
  // bit and return EP0 to idle state
  if (control_reg & rbSTSTL){
    POLL_WRITE_BYTE(E0CSR, 0);
    usb_ep0_status = EP_IDLE;
    return;
  }

  // If last setup transaction was ended prematurely then set
  if(control_reg & rbSUEND){
    POLL_WRITE_BYTE(E0CSR, rbDATAEND);
    
    // Serviced Setup End bit and return EP0 to idle state
    POLL_WRITE_BYTE(E0CSR, rbSSUEND);
    usb_ep0_status = EP_IDLE;
  }

  // If Endpoint 0 is in idle mode
  if(usb_ep0_status == EP_IDLE){
    
    // Make sure that EP 0 has an Out Packet ready from host
    // although if EP0 is idle, this should always be the case
    if(control_reg & rbOPRDY){                                 
      
      fifo0_read_C((BYTE *)&usb_setup_buf, 8);
      
      usb_request_completed = FALSE;
      
      // Call correct subroutine to handle each kind of standard request
      switch(usb_setup_buf.bmRequestType & DRT_MASK){
        // Standard device request
        case DRT_STD :
          usb_Std_Req();
          break;
        case DRT_CLASS :
          usb_Class_Req();
          break;
        case DRT_VENDOR :
          usb_Vendor_Req();
          break;
      }
      
      // Set index back to endpoint 0
      POLL_WRITE_BYTE(INDEX, EP0_IDX);
      
      if(!usb_request_completed){
        // Send stall to host if invalid request
        POLL_WRITE_BYTE(E0CSR, rbSDSTL);
        // Put the endpoint in stall status
        usb_ep0_status = EP_STALL;
        return;
      }
      
      // Indicate setup packet has been serviced
      control_reg = rbSOPRDY;
      if(usb_ep0_status == EP_IDLE){
        control_reg |= rbDATAEND;
      }
      POLL_WRITE_BYTE(E0CSR, control_reg);
    }
  } 

  // See if the endpoint has data to transmit to host
  switch(usb_ep0_status){
    case EP_TX: {
      // Make sure you don't overwrite last packet
      if (control_reg & rbINPRDY){break;}
        
      // Read control register
      POLL_READ_BYTE(E0CSR, control_reg);          
     
      // Check to see if Setup End or Out Packet received, if so                            
      // do not put any new data on FIFO
      if((control_reg & rbSUEND) && (control_reg & rbOPRDY)){break;}
              
      // Add In Packet ready flag to E0CSR bitmask
      control_reg = rbINPRDY;
     
      {
        write_target_t target;
        target.array = ep0_Data_Ptr;
        if(ep0_Data_Size -= fifo0_write_C(&target, ep0_Data_Size)){
          
          // Advance data pointer
          ep0_Data_Ptr += PACKET_SIZE_EP0;
        }else if(ep0_callback){
          ep0_callback();
        }else{
          
          // Add Data End bit to bitmask
          control_reg |= rbDATAEND;
        
          // Return EP 0 to idle state
          usb_ep0_status = EP_IDLE;
        }
      }
      
      // Write mask to E0CSR
      POLL_WRITE_BYTE(E0CSR, control_reg);
      
      break;
    }
    case EP_RX: {
      // Verify packet was received
      if(control_reg & rbOPRDY){
        
        BYTE recieved;
        
        control_reg = rbSOPRDY;
  
        // get data bytes on the FIFO
        POLL_READ_BYTE(E0CNT, recieved);
        
        // Empty the FIFO
        // FIFO must be read out just the size it has,
        // otherwize the FIFO pointer on the SIE goes out of synch.
        fifo0_read_C((BYTE*)ep0_Data_Ptr, recieved);
        
        if(ep0_Data_Size > recieved){
          // Update the scheduled number to be received
          ep0_Data_Size -= recieved;
          
          // Advance the pointer
          ep0_Data_Ptr += recieved;
        }else{
          // Meet the end of the data stage
          ep0_Data_Size = 0;
          
          // Signal end of data stage
          control_reg |= rbDATAEND;
          
          usb_ep0_status = EP_IDLE;
  
          ep0_callback();
        }
  
        POLL_WRITE_BYTE(E0CSR, control_reg);
      }
      break;
    }
  }
}

/**
 * Resume normal USB operation
 * 
 * Add code to turn on anything turned off when entering suspend mode
 */
static void resume(){ 
  
}

/**
 * Enter suspend mode after suspend signalling is present on the bus
 * 
 * Add power-down features here if you wish to
 * reduce power consumption during suspend mode
 */
static void suspend(){
  
}

/**
 * Set state to default
 * Clear USB Inhibit bit
 */
static void reset(){
  // Set device state to default
  usb_state = DEV_DEFAULT;
  
  // Set default Endpoint Status
  usb_ep0_status = EP_IDLE;
  
  // clear EP1-3 states, stall callback funcs, and out counters
  {
    unsigned char ep_index;
    for(ep_index = 1; ep_index <= 3; ep_index++){
      usb_ep_stat[ep_index] = EP_HALT;
      stall_callbacks[EP_INDEX_2_LINER_INDEX(DIR_IN, ep_index)] = NULL;
      stall_callbacks[EP_INDEX_2_LINER_INDEX(DIR_OUT, ep_index)] = NULL;
      count_ep_out(ep_index) = 0;
    }
  }
  
  // Clear USB inhibit bit to enable USB suspend detection
  POLL_WRITE_BYTE(POWER, 0x01);
  
  // initialize class
  usb_Class_init();
}

/**
 * USB Initialization
 *
 * Initialize USB0
 * Enable USB0 interrupts
 * Enable USB0 transceiver
 * Enable USB0 with suspend detection
 */
void usb0_init(){
  usb_sof = NULL;
  POLL_WRITE_BYTE(POWER,  0x08); // Force Asynchronous USB Reset
  POLL_WRITE_BYTE(IN1IE,  0x0F); // Enable Endpoint 0-3 in interrupts
  POLL_WRITE_BYTE(OUT1IE, 0x0F); // Enable Endpoint 0-3 out interrupts
  POLL_WRITE_BYTE(CMIE,   0x07); // Enable Reset, Resume, and Suspend interrupts
#ifdef _USB_LOW_SPEED_
  USB0XCN = 0xC0;                // Enable transceiver; select low speed
  POLL_WRITE_BYTE(CLKREC, 0xA0); // Enable clock recovery; single-step mode disabled; low speed mode enabled
#else
  USB0XCN = 0xE0;                // Enable transceiver; select full speed
  POLL_WRITE_BYTE(CLKREC, 0x80); // Enable clock recovery, single-step mode disabled
#endif /* _USB_LOW_SPEED_ */

  reset();
  usb_mode = USB_INACTIVE;
}

void (* __xdata usb_sof)();

/**
 * Top-level USB ISR
 * 
 * Called after any USB type interrupt, this handler determines which type
 * of interrupt occurred, and calls the specific routine to handle it.
 * 
 */
void usb_isr() __interrupt (INTERRUPT_USB0) {
  BYTE bCommon, bIn, bOut;
  POLL_READ_BYTE(CMINT, bCommon);      // Read all interrupt registers
  POLL_READ_BYTE(IN1INT, bIn);         // this read also clears the register
  POLL_READ_BYTE(OUT1INT, bOut);

  // Handle Resume interrupt
  if(bCommon & rbRSUINT){resume();}
    
  // Handle Setup packet received
  // or packet transmitted if Endpoint 0 is transmit mode 
  if(bIn & rbEP0){handle_setup();}
  
  {
    unsigned char ep_index, ep_mask = 0x02;
    for(ep_index = 1; ep_index <= 3; ep_index++, ep_mask <<= 1){
      
      // Handle In packet received
      if(bIn & ep_mask){usb_handle_in(ep_index);}
      
      // Handle Out packet received, take data off OUT endpoint
      if(bOut & ep_mask){usb_handle_out(ep_index);}
    }
  }
  
  // Handle Start of Frame interrupt
  if(bCommon & rbSOF){
    if(usb_sof){usb_sof();}
  }
  
  // Handle Suspend interrupt
  if(bCommon & rbSUSINT){suspend();}
  
  // Handle Reset interrupt
  if(bCommon & rbRSTINT){reset();}
}

volatile usb_mode_t usb_mode;

static unsigned int usb_tx(
  write_target_t *target,
  unsigned int count,
  unsigned char index
){
  BYTE *ptr_buf = target->array;
  unsigned int count_orig = count, add;
  do{
    add = 0;
    CRITICAL_USB0_SPECIAL(      
      BYTE control_reg;
  
      // Set index to EP ep_index
      POLL_WRITE_BYTE(INDEX, index);
      
      // If endpoint is halted, send a stall
      if(ep_strip_owner(usb_ep_status(DIR_IN, index)) == EP_HALT){
        POLL_WRITE_BYTE(EINCSR1, rbInSDSTL);
      
      }else{
        
        // Read contol register for EP ep_index
        POLL_READ_BYTE(EINCSR1, control_reg);
        
        // Regist data for transmition
        if(!(control_reg & rbInINPRDY)){
          
          // Clear underrun bit if it was set
          if(control_reg & rbInUNDRUN){
            POLL_WRITE_BYTE(EINCSR1, 0x00);
          }
      
          // Put new data on Fifo
          if(add = fifo_write_C(target, count, index)){
            // Set In Packet ready bit, indicating fresh data on Fifo
            POLL_WRITE_BYTE(EINCSR1, rbInINPRDY);
          }
        }
      }
    );
    
    if(!add){break;}
    if(ptr_buf){
      ptr_buf += add;
      target->array = ptr_buf;
    }
    count -= add;
  }while(count);
  return count_orig - count;
}

unsigned int usb_write(
  BYTE* ptr_buf,
  unsigned int count,
  unsigned char index
){
  write_target_t target;
  target.array = ptr_buf;  
  return usb_tx(&target, count, index);
}

unsigned int usb_fill(
  BYTE c,
  unsigned int count,
  unsigned char index
){
  write_target_t target = {NULL, c};
  return usb_tx(&target, count, index);
}

unsigned int usb_count_ep_out(unsigned char index){
  unsigned int res;
  CRITICAL_USB0_SPECIAL(
    res = count_ep_out(index);
  );
  return res;
}

unsigned int usb_read(
  BYTE* ptr_buf,
  unsigned int count,
  unsigned char index
){
  unsigned int read_count = fifo_read_C(ptr_buf, count, index);
  count_ep_out(index) -= read_count;
  if(!(count_ep_out(index))){
    CRITICAL_USB0_SPECIAL(
      POLL_WRITE_BYTE(INDEX, index);
      POLL_WRITE_BYTE(EOUTCSR1, 0);
    );
  }
  return read_count;
}

unsigned int usb_skip(
  unsigned int count,
  unsigned char index
){
  return usb_read(NULL, count, index);
}

void usb_flush(unsigned char index){
  // Clear Out Packet ready bit
  usb_read(NULL, count_ep_out(index), index);
}

void usb_status_lock(unsigned char dir, unsigned char ep_index){
  CRITICAL_USB0_SPECIAL(
    usb_ep_status(dir, ep_index) |= EP_OWNED_BY_USER;
  );
}

void usb_status_unlock(unsigned char dir, unsigned char ep_index){
  CRITICAL_USB0_SPECIAL(
    usb_ep_status(dir, ep_index) &= ~EP_OWNED_BY_USER;
  );
}

void usb_stall(unsigned char dir, unsigned char ep_index,  
    __code void(*callback)()){
  CRITICAL_USB0_SPECIAL(
    usb_ep_status(dir, ep_index) 
        = (usb_ep_status(dir, ep_index) & EP_OWNED_BY_USER) | EP_HALT;
  
    // Set index to EP ep_index
    POLL_WRITE_BYTE(INDEX, ep_index);
    
    // send a stall immediately
    if(dir == DIR_IN){
      POLL_WRITE_BYTE(EINCSR1, rbInSDSTL);
      stall_callbacks[EP_INDEX_2_LINER_INDEX(DIR_IN, ep_index)] = callback;
    }else{
      POLL_WRITE_BYTE(EOUTCSR1, rbOutSDSTL);
      stall_callbacks[EP_INDEX_2_LINER_INDEX(DIR_OUT, ep_index)] = callback;
    }
  );
}

void usb_clear_halt(unsigned char dir, unsigned char ep_index){
  CRITICAL_USB0_SPECIAL(
    usb_ep_status(dir, ep_index) 
        = (usb_ep_status(dir, ep_index) & EP_OWNED_BY_USER) | EP_IDLE;
    
    // Set index to EP ep_index
    POLL_WRITE_BYTE(INDEX, ep_index);
    
    // clear flag
    if(dir == DIR_IN){
      POLL_WRITE_BYTE(EINCSR1, 0);
    }else{
      POLL_WRITE_BYTE(EOUTCSR1, 0);
    }
  );
}

void usb_polling(){
  
  switch(usb_mode){
    case USB_INACTIVE:
      if(REG01CN & 0x40){
        usb_mode = USB_CABLE_CONNECTED;
        EIE1 |= 0x02; // Enable USB0 Interrupts
      }
      return;
    case USB_CABLE_CONNECTED:
      break;
    case USB_CDC_READY:
      usb_mode = USB_CDC_ACTIVE;
      break;
    case USB_CDC_ACTIVE:
      cdc_polling();
      break;
    case USB_MSD_READY:
      usb_mode = USB_MSD_ACTIVE;
      break;
    case USB_MSD_ACTIVE:
      msd_polling();
      break;
  }
  
  if(!(REG01CN & 0x40)){
    EIE1 &= ~0x02;  // Disable USB0 Interrupts
    usb_mode = USB_INACTIVE;
  }
}
