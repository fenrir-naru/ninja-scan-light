#include "main.h"
#include "f38x_usb.h"

#include "usb_other_req.h"
#include "usb_descriptor.h"
//#include "usb_cdc.h"
//#include "usb_msd.h"

void usb_Class_init(){
  usb_MSD_init();
}

void usb_Class_Req(){
  switch(usb_setup_buf.wIndex.i){
    case 0:
#ifndef DEBUG_MSD_ONLY
#ifndef CDC_IS_REPLACED_BY_FTDI
      usb_CDC_req();
      break;
    case 2:
#else
    case 1:
#endif
#endif
      usb_MSD_req();
      break;
  }
}

void usb_Vendor_Req(){
#ifdef CDC_IS_REPLACED_BY_FTDI 
  usb_CDC_req();
#endif
}
