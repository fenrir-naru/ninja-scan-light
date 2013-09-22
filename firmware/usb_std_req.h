#ifndef _USB_STD_REQ_H_
#define _USB_STD_REQ_H_

// Standard Request Codes
#define GET_STATUS          0x00    // Code for Get Status
#define CLEAR_FEATURE       0x01    // Code for Clear Feature
#define SET_FEATURE         0x03    // Code for Set Feature
#define SET_ADDRESS         0x05    // Code for Set Address
#define GET_DESCRIPTOR      0x06    // Code for Get Descriptor
#define SET_DESCRIPTOR      0x07    // Code for Set Descriptor(not used)
#define GET_CONFIGURATION   0x08    // Code for Get Configuration
#define SET_CONFIGURATION   0x09    // Code for Set Configuration
#define GET_INTERFACE       0x0A    // Code for Get Interface
#define SET_INTERFACE       0x0B    // Code for Set Interface
#define SYNCH_FRAME         0x0C    // Code for Synch Frame(not used)

void usb_Std_Req();

#endif /* _USB_STD_REQ_H_ */
