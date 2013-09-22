#ifndef __CDC_H__
#define __CDC_H__

#include "type.h"
#define CDC_IS_REPLACED_BY_FTDI

/**
 * Header Functional Descriptor
 */
typedef struct {
  BYTE bLength;
  BYTE bDescriptorType;
  BYTE bDescriptorSubtype;
  WORD bcdCDC;
} cdc_header_func_descriptor_t;

/**
 * Call Management Functional Descriptor
 */
typedef struct {
  BYTE bLength;
  BYTE bDescriptorType;
  BYTE bDescriptorSubtype;
  BYTE bmCapabilities;
  BYTE bDataInterface;
} cdc_call_man_func_descriptor_t;

/**
 * Abstract Control Management Functional Descriptor
 */
typedef struct {
  BYTE bLength;
  BYTE bDescriptorType;
  BYTE bDescriptorSubtype;
  BYTE bmCapabilities;
} cdc_abst_control_mana_descriptor_t;

/**
 * Union Functional Descriptor
 */
typedef struct {
  BYTE bLength;
  BYTE bDescriptorType;
  BYTE bDescriptorSubtype;
  BYTE bMasterInterface;
  BYTE bSlaveInterface0;
} cdc_union_func_descriptor_t;

typedef __code struct {
  cdc_header_func_descriptor_t header;
  cdc_abst_control_mana_descriptor_t abst_control_mana;
  cdc_union_func_descriptor_t union_func;
  cdc_call_man_func_descriptor_t call_ma;
} cdc_descriptor_t;

#define cdc_config_length() sizeof(cdc_descriptor_t)

typedef struct {
  DWORD_t baudrate;
  BYTE stopbit;
  BYTE parity;
  BYTE databit;
} cdc_line_coding_t;

// Descriptor type (Class specific)
#define DSC_TYPE_CS_INTERFACE       0x24
#define DSC_SUBTYPE_CS_HEADER_FUNC  0x00
#define DSC_SUBTYPE_CS_CALL_MAN     0x01
#define DSC_SUBTYPE_CS_ABST_CNTRL   0x02
#define DSC_SUBTYPE_CS_UNION_FUNC   0x06

// CDC ACM class specifc requests
#define SEND_ENCAPSULATED_COMMAND 0x00
#define GET_ENCAPSULATED_RESPONSE 0x01
#define SET_LINE_CODING           0x20
#define GET_LINE_CODING           0x21
#define SET_CONTROL_LINE_STATE    0x22
#define SEND_BREAK                0x23

extern volatile __bit cdc_need_line_status_update;
void usb_CDC_req();
void cdc_polling();
u16 cdc_tx(u8 *buf, u16 size);
u16 cdc_rx(u8 *buf, u16 size);

#define CDC_COM_EP_IN  1
#define CDC_DATA_EP_IN  2
#define CDC_DATA_EP_OUT 2

#ifndef CONCAT2_
#define CONCAT2_(a,b) (a ## b)
#endif
#ifndef CONCAT2
#define CONCAT2(a,b) CONCAT2_(a, b) 
#endif

#define CDC_DATA_EP_IN_PACKET_SIZE  (CONCAT2(PACKET_SIZE_EP, CDC_DATA_EP_IN))
#define CDC_DATA_EP_OUT_PACKET_SIZE  (CONCAT2(PACKET_SIZE_EP, CDC_DATA_EP_OUT))

#endif
