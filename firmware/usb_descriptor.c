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

#include "main.h"
#include "usb_descriptor.h"

#include "usb_cdc.h"
#include "usb_msd.h"

#define DESC_DEVICE_bcdUSB        0x0200
#define DESC_DEVICE_bcdDevice     0x0000
#define DESC_DEVICE_iManufacturer 0x01
#define DESC_DEVICE_iProduct      0x02
#define DESC_DEVICE_iSerialNumber 0x03
#define DESC_DEVICE_idVendor      0x10C4 

const device_descriptor_t DESC_DEVICE = {
  sizeof(device_descriptor_t),   // bLength(0x12)
  DSC_TYPE_DEVICE,                // bDescriptorType
  {DESC_DEVICE_bcdUSB},           // bcdUSB
#ifndef DEBUG_MSD_ONLY
#if !defined(DEBUG_CDC_ONLY) || defined(USE_IAD)
  0xEF,                           // bDeviceClass
  0x02,                           // bDeviceSubClass
  0x01,                           // bDeviceProtocol
#else
  0x02,                           // bDeviceClass (Communication Class)
  0x00,                           // bDeviceSubClass
  0x00,                           // bDeviceProtocol
#endif
#else
  0x00,                           // bDeviceClass (defined at interface level)
  0x00,                           // bDeviceSubClass
  0x00,                           // bDeviceProtocol
#endif
  PACKET_SIZE_EP0,                // bMaxPacketSize0
  {DESC_DEVICE_idVendor},         // idVendor
#ifndef DEBUG_MSD_ONLY
#ifndef DEBUG_CDC_ONLY
#ifndef CDC_IS_REPLACED_BY_FTDI
  {0x0199},                       // idProduct(M+C)
#else
  {0x0197},                       // idProduct(M+C(FTDI))
#endif
#else 
  {0x0198},                       // idProduct(C)
#endif
#else
  {0x0200},                       // idProduct(M)
#endif
#ifndef CDC_IS_REPLACED_BY_FTDI
  {DESC_DEVICE_bcdDevice},        // bcdDevice
#else  
  {0x0600},                       // bcdDevice(FT232R)
#endif
  DESC_DEVICE_iManufacturer,      // iManufacturer
  DESC_DEVICE_iProduct,           // iProduct
  DESC_DEVICE_iSerialNumber,      // iSerialNumber
  0x01                            // bNumConfigurations
}; //end of DEVICE

#define DESC_CONFIG_bConfigurationValue 0x01                               
#define DESC_CONFIG_iConfiguration      0x00
#define DESC_CONFIG_bmAttributes        0x80
#define DESC_CONFIG_MaxPower            0x0F

const configuration_descriptor_t DESC_CONFIG = {
  sizeof(configuration_descriptor_t),// Length(0x09)
  DSC_TYPE_CONFIG,                    // Type
#ifndef DEBUG_MSD_ONLY
#ifndef DEBUG_CDC_ONLY
#ifndef CDC_IS_REPLACED_BY_FTDI
  {config_length_total(3, 5)          // + ((interface : 3) + (endpoint : 5))
    + cdc_config_length()             // + cdc
    + iad_length()},                  // + iad           
  0x03,                               // NumInterfaces(M+C)
#else
  {config_length_total(2, 4)          // + ((interface : 2) + (endpoint : 4))
    + iad_length()},                  // + iad           
  0x02,                               // NumInterfaces(M+C(FTDI))
#endif
#else
  {
    config_length_total(2, 3)         // + ((interface : 2) + (endpoint : 3))
      + cdc_config_length()           // + cdc
#ifdef USE_IAD
      + iad_length()                  // + iad
#endif
  },
  0x02,                               // NumInterfaces(C)
#endif
#else
  {config_length_total(1, 2)},
  0x01,                               // NumInterfaces(M)
#endif
  DESC_CONFIG_bConfigurationValue,    // bConfigurationValue
  DESC_CONFIG_iConfiguration,         // iConfiguration
  DESC_CONFIG_bmAttributes,           // bmAttributes
  DESC_CONFIG_MaxPower                // MaxPower
}; //end of CONFIG

#ifndef DEBUG_MSD_ONLY
#if !defined(DEBUG_CDC_ONLY) || defined(USE_IAD) 
const interface_association_descriptor_t DESC_IASSOC1 = {
  sizeof(interface_association_descriptor_t),
  DSC_TYPE_IASSOC,        // Descriptor Type
  0x00,                   // bFirstInterface
#ifndef CDC_IS_REPLACED_BY_FTDI
  0x02,                   // bInterfaceCount 
  0x02,                   // bFunctionClass (Communication Class)
  0x00,                   // bFunctionSubClass 
  0x00,                   // bFunctionProtocol
#else
  0x01,                   // bInterfaceCount 
  0x00,                   // bFunctionClass
  0x00,                   // bFunctionSubClass 
  0x00,                   // bFunctionProtocol
#endif
  0x00                    // iInterface
};
#endif

#ifndef CDC_IS_REPLACED_BY_FTDI
const interface_descriptor_t DESC_INTERFACE1 = { // Communication Class
  sizeof(interface_descriptor_t),// bLength
  DSC_TYPE_INTERFACE,             // bDescriptorType
  0x00,                           // bInterfaceNumber
  0x00,                           // bAlternateSetting
  0x01,                           // bNumEndpoints
  0x02,                           // bInterfaceClass (Communication Class)
  0x02,                           // bInterfaceSubClass (Abstract Control Model)
  0x01,                           // bInterfaceProcotol (V.25ter, Common AT commands)
  0x00                            // iInterface
}; // end of INTERFACE1

const cdc_descriptor_t DESC_CDC = {
  {   // Header Functional Descriptor
    sizeof(cdc_header_func_descriptor_t),
    DSC_TYPE_CS_INTERFACE,        // bDescriptorType (CS_INTERFACE)
    DSC_SUBTYPE_CS_HEADER_FUNC,   // bDescriptorSubtype (Header Functional)
    {0x0110}                      // bcdCDC (CDC spec release number, 1.1)
  },
  {   // Abstract Control Management Functional Descriptor
    sizeof(cdc_abst_control_mana_descriptor_t),
    DSC_TYPE_CS_INTERFACE,        // bDescriptorType (CS_INTERFACE)
    DSC_SUBTYPE_CS_ABST_CNTRL,    // bDescriptorSubtype (Abstract Control Management)
    0x06                          // bmCapabilities (Supports Send_Break, Set_Line_Coding, Set_Control_Line_State, Get_Line_Coding, and the notification Serial_State)
  },
  {   // Union Functional Descriptor
    sizeof(cdc_union_func_descriptor_t),
    DSC_TYPE_CS_INTERFACE,        // bDescriptorType (CS_INTERFACE)
    DSC_SUBTYPE_CS_UNION_FUNC,    // bDescriptorSubtype (Union Functional)
    0x00,                         // bMasterInterface (Interface number master interface in the union)
    0x01                          // bSlaveInterface0 (Interface number slave interface in the union)
  },
  {   // Call Management Functional Descriptor
    sizeof(cdc_call_man_func_descriptor_t),
    DSC_TYPE_CS_INTERFACE,        // bDescriptorType (CS_INTERFACE)
    DSC_SUBTYPE_CS_CALL_MAN,      // bDescriptorSubtype (Call Management)
    0x01,                         // bmCapabilities (only over Communication Class IF / handles itself)
    0x01                          // bDataInterface (Interface number of Data Class interface)
  }
};

const endpoint_descriptor_t DESC_ENDPOINT1 = {
  sizeof(endpoint_descriptor_t), // bLength
  DSC_TYPE_ENDPOINT,              // bDescriptorType
  IN_EP1,                         // bEndpointAddress
  DSC_EP_INTERRUPT,               // bmAttributes
  {PACKET_SIZE_EP1},              // MaxPacketSize
  1                               // bInterval
}; //end of ENDPOINT1
#endif

const interface_descriptor_t DESC_INTERFACE2 = { // Data Interface Class
  sizeof(interface_descriptor_t),// bLength
  DSC_TYPE_INTERFACE,             // bDescriptorType
#ifndef CDC_IS_REPLACED_BY_FTDI
  0x01,                           // bInterfaceNumber
#else
  0x00,                           // bInterfaceNumber
#endif
  0x00,                           // bAlternateSetting
  0x02,                           // bNumEndpoints
#ifndef CDC_IS_REPLACED_BY_FTDI
  0x0A,                           // bInterfaceClass (Data Interface Class)
  0x00,                           // bInterfaceSubClass
  0x00,                           // bInterfaceProcotol (No class specific protocol required)
#else
  0xFF,                           // bInterfaceClass (Data Interface Class)
  0xFF,                           // bInterfaceSubClass
  0xFF,                           // bInterfaceProcotol (No class specific protocol required)
#endif
  0x00                            // iInterface
}; //end of INTERFACE2

const endpoint_descriptor_t DESC_ENDPOINT2 = {
  sizeof(endpoint_descriptor_t), // bLength
  DSC_TYPE_ENDPOINT,              // bDescriptorType
  IN_EP2,                         // bEndpointAddress
  DSC_EP_INTERRUPT,               // bmAttributes
  {PACKET_SIZE_EP2},              // MaxPacketSize
  1                               // bInterval
}; //end of ENDPOINT2

const endpoint_descriptor_t DESC_ENDPOINT3 = {
  sizeof(endpoint_descriptor_t), // bLength
  DSC_TYPE_ENDPOINT,              // bDescriptorType
  OUT_EP2,                        // bEndpointAddress
  DSC_EP_BULK,                    // bmAttributes
  {PACKET_SIZE_EP2},              // MaxPacketSize
  0                               // bInterval
}; //end of ENDPOINT3
#endif

#ifndef DEBUG_CDC_ONLY
const interface_descriptor_t DESC_INTERFACE3 = {
  sizeof(interface_descriptor_t),  // bLength
  DSC_TYPE_INTERFACE,               // bDescriptorType
#ifndef DEBUG_MSD_ONLY
#ifndef CDC_IS_REPLACED_BY_FTDI
  0x02,                             // bInterfaceNumber
#else
  0x01,                             // bInterfaceNumber
#endif
#else
  0x00,                             // (for MSD only)
#endif
  0x00,                             // bAlternateSetting
  0x02,                             // bNumEndpoints
  0x08,                             // bInterfaceClass // MASS STORAGE DEVICE 
  0x06,                             // bInterfaceSubClass // SCSI Transparent command set
  0x50,                             // bInterfaceProcotol // BULK-ONLY transport
  0x00                              // iInterface
}; //end of INTERFACE3

const endpoint_descriptor_t DESC_ENDPOINT4 = {
  sizeof(endpoint_descriptor_t), // bLength
  DSC_TYPE_ENDPOINT,              // bDescriptorType
  IN_EP3,                         // bEndpointAddress
  DSC_EP_BULK,                    // bmAttributes
  {PACKET_SIZE_EP3},              // MaxPacketSize
  0                               // bInterval
}; //end of ENDPOINT4

const endpoint_descriptor_t DESC_ENDPOINT5 = {
  sizeof(endpoint_descriptor_t), // bLength
  DSC_TYPE_ENDPOINT,              // bDescriptorType
  OUT_EP3,		                    // bEndpointAddress
  DSC_EP_BULK,                    // bmAttributes
  {PACKET_SIZE_EP3},              // MaxPacketSize
  0                               // bInterval
}; //end of ENDPOINT5
#endif

#define STR0LEN 4
__code const BYTE DESC_STRING0[STR0LEN] = {
  STR0LEN, 0x03, 0x09, 0x04
}; //end of String0_Desc

#define STR1LEN sizeof("Silicon Laboratories")*2

__code const BYTE DESC_STRING1[STR1LEN] = {
  STR1LEN, 0x03,
  'S', 0,
  'i', 0,
  'l', 0,
  'i', 0,
  'c', 0,
  'o', 0,
  'n', 0,
  ' ', 0,
  'L', 0,
  'a', 0,
  'b', 0,
  'o', 0,
  'r', 0,
  'a', 0,
  't', 0,
  'o', 0,
  'r', 0,
  'i', 0,
  'e', 0,
  's', 0,
}; //end of String1_Desc

#define STR2LEN sizeof("C8051Fxxx Development Board")*2

__code const BYTE DESC_STRING2[STR2LEN] = {
  STR2LEN, 0x03,
  'C', 0,
  '8', 0,
  '0', 0,
  '5', 0,
  '1', 0,
  'F', 0,
  'x', 0,
  'x', 0,
  'x', 0,
  ' ', 0,
  'D', 0,
  'e', 0,
  'v', 0,
  'e', 0,
  'l', 0,
  'o', 0,
  'p', 0,
  'm', 0,
  'e', 0,
  'n', 0,
  't', 0,
  ' ', 0,
  'B', 0,
  'o', 0,
  'a', 0,
  'r', 0,
  'd', 0
}; //end of String2_Desc

#define STR3LEN sizeof("0079876543210")*2

__code const BYTE DESC_STRING3[STR3LEN] = {
  STR3LEN, 0x03,
  '0', 0,
  '0', 0,
  '7', 0,
  '9', 0,
  '8', 0,
  '7', 0,
  '6', 0,
  '5', 0,
  '4', 0,
  '3', 0,
  '2', 0,
  '1', 0,
  '0', 0
}; //end of String2_Desc

// code上に作ったcode pointerの配列参照をしようとすると、なぜか配列がidataに存在するようにコンパイルされる
// xdata上に作ってもidataを参照してしまう
// ･･･と思ったら、USB_Std_Req.cにexternがあることに気づかなかった
__code BYTE * __code DESC_STRINGs[4] = {
  DESC_STRING0,
  DESC_STRING1,
  DESC_STRING2,
  DESC_STRING3
};


// SDをはずした際はCDCデバイスのみとして機能するようにする

#if (!defined(DEBUG_MSD_ONLY)) && (!defined(DEBUG_CDC_ONLY))

//---------------------------
// Descriptor Declarations 2
//---------------------------
const device_descriptor_t DESC2_DEVICE = {
  sizeof(device_descriptor_t),   // bLength(0x12)
  DSC_TYPE_DEVICE,                // bDescriptorType
  {DESC_DEVICE_bcdUSB},           // bcdUSB
#ifndef CDC_IS_REPLACED_BY_FTDI
  0x02,                           // bDeviceClass (Communication Class)
  0x00,                           // bDeviceSubClass
  0x00,                           // bDeviceProtocol
#else
  0x00,                           // bDeviceClass (defined at interface level)
  0x00,                           // bDeviceSubClass
  0x00,                           // bDeviceProtocol
#endif
  PACKET_SIZE_EP0,                // bMaxPacketSize0
  {DESC_DEVICE_idVendor},         // idVendor
#ifndef CDC_IS_REPLACED_BY_FTDI
  {0x0198},                       // idProduct(C)
  {DESC_DEVICE_bcdDevice},        // bcdDevice
#else
  {0x0196},                       // idProduct(C(FTDI))
  {0x0600},                       // bcdDevice(FT232R)
#endif
  DESC_DEVICE_iManufacturer,      // iManufacturer
  DESC_DEVICE_iProduct,           // iProduct
  DESC_DEVICE_iSerialNumber,      // iSerialNumber
  0x01                            // bNumConfigurations
}; //end of DEVICE

const configuration_descriptor_t DESC2_CONFIG = {
  sizeof(configuration_descriptor_t),// Length(0x09)
  DSC_TYPE_CONFIG,                    // Type
#ifndef CDC_IS_REPLACED_BY_FTDI
  {config_length_total(2, 3)          // + ((interface : 2) + (endpoint : 3))
    + cdc_config_length()},           // + cdc
  0x02,                               // NumInterfaces(C)
#else
  {config_length_total(1, 2)},        // + ((interface : 1) + (endpoint : 2))           
  0x01,                               // NumInterfaces(C(FTDI))
#endif
  DESC_CONFIG_bConfigurationValue,    // bConfigurationValue
  DESC_CONFIG_iConfiguration,         // iConfiguration
  DESC_CONFIG_bmAttributes,           // bmAttributes
  DESC_CONFIG_MaxPower                // MaxPower
}; //end of CONFIG

#ifndef CDC_IS_REPLACED_BY_FTDI
const interface_descriptor_t DESC2_INTERFACE1 = { // Communication Class
  sizeof(interface_descriptor_t),// bLength
  DSC_TYPE_INTERFACE,             // bDescriptorType
  0x00,                           // bInterfaceNumber
  0x00,                           // bAlternateSetting
  0x01,                           // bNumEndpoints
  0x02,                           // bInterfaceClass (Communication Class)
  0x02,                           // bInterfaceSubClass (Abstract Control Model)
  0x01,                           // bInterfaceProcotol (V.25ter, Common AT commands)
  0x00                            // iInterface
}; // end of INTERFACE1

const code cdc_descriptor_t DESC2_CDC = DESC_CDC; 

const code endpoint_descriptor_t DESC2_ENDPOINT1 = DESC_ENDPOINT1; 
#endif

const interface_descriptor_t DESC2_INTERFACE2 = { // Data Interface Class
  sizeof(interface_descriptor_t),// bLength
  DSC_TYPE_INTERFACE,             // bDescriptorType
#ifndef CDC_IS_REPLACED_BY_FTDI
  0x01,                           // bInterfaceNumber
#else
  0x00,                           // bInterfaceNumber
#endif
  0x00,                           // bAlternateSetting
  0x02,                           // bNumEndpoints
#ifndef CDC_IS_REPLACED_BY_FTDI
  0x0A,                           // bInterfaceClass (Data Interface Class)
  0x00,                           // bInterfaceSubClass
  0x00,                           // bInterfaceProcotol (No class specific protocol required)
#else
  0xFF,                           // bInterfaceClass (Data Interface Class)
  0xFF,                           // bInterfaceSubClass
  0xFF,                           // bInterfaceProcotol (No class specific protocol required)
#endif
  0x00                            // iInterface
}; //end of INTERFACE2

const endpoint_descriptor_t DESC2_ENDPOINT2 = {
  sizeof(endpoint_descriptor_t), // bLength
  DSC_TYPE_ENDPOINT,              // bDescriptorType
  IN_EP2,                         // bEndpointAddress
  DSC_EP_BULK,                    // bmAttributes
  {PACKET_SIZE_EP2},              // MaxPacketSize
  0                               // bInterval
}; //end of ENDPOINT2_2

const endpoint_descriptor_t DESC2_ENDPOINT3 = {
  sizeof(endpoint_descriptor_t), // bLength
  DSC_TYPE_ENDPOINT,              // bDescriptorType
  OUT_EP2,                        // bEndpointAddress
  DSC_EP_BULK,                    // bmAttributes
  {PACKET_SIZE_EP2},              // MaxPacketSize
  0                               // bInterval
}; //end of ENDPOINT2_3

#endif
