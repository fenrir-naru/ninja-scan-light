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
