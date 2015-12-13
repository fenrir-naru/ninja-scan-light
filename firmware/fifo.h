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

#ifndef __FIFO_H__
#define __FIFO_H__

#include "main.h"
#include <stddef.h>

#define CONCAT3(str1, str2, str3) str1 ## str2 ## str3
#define CONCAT4(str1, str2, str3, str4) str1 ## str2 ## str3 ## str4

#ifndef FIFO_TYPE
#define FIFO_TYPE char
#endif
#define FIFO_T(type) CONCAT3(fifo_, type, _t)
#define FIFO_METHOD(type, name) CONCAT4(fifo_, type, _, name)

#ifndef FIFO_SIZE_T
#define FIFO_SIZE_T unsigned char
#endif

#ifndef NULL_CHECK
#define NULL_CHECK 0
#endif

#ifndef FIFO_BUFFER_STORAGE
#define FIFO_BUFFER_STORAGE
#endif

typedef struct {
  FIFO_BUFFER_STORAGE FIFO_TYPE *buffer;
  FIFO_SIZE_T size;
  FIFO_BUFFER_STORAGE FIFO_TYPE *prius;
  FIFO_BUFFER_STORAGE FIFO_TYPE *follower;
} FIFO_T(FIFO_TYPE);

/* The rule of pointers in a ring buffer.
 * 1) The "follower" can reach the "prius" , but can not overtake it.
 * 2) The "prius" can not reach the "follower" more than overtake it.
 */

FIFO_SIZE_T FIFO_METHOD(FIFO_TYPE, write) (FIFO_T(FIFO_TYPE) *fifo, FIFO_TYPE *values, FIFO_SIZE_T size);
FIFO_SIZE_T FIFO_METHOD(FIFO_TYPE, put) (FIFO_T(FIFO_TYPE) *fifo, FIFO_TYPE *value);
FIFO_SIZE_T FIFO_METHOD(FIFO_TYPE, read) (FIFO_T(FIFO_TYPE) *fifo, FIFO_TYPE *buffer, FIFO_SIZE_T size);
FIFO_SIZE_T FIFO_METHOD(FIFO_TYPE, get) (FIFO_T(FIFO_TYPE) *fifo, FIFO_TYPE *buffer);
FIFO_SIZE_T FIFO_METHOD(FIFO_TYPE, size) (FIFO_T(FIFO_TYPE) *fifo);
FIFO_SIZE_T FIFO_METHOD(FIFO_TYPE, margin) (FIFO_T(FIFO_TYPE) *fifo);
FIFO_T(FIFO_TYPE) * FIFO_METHOD(FIFO_TYPE,init) (FIFO_T(FIFO_TYPE) *fifo, FIFO_BUFFER_STORAGE FIFO_TYPE *buffer, FIFO_SIZE_T size);

#define FIFO_DIRECT_PUT(fifo, value, res) { \
  FIFO_BUFFER_STORAGE FIFO_TYPE *next = (fifo).prius + 1; \
  res = 0; \
  if(next == ((fifo).buffer + (fifo).size)){next = (fifo).buffer;} \
  if(next != (fifo).follower){ \
    *((fifo).prius) = value; \
    (fifo).prius = next; \
    res = 1; \
  } \
}

#define FIFO_DIRECT_GET(fifo, buf, res) { \
  res = 0; \
  if((fifo).follower != (fifo).prius){ \
    buf = *((fifo).follower++); \
    if((fifo).follower == (fifo).buffer + (fifo).size){ \
      (fifo).follower = (fifo).buffer; \
    } \
    res = 1; \
  } \
}

#endif /* __FIFO_H__ */
