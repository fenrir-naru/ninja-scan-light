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

#include "fifo.h"
#include <string.h>

#ifndef min
#define min(x, y) (((x) < (y)) ? (x) : (y))
#endif

#ifndef max
#define max(x, y) (((x) > (y)) ? (x) : (y))
#endif

#define NULL_CHECK 0

/**
 * FIFOを初期化します。
 * 
 * @param fifo
 * @param buffer
 * @param size 
 * @return ( FIFO_T(FIFO_TYPE) ) 
 */
FIFO_T(FIFO_TYPE) * FIFO_METHOD(FIFO_TYPE, init) (
      FIFO_T(FIFO_TYPE) *fifo, 
      FIFO_BUFFER_STORAGE FIFO_TYPE * buffer, 
      FIFO_SIZE_T size){
  if(size > 0){
    fifo->buffer = buffer;
    fifo->size = size;
    fifo->prius = fifo->buffer; 
    fifo->follower = fifo->buffer;
    return fifo;
  }else{return NULL;}
}

/**
 * FIFOに書き出します
 * 
 * @param fifo 
 * @param data 
 * @param size 
 * @return (int)
 */
FIFO_SIZE_T FIFO_METHOD(FIFO_TYPE, write) (
      FIFO_T(FIFO_TYPE) *fifo, 
      FIFO_TYPE *values, 
      FIFO_SIZE_T size){
  FIFO_SIZE_T _size;
  FIFO_BUFFER_STORAGE FIFO_TYPE *prius_next;
#if NULL_CHECK
  if(values == NULL){return 0;}
#endif
  size = min(FIFO_METHOD(FIFO_TYPE, margin)(fifo), size);
  _size = fifo->buffer + fifo->size - fifo->prius;
  if(_size <= size){
    memcpy(fifo->prius, values, _size);
    prius_next = fifo->buffer;
    values += _size;
    _size = size - _size;
  }else{
    prius_next = fifo->prius;
    _size = size;
  }
  if(_size > 0){
    memcpy(prius_next, values, _size);
    prius_next += _size;
  }
  fifo->prius = prius_next;
  return size;
}

/**
 * FIFOに書き出します
 * 
 * @param fifo 
 * @param data 
 * @return (int)
 */
FIFO_SIZE_T FIFO_METHOD(FIFO_TYPE, put) (
      FIFO_T(FIFO_TYPE) *fifo, 
      FIFO_TYPE *value){
#if NULL_CHECK
  if(values == NULL){return 0;}
#endif
  {
    FIFO_BUFFER_STORAGE FIFO_TYPE *next = fifo->prius + 1;
    if(next == (fifo->buffer + fifo->size)) next = fifo->buffer;
    if(next != fifo->follower){
      *fifo->prius = *value;
      fifo->prius = next;
      return 1;
    }else{
      return 0;
    }
  }
}

/**
 * FIFOから読み込みます
 * 
 * @param fifo 
 * @param buffer 
 * @param size 
 * @return (int) 
 */
FIFO_SIZE_T FIFO_METHOD(FIFO_TYPE, read) (
      FIFO_T(FIFO_TYPE) *fifo, 
      FIFO_TYPE *buffer, 
      FIFO_SIZE_T size){
  FIFO_SIZE_T _size;
  FIFO_BUFFER_STORAGE FIFO_TYPE *follower_next;
#if NULL_CHECK
  if(buffer == NULL){return 0;}
#endif
  size = min(FIFO_METHOD(FIFO_TYPE, size)(fifo), size);
  _size = fifo->buffer + fifo->size - fifo->follower;
  if(_size <= size){
    memcpy(buffer, fifo->follower, _size);
    follower_next = fifo->buffer;
    buffer += _size;
    _size = size - _size;
  }else{
    follower_next = fifo->follower;
    _size = size;
  }
  if(_size > 0){
    memcpy(buffer, follower_next, _size);
    follower_next += _size;
  }
  fifo->follower = follower_next;
  return size;
}

/**
 * FIFOから読み込みます
 * 
 * @param fifo 
 * @param buffer 
 * @return (int) 
 */
FIFO_SIZE_T FIFO_METHOD(FIFO_TYPE, get) (
      FIFO_T(FIFO_TYPE) *fifo, 
      FIFO_TYPE *buffer){
#if NULL_CHECK
  if(buffer == NULL){return 0;}
#endif
  if(fifo->follower != fifo->prius){
    *buffer = *(fifo->follower++);
    if(fifo->follower == fifo->buffer + fifo->size){
      fifo->follower = fifo->buffer;
    }
    return 1;
  }else{
    return 0;
  }
}

/**
 * FIFOの書込み済みデータ大きさを返します
 * 
 * @param fifo 
 * @return int 
 */
FIFO_SIZE_T FIFO_METHOD(FIFO_TYPE, size) (FIFO_T(FIFO_TYPE) *fifo){
  FIFO_BUFFER_STORAGE FIFO_TYPE *_prius = fifo->prius;
  FIFO_BUFFER_STORAGE FIFO_TYPE *_follower = fifo->follower;
  return (_prius >= _follower ?
           _prius - _follower:
           _prius + fifo->size - _follower);
}

/**
 * FIFOの空き領域の大きさを返します
 * 
 * @param fifo 
 * @return int 
 */
FIFO_SIZE_T FIFO_METHOD(FIFO_TYPE, margin) (FIFO_T(FIFO_TYPE) *fifo){
  FIFO_BUFFER_STORAGE FIFO_TYPE *_prius = fifo->prius;
  FIFO_BUFFER_STORAGE FIFO_TYPE *_follower = fifo->follower;
  return (_follower <= _prius ?
           _follower + fifo->size - _prius - 1:
           _follower - _prius - 1);
}
