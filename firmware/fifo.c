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

/**
 * Initialize FIFO
 * 
 * @param fifo
 * @param buffer buffer for FIFO
 * @param size maximum number of data to be stored
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
 * Write multiple data to FIFO
 * 
 * @param fifo 
 * @param values data to be written
 * @param size maximum number of data to be written
 * @return (FIFO_SIZE_T) number of accepted data
 */
FIFO_SIZE_T FIFO_METHOD(FIFO_TYPE, write) (
      FIFO_T(FIFO_TYPE) *fifo, 
      FIFO_TYPE *values, 
      FIFO_SIZE_T size){
  FIFO_SIZE_T _size;
  FIFO_BUFFER_STORAGE FIFO_TYPE *prius_next;
  if(NULL_CHECK && (values == NULL)){return 0;}
  size = min(FIFO_METHOD(FIFO_TYPE, margin)(fifo), size);
  _size = fifo->buffer + fifo->size - fifo->prius;
  if(_size <= size){
    memcpy(fifo->prius, values, _size * sizeof(FIFO_TYPE));
    prius_next = fifo->buffer;
    values += _size;
    _size = size - _size;
  }else{
    prius_next = fifo->prius;
    _size = size;
  }
  if(_size > 0){
    memcpy(prius_next, values, _size * sizeof(FIFO_TYPE));
    prius_next += _size;
  }
  fifo->prius = prius_next;
  return size;
}

/**
 * Write single data to FIFO
 * 
 * @param fifo 
 * @param value data to be written
 * @return (FIFO_SIZE_T) number of accepted data (1 or 0)
 */
FIFO_SIZE_T FIFO_METHOD(FIFO_TYPE, put) (
      FIFO_T(FIFO_TYPE) *fifo, 
      FIFO_TYPE *value){
  FIFO_SIZE_T res = 0;
  if((!NULL_CHECK) || (value != NULL)){
    FIFO_DIRECT_PUT(*fifo, *value, res);
  }
  return res;
}

/**
 * Read multiple data from FIFO
 * 
 * @param fifo 
 * @param buffer buffer to be loaded
 * @param size size of buffer
 * @return (FIFO_SIZE_T) number of loaded data
 */
FIFO_SIZE_T FIFO_METHOD(FIFO_TYPE, read) (
      FIFO_T(FIFO_TYPE) *fifo, 
      FIFO_TYPE *buffer, 
      FIFO_SIZE_T size){
  FIFO_SIZE_T _size;
  FIFO_BUFFER_STORAGE FIFO_TYPE *follower_next;
  if(NULL_CHECK && (buffer == NULL)){return 0;}
  size = min(FIFO_METHOD(FIFO_TYPE, size)(fifo), size);
  _size = fifo->buffer + fifo->size - fifo->follower;
  if(_size <= size){
    memcpy(buffer, fifo->follower, _size * sizeof(FIFO_TYPE));
    follower_next = fifo->buffer;
    buffer += _size;
    _size = size - _size;
  }else{
    follower_next = fifo->follower;
    _size = size;
  }
  if(_size > 0){
    memcpy(buffer, follower_next, _size * sizeof(FIFO_TYPE));
    follower_next += _size;
  }
  fifo->follower = follower_next;
  return size;
}

/**
 * Read single data from FIFO
 * 
 * @param fifo 
 * @param buffer buffer to be loaded
 * @return (FIFO_SIZE_T) number of loaded data (1 or 0)
 */
FIFO_SIZE_T FIFO_METHOD(FIFO_TYPE, get) (
      FIFO_T(FIFO_TYPE) *fifo, 
      FIFO_TYPE *buffer){
  FIFO_SIZE_T res = 0;
  if((!NULL_CHECK) || (buffer != NULL)){
    FIFO_DIRECT_GET(*fifo, *buffer, res);
  }
  return res;
}

/**
 * Return size of used area in FIFO
 * 
 * @param fifo 
 * @return (FIFO_SIZE_T) used size
 */
FIFO_SIZE_T FIFO_METHOD(FIFO_TYPE, size) (FIFO_T(FIFO_TYPE) *fifo){
  FIFO_BUFFER_STORAGE FIFO_TYPE *_prius = fifo->prius;
  FIFO_BUFFER_STORAGE FIFO_TYPE *_follower = fifo->follower;
  return (_prius >= _follower ?
           _prius - _follower:
           _prius + fifo->size - _follower);
}

/**
 * Return size of free area in FIFO
 * 
 * @param fifo 
 * @return (FIFO_SIZE_T) free size
 */
FIFO_SIZE_T FIFO_METHOD(FIFO_TYPE, margin) (FIFO_T(FIFO_TYPE) *fifo){
  FIFO_BUFFER_STORAGE FIFO_TYPE *_prius = fifo->prius;
  FIFO_BUFFER_STORAGE FIFO_TYPE *_follower = fifo->follower;
  return (_follower <= _prius ?
           _follower + fifo->size - _prius - 1:
           _follower - _prius - 1);
}
