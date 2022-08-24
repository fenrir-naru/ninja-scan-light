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

#include <cstring>

template <typename StorageT>
struct FIFO_Duplicator {
  struct memcpy_t {
    memcpy_t(const StorageT *src, StorageT *dist){
      std::memcpy(dist, src, sizeof(StorageT));
    }
    template <typename SizeT>
    memcpy_t(const StorageT *src, StorageT *dist, SizeT size){
      std::memcpy(dist, src, sizeof(StorageT) * size);
    }
  };

  struct operator_eq_t {
    operator_eq_t(const StorageT *src, StorageT *dist){
      *dist = *src;
    }
    template <typename SizeT>
    operator_eq_t(const StorageT *src, StorageT *dist, SizeT size){
      while(size--){
        *(dist++) = *(src++);
      }
    }
  };
};

template <
  typename StorageT,
  typename DuplicatorT = typename FIFO_Duplicator<StorageT>::memcpy_t
>
class FIFO {
  public:
    typedef StorageT storage_t;
    static const unsigned storage_bytes;
  protected:
    unsigned int capacity;
    StorageT *storage;
    StorageT *prius;
    StorageT *follower;
#ifndef BOOL_IS_UCHAR
    typedef bool bool_t;
#else
    typedef unsigned char bool_t;
#endif
  public:
    typedef FIFO<StorageT, DuplicatorT> self_t;
    FIFO(const unsigned int &_capacity)
        : capacity(_capacity), storage(new StorageT[capacity]),
        prius(storage), follower(storage) {
    }
    FIFO()
        : capacity(0), storage(NULL),
        prius(NULL), follower(NULL){}
    virtual ~FIFO(){
      delete [] storage;
    }
    
    unsigned int size() const{
      return capacity;
    } 
    int stored() const {
      return (prius >= follower ?
           prius - follower :
           prius + capacity - follower);
    }
    bool_t is_empty() const {
      return prius == follower;
    }
    
    int margin() const {
      return capacity - stored() - 1;
    }
    bool_t has_margin() const {
      return margin() > 0;
    }
    
    /**
     * Write data to FIFO
     * 
     * @param data 
     * @param size 
     * @return (int)
     */
   unsigned int write(const StorageT *values, unsigned int size){
      unsigned int _size;
      StorageT *prius_next;
      if(values == NULL){return 0;}
      if((_size = margin()) <= size){size = _size;}
      _size = storage + capacity - prius;
      if(_size <= size){
        DuplicatorT(values, prius, _size);
        prius_next = storage;
        values += _size;
        _size = size - _size;
      }else{
        prius_next = prius;
        _size = size;
      }
      if(_size > 0){
        DuplicatorT(values, prius_next, _size);
        prius_next += _size;
      }
      prius = prius_next;
      return size;
    }
    
    /**
     * Push data to FIFO
     * 
     * @param data 
     * @return (int)
     */
    unsigned int push(const StorageT *value){
      if(value == NULL){return 0;}
      {
        StorageT *next(prius + 1);
        if(next == (storage + capacity)) next = storage;
        if(next != follower){
          DuplicatorT(value, prius);
          prius = next;
          return 1;
        }else{
          return 0;
        }
      }
    }
    unsigned int push(const StorageT &value){
      return push(&value);
    }
    
    unsigned int skip(unsigned int size){
      unsigned int _size;
      StorageT *follower_next;
      if((_size = stored()) <= size){size = _size;}
      _size = storage + capacity - follower;
      if(_size <= size){
        follower_next = storage;
        _size = size - _size;
      }else{
        follower_next = follower;
        _size = size;
      }
      if(_size > 0){
        follower_next += _size;
      }
      follower = follower_next;
      return size;
    }
    
    /**
     * read data from FIFO
     *  
     * @param buffer 
     * @param size 
     * @return (int) 
     */
    unsigned int read(StorageT *buffer, unsigned int size){
      unsigned int _size;
      StorageT *follower_next;
      if(buffer == NULL){return 0;}
      if((_size = stored()) <= size){size = _size;}
      _size = storage + capacity - follower;
      if(_size <= size){
        DuplicatorT(follower, buffer, _size);
        follower_next = storage;
        buffer += _size;
        _size = size - _size;
      }else{
        follower_next = follower; 
        _size = size;
      }
      if(_size > 0){
        DuplicatorT(follower_next, buffer, _size);
        follower_next += _size;
      }
      follower = follower_next;
      return size;
    }
    
    /**
     * pop data from FIFO
     *  
     * @param buffer 
     * @return (int) 
     */
    unsigned int pop(StorageT *buffer){
      if(buffer == NULL){return 0;}
      if(follower != prius){
        DuplicatorT(follower, buffer);
        follower++;
        if(follower == storage + capacity){
          follower = storage;
        }
        return 1;
      }else{
        return 0;
      }
    }
    const StorageT &operator[] (const int &index) const {
      StorageT *result;
      if(index >= 0){
        result = follower + index;
        if(result >= (storage + capacity)){
          result -= capacity;
        }
      }else{
        result = prius + index;
        if(result < storage){
          result += capacity;
        }
      }
      return *result;
    }
    StorageT &operator[] (const int &index) {
      return const_cast<StorageT &>(static_cast<const self_t *>(this)->operator[](index));
    }
    const StorageT &head() const {
      return operator[](0);
    }
    StorageT &head() {
      return operator[](0);
    }
    const StorageT &tail() const {
      return operator[](-1);
    }
    StorageT &tail() {
      return operator[](-1);
    }
    
    /**
     * inspect data in FIFO
     *  
     * @param buffer 
     * @param size
     * @param offset 
     * @return (int) 
     */
    unsigned int inspect(
        StorageT *buffer, 
        unsigned int size, 
        const unsigned int &offset = 0) const {
      unsigned int _size;
      StorageT *follower2;
      if(buffer == NULL){return 0;}
      if((_size = stored()) <= offset){return 0;}
      if((_size -= offset) <= size){size = _size;}
      if((follower2 = follower + offset) >= (storage + capacity)){
        follower2 -= capacity;
      }
      _size = storage + capacity - follower2;
      if(_size <= size){
        DuplicatorT(follower2, buffer, _size);
        follower2 = storage;
        buffer += _size;
        _size = size - _size;
      }else{_size = size;}
      if(_size > 0){
        DuplicatorT(follower2, buffer, _size);
      }
      return size;
    }
    
    /**
     * Resize FIFO capacity
     * 
     * @param _capacity new size
     */
    void resize(const unsigned int &_capacity) {
      StorageT *new_storage = new StorageT[_capacity];
      prius = new_storage + inspect(new_storage, _capacity);
      follower = new_storage;
      if(storage){delete [] storage;}
      storage = new_storage;
      capacity = _capacity;
    }

    FIFO(const self_t &orig)
        : capacity(orig.capacity), storage(new StorageT[capacity]),
        prius(storage), follower(storage) {
      prius += orig.inspect(storage, orig.size());
    }
    self_t &operator=(const self_t &another){
      if(storage != another.storage){
        if(capacity < another.capacity){
          delete [] storage;
          storage = new StorageT[another.capacity];
          capacity = another.capacity;
        }
        prius = follower = storage;
        prius += another.inspect(storage, another.size());
      }
      return *this;
    }
};

template <
  typename StorageT,
  typename DuplicatorT
>
const unsigned FIFO<StorageT, DuplicatorT>::storage_bytes = sizeof(StorageT);

#endif /* __FIFO_H__ */
