/*
 * Copyright (c) 2021, M.Naruoka (fenrir)
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

/** @file
 * @brief bit array library as an alternative of std::bitset
 */


#ifndef __BIT_ARRAY_H__
#define __BIT_ARRAY_H__

#include <climits>
#include <cstring>
#include <cstdlib>

#include <vector>

template <int MAX_SIZE, class ContainerT = unsigned char>
struct BitArray { ///< alternation of std::bitset
  static const int bits_per_addr = (int)sizeof(ContainerT) * CHAR_BIT;
  ContainerT buf[(MAX_SIZE + bits_per_addr - 1) / bits_per_addr];
  void set(const bool &new_bit = false) {
    std::memset(buf, (new_bit ? (~0) : 0), sizeof(buf));
  }
  void reset() {set(false);}
  void clear() {reset();}
  bool operator[](const int &idx) const {
    if((idx < 0) || (idx >= MAX_SIZE)){return false;}
    if(MAX_SIZE > bits_per_addr){
      std::div_t qr(std::div(idx, bits_per_addr));
      ContainerT mask((ContainerT)1 << qr.rem);
      return buf[qr.quot] & mask;
    }else{
      return buf[0] & ((ContainerT)1 << idx);
    }
  }
  void set(const int &idx, const bool &bit = true) {
    if((idx < 0) || (idx >= MAX_SIZE)){return;}
    if(MAX_SIZE > bits_per_addr){
      std::div_t qr(std::div(idx, bits_per_addr));
      ContainerT mask((ContainerT)1 << qr.rem);
      bit ? (buf[qr.quot] |= mask) : (buf[qr.quot] &= ~mask);
    }else{
      ContainerT mask((ContainerT)1 << idx);
      bit ? (buf[0] |= mask) : (buf[0] &= ~mask);
    }
  }
  void reset(const int &idx) {
    set(idx, false);
  }
  /**
   * Generate bit pattern with arbitrary start and end indices
   */
  unsigned int pattern(const int &idx_lsb, int idx_msb) const {
    if((idx_msb < idx_lsb) || (idx_lsb < 0) || (idx_msb >= MAX_SIZE)){
      return 0;
    }
    static const int res_bits((int)sizeof(unsigned int) * CHAR_BIT);
    if((idx_msb - idx_lsb) >= res_bits){
      // check output boundary; if overrun, msb will be truncated.
      idx_msb = idx_lsb + res_bits - 1;
    }

    std::div_t qr_lsb(std::div(idx_lsb, bits_per_addr)),
        qr_msb(std::div(idx_msb, bits_per_addr));
    if(res_bits > bits_per_addr){
      unsigned int res(buf[qr_msb.quot] & ((qr_msb.rem == bits_per_addr - 1)
          ? ~((ContainerT)0)
          : (((ContainerT)1 << (qr_msb.rem + 1)) - 1))); // MSB byte
      if(qr_msb.quot > qr_lsb.quot){
        for(int i(qr_msb.quot - 1); i > qr_lsb.quot; --i){ // Fill intermediate
          res <<= bits_per_addr;
          res |= buf[i];
        }
        res <<= (bits_per_addr - qr_lsb.rem);
        res |= (buf[qr_lsb.quot] >> qr_lsb.rem); // Last byte
      }else{
        res >>= qr_lsb.rem;
      }
      return res;
    }else{
      ContainerT res(buf[qr_msb.quot] & ((qr_msb.rem == bits_per_addr - 1)
          ? ~((ContainerT)0)
          : (((ContainerT)1 << (qr_msb.rem + 1)) - 1))); // MSB byte
      if(qr_msb.quot > qr_lsb.quot){
        res <<= (bits_per_addr - qr_lsb.rem);
        res |= (buf[qr_lsb.quot] >> qr_lsb.rem); // Last byte
      }else{
        res >>= qr_lsb.rem;
      }
      return (unsigned int)res;
    }
  }
  std::vector<int> indices_one() const {
    std::vector<int> res;
    int idx(0);
    static const std::div_t qr(std::div(MAX_SIZE, bits_per_addr));
    int rem(qr.rem);
    for(int i(0); i < qr.quot; ++i, idx += bits_per_addr){
      int idx2(idx);
      for(ContainerT temp(buf[i]); temp > 0; temp >>= 1, ++idx2){
        if(temp & 0x1){res.push_back(idx2);}
      }
    }
    for(ContainerT temp(buf[qr.quot + 1]); (temp > 0) && (rem > 0); --rem, ++idx, temp >>= 1){
      if(temp & 0x1){res.push_back(idx);}
    }
    return res;
  }
};

#endif /* __BIT_ARRAY_H__ */
