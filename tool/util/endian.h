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

#ifndef __ENDIAN_H__
#define __ENDIAN_H__

#ifndef IS_LITTLE_ENDIAN
#define IS_LITTLE_ENDIAN 1
#endif

template <typename NumberT>
inline NumberT swap_endian(const NumberT &v){
  char *v_p((char *)&v);
  NumberT result;
  char *result_p((char *)&result);
  for(int i = 0, j = sizeof(NumberT) - 1; 
      i < sizeof(NumberT); i++, j--){
    *(result_p + i) = *(v_p + j);
  }
  return result;
}
#if IS_LITTLE_ENDIAN
template <typename NumberT>
inline NumberT le_num_2_num(const NumberT &v){
  return v;
}
template <typename NumberT>
inline NumberT be_num_2_num(const NumberT &v){
  return swap_endian<NumberT>(v);
}
template <typename NumberT>
inline NumberT num_2_le_num(const NumberT &v){
  return v;
}
template <typename NumberT>
inline NumberT num_2_be_num(const NumberT &v){
  return swap_endian<NumberT>(v);
}
#else
template <typename NumberT>
inline NumberT le_num_2_num(const NumberT &v){
  return swap_endian<NumberT>(v);
}
template <typename NumberT>
inline NumberT be_num_2_num(const NumberT &v){
  return v;
}
template <typename NumberT>
inline NumberT num_2_le_num(const NumberT &v){
  return swap_endian<NumberT>(v);
}
template <typename NumberT>
inline NumberT num_2_be_num(const NumberT &v){
  return v;
}
#endif

template <typename NumberT>
#if IS_LITTLE_ENDIAN
inline NumberT &le_char8_2_num(const char &top){
  return (NumberT &)top;
}
#else
inline NumberT le_char8_2_num(const char &top){
  char *char_p(const_cast<char *>(&top) + 7);
  NumberT result(0);
  while(true){
    result |= (unsigned char)*(char_p--);
    if(char_p < (&top)){break;}
    result <<= 8;    
  }
  return result;
}
#endif

template <typename NumberT>
#if IS_LITTLE_ENDIAN
inline NumberT be_char8_2_num(const char &top){
  char *top_p(const_cast<char *>(&top));
  NumberT result(0);
  while(true){
    result |= (unsigned char)*(top_p++);
    if(top_p >= ((&top) + 8)){break;}
    result <<= 8;    
  }
  return result;
}
#else
inline NumberT &be_char8_2_num(const char &top){
  return (NumberT &)top;
}
#endif

template <typename NumberT>
#if IS_LITTLE_ENDIAN
inline NumberT &le_char4_2_num(const char &top){
  return (NumberT &)top;
}
#else
inline NumberT le_char4_2_num(const char &top){
  const char *top_p(&top);
  NumberT result(0);
  result |= (unsigned char)*(top_p + 3);
  result <<= 8;
  result |= (unsigned char)*(top_p + 2);
  result <<= 8;
  result |= (unsigned char)*(top_p + 1);
  result <<= 8;
  result |= (unsigned char)*(top_p);
  return result;
}
#endif

template <typename NumberT>
#if IS_LITTLE_ENDIAN
inline NumberT be_char4_2_num(const char &top){
  const char *top_p(&top);
  NumberT result(0);
  result |= (unsigned char)*(top_p);
  result <<= 8;
  result |= (unsigned char)*(top_p + 1);
  result <<= 8;
  result |= (unsigned char)*(top_p + 2);
  result <<= 8;
  result |= (unsigned char)*(top_p + 3);
  return result;
}
#else
inline NumberT &be_char4_2_num(const char &top){
  return (NumberT &)top;
}
#endif

template <typename NumberT>
#if IS_LITTLE_ENDIAN
inline NumberT &le_char2_2_num(const char &top){
  return (NumberT &)top;
}
#else
inline NumberT le_char2_2_num(const char &top){
  const char *top_p(&top);
  NumberT result(0);
  result |= (unsigned char)*(top_p + 1);
  result <<= 8;
  result |= (unsigned char)*(top_p);
  return result;
}
#endif

template <typename NumberT>
#if IS_LITTLE_ENDIAN
inline NumberT be_char2_2_num(const char &top){
  const char *top_p(&top);
  NumberT result(0);
  result |= (unsigned char)*(top_p);
  result <<= 8;
  result |= (unsigned char)*(top_p + 1);
  return result;
}
#else
inline NumberT &be_char2_2_num(const char &top){
  return (NumberT &)top;
}
#endif

#endif /* __ENDIAN_H__ */
