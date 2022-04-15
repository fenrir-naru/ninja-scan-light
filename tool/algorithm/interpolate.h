/*
 * Copyright (c) 2022, M.Naruoka (fenrir)
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

#ifndef __INTERPOLATE_H__
#define __INTERPOLATE_H__

#include <stdexcept>
#include <cstddef>
#include <vector>

/*
 * perform Neville's interpolation
 *
 * @param x_given list of x assumed to be accessible with [], order is non-sensitive
 * @param y_given list of y assumed to be accessible with [], order is non-sensitive
 * @param x desired x
 * @param y_buf buffer to store temporary results
 * @param n order
 * @return (Ty) interpolated result
 */
template <class Tx_Array, class Ty_Array, class Tx, class Ty>
Ty interpolate_Neville(
    const Tx_Array &x_given, const Ty_Array &y_given,
    const Tx &x,
    Ty *y_buf, const std::size_t &n) {
  if(n == 0){return y_given[0];}
  for(int j(1); j <= (int)n; ++j){
    for(int i(0); i <= ((int)n - j); ++i){
      Tx width(x_given[i + j] - x_given[i]);
      Tx w_a((x_given[i + j] - x) / width), w_b((x - x_given[i]) / width);
      if(j > 1){
        // 2nd, 3rd, ... order interpolation
        y_buf[i] = y_buf[i] * w_a + y_buf[i + 1] * w_b;
      }else{
        // linear interpolation
        y_buf[i] = y_given[i] * w_a + y_given[i + 1] * w_b;
      }
    }
  }
  return y_buf[0];
}

template <class Tx_Array, class Ty_Array, class Tx, class Ty, std::size_t N>
Ty interpolate_Neville(
    const Tx_Array &x_given, const Ty_Array &y_given,
    const Tx &x,
    Ty (&y_buf)[N]) {
  return interpolate_Neville<Tx_Array, Ty_Array, Tx, Ty>(x_given, y_given, x, y_buf, N);
}

template <std::size_t N, class Tx, class Ty>
Ty interpolate_Neville(
    const std::vector<Tx> &x_given, const std::vector<Ty> &y_given, const Tx &x) {
  if((N > x_given.size()) || (N > y_given.size())){
    throw std::out_of_range("Given x or y list are too short");
  }
  Ty y_buf[N];
  return interpolate_Neville<
      std::vector<Tx>, std::vector<Ty>, Tx, Ty, N>(x_given, y_given, x, y_buf);
}

#endif /* __INTERPOLATE_H__ */
