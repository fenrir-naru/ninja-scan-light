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

#include <cstddef>
#include <vector>

/*
 * perform Neville's interpolation (with derivative)
 *
 * @param x_given list of x assumed to be accessible with [], order is non-sensitive
 * @param y_given list of y assumed to be accessible with [], order is non-sensitive
 * @param x desired x
 * @param y y[0] is output and the others are used as buffer to store temporary results
 * @param n order
 * @param dy dy[i][0] is (i+1)th derivative outputs like y[0]
 * @param nd maximum order of required derivative
 * @return (Ty &) [0] = interpolated result
 */
template <class Tx_Array, class Ty_Array, class Tx, class Ty>
Ty &interpolate_Neville(
    const Tx_Array &x_given, const Ty_Array &y_given,
    const Tx &x, Ty &y, const std::size_t &n,
    Ty *dy = NULL, const std::size_t &nd = 0) {
  if(n == 0){
    y[0] = y_given[0];
    // for(int d(nd); d >= 0; --d){dy[d][0] = 0;} // assumption
  }
  for(int j(1); j <= (int)n; ++j){
    for(int d(-1 + (((int)nd >= j) ? j : nd)); d >= 0; --d){
      // d = 0, 1, ... are 1st, 2nd, ... order derivative
      // skip dy[d(>=j)] because of 0
      Ty &dy0(dy[d]), &dy1(d > 0 ? (Ty &)dy[d - 1] : (Ty &)y); // cast required for MSVC
      for(int i(0); i <= ((int)n - j); ++i){
        if(d + 1 < j){
          Tx a(x_given[i + j] - x), b(x - x_given[i]);
          dy0[i] = dy0[i] * a + dy0[i + 1] * b;
          dy0[i] += ((j > 1)
              ? (dy1[i+1] - dy1[i])
              : (y_given[i+1] - y_given[i])) * (d + 1);
        }else{
          dy0[i] = ((j > 1)
              ? (dy1[i+1] - dy1[i])
              : (y_given[i+1] - y_given[i])) * (d + 1);
        }
        dy0[i] /= (x_given[i + j] - x_given[i]);
      }
    }
    for(int i(0); i <= ((int)n - j); ++i){ // d = -1 (interpolation)
      Tx a(x_given[i + j] - x), b(x - x_given[i]);
      y[i] = (j > 1)
          ? (y[i] * a + y[i + 1] * b) // 2nd, 3rd, ... order
          : (y_given[i] * a + y_given[i + 1] * b); // linear
      y[i] /= (x_given[i + j] - x_given[i]);
    }
  }
  return y;
}

template <class Tx_Array, class Ty_Array, class Tx, class Ty, std::size_t N>
Ty interpolate_Neville(
    const Tx_Array &x_given, const Ty_Array &y_given,
    const Tx &x, Ty (&y_buf)[N]) {
  return interpolate_Neville(x_given, y_given, x, y_buf, N)[0];
}

template <class Ty, class Tx_Array, class Ty_Array, class Tx>
Ty interpolate_Neville(
    const Tx_Array &x_given, const Ty_Array &y_given, const Tx &x, const std::size_t &n) {
  if(n == 0){return y_given[0];}
  std::vector<Ty> y_buf(n);
  return interpolate_Neville(x_given, y_given, x, y_buf, n)[0];
}

#endif /* __INTERPOLATE_H__ */
