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
 * @param x_given list of x assumed to be accessible with [0..n], order is non-sensitive
 * @param y_given list of y assumed to be accessible with [0..n], order is non-sensitive
 * @param x desired x
 * @param y y[0] is output, and y[1..n-1] are used as buffer to store temporary results
 * @param n order
 * @param dy dy[i][0] is (i+1)th derivative outputs like y[0];
 * dy[0..nd][0..n-1] should be accessible
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
    // for(std::size_t d(nd); d >= 0; --d){dy[d][0] = 0;}
    return y;
  }
  { // first step
    if(nd > 0){ // for 1st order derivative
      for(std::size_t i(0); i < n; ++i){
        dy[0][i] = (y_given[i+1] - y_given[i]);
        dy[0][i] /= (x_given[i+1] - x_given[i]);
      }
    }
    for(std::size_t i(0); i < n; ++i){ // linear interpolation
      Tx a(x_given[i+1] - x), b(x - x_given[i]);
      y[i] = (y_given[i] * a + y_given[i+1] * b);
      y[i] /= (x_given[i+1] - x_given[i]);
    }
  }
  for(std::size_t j(2); j <= n; ++j){
    int d((nd >= j) ? j : nd);
    // d = 1, 2, ... are 1st, 2nd, ... order derivative
    // In order to avoid overwriting of temporary calculation,
    // higher derivative calculation is performed earlier.
    // dy[d(>=j+1)] is skipped because of 0
    if(d >= (int)j){ // for derivative, just use lower level
      Ty &dy0(dy[d-1]), &dy1(d > 1 ? (Ty &)dy[d-2] : (Ty &)y); // cast required for MSVC
      for(std::size_t i(0); i <= (n - j); ++i){
        dy0[i] = (dy1[i+1] - dy1[i]) * d;
        dy0[i] /= (x_given[i + j] - x_given[i]);
      }
      --d;
    }
    for(; d > 0; --d){ // for derivative, use same and lower level
      Ty &dy0(dy[d-1]), &dy1(d > 1 ? (Ty &)dy[d-2] : (Ty &)y); // cast required for MSVC
      for(std::size_t i(0); i <= (n - j); ++i){
        Tx a(x_given[i + j] - x), b(x - x_given[i]);
        dy0[i] = dy0[i] * a + dy0[i+1] * b;
        dy0[i] += (dy1[i+1] - dy1[i]) * d;
        dy0[i] /= (x_given[i + j] - x_given[i]);
      }
    }
    for(std::size_t i(0); i <= (n - j); ++i){ // d == 0 (interpolation), just use same level
      Tx a(x_given[i + j] - x), b(x - x_given[i]);
      y[i] = (y[i] * a + y[i+1] * b);
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
