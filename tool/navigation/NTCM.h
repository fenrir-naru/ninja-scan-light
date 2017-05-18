/*
 * Copyright (c) 2017, M.Naruoka (fenrir)
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

#ifndef __NTCM_H__
#define __NTCM_H__

#include <cmath>

template <class FloatT>
class NTCM_GL {
  public:
    typedef FloatT float_t;
    static const float_t C[];

    static const float_t lt_D;
    static const float_t P_F1;
    static float_t f1(
        const float_t &lt,
        const float_t &phi, const float_t &delta){
      float_t
          v_D((lt - lt_D) / 24 * M_PI * 2),
          v_SD(lt / 12 * M_PI * 2),
          v_TD(lt / 8 * M_PI * 2);
      float_t
          cos_Chi_star(std::sin(phi) * std::sin(delta) + std::cos(phi) * std::cos(delta)),
          cos_Chi_star2(cos_Chi_star - std::sin(delta) * phi * 2 / M_PI),
          cos_Chi_star3(std::sqrt(cos_Chi_star + P_F1));
      return cos_Chi_star3
          + (C[0] * std::cos(v_D)
            + C[1] * std::cos(v_SD) + C[2] * std::sin(v_SD)
            + C[3] * std::cos(v_TD) + C[4] * std::sin(v_TD)) * cos_Chi_star;
    }

    static const float_t doy_A, doy_SA;
    static float_t f2(const float_t &doy){
      float_t
          v_A((doy - doy_A) / 365.25 * M_PI * 2),
          v_SA((doy - doy_SA) / 365.25 * M_PI * 4);
      return C[5] * std::cos(v_A) + C[6] * std::cos(v_SA) + 1;
    }

    static float_t f3(const float_t &phi_m){
      return C[7] * std::cos(phi_m) + 1;
    }

    static const float_t phi_c1, sigma_c1;
    static const float_t phi_c2, sigma_c2;
    static float_t f4(const float_t &phi_m) {
      float_t
          ec1(std::pow((phi_m - phi_c1) / sigma_c1, 2) * -0.5),
          ec2(std::pow((phi_m - phi_c2) / sigma_c2, 2) * -0.5);
      return C[8] * std::exp(ec1) + C[9] * std::exp(ec2) + 1;
    }

    static float_t f5(const float_t &f_10_7){
      return C[10] + (C[11] * f_10_7);
    }

    static float_t tec_vert(
        const float_t &lt, const float_t &phi, const float_t &delta,
        const float_t &doy,
        const float_t &phi_m,
        const float_t &f_10_7){
      return f1(lt, phi, delta) * f2(doy) * f3(phi_m) * f4(phi_m) * f5(f_10_7);
    }
};

template <class FloatT>
const typename NTCM_GL<FloatT>::float_t NTCM_GL<FloatT>::C[] = {
  0.89656,
  0.16984,
  -0.02166,
  0.05928,
  0.00738,
  0.13912,
  -0.17593,
  -0.34545,
  1.1167,
  1.1573,
  -4.3356,
  0.17775
};

template <class FloatT>
const typename NTCM_GL<FloatT>::float_t NTCM_GL<FloatT>::lt_D = 14;
template <class FloatT>
const typename NTCM_GL<FloatT>::float_t NTCM_GL<FloatT>::P_F1 = 0.4;

template <class FloatT>
const typename NTCM_GL<FloatT>::float_t NTCM_GL<FloatT>::doy_A = 18;
template <class FloatT>
const typename NTCM_GL<FloatT>::float_t NTCM_GL<FloatT>::doy_SA = 6;

template <class FloatT>
const typename NTCM_GL<FloatT>::float_t NTCM_GL<FloatT>::phi_c1 = M_PI / 180 * 16;
template <class FloatT>
const typename NTCM_GL<FloatT>::float_t NTCM_GL<FloatT>::phi_c2 = M_PI / 180 * -10;
template <class FloatT>
const typename NTCM_GL<FloatT>::float_t NTCM_GL<FloatT>::sigma_c1 = M_PI / 180 * 12;
template <class FloatT>
const typename NTCM_GL<FloatT>::float_t NTCM_GL<FloatT>::sigma_c2 = M_PI / 180 * 13;

#endif /* __NTCM_H__ */
