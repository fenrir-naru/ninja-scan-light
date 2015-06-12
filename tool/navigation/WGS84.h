/*
 * Copyright (c) 2015, M.Naruoka (fenrir)
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

#ifndef __WGS84_H__
#define __WGS84_H__

#include <cmath>

/** @file
 * @brief 地球モデルWGS84
 * 
 * 地球の物理モデルのひとつであるWGS84について記述したファイル。
 */

/**
 * @brief 地球モデルWGS84
 * 
 * 地球モデルであるWGS84を表現するクラス。
 * 
 * 極率半径および重力の計算ができます。
 */
template <class FloatT>
class WGS84Generic{
  public:
    static const FloatT R_e;            ///< 赤道半径[m]
    static const FloatT F_e;            ///< 地球の扁平率
    static const FloatT Omega_Earth;    ///< 地球自転速度
    static const FloatT Omega_Earth_IAU;///< 地球自転速度(IAU, GRS67)
    static const FloatT mu_Earth;       ///< 地球重力定数[m^3/s^2]
    static const FloatT epsilon_Earth;  ///< 第一偏心性
    static const FloatT g_WGS0;         ///< 赤道上重力
    static const FloatT g_WGS1;         ///< 重力公式定数

#ifndef pow2
#define pow2(x) ((x) * (x))
#else
#define ALREADY_POW2_DEFINED
#endif
    /**
     * 南北方向(すなわち経線上)の極率半径を求めます。
     * 
     * @param latitude 緯度[rad]
     * @return (FloatT) 南北方向の極率半径[m]
     */
    static FloatT R_meridian(const FloatT &latitude){
      return R_e * (1. - pow2(epsilon_Earth))
                  / std::pow((1. - pow2(epsilon_Earth) * pow2(std::sin(latitude))), 1.5);
    }
    
    /**
     * 東西方向の極率半径を求めます。
     * 
     * @param latitude 緯度[rad]
     * @return (FloatT) 東西方向の極率半径[m]
     */
    static FloatT R_normal(const FloatT &latitude){
      return R_e / std::sqrt(1. - pow2(epsilon_Earth) * pow2(std::sin(latitude)));  
    }
    
    /**
     * 重力を求めます
     * 
     * @param latitude 緯度[rad]
     * @return 重力[m/s^2]
     */
    static FloatT gravity(const FloatT &latitude){
      return g_WGS0 * (1. + g_WGS1 * pow2(std::sin(latitude)))
                / std::sqrt(1. - pow2(epsilon_Earth) * pow2(std::sin(latitude))); 
    }
#ifdef ALREADY_POW2_DEFINED
#undef ALREADY_POW2_DEFINED
#else 
#undef pow2
#endif
};

template <class FloatT>
const FloatT WGS84Generic<FloatT>::R_e = 6378137;
template <class FloatT>
const FloatT WGS84Generic<FloatT>::F_e = (1.0 / 298.257223563);
template <class FloatT>
const FloatT WGS84Generic<FloatT>::Omega_Earth = 7.292115E-5;
template <class FloatT>
const FloatT WGS84Generic<FloatT>::Omega_Earth_IAU = 7.2921151467E-5;
template <class FloatT>
const FloatT WGS84Generic<FloatT>::mu_Earth = 3.986005E14;
template <class FloatT>
const FloatT WGS84Generic<FloatT>::epsilon_Earth = 0.0818191908426;
template <class FloatT>
const FloatT WGS84Generic<FloatT>::g_WGS0 = 9.7803267714;
template <class FloatT>
const FloatT WGS84Generic<FloatT>::g_WGS1 = 0.00193185138639;

typedef WGS84Generic<double> WGS84;

#endif /* __WGS84_H__ */
