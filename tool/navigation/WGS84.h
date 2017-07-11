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
 * @brief World Geodetic System 1984 (WGS84) (地球モデルWGS84)
 *
 * 地球モデルであるWGS84を表現するクラス。
 * 極率半径および重力の計算ができます。
 *
 * CAUTION: Part of parameters are refined.
 * 
 * @see  NIMA TR 8350.2 Third Edition
 * Department of Defense World Geodetic System 1984
 * Its Definition and Relationships with Local Geodetic Systems
 */

template <class FloatT>
class WGS84Generic{
  public:
    // Defining parameters
    static const FloatT R_e;              ///< 3.2.1 (a) Semi-major Axis 赤道半径 [m]
    static const FloatT F_e;              ///< 3.2.2 (f) flattening 地球の扁平率
    static const FloatT Omega_Earth;      ///< 3.2.4 (omega) Angular Velocity of the Earth 地球自転速度 [rad/s]
    static const FloatT Omega_Earth_IAU;  ///< 3.2.4 (omega') Angular Velocity of the Earth 地球自転速度; IAU, GRS67 version [rad/s]
    static const FloatT mu_Earth;         ///< 3.2.3.2 (GM_orig) Earth’s Gravitational Constant 地球重力定数; original [m^3/s^2]
    static const FloatT mu_Earth_refined; ///< 3.2.3 (GM) Earth’s Gravitational Constant 地球重力定数; refined [m^3/s^2]

    // Derived parameters (refined)
    static const FloatT epsilon_Earth;    ///< Table 3.3 (e) First Eccentricity 第一偏心性
    static const FloatT g_WGS0;           ///< Table 3.4 (gamma_e) Theoretical (Normal) Gravity at the Equator on the Ellipsoid 赤道上重力 [m/s^2]
    static const FloatT g_WGS1;           ///< Table 3.4 (k) Theoretical (Normal) Gravity Formula Constant 重力公式定数
    static const FloatT m_derived;        ///< Table 3.4 (m) omega^2 * a^2 * b / GM

#ifndef pow2
#define pow2(x) ((x) * (x))
#else
#define ALREADY_POW2_DEFINED
#endif
    /**
     * Calculate curvature radius (north-south)
     * 南北方向(すなわち経線上)の極率半径を求めます。
     * 
     * @param latitude (geodetic) latitude (測地)緯度 [rad]
     * @return (FloatT) radius 南北方向の極率半径[m]
     */
    static FloatT R_meridian(const FloatT &latitude){
      return R_e * (1. - pow2(epsilon_Earth))
                  / std::pow((1. - pow2(epsilon_Earth) * pow2(std::sin(latitude))), 1.5);
    }
    
    /**
     * Calculate curvature radius (east-west)
     * 東西方向の極率半径を求めます。
     * 
     * @param latitude (geodetic) latitude (測地)緯度 [rad]
     * @return (FloatT) radius 東西方向の極率半径[m]
     */
    static FloatT R_normal(const FloatT &latitude){
      return R_e / std::sqrt(1. - pow2(epsilon_Earth) * pow2(std::sin(latitude)));  
    }
    
    /**
     * Calculate gravity
     * 重力を求めます
     * 
     * @param latitude (geodetic) latitude (測地)緯度 [rad]
     * @param altitude (高度)[m]
     * @return gravity 重力[m/s^2]
     */
    static FloatT gravity(const FloatT &latitude, const FloatT &altitude = 0){
      FloatT slat2(pow2(std::sin(latitude)));
      FloatT g0(g_WGS0 * (1. + g_WGS1 * slat2) / std::sqrt(1. - pow2(epsilon_Earth) * slat2));
      if(altitude == 0){
        return g0;
      }else{
        // @see Eq. (4-3)
        return g0 * (1.0 \
            - (2.0 / R_e * (1.0 + F_e + m_derived - 2.0 * F_e * slat2) * altitude) \
            + (3.0 / pow2(R_e) * pow2(altitude)));
      }
    }
#ifdef ALREADY_POW2_DEFINED
#undef ALREADY_POW2_DEFINED
#else 
#undef pow2
#endif
};

template <class FloatT>
const FloatT WGS84Generic<FloatT>::R_e = 6378137.0;
template <class FloatT>
const FloatT WGS84Generic<FloatT>::F_e = (1.0 / 298.257223563);
template <class FloatT>
const FloatT WGS84Generic<FloatT>::Omega_Earth = 7292115.0E-11;
template <class FloatT>
const FloatT WGS84Generic<FloatT>::Omega_Earth_IAU = 7292115.1467E-11;
template <class FloatT>
const FloatT WGS84Generic<FloatT>::mu_Earth = 3986005.0E8;
template <class FloatT>
const FloatT WGS84Generic<FloatT>::mu_Earth_refined = 3986004.418E8;

template <class FloatT>
const FloatT WGS84Generic<FloatT>::epsilon_Earth = 8.1819190842622E-2;
template <class FloatT>
const FloatT WGS84Generic<FloatT>::g_WGS0 = 9.7803253359;
template <class FloatT>
const FloatT WGS84Generic<FloatT>::g_WGS1 = 0.00193185265241;
template <class FloatT>
const FloatT WGS84Generic<FloatT>::m_derived = 0.00344978650684;


typedef WGS84Generic<double> WGS84;

#endif /* __WGS84_H__ */
