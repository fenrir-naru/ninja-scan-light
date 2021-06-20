/*
 * Copyright (c) 2016, M.Naruoka (fenrir)
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

#ifndef __GLONASS_H__
#define __GLONASS_H__

/** @file
 * @brief GLONASS ICD definitions
 */


#include <ctime>
#define _USE_MATH_DEFINES
#include <cmath>
#include <climits>
#include <cstdlib>

#include "GPS.h"

template <class FloatT = double>
class GLONASS_SpaceNode {
  public:
    typedef FloatT float_t;
    
  public:
    typedef GLONASS_SpaceNode<float_t> self_t;

    typedef unsigned char u8_t;
    typedef signed char s8_t;
    typedef unsigned short u16_t;
    typedef signed short s16_t;
    typedef unsigned int u32_t;
    typedef signed int s32_t;
    
    typedef int int_t;
    typedef unsigned int uint_t;

    typedef typename GPS_SpaceNode<float_t>::DataParser DataParser;

    template <class InputT,
        int EffectiveBits = sizeof(InputT) * CHAR_BIT,
        int PaddingBits_MSB = (int)sizeof(InputT) * CHAR_BIT - EffectiveBits>
    struct BroadcastedMessage : public DataParser {
#define convert_u(bits, offset_bits, length, name) \
static u ## bits ## _t name(const InputT *buf){ \
  return \
      DataParser::template bits2num<u ## bits ## _t, EffectiveBits, PaddingBits_MSB>( \
        buf, offset_bits, length); \
}
#define convert_s(bits, offset_bits, length, name) \
static s ## bits ## _t name(const InputT *buf){ \
  u ## bits ## _t temp( \
      DataParser::template bits2num<u ## bits ## _t, EffectiveBits, PaddingBits_MSB>( \
        buf, offset_bits, length)); \
  static const u ## bits ## _t mask((u ## bits ## _t)1 << (length - 1)); /* MSB is sign bit */ \
  return (temp & mask) ? ((s ## bits ## _t)-1 * (temp & (mask - 1))) : temp; \
}
      convert_u( 8,  1,  4, m);
      convert_u( 8, 77,  8, KX);

      struct String1 {
        convert_u( 8,  7,  2, P1);
        convert_u(16,  9, 12, t_k);
        convert_s(32, 21, 24, xn_dot);
        convert_s( 8, 45,  5, xn_ddot);
        convert_s(32, 50, 27, xn);
      };
      struct String2 {
        convert_u( 8,  5,  3, B_n);
        convert_u( 8,  8,  1, P2);
        convert_u( 8,  9,  7, t_b);
        convert_s(32, 21, 24, yn_dot);
        convert_s( 8, 45,  5, yn_ddot);
        convert_s(32, 50, 27, yn);
      };
      struct String3 {
        convert_u( 8,  5,  1, P3);
        convert_s(16,  6, 11, gamma_n);
        convert_u( 8, 18,  2, p);
        convert_u( 8, 20,  1, l_n);
        convert_s(32, 21, 24, zn_dot);
        convert_s( 8, 45,  5, zn_ddot);
        convert_s(32, 50, 27, zn);
      };
      struct String4 {
        convert_s(32,  5, 22, tau_n);
        convert_u( 8, 27,  5, delta_tau_n);
        convert_u( 8, 32,  5, E_n);
        convert_u( 8, 51,  1, P4);
        convert_u( 8, 52,  4, F_T);
        convert_u(16, 59, 11, N_T);
        convert_u( 8, 70,  5, n);
        convert_u( 8, 75,  2, M);
      };
      struct String5_Alnamac {
        convert_u(16,  5, 11, NA);
        convert_s(32, 16, 32, tau_c);
        convert_u( 8, 49,  5, N_4);
        convert_s(32, 54, 22, tau_GPS);
        convert_u( 8, 76,  1, l_n);
      };

      struct String6_Alnamac { // String 6; same as 8, 10, 12, 14
        convert_u( 8,  5,  1, C_n);
        convert_u( 8,  6,  2, M_n);
        convert_u( 8,  8,  5, nA);
        convert_s(16, 13, 10, tauA_n);
        convert_s(32, 23, 21, lambdaA_n);
        convert_s(32, 44, 18, delta_iA_n);
        convert_u(16, 62, 15, epsilonA_n);
      };
      struct String7_Alnamac { // String 7; same as 9, 11, 13, 15
        convert_s(16,  5, 16, omegaA_n);
        convert_u(32, 21, 21, tA_lambda_n);
        convert_s(32, 42, 22, delta_TA_n);
        convert_s( 8, 64,  7, delta_TA_dot_n);
        convert_u( 8, 71,  5, HA_n);
        convert_u( 8, 76,  1, l_n);
      };

      struct Frame5_String14 {
        convert_s(16,  5, 11, B1);
        convert_s(16, 16, 10, B2);
        convert_u( 8, 26,  2, KP);
      };
#undef convert_s
#undef convert_u
    };

    struct TimeProperties {
      float_t tau_c; // [s]
      float_t tau_GPS; // [s]
      struct date_t {
        uint_t year; // 20XX
        uint_t day_of_year; // Jan. 1st = 0
      } date;
      bool l_n;

      struct raw_t {
        s32_t tau_c;
        s32_t tau_GPS;
        u8_t N_4;
        u16_t NA;
        bool l_n;
#define fetch_item(name) name = BroadcastedMessage< \
   InputT, (int)sizeof(InputT) * CHAR_BIT - PaddingBits_MSB - PaddingBits_LSB, PaddingBits_MSB> \
   :: String5_Alnamac :: name (src)
        template <int PaddingBits_MSB, int PaddingBits_LSB, class InputT>
        void update_string5(const InputT *src){
          fetch_item(tau_c);
          fetch_item(tau_GPS);
          fetch_item(N_4);
          fetch_item(NA);
          fetch_item(l_n);
        }
#undef fetch_item

        enum {
          SF_tau_c, SF_tau_GPS,
          SF_NUM,
        };
        static const float_t sf[SF_NUM];

        static date_t raw2date(const u8_t &N_4_, const u16_t &NA_) {
          // @see A.3.1.3 (ver.5.1)
          date_t res;
          res.year = 1996 + 4 * (N_4_ - 1);
          if(NA_ <= 366){
            res.day_of_year = NA_ - 1;
          }else if(NA_ <= 731){
            res.year += 1;
            res.day_of_year = NA_ - 367;
          }else if(NA_ <= 1096){
            res.year += 2;
            res.day_of_year = NA_ - 732;
          }else if(NA_ <= 1461){
            res.year += 3;
            res.day_of_year = NA_ - 1097;
          }
          return res;
        }

        operator TimeProperties() const {
          TimeProperties res;
#define CONVERT(TARGET) \
{res.TARGET = sf[SF_ ## TARGET] * TARGET;}
          CONVERT(tau_c);
          CONVERT(tau_GPS);
          res.date = raw2date(N_4, NA);
          res.l_n = (l_n > 0);
#undef CONVERT
          return res;
        }

        raw_t &operator=(const TimeProperties &t){
#define CONVERT(TARGET) \
{TARGET = (s32_t)((t.TARGET + 0.5 * sf[SF_ ## TARGET]) / sf[SF_ ## TARGET]);}
          CONVERT(tau_c);
          CONVERT(tau_GPS);
          std::div_t divmod(std::div(t.date.year - 1996, 4));
          N_4 = divmod.quot + 1;
          NA = t.date.day_of_year + 1;
          switch(divmod.rem){
            case 1: NA += 366; break;
            case 2: NA += 731; break;
            case 3: NA += 1096; break;
          }
          l_n = (t.l_n ? 1 : 0);
#undef CONVERT
          return *this;
        }
      };
    };

    class SatelliteProperties {
      public:
        struct Ephemeris {
          uint_t t_k; // time base [s]
          uint_t t_b; // time interval to UTC(SU) + 3hr [s]
          uint_t M; // satellite type; 0 - GLONASS, 1 - GLONASS-M
          float_t gamma_n;
          float_t tau_n; // [s]
          float_t xn, yn, zn; // [m]
          float_t xn_dot, yn_dot, zn_dot; // [m/s]
          float_t xn_ddot, yn_ddot, zn_ddot; // [m/s^2]
          uint_t B_n; // health flag
          uint_t p; // satellite operation mode
          uint_t N_T; // current date counting up from leap year Jan. 1st [days]
          float_t F_T; // user range accuracy at t_b [m]
          uint_t n; // time difference between L1 and L2
          float_t delta_tau_n;
          uint_t E_n; // [days]
          uint_t P1; // [s] time interval of two adjacent t_b; 0 - 0, 1 - 30, 2 - 45, 3 - 60 mins
          bool P2;
          //bool P3; // flag for alnamac; 1 - five, 0 - four
          bool P4; // flag for ephemeris; 1 - uploaded by the control segment
          bool l_n; // health flag; 0 - healthy, 1 - malfunction

          u8_t F_T_index() const;

          struct raw_t {
            // String1
            u8_t P1;
            u16_t t_k;
            s32_t xn_dot;       ///< SF: 2^-20 [km/s]
            s8_t xn_ddot;       ///< SF: 2^-30 [km/s^2]
            s32_t xn;           ///< SF: 2^-11 [km]

            // String2
            u8_t B_n;
            u8_t P2;
            u8_t t_b;           ///< SF: 15
            s32_t yn_dot;
            s8_t yn_ddot;
            s32_t yn;

            // String3
            u8_t P3;
            s16_t gamma_n;      ///< SF: 2^-40
            u8_t p;
            u8_t l_n;
            s32_t zn_dot;
            s8_t zn_ddot;
            s32_t zn;

            // String4
            s32_t tau_n;        ///< SF: 2^-30
            u8_t delta_tau_n;   ///< SF: 2^-30
            u8_t E_n;
            u8_t P4;
            u8_t F_T;
            u16_t N_T;          ///< [days]
            u8_t n;
            u8_t M;

#define fetch_item(num, name) name = BroadcastedMessage< \
   InputT, (int)sizeof(InputT) * CHAR_BIT - PaddingBits_MSB - PaddingBits_LSB, PaddingBits_MSB> \
   :: String ## num :: name (src)
            template <int PaddingBits_MSB, int PaddingBits_LSB, class InputT>
            void update_string1(const InputT *src){
              fetch_item(1, P1);
              fetch_item(1, t_k);
              fetch_item(1, xn_dot);
              fetch_item(1, xn_ddot);
              fetch_item(1, xn);
            }
            template <int PaddingBits_MSB, int PaddingBits_LSB, class InputT>
            void update_string2(const InputT *src){
              // String2
              fetch_item(2, B_n);
              fetch_item(2, P2);
              fetch_item(2, t_b);
              fetch_item(2, yn_dot);
              fetch_item(2, yn_ddot);
              fetch_item(2, yn);
            }
            template <int PaddingBits_MSB, int PaddingBits_LSB, class InputT>
            void update_string3(const InputT *src){
              fetch_item(3, P3);
              fetch_item(3, gamma_n);
              fetch_item(3, p);
              fetch_item(3, l_n);
              fetch_item(3, zn_dot);
              fetch_item(3, zn_ddot);
              fetch_item(3, zn);
            }
            template <int PaddingBits_MSB, int PaddingBits_LSB, class InputT>
            void update_string4(const InputT *src){
              fetch_item(4, tau_n);
              fetch_item(4, delta_tau_n);
              fetch_item(4, E_n);
              fetch_item(4, P4);
              fetch_item(4, F_T);
              fetch_item(4, N_T);
              fetch_item(4, n);
              fetch_item(4, M);
            }
#undef fetch_item

            enum {
              SF_xn,       SF_yn = SF_xn,            SF_zn = SF_xn,
              SF_xn_dot,   SF_yn_dot = SF_xn_dot,    SF_zn_dot = SF_xn_dot,
              SF_xn_ddot,  SF_yn_ddot = SF_xn_ddot,  SF_zn_ddot = SF_xn_ddot,
              SF_t_b,
              SF_gamma_n,
              SF_tau_n,
              SF_delta_tau_n,
              SF_NUM,
            };
            static const float_t sf[SF_NUM];

            static const float_t F_T_table[15]; ///< @see Table 4.4

            operator Ephemeris() const {
              Ephemeris res;
#define CONVERT(TARGET) \
{res.TARGET = sf[SF_ ## TARGET] * TARGET;}
              res.t_k = (((t_k >> 7) & 0x1F) * 3600) // hour
                  + (((t_k >> 1) & 0x3F) * 60) // min
                  + ((t_k & 0x1) ? 30 : 0); // sec
              CONVERT(t_b);
              res.M = M;
              CONVERT(gamma_n);
              CONVERT(tau_n);
              CONVERT(xn);  CONVERT(xn_dot);  CONVERT(xn_ddot);
              CONVERT(yn);  CONVERT(yn_dot);  CONVERT(yn_ddot);
              CONVERT(zn);  CONVERT(zn_dot);  CONVERT(zn_ddot);
              res.B_n = B_n;
              res.p = p;
              res.N_T = N_T;

              if(F_T >= (sizeof(F_T_table) / sizeof(F_T_table[0]))){
                res.F_T = -1; // not used
              }else{
                res.F_T = F_T_table[F_T];
              }
              res.n = n;
              CONVERT(delta_tau_n);
              res.E_n = E_n;
              switch(P1){
                case 1: res.P1 = 30 * 60; break;
                case 2: res.P1 = 45 * 60; break;
                case 3: res.P1 = 60 * 60; break;
                case 0: default: res.P1 = 0; break;
              }
              res.P2 = (P2 > 0);
              //res.P3 = (P3 > 0);
              res.P4 = (P4 > 0);
              res.l_n = (l_n > 0);
#undef CONVERT
              return res;
            }
            raw_t &operator=(const Ephemeris &eph){
              // TODO: m?
#define CONVERT(TARGET) \
{TARGET = (s32_t)((eph.TARGET + 0.5 * sf[SF_ ## TARGET]) / sf[SF_ ## TARGET]);}
              { // t_k
                std::div_t minutes(div(eph.t_k, 60));
                std::div_t hrs(div(minutes.quot, 60));
                t_k = (((hrs.quot << 6) + minutes.quot) << 1) + (minutes.rem > 0 ? 1 : 0);
              }
              CONVERT(t_b);
              M = eph.M;
              CONVERT(gamma_n);
              CONVERT(tau_n);
              CONVERT(xn);  CONVERT(xn_dot);  CONVERT(xn_ddot);
              CONVERT(yn);  CONVERT(yn_dot);  CONVERT(yn_ddot);
              CONVERT(zn);  CONVERT(zn_dot);  CONVERT(zn_ddot);
              B_n = eph.B_n;
              p = eph.p;
              N_T = eph.N_T;
              F_T = eph.F_T_index();
              n = eph.n;
              CONVERT(delta_tau_n);
              E_n  = eph.E_n;
              if(eph.P1 > 45 * 60){
                P1 = 3;
              }else if(eph.P1 > 30 * 60){
                P1 = 2;
              }else if(eph.P1 > 0){
                P1 = 1;
              }else{
                P1 = 0;
              }
              P2 = (eph.P2 ? 1 : 0);
              //P3 = (eph.P3 ? 1 : 0);
              P4 = (eph.P4 ? 1 : 0);
              l_n = (eph.l_n ? 1 : 0);
#undef CONVERT
              return *this;
            }
          };

          bool is_equivalent(const Ephemeris &eph) const {
            do{
#define CHECK(TARGET) if(TARGET != eph.TARGET){break;}
              CHECK(t_k);
              CHECK(t_b);
              CHECK(M);
              CHECK(B_n);
              CHECK(p);
              CHECK(N_T);
              CHECK(F_T_index());
              CHECK(n);
              CHECK(E_n);
              CHECK(P1);
              CHECK(P2);
              //CHECK(P3);
              CHECK(P4);
              CHECK(l_n);
#undef CHECK
#define CHECK(TARGET) \
if(std::abs(TARGET - eph.TARGET) > raw_t::sf[raw_t::SF_ ## TARGET]){break;}
              CHECK(gamma_n);
              CHECK(tau_n);
              CHECK(xn);  CHECK(xn_dot);  CHECK(xn_ddot);
              CHECK(yn);  CHECK(yn_dot);  CHECK(yn_ddot);
              CHECK(zn);  CHECK(zn_dot);  CHECK(zn_ddot);
              CHECK(delta_tau_n);
#undef CHECK
              return true;
            }while(false);
            return false;
          }
        };
    };

    struct Satellite : public SatelliteProperties {

    };
};

#define POWER_2(n) \
(((n) >= 0) \
  ? (float_t)(1 << (n >= 0 ? n : 0)) \
  : (((float_t)1) / (1 << (-(n) >= 30 ? 30 : -(n > 0 ? 0 : n))) \
    / (1 << (-(n) >= 30 ? (-(n) - 30) : 0))))

template <class FloatT>
const typename GLONASS_SpaceNode<FloatT>::float_t GLONASS_SpaceNode<FloatT>::TimeProperties::raw_t::sf[] = {
  POWER_2(-31), // tau_c [s]
  POWER_2(-30) * 60 * 60 * 24, // tau_GPS [day] => [s]
};

template <class FloatT>
const typename GLONASS_SpaceNode<FloatT>::float_t GLONASS_SpaceNode<FloatT>::SatelliteProperties::Ephemeris::raw_t::sf[] = {
  POWER_2(-11) * 1E3, // x
  POWER_2(-20) * 1E3, // x_dot
  POWER_2(-30) * 1E3, // x_ddot
  15 * 60, // t_b
  POWER_2(-40), // gamma_n
  POWER_2(-30), // tau
  POWER_2(-30), // delta_tau
};

#undef POWER_2

template <class FloatT>
const typename GLONASS_SpaceNode<FloatT>::float_t GLONASS_SpaceNode<FloatT>::SatelliteProperties::Ephemeris::raw_t::F_T_table[] = {
  1, 2, 2.5, 4, 5, 7, 10, 12, 14, 16, 32, 64, 128, 256, 512,
};

template <class FloatT>
typename GLONASS_SpaceNode<FloatT>::u8_t GLONASS_SpaceNode<FloatT>::SatelliteProperties::Ephemeris::F_T_index() const {
  if(F_T <= 0){ // invalid value
    return sizeof(raw_t::F_T_table) / sizeof(raw_t::F_T_table[0]);
  }
  u8_t res(0);
  while(res < (sizeof(raw_t::F_T_table) / sizeof(raw_t::F_T_table[0]))){
    if(F_T <= raw_t::F_T_table[res]){break;}
    ++res;
  }
  return res;
}

#endif /* __GLONASS_H__ */
