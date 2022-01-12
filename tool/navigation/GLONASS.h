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
#include <ctime>

#include "GPS.h"
#include "algorithm/integral.h"

template <class FloatT = double>
class GLONASS_SpaceNode {
  public:
    typedef FloatT float_t;
    
    static const float_t light_speed;
    static const float_t L1_frequency;
    static const float_t L1_frequency_gap;
    static const float_t L2_frequency;
    static const float_t L2_frequency_gap;

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
      float_t tau_c; ///< GLONASS time scale correction to UTC(SU) time [s]
      float_t tau_GPS; ///< fractional part of time difference from GLONASS time to GPS time [s], integer part should be derived from another source
      struct date_t {
        int_t year; // 20XX
        int_t day_of_year; // Jan. 1st = 0
        static const int_t days_m[12];
        /**
         * @return (std::tm)
         */
        std::tm c_tm() const {
          std::tm res = {0};
          res.tm_year = year - 1900;
          res.tm_yday = res.tm_mday = day_of_year; // tm_yday ranges [0, 365]
          do{
            if(res.tm_mday < days_m[0]){break;} // tm_mon ranges [0, 11], Check January
            res.tm_mday -= days_m[0];
            ++res.tm_mon;
            if((year % 400 == 0) || ((year % 4 == 0) && (year % 100 != 0))){ // is leap year?
              if(res.tm_mday < (days_m[1] + 1)){break;} // Check February in leap year
              res.tm_mday -= (days_m[1] + 1);
              ++res.tm_mon;
            }
            for(; res.tm_mon < (int)(sizeof(days_m) / sizeof(days_m[0])); ++res.tm_mon){
              if(res.tm_mday < days_m[res.tm_mon]){break;}
              res.tm_mday -= days_m[res.tm_mon];
            }
          }while(false);
          ++res.tm_mday; // tm_mday ranges [1, 31]
          {
            /* day of week according to (modified) Sakamoto's methods
             *  @see https://en.wikipedia.org/wiki/Determination_of_the_day_of_the_week#Sakamoto's_methods
             */
            int_t y(year - 1); // year/1/1
            res.tm_wday = (y + y/4 - y/100 + y/400 + day_of_year + 1) % 7;
          }
          return res;
        }
        static date_t from_c_tm(const std::tm &date){
          static const struct days_t {
            int_t sum_m[sizeof(days_m) / sizeof(days_m[0])];
            days_t(){
              sum_m[0] = 0;
              for(unsigned int i(1); i < sizeof(days_m) / sizeof(days_m[0]); ++i){
                sum_m[i] = sum_m[i - 1] + days_m[i - 1];
              }
            }
          } days;
          date_t res = {date.tm_year + 1900, days.sum_m[date.tm_mon] + date.tm_mday - 1};
          if((date.tm_mon >= 2)
              && ((res.year % 400 == 0) || ((res.year % 4 == 0) && (res.year % 100 != 0)))){
            ++res.day_of_year; // leap year
          }
          return res;
        }
        static float_t julian_day(const std::tm &date){
          // (5.48) of "Preliminary Orbit Determination" by Curtis, Howard D
          // @see original Boulet(1991)
          int y(date.tm_year + 1900), m(date.tm_mon + 1);
          return 1721013.5 // expect all argument of (int) to be >= 0
              + 367 * y
              - (int)(7 * (y + (int)((m + 9) / 12)) / 4)
              + (int)(275 * m / 9)
              + date.tm_mday;
        }
        static float_t Greenwich_sidereal_time_deg(
            const std::tm &date, const float_t &ut_hr = 0){
          // @see "Preliminary Orbit Determination" by Curtis, Howard D
          float_t T0((julian_day(date) - 2451545) / 36525); // (5.49)
          float_t theta_G0_deg( // (5.50)
              100.4606184
              + 36000.77004 * T0
              + 0.000387933 * std::pow(T0, 2)
              - 2.583E-8 * std::pow(T0, 3));
          return theta_G0_deg + 360.98564724 * ut_hr / 24; // (5.51)
        }
      } date;
      bool l_n; // health flag; 0 - healthy, 1 - malfunction

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

      bool is_equivalent(const TimeProperties &t) const {
        do{
#define CHECK(TARGET) if(TARGET != t.TARGET){break;}
          CHECK(l_n);
          CHECK(date.year);
          CHECK(date.day_of_year);
#undef CHECK
#define CHECK(TARGET) \
if(std::abs(TARGET - t.TARGET) > raw_t::sf[raw_t::SF_ ## TARGET]){break;}
          CHECK(tau_c);
          CHECK(tau_GPS);
#undef CHECK
          return true;
        }while(false);
        return false;
      }
    };

    class SatelliteProperties {
      public:
        struct Ephemeris {
          uint_t svid;
          int_t freq_ch;
          uint_t t_k; ///< time referenced to the beginning of the frame from current day [s]
          uint_t t_b; ///< base time of ephemeris parameters in UTC(SU) + 3hr [s]
          uint_t M; ///< satellite type; 0 - GLONASS, 1 - GLONASS-M
          float_t gamma_n; ///< gamma_n(t_c = t_b) = d/dt (t_c - t_n) [dimensionless]
          float_t tau_n; ///< tau_n(t_c = t_b) = t_c (GLONASS time) - t_n(n-th satellite time) [s]
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

          float_t frequncy_L1() const {
            return L1_frequency + L1_frequency_gap * freq_ch;
          }

          float_t frequncy_L2() const {
            return L2_frequency + L2_frequency_gap * freq_ch;
          }

          struct constellation_t { // TODO make it to be a subclass of System_XYZ<float_t, PZ90>
            float_t position[3];
            float_t velocity[3];
            float_t pos_abs2() const {
              float_t res(0);
              for(int i(0); i < 3; ++i){
                res += (position[i] * position[i]);
              }
              return res;
            }
            constellation_t operator+(const constellation_t &another) const { // for RK4
              constellation_t res(*this);
              for(int i(0); i < 3; ++i){
                res.position[i] += another.position[i];
                res.velocity[i] += another.velocity[i];
              }
              return res;
            }
            constellation_t operator*(const float_t &sf) const { // for RK4
              constellation_t res(*this);
              for(int i(0); i < 3; ++i){
                res.position[i] *= sf;
                res.velocity[i] *= sf;
              }
              return res;
            }
            constellation_t operator/(const float_t &sf) const { // for RK4
              return operator*(((float_t)1)/sf);
            }
            // TODO constant definitions should be moved to PZ-90.02
            static const float_t omega_E;
            constellation_t abs_corrdinate(const float_t &sidereal_time_in_rad){
              // @see Appendix.A PZ-90.02 to O-X0Y0Z0
              float_t crad(std::cos(sidereal_time_in_rad)), srad(std::sin(sidereal_time_in_rad));
              float_t
                  x0(position[0] * crad - position[1] * srad),
                  y0(position[0] * srad + position[1] * crad);
              constellation_t res = {
                {x0, y0, position[2]},
                {
                  velocity[0] * crad - velocity[1] * srad - omega_E * y0,
                  velocity[0] * srad + velocity[1] * crad + omega_E * x0,
                  velocity[2]
                }
              };
              return res;
            }
            constellation_t rel_corrdinate(const float_t &sidereal_time_in_rad){
              // @see Appendix.A O-X0Y0Z0 to PZ-90.02
              float_t crad(std::cos(sidereal_time_in_rad)), srad(std::sin(sidereal_time_in_rad));
              float_t
                  x( position[0] * crad + position[1] * srad),
                  y(-position[0] * srad + position[1] * crad);
              constellation_t res = {
                {x, y, position[2]},
                {
                   velocity[0] * crad + velocity[1] * srad + omega_E * y,
                  -velocity[0] * srad + velocity[1] * crad - omega_E * x,
                   velocity[2]
                }
              };
              return res;
            }
          };

          struct eccentric_anomaly_t {
            float_t E_k;
            float_t snu_k, cnu_k;
            eccentric_anomaly_t(const float_t &g_k, const float_t &e_k){
              static const float_t delta_limit(1E-12);
              for(int loop(0); loop < 10; loop++){
                float_t E_k2(g_k + e_k * std::sin(E_k));
                if(std::abs(E_k2 - E_k) < delta_limit){break;}
                E_k = E_k2;
              }
              snu_k = std::sqrt(-e_k * e_k + 1) * std::sin(E_k) / (-e_k * std::cos(E_k) + 1);
              cnu_k = (std::cos(E_k) - e_k) * std::sin(E_k) / (-e_k * std::cos(E_k) + 1);
            }
          };

          struct lunar_solar_perturbations_t {
            struct {
              float_t xi, eta, zeta, r;
            } m, s;
            /**
             * @param days Sum of days from the epoch at 00 hours Moscow Time (MT) on 1st January 1975
             * to the epoch at 00 hours MT of current date within which the instant t_e is.
             * @param t_e reference time of ephemeris parameters (in Julian centuries of 36525 ephemeris days);
             */
            lunar_solar_perturbations_t(const float_t &days, const float_t &t_e){
#define dms2rad(deg, min, sec) \
(M_PI / 180 * ((float_t)sec / 3600 + (float_t)min / 60 + deg))
              static const float_t
                  a_m(3.84385243E8), // [m]
                  a_s(1.49598E11), //[m]
                  e_m(0.054900489),
                  e_s(0.016719),
                  i_m(dms2rad(5, 8, 43.4)),
                  epsilon(dms2rad(23, 26, 33)),
                  g_om(dms2rad(-63, 53, 43.41)),
                  g_1m(dms2rad(477198, 50, 56.79)),
                  Omega_om(dms2rad(259, 10, 59.79)),
                  Omega_1m(dms2rad(-1934, 8, 31.23)),
                  Gamma_o(dms2rad(-334, 19, 46.40)),
                  Gamma_1(dms2rad(4069, 2, 2.52)),
                  g_os(dms2rad(358, 28, 33.04)),
                  g_1s(dms2rad(0, 0, 129596579.10));

              static const float_t
                  sf_ci_m(-std::cos(i_m) + 1), si_m(std::sin(i_m)),
                  cepsilon(std::cos(epsilon)), sepsilon(std::sin(epsilon));

              float_t T((27392.375 + days + t_e / 86400) / 36525);

              { // Eq.(3) lunar (m)
                float_t Omega_m(Omega_om + Omega_1m * T);
                float_t sOmega_m(std::sin(Omega_m)), cOmega_m(std::sin(Omega_m));

                float_t xi_ast(-cOmega_m * cOmega_m * sf_ci_m + 1);
                float_t eta_ast(sOmega_m * si_m);
                float_t zeta_ast(cOmega_m * si_m);

                float_t xi_11(sOmega_m * cOmega_m * sf_ci_m);
                float_t xi_12(-sOmega_m * sOmega_m * sf_ci_m + 1);

                float_t eta_11(xi_ast * cepsilon - zeta_ast * sepsilon);
                float_t eta_12(xi_11 * cepsilon + eta_ast * sepsilon);

                float_t zeta_11(xi_ast * sepsilon + zeta_ast * cepsilon);
                float_t zeta_12(zeta_11 * sepsilon - eta_ast * cepsilon);

                float_t Gamma(Gamma_o + Gamma_1 * T);
                float_t sGamma(std::sin(Gamma)), cGamma(std::cos(Gamma));

                eccentric_anomaly_t E_m(g_om + g_1m * T, e_m);

                float_t
                    srad(E_m.snu_k * cGamma + E_m.cnu_k * sGamma),
                    crad(E_m.cnu_k * cGamma - E_m.snu_k * sGamma);
                m.xi    = srad * xi_11    + crad * xi_12;
                m.eta   = srad * eta_11   + crad * eta_12;
                m.zeta  = srad * zeta_11  + crad * zeta_12;
                m.r     = a_m * (-e_m * std::cos(E_m.E_k) + 1);
              }

              { // Eq.(3) solar (s)
                float_t omega_s(dms2rad(281, 13, 15.00 + (6189.03 * T)));
                float_t co(std::cos(omega_s)), so(std::sin(omega_s));

                eccentric_anomaly_t E_s(g_os + g_1s * T, e_s);

                float_t
                    srad(E_s.snu_k * co + E_s.cnu_k * so), // = sin(nu_s + omega_s)
                    crad(E_s.cnu_k * co - E_s.snu_k * so); // = cos(nu_s + omega_s)

                s.xi    = crad;
                s.eta   = srad * cepsilon;
                s.zeta  = srad * sepsilon;
                s.r     = a_s * (-e_s * std::cos(E_s.E_k) + 1);
              }
#undef dms2rad
            }
          };

          struct differential_t {
            float_t J_x_a, J_y_a, J_z_a;
            differential_t() : J_x_a(0), J_y_a(0), J_z_a(0) {}
            differential_t(const float_t &J_x, const float_t &J_y, const float_t &J_z)
                : J_x_a(J_x), J_y_a(J_y), J_z_a(J_z) {}
            /**
             * @param sidereal_time_in_rad TODO calculate from t_b and UTC
             */
            differential_t(const Ephemeris &eph, const float_t &sidereal_time_in_rad){
              float_t crad(std::cos(sidereal_time_in_rad)), srad(std::sin(sidereal_time_in_rad));
              J_x_a = eph.xn_ddot * crad - eph.yn_ddot * srad;
              J_y_a = eph.xn_ddot * srad + eph.yn_ddot * crad;
              J_z_a = eph.zn_ddot;
            }
            constellation_t operator()(const float_t &t, const constellation_t &x) const {
              // @see APPENDIX 3 EXAMPLES OF ALGORITHMS FOR CALCULATION OF COORDINATES AND VELOCITY
              float_t r2(x.pos_abs2()), r(std::sqrt(r2));
              static const float_t
                  a_e(6378136), mu(398600.44E9), C_20(1082625.75E-9); // values are obtained from ver.5.1; TODO move to PZ-90
              float_t mu_bar(mu / r2),
                  x_a_bar(x.position[0] / r), y_a_bar(x.position[1] / r), z_a_bar(x.position[2] / r),
                  sf_z_a_bar(z_a_bar * z_a_bar * -5 + 1),
                  rho2(a_e * a_e / r2);
              constellation_t res = {
                {x.velocity[0], x.velocity[1], x.velocity[2]},
                { // @see ver.5.1 Eq.(1)
                  ((-C_20 * rho2 * sf_z_a_bar * 3 / 2) - 1) * mu_bar * x_a_bar + J_x_a,
                  ((-C_20 * rho2 * sf_z_a_bar * 3 / 2) - 1) * mu_bar * y_a_bar + J_y_a,
                  ((-C_20 * rho2 * (sf_z_a_bar + 2) * 3 / 2) - 1) * mu_bar * z_a_bar + J_z_a,
                }
              };
#if 0
              // If integration is performed in PZ-90, then these additional term is required,
              // This is derived form simplified algorithm in CDMA ICD, (neither ver 5.1 nor ver.4 ICDs use it)
              static const float_t omega_E(7.2921151467E-5), omega_E2(omega_E * omega_E);
              res.velocity[0] += omega_E2 * x.position[0] + omega_E * 2 * x.velocity[1];
              res.velocity[1] += omega_E2 * x.position[1] - omega_E * 2 * x.velocity[0];
#endif
              return res;
            }
          };

          struct differential2_t {
            lunar_solar_perturbations_t J_src;
            constellation_t operator()(const float_t &t, const constellation_t &x) const {
              // @see APPENDIX 3 EXAMPLES OF ALGORITHMS FOR CALCULATION OF COORDINATES AND VELOCITY

              float_t J_x_am, J_y_am, J_z_am;
              { // Eq.(2) lunar (m)
                float_t
                    mu_bar(4902.835E9 / std::pow(J_src.m.r, 2)), // mu_m in [m/s]
                    x_a_bar(x.position[0] / J_src.m.r),
                    y_a_bar(x.position[1] / J_src.m.r),
                    z_a_bar(x.position[2] / J_src.m.r),
                    delta_x(J_src.m.xi - x_a_bar),
                    delta_y(J_src.m.eta  - y_a_bar),
                    delta_z(J_src.m.zeta - z_a_bar),
                    Delta3(std::pow(
                      std::pow(delta_x, 2) + std::pow(delta_y, 2) + std::pow(delta_z, 2), 1.5));
                J_x_am = mu_bar * (delta_x / Delta3 - J_src.m.xi);
                J_y_am = mu_bar * (delta_y / Delta3 - J_src.m.eta);
                J_z_am = mu_bar * (delta_z / Delta3 - J_src.m.zeta);
              }

              float_t J_x_as, J_y_as, J_z_as;
              { // Eq.(2) solar (s)
                float_t
                    mu_bar(0.1325263E21 / std::pow(J_src.s.r, 2)), // mu_s in [m/s]
                    x_a_bar(x.position[0] / J_src.s.r),
                    y_a_bar(x.position[1] / J_src.s.r),
                    z_a_bar(x.position[2] / J_src.s.r),
                    delta_x(J_src.s.xi - x_a_bar),
                    delta_y(J_src.s.eta  - y_a_bar),
                    delta_z(J_src.s.zeta - z_a_bar),
                    Delta3(std::pow(
                      std::pow(delta_x, 2) + std::pow(delta_y, 2) + std::pow(delta_z, 2), 1.5));
                J_x_as = mu_bar * (delta_x / Delta3 - J_src.s.xi);
                J_y_as = mu_bar * (delta_y / Delta3 - J_src.s.eta);
                J_z_as = mu_bar * (delta_z / Delta3 - J_src.s.zeta);
              }
              return differential_t(J_x_am + J_x_as, J_y_am + J_y_as, J_z_am + J_z_as)(t, x);
            }
          };

          u8_t F_T_index() const;

          struct raw_t {
            u8_t svid;

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
              res.svid = svid;
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
              svid = eph.svid;
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
              //CHECK(t_k); // t_k is just a reference time
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

        /**
         *
         * Relationship of time systems
         * 1) t_UTC = t_GL + tau_c - 3hr (polarity of tau_c is not defined in ICD!)
         * 2) t_GPS = t_GL + delta_T + tau_GPS (defined in ICD)
         * 3) t_n(t_b) = t_GL(t_b) - tau_n (@(t_UTC + 3hr) = t_b, defined in ICD)
         * 4) t_n = t_GL - tau_n + gamma_n * (t_GL - t_b) (not explicitly defined in ICD, but in RINEX spec.)
         */
        struct Ephemeris_with_Time : public Ephemeris, TimeProperties {
          typedef typename Ephemeris::constellation_t constellation_t;
          constellation_t xa_t_b;
          float_t sidereal_t_b_rad;
          typename Ephemeris::differential_t eq_of_motion;

          void calculate_additional() {
            sidereal_t_b_rad = TimeProperties::date_t::Greenwich_sidereal_time_deg(
                TimeProperties::date.c_tm(),
                (float_t)(this->t_b) / (60 * 60) - 3) / 180 * M_PI;
            constellation_t x_t_b = {
              {this->xn, this->yn, this->zn},
              {this->xn_dot, this->yn_dot, this->zn_dot},
            };
            xa_t_b = x_t_b.abs_corrdinate(sidereal_t_b_rad); // PZ-90 => O-XYZ
            eq_of_motion = typename Ephemeris::differential_t((*this), sidereal_t_b_rad);
          }
          Ephemeris_with_Time(const Ephemeris &eph, const TimeProperties &t_prop)
              : Ephemeris(eph), TimeProperties(t_prop) {
            calculate_additional();
          }
          Ephemeris_with_Time(const Ephemeris &eph, const std::tm &t_utc)
              : Ephemeris(eph) {
            this->tau_c = this->tau_GPS = 0;
            std::tm t_mt(t_utc); // calculate Moscow time
            t_mt.tm_hour += 3;
            std::mktime(&t_mt); // renormalization
            this->date = TimeProperties::date_t::from_c_tm(t_mt);
            this->t_b = (t_mt.tm_hour * 60 + t_mt.tm_min) * 60 + t_mt.tm_sec;
            calculate_additional();
          }
          std::tm c_tm_utc() const {
            std::tm t(TimeProperties::date.c_tm()); // set date on Moscow time
            (t.tm_sec = (int)(this->t_b)) -= 3 * 60 * 60; // add second on UTC
            std::mktime(&t); // renormalization
            return t;
          }
          struct raw_t : public Ephemeris::raw_t, TimeProperties::raw_t {
            operator Ephemeris_with_Time() const {
              return Ephemeris_with_Time((Ephemeris)(*this), (TimeProperties)(*this));
            }
            raw_t &operator=(const Ephemeris_with_Time &eph){
              (typename Ephemeris::raw_t &)(*this) = eph;
              (typename TimeProperties::raw_t &)(*this) = eph;
              return *this;
            }
          };
          bool is_equivalent(const Ephemeris_with_Time &eph) const {
            return Ephemeris::is_equivalent(eph) && TimeProperties::is_equivalent(eph);
          }
          float_t calculate_clock_error(float_t delta_t, const float_t &pseudo_range = 0) const {
            delta_t -= pseudo_range / light_speed;
            return -Ephemeris::tau_n + Ephemeris::gamma_n * delta_t;
          }
          /**
           * @param t_arrival_glonass signal arrival time in GLONASS time scale (t_GL = t_UTC + 3 hr + tau_c).
           */
          float_t clock_error(
              const float_t &t_arrival_glonass, const float_t &pseudo_range = 0) const {
            return calculate_clock_error(
                t_arrival_glonass + TimeProperties::tau_c - Ephemeris::t_b, pseudo_range); // measure in UTC + 3hr scale
          }
          constellation_t calculate_constellation(
              float_t delta_t, const float_t &pseudo_range = 0) const {
            delta_t -= pseudo_range / light_speed;

            constellation_t res(xa_t_b);
            { // time integration from t_b to t_arrival
              float_t t_step_max(delta_t >= 0 ? 60 : -60);
              int i(std::floor(delta_t / t_step_max));
              float_t t_step_remain(delta_t - t_step_max * i);
              float_t delta_t_itg(0); // accumulative time of integration
              for(; i > 0; --i, delta_t_itg += t_step_max){
                res = nextByRK4(eq_of_motion, delta_t_itg, res, t_step_max);
              }
              res = nextByRK4(eq_of_motion, delta_t_itg, res, t_step_remain);
            }

            static const float_t omega_E(0.7292115E-4); // Earth's rotation rate, TODO move to PZ-90.02
            return res.rel_corrdinate(sidereal_t_b_rad + (omega_E * delta_t)); // transform from abs to PZ-90.02
          }
          /**
           * @param t_arrival_glonass signal arrival time in GLONASS time scale (t_GL = t_UTC + 3 hr + tau_c).
           */
          constellation_t constellation(
              const float_t &t_arrival_glonass, const float_t &pseudo_range = 0) const {
            return calculate_constellation(
                t_arrival_glonass + TimeProperties::tau_c - Ephemeris::t_b, pseudo_range); // measure in UTC + 3hr scale
          }
        };

        struct Ephemeris_with_GPS_Time : public Ephemeris_with_Time {
          GPS_Time<float_t> t_b_gps;
          Ephemeris_with_GPS_Time()
              : Ephemeris_with_Time(Ephemeris(), GPS_Time<float_t>(0, 0).c_tm()),
              t_b_gps(0, 0) {}
          /**
           *
           * @param deltaT integer part of difference of GPS and GLONASS time scales.
           * This is often identical to the leap seconds because GLONASS time base is UTC
           */
          Ephemeris_with_GPS_Time(const Ephemeris_with_Time &eph, const int_t &deltaT = 0)
              : Ephemeris_with_Time(eph),
              t_b_gps((GPS_Time<float_t>(eph.c_tm_utc()) // in UTC scale
                + (-Ephemeris_with_Time::tau_c // in Moscow Time scale
                  + Ephemeris_with_Time::tau_GPS // in (GPS - delta_T) scale (delta_T is integer)
                  + deltaT))) {
          }
          GPS_Time<float_t> base_time() const {
            return t_b_gps;
          }
          bool is_valid(const GPS_Time<float_t> &t) const {
            return std::abs(t_b_gps.interval(t)) <= 60 * 60; // 1 hour
          }
          using Ephemeris_with_Time::clock_error;
          float_t clock_error(
              const GPS_Time<float_t> &t_arrival, const float_t &pseudo_range = 0) const {
            return Ephemeris_with_Time::calculate_clock_error(t_arrival - t_b_gps, pseudo_range);
          }
          using Ephemeris_with_Time::constellation;
          typename Ephemeris_with_Time::constellation_t constellation(
              const GPS_Time<float_t> &t_arrival, const float_t &pseudo_range = 0) const {
            return calculate_constellation(t_arrival - t_b_gps, pseudo_range);
          }
        };
    };

    struct Satellite : public SatelliteProperties {
      public:
        typedef typename SatelliteProperties::Ephemeris_with_GPS_Time eph_t;
        typedef typename GPS_SpaceNode<float_t>::template PropertyHistory<eph_t> eph_list_t;
      protected:
        eph_list_t eph_history;
      public:
        Satellite() : eph_history() {
          // TODO setup first ephemeris as invalid one
          // eph_t &eph_current(const_cast<eph_t &>(eph_history.current()));
        }

        template <class Functor>
        void each_ephemeris(
            Functor &functor,
            const typename eph_list_t::each_mode_t &mode = eph_list_t::EACH_ALL) const {
          eph_history.each(functor, mode);
        }

        void register_ephemeris(const eph_t &eph, const int &priority_delta = 1){
          eph_history.add(eph, priority_delta);
        }

        void merge(const Satellite &another, const bool &keep_original = true){
          eph_history.merge(another.eph_history, keep_original);
        }

        const eph_t &ephemeris() const {
          return eph_history.current();
        }

        /**
         * Select appropriate ephemeris within registered ones.
         *
         * @param target_time time at measurement
         * @return if true, appropriate ephemeris is selected, otherwise, not selected.
         */
        bool select_ephemeris(const GPS_Time<float_t> &target_time){
          return eph_history.select(target_time, &eph_t::is_valid);
        }

        float_t clock_error(const GPS_Time<float_t> &t, const float_t &pseudo_range = 0) const{
          return ephemeris().clock_error(t, pseudo_range);
        }

        typename GPS_SpaceNode<float_t>::SatelliteProperties::constellation_t constellation(
            const GPS_Time<float_t> &t, const float_t &pseudo_range = 0) const {
          typename eph_t::constellation_t constellation_PZ9002(
              ephemeris().constellation(t, pseudo_range));
          // @see https://www.gsi.go.jp/common/000070971.pdf
          // @see (originally) Federal Air Navigation Authority (FANA), Aeronautical Information Circular
          // of the Russian Federation, 12 February 2009, Russia.
          float_t (&pos)[3](constellation_PZ9002.position);
          float_t (&vel)[3](constellation_PZ9002.velocity);
          typename GPS_SpaceNode<float_t>::SatelliteProperties::constellation_t res = {
            typename GPS_SpaceNode<float_t>::xyz_t(pos[0] - 0.36, pos[1] + 0.08, pos[2] + 0.18),
            typename GPS_SpaceNode<float_t>::xyz_t(vel[0], vel[1], vel[2]),
          };
          return res;
        }

        typename GPS_SpaceNode<float_t>::xyz_t position(const GPS_Time<float_t> &t, const float_t &pseudo_range = 0) const {
          return constellation(t, pseudo_range).position;
        }

        typename GPS_SpaceNode<float_t>::xyz_t velocity(const GPS_Time<float_t> &t, const float_t &pseudo_range = 0) const {
          return constellation(t, pseudo_range).velocity;
        }
    };
  public:
    typedef std::map<int, Satellite> satellites_t;
  protected:
    satellites_t _satellites;
  public:
    GLONASS_SpaceNode()
        : _satellites() {}
    ~GLONASS_SpaceNode(){
      _satellites.clear();
    }
    const satellites_t &satellites() const {
      return _satellites;
    }
    Satellite &satellite(const int &prn) {
      return _satellites[prn];
    }
    bool has_satellite(const int &prn) const {
      return _satellites.find(prn) !=  _satellites.end();
    }
    void update_all_ephemeris(const GPS_Time<float_t> &target_time) {
      for(typename satellites_t::iterator it(_satellites.begin()), it_end(_satellites.end());
          it != it_end; ++it){
        it->second.select_ephemeris(target_time);
      }
    }
    typename Satellite::eph_t latest_ephemeris() const {
      struct {
        typename Satellite::eph_t res;
        void operator()(const typename Satellite::eph_t &eph){
          if(res.t_b_gps < eph.t_b_gps){res = eph;}
        }
      } functor;
      for(typename satellites_t::const_iterator
          it(_satellites.begin()), it_end(_satellites.end());
          it != it_end; ++it){
        it->second.each_ephemeris(functor, Satellite::eph_list_t::EACH_NO_REDUNDANT);
      }
      return functor.res;
    }
};

template <class FloatT>
const typename GLONASS_SpaceNode<FloatT>::float_t GLONASS_SpaceNode<FloatT>::light_speed
    = 299792458; // [m/s]

template <class FloatT>
const typename GLONASS_SpaceNode<FloatT>::float_t GLONASS_SpaceNode<FloatT>::L1_frequency
    = 1602E6; // [Hz]

template <class FloatT>
const typename GLONASS_SpaceNode<FloatT>::float_t GLONASS_SpaceNode<FloatT>::L1_frequency_gap
    = 562.5E3; // [Hz]

template <class FloatT>
const typename GLONASS_SpaceNode<FloatT>::float_t GLONASS_SpaceNode<FloatT>::L2_frequency
    = 1246E6; // [Hz]

template <class FloatT>
const typename GLONASS_SpaceNode<FloatT>::float_t GLONASS_SpaceNode<FloatT>::L2_frequency_gap
    = 437.5E3; // [Hz]

template <class FloatT>
const typename GLONASS_SpaceNode<FloatT>::float_t GLONASS_SpaceNode<FloatT>::SatelliteProperties::Ephemeris::constellation_t::omega_E
    = 0.7292115E-4; // [s^-1]

template <class FloatT>
const typename GLONASS_SpaceNode<FloatT>::int_t GLONASS_SpaceNode<FloatT>::TimeProperties::date_t::days_m[] = {
  31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
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
  POWER_2(-30), // tau_n
  POWER_2(-30), // delta_tau_n
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
