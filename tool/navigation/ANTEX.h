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

/** @file
 * @brief SP3 Reader/Writer, support ver.D
 *
 */

#ifndef __ANTEX_H__
#define __ANTEX_H__

#include <string>
#include <ctime>

#include <vector>
#include <map>

#include "util/text_helper.h"
#include "GPS.h"
#include "SP3.h"
#include "coordinate.h"

template <class FloatT>
struct ANTEX_Product {
  struct per_freq_t {
    FloatT north_east_up[3];
    std::vector<FloatT> noazi; // TODO
    std::map<FloatT, std::vector<FloatT> > azi; // TODO
  };
  struct antenna_t {
    std::string type, serial;
    GPS_Time<FloatT> valid_from;
    GPS_Time<FloatT> valid_until;
    std::map<std::string, per_freq_t> freq;
    std::map<std::string, per_freq_t> freq_rms;
    bool is_satellite(int *sat_id = NULL) const {
      if(serial.size() != 3){return false;}
      int dummy;
      return SP3_Reader<FloatT>::conv_t::sat_id(
          const_cast<std::string &>(serial), 0, 3, (sat_id ? sat_id : &dummy));
    }
  };
  typedef std::vector<antenna_t> prop_t;
  prop_t prop;

  struct eci2ecef_opt_t {
    FloatT delta_UT1;
    FloatT x_p, y_p;
    eci2ecef_opt_t() : delta_UT1(0), x_p(0), y_p(0) {}
  };
  static typename System_XYZ<FloatT>::Rotation eci2ecef(
      const GPS_Time<FloatT> &t_gps,
      const eci2ecef_opt_t &opt = eci2ecef_opt_t() ) {
    // ECI -> ECEF
    // @see https://doi.org/10.1007/978-3-540-78522-4_2 Chapter 2
    // X_ECEF = R_M * R_S * R_N * R_P * X_ECI
    // R_M: polar motion, R_S: Earth rotation, R_N: nutation, R_P: precession

    typedef typename System_XYZ<FloatT>::Rotation rot_t;

    // pp.20 in IERS Conventions (1996) and Eq.(5.2) in Chapter 5 of IERS Conventions (2010) defines
    // t = (TT - 2000 January ld 12h TT) in days/36525,
    // where t is relative time as it computed with a difference of two Terrestrial Time (TT).
    // Note: TT is approximately TAI + 32.184s = GPS + 19 + 32.184.
    FloatT t_date(t_gps.julian_date_2000()/* + (19.0 + 32.184) / 86400*/); // TODO need offset?
    FloatT t(t_date / 36525), t2(t * t), t3(t2 * t); // century and its powered

#define AS2RAD(x) ((x) * (M_PI / 3600 / 180)) // arcseconds to radians
#define DMS2RAD(d, m, s) AS2RAD((FloatT)(s) + (FloatT)(m) * 60 + (FloatT)(d) * 3600)
#define DEG2RAD(d) DMS2RAD(d, 0, 0)
    // astronomical arguments
    struct ast_args_t {
      FloatT f[5]; // rad
      ast_args_t(const FloatT &t){
        // @see Eq.(5.43) in Chapter 5 of IERS Conventions (2010)
        // https://www.iers.org/SharedDocs/Publikationen/EN/IERS/Publications/tn/TechnNote36/tn36_043.pdf#page=25
        static const FloatT fc[][5] = {
          // F1(l): Mean Anomaly of the Moon
          {DEG2RAD(134.96340251), AS2RAD(1717915923.2178), AS2RAD( 31.8792), AS2RAD( 0.051635), AS2RAD(-0.00024470)},
          // F2(l'): Mean Anomaly of the Sun
          {DEG2RAD(357.52910918), AS2RAD( 129596581.0481), AS2RAD( -0.5532), AS2RAD( 0.000136), AS2RAD(-0.00001149)},
          // F3(F)
          {DEG2RAD( 93.27209062), AS2RAD(1739527262.8478), AS2RAD(-12.7512), AS2RAD(-0.001037), AS2RAD( 0.00000417)},
          // F4(D): Mean Elongation of the Moon from the Sun
          {DEG2RAD(297.85019547), AS2RAD(1602961601.2090), AS2RAD( -6.3706), AS2RAD( 0.006593), AS2RAD(-0.00003169)},
          // F5(Omega): Mean Longitude of the Ascending Node of the Moon
          {DEG2RAD(125.04455501), AS2RAD(  -6962890.2665), AS2RAD(  7.4722), AS2RAD( 0.007702), AS2RAD(-0.00005939)},
        };
        FloatT tt[] = {1, t, std::pow(t, 2), std::pow(t, 3), std::pow(t, 4)};
        for(std::size_t i(0); i < sizeof(f) / sizeof(f[0]); i++){
          f[i] = 0;
          for(std::size_t j(0); j < sizeof(fc[0]) / sizeof(fc[0][0]); ++j){
            f[i] += fc[i][j] * tt[j];
          }
          f[i] = std::fmod(f[i], M_PI * 2);
        }

      }
    } ast_args(t);

    // IAU 1976 precession
    FloatT
        zeta (t * AS2RAD(2306.2181) + t2 * AS2RAD(0.30188) + t3 * AS2RAD(0.017998)), // same as RTKlib
        theta(t * AS2RAD(2004.3109) - t2 * AS2RAD(0.42665) - t3 * AS2RAD(0.041833)),
        z    (t * AS2RAD(2306.2181) + t2 * AS2RAD(1.09468) + t3 * AS2RAD(0.018203));

    // Precession=R3(-z)*R2(theta)*R3(-zeta)
    rot_t r_p(rot_t().then_z(-zeta).then_y(theta).then_z(-z));

    struct nut_iau1980_t {
      FloatT dpsi, deps;
      nut_iau1980_t(const FloatT &t, const FloatT (&f)[5])
          : dpsi(0), deps(0){
        static const struct {
          int arg[5]; // l, l', F, D, Omega
          FloatT period; // (days)
          int lng_A; // longitude(1E-4)
          FloatT lng_At; // [1/century]
          int obl_B; // obliquity(1E-4)
          FloatT obl_Bt; // [1/century]
        } nut[106] = {
          {{   0,   0,   0,   0,   1},-6798.4, -171996, -174.2, 92025,   8.9},
          {{   0,   0,   2,  -2,   2},  182.6,  -13187,   -1.6,  5736,  -3.1},
          {{   0,   0,   2,   0,   2},   13.7,   -2274,   -0.2,   977,  -0.5},
          {{   0,   0,   0,   0,   2},-3399.2,    2062,    0.2,  -895,   0.5},
          {{   0,  -1,   0,   0,   0}, -365.3,   -1426,    3.4,    54,  -0.1},
          {{   1,   0,   0,   0,   0},   27.6,     712,    0.1,    -7,   0.0},
          {{   0,   1,   2,  -2,   2},  121.7,    -517,    1.2,   224,  -0.6},
          {{   0,   0,   2,   0,   1},   13.6,    -386,   -0.4,   200,   0.0},
          {{   1,   0,   2,   0,   2},    9.1,    -301,    0.0,   129,  -0.1},
          {{   0,  -1,   2,  -2,   2},  365.2,     217,   -0.5,   -95,   0.3},
          {{  -1,   0,   0,   2,   0},   31.8,     158,    0.0,    -1,   0.0},
          {{   0,   0,   2,  -2,   1},  177.8,     129,    0.1,   -70,   0.0},
          {{  -1,   0,   2,   0,   2},   27.1,     123,    0.0,   -53,   0.0},
          {{   1,   0,   0,   0,   1},   27.7,      63,    0.1,   -33,   0.0},
          {{   0,   0,   0,   2,   0},   14.8,      63,    0.0,    -2,   0.0},
          {{  -1,   0,   2,   2,   2},    9.6,     -59,    0.0,    26,   0.0},
          {{  -1,   0,   0,   0,   1},  -27.4,     -58,   -0.1,    32,   0.0},
          {{   1,   0,   2,   0,   1},    9.1,     -51,    0.0,    27,   0.0},
          {{  -2,   0,   0,   2,   0}, -205.9,     -48,    0.0,     1,   0.0},
          {{  -2,   0,   2,   0,   1}, 1305.5,      46,    0.0,   -24,   0.0},
          {{   0,   0,   2,   2,   2},    7.1,     -38,    0.0,    16,   0.0},
          {{   2,   0,   2,   0,   2},    6.9,     -31,    0.0,    13,   0.0},
          {{   2,   0,   0,   0,   0},   13.8,      29,    0.0,    -1,   0.0},
          {{   1,   0,   2,  -2,   2},   23.9,      29,    0.0,   -12,   0.0},
          {{   0,   0,   2,   0,   0},   13.6,      26,    0.0,    -1,   0.0},
          {{   0,   0,   2,  -2,   0},  173.3,     -22,    0.0,     0,   0.0},
          {{  -1,   0,   2,   0,   1},   27.0,      21,    0.0,   -10,   0.0},
          {{   0,   2,   0,   0,   0},  182.6,      17,   -0.1,     0,   0.0},
          {{   0,   2,   2,  -2,   2},   91.3,     -16,    0.1,     7,   0.0},
          {{  -1,   0,   0,   2,   1},   32.0,      16,    0.0,    -8,   0.0},
          {{   0,   1,   0,   0,   1},  386.0,     -15,    0.0,     9,   0.0},
          {{   1,   0,   0,  -2,   1},  -31.7,     -13,    0.0,     7,   0.0},
          {{   0,  -1,   0,   0,   1}, -346.6,     -12,    0.0,     6,   0.0},
          {{   2,   0,  -2,   0,   0},-1095.2,      11,    0.0,     0,   0.0},
          {{  -1,   0,   2,   2,   1},    9.5,     -10,    0.0,     5,   0.0},
          {{   1,   0,   2,   2,   2},    5.6,      -8,    0.0,     3,   0.0},
          {{   0,  -1,   2,   0,   2},   14.2,      -7,    0.0,     3,   0.0},
          {{   0,   0,   2,   2,   1},    7.1,      -7,    0.0,     3,   0.0},
          {{   1,   1,   0,  -2,   0},  -34.8,      -7,    0.0,     0,   0.0},
          {{   0,   1,   2,   0,   2},   13.2,       7,    0.0,    -3,   0.0},
          {{  -2,   0,   0,   2,   1}, -199.8,      -6,    0.0,     3,   0.0},
          {{   0,   0,   0,   2,   1},   14.8,      -6,    0.0,     3,   0.0},
          {{   2,   0,   2,  -2,   2},   12.8,       6,    0.0,    -3,   0.0},
          {{   1,   0,   0,   2,   0},    9.6,       6,    0.0,     0,   0.0},
          {{   1,   0,   2,  -2,   1},   23.9,       6,    0.0,    -3,   0.0},
          {{   0,   0,   0,  -2,   1},  -14.7,      -5,    0.0,     3,   0.0},
          {{   0,  -1,   2,  -2,   1},  346.6,      -5,    0.0,     3,   0.0},
          {{   2,   0,   2,   0,   1},    6.9,      -5,    0.0,     3,   0.0},
          {{   1,  -1,   0,   0,   0},   29.8,       5,    0.0,     0,   0.0},
          {{   1,   0,   0,  -1,   0},  411.8,      -4,    0.0,     0,   0.0},
          {{   0,   0,   0,   1,   0},   29.5,      -4,    0.0,     0,   0.0},
          {{   0,   1,   0,  -2,   0},  -15.4,      -4,    0.0,     0,   0.0},
          {{   1,   0,  -2,   0,   0},  -26.9,       4,    0.0,     0,   0.0},
          {{   2,   0,   0,  -2,   1},  212.3,       4,    0.0,    -2,   0.0},
          {{   0,   1,   2,  -2,   1},  119.6,       4,    0.0,    -2,   0.0},
          {{   1,   1,   0,   0,   0},   25.6,      -3,    0.0,     0,   0.0},
          {{   1,  -1,   0,  -1,   0},-3232.9,      -3,    0.0,     0,   0.0},
          {{  -1,  -1,   2,   2,   2},    9.8,      -3,    0.0,     1,   0.0},
          {{   0,  -1,   2,   2,   2},    7.2,      -3,    0.0,     1,   0.0},
          {{   1,  -1,   2,   0,   2},    9.4,      -3,    0.0,     1,   0.0},
          {{   3,   0,   2,   0,   2},    5.5,      -3,    0.0,     1,   0.0},
          {{  -2,   0,   2,   0,   2}, 1615.7,      -3,    0.0,     1,   0.0},
          {{   1,   0,   2,   0,   0},    9.1,       3,    0.0,     0,   0.0},
          {{  -1,   0,   2,   4,   2},    5.8,      -2,    0.0,     1,   0.0},
          {{   1,   0,   0,   0,   2},   27.8,      -2,    0.0,     1,   0.0},
          {{  -1,   0,   2,  -2,   1},  -32.6,      -2,    0.0,     1,   0.0},
          {{   0,  -2,   2,  -2,   1}, 6786.3,      -2,    0.0,     1,   0.0},
          {{  -2,   0,   0,   0,   1},  -13.7,      -2,    0.0,     1,   0.0},
          {{   2,   0,   0,   0,   1},   13.8,       2,    0.0,    -1,   0.0},
          {{   3,   0,   0,   0,   0},    9.2,       2,    0.0,     0,   0.0},
          {{   1,   1,   2,   0,   2},    8.9,       2,    0.0,    -1,   0.0},
          {{   0,   0,   2,   1,   2},    9.3,       2,    0.0,    -1,   0.0},
          {{   1,   0,   0,   2,   1},    9.6,      -1,    0.0,     0,   0.0},
          {{   1,   0,   2,   2,   1},    5.6,      -1,    0.0,     1,   0.0},
          {{   1,   1,   0,  -2,   1},  -34.7,      -1,    0.0,     0,   0.0},
          {{   0,   1,   0,   2,   0},   14.2,      -1,    0.0,     0,   0.0},
          {{   0,   1,   2,  -2,   0},  117.5,      -1,    0.0,     0,   0.0},
          {{   0,   1,  -2,   2,   0}, -329.8,      -1,    0.0,     0,   0.0},
          {{   1,   0,  -2,   2,   0},   23.8,      -1,    0.0,     0,   0.0},
          {{   1,   0,  -2,  -2,   0},   -9.5,      -1,    0.0,     0,   0.0},
          {{   1,   0,   2,  -2,   0},   32.8,      -1,    0.0,     0,   0.0},
          {{   1,   0,   0,  -4,   0},  -10.1,      -1,    0.0,     0,   0.0},
          {{   2,   0,   0,  -4,   0},  -15.9,      -1,    0.0,     0,   0.0},
          {{   0,   0,   2,   4,   2},    4.8,      -1,    0.0,     0,   0.0},
          {{   0,   0,   2,  -1,   2},   25.4,      -1,    0.0,     0,   0.0},
          {{  -2,   0,   2,   4,   2},    7.3,      -1,    0.0,     1,   0.0},
          {{   2,   0,   2,   2,   2},    4.7,      -1,    0.0,     0,   0.0},
          {{   0,  -1,   2,   0,   1},   14.2,      -1,    0.0,     0,   0.0},
          {{   0,   0,  -2,   0,   1},  -13.6,      -1,    0.0,     0,   0.0},
          {{   0,   0,   4,  -2,   2},   12.7,       1,    0.0,     0,   0.0},
          {{   0,   1,   0,   0,   2},  409.2,       1,    0.0,     0,   0.0},
          {{   1,   1,   2,  -2,   2},   22.5,       1,    0.0,    -1,   0.0},
          {{   3,   0,   2,  -2,   2},    8.7,       1,    0.0,     0,   0.0},
          {{  -2,   0,   2,   2,   2},   14.6,       1,    0.0,    -1,   0.0},
          {{  -1,   0,   0,   0,   2},  -27.3,       1,    0.0,    -1,   0.0},
          {{   0,   0,  -2,   2,   1}, -169.0,       1,    0.0,     0,   0.0},
          {{   0,   1,   2,   0,   1},   13.1,       1,    0.0,     0,   0.0},
          {{  -1,   0,   4,   0,   2},    9.1,       1,    0.0,     0,   0.0},
          {{   2,   1,   0,  -2,   0},  131.7,       1,    0.0,     0,   0.0},
          {{   2,   0,   0,   2,   0},    7.1,       1,    0.0,     0,   0.0},
          {{   2,   0,   2,  -2,   1},   12.8,       1,    0.0,    -1,   0.0},
          {{   2,   0,  -2,   0,   1}, -943.2,       1,    0.0,     0,   0.0},
          {{   1,  -1,   0,  -2,   0},  -29.3,       1,    0.0,     0,   0.0},
          {{  -1,   0,   0,   1,   1}, -388.3,       1,    0.0,     0,   0.0},
          {{  -1,  -1,   0,   2,   1},   35.0,       1,    0.0,     0,   0.0},
          {{   0,   1,   0,   1,   0},   27.3,       1,    0.0,     0,   0.0},
        };
        for(std::size_t i(0); i < sizeof(nut) / sizeof(nut[0]); i++){
          FloatT ang(0);
          for(int j(0); j < 5; j++){
            ang += f[j] * nut[i].arg[j];
          }
          dpsi += (t * nut[i].lng_At + nut[i].lng_A) * std::sin(ang);
          deps += (t * nut[i].obl_Bt + nut[i].obl_B) * std::cos(ang);
        }
        dpsi = AS2RAD(dpsi * 1E-4); // 0.1 mas -> rad
        deps = AS2RAD(deps * 1E-4);
      }
    } nut_iau1980(t, ast_args.f);

    FloatT eps(AS2RAD(84381.448)
        - t  * AS2RAD(46.8150)
        - t2 * AS2RAD( 0.00059)
        + t3 * AS2RAD( 0.001813));

    // IAU 1980 nutation
    // Nutation=R1(-epsilon-de)*R3(dpsi)*R1(epsilon)
    rot_t r_n(rot_t().then_x(eps).then_z(-nut_iau1980.dpsi).then_x(-eps - nut_iau1980.deps));

    rot_t r_s;
#if 1
    { // Greenwich apparent sidereal time (rad)
      // IERS Conventions 1996 Chapter.5 pp.21
      FloatT gmst(
          t_gps.greenwich_mean_sidereal_time_sec_ires1996(opt.delta_UT1)
            * (M_PI * 2 / 86400)); // [rad]
      FloatT gast(gmst // GAST or GST
          + nut_iau1980.dpsi * std::cos(eps)
          + AS2RAD(0.00264) * std::sin(ast_args.f[4])
          + AS2RAD(0.000063) * std::sin(ast_args.f[4] * 2));
      r_s.then_z(gast);
    }
#else
    // Eq.(5.14) of IERS 2010(Technical Note No.36)
    // TODO t_date will be corrected to make it based on UT1
    r_s.then_z(t_gps.earth_rotation_angle(opt.delta_UT1));
#endif

    // Polar motion
    rot_t r_m(rot_t().then_x(-opt.y_p).then_y(-opt.x_p));

    // eci to ecef transformation matrix
    return r_p.then(r_n).then(r_s).then(r_m);
#undef DEG2RAD
#undef DMS2RAD
#undef AS2RAD
  }

  static Vector3<FloatT> sun_direction_ecef(
      const GPS_Time<FloatT> &t,
      FloatT *r = NULL){
    // @see https://en.wikipedia.org/wiki/Position_of_the_Sun
    // @see https://astronomy.stackexchange.com/a/37199
#define DEG2RAD(deg) ((deg) / 180 * M_PI)
    FloatT n(t.julian_date_2000()/* + 19.0 / 86400*/); // GPS_Time -> Julian day -> J2000.0
    FloatT L(DEG2RAD(280.4606184) + DEG2RAD(36000.77005361 / 36525) * n); // mean longitude
    FloatT g(DEG2RAD(357.5277233) + DEG2RAD(35999.05034 / 36525) * n); // mean anomaly
    FloatT lambda(L
        + DEG2RAD(1.914666471) * std::sin(g)
        + DEG2RAD(0.918994643) * std::sin(g * 2)); // ecliptic longitude of the Sun
    FloatT clambda(std::cos(lambda)), slambda(std::sin(lambda));
    FloatT epsilon(DEG2RAD(23.43929) - DEG2RAD(46.8093/3600/36525) * n); // eccentricity
    FloatT cepsilon(std::cos(epsilon)), sepsilon(std::sin(epsilon));
    if(r){
      *r = (1.000140612 - 0.016708617 * std::cos(g) - 0.000139589 * std::cos(g * 2))
          * 1.495978707E11; // AU -> meter
    }
    FloatT dir[] = {clambda, cepsilon * slambda, sepsilon * slambda};
    eci2ecef(t).apply(dir); // ECI -> ECEF
    return Vector3<FloatT>(dir);
#undef DEG2RAD
  }

  static System_XYZ<FloatT> sun_position_ecef(const GPS_Time<FloatT> &t){
    FloatT r;
    return System_XYZ<FloatT>(sun_direction_ecef(t, &r) * r);
  }

  int move_to_antenna_position(
      SP3_Product<FloatT> &sp3,
      const std::map<char, std::string> &freq_table = std::map<char, std::string>()) const {

    static const struct default_opt_t {
      std::map<char, std::string> freq_table;
      default_opt_t() : freq_table() {
        freq_table['G'] = "G01"; // GPS L1; others are "G02"(L2), "G05"(L5)
        freq_table['R'] = "R01"; // GLONASS G1; other is "R02"(G2)
        freq_table['E'] = "E01"; // Galileo E1; others are "E05"(E5a), "E07"(E5b), "E08"(E5a+E5b), "E06"(E6)
        freq_table['C'] = "C01"; // Compass E1; others are "C02"(E2), "C07"(E5b), "C06"(E6)
        freq_table['J'] = "J01"; // QZSS L1; others are "J02"(L2), "J05"(L5), "J06"(LEX)
        freq_table['S'] = "S01"; // SBAS L1; other is "S05"(L5)
      }
    } default_opt;

    std::map<char, std::string> freq_table2(default_opt.freq_table);
    freq_table2.insert(freq_table.begin(), freq_table.end());

    int moved(0);
    for(typename prop_t::const_iterator it_ant(prop.begin()), it_ant_end(prop.end());
        it_ant != it_ant_end; ++it_ant){
      int sat_id;
      if(!it_ant->is_satellite(&sat_id)){continue;} // If not satellite antenna, then skip.

      typename std::map<std::string, per_freq_t>::const_iterator it_freq(
          it_ant->freq.find(freq_table2[it_ant->serial[0]]));
      if(it_freq == it_ant->freq.end()){continue;} // If target frequency is not registered, then skip.

      typename SP3_Product<FloatT>::satellites_t::iterator it_sat(sp3.satellites.find(sat_id));
      if(it_sat == sp3.satellites.end()){continue;} // If satellite is not registered, then skip.

      for(typename SP3_Product<FloatT>::per_satellite_t::history_t::iterator
            it_pos(it_sat->second.pos_history.begin()), it_pos_end(it_sat->second.pos_history.end());
          it_pos != it_pos_end; ++it_pos){ // ascending order of target time
        if(it_pos->first > it_ant->valid_until){break;} // skip if target time of position is after valid_until.
        if(it_pos->first < it_ant->valid_from){continue;} // skip if target time of position is before valid_from.

        // Calculate satellite frame unit vector in ECEF coordinate
        Vector3<FloatT> z_dir(it_pos->second.xyz / -it_pos->second.xyz.abs());
        Vector3<FloatT> y_dir(z_dir * sun_direction_ecef(it_pos->first));
        y_dir /= y_dir.abs();
        Vector3<FloatT> x_dir(y_dir * z_dir);

        // correction
        const FloatT (&xyz)[3](it_freq->second.north_east_up);
        Vector3<FloatT> antenna_ecef(x_dir * xyz[0] + y_dir * xyz[1] + z_dir * xyz[2]);
        it_pos->second.xyz += antenna_ecef;

        ++moved;
      }
    }

    return moved;
  }
};

template <class FloatT>
class ANTEX_Reader {
  protected:
    typename TextHelper<>::crlf_stream_t src;
  public:
    ANTEX_Reader(std::istream &in) : src(in) {}

    bool has_next() {
      return !(src.eof() || src.fail());
    }

    struct type_serial_t {
      char type[20];
      char serial_or_prn[20];
      char blank_or_code[10];
      char blank_or_cospar[10];
      template <std::size_t N>
      static std::string to_string(const char (&c)[N]) {
        std::string buf(c, N);
        return buf.substr(0, buf.find_last_not_of(' ') + 1);
      }
    };
    struct time_t {
      int year, month, day_of_month, hour, minute;
      FloatT second;
      operator std::tm() const {
        std::tm res = {
          (int)second, minute, hour, day_of_month, month - 1, year - 1900, 0, 0, 0
        };
        std::mktime(&res);
        return res;
      }
      operator GPS_Time<FloatT>() const {
        return GPS_Time<FloatT>(std::tm(*this));
      }
    };
    struct freq_t {
      char freq_name[3];
    };
    struct neu_t {
      FloatT values[3];
    };

    struct parsed_t {
      enum {
        ANTEX_VERSION_SYST,
        PCV_TYPE_REFANT,
        COMMENT,
        END_OF_HEADER,
        START_OF_ANTENNA,
        TYPE_SERIAL_NO,
        METH_BY_NUM_DATE,
        DAZI,
        ZEN1_ZEN2_DZEN,
        NUM_OF_FREQUENCIES,
        VALID_FROM,
        VALID_UNTIL,
        SINEX_CODE,
        START_OF_FREQUENCY,
        NORTH_EAST_UP,
        NOAZI_VALUES,
        DAZI_VALUES,
        END_OF_FREQUENCY,
        START_OF_FREQ_RMS,
        END_OF_FREQ_RMS,
        END_OF_ANTENNA,
        IGNORABLE,
        UNKNOWN,
      } type;
      union {
        type_serial_t type_serial;
        time_t time;
        freq_t freq;
        neu_t neu;
      } item;
    };

    static const typename TextHelper<>::convert_item_t type_serial_items[4];
    static const typename TextHelper<>::convert_item_t time_items[6];
    static const typename TextHelper<>::convert_item_t freq_items[1];
    static const typename TextHelper<>::convert_item_t neu_items[3];

    parsed_t parse_line() {
      parsed_t res = {parsed_t::UNKNOWN, {0}};

      char buf[0x400] = {0};
      src.getline(buf, sizeof(buf));
      std::string line(buf);

      if(line.size() == 0){
        res.type = parsed_t::IGNORABLE;
        return res;
      }
      line = line.substr(0, line.find_last_not_of(" ") + 1);

      if(line.size() > 60){
        std::string label(line.substr(60));

        if(label.compare("ANTEX VERSION / SYST") == 0){
          res.type = parsed_t::ANTEX_VERSION_SYST;
          return res;
        }
        if(label.compare("PCV TYPE / REFANT") == 0){
          res.type = parsed_t::PCV_TYPE_REFANT;
          return res;
        }
        if(label.compare("COMMENT") == 0){
          res.type = parsed_t::COMMENT;
          return res;
        }
        if(label.compare("END OF HEADER") == 0){
          res.type = parsed_t::END_OF_HEADER;
          return res;
        }
        if(label.compare("START OF ANTENNA") == 0){
          res.type = parsed_t::START_OF_ANTENNA;
          return res;
        }
        if(label.compare("TYPE / SERIAL NO") == 0){
          TextHelper<>::str2val(type_serial_items, line, &res.item);
          res.type = parsed_t::TYPE_SERIAL_NO;
          return res;
        }
        if(label.compare("METH / BY / # / DATE") == 0){
          res.type = parsed_t::METH_BY_NUM_DATE;
          return res;
        }
        if(label.compare("DAZI") == 0){
          res.type = parsed_t::DAZI;
          return res;
        }
        if(label.compare("ZEN1 / ZEN2 / DZEN") == 0){
          res.type = parsed_t::ZEN1_ZEN2_DZEN;
          return res;
        }
        if(label.compare("# OF FREQUENCIES") == 0){
          res.type = parsed_t::NUM_OF_FREQUENCIES;
          return res;
        }
        if(label.compare("VALID FROM") == 0){
          TextHelper<>::str2val(time_items, line, &res.item);
          res.type = parsed_t::VALID_FROM;
          return res;
        }
        if(label.compare("VALID UNTIL") == 0){
          TextHelper<>::str2val(time_items, line, &res.item);
          res.type = parsed_t::VALID_UNTIL;
          return res;
        }
        if(label.compare("SINEX CODE") == 0){
          res.type = parsed_t::SINEX_CODE;
          return res;
        }
        if(label.compare("START OF FREQUENCY") == 0){
          TextHelper<>::str2val(freq_items, line, &res.item);
          res.type = parsed_t::START_OF_FREQUENCY;
          return res;
        }
        if(label.compare("NORTH / EAST / UP") == 0){
          TextHelper<>::str2val(neu_items, line, &res.item);
          res.type = parsed_t::NORTH_EAST_UP;
          return res;
        }
        if(label.compare("END OF FREQUENCY") == 0){
          res.type = parsed_t::END_OF_FREQUENCY;
          return res;
        }
        if(label.compare("START OF FREQ RMS") == 0){
          TextHelper<>::str2val(freq_items, line, &res.item);
          res.type = parsed_t::START_OF_FREQ_RMS;
          return res;
        }
        if(label.compare("END OF FREQ RMS") == 0){
          res.type = parsed_t::END_OF_FREQ_RMS;
          return res;
        }
        if(label.compare("END OF ANTENNA") == 0){
          res.type = parsed_t::END_OF_ANTENNA;
          return res;
        }
      }
      if(line.substr(3, 5).compare("NOAZI") == 0){
        // (Values of a non-azimuth-dependent pattern)
        // or (Rms values of a non-azimuth-dependent pattern)
        res.type = parsed_t::NOAZI_VALUES;
        return res;
      }
      for(double d; !!(std::stringstream(line.substr(0, 8)) >> d); ){
        // (Values of an azimuth-dependent pattern)
        // or (Rms Values of an azimuth-dependent pattern)
        res.type = parsed_t::DAZI_VALUES;
        return res;
      }

      res.type = parsed_t::UNKNOWN;
      return res;
    }

    static int read_all(std::istream &in, ANTEX_Product<FloatT> &dst) {
      ANTEX_Reader<FloatT> src(in);
      enum {
        INITIAL_STATE = 1,
        ON_HEADER,
        FREE_STATE,
        ON_ANTENNA,
        ON_FREQUENCY,
        ON_FREQUENCY_RMS,
      } state = INITIAL_STATE;

      typedef typename ANTEX_Product<FloatT>::antenna_t antenna_t;
      antenna_t *antenna;
      typedef typename ANTEX_Product<FloatT>::per_freq_t per_freq_t;
      per_freq_t *per_freq(NULL); // NULL fot suppression of GCC uninitialized variable warning

      while(src.has_next()){
        parsed_t parsed(src.parse_line());
        if(parsed.type == parsed_t::IGNORABLE){continue;}
        switch(state){
          case INITIAL_STATE:
            if(parsed.type != parsed_t::ANTEX_VERSION_SYST){
              return -INITIAL_STATE; // error
            }
            state = ON_HEADER;
            break;
          case ON_HEADER:
            switch(parsed.type){
              case parsed_t::COMMENT: break;
              case parsed_t::PCV_TYPE_REFANT: break;
              case parsed_t::END_OF_HEADER:
                state = FREE_STATE;
                break;
              default: // error
                return -ON_HEADER;
            }
            break;
          case FREE_STATE:
            if(parsed.type != parsed_t::START_OF_ANTENNA){
              return -FREE_STATE; // error
            }
            state = ON_ANTENNA;
            dst.prop.resize(dst.prop.size() + 1);
            antenna = &dst.prop.back();
            break;
          case ON_ANTENNA:
            switch(parsed.type){
              case parsed_t::TYPE_SERIAL_NO: {
                antenna->type = type_serial_t::to_string(parsed.item.type_serial.type);
                antenna->serial = type_serial_t::to_string(parsed.item.type_serial.serial_or_prn);
                antenna->valid_until = GPS_Time<FloatT>::now();
                break;
              }
              case parsed_t::METH_BY_NUM_DATE: break;
              case parsed_t::DAZI: break;
              case parsed_t::ZEN1_ZEN2_DZEN: break;
              case parsed_t::NUM_OF_FREQUENCIES: break;
              case parsed_t::VALID_FROM:
                antenna->valid_from = parsed.item.time;
                break;
              case parsed_t::VALID_UNTIL:
                antenna->valid_until = parsed.item.time;
                break;
              case parsed_t::SINEX_CODE: break;
              case parsed_t::COMMENT: break;
              case parsed_t::START_OF_FREQUENCY:
                state = ON_FREQUENCY;
                per_freq = &antenna->freq[
                     std::string(parsed.item.freq.freq_name, sizeof(parsed.item.freq.freq_name))];
                break;
              case parsed_t::START_OF_FREQ_RMS:
                per_freq = &antenna->freq_rms[
                     std::string(parsed.item.freq.freq_name, sizeof(parsed.item.freq.freq_name))];
                state = ON_FREQUENCY_RMS;
                break;
              case parsed_t::END_OF_ANTENNA:
                state = FREE_STATE;
                break;
              default: // error
                return -ON_ANTENNA;
            }
            break;
          case ON_FREQUENCY:
          case ON_FREQUENCY_RMS:
            switch(parsed.type){
              case parsed_t::NORTH_EAST_UP:
                for(std::size_t i(0);
                    i < sizeof(per_freq->north_east_up) / sizeof(per_freq->north_east_up[0]);
                    ++ i){
                  per_freq->north_east_up[i] = parsed.item.neu.values[i] * 1E-3; // mm -> m
                }
                break;
              case parsed_t::NOAZI_VALUES: break; // TODO variable size, implement callback of parse_line()?
              case parsed_t::DAZI_VALUES: break; // TODO
              case parsed_t::END_OF_FREQUENCY:
                if(state == ON_FREQUENCY){state = ON_ANTENNA; break;}
                return -state;
              case parsed_t::END_OF_FREQ_RMS:
                if(state != ON_FREQUENCY_RMS){state = ON_ANTENNA; break;}
                return -state;
              default: // error
                return -state;
            }
            break;
        }
      }
      return dst.prop.size();
    }
};

#define GEN_C(offset, length, container_type, container_member) \
    {TextHelper<>::template format_t<char>::c, offset, length, \
      offsetof(container_type, container_member), 0}
#define GEN_I(offset, length, container_type, container_member) \
    {TextHelper<>::template format_t<int>::d, offset, length, \
      offsetof(container_type, container_member), 0}
#define GEN_F(offset, length, container_type, container_member, precision) \
    {TextHelper<>::template format_t<FloatT>::f, offset, length, \
      offsetof(container_type, container_member), precision}

template <class FloatT>
const typename TextHelper<>::convert_item_t ANTEX_Reader<FloatT>::type_serial_items[4] = {
  GEN_C( 0, 20, type_serial_t, type), // not null-terminated, following the same
  GEN_C(20, 20, type_serial_t, serial_or_prn),
  GEN_C(40, 10, type_serial_t, blank_or_code),
  GEN_C(50, 10, type_serial_t, blank_or_cospar),
};

template <class FloatT>
const typename TextHelper<>::convert_item_t ANTEX_Reader<FloatT>::time_items[6] = {
  GEN_I( 0, 6, time_t, year),
  GEN_I( 6, 6, time_t, month),
  GEN_I(12, 6, time_t, day_of_month),
  GEN_I(18, 6, time_t, hour),
  GEN_I(24, 6, time_t, minute),
  GEN_F(30, 13, time_t, second, 7),
};

template <class FloatT>
const typename TextHelper<>::convert_item_t ANTEX_Reader<FloatT>::freq_items[1] = {
  GEN_C( 3,  3, freq_t, freq_name),
};

template <class FloatT>
const typename TextHelper<>::convert_item_t ANTEX_Reader<FloatT>::neu_items[3] = {
  GEN_F( 0, 10, neu_t, values[0], 2),
  GEN_F(10, 10, neu_t, values[1], 2),
  GEN_F(20, 10, neu_t, values[2], 2),
};

#undef GEN_C
#undef GEN_I
#undef GEN_F

#endif /* #define __ANTEX_H__ */
