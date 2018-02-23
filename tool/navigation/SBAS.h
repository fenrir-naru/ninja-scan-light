/**
 * @file Satellite based augmentation system (SBAS)
 * @see DO-229D
 */

/*
 * Copyright (c) 2018, M.Naruoka (fenrir)
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

#ifndef __SBAS_H__
#define __SBAS_H__

#include <cmath>

#include "GPS.h"

template <class FloatT = double>
class SBAS_Signal {
  public:
    typedef FloatT float_t;
    class G2 : public GPS_Signal<float_t>::PRN {
      public:
        typedef typename GPS_Signal<float_t>::PRN super_t;
        G2(const int &initial_g2) : super_t((unsigned long)initial_g2) {}
        ~G2(){}
        bool get() const {return super_t::_content[9];}
        void next(){
          bool tmp(super_t::_content[1]
                     ^ super_t::_content[2]
                     ^ super_t::_content[5]
                     ^ super_t::_content[7]
                     ^ super_t::_content[8]
                     ^ super_t::_content[9]);
          super_t::_content <<= 1;
          super_t::_content[0] = tmp;
        }
    };
};

template <class FloatT = double>
class SBAS_SpaceNode {
  public:
    typedef FloatT float_t;
    typedef GPS_SpaceNode<float_t> gps_space_node_t;

#define type_copy(type) \
typedef typename gps_space_node_t::type type
    type_copy(gps_time_t);
    type_copy(xyz_t);

    type_copy(u8_t);
    type_copy(s8_t);
    type_copy(u16_t);
    type_copy(s16_t);
    type_copy(u32_t);
    type_copy(s32_t);

    type_copy(int_t);
    type_copy(uint_t);
#undef type_copy

    struct RangingCode {
      int prn;
      int g2_delay_chips;
      int initial_g2;
      const char *name;
      typename SBAS_Signal<float_t>::G2 get_G2() const {
        return typename SBAS_Signal<float_t>::G2(initial_g2);
      }
    };
    static const RangingCode ranging_codes[];

    enum MessageType {
      DONT_USE = 0,
      PRN_MASK = 1,
      FAST_CORRECTION_2 = 2,
      FAST_CORRECTION_3 = 3,
      FAST_CORRECTION_4 = 4,
      FAST_CORRECTION_5 = 5,
      INTEGRITY_INFORMATION = 6,
      FAST_CORRECTION_DEGRADATION = 7,
      GEO_NAVIGATION = 9,
      DEGRADATION_PARAMS = 10,
      SBAS_NETWORK_TIME_UTC_OFFSET_PARAMS = 12,
      GEO_SAT_ALNAMACS = 17,
      IONO_GRID_POINT_MASKS = 18,
      MIXED_CORRECTION_FAST_AND_LONG_TERM = 24,
      LONG_TERM_CORRECTION = 25,
      IONO_DELAY_CORRECTION = 26,
      SERVICE_MESSAGE = 27,
      CLOCK_EPHEMERIS_COV_MAT = 28,
      INTERNAL_TEST_MESSAGE = 62,
      NULL_MESSAGES = 63,
    }; ///< @see Table A-3

    struct DataBlock {
      template <class T>
      static T bits2num(const char *buf, const uint_t &index){
        std::div_t aligned(std::div(index, 8));
        T res((u8_t)buf[aligned.quot++]);
        for(int i(0); i < sizeof(T) - 1; ++i){
          res <<= 8;
          res |= (u8_t)buf[aligned.quot++];
        }
        if(aligned.rem > 0){
          res = ((res << aligned.rem) | (((u8_t)buf[aligned.quot]) >> (8 - aligned.rem)));
        }
        return res;
      }
      template <class T>
      static T bits2num(const char *buf, const uint_t &index, const uint_t &length){
        return (bits2num<T>(buf, index) >> ((sizeof(T) * 8) - length));
      }

#define convert_u(bits, offset_bits, length, name) \
static u ## bits ## _t name(const char *buf){ \
  return bits2num<u ## bits ## _t>(buf, offset_bits, length); \
}
#define convert_s(bits, offset_bits, length, name) \
static s ## bits ## _t name(const char *buf){ \
  return ((s ## bits ## _t)bits2num<u ## bits ## _t>(buf, offset_bits, length)) \
      >> (bits - length); \
}
#define convert_u_ch(bits, offset_bits, length, ch_offset_bits, name) \
static u ## bits ## _t name(const char *buf, const uint_t &ch){ \
  return bits2num<u ## bits ## _t>(buf, offset_bits + (ch_offset_bits * ch), length); \
}
#define convert_s_ch(bits, offset_bits, length, ch_offset_bits, name) \
static s ## bits ## _t name(const char *buf, const uint_t &ch){ \
  return ((s ## bits ## _t)bits2num<u ## bits ## _t>(buf, offset_bits + (ch_offset_bits * ch), length)) \
      >> (bits - length); \
}
      convert_u(8, 0, 8, preamble);
      convert_u(8, 8, 6, message_type);

      struct Type18 { ///< @see Table A-15 IONO_GRID_POINT_MASKS
        convert_u(8, 14, 4, broadcasted_bands);
        convert_u(8, 18, 4, band);
        convert_u(8, 22, 2, iodi);
        struct mask_t {
          u8_t valid;
          union {
            u8_t linear[201];
            u8_t block[14][15];
          };
        };
        static mask_t mask(const char *buf){
          mask_t res = {0};
          buf = &buf[2]; // 24 bits shift
          // [mask7, mask6, .., mask0], [mask15, mask14, .., mask8], ...
          u8_t compared(0);
          for(int i(0); i < 201; ++i, compared >>= 1){
            if(compared == 0){ // rotate
              compared = 0x80;
              buf++;
            }
            if((u8_t)(*buf) & compared){
              res.linear[res.valid++] = i;
            }
          }
          return res;
        }
      };

      struct Type26 { ///< @see Table A-16 IONO_DELAY_CORRECTION
        convert_u(8, 14, 4, band);
        convert_u(8, 18, 4, block_id);
        convert_u_ch(16, 22, 9, 13, delay);
        convert_u_ch( 8, 31, 4, 13, error_indicator);
        convert_u(8, 22 + (13 * 15), 4, iodi);
      };

      struct Type9 { ///< @see Table A-18 GEO_NAVIGATION
        convert_u(16, 22, 13, t0);
        convert_u( 8, 35,  4, ura);
        convert_s(32, 39, 30, x);
        convert_s(32, 69, 30, y);
        convert_s(32, 99, 25, z);
        convert_s(32, 124, 17, dx);
        convert_s(32, 141, 17, dy);
        convert_s(32, 158, 18, dz);
        convert_s(16, 176, 10, ddx);
        convert_s(16, 186, 10, ddy);
        convert_s(16, 196, 10, ddz);
        convert_s(16, 206, 12, a_Gf0);
        convert_s( 8, 218,  8, a_Gf1);
      };

      struct Type17 { ///< @see Table A-17 GEO_SAT_ALNAMACS
        convert_u_ch( 8, 14,  2, 67, id);
        convert_u_ch( 8, 16,  8, 67, prn);
        convert_u_ch( 8, 24,  8, 67, health_status);
        convert_s_ch(16, 32, 15, 67, x);
        convert_s_ch(16, 47, 15, 67, y);
        convert_s_ch(16, 62,  9, 67, z);
        convert_s_ch( 8, 71,  3, 67, x_dot);
        convert_s_ch( 8, 74,  3, 67, y_dot);
        convert_s_ch( 8, 77,  4, 67, z_dot);
        convert_u(16, 215, 11, t0);
      };
#undef convert_s_ch
#undef convert_u_ch
#undef convert_s
#undef convert_u
    };

    class IonosphericGridPoints {

      public:
        struct position_index_t;

        struct position_t {
          int_t latitude_deg;  ///< latitude in degrees, north is positive. [-85, 85].
          int_t longitude_deg; ///< longitude in degrees, east is positive. [-180, 175]
          operator position_index_t() const;
          bool is_predefined() const {

            // range check
            if((latitude_deg < -85) || (latitude_deg > 85)){return false;}
            if((longitude_deg < -180) || (longitude_deg >= 180)){return false;}

            // at least, on 5 deg grid
            if((latitude_deg + 85) % 5 != 0){return false;}
            int_t lng_reg(longitude_deg + 180);
            if(lng_reg % 5 != 0){return false;}

            switch(latitude_deg){
              case 80:
              case -80:
                return false;
              case 85:
                return (lng_reg % 30 == 0); // W180(=0), W150(=30), ...
              case -85:
                return (lng_reg % 30 == 10); // W170(=10), W140(=40), ...
            }

            if((latitude_deg >= 65) || (latitude_deg <= -65)){
              return (lng_reg % 10 == 0);
            }

            return true;
          }
          /**
           * Compute longitude difference from another IGP.
           * @param from
           * @return difference (always positive) [0, 360)
           */
          int_t delta_lng(const position_t &from) const {
            int_t res(longitude_deg - from.longitude_deg);
            return (res < 0) ? (res + 360) : res;
          }
        };
        /**
         * Resolve ionospheric grid point position
         * @param band valid range is [0, 10]
         * @param mask_pos valid range is [0, 200 (or 199, 191)] (be careful, not [1, 201])
         * @return (pos_t) grid point position
         * @see Table A-14
         */
        static position_t position(const uint_t &band, const uint_t &mask_pos){
          position_t res;
          if(band <= 8){
            uint_t row_index_28grids((band / 2) * 2); // where 28 grids at the same longitude are appeared
            int row_index(0), col_index((int)mask_pos);
            do{
              int grids(row_index_28grids == row_index ? 28 : 27);
              if(col_index < grids){
                col_index -= 2; // col_index => [-2, 24 (or 25)]
                if((grids > 27) && (band % 2 == 1)){ // col_index => [-3, 24]
                  col_index--;
                }
                break;
              }
              col_index -= grids;
              row_index++;

              grids = 23;
              if(col_index < grids){break;}
              col_index -= grids;
              row_index++;
            }while(row_index < 8);

            if(row_index < 8){
              res.longitude_deg = -180 + band * 40 + row_index * 5;
              switch(col_index){
                case -3: res.latitude_deg = -85; break;
                case -2: res.latitude_deg = -75; break;
                case -1: res.latitude_deg = -65; break;
                case 23: res.latitude_deg =  65; break;
                case 24: res.latitude_deg =  75; break;
                case 25: res.latitude_deg =  85; break;
                default:
                  res.latitude_deg = -55 + col_index * 5;
                  break;
              }
            }
          }else if(band <= 10){
            if(mask_pos < 72){
              res.latitude_deg = 60 * ((band == 10) ? -1 : 1);
              res.longitude_deg = mask_pos * 5 - 180;
            }else if(mask_pos < 180){
              std::div_t a(std::div(mask_pos - 72, 36));
              res.latitude_deg = (65 + a.quot * 5) * ((band == 10) ? -1 : 1);
              res.longitude_deg = a.rem * 10 - 180;
            }else if(mask_pos < 192){
              res.latitude_deg = 85;
              res.longitude_deg = (mask_pos - 180) * 30 - 180;
              if(band == 10){
                res.latitude_deg *= -1;
                res.longitude_deg += 10;
                if(res.longitude_deg > 180){
                  res.longitude_deg -= 360;
                }
              }
            }
          }
          return res;
        }

        struct position_index_t {
          int_t lat_index; ///< Latitude index; N85(0), N75(1), N70(2), N65(3), N60(4), ..., 0(16) ..., S60(28), S65(29), S70(30), S75(31), S85(32)
          int_t lng_index; ///< Longitude index; W180(0), W175(1), W170(2), ..., 0(36), ..., E170(70), E175(71)
          enum {
            LAT_INDEX_N85 = 0,  // 30 deg grid
            LAT_INDEX_N75 = 1,  // 10 deg grid
            LAT_INDEX_N65 = 3,  // 10 deg grid
            LAT_INDEX_S65 = 29, // 10 deg grid
            LAT_INDEX_S75 = 31, // 10 deg grid
            LAT_INDEX_S85 = 32, // 30 deg grid
            LAT_INDEX_MAX = LAT_INDEX_S85,
          };
          enum {
            LNG_INDEX_MAX = 71,
          };
          ///< Convert latitude in degrees to index
          static int_t lat2idx(const int_t &lat_deg){
            return (lat_deg == 85)
                ? LAT_INDEX_N85
                : ((lat_deg == -85)
                      ? LAT_INDEX_S85
                      : (80 - lat_deg) / 5);
          }
          ///< Convert latitude index to degrees
          static int_t idx2lat(const int_t &lat_idx){
            return (lat_idx % 32 == 0)
                ? ((lat_idx == 0) ? 85 : -85)
                : ((16 - lat_idx) * 5);
          }
          ///< Convert longitude in degrees to index
          static int_t lng2idx(const int_t &lng_deg){
            return (lng_deg + 180) / 5;
          }
          ///< Convert longitude index to degrees
          static int_t idx2lng(const int_t &lng_idx){
            return (lng_idx - 36) * 5;
          }
          operator position_t() const {
            position_t res = {idx2lat(lat_index), idx2lng(lng_index)};
            return res;
          }
        };

        struct pivot_t {
          position_t igp;
          struct {
            float_t latitude_deg, longitude_deg;
          } delta;
        };
        /**
         * Get a pivot IGP, and compute distance (delta) from the IGP.
         * The "pivot" one means "nearest west, and north if in south hemisphere,
         * south if otherwise (in north hemisphere, or on equator), one".
         * If the specified input latitude is not identical to zero, and exactly same as a latitude where IGPs exist,
         * then the return IGP is shifted to the nearest west (north hemisphere) or north (east hemisphere) point.
         * For example:
         * 1) (lat, lng) = (10, 0) => igp = {5, 0}, delta = {5, 0}
         * 2) (lat, lng) = (85, 15) => igp = {75, 10}, delta = {10, 5}
         * @param latitude_deg latitude in degrees; [-90, 90]
         * @param longitude_deg longitude in degrees
         * @return pivot IGP position and delta
         */
        template <class T>
        static pivot_t get_pivot(const T &latitude_deg, const T &longitude_deg){

          T lng(longitude_deg); // => [-180, 180)
          int_t lng_reg; // => [0, 360), mapping W180(=> 0), ... E180(=> 360)
          {
            if(longitude_deg < -180){
              lng += (((int_t)(-longitude_deg + 180) / 360) * 360); // => [-*, -180) => (-180, 180]
              if(lng == 180){
                lng -= 360; // 180 => -180
              }
            }else{
              lng -= (((int_t)(longitude_deg + 180) / 360) * 360); // => [-180, +*] => [-180, 180)
            }
            lng_reg = 180 + lng; // => [0, 360)
          }

          pivot_t res;
          if(latitude_deg > 85){
            res.igp.latitude_deg = 85;
            lng_reg = (lng_reg / 90) * 90; // W180, W90, ... @see A 4.4.10.2 d), and P-2
            /*if(latitude_deg == 85){
              // intentionally commented out; when exactly same as lat == 85,
              // a lat-10 deg grid will be used, because of P-2.
              lng_reg = (lng_reg / 30) * 30; // W180, W150, ...
            }*/
          }else if(latitude_deg < -85){
            res.igp.latitude_deg = -85;
            lng_reg = (lng_reg < 40) ? (130 + 180) : (((lng_reg - 40) / 90) * 90 + 40); // W140, W50, ...  @see A 4.4.10.2 e), and P-2
            /*if(latitude_deg == -85){
              // intentionally commented out
              lng_reg = (lng_reg < 10) ? (160 + 180) : (((lng_reg - 10) / 30) * 30 + 10); // W170, W140, ...
            }*/
          }else{
            if(latitude_deg > 75){
              res.igp.latitude_deg = 75;
            }else if(latitude_deg < -75){
              res.igp.latitude_deg = -75;
            }else{
              if(latitude_deg >= 0){
                res.igp.latitude_deg = ((int_t)latitude_deg / 5) * 5;
              }else{
                res.igp.latitude_deg = -((int_t)(-latitude_deg) / 5) * 5;
              }
              if((res.igp.latitude_deg == latitude_deg) && (res.igp.latitude_deg != 0)){ // on grid point latitude, see P-2
                res.igp.latitude_deg += (res.igp.latitude_deg > 0 ? -5 : 5);
              }
            }

            if((res.igp.latitude_deg >= 60) || (res.igp.latitude_deg <= -60)){
              lng_reg = (lng_reg / 10) * 10; // W180, W170, ...
            }else{
              lng_reg = (lng_reg / 5) * 5; // W180, W175, ...
            }
          }
          res.igp.longitude_deg = lng_reg - 180; // [0, 360) => [-180, 180)
          res.delta.latitude_deg = latitude_deg - res.igp.latitude_deg;
          res.delta.longitude_deg = lng - res.igp.longitude_deg;
          if(res.delta.longitude_deg < 0){res.delta.longitude_deg += 360;} // always east (delta.lng >= 0)
          return res;
        }

        struct trapezoid_t {
          /**
           * igp(2)=[1] -- igp(1)=[0]           igp(3)=[2] -- igp(4)=[3]
           *      |             |     in north,      |         |         in south
           * igp(3)=[2] -- igp(4)=[3]           igp(2)=[1] -- igp(1)=[0]
           * This is based on Fig.A-19.
           * result of get_pivot() will be assigned to igp(3)=[2].
           *
           * assumption
           * igp[0].lat = igp[1].lat, igp[2].lat = igp[3].lat
           * igp[1].lng < igp[0].lng, igp[2].lng < igp[3].lng, (igp[1].lng is not necessarily indentical to igp[2].lng)
           */
          position_t igp[4];
          bool checked[4]; ///< If igp[i] has been check to be available, true; otherwise, false.
          float_t weight[4];

          /**
           * @param delta_phi
           * @param delta_lambda
           * @see Fig.A-19
           */
          void compute_weight(const float_t &delta_phi, const float_t &delta_lambda){
            // (A-25)-(A-32)
            float_t
                w_lat10(delta_phi / (igp[1].latitude_deg - igp[2].latitude_deg)), // a.k.a. y_pp
                w_lat23(1. - w_lat10),
                w_lng0((delta_lambda + igp[2].delta_lng(igp[1])) / igp[0].delta_lng(igp[1])),
                w_lng3(delta_lambda / igp[3].delta_lng(igp[2])); // a.k.a. x_pp
            // res = (w_lng0 * [0] + (1. - w_lng0) * [1]) * w_lat10 + ((1. - w_lng3) * [2] + w_lng3 * [3]) * w_lat23;
            weight[0] = w_lng0        * w_lat10;
            weight[1] = (1. - w_lng0) * w_lat10;
            weight[2] = (1. - w_lng3) * w_lat23;
            weight[3] = w_lng3        * w_lat23;
          }
          void compute_weight_pole(const float_t &delta_phi, const float_t &delta_lambda){
            float_t
                y_pp(delta_phi * ((delta_phi < 0) ? -1 : 1) / 10), // (A-33)
                x_pp((1. - y_pp * 2) * (delta_lambda / 90) + y_pp), // (A-34)
                x_pp_inv(1. - x_pp),
                y_pp_inv(1. - y_pp);
            weight[0] = x_pp     * y_pp;
            weight[1] = x_pp_inv * y_pp;
            weight[2] = x_pp_inv * y_pp_inv;
            weight[3] = x_pp     * y_pp_inv;
          }
          /**
           * @return If extrapolation occurs, false is returned; otherwise true.
           */
          bool compute_weight_three(const float_t &delta_phi, const float_t &delta_lambda,
              const int_t &skip){
            float_t
                y_pp(delta_phi / (igp[1].latitude_deg - igp[2].latitude_deg)),
                x_pp(delta_lambda / igp[3].delta_lng(igp[2]));
            switch(skip){ // assignment rule: sum is 1, weight of non-diagonal point is remain?
              case 0:
                weight[0] = 0;
                weight[1] = y_pp;
                weight[2] = 1. - x_pp - y_pp;
                weight[3] = x_pp;
                if(weight[2] < 0){return false;}
                break;
              case 1:
                weight[0] = y_pp;
                weight[1] = 0;
                weight[2] = 1. - x_pp;
                weight[3] = x_pp - y_pp;
                if(weight[3] < 0){return false;}
                break;
              case 2:
                weight[0] = x_pp + y_pp - 1;
                weight[1] = 1. - x_pp;
                weight[2] = 0;
                weight[3] = 1. - y_pp;
                if(weight[0] < 0){return false;}
                break;
              case 3:
              default:
                weight[0] = x_pp;
                weight[1] = y_pp - x_pp;
                weight[2] = 1. - y_pp;
                weight[3] = 0;
                if(weight[1] < 0){return false;}
                break;
            }
            return true;
          }
          /**
           * @return If three point interpolation is successfully prepared, true; otherwise false.
           */
          bool compute_weight_three(const float_t &delta_phi, const float_t &delta_lambda){
            for(int i(0); i <= 3; i++){
              if(!checked[i]){ // automatically find unavailable point
                return compute_weight_three(delta_phi, delta_lambda, i);
              }
            }
            return false;
          }

          static trapezoid_t generate_rectangle(const position_t &pivot,
              const int_t &delta_lat = 5, const int_t &delta_lng = 5){
            int_t lng(pivot.longitude_deg + delta_lng);
            if(lng >= 180){lng -= 360;}
            trapezoid_t res = {{
              {pivot.latitude_deg + delta_lat, lng},
              {pivot.latitude_deg + delta_lat, pivot.longitude_deg},
              pivot,
              {pivot.latitude_deg,             lng},
            }, {
              false, false, false, false,
            }};
            return res;
          }
          static trapezoid_t generate_rectangle_pole(const position_t &pivot){
            int_t
                lng0(pivot.longitude_deg - 180),
                lng1(pivot.longitude_deg - 90),
                lng3(pivot.longitude_deg + 90);
            trapezoid_t res = {{
              {pivot.latitude_deg, (lng0 < -180) ? (lng0 + 360) : lng0},
              {pivot.latitude_deg, (lng1 < -180) ? (lng1 + 360) : lng1},
              pivot,
              {pivot.latitude_deg, (lng3 >= 180) ? (lng3 - 360) : lng3},
            }, {
              false, false, false, false,
            }};
            return res;
          }
          /**
           * Get expanded rectangle
           *
           * @param delta_lat When positive, move a parallel between igp[1] and igp[0] toward the nearer pole;
           * when negative, move a parallel between igp[2] and igp[3] toward the other pole.
           * Be careful, when negative, the difference of IPP from pivot, i.e. igp[2], should be recalculated.
           *
           * @param delta_lng When positive, move a meridian between igp[0] and igp[3] east;
           * when negative, move a meridian between igp[1] and igp[2] west.
           * Be careful, when negative, the difference of IPP from pivot, i.e. igp[2], should be recalculated.
           *
           * @return Expanded rectangle
           */
          trapezoid_t expand_rectangle(const int_t &delta_lat, const int_t &delta_lng) const {
            trapezoid_t res(*this);
            if(delta_lat != 0){
              int_t delta_lat2(delta_lat * ((res.igp[1].latitude_deg >= 0) ? 1 : -1)); // check hemisphere
              if(delta_lat > 0){
                res.igp[1].latitude_deg = (res.igp[0].latitude_deg += delta_lat2);
                res.checked[1] = res.checked[0] = false;
              }else{
                res.igp[2].latitude_deg = (res.igp[3].latitude_deg += delta_lat2);
                res.checked[2] = res.checked[3] = false;
              }
            }
            if(delta_lng > 0){
              res.igp[0].longitude_deg += delta_lng;
              if(res.igp[0].longitude_deg >= 180){res.igp[0].longitude_deg -= 360;}
              res.igp[3].longitude_deg = res.igp[0].longitude_deg;
              res.checked[3] = res.checked[0] = false;
            }else if(delta_lng < 0){
              res.igp[1].longitude_deg += delta_lng;
              if(res.igp[1].longitude_deg < -180){res.igp[1].longitude_deg += 360;}
              res.igp[2].longitude_deg = res.igp[1].longitude_deg;
              res.checked[2] = res.checked[1] = false;
            }
            return res;
          }
        };

        template <class T>
        T check_avialability_hook(trapezoid_t &in, const T &out) const {
          return out; // for debug
        }

        /**
         * @return available IGP(s)
         */
        int_t check_avialability(trapezoid_t &target) const {
          int_t res(0);
          for(int i(0); i < 4; ++i){
            if(target.checked[i]){res++; continue;}
            // TODO
          }
          return check_avialability_hook(target, res);
        }

        void interpolate(const float_t &latitude_deg, const float_t &longitude_deg) const { // TODO return type
          pivot_t pivot(get_pivot(latitude_deg, longitude_deg));

          bool north_hemisphere(latitude_deg >= 0);
          int_t lat_deg_abs(pivot.igp.latitude_deg * (north_hemisphere ? 1 : -1));

          if(lat_deg_abs <= 55){
            trapezoid_t rect_5_5(trapezoid_t::generate_rectangle(pivot.igp, north_hemisphere ? 5 : -5, 5)); // A4.4.10.2 a-1)
            switch(check_avialability(rect_5_5)){
              case 4: // a-1)
                rect_5_5.compute_weight(pivot.delta.latitude_deg, pivot.delta.longitude_deg);
                return;
              case 3: // a-2)
                if(rect_5_5.compute_weight_three(pivot.delta.latitude_deg, pivot.delta.longitude_deg)){
                  return;
                }
            }

            struct {
              trapezoid_t rect;
              float_t delta_lat, delta_lng;
              int_t availability;
            } rect_10_10[] = { // A4.4.10.2 a-3), 5x5 => 10x10
              {rect_5_5.expand_rectangle( 5,  5), pivot.delta.latitude_deg, pivot.delta.longitude_deg},
              {rect_5_5.expand_rectangle( 5, -5), pivot.delta.latitude_deg, pivot.delta.longitude_deg + 5},
              {rect_5_5.expand_rectangle(-5,  5),
                  pivot.delta.latitude_deg + (north_hemisphere ? 5 : -5), pivot.delta.longitude_deg},
              {rect_5_5.expand_rectangle(-5, -5),
                  pivot.delta.latitude_deg + (north_hemisphere ? 5 : -5), pivot.delta.longitude_deg + 5},
            };
            for(int i(0); i < 4; ++i){ // a-3) four point interpolation

              if((lat_deg_abs == 55)
                  && (rect_10_10[i].rect.igp[1].latitude_deg * (north_hemisphere ? 1 : -1) == 65)
                  && ((rect_10_10[i].rect.igp[1].longitude_deg + 180) % 10 != 0)){
                // When pivot lat = 55, -55, one 10x10 trapezoids are unable to be formed,
                // because of lack of grid points at lat = 65, -65.
                rect_10_10[i].availability = 0;
                continue;
              }

              if((rect_10_10[i].availability = check_avialability(rect_10_10[i].rect)) == 4){
                rect_10_10[i].rect.compute_weight(rect_10_10[i].delta_lat, rect_10_10[i].delta_lng);
                return;
              }
            }
            for(int i(0); i < 4; ++i){ // a-4) three point interpolation
              if((rect_10_10[i].availability == 3)
                  && rect_10_10[i].rect.compute_weight_three(rect_10_10[i].delta_lat, rect_10_10[i].delta_lng)){
                return;
              }
            }
          }else if(lat_deg_abs <= 70){
            trapezoid_t rect_5_10(trapezoid_t::generate_rectangle(pivot.igp, north_hemisphere ? 5 : -5, 10)); // A4.4.10.2 b-1)
            switch(check_avialability(rect_5_10)){
              case 4: // b-1)
                rect_5_10.compute_weight(pivot.delta.latitude_deg, pivot.delta.longitude_deg);
                return;
              case 3: // b-2)
                if(rect_5_10.compute_weight_three(pivot.delta.latitude_deg, pivot.delta.longitude_deg)){
                  return;
                }
            }

            struct {
              trapezoid_t rect;
              float_t delta_lat, delta_lng;
              int_t availability;
            } rect_10_10[] = { // A4.4.10.2 b-3) , 5x10 => 10x10
              {rect_5_10.expand_rectangle( 5, 0),
                  pivot.delta.latitude_deg, pivot.delta.longitude_deg},
              {rect_5_10.expand_rectangle(-5, 0),
                  pivot.delta.latitude_deg + (north_hemisphere ? 5 : -5), pivot.delta.longitude_deg},
            };
            for(int i(0); i < 2; ++i){ // b-3) four point interpolation

              if((lat_deg_abs == 70)
                  && (rect_10_10[i].rect.igp[1].latitude_deg * (north_hemisphere ? 1 : -1) == 80)){
                // When pivot lat = 70, -70, one 10x10 trapezoids are unable to be formed,
                // because of no grid point at lat = 80, -80.
                rect_10_10[i].availability = 0;
                continue;
              }

              if((rect_10_10[i].availability = check_avialability(rect_10_10[i].rect)) == 4){
                rect_10_10[i].rect.compute_weight(rect_10_10[i].delta_lat, rect_10_10[i].delta_lng);
                return;
              }
            }
            for(int i(0); i < 2; ++i){ // b-4) three point interpolation
              if((rect_10_10[i].availability == 3)
                  && rect_10_10[i].rect.compute_weight_three(rect_10_10[i].delta_lat, rect_10_10[i].delta_lng)){
                return;
              }
            }
          }else if(lat_deg_abs <= 75){
            trapezoid_t rect_10_10(trapezoid_t::generate_rectangle(pivot.igp, north_hemisphere ? 10 : -10, 10));

            // maximum 4 kinds of trial
            // 1)   10x30, both 85 points are band 9-10 (30 deg separation)
            // 2,3) 10X30, one 85 point is in band 0-8, the other is in band 9-10
            // 4)   10x90, both 85 points are band 0-8 (90 deg separation)

            int_t lng_85_west_low_band, lng_85_west_high_band;
            int_t lng_85_east_low_band, lng_85_east_high_band;
            {
              int_t lng_reg(pivot.igp.longitude_deg + 180); // [-180, 180) => [0, 360)
              if(north_hemisphere){
                lng_85_west_low_band = (lng_reg / 30 * 30) - 180; // W180, W150, ...
                lng_85_west_high_band = (lng_reg / 90 * 90) - 180; // W180, W90, ...
              }else{
                lng_85_west_low_band = (lng_reg < 10) ? 160 : (((lng_reg - 10) / 30) * 30 - 170); // W170, W140, ..., E160
                lng_85_west_high_band = (lng_reg < 40) ? 130 : (((lng_reg - 40) / 90) * 90 - 140); // W140, W50, ..., E130
              }
              lng_85_east_low_band = lng_85_west_low_band + 30;
              lng_85_east_high_band = lng_85_west_high_band + 90;
              if(lng_85_east_low_band >= 180){lng_85_east_low_band -= 360;}
              if(lng_85_east_high_band >= 180){lng_85_east_high_band -= 360;}
            }

            { // check 1)
              rect_10_10.igp[1].longitude_deg = lng_85_west_low_band;
              rect_10_10.igp[0].longitude_deg = lng_85_east_low_band;
              if(check_avialability(rect_10_10) == 4){
                rect_10_10.compute_weight(pivot.delta.latitude_deg, pivot.delta.longitude_deg);
                return;
              }
            }

            if(rect_10_10.checked[2] && rect_10_10.checked[3]){
              // Requirement: lower latitude point information is broadcasted.

              bool check_again(true);

              do{
                if(lng_85_west_low_band == lng_85_west_high_band){
                  // |[1]<--(30)-->[0]|----(90)---->|
                  if(rect_10_10.checked[1]){
                    // prepare for the last trial
                    rect_10_10.igp[0].longitude_deg = lng_85_east_high_band;
                    //rect_10_10.checked[0] = false; // already set
                  }else{
                    check_again = false;
                  }
                  break;
                }
                if(lng_85_east_low_band == lng_85_east_high_band){
                  // |<----(90)----|[1]<--(30)-->[0]|
                  if(rect_10_10.checked[0]){
                    // prepare for the last trial
                    rect_10_10.igp[1].longitude_deg = lng_85_west_high_band;
                    //rect_10_10.checked[1] = false; // already set
                  }else{
                    check_again = false;
                  }
                  break;
                }

                // just middle case: |<--(90)--|[1]<--(30)-->[0]|---->|

                if(!rect_10_10.checked[0]){ // check 2) |<--(90)--|[1]<--(30)-->[0]|---->[0]'|
                  rect_10_10.igp[0].longitude_deg = lng_85_east_high_band;
                  //rect_10_10.checked[0] = false; // already set, 1st trial
                }

                if(!rect_10_10.checked[1]){ // check 3) |[1]'<--(90)--|[1]<--(30)-->[0]|---->|
                  rect_10_10.igp[1].longitude_deg = lng_85_west_high_band;
                  //rect_10_10.checked[1] = false; // already set, 1st trial
                }
              }while(false);

              // check 2-4)
              if(check_again && (check_avialability(rect_10_10) == 4)){
                rect_10_10.compute_weight(pivot.delta.latitude_deg, pivot.delta.longitude_deg);
                return;
              }
            }
          }else{ // pole
            trapezoid_t rect(trapezoid_t::generate_rectangle_pole(pivot.igp));
            if(check_avialability(rect) == 4){
              rect.compute_weight_pole(pivot.delta.latitude_deg, pivot.delta.longitude_deg);
              return;
            }
          }

          // Correction unavailable
        }
    };

    class Satellite {
      public:
        typedef typename gps_space_node_t::constellation_t constellation_t;

        /**
         * SBAS Ephemeris
         * @see Table A-18
         */
        struct Ephemeris {
          uint_t svid;            ///< Satellite number
          uint_t WN;              ///< Week number

          float_t t_0;            ///< Time of applicability (s) <= time of a week
          int_t URA;              ///< User range accuracy (index)
          float_t x, y, z;        ///< ECEF position (m)
          float_t dx, dy, dz;     ///< ECEF velocity (m/s)
          float_t ddx, ddy, ddz;  ///< ECEF acceleration (m/s^2)
          float_t a_Gf0;           ///< Clock correction parameter (s)
          float_t a_Gf1;           ///< Clock correction parameter (s/s)

          /**
           * Adjust time of ephemeris with time of current
           */
          void adjust_time(const gps_time_t &t_current){
            WN = t_current.WN;
            float_t sec_of_a_day(std::fmod(t_current.seconds, gps_time_t::seconds_day)), t_0_orig(t_0);
            t_0 += (t_current.seconds - sec_of_a_day);

            // roll over check
            if(sec_of_a_day - t_0_orig > gps_time_t::seconds_day / 4 * 3){
              t_0 += gps_time_t::seconds_day;
              if(t_0 >= gps_time_t::seconds_week){
                WN++;
                t_0 -= gps_time_t::seconds_week;
              }
            }else if(sec_of_a_day - t_0_orig < -gps_time_t::seconds_day / 4 * 3){
              t_0 -= gps_time_t::seconds_day;
              if(t_0 < 0){
                WN--;
                t_0 += gps_time_t::seconds_week;
              }
            }
          }

          constellation_t constellation(
              const gps_time_t &t_rx, const float_t &pseudo_range = 0,
              const bool &with_velocity = true) const {

            float_t t_G(-t_rx.interval(WN, 0) - (pseudo_range / gps_space_node_t::light_speed));

            float_t t(t_G - (a_Gf0 + a_Gf1 * (t_G - t_0))); // Eq.(A-45)

            float_t delta_t(t - t_0), delta_t2(delta_t * delta_t / 2);

            constellation_t res = {
              xyz_t(
                  x + dx * delta_t + ddx * delta_t2,
                  y + dy * delta_t + ddy * delta_t2,
                  z + dz * delta_t + ddz * delta_t2), // Eq. (A-44)
              xyz_t(
                  dx + ddx * delta_t,
                  dy + ddy * delta_t,
                  dz + ddz * delta_t),
            };

            // TODO Sagnac correction

            return res;
          }

          struct raw_t {
            u8_t  svid;           ///< Satellite number

            u16_t t_0;            ///< Time of applicability (16, s) [0,86384] <= time of a day
            u8_t  URA;            ///< User range accuracy
            s32_t x, y, z;        ///< ECEF position (0.08(xy), 0.4(z) m)
            s32_t dx, dy, dz;     ///< ECEF velocity (0.000625(xy), 0.004(z) m)
            s16_t ddx, ddy, ddz;  ///< ECEF acceleration (0.0000125(xy), 0.0000625(z) m/s^2)
            s16_t a_Gf0;          ///< Clock correction parameter (2^-31, s)
            s8_t a_Gf1;           ///< Clock correction parameter (2^-40, s/s)

            static raw_t fetch(const char *buf){
              typedef typename DataBlock::Type9 msg_t;
              raw_t res = {
                0, // svid
                msg_t::t0(buf), // t_0
                msg_t::ura(buf), // URA
                msg_t::x(buf), msg_t::y(buf), msg_t::z(buf), // x, y, z
                msg_t::dx(buf), msg_t::dy(buf), msg_t::dz(buf), // dx, dy, dz
                msg_t::ddx(buf), msg_t::ddy(buf), msg_t::ddz(buf), // ddx, ddy, ddz
                msg_t::a_Gf0(buf), msg_t::a_Gf1(buf), // a_Gf0, a_Gf1
              };
              return res;
            }

            enum {
              SF_t_0,
              SF_xy,    SF_z,
              SF_dxy,   SF_dz,
              SF_ddxy,  SF_ddz,
              SF_a_Gf0,
              SF_a_Gf1,

              SF_NUM,
            };
            static const float_t sf[SF_NUM];

            operator Ephemeris() const {
              Ephemeris converted;
#define CONVERT2(TARGET, TARGET_SF) \
{converted.TARGET = sf[SF_ ## TARGET_SF] * TARGET;}
#define CONVERT(TARGET) CONVERT2(TARGET, TARGET)
              converted.svid = svid;
              converted.WN = 0; // Week number (must be configured later) @see adjust_time

              converted.URA = URA;
              CONVERT(t_0);     // Time of a day => time of a week (must be configured later) @see adjust_time
              CONVERT2(x, xy);      CONVERT2(y, xy);      CONVERT(z);
              CONVERT2(dx, dxy);    CONVERT2(dy, dxy);    CONVERT(dz);
              CONVERT2(ddx, ddxy);  CONVERT2(ddy, ddxy);  CONVERT(ddz);
              CONVERT(a_Gf0);
              CONVERT(a_Gf1);
#undef CONVERT
#undef CONVERT2
              return converted;
            }

            raw_t &operator=(const Ephemeris &eph){
#define CONVERT3(TARGET_DST, TARGET_SRC, TARGET_SF) \
{TARGET_DST = (s32_t)((TARGET_SRC + 0.5 * sf[SF_ ## TARGET_SF]) / sf[SF_ ## TARGET_SF]);}
#define CONVERT2(TARGET, TARGET_SF) CONVERT3(eph.TARGET, eph.TARGET, TARGET_SF)
#define CONVERT(TARGET) CONVERT2(TARGET, TARGET)
              svid = eph.svid;

              URA = eph.URA;
              CONVERT3(t_0, std::fmod(t_0, gps_time_t::seconds_day), t_0);
              CONVERT2(x, xy);      CONVERT2(y, xy);      CONVERT(z);
              CONVERT2(dx, dxy);    CONVERT2(dy, dxy);    CONVERT(dz);
              CONVERT2(ddx, ddxy);  CONVERT2(ddy, ddxy);  CONVERT(ddz);
              CONVERT(a_Gf0);
              CONVERT(a_Gf1);
#undef CONVERT
#undef CONVERT2
#undef CONVERT3
              return *this;
            }
          };

          bool is_equivalent(const Ephemeris &eph) const {
            do{
              if(WN != eph.WN){break;}
              if(URA != eph.URA){break;}

#define CHECK2(TARGET, TARGET_SF) \
if(std::abs(TARGET - eph.TARGET) > raw_t::sf[raw_t::SF_ ## TARGET_SF]){break;}
#define CHECK(TARGET) CHECK2(TARGET, TARGET)
              CHECK(t_0);
              CHECK2(x, xy);      CHECK2(y, xy);      CHECK(z);
              CHECK2(dx, dxy);    CHECK2(dy, dxy);    CHECK(dz);
              CHECK2(ddx, ddxy);  CHECK2(ddy, ddxy);  CHECK(ddz);
              CHECK(a_Gf0);
              CHECK(a_Gf1);
#undef CHECK
#undef CHECK2
              return true;
            }while(false);
            return false;
          }
        };

        /**
         * SBAS almanac
         * @see Table A-19
         */
        struct Almanac {
          uint_t data_id;
          uint_t prn;         ///< PRN number
          uint_t SV_health;   ///< Health status
          float_t x, y, z;    ///< ECEF position (m)
          float_t dx, dy, dz; ///< ECEF velocity (m/s)
          float_t t_0;        ///< Time of applicability (s)

          ///< Up-cast to ephemeris
          operator Ephemeris() const {
            Ephemeris converted = {
              prn,        // Satellite number
              t_0,        // Time of applicability (s)
              -1,         // User range accuracy (index)
              x, y, z,    // ECEF position (m)
              dx, dy, dz, // ECEF velocity (m/s)
              0, 0, 0,    // ECEF acceleration (m/s^2)
              0,          // Clock correction parameter (s)
              0,          // Clock correction parameter (s/s)
            };
            return converted;
          }

          struct raw_t {
            u8_t data_id;
            u8_t prn;         ///< PRN number
            u8_t SV_health;   ///< Health status
            s16_t x, y, z;    ///< ECEF position (2600(xy), 26000(z), m)
            s8_t dx, dy, dz;  ///< ECEF velocity (10(xy), 60(z), m/s)
            u16_t t_0;        ///< Time of applicability (64, s)

            static raw_t fetch(const char *buf, const uint_t &ch){
              typedef typename DataBlock::Type17 msg_t;
              raw_t res = {
                msg_t::id(buf, ch), // data_id
                msg_t::prn(buf, ch), // prn
                msg_t::health_status(buf, ch), // SV_health
                msg_t::x(buf, ch), msg_t::y(buf, ch), msg_t::z(buf, ch), // x, y, z
                msg_t::dx(buf, ch), msg_t::dy(buf, ch), msg_t::dz(buf, ch), // dx, dy, dz
                msg_t::t0(buf), // t_0
              };
              return res;
            }

            enum {
              SF_xy,  SF_z,
              SF_dxy, SF_dz,
              SF_t_0,

              SF_NUM,
            };
            static const float_t sf[SF_NUM];

            operator Almanac() const {
              Almanac converted;
#define CONVERT2(TARGET, TARGET_SF) \
{converted.TARGET = sf[SF_ ## TARGET_SF] * TARGET;}
#define CONVERT(TARGET) CONVERT2(TARGET, TARGET)
                converted.data_id = data_id;
                converted.prn = prn;
                converted.SV_health = SV_health;
                CONVERT2(x, xy);    CONVERT2(y, xy);    CONVERT(z);
                CONVERT2(dx, dxy);  CONVERT2(dy, dxy);  CONVERT(dz);
                CONVERT(t_0);
#undef CONVERT
#undef CONVERT2
              return converted;
            }
          };
        };
    };
};

template <class FloatT>
const typename SBAS_SpaceNode<FloatT>::RangingCode SBAS_SpaceNode<FloatT>::ranging_codes[] = {
  {120,  145, 01106, "INMARSAT 3F2 AOR-E"},
  {121,  175, 01241, "INMARSAT 4F2"},
  {122,   52, 00267, "INMARSAT 3F4 AOR-W"},
  {123,   21, 00232, "LM RPS-1, RPS-2"},
  {124,  237, 01617, "Artemis"},
  {125,  235, 01076, "LM RPS-1, RPS-2"},
  {126,  886, 01764, "INMARSAT 3F5 IND-W"},
  {127,  657, 00717, "INSATNAV"},
  {128,  634, 01532, "INSATNAV"},
  {129,  762, 01250, "MTSAT-1R (or MTSAT-2)"},
  {130,  355, 00341, "INMARSAT 4F1"},
  {131, 1012, 00551, "INMARSAT 3F1 IOR"},
  {132,  176, 00520, "Unallocated"},
  {133,  603, 01731, "INMARSAT 4F3"},
  {134,  130, 00706, "INMARSAT 3F3 POR"},
  {135,  359, 01216, "LM RPS-1"},
  {136,  595, 00740, "INMARSAT Reserved"},
  {137,   68, 01007, "MTSAT-2 (or MTSAT-1R)"},
  {138,  386, 00450, "LM RPS-2"},
}; ///< @see Table A-1

template <class FloatT>
SBAS_SpaceNode<FloatT>::IonosphericGridPoints::position_t::operator typename SBAS_SpaceNode<FloatT>::IonosphericGridPoints::position_index_t() const {
  SBAS_SpaceNode<FloatT>::IonosphericGridPoints::position_index_t res = {
      position_index_t::lat2idx(latitude_deg),
      position_index_t::lng2idx(longitude_deg)};
  return res;
}

#define POWER_2(n) \
(((n) >= 0) \
  ? (float_t)(1 << (n >= 0 ? n : 0)) \
  : (((float_t)1) / (1 << (-(n) >= 30 ? 30 : -(n > 0 ? 0 : n))) \
    / (1 << (-(n) >= 30 ? (-(n) - 30) : 0))))

template <class FloatT>
const typename SBAS_SpaceNode<FloatT>::float_t SBAS_SpaceNode<FloatT>::Satellite::Ephemeris::raw_t::sf[] = {
  16,           // SF_t_0
  0.08,         // SF_xy
  0.4,          // SF_z
  0.000625,     // SF_dxy
  0.004,        // SF_dz
  0.0000125,    // SF_ddxy
  0.0000625,    // SF_ddz
  POWER_2(-31), // SF_a_Gf0
  POWER_2(-40), // SF_a_Gf1
};

template <class FloatT>
const typename SBAS_SpaceNode<FloatT>::float_t SBAS_SpaceNode<FloatT>::Satellite::Almanac::raw_t::sf[] = {
  2600,   // SF_xy
  26000,  // SF_z
  10,     // SF_dxy
  60,     // SF_dz
  64,     // SF_t_0
};

#undef POWER_2

#endif /* __SBAS_H__ */
