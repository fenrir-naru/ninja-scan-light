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

#include <iostream>
#include <cmath>
#include <cstring>
#include <cstddef>
#include <map>
#include <algorithm>

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
    type_copy(enu_t);
    type_copy(llh_t);

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
      float_t lng_deg;
      const char *name;
      typename SBAS_Signal<float_t>::G2 get_G2() const {
        return typename SBAS_Signal<float_t>::G2(initial_g2);
      }
      struct prn_sorter_t {
        bool operator()(const RangingCode *left, const RangingCode *right) const {
          return (left->prn < right->prn);
        }
      };
      struct lng_sorter_t {
        bool operator()(const RangingCode *left, const RangingCode *right) const {
          return (left->lng_deg < right->lng_deg);
        }
      };
      struct lng_sorter2_t {
        float_t base_lng_deg;
        lng_sorter2_t(const float_t &lng_deg) : base_lng_deg(lng_deg) {}
        bool operator()(const RangingCode *left, const RangingCode *right) const {
          float_t delta_l(left->lng_deg - base_lng_deg);
          if(delta_l < 0){delta_l *= -1;}
          if(delta_l >= 180){delta_l = -delta_l + 360;}
          float_t delta_r(right->lng_deg - base_lng_deg);
          if(delta_r < 0){delta_r *= -1;}
          if(delta_r >= 180){delta_r = -delta_r + 360;}
          return (delta_l < delta_r);
        }
      };
      struct name_sorter_t {
        bool operator()(const RangingCode *left, const RangingCode *right) const {
          int cmp(std::strcmp(left->name, right->name));
          return cmp != 0 ? (cmp < 0) : (left->prn < right->prn);
        }
      };
    };

    struct KnownSatellites {
      typedef std::vector<const RangingCode *> list_t;
      template <typename T>
      static list_t sort(T sorter);
      static const list_t prn_ordered;
      static const list_t longitude_ordered;
      static const list_t name_ordered;
      static list_t nearest_ordered(const float_t &lng_deg){
        return sort(typename RangingCode::lng_sorter2_t(lng_deg));
      }
    };

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
      UNSUPPORTED_MESSAGE = 64,
    }; ///< @see Table A-3

    struct Timing {
      enum {
        DONT_USE_FOR_SAFETY_APPLICATIONS,
        PRN_MASK,
        UDREI,
        FAST_CORRECTIONS,
        LONG_TERM_CORRECTIONS,
        GEO_NAVIGATION_DATA,
        FAST_CORRECTION_DEGRADATION,
        DEGRADATION_PARAMETERS,
        IONOSPHERIC_GRID_MASK,
        IONOSPHERIC_CORRECTIONS,
        UTC_TIMING_DATA,
        ALNAMAC_DATA,
        SERVICE_LEVEL,
        CLOCK_EPHEMERIS_COVARIANCE_MATRIX,
        NUM_OF_TIMING_ITEMS,
      };
      struct values_t {
        int interval, timeout_EN_Route_Terminal_LNAV, timeout_LNAV_VNAV_LP_LPV_approach;
      };
      static const values_t values[NUM_OF_TIMING_ITEMS];
    }; ///< @see Table A-25;

    struct DataBlock : public gps_space_node_t::DataParser {
      typedef typename gps_space_node_t::DataParser parser_t;

#define convert_u(bits, offset_bits, length, name) \
template <class InputT> \
static u ## bits ## _t name(const InputT *buf){ \
  return parser_t::template bits2num<u ## bits ## _t, InputT>( \
      buf, offset_bits, length); \
} \
template <class BufferT> \
static void name ## _set(BufferT *dest, const u ## bits ## _t &src){ \
  parser_t::template num2bits<u ## bits ## _t, BufferT>( \
      dest, src, offset_bits, length); \
}
#define convert_s(bits, offset_bits, length, name) \
template <class InputT> \
static s ## bits ## _t name(const InputT *buf){ \
  return (s ## bits ## _t)parser_t::template bits2num<u ## bits ## _t, InputT>( \
        buf, offset_bits) \
      >> (bits - length); \
} \
template <class BufferT> \
static void name ## _set(BufferT *dest, const s ## bits ## _t &src){ \
  parser_t::template num2bits<u ## bits ## _t, BufferT>( \
      dest, *(u ## bits ## _t *)(&src), offset_bits, length); \
}
#define convert_u_ch(bits, offset_bits, length, bits_per_ch, name) \
template <class InputT> \
static u ## bits ## _t name(const InputT *buf, const uint_t &ch){ \
  return parser_t::template bits2num<u ## bits ## _t>( \
      buf, offset_bits + (bits_per_ch * ch), length); \
} \
template <class BufferT> \
static void name ## _set(BufferT *dest, const uint_t &ch, const u ## bits ## _t &src){ \
  parser_t::template num2bits<u ## bits ## _t, BufferT>( \
      dest, src, offset_bits + (bits_per_ch * ch), length); \
}
#define convert_s_ch(bits, offset_bits, length, bits_per_ch, name) \
template <class InputT> \
static s ## bits ## _t name(const InputT *buf, const uint_t &ch){ \
  return (s ## bits ## _t)parser_t::template bits2num<u ## bits ## _t>( \
        buf, offset_bits + (bits_per_ch * ch)) \
      >> (bits - length); \
} \
template <class BufferT> \
static void name ## _set(BufferT *dest, const uint_t &ch, const s ## bits ## _t &src){ \
  parser_t::template num2bits<u ## bits ## _t, BufferT>( \
      dest, *(u ## bits ## _t *)(&src), offset_bits + (bits_per_ch * ch), length); \
}

      /**
       * Convert bit pattern to indices array having '1'
       * Ex) 0b00100100 => [2, 5]
       * @param buf bit pattern
       * @param indices array to which results are stored
       * @param length length of bits to be inspected
       * @param offset starting offset of bits
       * @return number of '1'
       */
      template <class InputT>
      static u8_t bits2linear(
          const InputT *buf, u8_t *indices,
          const u8_t &length, const u8_t &offset = 0){
        u8_t *hits(indices);
        std::div_t aligned(std::div(offset, (int)sizeof(InputT) * 8));
        buf += aligned.quot;
        InputT compared((InputT)0x1 << (sizeof(InputT) * 8 - aligned.rem - 1));
        // [mask7, mask6, .., mask0], [mask15, mask14, .., mask8], ...
        for(u8_t i(0); i < length; ++i, compared >>= 1){
          if(compared == 0){ // rotate
            compared = ((InputT)0x1 << (sizeof(InputT) * 8 - 1));
            buf++;
          }
          if(*buf & compared){
            *(hits++) = i;
          }
        }
        return hits - indices;
      }

      convert_u(8, 0, 8, preamble);
      template <class BufferT>
      static void preamble_set2(BufferT *dest, const unsigned int &index = 0){
        static const u8_t preamble_list[] = {0x53, 0x9A, 0xC6};
        preamble_set(dest, preamble_list[index % 3]);
      }
      convert_u(8, 8, 6, message_type);

      convert_u(32, 226, 24, parity);
      /**
       * update parity bits based on current buffer
       * @param buf buffer
       * @see SBAS MOPS (DO-229) A.4.3.3
       */
      template <class BufferT>
      static void parity_set(BufferT *dst){
        // CRC-24Q
        static const u32_t poly(0x1864CFBu);
        static const struct tbl_t {
          u32_t fw[0x100];
          tbl_t() {
            for(int i(0); i < 0x100; ++i){
              fw[i] = i << 16;
              for(int j(0); j < 8; ++j){
                fw[i] <<= 1;
                if(fw[i] & 0x1000000u){fw[i] ^= poly;}
              }
            }
          }
        } tbl;
        u32_t crc(0);
        u8_t buf;
        int bit_idx(0);
        for(int bytes(226 / 8); bytes > 0; bytes--, bit_idx += 8){
          buf = parser_t::template bits2num<u8_t, BufferT>(dst, bit_idx, 8);
          crc = (((crc << 8) & 0xFFFF00) | buf) ^ tbl.fw[(crc >> 16) & 0xFF];
        }
        buf = parser_t::template bits2num<u8_t, BufferT>(dst, bit_idx, 8);
        for(unsigned char mask(0x80); bit_idx < 226; bit_idx++, mask >>= 1){
          crc <<= 1;
          if(buf & mask){crc |= 1;}
          if(crc & 0x1000000u){crc ^= poly;}
        }
        for(int i(0); i < 3; i++){ // forward operation for padding
          unsigned int cache(tbl.fw[(crc >> 16) & 0xFF]);
          crc = (((crc << 8) ^ cache) & 0xFFFF00) | (cache & 0xFF);
        }
        parity_set(dst, crc);
      }

      struct Type1 { ///< @see Fig. A-6 PRN_MASK
        struct mask_t {
          u8_t valid;
          static const int each_block = 13;
          union {
            u8_t prn_minus_one[210];
            u8_t block[4][each_block];
          };
        };
        template <class InputT>
        static mask_t mask(const InputT *buf, const u8_t &band){
          mask_t res;
          res.valid = bits2linear(buf, res.prn_minus_one, 210, 14); // 14 bit shift
          if(res.valid > 51){res.valid = 0;} // invalid, because up to 51 masks
          return res;
        }
        convert_u(8, 224, 2, iodp);
      };

      struct Type2_5 { ///< @see Fig. A-7 FAST_CORRECTION_2-5
        convert_u( 8, 14, 2, iodf);
        convert_u( 8, 16, 2, iodp);
        convert_s_ch(16,  18, 12, 12, prc_f); // pseudo range correction (fast)
        convert_u_ch( 8, 174,  4,  4, udrei);
      };

      struct Type6 { ///< @see Fig. A-8, Table A-5 INTEGRITY_INFORMATION
        convert_u_ch( 8, 14, 2, 2, iodf); // iodf2-5
        convert_u_ch( 8, 22, 4, 4, udrei);
      };

      struct Type7 { ///< @see Fig. A-9, Table A-7 FAST_CORRECTION_DEGRADATION
        convert_u( 8, 14, 4, t_lat);
        convert_u( 8, 18, 2, iodp);
        convert_u_ch( 8, 22, 4, 4, ai_i);
      };

      struct Type25 { ///< @see Fig. A-10,11, Table A-10,11 LONG_TERM_CORRECTION
        convert_u_ch( 8, 14, 1, 106, vel_code);
        convert_u_ch( 8, 15, 6, 106, prn_mask);
        convert_u_ch( 8, 21, 8, 106, iod);
        struct V0 {
          convert_s_ch(16,  29,  9, 106, dx);
          convert_s_ch(16,  38,  9, 106, dy);
          convert_s_ch(16,  47,  9, 106, dz);
          convert_s_ch(16,  56, 10, 106, da_f0);
          convert_u_ch( 8,  66,  6, 106, prn_mask_2);
          convert_u_ch( 8,  72,  8, 106, iod_2);
          convert_s_ch(16,  80,  9, 106, dx_2);
          convert_s_ch(16,  89,  9, 106, dy_2);
          convert_s_ch(16,  98,  9, 106, dz_2);
          convert_s_ch(16, 107, 10, 106, da_f0_2);
          convert_u_ch( 8, 117,  2, 106, iodp);
        };
        struct V1 {
          convert_s_ch(16,  29, 11, 106, dx);
          convert_s_ch(16,  40, 11, 106, dy);
          convert_s_ch(16,  51, 11, 106, dz);
          convert_s_ch(16,  62, 11, 106, da_f0);
          convert_s_ch( 8,  73,  8, 106, dx_dot);
          convert_s_ch( 8,  81,  8, 106, dy_dot);
          convert_s_ch( 8,  89,  8, 106, dz_dot);
          convert_s_ch( 8,  97,  8, 106, da_f1);
          convert_u_ch(16, 105, 13, 106, t_0);
          convert_u_ch( 8, 118,  2, 106, iodp);
        };
      };

      struct Type24 : public Type25 { ///< @see Fig. A-12 MIXED_CORRECTION_FAST_AND_LONG_TERM
        convert_s_ch(16,  14, 12, 12, prc_f); // pseudo range correction (fast)
        convert_u_ch( 8,  86,  4,  4, udrei);
        convert_u( 8, 110, 2, iodp);
        convert_u( 8, 112, 2, block_id); // 1-6(Type2), 14-19(Type3), 27-32(Type4), 40-45(Type5)
        convert_u( 8, 114, 2, iodf);
        // Second half of 106 bits are same as Type25(ch1)
      };

      struct Type18 { ///< @see Table A-15 IONO_GRID_POINT_MASKS
        convert_u(8, 14, 4, broadcasted_bands);
        convert_u(8, 18, 4, band);
        convert_u(8, 22, 2, iodi);
        struct mask_t {
          u8_t valid;
          static const int each_block = 15;
          union {
            u8_t linear[201];
            u8_t block[14][each_block];
          };
        };
        /**
         * @param band [0,10]
         * @return Number of mask bits
         */
        static u8_t mask_bits(const u8_t &band){
          switch(band){
            case 8:
              return 200;
            case 9:
            case 10:
              return 192;
            default:
              return 201;
          }
        }
        template <class InputT>
        static mask_t mask(const InputT *buf, const u8_t &band){
          mask_t res;
          res.valid = bits2linear(buf, res.linear, mask_bits(band), 24); // 24 bit shift
          return res;
        }
        template <class InputT>
        static mask_t mask(const InputT *buf){
          return mask(buf, band(buf));
        }
      };

      struct Type26 { ///< @see Table A-16 IONO_DELAY_CORRECTION
        convert_u(8, 14, 4, band);
        convert_u(8, 18, 4, block_id);
        convert_u_ch(16, 22, 9, 13, delay);
        convert_u_ch( 8, 31, 4, 13, error_indicator);
        convert_u(8, 22 + (13 * 15), 2, iodi); // 22 + (13 * 15) + 2 + 7(spare) + 24(parity) = 250
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

      struct Type10 { ///< @see Table A-9 Degradation factors
        convert_u(16,  14, 10, B_rcc);
        convert_u(16,  24, 10, C_ltc_lsb);
        convert_u(16,  34, 10, C_ltc_v1);
        convert_u(16,  44,  9, I_ltc_v1);
        convert_u(16,  53, 10, C_ltc_v0);
        convert_u(16,  63,  9, I_ltc_v0);
        convert_u(16,  72, 10, C_geo_lsb);
        convert_u(16,  82, 10, C_geo_v);
        convert_u(16,  92,  9, I_geo);
        convert_u( 8, 101,  6, C_er);
        convert_u(16, 107, 10, C_iono_step);
        convert_u(16, 117,  9, I_iono);
        convert_u(16, 126, 10, C_iono_ramp);
        convert_u( 8, 136,  1, RSS_UDRE);
        convert_u( 8, 137,  1, RSS_iono);
        convert_u( 8, 138,  7, C_covariance); // 138 + 7 + 81(spare) + 24(parity) = 250
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

      struct Type12 { // @see Table A-22 SBAS network Time/UTC parameters
        convert_s(32,  14, 24, A1_SNT);
        convert_s(32,  38, 32, A0_SNT);
        convert_u( 8,  70,  8, t_ot);
        convert_u( 8,  78,  8, WN_t);
        convert_s( 8,  86,  8, delta_t_LS);
        convert_u( 8,  94,  8, WN_LSF);
        convert_u( 8, 102,  8, DN);
        convert_s( 8, 110,  8, delta_t_LSF);
        convert_u( 8, 118,  3, UTC_standard_identifier);
        convert_u(32, 121, 20, TOW);
        convert_u(16, 141, 10, WN);
      };
#undef convert_s_ch
#undef convert_u_ch
#undef convert_s
#undef convert_u
    };

    struct DegradationFactors {
      float_t B_rcc;
      float_t C_ltc_lsb;
      float_t C_ltc_v1;
      int I_ltc_v1;
      float_t C_ltc_v0;
      int I_ltc_v0;
      float_t C_geo_lsb;
      float_t C_geo_v;
      int I_geo;
      float_t C_er;
      float_t C_iono_step;
      int I_iono;
      float_t C_iono_ramp;
      bool RSS_UDRE;
      bool RSS_iono;
      float_t C_covariance;

      struct raw_t {

        enum {
          SF_B_rcc,
          SF_C_ltc_lsb,
          SF_C_ltc_v1,
          SF_C_ltc_v0,
          SF_C_geo_lsb,
          SF_C_geo_v,
          SF_C_er,
          SF_C_iono_step,
          SF_C_iono_ramp,
          SF_C_covariance,

          SF_NUM,
        };
        static const float_t sf[SF_NUM];

        template <class InputT>
        static DegradationFactors fetch(const InputT *buf){
          typedef typename DataBlock::Type10 msg_t;
          DegradationFactors res = {
#define CONVERT(TARGET) \
sf[SF_ ## TARGET] * msg_t::TARGET(buf)
            CONVERT(B_rcc),
            CONVERT(C_ltc_lsb),
            CONVERT(C_ltc_v1),
            msg_t::I_ltc_v1(buf),
            CONVERT(C_ltc_v0),
            msg_t::I_ltc_v0(buf),
            CONVERT(C_geo_lsb),
            CONVERT(C_geo_v),
            msg_t::I_geo(buf),
            CONVERT(C_er),
            CONVERT(C_iono_step),
            msg_t::I_iono(buf),
            CONVERT(C_iono_ramp),
            (msg_t::RSS_UDRE(buf) == 1),
            (msg_t::RSS_iono(buf) == 1),
            CONVERT(C_covariance),
#undef CONVERT
          };
          return res;
        }
      };
    };

    class IonosphericGridPoints {

      public:
        struct PointProperty {
          float_t delay; // [m]
          float_t sigma2; // [m^2], negative value means "not monitored"

          struct raw_t {
            u16_t delay;
            u8_t error_indicator;

            template <class InputT>
            static raw_t fetch(const InputT *buf, const uint_t &ch){
              typedef typename DataBlock::Type26 msg_t;
              raw_t res = {
                msg_t::delay(buf, ch), // delay
                msg_t::error_indicator(buf, ch), // error_indicator
              };
              return res;
            }

            enum {
              DELAY_DONT_USE = 0x1FF,
              ERROR_INDICATOR_NOT_MONITORED = 15,
            };

            static const raw_t unavailable;

            static float_t raw2delay(const u16_t &v){
              return 0.125 * v;
            }

            static float_t raw2sigma2(const u8_t &v){
              switch(v){ ///< @see Table A-17
                case 0:   return 0.0084;
                case 1:   return 0.0333;
                case 2:   return 0.0749;
                case 3:   return 0.1331;
                case 4:   return 0.2079;
                case 5:   return 0.2994;
                case 6:   return 0.4075;
                case 7:   return 0.5322;
                case 8:   return 0.6735;
                case 9:   return 0.8315;
                case 10:  return 1.1974;
                case 11:  return 1.8709;
                case 12:  return 3.3260;
                case 13:  return 20.7870;
                case 14:  return 187.0826;
              }
              return -1;
            }
            operator PointProperty() const {
              PointProperty res = {
                raw2delay(delay),
                raw2sigma2(error_indicator),
              };
              return res;
            }
            bool is_available() const {
              return (delay < DELAY_DONT_USE)
                  && (error_indicator < ERROR_INDICATOR_NOT_MONITORED);
            }
          };

          bool is_available() const {
            return (sigma2 > 0);
          }

          static const PointProperty unavailable;
        };

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
        static position_t position(const u8_t &band, const u8_t &mask_pos){
          position_t res;
          if(band <= 8){
            u8_t row_index_28(band & (~(u8_t)0x01)); // where 28 grid points on the same longitude are appeared
            int row_index(0), col_index((int)mask_pos);
            do{
              int points(row_index_28 == row_index ? 28 : 27);
              if(col_index < points){
                col_index -= 2; // col_index => [-2, 24 (or 25)]
                if((points > 27) && (band % 2 == 1)){ // col_index => [-3, 24]
                  col_index--;
                }
                break;
              }
              col_index -= points;
              row_index++;

              points = 23;
              if(col_index < points){break;}
              col_index -= points;
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

          PointProperty compute_property(
#if defined(SWIG) // work around for SWIG parser error
              const typename PointProperty::raw_t *selected
#else
              const typename PointProperty::raw_t *(&selected)[4]
#endif
              ){
            float_t delay_raw(0), sigma2(0);
            bool use_sigma(true);
            for(int i(0); i <= 3; i++){
              if(!checked[i]){continue;}
              delay_raw += weight[i] * selected[i]->delay;
              if(selected[i]->error_indicator == PointProperty::raw_t::ERROR_INDICATOR_NOT_MONITORED){
                use_sigma = false;
              }else{
                sigma2 += weight[i] * PointProperty::raw_t::raw2sigma2(selected[i]->error_indicator);
              }
            }
            PointProperty res = {
              PointProperty::raw_t::raw2delay(delay_raw),
              use_sigma
                  ? sigma2
                  : PointProperty::raw_t::raw2sigma2(PointProperty::raw_t::ERROR_INDICATOR_NOT_MONITORED),
            };
            return res;
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

      protected:
        typename PointProperty::raw_t properties[position_index_t::LAT_INDEX_MAX + 1][position_index_t::LNG_INDEX_MAX + 1];

      public:
        template <class T>
        T check_availability_hook(trapezoid_t &in, const T &out) const {
          return out; // for debug
        }
        /**
         * @return available IGP(s)
         */
        int_t check_availability(trapezoid_t &target,
#if defined(SWIG) // work around for SWIG parser error
            const typename PointProperty::raw_t *cache
#else
            const typename PointProperty::raw_t *(&cache)[4]
#endif
            ) const {
          int_t res(0);
          for(int i(0); i < 4; ++i){
            if(target.checked[i]){res++; continue;}
            position_index_t index(target.igp[i]);
            if((cache[i] = &properties[index.lat_index][index.lng_index])->is_available()){
              target.checked[i] = true;
              res++;
            }
          }
          return check_availability_hook(target, res);
        }

        /**
         * Select appropriate grids and perform interpolation with the selected grids
         *
         * @param latitude_deg Latitude in degrees
         * @param longitude_deg Longitude in degrees
         * @return Interpolated result; if interpolation fails, PointProperty::unavailable will be returned.
         * @see A.4.4.10.2, A.4.4.10.3
         */
        PointProperty interpolate(const float_t &latitude_deg, const float_t &longitude_deg) const {
          pivot_t pivot(get_pivot(latitude_deg, longitude_deg));
          const typename PointProperty::raw_t *selected[4];

          bool north_hemisphere(latitude_deg >= 0);
          int_t lat_deg_abs(pivot.igp.latitude_deg * (north_hemisphere ? 1 : -1));

          if(lat_deg_abs <= 55){
            trapezoid_t rect_5_5(trapezoid_t::generate_rectangle(pivot.igp, north_hemisphere ? 5 : -5, 5)); // A4.4.10.2 a-1)
            switch(check_availability(rect_5_5, selected)){
              case 4: // a-1)
                rect_5_5.compute_weight(pivot.delta.latitude_deg, pivot.delta.longitude_deg);
                return rect_5_5.compute_property(selected);
              case 3: // a-2)
                if(rect_5_5.compute_weight_three(pivot.delta.latitude_deg, pivot.delta.longitude_deg)){
                  return rect_5_5.compute_property(selected);
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

              if((rect_10_10[i].availability = check_availability(rect_10_10[i].rect, selected)) == 4){
                rect_10_10[i].rect.compute_weight(rect_10_10[i].delta_lat, rect_10_10[i].delta_lng);
                return rect_10_10[i].rect.compute_property(selected);
              }
            }
            for(int i(0); i < 4; ++i){ // a-4) three point interpolation
              if((rect_10_10[i].availability == 3)
                  && rect_10_10[i].rect.compute_weight_three(rect_10_10[i].delta_lat, rect_10_10[i].delta_lng)){
                return rect_10_10[i].rect.compute_property(selected);
              }
            }
          }else if(lat_deg_abs <= 70){
            trapezoid_t rect_5_10(trapezoid_t::generate_rectangle(pivot.igp, north_hemisphere ? 5 : -5, 10)); // A4.4.10.2 b-1)
            switch(check_availability(rect_5_10, selected)){
              case 4: // b-1)
                rect_5_10.compute_weight(pivot.delta.latitude_deg, pivot.delta.longitude_deg);
                return rect_5_10.compute_property(selected);
              case 3: // b-2)
                if(rect_5_10.compute_weight_three(pivot.delta.latitude_deg, pivot.delta.longitude_deg)){
                  return rect_5_10.compute_property(selected);
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

              if((rect_10_10[i].availability = check_availability(rect_10_10[i].rect, selected)) == 4){
                rect_10_10[i].rect.compute_weight(rect_10_10[i].delta_lat, rect_10_10[i].delta_lng);
                return rect_10_10[i].rect.compute_property(selected);
              }
            }
            for(int i(0); i < 2; ++i){ // b-4) three point interpolation
              if((rect_10_10[i].availability == 3)
                  && rect_10_10[i].rect.compute_weight_three(rect_10_10[i].delta_lat, rect_10_10[i].delta_lng)){
                return rect_10_10[i].rect.compute_property(selected);
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
              if(check_availability(rect_10_10, selected) == 4){
                rect_10_10.compute_weight(pivot.delta.latitude_deg, pivot.delta.longitude_deg);
                return rect_10_10.compute_property(selected);
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
              if(check_again && (check_availability(rect_10_10, selected) == 4)){
                rect_10_10.compute_weight(pivot.delta.latitude_deg, pivot.delta.longitude_deg);
                return rect_10_10.compute_property(selected);
              }
            }
          }else{ // pole
            trapezoid_t rect(trapezoid_t::generate_rectangle_pole(pivot.igp));
            if(check_availability(rect, selected) == 4){
              rect.compute_weight_pole(pivot.delta.latitude_deg, pivot.delta.longitude_deg);
              return rect.compute_property(selected);
            }
          }

          // Correction unavailable
          return PointProperty::unavailable;
        }

      protected:
        static const u8_t IODI_INVALID = 4; // valid value ranges from 0 to 3.
        struct {
          u8_t iodi;
          typename DataBlock::Type18::mask_t mask;
          void clear() {
            iodi = IODI_INVALID;
            mask.valid = 0;
          }
        } masks[11], masks_new[11]; // "masks" for normal use, "masks_new" for temporal use for iodi transition

        /**
         * Update mask and IGPs based on new mask having new IODI
         * @param band band number
         * @see 2.1.4.9.3 "The equipment shall be able to store and use two IGP masks ..."
         */
        void transit_to_new_mask(const u8_t &band) {
          // remove points which is not activated in the new mask
          typename DataBlock::Type18::mask_t
              &mask_old(masks[band].mask), &mask_new(masks_new[band].mask);
          for(u8_t i(0), j(0); i < mask_old.valid; ++i){
            bool still_use(false);
            for(; j < mask_new.valid; ++j){
              if(mask_old.linear[i] < mask_new.linear[j]){
                break;
              }
              if(mask_old.linear[i] == mask_new.linear[j]){
                still_use = true;
                ++j;
                break;
              }
            }
            if(still_use){continue;}
            position_index_t index(position(band, mask_old.linear[i]));
            properties[index.lat_index][index.lng_index] = PointProperty::raw_t::unavailable;
          }
          masks[band] = masks_new[band];
          masks_new[band].clear();
        }

        void clear_igp() {
          for(unsigned int i(0); i < sizeof(properties) / sizeof(properties[0]); ++i){
            for(unsigned int j(0); j < sizeof(properties[0]) / sizeof(properties[0][0]); ++j){
              properties[i][j] = PointProperty::raw_t::unavailable;
            }
          }
        }

        void clear_mask(){
          for(unsigned int i(0); i < sizeof(masks) / sizeof(masks[0]); ++i){
            masks[i].clear();
          }
          for(unsigned int i(0); i < sizeof(masks_new) / sizeof(masks_new[0]); ++i){
            masks_new[i].clear();
          }
        }
      public:
        /**
         * Update mask
         * @param band IGP band
         * @param iodi_new Issue of data, ionospheric
         * @param mask_new New mask
         * @return True if mask is up to date, otherwise false.
         */
        bool update_mask(const u8_t &band,
            const u8_t &iodi_new,
            const typename DataBlock::Type18::mask_t &mask_new){
          if((masks[band].iodi == iodi_new) || (masks_new[band].iodi == iodi_new)){
            // lazy decline, because the same iodi means up to date
            return true;
          }
          if(masks[band].iodi == IODI_INVALID){
            masks[band].iodi = iodi_new;
            masks[band].mask = mask_new;
            return true;
          }
          if(masks_new[band].iodi == IODI_INVALID){
            masks_new[band].iodi = iodi_new;
            masks_new[band].mask = mask_new;
            return true;
          }
          return false;
        }
        /**
         * Update mask with broadcasted data
         * @param type18 Type 18 message
         * @return True if mask is updated and changed, otherwise false.
         */
        template <class Input>
        bool update_mask(const Input *type18){
          typedef typename DataBlock::Type18 msg_t;
          u8_t band(msg_t::band(type18));
          return update_mask(band, msg_t::iodi(type18), msg_t::mask(type18, band));
        }

        /**
         * Register new ionospheric grind point property
         * @param IGP position
         * @param prop New property
         * @return Always true
         */
        bool register_igp(const position_t &pos, const typename PointProperty::raw_t &prop){
          position_index_t index(pos);
          properties[index.lat_index][index.lng_index] = prop;
          return true;
        }
        /**
         * Register new properties of ionospheric grid points (IGPs) with broadcasted data
         * @param type26 Type 26 message
         * @return True if IGPs are registered, otherwise false
         */
        template <class InputT>
        bool register_igp(const InputT *type26){
          typedef typename DataBlock::Type26 msg_t;
          u8_t band(msg_t::band(type26)), iodi(msg_t::iodi(type26));
          if(masks[band].iodi != iodi){
            if(masks_new[band].iodi != iodi){
              return false;
            }
            // Change mask due to new arrival of IODI
            transit_to_new_mask(band);
          }
          u8_t *mask_pos(masks[band].mask.block[msg_t::block_id(type26)]);
          int i_max(masks[band].mask.valid - (mask_pos - masks[band].mask.linear));
          if(i_max > DataBlock::Type18::mask_t::each_block){
            i_max = DataBlock::Type18::mask_t::each_block;
          }
          for(int i(0); i < i_max; ++i, ++mask_pos){
            register_igp(position(band, *mask_pos), PointProperty::raw_t::fetch(type26, i));
          }
          return true;
        }

        IonosphericGridPoints(){
          clear_igp();
          clear_mask();
        }
        virtual ~IonosphericGridPoints(){}

        /**
         * Print delay map with ASCII characters
         */
        friend std::ostream &operator<<(std::ostream &out, const IonosphericGridPoints &igp){
          static const char base32_chr[] = "0123456789ABCdEFGHiJKLMNoPQRSTUV"; // base 32

          // the first line
          out << "     ";
          for(std::size_t j(0); j < sizeof(igp.properties[0]) / sizeof(igp.properties[0][0]); ++j){
            int lng(position_index_t::idx2lng(j));
            out << (lng == 0 ? '0' : (lng % 90 == 0 ? '|' : (lng % 30 == 0 ? '+' : ' ')));
          }
          out << std::endl;

          for(std::size_t i(0); i < sizeof(igp.properties) / sizeof(igp.properties[0]); ++i){
            out.width(4);
            out << position_index_t::idx2lat(i) << ' ';
            for(std::size_t j(0); j < sizeof(igp.properties[0]) / sizeof(igp.properties[0][0]); ++j){
              if(!igp.properties[i][j].is_available()){
                out << ' ';
              }else{
                out << base32_chr[igp.properties[i][j].delay >> 4]; // 0-510 => 0-31
              }
            }
            out << std::endl;
          }
          return out;
        }

        /**
         * Calculate correction value in accordance with ionospheric model
         *
         * @param relative_pos satellite position (relative position, NEU)
         * @param usrllh user position (absolute position, LLH)
         * @return correction in meters
         * @see A.4.4.10
         */
        PointProperty iono_correction(
            const enu_t &relative_pos,
            const llh_t &usrllh) const {

          // A.4.4.10.1 Pierce point calculation
          typename gps_space_node_t::pierce_point_res_t pp(gps_space_node_t::pierce_point(relative_pos, usrllh));

          // A.4.4.10.2 IGP selection, and A.4.4.10.3 Interpolation
          PointProperty prop(interpolate(pp.latitude / M_PI * 180, pp.longitude / M_PI * 180));
          if(prop.is_available()){

            // A.4.4.10.4 Compute slant delay
            float_t fpp(gps_space_node_t::slant_factor(relative_pos));
            prop.delay *= fpp; // (A-41)
            prop.sigma2 *= (fpp * fpp); // (A-43)
          }

          return prop;
        }
    };

    class IonosphericGridPoints_with_Timeout : public IonosphericGridPoints {
      protected:
        gps_time_t t_last_mask;
        gps_time_t t_last_correction;

        bool is_mask_timeout(const gps_time_t &t_reception, const bool &LNAV_VNAV_LP_LPV_approach){
          float_t delta_t(t_last_mask.interval(t_reception));
          return LNAV_VNAV_LP_LPV_approach
              ? (delta_t >= Timing::values[Timing::IONOSPHERIC_GRID_MASK].timeout_LNAV_VNAV_LP_LPV_approach)
              : (delta_t >= Timing::values[Timing::IONOSPHERIC_GRID_MASK].timeout_EN_Route_Terminal_LNAV);
        }
        bool is_correction_timeout(const gps_time_t &t_reception, const bool &LNAV_VNAV_LP_LPV_approach){
          float_t delta_t(t_last_correction.interval(t_reception));
          return LNAV_VNAV_LP_LPV_approach
              ? (delta_t >= Timing::values[Timing::IONOSPHERIC_CORRECTIONS].timeout_LNAV_VNAV_LP_LPV_approach)
              : (delta_t >= Timing::values[Timing::IONOSPHERIC_CORRECTIONS].timeout_EN_Route_Terminal_LNAV);
        }

      public:
        IonosphericGridPoints_with_Timeout()
            : IonosphericGridPoints(),
            t_last_mask(0, 0), t_last_correction(0, 0) {}

        using IonosphericGridPoints::update_mask;
        using IonosphericGridPoints::register_igp;
        using IonosphericGridPoints::iono_correction;

        template <class Input>
        bool update_mask(
            const Input *type18,
            const gps_time_t &t_reception, const bool &LNAV_VNAV_LP_LPV_approach = false){

          if(is_mask_timeout(t_reception, LNAV_VNAV_LP_LPV_approach)){
            IonosphericGridPoints::clear_mask();
          }
          bool res(update_mask(type18));
          if(res){
            t_last_mask = t_reception;
          }
          return res;
        }

        template <class InputT>
        bool register_igp(
            const InputT *type26,
            const gps_time_t &t_reception, const bool &LNAV_VNAV_LP_LPV_approach = false){

          if(is_mask_timeout(t_reception, LNAV_VNAV_LP_LPV_approach)){
            IonosphericGridPoints::clear_mask();
            return false;
          }
          if(is_correction_timeout(t_reception, LNAV_VNAV_LP_LPV_approach)){
            IonosphericGridPoints::clear_igp();
          }
          bool res(register_igp(type26));
          if(res){
            t_last_correction = t_reception;
          }
          return res;
        }

        typename IonosphericGridPoints::PointProperty iono_correction(
            const enu_t &relative_pos, const llh_t &usrllh,
            const gps_time_t &t, const bool &LNAV_VNAV_LP_LPV_approach = false) const {

          if(is_correction_timeout(t, LNAV_VNAV_LP_LPV_approach)){
            return IonosphericGridPoints::PointProperty::unavailable;
          }
          return iono_correction(relative_pos, usrllh);
        }
    };

    /**
     * Calculate Sagnac correction range in meter, which must be accounted
     * for residual calculation of pseudo range.
     *
     * @param sat_pos Satellite position
     * @param usr_pos User position
     * @return correction range
     * @see A.4.4.11
     * @see SBAS_SpaceNode::Satellite::Ephemeris::constellation
     */
    static float_t sagnac_correction(
        const xyz_t sat_pos,
        const xyz_t usr_pos) {
      return WGS84::Omega_Earth_IAU
          * (sat_pos.x() * usr_pos.y() - sat_pos.y() * usr_pos.x())
          / gps_space_node_t::light_speed;
    }

    /**
     * Calculate correction value in accordance with tropospheric model
     *
     * @param year_utc UTC floating-point year
     * @param relative_pos satellite position (relative position, NEU)
     * @param usrllh user position (absolute position, LLH)
     * @see A.4.2.4
     * @return correction in meters
     */
    static float_t tropo_correction(
        const float_t &year_utc,
        const enu_t &relative_pos,
        const llh_t &usrllh){

      if(usrllh.height() > 10E3){ // same as RTKlib; troposphere ranges from 0 to approximately 11km
        return 0;
      }

      union MeteologicalParameter {
        struct {
          float_t p, T, e, beta, lambda; // mbar, K, mbar, K/m, [dimless]
        };
        float_t v[5];
      };

      static const struct {
        MeteologicalParameter average, seasonal_variation;
      } preset[] = {
        {{1013.25, 299.65, 26.31, 6.30E-3, 2.77}, { 0.00,  0.00, 0.00, 0.00E-3, 0.00}}, // 15
        {{1017.25, 294.15, 21.79, 6.05E-3, 3.15}, {-3.75,  7.00, 8.85, 0.25E-3, 0.33}}, // 30
        {{1015.75, 283.15, 11.66, 5.58E-3, 2.57}, {-2.25, 11.00, 7.24, 0.32E-3, 0.46}}, // 45
        {{1011.75, 272.15,  6.78, 5.39E-3, 1.81}, {-1.75, 15.00, 5.36, 0.81E-3, 0.74}}, // 60
        {{1013.00, 263.65,  4.11, 4.53E-3, 1.55}, {-0.50, 14.50, 3.39, 0.62E-3, 0.30}}, // 75
      }; // Table A-2
      static const int preset_langth(sizeof(preset) / sizeof(preset[0]));

      float_t preset_idx_f(std::abs(usrllh.latitude()) / (M_PI / 180 * 15));
      int preset_idx(preset_idx_f);

      MeteologicalParameter average, seasonal_variation;
      if(preset_idx == 0){
        average = preset[preset_idx].average;
        seasonal_variation = preset[preset_idx].seasonal_variation;
      }else if(preset_idx >= (preset_langth - 1)){
        average = preset[preset_langth - 1].average;
        seasonal_variation = preset[preset_langth - 1].seasonal_variation;
      }else{
        // linear interpolation
        float_t
            weight_b(preset_idx_f - preset_idx),
            weight_a(1. - weight_b);
        for(std::size_t j(0); j < sizeof(average.v) / sizeof(average.v[0]); ++j){
          average.v[j]
              = preset[preset_idx].average.v[j] * weight_a
                + preset[preset_idx + 1].average.v[j] * weight_b; // (A-4)
          seasonal_variation.v[j]
              = preset[preset_idx].seasonal_variation.v[j] * weight_a
                + preset[preset_idx + 1].seasonal_variation.v[j] * weight_b; // (A-5)
        }
      }

      float_t d_hyd, d_wet;
      {
        // (A-3)
        MeteologicalParameter param;
        {
          float_t Dmin_year(((usrllh.latitude() < 0) ? 211 : 28) / 365.25);
          float_t year_int;
          float_t k(std::cos(M_PI * 2 * (std::modf(year_utc, &year_int) - Dmin_year)));
          for(std::size_t j(0); j < sizeof(param.v) / sizeof(param.v[0]); ++j){
            param.v[j] = average.v[j] - seasonal_variation.v[j] * k;
          }
        }

        static const float_t
            k1(77.604), k2(382000), Rd(287.054), gm(9.784); // K/mbar, K^2/mbar, J/(kg*K), m/s^2
        // Zero-altitude zenith delay term (z_hyd, z_wet)
        float_t
            z_hyd(1E-6 * k1 * Rd * param.p / gm), // (A-6)
            z_wet(1E-6 * k2 * Rd / (gm * (param.lambda + 1) - param.beta * Rd) * param.e / param.T); // (A-7)

        {
          const float_t &h(usrllh.height()); // Altitude (m)
          static const float_t g(9.80665); // m/s^2
          if(h > 0){
            float_t x(1. - (param.beta * h / param.T)), y(g / Rd / param.beta);
            d_hyd = std::pow(x, y) * z_hyd; // (A-8)
            d_wet = std::pow(x, y * (param.lambda + 1) - 1) * z_wet; // (A-9)
          }else{
            d_hyd = z_hyd;
            d_wet = z_wet;
          }
        }
      }

      float_t m_el;
      {
        // Elevation (rad)
        float_t el(relative_pos.elevation());
        m_el = 1.001 / std::sqrt(0.002001 + std::pow(std::sin(el), 2)); // (A-10a)
      }

      return -(d_hyd + d_wet) * m_el;
    }

    /**
     * UTC parameters
     *
     */
    struct UTC_Parameters {
      float_t A1;          ///< UTC parameter for SBAS network time (s/s)
      float_t A0;          ///< UTC parameter for SBAS network time (s)
      uint_t t_ot;         ///< Epoch time (UTC) (s)
      uint_t WN_t;         ///< Epoch time (UTC) (weeks)
      int_t delta_t_LS;    ///< Current leap seconds (s)
      uint_t WN_LSF;       ///< Last leap second update week (weeks)
      uint_t DN;           ///< Last leap second update day (days)
      int_t delta_t_LSF;   ///< Updated leap seconds (s)

      struct raw_t {
        s32_t A1;           ///< UTC parameter (-50, s/s)
        s32_t A0;           ///< UTC parameter (-30, s)
        u8_t  t_ot;         ///< Epoch time (UTC) (12, s)
        u8_t  WN_t;         ///< Epoch time (UTC) (weeks, truncated)
        s8_t  delta_t_LS;   ///< Current leap seconds (s)
        u8_t  WN_LSF;       ///< Last leap second update week (weeks, truncated)
        u8_t  DN;           ///< Last leap second update day (days)
        s8_t  delta_t_LSF;  ///< Updated leap seconds (s)

        template <class InputT>
        static raw_t fetch(const InputT *buf){
          typedef typename DataBlock::Type12 msg_t;
          raw_t res = {
            msg_t::A1_SNT(buf),
            msg_t::A0_SNT(buf),
            msg_t::t_ot(buf),
            msg_t::WN_t(buf),
            msg_t::delta_t_LS(buf),
            msg_t::WN_LSF(buf),
            msg_t::DN(buf),
            msg_t::delta_t_LSF(buf),
          };
          return res;
        }

        enum {
          SF_A1,
          SF_A0,

          SF_NUM,
        };
        static const float_t sf[SF_NUM];

        operator UTC_Parameters() const {
          UTC_Parameters converted;
#define CONVERT(TARGET) \
{converted.TARGET = sf[SF_ ## TARGET] * TARGET;}
            CONVERT(A1);
            CONVERT(A0);
            converted.t_ot = ((uint_t)t_ot) << 12;
            converted.WN_t = WN_t;
            converted.delta_t_LS = delta_t_LS;
            converted.WN_LSF = WN_LSF;
            converted.DN = DN;
            converted.delta_t_LSF = delta_t_LSF;
#undef CONVERT
          return converted;
        };
      };
    };

    class SatelliteProperties {
      public:
        typedef typename gps_space_node_t::SatelliteProperties::constellation_t constellation_t;

        /**
         * SBAS Ephemeris
         * @see Table A-18
         */
        struct Ephemeris {
          uint_t svid;            ///< Satellite number
          uint_t WN;              ///< Week number

          float_t t_0;            ///< Time of applicability (s) <= time of a week
          float_t URA;            ///< User range accuracy (m)
          float_t x, y, z;        ///< ECEF position (m)
          float_t dx, dy, dz;     ///< ECEF velocity (m/s)
          float_t ddx, ddy, ddz;  ///< ECEF acceleration (m/s^2)
          float_t a_Gf0;           ///< Clock correction parameter (s)
          float_t a_Gf1;           ///< Clock correction parameter (s/s)

          /**
           * Adjust time of ephemeris with time of current
           */
          void adjust_time(const gps_time_t &t_current){
            WN = t_current.week;
            float_t sec_of_a_day(std::fmod(t_current.seconds, gps_time_t::seconds_day)), t_0_orig(t_0);
            t_0 += (t_current.seconds - sec_of_a_day);

            // roll over check
            float_t delta(sec_of_a_day - t_0_orig);
            if(delta > (gps_time_t::seconds_day / 4 * 3)){
              // 0 --> t_0 ---------(3/4)---------> current --> day
              t_0 += gps_time_t::seconds_day; // probably, current --> t_0
              if(t_0 >= gps_time_t::seconds_week){
                WN++;
                t_0 -= gps_time_t::seconds_week;
              }
            }else if(-delta > (gps_time_t::seconds_day / 4 * 3)){
              // 0 --> current ---------(3/4)---------> t_0 --> day
              t_0 -= gps_time_t::seconds_day; // probably, t_0 --> current
              if(t_0 < 0){
                WN--;
                t_0 += gps_time_t::seconds_week;
              }
            }
          }

          /**
           * Check availability
           * @param t target time
           * @return always true, because timeout, which is based on the reception time,
           * is the only way to make this ephemeris unavailable.
           * This class does not take the reception time into account.
           * @see 2.1.1.4.9, A.4.5.1.3.3
           */
          bool is_valid(const gps_time_t &t) const {
            return ((WN > 0) || (t_0 >= 0));
          }

          /**
           * Check fitness based on time of applicability (t_0) of ephemeris
           * @param t target time
           * @return true when best fit, otherwise, false
           * @see A.4.5.1.3.3 (A-56)
           */
          bool maybe_better_one_avilable(const gps_time_t &t) const {
            float_t delta_t(-t.interval(WN, t_0));
            return (delta_t < 0) || (delta_t > Timing::values[Timing::GEO_NAVIGATION_DATA].interval);
          }

          float_t clock_error(const gps_time_t &t) const {
            float_t t_G(-t.interval(WN, 0)); // t_0 is expected to be modified as time of week
            return a_Gf0 + a_Gf1 * (t_G - t_0); // Eq.(A-45), delay is positive
          }

          float_t clock_error_dot(const gps_time_t &t) const {
            return a_Gf1;
          }

          constellation_t constellation(
              const gps_time_t &t_tx, const float_t &dt_transit = 0,
              const bool &with_velocity = true) const {

            float_t delta_t(-t_tx.interval(WN, t_0)), delta_t2(delta_t * delta_t / 2);

            constellation_t res = {
              xyz_t(
                  x + dx * delta_t + ddx * delta_t2,
                  y + dy * delta_t + ddy * delta_t2,
                  z + dz * delta_t + ddz * delta_t2).after(dt_transit), // Eq. (A-44)
              xyz_t(
                  dx + ddx * delta_t,
                  dy + ddy * delta_t,
                  dz + ddz * delta_t).after(dt_transit),
            };

            /* Sagnac correction, which is expected to perform before geometric distance calculation,
             * can be skipped by using non-zero dt_transit.
             * This is because the return values is corrected to be along with the coordinate
             * at the reception time.
             * @see SBAS_SpaceNode::sagnac_correction
             */

            return res;
          }

          static const float_t URA_table[15];

          inline static float_t URA_meter(const int_t &index){
            return gps_space_node_t::SatelliteProperties::Ephemeris::URA_meter(index, URA_table);
          }
          inline static int_t URA_index(const float_t &meter){
            return gps_space_node_t::SatelliteProperties::Ephemeris::URA_index(meter, URA_table);
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

            template <class InputT>
            static raw_t fetch(const InputT *buf, const u8_t &sv_number = 0){
              typedef typename DataBlock::Type9 msg_t;
              raw_t res = {
                sv_number, // svid
                msg_t::t0(buf), // t_0
                msg_t::ura(buf), // URA
                msg_t::x(buf), msg_t::y(buf), msg_t::z(buf), // x, y, z
                msg_t::dx(buf), msg_t::dy(buf), msg_t::dz(buf), // dx, dy, dz
                msg_t::ddx(buf), msg_t::ddy(buf), msg_t::ddz(buf), // ddx, ddy, ddz
                msg_t::a_Gf0(buf), msg_t::a_Gf1(buf), // a_Gf0, a_Gf1
              };
              return res;
            }
            template <class BufferT>
            void dump(BufferT *dst){
              typedef typename DataBlock::Type9 msg_t;
#define dump_item(name) msg_t:: name ## _set(dst, name)
              DataBlock::message_type_set(dst, 9);
              msg_t::t0_set(dst, t_0);  msg_t::ura_set(dst, URA);
              dump_item(x);   dump_item(y);   dump_item(z);
              dump_item(dx);  dump_item(dy);  dump_item(dz);
              dump_item(ddx); dump_item(ddy); dump_item(ddz);
              dump_item(a_Gf0); dump_item(a_Gf1);
#undef dump_item
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

              converted.URA = URA_meter(URA);
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
{TARGET_DST = std::floor((TARGET_SRC / sf[SF_ ## TARGET_SF]) + 0.5);}
#define CONVERT2(TARGET, TARGET_SF) CONVERT3(TARGET, eph.TARGET, TARGET_SF)
#define CONVERT(TARGET) CONVERT2(TARGET, TARGET)
              svid = eph.svid;

              URA = URA_index(eph.URA);
              CONVERT3(t_0, std::fmod(eph.t_0, gps_time_t::seconds_day), t_0);
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

          gps_time_t base_time() const {
            return gps_time_t(WN, t_0);
          }
        };

        struct Ephemeris_with_Timeout : public Ephemeris {
          gps_time_t t_reception;

          Ephemeris_with_Timeout() : Ephemeris(), t_reception(0, 0) {}

          /**
           * Constructor to convert Ephemeris
           * reception time is interpreted to be equivalent to time of applicability (t_0) of eph,
           * therefore, eph is assumed that its t_0, i.e., week number and time of week,
           * has already been adjusted.
           * @param eph Epehemris
           */
          Ephemeris_with_Timeout(const Ephemeris &eph)
              : Ephemeris(eph), t_reception(eph.WN, eph.t_0) {}

          /**
           * Constructor to convert Ephemeris with reception time
           * reception time will be adjusted with time of reception.
           * @param eph Ephemeris
           * @param t_rx Reception time
           */
          Ephemeris_with_Timeout(const Ephemeris &eph, const gps_time_t &t_rx)
              : Ephemeris(eph), t_reception(t_rx) {
            Ephemeris::adjust_time(t_rx);
          }

          /**
           * Check availability on EN_Route, Terminal, or LNAV
           * @param t target time
           * @return true when interval between message reception time
           * and current time is within timeout threshold, otherwise false.
           * @see 2.1.1.4.9, A.4.5.1.3.3
           */
          bool is_valid_EN_Route_Terminal_LNAV(const gps_time_t &t) const {
            float_t delta_t(t_reception.interval(t));
            return delta_t < Timing::values[
                Timing::GEO_NAVIGATION_DATA].timeout_EN_Route_Terminal_LNAV;
          }

          /**
           * Check availability on LNAV, VNAV, LP, or LPV approach
           * @param t target time
           * @return true when interval between message reception time
           * and current time is within timeout threshold, otherwise false.
           * @see 2.1.1.4.9, A.4.5.1.3.3
           */
          bool is_valid_LNAV_VNAV_LP_LPV_approach(const gps_time_t &t) const {
            float_t delta_t(t_reception.interval(t));
            return delta_t < Timing::values[
                Timing::GEO_NAVIGATION_DATA].timeout_LNAV_VNAV_LP_LPV_approach;
          }

          /**
           * Return reception time as base time, which is different from non-timeout version for time of applicability.
           * This function will be used to sort multiple ephemeris in order of reception time.
           * @see Ephemeris::base_time()
           */
          gps_time_t base_time() const {
            return t_reception;
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

          ///< Upgrade to ephemeris
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
          ///< Downgrade from ephemeris
          Almanac &operator=(const Ephemeris &eph) {
            prn = eph.svid;
            t_0 = eph.t_0;
            x = eph.x; y = eph.y; z = eph.z;
            dx = eph.dx; dy = eph.dy; dz = eph.dz;
            return *this;
          }

          struct raw_t {
            u8_t data_id;
            u8_t prn;         ///< PRN number
            u8_t SV_health;   ///< Health status
            s16_t x, y, z;    ///< ECEF position (2600(xy), 26000(z), m)
            s8_t dx, dy, dz;  ///< ECEF velocity (10(xy), 60(z), m/s)
            u16_t t_0;        ///< Time of applicability (64, s)

            template <class InputT>
            static raw_t fetch(const InputT *buf, const uint_t &ch){
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
            template <class BufferT>
            void dump(BufferT *dst, const uint_t &ch = 0){
              typedef typename DataBlock::Type17 msg_t;
#define dump_item(name) msg_t:: name ## _set(dst, ch, name)
              if(ch == 0){
                DataBlock::message_type_set(dst, 17);
                msg_t::t0_set(dst, t_0);
              }
              msg_t::id_set(dst, ch, data_id);
              dump_item(prn);
              msg_t::health_status_set(dst, ch, SV_health);
              dump_item(x);   dump_item(y);   dump_item(z);
              dump_item(dx);  dump_item(dy);  dump_item(dz);
#undef dump_item
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

    class Satellite : public SatelliteProperties {
      friend class SBAS_SpaceNode;
      public:
        typedef typename SatelliteProperties::Ephemeris_with_Timeout eph_t;
        typedef typename gps_space_node_t::template PropertyHistory<eph_t> eph_list_t;
        typedef IonosphericGridPoints_with_Timeout igp_t;
      protected:
        eph_list_t eph_history;
        igp_t igp;
      public:
        Satellite() : eph_history(), igp() {
          // setup first ephemeris as invalid one
          eph_t &eph_current(const_cast<eph_t &>(eph_history.current()));
          eph_current.WN = 0;
          eph_current.t_0 = -1;
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

        const eph_t &ephemeris() const {
          return eph_history.current();
        }

        /**
         * Select appropriate ephemeris within registered ones.
         *
         * @param target_time time at measurement
         * @param LNAV_VNAV_LP_LPV_approach
         * @return if true, appropriate ephemeris is selected, otherwise, not selected.
         */
        bool select_ephemeris(
            const gps_time_t &target_time,
            const bool &LNAV_VNAV_LP_LPV_approach = false){
          bool (eph_t::*is_valid_func)(const gps_time_t &) const (
              LNAV_VNAV_LP_LPV_approach
                ? &eph_t::is_valid_LNAV_VNAV_LP_LPV_approach
                : &eph_t::is_valid_EN_Route_Terminal_LNAV);
          return (ephemeris().*is_valid_func)(target_time) // conservative
              || eph_history.select(target_time, is_valid_func);
        }

        const igp_t &ionospheric_grid_points() const {
          return igp;
        }
    };

  public:
    typedef std::map<int, Satellite> satellites_t;
  protected:
    satellites_t _satellites;
  public:
    SBAS_SpaceNode() : _satellites() {

    }
    ~SBAS_SpaceNode(){
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
    void update_all_ephemeris(const gps_time_t &target_time) {
      for(typename satellites_t::iterator it(_satellites.begin());
          it != _satellites.end(); ++it){
        it->second.select_ephemeris(target_time);
      }
    }

    typedef std::vector<std::pair<int, const Satellite *> > available_satellites_t;
    /**
     * Return available satellites
     * @param lng_deg longitude of user position
     * @return (available_satellites_t) available satellites, nearer is faster
     */
    available_satellites_t available_satellites(const float_t &lng_deg) const {
      available_satellites_t res;
      typename KnownSatellites::list_t nearest(KnownSatellites::nearest_ordered(lng_deg));
      for(typename KnownSatellites::list_t::const_iterator it(nearest.begin());
          it != nearest.end();
          ++it){
        int prn((*it)->prn);
        if(!has_satellite(prn)){continue;}
        res.push_back(std::make_pair(prn, &(const_cast<SBAS_SpaceNode *>(this)->_satellites[prn])));
      }
      return res;
    }

    template <class InputT>
    MessageType decode_message(
        const InputT *buf, const int &prn, const gps_time_t &t_reception,
        const bool &LNAV_VNAV_LP_LPV_approach = false){

      MessageType message_type((MessageType)DataBlock::message_type(buf));
      Satellite &sat(_satellites[prn]);

      switch(message_type){
        case GEO_NAVIGATION: { // 9
          typename Satellite::eph_t eph(
              Satellite::eph_t::raw_t::fetch(buf, prn), t_reception);
          sat.eph_history.add(eph);
          break;
        }
        case DEGRADATION_PARAMS: { // 10
          DegradationFactors dfactor(DegradationFactors::raw_t::fetch(buf));
          break;
        }
        case IONO_GRID_POINT_MASKS: // 18
          sat.igp.update_mask(buf, t_reception, LNAV_VNAV_LP_LPV_approach);
          break;
        case IONO_DELAY_CORRECTION: // 26
          if(!sat.igp.register_igp(buf, t_reception, LNAV_VNAV_LP_LPV_approach)){
            message_type = UNSUPPORTED_MESSAGE;
          }
          break;
        case NULL_MESSAGES: // 63
          break;
        default:
          message_type = UNSUPPORTED_MESSAGE;
      }

      return message_type;
    }
};

template <class FloatT>
template <typename T>
typename SBAS_SpaceNode<FloatT>::KnownSatellites::list_t
    SBAS_SpaceNode<FloatT>::KnownSatellites::sort(T sorter){
  static const typename SBAS_SpaceNode<FloatT>::RangingCode codes[] = {
    {120,  145, 01106,    5,   "ASECNA (A-SBAS)"},
    {121,  175, 01241,    5,   "EGNOS (Eutelsat 5WB)"},
    {122,   52, 00267,  143.5, "SPAN (INMARSAT 4F1)"},
    {123,   21, 00232,   31.5, "EGNOS (ASTRA 5B)"},
    //{124,  237, 01617,    0,   "(Reserved)"},
    {125,  235, 01076,  -16,   "SDCM (Luch-5A)"},
    {126,  886, 01764,   63.9, "EGNOS (INMARSAT 4F2)"},
    {127,  657, 00717,   55,   "GAGAN (GSAT-8)"},
    {128,  634, 01532,   83,   "GAGAN (GSAT-10)"},
    {129,  762, 01250,  127,   "MSAS (QZS-3)"},
    {130,  355, 00341,  140,   "BDSBAS (G6)"},
    {131, 1012, 00551, -117,   "WAAS (Eutelsat 117West B)"},
    {132,  176, 00520,   93.5, "GAGAN (GSAT-15)"},
    {133,  603, 01731, -129,   "WAAS (SES-15)"},
    {134,  130, 00706,   91.5, "KASS (MEASAT-3D)"},
    {135,  359, 01216, -125,   "WAAS (Intelsat Galaxy 30)"},
    {136,  595, 00740,    5,   "EGNOS (HOTBIRD 13G)"},
    {137,   68, 01007,  127,   "MSAS (QZS-3)"},
    {138,  386, 00450,  107.3, "WAAS (ANIK F1R)"},
    //{139,  797, 00305,    0,   "MSAS (QZS-7)"},
    {140,  456, 01653,   95,   "SDCM (Luch-5V)"},
    {141,  499, 01411,  167,   "SDCM (Luch-5A)"},
    //{142,  883, 01644,    0,   "(Unallocated)"},
    {143,  307, 01312,  110.5, "BDSBAS (G3)"},
    {144,  127, 01060,   80,   "BDSBAS (G2)"},
    //{145,  211, 01560,    0,   "(Unallocated)"},
    //{146,  121, 00035,    0,   "(Unallocated)"},
    {147,  118, 00355,    5,   "ASECNA (A-SBAS)"},
    {148,  163, 00335,  -24.8, "ASAL (ALCOMSAT-1)"},
    //{149,  628, 01254,    0,   "(Unallocated)"},
    //{150,  853, 01041,    0,   "EGNOS"},
    //{151,  484, 00142,    0,   "(Unallocated)"},
    //{152,  289, 01641,    0,   "(Unallocated)"},
    //{153,  811, 01504,    0,   "(Unallocated)"},
    //{154,  202, 00751,    0,   "(Unallocated)"},
    //{155, 1021, 01774,    0,   "(Unallocated)"},
    //{156,  463, 00107,    0,   "(Unallocated)"},
    //{157,  568, 01153,    0,   "(Unallocated)"},
    //{158,  904, 01542,    0,   "(Unallocated)"},
  }; ///< @see https://www.gps.gov/technical/prn-codes/L1-CA-PRN-code-assignments-2021-Jun.pdf
  list_t res;
  res.reserve(sizeof(codes) / sizeof(codes[0]));
  for(unsigned int i(0); i < sizeof(codes) / sizeof(codes[0]); ++i){
    res.push_back(&codes[i]);
  }
  std::sort(res.begin(), res.end(), sorter);
  return res;
}

template <class FloatT>
const typename SBAS_SpaceNode<FloatT>::KnownSatellites::list_t
    SBAS_SpaceNode<FloatT>::KnownSatellites::prn_ordered
      = SBAS_SpaceNode<FloatT>::KnownSatellites::sort(
        typename SBAS_SpaceNode<FloatT>::RangingCode::prn_sorter_t());

template <class FloatT>
const typename SBAS_SpaceNode<FloatT>::KnownSatellites::list_t
    SBAS_SpaceNode<FloatT>::KnownSatellites::longitude_ordered
      = SBAS_SpaceNode<FloatT>::KnownSatellites::sort(
        typename SBAS_SpaceNode<FloatT>::RangingCode::lng_sorter_t());

template <class FloatT>
const typename SBAS_SpaceNode<FloatT>::KnownSatellites::list_t
    SBAS_SpaceNode<FloatT>::KnownSatellites::name_ordered
      = SBAS_SpaceNode<FloatT>::KnownSatellites::sort(
        typename SBAS_SpaceNode<FloatT>::RangingCode::name_sorter_t());


template <class FloatT>
const typename SBAS_SpaceNode<FloatT>::Timing::values_t
    SBAS_SpaceNode<FloatT>::Timing::values[SBAS_SpaceNode<FloatT>::Timing::NUM_OF_TIMING_ITEMS] = {
  {  6},                // DONT_USE_FOR_SAFETY_APPLICATIONS (0)
  {120,   600,   600},  // PRN_MASK (1)
  {  6,    18,    12},  // UDREI (2-6, 24)
  {},                   // FAST_CORRECTIONS (2-5, 24)
  {120,   360,   240},  // LONG_TERM_CORRECTIONS (24, 25)
  {120,   360,   240},  // GEO_NAVIGATION_DATA (9)
  {120,   360,   240},  // FAST_CORRECTION_DEGRADATION (7)
  {120,   360,   240},  // DEGRADATION_PARAMETERS (10)
  {300,  1200,  1200},  // IONOSPHERIC_GRID_MASK (18)
  {300,   600,   600},  // IONOSPHERIC_CORRECTIONS (26)
  {300, 86400, 86400},  // UTC_TIMING_DATA (12)
  {300},                // ALNAMAC_DATA (17)
  {300, 86400, 86400},  // SERVICE_LEVEL (27)
  {120,   360,   240},  // CLOCK_EPHEMERIS_COVARIANCE_MATRIX (28)
}; ///< @see Table A-25

template <class FloatT>
const typename SBAS_SpaceNode<FloatT>::IonosphericGridPoints::PointProperty::raw_t
    SBAS_SpaceNode<FloatT>::IonosphericGridPoints::PointProperty::raw_t::unavailable = {
  DELAY_DONT_USE,
  ERROR_INDICATOR_NOT_MONITORED,
};

template <class FloatT>
const typename SBAS_SpaceNode<FloatT>::IonosphericGridPoints::PointProperty
    SBAS_SpaceNode<FloatT>::IonosphericGridPoints::PointProperty::unavailable = {
  0,
  raw_t::raw2sigma2(raw_t::ERROR_INDICATOR_NOT_MONITORED),
};

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
const typename SBAS_SpaceNode<FloatT>::float_t SBAS_SpaceNode<FloatT>::DegradationFactors::raw_t::sf[] = {
  0.002,    // SF_B_rcc
  0.002,    // SF_C_ltc_lsb
  0.00005,  // SF_C_ltc_v1
  0.002,    // SF_C_ltc_v0
  0.0005,   // SF_C_geo_lsb
  0.00005,  // SF_C_geo_v
  0.5,      // SF_C_er
  0.001,    // SF_C_iono_step
  0.000005, // SF_C_iono_ramp
  0.1,      // SF_C_covariance
};

template <class FloatT>
const typename SBAS_SpaceNode<FloatT>::float_t SBAS_SpaceNode<FloatT>::SatelliteProperties::Ephemeris::raw_t::sf[] = {
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
const typename SBAS_SpaceNode<FloatT>::float_t SBAS_SpaceNode<FloatT>::SatelliteProperties::Almanac::raw_t::sf[] = {
  2600,   // SF_xy
  26000,  // SF_z
  10,     // SF_dxy
  60,     // SF_dz
  64,     // SF_t_0
};

template <class FloatT>
const typename SBAS_SpaceNode<FloatT>::float_t SBAS_SpaceNode<FloatT>::UTC_Parameters::raw_t::sf[] = {
  POWER_2(-50), // SF_A1
  POWER_2(-30), // SF_A0
};

#undef POWER_2

template <class FloatT>
const typename SBAS_SpaceNode<FloatT>::float_t SBAS_SpaceNode<FloatT>::SatelliteProperties::Ephemeris::URA_table[] = {
  2,
  2.8,
  4,
  5.7,
  8,
  11.3,
  16,
  32,
  64,
  128,
  256,
  512,
  1024,
  2048,
  4096,
};

#endif /* __SBAS_H__ */
