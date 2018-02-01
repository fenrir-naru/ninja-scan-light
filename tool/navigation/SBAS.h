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



    class Satellite {
      public:
        typedef typename gps_space_node_t::constellation_t constellation_t;

        /**
         * SBAS Ephemeris
         * @see Table A-18
         */
        struct Ephemeris {
          uint_t svid;            ///< Satellite number

          float_t t_0;            ///< Time of applicability (s)
          int_t URA;              ///< User range accuracy (index)
          float_t x, y, z;        ///< ECEF position (m)
          float_t dx, dy, dz;     ///< ECEF velocity (m/s)
          float_t ddx, ddy, ddz;  ///< ECEF acceleration (m/s^2)
          float_t a_f0;           ///< Clock correction parameter (s)
          float_t a_f1;           ///< Clock correction parameter (s/s)

          constellation_t constellation(
              const gps_time_t &t, const float_t &pseudo_range = 0,
              const bool &with_velocity = true) const {

            constellation_t res = { // TODO (A-44, A-45)
              xyz_t(x, y, z),
              xyz_t(dx, dy, dz),
            };

            return res;
          }

          struct raw_t {
            u8_t  svid;           ///< Satellite number

            u16_t t_0;            ///< Time of applicability (16, s)
            u8_t  URA;            ///< User range accuracy
            s32_t x, y, z;        ///< ECEF position (0.08(xy), 0.4(z) m)
            s32_t dx, dy, dz;     ///< ECEF velocity (0.000625(xy), 0.004(z) m)
            s16_t ddx, ddy, ddz;  ///< ECEF acceleration (0.0000125(xy), 0.0000625(z) m/s^2)
            s16_t a_f0;           ///< Clock correction parameter (2^-31, s)
            s16_t a_f1;           ///< Clock correction parameter (2^-40, s/s)

            enum {
              SF_t_0,
              SF_xy,    SF_z,
              SF_dxy,   SF_dz,
              SF_ddxy,  SF_ddz,
              SF_a_f0,
              SF_a_f1,

              SF_NUM,
            };
            static const float_t sf[SF_NUM];

            operator Ephemeris() const {
              Ephemeris converted;
#define CONVERT2(TARGET, TARGET_SF) \
{converted.TARGET = sf[SF_ ## TARGET_SF] * TARGET;}
#define CONVERT(TARGET) CONVERT2(TARGET, TARGET)
              converted.svid = svid;

              converted.URA = URA;
              CONVERT(t_0);
              CONVERT2(x, xy);      CONVERT2(y, xy);      CONVERT(z);
              CONVERT2(dx, dxy);    CONVERT2(dy, dxy);    CONVERT(dz);
              CONVERT2(ddx, ddxy);  CONVERT2(ddy, ddxy);  CONVERT(ddz);
              CONVERT(a_f0);
              CONVERT(a_f1);
#undef CONVERT
#undef CONVERT2
              return converted;
            }

            raw_t &operator=(const Ephemeris &eph){
#define CONVERT2(TARGET, TARGET_SF) \
{TARGET = (s32_t)((eph.TARGET + 0.5 * sf[SF_ ## TARGET_SF]) / sf[SF_ ## TARGET_SF]);}
#define CONVERT(TARGET) CONVERT2(TARGET, TARGET)
              svid = eph.svid;

              URA = eph.URA;
              CONVERT(t_0);
              CONVERT2(x, xy);      CONVERT2(y, xy);      CONVERT(z);
              CONVERT2(dx, dxy);    CONVERT2(dy, dxy);    CONVERT(dz);
              CONVERT2(ddx, ddxy);  CONVERT2(ddy, ddxy);  CONVERT(ddz);
              CONVERT(a_f0);
              CONVERT(a_f1);
#undef CONVERT
#undef CONVERT2
              return *this;
            }
          };

          bool is_equivalent(const Ephemeris &eph) const {
            do{
              if(URA != eph.URA){break;}

#define CHECK2(TARGET, TARGET_SF) \
if(std::abs(TARGET - eph.TARGET) > raw_t::sf[raw_t::SF_ ## TARGET_SF]){break;}
#define CHECK(TARGET) CHECK2(TARGET, TARGET)
              CHECK(t_0);
              CHECK2(x, xy);      CHECK2(y, xy);      CHECK(z);
              CHECK2(dx, dxy);    CHECK2(dy, dxy);    CHECK(dz);
              CHECK2(ddx, ddxy);  CHECK2(ddy, ddxy);  CHECK(ddz);
              CHECK(a_f0);
              CHECK(a_f1);
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
  POWER_2(-31), // SF_a_f0
  POWER_2(-40), // SF_a_f1
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
