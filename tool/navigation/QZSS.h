/*
 * Copyright (c) 2021, M.Naruoka (fenrir)
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

#ifndef __QZSS_H__
#define __QZSS_H__

/** @file
 * @brief QZSS ICD definitions
 */

#include "GPS.h"
#include <limits>

template <class FloatT>
struct QZSS_SpaceNode {
  struct SatelliteProperties {
    struct Ephemeris {
      typedef typename GPS_SpaceNode<FloatT>::SatelliteProperties::Ephemeris eph_gps_t;
      struct raw_t : public eph_gps_t::raw_t {
        typedef typename eph_gps_t::raw_t super_t;

        operator eph_gps_t() const {
          eph_gps_t res(super_t::operator eph_gps_t());
          res.fit_interval = super_t::fit_interval_flag
              ? (std::numeric_limits<FloatT>::max)()
              : (2 * 60 * 60); // Fit interval (ICD:4.1.2.4 Subframe 2); should be zero
          return res;
        };

        raw_t &operator=(const eph_gps_t &eph){
          super_t::operator=(eph);
          super_t::fit_interval_flag = (eph.fit_interval > 2 * 60 * 60);
          return *this;
        }
      };
    };
    struct Almanac {
      typedef typename GPS_SpaceNode<FloatT>::SatelliteProperties::Almanac alm_gps_t;
      struct raw_t : public alm_gps_t::raw_t {
        typedef typename alm_gps_t::raw_t super_t;
        template <int PaddingBits_MSB, int PaddingBits_LSB, class InputT>
        void update(const InputT *src){
          super_t::template update<PaddingBits_MSB, PaddingBits_LSB, InputT>(src);
          if((super_t::svid >= 1) && (super_t::svid <= 10)){
            super_t::svid += 192;
          }
        }
        template <int PaddingBits_MSB, int PaddingBits_LSB, class BufferT>
        void dump(BufferT *dst, const typename GPS_SpaceNode<FloatT>::u8_t &sf = 4){
          super_t::template dump<PaddingBits_MSB, PaddingBits_LSB, BufferT>(dst);
          typedef typename GPS_SpaceNode<FloatT>::template BroadcastedMessage<
              BufferT, (int)sizeof(BufferT) * CHAR_BIT - PaddingBits_MSB - PaddingBits_LSB, PaddingBits_MSB>
              deparse_t;
          if((super_t::svid >= 193) && (super_t::svid <= 202)){
            deparse_t::subframe_id_set(dst, sf); // sf = 4 or 5
            deparse_t::sv_page_id_set(dst, super_t::svid - 192); // [1, 10]
          }else{
            return;
          }
          deparse_t::data_id_set(dst, 3); // always 3 @see 4.1.2.6.2. QZS Almanac
        }
      };
    };
  };
};

#endif /* __QZSS_H__ */
