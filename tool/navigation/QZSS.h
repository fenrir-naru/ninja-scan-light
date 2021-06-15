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
struct QZSS_LNAV_Ephemeris_Raw : public GPS_SpaceNode<FloatT>::Satellite::Ephemeris::raw_t {
  typedef typename GPS_SpaceNode<FloatT>::Satellite::Ephemeris eph_t;
  typedef typename eph_t::raw_t super_t;

  operator eph_t() const {
    eph_t res(super_t::operator eph_t());
    res.fit_interval = super_t::fit_interval_flag
        ? (std::numeric_limits<FloatT>::max)()
        : (2 * 60 * 60); // Fit interval (ICD:4.1.2.4 Subframe 2); should be zero
    return res;
  };

  QZSS_LNAV_Ephemeris_Raw &operator=(const eph_t &eph){
    super_t::operator=(eph);
    super_t::fit_interval_flag = (eph.fit_interval > 2 * 60 * 60);
    return *this;
  }
};

#endif /* __QZSS_H__ */
