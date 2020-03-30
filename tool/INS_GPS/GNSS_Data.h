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

#ifndef __GNSS_DATA_H__
#define __GNSS_DATA_H__

#ifndef IS_LITTLE_ENDIAN
#define IS_LITTLE_ENDIAN 1
#endif
#include "SylphideProcessor.h"

#include "navigation/GPS.h"

template <class FloatT>
struct GNSS_Data {

  struct Loader;
  mutable Loader *loader;

  typedef G_Packet_Observer<FloatT> observer_t;
  typedef typename observer_t::subframe_t subframe_t;
  subframe_t subframe;

  typedef GPS_Time<FloatT> gps_time_t;
  gps_time_t time_of_reception;

  struct Loader {
    typedef GPS_SpaceNode<FloatT> gps_t;
    gps_t *gps;

    struct gps_ephemeris_t : public gps_t::Satellite::Ephemeris {
      typename gps_t::uint_t how;
      typename gps_t::int_t iode2;
      typedef typename gps_t::Satellite::Ephemeris super_t;
      gps_ephemeris_t(){
        super_t::iodc = super_t::iode = iode2 = -1;
      }
    } gps_ephemeris[32];

    typedef typename gps_t::Ionospheric_UTC_Parameters gps_iono_utc_t;

    Loader() : gps(NULL) {
      for(unsigned int i(0); i < sizeof(gps_ephemeris) / sizeof(gps_ephemeris[0]); ++i){
        gps_ephemeris[i].svid = i + 1;
      }
    }

    static gps_iono_utc_t fetch_as_GPS_iono_utc(const subframe_t &subframe){

      typename gps_iono_utc_t::raw_t raw;
      typedef typename observer_t::s8_t s8_t;
      typedef typename observer_t::s32_t s32_t;
      typedef typename observer_t::u32_t u32_t;
#define get8(index) (subframe.bits2u8_align(index, 8))
      { // IONO parameter
        raw.alpha0 = (s8_t)get8( 68);
        raw.alpha1 = (s8_t)get8( 76);
        raw.alpha2 = (s8_t)get8( 90);
        raw.alpha3 = (s8_t)get8( 98);
        raw.beta0  = (s8_t)get8(106);
        raw.beta1  = (s8_t)get8(120);
        raw.beta2  = (s8_t)get8(128);
        raw.beta3  = (s8_t)get8(136);
      }
      { // UTC parameter
        {
          u32_t buf(get8(150));
          if(buf & 0x80){buf |= 0xFF00;}
          buf <<= 8; buf |= get8(158);
          buf <<= 8; buf |= get8(166);
          raw.A1 = (s32_t)buf;
        }
        {
          u32_t buf(get8(180));
          buf <<= 8; buf |= get8(188);
          buf <<= 8; buf |= get8(196);
          buf <<= 8; buf |= get8(210);
          raw.A0 = (s32_t)buf;
        }
        raw.t_ot = get8(218);
        raw.WN_t = get8(226); // truncated
        raw.delta_t_LS = (s8_t)get8(240);
        raw.WN_LSF = get8(248); // truncated
        raw.DN = get8(256);
        raw.delta_t_LSF = (s8_t)get8(270);
      }
#undef get8
      return (gps_iono_utc_t)raw;
    }

    static void fetch_as_GPS_subframe1(const subframe_t &in, gps_ephemeris_t &out){
      out.WN        = in.ephemeris_wn();
      out.URA       = in.ephemeris_ura();
      out.SV_health = in.ephemeris_sv_health();
      out.iodc      = in.ephemeris_iodc();
      out.t_GD      = in.ephemeris_t_gd();
      out.t_oc      = in.ephemeris_t_oc();
      out.a_f2      = in.ephemeris_a_f2();
      out.a_f1      = in.ephemeris_a_f1();
      out.a_f0      = in.ephemeris_a_f0();
      out.how       = in.how();
    }

    static void fetch_as_GPS_subframe2(const subframe_t &in, gps_ephemeris_t &out){
      out.iode    = in.ephemeris_iode_subframe2();
      out.c_rs    = in.ephemeris_c_rs();
      out.delta_n = in.ephemeris_delta_n();
      out.m0      = in.ephemeris_m_0();
      out.c_uc    = in.ephemeris_c_uc();
      out.e       = in.ephemeris_e();
      out.c_us    = in.ephemeris_c_us();
      out.sqrt_A  = in.ephemeris_root_a();
      out.t_oe    = in.ephemeris_t_oe();
      out.fit_interval
          = gps_ephemeris_t::super_t::raw_t::fit_interval(
              in.ephemeris_fit(), out.iodc);
    }

    static void fetch_as_GPS_subframe3(const subframe_t &in, gps_ephemeris_t &out){
      out.c_ic        = in.ephemeris_c_ic();
      out.Omega0      = in.ephemeris_omega_0();
      out.c_is        = in.ephemeris_c_is();
      out.i0          = in.ephemeris_i_0();
      out.c_rc        = in.ephemeris_c_rc();
      out.omega       = in.ephemeris_omega();
      out.dot_Omega0  = in.ephemeris_omega_0_dot();
      out.iode2       = in.ephemeris_iode_subframe3();
      out.dot_i0      = in.ephemeris_i_0_dot();
    }

    bool load(const gps_ephemeris_t &eph){
      gps->satellite(eph.svid).register_ephemeris(eph);
      return true;
    }

    bool load(const GNSS_Data &data){
      if(data.subframe.sv_number > 32){return false;}

      int week_number(data.time_of_reception.week);
      // If invalid week number, estimate it based on current time
      // This is acceptable because it will be used to compensate truncated upper significant bits.
      if(week_number < 0){week_number = gps_time_t::now().week;}

      if(data.subframe.subframe_no <= 3){
        gps_ephemeris_t &eph(gps_ephemeris[data.subframe.sv_number - 1]);
        switch(data.subframe.subframe_no){
          case 1: fetch_as_GPS_subframe1(data.subframe, eph); break;
          case 2: fetch_as_GPS_subframe2(data.subframe, eph); break;
          case 3: fetch_as_GPS_subframe3(data.subframe, eph); break;
        }
        if((eph.iodc >= 0) && (eph.iode >= 0) && (eph.iode2 >= 0)
            && (eph.iode == eph.iode2) && ((eph.iodc & 0xFF) == eph.iode)){
          // Original WN is truncated to 10 bits.
          eph.WN = (week_number - (week_number % 0x400)) + (eph.WN % 0x400);
          load(eph);
          eph.iodc = eph.iode = eph.iode2 = -1; // invalidate
          return true;
        }
      }else if((data.subframe.subframe_no == 4) && (data.subframe.sv_or_page_id == 56)){ // IONO UTC parameters
        gps_iono_utc_t iono_utc(fetch_as_GPS_iono_utc(data.subframe));
        // taking truncation into account
        int week_number_base(week_number - (week_number % 0x100));
        iono_utc.WN_t = week_number_base + (iono_utc.WN_t % 0x100);
        iono_utc.WN_LSF = week_number_base + (iono_utc.WN_LSF % 0x100);
        gps->update_iono_utc(iono_utc);
        return true;
      }

      return false;
    }
  };
};

#endif /* __GNSS_DATA_H__ */
