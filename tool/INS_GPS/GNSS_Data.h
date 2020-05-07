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

    typedef typename gps_t::Satellite::Ephemeris gps_ephemeris_t;
    struct gps_ephemeris_raw_t : public gps_t::Satellite::Ephemeris::raw_t {
      bool set_iodc;
      int iode_subframe2, iode_subframe3;
      typedef typename gps_t::Satellite::Ephemeris::raw_t super_t;
      gps_ephemeris_raw_t()
          : set_iodc(false), iode_subframe2(-1), iode_subframe3(-1) {}
    } gps_ephemeris[32];

    typedef typename gps_t::Ionospheric_UTC_Parameters gps_iono_utc_t;

    Loader() : gps(NULL) {
      for(unsigned int i(0); i < sizeof(gps_ephemeris) / sizeof(gps_ephemeris[0]); ++i){
        gps_ephemeris[i].svid = i + 1;
      }
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
    }

    static void fetch_as_GPS_subframe2(const subframe_t &in, gps_ephemeris_t &out){
      out.iode    = in.ephemeris_iode_subframe2();
      out.c_rs    = in.ephemeris_c_rs();
      out.delta_n = in.ephemeris_delta_n();
      out.M0      = in.ephemeris_m_0();
      out.c_uc    = in.ephemeris_c_uc();
      out.e       = in.ephemeris_e();
      out.c_us    = in.ephemeris_c_us();
      out.sqrt_A  = in.ephemeris_root_a();
      out.t_oe    = in.ephemeris_t_oe();
      out.fit_interval
          = gps_ephemeris_t::raw_t::fit_interval(in.ephemeris_fit(), out.iodc);
    }

    static void fetch_as_GPS_subframe3(const subframe_t &in, gps_ephemeris_t &out){
      out.c_ic        = in.ephemeris_c_ic();
      out.Omega0      = in.ephemeris_omega_0();
      out.c_is        = in.ephemeris_c_is();
      out.i0          = in.ephemeris_i_0();
      out.c_rc        = in.ephemeris_c_rc();
      out.omega       = in.ephemeris_omega();
      out.dot_Omega0  = in.ephemeris_omega_0_dot();
      out.dot_i0      = in.ephemeris_i_0_dot();
    }

    /**
     * Used for legacy interface such as RXM-EPH(0x0231), AID-EPH(0x0B31)
     */
    template <class Base>
    struct gps_ephemeris_extended_t : public Base {
      unsigned int &sv_number; // sv_number is alias.
      bool valid;
      typename gps_t::uint_t how;
      gps_ephemeris_extended_t()
          : Base(),
          sv_number(Base::svid), valid(false){}

      void fetch_as_subframe1(const subframe_t &buf){
        fetch_as_GPS_subframe1(buf, *this);
      }
      void fetch_as_subframe2(const subframe_t &buf){
        fetch_as_GPS_subframe2(buf, *this);
      }
      void fetch_as_subframe3(const subframe_t &buf){
        fetch_as_GPS_subframe3(buf, *this);
      }
    };

    bool load(const gps_ephemeris_t &eph){
      if(!gps){return false;}
      gps->satellite(eph.svid).register_ephemeris(eph);
      return true;
    }

    bool load(const GNSS_Data &data){
      if(data.subframe.sv_number > 32){return false;}

      int week_number(data.time_of_reception.week);
      // If invalid week number, estimate it based on current time
      // This is acceptable because it will be used to compensate truncated upper significant bits.
      if(week_number < 0){week_number = gps_time_t::now().week;}

      typename observer_t::u32_t *buf((typename observer_t::u32_t *)data.subframe.buffer);
      for(int i(0); i < sizeof(data.subframe.buffer); i += sizeof(typename observer_t::u32_t)){
        /* Move padding bits(=) in accordance with ICD
         * // 0x==_0xDD_0xDD_0xDD => 0b==DDDDDD_0xDD_0xDD_0bDD======
         * TODO This is based on old RXM-SFRB; format of new RXM-SFRBX is already aligned.
         */
        buf[i] <<= 6;
      }

      if(data.subframe.subframe_no <= 3){
        gps_ephemeris_raw_t &eph(gps_ephemeris[data.subframe.sv_number - 1]);

        switch(data.subframe.subframe_no){
          case 1: eph.template update_subframe1<2, 0>(buf); eph.set_iodc = true; break;
          case 2: eph.iode_subframe2 = eph.template update_subframe2<2, 0>(buf); break;
          case 3: eph.iode_subframe3 = eph.template update_subframe3<2, 0>(buf); break;
        }
        if(eph.set_iodc && (eph.iode_subframe2 >= 0) && (eph.iode_subframe3 >= 0)
            && (eph.iode_subframe2 == eph.iode_subframe3) && ((eph.iodc & 0xFF) == eph.iode)){
          // Original WN is truncated to 10 bits.
          eph.WN = (week_number - (week_number % 0x400)) + (eph.WN % 0x400);
          bool res(load((gps_ephemeris_t)eph));
          eph.set_iodc = false; eph.iode_subframe2 = eph.iode_subframe3 = -1; // invalidate
          return res;
        }
      }else if((data.subframe.subframe_no == 4) && (data.subframe.sv_or_page_id == 56)){ // IONO UTC parameters
        typename gps_iono_utc_t::raw_t raw;
        raw.template update<2, 0>(buf);
        gps_iono_utc_t iono_utc((gps_iono_utc_t)raw);

        // taking truncation into account
        int week_number_base(week_number - (week_number % 0x100));
        iono_utc.WN_t = week_number_base + (iono_utc.WN_t % 0x100);
        iono_utc.WN_LSF = week_number_base + (iono_utc.WN_LSF % 0x100);
        if(gps){
          gps->update_iono_utc(iono_utc);
          return true;
        }
      }

      return false;
    }
  };
};

#endif /* __GNSS_DATA_H__ */
