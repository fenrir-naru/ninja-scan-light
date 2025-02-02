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

#include <iostream>

#ifndef IS_LITTLE_ENDIAN
#define IS_LITTLE_ENDIAN 1
#endif
#include "SylphideProcessor.h"

#include "navigation/GPS.h"
#include "navigation/SBAS.h"
#include "navigation/QZSS.h"
#include "navigation/GLONASS.h"

template <class FloatT>
struct GNSS_Data {

  struct Loader;
  mutable Loader *loader;

  typedef G_Packet_Observer<FloatT> observer_t;
  struct subframe_t {
    unsigned int gnssID;
    unsigned int sv_number;
    unsigned int bytes;
    typename observer_t::v8_t buffer[40];
    int glonass_freq_ch;
  } subframe;

  typedef GPS_Time<FloatT> gps_time_t;
  gps_time_t time_of_reception;

  struct Loader {
    typedef GPS_SpaceNode<FloatT> gps_t;
    gps_t *gps;
    typedef SBAS_SpaceNode<FloatT> sbas_t;
    sbas_t *sbas;
    typedef GLONASS_SpaceNode<FloatT> glonass_t;
    glonass_t *glonass;

    typedef typename gps_t::Satellite::Ephemeris gps_ephemeris_t;
    struct gps_ephemeris_raw_t : public gps_ephemeris_t::raw_t {
      bool set_iodc;
      int iode_subframe2, iode_subframe3;
      bool is_qzss;
      typedef typename gps_ephemeris_t::raw_t super_t;
      gps_ephemeris_raw_t()
          : super_t(), set_iodc(false), iode_subframe2(-1), iode_subframe3(-1), is_qzss(false) {}
      operator gps_ephemeris_t() const {
        return is_qzss
            ? (gps_ephemeris_t)(
              reinterpret_cast<
                const typename QZSS_SpaceNode<FloatT>::SatelliteProperties::Ephemeris::raw_t &>(*this))
            : super_t::operator gps_ephemeris_t();
      }
    } gps_ephemeris[32], qzss_ephemeris[10];

    typedef typename gps_t::Ionospheric_UTC_Parameters gps_iono_utc_t;

    typedef typename glonass_t::Satellite::Ephemeris_with_GPS_Time glonass_ephemeris_t;
    struct glonass_ephemeris_raw_t : public glonass_ephemeris_t::raw_t {
      unsigned int super_frame;
      unsigned int has_string;
      glonass_ephemeris_raw_t() : has_string(0) {}
    } glonass_ephemeris[24];

    Loader() : gps(NULL), sbas(NULL), glonass(NULL) {
      for(unsigned int i(0); i < sizeof(gps_ephemeris) / sizeof(gps_ephemeris[0]); ++i){
        gps_ephemeris[i].svid = i + 1;
      }
      for(unsigned int i(0); i < sizeof(qzss_ephemeris) / sizeof(qzss_ephemeris[0]); ++i){
        qzss_ephemeris[i].svid = i + 193;
        qzss_ephemeris[i].is_qzss = true;
      }
      for(unsigned int i(0); i < sizeof(glonass_ephemeris) / sizeof(glonass_ephemeris[0]); ++i){
        glonass_ephemeris[i].svid = i + 1;
      }
    }

    /**
     * Used for legacy interface such as RXM-EPH(0x0231), AID-EPH(0x0B31)
     */
    template <class Base>
    struct gps_ephemeris_extended_t : public Base {
      bool valid;
      gps_ephemeris_extended_t()
          : Base(), valid(false){}

      void fetch(const observer_t &in){
        if(in.current_packet_size() < (8 + 104)){return;}
        typename gps_ephemeris_t::raw_t raw;
        typename observer_t::v8_t buf[40];
        in.inspect(buf, 4, 6);
        raw.svid = le_char4_2_num<typename observer_t::u32_t>(buf[0]);
        for(int i(0); i <= 2; ++i){
          in.inspect(&(buf[8]), 32, 6 + 8 + i * 32);
          switch(i){
            case 0: raw.template update_subframe1<8, -6>((typename observer_t::u32_t *)buf); break;
            case 1: raw.template update_subframe2<8, -6>((typename observer_t::u32_t *)buf); break;
            case 2: raw.template update_subframe3<8, -6>((typename observer_t::u32_t *)buf); break;
          }
        }
        *(gps_ephemeris_t *)this = raw;
        valid = true;
      }
    };

    bool load(const gps_ephemeris_t &eph){
      if(!gps){return false;}
      gps->satellite(eph.svid).register_ephemeris(eph);
      return true;
    }

    bool load_gps(const GNSS_Data &data){
      bool valid_time_of_reception(data.time_of_reception.week >= 0);
      int week_number(data.time_of_reception.week);
      // If invalid week number, estimate it based on current time
      // This is acceptable because it will be used to compensate truncated upper significant bits.
      if(!valid_time_of_reception){week_number = gps_time_t::now().week;}

      typename observer_t::u32_t *buf((typename observer_t::u32_t *)data.subframe.buffer);
      typedef typename gps_t::template BroadcastedMessage<typename observer_t::u32_t, 30> parser_t;
      int subframe_no(parser_t::subframe_id(buf));

      if(subframe_no <= 3){
        gps_ephemeris_raw_t &eph(
            (data.subframe.gnssID == observer_t::gnss_svid_t::QZSS)
            ? qzss_ephemeris[data.subframe.sv_number - 1]
            : gps_ephemeris[data.subframe.sv_number - 1]);

        switch(subframe_no){
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
      }else if((subframe_no == 4) && (parser_t::sv_page_id(buf) == 56)){ // IONO UTC parameters
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

    bool load_sbas(const GNSS_Data &data){
      bool valid_time_of_reception(data.time_of_reception.week >= 0);
      if(!valid_time_of_reception){return false;}

      switch(sbas->decode_message(
          (typename observer_t::u32_t *)data.subframe.buffer,
          data.subframe.sv_number, data.time_of_reception)){
        case sbas_t::IONO_DELAY_CORRECTION: // 26
          std::cerr
              << "Ionospheric delay grid map (S" << data.subframe.sv_number << "):" << std::endl
              << sbas->satellite(data.subframe.sv_number).ionospheric_grid_points() << std::endl;
          break;
        case sbas_t::UNSUPPORTED_MESSAGE:
          return false;
      }
      return true;
    }

    bool load(const glonass_ephemeris_t &eph){
      if(!glonass){return false;}
      glonass->satellite(eph.svid).register_ephemeris(eph);
      return true;
    }

    bool load_glonass(const GNSS_Data &data){
      if(data.subframe.bytes != 16){return false;}
      if((data.subframe.sv_number == 0)
          || (data.subframe.sv_number > (sizeof(glonass_ephemeris) / sizeof(glonass_ephemeris[0])))){
        return false; // exclude unknown satellite, whose sv_number may be 255.
      }
      typename observer_t::u32_t *buf((typename observer_t::u32_t *)data.subframe.buffer);
      typedef typename glonass_t::template BroadcastedMessage<typename observer_t::u32_t> parser_t;
      unsigned int super_frame(buf[3] >> 16), frame(buf[3] & 0xF), string_no(parser_t::m(buf));

      if((string_no >= 1) && (string_no <= 5)){ // immediate info. (ephemeris) with time property (in non-immediate info,)
        glonass_ephemeris_raw_t &eph(glonass_ephemeris[data.subframe.sv_number - 1]);
        if((eph.has_string > 0) && (eph.super_frame != super_frame)){
          eph.has_string = 0; // clean up
        }
        eph.super_frame = super_frame;
        eph.has_string |= (0x1 << (string_no - 1));
        switch(string_no){ // string num
          case 1: eph.template update_string1<0, 0>(buf); break;
          case 2: eph.template update_string2<0, 0>(buf); break;
          case 3: eph.template update_string3<0, 0>(buf); break;
          case 4: eph.template update_string4<0, 0>(buf); break;
          case 5: {
            eph.template update_string5<0, 0>(buf);
            if(frame == 4){
              // TODO: require special care for 50th frame? @see Table 4.9 note (4)
            }
            break;
          }
        }
        if(eph.has_string == 0x1F){ // get all ephemeris and time info. in the same super frame
          // Ephemeris_with_Time::raw_t =(cast)=> Ephemeris_with_Time => Ephemeris_with_GPS_Time
          glonass_ephemeris_t eph_converted(eph);
          eph_converted.freq_ch = data.subframe.glonass_freq_ch; // frequency channel
          eph_converted.t_b_gps += gps->is_valid_utc() // leap second correction
              ? gps->iono_utc().delta_t_LS
              : gps_time_t::guess_leap_seconds(eph_converted.t_b_gps);
          load(eph_converted); // register ephemeris
          eph.has_string = 0;
        }
        return true;
      }else{ // non-immediate info. except for time info. (almanac)

      }
      return false;
    }

    bool load(const GNSS_Data &data){
      switch(data.subframe.gnssID){
        // TODO: other satellite systems
        case observer_t::gnss_svid_t::SBAS:
          return load_sbas(data);
        case observer_t::gnss_svid_t::GLONASS:
          return load_glonass(data);
        case observer_t::gnss_svid_t::QZSS:
          if(data.subframe.bytes != sizeof(data.subframe.buffer)){return false;}
        case observer_t::gnss_svid_t::GPS:
          return load_gps(data);
      }
      return false;
    }
  };
};

#endif /* __GNSS_DATA_H__ */
