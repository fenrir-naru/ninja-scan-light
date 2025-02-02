/*
 * Copyright (c) 2017, M.Naruoka (fenrir)
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
 * @brief RINEX Loader, support RINEX 2 and 3
 *
 */

#ifndef __RINEX_H__
#define __RINEX_H__

#include <istream>
#include <ostream>
#include <sstream>
#include <map>
#include <string>
#include <queue>
#include <vector>
#include <iomanip>
#include <ctime>
#include <exception>
#include <algorithm>
#include <cstddef>
#include <cstring>
#include <limits>

#include "util/text_helper.h"
#include "GPS.h"
#include "SBAS.h"
#include "GLONASS.h"

template <class U = void>
class RINEX_Reader {
  public:
    typedef RINEX_Reader<U> self_t;
    typedef std::map<std::string, std::vector<std::string> > header_t;

  protected:
    header_t _header;
    typename TextHelper<>::crlf_stream_t src;
    bool _has_next;

  public:
    struct version_type_t {
      int version;
      enum file_type_t {
        FTYPE_UNKNOWN = 0,
        FTYPE_OBSERVATION,
        FTYPE_NAVIGATION,
        FTYPE_METEOROLOGICAL,
        FTYPE_CLOCK,
      } file_type;
      enum sat_system_t {
        SYS_UNKNOWN = 0,
        SYS_GPS,
        SYS_GLONASS,
        SYS_GALILEO,
        SYS_QZSS,
        SYS_BDS,
        SYS_IRNSS,
        SYS_SBAS,
        SYS_TRANSIT,
        SYS_MIXED,
      } sat_system;
      static sat_system_t char2sys_v3(const char &c){
        switch(c){
          case 'G': return SYS_GPS;
          case 'R': return SYS_GLONASS;
          case 'E': return SYS_GALILEO;
          case 'J': return SYS_QZSS;
          case 'C': return SYS_BDS;
          case 'I': return SYS_IRNSS;
          case 'S': return SYS_SBAS;
          case 'M': return SYS_MIXED;
        }
        return SYS_UNKNOWN;
      }
      version_type_t(
          const int &ver = 0, const file_type_t &ftype = FTYPE_UNKNOWN, const sat_system_t &sys = SYS_UNKNOWN)
          : version(ver), file_type(ftype), sat_system(sys) {}
      void parse(const std::string &buf);
      void dump(std::string &buf) const;
    };
  protected:
    version_type_t version_type;

  public:
    RINEX_Reader(
        std::istream &in,
        std::string &(*modify_header)(std::string &, std::string &) = NULL)
        : src(in), _has_next(false),
        version_type() {
      if(src.fail()){return;}

      char buf[256];
      
      // Read header
      while(!src.eof()){
        src.getline(buf, sizeof(buf));
        
        std::string content(buf);
        std::string label(content, 60, 20);
        {
          int real_length(label.find_last_not_of(' ') + 1);
          if(real_length < (int)label.length()){
            label = label.substr(0, real_length);
          }
        }
        // std::cerr << label << " (" << label.length() << ")" << std::endl;
        
        if(label.find("END OF HEADER") == 0){break;}
        
        content = content.substr(0, 60);
        if(modify_header){content = modify_header(label, content);}
        _header[label].push_back(content);
      }

      // version and type extraction
      for(const header_t::const_iterator it(_header.find("RINEX VERSION / TYPE"));
          it != _header.end(); ){
        version_type.parse(it->second.front());
        break;
      }
    }
    RINEX_Reader(
        std::istream &in,
        void (RINEX_Reader::*read_header)())
        : src(in), _has_next(false),
        version_type() {
      (this->*read_header)();
    }
    virtual ~RINEX_Reader(){_header.clear();}
    header_t &header() {return _header;}
    const header_t &header() const {return const_cast<self_t *>(this)->header();}
    bool has_next() const {return _has_next;}

    template <class T, bool is_integer = std::numeric_limits<T>::is_integer>
    struct conv_t : public TextHelper<>::template format_t<T> {
      static bool e_dot_head(
          std::string &buf, const int &offset, const int &length, void *value, const int &precision = 0, const bool &str2val = true){
        if(str2val){
          std::string s(buf.substr(offset, length));
          std::string::size_type pos(s.find("D"));
          if(pos != std::string::npos){
            s.replace(pos, 1, "E");
          }
          std::stringstream ss(s);
          ss >> *(T *)value;
          return (ss.rdstate() & std::ios_base::failbit) == 0;
        }else{
          int w((std::max)(length, precision + 6)); // parentheses of std::max mitigates error C2589 under Windows VC

          std::stringstream ss;
          ss << std::setprecision(precision - 1) << std::scientific
              << ((*(T *)value) * 1E1);
          std::string s(ss.str());

          // -1.2345E+06 => -.12345E+06
          int index(s[0] == '-' ? 1 : 0);
          s[index + 1] = s[index + 0];
          s[index + 0] = '.';

          // -.12345E+06 => -.12345D+06
          s[index + precision + 1] = 'D';
          // If exponent portion digits are more than 2, then truncate it.
          s.erase(index + precision + 3, s.size() - (index + precision + 5));

          ss.str("");
          ss << std::setfill(' ') << std::right << std::setw(w) << s;
          buf.replace(offset, length, ss.str());
          return true;
        }
      }
    };

    template <class T>
    struct conv_t<T, true> : public TextHelper<>::template format_t<T> {
      static bool e_dot_head(
          std::string &buf, const int &offset, const int &length, void *value, const int &precision = 0, const bool &str2val = true){
        double v(*(T *)value);
        bool res(
            conv_t<double, false>::e_dot_head(buf, offset, length, &v, precision, str2val));
        *(T *)value = static_cast<T>(v);
        return res;
      }
    };

    typedef typename TextHelper<>::convert_item_t convert_item_t;
    
    static inline bool convert(
        const convert_item_t *items, const int &size, const std::string &buf, void *values,
        bool (*recovery)(const int &, const std::string &, void *) = NULL){
      return TextHelper<>::str2val(items, size, buf, values, recovery);
    }
    template <int N>
    static inline bool convert(
        const convert_item_t (&items)[N], const std::string &buf, void *values,
        bool (*recovery)(const int &, const std::string &, void *) = NULL){
      return convert(items, N, buf, values, recovery);
    }
};

template <class U>
void RINEX_Reader<U>::version_type_t::parse(const std::string &buf){
  int temp;
  conv_t<int>::d(const_cast<std::string &>(buf), 0, 6, &temp);
  version = temp * 100;
  conv_t<int>::d(const_cast<std::string &>(buf), 7, 2, &temp);
  version += temp;
  switch(version / 100){
    case 2:
      switch(buf[20]){
        case 'O':
          file_type = FTYPE_OBSERVATION;
          switch(buf[40]){
            case ' ':
            case 'G': sat_system = SYS_GPS;     break;
            case 'R': sat_system = SYS_GLONASS; break;
            case 'S': sat_system = SYS_SBAS;    break;
            case 'M': sat_system = SYS_MIXED;   break;
            case 'T': sat_system = SYS_TRANSIT; break;
          }
          break;
        case 'M': file_type = FTYPE_METEOROLOGICAL; break;
        case 'N': file_type = FTYPE_NAVIGATION; sat_system = SYS_GPS;     break;
        case 'G': file_type = FTYPE_NAVIGATION; sat_system = SYS_GLONASS; break;
        case 'H': file_type = FTYPE_NAVIGATION;
          sat_system = SYS_SBAS; // TODO: Is geostational as SBAS?
          break;
        case 'C': file_type = FTYPE_CLOCK; break;
      }
      break;
    case 3:
      switch(buf[20]){
        case 'O': file_type = FTYPE_OBSERVATION;    break;
        case 'N': file_type = FTYPE_NAVIGATION;     break;
        case 'M': file_type = FTYPE_METEOROLOGICAL; break;
        case 'C': file_type = FTYPE_CLOCK;          break;
      }
      if((file_type == FTYPE_OBSERVATION)
          || (file_type == FTYPE_NAVIGATION)
          || (file_type == FTYPE_CLOCK)){
        sat_system = char2sys_v3(buf[40]);
      }
      break;
  }
}
template <class U>
void RINEX_Reader<U>::version_type_t::dump(std::string &buf) const {
  int temp;
  conv_t<int>::d(buf, 0, 6, &(temp = version / 100), 0, false);
  if(version % 100 != 0){
    buf[6] = '.';
    conv_t<int>::d(buf, 7, 2, &(temp = version % 100), 1, false);
  }
  const char *type_str(NULL), *sys_str(NULL);
  switch(version / 100){
    case 2: {
      switch(file_type){
        case FTYPE_OBSERVATION: {
          type_str = "OBSERVATION DATA";
          switch(sat_system){
            case SYS_GPS:     sys_str = "G: GPS";     break;
            case SYS_GLONASS: sys_str = "R: GLONASS"; break;
            case SYS_SBAS:    sys_str = "S: Geostat"; break;
            case SYS_MIXED:   sys_str = "M: MIXED";   break;
            case SYS_TRANSIT: sys_str = "T: TRANSIT"; break;
            default: break;
          }
          break;
        }
        case FTYPE_NAVIGATION:
          switch(sat_system){
            case SYS_GPS:     type_str = "N: GPS NAV DATA";     break;
            case SYS_GLONASS: type_str = "G: GLONASS NAV DATA"; break;
            case SYS_SBAS:    type_str = "H: GEO NAV MSG DATA"; break;
            default: break;
          }
          break;
        case FTYPE_METEOROLOGICAL: type_str = "METEOROLOGICAL DATA"; break;
        default: break;
      }
      break;
    }
    case 3: {
      switch(file_type){
        case FTYPE_OBSERVATION:    type_str = "OBSERVATION DATA";    break;
        case FTYPE_NAVIGATION:     type_str = "N: GNSS NAV DATA";    break;
        case FTYPE_METEOROLOGICAL: type_str = "METEOROLOGICAL DATA"; break;
        default: break;
      }
      if((file_type != FTYPE_OBSERVATION) && (file_type != FTYPE_NAVIGATION)){break;}
      switch(sat_system){
        case SYS_GPS:     sys_str = "G: GPS";     break;
        case SYS_GLONASS: sys_str = "R: GLONASS"; break;
        case SYS_GALILEO: sys_str = "E: GALILEO"; break;
        case SYS_QZSS:    sys_str = "Q: QZSS";    break;
        case SYS_BDS:     sys_str = "B: BDS";     break;
        case SYS_IRNSS:   sys_str = "I: IRNSS";   break;
        case SYS_SBAS:    sys_str = "S: SBAS";    break;
        case SYS_MIXED:   sys_str = "M: MIXED";   break;
        default: break;
      }
      break;
    }
  }
  if(type_str){
    int n(std::strlen(type_str));
    buf.replace(20, n, type_str, n);
  }
  if(sys_str){
    int n(std::strlen(sys_str));
    buf.replace(40, n, sys_str, n);
  }
}


template <class FloatT>
struct RINEX_NAV {
  typedef GPS_SpaceNode<FloatT> space_node_t;
  typedef typename space_node_t::Satellite::Ephemeris ephemeris_t;
  struct message_t {
    ephemeris_t eph;
    std::tm t_oc_tm;
    int t_oc_year4, t_oc_year2, t_oc_mon12;
    FloatT t_oc_sec;
    FloatT t_oe_WN;
    FloatT t_ot;  ///< Transmitting time [s]
    FloatT fit_interval_hr;
    FloatT dummy;

    message_t() {}
    message_t(const ephemeris_t &eph_)
        : eph(eph_),
        t_oc_tm(typename space_node_t::gps_time_t(eph.WN, eph.t_oc).c_tm()),
        t_oc_year4(t_oc_tm.tm_year + 1900),
        t_oc_year2(t_oc_tm.tm_year % 100),
        t_oc_mon12(t_oc_tm.tm_mon + 1),
        t_oc_sec(std::fmod(eph.t_oc, 60)),
        t_oe_WN(eph.WN),
        t_ot(0), // TODO
        fit_interval_hr(eph.fit_interval / (60 * 60)),
        dummy(0) {
    }
    void update() {
      typename space_node_t::gps_time_t t_oc(t_oc_tm);
      t_oc += (t_oc_sec - t_oc_tm.tm_sec);
      eph.WN = t_oc.week;
      eph.t_oc = t_oc.seconds;

      /* @see ftp://igs.org/pub/data/format/rinex210.txt
       * 6.7 Satellite Health
       * RINEX Value:   0    Health OK
       * RINEX Value:   1    Health not OK (bits 18-22 not stored)
       * RINEX Value: >32    Health not OK (bits 18-22 stored)
       * eph.SV_health may have a value greater than 32
       */

      // At least 4 hour validity, then, hours => seconds;
      eph.fit_interval = ((fit_interval_hr < 4) ? 4 : fit_interval_hr) * 60 * 60;
    }
    static message_t from_qzss(const ephemeris_t &eph_){
      message_t res(eph_);
      res.eph.svid -= 192;
      res.fit_interval_hr = (res.fit_interval_hr) > 2 ? 1 : 0;
      return res;
    }
    ephemeris_t eph_qzss() const {
      ephemeris_t res(eph);
      res.svid += 192;
      res.fit_interval = ((fit_interval_hr > 0) ? 4 : 2) * 60 * 60;
      return res;
    }
  };
  struct message_sbas_t {
    typedef typename SBAS_SpaceNode<FloatT>
        ::SatelliteProperties::Ephemeris eph_t;
    int svid;
    std::tm date_tm;
    int t_year4, t_year2, t_mon12;
    FloatT t_sec;
    FloatT a_Gf0, a_Gf1;
    FloatT t_t; // Transmission time of message (start of the message) in GPS seconds of the week
    FloatT x_km, dx_km_s, ddx_km_s2;
    FloatT y_km, dy_km_s, ddy_km_s2;
    FloatT z_km, dz_km_s, ddz_km_s2;
    unsigned int health;
    FloatT URA;
    unsigned int iodn;
    message_sbas_t() {}
    message_sbas_t(const eph_t &eph)
        : svid((int)eph.svid - 100),
        date_tm(eph.base_time().c_tm()),
        t_year4(date_tm.tm_year + 1900),
        t_year2(date_tm.tm_year % 100),
        t_mon12(date_tm.tm_mon + 1),
        t_sec(date_tm.tm_sec),
        a_Gf0(eph.a_Gf0), a_Gf1(eph.a_Gf1),
        t_t(eph.t_0), // TODO maybe differ from t_e slightly
        x_km(1E-3 * eph.x), dx_km_s(1E-3 * eph.dx), ddx_km_s2(1E-3 * eph.ddx),
        y_km(1E-3 * eph.y), dy_km_s(1E-3 * eph.dy), ddy_km_s2(1E-3 * eph.ddy),
        z_km(1E-3 * eph.z), dz_km_s(1E-3 * eph.dz), ddz_km_s2(1E-3 * eph.ddz),
        health(0), URA(eph.URA), iodn(0) {
    }
    operator eph_t() const {
      eph_t eph = {0};
      eph.svid = (unsigned int)svid + 100;
      typename space_node_t::gps_time_t t(date_tm);
      t += (t_sec - date_tm.tm_sec);
      eph.WN = t.week;
      eph.t_0 = t.seconds;
      eph.a_Gf0 = a_Gf0; eph.a_Gf1 = a_Gf1;
      eph.x = 1E3 * x_km; eph.dx = 1E3 * dx_km_s; eph.ddx = 1E3 * ddx_km_s2;
      eph.y = 1E3 * y_km; eph.dy = 1E3 * dy_km_s; eph.ddy = 1E3 * ddy_km_s2;
      eph.z = 1E3 * z_km; eph.dz = 1E3 * dz_km_s; eph.ddz = 1E3 * ddz_km_s2;
      eph.URA = URA;
      return eph;
    }
  };
  struct message_glonass_t {
    typedef typename GLONASS_SpaceNode<FloatT>
        ::SatelliteProperties::Ephemeris_with_Time eph_t;
    int svid;
    std::tm date_tm;
    int t_year4, t_year2, t_mon12;
    FloatT t_sec;
    FloatT tau_n_neg, gamma_n;
    unsigned int t_k;
    FloatT x_km, dx_km_s, ddx_km_s2;
    FloatT y_km, dy_km_s, ddy_km_s2;
    FloatT z_km, dz_km_s, ddz_km_s2;
    unsigned int B_n, E_n;
    int freq_num; // 1-24(ver.2), -7-13(ver.3)

    // since ver.3.05
    unsigned int status_flags, urai, health_flags;
    FloatT delta_tau;

    message_glonass_t(){
      status_flags
          = ((0x01 & 0x3) << 7) // GLONASS-M
            | ((0x3) << 2); // upload/validity interval = 60 min
      urai = 15; // unknown
      health_flags = 0;
      delta_tau = 0;
    }
    message_glonass_t(const eph_t &eph)
        : svid((int)eph.svid),
        date_tm(eph.c_tm_utc()),
        t_year4(date_tm.tm_year + 1900),
        t_year2(date_tm.tm_year % 100),
        t_mon12(date_tm.tm_mon + 1),
        t_sec(date_tm.tm_sec),
        tau_n_neg(-eph.tau_n), gamma_n(eph.gamma_n), t_k(eph.t_k),
        x_km(1E-3 * eph.xn), dx_km_s(1E-3 * eph.xn_dot), ddx_km_s2(1E-3 * eph.xn_ddot),
        y_km(1E-3 * eph.yn), dy_km_s(1E-3 * eph.yn_dot), ddy_km_s2(1E-3 * eph.yn_ddot),
        z_km(1E-3 * eph.zn), dz_km_s(1E-3 * eph.zn_dot), ddz_km_s2(1E-3 * eph.zn_ddot),
        B_n(eph.B_n), E_n(eph.E_n),
        freq_num(eph.freq_ch),
        urai(eph.F_T_index()), delta_tau(eph.delta_tau_n) {
      status_flags
          = ((eph.M & 0x3) << 7)
            | (eph.P4 ? 0x40 : 0)
            | (eph.P2 ? 0x10 : 0)
            | ((eph.P1_index() & 0x3) << 2);
      health_flags = 0;
    }
    operator eph_t() const {
      typename GLONASS_SpaceNode<FloatT>::SatelliteProperties::Ephemeris eph = {0};
      eph.svid = (unsigned int)svid;
      eph.freq_ch = freq_num;
      eph.tau_n = -tau_n_neg; eph.gamma_n = gamma_n; eph.t_k = t_k;
      eph.xn = 1E3 * x_km; eph.xn_dot = 1E3 * dx_km_s; eph.xn_ddot = 1E3 * ddx_km_s2;
      eph.yn = 1E3 * y_km; eph.yn_dot = 1E3 * dy_km_s; eph.yn_ddot = 1E3 * ddy_km_s2;
      eph.zn = 1E3 * z_km; eph.zn_dot = 1E3 * dz_km_s; eph.zn_ddot = 1E3 * ddz_km_s2;
      eph.B_n = B_n; eph.E_n = E_n;
      eph.F_T = eph_t::Ephemeris::raw_t::F_T_value(urai);
      eph.delta_tau_n = (delta_tau > 0.999999999998E+09 ? 0 : delta_tau);
      eph.M = ((status_flags >> 7) & 0x3);
      eph.P4 = (status_flags & 0x40);
      eph.P2 = (status_flags & 0x10);
      eph.P1 = eph_t::Ephemeris::raw_t::P1_value((status_flags >> 2) & 0x03);
      return eph_t(eph, date_tm);
    }
  };
};

template <class FloatT = double>
class RINEX_NAV_Reader : public RINEX_Reader<> {
  protected:
    typedef RINEX_NAV_Reader<FloatT> self_t;
    typedef RINEX_Reader<> super_t;
  public:
    typedef typename RINEX_NAV<FloatT>::space_node_t space_node_t;
    typedef typename RINEX_NAV<FloatT>::message_t message_t;
    typedef typename RINEX_NAV<FloatT>::message_sbas_t message_sbas_t;
    typedef typename RINEX_NAV<FloatT>::message_glonass_t message_glonass_t;
    typedef typename space_node_t::Ionospheric_UTC_Parameters iono_utc_t;

    static const typename super_t::convert_item_t eph0_v2[10], eph0_v3[10];
    static const typename super_t::convert_item_t eph1_v2[4], eph1_v3[4];
    static const typename super_t::convert_item_t eph2_v2[4], eph2_v3[4];
    static const typename super_t::convert_item_t eph3_v2[4], eph3_v3[4];
    static const typename super_t::convert_item_t eph4_v2[4], eph4_v3[4];
    static const typename super_t::convert_item_t eph5_v2[4], eph5_v3[4];
    static const typename super_t::convert_item_t eph6_v2[4], eph6_v3[4];
    static const typename super_t::convert_item_t eph7_v2[2], eph7_v3[2];

    static const typename super_t::convert_item_t eph0_sbas_v2[10], eph0_sbas_v3[10];
    static const typename super_t::convert_item_t eph1_sbas_v2[4], eph1_sbas_v3[4];
    static const typename super_t::convert_item_t eph2_sbas_v2[4], eph2_sbas_v3[4];
    static const typename super_t::convert_item_t eph3_sbas_v2[4], eph3_sbas_v3[4];

    static const typename super_t::convert_item_t eph0_glonass_v2[10], eph0_glonass_v3[10];
    static const typename super_t::convert_item_t eph1_glonass_v2[4], eph1_glonass_v3[4];
    static const typename super_t::convert_item_t eph2_glonass_v2[4], eph2_glonass_v3[4];
    static const typename super_t::convert_item_t eph3_glonass_v2[4], eph3_glonass_v3[4];
    static const typename super_t::convert_item_t eph4_glonass_v305[4];

  protected:
    typename super_t::version_type_t::sat_system_t sys_of_msg;
    message_t msg;
    message_sbas_t msg_sbas;
    message_glonass_t msg_glonass;

    void seek_next_v2_gps() {
      char buf[256];

      for(int i(0); i < 8; i++){
        if((!super_t::src.good())
            || super_t::src.getline(buf, sizeof(buf)).fail()){return;}
        std::string line(buf);

        switch(i){
          case 0: {
            super_t::convert(eph0_v2, line, &msg);
            msg.t_oc_tm.tm_year = msg.t_oc_year2 + (msg.t_oc_year2 < 80 ? 100 : 0); // greater than 1980
            msg.t_oc_tm.tm_mon = msg.t_oc_mon12 - 1; // month [0, 11]
            msg.t_oc_tm.tm_sec = (int)msg.t_oc_sec;
            break;
          }
          case 1: super_t::convert(eph1_v2, line, &msg); break;
          case 2: super_t::convert(eph2_v2, line, &msg); break;
          case 3: super_t::convert(eph3_v2, line, &msg); break;
          case 4: super_t::convert(eph4_v2, line, &msg); break;
          case 5: super_t::convert(eph5_v2, line, &msg); break;
          case 6: super_t::convert(eph6_v2, line, &msg); break;
          case 7: super_t::convert(eph7_v2, line, &msg); break;
        }
      }
      msg.update();
      sys_of_msg = super_t::version_type_t::SYS_GPS;
      super_t::_has_next = true;
    }

    void seek_next_v2_glonass() {
      char buf[256];

      for(int i(0); i < 4; i++){
        if((!super_t::src.good())
            || super_t::src.getline(buf, sizeof(buf)).fail()){return;}
        std::string line(buf);

        switch(i){
          case 0: {
            super_t::convert(eph0_glonass_v2, line, &msg_glonass);
            msg_glonass.date_tm.tm_year = msg_glonass.t_year2 + (msg_glonass.t_year2 < 80 ? 100 : 0); // greater than 1980
            msg_glonass.date_tm.tm_mon = msg_glonass.t_mon12 - 1; // month [0, 11]
            msg_glonass.date_tm.tm_sec = (int)msg_glonass.t_sec;
            break;
          }
          case 1: super_t::convert(eph1_glonass_v2, line, &msg_glonass); break;
          case 2:
            super_t::convert(eph2_glonass_v2, line, &msg_glonass);
            if(super_t::version_type.version < 211){
              //msg_glonass.freq_num; // TODO 1..24? convert to value ranging from -7 to 6?
            }
            break;
          case 3: super_t::convert(eph3_glonass_v2, line, &msg_glonass); break;
        }
      }
      sys_of_msg = super_t::version_type_t::SYS_GLONASS;
      super_t::_has_next = true;
    }

    void seek_next_v2_sbas() {
      char buf[256];

      for(int i(0); i < 4; i++){
        if((!super_t::src.good())
            || super_t::src.getline(buf, sizeof(buf)).fail()){return;}
        std::string line(buf);

        switch(i){
          case 0: {
            super_t::convert(eph0_sbas_v2, line, &msg_sbas);
            msg_sbas.date_tm.tm_year = msg_sbas.t_year2 + (msg_sbas.t_year2 < 80 ? 100 : 0); // greater than 1980
            msg_sbas.date_tm.tm_mon = msg_sbas.t_mon12 - 1; // month [0, 11]
            msg_sbas.date_tm.tm_sec = (int)msg_sbas.t_sec;
            break;
          }
          case 1: super_t::convert(eph1_sbas_v2, line, &msg_sbas); break;
          case 2: super_t::convert(eph2_sbas_v2, line, &msg_sbas); break;
          case 3: super_t::convert(eph3_sbas_v2, line, &msg_sbas); break;
        }
      }
      sys_of_msg = super_t::version_type_t::SYS_SBAS;
      super_t::_has_next = true;
    }

    void seek_next_v2() {
      switch(super_t::version_type.sat_system){
        case super_t::version_type_t::SYS_GPS: seek_next_v2_gps(); return;
        case super_t::version_type_t::SYS_GLONASS: seek_next_v2_glonass(); return;
        case super_t::version_type_t::SYS_SBAS: seek_next_v2_sbas(); return;
        default: break;
      }
    }

    template <std::size_t N>
    void seek_next_v3_gps(char (&buf)[N]) {
      super_t::convert(eph0_v3, std::string(buf), &msg);
      msg.t_oc_tm.tm_year = msg.t_oc_year4 - 1900; // tm_year base is 1900
      msg.t_oc_tm.tm_mon = msg.t_oc_mon12 - 1; // month [0, 11]
      msg.t_oc_sec = msg.t_oc_tm.tm_sec;

      for(int i(1); i < 8; i++){
        if((!super_t::src.good())
            || super_t::src.getline(buf, sizeof(buf)).fail()){return;}
        std::string line(buf);

        switch(i){
          case 1: super_t::convert(eph1_v3, line, &msg); break;
          case 2: super_t::convert(eph2_v3, line, &msg); break;
          case 3: super_t::convert(eph3_v3, line, &msg); break;
          case 4: super_t::convert(eph4_v3, line, &msg); break;
          case 5: super_t::convert(eph5_v3, line, &msg); break;
          case 6: super_t::convert(eph6_v3, line, &msg); break;
          case 7: super_t::convert(eph7_v3, line, &msg); break;
        }
      }
      msg.update();
      sys_of_msg = super_t::version_type_t::SYS_GPS;
      super_t::_has_next = true;
    }

    template <std::size_t N>
    void seek_next_v3_sbas(char (&buf)[N]) {
      super_t::convert(eph0_sbas_v3, std::string(buf), &msg_sbas);
      msg_sbas.date_tm.tm_year = msg_sbas.t_year4 - 1900; // tm_year base is 1900
      msg_sbas.date_tm.tm_mon = msg_sbas.t_mon12 - 1; // month [0, 11]
      msg_sbas.t_sec = msg_sbas.date_tm.tm_sec;

      for(int i(1); i < 4; i++){
        if((!super_t::src.good())
            || super_t::src.getline(buf, sizeof(buf)).fail()){return;}
        std::string line(buf);

        switch(i){
          case 1: super_t::convert(eph1_sbas_v3, line, &msg_sbas); break;
          case 2: super_t::convert(eph2_sbas_v3, line, &msg_sbas); break;
          case 3: super_t::convert(eph3_sbas_v3, line, &msg_sbas); break;
        }
      }
      sys_of_msg = super_t::version_type_t::SYS_SBAS;
      super_t::_has_next = true;
    }

    template <std::size_t N>
    void seek_next_v3_qzss(char (&buf)[N]) {
      seek_next_v3_gps(buf);
      if(!super_t::_has_next){return;}
      sys_of_msg = super_t::version_type_t::SYS_QZSS;
    }

    template <std::size_t N>
    void seek_next_v3_glonass(char (&buf)[N]) {
      super_t::convert(eph0_glonass_v3, std::string(buf), &msg_glonass);
      msg_glonass.date_tm.tm_year = msg_glonass.t_year4 - 1900; // tm_year base is 1900
      msg_glonass.date_tm.tm_mon = msg_glonass.t_mon12 - 1; // month [0, 11]
      msg_glonass.t_sec = msg_glonass.date_tm.tm_sec;

      for(int i(1);
          i < ((super_t::version_type.version <= 304) ? 4 : 5);
          i++){
        if((!super_t::src.good())
            || super_t::src.getline(buf, sizeof(buf)).fail()){return;}
        std::string line(buf);

        switch(i){
          case 1: super_t::convert(eph1_glonass_v3, line, &msg_glonass); break;
          case 2: super_t::convert(eph2_glonass_v3, line, &msg_glonass); break;
          case 3: super_t::convert(eph3_glonass_v3, line, &msg_glonass); break;
          case 4: super_t::convert(eph4_glonass_v305, line, &msg_glonass); break;
        }
      }
      sys_of_msg = super_t::version_type_t::SYS_GLONASS;
      super_t::_has_next = true;
    }

    template <std::size_t N>
    void seek_next_v3_not_implemented(char (&buf)[N], const int &lines) {
      for(int i(1); i < lines; i++){
        if((!super_t::src.good())
            || super_t::src.getline(buf, sizeof(buf)).fail()){return;}
      }
      sys_of_msg = super_t::version_type_t::SYS_UNKNOWN;
      super_t::_has_next = true;
    }

    void seek_next_v3() {
      char buf[256];

      while(super_t::src.good()
          && (!super_t::src.getline(buf, sizeof(buf)).fail())){

        switch(buf[0]){
          case 'G': seek_next_v3_gps(buf); return; // GPS
          case 'E': seek_next_v3_not_implemented(buf, 8); return; // Galileo
          case 'R': seek_next_v3_glonass(buf); return; // Glonass
          case 'J': seek_next_v3_qzss(buf); return; // QZSS
          case 'C': seek_next_v3_not_implemented(buf, 8); return; // Beido
          case 'S': seek_next_v3_sbas(buf); return; // SBAS
          case 'T': seek_next_v3_not_implemented(buf, 8); return; // IRNSS
          default: break;
        }
      }
    }

    void seek_next() {
      super_t::_has_next = false;
      sys_of_msg = super_t::version_type_t::SYS_UNKNOWN;
      super_t::version_type.version >= 300 ? seek_next_v3() : seek_next_v2();
    }

  public:
    RINEX_NAV_Reader(std::istream &in) : super_t(in) {
      seek_next();
    }
    ~RINEX_NAV_Reader(){}
    
    void next() {
      seek_next();
    }
    
    static const typename super_t::convert_item_t iono_alpha_v2[4];
    static const typename super_t::convert_item_t iono_beta_v2[4];
    static const typename super_t::convert_item_t utc_v2[4];
    static const typename super_t::convert_item_t utc_leap_v2[1];

    /**
     * Obtain ionospheric delay coefficient and UTC parameters.
     * 
     * @return true when successfully obtained, otherwise false
     */
    bool extract_iono_utc_v2(GPS_SpaceNode<FloatT> &space_node) const {
      iono_utc_t iono_utc;
      bool alpha, beta, utc, leap;
      super_t::header_t::const_iterator it;

      if((alpha = ((it = _header.find("ION ALPHA")) != _header.end()))){
        super_t::convert(iono_alpha_v2, it->second.front(), &iono_utc);
      }
      
      if((beta = ((it = _header.find("ION BETA")) != _header.end()))){
        super_t::convert(iono_beta_v2, it->second.front(), &iono_utc);
      }

      if((utc = ((it = _header.find("DELTA-UTC: A0,A1,T,W")) != _header.end()))){
        super_t::convert(utc_v2, it->second.front(), &iono_utc);
      }

      if((leap = ((it = _header.find("LEAP SECONDS")) != _header.end()))){
        super_t::convert(utc_leap_v2, it->second.front(), &iono_utc);
      }

      space_node.update_iono_utc(iono_utc, alpha && beta, utc && leap);
      return alpha && beta && utc && leap;
    }

    static const typename super_t::convert_item_t iono_alpha_v3[4];
    static const typename super_t::convert_item_t iono_beta_v3[4];
    static const typename super_t::convert_item_t utc_v3[4];
    static const typename super_t::convert_item_t utc_leap_v301[4];

    bool extract_iono_utc_v3(GPS_SpaceNode<FloatT> &space_node) const {
      iono_utc_t iono_utc;
      bool alpha(false), beta(false), utc(false), leap(false);
      typedef super_t::header_t::const_iterator it_t;
      typedef super_t::header_t::mapped_type::const_iterator it2_t;

      it_t it;

      if((it = _header.find("IONOSPHERIC CORR")) != _header.end()){
        for(it2_t it2(it->second.begin()), it2_end(it->second.end()); it2 != it2_end; ++it2){
          if(it2->find("GPSA") != it2->npos){
            super_t::convert(iono_alpha_v3, *it2, &iono_utc);
            alpha = true;
          }else if(it2->find("GPSB") != it2->npos){
            super_t::convert(iono_beta_v3, *it2, &iono_utc);
            beta = true;
          }
        }
      }

      if((it = _header.find("TIME SYSTEM CORR")) != _header.end()){
        for(it2_t it2(it->second.begin()), it2_end(it->second.end()); it2 != it2_end; ++it2){
          if(it2->find("GPUT") == it2->npos){continue;}
          super_t::convert(utc_v3, *it2, &iono_utc);
          utc = true;
        }
      }

      if((it = _header.find("LEAP SECONDS")) != _header.end()){
        iono_utc.delta_t_LSF = iono_utc.WN_LSF = iono_utc.DN = 0;
        if(version_type.version >= 301){
          super_t::convert(utc_leap_v301, it->second.front(), &iono_utc);
        }else{
          super_t::convert(utc_leap_v2, it->second.front(), &iono_utc);
        }
        leap = true;
      }

      space_node.update_iono_utc(iono_utc, alpha && beta, utc && leap);
      return alpha && beta && utc && leap;
    }

    struct t_corr_glonass_t {
      int year, month, day;
      FloatT tau_c_neg, tau_GPS; // TODO check tau_GPS polarity
      int leap_sec;
      int flags;
      enum {
        TAU_C_NEG = 0x01,
        TAU_GPS   = 0x02,
        LEAP_SEC  = 0x04,
      };
    };
    static const typename super_t::convert_item_t t_corr_glonass_v2[4];

    bool extract_t_corr_glonass_v2(t_corr_glonass_t &t_corr_glonass) const {
      t_corr_glonass.flags = 0;
      super_t::header_t::const_iterator it;

      if((it = _header.find("CORR TO SYSTEM TIME")) != _header.end()){
        super_t::convert(t_corr_glonass_v2, it->second.front(), &t_corr_glonass);
        t_corr_glonass.flags |= t_corr_glonass_t::TAU_C_NEG;
      }

      if((it = _header.find("LEAP SECONDS")) != _header.end()){
        iono_utc_t iono_utc;
        super_t::convert(utc_leap_v2, it->second.front(), &iono_utc);
        t_corr_glonass.leap_sec = iono_utc.delta_t_LS;
        t_corr_glonass.flags |= t_corr_glonass_t::LEAP_SEC;
      }

      return t_corr_glonass.flags > 0;
    }

    bool extract_t_corr_glonass_v3(t_corr_glonass_t &t_corr_glonass) const {
      iono_utc_t iono_utc;
      t_corr_glonass.flags = 0;
      typedef super_t::header_t::const_iterator it_t;
      typedef super_t::header_t::mapped_type::const_iterator it2_t;

      it_t it;

      if((it = _header.find("TIME SYSTEM CORR")) != _header.end()){
        for(it2_t it2(it->second.begin()), it2_end(it->second.end()); it2 != it2_end; ++it2){
          if(it2->find("GLUT") != it2->npos){
            super_t::convert(utc_v3, *it2, &iono_utc);
            t_corr_glonass.year = t_corr_glonass.month = t_corr_glonass.day = 0;
            t_corr_glonass.tau_c_neg = iono_utc.A0;
            t_corr_glonass.flags |= t_corr_glonass_t::TAU_C_NEG;
          }else if(it2->find("GLGP") != it2->npos){
            super_t::convert(utc_v3, *it2, &iono_utc);
            t_corr_glonass.tau_GPS = iono_utc.A0;
            t_corr_glonass.flags |= t_corr_glonass_t::TAU_GPS;
          }
        }
      }

      if((it = _header.find("LEAP SECONDS")) != _header.end()){
        if(version_type.version >= 301){
          super_t::convert(utc_leap_v301, it->second.front(), &iono_utc);
        }else{
          super_t::convert(utc_leap_v2, it->second.front(), &iono_utc);
        }
        t_corr_glonass.leap_sec = iono_utc.delta_t_LS;
        t_corr_glonass.flags |= t_corr_glonass_t::LEAP_SEC;
      }

      return t_corr_glonass.flags > 0;
    }

    struct space_node_list_t {
      space_node_t *gps;
      SBAS_SpaceNode<FloatT> *sbas;
      space_node_t *qzss;
      GLONASS_SpaceNode<FloatT> *glonass;
    };

    static int read_all(std::istream &in, space_node_list_t &space_nodes = {0}){
      RINEX_NAV_Reader reader(in);
      if(reader.version_type.file_type != version_type_t::FTYPE_NAVIGATION){
        return -1;
      }
      if(space_nodes.gps){
        (reader.version_type.version >= 300)
            ? reader.extract_iono_utc_v3(*space_nodes.gps)
            : reader.extract_iono_utc_v2(*space_nodes.gps);
      }
      if(space_nodes.qzss && (space_nodes.gps != space_nodes.qzss)
          && (reader.version_type.version >= 302)){
        reader.extract_iono_utc_v3(*space_nodes.qzss);
      }
      t_corr_glonass_t t_corr_glonass = {0};
      if(space_nodes.glonass){
        (reader.version_type.version >= 300)
            ? reader.extract_t_corr_glonass_v3(t_corr_glonass)
            : reader.extract_t_corr_glonass_v2(t_corr_glonass);
      }
      int res(0);
      for(; reader.has_next(); reader.next()){
        switch(reader.sys_of_msg){
          case super_t::version_type_t::SYS_GPS:
            if(!space_nodes.gps){break;}
            space_nodes.gps->satellite(reader.msg.eph.svid).register_ephemeris(reader.msg.eph);
            res++;
            break;
          case super_t::version_type_t::SYS_SBAS: {
            if(!space_nodes.sbas){break;}
            typename message_sbas_t::eph_t eph(reader.msg_sbas);
            space_nodes.sbas->satellite(eph.svid).register_ephemeris(eph);
            res++;
            break;
          }
          case super_t::version_type_t::SYS_QZSS: {
            if(!space_nodes.qzss){break;}
            typename RINEX_NAV<FloatT>::ephemeris_t eph(reader.msg.eph_qzss());
            space_nodes.qzss->satellite(eph.svid).register_ephemeris(eph);
            res++;
            break;
          }
          case super_t::version_type_t::SYS_GLONASS: {
            if(!space_nodes.glonass){break;}
            typename message_glonass_t::eph_t eph0(reader.msg_glonass);
            eph0.tau_c = -t_corr_glonass.tau_c_neg;
            eph0.tau_GPS = t_corr_glonass.tau_GPS;
            typename GLONASS_SpaceNode<FloatT>::SatelliteProperties::Ephemeris_with_GPS_Time eph(
                eph0,
                (t_corr_glonass.flags & t_corr_glonass_t::LEAP_SEC)
                   ? t_corr_glonass.leap_sec
                   : GPS_Time<FloatT>::guess_leap_seconds(reader.msg_glonass.date_tm));
            space_nodes.glonass->satellite(reader.msg_glonass.svid).register_ephemeris(eph);
            res++;
            break;
          }
          default: break;
        }
      }
      return res;
    }

    static int read_all(std::istream &in, space_node_t &space_node){
      space_node_list_t list = {
        &space_node,
      };
      return read_all(in, list);
    }
};

template <class FloatT>
struct RINEX_OBS {
  struct epoch_flag_t {
    std::tm epoch;
    int epoch_year4, epoch_year2, epoch_mon12;
    FloatT epoch_sec;
    int flag;
    int items_followed;
    FloatT receiver_clock_error;

    epoch_flag_t &operator=(const GPS_Time<FloatT> &t){
      epoch = t.c_tm();
      epoch_year4 = epoch.tm_year + 1900;
      epoch_year2 = epoch.tm_year % 100;
      epoch_mon12 = epoch.tm_mon + 1;
      epoch_sec = std::fmod(t.seconds, 60);
      return *this;
    }
    operator GPS_Time<FloatT>() const {
      return GPS_Time<FloatT>(epoch) + (epoch_sec - epoch.tm_sec);
    }
  };

  struct observation_t {
    GPS_Time<FloatT> t_epoch;
    FloatT receiver_clock_error;
    struct record_t {
      bool valid;
      FloatT value;
      int lli, ss; // if negative
    };
    typedef std::map<int, std::vector<record_t> > per_satellite_t;
    per_satellite_t per_satellite;

    observation_t &operator=(const epoch_flag_t &epoch_flag){
      t_epoch = (GPS_Time<FloatT>)epoch_flag;
      receiver_clock_error = epoch_flag.receiver_clock_error;
      return *this;
    }
    operator epoch_flag_t() const {
      epoch_flag_t res = {0};
      res = t_epoch;
      res.receiver_clock_error = receiver_clock_error;
      return res;
    }
  };
};

template <class FloatT = double>
class RINEX_OBS_Reader : public RINEX_Reader<> {
  protected:
    typedef RINEX_OBS_Reader<FloatT> self_t;
    typedef RINEX_Reader<> super_t;
  public:
    typedef typename RINEX_OBS<FloatT>::epoch_flag_t epoch_flag_t;
    typedef typename RINEX_OBS<FloatT>::observation_t::record_t record_t;

    static const typename super_t::convert_item_t epoch_flag_v2[9], epoch_flag_v3[9];
    static const typename super_t::convert_item_t record_v2v3[3];

    typedef typename RINEX_OBS<FloatT>::observation_t observation_t;

    static int sys2serial(const char &c){
      switch(c){
        case ' ':
        case 'G': return 0; // NAVSTAR
        case 'R': return 0x100; break; // GLONASS
        case 'E': return 0x200; break; // Galileo, since ver 2.11
        case 'J': return 192; break; // QZSS, since ver 3.02, J01 = PRN193
        case 'C': return 0x300; break; // BDS, since ver 3.02
        case 'I': return 0x400; break; // IRNSS, since ver 3.03
        case 'S': return 100;   break; // SBAS, S20 = PRN120
        default: return 0x800;
      }
    }
    static char serial2sys(const int &serial, int &offset){
      switch(serial & 0xF00){
        case 0:
          if(serial < 100){
            offset = serial; return 'G'; // NAVSTAR
          }else if(serial < 192){
            offset = serial - 100; return 'S'; // SBAS, S20 = PRN120
          }else{
            offset = serial - 192; return 'J'; // QZSS, since ver 3.02, J01 = PRN193
          }
          break;
        case 0x100: offset = serial - 0x100; return 'R'; // GLONASS
        case 0x200: offset = serial - 0x200; return 'E'; // Galileo, since ver 2.11
        case 0x300: offset = serial - 0x300; return 'C'; // BDS, since ver 3.02
        case 0x400: offset = serial - 0x400; return 'I'; // IRNSS, since ver 3.03
      }
      offset = serial - 0x800; return ' '; // unknown
    }


  protected:
    typedef std::map<char, std::vector<std::string> > obs_types_t;
    obs_types_t obs_types;
    observation_t obs;
    static std::string &modify_header(std::string &label, std::string &content){
      return content;
    }

    void seek_next_v2() {
      char buf[256];
      typedef std::vector<int> sat_list_t;
      sat_list_t sat_list;
      obs.per_satellite.clear();

      while(true){

        // Read lines for epoch
        {
          if(super_t::src.getline(buf, sizeof(buf)).fail()){return;}
          std::string epoch_str(buf);
          if((epoch_str.size() < 80) && (epoch_str.size() >= 32)){ // minimum 32 characters are required.
            epoch_str.append(80 - epoch_str.size(), ' '); // add blank in case line is truncated
          }

          epoch_flag_t epoch_flag;
          super_t::convert(epoch_flag_v2, epoch_str, &epoch_flag);
          epoch_flag.epoch.tm_year = epoch_flag.epoch_year2 + (epoch_flag.epoch_year2 < 80 ? 100 : 0); // greater than 1980
          epoch_flag.epoch.tm_mon = epoch_flag.epoch_mon12 - 1; // month [0, 11]
          epoch_flag.epoch.tm_sec = (int)epoch_flag.epoch_sec;
          obs = epoch_flag;

          if(epoch_flag.flag >= 2){
            for(int i(0); i < epoch_flag.items_followed; ++i){
              if(super_t::src.getline(buf, sizeof(buf)).fail()){return;}
            }
            continue;
          }

          int prn;
          for(int items(0), i(0); items < epoch_flag.items_followed; items++, i++){
            if(i >= 12){  // if number of satellites is greater than 12, move to the next line
              if(super_t::src.getline(buf, sizeof(buf)).fail()){return;}
              epoch_str = std::string(buf);
              i = 0;
            }
            super_t::template conv_t<int>::d(epoch_str, 33 + (i * 3), 2, &prn);
            sat_list.push_back(prn + sys2serial(epoch_str[32 + (i * 3)]));
          }
        }

        // Observation data per satellite
        int types(obs_types[' '].size());
        for(typename sat_list_t::const_iterator it(sat_list.begin()), it_end(sat_list.end());
            it != it_end; ++it){
          std::string obs_str;
          for(int i(0), offset(80); i < types; i++, offset += 16){
            if(offset >= 80){
              if(super_t::src.getline(buf, sizeof(buf)).fail()){return;}
              obs_str = std::string(buf);
              if(obs_str.size() < 80){
                obs_str.append(80 - obs_str.size(), ' '); // add blank in case line is truncated
              }
              offset = 0;
            }
            typename observation_t::record_t record = {false, 0};
            std::string record_str(obs_str.substr(offset, 16));
            if(record_str[10] == '.'){
              record.valid = true;
              super_t::convert(record_v2v3, record_str, &record);
            }
            obs.per_satellite[*it].push_back(record);
          }
        }

        break;
      }

      super_t::_has_next = true;
    }

    void seek_next_v3() {
      char buf[1024];
      typedef std::vector<int> sat_list_t;
      sat_list_t sat_list;
      obs.per_satellite.clear();

      while(true){

        // Read lines for epoch
        epoch_flag_t epoch_flag;
        {
          if(super_t::src.getline(buf, sizeof(buf)).fail()){return;}
          std::string epoch_str(buf);
          if((epoch_str.size() < 56) && (epoch_str.size() >= 35)){
            epoch_str.append(56 - epoch_str.size(), ' ');
          }

          super_t::convert(epoch_flag_v3, epoch_str, &epoch_flag);
          epoch_flag.epoch.tm_year = epoch_flag.epoch_year4 - 1900; // greater than 1980
          epoch_flag.epoch.tm_mon = epoch_flag.epoch_mon12 - 1; // month [0, 11]
          epoch_flag.epoch.tm_sec = (int)epoch_flag.epoch_sec;
          obs = epoch_flag;

          if(epoch_flag.flag >= 2){
            for(int i(0); i < epoch_flag.items_followed; ++i){
              if(super_t::src.getline(buf, sizeof(buf)).fail()){return;}
            }
            continue;
          }
        }

        // Observation data per satellite
        for(int i(0); i < epoch_flag.items_followed; ++i){
          if(super_t::src.getline(buf, sizeof(buf)).fail()){return;}
          std::string obs_str(buf);
          int types(obs_types[obs_str[0]].size()), serial;
          super_t::template conv_t<int>::d(obs_str, 1, 3, &serial);
          serial += sys2serial(obs_str[0]);
          do{
            int truncated((3 + 16 * types) - obs_str.size());
            if(truncated <= 0){break;}
            obs_str.append(truncated, ' ');
          }while(false);
          for(int j(0), offset(3); j < types; j++, offset += 16){
            typename observation_t::record_t record = {false, 0};
            std::string record_str(obs_str.substr(offset, 16));
            if(record_str[10] == '.'){
              record.valid = true;
              super_t::convert(record_v2v3, record_str, &record);
            }
            obs.per_satellite[serial].push_back(record);
          }
        }

        break;
      }

      super_t::_has_next = true;
    }

    void seek_next(){
      if(obs_types.size() == 0){return;}
      switch(super_t::version_type.version / 100){
        case 2: seek_next_v2(); break;
        case 3: seek_next_v3(); break;
      }
    }

  public:
    RINEX_OBS_Reader(std::istream &in)
        : super_t(in, self_t::modify_header),
          obs_types() {
      if(super_t::version_type.file_type != version_type_t::FTYPE_OBSERVATION){
        return;
      }
      typedef super_t::header_t::const_iterator it_t;
      typedef super_t::header_t::mapped_type::const_iterator it2_t;
      it_t it;
      switch(super_t::version_type.version / 100){
        case 2: if((it = _header.find("# / TYPES OF OBSERV")) != _header.end()){
          int types(0);
          for(it2_t it2(it->second.begin()), it2_end(it->second.end()); it2 != it2_end; ++it2){
            if(types == 0){super_t::conv_t<int>::d(const_cast<std::string &>(*it2), 0, 6, &types);}
            for(int i(0); i < 9; ++i){
              std::string param_name(it2->substr(6 * i + 10, 2));
              if(param_name != "  "){
                obs_types[' '].push_back(param_name);
              }
            }
          }
        }
        break;
        case 3: if((it = _header.find("SYS / # / OBS TYPES")) != _header.end()){
          int types(0); char sys;
          for(it2_t it2(it->second.begin()), it2_end(it->second.end()); it2 != it2_end; ++it2){
            if(types == 0){
              sys = (*it2)[0];
              super_t::conv_t<int>::d(const_cast<std::string &>(*it2), 3, 3, &types);
            }
            for(int i(0); i < 13; ++i){
              std::string param_name(it2->substr(4 * i + 7, 3));
              if(param_name == "   "){continue;}
              obs_types[sys].push_back(param_name);
              if((int)(obs_types[sys].size()) >= types){
                types = 0;
                break;
              }
            }
          }
        }
        break;
      }
      seek_next();
    }
    ~RINEX_OBS_Reader(){}

    observation_t next() {
      observation_t current(obs);
      super_t::_has_next = false;
      seek_next();
      return current;
    }
    /**
     * Return index on data lines corresponding to specified label
     * If not found, return -1.
     *
     */
    int observed_index(const std::string &label, const char &system = ' ') const {
      obs_types_t::const_iterator it(obs_types.find(system));
      if(it == obs_types.end()){return -1;}
      int res(distance(it->second.begin(),
          find(it->second.begin(), it->second.end(), label)));
      return (res >= (int)(it->second.size()) ? -1 : res);
    }
};

#define GEN_D(offset, length, container_type, container_member, value_type) \
    {super_t::template conv_t<value_type>::d, offset, length, \
      offsetof(container_type, container_member)}
#define GEN_I(offset, length, container_type, container_member, value_type) \
    {super_t::template conv_t<value_type>::d, offset, length, \
      offsetof(container_type, container_member), 1}
#define GEN_F2(offset, length, precision, container_type, container_member, value_type) \
    {super_t::template conv_t<value_type>::f_dot_head, offset, length, \
      offsetof(container_type, container_member), precision}
#define GEN_E2(offset, length, precision, container_type, container_member, value_type) \
    {super_t::template conv_t<value_type>::e_dot_head, offset, length, \
      offsetof(container_type, container_member), precision}
#define GEN_F(offset, length, precision, container_type, container_member) \
    GEN_F2(offset, length, precision, container_type, container_member, FloatT)
#define GEN_E(offset, length, precision, container_type, container_member) \
    GEN_E2(offset, length, precision, container_type, container_member, FloatT)

template <class FloatT>
const typename RINEX_NAV_Reader<FloatT>::convert_item_t RINEX_NAV_Reader<FloatT>::eph0_v2[] = {
  GEN_D( 0,  2,     message_t, eph.svid, int),
  GEN_D( 3,  2,     message_t, t_oc_year2,      int),
  GEN_D( 6,  2,     message_t, t_oc_mon12,      int),
  GEN_D( 9,  2,     message_t, t_oc_tm.tm_mday, int),
  GEN_D(12,  2,     message_t, t_oc_tm.tm_hour, int),
  GEN_D(15,  2,     message_t, t_oc_tm.tm_min,  int),
  GEN_F(17,  5,  1, message_t, t_oc_sec),
  GEN_E(22, 19, 12, message_t, eph.a_f0),
  GEN_E(41, 19, 12, message_t, eph.a_f1),
  GEN_E(60, 19, 12, message_t, eph.a_f2),
};
template <class FloatT>
const typename RINEX_NAV_Reader<FloatT>::convert_item_t RINEX_NAV_Reader<FloatT>::eph0_v3[] = {
  GEN_I( 1,  2,     message_t, eph.svid, int),
  GEN_I( 4,  4,     message_t, t_oc_year4,      int),
  GEN_I( 9,  2,     message_t, t_oc_mon12,      int),
  GEN_I(12,  2,     message_t, t_oc_tm.tm_mday, int),
  GEN_I(15,  2,     message_t, t_oc_tm.tm_hour, int),
  GEN_I(18,  2,     message_t, t_oc_tm.tm_min,  int),
  GEN_I(21,  2,     message_t, t_oc_tm.tm_sec,  int),
  GEN_E(23, 19, 12, message_t, eph.a_f0),
  GEN_E(42, 19, 12, message_t, eph.a_f1),
  GEN_E(61, 19, 12, message_t, eph.a_f2),
};

template <class FloatT>
const typename RINEX_NAV_Reader<FloatT>::convert_item_t RINEX_NAV_Reader<FloatT>::eph1_v2[] = {
  GEN_E2( 3, 19, 12, message_t, eph.iode, int),
  GEN_E (22, 19, 12, message_t, eph.c_rs),
  GEN_E (41, 19, 12, message_t, eph.delta_n),
  GEN_E (60, 19, 12, message_t, eph.M0),
};
template <class FloatT>
const typename RINEX_NAV_Reader<FloatT>::convert_item_t RINEX_NAV_Reader<FloatT>::eph1_v3[] = {
  GEN_E2( 4, 19, 12, message_t, eph.iode, int),
  GEN_E (23, 19, 12, message_t, eph.c_rs),
  GEN_E (42, 19, 12, message_t, eph.delta_n),
  GEN_E (61, 19, 12, message_t, eph.M0),
};

template <class FloatT>
const typename RINEX_NAV_Reader<FloatT>::convert_item_t RINEX_NAV_Reader<FloatT>::eph2_v2[] = {
  GEN_E( 3, 19, 12, message_t, eph.c_uc),
  GEN_E(22, 19, 12, message_t, eph.e),
  GEN_E(41, 19, 12, message_t, eph.c_us),
  GEN_E(60, 19, 12, message_t, eph.sqrt_A),
};
template <class FloatT>
const typename RINEX_NAV_Reader<FloatT>::convert_item_t RINEX_NAV_Reader<FloatT>::eph2_v3[] = {
  GEN_E( 4, 19, 12, message_t, eph.c_uc),
  GEN_E(23, 19, 12, message_t, eph.e),
  GEN_E(42, 19, 12, message_t, eph.c_us),
  GEN_E(61, 19, 12, message_t, eph.sqrt_A),
};

template <class FloatT>
const typename RINEX_NAV_Reader<FloatT>::convert_item_t RINEX_NAV_Reader<FloatT>::eph3_v2[] = {
  GEN_E( 3, 19, 12, message_t, eph.t_oe),
  GEN_E(22, 19, 12, message_t, eph.c_ic),
  GEN_E(41, 19, 12, message_t, eph.Omega0),
  GEN_E(60, 19, 12, message_t, eph.c_is),
};
template <class FloatT>
const typename RINEX_NAV_Reader<FloatT>::convert_item_t RINEX_NAV_Reader<FloatT>::eph3_v3[] = {
  GEN_E( 4, 19, 12, message_t, eph.t_oe),
  GEN_E(23, 19, 12, message_t, eph.c_ic),
  GEN_E(42, 19, 12, message_t, eph.Omega0),
  GEN_E(61, 19, 12, message_t, eph.c_is),
};

template <class FloatT>
const typename RINEX_NAV_Reader<FloatT>::convert_item_t RINEX_NAV_Reader<FloatT>::eph4_v2[] = {
  GEN_E( 3, 19, 12, message_t, eph.i0),
  GEN_E(22, 19, 12, message_t, eph.c_rc),
  GEN_E(41, 19, 12, message_t, eph.omega),
  GEN_E(60, 19, 12, message_t, eph.dot_Omega0),
};
template <class FloatT>
const typename RINEX_NAV_Reader<FloatT>::convert_item_t RINEX_NAV_Reader<FloatT>::eph4_v3[] = {
  GEN_E( 4, 19, 12, message_t, eph.i0),
  GEN_E(23, 19, 12, message_t, eph.c_rc),
  GEN_E(42, 19, 12, message_t, eph.omega),
  GEN_E(61, 19, 12, message_t, eph.dot_Omega0),
};

template <class FloatT>
const typename RINEX_NAV_Reader<FloatT>::convert_item_t RINEX_NAV_Reader<FloatT>::eph5_v2[] = {
  GEN_E( 3, 19, 12, message_t, eph.dot_i0),
  GEN_E(22, 19, 12, message_t, dummy), // Codes on L2 channel
  GEN_E(41, 19, 12, message_t, t_oe_WN),
  GEN_E(60, 19, 12, message_t, dummy), // L2 P data flag
};
template <class FloatT>
const typename RINEX_NAV_Reader<FloatT>::convert_item_t RINEX_NAV_Reader<FloatT>::eph5_v3[] = {
  GEN_E( 4, 19, 12, message_t, eph.dot_i0),
  GEN_E(23, 19, 12, message_t, dummy), // Codes on L2 channel
  GEN_E(42, 19, 12, message_t, t_oe_WN),
  GEN_E(61, 19, 12, message_t, dummy), // L2 P data flag
};

template <class FloatT>
const typename RINEX_NAV_Reader<FloatT>::convert_item_t RINEX_NAV_Reader<FloatT>::eph6_v2[] = {
  GEN_E ( 3, 19, 12, message_t, eph.URA),
  GEN_E2(22, 19, 12, message_t, eph.SV_health, unsigned int),
  GEN_E (41, 19, 12, message_t, eph.t_GD),
  GEN_E2(60, 19, 12, message_t, eph.iodc, int),
};
template <class FloatT>
const typename RINEX_NAV_Reader<FloatT>::convert_item_t RINEX_NAV_Reader<FloatT>::eph6_v3[] = {
  GEN_E ( 4, 19, 12, message_t, eph.URA),
  GEN_E2(23, 19, 12, message_t, eph.SV_health, unsigned int),
  GEN_E (42, 19, 12, message_t, eph.t_GD),
  GEN_E2(61, 19, 12, message_t, eph.iodc, int),
};

template <class FloatT>
const typename RINEX_NAV_Reader<FloatT>::convert_item_t RINEX_NAV_Reader<FloatT>::eph7_v2[] = {
  GEN_E( 3, 19, 12, message_t, t_ot),
  GEN_E(22, 19, 12, message_t, fit_interval_hr),
};
template <class FloatT>
const typename RINEX_NAV_Reader<FloatT>::convert_item_t RINEX_NAV_Reader<FloatT>::eph7_v3[] = {
  GEN_E( 4, 19, 12, message_t, t_ot),
  GEN_E(23, 19, 12, message_t, fit_interval_hr),
};


template <class FloatT>
const typename RINEX_NAV_Reader<FloatT>::convert_item_t RINEX_NAV_Reader<FloatT>::eph0_sbas_v2[] = {
  GEN_D( 0,  2,     message_sbas_t, svid,            int),
  GEN_D( 3,  2,     message_sbas_t, t_year2,         int),
  GEN_D( 6,  2,     message_sbas_t, t_mon12,         int),
  GEN_D( 9,  2,     message_sbas_t, date_tm.tm_mday, int),
  GEN_D(12,  2,     message_sbas_t, date_tm.tm_hour, int),
  GEN_D(15,  2,     message_sbas_t, date_tm.tm_min,  int),
  GEN_F(17,  5,  1, message_sbas_t, t_sec),
  GEN_E(22, 19, 12, message_sbas_t, a_Gf0),
  GEN_E(41, 19, 12, message_sbas_t, a_Gf1),
  GEN_E(60, 19, 12, message_sbas_t, t_t),
};
template <class FloatT>
const typename RINEX_NAV_Reader<FloatT>::convert_item_t RINEX_NAV_Reader<FloatT>::eph0_sbas_v3[] = {
  GEN_I( 1,  2,     message_sbas_t, svid,            int),
  GEN_I( 4,  4,     message_sbas_t, t_year4,         int),
  GEN_I( 9,  2,     message_sbas_t, t_mon12,         int),
  GEN_I(12,  2,     message_sbas_t, date_tm.tm_mday, int),
  GEN_I(15,  2,     message_sbas_t, date_tm.tm_hour, int),
  GEN_I(18,  2,     message_sbas_t, date_tm.tm_min,  int),
  GEN_I(21,  2,     message_sbas_t, date_tm.tm_sec,  int),
  GEN_E(23, 19, 12, message_sbas_t, a_Gf0),
  GEN_E(42, 19, 12, message_sbas_t, a_Gf1),
  GEN_E(61, 19, 12, message_sbas_t, t_t),
};

template <class FloatT>
const typename RINEX_NAV_Reader<FloatT>::convert_item_t RINEX_NAV_Reader<FloatT>::eph1_sbas_v2[] = {
  GEN_E( 3, 19, 12, message_sbas_t, x_km),
  GEN_E(22, 19, 12, message_sbas_t, dx_km_s),
  GEN_E(41, 19, 12, message_sbas_t, ddx_km_s2),
  GEN_E2(60, 19, 12, message_sbas_t, health, unsigned int),
};
template <class FloatT>
const typename RINEX_NAV_Reader<FloatT>::convert_item_t RINEX_NAV_Reader<FloatT>::eph1_sbas_v3[] = {
  GEN_E( 4, 19, 12, message_sbas_t, x_km),
  GEN_E(23, 19, 12, message_sbas_t, dx_km_s),
  GEN_E(42, 19, 12, message_sbas_t, ddx_km_s2),
  GEN_E2(61, 19, 12, message_sbas_t, health, unsigned int),
};

template <class FloatT>
const typename RINEX_NAV_Reader<FloatT>::convert_item_t RINEX_NAV_Reader<FloatT>::eph2_sbas_v2[] = {
  GEN_E( 3, 19, 12, message_sbas_t, y_km),
  GEN_E(22, 19, 12, message_sbas_t, dy_km_s),
  GEN_E(41, 19, 12, message_sbas_t, ddy_km_s2),
  GEN_E(60, 19, 12, message_sbas_t, URA),
};
template <class FloatT>
const typename RINEX_NAV_Reader<FloatT>::convert_item_t RINEX_NAV_Reader<FloatT>::eph2_sbas_v3[] = {
  GEN_E( 4, 19, 12, message_sbas_t, y_km),
  GEN_E(23, 19, 12, message_sbas_t, dy_km_s),
  GEN_E(42, 19, 12, message_sbas_t, ddy_km_s2),
  GEN_E(61, 19, 12, message_sbas_t, URA),
};

template <class FloatT>
const typename RINEX_NAV_Reader<FloatT>::convert_item_t RINEX_NAV_Reader<FloatT>::eph3_sbas_v2[] = {
  GEN_E( 3, 19, 12, message_sbas_t, z_km),
  GEN_E(22, 19, 12, message_sbas_t, dz_km_s),
  GEN_E(41, 19, 12, message_sbas_t, ddz_km_s2),
  GEN_E2(60, 19, 12, message_sbas_t, iodn, unsigned int),
};
template <class FloatT>
const typename RINEX_NAV_Reader<FloatT>::convert_item_t RINEX_NAV_Reader<FloatT>::eph3_sbas_v3[] = {
  GEN_E( 4, 19, 12, message_sbas_t, z_km),
  GEN_E(23, 19, 12, message_sbas_t, dz_km_s),
  GEN_E(42, 19, 12, message_sbas_t, ddz_km_s2),
  GEN_E2(61, 19, 12, message_sbas_t, iodn, unsigned int),
};


template <class FloatT>
const typename RINEX_NAV_Reader<FloatT>::convert_item_t RINEX_NAV_Reader<FloatT>::eph0_glonass_v2[] = {
  GEN_D( 0,  2,     message_glonass_t, svid,            int),
  GEN_D( 3,  2,     message_glonass_t, t_year2,         int),
  GEN_D( 6,  2,     message_glonass_t, t_mon12,         int),
  GEN_D( 9,  2,     message_glonass_t, date_tm.tm_mday, int),
  GEN_D(12,  2,     message_glonass_t, date_tm.tm_hour, int),
  GEN_D(15,  2,     message_glonass_t, date_tm.tm_min,  int),
  GEN_F(17,  5,  1, message_glonass_t, t_sec),
  GEN_E(22, 19, 12, message_glonass_t, tau_n_neg),
  GEN_E(41, 19, 12, message_glonass_t, gamma_n),
  GEN_E2(60, 19, 12, message_glonass_t, t_k, unsigned int),
};
template <class FloatT>
const typename RINEX_NAV_Reader<FloatT>::convert_item_t RINEX_NAV_Reader<FloatT>::eph0_glonass_v3[] = {
  GEN_I( 1,  2,     message_glonass_t, svid,            int),
  GEN_I( 4,  4,     message_glonass_t, t_year4,         int),
  GEN_I( 9,  2,     message_glonass_t, t_mon12,         int),
  GEN_I(12,  2,     message_glonass_t, date_tm.tm_mday, int),
  GEN_I(15,  2,     message_glonass_t, date_tm.tm_hour, int),
  GEN_I(18,  2,     message_glonass_t, date_tm.tm_min,  int),
  GEN_I(21,  2,     message_glonass_t, date_tm.tm_sec,  int),
  GEN_E(23, 19, 12, message_glonass_t, tau_n_neg),
  GEN_E(42, 19, 12, message_glonass_t, gamma_n),
  GEN_E2(61, 19, 12, message_glonass_t, t_k, unsigned int),
};

template <class FloatT>
const typename RINEX_NAV_Reader<FloatT>::convert_item_t RINEX_NAV_Reader<FloatT>::eph1_glonass_v2[] = {
  GEN_E( 3, 19, 12, message_glonass_t, x_km),
  GEN_E(22, 19, 12, message_glonass_t, dx_km_s),
  GEN_E(41, 19, 12, message_glonass_t, ddx_km_s2),
  GEN_E2(60, 19, 12, message_glonass_t, B_n, unsigned int),
};
template <class FloatT>
const typename RINEX_NAV_Reader<FloatT>::convert_item_t RINEX_NAV_Reader<FloatT>::eph1_glonass_v3[] = {
  GEN_E( 4, 19, 12, message_glonass_t, x_km),
  GEN_E(23, 19, 12, message_glonass_t, dx_km_s),
  GEN_E(42, 19, 12, message_glonass_t, ddx_km_s2),
  GEN_E2(61, 19, 12, message_glonass_t, B_n, unsigned int),
};

template <class FloatT>
const typename RINEX_NAV_Reader<FloatT>::convert_item_t RINEX_NAV_Reader<FloatT>::eph2_glonass_v2[] = {
  GEN_E( 3, 19, 12, message_glonass_t, y_km),
  GEN_E(22, 19, 12, message_glonass_t, dy_km_s),
  GEN_E(41, 19, 12, message_glonass_t, ddy_km_s2),
  GEN_E2(60, 19, 12, message_glonass_t, freq_num, int),
};
template <class FloatT>
const typename RINEX_NAV_Reader<FloatT>::convert_item_t RINEX_NAV_Reader<FloatT>::eph2_glonass_v3[] = {
  GEN_E( 4, 19, 12, message_glonass_t, y_km),
  GEN_E(23, 19, 12, message_glonass_t, dy_km_s),
  GEN_E(42, 19, 12, message_glonass_t, ddy_km_s2),
  GEN_E2(61, 19, 12, message_glonass_t, freq_num, int),
};

template <class FloatT>
const typename RINEX_NAV_Reader<FloatT>::convert_item_t RINEX_NAV_Reader<FloatT>::eph3_glonass_v2[] = {
  GEN_E( 3, 19, 12, message_glonass_t, z_km),
  GEN_E(22, 19, 12, message_glonass_t, dz_km_s),
  GEN_E(41, 19, 12, message_glonass_t, ddz_km_s2),
  GEN_E2(60, 19, 12, message_glonass_t, E_n, unsigned int),
};
template <class FloatT>
const typename RINEX_NAV_Reader<FloatT>::convert_item_t RINEX_NAV_Reader<FloatT>::eph3_glonass_v3[] = {
  GEN_E( 4, 19, 12, message_glonass_t, z_km),
  GEN_E(23, 19, 12, message_glonass_t, dz_km_s),
  GEN_E(42, 19, 12, message_glonass_t, ddz_km_s2),
  GEN_E2(61, 19, 12, message_glonass_t, E_n, unsigned int),
};

template <class FloatT>
const typename RINEX_NAV_Reader<FloatT>::convert_item_t RINEX_NAV_Reader<FloatT>::eph4_glonass_v305[] = {
  GEN_E2( 4, 19, 12, message_glonass_t, status_flags, unsigned int),
  GEN_E (23, 19, 12, message_glonass_t, delta_tau),
  GEN_E2(42, 19, 12, message_glonass_t, urai, unsigned int),
  GEN_E2(61, 19, 12, message_glonass_t, health_flags, unsigned int),
};

template <class FloatT>
const typename RINEX_NAV_Reader<FloatT>::convert_item_t RINEX_NAV_Reader<FloatT>::iono_alpha_v2[] = {
  GEN_E( 2, 12, 4, iono_utc_t, alpha[0]),
  GEN_E(14, 12, 4, iono_utc_t, alpha[1]),
  GEN_E(26, 12, 4, iono_utc_t, alpha[2]),
  GEN_E(38, 12, 4, iono_utc_t, alpha[3]),
};

template <class FloatT>
const typename RINEX_NAV_Reader<FloatT>::convert_item_t RINEX_NAV_Reader<FloatT>::iono_beta_v2[] = {
  GEN_E( 2, 12, 4, iono_utc_t, beta[0]),
  GEN_E(14, 12, 4, iono_utc_t, beta[1]),
  GEN_E(26, 12, 4, iono_utc_t, beta[2]),
  GEN_E(38, 12, 4, iono_utc_t, beta[3]),
};

template <class FloatT>
const typename RINEX_NAV_Reader<FloatT>::convert_item_t RINEX_NAV_Reader<FloatT>::utc_v2[] = {
  GEN_E(  3, 19, 12, iono_utc_t, A0),
  GEN_E( 22, 19, 12, iono_utc_t, A1),
  GEN_D( 41,  9,     iono_utc_t, t_ot, int),
  GEN_D( 50,  9,     iono_utc_t, WN_t, int),
};

template <class FloatT>
const typename RINEX_NAV_Reader<FloatT>::convert_item_t RINEX_NAV_Reader<FloatT>::utc_leap_v2[] = {
  GEN_D(0, 6, iono_utc_t, delta_t_LS, int),
};


template <class FloatT>
const typename RINEX_NAV_Reader<FloatT>::convert_item_t RINEX_NAV_Reader<FloatT>::iono_alpha_v3[] = {
  GEN_E( 5, 12, 4, iono_utc_t, alpha[0]),
  GEN_E(17, 12, 4, iono_utc_t, alpha[1]),
  GEN_E(29, 12, 4, iono_utc_t, alpha[2]),
  GEN_E(41, 12, 4, iono_utc_t, alpha[3]),
};

template <class FloatT>
const typename RINEX_NAV_Reader<FloatT>::convert_item_t RINEX_NAV_Reader<FloatT>::iono_beta_v3[] = {
  GEN_E( 5, 12, 4, iono_utc_t, beta[0]),
  GEN_E(17, 12, 4, iono_utc_t, beta[1]),
  GEN_E(29, 12, 4, iono_utc_t, beta[2]),
  GEN_E(41, 12, 4, iono_utc_t, beta[3]),
};

template <class FloatT>
const typename RINEX_NAV_Reader<FloatT>::convert_item_t RINEX_NAV_Reader<FloatT>::utc_v3[] = {
  GEN_E( 5, 17, 10, iono_utc_t, A0),
  GEN_E(22, 16,  9, iono_utc_t, A1),
  GEN_D(39,  6,     iono_utc_t, t_ot, int),
  GEN_D(46,  4,     iono_utc_t, WN_t, int),
};

template <class FloatT>
const typename RINEX_NAV_Reader<FloatT>::convert_item_t RINEX_NAV_Reader<FloatT>::utc_leap_v301[] = {
  GEN_D( 0, 6, iono_utc_t, delta_t_LS,  int),
  GEN_D( 6, 6, iono_utc_t, delta_t_LSF, int),
  GEN_D(12, 6, iono_utc_t, WN_LSF,      int),
  GEN_D(18, 6, iono_utc_t, DN,          int),
};

template <class FloatT>
const typename RINEX_NAV_Reader<FloatT>::convert_item_t RINEX_NAV_Reader<FloatT>::t_corr_glonass_v2[] = {
  GEN_D( 0,  6,     t_corr_glonass_t, year,  int),
  GEN_D( 6,  6,     t_corr_glonass_t, month, int),
  GEN_D(12,  6,     t_corr_glonass_t, day,   int),
  GEN_E(21, 19, 12, t_corr_glonass_t, tau_c_neg),
};

template <class FloatT>
const typename RINEX_OBS_Reader<FloatT>::convert_item_t RINEX_OBS_Reader<FloatT>::epoch_flag_v2[] = {
  GEN_I( 1,  2,     epoch_flag_t, epoch_year2,    int),
  GEN_D( 4,  2,     epoch_flag_t, epoch_mon12,    int),
  GEN_D( 7,  2,     epoch_flag_t, epoch.tm_mday,  int),
  GEN_D(10,  2,     epoch_flag_t, epoch.tm_hour,  int),
  GEN_D(13,  2,     epoch_flag_t, epoch.tm_min,   int),
  GEN_F(15, 11,  7, epoch_flag_t, epoch_sec),
  GEN_D(28,  1,     epoch_flag_t, flag,           int),
  GEN_D(29,  3,     epoch_flag_t, items_followed, int), // Satellite(flag = 0/1) or Line number(except for  0/1)
  GEN_F(68, 12,  9, epoch_flag_t, receiver_clock_error),
};

template <class FloatT>
const typename RINEX_OBS_Reader<FloatT>::convert_item_t RINEX_OBS_Reader<FloatT>::epoch_flag_v3[] = {
  GEN_I( 2,  4,     epoch_flag_t, epoch_year4,    int),
  GEN_I( 7,  2,     epoch_flag_t, epoch_mon12,    int),
  GEN_I(10,  2,     epoch_flag_t, epoch.tm_mday,  int),
  GEN_I(13,  2,     epoch_flag_t, epoch.tm_hour,  int),
  GEN_I(16,  2,     epoch_flag_t, epoch.tm_min,   int),
  GEN_F(18, 11,  7, epoch_flag_t, epoch_sec),
  GEN_D(31,  1,     epoch_flag_t, flag,           int),
  GEN_D(32,  3,     epoch_flag_t, items_followed, int), // Satellite(flag = 0/1) or Line number(except for  0/1)
  GEN_F(41, 15, 12, epoch_flag_t, receiver_clock_error),
};

template <class FloatT>
const typename RINEX_OBS_Reader<FloatT>::convert_item_t RINEX_OBS_Reader<FloatT>::record_v2v3[] = {
  GEN_F( 0, 14,  3, record_t, value),
  GEN_D(14,  1,     record_t, lli,      int),
  GEN_D(15,  1,     record_t, ss,       int),
};

#undef GEN_D
#undef GEN_I
#undef GEN_F
#undef GEN_E

template <class U = void>
class RINEX_Writer {
  public:
    typedef typename RINEX_Reader<U>::version_type_t version_type_t;
    typedef struct {const char *key, *value;} header_item_t;
    
    struct header_t : public std::vector<std::pair<std::string, std::string> > {
      typedef std::vector<std::pair<std::string, std::string> > super_t;

      using super_t::operator[];
      struct bracket_accessor_t {
        header_t &header;
        typename super_t::value_type::first_type key;
        typename super_t::iterator it_head, it_tail;
        bracket_accessor_t(header_t &_header, const typename super_t::value_type::first_type &_key)
            : header(_header), key(_key), it_head(header.end()), it_tail(header.end()) {
          for(typename super_t::iterator it(header.begin()), it_end(header.end());
              it != it_end; ++it){
            if(it->first != key){continue;}
            it_head = it;
            break;
          }
          for(typename super_t::reverse_iterator it(header.rbegin()), it_end(header.rend());
              it != it_end; ++it){
            if(it->first != key){continue;}
            it_tail = it.base(); // it_tail points to the next element of an element having the smae key
            break;
          }
        }
        /**
         * replace value corresponding to key with erase of other entries having the same key
         */
        bracket_accessor_t &operator=(
            const typename super_t::value_type::second_type &value){
          if(it_head == header.end()){
            header.push_back(typename super_t::value_type(key, value));
            it_tail = header.end(); // in case of invalidation
            it_head = it_tail - 1;
            return *this;
          }
          it_head->second = value;
          for(--it_tail; it_tail != it_head; --it_tail){
            if(it_tail->first != key){continue;}
            header.erase(it_tail);
          }
          it_tail = it_head + 1;
          return *this;
        }
        /**
         * add value corresponding to key after the last entry having the same key
         */
        bracket_accessor_t &operator<<(
            const typename super_t::value_type::second_type &value){
          super_t::size_type i_head(it_head - header.begin()), i_tail(it_tail - header.begin());
          header.insert(it_tail, typename super_t::value_type(key, value));
          it_head = header.begin() + i_head;  // in case of invalidation
          it_tail = header.begin() + i_tail + 1;
          return *this;
        }
        unsigned int entries() const {
          return it_tail - it_head;
        }
        typename super_t::iterator find(
            const typename super_t::value_type::second_type &value) const {
          for(typename super_t::iterator it(it_head); it != it_tail; ++it){
            if(it->second.find(value) != super_t::value_type::second_type::npos){
              return it;
            }
          }
          return header.end();
        }
      };
      bracket_accessor_t operator[]( // mimic of std::map::operator[]=
          const typename super_t::value_type::first_type &key){
        return bracket_accessor_t(*this, key);
      }
      /**
       * @param mask Defualt key-value pairs; its order is preserved for ourputs
       * @param mask_size size of masked items
       */
      header_t(
          const header_item_t *mandatory_items = NULL,
          const int &mandatory_item_size = 0)
          : super_t() {
        for(int i(0); i < mandatory_item_size; i++){
          super_t::push_back(typename super_t::value_type(
              mandatory_items[i].key, mandatory_items[i].value ? mandatory_items[i].value : ""));
        }
      }
      ~header_t() {}
      friend std::ostream &operator<<(std::ostream &out, const header_t &header){
        std::stringstream ss;
        ss << std::setfill(' ') << std::left;
        for(typename header_t::const_iterator it(header.begin()), it_end(header.end());
            it != it_end; ++it){
          ss << std::setw(60) << it->second.substr(0, 60)
              << std::setw(20) << it->first.substr(0, 20)
              << std::endl;
        }
        ss << std::setw(60) << "" << std::setw(20) << "END OF HEADER" << std::endl;
        return out << ss.str();
      }
    };
  protected:
    version_type_t _version_type;
    header_t _header;
    std::ostream &dest;
    
    void set_version_type(const version_type_t &version_type){
      _version_type = version_type;
      std::string buf(60, ' ');
      _version_type.dump(buf);
      _header["RINEX VERSION / TYPE"] = buf;
    }

  public:
    typedef RINEX_Writer<U> self_t;
    RINEX_Writer(
        std::ostream &out,
        const header_item_t *header_mask = NULL, 
        const int header_mask_size = 0) 
        : _version_type(), _header(header_mask, header_mask_size), dest(out) {
    }
    virtual ~RINEX_Writer() {_header.clear();}

    header_t &header(){return _header;}
    const header_t &header() const {return const_cast<self_t *>(this)->header();}
    
    template <class FloatT>
    static std::string RINEX_Float(
        const FloatT &value, const int width = 19, const int precision = 12){
      std::string s;
      RINEX_Reader<U>::template conv_t<FloatT>::f_dot_head(s, 0, width, &const_cast<FloatT &>(value), precision, false);
      return s;
    }
    template <class FloatT>
    static std::string RINEX_FloatD(
        const FloatT &value, const int width = 19, const int precision = 12){
      std::string s;
      RINEX_Reader<U>::template conv_t<FloatT>::e_dot_head(s, 0, width, &const_cast<FloatT &>(value), precision, false);
      return s;
    }
    template <class T>
    static std::string RINEX_Value(const T &value, const int width = 6){
      std::string s;
      RINEX_Reader<U>::template conv_t<T>::d(s, 0, width, &const_cast<T &>(value), 0, false);
      return s;
    }
    
    static bool convert(
        const typename RINEX_Reader<U>::convert_item_t *items, const int &size,
        std::string &buf, const void *values){
      return TextHelper<>::val2str(items, size, buf, values);
    }
    template <int N>
    static inline bool convert(
        const typename RINEX_Reader<U>::convert_item_t (&items)[N], std::string &buf, const void *values){
      return convert(items, N, buf, values);
    }

    void pgm_runby_date(
        const std::string &pgm, const std::string &runby,
        const std::tm &t, const char *tz = "UTC"){
      // ex) "XXRINEXO V9.9       AIUB                19900912 124300 UTC"
      char buf[21] = {0};
      std::strftime(buf, sizeof(buf) - 1, "%Y%m%d %H%M%S", &t);
      std::sprintf(&buf[15], " %-4s", tz);
      std::stringstream ss;
      ss << std::setfill(' ') << std::left
          << std::setw(20) << pgm.substr(0, 20)
          << std::setw(20) << runby.substr(0, 20)
          << buf;
      _header["PGM / RUN BY / DATE"] = ss.str();
    }
};

template <class FloatT>
class RINEX_NAV_Writer : public RINEX_Writer<> {
  public:
    typedef RINEX_NAV_Reader<FloatT> reader_t;
    typedef RINEX_NAV_Writer self_t;
    typedef RINEX_Writer<> super_t;
  protected:
    using super_t::_header;
    using super_t::dest;
  public:
    typedef typename RINEX_NAV<FloatT>::space_node_t space_node_t;
    typedef typename RINEX_NAV<FloatT>::message_t message_t;
    typedef typename RINEX_NAV<FloatT>::message_sbas_t message_sbas_t;
    typedef typename RINEX_NAV<FloatT>::message_glonass_t message_glonass_t;

    static const typename super_t::header_item_t default_header[];
    static const int default_header_size;
    void iono_alpha(const space_node_t &space_node){
      std::string s(60, ' ');
      switch(super_t::_version_type.version / 100){
        case 2:
          super_t::convert(reader_t::iono_alpha_v2, s, &space_node.iono_utc());
          _header["ION ALPHA"] = s;
          break;
        case 3:
          super_t::convert(reader_t::iono_alpha_v3, s, &space_node.iono_utc());
          _header["IONOSPHERIC CORR"] << s.replace(0, 4, "GPSA", 4);
          break;
      }
    }
    void iono_beta(const space_node_t &space_node){
      std::string s(60, ' ');
      switch(super_t::_version_type.version / 100){
        case 2:
          super_t::convert(reader_t::iono_beta_v2, s, &space_node.iono_utc());
          _header["ION BETA"] = s;
          break;
        case 3:
          super_t::convert(reader_t::iono_beta_v3, s, &space_node.iono_utc());
          _header["IONOSPHERIC CORR"] << s.replace(0, 4, "GPSB", 4);
          break;
      }
    }
    void utc_params(const space_node_t &space_node){
      std::string s(60, ' ');
      switch(super_t::_version_type.version / 100){
        case 2:
          super_t::convert(reader_t::utc_v2, s, &space_node.iono_utc());
          _header["DELTA-UTC: A0,A1,T,W"] = s;
          break;
        case 3:
          super_t::convert(reader_t::utc_v3, s, &space_node.iono_utc());
          _header["TIME SYSTEM CORR"] << s.replace(0, 4, "GPUT", 4);
          break;
      }
    }
    void leap_seconds(const space_node_t &space_node){
      std::string s(60, ' ');
      if(super_t::_version_type.version >= 301){
        super_t::convert(reader_t::utc_leap_v301, s, &space_node.iono_utc());
        if(space_node.iono_utc().WN_LSF == 0){
          s.replace(6, 18, 18, ' ');
        }
      }else{
        super_t::convert(reader_t::utc_leap_v2, s, &space_node.iono_utc());
      }
      _header["LEAP SECONDS"] = s;
    }
    RINEX_NAV_Writer(std::ostream &out)
        : super_t(out, default_header, default_header_size) {}
    ~RINEX_NAV_Writer(){}

    self_t &dump(const message_t &msg, const bool &is_qzss = false){
      std::stringstream buf;
      switch(super_t::_version_type.version / 100){
        case 2:
          for(int i(0); i < 8; ++i){
            std::string s(80, ' ');
            switch(i){
              case 0: super_t::convert(reader_t::eph0_v2, s, &msg); break;
              case 1: super_t::convert(reader_t::eph1_v2, s, &msg); break;
              case 2: super_t::convert(reader_t::eph2_v2, s, &msg); break;
              case 3: super_t::convert(reader_t::eph3_v2, s, &msg); break;
              case 4: super_t::convert(reader_t::eph4_v2, s, &msg); break;
              case 5: super_t::convert(reader_t::eph5_v2, s, &msg); break;
              case 6: super_t::convert(reader_t::eph6_v2, s, &msg); break;
              case 7: super_t::convert(reader_t::eph7_v2, s, &msg); break;
            }
            buf << s << std::endl;
          }
          break;
        case 3:
          for(int i(0); i < 8; ++i){
            std::string s(80, ' ');
            switch(i){
              case 0: super_t::convert(reader_t::eph0_v3, s, &msg);
                s[0] = is_qzss ? 'J' : 'G'; break;
              case 1: super_t::convert(reader_t::eph1_v3, s, &msg); break;
              case 2: super_t::convert(reader_t::eph2_v3, s, &msg); break;
              case 3: super_t::convert(reader_t::eph3_v3, s, &msg); break;
              case 4: super_t::convert(reader_t::eph4_v3, s, &msg); break;
              case 5: super_t::convert(reader_t::eph5_v3, s, &msg); break;
              case 6: super_t::convert(reader_t::eph6_v3, s, &msg); break;
              case 7: super_t::convert(reader_t::eph7_v3, s, &msg); break;
            }
            buf << s << std::endl;
          }
          break;
      }
      dest << buf.str();
      return *this;
    }
    self_t &operator<<(const message_t &msg){
      return dump(msg);
    }
    self_t &operator<<(const message_sbas_t &msg){
      std::stringstream buf;
      switch(super_t::_version_type.version / 100){
        case 2:
          for(int i(0); i < 8; ++i){
            std::string s(80, ' ');
            switch(i){
              case 0: super_t::convert(reader_t::eph0_sbas_v2, s, &msg); break;
              case 1: super_t::convert(reader_t::eph1_sbas_v2, s, &msg); break;
              case 2: super_t::convert(reader_t::eph2_sbas_v2, s, &msg); break;
              case 3: super_t::convert(reader_t::eph3_sbas_v2, s, &msg); break;
            }
            buf << s << std::endl;
          }
          break;
        case 3:
          for(int i(0); i < 8; ++i){
            std::string s(80, ' ');
            switch(i){
              case 0: super_t::convert(reader_t::eph0_sbas_v3, s, &msg); s[0] = 'S'; break;
              case 1: super_t::convert(reader_t::eph1_sbas_v3, s, &msg); break;
              case 2: super_t::convert(reader_t::eph2_sbas_v3, s, &msg); break;
              case 3: super_t::convert(reader_t::eph3_sbas_v3, s, &msg); break;
            }
            buf << s << std::endl;
          }
          break;
      }
      dest << buf.str();
      return *this;
    }
    self_t &operator<<(const message_glonass_t &msg){
      std::stringstream buf;
      switch(super_t::_version_type.version / 100){
        case 2:
          for(int i(0); i < 4; ++i){
            std::string s(80, ' ');
            switch(i){
              case 0: super_t::convert(reader_t::eph0_glonass_v2, s, &msg); break;
              case 1: super_t::convert(reader_t::eph1_glonass_v2, s, &msg); break;
              case 2:
                if(super_t::_version_type.version < 211){
                  //msg_glonass.freq_num; // TODO convert to value 1..24?
                }
                super_t::convert(reader_t::eph2_glonass_v2, s, &msg);
                break;
              case 3: super_t::convert(reader_t::eph3_glonass_v2, s, &msg); break;
            }
            buf << s << std::endl;
          }
          break;
        case 3:
          for(int i(0);
              i < ((super_t::_version_type.version <= 304) ? 4 : 5);
              ++i){
            std::string s(80, ' ');
            switch(i){
              case 0: super_t::convert(reader_t::eph0_glonass_v3, s, &msg); s[0] = 'R'; break;
              case 1: super_t::convert(reader_t::eph1_glonass_v3, s, &msg); break;
              case 2: super_t::convert(reader_t::eph2_glonass_v3, s, &msg); break;
              case 3: super_t::convert(reader_t::eph3_glonass_v3, s, &msg); break;
              case 4: super_t::convert(reader_t::eph4_glonass_v305, s, &msg); break;
            }
            buf << s << std::endl;
          }
          break;
      }
      dest << buf.str();
      return *this;
    }

  public:
    void set_version(
        const int &version,
        const super_t::version_type_t::sat_system_t &sys = super_t::version_type_t::SYS_GPS){
      super_t::set_version_type(typename super_t::version_type_t(
          version, super_t::version_type_t::FTYPE_NAVIGATION, sys));
    }

    struct space_node_list_t {
      const space_node_t *gps;
      const SBAS_SpaceNode<FloatT> *sbas;
      const space_node_t *qzss;
      const GLONASS_SpaceNode<FloatT> *glonass;
    };
    int write_all(
        const space_node_list_t &space_nodes,
        const int &version = 304){
      int res(-1);
      int systems(0);
      set_version(version, super_t::version_type_t::SYS_UNKNOWN);
      do{
        if(!space_nodes.gps){break;}
        ++systems;
        set_version(version, super_t::version_type_t::SYS_GPS);
        if(!space_nodes.gps->is_valid_iono_utc()){break;}
        switch(version / 100){
          case 2:
            if(_header["ION ALPHA"].entries() == 0){iono_alpha(*space_nodes.gps);}
            if(_header["ION BETA"].entries() == 0){iono_beta(*space_nodes.gps);}
            if(_header["DELTA-UTC: A0,A1,T,W"].entries() == 0){utc_params(*space_nodes.gps);}
            if(_header["LEAP SECONDS"].entries() == 0){leap_seconds(*space_nodes.gps);}
            break;
          case 3:
            if(_header["IONOSPHERIC CORR"].find("GPSA") == _header.end()){iono_alpha(*space_nodes.gps);}
            if(_header["IONOSPHERIC CORR"].find("GPSB") == _header.end()){iono_beta(*space_nodes.gps);}
            if(_header["TIME SYSTEM CORR"].find("GPUT") == _header.end()){utc_params(*space_nodes.gps);}
            if(_header["LEAP SECONDS"].entries() == 0){leap_seconds(*space_nodes.gps);}
            break;
        }
      }while(false);
      do{
        if(!space_nodes.sbas){break;}
        ++systems;
        set_version(version, super_t::version_type_t::SYS_SBAS);
      }while(false);
      do{
        if((version < 302) || (!space_nodes.qzss)){break;}
        ++systems;
        set_version(version, super_t::version_type_t::SYS_QZSS);
        if(!space_nodes.qzss->is_valid_iono_utc()){break;}
        switch(version / 100){
          case 3:
            if(_header["IONOSPHERIC CORR"].find("GPSA") == _header.end()){iono_alpha(*space_nodes.qzss);}
            if(_header["IONOSPHERIC CORR"].find("GPSB") == _header.end()){iono_beta(*space_nodes.qzss);}
            if(_header["TIME SYSTEM CORR"].find("QZUT") == _header.end()){utc_params(*space_nodes.qzss);}
            if(_header["LEAP SECONDS"].entries() == 0){leap_seconds(*space_nodes.qzss);}
            break;
        }
      }while(false);
      while(space_nodes.glonass){
        ++systems;
        set_version(version, super_t::version_type_t::SYS_GLONASS);
        typename GLONASS_SpaceNode<FloatT>::Satellite::eph_t latest(
            space_nodes.glonass->latest_ephemeris());
        if(latest.t_b_gps.week <= 0){break;}
        typename reader_t::iono_utc_t iono_utc = {0};
        iono_utc.t_ot = latest.t_b_gps.seconds;
        iono_utc.WN_t = latest.t_b_gps.week;
        iono_utc.delta_t_LS = (int)std::floor(0.5
            + typename space_node_t::gps_time_t(latest.c_tm_utc()).interval(latest.t_b_gps));
        switch(version / 100){
          case 2:
            if((_header["CORR TO SYSTEM TIME"].entries() == 0) && (latest.tau_c != 0)){
              std::tm t_tm(typename space_node_t::gps_time_t(iono_utc.WN_t, iono_utc.t_ot).c_tm());
              typename reader_t::t_corr_glonass_t t_corr_glonass = {
                t_tm.tm_year + 1900, t_tm.tm_mon + 1, t_tm.tm_mday, // year, month, day
                -latest.tau_c,
              };
              std::string s(60, ' ');
              super_t::convert(reader_t::t_corr_glonass_v2, s, &t_corr_glonass);
              _header["CORR TO SYSTEM TIME"] = s;
            }
            break;
          case 3:
            if((_header["TIME SYSTEM CORR"].find("GLUT") == _header.end()) && (latest.tau_c != 0)){
              std::string s(60, ' ');
              iono_utc.A0 = -latest.tau_c;
              super_t::convert(reader_t::utc_v3, s, &iono_utc);
              _header["TIME SYSTEM CORR"] << s.replace(0, 4, "GLUT", 4);
            }
            if((_header["TIME SYSTEM CORR"].find("GLGP") == _header.end()) && (latest.tau_GPS != 0)){
              std::string s(60, ' ');
              iono_utc.A0 = latest.tau_GPS;
              super_t::convert(reader_t::utc_v3, s, &iono_utc);
              _header["TIME SYSTEM CORR"] << s.replace(0, 4, "GLGP", 4);
            }
            break;
        }
        if((_header["LEAP SECONDS"].entries() == 0) && (iono_utc.delta_t_LS != 0)){
          // ver.3 can use ver.2 format with blank fields
          std::string s(60, ' ');
          super_t::convert(reader_t::utc_leap_v2, s, &iono_utc);
          _header["LEAP SECONDS"] = s;
        }
        break;
      }
      if(systems > 1){
        set_version(version, super_t::version_type_t::SYS_MIXED);
      }
      super_t::dest << header();
      res++;

      struct {
        RINEX_NAV_Writer &w;
        int &counter;
        bool gps, qzss;
        void operator()(const typename space_node_t::Satellite::Ephemeris &eph) {
          if(gps && (eph.svid <= 32)){
            w << message_t(eph);
          }else if(qzss && (eph.svid >= 193) && (eph.svid < 202)){
            w.dump(message_t::from_qzss(eph), true);
          }else{
            return;
          }
          counter++;
        }
        void operator()(const typename message_sbas_t::eph_t &eph) {
          w << message_sbas_t(eph);
          counter++;
        }
        void operator()(const typename message_glonass_t::eph_t &eph) {
          w << message_glonass_t(eph);
          counter++;
        }
      } functor = {*this, res, false, false};
      if(space_nodes.gps){
        functor.gps = true;
        if(space_nodes.gps == space_nodes.qzss){functor.qzss = true;}
        for(typename space_node_t::satellites_t::const_iterator
              it(space_nodes.gps->satellites().begin()), it_end(space_nodes.gps->satellites().end());
            it != it_end; ++it){
          it->second.each_ephemeris(
              functor,
              space_node_t::Satellite::eph_list_t::EACH_ALL_INVERTED);
        }
      }
      if(space_nodes.sbas){
        for(typename SBAS_SpaceNode<FloatT>::satellites_t::const_iterator
              it(space_nodes.sbas->satellites().begin()),
              it_end(space_nodes.sbas->satellites().end());
            it != it_end; ++it){
          it->second.each_ephemeris(
              functor,
              SBAS_SpaceNode<FloatT>::Satellite::eph_list_t::EACH_ALL_INVERTED);
        }
      }
      if((version >= 302) && (!functor.qzss) && (space_nodes.qzss)){
        functor.qzss = true;
        functor.gps = false;
        for(typename space_node_t::satellites_t::const_iterator
              it(space_nodes.qzss->satellites().begin()), it_end(space_nodes.qzss->satellites().end());
            it != it_end; ++it){
          it->second.each_ephemeris(
              functor,
              space_node_t::Satellite::eph_list_t::EACH_ALL_INVERTED);
        }
      }
      if(space_nodes.glonass){
        for(typename GLONASS_SpaceNode<FloatT>::satellites_t::const_iterator
              it(space_nodes.glonass->satellites().begin()),
              it_end(space_nodes.glonass->satellites().end());
            it != it_end; ++it){
          it->second.each_ephemeris(
              functor,
              GLONASS_SpaceNode<FloatT>::Satellite::eph_list_t::EACH_ALL_INVERTED);
        }
      }
      return res;
    }
    static int write_all(
        std::ostream &out,
        const space_node_list_t &space_nodes,
        const int &version = 304){
      return RINEX_NAV_Writer(out).write_all(space_nodes, version);
    }
    int write_all(const space_node_t &space_node, const int &version = 304){
      space_node_list_t list = {&space_node};
      return write_all(list, version);
    }
    static int write_all(std::ostream &out, const space_node_t &space_node, const int &version = 304){
      return RINEX_NAV_Writer(out).write_all(space_node, version);
    }
};
template <class FloatT>
const typename RINEX_Writer<>::header_item_t RINEX_NAV_Writer<FloatT>::default_header[] = {
    {"RINEX VERSION / TYPE",
        "     2              NAVIGATION DATA"},
    {"PGM / RUN BY / DATE", NULL}};
template <class FloatT>
const int RINEX_NAV_Writer<FloatT>::default_header_size
    = sizeof(RINEX_NAV_Writer<FloatT>::default_header) / sizeof(RINEX_NAV_Writer<FloatT>::default_header[0]);

template <class FloatT>
class RINEX_OBS_Writer : public RINEX_Writer<> {
  public:
    typedef typename RINEX_OBS<FloatT>::observation_t observation_t;
    typedef RINEX_OBS_Writer self_t;
    typedef RINEX_Writer<> super_t;
    typedef RINEX_OBS_Reader<FloatT> reader_t;
  protected:
    using super_t::RINEX_Float;
    using super_t::RINEX_FloatD;
    using super_t::RINEX_Value;
    using super_t::_header;
    using super_t::dest;
  public:
    void set_version(
        const int &version,
        const super_t::version_type_t::sat_system_t &sys = super_t::version_type_t::SYS_GPS){
      super_t::set_version_type(typename super_t::version_type_t(
          version, super_t::version_type_t::FTYPE_OBSERVATION, sys));
    }

    static const header_item_t default_header[];
    static const int default_header_size;
    void maker_name(
        const std::string &name){
      _header["MARKER NAME"] = name.substr(0, 60);
    }
    void observer_agency(
        const std::string &observer, const std::string &agency){
      // ex) "BILL SMITH          ABC INSTITUTE"
      std::stringstream ss;
      ss << std::setfill(' ') << std::left 
          << std::setw(20) << observer.substr(0, 20)
          << std::setw(20) << agency.substr(0, 20);
      _header["OBSERVER / AGENCY"] = ss.str();
    }
    void receiver_spec(
        const std::string &num, const std::string &type, const std::string &vers){
      // ex) "X1234A123           XX                 ZZZ"
      std::stringstream ss;
      ss << std::setfill(' ') << std::left 
          << std::setw(20) << num.substr(0, 20)
          << std::setw(20) << type.substr(0, 20)
          << std::setw(20) << vers.substr(0, 20);
      _header["REC # / TYPE / VERS"] = ss.str();
    }
    void antenna_spec(
        const std::string &num, const std::string &type){
      // ex) "234                 YY"
      std::stringstream ss;
      ss << std::setfill(' ') << std::left 
          << std::setw(20) << num.substr(0, 20)
          << std::setw(20) << type.substr(0, 20);
      _header["ANT # / TYPE"] = ss.str();
    }
    void wavelength_fact(
        const int l1, const int l2){
      // ex) "     1     1"
      std::stringstream ss;
      ss << RINEX_Value(l1, 6) << RINEX_Value(l2, 6);
      _header["WAVELENGTH FACT L1/2"] = ss.str();
    }
    void approx_position(
        const FloatT &x, const FloatT &y, const FloatT &z){
      std::stringstream ss;
      ss << RINEX_Float(x, 14, 4)
          << RINEX_Float(y, 14, 4)
          << RINEX_Float(z, 14, 4);
      _header["APPROX POSITION XYZ"] = ss.str();
    }
    void antenna_delta_hew(
        const FloatT &h, const FloatT &e, const FloatT &w){
      std::stringstream ss;
      ss << RINEX_Float(h, 14, 4)
          << RINEX_Float(e, 14, 4)
          << RINEX_Float(w, 14, 4);
      _header["ANTENNA: DELTA H/E/W"] = ss.str();
    }
    void types_of_obs(
        const char *type_list[], const int list_size){
      std::stringstream ss;
      ss << RINEX_Value(list_size, 6);
      for(int i(0); i < list_size; i++){
        ss << RINEX_Value(
            std::string(type_list[i]).substr(0, 6), 6);
      }
      _header["# / TYPES OF OBSERV"] = ss.str();
    }
    void interval(const FloatT &seconds){
      std::stringstream ss;
      ss << RINEX_Float(seconds, 6, 4);
      _header["INTERVAL"] = ss.str();
    }
    void interval(const int seconds){
      std::stringstream ss;
      ss << RINEX_Value(seconds, 6);
      _header["INTERVAL"] = ss.str();
    }
    void insert_header_time_item(
        const char *label, 
        const struct tm &t, const FloatT &rest_second = 0,
        const bool is_gps_time = true){
      std::stringstream ss;
      if(t.tm_year < 80){
        ss << RINEX_Value(2000 + t.tm_year, 6);
      }else{
        ss << RINEX_Value(1900 + t.tm_year, 6);
      }
      ss << RINEX_Value(t.tm_mon + 1, 6);
      ss << RINEX_Value(t.tm_mday, 6);
      ss << RINEX_Value(t.tm_hour, 6);
      ss << RINEX_Value(t.tm_min, 6);
      ss << RINEX_Float(rest_second + t.tm_sec, 12, 6);
      if(is_gps_time){
        ss << RINEX_Value("GPS", 9);
      }
      _header[label] = ss.str();
    }
    void insert_header_time_item(
        const char *label, const GPS_Time<FloatT> &t){
      FloatT sec_f(t.seconds), sec_i;
      sec_f = std::modf(sec_f, &sec_i);
      insert_header_time_item(label, 
          t.c_tm(), sec_f, true);
    }
    void first_obs(
        const struct tm &t, const double rest_second = 0, 
        const bool is_gps_time = true){
      insert_header_time_item("TIME OF FIRST OBS", 
          t, rest_second, is_gps_time);
    }
    void first_obs(const GPS_Time<FloatT> &t){
      insert_header_time_item("TIME OF FIRST OBS", t);
    }
    void last_obs(
        const struct tm &t, const double rest_second = 0, 
        const bool is_gps_time = true){
      insert_header_time_item("TIME OF LAST OBS",
          t, rest_second, is_gps_time);
    }
    void last_obs(const GPS_Time<FloatT> &t){
      insert_header_time_item("TIME OF LAST OBS", t);
    }
    RINEX_OBS_Writer(std::ostream &out)
        : super_t(out, default_header, default_header_size) {}
    ~RINEX_OBS_Writer(){}
    self_t &operator<<(const observation_t &obs){
      if(obs.per_satellite.size() <= 0){return *this;}
      
      typedef typename observation_t::per_satellite_t per_satellite_t;
      
      std::stringstream top, rest;
      
      std::string buf(80, ' ');
      int ver_major(super_t::_version_type.version / 100);
      { // epoch
        typename RINEX_OBS<FloatT>::epoch_flag_t epoch_flag(obs);
        epoch_flag.items_followed = obs.per_satellite.size(); // Num. of satellites
        switch(ver_major){
          case 2: super_t::convert(reader_t::epoch_flag_v2, buf, &epoch_flag); break;
          case 3: super_t::convert(reader_t::epoch_flag_v3, buf, &epoch_flag); buf[0] = '>'; break;
          default: return *this;
        }
      }
      
      int i(0);
      for(typename per_satellite_t::const_iterator
            it(obs.per_satellite.begin()), it_end(obs.per_satellite.end());
          it != it_end; ++it, ++i){ // Enumerate satellites
        
        // If satellites are more than 12, then use the next line
        if((i == 12) && (ver_major == 2)){
          i = 0;
          top << buf << std::endl;
          buf = std::string(80, ' ');
        }
        
        // Write PRN number to list
        {
          int serial(it->first), prn;
          char sys(reader_t::serial2sys(serial, prn));
          std::string sat_str(3, ' ');
          sat_str[0] = sys;
          reader_t::template conv_t<int>::d(sat_str, 1, 2, &prn, 1, false);
          switch(ver_major){
            case 2: buf.replace(32 + i * 3, 3, sat_str); break;
            case 3: rest << sat_str; break;
          }
        }

        // Observation data
        int i2(5);
        for(typename per_satellite_t::mapped_type::const_iterator it2(it->second.begin()),
              it2_end(it->second.end());
            it2 != it2_end; ++it2){
          if((ver_major == 2) && (i2-- == 0)){
            i2 = 4;
            rest << std::endl;
          }
          std::string buf2(16, ' ');
          if(it2->valid){
            super_t::convert(reader_t::record_v2v3, buf2, &(*it2));
            if(it2->lli == 0){buf2[14] = ' ';}
            if(it2->ss == 0){buf2[15] = ' ';}
          }
          rest << buf2;
        }
        if(ver_major == 2){
          rest << std::string(16 * i2, ' ');
        }
        rest << std::endl;
      }
      top << buf << std::endl;
      
      dest << top.str() << rest.str();
      return *this;
    }
};
template <class FloatT>
const typename RINEX_Writer<>::header_item_t RINEX_OBS_Writer<FloatT>::default_header[] = {
    {"RINEX VERSION / TYPE",
        "     2              OBSERVATION DATA"},
    {"PGM / RUN BY / DATE", 
        "XXRINEXO V9.9       AIUB                12-SEP-90 12:43"},
    {"COMMENT", NULL},
    {"MARKER NAME",  "A 9080"},
    {"MARKER NUMBER", NULL},
    {"OBSERVER / AGENCY", 
        "BILL SMITH          ABC INSTITUTE"},
    {"REC # / TYPE / VERS", 
        "X1234A123           XX                 ZZZ"},
    {"ANT # / TYPE", "234                 YY"},
    {"APPROX POSITION XYZ", 
        " -3947762.7496  3364399.8789  3699428.5111"},
    {"ANTENNA: DELTA H/E/W", 
        "        0.0000        0.0000        0.0000"},
    {"WAVELENGTH FACT L1/2", 
        "     1     1"},
    {"# / TYPES OF OBSERV", 
        "     4    P1    L1    L2    P2"},
    {"INTERVAL", "     1"},
    {"TIME OF FIRST OBS", "  1990     3    24    13   10    36.000000"},
    {"TIME OF LAST OBS", NULL},
    {"# OF SATELLITES", NULL},
    {"PRN / # OF OBS", NULL}};
template <class FloatT>
const int RINEX_OBS_Writer<FloatT>::default_header_size
    = sizeof(RINEX_OBS_Writer<FloatT>::default_header) / sizeof(RINEX_OBS_Writer<FloatT>::default_header[0]);

#endif //  __RINEX_H__
