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

#include "GPS.h"

template <class U = void>
class RINEX_Reader {
  public:
    typedef RINEX_Reader<U> self_t;
    typedef std::multimap<std::string, std::string> header_t;
    
  protected:
    header_t _header;
    std::istream &src;
    bool _has_next;
    int version;
    enum {
      RINEX_FILE_UNKNOWN = 0,
      RINEX_FILE_OBSERVATION,
      RINEX_FILE_NAVIGATION,
      RINEX_FILE_METEOROLOGICAL,
    } file_type;
    enum {
      RINEX_SYS_UNKNOWN = 0,
      RINEX_SYS_GPS,
      RINEX_SYS_GLONASS,
      RINEX_SYS_GALILEO,
      RINEX_SYS_QZSS,
      RINEX_SYS_BDS,
      RINEX_SYS_IRNSS,
      RINEX_SYS_SBAS,
      RINEX_SYS_TRANSIT,
      RINEX_SYS_MIXED,
    } sat_system;
    
  public:
    RINEX_Reader(
        std::istream &in,
        std::string &(*modify_header)(std::string &, std::string &) = NULL)
        : src(in), _has_next(false),
        version(0), file_type(RINEX_FILE_UNKNOWN), sat_system(RINEX_SYS_UNKNOWN) {
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
        _header.insert(header_t::value_type(label, content));
      }

      // version and type extraction
      for(const header_t::const_iterator it(_header.find("RINEX VERSION / TYPE"));
          it != _header.end(); ){
        version = std::atoi(it->second.substr(0, 6).c_str()) * 100
            + std::atoi(it->second.substr(7, 2).c_str());
        switch(it->second[20]){
          case 'O': file_type = RINEX_FILE_OBSERVATION;    break;
          case 'N': file_type = RINEX_FILE_NAVIGATION;     break;
          case 'M': file_type = RINEX_FILE_METEOROLOGICAL; break;
        }
        if((file_type == RINEX_FILE_NAVIGATION) || (file_type == RINEX_FILE_OBSERVATION)){
          switch(it->second[40]){
            case 'G': sat_system = RINEX_SYS_GPS;     break;
            case 'R': sat_system = RINEX_SYS_GLONASS; break;
            case 'E': sat_system = RINEX_SYS_GALILEO; break;
            case 'J': sat_system = RINEX_SYS_QZSS;    break;
            case 'C': sat_system = RINEX_SYS_BDS;     break;
            case 'I': sat_system = RINEX_SYS_IRNSS;   break;
            case 'S': sat_system = RINEX_SYS_SBAS;    break;
            case 'M': sat_system = RINEX_SYS_MIXED;   break;
            case 'T': sat_system = RINEX_SYS_TRANSIT; break; // ver.2 only
          }
        }
        break;
      }

    }
    virtual ~RINEX_Reader(){_header.clear();}
    header_t &header() {return _header;}
    const header_t &header() const {return const_cast<self_t *>(this)->header();}
    bool has_next() const {return _has_next;}

    template <class T>
    struct conv_t {
      static void d(
          std::string &buf, const int &offset, const int &length, void *value, const int &opt = 0, const bool &str2val = true){
        if(str2val){
          std::stringstream(buf.substr(offset, length)) >> *(T *)value;
        }else{
          std::stringstream ss;
          ss << std::setfill(' ') << std::right << std::setw(length) << *(T *)value;
          buf.replace(offset, length, ss.str());
        }
      }
      static void f(
          std::string &buf, const int &offset, const int &length, void *value, const int &precision = 0, const bool &str2val = true){
        if(str2val){
          std::stringstream(buf.substr(offset, length)) >> *(T *)value;
        }else{
          std::stringstream ss;
          ss << std::setfill(' ') << std::right << std::setw(length)
              << std::setprecision(precision) << std::fixed
              << *(T *)value;
          buf.replace(offset, length, ss.str());
        }
      }
      static void e(
          std::string &buf, const int &offset, const int &length, void *value, const int &precision = 0, const bool &str2val = true){
        if(str2val){
          std::string s(buf.substr(offset, length));
          int pos(s.find("D"));
          if(pos != std::string::npos){
            s.replace(pos, 1, "E");
          }
          std::stringstream(s) >> *(T *)value;
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
        }
      }
    };

    struct convert_item_t {
      void (*func)(
          std::string &buf, const int &offset, const int &length, void *value,
          const int &opt, const bool &str2val);
      int offset;
      int length;
      int value_offset;
      int opt;
    };

    static void convert(const convert_item_t *items, const int &size, const std::string &buf, void *values){
      // str => value
      for(int i(0); i < size; ++i){
        (*items[i].func)(
            const_cast<std::string &>(buf), items[i].offset, items[i].length, (char *)values + items[i].value_offset,
            items[i].opt, true);
      }
    }
    template <int N>
    static inline void convert(const convert_item_t (&items)[N], const std::string &buf, void *values){
      convert(items, N, buf, values);
    }
};

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
    FloatT ura_meter;
    FloatT SV_health_f;
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
        ura_meter(ephemeris_t::URA_meter(eph.URA)),
        SV_health_f(((eph.SV_health & 0x20) && (eph.SV_health & 0x1F == 0)) ? 1 : eph.SV_health),
        t_ot(0), // TODO
        fit_interval_hr(eph.fit_interval / (60 * 60)),
        dummy(0) {
    }
    void update() {
      typename space_node_t::gps_time_t t_oc(t_oc_tm);
      t_oc += (t_oc_sec - t_oc_tm.tm_sec);
      eph.WN = t_oc.week;
      eph.t_oc = t_oc.seconds;

      eph.URA = ephemeris_t::URA_index(ura_meter); // meter to index

      /* @see ftp://igs.org/pub/data/format/rinex210.txt
       * 6.7 Satellite Health
       * RINEX Value:   0    Health OK
       * RINEX Value:   1    Health not OK (bits 18-22 not stored)
       * RINEX Value: >32    Health not OK (bits 18-22 stored)
       */
      eph.SV_health = (unsigned int)SV_health_f;
      if(eph.SV_health > 32){
        eph.SV_health &= 0x3F; // 0b111111
      }else if(eph.SV_health > 0){
        eph.SV_health = 0x20; // 0b100000
      }

      // At least 4 hour validity, then, hours => seconds;
      eph.fit_interval = ((fit_interval_hr < 4) ? 4 : fit_interval_hr) * 60 * 60;
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
    typedef typename space_node_t::Ionospheric_UTC_Parameters iono_utc_t;

    static const typename super_t::convert_item_t eph0_v2[10], eph0_v3[10];
    static const typename super_t::convert_item_t eph1_v2[4], eph1_v3[4];
    static const typename super_t::convert_item_t eph2_v2[4], eph2_v3[4];
    static const typename super_t::convert_item_t eph3_v2[4], eph3_v3[4];
    static const typename super_t::convert_item_t eph4_v2[4], eph4_v3[4];
    static const typename super_t::convert_item_t eph5_v2[4], eph5_v3[4];
    static const typename super_t::convert_item_t eph6_v2[4], eph6_v3[4];
    static const typename super_t::convert_item_t eph7_v2[2], eph7_v3[2];

  protected:
    message_t msg;
    
    void seek_next_v2() {
      char buf[256];

      for(int i = 0; (i < 8) && (super_t::src.good()); i++){
        if(super_t::src.getline(buf, sizeof(buf)).fail()){return;}
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
      super_t::_has_next = true;
    }

    void seek_next_v3() {
      char buf[256];

      for(int i = 0; (i < 8) && (super_t::src.good()); i++){
        if(super_t::src.getline(buf, sizeof(buf)).fail()){return;}
        std::string line(buf);

        switch(i){
          case 0: {
            // if(line_data[0] != 'G'){} // TODO check GPS before parsing
            super_t::convert(eph0_v3, line, &msg);
            msg.t_oc_tm.tm_year = msg.t_oc_year4 - 1900; // tm_year base is 1900
            msg.t_oc_tm.tm_mon = msg.t_oc_mon12 - 1; // month [0, 11]
            msg.t_oc_sec = msg.t_oc_tm.tm_sec;
            break;
          }
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
      super_t::_has_next = true;
    }

    void seek_next() {
      super_t::version >= 300 ? seek_next_v3() : seek_next_v2();
    }

  public:
    RINEX_NAV_Reader(std::istream &in) : super_t(in) {
      seek_next();
    }
    ~RINEX_NAV_Reader(){}
    
    typename space_node_t::Satellite::Ephemeris next() {
      typename space_node_t::Satellite::Ephemeris current(msg.eph);
      super_t::_has_next = false;
      seek_next();
      return current;
    }
    
    static const typename super_t::convert_item_t iono_alpha_v2[4];
    static const typename super_t::convert_item_t iono_beta_v2[4];
    static const typename super_t::convert_item_t utc_v2[4];
    static const typename super_t::convert_item_t utc_leap[1];

    /**
     * Obtain ionospheric delay coefficient and UTC parameters.
     * 
     * @return true when successfully obtained, otherwise false
     */
    bool extract_iono_utc(GPS_SpaceNode<FloatT> &space_node) const {
      iono_utc_t iono_utc;
      bool alpha, beta, utc, leap;
      super_t::header_t::const_iterator it;

      if(alpha = ((it = _header.find("ION ALPHA")) != _header.end())){
        super_t::convert(iono_alpha_v2, it->second, &iono_utc);
      }
      
      if(beta = ((it = _header.find("ION BETA")) != _header.end())){
        super_t::convert(iono_beta_v2, it->second, &iono_utc);
      }

      if(utc = ((it = _header.find("DELTA-UTC: A0,A1,T,W")) != _header.end())){
        super_t::convert(utc_v2, it->second, &iono_utc);
      }

      if(leap = ((it = _header.find("LEAP SECONDS")) != _header.end())){
        super_t::convert(utc_leap, it->second, &iono_utc);
      }

      space_node.update_iono_utc(iono_utc, alpha && beta, utc && leap);
      return alpha && beta && utc && leap;
    }

    static const typename super_t::convert_item_t iono_alpha_v3[4];
    static const typename super_t::convert_item_t iono_beta_v3[4];
    static const typename super_t::convert_item_t utc_v3[4];

    bool extract_iono_utc_v3(GPS_SpaceNode<FloatT> &space_node) const {
      iono_utc_t iono_utc;
      bool alpha(false), beta(false), utc(false), leap(false);
      typedef super_t::header_t::const_iterator it_t;

      {
        std::pair<it_t, it_t> range(_header.equal_range("IONOSPHERIC CORR"));
        for(it_t it(range.first); it != range.second; ++it){
          if(it->second.find("GPSA") != it->second.npos){
            super_t::convert(iono_alpha_v3, it->second, &iono_utc);
            alpha = true;
          }else if(it->second.find("GPSB") != it->second.npos){
            super_t::convert(iono_beta_v3, it->second, &iono_utc);
            beta = true;
          }
        }
      }

      {
        std::pair<it_t, it_t> range(_header.equal_range("TIME SYSTEM CORR"));
        for(it_t it(range.first); it != range.second; ++it){
          if(it->second.find("GPUT") == it->second.npos){continue;}
          super_t::convert(utc_v3, it->second, &iono_utc);
          utc = true;
        }
      }

      {
        it_t it(_header.find("LEAP SECONDS"));
        if(it != _header.end()){
          super_t::convert(utc_leap, it->second, &iono_utc);
          leap = true;
        }
      }

      space_node.update_iono_utc(iono_utc, alpha && beta, utc && leap);
      return alpha && beta && utc && leap;
    }

    static int read_all(std::istream &in, space_node_t &space_node){
      int res(-1);
      RINEX_NAV_Reader reader(in);
      (reader.version >= 300)
          ? reader.extract_iono_utc_v3(space_node)
          : reader.extract_iono_utc(space_node);
      res++;
      for(; reader.has_next(); ++res){
        typename space_node_t::Satellite::Ephemeris eph(reader.next());
        space_node.satellite(eph.svid).register_ephemeris(eph);
      }
      return res;
    }
};

#define GEN_D(offset, length, container_type, container_member, value_type) \
    {super_t::template conv_t<value_type>::d, offset, length, \
      offsetof(container_type, container_member)}
#define GEN_F(offset, length, precision, container_type, container_member) \
    {super_t::template conv_t<typename space_node_t::float_t>::f, offset, length, \
      offsetof(container_type, container_member), precision}
#define GEN_E(offset, length, precision, container_type, container_member) \
    {super_t::template conv_t<typename space_node_t::float_t>::e, offset, length, \
      offsetof(container_type, container_member), precision}

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
  GEN_D( 1,  2,     message_t, eph.svid, int),
  GEN_D( 4,  4,     message_t, t_oc_year4,      int),
  GEN_D( 9,  2,     message_t, t_oc_mon12,      int),
  GEN_D(12,  2,     message_t, t_oc_tm.tm_mday, int),
  GEN_D(15,  2,     message_t, t_oc_tm.tm_hour, int),
  GEN_D(18,  2,     message_t, t_oc_tm.tm_min,  int),
  GEN_D(21,  2,     message_t, t_oc_tm.tm_sec,  int),
  GEN_E(23, 19, 12, message_t, eph.a_f0),
  GEN_E(42, 19, 12, message_t, eph.a_f1),
  GEN_E(61, 19, 12, message_t, eph.a_f2),
};

template <class FloatT>
const typename RINEX_NAV_Reader<FloatT>::convert_item_t RINEX_NAV_Reader<FloatT>::eph1_v2[] = {
  GEN_E( 3, 19, 12, message_t, eph.iode),
  GEN_E(22, 19, 12, message_t, eph.c_rs),
  GEN_E(41, 19, 12, message_t, eph.delta_n),
  GEN_E(60, 19, 12, message_t, eph.M0),
};
template <class FloatT>
const typename RINEX_NAV_Reader<FloatT>::convert_item_t RINEX_NAV_Reader<FloatT>::eph1_v3[] = {
  GEN_E( 4, 19, 12, message_t, eph.iode),
  GEN_E(23, 19, 12, message_t, eph.c_rs),
  GEN_E(42, 19, 12, message_t, eph.delta_n),
  GEN_E(61, 19, 12, message_t, eph.M0),
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
  GEN_E( 3, 19, 12, message_t, ura_meter),
  GEN_E(22, 19, 12, message_t, SV_health_f),
  GEN_E(41, 19, 12, message_t, eph.t_GD),
  GEN_E(60, 19, 12, message_t, eph.iodc),
};
template <class FloatT>
const typename RINEX_NAV_Reader<FloatT>::convert_item_t RINEX_NAV_Reader<FloatT>::eph6_v3[] = {
  GEN_E( 4, 19, 12, message_t, ura_meter),
  GEN_E(23, 19, 12, message_t, SV_health_f),
  GEN_E(42, 19, 12, message_t, eph.t_GD),
  GEN_E(61, 19, 12, message_t, eph.iodc),
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
const typename RINEX_NAV_Reader<FloatT>::convert_item_t RINEX_NAV_Reader<FloatT>::utc_leap[] = {
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
  GEN_F(  5, 17, 10, iono_utc_t, A0),
  GEN_F( 22, 16,  9, iono_utc_t, A1),
  GEN_D( 39,  6,     iono_utc_t, t_ot, int),
  GEN_D( 46,  4,     iono_utc_t, WN_t, int),
};

#undef GEN_D
#undef GEN_F
#undef GEN_E

template <class FloatT>
struct RINEX_OBS {
  struct ObservedItem {
    GPS_Time<FloatT> t_epoc;
    unsigned event_flag;
    FloatT receiver_clock_error;
    typedef struct {
      FloatT observed;
      unsigned lli, ss;
    } data_t;
    typedef std::vector<data_t> data_1sat_t;
    typedef std::map<int, data_1sat_t> sat_data_t;
    sat_data_t sat_data;
  };
};

template <class FloatT = double>
class RINEX_OBS_Reader : public RINEX_Reader<> {
  protected:
    typedef RINEX_OBS<FloatT> content_t;
    typedef RINEX_OBS_Reader<FloatT> self_t;
    typedef RINEX_Reader<> super_t;
  public:
    typedef typename content_t::ObservedItem ObservedItem;
  
  protected:
    std::vector<std::string> types_of_observe;
    ObservedItem item;
    static std::string &modify_header(std::string &label, std::string &content){
      return content;
    }
    
    void seek_next() {
      if(types_of_observe.size() == 0){return;}
      
      char buf[256];
      std::queue<int> sat_list;
      item.sat_data.clear();
      
      while(true){
      
        // Read lines for epoch
        {
          unsigned num_of_followed_data(0);
          
          if(super_t::src.getline(buf, sizeof(buf)).fail()){return;}
          std::string data_line(buf);
          
          std::stringstream(data_line.substr(26, 3)) >> item.event_flag;
          std::stringstream(data_line.substr(29, 3)) >> num_of_followed_data;  // Satellite(flag = 0/1) or Line number(except for  0/1)
          
          if(item.event_flag >= 2){
            while(num_of_followed_data--){
              if(super_t::src.getline(buf, sizeof(buf)).fail()){return;}
            }
            continue;
          }
          
          std::stringstream data(data_line);
          
          // Epoch time
          struct tm t;
          data >> t.tm_year; // year - 1900
          if(t.tm_year < 80){t.tm_year += 100;} // greater than 1980
          data >> t.tm_mon;  // month
          --(t.tm_mon);
          data >> t.tm_mday; // day
          data >> t.tm_hour; // hour
          data >> t.tm_min;  // minute
          t.tm_sec = 0;
          item.t_epoc = GPS_Time<FloatT>(t);
          FloatT v;
          data >> v; // •b
          item.t_epoc += v;
          
          // Receiver clock error
          std::stringstream(data_line.substr(68)) >> item.receiver_clock_error;
          
          int prn;
          for(int i(0); num_of_followed_data > 0; i++, num_of_followed_data--){
            if(i == 12){  // if satellites are more than 12, move to the next line
              if(super_t::src.getline(buf, sizeof(buf)).fail()){return;}
              data_line = std::string(buf);
              i = 0;
            }
            std::stringstream(data_line.substr(33 + (i * 3), 2)) >> prn;
            switch(data_line[32 + (i * 3)]){
              case ' ':
              case 'G': 
                break;  // NAVSTAR
              case 'R':
                prn += 200;  break;  // GLONASS
              case 'S':
                prn += 100;  break;  // SBAS
              default:
                prn += 300;
            }
            sat_list.push(prn);
            //std::cerr << prn << std::endl;
          }
        }
      
        // Observation data per satellite
        while(!sat_list.empty()){
          int prn(sat_list.front());
          sat_list.pop();
          item.sat_data[prn];
          std::string data_line;
          for(int i(0); i < types_of_observe.size(); i++){
            int offset_index(i % 5);
            if(offset_index == 0){
              if(super_t::src.getline(buf, sizeof(buf)).fail()){return;}
              data_line = std::string(buf);
            }
            typename ObservedItem::data_t data;
            std::string s(data_line.substr(offset_index * 16, 16));
            std::stringstream ss(s);
            ss >> data.observed;
            data.lli = data.ss = 0;
            if(s.size() >= 15){unsigned i(s[14] - '0'); if(i < 10){data.lli = i;}}
            if(s.size() >= 16){unsigned i(s[15] - '0'); if(i < 10){data.ss = i;}}
            //std::cerr << data.observed << ", " << data.lli << "," << data.ss << std::endl;
            item.sat_data[prn].push_back(data);
          }
        }
        
        break;
      }
      
      super_t::_has_next = true;
    }
  
  public:
    RINEX_OBS_Reader(std::istream &in)
        : super_t(in, self_t::modify_header),
          types_of_observe() {
      super_t::header_t::const_iterator it(super_t::_header.find("# / TYPES OF OBSERV"));
      if(it != super_t::_header.end()){
        std::stringstream data(it->second);
        unsigned num_types_of_observe; 
        data >> num_types_of_observe;
        while(num_types_of_observe){
          std::string param_name;
          data >> param_name;
          if(!data.good()){break;}
          types_of_observe.push_back(param_name);
          num_types_of_observe--;
        }
        if(num_types_of_observe == 0){
          seek_next();
        }else{
          types_of_observe.clear();
        }
      }
    }
    ~RINEX_OBS_Reader(){}
    
    ObservedItem next() {
      ObservedItem current(item);
      super_t::_has_next = false;
      seek_next();
      return current;
    }
    /**
     * Return index on data lines corresponding to specified label
     * If not found, return -1.
     * 
     */
    int observed_index(const std::string &label) const {
      int res(distance(types_of_observe.begin(), 
          find(types_of_observe.begin(), types_of_observe.end(), label)));
      return (res >= types_of_observe.size() ? -1 : res);
    }
    int observed_index(const char *label) const {
      return observed_index(std::string(label));
    }
};

template <class U = void>
class RINEX_Writer {
  public:
    typedef struct {const char *key, *value;} header_item_t;
    
    struct header_t : public std::vector<std::pair<std::string, std::string> > {
      typedef std::vector<std::pair<std::string, std::string> > super_t;

      using super_t::operator[];
      typename super_t::value_type::second_type operator[]( // mimic of std::map::operator[]
          const typename super_t::value_type::first_type &key) const {
        for(typename super_t::const_iterator it(super_t::begin()), it_end(super_t::end());
            it != it_end; ++it){
          if(it->first == key){return it->second;}
        }
        return typename super_t::value_type::second_type();
      }
      typename super_t::value_type::second_type &operator[]( // mimic of std::map::operator[]=
          const typename super_t::value_type::first_type &key){
        for(typename super_t::iterator it(super_t::begin()), it_end(super_t::end());
            it != it_end; ++it){
          if(it->first == key){return it->second;}
        }
        super_t::push_back(typename super_t::value_type(
            key, typename super_t::value_type::second_type()));
        return super_t::back().second;
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
        for(header_t::const_iterator it(header.begin()), it_end(header.end());
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
    header_t _header;
    std::ostream &dist;
    
  public:
    typedef RINEX_Writer<U> self_t;
    RINEX_Writer(
        std::ostream &out,
        const header_item_t *header_mask = NULL, 
        const int header_mask_size = 0) 
        : _header(header_mask, header_mask_size), dist(out) {
      
    }
    virtual ~RINEX_Writer() {_header.clear();}

    header_t &header(){return _header;}
    const header_t &header() const {return const_cast<self_t *>(this)->header();}
    
    template <class FloatT>
    static std::string RINEX_Float(
        const FloatT &value, const int width = 19, const int precision = 12){
      std::string s;
      RINEX_Reader<U>::template conv_t<FloatT>::f(s, 0, width, &const_cast<FloatT &>(value), precision, false);
      return s;
    }
    template <class FloatT>
    static std::string RINEX_FloatD(
        const FloatT &value, const int width = 19, const int precision = 12){
      std::string s;
      RINEX_Reader<U>::template conv_t<FloatT>::e(s, 0, width, &const_cast<FloatT &>(value), precision, false);
      return s;
    }
    template <class T>
    static std::string RINEX_Value(const T &value, const int width = 6){
      std::string s;
      RINEX_Reader<U>::template conv_t<T>::d(s, 0, width, &const_cast<T &>(value), 0, false);
      return s;
    }
    
    static void convert(
        const typename RINEX_Reader<U>::convert_item_t *items, const int &size,
        std::string &buf, const void *values){
      // value => string
      for(int i(0); i < size; ++i){
        (*items[i].func)(
            buf, items[i].offset, items[i].length, (char *)(const_cast<void *>(values)) + items[i].value_offset,
            items[i].opt, false);
      }
    }
    template <int N>
    static inline void convert(
        const typename RINEX_Reader<U>::convert_item_t (&items)[N], std::string &buf, const void *values){
      convert(items, N, buf, values);
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
    using super_t::dist;
  public:
    typedef typename RINEX_NAV<FloatT>::space_node_t space_node_t;
    typedef typename RINEX_NAV<FloatT>::message_t message_t;

    static const typename super_t::header_item_t default_header[];
    static const int default_header_size;
    void iono_alpha(const space_node_t &space_node){
      std::string s(60, ' ');
      super_t::convert(reader_t::iono_alpha_v2, s, &space_node.iono_utc());
      _header["ION ALPHA"] = s;
    }
    void iono_beta(const space_node_t &space_node){
      std::string s(60, ' ');
      super_t::convert(reader_t::iono_beta_v2, s, &space_node.iono_utc());
      _header["ION BETA"] = s;
    }
    void utc_params(const space_node_t &space_node){
      std::string s(60, ' ');
      super_t::convert(reader_t::utc_v2, s, &space_node.iono_utc());
      _header["DELTA UTC: A0,A1,T,W"] = s;
    }
    void leap_seconds(const space_node_t &space_node){
      std::string s(60, ' ');
      super_t::convert(reader_t::utc_leap, s, &space_node.iono_utc());
      _header["LEAP SECONDS"] = s;
    }
    RINEX_NAV_Writer(std::ostream &out)
        : super_t(out, default_header, default_header_size) {}
    ~RINEX_NAV_Writer(){}
    self_t &operator<<(const message_t &msg){
      std::stringstream buf;
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
      dist << buf.str();
      return *this;
    }

  protected:
    struct WriteAllFunctor {
      RINEX_NAV_Writer &w;
      int &counter;
      void operator()(const typename space_node_t::Satellite::Ephemeris &eph) {
        w << message_t(eph);
        counter++;
      }
    };

  public:
    static int write_all(std::ostream &out, const space_node_t &space_node){
      int res(-1);
      RINEX_NAV_Writer writer(out);
      if(!space_node.is_valid_iono_utc()){return res;}
      {
        writer.iono_alpha(space_node);
        writer.iono_beta(space_node);
        writer.utc_params(space_node);
        writer.leap_seconds(space_node);
      }
      out << writer.header();
      res++;

      WriteAllFunctor functor = {writer, res};
      for(typename space_node_t::satellites_t::const_iterator
            it(space_node.satellites().begin()), it_end(space_node.satellites().end());
          it != it_end; ++it){
        it->second.each_ephemeris(
            functor,
            space_node_t::Satellite::eph_list_t::EACH_ALL_INVERTED);
      }
      return res;
    }
};
template <class FloatT>
const typename RINEX_Writer<>::header_item_t RINEX_NAV_Writer<FloatT>::default_header[] = {
    {"RINEX VERSION / TYPE",
        "     2              NAVIGATION DATA"},
    {"COMMENT", NULL},
    {"ION ALPHA", NULL},
    {"ION BETA", NULL},
    {"DELTA UTC: A0,A1,T,W", NULL},
    {"LEAP SECONDS", NULL}};
template <class FloatT>
const int RINEX_NAV_Writer<FloatT>::default_header_size
    = sizeof(RINEX_NAV_Writer<FloatT>::default_header) / sizeof(RINEX_NAV_Writer<FloatT>::default_header[0]);

template <class FloatT>
class RINEX_OBS_Writer : public RINEX_Writer<> {
  public:
    typedef RINEX_OBS<FloatT> content_t;
    typedef RINEX_OBS_Writer self_t;
    typedef RINEX_Writer<> super_t;
  protected:
    using super_t::RINEX_Float;
    using super_t::RINEX_FloatD;
    using super_t::RINEX_Value;
    using super_t::_header;
    using super_t::dist;
  public:
    static const header_item_t default_header[];
    static const int default_header_size;
    void pgm_runby_date(
        const std::string &pgm, const std::string &runby,
        const struct tm &t){
      // ex) "XXRINEXO V9.9       AIUB                12-SEP-90 12:43"
      char buf[20] = {0};
      std::strftime(buf, sizeof(buf) - 1, "%d-%b-%y %H:%M", &t);
      std::stringstream ss;
      ss << std::setfill(' ') << std::left 
          << std::setw(20) << pgm.substr(0, 20)
          << std::setw(20) << runby.substr(0, 20)
          << buf;
      _header["PGM / RUN BY / DATE"] = ss.str();
    }
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
    self_t &operator<<(const typename content_t::ObservedItem &data){
      
      typedef 
          typename content_t::ObservedItem::sat_data_t
          sat_data_t;
      
      std::stringstream top, rest;
      
      { // Treat time
        struct tm t(data.t_epoc.c_tm());
        FloatT sec_f(data.t_epoc.seconds), sec_i;
        sec_f = std::modf(sec_f, &sec_i);
        t.tm_year %= 100;
        top << ((t.tm_year < 10) ? " 0" : " ") << t.tm_year
            << RINEX_Value(t.tm_mon + 1, 3)
            << RINEX_Value(t.tm_mday, 3)
            << RINEX_Value(t.tm_hour, 3)
            << RINEX_Value(t.tm_min, 3)
            << RINEX_Float(sec_f + t.tm_sec, 11, 7);
      }
      
      top << RINEX_Value(0, 3); // Epoch flag
      top << RINEX_Value(data.sat_data.size(), 3); // Num. of satellites
      
      int index(0);
      for(typename sat_data_t::const_iterator it(data.sat_data.begin());
          it != data.sat_data.end();
          ++it, ++index){ // Enumerate satellites
        
        // If satellites are more than 12, then use the next line
        if(index == 12){
          // Reception clock error
          top << RINEX_Float(data.receiver_clock_error, 12, 9);
        }
        if((index % 12 == 0) && (index > 0)){
          top << std::endl << std::string(32, ' ');
        }
        
        // Write PRN number to list
        int prn(it->first);
        if(prn < 100){
          top << 'G';
        }else if(prn < 200){
          top << 'S';
        }else if(prn < 300){
          top << 'R';
        }else{
          top << ' ';
        }
        top << RINEX_Value(prn % 100, 2);
        
        // Observation data
        int index2(0);
        for(typename sat_data_t::mapped_type::const_iterator it2(it->second.begin());
            it2 != it->second.end();
            ++it2, ++index2){
          if((index2 % 5 == 0) && (index2 > 0)){
            rest << std::endl;
          }
          rest << RINEX_Float(it2->observed, 14, 3);
          if(it2->lli){rest << (it2->lli % 10);}else{rest << ' ';}
          if(it2->ss){rest << (it2->ss % 10);}else{rest << ' ';}
        }
        rest << std::endl;
      }
      if(index <= 12){
        top << std::string((12 - index) * 3, ' ');
        // Reception clock error
        top << RINEX_Float(data.receiver_clock_error, 12, 9);
      }
      top << std::endl;
      
      dist << top.str() << rest.str();
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
