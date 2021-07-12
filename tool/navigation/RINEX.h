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
    
  public:
    struct version_type_t {
      int version;
      enum file_type_t {
        FTYPE_UNKNOWN = 0,
        FTYPE_OBSERVATION,
        FTYPE_NAVIGATION,
        FTYPE_METEOROLOGICAL,
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
        _header.insert(header_t::value_type(label, content));
      }

      // version and type extraction
      for(const header_t::const_iterator it(_header.find("RINEX VERSION / TYPE"));
          it != _header.end(); ){
        version_type.parse(it->second);
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
          ss << std::setfill(opt == 1 ? '0' : ' ') << std::right << std::setw(length) << *(T *)value;
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
          std::string::size_type pos(s.find("D"));
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
      }
      break;
    case 3:
      switch(buf[20]){
        case 'O': file_type = FTYPE_OBSERVATION;    break;
        case 'N': file_type = FTYPE_NAVIGATION;     break;
        case 'M': file_type = FTYPE_METEOROLOGICAL; break;
      }
      if((file_type == FTYPE_OBSERVATION) || (file_type == FTYPE_NAVIGATION)){
        switch(buf[40]){
          case 'G': sat_system = SYS_GPS;     break;
          case 'R': sat_system = SYS_GLONASS; break;
          case 'E': sat_system = SYS_GALILEO; break;
          case 'J': sat_system = SYS_QZSS;    break;
          case 'C': sat_system = SYS_BDS;     break;
          case 'I': sat_system = SYS_IRNSS;   break;
          case 'S': sat_system = SYS_SBAS;    break;
          case 'M': sat_system = SYS_MIXED;   break;
        }
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
          }
          break;
        }
        case FTYPE_NAVIGATION:
          switch(sat_system){
            case SYS_GPS:     type_str = "N: GPS NAV DATA";     break;
            case SYS_GLONASS: type_str = "G: GLONASS NAV DATA"; break;
            case SYS_SBAS:    type_str = "H: GEO NAV MSG DATA"; break;
          }
          break;
        case FTYPE_METEOROLOGICAL: type_str = "METEOROLOGICAL DATA"; break;
      }
      break;
    }
    case 3: {
      switch(file_type){
        case FTYPE_OBSERVATION:    type_str = "OBSERVATION DATA";    break;
        case FTYPE_NAVIGATION:     type_str = "N: GNSS NAV DATA";    break;
        case FTYPE_METEOROLOGICAL: type_str = "METEOROLOGICAL DATA"; break;
      }
      if((file_type != FTYPE_OBSERVATION) && (file_type != FTYPE_NAVIGATION)){break;}
      switch(sat_system){
        case SYS_GPS:     sys_str = "G: GPS";     break;
        case SYS_GLONASS: sys_str = "R: GLONASS"; break;
        case SYS_GALILEO: sys_str = "G: GALILEO"; break;
        case SYS_QZSS:    sys_str = "Q: QZSS";    break;
        case SYS_BDS:     sys_str = "B: BDS";     break;
        case SYS_IRNSS:   sys_str = "I: IRNSS";   break;
        case SYS_SBAS:    sys_str = "S: SBAS";    break;
        case SYS_MIXED:   sys_str = "M: MIXED";   break;
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
      super_t::version_type.version >= 300 ? seek_next_v3() : seek_next_v2();
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
      (reader.version_type.version >= 300)
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

template <class FloatT>
struct RINEX_OBS {
  struct epoch_flag_t {
    std::tm epoch;
    int epoch_year2, epoch_mon12;
    FloatT epoch_sec;
    int flag;
    int items_followed;

    epoch_flag_t &operator=(const GPS_Time<FloatT> &t){
      epoch = t.c_tm();
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
    GPS_Time<FloatT> t_epoc;
    FloatT receiver_clock_error;
    struct record_t {
      FloatT value;
      unsigned lli, ss;
    };
    typedef std::map<int, std::vector<record_t> > per_satellite_t;
    per_satellite_t per_satellite;
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

    static const typename super_t::convert_item_t epoch_flag_v2[8];
    static const typename super_t::convert_item_t record_v2[3];

    typedef typename RINEX_OBS<FloatT>::observation_t observation_t;

  protected:
    std::vector<std::string> types_of_observe;
    observation_t obs;
    static std::string &modify_header(std::string &label, std::string &content){
      return content;
    }

    void seek_next() {
      if(types_of_observe.size() == 0){return;}

      char buf[256];
      typedef std::vector<int> sat_list_t;
      sat_list_t sat_list;
      obs.per_satellite.clear();

      while(true){

        // Read lines for epoch
        {
          if(super_t::src.getline(buf, sizeof(buf)).fail()){return;}
          std::string data_line(buf);

          epoch_flag_t epoch_flag;
          super_t::convert(epoch_flag_v2, data_line, &epoch_flag);
          epoch_flag.epoch.tm_year = epoch_flag.epoch_year2 + (epoch_flag.epoch_year2 < 80 ? 100 : 0); // greater than 1980
          epoch_flag.epoch.tm_mon = epoch_flag.epoch_mon12 - 1; // month [0, 11]
          epoch_flag.epoch.tm_sec = (int)epoch_flag.epoch_sec;
          obs.t_epoc = epoch_flag;

          if(epoch_flag.flag >= 2){
            for(int i(0); i < epoch_flag.items_followed; ++i){
              if(super_t::src.getline(buf, sizeof(buf)).fail()){return;}
            }
            continue;
          }

          // Receiver clock error
          super_t::template conv_t<FloatT>::f(data_line, 68, 12, &obs.receiver_clock_error, 9);

          int prn;
          for(int items(0), i(0); items < epoch_flag.items_followed; items++, i++){
            if(i % 12 == 0){  // if number of satellites is greater than 12, move to the next line
              if(super_t::src.getline(buf, sizeof(buf)).fail()){return;}
              data_line = std::string(buf);
              i = 0;
            }
            super_t::template conv_t<int>::d(data_line, 33 + (i * 3), 2, &prn);
            switch(data_line[32 + (i * 3)]){
              case ' ':
              case 'G':               break; // NAVSTAR
              case 'R': prn += 0x100; break; // GLONASS
              case 'S': prn += 100;   break; // SBAS
              default: prn += 0x400;
            }
            sat_list.push_back(prn);
          }
        }

        // Observation data per satellite
        for(typename sat_list_t::const_iterator it(sat_list.begin()), it_end(sat_list.end());
            it != it_end; ++it){
          std::string data_line;
          for(int i(0); i < types_of_observe.size(); i++){
            int offset_index(i % 5);
            if(offset_index == 0){
              if(super_t::src.getline(buf, sizeof(buf)).fail()){return;}
              data_line = std::string(buf);
            }
            typename observation_t::record_t record = {0};
            super_t::convert(record_v2, data_line.substr(offset_index * 16, 16), &record);
            obs.per_satellite[*it].push_back(record);
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
      typedef super_t::header_t::iterator it_t;
      std::pair<it_t, it_t> range(super_t::_header.equal_range("# / TYPES OF OBSERV"));
      int types(0);
      for(it_t it(range.first); it != range.second; ++it){
        int tmp(0);
        super_t::conv_t<int>::d(it->second, 0, 6, &tmp);
        if(tmp > types){types = tmp;}
        std::stringstream ss(it->second.substr(6));
        for(int i(0); i < 9; ++i){
          std::string param_name;
          ss >> param_name;
          if(!ss.good()){break;}
          types_of_observe.push_back(param_name);
        }
      }
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
    int observed_index(const std::string &label) const {
      int res(distance(types_of_observe.begin(),
          find(types_of_observe.begin(), types_of_observe.end(), label)));
      return (res >= types_of_observe.size() ? -1 : res);
    }
    int observed_index(const char *label) const {
      return observed_index(std::string(label));
    }
};

#define GEN_D(offset, length, container_type, container_member, value_type) \
    {super_t::template conv_t<value_type>::d, offset, length, \
      offsetof(container_type, container_member)}
#define GEN_I(offset, length, container_type, container_member, value_type) \
    {super_t::template conv_t<value_type>::d, offset, length, \
      offsetof(container_type, container_member), 1}
#define GEN_F(offset, length, precision, container_type, container_member) \
    {super_t::template conv_t<FloatT>::f, offset, length, \
      offsetof(container_type, container_member), precision}
#define GEN_E(offset, length, precision, container_type, container_member) \
    {super_t::template conv_t<FloatT>::e, offset, length, \
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
};

template <class FloatT>
const typename RINEX_OBS_Reader<FloatT>::convert_item_t RINEX_OBS_Reader<FloatT>::record_v2[] = {
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
            it_tail = it.base() - 1;
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
            it_head = it_tail = header.rbegin().base();
            return *this;
          }
          it_head->second = value;
          for(; it_tail != it_head; --it_tail){
            if(it_tail->first == key){continue;}
            header.erase(it_tail);
          }
          return *this;
        }
        /**
         * add value corresponding to key after the last entry having the same key
         */
        bracket_accessor_t &operator<<(
            const typename super_t::value_type::second_type &value){
          if(it_tail == header.end()){
            header.push_back(typename super_t::value_type(key, value));
            it_head = it_tail = header.rbegin().base();
            return *this;
          }
          header.insert(++it_tail, typename super_t::value_type(key, value));
          return *this;
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
    version_type_t _version_type;
    header_t _header;
    std::ostream &dist;
    
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
        : _version_type(), _header(header_mask, header_mask_size), dist(out) {
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
      super_t::convert(reader_t::utc_leap, s, &space_node.iono_utc());
      _header["LEAP SECONDS"] = s;
    }
    RINEX_NAV_Writer(std::ostream &out)
        : super_t(out, default_header, default_header_size) {}
    ~RINEX_NAV_Writer(){}

    self_t &operator<<(const message_t &msg){
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
              case 0: super_t::convert(reader_t::eph0_v3, s, &msg); s[0] = 'G'; break;
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
    int write_all(const space_node_t &space_node, const int &version = 304){
      int res(-1);
      set_version_type(typename super_t::version_type_t(
          version, super_t::version_type_t::FTYPE_NAVIGATION, super_t::version_type_t::SYS_GPS));
      if(space_node.is_valid_iono_utc()){
        iono_alpha(space_node);
        iono_beta(space_node);
        utc_params(space_node);
        leap_seconds(space_node);
      }
      super_t::dist << header();
      res++;

      WriteAllFunctor functor = {*this, res};
      for(typename space_node_t::satellites_t::const_iterator
            it(space_node.satellites().begin()), it_end(space_node.satellites().end());
          it != it_end; ++it){
        it->second.each_ephemeris(
            functor,
            space_node_t::Satellite::eph_list_t::EACH_ALL_INVERTED);
      }
      return res;
    }
    static int write_all(std::ostream &out, const space_node_t &space_node, const int &version = 304){
      RINEX_NAV_Writer writer(out);
      return writer.write_all(space_node, version);
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
    self_t &operator<<(const observation_t &obs){
      if(obs.per_satellite.size() <= 0){return *this;}
      
      typedef typename observation_t::per_satellite_t per_satellite_t;
      
      std::stringstream top, rest;
      
      std::string buf(80, ' ');
      { // epoch
        typename RINEX_OBS<FloatT>::epoch_flag_t epoch_flag = {0};
        epoch_flag = obs.t_epoch;
        epoch_flag.items_followed = obs.per_satellite.size(); // Num. of satellites
        super_t::convert(reader_t::epoch_flag, buf, epoch_flag);
        reader_t::template conv_t<FloatT>::f(
            buf, 68, 12, &(const_cast<observation_t &>(obs).receiver_clock_error), 9, false);
      }
      
      int i(0);
      for(typename per_satellite_t::const_iterator
            it(obs.per_satellite.begin()), it_end(obs.per_satellite.end());
          it != it_end(); ++it, ++i){ // Enumerate satellites
        
        // If satellites are more than 12, then use the next line
        if(i == 12){
          i = 0;
          top << buf << std::endl;
          buf = std::string(80, ' ');
        }
        
        // Write PRN number to list
        int prn(it->first);
        if(prn < 100){
          buf[32 + i * 3] = 'G';
        }else if(prn < 200){
          buf[32 + i * 3] = 'S';
          prn -= 100;
        }else if(prn & 0x100){
          buf[32 + i * 3] = 'R';
          prn &= 0xFF;
        }else{continue;}
        reader_t::template conv_t<int>::d(buf, 33 + i * 3, 2, &prn, 0, false);

        // Observation data
        int i2(5);
        for(typename per_satellite_t::mapped_type::const_iterator it2(it->second.begin()),
              it2_end(it->second.end());
            it2 != it2_end; ++it2){
          if(--i2 == 0){
            i2 = 5;
            rest << std::endl;
          }
          std::string buf2(16, ' ');
          super_t::convert(reader_t::record_v2, buf2, &(*it2));
          rest << buf2;
        }
        rest << std::string(16 * i2, ' ') << std::endl;
      }
      top << buf << std::endl;
      
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
