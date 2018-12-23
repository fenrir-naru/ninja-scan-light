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
 * @brief RINEX Loader, support RINEX 2
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

#include "GPS.h"

class RINEX_Reader {
  public:
    typedef std::map<std::string, std::string> header_t;
    
  protected:
    header_t _header;
    std::istream &src;
    bool _has_next;
    
  public:
    RINEX_Reader(
        std::istream &in,
        std::string &(*modify_header)(std::string &, std::string &) = NULL)
        : src(in), _has_next(false){
      if(src.fail()){return;}
      
      char buf[256];
      
      // Read header
      while(!src.eof()){
        src.getline(buf, sizeof(buf));
        
        std::string content(buf);
        std::string label(content, 60, 20);
        {
          int real_length(label.find_last_not_of(' ') + 1);
          if(real_length < label.length()){
            label = label.substr(0, real_length);
          }
        }
        // std::cerr << label << " (" << label.length() << ")" << std::endl;
        
        if(label.find("END OF HEADER") == 0){break;}
        
        content = content.substr(0, 60);
        if(modify_header){content = modify_header(label, content);}
        if(_header.find(label) == _header.end()){
          _header[label] = content;
        }else{
          _header[label].append(content); // If already exist, then append.
        }
      }
    }
    virtual ~RINEX_Reader(){_header.clear();}
    header_t &header() {return _header;}
    bool has_next() const {return _has_next;}
};

template <class FloatT>
struct RINEX_NAV {
  typedef GPS_SpaceNode<FloatT> space_node_t;
  typedef typename space_node_t::Ionospheric_UTC_Parameters iono_utc_t;
  typedef typename space_node_t::Satellite::Ephemeris ephemeris_t;
  struct SatelliteInfo {
    unsigned int svid;
    FloatT t_ot;  ///< Transmitting time [s]
    ephemeris_t ephemeris;
  };
};

template <class FloatT = double>
class RINEX_NAV_Reader : public RINEX_Reader {
  protected:
    typedef RINEX_NAV<FloatT> content_t;
    typedef RINEX_NAV_Reader<FloatT> self_t;
    typedef RINEX_Reader super_t;
  public:
    typedef typename content_t::space_node_t space_node_t;
    typedef typename content_t::ephemeris_t ephemeris_t;
    typedef typename content_t::SatelliteInfo SatelliteInfo;
  
  protected:
    SatelliteInfo info;
    static std::string &modify_header(std::string &label, std::string &content){
      if((label.find("ION ALPHA") == 0)
          || (label.find("ION BETA") == 0)
          || (label.find("DELTA-UTC: A0,A1,T,W") == 0)){
        for(int pos(0);
            (pos = content.find("D", pos)) != std::string::npos;
            ){
          content.replace(pos, 1, "E");
        }
      }
      return content;
    }
    
    void seek_next() {
      char buf[256];
      
      FloatT ura_meter;
      for(int i = 0; (i < 8) && (super_t::src.good()); i++){
        if(super_t::src.getline(buf, sizeof(buf)).fail()){return;}
        std::string line_data(buf);
        for(int pos(0);
            (pos = line_data.find("D", pos)) != std::string::npos;
            ){
          line_data.replace(pos, 1, "E");
        }
        std::stringstream data(line_data);
#define GET_SS(offset, length) \
std::stringstream(line_data.size() > offset ? line_data.substr(offset, length) : "0")
        
        FloatT dummy;
        switch(i){
          case 0: {
            GET_SS(0, 2) >> info.svid;
            info.ephemeris.svid = info.svid;
            struct tm t;
            GET_SS(3, 2) >> t.tm_year; // year
            GET_SS(6, 2) >> t.tm_mon;  // month
            --(t.tm_mon);
            GET_SS(9, 2) >> t.tm_mday; // day of month
            GET_SS(12, 2) >> t.tm_hour; // hour
            GET_SS(15, 2) >> t.tm_min;  // minute
            t.tm_sec = 0;
            GPS_Time<FloatT> gps_time(t);
            info.ephemeris.WN = gps_time.week;
            GET_SS(17, 5) >> info.ephemeris.t_oc; // second
            info.ephemeris.t_oc += gps_time.seconds;
            GET_SS(22, 19) >> info.ephemeris.a_f0;
            GET_SS(41, 19) >> info.ephemeris.a_f1;
            GET_SS(60, 19) >> info.ephemeris.a_f2;
            break;
          }
#define READ_AND_STORE(line_num, v0, v1, v2, v3) \
case line_num: { \
  FloatT v; \
  try{GET_SS( 3, 19) >> v; v0 = v;}catch(std::exception &e){v0 = 0;} \
  try{GET_SS(22, 19) >> v; v1 = v;}catch(std::exception &e){v1 = 0;} \
  try{GET_SS(41, 19) >> v; v2 = v;}catch(std::exception &e){v2 = 0;} \
  try{GET_SS(60, 19) >> v; v3 = v;}catch(std::exception &e){v3 = 0;} \
  break; \
}
          READ_AND_STORE(1,
            info.ephemeris.iode,
            info.ephemeris.c_rs,
            info.ephemeris.delta_n,
            info.ephemeris.m0);
          READ_AND_STORE(2,
            info.ephemeris.c_uc,
            info.ephemeris.e,
            info.ephemeris.c_us,
            info.ephemeris.sqrt_A);
          READ_AND_STORE(3,
            info.ephemeris.t_oe,
            info.ephemeris.c_ic,
            info.ephemeris.Omega0,
            info.ephemeris.c_is);
          READ_AND_STORE(4,
            info.ephemeris.i0,
            info.ephemeris.c_rc,
            info.ephemeris.omega,
            info.ephemeris.dot_Omega0);
          READ_AND_STORE(5,
            info.ephemeris.dot_i0,
            dummy,  // Codes on L2 channel
            info.ephemeris.WN,
            dummy); // L2 P data flag
          READ_AND_STORE(6,
            ura_meter,
            info.ephemeris.SV_health,
            info.ephemeris.t_GD,
            info.ephemeris.iodc);
          READ_AND_STORE(7,
            info.t_ot,
            info.ephemeris.fit_interval, // in hours
            dummy,  // spare
            dummy); // spare
#undef READ_AND_STORE
#undef GET_SS
        }
      }
      
      info.ephemeris.URA = ephemeris_t::URA_index(ura_meter); // meter to index

      /* @see ftp://igs.org/pub/data/format/rinex210.txt
       * 6.7 Satellite Health
       * RINEX Value:   0    Health OK
       * RINEX Value:   1    Health not OK (bits 18-22 not stored)
       * RINEX Value: >32    Health not OK (bits 18-22 stored)
       */
      if(info.ephemeris.SV_health > 32){
        info.ephemeris.SV_health &= 0x3F; // 0b111111
      }else if(info.ephemeris.SV_health > 0){
        info.ephemeris.SV_health = 0x20; // 0b100000
      }

      if(info.ephemeris.fit_interval < 4){
        info.ephemeris.fit_interval = 4; // At least 4 hour validity
      }
      info.ephemeris.fit_interval *= (60 * 60); // hours => seconds;
      
      super_t::_has_next = true;
    }
  
  public:
    RINEX_NAV_Reader(std::istream &in) : super_t(in, self_t::modify_header) {
      seek_next();
    }
    ~RINEX_NAV_Reader(){}
    
    SatelliteInfo next() {
      SatelliteInfo current(info);
      super_t::_has_next = false;
      seek_next();
      return current;
    }
    
    /**
     * Obtain ionospheric delay coefficient.
     * 
     * @return true when successfully obtained, otherwise false
     */
    bool extract_iono_utc(GPS_SpaceNode<FloatT> &space_node) const {
      typename space_node_t::Ionospheric_UTC_Parameters iono_utc;

      if(_header.find("ION ALPHA") == _header.end()){return false;}
      {
        std::stringstream sstr(const_cast<super_t::header_t *>(&(_header))
            ->operator[]("ION ALPHA"));
        for(int i(0); i < sizeof(iono_utc.alpha) / sizeof(iono_utc.alpha[0]); ++i){
          sstr >> iono_utc.alpha[i];
        }
      }
      
      if(_header.find("ION BETA") == _header.end()){return false;}
      {
        std::stringstream sstr(const_cast<super_t::header_t *>(&(_header))
            ->operator[]("ION BETA"));
        for(int i(0); i < sizeof(iono_utc.beta) / sizeof(iono_utc.beta[0]); ++i){
          sstr >> iono_utc.beta[i];
        }
      }

      if(_header.find("DELTA-UTC: A0,A1,T,W") == _header.end()){return false;}
      {
        std::stringstream sstr(const_cast<super_t::header_t *>(&(_header))
            ->operator[]("DELTA-UTC: A0,A1,T,W"));
        sstr >> iono_utc.A0;
        sstr >> iono_utc.A1;
        sstr >> iono_utc.t_ot;
        sstr >> iono_utc.WN_t;
      }

      if(_header.find("LEAP SECONDS") != _header.end()){
        std::stringstream sstr(const_cast<super_t::header_t *>(&(_header))
            ->operator[]("LEAP SECONDS"));
        sstr >> iono_utc.delta_t_LS;
      }

      space_node.update_iono_utc(iono_utc);
      return true;
    }

    static int read_all(std::istream &in, space_node_t &space_node){
      int res(-1);
      RINEX_NAV_Reader reader(in);
      if(!reader.extract_iono_utc(space_node)){return res;}
      res++;
      for(; reader.has_next(); ++res){
        SatelliteInfo info(reader.next());
        space_node.satellite(info.svid).register_ephemeris(info.ephemeris);
      }
      return res;
    }
};

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
class RINEX_OBS_Reader : public RINEX_Reader {
  protected:
    typedef RINEX_OBS<FloatT> content_t;
    typedef RINEX_OBS_Reader<FloatT> self_t;
    typedef RINEX_Reader super_t;
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
          data >> t.tm_year; // year
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
      if(super_t::_header.find("# / TYPES OF OBSERV") != super_t::_header.end()){
        std::stringstream data(super_t::_header["# / TYPES OF OBSERV"]);
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

class RINEX_Writer {
  public:
    typedef struct {const char *key, *value;} header_item_t;
    
    class header_t : public std::map<std::string, std::string> {
      private:
        template <class T1, class T2>
        static void write_header_item(
            std::ostream &out, 
            const T1 &key, const T2 &value){
          out << std::setw(60) << value;
          out << std::setw(20) << key << std::endl;
        }
      public:
        const header_item_t *mask;
        const int mask_size;
        header_t(
            const header_item_t *_mask = NULL, 
            const int _mask_size = 0) 
            : std::map<std::string, std::string>(), mask(_mask), mask_size(_mask_size) {}
        ~header_t() {}
        friend std::ostream &operator<<(std::ostream &out, const header_t &header){
          
          std::stringstream ss;
          ss << std::setfill(' ') << std::left;
          if(header.mask){
            // Output as well as sort in accordance with mask
            for(int i(0); i < header.mask_size; i++){
              header_t::const_iterator it(header.find(header.mask[i].key));
              if(it != header.end()){
                write_header_item(
                    ss, header.mask[i].key, it->second);
              }else if(header.mask[i].value){
                // Default value
                write_header_item(
                    ss, header.mask[i].key, header.mask[i].value);
              }
            }
          }else{
            for(header_t::const_iterator it(header.begin()); 
                it != header.end(); 
                ++it){
              for(int index(0); index < it->second.length(); index += 60){
                write_header_item(
                    ss, it->first.substr(0, 20), it->second.substr(index, 60));
              }
            }
          }
          write_header_item(ss, "END OF HEADER", "");
          
          return out << ss.str();
        }
    };
  protected:
    header_t _header;
    std::ostream &dist;
    
  public:
    RINEX_Writer(
        std::ostream &out,
        const header_item_t *header_mask = NULL, 
        const int header_mask_size = 0) 
        : _header(header_mask, header_mask_size), dist(out) {
      
    }
    virtual ~RINEX_Writer() {_header.clear();}

    const header_t &header() const {return _header;}
    header_t &header(){return const_cast<header_t &>(static_cast<const RINEX_Writer *>(this)->header());}
    
    template <class FloatT>
    static std::string RINEX_Float(
        const FloatT &value, const int width = 19, const int precision = 12){
      std::stringstream ss;
      ss << std::setfill(' ') << std::right << std::setw(width)
          << std::setprecision(precision) << std::fixed 
          << value;
      return ss.str();
    }
    
    template <class FloatT>
    static std::string RINEX_FloatD(
        const FloatT &value, const int width = 19, const int precision = 12){
      int w(std::max(width, precision + 6));
      
      std::stringstream ss;
      ss << std::setprecision(precision - 1) << std::scientific
          << (value * 1E1);
      std::string s(ss.str());
      //std::cerr << s << std::endl;
      
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
      
      return ss.str();
    }
    template <class T>
    static std::string RINEX_Value(
        const T &value, const int width = 6){
      std::stringstream ss;
      ss << std::setfill(' ') << std::right << std::setw(width) << value;
      return ss.str();
    }
    
    void leap_seconds(const int &seconds){
      std::stringstream ss;
      ss << RINEX_Value(seconds, 6);
      _header["LEAP SECONDS"] = ss.str();
    }
};

template <class FloatT>
class RINEX_NAV_Writer : public RINEX_Writer {
  public:
    typedef RINEX_NAV<FloatT> content_t;
    typedef RINEX_NAV_Writer self_t;
    typedef RINEX_Writer super_t;
  protected:
    using super_t::RINEX_Float;
    using super_t::RINEX_FloatD;
    using super_t::RINEX_Value;
    using super_t::_header;
    using super_t::dist;
  public:
    typedef typename content_t::space_node_t space_node_t;
    typedef typename content_t::ephemeris_t ephemeris_t;

    static const super_t::header_item_t default_header[];
    static const int default_header_size;
    void iono_alpha(
        const FloatT &a0, const FloatT &a1, 
        const FloatT &a2, const FloatT &a3){
      std::stringstream ss;
      ss << "  " 
          << RINEX_FloatD(a0, 12, 4)
          << RINEX_FloatD(a1, 12, 4)
          << RINEX_FloatD(a2, 12, 4)
          << RINEX_FloatD(a3, 12, 4);
      _header["ION ALPHA"] = ss.str();
    }
    void iono_beta(
        const FloatT &b0, const FloatT &b1, 
        const FloatT &b2, const FloatT &b3){
      std::stringstream ss;
      ss << "  " 
          << RINEX_FloatD(b0, 12, 4)
          << RINEX_FloatD(b1, 12, 4)
          << RINEX_FloatD(b2, 12, 4)
          << RINEX_FloatD(b3, 12, 4);
      _header["ION BETA"] = ss.str();
    }
    void utc_params(
        const FloatT &A0, const FloatT &A1,
        const int &T, const int &W){
      std::stringstream ss;
      ss << "   "
          << RINEX_FloatD(A0, 19, 12)
          << RINEX_FloatD(A1, 19, 12)
          << RINEX_Value(T, 9)
          << RINEX_Value(W, 9);
      _header["DELTA UTC: A0,A1,T,W"] = ss.str();
    }
    RINEX_NAV_Writer(std::ostream &out)
        : super_t(out, default_header, default_header_size) {}
    ~RINEX_NAV_Writer(){}
    self_t &operator<<(const typename content_t::SatelliteInfo &data){
      std::stringstream buf;
      
      // PRN
      buf << RINEX_Value(data.svid % 100, 2);
      
      // Time
      {
        GPS_Time<FloatT> gps_time(data.ephemeris.WN, data.ephemeris.t_oc);
        struct tm t(gps_time.c_tm());
        FloatT sec_f(gps_time.seconds), sec_i;
        sec_f = std::modf(sec_f, &sec_i);
        buf << (t.tm_year < 10 ? " 0" : " ") << t.tm_year
            << RINEX_Value(t.tm_mon + 1, 3)
            << RINEX_Value(t.tm_mday, 3)
            << RINEX_Value(t.tm_hour, 3)
            << RINEX_Value(t.tm_min, 3)
            << RINEX_Float(sec_f + t.tm_sec, 5, 1);
      }
      
      // Rest of the 1st line
      buf << RINEX_FloatD(data.ephemeris.a_f0, 19, 12)
          << RINEX_FloatD(data.ephemeris.a_f1, 19, 12)
          << RINEX_FloatD(data.ephemeris.a_f2, 19, 12)
          << std::endl;
      
      // 2nd line
      buf << std::string(3, ' ')
          << RINEX_FloatD(data.ephemeris.iode, 19, 12)
          << RINEX_FloatD(data.ephemeris.c_rs, 19, 12)
          << RINEX_FloatD(data.ephemeris.delta_n, 19, 12)
          << RINEX_FloatD(data.ephemeris.m0, 19, 12)
          << std::endl;
      
      // 3rd line
      buf << std::string(3, ' ')
          << RINEX_FloatD(data.ephemeris.c_uc, 19, 12)
          << RINEX_FloatD(data.ephemeris.e, 19, 12)
          << RINEX_FloatD(data.ephemeris.c_us, 19, 12)
          << RINEX_FloatD(data.ephemeris.sqrt_A, 19, 12)
          << std::endl;
      
      // 4th line
      buf << std::string(3, ' ')
          << RINEX_FloatD(data.ephemeris.t_oe, 19, 12)
          << RINEX_FloatD(data.ephemeris.c_ic, 19, 12)
          << RINEX_FloatD(data.ephemeris.Omega0, 19, 12)
          << RINEX_FloatD(data.ephemeris.c_is, 19, 12)
          << std::endl;
      
      // 5th line
      buf << std::string(3, ' ')
          << RINEX_FloatD(data.ephemeris.i0, 19, 12)
          << RINEX_FloatD(data.ephemeris.c_rc, 19, 12)
          << RINEX_FloatD(data.ephemeris.omega, 19, 12)
          << RINEX_FloatD(data.ephemeris.dot_Omega0, 19, 12)
          << std::endl;
      
      // 6th line
      buf << std::string(3, ' ')
          << RINEX_FloatD(data.ephemeris.dot_i0, 19, 12)
          << RINEX_FloatD(0.0, 19, 12) // Codes on L2 channel
          << RINEX_FloatD(data.ephemeris.WN, 19, 12)
          << RINEX_FloatD(0.0, 19, 12) // L2 P data flag
          << std::endl;
      
      // 7th line
      unsigned int health(data.ephemeris.SV_health);
      if((health & 0x20) && (health & 0x1F == 0)){
        health = 1;
      }
      buf << std::string(3, ' ')
          << RINEX_FloatD(ephemeris_t::URA_meter(data.ephemeris.URA), 19, 12)
          << RINEX_FloatD(health, 19, 12)
          << RINEX_FloatD(data.ephemeris.t_GD, 19, 12)
          << RINEX_FloatD(data.ephemeris.iodc, 19, 12)
          << std::endl;
      
      // 8th line
      buf << std::string(3, ' ')
          << RINEX_FloatD(data.t_ot, 19, 12)
          << RINEX_FloatD(data.ephemeris.fit_interval / (60 * 60), 19, 12)
          << RINEX_FloatD(0.0, 19, 12) // spare
          << RINEX_FloatD(0.0, 19, 12) // spare
          << std::endl;
      
      dist << buf.str();
      return *this;
    }

  protected:
    struct WriteAllFunctor {
      typename content_t::SatelliteInfo info;
      RINEX_NAV_Writer &w;
      int &counter;
      void operator()(const typename content_t::ephemeris_t &eph) {
        info.svid = eph.svid;
        info.ephemeris = eph;
        w << info;
        counter++;
      }
    };

  public:
    static int write_all(std::ostream &out, const space_node_t &space_node){
      int res(-1);
      RINEX_NAV_Writer writer(out);
      if(!space_node.is_valid_iono_utc()){return res;}
      {
        const typename content_t::iono_utc_t &iu(space_node.iono_utc());
        writer.iono_alpha(iu.alpha[0], iu.alpha[1], iu.alpha[2], iu.alpha[3]);
        writer.iono_beta(iu.beta[0], iu.beta[1], iu.beta[2], iu.beta[3]);
        writer.utc_params(iu.A0, iu.A1, iu.t_ot, iu.WN_t);
        writer.leap_seconds(iu.delta_t_LS);
      }
      out << writer.header();
      res++;

      WriteAllFunctor functor = {{0, 0}, writer, res};
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
const RINEX_Writer::header_item_t RINEX_NAV_Writer<FloatT>::default_header[] = {
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
class RINEX_OBS_Writer : public RINEX_Writer {
  public:
    typedef RINEX_OBS<FloatT> content_t;
    typedef RINEX_OBS_Writer self_t;
    typedef RINEX_Writer super_t;
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
        if(t.tm_year < 10){
          top << " 0" << t.tm_year;
        }else{
          top << ' ' << t.tm_year;
        }
        top << RINEX_Value(t.tm_mon + 1, 3)
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
const RINEX_Writer::header_item_t RINEX_OBS_Writer<FloatT>::default_header[] = {
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
