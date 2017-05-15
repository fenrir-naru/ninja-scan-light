/*
 * Copyright (c) 2016, M.Naruoka (fenrir)
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

#ifndef __ANALYZE_COMMON_H__
#define __ANALYZE_COMMON_H__

#include <iostream>
#include <fstream>
#include <map>

#include <cstdio>
#include <cstring>
#include <cmath>
#include <cstdlib>
#include <climits>
#include <cfloat>
#if defined(_MSC_VER) || defined(__CYGWIN__)
#include <io.h>
#include <fcntl.h>
#endif

#include "util/comstream.h"
#include "util/nullstream.h"
#include "util/endian.h"

/**
 * Convert units from degrees to radians
 *
 * @param degrees
 * @return radians
 */
template <class FloatT>
FloatT deg2rad(const FloatT &degrees){return degrees * M_PI / 180;}
/**
 * Convert units from radians to degrees
 *
 * @param radians
 * @return degrees
 */
template <class FloatT>
FloatT rad2deg(const FloatT &radians){return radians * 180 / M_PI;}

template <class FloatT>
struct GlobalOptions {
  protected:
  GlobalOptions(const GlobalOptions<FloatT> &);
  GlobalOptions<FloatT> &operator=(const GlobalOptions<FloatT> &);

  public:
  struct gps_time_t {
    FloatT sec; ///< GPS time
    int wn; ///< GPS week number
    gps_time_t() : sec(0), wn(0) {}
    gps_time_t(const FloatT &_sec, const int &_wn)
        : sec(_sec), wn(_wn) {}
    template <class T1, class T2>
    bool is_before(const T1 &base_sec, const T2 &base_wn) const {
      return (base_wn > wn) || ((base_wn == wn) && (base_sec >= sec));
    }
    template <class T1, class T2>
    bool is_after(const T1 &base_sec, const T2 &base_wn) const {
      return (base_wn < wn) || ((base_wn == wn) && (base_sec <= sec));
    }
  };
  gps_time_t start_gpstime;  ///< Start GPS time
  gps_time_t end_gpstime;    ///< End GPS time
  bool reduce_1pps_sync_error; ///< True when auto correction for 1pps sync. error is activated
  NullStream blackhole;
  std::ostream *_out; ///< Pointer for output stream
  std::ostream *_out_debug; ///< Pointer for debug output stream
  bool in_sylphide;   ///< True when inputs is Sylphide formated
  bool out_sylphide;  ///< True when outputs is Sylphide formated
  typedef std::map<const char *, std::iostream *> iostream_pool_t;
  iostream_pool_t iostream_pool;

  static const char *null_fname(){
#if defined(_MSC_VER)
    return "nul";
#else
    return "/dev/null";
#endif
  }
  
  GlobalOptions()
      : start_gpstime(0, -1), end_gpstime(DBL_MAX, INT_MAX),
      reduce_1pps_sync_error(true),
      blackhole(),
      _out(&(std::cout)),
      _out_debug(&blackhole),
      in_sylphide(false), out_sylphide(false),
      iostream_pool() {};
  virtual ~GlobalOptions(){
    for(iostream_pool_t::iterator it(iostream_pool.begin());
        it != iostream_pool.end();
        ++it){
      it->second->flush();
      delete it->second;
    }
  }
  
  template <class T1, class T2>
  bool is_time_after_start(const T1 &sec, const T2 &wn) const {
    return start_gpstime.is_before(sec, wn);
  }

  template <class T1, class T2>
  bool is_time_before_end(const T1 &sec, const T2 &wn) const {
    return end_gpstime.is_after(sec, wn);
  }

  template <class T1, class T2>
  bool is_time_in_range(const T1 &sec, const T2 &wn) const {
    return is_time_after_start(sec, wn) && is_time_before_end(sec, wn);
  }

  void set_baudrate(ComportStream &com, const char *baudrate_spec){
    int baudrate(std::atoi(baudrate_spec));
    if(baudrate != com.buffer().set_baudrate(baudrate)){
      std::cerr << " => Unsupported baudrate!!" << std::endl;
      exit(-1);
    }
  }

#ifdef _WIN32
#define COMPORT_PREFIX "COM"
#else
#define COMPORT_PREFIX "/dev/tty"
#endif
  
  std::istream &spec2istream(
      const char *spec, 
      const bool &force_fstream = false){
    if(!force_fstream){
      if(strcmp(spec, "-") == 0){
        // '-' stands for standard inputs
        std::cerr << "[std::cin]" << std::endl;
#if defined(_MSC_VER) || defined(__CYGWIN__)
        setmode(fileno(stdin), O_BINARY);
#endif
        return std::cin;
      }else if(strstr(spec, COMPORT_PREFIX) == spec){
        std::cerr << spec << std::endl;
        // COM ports
        // COM_name[:baudrate] format is acceptable.
        char *baudrate_spec((char *)strchr(spec, ':'));
        if(baudrate_spec){
          *baudrate_spec = '\0';
          baudrate_spec++;
        }
        if(iostream_pool.find(spec) == iostream_pool.end()){
          ComportStream *com_in = new ComportStream(spec);
          if(baudrate_spec){set_baudrate(*com_in, baudrate_spec);}
          iostream_pool[spec] = com_in;
          return *com_in;
        }else{
          return *(iostream_pool[spec]);
        }
      }
    }
    
    std::cerr << spec;
    std::fstream *fin(new std::fstream(spec, std::ios::in | std::ios::binary));    
    if(fin->fail()){
      std::cerr << " => File not found!!" << std::endl;
      exit(-1);
    }
    std::cerr << std::endl;
    iostream_pool[spec] = fin;
    return *fin;
  }
  
  std::ostream &spec2ostream(
      const char *spec,
      const bool &force_fstream = false){
    if(!force_fstream){
      if(std::strcmp(spec, "-") == 0){
        // '-' stands for standard outputs
        std::cerr << "[std::cout]" << std::endl;
#if defined(_MSC_VER) || defined(__CYGWIN__)
        setmode(fileno(stdout), O_BINARY);
#endif
        return std::cout;
      }else if(std::strstr(spec, COMPORT_PREFIX) == spec){
        std::cerr << spec << std::endl;
        // COMƒ|[ƒg
        // COM_name[:baudrate] format is acceptable.
        char *baudrate_spec((char *)std::strchr(spec, ':'));
        if(baudrate_spec){
          *baudrate_spec = '\0';
          baudrate_spec++;
        }
        if(iostream_pool.find(spec) == iostream_pool.end()){
          ComportStream *com_out = new ComportStream(spec);
          if(baudrate_spec){set_baudrate(*com_out, baudrate_spec);}
          iostream_pool[spec] = com_out;
          return *com_out;
        }else{
          return *(iostream_pool[spec]);
        }
      }
    }
    
    std::cerr << spec;
    std::fstream *fout(new std::fstream(spec, std::ios::out | std::ios::binary));
    std::cerr << std::endl;
    iostream_pool[spec] = fout;
    return *fout;
  }
  
  std::ostream &out() const {return *_out;}
  std::ostream &out_debug() const {return *_out_debug;}

  /**
   * @param spec check target
   * @param key_head pointer to key head pointer to be stored, only available when key found.
   * @return if key found, return key length, otherwise zero.
   */
  static unsigned int get_key(const char *spec, const char **key_head){
    *key_head = NULL;
    if(std::strstr(spec, "--") != spec){return 0;}
    unsigned int offset(2);
    *key_head = &spec[2];
    while((spec[offset] != '\0') && (spec[offset] != '=')){
      offset++;
    }
    return offset - 2;
  }

  /**
   * get leading pointer of values in format of --key[=value] string
   * @param spec string to be checked
   * @param key_length length of key; if zero, key is automatically searched
   * @param accept_no_value when true, return true even if the value is omitted, otherwise, return NULL
   */
  static const char *get_value(
      const char *spec,
      const unsigned int &key_length = 0, const bool &accept_no_value = true){
    unsigned int offset(key_length);
    if(key_length == 0){ // check key length
      const char *key_head;
      if((offset = get_key(spec, &key_head)) == 0){
        return NULL;
      }
    }
    offset += 2; // move pointer to position followed by "--(key)"
    if((spec[offset] != '\0') && (spec[offset] == '=')){
      return spec + offset + 1;
    }else{
      return accept_no_value ? "true" : NULL;
    }
  }

  /**
   * get leading pointer of values in format of --key[=value] string
   * @param spec string to be checked
   * @param key key to be checked
   * @param accept_no_value when true, return true even if the value is omitted, otherwise, return NULL
   */
  static const char *get_value(
      const char *spec, const char *key, const bool &accept_no_value = true){
    const char *key_head;
    unsigned int key_length(get_key(spec, &key_head));
    if(key_length != std::strlen(key)){return NULL;} // key found? and same key_length?
    if(std::strncmp(key, key_head, key_length) != 0){ // same key?
      return NULL;
    }
    return get_value(spec, key_length, accept_no_value);
  }

  /**
   * get leading pointer of values in format of "key value1 value2 ..." string
   * @param spec string to be checked
   * @param key
   */
  static const char *get_value2(const char *spec, const char *key){
    int offset(std::strlen(key));
    if(std::strncmp(spec, key, offset) != 0){return NULL;}
    if((spec[offset] == '\0') || std::isgraph(spec[offset])){return NULL;} // no value or different key.
    while(spec[++offset] != '\0'){
      if(std::isgraph(spec[offset])){return &spec[offset];}
    }
    return NULL; // no value
  }

  static bool is_true(const char *value){
    return (std::strcmp(value, "on") == 0) || (std::strcmp(value, "true") == 0);
  }

  /**
   * Check spec
   * 
   * @param spec
   * @return (bool) True when interpreted, otherwise false.
   */
  virtual bool check_spec(const char *spec){
    using std::cerr;
    using std::endl;
    
    const char *key;
    const unsigned int key_length(get_key(spec, &key));
    if(key_length == 0){return false;}

    bool key_checked(false);

#define CHECK_KEY(name) \
  (key_checked \
    || (key_checked = ((key_length == std::strlen(#name)) \
        && (std::strncmp(key, #name, key_length) == 0))))
#define CHECK_ALIAS(name) CHECK_KEY(name)
#define CHECK_OPTION(name, novalue, operation, disp) { \
  if(CHECK_KEY(name)){ \
    const char *value(get_value(spec, key_length, novalue)); \
    if(!value){return false;} \
    {operation;} \
    std::cerr.write(key, key_length) << ": " << disp << std::endl; \
    return true; \
  } \
}
#define CHECK_OPTION_BOOL(target) \
CHECK_OPTION(target, true, target = is_true(value), (target ? "on" : "off"));

#define CHECK_GPSTIME(prefix) \
if(key_checked){ \
  const char *value(get_value(spec, key_length, false)); \
  if(!value){return false;} \
  int dummy_i; \
  double dummy_d; \
  if(std::sscanf(value, "%i:%lf", &dummy_i, &dummy_d) == 2){ \
    prefix ## _gpstime.sec = dummy_d; \
    prefix ## _gpstime.wn = dummy_i; \
    std::cerr.write(key, key_length) << ": " \
        << prefix ## _gpstime.wn << ":" \
        << prefix ## _gpstime.sec << std::endl; \
  }else{ \
    prefix ## _gpstime.sec = atof(value); \
    std::cerr.write(key, key_length) << ": " << prefix ## _gpstime.sec << std::endl; \
  } \
  return true; \
}

    CHECK_ALIAS(start-gpst);
    CHECK_KEY(start_gpst);
    CHECK_GPSTIME(start);

    CHECK_ALIAS(end-gpst);
    CHECK_KEY(end_gpst);
    CHECK_GPSTIME(end);
#undef CHECK_GPSTIME

    CHECK_ALIAS(start-gpswn);
    CHECK_OPTION(start_gpswn, false,
        start_gpstime.wn = std::atof(value),
        start_gpstime.wn);

    CHECK_ALIAS(end-gpswn);
    CHECK_OPTION(end_gpswn, false,
        end_gpstime.wn = std::atoi(value),
        end_gpstime.wn);
    
    CHECK_OPTION_BOOL(reduce_1pps_sync_error);
    
    CHECK_OPTION(out, false,
        {cerr << "out: "; _out = &(spec2ostream(value)); return true;},
        "");
    CHECK_OPTION(out_debug, false,
        {cerr << "out_debug: "; _out_debug = &(spec2ostream(value)); return true;},
        "");
    
    CHECK_ALIAS(direct_sylphide);
    CHECK_OPTION_BOOL(in_sylphide);

    CHECK_OPTION_BOOL(out_sylphide);
#undef CHECK_OPTION_BOOL
#undef CHECK_OPTION
    return false;
  }
};

template <class FloatT>
class NAVData {
  public:
    virtual FloatT longitude() const = 0;
    virtual FloatT latitude() const = 0;
    virtual FloatT height() const = 0;
    virtual FloatT v_north() const = 0;
    virtual FloatT v_east() const = 0;
    virtual FloatT v_down() const = 0;
    virtual FloatT heading() const = 0;
    virtual FloatT euler_phi() const = 0;
    virtual FloatT euler_theta() const = 0;
    virtual FloatT euler_psi() const = 0;
    virtual FloatT azimuth() const = 0;
    
    /**
     * Print label
     * 
     */
    virtual void label(std::ostream &out = std::cout) const {
      out << "longitude"
          << ',' << "latitude"
          << ',' << "height"
          << ',' << "v_north"
          << ',' << "v_east"
          << ',' << "v_down"
          << ',' << "Yaw(psi)"
          << ',' << "Pitch(theta)"
          << ',' << "Roll(phi)"
          << ',' << "Azimuth(alpha)";
    }
    
  protected:
    /**
     * Print current state
     * 
     */
    virtual void dump(std::ostream &out) const {
      out << rad2deg(longitude())
          << ',' << rad2deg(latitude())
          << ',' << height()
          << ',' << v_north()
          << ',' << v_east()
          << ',' << v_down()
          << ',' << rad2deg(heading())      // yaw   <- q_{g}^{b}
          << ',' << rad2deg(euler_theta())  // pitch <- q_{n}^{b}
          << ',' << rad2deg(euler_phi())    // roll  <- q_{n}^{b}
          << ',' << rad2deg(azimuth());     // azimuth)
    }
    
  public:
    /**
     * Print current state
     * 
     */
    friend std::ostream &operator<<(std::ostream &out, const NAVData<FloatT> &nav){
      nav.dump(out);
      return out;
    }
    
    /**
     * Make N0 packet
     * 
     * @param itow Time stamp
     * @param buf buffer to store results
     */
    void encode_N0(
        const FloatT &itow,
        char buf[32]) const {
      typedef unsigned int v_u32_t;
      typedef int v_s32_t;
      typedef short v_s16_t;
      
      v_u32_t t(itow * 1000);
      v_s32_t lat(rad2deg(latitude()) * 1E7), 
          lng(rad2deg(longitude()) * 1E7), 
          h(height() * 1E4);
      v_s16_t v_n(v_north() * 1E2), 
          v_e(v_east() * 1E2), 
          v_d(v_down() * 1E2);
      v_s16_t psi(rad2deg(heading()) * 1E2), 
          theta(rad2deg(euler_theta()) * 1E2), 
          phi(rad2deg(euler_phi()) * 1E2);
      buf[0] = 'N';
      buf[1] = '\0';
      buf[2] = '\0';
      buf[3] = '\0';
      *(v_u32_t *)(&buf[4]) = le_char4_2_num<v_u32_t>(*(const char *)&t);
      *(v_s32_t *)(&buf[8]) = le_char4_2_num<v_s32_t>(*(const char *)&lat);
      *(v_s32_t *)(&buf[12]) = le_char4_2_num<v_s32_t>(*(const char *)&lng);
      *(v_s32_t *)(&buf[16]) = le_char4_2_num<v_s32_t>(*(const char *)&h);
      *(v_s16_t *)(&buf[20]) = le_char4_2_num<v_s16_t>(*(const char *)&v_n);
      *(v_s16_t *)(&buf[22]) = le_char4_2_num<v_s16_t>(*(const char *)&v_e);
      *(v_s16_t *)(&buf[24]) = le_char4_2_num<v_s16_t>(*(const char *)&v_d);
      *(v_s16_t *)(&buf[26]) = le_char4_2_num<v_s16_t>(*(const char *)&psi);
      *(v_s16_t *)(&buf[28]) = le_char4_2_num<v_s16_t>(*(const char *)&theta);
      *(v_s16_t *)(&buf[30]) = le_char4_2_num<v_s16_t>(*(const char *)&phi);
    }

    virtual FloatT time_stamp() const {return 0;}

    void encode_N0(char buf[32]) const {
      encode_N0(time_stamp(), buf);
    }

};

#endif
