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
#include <ctime>
#include <cctype>
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

static std::time_t utc2time_t(
    const int &year, const int &month, const int &mday,
    const int &hour = 0, const int &min = 0, const int &sec = 0){
  std::tm utc;
  utc.tm_sec = sec;
  utc.tm_min = min;
  utc.tm_hour = hour;
  utc.tm_mday = mday;
  utc.tm_mon = month - 1;
  utc.tm_year = year - 1900;
  utc.tm_isdst = 0;
  return
#if defined(_MSC_VER)
      _mkgmtime(&utc);
#else
      timegm(&utc);
#endif
}

template <class FloatT>
struct GlobalOptions {
  protected:
  GlobalOptions(const GlobalOptions<FloatT> &);
  GlobalOptions<FloatT> &operator=(const GlobalOptions<FloatT> &);

  public:
  struct gps_time_t {
    FloatT sec; ///< GPS time
    int wn; ///< GPS week number
    static const int WN_INVALID = -1;
    gps_time_t(const FloatT &_sec = 0, const int &_wn = WN_INVALID)
        : sec(_sec), wn(_wn) {}
    template <class T1, class T2>
    bool is_before(const T1 &base_sec, const T2 &base_wn) const {
      return ((WN_INVALID >= wn) || (base_wn == wn))
          ? (base_sec >= sec)
          : (base_wn > wn);
    }
    template <class T1, class T2>
    bool is_after(const T1 &base_sec, const T2 &base_wn) const {
      return ((WN_INVALID >= wn) || (base_wn == wn))
          ? (base_sec <= sec)
          : (base_wn < wn);
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
      : start_gpstime(0), end_gpstime(DBL_MAX),
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
    prefix ## _gpstime.wn = gps_time_t::WN_INVALID; \
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

  template <class CalibrationT>
  static bool set_typical_calibration_specs(CalibrationT &target){
    static const char *specs[] = { // NinjaScan default calibration parameters
      "index_base 0",
      "index_temp_ch 8",
      "acc_bias 32768 32768 32768",
      "acc_bias_tc 0 0 0",  // No temperature compensation
      "acc_sf 4.1767576e+2 4.1767576e+2 4.1767576e+2",  // MPU-6000/9250 8[G] full scale; (1<<15)/(8*9.80665) [1/(m/s^2)]
      "acc_mis 1 0 0 0 1 0 0 0 1",  // No misalignment compensation
      "gyro_bias 32768 32768 32768",
      "gyro_bias_tc 0 0 0",  // No temperature compensation
      "gyro_sf 9.3873405e+2 9.3873405e+2 9.3873405e+2",  // MPU-6000/9250 2000[dps] full scale; (1<<15)/(2000/180*PI) [1/(rad/s)]
      "gyro_mis 1 0 0 0 1 0 0 0 1",  // No misalignment compensation
      "sigma_accel 0.05 0.05 0.05",  // approx. 150[mG] ? standard deviation
      "sigma_gyro 5e-3 5e-3 5e-3",  // approx. 0.3[dps] standard deviation
    };
    return target.check_specs(specs, get_value2);
  }

  template <class CalibrationT>
  bool load_calibration_file(CalibrationT &target, const char *fname){
    std::cerr << "IMU Calibration file (" << fname << ") reading..." << std::endl;
    std::istream &in(spec2istream(fname));
    char buf[1024];
    while(!in.eof()){
      in.getline(buf, sizeof(buf));
      if(!buf[0]){continue;}
      if(!target.check_spec(buf, get_value2)){
        std::cerr << "unknown_calib_param! : " << buf << std::endl;
        return false;
      }
    }
    return true;
  }
};

template <class FloatT>
struct CalendarTime {
  typedef FloatT float_t;
  int year, month, mday, hour, min;
  float_t sec;

  typedef typename GlobalOptions<FloatT>::gps_time_t gps_time_t;

  struct Converter {
    enum {
      LEAP_SECONDS_UNKNOWN,
      LEAP_SECONDS_ESTIMATED,
      LEAP_SECONDS_CORRECTED
    } leap_seconds;
    gps_time_t gps_time; ///< base GPS time
    std::time_t utc_time; ///< corresponding base UTC time in time_t
    int correction_sec;
    static const std::time_t gps_time_zero;

    mutable struct roll_over_monitor_t {
      static const int one_week = 60 * 60 * 7 * 24;
      static const int threshold = 60 * 30; // 30min
      int roll_over_offset;
      float_t itow_previous;
      roll_over_monitor_t() : roll_over_offset(0), itow_previous(0) {}
      float_t operator()(const float_t &itow){
        float_t delta(itow - itow_previous);
        do{
          if(delta < -(one_week / 2)){ // detect backward super jump (now << previous)
            if(itow_previous > (one_week - threshold)){ // roll over
              roll_over_offset += one_week;
              break;
            }
          }else if(delta > (one_week / 2)){ // detect forward super jump (previous << now)
            if(itow_previous < threshold){ // roll over (backward)
              roll_over_offset -= one_week;
              break;
            }
          }else if((delta > -threshold) && (delta < threshold)){ // normal
            break;
          }
          // otherwise probably reset occurred
          roll_over_offset = 0;
        }while(false);
        itow_previous = itow;
        return itow + roll_over_offset;
      }
    } roll_over_monitor;

    Converter()
        : gps_time(0),
        leap_seconds(LEAP_SECONDS_UNKNOWN),
        correction_sec(0),
        roll_over_monitor() {}

    CalendarTime convert(const std::time_t &in, const float_t &subsec = 0) const {
      tm *t(std::gmtime(&in));
      CalendarTime res = {
          t->tm_year + 1900,
          t->tm_mon + 1,
          t->tm_mday,
          t->tm_hour,
          t->tm_min,
          subsec + t->tm_sec};
      return res;
    }
    CalendarTime convert(const float_t &itow) const {
      if(gps_time.wn != gps_time_t::WN_INVALID){
        float_t gap(roll_over_monitor(itow) - gps_time.sec);
        int gap_sec(std::floor(gap));
        return convert(std::time_t(utc_time + gap_sec + correction_sec), gap - gap_sec);
      }else{
        CalendarTime res = {0, 0, 0, 0, 0, roll_over_monitor(itow)};
        return res;
      }
    }
    static int estimate_leap_sec(const std::time_t &t_gps){
      static const struct {
        std::time_t t;
        int sec;
      } leap_seconds_db[] = {
#define make_entry(y, m, d, sec) {utc2time_t(y, m, d) + sec, sec}
        make_entry(1981, 7, 1,  1),
        make_entry(1982, 7, 1,  2),
        make_entry(1983, 7, 1,  3),
        make_entry(1985, 7, 1,  4),
        make_entry(1988, 1, 1,  5),
        make_entry(1990, 1, 1,  6),
        make_entry(1991, 1, 1,  7),
        make_entry(1992, 7, 1,  8),
        make_entry(1993, 7, 1,  9),
        make_entry(1994, 7, 1, 10),
        make_entry(1996, 1, 1, 11),
        make_entry(1997, 7, 1, 12),
        make_entry(1999, 1, 1, 13),
        make_entry(2006, 1, 1, 14),
        make_entry(2009, 1, 1, 15),
        make_entry(2012, 7, 1, 16),
        make_entry(2015, 7, 1, 17),
        make_entry(2017, 1, 1, 18),
#undef make_entry
      };

      int res(0);
      for(int i(0); i < sizeof(leap_seconds_db) / sizeof(leap_seconds_db[0]); ++i){
        if(t_gps < leap_seconds_db[i].t){break;}
        res = leap_seconds_db[i].sec;
      }
      return res;
    }
    void update(const float_t &gps_sec){
      gps_time.sec = gps_sec;
      gps_time.wn = gps_time_t::WN_INVALID;
    }
    void update(const float_t &gps_sec, const int &gps_wn, const int &leap_sec){
      gps_time.sec = gps_sec;
      if(gps_time.wn != gps_time_t::WN_INVALID){
        // GPS second will be greater than one week to assume roll over of itow
        gps_time.sec += (gps_wn - gps_time.wn) * roll_over_monitor_t::one_week;
      }else{
        gps_time.wn = gps_wn;
      }
      utc_time = gps_time_zero
          + (7u * 24 * 60 * 60) * gps_time.wn
          + gps_time.sec - leap_sec; // POSIX time ignores leap seconds.
      leap_seconds = LEAP_SECONDS_CORRECTED;
    }
    void update(const float_t &gps_sec, const int &gps_wn){
      update(gps_sec, gps_wn, 0);
      utc_time -= estimate_leap_sec(utc_time);
      leap_seconds = LEAP_SECONDS_ESTIMATED;
    }
    CalendarTime convert(const gps_time_t &current_gps) const {
      if(current_gps.wn == gps_time_t::WN_INVALID){
        CalendarTime res = {0, 0, 0, 0, 0, roll_over_monitor(current_gps.sec)};
        return res;
      }else if(gps_time.wn == gps_time_t::WN_INVALID){
        // use estimated leap seconds
        float_t gap(gps_time_zero + current_gps.wn * roll_over_monitor_t::one_week + current_gps.sec);
        gap -= estimate_leap_sec(std::time_t(gap));
        int gap_sec(std::floor(gap));
        return convert(std::time_t(gap_sec + correction_sec), gap - gap_sec);
      }else{
        // use internal correction value for leap seconds
        float_t gap(
            (current_gps.wn - gps_time.wn) * roll_over_monitor_t::one_week
              + (current_gps.sec - gps_time.sec));
        int gap_sec(std::floor(gap));
        return convert(std::time_t(utc_time + gap_sec + correction_sec), gap - gap_sec);
      }
    }
  };
};

template <class FloatT>
const std::time_t CalendarTime<FloatT>::Converter::gps_time_zero
    = utc2time_t(1980, 1, 6); // 1980/1/6 00:00:00

template <class FloatT>
class NAVData {
  public:
    virtual ~NAVData(){}
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
