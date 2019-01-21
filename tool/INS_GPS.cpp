/**
 * @file INS/GPS post-processor for NinjaScan
 *
 */

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

/*
 * === Quick guide ===
 *
 * This program is used to analyze data gathered with NinjaScan logger
 * by using Kalman filter based integrated navigation technology
 * called INS/GPS, which stands for inertial navigation system (INS)
 * and global positioning system (GPS).
 * The program outputs position (longitude, latitude, and WGS84 altitude),
 * velocity (north, east, and down), and attitude (true heading, roll and pitch angles)
 * with a time stamp.
 *
 * It is briefly technically noted that this program utilizes loosely-coupled INS/GPS algorithm,
 * which implies at least four GPS satellites must be available to output the results.
 * In addition, this program is not suitable to be used for real-time application.
 * This is because its processing strategy is post-process, that is, the data is sorted
 * in time-series order before application of the INS/GPS algorithm,
 * in order to compensate for the output delay of a GPS receiver installed on the logger.
 *
 * Its usage is
 *   INS_GPS [option(s)] <log.dat>,
 * where <log.dat> is a mandatory parameter pointing to a log file stored in a logger.
 * There are some reserved values for <log.dat>; If <log.dat> equals to - (hyphen),
 * the program will try to read log data from the standard input.
 * Or, when <log.dat> is COMx for Windows or /dev/ttyACMx for *NIX,
 * the program will try to read data from the specified serial port.
 *
 * The [option(s)] is optional parameter(s).
 * If multiple parameters are specified, they should be separated by space.
 * The representative parameters are the followings;
 *
 *   --start_gpst=(GPS time in week [sec])
 *      specifies start GPS time for INS/GPS post-process.
 *   --start_gpst=(GPS week):(GPS time in week [sec])
 *      specifies start GPS week and time for INS/GPS post-process.
 *
 *   --end_gpst=(GPS time in week [sec])
 *      specifies end GPS time for INS/GPS post-process.
 *   --end_gpst=(GPS week):(GPS time in week [sec])
 *      specifies end GPS week and time for INS/GPS post-process.
 *
 *   --dump_update=<on|off>
 *      specifies whether the program outputs results when inertial information is obtained
 *      (so called, results for time update), or not. Its default is on.
 *   --dump_correct=<off|on>
 *      specifies whether the program outputs results when information processed by a GPS receiver
 *      is obtained, (so called, results for measurement update) or not. Its default is off.
 *
 *   --calendar_time
 *      changes time stamp of output from (internal) GPS time of week (default)
 *      to year, month, day of month, hour, minute, and second.
 *
 *   --init_attitude_deg=(heading [deg]),(pitch [deg]),(roll [deg])
 *      specifies initial true heading, pitch and roll angles. Their default values are
 *      computed by using Earth magnetic force and gravity observed with magnetic sensor
 *      and accelerometer, respectively, under the assumption that the logger may be stationary
 *      at the beginning. While the default initial pitch and roll angles can be approximately
 *      true because Earth's gravity is comparatively large, the default true heading is not
 *      so reliable because magnetic field is easily disturbed by surrounding metal objects
 *      or electric current. Therefore, to specify the initial heading angle is strongly
 *      recommended. Please also check the next --init_yaw_deg option, which only specifies
 *      the initial heading angle.
 *   --init_yaw_deg=(heading [deg])
 *      specifies initial true heading in degree. Please also refer the above explanation
 *      about --init_attitude_deg.
 *
 *   --est_bias=<on|off>
 *      specifies whether the mechanism to estimate sensor bias drift is utilized, or not.
 *      The default is on.
 *   --use_udkf=<off|on>
 *      specifies whether the UD factorized Kalamn filter (UDKF), or the standard Kalman
 *      filter is utilized. The default is off (standard KF).
 *
 *   --direct_sylphide=<off|on>
 *   --in_sylphide=<off|on>
 *      specifies whether the format of the input log file follows Sylphide protocol or not.
 *      The default is off. Please use this option with "on" value when the program directly read
 *      data from the NinjaScan logger in USB CDC Mode.
 *
 *   --gps_init_acc_2d=(sigma [m])
 *   --gps_init_acc_v=(sigma [m])
 *   --gps_cont_acc_2d=(sigma [m])
 *      specify initial measurement update threshold for GPS 2D estimated error,
 *      initial measurement update threshold for GPS vertical estimated error,
 *      and continual measurement update threshold for GPS 2D estimated error, respectively.
 *      These default values are 20, 10, and 100, respectively.
 *
 * The followings are advanced (i.e., very experimental) options;
 *   --back_propagate
 *      apply Kalman filter smoothing to previously time-updated data
 *      (exclusive with --realtime)
 *   --realtime
 *      change GPS synchronization strategy to support realtime applications.
 *      It processes data without sorting and outputs calculation results as quick as possible.
 *      (exclusive with --back_propagate)
 *
 */

// Comment-In when QNAN DEBUG
//#include <float.h>
//unsigned int fp_control_state = _controlfp(_EM_INEXACT, _MCW_EM);

#include <iostream>
#include <sstream>
#include <iomanip>
#include <string>
#include <exception>

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <cctype>

#include <vector>
#include <map>
#include <utility>
#include <deque>
#include <algorithm>

#define IS_LITTLE_ENDIAN 1
#include "SylphideStream.h"
#include "SylphideProcessor.h"

typedef double float_sylph_t;

#include "param/matrix.h"
#include "param/vector3.h"
#include "param/quaternion.h"
#include "param/complex.h"

template <>
struct Vector3Data_TypeMapper<float_sylph_t> {
  typedef Vector3Data_NoFlyWeight<float_sylph_t> res_t;
};
template <>
struct QuaternionData_TypeMapper<float_sylph_t> {
  typedef QuaternionData_NoFlyWeight<float_sylph_t> res_t;
};

#include "algorithm/kalman.h"

#include "navigation/INS_GPS_Factory.h"
#include "navigation/INS_GPS_Synchronization.h"
#include "navigation/INS_GPS_Debug.h"
#include "navigation/GPS.h"
#include "navigation/GPS_Solver.h"
#include "navigation/RINEX.h"

#include "navigation/MagneticField.h"

#include "analyze_common.h"

#include "INS_GPS/GNSS_Data.h"

struct Options : public GlobalOptions<float_sylph_t> {
  typedef GlobalOptions<float_sylph_t> super_t;

  // Output
  bool dump_update; ///< True for dumping states at time updates
  bool dump_correct; ///< True for dumping states at measurement updates
  bool dump_stddev; ///< True for dumping standard deviations
  bool out_is_N_packet; ///< True for NPacket formatted outputs

  // Time Stamp
  struct time_stamp_t {
    enum mode_t {
      ITOW,
      CALENDAR_TIME,
    } mode;
    const char *spec;
    time_stamp_t()
        : mode(ITOW), spec(NULL) {}
    struct calendar_spec_parsed_t {
      int correction_hr;
      friend std::ostream &operator<<(std::ostream &out, const calendar_spec_parsed_t &parsed){
        out << "UTC";
        if(parsed.correction_hr != 0){
          out << (parsed.correction_hr > 0 ? " +" : " ")
              << parsed.correction_hr << " [hr]";
        }
        return out;
      }
    };
    calendar_spec_parsed_t calendar_spec_parse() const {
      calendar_spec_parsed_t res = {0};
      if(is_true(spec)){return res;}
      char *spec_end;
      res.correction_hr = std::strtol(spec, &spec_end, 10);
      if(spec == spec_end){
        std::cerr << "Invalid spec for --calendar_time[=(+/-hr)]: " << spec << std::endl;
        exit(-1);
      }
      return res;
    }
  } time_stamp;

  // Navigation strategies
  enum {
    INS_GPS_INTEGRATION_LOOSELY,
    INS_GPS_INTEGRATION_TIGHTLY,
  } ins_gps_integration; ///< integration style.
  enum {
    INS_GPS_SYNC_OFFLINE,
    INS_GPS_SYNC_BACK_PROPAGATION, ///< a.k.a, smoothing
    INS_GPS_SYNC_REALTIME,
  } ins_gps_sync_strategy;
  bool est_bias; ///< True for performing bias estimation
  bool use_udkf; ///< True for UD Kalman filtering
  bool use_egm; ///< True for precise Earth gravity model

  INS_GPS_Back_Propagate_Property<float_sylph_t> back_propagate_property;
  INS_GPS_RealTime_Property realttime_property;

  // GPS options
  bool gps_fake_lock; ///< true when gps dummy date is used.
  struct gps_threshold_t {
    float_sylph_t init_acc_2d; ///< Initial measurement update threshold for GPS 2D estimated error
    float_sylph_t init_acc_v;  ///< Initial measurement update threshold for GPS vertical estimated error
    float_sylph_t cont_acc_2d; ///< Continual measurement update threshold for GPS 2D estimated error
    gps_threshold_t()
        : init_acc_2d(20.), init_acc_v(10.),
        cont_acc_2d(100.) {}
  } gps_threshold;

  // Magnetic sensor
  bool use_magnet; ///< True for utilizing magnetic sensor
  float_sylph_t mag_heading_accuracy_deg; ///< Accuracy of magnetic sensor in degrees
  float_sylph_t yaw_correct_with_mag_when_speed_less_than_ms; ///< Threshold for yaw compensation; performing it when under this value [m/s], or ignored non-positive values

  // Manual initialization
  struct initial_attitude_t {
    float_sylph_t yaw_deg, pitch_deg, roll_deg; ///< Initial attitude [deg] (yaw, pitch, roll)
    enum mode_t {NOT_GIVEN = 0, YAW_ONLY = 1, YAW_PITCH = 2, FULL_GIVEN = 3} mode; ///< Whether initial attitude is given.
    initial_attitude_t()
        : yaw_deg(0), pitch_deg(0), roll_deg(0),
        mode(NOT_GIVEN) {}
    initial_attitude_t &parse(const char *spec){
      int converted(std::sscanf(spec, "%lf,%lf,%lf",
          &yaw_deg, &pitch_deg, &roll_deg));
      if(converted > 0){mode = (mode_t)converted;}
      return *this;
    }
    initial_attitude_t &parse_yaw(const char *spec){
      yaw_deg = std::atof(spec);
      mode = YAW_ONLY;
      return *this;
    }
    friend std::ostream &operator<<(std::ostream &out, const initial_attitude_t &atti){
      return out << "(yaw, pitch, roll) (args:"
          << atti.mode << "): "
          << atti.yaw_deg << ", "
          << atti.pitch_deg << ", "
          << atti.roll_deg;
    }
  } initial_attitude;
  std::istream *init_misc; ///< other manual initialization
  std::stringstream init_misc_buf; ///< buffer for init_misc string
  std::ostream *out_raw_pvt;

  // Debug
  INS_GPS_Debug_Property debug_property;

  Options()
      : super_t(),
      dump_update(true), dump_correct(false), dump_stddev(false),
      out_is_N_packet(false),
      time_stamp(),
      ins_gps_integration(INS_GPS_INTEGRATION_LOOSELY), ins_gps_sync_strategy(INS_GPS_SYNC_OFFLINE),
      est_bias(true), use_udkf(false), use_egm(false),
      back_propagate_property(),
      realttime_property(),
      gps_fake_lock(false), gps_threshold(),
      use_magnet(false),
      mag_heading_accuracy_deg(3),
      yaw_correct_with_mag_when_speed_less_than_ms(5),
      initial_attitude(),
      init_misc_buf(), init_misc(&init_misc_buf),
      out_raw_pvt(NULL),
      debug_property() {
    realttime_property.rt_mode = INS_GPS_RealTime_Property::RT_LIGHT_WEIGHT;
  }
  ~Options(){}

  /**
   * Check spec
   * 
   * @param spec command
   * @return (bool) true when consumed, otherwise false
   */
  bool check_spec(const char *spec){

    const char *key;
    const unsigned int key_length(get_key(spec, &key));
    if(key_length == 0){return super_t::check_spec(spec);}

    bool key_checked(false);

#define CHECK_KEY(name) \
  (key_checked \
    || (key_checked = ((key_length == std::strlen(#name)) \
        && (std::strncmp(key, #name, key_length) == 0))))
#define CHECK_ALIAS(name) CHECK_KEY(name)
#define CHECK_OPTION(name, accept_no_value, operation, disp) { \
  while(CHECK_KEY(name)){ \
    key_checked = false; \
    const char *value(get_value(spec, key_length, accept_no_value)); \
    if((!accept_no_value) && (!value)){return false;} \
    {operation;} \
    std::cerr << #name << ": " << disp << std::endl; \
    return true; \
  } \
}
#define CHECK_OPTION_BOOL(target) \
CHECK_OPTION(target, true, target = is_true(value), (target ? "on" : "off"));

    CHECK_ALIAS(dump-update);
    CHECK_OPTION_BOOL(dump_update);
    CHECK_ALIAS(dump-correct);
    CHECK_OPTION_BOOL(dump_correct);
    CHECK_OPTION_BOOL(dump_stddev);
    CHECK_ALIAS(out_N_packet);
    CHECK_OPTION_BOOL(out_is_N_packet);

    CHECK_OPTION(calendar_time, true, {
          time_stamp.mode = time_stamp_t::CALENDAR_TIME;
          time_stamp.spec = value;
        }, time_stamp.calendar_spec_parse());

    CHECK_ALIAS(tight);
    CHECK_OPTION(tightly, true,
        if(is_true(value)){ins_gps_integration = INS_GPS_INTEGRATION_TIGHTLY;},
        (ins_gps_integration == INS_GPS_INTEGRATION_TIGHTLY ? "on" : "off"));
    CHECK_OPTION(back_propagate, true,
        if(is_true(value)){ins_gps_sync_strategy = INS_GPS_SYNC_BACK_PROPAGATION;},
        (ins_gps_sync_strategy == INS_GPS_SYNC_BACK_PROPAGATION ? "on" : "off"));
    CHECK_OPTION(realtime, true,
        if(is_true(value)){ins_gps_sync_strategy = INS_GPS_SYNC_REALTIME;},
        (ins_gps_sync_strategy == INS_GPS_SYNC_REALTIME ? "on" : "off"));
    CHECK_OPTION_BOOL(est_bias);
    CHECK_OPTION_BOOL(use_udkf);
    CHECK_OPTION_BOOL(use_egm);
    CHECK_OPTION(bp_depth, false,
        back_propagate_property.back_propagate_depth = std::atof(value),
        back_propagate_property.back_propagate_depth);

    CHECK_ALIAS(fake_lock);
    CHECK_OPTION_BOOL(gps_fake_lock);
    CHECK_OPTION(gps_init_acc_2d, false,
        gps_threshold.init_acc_2d = std::atof(value),
        gps_threshold.init_acc_2d << " [m]");
    CHECK_OPTION(gps_init_acc_v, false,
        gps_threshold.init_acc_v = std::atof(value),
        gps_threshold.init_acc_v << " [m]");
    CHECK_OPTION(gps_cont_acc_2d, false,
        gps_threshold.cont_acc_2d = std::atof(value),
        gps_threshold.cont_acc_2d << " [m]");

    CHECK_OPTION_BOOL(use_magnet);
    CHECK_OPTION(mag_heading_accuracy_deg, false,
        mag_heading_accuracy_deg = std::atof(value),
        mag_heading_accuracy_deg << " [deg]");
    CHECK_OPTION(yaw_correct_with_mag_when_speed_less_than_ms, false,
        yaw_correct_with_mag_when_speed_less_than_ms = std::atof(value),
        yaw_correct_with_mag_when_speed_less_than_ms << " [m/s]");

    CHECK_ALIAS(init-attitude-deg);
    if(CHECK_KEY(init_attitude_deg)){
      const char *value(get_value(spec, key_length, false));
      if(!value){return false;}
      std::cerr.write(key, key_length)
          << " " << initial_attitude.parse(value) << std::endl;
      return true;
    }
    CHECK_ALIAS(init-yaw-deg);
    CHECK_OPTION(init_yaw_deg, false,
        initial_attitude.parse_yaw(value),
        initial_attitude.yaw_deg << " [deg]");

    CHECK_OPTION(init_misc, false,
        {init_misc_buf << value << std::endl; return true;},
        "");
    CHECK_OPTION(init_misc_fname, false,
        {std::cerr << "Checking... "; init_misc = &spec2istream(value);},
        value);
    CHECK_OPTION(out_raw_pvt, false,
        {out_raw_pvt = &spec2ostream(value);},
        value);

    CHECK_OPTION(debug, false,
        if(!debug_property.check_debug_property_spec(value)){break;},
        debug_property.show_debug_property());
#undef CHECK_OPTION
    
    return super_t::check_spec(spec);
  }
} options;

template <class FloatT>
struct CalendarTimeStamp : public CalendarTime<FloatT> {
  typedef CalendarTime<FloatT> super_t;
#if defined(__GNUC__) && (__GNUC__ < 5)
  typedef typename super_t::float_t float_t;
#else
  using typename super_t::float_t;
#endif
  float_t itow;
  CalendarTimeStamp &operator=(const float_t &t){
    super_t::year = super_t::month = super_t::mday = super_t::hour = super_t::min = 0;
    super_t::sec = itow = t;
    return *this;
  }
  CalendarTimeStamp(const super_t &t1, const float_t &t2)
      : super_t(t1), itow(t2) {}
  CalendarTimeStamp(const float_t &t = 0) : super_t(), itow(t) {
    operator=(t);
  }
  operator float_t() const {return itow;}
  static void label(std::ostream &out){
    out << "year" << ','
        << "month" << ','
        << "day" << ','
        << "hour" << ','
        << "min" << ','
        << "sec";
  }
  friend std::ostream &operator<<(std::ostream &out, const CalendarTimeStamp &time){
    out << time.year << ','
        << time.month << ','
        << time.mday << ','
        << time.hour << ','
        << time.min << ','
        << time.sec;
    return out;
  }
};

struct A_Packet;
struct G_Packet;
struct G_Packet_Measurement;
struct G_Packet_Data;
struct G_Packet_GPS_Ephemeris;
struct M_Packet;
struct TimePacket;

struct Updatable {
  virtual ~Updatable() {}
  virtual void update(const A_Packet &){}
  virtual void update(const G_Packet &){}
  virtual void update(const G_Packet_Measurement &){}
  virtual void update(const G_Packet_Data &);
  virtual void update(const G_Packet_GPS_Ephemeris &);
  virtual void update(const M_Packet &){}
  virtual void update(const TimePacket &){}
} updatable_blackhole;

class NAV : public Updatable {
  public:
    typedef NAVData<float_sylph_t> data_t;
    typedef std::vector<const data_t *> updated_items_t;
    virtual ~NAV(){}
  public:
    virtual void label(std::ostream &out) const = 0;
    virtual updated_items_t updated_items() const {
      return updated_items_t();
    }
    virtual void inspect(std::ostream &out) const {}
    virtual float_sylph_t &operator[](const unsigned &index) = 0;

    template <class Container>
    static typename Container::const_iterator nearest(
        const Container &packets_time_series, const float_sylph_t &itow,
        const unsigned int &group_size = 1){
      typename Container::const_iterator
          it_head(packets_time_series.begin()),
          it_eval(it_head + (group_size / 2)),
          it_end(packets_time_series.end());
      for(int i(distance(it_head, it_end));
          i > group_size;
          i--, it_head++, it_eval++){
        if(it_eval->itow >= itow){break;}
      }
      return it_head;
    }

    /**
     * Estimate yaw correction angle by using magnetic sensor values
     *
     * @param attitude
     * @return yaw correction angle [rad]
     */
    static float_sylph_t get_mag_delta_yaw(
        const Vector3<float_sylph_t> &mag,
        const Quaternion<float_sylph_t> &attitude,
        const float_sylph_t &latitude, const float_sylph_t &longitude, const float_sylph_t &altitude){

      typedef Vector3<float_sylph_t> vec_t;
      typedef Quaternion<float_sylph_t> quat_t;

      // Cancel attitude (inverted)
      vec_t mag_horizontal((attitude * quat_t(0, mag) * attitude.conj()).vector());

      // Call Earth's magnetic field model
      MagneticField::field_components_res_t mag_model(
          MagneticField::field_components(IGRF12::IGRF2015,
              latitude, longitude, altitude));
      vec_t mag_field(mag_model.north, mag_model.east, mag_model.down);

      // Get the correction angle with the model
      return std::atan2(mag_field[1], mag_field[0])
          - std::atan2(mag_horizontal[1], mag_horizontal[0]);
    }

    float_sylph_t get_mag_delta_yaw(
        const Vector3<float_sylph_t> &mag,
        const Quaternion<float_sylph_t> &attitude,
        const data_t &data){

      return get_mag_delta_yaw(mag, attitude, data.latitude(), data.longitude(), data.height());
    }

    float_sylph_t get_mag_delta_yaw(
        const Vector3<float_sylph_t> &mag,
        const data_t &data){

      return get_mag_delta_yaw(mag,
          INS<float_sylph_t>::euler2q(data.euler_psi(), data.euler_theta(), data.euler_phi()),
          data.latitude(), data.longitude(), data.height());
    }

    /**
     * Estimate yaw angle
     *
     */
    static float_sylph_t get_mag_yaw(
        const Vector3<float_sylph_t> &mag,
        const float_sylph_t &pitch, const float_sylph_t &roll,
        const float_sylph_t &latitude, const float_sylph_t &longitude, const float_sylph_t &altitude){

      return get_mag_delta_yaw(
          mag,
          INS<float_sylph_t>::euler2q(0, pitch, roll),
          latitude, longitude, altitude);
    }

    float_sylph_t get_mag_yaw(
        const Vector3<float_sylph_t> &mag,
        const data_t &data){

      return get_mag_yaw(
          mag,
          data.euler_theta(), data.euler_phi(),
          data.latitude(), data.longitude(), data.height());
    }
};

template <class BaseNAV>
struct NAV_Factory {
  typedef BaseNAV self_t;

  struct NAVDisplay : public BaseNAV {
    NAVDisplay() : BaseNAV() {}
    void label(std::ostream &out = std::cout) const {
      if(options.out_is_N_packet){return;}
      BaseNAV::label(options.out());
      options.out() << std::endl;
    }
    void updated() const {
      const NAV::updated_items_t &items(BaseNAV::updated_items());
      if(items.empty()){return;}

      for(NAV::updated_items_t::const_iterator it(items.begin());
          it != items.end(); ++it){
        if(options.out_is_N_packet){
          char buf[SYLPHIDE_PAGE_SIZE];
          (*it)->encode_N0(buf);
          options.out().write(buf, sizeof(buf));
          return;
        }else{
          options.out() << (**it) << std::endl;
        }
      }

      options.out_debug() << (**(items.rbegin())).time_stamp() << ',';
      BaseNAV::inspect(options.out_debug());
      options.out_debug() << std::endl;
    }
#define update_func(type) \
virtual void update(const type &packet){ \
  BaseNAV::update(packet); \
  updated(); \
}
    update_func(A_Packet);
    update_func(G_Packet);
    update_func(G_Packet_Measurement);
    update_func(M_Packet);
#undef update_func
  };

  typedef NAVDisplay disp_t;
};

/**
 * Base class of log data, which has time stamp.
 */
struct Packet{
  virtual ~Packet() {}
  virtual void apply(NAV &nav) const = 0;

  float_sylph_t itow;

  /**
   * Get interval time between another one
   *
   * @param another
   * @return if another is bigger, positive number will be returned.
   */
  float_sylph_t interval(const Packet &another) const {
    return another.itow - itow;
  }
  static bool compare(const Packet *a, const Packet *b) {
    return a->itow < b->itow;
  }
  /**
   * Get interval time between another one
   * in consideration of one week roll over
   *
   * @param another
   * @return The returned number range is [-one_week/2, +one_week/2)
   * if another is bigger, positive number will be returned.
   */
  float_sylph_t interval_rollover(const Packet &another) const {
    float_sylph_t delta(another.itow - itow);
    static const int one_week(60 * 60 * 24 * 7);
    return delta - (std::floor((delta / one_week) + 0.5) * one_week);
  }
  static bool compare_rollover(const Packet *a, const Packet *b) {
    return a->interval_rollover(*b) > 0;
  }
};

template <class T>
struct BasicPacket : public Packet {
  void apply(NAV &nav) const {
    nav.update(static_cast<const T &>(*this));
  }
};

/**
 * Inertial and temperature sensor data (ADC raw value)
 */
struct A_Packet : public BasicPacket<A_Packet> {
  Vector3<float_sylph_t> accel; ///< Acceleration
  Vector3<float_sylph_t> omega; ///< Angular speed
};

/**
 * GPS data
 */
struct G_Packet : public BasicPacket<G_Packet> {
  GPS_Solution<float_sylph_t> solution;
  Vector3<float_sylph_t> *lever_arm;

  G_Packet() : solution(), lever_arm(NULL) {}

  operator const GPS_Solution<float_sylph_t> &() const {
    return solution;
  }
};

struct G_Packet_Measurement : public BasicPacket<G_Packet_Measurement> {
  typedef GPS_RawData<float_sylph_t> raw_data_t;
  raw_data_t raw_data;
  Vector3<float_sylph_t> *lever_arm;

  operator const raw_data_t &() const {
    return raw_data;
  }

  void take_consistency() const {
    // Select most preferable ephemeris
    const_cast<raw_data_t::space_node_t &>(raw_data.solver->space_node())
        .update_all_ephemeris(raw_data.gpstime);
  }

  typedef raw_data_t::solver_t::user_pvt_t pvt_t;
  bool get_pvt(pvt_t &pvt) const {
    while(true){
      if(pvt.error_code == pvt_t::ERROR_NO){
        float_sylph_t delta_t(std::abs(raw_data.gpstime - pvt.receiver_time));
        if(delta_t < 5E-3){ // 5 ms
          return true; // already updated
        }else if(delta_t < 300){ // 300 sec
          pvt = raw_data.solver->solve_user_pvt(
              raw_data.measurement_of(raw_data_t::L1_PSEUDORANGE),
              raw_data.measurement_of(raw_data_t::L1_RANGE_RATE),
              raw_data.gpstime,
              pvt.user_position,
              pvt.receiver_error);
          break;
        }
      }
      pvt = raw_data.solver->solve_user_pvt(
          raw_data.measurement_of(raw_data_t::L1_PSEUDORANGE),
          raw_data.measurement_of(raw_data_t::L1_RANGE_RATE),
          raw_data.gpstime);
      break;
    }
    return pvt.error_code == pvt_t::ERROR_NO;
  }

  operator G_Packet() const {
    G_Packet res;
    res.itow = this->itow;
    res.lever_arm = this->lever_arm;
    pvt_t pvt;
    if(get_pvt(pvt)){
      // TODO calculation of estimated accuracy
      res.solution.v_n = pvt.user_velocity_enu.north();
      res.solution.v_e = pvt.user_velocity_enu.east();
      res.solution.v_d = -pvt.user_velocity_enu.up();
      //res.solution.sigma_vel;
      res.solution.latitude = pvt.user_position.llh.latitude();
      res.solution.longitude = pvt.user_position.llh.longitude();
      res.solution.height = pvt.user_position.llh.height();
      //res.solution.sigma_2d;
      //res.solution.sigma_height;
    }else{
      res.solution.sigma_vel = 1E4; // 10 km/s
      res.solution.sigma_2d = 1E6; // 1000 km
      res.solution.sigma_height = 1E6; // 1000 km
    }
    return res;
  }

  G_Packet_Measurement() : raw_data(), lever_arm(NULL) {}
};

struct gps_pvt_t : public G_Packet_Measurement::pvt_t {
  gps_pvt_t() : G_Packet_Measurement::pvt_t() {}
  static void label(std::ostream &out) {
    out << "week"
        << ',' << "itow"
        << ',' << "receiver_clock_error_meter"
        << ',' << "longitude"
        << ',' << "latitude"
        << ',' << "height"
        << ',' << "gdop"
        << ',' << "pdop"
        << ',' << "hdop"
        << ',' << "vdop"
        << ',' << "tdop"
        << ',' << "v_north"
        << ',' << "v_east"
        << ',' << "v_down";
  }

  friend std::ostream &operator<<(std::ostream &out, const gps_pvt_t &pvt){
    out << pvt.receiver_time.week
        << ',' << pvt.receiver_time.seconds
        << ',' << pvt.receiver_error
        << ',' << rad2deg(pvt.user_position.llh.longitude())
        << ',' << rad2deg(pvt.user_position.llh.latitude())
        << ',' << pvt.user_position.llh.height()
        << ',' << pvt.gdop
        << ',' << pvt.pdop
        << ',' << pvt.hdop
        << ',' << pvt.vdop
        << ',' << pvt.tdop
        << ',' << pvt.user_velocity_enu.north()
        << ',' << pvt.user_velocity_enu.east()
        << ',' << -pvt.user_velocity_enu.up();
    return out;
  }
};

struct G_Packet_Data
    : public BasicPacket<G_Packet_Data>, public GNSS_Data<float_sylph_t> {
  G_Packet_Data() : GNSS_Data<float_sylph_t>() {
    Packet::itow = 0; // to invoke immediate update
  }
};
void Updatable::update(const G_Packet_Data &packet){
  packet.loader->load(packet);
}

struct G_Packet_GPS_Ephemeris
    : public BasicPacket<G_Packet_GPS_Ephemeris>, public GNSS_Data<float_sylph_t>::Loader::gps_ephemeris_t {
  typedef GNSS_Data<float_sylph_t>::Loader::gps_t space_node_t;
  mutable space_node_t *space_node;
  typedef GNSS_Data<float_sylph_t>::Loader::gps_ephemeris_t ephemeris_t;

  G_Packet_GPS_Ephemeris() : ephemeris_t() {
    Packet::itow = 0; // to invoke immediate update
  }
};
void Updatable::update(const G_Packet_GPS_Ephemeris &packet){
  packet.space_node->satellite(packet.svid).register_ephemeris(packet);
}

/**
 * Magnetic sensor data
 */
struct M_Packet : public BasicPacket<M_Packet> {
  Vector3<float_sylph_t> mag;
};

struct TimePacket : public BasicPacket<TimePacket> {
  typedef BasicPacket<TimePacket> super_t;
  int week_num, leap_sec;
  bool valid_week_num, valid_leap_sec;
  using super_t::apply;
  template <class FloatT>
  void apply(typename CalendarTime<FloatT>::Converter &converter) const {
    valid_week_num
        ? (valid_leap_sec
            ? converter.update(super_t::itow, week_num, leap_sec)
            : converter.update(super_t::itow, week_num))
        : converter.update(super_t::itow);
  }
};

template <class INS_GPS>
class INS_GPS_NAVData;

template <class INS_GPS>
class INS_GPS_NAV : public NAV {
  public:
    typedef INS_GPS ins_gps_t;
    typedef typename ins_gps_t::float_t float_t;
    typedef typename ins_gps_t::mat_t mat_t;
    typedef typename ins_gps_t::vec3_t vec3_t;
    class Helper;
  protected:
    ins_gps_t *ins_gps;
    Helper helper;

    void setup_filter(void *){}

    template <class BaseINS, template <class> class Filter>
    void setup_filter(Filtered_INS2<BaseINS, Filter> *) {
      /**
       * Initialization of matrix P, system covariance matrix, of Kalman filter.
       * orthogonal elements are
       *  0-2 : initial velocity variance in N, E, D axes, [m/s^2]^2
       *  3-5 : initial variance of position on the Earth which is represented by a delta quaternion
       *        composed of latitude, longitude, and wander azimuth angle.
       *        For instance, 1E-8 is a sufficiently big value.
       *  6   : initial altitude variance [m]^2
       *  7-9 : initial attitude variance represented by a delta quaternion
       *        composed of yaw, pitch, and roll angles.
       *        default values are sufficiently big.
       */
      {
        mat_t P(ins_gps->getFilter().getP());

        P(0, 0) = P(1, 1) = P(2, 2) = 1E+1;
        P(3, 3) = P(4, 4) = P(5, 5) = 1E-8;
        P(6, 6) = 1E+2;
        P(7, 7) = P(8, 8) = 1E-4; // mainly for roll, pitch. 1-sigma about 1 deg.
        P(9, 9) = 5E-3; // mainly for heading. 1-sigma about 7 deg.

        ins_gps->getFilter().setP(P);
      }

      /**
       * Initialization of matrix Q, input covariance matrix, of Kalman filter.
       * orthogonal elements are
       *  0-2 : accelerometer output variance in X, Y, Z axes, [m/s^2]^2
       *  3-5 : angular speed output variance in X, Y, Z axes, [rad/s]^2
       *  6   : gravity variance [m/s^2]^2, normally set small value, such as 1E-6
       */
      {
        mat_t Q(ins_gps->getFilter().getQ());
        
        Q(0, 0) = Q(1, 1) = Q(2, 2) = 25E-4;
        Q(3, 3) = Q(4, 4) = Q(5, 5) = 25E-6;
        Q(6, 6) = 1E-6; //1E-14
        
        ins_gps->getFilter().setQ(Q);
      }
    }

    void setup_filter(
        const vec3_t &accel_sigma,
        const vec3_t &gyro_sigma,
        void *) {}

    template <class BaseINS, template <class> class Filter>
    void setup_filter(
        const vec3_t &accel_sigma,
        const vec3_t &gyro_sigma,
        Filtered_INS2<BaseINS, Filter> *) {

      mat_t Q(ins_gps->getFilter().getQ());
      for(int i(0), j(3); i < 3; i++, j++){
        Q(i, i) = pow(accel_sigma[i], 2);
        Q(j, j) = pow(gyro_sigma[i], 2);
      }
      ins_gps->getFilter().setQ(Q);
    }

    template <class BaseFINS>
    void setup_filter(Filtered_INS_BiasEstimated<BaseFINS> *) {

      setup_filter((BaseFINS *)ins_gps);

      {
        mat_t P(ins_gps->getFilter().getP());
        static const unsigned NP(
            Filtered_INS_BiasEstimated<BaseFINS>::P_SIZE_WITHOUT_BIAS);
        P(NP,     NP)     = P(NP + 1, NP + 1) = P(NP + 2, NP + 2) = 1E-4; // for accelerometer bias drift
        P(NP + 3, NP + 3) = P(NP + 4, NP + 4) = P(NP + 5, NP + 5) = 1E-7; // for gyro bias drift
        ins_gps->getFilter().setP(P);
      }

      {
        mat_t Q(ins_gps->getFilter().getQ());
        static const unsigned NQ(
            Filtered_INS_BiasEstimated<BaseFINS>::Q_SIZE_WITHOUT_BIAS);
        Q(NQ,     NQ)     = Q(NQ + 1, NQ + 1) = Q(NQ + 2, NQ + 2) = 1E-6; // for accelerometer bias drift
        Q(NQ + 3, NQ + 3) = Q(NQ + 4, NQ + 4) = Q(NQ + 5, NQ + 5) = 1E-8; // for gyro bias drift
        ins_gps->getFilter().setQ(Q);
      }

      ins_gps->beta_accel() *= 0.1;
      ins_gps->beta_gyro() *= 0.1; //mems_g.BETA;
    }

    template <class BaseFINS>
    void setup_filter(Filtered_INS_ClockErrorEstimated<BaseFINS> *) {

      setup_filter((BaseFINS *)ins_gps);

      {
        mat_t P(ins_gps->getFilter().getP());
        static const unsigned NP(Filtered_INS_ClockErrorEstimated<BaseFINS>::P_SIZE_WITHOUT_CLOCK_ERROR);
        for(int i(0); i < Filtered_INS_ClockErrorEstimated<BaseFINS>::P_SIZE_CLOCK_ERROR; i += 2){
          P(NP + i,     NP + i)     = 1E4; // TODO
          P(NP + i + 1, NP + i + 1) = 1E2;
        }
        ins_gps->getFilter().setP(P);
      }

      {
        mat_t Q(ins_gps->getFilter().getQ());
        static const unsigned NQ(Filtered_INS_ClockErrorEstimated<BaseFINS>::Q_SIZE_WITHOUT_CLOCK_ERROR);
        for(int i(0); i < Filtered_INS_ClockErrorEstimated<BaseFINS>::P_SIZE_CLOCK_ERROR; i += 2){
          Q(NQ + i,     NQ + i)     = 1E3; // TODO
          Q(NQ + i + 1, NQ + i + 1) = 1E1;
        }
        ins_gps->getFilter().setQ(Q);
      }

      ins_gps->beta_clock_error() *= 1; // TODO
    }

    template <class Base_INS_GPS>
    void setup_filter(INS_GPS_Back_Propagate<Base_INS_GPS> *){
      setup_filter((Base_INS_GPS *)ins_gps);
      ins_gps->setup_back_propagation(options.back_propagate_property);
    }

    template <class Base_INS_GPS>
    void setup_filter(INS_GPS_RealTime<Base_INS_GPS> *){
      setup_filter((Base_INS_GPS *)ins_gps);
      ins_gps->setup_realtime(options.realttime_property);
    }

    template <class Base_INS_GPS>
    void setup_filter(INS_GPS_Debug<Base_INS_GPS> *){
      setup_filter((Base_INS_GPS *)ins_gps);
      ins_gps->setup_debug(options.debug_property);
    }

  public:
    INS_GPS_NAV()
        : NAV(),
        ins_gps(new INS_GPS()), helper(*this) {
      setup_filter(ins_gps);
      //std::cerr << "P:" << ins_gps->getFilter().getP() << std::endl;
      //std::cerr << "Q:" << ins_gps->getFilter().getQ() << std::endl;
    }
    virtual ~INS_GPS_NAV() {
      delete ins_gps;
    }

    void setup_filter(
        const vec3_t &accel_sigma,
        const vec3_t &gyro_sigma){
      setup_filter(accel_sigma, gyro_sigma, ins_gps);
    }

    void inspect(std::ostream &out, void *) const {}
    template <class INS_GPS_base>
    void inspect(std::ostream &out, INS_GPS_Debug<INS_GPS_base> *) const {
      ins_gps->inspect(out);
    }
    void inspect(std::ostream &out) const {
      inspect(out, ins_gps);
    }

  protected:
    static void set_matrix_full(mat_t &mat, const char *spec){
      char *_spec(const_cast<char *>(spec));
      for(int i(0); i < mat.rows(); i++){
        for(int j(0); j < mat.columns(); j++){
          mat(i, j) = std::strtod(_spec, &_spec);
        }
      }
    }
    static void set_matrix_diagonal(mat_t &mat, const char *spec){
      char *_spec(const_cast<char *>(spec));
      for(int i(0); i < mat.rows(); i++){
        mat(i, i) = std::strtod(_spec, &_spec);
      }
    }
    static void set_matrix_1element(mat_t &mat, const char *spec){
      char *_spec(const_cast<char *>(spec));
      int i((int)std::strtol(_spec, &_spec, 10));
      int j((int)std::strtol(_spec, &_spec, 10));
      mat(i, j) = std::strtod(_spec, &_spec);
    }

    bool init_misc(const char *line, void *){
      return false;
    }
    bool init_misc(const char *line, INS<float_t> *){
      const char *value;
      if(value = Options::get_value2(line, "x")){
        char *spec(const_cast<char *>(value));
        int i((int)std::strtol(spec, &spec, 10));
        (*ins_gps)[i] = std::strtod(spec, &spec);
        return true;
      }
      return false;
    }
    template <class BaseINS, template <class> class Filter>
    bool init_misc(const char *line, Filtered_INS2<BaseINS, Filter> *){
      const char *value;

      while(true){
        mat_t P(ins_gps->getFilter().getP());
        if(value = Options::get_value2(line, "P")){
          set_matrix_full(P, value);
        }else if(value = Options::get_value2(line, "P_diag")){
          set_matrix_diagonal(P, value);
        }else if(value = Options::get_value2(line, "P_elm")){
          set_matrix_1element(P, value);
        }else{break;}
        ins_gps->getFilter().setP(P);
        return true;
      }

      while(true){
        mat_t Q(ins_gps->getFilter().getQ());
        if(value = Options::get_value2(line, "Q")){
          set_matrix_full(Q, value);
        }else if(value = Options::get_value2(line, "Q_diag")){
          set_matrix_diagonal(Q, value);
        }else if(value = Options::get_value2(line, "Q_elm")){
          set_matrix_1element(Q, value);
        }else{break;}
        ins_gps->getFilter().setQ(Q);
        return true;
      }

      return init_misc(line, (BaseINS *)ins_gps);
    }

  public:
    bool init_misc(const char *line){
      if(std::strlen(line) == 0){return true;}

      bool res(init_misc(line, ins_gps));
      if(res){
        std::cerr << "Init (misc): " << line << std::endl;
      }
      return res;
    }
    
    float_t &operator[](const unsigned &index){return ins_gps->operator[](index);}
    
    NAV &update(
        const vec3_t &accel,
        const vec3_t &gyro,
        const float_t &elapsedT){
      ins_gps->update(accel, gyro, elapsedT);
      return *this;
    }
  
  public:
    template <class GPS_Packet>
    NAV &correct(const GPS_Packet &gps){
      ins_gps->correct(gps);
      return *this;
    }

    template <class GPS_Packet>
    NAV &correct(
        const GPS_Packet &gps,
        const vec3_t &lever_arm_b,
        const vec3_t &omega_b2i_4b){
      ins_gps->correct(gps, lever_arm_b, omega_b2i_4b);
      return *this;
    }

  protected:
    bool setup_correct(const float_t &advanceT, void *){
      return true;
    }

    template <class INS_GPS_orig>
    bool setup_correct(const float_t &advanceT, INS_GPS_RealTime<INS_GPS_orig> *){
      return ins_gps->setup_correct(advanceT);
    }

  public:
    template <class GPS_Packet>
    NAV &correct(
        const GPS_Packet &gps,
        const float_t &advanceT){
      if(setup_correct(advanceT, ins_gps)){
        return correct(gps);
      }else{
        return *this;
      }
    }

    template <class GPS_Packet>
    NAV &correct(
        const GPS_Packet &gps,
        const vec3_t &lever_arm_b,
        const vec3_t &omega_b2i_4b,
        const float_t &advanceT){
      if(setup_correct(advanceT, ins_gps)){
        return correct(gps, lever_arm_b, omega_b2i_4b);
      }else{
        return *this;
      }
    }

    NAV &correct_yaw(const float_t &delta_yaw){
      ins_gps->correct_yaw(delta_yaw, pow(deg2rad(options.mag_heading_accuracy_deg), 2));
      return *this;
    }
    
    void label(std::ostream &out) const {
      ins_gps->label(out);
    }

    updated_items_t updated_items() const {
      return helper.updated_items();
    }

    void update(const A_Packet &packet){
      helper.before_any_update();
      helper.time_update(packet);
    }
    void update(const G_Packet &packet){
      helper.before_any_update();
      helper.measurement_update(packet);
    }
    void update(const G_Packet_Measurement &packet){
      helper.before_any_update();
      helper.measurement_update(packet);
    }
    void update(const M_Packet &packet){
      helper.before_any_update();
      helper.compass(packet);
    }
    void update(const TimePacket &packet){
      helper.t_stamp_generator.update(packet);
    }
};

template <class INS_GPS>
struct INS_GPS_NAV_Factory : public NAV_Factory<INS_GPS> {

  template <class Calibration>
  static NAV *generate(const Calibration &calibration){
    typedef INS_GPS_NAV_Factory<INS_GPS_NAV<INS_GPS> > nav_t;
    typename nav_t::disp_t *res(new typename nav_t::disp_t());
    res->setup_filter(calibration.sigma_accel(), calibration.sigma_gyro());
    return res;
  }

  private:
  template <class T>
  struct Checker {
    template <class Calibration>
    static NAV *check_covariance(const Calibration &calibration){
      switch(options.debug_property.debug_target){
        case INS_GPS_Debug_Property::DEBUG_KF_P:
        case INS_GPS_Debug_Property::DEBUG_KF_FULL:
          return INS_GPS_NAV_Factory<INS_GPS_Debug_Covariance<T> >::generate(calibration);
        case INS_GPS_Debug_Property::DEBUG_NONE:
        default:
          return INS_GPS_NAV_Factory<T>::generate(calibration);
      }
    }

    template <class Calibration>
    static NAV *check_synchronization(const Calibration &calibration){
      switch(options.ins_gps_sync_strategy){
        case Options::INS_GPS_SYNC_BACK_PROPAGATION:
          return Checker<INS_GPS_Back_Propagate<T> >::check_covariance(calibration);
        case Options::INS_GPS_SYNC_REALTIME:
          return Checker<INS_GPS_RealTime<T> >::check_covariance(calibration);
        case Options::INS_GPS_SYNC_OFFLINE:
        default:
          return check_covariance(calibration);
      }
    }

    template <class Calibration>
    static NAV *check_navdata(const Calibration &calibration){
      return Checker<INS_GPS_NAVData<T> >::check_synchronization(calibration);
    }

    template <class Calibration>
    static NAV *check_pure_ins(const Calibration &calibration){
      return (options.debug_property.debug_target == INS_GPS_Debug_Property::DEBUG_PURE_INERTIAL)
          ? Checker<INS_GPS_Debug_PureInertial<T> >::check_navdata(calibration)
          : check_navdata(calibration);
    }
  };

  template <class T>
  struct Checker<INS_GPS_Debug_PureInertial<T> > {
    template <class Calibration>
    static NAV *check_navdata(const Calibration &calibration){
      return INS_GPS_NAV_Factory<
            INS_GPS_NAVData<INS_GPS_Debug_PureInertial<T> >
          >::generate(calibration);
    }
  };

  public:
  template <class Calibration>
  static NAV *get_nav(const Calibration &calibration){
    return Checker<INS_GPS>::check_pure_ins(calibration);
  }
};

template <class PureINS, class TimeStamp = typename PureINS::float_t>
class INS_NAVData : public PureINS, public NAVData<typename PureINS::float_t> {
  public:
    typedef NAVData<typename PureINS::float_t> super_data_t;
    typedef TimeStamp time_stamp_t;
  protected:
    mutable const char *mode;
    mutable time_stamp_t itow;
  public:
    INS_NAVData() : PureINS(), mode("N/A"), itow(0) {}
    INS_NAVData(const INS_NAVData<PureINS, time_stamp_t> &orig, const bool &deepcopy = false)
        : PureINS(orig, deepcopy), mode(orig.mode), itow(orig.itow) {}
    ~INS_NAVData(){}
#define MAKE_PROXY_FUNC(fname) \
typename PureINS::float_t fname() const {return PureINS::fname();}
    MAKE_PROXY_FUNC(longitude);
    MAKE_PROXY_FUNC(latitude);
    MAKE_PROXY_FUNC(height);
    MAKE_PROXY_FUNC(v_north);
    MAKE_PROXY_FUNC(v_east);
    MAKE_PROXY_FUNC(v_down);
    MAKE_PROXY_FUNC(heading);
    MAKE_PROXY_FUNC(euler_phi);
    MAKE_PROXY_FUNC(euler_theta);
    MAKE_PROXY_FUNC(euler_psi);
    MAKE_PROXY_FUNC(azimuth);
#undef MAKE_PROXY_FUNC
    typename PureINS::float_t time_stamp() const {return (typename PureINS::float_t)itow;}

    void set_header(const char *_mode) const {
      mode = _mode;
    }
    void set_header(const char *_mode, const time_stamp_t &_itow) const {
      mode = _mode;
      itow = _itow;
    }
  protected:
    static void label_time(std::ostream &out, const void *){
      out << "itow";
    }

    template <class FloatT>
    static void label_time(std::ostream &out, const CalendarTimeStamp<FloatT> *){
      CalendarTimeStamp<FloatT>::label(out);
    }
  public:
    void label(std::ostream &out) const {
      out << "mode" << ',';
      label_time(out, &itow);
      out << ',';
      super_data_t::label(out);
    }
    void dump(std::ostream &out) const {
      out << mode << ','
          << itow << ',';
      super_data_t::dump(out);
    }
};

template <class INS_GPS>
class INS_GPS_NAVData : public INS_GPS {
  public:
#if defined(__GNUC__) && (__GNUC__ < 5)
    typedef typename INS_GPS::vec3_t vec3_t;
    typedef typename INS_GPS::mat_t mat_t;
#else
    using typename INS_GPS::vec3_t;
    using typename INS_GPS::mat_t;
#endif
  public:
    INS_GPS_NAVData() : INS_GPS() {}
    INS_GPS_NAVData(const INS_GPS_NAVData<INS_GPS> &orig, const bool &deepcopy = false)
        : INS_GPS(orig, deepcopy) {}
    ~INS_GPS_NAVData(){}
  protected:
    static void label2(std::ostream &out, const void *){}

    template <class BaseINS, template <class> class Filter>
    static void label2(std::ostream &out, const Filtered_INS2<BaseINS, Filter> *fins){
      label2(out, (const BaseINS *)fins);
      if(options.dump_stddev){
        out << ',' << "s1(longitude)"
            << ',' << "s1(latitude)"
            << ',' << "s1(height)"
            << ',' << "s1(v_north)"
            << ',' << "s1(v_east)"
            << ',' << "s1(v_down)"
            << ',' << "s1(psi)"
            << ',' << "s1(theta)"
            << ',' << "s1(phi)";
      }
    }

    template <class BaseINS>
    static void label2(std::ostream &out, const INS_BiasEstimated<BaseINS> *ins){
      label2(out, (const BaseINS *)ins);
      out << ',' << "bias_accel(X)"   //Bias
          << ',' << "bias_accel(Y)"
          << ',' << "bias_accel(Z)"
          << ',' << "bias_gyro(X)"
          << ',' << "bias_gyro(Y)"
          << ',' << "bias_gyro(Z)" ;
    }

    template <class BaseFINS>
    static void label2(std::ostream &out, const Filtered_INS_BiasEstimated<BaseFINS> *fins){
      label2(out, (const BaseFINS *)fins);
      if(options.dump_stddev){
        out << ',' << "s1(bias_accel(X))"
            << ',' << "s1(bias_accel(Y))"
            << ',' << "s1(bias_accel(Z))"
            << ',' << "s1(bias_gyro(X))"
            << ',' << "s1(bias_gyro(Y))"
            << ',' << "s1(bias_gyro(Z))";
      }
    }

    template <class BaseINS, unsigned int Clocks>
    static void label2(
        std::ostream &out, const INS_ClockErrorEstimated<BaseINS, Clocks> *ins){
      label2(out, (const BaseINS *)ins);
      for(unsigned i(0); i < INS_ClockErrorEstimated<BaseINS, Clocks>::CLOCKS_SUPPORTED; ++i){
        out << ',' << "receiver_clock_error[" << i << "]"
            << ',' << "receiver_clock_error_rate[" << i << "]";
      }
    }

    template <class BaseFINS>
    static void label2(
        std::ostream &out, const Filtered_INS_ClockErrorEstimated<BaseFINS> *fins){
      label2(out, (const BaseFINS *)fins);
      if(options.dump_stddev){
        for(unsigned i(0); i < Filtered_INS_ClockErrorEstimated<BaseFINS>::CLOCKS_SUPPORTED; ++i){
          out << ',' << "s1(receiver_clock_error[" << i << "])"
              << ',' << "s1(receiver_clock_error_rate[" << i << "])";
        }
      }
    }

  public:
    /**
     * print label
     */
    void label(std::ostream &out) const {
      INS_GPS::label(out);
      label2(out, this);
    }
  
  protected:
    void dump2(std::ostream &out, const void *) const {}

    template <class BaseINS, template <class> class Filter>
    void dump2(std::ostream &out, const Filtered_INS2<BaseINS, Filter> *fins) const {
      dump2(out, (const BaseINS *)fins);
      if(options.dump_stddev){
        typename Filtered_INS2<BaseINS, Filter>::StandardDeviations sigma(fins->getSigma());
        out << ',' << rad2deg(sigma.longitude_rad)
            << ',' << rad2deg(sigma.latitude_rad)
            << ',' << sigma.height_m
            << ',' << sigma.v_north_ms
            << ',' << sigma.v_east_ms
            << ',' << sigma.v_down_ms
            << ',' << rad2deg(sigma.heading_rad)
            << ',' << rad2deg(sigma.pitch_rad)
            << ',' << rad2deg(sigma.roll_rad);
      }
    }

    template <class BaseINS>
    void dump2(std::ostream &out, const INS_BiasEstimated<BaseINS> *ins) const {
      dump2(out, (const BaseINS *)ins);
      vec3_t &ba(const_cast<INS_BiasEstimated<BaseINS> *>(ins)->bias_accel());
      vec3_t &bg(const_cast<INS_BiasEstimated<BaseINS> *>(ins)->bias_gyro());
      out << ',' << ba.getX()      // Bias
          << ',' << ba.getY()
          << ',' << ba.getZ()
          << ',' << bg.getX()
          << ',' << bg.getY()
          << ',' << bg.getZ();
    }

    template <class BaseFINS>
    void dump2(
        std::ostream &out, const Filtered_INS_BiasEstimated<BaseFINS> *fins) const {
      dump2(out, (const BaseFINS *)fins);
      if(options.dump_stddev){
        const mat_t &P(
            const_cast<Filtered_INS_BiasEstimated<BaseFINS> *>(fins)->getFilter().getP());
        for(int i(Filtered_INS_BiasEstimated<BaseFINS>::P_SIZE_WITHOUT_BIAS), j(0);
            j < Filtered_INS_BiasEstimated<BaseFINS>::P_SIZE_BIAS; ++i, ++j){
          out << ',' << sqrt(P(i, i));
        }
      }
    }

    template <class BaseINS, unsigned int Clocks>
    void dump2(
        std::ostream &out, const INS_ClockErrorEstimated<BaseINS, Clocks> *ins) const {
      dump2(out, (const BaseINS *)ins);
      for(unsigned i(0); i < INS_ClockErrorEstimated<BaseINS, Clocks>::CLOCKS_SUPPORTED; ++i){
        out << ',' << const_cast<INS_ClockErrorEstimated<BaseINS, Clocks> *>(ins)->clock_error(i)
            << ',' << const_cast<INS_ClockErrorEstimated<BaseINS, Clocks> *>(ins)->clock_error_rate(i);
      }
    }

    template <class BaseFINS>
    void dump2(
        std::ostream &out, const Filtered_INS_ClockErrorEstimated<BaseFINS> *fins) const {
      dump2(out, (const BaseFINS *)fins);
      if(options.dump_stddev){
        const mat_t &P(
            const_cast<Filtered_INS_ClockErrorEstimated<BaseFINS> *>(fins)->getFilter().getP());
        for(int i(Filtered_INS_ClockErrorEstimated<BaseFINS>::P_SIZE_WITHOUT_CLOCK_ERROR), j(0);
            j < Filtered_INS_ClockErrorEstimated<BaseFINS>::P_SIZE_CLOCK_ERROR; ++i, ++j){
          out << ',' << sqrt(P(i, i));
        }
      }
    }

  public:
    /**
     * print current state
     * 
     * @param itow current time
     */
    void dump(std::ostream &out) const {
      INS_GPS::dump(out);
      dump2(out, this);
    }
};

struct StandardCalibration {

  int index_base, index_temp_ch;
  template <std::size_t N>
  struct calibration_info_t {
    float_sylph_t bias_tc[N];
    float_sylph_t bias_base[N];
    float_sylph_t sf[N];
    float_sylph_t alignment[N][N];
    float_sylph_t sigma[N];
    static void set(char *spec, float_sylph_t target[N]){
      for(int i(0); i < N; i++){
        target[i] = std::strtod(spec, &spec);
      }
    }
    static void set(char *spec, float_sylph_t target[N][N]){
      for(int i(0); i < N; i++){
        for(int j(0); j < N; j++){
          target[i][j] = std::strtod(spec, &spec);
        }
      }
    }
    static std::ostream &dump(std::ostream &out, const float_sylph_t target[N]){
      for(int i(0); i < N; i++){
        out << " " << target[i];
      }
      return out;
    }
    static std::ostream &dump(std::ostream &out, const float_sylph_t target[N][N]){
      for(int i(0); i < N; i++){
        for(int j(0); j < N; j++){
          out << " " << target[i][j];
        }
      }
      return out;
    }
  };
  typedef calibration_info_t<3> dof3_t;
  dof3_t accel, gyro;

  bool check_spec(const char *line){
    const char *value;
    if(value = Options::get_value2(line, "index_base")){
      index_base = std::atoi(value);
      return true;
    }
    if(value = Options::get_value2(line, "index_temp_ch")){
      index_temp_ch = std::atoi(value);
      return true;
    }
#define TO_STRING(name) # name
#define check_proc(name, sensor, item) \
if(value = Options::get_value2(line, TO_STRING(name))){ \
  dof3_t::set(const_cast<char *>(value), sensor.item); \
  return true; \
}
    check_proc(acc_bias_tc, accel, bias_tc);
    check_proc(acc_bias, accel, bias_base);
    check_proc(acc_sf, accel, sf);
    check_proc(acc_mis, accel, alignment);
    check_proc(gyro_bias_tc, gyro, bias_tc);
    check_proc(gyro_bias, gyro, bias_base);
    check_proc(gyro_sf, gyro, sf);
    check_proc(gyro_mis, gyro, alignment);
    check_proc(sigma_accel, accel, sigma);
    check_proc(sigma_gyro, gyro, sigma);
#undef check_proc
#undef TO_STRING

    return false;
  }

  friend std::ostream &operator<<(
      std::ostream &out, const StandardCalibration &calib){
    out << "index_base " << calib.index_base << std::endl;
    out << "index_temp_ch " << calib.index_temp_ch << std::endl;
#define TO_STRING(name) # name
#define dump_proc(name, sensor, item) \
  out << TO_STRING(name); \
  dof3_t::dump(out, calib.sensor.item)
    dump_proc(acc_bias_tc, accel, bias_tc) << std::endl;
    dump_proc(acc_bias, accel, bias_base) << std::endl;
    dump_proc(acc_sf, accel, sf) << std::endl;
    dump_proc(acc_mis, accel, alignment) << std::endl;
    dump_proc(gyro_bias_tc, gyro, bias_tc) << std::endl;
    dump_proc(gyro_bias, gyro, bias_base) << std::endl;
    dump_proc(gyro_sf, gyro, sf) << std::endl;
    dump_proc(gyro_mis, gyro, alignment) << std::endl;
    dump_proc(sigma_accel, accel, sigma) << std::endl;
    dump_proc(sigma_gyro, gyro, sigma);
#undef dump_proc
#undef TO_STRING
    return out;
  }

  template <class NumType, std::size_t N>
  static void calibrate(
      const NumType raw[],
      const NumType &bias_mod,
      const calibration_info_t<N> &info,
      float_sylph_t (&res)[N]) {

    // Temperature compensation
    float_sylph_t bias[N];
    for(int i(0); i < N; i++){
      bias[i] = info.bias_base[i] + (info.bias_tc[i] * bias_mod);
    }

    // Convert raw values to physical quantity by using scale factor
    float_sylph_t tmp[N];
    for(int i(0); i < N; i++){
      tmp[i] = (((float_sylph_t)raw[i] - bias[i]) / info.sf[i]);
    }

    // Misalignment compensation
    for(int i(0); i < N; i++){
      res[i] = 0;
      for(int j(0); j < N; j++){
        res[i] += info.alignment[i][j] * tmp[j];
      }
    }
  }

  StandardCalibration() {}
  ~StandardCalibration() {}

  /**
   * Get acceleration in m/s^2
   */
  Vector3<float_sylph_t> raw2accel(const int *raw_data) const{
    float_sylph_t res[3];
    calibrate(
        &raw_data[index_base], raw_data[index_temp_ch],
        accel, res);
    return Vector3<float_sylph_t>(res[0], res[1], res[2]);
  }

  /**
   * Get angular speed in rad/sec
   */
  Vector3<float_sylph_t> raw2omega(const int *raw_data) const{
    float_sylph_t res[3];
    calibrate(
        &raw_data[index_base + 3], raw_data[index_temp_ch],
        gyro, res);
    return Vector3<float_sylph_t>(res[0], res[1], res[2]);
  }

  /**
   * Accelerometer output variance in [m/s^2]^2
   */
  Vector3<float_sylph_t> sigma_accel() const{
    return Vector3<float_sylph_t>(accel.sigma[0], accel.sigma[1], accel.sigma[2]);
  }

  /**
   * Angular speed output variance in X, Y, Z axes, [rad/s]^2
   */
  Vector3<float_sylph_t> sigma_gyro() const{
    return Vector3<float_sylph_t>(gyro.sigma[0], gyro.sigma[1], gyro.sigma[2]);
  }
};

using namespace std;

struct G_Packet_GPS_Ephemeris_Extended : public G_Packet_GPS_Ephemeris {
  space_node_t::uint_t &sv_number; // sv_number is alias.
  bool valid;

  G_Packet_GPS_Ephemeris_Extended()
      : G_Packet_GPS_Ephemeris(),
      sv_number(ephemeris_t::svid), valid(false) {}
};

template <> template <>
void G_Packet_Observer<float_sylph_t>::subframe_t::fetch_as_subframe1(
    G_Packet_GPS_Ephemeris_Extended &ephemeris) const {
  GNSS_Data<float_sylph_t>::Loader::fetch_as_GPS_subframe1(*this, ephemeris);
}

template <> template <>
void G_Packet_Observer<float_sylph_t>::subframe_t::fetch_as_subframe2(
    G_Packet_GPS_Ephemeris_Extended &ephemeris) const {
  GNSS_Data<float_sylph_t>::Loader::fetch_as_GPS_subframe2(*this, ephemeris);
}

template <> template <>
void G_Packet_Observer<float_sylph_t>::subframe_t::fetch_as_subframe3(
    G_Packet_GPS_Ephemeris_Extended &ephemeris) const {
  GNSS_Data<float_sylph_t>::Loader::fetch_as_GPS_subframe3(*this, ephemeris);
}

class StreamProcessor
    : public AbstractSylphideProcessor<float_sylph_t> {

  public:
    static const unsigned int buffer_size;

  protected:
    Updatable *updatable;

    typedef AbstractSylphideProcessor<float_sylph_t> super_t;
    typedef A_Packet_Observer<float_sylph_t> A_Observer_t;
    typedef G_Packet_Observer<float_sylph_t> G_Observer_t;
    typedef M_Packet_Observer<float_sylph_t> M_Observer_t;

    struct Handler {
      StreamProcessor &outer;
      Handler(StreamProcessor &invoker) : outer(invoker) {}
      Handler &operator=(const Handler &another){
        return *this;
      }
    };

    /**
     * A page (ADC value)
     */
    struct AHandler : public A_Observer_t, public Handler {
      bool previous_seek_next;
      A_Packet packet_latest;
      StandardCalibration calibration;

      AHandler(StreamProcessor &invoker) : A_Observer_t(buffer_size),
          Handler(invoker),
          packet_latest(),
          calibration() {

        previous_seek_next = A_Observer_t::ready();

        { // NinjaScan default calibration parameters
#define config(spec) calibration.check_spec(spec);
          config("index_base 0");
          config("index_temp_ch 8");
          config("acc_bias 32768 32768 32768");
          config("acc_bias_tc 0 0 0"); // No temperature compensation
          config("acc_sf 4.1767576e+2 4.1767576e+2 4.1767576e+2"); // MPU-6000/9250 8[G] full scale; (1<<15)/(8*9.80665) [1/(m/s^2)]
          config("acc_mis 1 0 0 0 1 0 0 0 1"); // No misalignment compensation
          config("gyro_bias 32768 32768 32768");
          config("gyro_bias_tc 0 0 0"); // No temperature compensation
          config("gyro_sf 9.3873405e+2 9.3873405e+2 9.3873405e+2"); // MPU-6000/9250 2000[dps] full scale; (1<<15)/(2000/180*PI) [1/(rad/s)]
          config("gyro_mis 1 0 0 0 1 0 0 0 1"); // No misalignment compensation
          config("sigma_accel 0.05 0.05 0.05"); // approx. 150[mG] ? standard deviation
          config("sigma_gyro 5e-3 5e-3 5e-3"); // approx. 0.3[dps] standard deviation
#undef config
        }
      }
      ~AHandler(){}
      void operator()(const A_Observer_t &observer){
        if(!observer.validate()){return;}

        float_sylph_t itow(observer.fetch_ITOW());
        if(options.reduce_1pps_sync_error){
          float_sylph_t delta_t(itow - packet_latest.itow);
          if((delta_t >= 1) && (delta_t < 2)){itow -= 1;}
        }
        packet_latest.itow = itow;

        int ch[9];
        A_Observer_t::values_t values(observer.fetch_values());
        for(int i = 0; i < 8; i++){
          ch[i] = values.values[i];
        }
        ch[8] = values.temperature;
        packet_latest.accel = calibration.raw2accel(ch);
        packet_latest.omega = calibration.raw2omega(ch);

        Handler::outer.updatable->update(packet_latest);
      }
    } a_handler;

    /**
     * G page (u-blox)
     */
    struct GHandler : public G_Observer_t, public Handler {
      bool previous_seek_next;
      Vector3<float_sylph_t> lever_arm;
      G_Packet packet_latest;
      int itow_ms_0x0102, itow_ms_0x0112;
      int week_number;
      struct status_t {
        unsigned int gps;
        enum {
          TIME_STAMP_INVALID,
          TIME_STAMP_BEFORE_START,
          TIME_STAMP_IN_RANGE,
          TIME_STAMP_AFTER_END,
        } time_stamp;
        status_t()
            : gps(G_Observer_t::status_t::NO_FIX),
            time_stamp(TIME_STAMP_INVALID) {}
      } status;

      // for tightly coupled
      typedef GPS_SpaceNode<float_sylph_t> gps_space_node_t;
      gps_space_node_t gps_space_node;
      G_Packet_Measurement packet_raw_latest;
      typedef G_Packet_Measurement::raw_data_t raw_data_t;
      raw_data_t::solver_t::options_t solver_options;
      GNSS_Data<float_sylph_t>::Loader loader;
      struct tightly_associator_t {
        GHandler &handler;
        raw_data_t::solver_t solver;
        void associate() {
          handler.packet_raw_latest.raw_data.solver = &solver;
          handler.loader.gps = &(handler.gps_space_node);
        }
        tightly_associator_t(GHandler &h)
            : handler(h),
            solver(handler.gps_space_node, handler.solver_options){
          associate();
        }
        tightly_associator_t &operator=(const tightly_associator_t &another){
          if(this != &another){associate();}
          return *this;
        }
      } tightly_associator;

      GHandler(StreamProcessor &invoker)
          : G_Observer_t(buffer_size),
          Handler(invoker),
          lever_arm(),
          packet_latest(),
          itow_ms_0x0102(-1), itow_ms_0x0112(-1),
          week_number(Options::gps_time_t::WN_INVALID), status(),
          gps_space_node(),
          packet_raw_latest(), solver_options(),
          loader(),
          tightly_associator(*this) {
        previous_seek_next = G_Observer_t::ready();
      }
      ~GHandler(){}

      template <class GHandler_Packet>
      void update(const GHandler_Packet &packet){
        switch(status.time_stamp){
          case status_t::TIME_STAMP_INVALID:
          case status_t::TIME_STAMP_AFTER_END:
            return;
          case status_t::TIME_STAMP_BEFORE_START:
            if(!options.is_time_after_start(packet.itow, week_number)){
              return;
            }
            status.time_stamp = status_t::TIME_STAMP_IN_RANGE;
          case status_t::TIME_STAMP_IN_RANGE:
            if(!options.is_time_before_end(packet.itow, week_number)){
              status.time_stamp = status_t::TIME_STAMP_AFTER_END;
              return;
            }
            break;
        }
        Handler::outer.updatable->update(packet);
      }

      /**
       * Extract the following data.
       * {class, id} = {0x01, 0x02} : position
       * {class, id} = {0x01, 0x03} : status
       * {class, id} = {0x01, 0x06} : solution
       * {class, id} = {0x01, 0x12} : velocity
       */
      void check_nav(const G_Observer_t &observer, const G_Observer_t::packet_type_t &packet_type){
        switch(packet_type.mid){
          case 0x02: { // NAV-POSLLH
            G_Observer_t::position_t
              position(observer.fetch_position());
            G_Observer_t::position_acc_t
              position_acc(observer.fetch_position_acc());

            //cerr << "G_Arrive 0x02 : " << observer.fetch_ITOW() << endl;

            itow_ms_0x0102 = observer.fetch_ITOW_ms();

            packet_latest.solution.latitude = deg2rad(position.latitude);
            packet_latest.solution.longitude = deg2rad(position.longitude);
            packet_latest.solution.height = position.altitude;
            packet_latest.solution.sigma_2d = position_acc.horizontal;
            packet_latest.solution.sigma_height = position_acc.vertical;

            break;
          }
          case 0x03: { // NAV-STATUS
            G_Observer_t::status_t st(observer.fetch_status());
            status.gps = st.fix_type;
            return;
          }
          case 0x06: { // NAV-SOL
            G_Observer_t::solution_t solution(observer.fetch_solution());
            if(solution.status_flags & G_Observer_t::solution_t::WN_VALID){
              week_number = solution.week;
              if(status.time_stamp == status_t::TIME_STAMP_INVALID){
                status.time_stamp = status_t::TIME_STAMP_BEFORE_START;
              }
            }
            return;
          }
          case 0x12: { // NAV-VELNED
            G_Observer_t::velocity_t
                velocity(observer.fetch_velocity());
            G_Observer_t::velocity_acc_t
                velocity_acc(observer.fetch_velocity_acc());

            //cerr << "G_Arrive 0x12 : " << current_itow << " =? " << packet.itow << endl;

            itow_ms_0x0112 = observer.fetch_ITOW_ms();

            packet_latest.solution.v_n = velocity.north;
            packet_latest.solution.v_e = velocity.east;
            packet_latest.solution.v_d = velocity.down;
            packet_latest.solution.sigma_vel = velocity_acc.acc;

            break;
          }
          case 0x20: { // NAV-TIMEGPS
            TimePacket packet;
            packet.itow = observer.fetch_ITOW();
            char buf[4];
            observer.inspect(buf, sizeof(buf), 6 + 8);
            if(packet.valid_week_num = ((unsigned char)buf[3] & 0x02)){
              // valid week number
              packet.week_num = le_char2_2_num<unsigned short>(*buf);
              if(packet.valid_leap_sec = ((unsigned char)buf[3] & 0x04)){
                // valid UTC (leap seconds)
                packet.leap_sec = (char)(buf[2]);
              }
            }
            Handler::outer.updatable->update(packet);
            break;
          }
          default: return;
        }

        // this flow is only available when 0x0102 or 0x0112
        if(itow_ms_0x0102 == itow_ms_0x0112){
          packet_latest.itow = (float_sylph_t)1E-3 * itow_ms_0x0102;
          if(options.gps_fake_lock){
            packet_latest.solution.latitude = packet_latest.solution.longitude = packet_latest.solution.height = 0;
            packet_latest.solution.sigma_2d = packet_latest.solution.sigma_height = 1E+1;
            packet_latest.solution.v_n = packet_latest.solution.v_e = packet_latest.solution.v_d = 0;
            packet_latest.solution.sigma_vel = 1;
          }
          update(packet_latest);
        }
      }

      void check_subframe(G_Packet_Data &packet){
        packet.loader = &loader;
        packet.time_of_reception.week
            = (status.time_stamp == status_t::TIME_STAMP_INVALID) ? -1 : week_number; // -1 means invalid
        packet.time_of_reception.seconds = packet_latest.itow;
        Handler::outer.updatable->update(packet);
      }

      void check_subframeX(
          const unsigned int &gnssID,
          const unsigned int &svID,
          G_Packet_Data &packet){

        switch(gnssID){
          case G_Observer_t::gnss_svid_t::GPS: {
            packet.subframe.sv_number = svID;
            // change format to UBX-RXM-SFRB by removing parity
            for(int i(0); i < sizeof(packet.subframe.buffer); i += sizeof(G_Observer_t::u32_t)){
              *(G_Observer_t::u32_t *)(&packet.subframe.buffer[i]) >>= 6;
            }
            packet.subframe.update_properties();
            check_subframe(packet);
            return;
          }
          case G_Observer_t::gnss_svid_t::SBAS: {
            packet.subframe.sv_number = svID; // svID (120-158)
            // change format to UBX-RXM-SFRB by removing padding on the last word
            *(G_Observer_t::u32_t *)(&packet.subframe.buffer[sizeof(G_Observer_t::u32_t) * 7]) >>= 6;
            check_subframe(packet);
            return;
          }
          // TODO GNSS
        }
      }

      void check_ephemeris(const G_Observer_t &observer){
        G_Packet_GPS_Ephemeris_Extended ephemeris;
        observer.fetch_ephemeris(ephemeris);
        if((week_number >= 0) && ephemeris.valid){
          ephemeris.WN += (week_number - (week_number % 0x400)); // Original WN is truncated to 10 bits.
          ephemeris.space_node = &gps_space_node;
          Handler::outer.updatable->update(ephemeris);
        }
      }

      /**
       * Extract the following data.
       * {class, id} = {0x02, 0x10 / 0x15} : raw measurement (pseudorange, carrier, doppler)
       * {class, id} = {0x02, 0x11 / 0x13} : subframe
       * {class, id} = {0x02, 0x31} : ephemeris
       */
      void check_rxm(const G_Observer_t &observer, const G_Observer_t::packet_type_t &packet_type){
        switch(packet_type.mid){
          case 0x10: { // RXM-RAW
            const unsigned int num_of_sv(observer[6 + 6]);
            if(num_of_sv == 0){return;}

            raw_data_t::gps_time_t current(observer.fetch_WN(), observer.fetch_ITOW());
#ifdef USE_GPS_SINGLE_DIFFERENCE_AS_RATE
            float_sylph_t delta_t(current - packet_raw_latest.raw_data.gpstime);
#endif
            packet_raw_latest.raw_data.gpstime = current;
            packet_raw_latest.itow = current.seconds;

            typedef raw_data_t::measurement_t dst_t;
            dst_t &dst(packet_raw_latest.raw_data.measurement);
#ifdef USE_GPS_SINGLE_DIFFERENCE_AS_RATE
            dst_t::mapped_type previous_pseudo_range(dst[raw_data_t::L1_PSEUDORANGE]);
#endif
            dst.clear();

            typedef dst_t::mapped_type::value_type v_t;
            for(int i(0); i < num_of_sv; i++){

              G_Observer_t::raw_measurement_t src(observer.fetch_raw(i));

              int prn(src.sv_number);
              dst[raw_data_t::L1_PSEUDORANGE].push_back(v_t(prn, src.pseudo_range));
              dst[raw_data_t::L1_CARRIER_PHASE].push_back(v_t(prn, src.carrier_phase));
              dst[raw_data_t::L1_DOPPLER].push_back(v_t(prn, src.doppler)); // positive sign for approaching satellite

              // calculate range rate derived from doppler
              dst[raw_data_t::L1_RANGE_RATE].push_back(v_t(
                  prn,
                  -src.doppler * gps_space_node_t::L1_WaveLength()));
            }

#ifdef USE_GPS_SINGLE_DIFFERENCE_AS_RATE
            if(dst[raw_data_t::L1_RANGE_RATE].empty()){
              // calculate range rate by using difference between current and previous range.
              dst[raw_data_t::L1_RANGE_RATE] = raw_data_t::difference(
                  dst[raw_data_t::L1_PSEUDORANGE], previous_pseudo_range, (1.0 / delta_t));
            }
#endif

            update(packet_raw_latest);
            return;
          }
          case 0x11: { // RXM-SFRB
            G_Packet_Data packet;
            observer.fetch_subframe(packet.subframe);
            check_subframe(packet);
            return;
          }
          case 0x13: { // RXM-SFRBX
            G_Packet_Data packet;
            unsigned int bytes((G_Observer_t::u8_t)observer[6 + 4] * 4); // numWords (1word = 32bits)
            if(bytes > sizeof(packet.subframe.buffer)){return;}
            observer.inspect(packet.subframe.buffer, bytes, 6 + 8);
            check_subframeX(
                (G_Observer_t::u8_t)observer[6 + 0],
                (G_Observer_t::u8_t)observer[6 + 1],
                packet);
            return;
          }
          case 0x15: { // RXM-RAWX
            const unsigned int num_of_measurement(observer[6 + 11]);
            if(num_of_measurement == 0){return;}

            {
              G_Observer_t::v8_t buf[10];
              observer.inspect(buf, sizeof(buf), 6);
              raw_data_t::gps_time_t current(
                  le_char2_2_num<G_Observer_t::u16_t>(*(buf + 8)), le_char8_2_num<double>(*buf));
              packet_raw_latest.raw_data.gpstime = current;
              packet_raw_latest.itow = current.seconds;
            }

            typedef raw_data_t::measurement_t dst_t;
            dst_t &dst(packet_raw_latest.raw_data.measurement);
            dst.clear();

            typedef dst_t::mapped_type::value_type v_t;
            for(int i(0); i < num_of_measurement; i++){
              unsigned int gnssID(observer[6 + 36 + (32 * i)]);
              if((gnssID != G_Observer_t::gnss_svid_t::GPS)
                  && (gnssID != G_Observer_t::gnss_svid_t::SBAS)){continue;} // TODO GNSS
              int prn(observer[6 + 37 + (32 * i)]); // svID

              G_Observer_t::v8_t buf[24];
              observer.inspect(buf, sizeof(buf), 6 + 16 + (32 * i));

              dst[raw_data_t::L1_PSEUDORANGE].push_back(v_t(prn, le_char8_2_num<double>(*buf)));
              dst[raw_data_t::L1_CARRIER_PHASE].push_back(v_t(prn, le_char8_2_num<double>(*(buf + 8))));
              float_sylph_t doppler(le_char4_2_num<float>(*(buf + 16)));
              dst[raw_data_t::L1_DOPPLER].push_back(v_t(prn, doppler));

              // calculate range rate derived from doppler
              dst[raw_data_t::L1_RANGE_RATE].push_back(v_t(
                  prn, -doppler * gps_space_node_t::L1_WaveLength()));
            }

            update(packet_raw_latest);
            return;
          }
          case 0x31: { // RXM-EPH
            check_ephemeris(observer);
            return;
          }
          default: return;
        }
      }

      static float_sylph_t le_char8_shift32(const char buf[8]) {
        float_sylph_t v(le_char4_2_num<unsigned int>(buf[0]));
        (v /= (1 << 16)) /= (1 << 16);
        return v + le_char4_2_num<unsigned int>(buf[4]);
      }

      void check_trk(const G_Observer_t &observer, const G_Observer_t::packet_type_t &packet_type){
        switch(packet_type.mid){
          case 0x0A: { // TRK-TRKD5
            if(week_number < 0){return;}
            int type(observer[6]);
            int offset, ranges, length_per_range;
            switch(type){
              case 6: { // currently, type(6) is only supported.
                offset = 86; length_per_range = 64;
                ranges = (G_Observer_t::u8_t)observer[7];
                int packet_size(observer.current_packet_size());

                if(packet_size != (offset + 2)
                    + (length_per_range * ranges)
                    + (20 * (G_Observer_t::u8_t)observer[8])){
                  return; // packet size error
                }
                break;
              }
              default: return;
            }

            if(ranges == 0){return;} // no measurement

            typedef raw_data_t::measurement_t dst_t;
            dst_t &dst(packet_raw_latest.raw_data.measurement);
            dst.clear();

            typedef dst_t::mapped_type::value_type v_t;

            float_sylph_t last_time_of_transmission(0);
            int ranges_valid(0);
            static const int TIME_OF_TRANSMISSION(raw_data_t::MEASUREMENT_ITEMS_PREDEFINED);

            for(int i(ranges); i > 0; --i, offset += length_per_range){
              int qi((G_Observer_t::u8_t)observer[offset + 41] & 0x07); // quality indicator
              if((qi < 4) || (qi > 7)){continue;}

              int prn;
              if(type == 6){
                //int freq((G_Observer_t::u8_t)observer[offset + 59] - 7);
                prn = (G_Observer_t::u8_t)observer[offset + 57];
                switch((G_Observer_t::u8_t)observer[offset + 56]){
                  case 5: // TODO QZSS(5)
                    prn += 192; continue;
                  case 0: case 1: // GPS(0), SBAS(1)
                    break;
                  case 2: case 3: case 6: default: // TODO GALILEO(2), BEIDO(3), GLONASS(6)
                    continue;
                }
              }else{
                prn = (G_Observer_t::u8_t)observer[offset + 34];
                if((prn > 32) && ((prn < 120) && (prn > 158))){continue;} // GPS or SBAS, TODO GNSS
              }

              // check phase lock
              unsigned int flag((G_Observer_t::u8_t)observer[offset + 54]);
              if((flag & 0x08) == 0){continue;}

              char buf[20];
              observer.inspect(buf, 20, offset);

              float_sylph_t time_of_transmission(1E-3 * le_char8_shift32(&buf[0]));
              dst[TIME_OF_TRANSMISSION].push_back(v_t(prn, time_of_transmission));
              if(time_of_transmission > last_time_of_transmission){
                last_time_of_transmission = time_of_transmission;
              }
              ++ranges_valid;

              if(qi > 6){
                float_sylph_t carrier(le_char8_shift32(&buf[8]));
                if(flag & 0x01){carrier += 0.5;}
                dst[raw_data_t::L1_CARRIER_PHASE].push_back(v_t(prn, -carrier));
              }

              float_sylph_t doppler(
                  (float_sylph_t)le_char4_2_num<G_Observer_t::s32_t>(*(buf + 16)) / (1 << 12));
              dst[raw_data_t::L1_DOPPLER].push_back(v_t(prn, doppler));

              // calculate range rate derived from doppler
              dst[raw_data_t::L1_RANGE_RATE].push_back(v_t(
                  prn, -doppler * gps_space_node_t::L1_WaveLength()));
            }

            if(ranges_valid == 0){return;}

            float_sylph_t est_time_of_reception(
                25E-3 * std::ceil((last_time_of_transmission + 66E-3) / 25E-3));
                // assume 25 ms alignment, transmission time is assumed to be between
                // 67ms(20200km, inc90deg) to 86ms(25785km, inc0deg); ref 72ms(21674km, inc45deg)

            packet_raw_latest.raw_data.gpstime.week = week_number;
            packet_raw_latest.raw_data.gpstime.seconds = est_time_of_reception;
            if(packet_raw_latest.raw_data.gpstime.seconds >= raw_data_t::gps_time_t::seconds_week){
              ++(packet_raw_latest.raw_data.gpstime.week);
              packet_raw_latest.raw_data.gpstime.seconds -= raw_data_t::gps_time_t::seconds_week;
            }
            packet_raw_latest.itow = packet_raw_latest.raw_data.gpstime.seconds;

            for(dst_t::mapped_type::const_iterator it(dst[TIME_OF_TRANSMISSION].begin());
                it != dst[TIME_OF_TRANSMISSION].end(); ++it){
              float_sylph_t delta_t(est_time_of_reception - it->second);
              if(delta_t > 100E-3){continue;} // Remove abnormal time lag (normally, from 67 to 86 ms);
              dst[raw_data_t::L1_PSEUDORANGE].push_back(v_t(it->first, gps_space_node_t::light_speed * delta_t));
            }

            update(packet_raw_latest);
            return;
          }
          case 0x0F: { // TRK-SFRBX
            G_Packet_Data packet;
            if(observer.current_packet_size() != 61){return;} // only GPS may be accepted
            observer.inspect(packet.subframe.buffer, sizeof(packet.subframe.buffer), 6 + 13);
            check_subframeX(
                (G_Observer_t::u8_t)observer[6 + 1],
                (G_Observer_t::u8_t)observer[6 + 2],
                packet);
            return;
          }
          default: return;
        }
      }

      void operator()(const G_Observer_t &observer){
        if(!observer.validate()){return;}

        G_Observer_t::packet_type_t packet_type(observer.packet_type());
        switch(packet_type.mclass){
          case 0x01: check_nav(observer, packet_type); break;
          case 0x02: check_rxm(observer, packet_type); break;
          case 0x03: check_trk(observer, packet_type); break;
          case 0x0B: {
            if(packet_type.mid == 0x31){
              check_ephemeris(observer);
            }
            break;
          }
        }
      }
    } g_handler;

    struct MHandler : public M_Observer_t, public Handler {
      bool previous_seek_next;
      M_Packet packet_latest;
      MHandler(StreamProcessor &invoker)
          : M_Observer_t(buffer_size),
          Handler(invoker),
          packet_latest() {
        previous_seek_next = M_Observer_t::ready();
      }
      ~MHandler(){}
      void operator()(const M_Observer_t &observer){
        if(!observer.validate()){return;}

        M_Observer_t::values_t values(observer.fetch_values());

        // Check value to remove outlier
        short *targets[] = {values.x, values.y, values.z};
        static const int threshold(200);
        for(int j(0); j < sizeof(targets) / sizeof(targets[0]); j++){
          for(int i(0); i < 3; i++){
            int diff_abs(abs(targets[j][i] - targets[j][3]));
            if((diff_abs > threshold) && (diff_abs < (4096 * 2 - threshold))){
              return;
            }
          }
        }

        float_sylph_t itow(observer.fetch_ITOW());
        if(options.reduce_1pps_sync_error){
          float_sylph_t delta_t(itow - packet_latest.itow);
          if((delta_t >= 1) && (delta_t < 2)){itow -= 1;}
        }
        packet_latest.itow = itow;

        // TODO: magnetic sensor axes must correspond to ones of accelerometer and gyro.
        packet_latest.mag[0] = values.x[3];
        packet_latest.mag[1] = values.y[3];
        packet_latest.mag[2] = values.z[3];

        Handler::outer.updatable->update(packet_latest);
      }
    } m_handler;

  protected:
    int invoked;
    istream *in;
    ostream *out_rinex_nav;
    
  public:
    StreamProcessor()
        : super_t(), updatable(&updatable_blackhole),
        in(NULL), invoked(0),
        out_rinex_nav(NULL),
        a_handler(*this),
        g_handler(*this),
        m_handler(*this) {

    }
    StreamProcessor(const StreamProcessor &another)
        : super_t(another), updatable(another.updatable),
        in(another.in), invoked(another.invoked),
        out_rinex_nav(another.out_rinex_nav),
        a_handler(*this),
        g_handler(*this),
        m_handler(*this) {
      a_handler = another.a_handler;
      g_handler = another.g_handler;
      m_handler = another.m_handler;
    }
    ~StreamProcessor(){
      if(invoked > 0){
        if(out_rinex_nav){
          RINEX_NAV_Writer<float_sylph_t>::write_all(*out_rinex_nav, g_handler.gps_space_node);
        }
      }
    }

    Updatable *&update_target() {
      return updatable;
    }

    const StandardCalibration &calibration() const{
      return a_handler.calibration;
    }

    const GHandler::gps_space_node_t &gps_space_node() const {
      return g_handler.gps_space_node;
    }
    GHandler::gps_space_node_t &gps_space_node() {
      return const_cast<GHandler::gps_space_node_t &>(
          static_cast<const StreamProcessor &>(*this).gps_space_node());
    }

    istream *&input() {
      return in;
    }

    /**
     * Process stream in units of 1 page
     * 
     * @param in stream
     * @return (bool) true when success, otherwise false.
     */
    bool process_1page(){
      char buffer[SYLPHIDE_PAGE_SIZE];
      
      int read_count;
      in->read(buffer, SYLPHIDE_PAGE_SIZE);
      read_count = static_cast<int>(in->gcount());
      if(in->fail() || (read_count == 0)){return false;}
      invoked++;
    
#if DEBUG
      cerr << "--read-- : " << invoked << " page" << endl;
      cerr << hex;
      for(int i = 0; i < read_count; i++){
        cerr 
          << setfill('0') 
          << setw(2)
          << (unsigned int)((unsigned char)buffer[i]) << ' ';
      }
      cerr << dec;
      cerr << endl;
#endif
    
      if(read_count < SYLPHIDE_PAGE_SIZE){
#if DEBUG
        cerr << "--skipped-- : " << invoked << " page ; count = " << read_count << endl;
#endif
      }

      switch(buffer[0]){
        case 'A':
          super_t::process_packet(
              buffer, read_count,
              a_handler, a_handler.previous_seek_next, a_handler);
          break;
        case 'G':
          super_t::process_packet(
              buffer, read_count,
              g_handler, g_handler.previous_seek_next, g_handler);
          if(g_handler.status.time_stamp == GHandler::status_t::TIME_STAMP_AFTER_END){ // Time check
            return false;
          }
          break;
        case 'M':
          if(!options.use_magnet){break;}
          super_t::process_packet(
              buffer, read_count,
              m_handler, m_handler.previous_seek_next, m_handler);
          break;
      }

      return true;
    }

    bool check_spec(const char *spec, const bool &dry_run = false){
      const char *value;
      if(value = Options::get_value(spec, "calib_file", false)){ // calibration file
        if(dry_run){return true;}
        cerr << "IMU Calibration file (" << value << ") reading..." << endl;
        istream &in(options.spec2istream(value));
        char buf[1024];
        while(!in.eof()){
          in.getline(buf, sizeof(buf));
          if(!buf[0]){continue;}
          if(!a_handler.calibration.check_spec(buf)){
            cerr << "unknown_calib_param! : " << buf << endl;
            return false;
          }
        }
        return true;
      }

      if(value = Options::get_value(spec, "lever_arm", false)){ // Lever Arm
        if(dry_run){return true;}
        double buf[3];
        if(std::sscanf(value, "%lf,%lf,%lf",
            &(buf[0]), &(buf[1]), &(buf[2])) != 3){
          cerr << "(error!) Lever arm option requires 3 arguments." << endl;
          return false;
        }
        for(int i(0); i < sizeof(buf) / sizeof(buf[0]); ++i){
          g_handler.lever_arm[i] = buf[i];
        }
        g_handler.packet_latest.lever_arm = &(g_handler.lever_arm);
        g_handler.packet_raw_latest.lever_arm = &(g_handler.lever_arm);
        std::cerr << "lever_arm: " << g_handler.lever_arm << std::endl;
        return true;
      }

      if(value = Options::get_value(spec, "rinex_nav", false)){
        if(dry_run){return true;}
        cerr << "RINEX Navigation file (" << value << ") reading..." << endl;
        istream &in(options.spec2istream(value));
        int ephemeris(RINEX_NAV_Reader<float_sylph_t>::read_all(
            in, g_handler.gps_space_node));
        if(ephemeris < 0){
          cerr << "(error!) Invalid format!" << endl;
          return false;
        }else{
          cerr << "rinex_nav: " << ephemeris << " items captured." << endl;
        }
        return true;
      }
      if(value = Options::get_value(spec, "out_rinex_nav", false)){
        if(dry_run){return true;}
        out_rinex_nav = &options.spec2ostream(value);
        cerr << "out_rinex_nav: " << value << endl;
        return true;
      }

      if(value = Options::get_value(spec, "F10.7", false)){
        if(dry_run){return true;}
        float_sylph_t f_10_7(std::atof(value));
        if((f_10_7 <= 0) || (f_10_7 > 1E3)){
          cerr << "(error!) Abnormal F10.7!" << value << endl;
          return false;
        }
        cerr << "F10.7: " << f_10_7 << endl;
        g_handler.solver_options.f_10_7 = f_10_7;
        g_handler.solver_options.insert_ionospheric_model(
            G_Packet_Measurement::raw_data_t::solver_t::options_t::IONOSPHERIC_NTCM_GL);
        return true;
      }

      return false;
    }
};

const unsigned int StreamProcessor::buffer_size = SYLPHIDE_PAGE_SIZE * 128;

typedef vector<StreamProcessor> processors_t;
processors_t processors;

template <class INS_GPS>
class INS_GPS_NAV<INS_GPS>::Helper {
  protected:
    enum {
      UNINITIALIZED,
      JUST_INITIALIZED,
      TIME_UPDATED,
      MEASUREMENT_UPDATED,
      WAITING_UPDATE,
    } status;
    INS_GPS_NAV<INS_GPS> &nav;
    const int min_a_packets_for_init; // must be greater than 0

    template <class T>
    struct PacketBuffer {
      const int max_size;
      typedef deque<T> buf_t;
      buf_t buf;
      PacketBuffer(const int &_max_size) : max_size(_max_size), buf() {}
      void push(const T &packet){
        if(buf.size() >= max_size){buf.pop_front();}
        buf.push_back(packet);
      }
    };

    typedef PacketBuffer<A_Packet> recent_a_t;
    recent_a_t recent_a;

    typedef PacketBuffer<M_Packet> recent_m_t;
    recent_m_t recent_m;

    vec3_t get_mag(const float_t &itow){
      if(recent_m.buf.size() < 2){
        return vec3_t(1, 0, 0); // heading is north
      }
      typename recent_m_t::buf_t::const_iterator
          it_a(nearest(recent_m.buf, itow, 2)),
          it_b(it_a + 1);
      float_t
          weight_a((it_b->itow - itow) / (it_b->itow - it_a->itow)),
          weight_b(1. - weight_a);
      /* Reduce excessive extrapolation.
       * The extrapolation is required, because M page combines several samples which are sometimes obtained late.
       * The threshold is +/- 2 steps.
       */
      if(weight_a > 3){
        weight_a = 1;
        weight_b = 0;
      }else if(weight_b > 3){
        weight_b = 1;
        weight_a = 0;
      }
      return (it_a->mag * weight_a) + (it_b->mag * weight_b);
    }

    gps_pvt_t gps_raw_pvt;
  public:
    template <class TimeStamp>
    struct TimeStampGenerator {
      void update(const TimePacket &packet){}
      TimeStamp operator()(const float_t &t) const {
        return (TimeStamp)t;
      }
    };

    template <class FloatT>
    struct TimeStampGenerator<CalendarTimeStamp<FloatT> > {
      typedef CalendarTimeStamp<FloatT> stamp_t;
      typename stamp_t::Converter itow2calendar;
      TimeStampGenerator() : itow2calendar() {
        itow2calendar.correction_sec
            = 60 * 60 * options.time_stamp.calendar_spec_parse().correction_hr;
      }
      void update(const TimePacket &packet){
        packet.apply<FloatT>(itow2calendar);
      }
      stamp_t operator()(const FloatT &itow) const {
        return stamp_t(itow2calendar.convert(itow), itow);
      }
    };

    TimeStampGenerator<typename INS_GPS::time_stamp_t> t_stamp_generator;

    void before_any_update(){
      if(status >= JUST_INITIALIZED){
        status = WAITING_UPDATE;
      }
    }

    void compass(const M_Packet &packet){
      recent_m.push(packet);
    }

    Helper(INS_GPS_NAV<INS_GPS> &_nav)
        : status(UNINITIALIZED), nav(_nav),
        min_a_packets_for_init(options.initial_attitude.mode == options.initial_attitude.FULL_GIVEN ? 1 : 0x10),
        recent_a(max(min_a_packets_for_init, 0x100)),
        recent_m(0x10),
        gps_raw_pvt(),
        t_stamp_generator() {
    }
  
  protected:
    template <class Base_INS_GPS>
    NAV::updated_items_t updated_items(
        const INS_GPS_Back_Propagate<Base_INS_GPS> *ins_gps) const {
      NAV::updated_items_t res;

      // When smoothing is activated
      switch(status){
        case MEASUREMENT_UPDATED: {
          float_t itow(recent_a.buf.back().itow);
          typedef typename INS_GPS_Back_Propagate<Base_INS_GPS>::snapshots_t snapshots_t;
          const snapshots_t &snapshots(ins_gps->get_snapshots());
          int index(0);
          for(typename snapshots_t::const_iterator it(snapshots.begin());
              it != snapshots.end();
              ++it, index++){
            if(it->elapsedT_from_last_correct >= options.back_propagate_property.back_propagate_depth){
              break;
            }

            if(index == 0){
              if(!options.dump_correct){continue;}
              it->ins_gps.set_header("BP_MU", t_stamp_generator(itow + it->elapsedT_from_last_correct));
              res.push_back(&(it->ins_gps));
            }else{
              if(!options.dump_update){continue;}
              it->ins_gps.set_header("BP_TU",  t_stamp_generator(itow + it->elapsedT_from_last_correct));
              res.push_back(&(it->ins_gps));
            }
          }
          break;
        }
      }

      return res;
    }

    NAV::updated_items_t updated_items(void *) const {
      NAV::updated_items_t res;

      switch(status){
        case TIME_UPDATED:
          if(!options.dump_update){break;}
          res.push_back(nav.ins_gps);
          break;
        case JUST_INITIALIZED:
        case MEASUREMENT_UPDATED:
          if(!options.dump_correct){break;}
          res.push_back(nav.ins_gps);
          break;
      }
      return res;
    }

  public:
    NAV::updated_items_t updated_items() const {
      return updated_items(nav.ins_gps);
    }

  protected:
    void time_update(const A_Packet &a_packet, float_t deltaT){

      static const int one_week(60 * 60 * 7 * 24);
      if(deltaT <= -(one_week / 2)){ // Check roll over
        deltaT += one_week;
      }

      // Check interval from the last time update
#define INTERVAL_THRESHOLD 10
      if((deltaT <= 0) || (deltaT >= INTERVAL_THRESHOLD)){
        // Skip update when discontinuity is too large.
        return;
      }

      nav.update(a_packet.accel, a_packet.omega, deltaT);
      status = TIME_UPDATED;
    }

  public:
    /**
     * Perform time update by using acceleration and angular speed obtained with accelerometer and gyro.
     * 
     * @param a_packet acceleration and angular speed
     */
    void time_update(const A_Packet &a_packet){

      if(status >= JUST_INITIALIZED){
        const A_Packet &previous(recent_a.buf.back());

        // Check interval from the last time update
        float_t deltaT(previous.interval(a_packet));
        time_update(a_packet, deltaT);
        nav.ins_gps->set_header("TU",  t_stamp_generator(a_packet.itow));
      }

      recent_a.push(a_packet);
    }

  protected:
    void time_update_after_initialization(const Packet &g_packet){
      typename recent_a_t::buf_t::const_reverse_iterator it_r(recent_a.buf.rbegin());
      for(; it_r != recent_a.buf.rend(); ++it_r){
        if(g_packet.interval_rollover(*it_r) <= 0){break;}
      }
      const Packet *packet(&g_packet);
      for(typename recent_a_t::buf_t::const_iterator it(it_r.base()); // it_r.base() position is not it_r position!!
          it != recent_a.buf.end(); ++it){
        time_update(*it, packet->interval(*it));
        packet = &(*it);
      }
      nav.ins_gps->set_header("MU",  t_stamp_generator(g_packet.itow));
    }

    void initialize_common(
        const float_t &itow,
        const float_t &latitude,
        const float_t &longitude,
        const float_t &height,
        const float_t &v_north,
        const float_t &v_east,
        const float_t &v_down){

      float_t
          yaw(deg2rad(options.initial_attitude.yaw_deg)),
          pitch(deg2rad(options.initial_attitude.pitch_deg)),
          roll(deg2rad(options.initial_attitude.roll_deg));

      while(options.initial_attitude.mode < options.initial_attitude.FULL_GIVEN){
        // Estimate initial attitude by using accelerometer and magnetic sensor (if available) under static assumption

        // Normalization
        vec3_t acc(0, 0, 0);
        for(typename recent_a_t::buf_t::iterator it(recent_a.buf.begin());
            it != recent_a.buf.end();
            ++it){
          acc += it->accel;
        }
        acc /= recent_a.buf.size();
        vec3_t acc_reg(-acc / acc.abs());

        // Estimate roll angle
        roll = atan2(acc_reg[1], acc_reg[2]);
        if(options.initial_attitude.mode >= options.initial_attitude.YAW_PITCH){break;}

        // Estimate pitch angle
        pitch = -asin(acc_reg[0]);
        if(options.initial_attitude.mode >= options.initial_attitude.YAW_ONLY){break;}

        // Estimate yaw when magnetic compass is available
        if(!recent_m.buf.empty()){
          yaw = nav.get_mag_yaw(get_mag(itow), pitch, roll,
              latitude, longitude, height);
        }
        break;
      }

      status = JUST_INITIALIZED;

      cerr << "Init : " << setprecision(10) << itow << endl;
      cerr << "Initial attitude (yaw, pitch, roll) [deg]: "
          << rad2deg(yaw) << ", "
          << rad2deg(pitch) << ", "
          << rad2deg(roll) << endl;

      nav.ins_gps->initPosition(latitude, longitude, height);
      nav.ins_gps->initVelocity(v_north, v_east, v_down);
      nav.ins_gps->initAttitude(yaw, pitch, roll);

      for(char buf[0x4000]; !options.init_misc->eof(); ){ // Miscellaneous setup
        options.init_misc->getline(buf, sizeof(buf));
        nav.init_misc(buf);
      }
    }

    void time_update_before_measurement_update(const float_t &advanceT, void *){
      if(advanceT <= 0){return;}
      // Time update up to the GPS observation
      time_update(recent_a.buf.back(), advanceT);
    }

    template <class Base_INS_GPS>
    void time_update_before_measurement_update(const float_t &advanceT, INS_GPS_RealTime<Base_INS_GPS> *){
      return;
    }

    template <class GPS_Packet>
    void measurement_update_common(const GPS_Packet &g_packet){
      cerr << "MU : " << setprecision(10) << g_packet.itow << endl;

      // calculate GPS data timing;
      // negative(realtime mode, delayed), or slightly positive(other modes, because of already sorted)
      float_t gps_advance(recent_a.buf.back().interval(g_packet));
      time_update_before_measurement_update(gps_advance, nav.ins_gps);

      if(g_packet.lever_arm){ // When use lever arm effect.
        vec3_t omega_b2i_4n;
        int packets_for_mean(0x10), i(0);
        typename recent_a_t::buf_t::const_iterator it(
            nearest(recent_a.buf, g_packet.itow, packets_for_mean));
        for(; (i < packets_for_mean) && (it != recent_a.buf.end()); i++, it++){
          omega_b2i_4n += it->omega;
        }
        omega_b2i_4n /= i;
        nav.correct(
            g_packet,
            *g_packet.lever_arm,
            omega_b2i_4n,
            gps_advance);
      }else{ // When do not use lever arm effect.
        nav.correct(
            g_packet,
            gps_advance);
      }
      if(!recent_m.buf.empty()){ // When magnetic sensor is activated, try to perform yaw compensation
        if((options.yaw_correct_with_mag_when_speed_less_than_ms > 0)
            && (pow(nav.ins_gps->v_north(), 2) + pow(nav.ins_gps->v_east(), 2)) < pow(options.yaw_correct_with_mag_when_speed_less_than_ms, 2)){
          nav.correct_yaw(nav.get_mag_delta_yaw(get_mag(g_packet.itow), *nav.ins_gps));
        }
      }
      status = MEASUREMENT_UPDATED;
      nav.ins_gps->set_header("MU");
    }

  public:
    /**
     * Perform measurement update by using position and velocity obtained with GPS receiver.
     * 
     * @param g_packet observation data of GPS receiver
     */
    void measurement_update(const G_Packet &g_packet, void *){

      if(g_packet.solution.sigma_2d >= options.gps_threshold.cont_acc_2d){ // When estimated accuracy is too big, skip.
        return;
      }
      if(status >= JUST_INITIALIZED){
        measurement_update_common(g_packet);
      }else if((recent_a.buf.size() >= min_a_packets_for_init)
          && (std::abs(recent_a.buf.front().itow - g_packet.itow) < (0.1 * recent_a.buf.size())) // time synchronization check
          && (g_packet.solution.sigma_2d <= options.gps_threshold.init_acc_2d)
          && (g_packet.solution.sigma_height <= options.gps_threshold.init_acc_v)){

        /*
         * Filter is activated when the estimate error in horizontal and vertical positions are under 20 and 10 meters, respectively.
         */
        initialize_common(
            g_packet.itow,
            g_packet.solution.latitude, g_packet.solution.longitude, g_packet.solution.height,
            g_packet.solution.v_n, g_packet.solution.v_e, g_packet.solution.v_d);
        time_update_after_initialization(g_packet);
      }
    }

    template <typename BaseFINS>
    void measurement_update(const G_Packet &g_packet,
        INS_GPS2_Tightly<BaseFINS> *){
      return;
    }

    void measurement_update(const G_Packet_Measurement &g_packet, void *){
      return;
    }

    /**
     * Perform measurement update by using raw measurement obtained with GPS receiver.
     *
     * @param g_packet observation data of GPS receiver
     */
    template <typename BaseFINS>
    void measurement_update(const G_Packet_Measurement &g_packet,
        INS_GPS2_Tightly<BaseFINS> *ins_gps){

      g_packet.take_consistency();
      if(options.out_raw_pvt && g_packet.get_pvt(gps_raw_pvt)){
        (*(options.out_raw_pvt)) << gps_raw_pvt << std::endl;
      }

      if(status >= JUST_INITIALIZED){
        measurement_update_common(g_packet);
      }else if((recent_a.buf.size() >= min_a_packets_for_init)
          && (std::abs(recent_a.buf.front().itow - g_packet.itow) < (0.1 * recent_a.buf.size())) // time synchronization check
          && g_packet.get_pvt(gps_raw_pvt)){

        initialize_common(
            g_packet.itow,
            gps_raw_pvt.user_position.llh.latitude(),
            gps_raw_pvt.user_position.llh.longitude(),
            gps_raw_pvt.user_position.llh.height(),
            gps_raw_pvt.user_velocity_enu.north(),
            gps_raw_pvt.user_velocity_enu.east(),
            -gps_raw_pvt.user_velocity_enu.up());

        ins_gps->clock_error(g_packet.raw_data.clock_index) = gps_raw_pvt.receiver_error;
        ins_gps->clock_error_rate(g_packet.raw_data.clock_index) = gps_raw_pvt.receiver_error_rate;
        
        time_update_after_initialization(g_packet);
      }
    }
  public:
    template <class GPS_Packet>
    void measurement_update(const GPS_Packet &g_packet){
      measurement_update(g_packet, nav.ins_gps);
    }
};

class NAV_Generator {
  private:
    template <class T>
    static NAV *final(){
      return INS_GPS_NAV_Factory<typename T::product>::get_nav(processors.front().calibration());
    }
    template <class T>
    static NAV *check_bias(){
      return options.est_bias ? final<typename T::template bias<> >() : final<T>();
    }
    template <class T>
    static NAV *check_coupling(){
      switch(options.ins_gps_integration){
        case Options::INS_GPS_INTEGRATION_TIGHTLY: // Tightly
          return check_bias<typename T::template tightly<> >();
        case Options::INS_GPS_INTEGRATION_LOOSELY: // Loosely
        default:
          return check_bias<T>();
      }
    }
    template <class T>
    static NAV *check_udkf(){
      return options.use_udkf
          ? check_coupling<typename T::template kf<KalmanFilterUD> >()
          : check_coupling<typename T::template kf<KalmanFilter> >();
    }
    template <class T>
    static NAV *check_egm(){
      return options.use_egm ? check_udkf<typename T::template egm<> >() : check_udkf<T>();
    }
  public:
    static NAV *generate(){
      switch(options.time_stamp.mode){
        case Options::time_stamp_t::CALENDAR_TIME:
          return check_egm<INS_GPS_Factory<
              INS_NAVData<INS<float_sylph_t>, CalendarTimeStamp<float_sylph_t> > > >();
        case Options::time_stamp_t::ITOW:
        default:
          return check_egm<INS_GPS_Factory<
              INS_NAVData<INS<float_sylph_t> > > >();
      }
    }
};

void loop(){
  struct NAV_Manager {
    NAV *nav;
    NAV_Manager() : nav(NAV_Generator::generate()){}
    ~NAV_Manager(){
      delete nav;
    }
  } nav_manager;
  
  nav_manager.nav->label(options.out());

  // TODO multiple log stream will be support.
  StreamProcessor &proc(processors.front());
  if(options.ins_gps_sync_strategy == Options::INS_GPS_SYNC_REALTIME){
    // Realtime mode supports only one stream.
    proc.update_target() = nav_manager.nav;
    while(proc.process_1page());
    return;
  }

  struct buffer_t : public Updatable {
    typedef deque<const Packet *> packet_pool_t;
    packet_pool_t packet_pool;
    NAV &nav;
    void sort_and_apply(int packets){
      stable_sort(packet_pool.begin(), packet_pool.end(), Packet::compare_rollover);
      while(packets-- > 0){
        packet_pool_t::reference front(packet_pool.front());
        front->apply(nav);
        delete front;
        packet_pool.pop_front();
      }
    }
    void sort_and_apply2 () {
      if(packet_pool.size() < 0x200){return;}
      sort_and_apply(0x100);
    }
    buffer_t(NAV &_nav) : packet_pool(), nav(_nav) {}
    ~buffer_t() {
      sort_and_apply(packet_pool.size());
    }
#define update_func(type) \
virtual void update(const type &packet){ \
  packet_pool.push_back(new type(packet)); \
  sort_and_apply2(); \
}
    update_func(A_Packet);
    update_func(G_Packet);
    update_func(G_Packet_Measurement);
    update_func(G_Packet_GPS_Ephemeris);
    update_func(M_Packet);
    update_func(TimePacket);
#undef update_func
  } buffer(*nav_manager.nav);
  proc.update_target() = &buffer;

  while(proc.process_1page());
}

int main(int argc, char *argv[]){
  
  cout << setprecision(10);
  cerr << setprecision(10);

  cerr << "NinjaScan INS/GPS post-processor" << endl;
  cerr << "Usage: (exe) [options] log.dat" << endl;
  if(argc < 2){
    cerr << "Error: too few arguments; " << argc << " < min(2)" << endl;
    return -1;
  }

  // option check...
  cerr << "Option checking..." << endl;

  typedef vector<int> args_t;
  args_t args_proc_common;

  for(int arg_index(1); arg_index < argc; arg_index++){
    StreamProcessor stream_processor;
    args_t args_proc(args_proc_common);

    bool flag_common(false);
    for(; arg_index < argc; arg_index++){
      bool flag_common_current(flag_common);
      {
        /* check --common, which indicates its next argument will be applied
         * to the following logs.
         */
        const char *value(Options::get_value(argv[arg_index], "common"));
        if(value){
          flag_common = Options::is_true(value);
          continue;
        }else{
          flag_common = false;
        }
      }

      if(stream_processor.check_spec(argv[arg_index], true)){
        args_proc.push_back(arg_index);
        if(flag_common_current){args_proc_common.push_back(arg_index);}
        continue;
      }

      if(options.check_spec(argv[arg_index])){continue;}

      cerr << "Log file(" << processors.size() << "): ";
      istream &in(options.spec2istream(argv[arg_index]));
      stream_processor.input()
          = options.in_sylphide ? new SylphideIStream(in, SYLPHIDE_PAGE_SIZE) : &in;

      for(args_t::const_iterator it(args_proc.begin());
          it != args_proc.end(); ++it){
        if(!stream_processor.check_spec(argv[*it])){
          exit(-1); // some error occurred
        }
      }
      args_proc.clear();

      processors.push_back(stream_processor);
      cerr << stream_processor.calibration() << endl;
      break;
    }

    if(args_proc.size() > args_proc_common.size()){
      cerr << "(error!) unused log specific arguments." << endl;
      exit(-1);
    }
  }

  if(processors.empty()){
    cerr << "(error!) No log file." << endl;
    exit(-1);
  }
  if(processors.size() > 1){ // TODO multiple log stream will be support.
    cerr << "(error!) too many log." << endl;
    exit(-1);
  }

  if(options.out_sylphide){
    options._out = new SylphideOStream(options.out(), SYLPHIDE_PAGE_SIZE);
  }else{
    options.out() << setprecision(10);
  }
  options.out_debug() << setprecision(16);

  if(options.out_raw_pvt){
    gps_pvt_t::label(*options.out_raw_pvt);
    (*options.out_raw_pvt) << endl;
    options.out_raw_pvt->precision(12);
  }

  loop();

  return 0;
}
