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
 * with GPS time.
 *
 * It is briefly technically noted that this program utilizes loosely-coupled INS/GPS algorithm,
 * which implies at least four GPS satellites must be available to output the results.
 * In addition, this program is not suitable to be used for real-time application.
 * This is because its processing strategy is post-process, that is, the data is sorted
 * in time-series order before application of the INS/GPS algorithm,
 * in order to compensate for the output delay of a GPS receiver installed on the logger.
 *
 * Its usage is
 *   INS_GPS [option] <log.dat>,
 * where <log.dat> is mandatory value pointed to a log file gathered by a logger.
 * There are some reserved values; If <log.dat> equals to - (hyphen),
 * the program will try to read log data from the standard input.
 * Or, when <log.dat> is COMx for Windows or /dev/ttyACMx for *NIX,
 * the program will try to read data from the requested serial port.
 *
 * The [option] is composed of optional values separated by space.
 * Its representatives are the followings:
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
#include <cmath>
#include <cstring>
#include <cctype>

#include <vector>
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
VECTOR3_NO_FLY_WEIGHT(float_sylph_t);
QUATERNION_NO_FLY_WEIGHT(float_sylph_t);

#include "navigation/INS_GPS2.h"
#include "navigation/INS_GPS_Synchronization.h"
#include "navigation/INS_GPS_Debug.h"
#include "navigation/BiasEstimation.h"

#include "navigation/MagneticField.h"

#include "analyze_common.h"

struct Options : public GlobalOptions<float_sylph_t> {
  typedef GlobalOptions<float_sylph_t> super_t;

  // Output
  bool dump_update; ///< True for dumping states at time updates
  bool dump_correct; ///< True for dumping states at measurement updates
  bool dump_stddev; ///< True for dumping standard deviations
  bool out_is_N_packet; ///< True for NPacket formatted outputs

  // Navigation strategies
  enum {
    INS_GPS_SYNC_OFFLINE,
    INS_GPS_SYNC_BACK_PROPAGATION, ///< a.k.a, smoothing
    INS_GPS_SYNC_REALTIME,
  } ins_gps_sync_strategy;
  bool est_bias; ///< True for performing bias estimation
  bool use_udkf; ///< True for UD Kalman filtering

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

  // Debug
  INS_GPS_Debug_Property debug_property;

  Options()
      : super_t(),
      dump_update(true), dump_correct(false), dump_stddev(false),
      out_is_N_packet(false),
      ins_gps_sync_strategy(INS_GPS_SYNC_OFFLINE),
      est_bias(true), use_udkf(false),
      back_propagate_property(),
      realttime_property(),
      gps_fake_lock(false), gps_threshold(),
      use_magnet(false),
      mag_heading_accuracy_deg(3),
      yaw_correct_with_mag_when_speed_less_than_ms(5),
      initial_attitude(),
      init_misc_buf(), init_misc(&init_misc_buf),
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

    CHECK_OPTION(back_propagate, true,
        if(is_true(value)){ins_gps_sync_strategy = INS_GPS_SYNC_BACK_PROPAGATION;},
        (ins_gps_sync_strategy == INS_GPS_SYNC_BACK_PROPAGATION ? "on" : "off"));
    CHECK_OPTION(realtime, true,
        if(is_true(value)){ins_gps_sync_strategy = INS_GPS_SYNC_REALTIME;},
        (ins_gps_sync_strategy == INS_GPS_SYNC_REALTIME ? "on" : "off"));
    CHECK_OPTION_BOOL(est_bias);
    CHECK_OPTION_BOOL(use_udkf);
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

    CHECK_OPTION(debug, false,
        if(!debug_property.check_debug_property_spec(value)){break;},
        debug_property.show_debug_property());
#undef CHECK_OPTION
    
    return super_t::check_spec(spec);
  }
} options;

struct A_Packet;
struct G_Packet;
struct M_Packet;

class NAV : public NAVData<float_sylph_t> {
  public:
    typedef std::vector<const NAVData<float_sylph_t> *> updated_items_t;
    virtual ~NAV(){}
  public:
    virtual updated_items_t updated_items() const {
      return updated_items_t();
    }
    virtual void inspect(std::ostream &out) const {}
    virtual float_sylph_t &operator[](const unsigned &index) = 0;
    
    virtual NAV &update(const A_Packet &){
      return *this;
    }
    virtual NAV &update(const G_Packet &){
      return *this;
    }
    virtual NAV &update(const M_Packet &){
      return *this;
    }

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
      MagneticField::filed_components_res_t mag_model(
          MagneticField::filed_components(IGRF11::IGRF2010,
              latitude, longitude, altitude));
      vec_t mag_filed(mag_model.north, mag_model.east, mag_model.down);

      // Get the correction angle with the model
      return std::atan2(mag_filed[1], mag_filed[0])
          - std::atan2(mag_horizontal[1], mag_horizontal[0]);
    }

    float_sylph_t get_mag_delta_yaw(
        const Vector3<float_sylph_t> &mag,
        const Quaternion<float_sylph_t> &attitude){

      return get_mag_delta_yaw(mag, attitude, latitude(), longitude(), height());
    }

    float_sylph_t get_mag_delta_yaw(
        const Vector3<float_sylph_t> &mag){

      return get_mag_delta_yaw(mag,
          INS<float_sylph_t>::euler2q(euler_psi(), euler_theta(), euler_phi()),
          latitude(), longitude(), height());
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

    float_sylph_t get_mag_yaw(const Vector3<float_sylph_t> &mag){

      return get_mag_yaw(
          mag,
          euler_theta(), euler_phi(),
          latitude(), longitude(), height());
    }
};

/**
 * Base class of log data, which has time stamp.
 */
struct Packet{
  virtual ~Packet() {}
  virtual Packet *clone() const = 0;
  virtual NAV &apply(NAV &nav) const = 0;

  float_sylph_t itow;

  /**
   * Get interval time between another one
   *
   * @param another one
   */
  float_sylph_t interval(const Packet &another) const {
    return another.itow - itow;
  }
  static bool compare(const Packet *a, const Packet *b) {
    return a->itow < b->itow;
  }
};

/**
 * Inertial and temperature sensor data (ADC raw value)
 */
struct A_Packet : public Packet {
  Packet *clone() const {
    return new A_Packet(*this);
  }
  NAV &apply(NAV &nav) const {
    return nav.update(*this);
  }
  Vector3<float_sylph_t> accel; ///< Acceleration
  Vector3<float_sylph_t> omega; ///< Angular speed
};

/**
 * GPS data
 */
struct G_Packet : public Packet {
  Packet *clone() const {
    return new G_Packet(*this);
  }
  NAV &apply(NAV &nav) const {
    return nav.update(*this);
  }

  GPS_Solution<float_sylph_t> solution;
  Vector3<float_sylph_t> *lever_arm;

  G_Packet() : solution(), lever_arm(NULL) {}

  operator const GPS_Solution<float_sylph_t> &() const {
    return solution;
  }
};

/**
 * Magnetic sensor data
 */
struct M_Packet : public Packet {
  Packet *clone() const {
    return new M_Packet(*this);
  }
  NAV &apply(NAV &nav) const {
    return nav.update(*this);
  }

  Vector3<float_sylph_t> mag;
};

typedef INS_GPS2<
    float_sylph_t,
    KalmanFilter> ins_gps_ekf_t;
typedef INS_GPS2<
    float_sylph_t,
    KalmanFilterUD> ins_gps_ekf_ud_t;
typedef INS_GPS2_BiasEstimated<
    float_sylph_t,
    KalmanFilter> ins_gps_bias_ekf_t;
typedef INS_GPS2_BiasEstimated<
    float_sylph_t,
    KalmanFilterUD> ins_gps_bias_ekf_ud_t;

template <class INS_GPS>
class INS_GPS_NAVData;

template <class INS_GPS>
class INS_GPS_NAV : public NAV {
  public:
    typedef INS_GPS ins_gps_t;
    class Helper;
  protected:
    INS_GPS *ins_gps;
    Helper helper;

    template <class Calibration>
    void setup_filter(const Calibration &calibration, void *){}

    template <class Calibration, class BaseINS, template <class> class Filter>
    void setup_filter(
        const Calibration &calibration,
        Filtered_INS2<BaseINS, Filter> *fins) {
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
        Matrix<float_sylph_t> P(ins_gps->getFilter().getP());

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
        const Vector3<float_sylph_t>
            accel(calibration.sigma_accel()),
            gyro(calibration.sigma_gyro());

        Matrix<float_sylph_t> Q(ins_gps->getFilter().getQ());
        
        Q(0, 0) = pow(accel.getX(), 2);
        Q(1, 1) = pow(accel.getY(), 2);
        Q(2, 2) = pow(accel.getZ(), 2);
        Q(3, 3) = pow(gyro.getX(), 2);
        Q(4, 4) = pow(gyro.getY(), 2);
        Q(5, 5) = pow(gyro.getZ(), 2);
        Q(6, 6) = 1E-6; //1E-14
        
        ins_gps->getFilter().setQ(Q);
      }
    }

    template <class Calibration, class BaseFINS>
    void setup_filter(
        const Calibration &calibration,
        Filtered_INS_BiasEstimated<BaseFINS> *fins) {

      setup_filter(calibration, (BaseFINS *)fins);

      {
        Matrix<float_sylph_t> P(ins_gps->getFilter().getP());
        static const unsigned NP(
            Filtered_INS_BiasEstimated<BaseFINS>::P_SIZE_WITHOUT_BIAS);
        P(NP,     NP)     = P(NP + 1, NP + 1) = P(NP + 2, NP + 2) = 1E-4; // for accelerometer bias drift
        P(NP + 3, NP + 3) = P(NP + 4, NP + 4) = P(NP + 5, NP + 5) = 1E-7; // for gyro bias drift
        ins_gps->getFilter().setP(P);
      }

      {
        Matrix<float_sylph_t> Q(ins_gps->getFilter().getQ());
        static const unsigned NQ(
            Filtered_INS_BiasEstimated<BaseFINS>::Q_SIZE_WITHOUT_BIAS);
        Q(NQ,     NQ)     = Q(NQ + 1, NQ + 1) = Q(NQ + 2, NQ + 2) = 1E-6; // for accelerometer bias drift
        Q(NQ + 3, NQ + 3) = Q(NQ + 4, NQ + 4) = Q(NQ + 5, NQ + 5) = 1E-8; // for gyro bias drift
        ins_gps->getFilter().setQ(Q);
      }

      ins_gps->beta_accel() *= 0.1;
      ins_gps->beta_gyro() *= 0.1; //mems_g.BETA;
    }

    template <class Calibration, class Base_INS_GPS>
    void setup_filter(const Calibration &calibration, INS_GPS_Back_Propagate<Base_INS_GPS> *ins_gps){
      setup_filter(calibration, (Base_INS_GPS *)ins_gps);
      ins_gps->setup_back_propagation(options.back_propagate_property);
    }

    template <class Calibration, class Base_INS_GPS>
    void setup_filter(const Calibration &calibration, INS_GPS_RealTime<Base_INS_GPS> *ins_gps){
      setup_filter(calibration, (Base_INS_GPS *)ins_gps);
      ins_gps->setup_realtime(options.realttime_property);
    }

    template <class Calibration, class Base_INS_GPS>
    void setup_filter(const Calibration &calibration, INS_GPS_Debug<Base_INS_GPS> *ins_gps){
      setup_filter(calibration, (Base_INS_GPS *)ins_gps);
      ins_gps->setup_debug(options.debug_property);
    }

  public:
    template <class Calibration>
    INS_GPS_NAV(const Calibration &calibration)
        : NAV(),
        ins_gps(new INS_GPS()), helper(*this) {
      setup_filter(calibration, ins_gps);
    }
    virtual ~INS_GPS_NAV() {
      delete ins_gps;
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
    static void set_matrix_full(Matrix<float_sylph_t> &mat, const char *spec){
      char *_spec(const_cast<char *>(spec));
      for(int i(0); i < mat.rows(); i++){
        for(int j(0); j < mat.columns(); j++){
          mat(i, j) = std::strtod(_spec, &_spec);
        }
      }
    }
    static void set_matrix_diagonal(Matrix<float_sylph_t> &mat, const char *spec){
      char *_spec(const_cast<char *>(spec));
      for(int i(0); i < mat.rows(); i++){
        mat(i, i) = std::strtod(_spec, &_spec);
      }
    }
    static void set_matrix_1element(Matrix<float_sylph_t> &mat, const char *spec){
      char *_spec(const_cast<char *>(spec));
      int i((int)std::strtol(_spec, &_spec, 10));
      int j((int)std::strtol(_spec, &_spec, 10));
      mat(i, j) = std::strtod(_spec, &_spec);
    }

    bool init_misc(const char *line, void *){
      return false;
    }
    template <class BaseINS, template <class> class Filter>
    bool init_misc(const char *line, Filtered_INS2<BaseINS, Filter> *fins){
      const char *value;

      while(true){
        Matrix<float_sylph_t> P(ins_gps->getFilter().getP());
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
        Matrix<float_sylph_t> Q(ins_gps->getFilter().getQ());
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

      return false;
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

#define MAKE_PROXY_FUNC(fname) \
float_sylph_t fname() const {return ins_gps->fname();}
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
    MAKE_PROXY_FUNC(time_stamp);
#undef MAKE_PROXY_FUNC
    
    float_sylph_t &operator[](const unsigned &index){return ins_gps->operator[](index);}
    
    NAV &update(
        const Vector3<float_sylph_t> &accel, 
        const Vector3<float_sylph_t> &gyro, 
        const float_sylph_t &elapsedT){
      ins_gps->update(accel, gyro, elapsedT);
      return *this;
    }
  
  public:
    NAV &correct(const G_Packet &gps){
      ins_gps->correct(gps);
      return *this;
    }

    NAV &correct(
        const G_Packet &gps,
        const Vector3<float_sylph_t> &lever_arm_b,
        const Vector3<float_sylph_t> &omega_b2i_4b){
      ins_gps->correct(gps, lever_arm_b, omega_b2i_4b);
      return *this;
    }

  protected:
    bool setup_correct(const float_sylph_t &advanceT, void *){
      return true;
    }

    template <class INS_GPS_orig>
    bool setup_correct(const float_sylph_t &advanceT, INS_GPS_RealTime<INS_GPS_orig> *){
      return ins_gps->setup_correct(advanceT);
    }

  public:
    NAV &correct(
        const G_Packet &gps,
        const float_sylph_t &advanceT){
      if(setup_correct(advanceT, ins_gps)){
        return correct(gps);
      }else{
        return *this;
      }
    }

    NAV &correct(
        const G_Packet &gps,
        const Vector3<float_sylph_t> &lever_arm_b,
        const Vector3<float_sylph_t> &omega_b2i_4b,
        const float_sylph_t &advanceT){
      if(setup_correct(advanceT, ins_gps)){
        return correct(gps, lever_arm_b, omega_b2i_4b);
      }else{
        return *this;
      }
    }

    NAV &correct_yaw(const float_sylph_t &delta_yaw){
      ins_gps->correct_yaw(delta_yaw, pow(deg2rad(options.mag_heading_accuracy_deg), 2));
      return *this;
    }
    
    void label(std::ostream &out) const {
      ins_gps->label(out);
    }

    void dump(std::ostream &out) const {
      ins_gps->dump(out);
    }

    updated_items_t updated_items() const {
      return helper.updated_items();
    }

    NAV &update(const A_Packet &packet){
      helper.before_any_update();
      helper.time_update(packet);
      return *this;
    }
    NAV &update(const G_Packet &packet){
      helper.before_any_update();
      helper.measurement_update(packet);
      return *this;
    }
    NAV &update(const M_Packet &packet){
      helper.before_any_update();
      helper.compass(packet);
      return *this;
    }
};

template <class INS_GPS>
struct INS_GPS_NAV_Factory {
  template <class Calibration>
  static NAV *get_nav(const Calibration &calibration){
    if(options.debug_property.debug_target == INS_GPS_Debug_Property::DEBUG_NONE){
      switch(options.ins_gps_sync_strategy){
        case Options::INS_GPS_SYNC_BACK_PROPAGATION:
          return new INS_GPS_NAV<
              INS_GPS_Back_Propagate<
                INS_GPS_NAVData<INS_GPS> > >(calibration);
        case Options::INS_GPS_SYNC_REALTIME:
          return new INS_GPS_NAV<
              INS_GPS_RealTime<
                INS_GPS_NAVData<INS_GPS> > >(calibration);
        default:
          return new INS_GPS_NAV<
                INS_GPS_NAVData<INS_GPS> >(calibration);
      }
    }else if(options.debug_property.debug_target == INS_GPS_Debug_Property::DEBUG_PURE_INERTIAL){
      return new INS_GPS_NAV<INS_GPS_NAVData<
          INS_GPS_Debug_PureInertial<INS_GPS> > >(calibration);
    }else{
      switch(options.ins_gps_sync_strategy){
        case Options::INS_GPS_SYNC_BACK_PROPAGATION:
          return new INS_GPS_NAV<INS_GPS_Debug<
              INS_GPS_Back_Propagate<
                INS_GPS_NAVData<INS_GPS> > > >(calibration);
        case Options::INS_GPS_SYNC_REALTIME:
          return new INS_GPS_NAV<INS_GPS_Debug<
              INS_GPS_RealTime<
                INS_GPS_NAVData<INS_GPS> > > >(calibration);
        default:
          return new INS_GPS_NAV<INS_GPS_Debug<
                INS_GPS_NAVData<INS_GPS> > >(calibration);
      }
    }
  }
};

template <class INS_GPS>
class INS_GPS_NAVData : public INS_GPS, public NAVData<typename INS_GPS::float_t> {
  protected:
    mutable const char *mode;
    mutable typename INS_GPS::float_t itow;
  public:
    INS_GPS_NAVData() : INS_GPS(), mode("N/A"), itow(0) {}
    INS_GPS_NAVData(const INS_GPS_NAVData<INS_GPS> &orig, const bool &deepcopy = false)
        : INS_GPS(orig, deepcopy), mode(orig.mode), itow(orig.itow) {}
    ~INS_GPS_NAVData(){}
#define MAKE_PROXY_FUNC(fname) \
typename INS_GPS::float_t fname() const {return INS_GPS::fname();}
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
    typename INS_GPS::float_t time_stamp() const {return itow;}

    void set_header(const char *_mode) const {
      mode = _mode;
    }
    void set_header(const char *_mode, const typename INS_GPS::float_t &_itow) const {
      mode = _mode;
      itow = _itow;
    }
  protected:
    void label2(std::ostream &out, const void *) const {}

    template <class BaseINS, template <class> class Filter>
    void label2(std::ostream &out, const Filtered_INS2<BaseINS, Filter> *fins) const {
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
    void label2(std::ostream &out, const INS_BiasEstimated<BaseINS> *ins) const {
      label2(out, (const BaseINS *)ins);
      out << ',' << "bias_accel(X)"   //Bias
          << ',' << "bias_accel(Y)"
          << ',' << "bias_accel(Z)"
          << ',' << "bias_gyro(X)"
          << ',' << "bias_gyro(Y)"
          << ',' << "bias_gyro(Z)" ;
    }

    template <class BaseFINS>
    void label2(std::ostream &out, const Filtered_INS_BiasEstimated<BaseFINS> *fins) const {
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

  public:
    /**
     * print label
     */
    void label(std::ostream &out = std::cout) const {
      out << "mode" << ','
          << "itow" << ',';
      NAVData<typename INS_GPS::float_t>::label(out);
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
      Vector3<float_sylph_t> &ba(const_cast<INS_BiasEstimated<BaseINS> *>(ins)->bias_accel());
      Vector3<float_sylph_t> &bg(const_cast<INS_BiasEstimated<BaseINS> *>(ins)->bias_gyro());
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
        const Matrix<float_sylph_t> &P(
            const_cast<Filtered_INS_BiasEstimated<BaseFINS> *>(fins)->getFilter().getP());
        for(int i(Filtered_INS_BiasEstimated<BaseFINS>::P_SIZE_WITHOUT_BIAS), j(0);
            j < Filtered_INS_BiasEstimated<BaseFINS>::P_SIZE_BIAS; ++i, ++j){
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
      out << mode << ','
          << itow << ',';
      NAVData<typename INS_GPS::float_t>::dump(out);
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

class StreamProcessor
    : public AbstractSylphideProcessor<float_sylph_t> {

  public:
    static const unsigned int buffer_size;

  protected:
    const Packet *_latest_packet;

    typedef AbstractSylphideProcessor<float_sylph_t> super_t;
    typedef A_Packet_Observer<float_sylph_t> A_Observer_t;
    typedef G_Packet_Observer<float_sylph_t> G_Observer_t;
    typedef M_Packet_Observer<float_sylph_t> M_Observer_t;

    /**
     * A page (ADC value)
     */
    struct AHandler : public A_Observer_t {
      StreamProcessor &outer;
      bool previous_seek_next;
      A_Packet packet_latest;
      StandardCalibration calibration;

      AHandler(StreamProcessor &invoker) : A_Observer_t(buffer_size),
          outer(invoker),
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
      AHandler &operator=(const AHandler &another){
        A_Observer_t::operator=(another);
        previous_seek_next = another.previous_seek_next;
        packet_latest = another.packet_latest;
        calibration = another.calibration;
        return *this;
      }
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

        outer._latest_packet = &packet_latest;
      }
    } a_handler;

    /**
     * G page (u-blox)
     */
    struct GHandler : public G_Observer_t  {
      StreamProcessor &outer;
      bool previous_seek_next;
      Vector3<float_sylph_t> lever_arm;
      G_Packet packet_latest;
      int itow_ms_0x0102, itow_ms_0x0112;
      unsigned int gps_status;
      int week_number;

      GHandler(StreamProcessor &invoker)
          : G_Observer_t(buffer_size),
          outer(invoker),
          lever_arm(),
          packet_latest(),
          itow_ms_0x0102(-1), itow_ms_0x0112(-1),
          gps_status(status_t::NO_FIX), week_number(0) {
        previous_seek_next = G_Observer_t::ready();
      }
      ~GHandler(){}
      GHandler &operator=(const GHandler &another){
        G_Observer_t::operator=(another);
        previous_seek_next = another.previous_seek_next;
        lever_arm = another.lever_arm;
        packet_latest = another.packet_latest;
        itow_ms_0x0102 = another.itow_ms_0x0102;
        itow_ms_0x0112 = another.itow_ms_0x0112;
        gps_status = another.gps_status;
        week_number = another.week_number;
        return *this;
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
            G_Observer_t::status_t status(observer.fetch_status());
            gps_status = status.fix_type;
            return;
          }
          case 0x06: { // NAV-SOL
            G_Observer_t::solution_t solution(observer.fetch_solution());
            if(solution.status_flags & G_Observer_t::solution_t::WN_VALID){
              week_number = solution.week;
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
          outer._latest_packet = &packet_latest;
        }
      }
      /**
       * Extract the following data.
       * {class, id} = {0x02, 0x10} : raw measurement (pseudorange, carrier, doppler)
       * {class, id} = {0x02, 0x11} : subframe
       * {class, id} = {0x02, 0x31} : ephemeris
       */
      void check_rxm(const G_Observer_t &observer, const G_Observer_t::packet_type_t &packet_type){
        switch(packet_type.mid){
          case 0x10: { // RXM-RAW
            return;
          }
          case 0x11: { // RXM-SFRB
            return;
          }
          case 0x31: { // RXM-EPH
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
        }
      }
    } g_handler;

    struct MHandler : public M_Observer_t {
      StreamProcessor &outer;
      bool previous_seek_next;
      M_Packet packet_latest;
      MHandler(StreamProcessor &invoker)
          : M_Observer_t(buffer_size),
          outer(invoker),
          packet_latest() {
        previous_seek_next = M_Observer_t::ready();
      }
      ~MHandler(){}
      MHandler &operator=(const MHandler &another){
        previous_seek_next = another.previous_seek_next;
        packet_latest = another.packet_latest;
        return *this;
      }
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

        outer._latest_packet = &packet_latest;
      }
    } m_handler;

    void move_latest_packet(const StreamProcessor &another){
      struct {
        const Packet &external;
        const Packet &internal;
      } targets[] = {
        {another.a_handler.packet_latest, a_handler.packet_latest},
        {another.g_handler.packet_latest, g_handler.packet_latest},
        {another.m_handler.packet_latest, m_handler.packet_latest},
      };
      for(int i(0); i < sizeof(targets) / sizeof(targets[0]); i++){
        if(another._latest_packet == &targets[i].external){
          _latest_packet = &targets[i].internal;
          break;
        }
      }
    }

  protected:
    int invoked;
    istream *_in;
    
  public:
    StreamProcessor()
        : super_t(), _latest_packet(NULL),
        _in(NULL), invoked(0),
        a_handler(*this),
        g_handler(*this),
        m_handler(*this) {

    }
    StreamProcessor(const StreamProcessor &another)
        : super_t(another), _latest_packet(NULL),
        _in(another._in), invoked(another.invoked),
        a_handler(*this),
        g_handler(*this),
        m_handler(*this) {
      a_handler = another.a_handler;
      g_handler = another.g_handler;
      m_handler = another.m_handler;
      move_latest_packet(another);
    }
    ~StreamProcessor(){}
    
    StreamProcessor &operator=(const StreamProcessor &another){
      if(this != &another){
        super_t::operator=(another);
        _in = another._in;
        invoked = another.invoked;
        a_handler = another.a_handler;
        g_handler = another.g_handler;
        m_handler = another.m_handler;
        move_latest_packet(another);
      }
      return *this;
    }

    const Packet *latest_packet() const {
      return _latest_packet;
    }

    const StandardCalibration &calibration() const{
      return a_handler.calibration;
    }

    void set_stream(istream *in){_in = in;}

    /**
     * Process stream in units of 1 page
     * 
     * @param in stream
     * @return (bool) true when success, otherwise false.
     */
    bool process_1page(){
      char buffer[SYLPHIDE_PAGE_SIZE];
      
      int read_count;
      _in->read(buffer, SYLPHIDE_PAGE_SIZE);
      read_count = static_cast<int>(_in->gcount());
      if(_in->fail() || (read_count == 0)){return false;}
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
      
      _latest_packet = NULL;

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
          if(_latest_packet){
            // Time check
            if(!options.is_time_before_end(_latest_packet->itow, g_handler.week_number)){
              return false;
            }
            if(!options.is_time_after_start(_latest_packet->itow, g_handler.week_number)){
              _latest_packet = NULL;
            }
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

    bool check_spec(const char *spec){
      const char *value;
      if(value = Options::get_value(spec, "calib_file", false)){ // calibration file
        cerr << "IMU Calibration file (" << value << ") reading..." << endl;
        istream &in(options.spec2istream(value));
        char buf[1024];
        while(!in.eof()){
          in.getline(buf, sizeof(buf));
          if(!buf[0]){continue;}
          if(!a_handler.calibration.check_spec(buf)){
            cerr << "unknown_calib_param! : " << buf << endl;
          }
        }
        return true;
      }

      if(value = Options::get_value(spec, "lever_arm", false)){ // Lever Arm
        double buf[3];
        if(std::sscanf(value, "%lf,%lf,%lf",
            &(buf[0]), &(buf[1]), &(buf[2])) != 3){
          cerr << "(error!) Lever arm option requires 3 arguments." << endl;
          exit(-1);
        }
        for(int i(0); i < sizeof(buf) / sizeof(buf[0]); ++i){
          g_handler.lever_arm[i] = buf[i];
        }
        g_handler.packet_latest.lever_arm = &(g_handler.lever_arm);
        std::cerr << "lever_arm: " << g_handler.lever_arm << std::endl;
        return true;
      }

      return false;
    }
};

const unsigned int StreamProcessor::buffer_size = SYLPHIDE_PAGE_SIZE * 64;

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

    Vector3<float_sylph_t> get_mag(const float_sylph_t &itow){
      if(recent_m.buf.size() < 2){
        return Vector3<float_sylph_t>(1, 0, 0); // heading is north
      }
      typename recent_m_t::buf_t::const_iterator
          it_a(nearest(recent_m.buf, itow, 2)),
          it_b(it_a + 1);
      float_sylph_t
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
  public:
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
        recent_m(0x10){
    }
  
  protected:
    template <class Base_INS_GPS>
    NAV::updated_items_t updated_items(
        const INS_GPS_Back_Propagate<Base_INS_GPS> *ins_gps) const {
      NAV::updated_items_t res;

      // When smoothing is activated
      switch(status){
        case MEASUREMENT_UPDATED: {
          float_sylph_t itow(recent_a.buf.back().itow);
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
              it->ins_gps.set_header("BP_MU", itow + it->elapsedT_from_last_correct);
              res.push_back(&(it->ins_gps));
            }else{
              if(!options.dump_update){continue;}
              it->ins_gps.set_header("BP_TU", itow + it->elapsedT_from_last_correct);
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
    void time_update(const A_Packet &a_packet, float_sylph_t deltaT){

      // Check interval from the last time update
#define INTERVAL_THRESHOLD 10
#define INTERVAL_FORCE_VALUE 0.01
      if((deltaT < 0) || (deltaT >= INTERVAL_THRESHOLD)){
        // Rewrite time stamp forcedly when discontinuity is too large.
        deltaT = INTERVAL_FORCE_VALUE;
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
        float_sylph_t deltaT(previous.interval(a_packet));
        time_update(a_packet, deltaT);
        nav.ins_gps->set_header("TU", a_packet.itow);
      }

      recent_a.push(a_packet);
    }

  protected:
    void time_update_after_initialization(const G_Packet &g_packet){
      typename recent_a_t::buf_t::const_reverse_iterator it_r(recent_a.buf.rbegin());
      for(; it_r != recent_a.buf.rend(); ++it_r){
        if(g_packet.interval(*it_r) <= 0){break;}
      }
      const Packet *packet(&g_packet);
      for(typename recent_a_t::buf_t::const_iterator it(it_r.base()); // it_r.base() position is not it_r position!!
          it != recent_a.buf.end(); ++it){
        time_update(*it, packet->interval(*it));
        packet = &(*it);
      }
      nav.ins_gps->set_header("MU", g_packet.itow);
    }

    void initialize(
        const float_sylph_t &itow,
        const float_sylph_t &latitude,
        const float_sylph_t &longitude,
        const float_sylph_t &height,
        const float_sylph_t &v_north,
        const float_sylph_t &v_east,
        const float_sylph_t &v_down){

      float_sylph_t
          yaw(deg2rad(options.initial_attitude.yaw_deg)),
          pitch(deg2rad(options.initial_attitude.pitch_deg)),
          roll(deg2rad(options.initial_attitude.roll_deg));

      while(options.initial_attitude.mode < options.initial_attitude.FULL_GIVEN){
        // Estimate initial attitude by using accelerometer and magnetic sensor (if available) under static assumption

        typedef Vector3<float_sylph_t> vec_t;

        // Normalization
        vec_t acc(0, 0, 0);
        for(typename recent_a_t::buf_t::iterator it(recent_a.buf.begin());
            it != recent_a.buf.end();
            ++it){
          acc += it->accel;
        }
        acc /= recent_a.buf.size();
        vec_t acc_reg(-acc / acc.abs());

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

    void time_update_before_measurement_update(const float_sylph_t &advanceT, void *){
      if(advanceT <= 0){return;}
      // Time update up to the GPS observation
      time_update(recent_a.buf.back(), advanceT);
    }

    template <class Base_INS_GPS>
    void time_update_before_measurement_update(const float_sylph_t &advanceT, INS_GPS_RealTime<Base_INS_GPS> *){
      return;
    }

  public:
    /**
     * Perform measurement update by using position and velocity obtained with GPS receiver.
     * 
     * @param g_packet observation data of GPS receiver
     */
    void measurement_update(const G_Packet &g_packet){

      if(g_packet.solution.sigma_2d >= options.gps_threshold.cont_acc_2d){ // When estimated accuracy is too big, skip.
        return;
      }
      if(status >= JUST_INITIALIZED){
        cerr << "MU : " << setprecision(10) << g_packet.itow << endl;
        
        // calculate GPS data timing;
        // negative(realtime mode, delayed), or slightly positive(other modes, because of already sorted)
        float_sylph_t gps_advance(recent_a.buf.back().interval(g_packet));
        time_update_before_measurement_update(gps_advance, nav.ins_gps);

        if(g_packet.lever_arm){ // When use lever arm effect.
          Vector3<float_sylph_t> omega_b2i_4n;
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
              && (pow(g_packet.solution.v_n, 2) + pow(g_packet.solution.v_e, 2)) < pow(options.yaw_correct_with_mag_when_speed_less_than_ms, 2)){
            nav.correct_yaw(nav.get_mag_delta_yaw(get_mag(g_packet.itow)));
          }
        }
        status = MEASUREMENT_UPDATED;
        nav.ins_gps->set_header("MU");
      }else if((recent_a.buf.size() >= min_a_packets_for_init)
          && (std::abs(recent_a.buf.front().itow - g_packet.itow) < (0.1 * recent_a.buf.size())) // time synchronization check
          && (g_packet.solution.sigma_2d <= options.gps_threshold.init_acc_2d)
          && (g_packet.solution.sigma_height <= options.gps_threshold.init_acc_v)){

        /*
         * Filter is activated when the estimate error in horizontal and vertical positions are under 20 and 10 meters, respectively.
         */
        initialize(
            g_packet.itow,
            g_packet.solution.latitude, g_packet.solution.longitude, g_packet.solution.height,
            g_packet.solution.v_n, g_packet.solution.v_e, g_packet.solution.v_d);
        time_update_after_initialization(g_packet);
      }
    }
};

void loop(){
  struct NAV_Manager {
    NAV *nav;
    NAV_Manager(){
      const StandardCalibration &calibration(processors.front().calibration());
      if(options.use_udkf){
        if(options.est_bias){
          nav = INS_GPS_NAV_Factory<ins_gps_bias_ekf_ud_t>::get_nav(calibration);
        }else{
          nav = INS_GPS_NAV_Factory<ins_gps_ekf_ud_t>::get_nav(calibration);
        }
      }else{
        if(options.est_bias){
          nav = INS_GPS_NAV_Factory<ins_gps_bias_ekf_t>::get_nav(calibration);
        }else{
          nav = INS_GPS_NAV_Factory<ins_gps_ekf_t>::get_nav(calibration);
        }
      }
    }
    ~NAV_Manager(){
      delete nav;
    }
    void dump(){
      const NAV::updated_items_t &updated(nav->updated_items());
      if(updated.empty()){return;}

      for(NAV::updated_items_t::const_iterator it(updated.begin());
          it != updated.end(); ++it){
        if(options.out_is_N_packet){
          char buf[SYLPHIDE_PAGE_SIZE];
          (*it)->encode_N0(buf);
          options.out().write(buf, sizeof(buf));
          return;
        }else{
          options.out() << (**it) << endl;
        }
      }

      options.out_debug() << nav->time_stamp() << ',';
      nav->inspect(options.out_debug());
      options.out_debug() << endl;
    }
  } nav_manager;
  
  if(!options.out_is_N_packet){
    nav_manager.nav->label(options.out());
    options.out() << endl;
  }

  // TODO multiple log stream will be support.
  StreamProcessor &proc(processors.front());
  if(options.ins_gps_sync_strategy == Options::INS_GPS_SYNC_REALTIME){
    while(proc.process_1page()){
      const Packet *latest(proc.latest_packet());
      if(!latest){continue;}
      latest->apply(*nav_manager.nav);
      nav_manager.dump();
    }
    return;
  }

  typedef deque<const Packet *> packet_pool_t;
  packet_pool_t packet_pool;

  while(true){
    bool alive(proc.process_1page());
    int packets;
    if(alive){
      const Packet *latest(proc.latest_packet());
      if(!latest){continue;}
      packet_pool.push_back(latest->clone());
      if(packet_pool.size() < 0x200){continue;}
      packets = packet_pool.size() / 2;
    }else{
      packets = packet_pool.size();
    }
    stable_sort(packet_pool.begin(), packet_pool.end(), Packet::compare);
    while(packets-- > 0){
      packet_pool_t::reference front(packet_pool.front());
      front->apply(*nav_manager.nav);
      nav_manager.dump();
      delete front;
      packet_pool.pop_front();
    }
    if(!alive){return;}
  }
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

  for(int arg_index(1); arg_index < argc; arg_index++){
    StreamProcessor stream_processor;

    for(; arg_index < argc; arg_index++){
      if(stream_processor.check_spec(argv[arg_index])){continue;}
      if(options.check_spec(argv[arg_index])){continue;}

      cerr << "Log file(" << processors.size() << "): ";
      istream &in(options.spec2istream(argv[arg_index]));
      stream_processor.set_stream(
          options.in_sylphide ? new SylphideIStream(in, SYLPHIDE_PAGE_SIZE) : &in);

      processors.push_back(stream_processor);
      cerr << stream_processor.calibration() << endl;
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

  loop();

  return 0;
}
