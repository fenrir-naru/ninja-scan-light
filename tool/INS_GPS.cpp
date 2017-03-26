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
    struct history_item_t {
      float_sylph_t deltaT;
      const NAVData<float_sylph_t> *nav;
    };
    typedef std::vector<history_item_t> history_t;
    virtual ~NAV(){}
  public:
    virtual history_t history() const {
      return history_t();
    }
    virtual void inspect(std::ostream &out) const {}
    virtual float_sylph_t &operator[](unsigned index) = 0;
    
    virtual NAV &update(const A_Packet &){
      return *this;
    }
    virtual NAV &update(const G_Packet &){
      return *this;
    }
    virtual NAV &update(const M_Packet &){
      return *this;
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
  float_sylph_t itow;

  /**
   * Get interval time between another one
   *
   * @param another one
   */
  float_sylph_t interval(const Packet &another) const {
    return another.itow - itow;
  }
};

/**
 * Inertial and temperature sensor data (ADC raw value)
 */
struct A_Packet : public Packet {
  Vector3<float_sylph_t> accel; ///< Acceleration
  Vector3<float_sylph_t> omega; ///< Angular speed
};

/**
 * GPS data
 */
struct G_Packet : public Packet {
  GPS_Solution<float_sylph_t> solution;

  operator GPS_Solution<float_sylph_t>() const {
    return solution;
  }
};

/**
 * Magnetic sensor data
 */
struct M_Packet : public Packet {
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
    void setup_filter2(const Calibration &calibration, void *){}

    template <class Calibration, class BaseINS, template <class> class Filter>
    void setup_filter2(
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
    void setup_filter2(
        const Calibration &calibration,
        Filtered_INS_BiasEstimated<BaseFINS> *fins) {

      setup_filter2(calibration, (BaseFINS *)fins);

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
    void setup_filter2(const Calibration &calibration, INS_GPS_Back_Propagate<Base_INS_GPS> *ins_gps){
      setup_filter2(calibration, (Base_INS_GPS *)ins_gps);
      ins_gps->setup_back_propagation(options.back_propagate_property);
    }

    template <class Calibration, class Base_INS_GPS>
    void setup_filter2(const Calibration &calibration, INS_GPS_RealTime<Base_INS_GPS> *ins_gps){
      setup_filter2(calibration, (Base_INS_GPS *)ins_gps);
      ins_gps->setup_realtime(options.realttime_property);
    }

    template <class Calibration, class Base_INS_GPS>
    void setup_filter2(const Calibration &calibration, INS_GPS_Debug<Base_INS_GPS> *ins_gps){
      setup_filter2(calibration, (Base_INS_GPS *)ins_gps);
      ins_gps->setup_debug(options.debug_property);
    }

    template <class Calibration>
    void setup_filter(const Calibration &calibration){
      setup_filter2(calibration, ins_gps);
    }
  public:
    template <class Calibration>
    INS_GPS_NAV(const Calibration &calibration)
        : NAV(),
        ins_gps(new INS_GPS()), helper(*this) {
      setup_filter(calibration);
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
    history_t history2(const void *) const {
      return NAV::history();
    }
    template <class INS_GPS_base>
    history_t history2(const INS_GPS_Back_Propagate<INS_GPS_base> *) const {
      typename NAV::history_t res;
      typedef typename INS_GPS_Back_Propagate<INS_GPS_base>::snapshots_t snapshots_t;
      const snapshots_t &snapshots(ins_gps->get_snapshots());
      for(typename snapshots_t::const_iterator it(snapshots.begin());
          it != snapshots.end();
          ++it){
        NAV::history_item_t item = {
            it->elapsedT_from_last_correct, &(it->ins_gps)};
        res.push_back(item);
      }
      return res;
    }
  public:
    history_t history() const {
      return history2(ins_gps);
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

  public:
    bool init_misc(const char *line){
      if(std::strlen(line) == 0){return true;}

      const char *value;
      bool checked(false);

      while(!checked){
        Matrix<float_sylph_t> P(ins_gps->getFilter().getP());
        if(value = Options::get_value2(line, "P")){
          set_matrix_full(P, value);
        }else if(value = Options::get_value2(line, "P_diag")){
          set_matrix_diagonal(P, value);
        }else if(value = Options::get_value2(line, "P_elm")){
          set_matrix_1element(P, value);
        }else{break;}
        ins_gps->getFilter().setP(P);
        checked = true;
        break;
      }

      while(!checked){
        Matrix<float_sylph_t> Q(ins_gps->getFilter().getQ());
        if(value = Options::get_value2(line, "Q")){
          set_matrix_full(Q, value);
        }else if(value = Options::get_value2(line, "Q_diag")){
          set_matrix_diagonal(Q, value);
        }else if(value = Options::get_value2(line, "Q_elm")){
          set_matrix_1element(Q, value);
        }else{break;}
        ins_gps->getFilter().setQ(Q);
        checked = true;
        break;
      }

      if(!checked){return false;}

      std::cerr << "Init (misc): " << line << std::endl;
      return true;
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
#undef MAKE_PROXY_FUNC
    
    float_sylph_t &operator[](unsigned index){return ins_gps->operator[](index);}
    
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
      helper.label(out);
    }

    void dump(std::ostream &out) const {
      helper.dump(out);
    }

    NAV &update(const A_Packet &packet){
      helper.time_update(packet);
      return *this;
    }
    NAV &update(const G_Packet &packet){
      helper.measurement_update(packet);
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
  public:
    INS_GPS_NAVData() : INS_GPS(){}
    INS_GPS_NAVData(const INS_GPS_NAVData<INS_GPS> &orig, const bool &deepcopy = false)
        : INS_GPS(orig, deepcopy){}
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
  protected:
    void label2(std::ostream &out, const void *) const {}

    template <class BaseINS, template <class> class Filter>
    void label2(std::ostream &out, const Filtered_INS2<BaseINS, Filter> *fins) const {
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

    template <class BaseFINS>
    void label2(std::ostream &out, const Filtered_INS_BiasEstimated<BaseFINS> *fins) const {
      label2(out, (const BaseFINS *)fins);
      out << ',' << "bias_accel(X)"   //Bias
          << ',' << "bias_accel(Y)"
          << ',' << "bias_accel(Z)"
          << ',' << "bias_gyro(X)"
          << ',' << "bias_gyro(Y)"
          << ',' << "bias_gyro(Z)" ;
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
      NAVData<typename INS_GPS::float_t>::label(out);
      label2(out, this);
    }
  
  protected:
    void dump2(std::ostream &out, const void *) const {}

    template <class BaseINS, template <class> class Filter>
    void dump2(std::ostream &out, const Filtered_INS2<BaseINS, Filter> *fins) const {
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

    template <class BaseFINS>
    void dump2(
        std::ostream &out, const Filtered_INS_BiasEstimated<BaseFINS> *fins) const {
      dump2(out, (const BaseFINS *)fins);
      Vector3<float_sylph_t> &ba(const_cast<Filtered_INS_BiasEstimated<BaseFINS> *>(fins)->bias_accel());
      Vector3<float_sylph_t> &bg(const_cast<Filtered_INS_BiasEstimated<BaseFINS> *>(fins)->bias_gyro());
      out << ',' << ba.getX()      // Bias
          << ',' << ba.getY()
          << ',' << ba.getZ()
          << ',' << bg.getX()
          << ',' << bg.getY()
          << ',' << bg.getZ() ;
      if(options.dump_stddev){
        Matrix<float_sylph_t> &P(
            const_cast<Matrix<float_sylph_t> &>(
              const_cast<Filtered_INS_BiasEstimated<BaseFINS> *>(fins)->getFilter().getP()));
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
  };
  calibration_info_t<3> accel, gyro;

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
#define make_proc1(name, sensor, item) \
if(value = Options::get_value2(line, TO_STRING(name))){ \
  std::cerr << TO_STRING(name) << ":"; \
  char *spec(const_cast<char *>(value)); \
  for(int i(0); i < 3; i++){ \
    sensor.item[i] = std::strtod(spec, &spec); \
    std::cerr << " " << sensor.item[i]; \
  } \
  std::cerr << std::endl; \
  return true; \
}
#define make_proc2(name, sensor, item) \
if(value = Options::get_value2(line, TO_STRING(name))){ \
  std::cerr << TO_STRING(name) << ": {"; \
  char *spec(const_cast<char *>(value)); \
  for(int i(0); i < 3; i++){ \
    for(int j(0); j < 3; j++){ \
      sensor.item[i][j] = std::strtod(spec, &spec); \
    } \
    std::cerr \
        << std::endl << "{" \
        << sensor.item[i][0] << ", " \
        << sensor.item[i][1] << ", " \
        << sensor.item[i][2] << "}"; \
  } \
  std::cerr << "}" << std::endl; \
  return true; \
}
    make_proc1(acc_bias_tc, accel, bias_tc);
    make_proc1(acc_bias, accel, bias_base);
    make_proc1(acc_sf, accel, sf);
    make_proc2(acc_mis, accel, alignment);
    make_proc1(gyro_bias_tc, gyro, bias_tc);
    make_proc1(gyro_bias, gyro, bias_base);
    make_proc1(gyro_sf, gyro, sf);
    make_proc2(gyro_mis, gyro, alignment);
    make_proc1(sigma_accel, accel, sigma);
    make_proc1(sigma_gyro, gyro, sigma);
#undef make_proc1
#undef make_proc2
#undef TO_STRING

    return false;
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
    typedef AbstractSylphideProcessor<float_sylph_t> super_t;
    typedef A_Packet_Observer<float_sylph_t> A_Observer_t;
    typedef G_Packet_Observer<float_sylph_t> G_Observer_t;
    typedef M_Packet_Observer<float_sylph_t> M_Observer_t;

    /**
     * A page (ADC value)
     */
    struct AHandler : public A_Observer_t {
      bool previous_seek_next;
      deque<A_Packet> recent_packets;
      bool packet_updated;
      StandardCalibration calibration;

      AHandler() : A_Observer_t(buffer_size),
          recent_packets(), packet_updated(false),
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

        A_Packet packet;
        packet.itow = observer.fetch_ITOW();

        int ch[9];
        A_Observer_t::values_t values(observer.fetch_values());
        for(int i = 0; i < 8; i++){
          ch[i] = values.values[i];
        }
        ch[8] = values.temperature;
        packet.accel = calibration.raw2accel(ch);
        packet.omega = calibration.raw2omega(ch);

        while(options.reduce_1pps_sync_error){
          if(recent_packets.empty()){break;}
          float_sylph_t delta_t(packet.itow - recent_packets.back().itow);
          if((delta_t < 1) || (delta_t >= 2)){break;}
          packet.itow -= 1;
          break;
        }

        while(recent_packets.size() >= 128){
          recent_packets.pop_front();
        }
        recent_packets.push_back(packet);
        packet_updated = true;
      }
    } a_handler;

    /**
     * G page (u-blox)
     */
    struct GHandler : public G_Observer_t  {
      bool previous_seek_next;
      G_Packet packet_latest;
      bool packet_updated;
      int itow_ms_0x0102, itow_ms_0x0112;
      unsigned int gps_status;
      int week_number;

      GHandler()
          : G_Observer_t(buffer_size),
          packet_latest(), packet_updated(false),
          itow_ms_0x0102(-1), itow_ms_0x0112(-1),
          gps_status(status_t::NO_FIX), week_number(0) {
        previous_seek_next = G_Observer_t::ready();
      }
      ~GHandler(){}

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
          packet_updated = true;
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
      bool previous_seek_next;
      deque<M_Packet> recent_packets;
      MHandler() : M_Observer_t(buffer_size), recent_packets() {
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

        // TODO: magnetic sensor axes must correspond to ones of accelerometer and gyro.
        Vector3<float_sylph_t> mag(values.x[3], values.y[3], values.z[3]);
        M_Packet m_packet;
        m_packet.itow = observer.fetch_ITOW();
        m_packet.mag = mag;

        while(options.reduce_1pps_sync_error){
          if(recent_packets.empty()){break;}
          float_sylph_t delta_t(m_packet.itow - recent_packets.back().itow);
          if((delta_t < 1) || (delta_t >= 2)){break;}
          m_packet.itow -= 1;
          break;
        }

        while(recent_packets.size() > 0x40){
          recent_packets.pop_front();
        }
        recent_packets.push_back(m_packet);
      }
    } m_handler;

  protected:
    int invoked;
    istream *_in;
    Vector3<float_sylph_t> *ptr_lever_arm;
    
  public:
    deque<A_Packet> &a_packets;
    bool &a_packet_updated;
    G_Packet &g_packet;
    bool &g_packet_updated;
    int &g_packet_wn;
    deque<M_Packet> &m_packets;

    StreamProcessor()
        : super_t(), _in(NULL), invoked(0),
        ptr_lever_arm(NULL),
        a_handler(),
        a_packets(a_handler.recent_packets), a_packet_updated(a_handler.packet_updated),
        g_handler(),
        g_packet(g_handler.packet_latest), g_packet_updated(g_handler.packet_updated), g_packet_wn(g_handler.week_number),
        m_handler(),
        m_packets(m_handler.recent_packets) {

    }
    ~StreamProcessor(){
      delete ptr_lever_arm;
    }
    
    const StandardCalibration &calibration() const{
      return a_handler.calibration;
    }

    const Vector3<float_sylph_t> *lever_arm() const {
      return ptr_lever_arm;
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

    Vector3<float_sylph_t> get_mag() {
      return m_packets.empty()
          ? Vector3<float_sylph_t>(1, 0, 0) // heading is north
          : m_packets.back().mag;
    }

    Vector3<float_sylph_t> get_mag(const float_sylph_t &itow){
      if(m_packets.size() < 2){
        return Vector3<float_sylph_t>(1, 0, 0); // heading is north
      }
      deque<M_Packet>::iterator
          previous_it(m_packets.begin()),
          next_it(m_packets.begin() + 1);
      for(int i(distance(previous_it, m_packets.end()));
          i > 2;
          i--, previous_it++, next_it++){
        if(next_it->itow >= itow){break;}
      }
      float_sylph_t
          weight_previous((next_it->itow - itow) / (next_it->itow - previous_it->itow)),
          weight_next(1. - weight_previous);
      /* Reduce excessive extrapolation.
       * The extrapolation is required, because M page combines several samples which are sometimes obtained late.
       * The threshold is +/- 2 steps.
       */
      if(weight_previous > 3){
        weight_previous = 1;
        weight_next = 0;
      }else if(weight_next > 3){
        weight_next = 1;
        weight_previous = 0;
      }
      return (previous_it->mag * weight_previous) + (next_it->mag * weight_next);
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
        if(!ptr_lever_arm){
          ptr_lever_arm = new Vector3<float_sylph_t>(buf[0], buf[1], buf[2]);
        }else{
          for(int i(0); i < sizeof(buf) / sizeof(buf[0]); ++i){
            (*ptr_lever_arm)[i] = buf[i];
          }
        }
        std::cerr << "lever_arm: " << *ptr_lever_arm << std::endl;
        return true;
      }

      return false;
    }
} *current_processor;

const unsigned int StreamProcessor::buffer_size = SYLPHIDE_PAGE_SIZE * 64;

typedef vector<StreamProcessor *> processors_t;
processors_t processors;

template <class INS_GPS>
class INS_GPS_NAV<INS_GPS>::Helper {
  protected:
    enum {
      UNINITIALIZED,
      JUST_INITIALIZED,
      TIME_UPDATED,
      MEASUREMENT_UPDATED,
    } status;
    INS_GPS_NAV<INS_GPS> &nav;
    const int min_a_packets_for_init; // must be greater than 0
    deque<A_Packet> recent_a_packets;
    const unsigned int max_recent_a_packets;

  // Used for compensation of lever arm effect
  protected:
    Vector3<float_sylph_t> gyro_storage[16];
    int gyro_index;
    bool gyro_init;

  public:
    Helper(INS_GPS_NAV<INS_GPS> &_nav)
        : status(UNINITIALIZED), nav(_nav), gyro_index(0), gyro_init(false),
        min_a_packets_for_init(options.initial_attitude.mode == options.initial_attitude.FULL_GIVEN ? 1 : 0x10),
        recent_a_packets(), max_recent_a_packets(max(min_a_packets_for_init, 0x100)){
    }
  
  public:
    void label(std::ostream &out) const {
      if(options.out_is_N_packet){return;}
      out << "mode"
          << ',' << "itow";
      nav.ins_gps->label(out);
      out << endl;
    }
  
  protected:
    /**
     * Dump state
     * 
     * @param label operation mode string
     * @param itow current time
     * @param target NAV to be outputted
     */
    static void dump(
        ostream &out,
        const char *label, const float_sylph_t &itow,
        const NAVData<float_sylph_t> &target){
      
      if(options.out_is_N_packet){
        char buf[SYLPHIDE_PAGE_SIZE];
        target.encode_N0(itow, buf);
        out.write(buf, sizeof(buf));
        return;
      }

      out << label
          << ',' << itow
          << target << endl;
    }

    template <class Base_INS_GPS>
    void dump(ostream &out, const float_sylph_t &itow,
        const INS_GPS_Back_Propagate<Base_INS_GPS> *ins_gps) const {

      // When smoothing is activated
      switch(status){
        case MEASUREMENT_UPDATED: {
          typedef typename INS_GPS_Back_Propagate<Base_INS_GPS>::snapshots_t snapshots_t;
          const snapshots_t &snapshots(ins_gps->get_snapshots());
          int index(0);
          for(typename snapshots_t::const_iterator it(snapshots.begin());
              it != snapshots.end();
              ++it, index++){
            if(it->elapsedT_from_last_correct >= options.back_propagate_property.back_propagate_depth){
              break;
            }

            const char *label;
            if(index == 0){
              if(!options.dump_correct){continue;}
              label = "BP_MU";
            }else{
              if(!options.dump_update){continue;}
              label = "BP_TU";
            }
            dump(out, label, itow + it->elapsedT_from_last_correct, it->ins_gps);
          }
          break;
        }
        default:
          return;
      }
    }

    void dump(ostream &out, const float_sylph_t &itow, void *) const {

      switch(status){
        case TIME_UPDATED:
          if(!options.dump_update){return;}
          dump(out, "TU", itow, *(nav.ins_gps));
          break;
        case MEASUREMENT_UPDATED:
          if(!options.dump_correct){return;}
          dump(out, "MU", itow, *(nav.ins_gps));
          break;
        default:
          return;
      }
    }

  public:
    void dump(std::ostream &out) const {
      if(status == UNINITIALIZED){return;}

      const float_sylph_t &itow(recent_a_packets.back().itow);
      dump(out, itow, nav.ins_gps);

      options.out_debug() << itow << ',';
      nav.inspect(options.out_debug());
      options.out_debug() << endl;
    }

    /**
     * Perform time update by using acceleration and angular speed obtained with accelerometer and gyro.
     * 
     * @param a_packet raw values of ADC
     */
    void time_update(const A_Packet &a_packet){

      if(status >= JUST_INITIALIZED){
        const A_Packet &previous(recent_a_packets.back());
      
        { // for LAE
          gyro_storage[gyro_index++] = a_packet.omega;
          if(gyro_index == (sizeof(gyro_storage) / sizeof(gyro_storage[0]))){
            gyro_index = 0;
            if(!gyro_init) gyro_init = true;
          }
        }

        // Check interval from the last time update
        float_sylph_t interval(previous.interval(a_packet));
#define INTERVAL_THRESHOLD 10
#define INTERVAL_FORCE_VALUE 0.01
        if((interval < 0) || (interval >= INTERVAL_THRESHOLD)){
          // Rewrite time stamp forcedly when discontinuity is too large.
          interval = INTERVAL_FORCE_VALUE;
        }

        nav.update(a_packet.accel, a_packet.omega, interval);
        status = TIME_UPDATED;
      }

      recent_a_packets.push_back(a_packet);
      if(recent_a_packets.size() > max_recent_a_packets){
        recent_a_packets.pop_front();
      }
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
        for(deque<A_Packet>::iterator it(recent_a_packets.begin());
            it != recent_a_packets.end();
            ++it){
          acc += it->accel;
        }
        acc /= recent_a_packets.size();
        vec_t acc_reg(-acc / acc.abs());

        // Estimate roll angle
        roll = atan2(acc_reg[1], acc_reg[2]);
        if(options.initial_attitude.mode >= options.initial_attitude.YAW_PITCH){break;}

        // Estimate pitch angle
        pitch = -asin(acc_reg[0]);
        if(options.initial_attitude.mode >= options.initial_attitude.YAW_ONLY){break;}

        // Estimate yaw when magnetic sensor is available
        if(!current_processor->m_packets.empty()){
          yaw = nav.get_mag_yaw(current_processor->get_mag(itow), pitch, roll,
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
        float_sylph_t gps_advance(recent_a_packets.back().interval(g_packet));

        if(gyro_init && (current_processor->lever_arm())){ // When use lever arm effect.
          Vector3<float_sylph_t> omega_b2i_4n;
          for(int i(0); i < (sizeof(gyro_storage) / sizeof(gyro_storage[0])); i++){
            omega_b2i_4n += gyro_storage[i];
          }
          omega_b2i_4n /= (sizeof(gyro_storage) / sizeof(gyro_storage[0]));
          nav.correct(
              g_packet,
              *(current_processor->lever_arm()),
              omega_b2i_4n,
              gps_advance);
        }else{ // When do not use lever arm effect.
          nav.correct(
              g_packet,
              gps_advance);
        }
        if(!current_processor->m_packets.empty()){ // When magnetic sensor is activated, try to perform yaw compensation
          if((options.yaw_correct_with_mag_when_speed_less_than_ms > 0)
              && (pow(g_packet.solution.v_n, 2) + pow(g_packet.solution.v_e, 2)) < pow(options.yaw_correct_with_mag_when_speed_less_than_ms, 2)){
            nav.correct_yaw(nav.get_mag_delta_yaw(current_processor->get_mag(g_packet.itow)));
          }
        }
        status = MEASUREMENT_UPDATED;
      }else if((current_processor == processors.front())
          && (recent_a_packets.size() >= min_a_packets_for_init)
          && (std::abs(recent_a_packets.front().itow - g_packet.itow) < (0.1 * recent_a_packets.size())) // time synchronization check
          && (g_packet.solution.sigma_2d <= options.gps_threshold.init_acc_2d)
          && (g_packet.solution.sigma_height <= options.gps_threshold.init_acc_v)){

        /*
         * Filter is activated when the estimate error in horizontal and vertical positions are under 20 and 10 meters, respectively.
         */
        initialize(
            g_packet.itow,
            g_packet.solution.latitude, g_packet.solution.longitude, g_packet.solution.height,
            g_packet.solution.v_n, g_packet.solution.v_e, g_packet.solution.v_d);
      }
    }
};

void loop(){
  struct NAV_Manager {
    NAV *nav;
    NAV_Manager(){
      const StandardCalibration &calibration(processors.front()->calibration());
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
  } nav_manager;
  
  nav_manager.nav->label(options.out());

  if(options.ins_gps_sync_strategy == Options::INS_GPS_SYNC_REALTIME){
    // Realtime mode supports only one stream.
    current_processor = processors.front();
    StreamProcessor &proc(*current_processor);
    bool started(false);
    while(proc.process_1page()){
      if(proc.a_packet_updated){
        proc.a_packet_updated = false;
        A_Packet &packet(proc.a_packets.back());
        nav_manager.nav->update(packet);
        options.out() << *(nav_manager.nav);
      }else if(current_processor->g_packet_updated){
        proc.g_packet_updated = false;
        G_Packet &packet(proc.g_packet);
        if(!started){
          if(!options.is_time_after_start(packet.itow, proc.g_packet_wn)){ // Time check
            continue;
          }else{
            started = true;
          }
        }
        nav_manager.nav->update(packet);
        options.out() << *(nav_manager.nav);
        if(!options.is_time_before_end(packet.itow, proc.g_packet_wn)){
          break;
        }
      }
    }
    return;
  }

  while(true){
    for(processors_t::iterator it(processors.begin()); it != processors.end(); ++it){
      while(!((*it)->g_packet_updated)){ // find new GPS data
        if((*it)->process_1page()){continue;}
        // when stream read failed
        if(it == processors.begin()){ // no more GPS data in front processor
          return;
        }else{
          break;
        }
      }
    }

    // Check and sort G packets in order of observation time
    typedef vector<StreamProcessor *> mu_queue_t;
    mu_queue_t mu_queue;
    
    mu_queue.push_back(processors.front());
    bool require_search_again(false);
    for(processors_t::iterator it(processors.begin() + 1); it != processors.end(); ++it){
      // check GPS data synchronization between multiple streams
      if(!((*it)->g_packet_updated)){continue;}
      float_sylph_t itow((*it)->g_packet.itow);
      float_sylph_t interval(
          itow - processors.front()->g_packet.itow);
      
      if(std::abs(interval) <= 0.2){ // When the observation times are near, use the data
        // mu_queue is chronologically sorted
        mu_queue_t::iterator it2(mu_queue.begin());
        while(it2 != mu_queue.end()){
          if((*it2)->g_packet.itow > itow){break;}
          ++it2;
        }
        mu_queue.insert(it2, *it);
      }else if(interval < 0){ // When old data, try to search again
        (*it)->g_packet_updated = false;
        require_search_again = true;
        break;
      }
    }
    
    if(require_search_again){continue;} // find another GPS data
    
    float_sylph_t latest_measurement_update_itow(0);
    int latest_measurement_update_gpswn(0);

    for(mu_queue_t::iterator it_mu(mu_queue.begin());
        it_mu != mu_queue.end();
        ++it_mu){
      
      current_processor = *it_mu;
      G_Packet &g_packet(current_processor->g_packet);
      current_processor->g_packet_updated = false;

      if(!options.is_time_after_start(g_packet.itow, current_processor->g_packet_wn)){ // Time check
        continue;
      }
      
      deque<A_Packet> &a_packets(processors.front()->a_packets);
      deque<A_Packet>::iterator it_tu(a_packets.begin()), it_tu_end(a_packets.end());
      const bool a_packet_deque_has_item(it_tu != it_tu_end);
    
      // Time update up to the last sample before GPS observation
      for(; 
          (it_tu != it_tu_end) && (it_tu->itow < g_packet.itow);
          ++it_tu){
        nav_manager.nav->update(*it_tu);
        options.out() << *(nav_manager.nav);
      }
      
      // Time update up to the GPS observation
      if(a_packet_deque_has_item){
        A_Packet interpolation((it_tu != it_tu_end) ? *it_tu : *(it_tu - 1));
        interpolation.itow = g_packet.itow;
        nav_manager.nav->update(interpolation);
      }
      
      a_packets.erase(a_packets.begin(), it_tu);
      
      // Measurement update
      nav_manager.nav->update(g_packet);
      options.out() << *(nav_manager.nav);

      latest_measurement_update_itow = g_packet.itow;
      latest_measurement_update_gpswn = current_processor->g_packet_wn;
    }
    
    if(!options.is_time_before_end(latest_measurement_update_itow, latest_measurement_update_gpswn)){
      break;
    }
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

  StreamProcessor *stream_processor(new StreamProcessor());

  for(int arg_index(1); arg_index < argc; arg_index++){

    if(stream_processor->check_spec(argv[arg_index])){continue;}

    if(options.check_spec(argv[arg_index])){continue;}
    
    if(!processors.empty()){
      cerr << "(error!) Unknown option or too many log." << endl;
      exit(-1);
    }
    
    cerr << "Log file: ";
    istream &in(options.spec2istream(argv[arg_index]));
    stream_processor->set_stream(
        options.in_sylphide ? new SylphideIStream(in, SYLPHIDE_PAGE_SIZE) : &in);

    processors.push_back(stream_processor);
  }

  if(processors.empty()){
    cerr << "(error!) No log file." << endl;
    exit(-1);
  }

  if(options.out_sylphide){
    options._out = new SylphideOStream(options.out(), SYLPHIDE_PAGE_SIZE);
  }else{
    options.out() << setprecision(10);
  }
  options.out_debug() << setprecision(16);

  loop();
  
  for(processors_t::iterator it(processors.begin());
      it != processors.end();
      ++it){
    delete *it;
  }

  return 0;
}
