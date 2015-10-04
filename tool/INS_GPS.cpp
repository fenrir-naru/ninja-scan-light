/**
 * @file INS/GPS post-processor for NinjaScan
 *
 */

/*
 * Copyright (c) 2015, M.Naruoka (fenrir)
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

// Comment-In when QNAN DEBUG
//#include <float.h>
//unsigned int fp_control_state = _controlfp(_EM_INEXACT, _MCW_EM);

#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <exception>

#include <cstdio>
#include <cmath>
#include <cstring>

#include <vector>
#include <utility>
#include <deque>

#define IS_LITTLE_ENDIAN 1
#include "SylphideStream.h"
#include "SylphideProcessor.h"

typedef double float_sylph_t;
typedef SylphideProcessor<float_sylph_t> Processor_t;
typedef Processor_t::A_Observer_t A_Observer_t;
typedef Processor_t::G_Observer_t G_Observer_t;
typedef Processor_t::M_Observer_t M_Observer_t;

#include "param/matrix.h"
#include "param/vector3.h"
#include "param/quaternion.h"
#include "param/complex.h"
VECTOR3_NO_FLY_WEIGHT(float_sylph_t);
QUATERNION_NO_FLY_WEIGHT(float_sylph_t);

#include "navigation/INS_GPS2.h"
#include "navigation/INS_GPS_BE.h"

#include "navigation/MagneticField.h"

#include "analyze_common.h"

struct Options : public GlobalOptions<float_sylph_t> {
  bool back_propagate;  //< true when use back_propagation, that is, smoothing.
  
  /**
   * Number of snapshots to be back-propagated.
   * Zero means the last snapshot to be corrected, and negative values mean deeper.
   */
  float_sylph_t back_propagate_depth;
  
  bool gps_fake_lock; //< true when gps dummy date is used.

  Options()
      : GlobalOptions(),
      back_propagate(false), back_propagate_depth(0),
      gps_fake_lock(false) {}
  ~Options(){}
  
  /**
   * Check spec
   * 
   * @param spec command
   * @return (bool) true when consumed, otherwise false
   */
  bool check_spec(const char *spec){

#define CHECK_OPTION(name, operation, disp) { \
  const char *value(get_value(spec, #name)); \
  if(value){ \
    {operation;} \
    std::cerr << #name << ": " << disp << std::endl; \
    return true; \
  } \
}
    CHECK_OPTION(back_propagate,
        back_propagate = is_true(value),
        (back_propagate ? "on" : "off"));
    CHECK_OPTION(bp_depth,
        back_propagate_depth = atof(value),
        back_propagate_depth);
    CHECK_OPTION(fake_lock,
        gps_fake_lock = is_true(value),
        (gps_fake_lock ? "on" : "off"));
#undef CHECK_OPTION
    
    return GlobalOptions::check_spec(spec);
  }
} options;

class NAV : public NAVData {
  public:
    struct back_propagated_item_t {
      float_sylph_t deltaT_from_last_correct;
      NAVData &nav;
      back_propagated_item_t(
          const float_sylph_t &_deltaT,
          NAVData &_nav) : deltaT_from_last_correct(_deltaT), nav(_nav) {}
      ~back_propagated_item_t(){}
      back_propagated_item_t &operator=(const back_propagated_item_t &another){
        deltaT_from_last_correct = another.deltaT_from_last_correct;
        nav = another.nav;
        return *this;
      }
    };
    typedef std::vector<
        back_propagated_item_t> back_propagated_list_t;
    virtual ~NAV(){}
  public:
    virtual back_propagated_list_t back_propagated() = 0;
    virtual void init(
        const float_sylph_t &latitude, 
        const float_sylph_t &longitude, 
        const float_sylph_t &height,
        const float_sylph_t &v_north, 
        const float_sylph_t &v_east, 
        const float_sylph_t &v_down,
        const float_sylph_t &yaw, 
        const float_sylph_t &pitch, 
        const float_sylph_t &roll) = 0;
    virtual float_sylph_t &operator[](unsigned index) = 0;
    virtual NAV &update(
        const Vector3<float_sylph_t> &accel, 
        const Vector3<float_sylph_t> &gyro, 
        const float_sylph_t &deltaT) = 0;
    
    virtual NAV &correct(const GPS_UBLOX_3D<float_sylph_t> &gps) = 0;
    virtual NAV &correct(
        const GPS_UBLOX_3D<float_sylph_t> &gps, 
        const Vector3<float_sylph_t> &lever_arm_b,
        const Vector3<float_sylph_t> &omega_b2i_4b){
      return correct(gps);
    }
    virtual NAV &correct_yaw(const float_sylph_t &delta_yaw){
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
struct A_Packet : Packet {
  int ch[9]; ///< ADC raw values
};

/**
 * GPS data
 */
struct G_Packet : Packet {
  float_sylph_t llh[3];
  float_sylph_t acc_2d, acc_v;
  float_sylph_t vel_ned[3];
  float_sylph_t acc_vel;

  /**
   * convert to a structured data required by INS/GPS routine
   */
  GPS_UBLOX_3D<float_sylph_t> convert() const {
    GPS_UBLOX_3D<float_sylph_t> packet;
    {
      packet.v_n = vel_ned[0];
      packet.v_e = vel_ned[1];
      packet.v_d = vel_ned[2];
      packet.sigma_vel = acc_vel;
      packet.latitude = deg2rad(llh[0]);
      packet.longitude = deg2rad(llh[1]);
      packet.height = llh[2];
      packet.sigma_2d = acc_2d;
      packet.sigma_height = acc_v;
    }
    return packet;
  }
};

/**
 * Magnetic sensor data
 */
struct M_Packet : Packet {
  Vector3<float_sylph_t> mag;
};

template <class INS_GPS>
class INS_GPS_NAV : public NAV {
  protected:
    class INS_GPS_back_propagate;
    struct snapshot_content_t {
      INS_GPS_back_propagate *ins_gps;
      Matrix<float_sylph_t> Phi;
      Matrix<float_sylph_t> GQGt;
      float_sylph_t deltaT_from_last_correct;
      snapshot_content_t(
          INS_GPS_back_propagate *_ins_gps,
          const Matrix<float_sylph_t> &_Phi,
          const Matrix<float_sylph_t> &_GQGt, 
          const float_sylph_t &_deltaT)
          : ins_gps(_ins_gps), Phi(_Phi), GQGt(_GQGt), 
          deltaT_from_last_correct(_deltaT){
      }
      snapshot_content_t &operator=(const snapshot_content_t &another){
        ins_gps = another.ins_gps;
        Phi = another.Phi;
        GQGt = another.GQGt;
        deltaT_from_last_correct = another.deltaT_from_last_correct;
        return *this;
      }
    };
    typedef std::vector<snapshot_content_t> snapshots_t;
    snapshots_t snapshots;
    class INS_GPS_back_propagate : public INS_GPS, public NAVData {
      protected:
        snapshots_t &snapshots;
      public:
        INS_GPS_back_propagate(snapshots_t &_snapshots) 
            : INS_GPS(), snapshots(_snapshots) {}
        INS_GPS_back_propagate(
            const INS_GPS_back_propagate &orig, 
            const bool deepcopy = false)
            : INS_GPS(orig, deepcopy), snapshots(orig.snapshots){}
        INS_GPS_back_propagate &operator=(const INS_GPS_back_propagate &another){
          snapshots = another.snapshots;
          INS_GPS::operator=(another);
          return *this;
        }
#define MAKE_COMMIT_FUNC(fname) \
float_sylph_t fname() const {return INS_GPS::fname();}
        MAKE_COMMIT_FUNC(longitude);
        MAKE_COMMIT_FUNC(latitude);
        MAKE_COMMIT_FUNC(height);
        MAKE_COMMIT_FUNC(v_north);
        MAKE_COMMIT_FUNC(v_east);
        MAKE_COMMIT_FUNC(v_down);
        MAKE_COMMIT_FUNC(heading);
        MAKE_COMMIT_FUNC(euler_phi);
        MAKE_COMMIT_FUNC(euler_theta);
        MAKE_COMMIT_FUNC(euler_psi);
        MAKE_COMMIT_FUNC(azimuth);
#undef MAKE_COMMIT_FUNC
        
      protected:
        /**
         * Call-back function for time update
         * 
         * @param A matrix A
         * @param B matrix B
         * @patam deltaT interval time
         */
        void before_update_INS(
            const Matrix<float_sylph_t> &A, const Matrix<float_sylph_t> &B, 
            const float_sylph_t &deltaT){
          Matrix<float_sylph_t> Phi(A * deltaT);
          for(unsigned i(0); i < A.rows(); i++){Phi(i, i) += 1;}
          Matrix<float_sylph_t> Gamma(B * deltaT);
          
          float_sylph_t deltaT_from_last_correct(deltaT);
          if(!snapshots.empty()){
            deltaT_from_last_correct += snapshots.back().deltaT_from_last_correct;
          }
          
          snapshots.push_back(
              snapshot_content_t(new INS_GPS_back_propagate(*this, true), 
                  Phi, Gamma * INS_GPS::getFilter().getQ() * Gamma.transpose(),
                  deltaT_from_last_correct));
        }
    
        /**
         * Call-back function for measurement (correct) update
         * 
         * @param H matrix H
         * @param R matrix R
         * @patam K matrix K (Kalman gain)
         * @param v =(z - H x)
         * @param x_hat values to be corrected
         */
        void before_correct_INS(
            const Matrix<float_sylph_t> &H,
            const Matrix<float_sylph_t> &R,
            const Matrix<float_sylph_t> &K,
            const Matrix<float_sylph_t> &v,
            Matrix<float_sylph_t> &x_hat){
          if(!snapshots.empty()){
            
            // This routine is invoked by measurement update function called correct().
            // In addition, this routine is reduced to be activated once with the following if-statement.
            float_sylph_t mod_deltaT(snapshots.back().deltaT_from_last_correct);
            if(mod_deltaT > 0){
              
              // The latest is the first
              for(typename snapshots_t::reverse_iterator it(snapshots.rbegin());
                  it != snapshots.rend();
                  ++it){
                // This statement controls depth of back propagation.
                if(it->deltaT_from_last_correct < options.back_propagate_depth){
                  if(mod_deltaT > 0.1){ // Skip only when sufficient amount of snapshots are existed.
                    for(typename snapshots_t::reverse_iterator it2(it);
                        it2 != snapshots.rend();
                        ++it2){
                      delete it2->ins_gps;
                    }
                    snapshots.erase(snapshots.begin(), it.base());
                    //cerr << "[erase]" << endl;
                    if(snapshots.empty()){return;}
                  }
                  break;
                }
                // Positive value stands for states to which applied back-propagation have not been applied
                it->deltaT_from_last_correct -= mod_deltaT;
              }
            }
            
            snapshot_content_t previous(snapshots.back());
            snapshots.pop_back();
            
            // Perform back-propagation
            Matrix<float_sylph_t> H_dash(H * previous.Phi);
            Matrix<float_sylph_t> R_dash(R + H * previous.GQGt * H.transpose());
            previous.ins_gps->correct(H_dash, v, R_dash);
            
            snapshots.push_back(previous);
          }
        }
    }; 
    INS_GPS *_nav, &nav;
  public:
    INS_GPS_NAV() 
        : NAV(), snapshots(), 
        _nav(options.back_propagate ? new INS_GPS_back_propagate(snapshots) : new INS_GPS()),
        nav(*_nav) {}
    ~INS_GPS_NAV() {
      delete _nav;
    }
    back_propagated_list_t back_propagated() {
      back_propagated_list_t res;
      if(options.back_propagate){
        for(typename snapshots_t::iterator it(snapshots.begin());
            it != snapshots.end();
            ++it){
          res.push_back(
              NAV::back_propagated_item_t(it->deltaT_from_last_correct, *(it->ins_gps)));
        }
        //cerr << "snapshots.size() : " << snapshots.size() << endl;
      }
      return res;
    }
    
  public:
    void set_filter_Q(const Vector3<float_sylph_t> &accel, const Vector3<float_sylph_t> &gyro){
      /**
       * Initialization of matrix Q, input covariance matrix, of Kalman filter.
       * orthogonal elements are
       *  0-2 : accelerometer output variance in X, Y, Z axes, [m/s^2]^2
       *  3-5 : angular speed output variance in X, Y, Z axes, [rad/s]^2
       *  6   : gravity variance [m/s^2]^2, normally set small value, such as 1E-6
       */
      {
        Matrix<float_sylph_t> Q(nav.getFilter().getQ());
        
        Q(0, 0) = pow(accel.getX(), 2);
        Q(1, 1) = pow(accel.getY(), 2);
        Q(2, 2) = pow(accel.getZ(), 2);
        Q(3, 3) = pow(gyro.getX(), 2);
        Q(4, 4) = pow(gyro.getY(), 2);
        Q(5, 5) = pow(gyro.getZ(), 2);
        Q(6, 6) = 1E-6; //1E-14
        
        nav.getFilter().setQ(Q);
      }
    }

    void init(
        const float_sylph_t &latitude, 
        const float_sylph_t &longitude, 
        const float_sylph_t &height,
        const float_sylph_t &v_north, 
        const float_sylph_t &v_east, 
        const float_sylph_t &v_down,
        const float_sylph_t &yaw, 
        const float_sylph_t &pitch, 
        const float_sylph_t &roll){
      
      nav.initPosition(latitude, longitude, height);
      nav.initVelocity(v_north, v_east, v_down);
      nav.initAttitude(yaw, pitch, roll);

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
       *        For instance, 1E-4 is a sufficiently big value.
       */
      {
        Matrix<float_sylph_t> P(nav.getFilter().getP());

        P(0, 0) = P(1, 1) = P(2, 2) = 1E+1;
        P(3, 3) = P(4, 4) = P(5, 5) = 1E-8;
        P(6, 6) = 1E+2;
        P(7, 7) = P(8, 8) = P(9, 9) = 1E-4;

        nav.getFilter().setP(P);
      }
    }

#define MAKE_COMMIT_FUNC(fname, rtype, attr) \
rtype fname() attr {return nav.fname();}
    MAKE_COMMIT_FUNC(longitude, float_sylph_t, const);
    MAKE_COMMIT_FUNC(latitude, float_sylph_t, const);
    MAKE_COMMIT_FUNC(height, float_sylph_t, const);
    MAKE_COMMIT_FUNC(v_north, float_sylph_t, const);
    MAKE_COMMIT_FUNC(v_east, float_sylph_t, const);
    MAKE_COMMIT_FUNC(v_down, float_sylph_t, const);
    MAKE_COMMIT_FUNC(heading, float_sylph_t, const);
    MAKE_COMMIT_FUNC(euler_phi, float_sylph_t, const);
    MAKE_COMMIT_FUNC(euler_theta, float_sylph_t, const);
    MAKE_COMMIT_FUNC(euler_psi, float_sylph_t, const);
    MAKE_COMMIT_FUNC(azimuth, float_sylph_t, const);
#undef MAKE_COMMIT_FUNC
    
    float_sylph_t &operator[](unsigned index){return nav[index];}
    
    NAV &update(
        const Vector3<float_sylph_t> &accel, 
        const Vector3<float_sylph_t> &gyro, 
        const float_sylph_t &deltaT){
      nav.update(accel, gyro, deltaT);
      return *this;
    }
  
  public:
    NAV &correct(const GPS_UBLOX_3D<float_sylph_t> &gps){
      nav.correct(gps);
      return *this;
    }
    NAV &correct(
        const GPS_UBLOX_3D<float_sylph_t> &gps, 
        const Vector3<float_sylph_t> &lever_arm_b,
        const Vector3<float_sylph_t> &omega_b2i_4b){
      nav.correct(gps, lever_arm_b, omega_b2i_4b);
      return *this;
    }
    NAV &correct_yaw(const float_sylph_t &delta_yaw){
      nav.correct_yaw(delta_yaw, pow(deg2rad(options.mag_heading_accuracy_deg), 2));
      return *this;
    }
    
    /**
     * print label
     */
    void label(std::ostream &out = std::cout) const {
      NAV::label(out);
    }
  
  protected:
    /**
     * print current state
     * 
     * @param itow current time
     */
    void dump(std::ostream &out) const {
      NAV::dump(out);
    }
};

template <class INS_GPS_BE>
class INS_GPS_BE_NAV : public INS_GPS_NAV<INS_GPS_BE> {
  public:
    typedef INS_GPS_NAV<INS_GPS_BE> super_t;
    INS_GPS_BE_NAV() : super_t() {
      /**
       * Configuration for bias drift of accelerometer and gyro.
       */
      super_t::nav.beta_accel() *= 0.1;
      super_t::nav.beta_gyro() *= 0.1;
      {
        Matrix<float_sylph_t> P(super_t::nav.getFilter().getP());
        P(10, 10) = P(11, 11) = P(12, 12) = 1E-4;
        P(13, 13) = P(14, 14) = P(15, 15) = 1E-6;
        super_t::nav.getFilter().setP(P);
      }
      {
        Matrix<float_sylph_t> Q(super_t::nav.getFilter().getQ());
        Q(7, 7) = 1E-6;
        Q(8, 8) = 1E-6;
        Q(9, 9) = 1E-6;
        Q(10, 10) = 1E-8;
        Q(11, 11) = 1E-8;
        Q(12, 12) = 1E-8;
        super_t::nav.getFilter().setQ(Q);
      }
    }
    ~INS_GPS_BE_NAV(){}
    
    NAV &correct(
        const GPS_UBLOX_3D<float_sylph_t> &gps, 
        const Vector3<float_sylph_t> &lever_arm_b,
        const Vector3<float_sylph_t> &omega_b2i_4b){
      return super_t::correct(
          gps, lever_arm_b, omega_b2i_4b - super_t::nav.bias_gyro());
    }
    
    void label(std::ostream &out = std::cout) const {
      super_t::label(out);
      out << "bias_accel(X)" << ", "  //Bias
         << "bias_accel(Y)" << ", "
         << "bias_accel(Z)" << ", "
         << "bias_gyro(X)" << ", "
         << "bias_gyro(Y)" << ", "
         << "bias_gyro(Z)" << ", ";
    }
  
  protected:
    void dump(std::ostream &out) const {
      super_t::dump(out);
      Vector3<float_sylph_t> &ba(super_t::nav.bias_accel());
      Vector3<float_sylph_t> &bg(super_t::nav.bias_gyro());
      out << ba.getX() << ", "     // Bias
           << ba.getY() << ", "
           << ba.getZ() << ", "
           << bg.getX() << ", "
           << bg.getY() << ", "
           << bg.getZ() << ", ";
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

  bool check_spec(const char *str){
    using std::strstr;
    using std::strlen;
    using std::strcmp;
    using std::atoi;
    using std::atof;
    char *spec(const_cast<char *>(str));
    if(strstr(spec, "index_base") == spec){
      spec += strlen("index_base");
      index_base = atoi(spec);
      return true;
    }
    if(strstr(spec, "index_temp_ch") == spec){
      spec += strlen("index_temp_ch");
      index_temp_ch = atoi(spec);
      return true;
    }
#define TO_STRING(name) # name
#define make_proc1(name, sensor, item) \
if(strstr(spec, TO_STRING(name)) == spec){ \
  spec += strlen(TO_STRING(name)); \
  std::cerr << TO_STRING(name) << ":"; \
  for(int i(0); i < 3; i++){ \
    sensor.item[i] = strtod(spec, &spec); \
    std::cerr << " " << sensor.item[i]; \
  } \
  std::cerr << std::endl; \
  return true; \
}
#define make_proc2(name, sensor, item) \
if(strstr(spec, TO_STRING(name)) == spec){ \
  spec += strlen(TO_STRING(name)); \
  std::cerr << TO_STRING(name) << ": {"; \
  for(int i(0); i < 3; i++){ \
    for(int j(0); j < 3; j++){ \
      sensor.item[i][j] = strtod(spec, &spec); \
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
  Vector3<float_sylph_t> raw2gyro(const int *raw_data) const{
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

void a_packet_handler(const A_Observer_t &);
void g_packet_handler(const G_Observer_t &);
void m_packet_handler(const M_Observer_t &);

bool require_switch_processor(false);

class StreamProcessor : public Processor_t {
  protected:
    int invoked;
    istream *_in;
    
  public:
    bool use_lever_arm;
    Vector3<float_sylph_t> lever_arm;
    StandardCalibration calibration;
    deque<A_Packet> a_packet_deque;
    G_Packet g_packet;
    bool g_packet_updated;
    int g_packet_wn;
    deque<M_Packet> m_packet_deque;
    StreamProcessor()
        : Processor_t(), _in(NULL), invoked(0),
        use_lever_arm(false), lever_arm(), calibration(),
        a_packet_deque(),
        g_packet(), g_packet_updated(false), g_packet_wn(0),
        m_packet_deque() {

    }
    ~StreamProcessor(){}
    
    void set_stream(istream *in){_in = in;}

    /**
     * Process stream in units of 1 page
     * 
     * @param in stream
     * @return (bool) true when success, otherwise false.
     */
    bool process_1page(){
      char buffer[PAGE_SIZE];
      
      int read_count;
      _in->read(buffer, PAGE_SIZE);
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
    
      if(read_count < PAGE_SIZE){
#if DEBUG
        cerr << "--skipped-- : " << invoked << " page ; count = " << read_count << endl;
#endif
      }
      
      process(buffer, read_count);
      return true;
    }

    Vector3<float_sylph_t> get_mag() {
      return m_packet_deque.empty()
          ? Vector3<float_sylph_t>(1, 0, 0) // heading is north
          : m_packet_deque.back().mag;
    }

    Vector3<float_sylph_t> get_mag(const float_sylph_t &itow){
      if(m_packet_deque.size() < 2){
        return Vector3<float_sylph_t>(1, 0, 0); // heading is north
      }
      deque<M_Packet>::iterator
          previous_it(m_packet_deque.begin()),
          next_it(m_packet_deque.begin() + 1);
      for(int i(distance(previous_it, m_packet_deque.end()));
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
} *current_processor;

typedef vector<StreamProcessor *> processor_storage_t;
processor_storage_t processor_storage;

class Status{
  private:
    bool initalized;
    NAV &nav;
    deque<A_Packet> before_init_a_packets;
    int before_init_counter;
    int before_init_counter_min;

  // Used for compensation of lever arm effect
  protected:
    Vector3<float_sylph_t> gyro_storage[16];
    int gyro_index;
    bool gyro_init;

  public:
    Status(NAV &_nav) : initalized(false), nav(_nav), gyro_index(0), gyro_init(false),
        before_init_a_packets(),
        before_init_counter(0),
        before_init_counter_min(options.has_initial_attitude ? 1 : 0x10) {
    }
  
  public:
    NAV &get_nav() {return nav;}
    
    void dump_label(){
      if(!options.out_is_N_packet){
        options.out() << "mode" << ", "
            << "itow" << ", ";
        nav.label(options.out());
        options.out() << endl;
      }
    }
    
    enum dump_mode_t {
      DUMP_UPDATE, DUMP_CORRECT, 
    };
  
  protected:
    /**
     * Dump state
     * 
     * @param label operation mode string
     * @param itow current time
     * @param target NAV to be outputted
     */
    static void dump(const char *label, const float_sylph_t &itow, const NAVData &target){
      
      if(options.out_is_N_packet){
        char buf[PAGE_SIZE];
        target.encode_N0(itow, buf);
        options.out().write(buf, sizeof(buf));
        return;
      }

      options.out() << label << ", "
          << itow << ", "
          << target << endl;
    }

  public:
    /**
     * Dump state
     *
     * @param mode operation mode
     * @param itow current time
     */
    void dump(const dump_mode_t mode, const float_sylph_t &itow){

      if(!initalized){return;}
      
      switch(mode){
        case DUMP_UPDATE:
          if(!options.dump_update){return;}
          if(options.back_propagate){return;} // When smoothing is activated, skip.
          dump("TU", itow, nav);
          break;
        case DUMP_CORRECT:
          if(options.back_propagate){ // When smoothing is activated
            NAV::back_propagated_list_t list(nav.back_propagated());
            int index(0);
            for(NAV::back_propagated_list_t::iterator it(list.begin());
                it != list.end();
                ++it, index++){
              if(it->deltaT_from_last_correct >= options.back_propagate_depth){
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
              dump(label, itow + it->deltaT_from_last_correct, it->nav);
            }
            break;
          }
          
          if(!options.dump_correct){return;}
          dump("MU", itow, nav);
          break;
        default:
          return;
      }
    }
    
    /**
     * Perform time update by using acceleration and angular speed obtained with accelerometer and gyro.
     * 
     * @param a_packet raw values of ADC
     */
    void time_update(const A_Packet &a_packet){
      static A_Packet previous;
      static bool has_previous(false);
      
      Vector3<float_sylph_t>
          accel(current_processor->calibration.raw2accel(a_packet.ch)),
          gyro(current_processor->calibration.raw2gyro(a_packet.ch));

#define pow2(x) ((x) * (x))
      if(!initalized){
        before_init_a_packets.push_back(a_packet);
        if(++before_init_counter > before_init_counter_min){
          before_init_a_packets.pop_front();
        }
        return;
      }
      
      // for LAE
      {
        gyro_storage[gyro_index++] = gyro;
        if(gyro_index == (sizeof(gyro_storage) / sizeof(gyro_storage[0]))){
          gyro_index = 0;
          if(!gyro_init) gyro_init = true;
        }
      }
      
      if(has_previous){ // Check interval from the last measurement update
        float_sylph_t interval(previous.interval(a_packet));
#define INTERVAL_THRESHOLD 10
#define INTERVAL_FORCE_VALUE 0.01
        if((interval < 0) || (interval >= INTERVAL_THRESHOLD)){
          // Rewrite time stamp forcedly when discontinuity is too large.
          interval = INTERVAL_FORCE_VALUE;
        }

        nav.update(accel, gyro, interval);
      }
      previous = a_packet;
      has_previous = true;
    }

    /**
     * Perform measurement update by using position and velocity obtained with GPS receiver.
     * 
     * @param g_packet observation data of GPS receiver
     */
    void measurement_update(const G_Packet &g_packet){
      
      if(g_packet.acc_2d >= 100.){return;} // When estimated accuracy is too big, skip.
      if(initalized){
        cerr << "MU : " << setprecision(10) << g_packet.itow << endl;
        
        if(gyro_init && (current_processor->use_lever_arm)){ // When use lever arm effect.
          Vector3<float_sylph_t> omega_b2i_4n;
          for(int i(0); i < (sizeof(gyro_storage) / sizeof(gyro_storage[0])); i++){
            omega_b2i_4n += gyro_storage[i];
          }
          omega_b2i_4n /= (sizeof(gyro_storage) / sizeof(gyro_storage[0]));
          nav.correct(
              g_packet.convert(), 
              current_processor->lever_arm,
              omega_b2i_4n);
        }else{ // When do not use lever arm effect.
          nav.correct(g_packet.convert());
        }
        if(!current_processor->m_packet_deque.empty()){ // When magnetic sensor is activated, try to perform yaw compensation
          if((options.yaw_correct_with_mag_when_speed_less_than_ms > 0)
              && (pow(g_packet.vel_ned[0], 2) + pow(g_packet.vel_ned[1], 2)) < pow(options.yaw_correct_with_mag_when_speed_less_than_ms, 2)){
            nav.correct_yaw(nav.get_mag_delta_yaw(current_processor->get_mag(g_packet.itow)));
          }
        }
      }else if((current_processor == processor_storage.front())
          && (before_init_counter >= before_init_counter_min)
          && (std::abs(before_init_a_packets.front().itow - g_packet.itow) < (0.1 * before_init_a_packets.size())) // time synchronization check
          && (g_packet.acc_2d <= 20.) && (g_packet.acc_v <= 10.)){
        /*
         * Filter is activated when the estimate error in horizontal and vertical positions are under 20 and 10 meters, respectively.
         */
        
        float_sylph_t attitude[3];
        for(int i(0); i < sizeof(attitude) / sizeof(attitude[0]); ++i){
          attitude[i] = deg2rad(options.init_attitude_deg[i]);
        }
        float_sylph_t&yaw(attitude[0]), &pitch(attitude[1]), &roll(attitude[2]);
        float_sylph_t latitude(deg2rad(g_packet.llh[0])), longitude(deg2rad(g_packet.llh[1]));
        
        while(!options.has_initial_attitude){
          // Estimate initial attitude by using accelerometer and magnetic sensor (if available) under static assumption
          
          typedef Vector3<float_sylph_t> vec_t;
          
          // Normalization
          vec_t acc(0, 0, 0);
          for(deque<A_Packet>::iterator it(before_init_a_packets.begin());
              it != before_init_a_packets.end();
              ++it){
            acc += current_processor->calibration.raw2accel(it->ch);
          }
          acc /= before_init_a_packets.size();
          vec_t acc_reg(-acc / acc.abs());
          
          // Estimate pitch angle
          pitch = -asin(acc_reg[0]);
          // Then estimate roll angle
          roll = atan2(acc_reg[1], acc_reg[2]);
          
          // Estimate yaw when magnetic sensor is available
          if(!current_processor->m_packet_deque.empty()){
            yaw = nav.get_mag_yaw(current_processor->get_mag(g_packet.itow), pitch, roll, latitude, longitude, g_packet.llh[2]);
          }
          
          break;
        }
        
        cerr << "Init : " << setprecision(10) << g_packet.itow << endl;
        initalized = true;
        nav.init(
            latitude, longitude, g_packet.llh[2],
            g_packet.vel_ned[0], g_packet.vel_ned[1], g_packet.vel_ned[2],
            yaw, pitch, roll);
        cerr << "Initial attitude (yaw, pitch, roll) [deg]: "
            << rad2deg(yaw) << ", "
            << rad2deg(pitch) << ", "
            << rad2deg(roll) << endl;
        
        dump_label();
      }
    }
    
    bool is_initalized(){return initalized;}
};

/**
 * A page (ADC value)
 * 
 * @param observer
 */
void a_packet_handler(const A_Observer_t &observer){
  if(!observer.validate()){return;}
  
  A_Packet packet;
  packet.itow = observer.fetch_ITOW();
  
  A_Observer_t::values_t values(observer.fetch_values());
  for(int i = 0; i < 8; i++){
    packet.ch[i] = values.values[i];
  }
  packet.ch[8] = values.temperature;
  
  deque<A_Packet> &a_packet_deque(
      current_processor->a_packet_deque);

  while(options.reduce_1pps_sync_error){
    if(a_packet_deque.empty()){break;}
    float_sylph_t delta_t(packet.itow - a_packet_deque.back().itow);
    if((delta_t < 1) || (delta_t >= 2)){break;}
    packet.itow -= 1;
    break;
  }

  while(a_packet_deque.size() >= 128){
    a_packet_deque.pop_front();
  }
  a_packet_deque.push_back(packet);
}

/**
 * G page (u-blox)
 * 
 * Extract the following data.
 * {class, id} = {0x01, 0x02} : position
 * {class, id} = {0x01, 0x12} : velocity
 * 
 * @param observer
 */
void g_packet_handler(const G_Observer_t &observer){
  if(!observer.validate()){return;}
  
  G_Packet &packet(current_processor->g_packet);
  
  G_Observer_t::packet_type_t
      packet_type(observer.packet_type());
  switch(packet_type.mclass){
    case 0x01: {
      switch(packet_type.mid){
        case 0x02: { // NAV-POSLLH
          G_Observer_t::position_t 
            position(observer.fetch_position());
          G_Observer_t::position_acc_t
            position_acc(observer.fetch_position_acc());
          
          packet.itow = observer.fetch_ITOW();
          packet.llh[0] = position.latitude;
          packet.llh[1] = position.longitude;
          packet.llh[2] = position.altitude;
          packet.acc_2d = position_acc.horizontal;
          packet.acc_v = position_acc.vertical;
          
          break;
        }
        case 0x06: { // NAV-SOL
          G_Observer_t::solution_t solution(observer.fetch_solution());
          if(solution.status_flags & G_Observer_t::solution_t::WN_VALID){
            current_processor->g_packet_wn = solution.week;
          }
          break;
        }
        case 0x12: { // NAV-VELNED
          G_Observer_t::velocity_t
            velocity(observer.fetch_velocity());
          G_Observer_t::velocity_acc_t
            velocity_acc(observer.fetch_velocity_acc());
          
          if(std::abs(packet.itow - observer.fetch_ITOW()) < 1E-3){
            packet.vel_ned[0] = velocity.north;
            packet.vel_ned[1] = velocity.east;
            packet.vel_ned[2] = velocity.down;
            packet.acc_vel = velocity_acc.acc;
            
            current_processor->g_packet_updated = true;
            
            require_switch_processor = true;
          }
            
          break;
        }
      }
      break;
    }
    case 0x02: { 
      break;
    }
    default:
      break;
  }
}

void m_packet_handler(const M_Observer_t &observer){
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
  
  while(current_processor->m_packet_deque.size() > 0x40){
    current_processor->m_packet_deque.pop_front();
  }

  // TODO: magnetic sensor axes must correspond to ones of accelerometer and gyro.
  Vector3<float_sylph_t> mag(values.x[3], values.y[3], values.z[3]);
  M_Packet m_packet;
  m_packet.itow = observer.fetch_ITOW();
  m_packet.mag = mag;

  while(options.reduce_1pps_sync_error){
    if(current_processor->m_packet_deque.empty()){break;}
    float_sylph_t delta_t(m_packet.itow - current_processor->m_packet_deque.back().itow);
    if((delta_t < 1) || (delta_t >= 2)){break;}
    m_packet.itow -= 1;
    break;
  }

  current_processor->m_packet_deque.push_back(m_packet);
}

void loop(){
  INS_GPS_NAV<
      INS_GPS2<
          float_sylph_t,
          KalmanFilter<float_sylph_t> > > nav;
  INS_GPS_NAV<INS_GPS2<float_sylph_t> > nav_udkf;
  
  INS_GPS_BE_NAV<
      INS_GPS2_BiasEstimated<
          float_sylph_t,
          KalmanFilter<float_sylph_t> > > nav_bias;
  INS_GPS_BE_NAV<INS_GPS2_BiasEstimated<float_sylph_t> > nav_bias_udkf;
  
#define setQ(ins_gps) \
ins_gps.set_filter_Q( \
    processor_storage.front()->calibration.sigma_accel(), \
    processor_storage.front()->calibration.sigma_gyro());
  setQ(nav);
  setQ(nav_udkf);
  setQ(nav_bias);
  setQ(nav_bias_udkf);
#undef setQ

  Status status(
      options.use_udkf
          ? (options.est_bias ? (NAV &)nav_bias_udkf : (NAV &)nav_udkf)
          : (options.est_bias ? (NAV &)nav_bias : (NAV &)nav));
  
  while(true){
    for(processor_storage_t::iterator it(processor_storage.begin());
        it != processor_storage.end();
        ++it){
      if((*it)->g_packet_updated){continue;}
      current_processor = *it;
      require_switch_processor = false;
      while(!require_switch_processor){
        if(!current_processor->process_1page()){
          if(it == processor_storage.begin()){
            return;
          }else{
            break;
          }
        }
      }
    }

    // Check and sort G packets in order of observation time
    typedef vector<StreamProcessor *> mu_queue_t;
    mu_queue_t mu_queue;
    
    mu_queue.push_back(*processor_storage.begin());
    bool require_search_again(false);
    for(processor_storage_t::iterator it(processor_storage.begin() + 1);
        it != processor_storage.end();
        ++it){
      if(!((*it)->g_packet_updated)){continue;}
      float_sylph_t itow((*it)->g_packet.itow);
      float_sylph_t interval(
          itow - processor_storage.front()->g_packet.itow);
      
      if(std::abs(interval) <= 0.2){ // When the observation times are near, use the data
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
    
    if(require_search_again){continue;}
    
    float_sylph_t latest_measurement_update_itow(0);
    int latest_measurement_update_gpswn(0);
    for(mu_queue_t::iterator it_mu(mu_queue.begin());
        it_mu != mu_queue.end();
        ++it_mu){
      
      current_processor = *it_mu;
      G_Packet g_packet(current_processor->g_packet);
      current_processor->g_packet_updated = false;

      if((options.start_gpswn > current_processor->g_packet_wn) // Week number check
          || (options.start_gpstime > g_packet.itow)){ // Time check
        continue;
      }
      
      deque<A_Packet> &a_packet_deque(
          processor_storage.front()->a_packet_deque);
      deque<A_Packet>::iterator it_tu(a_packet_deque.begin()), it_tu_end(a_packet_deque.end());
      const bool a_packet_deque_has_item(it_tu != it_tu_end);

      if(options.gps_fake_lock){
        if(a_packet_deque_has_item){
          g_packet.itow = (it_tu_end - 1)->itow;
        }
        g_packet.llh[0] = g_packet.llh[1] = g_packet.llh[2] = 0;
        g_packet.acc_2d = g_packet.acc_v = 1E+1;
        g_packet.vel_ned[0] = g_packet.vel_ned[1] = g_packet.vel_ned[2] = 0;
        g_packet.acc_vel = 1;
      }
    
      // Time update to the last sample before GPS observation
      for(; 
          (it_tu != it_tu_end) && (it_tu->itow < g_packet.itow);
          ++it_tu){
        status.time_update(*it_tu);
        status.dump(Status::DUMP_UPDATE, it_tu->itow);
      }
      
      // Time update to the GPS observation
      if(a_packet_deque_has_item){
        A_Packet interpolation((it_tu != it_tu_end) ? *it_tu : *(it_tu - 1));
        interpolation.itow = g_packet.itow;
        status.time_update(interpolation);
      }
      
      a_packet_deque.erase(a_packet_deque.begin(), it_tu);
      
      // Measurement update
      status.measurement_update(g_packet);
      latest_measurement_update_itow = g_packet.itow;
      latest_measurement_update_gpswn = current_processor->g_packet_wn;
    }
    
    status.dump(Status::DUMP_CORRECT, latest_measurement_update_itow);
    
    if((latest_measurement_update_itow >= options.end_gpstime)
        && (latest_measurement_update_gpswn >= options.end_gpswn)){
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

  { // NinjsScan default calibration parameters
#define config(spec) stream_processor->calibration.check_spec(spec);
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

  for(int arg_index(1); arg_index < argc; arg_index++){
    const char *value;
    if(value = Options::get_value(argv[arg_index], "calib_file", false)){ // calibration file
      cerr << "IMU Calibration file (" << value << ") reading..." << endl;
      fstream fin(value);
      if(fin.fail()){
        cerr << "(error!) Calibration file not found: " << value << endl;
        return -1;
      }
      char buf[1024];
      while(!fin.eof()){
        fin.getline(buf, sizeof(buf));
        stream_processor->calibration.check_spec(buf);
      }
      continue;
    }

    if(value = Options::get_value(argv[arg_index], "lever_arm", false)){ // Lever Arm
      if(std::sscanf(value, "%lf,%lf,%lf",
          &(stream_processor->lever_arm[0]),
          &(stream_processor->lever_arm[1]),
          &(stream_processor->lever_arm[2])) != 3){
        cerr << "(error!) Lever arm option requires 3 arguments." << endl;
        exit(-1);
      }
      std::cerr << "lever_arm: " << stream_processor->lever_arm << std::endl;
      stream_processor->use_lever_arm = true;
      continue;
    }

    if(options.check_spec(argv[arg_index])){continue;}
    
    if(!stream_processor){
      cerr << "(error!) Too many log files." << endl;
      exit(-1);
    }

    stream_processor->set_a_handler(a_packet_handler);  // Register A page handler
    stream_processor->set_g_handler(g_packet_handler);  // Register G page handler
    if(options.use_magnet){
      stream_processor->set_m_handler(m_packet_handler);  // Register M page handler
    }
    
    cerr << "Log file: ";
    istream &in(options.spec2istream(argv[arg_index]));
    stream_processor->set_stream(
        options.in_sylphide ? new SylphideIStream(in, PAGE_SIZE) : &in);

    processor_storage.push_back(stream_processor);
    stream_processor = NULL;
  }

  if(processor_storage.empty()){
    cerr << "(error!) No log file." << endl;
    exit(-1);
  }

  if(options.out_sylphide){
    options._out = new SylphideOStream(options.out(), PAGE_SIZE);
  }else{
    options.out() << setprecision(10);
  }

  loop();
  
  for(processor_storage_t::iterator it(processor_storage.begin());
      it != processor_storage.end();
      ++it){
    delete *it;
  }

  return 0;
}
