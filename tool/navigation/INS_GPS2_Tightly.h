/*
 *  INS_GPS2_Tightly.h, header file to perform calculation of tightly integration of INS and GPS.
 *  Copyright (C) 2018 M.Naruoka (fenrir)
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __INS_GPS2_TIGHTLY_H__
#define __INS_GPS2_TIGHTLY_H__

#include <cmath>
#include <iostream>
#include <vector>
#include <utility>

#include "INS.h"
#include "Filtered_INS2.h"
#include "INS_GPS2.h"
#include "GPS_Solver_Base.h"
#include "coordinate.h"

template <typename BaseINS, unsigned int Clocks>
class INS_ClockErrorEstimated;

template <class BaseINS, unsigned int Clocks>
struct INS_Property<INS_ClockErrorEstimated<BaseINS, Clocks> > {
  static const unsigned CLOCKS_SUPPORTED = Clocks;
  static const unsigned STATE_VALUES_WITHOUT_CLOCK_ERROR = INS_Property<BaseINS>::STATE_VALUES;
  static const unsigned STATE_VALUES_CLOCK_ERROR = 2 * CLOCKS_SUPPORTED;
  static const unsigned STATE_VALUES = STATE_VALUES_WITHOUT_CLOCK_ERROR + STATE_VALUES_CLOCK_ERROR;
};

template <
    typename BaseINS = INS<>, unsigned int Clocks = 1>
class INS_ClockErrorEstimated : public BaseINS {
  public:
#if defined(__GNUC__) && (__GNUC__ < 5)
    typedef typename BaseINS::float_t float_t;
    typedef typename BaseINS::vec3_t vec3_t;
#else
    using typename BaseINS::float_t;
    using typename BaseINS::vec3_t;
#endif

  public:
    static const unsigned CLOCKS_SUPPORTED
        = INS_Property<INS_ClockErrorEstimated<BaseINS, Clocks> >::CLOCKS_SUPPORTED;
    static const unsigned STATE_VALUES_WITHOUT_CLOCK_ERROR
        = INS_Property<INS_ClockErrorEstimated<BaseINS, Clocks> >::STATE_VALUES_WITHOUT_CLOCK_ERROR;
    static const unsigned STATE_VALUES_CLOCK_ERROR
        = INS_Property<INS_ClockErrorEstimated<BaseINS, Clocks> >::STATE_VALUES_CLOCK_ERROR;
    static const unsigned STATE_VALUES
        = INS_Property<INS_ClockErrorEstimated<BaseINS, Clocks> >::STATE_VALUES; ///< Number of state values
    virtual unsigned state_values() const {return STATE_VALUES;}

  protected:
    float_t m_clock_error[CLOCKS_SUPPORTED];  ///< receiver clock error [m]
    float_t m_clock_error_rate[CLOCKS_SUPPORTED];  ///< receiver clock error rate [m/s]

  public:
    INS_ClockErrorEstimated()
        : BaseINS() {
      for(unsigned int i(0); i < CLOCKS_SUPPORTED; ++i){
        m_clock_error[i] = 0;
        m_clock_error_rate[i] = 0;
      }
    }

    INS_ClockErrorEstimated(const INS_ClockErrorEstimated &orig, const bool &deepcopy = false)
        : BaseINS(orig, deepcopy) {
      for(unsigned int i(0); i < CLOCKS_SUPPORTED; ++i){
        m_clock_error[i] = orig.m_clock_error[i];
        m_clock_error_rate[i] = orig.m_clock_error_rate[i];
      }
    }

    virtual ~INS_ClockErrorEstimated(){}

    float_t &clock_error(const unsigned int &index = 0){return m_clock_error[index];}
    float_t &clock_error_rate(const unsigned int &index = 0){return m_clock_error_rate[index];}

    using BaseINS::operator[];

    const float_t &operator[](const unsigned &index) const {
      int index_offset(index - STATE_VALUES_WITHOUT_CLOCK_ERROR);
      if((index_offset >= 0) && (index_offset < STATE_VALUES_CLOCK_ERROR)){
        int index_clock(index_offset >> 1);
        return (index_offset % 2 == 0) ? m_clock_error[index_clock] : m_clock_error_rate[index_clock];
      }
      return BaseINS::operator[](index);
    }

    void update(
        const vec3_t &accel, const vec3_t &gyro,
        const float_t &deltaT){
      for(unsigned int i(0); i < CLOCKS_SUPPORTED; ++i){
        m_clock_error[i] += m_clock_error_rate[i] * deltaT;
      }
      BaseINS::update(accel, gyro, deltaT);
    }
};

template <class BaseINS, unsigned int Clocks>
class Filtered_INS2_Property<INS_ClockErrorEstimated<BaseINS, Clocks> >
    : public Filtered_INS2_Property<BaseINS> {
  public:
    static const unsigned P_SIZE_WITHOUT_CLOCK_ERROR
#if defined(_MSC_VER)
        = Filtered_INS2_Property<BaseINS>::P_SIZE
#endif
        ;
    static const unsigned Q_SIZE_WITHOUT_CLOCK_ERROR
#if defined(_MSC_VER)
        = Filtered_INS2_Property<BaseINS>::Q_SIZE
#endif
        ;
    static const unsigned P_SIZE_CLOCK_ERROR
#if defined(_MSC_VER)
        = INS_ClockErrorEstimated<BaseINS, Clocks>::STATE_VALUES_CLOCK_ERROR
#endif
        ;
    static const unsigned Q_SIZE_CLOCK_ERROR
#if defined(_MSC_VER)
        = INS_ClockErrorEstimated<BaseINS, Clocks>::STATE_VALUES_CLOCK_ERROR
#endif
        ;
    static const unsigned P_SIZE
#if defined(_MSC_VER)
        = P_SIZE_WITHOUT_CLOCK_ERROR + P_SIZE_CLOCK_ERROR
#endif
        ;
    static const unsigned Q_SIZE
#if defined(_MSC_VER)
        = Q_SIZE_WITHOUT_CLOCK_ERROR + Q_SIZE_CLOCK_ERROR
#endif
        ;
};

#if !defined(_MSC_VER)
template <class BaseINS, unsigned int Clocks>
const unsigned Filtered_INS2_Property<INS_ClockErrorEstimated<BaseINS, Clocks> >::P_SIZE_WITHOUT_CLOCK_ERROR
    = Filtered_INS2_Property<BaseINS>::P_SIZE;

template <class BaseINS, unsigned int Clocks>
const unsigned Filtered_INS2_Property<INS_ClockErrorEstimated<BaseINS, Clocks> >::Q_SIZE_WITHOUT_CLOCK_ERROR
    = Filtered_INS2_Property<BaseINS>::Q_SIZE;

template <class BaseINS, unsigned int Clocks>
const unsigned Filtered_INS2_Property<INS_ClockErrorEstimated<BaseINS, Clocks> >::P_SIZE_CLOCK_ERROR
    = INS_ClockErrorEstimated<BaseINS>::STATE_VALUES_CLOCK_ERROR;

template <class BaseINS, unsigned int Clocks>
const unsigned Filtered_INS2_Property<INS_ClockErrorEstimated<BaseINS, Clocks> >::Q_SIZE_CLOCK_ERROR
    = INS_ClockErrorEstimated<BaseINS>::STATE_VALUES_CLOCK_ERROR;

template <class BaseINS, unsigned int Clocks>
const unsigned Filtered_INS2_Property<INS_ClockErrorEstimated<BaseINS, Clocks> >::P_SIZE
    = P_SIZE_WITHOUT_CLOCK_ERROR + P_SIZE_CLOCK_ERROR;

template <class BaseINS, unsigned int Clocks>
const unsigned Filtered_INS2_Property<INS_ClockErrorEstimated<BaseINS, Clocks> >::Q_SIZE
    = Q_SIZE_WITHOUT_CLOCK_ERROR + Q_SIZE_CLOCK_ERROR;
#endif

template <
    class BaseFINS = Filtered_INS2<INS_ClockErrorEstimated<INS<> > > >
class Filtered_INS_ClockErrorEstimated : public BaseFINS {
  public:
#if defined(__GNUC__) && (__GNUC__ < 5)
    typedef typename BaseFINS::float_t float_t;
    typedef typename BaseFINS::vec3_t vec3_t;
    typedef typename BaseFINS::mat_t mat_t;
#else
    using typename BaseFINS::float_t;
    using typename BaseFINS::vec3_t;
    using typename BaseFINS::mat_t;
#endif

  protected:
    float_t m_beta_clock_error;
    float_t m_beta_clock_error_rate;

  public:
    using BaseFINS::ins_t::CLOCKS_SUPPORTED;
    using BaseFINS::ins_t::STATE_VALUES_WITHOUT_CLOCK_ERROR;
    using BaseFINS::ins_t::STATE_VALUES_CLOCK_ERROR;
    using BaseFINS::property_t::P_SIZE_WITHOUT_CLOCK_ERROR;
    using BaseFINS::property_t::Q_SIZE_WITHOUT_CLOCK_ERROR;
    using BaseFINS::property_t::P_SIZE_CLOCK_ERROR;
    using BaseFINS::property_t::Q_SIZE_CLOCK_ERROR;
    using BaseFINS::m_clock_error;

    Filtered_INS_ClockErrorEstimated()
        : BaseFINS(),
        m_beta_clock_error(1), m_beta_clock_error_rate(1) {

    }

    Filtered_INS_ClockErrorEstimated(
        const Filtered_INS_ClockErrorEstimated &orig,
        const bool &deepcopy = false)
        : BaseFINS(orig, deepcopy),
        m_beta_clock_error(orig.m_beta_clock_error),
        m_beta_clock_error_rate(orig.m_beta_clock_error_rate) {

    }

    ~Filtered_INS_ClockErrorEstimated(){}

    float_t &beta_clock_error(){return m_beta_clock_error;}
    float_t &beta_clock_error_rate(){return m_beta_clock_error_rate;}

    void getAB(
        const vec3_t &accel,
        const vec3_t &gyro,
        typename BaseFINS::getAB_res &res) const {

      BaseFINS::getAB(accel, gyro, res);

      { // A matrix modification
        // Modify A associated with clock error, then, rate
        for(unsigned i(P_SIZE_WITHOUT_CLOCK_ERROR), j(0);
            j < CLOCKS_SUPPORTED; i += 2, j++){

          /* Matrix layout (A)
           * [-b_c] [      1] : row(j*2)   => clock(j) error
           * [   0] [-b_cdot] : row(j*2+1) => clock(j) error rate
           */
          res.A[i][i] += (-m_beta_clock_error);
          res.A[i][i + 1] += 1; // d(clock_error)/dt = clock_error_rate
          res.A[i + 1][i + 1] += (-m_beta_clock_error_rate);
        }
      }

      { // B matrix modification
        for(unsigned i(P_SIZE_WITHOUT_CLOCK_ERROR), j(Q_SIZE_WITHOUT_CLOCK_ERROR), k(0);
             k < Q_SIZE_CLOCK_ERROR;
             i++, j++, k++){

          /* Matrix layout (B)
           * [1] : row(j*2)   => clock(j) error
           * [1] : row(j*2+1) => clock(j) error rate
           */
          res.B[i][j] += 1;
        }
      }
    }

    /**
     * Time update
     *
     * @param accel acceleration
     * @param gyro angular speed
     * @param deltaT delta T
     */
    void update(const vec3_t &accel, const vec3_t &gyro, const float_t &deltaT){
      BaseFINS::update(accel, gyro, deltaT);
    }

    /**
     * Kalman By using correction values @f$ \Hat{x} @f$ estimated by filter, correct INS.
     *
     * @param x_hat correction values estimated by Kalman Filter
     */
    void correct_INS(mat_t &x_hat){
      for(unsigned i(P_SIZE_WITHOUT_CLOCK_ERROR), j(STATE_VALUES_WITHOUT_CLOCK_ERROR), k(0);
          k < STATE_VALUES_CLOCK_ERROR;
          i++, j++, k++){
        (*this)[j] -= x_hat(i, 0);
      }
      BaseFINS::correct_INS(x_hat);
    }

    using BaseFINS::correct_primitive;

    /**
     * Measurement update
     *
     * @param H Observation matrix
     * @param z Observation values
     * @param R Error covariance of z
     */
    void correct_primitive(const mat_t &H, const mat_t &z, const mat_t &R){
      BaseFINS::correct_primitive(H, z, R);
    }

    /*
     * Dummy function in case there is no correct function.
     * This is for "using super_class::correct" in the following INS_GPS2_Tightly
     */
    void correct_info();
    void correct();
};

template <class FloatT, class PVT_BaseT = typename GPS_Solver_Base<FloatT>::user_pvt_t>
struct GPS_Solution_PVT : public PVT_BaseT {
  unsigned int clock_index;

  static char (&check_pvt(typename GPS_Solver_Base<FloatT>::user_pvt_t *) )[1];
  static const int base_pvt_should_be_a_subclass_of_user_pvt_t
      = sizeof(check_pvt(static_cast<PVT_BaseT *>(0)));

  /**
   * PVT Converter for loosely integration
   * @return solution
   */
  operator GPS_Solution<FloatT> () const {
    GPS_Solution<FloatT> res;
    res.v_n = this->user_velocity_enu.north();
    res.v_e = this->user_velocity_enu.east();
    res.v_d = -this->user_velocity_enu.up();
    res.latitude = this->user_position.llh.latitude();
    res.longitude = this->user_position.llh.longitude();
    res.height = this->user_position.llh.height();
    // Calculation of estimated accuracy
    /* Position standard deviation is roughly estimated as (DOP * 2 meters)
     * @see https://www.gps.gov/systems/gps/performance/2016-GPS-SPS-performance-analysis.pdf Table 3.2
     */
    res.sigma_2d = this->dop.h * 2;
    res.sigma_height = this->dop.v * 2;
    // Speed standard deviation is roughly estimated as (DOP * 0.1 meter / seconds)
    res.sigma_vel = this->dop.p * 0.1;
    return res;
  }
};

template <class FloatT, class SolverT = GPS_Solver_Base<FloatT> >
struct GPS_RawData {
  typedef SolverT solver_t;
  const solver_t *solver;

  unsigned int clock_index;

  typedef typename solver_t::measurement_t measurement_t;
  measurement_t measurement;

  typename solver_t::prn_obs_t measurement_of(
      const typename measurement_t::mapped_type::key_type &key,
      const FloatT &scaling = FloatT(1)) const {
    return solver_t::measurement_util_t::gather(measurement, key, scaling);
  }

  typedef typename solver_t::gps_time_t gps_time_t;
  gps_time_t gpstime;

  GPS_RawData(const unsigned int &_clock_index = 0)
      : solver(NULL),
      clock_index(_clock_index), measurement(), gpstime() {}
  ~GPS_RawData(){}

  typedef GPS_Solution_PVT<FloatT, typename solver_t::user_pvt_t> pvt_t;

  pvt_t pvt(const pvt_t &hint = pvt_t()) const {
    pvt_t res;
    res.clock_index = clock_index;
    do{
      if(hint.error_code == pvt_t::ERROR_NO){
        FloatT delta_t(std::abs(gpstime - hint.receiver_time));
        if(delta_t < 5E-3){
          // already updated
          (typename solver_t::user_pvt_t &)res = hint;
          break;
        }else if(!solver){
          break;
        }else if(delta_t < 300){
          // Use hint, because solution may not be changed extremely in short time
          (typename solver_t::user_pvt_t &)res = solver->solve().user_pvt(
              measurement,
              gpstime,
              hint.user_position,
              hint.receiver_error);
          break;
        }
      }else if(!solver){
        break;
      }
      (typename solver_t::user_pvt_t &)res = solver->solve().user_pvt(
          measurement,
          gpstime);
    }while(false);
    return res;
  }
};

/**
 * @brief Tightly coupled INS/GPS
 *
 * Tightly coupled INS/GPS
 *
 * @param BaseFINS Filtered INS, which mainly supports for time update procedure.
 * @see Filtered_INS2
 */
template <
  class BaseFINS = Filtered_INS_ClockErrorEstimated<Filtered_INS2<
      INS_ClockErrorEstimated<INS<> > > >
>
class INS_GPS2_Tightly : public BaseFINS {
  public:
    typedef BaseFINS super_t;
#if defined(__GNUC__) && (__GNUC__ < 5)
    typedef typename super_t::float_t float_t;
    typedef typename super_t::vec3_t vec3_t;
    typedef typename super_t::quat_t quat_t;
    typedef typename super_t::mat_t mat_t;
#else
    using typename super_t::float_t;
    using typename super_t::vec3_t;
    using typename super_t::quat_t;
    using typename super_t::mat_t;
#endif
  public:
    INS_GPS2_Tightly() : super_t() {}

    INS_GPS2_Tightly(const INS_GPS2_Tightly &orig, const bool &deepcopy = false)
        : super_t(orig, deepcopy) {
    }

    ~INS_GPS2_Tightly(){}
    
    typedef INS_GPS2_Tightly<super_t> self_t;

    typedef GPS_Solver_Base<float_t> solver_t;
    typedef typename solver_t::space_node_t space_node_t;

    using super_t::CLOCKS_SUPPORTED;
    using super_t::P_SIZE;
    using super_t::property_t::P_SIZE_WITHOUT_CLOCK_ERROR;

  protected:
    struct receiver_state_t {
      typename solver_t::gps_time_t t;
      unsigned int clock_index;
      float_t clock_error;
      typename solver_t::pos_t pos;
      typename solver_t::xyz_t vel;
    };
    receiver_state_t receiver_state(
        const typename solver_t::gps_time_t &t,
        const unsigned int &clock_index,
        const float_t &clock_error_shift = 0) const {
      float_t clock_error(
          super_t::m_clock_error[clock_index] + clock_error_shift);
      receiver_state_t res = {
        t - clock_error / space_node_t::light_speed,
        clock_index,
        clock_error,
        {
          super_t::template position_xyz<typename solver_t::xyz_t>(),
          typename solver_t::llh_t(super_t::phi, super_t::lambda, super_t::h),
        },
        super_t::template velocity_xyz<typename solver_t::xyz_t>(),
      };
      return res;
    }

    struct relative_property_t : public solver_t::relative_property_t {
      float_t sigma_range;
      float_t rate_residual;
      float_t sigma_rate;
    };

    /**
     * Calculate relative property per satellite such as range and rate residuals
     *
     * @param solver residual calculator
     * @param prn GNSS satellite number used as target
     * @param measurement Measurement per satellite containg pseudorange and range rate
     * @param x receiver state represented by current position and clock properties
     * @return (relative_property_t) relative propertys
     */
    relative_property_t relative_property(
        const solver_t &solver,
        const typename solver_t::prn_t &prn,
        const typename solver_t::measurement_t::mapped_type &measurement,
        const receiver_state_t &x) const {

      relative_property_t res;
      res.sigma_range = res.sigma_rate = -1; // initialization with invalid value;

      const solver_t &solver_selected(solver.select(prn));
      (typename solver_t::relative_property_t &)res = solver_selected.relative_property(
          prn, measurement, x.clock_error, x.t, x.pos, x.vel);

      if(res.weight <= 0){return res;}

      if(!solver_selected.range_sigma(measurement, res.sigma_range)){
        // If receiver's range variance is not provided
        res.sigma_range = 1.0 / (res.weight < 1E-1 ? 1E-1 : res.weight); // TODO range error variance [m]
      }

      do{
        float_t rate;
        if(!solver_selected.rate(measurement, rate)){break;}
        res.rate_residual = rate
            - super_t::m_clock_error_rate[x.clock_index] + res.rate_relative_neg;

        if(!solver_selected.rate_sigma(measurement, res.sigma_rate)){
          // If receiver's rate variance is not provided
          res.sigma_rate = res.sigma_range * 1E-1; // TODO rate error variance [m/s]
        }
      }while(false);

      return res;
    }

    typedef std::vector<std::pair<typename solver_t::prn_t, relative_property_t> > relative_property_list_t;

    /**
     * Assign items of z, H and R of Kalman filter matrices based on range and rate residuals
     *
     * @param x receiver state represented by current position and clock properties
     * @param props relative properties
     * @return (CorrectInfo) z, H, R
     */
    CorrectInfo<float_t> assign_z_H_R(
        const receiver_state_t &x,
        const relative_property_list_t &props) const {

      struct buf_t {
        float_t *z;
        float_t (*H)[P_SIZE];
        float_t *R_diag;
        buf_t(const unsigned int &size)
            : z(new float_t [size * (P_SIZE + 2)]),
            R_diag(&z[size]), H((float_t (*)[P_SIZE])&z[size * 2]) {}
        ~buf_t(){
          delete [] z;
        }
      } buf(props.size() * 2); // range + rate

      float_t H_uh[3][4] = {{0}};
      {
#define pow2(x) ((x) * (x))
#define q_e2n(i) super_t::q_e2n.get(i)
        const float_t
            q_alpha((pow2(q_e2n(0)) + pow2(q_e2n(3))) * 2 - 1),
            q_beta((q_e2n(0) * q_e2n(1) - q_e2n(2) * q_e2n(3)) * 2),
            q_gamma((q_e2n(0) * q_e2n(2) + q_e2n(1) * q_e2n(3)) * 2);
        static const float_t &e(super_t::Earth::epsilon_Earth);
        const float_t n(super_t::Earth::R_e / std::sqrt(1.0 - pow2(e * q_alpha)));
        const float_t sf(n * pow2(e) * q_alpha * -2 / (1.0 - pow2(e) * pow2(q_alpha)));
        const float_t n_h((n + super_t::h) * 2);
#undef q_e2n
        H_uh[0][0] = -q_gamma * q_beta * sf;
        H_uh[0][1] = -pow2(q_gamma) * sf - n_h * q_alpha;
        H_uh[0][2] = -n_h * q_beta;
        H_uh[0][3] = -q_gamma;

        H_uh[1][0] = pow2(q_beta) * sf + n_h * q_alpha;
        H_uh[1][1] = q_beta * q_gamma * sf;
        H_uh[1][2] = -n_h * q_gamma;
        H_uh[1][3] = q_beta;

        const float_t sf2(sf * -(1.0 - pow2(e)));
        const float_t n_h2((n * (1.0 - pow2(e)) + super_t::h) * 2);
        H_uh[2][0] = q_alpha * q_beta * sf2 + n_h2 * q_beta;
        H_uh[2][1] = q_alpha * q_gamma * sf2 + n_h2 * q_gamma;
        H_uh[2][3] = -q_alpha;
      }
#undef pow2

      mat_t dcm_q_e2n_star(super_t::q_e2n.conj().getDCM());

      const float_t &vx(x.vel.x()), &vy(x.vel.y()), &vz(x.vel.z());

      // count up valid measurement, and make observation matrices
      // whose contents are packed by skipping unused (=zero weight) satellite observations.
      // Thus, there is order inconsistency between input(props) and output(CorrectInfo).
      int i_z(0);

      for(typename relative_property_list_t::const_iterator it(props.begin());
          it != props.end(); ++it){

        const relative_property_t &prop(it->second);

        { // range
          if(prop.sigma_range <= 0){continue;} // No measurement
          for(int j(0); j < P_SIZE; ++j){buf.H[i_z][j] = 0;} // zero clear

          for(int j(0), k(3); j < sizeof(H_uh[0]) / sizeof(H_uh[0][0]); ++j, ++k){
            for(int i(0); i < sizeof(prop.los_neg) / sizeof(prop.los_neg[0]); ++i){
              buf.H[i_z][k] -= prop.los_neg[i] * H_uh[i][j]; // polarity checked.
            }
          }
          buf.H[i_z][P_SIZE_WITHOUT_CLOCK_ERROR + (x.clock_index * 2)] = -1; // polarity checked.

          buf.z[i_z] = prop.range_residual;
          buf.R_diag[i_z] = std::pow(prop.sigma_range, 2);
          ++i_z;
        }

        { // rate
          if(prop.sigma_rate <= 0){continue;} // No rate measurement
          for(int j(0); j < P_SIZE; ++j){buf.H[i_z][j] = 0;} // zero clear

          { // velocity
            for(unsigned int j(0); j < dcm_q_e2n_star.columns(); ++j){
              for(int i(0); i < sizeof(prop.los_neg) / sizeof(prop.los_neg[0]); ++i){
                buf.H[i_z][j] -= prop.los_neg[i] * dcm_q_e2n_star(i, j); // polarity checked.
              }
            }
          }
          { // position
            buf.H[i_z][3] -= (                        prop.los_neg[1] * -vz + prop.los_neg[2] *  vy) * 2;  // polarity checked.
            buf.H[i_z][4] -= (prop.los_neg[0] *  vz                         + prop.los_neg[2] * -vx) * 2;
            buf.H[i_z][5] -= (prop.los_neg[0] * -vy + prop.los_neg[1] *  vx                        ) * 2;
          }
          // error rate
          buf.H[i_z][P_SIZE_WITHOUT_CLOCK_ERROR + (x.clock_index * 2) + 1] = -1; // polarity checked.

          buf.z[i_z] = prop.rate_residual;
          buf.R_diag[i_z] = std::pow(prop.sigma_rate, 2);
          ++i_z;
        }
      }

      if(i_z <= 0){return CorrectInfo<float_t>::no_info();}

      mat_t H(i_z, P_SIZE, (float_t *)buf.H);
      mat_t z(i_z, 1, buf.z);
      mat_t R(i_z, i_z);
      for(int i(0); i < i_z; ++i){
        R(i, i) = buf.R_diag[i];
      }

      return CorrectInfo<float_t>(H, z, R);
    }

    /**
     * Calculate DOP
     *
     * @param x receiver state represented by current position and clock properties
     * @param props relative properties
     * @param res buffer of calculation result
     * @return (dop_t *) unavailable when null; otherwise, DOP
     */
    typename solver_t::user_pvt_t::dop_t *get_DOP(
        const receiver_state_t &x,
        const relative_property_list_t &props,
        typename solver_t::user_pvt_t::dop_t &res) const {

      mat_t G_full(props.size(), 4); // design matrix

      // count up valid measurement
      int i_row(0);
      for(typename relative_property_list_t::const_iterator it(props.begin());
          it != props.end(); ++it){

        const relative_property_t &prop(it->second);

        if(prop.sigma_range <= 0){continue;} // No measurement
        for(int i(0); i < sizeof(prop.los_neg) / sizeof(prop.los_neg[0]); ++i){
          G_full(i_row, i) = prop.los_neg[i];
        }
        G_full(i_row, 3) = 1;
        ++i_row;
      }
      if(i_row < 4){return NULL;}

      typename mat_t::partial_offsetless_t G(G_full.partial(i_row, 4));
      return &(res = solver_t::user_pvt_t::dop_t::get(
          (G.transpose() * G).inverse(), x.pos));
    }

    /**
     * Placeholder to filter relative properties to be used for correction
     *
     * @param x receiver state
     * @param measurement measurement used for calculation of relative properties (props)
     * @param props source relative properties to be filtered
     * @return filtered relative properties. This function is just a placeholder, and return props itself.
     */
    virtual relative_property_list_t &filter_relative_properties(
        const receiver_state_t &x,
        const typename solver_t::measurement_t &measurement,
        relative_property_list_t &props) const {

      /* TODO
       * Control whether use or not use specific measurement
       * by regulating props[n].sigma_(range|rate), whose negative or zero value yields
       * intentional exclusion.
       */
      /*{ // check DOP
        typename solver_t::user_pvt_t::dop_t dop;
        if(get_DOP(x, props, dop)){
          // do something
          std::cerr << dop.t << std::endl;
        }
      }*/
      return props;
    }

  public:
    using super_t::correct_info;

    /**
     * Calculate information required for measurement update
     *
     * @param gps GPS observation mainly consisting of range (and rate, if available)
     * @param clock_error_shift forcefully shifting value of clock error in meter,
     * which will be used when receiver clock error exceeds predefined threshold
     * of allowable delta from true GPS time. Normally it is +/- (1 ms * speed of light).
     */
    template <class SolverT>
    CorrectInfo<float_t> correct_info(
        const GPS_RawData<float_t, SolverT> &gps,
        const float_t &clock_error_shift = 0) const {

      if(gps.clock_index >= CLOCKS_SUPPORTED){return CorrectInfo<float_t>::no_info();}

      // check space_node is configured
      if(!gps.solver){return CorrectInfo<float_t>::no_info();}

      receiver_state_t x(receiver_state(gps.gpstime, gps.clock_index, clock_error_shift));

      relative_property_list_t props;
      props.reserve(gps.measurement.size());

      for(typename solver_t::measurement_t::const_iterator it(gps.measurement.begin());
          it != gps.measurement.end(); ++it){

        props.push_back(std::make_pair(
            it->first, relative_property(*gps.solver, it->first, it->second, x)));
      }

      return assign_z_H_R(x, filter_relative_properties(x, gps.measurement, props));
    }

    template <class SolverT>
    CorrectInfo<float_t> correct_info(
        const GPS_RawData<float_t, SolverT> &gps,
        const vec3_t &lever_arm_b, const vec3_t &omega_b2i_4b,
        const float_t &clock_error_shift = 0) const {
      // TODO
      return correct_info(gps, clock_error_shift);
    }

  protected:
    float_t range_residual_mean_ms(
        const unsigned int &clock_index,
        const CorrectInfo<float_t> &info) const {

      float_t range_residual_sum(0);
      unsigned int z_ranges(0);

      for(unsigned int i(0); i < info.z.rows(); i++){
        if(info.H(i, P_SIZE_WITHOUT_CLOCK_ERROR + (clock_index * 2)) > -0.5){continue;}
        range_residual_sum += info.z(i, 0);
        z_ranges++;
      }

      return (z_ranges > 0)
          ? (range_residual_sum / z_ranges / space_node_t::light_speed / 1E-3)
          : 0;
    }

    struct CorrectInfoGenerator {
      const vec3_t *lever_arm_b;
      const vec3_t *omega_b2i_4b;
      CorrectInfoGenerator(const vec3_t *lever = NULL, const vec3_t *omega = NULL)
          : lever_arm_b(lever), omega_b2i_4b(omega) {}
      template <class ObservationT>
      CorrectInfo<float_t> operator()(
          const self_t &self, const ObservationT &gps,
          const float_t &clock_error_shift = 0) const {
        return lever_arm_b
            ? self.correct_info(gps, *lever_arm_b, *omega_b2i_4b, clock_error_shift)
            : self.correct_info(gps, clock_error_shift);
      }
    };

    template <class ObservationT>
    void correct_with_clock_jump_check(
        const ObservationT &gps, const CorrectInfoGenerator &generator){
      CorrectInfo<float_t> info(generator(*this, gps));
      if(info.z.rows() < 1){return;}

      /*
       * detection for automatic receiver clock error correction,
       * which will be performed to adjust clock in its maximum error between +/- 1ms.
       */
      float_t delta_ms(range_residual_mean_ms(gps.clock_index, info));
      if((delta_ms >= 0.9) || (delta_ms <= -0.9)){ // 0.9 ms
        std::cerr << "Detect receiver clock jump: " << delta_ms << " [ms] => ";
        float_t clock_error_shift(
            space_node_t::light_speed * 1E-3 * std::floor(delta_ms + 0.5));
        info = generator(*this, gps, clock_error_shift);
        delta_ms = range_residual_mean_ms(gps.clock_index, info);
        if((delta_ms < 0.9) && (delta_ms > -0.9)){
          std::cerr << "Fixed." << std::endl;
          // correct internal clock error
          super_t::m_clock_error[gps.clock_index] += clock_error_shift;
        }else{
          std::cerr << "Skipped." << std::endl;
          return; // unknown error!
        }
      }

      super_t::correct_primitive(info);
    }

  public:
    using super_t::correct;

    /**
     * Measurement update with GPS raw measurement
     *
     * @param gps GPS measurement
     */
    template <class SolverT>
    void correct(const GPS_RawData<float_t, SolverT> &gps){
      correct_with_clock_jump_check(gps, CorrectInfoGenerator());
    }

    /**
     * Measurement update with GPS raw measurement and lever arm effect
     *
     * @param gps GPS measurement
     * @param lever_arm_b lever arm vector in b-frame
     * @param omega_b2i_4b angular speed vector in b-frame
     */
    template <class SolverT>
    void correct(const GPS_RawData<float_t, SolverT> &gps,
        const vec3_t &lever_arm_b,
        const vec3_t &omega_b2i_4b){
      correct_with_clock_jump_check(gps, CorrectInfoGenerator(&lever_arm_b, &omega_b2i_4b));
    }

    // { // PVT (loosely) interface
    template <class PVT_BaseT>
    CorrectInfo<float_t> correct_info_pvt(
        const GPS_Solution_PVT<float_t, PVT_BaseT> &,
        const float_t &,
        void *,
        const vec3_t * = NULL, const vec3_t * = NULL) const {
      return CorrectInfo<float_t>::no_info();
    }
    template <class PVT_BaseT, class BaseFINS2>
    CorrectInfo<float_t> correct_info_pvt(
        const GPS_Solution_PVT<float_t, PVT_BaseT> &pvt,
        const float_t &clock_error_shift,
        const INS_GPS2<BaseFINS2> *,
        const vec3_t *lever_arm_b = NULL, const vec3_t *omega_b2i_4b = NULL) const {
      if((pvt.error_code != solver_t::user_pvt_t::ERROR_NO)
          || (pvt.clock_index >= CLOCKS_SUPPORTED)){
        return CorrectInfo<float_t>::no_info();
      }
      // If super class has loosely-coupled interface, PVT input is acceptable.
      CorrectInfo<float_t> info_loosely(lever_arm_b
          ? super_t::correct_info((GPS_Solution<float_t>)pvt, *lever_arm_b, *omega_b2i_4b)
          : super_t::correct_info((GPS_Solution<float_t>)pvt));

      // expand H, z, R rows to include clock and clock rate residual
      unsigned int rows_orig(info_loosely.z.rows()), rows_new(rows_orig + 2);
      mat_t H(rows_new, info_loosely.H.columns()), z(rows_new, 1), R(rows_new, rows_new);
      H.pivotMerge(0, 0, info_loosely.H);
      z.pivotMerge(0, 0, info_loosely.z);
      R.pivotMerge(0, 0, info_loosely.R);

      // fill clock (H = -1 because of correspondence of tightly)
      H(rows_new - 2, P_SIZE_WITHOUT_CLOCK_ERROR + (pvt.clock_index * 2)) = -1;
      z(rows_new - 2, 0)
          = pvt.receiver_error
            - (super_t::m_clock_error[pvt.clock_index] + clock_error_shift);
      R(rows_new - 2, rows_new - 2) = 1E1; // TODO

      // fill clock rate (H = -1 because of correspondence of tightly)
      H(rows_new - 1, P_SIZE_WITHOUT_CLOCK_ERROR + (pvt.clock_index * 2) + 1) = -1;
      z(rows_new - 1, 0)
          = pvt.receiver_error_rate
            - super_t::m_clock_error_rate[pvt.clock_index];
      R(rows_new - 1, rows_new - 1) = 1E-1; // TODO

      return CorrectInfo<float_t>(H, z, R);
    }
    template <class PVT_BaseT>
    CorrectInfo<float_t> correct_info(
        const GPS_Solution_PVT<float_t, PVT_BaseT> &pvt,
        const float_t &clock_error_shift = 0) const {
      return correct_info_pvt(pvt, clock_error_shift, this);
    }
    template <class PVT_BaseT>
    CorrectInfo<float_t> correct_info(
        const GPS_Solution_PVT<float_t, PVT_BaseT> &pvt,
        const vec3_t &lever_arm_b, const vec3_t &omega_b2i_4b,
        const float_t &clock_error_shift = 0) const {
      return correct_info_pvt(pvt, clock_error_shift, this,
          &lever_arm_b, &omega_b2i_4b);
    }
    template <class PVT_BaseT>
    void correct(
        const GPS_Solution_PVT<float_t, PVT_BaseT> &pvt){
      correct_with_clock_jump_check(
          pvt, CorrectInfoGenerator());
    }
    template <class PVT_BaseT>
    void correct(
        const GPS_Solution_PVT<float_t, PVT_BaseT> &pvt,
        const vec3_t &lever_arm_b, const vec3_t &omega_b2i_4b){
      correct_with_clock_jump_check(
          pvt, CorrectInfoGenerator(&lever_arm_b, &omega_b2i_4b));
    }
    // } // PVT (loosely) interface
};

#endif /* __INS_GPS2_TIGHTLY_H__ */
