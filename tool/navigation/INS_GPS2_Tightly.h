/*
 *  INS_GPS2.h, header file to perform calculation of integration of INS and GPS.
 *  Copyright (C) 2015 M.Naruoka (fenrir)
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

#include "INS.h"
#include "Filtered_INS2.h"
#include "INS_GPS2.h"
#include "GPS_SP.h"

template <typename BaseINS>
class INS_ClockErrorEstimated;

template <class BaseINS>
struct INS_Property<INS_ClockErrorEstimated<BaseINS> > {
  static const unsigned STATE_VALUES_WITHOUT_CLOCK_ERROR = INS_Property<BaseINS>::STATE_VALUES;
  static const unsigned STATE_VALUES_CLOCK_ERROR = 1;
  static const unsigned STATE_VALUES = STATE_VALUES_WITHOUT_CLOCK_ERROR + STATE_VALUES_CLOCK_ERROR;
};

template <
    typename BaseINS = INS<> >
class INS_ClockErrorEstimated : public BaseINS {
  public:
#if defined(__GNUC__) && (__GNUC__ < 5)
    typedef typename BaseINS::float_t float_t;
#else
    using typename BaseINS::float_t;
#endif

  protected:
    float_t m_clock_error;

  public:
    static const unsigned STATE_VALUES_WITHOUT_CLOCK_ERROR
        = INS_Property<INS_ClockErrorEstimated<BaseINS> >::STATE_VALUES_WITHOUT_CLOCK_ERROR;
    static const unsigned STATE_VALUES_CLOCK_ERROR
        = INS_Property<INS_ClockErrorEstimated<BaseINS> >::STATE_VALUES_CLOCK_ERROR;
    static const unsigned STATE_VALUES
        = INS_Property<INS_ClockErrorEstimated<BaseINS> >::STATE_VALUES; ///< Number of state values
    virtual unsigned state_values() const {return STATE_VALUES;}

    INS_ClockErrorEstimated()
        : BaseINS(),
          m_clock_error(0) {
    }

    INS_ClockErrorEstimated(const INS_ClockErrorEstimated &orig, const bool deepcopy = false)
        : BaseINS(orig, deepcopy),
          m_clock_error(orig.m_clock_error) {
    }

    virtual ~INS_ClockErrorEstimated(){}

    float_t &clock_error(){return m_clock_error;}

    using BaseINS::operator[];

    const float_t &operator[](const unsigned &index) const {
      switch(index){
        case STATE_VALUES_WITHOUT_CLOCK_ERROR: return m_clock_error;
        default: return BaseINS::operator[](index);
      }
    }
};

template <class BaseINS>
class Filtered_INS2_Property<INS_ClockErrorEstimated<BaseINS> >
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
    static const unsigned P_SIZE
#if defined(_MSC_VER)
        = P_SIZE_WITHOUT_CLOCK_ERROR + INS_ClockErrorEstimated<BaseINS>::STATE_VALUES_CLOCK_ERROR
#endif
        ;
    static const unsigned Q_SIZE
#if defined(_MSC_VER)
        = Q_SIZE_WITHOUT_CLOCK_ERROR + INS_ClockErrorEstimated<BaseINS>::STATE_VALUES_CLOCK_ERROR
#endif
        ;
};

#if !defined(_MSC_VER)
template <class BaseINS>
const unsigned Filtered_INS2_Property<INS_ClockErrorEstimated<BaseINS> >::P_SIZE_WITHOUT_CLOCK_ERROR
    = Filtered_INS2_Property<BaseINS>::P_SIZE;

template <class BaseINS>
const unsigned Filtered_INS2_Property<INS_ClockErrorEstimated<BaseINS> >::Q_SIZE_WITHOUT_CLOCK_ERROR
    = Filtered_INS2_Property<BaseINS>::Q_SIZE;

template <class BaseINS>
const unsigned Filtered_INS2_Property<INS_ClockErrorEstimated<BaseINS> >::P_SIZE
    = P_SIZE_WITHOUT_CLOCK_ERROR + INS_ClockErrorEstimated<BaseINS>::STATE_VALUES_CLOCK_ERROR;

template <class BaseINS>
const unsigned Filtered_INS2_Property<INS_ClockErrorEstimated<BaseINS> >::Q_SIZE
    = Q_SIZE_WITHOUT_CLOCK_ERROR + INS_ClockErrorEstimated<BaseINS>::STATE_VALUES_CLOCK_ERROR;
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

  public:
    using BaseFINS::ins_t::STATE_VALUES_WITHOUT_CLOCK_ERROR;
    using BaseFINS::ins_t::STATE_VALUES_CLOCK_ERROR;
    using BaseFINS::property_t::P_SIZE_WITHOUT_CLOCK_ERROR;
    using BaseFINS::property_t::Q_SIZE_WITHOUT_CLOCK_ERROR;
    using BaseFINS::m_clock_error;

    Filtered_INS_ClockErrorEstimated()
        : BaseFINS(),
        m_beta_clock_error(1){

    }

    Filtered_INS_ClockErrorEstimated(
        const Filtered_INS_ClockErrorEstimated &orig,
        const bool deepcopy = false) :
      BaseFINS(orig, deepcopy),
      m_beta_clock_error(orig.m_beta_clock_error){

    }

    ~Filtered_INS_ClockErrorEstimated(){}

    float_t &beta_clock_error(){return m_beta_clock_error;}

    void getAB(
        const vec3_t &accel,
        const vec3_t &gyro,
        typename BaseFINS::getAB_res &res) const {

      BaseFINS::getAB(accel, gyro, res);

      { // A matrix modification
        // Copy from part of B associated with clock error
        for(unsigned i(0); i < P_SIZE_WITHOUT_CLOCK_ERROR; i++){
          for(unsigned j(P_SIZE_WITHOUT_CLOCK_ERROR), k(0); k < STATE_VALUES_CLOCK_ERROR; j++, k++){
            res.A[i][j] = res.B[i][k];
          }
        }
        // Modify part of A associated with clock error
        for(unsigned i(P_SIZE_WITHOUT_CLOCK_ERROR), j(0); j < STATE_VALUES_CLOCK_ERROR; i++, j++){
          res.A[i][i] += -m_beta_clock_error;
        }
      }

      { // B matrix modification
        for(unsigned i(P_SIZE_WITHOUT_CLOCK_ERROR), j(Q_SIZE_WITHOUT_CLOCK_ERROR), k(0);
             k < STATE_VALUES_CLOCK_ERROR;
             i++, j++, k++){
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
};

template <class FloatT>
struct GPS_RawData {
  typedef GPS_SpaceNode<FloatT> space_node_t;
  const space_node_t &space_node;

  typedef GPS_SinglePositioning<float_sylph_t> solver_t;
  const solver_t solver;

  enum {
    L1_PSEUDORANGE,
    L1_RANGERATE,
    L1_CARRIER_PHASE,
  };
  typedef std::vector<std::pair<int, float_sylph_t> > prn_obs_t;
  typedef std::map<int, prn_obs_t> measurement_t;
  measurement_t measurement;

  GPS_RawData(const space_node_t &_space_node)
      : space_node(_space_node),
      solver(space_node), measurement() {}
  ~GPS_RawData(){}
};

/**
 * @brief Tightly coupled INS/GPS
 *
 * Tightly coupled INS/GPS
 *
 * @param FloatT precision
 * @param Filter type of Kalman filter
 * @param BaseFINS Filtered INS, which mainly supports for time update procedure.
 * @see Filtered_INS2
 */
template <
  class FloatT,
  template <class> class Filter = KalmanFilterUD,
  typename BaseFINS = Filtered_INS_ClockErrorEstimated<
      Filtered_INS2<INS_ClockErrorEstimated<INS<FloatT> >, Filter> >
>
class INS_GPS2_Tightly : public BaseFINS{
  public:
#if defined(__GNUC__) && (__GNUC__ < 5)
    typedef typename BaseFINS::float_t float_t;
    typedef typename BaseFINS::vec3_t vec3_t;
    typedef typename BaseFINS::quat_t quat_t;
    typedef typename BaseFINS::mat_t mat_t;
#else
    using typename BaseFINS::float_t;
    using typename BaseFINS::vec3_t;
    using typename BaseFINS::quat_t;
    using typename BaseFINS::mat_t;
#endif
  public:
    INS_GPS2_Tightly() : BaseFINS(){}

    INS_GPS2_Tightly(const INS_GPS2_Tightly &orig, const bool deepcopy = false) :
      BaseFINS(orig, deepcopy){

    }

    ~INS_GPS2_Tightly(){}
    
    typedef GPS_RawData<float_t> raw_data_t;

    CorrectInfo<float_t> correct_info(const GPS_RawData<float_t> &gps) const {

      mat_t H;
      mat_t z;
      mat_t R;

      return CorrectInfo<float_t>(H, z, R);
    }

    /**
     * Measurement update with GPS raw measurement
     *
     * @param gps GPS measurement
     */
    void correct(const GPS_RawData<float_t> &gps){
      BaseFINS::correct_primitive(correct_info(gps));
    }


    CorrectInfo<float_t> correct_info(const GPS_RawData<float_t> &gps,
        const vec3_t &lever_arm_b,
        const vec3_t &omega_b2i_4b) const {
      return correct_info(gps);
    }

    /**
     * Measurement update with GPS raw measurement and lever arm effect
     *
     * @param gps GPS measurement
     * @param lever_arm_b lever arm vector in b-frame
     * @param omega_b2i_4b angular speed vector in b-frame
     */
    void correct(const GPS_RawData<float_t> &gps,
        const vec3_t &lever_arm_b,
        const vec3_t &omega_b2i_4b){
      BaseFINS::correct_primitive(correct_info(gps, lever_arm_b, omega_b2i_4b));
    }
};

#endif /* __INS_GPS2_TIGHTLY_H__ */
