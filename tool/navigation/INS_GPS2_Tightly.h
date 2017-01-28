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
    static const unsigned STATE_VALUES_WITHOUT_CLOCK_ERROR = BaseINS::STATE_VALUES;
    static const unsigned STATE_VALUES_CLOCK_ERROR = 1;
    static const unsigned STATE_VALUES; ///< Number of state values
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

template <
    typename BaseINS>
const unsigned INS_ClockErrorEstimated<BaseINS>::STATE_VALUES
    = STATE_VALUES_WITHOUT_CLOCK_ERROR + STATE_VALUES_CLOCK_ERROR;

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
        = P_SIZE_WITHOUT_CLOCK_ERROR + INS_ClockErrorEstimated<BaseINS>::STATE_VALUES_BIAS
#endif
        ;
    static const unsigned Q_SIZE
#if defined(_MSC_VER)
        = Q_SIZE_WITHOUT_CLOCK_ERROR + INS_ClockErrorEstimated<BaseINS>::STATE_VALUES_BIAS;
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

    /**
     * コピーコンストラクタ
     *
     * @param orig コピー元
     * @param deepcopy ディープコピーを作成するかどうか
     */
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

      { // A行列の修正
        // B行列の加速度、角速度に関する項をA行列へ
        for(unsigned i(0); i < P_SIZE_WITHOUT_CLOCK_ERROR; i++){
          for(unsigned j(P_SIZE_WITHOUT_CLOCK_ERROR), k(0); k < STATE_VALUES_CLOCK_ERROR; j++, k++){
            res.A[i][j] = res.B[i][k];
          }
        }
        // A行列のバイアスに関する対角成分を埋める
        for(unsigned i(P_SIZE_WITHOUT_CLOCK_ERROR), j(0); j < STATE_VALUES_CLOCK_ERROR; i++, j++){
          res.A[i][i] += -m_beta_clock_error;
        }
      }

      { // B行列の修正
        for(unsigned i(P_SIZE_WITHOUT_CLOCK_ERROR), j(Q_SIZE_WITHOUT_CLOCK_ERROR), k(0);
             k < STATE_VALUES_CLOCK_ERROR;
             i++, j++, k++){
          res.B[i][j] += 1;
        }
      }
    }

    /**
     * 時間更新
     *
     * @param accel 加速度計の値
     * @param gyro ジャイロの値
     * @param deltaT 時間間隔
     */
    void update(const vec3_t &accel, const vec3_t &gyro, const float_t &deltaT){
      BaseFINS::update(accel, gyro, deltaT);
    }

    /**
     * Kalman Filterによって得られた@f$ \Hat{x} @f$を利用して、INSを修正します。
     *
     * @param x_hat Kalman Filterによって得られたx_hat
     */
    void correct_INS(mat_t &x_hat){
      for(unsigned i(P_SIZE_WITHOUT_CLOCK_ERROR), j(STATE_VALUES_WITHOUT_CLOCK_ERROR), k(0);
          k < STATE_VALUES_CLOCK_ERROR;
          i++, j++, k++){
        (*this)[j] -= x_hat(i, 0);
      }
      BaseFINS::correct_INS(x_hat);
    }

    /**
     * 修正します。
     *
     * @param H 観測行列
     * @param z 観測量
     * @param R 誤差共分散行列
     */
    void correct_primitive(const mat_t &H, const mat_t &z, const mat_t &R){
      BaseFINS::correct_primitive(H, z, R);
    }
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
      Filtered_INS2<INS_ClockErrorEstimated<INS<> >, Filter> >
>
class INS_GPS2_Tightly : public BaseFINS{
  public:
    INS_GPS2_Tightly() : BaseFINS(){} /**< コンストラクタ */

    /**
     * コピーコンストラクタ
     *
     * @param orig コピー元
     * @param deepcopy ディープコピーを作成するかどうか
     */
    INS_GPS2_Tightly(const INS_GPS2_Tightly &orig, const bool deepcopy = false) :
      BaseFINS(orig, deepcopy){

    }

    ~INS_GPS2_Tightly(){}         /**< デストラクタ */
};

#endif /* __INS_GPS2_TIGHTLY_H__ */
