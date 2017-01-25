/*
 *  Filtered_INS_BE.h, header file to perform calculation of INS with bias drift estimation.
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

#ifndef __FILTERED_INS_BE_H__
#define __FILTERED_INS_BE_H__


#include "param/matrix.h"
#include "navigation/Filtered_INS2.h"
#include "algorithm/kalman.h"

#define BIAS_EST_MODE 2 // 初期ゼロ点推定含む
#define ZERO_CONST_ALPHA 0 //(FloatT(1) / 1E4) // 0

#ifndef BIAS_EST_MODE
#define BIAS_EST_MODE 0
#endif

template <
    typename BaseINS = INS<> >
class INS_BiasEstimated : public BaseINS {
  protected:
    typename BaseINS::vec3_t m_bias_accel, m_bias_gyro;

  public:
    static const unsigned STATE_VALUES_WITHOUT_BIAS = BaseINS::STATE_VALUES;
    static const unsigned STATE_VALUES_BIAS = 6;
    static const unsigned STATE_VALUES; ///< 状態量の数
    virtual unsigned state_values() const {return STATE_VALUES;}

    INS_BiasEstimated()
        : BaseINS(),
          m_bias_accel(), m_bias_gyro() {
    }

    INS_BiasEstimated(const INS_BiasEstimated &orig, const bool deepcopy = false)
        : BaseINS(orig, deepcopy),
          m_bias_accel(deepcopy ? orig.m_bias_accel.copy() : orig.m_bias_accel),
          m_bias_gyro(deepcopy ? orig.m_bias_gyro.copy() : orig.m_bias_gyro) {
    }

    virtual ~INS_BiasEstimated(){}

    typename BaseINS::vec3_t &bias_accel(){return m_bias_accel;}
    typename BaseINS::vec3_t &bias_gyro(){return m_bias_gyro;}

    typename BaseINS::float_t &operator[](const unsigned &index){
      switch(index){
        case STATE_VALUES_WITHOUT_BIAS:     return m_bias_accel[0];
        case STATE_VALUES_WITHOUT_BIAS + 1: return m_bias_accel[1];
        case STATE_VALUES_WITHOUT_BIAS + 2: return m_bias_accel[2];
        case STATE_VALUES_WITHOUT_BIAS + 3: return m_bias_gyro[0];
        case STATE_VALUES_WITHOUT_BIAS + 4: return m_bias_gyro[1];
        case STATE_VALUES_WITHOUT_BIAS + 5: return m_bias_gyro[2];
        default: return BaseINS::operator[](index);
      }
    }

    /**
     * 時間更新
     *
     * @param accel 加速度計の値
     * @param gyro ジャイロの値
     * @param deltaT 時間間隔
     */
    void update(
        const typename BaseINS::vec3_t &accel, const typename BaseINS::vec3_t &gyro,
        const typename BaseINS::float_t &deltaT){
      // バイアス除去して航法演算
      BaseINS::update(accel + m_bias_accel, gyro + m_bias_gyro, deltaT);
    }
};

template <
    typename BaseINS>
const unsigned INS_BiasEstimated<BaseINS>::STATE_VALUES
    = STATE_VALUES_WITHOUT_BIAS + STATE_VALUES_BIAS;

template <class FloatT, class BaseINS>
class Filtered_INS2_Property<FloatT, INS_BiasEstimated<BaseINS> >
    : public Filtered_INS2_Property<FloatT, BaseINS> {
  public:
    static const unsigned P_SIZE_WITHOUT_BIAS
#if defined(_MSC_VER)
        = Filtered_INS2_Property<FloatT, BaseINS>::P_SIZE
#endif
        ;
    static const unsigned Q_SIZE_WITHOUT_BIAS
#if defined(_MSC_VER)
        = Filtered_INS2_Property<FloatT, BaseINS>::Q_SIZE
#endif
        ;
    static const unsigned P_SIZE
#if defined(_MSC_VER)
        = P_SIZE_WITHOUT_BIAS + INS_BiasEstimated<BaseINS>::STATE_VALUES_BIAS
#endif
        ;
    static const unsigned Q_SIZE
#if defined(_MSC_VER)
        = Q_SIZE_WITHOUT_BIAS + INS_BiasEstimated<BaseINS>::STATE_VALUES_BIAS;
#endif
        ;
};

#if !defined(_MSC_VER)
template <class FloatT, class BaseINS>
const unsigned Filtered_INS2_Property<FloatT, INS_BiasEstimated<BaseINS> >::P_SIZE_WITHOUT_BIAS
    = Filtered_INS2_Property<FloatT, BaseINS>::P_SIZE;

template <class FloatT, class BaseINS>
const unsigned Filtered_INS2_Property<FloatT, INS_BiasEstimated<BaseINS> >::Q_SIZE_WITHOUT_BIAS
    = Filtered_INS2_Property<FloatT, BaseINS>::Q_SIZE;

template <class FloatT, class BaseINS>
const unsigned Filtered_INS2_Property<FloatT, INS_BiasEstimated<BaseINS> >::P_SIZE
    = P_SIZE_WITHOUT_BIAS + INS_BiasEstimated<BaseINS>::STATE_VALUES_BIAS;

template <class FloatT, class BaseINS>
const unsigned Filtered_INS2_Property<FloatT, INS_BiasEstimated<BaseINS> >::Q_SIZE
    = Q_SIZE_WITHOUT_BIAS + INS_BiasEstimated<BaseINS>::STATE_VALUES_BIAS;
#endif


template <
    class FloatT,
    template <class> class Filter = KalmanFilterUD,
    typename BaseFINS = Filtered_INS2<FloatT, Filter, INS_BiasEstimated<INS<FloatT> > > >
class Filtered_INS_BiasEstimated : public BaseFINS{
  protected:
    Vector3<FloatT> m_beta_accel, m_beta_gyro; // ベータの修正を!!
    FloatT m_deltaT_sum;

#if BIAS_EST_MODE == 1
    Vector3<FloatT> previous_modified_bias_accel;
    Vector3<FloatT> previous_modified_bias_gyro;
#elif BIAS_EST_MODE == 2
    FloatT previous_delteT_sum;
    Vector3<FloatT> drift_bias_accel;
    Vector3<FloatT> drift_bias_gyro;
#endif
    
  public:
    using BaseFINS::ins_t::STATE_VALUES_WITHOUT_BIAS;
    using BaseFINS::ins_t::STATE_VALUES_BIAS;
    using BaseFINS::property_t::P_SIZE_WITHOUT_BIAS;
    using BaseFINS::property_t::Q_SIZE_WITHOUT_BIAS;
    using BaseFINS::m_bias_accel;
    using BaseFINS::m_bias_gyro;
    
    Filtered_INS_BiasEstimated() 
        : BaseFINS(),
        m_beta_accel(FloatT(1), FloatT(1), FloatT(1)),
        m_beta_gyro(FloatT(1), FloatT(1), FloatT(1)),
        m_deltaT_sum(0)
#if BIAS_EST_MODE == 1
        , previous_modified_bias_accel()
        , previous_modified_bias_gyro()
#elif BIAS_EST_MODE == 2
        , previous_delteT_sum(0)
        , drift_bias_accel()
        , drift_bias_gyro()
#endif
        {
      
      // 共分散行列の初期化
    }
    
    /**
     * コピーコンストラクタ
     * 
     * @param orig コピー元
     * @param deepcopy ディープコピーを作成するかどうか
     */
    Filtered_INS_BiasEstimated(
        const Filtered_INS_BiasEstimated &orig, 
        const bool deepcopy = false) :
      BaseFINS(orig, deepcopy),
      m_beta_accel(deepcopy ? orig.m_beta_accel.copy() : orig.m_beta_accel),
      m_beta_gyro(deepcopy ? orig.m_beta_gyro.copy() : orig.m_beta_gyro),
      m_deltaT_sum(orig.m_deltaT_sum)
#if BIAS_EST_MODE == 1
      , previous_modified_bias_accel(deepcopy 
          ? orig.previous_modified_bias_accel.copy() 
          : orig.previous_modified_bias_accel)
      , previous_modified_bias_gyro(deepcopy 
          ? orig.previous_modified_bias_gyro.copy() 
          : orig.previous_modified_bias_gyro)
#elif BIAS_EST_MODE == 2
      , previous_delteT_sum(orig.previous_delteT_sum)
      , drift_bias_accel(deepcopy 
          ? orig.drift_bias_accel.copy() 
          : orig.drift_bias_accel)
      , drift_bias_gyro(deepcopy 
          ? orig.drift_bias_gyro.copy() 
          : orig.drift_bias_gyro)
#endif      
      {
      
    }
    
    ~Filtered_INS_BiasEstimated(){}

    Vector3<FloatT> &beta_accel(){return m_beta_accel;}
    Vector3<FloatT> &beta_gyro(){return m_beta_gyro;}

    void getAB(
        const Vector3<FloatT> &accel,
        const Vector3<FloatT> &gyro,
        typename BaseFINS::getAB_res &res) const {
      
      BaseFINS::getAB(accel, gyro, res);

      { // A行列の修正
        // B行列の加速度、角速度に関する項をA行列へ
        for(unsigned i(0); i < P_SIZE_WITHOUT_BIAS; i++){
          for(unsigned j(P_SIZE_WITHOUT_BIAS), k(0); k < STATE_VALUES_BIAS; j++, k++){
            res.A[i][j] = res.B[i][k];
          }
        }
        // A行列のバイアスに関する対角成分を埋める
        for(unsigned i(P_SIZE_WITHOUT_BIAS), j(0); j < STATE_VALUES_BIAS; i++, j++){
          res.A[i][i] += -((j < Vector3<FloatT>::OUT_OF_INDEX)
              ? m_beta_accel.get(j)
              : m_beta_gyro.get(j - (Vector3<FloatT>::OUT_OF_INDEX)));
        }
      }
      
      { // B行列の修正
        for(unsigned i(P_SIZE_WITHOUT_BIAS), j(Q_SIZE_WITHOUT_BIAS), k(0);
             k < STATE_VALUES_BIAS;
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
    void update(const Vector3<FloatT> &accel, const Vector3<FloatT> &gyro, const FloatT &deltaT){
      // 時間を足す
      m_deltaT_sum += deltaT;
      BaseFINS::update(accel, gyro, deltaT);
    }
    
    /**
     * Kalman Filterによって得られた@f$ \Hat{x} @f$を利用して、INSを修正します。
     *
     * @param x_hat Kalman Filterによって得られたx_hat
     */
    void correct_INS(Matrix<FloatT> &x_hat){
      for(unsigned i(P_SIZE_WITHOUT_BIAS), j(STATE_VALUES_WITHOUT_BIAS), k(0);
          k < STATE_VALUES_BIAS;
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
    void correct(const Matrix<FloatT> &H, const Matrix<FloatT> &z, const Matrix<FloatT> &R){

#if BIAS_EST_MODE == 0
      { //その1
        // バイアスを減らしておく
        for(unsigned i = 0; i < Vector3<FloatT>::OUT_OF_INDEX; i++){
          m_bias_accel[i] *= exp(-m_beta_accel[i] * m_deltaT_sum);
          m_bias_gyro[i] *= exp(-m_beta_gyro[i] * m_deltaT_sum);
        }
        m_deltaT_sum = 0;
        
        // 突っ込む
        BaseFINS::correct(H, z, R);
      }

#elif BIAS_EST_MODE == 1
      { // その2
        Vector3<FloatT> _bias_accel(m_bias_accel.copy());
        Vector3<FloatT> _bias_gyro(m_bias_gyro.copy());
        
        // 突っ込む
        BaseFINS::correct(H, z, R);
        
        _bias_accel -= m_bias_accel;
        _bias_gyro -= m_bias_gyro;
        
        // バイアスを減らしておく
        m_bias_accel -= previous_modified_bias_accel * exp(-m_beta_accel * m_deltaT_sum);
        m_bias_gyro -= previous_modified_bias_gyro * exp(-m_beta_gyro * m_deltaT_sum);
        m_deltaT_sum = 0;
        
        // 保存しておく
        previous_modified_bias_accel = _bias_accel;
        previous_modified_bias_gyro = _bias_gyro;
      }

#elif BIAS_EST_MODE == 2
      { // その3、初期ゼロ点推定含む
        // バイアスを減らしておく
        for(unsigned i = 0; i < Vector3<FloatT>::OUT_OF_INDEX; i++){
          {
            FloatT delta_drift_accel(
                drift_bias_accel[i] 
                  * (FloatT(1) - std::exp(-m_beta_accel[i] * (m_deltaT_sum - previous_delteT_sum))));
            drift_bias_accel[i] -= delta_drift_accel;
            m_bias_accel[i] -= delta_drift_accel;
          }
          
          {
            FloatT delta_drift_gyro(
                drift_bias_gyro[i]
                  * (FloatT(1) - std::exp(-m_beta_gyro[i] * (m_deltaT_sum - previous_delteT_sum))));
            drift_bias_gyro[i] -= delta_drift_gyro;
            m_bias_gyro[i] -= delta_drift_gyro;
          }
        }
        
        Vector3<FloatT> before_correct_bias_accel(m_bias_accel.copy());
        Vector3<FloatT> before_correct_bias_gyro(m_bias_gyro.copy());
        
        // 突っ込む
        BaseFINS::correct(H, z, R);
        
        // 修正量の分配
        {
          Vector3<FloatT> delta_bias_accel(m_bias_accel - before_correct_bias_accel);
          Vector3<FloatT> delta_bias_gyro(m_bias_gyro - before_correct_bias_gyro);
          
          // 時定数の調整はここで
#ifndef ZERO_CONST_ALPHA
#define ZERO_CONST_ALPHA (FloatT(1) / 1000) // [sec]
#endif
          drift_bias_accel += delta_bias_accel * (1 - std::exp(-m_deltaT_sum * ZERO_CONST_ALPHA));
          drift_bias_gyro += delta_bias_gyro * (1 - std::exp(-m_deltaT_sum * ZERO_CONST_ALPHA));
        }
        
        // 時刻の保存
        previous_delteT_sum = m_deltaT_sum;
      }
#endif
    }
};
#endif /* __FILTERED_INS_BE_H__ */
