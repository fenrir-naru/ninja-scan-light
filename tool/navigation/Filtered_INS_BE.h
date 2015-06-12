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
    class FloatT,
    typename Filter = KalmanFilterUD<FloatT>,
    typename FINS = Filtered_INS2<FloatT, Filter> >
class Filtered_INS_BiasEstimated : public FINS{
  protected:
    Vector3<FloatT> m_bias_accel;
    Vector3<FloatT> m_bias_gyro;
    FloatT m_deltaT_sum;
    Vector3<FloatT> m_beta_accel, m_beta_gyro; // ベータの修正を!!
    
#if BIAS_EST_MODE == 1
    Vector3<FloatT> previous_modified_bias_accel;
    Vector3<FloatT> previous_modified_bias_gyro;
#elif BIAS_EST_MODE == 2
    FloatT previous_delteT_sum;
    Vector3<FloatT> drift_bias_accel;
    Vector3<FloatT> drift_bias_gyro;
#endif
    
  public:
    static const unsigned P_SIZE
#if defined(_MSC_VER)
        = FINS::P_SIZE + 6
#endif
        ;
    static const unsigned Q_SIZE
#if defined(_MSC_VER)
        = FINS::Q_SIZE + 6
#endif
        ;
    static const unsigned STATE_VALUES
#if defined(_MSC_VER)
        = FINS::STATE_VALUES + 6
#endif
        ; ///< 状態量の数
    virtual unsigned state_values() const {return STATE_VALUES;}
    
    Filtered_INS_BiasEstimated() 
        : FINS(Matrix<FloatT>::getI(P_SIZE), Matrix<FloatT>::getI(Q_SIZE)), 
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
      FINS(orig, deepcopy),
      m_bias_accel(deepcopy ? orig.m_bias_accel.copy() : orig.m_bias_accel),
      m_bias_gyro(deepcopy ? orig.m_bias_gyro.copy() : orig.m_bias_gyro), 
      m_deltaT_sum(orig.m_deltaT_sum),
      m_beta_accel(deepcopy ? orig.m_beta_accel.copy() : orig.m_beta_accel), 
      m_beta_gyro(deepcopy ? orig.m_beta_gyro.copy() : orig.m_beta_gyro)
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
    
    Vector3<FloatT> &bias_accel(){return m_bias_accel;}
    Vector3<FloatT> &bias_gyro(){return m_bias_gyro;}
    Vector3<FloatT> &beta_accel(){return m_beta_accel;}
    Vector3<FloatT> &beta_gyro(){return m_beta_gyro;}
    
    FloatT &operator[](const unsigned &index){
      switch(index){
        case INS<FloatT>::STATE_VALUES:     return m_bias_accel[0];
        case INS<FloatT>::STATE_VALUES + 1: return m_bias_accel[1];
        case INS<FloatT>::STATE_VALUES + 2: return m_bias_accel[2];
        case INS<FloatT>::STATE_VALUES + 3: return m_bias_gyro[0];
        case INS<FloatT>::STATE_VALUES + 4: return m_bias_gyro[1];
        case INS<FloatT>::STATE_VALUES + 5: return m_bias_gyro[2];
        default: return INS<FloatT>::operator[](index);
      }
    }

    using FINS::before_update_INS;
    
    /**
     * 時間更新
     * 
     * @param accel 加速度計の値
     * @param gyro ジャイロの値
     * @param deltaT 時間間隔
     */
    void update(const Vector3<FloatT> &accel, const Vector3<FloatT> &gyro, const FloatT &deltaT){
      
      // まずバイアス除去
      Vector3<FloatT> _accel(accel + m_bias_accel);
      Vector3<FloatT> _gyro(gyro + m_bias_gyro);
      
      Matrix<FloatT> orig_A(FINS::getA(_accel, _gyro));
      Matrix<FloatT> orig_B(FINS::getB(_accel, _gyro));
      
      // KF用の行列計算
      Matrix<FloatT> A(P_SIZE, P_SIZE);
      {
        A.pivotMerge(0, 0, orig_A);
        A.pivotMerge(0, FINS::P_SIZE, orig_B);
        
        for(unsigned i = FINS::P_SIZE, j = 0; i < A.rows(); i++, j++){
          //cout << i << endl; 
          A(i, i) += -((j < Vector3<FloatT>::OUT_OF_INDEX) ? 
                          m_beta_accel[j] : 
                          m_beta_gyro[j - (Vector3<FloatT>::OUT_OF_INDEX)]);
        }
      }
      
      Matrix<FloatT> B(P_SIZE, Q_SIZE);
      {
        B.pivotMerge(0, 0, orig_B);
        for(unsigned i = FINS::P_SIZE, j = FINS::Q_SIZE; 
             i < B.rows();
             i++, j++) B(i, j) += 1;
      }
      
      //cout << "deltaT:" << deltaT << endl;
      //cout << "A:" << A << endl;
      //cout << "B:" << B << endl;
      
      // 突っ込む
      FINS::m_filter.predict(A, B, deltaT);
      before_update_INS(A, B, deltaT);
      INS<FloatT>::update(_accel, _gyro, deltaT);
      
      // 時間を足す
      m_deltaT_sum += deltaT;
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
        FINS::correct(H, z, R);
      }

#elif BIAS_EST_MODE == 1
      { // その2
        Vector3<FloatT> _bias_accel(m_bias_accel.copy());
        Vector3<FloatT> _bias_gyro(m_bias_gyro.copy());
        
        // 突っ込む
        FINS::correct(H, z, R);
        
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
        FINS::correct(H, z, R);
        
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

#if !defined(_MSC_VER)
template <
    class FloatT, 
    typename Filter,
    typename FINS>
const unsigned Filtered_INS_BiasEstimated<FloatT, Filter, FINS>::P_SIZE
    = FINS::P_SIZE + 6;

template <
    class FloatT, 
    typename Filter,
    typename FINS>
const unsigned Filtered_INS_BiasEstimated<FloatT, Filter, FINS>::Q_SIZE
    = FINS::Q_SIZE + 6;

template <
    class FloatT, 
    typename Filter,
    typename FINS>
const unsigned Filtered_INS_BiasEstimated<FloatT, Filter, FINS>::STATE_VALUES
    = FINS::STATE_VALUES + 6;
#endif

#endif /* __FILTERED_INS_BE_H__ */
