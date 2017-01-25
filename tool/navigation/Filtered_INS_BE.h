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

#define BIAS_EST_MODE 2 // �����[���_����܂�
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
    static const unsigned STATE_VALUES; ///< ��ԗʂ̐�
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
     * ���ԍX�V
     *
     * @param accel �����x�v�̒l
     * @param gyro �W���C���̒l
     * @param deltaT ���ԊԊu
     */
    void update(
        const typename BaseINS::vec3_t &accel, const typename BaseINS::vec3_t &gyro,
        const typename BaseINS::float_t &deltaT){
      // �o�C�A�X�������čq�@���Z
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
    Vector3<FloatT> m_beta_accel, m_beta_gyro; // �x�[�^�̏C����!!
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
      
      // �����U�s��̏�����
    }
    
    /**
     * �R�s�[�R���X�g���N�^
     * 
     * @param orig �R�s�[��
     * @param deepcopy �f�B�[�v�R�s�[���쐬���邩�ǂ���
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

      { // A�s��̏C��
        // B�s��̉����x�A�p���x�Ɋւ��鍀��A�s���
        for(unsigned i(0); i < P_SIZE_WITHOUT_BIAS; i++){
          for(unsigned j(P_SIZE_WITHOUT_BIAS), k(0); k < STATE_VALUES_BIAS; j++, k++){
            res.A[i][j] = res.B[i][k];
          }
        }
        // A�s��̃o�C�A�X�Ɋւ���Ίp�����𖄂߂�
        for(unsigned i(P_SIZE_WITHOUT_BIAS), j(0); j < STATE_VALUES_BIAS; i++, j++){
          res.A[i][i] += -((j < Vector3<FloatT>::OUT_OF_INDEX)
              ? m_beta_accel.get(j)
              : m_beta_gyro.get(j - (Vector3<FloatT>::OUT_OF_INDEX)));
        }
      }
      
      { // B�s��̏C��
        for(unsigned i(P_SIZE_WITHOUT_BIAS), j(Q_SIZE_WITHOUT_BIAS), k(0);
             k < STATE_VALUES_BIAS;
             i++, j++, k++){
          res.B[i][j] += 1;
        }
      }
    }

    /**
     * ���ԍX�V
     *
     * @param accel �����x�v�̒l
     * @param gyro �W���C���̒l
     * @param deltaT ���ԊԊu
     */
    void update(const Vector3<FloatT> &accel, const Vector3<FloatT> &gyro, const FloatT &deltaT){
      // ���Ԃ𑫂�
      m_deltaT_sum += deltaT;
      BaseFINS::update(accel, gyro, deltaT);
    }
    
    /**
     * Kalman Filter�ɂ���ē���ꂽ@f$ \Hat{x} @f$�𗘗p���āAINS���C�����܂��B
     *
     * @param x_hat Kalman Filter�ɂ���ē���ꂽx_hat
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
     * �C�����܂��B
     * 
     * @param H �ϑ��s��
     * @param z �ϑ���
     * @param R �덷�����U�s��
     */
    void correct(const Matrix<FloatT> &H, const Matrix<FloatT> &z, const Matrix<FloatT> &R){

#if BIAS_EST_MODE == 0
      { //����1
        // �o�C�A�X�����炵�Ă���
        for(unsigned i = 0; i < Vector3<FloatT>::OUT_OF_INDEX; i++){
          m_bias_accel[i] *= exp(-m_beta_accel[i] * m_deltaT_sum);
          m_bias_gyro[i] *= exp(-m_beta_gyro[i] * m_deltaT_sum);
        }
        m_deltaT_sum = 0;
        
        // �˂�����
        BaseFINS::correct(H, z, R);
      }

#elif BIAS_EST_MODE == 1
      { // ����2
        Vector3<FloatT> _bias_accel(m_bias_accel.copy());
        Vector3<FloatT> _bias_gyro(m_bias_gyro.copy());
        
        // �˂�����
        BaseFINS::correct(H, z, R);
        
        _bias_accel -= m_bias_accel;
        _bias_gyro -= m_bias_gyro;
        
        // �o�C�A�X�����炵�Ă���
        m_bias_accel -= previous_modified_bias_accel * exp(-m_beta_accel * m_deltaT_sum);
        m_bias_gyro -= previous_modified_bias_gyro * exp(-m_beta_gyro * m_deltaT_sum);
        m_deltaT_sum = 0;
        
        // �ۑ����Ă���
        previous_modified_bias_accel = _bias_accel;
        previous_modified_bias_gyro = _bias_gyro;
      }

#elif BIAS_EST_MODE == 2
      { // ����3�A�����[���_����܂�
        // �o�C�A�X�����炵�Ă���
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
        
        // �˂�����
        BaseFINS::correct(H, z, R);
        
        // �C���ʂ̕��z
        {
          Vector3<FloatT> delta_bias_accel(m_bias_accel - before_correct_bias_accel);
          Vector3<FloatT> delta_bias_gyro(m_bias_gyro - before_correct_bias_gyro);
          
          // ���萔�̒����͂�����
#ifndef ZERO_CONST_ALPHA
#define ZERO_CONST_ALPHA (FloatT(1) / 1000) // [sec]
#endif
          drift_bias_accel += delta_bias_accel * (1 - std::exp(-m_deltaT_sum * ZERO_CONST_ALPHA));
          drift_bias_gyro += delta_bias_gyro * (1 - std::exp(-m_deltaT_sum * ZERO_CONST_ALPHA));
        }
        
        // �����̕ۑ�
        previous_delteT_sum = m_deltaT_sum;
      }
#endif
    }
};
#endif /* __FILTERED_INS_BE_H__ */
