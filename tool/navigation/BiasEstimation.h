/*
 *  BiasEstimation.h, header file to perform calculation of INS/GPS with bias drift estimation.
 *  Copyright (C) 2016 M.Naruoka (fenrir)
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

#ifndef __BIAS_ESTIMATION_H__
#define __BIAS_ESTIMATION_H__


#include "param/matrix.h"
#include "Filtered_INS2.h"
#include "INS_GPS2.h"

#define BIAS_EST_MODE 2 // �����[���_����܂�
#define ZERO_CONST_ALPHA 0 //(FloatT(1) / 1E4) // 0

#ifndef BIAS_EST_MODE
#define BIAS_EST_MODE 0
#endif

template <typename BaseINS>
class INS_BiasEstimated;

template <class BaseINS>
struct INS_Property<INS_BiasEstimated<BaseINS> > {
  static const unsigned STATE_VALUES_WITHOUT_BIAS = INS_Property<BaseINS>::STATE_VALUES;
  static const unsigned STATE_VALUES_BIAS = 6;
  static const unsigned STATE_VALUES = STATE_VALUES_WITHOUT_BIAS + STATE_VALUES_BIAS;
};

template <
    typename BaseINS = INS<> >
class INS_BiasEstimated : public BaseINS {
  public:
#if defined(__GNUC__) && (__GNUC__ < 5)
    typedef typename BaseINS::float_t float_t;
    typedef typename BaseINS::vec3_t vec3_t;
#else
    using typename BaseINS::float_t;
    using typename BaseINS::vec3_t;
#endif

  protected:
    vec3_t m_bias_accel, m_bias_gyro;

  public:
    static const unsigned STATE_VALUES_WITHOUT_BIAS
        = INS_Property<INS_BiasEstimated<BaseINS> >::STATE_VALUES_WITHOUT_BIAS;
    static const unsigned STATE_VALUES_BIAS
        = INS_Property<INS_BiasEstimated<BaseINS> >::STATE_VALUES_BIAS;
    static const unsigned STATE_VALUES
        = INS_Property<INS_BiasEstimated<BaseINS> >::STATE_VALUES;
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

    vec3_t &bias_accel(){return m_bias_accel;}
    vec3_t &bias_gyro(){return m_bias_gyro;}

    float_t &operator[](const unsigned &index){
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
        const vec3_t &accel, const vec3_t &gyro,
        const float_t &deltaT){
      // �o�C�A�X�������čq�@���Z
      BaseINS::update(accel + m_bias_accel, gyro + m_bias_gyro, deltaT);
    }
};

template <class BaseINS>
class Filtered_INS2_Property<INS_BiasEstimated<BaseINS> >
    : public Filtered_INS2_Property<BaseINS> {
  public:
    static const unsigned P_SIZE_WITHOUT_BIAS
#if defined(_MSC_VER)
        = Filtered_INS2_Property<BaseINS>::P_SIZE
#endif
        ;
    static const unsigned Q_SIZE_WITHOUT_BIAS
#if defined(_MSC_VER)
        = Filtered_INS2_Property<BaseINS>::Q_SIZE
#endif
        ;
    static const unsigned P_SIZE_BIAS
#if defined(_MSC_VER)
        = INS_BiasEstimated<BaseINS>::STATE_VALUES_BIAS
#endif
        ;
    static const unsigned Q_SIZE_BIAS
#if defined(_MSC_VER)
        = INS_BiasEstimated<BaseINS>::STATE_VALUES_BIAS
#endif
        ;
    static const unsigned P_SIZE
#if defined(_MSC_VER)
        = P_SIZE_WITHOUT_BIAS + P_SIZE_BIAS
#endif
        ;
    static const unsigned Q_SIZE
#if defined(_MSC_VER)
        = Q_SIZE_WITHOUT_BIAS + Q_SIZE_BIAS;
#endif
        ;
};

#if !defined(_MSC_VER)
template <class BaseINS>
const unsigned Filtered_INS2_Property<INS_BiasEstimated<BaseINS> >::P_SIZE_WITHOUT_BIAS
    = Filtered_INS2_Property<BaseINS>::P_SIZE;

template <class BaseINS>
const unsigned Filtered_INS2_Property<INS_BiasEstimated<BaseINS> >::Q_SIZE_WITHOUT_BIAS
    = Filtered_INS2_Property<BaseINS>::Q_SIZE;

template <class BaseINS>
const unsigned Filtered_INS2_Property<INS_BiasEstimated<BaseINS> >::P_SIZE_BIAS
    = INS_BiasEstimated<BaseINS>::STATE_VALUES_BIAS;

template <class BaseINS>
const unsigned Filtered_INS2_Property<INS_BiasEstimated<BaseINS> >::Q_SIZE_BIAS
    = INS_BiasEstimated<BaseINS>::STATE_VALUES_BIAS;

template <class BaseINS>
const unsigned Filtered_INS2_Property<INS_BiasEstimated<BaseINS> >::P_SIZE
    = P_SIZE_WITHOUT_BIAS + P_SIZE_BIAS;

template <class BaseINS>
const unsigned Filtered_INS2_Property<INS_BiasEstimated<BaseINS> >::Q_SIZE
    = Q_SIZE_WITHOUT_BIAS + Q_SIZE_BIAS;
#endif


template <
    class BaseFINS = Filtered_INS2<INS_BiasEstimated<INS<> > > >
class Filtered_INS_BiasEstimated : public BaseFINS {
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
    vec3_t m_beta_accel, m_beta_gyro; // �x�[�^�̏C����!!
    float_t m_deltaT_sum;

#if BIAS_EST_MODE == 1
    vec3_t previous_modified_bias_accel;
    vec3_t previous_modified_bias_gyro;
#elif BIAS_EST_MODE == 2
    float_t previous_delteT_sum;
    vec3_t drift_bias_accel;
    vec3_t drift_bias_gyro;
#endif
    
  public:
    using BaseFINS::ins_t::STATE_VALUES_WITHOUT_BIAS;
    using BaseFINS::ins_t::STATE_VALUES_BIAS;
    using BaseFINS::property_t::P_SIZE_WITHOUT_BIAS;
    using BaseFINS::property_t::Q_SIZE_WITHOUT_BIAS;
    using BaseFINS::property_t::P_SIZE_BIAS;
    using BaseFINS::property_t::Q_SIZE_BIAS;
    using BaseFINS::m_bias_accel;
    using BaseFINS::m_bias_gyro;
    
    using BaseFINS::correct_primitive;

    Filtered_INS_BiasEstimated() 
        : BaseFINS(),
        m_beta_accel(1, 1, 1),
        m_beta_gyro(1, 1, 1),
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

    vec3_t &beta_accel(){return m_beta_accel;}
    vec3_t &beta_gyro(){return m_beta_gyro;}

    void getAB(
        const vec3_t &accel,
        const vec3_t &gyro,
        typename BaseFINS::getAB_res &res) const {
      
      BaseFINS::getAB(accel, gyro, res);

      { // A�s��̏C��
        // B�s��̉����x�A�p���x�Ɋւ��鍀��A�s���
        for(unsigned i(0); i < P_SIZE_WITHOUT_BIAS; i++){
          for(unsigned j(P_SIZE_WITHOUT_BIAS), k(0); k < P_SIZE_BIAS; j++, k++){
            res.A[i][j] = res.B[i][k];
          }
        }
        // A�s��̃o�C�A�X�Ɋւ���Ίp�����𖄂߂�
        for(unsigned i(P_SIZE_WITHOUT_BIAS), j(0); j < P_SIZE_BIAS; i++, j++){
          res.A[i][i] += -((j < vec3_t::OUT_OF_INDEX)
              ? m_beta_accel.get(j)
              : m_beta_gyro.get(j - (vec3_t::OUT_OF_INDEX)));
        }
      }
      
      { // B�s��̏C��
        for(unsigned i(P_SIZE_WITHOUT_BIAS), j(Q_SIZE_WITHOUT_BIAS), k(0);
             k < Q_SIZE_BIAS;
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
    void update(const vec3_t &accel, const vec3_t &gyro, const float_t &deltaT){
      // ���Ԃ𑫂�
      m_deltaT_sum += deltaT;
      BaseFINS::update(accel, gyro, deltaT);
    }
    
    /**
     * Kalman Filter�ɂ���ē���ꂽ@f$ \Hat{x} @f$�𗘗p���āAINS���C�����܂��B
     *
     * @param x_hat Kalman Filter�ɂ���ē���ꂽx_hat
     */
    void correct_INS(mat_t &x_hat){
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
    void correct_primitive(const mat_t &H, const mat_t &z, const mat_t &R){

#if BIAS_EST_MODE == 0
      { //����1
        // �o�C�A�X�����炵�Ă���
        for(unsigned i = 0; i < vec3_t::OUT_OF_INDEX; i++){
          m_bias_accel[i] *= exp(-m_beta_accel[i] * m_deltaT_sum);
          m_bias_gyro[i] *= exp(-m_beta_gyro[i] * m_deltaT_sum);
        }
        m_deltaT_sum = 0;
        
        // �˂�����
        BaseFINS::correct_primitive(H, z, R);
      }

#elif BIAS_EST_MODE == 1
      { // ����2
        vec3_t _bias_accel(m_bias_accel.copy());
        vec3_t _bias_gyro(m_bias_gyro.copy());
        
        // �˂�����
        BaseFINS::correct_primitive(H, z, R);
        
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
        for(unsigned i = 0; i < vec3_t::OUT_OF_INDEX; i++){
          {
            float_t delta_drift_accel(
                drift_bias_accel[i] 
                  * (float_t(1) - std::exp(-m_beta_accel[i] * (m_deltaT_sum - previous_delteT_sum))));
            drift_bias_accel[i] -= delta_drift_accel;
            m_bias_accel[i] -= delta_drift_accel;
          }
          
          {
            float_t delta_drift_gyro(
                drift_bias_gyro[i]
                  * (float_t(1) - std::exp(-m_beta_gyro[i] * (m_deltaT_sum - previous_delteT_sum))));
            drift_bias_gyro[i] -= delta_drift_gyro;
            m_bias_gyro[i] -= delta_drift_gyro;
          }
        }
        
        vec3_t before_correct_bias_accel(m_bias_accel.copy());
        vec3_t before_correct_bias_gyro(m_bias_gyro.copy());
        
        // �˂�����
        BaseFINS::correct_primitive(H, z, R);
        
        // �C���ʂ̕��z
        {
          vec3_t delta_bias_accel(m_bias_accel - before_correct_bias_accel);
          vec3_t delta_bias_gyro(m_bias_gyro - before_correct_bias_gyro);
          
          // ���萔�̒����͂�����
#ifndef ZERO_CONST_ALPHA
#define ZERO_CONST_ALPHA (float_t(1) / 1000) // [sec]
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

template <
  class FloatT,
  template <class> class Filter = KalmanFilterUD,
  typename BaseFINS = Filtered_INS_BiasEstimated<Filtered_INS2<INS_BiasEstimated<INS<FloatT> >, Filter> >
>
class INS_GPS2_BiasEstimated
    : public INS_GPS2<FloatT, Filter, BaseFINS>{
  public:
#if defined(__GNUC__) && (__GNUC__ < 5)
    typedef typename BaseFINS::float_t float_t;
    typedef typename BaseFINS::vec3_t vec3_t;
#else
    using typename BaseFINS::float_t;
    using typename BaseFINS::vec3_t;
#endif
    typedef INS_GPS2<FloatT, Filter, BaseFINS> super_t;

  public:
    INS_GPS2_BiasEstimated() : super_t(){}

    /**
     * �R�s�[�R���X�g���N�^
     *
     * @param orig �R�s�[��
     * @param deepcopy �f�B�[�v�R�s�[���쐬���邩�ǂ���
     */
    INS_GPS2_BiasEstimated(const INS_GPS2_BiasEstimated &orig, const bool deepcopy = false)
        : super_t(orig, deepcopy){

    }

    ~INS_GPS2_BiasEstimated(){}

    using super_t::correct;

    void correct(const GPS_UBLOX_3D<float_t> &gps,
        const vec3_t &lever_arm_b,
        const vec3_t &omega_b2i_4b){
      super_t::correct(gps, lever_arm_b, omega_b2i_4b - BaseFINS::m_bias_gyro);
    }
};

#endif /* __BIAS_ESTIMATION_H__ */
