/*
 *  Filtered_INS2.h, header file to perform calculation of Kalman filter integration for INS.
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

#ifndef __FILTERED_INS2_H__
#define __FILTERED_INS2_H__

/** @file
 * @brief �����q�@���u(INS)�Ƃ̓����̋��n��(Multiplicative)
 * 
 * �����q�@���u(INS)�Ƒ��̍q�@���u��
 * �J���}���t�B���^��ʂ��ē�������̂ɕK�v�ƂȂ�
 * �s�񉉎Z�ɂ��ċL�q�����t�@�C���B
 * �N�H�[�^�j�I���̐��`���͐ώZ�^(Multiplicative)�ōs���Ă��܂��B
 */

#include "INS.h"
#include "param/matrix.h"
#include "algorithm/kalman.h"

template <class BaseINS>
class Filtered_INS2_Property {
  public:
    static const unsigned P_SIZE
#if defined(_MSC_VER)
        = BaseINS::STATE_VALUES - 2
#endif
        ; ///< P�s��(�V�X�e���덷�����U�s��)�̑傫��
    static const unsigned Q_SIZE
#if defined(_MSC_VER)
        = BaseINS::STATE_VALUES - 5
#endif
        ; ///< Q�s��(���͌덷�����U�s��)�̑傫��
};

#if !defined(_MSC_VER)
template <class BaseINS>
const unsigned Filtered_INS2_Property<BaseINS>::P_SIZE = BaseINS::STATE_VALUES - 2;

template <class BaseINS>
const unsigned Filtered_INS2_Property<BaseINS>::Q_SIZE = BaseINS::STATE_VALUES - 5;
#endif

/**
 * @brief ���̍q�@���u�𓝍�����ׂ�INS�g���N���X(Multiplicative)
 * 
 * �J���}���t�B���^��ʂ��āAINS�𑼂̍q�@���u�Ɠ������s���̂�
 * �K�v�ƂȂ鏔�s�񉉎Z�ɂ��Ē�`������INS�g���N���X�B
 * �������A�N�H�[�^�j�I���̐��`���͐ώZ�^(Multiplicative)�ōs���Ă��܂��B
 * ���Ȃ킿
 * @f[
 *    \Tilde{q} + \Delta \Tilde{q}
 *      \equiv \begin{Bmatrix} 
 *          1 \\
 *          \Delta \vec{u}
 *        \end{Bmatrix} \begin{Bmatrix} 
 *          q_{0} \\
 *          \vec{q}
 *        \end{Bmatrix} 
 * @f]
 * �Œ�`����Ă��܂��B
 * 
 * @param FloatT ���Z���x
 * @param Filter �J���}���t�B���^
 */
template <
    class FloatT,
    template <class> class Filter = KalmanFilterUD,
    typename BaseINS = INS<FloatT> >
class Filtered_INS2
    : public BaseINS,
      public Filtered_INS2_Property<BaseINS> {
  public:
    typedef BaseINS ins_t;
    typedef Filtered_INS2_Property<ins_t> property_t;
    using property_t::P_SIZE;
    using property_t::Q_SIZE;
    
  protected:
    Filter<FloatT> m_filter;  ///< �J���}���t�B���^�{��
    
#define R_STRICT ///< �ȗ����a�������Ɍv�Z���邩�̃X�C�b�`�A���̏ꍇ�v�Z����
    
    using BaseINS::get;
    
    struct getAB_res {
      FloatT A[P_SIZE][P_SIZE];
      FloatT B[P_SIZE][Q_SIZE];
      getAB_res(){
        for(int i(0); i < sizeof(A) / sizeof(A[0]); ++i){
          for(int j(0); j < sizeof(A[0]) / sizeof(A[0][0]); ++j){
            A[i][j] = FloatT(0);
          }
        }
        for(int i(0); i < sizeof(B) / sizeof(B[0]); ++i){
          for(int j(0); j < sizeof(B[0]) / sizeof(B[0][0]); ++j){
            B[i][j] = FloatT(0);
          }
        }
      }
      Matrix<FloatT> getA() const {
        return Matrix<FloatT>(
            sizeof(A) / sizeof(A[0]),
            sizeof(A[0]) / sizeof(A[0][0]),
            (FloatT *)&A);
      }
      Matrix<FloatT> getB() const {
        return Matrix<FloatT>(
            sizeof(B) / sizeof(B[0]),
            sizeof(B[0]) / sizeof(B[0][0]),
            (FloatT *)&B);
      }
    };

    /**
     * �����q�@������(�����V�X�e��������)�ɂ����āA
     * ���̏�ԗʂ̌덷�ɑ΂��Đ��`�������ꍇ�̎��A
     * ���Ȃ킿�덷�V�X�e��������
     * @f[
     *    \frac{d}{dt} \Bar{x} = A \Bar{x} + B \Bar{u}
     * @f]
     * �ɂ�����s�� A, B��Ԃ��܂��B
     * ���̎��ɂ����� @f$ \Bar{x} @f$�͏�ԗʁA�܂葬�x�E�ʒu�E�p���A�̌덷�A
     * @f$ \Bar{u} @f$�͓��́A���Ȃ킿�����x��p���x(�A�����Ă����ł͏d�͂�)�A�̌덷��\���܂��B
     * 
     * @param accel �����x
     * @param gyro �p���x
     * @param res �v�Z�l���i�[����X�y�[�X
     * @return (getAB_res) A,B�s��
     */
    virtual void getAB(
        const Vector3<FloatT> &accel, 
        const Vector3<FloatT> &gyro,
        getAB_res &res) const {

      // ��]�s��̌v�Z
      Matrix<FloatT> dcm_e2n((this->q_e2n).getDCM()); ///< @f$ \mathrm{DCM} \left( \Tilde{q}_{e}^{n} \right) @f$
      Matrix<FloatT> dcm_n2b((this->q_n2b).getDCM()); ///< @f$ \mathrm{DCM} \left( \Tilde{q}_{n}^{b} \right) @f$
      
#define dcm_e2n(r, c) (const_cast<Matrix<FloatT> *>(&dcm_e2n)->operator()(r, c))
#define dcm_n2b(r, c) (const_cast<Matrix<FloatT> *>(&dcm_n2b)->operator()(r, c))
     
#ifndef pow2
#define pow2(x) ((x) * (x))
#else
#define ALREADY_POW2_DEFINED
#endif

      typedef typename BaseINS::Earth Earth;

      { //�s��A�̌v�Z
#define A(i, j) res.A[i][j]

        Vector3<FloatT> omega_1(this->omega_e2i_4n * 2 + this->omega_n2e_4n);

#ifdef R_STRICT
        FloatT Rn_1(Earth::R_normal(this->phi) + get(7));
        FloatT Rm_1(Earth::R_meridian(this->phi) + get(7));
        FloatT Rn_2(pow2(Rn_1));
        FloatT Rm_2(pow2(Rm_1));
#else
        FloatT R(Earth::R_e + get(7));
        FloatT R2(pow2(R));
  #define Rn_1 R
  #define Rm_1 R
  #define Rn_2 R2
  #define Rm_2 R2
#endif
        
        {
          A(0, 0) =  get(2) / Rm_1;
          A(0, 1) =  omega_1[2];
          A(0, 2) = -omega_1[1];
    
          A(0, 3) = Earth::Omega_Earth * 4 * (dcm_e2n(2, 1) * get(1) - dcm_e2n(1, 1) * get(2));
          A(0, 4) = Earth::Omega_Earth * 4 * (dcm_e2n(1, 0) * get(2) - dcm_e2n(2, 0) * get(1));
    
          A(0, 6) = -get(0) * get(2) / Rm_2;
    
          //A(0, 7) = 0;
          A(0, 8) = (dcm_n2b(0, 2) * accel.get(0) + dcm_n2b(1, 2) * accel.get(1) + dcm_n2b(2, 2) * accel.get(2)) *  2;
          A(0, 9) = (dcm_n2b(0, 1) * accel.get(0) + dcm_n2b(1, 1) * accel.get(1) + dcm_n2b(2, 1) * accel.get(2)) * -2;
        }
    
        {
          A(1, 0) = -omega_1[2];
          A(1, 1) =  get(2) / Rn_1;
          A(1, 2) =  omega_1[0];
          
          A(1, 3) = Earth::Omega_Earth * 4 * (dcm_e2n(0, 1) * get(2) - dcm_e2n(2, 1) * get(0));
          A(1, 4) = Earth::Omega_Earth * 4 * (dcm_e2n(2, 0) * get(0) - dcm_e2n(0, 0) * get(2));
    
          A(1, 6) = -get(1) * get(2) / Rn_2;
    
          A(1, 7) = -A(0, 8);
          //A(1, 8) = 0;
          A(1, 9) = (dcm_n2b(0, 0) * accel.get(0) + dcm_n2b(1, 0) * accel.get(1) + dcm_n2b(2, 0) * accel.get(2)) *  2;
        }
    
        {
          A(2, 0) =  omega_1[1] - get(0) / Rm_1;
          A(2, 1) = -omega_1[0] - get(1) / Rn_1;
    
          A(2, 3) = Earth::Omega_Earth * 4 * (dcm_e2n(1, 1) * get(0) - dcm_e2n(0, 1) * get(1));
          A(2, 4) = Earth::Omega_Earth * 4 * (dcm_e2n(0, 0) * get(1) - dcm_e2n(1, 0) * get(0));
    
          A(2, 6) = pow2(get(0)) / Rm_2 + pow2(get(1)) / Rn_2;

          A(2, 7) = -A(0, 9);
          A(2, 8) = -A(1, 9);
          //A(2, 9) = 0;
        }
        
#define CENTRIPETAL ///< ���S�͌덷���l�����邩�̃X�C�b�`�A���̏ꍇ�l������
#ifdef CENTRIPETAL
        {
#define by_delta_r(i, j) A(i + 0, j + 3)
          {
            by_delta_r(0, 0) =  dcm_e2n(0, 1) * dcm_e2n(2, 2) + dcm_e2n(0, 2) * dcm_e2n(2, 1);
            by_delta_r(0, 1) = -dcm_e2n(0, 0) * dcm_e2n(2, 2) - dcm_e2n(0, 2) * dcm_e2n(2, 0);
            
            by_delta_r(1, 0) =  dcm_e2n(1, 1) * dcm_e2n(2, 2) + dcm_e2n(1, 2) * dcm_e2n(2, 1);
            by_delta_r(1, 1) = -dcm_e2n(1, 0) * dcm_e2n(2, 2) - dcm_e2n(1, 2) * dcm_e2n(2, 0);
            
            by_delta_r(2, 0) =  dcm_e2n(2, 1) * dcm_e2n(2, 2) + dcm_e2n(2, 2) * dcm_e2n(2, 1);
            by_delta_r(2, 1) = -dcm_e2n(2, 0) * dcm_e2n(2, 2) - dcm_e2n(2, 2) * dcm_e2n(2, 0);
            
            FloatT coef(pow2(Earth::Omega_Earth) * 2 * Rn_1);
            by_delta_r(0, 0) *= coef;
            by_delta_r(0, 1) *= coef;
            
            by_delta_r(1, 0) *= coef;
            by_delta_r(1, 1) *= coef;
            
            by_delta_r(2, 0) *= coef;
            by_delta_r(2, 1) *= coef;
          }
#undef by_delta_r
          
          A(0, 6) -= (dcm_e2n(0, 0) * dcm_e2n(2, 0) - dcm_e2n(0, 1) * dcm_e2n(2, 1)) * pow2(Earth::Omega_Earth);
          A(1, 6) -= (dcm_e2n(1, 0) * dcm_e2n(2, 0) - dcm_e2n(1, 1) * dcm_e2n(2, 1)) * pow2(Earth::Omega_Earth);
          A(2, 6) -= (dcm_e2n(2, 0) * dcm_e2n(2, 0) - dcm_e2n(2, 1) * dcm_e2n(2, 1)) * pow2(Earth::Omega_Earth);
        }
#endif
    
        {
          A(3, 0) = -dcm_e2n(1, 0) / 2 / Rm_1;
          A(3, 1) =  dcm_e2n(0, 0) / 2 / Rn_1;
          
          A(3, 6) =  (dcm_e2n(1, 0) * get(0) / Rm_2 - dcm_e2n(0, 0) * get(1) / Rn_2) / 2;
        }
    
        {
          A(4, 0) = -dcm_e2n(1, 1) / 2 / Rm_1;
          A(4, 1) =  dcm_e2n(0, 1) / 2 / Rn_1;
          
          A(4, 6) =  (dcm_e2n(1, 1) * get(0) / Rm_2 - dcm_e2n(0, 1) * get(1) / Rn_2) / 2;
        }
        
        {
          A(5, 0) = -dcm_e2n(1, 2) / 2 / Rm_1;
          A(5, 1) =  dcm_e2n(0, 2) / 2 / Rn_1;
          
          A(5, 6) =  (dcm_e2n(1, 2) * get(0) / Rm_2 - dcm_e2n(0, 2) * get(1) / Rn_2) / 2;
        }
    
        {
          A(6, 2) = -1;
        }
    
        Vector3<FloatT> omega_2(this->omega_e2i_4n + this->omega_n2e_4n);
        {
          A(7, 1) = -1. / Rn_1 / 2;
      
          A(7, 3) = -Earth::Omega_Earth * dcm_e2n(0, 1);
          A(7, 4) =  Earth::Omega_Earth * dcm_e2n(0, 0);
      
          A(7, 6) = get(1) / Rn_2 / 2;
      
          A(7, 8) =  omega_2[2];
          A(7, 9) = -omega_2[1];
        }
    
        {
          A(8, 0) =  1. / Rm_1 / 2;
      
          A(8, 3) = -Earth::Omega_Earth * dcm_e2n(1, 1);
          A(8, 4) =  Earth::Omega_Earth * dcm_e2n(1, 0);
      
          A(8, 6) = -get(0) / Rm_2 / 2;
      
          A(8, 7) = -omega_2[2];
          A(8, 9) =  omega_2[0];
        }
    
        {
          A(9, 3) = -Earth::Omega_Earth * dcm_e2n(2, 1);
          A(9, 4) =  Earth::Omega_Earth * dcm_e2n(2, 0);
      
          A(9, 7) =  omega_2[1];
          A(9, 8) = -omega_2[0];
        }
#undef A
      }
#ifdef ALREADY_POW2_DEFINED
#undef ALREADY_POW2_DEFINED
#else
#undef pow2
#endif

      { //�s��B�̌v�Z
#define B(i, j) res.B[i][j]

        B(0, 0) = dcm_n2b(0, 0);
        B(0, 1) = dcm_n2b(1, 0);
        B(0, 2) = dcm_n2b(2, 0);
    
        B(1, 0) = dcm_n2b(0, 1);
        B(1, 1) = dcm_n2b(1, 1);
        B(1, 2) = dcm_n2b(2, 1);
    
        B(2, 0) = dcm_n2b(0, 2);
        B(2, 1) = dcm_n2b(1, 2);
        B(2, 2) = dcm_n2b(2, 2);
        
        B(2, 6) = FloatT(1); // �d�͌덷Z�̂�
    
        B(7, 3) = dcm_n2b(0, 0) / 2;
        B(7, 4) = dcm_n2b(1, 0) / 2;
        B(7, 5) = dcm_n2b(2, 0) / 2;
    
        B(8, 3) = dcm_n2b(0, 1) / 2;
        B(8, 4) = dcm_n2b(1, 1) / 2;
        B(8, 5) = dcm_n2b(2, 1) / 2;
    
        B(9, 3) = dcm_n2b(0, 2) / 2;
        B(9, 4) = dcm_n2b(1, 2) / 2;
        B(9, 5) = dcm_n2b(2, 2) / 2;
#undef B
      }
    }
  
  public:
    
    /**
     * �R���X�g���N�^�B
     * �s��P(�V�X�e���덷�����U�s��)��
     * Q(���͌덷�����U�s��)�̍쐬�͓����I�ɍs���܂��B
     * 
     */
    Filtered_INS2() 
        : BaseINS(),
          m_filter(Matrix<FloatT>::getI(P_SIZE), Matrix<FloatT>::getI(Q_SIZE)){
    }
    
    /**
     * �R���X�g���N�^�B
     * �s��P(�V�X�e���덷�����U�s��)��
     * Q(���͌덷�����U�s��)���w�肵�ď��������������܂��B
     * 
     * @param P P�s��(�V�X�e���덷�����U�s��)
     * @param Q Q�s��(���͌덷�����U�s��)
     */
    Filtered_INS2(const Matrix<FloatT> &P, const Matrix<FloatT> &Q) 
        : BaseINS(), m_filter(P, Q) {}
    
    /**
     * �R�s�[�R���X�g���N�^
     * 
     * @param orig �R�s�[��
     * @param deepcopy �f�B�[�v�R�s�[���쐬���邩�ǂ���
     */
    Filtered_INS2(const Filtered_INS2 &orig, const bool deepcopy = false)
        : BaseINS(orig, deepcopy),
          m_filter(orig.m_filter, deepcopy){
    }
    
    virtual ~Filtered_INS2(){}

  protected:
    /**
     * ���ԍX�V�ɂ����Č㏈�������邽�߂̃R�[���o�b�N�֐��B
     * �f�t�H���g�ł͉������܂���B
     * 
     * @param A A�s��
     * @param B B�s��
     * @param deltaT ���ԊԊu
     */
    virtual inline void before_update_INS(
        const Matrix<FloatT> &A, const Matrix<FloatT> &B, 
        const FloatT &deltaT
      ){}
      
  public:    
    /**
     * ���ԍX�V(Time Update)
     * 
     * @param accel �����x
     * @param gyro �p���x
     * @param deltaT ���ԊԊu
     */
    void update(const Vector3<FloatT> &accel, const Vector3<FloatT> &gyro, const FloatT &deltaT){
      getAB_res AB;
      getAB(accel, gyro, AB);
      Matrix<FloatT> A(AB.getA()), B(AB.getB());
      //std::cerr << "deltaT:" << deltaT << std::endl;
      //std::cerr << "A:" << A << std::endl;
      //std::cerr << "B:" << B << std::endl;
      //std::cerr << "P:" << m_filter.getP() << std::endl;
      m_filter.predict(A, B, deltaT);
      before_update_INS(A, B, deltaT);
      BaseINS::update(accel, gyro, deltaT);
    }
  
  protected:
    /**
     * �ϑ��X�V(Measurement Update)�ɂ����Č㏈�������邽�߂̃R�[���o�b�N�֐��B
     * �f�t�H���g�ł͉������܂���B
     * 
     * @param H H�s��
     * @param R R�s��
     * @param K K�s��A�J���}���Q�C��
     * @param v (z - H x)�����ł�EKF��x���[���ł��邽��z�ɓ�����
     * @param x_hat �C����
     */
    virtual inline void before_correct_INS(
        const Matrix<FloatT> &H,
        const Matrix<FloatT> &R,
        const Matrix<FloatT> &K,
        const Matrix<FloatT> &v,
        Matrix<FloatT> &x_hat
      ){}
      
    /**
     * Kalman Filter�ɂ���ē���ꂽ@f$ \Hat{x} @f$�𗘗p���āAINS���C�����܂��B
     * 
     * @param x_hat Kalman Filter�ɂ���ē���ꂽx_hat
     */
    virtual inline void correct_INS(Matrix<FloatT> &x_hat){
      
      // ���x
      for(unsigned i = 0; i < 3; i++){(*this)[i] -= x_hat(i, 0);}
      
      // 2D�ʒu�C��
#ifndef pow2
#define pow2(x) ((x) * (x))
#else
#define ALREADY_POW2_DEFINED
#endif
      Quaternion<FloatT> delta_q_e2n(1, -x_hat(3, 0), -x_hat(4, 0), -x_hat(5, 0));
      (this->q_e2n) = delta_q_e2n * (this->q_e2n);
      
      // z�ʒu
      (*this)[7] -= x_hat(6, 0);
      
      // �p���C��
      Quaternion<FloatT> delta_q_n2b(1, -x_hat(7, 0), -x_hat(8, 0), -x_hat(9, 0));
      (this->q_n2b) = delta_q_n2b * (this->q_n2b);
#ifdef ALREADY_POW2_DEFINED
#undef ALREADY_POW2_DEFINED
#else
#undef pow2
#endif

      BaseINS::recalc();
    }

  public:
    /**
     * �ϑ��X�V(Measurement Update)���܂��B
     * 
     * @param H �ϑ��s��
     * @param z �ϑ���
     * @param R �덷�����U�s��
     */
    virtual void correct(const Matrix<FloatT> &H, const Matrix<FloatT> &z, const Matrix<FloatT> &R){
            
      // �C���ʂ̌v�Z
      Matrix<FloatT> K(m_filter.correct(H, R)); //�J���}���Q�C��
      Matrix<FloatT> x_hat(K * z);
      before_correct_INS(H, R, K, z, x_hat);
      correct_INS(x_hat);
    }
    /**
     * �t�B���^�[���擾���܂��B
     * P�s���Q�s�񂪗~�����ꍇ��getFilter().getP()�ȂǂƂ��Ă��������B
     * 
     * @return (Filter &) �t�B���^�[
     */
    Filter<FloatT> &getFilter(){return m_filter;}

    struct StandardDeviations {
      FloatT v_north_ms, v_east_ms, v_down_ms;
      FloatT longitude_rad, latitude_rad, height_m;
      FloatT heading_rad, pitch_rad, roll_rad;
    };

    /**
     * �W���΍������݂̌`�ŋ��߂܂��B
     *
     * @return (StandardDeviations) [�k/��/���������x�A�o/�ܓx�A���x�A�w�f�B���O�A�s�b�`�A���[��]
     * �ō\�����ꂽ�W���΍�
     */
    StandardDeviations getSigma() const {
      StandardDeviations sigma;

      Matrix<FloatT> &P(const_cast<Matrix<FloatT> &>(
          const_cast<Filtered_INS2 *>(this)->getFilter().getP()));

      { // ���x
        sigma.v_north_ms = std::sqrt(P(0, 0));
        sigma.v_east_ms = std::sqrt(P(1, 1));
        sigma.v_down_ms = std::sqrt(P(2, 2));
      }

      { // �ʒu
        FloatT cl(std::cos(BaseINS::lambda)), sl(std::sin(BaseINS::lambda));

        Matrix<FloatT> M(2, 3);

        M(0, 0) = 0;
        M(0, 1) = 0;
        M(0, 2) = 1;

        M(1, 0) = - sl * cl * 2;
        M(1, 1) = cl * cl * 2 - 1;
        M(1, 2) = 0;

        Matrix<FloatT> P_euler_e2n(M * P.partial(3, 3, 3, 3) * M.transpose());

        sigma.longitude_rad = std::sqrt(P_euler_e2n(0, 0)) * 2; // �o�x
        sigma.latitude_rad = std::sqrt(P_euler_e2n(1, 1)) * 2; // �ܓx

        sigma.height_m = std::sqrt(P(6, 6)); // ���x
      }

      { // �p��
        FloatT psi(BaseINS::euler_psi()), theta(BaseINS::euler_theta()), phi(BaseINS::euler_phi());

        // �ŏ����@(���O�v�Z)
        FloatT cpsi(std::cos(psi)), spsi(std::sin(psi));
        FloatT ctheta(std::cos(theta)), ttheta(std::tan(theta));
        Matrix<FloatT> M_conv(3, 3);
        {
          M_conv(0, 0) =  cpsi * ttheta; M_conv(0, 1) = spsi * ttheta; M_conv(0, 2) = 1;
          M_conv(1, 0) = -spsi;          M_conv(1, 1) = cpsi;          M_conv(1, 2) = 0;
          M_conv(2, 0) =  cpsi / ctheta; M_conv(2, 1) = spsi / ctheta; M_conv(2, 2) = 0;
        }

        Matrix<FloatT> P_euler_n2b(M_conv * P.partial(3, 3, 7, 7) * M_conv.transpose());

        sigma.heading_rad = std::sqrt(P_euler_n2b(0, 0)) * 2; // ���[
        sigma.pitch_rad = std::sqrt(P_euler_n2b(1, 1)) * 2; // �s�b�`
        sigma.roll_rad = std::sqrt(P_euler_n2b(2, 2)) * 2; // ���[��
      }

      return sigma;
    }
};

#endif /* __FILTERED_INS2_H__ */
