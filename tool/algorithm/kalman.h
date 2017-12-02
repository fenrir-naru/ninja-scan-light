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

#ifndef __KALMAN_H__
#define __KALMAN_H__

#include "param/matrix.h"

/** @file
 * @brief Kalman Filter���L�q�����t�@�C���ł��B
 * 
 * Kalman�@Filter���L�q�����t�@�C���ł��B
 * ���݂�2��ނ̗��UKalman Filter�A���Ȃ킿�W���I��Kalman Filter��
 * UD����Klamn Filter���T�|�[�g���Ă��܂��B
 * �����I�ɍs��̌v�Z���s���Ă��邽�߁A�s�񃉃C�u����( Matrix )��K�v�Ƃ��܂��B
 * 
 * @see Matrix �s�񃉃C�u���� 
 */

/**
 * @brief �W���I��Kalman Filter
 * 
 * �W���I��Kalman Filter���`���Ă��܂��B
 * �g�p���邽�߂ɂ̓V�X�e���̌덷�����U�s��P�A���͂̌덷�����U�s��Q�����������Ɏw�肷��K�v������܂��B
 * ���������init()���Ăяo���ď����������������Ă��������B
 * ���ԍX�V(Time Update)�ł̓V�X�e���������Ɋ֘A����s��@f$ A @f$, @f$ B @f$ 
 * �܂��͂����Euler�ϕ�����@f$ \Phi @f$�܂���@f$ \Gamma @f$���K�v�ł��B
 * @f$ A @f$, @f$ B @f$�̒�`�͏�ԗ�@f$ x @f$�A���тɓ���@f$ u @f$�ɑ΂���
 * @f[
 *    \frac{d}{dt} x = A x + B u
 * @f]
 * �Œ�`����
 * @f[
 *    \Phi = \int_{t} A, \Gamma = \int_{t} B
 * @f]
 * �ł��B
 * �ϑ��X�V(Measurement Update)�ł͊ϑ��������Ɋ֘A����s��@f$ H @f$�A
 * �ϑ��덷�����U�s��@f$ R @f$�A���тɊϑ���@f$ z @f$���K�v�Ƃ���܂��B
 * �����̒�`��
 * @f[
 *    z = H x + v
 * @f] 
 * �ł��B
 * 
 * �Ȃ��A�e���v���[�gFloatT�͉��Z���x�̌^���w�肵�܂��B
 * ��قǂ̂��Ƃ��Ȃ�����Adouble�𐄏����܂��B
 * 
 * @param FloatT ���Z���x
 */
template <class FloatT>
class KalmanFilter{
  protected:
    Matrix<FloatT> m_P; ///< �J���}���t�B���^��P�s��(�V�X�e���덷�����U�s��)
    Matrix<FloatT> m_Q; ///< �J���}���t�B���^��Q�s��(���͌덷�����U�s��)
    
  public:
    /**
     * KalmanFilter�̃R���X�g���N�^�B
     * �덷�����U�s��@f$ P @f$, @f$ Q @f$���w�肷��K�v������܂��B
     * 
     * @param P @f$ P @f$�s��
     * @param Q @f$ Q @f$�s��
     */
    KalmanFilter(const Matrix<FloatT> &P,
                 const Matrix<FloatT> &Q) : m_P(P), m_Q(Q) {
    }
    
    /**
     * �R�s�[�R���X�g���N�^
     * 
     * @param orig �R�s�[��
     * @param deepcopy �f�B�[�v�R�s�[���쐬���邩�ǂ���
     */
    KalmanFilter(const KalmanFilter &orig, const bool &deepcopy = false) :
      m_P(deepcopy ? orig.m_P.copy() : orig.m_P), 
      m_Q(deepcopy ? orig.m_Q.copy() : orig.m_Q){
      //std::cerr << "KF" << std::endl;
      
    }
    
    /**
     * KalmanFilter�̃f�X�g���N�^�B
     * 
     */
    virtual ~KalmanFilter(){}
    
    /**
     * ����t�B���^�[�����ԍX�V���܂��B
     * ���U�n�o�[�W����
     * @f[
     *   x_{k+1} = \Phi_{k+1,K} * x_{k} + \Gamma_{k} * u_{k}
     * @f]
     * 
     * @param Phi @f$ \Phi @f$�s��
     * @param Gamma @f$ \Gamma @f$�s��
     */
    virtual void predict(const Matrix<FloatT> &Phi, const Matrix<FloatT> &Gamma){
    
#if DEBUG > 2
      std::cerr << "Phi:" << Phi << std::endl;
      std::cerr << "Gamma:" << Gamma << std::endl;
#endif

      (m_P = Phi * m_P * Phi.transpose()) += (Gamma * m_Q * Gamma.transpose());
    }
    
    /**
     * ����t�B���^�[��Euler�@�ɂ���Ď��ԍX�V���܂��B
     * �A���n�o�[�W�����B
     * @f{gather*}
     *   \frac{d}{dt} x = A x + B u \\
     *   \Phi_{k+1,k} = I + A \Delta t \\
     *   \Gamma_{k} = B \Delta t
     * @f}
     * 
     * @param A @f$ A @f$�s��
     * @param B @f$ B @f$�s��
     * @param delta ���ԊԊu 
     */
    virtual void predict(const Matrix<FloatT> &A, const Matrix<FloatT> &B, const FloatT &delta){
      
      //��
      Matrix<FloatT> Phi = A * delta;
      for(unsigned i = 0; i < Phi.rows(); i++) Phi(i, i) += 1;
    
      //��
      Matrix<FloatT> Gamma = B * delta;
      
      predict(Phi, Gamma);
    }
    
    /**
     * �t�B���^�[���ϑ��X�V(�C��)���A���̍ۂ̃J���}���Q�C�������߂܂��B
     * 
     * @param H @f$ H @f$�s��(�ϑ��s��)
     * @param R �ϑ��l�̌덷�����U�s��@f$ R @f$
     * @return (Matrix<FloatT>) �J���}���Q�C��@f$ K @f$
     */
    virtual Matrix<FloatT> correct(const Matrix<FloatT> &H, const Matrix<FloatT> &R){

      // �J���}���Q�C���̌v�Z
      Matrix<FloatT> K(m_P * H.transpose() * ((H * m_P * H.transpose()) += R).inverse());
#if DEBUG > 1
      std::cerr << "K:" << K << std::endl;
#endif

      // P �X�V
      m_P = (Matrix<FloatT>::getI(K.rows()) - K * H) * m_P;
#if DEBUG
      std::cerr << "P:" << m_P << std::endl;
#endif
      
      return K;
    }
    
    /**
     * �덷�����U�s��@f$ P @f$��Ԃ��܂��B
     * 
     * @return (const Matrix<FloatT> &) ���݂�@f$ P @f$�s��
     */
    virtual const Matrix<FloatT> &getP() const {return m_P;}

    /**
     * �덷�����U�s��@f$ P @f$��ݒ肵�܂��B
     *
     * @param P �V����@f$ P @f$�s��
     */
    virtual void setP(const Matrix<FloatT> &P){m_P = P;}
    
    /**
     * �덷�����U�s��@f$ Q @f$��Ԃ��܂��B
     * 
     * @return (Matrix<FloatT>) ���݂�@f$ Q @f$�s��
     */
    virtual const Matrix<FloatT> &getQ() const {return m_Q;}

    /**
     * �덷�����U�s��@f$ Q @f$��ݒ肵�܂��B
     *
     * @param Q �V����@f$ Q @f$�s��
     */
    virtual void setQ(const Matrix<FloatT> &Q){m_Q = Q;}
};

/**
 * @brief Information Filter
 * 
 * Information Filter(Kalman Filter�ŃV�X�e�������U�s�񂪋t�s��ɂȂ�)���`���Ă��܂��B
 * �g�p�����ł��̎g�����͕W���I��Kalman Filter�ƂȂ��ς��܂���B
 * 
 * @param FloatT ���Z���x
 * @see KalmanFilter
 */
template <class FloatT>
class InformationFilter : public KalmanFilter<FloatT>{
  protected:
    Matrix<FloatT> m_I;
    bool need_update_P;
    
    /**
     * �덷�����U�s��@f$ P @f$���X�V���܂��B
     * @f$ P @f$�̋t�s��@f$ I @f$����@f$ P @f$�𕜌����Ă��܂��B
     * ������s�����Ƃɂ���ăt�B���^�[�����̐��������ۂ���܂��B
     * 
     */
    void updateP(){
      if(!need_update_P){return;}
      //P�X�V
      KalmanFilter<FloatT>::m_P = m_I.inverse();
      need_update_P = false;
#if DEBUG
      std::cerr << "P:" << KalmanFilter<FloatT>::m_P << std::endl;
#endif
    }
    
  public:
    /**
     * �덷�����U�s��@f$ P @f$��Ԃ��܂��B
     * 
     * @return (Matrix<FloatT>) ���݂�@f$ P @f$�s��
     */
    const Matrix<FloatT> &getP() const {
      const_cast<InformationFilter *>(this)->updateP();
      return KalmanFilter<FloatT>::m_P;
    }

    /**
     * �덷�����U�s��@f$ P @f$��ݒ肵�܂��B
     *
     * @param P �V����@f$ P @f$�s��
     */
    void setP(const Matrix<FloatT> &P){
      m_I = P.inverse();
      need_update_P = true;
    }
  
    /**
     * UD����KalmanFilter�̃R���X�g���N�^�B
     * �덷�����U�s��@f$ P @f$, @f$ Q @f$���w�肷��K�v������܂��B
     * 
     * @param P @f$ P @f$�s��
     * @param Q @f$ Q @f$�s��
     */
    InformationFilter(
        const Matrix<FloatT> &P, const Matrix<FloatT> &Q)
          : KalmanFilter<FloatT>(P, Q), m_I(P.inverse()), need_update_P(false){
    }
    
    /**
     * �R�s�[�R���X�g���N�^
     * 
     * @param orig �R�s�[��
     * @param deepcopy �f�B�[�v�R�s�[���쐬���邩�ǂ���
     */
    InformationFilter(
        const InformationFilter &orig, const bool &deepcopy = false)
          : KalmanFilter<FloatT>(orig, deepcopy),
            m_I(deepcopy ? orig.m_I.copy() : orig.m_I),
            need_update_P(orig.need_update_P){
      //std::cerr << "KFUD" << std::endl;
    }
    
    /**
     * �f�X�g���N�^
     * 
     */
    ~InformationFilter(){}
    
    using KalmanFilter<FloatT>::predict;

    /**
     * ����t�B���^�[�����ԍX�V���܂��B
     * ���U�n�o�[�W����
     * @f[
     *   x_{k+1} = \Phi_{k+1,K} * x_{k} + \Gamma_{k} * u_{k}
     * @f]
     * 
     * @param Phi @f$ \Phi @f$�s��
     * @param Gamma @f$ \Gamma @f$�s��
     */
    void predict(const Matrix<FloatT> &Phi, const Matrix<FloatT> &Gamma){
     
#if DEBUG     
      KalmanFilter<FloatT>::predict(Phi, Gamma);
      std::cerr << "predict_KF_P:" << KalmanFilter<FloatT>::m_P << std::endl;
#endif

      Matrix<FloatT> inv_additive_term(
          (Gamma * KalmanFilter<FloatT>::m_Q * Gamma.transpose()).inverse());
      m_I = inv_additive_term
          - inv_additive_term * Phi 
            * (m_I + Phi.transpose() * inv_additive_term * Phi).inverse()
            * Phi.transpose() * inv_additive_term;

      //�s��P�̍X�V
      need_update_P = true;

#if DEBUG
      std::cerr << "predict_IF_P:" << getP() << std::endl;
#endif
    }
    
    /**
     * �t�B���^�[���ϑ��X�V(�C��)���A���̍ۂ̃J���}���Q�C�������߂܂��B
     * �����I��UD�����𗘗p���Ă��܂��B
     * 
     * @param H @f$ H @f$�s��(�ϑ��s��)
     * @param R �ϑ��l�̌덷�����U�s��@f$ R @f$
     * @return (Matrix<FloatT>) �J���}���Q�C��@f$ K @f$
     */
    Matrix<FloatT> correct(const Matrix<FloatT> &H, const Matrix<FloatT> &R){
#if DEBUG
      std::cerr << "correct_KF_K:" << KalmanFilter<FloatT>::correct(H, R) << std::endl;
      std::cerr << "correct_KF_P:" << KalmanFilter<FloatT>::m_P << std::endl;
#endif
      
      Matrix<FloatT> R_inv(R.inverse());
      Matrix<FloatT> H_trans(H.transpose());
      
      m_I += H_trans * R_inv * H;
      
      // �J���}���Q�C��
      Matrix<FloatT> K(m_I.inverse() * H_trans * R_inv);
      
      //�s��P�̍X�V
      need_update_P = true;

#if DEBUG
      std::cerr << "correct_IF_K:" << K << std::endl;
      std::cerr << "correct_IF_P:" << getP() << std::endl;
#endif

      return K;
    }
    
    /**
     * �덷�����U�s��@f$ P @f$�̋t�s��@f$ I @f$��Ԃ��܂��B
     * 
     * @return (const Matrix<FloatT> &) �s��@f$ I @f$
     */
    const Matrix<FloatT> &getI(){return m_I;}
};

/**
 * @brief UD����Kalman Filter
 * 
 * UD����Kalman Filter���`���Ă��܂��B
 * ���Z���x�������邽�߂ɓ����I��UD�����𗘗p���Ă��邱�Ƃ��W���I��Kalman Filter�Ƃ̈Ⴂ�ŁA
 * �g�p�����ł��̎g�����͕W���I��Kalman Filter�ƂȂ��ς��܂���B
 * 
 * @param FloatT ���Z���x
 * @see KalmanFilter
 */
template <class FloatT>
class KalmanFilterUD : public KalmanFilter<FloatT>{
  protected:
    Matrix<FloatT> m_U, m_D;
    bool need_update_P;
    
    /**
     * �덷�����U�s��@f$ P @f$���X�V���܂��B
     * @f$ P @f$��UD���������s�񂩂�@f$ P @f$�𕜌����Ă��܂��B
     * ������s�����Ƃɂ���ăt�B���^�[�����̐��������ۂ���܂��B
     * 
     */
    void updateP(){
      if(!need_update_P){return;}
      //P�X�V
      KalmanFilter<FloatT>::m_P = m_U * m_D * m_U.transpose();
      need_update_P = false;
#if DEBUG
      std::cerr << "P:" << KalmanFilter<FloatT>::m_P << std::endl;
#endif
    }
    
  public:
    /**
     * �덷�����U�s��@f$ P @f$��Ԃ��܂��B
     * 
     * @return (Matrix<FloatT>) ���݂�@f$ P @f$�s��
     */
    const Matrix<FloatT> &getP(){
      const_cast<KalmanFilterUD *>(this)->updateP();
        return KalmanFilter<FloatT>::m_P;
    }

    /**
     * �덷�����U�s��@f$ P @f$��ݒ肵�܂��B
     * �����I�ɂ͌덷�����U�s��@f$ P @f$��UD�������s���A��̌v�Z�ɔ����܂��B
     *
     * @param P �V����@f$ P @f$�s��
     */
    void setP(const Matrix<FloatT> &P){
      m_U = Matrix<FloatT>(P.rows(), P.columns());
      m_D = Matrix<FloatT>(P.rows(), P.columns());

      // UD����
      Matrix<FloatT> UD(P.decomposeUD(false));

      for(int i = 0; i < m_U.rows(); i++){
        m_D(i, i) = UD(i, i + m_U.columns());
        for(int j = 0; j < m_U.columns(); j++){
          m_U(i, j) = UD(i, j);
        }
      }
#if DEBUG
      std::cerr << "U:" << m_U << std::endl;
      std::cerr << "D:" << m_D << std::endl;
#endif
    }
  
    /**
     * UD����KalmanFilter�̃R���X�g���N�^�B
     * �덷�����U�s��@f$ P @f$, @f$ Q @f$���w�肷��K�v������܂��B
     * 
     * @param P @f$ P @f$�s��
     * @param Q @f$ Q @f$�s��
     */
    KalmanFilterUD(const Matrix<FloatT> &P,
                   const Matrix<FloatT> &Q)
        : KalmanFilter<FloatT>(P, Q), m_U(), m_D(), need_update_P(false){
      setP(P);
    }
    
    /**
     * �R�s�[�R���X�g���N�^
     * 
     * @param orig �R�s�[��
     * @param deepcopy �f�B�[�v�R�s�[���쐬���邩�ǂ���
     */
    KalmanFilterUD(const KalmanFilterUD &orig, const bool &deepcopy = false) :
      KalmanFilter<FloatT>(orig, deepcopy),
      m_U(deepcopy ? orig.m_U.copy() : orig.m_U), 
      m_D(deepcopy ? orig.m_D.copy() : orig.m_D),
      need_update_P(orig.need_update_P){
      //std::cerr << "KFUD" << std::endl;
    }
    
    /**
     * KalmanFilter�̃f�X�g���N�^�B
     * 
     */
    ~KalmanFilterUD(){}
    
    using KalmanFilter<FloatT>::predict;

    /**
     * ����t�B���^�[�����ԍX�V���܂��B
     * �����I��UD���������p���Ă��܂��B
     * ���U�n�o�[�W����
     * @f[
     *   x_{k+1} = \Phi_{k+1,K} * x_{k} + \Gamma_{k} * u_{k}
     * @f]
     * 
     * @param Phi @f$ \Phi @f$�s��
     * @param Gamma @f$ \Gamma @f$�s��
     */
    void predict(const Matrix<FloatT> &Phi, const Matrix<FloatT> &Gamma){
     
#if DEBUG     
      KalmanFilter<FloatT>::predict(Phi, Gamma);
      std::cerr << "predict_KF_P:" << KalmanFilter<FloatT>::m_P << std::endl;
#endif

      // �s��FU
      Matrix<FloatT> FU(Phi * m_U);
      
      // �s��W
      Matrix<FloatT> W(Phi.rows(), KalmanFilter<FloatT>::m_P.columns() + Gamma.columns());
      W.pivotMerge(0, 0, FU);
      W.pivotMerge(0, Phi.rows(), Gamma);
      
      // �s��Q
      Matrix<FloatT> Q(W.columns(), W.columns());
      for(int i = 0; i < m_D.rows(); i++){
        Q(i, i) = m_D(i, i);
      }
      for(int i = m_D.rows(); i < Q.rows(); i++){
        Q(i, i) = KalmanFilter<FloatT>::m_Q(i - m_D.rows(), i - m_D.rows());
      }

#if DEBUG
      std::cerr << "W:" << W << std::endl;
      std::cerr << "Q:" << Q << std::endl;
#endif
      
      for(int j = W.rows() - 1; j > 0; j--){
        typename Matrix<FloatT>::partial_t V(W.rowVector(j));
        
        Matrix<FloatT> Z(1, Q.columns()); // = V * Q�A�������̂��ߓW�J���ď���
        for(int i = 0; i < Z.columns(); i++){
          Z(0, i) = V(0, i) * Q(i, i); 
        }
        
        m_D(j, j) = (Z * V.transpose())(0, 0);
        for(int i = 0; i < j; i++){
          m_U(i, j) = (W.rowVector(i) * Z.transpose())(0, 0) / m_D(j, j);
          W.rowVector(i) -= m_U(i, j) * V;
        }
      }
      
      // m_D(0, 0) = (W.rowVector(0) * Q * W.rowVector(0).transpose())(0, 0);��������
      m_D(0, 0) = 0;
      for(int j = 0; j < W.columns(); j++){
        m_D(0, 0) += W(0, j) * W(0, j) * Q(j, j);
      }

      //�s��P�̍X�V
      need_update_P = true;

#if DEBUG
      std::cerr << "predict_UDKF_U:" << m_U << std::endl;
      std::cerr << "predict_UDKF_D:" << m_D << std::endl;
      std::cerr << "predict_UDKF_P:" << getP() << std::endl;
#endif
    }
    
    /**
     * �t�B���^�[���ϑ��X�V(�C��)���A���̍ۂ̃J���}���Q�C�������߂܂��B
     * �����I��UD�����𗘗p���Ă��܂��B
     * 
     * @param H @f$ H @f$�s��(�ϑ��s��)
     * @param R �ϑ��l�̌덷�����U�s��@f$ R @f$
     * @return (Matrix<FloatT>) �J���}���Q�C��@f$ K @f$
     */
    Matrix<FloatT> correct(const Matrix<FloatT> &H, const Matrix<FloatT> &R){
#if DEBUG
      std::cerr << "correct_KF_K:" << KalmanFilter<FloatT>::correct(H, R) << std::endl;
      std::cerr << "correct_KF_P:" << KalmanFilter<FloatT>::m_P << std::endl;
#endif
      
      // �J���}���Q�C��
      Matrix<FloatT> K(KalmanFilter<FloatT>::m_P.rows(), R.rows());
      
      for(int k = 0; k < R.rows(); k++){
        Matrix<FloatT> f(m_U.columns(), H.rows());
        Matrix<FloatT> g(m_D.rows(), f.columns());
        
        // f�̐���
        for(int i = 0; i < f.rows(); i++){
          for(int j = 0; j <= i; j++){
            f(i, 0) += H(k, j) * m_U(j, i);
          }
        }
        
        // g�̐���
        for(int i = 0; i < g.rows(); i++){
          g(i, 0) = m_D(i, i) * f(i, 0);
        }
        
        FloatT r(R(k, k));
        FloatT alpha = r + f(0, 0) * g(0, 0);
        K(0, k) = g(0, 0);
        m_D(0, 0) *= (r / alpha);
        
        for(int j = 1; j < f.rows(); j++){
          FloatT _alpha(alpha + f(j, 0) * g(j, 0));
          m_D(j, j) *= (alpha / _alpha);
          FloatT lambda(f(j, 0) / alpha);
          Matrix<FloatT> _u(m_U.columnVector(j).copy());
          m_U.columnVector(j) -= lambda * K.columnVector(k);
          K.columnVector(k) += g(j, 0) * _u;
          alpha = _alpha;
        }
        K.columnVector(k) /= alpha;
      }
      
      //�s��P�̍X�V
      need_update_P = true;

#if DEBUG
      std::cerr << "correct_UDKF_K:" << K << std::endl;
      std::cerr << "correct_UDKF_P:" << getP() << std::endl;
#endif

      return K;
    }
    
    /**
     * �덷�����U�s��@f$ P @f$��UD�������������̍s��@f$ U @f$��Ԃ��܂��B
     * 
     * @return (Matrix<FloatT>) �s��@f$ U @f$
     */
    const Matrix<FloatT> &getU() const {return m_U;}
    
    /**
     * �덷�����U�s��@f$ P @f$��UD�������������̍s��@f$ D @f$��Ԃ��܂��B
     * 
     * @return (Matrix<FloatT>) �s��@f$ D @f$
     */
    const Matrix<FloatT> &getD() const {return m_D;}
};

/**
 * @brief UnscentedKalman Filter
 * 
 * Unscented Kalman Filter���`���Ă��܂��B
 * 
 * @param FloatT ���Z���x
 * @see KalmanFilter
 */
template <class FloatT>
class UnscentedKalmanFilter : public KalmanFilter<FloatT>{
  protected:
    FloatT m_alpha, m_beta, m_kappa;
    
    bool need_recalc_coef;
    FloatT gamma, lambda;
    unsigned n_a;
    FloatT weightM_0, weightC_0, weight_i;
    Matrix<FloatT> m_sqrtQ;
    
    /**
     * �����U�s���sqrt(��������Ɏ����ƂȂ�)�����߂܂��B
     * 
     * @param cov �����U�s��
     * @return (Matrix<FloatT>) sqrt���ꂽ�s��
     */
    Matrix<FloatT> get_sqrt_cov(const Matrix<FloatT> &cov){
      Matrix<FloatT> sqrt_cov(cov.rows(), cov.columns());
      
      if(cov.isDiagonal()){   // �Ίp�s��̏ꍇ�A������
        for(int i(0); i < sqrt_cov.rows(); i++){
          sqrt_cov(i, i) = sqrt(cov(i, i));
        }
      }else{  // �����łȂ��ꍇ�͂܂��߂Ɍv�Z����
        Matrix<Complex<FloatT> > sqrt_cov_C(cov.sqrt());
        for(int i(0); i < sqrt_cov_C.rows(); i++){
          for(int j(0); j < sqrt_cov_C.columns(); j++){
            sqrt_cov(i, j) = sqrt_cov_C(i, j).real();
          }
        }
      }
      return sqrt_cov;
    }
    
    /**
     * �W���Ȃǃt�B���^�[���Z�ɕK�v�Ȓl�̃L���b�V�����X�V���܂��B
     * 
     */
    void recalc_coef(){
      if(!need_recalc_coef){return;}
      
      n_a = KalmanFilter<FloatT>::m_P.rows();
      lambda = pow(m_alpha, 2) * (m_kappa + n_a) - n_a;
      FloatT gamma2(lambda + n_a);
      gamma = ::sqrt(gamma2);
      weightM_0 = lambda / gamma2;
      weightC_0 = lambda / gamma2 + (FloatT(1) - pow(m_alpha, 2) + m_beta);
      weight_i = FloatT(1) / (gamma2 * 2);
      
      m_sqrtQ = get_sqrt_cov(KalmanFilter<FloatT>::m_Q);
      
      need_recalc_coef = false;
    }
    
  public:
    /**
     * UnscentedKalmanFilter�̃R���X�g���N�^�B
     * �덷�����U�s��@f$ P @f$, @f$ Q @f$���w�肷��K�v������܂��B
     * 
     * @param P @f$ P @f$�s��
     * @param Q @f$ Q @f$�s��
     */
    UnscentedKalmanFilter(const Matrix<FloatT> &P, const Matrix<FloatT> &Q)
        : KalmanFilter<FloatT>(P, Q),
          m_alpha(1),   // typically 0.001 - 1 (P.239, �ȉ�����) 
          m_beta(2),    // the optiomal value for Gaussian distribution
          m_kappa(0),   // 0 or 3 - n_a
          need_recalc_coef(true), m_sqrtQ(){
    }
    
    /**
     * �R�s�[�R���X�g���N�^
     * 
     * @param orig �R�s�[��
     * @param deepcopy �f�B�[�v�R�s�[���쐬���邩�ǂ���
     */
    UnscentedKalmanFilter(const UnscentedKalmanFilter &orig, const bool &deepcopy = false)
        : KalmanFilter<FloatT>(orig, deepcopy),
          m_alpha(orig.m_alpha), m_beta(orig.m_beta), m_kappa(orig.m_kappa), 
          need_recalc_coef(true), m_sqrtQ(){
      //std::cerr << "UKF" << std::endl;
    }
    
    /**
     * @f$ \alpha @f$ ���擾���܂�
     * 
     * @return (FloatT &)
     */
    FloatT &alpha(){
      need_recalc_coef = true;
      return m_alpha;
    }
    
    /**
     * @f$ \beta @f$ ���擾���܂�
     * 
     * @return (FloatT &)
     */
    FloatT &beta(){
      need_recalc_coef = true;
      return m_beta;
    } 
    
    /**
     * @f$ \kappa @f$ ���擾���܂�
     * 
     * @return (FloatT &)
     */
    FloatT &kappa(){
      need_recalc_coef = true;
      return m_kappa;
    }
    
    /**
     * �덷�����U�s��@f$ P @f$��ݒ肵�܂��B
     *
     * @param P �V����@f$ P @f$�s��
     */
    void setP(const Matrix<FloatT> &P){
      need_recalc_coef = true;
      KalmanFilter<FloatT>::m_P = P;
    }

    /**
     * �덷�����U�s��@f$ Q @f$��ݒ肵�܂��B
     *
     * @param Q �V����@f$ Q @f$�s��
     */
    void setQ(const Matrix<FloatT> &Q){
      need_recalc_coef = true;
      KalmanFilter<FloatT>::m_Q = Q;
    }
    
    /**
     * KalmanFilter�̃f�X�g���N�^�B
     * 
     */
    ~UnscentedKalmanFilter(){}
    
  protected:
    template <class StateValues>
    void get_perturbed_states(StateValues &state, StateValues *state_with_perturbation){
      Matrix<Complex<FloatT> > sqrtP(KalmanFilter<FloatT>::m_P.sqrt());
      for(unsigned k(0); k < n_a; k++){
        for(unsigned i(0); i < n_a; i++){
          FloatT perturbation(sqrtP(i, k).real());
          state_with_perturbation[k][i] = state[i] + gamma * perturbation;
          state_with_perturbation[k + n_a][i] = state[i] - gamma * perturbation;
        }
      }
    }
  
  public:
    /**
     * ��ԗʂƃt�B���^�[�����ԍX�V���܂��B
     * 
     * @param functor ���ԍX�V�֐�(2�܂���3�������Ƃ�operator()����`����Ă��邱��)
     * @param state ��ԗ�([]����`����Ă��邱��)
     * @param input �V�X�e���ւ̓���
     */
    template <class TimeUpdateFunctor, class StateValues, class InputValues>
    void predict(TimeUpdateFunctor &functor, StateValues &state, InputValues &input){
      recalc_coef();
      
      // �΍����������ꂽ��ԗ�(�V�O�}�|�C���g)���v�Z
      StateValues *state_sigma(new StateValues [n_a * 2]);
      get_perturbed_states(state, state_sigma);
      
      // ���̃X�e�b�v�̌v�Z��mean�̌v�Z(��ԗʂ̍X�V)
      StateValues state0_next = functor(state, input);
      for(unsigned i(0); i < n_a; i++){
        state[i] = weightM_0 * state0_next[i];
      }
      for(unsigned k(0); k < n_a; k++){
        state_sigma[k] = functor(state_sigma[k], input, m_sqrtQ);
        state_sigma[k + n_a] = functor(state_sigma[k + n_a], input, -m_sqrtQ);
        for(unsigned i(0); i < n_a; i++){
          state[i] += weight_i * state_sigma[k][i];
          state[i] += weight_i * state_sigma[k + n_a][i];
        }
      }
      
      // cov�̌v�Z
      {
        Matrix<FloatT> P_vec(n_a, 1);
        
        // i = 0
        for(unsigned i(0); i < n_a; i++){
          P_vec(i, 0) = state0_next[i] - state[i];
        }
        KalmanFilter<FloatT>::m_P = P_vec * P_vec.transpose() * weightC_0;
        
        // i > 0
        for(unsigned k(0); k < n_a * 2; k++){
          for(unsigned i(0); i < n_a; i++){
            P_vec(i, 0) = state_sigma[k][i] - state[i];
          }
          KalmanFilter<FloatT>::m_P += P_vec * P_vec.transpose() * weight_i;
        }
      }
      
      delete [] state_sigma;
    }
    
    /**
     * ��ԗʂƃt�B���^�[���ϑ��X�V(�C��)���A���̍ۂ̃J���}���Q�C�������߂܂��B
     * 
     * @param functor �ϑ�������(1�������Ƃ�operator()����`����Ă��邱��)
     * @param state ��ԗ�([]����`����Ă��邱��)
     * @param z �ϑ���([]�Avariables����`����Ă��邱��)
     * @return R �ϑ��l�̌덷�����U�s��@f$ R @f$
     */
    template <class ObserverationFunctor, class StateValues, class ObservedValues>
    Matrix<FloatT> correct(ObserverationFunctor &functor, 
        StateValues &state,
        const ObservedValues &z,
        const Matrix<FloatT> &R){
      
      recalc_coef();
      
      // �΍����������ꂽ��ԗ�(�V�O�}�|�C���g)���v�Z
      StateValues *state_sigma(new StateValues [n_a * 2]);
      get_perturbed_states(state, state_sigma);
      
      // �\���ϑ��ʂ̌v�Z
      ObservedValues y_from_state0 = functor(state);
      ObservedValues *y_from_sigma(new ObservedValues [n_a * 2]);
      
      unsigned n_y(ObservedValues::variables());
      
      // y_mean�̌v�Z
      ObservedValues y_mean;
      for(unsigned i(0); i < n_y; i++){
        y_mean[i] = weightM_0 * y_from_state0[i];
      }
      for(unsigned k(0); k < n_a; k++){
        y_from_sigma[k] = functor(state_sigma[k]);
        y_from_sigma[k + n_a] = functor(state_sigma[k + n_a]);
        for(unsigned i(0); i < n_y; i++){
          y_mean[i] += weight_i * y_from_sigma[k][i];
          y_mean[i] += weight_i * y_from_sigma[k + n_a][i];
        }
      }
      
      // P_yy, P_xy�̌v�Z
      Matrix<FloatT> P_y(n_y, 1);
      for(unsigned i(0); i < n_y; i++){
        P_y(i, 0) = y_from_state0[i] - y_mean[i];
      }
      
      Matrix<FloatT> P_yy(P_y * P_y.transpose() * weightC_0);
      Matrix<FloatT> P_xy(KalmanFilter<FloatT>::m_P.rows(), P_yy.columns());
      for(unsigned i(0); i < n_a * 2; i++){
        Matrix<FloatT> P_x(P_xy.rows(), 1);
        for(unsigned i2(0); i2 < n_a; i2++){
          P_x(i2, 0) = state_sigma[i][i2] - state[i2];
        }
        for(unsigned i2(0); i2 < n_y; i2++){
          P_y(i2, 0) = y_from_sigma[i][i2] - y_mean[i2];
        }
        
        P_yy += P_y * P_y.transpose() * weight_i;
        P_xy += P_x * P_y.transpose() * weight_i;
      }
      P_yy += R;
      
      // �J���}���Q�C��
      Matrix<FloatT> K(P_xy * P_yy.inverse());
      
      // ��ԗ�, P�̏C��
      Matrix<FloatT> delta_z(n_y, 1);
      for(unsigned i(0); i < n_y; i++){
        delta_z(i, 0) = const_cast<ObservedValues &>(z)[i] - y_mean[i];
      }
      
      Matrix<FloatT> mod_x(K * delta_z);
      for(unsigned i(0); i < n_a; i++){
        state[i] += mod_x(i, 0);
      }
      KalmanFilter<FloatT>::m_P -= K * P_yy * K.transpose();
      
      delete [] state_sigma;
      delete [] y_from_sigma;

      return K;
    }
};

#endif /* __KALMAN_H__ */
