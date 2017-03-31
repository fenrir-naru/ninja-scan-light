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

#ifndef __QUATERNION_H
#define __QUATERNION_H

/** @file
 * @brief �N�H�[�^�j�I��(Quaternion�A4����)���C�u����
 * 
 * �N�H�[�^�j�I�����`�������C�u�����B
 */

#include <cmath>
#include "param/matrix.h"

#include "param/vector3.h"

template <class FloatT>
struct QuaternionDataProperty{
  
#if defined(_MSC_VER) || defined(__TI_COMPILER_VERSION__)
  static const unsigned int OUT_OF_INDEX = 4; ///< �ő�v�f��(4)
#else
  static const unsigned int OUT_OF_INDEX; ///< �ő�v�f��
#endif
};

#if !(defined(_MSC_VER) || defined(__TI_COMPILER_VERSION__))
template <class FloatT>
const unsigned int QuaternionDataProperty<FloatT>::OUT_OF_INDEX
    = 4; ///< �ő�v�f��
#endif

template <class FloatT>
class QuaternionData : public QuaternionDataProperty<FloatT> {
  protected:
    typedef QuaternionData<FloatT> self_t;
  private:
    struct storage_t{
      FloatT scalar;  ///< �X�J���[�v�f
      Vector3<FloatT> vector; ///< �x�N�g���v�f
      int ref;  ///< �Q�ƃJ�E���^
      storage_t() : ref(1) {}
      storage_t(const FloatT &q0, const Vector3<FloatT> &v)
          : scalar(q0), vector(v), ref(1) {}
      storage_t(
          const FloatT &q0, const FloatT &q1, 
          const FloatT &q2, const FloatT &q3)
          : scalar(q0), vector(q1, q2, q3), ref(1) {}
    } *storage;  ///< �X�g���[�W
  protected:
    /**
     * �R���X�g���N�^
     * 
     */
    QuaternionData() : storage(new storage_t()){}
    
    /**
     * �R���X�g���N�^
     * 
     * @param q0 �X�J���[�v�f
     * @param v 3�����x�N�g���v�f
     */
    QuaternionData(const FloatT &q0, const Vector3<FloatT> &v)
        : storage(new storage_t(q0, v)){}
    
    /**
     * �R���X�g���N�^
     * 
     * @param q0 �X�J���[�v�f
     * @param q1 3�����x�N�g���v�f��(X)
     * @param q2 3�����x�N�g���v�f��(Y)
     * @param q3 3�����x�N�g���v�f��(Z)
     */
    QuaternionData(
        const FloatT &q0, const FloatT &q1, 
        const FloatT &q2, const FloatT &q3)
        : storage(new storage_t(q0, q1, q2, q3)) {}
    
    /**
     * �R�s�[�R���X�g���N�^
     * 
     * �V�����[�R�s�[���s���܂��B
     * 
     * @param q �R�s�[��
     */
    QuaternionData(const self_t &q){
      if(storage = q.storage){(storage->ref)++;}
    }
    
    /**
     * ������Z�q
     * 
     * �V�����[�R�s�[�őΉ����Ă��܂��B
     * 
     * @param q �R�s�[��
     */
    self_t &operator=(const self_t &q){
      if(this == &q){return *this;}
      if(storage && ((--(storage->ref)) <= 0)){delete storage;}
      if(storage = q.storage){(storage->ref)++;}
      return *this;
    }
    
    /**
     * �f�B�[�v�R�s�[���s���܂��B
     * 
     * @return (Quarterion) �R�s�[
     */
    self_t deep_copy() const{
      return self_t(
          storage->scalar,
          storage->vector.copy());
    }
    
  public:
    /**
     * �f�X�g���N�^
     * 
     * �Q�ƃJ�E���^�����Z���܂��B
     * �����Q�ƃJ�E���^��0�̏ꍇ�A�g�p���Ă�����������������܂��B
     */
    ~QuaternionData(){
      if(storage && ((--(storage->ref)) <= 0)){
        delete storage;
      }
    }
    
    /**
     * �X�J���[�v�f��Ԃ��܂��B
     * 
     * @return (T) �X�J���[�v�f
     */
    const FloatT &scalar() const {return storage->scalar;}
    FloatT &scalar(){
      return const_cast<FloatT &>(static_cast<const self_t &>(*this).scalar());
    }
    /**
     * �x�N�g���v�f��Ԃ��܂��B
     * 
     * @return (Vector<FloatT>) �x�N�g���v�f
     */
    const Vector3<FloatT> &vector() const {return storage->vector;}
    Vector3<FloatT> &vector(){
      return const_cast<Vector3<FloatT> &>(static_cast<const self_t &>(*this).vector());
    }
    
    /**
     * �v�f(�̎Q��)��Ԃ��܂��B
     * �]���đ�����\�ł��B
     * 
     * @param index �v�f�ԍ��A0:�X�J���[�v�f,1�`3:3�����x�N�g���v�fX�`Z
     * @return (FloatT &) �v�f�ւ̎Q��
     */
    const FloatT &operator[](const unsigned int &index) const {
      if(index == 0){return storage->scalar;}
      else{return (storage->vector)[index - 1];}
    }
    FloatT &operator[](const unsigned int &index){
      return const_cast<FloatT &>(static_cast<const self_t &>(*this)[index]);
    }
};

/**
 * @brief �N�H�[�^�j�I��
 * 
 * �N�H�[�^�j�I��@f$ \Tilde{q} \f$�̓X�J���[�v�f@f$ q_{0} \f$��
 * 3�����x�N�g���v�f@f$ \vec{q} \f$����Ȃ�
 * @f[
 *    \Tilde{q} \equiv \begin{Bmatrix} q_{0} \\ \vec{q} \end{Bmatrix}
 * @f]
 * �ƕ\������܂��B
 * 
 * ���ɑS�v�f�̓��a��1�ƂȂ�P�ʃN�H�[�^�j�I����p����ƁA
 * 3������ł̎p���p��\���ۂ� * �I�C���[�p�Ő�����
 * @f$ \tan{\frac{\pi}{2}} @f$�Ƃ��������ٓ_��������邱�Ƃ��ł��A
 * ���炩�ȉ��Z���s�����Ƃ��\�ł��B
 * 
 * �Ȃ��A�����I�ɎQ�ƃJ�E���^�𗘗p�������C�g�E�G�C�g�Ȏ����ɂȂ��Ă��邽�߁A
 * �������≉�Z�񐔂��ߖ񂳂�邱�Ƃ��@�̂���܂��B
 * ���̂��߂ɖ����I��copy()���\�b�h���g�p���Ȃ��ƃf�B�[�v�R�s�[������Ȃ��̂ŁA
 * �ė��p���s���ۂ͒��ӂ��Ă��������B
 * 
 * @param FloatT ���Z���x�Adouble�Ȃ�
 * @see Vector3<FloatT>
 */
template <class FloatT>
class Quaternion : public QuaternionData<FloatT> {
  protected:
    typedef Quaternion<FloatT> self_t;
    typedef QuaternionData<FloatT> super_t;
    
    Quaternion(const super_t &q) : super_t(q) {}
    
  public:
    using super_t::OUT_OF_INDEX;
    using super_t::operator[];
    using super_t::scalar;
    using super_t::vector;
    
    /**
     * �R���X�g���N�^
     * �S�v�f��0�ŏ��������܂��B
     */
    Quaternion()
        : super_t(FloatT(0), FloatT(0), FloatT(0), FloatT(0)) {}
    
    /**
     * �R���X�g���N�^
     * 
     * @param q0 �X�J���[�v�f
     * @param v 3�����x�N�g���v�f
     */
    Quaternion(const FloatT &q0, const Vector3<FloatT> &v)
        : super_t(q0, v){}
    
    /**
     * �R���X�g���N�^
     * 
     * @param q0 �X�J���[�v�f
     * @param q1 3�����x�N�g���v�f��(X)
     * @param q2 3�����x�N�g���v�f��(Y)
     * @param q3 3�����x�N�g���v�f��(Z)
     */
    Quaternion(
        const FloatT &q0, const FloatT &q1,
        const FloatT &q2, const FloatT &q3)
        : super_t(q0, q1, q2, q3) {}
    
    /**
     * �f�X�g���N�^
     * 
     */
    ~Quaternion(){}
    
    /**
     * �R�s�[�R���X�g���N�^
     * 
     * @param q �R�s�[��
     */
    Quaternion(const self_t &q) : super_t(q) {
      
    }
    
    /**
     * ������Z�q
     * 
     * @param q �R�s�[��
     */
    self_t &operator=(const self_t &q){
      super_t::operator=(q);
      return *this;
    }
    
    /**
     * �f�B�[�v�R�s�[���s���܂��B
     * 
     * @return (Quarterion) �R�s�[
     */
    self_t copy() const{
      return self_t(super_t::deep_copy());
    }
    
    /**
     * �v�f��ݒ肵�܂��B
     * �v�f�ԍ��̒�`��operator[](unsigned int)�ɂ���Ē�`����Ă��܂��B
     * 
     * @param index �v�f�ԍ�
     * @param value �ݒ肷��l
     * @see operator[](unsigned int)
     */
    void set(const unsigned int &index, const FloatT &value){(*this)[index] = value;}
    /**
     * �v�f���擾���܂��B
     * �v�f�ԍ��̒�`��operator[](unsigned int) const�ɂ���Ē�`����Ă��܂��B
     * 
     * @param index �v�f�ԍ�
     * @return FloatT �v�f
     * @see operator[](unsigned int) const
     */
    const FloatT &get(const unsigned int &index) const{return (*this)[index];}
    
    /**
     * �����N�H�[�^�j�I�������߂܂��B
     * �����N�H�[�^�j�I��@f$ \Tilde{q}^{*} \f$��
     * @f[
     *     \Tilde{q}^{*} \equiv \begin{Bmatrix} q_{0} \\ - \vec{q} \end{Bmatrix}
     * @f]
     * �Œ�`����܂��B
     * �Ȃ��A���߂�ۂɃf�B�[�v�R�s�[������Ă���̂ŁA�Ȍ�̕ԋp�l�ɑ΂��鑀��ɂ��āA
     * ���̃N�H�[�^�j�I���͔�j��ƂȂ�܂��B
     * 
     * @return (Quarterion<FloatT>) �����N�H�[�^�j�I��
     */
    self_t conj() const{
      return self_t(scalar(), -vector());
    }

#ifndef pow2
#define pow2(x) ((x) * (x))
#else
#define POW2_ALREADY_DEFINED
#endif
    /**
     * �v�f�̓��a@f$ \left| \Tilde{q} \right|^{2} @f$�����߂܂��B
     * @f[
     *     \left| \Tilde{q} \right|^{2} \equiv q_{0} {}^{2} + \left| \vec{q} \right|^{2}
     * @f]
     * �Œ�`����܂��B
     * 
     * @return (FloatT) ����
     */
    FloatT abs2() const{
      return pow2(scalar()) + vector().abs2();
    }
#ifndef POW2_ALREADY_DEFINED
#undef pow2
#else
#undef POW2_ALREADY_DEFINED
#endif
    /**
     * �v�f�̓��a�̕�����(�m����)�����߂܂��B
     * 
     * @return (FloatT) ����
     * @see abs2()
     */
    FloatT abs() const{return std::sqrt(abs2());}

    /**
     * �X�J���[�Ƃ̐ώZ���s���܂��B�j��I�ł��B
     * 
     * @return (Quaternion<FloatT>) ����
     */
    self_t &operator*=(const FloatT &t){
      scalar() *= t;
      vector() *= t;
      return *this;
    }
    
    /**
     * �X�J���[�Ƃ̐ώZ���s���܂��B
     * 
     * @return (Quaternion<FloatT>) ����
     */
    self_t operator*(const FloatT &t) const{return copy() *= t;}
    
    /**
     * �X�J���[�Ƃ̏��Z���s���܂��B�j��I�ł��B
     * 
     * @return (Quaternion<FloatT>) ����
     */
    self_t &operator/=(const FloatT &t){return (*this) *= (FloatT(1) / t);}
    
    /**
     * �X�J���[�Ƃ̏��Z���s���܂��B
     * 
     * @return (Quaternion<FloatT>) ����
     */
    self_t operator/(const FloatT &t) const{return copy() /= t;}
    
    /**
     * �m������1�ɂȂ�悤�ɐ��K�����܂��B
     * 
     * @return (Quaternion<FloatT>) ����
     * @see abs()
     */
    self_t regularize() const{return (*this) / abs();}
    
    /**
     * �N�H�[�^�j�I���Ƃ̐ώZ���s���܂��B
     * �ώZ @f$ \Tilde{q}_{a} \Tilde{q}_{b} @f$��
     * @f[
     *    \Tilde{q}_{a} \Tilde{q}_{b}
     *      \equiv \begin{Bmatrix} q_{0a} \\ \vec{q}_{a} \end{Bmatrix}
     *        \begin{Bmatrix} q_{0b} \\ \vec{q}_{b} \end{Bmatrix}
     *      \equiv \begin{Bmatrix}
     *          q_{0a} q_{0b} - \vec{q}_{a} \cdot \vec{q}_{b} \\
     *          q_{0a} \vec{q}_{b} + q_{0b} \vec{q}_{a} + \vec{q}_{a} \times \vec{q}_{b} 
     *        \end{Bmatrix}
     * @f] 
     * �Œ�`����܂��B
     * 
     * @return (Quaternion<FloatT>) ����
     */
    self_t operator*(const self_t &q) const{
      self_t result((
          scalar() * q.scalar())
            - (vector().innerp(q.vector())),
          (q.vector() * scalar())
            += (vector() * q.scalar())
            += (vector() * q.vector()));
      return result;
    }
    
    /**
     * �N�H�[�^�j�I���Ƃ̐ώZ���s���܂��B�j��I�ł��B
     * ��`��operator*(const Quaternion &) const���Q�Ƃ��Ă��������B
     * 
     * @return (Quaternion<FloatT>) ����
     * @see operator*(const Quaternion<FloatT> &) const
     */
    self_t &operator*=(const self_t &q){return (*this) = (*this) * q;}
    
    /**
     * �N�H�[�^�j�I���Ƃ̉��Z���s���܂��B�j��I�ł��B
     * ���Z @f$ \Tilde{q}_{a} + \Tilde{q}_{b} @f$��
     * @f[
     *    \Tilde{q}_{a} + \Tilde{q}_{b}
     *      \equiv \begin{Bmatrix} q_{0a} \\ \vec{q}_{a} \end{Bmatrix}
     *        + \begin{Bmatrix} q_{0b} \\ \vec{q}_{b} \end{Bmatrix}
     *      \equiv \begin{Bmatrix}
     *          q_{0a} + q_{0b} \\
     *          \vec{q}_{a} + \vec{q}_{b} 
     *        \end{Bmatrix}
     * @f] 
     * �Œ�`����܂��B
     * 
     * @return (Quaternion<FloatT>) ����
     */
    self_t &operator+=(const self_t &q){
      for(unsigned int i(0); i < OUT_OF_INDEX; i++){(*this)[i] += q[i];}
      return *this;
    }
    /**
     * �N�H�[�^�j�I���Ƃ̉��Z���s���܂��B
     * ��`��operator+=(const Quaternion<FloatT> &) const���Q�Ƃ��Ă��������B
     * 
     * @return (Quaternion<FloatT>) ����
     * @see operator+=(const Quaternion<FloatT> &)
     */
    self_t operator+(const self_t &q) const{return copy() += q;}
    
    /**
     * �N�H�[�^�j�I���Ƃ̌��Z���s���܂��B�j��I�ł��B
     * ���Z @f$ \Tilde{q}_{a} - \Tilde{q}_{b} @f$��
     * @f[
     *    \Tilde{q}_{a} - \Tilde{q}_{b}
     *      \equiv \begin{Bmatrix} q_{0a} \\ \vec{q}_{a} \end{Bmatrix}
     *        - \begin{Bmatrix} q_{0b} \\ \vec{q}_{b} \end{Bmatrix}
     *      \equiv \begin{Bmatrix}
     *          q_{0a} - q_{0b} \\
     *          \vec{q}_{a} - \vec{q}_{b} 
     *        \end{Bmatrix}
     * @f] 
     * �Œ�`����܂��B
     * 
     * @return (Quaternion<FloatT>) ����
     */
    self_t &operator-=(const self_t &q){
      for(unsigned int i(0); i < OUT_OF_INDEX; i++){(*this)[i] -= q[i];}
      return *this;
    }
    /**
     * �N�H�[�^�j�I���Ƃ̌��Z���s���܂��B
     * ��`��operator-=(const Quaternion<FloatT> &)���Q�Ƃ��Ă��������B
     * 
     * @return (Quaternion<FloatT>) ����
     * @see operator-=(const Quaternion<FloatT> &)
     */
    self_t operator-(const self_t &q) const{return copy() -= q;}
    
    /**
     * 3�����x�N�g���Ƃ̐ώZ���s���܂��B�j��I�ł��B
     * �ώZ @f$ \Tilde{q} \vec{v} @f$��
     * @f[
     *    \Tilde{q} \vec{v}
     *      \equiv \begin{Bmatrix} q_{0} \\ \vec{q} \end{Bmatrix}
     *        \begin{Bmatrix} 0 \\ \vec{v} \end{Bmatrix}
     *      \equiv \begin{Bmatrix}
     *          - \vec{q} \cdot \vec{v} \\
     *          q_{0} \vec{v} + \vec{q} \times \vec{v} 
     *        \end{Bmatrix}
     * @f] 
     * �Œ�`����܂��B
     * 
     * @return (Quaternion<FloatT>) ����
     */
    self_t &operator*=(const Vector3<FloatT> &v){
      FloatT temp_scalar(scalar());
      scalar() = -(vector().innerp(v));
      (vector() *= v) += v * temp_scalar;
      return (*this);
    }
    /**
     * 3�����x�N�g���Ƃ̐ώZ���s���܂��B
     * ��`��operator*=(const Vector3<FloatT> &)���Q�Ƃ��Ă��������B
     * 
     * @return (Quaternion<FloatT>) ����
     * @see operator*=(const Vector3<FloatT> &)
     */
    self_t operator*(const Vector3<FloatT> &v) const{return copy() *= v;}
    
    /**
     * ��]�p�̔��������߂܂��B
     * 
     * @return (T) ����
     */
    FloatT getTheta_2() const{return std::acos(regularize()[0]);}
    /**
     * ��]�p�����߂܂��B
     * 
     * @return (T) ����
     * @see getTheta_2()
     */
    FloatT getTheta() const{return getTheta_2() * 2;}
    /**
     * ��]���̍��W�����߂܂��B
     * 
     * @return (Vector3<FloatT>) ����
     */
    Vector3<FloatT> getAxis() const{
      Vector3<FloatT> axis;
      self_t r(regularize());
      FloatT theta_2(r.getTheta_2());
      for(unsigned int i(0); i < Vector3<FloatT>::OUT_OF_INDEX; i++){
        axis[i] = (*this)[i + 1] / std::sin(theta_2);
      }
      return axis;
    }

#ifndef pow2
#define pow2(x) ((x) * (x))
#else
#define POW2_ALREADY_DEFINED
#endif
    /**
     * @f$ 3 \times 3 @f$ ��Direction Cosine Matrix(DCM)�ɕϊ����܂��B
     * 
     * @return (Matrix<FloatT>) DCM
     */
    Matrix<FloatT> getDCM() const{
      self_t r = regularize();
      Matrix<FloatT> dcm = Matrix<FloatT>(3, 3);
      {
        //dcm(0, 0) = pow2(r[0]) + pow2(r[1]) - pow2(r[2]) - pow2(r[3]);
        dcm(0, 0) = FloatT(1) - (pow2(r[2]) + pow2(r[3])) * 2;
        dcm(0, 1) = ((r[1] * r[2]) + (r[0] * r[3])) * 2;
        dcm(0, 2) = ((r[1] * r[3]) - (r[0] * r[2])) * 2;
        
        dcm(1, 0) = ((r[1] * r[2]) - (r[0] * r[3])) * 2;
        //dcm(1, 1) = pow2(r[0]) - pow2(r[1]) + pow2(r[2]) - pow2(r[3]);
        dcm(1, 1) = FloatT(1) - (pow2(r[1]) + pow2(r[3])) * 2;
        dcm(1, 2) = ((r[2] * r[3]) + (r[0] * r[1])) * 2;
        
        dcm(2, 0) = ((r[1] * r[3]) + (r[0] * r[2])) * 2;
        dcm(2, 1) = ((r[2] * r[3]) - (r[0] * r[1])) * 2;
        //dcm(2, 2) = pow2(r[0]) - pow2(r[1]) - pow2(r[2]) + pow2(r[3]);
        dcm(2, 2) = FloatT(1) - (pow2(r[1]) + pow2(r[2])) * 2; 
      }
      return dcm;
    }
#ifndef POW2_ALREADY_DEFINED
#undef pow2
#else
#undef POW2_ALREADY_DEFINED
#endif

    /**
     * Quaternion�����₷���`�ŏo�͂��܂��B
     * 
     * @param out �o�̓X�g���[��
     * @param q �o�͑Ώ�
     * @return (ostream) �o�̓X�g���[��
     */
    friend std::ostream &operator<<(std::ostream &out, const self_t &q){
      for(unsigned int i(0); i < OUT_OF_INDEX; i++){
        out << (i == 0 ? "{" : ",") << q[i];
      }
      out << "}";
      return out;
    }
    
    /**
     * @f$ 4 \times 1 @f$�s�񂠂邢��@f$ 1 \times 4 @f$�s��A
     * ���邢��DCM�̏����𖞂���@f$ 3 \times 3 @f$�s���
     * Quaternion�^�ɕϊ����܂��B
     * 
     * @param matrix �s��
     * @throws MatrixException �s��̃T�C�Y���������Ȃ��Ƃ�
     */
    Quaternion(const Matrix<FloatT> &matrix) throw(MatrixException)
        : super_t() {
      
      Matrix<FloatT> &m(const_cast<Matrix<FloatT> &>(matrix));
      
      if(matrix.rows() == OUT_OF_INDEX && matrix.columns() == 1){
        for(int i(0); i < OUT_OF_INDEX; i++){(*this)[i]= m(i, 0);}
      }else if(matrix.rows() == 1 && matrix.columns() == OUT_OF_INDEX){
        for(int i(0); i < OUT_OF_INDEX; i++){(*this)[i]= m(0, i);}
      }else if((matrix.rows() == 3) && (matrix.columns() == 3)){
        
        // TODO: DCM�̏����𖞂����Ă��邩�A���ׂ邱��
        
        // �Ίp�v�f�𑫂��� 3 q_0^2 - (q_1^2 + q_2^2 + q_3^2) == 4 q_0^2 - 1 �ł���
        // q_0 >= 0�ł��邩��q_0����Ԃ͂��߂ɂƂ���        
        (*this)[0] = std::sqrt(((m(0, 0) + m(1, 1) + m(2, 2)) + 1) / 4);
        
        if((*this)[0] > 1E-10){
          // ��Ίp�v�f����c������߂�
          (*this)[1] = (m(1, 2) - m(2, 1)) / 4 / (*this)[0];
          (*this)[2] = (m(2, 0) - m(0, 2)) / 4 / (*this)[0];
          (*this)[3] = (m(0, 1) - m(1, 0)) / 4 / (*this)[0];
        }else{  // 0�ɋ߂��Ƃ���܂�悭�Ȃ�
          // �Ίp�v�f(0)+(1)-(2) => (q_0^2 + q_1^2 + q_2^2) - 3 q_3^2 == 1 - 4 q_3^2 �ł���
          // ���̂Ƃ�q_3�𐳂Ƃ���
          (*this)[3] = std::sqrt(((m(0, 0) + m(1, 1) - m(2, 2)) - 1) / -4);
          if((*this)[3] > 1E-10){
            (*this)[1] = (m(2, 0) + m(0, 2)) / 4 / (*this)[3];
            (*this)[2] = (m(2, 1) + m(1, 2)) / 4 / (*this)[3];
          }else{ // ���܂菬�����Ɩ��
            // �Ίp�v�f���狁�߂�
            // �Ίp�v�f-(0)+(1)+(2) => (q_0^2 + q_2^2 + q_3^2) - 3 q_1^2 == 1 - 4 q_1^2
            // �Ίp�v�f(0)-(1)+(2) => (q_0^2 + q_1^2 + q_3^2) - 3 q_2^2 == 1 - 4 q_2^2
            // ���̂Ƃ�q_2�𐳂Ƃ��Aq_1�̕����͔�Ίp�v�f���狁�߂�
            (*this)[1] = std::sqrt(((-m(0, 0) + m(1, 1) + m(2, 2)) - 1) / -4);
            (*this)[2] = std::sqrt(((m(0, 0) - m(1, 1) + m(2, 2)) - 1) / -4);
            if(m(0, 1) + m(1, 0) < 0){(*this)[1] *= -1;}
          }
        } 
      }else{
        throw MatrixException("Operatiorn void!! ; Need Matrix(4, 1) or Matrix(1, 4) or DCM_Matrix(3, 3)");
      }
    }
    
    /**
     * @f$ 4 \times 1 @f$�s��ɕϊ����܂��B
     * 
     * @return (Matrix<FloatT>) �s��
     */
    Matrix<FloatT> toMatrix() const{
      Matrix<FloatT> matrix = Matrix<FloatT>(OUT_OF_INDEX, 1);
      for(int i(0); i < OUT_OF_INDEX; i++){matrix(i, 0) = (*this)[i];}
      return matrix;
    }
};

#define QUATERNION_NO_FLY_WEIGHT(float_t) \
template <> \
class QuaternionData<float_t> : public QuaternionDataProperty<float_t> { \
  protected: \
    typedef QuaternionData<float_t> self_t; \
  private: \
    float_t _scalar; \
    Vector3<float_t> _vector; \
    \
  protected: \
    QuaternionData(){} \
    \
    QuaternionData(const float_t &q0, const Vector3<float_t> &v) \
        : _scalar(q0), _vector(v){} \
    \
    QuaternionData( \
        const float_t &q0, const float_t &q1, \
        const float_t &q2, const float_t &q3) \
        : _scalar(q0), _vector(q1, q2, q3) {} \
    \
    QuaternionData(const self_t &q) \
        : _scalar(q._scalar), _vector(q._vector){ \
      \
    } \
    \
    self_t &operator=(const self_t &q){ \
      _scalar = q._scalar; \
      _vector = q._vector; \
      return *this; \
    } \
    \
    self_t deep_copy() const{ \
      return self_t(_scalar, _vector.copy()); \
    } \
    \
  public: \
    ~QuaternionData(){} \
    \
    const float_t &scalar() const {return _scalar;} \
    float_t &scalar(){ \
      return const_cast<float_t &>(static_cast<const self_t &>(*this).scalar()); \
    } \
    \
    const Vector3<float_t> &vector() const {return _vector;} \
    Vector3<float_t> &vector(){ \
      return const_cast<Vector3<float_t> &>(static_cast<const self_t &>(*this).vector()); \
    } \
    \
    const float_t &operator[](const unsigned &index) const { \
      if(index == 0){return _scalar;} \
      else{return _vector[index - 1];} \
    } \
    float_t &operator[](const unsigned &index){ \
      return const_cast<float_t &>(static_cast<const self_t &>(*this)[index]); \
    } \
}

//QUATERNION_NO_FLY_WEIGHT(double);

#endif /* __QUATERNION_H */
