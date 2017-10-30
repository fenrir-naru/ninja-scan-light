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

#ifndef __VECTOR3_H
#define __VECTOR3_H

/** @file
 * @brief 3�����x�N�g�����C�u����
 * 
 * 3�����x�N�g�����`�������C�u�����B
 */

#include <cmath>
#include <cstring>
#include "param/matrix.h"

template <class FloatT>
struct Vector3DataProperty{
  /**
   * �C���f�b�N�X�̒�`
   * 
   */
  enum Index {
      X_INDEX = 0,  ///< X�v�f�̓C���f�b�N�X0
      Y_INDEX,      ///< Y�v�f�̓C���f�b�N�X1
      Z_INDEX};     ///< Z�v�f�̓C���f�b�N�X2
#if defined(_MSC_VER) || defined(__TI_COMPILER_VERSION__)
  static const unsigned int OUT_OF_INDEX = 3; ///< �ő�v�f��(3)
#else
  static const unsigned int OUT_OF_INDEX; ///< �ő�v�f��(3)
#endif
};

#if !(defined(_MSC_VER) || defined(__TI_COMPILER_VERSION__))
template <class FloatT>
const unsigned int Vector3DataProperty<FloatT>::OUT_OF_INDEX 
    = Vector3DataProperty<FloatT>::Z_INDEX + 1; ///< �ő�v�f��(3)
#endif

template <class FloatT>
class Vector3Data : public Vector3DataProperty<FloatT> {
  protected:
    typedef Vector3DataProperty<FloatT> super_t;
    typedef Vector3Data<FloatT> self_t;
    
  private:
    struct storage_t{
      FloatT values[super_t::OUT_OF_INDEX];  ///<�v�f�ۑ��p
      int ref;                      ///<�Q�ƃJ�E���^
      storage_t() : ref(1) {}
      storage_t(const FloatT &x, const FloatT &y, const FloatT &z)
          : ref(1) {
        values[super_t::X_INDEX] = x;
        values[super_t::Y_INDEX] = y;
        values[super_t::Z_INDEX] = z;
      }
    } *storage;  ///< �X�g���[�W
    
    Vector3Data(storage_t *_storage) : storage(_storage){
      
    }
    
  protected:
    /**
     * �R���X�g���N�^�B
     */
    Vector3Data() : storage(new storage_t()){
      
    }
    
    /**
     * �R���X�g���N�^�B
     * �w�肵���l�ŏ���������܂��B
     * 
     * @param x X�v�f�̒l
     * @param y Y�v�f�̒l
     * @param z Z�v�f�̒l
     */
    Vector3Data(const FloatT &x, const FloatT &y, const FloatT &z)
        : storage(new storage_t(x, y, z)){
      
    }
    
    /**
     * �R�s�[�R���X�g���N�^
     * 
     * �V�����[�R�s�[���s���܂��B
     * 
     * @param v �R�s�[��
     */
    Vector3Data(const self_t &v){
      if(storage = v.storage){(storage->ref)++;}
    }
    
    /**
     * ������Z�q
     * 
     * �V�����[�R�s�[�őΉ����Ă��܂��B
     * 
     * @param v �R�s�[��
     */
    self_t &operator=(const self_t &v){
      if(this == &v){return *this;}
      if(storage && ((--(storage->ref)) <= 0)){delete storage;}
      if(storage = v.storage){(storage->ref)++;}
      return *this;
    }
    
    /**
     * �f�B�[�v�R�s�[���s���܂��B
     * 
     * @return �R�s�[
     */
    self_t deep_copy() const {
      storage_t *copied(new storage_t());
      std::memcpy(
          copied->values, storage->values, 
          sizeof(FloatT[super_t::OUT_OF_INDEX]));
      return self_t(copied);
    }
  public:
    /**
     * �f�X�g���N�^�B
     * 
     * �Q�ƃJ�E���^�����Z���܂��B
     * �����Q�ƃJ�E���^��0�̏ꍇ�A�g�p���Ă�����������������܂��B
     */
    ~Vector3Data(){
      if(storage && ((--(storage->ref)) <= 0)){
        delete storage;
      }
    }
    
    /**
     * �v�f(�̎Q��)��Ԃ��܂��B
     * �]���đ�����\�ł��B
     * 
     * @param index �v�f�ԍ��A0�`2:�v�fX�`Z
     * @return (FloatT &) �v�f�ւ̎Q��
     */
    const FloatT &operator[](const unsigned int &index) const {
      //if(index => OUT_OF_INDEX){return NULL;}
      return (storage->values)[index];
    }
    FloatT &operator[](const unsigned int &index){
      return const_cast<FloatT &>(static_cast<const self_t &>(*this)[index]);
    }
};

/**
 * @brief 3�����x�N�g��
 * 
 * 3�����x�N�g���N���X�B
 * �x�N�g�����g�̒�`����ς�O�ςȂǂ��܂߂��l�X�ȉ��Z�̒�`���s���Ă��܂��B
 * 
 * �Ȃ��A�����I�ɎQ�ƃJ�E���^�𗘗p�������C�g�E�G�C�g�Ȏ����ɂȂ��Ă��邽�߁A
 * �������≉�Z�񐔂��ߖ񂳂�邱�Ƃ��@�̂���܂��B
 * ���̂��߂ɖ����I��copy()���\�b�h���g�p���Ȃ��ƃf�B�[�v�R�s�[������Ȃ��̂ŁA
 * �ė��p���s���ۂ͒��ӂ��Ă��������B
 * 
 * @param FloatT ���Z���x�Adouble�Ȃ�
 */
template <class FloatT>
class Vector3 : public Vector3Data<FloatT> {
  protected:
    typedef Vector3<FloatT> self_t;
    typedef Vector3Data<FloatT> super_t;
  
    Vector3(const super_t &v) : super_t(v){}
    
  public:
    /**
     * �R���X�g���N�^�B
     * �S�v�f��0�ŏ���������܂��B
     */
    Vector3() 
        : super_t(FloatT(0), FloatT(0), FloatT(0)){}
    
    /**
     * �R���X�g���N�^�B
     * �w�肵���l�ŏ���������܂��B
     * 
     * @param x X�v�f�̒l
     * @param y Y�v�f�̒l
     * @param z Z�v�f�̒l
     */
    Vector3(const FloatT &x, const FloatT &y, const FloatT &z)
        : super_t(x, y, z){}
    
    /**
     * �f�X�g���N�^�B
     * 
     */
    ~Vector3(){}
    
    /**
     * �R�s�[�R���X�g���N�^
     * 
     * @param v �R�s�[��
     */
    Vector3(const self_t &v) : super_t(v) {
      
    }
    
    /**
     * ������Z�q
     * 
     * @param v �R�s�[��
     */
    self_t &operator=(const self_t &v){
      super_t::operator=(v);
      return *this;
    }
    
    /**
     * �f�B�[�v�R�s�[���s���܂��B
     * 
     * @return (Vector3) �R�s�[
     */
    self_t copy() const{
      return self_t(super_t::deep_copy());
    }
    
    using super_t::X_INDEX;
    using super_t::Y_INDEX;
    using super_t::Z_INDEX;
    using super_t::OUT_OF_INDEX;
    using super_t::operator[];
    
    /**
     * �v�f��ݒ肵�܂��B
     * �v�f�ԍ��̒�`��operator[](const unsigned int &)�ɂ���Ē�`����Ă��܂��B
     * 
     * @param index �v�f�ԍ�
     * @param value �ݒ肷��l
     * @see operator[](const unsigned int &)
     */
    void set(const unsigned &index, const FloatT &value){(*this)[index] = value;}
    /**
     * X�v�f��ݒ肵�܂��B
     * @param x �ݒ肷��l
     */
    void setX(const FloatT &x){set(X_INDEX, x);}
    /**
     * Y�v�f��ݒ肵�܂��B
     * @param y �ݒ肷��l
     */
    void setY(const FloatT &y){set(Y_INDEX, y);}
    /**
     * Z�v�f��ݒ肵�܂��B
     * @param z �ݒ肷��l
     */
    void setZ(const FloatT &z){set(Z_INDEX, z);}
    
    /**
     * �v�f���擾���܂��B
     * �v�f�ԍ��̒�`��operator[](const unsigned int &) const�ɂ���Ē�`����Ă��܂��B
     * 
     * @param index �v�f�ԍ�
     * @return const FloatT & �v�f
     * @see operator[](const unsigned int &) const
     */
    const FloatT &get(const unsigned &index) const {return (*this)[index];}
    /**
     * X�v�f���擾���܂��B
     * @return FloatT X�v�f
     */
    const FloatT &getX() const {return get(X_INDEX);}
    /**
     * Y�v�f���擾���܂��B
     * @return FloatT Y�v�f
     */
    const FloatT &getY() const {return get(Y_INDEX);}
    /**
     * Z�v�f���擾���܂��B
     * @return FloatT Z�v�f
     */
    const FloatT &getZ() const {return get(Z_INDEX);}

#ifndef pow2
#define pow2(x) ((x) * (x))
#else
#define POW2_ALREADY_DEFINED
#endif
    /**
     * 3�����x�N�g��@f$ \vec{v} @f$�̃m����@f$ \left| \vec{v} \right| @f$��
     * �������߂܂��B
     * ��`��3�����x�N�g��
     * @f$ \vec{v} \equiv \begin{pmatrix} x \\ y \\ z \end{pmatrix} @f$�ɑ΂�
     * @f[
     *    \left| \vec{v} \right|^{2} \equiv x^{2} + y^{2} + z^{2}
     * @f]
     * �ł��B
     * 
     * @return (T) ����
     */
    FloatT abs2() const{
      FloatT result(0);
      for(unsigned int i(0); i < OUT_OF_INDEX; i++){result += pow2((*this)[i]);}
      return result;
    }
#ifndef POW2_ALREADY_DEFINED
#undef pow2
#else
#undef POW2_ALREADY_DEFINED
#endif
    /**
     * 3�����x�N�g��@f$ \vec{v} @f$�̃m����@f$ \left| \vec{v} \right| @f$��
     * �����߂܂��B
     * ��`��abs2()���Q�Ƃ��Ă��������B
     * 
     * @return (T) ����
     * @see abs2()
     */
    FloatT abs() const{return sqrt(abs2());}
    
    /**
     * �P���}�C�i�X�I�y���[�^�B
     * 
     * @return (Vector3<FloatT>) �S�v�f�Ƀ}�C�i�X����������
     */
    self_t operator-() const{return (copy() *= -1);}
    
    /**
     * �X�J���[�ς����܂��B�j��I�ł��B
     * 
     * @param t �X�J���[
     * @return (Vector3<FloatT>) ����
     */
    self_t &operator*=(const FloatT &t){
      for(unsigned int i(0); i < OUT_OF_INDEX; i++){(*this)[i] *= t;}
      return *this;
    }
    /**
     * �X�J���[�ς����܂��B
     * 
     * @param t �X�J���[
     * @return (Vector3<FloatT>) ����
     * @see oprtator*=(const FloatT &t)
     */
    self_t operator*(const FloatT &t) const{return copy() *= t;}
    
    /**
     * �X�J���[�������܂��B�j��I�ł��B
     * 
     * @param t �X�J���[
     * @return (Vector3<FloatT>) ����
     */
    self_t &operator/=(const FloatT &t){return (*this) *= (FloatT(1) / t);}
    /**
     * �X�J���[�������܂��B
     * 
     * @param t �X�J���[
     * @return (Vector3<FloatT>) ����
     * @see oprtator/=(const FloatT &t)
     */
    self_t operator/(const FloatT &t) const{return copy() /= t;}
    
    /**
     * 3�����x�N�g���Ƃ̉��Z�����܂��B�j��I�ł��B
     * 
     * @param v 3�����x�N�g��
     * @return (Vector3<FloatT>) ����
     */
    self_t &operator+=(const self_t &v){
      for(unsigned int i(0); i < OUT_OF_INDEX; i++){(*this)[i] += v[i];}
      return *this;
    }
    /**
     * 3�����x�N�g���Ƃ̉��Z�����܂��B
     * 
     * @param v 3�����x�N�g��
     * @return (Vector3<FloatT>) ����
     * @see operator+=(const Vector3<FloatT> &)
     */
    self_t operator+(const self_t &v) const{return copy() += v;}
    
    /**
     * 3�����x�N�g���Ƃ̌��Z�����܂��B�j��I�ł��B
     * 
     * @param v 3�����x�N�g��
     * @return (Vector3<FloatT>) ����
     */
    self_t &operator-=(const self_t &v){
     for(unsigned int i(0); i < OUT_OF_INDEX; i++){(*this)[i] -= v[i];}
      return *this;
    }
    /**
     * 3�����x�N�g���Ƃ̌��Z�����܂��B
     * 
     * @param v 3�����x�N�g��
     * @return (Vector3<FloatT>) ����
     * @see operator-=(const Vector3<FloatT> &)
     */
    self_t operator-(const self_t &v) const{return copy() -= v;}
    
    /**
     * �O�ς����܂��B
     * 
     * @param v 3�����x�N�g��
     * @return (Vector3<FloatT>) ����
     */
    self_t operator*(const self_t &v) const{
      self_t result;
      result[0] = (*this)[1] * v[2] - (*this)[2] * v[1];
      result[1] = (*this)[2] * v[0] - (*this)[0] * v[2];
      result[2] = (*this)[0] * v[1] - (*this)[1] * v[0];
      return result;
    }
    
    /**
     * ���ς����܂��B
     * 
     * @param v 3�����x�N�g��
     * @return (Vector3<FloatT>) ����
     */
    FloatT innerp(const self_t &v) const{
     FloatT result(0);
      for(unsigned int i(0); i < OUT_OF_INDEX; i++){
       result += (*this)[i] * v[i];
      }
     return result;
    }
    
    /**
     * �O�ς����܂��B�j��I�ł��B
     * 
     * @param v 3�����x�N�g��
     * @return (Vector3<FloatT>) ����
     * @see operator*(const Vector3<FloatT> &) const
     */
    self_t &operator*=(const self_t &v){
      FloatT temp0((*this)[0]), temp1((*this)[1]);
      (*this)[0] = (*this)[1] * v[2] - (*this)[2] * v[1];
      (*this)[1] = (*this)[2] * v[0] - temp0 * v[2];
      (*this)[2] = temp0 * v[1] - temp1 * v[0];
      return *this;
    }

    /**
     * 3�����x�N�g�������₷���`�ŏo�͂��܂��B
     * 
     * @param out �o�̓X�g���[��
     * @param v �o�͑Ώ�
     * @return (ostream) �o�̓X�g���[�� 
     */
    friend std::ostream &operator<<(std::ostream &out, const self_t &v){
      out << "{";
      for(unsigned int i(0); i < OUT_OF_INDEX; i++){
        out << (i == 0 ? "" : ",") << v[i];
      }
      out << "}";
      return out;  
    }

    /**
     * @f$ 3 \times 1 @f$�s�񂠂邢��@f$ 1 \times 3 @f$�s���Vector3�^�ɕϊ����܂��B
     * 
     * @param matrix �s��
     * @throws MatrixException �s��̃T�C�Y���������Ȃ��Ƃ�
     */
    Vector3(const Matrix<FloatT> &matrix) throw(MatrixException)
        : super_t() {
      if((matrix.rows() == OUT_OF_INDEX) && (matrix.columns() == 1)){
        for(unsigned int i = 0; i < OUT_OF_INDEX; i++){
          (*this)[i] = matrix(i, 0);
        }
      }else if((matrix.rows() == 1) && (matrix.columns() == OUT_OF_INDEX)){
        for(unsigned int i = 0; i < OUT_OF_INDEX; i++){
          (*this)[i] = matrix(0, i);
        }
      }else{
        throw MatrixException("Operatiorn void!! ; Need Matrix(3, 1) or Matrix(1, 3)");
      }
    }
    
    friend self_t operator*(const Matrix<FloatT> &matrix, const self_t &vec){
      if((matrix.rows() != OUT_OF_INDEX) || (matrix.columns() != OUT_OF_INDEX)){
        throw MatrixException("Operatiorn void!! ; Need Matrix(3, 3)");
      }
      self_t res;
      for(unsigned int i(0); i < OUT_OF_INDEX; i++){
        res[i] = FloatT(0);
        for(unsigned int j(0); j < OUT_OF_INDEX; j++){
          res[i] += matrix(i, j) * vec[j];
        }
      }
      return res;
    }
    
    /**
     * @f$ 3 \times 1 @f$�s��ɕϊ����܂��B
     * 
     * @return (Matrix<FloatT>) �s��
     */
    Matrix<FloatT> toMatrix() const{
      Matrix<FloatT> matrix(OUT_OF_INDEX, 1);
      for(unsigned int i(0); i < OUT_OF_INDEX; i++){matrix(i, 0) = (*this)[i];}
      return matrix;
    }
    
    /**
     * �O�ς��s��̐ς̌`�ɕϊ����܂��B���Ȃ킿�A
     * @f$ \vec{v}_{\mathrm{this}} * \vec{v}_{\mathrm{another}} 
     *    = \begin{bmatrix} 
     *        0 & -v_{Z} & v_{Y} \\
     *        v_{Z} & 0 & -v_{X} \\
     *        -v_{Y} & v_{X} & 0
     *      \end{bmatrix}_{v_\mathrm{this}} 
     *      v_{\mathrm{another}} @f$
     * �ƂȂ�܂��B���Ȃ݂�
     * @f$ \vec{v}_{\mathrm{another}} * \vec{v}_{\mathrm{this}} 
     *    = \begin{bmatrix} 
     *        0 & v_{Z} & -v_{Y} \\
     *        -v_{Z} & 0 & v_{X} \\
     *        v_{Y} & -v_{X} & 0
     *      \end{bmatrix}_{v_\mathrm{this}} 
     *      v_{\mathrm{another}} 
     *    = - \left[ \; \right]_{v_\mathrm{this}} v_{\mathrm{another}} @f$
     * �ł��B
     * 
     */
    Matrix<FloatT> skewMatrix() const{
      Matrix<FloatT> matrix(OUT_OF_INDEX, OUT_OF_INDEX);
      {
        matrix(0, 1) = -(*this)[2];
        matrix(0, 2) =  (*this)[1];
        matrix(1, 0) =  (*this)[2];
        matrix(1, 2) = -(*this)[0];
        matrix(2, 0) = -(*this)[1];
        matrix(2, 1) =  (*this)[0];
      }
      return matrix;
    }
};

#define VECTOR3_NO_FLY_WEIGHT(float_t) \
template <> \
class Vector3Data<float_t> : public Vector3DataProperty<float_t> { \
  protected: \
    typedef Vector3DataProperty<float_t> super_t; \
    typedef Vector3Data<float_t> self_t; \
    \
  private: \
    float_t values[super_t::OUT_OF_INDEX]; \
    \
  protected: \
    Vector3Data(){} \
    \
    Vector3Data(const float_t &x, const float_t &y, const float_t &z){ \
      values[super_t::X_INDEX] = x; \
      values[super_t::Y_INDEX] = y; \
      values[super_t::Z_INDEX] = z; \
    } \
    \
    Vector3Data(const self_t &v){ \
      std::memcpy(values, v.values, sizeof(values)); \
    } \
    \
    self_t &operator=(const self_t &v){ \
      std::memcpy(values, v.values, sizeof(values)); \
      return *this; \
    } \
    \
    self_t deep_copy() const { \
      self_t copied; \
      std::memcpy(copied.values, values, sizeof(values)); \
      return copied; \
    } \
  public: \
    ~Vector3Data(){} \
    \
    const float_t &operator[](const unsigned int &index) const { \
      return values[index]; \
    } \
    float_t &operator[](const unsigned int &index){ \
      return const_cast<float_t &>(static_cast<const self_t &>(*this)[index]); \
    } \
}

//VECTOR3_NO_FLY_WEIGHT(double);

#endif /* __VECTOR3_H */
