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
 * @brief 3次元ベクトルライブラリ
 * 
 * 3次元ベクトルを定義したライブラリ。
 */

#include <cmath>
#include <cstring>
#include "param/matrix.h"

template <class FloatT>
struct Vector3DataProperty{
  /**
   * インデックスの定義
   * 
   */
  enum Index {
      X_INDEX = 0,  ///< X要素はインデックス0
      Y_INDEX,      ///< Y要素はインデックス1
      Z_INDEX};     ///< Z要素はインデックス2
#if defined(_MSC_VER) || defined(__TI_COMPILER_VERSION__)
  static const unsigned int OUT_OF_INDEX = 3; ///< 最大要素数(3)
#else
  static const unsigned int OUT_OF_INDEX; ///< 最大要素数(3)
#endif
};

#if !(defined(_MSC_VER) || defined(__TI_COMPILER_VERSION__))
template <class FloatT>
const unsigned int Vector3DataProperty<FloatT>::OUT_OF_INDEX 
    = Vector3DataProperty<FloatT>::Z_INDEX + 1; ///< 最大要素数(3)
#endif

template <class FloatT>
class Vector3Data : public Vector3DataProperty<FloatT> {
  protected:
    typedef Vector3DataProperty<FloatT> super_t;
    typedef Vector3Data<FloatT> self_t;
    
  private:
    struct storage_t{
      FloatT values[super_t::OUT_OF_INDEX];  ///<要素保存用
      int ref;                      ///<参照カウンタ
      storage_t() : ref(1) {}
      storage_t(const FloatT &x, const FloatT &y, const FloatT &z)
          : ref(1) {
        values[super_t::X_INDEX] = x;
        values[super_t::Y_INDEX] = y;
        values[super_t::Z_INDEX] = z;
      }
    } *storage;  ///< ストレージ
    
    Vector3Data(storage_t *_storage) : storage(_storage){
      
    }
    
  protected:
    /**
     * コンストラクタ。
     */
    Vector3Data() : storage(new storage_t()){
      
    }
    
    /**
     * コンストラクタ。
     * 指定した値で初期化されます。
     * 
     * @param x X要素の値
     * @param y Y要素の値
     * @param z Z要素の値
     */
    Vector3Data(const FloatT &x, const FloatT &y, const FloatT &z)
        : storage(new storage_t(x, y, z)){
      
    }
    
    /**
     * コピーコンストラクタ
     * 
     * シャローコピーを行います。
     * 
     * @param v コピー元
     */
    Vector3Data(const self_t &v){
      if(storage = v.storage){(storage->ref)++;}
    }
    
    /**
     * 代入演算子
     * 
     * シャローコピーで対応しています。
     * 
     * @param v コピー元
     */
    self_t &operator=(const self_t &v){
      if(this == &v){return *this;}
      if(storage && ((--(storage->ref)) <= 0)){delete storage;}
      if(storage = v.storage){(storage->ref)++;}
      return *this;
    }
    
    /**
     * ディープコピーを行います。
     * 
     * @return コピー
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
     * デストラクタ。
     * 
     * 参照カウンタを減算します。
     * もし参照カウンタが0の場合、使用していたメモリを解放します。
     */
    ~Vector3Data(){
      if(storage && ((--(storage->ref)) <= 0)){
        delete storage;
      }
    }
    
    /**
     * 要素(の参照)を返します。
     * 従って代入も可能です。
     * 
     * @param index 要素番号、0～2:要素X～Z
     * @return (FloatT &) 要素への参照
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
 * @brief 3次元ベクトル
 * 
 * 3次元ベクトルクラス。
 * ベクトル自身の定義や内積や外積などを含めた様々な演算の定義を行っています。
 * 
 * なお、内部的に参照カウンタを利用したライトウエイトな実装になっているため、
 * メモリや演算回数が節約されることが機体されます。
 * そのために明示的にcopy()メソッドを使用しないとディープコピーがされないので、
 * 再利用を行う際は注意してください。
 * 
 * @param FloatT 演算精度、doubleなど
 */
template <class FloatT>
class Vector3 : public Vector3Data<FloatT> {
  protected:
    typedef Vector3<FloatT> self_t;
    typedef Vector3Data<FloatT> super_t;
  
    Vector3(const super_t &v) : super_t(v){}
    
  public:
    /**
     * コンストラクタ。
     * 全要素は0で初期化されます。
     */
    Vector3() 
        : super_t(FloatT(0), FloatT(0), FloatT(0)){}
    
    /**
     * コンストラクタ。
     * 指定した値で初期化されます。
     * 
     * @param x X要素の値
     * @param y Y要素の値
     * @param z Z要素の値
     */
    Vector3(const FloatT &x, const FloatT &y, const FloatT &z)
        : super_t(x, y, z){}
    
    /**
     * デストラクタ。
     * 
     */
    ~Vector3(){}
    
    /**
     * コピーコンストラクタ
     * 
     * @param v コピー元
     */
    Vector3(const self_t &v) : super_t(v) {
      
    }
    
    /**
     * 代入演算子
     * 
     * @param v コピー元
     */
    self_t &operator=(const self_t &v){
      super_t::operator=(v);
      return *this;
    }
    
    /**
     * ディープコピーを行います。
     * 
     * @return (Vector3) コピー
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
     * 要素を設定します。
     * 要素番号の定義はoperator[](const unsigned int &)によって定義されています。
     * 
     * @param index 要素番号
     * @param value 設定する値
     * @see operator[](const unsigned int &)
     */
    void set(const unsigned &index, const FloatT &value){(*this)[index] = value;}
    /**
     * X要素を設定します。
     * @param x 設定する値
     */
    void setX(const FloatT &x){set(X_INDEX, x);}
    /**
     * Y要素を設定します。
     * @param y 設定する値
     */
    void setY(const FloatT &y){set(Y_INDEX, y);}
    /**
     * Z要素を設定します。
     * @param z 設定する値
     */
    void setZ(const FloatT &z){set(Z_INDEX, z);}
    
    /**
     * 要素を取得します。
     * 要素番号の定義はoperator[](const unsigned int &) constによって定義されています。
     * 
     * @param index 要素番号
     * @return const FloatT & 要素
     * @see operator[](const unsigned int &) const
     */
    const FloatT &get(const unsigned &index) const {return (*this)[index];}
    /**
     * X要素を取得します。
     * @return FloatT X要素
     */
    const FloatT &getX() const {return get(X_INDEX);}
    /**
     * Y要素を取得します。
     * @return FloatT Y要素
     */
    const FloatT &getY() const {return get(Y_INDEX);}
    /**
     * Z要素を取得します。
     * @return FloatT Z要素
     */
    const FloatT &getZ() const {return get(Z_INDEX);}

#ifndef pow2
#define pow2(x) ((x) * (x))
#else
#define POW2_ALREADY_DEFINED
#endif
    /**
     * 3次元ベクトル@f$ \vec{v} @f$のノルム@f$ \left| \vec{v} \right| @f$の
     * 二乗を求めます。
     * 定義は3次元ベクトル
     * @f$ \vec{v} \equiv \begin{pmatrix} x \\ y \\ z \end{pmatrix} @f$に対し
     * @f[
     *    \left| \vec{v} \right|^{2} \equiv x^{2} + y^{2} + z^{2}
     * @f]
     * です。
     * 
     * @return (T) 結果
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
     * 3次元ベクトル@f$ \vec{v} @f$のノルム@f$ \left| \vec{v} \right| @f$の
     * を求めます。
     * 定義はabs2()を参照してください。
     * 
     * @return (T) 結果
     * @see abs2()
     */
    FloatT abs() const{return sqrt(abs2());}
    
    /**
     * 単項マイナスオペレータ。
     * 
     * @return (Vector3<FloatT>) 全要素にマイナスをつけたもの
     */
    self_t operator-() const{return (copy() *= -1);}
    
    /**
     * スカラー積をします。破壊的です。
     * 
     * @param t スカラー
     * @return (Vector3<FloatT>) 結果
     */
    self_t &operator*=(const FloatT &t){
      for(unsigned int i(0); i < OUT_OF_INDEX; i++){(*this)[i] *= t;}
      return *this;
    }
    /**
     * スカラー積をします。
     * 
     * @param t スカラー
     * @return (Vector3<FloatT>) 結果
     * @see oprtator*=(const FloatT &t)
     */
    self_t operator*(const FloatT &t) const{return copy() *= t;}
    
    /**
     * スカラー割をします。破壊的です。
     * 
     * @param t スカラー
     * @return (Vector3<FloatT>) 結果
     */
    self_t &operator/=(const FloatT &t){return (*this) *= (FloatT(1) / t);}
    /**
     * スカラー割をします。
     * 
     * @param t スカラー
     * @return (Vector3<FloatT>) 結果
     * @see oprtator/=(const FloatT &t)
     */
    self_t operator/(const FloatT &t) const{return copy() /= t;}
    
    /**
     * 3次元ベクトルとの加算をします。破壊的です。
     * 
     * @param v 3次元ベクトル
     * @return (Vector3<FloatT>) 結果
     */
    self_t &operator+=(const self_t &v){
      for(unsigned int i(0); i < OUT_OF_INDEX; i++){(*this)[i] += v[i];}
      return *this;
    }
    /**
     * 3次元ベクトルとの加算をします。
     * 
     * @param v 3次元ベクトル
     * @return (Vector3<FloatT>) 結果
     * @see operator+=(const Vector3<FloatT> &)
     */
    self_t operator+(const self_t &v) const{return copy() += v;}
    
    /**
     * 3次元ベクトルとの減算をします。破壊的です。
     * 
     * @param v 3次元ベクトル
     * @return (Vector3<FloatT>) 結果
     */
    self_t &operator-=(const self_t &v){
     for(unsigned int i(0); i < OUT_OF_INDEX; i++){(*this)[i] -= v[i];}
      return *this;
    }
    /**
     * 3次元ベクトルとの減算をします。
     * 
     * @param v 3次元ベクトル
     * @return (Vector3<FloatT>) 結果
     * @see operator-=(const Vector3<FloatT> &)
     */
    self_t operator-(const self_t &v) const{return copy() -= v;}
    
    /**
     * 外積をします。
     * 
     * @param v 3次元ベクトル
     * @return (Vector3<FloatT>) 結果
     */
    self_t operator*(const self_t &v) const{
      self_t result;
      result[0] = (*this)[1] * v[2] - (*this)[2] * v[1];
      result[1] = (*this)[2] * v[0] - (*this)[0] * v[2];
      result[2] = (*this)[0] * v[1] - (*this)[1] * v[0];
      return result;
    }
    
    /**
     * 内積をします。
     * 
     * @param v 3次元ベクトル
     * @return (Vector3<FloatT>) 結果
     */
    FloatT innerp(const self_t &v) const{
     FloatT result(0);
      for(unsigned int i(0); i < OUT_OF_INDEX; i++){
       result += (*this)[i] * v[i];
      }
     return result;
    }
    
    /**
     * 外積をします。破壊的です。
     * 
     * @param v 3次元ベクトル
     * @return (Vector3<FloatT>) 結果
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
     * 3次元ベクトルを見やすい形で出力します。
     * 
     * @param out 出力ストリーム
     * @param v 出力対象
     * @return (ostream) 出力ストリーム 
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
     * @f$ 3 \times 1 @f$行列あるいは@f$ 1 \times 3 @f$行列をVector3型に変換します。
     * 
     * @param matrix 行列
     * @throws MatrixException 行列のサイズが正しくないとき
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
     * @f$ 3 \times 1 @f$行列に変換します。
     * 
     * @return (Matrix<FloatT>) 行列
     */
    Matrix<FloatT> toMatrix() const{
      Matrix<FloatT> matrix(OUT_OF_INDEX, 1);
      for(unsigned int i(0); i < OUT_OF_INDEX; i++){matrix(i, 0) = (*this)[i];}
      return matrix;
    }
    
    /**
     * 外積を行列の積の形に変換します。すなわち、
     * @f$ \vec{v}_{\mathrm{this}} * \vec{v}_{\mathrm{another}} 
     *    = \begin{bmatrix} 
     *        0 & -v_{Z} & v_{Y} \\
     *        v_{Z} & 0 & -v_{X} \\
     *        -v_{Y} & v_{X} & 0
     *      \end{bmatrix}_{v_\mathrm{this}} 
     *      v_{\mathrm{another}} @f$
     * となります。ちなみに
     * @f$ \vec{v}_{\mathrm{another}} * \vec{v}_{\mathrm{this}} 
     *    = \begin{bmatrix} 
     *        0 & v_{Z} & -v_{Y} \\
     *        -v_{Z} & 0 & v_{X} \\
     *        v_{Y} & -v_{X} & 0
     *      \end{bmatrix}_{v_\mathrm{this}} 
     *      v_{\mathrm{another}} 
     *    = - \left[ \; \right]_{v_\mathrm{this}} v_{\mathrm{another}} @f$
     * です。
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
