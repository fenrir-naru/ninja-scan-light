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
 * @brief クォータニオン(Quaternion、4元数)ライブラリ
 * 
 * クォータニオンを定義したライブラリ。
 */

#include <cmath>
#include "param/matrix.h"

#include "param/vector3.h"

template <class FloatT>
struct QuaternionDataProperty{
  
#if defined(_MSC_VER) || defined(__TI_COMPILER_VERSION__)
  static const unsigned int OUT_OF_INDEX = 4; ///< 最大要素数(4)
#else
  static const unsigned int OUT_OF_INDEX; ///< 最大要素数
#endif
};

#if !(defined(_MSC_VER) || defined(__TI_COMPILER_VERSION__))
template <class FloatT>
const unsigned int QuaternionDataProperty<FloatT>::OUT_OF_INDEX
    = 4; ///< 最大要素数
#endif

template <class FloatT>
class QuaternionData : public QuaternionDataProperty<FloatT> {
  protected:
    typedef QuaternionData<FloatT> self_t;
  private:
    struct storage_t{
      FloatT scalar;  ///< スカラー要素
      Vector3<FloatT> vector; ///< ベクトル要素
      int ref;  ///< 参照カウンタ
      storage_t() : ref(1) {}
      storage_t(const FloatT &q0, const Vector3<FloatT> &v)
          : scalar(q0), vector(v), ref(1) {}
      storage_t(
          const FloatT &q0, const FloatT &q1, 
          const FloatT &q2, const FloatT &q3)
          : scalar(q0), vector(q1, q2, q3), ref(1) {}
    } *storage;  ///< ストレージ
  protected:
    /**
     * コンストラクタ
     * 
     */
    QuaternionData() : storage(new storage_t()){}
    
    /**
     * コンストラクタ
     * 
     * @param q0 スカラー要素
     * @param v 3次元ベクトル要素
     */
    QuaternionData(const FloatT &q0, const Vector3<FloatT> &v)
        : storage(new storage_t(q0, v)){}
    
    /**
     * コンストラクタ
     * 
     * @param q0 スカラー要素
     * @param q1 3次元ベクトル要素の(X)
     * @param q2 3次元ベクトル要素の(Y)
     * @param q3 3次元ベクトル要素の(Z)
     */
    QuaternionData(
        const FloatT &q0, const FloatT &q1, 
        const FloatT &q2, const FloatT &q3)
        : storage(new storage_t(q0, q1, q2, q3)) {}
    
    /**
     * コピーコンストラクタ
     * 
     * シャローコピーを行います。
     * 
     * @param q コピー元
     */
    QuaternionData(const self_t &q){
      if(storage = q.storage){(storage->ref)++;}
    }
    
    /**
     * 代入演算子
     * 
     * シャローコピーで対応しています。
     * 
     * @param q コピー元
     */
    self_t &operator=(const self_t &q){
      if(this == &q){return *this;}
      if(storage && ((--(storage->ref)) <= 0)){delete storage;}
      if(storage = q.storage){(storage->ref)++;}
      return *this;
    }
    
    /**
     * ディープコピーを行います。
     * 
     * @return (Quarterion) コピー
     */
    self_t deep_copy() const{
      return self_t(
          storage->scalar,
          storage->vector.copy());
    }
    
  public:
    /**
     * デストラクタ
     * 
     * 参照カウンタを減算します。
     * もし参照カウンタが0の場合、使用していたメモリを解放します。
     */
    ~QuaternionData(){
      if(storage && ((--(storage->ref)) <= 0)){
        delete storage;
      }
    }
    
    /**
     * スカラー要素を返します。
     * 
     * @return (T) スカラー要素
     */
    const FloatT &scalar() const {return storage->scalar;}
    FloatT &scalar(){
      return const_cast<FloatT &>(static_cast<const self_t &>(*this).scalar());
    }
    /**
     * ベクトル要素を返します。
     * 
     * @return (Vector<FloatT>) ベクトル要素
     */
    const Vector3<FloatT> &vector() const {return storage->vector;}
    Vector3<FloatT> &vector(){
      return const_cast<Vector3<FloatT> &>(static_cast<const self_t &>(*this).vector());
    }
    
    /**
     * 要素(の参照)を返します。
     * 従って代入も可能です。
     * 
     * @param index 要素番号、0:スカラー要素,1～3:3次元ベクトル要素X～Z
     * @return (FloatT &) 要素への参照
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
 * @brief クォータニオン
 * 
 * クォータニオン@f$ \Tilde{q} \f$はスカラー要素@f$ q_{0} \f$と
 * 3次元ベクトル要素@f$ \vec{q} \f$からなり
 * @f[
 *    \Tilde{q} \equiv \begin{Bmatrix} q_{0} \\ \vec{q} \end{Bmatrix}
 * @f]
 * と表現されます。
 * 
 * 特に全要素の二乗和が1となる単位クォータニオンを用いると、
 * 3次元上での姿勢角を表す際に * オイラー角で生じる
 * @f$ \tan{\frac{\pi}{2}} @f$といった特異点を回避することができ、
 * 滑らかな演算を行うことが可能です。
 * 
 * なお、内部的に参照カウンタを利用したライトウエイトな実装になっているため、
 * メモリや演算回数が節約されることが機体されます。
 * そのために明示的にcopy()メソッドを使用しないとディープコピーがされないので、
 * 再利用を行う際は注意してください。
 * 
 * @param FloatT 演算精度、doubleなど
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
     * コンストラクタ
     * 全要素を0で初期化します。
     */
    Quaternion()
        : super_t(FloatT(0), FloatT(0), FloatT(0), FloatT(0)) {}
    
    /**
     * コンストラクタ
     * 
     * @param q0 スカラー要素
     * @param v 3次元ベクトル要素
     */
    Quaternion(const FloatT &q0, const Vector3<FloatT> &v)
        : super_t(q0, v){}
    
    /**
     * コンストラクタ
     * 
     * @param q0 スカラー要素
     * @param q1 3次元ベクトル要素の(X)
     * @param q2 3次元ベクトル要素の(Y)
     * @param q3 3次元ベクトル要素の(Z)
     */
    Quaternion(
        const FloatT &q0, const FloatT &q1,
        const FloatT &q2, const FloatT &q3)
        : super_t(q0, q1, q2, q3) {}
    
    /**
     * デストラクタ
     * 
     */
    ~Quaternion(){}
    
    /**
     * コピーコンストラクタ
     * 
     * @param q コピー元
     */
    Quaternion(const self_t &q) : super_t(q) {
      
    }
    
    /**
     * 代入演算子
     * 
     * @param q コピー元
     */
    self_t &operator=(const self_t &q){
      super_t::operator=(q);
      return *this;
    }
    
    /**
     * ディープコピーを行います。
     * 
     * @return (Quarterion) コピー
     */
    self_t copy() const{
      return self_t(super_t::deep_copy());
    }
    
    /**
     * 要素を設定します。
     * 要素番号の定義はoperator[](unsigned int)によって定義されています。
     * 
     * @param index 要素番号
     * @param value 設定する値
     * @see operator[](unsigned int)
     */
    void set(const unsigned int &index, const FloatT &value){(*this)[index] = value;}
    /**
     * 要素を取得します。
     * 要素番号の定義はoperator[](unsigned int) constによって定義されています。
     * 
     * @param index 要素番号
     * @return FloatT 要素
     * @see operator[](unsigned int) const
     */
    const FloatT &get(const unsigned int &index) const{return (*this)[index];}
    
    /**
     * 共役クォータニオンを求めます。
     * 共役クォータニオン@f$ \Tilde{q}^{*} \f$は
     * @f[
     *     \Tilde{q}^{*} \equiv \begin{Bmatrix} q_{0} \\ - \vec{q} \end{Bmatrix}
     * @f]
     * で定義されます。
     * なお、求める際にディープコピーがされているので、以後の返却値に対する操作について、
     * 元のクォータニオンは非破壊となります。
     * 
     * @return (Quarterion<FloatT>) 共役クォータニオン
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
     * 要素の二乗和@f$ \left| \Tilde{q} \right|^{2} @f$を求めます。
     * @f[
     *     \left| \Tilde{q} \right|^{2} \equiv q_{0} {}^{2} + \left| \vec{q} \right|^{2}
     * @f]
     * で定義されます。
     * 
     * @return (FloatT) 結果
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
     * 要素の二乗和の平方根(ノルム)を求めます。
     * 
     * @return (FloatT) 結果
     * @see abs2()
     */
    FloatT abs() const{return std::sqrt(abs2());}

    /**
     * スカラーとの積算を行います。破壊的です。
     * 
     * @return (Quaternion<FloatT>) 結果
     */
    self_t &operator*=(const FloatT &t){
      scalar() *= t;
      vector() *= t;
      return *this;
    }
    
    /**
     * スカラーとの積算を行います。
     * 
     * @return (Quaternion<FloatT>) 結果
     */
    self_t operator*(const FloatT &t) const{return copy() *= t;}
    
    /**
     * スカラーとの除算を行います。破壊的です。
     * 
     * @return (Quaternion<FloatT>) 結果
     */
    self_t &operator/=(const FloatT &t){return (*this) *= (FloatT(1) / t);}
    
    /**
     * スカラーとの除算を行います。
     * 
     * @return (Quaternion<FloatT>) 結果
     */
    self_t operator/(const FloatT &t) const{return copy() /= t;}
    
    /**
     * ノルムが1になるように正規化します。
     * 
     * @return (Quaternion<FloatT>) 結果
     * @see abs()
     */
    self_t regularize() const{return (*this) / abs();}
    
    /**
     * クォータニオンとの積算を行います。
     * 積算 @f$ \Tilde{q}_{a} \Tilde{q}_{b} @f$は
     * @f[
     *    \Tilde{q}_{a} \Tilde{q}_{b}
     *      \equiv \begin{Bmatrix} q_{0a} \\ \vec{q}_{a} \end{Bmatrix}
     *        \begin{Bmatrix} q_{0b} \\ \vec{q}_{b} \end{Bmatrix}
     *      \equiv \begin{Bmatrix}
     *          q_{0a} q_{0b} - \vec{q}_{a} \cdot \vec{q}_{b} \\
     *          q_{0a} \vec{q}_{b} + q_{0b} \vec{q}_{a} + \vec{q}_{a} \times \vec{q}_{b} 
     *        \end{Bmatrix}
     * @f] 
     * で定義されます。
     * 
     * @return (Quaternion<FloatT>) 結果
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
     * クォータニオンとの積算を行います。破壊的です。
     * 定義はoperator*(const Quaternion &) constを参照してください。
     * 
     * @return (Quaternion<FloatT>) 結果
     * @see operator*(const Quaternion<FloatT> &) const
     */
    self_t &operator*=(const self_t &q){return (*this) = (*this) * q;}
    
    /**
     * クォータニオンとの加算を行います。破壊的です。
     * 加算 @f$ \Tilde{q}_{a} + \Tilde{q}_{b} @f$は
     * @f[
     *    \Tilde{q}_{a} + \Tilde{q}_{b}
     *      \equiv \begin{Bmatrix} q_{0a} \\ \vec{q}_{a} \end{Bmatrix}
     *        + \begin{Bmatrix} q_{0b} \\ \vec{q}_{b} \end{Bmatrix}
     *      \equiv \begin{Bmatrix}
     *          q_{0a} + q_{0b} \\
     *          \vec{q}_{a} + \vec{q}_{b} 
     *        \end{Bmatrix}
     * @f] 
     * で定義されます。
     * 
     * @return (Quaternion<FloatT>) 結果
     */
    self_t &operator+=(const self_t &q){
      for(unsigned int i(0); i < OUT_OF_INDEX; i++){(*this)[i] += q[i];}
      return *this;
    }
    /**
     * クォータニオンとの加算を行います。
     * 定義はoperator+=(const Quaternion<FloatT> &) constを参照してください。
     * 
     * @return (Quaternion<FloatT>) 結果
     * @see operator+=(const Quaternion<FloatT> &)
     */
    self_t operator+(const self_t &q) const{return copy() += q;}
    
    /**
     * クォータニオンとの減算を行います。破壊的です。
     * 加算 @f$ \Tilde{q}_{a} - \Tilde{q}_{b} @f$は
     * @f[
     *    \Tilde{q}_{a} - \Tilde{q}_{b}
     *      \equiv \begin{Bmatrix} q_{0a} \\ \vec{q}_{a} \end{Bmatrix}
     *        - \begin{Bmatrix} q_{0b} \\ \vec{q}_{b} \end{Bmatrix}
     *      \equiv \begin{Bmatrix}
     *          q_{0a} - q_{0b} \\
     *          \vec{q}_{a} - \vec{q}_{b} 
     *        \end{Bmatrix}
     * @f] 
     * で定義されます。
     * 
     * @return (Quaternion<FloatT>) 結果
     */
    self_t &operator-=(const self_t &q){
      for(unsigned int i(0); i < OUT_OF_INDEX; i++){(*this)[i] -= q[i];}
      return *this;
    }
    /**
     * クォータニオンとの減算を行います。
     * 定義はoperator-=(const Quaternion<FloatT> &)を参照してください。
     * 
     * @return (Quaternion<FloatT>) 結果
     * @see operator-=(const Quaternion<FloatT> &)
     */
    self_t operator-(const self_t &q) const{return copy() -= q;}
    
    /**
     * 3次元ベクトルとの積算を行います。破壊的です。
     * 積算 @f$ \Tilde{q} \vec{v} @f$は
     * @f[
     *    \Tilde{q} \vec{v}
     *      \equiv \begin{Bmatrix} q_{0} \\ \vec{q} \end{Bmatrix}
     *        \begin{Bmatrix} 0 \\ \vec{v} \end{Bmatrix}
     *      \equiv \begin{Bmatrix}
     *          - \vec{q} \cdot \vec{v} \\
     *          q_{0} \vec{v} + \vec{q} \times \vec{v} 
     *        \end{Bmatrix}
     * @f] 
     * で定義されます。
     * 
     * @return (Quaternion<FloatT>) 結果
     */
    self_t &operator*=(const Vector3<FloatT> &v){
      FloatT temp_scalar(scalar());
      scalar() = -(vector().innerp(v));
      (vector() *= v) += v * temp_scalar;
      return (*this);
    }
    /**
     * 3次元ベクトルとの積算を行います。
     * 定義はoperator*=(const Vector3<FloatT> &)を参照してください。
     * 
     * @return (Quaternion<FloatT>) 結果
     * @see operator*=(const Vector3<FloatT> &)
     */
    self_t operator*(const Vector3<FloatT> &v) const{return copy() *= v;}
    
    /**
     * 回転角の半分を求めます。
     * 
     * @return (T) 結果
     */
    FloatT getTheta_2() const{return std::acos(regularize()[0]);}
    /**
     * 回転角を求めます。
     * 
     * @return (T) 結果
     * @see getTheta_2()
     */
    FloatT getTheta() const{return getTheta_2() * 2;}
    /**
     * 回転軸の座標を求めます。
     * 
     * @return (Vector3<FloatT>) 結果
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
     * @f$ 3 \times 3 @f$ のDirection Cosine Matrix(DCM)に変換します。
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
     * Quaternionを見やすい形で出力します。
     * 
     * @param out 出力ストリーム
     * @param q 出力対象
     * @return (ostream) 出力ストリーム
     */
    friend std::ostream &operator<<(std::ostream &out, const self_t &q){
      for(unsigned int i(0); i < OUT_OF_INDEX; i++){
        out << (i == 0 ? "{" : ",") << q[i];
      }
      out << "}";
      return out;
    }
    
    /**
     * @f$ 4 \times 1 @f$行列あるいは@f$ 1 \times 4 @f$行列、
     * あるいはDCMの条件を満たす@f$ 3 \times 3 @f$行列を
     * Quaternion型に変換します。
     * 
     * @param matrix 行列
     * @throws MatrixException 行列のサイズが正しくないとき
     */
    Quaternion(const Matrix<FloatT> &matrix) throw(MatrixException)
        : super_t() {
      
      Matrix<FloatT> &m(const_cast<Matrix<FloatT> &>(matrix));
      
      if(matrix.rows() == OUT_OF_INDEX && matrix.columns() == 1){
        for(int i(0); i < OUT_OF_INDEX; i++){(*this)[i]= m(i, 0);}
      }else if(matrix.rows() == 1 && matrix.columns() == OUT_OF_INDEX){
        for(int i(0); i < OUT_OF_INDEX; i++){(*this)[i]= m(0, i);}
      }else if((matrix.rows() == 3) && (matrix.columns() == 3)){
        
        // TODO: DCMの条件を満たしているか、調べること
        
        // 対角要素を足すと 3 q_0^2 - (q_1^2 + q_2^2 + q_3^2) == 4 q_0^2 - 1 であり
        // q_0 >= 0であるからq_0が一番はじめにとける        
        (*this)[0] = std::sqrt(((m(0, 0) + m(1, 1) + m(2, 2)) + 1) / 4);
        
        if((*this)[0] > 1E-10){
          // 非対角要素から残りを求める
          (*this)[1] = (m(1, 2) - m(2, 1)) / 4 / (*this)[0];
          (*this)[2] = (m(2, 0) - m(0, 2)) / 4 / (*this)[0];
          (*this)[3] = (m(0, 1) - m(1, 0)) / 4 / (*this)[0];
        }else{  // 0に近いとあんまりよくない
          // 対角要素(0)+(1)-(2) => (q_0^2 + q_1^2 + q_2^2) - 3 q_3^2 == 1 - 4 q_3^2 であり
          // このときq_3を正とする
          (*this)[3] = std::sqrt(((m(0, 0) + m(1, 1) - m(2, 2)) - 1) / -4);
          if((*this)[3] > 1E-10){
            (*this)[1] = (m(2, 0) + m(0, 2)) / 4 / (*this)[3];
            (*this)[2] = (m(2, 1) + m(1, 2)) / 4 / (*this)[3];
          }else{ // あまり小さいと問題
            // 対角要素から求める
            // 対角要素-(0)+(1)+(2) => (q_0^2 + q_2^2 + q_3^2) - 3 q_1^2 == 1 - 4 q_1^2
            // 対角要素(0)-(1)+(2) => (q_0^2 + q_1^2 + q_3^2) - 3 q_2^2 == 1 - 4 q_2^2
            // このときq_2を正とし、q_1の符号は非対角要素から求める
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
     * @f$ 4 \times 1 @f$行列に変換します。
     * 
     * @return (Matrix<FloatT>) 行列
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
