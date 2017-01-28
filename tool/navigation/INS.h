/*
 *  INS.h, header file to perform calculation of inertial navigation system.
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

#ifndef __INS_H__
#define __INS_H__

/** @file
 * @brief 慣性航法装置(INS)
 * 
 * 慣性航法装置(INS)について記述したファイル。
 */

#include <cmath>

#ifndef M_PI
  #define M_PI 3.1415926535897932384626433832795
#endif

#include "param/vector3.h"
#include "param/quaternion.h"
#include "WGS84.h"

#ifdef pow2
#define POW2_ALREADY_DEFINED
#else
#define pow2(x) ((x)*(x))
#endif

template<class T>
struct INS_Extender {
  typedef T type;
  template<template<class> class Extended>
  struct extend {
    typedef INS_Extender<Extended<T> > eval;
  };
};

template <class T>
struct INS_Property {
  static const unsigned STATE_VALUES = 12; ///< 状態量の数
};

/**
 * @brief ストラップダウン慣性航法装置(INS)
 * 
 * ストラップダウン慣性航法装置(INS)の動作を記述したクラス。
 * 加速度、角速度を入力として、慣性航法方程式を解き、
 * 位置、速度、姿勢を求めます。
 * 
 * @param FloatT 演算精度、通常はdouble
 */
template <class FloatT = double>
class INS {
  public:
    typedef FloatT float_t;
    typedef Vector3<float_t> vec3_t;
    typedef Quaternion<float_t> quat_t;
  protected:
    vec3_t v_2e_4n;       ///< n-frameにおける速度 @f$ \vec{v}_{e}^{n} @f$
    float_t v_N,          ///< 北方向の速度 @f$ V_{N} @f$
           v_E;           ///< 東方向の速度 @f$ V_{E} @f$
    
    quat_t q_e2n;         ///< @f$ \Tilde{q}_{e}^{n} @f$、すなわち現在の経度、緯度、Azimuth角
    float_t h;            ///< 現在の高度[m]
    float_t phi,          ///< 緯度 @f$ \phi @f$
           lambda,        ///< 経度 @f$ \lambda @f$
           alpha;         ///< Azimuth角 @f$ \alpha @f$
    
    quat_t q_n2b;         ///< n-frame -> b-frmae、すなわち現在の姿勢
    
    vec3_t omega_e2i_4e;  ///< Earth Rate @f$ \Omega_{e/i}^{e} @f$
  
    vec3_t omega_e2i_4n;  ///< @f$ \vec{\omega}_{e/i}^{n} @f$
    vec3_t omega_n2e_4n;  ///< @f$ \vec{\omega}_{n/e}^{n} @f$
    
  public:
    typedef WGS84Generic<float_t> Earth; ///< 地球モデル
    static const unsigned STATE_VALUES = INS_Property<INS>::STATE_VALUES;
    virtual unsigned state_values() const {return STATE_VALUES;}
    
    /**
     * 回転軸から現在位置までの距離を返します。
     * 数式で表すと
     * @f[
     *    \left( R_{\mathrm{normal}} \left( \phi \right) + h \right) \cos{\phi} 
     * @f]
     * 
     * @return (float_t) 回転軸から現在位置までの距離
     */
    float_t beta() const{return (Earth::R_normal(phi) + h) * std::cos(phi);}
  
  private:
    /**
     * 緯度 @f$ \phi @f$を @f$ \Tilde{q}_{e}^{n} @f$から更新します。
     * 内部的。
     * 
     * @return (float_t) 更新された緯度
     */
    inline float_t update_phi(){
      return phi = std::asin(float_t(1) - (pow2(q_e2n.get(0)) + pow2(q_e2n.get(3))) * 2);
    }
    /**
     * 経度 @f$ \lambda @f$を @f$ \Tilde{q}_{e}^{n} @f$から更新します。
     * 内部的。
     * 
     * @return (float_t) 更新された経度
     */
    inline float_t update_lambda(){
      return lambda = std::atan2(
          (q_e2n.get(0) * q_e2n.get(1) - q_e2n.get(2) * q_e2n.get(3)),
          (-q_e2n.get(0) * q_e2n.get(2) - q_e2n.get(1) * q_e2n.get(3)));
    }
    /**
     * アジムス角 @f$ \alpha @f$を @f$ \Tilde{q}_{e}^{n} @f$から更新します。
     * 内部的。
     * 
     * @return (float_t) 更新されたアジムス角
     */
    inline float_t update_alpha(){
      return alpha = std::atan2(
          (-q_e2n.get(0) * q_e2n.get(1) - q_e2n.get(2) * q_e2n.get(3)),
          (q_e2n.get(1) * q_e2n.get(3) - q_e2n.get(0) * q_e2n.get(2)));
    }
    /**
     * 北方向速度 @f$ V_{N} @f$を @f$ \vec{v}_{e}^{n} @f$から更新します。
     * 内部的。
     * 
     * @param calpha cos(alpha)
     * @param salpha sin(alpha)
     * @return (float_t) 更新された北方向速度
     */
    inline float_t update_v_N(const float_t &calpha, const float_t &salpha){
      return v_N = v_2e_4n.get(0) * calpha - v_2e_4n.get(1) * salpha;
    }
    /**
     * 東方向速度 @f$ V_{E} @f$を @f$ \vec{v}_{e}^{n} @f$から更新します。
     * 内部的。
     * 
     * @param calpha cos(alpha)
     * @param salpha sin(alpha)
     * @return (float_t) 更新された東方向速度
     */
    inline float_t update_v_E(const float_t &calpha, const float_t &salpha){
      return v_E = v_2e_4n.get(0) * salpha + v_2e_4n.get(1) * calpha;
    }
    
    /**
     * 現在位置上での@f$ \vec{\omega}_{e/i}^{n} @f$を求めます。
     * 内部的。
     */
    inline void update_omega_e2i_4n(){
      omega_e2i_4n = (q_e2n.conj() * omega_e2i_4e * q_e2n).vector();
    }
    /**
     * 現在位置上での@f$ \vec{\omega}_{n/e}^{n} @f$を求めます。
     * 内部的。
     *
     * @param calpha cos(alpha)
     * @param salpha sin(alpha)
     */
    inline void update_omega_n2e_4n(const float_t &calpha, const float_t &salpha){
      float_t omega_n2e_4g_0 = v_E / (Earth::R_normal(phi) + h);
      float_t omega_n2e_4g_1 = -v_N / (Earth::R_meridian(phi) + h);
      
      omega_n2e_4n[0] =  omega_n2e_4g_0 * calpha + omega_n2e_4g_1 * salpha;
      omega_n2e_4n[1] = -omega_n2e_4g_0 * salpha + omega_n2e_4g_1 * calpha;
    }
    /**
     * 現在位置上での@f$ \vec{\omega}_{n/e}^{n} @f$を求めます。
     * 内部的。
     */
    inline void update_omega_n2e_4n(){
      update_omega_n2e_4n(std::cos(alpha), std::sin(alpha));
    }
  protected:
    /**
     * 付帯情報を再計算して最新の状態に保ちます。
     * 
     * @param regularize 内部のクォータニオンを正規化(ノルムを1にする)かどうか
     */
    inline void recalc(const bool regularize = true){
      //正規化
      if(regularize){
        q_e2n = q_e2n.regularize();
        q_n2b = q_n2b.regularize();
      }
      
      //緯度、経度、Azimuth角の更新
      update_phi();
      update_lambda();
      update_alpha();
      
      float_t ca(std::cos(alpha)), sa(std::sin(alpha));

      //v_north, v_eastの更新
      update_v_N(ca, sa);
      update_v_E(ca, sa);
      
      //ω_{e/i}^{n}
      update_omega_e2i_4n();
      
      //ω_{n/e}^{n}
      update_omega_n2e_4n(ca, sa);
    }
    
  public:
    /**
     * 位置を初期化します。
     * 
     * @param latitude 緯度 @f$ \phi @f$
     * @param longitude 経度 @f$ \lambda @f$
     * @param height 高度 @f$ h @f$
     */
    void initPosition(const float_t &latitude, const float_t &longitude, const float_t &height){
      using std::cos;
      using std::sin;
      phi = latitude;
      lambda = longitude;
      alpha = 0;

      float_t cl(cos(lambda / 2)), sl(sin(lambda / 2));
      float_t cp(cos(-phi / 2)), sp(sin(-phi / 2));
      float_t sqrt2(std::sqrt(2.));

      q_e2n[0] = cl * (cp + sp) / sqrt2;
      q_e2n[1] = sl * (cp - sp) / sqrt2;
      q_e2n[2] = -cl * (cp - sp) / sqrt2;
      q_e2n[3] = sl * (cp + sp) / sqrt2;
      h = height;
      
      update_omega_e2i_4n();
      update_omega_n2e_4n();
    }
    
    /**
     * 速度を初期化します。
     * 
     * @param v_north 北方向速度 @f$ V_{N} @f$
     * @param v_east 東方向速度 @f$ V_{E} @f$
     * @param v_down 下方向速度 @f$ V_{D} @f$
     */
    void initVelocity(const float_t &v_north, const float_t &v_east, const float_t &v_down){
      v_2e_4n[0] = v_N = v_north;
      v_2e_4n[1] = v_E = v_east;
      v_2e_4n[2] = v_down;
      
      update_omega_n2e_4n();
    }
    
  protected:
    /**
     * オイラー角からquarternionに変換します。
     * 
     * @param yaw ヨー@f$ \Psi @f$
     * @param pitch ピッチ@f$ \Theta @f$
     * @param roll ロール@f$ \Phi @f$
     * @param res 結果を格納する先
     * @return (quat_t) 変換結果
     */
    static quat_t &euler2q_internal(
        const float_t &yaw, const float_t &pitch, const float_t &roll,
        quat_t &res){
      using std::cos;
      using std::sin;
      float_t cy(cos(yaw / 2)), cp(cos(pitch / 2)), cr(cos(roll / 2));
      float_t sy(sin(yaw / 2)), sp(sin(pitch / 2)), sr(sin(roll / 2));

      res[0] = cy * cp * cr + sy * sp * sr;
      res[1] = cy * cp * sr - sy * sp * cr;
      res[2] = cy * sp * cr + sy * cp * sr;
      res[3] = sy * cp * cr - cy * sp * sr;

      return res;
    }
    
  public:
    /**
     * オイラー角からquarternionに変換します。
     * 
     * @param yaw ヨー@f$ \Psi @f$
     * @param pitch ピッチ@f$ \Theta @f$
     * @param roll ロール@f$ \Phi @f$
     * @return (quat_t) 変換結果
     */
    static quat_t euler2q(
        const float_t &yaw, const float_t &pitch, const float_t &roll){
      quat_t res;
      return euler2q_internal(yaw, pitch, roll, res);
    }
    
    /**
     * 姿勢を初期化します。
     * 
     * @param yaw ヨー@f$ \Psi @f$
     * @param pitch ピッチ@f$ \Theta @f$
     * @param roll ロール@f$ \Phi @f$
     */
    void initAttitude(const float_t &yaw, const float_t &pitch, const float_t &roll){
      euler2q_internal(yaw, pitch, roll, q_n2b);
    }
    
    /**
     * 姿勢を初期化します。
     * 
     * @param q 姿勢をあらわすquarternion
     */
    void initAttitude(const quat_t &q){
      q_n2b = q.copy();
    }
  
    /**
     * コンストラクタ
     * 
     */
    INS() : omega_e2i_4e(0, 0, Earth::Omega_Earth){
      initPosition(0, 0, 0);
      initVelocity(0, 0, 0);
      initAttitude(0, 0, 0); 
    }
    
    /**
     * コピーコンストラクタ
     * 
     * @param orig コピー元
     * @param deepcopy ディープコピーを作成するかどうか
     */
    INS(const INS &orig, const bool deepcopy = false) :
      v_2e_4n(deepcopy ? orig.v_2e_4n.copy() : orig.v_2e_4n), 
      v_N(orig.v_N), v_E(orig.v_E),
      q_e2n(deepcopy ? orig.q_e2n.copy() : orig.q_e2n), 
      h(orig.h), phi(orig.phi), lambda(orig.lambda), alpha(orig.alpha),
      q_n2b(deepcopy ? orig.q_n2b.copy() : orig.q_n2b), 
      omega_e2i_4e(deepcopy ? orig.omega_e2i_4e.copy() : orig.omega_e2i_4e), 
      omega_e2i_4n(deepcopy ? orig.omega_e2i_4n.copy() : orig.omega_e2i_4n), 
      omega_n2e_4n(deepcopy ? orig.omega_n2e_4n.copy() : orig.omega_n2e_4n){
    }
    
    /**
     * デストラクタ
     * 
     */
    virtual ~INS(){}
    
    /**
     * 状態量へ直接アクセス。
     * インデックスを指定して呼び出します。
     * インデックスは
     *    0～2: @f$ \vec{v}_{e}^{n} @f$,
     *    3～6: @f$ \Tilde{q}_{e}^{n} @f$,
     *    7: @f$ h @f$,
     *    8～11: @f$ \Tilde{q}_{n}^{b} @f$
     * です。
     * 状態量を外部から変更した際は必ずrecalc()をすること。
     * 
     * @param index 状態量番号
     * @return 状態量(への参照、代入も可)
     * @see recalc()
     */
    virtual float_t &operator[](const unsigned &index){
      switch(index){
        case 1:  return v_2e_4n[1];
        case 2:  return v_2e_4n[2];
        case 3:  return q_e2n[0];
        case 4:  return q_e2n[1];
        case 5:  return q_e2n[2];
        case 6:  return q_e2n[3];
        case 7:  return h;
        case 8:  return q_n2b[0];
        case 9:  return q_n2b[1];
        case 10: return q_n2b[2];
        case 11: return q_n2b[3];
        case 0: default: return v_2e_4n[0]; // インデックス範囲外の場合、[0]の要素を返す
      }
    }
    
    const float_t &get(const unsigned &index) const {
      return const_cast<INS<float_t> *>(this)->operator[](index);
    }
    
    void set(const unsigned &index, const float_t &v){
      (*this)[index] = v;
    }
    
    /**
     * INSを更新します。
     * 内部的にはオイラー積分(1次)を利用しています。
     * 
     * @param accel 加速度
     * @param gyro 角速度
     * @param deltaT 前回の更新からの時間間隔
     */
    virtual void update(const vec3_t &accel, const vec3_t &gyro, const float_t &deltaT){
      
      //速度の運動方程式
      vec3_t delta_v_2e_4n((q_n2b * accel * q_n2b.conj()).vector());
      delta_v_2e_4n[2] += Earth::gravity(phi);
      delta_v_2e_4n -= (omega_e2i_4n * 2 + omega_n2e_4n) * v_2e_4n;
      /*T centripetal_f_scalar(beta() * pow2(Earth::Omega_Earth));
      vec3_t centripetal_f(
                    centripetal_f_scalar * -cos(lambda),
                    centripetal_f_scalar * -sin(lambda),
                    0
                  );*/
      vec3_t centripetal_f(
                    q_e2n[1] * q_e2n[3] + q_e2n[0] * q_e2n[2],
                    q_e2n[3] * q_e2n[2] - q_e2n[1] * q_e2n[0],
                    0
                  ); // 符号要検討
      centripetal_f *= (pow2(Earth::Omega_Earth) * (Earth::R_normal(phi) + h) * 2);
      delta_v_2e_4n -= (q_e2n.conj() * centripetal_f * q_e2n).vector();
      
      //位置の運動方程式
      quat_t delta_q_e2n(q_e2n * omega_n2e_4n);
      delta_q_e2n /= 2;
      float_t delta_h(v_2e_4n[2] * -1);
      
      //姿勢の運動方程式
      quat_t dot_q_n2b(0, omega_e2i_4n + omega_n2e_4n);
      dot_q_n2b *= q_n2b;
      dot_q_n2b -= q_n2b * gyro;
      dot_q_n2b /= (-2);
      
      //更新
      v_2e_4n += delta_v_2e_4n * deltaT;
      q_e2n += delta_q_e2n * deltaT;
      h += delta_h * deltaT;
      q_n2b += dot_q_n2b * deltaT;
      
      //付随的情報の再計算
      recalc();
    }
    
    float_t v_north() const{return v_N;}            /**< 北方向速度を返します。 @return (float_t) 北方向速度 */
    float_t v_east() const{return v_E;}             /**< 東方向速度を返します。 @return (float_t) 東方向速度 */
    float_t v_down() const{return v_2e_4n.get(2);}  /**< 下方向速度を返します。 @return (float_t) 下方向速度 */
    
    float_t latitude() const{return phi;}           /**< 緯度を返します。 @return (float_t) 緯度 */
    float_t longitude() const{return lambda;}       /**< 経度を返します。 @return (float_t) 経度 */
    float_t height() const{return h;}               /**< 高度を返します。 @return (float_t) 高度 */
    float_t azimuth() const{return alpha;}          /**< アジムス角を返します。 @return (float_t) アジムス角 */
    
    quat_t n2b() const{return q_n2b.copy();}  /**< 姿勢をクォータニオンで返します。 @return (float_t) 姿勢 */
    
    /* オイラー角への変換 */
    /**
     * ヨー角を返します。
     * @param q 変換元
     * @return (float_t) ヨー角
     */
    static float_t q2psi(const quat_t &q){
      return std::atan2((q.get(1) * q.get(2) + q.get(0) * q.get(3)) * 2,
                      (pow2(q.get(0)) + pow2(q.get(1)) - pow2(q.get(2)) - pow2(q.get(3))));
    }
    /**
     * ピッチ角を返します。
     * @param q 変換元
     * @return (float_t) ピッチ角
     */
    static float_t q2theta(const quat_t &q){
      return std::asin((q.get(0) * q.get(2) - q.get(1) * q.get(3)) * 2);
    }
    
    /**
     * ロール角を返します。
     * @param q 変換元
     * @return (float_t) ロール角
     */
    static float_t q2phi(const quat_t &q){
      return std::atan2((q.get(2) * q.get(3) + q.get(0) * q.get(1)) * 2,
                      (pow2(q.get(0)) - pow2(q.get(1)) - pow2(q.get(2)) + pow2(q.get(3))));
    }
    
    /**
     * ヨー角を返します。
     * @return (float_t) ヨー角
     */
    float_t euler_psi() const{return q2psi(q_n2b);}
    
    /**
     * ピッチ角を返します。
     * @return (float_t) ピッチ角
     */
    float_t euler_theta() const{return q2theta(q_n2b);}
    
    /**
     * ロール角を返します。
     * @return (float_t) ロール角
     */
    float_t euler_phi() const{return q2phi(q_n2b);}

    /**
     * ヨー角を補正します。
     * 補正後のヨー角は @f$ \psi + \delta \psi @f$ になります。
     *
     * @param delta_psi ヨー角補正量
     */
    void mod_euler_psi(const float_t &delta_psi){
      float_t delta_psi_h(delta_psi / 2);
      quat_t delta_q(std::cos(delta_psi_h), 0, 0, std::sin(delta_psi_h));
      q_n2b = delta_q * q_n2b;
    }

    /**
     * ピッチ角を補正します。
     * 補正後のピッチ角は @f$ \theta + \delta \theta @f$ になります。
     *
     * @param delta_theta ピッチ角補正量
     */
    void mod_euler_theta(const float_t &delta_theta){
      float_t delta_theta_h(delta_theta / 2);
      float_t c_theta(std::cos(delta_theta_h)), s_theta(std::sin(delta_theta_h));
      float_t roll(euler_phi());
      if(false){
        // 真面目に計算させる
        float_t roll_h(roll / 2);
        float_t c_roll(std::cos(roll_h)), s_roll(std::sin(roll_h));
        quat_t roll_q_conj(c_roll, -s_roll, 0, 0);
        quat_t roll_q(c_roll, s_roll, 0, 0);
        quat_t delta_q(std::cos(delta_theta_h), 0, std::sin(delta_theta_h), 0);
        q_n2b *= roll_q_conj; // ロールキャンセル
        q_n2b *= delta_q; // ピッチ増加
        q_n2b *= roll_q; // ロール
      }else{
        // ピッチ増加分を回転させたものと考えればよく、直接計算すると以下のようになる
        quat_t delta_q(c_theta, 0, s_theta * std::cos(roll), -s_theta * std::sin(roll));
        q_n2b *= delta_q;
      }
    }

    /**
     * ロール角を補正します。
     * 補正後のロール角は @f$ \phi + \delta \phi @f$ になります。
     *
     * @param delta_phi ロール角補正量
     */
    void mod_euler_phi(const float_t &delta_phi){
      float_t delta_phi_h(delta_phi / 2);
      quat_t delta_q(std::cos(delta_phi_h), std::sin(delta_phi_h), 0, 0);
      q_n2b *= delta_q;
    }

    /**
     * ヨー角とアジムス角を足し合わせたヘディング(真方位角)を返します。
     * @return (float_t) ヘディング
     */
    float_t heading() const{
      float_t _heading(euler_psi() + azimuth());
      return _heading > M_PI ? (_heading - (2 * M_PI)) : (_heading < -M_PI ? (_heading + (2 * M_PI)) : _heading);
    }
    
    vec3_t omega_e2i() const{return omega_e2i_4n.copy();} /**< @f$ \vec{\omega}_{e/i}^{n} @f$ を返します。 @return @f$ \vec{\omega}_{e/i}^{n} @f$ */
    vec3_t omega_n2e() const{return omega_n2e_4n.copy();} /**< @f$ \vec{\omega}_{n/e}^{n} @f$ を返します。 @return @f$ \vec{\omega}_{n/e}^{n} @f$ */
    
    /**
     * ある距離の緯線方向の移動に対して、現在位置における緯度の変化を求めます。
     * 式で表すと、
     * @f[
     *    \frac{x}{R_{\mathrm{meridian}} \left( \phi \right)}
     * @f]。
     * ここで、高度は無視できるものとしています。
     * 
     * @return (float_t) 緯度の変化
     */
    float_t meter2lat(const float_t &distance) const{return distance / Earth::R_meridian(phi);}
    /**
     * ある距離の経線方向の移動に対して、現在位置における経度の変化を求めます。
     * 式で表すと、
     * @f[
     *    \frac{x}{\beta \left( \phi \right)}
     * @f]。
     * ここで、高度は無視できるものとしています。
     * 
     * @return (float_t) 経度の変化
     */
    float_t meter2long(const float_t &distance) const{return distance / beta();}
    
    /**
     * 現在のAzimuth角を元に、指定の緯度、経度で@f$ \Tilde{q}_{e}^{n} @f$を計算します。
     * 
     * @param latitude 緯度
     * @param longitude 経度
     * @return 結果
     */ 
    quat_t e2n(const float_t &latitude, const float_t &longitude) const{
      using std::cos;
      using std::sin;
      using std::sqrt;
      float_t clat(cos(latitude / 2)), slat(sin(latitude / 2));
      float_t sqrt2(sqrt(2.));
      quat_t q(
          cos((longitude + alpha) / 2) * (clat - slat) / sqrt2,
          sin((longitude - alpha) / 2) * (clat + slat) / sqrt2,
         -cos((longitude - alpha) / 2) * (clat + slat) / sqrt2,
          sin((longitude + alpha) / 2) * (clat - slat) / sqrt2);
      return q;
    }

    /**
     * 現在の状態量を見やすい形で出力します。
     * 
     * @param out 出力ストリーム
     * @param ins 出力対象
     * @return (ostream) 出力ストリーム
     */
    friend std::ostream &operator<<(std::ostream &out, const INS<float_t> &ins){
      for(unsigned i = 0; i < ins.state_values(); i++){
        std::cout << (i == 0 ? "" : "\t") << (*const_cast<INS<float_t> *>(&ins))[i];
      }
      return out;  
    }
};

#ifdef POW2_ALREADY_DEFINED
#undef POW2_ALREADY_DEFINED
#else
#undef pow2
#endif

#endif /* __INS_H__ */
