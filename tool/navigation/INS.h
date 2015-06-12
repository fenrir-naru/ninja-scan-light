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

typedef WGS84 Earth; ///< 地球モデル

#ifdef pow2
#define POW2_ALREADY_DEFINED
#else
#define pow2(x) ((x)*(x))
#endif


/**
 * @brief ストラップダウン慣性航法装置(INS)
 * 
 * ストラップダウン慣性航法装置(INS)の動作を記述したクラス。
 * 加速度、角速度を入力として、慣性航法方程式を解き、
 * 位置、速度、姿勢を求めます。
 * 
 * @param FloatT 演算精度、通常はdouble
 */
template <class FloatT>
class INS{
  protected:
    Vector3<FloatT> v_2e_4n;      ///< n-frameにおける速度 @f$ \vec{v}_{e}^{n} @f$
    FloatT v_N,                   ///< 北方向の速度 @f$ V_{N} @f$
           v_E;                   ///< 東方向の速度 @f$ V_{E} @f$
    
    Quaternion<FloatT> q_e2n;    ///< @f$ \Tilde{q}_{e}^{n} @f$、すなわち現在の経度、緯度、Azimuth角
    FloatT h;                     ///< 現在の高度[m]
    FloatT phi,                   ///< 緯度 @f$ \phi @f$
           lambda,                ///< 経度 @f$ \lambda @f$
           alpha;                 ///< Azimuth角 @f$ \alpha @f$
    
    Quaternion<FloatT> q_n2b;    ///< n-frame -> b-frmae、すなわち現在の姿勢
    
    Vector3<FloatT> omega_e2i_4e; ///< Earth Rate @f$ \Omega_{e/i}^{e} @f$
  
    Vector3<FloatT> omega_e2i_4n; ///< @f$ \vec{\omega}_{e/i}^{n} @f$
    Vector3<FloatT> omega_n2e_4n; ///< @f$ \vec{\omega}_{n/e}^{n} @f$
    
  public:
    static const unsigned STATE_VALUES = 12; ///< 状態量の数
    virtual unsigned state_values() const {return STATE_VALUES;}
    
    /**
     * 回転軸から現在位置までの距離を返します。
     * 数式で表すと
     * @f[
     *    \left( R_{\mathrm{normal}} \left( \phi \right) + h \right) \cos{\phi} 
     * @f]
     * 
     * @return (FloatT) 回転軸から現在位置までの距離
     */
    FloatT beta() const{return (Earth::R_normal(phi) + h) * std::cos(phi);}
  
  private:
    /**
     * 緯度 @f$ \phi @f$を @f$ \Tilde{q}_{e}^{n} @f$から更新します。
     * 内部的。
     * 
     * @return (FloatT) 更新された緯度 
     */
    inline FloatT update_phi(){
      return phi = std::asin(FloatT(1) - (pow2(q_e2n.get(0)) + pow2(q_e2n.get(3))) * 2);
    }
    /**
     * 経度 @f$ \lambda @f$を @f$ \Tilde{q}_{e}^{n} @f$から更新します。
     * 内部的。
     * 
     * @return (FloatT) 更新された経度 
     */
    inline FloatT update_lambda(){
      return lambda = std::atan2(
          (q_e2n.get(0) * q_e2n.get(1) - q_e2n.get(2) * q_e2n.get(3)),
          (-q_e2n.get(0) * q_e2n.get(2) - q_e2n.get(1) * q_e2n.get(3)));
    }
    /**
     * アジムス角 @f$ \alpha @f$を @f$ \Tilde{q}_{e}^{n} @f$から更新します。
     * 内部的。
     * 
     * @return (FloatT) 更新されたアジムス角 
     */
    inline FloatT update_alpha(){
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
     * @return (FloatT) 更新された北方向速度 
     */
    inline FloatT update_v_N(const FloatT &calpha, const FloatT &salpha){
      return v_N = v_2e_4n.get(0) * calpha - v_2e_4n.get(1) * salpha;
    }
    /**
     * 東方向速度 @f$ V_{E} @f$を @f$ \vec{v}_{e}^{n} @f$から更新します。
     * 内部的。
     * 
     * @param calpha cos(alpha)
     * @param salpha sin(alpha)
     * @return (FloatT) 更新された東方向速度
     */
    inline FloatT update_v_E(const FloatT &calpha, const FloatT &salpha){
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
    inline void update_omega_n2e_4n(const FloatT &calpha, const FloatT &salpha){
      FloatT omega_n2e_4g_0 = v_E / (Earth::R_normal(phi) + h);
      FloatT omega_n2e_4g_1 = -v_N / (Earth::R_meridian(phi) + h);
      
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
      
      FloatT ca(std::cos(alpha)), sa(std::sin(alpha));

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
    void initPosition(const FloatT &latitude, const FloatT &longitude, const FloatT &height){
      using std::cos;
      using std::sin;
      phi = latitude;
      lambda = longitude;
      alpha = 0;

      FloatT cl(cos(lambda / 2)), sl(sin(lambda / 2));
      FloatT cp(cos(-phi / 2)), sp(sin(-phi / 2));
      FloatT sqrt2(std::sqrt(2.));

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
    void initVelocity(const FloatT &v_north, const FloatT &v_east, const FloatT &v_down){
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
     * @return (Quaternion<FloatT>) 変換結果
     */
    static Quaternion<FloatT> &euler2q_internal(
        const FloatT &yaw, const FloatT &pitch, const FloatT &roll,
        Quaternion<FloatT> &res){
      using std::cos;
      using std::sin;
      FloatT cy(cos(yaw / 2)), cp(cos(pitch / 2)), cr(cos(roll / 2));
      FloatT sy(sin(yaw / 2)), sp(sin(pitch / 2)), sr(sin(roll / 2));

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
     * @return (Quaternion<FloatT>) 変換結果
     */
    static Quaternion<FloatT> euler2q(
        const FloatT &yaw, const FloatT &pitch, const FloatT &roll){
      Quaternion<FloatT> res;
      return euler2q_internal(yaw, pitch, roll, res);
    }
    
    /**
     * 姿勢を初期化します。
     * 
     * @param yaw ヨー@f$ \Psi @f$
     * @param pitch ピッチ@f$ \Theta @f$
     * @param roll ロール@f$ \Phi @f$
     */
    void initAttitude(const FloatT &yaw, const FloatT &pitch, const FloatT &roll){
      euler2q_internal(yaw, pitch, roll, q_n2b);
    }
    
    /**
     * 姿勢を初期化します。
     * 
     * @param q 姿勢をあらわすquarternion
     */
    void initAttitude(const Quaternion<FloatT> &q){
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
    virtual FloatT &operator[](const unsigned &index){
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
    
    const FloatT &get(const unsigned &index) const {
      return const_cast<INS<FloatT> *>(this)->operator[](index);
    }
    
    void set(const unsigned &index, const FloatT &v){
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
    virtual void update(const Vector3<FloatT> &accel, const Vector3<FloatT> &gyro, const FloatT &deltaT){
      
      //速度の運動方程式
      Vector3<FloatT> delta_v_2e_4n((q_n2b * accel * q_n2b.conj()).vector());
      delta_v_2e_4n[2] += Earth::gravity(phi);
      delta_v_2e_4n -= (omega_e2i_4n * 2 + omega_n2e_4n) * v_2e_4n;
      /*T centripetal_f_scalar(beta() * pow2(Earth::Omega_Earth));
      Vector3<FloatT> centripetal_f(
                    centripetal_f_scalar * -cos(lambda),
                    centripetal_f_scalar * -sin(lambda),
                    0
                  );*/
      Vector3<FloatT> centripetal_f(
                    q_e2n[1] * q_e2n[3] + q_e2n[0] * q_e2n[2],
                    q_e2n[3] * q_e2n[2] - q_e2n[1] * q_e2n[0],
                    0
                  ); // 符号要検討
      centripetal_f *= (pow2(Earth::Omega_Earth) * (Earth::R_normal(phi) + h) * 2);
      delta_v_2e_4n -= (q_e2n.conj() * centripetal_f * q_e2n).vector();
      
      //位置の運動方程式
      Quaternion<FloatT> delta_q_e2n(q_e2n * omega_n2e_4n);
      delta_q_e2n /= 2;
      FloatT delta_h(v_2e_4n[2] * -1);
      
      //姿勢の運動方程式
      Quaternion<FloatT> dot_q_n2b(0, omega_e2i_4n + omega_n2e_4n);
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
    
    FloatT v_north() const{return v_N;}            /**< 北方向速度を返します。 @return (FloatT) 北方向速度 */
    FloatT v_east() const{return v_E;}             /**< 東方向速度を返します。 @return (FloatT) 東方向速度 */
    FloatT v_down() const{return v_2e_4n.get(2);}  /**< 下方向速度を返します。 @return (FloatT) 下方向速度 */
    
    FloatT latitude() const{return phi;}           /**< 緯度を返します。 @return (FloatT) 緯度 */
    FloatT longitude() const{return lambda;}       /**< 経度を返します。 @return (FloatT) 経度 */
    FloatT height() const{return h;}               /**< 高度を返します。 @return (FloatT) 高度 */
    FloatT azimuth() const{return alpha;}          /**< アジムス角を返します。 @return (FloatT) アジムス角 */
    
    Quaternion<FloatT> n2b() const{return q_n2b.copy();}  /**< 姿勢をクォータニオンで返します。 @return (FloatT) 姿勢 */
    
    /* オイラー角への変換 */
    /**
     * ヨー角を返します。
     * @param q 変換元
     * @return (FloatT) ヨー角
     */
    static FloatT q2psi(const Quaternion<FloatT> &q){
      return std::atan2((q.get(1) * q.get(2) + q.get(0) * q.get(3)) * 2,
                      (pow2(q.get(0)) + pow2(q.get(1)) - pow2(q.get(2)) - pow2(q.get(3))));
    }
    /**
     * ピッチ角を返します。
     * @param q 変換元
     * @return (FloatT) ピッチ角
     */
    static FloatT q2theta(const Quaternion<FloatT> &q){
      return std::asin((q.get(0) * q.get(2) - q.get(1) * q.get(3)) * 2);
    }
    
    /**
     * ロール角を返します。
     * @param q 変換元
     * @return (FloatT) ロール角
     */
    static FloatT q2phi(const Quaternion<FloatT> &q){
      return std::atan2((q.get(2) * q.get(3) + q.get(0) * q.get(1)) * 2,
                      (pow2(q.get(0)) - pow2(q.get(1)) - pow2(q.get(2)) + pow2(q.get(3))));
    }
    
    /**
     * ヨー角を返します。
     * @return (FloatT) ヨー角
     */
    FloatT euler_psi() const{return q2psi(q_n2b);}
    
    /**
     * ピッチ角を返します。
     * @return (FloatT) ピッチ角
     */
    FloatT euler_theta() const{return q2theta(q_n2b);}
    
    /**
     * ロール角を返します。
     * @return (FloatT) ロール角
     */
    FloatT euler_phi() const{return q2phi(q_n2b);}

    /**
     * ヨー角を補正します。
     * 補正後のヨー角は @f$ \psi + \delta \psi @f$ になります。
     *
     * @param delta_psi ヨー角補正量
     */
    void mod_euler_psi(const FloatT &delta_psi){
      FloatT delta_psi_h(delta_psi / 2);
      Quaternion<FloatT> delta_q(std::cos(delta_psi_h), 0, 0, std::sin(delta_psi_h));
      q_n2b = delta_q * q_n2b;
    }

    /**
     * ピッチ角を補正します。
     * 補正後のピッチ角は @f$ \theta + \delta \theta @f$ になります。
     *
     * @param delta_theta ピッチ角補正量
     */
    void mod_euler_theta(const FloatT &delta_theta){
      FloatT delta_theta_h(delta_theta / 2);
      FloatT c_theta(std::cos(delta_theta_h)), s_theta(std::sin(delta_theta_h));
      FloatT roll(euler_phi());
      if(false){
        // 真面目に計算させる
        FloatT roll_h(roll / 2);
        FloatT c_roll(std::cos(roll_h)), s_roll(std::sin(roll_h));
        Quaternion<FloatT> roll_q_conj(c_roll, -s_roll, 0, 0);
        Quaternion<FloatT> roll_q(c_roll, s_roll, 0, 0);
        Quaternion<FloatT> delta_q(std::cos(delta_theta_h), 0, std::sin(delta_theta_h), 0);
        q_n2b *= roll_q_conj; // ロールキャンセル
        q_n2b *= delta_q; // ピッチ増加
        q_n2b *= roll_q; // ロール
      }else{
        // ピッチ増加分を回転させたものと考えればよく、直接計算すると以下のようになる
        Quaternion<FloatT> delta_q(c_theta, 0, s_theta * std::cos(roll), -s_theta * std::sin(roll));
        q_n2b *= delta_q;
      }
    }

    /**
     * ロール角を補正します。
     * 補正後のロール角は @f$ \phi + \delta \phi @f$ になります。
     *
     * @param delta_phi ロール角補正量
     */
    void mod_euler_phi(const FloatT &delta_phi){
      FloatT delta_phi_h(delta_phi / 2);
      Quaternion<FloatT> delta_q(std::cos(delta_phi_h), std::sin(delta_phi_h), 0, 0);
      q_n2b *= delta_q;
    }

    /**
     * ヨー角とアジムス角を足し合わせたヘディング(真方位角)を返します。
     * @return (FloatT) ヘディング
     */
    FloatT heading() const{
      FloatT _heading(euler_psi() + azimuth());
      return _heading > M_PI ? (_heading - (2 * M_PI)) : (_heading < -M_PI ? (_heading + (2 * M_PI)) : _heading);
    }
    
    Vector3<FloatT> omega_e2i() const{return omega_e2i_4n.copy();} /**< @f$ \vec{\omega}_{e/i}^{n} @f$ を返します。 @return @f$ \vec{\omega}_{e/i}^{n} @f$ */
    Vector3<FloatT> omega_n2e() const{return omega_n2e_4n.copy();} /**< @f$ \vec{\omega}_{n/e}^{n} @f$ を返します。 @return @f$ \vec{\omega}_{n/e}^{n} @f$ */
    
    /**
     * ある距離の緯線方向の移動に対して、現在位置における緯度の変化を求めます。
     * 式で表すと、
     * @f[
     *    \frac{x}{R_{\mathrm{meridian}} \left( \phi \right)}
     * @f]。
     * ここで、高度は無視できるものとしています。
     * 
     * @return (FloatT) 緯度の変化
     */
    FloatT meter2lat(const FloatT &distance) const{return distance / Earth::R_meridian(phi);}
    /**
     * ある距離の経線方向の移動に対して、現在位置における経度の変化を求めます。
     * 式で表すと、
     * @f[
     *    \frac{x}{\beta \left( \phi \right)}
     * @f]。
     * ここで、高度は無視できるものとしています。
     * 
     * @return (FloatT) 経度の変化
     */
    FloatT meter2long(const FloatT &distance) const{return distance / beta();}
    
    /**
     * 現在のAzimuth角を元に、指定の緯度、経度で@f$ \Tilde{q}_{e}^{n} @f$を計算します。
     * 
     * @param latitude 緯度
     * @param longitude 経度
     * @return 結果
     */ 
    Quaternion<FloatT> e2n(const FloatT &latitude, const FloatT &longitude) const{
      using std::cos;
      using std::sin;
      using std::sqrt;
      FloatT clat(cos(latitude / 2)), slat(sin(latitude / 2));
      FloatT sqrt2(sqrt(2.));
      Quaternion<FloatT> q(
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
    friend std::ostream &operator<<(std::ostream &out, const INS<FloatT> &ins){
      for(unsigned i = 0; i < ins.state_values(); i++){
        std::cout << (i == 0 ? "" : "\t") << (*const_cast<INS<FloatT> *>(&ins))[i];
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
