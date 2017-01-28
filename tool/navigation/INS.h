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
 * @brief �����q�@���u(INS)
 * 
 * �����q�@���u(INS)�ɂ��ċL�q�����t�@�C���B
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
  static const unsigned STATE_VALUES = 12; ///< ��ԗʂ̐�
};

/**
 * @brief �X�g���b�v�_�E�������q�@���u(INS)
 * 
 * �X�g���b�v�_�E�������q�@���u(INS)�̓�����L�q�����N���X�B
 * �����x�A�p���x����͂Ƃ��āA�����q�@�������������A
 * �ʒu�A���x�A�p�������߂܂��B
 * 
 * @param FloatT ���Z���x�A�ʏ��double
 */
template <class FloatT = double>
class INS {
  public:
    typedef FloatT float_t;
    typedef Vector3<float_t> vec3_t;
    typedef Quaternion<float_t> quat_t;
  protected:
    vec3_t v_2e_4n;       ///< n-frame�ɂ����鑬�x @f$ \vec{v}_{e}^{n} @f$
    float_t v_N,          ///< �k�����̑��x @f$ V_{N} @f$
           v_E;           ///< �������̑��x @f$ V_{E} @f$
    
    quat_t q_e2n;         ///< @f$ \Tilde{q}_{e}^{n} @f$�A���Ȃ킿���݂̌o�x�A�ܓx�AAzimuth�p
    float_t h;            ///< ���݂̍��x[m]
    float_t phi,          ///< �ܓx @f$ \phi @f$
           lambda,        ///< �o�x @f$ \lambda @f$
           alpha;         ///< Azimuth�p @f$ \alpha @f$
    
    quat_t q_n2b;         ///< n-frame -> b-frmae�A���Ȃ킿���݂̎p��
    
    vec3_t omega_e2i_4e;  ///< Earth Rate @f$ \Omega_{e/i}^{e} @f$
  
    vec3_t omega_e2i_4n;  ///< @f$ \vec{\omega}_{e/i}^{n} @f$
    vec3_t omega_n2e_4n;  ///< @f$ \vec{\omega}_{n/e}^{n} @f$
    
  public:
    typedef WGS84Generic<float_t> Earth; ///< �n�����f��
    static const unsigned STATE_VALUES = INS_Property<INS>::STATE_VALUES;
    virtual unsigned state_values() const {return STATE_VALUES;}
    
    /**
     * ��]�����猻�݈ʒu�܂ł̋�����Ԃ��܂��B
     * �����ŕ\����
     * @f[
     *    \left( R_{\mathrm{normal}} \left( \phi \right) + h \right) \cos{\phi} 
     * @f]
     * 
     * @return (float_t) ��]�����猻�݈ʒu�܂ł̋���
     */
    float_t beta() const{return (Earth::R_normal(phi) + h) * std::cos(phi);}
  
  private:
    /**
     * �ܓx @f$ \phi @f$�� @f$ \Tilde{q}_{e}^{n} @f$����X�V���܂��B
     * �����I�B
     * 
     * @return (float_t) �X�V���ꂽ�ܓx
     */
    inline float_t update_phi(){
      return phi = std::asin(float_t(1) - (pow2(q_e2n.get(0)) + pow2(q_e2n.get(3))) * 2);
    }
    /**
     * �o�x @f$ \lambda @f$�� @f$ \Tilde{q}_{e}^{n} @f$����X�V���܂��B
     * �����I�B
     * 
     * @return (float_t) �X�V���ꂽ�o�x
     */
    inline float_t update_lambda(){
      return lambda = std::atan2(
          (q_e2n.get(0) * q_e2n.get(1) - q_e2n.get(2) * q_e2n.get(3)),
          (-q_e2n.get(0) * q_e2n.get(2) - q_e2n.get(1) * q_e2n.get(3)));
    }
    /**
     * �A�W���X�p @f$ \alpha @f$�� @f$ \Tilde{q}_{e}^{n} @f$����X�V���܂��B
     * �����I�B
     * 
     * @return (float_t) �X�V���ꂽ�A�W���X�p
     */
    inline float_t update_alpha(){
      return alpha = std::atan2(
          (-q_e2n.get(0) * q_e2n.get(1) - q_e2n.get(2) * q_e2n.get(3)),
          (q_e2n.get(1) * q_e2n.get(3) - q_e2n.get(0) * q_e2n.get(2)));
    }
    /**
     * �k�������x @f$ V_{N} @f$�� @f$ \vec{v}_{e}^{n} @f$����X�V���܂��B
     * �����I�B
     * 
     * @param calpha cos(alpha)
     * @param salpha sin(alpha)
     * @return (float_t) �X�V���ꂽ�k�������x
     */
    inline float_t update_v_N(const float_t &calpha, const float_t &salpha){
      return v_N = v_2e_4n.get(0) * calpha - v_2e_4n.get(1) * salpha;
    }
    /**
     * ���������x @f$ V_{E} @f$�� @f$ \vec{v}_{e}^{n} @f$����X�V���܂��B
     * �����I�B
     * 
     * @param calpha cos(alpha)
     * @param salpha sin(alpha)
     * @return (float_t) �X�V���ꂽ���������x
     */
    inline float_t update_v_E(const float_t &calpha, const float_t &salpha){
      return v_E = v_2e_4n.get(0) * salpha + v_2e_4n.get(1) * calpha;
    }
    
    /**
     * ���݈ʒu��ł�@f$ \vec{\omega}_{e/i}^{n} @f$�����߂܂��B
     * �����I�B
     */
    inline void update_omega_e2i_4n(){
      omega_e2i_4n = (q_e2n.conj() * omega_e2i_4e * q_e2n).vector();
    }
    /**
     * ���݈ʒu��ł�@f$ \vec{\omega}_{n/e}^{n} @f$�����߂܂��B
     * �����I�B
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
     * ���݈ʒu��ł�@f$ \vec{\omega}_{n/e}^{n} @f$�����߂܂��B
     * �����I�B
     */
    inline void update_omega_n2e_4n(){
      update_omega_n2e_4n(std::cos(alpha), std::sin(alpha));
    }
  protected:
    /**
     * �t�я����Čv�Z���čŐV�̏�Ԃɕۂ��܂��B
     * 
     * @param regularize �����̃N�H�[�^�j�I���𐳋K��(�m������1�ɂ���)���ǂ���
     */
    inline void recalc(const bool regularize = true){
      //���K��
      if(regularize){
        q_e2n = q_e2n.regularize();
        q_n2b = q_n2b.regularize();
      }
      
      //�ܓx�A�o�x�AAzimuth�p�̍X�V
      update_phi();
      update_lambda();
      update_alpha();
      
      float_t ca(std::cos(alpha)), sa(std::sin(alpha));

      //v_north, v_east�̍X�V
      update_v_N(ca, sa);
      update_v_E(ca, sa);
      
      //��_{e/i}^{n}
      update_omega_e2i_4n();
      
      //��_{n/e}^{n}
      update_omega_n2e_4n(ca, sa);
    }
    
  public:
    /**
     * �ʒu�����������܂��B
     * 
     * @param latitude �ܓx @f$ \phi @f$
     * @param longitude �o�x @f$ \lambda @f$
     * @param height ���x @f$ h @f$
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
     * ���x�����������܂��B
     * 
     * @param v_north �k�������x @f$ V_{N} @f$
     * @param v_east ���������x @f$ V_{E} @f$
     * @param v_down ���������x @f$ V_{D} @f$
     */
    void initVelocity(const float_t &v_north, const float_t &v_east, const float_t &v_down){
      v_2e_4n[0] = v_N = v_north;
      v_2e_4n[1] = v_E = v_east;
      v_2e_4n[2] = v_down;
      
      update_omega_n2e_4n();
    }
    
  protected:
    /**
     * �I�C���[�p����quarternion�ɕϊ����܂��B
     * 
     * @param yaw ���[@f$ \Psi @f$
     * @param pitch �s�b�`@f$ \Theta @f$
     * @param roll ���[��@f$ \Phi @f$
     * @param res ���ʂ��i�[�����
     * @return (quat_t) �ϊ�����
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
     * �I�C���[�p����quarternion�ɕϊ����܂��B
     * 
     * @param yaw ���[@f$ \Psi @f$
     * @param pitch �s�b�`@f$ \Theta @f$
     * @param roll ���[��@f$ \Phi @f$
     * @return (quat_t) �ϊ�����
     */
    static quat_t euler2q(
        const float_t &yaw, const float_t &pitch, const float_t &roll){
      quat_t res;
      return euler2q_internal(yaw, pitch, roll, res);
    }
    
    /**
     * �p�������������܂��B
     * 
     * @param yaw ���[@f$ \Psi @f$
     * @param pitch �s�b�`@f$ \Theta @f$
     * @param roll ���[��@f$ \Phi @f$
     */
    void initAttitude(const float_t &yaw, const float_t &pitch, const float_t &roll){
      euler2q_internal(yaw, pitch, roll, q_n2b);
    }
    
    /**
     * �p�������������܂��B
     * 
     * @param q �p��������킷quarternion
     */
    void initAttitude(const quat_t &q){
      q_n2b = q.copy();
    }
  
    /**
     * �R���X�g���N�^
     * 
     */
    INS() : omega_e2i_4e(0, 0, Earth::Omega_Earth){
      initPosition(0, 0, 0);
      initVelocity(0, 0, 0);
      initAttitude(0, 0, 0); 
    }
    
    /**
     * �R�s�[�R���X�g���N�^
     * 
     * @param orig �R�s�[��
     * @param deepcopy �f�B�[�v�R�s�[���쐬���邩�ǂ���
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
     * �f�X�g���N�^
     * 
     */
    virtual ~INS(){}
    
    /**
     * ��ԗʂ֒��ڃA�N�Z�X�B
     * �C���f�b�N�X���w�肵�ČĂяo���܂��B
     * �C���f�b�N�X��
     *    0�`2: @f$ \vec{v}_{e}^{n} @f$,
     *    3�`6: @f$ \Tilde{q}_{e}^{n} @f$,
     *    7: @f$ h @f$,
     *    8�`11: @f$ \Tilde{q}_{n}^{b} @f$
     * �ł��B
     * ��ԗʂ��O������ύX�����ۂ͕K��recalc()�����邱�ƁB
     * 
     * @param index ��ԗʔԍ�
     * @return ��ԗ�(�ւ̎Q�ƁA�������)
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
        case 0: default: return v_2e_4n[0]; // �C���f�b�N�X�͈͊O�̏ꍇ�A[0]�̗v�f��Ԃ�
      }
    }
    
    const float_t &get(const unsigned &index) const {
      return const_cast<INS<float_t> *>(this)->operator[](index);
    }
    
    void set(const unsigned &index, const float_t &v){
      (*this)[index] = v;
    }
    
    /**
     * INS���X�V���܂��B
     * �����I�ɂ̓I�C���[�ϕ�(1��)�𗘗p���Ă��܂��B
     * 
     * @param accel �����x
     * @param gyro �p���x
     * @param deltaT �O��̍X�V����̎��ԊԊu
     */
    virtual void update(const vec3_t &accel, const vec3_t &gyro, const float_t &deltaT){
      
      //���x�̉^��������
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
                  ); // �����v����
      centripetal_f *= (pow2(Earth::Omega_Earth) * (Earth::R_normal(phi) + h) * 2);
      delta_v_2e_4n -= (q_e2n.conj() * centripetal_f * q_e2n).vector();
      
      //�ʒu�̉^��������
      quat_t delta_q_e2n(q_e2n * omega_n2e_4n);
      delta_q_e2n /= 2;
      float_t delta_h(v_2e_4n[2] * -1);
      
      //�p���̉^��������
      quat_t dot_q_n2b(0, omega_e2i_4n + omega_n2e_4n);
      dot_q_n2b *= q_n2b;
      dot_q_n2b -= q_n2b * gyro;
      dot_q_n2b /= (-2);
      
      //�X�V
      v_2e_4n += delta_v_2e_4n * deltaT;
      q_e2n += delta_q_e2n * deltaT;
      h += delta_h * deltaT;
      q_n2b += dot_q_n2b * deltaT;
      
      //�t���I���̍Čv�Z
      recalc();
    }
    
    float_t v_north() const{return v_N;}            /**< �k�������x��Ԃ��܂��B @return (float_t) �k�������x */
    float_t v_east() const{return v_E;}             /**< ���������x��Ԃ��܂��B @return (float_t) ���������x */
    float_t v_down() const{return v_2e_4n.get(2);}  /**< ���������x��Ԃ��܂��B @return (float_t) ���������x */
    
    float_t latitude() const{return phi;}           /**< �ܓx��Ԃ��܂��B @return (float_t) �ܓx */
    float_t longitude() const{return lambda;}       /**< �o�x��Ԃ��܂��B @return (float_t) �o�x */
    float_t height() const{return h;}               /**< ���x��Ԃ��܂��B @return (float_t) ���x */
    float_t azimuth() const{return alpha;}          /**< �A�W���X�p��Ԃ��܂��B @return (float_t) �A�W���X�p */
    
    quat_t n2b() const{return q_n2b.copy();}  /**< �p�����N�H�[�^�j�I���ŕԂ��܂��B @return (float_t) �p�� */
    
    /* �I�C���[�p�ւ̕ϊ� */
    /**
     * ���[�p��Ԃ��܂��B
     * @param q �ϊ���
     * @return (float_t) ���[�p
     */
    static float_t q2psi(const quat_t &q){
      return std::atan2((q.get(1) * q.get(2) + q.get(0) * q.get(3)) * 2,
                      (pow2(q.get(0)) + pow2(q.get(1)) - pow2(q.get(2)) - pow2(q.get(3))));
    }
    /**
     * �s�b�`�p��Ԃ��܂��B
     * @param q �ϊ���
     * @return (float_t) �s�b�`�p
     */
    static float_t q2theta(const quat_t &q){
      return std::asin((q.get(0) * q.get(2) - q.get(1) * q.get(3)) * 2);
    }
    
    /**
     * ���[���p��Ԃ��܂��B
     * @param q �ϊ���
     * @return (float_t) ���[���p
     */
    static float_t q2phi(const quat_t &q){
      return std::atan2((q.get(2) * q.get(3) + q.get(0) * q.get(1)) * 2,
                      (pow2(q.get(0)) - pow2(q.get(1)) - pow2(q.get(2)) + pow2(q.get(3))));
    }
    
    /**
     * ���[�p��Ԃ��܂��B
     * @return (float_t) ���[�p
     */
    float_t euler_psi() const{return q2psi(q_n2b);}
    
    /**
     * �s�b�`�p��Ԃ��܂��B
     * @return (float_t) �s�b�`�p
     */
    float_t euler_theta() const{return q2theta(q_n2b);}
    
    /**
     * ���[���p��Ԃ��܂��B
     * @return (float_t) ���[���p
     */
    float_t euler_phi() const{return q2phi(q_n2b);}

    /**
     * ���[�p��␳���܂��B
     * �␳��̃��[�p�� @f$ \psi + \delta \psi @f$ �ɂȂ�܂��B
     *
     * @param delta_psi ���[�p�␳��
     */
    void mod_euler_psi(const float_t &delta_psi){
      float_t delta_psi_h(delta_psi / 2);
      quat_t delta_q(std::cos(delta_psi_h), 0, 0, std::sin(delta_psi_h));
      q_n2b = delta_q * q_n2b;
    }

    /**
     * �s�b�`�p��␳���܂��B
     * �␳��̃s�b�`�p�� @f$ \theta + \delta \theta @f$ �ɂȂ�܂��B
     *
     * @param delta_theta �s�b�`�p�␳��
     */
    void mod_euler_theta(const float_t &delta_theta){
      float_t delta_theta_h(delta_theta / 2);
      float_t c_theta(std::cos(delta_theta_h)), s_theta(std::sin(delta_theta_h));
      float_t roll(euler_phi());
      if(false){
        // �^�ʖڂɌv�Z������
        float_t roll_h(roll / 2);
        float_t c_roll(std::cos(roll_h)), s_roll(std::sin(roll_h));
        quat_t roll_q_conj(c_roll, -s_roll, 0, 0);
        quat_t roll_q(c_roll, s_roll, 0, 0);
        quat_t delta_q(std::cos(delta_theta_h), 0, std::sin(delta_theta_h), 0);
        q_n2b *= roll_q_conj; // ���[���L�����Z��
        q_n2b *= delta_q; // �s�b�`����
        q_n2b *= roll_q; // ���[��
      }else{
        // �s�b�`����������]���������̂ƍl����΂悭�A���ڌv�Z����ƈȉ��̂悤�ɂȂ�
        quat_t delta_q(c_theta, 0, s_theta * std::cos(roll), -s_theta * std::sin(roll));
        q_n2b *= delta_q;
      }
    }

    /**
     * ���[���p��␳���܂��B
     * �␳��̃��[���p�� @f$ \phi + \delta \phi @f$ �ɂȂ�܂��B
     *
     * @param delta_phi ���[���p�␳��
     */
    void mod_euler_phi(const float_t &delta_phi){
      float_t delta_phi_h(delta_phi / 2);
      quat_t delta_q(std::cos(delta_phi_h), std::sin(delta_phi_h), 0, 0);
      q_n2b *= delta_q;
    }

    /**
     * ���[�p�ƃA�W���X�p�𑫂����킹���w�f�B���O(�^���ʊp)��Ԃ��܂��B
     * @return (float_t) �w�f�B���O
     */
    float_t heading() const{
      float_t _heading(euler_psi() + azimuth());
      return _heading > M_PI ? (_heading - (2 * M_PI)) : (_heading < -M_PI ? (_heading + (2 * M_PI)) : _heading);
    }
    
    vec3_t omega_e2i() const{return omega_e2i_4n.copy();} /**< @f$ \vec{\omega}_{e/i}^{n} @f$ ��Ԃ��܂��B @return @f$ \vec{\omega}_{e/i}^{n} @f$ */
    vec3_t omega_n2e() const{return omega_n2e_4n.copy();} /**< @f$ \vec{\omega}_{n/e}^{n} @f$ ��Ԃ��܂��B @return @f$ \vec{\omega}_{n/e}^{n} @f$ */
    
    /**
     * ���鋗���̈ܐ������̈ړ��ɑ΂��āA���݈ʒu�ɂ�����ܓx�̕ω������߂܂��B
     * ���ŕ\���ƁA
     * @f[
     *    \frac{x}{R_{\mathrm{meridian}} \left( \phi \right)}
     * @f]�B
     * �����ŁA���x�͖����ł�����̂Ƃ��Ă��܂��B
     * 
     * @return (float_t) �ܓx�̕ω�
     */
    float_t meter2lat(const float_t &distance) const{return distance / Earth::R_meridian(phi);}
    /**
     * ���鋗���̌o�������̈ړ��ɑ΂��āA���݈ʒu�ɂ�����o�x�̕ω������߂܂��B
     * ���ŕ\���ƁA
     * @f[
     *    \frac{x}{\beta \left( \phi \right)}
     * @f]�B
     * �����ŁA���x�͖����ł�����̂Ƃ��Ă��܂��B
     * 
     * @return (float_t) �o�x�̕ω�
     */
    float_t meter2long(const float_t &distance) const{return distance / beta();}
    
    /**
     * ���݂�Azimuth�p�����ɁA�w��̈ܓx�A�o�x��@f$ \Tilde{q}_{e}^{n} @f$���v�Z���܂��B
     * 
     * @param latitude �ܓx
     * @param longitude �o�x
     * @return ����
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
     * ���݂̏�ԗʂ����₷���`�ŏo�͂��܂��B
     * 
     * @param out �o�̓X�g���[��
     * @param ins �o�͑Ώ�
     * @return (ostream) �o�̓X�g���[��
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
