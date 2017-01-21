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


/**
 * @brief �X�g���b�v�_�E�������q�@���u(INS)
 * 
 * �X�g���b�v�_�E�������q�@���u(INS)�̓�����L�q�����N���X�B
 * �����x�A�p���x����͂Ƃ��āA�����q�@�������������A
 * �ʒu�A���x�A�p�������߂܂��B
 * 
 * @param FloatT ���Z���x�A�ʏ��double
 */
template <class FloatT>
class INS{
  protected:
    Vector3<FloatT> v_2e_4n;      ///< n-frame�ɂ����鑬�x @f$ \vec{v}_{e}^{n} @f$
    FloatT v_N,                   ///< �k�����̑��x @f$ V_{N} @f$
           v_E;                   ///< �������̑��x @f$ V_{E} @f$
    
    Quaternion<FloatT> q_e2n;    ///< @f$ \Tilde{q}_{e}^{n} @f$�A���Ȃ킿���݂̌o�x�A�ܓx�AAzimuth�p
    FloatT h;                     ///< ���݂̍��x[m]
    FloatT phi,                   ///< �ܓx @f$ \phi @f$
           lambda,                ///< �o�x @f$ \lambda @f$
           alpha;                 ///< Azimuth�p @f$ \alpha @f$
    
    Quaternion<FloatT> q_n2b;    ///< n-frame -> b-frmae�A���Ȃ킿���݂̎p��
    
    Vector3<FloatT> omega_e2i_4e; ///< Earth Rate @f$ \Omega_{e/i}^{e} @f$
  
    Vector3<FloatT> omega_e2i_4n; ///< @f$ \vec{\omega}_{e/i}^{n} @f$
    Vector3<FloatT> omega_n2e_4n; ///< @f$ \vec{\omega}_{n/e}^{n} @f$
    
  public:
    typedef WGS84Generic<FloatT> Earth; ///< �n�����f��
    static const unsigned STATE_VALUES = 12; ///< ��ԗʂ̐�
    virtual unsigned state_values() const {return STATE_VALUES;}
    
    /**
     * ��]�����猻�݈ʒu�܂ł̋�����Ԃ��܂��B
     * �����ŕ\����
     * @f[
     *    \left( R_{\mathrm{normal}} \left( \phi \right) + h \right) \cos{\phi} 
     * @f]
     * 
     * @return (FloatT) ��]�����猻�݈ʒu�܂ł̋���
     */
    FloatT beta() const{return (Earth::R_normal(phi) + h) * std::cos(phi);}
  
  private:
    /**
     * �ܓx @f$ \phi @f$�� @f$ \Tilde{q}_{e}^{n} @f$����X�V���܂��B
     * �����I�B
     * 
     * @return (FloatT) �X�V���ꂽ�ܓx 
     */
    inline FloatT update_phi(){
      return phi = std::asin(FloatT(1) - (pow2(q_e2n.get(0)) + pow2(q_e2n.get(3))) * 2);
    }
    /**
     * �o�x @f$ \lambda @f$�� @f$ \Tilde{q}_{e}^{n} @f$����X�V���܂��B
     * �����I�B
     * 
     * @return (FloatT) �X�V���ꂽ�o�x 
     */
    inline FloatT update_lambda(){
      return lambda = std::atan2(
          (q_e2n.get(0) * q_e2n.get(1) - q_e2n.get(2) * q_e2n.get(3)),
          (-q_e2n.get(0) * q_e2n.get(2) - q_e2n.get(1) * q_e2n.get(3)));
    }
    /**
     * �A�W���X�p @f$ \alpha @f$�� @f$ \Tilde{q}_{e}^{n} @f$����X�V���܂��B
     * �����I�B
     * 
     * @return (FloatT) �X�V���ꂽ�A�W���X�p 
     */
    inline FloatT update_alpha(){
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
     * @return (FloatT) �X�V���ꂽ�k�������x 
     */
    inline FloatT update_v_N(const FloatT &calpha, const FloatT &salpha){
      return v_N = v_2e_4n.get(0) * calpha - v_2e_4n.get(1) * salpha;
    }
    /**
     * ���������x @f$ V_{E} @f$�� @f$ \vec{v}_{e}^{n} @f$����X�V���܂��B
     * �����I�B
     * 
     * @param calpha cos(alpha)
     * @param salpha sin(alpha)
     * @return (FloatT) �X�V���ꂽ���������x
     */
    inline FloatT update_v_E(const FloatT &calpha, const FloatT &salpha){
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
    inline void update_omega_n2e_4n(const FloatT &calpha, const FloatT &salpha){
      FloatT omega_n2e_4g_0 = v_E / (Earth::R_normal(phi) + h);
      FloatT omega_n2e_4g_1 = -v_N / (Earth::R_meridian(phi) + h);
      
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
      
      FloatT ca(std::cos(alpha)), sa(std::sin(alpha));

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
     * ���x�����������܂��B
     * 
     * @param v_north �k�������x @f$ V_{N} @f$
     * @param v_east ���������x @f$ V_{E} @f$
     * @param v_down ���������x @f$ V_{D} @f$
     */
    void initVelocity(const FloatT &v_north, const FloatT &v_east, const FloatT &v_down){
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
     * @return (Quaternion<FloatT>) �ϊ�����
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
     * �I�C���[�p����quarternion�ɕϊ����܂��B
     * 
     * @param yaw ���[@f$ \Psi @f$
     * @param pitch �s�b�`@f$ \Theta @f$
     * @param roll ���[��@f$ \Phi @f$
     * @return (Quaternion<FloatT>) �ϊ�����
     */
    static Quaternion<FloatT> euler2q(
        const FloatT &yaw, const FloatT &pitch, const FloatT &roll){
      Quaternion<FloatT> res;
      return euler2q_internal(yaw, pitch, roll, res);
    }
    
    /**
     * �p�������������܂��B
     * 
     * @param yaw ���[@f$ \Psi @f$
     * @param pitch �s�b�`@f$ \Theta @f$
     * @param roll ���[��@f$ \Phi @f$
     */
    void initAttitude(const FloatT &yaw, const FloatT &pitch, const FloatT &roll){
      euler2q_internal(yaw, pitch, roll, q_n2b);
    }
    
    /**
     * �p�������������܂��B
     * 
     * @param q �p��������킷quarternion
     */
    void initAttitude(const Quaternion<FloatT> &q){
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
        case 0: default: return v_2e_4n[0]; // �C���f�b�N�X�͈͊O�̏ꍇ�A[0]�̗v�f��Ԃ�
      }
    }
    
    const FloatT &get(const unsigned &index) const {
      return const_cast<INS<FloatT> *>(this)->operator[](index);
    }
    
    void set(const unsigned &index, const FloatT &v){
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
    virtual void update(const Vector3<FloatT> &accel, const Vector3<FloatT> &gyro, const FloatT &deltaT){
      
      //���x�̉^��������
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
                  ); // �����v����
      centripetal_f *= (pow2(Earth::Omega_Earth) * (Earth::R_normal(phi) + h) * 2);
      delta_v_2e_4n -= (q_e2n.conj() * centripetal_f * q_e2n).vector();
      
      //�ʒu�̉^��������
      Quaternion<FloatT> delta_q_e2n(q_e2n * omega_n2e_4n);
      delta_q_e2n /= 2;
      FloatT delta_h(v_2e_4n[2] * -1);
      
      //�p���̉^��������
      Quaternion<FloatT> dot_q_n2b(0, omega_e2i_4n + omega_n2e_4n);
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
    
    FloatT v_north() const{return v_N;}            /**< �k�������x��Ԃ��܂��B @return (FloatT) �k�������x */
    FloatT v_east() const{return v_E;}             /**< ���������x��Ԃ��܂��B @return (FloatT) ���������x */
    FloatT v_down() const{return v_2e_4n.get(2);}  /**< ���������x��Ԃ��܂��B @return (FloatT) ���������x */
    
    FloatT latitude() const{return phi;}           /**< �ܓx��Ԃ��܂��B @return (FloatT) �ܓx */
    FloatT longitude() const{return lambda;}       /**< �o�x��Ԃ��܂��B @return (FloatT) �o�x */
    FloatT height() const{return h;}               /**< ���x��Ԃ��܂��B @return (FloatT) ���x */
    FloatT azimuth() const{return alpha;}          /**< �A�W���X�p��Ԃ��܂��B @return (FloatT) �A�W���X�p */
    
    Quaternion<FloatT> n2b() const{return q_n2b.copy();}  /**< �p�����N�H�[�^�j�I���ŕԂ��܂��B @return (FloatT) �p�� */
    
    /* �I�C���[�p�ւ̕ϊ� */
    /**
     * ���[�p��Ԃ��܂��B
     * @param q �ϊ���
     * @return (FloatT) ���[�p
     */
    static FloatT q2psi(const Quaternion<FloatT> &q){
      return std::atan2((q.get(1) * q.get(2) + q.get(0) * q.get(3)) * 2,
                      (pow2(q.get(0)) + pow2(q.get(1)) - pow2(q.get(2)) - pow2(q.get(3))));
    }
    /**
     * �s�b�`�p��Ԃ��܂��B
     * @param q �ϊ���
     * @return (FloatT) �s�b�`�p
     */
    static FloatT q2theta(const Quaternion<FloatT> &q){
      return std::asin((q.get(0) * q.get(2) - q.get(1) * q.get(3)) * 2);
    }
    
    /**
     * ���[���p��Ԃ��܂��B
     * @param q �ϊ���
     * @return (FloatT) ���[���p
     */
    static FloatT q2phi(const Quaternion<FloatT> &q){
      return std::atan2((q.get(2) * q.get(3) + q.get(0) * q.get(1)) * 2,
                      (pow2(q.get(0)) - pow2(q.get(1)) - pow2(q.get(2)) + pow2(q.get(3))));
    }
    
    /**
     * ���[�p��Ԃ��܂��B
     * @return (FloatT) ���[�p
     */
    FloatT euler_psi() const{return q2psi(q_n2b);}
    
    /**
     * �s�b�`�p��Ԃ��܂��B
     * @return (FloatT) �s�b�`�p
     */
    FloatT euler_theta() const{return q2theta(q_n2b);}
    
    /**
     * ���[���p��Ԃ��܂��B
     * @return (FloatT) ���[���p
     */
    FloatT euler_phi() const{return q2phi(q_n2b);}

    /**
     * ���[�p��␳���܂��B
     * �␳��̃��[�p�� @f$ \psi + \delta \psi @f$ �ɂȂ�܂��B
     *
     * @param delta_psi ���[�p�␳��
     */
    void mod_euler_psi(const FloatT &delta_psi){
      FloatT delta_psi_h(delta_psi / 2);
      Quaternion<FloatT> delta_q(std::cos(delta_psi_h), 0, 0, std::sin(delta_psi_h));
      q_n2b = delta_q * q_n2b;
    }

    /**
     * �s�b�`�p��␳���܂��B
     * �␳��̃s�b�`�p�� @f$ \theta + \delta \theta @f$ �ɂȂ�܂��B
     *
     * @param delta_theta �s�b�`�p�␳��
     */
    void mod_euler_theta(const FloatT &delta_theta){
      FloatT delta_theta_h(delta_theta / 2);
      FloatT c_theta(std::cos(delta_theta_h)), s_theta(std::sin(delta_theta_h));
      FloatT roll(euler_phi());
      if(false){
        // �^�ʖڂɌv�Z������
        FloatT roll_h(roll / 2);
        FloatT c_roll(std::cos(roll_h)), s_roll(std::sin(roll_h));
        Quaternion<FloatT> roll_q_conj(c_roll, -s_roll, 0, 0);
        Quaternion<FloatT> roll_q(c_roll, s_roll, 0, 0);
        Quaternion<FloatT> delta_q(std::cos(delta_theta_h), 0, std::sin(delta_theta_h), 0);
        q_n2b *= roll_q_conj; // ���[���L�����Z��
        q_n2b *= delta_q; // �s�b�`����
        q_n2b *= roll_q; // ���[��
      }else{
        // �s�b�`����������]���������̂ƍl����΂悭�A���ڌv�Z����ƈȉ��̂悤�ɂȂ�
        Quaternion<FloatT> delta_q(c_theta, 0, s_theta * std::cos(roll), -s_theta * std::sin(roll));
        q_n2b *= delta_q;
      }
    }

    /**
     * ���[���p��␳���܂��B
     * �␳��̃��[���p�� @f$ \phi + \delta \phi @f$ �ɂȂ�܂��B
     *
     * @param delta_phi ���[���p�␳��
     */
    void mod_euler_phi(const FloatT &delta_phi){
      FloatT delta_phi_h(delta_phi / 2);
      Quaternion<FloatT> delta_q(std::cos(delta_phi_h), std::sin(delta_phi_h), 0, 0);
      q_n2b *= delta_q;
    }

    /**
     * ���[�p�ƃA�W���X�p�𑫂����킹���w�f�B���O(�^���ʊp)��Ԃ��܂��B
     * @return (FloatT) �w�f�B���O
     */
    FloatT heading() const{
      FloatT _heading(euler_psi() + azimuth());
      return _heading > M_PI ? (_heading - (2 * M_PI)) : (_heading < -M_PI ? (_heading + (2 * M_PI)) : _heading);
    }
    
    Vector3<FloatT> omega_e2i() const{return omega_e2i_4n.copy();} /**< @f$ \vec{\omega}_{e/i}^{n} @f$ ��Ԃ��܂��B @return @f$ \vec{\omega}_{e/i}^{n} @f$ */
    Vector3<FloatT> omega_n2e() const{return omega_n2e_4n.copy();} /**< @f$ \vec{\omega}_{n/e}^{n} @f$ ��Ԃ��܂��B @return @f$ \vec{\omega}_{n/e}^{n} @f$ */
    
    /**
     * ���鋗���̈ܐ������̈ړ��ɑ΂��āA���݈ʒu�ɂ�����ܓx�̕ω������߂܂��B
     * ���ŕ\���ƁA
     * @f[
     *    \frac{x}{R_{\mathrm{meridian}} \left( \phi \right)}
     * @f]�B
     * �����ŁA���x�͖����ł�����̂Ƃ��Ă��܂��B
     * 
     * @return (FloatT) �ܓx�̕ω�
     */
    FloatT meter2lat(const FloatT &distance) const{return distance / Earth::R_meridian(phi);}
    /**
     * ���鋗���̌o�������̈ړ��ɑ΂��āA���݈ʒu�ɂ�����o�x�̕ω������߂܂��B
     * ���ŕ\���ƁA
     * @f[
     *    \frac{x}{\beta \left( \phi \right)}
     * @f]�B
     * �����ŁA���x�͖����ł�����̂Ƃ��Ă��܂��B
     * 
     * @return (FloatT) �o�x�̕ω�
     */
    FloatT meter2long(const FloatT &distance) const{return distance / beta();}
    
    /**
     * ���݂�Azimuth�p�����ɁA�w��̈ܓx�A�o�x��@f$ \Tilde{q}_{e}^{n} @f$���v�Z���܂��B
     * 
     * @param latitude �ܓx
     * @param longitude �o�x
     * @return ����
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
     * ���݂̏�ԗʂ����₷���`�ŏo�͂��܂��B
     * 
     * @param out �o�̓X�g���[��
     * @param ins �o�͑Ώ�
     * @return (ostream) �o�̓X�g���[��
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
