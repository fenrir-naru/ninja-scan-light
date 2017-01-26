/*
 *  INS_GPS2.h, header file to perform calculation of integration of INS and GPS.
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

#ifndef __INS_GPS2_H__
#define __INS_GPS2_H__

/** @file
 * @brief INS/GPS(Multiplicative)
 * 
 * �����q�@���u(INS)��GPS�̓�����Loose-coupling�ŋL�q�����t�@�C���B
 * INS�̌덷�v�Z�͐ώZ�^(Multiplicative)�𗘗p���Ă��܂��B
 * 
 * INS/GPS�̓����ɗp����GPS�̊e�f�[�^�^�ɂ��Ă�INS_GPS.h���Q�Ƃ��Ă��������B
 * 
 * @see Filtered_INS2.h
 * @see INS_GPS2.h
 */


#include "Filtered_INS2.h"
#include "algorithm/kalman.h"

#ifndef pow2
#define pow2(x) ((x) * (x))
#else
#define ALREADY_POW2_DEFINED
#endif

/**
 * @brief GPS�̃f�[�^�^(u-blox)
 *
 * u-blox GPS�ł̃f�[�^�^�B3�������ʂ̏ꍇ�B
 *
 * @param FloatT ���Z���x
 */
template <class FloatT>
struct GPS_UBLOX_3D{
  FloatT v_n,          ///< �k�������x
         v_e,          ///< ���������x
         v_d;          ///< ���������x
  FloatT sigma_vel;    ///< ���x�x�N�g���̐�Βl�̐���덷
  FloatT latitude,     ///< �ܓx
         longitude,    ///< �o�x
         height;       ///< ���x
  FloatT sigma_2d,     ///< �����ʏ�̐���덷
         sigma_height; ///< ���x�����̐���덷

  struct sigma_vel_ned_t {
    FloatT n, e, d;
  };
  /**
   * TODO: �v����
   *
   * NED�����ɕ����������x�덷�����߂܂��B
   *
   * @return (sigma_vel_ned_t) NED�����ɕ����������x�덷
   */
  sigma_vel_ned_t sigma_vel_ned() const {
    FloatT v2((v_n * v_n) + (v_e * v_e) + (v_d * v_d));
    sigma_vel_ned_t res;
    FloatT sigma_pos(std::sqrt((sigma_2d * sigma_2d) + (sigma_height * sigma_height)));
    res.n = res.e = sigma_vel * sigma_2d / sigma_pos;
    res.d = sigma_vel * sigma_height / sigma_pos;
    return res;
  }
};

template <class FloatT>
struct CorrectInfo {
  Matrix<FloatT> H;
  Matrix<FloatT> z;
  Matrix<FloatT> R;
  CorrectInfo(
      const Matrix<FloatT> &_H,
      const Matrix<FloatT> &_z,
      const Matrix<FloatT> &_R) : H(_H), z(_z), R(_R) {}
  ~CorrectInfo(){}
  CorrectInfo(const CorrectInfo &another)
      : H(another.H), z(another.z), R(another.R) {}
  CorrectInfo &operator=(const CorrectInfo &another){
    H = another.H;
    z = another.z;
    R = another.R;
    return *this;
  }
};

/**
 * @brief INS/GPS(Multiplicative)
 * 
 * INS/GPS Loose-coupling���J���}���t�B���^�Ŏ��������N���X�B
 * INS�̃V�X�e���덷�������͐ώZ�^�ŕ\������Ă��܂��B
 * 
 * @param FloatT ���Z���x
 * @param Filter �J���}���t�B���^
 * @param BaseFINS ���̍q�@���u�𓝍�����ׂ�INS�g���N���X
 * @see Filtered_INS2
 */
template <
  class FloatT, 
  template <class> class Filter = KalmanFilterUD,
  typename BaseFINS = Filtered_INS2<INS<FloatT>, Filter>
>
class INS_GPS2 : public BaseFINS{
  public:
    INS_GPS2() : BaseFINS(){} /**< �R���X�g���N�^ */
    
    /**
     * �R�s�[�R���X�g���N�^
     * 
     * @param orig �R�s�[��
     * @param deepcopy �f�B�[�v�R�s�[���쐬���邩�ǂ���
     */
    INS_GPS2(const INS_GPS2 &orig, const bool deepcopy = false) :
      BaseFINS(orig, deepcopy){
      
    }
    
    ~INS_GPS2(){}         /**< �f�X�g���N�^ */
    
    using BaseFINS::correct;
    
    /**
     * �ϑ��X�V(Measurement Update)���s���܂��B
     * 
     * @param info �C�����
     */
    void correct(const CorrectInfo<FloatT> &info){
      BaseFINS::correct(info.H, info.z, info.R);
    }
    
    using BaseFINS::P_SIZE;
    using BaseFINS::get;
    
    /**
     * 3D���ʎ��ɂ�����ϑ��X�V(Measurement Update)�p�̏������߂܂��B
     * u-blox�p�B
     * 
     * @param gps GPS�o�̓f�[�^
     * @return (CorrectInfo) �ϑ��X�V�p�̃f�[�^
     */
    CorrectInfo<FloatT> correct_info(const GPS_UBLOX_3D<FloatT> &gps) const {
      using std::cos;
      using std::sin;
      FloatT azimuth(INS<FloatT>::azimuth());
      
      //cout << "__correct__" << endl;
      
      Quaternion<FloatT> q_e2n_gps(INS<FloatT>::e2n(gps.latitude, gps.longitude));
      
      //cout << "__correct__" << endl;
      
      //�ϑ���z
      FloatT z_serialized[8][1];
#define z(i, j) z_serialized[i][j]
      {
        z(0, 0) = get(0) - (gps.v_n * cos(azimuth) + gps.v_e * sin(azimuth));
        z(1, 0) = get(1) - (gps.v_n * -sin(azimuth) + gps.v_e * cos(azimuth));
        z(2, 0) = get(2) - gps.v_d;
        z(3, 0) = get(3) - q_e2n_gps[0];
        z(4, 0) = get(4) - q_e2n_gps[1];
        z(5, 0) = get(5) - q_e2n_gps[2];
        z(6, 0) = get(6) - q_e2n_gps[3];
        z(7, 0) = get(7) - gps.height;
      }
#undef z
#define z_size (sizeof(z_serialized) / sizeof(z_serialized[0]))
      Matrix<FloatT> z(z_size, 1, (FloatT *)z_serialized);
      
      //�s��H�̍쐬
      FloatT H_serialized[z_size][P_SIZE] = {{FloatT(0)}};
#define H(i, j) H_serialized[i][j]
      {
        H(0, 0) = 1;
        H(1, 1) = 1;
        H(2, 2) = 1;
        
        H(3, 3) = -get(4);
        H(3, 4) = -get(5);
        H(3, 5) = -get(6);
        
        H(4, 3) =  get(3);
        H(4, 4) =  get(6);
        H(4, 5) = -get(5);
        
        H(5, 3) = -get(6);
        H(5, 4) =  get(3);
        H(5, 5) =  get(4);
        
        H(6, 3) =  get(5);
        H(6, 4) = -get(4);
        H(6, 5) =  get(3);
        
        H(7, 6) = 1;
      }
#undef H
      Matrix<FloatT> H(z_size, P_SIZE, (FloatT *)H_serialized);
      
      FloatT lat_sigma(INS<FloatT>::meter2lat(gps.sigma_2d));
      FloatT long_sigma(INS<FloatT>::meter2long(gps.sigma_2d));
      
      //�ϑ��l�덷�s��R
      FloatT R_serialized[z_size][z_size] = {{FloatT(0)}};
#define R(i, j) R_serialized[i][j]
      {
        typename GPS_UBLOX_3D<FloatT>::sigma_vel_ned_t sigma_vel_ned(
            gps.sigma_vel_ned());
        R(0, 0) = pow2(sigma_vel_ned.n);
        R(1, 1) = pow2(sigma_vel_ned.e);
        R(2, 2) = pow2(sigma_vel_ned.d);
        
        FloatT s_lambda1 = sin((gps.longitude + azimuth) / 2);
        FloatT s_lambda2 = sin((gps.longitude - azimuth) / 2);
        FloatT c_lambda1 = cos((gps.longitude + azimuth) / 2);
        FloatT c_lambda2 = cos((gps.longitude - azimuth) / 2);
        FloatT s_phi = sin(-gps.latitude / 2);
        FloatT c_phi = cos(-gps.latitude / 2);
        
        R(3, 3) = pow2( c_lambda1 * (s_phi - c_phi) * lat_sigma / 2) + pow2(-s_lambda1 * (c_phi + s_phi) * long_sigma / 2);
        R(4, 4) = pow2( s_lambda2 * (s_phi + c_phi) * lat_sigma / 2) + pow2( c_lambda2 * (c_phi - s_phi) * long_sigma / 2);
        R(5, 5) = pow2(-c_lambda2 * (s_phi + c_phi) * lat_sigma / 2) + pow2( s_lambda2 * (c_phi - s_phi) * long_sigma / 2);
        R(6, 6) = pow2( s_lambda1 * (s_phi - c_phi) * lat_sigma / 2) + pow2( c_lambda1 * (c_phi + s_phi) * long_sigma / 2);
        
        R(7, 7) = pow2(gps.sigma_height);
      }
#undef R
      Matrix<FloatT> R(z_size, z_size, (FloatT *)R_serialized);
#undef z_size
      
      return CorrectInfo<FloatT>(H, z, R);
    }
    
    /**
     * 3D���ʎ��ɂ�����ϑ��X�V(Measurement Update)���s���܂��B
     * u-blox�p�B
     * 
     * @param gps GPS�o�̓f�[�^
     */
    void correct(const GPS_UBLOX_3D<FloatT> &gps){
      correct(correct_info(gps));
    }
    
    /**
     * 3D���ʎ��ɂ�����ϑ��X�V(Measurement Update)�p�̏������߂܂��B
     * u-blox�p�B
     * lever arm effect���l���B
     * 
     * @param gps GPS�o�̓f�[�^
     * @param lever_arm_b lever arm�̑傫��
     * @param omega_b2i_4b �ϑ����̃W���C���̒l
     * @return (CorrectInfo) �ϑ��X�V�p�̃f�[�^
     */
    CorrectInfo<FloatT> correct_info(const GPS_UBLOX_3D<FloatT> &gps, 
        const Vector3<FloatT> &lever_arm_b,
        const Vector3<FloatT> &omega_b2i_4b) const {
                   
      FloatT azimuth(INS<FloatT>::azimuth());
      
      // �ʒu�֌W
      Vector3<FloatT> lever_arm_n(
        ((INS<FloatT>::q_n2b) * lever_arm_b * (INS<FloatT>::q_n2b).conj()).vector());
      Vector3<FloatT> lever_arm_g(
          lever_arm_n[0] * cos(azimuth) - lever_arm_n[1] * sin(azimuth),
          lever_arm_n[0] * sin(azimuth) + lever_arm_n[1] * cos(azimuth),
          lever_arm_n[2]
        );
      FloatT lever_lat(-INS<FloatT>::meter2lat(lever_arm_g[0]));
      FloatT lever_long(-INS<FloatT>::meter2long(lever_arm_g[1]));
      
      FloatT c_phi(cos((-INS<FloatT>::phi - lever_lat) / 2));
      FloatT s_phi(sin((-INS<FloatT>::phi - lever_lat) / 2));
      FloatT c_lambda_p(cos((INS<FloatT>::lambda + azimuth + lever_long) / 2));
      FloatT s_lambda_p(sin((INS<FloatT>::lambda + azimuth + lever_long) / 2));
      FloatT c_lambda_n(cos((INS<FloatT>::lambda - azimuth + lever_long) / 2));
      FloatT s_lambda_n(sin((INS<FloatT>::lambda - azimuth + lever_long) / 2));
      
      Matrix<FloatT> coefficient_pos_phi_lambda(4, 2);
      {
        coefficient_pos_phi_lambda(0, 0) = -c_lambda_p * (-s_phi + c_phi);
        coefficient_pos_phi_lambda(0, 1) = -s_lambda_p * ( c_phi + s_phi);
        coefficient_pos_phi_lambda(1, 0) = -s_lambda_n * (-s_phi - c_phi);
        coefficient_pos_phi_lambda(1, 1) =  c_lambda_n * ( c_phi - s_phi);
        coefficient_pos_phi_lambda(2, 0) =  c_lambda_n * (-s_phi - c_phi);
        coefficient_pos_phi_lambda(2, 1) =  s_lambda_n * ( c_phi - s_phi);
        coefficient_pos_phi_lambda(3, 0) = -s_lambda_p * (-s_phi + c_phi);
        coefficient_pos_phi_lambda(3, 1) =  c_lambda_p * ( c_phi + s_phi);
      }
      coefficient_pos_phi_lambda /= (std::sqrt(2.0) * 2);
      
      FloatT c_alpha(cos(azimuth));
      FloatT s_alpha(sin(azimuth));
      
      Matrix<FloatT> coefficient_pos_lever_g(2, 3);
      {
        coefficient_pos_lever_g(0, 0) = INS<FloatT>::meter2lat(s_alpha * lever_arm_n[2] * -2);
        coefficient_pos_lever_g(0, 1) = INS<FloatT>::meter2lat(c_alpha * lever_arm_n[2] * -2);
        coefficient_pos_lever_g(0, 2) = INS<FloatT>::meter2lat((-c_alpha * lever_arm_n[1] - s_alpha * lever_arm_n[0]) * -2);  
        coefficient_pos_lever_g(1, 0) = INS<FloatT>::meter2long(-c_alpha * lever_arm_n[2] * -2);
        coefficient_pos_lever_g(1, 1) = INS<FloatT>::meter2long(s_alpha * lever_arm_n[2] * -2);
        coefficient_pos_lever_g(1, 2) = INS<FloatT>::meter2long((-s_alpha * lever_arm_n[1] + c_alpha * lever_arm_n[0]) * -2);
      }
      
      Quaternion<FloatT> q_e2n_gps(
          INS<FloatT>::e2n(
              gps.latitude + lever_lat, 
              gps.longitude + lever_long));
      
      // ���x�̏C����
      Vector3<FloatT> omega_b2n_4b(
          omega_b2i_4b - INS<FloatT>::omega_e2i_4n + INS<FloatT>::omega_n2e_4n
        );
      Vector3<FloatT> v_induced(
        ((INS<FloatT>::q_n2b) 
          * (omega_b2n_4b * lever_arm_b) 
          * (INS<FloatT>::q_n2b).conj()).vector());
      
      //Matrix<FloatT> coefficient_vel_omega(omega_b2n_4n.skewMatrix());
      //Matrix<FloatT> coefficient_vel_lever(-lever_arm_n.skewMatrix());
      
      //�ϑ���z
      FloatT z_serialized[8][1];
#define z(i, j) z_serialized[i][j]
      {
        z(0, 0) = get(0) 
            - ((gps.v_n * cos(azimuth) + gps.v_e * sin(azimuth)) - v_induced[0]);
        z(1, 0) = get(1) 
            - ((gps.v_n * -sin(azimuth) + gps.v_e * cos(azimuth)) - v_induced[1]);
        z(2, 0) = get(2) 
            - (gps.v_d - v_induced[2]);
        z(3, 0) = get(3) - q_e2n_gps[0];
        z(4, 0) = get(4) - q_e2n_gps[1];
        z(5, 0) = get(5) - q_e2n_gps[2];
        z(6, 0) = get(6) - q_e2n_gps[3];
        z(7, 0) = get(7) - (gps.height - lever_arm_g[2]);
      }
#undef z
#define z_size (sizeof(z_serialized) / sizeof(z_serialized[0]))
      Matrix<FloatT> z(z_size, 1, (FloatT *)z_serialized);
      
      //�s��H�̍쐬
      Matrix<FloatT> H(z_size, P_SIZE);
      {
        H(0, 0) = 1;
        H(1, 1) = 1;
        H(2, 2) = 1;
        
        H(3, 3) = -get(4);
        H(3, 4) = -get(5);
        H(3, 5) = -get(6);
        
        H(4, 3) =  get(3);
        H(4, 4) =  get(6);
        H(4, 5) = -get(5);
        
        H(5, 3) = -get(6);
        H(5, 4) =  get(3);
        H(5, 5) =  get(4);
        
        H(6, 3) =  get(5);
        H(6, 4) = -get(4);
        H(6, 5) =  get(3);
        
        H(7, 6) = 1;
        
        // lever arm effect in position.
        H.pivotMerge(3, 7, -(coefficient_pos_phi_lambda * coefficient_pos_lever_g));
        H(7, 7) = lever_arm_n[1] * 2;
        H(7, 8) = -lever_arm_n[0] * 2;
        
        // lever arm effect in velocity.
        H.pivotMerge(0, 7, - v_induced.skewMatrix() * 2);
      }
      
      FloatT lat_sigma = BaseFINS::meter2lat(gps.sigma_2d);
      FloatT long_sigma = BaseFINS::meter2long(gps.sigma_2d);
      
      //�ϑ��l�덷�s��R
      FloatT R_serialized[z_size][z_size] = {{FloatT(0)}};
#define R(i, j) R_serialized[i][j]
      {
        R(0, 0) = pow2(gps.sigma_vel);
        R(1, 1) = pow2(gps.sigma_vel);
        R(2, 2) = pow2(gps.sigma_vel);
        
        FloatT s_lambda1 = sin((gps.longitude + azimuth) / 2);
        FloatT s_lambda2 = sin((gps.longitude - azimuth) / 2);
        FloatT c_lambda1 = cos((gps.longitude + azimuth) / 2);
        FloatT c_lambda2 = cos((gps.longitude - azimuth) / 2);
        FloatT s_phi = sin(-gps.latitude / 2);
        FloatT c_phi = cos(-gps.latitude / 2);
        
        R(3, 3) = pow2( c_lambda1 * (s_phi - c_phi) * lat_sigma / 2) + pow2(-s_lambda1 * (c_phi + s_phi) * long_sigma / 2);
        R(4, 4) = pow2( s_lambda2 * (s_phi + c_phi) * lat_sigma / 2) + pow2( c_lambda2 * (c_phi - s_phi) * long_sigma / 2);
        R(5, 5) = pow2(-c_lambda2 * (s_phi + c_phi) * lat_sigma / 2) + pow2( s_lambda2 * (c_phi - s_phi) * long_sigma / 2);
        R(6, 6) = pow2( s_lambda1 * (s_phi - c_phi) * lat_sigma / 2) + pow2( c_lambda1 * (c_phi + s_phi) * long_sigma / 2);
        
        R(7, 7) = pow2(gps.sigma_height);
        
        // ���x���ɂ���Ƀ�omega_{b/i}^{n}�̉e����������
      }
#undef R
      Matrix<FloatT> R(z_size, z_size, (FloatT *)R_serialized);
#undef z_size
      
      //for(int i = 0; i < z.rows(); i++) cout << z(i, 0) << '\t';
      //for(int i = 0; i < R.rows(); i++) cout << R(i, i) << '\t';
      //cout << endl;
      
      /*{
        R(0, 0) = R(1, 1) = 1.;
        R(2, 2) = R(3, 3) = R(4, 4) = R(5, 5) = 1E-12;
        R(6, 6) = pow2(); 
      }*/
      //for(int i = 0; i < R.rows(); i++){R(i, i) *= 2;}
      
      return CorrectInfo<FloatT>(H, z, R);
    }
    
    /**
     * 3D���ʎ��ɂ�����ϑ��X�V(Measurement Update)���s���܂��B
     * u-blox�p�B
     * lever arm effect���l���B
     * 
     * @param gps GPS�o�̓f�[�^
     * @param lever_arm_b lever arm�̑傫��
     * @param omega_b2i_4b �ϑ����̃W���C���̒l
     */
    void correct(const GPS_UBLOX_3D<FloatT> &gps, 
        const Vector3<FloatT> &lever_arm_b,
        const Vector3<FloatT> &omega_b2i_4b){
      correct(correct_info(gps, lever_arm_b, omega_b2i_4b));
    }

    /**
     * ���[�p�C�����t�B���^��ʂ��čs���܂��B
     *
     * @param delta_psi ���݃��[�p�Ƃ̍��� [rad]
     * @param sigma2_delta_psi delta_psi�̊m���炵��(���U) [rad^2]
     */
    void correct_yaw(const FloatT &delta_psi, const FloatT &sigma2_delta_psi){

      //�ϑ���z
      FloatT z_serialized[1][1] = {{-delta_psi}};
#define z_size (sizeof(z_serialized) / sizeof(z_serialized[0]))
      Matrix<FloatT> z(z_size, 1, (FloatT *)z_serialized);

      //�s��H�̍쐬
      FloatT H_serialized[z_size][P_SIZE] = {{FloatT(0)}};
#define H(i, j) H_serialized[i][j]
      {
        H(0, 9) = 2; // u_{3} {}_{n}^{b}
      }
#undef H
      Matrix<FloatT> H(z_size, P_SIZE, (FloatT *)H_serialized);

      //�ϑ��l�덷�s��R
      FloatT R_serialized[z_size][z_size] = {{sigma2_delta_psi}};
      Matrix<FloatT> R(z_size, z_size, (FloatT *)R_serialized);
#undef z_size

      // �C���ʂ̌v�Z
      Matrix<FloatT> K(BaseFINS::m_filter.correct(H, R)); //�J���}���Q�C��
      Matrix<FloatT> x_hat(K * z);
      //before_correct_INS(H, R, K, z, x_hat); // ���[�����␳������ꃂ�[�h�Ȃ��߁A����͌Ăяo���Ȃ�
      BaseFINS::correct_INS(x_hat);
    }
};

#ifdef ALREADY_POW2_DEFINED
#undef ALREADY_POW2_DEFINED
#else
#undef pow2
#endif

#endif /* __INS_GPS2_H__ */
