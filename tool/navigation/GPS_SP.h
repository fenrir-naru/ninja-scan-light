/**
 * @file GPS single positioning solver
 *
 */

/*
 * Copyright (c) 2016, M.Naruoka (fenrir)
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

#ifndef __GPS_SP_H__
#define __GPS_SP_H__

#include <utility>
#include <vector>
#include <exception>

#include <cmath>

#include "param/matrix.h"
#include "GPS.h"

template <class FloatT>
class GPS_SinglePositioning {

  public:
    typedef FloatT float_t;
    typedef Matrix<float_t> matrix_t;

    typedef GPS_SpaceNode<float_t> space_node_t;
    typedef typename space_node_t::gps_time_t gps_time_t;
    typedef typename space_node_t::Satellite satellite_t;

    typedef typename space_node_t::xyz_t xyz_t;
    typedef typename space_node_t::llh_t llh_t;
    typedef typename space_node_t::enu_t enu_t;

    typedef std::vector<std::pair<int, float_t> > prn_obs_t;

  protected:
    typedef std::vector<std::pair<
        typename space_node_t::satellites_t::const_iterator, float_t> > sat_obs_t;
    const space_node_t &_space_node;

  public:
    GPS_SinglePositioning(const space_node_t &sn)
        : _space_node(sn) {}

    ~GPS_SinglePositioning(){}

    const space_node_t &space_node(){return _space_node;}

  protected:
    struct geometric_matrices_t {
      matrix_t G; ///< Design matrix, whose component order corresponds to sat_range.iterator().
      matrix_t W; ///< Weight matrix, whose component order corresponds to sat_range.iterator().
      matrix_t delta_r; ///< Residual matrix, whose component order corresponds to sat_range.iterator().
      geometric_matrices_t(const unsigned int &size)
          : G(size, 4), W(size, size), delta_r(size, 1) {}
    };

    /**
     * Get geometric matrices in accordance with current status
     *
     * @param sat_range satellite, pseudo-range list
     * @param target_time time at measurement
     * @param user_position (temporal solution of) user position in meters
     * @param receiver_error (temporal solution of) receiver clock error in meters
     * @param mat matrices to be stored, already initialized with appropriate size
     * @param is_coarse_mode if true, precise correction will be skipped.
     */
    void get_geometric_matrices(
        const sat_obs_t &sat_range,
        const gps_time_t &target_time,
        const xyz_t &user_position,
        const float_t &receiver_error,
        geometric_matrices_t &mat,
        const bool &is_coarse_mode = false) const {

      gps_time_t target_time_est(
          target_time - (receiver_error / space_node_t::light_speed));

      llh_t usr_llh(user_position.llh());

      unsigned i(0);
      for(typename sat_obs_t::const_iterator it(sat_range.begin());
          it != sat_range.end();
          ++it, ++i){

        const satellite_t &sat(it->first->second);
        float_t range(it->second);

        // Temporal geometry range
        mat.delta_r(i, 0) = range - receiver_error;

        // Clock error correction
        mat.delta_r(i, 0) += sat.clock_error(target_time_est, mat.delta_r(i, 0)) * space_node_t::light_speed;

        // Calculate satellite position
        xyz_t sat_pos(sat.position(target_time_est, mat.delta_r(i, 0)));
        float_t range_est(user_position.dist(sat_pos));

        // Calculate residual
        mat.delta_r(i, 0) -= range_est;

        // Setup design matrix
        xyz_t &user_pos(const_cast<xyz_t &>(user_position));
        mat.G(i, 0) = -(sat_pos.x() - user_pos.x()) / range_est;
        mat.G(i, 1) = -(sat_pos.y() - user_pos.y()) / range_est;
        mat.G(i, 2) = -(sat_pos.z() - user_pos.z()) / range_est;
        mat.G(i, 3) = 1;

        if(is_coarse_mode){

          mat.W(i, i) = 1;
        }else{ // Perform more correction

          enu_t relative_pos(enu_t::relative(sat_pos, user_pos));

          // Ionospheric
          mat.delta_r(i, 0) += _space_node.iono_correction(relative_pos, usr_llh, target_time_est);

          // Tropospheric
          mat.delta_r(i, 0) += _space_node.tropo_correction(relative_pos, usr_llh);

          // Setup weight
          if(mat.delta_r(i, 0) > 30.0){
            // If residual is too big, exclude it by decreasing its weight.
            mat.W(i, i) = 1E-8;
          }else{
            // elevation weight based on "GPS実用プログラミング"
            mat.W(i, i) = std::pow(sin(relative_pos.elevation())/0.8, 2);
            if(mat.W(i, i) < 1E-3){mat.W(i, i) = 1E-3;}
          }
        }
      }
    }

  public:
    struct user_pvt_t {
      enum {
        ERROR_NO = 0,
        ERROR_IONO_PARAMS_INVALID,
        ERROR_INSUFFICIENT_SATELLITES,
        ERROR_POSITION,
        ERROR_VELOCITY,
      } error_code;
      llh_t user_position_llh;
      float_t receiver_error;
      enu_t user_velocity_enu;
      float_t receiver_error_rate;
      float_t gdop, pdop, hdop, vdop, tdop;

      user_pvt_t()
          : error_code(ERROR_NO),
            user_position_llh(), receiver_error(0),
            user_velocity_enu(), receiver_error_rate(0) {}
    };

    /**
     * Calculate User position/velocity with hint
     *
     * @param prn_range PRN, pseudo-range list
     * @param prn_rate PRN, pseudo-range rate list
     * @param target_time time at measurement
     * @param user_position_init initial solution of user position in meters
     * @param receiver_error_init initial solution of receiver clock error in meters
     * @param good_init if true, initial position and clock error are goodly guessed.
     * @param with_velocity if true, perform velocity estimation.
     * @return calculation results and matrices used for calculation
     * @see update_ephemeris(), register_ephemeris
     */
    user_pvt_t solve_user_pvt(
        const prn_obs_t &prn_range,
        const prn_obs_t &prn_rate,
        const gps_time_t &target_time,
        const xyz_t &user_position_init,
        const float_t &receiver_error_init,
        const bool &good_init = true,
        const bool &with_velocity = true) const {

      user_pvt_t res;

      if(!_space_node.is_valid_iono_utc()){
        res.error_code = user_pvt_t::ERROR_IONO_PARAMS_INVALID;
        return res;
      }

      sat_obs_t available_sat_range;

      const typename space_node_t::satellites_t &sats(_space_node.satellites());
      for(typename prn_obs_t::const_iterator it(prn_range.begin());
          it != prn_range.end();
          ++it){

        int prn(it->first);
        const typename space_node_t::satellites_t::const_iterator it_sat(sats.find(prn));
        if((it_sat != sats.end()) && it_sat->second.ephemeris().is_valid(target_time)){

          // Select satellite only when its ephemeris is available and valid.
          available_sat_range.push_back(typename sat_obs_t::value_type(it_sat, it->second));
        }
      }

      if(available_sat_range.size() < 4){
        res.error_code = user_pvt_t::ERROR_INSUFFICIENT_SATELLITES;
        return res;
      }

      xyz_t user_position(user_position_init);
      float_t receiver_error(receiver_error_init);

      geometric_matrices_t mat(available_sat_range.size());
      matrix_t &G(mat.G), &W(mat.W);

      // Navigation calculation
      try{
        // If initialization is not appropriate, more iteration will be performed.
        for(int i(good_init ? 0 : -2); i < 10; i++){
          get_geometric_matrices(
              available_sat_range,
              target_time,
              user_position, receiver_error,
              mat,
              i <= 0);

          // Least square
          matrix_t delta_x((G.transpose() * W * G).inverse() * G.transpose() * W * mat.delta_r);

          if(false){ // debug
            for(typename sat_obs_t::const_iterator it(available_sat_range.begin());
                it != available_sat_range.end();
                ++it){
              std::cerr << "PRN:" << it->first->first << " => "
                  << it->second
                  << " @ Ephemeris: t_oc => "
                  << it->first->second.ephemeris().WN << "w "
                  << it->first->second.ephemeris().t_oc << " +/- "
                  << (it->first->second.ephemeris().fit_interval / 2) << std::endl;
            }
            std::cerr << "G:" << G << std::endl;
            std::cerr << "W:" << W << std::endl;
            std::cerr << "delta_r:" << mat.delta_r << std::endl;
          }

          xyz_t delta_user_position(delta_x.partial(3, 1, 0, 0));
          float_t delta_receiver_error(delta_x(3, 0));

          user_position += delta_user_position;
          receiver_error += delta_receiver_error;

          if(delta_user_position.dist() <= 1E-6){
            matrix_t C((G.transpose() * G).inverse());

            // Calculate DOP
            res.gdop = std::sqrt(C.trace());
            res.pdop = std::sqrt(C.partial(3, 3, 0, 0).trace());
            res.hdop = std::sqrt(C.partial(2, 2, 0, 0).trace());
            res.vdop = std::sqrt(C(2, 2));
            res.tdop = std::sqrt(C(3, 3));

            break;
          }
        }
      }catch(std::exception &e){
        res.error_code = user_pvt_t::ERROR_POSITION;
        return res;
      }

      res.user_position_llh = user_position.llh();
      res.receiver_error = receiver_error;

      if((!prn_range.empty()) && with_velocity){ // Calculate velocity
        typedef std::vector<std::pair<int, int> > index_table_t;
        index_table_t index_table;

        // check correspondence between range and rate.
        int i(0);
        for(typename sat_obs_t::const_iterator it(available_sat_range.begin());
            it != available_sat_range.end();
            ++it, ++i){
          int j(0);
          for(typename prn_obs_t::const_iterator it2(prn_rate.begin());
              it2 != prn_rate.end();
              ++it2, ++j){
            if(it->first->first == it2->first){
              index_table.push_back(index_table_t::value_type(i, j));
              break;
            }
          }
        }

        gps_time_t target_time_est(
              target_time - (receiver_error / space_node_t::light_speed));
        i = 0;
        geometric_matrices_t mat_rate(index_table.size());
        for(typename index_table_t::const_iterator it(index_table.begin());
            it != index_table.end();
            ++it, ++i){

          int i_range(it->first), i_rate(it->second);
          int prn(prn_rate[i_rate].first);
          float_t range(available_sat_range[i_range].second - receiver_error);

          // Calculate satellite velocity
          const satellite_t &sat(available_sat_range[i_range].first->second);
          xyz_t sat_vel(sat.velocity(
              target_time_est,
              range));

          // copy design matrix
          for(int j(0); j < 4; ++j){
            mat_rate.G(i, j) = G(i_range, j);
          }
          mat_rate.W(i, i) = W(i_range, i_range);

          // Update range rate by subtracting LOS satellite velocity with design matrix G, and clock rate error
          mat_rate.delta_r(i, 0) = prn_rate[i_rate].second
              + mat_rate.G(i, 0) * sat_vel.x()
              + mat_rate.G(i, 1) * sat_vel.y()
              + mat_rate.G(i, 2) * sat_vel.z()
              + (sat.clock_error_dot(target_time_est, range) * space_node_t::light_speed);
        }

        try{
          // Least square
          matrix_t sol((mat_rate.G.transpose() * mat_rate.W * mat_rate.G).inverse()
              * mat_rate.G.transpose() * mat_rate.W * mat_rate.delta_r);

          res.user_velocity_enu = enu_t::relative_rel(xyz_t(sol.partial(3, 1, 0, 0)), res.user_position_llh);
          res.receiver_error_rate = sol(3, 0);
        }catch(std::exception &e){
          res.error_code = user_pvt_t::ERROR_VELOCITY;
          return res;
        }
      }

      return res;
    }

    /**
     * Calculate User position/velocity without hint
     *
     * @param prn_range PRN, pseudo-range list
     * @param prn_rate PRN, pseudo-range rate list
     * @param target_time time at measurement
     * @return calculation results and matrices used for calculation
     */
    user_pvt_t solve_user_pvt(
        const prn_obs_t &prn_range,
        const prn_obs_t &prn_rate,
        const gps_time_t &target_time) const {
      return solve_user_pvt(prn_range, prn_rate, target_time, xyz_t(), 0, false);
    }

    /**
     * Calculate User position with hint
     *
     * @param prn_range PRN, pseudo-range list
     * @param target_time time at measurement
     * @param user_position_init initial solution of user position in meters
     * @param receiver_error_init initial solution of receiver clock error in meters
     * @param good_init if true, initial position and clock error are goodly guessed.
     * @return calculation results and matrices used for calculation
     */
    user_pvt_t solve_user_position(
        const prn_obs_t &prn_range,
        const gps_time_t &target_time,
        const xyz_t &user_position_init,
        const float_t &receiver_error_init,
        const bool &good_init = true) const {

      return solve_user_pvt(
          prn_range, prn_obs_t(), target_time,
          user_position_init, receiver_error_init,
          good_init, false);
    }

    /**
     * Calculate User position without hint
     *
     * @param prn_range PRN and pseudo range
     * @param target_time time at measurement
     * @return calculation results and matrices used for calculation
     */
    user_pvt_t solve_user_position(
            const prn_obs_t &prn_range,
            const gps_time_t &target_time) const {
      return solve_user_position(prn_range, target_time, xyz_t(), 0, false);
    }
};

#endif /* __GPS_SP_H__ */
