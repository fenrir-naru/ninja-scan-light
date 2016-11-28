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
    typedef Matrix<FloatT> matrix_t;

    typedef GPS_SpaceNode<FloatT> space_node_t;
    typedef typename space_node_t::gps_time_t gps_time_t;
    typedef typename space_node_t::Satellite satellite_t;

    typedef typename space_node_t::xyz_t xyz_t;
    typedef typename space_node_t::llh_t llh_t;
    typedef typename space_node_t::enu_t enu_t;

    typedef std::pair<FloatT, gps_time_t> range_time_t;

    struct sat_range_t : public std::vector<std::pair<int, FloatT> > {
        typedef std::vector<std::pair<int, FloatT> > super_t;
        sat_range_t() : super_t() {}
        void push_back(const int &sat, const FloatT &range){
          push_back(super_t::value_type(sat, range));
        }
    };
    struct sat_range_rate_t : public std::vector<std::pair<int, std::pair<FloatT, FloatT> > > {
        typedef std::vector<std::pair<int, std::pair<FloatT, FloatT> > > super_t;
        sat_range_rate_t() : super_t() {}
        void push_back(const int &sat, const FloatT &range, const FloatT &rate){
          push_back(super_t::value_type(sat, super_t::value_type::second_type(range, rate)));
        }
        sat_range_rate_t(const sat_range_t &sat_range) : super_t() {
          for(typename sat_range_t::const_iterator it(sat_range.begin()); it != sat_range.end(); ++it){
            push_back(it->first, it->second, 0);
          }
        }
    };

  protected:
    const space_node_t &_space_node;

  public:
    GPS_SinglePositioning(const space_node_t &sn)
        : _space_node(sn) {}

    ~GPS_SinglePositioning(){}

    const space_node_t &space_node(){return _space_node;}

  protected:
    struct geometric_matrices_t {
      matrix_t G; ///< Design matrix, whose component order corresponds to sat_range_rate.iterator().
      matrix_t W; ///< Weight matrix, whose component order corresponds to sat_range_rate.iterator().
      matrix_t delta_r; ///< Residual matrix, whose component order corresponds to sat_range_rate.iterator().
      geometric_matrices_t(const unsigned int &size)
          : G(size, 4), W(size, size), delta_r(size, 1) {}
    };

    /**
     * Get geometric matrices in accordance with current status
     *
     * @param sat_range_rate PRN, pseudo-range, pseudo-range rate list (rate is not used in this function)
     * @param target_time time at measurement
     * @param user_position (temporal solution of) user position in meters
     * @param receiver_error (temporal solution of) receiver clock error in meters
     * @param mat matrices to be stored, already initialized with appropriate size
     * @param is_coarse_mode if true, precise correction will be skipped.
     */
    void get_geometric_matrices(
        const sat_range_rate_t &sat_range_rate,
        const gps_time_t &target_time,
        const xyz_t &user_position,
        const FloatT &receiver_error,
        geometric_matrices_t &mat,
        const bool &is_coarse_mode = false) const {

      gps_time_t target_time_est(
          target_time - (receiver_error / space_node_t::light_speed));

      llh_t usr_llh(user_position.llh());

      unsigned i(0);
      for(typename sat_range_rate_t::const_iterator it(sat_range_rate.begin());
          it != sat_range_rate.end();
          ++it, ++i){

        FloatT range(it->second.first);
        const satellite_t &sat(_space_node.satellite(it->first));

        // Temporal geometry range
        mat.delta_r(i, 0) = range - receiver_error;

        // Clock error correction
        mat.delta_r(i, 0) += sat.clock_error(target_time_est, mat.delta_r(i, 0)) * space_node_t::light_speed;

        // Calculate satellite position
        xyz_t sat_position(sat.position(target_time_est, mat.delta_r(i, 0)));
        FloatT range_est(user_position.dist(sat_position));

        // Calculate residual
        mat.delta_r(i, 0) -= range_est;

        // Setup design matrix
        mat.G(i, 0) = -(sat_position.x() - user_position.x()) / range_est;
        mat.G(i, 1) = -(sat_position.y() - user_position.y()) / range_est;
        mat.G(i, 2) = -(sat_position.z() - user_position.z()) / range_est;
        mat.G(i, 3) = 1;

        if(is_coarse_mode){

          mat.W(i, i) = 1;
        }else{ // Perform more correction

          enu_t relative_pos(enu_t::relative(sat_position, user_position));
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
            mat.W(i, i) = pow2(sin(relative_pos.elevation())/0.8);
            if(mat.W(i, i) < 1E-3){mat.W(i, i) = 1E-3;}
          }
        }
      }
    }

    struct user_pvt_t {
      enum {
        ERROR_NO = 0,
        ERROR_IONO_PARAMS_INVALID,
        ERROR_INSUFFICIENT_SATELLITES,
        ERROR_POSITION,
        ERROR_VELOCITY,
      } error_code;
      llh_t user_position_llh;
      FloatT receiver_error;
      enu_t user_velocity_enu;
      FloatT receiver_error_rate;
      FloatT gdop, pdop, hdop, vdop, tdop;

      user_pvt_t()
          : error_code(ERROR_NO),
            user_position_llh(), receiver_error(0),
            user_velocity_enu(), receiver_error_rate(0) {}
    };

    /**
     * Calculate User position/velocity with hint
     *
     * @param sat_range_rate PRN, pseudo-range, pseudo-range rate list
     * @param target_time time at measurement
     * @param user_position_init initial solution of user position in meters
     * @param receiver_error_init initial solution of receiver clock error in meters
     * @param good_init if true, initial position and clock error are goodly guessed.
     * @param with_velocity if true, perform velocity estimation.
     * @return calculation results and matrices used for calculation
     * @see update_ephemeris(), register_ephemeris
     */
    user_pvt_t solve_user_pvt(
        const sat_range_rate_t &sat_range_rate,
        const gps_time_t &target_time,
        const xyz_t &user_position_init,
        const FloatT &receiver_error_init,
        const bool &good_init = true,
        const bool &with_velocity = true) const {

      user_pvt_t res;

      if(!_space_node.is_valid_iono_utc()){
        res.error_code = user_pvt_t::ERROR_IONO_PARAMS_INVALID;
        return res;
      }

      sat_range_rate_t available_sat_range_rate;

      for(typename sat_range_rate_t::const_iterator it(sat_range_rate.begin());
          it != sat_range_rate.end();
          ++it){

        int prn(it->first);
        if(_space_node.has_satellite(prn)
            && _space_node.satellite(prn).ephemeris().is_valid(target_time)){

          // Select satellite only when its ephemeris is available and valid.
          available_sat_range_rate.push_back(*it);
        }
      }

      if(available_sat_range_rate.size() < 4){
        res.error_code = user_pvt_t::ERROR_INSUFFICIENT_SATELLITES;
        return res;
      }

      xyz_t user_position(user_position_init);
      FloatT receiver_error(receiver_error_init);

      geometric_matrices_t mat(sat_range_rate.size());
      matrix_t &G(mat.G), &W(mat.W);

      // Navigation calculation
      try{
        // If initialization is not appropriate, more iteration will be performed.
        for(int i(good_init ? 0 : -2); i < 10; i++){
          get_geometric_matrices(
              available_sat_range_rate,
              target_time,
              user_position, receiver_error,
              mat,
              i <= 0);

          // Least square
          matrix_t delta_x((G.transpose() * W * G).inverse() * G.transpose() * W * mat.delta_r);

          xyz_t delta_user_position(delta_x.partial(3, 1, 0, 0));
          FloatT delta_receiver_error(delta_x(3, 0));

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

      if(with_velocity){ // Calculate velocity
        matrix_t range_rate(available_sat_range_rate.size(), 1);
        int i(0);
        for(typename sat_range_rate_t::const_iterator it(available_sat_range_rate.begin());
            it != available_sat_range_rate.end();
            ++it, ++i){

          int prn(it->first);

          // Calculate satellite velocity
          xyz_t sat_vel(_space_node.satellite(prn).velocity(
              target_time - receiver_error / space_node_t::light_speed, it->second.first));

          // Update range rate by subtracting LOS satellite velocity with design matrix G
          range_rate(i, 0) = it->second.second
              + G(i, 0) * sat_vel.x()
              + G(i, 1) * sat_vel.y()
              + G(i, 2) * sat_vel.z();
        }

        try{
          // Least square
          matrix_t sol((G.transpose() * W * G).inverse() * G.transpose() * W * range_rate);

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
     * @param sat_range_rate PRN, pseudo-range, pseudo-range rate list
     * @param target_time time at measurement
     * @return calculation results and matrices used for calculation
     */
    user_pvt_t solve_user_pvt(
        const sat_range_rate_t &sat_range_rate,
        const gps_time_t &target_time) const {
      return solve_user_pvt(sat_range_rate, target_time, xyz_t(), 0, false);
    }

    /**
     * Calculate User position with hint
     *
     * @param sat_range PRN, pseudo-range list
     * @param target_time time at measurement
     * @param user_position_init initial solution of user position in meters
     * @param receiver_error_init initial solution of receiver clock error in meters
     * @param good_init if true, initial position and clock error are goodly guessed.
     * @return calculation results and matrices used for calculation
     */
    user_pvt_t solve_user_position(
        const sat_range_t &sat_range,
        const gps_time_t &target_time,
        const xyz_t &user_position_init,
        const FloatT &receiver_error_init,
        const bool &good_init = true) const {

      return solve_user_pvt(
          sat_range_rate_t(sat_range), target_time,
          user_position_init, receiver_error_init,
          good_init, false);
    }

    /**
     * Calculate User position without hint
     *
     * @param sat_range PRN and pseudo range
     * @param target_time time at measurement
     * @return calculation results and matrices used for calculation
     */
    user_pvt_t solve_user_position(
            const sat_range_t &sat_range,
            const gps_time_t &target_time) const {
      return solve_user_position(sat_range, target_time, xyz_t(), 0, false);
    }
};

#endif /* __GPS_SP_H__ */
