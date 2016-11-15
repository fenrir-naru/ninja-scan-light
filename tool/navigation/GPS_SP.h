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

#include <algorithm>
#include <utility>
#include <vector>
#include <map>
#include <exception>

#include <cmath>

#include "param/matrix.h"
#include "GPS.h"

template <class FloatT>
class GPS_SinglePositioning {

  public:
    typedef Matrix<FloatT> matrix_t;

    typedef GPS_SpaceNode<FloatT> space_node_t;
    typedef space_node_t::gps_time_t gps_time_t;

    typedef space_node_t::xyz_t xyz_t;
    typedef space_node_t::llh_t llh_t;
    typedef space_node_t::enu_t enu_t;

    typedef std::vector<std::pair<int, FloatT> > sat_range_t;
    typedef std::vector<std::pair<int, std::pair<FloatT, FloatT> > > sat_range_rate_t;
    typedef std::map<int, std::vector<space_node_t::Satellite::Ephemeris> > sat_ephemeris_list_t;
    typedef std::pair<FloatT, gps_time_t> range_time_t;

  protected:
    space_node_t _space_node;
    sat_ephemeris_list_t sat_eph_list;

    bool _ephemeris_lock;
    bool _valid_iono_param;

  public:
    GPS_SinglePositioning()
        : _space_node(), sat_eph_list(),
          _ephemeris_lock(false), _valid_iono_param(false) {}
    ~GPS_SinglePositioning(){}

    space_node_t &space_node(){return _space_node;}

    bool lock_ephemeris(bool flag){
      return _ephemeris_lock = flag;
    }

    bool valid_iono_param(bool flag){
      return _valid_iono_param = flag;
    }

  protected:
    struct solve_1step_res_t {
      matrix_t G; ///< Design matrix, whose component order corresponds to sat_range_rate.iterator().
      matrix_t W; ///< Weight matrix, whose component order corresponds to sat_range_rate.iterator().
      xyz_t delta_user_position;
      FloatT delta_receiver_error;
    };

    /**
     * Iteration single step of user position calculation
     *
     * @param sat_range_rate PRN, pseudo-range, pseudo-range rate list (rate is not used in this function)
     * @param target_time time at measurement
     * @param user_position temporal solution of user position in meters
     * @param receiver_error temporal solution of receiver clock error in meters
     * @param is_coarse_mode if true, precise correction will be skipped.
     * @return correction values and matrices used for calculation
     */
    solve_1step_res_t solve_1step(
        const sat_range_rate_t &sat_range_rate,
        const gps_time_t &target_time,
        xyz_t &user_position,
        FloatT &receiver_error,
        bool is_coarse_mode = true) throw (std::exception){

      gps_time_t target_time_est(target_time);
      target_time_est -= (receiver_error / space_node_t::light_speed);

      matrix_t G(sat_range_rate.size(), 4);
      matrix_t W(matrix_t::getI(sat_range_rate.size()));;
      matrix_t delta_r(sat_range_rate.size(), 1);

      llh_t usr_llh(user_position.llh());

      unsigned i(0);
      for(sat_range_rate_t::const_iterator it(sat_range_rate.begin());
          it != sat_range_rate.end();
          ++it, ++i){

        FloatT range(it->second.first);
        space_node_t::Satellite &sat(
            const_cast<space_node_t *>(&_space_node)->satellite(it->first));

        // Temporal geometry range
        delta_r(i, 0) = range - receiver_error;

        // Clock error correction
        delta_r(i, 0) += sat.clock_error(target_time_est, delta_r(i, 0)) * space_node_t::light_speed;

        // Calculate satellite position
        xyz_t sat_position(sat.whereis(target_time_est, delta_r(i, 0)));
        FloatT range_est(user_position.dist(sat_position));

        // Calculate residual
        delta_r(i, 0) -= range_est;

        // Setup design matrix
        G(i, 0) = -(sat_position.x() - user_position.x()) / range_est;
        G(i, 1) = -(sat_position.y() - user_position.y()) / range_est;
        G(i, 2) = -(sat_position.z() - user_position.z()) / range_est;
        G(i, 3) = 1.0;

        // Perform more correction
        if(!is_coarse_mode){
          enu_t relative_pos(enu_t::relative(sat_position, user_position));
          // Ionospheric
          FloatT iono_delta_r(_space_node.iono_correction(relative_pos, usr_llh, target_time_est));
          delta_r(i, 0) += iono_delta_r;

          // Tropospheric
          FloatT tropo_delta_r(_space_node.tropo_correction(relative_pos, usr_llh));
          delta_r(i, 0) += tropo_delta_r;

          // Setup weight
          if(delta_r(i, 0) > 30.0){
            // If residual is too big, exclude it by decreasing its weight.
            W(i, i) = 1E-8;
          }else{
            // elevation weight based on "GPS実用プログラミング"
            W(i, i) *= pow2(sin(relative_pos.elevation())/0.8);
            if(W(i, i) < 1E-3){W(i, i) = 1E-3;}
          }
        }
      }

      try{
        // Least square
        matrix_t delta_x(
            (G.transpose() * W * G).inverse() * G.transpose() * W * delta_r);

        solve_1step_res_t res;
        {
          res.G = G;
          res.W = W;
          res.delta_user_position = xyz_t(delta_x.partial(3, 1, 0, 0));
          res.delta_receiver_error = delta_x(3, 0);
        }

        user_position += res.delta_user_position;
        receiver_error += res.delta_receiver_error;

        return res;
      }catch(std::exception &e){throw e;}
    }

  protected:
    struct ephemeris_comparator {
      const gps_time_t &_t;
      ephemeris_comparator(const gps_time_t &t) : _t(t) {}
      bool operator()(
          const space_node_t::Satellite::Ephemeris &eph1,
          const space_node_t::Satellite::Ephemeris &eph2){
        return std::fabs(eph1.period_from_time_of_clock(_t)) < std::fabs(eph2.period_from_time_of_clock(_t));
      }
    };

  public:
    struct user_pvt_t {
      enum {
        ERROR_NO = 0,
        ERROR_INVALID_IONO_PARAMS,
        ERROR_INSUFFICIENT_SATELLITES,
        ERROR_POSITION,
        ERROR_VELOCITY,
      } error_code;
      llh_t user_position_llh;
      FloatT receiver_error;
      enu_t user_velocity_enu;
      FloatT receiver_error_rate;

      matrix_t design_matrix_G, weight_matrix_W;
      FloatT gdop, pdop, hdop, vdop, tdop;

      user_pvt_t()
          : error_code(ERROR_NO),
            user_position_llh(), receiver_error(0),
            user_velocity_enu(), receiver_error_rate(0),
            design_matrix_G(), weight_matrix_W() {}
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
     */
    user_pvt_t solve_user_pvt(
        const sat_range_rate_t &sat_range_rate,
        const gps_time_t &target_time,
        const xyz_t &user_position_init,
        const FloatT &receiver_error_init,
        const bool &good_init = false,
        const bool &with_velocity = true) const {

      user_pvt_t res;

      if(!_valid_iono_param){
        res.error_code = user_pvt_t::ERROR_INVALID_IONO_PARAMS;
        return res;
      }

      sat_range_rate_t available_sat_range_rate;

      for(sat_range_rate_t::const_iterator it(sat_range_rate.begin());
          it != sat_range_rate.end();
          ++it){

        int prn(it->first);

        // If no previous ephemeris, or possibility of newer one, then update.
        bool available(false);
        bool search_ephemeris(true);
        if(_space_node.has_satellite(prn)){
          available = true;
          if(!_space_node.satellite(prn).ephemeris().maybe_newer_one_avilable(target_time)){
            search_ephemeris = false;
          }
        }

        // Check ephemeris
        while(search_ephemeris){

          // Ephemeris available?
          if(sat_eph_list.find(prn) == sat_eph_list.end()){break;}

          // Search ephemeris
          _space_node.satellite(prn).ephemeris()
              = *std::min_element(sat_eph_list[prn].begin(), sat_eph_list[prn].end(),
                  ephemeris_comparator(target_time));

          available = true;
          break;
        }

        // Skip when ephemeris unavailable
        if(!available){continue;}

        // Reconfigure pseudo-range
        available_sat_range_rate.push_back(*it);
      }

      if(available_sat_range_rate.size() < 4){
        res.error_code = user_pvt_t::ERROR_INSUFFICIENT_SATELLITES;
        return res;
      }

      xyz_t user_position(user_position_init);
      FloatT receiver_error(receiver_error_init);

      // Navigation calculation
      try{
        // If initialization is not appropriate, more iteration will be performed.
        for(int i(good_init ? 0 : -2); i < 10; i++){
          solve_1step_res_t res_1step(
              solve_1step(
                available_sat_range_rate,
                target_time,
                user_position, receiver_error,
                i <= 0));

          if(res_1step.delta_user_position.dist() <= 1E-6){
            res.design_matrix_G = res_1step.G;
            res.weight_matrix_W = res_1step.W;

            matrix_t C((res_1step.G.transpose() * res_1step.G).inverse());

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
        for(sat_range_rate_t::const_iterator it(available_sat_range_rate.begin());
            it != available_sat_range_rate.end();
            ++it, ++i){

          int prn(it->first);

          // Calculate satellite velocity
          xyz_t sat_vel(const_cast<space_node_t *>(&_space_node)->satellite(prn)
              .velocity(target_time - receiver_error, it->second.first));

          // Update range rate by subtracting LOS satellite velocity with design matrix G
          range_rate(i, 0) = it->second.second
              + res.design_matrix_G(i, 0) * sat_vel.x()
              + res.design_matrix_G(i, 1) * sat_vel.y()
              + res.design_matrix_G(i, 2) * sat_vel.z();
        }

        try{
          matrix_t sol((res.design_matrix_G.transpose() * res.weight_matrix_W * res.design_matrix_G).inverse()
              * res.design_matrix_G.transpose() * res.weight_matrix_W * range_rate);
          xyz_t usr_vel(sol(0, 0), sol(1, 0), sol(2, 0));

          res.user_velocity_enu = enu_t::relative_rel(usr_vel, res.user_position_llh);
          res.receiver_error_rate = sol(3, 0);
        }catch(std::exception &e){
          res.error_code = user_pvt_t::ERROR_VELOCITY;
          return res;
        }
      }

      return res;
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
        const bool &good_init = false) const {

      sat_range_rate_t sat_range_rate;
      for(sat_range_t::const_iterator it(sat_range.begin()); it != sat_range.end(); ++it){
        sat_range_rate.push_back(
            sat_range_t::value_type(it->first,
                sat_range_t::value_type::second_type(it->second, 0)));
      }
      return solve_user_pvt(sat_range_rate, target_time, user_position_init, receiver_error_init, good_init, false);
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

    /**
     * Add new ephemeris
     *
     * @param eph ephemeris
     */
    void register_epehemris(
        const space_node_t::Satellite::Ephemeris &eph){

      if(_ephemeris_lock){return;}

      // sort in accordance with PRN, time
      if(sat_eph_list[eph.svid].empty()){
        sat_eph_list[eph.svid].push_back(eph);
      }else{
        for(sat_eph_list_t::mapped_type::reverse_iterator it(sat_eph_list[eph.svid].rbegin());
            it != sat_eph_list[eph.svid].rend();
            ++it){
          if(gps_time_t(eph.WN, eph.t_oc) >= gps_time_t(it->WN, it->t_oc)){
            if(eph.t_oc != it->t_oc){
              sat_eph_list[eph.svid].insert(it.base(), eph);
            }else{
              (*it) = eph;
            }
            break;
          }
        }
      }
    }
};
