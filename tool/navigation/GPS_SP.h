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

    typedef std::vector<pair<int, FloatT> > sat_range_t;
    typedef std::map<int, vector<space_node_t::Satellite::Ephemeris> > nav_data_t;
    typedef std::pair<FloatT, gps_time_t> range_time_t;
    typedef std::map<int, range_time_t> sat_range_time_t;

  protected:
    space_node_t _space_node;
    nav_data_t nav_data;

    bool _ephemeris_lock;
    bool _valid_iono_param;

    sat_range_time_t latest_sat_range_time;

  public:
    GPS_SinglePositioning()
        : _space_node(), nav_data(),
          _ephemeris_lock(false), _valid_iono_param(false),
          latest_sat_range_time() {}
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
      matrix_t G; ///< Design matrix, whose component order corresponds to sat_range.iterator().
      matrix_t W; ///< Weight matrix, whose component order corresponds to sat_range.iterator().
      xyz_t delta_user_position;
      FloatT delta_receiver_error;
    };

    /**
     * Iteration single step of user position calculation
     *
     * @param sat_range PRN and pseudo range
     * @param target_time time at measurement
     * @param user_position temporal solution of user position in meters
     * @param receiver_error temporal solution of receiver clock error in meters
     * @param is_coarse_mode if true, precise correction will be skipped.
     * @return correction values and matrices used for calculation
     */
    solve_1step_res_t solve_1step(
        const sat_range_t &sat_range,
        const gps_time_t &target_time,
        xyz_t &user_position,
        FloatT &receiver_error,
        bool is_coarse_mode = true) throw (exception){

      gps_time_t target_time_est(target_time);
      target_time_est -= (receiver_error / space_node_t::light_speed);

      matrix_t G(sat_range.size(), 4);
      matrix_t W(matrix_t::getI(sat_range.size()));;
      matrix_t delta_r(sat_range.size(), 1);

      llh_t usr_llh(user_position.llh());

      unsigned i(0);
      for(sat_range_t::const_iterator it(sat_range.begin());
          it != sat_range.end();
          ++it, ++i){

        FloatT range(it->second);
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
      }catch(exception e){throw e;}
    }

  protected:
    struct ephemeris_comparator {
      const gps_time_t &_t;
      ephemeris_comparator(const gps_time_t &t) : _t(t) {}
      bool operator()(
          const space_node_t::Satellite::Ephemeris &eph1,
          const space_node_t::Satellite::Ephemeris &eph2){
        return _abs(eph1.period_from_time_of_clock(_t))
            < _abs(eph2.period_from_time_of_clock(_t));
      }
    };

  public:
    struct solve_user_position_res_t {
      enum {
        ERROR_NO = 0,
        ERROR_INVALID_IONO_PARAMS,
        ERROR_INSUFFICIENT_SATELLITES,
      } error_code;
      llh_t user_position;
      FloatT receiver_error;
      matrix_t design_matrix_G, weight_matrix_W;
      FloatT gdop, pdop, hdop, vdop, tdop;
      solve_user_position_res_t() : error_code(ERROR_NO) {}
    };

    /**
     * Calculate User position with hint
     *
     * @param sat_range PRN and pseudo range
     * @param target_time time at measurement
     * @param user_position_init initial solution of user position in meters
     * @param receiver_error_init initial solution of receiver clock error in meters
     * @param good_init if true, initial position and clock error are goodly guessed.
     * @return calculation results and matrices used for calculation
     */
    solve_user_position_res_t solve_user_position(
        const sat_range_t &sat_range,
        const gps_time_t &target_time,
        const xyz_t &user_position_init,
        const FloatT &receiver_error_init,
        const bool &good_init = false) throw (exception) {

      solve_user_position_res_t res;

      if(!_valid_iono_param){
        res.error_code = solve_user_position_res_t::ERROR_INVALID_IONO_PARAMS;
        return res;
      }

      sat_range_t available_sat_range;

      for(sat_range_t::const_iterator it(sat_range.begin());
          it != sat_range.end();
          ++it){

        int prn(it->first);
        FloatT pseudo_range(it->second);

        // If no previous ephemeris, or possibility of newer one, then update.
        bool available(false);
        bool search_ephemeris(true);
        if(_space_node.has_satellite(it->first)){
          available = true;
          if(!_space_node.satellite(prn).ephemeris().maybe_newer_one_avilable(target_time)){
            search_ephemeris = false;
          }
        }

        // Check ephemeris
        while(search_ephemeris){

          // Ephemeris available?
          if(nav_data.find(prn) == nav_data.end()){break;}

          // Search ephemeris
          _space_node.satellite(prn).ephemeris()
              = *std::min_element(nav_data[prn].begin(), nav_data[prn].end(),
                  ephemeris_comparator(target_time));

          available = true;
          break;
        }

        // Skip when ephemeris unavailable
        if(!available){continue;}

        // Reconfigure pseudo range
        available_sat_range.push_back(sat_range_t::value_type(prn, pseudo_range));
      }

      if(available_sat_range.size() < 4){
        res.error_code = solve_user_position_res_t::ERROR_INSUFFICIENT_SATELLITES;
        return res;
      }

      // Navigation calculation
      try{
        xyz_t user_position(user_position_init);
        FloatT receiver_error(receiver_error_init);

        matrix_t C;

        // If initialization is not appropriate, more iteration will be performed.
        for(int i(good_init ? 0 : -2); i < 10; i++){
          solve_1step_res_t res_1step(
              solve_1step(
                available_sat_range,
                target_time,
                user_position, receiver_error,
                i <= 0));

          if(res_1step.delta_user_position.dist() <= 1E-6){
            res.design_matrix_G = res_1step.G;
            res.weight_matrix_W = res_1step.W;
            C = (res_1step.G.transpose() * res_1step.G).inverse();
            break;
          }
        }

        res.user_position_llh = user_position.llh();
        res.receiver_error = receiver_error;

        // Calculate DOP
        res.gdop = std::sqrt(C.trace());
        res.pdop = std::sqrt(C.partial(3, 3, 0, 0).trace());
        res.hdop = std::sqrt(C.partial(2, 2, 0, 0).trace());
        res.vdop = std::sqrt(C(2, 2));
        res.tdop = std::sqrt(C(3, 3));
      }catch(exception e){throw e;}

      return res;
    }

    /**
     * Calculate User position without hint
     *
     * @param sat_range PRN and pseudo range
     * @param target_time time at measurement
     * @return calculation results and matrices used for calculation
     */
    solve_user_position_res_t solve_user_position(
            const sat_range_t &sat_range,
            const gps_time_t &target_time) throw (exception) {
      return solve_user_position(sat_range, target_time, xyz_t(), 0, false);
    }

    struct solve_user_velocity_return_t {
      xyz_t user_velocity;
      FloatT dot_receiver_error;
    };

    solve_user_velocity_return_t solve_user_velocity(
        const sat_range_t &sat_range,
        const gps_time_t &target_time,
        const FloatT &receiver_error,
        const matrix_t &design_matrix_G,
        const matrix_t &weight_matrix_W) throw (exception) {

      matrix_t delta_range(sat_range.size(), 1);
      matrix_t G(design_matrix_G.rows(), design_matrix_G.columns());
      matrix_t W(weight_matrix_W.rows(), weight_matrix_W.columns());

      gps_time_t target_time_est(target_time - receiver_error / space_node_t::light_speed);

      // 擬似距離の変化率を求める
      int i1(0), i2(0);
      for(sat_range_t::const_iterator it(sat_range.begin());
          it != sat_range.end();
          ++it, ++i2){

        FloatT geo_range_est(it->second - receiver_error);

        // 前回の擬似距離情報があるか調べる
        sat_range_time_t::iterator it2(latest_sat_range_time.find(it->first));
        if(it2 != latest_sat_range_time.end()){
          // 擬似距離差 / 時間差 で正規化
          delta_range(i1, 0)
              = (geo_range_est - it2->second.first)
                  / it2->second.second.interval(target_time_est);

          for(int j(0); j < design_matrix_G.columns(); j++){
            G(i1, j) = const_cast<matrix_t &>(design_matrix_G)(i2, j);
          }
          W(i1, i1) = const_cast<matrix_t &>(weight_matrix_W)(i2, i2);

          // さらに衛星の速度ベクトルを算出(xyz_t)
          space_node_t::Satellite &sat(
              const_cast<space_node_t *>(&_space_node)->satellite(it->first));

          xyz_t sat_vel(sat.velocity(target_time_est, it->second));
          // デザイン行列にある視線ベクトルを利用して、
          // 擬似距離変化率から衛星の速度ベクトルに由来する部分を除去
          delta_range(i1, 0) += (G(i1, 0) * sat_vel.x()
              + G(i1, 1) * sat_vel.y()
              + G(i1, 2) * sat_vel.z());

          i1++;
        }

        // 最新の擬似距離情報を記録
        latest_sat_range_time[it->first] = range_time_t(geo_range_est, target_time_est);
      }
      G = G.partial(i1, design_matrix_G.columns(), 0, 0);
      W = W.partial(i1, i1, 0, 0);
      delta_range = delta_range.partial(i1, delta_range.columns(), 0, 0);

      // デザイン行列から速度を求める
      try{
        solve_user_velocity_return_t res;

        matrix_t sol((G.transpose() * W * G).inverse() * G.transpose() * W * delta_range);
        res.user_velocity.x() = sol(0, 0);
        res.user_velocity.y() = sol(1, 0);
        res.user_velocity.z() = sol(2, 0);
        res.dot_receiver_error = sol(3, 0);

        return res;
      }catch(exception e){throw e;}
    }

    /**
     * Add new ephemeris
     *
     * @param eph ephemeris
     */
    void register_epehemris(
        const space_node_t::Satellite::Ephemeris &eph){

      // sort in accordance with PRN, time
      if(nav_data[eph.svid].empty()){
        nav_data[eph.svid].push_back(eph);
      }else{
        for(nav_data_t::mapped_type::reverse_iterator it(nav_data[eph.svid].rbegin());
            it != nav_data[eph.svid].rend();
            ++it){
          if(gps_time_t(eph.WN, eph.t_oc) >= gps_time_t(it->WN, it->t_oc)){
            if(!_ephemeris_lock){
              if(eph.t_oc != it->t_oc){
                nav_data[eph.svid].insert(it.base(), eph);
              }else{
                (*it) = eph;
              }
            }
            break;
          }
        }
      }
    }
};
