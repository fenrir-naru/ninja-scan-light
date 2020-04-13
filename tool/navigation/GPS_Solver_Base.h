/**
 * @file GPS solver base
 */
/*
 * Copyright (c) 2020, M.Naruoka (fenrir)
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

#ifndef __GPS_SOLVER_BASE_H__
#define __GPS_SOLVER_BASE_H__

#include <vector>
#include <utility>

#include "GPS.h"

template <class FloatT>
struct GPS_Solver_Base {
  virtual ~GPS_Solver_Base() {}

  typedef FloatT float_t;

  typedef int prn_t;

  typedef GPS_SpaceNode<float_t> space_node_t;
  typedef typename space_node_t::gps_time_t gps_time_t;

  typedef typename space_node_t::xyz_t xyz_t;
  typedef typename space_node_t::llh_t llh_t;
  typedef typename space_node_t::enu_t enu_t;

  struct pos_t {
    xyz_t xyz;
    llh_t llh;
  };

  typedef std::vector<std::pair<prn_t, float_t> > prn_obs_t;

  struct relative_property_t {
    float_t weight; ///< How useful this information is. only positive value activates the other values.
    float_t range_corrected; ///< corrected range just including delay, and excluding receiver/satellite error
    float_t range_residual; ///< residual range
    float_t rate_relative_neg; /// relative rate
    float_t los_neg[3]; ///< line of site vector from satellite to user
  };

  /**
   * Calculate relative range and rate information to a satellite
   *
   * @param prn satellite number
   * @param range "corrected" pseudo range subtracted by (temporal solution of) receiver clock error in meter
   * @param time_arrival time when signal arrive at receiver
   * @param usr_pos (temporal solution of) user position
   * @param usr_vel (temporal solution of) user velocity
   * @return (relative_property_t) relative information
   */
  virtual relative_property_t relative_property(
          const prn_t &prn,
          const float_t &range,
          const gps_time_t &time_arrival,
          const pos_t &usr_pos,
          const xyz_t &usr_vel) const = 0;

  struct user_pvt_t {
    enum {
      ERROR_NO = 0,
      ERROR_UNSOLVED,
      ERROR_INVALID_IONO_MODEL,
      ERROR_INSUFFICIENT_SATELLITES,
      ERROR_POSITION_LS,
      ERROR_POSITION_NOT_CONVERGED,
      ERROR_DOP,
      ERROR_VELOCITY_SKIPPED,
      ERROR_VELOCITY_INSUFFICIENT_SATELLITES,
      ERROR_VELOCITY_LS,
    } error_code;
    gps_time_t receiver_time;
    pos_t user_position;
    float_t receiver_error;
    enu_t user_velocity_enu;
    float_t receiver_error_rate;
    float_t gdop, pdop, hdop, vdop, tdop;
    unsigned int used_satellites;
    unsigned long long used_satellite_mask; ///< bit pattern(use=1, otherwise=0), PRN 1(LSB) to 32 for GPS

    struct satellite_mask_t {
      static const int PRN_MAX = 32;
      typename space_node_t::u32_t pattern[PRN_MAX + 1]; // +1 for padding(PRN=0)
      satellite_mask_t() {
        for(int prn(1), shift(0); prn <= PRN_MAX; prn++, shift++){
          pattern[prn] = ((typename space_node_t::u32_t)1) << shift;
        }
      }
      const typename space_node_t::u32_t &operator[](const int &prn) const {
        return pattern[prn];
      }
    };
    static satellite_mask_t satellite_mask;

    user_pvt_t()
        : error_code(ERROR_UNSOLVED),
          receiver_time(),
          user_position(), receiver_error(0),
          user_velocity_enu(), receiver_error_rate(0) {}

    bool position_solved() const {
      switch(error_code){
        case ERROR_NO:
        case ERROR_VELOCITY_SKIPPED:
        case ERROR_VELOCITY_INSUFFICIENT_SATELLITES:
        case ERROR_VELOCITY_LS:
          return true;
      }
      return false;
    }
    bool velocity_solved() const {
      return error_code == ERROR_NO;
    }
  };

  /**
   * Calculate User position/velocity with hint
   *
   * @param prn_range PRN, pseudo-range list
   * @param prn_rate PRN, pseudo-range rate list
   * @param receiver_time receiver time at measurement
   * @param user_position_init initial solution of user position in XYZ meters and LLH
   * @param receiver_error_init initial solution of receiver clock error in meters
   * @param good_init if true, initial position and clock error are goodly guessed.
   * @param with_velocity if true, perform velocity estimation.
   * @return calculation results and matrices used for calculation
   * @see update_ephemeris(), register_ephemeris
   */
  virtual user_pvt_t solve_user_pvt(
      const prn_obs_t &prn_range,
      const prn_obs_t &prn_rate,
      const gps_time_t &receiver_time,
      const pos_t &user_position_init,
      const float_t &receiver_error_init,
      const bool &good_init = true,
      const bool &with_velocity = true) const = 0;

  /**
   * Calculate User position/velocity with hint
   *
   * @param prn_range PRN, pseudo-range list
   * @param prn_rate PRN, pseudo-range rate list
   * @param receiver_time receiver time at measurement
   * @param user_position_init_xyz initial solution of user position in meters
   * @param receiver_error_init initial solution of receiver clock error in meters
   * @param good_init if true, initial position and clock error are goodly guessed.
   * @param with_velocity if true, perform velocity estimation.
   * @return calculation results and matrices used for calculation
   * @see update_ephemeris(), register_ephemeris
   */
  user_pvt_t solve_user_pvt(
      const prn_obs_t &prn_range,
      const prn_obs_t &prn_rate,
      const gps_time_t &receiver_time,
      const xyz_t &user_position_init_xyz,
      const float_t &receiver_error_init,
      const bool &good_init = true,
      const bool &with_velocity = true) const {
    pos_t user_position_init = {user_position_init_xyz, user_position_init_xyz.llh()};
    return solve_user_pvt(
        prn_range, prn_rate, receiver_time,
        user_position_init, receiver_error_init,
        good_init, with_velocity);
  }

  /**
   * Calculate User position/velocity without hint
   *
   * @param prn_range PRN, pseudo-range list
   * @param prn_rate PRN, pseudo-range rate list
   * @param receiver_time receiver time at measurement
   * @return calculation results and matrices used for calculation
   */
  user_pvt_t solve_user_pvt(
      const prn_obs_t &prn_range,
      const prn_obs_t &prn_rate,
      const gps_time_t &receiver_time) const {
    return solve_user_pvt(prn_range, prn_rate, receiver_time, xyz_t(), 0, false);
  }

  /**
   * Calculate User position with hint
   *
   * @param prn_range PRN, pseudo-range list
   * @param receiver_time receiver time at measurement
   * @param user_position_init initial solution of user position in meters
   * @param receiver_error_init initial solution of receiver clock error in meters
   * @param good_init if true, initial position and clock error are goodly guessed.
   * @return calculation results and matrices used for calculation
   */
  user_pvt_t solve_user_position(
      const prn_obs_t &prn_range,
      const gps_time_t &receiver_time,
      const xyz_t &user_position_init,
      const float_t &receiver_error_init,
      const bool &good_init = true) const {

    return solve_user_pvt(
        prn_range, prn_obs_t(), receiver_time,
        user_position_init, receiver_error_init,
        good_init, false);
  }

  /**
   * Calculate User position without hint
   *
   * @param prn_range PRN and pseudo range
   * @param receiver_time receiver time at measurement
   * @return calculation results and matrices used for calculation
   */
  user_pvt_t solve_user_position(
          const prn_obs_t &prn_range,
          const gps_time_t &receiver_time) const {
    return solve_user_position(prn_range, receiver_time, xyz_t(), 0, false);
  }
};

template <class FloatT>
typename GPS_Solver_Base<FloatT>::user_pvt_t::satellite_mask_t
    GPS_Solver_Base<FloatT>::user_pvt_t::satellite_mask
      = typename GPS_Solver_Base<FloatT>::user_pvt_t::satellite_mask_t();

#endif /* __GPS_SOLVER_BASE_H__ */
