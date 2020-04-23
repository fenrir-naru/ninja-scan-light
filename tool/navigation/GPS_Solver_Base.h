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
#include <map>
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

  static prn_obs_t difference(
      const prn_obs_t &operand, const prn_obs_t &argument,
      const FloatT &scaling = FloatT(1)) {
    prn_obs_t res;
    for(typename prn_obs_t::const_iterator it(operand.begin()); it != operand.end(); ++it){
      for(typename prn_obs_t::const_iterator it2(argument.begin()); it2 != argument.end(); ++it2){
        if(it->first != it2->first){continue;}
        res.push_back(std::make_pair(it->first, (it->second - it2->second) * scaling));
        break;
      }
    }
    return res;
  }

  struct measurement_items_t {
    enum {
      L1_PSEUDORANGE,
      L1_DOPPLER,
      L1_CARRIER_PHASE,
      L1_RANGE_RATE,
      L1_PSEUDORANGE_SIGMA, // standard deviation(sigma)
      L1_DOPPLER_SIGMA,
      L1_CARRIER_PHASE_SIGMA,
      L1_RANGE_RATE_SIGMA,
      MEASUREMENT_ITEMS_PREDEFINED,
    };
  };
  typedef std::map<prn_t, std::map<int, float_t> > measurement_t;

  struct measurement_item_set_t {
    struct {
      int i, i_sigma;
    } pseudorange, doppler, carrier_phase, range_rate;
  };
  static const measurement_item_set_t L1CA;

  struct measurement_util_t {
    static prn_obs_t gather(
        const measurement_t &measurement,
        const typename measurement_t::mapped_type::key_type &key,
        const FloatT &scaling = FloatT(1)){
      prn_obs_t res;
      for(typename measurement_t::const_iterator it(measurement.begin());
          it != measurement.end(); ++it){
        typename measurement_t::mapped_type::const_iterator it2(it->second.find(key));
        if(it2 == it->second.end()){continue;}
        res.push_back(std::make_pair(it->first, it2->second * scaling));
      }
      return res;
    }
    static void merge(
        measurement_t &measurement,
        const prn_obs_t &new_item,
        const typename measurement_t::mapped_type::key_type &key) {
      for(typename prn_obs_t::const_iterator it(new_item.begin());
          it != new_item.end(); ++it){
        measurement[it->first].insert(std::make_pair(key, it->second));
      }
    }
  };

  /**
   * Find value corresponding to key from key-value map
   * of measurement_t::mapped_type
   * @param values key-value map
   * @param key key
   * @param buf buffer into which found value is stored
   * @return (float_t *) When value is found, pointer of buf will be returned.
   * Otherwise, NULL is returned.
   */
  static const float_t *find_value(
      const typename measurement_t::mapped_type &values,
      const typename measurement_t::mapped_type::key_type &key,
      float_t &buf) {
    typename measurement_t::mapped_type::const_iterator it;
    if((it = values.find(key)) != values.end()){
      return &(buf = it->second);
    }
    return NULL;
  }

  struct range_error_t {
    enum {
#define make_entry(name) \
  name, \
  MASK_ ## name = (1 << name), \
  DUMMY_ ## name = name
      make_entry(RECEIVER_CLOCK),
      make_entry(SATELLITE_CLOCK),
      make_entry(IONOSPHERIC),
      make_entry(TROPOSPHERIC),
#undef make_entry
      NUM_OF_ERRORS,
    };
    int unknown_flag;
    float_t value[NUM_OF_ERRORS];
    static const range_error_t not_corrected;
  };

  // TODO These range and rate functions will be overridden in subclass to support multi-frequency
  /**
   * Extract range information from measurement per satellite
   * @param values measurement[prn]
   * @param buf buffer into which range is stored
   * @param error optional argument in which error components of range will be returned
   * @return If valid range information is found, the pointer of buf will be returned; otherwise NULL
   */
  virtual const float_t *range(
      const typename measurement_t::mapped_type &values, float_t &buf,
      range_error_t *error = NULL) const {
    if(error){
      *error = range_error_t::not_corrected;
    }
    return find_value(values, measurement_items_t::L1_PSEUDORANGE, buf);
  }

  virtual const float_t *range_sigma(
      const typename measurement_t::mapped_type &values, float_t &buf) const {
    return find_value(values, measurement_items_t::L1_PSEUDORANGE_SIGMA, buf);
  }

  virtual const float_t *rate(
      const typename measurement_t::mapped_type &values, float_t &buf) const {
    const float_t *res;
    if(res = find_value(values, measurement_items_t::L1_RANGE_RATE, buf)){

    }else if(res = find_value(values, measurement_items_t::L1_DOPPLER, buf)){
      // Fall back to doppler
      buf *= -space_node_t::L1_WaveLength();
    }
    return res;
  }

  virtual const float_t *rate_sigma(
      const typename measurement_t::mapped_type &values, float_t &buf) const {
    const float_t *res;
    if(res = find_value(values, measurement_items_t::L1_RANGE_RATE_SIGMA, buf)){

    }else if(res = find_value(values, measurement_items_t::L1_DOPPLER_SIGMA, buf)){
      // Fall back to doppler
      buf *= space_node_t::L1_WaveLength();
    }
    return res;
  }

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
   * @param measurement measurement (per satellite) containing pseudo range
   * @param receiver_error (temporal solution of) receiver clock error in meter
   * @param time_arrival time when signal arrive at receiver
   * @param usr_pos (temporal solution of) user position
   * @param usr_vel (temporal solution of) user velocity
   * @return (relative_property_t) relative information
   */
  virtual relative_property_t relative_property(
      const prn_t &prn,
      const typename measurement_t::mapped_type &measurement,
      const float_t &receiver_error,
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
    static const satellite_mask_t satellite_mask;

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
   * @param measurement PRN, pseudo-range, and pseudo-range rate information
   * @param receiver_time receiver time at measurement
   * @param user_position_init initial solution of user position in XYZ meters and LLH
   * @param receiver_error_init initial solution of receiver clock error in meters
   * @param good_init if true, initial position and clock error are goodly guessed.
   * @param with_velocity if true, perform velocity estimation.
   * @return calculation results and matrices used for calculation
   * @see update_ephemeris(), register_ephemeris
   */
  virtual user_pvt_t solve_user_pvt(
      const measurement_t &measurement,
      const gps_time_t &receiver_time,
      const pos_t &user_position_init,
      const float_t &receiver_error_init,
      const bool &good_init = true,
      const bool &with_velocity = true) const = 0;

  /**
   * Calculate User position/velocity with hint
   *
   * @param measurement PRN, pseudo-range, and pseudo-range rate information
   * @param receiver_time receiver time at measurement
   * @param user_position_init_xyz initial solution of user position in meters
   * @param receiver_error_init initial solution of receiver clock error in meters
   * @param good_init if true, initial position and clock error are goodly guessed.
   * @param with_velocity if true, perform velocity estimation.
   * @return calculation results and matrices used for calculation
   * @see update_ephemeris(), register_ephemeris
   */
  user_pvt_t solve_user_pvt(
      const measurement_t &measurement,
      const gps_time_t &receiver_time,
      const xyz_t &user_position_init_xyz,
      const float_t &receiver_error_init,
      const bool &good_init = true,
      const bool &with_velocity = true) const {
    pos_t user_position_init = {user_position_init_xyz, user_position_init_xyz.llh()};
    return solve_user_pvt(
        measurement, receiver_time,
        user_position_init, receiver_error_init,
        good_init, with_velocity);
  }

  /**
   * Calculate User position/velocity without hint
   *
   * @param measurement PRN, pseudo-range, and pseudo-range rate information
   * @param receiver_time receiver time at measurement
   * @return calculation results and matrices used for calculation
   */
  user_pvt_t solve_user_pvt(
      const measurement_t &measurement,
      const gps_time_t &receiver_time) const {
    return solve_user_pvt(measurement, receiver_time, xyz_t(), 0, false);
  }

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
      const bool &with_velocity = true) const {
    measurement_t measurement;
    measurement_util_t::merge(measurement, prn_range, measurement_items_t::L1_PSEUDORANGE);
    measurement_util_t::merge(measurement, prn_rate, measurement_items_t::L1_RANGE_RATE);
    return solve_user_pvt(
        measurement, receiver_time,
        user_position_init, receiver_error_init,
        good_init, with_velocity);
  }

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
const typename GPS_Solver_Base<FloatT>::measurement_item_set_t
    GPS_Solver_Base<FloatT>::L1CA = {
#define make_entry(key) { \
    GPS_Solver_Base<FloatT>::measurement_items_t::L1_ ## key, \
    GPS_Solver_Base<FloatT>::measurement_items_t::L1_ ## key ## _SIGMA}
      make_entry(PSEUDORANGE),
      make_entry(DOPPLER),
      make_entry(CARRIER_PHASE),
      make_entry(RANGE_RATE),
#undef make_entry
    };

template <class FloatT>
const typename GPS_Solver_Base<FloatT>::range_error_t
    GPS_Solver_Base<FloatT>::range_error_t::not_corrected = {
      MASK_RECEIVER_CLOCK | MASK_SATELLITE_CLOCK | MASK_IONOSPHERIC | MASK_TROPOSPHERIC,
      {0},
    };

template <class FloatT>
const typename GPS_Solver_Base<FloatT>::user_pvt_t::satellite_mask_t
    GPS_Solver_Base<FloatT>::user_pvt_t::satellite_mask;

#endif /* __GPS_SOLVER_BASE_H__ */