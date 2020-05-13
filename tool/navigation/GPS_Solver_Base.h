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

#include <cmath>
#include <cstring>
#include <cstdlib>
#include <climits>

#include "param/matrix.h"
#include "GPS.h"

template <class FloatT>
struct GPS_Solver_Base {
  virtual ~GPS_Solver_Base() {}

  typedef FloatT float_t;
  typedef Matrix<float_t> matrix_t;

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

  /**
   * Select appropriate solver, this is provision for GNSS extension
   * @param prn satellite number
   * @return self, however, it will be overridden by a subclass
   */
  virtual const GPS_Solver_Base<FloatT> &select(const prn_t &prn) const {
    return *this;
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
   * This function will be overridden by a subclass to provide appropriate implementation
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
      const xyz_t &usr_vel) const {
    static const relative_property_t invalid = {0};
    return invalid;
  }

  template <int MAX_SIZE, class ContainerT = unsigned char>
  struct bit_array_t { ///< alternation of std::bitset
    static const int bits_per_addr = (int)sizeof(ContainerT) * CHAR_BIT;
    ContainerT buf[(MAX_SIZE + bits_per_addr - 1) / bits_per_addr];
    void set(const bool &new_bit = false) {
      std::memset(buf, (new_bit ? (~0) : 0), sizeof(buf));
    }
    void reset() {set(false);}
    void clear() {reset();}
    bool operator[](const int &idx) const {
      if((idx < 0) || (idx >= MAX_SIZE)){return false;}
      if(MAX_SIZE > bits_per_addr){
        std::div_t qr(std::div(idx, bits_per_addr));
        ContainerT mask((ContainerT)1 << qr.rem);
        return buf[qr.quot] & mask;
      }else{
        return buf[0] & ((ContainerT)1 << idx);
      }
    }
    void set(const int &idx, const bool &bit = true) {
      if((idx < 0) || (idx >= MAX_SIZE)){return;}
      if(MAX_SIZE > bits_per_addr){
        std::div_t qr(std::div(idx, bits_per_addr));
        ContainerT mask((ContainerT)1 << qr.rem);
        bit ? (buf[qr.quot] |= mask) : (buf[qr.quot] &= ~mask);
      }else{
        ContainerT mask((ContainerT)1 << idx);
        bit ? (buf[0] |= mask) : (buf[0] &= ~mask);
      }
    }
    void reset(const int &idx) {
      set(idx, false);
    }
    /**
     * Generate bit pattern with arbitrary start and end indices
     */
    unsigned int pattern(const int &idx_lsb, int idx_msb) const {
      if((idx_msb < idx_lsb) || (idx_lsb < 0) || (idx_msb >= MAX_SIZE)){
        return 0;
      }
      static const int res_bits((int)sizeof(unsigned int) * CHAR_BIT);
      if((idx_msb - idx_lsb) >= res_bits){
        // check output boundary; if overrun, msb will be truncated.
        idx_msb = idx_lsb + res_bits - 1;
      }

      std::div_t qr_lsb(std::div(idx_lsb, bits_per_addr)),
          qr_msb(std::div(idx_msb, bits_per_addr));
      if(res_bits >= bits_per_addr){
        unsigned int res(buf[qr_msb.quot] & ((qr_msb.rem == bits_per_addr - 1)
            ? ~((ContainerT)0)
            : (((ContainerT)1 << (qr_msb.rem + 1)) - 1))); // MSB byte
        if(qr_msb.quot > qr_lsb.quot){
          for(int i(qr_msb.quot - 1); i > qr_lsb.quot; --i){ // Fill intermediate
            res <<= bits_per_addr;
            res |= buf[i];
          }
          res <<= (bits_per_addr - qr_lsb.rem);
          res |= (buf[qr_lsb.quot] >> qr_lsb.rem); // Last byte
        }else{
          res >>= qr_lsb.rem;
        }
        return res;
      }else{
        ContainerT res(buf[qr_msb.quot] & ((qr_msb.rem == bits_per_addr - 1)
            ? ~((ContainerT)0)
            : (((ContainerT)1 << (qr_msb.rem + 1)) - 1))); // MSB byte
        if(qr_msb.quot > qr_lsb.quot){
          res <<= (bits_per_addr - qr_lsb.rem);
          res |= (buf[qr_lsb.quot] >> qr_lsb.rem); // Last byte
        }else{
          res >>= qr_lsb.rem;
        }
        return (unsigned int)res;
      }
    }
  };

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
    typedef bit_array_t<0x400> satellite_mask_t;
    satellite_mask_t used_satellite_mask; ///< bit pattern(use=1, otherwise=0), PRN 1(LSB) to 32 for GPS

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

    void update_DOP(const matrix_t &C){
      // Calculate DOP
      gdop = std::sqrt(C.trace());
      pdop = std::sqrt(C.partial(3, 3, 0, 0).trace());
      hdop = std::sqrt(C.partial(2, 2, 0, 0).trace());
      vdop = std::sqrt(C(2, 2));
      tdop = std::sqrt(C(3, 3));
    }
  };

protected:
  struct geometric_matrices_t {
    matrix_t G; ///< Design matrix
    matrix_t W; ///< Weight (diagonal) matrix
    matrix_t delta_r; ///< Residual vector
    geometric_matrices_t(const unsigned int &size)
        : G(size, 4), W(size, size), delta_r(size, 1) {
      for(unsigned int i(0); i < size; ++i){
        G(i, 3) = 1;
      }
    }

    typename matrix_t::partial_t Gp(const unsigned int &size) const {
      return G.partial(size, 4, 0, 0);
    }
    typename matrix_t::partial_t Wp(const unsigned int &size) const {
      return W.partial(size, size, 0, 0);
    }
    typename matrix_t::partial_t delta_rp(const unsigned int &size) const {
      return delta_r.partial(size, 1, 0, 0);
    }

    matrix_t C() const {
      return (G.transpose() * G).inverse();
    }
    matrix_t C(const unsigned int &size) const {
      typename matrix_t::partial_t Gp_(Gp(size));
      return (Gp_.transpose() * Gp_).inverse();
    }

    matrix_t least_square() const {
      matrix_t Gt_W(G.transpose() * W);
      return (Gt_W * G).inverse() * Gt_W * delta_r;
    }
    matrix_t least_square(const unsigned int &size) const {
      if(size >= G.rows()){return least_square();}
      typename matrix_t::partial_t Gp_(Gp(size));
      matrix_t Gpt_Wp(Gp_.transpose() * Wp(size));
      return (Gpt_Wp * Gp_).inverse() * Gpt_Wp * delta_rp(size);
    }

    void copy_G_W_row(const geometric_matrices_t &src,
        const unsigned int &i_src, const unsigned int &i_dst){
      for(unsigned int j(0); j < 4; ++j){
        G(i_dst, j) = src.G(i_src, j);
      }
      W(i_dst, i_dst) = src.W(i_src, i_src);
    }
  };

public:
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
      const bool &with_velocity = true) const {

    // Reference implementation (to be hidden by optimized one in sub class)

    user_pvt_t res;
    res.receiver_time = receiver_time;

    // 1. Position calculation

    res.user_position = user_position_init;
    res.receiver_error = receiver_error_init;

    gps_time_t time_arrival(
        receiver_time - (res.receiver_error / space_node_t::light_speed));

    geometric_matrices_t geomat(measurement.size());
    typedef std::vector<std::pair<prn_t, float_t> > sat_range_t;
    sat_range_t sat_rate_rel;
    sat_rate_rel.reserve(measurement.size());

    // If initialization is not appropriate, more iteration will be performed.
    bool converged(false);
    for(int i(good_init ? 0 : -2); i < 10; i++){

      sat_rate_rel.clear();
      unsigned int j(0);
      res.used_satellite_mask.clear();

      const bool coarse_estimation(i <= 0);
      for(typename measurement_t::const_iterator it(measurement.begin());
          it != measurement.end();
          ++it){

        static const xyz_t zero(0, 0, 0);
        relative_property_t prop(select(it->first).relative_property(
            it->first, it->second,
            res.receiver_error, time_arrival,
            res.user_position, zero));

        if(prop.weight <= 0){
          continue; // intentionally excluded satellite
        }else{
          res.used_satellite_mask.set(it->first);
        }

        if(coarse_estimation){
          prop.weight = 1;
        }else{
          sat_rate_rel.push_back(std::make_pair(it->first, prop.rate_relative_neg));
        }

        geomat.delta_r(j, 0) = prop.range_residual;
        geomat.G(j, 0) = prop.los_neg[0];
        geomat.G(j, 1) = prop.los_neg[1];
        geomat.G(j, 2) = prop.los_neg[2];
        geomat.W(j, j) = prop.weight;

        ++j;
      }

      if((res.used_satellites = j) < 4){
        res.error_code = user_pvt_t::ERROR_INSUFFICIENT_SATELLITES;
        return res;
      }

      try{
        // Least square
        matrix_t delta_x(geomat.least_square(res.used_satellites));

        xyz_t delta_user_position(delta_x.partial(3, 1, 0, 0));
        res.user_position.xyz += delta_user_position;
        res.user_position.llh = res.user_position.xyz.llh();

        const float_t &delta_receiver_error(delta_x(3, 0));
        res.receiver_error += delta_receiver_error;
        time_arrival -= (delta_receiver_error / space_node_t::light_speed);

        if((!coarse_estimation) && (delta_user_position.dist() <= 1E-6)){
          converged = true;
          break;
        }
      }catch(std::exception &e){
        res.error_code = user_pvt_t::ERROR_POSITION_LS;
        return res;
      }
    }

    if(!converged){
      res.error_code = user_pvt_t::ERROR_POSITION_NOT_CONVERGED;
      return res;
    }

    try{
      res.update_DOP(geomat.C(res.used_satellites));
    }catch(std::exception &e){
      res.error_code = user_pvt_t::ERROR_DOP;
      return res;
    }

    if(!with_velocity){
      res.error_code = user_pvt_t::ERROR_VELOCITY_SKIPPED;
      return res;
    }

    /* 2. Calculate velocity
     * Check consistency between range and rate for velocity calculation,
     * then, assign design and weight matrices
     */
    geometric_matrices_t geomat2(res.used_satellites);
    int i_range(0), i_rate(0);

    for(typename sat_range_t::const_iterator it(sat_rate_rel.begin());
        it != sat_rate_rel.end();
        ++it, ++i_range){

      float_t rate;
      if(!select(it->first).rate(
          measurement.find(it->first)->second, // const version of measurement[PRN]
          rate)){continue;}

      // Copy design matrix and set rate
      geomat2.copy_G_W_row(geomat, i_range, i_rate);
      geomat2.delta_r(i_rate, 0) = rate + it->second;

      ++i_rate;
    }

    if(i_rate < 4){
      res.error_code = user_pvt_t::ERROR_VELOCITY_INSUFFICIENT_SATELLITES;
      return res;
    }

    try{
      // Least square
      matrix_t sol(geomat2.least_square(i_rate));

      xyz_t vel_xyz(sol.partial(3, 1, 0, 0));
      res.user_velocity_enu = enu_t::relative_rel(
          vel_xyz, res.user_position.llh);
      res.receiver_error_rate = sol(3, 0);
    }catch(std::exception &e){
      res.error_code = user_pvt_t::ERROR_VELOCITY_LS;
      return res;
    }

    res.error_code = user_pvt_t::ERROR_NO;
    return res;
  }

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

#endif /* __GPS_SOLVER_BASE_H__ */
