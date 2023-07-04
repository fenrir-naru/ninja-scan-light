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
#include <deque>
#include <algorithm>

#include <cmath>
#include <cstring>
#include <cstdlib>

#include "param/matrix.h"
#include "param/bit_array.h"
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
    matrix_t ecef2enu() const {
      float_t buf[3][3];
      llh.rotation_ecef2enu(buf);
      return matrix_t(3, 3, (float_t *)buf);
    }
  };

  typedef std::vector<std::pair<prn_t, float_t> > prn_obs_t;

  static prn_obs_t difference(
      const prn_obs_t &operand, const prn_obs_t &argument,
      const FloatT &scaling = FloatT(1)) {
    prn_obs_t res;
    for(typename prn_obs_t::const_iterator it(operand.begin()), it_end(operand.end()); it != it_end; ++it){
      for(typename prn_obs_t::const_iterator it2(argument.begin()), it2_end(argument.end()); it2 != it2_end; ++it2){
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
      L1_SIGNAL_STRENGTH_dBHz,
      L1_LOCK_SEC,
      L1_CARRIER_PHASE_AMBIGUITY_SCALE,
      L1_FREQUENCY,
      MEASUREMENT_ITEMS_PREDEFINED,
    };
  };
  typedef std::map<prn_t, std::map<int, float_t> > measurement_t;

  struct measurement_item_set_t {
    struct {
      int i, i_sigma;
    } pseudorange, doppler, carrier_phase, range_rate;
    int signal_strength, lock_sec, cp_ambiguity_scale;
  };
  static const measurement_item_set_t L1CA;

  struct measurement_util_t {
    static prn_obs_t gather(
        const measurement_t &measurement,
        const typename measurement_t::mapped_type::key_type &key,
        const FloatT &scaling = FloatT(1)){
      prn_obs_t res;
      for(typename measurement_t::const_iterator it(measurement.begin()), it_end(measurement.end());
          it != it_end; ++it){
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
      for(typename prn_obs_t::const_iterator it(new_item.begin()), it_end(new_item.end());
          it != it_end; ++it){
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
    if((res = find_value(values, measurement_items_t::L1_RANGE_RATE, buf))){

    }else if((res = find_value(values, measurement_items_t::L1_DOPPLER, buf))){
      // Fall back to doppler
      buf *= -space_node_t::L1_WaveLength();
    }
    return res;
  }

  virtual const float_t *rate_sigma(
      const typename measurement_t::mapped_type &values, float_t &buf) const {
    const float_t *res;
    if((res = find_value(values, measurement_items_t::L1_RANGE_RATE_SIGMA, buf))){

    }else if((res = find_value(values, measurement_items_t::L1_DOPPLER_SIGMA, buf))){
      // Fall back to doppler
      buf *= space_node_t::L1_WaveLength();
    }
    return res;
  }

  struct satellite_t {
    const void *impl_xyz, *impl_t; ///< pointer to actual implementation to return position and clock error
    xyz_t (*impl_position)(const void *, const gps_time_t &, const float_t &);
    xyz_t (*impl_velocity)(const void *, const gps_time_t &, const float_t &);
    float_t (*impl_clock_error)(const void *, const gps_time_t &);
    float_t (*impl_clock_error_dot)(const void *, const gps_time_t &);
    const void *impl_error; ///< pointer to actual implementation of error model
    float_t (*impl_range_sigma)(const void *, const gps_time_t &);
    float_t (*impl_rate_sigma)(const void *, const gps_time_t &);
    inline bool is_available() const {
      return (impl_xyz != NULL) && (impl_t != NULL);
    }
    /**
     * Return satellite position at the transmission time in EFEC.
     * @param t_tx transmission time
     * @param dt_transit Transit time. default is zero.
     * If zero, the returned value is along with the ECEF at the transmission time.
     * Otherwise (non-zero), they are along with the ECEF at the reception time,
     * that is, the transmission time added by the transit time.
     */
    inline xyz_t position(const gps_time_t &t_tx, const float_t &dt_transit = 0) const {
      return impl_position(impl_xyz, t_tx, dt_transit);
    }
    /**
     * Return satellite velocity at the transmission time in EFEC.
     * @see position
     */
    inline xyz_t velocity(const gps_time_t &t_tx, const float_t &dt_transit = 0) const {
      return impl_velocity(impl_xyz, t_tx, dt_transit);
    }
    /**
     * Return satellite clock error [s] at the transmission time.
     * @param t_tx transmission time
     */
    inline float_t clock_error(const gps_time_t &t_tx) const {
      return impl_clock_error(impl_t, t_tx);
    }
    /**
     * Return satellite clock error derivative [s/s] at the transmission time.
     * @param t_tx transmission time
     */
    inline float_t clock_error_dot(const gps_time_t &t_tx) const {
      return impl_clock_error_dot(impl_t, t_tx);
    }
    /**
     * Return expected user range accuracy (URA) in standard deviation (1-sigma)
     * at the transmission time.
     * @param t_tx transmission time
     */
    inline float_t range_sigma(const gps_time_t &t_tx) const {
      return impl_range_sigma
          ? impl_range_sigma(impl_error, t_tx)
          : 3.5; // 7.0 [m] of 95% (2-sigma) URE in Sec. 3.4.1 of April 2020 GPS SPS PS;
    }
    /**
     * Return expected user range rate accuracy (URRA) in standard deviation (1-sigma)
     * at the transmission time.
     * @param t_tx transmission time
     */
    inline float_t rate_sigma(const gps_time_t &t_tx) const {
      return impl_rate_sigma
          ? impl_rate_sigma(impl_error, t_tx)
          : 0.003; // 0.006 [m/s] of 95% (2-sigma) URRE in Sec. 3.4.2 of April 2020 GPS SPS PS
    }
    static const satellite_t &unavailable() {
      struct impl_t {
        static xyz_t v3(const void *, const gps_time_t &, const float_t &){
          return xyz_t(0, 0, 0);
        }
        static float_t v(const void *, const gps_time_t &){
          return float_t(0);
        }
      };
      static const satellite_t res = {
          NULL, NULL, impl_t::v3, impl_t::v3, impl_t::v, impl_t::v,
          NULL, NULL, NULL};
      return res;
    }
  };

  /**
   * Select satellite by using PRN and time
   * This function should be overridden.
   *
   * @param prn satellite number
   * @param receiver_time receiver time
   * @return (satellite_t) If available, satellite.is_available() returning true is returned.
   */
  virtual satellite_t select_satellite(
      const prn_t &prn,
      const gps_time_t &receiver_time) const {
    return satellite_t::unavailable();
  }

  struct range_corrector_t {
    virtual ~range_corrector_t() {}
    virtual bool is_available(const gps_time_t &t) const {
      return false;
    }
    virtual float_t *calculate(
        const gps_time_t &t, const pos_t &usr_pos, const enu_t &sat_rel_pos,
        float_t &buf) const = 0;
  };
  static const struct no_correction_t : public range_corrector_t {
    bool is_available(const gps_time_t &t) const {
      return true;
    }
    float_t *calculate(
        const gps_time_t &t, const pos_t &usr_pos, const enu_t &sat_rel_pos,
        float_t &buf) const {
      return &(buf = 0);
    }
  } no_correction;
  struct range_correction_t : public std::deque<const range_corrector_t *> {
    typedef std::deque<const range_corrector_t *> super_t;
    range_correction_t() : super_t() {}
    const range_corrector_t *select(const gps_time_t &t) const {
      for(typename super_t::const_iterator it(super_t::begin()), it_end(super_t::end());
          it != it_end; ++it){
        if((*it)->is_available(t)){return *it;}
      }
      return NULL;
    }
    float_t operator()(
        const gps_time_t &t, const pos_t &usr_pos, const enu_t &sat_rel_pos) const {
      float_t res;
      for(typename super_t::const_iterator it(super_t::begin()), it_end(super_t::end());
          it != it_end; ++it){
        if((*it)->calculate(t, usr_pos, sat_rel_pos, res)){return res;}
      }
      return 0;
    }
    void remove(const typename super_t::value_type &v){
      std::remove(super_t::begin(), super_t::end(), v);
    }
    void add(const typename super_t::value_type &v){
      remove(v);
      super_t::push_front(v);
    }
    void add(const super_t &values){
      for(typename super_t::const_reverse_iterator
            it(values.rbegin()), it_end(values.rend());
          it != it_end; ++it){
        add(*it);
      }
    }
  };

  /**
   * Select appropriate solver, this is provision for GNSS extension
   * @param prn satellite number
   * @return self, however, it will be overridden by a subclass
   */
  virtual const GPS_Solver_Base<FloatT> &select(const prn_t &prn) const {
    return *this;
  }

  struct relative_property_t {
    float_t range_sigma; ///< Standard deviation of pseudorange; If zero or negative, other values are invalid.
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
    struct dop_t {
      float_t g, p, h, v, t;
      dop_t() {}
      dop_t(const matrix_t &C_enu)
          : g(std::sqrt(C_enu.trace())),
          p(std::sqrt(C_enu.partial(3, 3).trace())),
          h(std::sqrt(C_enu.partial(2, 2).trace())),
          v(std::sqrt(C_enu(2, 2))),
          t(std::sqrt(C_enu(3, 3))) {}
    } dop;
    unsigned int used_satellites;
    typedef BitArray<0x400> satellite_mask_t;
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
        default:
          return false;
      }
    }
    bool velocity_solved() const {
      return error_code == ERROR_NO;
    }
  };

  struct options_t {
    template <class OptionsT, class BaseSolverT = GPS_Solver_Base<float_t> >
    struct merge_t : public BaseSolverT::options_t, OptionsT {
      merge_t() : BaseSolverT::options_t(), OptionsT() {}
      merge_t(
          const typename BaseSolverT::options_t &opt_super,
          const OptionsT &opt = OptionsT())
          : BaseSolverT::options_t(opt_super), OptionsT(opt) {}
    };

    /**
     * Flags to invalidate specific satellite
     * Its index will be adjusted for PRN.
     */
    template <int prn_begin, int prn_end>
    struct exclude_prn_t
        : public BitArray<prn_end - prn_begin + 1, unsigned int> {
      typedef BitArray<prn_end - prn_begin + 1, unsigned int> super_t;
      exclude_prn_t() : super_t() {
        super_t::clear();
      }
      bool operator[](const int &prn) const {
        return super_t::operator[](prn - prn_begin);
      }
      using super_t::set;
      void set(const int &prn, const bool &bit = true) {
        super_t::set(prn - prn_begin, bit);
      }
      using super_t::reset;
      void reset(const int &prn) {set(prn, false);}
      std::vector<int> excluded() const {
        return super_t::indices_one(prn_begin);
      }
    };
  };

  options_t available_options() const {
    return options_t();
  }

  options_t available_options(const options_t &opt_wish) const {
    return opt_wish;
  }

  options_t update_options(const options_t &opt_wish){
    return opt_wish;
  }

protected:
  template <class MatrixT>
  struct linear_solver_t {
    MatrixT G; ///< Design matrix
    /**
     * Weighting (diagonal) matrix corresponding to inverse of covariance,
     * whose (i, j) element is assumed to be 1/sigma_{i}^2 (i == j) or 0 (i != j)
     */
    MatrixT W;
    MatrixT delta_r; ///< Observation delta, i.e., observation minus measurement (y)
    linear_solver_t(const MatrixT &G_, const MatrixT &W_, const MatrixT &delta_r_)
        : G(G_), W(W_), delta_r(delta_r_) {}
    /**
     * Transform coordinate of design matrix G
     * y = G x + v = G (R^{-1} x') + v = (G R^t) x' + v = G' x' + v,
     * where R x = x' and R is a rotation matrix.
     * For example, x is in ECEF with G being calculated in ECEF,
     * and if x' in ENU is required,
     * then, R is the ecef2enu matrix, because x' = ecef2enu x.
     *
     * @param G_ original design matrix
     * @param rotation_matrix 3 by 3 rotation matrix
     * @return transformed design matrix G'
     * @see Eq.(3) and (10) in https://gssc.esa.int/navipedia/index.php/Positioning_Error,
     * however, be careful that their R is used as both range error covariance in Eq.(1)
     * and rotation matrix in Eq.(3).
     */
    static matrix_t rotate_G(const MatrixT &G_, const matrix_t &rotation_matrix){
      unsigned int r(G_.rows()), c(G_.columns()); // Normally c=4
      matrix_t res(r, c);
      res.partial(r, 3).replace(
          G_.partial(r, 3) * rotation_matrix.transpose());
      res.partial(r, c - 3, 0, 3).replace(G_.partial(r, c - 3, 0, 3));
      return res;
    }

    matrix_t G_rotated(const matrix_t &rotation_matrix) const {
      return rotate_G(G, rotation_matrix);
    }

    /**
     * Calculate C matrix, where C = G^t * G to be used for DOP calculation
     * @return C matrix
     */
    matrix_t C() const {
      return (G.transpose() * G).inverse();
    }
    /**
     * Transform coordinate of matrix C
     * C' = (G * R^t)^t * (G * R^t) = R * G^t * G * R^t = R * C * R^t,
     * where R is a rotation matrix, for example, ECEF to ENU.
     *
     * @param rotation_matrix 3 by 3 rotation matrix
     * @return transformed matrix C'
     * @see rotate_G
     */
    static matrix_t rotate_C(const matrix_t &C, const matrix_t &rotation_matrix){
      unsigned int n(C.rows()); // Normally n=4
      matrix_t res(n, n);
      res.partial(3, 3).replace( // upper left
          rotation_matrix * C.partial(3, 3) * rotation_matrix.transpose());
      res.partial(3, n - 3, 0, 3).replace( // upper right
          rotation_matrix * C.partial(3, n - 3, 0, 3));
      res.partial(n - 3, 3, 3, 0).replace( // lower left
          C.partial(n - 3, 3, 3, 0) * rotation_matrix.transpose());
      res.partial(n - 3, n - 3, 3, 3).replace( // lower right
          C.partial(n - 3, n - 3, 3, 3));
      return res;
    }
    /**
     * Solve x of linear equation (y = G x + v) to minimize sigma{v^t * v}
     * where v =~ N(0, sigma), and y and G are observation delta (=delta_r variable)
     * and a design matrix, respectively.
     * This yields x = (G^t * W * G)^{-1} * (G^t * W) y = S y.
     *
     * 4 by row(y) S matrix (=(G^t * W * G)^{-1} * (G^t * W)) will be used to calculate protection level
     * to investigate relationship between bias on each satellite and solution.
     * residual v = (I - P) = (I - G S), where P = G S, which is irrelevant to rotation,
     * because P = G R R^{t} S = G' S'.
     *
     * @param S (output) coefficient matrix to calculate solution, i.e., (G^t * W * G)^{-1} * (G^t * W)
     * @return x vector
     * @see rotate_S()
     */
    inline matrix_t least_square(matrix_t &S) const {
      matrix_t Gt_W(G.transpose() * W);
      S = (Gt_W * G).inverse() * Gt_W;
      return S * delta_r;
    }
    matrix_t least_square() const {
      matrix_t S;
      return least_square(S);
    }
    /**
     * Transform coordinate of coefficient matrix of solution S
     * x' = R x = R S y, where R is a rotation matrix, for example, ECEF to ENU.
     * Be careful, R is not ENU to ECEF in the example, which should be consistent to rotate_G().
     *
     * @param S coefficient matrix of solution
     * @param rotation_matrix 3 by 3 rotation matrix
     * @return transformed coefficient matrix S'
     * @see rotate_G
     */
    static matrix_t rotate_S(const matrix_t &S, const matrix_t &rotation_matrix){
      unsigned int r(S.rows()), c(S.columns()); // Normally r=4
      matrix_t res(r, c);
      res.partial(3, c).replace(rotation_matrix * S.partial(3, c));
      res.partial(r - 3, c, 3, 0).replace(S.partial(r - 3, c, 3, 0));
      return res;
    }
    /**
     * Calculate linear effect from bias on each range measurement to horizontal/vertical estimation.
     *
     * @param S coefficient matrix of solution
     * @param rotation_matrix 3 by 3 matrix, which makes S aligned to ENU or NED coordinates
     * @return slopes matrix (1st and 2nd columns correspond to horizontal and vertical components, respectively)
     * @see Eq.(5.26), (5.27) in @article{Mink, title={Performance of Receiver Autonomous Integrity Monitoring (RAIM) for Maritime Operations}, author={Mink, Michael}, pages={220} }
     * Note: returned values are not performed to be multiplied by sqrt(N-4)
     */
    matrix_t slope_HV(const matrix_t &S, const matrix_t &rotation_matrix = matrix_t::getI(3)) const {
      matrix_t S_ENU_or_NED(rotate_S(S, rotation_matrix));
      matrix_t res(G.rows(), 2); // 1st column = horizontal, 2nd column = vertical
      matrix_t P(G * S);
      for(unsigned int i(0), i_end(res.rows()); i < i_end; i++){
        if(W(i, i) <= 0){
          res(i, 0) = res(i, 1) = 0;
          continue;
        }
        float_t denom(std::sqrt((-P(i, i) + 1) * W(i, i)));
        res(i, 0) = std::sqrt(  // horizontal
            std::pow(S_ENU_or_NED(0, i), 2) + std::pow(S_ENU_or_NED(1, i), 2)) / denom;
        res(i, 1) = std::abs(S_ENU_or_NED(2, i)) / denom; // vertical
      }
      return res;
    }
    /**
     * Calculate weighted square sum of residual (WSSR) based on least square solution.
     * v^t W v (= (y - G x)^t W (y - G x) = y^t W (I-P) y)
     *
     * @param x solution
     * @return WSSR scalar
     */
    float_t wssr(const matrix_t &x) const {
      matrix_t v(delta_r - G * x);
      return (v.transpose() * W * v)(0, 0);
    }
    float_t wssr() const {return wssr(least_square());}
    /**
     * Calculate weighted square sum of residual (WSSR) based on least square solution
     * with solution coefficient matrix (S).
     * v^t W v (= (y - G x)^t W (y - G x) = y^t W (I-G*S) y)
     *
     * @param S coefficient matrix of solution
     * @param W_dash (output) pointer to store scale factor matrix of y^t y,
     * i.e., *W_dash = norm(t(I-G*S)) * W * norm(I-G*S)
     * @return WSSR scalar
     */
    float_t wssr_S(const matrix_t &S, matrix_t *W_dash = NULL) const {
      matrix_t rhs(matrix_t::getI(W.rows()) - (G * S));
      if(W_dash){
        *W_dash = W.copy();
        for(unsigned int i(0), i_end(rhs.rows()); i < i_end; ++i){
          (*W_dash)(i, i) *= (rhs.rowVector(i) * rhs.rowVector(i).transpose())(0, 0);
        }
      }
      return (delta_r.transpose() * W * rhs * delta_r)(0, 0);
    }
    typedef linear_solver_t<typename MatrixT::partial_offsetless_t> partial_t;
    partial_t partial(unsigned int size) const {
      if(size >= G.rows()){size = G.rows();}
      return partial_t(
          G.partial(size, G.columns()), W.partial(size, size), delta_r.partial(size, 1));
    }
    typedef linear_solver_t<typename MatrixT::circular_t> exclude_t;
    exclude_t exclude(const unsigned int &row) const {
      unsigned int size(G.rows()), offset((row + 1) % size);
      if(size >= 1){size--;}
      // generate matrices having circular view
      return exclude_t(
          G.circular(offset, 0, size, G.columns()),
          W.circular(offset, offset, size, size),
          delta_r.circular(offset, 0, size, 1));
    }
    template <class MatrixT2>
    void copy_G_W_row(const linear_solver_t<MatrixT2> &src,
        const unsigned int &i_src, const unsigned int &i_dst){
      unsigned int c_dst(G.columns()), c_src(src.G.columns()),
          c(c_dst > c_src ? c_src : c_dst); // Normally c=4
      for(unsigned int j(0); j < c; ++j){
        G(i_dst, j) = src.G(i_src, j);
      }
      W(i_dst, i_dst) = src.W(i_src, i_src);
    }
  };

  struct geometric_matrices_t : public linear_solver_t<matrix_t> {
    typedef linear_solver_t<matrix_t> super_t;
    geometric_matrices_t(
        const unsigned int &capacity,
        const unsigned int &estimate_system_differences = 0)
        : super_t(
          matrix_t(capacity, 4 + estimate_system_differences),
          matrix_t(capacity, capacity), matrix_t(capacity, 1)) {
      for(unsigned int i(0); i < capacity; ++i){
        super_t::G(i, 3) = 1;
      }
    }
  };

  struct measurement2_item_t {
    prn_t prn;
    const typename measurement_t::mapped_type *k_v_map;
    const GPS_Solver_Base<FloatT> *solver;
  };
  typedef std::vector<measurement2_item_t> measurement2_t;


  /**
   * Update position solution
   * This function will be called multiple times in a solution calculation iteration.
   * It may be overridden in a subclass for extraction of calculation source
   * such as design matrix.
   *
   * @param geomat residual, desgin and weight matrices
   * @param res (in,out) current solution to be updated
   * @return (bool) If solution will be treated as the final solution, true is returned; otherwise false.
   */
  virtual bool update_position_solution(
      const geometric_matrices_t &geomat,
      user_pvt_t &res) const {

    // Least square
    matrix_t delta_x(geomat.partial(res.used_satellites).least_square());

    xyz_t delta_user_position(delta_x.partial(3, 1));
    res.user_position.xyz += delta_user_position;
    res.user_position.llh = res.user_position.xyz.llh();

    const float_t &delta_receiver_error(delta_x(3, 0));
    res.receiver_error += delta_receiver_error;

    return (delta_x.partial(4, 1).norm2F() <= 1E-6); // equivalent to abs(x) <= 1E-3 [m]
  }

  struct user_pvt_opt_t {
    /**
     * how many iterations are required to perform coarse estimation before fine one;
     * If initial position and clock error are goodly guessed, this will be zero.
     */
    int warmup;
    int max_trial;
    bool estimate_velocity;
    user_pvt_opt_t(
        const bool &good_init = true,
        const bool &with_velocity = true)
        : warmup(good_init ? 0 : 5),
        max_trial(10),
        estimate_velocity(with_velocity) {}
  };

  /**
   * Calculate User position/velocity by using associated solvers.
   * This function can be overridden in a subclass.
   *
   * @param res (out) calculation results and matrices used for calculation
   * @param measurement PRN, pseudo-range, and pseudo-range rate information
   * associated with a specific solver corresponding to a satellite system
   * @param receiver_time receiver time at measurement
   * @param user_position_init initial solution of user position in XYZ meters and LLH
   * @param receiver_error_init initial solution of receiver clock error in meters
   * @param opt options for user PVT calculation
   * @see update_ephemeris(), register_ephemeris
   */
  virtual void user_pvt(
      user_pvt_t &res,
      const measurement2_t &measurement,
      const gps_time_t &receiver_time,
      const pos_t &user_position_init,
      const float_t &receiver_error_init,
      const user_pvt_opt_t &opt = user_pvt_opt_t()) const {

    res.receiver_time = receiver_time;

    // 1. Position calculation

    res.user_position = user_position_init;
    res.receiver_error = receiver_error_init;

    geometric_matrices_t geomat(measurement.size());
    typedef std::vector<std::pair<unsigned int, float_t> > index_obs_t;
    index_obs_t idx_rate_rel; // [(index of measurement, relative rate), ...]
    idx_rate_rel.reserve(measurement.size());

    // If initialization is not appropriate, more iteration will be performed.
    bool converged(false);
    for(int i_trial(-opt.warmup); i_trial < opt.max_trial; i_trial++){

      idx_rate_rel.clear();
      unsigned int i_row(0), i_measurement(0);
      res.used_satellite_mask.clear();

      gps_time_t time_arrival(
          receiver_time - (res.receiver_error / space_node_t::light_speed));

      const bool coarse_estimation(i_trial <= 0);
      for(typename measurement2_t::const_iterator it(measurement.begin()), it_end(measurement.end());
          it != it_end;
          ++it, ++i_measurement){

        static const xyz_t zero(0, 0, 0);
        relative_property_t prop(it->solver->relative_property(
            it->prn, *(it->k_v_map),
            res.receiver_error, time_arrival,
            res.user_position, zero));

        if(prop.range_sigma <= 0){
          continue; // intentionally excluded satellite
        }else{
          res.used_satellite_mask.set(it->prn);
        }

        if(coarse_estimation){
          prop.range_sigma = 1;
        }else{
          idx_rate_rel.push_back(std::make_pair(i_measurement, prop.rate_relative_neg));
        }

        geomat.delta_r(i_row, 0) = prop.range_residual;
        geomat.G(i_row, 0) = prop.los_neg[0];
        geomat.G(i_row, 1) = prop.los_neg[1];
        geomat.G(i_row, 2) = prop.los_neg[2];
        geomat.W(i_row, i_row) = 1. / std::pow(prop.range_sigma, 2);

        ++i_row;
      }

      if((res.used_satellites = i_row) < 4){
        res.error_code = user_pvt_t::ERROR_INSUFFICIENT_SATELLITES;
        return;
      }

      try{
        converged = update_position_solution(geomat, res);
        if((!coarse_estimation) && converged){break;}
      }catch(const std::runtime_error &e){ // expect to detect matrix operation error
        res.error_code = user_pvt_t::ERROR_POSITION_LS;
        return;
      }
    }

    if(!converged){
      res.error_code = user_pvt_t::ERROR_POSITION_NOT_CONVERGED;
      return;
    }

    try{
      res.dop = typename user_pvt_t::dop_t(geomat.rotate_C(
          geomat.partial(res.used_satellites).C(), res.user_position.ecef2enu()));
    }catch(const std::runtime_error &e){ // expect to detect matrix operation error
      res.error_code = user_pvt_t::ERROR_DOP;
      return;
    }

    if(!opt.estimate_velocity){
      res.error_code = user_pvt_t::ERROR_VELOCITY_SKIPPED;
      return;
    }

    /* 2. Calculate velocity
     * Check consistency between range and rate for velocity calculation,
     * then, assign design and weight matrices
     */
    geometric_matrices_t geomat2(res.used_satellites);
    int i_range(0), i_rate(0);

    for(typename index_obs_t::const_iterator it(idx_rate_rel.begin()), it_end(idx_rate_rel.end());
        it != it_end;
        ++it, ++i_range){

      float_t rate;
      if(!(measurement[it->first].solver->rate(
          *(measurement[it->first].k_v_map), // const version of measurement[PRN]
          rate))){continue;}

      // Copy design matrix and set rate
      geomat2.copy_G_W_row(geomat, i_range, i_rate);
      geomat2.delta_r(i_rate, 0) = rate + it->second;

      ++i_rate;
    }

    if(i_rate < 4){
      res.error_code = user_pvt_t::ERROR_VELOCITY_INSUFFICIENT_SATELLITES;
      return;
    }

    try{
      // Least square
      matrix_t sol(geomat2.partial(i_rate).least_square());

      xyz_t vel_xyz(sol.partial(3, 1, 0, 0));
      res.user_velocity_enu = enu_t::relative_rel(
          vel_xyz, res.user_position.llh);
      res.receiver_error_rate = sol(3, 0);
    }catch(const std::runtime_error &e){ // expect to detect matrix operation error
      res.error_code = user_pvt_t::ERROR_VELOCITY_LS;
      return;
    }

    res.error_code = user_pvt_t::ERROR_NO;
  }

public:
  /**
   * Calculate User position/velocity with hint
   * This is multi-constellation version,
   * and reference implementation to be hidden by optimized one in sub class.
   *
   * @param res (out) calculation results and matrices used for calculation
   * @param measurement PRN, pseudo-range, and pseudo-range rate information
   * @param receiver_time receiver time at measurement
   * @param user_position_init initial solution of user position in XYZ meters and LLH
   * @param receiver_error_init initial solution of receiver clock error in meters
   * @param good_init if true, initial position and clock error are goodly guessed.
   * @param with_velocity if true, perform velocity estimation.
   * @see update_ephemeris(), register_ephemeris
   */
  virtual void user_pvt(
      user_pvt_t &res,
      const measurement_t &measurement,
      const gps_time_t &receiver_time,
      const pos_t &user_position_init,
      const float_t &receiver_error_init,
      const bool &good_init = true,
      const bool &with_velocity = true) const {

    measurement2_t measurement2;
    measurement2.reserve(measurement.size());
    for(typename measurement_t::const_iterator it(measurement.begin()), it_end(measurement.end());
        it != it_end; ++it){
      typename measurement2_t::value_type v = {
          it->first, &(it->second), &select(it->first)}; // prn, measurement, solver
      if(!v.solver->select_satellite(v.prn, receiver_time).is_available()){
        // pre-check satellite availability; remove it when its position is unknown
        continue;
      }
      measurement2.push_back(v);
    }
    user_pvt(
        res,
        measurement2, receiver_time, user_position_init, receiver_error_init,
        user_pvt_opt_t(good_init, with_velocity));
  }

  template <class SolverT>
  struct solver_interface_t {
    const SolverT &solver;
    solver_interface_t(const SolverT &solver_) : solver(solver_) {}

    typedef typename SolverT::user_pvt_t pvt_t;

    pvt_t user_pvt(
        const measurement_t &measurement,
        const gps_time_t &receiver_time,
        const pos_t &user_position_init,
        const float_t &receiver_error_init,
        const bool &good_init = true,
        const bool &with_velocity = true) const {
      pvt_t res;
      solver.user_pvt(res,
          measurement, receiver_time,
          user_position_init, receiver_error_init,
          good_init, with_velocity);
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
    pvt_t user_pvt(
        const measurement_t &measurement,
        const gps_time_t &receiver_time,
        const xyz_t &user_position_init_xyz,
        const float_t &receiver_error_init,
        const bool &good_init = true,
        const bool &with_velocity = true) const {
      pos_t user_position_init = {user_position_init_xyz, user_position_init_xyz.llh()};
      return user_pvt(
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
    pvt_t user_pvt(
        const measurement_t &measurement,
        const gps_time_t &receiver_time) const {
      return user_pvt(measurement, receiver_time, xyz_t(), 0, false);
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
    pvt_t user_pvt(
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
      return user_pvt(
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
    pvt_t user_pvt(
        const prn_obs_t &prn_range,
        const prn_obs_t &prn_rate,
        const gps_time_t &receiver_time,
        const xyz_t &user_position_init_xyz,
        const float_t &receiver_error_init,
        const bool &good_init = true,
        const bool &with_velocity = true) const {
      pos_t user_position_init = {user_position_init_xyz, user_position_init_xyz.llh()};
      return user_pvt(
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
    pvt_t user_pvt(
        const prn_obs_t &prn_range,
        const prn_obs_t &prn_rate,
        const gps_time_t &receiver_time) const {
      return user_pvt(prn_range, prn_rate, receiver_time, xyz_t(), 0, false);
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
    pvt_t user_position(
        const prn_obs_t &prn_range,
        const gps_time_t &receiver_time,
        const xyz_t &user_position_init,
        const float_t &receiver_error_init,
        const bool &good_init = true) const {

      return user_pvt(
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
    pvt_t user_position(
        const prn_obs_t &prn_range,
        const gps_time_t &receiver_time) const {
      return user_position(prn_range, receiver_time, xyz_t(), 0, false);
    }
  };

  solver_interface_t<GPS_Solver_Base<FloatT> > solve() const {
    return solver_interface_t<GPS_Solver_Base<FloatT> >(*this);
  }

  static typename user_pvt_t::dop_t dop(const matrix_t &C, const pos_t &user_position) {
    return typename user_pvt_t::dop_t(geometric_matrices_t::rotate_C(C, user_position.ecef2enu()));
  }
};

template <class FloatT>
const typename GPS_Solver_Base<FloatT>::measurement_item_set_t
    GPS_Solver_Base<FloatT>::L1CA = {
#define make_entry(key) \
    GPS_Solver_Base<FloatT>::measurement_items_t::L1_ ## key
#define make_entry2(key) { \
    make_entry(key), \
    make_entry(key ## _SIGMA)}
      make_entry2(PSEUDORANGE),
      make_entry2(DOPPLER),
      make_entry2(CARRIER_PHASE),
      make_entry2(RANGE_RATE),
      make_entry(SIGNAL_STRENGTH_dBHz),
      make_entry(LOCK_SEC),
      make_entry(CARRIER_PHASE_AMBIGUITY_SCALE),
#undef make_entry2
#undef make_entry
    };

template <class FloatT>
const typename GPS_Solver_Base<FloatT>::range_error_t
    GPS_Solver_Base<FloatT>::range_error_t::not_corrected = {
      MASK_RECEIVER_CLOCK | MASK_SATELLITE_CLOCK | MASK_IONOSPHERIC | MASK_TROPOSPHERIC,
      {0},
    };

template <class FloatT>
const typename GPS_Solver_Base<FloatT>::no_correction_t
    GPS_Solver_Base<FloatT>::no_correction;

template <class FloatT, class PVT_BaseT = typename GPS_Solver_Base<FloatT>::user_pvt_t>
struct GPS_PVT_Debug : public PVT_BaseT {
  typename GPS_Solver_Base<FloatT>::matrix_t G, W, delta_r;
};

template <class FloatT, class SolverBaseT = GPS_Solver_Base<FloatT> >
struct GPS_Solver_Base_Debug : public SolverBaseT {
  typedef SolverBaseT base_t;
  typedef GPS_Solver_Base_Debug<FloatT, SolverBaseT> self_t;
  virtual ~GPS_Solver_Base_Debug() {}

#if defined(__GNUC__) && (__GNUC__ < 5)
#define inheritate_type(x) typedef typename base_t::x x;
#else
#define inheritate_type(x) using typename base_t::x;
#endif
  inheritate_type(float_t);
  inheritate_type(geometric_matrices_t);
#undef inheritate_type

  typedef GPS_PVT_Debug<float_t, typename base_t::user_pvt_t> user_pvt_t;

  typename base_t::template solver_interface_t<self_t> solve() const {
    return typename base_t::template solver_interface_t<self_t>(*this);
  }

protected:
  virtual bool update_position_solution(
      const geometric_matrices_t &geomat,
      typename GPS_Solver_Base<FloatT>::user_pvt_t &res) const {

    // Least square
    if(!base_t::update_position_solution(geomat, res)){
      return false;
    }
    static_cast<user_pvt_t &>(res).G = geomat.G;
    static_cast<user_pvt_t &>(res).W = geomat.W;
    static_cast<user_pvt_t &>(res).delta_r = geomat.delta_r;
    return true;
  }
};


#endif /* __GPS_SOLVER_BASE_H__ */
