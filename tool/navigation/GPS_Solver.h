/**
 * @file GPS solver
 * - Mainly, single positioning
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

#ifndef __GPS_SOLVER_H__
#define __GPS_SOLVER_H__

#include <utility>
#include <vector>
#include <exception>

#include <cmath>
#include <cstddef>

#include "param/bit_array.h"

#include "GPS.h"
#include "GPS_Solver_Base.h"
#include "NTCM.h"

template <class FloatT>
struct GPS_Solver_GeneralOptions {
  FloatT elevation_mask;
  FloatT residual_mask;

  GPS_Solver_GeneralOptions()
      : elevation_mask(0), // elevation mask default is 0 [deg]
      residual_mask(30) { // range residual mask is 30 [m]

  }
};

template <class FloatT>
struct GPS_SinglePositioning_Options : public GPS_Solver_GeneralOptions<FloatT> {
  
  // PRN ranges from 1 to 256 (including GPS compatible systems such as QZSS)
  typename GPS_Solver_Base<FloatT>::options_t::template exclude_prn_t<1, 256> exclude_prn;
  
  GPS_SinglePositioning_Options() 
      : GPS_Solver_GeneralOptions<FloatT>(), exclude_prn() {

  }
};

template <class FloatT, class SolverBaseT = GPS_Solver_Base<FloatT> >
class GPS_SinglePositioning : public SolverBaseT {
  public:
    typedef GPS_SinglePositioning<FloatT, SolverBaseT> self_t;
    typedef SolverBaseT base_t;
  private:
    self_t &operator=(const self_t &);
    GPS_SinglePositioning(const self_t &);

  public:
#if defined(__GNUC__) && (__GNUC__ < 5)
#define inheritate_type(x) typedef typename base_t::x x;
#else
#define inheritate_type(x) using typename base_t::x;
#endif

    inheritate_type(float_t);
    inheritate_type(matrix_t);
    inheritate_type(prn_t);

    typedef typename base_t::space_node_t space_node_t;
    inheritate_type(gps_time_t);

    inheritate_type(xyz_t);
    inheritate_type(llh_t);
    inheritate_type(enu_t);

    inheritate_type(pos_t);

    inheritate_type(prn_obs_t);
    typedef typename base_t::measurement_t measurement_t;
    inheritate_type(measurement_items_t);
    typedef typename base_t::satellite_t satellite_t;
    typedef typename base_t::range_error_t range_error_t;
    typedef typename base_t::range_corrector_t range_corrector_t;
    typedef typename base_t::range_correction_t range_correction_t;

    inheritate_type(relative_property_t);
    inheritate_type(geometric_matrices_t);
    inheritate_type(user_pvt_t);
#undef inheritate_type

    typedef typename GPS_Solver_Base<float_t>::options_t::template merge_t<
        GPS_SinglePositioning_Options<float_t>, base_t> options_t;

  protected:
    GPS_SinglePositioning_Options<float_t> _options;

  public:
    struct satellites_t {
      const void *impl;
      satellite_t (*impl_select)(const void *, const prn_t &, const gps_time_t &);
      inline satellite_t select(const prn_t &prn, const gps_time_t &receiver_time) const {
        return impl_select(impl, prn, receiver_time);
      }
      static satellite_t select_broadcast(
          const void *ptr, const prn_t &prn, const gps_time_t &receiver_time){
        // If this static function is defined in inner struct,
        // C2440 error raises with VC2010
        const typename space_node_t::satellites_t &sats(
            reinterpret_cast<const space_node_t *>(ptr)->satellites());
        const typename space_node_t::satellites_t::const_iterator it_sat(sats.find(prn));
        if((it_sat == sats.end()) // has ephemeris?
            || (!it_sat->second.ephemeris().is_valid(receiver_time))){ // valid ephemeris?
          return satellite_t::unavailable();
        }
        struct impl_t {
          static inline const typename space_node_t::Satellite &sat(const void *ptr) {
            return *reinterpret_cast<const typename space_node_t::Satellite *>(ptr);
          }
          static xyz_t position(const void *ptr, const gps_time_t &t_tx, const float_t &dt_transit) {
            return sat(ptr).position(t_tx, dt_transit);
          }
          static xyz_t velocity(const void *ptr, const gps_time_t &t_tx, const float_t &dt_transit) {
            return sat(ptr).velocity(t_tx, dt_transit);
          }
          static float_t clock_error(const void *ptr, const gps_time_t &t_tx) {
            return sat(ptr).clock_error(t_tx);
          }
          static float_t clock_error_dot(const void *ptr, const gps_time_t &t_tx) {
            return sat(ptr).clock_error_dot(t_tx);
          }
          static float_t range_sigma(const void *ptr, const gps_time_t &t_tx) {
            return sat(ptr).ephemeris().URA;
          }
        };
        satellite_t res = {
            &(it_sat->second), &(it_sat->second), // position, clock
            impl_t::position, impl_t::velocity,
            impl_t::clock_error, impl_t::clock_error_dot,
            &(it_sat->second), impl_t::range_sigma, NULL}; // error model
        return res;
      }
      satellites_t(const space_node_t &sn)
          : impl(&sn), impl_select(select_broadcast) {}
    } satellites;

    struct klobuchar_t : public range_corrector_t {
      const space_node_t &space_node;
      klobuchar_t(const space_node_t &sn) : range_corrector_t(), space_node(sn) {}
      bool is_available(const gps_time_t &t) const {
        return space_node.is_valid_iono();
      }
      float_t *calculate(
          const gps_time_t &t, const pos_t &usr_pos, const enu_t &sat_rel_pos,
          float_t &buf) const {
        if(!is_available(t)){return NULL;}
        return &(buf = space_node.iono_correction(sat_rel_pos, usr_pos.llh, t));
      }
    } ionospheric_klobuchar;

    struct ntcm_gl_t : public range_corrector_t {
      float_t f_10_7;
      ntcm_gl_t() : range_corrector_t(), f_10_7(-1) {}
      bool is_available(const gps_time_t &t) const {
        return f_10_7 >= 0;
      }
      float_t *calculate(
          const gps_time_t &t, const pos_t &usr_pos, const enu_t &sat_rel_pos,
          float_t &buf) const {
        if(!is_available(t)){return NULL;}
        typename space_node_t::pierce_point_res_t pp(
            space_node_t::pierce_point(sat_rel_pos, usr_pos.llh));
        return &(buf = -space_node_t::tec2delay(space_node_t::slant_factor(sat_rel_pos)
            * NTCM_GL_Generic<float_t>::tec_vert(
              pp.latitude, pp.longitude, t.year(), f_10_7)));
      }
    } ionospheric_ntcm_gl;

    struct tropospheric_simplified_t : public range_corrector_t {
      tropospheric_simplified_t() : range_corrector_t() {}
      bool is_available(const gps_time_t &t) const {
        return true;
      }
      float_t *calculate(
          const gps_time_t &t, const pos_t &usr_pos, const enu_t &sat_rel_pos,
          float_t &buf) const {
        return &(buf = space_node_t::tropo_correction(sat_rel_pos, usr_pos.llh));
      }
    } tropospheric_simplified;

    range_correction_t ionospheric_correction, tropospheric_correction;

    options_t available_options() const {
      return options_t(base_t::available_options(), _options);
    }

    options_t available_options(const options_t &opt_wish) const {
      GPS_SinglePositioning_Options<float_t> opt(opt_wish);
      return options_t(base_t::available_options(opt_wish), opt);
    }

    options_t update_options(const options_t &opt_wish){
      _options = opt_wish;
      return options_t(base_t::update_options(opt_wish), _options);
    }

    GPS_SinglePositioning(const space_node_t &sn)
        : base_t(), _options(available_options(options_t())),
        satellites(sn),
        ionospheric_klobuchar(sn), ionospheric_ntcm_gl(),
        tropospheric_simplified(),
        ionospheric_correction(), tropospheric_correction() {

      // default ionospheric correction:
      // Broadcasted Klobuchar parameters are at least required for solution.
      ionospheric_correction.push_front(&ionospheric_klobuchar);

      // default troposheric correction: simplified
      tropospheric_correction.push_front(&tropospheric_simplified);
    }

    ~GPS_SinglePositioning(){}

    struct residual_t {
      float_t &residual;
      float_t &los_neg_x;
      float_t &los_neg_y;
      float_t &los_neg_z;
      float_t &range_sigma;
    };

    /**
     * Get corrected range in accordance with current status
     *
     * @param sat satellite
     * @param range "corrected" pseudo range subtracted by (temporal solution of) receiver clock error in meter
     * @param time_arrival time when signal arrive at receiver
     * @param usr_pos (temporal solution of) user position
     * @param residual calculated residual with line of site vector, and pseudorange standard deviation (sigma);
     * When sigma is equal to or less than zero, the calculated results should not be used.
     * @param error Some correction can be overwritten. If its unknown_flag is zero,
     * corrections will be skipped as possible. @see range_errors_t
     * @return (float_t) corrected range just including delay, and excluding receiver/satellite error.
     */
    float_t range_corrected(
        const satellite_t &sat,
        float_t range,
        const gps_time_t &time_arrival,
        const pos_t &usr_pos,
        residual_t &residual,
        const range_error_t &error = range_error_t::not_corrected) const {

      static const float_t &c(space_node_t::light_speed);

      // Clock error correction
      range += ((error.unknown_flag & range_error_t::SATELLITE_CLOCK)
          ? (sat.clock_error(time_arrival - range / c) * c)
          : error.value[range_error_t::SATELLITE_CLOCK]);

      // Calculate satellite position
      float_t dt_transit(range / c);
      gps_time_t time_depature(time_arrival - dt_transit);
      xyz_t sat_pos(sat.position(time_depature, dt_transit));
      float_t geometric_range(usr_pos.xyz.distance(sat_pos));

      // Calculate residual
      residual.residual = range - geometric_range;

      // Setup design matrix
      residual.los_neg_x = -(sat_pos.x() - usr_pos.xyz.x()) / geometric_range;
      residual.los_neg_y = -(sat_pos.y() - usr_pos.xyz.y()) / geometric_range;
      residual.los_neg_z = -(sat_pos.z() - usr_pos.xyz.z()) / geometric_range;

      enu_t relative_pos(enu_t::relative(sat_pos, usr_pos.xyz));

      if(error.unknown_flag & range_error_t::MASK_IONOSPHERIC){
        residual.residual += ionospheric_correction(time_arrival, usr_pos, relative_pos);
      }else{
        residual.residual += error.value[range_error_t::IONOSPHERIC];
      }

      // Tropospheric
      residual.residual += (error.unknown_flag & range_error_t::MASK_TROPOSPHERIC)
          ? tropospheric_correction(time_arrival, usr_pos, relative_pos)
          : error.value[range_error_t::TROPOSPHERIC];

      // Setup range standard deviation, whose reciprocal square is used as weight
      residual.range_sigma = 1E+4; // sufficiently big value, 1E4 [m]
      do{
        // If residual is too big, gently exclude it.
        if(std::abs(residual.residual) > _options.residual_mask){break;}

        float_t elv(relative_pos.elevation());
        if(elv < _options.elevation_mask){
          residual.range_sigma = 0; // exclude it when elevation is less than threshold
          break;
        }

        residual.range_sigma = sat.range_sigma(time_depature);

        /* elevation weight based on "GPS実用プログラミング"
         * elevation[deg] :   90    53    45    30    15    10    5
         * sf_sigma(k)    :   0.80  1.00  1.13  1.60  3.09  4.61  9.18
         * weight(k^-2)   :   1.56  1.00  0.78  0.39  0.10  0.05  0.01
         */
        static const float_t max_sf(10);
        static const float_t elv_limit(std::asin(0.8/max_sf)); // limit
        residual.range_sigma *= (elv > elv_limit) ? (0.8 / sin(elv)) : max_sf;
      }while(false);


      return range;
    }

    /**
     * Get relative rate (negative polarity) in accordance with current status
     *
     * @param sat satellite
     * @param range "corrected" pseudo range subtracted by (temporal solution of) receiver clock error in meter
     * @param time_arrival time when signal arrive at receiver
     * @param usr_vel (temporal solution of) user velocity
     * @param los_neg_x line of site X
     * @param los_neg_y line of site Y
     * @param los_neg_z line of site Z
     * @return (float_t) relative rate.
     */
    float_t rate_relative_neg(
        const satellite_t &sat,
        const float_t &range,
        const gps_time_t &time_arrival,
        const xyz_t &usr_vel,
        const float_t &los_neg_x, const float_t &los_neg_y, const float_t &los_neg_z) const {

      static const float_t &c(space_node_t::light_speed);

      float_t dt_transit(range / c);
      gps_time_t t_tx(time_arrival - dt_transit);
      xyz_t rel_vel(sat.velocity(t_tx, dt_transit) - usr_vel); // Calculate velocity
      return los_neg_x * rel_vel.x()
          + los_neg_y * rel_vel.y()
          + los_neg_z * rel_vel.z()
          + sat.clock_error_dot(t_tx) * c; // considering clock error rate
    }

    /**
     * Select satellite by using PRN and time
     *
     * @param prn satellite number
     * @param receiver_time receiver time
     * @return (satellite_t) If available, satellite.is_available() returning true is returned.
     */
    satellite_t select_satellite(
        const prn_t &prn,
        const gps_time_t &receiver_time) const {
      if(_options.exclude_prn[prn]){return satellite_t::unavailable();}
      return satellites.select(prn, receiver_time);
    }

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
    relative_property_t relative_property(
        const prn_t &prn,
        const typename measurement_t::mapped_type &measurement,
        const float_t &receiver_error,
        const gps_time_t &time_arrival,
        const pos_t &usr_pos,
        const xyz_t &usr_vel) const {

      relative_property_t res = {0};

      float_t range;
      range_error_t range_error;
      if(!this->range(measurement, range, &range_error)){
        return res; // If no range entry, return with range_sigma = 0
      }

      satellite_t sat(select_satellite(prn, time_arrival));
      if(!sat.is_available()){return res;} // If satellite is unavailable, return with range_sigma = 0

      residual_t residual = {
        res.range_residual,
        res.los_neg[0], res.los_neg[1], res.los_neg[2],
        res.range_sigma,
      };

      res.range_corrected = range_corrected(
          sat, range - receiver_error, time_arrival,
          usr_pos, residual, range_error);
      res.rate_relative_neg = rate_relative_neg(sat, res.range_corrected, time_arrival, usr_vel,
          res.los_neg[0], res.los_neg[1], res.los_neg[2]);

#if 0
      // TODO consider case when standard deviation of pseudorange measurement is provided by receiver
      if(!this->range_sigma(measurement, res.range_sigma)){
        // If receiver's range variance is not provided
        res.range_sigma = 1E0; // TODO range error variance [m]
      }
#endif

      return res;
    }

    /**
     * Calculate User position/velocity with hint
     * This is optimized version for GPS-only constellation
     *
     * @param res (out) calculation results and matrices used for calculation
     * @param measurement PRN, pseudo-range, pseudo-range rate information
     * @param receiver_time receiver time at measurement
     * @param user_position_init initial solution of user position in XYZ meters and LLH
     * @param receiver_error_init initial solution of receiver clock error in meters
     * @param good_init if true, initial position and clock error are goodly guessed.
     * @param with_velocity if true, perform velocity estimation.
     * @see update_ephemeris(), register_ephemeris
     */
    void user_pvt(
        user_pvt_t &res,
        const measurement_t &measurement,
        const gps_time_t &receiver_time,
        const pos_t &user_position_init,
        const float_t &receiver_error_init,
        const bool &good_init = true,
        const bool &with_velocity = true) const {

      res.receiver_time = receiver_time;

      if(!ionospheric_correction.select(receiver_time)){
        res.error_code = user_pvt_t::ERROR_INVALID_IONO_MODEL;
        return;
      }

      typename base_t::measurement2_t measurement2;
      measurement2.reserve(measurement.size());
      for(typename measurement_t::const_iterator it(measurement.begin()), it_end(measurement.end());
          it != it_end; ++it){

        float_t range;
        if(!this->range(it->second, range)){continue;} // No range entry

        if(!(select_satellite(it->first, receiver_time).is_available())){continue;} // No satellite

        typename base_t::measurement2_t::value_type v = {
            it->first, &(it->second), this}; // prn, measurement, solver
        measurement2.push_back(v);
      }
      base_t::user_pvt(
          res,
          measurement2, receiver_time, user_position_init, receiver_error_init,
          typename base_t::user_pvt_opt_t(good_init, with_velocity));
    }
};

#endif /* __GPS_SOLVER_H__ */
