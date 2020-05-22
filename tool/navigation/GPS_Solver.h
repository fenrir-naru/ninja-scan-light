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

#include "GPS.h"
#include "GPS_Solver_Base.h"
#include "NTCM.h"

template <class FloatT>
struct GPS_Solver_GeneralOptions {
  FloatT elevation_mask;

  enum ionospheric_model_t {
    IONOSPHERIC_KLOBUCHAR,
    IONOSPHERIC_NTCM_GL,
    IONOSPHERIC_NONE, // which allows no correction
    IONOSPHERIC_MODELS,
    IONOSPHERIC_SKIP = IONOSPHERIC_MODELS, // which means delegating the next slot
  };
  ionospheric_model_t ionospheric_models[IONOSPHERIC_MODELS]; // lower index means having higher priority

  int count_ionospheric_models() const {
    int res(0);
    for(int i(0); i < sizeof(ionospheric_models) / sizeof(ionospheric_models[0]); ++i){
      if(ionospheric_models[i] != IONOSPHERIC_SKIP){res++;}
    }
    return res;
  }

  FloatT f_10_7;

  GPS_Solver_GeneralOptions()
      : elevation_mask(0), f_10_7(-1) {
    for(int i(0); i < sizeof(ionospheric_models) / sizeof(ionospheric_models[0]); ++i){
      ionospheric_models[i] = IONOSPHERIC_SKIP;
    }
    // default: broadcasted Klobuchar parameters are at least required for solution.
    ionospheric_models[0] = IONOSPHERIC_KLOBUCHAR;
  }

  bool insert_ionospheric_model(const ionospheric_model_t &model, const int &index = 0) {
    if((index < 0)
        || (index >= sizeof(ionospheric_models) / sizeof(ionospheric_models[0]))){
      return false;
    }

    // shift, then replace
    for(int i(index), j(i + 1); j < sizeof(ionospheric_models) / sizeof(ionospheric_models[0]); ++i, ++j){
      ionospheric_models[j] = ionospheric_models[i];
    }
    ionospheric_models[index] = model;

    return true;
  }

  /**
   * Flags to invalidate specific satellite
   * Its index will be adjusted for PRN.
   */
  template <int prn_begin, int prn_end>
  struct exclude_prn_t
      : public GPS_Solver_Base<FloatT>::template bit_array_t<prn_end - prn_begin + 1, unsigned int> {
    typedef typename GPS_Solver_Base<FloatT>
        ::template bit_array_t<prn_end - prn_begin + 1, unsigned int> super_t;
    bool operator[](const int &prn) const {
      return super_t::operator[](prn - prn_begin);
    }
    using super_t::set;
    void set(const int &prn, const bool &bit = true) {
      super_t::set(prn - prn_begin, bit);
    }
    using super_t::reset;
    void reset(const int &prn) {set(prn, false);}
  };
};

template <class FloatT>
struct GPS_SinglePositioning_Options : public GPS_Solver_GeneralOptions<FloatT> {
  typename GPS_Solver_GeneralOptions<FloatT>
      ::template exclude_prn_t<1, 32> exclude_prn; // PRN ranges from 1 to 32

  GPS_SinglePositioning_Options() : GPS_Solver_GeneralOptions<FloatT>() {
    exclude_prn.clear();
  }
};

template <class FloatT>
class GPS_SinglePositioning : public GPS_Solver_Base<FloatT> {
  private:
    GPS_SinglePositioning<FloatT> &operator=(const GPS_SinglePositioning<FloatT> &);
  public:
    typedef GPS_SinglePositioning<FloatT> self_t;
    typedef GPS_Solver_Base<FloatT> base_t;

#if defined(__GNUC__) && (__GNUC__ < 5)
#define inheritate_type(x) typedef typename base_t::x x;
#else
#define inheritate_type(x) using typename base_t::x;
#endif

    inheritate_type(float_t);
    inheritate_type(matrix_t);
    inheritate_type(prn_t);

    inheritate_type(space_node_t);
    inheritate_type(gps_time_t);
    typedef typename space_node_t::Satellite satellite_t;

    inheritate_type(xyz_t);
    inheritate_type(llh_t);
    inheritate_type(enu_t);

    inheritate_type(pos_t);

    inheritate_type(prn_obs_t);
    inheritate_type(measurement_t);
    inheritate_type(measurement_items_t);
    inheritate_type(range_error_t);

    typedef GPS_SinglePositioning_Options<float_t> options_t;

    inheritate_type(relative_property_t);
    inheritate_type(geometric_matrices_t);
    inheritate_type(user_pvt_t);
#undef inheritate_type

  protected:
    const space_node_t &_space_node;
    options_t _options;

  public:
    const space_node_t &space_node() const {return _space_node;}

    /**
     * Check availability of ionospheric models
     * If a model is completely unavailable, it will be replaced to IONOSPHERIC_SKIP.
     * It implies that when a model has conditional applicability (such as SBAS), it will be retained.
     *
     * @return (int) number of (possibly) available models
     */
    int filter_ionospheric_models(options_t &opt) const {

      int available_models(0);

      for(int i(0); i < sizeof(opt.ionospheric_models) / sizeof(opt.ionospheric_models[0]); ++i){
        switch(opt.ionospheric_models[i]){
          case options_t::IONOSPHERIC_KLOBUCHAR:
            // check whether Klobuchar parameters alpha and beta have been already received
            if(_space_node.is_valid_iono()){break;}
            opt.ionospheric_models[i] = options_t::IONOSPHERIC_SKIP;
            continue;
          case options_t::IONOSPHERIC_NTCM_GL:
            if(opt.f_10_7 >= 0){break;}
            // check F10.7 has been already configured
            opt.ionospheric_models[i] = options_t::IONOSPHERIC_SKIP;
            continue;
          case options_t::IONOSPHERIC_SKIP:
            continue;
        }
        available_models++;
      }

      return available_models;
    }

    const options_t &available_options() const {
      return _options;
    }

    options_t available_options(options_t opt_wish) const {
      filter_ionospheric_models(opt_wish);
      return opt_wish;
    }

    const options_t &update_options(const options_t &opt_wish){
      return _options = available_options(opt_wish);
    }

    GPS_SinglePositioning(const space_node_t &sn, const options_t &opt_wish = options_t())
        : base_t(), _space_node(sn), _options(available_options(opt_wish)) {}

    ~GPS_SinglePositioning(){}

    struct residual_t {
      float_t &residual;
      float_t &los_neg_x;
      float_t &los_neg_y;
      float_t &los_neg_z;
      float_t &weight;
    };

    /**
     * Get corrected range in accordance with current status
     *
     * @param sat satellite
     * @param range "corrected" pseudo range subtracted by (temporal solution of) receiver clock error in meter
     * @param time_arrival time when signal arrive at receiver
     * @param usr_pos (temporal solution of) user position
     * @param residual calculated residual with line of site vector, and weight;
     * When weight is equal to or less than zero, the calculated results should not be used.
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

      // Clock error correction
      range += ((error.unknown_flag & range_error_t::SATELLITE_CLOCK)
          ? (sat.clock_error(time_arrival, range) * space_node_t::light_speed)
          : error.value[range_error_t::SATELLITE_CLOCK]);

      // Calculate satellite position
      xyz_t sat_pos(sat.position(time_arrival, range));
      float_t geometric_range(usr_pos.xyz.dist(sat_pos));

      // Calculate residual
      residual.residual = range - geometric_range;

      // Setup design matrix
      residual.los_neg_x = -(sat_pos.x() - usr_pos.xyz.x()) / geometric_range;
      residual.los_neg_y = -(sat_pos.y() - usr_pos.xyz.y()) / geometric_range;
      residual.los_neg_z = -(sat_pos.z() - usr_pos.xyz.z()) / geometric_range;

      enu_t relative_pos(enu_t::relative(sat_pos, usr_pos.xyz));

      if(error.unknown_flag & range_error_t::MASK_IONOSPHERIC){
        // Ionospheric model selection, the fall back is no correction
        bool iono_model_hit(false);
        for(int i(0); i < sizeof(_options.ionospheric_models) / sizeof(_options.ionospheric_models[0]); ++i){
          switch(_options.ionospheric_models[i]){
            case options_t::IONOSPHERIC_KLOBUCHAR:
              residual.residual += _space_node.iono_correction(relative_pos, usr_pos.llh, time_arrival);
              break;
            case options_t::IONOSPHERIC_NTCM_GL: {
              // TODO f_10_7 setup, optimization (mag_model etc.)
              typename space_node_t::pierce_point_res_t pp(_space_node.pierce_point(relative_pos, usr_pos.llh));
              residual.residual -= space_node_t::tec2delay(
                  _space_node.slant_factor(relative_pos)
                  * NTCM_GL_Generic<float_t>::tec_vert(
                      pp.latitude, pp.longitude,
                      time_arrival.year(), _options.f_10_7));
              break;
            }
            case options_t::IONOSPHERIC_NONE:
              break;
            default:
              continue;
          }
          iono_model_hit = true;
          break;
        }
      }else{
        residual.residual += error.value[range_error_t::IONOSPHERIC];
      }

      // Tropospheric
      residual.residual += (error.unknown_flag & range_error_t::MASK_TROPOSPHERIC)
          ? _space_node.tropo_correction(relative_pos, usr_pos.llh)
          : error.value[range_error_t::TROPOSPHERIC];

      // Setup weight
      if(std::abs(residual.residual) > 30.0){
        // If residual is too big, gently exclude it by decreasing its weight.
        residual.weight = 1E-8;
      }else{

        float_t elv(relative_pos.elevation());
        if(elv < _options.elevation_mask){
          residual.weight = 0; // exclude it when elevation is less than threshold
        }else{
          // elevation weight based on "GPS実用プログラミング"
          residual.weight = std::pow(sin(elv)/0.8, 2);
          if(residual.weight < 1E-3){residual.weight = 1E-3;}
        }
      }

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

      xyz_t rel_vel(sat.velocity(time_arrival, range) - usr_vel); // Calculate velocity
      return los_neg_x * rel_vel.x()
          + los_neg_y * rel_vel.y()
          + los_neg_z * rel_vel.z()
          + sat.clock_error_dot(time_arrival, range) * space_node_t::light_speed; // considering clock rate error
    }

    /**
     * Check availability of a satellite with which observation is associated
     *
     * @param prn satellite number
     * @param receiver_time receiver time
     * @return (const satellite_t *) If available, const pointer to satellite is returned,
     * otherwise NULL.
     */
    const satellite_t *is_available(
        const typename space_node_t::satellites_t::key_type &prn,
        const gps_time_t &receiver_time) const {

      if(_options.exclude_prn[prn]){return NULL;}

      const typename space_node_t::satellites_t &sats(_space_node.satellites());
      const typename space_node_t::satellites_t::const_iterator it_sat(sats.find(prn));
      if((it_sat == sats.end()) // has ephemeris?
          || (!it_sat->second.ephemeris().is_valid(receiver_time))){ // valid ephemeris?
        return NULL;
      }

      return &(it_sat->second);
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
        return res; // If no range entry, return with weight = 0
      }

      const satellite_t *sat(is_available(prn, time_arrival));
      if(!sat){return res;} // If satellite is unavailable, return with weight = 0

      residual_t residual = {
        res.range_residual,
        res.los_neg[0], res.los_neg[1], res.los_neg[2],
        res.weight,
      };

      res.range_corrected = range_corrected(
          *sat, range - receiver_error, time_arrival,
          usr_pos, residual, range_error);
      res.rate_relative_neg = rate_relative_neg(*sat, res.range_corrected, time_arrival, usr_vel,
          res.los_neg[0], res.los_neg[1], res.los_neg[2]);

      return res;

    }

    /**
     * Calculate User position/velocity with hint
     *
     * @param measurement PRN, pseudo-range, pseudo-range rate information
     * @param receiver_time receiver time at measurement
     * @param user_position_init initial solution of user position in XYZ meters and LLH
     * @param receiver_error_init initial solution of receiver clock error in meters
     * @param good_init if true, initial position and clock error are goodly guessed.
     * @param with_velocity if true, perform velocity estimation.
     * @return calculation results and matrices used for calculation
     * @see update_ephemeris(), register_ephemeris
     */
    user_pvt_t solve_user_pvt(
        const measurement_t &measurement,
        const gps_time_t &receiver_time,
        const pos_t &user_position_init,
        const float_t &receiver_error_init,
        const bool &good_init = true,
        const bool &with_velocity = true) const {

      user_pvt_t res;
      res.receiver_time = receiver_time;

      if(_options.count_ionospheric_models() == 0){
        res.error_code = user_pvt_t::ERROR_INVALID_IONO_MODEL;
        return res;
      }

      // 1. Check satellite availability for range. The results will be cached.

      typedef std::vector<std::pair<
          std::pair<prn_t, const satellite_t *>,
          std::pair<float_t, range_error_t> > > sat_range_cache_t;
      sat_range_cache_t sat_range_cache; // [[prn, *sat_GPS], [range, range_error]]
      sat_range_cache.reserve(measurement.size());

      for(typename measurement_t::const_iterator it(measurement.begin());
          it != measurement.end();
          ++it){

        float_t range;
        range_error_t range_error;
        if(!this->range(it->second, range, &range_error)){continue;} // No range entry

        sat_range_cache.push_back(std::make_pair(
            std::make_pair(it->first, is_available(it->first, receiver_time)),
            std::make_pair(range, range_error)));
      }

      // 2. Position calculation

      res.user_position = user_position_init;
      res.receiver_error = receiver_error_init;

      gps_time_t time_arrival(
          receiver_time - (res.receiver_error / space_node_t::light_speed));

      geometric_matrices_t geomat(sat_range_cache.size());
      typedef std::vector<std::pair<
          typename sat_range_cache_t::value_type::first_type,
          float_t> > sat_range_t;
      sat_range_t sat_range_corrected;
      sat_range_corrected.reserve(sat_range_cache.size());

      // If initialization is not appropriate, more iteration will be performed.
      bool converged(false);
      for(int i(good_init ? 0 : -2); i < 10; i++){

        sat_range_corrected.clear();
        unsigned int j(0);
        res.used_satellite_mask.clear();

        const bool coarse_estimation(i <= 0);
        for(typename sat_range_cache_t::const_iterator it(sat_range_cache.begin());
            it != sat_range_cache.end();
            ++it){

          residual_t residual = {
            geomat.delta_r(j, 0),
            geomat.G(j, 0), geomat.G(j, 1), geomat.G(j, 2),
            geomat.W(j, j)
          };

          const satellite_t *sat(it->first.second);
          float_t range(it->second.first);

          if(sat){
            static const range_error_t coarse_mode = {
                range_error_t::MASK_SATELLITE_CLOCK, {0}};
            range = range_corrected(*sat,
                range - res.receiver_error, time_arrival,
                res.user_position,
                residual,
                (coarse_estimation ? coarse_mode : it->second.second));
          }else{
            // TODO placeholder for non-GPS satellites
            residual.weight = 0;
          }

          if(residual.weight <= 0){
            continue; // intentionally excluded satellite
          }else{
            res.used_satellite_mask.set(it->first.first);
          }

          ++j;

          if(coarse_estimation){
            residual.weight = 1;
          }else{
            sat_range_corrected.push_back(std::make_pair(it->first, range));
          }
        }

        res.used_satellites = j;

        if(false){ // debug
          for(typename sat_range_cache_t::const_iterator it(sat_range_cache.begin());
              it != sat_range_cache.end();
              ++it){
            std::cerr << "PRN:" << it->first.first << " => "
                << it->second.first;
            if(it->first.second){
              std::cerr << " @ Ephemeris: t_oc => "
                  << it->first.second->ephemeris().WN << "w "
                  << it->first.second->ephemeris().t_oc << " +/- "
                  << (it->first.second->ephemeris().fit_interval / 2);
            }
            std::cerr << std::endl;
          }
          typename geometric_matrices_t::partial_t geomat_filled(geomat.partial(res.used_satellites));
          std::cerr << "G:" << geomat_filled.G << std::endl;
          std::cerr << "W:" << geomat_filled.W << std::endl;
          std::cerr << "delta_r:" << geomat_filled.delta_r << std::endl;
        }

        if(res.used_satellites < 4){
          res.error_code = user_pvt_t::ERROR_INSUFFICIENT_SATELLITES;
          return res;
        }

        try{
          // Least square
          matrix_t delta_x(geomat.partial(res.used_satellites).least_square());

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
        res.update_DOP(geomat.partial(res.used_satellites).C());
      }catch(std::exception &e){
        res.error_code = user_pvt_t::ERROR_DOP;
        return res;
      }

      if(!with_velocity){
        res.error_code = user_pvt_t::ERROR_VELOCITY_SKIPPED;
        return res;
      }

      // 3. Check consistency between range and rate for velocity calculation,
      // then, assign design and weight matrices
      geometric_matrices_t geomat2(sat_range_corrected.size());
      int i_range(0), i_rate(0);

      for(typename sat_range_t::const_iterator it(sat_range_corrected.begin());
          it != sat_range_corrected.end();
          ++it, ++i_range){

        float_t rate;
        if(!this->rate(
            measurement.find(it->first.first)->second, // const version of measurement[PRN]
            rate)){continue;}

        // Copy design matrix
        geomat2.copy_G_W_row(geomat, i_range, i_rate);
        static const xyz_t zero(0, 0, 0);

        // Update range rate by subtracting LOS satellite velocity with design matrix G
        geomat2.delta_r(i_rate, 0) = rate
            + rate_relative_neg(
                *(it->first.second), // satellite
                it->second, // range
                time_arrival,
                zero, // user velocity to be estimated is temporary zero
                geomat2.G(i_rate, 0), geomat2.G(i_rate, 1), geomat2.G(i_rate, 2)); // LOS

        ++i_rate;
      }

      if(i_rate < 4){
        res.error_code = user_pvt_t::ERROR_VELOCITY_INSUFFICIENT_SATELLITES;
        return res;
      }

      // 4. Calculate velocity
      try{
        // Least square
        matrix_t sol(geomat2.partial(i_rate).least_square());

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
};

#endif /* __GPS_SOLVER_H__ */
