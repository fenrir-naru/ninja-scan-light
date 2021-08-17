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

#include "param/bit_array.h"

#include "GPS.h"
#include "GPS_Solver_Base.h"
#include "NTCM.h"

template <class FloatT>
struct GPS_Solver_GeneralOptions {
  FloatT elevation_mask;
  FloatT residual_mask;

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
      : elevation_mask(0), // elevation mask default is 0 [deg]
      residual_mask(30), // range residual mask is 30 [m]
      f_10_7(-1) {
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
  private:
    GPS_SinglePositioning<FloatT> &operator=(const GPS_SinglePositioning<FloatT> &);
  public:
    typedef GPS_SinglePositioning<FloatT> self_t;
    typedef SolverBaseT base_t;

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
    typedef typename space_node_t::Satellite satellite_t;

    inheritate_type(xyz_t);
    inheritate_type(llh_t);
    inheritate_type(enu_t);

    inheritate_type(pos_t);

    inheritate_type(prn_obs_t);
    typedef typename base_t::measurement_t measurement_t;
    inheritate_type(measurement_items_t);
    typedef typename base_t::range_error_t range_error_t;

    inheritate_type(relative_property_t);
    inheritate_type(geometric_matrices_t);
    inheritate_type(user_pvt_t);
#undef inheritate_type

    typedef typename GPS_Solver_Base<float_t>::options_t::template merge_t<
        GPS_SinglePositioning_Options<float_t>, base_t> options_t;

  protected:
    const space_node_t &_space_node;
    GPS_SinglePositioning_Options<float_t> _options;

  public:
    const space_node_t &space_node() const {return _space_node;}

    /**
     * Check availability of ionospheric models
     * If a model is completely unavailable, it will be replaced to IONOSPHERIC_SKIP.
     * It implies that when a model has conditional applicability (such as SBAS), it will be retained.
     *
     * @return (int) number of (possibly) available models
     */
    int filter_ionospheric_models(GPS_SinglePositioning_Options<float_t> &opt) const {

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

    options_t available_options() const {
      return options_t(base_t::available_options(), _options);
    }

    options_t available_options(const options_t &opt_wish) const {
      GPS_SinglePositioning_Options<float_t> opt(opt_wish);
      filter_ionospheric_models(opt);
      return options_t(base_t::available_options(opt_wish), opt);
    }

    options_t update_options(const options_t &opt_wish){
      filter_ionospheric_models(_options = opt_wish);
      return options_t(base_t::update_options(opt_wish), _options);
    }

    GPS_SinglePositioning(const space_node_t &sn)
        : base_t(), _space_node(sn), _options() {
      filter_ionospheric_models(_options);
    }

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
      if(std::abs(residual.residual) > _options.residual_mask){
        // If residual is too big, gently exclude it by decreasing its weight.
        residual.weight = 1E-8;
      }else{

        float_t elv(relative_pos.elevation());
        if(elv < _options.elevation_mask){
          residual.weight = 0; // exclude it when elevation is less than threshold
        }else{
          /* elevation weight based on "GPS実用プログラミング"
           * elevation[deg] :   90    53    45    30    15    10    5
           * sigma(s)       :   0.80  1.00  1.13  1.60  3.09  4.61  9.18
           * weight(s^-2)   :   1.56  1.00  0.78  0.39  0.10  0.05  0.01
           */
          residual.weight = std::pow(sin(elv)/0.8, 2); // weight=1 @ elv=53[deg]
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

      if(_options.count_ionospheric_models() == 0){
        res.error_code = user_pvt_t::ERROR_INVALID_IONO_MODEL;
        return;
      }

      typename base_t::measurement2_t measurement2;
      measurement2.reserve(measurement.size());
      for(typename measurement_t::const_iterator it(measurement.begin()), it_end(measurement.end());
          it != it_end; ++it){

        float_t range;
        if(!this->range(it->second, range)){continue;} // No range entry

        if(!is_available(it->first, receiver_time)){continue;} // No satellite

        typename base_t::measurement2_t::value_type v = {
            it->first, &(it->second), this}; // prn, measurement, solver
        measurement2.push_back(v);
      }
      base_t::user_pvt(
          res,
          measurement2, receiver_time, user_position_init, receiver_error_init,
          good_init, with_velocity);
    }

    xyz_t *satellite_position(
        const prn_t &prn,
        const gps_time_t &time,
        xyz_t &res) const {

      const satellite_t *sat(is_available(prn, time));
      return sat ? &(res = sat->position(time)) : NULL;
    }
};

#endif /* __GPS_SOLVER_H__ */
