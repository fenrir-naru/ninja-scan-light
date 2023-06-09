/**
 * @file GLONASS solver
 *
 */

/*
 * Copyright (c) 2022, M.Naruoka (fenrir)
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

#ifndef __GLONASS_SOLVER_H__
#define __GLONASS_SOLVER_H__

#include "GLONASS.h"
#include "GPS.h"
#include "GPS_Solver_Base.h"
#include "GPS_Solver.h"

#include <cstddef>

template <class FloatT>
struct GLONASS_SinglePositioning_Options : public GPS_Solver_GeneralOptions<FloatT> {
  typedef GPS_Solver_GeneralOptions<FloatT> super_t;

  typename GPS_Solver_Base<FloatT>::options_t::template exclude_prn_t<1, 24> exclude_prn; // GLONASS svid ranges from 1 to 24
  GLONASS_SinglePositioning_Options()
      : super_t(), exclude_prn() {
    exclude_prn.set(true); // GLONASS ranging is default off.
  }
};

template <class FloatT, class SolverBaseT = GPS_Solver_Base<FloatT> >
class GLONASS_SinglePositioning : public SolverBaseT {
  public:
    typedef GLONASS_SinglePositioning<FloatT, SolverBaseT> self_t;
    typedef SolverBaseT base_t;
    typedef SolverBaseT super_t;
  private:
    self_t &operator=(const self_t &);
    GLONASS_SinglePositioning(const self_t &);

  public:
#if defined(__GNUC__) && (__GNUC__ < 5)
#define inheritate_type(x) typedef typename base_t::x x;
#else
#define inheritate_type(x) using typename base_t::x;
#endif

    inheritate_type(float_t);
    inheritate_type(prn_t);

    typedef GLONASS_SpaceNode<float_t> space_node_t;
    inheritate_type(gps_time_t);

    inheritate_type(xyz_t);
    inheritate_type(enu_t);

    inheritate_type(pos_t);

    typedef typename base_t::measurement_t measurement_t;
    typedef typename base_t::satellite_t satellite_t;
    typedef typename base_t::range_error_t range_error_t;
    typedef typename base_t::range_corrector_t range_corrector_t;
    typedef typename base_t::range_correction_t range_correction_t;

    inheritate_type(relative_property_t);
    typedef typename super_t::measurement_items_t measurement_items_t;
#undef inheritate_type

    static const typename base_t::measurement_item_set_t L1OF;

    typedef typename GPS_Solver_Base<float_t>::options_t::template merge_t<
        GLONASS_SinglePositioning_Options<float_t>, base_t> options_t;

  protected:
    GLONASS_SinglePositioning_Options<float_t> _options;

  public:
    struct satellites_t {
      const void *impl;
      satellite_t (*impl_select)(const void *, const prn_t &, const gps_time_t &);
      inline satellite_t select(const prn_t &prn, const gps_time_t &receiver_time) const {
        return impl_select(impl, prn, receiver_time);
      }
      static satellite_t select_broadcast(
          const void *ptr, const prn_t &prn, const gps_time_t &receiver_time){
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
            return sat(ptr).constellation(t_tx, dt_transit).position;
          }
          static xyz_t velocity(const void *ptr, const gps_time_t &t_tx, const float_t &dt_transit) {
            return sat(ptr).constellation(t_tx, dt_transit).velocity;
          }
          static float_t clock_error(const void *ptr, const gps_time_t &t_tx) {
            return sat(ptr).clock_error(t_tx);
          }
          static float_t clock_error_dot(const void *ptr, const gps_time_t &t_tx) {
            return sat(ptr).clock_error_dot(t_tx);
          }
          static float_t range_sigma(const void *ptr, const gps_time_t &t_tx) {
            return sat(ptr).ephemeris().F_T;
          }
        };
        satellite_t res = {
            &(it_sat->second), &(it_sat->second), // position, time
            impl_t::position, impl_t::velocity,
            impl_t::clock_error, impl_t::clock_error_dot,
            &(it_sat->second), impl_t::range_sigma, NULL}; // error model
        return res;
      }
      satellites_t(const space_node_t &sn)
          : impl(&sn), impl_select(select_broadcast) {}
    } satellites;

    range_correction_t ionospheric_correction, tropospheric_correction;

    options_t available_options() const {
      return options_t(base_t::available_options(), _options);
    }

    options_t available_options(const options_t &opt_wish) const {
      GLONASS_SinglePositioning_Options<float_t> opt(opt_wish);
      return options_t(base_t::available_options(opt_wish), opt);
    }

    options_t update_options(const options_t &opt_wish){
      _options = opt_wish;
      return options_t(base_t::update_options(opt_wish), _options);
    }

    GLONASS_SinglePositioning(const space_node_t &sn, const options_t &opt_wish = options_t())
        : base_t(), _options(available_options(opt_wish)),
        satellites(sn),
        ionospheric_correction(), tropospheric_correction() {

      // default ionospheric correction: no correction
      ionospheric_correction.push_front(&base_t::no_correction);

      // default troposheric correction: no correction
      tropospheric_correction.push_front(&base_t::no_correction);
    }

    ~GLONASS_SinglePositioning(){}

    virtual const float_t *rate(
        const typename measurement_t::mapped_type &values, float_t &buf) const {
      const float_t *res;
      if((res = super_t::find_value(values, measurement_items_t::L1_RANGE_RATE, buf))){
        return res;
      }
      // Fall back to doppler * frequency
      float_t doppler, freq;
      if(super_t::find_value(values, measurement_items_t::L1_DOPPLER, doppler)
          && super_t::find_value(values, measurement_items_t::L1_FREQUENCY, freq)){
        return &(buf = -doppler * (space_node_t::light_speed / freq));
      }
      return NULL;
    }

    virtual const float_t *rate_sigma(
        const typename measurement_t::mapped_type &values, float_t &buf) const {
      const float_t *res;
      if((res = super_t::find_value(values, measurement_items_t::L1_RANGE_RATE_SIGMA, buf))){
        return res;
      }
      // Fall back to doppler * frequency
      float_t doppler, freq;
      if(super_t::find_value(values, measurement_items_t::L1_DOPPLER_SIGMA, doppler)
          && super_t::find_value(values, measurement_items_t::L1_FREQUENCY, freq)){
        return &(buf = doppler * (space_node_t::light_speed / freq));
      }
      return NULL;
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
      prn_t prn_(prn & 0xFF);
      if(_options.exclude_prn[prn_]){return satellite_t::unavailable();}
      return satellites.select(prn_, receiver_time);
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

      satellite_t sat(select_satellite(prn, time_arrival));
      if(!sat.is_available()){return res;} // If satellite is unavailable, return with weight = 0

      range -= receiver_error;

      static const float_t &c(space_node_t::light_speed);

      // Clock correction, which will be considered in the next constellation()
      // as extra transmission time by using extra psuedo range.
      if(range_error.unknown_flag & range_error_t::SATELLITE_CLOCK){
        range += (sat.clock_error(time_arrival - range / c) * c);
      }else{
        range += range_error.value[range_error_t::SATELLITE_CLOCK];
      }

      // Calculate satellite position
      float_t dt_transit(range / c);
      gps_time_t t_tx(time_arrival - dt_transit);
      xyz_t sat_pos(sat.position(t_tx, dt_transit));
      float_t geometric_range(usr_pos.xyz.distance(sat_pos));

      // Calculate residual
      res.range_residual = range - geometric_range;

      // Setup design matrix
      res.los_neg[0] = -(sat_pos.x() - usr_pos.xyz.x()) / geometric_range;
      res.los_neg[1] = -(sat_pos.y() - usr_pos.xyz.y()) / geometric_range;
      res.los_neg[2] = -(sat_pos.z() - usr_pos.xyz.z()) / geometric_range;

      enu_t relative_pos(enu_t::relative(sat_pos, usr_pos.xyz));

      // Tropospheric
      res.range_residual += (range_error.unknown_flag & range_error_t::MASK_TROPOSPHERIC)
          ? tropospheric_correction(time_arrival, usr_pos, relative_pos)
          : range_error.value[range_error_t::TROPOSPHERIC];

      // Ionospheric
      if(range_error.unknown_flag & range_error_t::MASK_IONOSPHERIC){
        float_t freq(space_node_t::L1_frequency_base); // ch.0 is fallback
        super_t::find_value(measurement, measurement_items_t::L1_FREQUENCY, freq);
        // ionospheric models are assumed to work with GPS L1
        res.range_residual +=
            (ionospheric_correction(time_arrival, usr_pos, relative_pos)
              * GPS_SpaceNode<float_t>::gamma_per_L1(freq));
      }else{
        res.range_residual += range_error.value[range_error_t::IONOSPHERIC];
      }

      // Setup range standard deviation, whose reciprocal square is used as weight
      res.range_sigma = 1E+4; // sufficiently big value, 1E4 [m]
      do{
        // If residual is too big, gently exclude it.
        if(std::abs(res.range_residual) > _options.residual_mask){break;}

        float_t elv(relative_pos.elevation());
        if(elv < _options.elevation_mask){
          res.range_sigma = 0; // exclude it when elevation is less than threshold
          break;
        }

        res.range_sigma = sat.range_sigma(t_tx);

        /* elevation weight based on "GPS実用プログラミング"
         * elevation[deg] :   90    53    45    30    15    10    5
         * sf_sigma(k)    :   0.80  1.00  1.13  1.60  3.09  4.61  9.18
         * weight(k^-2)   :   1.56  1.00  0.78  0.39  0.10  0.05  0.01
         */
        static const float_t max_sf(10);
        static const float_t elv_limit(std::asin(0.8/max_sf)); // limit
        res.range_sigma *= (elv > elv_limit) ? (0.8 / sin(elv)) : max_sf;
      }while(false);

      res.range_corrected = range;

      xyz_t rel_vel(sat.velocity(t_tx, dt_transit) - usr_vel); // Calculate velocity

      res.rate_relative_neg = res.los_neg[0] * rel_vel.x()
          + res.los_neg[1] * rel_vel.y()
          + res.los_neg[2] * rel_vel.z()
          + sat.clock_error_dot(t_tx) * c;

      return res;
    }
};

template <class FloatT, class SolverBaseT>
const typename SolverBaseT::measurement_item_set_t
    GLONASS_SinglePositioning<FloatT, SolverBaseT>::L1OF
      = SolverBaseT::L1CA;

#endif /* __GLONASS_SOLVER_H__ */
