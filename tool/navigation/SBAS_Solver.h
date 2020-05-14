/**
 * @file SBAS solver
 *
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

#ifndef __SBAS_SOLVER_H__
#define __SBAS_SOLVER_H__

#include "SBAS.h"
#include "GPS_Solver_Base.h"
#include "GPS_Solver.h"

template <class FloatT>
struct SBAS_SinglePositioning_Options : public GPS_Solver_GeneralOptions<FloatT> {
  typedef GPS_Solver_GeneralOptions<FloatT> super_t;

  typename GPS_Solver_Base<FloatT>::options_t::template exclude_prn_t<120, 158> exclude_prn; // SBAS PRN ranges from 120 to 158
  SBAS_SinglePositioning_Options()
      : super_t() {
    // default: SBAS IGP, then broadcasted Klobuchar.
    super_t::ionospheric_models[0] = super_t::IONOSPHERIC_SBAS;
    super_t::ionospheric_models[1] = super_t::IONOSPHERIC_KLOBUCHAR;

    exclude_prn.set(true); // SBAS ranging is default off.
  }
};

template <class FloatT, class SolverBaseT = GPS_Solver_Base<FloatT> >
class SBAS_SinglePositioning : public SolverBaseT {
  private:
    SBAS_SinglePositioning<FloatT> &operator=(const SBAS_SinglePositioning<FloatT> &);
  public:
    typedef SBAS_SinglePositioning<FloatT> self_t;
    typedef SolverBaseT base_t;

#if defined(__GNUC__) && (__GNUC__ < 5)
#define inheritate_type(x) typedef typename base_t::x x;
#else
#define inheritate_type(x) using typename base_t::x;
#endif

    inheritate_type(float_t);
    inheritate_type(prn_t);

    typedef SBAS_SpaceNode<float_t> space_node_t;
    inheritate_type(gps_time_t);
    typedef typename space_node_t::Satellite satellite_t;

    inheritate_type(xyz_t);
    inheritate_type(enu_t);

    inheritate_type(pos_t);

    typedef typename base_t::measurement_t measurement_t;
    typedef typename base_t::range_error_t range_error_t;

    inheritate_type(relative_property_t);
#undef inheritate_type

    typedef typename GPS_Solver_Base<float_t>::options_t::template merge_t<
        SBAS_SinglePositioning_Options<float_t>, base_t> options_t;

    typedef GPS_SpaceNode<float_t> gps_space_node_t;
    const gps_space_node_t *space_node_gps; ///< optional
  protected:
    const space_node_t &_space_node;
    SBAS_SinglePositioning_Options<float_t> _options;

  public:
    const space_node_t &space_node() const {return _space_node;}

    /**
     * Check availability of ionospheric models
     * If a model is completely unavailable, it will be replaced to IONOSPHERIC_SKIP.
     * It implies that when a model has conditional applicability (such as SBAS), it will be retained.
     *
     * @return (int) number of (possibly) available models
     */
    int filter_ionospheric_models(SBAS_SinglePositioning_Options<float_t> &opt) const {
      int available_models(0);
      for(int i(0); i < sizeof(opt.ionospheric_models) / sizeof(opt.ionospheric_models[0]); ++i){
        switch(opt.ionospheric_models[i]){
          case options_t::IONOSPHERIC_KLOBUCHAR:
            // check whether Klobuchar parameters alpha and beta have been already received
            if(space_node_gps && space_node_gps->is_valid_iono()){break;}
            opt.ionospheric_models[i] = options_t::IONOSPHERIC_SKIP;
            continue;
          case options_t::IONOSPHERIC_SBAS:
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
      SBAS_SinglePositioning_Options<float_t> opt(opt_wish);
      filter_ionospheric_models(opt);
      return options_t(base_t::available_options(opt_wish), opt);
    }

    options_t update_options(const options_t &opt_wish){
      filter_ionospheric_models(_options = opt_wish);
      return options_t(base_t::update_options(opt_wish), _options);
    }

    SBAS_SinglePositioning(const space_node_t &sn, const options_t &opt_wish = options_t())
        : base_t(), 
        _space_node(sn), space_node_gps(NULL),
        _options(available_options(opt_wish)) {}

    ~SBAS_SinglePositioning(){}

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

      if(_options.exclude_prn[prn]){return res;}

      float_t range;
      range_error_t range_error;
      if(!this->range(measurement, range, &range_error)){
        return res; // If no range entry, return with weight = 0
      }

      const satellite_t *sat(is_available(prn, time_arrival));
      if(!sat){return res;} // If satellite is unavailable, return with weight = 0

      ///< The following procedure is based on Appendix.S with modification

      range -= receiver_error;

      // Clock correction will be performed in the following constellation()
      if(!(range_error.unknown_flag & range_error_t::SATELLITE_CLOCK)){
        range += range_error.value[range_error_t::SATELLITE_CLOCK];
      }

      // TODO WAAS long term clock correction (2.1.1.4.11)

      // Calculate satellite position and velocity
      typename space_node_t::SatelliteProperties::constellation_t sat_pos_vel(
          sat->ephemeris().constellation(time_arrival, range));

      const xyz_t &sat_pos(sat_pos_vel.position);
      float_t geometric_range(usr_pos.xyz.dist(sat_pos));

      // Calculate residual with Sagnac correction (A.4.4.11)
      res.range_residual = range
          + space_node_t::sagnac_correction(sat_pos, usr_pos.xyz)
          - geometric_range;

      // Setup design matrix
      res.los_neg[0] = -(sat_pos.x() - usr_pos.xyz.x()) / geometric_range;
      res.los_neg[1] = -(sat_pos.y() - usr_pos.xyz.y()) / geometric_range;
      res.los_neg[2] = -(sat_pos.z() - usr_pos.xyz.z()) / geometric_range;

      enu_t relative_pos(enu_t::relative(sat_pos, usr_pos.xyz));

      // Tropospheric (2.1.4.10.3, A.4.2.4)
      res.range_residual += (range_error.unknown_flag & range_error_t::MASK_TROPOSPHERIC)
          ? _space_node.tropo_correction(time_arrival.year(), relative_pos, usr_pos.llh)
          : range_error.value[range_error_t::TROPOSPHERIC];

      // Ionospheric (2.1.4.10.2, A.4.4.10.4)
      if(range_error.unknown_flag & range_error_t::MASK_IONOSPHERIC){
        // Ionospheric model selection, the fall back is no correction
        bool iono_model_hit(false);
        for(int i(0); i < sizeof(_options.ionospheric_models) / sizeof(_options.ionospheric_models[0]); ++i){
          switch(_options.ionospheric_models[i]){
            case options_t::IONOSPHERIC_KLOBUCHAR: // 20.3.3.5.2.5
              res.range_residual += space_node_gps->iono_correction(relative_pos, usr_pos.llh, time_arrival);
              break;
            case options_t::IONOSPHERIC_SBAS: { // 2.1.4.10.2, A.4.4.10.4
              typename space_node_t::IonosphericGridPoints::PointProperty prop(
                  sat->ionospheric_grid_points().iono_correction(relative_pos, usr_pos.llh));
              if(!prop.is_available()){continue;}
              res.range_residual += prop.delay;
              break;
            }
            case options_t::IONOSPHERIC_NTCM_GL: {
              // TODO f_10_7 setup, optimization (mag_model etc.)
              typename gps_space_node_t::pierce_point_res_t pp(gps_space_node_t::pierce_point(relative_pos, usr_pos.llh));
              res.range_residual -= gps_space_node_t::tec2delay(
                  gps_space_node_t::slant_factor(relative_pos)
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
        res.range_residual += range_error.value[range_error_t::IONOSPHERIC];
      }

      // TODO Fast corrections (2.1.1.4.12)

      // TODO Setup weight
      if(std::abs(res.range_residual) > _options.residual_mask){
        // If residual is too big, gently exclude it by decreasing its weight.
        res.weight = 1E-8;
      }else{

        float_t elv(relative_pos.elevation());
        if(elv < _options.elevation_mask){
          res.weight = 0; // exclude it when elevation is less than threshold
        }else{
          // elevation weight based on "GPS実用プログラミング" @see GPS_Solver.h
          res.weight = std::pow(sin(elv)/0.8, 2);
          if(res.weight < 1E-3){res.weight = 1E-3;}
        }
      }

      res.range_corrected = range;

      xyz_t rel_vel(sat_pos_vel.velocity - usr_vel); // Calculate velocity

      // Note: clock rate error is already accounted for in constellation()
      res.rate_relative_neg = res.los_neg[0] * rel_vel.x()
          + res.los_neg[1] * rel_vel.y()
          + res.los_neg[2] * rel_vel.z();

      return res;
    }

    xyz_t *satellite_position(
        const prn_t &prn,
        const gps_time_t &time,
        xyz_t &res) const {

      const satellite_t *sat(is_available(prn, time));
      return sat ? &(res = sat->ephemeris().constellation(time).position) : NULL;
    }
};

#endif /* __SBAS_SOLVER_H__ */
