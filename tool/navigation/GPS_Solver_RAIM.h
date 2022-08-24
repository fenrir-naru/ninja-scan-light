/**
 * @file GPS solver with RAIM (Receiver Autonomous Integrity Monitoring)
 */
/*
 * Copyright (c) 2021, M.Naruoka (fenrir)
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

#ifndef __GPS_SOLVER_RAIM_H__
#define __GPS_SOLVER_RAIM_H__

#include "GPS_Solver_Base.h"

#include <algorithm>
#include <deque>

template <class FloatT, class PVT_BaseT = typename GPS_Solver_Base<FloatT>::user_pvt_t>
struct GPS_PVT_RAIM_LSR : public PVT_BaseT {
  struct detection_t {
    bool valid;
    struct slope_t {
      FloatT max;
      typename GPS_Solver_Base<FloatT>::prn_t prn;
    } slope_HV[2];
    FloatT wssr;
    FloatT wssr_sf; ///< for nominal bias consideration, sum(eigen.values(W (I - P)))
    FloatT weight_max;
  } FD; ///< Fault detection
  struct exclusion_t : public detection_t {
    typename GPS_Solver_Base<FloatT>::prn_t excluded;
    typename GPS_Solver_Base<FloatT>::pos_t user_position;
    typename GPS_Solver_Base<FloatT>::float_t receiver_error;
    typename GPS_Solver_Base<FloatT>::user_pvt_t::dop_t dop;
  } FDE_min, FDE_2nd; ///< Fault exclusion
};

template <class FloatT>
struct GPS_Solver_RAIM_LSR_Options {
  bool skip_exclusion;
  GPS_Solver_RAIM_LSR_Options() : skip_exclusion(false) {}
};

/*
 * Comment on implementation of protection level (PL) calculation
 *
 * To calculate PL, WSSR threshold is firstly required.
 * WSSR threshold is a function of P_fa (false alarm), DoF (degree of freedom)
 * corresponding to number of usable satellites minus 4, and nominal biases.
 * If nominal biases are configured as zeros, the threshold can be implemented
 * as a map whose key is DoF, without online calculation of CFD of chi-squred distribution.
 * However, in order to consider nominal biases, online calculation is required,
 * and chi-squred distribution also must be implemented.
 * Therefore, currently, PL is assumed to be calculated outside this program,
 * which can be performed with wssr, wssr_sf, slope_HV.
 */

template <class FloatT, class SolverBaseT = GPS_Solver_Base<FloatT> >
struct GPS_Solver_RAIM_LSR : public SolverBaseT {
  typedef SolverBaseT super_t;
  typedef GPS_Solver_RAIM_LSR<FloatT, SolverBaseT> self_t;
  virtual ~GPS_Solver_RAIM_LSR() {}

#if defined(__GNUC__) && (__GNUC__ < 5)
#define inheritate_type(x) typedef typename super_t::x x;
#else
#define inheritate_type(x) using typename super_t::x;
#endif

  inheritate_type(float_t);
  inheritate_type(matrix_t);

  inheritate_type(gps_time_t);
  inheritate_type(pos_t);

  inheritate_type(geometric_matrices_t);
  inheritate_type(measurement2_t);
#undef inheritate_type

  typedef typename GPS_Solver_Base<float_t>::options_t::template merge_t<
      GPS_Solver_RAIM_LSR_Options<float_t>, super_t> options_t;

protected:
  GPS_Solver_RAIM_LSR_Options<float_t> _options;

public:
  options_t available_options() const {
    return options_t(super_t::available_options(), _options);
  }

  options_t available_options(const options_t &opt_wish) const {
    GPS_Solver_RAIM_LSR_Options<float_t> opt(opt_wish);
    return options_t(super_t::available_options(opt_wish), opt);
  }

  options_t update_options(const options_t &opt_wish){
    _options = opt_wish;
    return options_t(super_t::update_options(opt_wish), _options);
  }

  typedef GPS_PVT_RAIM_LSR<float_t, typename super_t::user_pvt_t> user_pvt_t;

  typename super_t::template solver_interface_t<self_t> solve() const {
    return typename super_t::template solver_interface_t<self_t>(*this);
  }

protected:
  typedef GPS_Solver_Base<FloatT> base_t;
  bool update_position_solution( // overriding function
      const geometric_matrices_t &geomat,
      typename base_t::user_pvt_t &res) const {

    if(!super_t::update_position_solution(geomat, res)){return false;}

    user_pvt_t &pvt(static_cast<user_pvt_t &>(res));
    if(!(pvt.FD.valid = (pvt.used_satellites >= 5))){return true;}

    // Perform least square again for Fault Detection
    matrix_t S, W_dash;
    typename geometric_matrices_t::partial_t geomat2(geomat.partial(res.used_satellites));
    geomat2.least_square(S);
    pvt.FD.wssr = geomat2.wssr_S(S, &W_dash);
    pvt.FD.wssr_sf = W_dash.trace();
    pvt.FD.weight_max = *std::max_element(geomat2.W.cbegin(), geomat2.W.cend());
    matrix_t slope_HV(geomat2.slope_HV(S, res.user_position.ecef2enu()));
    std::vector<int> prn_list(res.used_satellite_mask.indices_one());
    for(unsigned i(0); i < 2; ++i){ // horizontal, vertical
      typename matrix_t::partial_t slope_HV_i(slope_HV.partial(slope_HV.rows(), 1, 0, i));
      typename matrix_t::partial_t::const_iterator it(
          std::max_element(slope_HV_i.cbegin(), slope_HV_i.cend()));
      unsigned int row(it.row());
      pvt.FD.slope_HV[i].max = *it;
      pvt.FD.slope_HV[i].prn = (typename GPS_Solver_Base<FloatT>::prn_t)prn_list[row];
    }

    return true;
  }

public:
  using super_t::user_pvt;
protected:
  void user_pvt( // overriding function
      typename base_t::user_pvt_t &res,
      const measurement2_t &measurement,
      const gps_time_t &receiver_time,
      const pos_t &user_position_init,
      const float_t &receiver_error_init,
      const typename base_t::user_pvt_opt_t &opt
          = typename base_t::user_pvt_opt_t()) const {

    user_pvt_t &pvt(static_cast<user_pvt_t &>(res));
    pvt.FD.valid = pvt.FDE_min.valid = pvt.FDE_2nd.valid = false;

    // Solution with full satellites
    super_t::user_pvt(res,
        measurement, receiver_time,
        user_position_init, receiver_error_init,
        opt);

    if(_options.skip_exclusion
        || !pvt.position_solved()
        || (!pvt.FD.valid)
        || (pvt.used_satellites < 6)){return;}

    // Generate full set
    std::deque<typename measurement2_t::value_type> fullset;
    for(typename measurement2_t::const_iterator it(measurement.begin()), it_end(measurement.end());
        it != it_end; ++it){
      if(!pvt.used_satellite_mask[it->prn]){continue;}
      fullset.push_back(*it);
    }

    // Subset calculation for Fault Exclusion
    pvt.FDE_min.wssr = pvt.FDE_2nd.wssr = pvt.FD.wssr;
    user_pvt_t pvt_FDE(pvt);
    typename base_t::user_pvt_opt_t opt_subset(true, false); // good init, without velocity
    for(unsigned int i(0); i < pvt.used_satellites - 1;
        ++i, fullset.push_back(fullset.front()), fullset.pop_front()){
      pvt_FDE.FD.valid = false;
      measurement2_t subset(fullset.begin(), fullset.end() - 1);
      super_t::user_pvt(pvt_FDE,
          subset, receiver_time,
          pvt.user_position, pvt.receiver_error,
          opt_subset);

      if(!pvt_FDE.FD.valid){continue;}
      switch(pvt_FDE.error_code){
        case user_pvt_t::ERROR_NO:
        case user_pvt_t::ERROR_VELOCITY_SKIPPED:
          break;
        default:
          continue;
      }

      typename user_pvt_t::exclusion_t *target(NULL);
      if(pvt_FDE.FD.wssr < pvt.FDE_min.wssr){
        pvt.FDE_2nd = pvt.FDE_min;
        target = &pvt.FDE_min;
      }else if(pvt_FDE.FD.wssr < pvt.FDE_2nd.wssr){
        target = &pvt.FDE_2nd;
      }else{
        continue;
      }
      (typename user_pvt_t::detection_t &)(*target) = pvt_FDE.FD;
      target->excluded = fullset.back().prn;
      target->user_position = pvt_FDE.user_position;
      target->receiver_error = pvt_FDE.receiver_error;
      target->dop = pvt_FDE.dop;
    }
  }
};

#endif /* __GPS_SOLVER_RAIM_H__ */
