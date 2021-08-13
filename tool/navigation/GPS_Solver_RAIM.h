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

template <class FloatT, class PVT_BaseT = typename GPS_Solver_Base<FloatT>::user_pvt_t>
struct GPS_PVT_RAIM_LSR : public PVT_BaseT {
  struct info_t {
    bool valid;
    struct slope_t {
      FloatT max;
      typename GPS_Solver_Base<FloatT>::prn_t prn;
    } slope_HV[2];
    FloatT wssr;
    FloatT wssr_sf; ///< for nominal bias consideration, sum(eigen.values(W (I - P)))
    FloatT weight_max;
  };
  info_t FD; ///< Fault detection
};

template <class FloatT, class SolverBaseT = GPS_Solver_Base<FloatT> >
struct GPS_Solver_RAIM_LSR : public SolverBaseT {
  typedef SolverBaseT base_t;
  typedef GPS_Solver_RAIM_LSR<FloatT, SolverBaseT> self_t;
  virtual ~GPS_Solver_RAIM_LSR() {}

#if defined(__GNUC__) && (__GNUC__ < 5)
#define inheritate_type(x) typedef typename base_t::x x;
#else
#define inheritate_type(x) using typename base_t::x;
#endif

  inheritate_type(float_t);
  inheritate_type(matrix_t);

  inheritate_type(xyz_t);

  inheritate_type(geometric_matrices_t);
#undef inheritate_type

  typedef GPS_PVT_RAIM_LSR<float_t, typename base_t::user_pvt_t> user_pvt_t;

  typename base_t::template solver_interface_t<self_t> solve() const {
    return typename base_t::template solver_interface_t<self_t>(*this);
  }

protected:
  virtual bool update_position_solution(
      const geometric_matrices_t &geomat,
      typename GPS_Solver_Base<FloatT>::user_pvt_t &res) const {

    if(!base_t::update_position_solution(geomat, res)){return false;}

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
};

#endif /* __GPS_SOLVER_RAIM_H__ */
