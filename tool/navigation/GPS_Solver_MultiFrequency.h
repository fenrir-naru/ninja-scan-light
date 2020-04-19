/**
 * @file GPS multi-frequency solver
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

#ifndef __GPS_SOLVER_MULTI_FREQUENCY_H__
#define __GPS_SOLVER_MULTI_FREQUENCY_H__

#include "GPS_Solver.h"

template <class FloatT>
struct GPS_Solver_MultiFrequency_Options {
  bool exclude_L2C;
  GPS_Solver_MultiFrequency_Options() : exclude_L2C(false) {}
};

template <class FloatT, template <class> class BaseSolver = GPS_SinglePositioning>
class GPS_Solver_MultiFrequency : public BaseSolver<FloatT> {
  public:
    typedef GPS_Solver_MultiFrequency<FloatT, BaseSolver> self_t;
    typedef BaseSolver<FloatT> super_t;
  private:
    self_t &operator=(const self_t &);
  public:
    struct options_t
        : public super_t::options_t,
        public GPS_Solver_MultiFrequency_Options<FloatT> {};
    GPS_Solver_MultiFrequency_Options<FloatT> options_frequency;

#if defined(__GNUC__) && (__GNUC__ < 5)
#define inheritate_type(x) typedef typename super_t::x x;
#else
#define inheritate_type(x) using typename super_t::x;
#endif
    inheritate_type(float_t);
    inheritate_type(space_node_t);
    inheritate_type(measurement_item_set_t);
#undef inheritate_type

  public:
    GPS_Solver_MultiFrequency(
        const space_node_t &sn, const options_t &opt_wish = options_t())
        : super_t(sn, opt_wish), options_frequency(opt_wish) {}

    ~GPS_Solver_MultiFrequency(){}

    struct measurement_items_t : public super_t::measurement_items_t {
      enum {
        PREDEFINED_LAST = super_t::measurement_items_t::MEASUREMENT_ITEMS_PREDEFINED - 1,
#define make_entry(key) \
    L2CM_ ## key, \
    L2CM_ ## key ## _SIGMA, \
    L2CL_ ## key, \
    L2CL_ ## key ## _SIGMA
        make_entry(PSEUDORANGE),
        make_entry(CARRIER_PHASE),
        make_entry(DOPPLER),
        make_entry(RANGE_RATE),
#undef make_entry
        MEASUREMENT_ITEMS_PREDEFINED,
      };
    };

    /**
     * Extract range information from measurement per satellite
     * @param values measurement[prn]
     * @param buf buffer into which range is stored
     * @param errors optional argument in which error components of range will be returned
     * @return If valid range information is found, the pointer of buf will be returned; otherwise NULL
     */
    virtual const float_t *range(
        const typename super_t::measurement_t::mapped_type &values, float_t &buf,
        int *errors) const {
      float_t l1, l2;
      const float_t
          *l1_p(super_t::find_value(values, measurement_items_t::L1_PSEUDORANGE, l1)),
          *l2_p(NULL);
      options_frequency.exclude_L2C // if false then look for L2CM and L2CL. L2CM is higher priority than L2CL
          || (l2_p = super_t::find_value(values, measurement_items_t::L2CM_PSEUDORANGE, l2))
          || (l2_p = super_t::find_value(values, measurement_items_t::L2CL_PSEUDORANGE, l2));

      if(errors){
        *errors = (super_t::RANGE_ERROR_RECEIVER_CLOCK
            | super_t::RANGE_ERROR_SATELLITE_CLOCK
            | super_t::RANGE_ERROR_IONOSPHERIC
            | super_t::RANGE_ERROR_TROPOSPHERIC);
      }

      if(l1_p){
        if(l2_p){ // L1 and L2
          if(errors){
            *errors &= ~(super_t::RANGE_ERROR_IONOSPHERIC);
          }
          return &(buf
              = (space_node_t::gamma_L1_L2 * l1 - l2)
                / (space_node_t::gamma_L1_L2 - 1));
        }else{ // L1 only
          return &(buf = l1);
        }
      }
#if 0
      /* TODO L2 only, because rate and deviation without L2 has not
       * yet been implemented.
       */
      else if(l2_p){
        return &(buf = l2);
      }
#endif
      else{ // no range information
        return NULL;
      }
    }

    static const measurement_item_set_t L2CM, L2CL;
};

template <class FloatT, template <class> class BaseSolver>
const typename GPS_Solver_MultiFrequency<FloatT, BaseSolver>::measurement_item_set_t
    GPS_Solver_MultiFrequency<FloatT, BaseSolver>::L2CM = {
#define make_entry(key) { \
    GPS_Solver_MultiFrequency<FloatT, BaseSolver>::measurement_items_t::L2CM_ ## key, \
    GPS_Solver_MultiFrequency<FloatT, BaseSolver>::measurement_items_t::L2CM_ ## key ## _SIGMA}
      make_entry(PSEUDORANGE),
      make_entry(DOPPLER),
      make_entry(CARRIER_PHASE),
      make_entry(RANGE_RATE),
#undef make_entry
    };

template <class FloatT, template <class> class BaseSolver>
const typename GPS_Solver_MultiFrequency<FloatT, BaseSolver>::measurement_item_set_t
    GPS_Solver_MultiFrequency<FloatT, BaseSolver>::L2CL = {
#define make_entry(key) { \
    GPS_Solver_MultiFrequency<FloatT, BaseSolver>::measurement_items_t::L2CL_ ## key, \
    GPS_Solver_MultiFrequency<FloatT, BaseSolver>::measurement_items_t::L2CL_ ## key ## _SIGMA}
      make_entry(PSEUDORANGE),
      make_entry(DOPPLER),
      make_entry(CARRIER_PHASE),
      make_entry(RANGE_RATE),
#undef make_entry
    };

#endif /* __GPS_SOLVER_MULTI_FREQUENCY_H__ */
