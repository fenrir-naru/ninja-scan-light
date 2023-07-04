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

template <class BaseSolver = GPS_SinglePositioning<double> >
class GPS_Solver_MultiFrequency : public BaseSolver {
  public:
    typedef GPS_Solver_MultiFrequency<BaseSolver> self_t;
    typedef BaseSolver super_t;
  private:
    self_t &operator=(const self_t &);
    GPS_Solver_MultiFrequency(const self_t &);
  public:
#if defined(__GNUC__) && (__GNUC__ < 5)
#define inheritate_type(x) typedef typename super_t::x x;
#else
#define inheritate_type(x) using typename super_t::x;
#endif
    inheritate_type(float_t);
    inheritate_type(space_node_t);
    inheritate_type(range_error_t);
    inheritate_type(measurement_item_set_t);
#undef inheritate_type

    typedef typename GPS_Solver_Base<float_t>::options_t::template merge_t<
        GPS_Solver_MultiFrequency_Options<float_t>, super_t> options_t;

    GPS_Solver_MultiFrequency_Options<float_t> options_frequency;

  public:
    options_t available_options() const {
      return options_t(super_t::available_options());
    }

    options_t available_options(const options_t &opt_wish) const {
      return options_t(super_t::available_options(opt_wish), opt_wish);
    }

    options_t update_options(const options_t &opt_wish){
      return options_t(
          super_t::update_options(opt_wish),
          options_frequency = opt_wish);
    }

    GPS_Solver_MultiFrequency(const space_node_t &sn)
        : super_t(sn), options_frequency() {}

    ~GPS_Solver_MultiFrequency(){}

    struct measurement_items_t : public super_t::measurement_items_t {
      enum {
        PREDEFINED_LAST = super_t::measurement_items_t::MEASUREMENT_ITEMS_PREDEFINED - 1,
#define make_entry(key) \
    L2CM_ ## key, \
    L2CL_ ## key
#define make_entry2(key) \
    make_entry(key), \
    make_entry(key ## _SIGMA)
        make_entry2(PSEUDORANGE),
        make_entry2(CARRIER_PHASE),
        make_entry2(DOPPLER),
        make_entry2(RANGE_RATE),
        make_entry(SIGNAL_STRENGTH_dBHz),
        make_entry(LOCK_SEC),
        make_entry(CARRIER_PHASE_AMBIGUITY_SCALE),
#undef make_entry2
#undef make_entry
        MEASUREMENT_ITEMS_PREDEFINED,
      };
    };

    /**
     * Extract range information from measurement per satellite
     * @param values measurement[prn]
     * @param buf buffer into which range is stored
     * @param error optional argument in which error components of range will be returned
     * @return If valid range information is found, the pointer of buf will be returned; otherwise NULL
     */
    virtual const float_t *range(
        const typename super_t::measurement_t::mapped_type &values, float_t &buf,
        range_error_t *error) const {
      float_t l1, l2;
      const float_t
          *l1_p(super_t::find_value(values, measurement_items_t::L1_PSEUDORANGE, l1)),
          *l2_p(NULL);
      options_frequency.exclude_L2C // if false then look for L2CM and L2CL. L2CM is higher priority than L2CL
          || (l2_p = super_t::find_value(values, measurement_items_t::L2CM_PSEUDORANGE, l2))
          || (l2_p = super_t::find_value(values, measurement_items_t::L2CL_PSEUDORANGE, l2));

      if(error){
        *error = range_error_t::not_corrected;
      }

      if(l1_p){
        if(l2_p && error){ // L1 and L2
          error->unknown_flag &= ~(range_error_t::MASK_IONOSPHERIC);
          error->value[range_error_t::IONOSPHERIC]
              = (l2 - l1) / (space_node_t::gamma_L1_L2 - 1);
          /* @see IS-GPS-200H 20.3.3.3.3.3
           * PR = PR_L1 + (PR_L2 - PR_L1)/(1 - gamma), and PR + error = PR_L1.
           * Therefore, error = (PR_L2 - PR_L1)/(gamma - 1)
           */
        }
        return &(buf = l1);
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

template <class BaseSolver>
const typename GPS_Solver_MultiFrequency<BaseSolver>::measurement_item_set_t
    GPS_Solver_MultiFrequency<BaseSolver>::L2CM = {
#define make_entry(key) \
    GPS_Solver_MultiFrequency<BaseSolver>::measurement_items_t::L2CM_ ## key
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

template <class BaseSolver>
const typename GPS_Solver_MultiFrequency<BaseSolver>::measurement_item_set_t
    GPS_Solver_MultiFrequency<BaseSolver>::L2CL = {
#define make_entry(key) \
    GPS_Solver_MultiFrequency<BaseSolver>::measurement_items_t::L2CL_ ## key
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

#endif /* __GPS_SOLVER_MULTI_FREQUENCY_H__ */
