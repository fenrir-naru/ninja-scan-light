/*
 *  INS_GPS_Factory.h, header file to build up INS/GPS integrated
navigation algorithm.
 *  Copyright (C) 2017 M.Naruoka (fenrir)
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __INS_GPS_FACTORY_H__
#define __INS_GPS_FACTORY_H__

#include "INS.h"
#include "INS_EGM.h"
#include "Filtered_INS2.h"
#include "INS_GPS2.h"
#include "BiasEstimation.h"

template <class PureINS = INS<>, class Options = void>
struct INS_GPS_Factory {

  template <class T, class U = void>
  struct option_t {
    typedef T next_t;
    typedef typename next_t::ins_t ins_t;
    template <class INS_Type>
    struct filtered_ins_t {
      typedef typename next_t::template filtered_ins_t<INS_Type>::res_t res_t;
    };
    template <class FINS_Type>
    struct ins_gps_t {
      typedef typename next_t::template ins_gps_t<FINS_Type>::res_t res_t;
    };
  };
  template <class U>
  struct option_t<void, U> {
    typedef PureINS ins_t;
    template <class INS_Type>
    struct filtered_ins_t {
      typedef Filtered_INS2<INS_Type> res_t;
    };
    template <class FINS_Type>
    struct ins_gps_t {
      typedef INS_GPS2<FINS_Type> res_t;
    };
  };

  // Custom Kalman filter
  template <class T, template <class> class Filter>
  struct option_kf_t : option_t<T> {
    template <class INS_Type>
    struct filtered_ins_t {
      typedef Filtered_INS2<INS_Type, Filter> res_t;
    };
  };
  template <template <class> class Filter>
  struct kf : public INS_GPS_Factory<PureINS, option_kf_t<Options, Filter> > {};

  // INS with more precise Earth gravity model
  template <class T, class EGM>
  struct option_egm_t : option_t<T> {
    typedef INS_EGM<typename option_t<T>::ins_t, EGM> ins_t;
  };
  template <class T>
  struct option_egm_t<T, void> : option_t<T> {
    typedef INS_EGM<typename option_t<T>::ins_t> ins_t;
  };
  template <class EGM = void>
  struct egm : public INS_GPS_Factory<PureINS, option_egm_t<Options, EGM> > {};

  // bias estimation
  template <class T>
  struct option_bias_t : option_t<T> {
    typedef INS_BiasEstimated<typename option_t<T>::ins_t> ins_t;
    template <class INS_Type>
    struct filtered_ins_t {
      typedef Filtered_INS_BiasEstimated<
          typename option_t<T>::template filtered_ins_t<INS_Type>::res_t> res_t;
    };
    template <class FINS_Type>
    struct ins_gps_t {
      typedef INS_GPS_BiasEstimated<
          typename option_t<T>::template ins_gps_t<FINS_Type>::res_t> res_t;
    };
  };
  template <class U = void>
  struct bias : public INS_GPS_Factory<PureINS, option_bias_t<Options> > {};

  typedef typename option_t<Options>::template ins_gps_t<
      typename option_t<Options>::template filtered_ins_t<
        typename option_t<Options>::ins_t>::res_t>::res_t product;
};

#endif /* __INS_GPS_FACTORY_H__ */

