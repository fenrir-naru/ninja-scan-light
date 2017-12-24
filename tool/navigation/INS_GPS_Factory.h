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

struct INS_GPS_Factory_Options {

  template <class T, class U = void>
  struct option_t {
    template <class T_Add>
    struct add_t {
      typedef typename T::template add_t<T_Add>::res_t res_t;
    };
  };

  template <class U>
  struct option_t<void, U> {
    template <class T_Add>
    struct add_t {
      typedef T_Add res_t;
    };
  };

  enum {
    Priority_EGM,
    Priority_KF,
    Priority_Bias,
  };

  // Custom Kalman filter
  template <class T, template <class> class Filter>
  struct kf_t : option_t<T> {

    static const int priority = Priority_KF;

    template <class T_Change>
    struct change_t {
      typedef kf_t<T_Change, Filter> res_t;
    };

    template <class T_Add>
    struct add_t {
      template <class T_Rebuild>
      struct check_copy_t {
        template <bool new_is_under, class U = void>
        struct check_order_t { // new_opt<old_opt>
          typedef typename T_Rebuild::template change_t<kf_t<T, Filter> >::res_t res_t;
        };
        template <class U>
        struct check_order_t<true, U> { // old_opt_top<new_opt>
          typedef kf_t<T_Rebuild, Filter> res_t;
        };
        typedef typename check_order_t<(priority > T_Rebuild::priority)>::res_t res_t;
      };
      template <class T_Rebuild_Base, template <class> class Filter_New>
      struct check_copy_t<kf_t<T_Rebuild_Base, Filter_New> > {
        typedef kf_t<T_Rebuild_Base, Filter_New> res_t;
      };
      typedef typename check_copy_t<
          typename option_t<T>::template add_t<T_Add>::res_t>::res_t res_t;
    };
  };

  // INS with more precise Earth gravity model
  template <class T, class EGM>
  struct egm_t : option_t<T> {

    static const int priority = Priority_EGM;

    template <class T_Change>
    struct change_t {
      typedef egm_t<T_Change, EGM> res_t;
    };

    template <class T_Add>
    struct add_t {
      template <class T_Rebuild>
      struct check_copy_t {
        template <bool new_is_under, class U = void>
        struct check_order_t { // new_opt<old_opt>
          typedef typename T_Rebuild::template change_t<egm_t<T, EGM> >::res_t res_t;
        };
        template <class U>
        struct check_order_t<true, U> { // old_opt_top<new_opt>
          typedef egm_t<T_Rebuild, EGM> res_t;
        };
        typedef typename check_order_t<(priority > T_Rebuild::priority)>::res_t res_t;
      };
      template <class T_Rebuild_Base, class EGM_New>
      struct check_copy_t<egm_t<T_Rebuild_Base, EGM_New> > {
        typedef egm_t<T_Rebuild_Base, EGM_New> res_t;
      };
      typedef typename check_copy_t<
          typename option_t<T>::template add_t<T_Add>::res_t>::res_t res_t;
    };
  };

  // bias estimation
  template <class T>
  struct bias_t : option_t<T> {

    static const int priority = Priority_Bias;

    template <class T_Change>
    struct change_t {
      typedef bias_t<T_Change> res_t;
    };

    template <class T_Add>
    struct add_t {
      template <class T_Rebuild>
      struct check_copy_t {
        template <bool new_is_under, class U = void>
        struct check_order_t { // new_opt<old_opt>
          typedef typename T_Rebuild::template change_t<bias_t<T> >::res_t res_t;
        };
        template <class U>
        struct check_order_t<true, U> { // old_opt_top<new_opt>
          typedef bias_t<T_Rebuild> res_t;
        };
        typedef typename check_order_t<(priority > T_Rebuild::priority)>::res_t res_t;
      };
      template <class T_Rebuild_Base>
      struct check_copy_t<bias_t<T_Rebuild_Base> > {
        typedef bias_t<T_Rebuild_Base> res_t;
      };
      typedef typename check_copy_t<
          typename option_t<T>::template add_t<T_Add>::res_t>::res_t res_t;
    };
  };
};

template <class PureINS = INS<>, class Options = void>
struct INS_GPS_Factory {

  template <class T, class U = void>
  struct option_t {
    typedef T base_t;
    typedef typename base_t::ins_t ins_t;
    template <class INS_Type>
    struct filtered_ins_t {
      typedef typename base_t::template filtered_ins_t<INS_Type>::res_t res_t;
    };
    template <class FINS_Type>
    struct ins_gps_t {
      typedef typename base_t::template ins_gps_t<FINS_Type>::res_t res_t;
    };
  };

  typedef Options options_t;

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
  struct option_t<typename INS_GPS_Factory_Options::kf_t<T, Filter> > : option_t<T> {
    template <class INS_Type>
    struct filtered_ins_t {
      typedef Filtered_INS2<INS_Type, Filter> res_t;
    };
  };
  template <template <class> class Filter>
  struct kf : public INS_GPS_Factory<PureINS,
      typename INS_GPS_Factory_Options::template option_t<Options>
        ::template add_t<
          typename INS_GPS_Factory_Options::template kf_t<void, Filter> >::res_t> {};

  // INS with more precise Earth gravity model
  template <class T, class EGM>
  struct option_t<typename INS_GPS_Factory_Options::egm_t<T, EGM> > : option_t<T> {
    template <class EGM2, class U = void>
    struct check_egm_t {
      typedef INS_EGM<typename option_t<T>::ins_t, EGM> res_t;
    };
    template <class U>
    struct check_egm_t<void, U> {
      typedef INS_EGM<typename option_t<T>::ins_t> res_t;
    };
    typedef typename check_egm_t<EGM>::res_t ins_t;
  };
  template <class EGM = void>
  struct egm : public INS_GPS_Factory<PureINS,
      typename INS_GPS_Factory_Options::template option_t<Options>
        ::template add_t<
          typename INS_GPS_Factory_Options::template egm_t<void, EGM> >::res_t> {};

  // bias estimation
  template <class T>
  struct option_t<typename INS_GPS_Factory_Options::bias_t<T> > : option_t<T> {
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
  struct bias : public INS_GPS_Factory<PureINS,
      typename INS_GPS_Factory_Options::template option_t<Options>
        ::template add_t<
          typename INS_GPS_Factory_Options::template bias_t<void> >::res_t> {};

  typedef typename option_t<Options>::template ins_gps_t<
      typename option_t<Options>::template filtered_ins_t<
        typename option_t<Options>::ins_t>::res_t>::res_t product;
};

#endif /* __INS_GPS_FACTORY_H__ */

