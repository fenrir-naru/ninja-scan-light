/*
 * INS_GPS_Debug.h, header file for INS/GPS debug.
 *
 * Copyright (c) 2017, M.Naruoka (fenrir)
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

#ifndef __INS_GPS_DEBUG_H__
#define __INS_GPS_DEBUG_H__

#if defined(_MSC_VER)
#define _USE_MATH_DEFINES
#endif

#include <iostream>
#include <cstring>
#include <cmath>
#include <vector>
#include <map>
#include <string>

#include "param/matrix.h"
#include "Filtered_INS2.h"
#include "INS_GPS2_Tightly.h"

#if !defined(BUILD_WITHOUT_GNSS_MULTI_FREQUENCY)
#include "GPS_Solver_MultiFrequency.h"
#endif

template <class FloatT>
struct INS_GPS_Debug_Property {
  enum debug_target_t {
      DEBUG_NONE,
      DEBUG_KF_P, DEBUG_KF_FULL,
      DEBUG_TIGHTLY,
      DEBUG_PURE_INERTIAL} debug_target;
  INS_GPS_Debug_Property() : debug_target(DEBUG_NONE) {}

  bool check_debug_property_spec(const char *spec){
    if(std::strcmp(spec, "KF_P") == 0){
      debug_target = DEBUG_KF_P;
    }else if(std::strcmp(spec, "KF_FULL") == 0){
      debug_target = DEBUG_KF_FULL;
    }else if(std::strcmp(spec, "tightly") == 0){
      debug_target = DEBUG_TIGHTLY;
    }else if(std::strcmp(spec, "pure_inertial") == 0){
      debug_target = DEBUG_PURE_INERTIAL;
    }else{
      return false;
    }
    return true;
  }

  struct show_debug_property_t {
    const INS_GPS_Debug_Property &property;
    show_debug_property_t(const INS_GPS_Debug_Property<FloatT> &prop) : property(prop) {}
    friend std::ostream &operator<<(std::ostream &out, const show_debug_property_t &_this){
      switch(_this.property.debug_target){
        case DEBUG_NONE: break;
        case DEBUG_KF_P: out << "KF_P"; break;
        case DEBUG_KF_FULL: out << "KF_FULL"; break;
        case DEBUG_TIGHTLY: out << "tightly"; break;
        case DEBUG_PURE_INERTIAL: out << "pure_inertial"; break;
      }
      return out;
    }
  };
  show_debug_property_t show_debug_property() const {
    return show_debug_property_t(*this);
  }
};

template <class INS_GPS>
class INS_GPS_Debug : public INS_GPS, protected INS_GPS_Debug_Property<typename INS_GPS::float_t> {
  public:
    typedef INS_GPS_Debug_Property<typename INS_GPS::float_t> prop_t;
    INS_GPS_Debug()
        : INS_GPS(), prop_t() {}
    INS_GPS_Debug(
        const INS_GPS_Debug &orig,
        const bool &deepcopy = false)
        : INS_GPS(orig, deepcopy), prop_t(orig) {}
    INS_GPS_Debug &operator=(const INS_GPS_Debug &another){
      INS_GPS::operator=(another);
      prop_t::operator=(another);
      return *this;
    }
    virtual ~INS_GPS_Debug(){}
    void setup_debug(const prop_t &property){
      prop_t::operator=(property);
    }

    virtual void inspect(std::ostream &out) const {}
};

template <class INS_GPS>
class INS_GPS_Debug_Covariance : public INS_GPS_Debug<INS_GPS> {
  public:
    typedef INS_GPS_Debug<INS_GPS> super_t;
#if defined(__GNUC__) && (__GNUC__ < 5)
    typedef typename super_t::float_t float_t;
    typedef typename super_t::mat_t mat_t;
#else
    using typename super_t::float_t;
    using typename super_t::mat_t;
#endif
  protected:
    enum {ACTION_LAST_NOP, ACTION_LAST_UPDATE, ACTION_LAST_CORRECT} last_action;
    struct snapshot_t {
      mat_t A, B, H, R, K, v;
      snapshot_t() : A(), B(), H(), R(), K(), v() {}
      snapshot_t(const snapshot_t &orig, const bool &deepcopy = false)
          : A(deepcopy ? orig.A.copy() : orig.A),
            B(deepcopy ? orig.B.copy() : orig.B),
            H(deepcopy ? orig.H.copy() : orig.H),
            R(deepcopy ? orig.R.copy() : orig.R),
            K(deepcopy ? orig.K.copy() : orig.K),
            v(deepcopy ? orig.v.copy() : orig.v) {}
    } snapshot;
  public:
    INS_GPS_Debug_Covariance()
        : super_t(), last_action(ACTION_LAST_NOP) {}
    INS_GPS_Debug_Covariance(
        const INS_GPS_Debug_Covariance &orig,
        const bool &deepcopy = false)
        : super_t(orig, deepcopy), last_action(orig.last_action), snapshot(orig.snapshot, deepcopy) {}
    INS_GPS_Debug_Covariance<INS_GPS> &operator=(const INS_GPS_Debug_Covariance<INS_GPS> &another){
      INS_GPS_Debug_Covariance<INS_GPS>::operator=(another);
      last_action = another.last_action;
      snapshot.A = another.snapshot.A;
      snapshot.B = another.snapshot.B;
      snapshot.H = another.snapshot.H;
      snapshot.R = another.snapshot.R;
      snapshot.K = another.snapshot.K;
      snapshot.v = another.snapshot.v;
      return *this;
    }
    virtual ~INS_GPS_Debug_Covariance(){}

    static void inspect_matrix(
        std::ostream &out, const mat_t &mat){
      for(unsigned int i(0), i_end(mat.rows()); i < i_end; i++){
        for(unsigned int j(0), j_end(mat.columns()); j < j_end; j++){
          out << mat(i, j) << ',';
        }
      }
    }
    static void inspect_matrix2(
        std::ostream &out, const mat_t &mat, const char *header){
      out << header << '(' << mat.rows() << '*' << mat.columns() << "),";
      inspect_matrix(out, mat);
    }
    void inspect(std::ostream &out) const {
      typedef INS_GPS_Debug_Covariance<INS_GPS> self_t;
      switch(super_t::debug_target){
        case super_t::DEBUG_KF_P:
          inspect_matrix(out, const_cast<self_t *>(this)->getFilter().getP());
          break;
        case super_t::DEBUG_KF_FULL:
          switch(last_action){
            case ACTION_LAST_UPDATE:
              out << "TU,";
              inspect_matrix2(out, snapshot.A, "A");
              inspect_matrix2(out, snapshot.B, "B");
              break;
            case ACTION_LAST_CORRECT:
              out << "MU,";
              inspect_matrix2(out, snapshot.H, "H");
              inspect_matrix2(out, snapshot.R, "R");
              inspect_matrix2(out, snapshot.K, "K");
              inspect_matrix2(out, snapshot.v, "v");
              break;
          }
          inspect_matrix2(out, const_cast<self_t *>(this)->getFilter().getP(), "P");
          break;
      }
    }

  protected:
    void before_update_INS(
        const mat_t &A, const mat_t &B,
        const float_t &elapsedT){
      last_action = ACTION_LAST_UPDATE;
      snapshot.A = A;
      snapshot.B = B;
      super_t::before_update_INS(A, B, elapsedT);
    }

    void before_correct_INS(
        const mat_t &H,
        const mat_t &R,
        const mat_t &K,
        const mat_t &v,
        mat_t &x_hat){
      last_action = ACTION_LAST_CORRECT;
      snapshot.H = H;
      snapshot.R = R;
      snapshot.K = K;
      snapshot.v = v;
      super_t::before_correct_INS(H, R, K, v, x_hat);
    }
};

template <class INS_GPS>
class INS_GPS_Debug_Tightly : public INS_GPS_Debug<INS_GPS> {
  public:
    typedef INS_GPS_Debug<INS_GPS> super_t;
#if defined(__GNUC__) && (__GNUC__ < 5)
    typedef typename super_t::float_t float_t;
#else
    using typename super_t::float_t;
#endif
    typedef typename super_t::receiver_state_t state_t;
    typedef typename super_t::solver_t::measurement_t measurement_t;
    typedef typename super_t::relative_property_list_t prop_list_t;
  protected:
    mutable struct snapshot_t {
      state_t state;
      measurement_t measurement;
      prop_list_t props;
    } snapshot;
    mutable bool updated;
    mutable std::map<typename super_t::solver_t::prn_t, int> prn_list;
  public:
    INS_GPS_Debug_Tightly() : super_t(), snapshot(), updated(false), prn_list() {}
    INS_GPS_Debug_Tightly(
        const INS_GPS_Debug_Tightly<INS_GPS> &orig, const bool &deepcopy = false)
        : super_t(orig, deepcopy), snapshot(orig.snapshot), updated(orig.updated),
        prn_list(orig.prn_list) {}
    ~INS_GPS_Debug_Tightly() {}
    void inspect(std::ostream &out) const {
      if(!updated){return;}
      updated = false;

      out << snapshot.props.size() << ','; // number of satellites

      do{ // DOP
        typename super_t::solver_t::user_pvt_t::precision_t dop, sigma_pos;
        if(!super_t::get_DOP_sigma(snapshot.state, snapshot.props, dop, sigma_pos)){
          out << ",,,,,,,,";
          break;
        }
        out << dop.g << ','
            << dop.p << ','
            << dop.h << ','
            << dop.v << ','
            << dop.t << ','
            << sigma_pos.h << ','
            << sigma_pos.v << ','
            << sigma_pos.t << ',';
      }while(false);

      typedef std::vector<int> order_t;
      order_t order(prn_list.size() + snapshot.props.size(), -1);
      for(typename prop_list_t::const_iterator
          it_begin(snapshot.props.begin()), it(it_begin), it_end(snapshot.props.end());
          it != it_end; ++it){
        int index((prn_list.find(it->first) == prn_list.end())
            ? (prn_list[it->first] = prn_list.size())
            : prn_list[it->first]);
        order[index] = std::distance(it_begin, it);
      }
      order.resize(prn_list.size());

      static const typename super_t::solver_t::measurement_item_set_t signals[] = {
        super_t::solver_t::L1CA,
#if !defined(BUILD_WITHOUT_GNSS_MULTI_FREQUENCY)
        GPS_Solver_MultiFrequency<typename super_t::solver_t>::L2CM,
        GPS_Solver_MultiFrequency<typename super_t::solver_t>::L2CL,
#endif
      };

      for(typename order_t::const_iterator it2(order.begin()), it2_end(order.end());
          it2 != it2_end; ++it2){
        if(*it2 < 0){ // skip because of unavailable satellite
          out << std::string(7 + (sizeof(signals) / sizeof(signals[0]) * 2), ',');
          continue;
        }

        typename prop_list_t::const_iterator it(snapshot.props.begin() + (*it2));
        out << it->first << ','; // prn
        if(it->second.range_sigma > 0){ // range (residual)
          out << it->second.range_residual << ','
              << it->second.range_sigma << ',';
        }else{
          out << ",,";
        }
        if(it->second.rate_sigma > 0){ // rate (residual)
          out << it->second.rate_residual << ','
              << it->second.rate_sigma << ',';
        }else{
          out << ",,";
        }
        if(it->second.range_sigma > 0){ // azimuth, elevation
          typename super_t::solver_t::xyz_t sat_los(
              -it->second.los_neg[0], -it->second.los_neg[1], -it->second.los_neg[2]);
          typename super_t::solver_t::enu_t enu_sat(
              super_t::solver_t::enu_t::relative_rel(sat_los, snapshot.state.pos.llh));
          out << enu_sat.azimuth() / M_PI * 180 << ','
              << enu_sat.elevation() / M_PI * 180 << ',';
        }else{
          out << ",,";
        }

        float_t buf;
        for(int i(0); i < sizeof(signals) / sizeof(signals[0]); ++i){
#define print(key) \
(super_t::solver_t::find_value( \
    snapshot.measurement[it->first], signals[i].key, buf) ? (out << buf) : out)
          print(pseudorange.i) << ',';
          print(signal_strength) << ',';
#undef print
        }
      }
    }

  protected:
    virtual prop_list_t &filter_relative_properties(
        const state_t &x,
        const measurement_t &measurement,
        prop_list_t &props) const {
      snapshot.state = x;
      snapshot.measurement = measurement;
      prop_list_t &res(super_t::filter_relative_properties(x, measurement, props));
      snapshot.props = res;
      updated = true;
      return res;
    }
};

template <class INS_GPS, class T_INS = typename INS_GPS::ins_t>
class INS_GPS_Debug_PureInertial : public INS_GPS_Debug<T_INS> {
  public:
    typedef INS_GPS_Debug<T_INS> super_t;
#if defined(__GNUC__) && (__GNUC__ < 5)
    typedef typename super_t::float_t float_t;
    typedef typename super_t::vec3_t vec3_t;
#else
    using typename super_t::float_t;
    using typename super_t::vec3_t;
#endif
    typedef Matrix<float_t> mat_t;
  public:
    INS_GPS_Debug_PureInertial() : super_t() {}
    INS_GPS_Debug_PureInertial(
        const INS_GPS_Debug_PureInertial<INS_GPS> &orig, const bool &deepcopy = false)
        : super_t(orig, deepcopy) {

    }
    ~INS_GPS_Debug_PureInertial() {}

    /* Dummy functions to resolve difference between INS/GPS and INS */
    template <class GPS_Packet>
    void correct(const GPS_Packet &gps){}

    template <class GPS_Packet>
    void correct(
        const GPS_Packet &gps,
        const vec3_t &lever_arm_b,
        const vec3_t &omega_b2i_4b){}

    void correct_yaw(
        const float_t &delta_psi,
        const float_t &sigma2_delta_psi){}
};

#endif /* __INS_GPS_DEBUG_H__ */
