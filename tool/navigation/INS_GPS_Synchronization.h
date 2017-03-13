/*
 *  INS_GPS_Synchronization.h, header file summarizing synchronization algorithm of
 *  INS and GPS integration.
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

#ifndef __INS_GPS_SYNCHRONIZATION__
#define __INS_GPS_SYNCHRONIZATION__

#include "param/matrix.h"
#include "param/vector3.h"

template <class FloatT>
struct INS_GPS_Back_Propagate_Property {
  /**
   * Number of snapshots to be back-propagated.
   * Zero means the last snapshot to be corrected, and negative values mean deeper.
   */
  FloatT back_propagate_depth;
  INS_GPS_Back_Propagate_Property() : back_propagate_depth(0) {}
};

template <class INS_GPS>
class INS_GPS_Back_Propagate : public INS_GPS, protected INS_GPS_Back_Propagate_Property<typename INS_GPS::float_t> {
  public:
#if defined(__GNUC__) && (__GNUC__ < 5)
    typedef typename INS_GPS::float_t float_t;
    typedef typename INS_GPS::mat_t mat_t;
#else
    using typename INS_GPS::float_t;
    using typename INS_GPS::mat_t;
#endif
  public:
    struct snapshot_content_t {
      INS_GPS ins_gps;
      mat_t Phi;
      mat_t GQGt;
      float_t elapsedT_from_last_correct;
      snapshot_content_t(
          const INS_GPS &_ins_gps,
          const mat_t &_Phi,
          const mat_t &_GQGt,
          const float_t &_elapsedT)
          : ins_gps(_ins_gps, true), Phi(_Phi), GQGt(_GQGt),
          elapsedT_from_last_correct(_elapsedT){
      }
    };
    typedef std::vector<snapshot_content_t> snapshots_t;
  protected:
    snapshots_t snapshots;
  public:
    INS_GPS_Back_Propagate()
        : INS_GPS(), snapshots() {}
    INS_GPS_Back_Propagate(
        const INS_GPS_Back_Propagate &orig,
        const bool &deepcopy = false)
        : INS_GPS(orig, deepcopy), snapshots(orig.snapshots){}
    virtual ~INS_GPS_Back_Propagate(){}
    void setup_back_propagation(const INS_GPS_Back_Propagate_Property<float_t> &property){
      INS_GPS_Back_Propagate_Property<float_t>::operator=(property);
    }
    const snapshots_t &get_snapshots() {return snapshots;}

  protected:
    /**
     * Call-back function for time update
     *
     * @param A matrix A
     * @param B matrix B
     * @patam elapsedT interval time
     */
    void before_update_INS(
        const mat_t &A, const mat_t &B,
        const float_t &elapsedT){
      mat_t Phi(A * elapsedT);
      for(unsigned i(0); i < A.rows(); i++){Phi(i, i) += 1;}
      mat_t Gamma(B * elapsedT);

      float_t elapsedT_from_last_correct(elapsedT);
      if(!snapshots.empty()){
        elapsedT_from_last_correct += snapshots.back().elapsedT_from_last_correct;
      }

      snapshots.push_back(
          snapshot_content_t(*this,
              Phi, Gamma * INS_GPS::getFilter().getQ() * Gamma.transpose(),
              elapsedT_from_last_correct));
    }

    /**
     * Call-back function for measurement (correct) update
     *
     * @param H matrix H
     * @param R matrix R
     * @patam K matrix K (Kalman gain)
     * @param v =(z - H x)
     * @param x_hat values to be corrected
     */
    void before_correct_INS(
        const mat_t &H,
        const mat_t &R,
        const mat_t &K,
        const mat_t &v,
        mat_t &x_hat){
      if(!snapshots.empty()){

        // This routine is invoked by measurement update function called correct().
        // In addition, this routine is reduced to be activated once with the following if-statement.
        float_t mod_elapsedT(snapshots.back().elapsedT_from_last_correct);
        if(mod_elapsedT > 0){

          // The latest is the first
          for(typename snapshots_t::reverse_iterator it(snapshots.rbegin());
              it != snapshots.rend();
              ++it){
            // This statement controls depth of back propagation.
            if(it->elapsedT_from_last_correct
                < INS_GPS_Back_Propagate_Property<float_t>::back_propagate_depth){
              if(mod_elapsedT > 0.1){ // Skip only when sufficient amount of snapshots are existed.
                snapshots.erase(snapshots.begin(), it.base());
                //cerr << "[erase]" << endl;
                if(snapshots.empty()){return;}
              }
              break;
            }
            // Positive value stands for states to which applied back-propagation have not been applied
            it->elapsedT_from_last_correct -= mod_elapsedT;
          }
        }

        snapshot_content_t previous(snapshots.back());
        snapshots.pop_back();

        // Perform back-propagation
        mat_t H_dash(H * previous.Phi);
        mat_t R_dash(R + H * previous.GQGt * H.transpose());
        previous.ins_gps.correct_primitive(H_dash, v, R_dash);

        snapshots.push_back(previous);
      }
    }
};

struct INS_GPS_RealTime_Property {
  enum rt_mode_t {RT_NORMAL, RT_LIGHT_WEIGHT} rt_mode; ///< Algorithm selection for realtime mode
  INS_GPS_RealTime_Property() : rt_mode(RT_NORMAL) {}
};

template <class INS_GPS>
class INS_GPS_RealTime : public INS_GPS, protected INS_GPS_RealTime_Property {
  public:
#if defined(__GNUC__) && (__GNUC__ < 5)
    typedef typename INS_GPS::float_t float_t;
    typedef typename INS_GPS::vec3_t vec3_t;
    typedef typename INS_GPS::mat_t mat_t;
#else
    using typename INS_GPS::float_t;
    using typename INS_GPS::vec3_t;
    using typename INS_GPS::mat_t;
#endif
  protected:
    struct snapshot_content_t {
      INS_GPS ins_gps;
      mat_t A;
      mat_t Phi_inv;
      mat_t GQGt;
      float_t elapsedT_from_last_update;
      snapshot_content_t(
          const INS_GPS &_ins_gps,
          const mat_t &_A,
          const mat_t &_Phi_inv,
          const mat_t &_GQGt,
          const float_t &_elapsedT)
          : ins_gps(_ins_gps, true),
          A(_A), Phi_inv(_Phi_inv), GQGt(_GQGt),
          elapsedT_from_last_update(_elapsedT){
      }
    };
    typedef std::vector<snapshot_content_t> snapshots_t;
    snapshots_t snapshots;
  public:
    INS_GPS_RealTime()
        : INS_GPS(), snapshots() {}
    INS_GPS_RealTime(
        const INS_GPS_RealTime &orig,
        const bool &deepcopy = false)
        : INS_GPS(orig, deepcopy), snapshots(orig.snapshots){}
    virtual ~INS_GPS_RealTime(){}
    void setup_realtime(const INS_GPS_RealTime_Property &property){
      INS_GPS_RealTime_Property::operator=(property);
    }

  protected:
    /**
     * Call-back function for time update
     *
     * @param A matrix A
     * @param B matrix B
     * @patam elapsedT interval time
     */
    void before_update_INS(
        const mat_t &A, const mat_t &B,
        const float_t &elapsedT){
      mat_t Phi(A * elapsedT);
      for(unsigned i(0); i < A.rows(); i++){Phi(i, i) += 1;}
      mat_t Gamma(B * elapsedT);

      snapshots.push_back(
          snapshot_content_t(*this,
              A, Phi.inverse(), Gamma * INS_GPS::getFilter().getQ() * Gamma.transpose(),
              elapsedT));
    }

  public:
    /**
     * Sort snapshots before correct
     * to let a snapshot corresponding to the latest GPS information be first
     *
     * @param advanceT how old is the GPS information (negative value means old)
     * @return bool true when successfully sorted
     */
    bool setup_correct(float_t advanceT){
      if(advanceT > 0){return false;} // positive value (future) is odd

      for(typename snapshots_t::reverse_iterator it(snapshots.rbegin());
          it != snapshots.rend();
          ++it){
        advanceT += it->elapsedT_from_last_update;
        if(advanceT > -0.005){ // Find the closest
          if(it == snapshots.rbegin()){
            // Keep at least one snapshot
            it++;
          }
          snapshots.erase(snapshots.begin(), it.base());
          return true;
        }
      }

      return false; // Too old
    }

  protected:
    /**
     * Perform correction with modified correct information
     *
     * @param info Correction information
     */
    void correct_with_info(CorrectInfo<float_t> &info){
      mat_t &H(info.H), &R(info.R);
      switch(rt_mode){
        case RT_LIGHT_WEIGHT:
          if(!snapshots.empty()){
            mat_t sum_A(H.columns(), H.columns());
            mat_t sum_GQGt(sum_A.rows(), sum_A.rows());
            float_t bar_delteT(0);
            for(typename snapshots_t::iterator it(snapshots.begin());
                it != snapshots.end();
                ++it){
              sum_A += it->A;
              sum_GQGt += it->GQGt;
              bar_delteT += it->elapsedT_from_last_update;
            }
            int n(snapshots.size());
            bar_delteT /= n;
            mat_t sum_A_GQGt(sum_A * sum_GQGt);
            R += H
                * (sum_GQGt -= ((sum_A_GQGt + sum_A_GQGt.transpose()) *= (bar_delteT * (n + 1) / (2 * n))))
                * H.transpose();
            H *= (mat_t::getI(sum_A.rows()) - sum_A * bar_delteT);
          }
          break;
        case RT_NORMAL:
        default:
          for(typename snapshots_t::iterator it(snapshots.begin());
              it != snapshots.end();
              ++it){
            H *= it->Phi_inv;
            R += H * it->GQGt * H.transpose();
          }
      }
      INS_GPS::correct_primitive(info);
    }

  public:
    template <class GPS_Packet>
    void correct(const GPS_Packet &gps){
      CorrectInfo<float_t> info(snapshots[0].ins_gps.correct_info(gps));
      correct_with_info(info);
    }

    template <class GPS_Packet>
    void correct(const GPS_Packet &gps,
        const vec3_t &lever_arm_b,
        const vec3_t &omega_b2i_4b){
      CorrectInfo<float_t> info(snapshots[0].ins_gps.correct_info(gps, lever_arm_b, omega_b2i_4b));
      correct_with_info(info);
    }
};

#endif /* __INS_GPS_SYNCHRONIZATION__ */
