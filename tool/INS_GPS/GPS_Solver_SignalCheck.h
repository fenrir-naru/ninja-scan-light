/**
 * @file GPS solver with signal check
 *
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

#ifndef __GPS_SOLVER_SIGNAL_CHECK_H__
#define __GPS_SOLVER_SIGNAL_CHECK_H__

#include "navigation/GPS_Solver.h"

#include <map>
#include <istream>
#include <cstdlib>
#include <cmath>
#include <cctype>

template <class BaseSolver = GPS_SinglePositioning<double> >
class GPS_Solver_SignalCheck : public BaseSolver {
  public:
    typedef GPS_Solver_SignalCheck<BaseSolver> self_t;
    typedef BaseSolver super_t;
  private:
    self_t &operator=(const self_t &);
  public:
#if defined(__GNUC__) && (__GNUC__ < 5)
#define inheritate_type(x) typedef typename super_t::x x;
#else
#define inheritate_type(x) using typename super_t::x;
#endif
    inheritate_type(float_t);
    inheritate_type(space_node_t);

    inheritate_type(relative_property_t);
    inheritate_type(prn_t);
    //inheritate_type(measurement_t);
    inheritate_type(gps_time_t);
    inheritate_type(pos_t);
    inheritate_type(xyz_t);
#undef inheritate_type

    typedef std::map<prn_t, std::map<int, int> > signal_status_t; // {prn => {gps_time => status, ...}, ...}

    struct signal_info_t {
      signal_status_t status;

      int add_signal_status(const char *line){
        typedef std::vector<float_t> values_t;
        values_t values;
        const char *buf(line);
        char *next;
        while(*buf){
          if((*buf == '\r') || (*buf == '\n')){break;}
          float_t d((float_t)std::strtod(buf, &next));
          if(buf == next){
            if(!((*buf == ',') || std::isspace(*buf))){
              std::cerr << "Fail to parse => " << line << " ... " << std::endl;
              return -1;
            }
            buf++;
            continue;
          }
          values.push_back(d);
          buf = next;
        }
        if(values.empty()){return 0;} // empty line
        if(values.size() % 2 != 1){return -1;} // time,(prn,status)*N => odd length
        typename values_t::const_iterator it(values.begin());
        int t_msec(std::floor((1E1 * (*it++)) + 0.5) * 1E2); // time is managed in msec format with 100 ms ticks.
        int cnt(0);
        while(it != values.end()){
          int prn(*it++), flag(*it++);
          status[prn].insert(std::make_pair(t_msec, flag));
          cnt++;
        }
        return cnt;
      }

      int add_signal_status(std::istream &in){
        int cnt(0);
        char buf[0x400];
        while(!in.eof()){
          in.getline(buf, sizeof(buf));
          int cnt_add(add_signal_status(buf));
          if(cnt_add < 0){return -1;}
          cnt += cnt_add;
        }
        return cnt;
      }
    };

    struct options_t
        : public super_t::options_t,
        public signal_info_t {
      static options_t generate(
          const typename super_t::options_t &opt_super,
          const signal_info_t *info){
        options_t res;
        ((typename super_t::options_t &)res) = opt_super;
        if(info){
          (signal_info_t &)res = *info;
        }
        return res;
      }
    };
    const signal_info_t *signal_info;

  public:
    options_t available_options() const {
      return options_t::generate(super_t::available_options(), signal_info);
    }

    options_t available_options(options_t opt_wish) const {
      return options_t::generate(super_t::available_options(opt_wish), signal_info);
    }

    options_t update_options(const options_t &opt_wish){
      return options_t::generate(
          super_t::update_options(opt_wish),
          signal_info = &(const signal_info_t &)opt_wish);
    }

    GPS_Solver_SignalCheck(
        const space_node_t &sn,
        const options_t &opt_wish = options_t())
        : super_t(sn, opt_wish), signal_info(NULL) {}

    ~GPS_Solver_SignalCheck(){}

    relative_property_t relative_property(
        const prn_t &prn,
        const typename super_t::measurement_t::mapped_type &measurement,
        const float_t &receiver_error,
        const gps_time_t &time_arrival,
        const pos_t &usr_pos,
        const xyz_t &usr_vel) const {

      relative_property_t res(super_t::relative_property( // 元々の計算
          prn, measurement, receiver_error, time_arrival, usr_pos, usr_vel));

      while(signal_info){
        typename signal_status_t::const_iterator it(signal_info->status.find(prn));
        if(it == signal_info->status.end()){break;} // 衛星番号見つからない
        int t_msec(std::floor((1E1 * time_arrival.seconds) + 0.5) * 1E2); // 小数点1桁[sec]まで利用
        typename signal_status_t::mapped_type::const_iterator it2(it->second.find(t_msec));
        if(it2 == it->second.end()){break;} // 時刻見つからない

        switch(it2->second){ // フラグ
          case 0: // NLOS
            res.weight = 0; // 重みを0 => 排除される
            break;
          case 1: // LOS、そのまま
            break;
        }
        break;
      }

      return res;
    }
};

#endif /* __GPS_SOLVER_SIGNAL_CHECK_H__ */
