/*
 * Copyright (c) 2022, M.Naruoka (fenrir)
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

/** @file
 * @brief Loader of RINEX Clock extension, support versions are 3 including 3.04
 */

#ifndef __RINEX_CLOCK__
#define __RINEX_CLOCK__

#include <cstring>
#include <cctype>
#include <string>

#include "RINEX.h"
#include "GPS.h"
#include "algorithm/interpolate.h"
#include "GPS_Solver_Base.h"

template <class FloatT>
struct RINEX_CLK {
  struct record_t {
    char type[2], name[9];
    std::tm epoch;
    int epoch_year4, epoch_mon12;
    FloatT epoch_sec;
    int items_followed;
    bool valid[6];
    union {
      FloatT array[6];
      struct {
        FloatT bias, bias_sigma, rate, rate_sigma, acc, acc_sigma;
      } v;
    } values;

    GPS_Time<FloatT> get_epoch() const {
      return GPS_Time<FloatT>(epoch) + (epoch_sec - epoch.tm_sec);
    }
    record_t &set_epoch(const GPS_Time<FloatT> &t){
      epoch = t.c_tm();
      epoch_year4 = epoch.tm_year + 1900;
      epoch_mon12 = epoch.tm_mon + 1;
      epoch_sec = std::fmod(t.seconds, 60);
      return *this;
    }

    std::string get_name() const {
      std::string res;
      for(std::size_t i(0); i < sizeof(name); ++i){
        if(!std::isgraph(name[i])){break;}
        res.push_back(name[i]);
      }
      return res;
    }
    record_t &set_name(const std::string &str){
      std::memset(name, ' ', sizeof(name));
      str.copy(name, str.size() > sizeof(name) ? sizeof(name) : str.size());
      return *this;
    }
  };

  struct per_node_t {
    std::map<GPS_Time<FloatT>, FloatT> bias;
    typedef InterpolatableSet<GPS_Time<FloatT>, FloatT, FloatT> interpolator_t;
    static const typename interpolator_t::condition_t interpolator_cnd_default;
    mutable struct {
      interpolator_t bias;
    } subset;
    bool precheck(const GPS_Time<FloatT> &t) const {
      subset.bias.update(t, bias, interpolator_cnd_default);
      return subset.bias.ready;
    }
    FloatT clock_error(const GPS_Time<FloatT> &t) const {
      return subset.bias.update(t, bias, interpolator_cnd_default).interpolate(t);
    }
    FloatT clock_error_dot(const GPS_Time<FloatT> &t) const {
      FloatT res;
      subset.bias.update(t, bias, interpolator_cnd_default).interpolate(t, &res);
      return res;
    }
  };
  struct satellites_t {
    typedef std::map<int, per_node_t> buf_t; // prn => per_node_t
    buf_t buf;

    protected:

    typedef typename GPS_Solver_Base<FloatT>::satellite_t sat_t;

    mutable struct per_system_t {
      struct {
        const satellites_t *sats;
        int offset;
      } clk_rate;
      struct {
        const void *impl;
        sat_t (*impl_select)(const void *, const int &, const GPS_Time<FloatT> &);
      } pos_vel;
      struct sat_buf_t {
        sat_t pos_vel;
        const per_node_t *clk_rate;
      };
      std::map<int, sat_buf_t> sat_buf;
    } per_system[5];

    static sat_t select(
        const void *ptr, const int &prn, const GPS_Time<FloatT> &receiver_time){
      const per_system_t *ptr_sys(reinterpret_cast<const per_system_t *>(ptr));

      // check position/velocity availability
      sat_t sat_pos_vel(
          ptr_sys->pos_vel.impl_select(ptr_sys->pos_vel.impl, prn, receiver_time));
      if(!sat_pos_vel.is_available()){return sat_t::unavailable();}

      int sat_id((prn & 0xFF) + ptr_sys->clk_rate.offset);

      // check clock_error/clock_error_dot availability
      typename buf_t::const_iterator it(ptr_sys->clk_rate.sats->buf.find(sat_id));
      if((it == ptr_sys->clk_rate.sats->buf.end()) || !it->second.precheck(receiver_time)){
        return sat_t::unavailable();
      }

      struct impl_t {
        static inline const typename per_system_t::sat_buf_t &sat(const void *ptr) {
          return *reinterpret_cast<const typename per_system_t::sat_buf_t *>(ptr);
        }
        static typename GPS_Solver_Base<FloatT>::xyz_t position(
            const void *ptr, const GPS_Time<FloatT> &t_tx, const FloatT &dt_transit) {
          return sat(ptr).pos_vel.position(t_tx, dt_transit);
        }
        static typename GPS_Solver_Base<FloatT>::xyz_t velocity(
            const void *ptr, const GPS_Time<FloatT> &t_tx, const FloatT &dt_transit) {
          return sat(ptr).pos_vel.velocity(t_tx, dt_transit);
        }
        static typename GPS_Solver_Base<FloatT>::float_t clock_error(
            const void *ptr, const GPS_Time<FloatT> &t_tx) {
          return sat(ptr).clk_rate->clock_error(t_tx);
        }
        static typename GPS_Solver_Base<FloatT>::float_t clock_error_dot(
            const void *ptr, const GPS_Time<FloatT> &t_tx) {
          return sat(ptr).clk_rate->clock_error_dot(t_tx);
        }
      };
      typename per_system_t::sat_buf_t impl = {sat_pos_vel, &(it->second)};
      sat_t res = {
          &(const_cast<per_system_t *>(ptr_sys)->sat_buf[sat_id] = impl),
          impl_t::position, impl_t::velocity, impl_t::clock_error, impl_t::clock_error_dot};
      return res;
    }

    public:

    template <class SelectorT>
    bool push(SelectorT &slct, const char &sys = 'G') const {
      int sys_idx(0), offset(0);
      switch(sys){
        case 'G': case 'J': case 'S': break; // GPS, QZSS, SBAS
        case 'R': offset = (int)'R' << 8; sys_idx = 1; break; // GLONASS
        case 'E': offset = (int)'E' << 8; sys_idx = 2; break; // Galileo
        case 'C': offset = (int)'C' << 8; sys_idx = 3; break; // BeiDou
        case 'I': offset = (int)'I' << 8; sys_idx = 4; break; // IRNSS
        default: return false;
      }
      per_system[sys_idx].clk_rate.sats = this;
      per_system[sys_idx].clk_rate.offset = offset;
      if(slct.impl_select != select){
        per_system[sys_idx].pos_vel.impl = slct.impl;
        per_system[sys_idx].pos_vel.impl_select = slct.impl_select;
      }
      slct.impl_select = select;
      slct.impl = &per_system[sys_idx];
      return true;
    }
  };
  typedef std::map<std::string, per_node_t> collection_t; // satellites + receivers, name => per_node_t
};

template <class FloatT>
const typename RINEX_CLK<FloatT>::per_node_t::interpolator_t::condition_t
    RINEX_CLK<FloatT>::per_node_t::interpolator_cnd_default = {
      5, // max_x_size
      60 * 60 * 2, // subset.cnd.max_dx_range
    };

template <class FloatT = double>
class RINEX_CLK_Reader : public RINEX_Reader<> {
  protected:
    typedef RINEX_Reader<> super_t;
  public:
    typedef typename RINEX_CLK<FloatT>::record_t record_t;
    static const typename super_t::convert_item_t head_v3[7];
    static const typename super_t::convert_item_t head_v304[7];

  protected:
    record_t record;

    void read_header(){
      if(super_t::src.fail()){return;}

      char buf[256];
      int label_offset(60);

      // Read header
      for(int line(0); !super_t::src.eof(); ++line){
        super_t::src.getline(buf, sizeof(buf));

        std::string content(buf);

        if(line == 0){ // Check version and type extraction
          switch(content.find("RINEX VERSION / TYPE")){
            case 60:
              super_t::version_type.parse(content);
              break;
            case 65: {
              double temp(0);
              super_t::conv_t<double>::f(content, 0, 4, &temp, 2);
              super_t::version_type.version = (int)(temp * 100);
              if(super_t::version_type.version != 304){return;}
              if(content[21] != 'C'){return;}
              super_t::version_type.file_type
                  = super_t::version_type_t::FTYPE_CLOCK;
              super_t::version_type.sat_system
                  = super_t::version_type_t::char2sys_v3(content[42]);
              label_offset = 65;
              break;
            }
            default: // includes std::string::npos:
              return;
          }
        }

        std::string label(content, label_offset, 20);
        {
          int real_length(label.find_last_not_of(' ') + 1);
          if(real_length < (int)label.length()){
            label = label.substr(0, real_length);
          }
        }

        if(label.find("END OF HEADER") == 0){break;}

        _header[label].push_back(content.substr(0, label_offset));
      }
    }

  public:
    void seek_next() {
      super_t::_has_next = false;

      char buf[1024];
      std::string str;
      bool v304(super_t::version_type.version == 304);
      while(true){
        if(super_t::src.getline(buf, sizeof(buf)).fail()){return;}
        str.assign(buf);
        if(str.size() < (v304 ? 45 : 37)){continue;}
        str.copy(record.type, 2);
        if((std::memcmp(record.type, "AR", 2) == 0)
            || (std::memcmp(record.type, "AS", 2) == 0)
            || (std::memcmp(record.type, "CR", 2) == 0)
            || (std::memcmp(record.type, "DR", 2) == 0)
            || (std::memcmp(record.type, "MS", 2) == 0)){
          break;
        }
      }
      std::memset(record.name, ' ', sizeof(record.name));
      str.copy(record.name, v304 ? 9 : 4, 3);
      v304 ? super_t::convert(head_v304, str, &record) : super_t::convert(head_v3, str, &record);
      record.epoch.tm_year = record.epoch_year4 - 1900; // greater than 1980
      record.epoch.tm_mon = record.epoch_mon12 - 1; // month [0, 11]
      record.epoch.tm_sec = (int)record.epoch_sec;

      for(int i(0), j(2), k(v304 ? 45 : 40); i < 6; ++i, ++j, k += (v304 ? 21 : 20)){
        record.valid[i] = false;
        if(i >= record.items_followed){continue;}
        if(j % 4 == 0){
          j = 0; k = 3;
          if(super_t::src.getline(buf, sizeof(buf)).fail()){break;}
          str.assign(buf);
        }
        if(!super_t::template conv_t<FloatT>::e_dot_head(
            str, k, 19, &record.values.array[i], 12)){break;}
        record.valid[i] = true;
      }

      super_t::_has_next = true;
    }

    RINEX_CLK_Reader(std::istream &in)
        : super_t(in, static_cast<void(super_t::*)()>(
            &RINEX_CLK_Reader::read_header)) {
      seek_next();
    }
    ~RINEX_CLK_Reader(){}

    void next() {
      seek_next();
    }

  protected:
    static int read_all(
        std::istream &in,
        void *buf, bool (*callback)(void *, const record_t &)){
      RINEX_CLK_Reader reader(in);
      if(reader.version_type.file_type != version_type_t::FTYPE_CLOCK){
        return -1;
      }
      int res(0);
      for(; reader.has_next(); reader.next()){
        if(!reader.record.valid[0]){continue;}
        if(!callback(buf, reader.record)){continue;}
        ++res;
      }
      return res;
    }

  public:
    static int read_all(
        std::istream &in,
        typename RINEX_CLK<FloatT>::satellites_t &dst){
      struct func_t {
        static bool add(void *ptr, const record_t &record){
          typename RINEX_CLK<FloatT>::satellites_t &dst(
              *reinterpret_cast<typename RINEX_CLK<FloatT>::satellites_t *>(ptr));
          if((record.type[0] != 'A') || (record.type[1] != 'S')){return false;}
          int offset(0);
          char sys(record.name[0]);
          switch(sys){
            case 'G': break; // GPS
            case 'R': offset = (int)'R' << 8; break; // GLONASS
            case 'E': offset = (int)'E' << 8; break; // Galileo
            case 'C': offset = (int)'C' << 8; break; // BeiDou
            case 'J': offset = 192; break; // QZSS
            case 'S': offset = 100; break; // SBAS
            case 'I': offset = (int)'I' << 8; break; // IRNSS
            default: return false;
          }
          if(record.name[1] < '0' || record.name[1] > '9'
              || record.name[2] < '0' || record.name[2] > '9'){return false;}
          int idx((record.name[1] - '0') * 10 + (record.name[2] - '0') + offset);
          dst.buf[idx].bias[record.get_epoch()] = record.values.v.bias;
          return true;
        }
      };
      return read_all(in, &dst, func_t::add);
    }

    static int read_all(
        std::istream &in,
        typename RINEX_CLK<FloatT>::collection_t &dst){
      struct func_t {
        static bool add(void *ptr, const record_t &record){
          typename RINEX_CLK<FloatT>::collection_t &dst(
              *reinterpret_cast<typename RINEX_CLK<FloatT>::collection_t *>(ptr));
          dst[record.get_name()].bias[record.get_epoch()] = record.values.v.bias;
          return true;
        }
      };
      return read_all(in, &dst, func_t::add);
    }
};

#define GEN_D(offset, length, container_type, container_member, value_type) \
    {super_t::template conv_t<value_type>::d, offset, length, \
      offsetof(container_type, container_member)}
#define GEN_I(offset, length, container_type, container_member, value_type) \
    {super_t::template conv_t<value_type>::d, offset, length, \
      offsetof(container_type, container_member), 1}
#define GEN_F2(offset, length, precision, container_type, container_member, value_type) \
    {super_t::template conv_t<value_type>::f_dot_head, offset, length, \
      offsetof(container_type, container_member), precision}
#define GEN_F(offset, length, precision, container_type, container_member) \
    GEN_F2(offset, length, precision, container_type, container_member, FloatT)

template <class FloatT>
const typename RINEX_CLK_Reader<FloatT>::convert_item_t RINEX_CLK_Reader<FloatT>::head_v3[] = {
  GEN_I( 8,  4,     record_t, epoch_year4,    int),
  GEN_I(13,  2,     record_t, epoch_mon12,    int),
  GEN_I(16,  2,     record_t, epoch.tm_mday,  int),
  GEN_I(19,  2,     record_t, epoch.tm_hour,  int),
  GEN_I(22,  2,     record_t, epoch.tm_min,   int),
  GEN_F(24, 10,  6, record_t, epoch_sec),
  GEN_D(34,  3,     record_t, items_followed, int),
};

template <class FloatT>
const typename RINEX_CLK_Reader<FloatT>::convert_item_t RINEX_CLK_Reader<FloatT>::head_v304[] = {
  GEN_I(13,  4,     record_t, epoch_year4,    int),
  GEN_I(18,  2,     record_t, epoch_mon12,    int),
  GEN_I(21,  2,     record_t, epoch.tm_mday,  int),
  GEN_I(24,  2,     record_t, epoch.tm_hour,  int),
  GEN_I(27,  2,     record_t, epoch.tm_min,   int),
  GEN_F(29,  9,  6, record_t, epoch_sec),
  GEN_D(40,  2,     record_t, items_followed, int),
};

#undef GEN_D
#undef GEN_I
#undef GEN_F

#endif /* __RINEX_CLOCK__ */
