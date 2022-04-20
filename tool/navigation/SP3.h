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
 * @brief SP3 Reader/Writer, support ver.D
 *
 */

#ifndef __SP3_H__
#define __SP3_H__

#include <ctime>
#include <map>
#include <algorithm>
#include <stdexcept>

#include "util/text_helper.h"
#include "GPS.h"

#include "param/vector3.h"
#include "algorithm/interpolate.h"

template <class FloatT>
struct SP3_Product {
  struct prop_t {
    Vector3<FloatT> xyz;
    FloatT clk;
  };
  struct per_satellite_t {
    typedef std::map<GPS_Time<FloatT>, prop_t> history_t;
    history_t pos_history;
    history_t vel_history;

    mutable struct {
      struct target_t {

        typedef typename std::vector<std::pair<GPS_Time<FloatT>, prop_t> > buf_t;
        buf_t buf;
        GPS_Time<FloatT> t0;
        std::vector<FloatT> t;

        /**
         * maximum acceptable absolute time difference between current and epoch
         * for interpolation
         */
        FloatT max_delta_t;

        // max_epochs = 9 expects 15 min interval records; (2 hr * 2 / 15 min = 8)
        target_t(const std::size_t &max_epochs = 9)
            : buf(max_epochs), t0(0, 0), max_delta_t(60 * 60 * 2) {}

        target_t &update(const GPS_Time<FloatT> &current, const history_t &history){

          FloatT t_diff(t0 - current);
          if((std::abs(t_diff) <= 10)
              || ((t.size() >= 2)
                && (std::abs(t_diff + t[0]) <= std::abs(t_diff + t[1]))) ){
            return *this;
          }

          // If the 1st and 2nd nearest epochs are changed, then recalculate interpolation targets.
          struct {
            const GPS_Time<FloatT> &t_base;
            bool operator()(
                const typename history_t::value_type &rhs,
                const typename history_t::value_type &lhs) const {
              return std::abs(rhs.first - t_base) < std::abs(lhs.first - t_base);
            }
          } cmp = {(t0 = current)};

          t.clear();
          // extract t where t0-dt <= t <= t0+dt, then sort by ascending order of |t-t0|
          for(typename buf_t::const_iterator
                it(buf.begin()),
                it_end(std::partial_sort_copy(
                  history.lower_bound(current - max_delta_t),
                  history.upper_bound(current + max_delta_t),
                  buf.begin(), buf.end(), cmp));
              it != it_end; ++it){
            t.push_back(it->first - t0);
          }

          return *this;
        }

        Vector3<FloatT> interpolate_xyz(const GPS_Time<FloatT> &current) const {
          if(t.empty()){
            throw std::range_error("Insufficient records for interpolation");
          }
          struct second_iterator : public buf_t::const_iterator {
            second_iterator(const typename buf_t::const_iterator &it)
                : buf_t::const_iterator(it) {}
            const Vector3<FloatT> &operator[](const int &n) const {
              return buf_t::const_iterator::operator[](n).second.xyz;
            }
          } second(buf.begin());
          return interpolate_Neville<Vector3<FloatT> >(
              t, second, current - t0, t.size() - 1);
        }
        FloatT interpolate_clk(const GPS_Time<FloatT> &current) const {
          if(t.empty()){
            throw std::range_error("Insufficient records for interpolation");
          }
          struct second_iterator : public buf_t::const_iterator {
            second_iterator(const typename buf_t::const_iterator &it)
                : buf_t::const_iterator(it) {}
            const FloatT &operator[](const int &n) const {
              return buf_t::const_iterator::operator[](n).second.clk;
            }
          } second(buf.begin());
          return interpolate_Neville<FloatT>(
              t, second, current - t0, t.size() - 1);
        }
      } pos_clk, vel_rate;
    } subset;

    typename GPS_SpaceNode<FloatT>::SatelliteProperties::constellation_t
        constellation(
          const GPS_Time<FloatT> &t,
          const bool &with_velocity = true) const {
      typename GPS_SpaceNode<FloatT>::SatelliteProperties::constellation_t res;
      res.position = subset.pos_clk.update(t, pos_history).interpolate_xyz(t);
      if(with_velocity){
        res.velocity = subset.vel_rate.update(t, vel_history).interpolate_xyz(t);
      }
      return res;
    }
    typename GPS_SpaceNode<FloatT>::xyz_t position(
        const GPS_Time<FloatT> &t) const {
      return constellation(t, false).position;
    }
    typename GPS_SpaceNode<FloatT>::xyz_t velocity(
        const GPS_Time<FloatT> &t) const {
      return constellation(t, true).velocity;
    }
    FloatT clock_error(const GPS_Time<FloatT> &t) const {
      return subset.pos_clk.update(t, pos_history).interpolate_clk(t);
    }
    FloatT clock_error_dot(const GPS_Time<FloatT> &t) const {
      return subset.vel_rate.update(t, vel_history).interpolate_clk(t);
    }
  };
  std::map<int, per_satellite_t> satellites;
};

template <class FloatT>
class SP3_Reader {
  protected:
    typename TextHelper<>::crlf_stream_t src;
  public:
    struct l1_t {
      char version_symbol[2];
      char pos_or_vel_flag[1];
      int year_start;
      int month_start;
      int day_of_month_st;
      int hour_start;
      int minute_start;
      FloatT second_start;
      int number_of_epochs;
      char data_used[5];
      char coordinate_sys[5];
      char orbit_type[3];
      char agency[4];
    };
    struct l2_t {
      char symbols[2];
      int gps_week;
      FloatT seconds_of_week;
      FloatT epoch_interval;
      int mod_jul_day_st;
      FloatT fractional_day;
    };
    struct l3_11_t {
      char symbols[2];
      int number_of_sats;
      int sat_id[17];
    };
    struct l12_20_t {
      char symbols[2];
      int sat_accuracy[17];
    };
    struct l21_22_t {
      char symbols[2];
      char file_type[2];
      char _2_characters[2];
      char time_system[3];
      char _3_characters[3];
      char _4_characters[4][4];
      char _5_characters[4][5];
    };
    struct l23_24_t {
      char symbols[2];
      FloatT base_for_pos_vel;
      FloatT base_for_clk_rate;
      FloatT _14_column_float;
      FloatT _18_column_float;
    };
    struct l25_26_t {
      char symbols[2];
      int _4_column_int[4];
      int _6_column_int[4];
      int _9_column_int;
    };
    struct comment_t {
      char symbols[2];
      char comment[57];
    };
    struct epoch_t {
      char symbols[2];
      int year_start;
      int month_start;
      int day_of_month_st;
      int hour_start;
      int minute_start;
      FloatT second_start;
      std::tm c_tm() const {
        std::tm res = {
          (int)second_start,
          minute_start,
          hour_start,
          day_of_month_st,
          month_start - 1,
          year_start - 1900,
        };
        std::mktime(&res);
        return res;
      }
      operator GPS_Time<FloatT>() const {
        return GPS_Time<FloatT>(c_tm(), second_start - (int)second_start);
      }
    };
    struct position_clock_t {
      char symbol[1];
      int vehicle_id;
      FloatT coordinate_km[3];
      FloatT clock_us;
      bool has_sdev;
      int sdev_b_n_mm[3];
      int c_sdev_b_n_psec;
      char clock_event_flag[1];
      char clock_pred_flag[1];
      char maneuver_flag[1];
      char orbit_pred_flag[1];
    };
    struct position_clock_correlation_t {
      char symbols[2];
      int sdev_mm[3];
      int clk_sdev_psec;
      int xy_correlation;
      int xz_correlation;
      int xc_correlation;
      int yz_correlation;
      int yc_correlation;
      int zc_correlation;
    };
    struct velocity_rate_t {
      char symbol[1];
      int vehicle_id;
      FloatT velocity_dm_s[3];
      FloatT clock_rate_chg_100ps_s; // 10^-4 microseconds/second = 100 ps/s
      bool has_sdev;
      int vel_sdev_100nm_s[3];
      int clkrate_sdev_10_4_ps_s; // 10^-4 ps/s
    };
    struct velocity_rate_correlation_t {
      char symbols[2];
      int vel_sdev_100nm_s[3];
      int clkrate_sdev_10_4_ps_s;
      int xy_correlation;
      int xz_correlation;
      int xc_correlation;
      int yz_correlation;
      int yc_correlation;
      int zc_correlation;
    };
    struct parsed_t {
      enum {
        UNKNOWN,
        L1,
        L2,
        L3_11,
        L12_20,
        L21_22,
        L23_24,
        L25_26,
        COMMENT,
        EPOCH,
        POSITION_CLOCK,
        POSITION_CLOCK_CORRELATION,
        VELOCITY_RATE,
        VELOCITY_RATE_CORRELATION,
        PARSED_ITEMS,
      } type;
      union {
        struct l1_t l1;
        struct l2_t l2;
        struct l3_11_t l3_11;
        struct l12_20_t l12_20;
        struct l21_22_t l21_22;
        struct l23_24_t l23_24;
        struct l25_26_t l25_26;
        struct comment_t comment;
        struct epoch_t epoch;
        struct position_clock_t position_clock;
        struct position_clock_correlation_t position_clock_correlation;
        struct velocity_rate_t velocity_rate;
        struct velocity_rate_correlation_t velocity_rate_correlation;
      } item;
    };

    static const typename TextHelper<>::convert_item_t
        l1_items[13],
        l2_items[6],
        l3_11_items[19],
        l12_20_items[18],
        l21_22_items[13],
        l23_24_items[5],
        l25_26_items[10],
        comment_items[2],
        epoch_items[7],
        position_clock_items[14],
        position_clock_correlation_items[11],
        velocity_rate_items[10],
        velocity_rate_correlation_items[11];

    SP3_Reader(std::istream &in) : src(in) {}

    bool has_next() {
      return !(src.eof() || src.fail());
    }

    parsed_t parse_line() {
      parsed_t res = {parsed_t::UNKNOWN, {0}};

      char buf[0x100] = {0};
      src.getline(buf, sizeof(buf));
      std::string line(buf);

      switch(buf[0]){
        case '#':
          switch(buf[1]){
            case '#':
              TextHelper<>::str2val(l2_items, line, &res.item);
              res.type = parsed_t::L2;
              break;
            default:
              TextHelper<>::str2val(l1_items, line, &res.item);
              res.type = parsed_t::L1;
              break;
          }
          break;
        case '+':
          switch(buf[1]){
            case ' ':
              TextHelper<>::str2val(l3_11_items, line, &res.item);
              res.type = parsed_t::L3_11;
              break;
            case '+':
              TextHelper<>::str2val(l12_20_items, line, &res.item);
              res.type = parsed_t::L12_20;
              break;
          }
          break;
        case '%':
          switch(buf[1]){
            case 'c':
              TextHelper<>::str2val(l21_22_items, line, &res.item);
              res.type = parsed_t::L21_22;
              break;
            case 'f':
              TextHelper<>::str2val(l23_24_items, line, &res.item);
              res.type = parsed_t::L23_24;
              break;
            case 'i':
              TextHelper<>::str2val(l25_26_items, line, &res.item);
              res.type = parsed_t::L25_26;
              break;
          }
          break;
        case '/':
          res.type = parsed_t::COMMENT; // TODO
          break;
        case '*':
          TextHelper<>::str2val(epoch_items, line, &res.item);
          res.type = parsed_t::EPOCH;
          break;
        case 'P':
          res.item.position_clock.has_sdev = (line.length() > 60);
          TextHelper<>::str2val(
              position_clock_items,
              (res.item.position_clock.has_sdev ? 14 : 6),
              line, &res.item);
          res.type = parsed_t::POSITION_CLOCK;
          break;
        case 'V':
          res.item.velocity_rate.has_sdev = (line.length() > 60);
          TextHelper<>::str2val(
              velocity_rate_items,
              (res.item.velocity_rate.has_sdev ? 10 : 6),
              line, &res.item);
          res.type = parsed_t::VELOCITY_RATE;
          break;
        case 'E':
          switch(buf[1]){
            case 'P':
              TextHelper<>::str2val(position_clock_correlation_items, line, &res.item);
              res.type = parsed_t::POSITION_CLOCK_CORRELATION;
              break;
            case 'V':
              TextHelper<>::str2val(velocity_rate_correlation_items, line, &res.item);
              res.type = parsed_t::VELOCITY_RATE_CORRELATION;
              break;
          }
          break;
      }

      return res;
    }

    struct conv_t {
      /**
       * @param value satellite identifier. For GPS, SBAS, and QZSS, it is PRN number.
       * For other satellite systems, it is ((prefix_char << 8) | satellite_number)
       * like 20993 = ((82 << 8) | 1) indicating R01 (because 'R' = 82.
       */
      static bool sat_id(
          std::string &buf, const int &offset, const int &length, void *value,
          const int &opt = 0, const bool &str2val = true){
        // format: a letter followed by a 2-digit integer between 01 and 99.
        if(str2val){
          if(!TextHelper<>::template format_t<int>::d(
              buf, offset + 1, length - 1, value, opt, true)){
            return false;
          }
          switch(buf[offset]){
            case 'G': break; // GPS
            case 'S': // Satellite-Based Augmentation System (SBAS) satellites
              *static_cast<int *>(value) += 100; break;
            case 'J': // QZSS, nn=PRN-192, ex) J01 => PRN=193
              *static_cast<int *>(value) += 192; break;
            case 'R': // GLONASS
            case 'L': // Low-Earth Orbiting (LEO) satellites
            case 'E': // Galileo
            case 'C': // BeiDou
            case 'I': // IRNSS
              *static_cast<int *>(value) += ((int)buf[offset] << 8); break;
            case ' ':
              *static_cast<int *>(value) = 0; break; // TODO
            default:
              return false; // unsupported
          }
          return true;
        }else{
          int digit2(*static_cast<int *>(value));
          if(digit2 < 0){return false;}
          char prefix((char)((digit2 >> 8) & 0xFF));
          do{
            if(digit2 == 0){
              prefix = ' ';
              break;
            }
            if(digit2 <= 32){ // GPS
              prefix = 'G';
              break;
            }
            if(digit2 < 120){return false;}
            if(digit2 <= 158){ // SBAS
              prefix = 'S';
              digit2 -= 100;
              break;
            }
            if(digit2 < 193){return false;}
            if(digit2 <= 206){ // QZSS
              prefix = 'J';
              digit2 -= 192;
              break;
            }
            switch(prefix){
              case 'R': // GLONASS
              case 'L': // Low-Earth Orbiting (LEO) satellites
              case 'E': // Galileo
              case 'C': // BeiDou
              case 'I': // IRNSS
                digit2 &= 0xFF;
                break;
              default:
                return false; // unsupported
            }
          }while(false);
          buf[offset] = prefix;
          return TextHelper<>::template format_t<int>::d(
              buf, offset + 1, length - 1, &digit2, digit2 > 0 ? 1 : 0, false);
        }
      }
    };

    static int read_all(std::istream &in, SP3_Product<FloatT> &dst) {
      SP3_Reader<FloatT> src(in);
      typedef SP3_Product<FloatT> dst_t;
      int res(0);
      struct buf_t {
        dst_t &dst;
        int &res;
        epoch_t epoch;
        struct pv_t {
          position_clock_t pos;
          velocity_rate_t vel;
          pv_t(){pos.symbol[0] = vel.symbol[0] = 0;}
        };
        typedef std::map<int, pv_t> entries_t;
        entries_t entries;
        buf_t(dst_t &dst_, int &res_) : dst(dst_), res(res_), entries(){
          epoch.symbols[0] = 0;
        }
        void flush(){
          if(!epoch.symbols[0]){return;}
          GPS_Time<FloatT> gpst(epoch);
          for(typename entries_t::const_iterator it(entries.begin()), it_end(entries.end());
              it != it_end; ++it){
            if(it->second.pos.symbol[0]){
              typename dst_t::prop_t prop = {
                Vector3<FloatT>(it->second.pos.coordinate_km) * 1E3,
                it->second.pos.clock_us * 1E-6,
              };
              dst.satellites[it->first].pos_history.insert(std::make_pair(gpst, prop));
            }
            if(it->second.vel.symbol[0]){
              typename dst_t::prop_t prop = {
                Vector3<FloatT>(it->second.vel.velocity_dm_s) * 1E-1,
                it->second.vel.clock_rate_chg_100ps_s * 1E-10,
              };
              dst.satellites[it->first].vel_history.insert(std::make_pair(gpst, prop));
            }
            ++res;
          }
          entries.clear();
        }
      } buf(dst, res);
      while(src.has_next()){
        parsed_t parsed(src.parse_line());
        switch(parsed.type){
          case parsed_t::EPOCH:
            buf.flush();
            buf.epoch = parsed.item.epoch;
            break;
          case parsed_t::POSITION_CLOCK:
            buf.entries[parsed.item.position_clock.vehicle_id].pos = parsed.item.position_clock;
            break;
          case parsed_t::VELOCITY_RATE:
            buf.entries[parsed.item.velocity_rate.vehicle_id].vel = parsed.item.velocity_rate;
            break;
          default: break;
        }
      }
      buf.flush();
      return res;
    }
};

#define GEN_C(offset, length, container_type, container_member) \
    {TextHelper<>::template format_t<char>::c, offset, length, \
      offsetof(container_type, container_member), 0}
#define GEN_I(offset, length, container_type, container_member) \
    {TextHelper<>::template format_t<int>::d, offset, length, \
      offsetof(container_type, container_member), 0}
#define GEN_I2(offset, length, container_type, container_member) \
    {TextHelper<>::template format_t<int>::d_blank, offset, length, \
      offsetof(container_type, container_member), 0}
#define GEN_E(offset, length, container_type, container_member, precision) \
    {TextHelper<>::template format_t<FloatT>::f, offset, length, \
      offsetof(container_type, container_member), precision}
#define GEN_sat(offset, length, container_type, container_member) \
    {SP3_Reader<FloatT>::conv_t::sat_id, offset, length, \
      offsetof(container_type, container_member)}

template <class FloatT>
const typename TextHelper<>::convert_item_t SP3_Reader<FloatT>::l1_items[13] = {
  GEN_C(0, 2, l1_t, version_symbol),
  GEN_C(2, 1, l1_t, pos_or_vel_flag),
  GEN_I(3, 4, l1_t, year_start),
  GEN_I(8, 2, l1_t, month_start),
  GEN_I(11, 2, l1_t, day_of_month_st),
  GEN_I(14, 2, l1_t, hour_start),
  GEN_I(17, 2, l1_t, minute_start),
  GEN_E(20, 11, l1_t, second_start, 8),
  GEN_I(32, 7, l1_t, number_of_epochs),
  GEN_C(40, 5, l1_t, data_used),
  GEN_C(46, 5, l1_t, coordinate_sys),
  GEN_C(52, 3, l1_t, orbit_type),
  GEN_C(56, 4, l1_t, agency),
};
template <class FloatT>
const typename TextHelper<>::convert_item_t SP3_Reader<FloatT>::l2_items[6] = {
  GEN_C(0, 2, l2_t, symbols),
  GEN_I(3, 4, l2_t, gps_week),
  GEN_E(8, 15, l2_t, seconds_of_week, 8),
  GEN_E(24, 14, l2_t, epoch_interval, 8),
  GEN_I(39, 5, l2_t, mod_jul_day_st),
  GEN_E(45, 15, l2_t, fractional_day, 13),
};
template <class FloatT>
const typename TextHelper<>::convert_item_t SP3_Reader<FloatT>::l3_11_items[19] = {
  GEN_C(0, 2, l3_11_t, symbols),
  GEN_I2(3, 3, l3_11_t, number_of_sats),
  GEN_sat(9, 3, l3_11_t, sat_id[0]),
  GEN_sat(12, 3, l3_11_t, sat_id[1]),
  GEN_sat(15, 3, l3_11_t, sat_id[2]),
  GEN_sat(18, 3, l3_11_t, sat_id[3]),
  GEN_sat(21, 3, l3_11_t, sat_id[4]),
  GEN_sat(24, 3, l3_11_t, sat_id[5]),
  GEN_sat(27, 3, l3_11_t, sat_id[6]),
  GEN_sat(30, 3, l3_11_t, sat_id[7]),
  GEN_sat(33, 3, l3_11_t, sat_id[8]),
  GEN_sat(36, 3, l3_11_t, sat_id[9]),
  GEN_sat(39, 3, l3_11_t, sat_id[10]),
  GEN_sat(42, 3, l3_11_t, sat_id[11]),
  GEN_sat(45, 3, l3_11_t, sat_id[12]),
  GEN_sat(48, 3, l3_11_t, sat_id[13]),
  GEN_sat(51, 3, l3_11_t, sat_id[14]),
  GEN_sat(54, 3, l3_11_t, sat_id[15]),
  GEN_sat(57, 3, l3_11_t, sat_id[16]),
};
template <class FloatT>
const typename TextHelper<>::convert_item_t SP3_Reader<FloatT>::l12_20_items[18] = {
  GEN_C(0, 2, l12_20_t, symbols),
  GEN_I(9, 3, l12_20_t, sat_accuracy[0]),
  GEN_I(12, 3, l12_20_t, sat_accuracy[1]),
  GEN_I(15, 3, l12_20_t, sat_accuracy[2]),
  GEN_I(18, 3, l12_20_t, sat_accuracy[3]),
  GEN_I(21, 3, l12_20_t, sat_accuracy[4]),
  GEN_I(24, 3, l12_20_t, sat_accuracy[5]),
  GEN_I(27, 3, l12_20_t, sat_accuracy[6]),
  GEN_I(30, 3, l12_20_t, sat_accuracy[7]),
  GEN_I(33, 3, l12_20_t, sat_accuracy[8]),
  GEN_I(36, 3, l12_20_t, sat_accuracy[9]),
  GEN_I(39, 3, l12_20_t, sat_accuracy[10]),
  GEN_I(42, 3, l12_20_t, sat_accuracy[11]),
  GEN_I(45, 3, l12_20_t, sat_accuracy[12]),
  GEN_I(48, 3, l12_20_t, sat_accuracy[13]),
  GEN_I(51, 3, l12_20_t, sat_accuracy[14]),
  GEN_I(54, 3, l12_20_t, sat_accuracy[15]),
  GEN_I(57, 3, l12_20_t, sat_accuracy[16]),
};
template <class FloatT>
const typename TextHelper<>::convert_item_t SP3_Reader<FloatT>::l21_22_items[13] = {
  GEN_C(0, 2, l21_22_t, symbols),
  GEN_C(3, 2, l21_22_t, file_type),
  GEN_C(6, 2, l21_22_t, _2_characters),
  GEN_C(9, 3, l21_22_t, time_system),
  GEN_C(13, 3, l21_22_t, _3_characters),
  GEN_C(17, 4, l21_22_t, _4_characters[0]),
  GEN_C(22, 4, l21_22_t, _4_characters[1]),
  GEN_C(27, 4, l21_22_t, _4_characters[2]),
  GEN_C(32, 4, l21_22_t, _4_characters[3]),
  GEN_C(37, 5, l21_22_t, _5_characters[0]),
  GEN_C(43, 5, l21_22_t, _5_characters[1]),
  GEN_C(49, 5, l21_22_t, _5_characters[2]),
  GEN_C(55, 5, l21_22_t, _5_characters[3]),
};
template <class FloatT>
const typename TextHelper<>::convert_item_t SP3_Reader<FloatT>::l23_24_items[5] = {
  GEN_C(0, 2, l23_24_t, symbols),
  GEN_E(3, 10, l23_24_t, base_for_pos_vel, 7),
  GEN_E(14, 12, l23_24_t, base_for_clk_rate, 9),
  GEN_E(27, 14, l23_24_t, _14_column_float, 11),
  GEN_E(42, 18, l23_24_t, _18_column_float, 15),
};
template <class FloatT>
const typename TextHelper<>::convert_item_t SP3_Reader<FloatT>::l25_26_items[10] = {
  GEN_C(0, 2, l25_26_t, symbols),
  GEN_I(3, 4, l25_26_t, _4_column_int[0]),
  GEN_I(8, 4, l25_26_t, _4_column_int[1]),
  GEN_I(13, 4, l25_26_t, _4_column_int[2]),
  GEN_I(18, 4, l25_26_t, _4_column_int[3]),
  GEN_I(23, 6, l25_26_t, _6_column_int[0]),
  GEN_I(30, 6, l25_26_t, _6_column_int[1]),
  GEN_I(37, 6, l25_26_t, _6_column_int[2]),
  GEN_I(44, 6, l25_26_t, _6_column_int[3]),
  GEN_I(51, 9, l25_26_t, _9_column_int),
};
template <class FloatT>
const typename TextHelper<>::convert_item_t SP3_Reader<FloatT>::comment_items[2] = {
  GEN_C(0, 2, comment_t, symbols),
  GEN_C(3, 57, comment_t, comment),
};
template <class FloatT>
const typename TextHelper<>::convert_item_t SP3_Reader<FloatT>::epoch_items[7] = {
  GEN_C(0, 2, epoch_t, symbols),
  GEN_I(3, 4, epoch_t, year_start),
  GEN_I(8, 2, epoch_t, month_start),
  GEN_I(11, 2, epoch_t, day_of_month_st),
  GEN_I(14, 2, epoch_t, hour_start),
  GEN_I(17, 2, epoch_t, minute_start),
  GEN_E(20, 11, epoch_t, second_start, 8),
};
template <class FloatT>
const typename TextHelper<>::convert_item_t SP3_Reader<FloatT>::position_clock_items[14] = {
  GEN_C(0, 1, position_clock_t, symbol),
  GEN_sat(1, 3, position_clock_t, vehicle_id),
  GEN_E(4, 14, position_clock_t, coordinate_km[0], 6),
  GEN_E(18, 14, position_clock_t, coordinate_km[1], 6),
  GEN_E(32, 14, position_clock_t, coordinate_km[2], 6),
  GEN_E(46, 14, position_clock_t, clock_us, 6),
  GEN_I(61, 2, position_clock_t, sdev_b_n_mm[0]),
  GEN_I(64, 2, position_clock_t, sdev_b_n_mm[1]),
  GEN_I(67, 2, position_clock_t, sdev_b_n_mm[2]),
  GEN_I(70, 3, position_clock_t, c_sdev_b_n_psec),
  GEN_C(74, 1, position_clock_t, clock_event_flag),
  GEN_C(75, 1, position_clock_t, clock_pred_flag),
  GEN_C(78, 1, position_clock_t, maneuver_flag),
  GEN_C(79, 1, position_clock_t, orbit_pred_flag),
};
template <class FloatT>
const typename TextHelper<>::convert_item_t SP3_Reader<FloatT>::position_clock_correlation_items[11] = {
  GEN_C(0, 2, position_clock_correlation_t, symbols),
  GEN_I(4, 4, position_clock_correlation_t, sdev_mm[0]),
  GEN_I(9, 4, position_clock_correlation_t, sdev_mm[1]),
  GEN_I(14, 4, position_clock_correlation_t, sdev_mm[2]),
  GEN_I(19, 7, position_clock_correlation_t, clk_sdev_psec),
  GEN_I(27, 8, position_clock_correlation_t, xy_correlation),
  GEN_I(36, 8, position_clock_correlation_t, xz_correlation),
  GEN_I(45, 8, position_clock_correlation_t, xc_correlation),
  GEN_I(54, 8, position_clock_correlation_t, yz_correlation),
  GEN_I(63, 8, position_clock_correlation_t, yc_correlation),
  GEN_I(72, 8, position_clock_correlation_t, zc_correlation),
};
template <class FloatT>
const typename TextHelper<>::convert_item_t SP3_Reader<FloatT>::velocity_rate_items[10] = {
  GEN_C(0, 1, velocity_rate_t, symbol),
  GEN_sat(1, 3, velocity_rate_t, vehicle_id),
  GEN_E(4, 14, velocity_rate_t, velocity_dm_s[0], 6),
  GEN_E(18, 14, velocity_rate_t, velocity_dm_s[1], 6),
  GEN_E(32, 14, velocity_rate_t, velocity_dm_s[2], 6),
  GEN_E(46, 14, velocity_rate_t, clock_rate_chg_100ps_s, 6),
  GEN_I(61, 2, velocity_rate_t, vel_sdev_100nm_s[0]),
  GEN_I(64, 2, velocity_rate_t, vel_sdev_100nm_s[1]),
  GEN_I(67, 2, velocity_rate_t, vel_sdev_100nm_s[2]),
  GEN_I(70, 3, velocity_rate_t, clkrate_sdev_10_4_ps_s),
};
template <class FloatT>
const typename TextHelper<>::convert_item_t SP3_Reader<FloatT>::velocity_rate_correlation_items[11] = {
  GEN_C(0, 2, velocity_rate_correlation_t, symbols),
  GEN_I(4, 4, velocity_rate_correlation_t, vel_sdev_100nm_s[0]),
  GEN_I(9, 4, velocity_rate_correlation_t, vel_sdev_100nm_s[1]),
  GEN_I(14, 4, velocity_rate_correlation_t, vel_sdev_100nm_s[2]),
  GEN_I(19, 7, velocity_rate_correlation_t, clkrate_sdev_10_4_ps_s),
  GEN_I(27, 8, velocity_rate_correlation_t, xy_correlation),
  GEN_I(36, 8, velocity_rate_correlation_t, xz_correlation),
  GEN_I(45, 8, velocity_rate_correlation_t, xc_correlation),
  GEN_I(54, 8, velocity_rate_correlation_t, yz_correlation),
  GEN_I(63, 8, velocity_rate_correlation_t, yc_correlation),
  GEN_I(72, 8, velocity_rate_correlation_t, zc_correlation),
};

#undef GEN_C
#undef GEN_I
#undef GEN_I2
#undef GEN_E
#undef GEN_sat

template <class FloatT>
class SP3_Writer {
  public:
    typedef SP3_Reader<FloatT> reader_t;
    static std::string print_line(const typename reader_t::parsed_t &parsed){
      static const struct {
        const typename TextHelper<>::convert_item_t *items;
        int items_num;
        const char *symbol;
      } table[reader_t::parsed_t::PARSED_ITEMS] = {
        {0}, // UNKNOWN
#define MAKE_ENTRY(k, str) \
{reader_t::k, sizeof(reader_t::k) / sizeof(reader_t::k[0]), str}
        MAKE_ENTRY(l1_items, "#"), // L1
        MAKE_ENTRY(l2_items, "##"), // L2
        MAKE_ENTRY(l3_11_items, "+ "), // L3_11
        MAKE_ENTRY(l12_20_items, "++"), // L12_20
        MAKE_ENTRY(l21_22_items, "%c"), // L21_22
        MAKE_ENTRY(l23_24_items, "%f"), // L23_24
        MAKE_ENTRY(l25_26_items, "%i"), // L25_26
        MAKE_ENTRY(comment_items, "/"), // COMMENT
        MAKE_ENTRY(epoch_items, "*"), // EPOCH
        MAKE_ENTRY(position_clock_items, "P"), // POSITION_CLOCK
        MAKE_ENTRY(position_clock_correlation_items, "EP"), // POSITION_CLOCK_CORRELATION
        MAKE_ENTRY(velocity_rate_items, "V"), // VELOCITY_RATE
        MAKE_ENTRY(velocity_rate_correlation_items, "EV"), // VELOCITY_RATE_CORRELATION
#undef MAKE_ENTRY
      };

      int res_length(60);
      int items_num(table[parsed.type].items_num);
      switch(parsed.type){
        case reader_t::parsed_t::POSITION_CLOCK:
          if(parsed.item.position_clock.has_sdev){res_length = 80;}
          else{items_num = 6;}
          break;
        case reader_t::parsed_t::VELOCITY_RATE:
          if(parsed.item.velocity_rate.has_sdev){res_length = 80;}
          else{items_num = 6;}
          break;
        case reader_t::parsed_t::POSITION_CLOCK_CORRELATION:
        case reader_t::parsed_t::VELOCITY_RATE_CORRELATION:
          res_length = 80;
          break;
        default:
          break;
      }
      std::string res(res_length, ' ');
      if((items_num <= 0)
          || (!TextHelper<>::val2str(table[parsed.type].items, items_num, res, &parsed.item))){
        return std::string();
      }
      res.replace(0, std::strlen(table[parsed.type].symbol), table[parsed.type].symbol);
      return res;
    }
};


#endif /* #define __SP3_H__ */
