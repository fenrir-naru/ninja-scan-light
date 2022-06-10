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

#ifndef __ANTEX_H__
#define __ANTEX_H__

#include <string>
#include <ctime>

#include <vector>
#include <map>

#include "util/text_helper.h"
#include "GPS.h"
#include "SP3.h"

template <class FloatT>
struct ANTEX_Product {
  struct per_freq_t {
    FloatT north_east_up[3];
    std::vector<FloatT> noazi; // TODO
    std::map<FloatT, std::vector<FloatT> > azi; // TODO
  };
  struct antenna_t {
    std::string type, serial;
    GPS_Time<FloatT> valid_from;
    GPS_Time<FloatT> valid_until;
    std::map<std::string, per_freq_t> freq;
    std::map<std::string, per_freq_t> freq_rms;
  };
  typedef std::vector<antenna_t> prop_t;
  prop_t prop;

  void move_to_antenna_position(SP3_Product<FloatT> &sp3) const {
    // TODO
  }
};

template <class FloatT>
class ANTEX_Reader {
  protected:
    typename TextHelper<>::crlf_stream_t src;
  public:
    ANTEX_Reader(std::istream &in) : src(in) {}

    bool has_next() {
      return !(src.eof() || src.fail());
    }

    struct type_serial_t {
      char type[20];
      char serial_or_prn[20];
      char blank_or_code[10];
      char blank_or_cospar[10];
      template <std::size_t N>
      static std::string to_string(const char (&c)[N]) {
        std::string buf(c, N);
        return buf.substr(0, buf.find_last_not_of(' ') + 1);
      }
      bool is_satellite(int *sat_id = NULL) const {
        std::string buf(to_string(serial_or_prn));
        if(buf.size() != 3){return false;}
        int dummy;
        return SP3_Reader<FloatT>::conv_t::sat_id(buf, 0, 3, (sat_id ? sat_id : &dummy));
      }
    };
    struct time_t {
      int year, month, day_of_month, hour, minute;
      FloatT second;
      operator std::tm() const {
        std::tm res = {
          (int)second, minute, hour, day_of_month, month - 1, year - 1900, 0, 0, 0
        };
        std::mktime(&res);
        return res;
      }
      operator GPS_Time<FloatT>() const {
        return GPS_Time<FloatT>(std::tm(*this));
      }
    };
    struct freq_t {
      char freq_name[3];
    };
    struct neu_t {
      FloatT values[3];
    };

    struct parsed_t {
      enum {
        ANTEX_VERSION_SYST,
        PCV_TYPE_REFANT,
        COMMENT,
        END_OF_HEADER,
        START_OF_ANTENNA,
        TYPE_SERIAL_NO,
        METH_BY_NUM_DATE,
        DAZI,
        ZEN1_ZEN2_DZEN,
        NUM_OF_FREQUENCIES,
        VALID_FROM,
        VALID_UNTIL,
        SINEX_CODE,
        START_OF_FREQUENCY,
        NORTH_EAST_UP,
        NOAZI_VALUES,
        DAZI_VALUES,
        END_OF_FREQUENCY,
        START_OF_FREQ_RMS,
        END_OF_FREQ_RMS,
        END_OF_ANTENNA,
        IGNORABLE,
        UNKNOWN,
      } type;
      union {
        type_serial_t type_serial;
        time_t time;
        freq_t freq;
        neu_t neu;
      } item;
    };

    static const typename TextHelper<>::convert_item_t type_serial_items[4];
    static const typename TextHelper<>::convert_item_t time_items[6];
    static const typename TextHelper<>::convert_item_t freq_items[1];
    static const typename TextHelper<>::convert_item_t neu_items[3];

    parsed_t parse_line() {
      parsed_t res = {parsed_t::UNKNOWN, {0}};

      char buf[0x400] = {0};
      src.getline(buf, sizeof(buf));
      std::string line(buf);

      if(line.size() == 0){
        res.type = parsed_t::IGNORABLE;
        return res;
      }
      line = line.substr(0, line.find_last_not_of(" ") + 1);

      if(line.size() > 60){
        std::string label(line.substr(60));

        if(label.compare("ANTEX VERSION / SYST") == 0){
          res.type = parsed_t::ANTEX_VERSION_SYST;
          return res;
        }
        if(label.compare("PCV TYPE / REFANT") == 0){
          res.type = parsed_t::PCV_TYPE_REFANT;
          return res;
        }
        if(label.compare("COMMENT") == 0){
          res.type = parsed_t::COMMENT;
          return res;
        }
        if(label.compare("END OF HEADER") == 0){
          res.type = parsed_t::END_OF_HEADER;
          return res;
        }
        if(label.compare("START OF ANTENNA") == 0){
          res.type = parsed_t::START_OF_ANTENNA;
          return res;
        }
        if(label.compare("TYPE / SERIAL NO") == 0){
          TextHelper<>::str2val(type_serial_items, line, &res.item);
          res.type = parsed_t::TYPE_SERIAL_NO;
          return res;
        }
        if(label.compare("METH / BY / # / DATE") == 0){
          res.type = parsed_t::METH_BY_NUM_DATE;
          return res;
        }
        if(label.compare("DAZI") == 0){
          res.type = parsed_t::DAZI;
          return res;
        }
        if(label.compare("ZEN1 / ZEN2 / DZEN") == 0){
          res.type = parsed_t::ZEN1_ZEN2_DZEN;
          return res;
        }
        if(label.compare("# OF FREQUENCIES") == 0){
          res.type = parsed_t::NUM_OF_FREQUENCIES;
          return res;
        }
        if(label.compare("VALID FROM") == 0){
          TextHelper<>::str2val(time_items, line, &res.item);
          res.type = parsed_t::VALID_FROM;
          return res;
        }
        if(label.compare("VALID UNTIL") == 0){
          TextHelper<>::str2val(time_items, line, &res.item);
          res.type = parsed_t::VALID_UNTIL;
          return res;
        }
        if(label.compare("SINEX CODE") == 0){
          res.type = parsed_t::SINEX_CODE;
          return res;
        }
        if(label.compare("START OF FREQUENCY") == 0){
          TextHelper<>::str2val(freq_items, line, &res.item);
          res.type = parsed_t::START_OF_FREQUENCY;
          return res;
        }
        if(label.compare("NORTH / EAST / UP") == 0){
          TextHelper<>::str2val(neu_items, line, &res.item);
          res.type = parsed_t::NORTH_EAST_UP;
          return res;
        }
        if(label.compare("END OF FREQUENCY") == 0){
          res.type = parsed_t::END_OF_FREQUENCY;
          return res;
        }
        if(label.compare("START OF FREQ RMS") == 0){
          TextHelper<>::str2val(freq_items, line, &res.item);
          res.type = parsed_t::START_OF_FREQ_RMS;
          return res;
        }
        if(label.compare("END OF FREQ RMS") == 0){
          res.type = parsed_t::END_OF_FREQ_RMS;
          return res;
        }
        if(label.compare("END OF ANTENNA") == 0){
          res.type = parsed_t::END_OF_ANTENNA;
          return res;
        }
      }
      if(line.substr(3, 5).compare("NOAZI") == 0){
        // (Values of a non-azimuth-dependent pattern)
        // or (Rms values of a non-azimuth-dependent pattern)
        res.type = parsed_t::NOAZI_VALUES;
        return res;
      }
      for(double d; !!(std::stringstream(line.substr(0, 8)) >> d); ){
        // (Values of an azimuth-dependent pattern)
        // or (Rms Values of an azimuth-dependent pattern)
        res.type = parsed_t::DAZI_VALUES;
        return res;
      }

      res.type = parsed_t::UNKNOWN;
      return res;
    }

    static int read_all(std::istream &in, ANTEX_Product<FloatT> &dst) {
      ANTEX_Reader<FloatT> src(in);
      enum {
        INITIAL_STATE = 1,
        ON_HEADER,
        FREE_STATE,
        ON_ANTENNA,
        ON_FREQUENCY,
        ON_FREQUENCY_RMS,
      } state = INITIAL_STATE;

      typedef typename ANTEX_Product<FloatT>::antenna_t antenna_t;
      antenna_t *antenna;
      typedef typename ANTEX_Product<FloatT>::per_freq_t per_freq_t;
      per_freq_t *per_freq;

      while(src.has_next()){
        parsed_t parsed(src.parse_line());
        if(parsed.type == parsed_t::IGNORABLE){continue;}
        switch(state){
          case INITIAL_STATE:
            if(parsed.type != parsed_t::ANTEX_VERSION_SYST){
              return -INITIAL_STATE; // error
            }
            state = ON_HEADER;
            break;
          case ON_HEADER:
            switch(parsed.type){
              case parsed_t::COMMENT: break;
              case parsed_t::PCV_TYPE_REFANT: break;
              case parsed_t::END_OF_HEADER:
                state = FREE_STATE;
                break;
              default: // error
                return -ON_HEADER;
            }
            break;
          case FREE_STATE:
            if(parsed.type != parsed_t::START_OF_ANTENNA){
              return -FREE_STATE; // error
            }
            state = ON_ANTENNA;
            dst.prop.resize(dst.prop.size() + 1);
            antenna = &dst.prop.back();
            break;
          case ON_ANTENNA:
            switch(parsed.type){
              case parsed_t::TYPE_SERIAL_NO: {
                antenna->type = type_serial_t::to_string(parsed.item.type_serial.type);
                antenna->serial = type_serial_t::to_string(parsed.item.type_serial.serial_or_prn);
                antenna->valid_until = GPS_Time<FloatT>::now();
                break;
              }
              case parsed_t::METH_BY_NUM_DATE: break;
              case parsed_t::DAZI: break;
              case parsed_t::ZEN1_ZEN2_DZEN: break;
              case parsed_t::NUM_OF_FREQUENCIES: break;
              case parsed_t::VALID_FROM:
                antenna->valid_from = parsed.item.time;
                break;
              case parsed_t::VALID_UNTIL:
                antenna->valid_until = parsed.item.time;
                break;
              case parsed_t::SINEX_CODE: break;
              case parsed_t::COMMENT: break;
              case parsed_t::START_OF_FREQUENCY:
                state = ON_FREQUENCY;
                per_freq = &antenna->freq[
                     std::string(parsed.item.freq.freq_name, sizeof(parsed.item.freq.freq_name))];
                break;
              case parsed_t::START_OF_FREQ_RMS:
                per_freq = &antenna->freq_rms[
                     std::string(parsed.item.freq.freq_name, sizeof(parsed.item.freq.freq_name))];
                state = ON_FREQUENCY_RMS;
                break;
              case parsed_t::END_OF_ANTENNA:
                state = FREE_STATE;
                break;
              default: // error
                return -ON_ANTENNA;
            }
            break;
          case ON_FREQUENCY:
          case ON_FREQUENCY_RMS:
            switch(parsed.type){
              case parsed_t::NORTH_EAST_UP:
                for(std::size_t i(0);
                    i < sizeof(per_freq->north_east_up) / sizeof(per_freq->north_east_up[0]);
                    ++ i){
                  per_freq->north_east_up[i] = parsed.item.neu.values[i];
                }
                break;
              case parsed_t::NOAZI_VALUES: break; // TODO variable size, implement callback of parse_line()?
              case parsed_t::DAZI_VALUES: break; // TODO
              case parsed_t::END_OF_FREQUENCY:
                if(state == ON_FREQUENCY){state = ON_ANTENNA; break;}
                return -state;
              case parsed_t::END_OF_FREQ_RMS:
                if(state != ON_FREQUENCY_RMS){state = ON_ANTENNA; break;}
                return -state;
              default: // error
                return -state;
            }
            break;
        }
      }
      return dst.prop.size();
    }
};

#define GEN_C(offset, length, container_type, container_member) \
    {TextHelper<>::template format_t<char>::c, offset, length, \
      offsetof(container_type, container_member), 0}
#define GEN_I(offset, length, container_type, container_member) \
    {TextHelper<>::template format_t<int>::d, offset, length, \
      offsetof(container_type, container_member), 0}
#define GEN_F(offset, length, container_type, container_member, precision) \
    {TextHelper<>::template format_t<FloatT>::f, offset, length, \
      offsetof(container_type, container_member), precision}

template <class FloatT>
const typename TextHelper<>::convert_item_t ANTEX_Reader<FloatT>::type_serial_items[4] = {
  GEN_C( 0, 20, type_serial_t, type), // not null-terminated, following the same
  GEN_C(20, 20, type_serial_t, serial_or_prn),
  GEN_C(40, 10, type_serial_t, blank_or_code),
  GEN_C(50, 10, type_serial_t, blank_or_cospar),
};

template <class FloatT>
const typename TextHelper<>::convert_item_t ANTEX_Reader<FloatT>::time_items[6] = {
  GEN_I( 0, 6, time_t, year),
  GEN_I( 6, 6, time_t, month),
  GEN_I(12, 6, time_t, day_of_month),
  GEN_I(18, 6, time_t, hour),
  GEN_I(24, 6, time_t, minute),
  GEN_F(30, 13, time_t, second, 7),
};

template <class FloatT>
const typename TextHelper<>::convert_item_t ANTEX_Reader<FloatT>::freq_items[1] = {
  GEN_C( 3,  3, freq_t, freq_name),
};

template <class FloatT>
const typename TextHelper<>::convert_item_t ANTEX_Reader<FloatT>::neu_items[3] = {
  GEN_F( 0, 10, neu_t, values[0], 2),
  GEN_F(10, 10, neu_t, values[1], 2),
  GEN_F(20, 10, neu_t, values[2], 2),
};

#undef GEN_C
#undef GEN_I
#undef GEN_F

#endif /* #define __ANTEX_H__ */
