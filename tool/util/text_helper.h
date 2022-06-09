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
 * @brief Helper class/functions for text processing
 *
 */

#ifndef __TEXT_HELPER_H__
#define __TEXT_HELPER_H__

#include <istream>
#include <ostream>
#include <sstream>
#include <string>
#include <iomanip>
#include <ctime>
#include <cstddef>
#include <cstring>
#include <limits>

template <class U = void>
struct TextHelper {
  struct crlf_stream_t : public std::istream {
    crlf_stream_t(std::istream &is) : std::istream(is.rdbuf()) {}
    /**
     * getline() for multi-platform (in addition to \n, \r\n and \r are supported)
     *
     * @see https://stackoverflow.com/questions/6089231/getting-std-ifstream-to-handle-lf-cr-and-crlf
     * @see https://en.cppreference.com/w/cpp/io/basic_istream/getline
     * TODO gcount() mismatch
     */
    std::istream& getline(
        typename std::istream::char_type* s,
        std::streamsize n){
      std::streamsize i(1), consumed(0);
      typename std::istream::sentry se(*this, true);
      if((bool)se){
        std::streambuf* sb(this->rdbuf());
        for(; i < n; ++i){
          int c(sb->sbumpc());
          if(c == std::streambuf::traits_type::eof()){
            this->setstate(std::ios::eofbit);
            break;
          }
          ++consumed;
          if(c == '\n'){
            break;
          }else if(c == '\r'){
            if(sb->sgetc() == '\n'){
              sb->sbumpc();
              ++consumed;
            }
            break;
          }
          *(s++) = (typename std::istream::char_type)c;
        }
      }
      if(((i == 1) && (consumed == 0)) || (i == n)){
        this->setstate(std::ios::failbit);
      }else{
        *s = '\0';
      }
      return *this;
    }
  };

  template <class T, bool is_integer = std::numeric_limits<T>::is_integer>
  struct format_t {
    static bool d(
        std::string &buf, const int &offset, const int &length, void *value, const int &opt = 0, const bool &str2val = true){
      if(str2val){
        std::stringstream ss(buf.substr(offset, length));
        ss >> *(T *)value;
        return (ss.rdstate() & std::ios_base::failbit) == 0;
      }else{
        std::stringstream ss;
        ss << std::setfill(opt == 1 ? '0' : ' ') << std::right << std::setw(length) << *(T *)value;
        buf.replace(offset, length, ss.str());
        return true;
      }
    }
    static bool d_blank(
        std::string &buf, const int &offset, const int &length, void *value,
        const int &opt = 0, const bool &str2val = true){
      if((!str2val) && (*(T *)value == 0)){return true;}
      return d(buf, offset, length, value, opt, str2val);
    }
    static bool f(
        std::string &buf, const int &offset, const int &length, void *value, const int &precision = 0, const bool &str2val = true){
      if(str2val){
        std::stringstream ss(buf.substr(offset, length));
        ss >> *(T *)value;
        return (ss.rdstate() & std::ios_base::failbit) == 0;
      }else{
        std::stringstream ss;
        ss << std::setfill(' ') << std::right << std::setw(length)
            << std::setprecision(precision) << std::fixed
            << *(T *)value;
        buf.replace(offset, length, ss.str());
        return true;
      }
    }
    static bool f_dot_head(
        std::string &buf, const int &offset, const int &length, void *value, const int &precision = 0, const bool &str2val = true){
      bool res(f(buf, offset, length, value, precision, str2val));
      if((!str2val) && res){
        if((*(T *)value) >= 0){
          if((*(T *)value) < 1){
            int i(length - precision - 2);
            // 0.12345 => .12345
            if(i >= 0){buf[i + offset] = ' ';}
          }
        }else{
          if((*(T *)value) > -1){
            int i(length - precision - 2);
            // -0.12345 => -.12345
            if(i >= 0){buf[i + offset] = '-';}
            if(--i >= 0){buf[i + offset] = ' ';}
          }
        }
      }
      return res;
    }
  };
  template <class T>
  struct format_t<T, true> : public format_t<T, false> {
    static bool f(
        std::string &buf, const int &offset, const int &length, void *value, const int &precision = 0, const bool &str2val = true){
      double v(*(T *)value);
      bool res(
          format_t<double, false>::f(buf, offset, length, &v, precision, str2val));
      *(T *)value = static_cast<T>(v);
      return res;
    }
    static bool f_dot_head(
        std::string &buf, const int &offset, const int &length, void *value, const int &precision = 0, const bool &str2val = true){
      double v(*(T *)value);
      bool res(
          format_t<double, false>::f_dot_head(buf, offset, length, &v, precision, str2val));
      *(T *)value = static_cast<T>(v);
      return res;
    }
  };

  struct convert_item_t {
    bool (*func)(
        std::string &buf, const int &offset, const int &length, void *value,
        const int &opt, const bool &str2val);
    int offset;
    int length;
    int value_offset;
    int opt;
  };

  /**
   * @param recovery if conversion fails, then this functor is invoked.
   * If recovery is successfully performed, this functor should return true.
   * The return value of this function reflects it.
   * @return (bool) if all conversion are successfully performed, true is returned; otherwise false.
   */
  static bool str2val(
      const convert_item_t *items, const int &size, const std::string &buf, void *values,
      bool (*recovery)(const int &, const std::string &, void *) = NULL){
    // str => value
    bool res(true);
    for(int i(0); i < size; ++i){
      if((*items[i].func)(
          const_cast<std::string &>(buf), items[i].offset, items[i].length, (char *)values + items[i].value_offset,
          items[i].opt, true)){continue;}
      res &= (recovery ? (*recovery)(i, buf, values) : false);
    }
    return res;
  }
  template <int N>
  static inline bool str2val(
      const convert_item_t (&items)[N], const std::string &buf, void *values,
      bool (*recovery)(const int &, const std::string &, void *) = NULL){
    return str2val(items, N, buf, values, recovery);
  }

  /**
   * @return If all conversion are successfully performed, then true; otherwise false;
   */
  static bool val2str(
      const convert_item_t *items, const int &size,
      std::string &buf, const void *values){
    // value => string
    bool res(true);
    for(int i(0); i < size; ++i){
      res &= (*items[i].func)(
          buf, items[i].offset, items[i].length, (char *)(const_cast<void *>(values)) + items[i].value_offset,
          items[i].opt, false);
    }
    return res;
  }
  template <int N>
  static inline bool val2str(
      const convert_item_t (&items)[N], std::string &buf, const void *values){
    return val2str(items, N, buf, values);
  }
};

template <>
template <>
struct TextHelper<>::format_t<char, false> {
  static bool c(
      std::string &buf, const int &offset, const int &length, void *value,
      const int &opt = 0, const bool &str2val = true){
    if(str2val){
      return buf.copy(static_cast<char *>(value), length, offset) == (std::size_t)length;
    }else{
      if((length <= 0) || (!(static_cast<char *>(value)[0]))){return true;}
      buf.replace(offset, length, static_cast<char *>(value), length);
      return true;
    }
  }
};

#endif /* __TEXT_HELPER__ */
