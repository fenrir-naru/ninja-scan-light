/*
 * Copyright (c) 2016, M.Naruoka (fenrir)
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

#ifndef __GPS_H__
#define __GPS_H__

/** @file
 * @brief GPS ICD definitions including C/A code, time, ephemeris, ...
 */

#include <map>
#include <bitset>
#include <ctime>
#define _USE_MATH_DEFINES
#include <cmath>

#include "WGS84.h"

#include "coordinate.h"

#ifdef pow2
#define POW2_ALREADY_DEFINED
#else
#define pow2(x) ((x)*(x))
#endif
#ifdef pow3
#define POW3_ALREADY_DEFINED
#else
#define pow3(x) ((x)*(x)*(x))
#endif

template <class FloatT = double>
class CA_Code {
  public:
    static const FloatT FREQENCY;
    static const FloatT LENGTH_1CHIP;
  public:
    class PRN {
      public:
        typedef std::bitset<10> content_t;
      protected:
        content_t _content;
      public:
        void reset(){_content.set();}
        PRN() : _content() {reset();}
        virtual ~PRN(){}
        virtual bool get() const = 0;
        virtual void next() = 0;
        friend std::ostream &operator<<(std::ostream &out, const PRN &prn){
          out << const_cast<PRN *>(&prn)->_content;
          return out;
        }
    };
    
    class G1 : public PRN {
      public:
        G1() : PRN() {}
        ~G1(){}
        bool get() const {return PRN::_content[9];}
        void next(){
          bool tmp(PRN::_content[2] ^ PRN::_content[9]);
          PRN::_content <<= 1;
          PRN::_content[0] = tmp;
        }
    };
    
    class G2 : public PRN {
      protected:
        const int _selector1, _selector2;
      public:
        G2(int selector1, int selector2) : PRN(), _selector1(selector1), _selector2(selector2) {}
        ~G2(){}
        bool get() const {return PRN::_content[_selector1] ^ PRN::_content[_selector2];}
        void next(){
          bool tmp(PRN::_content[1] 
                     ^ PRN::_content[2]
                     ^ PRN::_content[5]
                     ^ PRN::_content[7]
                     ^ PRN::_content[8]
                     ^ PRN::_content[9]);
          PRN::_content <<= 1;
          PRN::_content[0] = tmp;
        }
        static G2 Satellite_G2(int satellite_ID){
          switch(satellite_ID){
            case  1: return G2(1, 5);
            case  2: return G2(2, 6);
            case  3: return G2(3, 7);
            case  4: return G2(4, 8);
            case  5: return G2(0, 8);
            case  6: return G2(1, 9);
            case  7: return G2(0, 7);
            case  8: return G2(1, 8);
            case  9: return G2(2, 9);
            case 10: return G2(1, 2);
            case 11: return G2(2, 3);
            case 12: return G2(4, 5);
            case 13: return G2(5, 6);
            case 14: return G2(6, 7);
            case 15: return G2(7, 8);
            case 16: return G2(8, 9);
            case 17: return G2(0, 3);
            case 18: return G2(1, 4);
            case 19: return G2(2, 5);
            case 20: return G2(3, 6);
            case 21: return G2(4, 7);
            case 22: return G2(5, 8);
            case 23: return G2(0, 2);
            case 24: return G2(3, 5);
            case 25: return G2(4, 6);
            case 26: return G2(5, 7);
            case 27: return G2(6, 8);
            case 28: return G2(7, 9);
            case 29: return G2(0, 5);
            case 30: return G2(1, 6);
            case 31: return G2(2, 7);
            case 32: return G2(3, 8);
            case 33: return G2(4, 9);
            case 34: return G2(3, 9);
            case 35: return G2(0, 6);
            case 36: return G2(1, 7);
            default:  return G2(3, 9);
          }
        }
    };
  protected:
    G1 _g1;
    G2 _g2;
  public:
    CA_Code(int sid) : _g1(), _g2(G2::Satellite_G2(sid)){}
    ~CA_Code(){}
    bool get() const {return _g1.get() ^ _g2.get();}
    int get_multi() const {return get() ? 1 : -1;}
    void next(){
      _g1.next();
      _g2.next();
    }
};

template <class FloatT>
const FloatT CA_Code<FloatT>::FREQENCY = 1.023E6;
template <class FloatT>
const FloatT CA_Code<FloatT>::LENGTH_1CHIP = 1. / CA_Code<FloatT>::FREQENCY;

template <class FloatT = double>
struct GPS_Time {
  static const unsigned int seconds_day = 60U * 60 * 24;
  static const unsigned int seconds_week = seconds_day * 7;
  
  static const int days_of_month[];
  
  int week;
  FloatT seconds;
  
  GPS_Time() {}
  GPS_Time(const GPS_Time &t)
      : week(t.week), seconds(t.seconds) {}
  GPS_Time(const int &_week, const FloatT &_seconds)
      : week(_week), seconds(_seconds) {}
  void canonicalize(){
    if(seconds >= seconds_week){
      int quot((int)(seconds / seconds_week));
      week += quot;
      seconds -= quot * seconds_week;
    }else if(seconds < 0){
      int quot((int)(-seconds / seconds_week) + 1);
      week -= quot;
      seconds += quot * seconds_week;
    }
  }
  GPS_Time(const struct tm &t, const FloatT leap_seconds = 0) {
    int days(-6);
    int y(t.tm_year - 80);
    if(y < 0){y += 100;}
    days += y * 365 + ((y + 3) / 4);
    for(int i(0); i < t.tm_mon; i++){
      days += days_of_month[i];
      if((i == 2) && (t.tm_year % 4 == 0)) days++;
    }
    days += t.tm_mday;
    
    week = days / 7;
    seconds = leap_seconds + (days % 7) * seconds_day
        + t.tm_hour * 60 * 60
        + t.tm_min * 60
        + t.tm_sec;
    canonicalize();
  }
  static GPS_Time now(const FloatT leap_seconds = 0) {
    time_t timer;
    struct tm t;
    
    time(&timer); // Current serial time, then convert the time to struct
#if defined(_MSC_VER)
    gmtime_s(&t, &timer);
#elif defined(__GNUC__)
    gmtime_r(&timer, &t);
#else
    t = *gmtime(&timer);
#endif
    
    return GPS_Time(t, leap_seconds);
  }
  
  GPS_Time &operator+=(const FloatT &sec){
    seconds += sec;
    canonicalize();
    return *this;
  }
  GPS_Time &operator-=(const FloatT &sec){
    return operator+=(-sec);
  }
  GPS_Time operator+(const FloatT &sec) const {
    GPS_Time t(*this);
    return (t += sec);
  }
  GPS_Time operator-(const FloatT &sec) const {
    return operator+(-sec);
  }
  /**
   * Get interval in unit of second
   */
  FloatT operator-(const GPS_Time &t) const {
    FloatT res(seconds - t.seconds);
    res += (week - t.week) * seconds_week;
    return res;
  }
  friend FloatT operator+(FloatT v, const GPS_Time &t){
    return v + ((t.week * seconds_week) + t.seconds);
  }
  friend FloatT operator-(FloatT v, const GPS_Time &t){
    return v - ((t.week * seconds_week) + t.seconds);
  }
  bool operator<(const GPS_Time &t) const {
    return ((week < t.week) ? true : ((week > t.week) ? false : (seconds < t.seconds)));
  }
  bool operator>(const GPS_Time &t) const {
    return t.operator<(*this);
  }
  bool operator==(const GPS_Time &t) const {
    return (week == t.week) && (seconds == t.seconds);
  }
  bool operator!=(const GPS_Time &t) const {
    return !(operator==(t));
  }
  bool operator<=(const GPS_Time &t) const {
    return ((week < t.week) ? true : ((week > t.week) ? false : (seconds <= t.seconds)));
  }
  bool operator>=(const GPS_Time &t) const {
    return t.operator<=(*this);
  }
  
  struct tm c_tm(const FloatT leap_seconds = 0) const {
    struct tm t;
    
    GPS_Time mod_t((*this) + leap_seconds);
    
    t.tm_min = (int)(mod_t.seconds / 60);  t.tm_sec = (int)(mod_t.seconds - t.tm_min * 60);
    t.tm_hour = (t.tm_min / 60);      t.tm_min -= t.tm_hour * 60;
    t.tm_mday = t.tm_hour / 24;       t.tm_hour -= t.tm_mday * 24;
    t.tm_wday = t.tm_mday;  
    t.tm_mday += 6 + (mod_t.week * 7);
    t.tm_year = (t.tm_mday / (365 * 3 + 366) * 4);
    t.tm_mday -= (t.tm_year / 4) * (365 * 3 + 366); 
    if(t.tm_mday > 366){t.tm_mday -= 366; (t.tm_year)++;}
    for(int i(0); i < 2; i++){
      if(t.tm_mday > 365){t.tm_mday -= 365; (t.tm_year)++;}
    }
    t.tm_yday = t.tm_mday;
    (t.tm_year += 80) %= 100;
    for(t.tm_mon = 0; 
        t.tm_mday > days_of_month[t.tm_mon];
        (t.tm_mon)++){
      if((t.tm_mon == 1) && (t.tm_year % 4 == 0)){
        if(t.tm_mday == 29){break;}
        else{t.tm_mday--;}
      }
      t.tm_mday -= days_of_month[t.tm_mon];
    }
    t.tm_isdst = 0;
    
    return t;
  }
    
  
  /**
   * When t >= self positive value will be returned,
   * otherwise(t  < self), negative.
   */
  FloatT interval(const unsigned int &t_week, 
      const FloatT &t_seconds) const {
    return t_seconds - seconds
        + (t_week - week) * seconds_week;
  }
  FloatT interval(const GPS_Time &t) const {
    return interval(t.week, t.seconds);
  }

  friend std::ostream &operator<<(std::ostream &out, const GPS_Time &t){
    out << t.week << " week " << t.seconds << " sec.";
    return out;
  }
};

template <class FloatT>
const int GPS_Time<FloatT>::days_of_month[] = {
      31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
    };

template <class FloatT = double>
class GPS_SpaceNode {
  public:
    static const FloatT light_speed;
    static const FloatT L1_Frequency;
    static const FloatT SC2RAD;
    
  protected:
    static FloatT rad2sc(const FloatT &rad) {return rad / M_PI;}
    static FloatT sc2rad(const FloatT &sc)  {return sc * M_PI;}
    
  public:
    typedef GPS_SpaceNode<FloatT> self_t;
    typedef GPS_Time<FloatT> gps_time_t; 
    typedef System_XYZ<FloatT, WGS84> xyz_t;
    typedef System_LLH<FloatT, WGS84> llh_t;
    typedef System_ENU<FloatT, WGS84> enu_t;
  protected:
    typedef unsigned char u8_t;
    typedef signed char s8_t;
    typedef unsigned short u16_t;
    typedef signed short s16_t;
    typedef unsigned int u32_t;
    typedef signed int s32_t;
    
    typedef int int_t;
    typedef unsigned int uint_t;
  public:
    /**
     * GPS Ionospheric correction parameters
     * 
     */
    struct IonosphericDelayCoef {
      FloatT alpha0;       ///< Ionospheric parameter (s)
      FloatT alpha1;       ///< Ionospheric parameter (s/sc)
      FloatT alpha2;       ///< Ionospheric parameter (s/sc^2)
      FloatT alpha3;       ///< Ionospheric parameter (s/sc^3)
      FloatT beta0;        ///< Ionospheric parameter (s)
      FloatT beta1;        ///< Ionospheric parameter (s/sc)
      FloatT beta2;        ///< Ionospheric parameter (s/sc^2)
      FloatT beta3;        ///< Ionospheric parameter (s/sc^3)
      FloatT A1;           ///< UTC parameter (s/s)
      FloatT A0;           ///< UTC parameter (s)
      FloatT t_ot;         ///< Epoch time (UTC) (s)
      uint_t wn_t;         ///< Epoch time (UTC) (weeks)
      FloatT delte_t_LS;   ///< Current leap seconds (s)
      uint_t wn_LSF;       ///< Last leap second update week (weeks)
      uint_t DN;           ///< Last leap second update day (days)
      uint_t delta_t_LSF;  ///< Updated leap seconds (s)
      
      struct raw_t {
        s8_t  alpha0;       ///< Ionospheric parameter (-30, s)
        s8_t  alpha1;       ///< Ionospheric parameter (-27, s/sc)
        s8_t  alpha2;       ///< Ionospheric parameter (-24, s/sc^2)
        s8_t  alpha3;       ///< Ionospheric parameter (-24, s/sc^3)
        s8_t  beta0;        ///< Ionospheric parameter (11, s)
        s8_t  beta1;        ///< Ionospheric parameter (14, s/sc)
        s8_t  beta2;        ///< Ionospheric parameter (16, s/sc^2)
        s8_t  beta3;        ///< Ionospheric parameter (16, s/sc^3)
        s32_t A1;           ///< UTC parameter (-50, s/s)
        s32_t A0;           ///< UTC parameter (-30, s)
        u8_t  t_ot;         ///< Epoch time (UTC) (12E0, s)
        u8_t  WN_t;         ///< Epoch time (UTC) (weeks)
        u8_t  delte_t_LS;   ///< Current leap seconds (s)
        u8_t  WN_LSF;       ///< Last leap second update week (weeks)
        u8_t  DN;           ///< Last leap second update day (days)
        u8_t  delta_t_LSF;  ///< Updated leap seconds (s)
        
        IonosphericDelayCoef convert() const {
          IonosphericDelayCoef converted;
#define CONVERT(TARGET, SF) \
{converted.TARGET = SF * TARGET;}
#define POWER_2(n) \
(((n) >= 0) \
  ? (FloatT)(1 << (n)) \
  : (((FloatT)1) / (1 << (-(n) >= 30 ? 30 : -(n))) \
    / (1 << (-(n) >= 30 ? (-(n) - 30) : 0))))
            CONVERT(alpha0,     POWER_2(-30));
            CONVERT(alpha1,     POWER_2(-27));
            CONVERT(alpha2,     POWER_2(-24));
            CONVERT(alpha3,     POWER_2(-24));
            CONVERT(beta0,      POWER_2( 11));
            CONVERT(beta1,      POWER_2( 14));
            CONVERT(beta2,      POWER_2( 16));
            CONVERT(beta3,      POWER_2( 16));
            CONVERT(A1,         POWER_2(-50));
            CONVERT(A0,         POWER_2(-30));
            CONVERT(t_ot,       12E0);
            converted.WN_t = WN_t;
            CONVERT(delte_t_LS, 1E0)
            converted.WN_LSF = WN_LSF;
            converted.DN = DN;
            converted.delta_t_LSF = delta_t_LSF;
#undef POWER_2
#undef CONVERT
          return converted;
        };
      };
    };
  public:
    class Satellite {
      public:
        
        /**
         * GPS ephemeris
         * (Subframe 1,2,3)
         * 
         */
        struct Ephemeris {
          uint_t svid;        ///< Satellite number
          
          // Subframe.1
          uint_t WN;          ///< Week number
          int_t URA;          ///< User range accuracy
          uint_t SV_health;   ///< Health status
          int_t iodc;         ///< Issue of clock data
          FloatT t_GD;        ///< Group delay (s)
          FloatT t_oc;        ///< Clock data reference time
          FloatT a_f2;        ///< Clock correction parameter (s/s^2)
          FloatT a_f1;        ///< Clock correction parameter (s/s)
          FloatT a_f0;        ///< Clock correction parameter (s)
          
          // Subframe.2
          int_t iode;          ///< Issue of ephemeris data
          FloatT c_rs;         ///< Sine correction, orbit (m)
          FloatT delta_n;      ///< Mean motion difference (rad/s)
          FloatT m0;           ///< Mean anomaly (rad)
          FloatT c_uc;         ///< Cosine correction, latitude (rad)
          FloatT e;            ///< Eccentricity
          FloatT c_us;         ///< Sine correction, latitude (rad)
          FloatT sqrt_A;       ///< Square root of semi-major axis (sqrt(m))
          FloatT t_oe;         ///< Reference time ephemeris (s)
          FloatT fit_interval; ///< Fit interval
          
          // Subframe.3
          FloatT c_ic;         ///< Cosine correction, inclination (rad)
          FloatT Omega0;       ///< Longitude of ascending node (rad)
          FloatT c_is;         ///< Sine correction, inclination (rad)
          FloatT i0;           ///< Inclination angle (rad)
          FloatT c_rc;         ///< Cosine correction, orbit (m)
          FloatT omega;        ///< Argument of perigee (rad)
          FloatT dot_Omega0;   ///< Rate of right ascension (rad/s)
          FloatT dot_i0;       ///< Rate of inclination angle (rad/s)
          
          FloatT period_from_time_of_clock(const gps_time_t &t) const {
            return -t.interval(WN, t_oc);
          }
        
          FloatT period_from_time_of_ephemeris(const gps_time_t &t) const {
            return -t.interval(WN, t_oe);
          }
          
          /**
           * Return true when valid
           * 
           * @param t GPS time
           */
          bool is_valid(const gps_time_t &t) const {
            return (_abs(t.interval(WN, t_oc)) <= (fit_interval / 2));
          }
          
          /**
           * Return true when newer ephemeris may be available
           * 
           * @param t GPS time
           */
          bool maybe_newer_one_avilable(const gps_time_t &t) const {
            FloatT interval_to_t_oc(t.interval(WN, t_oc));
            return !((0 <= interval_to_t_oc) && (interval_to_t_oc <= (fit_interval / 2)));
          }
          
          struct raw_t {
            u8_t  svid;         ///< Satellite number
            
            u16_t WN;           ///< Week number
            u8_t  URA;          ///< User range accuracy
            u8_t  SV_health;    ///< Health status
            u16_t iodc;         ///< Issue of clock data
            s8_t  t_GD;         ///< Group delay (-31, s)
            u16_t t_oc;         ///< Clock data reference time
            s8_t  a_f2;         ///< Clock correction parameter (-55, s/s^2)
            s16_t a_f1;         ///< Clock correction parameter (-43, s/s)
            s16_t a_f0;         ///< Clock correction parameter (-31, s)
            
            u8_t  iode;         ///< Issue of eph. data
            s16_t c_rs;         ///< Sin. correction, orbit ( -5, m)
            s16_t delta_n;      ///< Mean motion difference (-43, sc/s)
            s32_t m0;           ///< Mean anomaly           (-31, sc)
            s16_t c_uc;         ///< Cos. correction, lat.  (-29, rad)
            u32_t e;            ///< Eccentricity           (-33)
            s16_t c_us;         ///< Sin. correction, lat.  (-29, rad)
            u32_t sqrt_A;       ///< Root semi-Major Axis   (-19, sqrt(m))
            u16_t t_oe;         ///< Reference time eph.    (  4, s)
            bool fit_interval_flag;   ///< Fit interval flag (ICD:20.3.4.4)
            
            s16_t c_ic;         ///< Cos. correction, incl. (-29, rad)
            s32_t Omega0;       ///< Ascending node long.   (-31, sc)
            s16_t c_is;         ///< Sin. correction, incl. (-29, rad)
            s32_t i0;           ///< Inclination angle      (-31, sc)
            s16_t c_rc;         ///< Cos. correction, orbit ( -5, m)
            s32_t omega;        ///< Argument of perigee    (-31, sc)
            s32_t dot_Omega0;   ///< Right ascension rate   (-43, sc/s)
            s16_t dot_i0;       ///< Inclination angle rate (-43, sc/s)
            
            static FloatT fit_interval(const bool &_flag, const u16_t &_iodc){
              // Fit interval (ICD:20.3.4.4)
              if(_flag == false){
                // normal operation
                return 4 * 60 * 60;
              }else{
                // short/long-term extended operation
                if(_iodc >= 240 && _iodc <= 247){
                  return 8 * 60 * 60;
                }else if((_iodc >= 248 && _iodc <= 255) || (_iodc == 496)){
                  return 14 * 60 * 60;
                }else if(_iodc >= 497 && _iodc <= 503){
                  return 26 * 60 * 60;
                }else if(_iodc >= 504 && _iodc <= 510){
                  return 50 * 60 * 60;
                }else if((_iodc == 511) || (_iodc >= 752 && _iodc <= 756)){
                  return 74 * 60 * 60;
                }else if(_iodc >= 757 && _iodc <= 763){
                  return 98 * 60 * 60;
                }else if((_iodc >= 764 && _iodc <= 767) || (_iodc >= 1008 && _iodc <= 1010)){
                  return 122 * 60 * 60;
                }else if(_iodc >= 1011 && _iodc <= 1020){
                  return 146 * 60 * 60;
                }else{
                  return 6 * 60 * 60;
                }
              }
            }

            Ephemeris convert() const {
              Ephemeris converted;
#define CONVERT(TARGET, SF) \
{converted.TARGET = SF * TARGET;}
#define POWER_2(n) \
(((n) >= 0) \
  ? (FloatT)(1 << (n)) \
  : (((FloatT)1) / (1 << (-(n) >= 30 ? 30 : -(n))) \
    / (1 << (-(n) >= 30 ? (-(n) - 30) : 0))))
              converted.svid = svid;
              
              converted.WN = WN;
              converted.URA = URA;
              converted.SV_health = SV_health;
              converted.iodc = iodc;
              CONVERT(t_GD, POWER_2(-31));
              converted.t_oc = t_oc;
              CONVERT(a_f0, POWER_2(-55));
              CONVERT(a_f1, POWER_2(-43));
              CONVERT(a_f2, POWER_2(-31));
              
              converted.iode = iode;
              CONVERT(c_rs,       POWER_2(-5));
              CONVERT(delta_n,    SC2RAD * POWER_2(-43));
              CONVERT(m0,         SC2RAD * POWER_2(-31));
              CONVERT(c_uc,       POWER_2(-29));
              CONVERT(e,          POWER_2(-33));
              CONVERT(c_us,       POWER_2(-29));
              CONVERT(sqrt_A,     POWER_2(-19));
              CONVERT(t_oe,       POWER_2(4));
              
              CONVERT(c_ic,       POWER_2(-29));
              CONVERT(Omega0,     SC2RAD * POWER_2(-31));
              CONVERT(c_is,       POWER_2(-29));
              CONVERT(i0,         SC2RAD * POWER_2(-31));
              CONVERT(c_rc,       POWER_2(-5));
              CONVERT(omega,      SC2RAD * POWER_2(-31));
              CONVERT(dot_Omega0, SC2RAD * POWER_2(-43));
              CONVERT(dot_i0,     SC2RAD * POWER_2(-43));
#undef POWER_2
#undef CONVERT
              converted.fit_interval = fit_interval(fit_interval_flag, iodc);
              
              return converted;
            }
          };
        };
        
        /**
         * GPS almanac
         * (Subframe 4,5)
         * 
         */
        struct Almanac {
          uint_t  svid;        ///< Satellite number
          
          FloatT e;            ///< Eccentricity
          FloatT t_oa;         ///< Almanac reference time (s)
          FloatT delta_i;      ///< Correction to inclination (rad)
          FloatT dot_Omega0;   ///< Omega0 rate (rad/s)
          uint_t SV_health;    ///< Health status
          FloatT sqrt_A;       ///< Square root of semi-major axis (sqrt(m))
          FloatT Omega0;       ///< Longitude of ascending node (rad)
          FloatT omega;        ///< Argument of perigee (rad)
          FloatT M0;           ///< Mean anomaly (rad)
          FloatT a_f0;         ///< Clock correction parameter (s/s)
          FloatT a_f1;         ///< Clock correction parameter (s)
          
          /**
           * Up-cast to ephemeris
           * 
           */
          operator Ephemeris() const {
            Ephemeris converted;
            
            converted.svid          = svid;
            
            // Subframe.1
            converted.WN            = 0;          // Week number (must be configured later)
            converted.URA           = -1;         // User range accuracy
            converted.SV_health     = SV_health;  // Health status
            converted.iodc          = -1;         // Issue of clock data
            converted.t_GD          = 0;          // Group delay (s)
            converted.t_oc          = t_oa;       // Clock data reference time
            converted.a_f2          = 0;          // Clock correction parameter (s/s^2)
            converted.a_f1          = a_f1;       // Clock correction parameter (s/s)
            converted.a_f0          = a_f0;       // Clock correction parameter (s)
            
            // Subframe.2
            converted.iode          = -1;         // Issue of ephemeris data
            converted.c_rs          = 0;          // Sine correction, orbit (m)
            converted.delta_n       = 0;          // Mean motion difference (rad/s)
            converted.m0            = M0;         // Mean anomaly (rad)
            converted.c_uc          = 0;          // Cosine correction, latitude (rad)
            converted.e             = e;          // Eccentricity
            converted.c_us          = 0;          // Sine correction, latitude (rad)
            converted.sqrt_A        = sqrt_A;     // Square root of semi-major axis (sqrt(m))
            converted.t_oe          = t_oa;       // Reference time ephemeris (s)
            converted.fit_interval  = 4 * 60 * 60;// Fit interval
            
            // Subframe.3
            converted.c_ic          = 0;          // Cosine correction, inclination (rad)
            converted.Omega0        = Omega0;     // Longitude of ascending node (rad)
            converted.c_is          = 0;          // Sine correction, inclination (rad)
            converted.i0            = delta_i;    // Inclination angle (rad)
            converted.c_rc          = 0;          // Cosine correction, orbit (m)
            converted.omega         = omega;      // Argument of perigee (rad)
            converted.dot_Omega0    = dot_Omega0; // Rate of right ascension (rad/s)
            converted.dot_i0        = 0;          // Rate of inclination angle (rad/s)
            
            return converted;
          }
          
          struct raw_t {
            u8_t  svid;         ///< Satellite number
            
            u16_t e;            ///< Eccentricity       (-21)
            u8_t  t_oa;         ///< Almanac ref. time  ( 12, s)
            u16_t delta_i;      ///< Correction to inc. (-19, sc)
            u16_t dot_Omega0;   ///< Omega0 rate        (-38, sc/s)
            u8_t  SV_health;    ///< Health status
            u32_t sqrt_A;       ///< Semi-major axis    (-11, sqrt(m))
            u32_t Omega0;       ///< Long. of asc. node (-23, sc)
            u32_t omega;        ///< Arg. of perigee    (-23, sc)
            u32_t M0;           ///< Mean anomaly       (-23, sc)
            u16_t a_f0;         ///< Clock corr. param. (-20, s)
            u16_t a_f1;         ///< Clock corr. param. (-38, s)
            
            Almanac convert() const {
              Almanac converted;
#define CONVERT(TARGET, SF) \
{converted.TARGET = SF * TARGET;}
#define POWER_2(n) \
(((n) >= 0) \
  ? (FloatT)(1 << (n)) \
  : (((FloatT)1) / (1 << (-(n) >= 30 ? 30 : -(n))) \
    / (1 << (-(n) >= 30 ? (-(n) - 30) : 0))))
                converted.svid = svid;
                CONVERT(e,          POWER_2(-21));
                CONVERT(t_oa,       POWER_2(12));
                CONVERT(delta_i,    SC2RAD * POWER_2(-19));
                CONVERT(dot_Omega0, SC2RAD * POWER_2(-38));
                converted.SV_health = SV_health;
                CONVERT(sqrt_A,     POWER_2(-11));
                CONVERT(Omega0,     SC2RAD * POWER_2(-23));
                CONVERT(omega,      SC2RAD * POWER_2(-23));
                CONVERT(M0,         SC2RAD * POWER_2(-23));
                CONVERT(a_f0,       POWER_2(-20));
                CONVERT(a_f1,       POWER_2(-38));
#undef POWER_2
#undef CONVERT
              return converted;
            }
          };
        };
      protected:
        Ephemeris _ephemeris;
         
      public:
        Ephemeris &ephemeris() {return _ephemeris;}
        
        FloatT eccentric_anomaly(const FloatT &period_from_toe) const {
          // Kepler's Equation for Eccentric Anomaly M(Mk)
          FloatT n0(sqrt(WGS84::mu_Earth) / pow3(_ephemeris.sqrt_A));
          FloatT Mk(_ephemeris.m0
              + (n0 + _ephemeris.delta_n) * period_from_toe);
          
          // Eccentric Anomaly E(Ek)
          FloatT Ek(Mk);
#ifndef KEPLER_DELTA_LIMIT
#define KEPLER_DELTA_LIMIT 1E-12
#endif 
          for(int loop(0); loop < 10; loop++){
            FloatT Ek2(Mk + _ephemeris.e * sin(Ek));
            if(_abs(Ek2 - Ek) < KEPLER_DELTA_LIMIT){break;}
            Ek = Ek2;
          }
          
          return Ek;
        }
        
        FloatT eccentric_anomaly(const gps_time_t &t) const {
          return eccentric_anomaly(_ephemeris.period_from_time_of_ephemeris(t));
        }
        
        /**
         * Calculate correction value in accordance with clock error model
         *
         * @param t current time
         * @param pseudo_range pseudo range in meters
         * @return in seconds
         */
        FloatT clock_error(const gps_time_t &t, const FloatT pseudo_range = 0) const{
          FloatT tk(_ephemeris.period_from_time_of_clock(t));

          // Propagation time
          FloatT period_propagation(pseudo_range / self_t::light_speed);

          // Relativistic correction term
          FloatT tr(-2.0 * sqrt(WGS84::mu_Earth) / pow2(self_t::light_speed) 
              * _ephemeris.e * _ephemeris.sqrt_A 
              * sin(eccentric_anomaly(_ephemeris.period_from_time_of_ephemeris(t) - period_propagation)));
              
          tk -= period_propagation;
          
          FloatT dt(_ephemeris.a_f0
              + _ephemeris.a_f1 * tk
              + _ephemeris.a_f2 * pow2(tk));
          
          return dt + tr - _ephemeris.t_GD;
        }
        
        struct constellation_t {
          xyz_t pos;
          xyz_t vel;
        };
        
        constellation_t constellation(
            const gps_time_t &t, const FloatT &pseudo_range = 0,
            const bool &with_velocity = true) const {
          
          constellation_t res;
          
          // Time from ephemeris reference epoch (tk)
          FloatT tk0(_ephemeris.period_from_time_of_ephemeris(t));
          FloatT tk(tk0);

          // Remove propagation time
          tk -= pseudo_range / self_t::light_speed;

          // Eccentric Anomaly (Ek)
          FloatT Ek(eccentric_anomaly(tk));
          
          // Corrected Radius (rk)
          FloatT rk(pow2(_ephemeris.sqrt_A) 
              * (1.0 - _ephemeris.e * cos(Ek)));
          
          // True Anomaly (vk)
          FloatT vk(atan2(
              sqrt(1.0 - pow2(_ephemeris.e)) * sin(Ek),
              cos(Ek) - _ephemeris.e));
          
          // (Corrected) Argument of Latitude (pk) [rad]
          FloatT pk(vk + _ephemeris.omega);
          
          // (Corrected) Inclination (ik)
          FloatT ik(_ephemeris.i0);
          
          { // Correction
            FloatT pk2_sin(sin(pk * 2)),
                pk2_cos(cos(pk * 2));
            FloatT d_uk(
                _ephemeris.c_us * pk2_sin
                + _ephemeris.c_uc * pk2_cos);
            FloatT d_rk(
                _ephemeris.c_rs * pk2_sin
                + _ephemeris.c_rc * pk2_cos);
            FloatT d_ik(
                _ephemeris.c_is * pk2_sin
                + _ephemeris.c_ic * pk2_cos);
            
            pk += d_uk;
            rk += d_rk;
            ik += d_ik
                + _ephemeris.dot_i0 * tk;
          }
          
          // Position in orbital plane (xk, yk)
          FloatT xk(rk * cos(pk)),
              yk(rk * sin(pk));
          
          // Corrected longitude of ascending node (Omegak) [rad]
          FloatT Omegak(_ephemeris.Omega0);
          if(false){ // __MISUNDERSTANDING_ABOUT_OMEGA0_CORRECTION__
            Omegak += (
                (_ephemeris.dot_Omega0 - WGS84::Omega_Earth_IAU) * tk0
                - WGS84::Omega_Earth_IAU * _ephemeris.t_oe);
          }else{
            Omegak += (
                _ephemeris.dot_Omega0 * tk                            // corrected with the time when the wave is transmitted
                - WGS84::Omega_Earth_IAU * (_ephemeris.t_oe + tk0));  // corrected with the time when the wave is received
          }

          FloatT Omegak_sin(sin(Omegak)),
                 Omegak_cos(cos(Omegak));
          FloatT ik_sin(sin(ik)),
                 ik_cos(cos(ik));
          
          res.pos.x() = xk * Omegak_cos - yk * Omegak_sin * ik_cos;
          res.pos.y() = xk * Omegak_sin + yk * Omegak_cos * ik_cos;
          res.pos.z() = yk * ik_sin; 
          
          // Velocity calculation => GPS solution vol.8 (3) http://www.ngs.noaa.gov/gps-toolbox/bc_velo.htm
          if(with_velocity){
            FloatT n((sqrt(WGS84::mu_Earth) / pow3(_ephemeris.sqrt_A)) + _ephemeris.delta_n);
            FloatT Ek_dot(n / (1.0 - _ephemeris.e * cos(Ek)));
            FloatT vk_dot(sin(Ek) * Ek_dot * (1.0 + _ephemeris.e * cos(vk)) 
                / (sin(vk) * (1.0 - _ephemeris.e * cos(Ek))));
            
            FloatT pk2_sin(sin(pk * 2)), pk2_cos(cos(pk * 2));
            FloatT pk_dot(((_ephemeris.c_us * pk2_cos - _ephemeris.c_uc * pk2_sin) * 2 + 1.0) * vk_dot);
            FloatT rk_dot(pow2(_ephemeris.sqrt_A) * _ephemeris.e * sin(Ek) * n / (1.0 - _ephemeris.e * cos(Ek)) 
                + (_ephemeris.c_rs * pk2_cos - _ephemeris.c_rc * pk2_sin) * 2 * vk_dot);
            FloatT ik_dot(_ephemeris.dot_i0 + (_ephemeris.c_is * pk2_cos - _ephemeris.c_ic * pk2_sin) * 2 * vk_dot);
            
            // Velocity in orbital plane (xk_dot, yk_dot)
            FloatT xk_dot(rk_dot * cos(pk) - yk * pk_dot),
                yk_dot(rk_dot * sin(pk) + xk * pk_dot);
            
            FloatT Omegak_dot(_ephemeris.dot_Omega0 - WGS84::Omega_Earth_IAU);
            
            res.vel.x() = (xk_dot - yk * ik_cos * Omegak_dot) * Omegak_cos
                - (xk * Omegak_dot + yk_dot * ik_cos - yk * ik_sin * ik_dot) * Omegak_sin;
            res.vel.y() = (xk_dot - yk * ik_cos * Omegak_dot) * Omegak_sin
                + (xk * Omegak_dot + yk_dot * ik_cos - yk * ik_sin * ik_dot) * Omegak_cos;
            res.vel.z() = yk_dot * ik_sin + yk * ik_cos * ik_dot;
          }
          
          return res;
        }
        
        xyz_t whereis(const gps_time_t &t, const FloatT &pseudo_range = 0) const {
          return constellation(t, pseudo_range, false).pos;
        }
        
        xyz_t velocity(const gps_time_t &t, const FloatT &pseudo_range = 0) const {
          return constellation(t, pseudo_range, true).vel;
        }
    };
  public:
    typedef std::map<int, Satellite> satellites_t;
  protected:
    IonosphericDelayCoef _iono_coef;
    satellites_t _satellites;
  public:
    GPS_SpaceNode() : _satellites() {
    }
    ~GPS_SpaceNode(){
      _satellites.clear();
    }
    IonosphericDelayCoef &iono_coef() {
      return _iono_coef;
    }
    satellites_t &satellites() {
      return _satellites;
    }
    Satellite &satellite(int prn) {
      return _satellites[prn];
    }
    bool has_satellite(int prn) const {
      return _satellites.find(prn) !=  _satellites.end();
    }

    /**
     * Calculate correction value in accordance with ionospheric model
     * 
     * @param relative_pos satellite position (relative position, NEU)
     * @param usrllh user position (absolute position, LLH)
     * @return correction in meters
     */
    FloatT iono_correction(
        const enu_t &relative_pos,
        const llh_t &usrllh,
        const gps_time_t &t) const {
      
      
      // Elevation and azimuth
      FloatT el(relative_pos.elevation()),
             az(relative_pos.azimuth());
      FloatT sc_el(rad2sc(el)),
             sc_az(rad2sc(az));
             
      // Pierce point
      FloatT psi(0.0137 / (sc_el + 0.11) - 0.022);
      FloatT phi_i(rad2sc(const_cast<llh_t *>(&usrllh)->latitude()) 
          + psi * cos(az));
      if(phi_i > 0.416){phi_i = 0.416;}
      else if(phi_i < -0.416){phi_i = -0.416;}
      FloatT lambda_i(rad2sc(const_cast<llh_t *>(&usrllh)->longitude())
          + psi * sin(az) / cos(sc2rad(phi_i)));
      FloatT phi_m(phi_i 
          + 0.064 * cos(sc2rad(lambda_i - 1.617)));
      
      // Local time
      FloatT lt(4.32E4 * lambda_i + t.seconds);
      while(lt > gps_time_t::seconds_day){
        lt -= gps_time_t::seconds_day;
      }
      while(lt < 0){
        lt += gps_time_t::seconds_day;
      }
      
      // Period and amplitude of cosine function
      FloatT amp(0), per(0);
      const FloatT *alpha[] = {
          &(_iono_coef.alpha0), &(_iono_coef.alpha1),
          &(_iono_coef.alpha2), &(_iono_coef.alpha3)};
      const FloatT *beta[] = {
          &(_iono_coef.beta0), &(_iono_coef.beta1),
          &(_iono_coef.beta2), &(_iono_coef.beta3)};
      FloatT phi_mn(1.);
      for(int i(0); i < 4; i++){
        amp += *(alpha[i]) * phi_mn;
        per += *(beta[i]) * phi_mn;
        phi_mn *= phi_m;
      }
      if(amp < 0){amp = 0;}
      if(per < 72000){per = 72000;}
      
      // Obliquity factor
      FloatT F(1.0 + 16.0 * pow((0.53 - sc_el), 3));
      
      FloatT x(M_PI * 2 * (lt - 50400) / per);
      if(x > M_PI){
        do{x -= M_PI * 2;}while(x > M_PI);
      }else if(x < -M_PI){
        do{x += M_PI * 2;}while(x < -M_PI);
      }
      
      FloatT T_iono(5E-9);
      if(_abs(x) < 1.57){
        T_iono += amp * (1. - pow2(x) * (1.0 / 2 - pow2(x) / 24)); // ICD p.148
      }
      T_iono *= F;
      
      return -T_iono * self_t::light_speed; 
    }
    
    /**
     * Calculate correction value in accordance with ionospheric model
     *
     * @param sat satellite position (absolute position, XYZ)
     * @param usr user position (absolute position, XYZ)
     * @return correction in meters
     */
    FloatT iono_correction(
        const xyz_t &sat,
        const xyz_t &usr,
        const gps_time_t &t) const {
      return iono_correction(
          enu_t::relative(sat, usr), 
          usr.llh(), 
          t);
    }
    
    /**
     * Calculate correction value in accordance with tropospheric model
     * 
     * @param relative_pos satellite position (relative position, NEU)
     * @param usrllh user position (absolute position, LLH)
     * @return correction in meters
     */
    FloatT tropo_correction(
        const enu_t &relative_pos,
        const llh_t &usrllh) const {
      
      // Elevation (rad)
      FloatT el(relative_pos.elevation());
      
      // Altitude (m)
      FloatT h(const_cast<llh_t *>(&usrllh)->height());
      FloatT f(1.0);
      if(h > (1.0 / 2.3E-5)){
        f = 0;
      }else if(h > 0){
        f -= h * 2.3E-5;
      }
      
      return -2.47 * pow(f, 5) / (sin(el) + 0.0121);
    }
    
    /**
     * Calculate correction value in accordance with tropospheric model
     *
     * @param sat satellite position (absolute position, XYZ)
     * @param usr user position (absolute position, XYZ)
     * @return correction in meters
     */
    FloatT tropo_correction(
        const xyz_t &sat,
        const xyz_t &usr) const {
      return tropo_correction(
          enu_t::relative(sat, usr),
          usr.llh());
    }
};

template <class FloatT>
const FloatT GPS_SpaceNode<FloatT>::light_speed = 2.99792458E8;

template <class FloatT>
const FloatT GPS_SpaceNode<FloatT>::L1_Frequency = 1575.42E6;

template <class FloatT>
const FloatT GPS_SpaceNode<FloatT>::SC2RAD = 3.1415926535898;

#ifdef POW2_ALREADY_DEFINED
#undef POW2_ALREADY_DEFINED
#else
#undef pow2
#endif
#ifdef POW3_ALREADY_DEFINED
#undef POW3_ALREADY_DEFINED
#else
#undef pow3
#endif

#endif /* __GPS_H__ */
