/*
 * Copyright (c) 2013, M.Naruoka (fenrir)
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

#ifndef __SYLPHIDE_PROCESSOR_H__
#define __SYLPHIDE_PROCESSOR_H__

#ifdef __TI_COMPILER_VERSION__
  #include <string.h>
  #ifdef _BIG_ENDIAN
    #define IS_LITTLE_ENDIAN 0
  #else
    #define IS_LITTLE_ENDIAN 1
  #endif
#else
  #include <string>
  #include <exception>
#endif

#include "util/fifo.h"
#ifndef IS_LITTLE_ENDIAN
  #define IS_LITTLE_ENDIAN 1
#endif
#include "util/endian.h"

#include "std.h"

#define SYLPHIDE_PAGE_SIZE 32

template <class Container = char>
class Packet_Observer : public FIFO<Container>{
  public:
    Packet_Observer(const unsigned int &buffer_size)
      : FIFO<Container>(buffer_size){
        
    }
    virtual ~Packet_Observer(){}
    
    virtual bool ready() const = 0;
    virtual bool validate() const = 0;
    virtual bool seek_next() = 0;
    virtual unsigned int current_packet_size() const = 0;

    typedef Container v8_t;
    typedef Uint8 u8_t;
    typedef Uint16 u16_t;
    typedef Uint32 u32_t;
    typedef Int8 s8_t;
    typedef Int16 s16_t;
    typedef Int32 s32_t;
};

template <class FloatType = double>
class A_Packet_Observer : public Packet_Observer<>{
  public:
    static const unsigned int a_packet_size = SYLPHIDE_PAGE_SIZE - 1;
    A_Packet_Observer(const unsigned int &buffer_size) 
        : Packet_Observer<>(buffer_size){
      
    }
    ~A_Packet_Observer(){}
    bool ready() const {
      return (Packet_Observer<>::stored() >= a_packet_size);
    }
    bool validate() const {
      return true;
    }
    bool seek_next(){
      if(Packet_Observer<>::stored() < a_packet_size){return false;}
      Packet_Observer<>::skip(a_packet_size);
      return true;
    }
    unsigned int fetch_ITOW_ms() const {
      v8_t buf[4];
      this->inspect(buf, sizeof(buf), 1);
      return le_char4_2_num<u32_t>(*buf);
    }
    FloatType fetch_ITOW() const {
      return (FloatType)1E-3 * fetch_ITOW_ms();
    }
    unsigned int current_packet_size() const {
      return a_packet_size;
    }
    
    struct values_t {
      unsigned int values[8];
      unsigned short temperature;
    };
    values_t fetch_values() const {
      values_t result;
      
      {
        v8_t buf[4];
        buf[0] = 0x00;
        for(int i = 0; i < 8; i++){
          this->inspect(&(buf[1]), 3, 5 + (3 * i));
          result.values[i] = be_char4_2_num<u32_t>(*buf);
        }
      }
      
      {
        v8_t buf[2];
        this->inspect(buf, 2, 29);
        result.temperature = le_char2_2_num<u16_t>(*buf);
      }
      
      return result;
    }
};

template <class FloatType = double>
class F_Packet_Observer : public Packet_Observer<>{
  public:
    static const unsigned int f_packet_size = SYLPHIDE_PAGE_SIZE - 1;
    F_Packet_Observer(const unsigned int &buffer_size) 
        : Packet_Observer<>(buffer_size){
      
    }
    ~F_Packet_Observer(){}
    bool ready() const {
      return (Packet_Observer<>::stored() >= f_packet_size);
    }
    bool validate() const {
      return true;
    }
    bool seek_next(){
      if(Packet_Observer<>::stored() < f_packet_size){return false;}
      Packet_Observer<>::skip(f_packet_size);
      return true;
    }
    unsigned int fetch_ITOW_ms() const {
      v8_t buf[4];
      this->inspect(buf, sizeof(buf), 3);
      return le_char4_2_num<u32_t>(*buf);
    }
    FloatType fetch_ITOW() const {
      return (FloatType)1E-3 * fetch_ITOW_ms();
    }
    unsigned int current_packet_size() const {
      return f_packet_size;
    }
    
    struct values_t {
      unsigned int servo_in[8];
      unsigned int servo_out[8];
    };
    values_t fetch_values() const {
      values_t result;
      
      {
        v8_t buf[3];
        for(int i = 0; i < 8; i++){
          this->inspect(buf, 3, 7 + (3 * i));
          result.servo_out[i] = ((u8_t)buf[1] & 0x0F);
          result.servo_out[i] <<= 8;
          result.servo_out[i] |= (u8_t)buf[2];
          buf[1] >>= 4;
          result.servo_in[i] = (u8_t)buf[1];
          result.servo_in[i] <<= 8;
          result.servo_in[i] |= (u8_t)buf[0];
        }
      }
      
      return result;
    }
};

template <class FloatType = double>
class Data24Bytes_Packet_Observer : public Packet_Observer<>{
  public:
    static const unsigned int packet_size = SYLPHIDE_PAGE_SIZE - 1;
    Data24Bytes_Packet_Observer(const unsigned int &buffer_size) 
        : Packet_Observer<>(buffer_size){
      
    }
    ~Data24Bytes_Packet_Observer(){}
    bool ready() const {
      return (Packet_Observer<>::stored() >= packet_size);
    }
    bool validate() const {
      return true;
    }
    bool seek_next(){
      if(Packet_Observer<>::stored() < packet_size){return false;}
      Packet_Observer<>::skip(packet_size);
      return true;
    }
    unsigned int fetch_ITOW_ms() const {
      v8_t buf[4];
      this->inspect(buf, sizeof(buf), 3);
      return le_char4_2_num<u32_t>(*buf);
    }
    FloatType fetch_ITOW() const {
      return (FloatType)1E-3 * fetch_ITOW_ms();
    }
    unsigned int current_packet_size() const {
      return packet_size;
    }
};

template <class FloatType = double>
class P_Packet_Observer : public Data24Bytes_Packet_Observer<FloatType>{
  public:
    P_Packet_Observer(const unsigned int &buffer_size) 
        : Data24Bytes_Packet_Observer<FloatType>(buffer_size){
      
    }
    ~P_Packet_Observer(){}
    
    typedef Data24Bytes_Packet_Observer<FloatType> super_t;
    typedef typename super_t::v8_t v8_t;
    typedef typename super_t::u16_t u16_t;

    struct values_t {
      unsigned short air_speed[4];
      unsigned short air_alpha[4];
      unsigned short air_beta[4];
    };
    values_t fetch_values() const {
      values_t result;
      
      {
        v8_t buf[6];
        for(int i = 0; i < 4; i++){
          this->inspect(buf, 6, 7 + (6 * i));
          result.air_speed[i] = be_char2_2_num<u16_t>(buf[0]);
          result.air_alpha[i] = be_char2_2_num<u16_t>(buf[2]);
          result.air_beta[i]  = be_char2_2_num<u16_t>(buf[4]);
        }
      }
      
      return result;
    }
};

template <class FloatType = double>
class M_Packet_Observer : public Data24Bytes_Packet_Observer<FloatType>{
  public:
    M_Packet_Observer(const unsigned int &buffer_size) 
        : Data24Bytes_Packet_Observer<FloatType>(buffer_size){
      
    }
    ~M_Packet_Observer(){}
    
    typedef Data24Bytes_Packet_Observer<FloatType> super_t;
    typedef typename super_t::v8_t v8_t;
    typedef typename super_t::u8_t u8_t;
    typedef typename super_t::s16_t s16_t;

    struct values_t {
      short x[4];
      short y[4];
      short z[4];
    };
    values_t fetch_values() const {
      values_t result;
      
      v8_t flags;
      this->inspect(&flags, sizeof(flags), 0);
      if((u8_t)flags & 0x80){
        // Big Endian mode (HMC5843)
        v8_t buf[6];
        for(int i = 0; i < 4; i++){
          this->inspect(buf, 6, 7 + (6 * i));
          result.x[i] = be_char2_2_num<s16_t>(buf[0]);
          result.y[i] = be_char2_2_num<s16_t>(buf[2]);
          result.z[i] = be_char2_2_num<s16_t>(buf[4]);
        }
      }else{
        // Little Endian mode (HMR3300)
        v8_t buf[6];
        for(int i = 0; i < 4; i++){
          this->inspect(buf, 6, 7 + (6 * i));
          result.x[i] = le_char2_2_num<s16_t>(buf[0]);
          result.y[i] = le_char2_2_num<s16_t>(buf[2]);
          result.z[i] = le_char2_2_num<s16_t>(buf[4]);
        }
      }
      
      return result;
    }
};

template <class FloatType = double>
class G_Packet_Observer : public Packet_Observer<>{
  public:
    unsigned int current_packet_size() const {
      v8_t buf[2];
      this->inspect(buf, 2, 4);
      return min_macro(
          le_char2_2_num<u16_t>(*buf) + 8,
          (this->capacity / 2)
        );
    }
  protected:
    mutable bool validate_skippable;
    bool valid_header() const {
      if(Packet_Observer<>::stored() < 2){
        return false;
      }
      return ((((u8_t)((*this)[0])) == 0xB5)
                && (((u8_t)((*this)[1])) == 0x62));
    }
    bool valid_size() const {
      int _stored(Packet_Observer<>::stored());
      if(_stored < 6) return false;
      if(current_packet_size() > _stored) return false; 
      return true;
    }
    bool valid_parity() const {
      u8_t ck_a(0), ck_b(0);
      unsigned int packet_size(current_packet_size());
      for(unsigned int index(2); index < (packet_size - 2); index++){
        ck_a += (u8_t)(*this)[index];
        ck_b += ck_a;
      }
      return ((((u8_t)((*this)[packet_size - 2])) == ck_a)
                && (((u8_t)((*this)[packet_size - 1])) == ck_b));
    }
  public:
    G_Packet_Observer(const unsigned int &buffer_size) 
        : Packet_Observer<>(buffer_size),
        validate_skippable(false){
      
    }
    ~G_Packet_Observer(){}
    bool ready() const {
      if(!valid_header()) return false;
      if(!valid_size()) return false;
      return true;
    }
    bool validate() const {
      if(validate_skippable){
        return true;
      }
      return (validate_skippable = valid_parity());
    }
    bool seek_next(){
      if(ready()){
        Packet_Observer<>::skip(
            validate() ? current_packet_size() : 1);
      }
      int _stored(Packet_Observer<>::stored());
      validate_skippable = false;
      while(_stored > 0){
        if((u8_t)((*this)[0]) == 0xB5){
          if(_stored > 1){
            if((u8_t)((*this)[1]) == 0x62){return true;}
            else{_stored--; Packet_Observer<>::skip(1);}
          }else{break;}
        }
        _stored--; Packet_Observer<>::skip(1);
      }
      return false;
    }
    
    struct packet_type_t {
      unsigned char mclass, mid;
      bool equals(const unsigned char &klass, const unsigned char &id) const {
        return ((klass == mclass) && (id == mid));
      }
      packet_type_t(const unsigned char &klass, const unsigned char &id)
          : mclass(klass), mid(id) {}
      ~packet_type_t(){}
    };
    packet_type_t packet_type() const {
      return packet_type_t((unsigned char)((*this)[2]), (unsigned char)((*this)[3]));
    }
    
    unsigned int fetch_ITOW_ms() const {
      v8_t buf[4];
      this->inspect(buf, 4, 6);
      return le_char4_2_num<u32_t>(*buf);
    }
    FloatType fetch_ITOW() const {
      return (FloatType)1E-3 * fetch_ITOW_ms();
    }
    unsigned short fetch_WN() const {
      v8_t buf[2];
      this->inspect(buf, 2, 10);
      return le_char2_2_num<s16_t>(*buf);
    }
    
    struct position_t {
      FloatType longitude, latitude, altitude;
      position_t(){}
      position_t(const FloatType &lng, const FloatType &lat, const FloatType &alt)
          : longitude(lng), latitude(lat), altitude(alt) {};
    };
    position_t fetch_position() const {
      //if(!packet_type().equals(0x01, 0x02)){}
      
      v8_t buf[4];
      position_t pos;
      this->inspect(buf, 4, 6 + 4);
      pos.longitude = (FloatType)1E-7 * le_char4_2_num<s32_t>(*buf);
      this->inspect(buf, 4, 6 + 8);
      pos.latitude = (FloatType)1E-7 * le_char4_2_num<s32_t>(*buf);
      this->inspect(buf, 4, 6 + 12);
      pos.altitude = (FloatType)1E-3 * le_char4_2_num<s32_t>(*buf);
      
      return pos;
    }
    
    struct position_acc_t {
      FloatType horizontal, vertical;
      position_acc_t(){}
      position_acc_t(const FloatType &h_acc, const FloatType &v_acc)
          : horizontal(h_acc), vertical(v_acc) {};
    };
    position_acc_t fetch_position_acc() const {
      //if(!packet_type().equals(0x01, 0x02)){}
      
      v8_t buf[4];
      position_acc_t pos_acc;
      this->inspect(buf, 4, 6 + 20);
      pos_acc.horizontal = (FloatType)1E-3 * le_char4_2_num<u32_t>(*buf);
      this->inspect(buf, 4, 6 + 24);
      pos_acc.vertical = (FloatType)1E-3 * le_char4_2_num<u32_t>(*buf);
      
      return pos_acc;
    }
    
    struct velocity_t {
      FloatType north, east, down;
      velocity_t(){}
      velocity_t(const FloatType &v_n, const FloatType &v_e, const FloatType &v_d)
          : north(v_n), east(v_e), down(v_d) {};
    };
    velocity_t fetch_velocity() const {
      //if(!packet_type().equals(0x01, 0x12)){}
      
      v8_t buf[4];
      velocity_t vel;
      this->inspect(buf, 4, 6 + 4);
      vel.north = (FloatType)1E-2 * le_char4_2_num<s32_t>(*buf);
      this->inspect(buf, 4, 6 + 8);
      vel.east = (FloatType)1E-2 * le_char4_2_num<s32_t>(*buf);
      this->inspect(buf, 4, 6 + 12);
      vel.down = (FloatType)1E-2 * le_char4_2_num<s32_t>(*buf);
      
      return vel;
    }
    
    struct velocity_acc_t {
      FloatType acc;
      velocity_acc_t(){}
      velocity_acc_t(const FloatType &v_acc) : acc(v_acc) {};
    };
    velocity_acc_t fetch_velocity_acc() const {
      //if(!packet_type().equals(0x01, 0x12)){}
      
      v8_t buf[4];
      velocity_acc_t vel_acc;
      this->inspect(buf, 4, 6 + 28);
      vel_acc.acc = (FloatType)1E-2 * le_char4_2_num<s32_t>(*buf);
      
      return vel_acc;
    }
    
    struct status_t {
      unsigned int fix_type;
      unsigned int status_flags;
      unsigned int differential;
      unsigned int time_to_first_fix_ms;
      unsigned int time_to_reset_ms;
      enum {
          NO_FIX = 0,
          DEAD_RECKONING_ONLY,
          FIX_2D,
          FIX_3D,
          GPS_WITH_DR,
          TIME_ONLY_FIX};
      enum {
          FIX_OK = 0x01,
          DGPS_USED = 0x02,
          WN_VALID = 0x04,
          TOW_VALID = 0x08};
    };
    status_t fetch_status() const {
      //if(!packet_type().equals(0x01, 0x03)){}
      v8_t buf[4];
      status_t status;
      this->inspect(buf, sizeof(buf), 6 + 4);
      status.fix_type = (u8_t)buf[0];
      status.status_flags = (u8_t)buf[1];
      status.differential = (u8_t)buf[2];
      this->inspect(buf, sizeof(buf), 6 + 8);
      status.time_to_first_fix_ms = le_char4_2_num<u32_t>(*buf);
      this->inspect(buf, sizeof(buf), 6 + 12);
      status.time_to_reset_ms = le_char4_2_num<u32_t>(*buf);
      return status;
    }
    
    struct svinfo_t {
      unsigned int channel_num;
      unsigned int svid;
      unsigned int flags;
      unsigned int quality_indicator;
      int signal_strength;
      int elevation;
      int azimuth;
      int pseudo_residual;
    };
    svinfo_t fetch_svinfo(unsigned int chn) const {
      //if(!packet_type().equals(0x01, 0x30)){}
      v8_t buf[12];
      svinfo_t info;
      this->inspect(buf, sizeof(buf), 6 + 8 + (chn * sizeof(buf)));
      info.channel_num        = (u8_t)(*buf);
      info.svid               = (u8_t)(*(buf + 1));
      info.flags              = (u8_t)(*(buf + 2));
      info.quality_indicator  = (u8_t)(*(buf + 3));
      info.signal_strength    = (u8_t)(*(buf + 4));
      info.elevation          = (*(buf + 5));
      info.azimuth            = le_char2_2_num<s16_t>(*(buf + 6));
      info.pseudo_residual    = le_char4_2_num<s32_t>(*(buf + 8));
      return info;
    }
    
    struct solution_t {
      short week;
      unsigned int fix_type;
      unsigned int status_flags;
      int position_ecef_cm[3];
      unsigned int position_ecef_acc_cm;
      int velocity_ecef_cm_s[3];
      unsigned int velocity_ecef_acc_cm_s;
      unsigned int satellites_used;
      enum {
          NO_FIX = 0,
          DEAD_RECKONING_ONLY,
          FIX_2D,
          FIX_3D,
          GPS_WITH_DR,
          TIME_ONLY_FIX};
      enum {
          FIX_OK = 0x01,
          DGPS_USED = 0x02,
          WN_VALID = 0x04,
          TOW_VALID = 0x08};
    };
    solution_t fetch_solution() const {
      //if(!packet_type().equals(0x01, 0x06)){}
      v8_t buf[16];
      solution_t solution;
      this->inspect(buf, 4, 6 + 8);
      solution.week = le_char2_2_num<s16_t>(*buf);
      solution.fix_type = (u8_t)buf[2];
      solution.status_flags = (u8_t)buf[3];
      this->inspect(buf, sizeof(buf), 6 + 12);
      solution.position_ecef_cm[0] = le_char4_2_num<s32_t>(*buf);
      solution.position_ecef_cm[1] = le_char4_2_num<s32_t>(*(buf + 4));
      solution.position_ecef_cm[2] = le_char4_2_num<s32_t>(*(buf + 8));
      solution.position_ecef_acc_cm = le_char4_2_num<u32_t>(*(buf + 12));
      this->inspect(buf, sizeof(buf), 6 + 28);
      solution.velocity_ecef_cm_s[0] = le_char4_2_num<s32_t>(*buf);
      solution.velocity_ecef_cm_s[1] = le_char4_2_num<s32_t>(*(buf + 4));
      solution.velocity_ecef_cm_s[2] = le_char4_2_num<s32_t>(*(buf + 8));
      solution.velocity_ecef_acc_cm_s = le_char4_2_num<u32_t>(*(buf + 12));
      this->inspect(buf, 1, 6 + 47);
      solution.satellites_used = (u8_t)buf[0];
      return solution;
    }
    
    struct utc_t {
      unsigned short year;
      unsigned char month;
      unsigned char day_of_month;
      unsigned char hour_of_day;
      unsigned char minute_of_hour;
      unsigned char seconds_of_minute;
      bool valid;
    };
    utc_t fetch_utc() const {
      //if(!packet_type().equals(0x01, 0x21)){}
      v8_t buf[8];
      utc_t utc;
      this->inspect(buf, sizeof(buf), 6 + 12);
      utc.year = le_char2_2_num<u16_t>(*buf);
      utc.month = (u8_t)buf[2];
      utc.day_of_month = (u8_t)buf[3];
      utc.hour_of_day = (u8_t)buf[4];
      utc.minute_of_hour = (u8_t)buf[5];
      utc.seconds_of_minute = (u8_t)buf[6];
      utc.valid = ((u8_t)buf[7]) & 0x04;
      return utc;
    }

    struct raw_measurement_t {
      FloatType carrier_phase, pseudo_range, doppler;
      unsigned int sv_number;
      int quarity, signal_strength;
      unsigned int lock_indicator;
    };
    raw_measurement_t fetch_raw(unsigned int index) const {
      //if(!packet_type().equals(0x02, 0x10)){}
      
      v8_t buf[24];
      raw_measurement_t raw;
      this->inspect(buf, 24, 6 + 8 + (index * 24));
      raw.carrier_phase   = le_char8_2_num<double>(*buf);
      raw.pseudo_range    = le_char8_2_num<double>(*(buf + 8));
      raw.doppler         = le_char4_2_num<float>(*(buf + 16));
      raw.sv_number       = (u8_t)(*(buf + 20));
      raw.quarity         = (*(buf + 21));
      raw.signal_strength = (*(buf + 22));
      raw.lock_indicator  = (u8_t)(*(buf + 23));
      return raw;
    }
    
    struct subframe_t {
      unsigned int sv_number;
      unsigned int subframe_no, sv_or_page_id;
      v8_t buffer[40];

      u8_t bits2u8_align(
          const unsigned int &index,
          const unsigned int &length) const {
        return ((u8_t)(buffer[((index / 30) * 4) + (2 - ((index % 30) / 8))])
            & ((0xFF & (0xFF << (8 - length))) >> ((index % 30) % 8)))
              >> (8 - (length + (index % 30) % 8));
      }
      subframe_t() : subframe_no(0), sv_or_page_id(0) {}
      
#define bits2u8(index) (u8_t)(buffer[((index / 30) * 4) + (2 - ((index % 30) / 8))])
#define bits2s8(index) (s8_t)(bits2u8(index))
#define bits2u8_align(index, length) \
((bits2u8(index) \
  & ((0xFF & (0xFF << (8 - length))) >> ((index % 30) % 8)) ) \
    >> (8 - (length + (index % 30) % 8)))
#define bytes2u16(a, b) \
((((u16_t)a) << 8) | b)
#define bytes2s16(a, b) \
(s16_t)(bytes2u16(a, b))
#define bytes2u32(a, b, c, d) \
((((((((u32_t)a) << 8) | b) << 8) | c) << 8) | d)
#define bytes2s32(a, b, c, d) \
(s32_t)(bytes2u32(a, b, c, d))
#define GPS_PI 3.1415926535898
#define DIV_POWER_2(n) (1.0 / (1 << ((n <= 30) ? (n) : 30)) / (1 << ((n <= 30) ? 0 : (n - 30))))
      u16_t ephemeris_wn() const {
        return (((u16_t)bits2u8(60)) << 2) | bits2u8_align(68, 2);
      }
      u8_t ephemeris_ura() const {
        return bits2u8_align(72, 4);
      }
      u8_t ephemeris_sv_health() const {
        return bits2u8_align(76, 6);
      }
      u16_t ephemeris_iodc() const {
        return (((u16_t)bits2u8_align(82, 2)) << 2) | bits2u8(210);
      }
      FloatType ephemeris_t_gd() const {
        return (FloatType)bits2s8(196) * DIV_POWER_2(31);
      }
      u16_t ephemeris_t_oc() const {
        return bytes2u16(bits2u8(218), bits2u8(226)) << 4;
      }
      FloatType ephemeris_a_f2() const {
        return (FloatType)bits2s8(240) * DIV_POWER_2(55);
      }
      FloatType ephemeris_a_f1() const {
        return (FloatType)bytes2s16(bits2u8(248), bits2u8(256)) * DIV_POWER_2(43);
      }
      FloatType ephemeris_a_f0() const {
        return (FloatType)((((((s32_t)bits2s8(270)) << 8) | bits2u8(278)) << 6)
            | (bits2u8(286) >> 2))
            * DIV_POWER_2(31);
      }

      u8_t ephemeris_iode_subframe2() const {
        return bits2u8(60);
      }
      FloatType ephemeris_c_rs() const {
        return (FloatType)bytes2s16(bits2u8(68), bits2u8(76))
            * DIV_POWER_2(5);
      }
      FloatType ephemeris_delta_n() const {
        return (FloatType)bytes2s16(bits2u8(90), bits2u8(98))
            * DIV_POWER_2(43) * GPS_PI;
      }
      FloatType ephemeris_m_0() const {
        return (FloatType)bytes2s32(bits2u8(106), bits2u8(120), bits2u8(128), bits2u8(136))
            * DIV_POWER_2(31) * GPS_PI;
      }
      FloatType ephemeris_c_uc() const {
        return (FloatType)bytes2s16(bits2u8(150), bits2u8(158))
            * DIV_POWER_2(29);
      }
      FloatType ephemeris_e() const {
        return (FloatType)bytes2u32(bits2u8(166), bits2u8(180), bits2u8(188), bits2u8(196))
            * DIV_POWER_2(33);
      }
      FloatType ephemeris_c_us() const {
        return (FloatType)bytes2s16(bits2u8(210), bits2u8(218))
            * DIV_POWER_2(29);
      }
      FloatType ephemeris_root_a() const {
        return (FloatType)bytes2u32(bits2u8(226), bits2u8(240), bits2u8(248), bits2u8(256))
            * DIV_POWER_2(19);
      }
      u16_t ephemeris_t_oe() const {
        return bytes2u16(bits2u8(270), bits2u8(278)) << 4;
      }
      bool ephemeris_fit() const {
        return (bits2u8_align(286, 1) & 0x01) == 0x01;
      }

      FloatType ephemeris_c_ic() const {
        return (FloatType)bytes2s16(bits2u8(60), bits2u8(68))
            * DIV_POWER_2(29);
      }
      FloatType ephemeris_omega_0() const {
        return (FloatType)bytes2s32(bits2u8(76), bits2u8(90), bits2u8(98), bits2u8(106))
            * DIV_POWER_2(31) * GPS_PI;
      }
      FloatType ephemeris_c_is() const {
        return (FloatType)bytes2s16(bits2u8(120), bits2u8(128))
            * DIV_POWER_2(29);
      }
      FloatType ephemeris_i_0() const {
        return (FloatType)bytes2s32(bits2u8(136), bits2u8(150), bits2u8(158), bits2u8(166))
            * DIV_POWER_2(31) * GPS_PI;
      }
      FloatType ephemeris_c_rc() const {
        return (FloatType)bytes2s16(bits2u8(180), bits2u8(188))
            * DIV_POWER_2(5);
      }
      FloatType ephemeris_omega() const {
        return (FloatType)bytes2s32(bits2u8(196), bits2u8(210), bits2u8(218), bits2u8(226))
            * DIV_POWER_2(31) * GPS_PI;
      }
      FloatType ephemeris_omega_0_dot() const {
        return (FloatType)bytes2s32((bits2u8(240) & 0x80 ? 0xFF : 0), bits2u8(240), bits2u8(248), bits2u8(256))
            * DIV_POWER_2(43) * GPS_PI;
      }
      u8_t ephemeris_iode_subframe3() const {
        return bits2u8(270);
      }
      FloatType ephemeris_i_0_dot() const {
        return (FloatType)(s16_t)((((u16_t)bits2u8(278)) << 6) | bits2u8_align(286, 6)
              | (bits2u8(278) & 0x80 ? 0xC000 : 0x0000))
            * DIV_POWER_2(43) * GPS_PI;
      }
#undef DIV_POWER_2
#undef GPS_PI
#undef bytes2u16
#undef bytes2s16
#undef bytes2u32
#undef bytes2s32
#undef bits2u8_align
#undef bits2s8
#undef bits2uchar
      template <class EpemerisT>
      void fetch_as_subframe1(EpemerisT &ephemeris) const {
        ephemeris.wn = ephemeris_wn();
        ephemeris.ura = ephemeris_ura();
        ephemeris.sv_health = ephemeris_sv_health();
        ephemeris.iodc = ephemeris_iodc();
        ephemeris.t_gd = ephemeris_t_gd();
        ephemeris.t_oc = ephemeris_t_oc();
        ephemeris.a_f2 = ephemeris_a_f2();
        ephemeris.a_f1 = ephemeris_a_f1();
        ephemeris.a_f0 = ephemeris_a_f0();
      }
      template <class EpemerisT>
      void fetch_as_subframe2(EpemerisT &ephemeris) const {
        ephemeris.iode = ephemeris_iode_subframe2();
        ephemeris.c_rs = ephemeris_c_rs();
        ephemeris.delta_n = ephemeris_delta_n();
        ephemeris.m_0 = ephemeris_m_0();
        ephemeris.c_uc = ephemeris_c_uc();
        ephemeris.e = ephemeris_e();
        ephemeris.c_us = ephemeris_c_us();
        ephemeris.root_a = ephemeris_root_a();
        ephemeris.t_oe = ephemeris_t_oe();
        ephemeris.fit = ephemeris_fit();
      }
      template <class EpemerisT>
      void fetch_as_subframe3(EpemerisT &ephemeris) const {
        ephemeris.c_ic = ephemeris_c_ic();
        ephemeris.omega_0 = ephemeris_omega_0();
        ephemeris.c_is = ephemeris_c_is();
        ephemeris.i_0 = ephemeris_i_0();
        ephemeris.c_rc = ephemeris_c_rc();
        ephemeris.omega = ephemeris_omega();
        ephemeris.omega_0_dot = ephemeris_omega_0_dot();
        ephemeris.i_0_dot = ephemeris_i_0_dot();
      }
    };
    subframe_t fetch_subframe() const {
      //if(!packet_type().equals(0x02, 0x11)){}

      subframe_t subframe;
      {
        v8_t buf;
        this->inspect(&buf, sizeof(buf), 6 + 1); // SVID
        subframe.sv_number = buf;
      }

      this->inspect(subframe.buffer, sizeof(subframe.buffer), 6 + 2); // buffer
      if(subframe.sv_number <= 32){
        subframe.subframe_no = subframe.bits2u8_align(49, 3);
        switch(subframe.subframe_no){
          case 4:
          case 5:
            subframe.sv_or_page_id = subframe.bits2u8_align(62, 6);
            // subframe.4 page.2-5,7-10 correspond to SV 25-32
            // subframe.5 page.1-24 correspond to SV 1-24
            break;
        }
      }

      return subframe;
    }

    struct ephemeris_t {
      unsigned int sv_number, how;
      bool valid;

      // Subframe 1
      unsigned int wn, ura, sv_health, iodc, t_oc;
      FloatType t_gd,  a_f2, a_f1, a_f0;

      // Subframe 2
      unsigned int iode, t_oe;
      FloatType c_rs, delta_n, m_0, c_uc, e, c_us, root_a;
      bool fit;

      // Subframe 3
      FloatType c_ic, omega_0, c_is, i_0, c_rc, omega, omega_0_dot, i_0_dot;

      ephemeris_t() : valid(false) {}
    };
    template <class EphemerisT>
    void fetch_ephemeris(EphemerisT &ephemeris) const {
      //if(!packet_type().equals(0x02, 0x31)){}

      {
        v8_t buf[8];
        this->inspect(buf, sizeof(buf), 6); // SVID, HOW
        ephemeris.sv_number = le_char4_2_num<s32_t>(buf[0]);
        ephemeris.how = le_char4_2_num<s32_t>(buf[4]);
      }
      if((this->current_packet_size() > (8 + 8)) && ephemeris.how){
        ephemeris.valid = true;
        subframe_t subframe;

#define get_subframe(n) (this->inspect(&(subframe.buffer[8]), 32, 6 + 8 + n * 32))
        get_subframe(0); // Subframe 1
        subframe.fetch_as_subframe1(ephemeris);

        get_subframe(1); // Subframe 2
        subframe.fetch_as_subframe2(ephemeris);

        get_subframe(2); // Subframe 3
        subframe.fetch_as_subframe3(ephemeris);
#undef get_subframe
      }
    }
    ephemeris_t fetch_ephemeris() const {
      ephemeris_t ephemeris;
      fetch_ephemeris(ephemeris);
      return ephemeris;
    }
    struct health_utc_iono_t {
      struct {
        bool healthy[32];
        bool valid;
      } health;
      struct {
        FloatType a1, a0;
        unsigned int tot, wnt, ls, wnf, dn, lsf, spare;
        bool valid;  
      } utc;
      struct {
        FloatType klob_a0, klob_a1, klob_a2, klob_a3, klob_b0, klob_b1, klob_b2, klob_b3;
        bool valid;
      } iono;
      health_utc_iono_t(){
        health.valid = false;
        utc.valid = false;
        iono.valid = false;
      }
    };
    health_utc_iono_t fetch_health_utc_iono() const {
      //if(!packet_type().equals(0x0b, 0x02)){}
      health_utc_iono_t health_utc_iono;
      
      { // Valid flag
        v8_t buf;
        this->inspect(&buf, 1, 6 + 68);
        health_utc_iono.health.valid = ((u8_t)buf & 0x01);
        health_utc_iono.utc.valid = ((u8_t)buf & 0x02);
        health_utc_iono.iono.valid = ((u8_t)buf & 0x04);
      }
      
      if(health_utc_iono.health.valid){ // Health
        v8_t buf[4];
        this->inspect(buf, 4, 6);
        u32_t mask(le_char4_2_num<u32_t>(*(buf)));
        for(int i(0), j(1); i < 32; i++, j<<=1){
          health_utc_iono.health.healthy[i] = (mask & j);
        }
      }
      
      if(health_utc_iono.utc.valid){ // UTC
        v8_t buf[32];
        this->inspect(buf, 32, 10);
        health_utc_iono.utc.a1    = (FloatType)le_char8_2_num<double>(*buf); 
        health_utc_iono.utc.a0    = (FloatType)le_char8_2_num<double>(*(buf + 8));
        health_utc_iono.utc.tot   = le_char4_2_num<s32_t>(*(buf + 16));
        health_utc_iono.utc.wnt   = le_char2_2_num<s16_t>(*(buf + 20));
        health_utc_iono.utc.ls    = le_char2_2_num<s16_t>(*(buf + 22));
        health_utc_iono.utc.wnf   = le_char2_2_num<s16_t>(*(buf + 24));
        health_utc_iono.utc.dn    = le_char2_2_num<s16_t>(*(buf + 26));
        health_utc_iono.utc.lsf   = le_char2_2_num<s16_t>(*(buf + 28));
        health_utc_iono.utc.spare = le_char2_2_num<s16_t>(*(buf + 30));
      }
      
      if(health_utc_iono.iono.valid){ // iono
        v8_t buf[32];
        this->inspect(buf, 32, 42);
        health_utc_iono.iono.klob_a0 = (FloatType)le_char4_2_num<float>(*buf);
        health_utc_iono.iono.klob_a1 = (FloatType)le_char4_2_num<float>(*(buf + 4));
        health_utc_iono.iono.klob_a2 = (FloatType)le_char4_2_num<float>(*(buf + 8));
        health_utc_iono.iono.klob_a3 = (FloatType)le_char4_2_num<float>(*(buf + 12));
        health_utc_iono.iono.klob_b0 = (FloatType)le_char4_2_num<float>(*(buf + 16));
        health_utc_iono.iono.klob_b1 = (FloatType)le_char4_2_num<float>(*(buf + 20));
        health_utc_iono.iono.klob_b2 = (FloatType)le_char4_2_num<float>(*(buf + 24));
        health_utc_iono.iono.klob_b3 = (FloatType)le_char4_2_num<float>(*(buf + 28));
      }
      
      return health_utc_iono;
    }
};

template <class FloatType = double>
class N_Packet_Observer : public Packet_Observer<>{
  public:
    static const unsigned int n_packet_size = SYLPHIDE_PAGE_SIZE - 1;
    N_Packet_Observer(const unsigned int &buffer_size) 
        : Packet_Observer<>(buffer_size){
      
    }
    ~N_Packet_Observer(){}
    bool ready() const {
      return (Packet_Observer<>::stored() >= n_packet_size);
    }
    bool validate() const {
      return true;
    }
    bool seek_next(){
      if(Packet_Observer<>::stored() < n_packet_size){return false;}
      Packet_Observer<>::skip(n_packet_size);
      return true;
    }
    unsigned int current_packet_size() const {
      return SYLPHIDE_PAGE_SIZE;
    }
    
    unsigned int sequence_num() const {return this->operator[](0);}
    unsigned int kind() const {return this->operator[](1);}
    
    unsigned int fetch_ITOW_ms() const {
      v8_t buf[4];
      this->inspect(buf, sizeof(buf), 3);
      return le_char4_2_num<u32_t>(*buf);
    }
    FloatType fetch_ITOW() const {
      return (FloatType)1E-3 * fetch_ITOW_ms();
    }
    
    struct navdata_t {
      FloatType itow; // [s]
      FloatType latitude, longitude, altitude; // [deg], [deg], [m]
      FloatType v_north, v_east, v_down;  // [m/s]
      FloatType heading, pitch, roll; // [deg]
    };
    navdata_t fetch_navdata() const {
      //if(!kind() == 0x00){}
      navdata_t result;
      
      // ITOW
      result.itow = fetch_ITOW();
      
      v8_t buf[24];
      this->inspect(buf, sizeof(buf), 7);
      
      // à íu
      result.latitude = (FloatType)1E-7 * le_char4_2_num<s32_t>(*(buf));
      result.longitude = (FloatType)1E-7 * le_char4_2_num<s32_t>(*(buf + 4));
      result.altitude = (FloatType)1E-4 * le_char4_2_num<s32_t>(*(buf + 8));
      
      // ë¨ìx
      result.v_north = (FloatType)1E-2 * le_char2_2_num<s16_t>(*(buf + 12));
      result.v_east = (FloatType)1E-2 * le_char2_2_num<s16_t>(*(buf + 14));
      result.v_down = (FloatType)1E-2 * le_char2_2_num<s16_t>(*(buf + 16));
      
      // épê®
      result.heading = (FloatType)1E-2 * le_char2_2_num<s16_t>(*(buf + 18));
      result.pitch = (FloatType)1E-2 * le_char2_2_num<s16_t>(*(buf + 20));
      result.roll = (FloatType)1E-2 * le_char2_2_num<s16_t>(*(buf + 22));
      
      return result;
    }
};

#if defined(__SYLPHIDE_STREAM_H__)
template <class FloatType = double>
class Sylphide_Packet_Observer : public Packet_Observer<>{
  protected:
    mutable bool validate_skippable;
  public:
    unsigned int current_packet_size() const {
      return SylphideProtocol::Decorder::packet_size(*this);
    }
    unsigned int current_payload_size() const {
      return SylphideProtocol::Decorder::payload_size(*this);
    }
    unsigned int current_sequence_num() const {
      return SylphideProtocol::Decorder::sequence_num(*this);
    }
    Sylphide_Packet_Observer(const unsigned int &buffer_size) 
        : Packet_Observer<>(buffer_size),
        validate_skippable(false){
      
    }
    ~Sylphide_Packet_Observer(){}
    bool ready() const {
      return SylphideProtocol::Decorder::ready(*this);
    }
    bool validate() const {
      if(validate_skippable){
        return true;
      }
      return (validate_skippable 
          = SylphideProtocol::Decorder::validate(*this));
    }
    bool seek_next(){
      if(ready()){
        Packet_Observer<>::skip(
            validate() ? current_packet_size() : 1); 
      }
      validate_skippable = false;
      int _stored(Packet_Observer<>::stored());
      while(_stored >= SylphideProtocol::capsule_size){
        if(SylphideProtocol::Decorder::valid_head(*this)){
          return true;
        }
        _stored--;
        Packet_Observer<>::skip(1);
      }
      return false;
    }
};
#endif

template <class FloatType = double>
class AbstractSylphideProcessor{
  protected:
    template <class Observer, typename Callback>
    void process_raw(
        char *buffer, int read_count,
        Observer &observer, 
        bool &previous_seek_next,
        Callback &handler){
      observer.write(buffer, read_count);
      if(!previous_seek_next){
        if(observer.ready()){handler(observer);}
        previous_seek_next = observer.seek_next();
      }
      while(previous_seek_next && observer.ready()){
        handler(observer);
        previous_seek_next = observer.seek_next();
      }
    }
    template <class Observer, typename Callback>
    void process_packet(
        char *buffer, int read_count,
        Observer &observer,
        bool &previous_seek_next,
        Callback &handler){
      process_raw(buffer + 1, read_count - 1, observer, previous_seek_next, handler);
    }
};

template <class FloatType = double>
class SylphideProcessor : public AbstractSylphideProcessor<FloatType> {

#define assign_observer(type) \
public: \
  typedef type ## _Packet_Observer<FloatType> type ## _Observer_t; \
protected: \
  type ## _Observer_t observer_ ## type; \
  void (*packet_handler_ ## type)(const type ## _Observer_t &); \
  bool previous_seek_next_ ## type

    assign_observer(A);
    assign_observer(G);
    assign_observer(F);
    assign_observer(P);
    assign_observer(M);
    assign_observer(N);

#undef assign_observer
  
  protected:
    int process_count;
    typedef AbstractSylphideProcessor<FloatType> super_t;
    
  public:
#define assign_initializer(type) \
observer_ ## type(observer_buffer_size), \
packet_handler_ ## type(NULL), \
previous_seek_next_ ## type(observer_ ## type.ready())
    SylphideProcessor(const int &observer_buffer_size = SYLPHIDE_PAGE_SIZE * 32)
      : assign_initializer(A),
        assign_initializer(G),
        assign_initializer(F),
        assign_initializer(P),
        assign_initializer(M),
        assign_initializer(N),
        process_count(0) {
      
    }
#undef assign_initializer
    virtual ~SylphideProcessor(){}

#define assign_setter(type, mark) \
void set_ ## mark ## _handler(void (*handler)(const type ## _Observer_t &)){ \
  packet_handler_ ## type = handler; \
}
    assign_setter(A, a);
    assign_setter(G, g);
    assign_setter(F, f);
    assign_setter(P, p);
    assign_setter(M, m);
    assign_setter(N, n);
#undef assign_setter
  
  public:
    virtual void process(char *buffer, int read_count){
      switch(buffer[0]){
#define assign_case(type, header) \
case header : { \
  if(packet_handler_ ## type){ \
    super_t::process_packet( \
        buffer, read_count, \
        observer_ ## type , previous_seek_next_ ## type, packet_handler_ ## type); \
  } \
  break; \
}
        assign_case(A, 'A');
        assign_case(G, 'G');
        assign_case(F, 'F');
        assign_case(P, 'P');
        assign_case(M, 'M');
        assign_case(N, 'N');
#undef assign_case
      }
    }
};

#endif /* __SYLPHIDE_PROCESSOR_H__ */
