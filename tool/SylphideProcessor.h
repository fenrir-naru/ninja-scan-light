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

#define PAGE_SIZE 32

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
};

template <class FloatType = double>
class A_Packet_Observer : public Packet_Observer<>{
  public:
    static const unsigned int a_packet_size = PAGE_SIZE - 1;
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
      char buf[4];
      this->inspect(buf, sizeof(buf), 1);
      return le_char4_2_num<unsigned int>(*buf);
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
        char buf[4];
        buf[0] = 0x00;
        for(int i = 0; i < 8; i++){
          this->inspect(&(buf[1]), 3, 5 + (3 * i));
          result.values[i] = be_char4_2_num<unsigned int>(*buf);
        }
      }
      
      {
        char buf[2];
        this->inspect(buf, 2, 29);
        result.temperature = le_char2_2_num<unsigned short>(*buf);
      }
      
      return result;
    }
};

template <class FloatType = double>
class F_Packet_Observer : public Packet_Observer<>{
  public:
    static const unsigned int f_packet_size = PAGE_SIZE - 1;
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
      char buf[4];
      this->inspect(buf, sizeof(buf), 3);
      return le_char4_2_num<unsigned int>(*buf);
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
        unsigned char buf[3];
        for(int i = 0; i < 8; i++){
          this->inspect((char *)&(buf[0]), 3, 7 + (3 * i));
          result.servo_out[i] = (buf[1] & 0x0F);
          result.servo_out[i] <<= 8;
          result.servo_out[i] |= buf[2];
          buf[1] >>= 4;
          result.servo_in[i] = buf[1];
          result.servo_in[i] <<= 8;
          result.servo_in[i] |= buf[0];
        }
      }
      
      return result;
    }
};

template <class FloatType = double>
class Data24Bytes_Packet_Observer : public Packet_Observer<>{
  public:
    static const unsigned int packet_size = PAGE_SIZE - 1;
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
      char buf[4];
      this->inspect(buf, sizeof(buf), 3);
      return le_char4_2_num<unsigned int>(*buf);
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
    
    struct values_t {
      unsigned short air_speed[4];
      unsigned short air_alpha[4];
      unsigned short air_beta[4];
    };
    values_t fetch_values() const {
      values_t result;
      
      {
        char buf[6];
        for(int i = 0; i < 4; i++){
          this->inspect(&(buf[0]), 6, 7 + (6 * i));
          result.air_speed[i] = be_char2_2_num<unsigned short>(buf[0]);
          result.air_alpha[i] = be_char2_2_num<unsigned short>(buf[2]);
          result.air_beta[i]  = be_char2_2_num<unsigned short>(buf[4]);
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
    
    struct values_t {
      short x[4];
      short y[4];
      short z[4];
    };
    values_t fetch_values() const {
      values_t result;
      
      unsigned char flags;
      this->inspect((char *)&flags, sizeof(flags), 0);
      if(flags & 0x80){
        // Big Endian mode (HMC5843,HMC5883)
        char buf[6];
        for(int i = 0; i < 4; i++){
          this->inspect(&(buf[0]), 6, 7 + (6 * i));
          result.x[i] = be_char2_2_num<short>(buf[0]);
          result.y[i] = be_char2_2_num<short>(buf[2]);
          result.z[i] = be_char2_2_num<short>(buf[4]);
        }
      }else{
        // Little Endian mode (HMR3300)
        char buf[6];
        for(int i = 0; i < 4; i++){
          this->inspect(&(buf[0]), 6, 7 + (6 * i));
          result.x[i] = le_char2_2_num<short>(buf[0]);
          result.y[i] = le_char2_2_num<short>(buf[2]);
          result.z[i] = le_char2_2_num<short>(buf[4]);
        }
      }
      
      return result;
    }
};

template <class FloatType = double>
class G_Packet_Observer : public Packet_Observer<>{
  public:
    unsigned int current_packet_size() const {
      char buf[2];
      this->inspect(buf, 2, 4);
      return min_macro(
          le_char2_2_num<unsigned short>(*buf) + 8,
          (this->capacity / 2)
        );
    }
  protected:
    mutable bool validate_skippable;
    bool valid_header() const {
      if(Packet_Observer<>::stored() < 2){
        return false;
      }
      return ((((unsigned char)((*this)[0])) == 0xB5) 
                && (((unsigned char)((*this)[1])) == 0x62)); 
    }
    bool valid_size() const {
      int _stored(Packet_Observer<>::stored());
      if(_stored < 6) return false;
      if(current_packet_size() > _stored) return false; 
      return true;
    }
    bool valid_parity() const {
      int ck_a(0), ck_b(0), packet_size(current_packet_size());
      for(int index(2); index < (packet_size - 2); index++){
        ck_a += (unsigned char)(*this)[index];
        ck_b += ck_a;
      }
      ck_a &= 0xFF;
      ck_b &= 0xFF;
      return ((((unsigned char)((*this)[packet_size - 2])) == ck_a) 
                && (((unsigned char)((*this)[packet_size - 1])) == ck_b));
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
        if((unsigned char)((*this)[0]) == 0xB5){
          if(_stored > 1){
            if((unsigned char)((*this)[1]) == 0x62){return true;}
            else{_stored--; Packet_Observer<>::skip(1);}
          }else{break;}
        }
        _stored--; Packet_Observer<>::skip(1);
      }
      return false;
    }
    
    struct packet_type_t {
        const char &mclass;
        const char &mid;
        bool equals(const char klass, const char id) const {
          return ((mclass == klass) && (mid == id)); 
        }
        packet_type_t(const char &klass, const char &id)
            : mclass(klass), mid(id) {}
        ~packet_type_t(){}
      };
    packet_type_t packet_type() const {
      return packet_type_t((*this)[2], (*this)[3]);
    }
    
    unsigned int fetch_ITOW_ms() const {
      char buf[4];
      this->inspect(buf, 4, 6);
      return le_char4_2_num<unsigned int>(*buf);
    }
    FloatType fetch_ITOW() const {
      return (FloatType)1E-3 * fetch_ITOW_ms();
    }
    unsigned short fetch_WN() const {
      char buf[2];
      this->inspect(buf, 2, 10);
      return le_char2_2_num<unsigned short>(*buf);
    }
    
    struct position_t {
      FloatType longitude, latitude, altitude;
      position_t(FloatType longi, FloatType lati, FloatType alti)
          : longitude(longi), latitude(lati), altitude(alti) {};
    };
    position_t fetch_position() const {
      //if(!packet_type().equals(0x01, 0x02)){}
      
      char buf1[4], buf2[4], buf3[4];
      this->inspect(buf1, 4, 6 + 4);
      this->inspect(buf2, 4, 6 + 8);
      this->inspect(buf3, 4, 6 + 12);
      
      return position_t(
          (FloatType)1E-7 * le_char4_2_num<int>(*buf1),
          (FloatType)1E-7 * le_char4_2_num<int>(*buf2),
          (FloatType)1E-3 * le_char4_2_num<int>(*buf3)
        );
    }
    
    struct position_acc_t {
      FloatType horizontal, vertical;
      position_acc_t(FloatType h_acc, FloatType v_acc)
          : horizontal(h_acc), vertical(v_acc) {};
    };
    position_acc_t fetch_position_acc() const {
      //if(!packet_type().equals(0x01, 0x02)){}
      
      char buf1[4], buf2[4];
      this->inspect(buf1, 4, 6 + 20);
      this->inspect(buf2, 4, 6 + 24);
      
      return position_acc_t(
          (FloatType)1E-3 * le_char4_2_num<unsigned int>(*buf1),
          (FloatType)1E-3 * le_char4_2_num<unsigned int>(*buf2)
        );
    }
    
    struct velocity_t {
      FloatType north, east, down;
      velocity_t(FloatType v_n, FloatType v_e, FloatType v_d)
          : north(v_n), east(v_e), down(v_d) {};
    };
    velocity_t fetch_velocity() const {
      //if(!packet_type().equals(0x01, 0x12)){}
      
      char buf1[4], buf2[4], buf3[4];
      this->inspect(buf1, 4, 6 + 4);
      this->inspect(buf2, 4, 6 + 8);
      this->inspect(buf3, 4, 6 + 12);
      
      return velocity_t(
          (FloatType)1E-2 * le_char4_2_num<int>(*buf1),
          (FloatType)1E-2 * le_char4_2_num<int>(*buf2),
          (FloatType)1E-2 * le_char4_2_num<int>(*buf3)
        );
    }
    
    struct velocity_acc_t {
      FloatType acc;
      velocity_acc_t(FloatType v_acc)
          : acc(v_acc) {};
    };
    velocity_acc_t fetch_velocity_acc() const {
      //if(!packet_type().equals(0x01, 0x12)){}
      
      char buf[4];
      this->inspect(buf, 4, 6 + 28);
      
      return velocity_acc_t(
          (FloatType)1E-2 * le_char4_2_num<int>(*buf)
        );
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
      char buf[4];
      status_t status;
      this->inspect(buf, sizeof(buf), 6 + 4);
      status.fix_type = (unsigned char)buf[0];
      status.status_flags = (unsigned char)buf[1];
      status.differential = (unsigned char)buf[2];
      this->inspect(buf, sizeof(buf), 6 + 8);
      status.time_to_first_fix_ms = le_char4_2_num<unsigned int>(*buf);
      this->inspect(buf, sizeof(buf), 6 + 12);
      status.time_to_reset_ms = le_char4_2_num<unsigned int>(*buf);
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
      svinfo_t(){}
      svinfo_t(
          unsigned int chn, unsigned int _svid, 
          unsigned int _flags, unsigned int qi,
          int cno, int elev, int azim, int prres) 
          : channel_num(chn), svid(_svid), 
            flags(_flags), quality_indicator(qi),
            signal_strength(cno), elevation(elev), 
            azimuth(azim), pseudo_residual(prres) {};
    };
    svinfo_t fetch_svinfo(unsigned int chn) const {
      //if(!packet_type().equals(0x01, 0x30)){}
      char buf[12];
      this->inspect(buf, sizeof(buf), 6 + 8 + (chn * sizeof(buf)));
      
      return svinfo_t(
          (unsigned char)(*buf),
          (unsigned char)(*(buf + 1)),
          (unsigned char)(*(buf + 2)),
          (unsigned char)(*(buf + 3)),
          (unsigned char)(*(buf + 4)),
          (*(buf + 5)),
          le_char2_2_num<short>(*(buf + 6)),
          le_char4_2_num<int>(*(buf + 8)));
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
      char buf[16];
      solution_t solution;
      this->inspect(buf, 4, 6 + 8);
      solution.week = le_char2_2_num<short>(*buf);
      solution.fix_type = (unsigned char)buf[2];
      solution.status_flags = (unsigned char)buf[3];
      this->inspect(buf, sizeof(buf), 6 + 12);
      solution.position_ecef_cm[0] = le_char4_2_num<int>(*buf);
      solution.position_ecef_cm[1] = le_char4_2_num<int>(*(buf + 4));
      solution.position_ecef_cm[2] = le_char4_2_num<int>(*(buf + 8));
      solution.position_ecef_acc_cm = le_char4_2_num<unsigned int>(*(buf + 12));
      this->inspect(buf, sizeof(buf), 6 + 28);
      solution.velocity_ecef_cm_s[0] = le_char4_2_num<int>(*buf);
      solution.velocity_ecef_cm_s[1] = le_char4_2_num<int>(*(buf + 4));
      solution.velocity_ecef_cm_s[2] = le_char4_2_num<int>(*(buf + 8));
      solution.velocity_ecef_acc_cm_s = le_char4_2_num<unsigned int>(*(buf + 12));
      this->inspect(buf, 1, 6 + 47);
      solution.satellites_used = (unsigned char)buf[0];
      return solution;
    }
    
    struct raw_measurement_t {
      FloatType carrier_phase, pseudo_range, doppler;
      unsigned int sv_number;
      int quarity, signal_strength;
      unsigned int lock_indicator;
      raw_measurement_t(){}
      raw_measurement_t(
          FloatType cp, FloatType pr, FloatType dopp, 
          unsigned int sv_num, 
          int q, int signal, 
          unsigned int lock
        ) :
          carrier_phase(cp), pseudo_range(pr), doppler(dopp), 
          sv_number(sv_num), quarity(q), signal_strength(signal), lock_indicator(lock) {};
    };
    raw_measurement_t fetch_raw(unsigned int index) const {
      //if(!packet_type().equals(0x02, 0x10)){}
      
      char buf[24];
      this->inspect(buf, 24, 6 + 8 + (index * 24));
      
      return raw_measurement_t(
          (FloatType)le_char8_2_num<double>(*buf),
          (FloatType)le_char8_2_num<double>(*(buf + 8)),
          (FloatType)le_char4_2_num<float>(*(buf + 16)),
          (unsigned char)(*(buf + 20)),
          (*(buf + 21)),
          (*(buf + 22)),
          (unsigned char)(*(buf + 23))
        );
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
    ephemeris_t fetch_ephemeris() const {
      //if(!packet_type().equals(0x02, 0x31)){}
      
      ephemeris_t ephemeris;
      {
        char buf[8];
        this->inspect(buf, sizeof(buf), 6); // SVID, HOW
        ephemeris.sv_number = le_char4_2_num<int>(buf[0]);
        ephemeris.how = le_char4_2_num<int>(buf[4]);
      }
      if((this->current_packet_size() > (8 + 8)) && ephemeris.how){
        ephemeris.valid = true;
        unsigned char buf[32];

#define get_subframe(n) (this->inspect((char *)buf, 32, 6 + 8 + n * 32))
#define bits2uchar(index) buf[(((index / 30) - 2) * 4) + (2 - ((index % 30) / 8))]
#define bits2char(index) (char)(bits2uchar(index))
#define bits2uchar_align(index, length) \
((bits2uchar(index) \
  & ((0xFF << (8 - length)) >> ((index % 30) % 8)) ) \
    >> (8 - (length + (index % 30) % 8)))
#define bytes2ushort(a, b) \
((((unsigned short)a) << 8) | b)
#define bytes2short(a, b) \
(short)(bytes2ushort(a, b))
#define bytes2uint(a, b, c, d) \
((((((((unsigned int)a) << 8) | b) << 8) | c) << 8) | d)
#define bytes2int(a, b, c, d) \
(int)(bytes2uint(a, b, c, d))
#define GPS_PI 3.1415926535898
#define DIV_POWER_2(n) (1.0 / (1 << ((n <= 30) ? (n) : 30)) / (1 << ((n <= 30) ? 0 : (n - 30))))

        get_subframe(0); // Subframe 1
        ephemeris.wn
          = (((unsigned short)bits2uchar(60)) << 2) | bits2uchar_align(68, 2);
        ephemeris.ura = bits2uchar_align(72, 4);
        ephemeris.sv_health = bits2uchar_align(76, 6);
        ephemeris.iodc 
          = (((unsigned short)bits2uchar_align(82, 2)) << 2) | bits2uchar(210);
        ephemeris.t_gd 
          = (FloatType)bits2char(196) 
            * DIV_POWER_2(31);
        ephemeris.t_oc = bytes2ushort(bits2uchar(218), bits2uchar(226)) << 4;
        ephemeris.a_f2 
          = (FloatType)bits2char(240) 
            * DIV_POWER_2(55);
        ephemeris.a_f1 
          = (FloatType)bytes2short(bits2uchar(248), bits2uchar(256)) 
            * DIV_POWER_2(43);
        ephemeris.a_f0
          = (FloatType)((((((int)bits2char(270)) << 8) | bits2uchar(278)) << 6)
            | (bits2uchar(286) >> 2))
            * DIV_POWER_2(31);
        
        get_subframe(1); // Subframe 2
        ephemeris.iode = bits2uchar(60);
        ephemeris.c_rs 
          = (FloatType)bytes2short(bits2uchar(68), bits2uchar(76)) 
            * DIV_POWER_2(5);
        ephemeris.delta_n 
          = (FloatType)bytes2short(bits2uchar(90), bits2uchar(98)) 
            * DIV_POWER_2(43) * GPS_PI;
        ephemeris.m_0 
          = (FloatType)bytes2int(bits2uchar(106), bits2uchar(120), bits2uchar(128), bits2uchar(136))
            * DIV_POWER_2(31) * GPS_PI;
        ephemeris.c_uc 
          = (FloatType)bytes2short(bits2uchar(150), bits2uchar(158))
            * DIV_POWER_2(29);
        ephemeris.e 
          = (FloatType)bytes2uint(bits2uchar(166), bits2uchar(180), bits2uchar(188), bits2uchar(196))
            * DIV_POWER_2(33);
        ephemeris.c_us 
          = (FloatType)bytes2short(bits2uchar(210), bits2uchar(218))
            * DIV_POWER_2(29);
        ephemeris.root_a
          = (FloatType)bytes2uint(bits2uchar(226), bits2uchar(240), bits2uchar(248), bits2uchar(256))
            * DIV_POWER_2(19);
        ephemeris.t_oe = bytes2ushort(bits2uchar(270), bits2uchar(278)) << 4;
        ephemeris.fit = (bits2uchar_align(286, 1) & 0x01);
        
        /*
        cout << hex;
        cout << setw(2) << setfill('0') << (unsigned int)bits2uchar(106) << ","
             << setw(2) << setfill('0') << (unsigned int)bits2uchar(120) << ","
             << setw(2) << setfill('0') << (unsigned int)bits2uchar(128) << ","
             << setw(2) << setfill('0') << (unsigned int)bits2uchar(136) << endl;
        cout << ephemeris.m_0 << endl;
        cout << dec;
        */
        
        get_subframe(2); // Subframe 3
        ephemeris.c_ic 
          = (FloatType)bytes2short(bits2uchar(60), bits2uchar(68))
            * DIV_POWER_2(29);
        ephemeris.omega_0
          = (FloatType)bytes2int(bits2uchar(76), bits2uchar(90), bits2uchar(98), bits2uchar(106))
            * DIV_POWER_2(31) * GPS_PI;
        ephemeris.c_is 
          = (FloatType)bytes2short(bits2uchar(120), bits2uchar(128))
            * DIV_POWER_2(29);
        ephemeris.i_0
          = (FloatType)bytes2int(bits2uchar(136), bits2uchar(150), bits2uchar(158), bits2uchar(166))
            * DIV_POWER_2(31) * GPS_PI;
        ephemeris.c_rc 
          = (FloatType)bytes2short(bits2uchar(180), bits2uchar(188))
            * DIV_POWER_2(5);
        ephemeris.omega
          = (FloatType)bytes2int(bits2uchar(196), bits2uchar(210), bits2uchar(218), bits2uchar(226))
            * DIV_POWER_2(31) * GPS_PI;
        ephemeris.omega_0_dot
          = (FloatType)bytes2int((bits2uchar(240) & 0x80 ? 0xFF : 0), bits2uchar(240), bits2uchar(248), bits2uchar(256))
            * DIV_POWER_2(43) * GPS_PI;
        ephemeris.i_0_dot
          = (FloatType)(short)((((unsigned short)bits2uchar(278)) << 6) | bits2uchar_align(286, 6)
              | (bits2uchar(278) & 0x80 ? 0xC000 : 0x0000))
            * DIV_POWER_2(43) * GPS_PI;
        
        /*
        cout << hex;
        cout << setw(2) << setfill('0') << (unsigned int)bits2uchar(278) << ","
             << setw(2) << setfill('0') << (unsigned int)bits2uchar(286) << endl;
        cout << dec;
        */

#undef DIV_POWER_2
#undef GPS_PI
#undef bytes2ushort
#undef bytes2short
#undef bytes2uint
#undef bytes2int
#undef bits2uchar_align
#undef bits2char
#undef bits2uchar
#undef get_subframe
      }
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
        char buf;
        this->inspect(&buf, 1, 74);
        health_utc_iono.health.valid = (buf & 0x01);
        health_utc_iono.utc.valid = (buf & 0x02);
        health_utc_iono.iono.valid = (buf & 0x04);
      }
      
      if(health_utc_iono.health.valid){ // Health
        char buf[4];
        this->inspect(buf, 4, 6);
        unsigned int mask(le_char4_2_num<unsigned int>(*(buf)));
        for(int i(0), j(1); i < 32; i++, j<<=1){
          health_utc_iono.health.healthy[i] = (mask & j);
        }
      }
      
      if(health_utc_iono.utc.valid){ // UTC
        char buf[32];
        this->inspect(buf, 32, 10);
        health_utc_iono.utc.a1    = (FloatType)le_char8_2_num<double>(*buf); 
        health_utc_iono.utc.a0    = (FloatType)le_char8_2_num<double>(*(buf + 8));
        health_utc_iono.utc.tot   = le_char4_2_num<int>(*(buf + 16));
        health_utc_iono.utc.wnt   = le_char2_2_num<short>(*(buf + 20));
        health_utc_iono.utc.ls    = le_char2_2_num<short>(*(buf + 22));
        health_utc_iono.utc.wnf   = le_char2_2_num<short>(*(buf + 24));
        health_utc_iono.utc.dn    = le_char2_2_num<short>(*(buf + 26));
        health_utc_iono.utc.lsf   = le_char2_2_num<short>(*(buf + 28));
        health_utc_iono.utc.spare = le_char2_2_num<short>(*(buf + 30));
      }
      
      if(health_utc_iono.iono.valid){ // iono
        char buf[32];
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

/**
 * Nパケット(航法情報)に関する定義
 * 
 * Header (4byte)
 *   'N' (1byte)
 *   シーケンスNo. (1byte)
 *   Nパケットの種類 (1byte)
 *   拡張(チェックサムにでも使う?) (1byte)
 * 
 * [Nパケットの種類 = 0x00] Content (28bytes)
 *   ITOW (4bytes) [msec]
 *   位置 (12bytes)
 *     緯度 (4bytes -2147483648 ～ 2147483647)
 *       -90.0000000 ～ 90.0000000 [deg] * 10000000
 *     経度 (4bytes -2147483648 ～ 2147483647)
 *       -180.0000000 ～ 180.0000000 [deg] * 10000000
 *     高度 (4bytes -2147483648 ～ 2147483647)
 *       -214748.3648 ～ 214748.3647 [m] * 10000
 *   速度 (6bytes)
 *     NED (2bytes -32768 ～ 32767)
 *       -327.68 ～ 327.68 [m/s] * 100
 *   姿勢 (6ytes)
 *     ヘディング (2bytes -32768 ～ 32767)
 *       -180.00 ～ 180.00 [deg] * 100
 *     ロール、ピッチ (2bytes -32768 ～ 32767)
 *       -90.00 ～ 90.00 [deg] * 100
 */
template <class FloatType = double>
class N_Packet_Observer : public Packet_Observer<>{
  public:
    static const unsigned int n_packet_size = PAGE_SIZE - 1;
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
      return PAGE_SIZE;
    }
    
    unsigned int sequence_num() const {return this->operator[](0);}
    unsigned int kind() const {return this->operator[](1);}
    
    unsigned int fetch_ITOW_ms() const {
      char buf[4];
      this->inspect(buf, sizeof(buf), 3);
      return le_char4_2_num<unsigned int>(*buf);
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
      
      char buf[24];
      this->inspect(buf, sizeof(buf), 7);
      
      // 位置
      result.latitude = (FloatType)1E-7 * le_char4_2_num<int>(*(buf));
      result.longitude = (FloatType)1E-7 * le_char4_2_num<int>(*(buf + 4));
      result.altitude = (FloatType)1E-4 * le_char4_2_num<int>(*(buf + 8));
      
      // 速度
      result.v_north = (FloatType)1E-2 * le_char2_2_num<short>(*(buf + 12));
      result.v_east = (FloatType)1E-2 * le_char2_2_num<short>(*(buf + 14));
      result.v_down = (FloatType)1E-2 * le_char2_2_num<short>(*(buf + 16));
      
      // 姿勢
      result.heading = (FloatType)1E-2 * le_char2_2_num<short>(*(buf + 18));
      result.pitch = (FloatType)1E-2 * le_char2_2_num<short>(*(buf + 20));
      result.roll = (FloatType)1E-2 * le_char2_2_num<short>(*(buf + 22));
      
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
    SylphideProcessor(const int &observer_buffer_size = PAGE_SIZE * 32)
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
