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

#if defined(_MSC_VER) && _MSC_VER >= 1400
#define _USE_MATH_DEFINES
//#define _CRT_SECURE_NO_WARNINGS // stop warning recommending secure functions such as sprintf_s
typedef __int64 int64_t;
#endif

#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <exception>

#define IS_LITTLE_ENDIAN 1
#include "SylphideStream.h"
#include "SylphideProcessor.h"

typedef double float_sylph_t;
#include "analyze_common.h"
#include "calibration.h"

using namespace std;

struct Options : public GlobalOptions<float_sylph_t> {
  typedef GlobalOptions<float_sylph_t> super_t;
  bool page_A;
  bool page_G;
  bool page_F;
  bool page_P;
  bool page_M;
  bool page_N;
  bool page_other;
  int page_P_mode, page_F_mode, page_M_mode;
  int debug_level;
  typedef CalendarTime<float_sylph_t> calendar_time_t;
  calendar_time_t::Converter time_gps2local;
  bool use_calendar_time;

  typedef StandardCalibration<float_sylph_t> inertial_conv_t;

  struct {
    bool is_active;
    inertial_conv_t inertial_conv;
  } physical_converter;

  Options() 
      : super_t(),
      page_A(false), page_G(false), page_F(false), 
      page_P(false), page_M(false), page_N(false),
      page_other(false),
      page_P_mode(5),
      page_F_mode(3),
      page_M_mode(0),
      debug_level(0),
      time_gps2local(),
      use_calendar_time(false) {

    physical_converter.is_active = false;
    super_t::set_typical_calibration_specs(physical_converter.inertial_conv);
  }
  ~Options(){}
  
  struct formatted_time_t {
    const Options &options;
    float_sylph_t itow;
    friend ostream &operator<<(ostream &out, const formatted_time_t &t){
      if(t.options.use_calendar_time){ // year, month, mday, hour, min, sec
        calendar_time_t t2(
            t.options.time_gps2local.convert(t.itow));
        out << t2.year << ", "
            << t2.month << ", "
            << t2.mday << ", "
            << t2.hour << ", "
            << t2.min << ", "
            << t2.sec;
      }else{
        out << t.itow;
      }
      return out;
    }
  };

  template <class T>
  formatted_time_t format_time(const T &itow){
    formatted_time_t res = {*this, itow};
    return res;
  }

  template <class T>
  bool is_time_in_range(const T &sec) const {
    return super_t::is_time_in_range(sec, time_gps2local.gps_time.wn);
  }

  /**
   * Check command argument
   * 
   * @param spec Command
   * @return (bool) If recognized, true; otherwise false
   */
  bool check_spec(const char *spec){
#define CHECK_OPTION(name, novalue, operation, disp) { \
  const char *value(get_value(spec, #name, novalue)); \
  if(value){ \
    {operation;} \
    std::cerr << #name << ": " << disp << std::endl; \
    return true; \
  } \
}
    CHECK_OPTION(page_P_mode, false,
        page_P_mode = atoi(value),
        page_P_mode);
    CHECK_OPTION(page_F_mode, false,
        page_F_mode = atoi(value),
        page_F_mode);
    CHECK_OPTION(page_M_mode, false,
        page_M_mode = atoi(value),
        page_M_mode);
    CHECK_OPTION(page_other, true,
        page_other = is_true(value),
        (page_other ? "on" : "off"));

    do{ // Change time outputs from gpstime[ms] to calendartime(YY,MM,DD,HH,MM,SS) formats
      const char *value(get_value(spec, "calendar_time"));
      if(!value){break;}
      int correction_hr(0);
      if(!is_true(value)){
        correction_hr = atoi(value); // Specify time zone by hour.
      }
      use_calendar_time = true;
      time_gps2local.correction_sec = 60 * 60 * correction_hr;
      cerr << "use_calendar_time: UTC "
          << (correction_hr >= 0 ? '+' : '-')
          << correction_hr << " [hr]" << endl;
      return true;
    }while(false);

    do{
      const char *value(GlobalOptions::get_value(spec, "page", false));
      if(!value){break;}
      switch(*value){
        case 'A': page_A = true; break;
        case 'G': page_G = true; break;
        case 'F': page_F = true; break;
        case 'M': page_M = true; break;
        case 'N': page_N = true; break;
        case 'P': page_P = true; break;
        default: return false;
      }
      return true;
    }while(false);

    CHECK_OPTION(debug, false,
        debug_level = atoi(value),
        debug_level);

    CHECK_OPTION(physical, true,
        physical_converter.is_active = is_true(value),
        (physical_converter.is_active ? "on" : "off"));
#undef CHECK_OPTION
    return super_t::check_spec(spec);
  }
} options;

class StreamProcessor : public SylphideProcessor<float_sylph_t> {
  protected:
    int invoked;
    typedef SylphideProcessor<float_sylph_t> super_t;

    template <class Observer>
    static float_sylph_t get_corrected_ITOW(const Observer &observer){
      float_sylph_t raw_itow(observer.fetch_ITOW());
      if(options.reduce_1pps_sync_error){
        static float_sylph_t previous_itow(0);
        float_sylph_t delta_t(raw_itow - previous_itow);
        if((delta_t >= 1) && (delta_t < 2)){
          raw_itow -= 1;
        }
        previous_itow = raw_itow;
      }
      return raw_itow;
    }
  
  public:
    /**
     * Check A page (AD converted values)
     * 
     * @param observer A page observer
     */
    struct HandlerA {
      int count;
      void (HandlerA::*formatter)(const float_sylph_t &current, const A_Observer_t::values_t &values) const;
      void operator()(const super_t::A_Observer_t &observer){
        if(!observer.validate()){return;} // check validity
        
        float_sylph_t current(StreamProcessor::get_corrected_ITOW(observer));
        if(!options.is_time_in_range(current)){return;}
        
        A_Observer_t::values_t values(observer.fetch_values());
        (this->*formatter)(current, values);
        count++;
      }
      void dump_raw(const float_sylph_t &current, const A_Observer_t::values_t &values) const {
        options.out() 
            << count << ", "
            << options.format_time(current) << ", ";
        
        for(int i(0); i < 8; i++){
          options.out() << values.values[i] << ", ";
        }
        options.out() << values.temperature << endl;
      }
      void dump_physical(const float_sylph_t &current, const A_Observer_t::values_t &values) const {
        options.out() << options.format_time(current);

        int ch[9];
        for(int i = 0; i < 8; i++){
          ch[i] = values.values[i];
        }
        ch[8] = values.temperature;

        Options::inertial_conv_t::result_t
            accel(options.physical_converter.inertial_conv.raw2accel(ch)),
            omega(options.physical_converter.inertial_conv.raw2omega(ch));

        for(int i(0); i < 3; i++){ // accelerometer[m/s^2]
          options.out() << ", " << accel.values[i];
        }
        for(int i(0); i < 3; i++){ // gyro[deg/sec]
          options.out() << ", " << rad2deg(omega.values[i]);
        }
        options.out() << endl;
      }
      HandlerA() : count(0), formatter(&HandlerA::dump_raw) {}
    } handler_A;
    
    /**
     * Check G page (u-blox GPS receiver output)
     * 
     * @param observer G page observer
     */
    struct HandlerG {
      unsigned int itow_ms_0x0102, itow_ms_0x0112;
      super_t::G_Observer_t::position_t position;
      super_t::G_Observer_t::position_acc_t position_acc;
      super_t::G_Observer_t::velocity_t velocity;
      super_t::G_Observer_t::velocity_acc_t velocity_acc;
      HandlerG() 
          : itow_ms_0x0102(0), itow_ms_0x0112(0),
          position(0, 0, 0), position_acc(0, 0),
          velocity(0, 0, 0), velocity_acc(0) {
      }
      ~HandlerG(){}
      
      void operator()(const super_t::G_Observer_t &observer){
        if(!observer.validate()){return;}
        
        bool change_itow(false);
        super_t::G_Observer_t::packet_type_t
            packet_type(observer.packet_type());
        switch(packet_type.mclass){
          case 0x01: {
            switch(packet_type.mid){
              case 0x02: { // NAV-POSLLH
                position = observer.fetch_position();
                position_acc = observer.fetch_position_acc();
                
                itow_ms_0x0102 = observer.fetch_ITOW_ms();
                change_itow = true;
                
                break;
              }
              case 0x12: { // NAV-VELNED
                velocity = observer.fetch_velocity();
                velocity_acc = observer.fetch_velocity_acc();
                
                itow_ms_0x0112 = observer.fetch_ITOW_ms();
                change_itow = true;
                
                break;
              }
              case 0x20: { // NAV-TIMEGPS
                float_sylph_t itow(observer.fetch_ITOW());
                char buf[4];
                observer.inspect(buf, sizeof(buf), 6 + 8);
                if((unsigned char)buf[3] & 0x02){ // valid week number
                  int wn(le_char2_2_num<unsigned short>(*buf));

                  if((unsigned char)buf[3] & 0x04){ // valid UTC (leap seconds)
                    options.time_gps2local.update(itow, wn, (char)(buf[2]));
                  }else{
                    options.time_gps2local.update(itow, wn);
                  }
                }else{
                  options.time_gps2local.update(itow);
                }
                break;
              }
            }
            break;
          }
          default:
            break;
        }
        
        if(!options.page_G){return;}

        if(change_itow
            && (itow_ms_0x0102 == itow_ms_0x0112)){
          
          float_sylph_t current(1E-3 * itow_ms_0x0102);
          if(!options.is_time_in_range(current)){return;}
          
          options.out() << options.format_time(current) << ", "
              << position.latitude << ", "
              << position.longitude << ", "
              << position.altitude << ", "
              << position_acc.horizontal << ", "
              << position_acc.vertical << ", "
              << velocity.north << ", "
              << velocity.east << ", "
              << velocity.down << ", "
              << velocity_acc.acc << endl;
        }
      }
    } handler_G;
    
    /**
     * check F page (FPGA output), which is not used for NinjaScan
     * 
     * @param observer F page observer
     */
    struct HandlerF {
      int count;
      HandlerF() : count(0) {}
      void operator()(const F_Observer_t &observer){
        if(!observer.validate()){return;}
        
        float_sylph_t current(StreamProcessor::get_corrected_ITOW(observer));
        if(!options.is_time_in_range(current)){return;}
        
        options.out() << (count++)
             << ", " << options.format_time(current);
        
        F_Observer_t::values_t values(observer.fetch_values());
        for(int i = 0; i < 8; i++){
          //if(values.servo_in[i] < 1000){values.servo_in[i] += 1000;}
          if(options.page_F_mode & 0x01){ // bit 0 for input
            options.out() << ", " << values.servo_in[i];
          }
          if(options.page_F_mode & 0x02){ // bit 1 for output
            options.out() << ", " << values.servo_out[i];
          }
        }
        options.out() << endl;
      }
    } handler_F;
    
    
    struct HandlerP {
      void (HandlerP::*formatter)(
          const float_sylph_t &current, const int &index,
          const Int32 &pressure, const Int32 &temperature) const;
      void ms5611_convert(
          const Int32 &d1, const Int32 &d2,
          Int32 &pressure, Int32 &temperature,
          const Uint16 (&coef)[6]) const {
        Int32 dT(d2 - (coef[4] << 8));
        temperature = (Int32)2000 + (((int64_t)dT * coef[5]) >> 23);

        int64_t off(((int64_t)coef[1] << 16) + (((int64_t)coef[3] * dT) >> 7));
        int64_t sens(((int64_t)coef[0] << 15) + (((int64_t)coef[2] * dT) >> 8));

        // Figure 3
        if(temperature < 2000){
          Int32 t2(((int64_t)dT * dT) << 31);
          Int32 dT2(temperature - 2000), dT2_2(dT2 * dT2);
          Int32 off2((dT2_2 * 5) >> 1);
          Int32 sens2((dT2_2 * 5) >> 2);
          if(temperature < -1500){
            Int32 dT3(temperature + 1500), dT3_2(dT3 * dT3);
            off2 += dT3_2 * 7;
            sens2 += (dT3_2 * 11) >> 1;
          }
          temperature -= t2;
          off -= off2;
          sens -= sens2;
        }

        pressure = (Int32)(((((int64_t)d1 * sens) >> 21) - off) >> 15);
      }

      /**
       * check P page (pressure sensor)
       * 
       * @param observer F page observer
       */
      void operator()(const P_Observer_t &observer){
        if(!observer.validate()){return;}
        
        float_sylph_t current(StreamProcessor::get_corrected_ITOW(observer));
        if(!options.is_time_in_range(current)){return;}
        
        switch(options.page_P_mode){
          case 5: { // MS5611 with coefficients
            Uint16 coef[6];
            char buf[sizeof(Uint16)];
            for(int i(0); i < sizeof(coef) / sizeof(coef[0]); i++){
              observer.inspect(&buf[0], sizeof(buf), 19 + (sizeof(buf) * i));
              coef[i] = be_char2_2_num<Uint16>(*buf);
            }

            for(int i(0), j(-1); i < 2; i++, j++){
              char buf[2][4];
              buf[0][0] = buf[1][0] = 0;
              observer.inspect(&buf[0][1], 3, 7 + 6 * i);
              observer.inspect(&buf[1][1], 3, 10 + 6 * i);
              Uint32
                  d1(be_char4_2_num<Uint32>(buf[0][0])),
                  d2(be_char4_2_num<Uint32>(buf[1][0]));
              Int32 pressure, temperature;
              ms5611_convert(d1, d2, pressure, temperature, coef);
              (this->*formatter)(current, j, pressure, temperature);
            }
            break;
          }
        }
      }
      void dump_raw(
          const float_sylph_t &current, const int &index,
          const Int32 &pressure, const Int32 &temperature) const {
        options.out()
            << options.format_time(current) << ", " << index << ", "
            << pressure << ", " << temperature << endl;
      }
      void dump_physical(
          const float_sylph_t &current, const int &index,
          const Int32 &pressure, const Int32 &temperature) const {
        options.out()
            << options.format_time(current) << ", " << index << ", "
            << (float_sylph_t)pressure << ", "  // [Pa]
            << (float_sylph_t)temperature / 100 << endl; // [degC]
      }
      HandlerP() : formatter(&HandlerP::dump_raw) {}
    } handler_P;
    
    /**
     * check M page (magnetic sensor)
     * 
     * @param observer M page observer
     */
    struct HandlerM {
      void (HandlerM::*formatter)(const float_sylph_t &current, const M_Observer_t::values_t &values) const;
      void operator()(const M_Observer_t &observer){
        if(!observer.validate()){return;}
        
        float_sylph_t current(StreamProcessor::get_corrected_ITOW(observer));
        if(!options.is_time_in_range(current)){return;}
        
        M_Observer_t::values_t values(observer.fetch_values());

        switch(options.page_M_mode){
          case 1: // -atan2(y, x)‚µ‚½Œ‹‰Ê[deg]‚ð•\Ž¦
            for(int i(0), j(-3); i < 4; i++, j++){
              options.out() << options.format_time(current) << ", "
                   << j << ", "
                   << rad2deg(-atan2((double)values.y[i], (double)values.x[i])) << endl;
            }
            break;
          default:
            (this->*formatter)(current, values);
        }
      }
      void dump_raw(const float_sylph_t &current, const M_Observer_t::values_t &values) const {
        for(int i(0), j(-3); i < 4; i++, j++){
          options.out() << options.format_time(current) << ", "
               << j << ", "
               << values.x[i] << ", "
               << values.y[i] << ", "
               << values.z[i] << endl;
        }
      }
      void dump_physical(const float_sylph_t &current, const M_Observer_t::values_t &values) const {
        /* TODO how to resolve the difference of sensors;
         *
         * [ver1] MAG3110 => scale factor is 0.10 uT / LSB
         * [ver2] AK8963 in MPU9250 => scale factor is 0.15 uT / LSB (16-bit mode @see mpu9250.c)
         */
        dump_raw(current, values);
      }
      HandlerM() : formatter(&HandlerM::dump_raw) {}
    } handler_M;
    /**
    
     * check N page (navigation information), which is not used for NinjaScan
     * 
     * @param observer M page observer
     */
    struct HandlerN {
      void operator()(const N_Observer_t &observer){
        if(!observer.validate()){return;}
        
        float_sylph_t current(StreamProcessor::get_corrected_ITOW(observer));
        if(!options.is_time_in_range(current)){return;}
        
        switch(observer.kind()){
          case 0: {
            N_Observer_t::navdata_t values(observer.fetch_navdata());
            
            options.out() << options.format_time(values.itow) << ", "
                << values.longitude << ", "
                << values.latitude << ", "
                << values.altitude << ", "
                << values.v_north << ", "
                << values.v_east << ", "
                << values.v_down << ", "
                << values.heading << ", "
                << values.pitch << ", "
                << values.roll << endl;
            break;
          }
          default:
            return;
        }
      }
    } handler_N;
    
#if 0
    /**
     * checker for C page in SylphideProtocol format
     * 
     */
    class SylphideStreamHandler 
        : public Sylphide_Packet_Observer<float_sylph_t> {
      protected:
        typedef Sylphide_Packet_Observer<float_sylph_t> super_t;
        typedef SylphideStreamHandler self_t;
      public:
        bool previous_seek;
        unsigned int invoked;
        SylphideStreamHandler() 
            : super_t(SYLPHIDE_PAGE_SIZE),
            previous_seek(false), invoked(0) {}
        ~SylphideStreamHandler() {}
        
        /**
         * callback function when packet is found
         * 
         */
        void operator()(const self_t &observer){
          if(!observer.validate()){return;}
          
        }
    } handler_C;
#endif
    
  public:
    StreamProcessor()
        : super_t(), invoked(0) {
      
    }
    ~StreamProcessor(){}
    
    /**
     * Extract packet from stream until the end of stream is found
     * 
     * @param in stream
     */
    void process(istream &in){
      char buffer[SYLPHIDE_PAGE_SIZE];
      
      if(options.physical_converter.is_active){
        handler_A.formatter = &HandlerA::dump_physical;
        handler_P.formatter = &HandlerP::dump_physical;
        handler_M.formatter = &HandlerM::dump_physical;
        cerr << "Units are [m/s^2], [deg/s], [Pa], and [degC] "
            "for acceleration, angular speed, pressure, and temperature, respectively."
            << endl;
      }

        int read_count;
      while(true){
        in.read(buffer, SYLPHIDE_PAGE_SIZE);
        read_count = in.gcount();
        if(in.fail() || (read_count == 0)){return;}
        invoked++;
      
        if(options.debug_level){
          cerr << "--read-- : " << invoked << " page" << endl;
          cerr << hex;
          for(int i(0); i < read_count; i++){
            cerr << setfill('0') 
                << setw(2)
                << (unsigned int)((unsigned char)buffer[i]) << ' ';
          }
          cerr << dec;
          cerr << endl;
          
          if(read_count < SYLPHIDE_PAGE_SIZE){
            cerr << "--skipped-- : " << invoked << " page ; count = " << read_count << endl;
          }
        }
      
        switch(buffer[0]){
#define assign_case_cnd(type, mark, cnd) \
case mark: if(cnd){ \
  super_t::process_packet( \
      buffer, read_count, \
      observer_ ## type , previous_seek_next_ ## type, handler_ ## type); \
} \
break;
#define assign_case(type, mark) assign_case_cnd(type, mark, options.page_ ## type)
          assign_case(A, 'A');
          assign_case_cnd(G, 'G', true);
          assign_case(F, 'F');
          assign_case(P, 'P');
          assign_case(M, 'M');
          assign_case(N, 'N');
#undef assign_case
#if 0
          case 'C': if(options.out_C){
            super_t::process_packet(
                buffer, read_count,
                handler_C, handler_C.previous_seek, handler_C);
          }
          break;
#endif
          default: if(options.page_other){
            if(buffer[0] == 'T'){
              stringstream ss;
              ss << hex;
              for(int i(0); i < read_count; i++){
                ss << setfill('0') 
                    << setw(2)
                    << (unsigned int)((unsigned char)buffer[i]) << ' ';
              }
              ss << endl;
              options.out() << ss.str();
            }
          }
          break;
        }
      }
    }
};

int main(int argc, char *argv[]){

  cerr << "NinjaScan converter to make CSV format data." << endl;
  cerr << "Usage: " << argv[0] << " [options, ex) --page=A] log.dat" << endl;
  if(argc < 2){
    cerr << "Error: too few arguments; " << argc << " < min(2)" << endl;
    return -1;
  }
  
  // For backward compatibility
  if(strstr(argv[0], "log_AD_CSV") == argv[0]){
    options.page_A = true;
  }else if(strstr(argv[0], "log_F_CSV") == argv[0]){
    options.page_F = true;
  }else if(strstr(argv[0], "log_M_CSV") == argv[0]){
    options.page_M = true;
  }else if(strstr(argv[0], "log_P_CSV") == argv[0]){
    options.page_P = true;
  }
  
  StreamProcessor processor;
  
  int log_index(1); // log file name is assumed to be given by argv[1]
  
  // check options
  for(int i(1); i < argc; i++){
    if(options.check_spec(argv[i])){continue;}
    // if arg is not an option, assume arg as log file name
    if(log_index != 1){ // Detect unknown option by multiple substitution to log_index.
      cerr << "Unknown option!! : " << argv[i] << endl;
      return -1;
    }
    log_index = i;
  }
  
  options.out().precision(10);
  if(options.in_sylphide){
    SylphideIStream sylph_in(options.spec2istream(argv[log_index]), SYLPHIDE_PAGE_SIZE);
    processor.process(sylph_in);
  }else{
    processor.process(options.spec2istream(argv[log_index]));
  }
  
  return 0;
}
