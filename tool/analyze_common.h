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

#ifndef __ANALYZE_COMMON_H__
#define __ANALYZE_COMMON_H__

#include <iostream>
#include <fstream>
#include <map>

#include <cstdio>
#include <cstring>
#include <cmath>
#include <cstdlib>
#include <cfloat>
#if defined(_MSC_VER)
#include <io.h>
#include <fcntl.h>
#endif

#include "util/comstream.h"
#include "util/endian.h"

/**
 * 度からラジアンに変換
 *
 * @param degree 度
 * @return (double) ラジアン
 */
double deg2rad(const double &degree){return degree * M_PI / 180;}
/**
 * ラジアンから度に変換
 *
 * @param radians ラジアン
 * @return (double) 度
 */
double rad2deg(const double &radians){return radians * 180 / M_PI;}

struct GlobalOptions {
  
  bool dump_update; ///< 時間更新(Time Update)の際に現在の値を表示するか
  bool dump_correct; ///< 観測更新(Measurement Update)の際に現在の値を表示するか
  double init_yaw_deg;   ///< 初期ヨー角
  double start_gpstime;  ///< 計測開始時刻
  int start_gpswn; ///< 計測開始週番号
  double end_gpstime;    ///< 計測終了時刻
  int end_gpswn; ///< 計測終了週番号
  bool est_bias; ///< バイアス推定を行うか
  bool use_udkf; ///< UDKFを使うか
  bool use_magnet; ///< 磁気コンパスを使うか
  double mag_heading_accuracy_deg; ///< 磁気コンパスの精度[deg]
  double yaw_correct_with_mag_when_speed_less_than_ms; ///< 方位補正を速度[m/s]以下の場合行う、0以下で非適用
  bool out_is_N_packet; ///< 出力をNPacket形式で出す
  std::ostream *_out; ///< 出力先、out()で参照をとるべき
  bool in_sylphide;   ///< trueのとき、入力がSylphideプロトコル
  bool out_sylphide;  ///< trueのとき、出力がSylphideプロトコル
  typedef std::map<const char *, std::iostream *> iostream_pool_t;
  iostream_pool_t iostream_pool;
  
  GlobalOptions()
      : dump_update(true),
      dump_correct(false),
      init_yaw_deg(0),
      start_gpstime(0), end_gpstime(DBL_MAX),
      start_gpswn(0), end_gpswn(0),
      est_bias(true),
      use_udkf(false),
      use_magnet(false),
      mag_heading_accuracy_deg(3),
      yaw_correct_with_mag_when_speed_less_than_ms(5),
      out_is_N_packet(false),
      _out(&(std::cout)),
      in_sylphide(false), out_sylphide(false),
      iostream_pool() {};
  virtual ~GlobalOptions(){
    for(iostream_pool_t::iterator it(iostream_pool.begin());
        it != iostream_pool.end();
        ++it){
      it->second->flush();
      delete it->second;
    }
  }
  
  template <class T>
  bool is_time_in_range(const T &time){
    return (time >= start_gpstime) && (time <= end_gpstime);
  }
  
  void set_baudrate(ComportStream &com, const char *baudrate_spec){
    int baudrate(atoi(baudrate_spec));
    if(baudrate != com.buffer().set_baudrate(baudrate)){
      std::cerr << " => Unsupported baudrate!!" << std::endl;
      exit(-1);
    }
  }

#ifdef _WIN32
#define COMPORT_PREFIX "COM"
#else
#define COMPORT_PREFIX "/dev/tty"
#endif
  
  std::istream &spec2istream(
      const char *spec, 
      const bool force_fstream = false){
    if(!force_fstream){
      if(strcmp(spec, "-") == 0){
        // 標準入力は'-'で指定
        std::cerr << "[std::cin]" << std::endl;
#if defined(_MSC_VER)
      setmode(fileno(stdin), O_BINARY);
#endif
        return std::cin;
      }else if(strstr(spec, COMPORT_PREFIX) == spec){
        // COMポート
        // 先に登録されていないか確認する
        // (COMポート名[:速度])で指定、速度を分離する
        char *baudrate_spec((char *)strchr(spec, ':'));
        if(baudrate_spec){
          *baudrate_spec = '\0';
          baudrate_spec++;
        }
        if(iostream_pool.find(spec) == iostream_pool.end()){
          ComportStream *com_in = new ComportStream(spec);
          if(baudrate_spec){set_baudrate(*com_in, baudrate_spec);}
          iostream_pool[spec] = com_in;
          return *com_in;
        }else{
          return *(iostream_pool[spec]);
        }
      }
    }
    
    std::cerr << spec;
    std::fstream *fin(new std::fstream(spec, std::ios::in | std::ios::binary));    
    if(fin->fail()){
      std::cerr << " => File not found!!" << std::endl;
      exit(-1);
    }
    std::cerr << std::endl;
    iostream_pool[spec] = fin;
    return *fin;
  }
  
  std::ostream &spec2ostream(
      const char *spec,
      const bool force_fstream = false){
    if(!force_fstream){
      if(strcmp(spec, "-") == 0){
        // 標準出力は'-'で指定
        std::cerr << "[std::cout]" << std::endl;
#if defined(_MSC_VER)
      setmode(fileno(stdout), O_BINARY);
#endif
        return std::cout;
      }else if(strstr(spec, COMPORT_PREFIX) == spec){
        // COMポート
        // 先に登録されていないか確認する
        // (COMポート名[:速度])で指定、速度を分離する
        char *baudrate_spec((char *)strchr(spec, ':'));
        if(baudrate_spec){
          *baudrate_spec = '\0';
          baudrate_spec++;
        }
        if(iostream_pool.find(spec) == iostream_pool.end()){
          ComportStream *com_out = new ComportStream(spec);
          if(baudrate_spec){set_baudrate(*com_out, baudrate_spec);}
          iostream_pool[spec] = com_out;
          return *com_out;
        }else{
          return *(iostream_pool[spec]);
        }
      }
    }
    
    std::cerr << spec;
    std::fstream *fout(new std::fstream(spec, std::ios::out | std::ios::binary));
    std::cerr << std::endl;
    iostream_pool[spec] = fout;
    return *fout;
  }
  
  std::ostream &out() const {return *_out;}
  
  /**
   * コマンドに与えられた設定を読み解く
   * 
   * @param spec コマンド
   * @return (bool) 解読にヒットした場合はtrue、さもなければfalse
   */
  virtual bool check_spec(char *spec){
    using std::cerr;
    using std::endl;
    
#define CHECK_OPTION(name, operation, disp) \
if(std::strstr(spec, "--" #name "=") == spec){ \
  char *value(spec + strlen("--" #name "=")); \
  {operation;} \
  std::cerr << #name << ": " << disp << std::endl; \
  return true; \
}
    {
      int dummy_i;
      double dummy_d;
      if(std::sscanf(spec, "--start-gpst=%i:%lf", &dummy_i, &dummy_d) == 2){
        start_gpstime = dummy_d;
        start_gpswn = dummy_i;
        std::cerr << "start-gpst" << ": " << start_gpswn << ":" << start_gpstime << std::endl;
        return true;
      }
      if(std::sscanf(spec, "--end-gpst=%i:%lf", &dummy_i, &dummy_d) == 2){
        end_gpstime = dummy_d;
        end_gpswn = dummy_i;
        std::cerr << "end-gpst" << ": " << end_gpswn << ":" << end_gpstime << std::endl;
        return true;
      }
    }

    CHECK_OPTION(start-gpst, 
        start_gpstime = atof(value),
        start_gpstime);
    CHECK_OPTION(start-gpswn,
        start_gpswn = atof(value),
        start_gpswn);
    CHECK_OPTION(end-gpst, 
        end_gpstime = atof(value),
        end_gpstime);
    CHECK_OPTION(end-gpswn,
        end_gpswn = atoi(value),
        end_gpswn);
    CHECK_OPTION(dump-update, 
        dump_update = (strcmp(value, "on") == 0),
        (dump_update ? "on" : "off"));
    CHECK_OPTION(dump-correct, 
        dump_correct = (strcmp(value, "on") == 0),
        (dump_correct ? "on" : "off"));
    CHECK_OPTION(init-yaw-deg, 
        init_yaw_deg = atof(value),
        init_yaw_deg << " [deg]");
    CHECK_OPTION(est_bias, 
        est_bias = (strcmp(value, "on") == 0),
        (est_bias ? "on" : "off"));
    CHECK_OPTION(use_udkf, 
        use_udkf = (strcmp(value, "on") == 0),
        (use_udkf ? "on" : "off"));
    CHECK_OPTION(use_magnet, 
        use_magnet = (strcmp(value, "on") == 0),
        (use_magnet ? "on" : "off"));
    CHECK_OPTION(mag_heading_accuracy_deg,
        mag_heading_accuracy_deg = atof(value),
        mag_heading_accuracy_deg << " [deg]");
    CHECK_OPTION(yaw_correct_with_mag_when_speed_less_than_ms,
        yaw_correct_with_mag_when_speed_less_than_ms = atoi(value),
        yaw_correct_with_mag_when_speed_less_than_ms << " [m/s]");
    CHECK_OPTION(out_N_packet, 
        out_is_N_packet = (strcmp(value, "on") == 0),
        (out_is_N_packet ? "on" : "off"));
    
    if(strstr(spec, "--out=") == spec){
      char *value(spec + strlen("--out="));
      cerr << "out: ";
      _out = &(spec2ostream(value));
      return true;
    }
    
    CHECK_OPTION(in_sylphide, 
        in_sylphide = (strcmp(value, "on") == 0),
        (in_sylphide ? "on" : "off"));
    CHECK_OPTION(out_sylphide, 
        out_sylphide = (strcmp(value, "on") == 0),
        (out_sylphide ? "on" : "off"));
#undef CHECK_OPTION
    return false;
  }
};

class NAVData {
  public:
    virtual float_sylph_t longitude() const = 0;
    virtual float_sylph_t latitude() const = 0;
    virtual float_sylph_t height() const = 0;
    virtual float_sylph_t v_north() const = 0;
    virtual float_sylph_t v_east() const = 0;
    virtual float_sylph_t v_down() const = 0;
    virtual float_sylph_t heading() const = 0;
    virtual float_sylph_t euler_phi() const = 0;
    virtual float_sylph_t euler_theta() const = 0;
    virtual float_sylph_t euler_psi() const = 0;
    virtual float_sylph_t azimuth() const = 0;
    
    /**
     * 状態を出力する関数のためのラベル
     * 
     */
    virtual void label(std::ostream &out = std::cout) const {
      out << "longitude" << ", "
           << "latitude" << ", "
           << "height" << ", "
           << "v_north" << ", "
           << "v_east" << ", "
           << "v_down" << ", "
           << "Yaw(Ψ)" << ", "        //Ψ(yaw)
           << "Pitch(Θ)" << ", "      //Θ(pitch)
           << "Roll(Φ)" << ", "       //Φ(roll)
           << "Azimuth(α)" << ", ";   //α(azimuth)
    }
    
  protected:
    /**
     * 現在の状態を出力する関数
     * 
     * @param itow 現在時刻
     */
    virtual void dump(std::ostream &out) const {
      out << rad2deg(longitude()) << ", "
           << rad2deg(latitude()) << ", "
           << height() << ", "
           << v_north() << ", "
           << v_east() << ", "
           << v_down() << ", "
           << rad2deg(heading()) << ", "      //Ψ(yaw)   <- q_{g}^{b}
           << rad2deg(euler_theta()) << ", "  //Θ(pitch) <- q_{n}^{b}
           << rad2deg(euler_phi()) << ", "    //Φ(roll)  <- q_{n}^{b}
           << rad2deg(azimuth()) << ", ";     //α(azimuth)
    }
    
  public:
    /**
     * 現在の状態を出力する関数
     * 
     * @param itow 現在時刻
     */
    friend std::ostream &operator<<(std::ostream &out, const NAVData &nav){
      nav.dump(out);
      return out;
    }
    
    /**
     * N0 Packetを作成
     * 
     */
    void encode_N0(
        const float_sylph_t &itow,
        char buf[32]) const {
      typedef unsigned int v_u32_t;
      typedef int v_s32_t;
      typedef short v_s16_t;
      
      v_u32_t t(itow * 1000);
      v_s32_t lat(rad2deg(latitude()) * 1E7), 
          lng(rad2deg(longitude()) * 1E7), 
          h(height() * 1E4);
      v_s16_t v_n(v_north() * 1E2), 
          v_e(v_east() * 1E2), 
          v_d(v_down() * 1E2);
      v_s16_t psi(rad2deg(heading()) * 1E2), 
          theta(rad2deg(euler_theta()) * 1E2), 
          phi(rad2deg(euler_phi()) * 1E2);
      buf[0] = 'N';
      buf[1] = '\0';
      buf[2] = '\0';
      buf[3] = '\0';
      *(v_u32_t *)(&buf[4]) = le_char4_2_num<v_u32_t>(*(const char *)&t);
      *(v_s32_t *)(&buf[8]) = le_char4_2_num<v_s32_t>(*(const char *)&lat);
      *(v_s32_t *)(&buf[12]) = le_char4_2_num<v_s32_t>(*(const char *)&lng);
      *(v_s32_t *)(&buf[16]) = le_char4_2_num<v_s32_t>(*(const char *)&h);
      *(v_s16_t *)(&buf[20]) = le_char4_2_num<v_s16_t>(*(const char *)&v_n);
      *(v_s16_t *)(&buf[22]) = le_char4_2_num<v_s16_t>(*(const char *)&v_e);
      *(v_s16_t *)(&buf[24]) = le_char4_2_num<v_s16_t>(*(const char *)&v_d);
      *(v_s16_t *)(&buf[26]) = le_char4_2_num<v_s16_t>(*(const char *)&psi);
      *(v_s16_t *)(&buf[28]) = le_char4_2_num<v_s16_t>(*(const char *)&theta);
      *(v_s16_t *)(&buf[30]) = le_char4_2_num<v_s16_t>(*(const char *)&phi);
    }
};

#endif
