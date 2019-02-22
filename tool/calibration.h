/*
 * Copyright (c) 2019, M.Naruoka (fenrir)
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

#ifndef __CALIBRATION_H__
#define __CALIBRATION_H__

#include <iostream>
#include <cstdlib>

template <class FloatT>
struct StandardCalibration {

  int index_base, index_temp_ch;
  template <std::size_t N>
  struct calibration_info_t {
    FloatT bias_tc[N];
    FloatT bias_base[N];
    FloatT sf[N];
    FloatT alignment[N][N];
    FloatT sigma[N];
    static void set(char *spec, FloatT target[N]){
      for(int i(0); i < N; i++){
        target[i] = std::strtod(spec, &spec);
      }
    }
    static void set(char *spec, FloatT target[N][N]){
      for(int i(0); i < N; i++){
        for(int j(0); j < N; j++){
          target[i][j] = std::strtod(spec, &spec);
        }
      }
    }
    static std::ostream &dump(std::ostream &out, const FloatT target[N]){
      for(int i(0); i < N; i++){
        out << " " << target[i];
      }
      return out;
    }
    static std::ostream &dump(std::ostream &out, const FloatT target[N][N]){
      for(int i(0); i < N; i++){
        for(int j(0); j < N; j++){
          out << " " << target[i][j];
        }
      }
      return out;
    }
  };
  typedef calibration_info_t<3> dof3_t;
  dof3_t accel, gyro;

  static const dof3_t pass_through;

  bool check_spec(const char *line, const char *(*get_value)(const char *in, const char *header)){
    const char *value;
    if(value = get_value(line, "index_base")){
      index_base = std::atoi(value);
      return true;
    }
    if(value = get_value(line, "index_temp_ch")){
      index_temp_ch = std::atoi(value);
      return true;
    }
#define TO_STRING(name) # name
#define check_proc(name, sensor, item) \
if(value = get_value(line, TO_STRING(name))){ \
  dof3_t::set(const_cast<char *>(value), sensor.item); \
  return true; \
}
    check_proc(acc_bias_tc, accel, bias_tc);
    check_proc(acc_bias, accel, bias_base);
    check_proc(acc_sf, accel, sf);
    check_proc(acc_mis, accel, alignment);
    check_proc(gyro_bias_tc, gyro, bias_tc);
    check_proc(gyro_bias, gyro, bias_base);
    check_proc(gyro_sf, gyro, sf);
    check_proc(gyro_mis, gyro, alignment);
    check_proc(sigma_accel, accel, sigma);
    check_proc(sigma_gyro, gyro, sigma);
#undef check_proc
#undef TO_STRING

    return false;
  }

  template <std::size_t N>
  bool check_specs(const char *(&lines)[N], const char *(*get_value)(const char *, const char *)){
    for(int i(0); i < N; ++i){
      if(!check_spec(lines[i], get_value)){return false;}
    }
    return true;
  }

  friend std::ostream &operator<<(
      std::ostream &out, const StandardCalibration &calib){
    out << "index_base " << calib.index_base << std::endl;
    out << "index_temp_ch " << calib.index_temp_ch << std::endl;
#define TO_STRING(name) # name
#define dump_proc(name, sensor, item) \
  out << TO_STRING(name); \
  dof3_t::dump(out, calib.sensor.item)
    dump_proc(acc_bias_tc, accel, bias_tc) << std::endl;
    dump_proc(acc_bias, accel, bias_base) << std::endl;
    dump_proc(acc_sf, accel, sf) << std::endl;
    dump_proc(acc_mis, accel, alignment) << std::endl;
    dump_proc(gyro_bias_tc, gyro, bias_tc) << std::endl;
    dump_proc(gyro_bias, gyro, bias_base) << std::endl;
    dump_proc(gyro_sf, gyro, sf) << std::endl;
    dump_proc(gyro_mis, gyro, alignment) << std::endl;
    dump_proc(sigma_accel, accel, sigma) << std::endl;
    dump_proc(sigma_gyro, gyro, sigma);
#undef dump_proc
#undef TO_STRING
    return out;
  }

  template <class NumType, std::size_t N>
  static void calibrate(
      const NumType raw[],
      const NumType &bias_mod,
      const calibration_info_t<N> &info,
      FloatT (&res)[N]) {

    // Temperature compensation
    FloatT bias[N];
    for(int i(0); i < N; i++){
      bias[i] = info.bias_base[i] + (info.bias_tc[i] * bias_mod);
    }

    // Convert raw values to physical quantity by using scale factor
    FloatT tmp[N];
    for(int i(0); i < N; i++){
      tmp[i] = (((FloatT)raw[i] - bias[i]) / info.sf[i]);
    }

    // Misalignment compensation
    for(int i(0); i < N; i++){
      res[i] = 0;
      for(int j(0); j < N; j++){
        res[i] += info.alignment[i][j] * tmp[j];
      }
    }
  }

  StandardCalibration()
      : index_base(0), index_temp_ch(0), accel(pass_through), gyro(pass_through) {}
  ~StandardCalibration() {}

  struct result_t {
    FloatT values[3];
  };

  /**
   * Get acceleration in m/s^2
   */
  result_t raw2accel(const int *raw_data) const{
    result_t res;
    calibrate(
        &raw_data[index_base], raw_data[index_temp_ch],
        accel, res.values);
    return res;
  }

  /**
   * Get angular speed in rad/sec
   */
  result_t raw2omega(const int *raw_data) const{
    result_t res;
    calibrate(
        &raw_data[index_base + 3], raw_data[index_temp_ch],
        gyro, res.values);
    return res;
  }

  /**
   * Accelerometer output variance in [m/s^2]^2
   */
  result_t sigma_accel() const{
    result_t res = {{accel.sigma[0], accel.sigma[1], accel.sigma[2]}};
    return res;
  }

  /**
   * Angular speed output variance in X, Y, Z axes, [rad/s]^2
   */
  result_t sigma_gyro() const{
    result_t res = {{gyro.sigma[0], gyro.sigma[1], gyro.sigma[2]}};
    return res;
  }
};

template <class FloatT>
const typename StandardCalibration<FloatT>::dof3_t StandardCalibration<FloatT>::pass_through = {
  {0, 0, 0}, // bias_tc
  {0, 0, 0}, // bias_base
  {1, 1, 1}, // sf
  {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, // alignment
  {1, 1, 1}, // sigma
};

#endif /* __CALIBRATION_H__ */
