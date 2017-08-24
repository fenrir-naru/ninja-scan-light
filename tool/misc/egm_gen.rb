#!/usr/bin/ruby
# coding: cp932

require 'stringio'

# derivative
class Array
  def derivative!
    self.size.times{|i|
      self[i] *= i
    }
    self.shift
    self
  end
end

# @return [coefficient(c) of sin^0, c of sin^1, ...]
def legendre(n, cache = {})
  return n if cache.include?(n)
  
  res = case n
  when 0
    [Rational(1)]
  when 1
    [Rational(0), Rational(1)]
  else
    ([Rational(0)] + legendre(n - 1).collect{|v|
      v / n * (n * 2 - 1)
    }).zip(legendre(n - 2).collect{|v|
      v / n * (n - 1)
    }).collect{|a, b|
      b ? a - b : a
    }
  end
  cache[n] = res
  return res
end

print_proc = proc{|n_max, opt|
  legendre_cache = {}
  decl = []
  upper = []
  lower = []
  (2..n_max).each{|n|
    p_n = legendre(n, legendre_cache)
    p_nm = [p_n]
    p_nm_bar_coef = [(n * 2 + 1).to_f]
    n.times{|m|
      p_nm << p_nm[-1].clone.derivative!
      p_nm_bar_coef << p_nm_bar_coef[-1] / ((n - m) * (n + 1 + m))
    }
    p_nm.each_with_index{|f, m|
      c = Math::sqrt(p_nm_bar_coef[m] * (m == 0 ? 1 : 2))
      c_bar, s_bar = (opt[:CS_Bar][[n, m]] rescue ["C_BAR_#{n}_#{m}", "S_BAR_#{n}_#{m}"])
      decl << "static const FloatT P_#{n}_#{m}[];"
      upper << "template<class FloatT> const FloatT EGM<FloatT>::P_#{n}_#{m}[] = {
    #{f.collect{|v| "%a"%[v.to_f]}.join(', ')}};"
      lower << "{P_#{n}_#{m}, #{c_bar}, #{s_bar}, #{"%a"%[c]}}, // #{n}, #{m}"
    }
  }
  puts(<<__TEXT__)
#include <cmath>
#include "WGS84.h"

template <class FloatT>
struct EGM_Generic {
  struct coefficients_t {
    const FloatT *p_nm;
    const FloatT c_bar;
    const FloatT s_bar;
    const FloatT p_nm_bar;
  };
  
  protected:  
  template <int N_MAX>
  struct cache_t {
    FloatT cos_ml[N_MAX + 1], sin_ml[N_MAX + 1];
    FloatT cos_mp[N_MAX + 1], sin_mp[N_MAX + 1];
    cache_t(const FloatT &phi, const FloatT &lambda){
      for(int m(0); m <= N_MAX; m++){
        cos_ml[m] = std::cos(lambda * m);
        sin_ml[m] = std::cos(lambda * m);
        cos_mp[m] = std::pow(std::cos(phi), m);
        sin_mp[m] = std::pow(std::sin(phi), m);
      }
    }
  };

  template <int N_MAX>
  static FloatT gravity_potential_scale_n_fixed(
      const coefficients_t coefs[],
      const FloatT &a_r, const FloatT &phi, const FloatT &lambda) {

    cache_t<N_MAX> x(phi, lambda);

    FloatT res(1);
    for(int n(2), coef_i(0); n <= N_MAX; n++){
      FloatT sum_m(0);
      for(int m(0); m <= n; m++, coef_i++){
        FloatT p_nm(0);
        for(int pn(0); pn <= n - m; pn++){
          p_nm += coefs[coef_i].p_nm[pn] * x.sin_mp[pn];
        }
        sum_m += coefs[coef_i].p_nm_bar * x.cos_mp[m] * p_nm
            * (coefs[coef_i].c_bar * x.cos_ml[m] + coefs[coef_i].s_bar * x.sin_ml[m]);
      }
      res += std::pow(a_r, n) * sum_m;
    }
    return res;
  }
  template <int N_MAX>
  static FloatT gravity_r_scale_n_fixed(
      const coefficients_t coefs[],
      const FloatT &a_r, const FloatT &phi, const FloatT &lambda) {

    cache_t<N_MAX> x(phi, lambda);

    FloatT res(-1);
    for(int n(2), coef_i(0); n <= N_MAX; n++){
      FloatT sum_m(0);
      for(int m(0); m <= n; m++, coef_i++){
        FloatT p_nm(0);
        for(int pn(0); pn <= n - m; pn++){
          p_nm += coefs[coef_i].p_nm[pn] * x.sin_mp[pn];
        }
        sum_m += coefs[coef_i].p_nm_bar * x.cos_mp[m] * p_nm
            * (coefs[coef_i].c_bar * x.cos_ml[m] + coefs[coef_i].s_bar * x.sin_ml[m]);
      }
      res += std::pow(a_r, n) * -(n + 1) * sum_m;
    }
    return res;
  }
  
  template <int N_MAX>
  static FloatT gravity_phi_scale_n_fixed(
      const coefficients_t coefs[],
      const FloatT &a_r, const FloatT &phi, const FloatT &lambda) {

    cache_t<N_MAX + 1> x(phi, lambda);

    FloatT res(0);
    for(int n(2), coef_i(0); n <= N_MAX; n++){
      FloatT sum_m(0);
      for(int m(0); m <= n; m++, coef_i++){
        FloatT p_nm(0);
        if(m >= 1){
          FloatT p_nm_mneg(0);
          for(int pn(0); pn <= n - m; pn++){
            p_nm_mneg += coefs[coef_i].p_nm[pn] * x.sin_mp[pn + 1];
          }
          p_nm += x.cos_mp[m - 1] * p_nm_mneg * -m;
        }
        {
          FloatT p_nm_mpos(0);
          for(int pn(1); pn <= n - m; pn++){
            p_nm_mpos += coefs[coef_i].p_nm[pn] * x.sin_mp[pn - 1] * pn;
          }
          p_nm += x.cos_mp[m + 1] * p_nm_mpos;
        }
        sum_m += coefs[coef_i].p_nm_bar * p_nm
            * (coefs[coef_i].c_bar * x.cos_ml[m] + coefs[coef_i].s_bar * x.sin_ml[m]);
      }
      res += std::pow(a_r, n) * sum_m;
    }
    return res;
  }
  template <int N_MAX>
  static FloatT gravity_lambda_scale_n_fixed(
      const coefficients_t coefs[],
      const FloatT &a_r, const FloatT &phi, const FloatT &lambda) {

    cache_t<N_MAX> x(phi, lambda);

    FloatT res(0);
    for(int n(2), coef_i(0); n <= N_MAX; n++){
      FloatT sum_m(0);
      for(int m(1); m <= n; m++, coef_i++){
        FloatT p_nm(0);
        for(int pn(0); pn <= n - m; pn++){
          p_nm += coefs[coef_i].p_nm[pn] * x.sin_mp[pn];
        }
        sum_m += coefs[coef_i].p_nm_bar * x.cos_mp[m] * p_nm
            * m * (coefs[coef_i].c_bar * -x.sin_ml[m] + coefs[coef_i].s_bar * x.cos_ml[m]);
      }
      res += std::pow(a_r, n) * sum_m;
    }
    return res;
  }
};

template <class FloatT>
struct EGM : public EGM_Generic<FloatT> {
  static const typename EGM_Generic<FloatT>::coefficients_t coefficients[];
  static FloatT gravity_potential(
      const FloatT &r, const FloatT &phi, const FloatT &lambda){
    return (WGS84Generic<FloatT>::mu_Earth / r)
        * EGM_Generic<FloatT>::template gravity_potential_scale_n_fixed<#{n_max}>(
            coefficients, WGS84Generic<FloatT>::R_e / r, phi, lambda);
  }
  static FloatT gravity_r(
      const FloatT &r, const FloatT &phi, const FloatT &lambda){
    return WGS84Generic<FloatT>::mu_Earth * std::pow(r, -2)
        * EGM_Generic<FloatT>::template gravity_r_scale_n_fixed<#{n_max}>(
            coefficients, WGS84Generic<FloatT>::R_e / r, phi, lambda);
  }
  static FloatT gravity_phi(
      const FloatT &r, const FloatT &phi, const FloatT &lambda){
    return WGS84Generic<FloatT>::mu_Earth * std::pow(r, -2)
        * EGM_Generic<FloatT>::template gravity_phi_scale_n_fixed<#{n_max}>(
            coefficients, WGS84Generic<FloatT>::R_e / r, phi, lambda);
  }
  static FloatT gravity_lambda(
      const FloatT &r, const FloatT &phi, const FloatT &lambda){
    return WGS84Generic<FloatT>::mu_Earth * std::pow(r, -2)
        * EGM_Generic<FloatT>::template gravity_lambda_scale_n_fixed<#{n_max}>(
            coefficients, WGS84Generic<FloatT>::R_e / r, phi, lambda);
  }
  #{decl.join("\n  ")}
};

#{upper.join("\n")}

template<class FloatT>
const typename EGM_Generic<FloatT>::coefficients_t EGM<FloatT>::coefficients[] = {
  #{lower.join("\n  ")}
};
__TEXT__
}

read_coef_file = proc{|f|
  next nil unless f
  require 'open-uri' if f =~ /^https?:\/\//
  data = open(f){|io|
    case f
    when /\.z$/
      require 'tempfile'
      Tempfile.open(File::basename(__FILE__, '.*')){|f_tmp|
        f_tmp.write(io.read)
        IO::popen("gunzip -c #{f_tmp.path}", 'r'){|io2|
          io2.read
        }
      }
    else
      io.read
    end
  }
  res = {}
  data.each_line{|line|
    values = line.chomp.sub(/^ +/, '').split(/ +/)
    res[[0, 1].collect{|i| values[i].to_i}] = [2, 3].collect{|i| values[i]}
  }
  res
}

$stderr.puts "Usage #{$0} n [CS_bar_file]"
$stderr.puts "ex) #{$0} 10 http://earth-info.nga.mil/GandG/wgs84/gravitymod/egm96/egm96.z"
print_proc.call(ARGV.shift.to_i, {:CS_Bar => read_coef_file.call(ARGV.shift)})
