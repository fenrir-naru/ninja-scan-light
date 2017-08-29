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
  return cache[n] if cache.include?(n)
  
  res = case n
  when 0
    [Rational(1)]
  when 1
    [Rational(0), Rational(1)]
  else
    proc{ # definition: (2 ** (-n)) * (1/n!) * (d/dx)^n (x^2 - 1)^n
      coefs = proc{ # (x^2 - 1)^n
        res = []
        sign = ((n % 2 == 0) ? 1 : -1)
        proc{ # binomial expansion
          binexp = [1, 1]
          if cache[:b] then
            next cache[:b][n] if cache[:b][n]
            (2..(n-1)).to_a.reverse_each{|i|
              if cache[:b][i] then
                binexp = cache[:b][i].clone
                break
              end
            }
          else
            cache[:b] = {}  
          end
          (n + 1 - binexp.size).times{
            last = 0
            binexp.collect!{|v|
              temp = last
              last = v
              temp + v
            }
            binexp << 1
          }
          cache[:b][n] = binexp
        }.call.each{|v, i|
          res += [Rational(v * sign), 0]
          sign *= -1
        }
        res.pop
        res
      }.call
      
      (1..n).to_a.reverse_each{|i| # differential n times and divide by scale factor
        coefs.derivative!.collect!{|v|
          v / 2 / i
        }
      }
      
      coefs
    }.call
=begin
    # definition: n legendre(n) = (2*n - 1) * x * legendre(n-1) - (n-1) * legendre(n-2)
    ([Rational(0)] + legendre(n - 1).collect{|v|
      v / n * (n * 2 - 1)
    }).zip(legendre(n - 2).collect{|v|
      v / n * (n - 1)
    }).collect{|a, b|
      b ? a - b : a
    }
=end
  end
  cache[n] = res
  return res
end

def iterate_p_nm(n_max, cache = {}, &b)
  (2..n_max).each{|n|
    p_n = legendre(n, cache)
    p_nm = [p_n]
    p_nm_bar_coef = [Rational(n * 2 + 1)]
    (1..n).each{|m|
      p_nm << p_nm[-1].clone.derivative!
      p_nm_bar_coef << p_nm_bar_coef[-1] / ((n + 1 - m) * (n + m))
    }
    p_nm.each_with_index{|poly, m|
      b.call(n, m, p_nm_bar_coef[m] * (m == 0 ? 1 : 2), poly)
    }
  }
end

calc_potential = proc{|n_max, opt|
  r, phi, lamb = [:r, :phi, :lamb].collect{|k| opt[k]}
  a_r = 6378137.0 / r
  
  a_r_n = (0..n_max).to_a.collect{|n| a_r ** n}
  c_mp, s_mp = (0..n_max).to_a.collect{|m|
    [:cos, :sin].collect{|f| Math::send(f, phi) ** m}
  }.transpose
  c_ml, s_ml = (0..n_max).to_a.collect{|m|
    [:cos, :sin].collect{|f| Math::send(f, lamb * m)}
  }.transpose
  opt[:legendre] ||= {}
  
  sum_n = 1.0
  iterate_p_nm(n_max, opt[:legendre]){|n, m, c2, poly|
    c_bar, s_bar = opt[:CS_Bar][n][m].collect{|v| v.to_f}
    sum_poly = 0.0
    poly.each_with_index{|k, i|
      sum_poly += k.to_f * s_mp[i]
    }
    sum_n += a_r_n[n] * sum_poly * c_mp[m] * Math::sqrt(c2) * (c_bar * c_ml[m] + s_bar * s_ml[m])
  }
  
  3986004.418E8 / r * sum_n
}

print_cpp = proc{|n_max, opt|
  decl = []
  upper = []
  lower = []
  iterate_p_nm(n_max){|n, m, c2, poly|
    $stderr.puts "n, m = [#{n}, #{m}]"
    c_bar, s_bar = (opt[:CS_Bar][n][m] rescue ["C_BAR_#{n}_#{m}", "S_BAR_#{n}_#{m}"])
    decl << "static const FloatT P_#{n}_#{m}[];"
    upper << "template<class FloatT> const FloatT EGM<FloatT>::P_#{n}_#{m}[] = {
    #{poly.collect{|v| "%a"%[Math::sqrt(c2 * v * v)]}.join(', ')}};"
    lower << "{P_#{n}_#{m}, #{c_bar}, #{s_bar}}, // #{n}, #{m}"
  }
  make_func_without_cache = proc{|fname, extra_cache|
    cache_size = "N_MAX"
    cache_size += " + #{extra_cache}" if extra_cache
    <<__TEXT__
  template <int N_MAX>
  static FloatT #{fname}(
      const coefficients_t coefs[],
      const FloatT &a_r, const FloatT &phi, const FloatT &lambda) {
    cache_t<#{cache_size}> x(a_r, phi, lambda);
    return #{fname}<N_MAX, #{cache_size}>(coefs, x);
  }
__TEXT__
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
  };
  
  protected:  
  template <int N_MAX>
  struct cache_t {
    FloatT a_r_n[N_MAX + 1];
    FloatT cos_ml[N_MAX + 1], sin_ml[N_MAX + 1];
    FloatT cos_mp[N_MAX + 1], sin_mp[N_MAX + 1];
    cache_t(const FloatT &a_r, const FloatT &phi, const FloatT &lambda){
      for(int k(0); k <= N_MAX; k++){
        a_r_n[k] = std::pow(a_r, k);
        cos_ml[k] = std::cos(lambda * k);
        sin_ml[k] = std::sin(lambda * k);
        cos_mp[k] = std::pow(std::cos(phi), k);
        sin_mp[k] = std::pow(std::sin(phi), k);
      }
    }
  };

  template <int N_MAX, int N_CACHE>
  static FloatT gravity_potential_scale_n_fixed(
      const coefficients_t coefs[],
      const cache_t<N_CACHE> &x) {

    FloatT res(1);
    for(int n(2), coef_i(0); n <= N_MAX; n++){
      FloatT sum_m(0);
      for(int m(0); m <= n; m++, coef_i++){
        FloatT p_nm(0);
        for(int pn(0); pn <= n - m; pn++){
          p_nm += coefs[coef_i].p_nm[pn] * x.sin_mp[pn];
        }
        sum_m += x.cos_mp[m] * p_nm
            * (coefs[coef_i].c_bar * x.cos_ml[m] + coefs[coef_i].s_bar * x.sin_ml[m]);
      }
      res += x.a_r_n[n] * sum_m;
    }
    return res;
  }
#{make_func_without_cache.call(:gravity_potential_scale_n_fixed)}
  
  template <int N_MAX, int N_CACHE>
  static FloatT gravity_r_scale_n_fixed(
      const coefficients_t coefs[],
      const cache_t<N_CACHE> &x) {

    FloatT res(-1);
    for(int n(2), coef_i(0); n <= N_MAX; n++){
      FloatT sum_m(0);
      for(int m(0); m <= n; m++, coef_i++){
        FloatT p_nm(0);
        for(int pn(0); pn <= n - m; pn++){
          p_nm += coefs[coef_i].p_nm[pn] * x.sin_mp[pn];
        }
        sum_m += x.cos_mp[m] * p_nm
            * (coefs[coef_i].c_bar * x.cos_ml[m] + coefs[coef_i].s_bar * x.sin_ml[m]);
      }
      res += x.a_r_n[n] * -(n + 1) * sum_m;
    }
    return res;
  }
#{make_func_without_cache.call(:gravity_r_scale_n_fixed)}
  
  template <int N_MAX, int N_CACHE>
  static FloatT gravity_phi_scale_n_fixed(
      const coefficients_t coefs[],
      const cache_t<N_CACHE> &x) {

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
        sum_m += p_nm
            * (coefs[coef_i].c_bar * x.cos_ml[m] + coefs[coef_i].s_bar * x.sin_ml[m]);
      }
      res += x.a_r_n[n] * sum_m;
    }
    return res;
  }
#{make_func_without_cache.call(:gravity_phi_scale_n_fixed, 1)}
  
  template <int N_MAX, int N_CACHE>
  static FloatT gravity_lambda_scale_n_fixed(
      const coefficients_t coefs[],
      const cache_t<N_CACHE> &x) {

    FloatT res(0);
    for(int n(2), coef_i(0); n <= N_MAX; n++){
      FloatT sum_m(0);
      for(int m(1); m <= n; m++, coef_i++){
        FloatT p_nm(0);
        for(int pn(0); pn <= n - m; pn++){
          p_nm += coefs[coef_i].p_nm[pn] * x.sin_mp[pn];
        }
        sum_m += x.cos_mp[m] * p_nm
            * m * (coefs[coef_i].c_bar * -x.sin_ml[m] + coefs[coef_i].s_bar * x.cos_ml[m]);
      }
      res += x.a_r_n[n] * sum_m;
    }
    return res;
  }
#{make_func_without_cache.call(:gravity_lambda_scale_n_fixed)}
  
  public:
  struct gravity_t {
    FloatT r, phi, lambda;
  };
  protected:
  template <int N_MAX>
  static gravity_t gravity_scale_n_fixed(
      const coefficients_t coefs[],
      const FloatT &a_r, const FloatT &phi, const FloatT &lambda){
    cache_t<N_MAX + 1> x(a_r, phi, lambda);
    gravity_t res = {
        gravity_r_scale_n_fixed<N_MAX, N_MAX + 1>(coefs, x),
        gravity_phi_scale_n_fixed<N_MAX, N_MAX + 1>(coefs, x),
        gravity_lambda_scale_n_fixed<N_MAX, N_MAX + 1>(coefs, x)};
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
  static typename EGM_Generic<FloatT>::gravity_t gravity(
      const FloatT &r, const FloatT &phi, const FloatT &lambda){
    typename EGM_Generic<FloatT>::gravity_t res(
        EGM_Generic<FloatT>::template gravity_scale_n_fixed<#{n_max}>(
            coefficients, WGS84Generic<FloatT>::R_e / r, phi, lambda));
    FloatT sf(WGS84Generic<FloatT>::mu_Earth * std::pow(r, -2));
    res.r *= sf;
    res.phi *= sf;
    res.lambda *= sf;
    return res;
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
    n, m = [0, 1].collect{|i| values[i].to_i}
    res[n] ||= []
    res[n][m] = [2, 3].collect{|i| values[i]}
  }
  res
}

$stderr.puts "Usage #{$0} n [CS_bar_file]"
$stderr.puts "ex) #{$0} 10 http://earth-info.nga.mil/GandG/wgs84/gravitymod/egm96/egm96.z"

opt = {
  :n_max => ARGV.shift.to_i,
  :CS_Bar => read_coef_file.call(ARGV.shift),
}
print_cpp.call(opt[:n_max], opt)

proc{
  require 'WGS84'
  (-90..90).to_a.reverse_each{|lat_deg|
    $stderr.puts lat_deg
    phi_gd = Math::PI / 180 * lat_deg
    x, z = WGS84::xz(phi_gd, opt[:alt] || 0)
    r = ((x ** 2) + (z ** 2)) ** 0.5
    phi_gc = Math::atan2(z, x)
    puts (0..359).to_a.collect{|lng_deg|
      calc_potential.call(opt[:n_max], opt.merge!({
        :r => r,
        :phi => phi_gc,
        :lamb => Math::PI / 180 * lng_deg,
      }))
    }.join(',')
  }
}.call if false
