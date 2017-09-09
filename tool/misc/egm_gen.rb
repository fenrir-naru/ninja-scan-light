#!/usr/bin/ruby
# coding: cp932

require 'stringio'

print_cpp = proc{|n_max, opt|
  coefs = []
  (2..n_max).each{|n|
    (0..n).each{|m|
      c_bar, s_bar = (opt[:CS_Bar][n][m] rescue ["C_BAR_#{n}_#{m}", "S_BAR_#{n}_#{m}"])
      coefs << "{#{c_bar}#{", #{s_bar}" if m > 0}}, // #{n}, #{m}"
    }
  }
  egm = (opt[:version] || 'EGM')
  
  print(<<__TEXT__)
#include <cmath>
#include "WGS84.h"

template <class FloatT>
struct EGM_Generic {
  struct coefficients_t {
    const FloatT c_bar;
    const FloatT s_bar;
  };

  protected:

  template <int N_MAX>
  struct p_bar_nm_t{
    unsigned int n_current;
    const FloatT sp, cp;
    FloatT p_bar_cache[3][N_MAX + 1];
    FloatT *p_bar[3];

    void next(
        const unsigned int &n,
        FloatT *p_bar_n,
        const FloatT *p_bar_n1, const FloatT *p_bar_n2) const {

      { // m = 0
        FloatT b(std::sqrt(FloatT(2 * n + 1) / (2 * n - 1)));
        FloatT c(std::sqrt(FloatT(2 * n + 1) / (2 * n - 3)));
        p_bar_n[0] = (p_bar_n1[0] * sp * (2 * n - 1) * b - p_bar_n2[0] * (n - 1) * c) / n;
      }
      for(int m(1); m <= n - 2; ++m){ // 0 < m < n-1
        FloatT a(std::pow(((2 * n - 1) * (n + m - 1)), -0.5) * (m == 1 ? std::sqrt(2) : 1));
        FloatT b(std::sqrt(FloatT(n - m) / (2 * n - 1)));
        FloatT c(std::sqrt(FloatT((n - m) * (n - m - 1)) / ((2 * n - 3) * (n + m - 1))));
        p_bar_n[m] = (p_bar_n1[m - 1] * m * (2 * n - 1) * cp * a
            + (p_bar_n1[m] * sp * (2 * n - 1) * b - p_bar_n2[m] * (n - 1) * c))
              / n * std::sqrt(FloatT(2 * n + 1) / (n + m));
      }
      { // m = n-1
        FloatT a(std::sqrt(FloatT((2 * n + 1) * (n - 1)) / 2) * (n == 2 ? std::sqrt(2) : 1));
        FloatT b(std::sqrt(2 * n + 1));
        p_bar_n[n - 1] = (p_bar_n1[n - 2] * cp * a + p_bar_n1[n - 1] * sp * b) / n;
      }
      { // m = n
        p_bar_n[n] = p_bar_n1[n - 1] * cp * std::sqrt(0.5 / n + 1);
      }
    }
    p_bar_nm_t<N_MAX> &operator++(){
      { // rotate
        FloatT *temp(p_bar[2]);
        p_bar[2] = p_bar[1];
        p_bar[1] = p_bar[0];
        p_bar[0] = temp;
      }
      next(++n_current, p_bar[0], p_bar[1], p_bar[2]);
      return *this;
    }
    p_bar_nm_t(const FloatT &phi)
        : n_current(0), sp(std::sin(phi)), cp(std::cos(phi)) {
      p_bar[0] = p_bar_cache[0];
      p_bar[1] = p_bar_cache[1];
      p_bar[2] = p_bar_cache[2];
      for(int i(0); i <= N_MAX; ++i){
        p_bar[0][i] = p_bar[1][i] = p_bar[2][i] = 0;
      }
      if(N_MAX <= 0){
        p_bar[0][0] = 1;
      }else{
        ++n_current;
        p_bar[0][0] = sp * std::sqrt(3);
        p_bar[0][1] = cp * std::sqrt(3);
        p_bar[1][0] = 1;
      }
    }
  };

  public:

  template <int N_MAX>
  struct cache_t {
    unsigned int n_max;
    FloatT a_r_n[N_MAX + 1];
    FloatT p_bar[N_MAX + 1][N_MAX + 1];
    FloatT c_ml[N_MAX + 1], s_ml[N_MAX + 1];
    cache_t() : n_max(N_MAX) {}
    cache_t &update_a_r(const FloatT &a_r){
      for(int k(0); k <= N_MAX; k++){
        a_r_n[k] = std::pow(a_r, k);
      }
      return *this;
    }
    cache_t &update_phi(const FloatT &phi){
      p_bar_nm_t<N_MAX> p_bar_gen(phi);
      p_bar_gen.next(2, p_bar[2], p_bar_gen.p_bar[0], p_bar_gen.p_bar[1]);
      p_bar_gen.next(3, p_bar[3], p_bar[2], p_bar_gen.p_bar[0]);
      for(int n(4); n <= N_MAX; n++){
        p_bar_gen.next(n, p_bar[n], p_bar[n - 1], p_bar[n - 2]);
      }
      return *this;
    }
    cache_t &update_lambda(const FloatT &lambda){
      for(int k(0); k <= N_MAX; k++){
        c_ml[k] = std::cos(lambda * k);
        s_ml[k] = std::sin(lambda * k);
      }
      return *this;
    }
    cache_t &update(const FloatT &a_r, const FloatT &phi, const FloatT &lambda){
      update_a_r(a_r);
      update_phi(phi);
      update_lambda(lambda);
      return *this;
    }
  };
  
  template <int N_MAX>
  struct buffer_t : public p_bar_nm_t<N_MAX> {
    const FloatT &a_r_orig;
    FloatT a_r_n;
    FloatT c_ml[N_MAX + 1], s_ml[N_MAX + 1];
    buffer_t(const FloatT &a_r, const FloatT &phi, const FloatT &lambda)
        : p_bar_nm_t<N_MAX>(phi), a_r_orig(a_r), a_r_n(a_r) {
      for(int m(0); m <= N_MAX; ++m){
        c_ml[m] = std::cos(lambda * m);
        s_ml[m] = std::sin(lambda * m);
      }
    }
    buffer_t &operator++(){
      p_bar_nm_t<N_MAX>::operator ++();
      a_r_n *= a_r_orig; // a_r_n = std::pow(a_r_orig, p_bar_nm_t<N_MAX>::n_current);
      return *this;
    }
  };

#{[true, false].collect{|use_cache|
  <<__FUNC__
  template <int N_MAX>
  static FloatT gravity_potential_dimless(
      const coefficients_t coefs[],
      #{use_cache ? "const cache_t<N_MAX> &x" : "const FloatT &a_r, const FloatT &phi, const FloatT &lambda"}) {

    FloatT sum_n(1);#{" buffer_t<N_MAX> x(a_r, phi, lambda);" unless use_cache}
    for(int n(2), coef_i(0); n <= #{use_cache ? 'x.n_max' : 'N_MAX'}; n++){
      FloatT sum_m(0);#{" ++x;" unless use_cache}
      for(int m(0); m <= n; m++, coef_i++){
        sum_m += x.p_bar[#{use_cache ? 'n' : '0'}][m]
            * (coefs[coef_i].c_bar * x.c_ml[m] + coefs[coef_i].s_bar * x.s_ml[m]);
      }
      sum_n += x.a_r_n#{'[n]' if use_cache} * sum_m;
    }
    return sum_n;
  }
__FUNC__
}.join}

#{[true, false].collect{|use_cache|
  <<__FUNC__
  template <int N_MAX>
  static FloatT gravity_r_dimless(
      const coefficients_t coefs[],
      #{use_cache ? "const cache_t<N_MAX> &x" : "const FloatT &a_r, const FloatT &phi, const FloatT &lambda"}) {

    FloatT sum_n(1);#{" buffer_t<N_MAX> x(a_r, phi, lambda);" unless use_cache}
    for(int n(2), coef_i(0); n <= #{use_cache ? 'x.n_max' : 'N_MAX'}; n++){
      FloatT sum_m(0);#{" ++x;" unless use_cache}
      for(int m(0); m <= n; m++, coef_i++){
        sum_m += x.p_bar[#{use_cache ? 'n' : '0'}][m]
            * (coefs[coef_i].c_bar * x.c_ml[m] + coefs[coef_i].s_bar * x.s_ml[m]);
      }
      sum_n += x.a_r_n#{'[n]' if use_cache} * sum_m * (n + 1);
    }
    return sum_n;
  }
__FUNC__
}.join}

#{[true, false].collect{|use_cache|
  <<__FUNC__
  template <int N_MAX>
  static FloatT gravity_phi_dimless(
      const coefficients_t coefs[],
      #{use_cache ? "const cache_t<N_MAX> &x" : "const FloatT &a_r, const FloatT &phi, const FloatT &lambda"}) {

    FloatT sum_n(0);#{" buffer_t<N_MAX> x(a_r, phi, lambda);" unless use_cache}
    for(int n(2), coef_i(0); n <= #{use_cache ? 'x.n_max' : 'N_MAX'}; n++){
      FloatT sum_m(0);#{" ++x;" unless use_cache}
      for(int m(0); m <= n; m++, coef_i++){
        sum_m += (-x.p_bar[#{use_cache ? 'n' : '0'}][m] * m * x.c_ml[1] * x.s_ml[1]
              + ((m == n) ? 0 : std::sqrt((n - m) * (n + m + 1)) * x.p_bar[#{use_cache ? 'n' : '0'}][m+1]))
            * (coefs[coef_i].c_bar * x.c_ml[m] + coefs[coef_i].s_bar * x.s_ml[m]);
      }
      sum_n += x.a_r_n#{'[n]' if use_cache} * sum_m;
    }
    return sum_n;
  }
__FUNC__
}.join}

#{[true, false].collect{|use_cache|
  <<__FUNC__
  template <int N_MAX>
  static FloatT gravity_lambda_dimless(
      const coefficients_t coefs[],
      #{use_cache ? "const cache_t<N_MAX> &x" : "const FloatT &a_r, const FloatT &phi, const FloatT &lambda"}) {

    FloatT sum_n(0);#{" buffer_t<N_MAX> x(a_r, phi, lambda);" unless use_cache}
    for(int n(2), coef_i(0); n <= #{use_cache ? 'x.n_max' : 'N_MAX'}; n++){
      FloatT sum_m(0);#{" ++x;" unless use_cache}
      for(int m(0); m <= n; m++, coef_i++){
        sum_m += x.p_bar[#{use_cache ? 'n' : '0'}][m] * m
            * (-coefs[coef_i].c_bar * x.s_ml[m] + coefs[coef_i].s_bar * x.c_ml[m]);
      }
      sum_n += x.a_r_n#{'[n]' if use_cache} * sum_m;
    }
    return sum_n;
  }
__FUNC__
}.join}
};

template <class FloatT>
struct #{egm}_#{n_max}_Generic : public EGM_Generic<FloatT> {
  static const typename EGM_Generic<FloatT>::coefficients_t coefficients[];
  typedef typename EGM_Generic<FloatT>::template cache_t<#{n_max}> cache_t;

#define make_func(fname, sf) \\
static FloatT fname( \\
    const FloatT &r, const FloatT &phi, const FloatT &lambda){ \\
  return (sf) * EGM_Generic<FloatT>::template fname ## _dimless<#{n_max}>( \\
      coefficients, \\
      WGS84Generic<FloatT>::R_e / r, phi, lambda); \\
} \\
static FloatT fname( \\
    const cache_t &cache, \\
    const FloatT &r, const FloatT &phi, const FloatT &lambda){ \\
  return (sf) * EGM_Generic<FloatT>::template fname ## _dimless<#{n_max}>( \\
      coefficients, cache); \\
}

  make_func(gravity_potential, WGS84Generic<FloatT>::mu_Earth_refined / r);
  make_func(gravity_r, -WGS84Generic<FloatT>::mu_Earth_refined / std::pow(r, 2));
  make_func(gravity_phi, WGS84Generic<FloatT>::mu_Earth_refined / std::pow(r, 2));
  make_func(gravity_lambda, WGS84Generic<FloatT>::mu_Earth_refined / std::pow(r, 2) / std::cos(phi));
};

template<class FloatT>
const typename EGM_Generic<FloatT>::coefficients_t #{egm}_#{n_max}_Generic<FloatT>::coefficients[] = {
  #{coefs.join("\n  ")}
};

typedef #{egm}_#{n_max}_Generic<double> #{egm}_#{n_max};
__TEXT__

  puts(<<__TEXT__) if opt[:test_cpp]

#include <iostream>
#include <iomanip>

using namespace std;

int main(){

  cout << setprecision(16);

  const double alt(0);
  const bool use_cache(true);

  typedef #{egm}_#{n_max} egm_t;
  typedef egm_t::cache_t cache_t;

  cache_t #{n_max <= 100 ? "cache_i" : "*cache_p(new cache_t())"};
  cache_t &cache(#{n_max <= 100 ? "cache_i" : "*cache_p"});

  for(int lat_deg(90); lat_deg >= -90; lat_deg--){

    cerr << lat_deg << std::endl;
    double lat(M_PI / 180 * lat_deg);
    typename WGS84::xz_t xz(WGS84::xz(lat, alt));
    double phi(xz.geocentric_latitude()), r(xz.distance());
    cache.update_phi(phi);
    cache.update_a_r(WGS84::R_e / r);

    for(int lng_deg(0); lng_deg <= 359; lng_deg++){
      double lng(M_PI / 180 * lng_deg);
      double lambda(lng);
      cache.update_lambda(lambda);

      cout << (use_cache 
          ? egm_t::gravity_potential(cache, r, phi, lambda)
          : egm_t::gravity_potential(r, phi, lambda)) << ", ";
    }
    cout << endl;
  }
}
__TEXT__
}

read_coef_file = proc{|f, n_max|
  next [nil, n_max] unless f
  parse = proc{|io|
    res = []
    io.each{|line|
      values = line.chomp.sub(/^ +/, '').split(/ +/)
      n, m = [0, 1].collect{|i| values[i].to_i}
      unless res[n]
        break if ((n > n_max) rescue false)
        res[n] = []
      end
      res[n][m] = [2, 3].collect{|i| values[i].gsub(/[dD]([+-]?\d+)/, 'e\1')}
    }
    [res, res.size - 1]
  }
  f_orig = f
  if f_orig =~ /^https?:\/\//
    require 'open-uri'
    require 'tempfile'
    f_tmp = Tempfile.open(File::basename(__FILE__, '.*'))
    open(f_orig){|io|
      f_tmp.write(io.read)
    }
    f = f_tmp.path
  end
  case f_orig
  when /\.g?z$/
    IO::popen("gunzip -c #{f}", 'r'){|io|
      parse.call(io)
    }
  else
    open(f){|io|
      parse.call(io)
    }
  end
}

$stderr.puts "Usage: #{$0} n [CS_bar_file]"
$stderr.puts "ex) #{$0} 10 http://earth-info.nga.mil/GandG/wgs84/gravitymod/egm96/egm96.z"

opt = {
  :version => 'EGM',
}

ARGV.reject!{|arg|
  next false unless arg =~ /^--([^=]+)=?/
  opt[$1.to_sym] = ($' == '' ? true : $')
  true
}

n_max = begin
  i = Integer(ARGV.shift)
  i < 2 ? nil : i
rescue
  nil
end
opt[:CS_Bar], n_max = read_coef_file.call(ARGV.shift, n_max)
print_cpp.call(n_max, opt)