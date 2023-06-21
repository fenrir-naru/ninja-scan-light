/**
 * @file SWIG interface file for boost::math::distributions
 *
 */

%module BoostDistributions

%include std_common.i
%include std_except.i

%{
#include <sstream>
#include <stdexcept>

#include <boost/version.hpp>
#define BOOST_VERSION_LESS_THAN(ver) (BOOST_VERSION < ver)

#include <boost/math/distributions.hpp>
namespace boost { namespace math {
template <int RequiredVersion, bool check_passed = !BOOST_VERSION_LESS_THAN(RequiredVersion)>
struct distribution_shim_t {
  template <class DistT, class ValueT>
  inline static typename DistT::value_type pdf(const DistT &dist, const ValueT &x){
    return boost::math::pdf(dist, x);
  }
  template <class DistT, class ValueT>
  inline static typename DistT::value_type cdf(
      const DistT &dist, const ValueT &x, const bool &is_complement = false){
    return is_complement ?
        boost::math::cdf(complement(dist, x)) : boost::math::cdf(dist, x);
  }
  template <class DistT, class ValueT>
  inline static typename DistT::value_type quantile(
      const DistT &dist, const ValueT &p, const bool &is_complement = false){
    return is_complement ?
        boost::math::quantile(complement(dist, p)) : boost::math::quantile(dist, p);
  }
#define GEN_FUNC(func_name) \
template <class DistT> \
inline static typename DistT::value_type func_name(const DistT &dist){ \
  return boost::math::func_name(dist); \
}
  GEN_FUNC(mean);
  GEN_FUNC(median);
  GEN_FUNC(mode);
  GEN_FUNC(standard_deviation);
  GEN_FUNC(variance);
  GEN_FUNC(skewness);
  GEN_FUNC(kurtosis);
  GEN_FUNC(kurtosis_excess);
#undef GEN_FUNC
#define GEN_FUNC(func_name) \
template <class DistT> \
inline static std::pair<typename DistT::value_type, typename DistT::value_type> \
    func_name(const DistT &dist){ \
  return boost::math::func_name(dist); \
}
  GEN_FUNC(range);
  GEN_FUNC(support);
#undef GEN_FUNC
};
template <int RequiredVersion>
struct distribution_shim_t<RequiredVersion, false> { // dummy
  static const std::string err_msg(){
    std::stringstream ss;
    ss << "BOOST_VERSION(" << BOOST_VERSION 
        << ") should be >= " << RequiredVersion;
    return ss.str();
  }
  template <class DistT, class ValueT>
  inline static typename DistT::value_type pdf(const DistT &dist, const ValueT &x){
    throw std::logic_error(err_msg());
  }
  template <class DistT, class ValueT>
  inline static typename DistT::value_type cdf(
      const DistT &dist, const ValueT &x, const bool &is_complement = false){
    throw std::logic_error(err_msg());
  }
  template <class DistT, class ValueT>
  inline static typename DistT::value_type quantile(
      const DistT &dist, const ValueT &p, const bool &is_complement = false){
    throw std::logic_error(err_msg());
  }
#define GEN_FUNC(func_name) \
template <class DistT> \
inline static typename DistT::value_type func_name(const DistT &dist){ \
  throw std::logic_error(err_msg()); \
}
  GEN_FUNC(mean);
  GEN_FUNC(median);
  GEN_FUNC(mode);
  GEN_FUNC(standard_deviation);
  GEN_FUNC(variance);
  GEN_FUNC(skewness);
  GEN_FUNC(kurtosis);
  GEN_FUNC(kurtosis_excess);
#undef GEN_FUNC
#define GEN_FUNC(func_name) \
template <class DistT> \
inline static std::pair<typename DistT::value_type, typename DistT::value_type> \
    func_name(const DistT &dist){ \
  throw std::logic_error(err_msg()); \
}
  GEN_FUNC(range);
  GEN_FUNC(support);
#undef GEN_FUNC
};
} }
using namespace boost::math;
%}
%include boost/version.hpp

%define INSTANTIATE(dist_name, type, class_name, min_ver)
%{
#if BOOST_VERSION_LESS_THAN(min_ver)
namespace boost{ namespace math{
template <class RealType = double, class Policy = policies::policy<> >
struct dist_name ## _distribution { // dummy
  typedef RealType value_type;
};
} }
#endif
%}
#if BOOST_VERSION >= min_ver
%include boost/math/distributions/ ## dist_name ## .hpp
#else
namespace boost{ namespace math{
template <class RealType = double, class Policy = policies::policy<> >
struct dist_name ## _distribution { // dummy
  typedef RealType value_type;
};
} }
#endif
%typemap(out, fragment=SWIG_Traits_frag(type)) std::pair<type, type> {
  %append_output(swig::from($1.first));
  %append_output(swig::from($1.second));
}
%extend boost::math::dist_name ## _distribution {
  %catches(std::logic_error);
#define shim_t boost::math::distribution_shim_t<min_ver>
  value_type pdf(const value_type &x) const {
    return shim_t::pdf(*$self, x);
  }
  value_type cdf(const value_type &x, const bool &is_complement = false) const {
    return shim_t::cdf(*$self, x, is_complement);
  }
  value_type quantile(const value_type &p, const bool &is_complement = false) const {
    return shim_t::quantile(*$self, p, is_complement);
  }
  value_type mean() const {return shim_t::mean(*$self);}
  value_type median() const {return shim_t::median(*$self);}
  value_type mode() const {return shim_t::mode(*$self);}
  value_type standard_deviation() const {return shim_t::standard_deviation(*$self);}
  value_type variance() const {return shim_t::variance(*$self);}
  value_type skewness() const {return shim_t::skewness(*$self);}
  value_type kurtosis() const {return shim_t::kurtosis(*$self);}
  value_type kurtosis_excess() const {return shim_t::kurtosis_excess(*$self);}
  std::pair<value_type, value_type> range() const {return shim_t::range(*$self);}
  std::pair<value_type, value_type> support() const {return shim_t::support(*$self);}
#undef shim_t
};
%template(class_name) boost::math:: ## dist_name ## _distribution<type, policies::policy<> >;
%enddef

%extend boost::math::cauchy_distribution {
  %ignore mean;
  %ignore standard_deviation;
  %ignore variance;
  %ignore skewness;
  %ignore kurtosis;
  %ignore kurtosis_excess;
};
%extend boost::math::non_central_beta_distribution {
  %ignore skewness;
  %ignore kurtosis;
  %ignore kurtosis_excess;
};

// workaround for hyperexponential having RealT template parameter instead of RealType
%extend boost::math::hyperexponential_distribution {
  // TODO initializer list
  %ignore hyperexponential_distribution(std::initializer_list<RealT>, std::initializer_list<RealT>);
  %ignore hyperexponential_distribution(std::initializer_list<RealT>);
};

INSTANTIATE(arcsine, double, Arcsine, 105800);
INSTANTIATE(bernoulli, double, Bernoulli, 103500);
INSTANTIATE(beta, double, Beta, 103500);
INSTANTIATE(binomial, double, Binomial, 103500);
INSTANTIATE(cauchy, double, Cauchy, 103500);
INSTANTIATE(chi_squared, double, ChiSquared, 103500);
INSTANTIATE(exponential, double, Exponential, 103500);
INSTANTIATE(extreme_value, double, ExtremeValue, 103500);
INSTANTIATE(fisher_f, double, FisherF, 103500);
INSTANTIATE(gamma, double, Gamma, 103500);
INSTANTIATE(geometric, double, Geometric, 104600);
INSTANTIATE(hyperexponential, double, Hyperexponential, 105700);
INSTANTIATE(hypergeometric, double, Hypergeometric, 104000);
INSTANTIATE(inverse_chi_squared, double, InverseChiSquared, 104500);
INSTANTIATE(inverse_gamma, double, InverseGamma, 104500);
INSTANTIATE(inverse_gaussian, double, InverseGaussian, 104600);
INSTANTIATE(kolmogorov_smirnov, double, KolmogorovSmirnov, 107500);
INSTANTIATE(laplace, double, Laplace, 104000);
INSTANTIATE(logistic, double, Logistic, 104000);
INSTANTIATE(lognormal, double, Lognormal, 103500);
INSTANTIATE(negative_binomial, double, NegativeBinomial, 103500);
INSTANTIATE(non_central_beta, double, NonCentralBeta, 103600);
INSTANTIATE(non_central_chi_squared, double, NonCentralChiSquared, 103600);
INSTANTIATE(non_central_f, double, NonCentralF, 103600);
INSTANTIATE(non_central_t, double, NonCentralT, 103600);
INSTANTIATE(normal, double, Normal, 103500);
INSTANTIATE(pareto, double, Pareto, 103500);
INSTANTIATE(poisson, double, Poisson, 103500);
INSTANTIATE(rayleigh, double, Rayleigh, 103500);
INSTANTIATE(skew_normal, double, SkewNormal, 105000);
INSTANTIATE(students_t, double, StudentsT, 103500);
INSTANTIATE(triangular, double, Triangular, 103500);
INSTANTIATE(uniform, double, Uniform, 103500);
INSTANTIATE(weibull, double, Weibull, 103500);
