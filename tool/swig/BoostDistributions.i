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
#define GEN_FUNC(func_name) \
template <class DistT, class ValueT> \
inline static typename DistT::value_type func_name(const DistT &dist, const ValueT &x){ \
  return boost::math::func_name(dist, x); \
}
  GEN_FUNC(pdf);
  GEN_FUNC(hazard);
  GEN_FUNC(chf);
#undef GEN_FUNC
#define GEN_FUNC(func_name) \
template <class DistT, class ValueT> \
inline static typename DistT::value_type func_name( \
    const DistT &dist, const ValueT &x, const bool &is_complement = false){ \
  return is_complement \
      ? boost::math::func_name(complement(dist, x)) \
      : boost::math::func_name(dist, x); \
}
  GEN_FUNC(cdf);
  GEN_FUNC(quantile);
#undef GEN_FUNC
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
#define GEN_FUNC(func_name) \
template <class DistT, class ValueT> \
inline static typename DistT::value_type func_name(const DistT &dist, const ValueT &x){ \
  throw std::logic_error(err_msg()); \
}
  GEN_FUNC(pdf);
  GEN_FUNC(hazard);
  GEN_FUNC(chf);
#undef GEN_FUNC
#define GEN_FUNC(func_name) \
template <class DistT, class ValueT> \
inline static typename DistT::value_type func_name( \
    const DistT &dist, const ValueT &x, const bool &is_complement = false){ \
  throw std::logic_error(err_msg()); \
}
  GEN_FUNC(cdf);
  GEN_FUNC(quantile);
#undef GEN_FUNC
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
template <class RealType, int RequiredVersion>
struct distribution_dummy_t {
  typedef RealType value_type;
  static const std::string err_msg(){
    return distribution_shim_t<RequiredVersion, false>::err_msg();
  }
  distribution_dummy_t(){throw std::logic_error(err_msg());}
  
  // static/member functions
#define GEN_FUNC_m(ret_type, fun_name) \
ret_type fun_name(...) const {throw std::logic_error(err_msg());}
#define GEN_FUNC_s(ret_type, fun_name) \
static ret_type fun_name(...){throw std::logic_error(err_msg());}

  GEN_FUNC_m(RealType, x_max); // arcsine
  GEN_FUNC_m(RealType, x_min); // arcsine
  GEN_FUNC_m(RealType, success_fraction);
      // bernoulli, binomial, geometric, negative_binomial
  GEN_FUNC_m(RealType, alpha); // beta, non_central_beta
  GEN_FUNC_m(RealType, beta); // beta, non_central_beta
  GEN_FUNC_s(RealType, find_alpha); // beta
  GEN_FUNC_s(RealType, find_beta); // beta
  GEN_FUNC_s(RealType, find_lower_bound_on_p);
      // binomial, geometric, negative_binomial
  GEN_FUNC_s(RealType, find_maximum_number_of_trials);
      // binomial, geometric, negative_binomial
  GEN_FUNC_s(RealType, find_minimum_number_of_trials);
      // binomial, geometric, negative_binomial
  GEN_FUNC_s(RealType, find_upper_bound_on_p);
      // binomial, geometric, negative_binomial
  GEN_FUNC_m(RealType, trails); // binomial
  GEN_FUNC_m(RealType, location);
      // cauchy, extreme_value, inverse_gaussian, 
      // laplace, logistic, lognormal, normal, skew_normal
  GEN_FUNC_m(RealType, scale);
      // cauchy, extreme_value, gamma, inverse_chi_squared,
      // inverse_gamma, inverse_gaussian, laplace, logistic,
      // lognormal, normal, pareto, skew_normal, weibull
  GEN_FUNC_s(RealType, find_degrees_of_freedom);
      // chi_squared, non_central_chi_squared, students_t
  GEN_FUNC_m(RealType, degrees_of_freedom);
      // chi_squared, inverse_chi_squared, non_central_chi_squared,
      // non_central_t, students_t
  GEN_FUNC_m(RealType, lambda); // exponential
  GEN_FUNC_m(RealType, degrees_of_freedom1); // fisher_f, non_central_f
  GEN_FUNC_m(RealType, degrees_of_freedom2); // fisher_f, non_central_f
  GEN_FUNC_m(RealType, shape);
      // gamma, inverse_gamma, inverse_gaussian, pareto,
      // skew_normal, weibull
  GEN_FUNC_m(RealType, successes); // geometric, negative_binomial
  GEN_FUNC_m(std::vector<RealType>, probabilities); // hyperexponential
  GEN_FUNC_m(std::vector<RealType>, rates); // hyperexponential
  GEN_FUNC_m(std::size_t, num_phases); // hyperexponential
  GEN_FUNC_m(bool, check_params); // hypergeometric
  GEN_FUNC_m(bool, check_x); // hypergeometric
  GEN_FUNC_m(unsigned, defective); // hypergeometric
  GEN_FUNC_m(unsigned, sample_count); // hypergeometric
  GEN_FUNC_m(unsigned, total); // hypergeometric
  GEN_FUNC_m(RealType, mean); // inverse_gaussian, poisson
  GEN_FUNC_m(RealType, number_of_observations); // kolmogorov_smirnov
  GEN_FUNC_m(RealType, non_centrality);
      // non_central_beta, non_central_chi_squared, non_central_f,
      // non_central_t
  GEN_FUNC_s(RealType, find_non_centrality);
      // non_central_chi_squared
  GEN_FUNC_m(RealType, standard_deviation); // normal
  GEN_FUNC_m(RealType, sigma); // rayleigh
  GEN_FUNC_m(RealType, lower); // triangular, uniform
  GEN_FUNC_m(RealType, mode); // triangular
  GEN_FUNC_m(RealType, upper); // triangular, uniform

#undef GEN_FUNC_s
#undef GEN_FUNC_m
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
struct dist_name ## _distribution : public distribution_dummy_t<RealType, min_ver> { // dummy
  dist_name ## _distribution(...) : distribution_dummy_t<RealType, min_ver>() {}
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
  dist_name ## _distribution();
};
} }
#endif
%typemap(out, fragment=SWIG_Traits_frag(type)) std::pair<type, type> {
  %append_output(swig::from($1.first));
  %append_output(swig::from($1.second));
}
%extend boost::math::dist_name ## _distribution {
  %catches(std::logic_error) dist_name ## _distribution; // for ctor
  %catches(std::logic_error, std::runtime_error); // for the following member functions
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
  value_type hazard(const value_type &x) const {
    return shim_t::hazard(*$self, x);
  }
  value_type chf(const value_type &x) const {
    return shim_t::chf(*$self, x);
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
  %ignore hyperexponential_distribution(std::initializer_list<RealT>, std::initializer_list<RealT>);
  %ignore hyperexponential_distribution(std::initializer_list<RealT>);
  %typemap(out, fragment=SWIG_Traits_frag(RealT)) std::vector<RealT> {
    for(typename $1_type::const_iterator it($1.begin()), it_end($1.end());
        it != it_end; ++it){
      %append_output(swig::from(*it));
    }
  }
  %typemap(in, fragment=SWIG_Traits_frag(RealT)) const std::vector<RealT> & (std::vector<RealT> temp) {
#if defined(SWIGRUBY)
    if(RB_TYPE_P($input, T_ARRAY)){
      RealT v;
      for(unsigned int i(0), i_end((unsigned int)RARRAY_LEN($input)); i < i_end; ++i){
        VALUE val(RARRAY_AREF($input, i));
        if(!SWIG_IsOK(swig::asval(val, &v))){break;}
        temp.push_back(v);
      }
    }
#endif
    $1 = &temp;
  }
#if BOOST_VERSION >= 105700
  hyperexponential_distribution(
      const std::vector<RealT> &prob, const std::vector<RealT> &range){
    return new hyperexponential_distribution(
        prob.begin(), prob.end(), range.begin(), range.end());
  }
  hyperexponential_distribution(
      const std::vector<RealT> &range){
    return new hyperexponential_distribution(range);
    // two argument constructors have an overload problem. It is checked with 1.82.0.
    //return new hyperexponential_distribution(range.begin(), range.end());
  }
#endif
};

// workaround for swig parse error of hyperexponential::is_iterator (>=1.77.0)
#if BOOST_VERSION >= 107700
namespace boost{ namespace math{
template <typename RealT = double, typename PolicyT = policies::policy<> >
class hyperexponential_distribution { /* extracted from ver 1.81.0 */
  public:
    typedef RealT value_type;
    typedef PolicyT policy_type;
    
    hyperexponential_distribution();

    // Four arg constructor: no ambiguity here, the arguments must be two pairs of iterators:
    template <typename ProbIterT, typename RateIterT>
    hyperexponential_distribution(
        ProbIterT prob_first, ProbIterT prob_last,
        RateIterT rate_first, RateIterT rate_last);

    // Two arg constructor from 2 ranges, we SFINAE this out of existence if
    // either argument type is incrementable as in that case the type is
    // probably an iterator:
    template <
        typename ProbRangeT, typename RateRangeT, 
        typename std::enable_if<!is_iterator<ProbRangeT>::value && 
          !is_iterator<RateRangeT>::value, bool>::type = true>
    hyperexponential_distribution(
        ProbRangeT const& prob_range, RateRangeT const& rate_range);

    // Two arg constructor for a pair of iterators: we SFINAE this out of
    // existence if neither argument types are incrementable.
    // Note that we allow different argument types here to allow for
    // construction from an array plus a pointer into that array.
    template <
        typename RateIterT, typename RateIterT2,
        typename std::enable_if<
          is_iterator<RateIterT>::value || is_iterator<RateIterT2>::value, bool>::type = true>
    hyperexponential_distribution(
        RateIterT const& rate_first, RateIterT2 const& rate_last);

    // Initializer list constructor: allows for construction from array literals:
    hyperexponential_distribution(std::initializer_list<RealT> l1, std::initializer_list<RealT> l2);

    hyperexponential_distribution(std::initializer_list<RealT> l1);

    // Single argument constructor: argument must be a range.
    template <typename RateRangeT>
    hyperexponential_distribution(RateRangeT const& rate_range);

    std::vector<RealT> probabilities() const;
    std::vector<RealT> rates() const;
    std::size_t num_phases() const;
};
} }
#define BOOST_MATH_DISTRIBUTIONS_HYPEREXPONENTIAL_HPP
#endif

%define CTACH_LOGIC_ERROR(dist_name, func_name)
%extend boost::math::dist_name ## _distribution {
  %catches(std::logic_error) func_name;
};
%enddef

%define CTACH_LOGIC_ERROR_FOR_FIND_METHODS(dist_name)
CTACH_LOGIC_ERROR(dist_name, find_lower_bound_on_p);
CTACH_LOGIC_ERROR(dist_name, find_upper_bound_on_p);
CTACH_LOGIC_ERROR(dist_name, find_minimum_number_of_trials);
CTACH_LOGIC_ERROR(dist_name, find_maximum_number_of_trials);
%enddef
CTACH_LOGIC_ERROR_FOR_FIND_METHODS(binomial);
CTACH_LOGIC_ERROR_FOR_FIND_METHODS(geometric);
CTACH_LOGIC_ERROR_FOR_FIND_METHODS(negative_binomial);
#undef CTACH_LOGIC_ERROR_FOR_FIND_METHODS

CTACH_LOGIC_ERROR(chi_squared, find_degrees_of_freedom);
CTACH_LOGIC_ERROR(non_central_chi_squared, find_degrees_of_freedom);
CTACH_LOGIC_ERROR(students_t, find_degrees_of_freedom);

#undef CTACH_LOGIC_ERROR

%feature("autodoc", "1");

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

#undef INSTANTIATE
