/**
 * @file SWIG interface file for boost::math::distributions
 *
 */

%module BoostDistributions

%include exception.i
%exception {
  try {
    $action
  } catch (const std::exception& e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  }
}

%include std_pair.i

%{
#include <boost/math/distributions.hpp>
using namespace boost::math;
%}

%define ADD_BASIC_METHODS(dist_name)
%extend boost::math::dist_name {
  RealType pdf(const RealType &x) const {
    return pdf(*$self, x);
  }
  RealType cdf(const RealType &x, const bool &is_complement = false) const {
    return is_complement ? cdf(complement(*$self, x)) : cdf(*$self, x);
  }
  RealType quantile(const RealType &p, const bool &is_complement = false) const {
    return is_complement ? quantile(complement(*$self, p)) : quantile(*$self, p);
  }
  RealType mean() const {return mean(*$self);}
  RealType median() const {return median(*$self);}
  RealType mode() const {return mode(*$self);}
  RealType standard_deviation() const {return standard_deviation(*$self);}
  RealType variance() const {return variance(*$self);}
  RealType skewness() const {return skewness(*$self);}
  RealType kurtosis() const {return kurtosis(*$self);}
  RealType kurtosis_excess() const {return kurtosis_excess(*$self);}
#if defined(SWIGRUBY)
  %typemap(out) std::pair<RealType, RealType> {
    VALUE arr(rb_ary_new2(2));
    rb_ary_push(arr, DBL2NUM($1.first));
    rb_ary_push(arr, DBL2NUM($1.second));
    $result = arr;
  }
#endif
  std::pair<RealType, RealType> range() const {return range(*$self);}
  std::pair<RealType, RealType> support() const {return support(*$self);}
};
%enddef

%define INSTANTIATE(dist_name, type, class_name)
%include boost/math/distributions/ ## dist_name ## .hpp
ADD_BASIC_METHODS(dist_name ## _distribution);
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

// workaround for hyperexponential haaving RealT template parameter instead of RealType
%extend boost::math::hyperexponential_distribution {
  %ignore hyperexponential_distribution(std::initializer_list<RealT>, std::initializer_list<RealT>);
  %ignore hyperexponential_distribution(std::initializer_list<RealT>);
  RealT pdf(const RealT &x) const {
    return pdf(*$self, x);
  }
  RealT cdf(const RealT &x, const bool &is_complement = false) const {
    return is_complement ? cdf(complement(*$self, x)) : cdf(*$self, x);
  }
  RealT quantile(const RealT &p, const bool &is_complement = false) const {
    return is_complement ? quantile(complement(*$self, p)) : quantile(*$self, p);
  }
  RealT mean() const {return mean(*$self);}
  RealT median() const {return median(*$self);}
  RealT mode() const {return mode(*$self);}
  RealT standard_deviation() const {return standard_deviation(*$self);}
  RealT variance() const {return variance(*$self);}
  RealT skewness() const {return skewness(*$self);}
  RealT kurtosis() const {return kurtosis(*$self);}
  RealT kurtosis_excess() const {return kurtosis_excess(*$self);}
#if defined(SWIGRUBY)
  %typemap(out) std::pair<RealT, RealT> {
    VALUE arr(rb_ary_new2(2));
    rb_ary_push(arr, DBL2NUM($1.first));
    rb_ary_push(arr, DBL2NUM($1.second));
    $result = arr;
  }
#endif
  std::pair<RealT, RealT> range() const {return range(*$self);}
  std::pair<RealT, RealT> support() const {return support(*$self);}
};
#if !defined(__MINGW__)
%include boost/math/distributions/hyperexponential.hpp
%template(Hyperexponential) boost::math::hyperexponential_distribution<double, policies::policy<> >;
#endif

INSTANTIATE(arcsine, double, Arcsine);
INSTANTIATE(bernoulli, double, Bernoulli);
INSTANTIATE(beta, double, Beta);
INSTANTIATE(binomial, double, Binomial);
INSTANTIATE(cauchy, double, Cauchy);
INSTANTIATE(chi_squared, double, ChiSquared);
INSTANTIATE(exponential, double, Exponential);
INSTANTIATE(extreme_value, double, ExtremeValue);
INSTANTIATE(fisher_f, double, FisherF);
INSTANTIATE(gamma, double, Gamma);
INSTANTIATE(geometric, double, Geometric);
//INSTANTIATE(hyperexponential, double, Hyperexponential); // TODO due to initializer list
INSTANTIATE(hypergeometric, double, Hypergeometric);
INSTANTIATE(inverse_chi_squared, double, InverseChiSquared);
INSTANTIATE(inverse_gamma, double, InverseGamma);
INSTANTIATE(inverse_gaussian, double, InverseGaussian);
INSTANTIATE(laplace, double, Laplace);
INSTANTIATE(logistic, double, Logistic);
INSTANTIATE(lognormal, double, Lognormal);
INSTANTIATE(negative_binomial, double, NegativeBinomial);
INSTANTIATE(non_central_beta, double, NonCentralBeta);
INSTANTIATE(non_central_chi_squared, double, NonCentralChiSquared);
INSTANTIATE(non_central_f, double, NonCentralF);
INSTANTIATE(non_central_t, double, NonCentralT);
INSTANTIATE(normal, double, Normal);
INSTANTIATE(pareto, double, Pareto);
INSTANTIATE(poisson, double, Poisson);
INSTANTIATE(rayleigh, double, Rayleigh);
INSTANTIATE(skew_normal, double, SkewNormal);
INSTANTIATE(students_t, double, StudentsT);
INSTANTIATE(triangular, double, Triangular);
INSTANTIATE(uniform, double, Uniform);
INSTANTIATE(weibull, double, Weibull);
