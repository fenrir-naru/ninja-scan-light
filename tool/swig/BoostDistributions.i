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

%include std_common.i

%{
#include <boost/version.hpp>
#include <boost/math/distributions.hpp>
using namespace boost::math;
%}
%include boost/version.hpp

%define INSTANTIATE(dist_name, type, class_name, min_ver)
#if BOOST_VERSION >= min_ver
%include boost/math/distributions/ ## dist_name ## .hpp
%typemap(out, fragment=SWIG_Traits_frag(type)) std::pair<type, type> {
  %append_output(swig::from($1.first));
  %append_output(swig::from($1.second));
}
%extend boost::math::dist_name ## _distribution {
  value_type pdf(const value_type &x) const {
    return pdf(*$self, x);
  }
  value_type cdf(const value_type &x, const bool &is_complement = false) const {
    return is_complement ? cdf(complement(*$self, x)) : cdf(*$self, x);
  }
  value_type quantile(const value_type &p, const bool &is_complement = false) const {
    return is_complement ? quantile(complement(*$self, p)) : quantile(*$self, p);
  }
  value_type mean() const {return mean(*$self);}
  value_type median() const {return median(*$self);}
  value_type mode() const {return mode(*$self);}
  value_type standard_deviation() const {return standard_deviation(*$self);}
  value_type variance() const {return variance(*$self);}
  value_type skewness() const {return skewness(*$self);}
  value_type kurtosis() const {return kurtosis(*$self);}
  value_type kurtosis_excess() const {return kurtosis_excess(*$self);}
  std::pair<value_type, value_type> range() const {return range(*$self);}
  std::pair<value_type, value_type> support() const {return support(*$self);}
};
%template(class_name) boost::math:: ## dist_name ## _distribution<type, policies::policy<> >;
#endif
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
