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
};
%enddef

%define INSTANTIATE(dist_name, type, class_name)
%include /usr/include/boost/math/distributions/ ## dist_name ## .hpp
ADD_BASIC_METHODS(dist_name ## _distribution);
%template(class_name) boost::math:: ## dist_name ## _distribution<type, policies::policy<> >;
%enddef

INSTANTIATE(arcsine, double, Arcsine);
INSTANTIATE(bernoulli, double, Bernoulli);
INSTANTIATE(beta, double, Beta);
INSTANTIATE(binomial, double, Binomial);
//INSTANTIATE(cauchy, double, Cauchy); // TODO due to mean()
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
//INSTANTIATE(non_central_beta, double, NonCentralBeta); // TODO due to skewness()
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
