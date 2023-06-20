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
#include <boost/math/distributions.hpp>
using namespace boost::math;
%}

%define ADD_BASIC_METHODS(dist_name)
%extend boost::math::dist_name {
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
%enddef

%define INSTANTIATE(dist_name, type, class_name)
%include boost/math/distributions/ ## dist_name ## .hpp
%typemap(out, fragment=SWIG_Traits_frag(type)) std::pair<type, type> {
  %append_output(swig::from($1.first));
  %append_output(swig::from($1.second));
}
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

// workaround for hyperexponential having RealT template parameter instead of RealType
%extend boost::math::hyperexponential_distribution {
  // TODO initializer list
  %ignore hyperexponential_distribution(std::initializer_list<RealT>, std::initializer_list<RealT>);
  %ignore hyperexponential_distribution(std::initializer_list<RealT>);
};

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
INSTANTIATE(hyperexponential, double, Hyperexponential);
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
