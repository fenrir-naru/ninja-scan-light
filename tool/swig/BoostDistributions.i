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
  RealType pdf(const RealType &x) const {return pdf(*$self, x);}
  RealType cdf(const RealType &x) const {return cdf(*$self, x);}
  RealType quantile(const RealType &p) const {return quantile(*$self, p);}
  // TODO complement
};
%enddef

%include /usr/include/boost/math/distributions/non_central_chi_squared.hpp
ADD_BASIC_METHODS(non_central_chi_squared_distribution);
%template(NonCentralChiSquared) boost::math::non_central_chi_squared_distribution<double, policies::policy<> >;

%include /usr/include/boost/math/distributions/normal.hpp
ADD_BASIC_METHODS(normal_distribution);
%template(NormalDistribution) boost::math::normal_distribution<double, policies::policy<> >;