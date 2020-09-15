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
};
%enddef

%define INSTANTIATE(dist_name, class_name)
%include /usr/include/boost/math/distributions/ ## dist_name ## .hpp
ADD_BASIC_METHODS(dist_name ## _distribution);
%template(class_name) boost::math:: ## dist_name ## _distribution<double, policies::policy<> >;
%enddef

INSTANTIATE(non_central_chi_squared, NonCentralChiSquared);
INSTANTIATE(normal, Normal);
