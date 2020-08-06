/**
 * @file SWIG interface file for header files in param directory
 *
 */

%module SylphideMath

#define ENABLE_IOSTREAM 1

%{
#include <sstream>
#include <vector>

#if defined(SWIGRUBY) && defined(isfinite)
#undef isfinite_
#undef isfinite
#endif

#include "param/complex.h"
#include "param/matrix.h"

#if defined(SWIGRUBY) && defined(isfinite_)
#undef isfinite_
#define isfinite(x) finite(x)
#endif
%}

%include std_string.i
%include std_vector.i
%include exception.i

%exception {
  try {
    $action
  } catch (const std::exception& e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  }
}

%define STR(x)
#x
%enddef
%define MAKE_SETTER(name, type)
%rename(STR(name ## =)) set_ ## name;
type set_ ## name (const type &v) {
  return (self->name() = v);
}
%enddef
%define MAKE_GETTER(name, type)
%rename(STR(name)) get_ ## name;
type get_ ## name () {
  return self->name();
}
%enddef
%define MAKE_TO_S(type)
%extend type{
  std::string to_s() const {
    std::stringstream s;
    s << (*self);
    return s.str();
  }
};
%enddef

%feature("autodoc", "1");

%ignore Complex::real;
%ignore Complex::imaginary;
%ignore operator<<(std::ostream &, const Complex &);

%include param/complex.h

MAKE_TO_S(Complex);

%extend Complex{
  MAKE_SETTER(real, FloatT);
  MAKE_GETTER(real, FloatT);
  MAKE_SETTER(imaginary, FloatT);
  MAKE_GETTER(imaginary, FloatT);
  %alias power "**"
  %alias arg "angle,phase"
  %alias conjugate "conj"
  // fdiv // TODO
  // finite? // TODO
  %alias set_imaginary "imag=";
  %alias get_imaginary "imag";
  // infinite? // TODO
  %alias abs "magnitude"
  // polar // TOOO
  // rect,rectangle // TODO
};

%define CONCRETIZE_COMPLEX(type, suffix)
%template(Complex ## suffix) Complex<type>;
%enddef

CONCRETIZE_COMPLEX(double, D);