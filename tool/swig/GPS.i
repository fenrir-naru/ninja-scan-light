/**
 *
 *
 *
 */

%module GPS

%{
#if defined(SWIGRUBY)
#undef isfinite
#endif

#include <string>
#include <vector>

#include "navigation/GPS.h"
%}

%include typemaps.i
%include std_string.i
%include exception.i

%exception {
  try {
    $action
  } catch (const std::exception& e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  }
}

%feature("autodoc", "1");

%define CONCRETIZE(type)
%template(Time) GPS_Time<type>;
%enddef

%extend GPS_Time {
#if defined(SWIGRUBY)
  %typemap(out) std::tm {
    $result = rb_ary_new3(
        6,
        INT2NUM($1.tm_year + 1900),
        INT2NUM($1.tm_mon + 1),
        INT2NUM($1.tm_mday),
        INT2NUM($1.tm_hour),
        INT2NUM($1.tm_min),
        INT2NUM($1.tm_sec));
  }
  %typemap(in) const std::tm & (std::tm temp = {0}) {
    $1 = &temp;
    int *dst[] = {
      &(temp.tm_year),
      &(temp.tm_mon),
      &(temp.tm_mday),
      &(temp.tm_hour),
      &(temp.tm_min),
      &(temp.tm_sec),
    };
    int i_max(sizeof(dst) / sizeof(dst[0]));
    if(i_max > RARRAY_LEN($input)){i_max = RARRAY_LEN($input);}
    for(int i(0); i < i_max; ++i){
      VALUE v = rb_ary_entry($input, i);
      if(v == Qnil){break;}
      if(dst[i] == &(temp.tm_year)){
        *dst[i] = NUM2INT(v) - 1900;
      }else if(dst[i] == &(temp.tm_mon)){
        *dst[i] = NUM2INT(v) - 1;
      }else{
        *dst[i] = NUM2INT(v);
      }
    }
  }
  %typemap(typecheck, precedence=SWIG_TYPECHECK_POINTER) const std::tm & {
    $1 = (TYPE($input) == T_ARRAY) ? 1 : 0;
  }
#endif
  %apply int *OUTPUT { int *week };
  %apply FloatT *OUTPUT { FloatT *seconds };
  void to_a(int *week, FloatT *seconds) const {
    *week = self->week;
    *seconds = self->seconds;
  }
}

%include navigation/GPS.h

CONCRETIZE(double);