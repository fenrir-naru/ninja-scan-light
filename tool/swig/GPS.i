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
  %typemap(out) struct tm {
    $result = rb_ary_new3(
        6,
        INT2NUM((double)($1.tm_year + 1900)),
        INT2NUM((double)($1.tm_mon + 1)),
        INT2NUM((double)($1.tm_mday)),
        INT2NUM((double)($1.tm_hour)),
        INT2NUM((double)($1.tm_min)),
        INT2NUM((double)($1.tm_sec)));
  }
  %typemap(in) const struct tm & {
    $1 = new struct tm;
    int *dst[] = {
      &($1->tm_year),
      &($1->tm_mon),
      &($1->tm_mday),
      &($1->tm_hour),
      &($1->tm_min),
      &($1->tm_sec),
    };
    for(unsigned i(0); i < sizeof(dst) / sizeof(dst[0]); ++i){
      VALUE v = rb_ary_entry($input, i);
      if(v == Qnil){break;}
      if(dst[i] == &($1->tm_year)){
        *dst[i] = NUM2INT(v) - 1900;
      }else if(dst[i] == &($1->tm_mon)){
        *dst[i] = NUM2INT(v) - 1;
      }else{
        *dst[i] = NUM2INT(v);
      }
    }
  }
  %typemap(typecheck, precedence=SWIG_TYPECHECK_POINTER) const struct tm & {
    $1 = (TYPE($input) == T_ARRAY) ? 1 : 0;
  }
  %typemap(freearg) const struct tm & {
    delete $1;
  }
#endif
}

%include navigation/GPS.h

CONCRETIZE(double);