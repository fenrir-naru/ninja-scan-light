/**
 *
 *
 *
 */

%module NavigationModel

%{
#include <string>

#include "navigation/MagneticField.h"
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
%template(MagneticField) MagneticFieldGeneric<type>;
%template(IGRF11) IGRF11Generic<type>;
%template(IGRF12) IGRF12Generic<type>;
%template(WMM2010) WMM2010Generic<type>;
%inline %{
struct MagneticFieldModel : public MagneticFieldGeneric<type>::model_t {
  typedef typename MagneticFieldGeneric<type>::model_t super_t;
  MagneticFieldModel(const super_t &m) : super_t(m) {}
  const char *name() const {return super_t::name;}
  const type year() const {return super_t::year;}
  const int dof() const {return super_t::dof;}
  const type *coef() const {return super_t::coef;}
};
%}
%enddef

%extend MagneticFieldGeneric {
  %ignore model_t;
  %ignore field_components_res_t;
  %ignore latlng_t;
#if defined(SWIGRUBY)
  %typemap(out) field_components_res_t {
    $result = rb_hash_new();
    rb_hash_aset($result, ID2SYM(rb_intern("north")), DBL2NUM((double)($1.north)));
    rb_hash_aset($result, ID2SYM(rb_intern("east")), DBL2NUM((double)($1.east)));
    rb_hash_aset($result, ID2SYM(rb_intern("down")), DBL2NUM((double)($1.down)));
  }
  %typemap(out) latlng_t {
    $result = rb_hash_new();
    rb_hash_aset($result, ID2SYM(rb_intern("lat")), DBL2NUM((double)($1.latitude)));
    rb_hash_aset($result, ID2SYM(rb_intern("lng")), DBL2NUM((double)($1.longitude)));
  }
#endif
  %typemap(out) model_t {
  	$result = SWIG_NewPointerObj(SWIG_as_voidptr(new MagneticFieldModel($1)), $descriptor(MagneticFieldModel *), 1);
  }
  %typemap(varout) model_t {
  	$result = SWIG_NewPointerObj(SWIG_as_voidptr(new MagneticFieldModel($1)), $descriptor(MagneticFieldModel *), 1);
  }
}

%include navigation/MagneticField.h

CONCRETIZE(double);