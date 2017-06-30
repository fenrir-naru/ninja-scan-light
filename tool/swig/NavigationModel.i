/**
 *
 *
 *
 */

%module NavigationModel

%{
#include <string>

#include "navigation/MagneticField.h"
#include "navigation/WGS84.h"
#include "navigation/NTCM.h"
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
  typename MagneticFieldGeneric<type>::field_components_res_t field_components_geocentric(
      const type &geocentric_latitude,
      const type &longitude_rad,
      const type &radius_meter = 6371.2E3){
    return MagneticFieldGeneric<type>::field_components_geocentric(*this, geocentric_latitude, longitude_rad, radius_meter);
  }
  typename MagneticFieldGeneric<type>::field_components_res_t field_components(
      const type &latitude_rad,
      const type &longitude_rad,
      const type &height_meter){
    return MagneticFieldGeneric<type>::field_components_geocentric(*this, latitude_rad, longitude_rad, height_meter);
  }
  typename MagneticFieldGeneric<type>::latlng_t geomagnetic_latlng(
      const type &geocentric_latitude,
      const type &longitude){
    return MagneticFieldGeneric<type>::geomagnetic_latlng(*this, geocentric_latitude, longitude);
  }
};
%}

%template(WGS84) WGS84Generic<type>;

%template(NTCM_GL) NTCM_GL_Generic<type>;
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
%include navigation/WGS84.h
%include navigation/NTCM.h

CONCRETIZE(double);