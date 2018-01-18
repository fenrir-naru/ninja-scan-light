/**
 *
 *
 *
 */

%module NavigationModel

%{
#include <string>
#include <vector>

#include "navigation/MagneticField.h"
#include "navigation/WGS84.h"
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
%template(IGRF11) MagneticFieldGeneric2<type, IGRF11Generic>;
%template(IGRF12) MagneticFieldGeneric2<type, IGRF12Generic>;
%template(WMM2010) WMM2010Generic<type>;
%extend MagneticFieldModel {
#if defined(SWIGRUBY)
  %typemap(out) coef_t {
    VALUE arr = rb_ary_new2($1.coef_num);
    for(int i(0) ; i < $1.coef_num; i++){
      rb_ary_push(arr, DBL2NUM((double)($1.coef[i])));
    }
    $result = arr;
  }
#endif
}
%inline %{
struct MagneticFieldModel : public MagneticFieldGeneric<type>::model_t {
  typedef typename MagneticFieldGeneric<type>::model_t super_t;
  MagneticFieldModel(const super_t &m) : super_t(m) {}
  const char *name() const {return super_t::name;}
  const type year() const {return super_t::year;}
  const int dof() const {return super_t::dof;}
  struct coef_t {
    int coef_num;
    const type *coef;
  };
  const coef_t coef() const {
    coef_t res = {super_t::dof * (super_t::dof + 2), super_t::coef};
    return res;
  }
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
%extend MagneticFieldGeneric2 {
#if defined(SWIGRUBY)
  %typemap(out) std::vector<const typename MagneticFieldGeneric<FloatT>::model_t *> {
  	$result = rb_hash_new();
    for(typename std::vector<const typename MagneticFieldGeneric<FloatT>::model_t *>::const_iterator it($1.begin());
        it != $1.end();
        ++it){
      rb_hash_aset($result, ID2SYM(rb_intern((*it)->name)), 
          SWIG_NewPointerObj(SWIG_as_voidptr(new MagneticFieldModel(**it)), $descriptor(MagneticFieldModel *), 1));
    }
  }
#endif
  static std::vector<const typename MagneticFieldGeneric<FloatT>::model_t *> models() {
    std::vector<const typename MagneticFieldGeneric<FloatT>::model_t *> res;
    for(unsigned int i(0); 
        i < sizeof(Binder<FloatT>::models) / sizeof(Binder<FloatT>::models[0]); 
        ++i){
      res.push_back(Binder<FloatT>::models[i]);
    }
    return res;
  }
}

%include navigation/MagneticField.h
%include navigation/WGS84.h

CONCRETIZE(double);