/**
 * @file SWIG interface file for header files in param directory
 *
 */

%module SylphideMath

#define ENABLE_IOSTREAM 1

%{
#include <string>
#include <sstream>
#include <vector>
#include <exception>

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

%include std_common.i
%include std_string.i
//%include std_vector.i
%include exception.i
%include std_except.i

%ignore native_exception;
#if !defined(SWIGIMPORTED)
%exceptionclass native_exception;
%typemap(throws,noblock=1) native_exception {
  $1.regenerate();
  SWIG_fail;
}
%ignore native_exception;
%inline {
struct native_exception : public std::exception {
#if defined(SWIGRUBY)
  int state;
  native_exception(const int &state_) : std::exception(), state(state_) {}
  void regenerate() const {rb_jump_tag(state);}
#else
  void regenerate() const {}
#endif
};
}
#endif

%define MAKE_ACCESSOR(name, type)
%rename(%str(name ## =)) set_ ## name;
type set_ ## name (const type &v) {
  return (self->name() = v);
}
%rename(%str(name)) get_ ## name;
type get_ ## name () {
  return self->name();
}
%enddef
%define MAKE_TO_S(type)
%extend type{
  std::string __str__() const {
    std::stringstream s;
    s << (*self);
    return s.str();
  }
};
%enddef

#ifdef SWIGRUBY
%header {
static VALUE funcall_throw_if_error(VALUE (*func)(VALUE), VALUE arg) {
  int state;
  VALUE res = rb_protect(func, arg, &state);
  if(state != 0){throw native_exception(state);}
  return res;
}
static VALUE yield_throw_if_error(const int &argc, const VALUE *argv) {
  struct yield_t {
    const int &argc;
    const VALUE *argv;
    static VALUE run(VALUE v){
      yield_t *arg(reinterpret_cast<yield_t *>(v));
      return rb_yield_values2(arg->argc, arg->argv);
    }
  } arg = {argc, argv};
  return funcall_throw_if_error(yield_t::run, reinterpret_cast<VALUE>(&arg));
}
static std::string inspect_str(const VALUE &v){
  VALUE v_inspect(rb_inspect(v));
  return std::string(RSTRING_PTR(v_inspect), RSTRING_LEN(v_inspect));
}
}
#endif

%feature("autodoc", "1");

%ignore Complex::real;
%ignore Complex::imaginary;
%ignore Complex::check_infinity_t;
%ignore operator<<(std::ostream &, const Complex &);

template <class FloatT>
class Complex;

%copyctor Complex;

%fragment(SWIG_Traits_frag(ComplexGeneric), "header", fragment="StdTraits"){
// SWIG_Traits_frag(Complex) is invalid, which will be hidden by SWIG_Traits_frag(Complex<T>)
#ifdef SWIGRUBY
  namespace swig {
    template <class T> struct traits< Complex<T> > {
      typedef value_category category;
    };
    template <class T> struct traits_asval< Complex<T> > {
      typedef Complex<T> value_type;
      static int asval(VALUE obj, value_type *v) {
        if(RB_TYPE_P(obj, T_COMPLEX)){
%#if RUBY_API_VERSION_CODE < 20600
          static const ID id_r(rb_intern("real")), id_i(rb_intern("imag"));
          int res = swig::asval(rb_funcall(obj, id_r, 0), &(v->real()));
          if(!SWIG_IsOK(res)){return res;}
          return swig::asval(rb_funcall(obj, id_i, 0), &(v->imaginary()));
%#else
          int res = swig::asval(rb_complex_real(obj), &(v->real()));
          if(!SWIG_IsOK(res)){return res;}
          return swig::asval(rb_complex_imag(obj), &(v->imaginary()));
%#endif 
        }else{
          v->imaginary() = T(0);
          return swig::asval(obj, &(v->real()));
        }
      }
    };
    template <class T> struct traits_from< Complex<T> > {
      typedef Complex<T> value_type;
      static VALUE from(const value_type &v) {
        return rb_complex_new(swig::from(v.real()), swig::from(v.imaginary()));
      }
    };
    template <class T> struct traits_check< Complex<T>, value_category> {
      static bool check(VALUE obj) {
        if(RB_TYPE_P(obj, T_COMPLEX)){
%#if RUBY_API_VERSION_CODE < 20600
          static const ID id_r(rb_intern("real")), id_i(rb_intern("imag"));
          return swig::check<T>(rb_funcall(obj, id_r, 0))
              && swig::check<T>(rb_funcall(obj, id_i, 0));
%#else
          return swig::check<T>(rb_complex_real(obj))
              && swig::check<T>(rb_complex_imag(obj));
%#endif
        }else{
          return swig::check<T>(obj);
        }
      }
    };
  }
#endif
}

%extend Complex {
  %fragment(SWIG_Traits_frag(Complex<FloatT>), "header",
      fragment=SWIG_Traits_frag(FloatT),
      fragment=SWIG_Traits_frag(ComplexGeneric)){
    namespace swig {
      template <>
      inline swig_type_info *type_info<Complex<FloatT> >() {
        return $descriptor(Complex<FloatT> *);
      }
      template <>
      inline swig_type_info *type_info<Complex<FloatT> *>() {
        return $descriptor(Complex<FloatT> *);
      }
    }
  }
  %fragment(SWIG_Traits_frag(Complex<FloatT>));
  %typemap(typecheck,precedence=SWIG_TYPECHECK_POINTER) const Complex<FloatT> & {
    $1 = swig::check<$1_ltype >($input) || swig::check<$*1_ltype >($input);
  }
  %typemap(in) const Complex<FloatT> & (Complex<FloatT> temp) {
    if((!SWIG_IsOK(swig::asptr($input, &$1)))
        && (!SWIG_IsOK(swig::asval($input, ($1 = &temp))))){
      SWIG_exception(SWIG_TypeError, "in method '$symname', expecting type $*1_ltype");
    }
  }
  %typemap(out) Complex<FloatT> {
    $result = swig::from($1);
  }

  static Complex<FloatT> rectangular(const FloatT &r, const FloatT &i = FloatT(0)) noexcept {
    return Complex<FloatT>(r, i);
  }

  MAKE_ACCESSOR(real, FloatT);
  MAKE_ACCESSOR(imaginary, FloatT);

#if defined(SWIGRUBY)
  %alias power "**";
  %alias arg "angle,phase";
  %alias conjugate "conj";
  %alias operator/ "fdiv";
  %rename("finite?") isfinite;
  %rename("infinite?") isinf;
  %alias set_imaginary "imag=";
  %alias get_imaginary "imag";
  %alias abs "magnitude"
#endif
};

%include param/complex.h

MAKE_TO_S(Complex);

%define INSTANTIATE_COMPLEX(type, suffix)
%template(Complex ## suffix) Complex<type>;
#if defined(SWIGRUBY)
%fragment("init"{ComplexInit<type>}, "init") {
  { /* work around of %alias rectangular "rect"; %alias cannot be applied to singleton method */
    VALUE singleton = rb_singleton_class(
        ((swig_class *)$descriptor(Complex<type> *)->clientdata)->klass);
    rb_define_alias(singleton, "rect", "rectangular");
  }
}
%fragment("init"{ComplexInit<type>});
#endif
%enddef

INSTANTIATE_COMPLEX(double, D);

#undef INSTANTIATE_COMPLEX

#if defined(SWIGRUBY)
/* Work around of miss detection of negative value on Windows Ruby (devkit). 
 * This results from SWIG_AsVal(unsigned int) depends on SWIG_AsVal(unsigned long), 
 * and sizeof(long) == sizeof(int).
 */
%fragment("check_value"{unsigned int}, "header"){
  inline bool is_lt_zero_after_asval(const unsigned int &i){
    return ((sizeof(unsigned int) == sizeof(unsigned long)) && ((UINT_MAX >> 1) <= i));
  }
  void raise_if_lt_zero_after_asval(const unsigned int &i){
    if(is_lt_zero_after_asval(i)){
      SWIG_exception(SWIG_ValueError, "Expected positive value.");
    }
  } 
}
#endif

#define DO_NOT_INSTANTIATE_SCALAR_MATRIX
#define USE_MATRIX_VIEW_FILTER

#if defined(USE_MATRIX_VIEW_FILTER)
%{
template <class BaseView = MatrixViewBase<> >
struct MatrixViewFilter : public BaseView {
  typedef MatrixViewFilter<BaseView> self_t;

  struct {
    unsigned int row_offset, column_offset;
    unsigned int rows, columns;
    bool transposed;
    bool conjugated;
  } prop;

  MatrixViewFilter() : BaseView() {
    prop.row_offset = prop.column_offset = 0;
    prop.rows = prop.columns = 0; // to be configured
    prop.transposed = false;
    prop.conjugated = false;
  }
  MatrixViewFilter(const self_t &view)
      : BaseView((const BaseView &)view), prop(view.prop) {
  }
  
  void transpose() {
    prop.transposed = !prop.transposed;
  }
  void conjugate() {
    prop.conjugated = !prop.conjugated;
  }
  void partial(
      const unsigned int &new_rows, const unsigned int &new_columns,
      const unsigned int &row_offset, const unsigned int &column_offset) {
    if(prop.transposed){
      prop.row_offset += column_offset;
      prop.column_offset += row_offset;
      prop.rows = new_columns;
      prop.columns = new_rows;
    }else{
      prop.row_offset += row_offset;
      prop.column_offset += column_offset;
      prop.rows = new_rows;
      prop.columns = new_columns;
    }
  }
  
  template <class T, class Array2D_Type>
  struct mat_t : public Matrix_Frozen<T, Array2D_Type, self_t> {
    typedef Matrix_Frozen<T, Array2D_Type, self_t> super_t;
    mat_t(const Matrix_Frozen<T, Array2D_Type, BaseView> &orig) : super_t(orig) {
      super_t::view.prop.rows = orig.rows();
      super_t::view.prop.columns = orig.columns();
    }
    mat_t(const Matrix_Frozen<T, Array2D_Type, self_t> &orig) : super_t(orig) {}
    self_t &view() {return super_t::view;}
  };
  
  template <class T, class Array2D_Type, class ViewType>
  static Matrix_Frozen<T, Array2D_Type, self_t> transpose(
      const Matrix_Frozen<T, Array2D_Type, ViewType> &orig){
    mat_t<T, Array2D_Type> res(orig);
    res.view().transpose();
    return res;
  }
  template <class T, class Array2D_Type, class ViewType>
  static Matrix_Frozen<T, Array2D_Type, self_t> conjugate(
      const Matrix_Frozen<T, Array2D_Type, ViewType> &orig){
    return mat_t<T, Array2D_Type>(orig);
  }
  template <class T, class Array2D_Type, class ViewType>
  static Matrix_Frozen<Complex<T>, Array2D_Type, self_t> conjugate(
      const Matrix_Frozen<Complex<T>, Array2D_Type, ViewType> &orig){
    mat_t<Complex<T>, Array2D_Type> res(orig);
    res.view().conjugate();
    return res;
  }
  template <class T, class Array2D_Type, class ViewType>
  static Matrix_Frozen<T, Array2D_Type, self_t> partial(
      const Matrix_Frozen<T, Array2D_Type, ViewType> &orig,
      const unsigned int &new_rows, const unsigned int &new_columns,
      const unsigned int &row_offset, const unsigned int &column_offset) {
    if(new_rows + row_offset > orig.rows()){
      throw std::out_of_range("Row size exceeding");
    }else if(new_columns + column_offset > orig.columns()){
      throw std::out_of_range("Column size exceeding");
    }
    mat_t<T, Array2D_Type> res(orig);
    res.view().partial(new_rows, new_columns, row_offset, column_offset);
    return res;
  }

  template<class CharT, class Traits>
  friend std::basic_ostream<CharT, Traits> &operator<<(
      std::basic_ostream<CharT, Traits> &out, const self_t &view){
    out 
        << (view.prop.transposed 
          ? (view.prop.conjugated ? "[*] " : "[T] ") 
          : (view.prop.conjugated ? "[~] " : ""))
        << "[Size](" << view.prop.rows << "," << view.prop.columns << ") ";
    if((view.prop.row_offset > 0) || (view.prop.column_offset > 0)){
      out << "[Offset](" << view.prop.row_offset << "," << view.prop.column_offset << ") ";
    }
    return out << (const BaseView &)view;
  }

  inline const unsigned int rows(
      const unsigned int &_rows, const unsigned int &_columns) const noexcept {
    return prop.transposed ? prop.columns : prop.rows;
  }
  inline const unsigned int columns(
      const unsigned int &_rows, const unsigned int &_columns) const noexcept {
    return prop.transposed ? prop.rows : prop.columns;
  }

  template <class T>
  struct conjugate_t {
    static T run(const T &v, const bool &conjugated = false){return v;}
  };
  template <class T>
  struct conjugate_t<Complex<T> > {
    static Complex<T> run(const Complex<T> &v, const bool &conjugated = false){
      return conjugated ? v.conjugate() : v;
    }
  };

  template <class T, class Array2D_Type>
  inline T operator()(
      Array2D_Type &storage, const unsigned int &i, const unsigned int &j) const {
    return conjugate_t<T>::run(
        prop.transposed
          ? BaseView::template operator()<T, Array2D_Type>(
            storage, (j + prop.column_offset), (i + prop.row_offset))
          : BaseView::template operator()<T, Array2D_Type>(
            storage, (i + prop.row_offset), (j + prop.column_offset)),
        prop.conjugated);
  }
};
%}
#endif /* USE_MATRIX_VIEW_FILTER */

template <class T, class Array2D_Type, class ViewType = MatrixViewBase<> >
class Matrix_Frozen {
  protected:
    Matrix_Frozen();
  public:
    const unsigned int rows();
    const unsigned int columns();
    
    template <class T2, class Array2D_Type2, class ViewType2>
    bool operator==(const Matrix_Frozen<T2, Array2D_Type2, ViewType2> &matrix) const noexcept;
    // bool operator!= // automatically defined

    bool isSquare() const noexcept;
    bool isDiagonal() const noexcept;
    bool isLowerTriangular() const noexcept;
    bool isUpperTriangular() const noexcept;
    bool isSymmetric() const noexcept;
    bool isHermitian() const noexcept;
    bool isSkewSymmetric() const noexcept;
    bool isNormal() const noexcept;
    bool isOrthogonal() const noexcept;
    bool isUnitary() const noexcept;
    
    T trace() const;
    T sum() const noexcept;
    
    // bool isLU() const noexcept
    
    T determinant() const;
    unsigned int rank() const;
    T cofactor(
        const unsigned int &row, const unsigned int &column) const;
};

template <class T, class Array2D_Type, class ViewType = MatrixViewBase<> >
class Matrix : public Matrix_Frozen<T, Array2D_Type, ViewType> {
  public:
#if !defined(DO_NOT_INSTANTIATE_SCALAR_MATRIX)
    typedef Matrix_Frozen<T, Array2D_ScaledUnit<T> > scalar_matrix_t;
    static scalar_matrix_t getScalar(const unsigned int &size, const T &scalar);
    static scalar_matrix_t getI(const unsigned int &size);
#endif

    typedef Matrix<T, Array2D_Type, ViewType> self_t;
};

%inline {
typedef MatrixViewBase<> MatViewBase;
#if defined(USE_MATRIX_VIEW_FILTER)
typedef MatrixViewFilter<MatrixViewBase<> > MatView_f;
#else
typedef MatrixViewTranspose<MatrixViewBase<> > MatView_t;
typedef MatrixViewSizeVariable<MatrixViewOffset<MatrixViewBase<> > > MatView_p;
typedef MatrixViewTranspose<MatrixViewSizeVariable<MatrixViewOffset<MatrixViewBase<> > > > MatView_pt;
#endif
}

%define INSTANTIATE_MATRIX_FUNC(func_orig, func_new)
#if !defined(DO_NOT_INSTANTIATE_SCALAR_MATRIX)
%template(func_new) func_orig<T, Array2D_ScaledUnit<T>, MatViewBase>;
#if defined(USE_MATRIX_VIEW_FILTER)
%template(func_new) func_orig<T, Array2D_ScaledUnit<T>, MatView_f>;
#else
%template(func_new) func_orig<T, Array2D_ScaledUnit<T>, MatView_p>;
%template(func_new) func_orig<T, Array2D_ScaledUnit<T>, MatView_pt>;
#endif
#endif
%template(func_new) func_orig<T, Array2D_Dense<T>, MatViewBase>;
#if defined(USE_MATRIX_VIEW_FILTER)
%template(func_new) func_orig<T, Array2D_Dense<T>, MatView_f>;
#else
%template(func_new) func_orig<T, Array2D_Dense<T>, MatView_t>;
%template(func_new) func_orig<T, Array2D_Dense<T>, MatView_p>;
%template(func_new) func_orig<T, Array2D_Dense<T>, MatView_pt>;
#endif
%enddef

%{
struct MatrixUtil {
  struct each_break_t {
    unsigned int r, c;
    each_break_t(const unsigned int &row, const unsigned int &column)
        : r(row), c(column) {}
  };
  enum each_which_t {
    EACH_ALL,
    EACH_DIAGONAL,
    EACH_OFF_DIAGONAL,
    EACH_LOWER,
    EACH_UPPER,
    EACH_STRICT_LOWER,
    EACH_STRICT_UPPER,
    EACH_UNKNOWN,
  };
  template <class T, 
      class Array2D_Type, class ViewType,
      class Array2D_Type2 = Array2D_Dense<T>, class ViewType2 = MatrixViewBase<> >
  static void each(
      const Matrix_Frozen<T, Array2D_Type, ViewType> &src,
      void (*each_func)(
        const T &src_elm, T *dst_elm,
        const unsigned int &i, const unsigned int &j),
      const each_which_t &each_which = EACH_ALL,
      Matrix<T, Array2D_Type2, ViewType2> *dst = NULL) {
    unsigned int i_max(src.rows()), j_max(src.columns());
    switch(each_which){
      case EACH_DIAGONAL:
        for(unsigned int k(0), k_max(i_max >= j_max ? j_max : i_max); k < k_max; ++k){
          (*each_func)(src(k, k), (dst ? &((*dst)(k, k)) : NULL), k, k);
        }
        break;
      case EACH_OFF_DIAGONAL:
        for(unsigned int i(0); i < i_max; ++i){
          for(unsigned int j(0); j < j_max; ++j){
            if(i != j){
              (*each_func)(src(i, j), (dst ? &((*dst)(i, j)) : NULL), i, j);
            }
          }
        }
        break;
      case EACH_LOWER:
        for(unsigned int i(0); i < i_max; ++i){
          for(unsigned int j(0); j <= i; ++j){
            (*each_func)(src(i, j), (dst ? &((*dst)(i, j)) : NULL), i, j);
          }
        }
        break;
      case EACH_UPPER:
        for(unsigned int i(0); i < i_max; ++i){
          for(unsigned int j(i); j < j_max; ++j){
            (*each_func)(src(i, j), (dst ? &((*dst)(i, j)) : NULL), i, j);
          }
        }
        break;
      case EACH_STRICT_LOWER:
        for(unsigned int i(1); i < i_max; ++i){
          for(unsigned int j(0); j < i; ++j){
            (*each_func)(src(i, j), (dst ? &((*dst)(i, j)) : NULL), i, j);
          }
        }
        break;
      case EACH_STRICT_UPPER:
        for(unsigned int i(0); i < i_max; ++i){
          for(unsigned int j(i + 1); j < j_max; ++j){
            (*each_func)(src(i, j), (dst ? &((*dst)(i, j)) : NULL), i, j);
          }
        }
        break;
      case EACH_ALL:
      default:
        for(unsigned int i(0); i < i_max; ++i){
          for(unsigned int j(0); j < j_max; ++j){
            (*each_func)(src(i, j), (dst ? &((*dst)(i, j)) : NULL), i, j);
          }
        }
        break;
    }
  }
#if defined(SWIGRUBY)
  static each_which_t sym2each_which(const VALUE &value){
    if(!RB_TYPE_P(value, T_SYMBOL)){return EACH_UNKNOWN;}
    static const struct {
      VALUE sym;
      each_which_t which;
    } cmp[] = {
      {ID2SYM(rb_intern("all")),          EACH_ALL},
      {ID2SYM(rb_intern("diagonal")),     EACH_DIAGONAL},
      {ID2SYM(rb_intern("off_diagonal")), EACH_OFF_DIAGONAL},
      {ID2SYM(rb_intern("lower")),        EACH_LOWER},
      {ID2SYM(rb_intern("upper")),        EACH_UPPER},
      {ID2SYM(rb_intern("strict_lower")), EACH_STRICT_LOWER},
      {ID2SYM(rb_intern("strict_upper")), EACH_STRICT_UPPER},
    };
    unsigned int i(0);
    while(value != cmp[i].sym){
      if(++i >= (sizeof(cmp) / sizeof(cmp[0]))){break;}
    }
    if(i >= (sizeof(cmp) / sizeof(cmp[0]))){return EACH_UNKNOWN;}
    return cmp[i].which;
  }
#endif

  template <class T, class Array2D_Type, class ViewType>
  static bool replace(
      Matrix<T, Array2D_Type, ViewType> &dst,
      const void *src = NULL){
    unsigned int r(dst.rows()), c(dst.columns()), len(r * c);
    bool replaced(true);
#if defined(SWIGRUBY)
    struct bracket_read_t {
      static VALUE run(VALUE v) {
        VALUE *values = reinterpret_cast<VALUE *>(v);
        static const ID id_func(rb_intern("[]"));
        return rb_funcall2(values[0], id_func, 2, &values[1]);
      }
      static bool is_accessible(const VALUE &v) {
        static const ID id_func(rb_intern("[]"));
        return rb_respond_to(v, id_func) != 0;
      }
      static VALUE read(
          const VALUE &v, const unsigned int &row = 0, const unsigned int &column = 0) {
        VALUE values[3] = {v, UINT2NUM(row), UINT2NUM(column)};
        return funcall_throw_if_error(run, reinterpret_cast<VALUE>(values));
      }
    };
    const VALUE *value(static_cast<const VALUE *>(src));
    unsigned int i(0), j(0), i_elm(0);
    VALUE v_elm;
    if(value && RB_TYPE_P(*value, T_ARRAY)){
      if(RB_TYPE_P(RARRAY_AREF(*value, 0), T_ARRAY)){ // [[r0c0, r0c1, ...], ...]
        if((unsigned int)RARRAY_LEN(*value) < r){
          throw std::invalid_argument("Length is too short");
        }
        VALUE value_r;
        for(; i_elm < len; i_elm++){
          if(j == 0){
            value_r = RARRAY_AREF(*value, i);
            if(!RB_TYPE_P(value_r, T_ARRAY)){
              throw std::invalid_argument("double array [[...], ...] is required");
            }else if((unsigned int)RARRAY_LEN(value_r) < c){
              throw std::invalid_argument("Length is too short");
            }
          }
          v_elm = RARRAY_AREF(value_r, j);
          if(!SWIG_IsOK(swig::asval(v_elm, &dst(i, j)))){break;}
          if(++j >= c){j = 0; ++i;}
        }
      }else{ // [r0c0, r0c1, ...]
        if((unsigned int)RARRAY_LEN(*value) < len){
          throw std::invalid_argument("Length is too short");
        }
        for(; i_elm < len; i_elm++){
          v_elm = RARRAY_AREF(*value, i_elm);
          if(!SWIG_IsOK(swig::asval(v_elm, &dst(i, j)))){break;}
          if(++j >= c){j = 0; ++i;}
        }
      }
    }else if(value && bracket_read_t::is_accessible(*value)){
      for(; i_elm < len; i_elm++){
        v_elm = bracket_read_t::read(*value, i, j);
        if(!SWIG_IsOK(swig::asval(v_elm, &dst(i, j)))){break;}
        if(++j >= c){j = 0; ++i;}
      }
    }else if(rb_block_given_p()){
      for(; i_elm < len; i_elm++){
        VALUE args[2] = {UINT2NUM(i), UINT2NUM(j)};
        v_elm = yield_throw_if_error(2, args);
        if(!SWIG_IsOK(swig::asval(v_elm, &dst(i, j)))){break;}
        if(++j >= c){j = 0; ++i;}
      }
    }else{
      replaced = false;
    }
    if(replaced && (i_elm < len)){
      std::stringstream s;
      s << "Unexpected input [" << i << "," << j << "]: ";
      throw std::invalid_argument(s.str().append(inspect_str(v_elm)));
    }
#endif
    return replaced;
  }
  template <class T, class Array2D_Type, class ViewType>
  static bool replace(
      Matrix<T, Array2D_Type, ViewType> &dst,
      const T *src){
    if(!src){return false;}
    for(unsigned int i(0), r(dst.rows()); i < r; ++i){
      for(unsigned int j(0), c(dst.columns()); j < c; ++j){
        dst(i, j) = *(src++);
      }
    }
    return true;
  }
};
%}

%extend Matrix_Frozen {
  %catches(std::logic_error) trace;
  %catches(std::out_of_range) partial;
  %catches(std::out_of_range) rowVector;
  %catches(std::out_of_range) columnVector;
  %catches(std::logic_error, std::runtime_error) determinant;
  %catches(std::logic_error) rank;
  %catches(std::logic_error, std::runtime_error) cofactor;

  T __getitem__(const unsigned int &row, const unsigned int &column) const {
    return ($self)->operator()(row, column);
  }
  
  Matrix<T, Array2D_Dense<T> > copy() const {
    return $self->operator Matrix<T, Array2D_Dense<T> >();
  }

  Matrix<T, Array2D_Dense<T> > circular(
      const unsigned int &row_offset, const unsigned int &column_offset,
      const unsigned int &new_rows, const unsigned int &new_columns) const noexcept {
    return (Matrix<T, Array2D_Dense<T> >)(($self)->circular(
        row_offset, column_offset, new_rows, new_columns));
  }
  Matrix<T, Array2D_Dense<T> > circular(
      const unsigned int &row_offset, const unsigned int &column_offset) const noexcept {
    return (Matrix<T, Array2D_Dense<T> >)(($self)->circular(row_offset, column_offset));
  }

  INSTANTIATE_MATRIX_FUNC(operator==, __eq__);

  template <class T2, class Array2D_Type2, class ViewType2>
  bool isDifferentSize(const Matrix_Frozen<T2, Array2D_Type2, ViewType2> &matrix) const {
    return ($self)->isDifferentSize(matrix);
  }
  INSTANTIATE_MATRIX_FUNC(isDifferentSize, isDifferentSize);
  
  Matrix<T, Array2D_Dense<T> > operator*(const T &scalar) const noexcept {
    return (Matrix<T, Array2D_Dense<T> >)(($self)->operator*(scalar));
  }
  Matrix<T, Array2D_Dense<T> > operator/(const T &scalar) const noexcept {
    return (Matrix<T, Array2D_Dense<T> >)(($self)->operator/(scalar));
  }
  Matrix<T, Array2D_Dense<T> > operator-() const noexcept {
    return (Matrix<T, Array2D_Dense<T> >)(($self)->operator-());
  }
  
  %catches(std::invalid_argument) operator+;
  template <class T2, class Array2D_Type2, class ViewType2>
  Matrix<T, Array2D_Dense<T> > operator+(
      const Matrix_Frozen<T2, Array2D_Type2, ViewType2> &matrix) const {
    return (Matrix<T, Array2D_Dense<T> >)(($self)->operator+(matrix));
  }
  INSTANTIATE_MATRIX_FUNC(operator+, __add__);
  Matrix<T, Array2D_Dense<T> > operator+(const T &scalar) const {
    return (Matrix<T, Array2D_Dense<T> >)(($self)->operator+(scalar));
  }
  
  %catches(std::invalid_argument) operator-;
  template <class T2, class Array2D_Type2, class ViewType2>
  Matrix<T, Array2D_Dense<T> > operator-(
      const Matrix_Frozen<T2, Array2D_Type2, ViewType2> &matrix) const {
    return (Matrix<T, Array2D_Dense<T> >)(($self)->operator-(matrix));
  }
  INSTANTIATE_MATRIX_FUNC(operator-, __sub__);
  Matrix<T, Array2D_Dense<T> > operator-(const T &scalar) const {
    return (Matrix<T, Array2D_Dense<T> >)(($self)->operator-(scalar));
  }
  
#ifdef SWIGRUBY
  %alias entrywise_product "hadamard_product";
#endif
  %catches(std::invalid_argument) entrywise_product;
  template <class T2, class Array2D_Type2, class ViewType2>
  Matrix<T, Array2D_Dense<T> > entrywise_product(
      const Matrix_Frozen<T2, Array2D_Type2, ViewType2> &matrix) const {
    return (Matrix<T, Array2D_Dense<T> >)(($self)->entrywise_product(matrix));
  }
  INSTANTIATE_MATRIX_FUNC(entrywise_product, entrywise_product);
  
  template <class T2, class Array2D_Type2, class ViewType2>
  Matrix<T, Array2D_Dense<T> > operator*(
      const Matrix_Frozen<T2, Array2D_Type2, ViewType2> &matrix)
      const throw(std::invalid_argument) {
    return (Matrix<T, Array2D_Dense<T> >)(($self)->operator*(matrix));
  }
  INSTANTIATE_MATRIX_FUNC(operator*, __mul__);
  
  // TODO __pow__ for **
  // TODO __pos__ for +@

  Matrix<T, Array2D_Dense<T> > first_minor(
      const unsigned int &row,
      const unsigned int &column) const noexcept {
    return (Matrix<T, Array2D_Dense<T> >)(($self)->matrix_for_minor(row, column));
  }
  Matrix<T, Array2D_Dense<T> > adjugate() const {
    return (Matrix<T, Array2D_Dense<T> >)(($self)->adjugate());
  }

  %typemap(in,numinputs=0)
      Matrix<T, Array2D_Dense<T> > &output_L (Matrix<T, Array2D_Dense<T> > temp),
      Matrix<T, Array2D_Dense<T> > &output_U (Matrix<T, Array2D_Dense<T> > temp),
      Matrix<T, Array2D_Dense<T> > &output_P (Matrix<T, Array2D_Dense<T> > temp),
      Matrix<T, Array2D_Dense<T> > &output_D (Matrix<T, Array2D_Dense<T> > temp),
      Matrix<T, Array2D_Dense<T> > &output_Q (Matrix<T, Array2D_Dense<T> > temp),
      Matrix<T, Array2D_Dense<T> > &output_R (Matrix<T, Array2D_Dense<T> > temp) %{
    $1 = &temp;
  %}
  %typemap(argout)
      Matrix<T, Array2D_Dense<T> > &output_L,
      Matrix<T, Array2D_Dense<T> > &output_U,
      Matrix<T, Array2D_Dense<T> > &output_P,
      Matrix<T, Array2D_Dense<T> > &output_D,
      Matrix<T, Array2D_Dense<T> > &output_Q,
      Matrix<T, Array2D_Dense<T> > &output_R {
    %append_output(SWIG_NewPointerObj((new $*1_ltype(*$1)), $1_descriptor, SWIG_POINTER_OWN));
  }
  %catches(std::logic_error, std::runtime_error) lup;
  void lup(
      Matrix<T, Array2D_Dense<T> > &output_L, 
      Matrix<T, Array2D_Dense<T> > &output_U, 
      Matrix<T, Array2D_Dense<T> > &output_P) const {
    struct buf_t {
      unsigned int pivot_len, pivot_num;
      unsigned int *pivot;
      buf_t(const unsigned int &size) 
          : pivot_len(size), pivot(new unsigned int[size]) {}
      ~buf_t(){
        delete [] pivot;
      }
      Matrix<T, Array2D_Dense<T> > P() const {
        Matrix<T, Array2D_Dense<T> > res(pivot_len, pivot_len);
        for(unsigned int i(0); i < pivot_len; ++i){
          res(i, pivot[i]) = 1;
        }
        return res;
      }
    } buf($self->rows());
    Matrix<T, Array2D_Dense<T> > LU($self->decomposeLUP(buf.pivot_num, buf.pivot));
    output_L = LU.partial($self->rows(), $self->columns()).copy();
    output_U = LU.partial($self->rows(), $self->columns(), 0, $self->rows()).copy();
    output_P = buf.P();
  }
  %catches(std::logic_error) ud;
  void ud(
      Matrix<T, Array2D_Dense<T> > &output_U, 
      Matrix<T, Array2D_Dense<T> > &output_D) const {
    Matrix<T, Array2D_Dense<T> > UD($self->decomposeUD());
    output_U = UD.partial($self->rows(), $self->columns()).copy();
    output_D = UD.partial($self->rows(), $self->columns(), 0, $self->rows()).copy();
  }
  void qr(
      Matrix<T, Array2D_Dense<T> > &output_Q, 
      Matrix<T, Array2D_Dense<T> > &output_R) const {
    Matrix<T, Array2D_Dense<T> > QR($self->decomposeQR());
    output_Q = QR.partial($self->rows(), $self->rows()).copy();
    output_R = QR.partial($self->rows(), $self->columns(), 0, $self->rows()).copy();
  }

  %catches(std::logic_error, std::runtime_error) inverse;
  Matrix<T, Array2D_Dense<T> > inverse() const {
    return (Matrix<T, Array2D_Dense<T> >)(($self)->inverse());
  }
  template <class T2, class Array2D_Type2, class ViewType2>
  Matrix<T, Array2D_Dense<T> > operator/(
      const Matrix_Frozen<T2, Array2D_Type2, ViewType2> &matrix)
      const throw(std::logic_error, std::runtime_error) {
    return (Matrix<T, Array2D_Dense<T> >)(($self)->operator/(matrix));
  }
  INSTANTIATE_MATRIX_FUNC(operator/, __div__);
  
  std::string debug() const {
    std::stringstream s;
    s << $self->inspect();
    return s.str();
  }
  
  /* The followings are better to be implemented in Ruby
   * combine, hstack, vstack (due to their arguments are variable)
   */

#ifdef SWIGRUBY
  %rename("square?") isSquare;
  %rename("diagonal?") isDiagonal;
  %rename("lower_triangular?") isLowerTriangular;
  %rename("upper_triangular?") isUpperTriangular;
  %rename("symmetric?") isSymmetric;
  %rename("hermitian?") isHermitian;
  %rename("skew_symmetric?") isSkewSymmetric;
  %alias isSkewSymmetric "antisymmetric?"
  %rename("normal?") isNormal;
  %rename("orthogonal?") isOrthogonal;
  %rename("unitary?") isUnitary;
  %rename("different_size?") isDifferentSize;
  %alias __getitem__ "element,component";
  // %alias __eq__ "eql?"; // Intentionally commented out because eql? is more strict than ==
  %alias rows "row_size,row_count";
  %alias columns "column_size,column_count";
  %alias trace "tr";
  %alias determinant "det";
  %alias inverse "inv";
  %alias transpose "t";
  %alias conjugate "conj";
  %alias lup "lup_decomposition";
  %alias ud "ud_decomposition";
  %alias qr "qr_decomposition";
  
  %fragment(SWIG_From_frag(Matrix_Frozen_Helper<T>), "header", 
      fragment=SWIG_Traits_frag(T)){
    template <bool with_index = false, bool assign = false>
    static inline VALUE matrix_yield_internal(
        const T &src, T *dst, const unsigned int &i, const unsigned int &j){
      SWIG_Object v;
      if(with_index){
        VALUE values[] = {swig::from(src), UINT2NUM(i), UINT2NUM(j)};
        v = yield_throw_if_error(3, values);
      }else{
        VALUE values[] = {swig::from(src)};
        v = yield_throw_if_error(1, values);
      }
      if(assign && !SWIG_IsOK(swig::asval(v, dst))){
        std::stringstream s;
        s << "Unknown input (T expected) [" << i << "," << j << "]: ";
        throw std::invalid_argument(s.str().append(inspect_str(v)));
      }
      return v;
    }
    static void matrix_yield(
        const T &src, T *dst, const unsigned int &i, const unsigned int &j){
      matrix_yield_internal<false, false>(src, dst, i, j);
    }
    static void matrix_yield_with_index(
        const T &src, T *dst, const unsigned int &i, const unsigned int &j){
      matrix_yield_internal<true, false>(src, dst, i, j);
    }
    static void matrix_yield_get(
        const T &src, T *dst, const unsigned int &i, const unsigned int &j){
      matrix_yield_internal<false, true>(src, dst, i, j);
    }
    static void matrix_yield_get_with_index(
        const T &src, T *dst, const unsigned int &i, const unsigned int &j){
      matrix_yield_internal<true, true>(src, dst, i, j);
    }
    
    static void matrix_yield_check(
        const T &src, T *dst, const unsigned int &i, const unsigned int &j){
      VALUE res(matrix_yield_internal<false, false>(src, dst, i, j));
      if(RTEST(res)){throw typename MatrixUtil::each_break_t(i, j);}
    }
    static void (*matrix_each(const T *))
        (const T &, T *, const unsigned int &, const unsigned int &) {
      ID id_thisf(rb_frame_this_func()), id_callee(rb_frame_callee());
      static const ID 
          id_map(rb_intern("map")), id_mapb(rb_intern("map!")),
          id_eachwi(rb_intern("each_with_index"));
      if((id_thisf == id_map) || (id_thisf == id_mapb)){
        static const ID with_index[] = {
            rb_intern("map_with_index"), rb_intern("map_with_index!"), 
            rb_intern("collect_with_index"), rb_intern("collect_with_index!")};
        for(std::size_t i(0); i < sizeof(with_index) / sizeof(with_index[0]); ++i){
          if(id_callee == with_index[i]){
            return matrix_yield_get_with_index;
          }
        }
        return matrix_yield_get;
      }else if(id_callee == id_eachwi){
        return matrix_yield_with_index;
      }else{
        return matrix_yield;
      }     
    }
  }
  %typemap(in,numinputs=0, fragment=SWIG_From_frag(Matrix_Frozen_Helper<T>)) 
      void (*each_func)(const T &src, T *dst, const unsigned int &i, const unsigned int &j) {
    if(!rb_block_given_p()){
      return rb_enumeratorize(self, ID2SYM(rb_frame_callee()), argc, argv);
    }
    $1 = matrix_each((const T *)0);
  }
  %typemap(typecheck,precedence=SWIG_TYPECHECK_POINTER) const typename MatrixUtil::each_which_t each_which {
    $1 = (MatrixUtil::sym2each_which($input) != MatrixUtil::EACH_UNKNOWN);
  }
  %typemap(in) const typename MatrixUtil::each_which_t each_which {
    $1 = MatrixUtil::sym2each_which($input);
    if($1 == MatrixUtil::EACH_UNKNOWN){
      SWIG_exception(SWIG_ValueError,
          std::string("Unknown enumerate direction: ").append(inspect_str($1)).c_str());
    }
  }
  %catches(native_exception) each;
  const Matrix_Frozen<T, Array2D_Type, ViewType> &each(
      void (*each_func)(
        const T &src, T *dst,
        const unsigned int &i, const unsigned int &j), 
      const typename MatrixUtil::each_which_t each_which = MatrixUtil::EACH_ALL) const {
    MatrixUtil::each(*$self, each_func, each_which);
    return *$self;
  }
  %alias each "each_with_index";
  
  %catches(native_exception, std::invalid_argument) map;
  Matrix<T, Array2D_Dense<T> > map(
      void (*each_func)(
        const T &src, T *dst,
        const unsigned int &i, const unsigned int &j), 
      const typename MatrixUtil::each_which_t each_which = MatrixUtil::EACH_ALL) const {
    Matrix<T, Array2D_Dense<T> > res($self->operator Matrix<T, Array2D_Dense<T> >());
    MatrixUtil::each(*$self, each_func, each_which, &res);
    return res;
  }
  %alias map "collect,map_with_index,collect_with_index";
  
  %catches(native_exception) index;
  VALUE index(
      const typename MatrixUtil::each_which_t each_which = MatrixUtil::EACH_ALL) const {
    try{
      MatrixUtil::each(*$self, matrix_yield_check, each_which);
      return Qnil;
    }catch(const typename MatrixUtil::each_break_t &each_break){
      return rb_ary_new_from_args(2, UINT2NUM(each_break.r), UINT2NUM(each_break.c));
    }
  }
  %typemap(check,noblock=1) VALUE idx_selector {
    if(MatrixUtil::sym2each_which($1) == MatrixUtil::EACH_UNKNOWN){
      SWIG_exception(SWIG_ValueError,
          std::string("Unknown enumerate direction: ").append(inspect_str($1)).c_str());
    }
  }
  VALUE index(VALUE value, VALUE idx_selector = Qnil) const {
    return rb_block_call(
        rb_current_receiver(), rb_frame_callee(),
        (RTEST(idx_selector) ? 1 : 0), &idx_selector,
        (rb_block_call_func_t)rb_equal, value);
  }
  %alias index "find_index";
  
  SWIG_Object to_a() const {
    unsigned int i_max($self->rows()), j_max($self->columns());
    SWIG_Object res = rb_ary_new2(i_max);
    for(unsigned int i(0); i < i_max; ++i){
      SWIG_Object row = rb_ary_new2(j_max);
      for(unsigned int j(0); j < j_max; ++j){
        rb_ary_store(row, j, swig::from((*($self))(i, j)));
      }
      rb_ary_store(res, i, row);
    }
    return res;
  }
#endif
};

#if defined(SWIGRUBY)
%mixin Matrix_Frozen "Enumerable";
#endif

MAKE_TO_S(Matrix_Frozen)

%extend Matrix {
  %typemap(typecheck,precedence=SWIG_TYPECHECK_VOIDPTR) const void *replacer {
#if defined(SWIGRUBY)
    $1 = rb_block_given_p() ? 0 : 1;
#else
    $1 = 0;
#endif
  }
  %typemap(in) const void *replacer {
    $1 = &$input;
  }
  %fragment(SWIG_Traits_frag(T));

  Matrix(const unsigned int &rows, const unsigned int &columns, 
      const void *replacer = NULL)
      throw(native_exception, std::invalid_argument) {
    Matrix<T, Array2D_Type, ViewType> res(rows, columns);
    MatrixUtil::replace(res, replacer);
    return new Matrix<T, Array2D_Type, ViewType>(res);
  }
  Matrix(const unsigned int &rows, const unsigned int &columns,
      const T *serialized){
    Matrix<T, Array2D_Type, ViewType> res(rows, columns);
    MatrixUtil::replace(res, serialized);
    return new Matrix<T, Array2D_Type, ViewType>(res);
  }
#if defined(SWIGRUBY)
  %fragment(SWIG_AsVal_frag(unsigned int));
  %fragment("check_value"{unsigned int});
  Matrix(const void *replacer) throw(native_exception, std::invalid_argument) {
    const SWIG_Object *value(static_cast<const SWIG_Object *>(replacer));
    static const ID id_r(rb_intern("row_size")), id_c(rb_intern("column_size"));
    if(value && RB_TYPE_P(*value, T_ARRAY) && RB_TYPE_P(RARRAY_AREF(*value, 0), T_ARRAY)){
      Matrix<T, Array2D_Type, ViewType> res(
          (unsigned int)RARRAY_LEN(*value),
          (unsigned int)RARRAY_LEN(RARRAY_AREF(*value, 0)));
      MatrixUtil::replace(res, replacer);
      return new Matrix<T, Array2D_Type, ViewType>(res);
    }else if(value && rb_respond_to(*value, id_r) && rb_respond_to(*value, id_c)){
      unsigned int r, c; 
      VALUE v_r(rb_funcall(*value, id_r, 0, 0)), v_c(rb_funcall(*value, id_c, 0, 0));
      if(!SWIG_IsOK(SWIG_AsVal(unsigned int)(v_r, &r)) || is_lt_zero_after_asval(r)
          || !SWIG_IsOK(SWIG_AsVal(unsigned int)(v_c, &c)) || is_lt_zero_after_asval(c)){
        throw std::invalid_argument(
            std::string("Invalid length [")
              .append(inspect_str(v_r)).append(", ")
              .append(inspect_str(v_c)).append("]"));
      }
      Matrix<T, Array2D_Type, ViewType> res(r, c);
      MatrixUtil::replace(res, replacer);
      return new Matrix<T, Array2D_Type, ViewType>(res);
    }else{
      throw std::invalid_argument("double array [[...], ...] or Matrix is required");
    }
  }
#endif

  /*
   * Returning (*this) requires special care;
   * "self_t &func(){return (*this);}" in a C++ file may generate
   * a new wrapped object deleted by GC in the target language
   * unless the care.
   * 
   * Work around 1)
   *   %typemap(in, numinputs=0) self_t *self_p "";
   *   %typemap(argout) self_t *self_p "$result = self;";
   *   void func(self_t *self_p){...}
   *
   * Work around 2) (useful without overwrite of the original source, but may overfit)
   *   %typemap(out) self_t & "$result = self;"
   *   self_t &func(){...; return *$self;}
   */
  %typemap(in, numinputs=0) self_t *self_p "";
  %typemap(argout) self_t *self_p "$result = self;";

  T &__setitem__(const unsigned int &row, const unsigned int &column, const T &value) {
    return (($self)->operator()(row, column) = value);
  }
#if defined(DO_NOT_INSTANTIATE_SCALAR_MATRIX)
  static Matrix<T, Array2D_Dense<T> > getScalar(const unsigned int &size, const T &scalar) {
    return Matrix<T, Array2D_Dense<T> >(
        Matrix_Frozen<T, Array2D_Type, ViewType>::getScalar(size, scalar));
  }
  static Matrix<T, Array2D_Dense<T> > getI(const unsigned int &size) {
    return Matrix<T, Array2D_Dense<T> >(
        Matrix_Frozen<T, Array2D_Type, ViewType>::getI(size));
  }
#endif
  %rename("scalar") getScalar;
  %rename("I") getI;

  %catches(std::out_of_range) swap_rows;
  void swap_rows(
      self_t *self_p,
      const unsigned int &r1, const unsigned int &r2){
    $self->swapRows(r1, r2);
  }
  %catches(std::out_of_range) swap_columns;
  void swap_columns(
      self_t *self_p,
      const unsigned int &c1, const unsigned int &c2){
    $self->swapColumns(c1, c2);
  }

  template <class T2, class Array2D_Type2, class ViewType2>
  void replace(
      self_t *self_p,
      const Matrix_Frozen<T2, Array2D_Type2, ViewType2> &matrix)
      throw(std::invalid_argument) {
    $self->replace(matrix);
  }
  INSTANTIATE_MATRIX_FUNC(replace, replace);

  void replace(self_t *self_p, const void *replacer = NULL)
      throw(native_exception, std::invalid_argument, std::runtime_error){
    if(!MatrixUtil::replace(*$self, replacer)){
      throw std::runtime_error("Unsupported replacement");
    }
  }

  void replace(self_t *self_p, const T *serialized) throw(std::runtime_error) {
    if(!MatrixUtil::replace(*$self, serialized)){
      throw std::runtime_error("Unsupported replacement");
    }
  }
  
#ifdef SWIGRUBY
  %bang swap_rows;
  %bang swap_columns;
  %rename("replace!") replace;
  
  %catches(native_exception, std::invalid_argument) map_bang;
  void map_bang(
      self_t *self_p,
      void (*each_func)(
        const T &src, T *dst,
        const unsigned int &i, const unsigned int &j), 
      const typename MatrixUtil::each_which_t each_which = MatrixUtil::EACH_ALL){
    MatrixUtil::each(*$self, each_func, each_which, $self);
  }
  %rename("map!") map_bang;
  %alias map_bang "collect!,map_with_index!,collect_with_index!";
#endif
};

%define INSTANTIATE_MATRIX_TRANSPOSE(type, storage, view_from, view_to)
%extend Matrix_Frozen<type, storage, view_from> {
  Matrix_Frozen<type, storage, view_to> transpose() const {
#if defined(USE_MATRIX_VIEW_FILTER)
    return MatView_f::transpose(*$self);
#else
    return $self->transpose();
#endif
  }
};
%enddef

%define INSTANTIATE_MATRIX_PARTIAL(type, storage, view_from, view_to)
%extend Matrix_Frozen<type, storage, view_from> {
  Matrix_Frozen<type, storage, view_to> partial(
      const unsigned int &new_rows, const unsigned int &new_columns,
      const unsigned int &row_offset, const unsigned int &column_offset) const {
#if defined(USE_MATRIX_VIEW_FILTER)
    return MatView_f::partial(*$self, new_rows, new_columns, row_offset, column_offset);
#else
    return $self->partial(new_rows, new_columns, row_offset, column_offset);
#endif
  }
  Matrix_Frozen<type, storage, view_to> partial(
      const unsigned int &new_rows, const unsigned int &new_columns) const {
#if defined(USE_MATRIX_VIEW_FILTER)
    return MatView_f::partial(*$self, new_rows, new_columns, 0, 0);
#else
    return $self->partial(new_rows, new_columns, 0, 0);
#endif
  }
  Matrix_Frozen<type, storage, view_to> row_vector(const unsigned int &row) const {
#if defined(USE_MATRIX_VIEW_FILTER)
    return MatView_f::partial(*$self, 1, $self->columns(), row, 0);
#else
    return $self->rowVector(row);
#endif
  }
  Matrix_Frozen<type, storage, view_to> column_vector(const unsigned int &column) const {
#if defined(USE_MATRIX_VIEW_FILTER)
    return MatView_f::partial(*$self, $self->rows(), 1, 0, column);
#else
    return $self->columnVector(column);
#endif
  }
};
/* Ruby #row, #column, #row_vectors, #column_vectors are not intentionally implemented 
 * because a vector is treated as a (1*n) or (n*1) matrix in C++.
 */
%enddef

%define INSTANTIATE_MATRIX_EIGEN2(type, ctype, storage, view)
%extend Matrix_Frozen<type, storage, view> {
  %typemap(in,numinputs=0)
      Matrix<ctype, Array2D_Dense<ctype > > &output_D 
        (Matrix<ctype, Array2D_Dense<ctype > > temp),
      Matrix<ctype, Array2D_Dense<ctype > > &output_V 
        (Matrix<ctype, Array2D_Dense<ctype > > temp) %{
    $1 = &temp;
  %}
  %typemap(argout)
      Matrix<ctype, Array2D_Dense<ctype > > &output_D,
      Matrix<ctype, Array2D_Dense<ctype > > &output_V {
    %append_output(SWIG_NewPointerObj((new $*1_ltype(*$1)), $1_descriptor, SWIG_POINTER_OWN));
  }
  %catches(std::logic_error, std::runtime_error) eigen;
  void eigen(
      Matrix<ctype, Array2D_Dense<ctype > > &output_V, 
      Matrix<ctype, Array2D_Dense<ctype > > &output_D) const {
    typedef Matrix<ctype, Array2D_Dense<ctype > > cmat_t;
    cmat_t VD($self->eigen());
    output_V = VD.partial($self->rows(), $self->rows()).copy();
    cmat_t D($self->rows(), $self->rows());
    for(unsigned int i(0); i < $self->rows(); ++i){
      D(i, i) = VD(i, $self->rows());
    }
    output_D = D;
  }
};
%enddef
%define INSTANTIATE_MATRIX_EIGEN(type, ctype)
#if !defined(DO_NOT_INSTANTIATE_SCALAR_MATRIX)
INSTANTIATE_MATRIX_EIGEN2(type, ctype, Array2D_ScaledUnit<type >, MatViewBase);
#if defined(USE_MATRIX_VIEW_FILTER)
INSTANTIATE_MATRIX_EIGEN2(type, ctype, Array2D_ScaledUnit<type >, MatView_f);
#else
INSTANTIATE_MATRIX_EIGEN2(type, ctype, Array2D_ScaledUnit<type >, MatView_p);
INSTANTIATE_MATRIX_EIGEN2(type, ctype, Array2D_ScaledUnit<type >, MatView_pt);
#endif
#endif
INSTANTIATE_MATRIX_EIGEN2(type, ctype, Array2D_Dense<type >, MatViewBase);
#if defined(USE_MATRIX_VIEW_FILTER)
INSTANTIATE_MATRIX_EIGEN2(type, ctype, Array2D_Dense<type >, MatView_f);
#else
INSTANTIATE_MATRIX_EIGEN2(type, ctype, Array2D_Dense<type >, MatView_p);
INSTANTIATE_MATRIX_EIGEN2(type, ctype, Array2D_Dense<type >, MatView_t);
INSTANTIATE_MATRIX_EIGEN2(type, ctype, Array2D_Dense<type >, MatView_pt);
#endif
%enddef

#if defined(USE_MATRIX_VIEW_FILTER)
%extend Matrix_Frozen {
  Matrix_Frozen<T, Array2D_Type, MatView_f> conjugate() const {
    return MatView_f::conjugate(*$self);
  }
  Matrix_Frozen<T, Array2D_Type, MatView_f> adjoint() const {
    return MatView_f::conjugate(MatView_f::transpose(*$self));
  }
};
#else
%extend Matrix_Frozen {
  Matrix<T, Array2D_Dense<T> > conjugate() const {
    return (Matrix<T, Array2D_Dense<T> >)(($self)->conjugate());
  }
  Matrix<T, Array2D_Dense<T> > adjoint() const {
    return (Matrix<T, Array2D_Dense<T> >)(($self)->adjoint());
  }
};
#endif

%define INSTANTIATE_MATRIX(type, suffix)
%typemap(check, fragment="check_value"{unsigned int})
    const unsigned int & "raise_if_lt_zero_after_asval(*$1);"
#if !defined(DO_NOT_INSTANTIATE_SCALAR_MATRIX)
%extend Matrix_Frozen<type, Array2D_ScaledUnit<type >, MatViewBase> {
  const Matrix_Frozen<type, Array2D_ScaledUnit<type >, MatViewBase> &transpose() const {
    return *($self);
  }
  Matrix_Frozen<type, Array2D_ScaledUnit<type >, MatViewBase> inverse() const {
    return $self->inverse();
  }
};
#if defined(USE_MATRIX_VIEW_FILTER)
INSTANTIATE_MATRIX_TRANSPOSE(type, Array2D_ScaledUnit<type >, MatView_f, MatView_f);
INSTANTIATE_MATRIX_PARTIAL(type, Array2D_ScaledUnit<type >, MatViewBase, MatView_f);
INSTANTIATE_MATRIX_PARTIAL(type, Array2D_ScaledUnit<type >, MatView_f, MatView_f);

%template(Matrix_Scalar ## suffix) Matrix_Frozen<type, Array2D_ScaledUnit<type >, MatViewBase>;
%template(Matrix_Scalar ## suffix ## _f) Matrix_Frozen<type, Array2D_ScaledUnit<type >, MatView_f>;
#else
INSTANTIATE_MATRIX_TRANSPOSE(type, Array2D_ScaledUnit<type >, MatView_p, MatView_pt);
INSTANTIATE_MATRIX_TRANSPOSE(type, Array2D_ScaledUnit<type >, MatView_pt, MatView_p);
INSTANTIATE_MATRIX_PARTIAL(type, Array2D_ScaledUnit<type >, MatViewBase, MatView_p);
INSTANTIATE_MATRIX_PARTIAL(type, Array2D_ScaledUnit<type >, MatView_p, MatView_p);
INSTANTIATE_MATRIX_PARTIAL(type, Array2D_ScaledUnit<type >, MatView_pt, MatView_pt);

%template(Matrix_Scalar ## suffix) Matrix_Frozen<type, Array2D_ScaledUnit<type >, MatViewBase>;
%template(Matrix_Scalar ## suffix ## _p) Matrix_Frozen<type, Array2D_ScaledUnit<type >, MatView_p>;
%template(Matrix_Scalar ## suffix ## _pt) Matrix_Frozen<type, Array2D_ScaledUnit<type >, MatView_pt>;
#endif
#endif

#if defined(USE_MATRIX_VIEW_FILTER)
INSTANTIATE_MATRIX_TRANSPOSE(type, Array2D_Dense<type >, MatViewBase, MatView_f);
INSTANTIATE_MATRIX_TRANSPOSE(type, Array2D_Dense<type >, MatView_f, MatView_f);
INSTANTIATE_MATRIX_PARTIAL(type, Array2D_Dense<type >, MatViewBase, MatView_f);
INSTANTIATE_MATRIX_PARTIAL(type, Array2D_Dense<type >, MatView_f, MatView_f);

%template(Matrix_Frozen ## suffix) Matrix_Frozen<type, Array2D_Dense<type >, MatViewBase>;
%template(Matrix_Frozen ## suffix ## _f) Matrix_Frozen<type, Array2D_Dense<type >, MatView_f>;
#else
INSTANTIATE_MATRIX_TRANSPOSE(type, Array2D_Dense<type >, MatViewBase, MatView_t);
INSTANTIATE_MATRIX_TRANSPOSE(type, Array2D_Dense<type >, MatView_t, MatViewBase);
INSTANTIATE_MATRIX_TRANSPOSE(type, Array2D_Dense<type >, MatView_p, MatView_pt);
INSTANTIATE_MATRIX_TRANSPOSE(type, Array2D_Dense<type >, MatView_pt, MatView_p);
INSTANTIATE_MATRIX_PARTIAL(type, Array2D_Dense<type >, MatViewBase, MatView_p);
INSTANTIATE_MATRIX_PARTIAL(type, Array2D_Dense<type >, MatView_p, MatView_p);
INSTANTIATE_MATRIX_PARTIAL(type, Array2D_Dense<type >, MatView_t, MatView_pt);
INSTANTIATE_MATRIX_PARTIAL(type, Array2D_Dense<type >, MatView_pt, MatView_pt);

%template(Matrix_Frozen ## suffix) Matrix_Frozen<type, Array2D_Dense<type >, MatViewBase>;
%template(Matrix_Frozen ## suffix ## _t) Matrix_Frozen<type, Array2D_Dense<type >, MatView_t>;
%template(Matrix_Frozen ## suffix ## _p) Matrix_Frozen<type, Array2D_Dense<type >, MatView_p>;
%template(Matrix_Frozen ## suffix ## _pt) Matrix_Frozen<type, Array2D_Dense<type >, MatView_pt>;
#endif

%extend Matrix<type, Array2D_Dense<type > > {
#if defined(SWIGRUBY)
  %bang resize;
#endif
  %typemap(in, fragment="check_value"{unsigned int}) 
      unsigned int *r_p (unsigned int temp), unsigned int *c_p (unsigned int temp) {
    if(SWIG_IsOK(SWIG_AsVal(unsigned int)($input, &temp))){
#if defined(SWIGRUBY)
      raise_if_lt_zero_after_asval(temp);
#endif
      $1 = &temp;
    }
#if defined(SWIGRUBY)
    else if(NIL_P($input)){$1 = NULL;}
#endif
    else{SWIG_exception(SWIG_TypeError, "$*1_ltype is expected");}
  }
  Matrix<type, Array2D_Dense<type > > &resize(
      const unsigned int *r_p, const unsigned int *c_p){
    unsigned int r(r_p ? *r_p : $self->rows()), c(c_p ? *c_p : $self->columns());
    Matrix<type, Array2D_Dense<type > > mat_new(r, c);
    unsigned int r_min(r), c_min(c);
    if(r_min > $self->rows()){r_min = $self->rows();}
    if(c_min > $self->columns()){c_min = $self->columns();}
    mat_new.partial(r_min, c_min).replace($self->partial(r_min, c_min), false);
    return (*($self) = mat_new);
  }
};

%template(Matrix ## suffix) Matrix<type, Array2D_Dense<type > >;
#if defined(SWIGRUBY)
%fragment("init"{Matrix<type, Array2D_Dense<type > >}, "init") {
  { /* work around of %alias I "unit,identity"; %alias cannot be applied to singleton method */
    VALUE singleton = rb_singleton_class(
        ((swig_class *)$descriptor(Matrix<type, Array2D_Dense<type > > *)->clientdata)->klass);
    rb_define_alias(singleton, "identity", "I");
    rb_define_alias(singleton, "unit", "I");
  }
}
%fragment("init"{Matrix<type, Array2D_Dense<type > >});
#endif
%typemap(check) const unsigned int &;
%enddef

INSTANTIATE_MATRIX(double, D);
INSTANTIATE_MATRIX_EIGEN(double, Complex<double>);
INSTANTIATE_MATRIX(Complex<double>, ComplexD);
INSTANTIATE_MATRIX_EIGEN(Complex<double>, Complex<double>);

%rename("tolerance=") set_tolerance;
%rename("tolerance") get_tolerance;
%inline %{
double set_tolerance(const double &width){
  MatrixValue<double>::zero = width;
  MatrixValue<Complex<double> >::zero = width;
  return width;
}
double get_tolerance(){
  return set_tolerance(MatrixValue<double>::zero.width);
}
%}

#undef INSTANTIATE_MATRIX_FUNC
#undef INSTANTIATE_MATRIX_TRANSPOSE
#undef INSTANTIATE_MATRIX_PARTIAL
#undef INSTANTIATE_MATRIX_EIGEN2
#undef INSTANTIATE_MATRIX_EIGEN
#undef INSTANTIATE_MATRIX

#if defined(DO_NOT_INSTANTIATE_SCALAR_MATRIX)
#undef DO_NOT_INSTANTIATE_SCALAR_MATRIX
#endif

#undef MAKE_ACCESSOR
#undef MAKE_TO_S
