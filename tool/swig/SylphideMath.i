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

//%include std_common.i
%include std_string.i
//%include std_vector.i
%include exception.i

%exception {
  try {
    $action
  } catch (const std::exception& e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  }
}

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

%header {
template <class T>
SWIG_Object to_value(swig_type_info *info, const T &v){
  return SWIG_NewPointerObj((void *)&v, info, 0);
}
template <class T>
bool from_value(const SWIG_Object &obj, swig_type_info *info, T &v){
  T *ptr;
  int res(info ? SWIG_ConvertPtr(obj, (void **)&ptr, info, 0) : SWIG_ERROR);
  if(SWIG_IsOK(res)){
    if(ptr){v = *ptr;} // if nil, then keep current v
    if(SWIG_IsNewObj(res)){delete ptr;}
    return true;
  }
  return false;
}
template <>
SWIG_Object to_value(swig_type_info *info, const double &v);
template <>
bool from_value(const SWIG_Object &obj, swig_type_info *info, double &v);
}

%wrapper {
template <>
SWIG_Object to_value(swig_type_info *info, const double &v){
  return SWIG_From(double)(v);
}
template <>
bool from_value(const SWIG_Object &obj, swig_type_info *info, double &v){
  return SWIG_IsOK(SWIG_AsVal(double)(obj, &v)) ? true : false;
}
}

#ifdef SWIGRUBY
%header {
template <class T>
SWIG_Object to_value(swig_type_info *info, const Complex<T> &v){
  return rb_complex_new(
      to_value(info, v.real()), 
      to_value(info, v.imaginary()));
}
template <class T>
bool from_value(const SWIG_Object &obj, swig_type_info *info, Complex<T> &v){
  if(RB_TYPE_P(obj, T_COMPLEX)){
    return from_value(rb_complex_real(obj), NULL, v.real())
        && from_value(rb_complex_imag(obj), NULL, v.imaginary());
  }else{
    v.imaginary() = T(0);
    return from_value(obj, NULL, v.real());
  }
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

%extend Complex {
  %typemap(typecheck,precedence=SWIG_TYPECHECK_POINTER) const Complex<FloatT> & {
    void *vptr = 0;
    Complex<FloatT> temp;
    $1 = SWIG_CheckState(SWIG_ConvertPtr($input, &vptr, $1_descriptor, 0));
    $1 = $1 || from_value($input, $1_descriptor, temp);
  }
  %typemap(in) const Complex<FloatT> & (Complex<FloatT> temp) {
    do{
      if(SWIG_IsOK(SWIG_ConvertPtr($input, (void **)&$1, $1_descriptor, 0))){break;}
      if(from_value($input, $1_descriptor, temp)){
        $1 = &temp;
        break;
      }
      SWIG_exception(SWIG_TypeError, "in method '$symname', expecting type $*1_ltype");
    }while(false);
  }
  %typemap(out) Complex<FloatT> {
    $result = to_value($&1_descriptor, $1);
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
  template <class T, class Array2D_Type, class ViewType>
  static Matrix_Frozen<T, Array2D_Type, self_t> partial(
      const Matrix_Frozen<T, Array2D_Type, ViewType> &orig,
      const unsigned int &new_rows, const unsigned int &new_columns) {
    return partial(orig, new_rows, new_columns, 0, 0);
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
    bool isSymmetric() const noexcept;
    
    T trace(const bool &do_check = true) const;
    T sum() const noexcept;
    
    // bool isLU() const noexcept
    
    T determinant(const bool &do_check = true) const;
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
    self_t &swapRows(const unsigned int &row1, const unsigned int &row2);
    self_t &swapColumns(const unsigned int &column1, const unsigned int &column2);
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
  enum each_which_t {
    EACH_ALL,
    EACH_DIAGONAL,
    EACH_OFF_DIAGONAL,
    EACH_LOWER,
    EACH_UPPER,
    EACH_STRICT_LOWER,
    EACH_STRICT_UPPER,
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
      Matrix<T, Array2D_Type2, ViewType2> *dst = NULL){
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
  static const each_which_t &sym2each_which(const VALUE &value){
    if(!RB_TYPE_P(value, T_SYMBOL)){
      std::runtime_error("Symbol is required");
    }
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
    if(i >= (sizeof(cmp) / sizeof(cmp[0]))){
      std::runtime_error("Unknown enumerate direction");
    }
    return cmp[i].which;
  }
#endif

  template <class T, class Array2D_Type, class ViewType>
  static bool replace(
      Matrix<T, Array2D_Type, ViewType> &dst,
      bool (*conv)(const void *src, T &dst),
      const void *src = NULL){
    unsigned int r(dst.rows()), c(dst.columns()), len(r * c);
    bool replaced(true);
#if defined(SWIGRUBY)
    struct bracket_read_t {
      static VALUE run(VALUE v) {
        VALUE *values = reinterpret_cast<VALUE *>(v);
        static const ID id_b(rb_intern("[]"));
        return rb_funcall2(values[0], id_b, 2, &values[1]);
      }
      static bool is_accessible(
          const VALUE &v, const unsigned int &row = 0, const unsigned int &column = 0) {
        int state;
        VALUE values[3] = {v, UINT2NUM(row), UINT2NUM(column)};
        rb_protect(run, reinterpret_cast<VALUE>(values), &state);
        return state == 0;
      }
      static VALUE read(
          const VALUE &v, const unsigned int &row = 0, const unsigned int &column = 0) {
        int state;
        VALUE values[3] = {v, UINT2NUM(row), UINT2NUM(column)};
        VALUE res = rb_protect(run, reinterpret_cast<VALUE>(values), &state);
        return (state == 0) ? res : Qnil;
      }
    };
    const VALUE *value(static_cast<const VALUE *>(src));
    unsigned int i(0), j(0), i_elm(0);
    VALUE v_elm;
    if(value && RB_TYPE_P(*value, T_ARRAY)){
      if(RB_TYPE_P(RARRAY_AREF(*value, 0), T_ARRAY)){ // [[r0c0, r0c1, ...], ...]
        if((unsigned int)RARRAY_LEN(*value) < r){
          throw std::runtime_error("Length is too short");
        }
        VALUE value_r;
        for(; i_elm < len; i_elm++){
          if(j == 0){
            value_r = RARRAY_AREF(*value, i);
            if(!RB_TYPE_P(value_r, T_ARRAY)){
              throw std::runtime_error("double array [[...], ...] is required");
            }else if((unsigned int)RARRAY_LEN(value_r) < c){
              throw std::runtime_error("Length is too short");
            }
          }
          v_elm = RARRAY_AREF(value_r, j);
          if(!conv(&v_elm, dst(i, j))){break;}
          if(++j >= c){j = 0; ++i;}
        }
      }else{ // [r0c0, r0c1, ...]
        if((unsigned int)RARRAY_LEN(*value) < len){
          throw std::runtime_error("Length is too short");
        }
        for(; i_elm < len; i_elm++){
          v_elm = RARRAY_AREF(*value, i_elm);
          if(!conv(&v_elm, dst(i, j))){break;}
          if(++j >= c){j = 0; ++i;}
        }
      }
    }else if(value && bracket_read_t::is_accessible(*value)){
      for(; i_elm < len; i_elm++){
        v_elm = bracket_read_t::read(*value, i, j);
        if(!conv(&v_elm, dst(i, j))){break;}
        if(++j >= c){j = 0; ++i;}
      }
    }else if(rb_block_given_p()){
      for(; i_elm < len; i_elm++){
        v_elm = rb_yield_values(2, UINT2NUM(i), UINT2NUM(j));
        if(!conv(&v_elm, dst(i, j))){break;}
        if(++j >= c){j = 0; ++i;}
      }
    }else{
      replaced = false;
    }
    if(replaced && (i_elm < len)){
      VALUE v_inspect(rb_inspect(v_elm));
      std::stringstream s;
      s << "Unexpected input [" << i << "," << j << "]: ";
      throw std::runtime_error(
          s.str().append(RSTRING_PTR(v_inspect), RSTRING_LEN(v_inspect)));
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
  
  template <class T2, class Array2D_Type2, class ViewType2>
  Matrix<T, Array2D_Dense<T> > operator+(
      const Matrix_Frozen<T2, Array2D_Type2, ViewType2> &matrix) const {
    return (Matrix<T, Array2D_Dense<T> >)(($self)->operator+(matrix));
  }
  INSTANTIATE_MATRIX_FUNC(operator+, __add__);
  Matrix<T, Array2D_Dense<T> > operator+(const T &scalar) const {
    return (Matrix<T, Array2D_Dense<T> >)(($self)->operator+(scalar));
  }
  
  template <class T2, class Array2D_Type2, class ViewType2>
  Matrix<T, Array2D_Dense<T> > operator-(
      const Matrix_Frozen<T2, Array2D_Type2, ViewType2> &matrix) const {
    return (Matrix<T, Array2D_Dense<T> >)(($self)->operator-(matrix));
  }
  INSTANTIATE_MATRIX_FUNC(operator-, __sub__);
  Matrix<T, Array2D_Dense<T> > operator-(const T &scalar) const {
    return (Matrix<T, Array2D_Dense<T> >)(($self)->operator-(scalar));
  }
  
  template <class T2, class Array2D_Type2, class ViewType2>
  Matrix<T, Array2D_Dense<T> > operator*(
      const Matrix_Frozen<T2, Array2D_Type2, ViewType2> &matrix) const {
    return (Matrix<T, Array2D_Dense<T> >)(($self)->operator*(matrix));
  }
  INSTANTIATE_MATRIX_FUNC(operator*, __mul__);

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

  Matrix<T, Array2D_Dense<T> > inverse() const {
    return (Matrix<T, Array2D_Dense<T> >)(($self)->inverse());
  }
  template <class T2, class Array2D_Type2, class ViewType2>
  Matrix<T, Array2D_Dense<T> > operator/(
      const Matrix_Frozen<T2, Array2D_Type2, ViewType2> &matrix) const {
    return (Matrix<T, Array2D_Dense<T> >)(($self)->operator/(matrix));
  }
  INSTANTIATE_MATRIX_FUNC(operator/, __div__);
  
  std::string debug() const {
    std::stringstream s;
    s << $self->inspect();
    return s.str();
  }

#ifdef SWIGRUBY
  %rename("square?") isSquare;
  %rename("diagonal?") isDiagonal;
  %rename("symmetric?") isSymmetric;
  %rename("different_size?") isDifferentSize;
  %alias trace "tr";
  %alias determinant "det";
  %alias inverse "inv";
  %alias transpose "t";
  %alias lup "lup_decomposition";
  %alias ud "ud_decomposition";
  %alias qr "qr_decomposition";
  
  %fragment(SWIG_From_frag(Matrix_Frozen_Helper<T>), "header"){
    static void matrix_yield(const T &v, T *, const unsigned int &i, const unsigned int &j){
      rb_yield_values(1, to_value($descriptor(const T &), v));
    }
    static void matrix_yield_with_index(const T &v, T *, const unsigned int &i, const unsigned int &j){
      rb_yield_values(3, to_value($descriptor(const T &), v), UINT2NUM(i), UINT2NUM(j));
    }
    static void matrix_assign(const SWIG_Object &v, T *dst, const unsigned int &i, const unsigned int &j){
      if(!from_value(v, $descriptor(T &), *dst)){
        VALUE v_inspect(rb_inspect(v));
        std::stringstream s;
        s << "Unknown input (T expected) [" << i << "," << j << "]: ";
        throw std::runtime_error(
            s.str().append(RSTRING_PTR(v_inspect), RSTRING_LEN(v_inspect)));
      }
    }
    static void matrix_yield_get(const T &src, T *dst, const unsigned int &i, const unsigned int &j){
      matrix_assign(rb_yield_values(1, to_value($descriptor(const T &), src)), dst, i, j);
    }
    static void matrix_yield_get_with_index(const T &src, T *dst, const unsigned int &i, const unsigned int &j){
      matrix_assign(rb_yield_values(3, to_value($descriptor(const T &), src), UINT2NUM(i), UINT2NUM(j)), dst, i, j);
    }
    static void (*matrix_each(const T *))
        (const T &, T *, const unsigned int &, const unsigned int &) {
      ID id_thisf(rb_frame_this_func()), id_callee(rb_frame_callee());
      if((id_thisf == rb_intern("map")) || (id_thisf == rb_intern("map!"))){
        ID with_index[] = {
            rb_intern("map_with_index"), rb_intern("map_with_index!"), 
            rb_intern("collect_with_index"), rb_intern("collect_with_index!")};
        for(int i(0); i < sizeof(with_index) / sizeof(with_index[0]); ++i){
          if(id_callee == with_index[i]){
            return matrix_yield_get_with_index;
          }
        }
        return matrix_yield_get;
      }else if(id_callee == rb_intern("each_with_index")){
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
  %typemap(typecheck) const typename MatrixUtil::each_which_t &each_which {
    $1 = RB_TYPE_P($input, T_SYMBOL);
  }
  %typemap(in) const typename MatrixUtil::each_which_t &each_which {
    try{
      $1 = &const_cast<typename MatrixUtil::each_which_t &>(MatrixUtil::sym2each_which($input));
    }catch(std::runtime_error &e){
      SWIG_exception(SWIG_TypeError, e.what());
    }
  }
  const Matrix_Frozen<T, Array2D_Type, ViewType> &each(
      void (*each_func)(
        const T &src, T *dst,
        const unsigned int &i, const unsigned int &j), 
      const typename MatrixUtil::each_which_t &each_which = MatrixUtil::EACH_ALL) const {
    MatrixUtil::each(*$self, each_func, each_which);
    return *$self;
  }
  %alias each "each_with_index";
  
  Matrix<T, Array2D_Dense<T> > map(
      void (*each_func)(
        const T &src, T *dst,
        const unsigned int &i, const unsigned int &j), 
      const typename MatrixUtil::each_which_t &each_which = MatrixUtil::EACH_ALL) const {
    Matrix<T, Array2D_Dense<T> > res($self->operator Matrix<T, Array2D_Dense<T> >());
    MatrixUtil::each(*$self, each_func, each_which, &res);
    return res;
  }
  %alias map "collect,map_with_index,collect_with_index";
  
  SWIG_Object to_a() const {
    unsigned int i_max($self->rows()), j_max($self->columns());
    SWIG_Object res = rb_ary_new2(i_max);
    for(unsigned int i(0); i < i_max; ++i){
      SWIG_Object row = rb_ary_new2(j_max);
      for(unsigned int j(0); j < j_max; ++j){
        rb_ary_store(row, j, to_value($descriptor(const T &), (*($self))(i, j)));
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
  %fragment(SWIG_From_frag(MatrixHelper<T>), "header"){
    static bool from_value(const void *src, T &dst){
      return from_value(*static_cast<const SWIG_Object *>(src), $descriptor(T &), dst);
    }
  }
  %typemap(in, numinputs=0, fragment=SWIG_From_frag(MatrixHelper<T>)) bool (*conv)(const void *src, T &dst) {
    $1 = from_value;
  }

  Matrix(const unsigned int &rows, const unsigned int &columns, 
      bool (*conv)(const void *src, T &dst), const void *replacer = NULL){
    Matrix<T, Array2D_Type, ViewType> res(rows, columns);
    MatrixUtil::replace(res, conv, replacer);
    return new Matrix<T, Array2D_Type, ViewType>(res);
  }
  Matrix(const unsigned int &rows, const unsigned int &columns,
      const T *serialized){
    Matrix<T, Array2D_Type, ViewType> res(rows, columns);
    MatrixUtil::replace(res, serialized);
    return new Matrix<T, Array2D_Type, ViewType>(res);
  }
#if defined(SWIGRUBY)
  Matrix(bool (*conv)(const void *src, T &dst), const void *replacer){
    const SWIG_Object *value(static_cast<const SWIG_Object *>(replacer));
    static const ID id_r(rb_intern("row_size")), id_c(rb_intern("column_size"));
    if(value && RB_TYPE_P(*value, T_ARRAY) && RB_TYPE_P(RARRAY_AREF(*value, 0), T_ARRAY)){
      Matrix<T, Array2D_Type, ViewType> res(
          (unsigned int)RARRAY_LEN(*value),
          (unsigned int)RARRAY_LEN(RARRAY_AREF(*value, 0)));
      MatrixUtil::replace(res, conv, replacer);
      return new Matrix<T, Array2D_Type, ViewType>(res);
    }else if(value && rb_respond_to(*value, id_r) && rb_respond_to(*value, id_c)){
      Matrix<T, Array2D_Type, ViewType> res(
          NUM2UINT(rb_funcall(*value, id_r, 0, 0)), 
          NUM2UINT(rb_funcall(*value, id_c, 0, 0)));
      MatrixUtil::replace(res, conv, replacer);
      return new Matrix<T, Array2D_Type, ViewType>(res);
    }else{
      throw std::runtime_error("double array [[...], ...] or Matrix is required");
    }
  }
#endif

  %typemap(out) self_t & "$result = self;"

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
  %rename("swap_rows") swapRows;
  %rename("swap_columns") swapColumns;

  template <class T2, class Array2D_Type2, class ViewType2>
  self_t &replace(const Matrix_Frozen<T2, Array2D_Type2, ViewType2> &matrix){
    return $self->replace(matrix);
  }
  INSTANTIATE_MATRIX_FUNC(replace, replace);

  self_t &replace(bool (*conv)(const void *src, T &dst), const void *replacer = NULL){
    if(!MatrixUtil::replace(*$self, conv, replacer)){
      throw std::runtime_error("Unsupported replacement");
    }
    return *$self;
  }

  self_t &replace(const T *serialized){
    if(!MatrixUtil::replace(*$self, serialized)){
      throw std::runtime_error("Unsupported replacement");
    }
    return *$self;
  }
  
#ifdef SWIGRUBY
  %bang swapRows(const unsigned int &, const unsigned int &);
  %bang swapColumns(const unsigned int &, const unsigned int &);
  %rename("replace!") replace;
  
  self_t &map_bang(
      void (*each_func)(
        const T &src, T *dst,
        const unsigned int &i, const unsigned int &j), 
      const typename MatrixUtil::each_which_t &each_which = MatrixUtil::EACH_ALL){
    MatrixUtil::each(*$self, each_func, each_which, $self);
    return *$self;
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
  void eigen(
      Matrix<ctype, Array2D_Dense<ctype > > &output_V, 
      Matrix<ctype, Array2D_Dense<ctype > > &output_D) const {
    typedef typename Matrix_Frozen<type, storage, view >::complex_t::m_t cmat_t;
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
%enddef

INSTANTIATE_MATRIX(double, D);
INSTANTIATE_MATRIX_EIGEN(double, Complex<double>);
INSTANTIATE_MATRIX(Complex<double>, ComplexD);
INSTANTIATE_MATRIX_EIGEN(Complex<double>, Complex<double>);

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
