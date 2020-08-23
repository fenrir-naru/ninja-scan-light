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
  std::string __str__() const {
    std::stringstream s;
    s << (*self);
    return s.str();
  }
};
%enddef


#ifdef SWIGRUBY
%{
template <class T>
VALUE to_value(swig_type_info *info, const T &v){
  return SWIG_NewPointerObj((void *)&v, info, 0);
}
template <>
VALUE to_value(swig_type_info *info, const double &v){
  return DBL2NUM(v);
}
template <class T>
void from_value(VALUE obj, swig_type_info *info, T &v){
  void *ptr;
  int res(SWIG_ConvertPtr(obj, &ptr, info, 1));
  if(SWIG_IsOK(res)){
    v = *(T *)ptr;
    if(SWIG_IsNewObj(res)){delete ptr;}
  }
}
template <>
void from_value(VALUE obj, swig_type_info *info, double &v){
  switch(TYPE(obj)){
    case T_FIXNUM:
      v = NUM2INT(obj);
      break;
    case T_BIGNUM:
    case T_FLOAT:
    case T_RATIONAL:
      v = NUM2DBL(obj);
      break;
  }
}
%}
#endif

%feature("autodoc", "1");

%ignore Complex::real;
%ignore Complex::imaginary;
%ignore operator<<(std::ostream &, const Complex &);

%include param/complex.h

MAKE_TO_S(Complex);

%extend Complex {
#ifdef SWIGRUBY
  %typemap(typecheck,precedence=SWIG_TYPECHECK_POINTER) const FloatT real_imag[2] {
    $1 = RB_TYPE_P($input, T_COMPLEX);
  }
  %typemap(in) const FloatT real_imag[2] (FloatT temp[2]) {
    from_value(rb_complex_real($input), $*1_descriptor, temp[0]);
    from_value(rb_complex_imag($input), $*1_descriptor, temp[1]);
    $1 = temp;
  }
  Complex(const FloatT real_imag[2]) noexcept {
    return new Complex<FloatT>(real_imag[0], real_imag[1]);
  }
#endif
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

%define INSTANTIATE_COMPLEX(type, suffix)
%template(Complex ## suffix) Complex<type>;
%enddef

INSTANTIATE_COMPLEX(double, D);

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
    
    // bool isLU() const noexcept
    
    T determinant(const bool &do_check = true) const;
};

template <class T, class Array2D_Type, class ViewType = MatrixViewBase<> >
class Matrix : public Matrix_Frozen<T, Array2D_Type, ViewType> {
  public:
    typedef Matrix_Frozen<T, Array2D_ScaledUnit<T> > scalar_matrix_t;
    static scalar_matrix_t getScalar(const unsigned int &size, const T &scalar);
    static scalar_matrix_t getI(const unsigned int &size);
    
    typedef Matrix<T, Array2D_Type, ViewType> self_t;
    self_t &swapRows(const unsigned int &row1, const unsigned int &row2);
    self_t &swapColumns(const unsigned int &column1, const unsigned int &column2);
};

%inline %{
typedef MatrixViewBase<> MatViewBase;
typedef MatrixViewTranspose<MatrixViewBase<> > MatView_t;
typedef MatrixViewSizeVariable<MatrixViewOffset<MatrixViewBase<> > > MatView_p;
typedef MatrixViewTranspose<MatrixViewSizeVariable<MatrixViewOffset<MatrixViewBase<> > > > MatView_pt;
%}

%define INSTANTIATE_MATRIX_FUNC(func_orig, func_new)
%template(func_new) func_orig<T, Array2D_ScaledUnit<T>, MatViewBase>;
%template(func_new) func_orig<T, Array2D_ScaledUnit<T>, MatView_p>;
%template(func_new) func_orig<T, Array2D_ScaledUnit<T>, MatView_pt>;
%template(func_new) func_orig<T, Array2D_Dense<T>, MatViewBase>;
%template(func_new) func_orig<T, Array2D_Dense<T>, MatView_t>;
%template(func_new) func_orig<T, Array2D_Dense<T>, MatView_p>;
%template(func_new) func_orig<T, Array2D_Dense<T>, MatView_pt>;
%enddef

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
  
  %typemap(in,numinputs=0) (swig_type_info *info_for_each, T) {
    if(!rb_block_given_p()){
      return rb_enumeratorize(self, ID2SYM(rb_intern("each")), argc, argv);
    }
    $1 = $2_descriptor;
  }
  %typemap(argout) (swig_type_info *info_for_each, T) {
    $result = self;
  }
  void each(swig_type_info *info_for_each, T) const {
    for(unsigned int i(0); i < $self->rows(); ++i){
      for(unsigned int j(0); j < $self->columns(); ++j){
        rb_yield_values(3,
            to_value(info_for_each, (*($self))(i, j)),
            UINT2NUM(i), UINT2NUM(j));
      }
    }
  }
#endif
};

#if defined(SWIGRUBY)
%mixin Matrix_Frozen "Enumerable";
#endif

MAKE_TO_S(Matrix_Frozen)

%extend Matrix {
  %typemap(default) (const T *serialized, int length, swig_type_info *info) {
    $1 = NULL;
    $2 = 0;
    $3 = $*1_descriptor;
  }
#ifdef SWIGRUBY
  %typemap(in) (const T *serialized, int length, swig_type_info *info) {
    if(RB_TYPE_P($input, T_ARRAY)){
      $2 = RARRAY_LEN($input);
      $1 = new T [$2];
      for(unsigned int i(0); i < $2; ++i){
        VALUE rb_obj(RARRAY_AREF($input, i));
        from_value(rb_obj, $3, $1[i]);
      }
    }
  }
#endif
  %typemap(freearg) (const T *serialized, int length, swig_type_info *info) {
    delete [] $1;
  }
  Matrix(
      const unsigned int &rows, const unsigned int &columns, 
      const T *serialized, int length, swig_type_info *info){
    if(serialized){
      if(length < (rows * columns)){
        throw std::runtime_error("Length is too short");
      }
      return new Matrix<T, Array2D_Type, ViewType>(rows, columns, serialized);
    }
    Matrix<T, Array2D_Type, ViewType> *res(
        new Matrix<T, Array2D_Type, ViewType>(rows, columns));
#ifdef SWIGRUBY
    if(rb_block_given_p()){
      for(unsigned int i(0); i < rows; ++i){
        for(unsigned int j(0); j < columns; ++j){
          from_value(
              rb_yield_values(2, UINT2NUM(i), UINT2NUM(j)),
              info, (*res)(i, j));
        }
      }
    }
#endif
    return res;
  }

  T &__setitem__(const unsigned int &row, const unsigned int &column, const T &value) {
    return (($self)->operator()(row, column) = value);
  }
  %rename("scalar") getScalar;
  %rename("I") getI;
  %rename("swap_rows") swapRows;
  %rename("swap_columns") swapColumns;
  
  template <class T2, class Array2D_Type2, class ViewType2>
  self_t &replace(const Matrix_Frozen<T2, Array2D_Type2, ViewType2> &matrix){
    return $self->replace(matrix);
  }
  INSTANTIATE_MATRIX_FUNC(replace, replace);
#ifdef SWIGRUBY
  %bang swapRows(const unsigned int &, const unsigned int &);
  %bang swapColumns(const unsigned int &, const unsigned int &);
  %rename("replace!") replace;
#endif
};

%define INSTANTIATE_MATRIX_TRANSPOSE(type, storage, view_from, view_to)
%extend Matrix_Frozen<type, storage, view_from> {
  Matrix_Frozen<type, storage, view_to> transpose() const {
    return $self->transpose();
  }
};
%enddef

%define INSTANTIATE_MATRIX_PARTIAL(type, storage, view_from, view_to)
%extend Matrix_Frozen<type, storage, view_from> {
  Matrix_Frozen<type, storage, view_to> partial(
      const unsigned int &new_rows, const unsigned int &new_columns,
      const unsigned int &row_offset, const unsigned int &column_offset) const {
    return $self->partial(new_rows, new_columns, row_offset, column_offset);
  }
  Matrix_Frozen<type, storage, view_to> row_vector(const unsigned int &row) const {
    return $self->rowVector(row);
  }
  Matrix_Frozen<type, storage, view_to> column_vector(const unsigned int &column) const {
    return $self->columnVector(column);
  }
};
%enddef

%define INSTANTIATE_MATRIX(type, suffix)
%extend Matrix_Frozen<type, Array2D_ScaledUnit<type >, MatViewBase> {
  const Matrix_Frozen<type, Array2D_ScaledUnit<type >, MatViewBase> &transpose() const {
    return *($self);
  }
  Matrix_Frozen<type, Array2D_ScaledUnit<type >, MatViewBase> inverse() const {
    return $self->inverse();
  }
};
INSTANTIATE_MATRIX_TRANSPOSE(type, Array2D_ScaledUnit<type >, MatView_p, MatView_pt);
INSTANTIATE_MATRIX_TRANSPOSE(type, Array2D_ScaledUnit<type >, MatView_pt, MatView_p);
INSTANTIATE_MATRIX_PARTIAL(type, Array2D_ScaledUnit<type >, MatViewBase, MatView_p);
INSTANTIATE_MATRIX_PARTIAL(type, Array2D_ScaledUnit<type >, MatView_p, MatView_p);
INSTANTIATE_MATRIX_PARTIAL(type, Array2D_ScaledUnit<type >, MatView_pt, MatView_pt);

%template(Matrix_Scalar ## suffix) Matrix_Frozen<type, Array2D_ScaledUnit<type >, MatViewBase>;
%template(Matrix_Scalar ## suffix ## _p) Matrix_Frozen<type, Array2D_ScaledUnit<type >, MatView_p>;
%template(Matrix_Scalar ## suffix ## _pt) Matrix_Frozen<type, Array2D_ScaledUnit<type >, MatView_pt>;

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

%template(Matrix ## suffix) Matrix<type, Array2D_Dense<type > >;
%init %{
#if SWIGRUBY
  { /* work around of %alias I "unit,identity"; // %alias cannot be applied to singleton method */
    VALUE singleton = rb_singleton_class(SwigClassMatrix ## suffix ## .klass);
    rb_define_alias(singleton, "identity", "I");
    rb_define_alias(singleton, "unit", "I");
  }
#endif
%}
%enddef

INSTANTIATE_MATRIX(double, D);
//INSTANTIATE_MATRIX(Complex<double>, ComplexD);