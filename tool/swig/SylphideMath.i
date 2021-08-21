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
VALUE to_value(swig_type_info *info, const Complex<T> &v){
  return rb_complex_new(
      to_value(info, v.real()), 
      to_value(info, v.imaginary()));
}
template <class T>
bool from_value_object(const VALUE &obj, swig_type_info *info, T &v){
  T *ptr;
  int res(info ? SWIG_ConvertPtr(obj, (void **)&ptr, info, 1) : SWIG_ERROR);
  if(SWIG_IsOK(res)){
    if(ptr){v = *ptr;} // if nil, then keep current v
    if(SWIG_IsNewObj(res)){delete ptr;}
    return true;
  }
  return false;
}
template <class T>
bool from_value_primitive(const VALUE &obj, swig_type_info *info, T &v){
  switch(TYPE(obj)){
    case T_NIL: // if nil, then keep current v
      return true;
    case T_FIXNUM:
      v = NUM2INT(obj);
      return true;
    case T_BIGNUM:
    case T_FLOAT:
    case T_RATIONAL:
      v = NUM2DBL(obj);
      return true;
  }
  return false;
}
template <class T>
bool from_value(const VALUE &obj, swig_type_info *info, T &v){
  return from_value_object(obj, info, v);
}
template <>
bool from_value(const VALUE &obj, swig_type_info *info, double &v){
  return from_value_primitive(obj, info, v);
}
template <class T>
bool from_value(const VALUE &obj, swig_type_info *info, Complex<T> &v){
  if(RB_TYPE_P(obj, T_COMPLEX)){
    return from_value(rb_complex_real(obj), NULL, v.real())
        && from_value(rb_complex_imag(obj), NULL, v.imaginary());
  }else if(from_value(obj, NULL, v.real())){
    v.imaginary() = T(0);
    return true;
  }
  return from_value_object(obj, info, v);
}
%}
#endif

%feature("autodoc", "1");

%ignore Complex::real;
%ignore Complex::imaginary;
%ignore Complex::check_infinity_t;
%ignore operator<<(std::ostream &, const Complex &);

template <class FloatT>
class Complex;

%extend Complex {
#ifdef SWIGRUBY
  %typemap(typecheck,precedence=SWIG_TYPECHECK_POINTER) const Complex<FloatT> & {
    void *vptr = 0;
    $1 = RB_TYPE_P($input, T_COMPLEX)
        || SWIG_CheckState(SWIG_ConvertPtr($input, &vptr, $1_descriptor, 0));
  }
  %typemap(in) const Complex<FloatT> & (Complex<FloatT> temp) {
    from_value($input, $1_descriptor, temp);
    $1 = &temp;
  }
#endif
  Complex(const Complex<FloatT> &complex) noexcept {
    return new Complex<FloatT>(complex);
  }

  static Complex<FloatT> rectangular(const FloatT &r, const FloatT &i = FloatT(0)) noexcept {
    return Complex<FloatT>(r, i);
  }

  MAKE_ACCESSOR(real, FloatT);
  MAKE_ACCESSOR(imaginary, FloatT);

#ifdef SWIGRUBY
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
%init %{
#if SWIGRUBY
  { /* work around of %alias I "unit,identity"; // %alias cannot be applied to singleton method */
    VALUE singleton = rb_singleton_class(SwigClassComplex ## suffix ## .klass);
    rb_define_alias(singleton, "rect", "rectangular");
  }
#endif
%}
%enddef

INSTANTIATE_COMPLEX(double, D);

#undef INSTANTIATE_COMPLEX

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

  %typemap(in,numinputs=0)
      Matrix<T, Array2D_Dense<T> > &output_L (Matrix<T, Array2D_Dense<T> > temp),
      Matrix<T, Array2D_Dense<T> > &output_U (Matrix<T, Array2D_Dense<T> > temp),
      Matrix<T, Array2D_Dense<T> > &output_P (Matrix<T, Array2D_Dense<T> > temp),
      Matrix<T, Array2D_Dense<T> > &output_D (Matrix<T, Array2D_Dense<T> > temp) %{
    $1 = &temp;
  %}
  %typemap(argout)
      Matrix<T, Array2D_Dense<T> > &output_L,
      Matrix<T, Array2D_Dense<T> > &output_U,
      Matrix<T, Array2D_Dense<T> > &output_P,
      Matrix<T, Array2D_Dense<T> > &output_D {
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

#ifdef SWIGRUBY
%{
template <class T, class Array2D_Type, class ViewType>
bool Matrix_replace_with_block(Matrix<T, Array2D_Type, ViewType> &mat, swig_type_info *info){
  if(!rb_block_given_p()){return false;}
  for(unsigned int i(0); i < mat.rows(); ++i){
    for(unsigned int j(0); j < mat.columns(); ++j){
      VALUE v(rb_yield_values(2, UINT2NUM(i), UINT2NUM(j)));
      if(!from_value(v, info, mat(i, j))){
        VALUE v_inspect(rb_inspect(v));
        std::stringstream s;
        s << "Unknown input [" << i << "," << j << "]: ";
        throw std::runtime_error(
            s.str().append(RSTRING_PTR(v_inspect), RSTRING_LEN(v_inspect)));
      }
    }
  }
  return true;
}
%}
#endif

%extend Matrix {
  %typemap(default) (const T *serialized, int length, swig_type_info *info) {
    $1 = NULL;
    $2 = 0;
    $3 = $1_descriptor;
  }
#ifdef SWIGRUBY
  %typemap(typecheck) (const T *serialized, int length, swig_type_info *info) {
    $1 = RB_TYPE_P($input, T_ARRAY);
  }
  %typemap(in) (const T *serialized, int length, swig_type_info *info) {
    if(RB_TYPE_P($input, T_ARRAY)){
      $2 = RARRAY_LEN($input);
      $1 = new T [$2];
      for(int i(0); i < $2; ++i){
        VALUE rb_obj(RARRAY_AREF($input, i));
        if(!from_value(rb_obj, $3, $1[i])){
          SWIG_exception(SWIG_TypeError, "$*1_ltype expected");
        }
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
      if((unsigned int)length < (rows * columns)){
        throw std::runtime_error("Length is too short");
      }
      return new Matrix<T, Array2D_Type, ViewType>(rows, columns, serialized);
    }else{
      Matrix<T, Array2D_Type, ViewType> *res(
         new Matrix<T, Array2D_Type, ViewType>(rows, columns));
      Matrix_replace_with_block(*res, info);
      return res;
    }
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
  self_t &replace(const T *serialized, int length, swig_type_info *info){
    unsigned int r($self->rows()), c($self->columns());
    if(serialized){
      if((unsigned int)length < (r * c)){
        throw std::runtime_error("Length is too short");
      }
      for(unsigned int i(0); i < r; ++i){
        for(unsigned int j(0); j < c; ++j){
          (*($self))(i, j) = *(serialized++);
        }
      }
    }else if(Matrix_replace_with_block(*($self), info)){
      
    }else{
      throw std::runtime_error("Unsupported replacement");
    }
    return *($self);
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

%define INSTANTIATE_MATRIX_EIGEN2(type, storage, view)
%extend Matrix_Frozen<type, storage, view> {
  %typemap(in,numinputs=0)
      Matrix<Complex<type>, Array2D_Dense<Complex<type> > > &output_D 
        (Matrix<Complex<type>, Array2D_Dense<Complex<type> > > temp),
      Matrix<Complex<type>, Array2D_Dense<Complex<type> > > &output_V 
        (Matrix<Complex<type>, Array2D_Dense<Complex<type> > > temp) %{
    $1 = &temp;
  %}
  %typemap(argout)
      Matrix<Complex<type>, Array2D_Dense<Complex<type> > > &output_D,
      Matrix<Complex<type>, Array2D_Dense<Complex<type> > > &output_V {
    %append_output(SWIG_NewPointerObj((new $*1_ltype(*$1)), $1_descriptor, SWIG_POINTER_OWN));
  }
  void eigen(
      Matrix<Complex<type>, Array2D_Dense<Complex<type> > > &output_V, 
      Matrix<Complex<type>, Array2D_Dense<Complex<type> > > &output_D) const {
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
%define INSTANTIATE_MATRIX_EIGEN(type)
INSTANTIATE_MATRIX_EIGEN2(type, Array2D_ScaledUnit<type >, MatViewBase);
INSTANTIATE_MATRIX_EIGEN2(type, Array2D_ScaledUnit<type >, MatView_p);
INSTANTIATE_MATRIX_EIGEN2(type, Array2D_ScaledUnit<type >, MatView_pt);
INSTANTIATE_MATRIX_EIGEN2(type, Array2D_Dense<type >, MatViewBase);
INSTANTIATE_MATRIX_EIGEN2(type, Array2D_Dense<type >, MatView_p);
INSTANTIATE_MATRIX_EIGEN2(type, Array2D_Dense<type >, MatView_t);
INSTANTIATE_MATRIX_EIGEN2(type, Array2D_Dense<type >, MatView_pt);
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
INSTANTIATE_MATRIX_EIGEN(double);
INSTANTIATE_MATRIX(Complex<double>, ComplexD);

#undef INSTANTIATE_MATRIX_FUNC
#undef INSTANTIATE_MATRIX_TRANSPOSE
#undef INSTANTIATE_MATRIX_PARTIAL
#undef INSTANTIATE_MATRIX_EIGEN2
#undef INSTANTIATE_MATRIX_EIGEN
#undef INSTANTIATE_MATRIX

#undef MAKE_ACCESSOR
#undef MAKE_TO_S
