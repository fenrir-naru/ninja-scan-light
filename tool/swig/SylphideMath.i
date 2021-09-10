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
  
  %typemap(in,numinputs=0) (void *dummy_for_each) {
    if(!rb_block_given_p()){
      return rb_enumeratorize(self, ID2SYM(rb_intern("each")), argc, argv);
    }
  }
  %typemap(argout) (void *dummy_for_each) {
    $result = self;
  }
  void each(void *dummy_for_each) const {
    for(unsigned int i(0); i < $self->rows(); ++i){
      for(unsigned int j(0); j < $self->columns(); ++j){
        rb_yield_values(3,
            to_value($descriptor(const T &), (*($self))(i, j)),
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
  %typemap(typecheck,precedence=SWIG_TYPECHECK_VOIDPTR) const void *replacer {
#if defined(SWIGRUBY)
    $1 = RB_TYPE_P($input, T_ARRAY);
#else
    $1 = 0;
#endif
  }
  %typemap(in) const void *replacer {
    $1 = &$input;
  }

  Matrix(const unsigned int &rows, const unsigned int &columns, 
      const void *replacer = NULL){
    return new Matrix<T, Array2D_Type, ViewType>(rows, columns);
  }
  Matrix(const unsigned int &rows, const unsigned int &columns,
      const T *serialized){
    return new Matrix<T, Array2D_Type, ViewType>(rows, columns);
  }
#if defined(SWIGRUBY)
  %exception Matrix {
    $action
    if(argc > 2){
      rb_funcall2(self, rb_intern("replace!"), argc - 2, &argv[2]);
    }else if(rb_block_given_p()){
      rb_funcall_passing_block(self, rb_intern("replace!"), 0, NULL);
    }
  }
#endif

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

  self_t &replace(const void *replacer = NULL){
    unsigned int r($self->rows()), c($self->columns());
#if defined(SWIGRUBY)
    if(replacer){
      const SWIG_Object &value(*static_cast<const SWIG_Object *>(replacer));
      if(!RB_TYPE_P(value, T_ARRAY)){
        throw std::runtime_error("Array is required");
      }
      if((unsigned int)RARRAY_LEN(value) < (r * c)){
        throw std::runtime_error("Length is too short");
      }
      unsigned int i_array(0);
      for(unsigned int i(0); i < r; ++i){
        for(unsigned int j(0); j < c; ++j){
          VALUE rb_obj(RARRAY_AREF(value, i_array++));
          if(!from_value(rb_obj, $descriptor(T &), (*$self)(i, j))){
            SWIG_exception(SWIG_TypeError, "T expected");
          }
        }
      }
    } else if(rb_block_given_p()){
      for(unsigned int i(0); i < r; ++i){
        for(unsigned int j(0); j < c; ++j){
          VALUE v(rb_yield_values(2, UINT2NUM(i), UINT2NUM(j)));
          if(!from_value(v, $descriptor(T &), (*$self)(i, j))){
            VALUE v_inspect(rb_inspect(v));
            std::stringstream s;
            s << "Unknown input [" << i << "," << j << "]: ";
            throw std::runtime_error(
                s.str().append(RSTRING_PTR(v_inspect), RSTRING_LEN(v_inspect)));
          }
        }
      }
    } else
#endif
    {
      throw std::runtime_error("Unsupported replacement");
    }
    return *$self;
  }

  self_t &replace(const T *serialized){
    if(serialized){
      unsigned int r($self->rows()), c($self->columns());
      for(unsigned int i(0); i < r; ++i){
        for(unsigned int j(0); j < c; ++j){
          (*$self)(i, j) = *(serialized++);
        }
      }
    }
    return *$self;
  }
  
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
