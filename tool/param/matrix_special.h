/*
 * Copyright (c) 2020, M.Naruoka (fenrir)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * - Neither the name of the naruoka.org nor the names of its contributors
 *   may be used to endorse or promote products derived from this software
 *   without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __MATRIX_SPECIAL_H__
#define __MATRIX_SPECIAL_H__

/** @file
 * @brief extension of Portable matrix library to add special type matrices such as symmetric matrix
 *
 */

#include "param/matrix.h"

#if (__cplusplus < 201103L) && !defined(noexcept)
#define noexcept throw()
#endif
#if defined(DEBUG) && !defined(throws_when_debug)
#define throws_when_debug
#else
#define throws_when_debug noexcept
#endif

#if defined(_MSC_VER)
#define DELETE_IF_MSC(x)
#else
#define DELETE_IF_MSC(x) x
#endif

template <class BaseView>
struct MatrixViewSpecial_Symmetric;

template <class BaseView>
struct MatrixViewSpecial_Diagonal;

template <class View>
struct MatrixViewSpecialBuilder : public MatrixViewBuilder<View> {
  typedef View view_t;
  template <template <class> class RemoveView>
  struct remove_t {
    typedef MatrixViewSpecialBuilder<
        typename MatrixViewBuilder<View>::template remove_t<RemoveView>::res_t> builder_t;
  };
  typedef typename remove_t<MatrixViewSpecial_Symmetric>::builder_t
      ::template remove_t<MatrixViewSpecial_Diagonal>::builder_t
      ::view_t none_special_t;
};

template <class MatrixT, template <class> class ViewType_Special>
struct MatrixBuilderSpecial;

template <
    class T, class Array2D_Type, class ViewType,
    template <class> class ViewType_Special>
struct MatrixBuilderSpecial<Matrix_Frozen<T, Array2D_Type, ViewType>, ViewType_Special> {
  typedef ViewType_Special<
      typename MatrixViewSpecialBuilder<ViewType>::none_special_t> view_special_t;
  typedef Matrix_Frozen<T, Array2D_Type, view_special_t> special_t;
};
template <class T, template <class> class ViewType_Special>
struct MatrixBuilderSpecial<Matrix_Frozen<T, Array2D_ScaledUnit<T>, MatrixViewBase<> >, ViewType_Special> {
  typedef Matrix_Frozen<T, Array2D_ScaledUnit<T>, MatrixViewBase<> > special_t;
};

template <
    class T, class Array2D_Type, class ViewType,
    template <class> class ViewType_Special>
struct Matrix_Frozen_Special
    : public Matrix_Frozen<T, Array2D_Type, typename ViewType_Special<ViewType>::base_t> {
  typedef Matrix_Frozen_Special<T, Array2D_Type, ViewType, ViewType_Special> self_t;
  typedef Matrix_Frozen<T, Array2D_Type, typename ViewType_Special<ViewType>::base_t> super_t;
  typedef typename MatrixBuilderSpecial<
      Matrix_Frozen<T, Array2D_Type, ViewType>, ViewType_Special>::special_t special_t;
  typedef MatrixBuilder<super_t> builder_t;
  template <class ViewType2>
  Matrix_Frozen_Special(const Matrix_Frozen<T, Array2D_Type, ViewType2> &mat) noexcept
      : super_t(mat){
    /* Reason to use template ViewType2 is to treat with duplication of special view.
     * For example, if as_symmetric(as_symmetric(mat)), then this construct receives
     * ViewType2 == Symmetric<AnotherBaseView> different from ViewType = AnotherBaseView.
     */
  }
  operator typename builder_t::assignable_t() const {
    typedef typename builder_t::assignable_t res_t;
    res_t res(res_t::blank(super_t::rows(), super_t::columns()));
    builder_t::copy_value(res, *this);
    return res;
  }
  static special_t as_special(const Matrix_Frozen<T, Array2D_Type, ViewType> &mat){
    return special_t(mat);
  }

  using super_t::operator+;
  using super_t::operator-;
  using super_t::operator*;
  using super_t::operator/;

  template<typename U> struct arg_type;
  template<typename U, typename V> struct arg_type<U(V)>{typedef V type;};
#define get_type(t) typename arg_type<void(t)>::type
#define upgrade_function(fname, in_type, out_type) \
typename MatrixBuilderSpecial<get_type(out_type), ViewType_Special>::special_t \
    fname(const get_type(in_type) &in) const { \
  return typename MatrixBuilderSpecial< \
      get_type(out_type), ViewType_Special>::special_t(super_t::fname(in)); \
}
#define upgrade_friend_operator(op, in_type, out_type) \
friend typename MatrixBuilderSpecial<get_type(out_type), ViewType_Special>::special_t \
    operator op(const get_type(in_type) &in, const self_t &matrix) { \
  return typename MatrixBuilderSpecial<get_type(out_type), ViewType_Special> \
      ::special_t(in op (const super_t &)matrix); \
}

  upgrade_function(operator*, T, typename super_t::mul_mat_scalar_t::mat_t);

  upgrade_friend_operator(*, T, typename super_t::mul_mat_scalar_t::mat_t);

  upgrade_function(operator/, T, typename super_t::mul_mat_scalar_t::mat_t);

  typename MatrixBuilderSpecial<
      typename super_t::mul_mat_scalar_t::mat_t, ViewType_Special>::special_t
      operator-() const noexcept {
    return typename MatrixBuilderSpecial<
        typename super_t::mul_mat_scalar_t::mat_t, ViewType_Special>
        ::special_t(super_t::operator-());
  }

  upgrade_function(operator+, T,
      typename super_t::template Add_Matrix_to_Matrix<typename super_t::scalar_matrix_t>::mat_t);

  upgrade_function(operator-, T,
      typename super_t::template Add_Matrix_to_Matrix<typename super_t::scalar_matrix_t>::mat_t);

  upgrade_friend_operator(+, T,
      typename super_t::template Add_Matrix_to_Matrix<typename super_t::scalar_matrix_t>::mat_t);

  upgrade_friend_operator(-, T,
      (typename super_t::scalar_matrix_t::template Add_Matrix_to_Matrix<super_t, false>::mat_t));

  // Preserve feature even if scalar is multiplied
#if !(defined(__GNUC__) && (__GNUC__ <= 5))
  template <class T2>
  typename MatrixBuilderSpecial<
      typename super_t::template Multiply_Matrix_by_Scalar<T2>::mat_t,
      ViewType_Special>::special_t operator*(
        const Matrix_Frozen<T2, Array2D_ScaledUnit<T2> > &in) const {
    return typename MatrixBuilderSpecial<
        typename super_t::template Multiply_Matrix_by_Scalar<T2>::mat_t,
        ViewType_Special>::special_t(super_t::operator*(in));
  }
#else
  /* The below code is ideal, however, it cannot be applied to T2 instead of T,
   * due to macro expansion and overload deduction.
   * In addition, g++-5.4 raises build error (maybe bug?) of ambiguous overload
   * against super class functions, therefore, only T overload is applied.
   */
  upgrade_function(operator*, (Matrix_Frozen<T, Array2D_ScaledUnit<T> >),
      typename super_t::template Multiply_Matrix_by_Scalar<T>::mat_t);
#endif


  // Adding / Subtracting a matrix having same or different special feature {
  template <
      class T2, class Array2D_Type2, class ViewType2,
      template <class> class ViewType_Special_self, template <class> class ViewType_Special_other,
      bool rhs_positive = true>
  struct add_mat_mat_t {
    // default: error and call original due to SFINAE, consequently special feature is removed from return type
  };
  template <class T2, class Array2D_Type2, class ViewType2, bool rhs_positive>
  struct add_mat_mat_t<T2, Array2D_Type2, ViewType2, ViewType_Special, ViewType_Special, rhs_positive> {
    // (same feature) + (same feature) => (same feature), ex) (symmetric) + (symmetric) => (symmetric)
    typedef typename MatrixBuilderSpecial<
        typename super_t::template Add_Matrix_to_Matrix<
          Matrix_Frozen<T2, Array2D_Type2, ViewType_Special<ViewType2> >, rhs_positive>::mat_t,
        ViewType_Special>::special_t res_t;
  };
  template <class T2, class Array2D_Type2, class ViewType2, bool rhs_positive>
  struct add_mat_mat_t<T2, Array2D_Type2, ViewType2, MatrixViewSpecial_Diagonal, MatrixViewSpecial_Symmetric, rhs_positive> {
    // (diagonal) + (symmetric) => (symmetric)
    typedef typename MatrixBuilderSpecial<
        typename super_t::template Add_Matrix_to_Matrix<
          Matrix_Frozen<T2, Array2D_Type2, MatrixViewSpecial_Symmetric<ViewType2> >, rhs_positive>::mat_t,
        MatrixViewSpecial_Symmetric>::special_t res_t;
  };
  template <class T2, class Array2D_Type2, class ViewType2, bool rhs_positive>
  struct add_mat_mat_t<T2, Array2D_Type2, ViewType2, MatrixViewSpecial_Symmetric, MatrixViewSpecial_Diagonal, rhs_positive> {
    // (symmetric) + (diagonal) => (symmetric)
    typedef typename MatrixBuilderSpecial<
        typename super_t::template Add_Matrix_to_Matrix<
          Matrix_Frozen<T2, Array2D_Type2, MatrixViewSpecial_Diagonal<ViewType2> >, rhs_positive>::mat_t,
        MatrixViewSpecial_Symmetric>::special_t res_t;
  };

  template <class T2, class Array2D_Type2, class ViewType2, template <class> class ViewType_Special2>
  typename add_mat_mat_t<T2, Array2D_Type2, ViewType2, ViewType_Special, ViewType_Special2>::res_t operator+(
        const Matrix_Frozen<T2, Array2D_Type2, ViewType_Special2<ViewType2> > &matrix) const {
    return typename add_mat_mat_t<T2, Array2D_Type2, ViewType2, ViewType_Special, ViewType_Special2>::res_t(
        super_t::operator+(matrix));
  }
  template <class T2, class Array2D_Type2, class ViewType2, template <class> class ViewType_Special2>
  typename add_mat_mat_t<T2, Array2D_Type2, ViewType2, ViewType_Special, ViewType_Special2, false>::res_t operator-(
        const Matrix_Frozen<T2, Array2D_Type2, ViewType_Special2<ViewType2> > &matrix) const {
    return typename add_mat_mat_t<T2, Array2D_Type2, ViewType2, ViewType_Special, ViewType_Special2, false>::res_t(
        super_t::operator-(matrix));
  }
  // }

  // Multiplying a matrix having same or different special feature {
  template <
  class T2, class Array2D_Type2, class ViewType2,
  template <class> class ViewType_Special_self, template <class> class ViewType_Special_other>
  struct mul_mat_mat_t {
    // default: error and call original due to SFINAE, consequently special feature is removed from return type
  };
  template <class T2, class Array2D_Type2, class ViewType2>
  struct mul_mat_mat_t<T2, Array2D_Type2, ViewType2, ViewType_Special, ViewType_Special> {
    // (same feature) * (same feature) => (same feature), ex) (symmetric) * (symmetric) => (symmetric)
    typedef typename MatrixBuilderSpecial<
        typename super_t::template Multiply_Matrix_by_Matrix<
          Matrix_Frozen<T2, Array2D_Type2, ViewType_Special<ViewType2> > >::mat_t,
        ViewType_Special>::special_t res_t;
  };
  template <class T2, class Array2D_Type2, class ViewType2>
  struct mul_mat_mat_t<T2, Array2D_Type2, ViewType2, MatrixViewSpecial_Diagonal, MatrixViewSpecial_Symmetric> {
    // (diagonal) * (symmetric) => (symmetric)
    typedef typename MatrixBuilderSpecial<
        typename super_t::template Multiply_Matrix_by_Matrix<
          Matrix_Frozen<T2, Array2D_Type2, MatrixViewSpecial_Symmetric<ViewType2> > >::mat_t,
        MatrixViewSpecial_Symmetric>::special_t res_t;
  };
  template <class T2, class Array2D_Type2, class ViewType2>
  struct mul_mat_mat_t<T2, Array2D_Type2, ViewType2, MatrixViewSpecial_Symmetric, MatrixViewSpecial_Diagonal> {
    // (symmetric) * (diagonal) => (symmetric)
    typedef typename MatrixBuilderSpecial<
        typename super_t::template Multiply_Matrix_by_Matrix<
          Matrix_Frozen<T2, Array2D_Type2, MatrixViewSpecial_Diagonal<ViewType2> > >::mat_t,
        MatrixViewSpecial_Symmetric>::special_t res_t;
  };

  template <class T2, class Array2D_Type2, class ViewType2, template <class> class ViewType_Special2>
  typename mul_mat_mat_t<T2, Array2D_Type2, ViewType2, ViewType_Special, ViewType_Special2>::res_t operator*(
      const Matrix_Frozen<T2, Array2D_Type2, ViewType_Special2<ViewType2> > &matrix) const {
    return typename mul_mat_mat_t<T2, Array2D_Type2, ViewType2, ViewType_Special, ViewType_Special2>::res_t(
        super_t::operator*(matrix));
  }
  // }

#undef upgrade_function
#undef upgrade_friend_operator
#undef get_type
};

#define upgrade_square_matrix(fname, special_upgraded) \
template <class T, class Array2D_Type, class ViewType> \
typename MatrixBuilderSpecial< \
    Matrix_Frozen<T, Array2D_Type, ViewType>, special_upgraded>::special_t fname( \
    const Matrix_Frozen<T, Array2D_Type, ViewType> &mat, \
    const bool &do_check = false){ \
  if(do_check && (!mat.isSquare())){ \
    throw std::runtime_error("Could not be upgraded to " #special_upgraded); \
  } \
  return typename MatrixBuilderSpecial< \
      Matrix_Frozen<T, Array2D_Type, ViewType>, special_upgraded>::special_t(mat); \
}


// Symmetric {
template <class BaseView>
struct MatrixViewSpecial_SymmetricBase : public BaseView {
  struct {} prop;
  template <class T, class Array2D_Type>
  inline T operator()(
      Array2D_Type storage, const unsigned int &i, const unsigned int &j) const {
    return (i > j) // use upper triangle forcedly
        ? BaseView::DELETE_IF_MSC(template) operator()<T, Array2D_Type>(storage, j, i)
        : BaseView::DELETE_IF_MSC(template) operator()<T, Array2D_Type>(storage, i, j);
  }
  static const char *name;
  template<class CharT, class Traits>
  friend std::basic_ostream<CharT, Traits> &operator<<(
      std::basic_ostream<CharT, Traits> &out, const MatrixViewSpecial_SymmetricBase<BaseView> &view){
    return out << name << " " << (const BaseView &)view;
  }
};
template <class BaseView>
const char *MatrixViewSpecial_SymmetricBase<BaseView>::name = "[Symmetric]";
template <class BaseView>
struct MatrixViewSpecial_Symmetric : public MatrixViewSpecial_SymmetricBase<BaseView> {
  typedef MatrixViewSpecial_SymmetricBase<BaseView> base_t;
  static const char *name;
};
template <class BaseView>
const char *MatrixViewSpecial_Symmetric<BaseView>::name = MatrixViewSpecial_SymmetricBase<BaseView>::name;

upgrade_square_matrix(as_symmetric, MatrixViewSpecial_Symmetric);
template <class T, class Array2D_Type, class ViewType>
struct Matrix_Frozen<T, Array2D_Type, MatrixViewSpecial_Symmetric<ViewType> >
    : public Matrix_Frozen_Special<T, Array2D_Type, ViewType, MatrixViewSpecial_Symmetric> {
  typedef Matrix_Frozen_Special<T, Array2D_Type, ViewType, MatrixViewSpecial_Symmetric> super_t;
  template <class ViewType2>
  Matrix_Frozen(const Matrix_Frozen<T, Array2D_Type, ViewType2> &mat) noexcept
      : super_t(mat){
  }
  bool isSquare() const noexcept {return true;}
  bool isDiagonal() const noexcept {return false;}
  bool isSymmetric() const noexcept {return true;}
  typename super_t::special_t transpose() const noexcept {
    return typename super_t::special_t(*this);
  }
};
template <class T, class Array2D_Type, class ViewType>
struct MatrixBuilder_ValueCopier<
    Matrix_Frozen<
      T, Array2D_Type, MatrixViewSpecial_SymmetricBase<ViewType> > > {

  template <class T2, class Array2D_Type2, class ViewType2>
  static Matrix<T2, Array2D_Type2, ViewType2> &copy_value(
      Matrix<T2, Array2D_Type2, ViewType2> &dest,
      const Matrix_Frozen<
        T, Array2D_Type, MatrixViewSpecial_SymmetricBase<ViewType> > &src) {
    // use only upper triangle; lower one is cloned with upper one.
    for(unsigned int i(0); i < src.rows(); ++i){
      dest(i, i) = (T2)(src(i, i));
      for(unsigned int j(i+1); j < src.columns(); ++j){
        dest(i, j) = dest(j, i) = (T2)(src(i, j));
      }
    }
    return dest;
  }
};
// }

// Diagonal {
template <class BaseView>
struct MatrixViewSpecial_DiagonalBase : public BaseView {
  struct {} prop;
  template <class T, class Array2D_Type>
  inline T operator()(
      Array2D_Type storage, const unsigned int &i, const unsigned int &j) const {
    return (i != j) // use upper triangle forcedly
        ? 0
        : BaseView::DELETE_IF_MSC(template) operator()<T, Array2D_Type>(storage, i, i);
  }
  static const char *name;
  template<class CharT, class Traits>
  friend std::basic_ostream<CharT, Traits> &operator<<(
      std::basic_ostream<CharT, Traits> &out, const MatrixViewSpecial_DiagonalBase<BaseView> &view){
    return out << name << " " << (const BaseView &)view;
  }
};
template <class BaseView>
const char *MatrixViewSpecial_DiagonalBase<BaseView>::name = "[Diagonal]";
template <class BaseView>
struct MatrixViewSpecial_Diagonal : public MatrixViewSpecial_DiagonalBase<BaseView> {
  typedef MatrixViewSpecial_DiagonalBase<BaseView> base_t;
  static const char *name;
};
template <class BaseView>
const char *MatrixViewSpecial_Diagonal<BaseView>::name = MatrixViewSpecial_DiagonalBase<BaseView>::name;

upgrade_square_matrix(as_diagonal, MatrixViewSpecial_Diagonal);
template <class T, class Array2D_Type, class ViewType>
struct Matrix_Frozen<T, Array2D_Type, MatrixViewSpecial_Diagonal<ViewType> >
    : public Matrix_Frozen_Special<T, Array2D_Type, ViewType, MatrixViewSpecial_Diagonal> {
  typedef Matrix_Frozen_Special<T, Array2D_Type, ViewType, MatrixViewSpecial_Diagonal> super_t;
  template <class ViewType2>
  Matrix_Frozen(const Matrix_Frozen<T, Array2D_Type, ViewType2> &mat) noexcept
      : super_t(mat){
  }
  bool isSquare() const noexcept {return true;}
  bool isDiagonal() const noexcept {return true;}
  bool isSymmetric() const noexcept {return true;}
  typename super_t::special_t transpose() const noexcept {
    return typename super_t::special_t(*this);
  }
};
template <class T, class Array2D_Type, class ViewType>
struct MatrixBuilder_ValueCopier<
    Matrix_Frozen<
      T, Array2D_Type, MatrixViewSpecial_DiagonalBase<ViewType> > > {

  template <class T2, class Array2D_Type2, class ViewType2>
  static Matrix<T2, Array2D_Type2, ViewType2> &copy_value(
      Matrix<T2, Array2D_Type2, ViewType2> &dest,
      const Matrix_Frozen<
        T, Array2D_Type, MatrixViewSpecial_DiagonalBase<ViewType> > &src) {
    for(unsigned int i(0); i < src.rows(); ++i){
      dest(i, i) = (T2)(src(i, i));
      for(unsigned int j(i+1); j < src.columns(); ++j){
        dest(i, j) = dest(j, i) = (T2)0;
      }
    }
    return dest;
  }
};
// }

// Optimization for multiplication of special matrix {
#define upgrade_mul_mat_mat(view_lhs, view_rhs) \
template < \
    class T, class Array2D_Type, class ViewType, \
    class T2, class Array2D_Type2, class ViewType2> \
struct Array2D_Operator_Multiply< \
      Matrix_Frozen<T, Array2D_Type, view_lhs >, \
      Matrix_Frozen<T2, Array2D_Type2, view_rhs > > \
    : public Array2D_Operator_Binary< \
        Matrix_Frozen<T, Array2D_Type, view_lhs >, \
        Matrix_Frozen<T2, Array2D_Type2, view_rhs > >{ \
  typedef Array2D_Operator_Binary< \
      Matrix_Frozen<T, Array2D_Type, view_lhs >, \
      Matrix_Frozen<T2, Array2D_Type2, view_rhs > > super_t; \
  static const int tag = super_t::lhs_t::OPERATOR_2_Multiply_Matrix_by_Matrix; \
  typedef typename super_t::lhs_t::template Multiply_Matrix_by_Matrix<typename super_t::rhs_t> gen_t; \
  typename gen_t::lhs_opt_t lhs_opt; \
  typename gen_t::rhs_opt_t rhs_opt; \
  Array2D_Operator_Multiply( \
      const typename super_t::lhs_t &_lhs, \
      const typename super_t::rhs_t &_rhs) noexcept \
      : super_t(_lhs, _rhs), lhs_opt(_lhs), rhs_opt(_rhs) {} \
  T operator()(const unsigned int &row, const unsigned int &column) const noexcept; \
};
#define upgrade_mul_mat_mat_function(view_lhs, view_rhs, fname, out_type) \
template < \
    class T, class Array2D_Type, class ViewType, \
    class T2, class Array2D_Type2, class ViewType2> \
out_type Array2D_Operator_Multiply< \
    Matrix_Frozen<T, Array2D_Type, view_lhs >, \
    Matrix_Frozen<T2, Array2D_Type2, view_rhs > >::fname

upgrade_mul_mat_mat(ViewType, MatrixViewSpecial_Diagonal<ViewType2>);
upgrade_mul_mat_mat_function(ViewType, MatrixViewSpecial_Diagonal<ViewType2>, operator(), T)
    (const unsigned int &row, const unsigned int &column) const noexcept{
  return lhs_opt(row, column) * rhs_opt(column, column);
}
upgrade_mul_mat_mat(MatrixViewSpecial_DiagonalBase<ViewType>, ViewType2);
upgrade_mul_mat_mat_function(MatrixViewSpecial_DiagonalBase<ViewType>, ViewType2, operator(), T)
    (const unsigned int &row, const unsigned int &column) const noexcept{
  return lhs_opt(row, row) * rhs_opt(row, column);
}
upgrade_mul_mat_mat(MatrixViewSpecial_DiagonalBase<ViewType>, MatrixViewSpecial_Diagonal<ViewType2>);
upgrade_mul_mat_mat_function(
    MatrixViewSpecial_DiagonalBase<ViewType>, MatrixViewSpecial_Diagonal<ViewType2>, operator(), T)
    (const unsigned int &row, const unsigned int &column) const noexcept{
  return (row == column) ? (lhs_opt(row, row) * rhs_opt(column, column)) : T(0);
}

#undef upgrade_mul_mat_mat
#undef upgrade_mul_mat_mat_function
// }

#undef upgrade_square_matrix

#undef DELETE_IF_MSC

#undef throws_when_debug
#if (__cplusplus < 201103L) && defined(noexcept)
#undef noexcept
#endif

#endif /* __MATRIX_SPECIAL_H__ */
