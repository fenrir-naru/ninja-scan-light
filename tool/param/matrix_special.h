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

template <class BaseView>
struct MatrixViewSpecial_Symmetric;

template <class View>
struct MatrixViewSpecialBuilder : public MatrixViewBuilder<View> {
  typedef View view_t;
  template <template <class> class RemoveView>
  struct remove_t {
    typedef MatrixViewSpecialBuilder<
        typename MatrixViewBuilder<View>::template remove_t<RemoveView>::res_t> builder_t;
  };
  typedef typename remove_t<MatrixViewSpecial_Symmetric>::builder_t::view_t none_special_t;
};

// Symmetric {
template <class BaseView>
struct MatrixViewSpecial_Symmetric : public BaseView {
  static const char *name;
  template<class CharT, class Traits>
  friend std::basic_ostream<CharT, Traits> &operator<<(
      std::basic_ostream<CharT, Traits> &out,
      const MatrixViewSpecial_Symmetric<BaseView> &view){
    return out << name << " " << (const BaseView &)view;
  }
};
template <class BaseView>
const char *MatrixViewSpecial_Symmetric<BaseView>::name = "[Symmetric]";

template <class T, class Array2D_Type, class ViewType>
struct Matrix_Symmetric
    : public Matrix_Frozen<T, Array2D_Type, MatrixViewSpecial_Symmetric<ViewType> > {
  typedef Matrix_Symmetric<T, Array2D_Type, ViewType> self_t;
  typedef Matrix_Frozen<T, Array2D_Type, MatrixViewSpecial_Symmetric<ViewType> > super_t;
  template <class ViewType2>
  Matrix_Symmetric(const Matrix_Frozen<T, Array2D_Type, ViewType2> &mat) noexcept
      : super_t(mat){
    /* Reason to use template ViewType2 is to treat with duplication of special view.
     * For example, if as_symmetric(as_symmetric(mat)), then this construct receives
     * ViewType2 == Symmetric<AnotherBaseView> different from ViewType = AnotherBaseView.
     */
  }
  // TODO some functions
};
template <class T, class Array2D_Type, class ViewType>
Matrix_Symmetric<
    T, Array2D_Type,
    typename MatrixViewSpecialBuilder<ViewType>::none_special_t>
    as_symmetric(
    const Matrix_Frozen<T, Array2D_Type, ViewType> &mat){
  return Matrix_Symmetric<
      T, Array2D_Type,
      typename MatrixViewSpecialBuilder<ViewType>::none_special_t>(mat);
}
template <
    template <class, class, class> class MatrixT,
    class T, class Array2D_Type, class ViewType>
struct MatrixBuilder_ViewTransformer<
    MatrixT<T, Array2D_Type, MatrixViewSpecial_Symmetric<ViewType> > >
    : public MatrixBuilder_ViewTransformerBase<
        MatrixT<T, Array2D_Type, ViewType> > {
  typedef MatrixT<T, Array2D_Type, MatrixViewSpecial_Symmetric<ViewType> > transpose_t;
};
template <class T, class Array2D_Type, class ViewType>
struct MatrixBuilderBase<
    Matrix_Frozen<
      T, Array2D_Type, MatrixViewSpecial_Symmetric<ViewType> > >
    : public MatrixBuilder_ViewTransformer<Matrix_Frozen<
      T, Array2D_Type, MatrixViewSpecial_Symmetric<ViewType> > > {

  template <class T2, class Array2D_Type2, class ViewType2>
  static Matrix<T2, Array2D_Type2, ViewType2> &clone_value(
      Matrix<T2, Array2D_Type2, ViewType2> &dest,
      const Matrix_Frozen<
        T, Array2D_Type, MatrixViewSpecial_Symmetric<ViewType> > &src) {
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

#undef throws_when_debug
#if (__cplusplus < 201103L) && defined(noexcept)
#undef noexcept
#endif

#endif /* __MATRIX_SPECIAL_H__ */
