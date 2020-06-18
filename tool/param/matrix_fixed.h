/*
 * Copyright (c) 2019, M.Naruoka (fenrir)
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

#ifndef __MATRIX_FIXED_H__
#define __MATRIX_FIXED_H__

/** @file
 * @brief extension of Portable matrix library to add fixed size matrix
 *
 * This is hand-made fixed size matrix library whose features are
 * 1) to use template for generic primitive type
 * including not only double for general purpose,
 * but also int used with fixed float for embedded environment.
 * 2) Predetermined the buffer size, which implies running without heap memory.
 * 3) to use views for transpose and partial matrices
 * to reduce copies.
 * 4) to use expression template technique
 * for matrix multiplying, adding, and subtracting
 * to eliminate temporary objects.
 *
 * Be careful, being different from the original (flexible) matrix,
 * this fixed size matrix always utilizes deep copy.
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

template <class T, int nR, int nC = nR, bool has_buffer = false>
struct Array2D_Fixed : public Array2D<T, Array2D_Fixed<T, nR, nC, has_buffer> > {
  const T * const src;
  Array2D_Fixed(
      const unsigned int &rows, const unsigned int &columns,
      const T *serialized = NULL)
      : Array2D<T, Array2D_Fixed<T, nR, nC, has_buffer> >(rows, columns),
      src(serialized) {}
  bool has_source() const noexcept {return src != NULL;}
  T operator()(const unsigned int &row, const unsigned int &column) const {
    return src[(row * this->columns()) + column];
  }
};

template <class T, int nR, int nC>
struct Array2D<T, Array2D_Fixed<T, nR, nC, false> > : public Array2D_Frozen<T> {
  Array2D(const unsigned int &rows, const unsigned int &columns) noexcept
      : Array2D_Frozen<T>(rows, columns){}
};

template <class T, int nR, int nC>
class Array2D_Fixed<T, nR, nC, true> : public Array2D<T, Array2D_Fixed<T, nR, nC, true> > {
  public:
    typedef Array2D_Fixed<T, nR, nC, true> self_t;
    typedef Array2D<T, self_t> super_t;
    static const bool clonable = false;

    template <class T2>
    struct family_t {
      typedef Array2D_Fixed<T2, nR, nC, true> res_t;
    };

    using super_t::rows;
    using super_t::columns;

    template <class T2, class Array2D_Type2, class ViewType2>
    friend class Matrix; // for protected copy(), which can only generate shallow copy

  protected:
    T (* const values)[nR][nC]; ///< array for values

    void check_size() const {
      if((nR < rows()) || (nC < columns())){
        throw std::runtime_error("larger rows or columns");
      }
    }

    template <class T2>
    void fill_values(const Array2D_Frozen<T2> &array){
      if(!values){
        throw std::runtime_error("No buffer");
      }
      for(unsigned int i(0); i < array.rows(); ++i){
        for(unsigned int j(0); j < array.columns(); ++j){
          (*values)[i][j] = array(i, j);
        }
      }
    }

  public:
    Array2D_Fixed(const unsigned int &rows = 0, const unsigned int &columns = 0)
        : super_t(rows, columns), values(NULL) {
      check_size();
    }

    Array2D_Fixed(T (*buf)[nR][nC])
        : super_t(0, 0), values(buf) {}

    /**
     * Copy constructor, which performs shallow copy
     */
    Array2D_Fixed(const self_t &another)
        : super_t(another.rows(), another.columns()), values(another.values) {}


    /**
     * Destructor
     */
    ~Array2D_Fixed(){}

    // Assigners, which performs deep copy
    self_t &operator=(const self_t &rhs) noexcept {
      if(this != &rhs){
        super_t::m_rows = rhs.m_rows;
        super_t::m_columns = rhs.m_columns;
        if(rhs.values){fill_values(rhs);}
      }
      return *this;
    }

    self_t &operator=(const Array2D_Fixed<T, nR, nC, false> &array){
      super_t::m_rows = array.rows();
      super_t::m_columns = array.columns();
      check_size();
      if(array.has_source()){fill_values(array);}
      return *this;
    }

    template <class T2>
    self_t &operator=(const Array2D_Frozen<T2> &array){
      super_t::m_rows = array.rows();
      super_t::m_columns = array.columns();
      check_size();
      fill_values(array);
      return *this;
    }

  protected:
    inline const T &get(
        const unsigned int &row,
        const unsigned int &column) const throws_when_debug {
#if defined(DEBUG)
      super_t::check_index(row, column);
#endif
      return (*values)[row][column];
    }

  public:
    /**
     * Accessor for element
     *
     * @param row Row index
     * @param column Column Index
     * @return (T) Element
     * @throw std::out_of_range When the indices are out of range
     */
    T operator()(
        const unsigned int &row,
        const unsigned int &column) const throws_when_debug {
      return get(row, column);
    }
    T &operator()(
        const unsigned int &row,
        const unsigned int &column) throws_when_debug {
      return const_cast<T &>(
          const_cast<const self_t *>(this)->get(row, column));
    }

    void clear() noexcept {
      std::memset(values, 0, sizeof(T) * nR * nC);
    }

  protected:
    self_t copy(const bool &is_deep = false) const {
      return self_t(*this); ///< is_deep flag will be ignored, and return shallow copy
    }
};

template <class T, int nR, int nC, class ViewType>
class Matrix_Frozen<T, Array2D_Fixed<T, nR, nC, false>, ViewType>
    : public Matrix_Frozen<T, Array2D_Fixed<T, nR, nC, true>, ViewType> {
  public:
    typedef Matrix_Frozen<T, Array2D_Fixed<T, nR, nC, true>, ViewType> super_t;

  protected:
    T buf[nR][nC]; ///< fixed size buffer
    Matrix_Frozen() : super_t(typename super_t::storage_t(&buf)) {}
    Matrix_Frozen(const typename super_t::storage_t &src)
        : super_t(typename super_t::storage_t(&buf)) {
      super_t::storage = src;
    }
    /*
     * This constructor is called by Matrix(row, columns [, serialized]) when has_buffer = false.
     * When has_buffer = true, Matrix(const storage_t &) calls this, which be also invoked by
     * Matrix::copy()
     */
    template <bool has_buffer>
    Matrix_Frozen(const Array2D_Fixed<T, nR, nC, has_buffer> &src)
        : super_t(typename super_t::storage_t(&buf)) {
      super_t::storage = src;
    }
  public:
    Matrix_Frozen(const Matrix<T, Array2D_Fixed<T, nR, nC, false>, ViewType> &matrix)
        : super_t(typename super_t::storage_t(&buf)) {
      // change copy style from shallow to deep
      super_t::storage = matrix.storage;
      super_t::view = matrix.view;
    }
    template <class T2, class Array2D_Type2>
    Matrix_Frozen(const Matrix_Frozen<T2, Array2D_Type2, ViewType> &matrix)
        : super_t(typename super_t::storage_t(&buf)) {
      super_t::storage = matrix.storage;
      super_t::view = matrix.view;
    }
  protected:
    /* This view changer is not assumed to be called, because
     * Matrix<T, Array2D_Fixed<T, nR, nC, false>, ViewType2>::view_change() returns
     * Matrix<T, Array2D_Fixed<T, nR, nC, true>, ViewType2::view_changed> resulting from the following
     * specialization of MatrixBuilder_ViewTransformer.
     */
    template <class ViewType2>
    Matrix_Frozen(const Matrix<T, Array2D_Fixed<T, nR, nC, false>, ViewType2> &matrix);
  public:
    template <class ViewType2>
    operator Matrix<T, Array2D_Fixed<T, nR, nC, true>, ViewType2> () const {
      return Matrix<T, Array2D_Fixed<T, nR, nC, true>, ViewType2>(
          Matrix<T, Array2D_Fixed<T, nR, nC, true>, ViewType>(*this));
    }
};

template <class T, int nR, int nC, class ViewType>
struct MatrixBuilder_ViewTransformer<Matrix<T, Array2D_Fixed<T, nR, nC, false>, ViewType> >
    : public MatrixBuilder_ViewTransformerBase<Matrix<T, Array2D_Fixed<T, nR, nC, true>, ViewType> > {};

template <class T, int nR, int nC, class ViewType>
struct MatrixBuilder_ViewTransformer<Matrix_Frozen<T, Array2D_Fixed<T, nR, nC, false>, ViewType> >
    : public MatrixBuilder_ViewTransformerBase<Matrix_Frozen<T, Array2D_Fixed<T, nR, nC, true>, ViewType> > {};


template <class T, int nR, int nC = nR>
struct Matrix_Fixed {
  typedef Matrix<T, Array2D_Fixed<T, nR, nC> > mat_t;
};

template <
    template <class, class, class> class MatrixT,
    class T, int nR, int nC, bool has_buffer, class ViewType,
    int nR_add, int nC_add, int nR_multiply, int nC_multiply>
struct MatrixBuilder<
    MatrixT<T, Array2D_Fixed<T, nR, nC, has_buffer>, ViewType>,
    nR_add, nC_add, nR_multiply, nC_multiply>
    : public MatrixBuilderBase<MatrixT<T, Array2D_Fixed<T, nR, nC, has_buffer>, ViewType> >{

  typedef Matrix<T, Array2D_Fixed<T,
      (MatrixViewProperty<ViewType>::transposed ? nC : nR) * nR_multiply + nR_add,
      (MatrixViewProperty<ViewType>::transposed ? nR : nC) * nC_multiply + nC_add> > assignable_t;

  template <class T2>
  struct family_t {
    typedef typename MatrixBuilder<
        MatrixT<T2, Array2D_Fixed<T2, nR, nC>, ViewType>,
        nR_add, nC_add, nR_multiply, nC_multiply>::assignable_t assignable_t;
  };
};

template <
    template <class, class, class> class MatrixT,
    class T, class Array2D_Type, class ViewType,
    int nR_add, int nC_add>
struct MatrixBuilder<
    MatrixT<T, Array2D_Type, ViewType>,
    nR_add, nC_add, 0, 0>
    : public MatrixBuilder<MatrixT<T, Array2D_Fixed<T, nR_add, nC_add>, ViewType> > {};

// For resolution of partial specialization ambiguity
template <
    template <class, class, class> class MatrixT,
    class T, int nR, int nC, bool has_buffer, class ViewType,
    int nR_add, int nC_add>
struct MatrixBuilder<
    MatrixT<T, Array2D_Fixed<T, nR, nC, has_buffer>, ViewType>,
    nR_add, nC_add, 0, 0>
    : public MatrixBuilder<MatrixT<T, Array2D_Fixed<T, nR_add, nC_add, has_buffer>, ViewType> > {};

template <
    class T,
    class ViewType,
    int nR_L, int nC_L, bool has_buffer_L, class ViewType_L,
    int nR_R, int nC_R, bool has_buffer_R, class ViewType_R>
struct MatrixBuilder<
    Matrix_Frozen<
        T,
        Array2D_Operator<T, Array2D_Operator_Multiply<
          Matrix_Frozen<T, Array2D_Fixed<T, nR_L, nC_L, has_buffer_L>, ViewType_L>,
          Matrix_Frozen<T, Array2D_Fixed<T, nR_R, nC_R, has_buffer_R>, ViewType_R> > >,
        ViewType>,
    0, 0, 1, 1>
    : public MatrixBuilderBase<
      Matrix_Frozen<
          T,
          Array2D_Operator<T, Array2D_Operator_Multiply<
            Matrix_Frozen<T, Array2D_Fixed<T, nR_L, nC_L, has_buffer_L>, ViewType_L>,
            Matrix_Frozen<T, Array2D_Fixed<T, nR_R, nC_R, has_buffer_R>, ViewType_R> > >,
          ViewType> > {
  typedef Matrix<T, Array2D_Fixed<T,
      (MatrixViewProperty<ViewType>::transposed
          ? (MatrixViewProperty<ViewType_R>::transposed ? nR_R : nC_R)
          : (MatrixViewProperty<ViewType_L>::transposed ? nC_L : nR_L)),
      (MatrixViewProperty<ViewType>::transposed
          ? (MatrixViewProperty<ViewType_L>::transposed ? nC_L : nR_L)
          : (MatrixViewProperty<ViewType_R>::transposed ? nR_R : nC_R))> > assignable_t;
};

template <
    class T,
    class ViewType,
    int nR_L, int nC_L, bool has_buffer_L, class ViewType_L,
    int nR_R, int nC_R, bool has_buffer_R, class ViewType_R,
    int nR_add, int nC_add, int nR_multiply, int nC_multiply>
struct MatrixBuilder<
    Matrix_Frozen<
        T,
        Array2D_Operator<T, Array2D_Operator_Multiply<
          Matrix_Frozen<T, Array2D_Fixed<T, nR_L, nC_L, has_buffer_L>, ViewType_L>,
          Matrix_Frozen<T, Array2D_Fixed<T, nR_R, nC_R, has_buffer_R>, ViewType_R> > >,
        ViewType>,
    nR_add, nC_add, nR_multiply, nC_multiply> {
  typedef Matrix<T, Array2D_Fixed<T,
      (MatrixViewProperty<ViewType>::transposed
          ? (MatrixViewProperty<ViewType_R>::transposed ? nR_R : nC_R)
          : (MatrixViewProperty<ViewType_L>::transposed ? nC_L : nR_L))
            * nR_multiply + nR_add,
      (MatrixViewProperty<ViewType>::transposed
          ? (MatrixViewProperty<ViewType_L>::transposed ? nC_L : nR_L)
          : (MatrixViewProperty<ViewType_R>::transposed ? nR_R : nC_R))
            * nC_multiply + nC_add> > assignable_t;
};

#undef throws_when_debug
#if (__cplusplus < 201103L) && defined(noexcept)
#undef noexcept
#endif

#endif /* __MATRIX_FIXED_H__ */
