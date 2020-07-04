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

template <class T, int nR, int nC = nR>
class Array2D_Fixed : public Array2D<T, Array2D_Fixed<T, nR, nC> > {
  public:
    typedef Array2D_Fixed<T, nR, nC> self_t;
    typedef Array2D<T, self_t> super_t;
    static const bool clonable = false;

    template <class T2>
    struct family_t {
      typedef Array2D_Fixed<T2, nR, nC> res_t;
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

    Array2D_Fixed(
        T (*buf)[nR][nC], const unsigned int &rows = 0, const unsigned int &columns = 0)
        : super_t(rows, columns), values(buf) {
      check_size();
    }

    /**
     * Copy constructor, which performs shallow copy.
     *
     * @param array another one
     */
    Array2D_Fixed(const self_t &src) noexcept
        : super_t(src.rows(), src.columns()), values(src.values){
    }

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

template <class T, int nR, int nC = nR>
class Matrix_Fixed : public Matrix<T, Array2D_Fixed<T, nR, nC> > {
  public:
    typedef Matrix<T, Array2D_Fixed<T, nR, nC> > super_t;

#if defined(__GNUC__) && (__GNUC__ < 5)
    typedef typename super_t::storage_t storage_t;
#elif defined(_MSC_VER)
    typedef typename super_t::storage_t storage_t; // To fix VC2010(C2514) in constructor
    using super_t::operator(); // To fix VC2010(C2106) of X(r, c) = something
#else
    using typename super_t::storage_t;
#endif

    typedef Matrix_Fixed<T, nR, nC> self_t;

  protected:
    T buf[nR][nC]; ///< fixed size buffer

    Matrix_Fixed(const storage_t &storage) noexcept
        : super_t(storage_t(&buf)) {
      super_t::storage = storage;
    }

    template <class T2, class Array2D_Type2, class ViewType2>
    friend class Matrix;

    struct BufferInitializer : public Array2D_Frozen<T> {
      const T * const src;
      BufferInitializer(
          const unsigned int &rows, const unsigned int &columns,
          const T *serialized = NULL)
          : Array2D_Frozen<T>(rows, columns), src(serialized) {}
      T operator()(const unsigned int &row, const unsigned int &column) const {
        return src[(row * this->columns()) + column];
      }
    };
  public:
    /**
     * Constructor without customization.
     * The elements will be cleared with T(0).
     *
     */
    Matrix_Fixed() noexcept : super_t(storage_t(&buf)){
      super_t::clear();
    }

    /**
     * Constructor with specified row and column numbers, and values.
     * The size will be compared with the predefined numbers.
     * If the size is larger than buffer, then error will be thrown.
     * The elements will be cleared with T(0) if serialized is NULL (default).
     *
     * @param rows Row number
     * @param columns Column number
     * @param serialized Initial values of elements
     */
    Matrix_Fixed(
        const unsigned int &rows, const unsigned int &columns,
        const T *serialized = NULL)
        : super_t(storage_t(&buf, rows, columns)) {
      if(serialized){
        super_t::storage = BufferInitializer(rows, columns, serialized);
      }else{
        super_t::clear();
      }
    }

    /**
     * Copy constructor generating deep copy.
     */
    Matrix_Fixed(const self_t &matrix) noexcept
        : super_t(storage_t(&buf)) {
      super_t::storage = matrix.storage;
    }

    template <class T2, class Array2D_Type2, class ViewType2>
    Matrix_Fixed(const Matrix_Frozen<T2, Array2D_Type2, ViewType2> &matrix)
        : super_t(storage_t(&buf, matrix.rows(), matrix.columns())){
      super_t::replace(matrix);
    }
    /**
     * Destructor
     */
    virtual ~Matrix_Fixed(){}

    self_t &operator=(const self_t &matrix){
      super_t::operator=(matrix); // frozen_t::operator=(const frozen_t &) is exactly called.
      return *this;
    }

    template <class T2, class Array2D_Type2>
    self_t &operator=(const Matrix<T2, Array2D_Type2> &matrix){
      super_t::operator=(matrix);
      /* frozen_t::operator=(const frozen_t &) or frozen_t::operator=(const another_frozen_t &)
       * is conditionally called.
       */
      return *this;
    }
};

template <
    class T, int nR, int nC,
    int nR_add, int nC_add, int nR_multiply, int nC_multiply>
struct MatrixBuilder<
    Matrix_Fixed<T, nR, nC>,
    nR_add, nC_add, nR_multiply, nC_multiply> {

  typedef Matrix_Fixed<T, nR * nR_multiply + nR_add, nC * nC_multiply + nC_add> assignable_t;

  template <class T2>
  struct family_t {
    typedef Matrix_Fixed<T2, nR * nR_multiply + nR_add, nC * nC_multiply + nC_add> assignable_t;
  };
};

template <
    template <class, class, class> class MatrixT,
    class T, int nR, int nC, class ViewType>
struct MatrixBuilder<
    MatrixT<T, Array2D_Fixed<T, nR, nC>, ViewType>,
    0, 0, 1, 1>
    : public MatrixBuilderBase<
        MatrixT<T, Array2D_Fixed<T, nR, nC>, ViewType> > {

  typedef Matrix_Fixed<T,
      (MatrixViewProperty<ViewType>::transposed ? nC : nR),
      (MatrixViewProperty<ViewType>::transposed ? nR : nC)> assignable_t;
};

template <
    template <class, class, class> class MatrixT,
    class T, int nR, int nC, class ViewType,
    int nR_add, int nC_add, int nR_multiply, int nC_multiply>
struct MatrixBuilder<
    MatrixT<T, Array2D_Fixed<T, nR, nC>, ViewType>,
    nR_add, nC_add, nR_multiply, nC_multiply> {

  typedef Matrix_Fixed<T,
      (MatrixViewProperty<ViewType>::transposed ? nC : nR) * nR_multiply + nR_add,
      (MatrixViewProperty<ViewType>::transposed ? nR : nC) * nC_multiply + nC_add> assignable_t;
};

template <
    template <class, class, class> class MatrixT,
    class T, class Array2D_Type, class ViewType,
    int nR_add, int nC_add>
struct MatrixBuilder<
    MatrixT<T, Array2D_Type, ViewType>,
    nR_add, nC_add, 0, 0>
    : public MatrixBuilderBase<MatrixT<T, Array2D_Type, ViewType> > {

  typedef Matrix_Fixed<T, nR_add, nC_add> assignable_t;
  template <class T2>
  struct family_t {
    typedef Matrix_Fixed<T2, nR_add, nC_add> assignable_t;
  };
};

// For resolution of partial specialization ambiguity
template <
    template <class, class, class> class MatrixT,
    class T, int nR, int nC, class ViewType,
    int nR_add, int nC_add>
struct MatrixBuilder<
    MatrixT<T, Array2D_Fixed<T, nR, nC>, ViewType>,
    nR_add, nC_add, 0, 0>
    : public MatrixBuilder<MatrixT<T, Array2D_Fixed<T, nR_add, nC_add>, ViewType> > {};

template <
    class T,
    class ViewType,
    int nR_L, int nC_L, class ViewType_L,
    int nR_R, int nC_R, class ViewType_R>
struct MatrixBuilder<
    Matrix_Frozen<
        T,
        Array2D_Operator<T, Array2D_Operator_Multiply_by_Matrix<
          Matrix_Frozen<T, Array2D_Fixed<T, nR_L, nC_L>, ViewType_L>,
          Matrix_Frozen<T, Array2D_Fixed<T, nR_R, nC_R>, ViewType_R> > >,
        ViewType>,
    0, 0, 1, 1>
    : public MatrixBuilderBase<
      Matrix_Frozen<
          T,
          Array2D_Operator<T, Array2D_Operator_Multiply_by_Matrix<
            Matrix_Frozen<T, Array2D_Fixed<T, nR_L, nC_L>, ViewType_L>,
            Matrix_Frozen<T, Array2D_Fixed<T, nR_R, nC_R>, ViewType_R> > >,
          ViewType> > {
  typedef Matrix_Fixed<T,
      (MatrixViewProperty<ViewType>::transposed
          ? (MatrixViewProperty<ViewType_R>::transposed ? nR_R : nC_R)
          : (MatrixViewProperty<ViewType_L>::transposed ? nC_L : nR_L)),
      (MatrixViewProperty<ViewType>::transposed
          ? (MatrixViewProperty<ViewType_L>::transposed ? nC_L : nR_L)
          : (MatrixViewProperty<ViewType_R>::transposed ? nR_R : nC_R))> assignable_t;
};

template <
    class T,
    class ViewType,
    int nR_L, int nC_L, class ViewType_L,
    int nR_R, int nC_R, class ViewType_R,
    int nR_add, int nC_add, int nR_multiply, int nC_multiply>
struct MatrixBuilder<
    Matrix_Frozen<
        T,
        Array2D_Operator<T, Array2D_Operator_Multiply_by_Matrix<
          Matrix_Frozen<T, Array2D_Fixed<T, nR_L, nC_L>, ViewType_L>,
          Matrix_Frozen<T, Array2D_Fixed<T, nR_R, nC_R>, ViewType_R> > >,
        ViewType>,
    nR_add, nC_add, nR_multiply, nC_multiply> {
  typedef Matrix_Fixed<T,
      (MatrixViewProperty<ViewType>::transposed
          ? (MatrixViewProperty<ViewType_R>::transposed ? nR_R : nC_R)
          : (MatrixViewProperty<ViewType_L>::transposed ? nC_L : nR_L))
            * nR_multiply + nR_add,
      (MatrixViewProperty<ViewType>::transposed
          ? (MatrixViewProperty<ViewType_L>::transposed ? nC_L : nR_L)
          : (MatrixViewProperty<ViewType_R>::transposed ? nR_R : nC_R))
            * nC_multiply + nC_add> assignable_t;
};

template <class T, int nR_L, int nC_L, class RHS_T>
struct Array2D_Operator_Multiply_by_Scalar<Matrix_Fixed<T, nR_L, nC_L>, RHS_T> {
  typedef Matrix_Fixed<T, nR_L, nC_L> lhs_t;
  typedef RHS_T rhs_t;
  typedef typename Array2D_Operator_Multiply_by_Scalar<
      Matrix_Frozen<T, Array2D_Fixed<T, nR_L, nC_L> >, rhs_t>::impl_t impl_t;
  typedef Matrix_Frozen<T, Array2D_Operator<T, impl_t> > frozen_t;
  struct buf_t {
    lhs_t lhs;
    buf_t(const lhs_t &lhs_) : lhs(lhs_) {}
  };
  struct mat_t : public buf_t, frozen_t {
    mat_t(const lhs_t &mat, const rhs_t &scalar)
        : buf_t(mat),
        frozen_t(typename mat_t::storage_t(
          buf_t::lhs.rows(), buf_t::lhs.columns(), impl_t(buf_t::lhs, scalar))) {
    }
  };
  static mat_t generate(const lhs_t &mat, const rhs_t &scalar) {
    return mat_t(mat, scalar);
  }
};

template <
    class T, class Array2D_Type, class ViewType,
    int nR_R, int nC_R>
struct Array2D_Operator_Multiply_by_Matrix<
    Matrix_Frozen<T, Array2D_Type, ViewType>,
    Matrix_Fixed<T, nR_R, nC_R> > {
  typedef Matrix_Frozen<T, Array2D_Type, ViewType> lhs_t;
  typedef Matrix_Fixed<T, nR_R, nC_R> rhs_t;
  typedef Array2D_Operator_Multiply_by_Matrix<lhs_t, typename rhs_t::frozen_t> op_t;
  typedef Matrix_Frozen<T, Array2D_Operator<T, op_t> > frozen_t;
  struct buf_t {
    rhs_t rhs;
    buf_t(const rhs_t &rhs_) : rhs(rhs_) {}
  };
  struct mat_t : public buf_t, frozen_t {
    mat_t(const lhs_t &lhs, const rhs_t &rhs)
        : buf_t(rhs),
        frozen_t(typename frozen_t::storage_t(
            lhs.rows(), rhs.columns(), op_t(lhs, buf_t::rhs))) {}
  };
  static mat_t generate(const lhs_t &lhs, const rhs_t &rhs) noexcept {
    return mat_t(lhs, rhs);
  }
};

template <
    class T, int nR_L, int nC_L,
    class Array2D_Type, class ViewType>
struct Array2D_Operator_Multiply_by_Matrix<
    Matrix_Fixed<T, nR_L, nC_L>,
    Matrix_Frozen<T, Array2D_Type, ViewType> > {
  typedef Matrix_Fixed<T, nR_L, nC_L> lhs_t;
  typedef Matrix_Frozen<T, Array2D_Type, ViewType> rhs_t;
  typedef Array2D_Operator_Multiply_by_Matrix<typename lhs_t::frozen_t, rhs_t> op_t;
  typedef Matrix_Frozen<T, Array2D_Operator<T, op_t> > frozen_t;
  struct buf_t {
    lhs_t lhs;
    buf_t(const lhs_t &lhs_) : lhs(lhs_) {}
  };
  struct mat_t : public buf_t, frozen_t {
    mat_t(const lhs_t &lhs, const rhs_t &rhs)
        : buf_t(lhs),
        frozen_t(typename frozen_t::storage_t(
            lhs.rows(), rhs.columns(), op_t(buf_t::lhs, rhs))) {}
  };
  static mat_t generate(const lhs_t &lhs, const rhs_t &rhs) noexcept {
    return mat_t(lhs, rhs);
  }
};

template <
    class T, int nR_L, int nC_L, int nR_R, int nC_R>
struct Array2D_Operator_Multiply_by_Matrix<
    Matrix_Fixed<T, nR_L, nC_L>,
    Matrix_Fixed<T, nR_R, nC_R> > {
  typedef Matrix_Fixed<T, nR_L, nC_L> lhs_t;
  typedef Matrix_Fixed<T, nR_R, nC_R> rhs_t;
  typedef Array2D_Operator_Multiply_by_Matrix<
      typename lhs_t::frozen_t, typename rhs_t::frozen_t> op_t;
  typedef Matrix_Frozen<T, Array2D_Operator<T, op_t> > frozen_t;
  struct buf_t {
    lhs_t lhs;
    rhs_t rhs;
    buf_t(const lhs_t &lhs_, const rhs_t &rhs_) : lhs(lhs_), rhs(rhs_) {}
  };
  struct mat_t : public buf_t, frozen_t {
    mat_t(const lhs_t &lhs, const rhs_t &rhs)
        : buf_t(lhs, rhs),
        frozen_t(typename frozen_t::storage_t(
            lhs.rows(), rhs.columns(), op_t(buf_t::lhs, buf_t::rhs))) {}
  };
  static mat_t generate(const lhs_t &lhs, const rhs_t &rhs) noexcept {
    return mat_t(lhs, rhs);
  }
};

#undef throws_when_debug
#if (__cplusplus < 201103L) && defined(noexcept)
#undef noexcept
#endif

#endif /* __MATRIX_FIXED_H__ */
