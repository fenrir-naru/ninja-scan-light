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
    struct cast_t {
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
      const unsigned int i_end(array.rows()), j_end(array.columns());
      for(unsigned int i(0); i < i_end; ++i){
        for(unsigned int j(0); j < j_end; ++j){
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
        T (&buf)[nR][nC], const unsigned int &rows = 0, const unsigned int &columns = 0)
        : super_t(rows, columns), values(&buf) {
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

  protected:
    template <class T2, bool do_memory_op = std::numeric_limits<T2>::is_specialized>
    struct setup_t {
      static void copy(T2 *dest, const T2 *src, const unsigned int &length){
        for(unsigned int i(0); i < length; ++i){
          dest[i] = src[i];
        }
      }
      static void clear(T2 *target){
        for(unsigned int i(0); i < nR * nC; ++i){
          target[i] = T2();
        }
      }
    };
    template <class T2>
    struct setup_t<T2, true> {
      static void copy(T2 *dest, const T2 *src, const unsigned int &length){
        std::memcpy(dest, src, sizeof(T2) * length);
      }
      static void clear(T2 *target){
        std::memset(target, 0, sizeof(T2) * nR * nC);
      }
    };

  public:
    void clear() noexcept {
      setup_t<T>::clear((T *)values);
    }

  protected:
    self_t copy(const bool &is_deep = false) const {
      return self_t(*this); ///< is_deep flag will be ignored, and return shallow copy
    }

  public:
    struct buf_t {
      T buf[nR][nC]; ///< fixed size buffer
      buf_t() noexcept {}
      buf_t(
          const unsigned int &rows, const unsigned int &columns,
          const T *serialized) noexcept {
        if(serialized){
          for(unsigned int i(0), idx(0); i < rows; ++i, idx += columns){
            setup_t<T>::copy(buf[i], &serialized[idx], columns);
          }
        }else{
          setup_t<T>::clear((T *)&buf);
        }
      }
    };
};

template <class T, int nR, int nC = nR>
class Matrix_Fixed
    : protected Array2D_Fixed<T, nR, nC>::buf_t,
    public Matrix<T, Array2D_Fixed<T, nR, nC> > {
  public:
    typedef typename Array2D_Fixed<T, nR, nC>::buf_t buf_t;
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
    Matrix_Fixed(const storage_t &storage) noexcept
        : buf_t(), super_t(storage_t(buf_t::buf)) {
      super_t::storage = storage;
    }

    template <class T2, class Array2D_Type2, class ViewType2>
    friend class Matrix;

  public:
    /**
     * Constructor without customization.
     * The elements will be cleared with T(0).
     *
     */
    Matrix_Fixed() noexcept
        : buf_t(), super_t(storage_t(buf_t::buf)){}

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
        : buf_t(rows, columns, serialized),
        super_t(storage_t(buf_t::buf, rows, columns)) {}

    /**
     * Copy constructor generating deep copy.
     */
    Matrix_Fixed(const self_t &matrix) noexcept
        : buf_t(), super_t(storage_t(buf_t::buf)) {
      super_t::storage = matrix.storage;
    }

    template <class T2, class Array2D_Type2, class ViewType2>
    Matrix_Fixed(const Matrix_Frozen<T2, Array2D_Type2, ViewType2> &matrix)
        : buf_t(), super_t(storage_t(buf_t::buf, matrix.rows(), matrix.columns())){
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

    struct wrapped_t {
      self_t mat;
      wrapped_t(const self_t &mat_) noexcept
          : mat(mat_) {}
    };
};

template <class T, int nR, int nC>
struct MatrixBuilder<Matrix_Fixed<T, nR, nC> >
    : public MatrixBuilder<Matrix<T, Array2D_Fixed<T, nR, nC> > > {};
// MatrixBuilder<typename Matrix_Fixed<T, nR, nC>::super_t> invokes "invalid use of incomplete type" error

template <
    template <class, class, class> class MatrixT,
    class T, int nR, int nC, class ViewType>
struct MatrixBuilder_Dependency<MatrixT<T, Array2D_Fixed<T, nR, nC>, ViewType> > {

  static const int row_buffer = (MatrixViewProperty<ViewType>::transposed ? nC : nR);
  static const int column_buffer = (MatrixViewProperty<ViewType>::transposed ? nR : nC);

  typedef Matrix_Fixed<T, row_buffer, column_buffer> assignable_t;

  template <class T2>
  struct cast_t {
    typedef Matrix_Fixed<T2, row_buffer, column_buffer> assignable_t;
  };

  template <int nR_add = 0, int nC_add = 0, int nR_multiply = 1, int nC_multiply = 1>
  struct resize_t {
    typedef Matrix_Fixed<T,
      row_buffer * nR_multiply + nR_add,
      column_buffer * nC_multiply + nC_add> assignable_t;
  };
};

template <
    class T,
    class ViewType,
    int nR_L, int nC_L, class ViewType_L,
    int nR_R, int nC_R, class ViewType_R>
struct MatrixBuilder_Dependency<
    Matrix_Frozen<
        T,
        Array2D_Operator<T, Array2D_Operator_Multiply_by_Matrix<
          Matrix_Frozen<T, Array2D_Fixed<T, nR_L, nC_L>, ViewType_L>,
          Matrix_Frozen<T, Array2D_Fixed<T, nR_R, nC_R>, ViewType_R> > >,
        ViewType> >
    : public MatrixBuilder_Dependency<
        Matrix_Frozen<
          T,
          Array2D_Fixed<
            T,
            (MatrixViewProperty<ViewType_L>::transposed ? nC_L : nR_L),
            (MatrixViewProperty<ViewType_R>::transposed ? nR_R : nC_R)>,
          ViewType> > {};

// For optimization of local temporary matrix
template <
    template <class, class, class> class MatrixT,
    class T, class Array2D_Type, class ViewType>
struct MatrixBuilder<MatrixT<T, Array2D_Type, ViewType> >
    : public MatrixBuilderBase<MatrixT<T, Array2D_Type, ViewType> > {

  template <int nR_add = 0, int nC_add = 0, int nR_multiply = 1, int nC_multiply = 1>
  struct resize_t
      : public MatrixBuilderBase<MatrixT<T, Array2D_Type, ViewType> >
          ::template resize_t<nR_add, nC_add, nR_multiply, nC_multiply> {};

  template <int nR_add, int nC_add>
  struct resize_t<nR_add, nC_add, 0, 0> {
    typedef Matrix_Fixed<T, nR_add, nC_add> assignable_t;
  };
};

template <
    class LHS_T, class RHS_T,
    bool lhs_buffered = false, bool rhs_buffered = false>
struct Matrix_Fixed_BinaryOperator_buffer;

template <class LHS_T, class RHS_T>
struct Matrix_Fixed_BinaryOperator_buffer<LHS_T, RHS_T, true, false> {
  LHS_T lhs;
  const RHS_T &rhs;
  Matrix_Fixed_BinaryOperator_buffer(const LHS_T &lhs_, const RHS_T &rhs_) noexcept
      : lhs(lhs_), rhs(rhs_) {}
};
template <class LHS_T, class RHS_T>
struct Matrix_Fixed_BinaryOperator_buffer<LHS_T, RHS_T, false, true> {
  const LHS_T &lhs;
  RHS_T rhs;
  Matrix_Fixed_BinaryOperator_buffer(const LHS_T &lhs_, const RHS_T &rhs_) noexcept
      : lhs(lhs_), rhs(rhs_) {}
};
template <class LHS_T, class RHS_T>
struct Matrix_Fixed_BinaryOperator_buffer<LHS_T, RHS_T, true, true> {
  LHS_T lhs;
  RHS_T rhs;
  Matrix_Fixed_BinaryOperator_buffer(const LHS_T &lhs_, const RHS_T &rhs_) noexcept
      : lhs(lhs_), rhs(rhs_) {}
};

template <
    class Result_FrozenT, class LHS_T, class RHS_T,
    bool lhs_buffered = false>
struct Matrix_Fixed_multipled_by_Scalar
    : protected Matrix_Fixed_BinaryOperator_buffer<LHS_T, RHS_T, lhs_buffered>,
    public Result_FrozenT {
  typedef Matrix_Fixed_BinaryOperator_buffer<LHS_T, RHS_T, lhs_buffered> buf_t;
  Matrix_Fixed_multipled_by_Scalar(const Matrix_Fixed_multipled_by_Scalar &another) noexcept
      : buf_t(another.buf_t::lhs, another.buf_t::rhs),
      Result_FrozenT(typename Result_FrozenT::storage_t(
        buf_t::lhs.rows(), buf_t::lhs.columns(),
        typename Result_FrozenT::storage_t::op_t(buf_t::lhs, buf_t::rhs))) {}
  Matrix_Fixed_multipled_by_Scalar(const LHS_T &lhs, const RHS_T &rhs) noexcept
      : buf_t(lhs, rhs),
      Result_FrozenT(typename Result_FrozenT::storage_t(
        lhs.rows(), lhs.columns(),
        typename Result_FrozenT::storage_t::op_t(buf_t::lhs, buf_t::rhs))) {}
  template <class, class, class, bool> friend struct Matrix_Fixed_multipled_by_Scalar;
  template <class Result_FrozenT2>
  Matrix_Fixed_multipled_by_Scalar(
      const Matrix_Fixed_multipled_by_Scalar<
          Result_FrozenT2, LHS_T, RHS_T, lhs_buffered> &another) noexcept
      : buf_t(another), // use default copy constructor
      Result_FrozenT(typename Result_FrozenT::storage_t(
        buf_t::lhs.rows(), buf_t::lhs.columns(),
        typename Result_FrozenT::storage_t::op_t(buf_t::lhs, buf_t::rhs))) {}
};

template <class T, int nR_L, int nC_L, class RHS_T>
struct Matrix_multiplied_by_Scalar<Matrix_Fixed<T, nR_L, nC_L>, RHS_T> {
  typedef Matrix_Fixed<T, nR_L, nC_L> lhs_t;
  typedef RHS_T rhs_t;
  typedef Array2D_Operator_Multiply_by_Scalar<
      Matrix_Frozen<T, Array2D_Fixed<T, nR_L, nC_L> >, rhs_t> impl_t;
  typedef Matrix_Frozen<T, Array2D_Operator<T, impl_t> > frozen_t;
  typedef Matrix_Fixed_multipled_by_Scalar<frozen_t, lhs_t, rhs_t, true> mat_t;
  static mat_t generate(const lhs_t &mat, const rhs_t &scalar) {
    return mat_t(mat, scalar);
  }
};

template <
    class Result_FrozenT, class LHS_T, class RHS_T,
    bool lhs_buffered = false, bool rhs_buffered = false>
struct Matrix_Fixed_multipled_by_Matrix
    : protected Matrix_Fixed_BinaryOperator_buffer<LHS_T, RHS_T, lhs_buffered, rhs_buffered>,
    public Result_FrozenT {
  typedef Matrix_Fixed_BinaryOperator_buffer<LHS_T, RHS_T, lhs_buffered, rhs_buffered> buf_t;
  Matrix_Fixed_multipled_by_Matrix(const Matrix_Fixed_multipled_by_Matrix &another) noexcept
      : buf_t(another.buf_t::lhs, another.buf_t::rhs),
      Result_FrozenT(typename Result_FrozenT::storage_t(
        buf_t::lhs.rows(), buf_t::rhs.columns(),
        typename Result_FrozenT::storage_t::op_t(buf_t::lhs, buf_t::rhs))) {}
  Matrix_Fixed_multipled_by_Matrix(const LHS_T &lhs, const RHS_T &rhs) noexcept
      : buf_t(lhs, rhs),
      Result_FrozenT(typename Result_FrozenT::storage_t(
        lhs.rows(), rhs.columns(),
        typename Result_FrozenT::storage_t::op_t(buf_t::lhs, buf_t::rhs))) {}
  template <class, class, class, bool, bool> friend struct Matrix_Fixed_multipled_by_Matrix;
  template <class Result_FrozenT2>
  Matrix_Fixed_multipled_by_Matrix(
      const Matrix_Fixed_multipled_by_Matrix<
          Result_FrozenT2, LHS_T, RHS_T, lhs_buffered, rhs_buffered> &another) noexcept
      : buf_t(another), // use default copy constructor
      Result_FrozenT(typename Result_FrozenT::storage_t(
          buf_t::lhs.rows(), buf_t::rhs.columns(),
        typename Result_FrozenT::storage_t::op_t(buf_t::lhs, buf_t::rhs))) {}

  /* for optimization of scalar multiplication of (M * S) * M, M * (M * S), (M * S) * (M * S) */
  template <class T>
  struct Multiply_Matrix_by_Scalar {
    typedef Matrix_Fixed_multipled_by_Matrix<
        Result_FrozenT, LHS_T, RHS_T, lhs_buffered, rhs_buffered> lhs_t;
    typedef T rhs_t;
    typedef Array2D_Operator_Multiply_by_Scalar<Result_FrozenT, rhs_t> impl_t;
    typedef Matrix_Frozen<T, Array2D_Operator<T, impl_t> > frozen_t;
    typedef Matrix_Fixed_multipled_by_Scalar<frozen_t, lhs_t, rhs_t, lhs_buffered || rhs_buffered> mat_t;
    static mat_t generate(const lhs_t &mat, const rhs_t &scalar) {
      return mat_t(mat, scalar);
    }
  };
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
  typedef Matrix_Fixed_multipled_by_Matrix<frozen_t, lhs_t, rhs_t, false, true> mat_t;
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
  typedef Matrix_Fixed_multipled_by_Matrix<frozen_t, lhs_t, rhs_t, true, false> mat_t;
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
  typedef Matrix_Fixed_multipled_by_Matrix<frozen_t, lhs_t, rhs_t, true, true> mat_t;
  static mat_t generate(const lhs_t &lhs, const rhs_t &rhs) noexcept {
    return mat_t(lhs, rhs);
  }
};

template <
    class T, int nR, int nC,
    class Result_FrozenT = typename Matrix_Fixed<T, nR, nC>::frozen_t>
struct Matrix_Fixed_Wrapped
    : protected Matrix_Fixed<T, nR, nC>::wrapped_t,
    public Result_FrozenT {
  typedef typename Matrix_Fixed<T, nR, nC>::wrapped_t buf_t;
  Matrix_Fixed_Wrapped(const Matrix_Fixed<T, nR, nC> &mat) noexcept
      : buf_t(mat),
      Result_FrozenT(buf_t::mat) {}
  Matrix_Fixed_Wrapped(const Matrix_Fixed_Wrapped<T, nR, nC, Result_FrozenT> &another) noexcept
      : buf_t(another.buf_t::mat),
      Result_FrozenT(buf_t::mat) {}
};

// { /* For matrix_special.h */
template <class MatrixT, template <class> class ViewType_Special>
struct MatrixBuilderSpecial;

template <
    class T, int nR, int nC,
    template <class> class ViewType_Special>
struct MatrixBuilderSpecial<Matrix_Fixed<T, nR, nC>, ViewType_Special>
    : public MatrixBuilderSpecial<
        typename Matrix_Fixed<T, nR, nC>::super_t, ViewType_Special> {
  typedef Matrix_Fixed<T, nR, nC> fixed_t;
  typedef Matrix_Fixed_Wrapped<T, nR, nC,
      typename MatrixBuilderSpecial<
        typename fixed_t::frozen_t, ViewType_Special>::special_t> special_t;
};

template <
    class Result_FrozenT, class LHS_T, class RHS_T,
    bool lhs_buffered, bool rhs_buffered,
    template <class> class ViewType_Special>
struct MatrixBuilderSpecial<
    Matrix_Fixed_multipled_by_Matrix<Result_FrozenT, LHS_T, RHS_T, lhs_buffered, rhs_buffered>,
    ViewType_Special>
    : public MatrixBuilderSpecial<Result_FrozenT, ViewType_Special> {
  typedef Matrix_Fixed_multipled_by_Matrix<
      typename MatrixBuilderSpecial<Result_FrozenT, ViewType_Special>::special_t,
      LHS_T, RHS_T, lhs_buffered, rhs_buffered> special_t;
};

// { /* for operator/(special(Matrix_Fixed)) */
template <
    class T, class Array2D_Type, class ViewType,
    int nR, int nC, class Result_FrozenT>
struct Array2D_Operator_Multiply_by_Matrix<
    Matrix_Frozen<T, Array2D_Type, ViewType>,
    Matrix_Fixed_Wrapped<T, nR, nC, Result_FrozenT> > {
  typedef Matrix_Frozen<T, Array2D_Type, ViewType> lhs_t;
  typedef Matrix_Fixed_Wrapped<T, nR, nC, Result_FrozenT> rhs_t;
  typedef Array2D_Operator_Multiply_by_Matrix<lhs_t, Result_FrozenT> op_t;
  typedef Matrix_Frozen<T, Array2D_Operator<T, op_t> > frozen_t;
  typedef Matrix_Fixed_multipled_by_Matrix<frozen_t, lhs_t, rhs_t, false, true> mat_t;
  static mat_t generate(const lhs_t &lhs, const rhs_t &rhs) noexcept {
    return mat_t(lhs, rhs);
  }
};
template <
    class Result_FrozenT, class LHS_T, class RHS_T,
    bool lhs_buffered, bool rhs_buffered>
struct MatrixBuilder<
    Matrix_Fixed_multipled_by_Matrix<Result_FrozenT, LHS_T, RHS_T, lhs_buffered, rhs_buffered> > {
  template <class ViewType>
  struct view_replace_t {
    typedef Matrix_Fixed_multipled_by_Matrix<
        typename MatrixBuilder<Result_FrozenT>::template view_replace_t<ViewType>::replaced_t,
        LHS_T, RHS_T, lhs_buffered, rhs_buffered> replaced_t;
  };
};
// }

// for friend operator/(scalar, special(Matrix_Fixed))
template <class T, int nR, int nC, class Result_FrozenT, class RHS_T>
struct Matrix_multiplied_by_Scalar<Matrix_Fixed_Wrapped<T, nR, nC, Result_FrozenT>, RHS_T> {
  typedef Matrix_Fixed_Wrapped<T, nR, nC, Result_FrozenT> lhs_t;
  typedef RHS_T rhs_t;
  typedef Array2D_Operator_Multiply_by_Scalar<Result_FrozenT, rhs_t> impl_t;
  typedef Matrix_Frozen<T, Array2D_Operator<T, impl_t> > frozen_t;
  typedef Matrix_Fixed_multipled_by_Scalar<frozen_t, lhs_t, rhs_t, true> mat_t;
  static mat_t generate(const lhs_t &mat, const rhs_t &scalar) {
    return mat_t(mat, scalar);
  }
};

template <
    class Result_FrozenT, class LHS_T, class RHS_T,
    template <class> class ViewType_Special>
struct MatrixBuilderSpecial<
    Matrix_Fixed_multipled_by_Scalar<Result_FrozenT, LHS_T, RHS_T, true>,
    ViewType_Special>
    : public MatrixBuilderSpecial<Result_FrozenT, ViewType_Special> {
  typedef Matrix_Fixed_multipled_by_Scalar<
      typename MatrixBuilderSpecial<Result_FrozenT, ViewType_Special>::special_t,
      LHS_T, RHS_T, true> special_t;
};
// }

#undef throws_when_debug
#if (__cplusplus < 201103L) && defined(noexcept)
#undef noexcept
#endif

#endif /* __MATRIX_FIXED_H__ */
