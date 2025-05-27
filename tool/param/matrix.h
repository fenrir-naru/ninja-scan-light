/*
 * Copyright (c) 2015, M.Naruoka (fenrir)
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

#ifndef __MATRIX_H
#define __MATRIX_H

/** @file
 * @brief Portable matrix library
 *
 * This is hand-made matrix library whose features are
 * 1) to use template for generic primitive type
 * including not only double for general purpose,
 * but also int used with fixed float for embedded environment.
 * 2) to utilize shallow copy for small memory usage,
 * which is very important for embedded environment.
 * 3) to use views for transposed and partial matrices
 * to reduce copies.
 * 4) to use expression template technique
 * for matrix multiplying, adding, and subtracting
 * to eliminate temporary objects.
 *
 * Currently it only supports dense matrices,
 * whose storage is prepared as continuous array,
 * however it can support sparse matrices by extending
 * the Array2D class structure.
 */

#include <string>
#include <stdexcept>

#include <cstring>
#include <cmath>
#include <cfloat>
#include <cstdlib>
#include <ostream>
#include <limits>
#include "param/complex.h"

#include <iterator>

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

/**
 * @brief 2D array abstract class for fixed content
 *
 * This class provides basic interface of 2D array, such as row and column numbers,
 * accessor for element.
 *
 * @param T precision, for example, double
 */
template<class T>
class Array2D_Frozen{
  public:
    typedef Array2D_Frozen<T> self_t;
    static const bool writable = false;

  protected:
    unsigned int m_rows;    ///< Rows
    unsigned int m_columns; ///< Columns
    
  public:
    typedef T content_t;

    /**
     * Constructor of Array2D
     *
     * @param rows Rows
     * @param columns Columns
     */
    Array2D_Frozen(const unsigned int &rows, const unsigned int &columns) noexcept
        : m_rows(rows), m_columns(columns){}

    /**
     * Destructor of Array2D
     */
    virtual ~Array2D_Frozen(){}

    /**
     * Return rows
     *
     * @return (unsigned int) Rows
     */
    const unsigned int &rows() const noexcept {return m_rows;}
    /**
     * Return columns
     *
     * @return (int) Columns
     */
    const unsigned int &columns() const noexcept {return m_columns;}

    /**
     * Accessor for element
     *
     * @param row Row index (the first row is zero)
     * @param column Column index (the first column is zero)
     * @return (T) content
     */
    virtual T operator()(
        const unsigned int &row,
        const unsigned int &column) const = 0;

    inline void check_index(
        const unsigned int &row,
        const unsigned int &column) const {
      if(row >= rows()){
        throw std::out_of_range("Row index incorrect");
      }else if(column >= columns()){
        throw std::out_of_range("Column index incorrect");
      }
    }

  protected:
    self_t &operator=(const self_t &array);
};

/**
 * @brief 2D array abstract class for changeable content
 *
 * @param T precision, for example, double
 */
template<class T, class ImplementedT>
class Array2D : public Array2D_Frozen<T> {
  public:
    typedef Array2D<T, ImplementedT> self_t;
    static const bool writable = true;

    /**
     * Constructor of Array2D
     *
     * @param rows Rows
     * @param columns Columns
     */
    Array2D(const unsigned int &rows, const unsigned int &columns) noexcept
        : Array2D_Frozen<T>(rows, columns){}

    /**
     * Destructor of Array2D
     */
    virtual ~Array2D(){}

    /**
     * Assigner, which performs shallow copy if possible.
     *
     * @param array another one
     * @return (ImplementedT)
     */
    virtual ImplementedT &operator=(const ImplementedT &array) = 0;

    /**
     * Accessor for element
     *
     * @param row Row index (the first row is zero)
     * @param column Column index (the first column is zero)
     * @return (T) content
     */
    using Array2D_Frozen<T>::operator();
    virtual T &operator()(
        const unsigned int &row,
        const unsigned int &column) = 0;
    
    /**
     * Perform zero clear
     *
     */
    virtual void clear() = 0;

    /**
     * Perform copy
     *
     * @param is_deep If true, return deep copy if possible, otherwise return shallow copy (just link).
     * @return (ImplementedT) Copy
     */
    virtual ImplementedT copy(const bool &is_deep = false) const = 0;
};

/**
 * @brief Array2D whose elements are dense, and are stored in sequential 1D array.
 * In other words, (i, j) element is mapped to [i * rows + j].
 *
 * @param T precision, for example, double
 */
template <class T>
class Array2D_Dense : public Array2D<T, Array2D_Dense<T> > {
  public:
    typedef Array2D_Dense<T> self_t;
    typedef Array2D<T, self_t> super_t;
    
    template <class T2>
    struct cast_t {
      typedef Array2D_Dense<T2> res_t;
    };

    using super_t::rows;
    using super_t::columns;

    typedef int ref_cnt_t;

  protected:
    static const int offset = (sizeof(T) >= sizeof(ref_cnt_t)) ? 1 : ((sizeof(ref_cnt_t) + sizeof(T) - 1) / sizeof(T));
    ref_cnt_t *ref_cnt;  ///< reference counter TODO alignment?
    T *values; ///< array for values

    template <class T2, bool do_memory_op = std::numeric_limits<T2>::is_specialized>
    struct setup_t {
      static void copy(Array2D_Dense<T2> &dest, const T2 *src){
        for(int i(dest.rows() * dest.columns() - 1); i >= 0; --i){
          dest.values[i] = src[i];
        }
      }
      static void clear(Array2D_Dense<T2> &target){
        for(int i(target.rows() * target.columns() - 1); i >= 0; --i){
          target.values[i] = T2();
        }
      }
    };
    template <class T2>
    struct setup_t<T2, true> {
      static void copy(Array2D_Dense<T2> &dest, const T2 *src){
        std::memcpy(dest.values, src, sizeof(T2) * dest.rows() * dest.columns());
      }
      static void clear(Array2D_Dense<T2> &target){
        std::memset(target.values, 0, sizeof(T2) * target.rows() * target.columns());
      }
    };

  public:
    Array2D_Dense() : super_t(0, 0), ref_cnt(NULL), values(NULL) {
    }

    /**
     * Constructor
     *
     * @param rows Rows
     * @param columns Columns
     */
    Array2D_Dense(
        const unsigned int &rows,
        const unsigned int &columns)
        : super_t(rows, columns),
        ref_cnt(reinterpret_cast<ref_cnt_t *>(new T[offset + (rows * columns)])),
        values(reinterpret_cast<T *>(ref_cnt) + offset) {
      *ref_cnt = 1;
    }
    /**
     * Constructor with initializer
     *
     * @param rows Rows
     * @param columns Columns
     * @param serialized Initializer
     */
    Array2D_Dense(
        const unsigned int &rows,
        const unsigned int &columns,
        const T *serialized)
        : super_t(rows, columns),
        ref_cnt(reinterpret_cast<ref_cnt_t *>(new T[offset + (rows * columns)])),
        values(reinterpret_cast<T *>(ref_cnt) + offset) {
      *ref_cnt = 1;
      setup_t<T>::copy(*this, serialized);
    }
    /**
     * Copy constructor, which performs shallow copy.
     *
     * @param array another one
     */
    Array2D_Dense(const self_t &array)
        : super_t(array.m_rows, array.m_columns),
          ref_cnt(array.ref_cnt), values(array.values){
      if(ref_cnt){++(*ref_cnt);}
    }
    /**
     * Constructor based on another type array, which performs deep copy.
     *
     * @param array another one
     */
    template <class T2>
    Array2D_Dense(const Array2D_Frozen<T2> &array)
        : super_t(array.rows(), array.columns()),
        ref_cnt(reinterpret_cast<ref_cnt_t *>(new T[offset + (array.rows() * array.columns())])),
        values(reinterpret_cast<T *>(ref_cnt) + offset) {
      *ref_cnt = 1;
      T *buf(values);
      const unsigned int i_end(array.rows()), j_end(array.columns());
      for(unsigned int i(0); i < i_end; ++i){
        for(unsigned int j(0); j < j_end; ++j){
          *(buf++) = array(i, j);
        }
      }
    }
    /**
     * Destructor
     *
     * The reference counter will be decreased, and when the counter equals to zero,
     * allocated memory for elements will be deleted.
     */
    ~Array2D_Dense(){
      if(ref_cnt && ((--(*ref_cnt)) <= 0)){
        delete [] reinterpret_cast<T *>(ref_cnt);
      }
    }

    /**
     * Assigner, which performs shallow copy.
     *
     * @param array another one
     * @return self_t
     */
    self_t &operator=(const self_t &array){
      if(this != &array){
        if(ref_cnt && ((--(*ref_cnt)) <= 0)){delete [] reinterpret_cast<T *>(ref_cnt);}
        super_t::m_rows = array.m_rows;
        super_t::m_columns = array.m_columns;
        if((ref_cnt = array.ref_cnt)){++(*ref_cnt);}
        values = array.values;
      }
      return *this;
    }

    /**
     * Assigner for different type, which performs deep copy.
     *
     * @param array another one
     * @return self_t
     */
    template <class T2>
    self_t &operator=(const Array2D_Frozen<T2> &array){
      return ((*this) = self_t(array));
    }
  protected:
    inline const T &get(
        const unsigned int &row,
        const unsigned int &column) const throws_when_debug {
#if defined(DEBUG)
      super_t::check_index(row, column);
#endif
      return values[(row * columns()) + column];
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

    void clear(){
      setup_t<T>::clear(*this);
    }

    /**
     * Perform copy
     *
     * @param is_deep If true, return deep copy, otherwise return shallow copy (just link).
     * @return (self_t) copy
     */
    self_t copy(const bool &is_deep = false) const {
      return is_deep ? self_t(rows(), columns(), values) : self_t(*this);
    }
};

/**
 * @brief special Array2D representing scaled unit
 *
 * @param T precision, for example, double
 */
template <class T>
class Array2D_ScaledUnit : public Array2D_Frozen<T> {
  public:
    typedef Array2D_ScaledUnit<T> self_t;
    typedef Array2D_Frozen<T> super_t;

  protected:
    const T value; ///< scaled unit

  public:
    /**
     * Constructor
     *
     * @param rows Rows
     * @param columns Columns
     */
    Array2D_ScaledUnit(const unsigned int &size, const T &v)
        : super_t(size, size), value(v){}

    /**
     * Accessor for element
     *
     * @param row Row index
     * @param column Column Index
     * @return (T) Element
     * @throw std::out_of_range When the indices are out of range
     */
    T operator()(
        const unsigned int &row, const unsigned int &column) const throws_when_debug {
#if defined(DEBUG)
      super_t::check_index(row, column);
#endif
      return (row == column) ? value : 0;
    }
};

/**
 * @brief special Array2D representing operation
 *
 * @param T precision, for example, double
 */
template <class T, class OperatorT>
struct Array2D_Operator : public Array2D_Frozen<T> {
  public:
    typedef OperatorT op_t;
    typedef Array2D_Operator<T, op_t> self_t;
    typedef Array2D_Frozen<T> super_t;

    const op_t op;

    /**
     * Constructor
     *
     * @param rows Rows
     * @param columns Columns
     */
    Array2D_Operator(
        const unsigned int &r, const unsigned int &c,
        const op_t &_op)
        : super_t(r, c), op(_op){}

    /**
     * Accessor for element
     *
     * @param row Row index
     * @param column Column Index
     * @return (T) Element
     * @throw std::out_of_range When the indices are out of range
     */
    T operator()(
        const unsigned int &row, const unsigned int &column) const throws_when_debug {
#if defined(DEBUG)
      super_t::check_index(row, colunmn);
#endif
      return op(row, column);
    }
};

template <class LHS_T, class RHS_T>
struct Array2D_Operator_Binary {
  typedef LHS_T first_t;
  typedef LHS_T lhs_t;
  typedef RHS_T rhs_t;
  lhs_t lhs; ///< Left hand side value
  rhs_t rhs; ///< Right hand side value
  Array2D_Operator_Binary(const lhs_t &_lhs, const rhs_t &_rhs) noexcept
      : lhs(_lhs), rhs(_rhs) {}
};

template <class LHS_T, class RHS_T>
struct Array2D_Operator_Multiply_by_Matrix;

template <class LHS_T, class RHS_T>
struct Array2D_Operator_Multiply_by_Scalar;

template <class LHS_T, class RHS_T, bool rhs_positive>
struct Array2D_Operator_Add;

template <class LHS_T, class RHS_T>
struct Array2D_Operator_EntrywiseMultiply;

template <class LHS_T, class RHS_T, bool rhs_horizontal>
struct Array2D_Operator_Stack;


template <class T>
struct MatrixValue_Special;

template <class T>
struct MatrixValue {
  template <class T2>
  struct check_complex_t {
    static const bool hit = false;
    typedef T2 r_t;
    typedef Complex<T2> c_t;
  };
  template <class T2>
  struct check_complex_t<Complex<T2> > {
    static const bool hit = true;
    typedef T2 r_t;
    typedef Complex<T2> c_t;
  };
  static const bool is_complex = check_complex_t<T>::hit;
  typedef typename check_complex_t<T>::r_t real_t;
  typedef typename check_complex_t<T>::c_t complex_t;
  static real_t get_real(const real_t &v) noexcept {
    return v;
  }
  static real_t get_real(const complex_t &v) noexcept {
    return v.real();
  }
  static complex_t get_sqrt(const real_t &v) noexcept {
    return (v >= 0) ? complex_t(std::sqrt(v)) : complex_t(0, std::sqrt(-v));
  }
  static complex_t get_sqrt(const complex_t &v) noexcept {
    return v.sqrt();
  }
  static real_t get_abs(const real_t &v) noexcept {
    return std::abs(v);
  }
  static real_t get_abs(const complex_t &v) noexcept {
    return v.abs();
  }
  static real_t get_abs2(const real_t &v) noexcept {
    return v * v;
  }
  static real_t get_abs2(const complex_t &v) noexcept {
    return v.abs2();
  }
  static bool is_nan_or_infinite(const real_t &v) noexcept {
#if defined(_MSC_VER)
    return _isnan(v) || !_finite(v);
#else
    return std::isnan(v) || !std::isfinite(v);
#endif
  }
  static bool is_nan_or_infinite(const complex_t &v) noexcept {
    return is_nan_or_infinite(v.real()) || is_nan_or_infinite(v.imaginary());
  }

  static typename MatrixValue_Special<T>::zero_t zero; // System-wise zero
};

template <class T>
struct MatrixValue_Special {
  typedef MatrixValue<T> v_t;
  struct wide_zero_t {
    typename v_t::real_t width, width_abs2;
    wide_zero_t(const typename v_t::real_t &width_)
        : width(v_t::get_abs(width_)), width_abs2(v_t::get_abs2(width_)) {}
    wide_zero_t &operator=(const typename v_t::real_t &width_) {
      width = v_t::get_abs(width_);
      width_abs2 = v_t::get_abs2(width_);
      return *this;
    }
    bool operator==(const typename v_t::real_t &v) const noexcept {return v_t::get_abs(v) <= width;}
    bool operator==(const typename v_t::complex_t &v) const noexcept {return v_t::get_abs2(v) <= width_abs2;}
    bool operator!=(const T &v) const noexcept {return !operator==(v);}
  };
  template <
      bool is_integer = std::numeric_limits<T>::is_integer,
      class U = void>
  struct zero_selector_t {
    typedef wide_zero_t res_t;
  };
  template <class U>
  struct zero_selector_t<true, U> {
    // When T is a kind of integer types, exact equalness is easiliy achievable.
    typedef T res_t;
  };
  typedef typename zero_selector_t<>::res_t zero_t;
};

template <class T>
typename MatrixValue_Special<T>::zero_t MatrixValue<T>::zero = 0;

#if 0 // If exact zero is required, then specialization resolves it.
template <>
struct MatrixValue_Special<double> {typedef double zero_t;};
#endif

template <class BaseView = void>
struct MatrixViewBase {
  typedef MatrixViewBase self_t;

  struct {} prop;

  static const char *name;
  template<class CharT, class Traits>
  friend std::basic_ostream<CharT, Traits> &operator<<(
      std::basic_ostream<CharT, Traits> &out, const self_t &view){
    return out << name;
  }

  inline const unsigned int rows(
      const unsigned int &_rows, const unsigned int &_columns) const noexcept {
    return _rows;
  }
  inline const unsigned int columns(
      const unsigned int &_rows, const unsigned int &_columns) const noexcept {
    return _columns;
  }
  template <class T, class Array2D_Type>
  inline T operator()(Array2D_Type &storage, const unsigned int &i, const unsigned int &j) const {
    return storage.Array2D_Type::operator()(i, j); // direct call instead of via vtable
  }

  void update_size(const unsigned int &rows, const unsigned int &columns){}
  void update_offset(const unsigned int &row, const unsigned int &column){}
  void update_loop(const unsigned int &rows, const unsigned int &columns){}
};
template <class BaseView>
const char *MatrixViewBase<BaseView>::name = "[Base]";

template <class BaseView>
struct MatrixViewTranspose;

template <class BaseView>
struct MatrixViewConjugate;

template <class BaseView>
struct MatrixViewSizeVariable;

template <class BaseView>
struct MatrixViewOffset;

template <class BaseView>
struct MatrixViewLoop;


template <class View>
struct MatrixViewProperty {
  typedef View self_t;
  static const bool anchor = true;
  static const bool viewless = false;
  static const bool transposed = false;
  static const bool conjugated = false;
  static const bool offset = false;
  static const bool variable_size = false;

  static const char *name;

  struct inspect_t {
    template<class CharT, class Traits>
    friend std::basic_ostream<CharT, Traits> &operator<<(
        std::basic_ostream<CharT, Traits> &out, const inspect_t &){
      return out << name;
    }
  };
  static inspect_t inspect(){return inspect_t();}
};
template <class View>
const char *MatrixViewProperty<View>::name = "";

template <class V1, template <class> class V2>
struct MatrixViewProperty<V2<V1> > {
  typedef V2<V1> self_t;
  static const bool anchor = false;

  template <template <class> class TargetView>
  struct check_of_t {
    template <template <class> class T, class U = void>
    struct check_t {
      template <bool is_next_anchor, class U2 = void>
      struct next_t {
        static const bool res = MatrixViewProperty<V1>::template check_of_t<TargetView>::res;
      };
      template <class U2>
      struct next_t<true, U2> {
        static const bool res = false;
      };
      static const bool res = next_t<MatrixViewProperty<V1>::anchor>::res;
    };
    template <class U>
    struct check_t<TargetView, U> {
        static const bool res = true;
    };
    static const bool res = check_t<V2>::res;
  };

  static const bool viewless = MatrixViewProperty<V2<void> >::template check_of_t<MatrixViewBase>::res;
  static const bool transposed = check_of_t<MatrixViewTranspose>::res;
  static const bool conjugated = check_of_t<MatrixViewConjugate>::res;
  static const bool offset = check_of_t<MatrixViewOffset>::res;
  static const bool variable_size = check_of_t<MatrixViewSizeVariable>::res;

  static const char *name;

  struct inspect_t {
    template<class CharT, class Traits>
    friend std::basic_ostream<CharT, Traits> &operator<<(
        std::basic_ostream<CharT, Traits> &out, const inspect_t &){
      if(MatrixViewProperty<V1>::anchor){
        return out << name;
      }else{
        return out << name << " " << MatrixViewProperty<V1>::inspect();
      }
    }
  };
  static inspect_t inspect(){return inspect_t();}
};
template <class V1, template <class> class V2>
const char *MatrixViewProperty<V2<V1> >::name = V2<MatrixViewBase<> >::name;

template <class View>
struct MatrixViewBuilder {
  typedef MatrixViewProperty<View> property_t;

  template <template <class> class V, class U = void>
  struct priority_t {
    static const int priority = -1;
  };
  enum {
    Offset = 0,
    Loop = Offset,
    SizeVariable,
    Transpose,
    Conjugate,
  };
#define make_priority_table(name) \
template <class U> \
struct priority_t<MatrixView ## name, U> { \
  static const int priority = name; \
}
  make_priority_table(Offset);
  make_priority_table(Loop);
  make_priority_table(SizeVariable);
  make_priority_table(Transpose);
  make_priority_table(Conjugate);
#undef make_priority_table

  template <template <class> class AddView, class BaseView = View>
  struct add_t {
    /* Add view in accordance with view priority
     * The first rule is that higher priority view is outer, lower one is inner.
     * The second rule is if two views have a same priority, then original oredr is kept.
     * For example;
     *   Adding "2-C" to "3 <2-A <2-B <1 <0> > > >" yields "3 <2-C <2-A <2-B <1 <0> > > > >"
     */
    typedef AddView<BaseView> res_t;
  };
  template <template <class> class V1, template <class> class V2, class V3>
  struct add_t<V1, V2<V3> > {
    template <bool is_V1_has_higher_priority, class U = void>
    struct rebuild_t {
      typedef V1<typename add_t<V2, V3>::res_t> res_t;
    };
    template <class U>
    struct rebuild_t<false, U> {
      typedef V2<typename add_t<V1, V3>::res_t> res_t;
    };
    typedef typename rebuild_t<
        (priority_t<V1>::priority >= priority_t<V2>::priority)>::res_t res_t;
  };

  template <template <class> class RemoveView>
  struct remove_t {
    template <class V>
    struct rebuild_t {
      template <class V1>
      struct check_t {
        typedef V1 res_t;
      };
      template <class V1, template <class> class V2>
      struct check_t<V2<V1> > {
        typedef V2<typename rebuild_t<V1>::res_t> res_t;
      };
      typedef typename check_t<V>::res_t res_t;
    };
    template <class V>
    struct rebuild_t<RemoveView<V> > {
      typedef typename rebuild_t<V>::res_t res_t;
    };
    typedef typename rebuild_t<View>::res_t res_t;
  };

  template <template <class> class SwitchView>
  struct switch_t { // off => on => off => ...
    template <class V>
    struct rebuild_t {
      typedef V res_t;
    };
    template <class V1, template <class> class V2>
    struct rebuild_t<V2<V1> > {
      typedef V2<typename rebuild_t<V1>::res_t> res_t;
    };
    template <class V>
    struct rebuild_t<SwitchView<SwitchView<V> > > {
      typedef typename rebuild_t<V>::res_t res_t;
    };
    typedef typename rebuild_t<typename add_t<SwitchView>::res_t>::res_t res_t;
  };

  template <template <class> class UniqueView>
  struct unique_t { // off => on => on => ...
    typedef typename MatrixViewBuilder<
        typename remove_t<UniqueView>::res_t> // remove all UniqueViews, then add an UniqueView
            ::template add_t<UniqueView>::res_t res_t;
  };

  template <template <class> class GroupView>
  struct group_t {
    /* If GroupViews are consecutive, then summarize them into one;
     * if no GroupViews, just add it
     */
    template <class V>
    struct rebuild_t {
      typedef V res_t;
    };
    template <class V1, template <class> class V2>
    struct rebuild_t<V2<V1> > {
      typedef V2<typename rebuild_t<V1>::res_t> res_t;
    };
    template <class V>
    struct rebuild_t<GroupView<GroupView<V> > > {
      typedef typename rebuild_t<GroupView<V> >::res_t res_t;
    };
    typedef typename rebuild_t<typename add_t<GroupView>::res_t>::res_t res_t;
  };

  typedef typename switch_t<MatrixViewTranspose>::res_t transpose_t;
  typedef typename switch_t<MatrixViewConjugate>::res_t conjugate_t;
  typedef typename group_t<MatrixViewOffset>::res_t offset_t;
  typedef typename unique_t<MatrixViewSizeVariable>::res_t size_variable_t;
  typedef typename group_t<MatrixViewLoop>::res_t loop_t;

  struct reverse_op_t {
    template <class V1, class V2>
    struct rebuild_t {
      typedef V2 res_t;
    };
    template <template <class> class V1, class V2, class V3>
    struct rebuild_t<V1<V2>, V3> {
      typedef typename rebuild_t<V2, V1<V3> >::res_t res_t;
    };
    typedef typename rebuild_t<View, void>::res_t res_t;
  };
  typedef typename reverse_op_t::res_t reverse_t;

  /**
   * Calculate view to which another view is applied
   * For example, current = [Transpose(2)] [Offset(0)],
   * and applied = [Transpose(2)] [VariableSize(1)],
   * then intermediate = [Transpose(2)] [Transpose(2)] [VariableSize(1)] [Offset(0)],
   * and finally, result = [VariableSize(1)] [Offset(0)].
   * Here, each view elements are applied in accordance with their characteristics.
   * @param SrcView view to be applied to current view
   */
  template <class SrcView>
  struct apply_t {
    template <class SrcView_Reverse, class DestView>
    struct next_t {
      typedef DestView res_t;
    };
    template <template <class> class V1, class V2, class DestView>
    struct next_t<V1<V2>, DestView> {
      typedef typename next_t<V2, DestView>::res_t res_t;
    };
#define make_entry(view_name, result_type) \
template <class V, class DestView> \
struct next_t<view_name<V>, DestView> { \
  typedef typename next_t<V, typename MatrixViewBuilder<DestView>::result_type>::res_t res_t; \
};
    make_entry(MatrixViewTranspose, transpose_t);
    make_entry(MatrixViewConjugate, conjugate_t);
    make_entry(MatrixViewOffset, offset_t);
    make_entry(MatrixViewSizeVariable, size_variable_t);
    make_entry(MatrixViewLoop, loop_t);
#undef make_entry
    typedef typename next_t<typename MatrixViewBuilder<SrcView>::reverse_t, View>::res_t res_t;
  };

  template <
      class DestView, class DestView_Reverse,
      class SrcView, class SrcView_Reverse>
  struct copy_t {
    template <class DestR = DestView_Reverse, class SrcR = SrcView_Reverse>
    struct downcast_copy_t {
      static void run(DestView *dest, const SrcView *src){}
    };
    template <
        template <class> class Dest_Src_R1,
        class DestR2, class SrcR2>
    struct downcast_copy_t<Dest_Src_R1<DestR2>, Dest_Src_R1<SrcR2> > {
      // catch if same views exist at the same stack level in both source and destination
      static void run(DestView *dest, const SrcView *src){
        std::memcpy(
            &static_cast<Dest_Src_R1<DestView> *>(dest)->prop,
            &static_cast<const Dest_Src_R1<SrcView> *>(src)->prop,
            sizeof(static_cast<Dest_Src_R1<DestView> *>(dest)->prop));
        copy_t<
            Dest_Src_R1<DestView>, DestR2,
            Dest_Src_R1<SrcView>, SrcR2>::run(
              static_cast<Dest_Src_R1<DestView> *>(dest),
              static_cast<const Dest_Src_R1<SrcView> *>(src));
      }
    };
    template <
        template <class> class DestR1, class DestR2,
        template <class> class SrcR1, class SrcR2>
    struct downcast_copy_t<DestR1<DestR2>, SrcR1<SrcR2> > {
      template<
          bool is_DestR1_higher = (priority_t<DestR1>::priority > priority_t<SrcR1>::priority),
          bool is_DestR1_lower = (priority_t<DestR1>::priority < priority_t<SrcR1>::priority)>
      struct next_t { // catch srcR1.priority == destR1.priority
        typedef DestR1<DestView> dest_t;
        typedef DestR2 dest_R_t;
        typedef SrcR1<SrcView> src_t;
        typedef SrcR2 src_R_t;
      };
      template<bool is_DestR1_lower>
      struct next_t<true, is_DestR1_lower> { // catch destR1.priority > srcR1.priority
        typedef DestView dest_t;
        typedef DestR1<DestR2> dest_R_t;
        typedef SrcR1<SrcView> src_t;
        typedef SrcR2 src_R_t;
      };
      template<bool is_DestR1_higher>
      struct next_t<is_DestR1_higher, true> { // catch destR1.priority < srcR1.priority
        typedef DestR1<DestView> dest_t;
        typedef DestR2 dest_R_t;
        typedef SrcView src_t;
        typedef SrcR1<SrcR2> src_R_t;
      };
      static void run(DestView *dest, const SrcView *src){
        copy_t<
            typename next_t<>::dest_t, typename next_t<>::dest_R_t,
            typename next_t<>::src_t, typename next_t<>::src_R_t>::run(
              static_cast<typename next_t<>::dest_t *>(dest),
              static_cast<const typename next_t<>::src_t *>(src));
      }
    };
    static void run(DestView *dest, const SrcView *src){
      downcast_copy_t<>::run(dest, src);
    }
  };

  /**
   * Copy views as much as possible even if source and destination views are different.
   * As the primary rule, the copy is performed from a lower view,
   * which means a nearer component from MatrixViewBase, to higher one.
   * The secondary rule is if both source and destination views have same component,
   * its property such as offset is copied, otherwise copy is skipped.
   *
   * For example, destination = [Transpose(2)] [VariableSize(1)] [Offset(0)], and
   * source = [Transpose(2)] [Offset(0)], then only properties of transpose and offset
   * are copied.
   *
   * @param dest Destination view
   * @param src Source view
   */
  template <class View2>
  static void copy(View &dest, const View2 &src){
    copy_t<
        void, reverse_t,
        void, typename MatrixViewBuilder<View2>::reverse_t>::run(&dest, &src);
  }
};

template <class BaseView>
struct MatrixViewTranspose : public BaseView {
  typedef MatrixViewTranspose<BaseView> self_t;

  struct {} prop;

  MatrixViewTranspose() : BaseView() {}
  MatrixViewTranspose(const self_t &view)
      : BaseView((const BaseView &)view) {}

  template <class View>
  friend struct MatrixViewBuilder;

  static const char *name;

  template<class CharT, class Traits>
  friend std::basic_ostream<CharT, Traits> &operator<<(
      std::basic_ostream<CharT, Traits> &out, const MatrixViewTranspose<BaseView> &view){
    return out << name << " " << (const BaseView &)view;
  }

  inline const unsigned int rows(
      const unsigned int &_rows, const unsigned int &_columns) const noexcept {
    return BaseView::columns(_rows, _columns);
  }
  inline const unsigned int columns(
      const unsigned int &_rows, const unsigned int &_columns) const noexcept {
    return BaseView::rows(_rows, _columns);
  }
  template <class T, class Array2D_Type>
  inline T operator()(
      Array2D_Type &storage, const unsigned int &i, const unsigned int &j) const {
    return BaseView::DELETE_IF_MSC(template) operator()<T, Array2D_Type>(storage, j, i);
  }

  void update_size(const unsigned int &rows, const unsigned int &columns){
    BaseView::update_size(columns, rows);
  }
  void update_offset(const unsigned int &row, const unsigned int &column){
    BaseView::update_offset(column, row);
  }
  void update_loop(const unsigned int &rows, const unsigned int &columns){
    BaseView::update_loop(columns, rows);
  }
};
template <class BaseView>
const char *MatrixViewTranspose<BaseView>::name = "[T]";

template <class BaseView>
struct MatrixViewConjugate : public BaseView {
  typedef MatrixViewConjugate<BaseView> self_t;

  struct {} prop;

  MatrixViewConjugate() : BaseView() {}
  MatrixViewConjugate(const self_t &view)
      : BaseView((const BaseView &)view) {}

  template <class View>
  friend struct MatrixViewBuilder;

  static const char *name;

  template<class CharT, class Traits>
  friend std::basic_ostream<CharT, Traits> &operator<<(
      std::basic_ostream<CharT, Traits> &out, const MatrixViewConjugate<BaseView> &view){
    return out << name << " " << (const BaseView &)view;
  }

  template <class T, class Array2D_Type>
  inline T operator()(
      Array2D_Type &storage, const unsigned int &i, const unsigned int &j) const {
    return BaseView::DELETE_IF_MSC(template) operator()<T, Array2D_Type>(storage, i, j).conjugate();
  }
};
template <class BaseView>
const char *MatrixViewConjugate<BaseView>::name = "[~]";

template <class BaseView>
struct MatrixViewOffset : public BaseView {
  typedef MatrixViewOffset<BaseView> self_t;

  struct {
    unsigned int row, column;
  } prop;

  MatrixViewOffset() : BaseView() {
    prop.row = prop.column = 0;
  }
  MatrixViewOffset(const self_t &view)
      : BaseView((const BaseView &)view), prop(view.prop) {
  }

  template <class View>
  friend struct MatrixViewBuilder;

  static const char *name;

  template<class CharT, class Traits>
  friend std::basic_ostream<CharT, Traits> &operator<<(
      std::basic_ostream<CharT, Traits> &out, const MatrixViewOffset<BaseView> &view){
    return out << name << "("
         << view.prop.row << ","
         << view.prop.column << ") "
         << (const BaseView &)view;
  }

  template <class T, class Array2D_Type>
  inline T operator()(
      Array2D_Type &storage, const unsigned int &i, const unsigned int &j) const {
    return BaseView::DELETE_IF_MSC(template) operator()<T, Array2D_Type>(
        storage, i + prop.row, j + prop.column);
  }

  void update_offset(const unsigned int &row, const unsigned int &column){
    prop.row += row;
    prop.column += column;
  }
};
template <class BaseView>
const char *MatrixViewOffset<BaseView>::name = "[Offset]";

template <class BaseView>
struct MatrixViewSizeVariable : public BaseView {
  typedef MatrixViewSizeVariable<BaseView> self_t;

  struct {
    unsigned int rows, columns;
  } prop;

  MatrixViewSizeVariable() : BaseView() {
    prop.rows = prop.columns = 0;
  }
  MatrixViewSizeVariable(const self_t &view)
      : BaseView((const BaseView &)view), prop(view.prop) {
  }

  template <class View>
  friend struct MatrixViewBuilder;

  static const char *name;

  template<class CharT, class Traits>
  friend std::basic_ostream<CharT, Traits> &operator<<(
      std::basic_ostream<CharT, Traits> &out, const MatrixViewSizeVariable<BaseView> &view){
    return out << name << "("
         << view.prop.rows << ","
         << view.prop.columns << ") "
         << (const BaseView &)view;
  }

  inline const unsigned int rows(
      const unsigned int &_rows, const unsigned int &_columns) const noexcept {
    return prop.rows;
  }
  inline const unsigned int columns(
      const unsigned int &_rows, const unsigned int &_columns) const noexcept {
    return prop.columns;
  }

  void update_size(const unsigned int &rows, const unsigned int &columns){
    prop.rows = rows;
    prop.columns = columns;
  }
};
template <class BaseView>
const char *MatrixViewSizeVariable<BaseView>::name = "[Size]";

template <class BaseView>
struct MatrixViewLoop : public BaseView {
  typedef MatrixViewLoop<BaseView> self_t;

  struct {
    unsigned int rows, columns;
  } prop;

  MatrixViewLoop() : BaseView() {
    prop.rows = prop.columns = 0;
  }
  MatrixViewLoop(const self_t &view)
      : BaseView((const BaseView &)view), prop(view.prop) {
  }

  template <class View>
  friend struct MatrixViewBuilder;

  static const char *name;

  template<class CharT, class Traits>
  friend std::basic_ostream<CharT, Traits> &operator<<(
      std::basic_ostream<CharT, Traits> &out, const MatrixViewLoop<BaseView> &view){
    return out << name << "("
         << view.prop.rows << ","
         << view.prop.columns << ") "
         << (const BaseView &)view;
  }

  template <class T, class Array2D_Type>
  inline T operator()(
      Array2D_Type &storage, const unsigned int &i, const unsigned int &j) const {
    return BaseView::DELETE_IF_MSC(template) operator()<T, Array2D_Type>(
        storage, i % prop.rows, j % prop.columns);
  }

  void update_loop(const unsigned int &rows, const unsigned int &columns){
    prop.rows = rows;
    prop.columns = columns;
  }
};
template <class BaseView>
const char *MatrixViewLoop<BaseView>::name = "[Loop]";


template <
    class T,
    class Array2D_Type = Array2D_Dense<T>,
    class ViewType = MatrixViewBase<> >
class Matrix;

template <class MatrixT>
struct MatrixBuilder_ViewTransformerBase;

template <
    template <class, class, class> class MatrixT,
    class T, class Array2D_Type, class ViewType>
struct MatrixBuilder_ViewTransformerBase<
    MatrixT<T, Array2D_Type, ViewType> > {
  typedef MatrixViewBuilder<ViewType> view_builder_t;

  typedef MatrixT<T, Array2D_Type,
      typename view_builder_t::transpose_t> transpose_t;
  typedef MatrixT<T, Array2D_Type,
      typename view_builder_t::size_variable_t> partial_offsetless_t;
  typedef MatrixT<T, Array2D_Type,
      typename MatrixViewBuilder<
        typename view_builder_t::offset_t>::size_variable_t> partial_t;
  typedef MatrixT<T, Array2D_Type,
      typename MatrixViewBuilder<
        typename view_builder_t::loop_t>::offset_t> circular_bijective_t;
  typedef MatrixT<T, Array2D_Type,
      typename MatrixViewBuilder<
        typename MatrixViewBuilder<
          typename view_builder_t::loop_t>::offset_t>::size_variable_t> circular_t;

  template <bool is_complex = MatrixValue<T>::is_complex, class U = void>
  struct check_complex_t {
    typedef MatrixT<T, Array2D_Type, ViewType> conjugate_t;
  };
  template <class U>
  struct check_complex_t<true, U> {
    typedef MatrixT<T, Array2D_Type, typename view_builder_t::conjugate_t> conjugate_t;
  };
  typedef typename check_complex_t<>::conjugate_t conjugate_t;

  template <class ViewType2>
  struct view_replace_t {
    typedef MatrixT<T, Array2D_Type, ViewType2> replaced_t;
  };
  template <class ViewType2>
  struct view_apply_t {
    typedef MatrixT<T, Array2D_Type,
        typename view_builder_t::template apply_t<ViewType2>::res_t> applied_t;
  };
};

template <class MatrixT>
struct MatrixBuilder_ViewTransformer
    : public MatrixBuilder_ViewTransformerBase<MatrixT> {};

template <template <class, class, class> class MatrixT, class T>
struct MatrixBuilder_ViewTransformer<MatrixT<T, Array2D_ScaledUnit<T>, MatrixViewBase<> > >
    : public MatrixBuilder_ViewTransformerBase<
        MatrixT<T, Array2D_ScaledUnit<T>, MatrixViewBase<> > > {
  typedef MatrixT<T, Array2D_ScaledUnit<T>, MatrixViewBase<> > transpose_t;
};

template <class MatrixT>
struct MatrixBuilder_ValueCopier {
  template <class T2, class Array2D_Type2, class ViewType2>
  static Matrix<T2, Array2D_Type2, ViewType2> &copy_value(
      Matrix<T2, Array2D_Type2, ViewType2> &dest, const MatrixT &src) {
    const unsigned int i_end(src.rows()), j_end(src.columns());
    for(unsigned int i(0); i < i_end; ++i){
      for(unsigned int j(0); j < j_end; ++j){
        dest(i, j) = (T2)(src(i, j));
      }
    }
    return dest;
  }
};

template <class MatrixT>
struct MatrixBuilder_Dependency;

template <class MatrixT>
struct MatrixBuilderBase
    : public MatrixBuilder_ViewTransformer<MatrixT>,
    public MatrixBuilder_ValueCopier<MatrixT>,
    public MatrixBuilder_Dependency<MatrixT> {};

template <class MatrixT>
struct MatrixBuilder : public MatrixBuilderBase<MatrixT> {};

template <
    template <class, class, class> class MatrixT,
    class T, class Array2D_Type, class ViewType>
struct MatrixBuilder_Dependency<MatrixT<T, Array2D_Type, ViewType> > {
  static const int row_buffer = 0; // unknown
  static const int column_buffer = 0;

  template <class T2, bool is_writable_array = Array2D_Type::writable>
  struct assignable_matrix_t {
    typedef Matrix<T2> res_t;
  };
  template <class T2>
  struct assignable_matrix_t<T2, true> {
    typedef Matrix<T2, typename Array2D_Type::template cast_t<T2>::res_t> res_t;
  };
  typedef typename assignable_matrix_t<T>::res_t assignable_t;

  template <class T2>
  struct cast_t {
    typedef typename MatrixBuilder<
        typename assignable_matrix_t<T2>::res_t>::assignable_t assignable_t;
  };

  template <int nR_add = 0, int nC_add = 0, int nR_multiply = 1, int nC_multiply = 1>
  struct resize_t {
    typedef typename MatrixBuilder_Dependency<MatrixT<T, Array2D_Type, ViewType> >::assignable_t assignable_t;
  };
};

template <class LHS_MatrixT, class RHS_T>
struct Matrix_multiplied_by_Scalar;

/**
 * @brief Matrix for unchangeable content
 *
 * @see Matrix
 */
template <
    class T, class Array2D_Type,
    class ViewType = MatrixViewBase<> >
class Matrix_Frozen {
  public:
    typedef Array2D_Type storage_t;
    typedef ViewType view_t;
    typedef Matrix_Frozen<T, Array2D_Type, ViewType> self_t;
    typedef self_t frozen_t;
    typedef MatrixValue<T> value_t;

    typedef MatrixBuilder<self_t> builder_t;

    template <class T2, class Array2D_Type2, class ViewType2>
    friend class Matrix_Frozen;

    template <class T2>
    static char (&check_storage(Array2D_Frozen<T2> *) )[1];
    static const int storage_t_should_be_derived_from_Array2D_Frozen
        = sizeof(check_storage(static_cast<storage_t *>(0)));

  protected:
    storage_t storage; ///< 2D storage
    view_t view;

    /**
     * Constructor without storage.
     *
     */
    Matrix_Frozen() : storage(), view(){}

    /**
     * Constructor with storage
     *
     * @param new_storage new storage
     */
    Matrix_Frozen(const storage_t &new_storage)
        : storage(new_storage),
        view() {}
  public:
    /**
     * Return row number.
     *
     * @return row number.
     */
    const unsigned int rows() const noexcept {
      return view.rows(storage.rows(), storage.columns());
    }

    /**
     * Return column number.
     *
     * @return column number.
     */
    const unsigned int columns() const noexcept {
      return view.columns(storage.rows(), storage.columns());
    }

    /**
     * Return matrix element of specified indices.
     *
     * @param row Row index starting from 0.
     * @param column Column index starting from 0.
     * @return element
     */
    T operator()(
        const unsigned int &row,
        const unsigned int &column) const {
      // T and Array2D_Type::content_t are not always identical,
      // for example, T = Complex<double>, Array2D_Type::content_t = double
      return (T)(view.DELETE_IF_MSC(template) operator()<
          typename Array2D_Type::content_t, const Array2D_Type>(storage, row, column));
    }

    template <class impl_t>
    class iterator_base_t {
      public:
        typedef impl_t self_type;
        typedef int difference_type;
        typedef std::random_access_iterator_tag iterator_category;
      protected:
        difference_type idx;
        unsigned int rows, columns;
        unsigned int r, c;
      public:
        const unsigned int &row() const {return r;}
        const unsigned int &column() const {return c;}

        // @see http://www.cplusplus.com/reference/iterator/
        // required for input iterator
        iterator_base_t(const self_t &mat, const difference_type &idx_ = 0)
            : idx(idx_), rows(mat.rows()), columns(mat.columns()), r(0), c(0) {}
        iterator_base_t()
            : idx(0), rows(0), columns(0), r(0), c(0) {}
        self_type &operator++() {return static_cast<self_type &>(*this).operator+=(1);}
        self_type operator++(int) {
          self_type res(static_cast<self_type &>(*this));
          static_cast<self_type &>(*this).operator++();
          return res;
        }
        friend bool operator==(const self_type &lhs, const self_type &rhs) {
          return lhs.idx == rhs.idx;
        }
        friend bool operator!=(const self_type &lhs, const self_type &rhs) {return !(lhs == rhs);}

        // required for forward iterator
        //iterator_t() // ctor.

        // required for bidirectional iterator
        self_type &operator--() {return static_cast<self_type &>(*this).operator+=(-1);}
        self_type operator--(int) {
          self_type res(static_cast<self_type &>(*this));
          static_cast<self_type &>(*this).operator--();
          return res;
        }

        // required for random access iterator
        friend bool operator<(const self_type &lhs, const self_type &rhs){
          return lhs.idx < rhs.idx;
        }
        friend bool operator>(const self_type &lhs, const self_type &rhs){
          return !((lhs < rhs) || (lhs == rhs));
        }
        friend bool operator<=(const self_type &lhs, const self_type &rhs){
          return !(lhs > rhs);
        }
        friend bool operator>=(const self_type &lhs, const self_type &rhs){
          return !(lhs < rhs);
        }

        // required for random access iterator
        self_type &operator+=(const difference_type &n){
          idx += n;
          return static_cast<self_type &>(*this);
        }
        friend self_type operator+(const self_type &lhs, const difference_type &n){
          return self_type(lhs) += n;
        }
        friend self_type operator+(const difference_type &n, const self_type &rhs){
          return rhs + n;
        }
        self_type &operator-=(const difference_type &n){
          return operator+=(-n);
        }
        friend self_type operator-(const self_type &lhs, const difference_type &n){
          return lhs + (-n);
        }
        friend self_type operator-(const difference_type &n, const self_type &rhs){
          return rhs - n;
        }
        friend difference_type operator-(const self_type &lhs, const self_type &rhs){
          return lhs.idx - rhs.idx;
        }

        self_type &head() {return static_cast<self_type &>(*this) -= idx;}
        self_type &tail() {return static_cast<self_type &>(*this) += (rows * columns - idx);}
    };

    struct iterator_mapper_t {
      template <class impl_t>
      class all_t : public iterator_base_t<impl_t> {
        protected:
          typedef iterator_base_t<impl_t> base_t;
          typename base_t::difference_type idx_max;
          void update(){
            if(base_t::idx <= 0){
              base_t::r = base_t::c = 0;
            }else if(base_t::idx >= idx_max){
              base_t::r = base_t::rows;
              base_t::c = 0;
            }else{
              std::div_t rc(std::div(base_t::idx, base_t::columns));
              base_t::r = (unsigned int)rc.quot;
              base_t::c = (unsigned int)rc.rem;
            }
          }
        public:
          all_t(const self_t &mat, const typename base_t::difference_type &idx_ = 0)
              : base_t(mat, idx_),
              idx_max((typename base_t::difference_type)(base_t::rows * base_t::columns)) {
            update();
          }
          all_t()
              : base_t(),
              idx_max((typename base_t::difference_type)(base_t::rows * base_t::columns)) {
            update();
          }
          impl_t &operator+=(const typename base_t::difference_type &n){
            base_t::operator+=(n);
            update();
            return static_cast<impl_t &>(*this);
          }
          using base_t::operator++;
          impl_t &operator++() {
            if(base_t::r < base_t::rows){
              if(++base_t::c == base_t::columns){
                ++base_t::r;
                base_t::c = 0;
              }
            }
            ++base_t::idx;
            return static_cast<impl_t &>(*this);
          }
          using base_t::operator--;
          impl_t &operator--() {
            if(base_t::idx-- > 0){
              if(base_t::c == 0){
                --base_t::r;
                base_t::c = base_t::columns;
              }
              --base_t::c;
            }
            return static_cast<impl_t &>(*this);
          }
      };
      template <class impl_t>
      class diagonal_t : public iterator_base_t<impl_t> {
        protected:
          typedef iterator_base_t<impl_t> base_t;
          typename base_t::difference_type idx_max;
          void update(){
            if(base_t::idx <= 0){
              base_t::idx = 0;
            }else if(base_t::idx >= idx_max){
              base_t::idx = idx_max;
            }
            base_t::r = base_t::c = base_t::idx;
          }
        public:
          diagonal_t(const self_t &mat, const typename base_t::difference_type &idx_ = 0)
              : base_t(mat, idx_),
              idx_max((typename base_t::difference_type)(
                (base_t::rows >= base_t::columns) ? base_t::columns : base_t::rows)) {
            update();
          }
          diagonal_t()
              : base_t(),
              idx_max((typename base_t::difference_type)(
                (base_t::rows >= base_t::columns) ? base_t::columns : base_t::rows)) {
            update();
          }
          impl_t &operator+=(const typename base_t::difference_type &n){
            base_t::operator+=(n);
            update();
            return static_cast<impl_t &>(*this);
          }
          impl_t &tail() {return static_cast<impl_t &>(*this) += (idx_max - base_t::idx);}
      };
      template <class impl_t>
      class offdiagonal_t : public iterator_base_t<impl_t> {
        protected:
          typedef iterator_base_t<impl_t> base_t;
          typename base_t::difference_type idx_max, offset;
          void setup(){
            idx_max = (typename base_t::difference_type)(base_t::rows * base_t::columns);
            if(idx_max == 0){
              offset = 0;
            }else if(base_t::rows >= base_t::columns){
              idx_max -= base_t::columns;
              offset = base_t::columns * (base_t::columns - 1);
            }else{
              idx_max -= base_t::rows;
              offset = idx_max;
            }
          }
          void update(){
            if(base_t::idx <= 0){
              base_t::idx = 0;
            }else if(base_t::idx >= idx_max){
              base_t::idx = idx_max;
            }
            if(base_t::idx >= offset){ // on row having no diagonal element
              std::div_t rc(std::div(base_t::idx - offset, base_t::columns));
              base_t::r = base_t::columns + (unsigned int)rc.quot;
              base_t::c = (unsigned int)rc.rem;
            }else{ // on row having a diagonal element
              std::div_t rc(std::div(base_t::idx, base_t::columns - 1));
              base_t::r = (unsigned int)rc.quot;
              base_t::c = (unsigned int)rc.rem;
              if(base_t::c >= base_t::r){++base_t::c;}
            }
          }
        public:
          offdiagonal_t(const self_t &mat, const typename base_t::difference_type &idx_ = 0)
              : base_t(mat, idx_) {
            setup();
            update();
          }
          offdiagonal_t() : base_t() {
            setup();
            update();
          }
          impl_t &operator+=(const typename base_t::difference_type &n){
            base_t::operator+=(n);
            update();
            return static_cast<impl_t &>(*this);
          }
          impl_t &tail() {return static_cast<impl_t &>(*this) += (idx_max - base_t::idx);}
      };
      template <bool is_lower, int right_shift = 0>
      struct triangular_t {
        template <class impl_t>
        class mapper_t : public iterator_base_t<impl_t> {
          protected:
            typedef iterator_base_t<impl_t> base_t;
            typename base_t::difference_type idx_max, offset1, offset2;
            void setup() {
              int shift(is_lower ? right_shift : -right_shift),
                  len1((int)(is_lower ? base_t::rows : base_t::columns)),
                  len2((int)(is_lower ? base_t::columns : base_t::rows));
              // calculation concept: (big triangle) - (2 small triangles if nesccesary)
              idx_max = offset1 = offset2 = 0;
              int triangle_len(shift + len1);
              if(triangle_len < 0){return;}
              if(shift > 0){ // exclude top left triangle
                int area_tl((shift + 1) * shift / 2);
                idx_max -= area_tl;
                if(is_lower){offset1 = area_tl;}
                else{offset2 = -shift * len1;}
              }
              idx_max += (triangle_len + 1) * triangle_len / 2; // add main triangle
              if(triangle_len > len2){ // exclude bottom right triangle
                int len_br(triangle_len - len2);
                int area_br((len_br + 1) * len_br / 2);
                idx_max -= area_br;
                if(is_lower){offset2 = -len_br * len2;}
                else{offset1 = area_br;}
              }
              offset2 += idx_max;
            }
            void update(){
              int idx(base_t::idx);
              if(idx < 0){
                idx = 0;
              }else if(idx > idx_max){
                idx = idx_max;
              }
              if(!is_lower){idx = idx_max - idx - 1;}
              if(idx <= offset2){ // in triangle
                idx += (offset1 + 1);
                int r_offset(std::ceil((std::sqrt((double)(8 * idx + 1)) - 1) / 2));
                base_t::r = r_offset;
                base_t::c = base_t::r + idx - (r_offset * (r_offset + 1) / 2);
                if(is_lower){
                  base_t::r -= (right_shift + 1);
                  base_t::c -= 1;
                }else{
                  base_t::r = (base_t::columns - right_shift) - base_t::r;
                  base_t::c = base_t::columns - base_t::c;
                }
              }else{ // in rectangle
                std::div_t rc(std::div(idx - offset2, base_t::columns));
                base_t::r = (unsigned int)rc.quot;
                base_t::c = (unsigned int)rc.rem;
                if(is_lower){
                  base_t::r += (base_t::columns - right_shift);
                }else{
                  base_t::r = -right_shift - base_t::r - 1;
                  base_t::c = base_t::columns - base_t::c - 1;
                }
              }
            }
          public:
            mapper_t(const self_t &mat, const typename base_t::difference_type &idx_ = 0)
                : base_t(mat, idx_) {
              setup();
              update();
            }
            mapper_t() : base_t() {
              setup();
              update();
            }
            impl_t &operator+=(const typename base_t::difference_type &n){
              base_t::operator+=(n);
              update();
              return static_cast<impl_t &>(*this);
            }
            impl_t &tail() {return static_cast<impl_t &>(*this) += (idx_max - base_t::idx);}
            using base_t::operator++;
            impl_t &operator++() {
              if(base_t::idx++ < idx_max){
                ++base_t::c;
                if(is_lower){
                  if((base_t::c == base_t::columns)
                      || ((int)base_t::c == right_shift + 1 + (int)base_t::r)){
                    ++base_t::r;
                    base_t::c = 0;
                  }
                }else{
                  if(base_t::c == base_t::columns){
                    ++base_t::r;
                    base_t::c = ((int)base_t::r < -right_shift)
                        ? 0
                        : (unsigned int)(right_shift + (int)base_t::r);
                  }
                }
              }
              return static_cast<impl_t &>(*this);
            }
            using base_t::operator--;
            impl_t &operator--() {
              if(base_t::idx-- > 0){
                if(is_lower){
                  if(base_t::c == 0){
                    base_t::c = (unsigned int)(right_shift + 1 + (int)(--base_t::r));
                    if(base_t::c > base_t::columns){base_t::c = base_t::columns;}
                  }
                }else{
                  if((base_t::c == 0)
                      || ((int)base_t::c == (right_shift + (int)base_t::r))){
                    --base_t::r;
                    base_t::c = base_t::columns;
                  }
                }
                --base_t::c;
              }
              return static_cast<impl_t &>(*this);
            }
        };
      };
#if defined(__cplusplus) && (__cplusplus >= 201103L)
#define MAKE_TRIANGULAR_ALIAS(name, is_lower, right_shift) \
template <class impl_t> using name \
    = typename triangular_t<is_lower, right_shift>::template mapper_t<impl_t>;
#else
#define MAKE_TRIANGULAR_ALIAS(name, is_lower, right_shift) \
template <class impl_t> \
struct name : public triangular_t<is_lower, right_shift>::template mapper_t<impl_t> { \
  typedef typename triangular_t<is_lower, right_shift>::template mapper_t<impl_t> base_t; \
  name(const self_t &mat, const typename base_t::difference_type &idx_ = 0) \
      : base_t(mat, idx_) {} \
  name() : base_t() {} \
};
#endif
      MAKE_TRIANGULAR_ALIAS(lower_triangular_t, true, 0);
      MAKE_TRIANGULAR_ALIAS(lower_triangular_offdiagonal_t, true, -1);
      MAKE_TRIANGULAR_ALIAS(upper_triangular_t, false, 0);
      MAKE_TRIANGULAR_ALIAS(upper_triangular_offdiagonal_t, false, 1);
#undef MAKE_TRIANGULAR_ALIAS
    };

    template <template <typename> class MapperT = iterator_base_t>
    class const_iterator_skelton_t : public MapperT<const_iterator_skelton_t<MapperT> > {
      public:
        typedef const T value_type;
        typedef const T reference;
        struct pointer {
          reference value;
          const T *operator->() const {return &value;}
        };
      protected:
        typedef MapperT<const_iterator_skelton_t<MapperT> > base_t;
        self_t mat;
      public:
        reference operator*() const {return mat(base_t::r, base_t::c);}
        pointer operator->() const {
          pointer p = {operator*()};
          return p;
        }
        const_iterator_skelton_t(const self_t &mat_, const typename base_t::difference_type &idx_ = 0)
            : base_t(mat_, idx_), mat(mat_) {}
        const_iterator_skelton_t()
            : base_t(), mat() {}
        reference operator[](const typename base_t::difference_type &n) const {
          return *((*this) + n);
        }
    };

    typedef const_iterator_skelton_t<iterator_mapper_t::template all_t> const_iterator;
    const_iterator begin() const {return const_iterator(*this);}
    const_iterator end() const {return const_iterator(*this).tail();}

    template <template <typename> class MapperT>
    const_iterator_skelton_t<MapperT> begin() const {
      return const_iterator_skelton_t<MapperT>(*this);
    }
    template <template <typename> class MapperT>
    const_iterator_skelton_t<MapperT> end() const {
      return const_iterator_skelton_t<MapperT>(*this).tail();
    }

    /**
     * Copy constructor generating shallow copy linking to source matrix
     *
     * @param another source matrix
     */
    Matrix_Frozen(const self_t &another)
        : storage(another.storage),
        view(another.view){}

		/**
		 * Constructor with different storage type
		 */
    template <class T2, class Array2D_Type2>
    Matrix_Frozen(const Matrix_Frozen<T2, Array2D_Type2, ViewType> &matrix)
        : storage(matrix.storage),
        view(matrix.view) {}
  protected:
    /**
     * Constructor with different view
     */
    template <class ViewType2>
    Matrix_Frozen(const Matrix_Frozen<T, Array2D_Type, ViewType2> &matrix)
        : storage(matrix.storage),
        view() {
      builder_t::view_builder_t::copy(view, matrix.view);
    }

  public:
    /**
     * Destructor
     */
    virtual ~Matrix_Frozen(){}

    typedef Matrix_Frozen<T, Array2D_ScaledUnit<T> > scalar_matrix_t;

    /**
     * Generate scalar matrix
     *
     * @param size Row and column number
     * @param scalar
     */
    static scalar_matrix_t getScalar(const unsigned int &size, const T &scalar){
      return scalar_matrix_t(typename scalar_matrix_t::storage_t(size, scalar));
    }

    /**
     * Generate unit matrix
     *
     * @param size Row and column number
     */
    static scalar_matrix_t getI(const unsigned int &size){
      return getScalar(size, T(1));
    }

    /**
     * Down cast by creating deep (totally unlinked to this) copy having changeable content
     *
     */
    operator typename builder_t::assignable_t() const {
      typedef typename builder_t::assignable_t res_t;
      res_t res(res_t::blank(rows(), columns()));
      builder_t::copy_value(res, *this);
      return res;
    }

  protected:
    /**
     * Assigner for subclass to modify storage and view
     * Its copy storategy is deoendent on storage assigner implementation.
     */
    self_t &operator=(const self_t &another){
      if(this != &another){
        storage = another.storage;
        view = another.view;
      }
      return *this;
    }
    /**
     * Assigner for subclass to modify storage and view with different storage type
     * Its copy storategy is deoendent on storage assigner implementation.
     */
    template <class T2, class Array2D_Type2>
    self_t &operator=(const Matrix_Frozen<T2, Array2D_Type2, ViewType> &matrix){
      storage = matrix.storage;
      view = matrix.view;
      return *this;
    }

  public:
    /**
     * Test whether elements are identical
     *
     * @param matrix Matrix to be compared
     * @return true when elements of two matrices are identical, otherwise false.
     */
    template <
        class T2, class Array2D_Type2,
        class ViewType2>
    bool operator==(const Matrix_Frozen<T2, Array2D_Type2, ViewType2> &matrix) const noexcept {
      if(static_cast<const void *>(this) == static_cast<const void *>(&matrix)){return true;}
      if((rows() != matrix.rows())
          || columns() != matrix.columns()){
        return false;
      }
      const unsigned int i_end(rows()), j_end(columns());
      for(unsigned int i(0); i < i_end; i++){
        for(unsigned int j(0); j < j_end; j++){
          if(value_t::zero != ((*this)(i, j) - matrix(i, j))){
            return false;
          }
        }
      }
      return true;
    }

    template <
        class T2, class Array2D_Type2,
        class ViewType2>
    bool operator!=(const Matrix_Frozen<T2, Array2D_Type2, ViewType2> &matrix) const noexcept {
      return !(operator==(matrix));
    }

    /**
     * Test whether matrix is square
     *
     * @return true when square, otherwise false.
     */
    bool isSquare() const noexcept {return rows() == columns();}

    /**
     * Test whether size of matrices is different
     *
     * @param matrix Matrix to be compared
     * @return true when size different, otherwise false.
     */
    template <class T2, class Array2D_Type2, class ViewType2>
    bool isDifferentSize(const Matrix_Frozen<T2, Array2D_Type2, ViewType2> &matrix) const noexcept {
      return (rows() != matrix.rows()) || (columns() != matrix.columns());
    }

    /**
     * Return trace of matrix
     *
     * @param do_check Check matrix size property. The default is true
     * @return Trace
     * @throw std::logic_error When matrix is not square
     */
    T trace(const bool &do_check = true) const {
      if(do_check && !isSquare()){throw std::logic_error("rows != columns");}
      T tr(0);
      for(unsigned i(0), i_end(rows()); i < i_end; i++){
        tr += (*this)(i, i);
      }
      return tr;
    }

    /**
     * Return sum of matrix elements
     *
     * @return Summation
     */
    T sum() const noexcept {
      T sum(0);
      for(unsigned int i(0), i_end(rows()); i < i_end; i++){
        for(unsigned int j(0), j_end(columns()); j < j_end; j++){
          sum += (*this)(i, j);
        }
      }
      return sum;
    }

  protected:
    template <class T2>
    bool isEqual_Block(
        const T2 &v,
        const bool &upper_offdiagonal = true, const bool &lower_offdiagonal = false,
        const bool &diagonal = false) const {
      const unsigned int i_end(rows()), j_end(columns());
      if(upper_offdiagonal){
        for(unsigned int i(0); i < i_end; i++){
          for(unsigned int j(i + 1); j < j_end; j++){
            if(v != (*this)(i, j)){return false;}
          }
        }
      }
      if(lower_offdiagonal){
        for(unsigned int i(1); i < i_end; i++){
          for(unsigned int j(0); j < i; j++){
            if(v != (*this)(i, j)){return false;}
          }
        }
      }
      if(diagonal){
        for(unsigned int i2(0), i2_end((i_end < j_end) ? i_end : j_end); i2 < i2_end; i2++){
          if(v != (*this)(i2, i2)){return false;}
        }
      }
      return true;
    }
  public:
    /**
     * Test whether matrix is diagonal
     *
     * @return true when diagonal matrix, otherwise false.
     */
    bool isDiagonal() const noexcept {
      return isSquare() && isEqual_Block(value_t::zero, true, true);
    }
    /**
     * Test whether matrix is lower triangular
     *
     * @return true when lower triangular matrix, otherwise false.
     */
    bool isLowerTriangular() const noexcept {
      return isSquare() && isEqual_Block(value_t::zero, true, false);
    }
    /**
     * Test whether matrix is upper triangular
     *
     * @return true when upper triangular matrix, otherwise false.
     */
    bool isUpperTriangular() const noexcept {
      return isSquare() && isEqual_Block(value_t::zero, false, true);
    }

    typedef typename builder_t::transpose_t transpose_t;
    /**
     * Generate transpose matrix
     *
     * @return Transpose matrix
     */
    transpose_t transpose() const noexcept {
      return transpose_t(*this);
    }

    typedef typename builder_t::conjugate_t conjugate_t;
    /**
     * Generate conjugate matrix
     *
     * @return Conjugate matrix
     */
    conjugate_t conjugate() const noexcept {
      return conjugate_t(*this);
    }

    typedef typename builder_t::transpose_t::builder_t::conjugate_t adjoint_t;
    /**
     * Generate adjoint matrix
     *
     * @return Adjoint matrix
     */
    adjoint_t adjoint() const noexcept {
      return adjoint_t(*this);
    }

  protected:
    template <class MatrixT>
    static typename MatrixBuilder<MatrixT>::partial_t partial_internal(
        const MatrixT &self,
        const unsigned int &new_rows,
        const unsigned int &new_columns,
        const unsigned int &row_offset,
        const unsigned int &column_offset){
      if(new_rows + row_offset > self.rows()){
        throw std::out_of_range("Row size exceeding");
      }else if(new_columns + column_offset > self.columns()){
        throw std::out_of_range("Column size exceeding");
      }
      typename MatrixBuilder<MatrixT>::partial_t res(self);
      res.view.update_size(new_rows, new_columns);
      res.view.update_offset(row_offset, column_offset);
      return res;
    }

  public:
    typedef typename builder_t::partial_t partial_t;
    /**
     * Generate partial matrix
     *
     * @param rowSize Row number
     * @param columnSize Column number
     * @param rowOffset Upper row index of original matrix for partial matrix
     * @param columnOffset Left column index of original matrix for partial matrix
     * @throw std::out_of_range When either row or column size exceeds original one
     * @return partial matrix
     *
     */
    partial_t partial(
        const unsigned int &new_rows,
        const unsigned int &new_columns,
        const unsigned int &row_offset,
        const unsigned int &column_offset) const {
      return partial_internal(*this,
          new_rows, new_columns, row_offset, column_offset);
    }

    /**
     * Generate row vector by using partial()
     *
     * @param row Row index of original matrix for row vector
     * @return Row vector
     * @see partial()
     */
    partial_t rowVector(const unsigned int &row) const {
      return partial(1, columns(), row, 0);
    }
    /**
     * Generate column vector by using partial()
     *
     * @param column Column index of original matrix for column vector
     * @return Column vector
     * @see partial()
     */
    partial_t columnVector(const unsigned int &column) const {
      return partial(rows(), 1, 0, column);
    }

  protected:
    template <class MatrixT>
    static typename MatrixBuilder<MatrixT>::partial_offsetless_t partial_internal(
        const MatrixT &self,
        const unsigned int &new_rows,
        const unsigned int &new_columns){
      if(new_rows > self.rows()){
        throw std::out_of_range("Row size exceeding");
      }else if(new_columns > self.columns()){
        throw std::out_of_range("Column size exceeding");
      }
      typename MatrixBuilder<MatrixT>::partial_offsetless_t res(self);
      res.view.update_size(new_rows, new_columns);
      return res;
    }

  public:
    typedef typename builder_t::partial_offsetless_t partial_offsetless_t;
    /**
     * Generate partial matrix by just reducing its size;
     * The origins and direction of original and return matrices are the same.
     *
     * @param rowSize Row number
     * @param columnSize Column number
     * @throw std::out_of_range When either row or column size exceeds original one
     * @return partial matrix
     */
    partial_offsetless_t partial(
        const unsigned int &new_rows,
        const unsigned int &new_columns) const {
      return partial_internal(*this, new_rows, new_columns);
    }

  protected:
    template <class MatrixT>
    static typename MatrixBuilder<MatrixT>::circular_t circular_internal(
        const MatrixT &self,
        const unsigned int &loop_rows,
        const unsigned int &loop_columns,
        const unsigned int &new_rows,
        const unsigned int &new_columns,
        const unsigned int &row_offset,
        const unsigned int &column_offset){
      if(loop_rows > self.rows()){
        throw std::out_of_range("Row loop exceeding");
      }else if(loop_columns > self.columns()){
        throw std::out_of_range("Column loop exceeding");
      }
      typename MatrixBuilder<MatrixT>::circular_t res(self);
      res.view.update_loop(loop_rows, loop_columns);
      res.view.update_size(new_rows, new_columns);
      res.view.update_offset(row_offset, column_offset);
      return res;
    }
  public:
    typedef typename builder_t::circular_t circular_t;
    /**
     * Generate matrix with circular view
     * "circular" means its index is treated with roll over correction, for example,
     * if specified index is 5 to a matrix of 4 rows, then it will be treated
     * as the same as index 1 (= 5 % 4) is selected.
     *
     * Another example; [4x3].circular(1,2,5,6) is
     *  00 01 02  =>  12 10 11 12 10 11
     *  10 11 12      22 20 21 22 20 21
     *  20 21 22      32 30 31 32 30 31
     *  30 31 32      02 00 01 02 00 01
     *                12 10 11 12 10 11
     *
     * @param row_offset Upper row index of original matrix for circular matrix
     * @param column_offset Left column index of original matrix for circular matrix
     * @param new_rows Row number
     * @param new_columns Column number
     * @throw std::out_of_range When either row or column loop exceeds original size
     * @return matrix with circular view
     */
    circular_t circular(
        const unsigned int &row_offset,
        const unsigned int &column_offset,
        const unsigned int &new_rows,
        const unsigned int &new_columns) const {
      return circular_internal(*this,
          rows(), columns(), new_rows, new_columns, row_offset, column_offset);
    }

  protected:
    template <class MatrixT>
    static typename MatrixBuilder<MatrixT>::circular_bijective_t circular_internal(
        const MatrixT &self,
        const unsigned int &row_offset,
        const unsigned int &column_offset) noexcept {
      typename MatrixBuilder<MatrixT>::circular_bijective_t res(self);
      res.view.update_loop(self.rows(), self.columns());
      res.view.update_offset(row_offset, column_offset);
      return res;
    }
  public:
    typedef typename builder_t::circular_bijective_t circular_bijective_t;
    /**
     * Generate matrix with circular view, keeping original size version.
     * For example, [4x3].circular(1,2) is
     *  00 01 02  =>  12 10 11
     *  10 11 12      22 20 21
     *  20 21 22      32 30 31
     *  30 31 32      02 00 01
     * In addition, this view is "bijective", that is, one-to-one mapping.
     *
     * @param row_offset Upper row index of original matrix for circular matrix
     * @param column_offset Left column index of original matrix for circular matrix
     * @return matrix with circular view
     * @see circular(
     *    const unsigned int &, const unsigned int &,
     *    const unsigned int &, const unsigned int &)
     */
    circular_bijective_t circular(
        const unsigned int &row_offset,
        const unsigned int &column_offset) const noexcept {
      return circular_internal(*this, row_offset, column_offset);
    }


    enum {
      OPERATOR_2_Generic,
      OPERATOR_2_Multiply_Matrix_by_Scalar,
      OPERATOR_2_Add_Matrix_to_Matrix,
      OPERATOR_2_Subtract_Matrix_from_Matrix,
      OPERATOR_2_Multiply_Matrix_by_Matrix,
      OPERATOR_2_Entrywise_Multiply_Matrix_by_Matrix,
      OPERATOR_2_Stack_Horizontal,
      OPERATOR_2_Stack_Vertical,
      OPERATOR_NONE,
    };

    template <class MatrixT = self_t>
    struct OperatorProperty {
      template <class U>
      struct check1_t {
        static const int tag = OPERATOR_NONE;
        typedef void operator_t;
      };
      template <class T2, class Array2D_Type2, class View_Type2>
      struct check1_t<Matrix_Frozen<T2, Array2D_Type2, View_Type2> > {
        template <class Array2D_Type3>
        struct check2_t {
          static const int tag = OPERATOR_NONE;
          typedef void operator_t;
        };
        template <class T2_op, class OperatorT>
        struct check2_t<Array2D_Operator<T2_op, OperatorT> >{
          static const int tag = OperatorT::tag;
          typedef OperatorT operator_t;
        };
        static const int tag = check2_t<Array2D_Type2>::tag;
        typedef typename check2_t<Array2D_Type2>::operator_t operator_t;
      };
      static const int tag = check1_t<MatrixT>::tag;
      typedef typename check1_t<MatrixT>::operator_t operator_t;
    };

    template <class LHS_MatrixT, class RHS_T>
    friend struct Matrix_multiplied_by_Scalar;

    template <class RHS_T, class LHS_MatrixT = self_t>
    struct Multiply_Matrix_by_Scalar {

      template <
          bool is_lhs_multi_mat_by_scalar
            = (OperatorProperty<LHS_MatrixT>::tag == OPERATOR_2_Multiply_Matrix_by_Scalar),
          class U = void>
      struct check_lhs_t {
        typedef Matrix_multiplied_by_Scalar<LHS_MatrixT, RHS_T> op_t;
        typedef typename op_t::mat_t res_t;
        static res_t generate(const LHS_MatrixT &mat, const RHS_T &scalar) noexcept {
          return op_t::generate(mat, scalar);
        }
      };
#if 1 // 0 = Remove optimization
      /*
       * Optimization policy: If upper pattern is M * scalar, then reuse it.
       * For example, (M * scalar) * scalar is transformed to M * (scalar * scalar).
       */
      template <class U>
      struct check_lhs_t<true, U> {
        typedef self_t res_t;
        static res_t generate(const LHS_MatrixT &mat, const RHS_T &scalar) noexcept {
          return res_t(
              typename res_t::storage_t(
                mat.rows(), mat.columns(),
                typename OperatorProperty<res_t>::operator_t(
                  mat.storage.op.lhs, mat.storage.op.rhs * scalar)));
        }
      };
#endif
      typedef typename check_lhs_t<>::res_t mat_t;
      static mat_t generate(const LHS_MatrixT &mat, const RHS_T &scalar) noexcept {
        return check_lhs_t<>::generate(mat, scalar);
      }
    };
    template <class RHS_T>
    struct Multiply_Matrix_by_Scalar<RHS_T, scalar_matrix_t> {
      typedef scalar_matrix_t mat_t;
      static mat_t generate(const self_t &mat, const RHS_T &scalar) noexcept {
        return getScalar(mat.rows(), mat(0, 0) * scalar);
      }
    };
    typedef Multiply_Matrix_by_Scalar<T> mul_mat_scalar_t;

    /**
     * Multiply matrix by scalar
     *
     * @param scalar
     * @return multiplied matrix
     */
    typename mul_mat_scalar_t::mat_t operator*(const T &scalar) const noexcept {
      return mul_mat_scalar_t::generate(*this, scalar);
    }

    /**
     * Multiply scalar by matrix
     *
     * @param scalar
     * @param matrix
     * @return multiplied matrix
     */
    friend typename mul_mat_scalar_t::mat_t operator*(const T &scalar, const self_t &matrix) noexcept {
      return matrix * scalar;
    }

    /**
     * Divide matrix by scalar
     *
     * @param scalar
     * @return divided matrix
     */
    typename mul_mat_scalar_t::mat_t operator/(const T &scalar) const noexcept {
      return operator*(T(1) / scalar);
    }

    /**
     * Unary minus operator, which is alias of matrix * -1.
     *
     * @return matrix * -1.
     */
    typename mul_mat_scalar_t::mat_t operator-() const noexcept {
      return operator*(-1);
    }


    template <class RHS_MatrixT, bool rhs_positive = true>
    struct Add_Matrix_to_Matrix {
      typedef Array2D_Operator_Add<self_t, RHS_MatrixT, rhs_positive> op_t;
      typedef Matrix_Frozen<T, Array2D_Operator<T, op_t> > mat_t;
      static mat_t generate(const self_t &mat1, const RHS_MatrixT &mat2){
        if(mat1.isDifferentSize(mat2)){throw std::invalid_argument("Incorrect size");}
        return mat_t(
            typename mat_t::storage_t(
              mat1.rows(), mat1.columns(), op_t(mat1, mat2)));
      }
    };

    /**
     * Add matrix to matrix
     *
     * @param matrix Matrix to add
     * @return added matrix
     * @throw std::invalid_argument When matrix sizes are not identical
     */
    template <class T2, class Array2D_Type2, class ViewType2>
    typename Add_Matrix_to_Matrix<Matrix_Frozen<T2, Array2D_Type2, ViewType2> >::mat_t
        operator+(const Matrix_Frozen<T2, Array2D_Type2, ViewType2> &matrix) const {
      return Add_Matrix_to_Matrix<Matrix_Frozen<T2, Array2D_Type2, ViewType2> >::generate(*this, matrix);
    }

    /**
     * Subtract matrix from matrix
     *
     * @param matrix Matrix to subtract
     * @return subtracted matrix
     * @throw std::invalid_argument When matrix sizes are not identical
     */
    template <class T2, class Array2D_Type2, class ViewType2>
    typename Add_Matrix_to_Matrix<Matrix_Frozen<T2, Array2D_Type2, ViewType2>, false>::mat_t
        operator-(const Matrix_Frozen<T2, Array2D_Type2, ViewType2> &matrix) const{
      return Add_Matrix_to_Matrix<Matrix_Frozen<T2, Array2D_Type2, ViewType2>, false>::generate(*this, matrix);
    }

    /**
     * Add scalar to matrix
     *
     * @param scalar scalar to add
     * @return added matrix
     * @throw std::invalid_argument When matrix sizes are not identical
     */
    typename Add_Matrix_to_Matrix<scalar_matrix_t>::mat_t
        operator+(const T &scalar) const {
      return *this + getScalar(rows(), scalar);
    }

    /**
     * Subtract scalar from matrix
     *
     * @param scalar scalar to subtract
     * @return subtracted matrix
     * @throw std::invalid_argument When matrix sizes are not identical
     */
    typename Add_Matrix_to_Matrix<scalar_matrix_t>::mat_t
        operator-(const T &scalar) const {
      return *this + (-scalar);
    }

    /**
     * Add matrix to scalar
     *
     * @param scalar scalar to be added
     * @param matrix matrix to add
     * @return added matrix
     * @throw std::invalid_argument When matrix sizes are not identical
     */
    friend typename Add_Matrix_to_Matrix<scalar_matrix_t>::mat_t
        operator+(const T &scalar, const self_t &matrix){
      return matrix + scalar;
    }

    /**
     * Subtract matrix from scalar
     *
     * @param scalar to be subtracted
     * @param matrix matrix to subtract
     * @return added matrix
     * @throw std::invalid_argument When matrix sizes are not identical
     */
    friend typename scalar_matrix_t::template Add_Matrix_to_Matrix<self_t, false>::mat_t
        operator-(const T &scalar, const self_t &matrix){
      return getScalar(matrix.rows(), scalar) - matrix;
    }

    template <class RHS_MatrixT>
    struct Entrywise_Multiply_Matrix_by_Matrix {
      typedef Array2D_Operator_EntrywiseMultiply<self_t, RHS_MatrixT> op_t;
      typedef Matrix_Frozen<T, Array2D_Operator<T, op_t> > mat_t;
      static mat_t generate(const self_t &mat1, const RHS_MatrixT &mat2){
        if(mat1.isDifferentSize(mat2)){throw std::invalid_argument("Incorrect size");}
        return mat_t(
            typename mat_t::storage_t(
              mat1.rows(), mat1.columns(), op_t(mat1, mat2)));
      }
    };

    /**
     * Entrywise product of two matrices
     *
     * @param matrix Matrix to multiply
     * @return entrywise multiplied matrix
     * @throw std::invalid_argument When matrix sizes are not identical
     */
    template <class T2, class Array2D_Type2, class ViewType2>
    typename Entrywise_Multiply_Matrix_by_Matrix<Matrix_Frozen<T2, Array2D_Type2, ViewType2> >::mat_t
        entrywise_product(const Matrix_Frozen<T2, Array2D_Type2, ViewType2> &matrix) const {
      return Entrywise_Multiply_Matrix_by_Matrix<Matrix_Frozen<T2, Array2D_Type2, ViewType2> >::generate(*this, matrix);
    }

    template <class RHS_MatrixT, bool rhs_horizontal = true>
    struct Stacked_Matrix {
      typedef Array2D_Operator_Stack<self_t, RHS_MatrixT, rhs_horizontal> op_t;
      typedef Matrix_Frozen<T, Array2D_Operator<T, op_t> > mat_t;
      static mat_t generate(const self_t &mat1, const RHS_MatrixT &mat2){
        if(rhs_horizontal ? (mat1.rows() > mat2.rows()) : (mat1.columns() > mat2.columns())){
          throw std::invalid_argument("Incorrect size");
        }
        return mat_t(
            typename mat_t::storage_t(
              mat1.rows() + (rhs_horizontal ? 0 : mat2.rows()),
              mat1.columns() + (rhs_horizontal ? mat2.columns() : 0),
              op_t(mat1, mat2)));
      }
    };

    /**
     * Horizontal stack of two matrices
     *
     * @param matrix Matrix to stack horizontally, i.e., right side
     * @return stacked matrix
     * @throw std::invalid_argument When matrix row number is smaller
     */
    template <class T2, class Array2D_Type2, class ViewType2>
    typename Stacked_Matrix<Matrix_Frozen<T2, Array2D_Type2, ViewType2>, true>::mat_t
        hstack(const Matrix_Frozen<T2, Array2D_Type2, ViewType2> &matrix) const {
      return Stacked_Matrix<Matrix_Frozen<T2, Array2D_Type2, ViewType2>, true>::generate(*this, matrix);
    }

    /**
     * Vertical stack of two matrices
     *
     * @param matrix Matrix to stack vertically, i.e., bottom side
     * @return stacked matrix
     * @throw std::invalid_argument When matrix row number is smaller
     */
    template <class T2, class Array2D_Type2, class ViewType2>
    typename Stacked_Matrix<Matrix_Frozen<T2, Array2D_Type2, ViewType2>, false>::mat_t
        vstack(const Matrix_Frozen<T2, Array2D_Type2, ViewType2> &matrix) const {
      return Stacked_Matrix<Matrix_Frozen<T2, Array2D_Type2, ViewType2>, false>::generate(*this, matrix);
    }

    /**
     * Test whether matrix is symmetric
     *
     * @return true when symmetric, otherwise false.
     */
    bool isSymmetric() const noexcept {
      return isSquare() && ((*this) - transpose()).isEqual_Block(value_t::zero);
    }
    /**
     * Test whether matrix is Hermitian
     *
     * @return true when Hermitian matrix, otherwise false.
     */
    bool isHermitian() const noexcept {
      return isSquare() && ((*this) - adjoint()).isEqual_Block(value_t::zero);
    }
    /**
     * Test whether matrix is skew-symmetric
     *
     * @return true when skew-symmetric matrix, otherwise false.
     */
    bool isSkewSymmetric() const noexcept {
      return isSquare() && ((*this) + transpose()).isEqual_Block(value_t::zero);
    }


    template <class LHS_T, class RHS_T>
    friend struct Array2D_Operator_Multiply_by_Matrix;

    template <class RHS_MatrixT, class LHS_MatrixT = self_t>
    struct Multiply_Matrix_by_Matrix {

      template <class MatrixT, int tag = OperatorProperty<MatrixT>::tag>
      struct check_t {
#if defined(_MSC_VER)
        /* work-around of MSVC bug for non-type template parameter
         * @see https://stackoverflow.com/questions/2763836/sfinae-failing-with-enum-template-parameter
         */
        template <bool is_operator2, class U = void>
        struct check_op_t {
          static const int complexity_linear = 1;
          static const int complexity_square = 1;
        };
        template <class U>
        struct check_op_t<true, U> {
          typedef typename OperatorProperty<MatrixT>::operator_t op_t;
          static const int complexity_lhs = check_t<op_t::lhs_t>::complexity;
          static const int complexity_rhs = check_t<op_t::rhs_t>::complexity;
          static const int complexity_linear = complexity_lhs + complexity_rhs;
          static const int complexity_square = (complexity_lhs + 1) * (complexity_rhs + 1);
        };
        static const bool is_operator2
            = (tag == OPERATOR_2_Multiply_Matrix_by_Scalar)
              || (tag == OPERATOR_2_Add_Matrix_to_Matrix)
              || (tag == OPERATOR_2_Subtract_Matrix_from_Matrix)
              || (tag == OPERATOR_2_Multiply_Matrix_by_Matrix)
              || (tag == OPERATOR_2_Entrywise_Multiply_Matrix_by_Matrix);
        static const int complexity
            = (tag == OPERATOR_2_Multiply_Matrix_by_Matrix)
                ? check_op_t<is_operator2>::complexity_square
                : check_op_t<is_operator2>::complexity_linear;
#else
        template <int tag2, class U = void>
        struct check_operator_t {
          static const int complexity = 1;
        };
        template <class U>
        struct check_operator_t<OPERATOR_2_Generic, U> {
          typedef check_t<typename OperatorProperty<MatrixT>::operator_t::lhs_t> lhs_t;
          typedef check_t<typename OperatorProperty<MatrixT>::operator_t::rhs_t> rhs_t;
          static const int complexity = lhs_t::complexity + rhs_t::complexity; // linear
        };
#define make_binary_item(tag_name) \
template <class U> struct check_operator_t<tag_name, U> \
    : public check_operator_t<OPERATOR_2_Generic, U>
        make_binary_item(OPERATOR_2_Multiply_Matrix_by_Scalar) {};
        make_binary_item(OPERATOR_2_Add_Matrix_to_Matrix) {};
        make_binary_item(OPERATOR_2_Subtract_Matrix_from_Matrix) {};
        make_binary_item(OPERATOR_2_Multiply_Matrix_by_Matrix) {
          typedef check_operator_t<OPERATOR_2_Generic, U> super_t;
          static const int complexity // square power
              = (super_t::lhs_t::complexity + 1) * (super_t::rhs_t::complexity + 1);
        };
        make_binary_item(OPERATOR_2_Entrywise_Multiply_Matrix_by_Matrix) {};
        // Stack_Horizontal/Vertical is intentionally ignored.
#undef make_binary_item
        static const int complexity = check_operator_t<tag>::complexity;
#endif
        static const bool is_multi_mat_by_scalar
            = (tag == OPERATOR_2_Multiply_Matrix_by_Scalar);
      };

      /*
       * [Optimization policy 1]
       * If each side includes complex calculation such as M * M, then use cache.
       * For example, (M * M) * M, and (M * M + M) * M use cache for the first parenthesis terms.
       * (M * M + M) * (M * M + M) uses cache for the first and second parenthesis terms.
       */
      template <class MatrixT, bool cache_on = (check_t<MatrixT>::complexity >= 3)>
      struct optimizer1_t {
        typedef MatrixT res_t;
      };
#if 1 // 0 = remove optimizer
      template <class MatrixT>
      struct optimizer1_t<MatrixT, true> {
        typedef typename MatrixT::builder_t::assignable_t res_t;
      };
#endif
      typedef typename optimizer1_t<self_t>::res_t lhs_buf_t;
      typedef typename optimizer1_t<RHS_MatrixT>::res_t rhs_buf_t;

      /*
       * [Optimization policy 2]
       * Sort lhs and rhs to make the scalar multiplication term last.
       */
      template <
          bool lhs_m_by_s = check_t<self_t>::is_multi_mat_by_scalar,
          bool rhs_m_by_s = check_t<RHS_MatrixT>::is_multi_mat_by_scalar,
          class U = void>
      struct optimizer2_t {
        // M * M
        typedef Array2D_Operator_Multiply_by_Matrix<lhs_buf_t, rhs_buf_t> op_t;
        typedef typename op_t::mat_t res_t;
        static res_t generate(const self_t &mat1, const RHS_MatrixT &mat2){
          return op_t::generate(mat1, mat2);
        }
      };
#if 1 // 0 = remove optimizer
      template <class U>
      struct optimizer2_t<true, false, U> {
        // (M * S) * M => (M * M) * S
        typedef typename OperatorProperty<self_t>::operator_t::lhs_t
            ::template Multiply_Matrix_by_Matrix<RHS_MatrixT> stage1_t;
        typedef typename stage1_t::mat_t
            ::template Multiply_Matrix_by_Scalar<
              typename OperatorProperty<self_t>::operator_t::rhs_t> stage2_t;
        typedef typename stage2_t::mat_t res_t;
        static res_t generate(const self_t &mat1, const RHS_MatrixT &mat2){
          return stage2_t::generate(
              stage1_t::generate(mat1.storage.op.lhs, mat2), mat1.storage.op.rhs);
        }
      };
      template <class U>
      struct optimizer2_t<false, true, U> {
        // M * (M * S) => (M * M) * S
        typedef typename self_t
            ::template Multiply_Matrix_by_Matrix<
              typename OperatorProperty<RHS_MatrixT>::operator_t::lhs_t> stage1_t;
        typedef typename stage1_t::mat_t
            ::template Multiply_Matrix_by_Scalar<
              typename OperatorProperty<RHS_MatrixT>::operator_t::rhs_t> stage2_t;
        typedef typename stage2_t::mat_t res_t;
        static res_t generate(const self_t &mat1, const RHS_MatrixT &mat2){
          return stage2_t::generate(
              stage1_t::generate(mat1, mat2.storage.op.lhs), mat2.storage.op.rhs);
        }
      };
      template <class U>
      struct optimizer2_t<true, true, U> {
        // (M * S) * (M * S) => (M * M) * (S * S)
        typedef typename OperatorProperty<self_t>::operator_t::lhs_t
            ::template Multiply_Matrix_by_Matrix<
              typename OperatorProperty<RHS_MatrixT>::operator_t::lhs_t> stage1_t;
        typedef typename stage1_t::mat_t
            ::template Multiply_Matrix_by_Scalar<
              typename OperatorProperty<self_t>::operator_t::rhs_t> stage2_t;
        typedef typename stage2_t::mat_t res_t;
        static res_t generate(const self_t &mat1, const RHS_MatrixT &mat2){
          return stage2_t::generate(
              stage1_t::generate(mat1.storage.op.lhs, mat2.storage.op.lhs),
              mat1.storage.op.rhs * mat2.storage.op.rhs);
        }
      };
#endif

      typedef typename optimizer2_t<>::res_t mat_t;
      static mat_t generate(const self_t &mat1, const RHS_MatrixT &mat2) noexcept {
        return optimizer2_t<>::generate(mat1, mat2);
      }
    };

    template <class RHS_MatrixT>
    struct Multiply_Matrix_by_Matrix<RHS_MatrixT, scalar_matrix_t> {
      // Specialization for (Scalar_M * M)
      typedef typename RHS_MatrixT::template Multiply_Matrix_by_Scalar<T, RHS_MatrixT> generator_t;
      typedef typename generator_t::mat_t mat_t;
      static mat_t generate(const self_t &mat1, const RHS_MatrixT &mat2) noexcept {
        return generator_t::generate(mat2, mat1(0, 0));
      }
    };

    /**
     * Multiply matrix by matrix
     * The following special cases are in consideration.
     * If this matrix is a scalar matrix, and if the right hand side matrix is also a scalar matrix,
     * multiplied scalar matrix will be returned; otherwise, then right hand side matrix
     * multiplied by this (as scalar) will be returned.
     * Only If the right hand side matrix is a scalar matrix,
     * matrix multiplied by scalar will be returned.
     * Otherwise, matrix * matrix will be returned.
     *
     * @param matrix matrix to multiply
     * @return multiplied matrix
     * @throw std::invalid_argument When operation is undefined
     */
    template <class T2, class Array2D_Type2, class ViewType2>
    typename Multiply_Matrix_by_Matrix<Matrix_Frozen<T2, Array2D_Type2, ViewType2> >::mat_t
        operator*(const Matrix_Frozen<T2, Array2D_Type2, ViewType2> &matrix) const {
      if(columns() != matrix.rows()){throw std::invalid_argument("Incorrect size");}
      return Multiply_Matrix_by_Matrix<Matrix_Frozen<T2, Array2D_Type2, ViewType2> >::generate(*this, matrix);
    }

    /**
     * Test whether matrix is normal
     *
     * @return true when normal matrix, otherwise false.
     */
    bool isNormal() const noexcept {
      return isSquare()
          && ((*this) * adjoint() - adjoint() * (*this)).isEqual_Block(value_t::zero, true, false, true);
    }
    /**
     * Test whether matrix is orthogonal
     *
     * @return true when orthogonal matrix, otherwise false.
     */
    bool isOrthogonal() const noexcept {
      return isSquare()
          && ((*this) * transpose() - 1).isEqual_Block(value_t::zero, true, false, true)
          && (transpose() * (*this) - 1).isEqual_Block(value_t::zero, true, false, true);
    }
    /**
     * Test whether matrix is unitary
     *
     * @return true when unitary matrix, otherwise false.
     */
    bool isUnitary() const noexcept {
      return isSquare()
          && ((*this) * adjoint() - 1).isEqual_Block(value_t::zero, true, false, true)
          && (adjoint() * (*this) - 1).isEqual_Block(value_t::zero, true, false, true);
    }

    /**
     * Generate a matrix in which i-th row and j-th column are removed
     * to calculate first-minor (第一小行列式、1行1列ずつ取り除いた行列の行列式)
     *
     * @param row Row to be removed
     * @param column Column to be removed
     * @return Removed matrix
     */
    typename builder_t::template resize_t<-1, -1>::assignable_t matrix_for_minor(
        const unsigned int &row,
        const unsigned int &column) const noexcept {
      typedef typename builder_t::template resize_t<-1, -1>::assignable_t res_t;
      res_t res(res_t::blank(rows() - 1, columns() - 1));
#if 0
      unsigned int i(0), i2(0);
      const unsigned int i_end(res.rows()), j_end(res.columns());
      for( ; i < row; ++i, ++i2){
        unsigned int j(0), j2(0);
        for( ; j < column; ++j, ++j2){
          res(i, j) = operator()(i2, j2);
        }
        ++j2;
        for( ; j < j_end; ++j, ++j2){
          res(i, j) = operator()(i2, j2);
        }
      }
      ++i2;
      for( ; i < i_end; ++i, ++i2){
        unsigned int j(0), j2(0);
        for( ; j < column; ++j, ++j2){
          res(i, j) = operator()(i2, j2);
        }
        ++j2;
        for( ; j < j_end; ++j, ++j2){
          res(i, j) = operator()(i2, j2);
        }
      }
#else
      // equivalent version to use circular view
      res.circular(row, column).replace(circular(row + 1, column + 1, rows() - 1, columns() - 1), false);
#endif
      return res;
    }

    /**
     * Calculate determinant by using first-minor (slow algorithm)
     *
     * @param do_check Whether check size property. The default is true.
     * @return Determinant
     * @throw std::logic_error When operation is undefined
     */
    T determinant_minor(const bool &do_check = true) const {
      if(do_check && !isSquare()){throw std::logic_error("rows() != columns()");}
      switch(rows()){
        case 1: return (*this)(0, 0);
        case 2: return (*this)(0, 0) * (*this)(1, 1) - (*this)(0, 1) * (*this)(1, 0);
        case 3:
          return (*this)(0, 0) * (*this)(1, 1) * (*this)(2, 2)
              + (*this)(0, 1) * (*this)(1, 2) * (*this)(2, 0)
              + (*this)(0, 2) * (*this)(1, 0) * (*this)(2, 1)
              - (*this)(0, 0) * (*this)(1, 2) * (*this)(2, 1)
              - (*this)(0, 1) * (*this)(1, 0) * (*this)(2, 2)
              - (*this)(0, 2) * (*this)(1, 1) * (*this)(2, 0);
        default: {
          T sum(0);
          T sign(1);
          for(unsigned int i(0), i_end(rows()); i < i_end; i++){
            if(value_t::zero != (*this)(i, 0)){
              sum += (*this)(i, 0) * (matrix_for_minor(i, 0).determinant_minor(false)) * sign;
            }
            sign = -sign;
          }
          return sum;
        }
      }
    }

    /**
     * Perform LUP decomposition
     * Return matrix is
     * (0, 0)-(n-1, n-1):  L matrix
     * (0, n)-(n-1, 2n-1): U matrix
     *
     * @param pivot_num Number of pivoting to be returned
     * @param pivot Array of pivoting indices to be returned, NULL is acceptable (no return).
     * For example, [0,2,1] means the left hand side pivot matrix,
     * which multiplies original matrix (not to be multiplied), equals to
     * [[1,0,0], [0,0,1], [0,1,0]].
     * @param do_check Check size, the default is true.
     * @return LU decomposed matrix
     * @throw std::logic_error When operation is undefined
     * @throw std::runtime_error When operation is unavailable
     */
    typename builder_t::template resize_t<0, 0, 1, 2>::assignable_t decomposeLUP(
        unsigned int &pivot_num,
        unsigned int *pivot = NULL,
        const bool &do_check = true) const {
      if(do_check && !isSquare()){throw std::logic_error("rows() != columns()");}

      typedef typename builder_t::template resize_t<0, 0, 1, 2>::assignable_t res_t;
      res_t LU(res_t::blank(rows(), columns() * 2));

      typename res_t::partial_offsetless_t L(LU.partial(rows(), columns()));
      typename res_t::partial_t U(LU.partial(rows(), columns(), 0, columns()));
      const unsigned int rows_(rows());
      for(unsigned int i(0); i < rows_; ++i){
        U(i, i) = (*this)(i, i);
        L(i, i) = T(1);
        for(unsigned int j(i + 1); j < rows_; ++j){
          U(i, j) = (*this)(i, j);
          U(j, i) = (*this)(j, i); // U is full copy
          L(i, j) = T(0);
        }
      }
      pivot_num = 0;
      if(pivot){
        for(unsigned int i(0); i < rows_; ++i){
          pivot[i] = i;
        }
      }
      // apply Gaussian elimination
      for(unsigned int i(0); i < rows_; ++i){
        if(value_t::zero == U(i, i)){ // check (i, i) is not zero
          unsigned int j(i);
          do{
            if(++j == rows_){
              throw std::runtime_error("LU decomposition cannot be performed");
            }
          }while(value_t::zero == U(i, j));
          for(unsigned int i2(0); i2 < rows_; ++i2){ // swap i-th and j-th columns
            T temp(U(i2, i));
            U(i2, i) = U(i2, j);
            U(i2, j) = temp;
          }
          pivot_num++;
          if(pivot){
            unsigned int temp(pivot[i]);
            pivot[i] = pivot[j];
            pivot[j] = temp;
          }
        }
        for(unsigned int i2(i + 1); i2 < rows_; ++i2){
          L(i2, i) = U(i2, i) / U(i, i);
          U(i2, i) = T(0);
          for(unsigned int j2(i + 1); j2 < rows_; ++j2){
            U(i2, j2) -= L(i2, i) * U(i, j2);
          }
        }
      }
      return LU;
    }

    void decomposeLUP_property(
        unsigned int &rank, T &determinant, unsigned int &pivot_num,
        const bool &do_check = true) const {
      // Algorithm is the same as decomposeLUP, but L matrix is not calculated
      // because rank/determinant can be obtained throught calculation of lower triangular of U.
      if(do_check && !isSquare()){throw std::logic_error("rows() != columns()");}

      typename builder_t::assignable_t U(this->operator typename builder_t::assignable_t()); // copy
      const unsigned int rows_(rows());
      pivot_num = 0;
      determinant = T(1);

      // apply Gaussian elimination
      for(unsigned int i(0); i < rows_; ++i){
        if(value_t::zero == U(i, i)){ // check (i, i) is not zero
          unsigned int j(i);
          do{
            if(++j == rows_){
              rank = i;
              return;
            }
          }while(value_t::zero == U(i, j));
          for(unsigned int i2(i); i2 < rows_; ++i2){ // swap i-th and j-th columns
            T temp(U(i2, i));
            U(i2, i) = U(i2, j);
            U(i2, j) = temp;
          }
          pivot_num++;
        }
#if 0
        for(unsigned int i2(i + 1); i2 < rows_; ++i2){
          T L_i2_i(U(i2, i) / U(i, i)); // equivalent to L(i2, i) = U(i2, i) / U(i, i); skip U(i2, i) = T(0);
          for(unsigned int j2(i + 1); j2 < rows_; ++j2){
            U(i2, j2) -= L_i2_i * U(i, j2);
          }
        }
        determinant *= U(i, i);
#else
        // integer preservation algorithm (Bareiss)
        for(unsigned int i2(i + 1); i2 < rows_; ++i2){
          for(unsigned int j2(i + 1); j2 < rows_; ++j2){
            ((U(i2, j2) *= U(i, i)) -= U(i2, i) * U(i, j2)) /= determinant;
          }
        }
        determinant = U(i, i);
#endif
      }
      rank = rows_;
      determinant *= ((pivot_num % 2 == 0) ? 1 : -1);
    }

    typename builder_t::template resize_t<0, 0, 1, 2>::assignable_t decomposeLU(
        const bool &do_check = true) const {
      unsigned int pivot_num;
      return decomposeLUP(pivot_num, NULL, do_check);
    }

    /**
     * Test whether matrix is LU decomposed.
     * The assumption of elements is
     * (0, 0)-(n-1, n-1):  L matrix
     * (0, n)-(n-1, 2n-1): U matrix
     *
     * @return true when LU, otherwise false.
     */
    bool isLU() const noexcept {
      const unsigned int r(rows());
      return (r * 2 == columns())
          && partial(r, r).isLowerTriangular()
          && partial(r, r, 0, r).isUpperTriangular();
    }

    /**
     * Resolve x of (Ax = y), where this matrix is A and has already been decomposed as LU.
     *
     * @param y Right hand term
     * @param do_check Check whether already LU decomposed
     * @return Left hand second term
     * @throw std::logic_error When operation is undefined
     * @throw std::invalid_argument When input is incorrect
     * @see decomposeLU(const bool &)
     */
    template <class T2, class Array2D_Type2, class ViewType2>
    typename Matrix_Frozen<T2, Array2D_Type2, ViewType2>::builder_t::assignable_t
        solve_linear_eq_with_LU(
            const Matrix_Frozen<T2, Array2D_Type2, ViewType2> &y, const bool &do_check = true)
            const {
      if(do_check && (!isLU())){
        throw std::logic_error("Not LU decomposed matrix!!");
      }
      if(do_check && ((y.columns() != 1) || (y.rows() != rows()))){
        throw std::invalid_argument("Incorrect y size");
      }

      typename builder_t::partial_offsetless_t L(partial(rows(), rows()));
      typename builder_t::partial_t U(partial(rows(), rows(), 0, rows()));
      typedef typename Matrix_Frozen<T2, Array2D_Type2, ViewType2>::builder_t::assignable_t y_t;
      // By using L(Ux) = y, firstly y' = (Ux) will be solved; L(Ux) = y で y' = (Ux)をまず解く
      y_t y_copy(y.operator y_t());
      y_t y_prime(y_t::blank(y.rows(), 1));
      for(unsigned i(0), rows_(rows()); i < rows_; i++){
        y_prime(i, 0) = y_copy(i, 0) / L(i, i);
        for(unsigned j(i + 1); j < rows_; j++){
          y_copy(j, 0) -= L(j, i) * y_prime(i, 0);
        }
      }

      // Then, Ux = y' gives solution of x; 続いてUx = y'で xを解く
      y_t x(y_t::blank(y.rows(), 1));
      for(unsigned i(rows()); i > 0;){
        i--;
        x(i, 0) = y_prime(i, 0) / U(i, i);
        for(unsigned j(i); j > 0;){
          j--;
          y_prime(j, 0) -= U(j, i) * x(i, 0);
        }
      }

      return x;
    }

    /**
     * Calculate determinant by using LU decomposition
     *
     * @param do_check Whether check size property. The default is true.
     * @return Determinant
     */
    T determinant_LU(const bool &do_check = true) const {
      unsigned int pivot_num;
      typename builder_t::template resize_t<0, 0, 1, 2>::assignable_t LU(
          decomposeLUP(pivot_num, NULL, do_check));
      T res((pivot_num % 2 == 0) ? 1 : -1);
      for(unsigned int i(0), i_end(rows()), j(rows()); i < i_end; ++i, ++j){
        res *= LU(i, i) * LU(i, j);
      }
      return res;
    }

    /**
     * Calculate determinant by using LU decomposition (faster algorithm)
     *
     * @param do_check Whether check size property. The default is true.
     * @return Determinant
     */
    T determinant_LU2(const bool &do_check = true) const {
      unsigned int pivot_num, rank;
      T res;
      decomposeLUP_property(rank, res, pivot_num, do_check);
      if(rank != rows()){
        throw std::runtime_error("LU decomposition cannot be performed");
      }
      return res;
    }

    T determinant(const bool &do_check = true) const {
      return determinant_LU2(do_check);
    }

    /**
     * Calculate rank by using LU decomposition
     *
     * @param do_check Whether check size property. The default is true.
     * @return rank
     */
    unsigned int rank_LU2(const bool &do_check = true) const {
      unsigned int pivot_num, res;
      T det;
      decomposeLUP_property(res, det, pivot_num, do_check);
      return res;
    }

    unsigned int rank(const bool &do_check = true) const {
      return rank_LU2(do_check);
    }

    /**
     * Calculate cofactor (余因子), i.e.,
     * determinant of smaller square matrix removing specified one row and column
     * fro original matrix.
     *
     * @param row
     * @param column
     * @param do_check check size (true) or not (false). The default is true.
     * @return Cofactor
     */
    T cofactor(
        const unsigned int &row, const unsigned int &column,
        const bool &do_check = true) const {
      if(do_check && !isSquare()){throw std::logic_error("rows != columns");}
      if((row >= rows()) || (column >= columns())){
        throw std::out_of_range("incorrect row and/or column indices");
      }
      // circular.determinant is equivalent to matrix_for_minor.determinant
      // in terms of absolute values (polarity should be taken care of.)
      //return matrix_for_minor(row, column).determinant(false) * (((row + column) % 2 == 0) ? 1 : -1);
      return circular(row + 1, column + 1, rows() - 1, columns() - 1).determinant(false)
          * (((rows() % 2 == 1) || ((row + column) % 2 == 0)) ? 1 : -1);
    }

    /**
     * Return adjugate (余因子行列), i.e. transposed cofactor matrix.
     * X * adjugate(X) = det(X) I
     *
     * @param do_check check size (true) or not (false). The default is true.
     * @return Adjugate
     */
    typename builder_t::assignable_t adjugate(const bool &do_check = true) const {
      if(do_check && !isSquare()){throw std::logic_error("rows != columns");}
      typename builder_t::assignable_t res(builder_t::assignable_t::blank(rows(), columns()));
      for(unsigned int i(0), i_end(rows()); i < i_end; ++i){
        for(unsigned int j(0), j_end(columns()); j < j_end; ++j){
          res(i, j) = cofactor(j, i, false);
        }
      }
      return res;
    }

    /**
     * Perform UD decomposition
     * Return matrix is
     * (0, 0)-(n-1,n-1):  U matrix
     * (0, n)-(n-1,2n-1): D matrix
     *
     * @param do_check Check size, the default is true.
     * @return UD decomposed matrix
     * @throw std::logic_error When operation is undefined
     */
    typename builder_t::template resize_t<0, 0, 1, 2>::assignable_t decomposeUD(
        const bool &do_check = true) const {
      if(do_check && !isSymmetric()){throw std::logic_error("not symmetric");}
      typename builder_t::assignable_t P(this->operator typename builder_t::assignable_t());
      typedef typename builder_t::template resize_t<0, 0, 1, 2>::assignable_t res_t;
      res_t UD(rows(), columns() * 2);
      typename res_t::partial_offsetless_t U(UD.partial(rows(), columns()));
      typename res_t::partial_t D(UD.partial(rows(), columns(), 0, columns()));
      for(int i(rows() - 1); i >= 0; i--){
        D(i, i) = P(i, i);
        U(i, i) = T(1);
        for(int j(0); j < i; j++){
          U(j, i) = P(j, i) / D(i, i);
          for(int k(0); k <= j; k++){
            P(k, j) -= U(k, i) * D(i, i) * U(j, i);
          }
        }
      }
      return UD;
    }

    template <class MatrixT = self_t, class U = void>
    struct Inverse_Matrix {
      typedef typename MatrixT::builder_t::assignable_t mat_t;
      static mat_t generate(const MatrixT &mat) {
        if(!mat.isSquare()){throw std::logic_error("rows() != columns()");}

#if 0
        // Cramer (slow); クラメール
        T det(mat.determinant(false));
        if(value_t::zero == det){throw std::runtime_error("Operation void!!");}
        return mat.adjugate() / det;
#endif

        // Gaussian elimination; ガウス消去法
        mat_t left(mat.operator mat_t());
        mat_t right(getI(mat.rows()));
        const unsigned int rows_(left.rows()), columns_(left.columns());
        for(unsigned int i(0); i < rows_; i++){
          if(value_t::zero == left(i, i)){
            unsigned int i2(i);
            do{
              if(++i2 == rows_){
                throw std::runtime_error("invert matrix not exist");
              }
            }while(value_t::zero == left(i2, i));
            // swap i-th and i2-th rows
            for(unsigned int j(i); j < columns_; ++j){
              T temp(left(i, j));
              left(i, j) = left(i2, j);
              left(i2, j) = temp;
            }
            right.swapRows(i, i2);
          }
          if(value_t::zero != (left(i, i) - 1)){
            for(unsigned int j(0); j < columns_; j++){right(i, j) /= left(i, i);}
            for(unsigned int j(i+1); j < columns_; j++){left(i, j) /= left(i, i);}
            left(i, i) = T(1);
          }
          for(unsigned int k(0); k < rows_; k++){
            if(k == i){continue;}
            if(value_t::zero != left(k, i)){
              for(unsigned int j(0); j < columns_; j++){right(k, j) -= right(i, j) * left(k, i);}
              for(unsigned int j(i+1); j < columns_; j++){left(k, j) -= left(i, j) * left(k, i);}
              left(k, i) = T(0);
            }
          }
        }
        //std::cout << "L:" << left << std::endl;
        //std::cout << "R:" << right << std::endl;

        return right;

        // TODO: method to use LU decomposition
        /*
        */
      }
    };
    template <class U>
    struct Inverse_Matrix<scalar_matrix_t, U> {
      typedef scalar_matrix_t mat_t;
      static mat_t generate(const scalar_matrix_t &mat) noexcept {
        return getScalar(mat.rows(), T(1) / mat(0, 0));
      }
    };

    /**
     * Calculate inverse matrix
     *
     * @return Inverse matrix
     * @throw std::logic_error When operation is undefined
     * @throw std::runtime_error When operation is unavailable
     */
    typename Inverse_Matrix<>::mat_t inverse() const {
      return Inverse_Matrix<>::generate(*this);
    }

    /**
     * Divide matrix by matrix, in other words, multiply by inverse matrix
     *
     * @param matrix Matrix to divide
     * @return divided matrix
     */
    template <class T2, class Array2D_Type2, class ViewType2>
    typename Multiply_Matrix_by_Matrix<
        typename Matrix_Frozen<T2, Array2D_Type2, ViewType2>
          ::template Inverse_Matrix<Matrix_Frozen<T2, Array2D_Type2, ViewType2> >::mat_t>::mat_t
        operator/(const Matrix_Frozen<T2, Array2D_Type2, ViewType2> &matrix) const {
      typedef typename Matrix_Frozen<T2, Array2D_Type2, ViewType2>
          ::template Inverse_Matrix<Matrix_Frozen<T2, Array2D_Type2, ViewType2> >::mat_t inv_t;
      return Multiply_Matrix_by_Matrix<inv_t>::generate(
          *this, matrix.inverse()); // equal to (*this) * matrix.inverse()
    }

    /**
     * Divide scalar by matrix, which is equivalent to inverted matrix multiplied by scalar
     *
     * @param scalar
     * @param matrix
     * @return result matrix
     */
    friend typename Multiply_Matrix_by_Scalar<T, typename Inverse_Matrix<>::mat_t>::mat_t
        operator/(const T &scalar, const self_t &matrix) {
      return Multiply_Matrix_by_Scalar<T, typename Inverse_Matrix<>::mat_t>
          ::generate(matrix.inverse(), scalar); // equal to matrix.inverse() * scalar
    }

    /**
     * Add matrix to matrix with specified pivot
     *
     * @param row Upper row index (pivot) of matrix to be added
     * @param column Left column index (pivot) of matrix to be added
     * @param matrix Matrix to add
     * @return added (deep) copy
     */
    template <class T2, class Array2D_Type2, class ViewType2>
    typename builder_t::assignable_t pivotAdd(
        const unsigned int &row, const unsigned int &column,
        const Matrix_Frozen<T2, Array2D_Type2, ViewType2> &matrix) const{
      return this->operator typename builder_t::assignable_t().pivotMerge(row, column, matrix);
    }

    Matrix_Frozen<typename value_t::complex_t, Array2D_Type, ViewType>
        complex() const noexcept {
      return Matrix_Frozen<typename value_t::complex_t, Array2D_Type, ViewType>(*this);
    }

    /**
     * Calculate the 2nd power of Frobenius norm.
     *
     * @return tr(A* * A)
     */
    typename value_t::real_t norm2F() const noexcept {
      // return value_t::get_real((adjoint() * (*this)).trace(false));
      /* The above is as definition, however, double calculation may occur
       * when (*this) is a expression matrix such as multiplication.
       * To avoid such overhead, its expansion form to take summation of
       * abs(element)**2 is used.
       */
      typename value_t::real_t res(0);
      for(unsigned int i(0), i_end(rows()); i < i_end; ++i){
        for(unsigned int j(0), j_end(columns()); j < j_end; ++j){
          res += value_t::get_abs2((*this)(i, j));
        }
      }
      return res;
    }

  protected:
    /**
     * Convert to a scaled vector having  to be used for Householder transformation
     *
     * @param x column vector to be converted (bang method)
     * @return 2nd power of Frobenius norm of x
     */
    template <class Array2D_Type2, class ViewType2>
    static typename value_t::real_t householder_vector(
        Matrix<T, Array2D_Type2, ViewType2> &x){
      // x = {(0,0), (1,0), ..., (N-1,0)}^{T}

      typename value_t::real_t x_abs2(x.norm2F());
      if(x_abs2 == 0){return x_abs2;}

      typename value_t::real_t x_abs(std::sqrt(x_abs2));
      typename value_t::real_t x_top_abs(std::abs(x(0, 0))); // x(0,0)
      T rho(x(0, 0) / x_top_abs * -1);
      // if x(0,0) is real, then rho = -sign(x(0,0)),
      // otherwise rho = - e^{i \phi}, where x(0,0) \equiv e^{i \phi} |x(0,0)|

      // x -> x_dash
      // x_dash = {(0,0) - \rho |x|, (1,0), ..., (N-1,0)}^{T}
      // x_dash_abs2
      //   = x_abs2 * (1 + \rho \bar{\rho}) - x_abs * (\rho \bar{x(0,0)} + \bar{\rho} x(0,0))
      //   = (x_abs2 + x_top_abs * x_abs) * 2
      x(0, 0) -= rho * x_abs;
      typename value_t::real_t x_dash_abs2((x_abs2 + x_top_abs * x_abs) * 2);

      return x_dash_abs2;
    }

  public:

    struct opt_decomposeQR_t {
      bool force_zeros;
      opt_decomposeQR_t()
          : force_zeros(true)
          {};
    };

    /**
     * Perform QR decomposition
     * Return matrix is
     * (0, 0)-(n-1, n-1):  Q matrix
     * (0, n)-(n-1, n + c): R matrix
     *
     * @return QR decomposed matrix
     */
    typename builder_t::template resize_t<0, builder_t::row_buffer>::assignable_t decomposeQR(
        const opt_decomposeQR_t &opt = opt_decomposeQR_t()) const {

      typedef typename builder_t::template resize_t<0, builder_t::row_buffer>::assignable_t res_t;
      res_t QR(res_t::blank(rows(), rows() + columns()));
      typename res_t::partial_offsetless_t Q(QR.partial(rows(), rows()));
      typename res_t::partial_t R(QR.partial(rows(), columns(), 0, rows()));
      Q.replace(getI(rows()), false);
      R.replace(*this, false);

      typedef typename builder_t::template resize_t<0, builder_t::row_buffer, 1, 0>::assignable_t Q_t;
      typedef typename builder_t::assignable_t R_t;

      unsigned int rc_min(columns() > rows() ? rows() : columns());

      { // Householder transformation
        typedef typename builder_t::template resize_t<0, 1, 1, 0>::assignable_t x_buf_t;
        x_buf_t x_buf(x_buf_t::blank(rows(), 1));

        for(unsigned int j(0); j < rc_min; j++){
          typename x_buf_t::partial_offsetless_t x(x_buf.partial(rows() - j, 1));
          x.replace(R.partial(x.rows(), 1, j, j), false);
          // x_0 = {(0,0), (1,0), ..., (N-1,0)}^{T}, (N-1)*1
          // x_1 = {(1,1), (2,1), ..., (N-1,1)}^{T}, (N-2)*1, ...

          typename value_t::real_t x_abs2(householder_vector(x));
          // x_0 <- {(0,0) - \rho |x|, (1,0), ..., (N-1,0)}^{T}
          // x_1 <- {(1,1) - \rho |x|, (2,1), ..., (N-1,1)}^{T}, ...
          if(x_abs2 == 0){continue;}

          if(false){ // as definition
            Q_t P(getI(rows()));
            P.pivotMerge(j, j, x * x.adjoint() * -2 / x_abs2);
            R.replace(R_t(P * R), false); // R and Q have partial views, therefore R = P * R, Q = Q * P raise build error.
            Q.replace(Q_t(Q * P), false);
          }else{ // optimized
            Q_t P((x * x.adjoint() * -2 / x_abs2) + 1);
            R.partial(rows() - j, columns(), j, 0).replace(R_t(P * R.partial(rows() - j, columns(), j, 0)), false);
            Q.partial(rows(), rows() - j, 0, j).replace(Q_t(Q.partial(rows(), rows() - j, 0, j) * P), false);
          }
        }
      }

      // TODO Gram-Schmidt orthonormalization
      // TODO Givens rotation

      if(opt.force_zeros){ // Zero clear
        for(unsigned int j(0), j_end(rc_min); j < j_end; j++){
          for(unsigned int i(j + 1), i_end(rows()); i < i_end; i++){
            R(i, j) = T(0);
          }
        }
      }

      return QR;
    }

    struct opt_hessenberg_t {
      enum {
        NOT_CHECKED, SQUARE, SYMMETRIC,
      } mat_prop;
      bool force_zeros;
      opt_hessenberg_t()
          : mat_prop(NOT_CHECKED), force_zeros(true)
          {};
    };

    /**
     * Calculate Hessenberg matrix by performing householder conversion
     *
     * @param transform Pointer to store multiplication of matrices used for the transformation.
     * If NULL is specified, the store will not be performed, The default is NULL.
     * @param opt calculation options
     * @return Hessenberg matrix
     * @throw std::logic_error When operation is undefined
     * @see https://people.inf.ethz.ch/arbenz/ewp/Lnotes/chapter4.pdf
     */
    template <class T2, class Array2D_Type2, class ViewType2>
    typename builder_t::assignable_t hessenberg(
        Matrix<T2, Array2D_Type2, ViewType2> *transform,
        const opt_hessenberg_t &opt = opt_hessenberg_t()) const {
      if((opt.mat_prop == opt_hessenberg_t::NOT_CHECKED) && !isSquare()){
        throw std::logic_error("rows() != columns()");
      }

      bool real_symmetric(
          (!value_t::is_complex)
          && ((opt.mat_prop == opt_hessenberg_t::SYMMETRIC)
            || ((opt.mat_prop == opt_hessenberg_t::NOT_CHECKED) && isSymmetric())));

      typename builder_t::assignable_t result(this->operator typename builder_t::assignable_t());
      typedef typename builder_t::template resize_t<0, 1, 1, 0>::assignable_t x_buf_t;
      x_buf_t x_buf(x_buf_t::blank(rows() - 1, 1));
      for(unsigned int j(0), j_end(columns() - 2); j < j_end; j++){
        typename x_buf_t::partial_offsetless_t x(x_buf.partial(rows() - (j+1), 1));
        x.replace(result.partial(x.rows(), 1, j+1, j), false);
        // x_0 = {(1,0), (2,0), ..., (N-1,0)}^{T}, (N-1)*1
        // x_1 = {(2,1), (3,1), ..., (N-1,1)}^{T}, (N-2)*1, ...

        typename value_t::real_t x_abs2(householder_vector(x));
        // x_0 <- {(1,0) - \rho |x|, (2,0), ..., (N-1,0)}^{T}
        // x_1 <- {(2,1) - \rho |x|, (3,1), ..., (N-1,1)}^{T}, ...
        if(x_abs2 == 0){continue;}

        // Householder transformation
        if(false){ // as definition
          typename builder_t::assignable_t P(getI(rows()));
          P.pivotMerge(j+1, j+1, x * x.adjoint() * -2 / x_abs2);
          result = P * result * P;
          if(transform){(*transform) *= P;}
        }else{ // optimized
          typename builder_t::assignable_t P((x * x.adjoint() * -2 / x_abs2) + 1);
          typename builder_t::assignable_t PX(P * result.partial(rows() - (j+1), columns(), j+1, 0));
          result.partial(rows() - (j+1), columns(), j+1, 0).replace(PX, false);
          typename builder_t::assignable_t PXP(result.partial(rows(), columns()-(j+1), 0, j+1) * P);
          result.partial(rows(), columns()-(j+1), 0, j+1).replace(PXP, false);
          if(transform){
            typename Matrix<T2, Array2D_Type2, ViewType2>::builder_t::assignable_t Pk(
                transform->partial(rows(), columns() - (j+1), 0, (j+1)) * P);
            transform->partial(rows(), columns() - (j+1), 0, (j+1)).replace(Pk, false);
          }
        }
      }

      if(opt.force_zeros){ // Zero clear
        for(unsigned int j(0), j_end(columns() - 2); j < j_end; j++){
          for(unsigned int i(j + 2), i_end(rows()); i < i_end; i++){
            result(i, j) = T(0);
            if(real_symmetric){result(j, i) = T(0);}
          }
        }
      }

      return result;
    }

    /**
     * Calculate Hessenberg matrix by performing householder conversion without return of multipled matrices
     *
     * @param opt calculation options
     * @return Hessenberg matrix
     * @throw std::logic_error When operation is undefined
     * @see https://people.inf.ethz.ch/arbenz/ewp/Lnotes/chapter4.pdf
     */
    typename builder_t::assignable_t hessenberg(const opt_hessenberg_t &opt = opt_hessenberg_t()) const {
      return hessenberg(static_cast<typename builder_t::assignable_t *>(NULL), opt);
    }

    /**
     * Calculate eigenvalues of 2 by 2 partial matrix.
     *
     * @param row Upper row index of the partial matrix
     * @param column Left column index of the partial matrix
     * @param upper Eigenvalue (1)
     * @param lower Eigenvalue (2)
     */
    void eigen22(
        const unsigned int &row, const unsigned int &column,
        typename value_t::complex_t &upper, typename value_t::complex_t &lower) const {
      const T
          &a((*this)(row, column)),     &b((*this)(row, column + 1)),
          &c((*this)(row + 1, column)), &d((*this)(row + 1, column + 1));
      typename value_t::complex_t root(value_t::get_sqrt((a - d) * (a - d) + b * c * 4));
      upper = ((root + a + d) / 2);
      lower = ((-root + a + d) / 2);
    }

    struct opt_eigen_t {
      enum {
        NOT_CHECKED, SQUARE,
      } mat_prop;
      typename value_t::real_t threshold_abs; ///< Absolute error to be used for convergence determination
      typename value_t::real_t threshold_rel; ///< Relative error to be used for convergence determination
      unsigned int inverse_power_max_iter;

      opt_eigen_t()
          : mat_prop(NOT_CHECKED),
          threshold_abs(1E-10), threshold_rel(1E-7),
          inverse_power_max_iter(10)
          {}
    };

    /**
     * Calculate eigenvalues and eigenvectors.
     * The return (n, n+1) matrix consists of
     * (0,j)-(n-1,j): Eigenvector (j) (0 <= j <= n-1)
     * (j,n): Eigenvalue (j)
     *
     * @param opt option to calculate eigenvalue/eigenvector
     * @return Eigenvalues and eigenvectors
     * @throw std::logic_error When operation is undefined
     * @throw std::runtime_error When operation is unavailable
     */
    typename MatrixBuilder< // a.k.a. (assignable complex matrix)(r, c+1)
          typename builder_t::template cast_t<typename value_t::complex_t>::assignable_t>
        ::template resize_t<0, 1>::assignable_t eigen(
        const opt_eigen_t &opt = opt_eigen_t()) const {

      typedef typename builder_t::template cast_t<typename value_t::complex_t>::assignable_t cmat_t;
      typedef typename MatrixBuilder<cmat_t>::template resize_t<0, 1, 1, 0>::assignable_t cvec_t;
      typedef typename MatrixBuilder<cmat_t>::template resize_t<0, 1>::assignable_t res_t;

      if((opt.mat_prop == opt_eigen_t::NOT_CHECKED) && (!isSquare())){
        throw std::logic_error("rows() != columns()");
      }

#if 0
      //パワー法(べき乗法)
      const unsigned int rows_(rows()), columns_(columns());
      typename builder_t::template resize_t<0, 1>::assignable_t result(rows_, rows_ + 1);
      typename builder_t::assignable_t source(this->operator typename builder_t::assignable_t());
      for(unsigned int i(0); i < columns_; i++){result(0, i) = T(1);}
      for(unsigned int i(0); i < columns_; i++){
        while(true){
          typename builder_t::template resize_t<0, 1>::assignable_t approxVec(
              source * result.columnVector(i));
          T approxVal(0);
          for(unsigned int j(0); j < approxVec.rows(); j++){approxVal += pow(approxVec(j, 0), 2);}
          approxVal = sqrt(approxVal);
          for(unsigned int j(0); j < approxVec.rows(); j++){result(j, i) = approxVec(j, 0) / approxVal;}
          T before = result(i, rows_);
          if(abs(before - (result(i, rows_) = approxVal)) < threshold){break;}
        }
        for(unsigned int j(0); (i < rows_ - 1) && (j < rows_); j++){
          for(unsigned int k(0); k < rows_; k++){
            source(j, k) -= result(i, rows_) * result(j, i) * result(k, i);
          }
        }
      }
      return result;
#endif

      // Double QR method
      /* <Procedure>
       * 1) Transform this to upper Hessenburg's matrix by using Householder's method
       * ハウスホルダー法を適用して、上ヘッセンベルク行列に置換後
       * 2) Then, Apply double QR method to get eigenvalues
       * ダブルQR法を適用。
       * 3) Finally, compute eigenvectors
       * 結果、固有値が得られるので、固有ベクトルを計算。
       */

      const unsigned int &_rows(rows());

      // Buffer to store resultant; 結果の格納用の行列
      res_t result(res_t::blank(_rows, _rows + 1));

      // Eigenvalue computation; 固有値の計算
#define lambda(i) result(i, _rows)

      int m = _rows;

      typename builder_t::assignable_t transform(getI(_rows));
      opt_hessenberg_t opt_A;
      opt_A.mat_prop = isSymmetric() ?  opt_hessenberg_t::SYMMETRIC : opt_hessenberg_t::SQUARE;
      typename builder_t::assignable_t A(hessenberg(&transform, opt_A));
      typename builder_t::assignable_t A_(A.copy());

      while(true){

        //m = 1 or m = 2
        if(m == 1){
          lambda(0) = A(0, 0);
          break;
        }else if(m == 2){
          A.eigen22(0, 0, lambda(0), lambda(1));
          break;
        }

        // Apply Householder transformation iteratively; ハウスホルダー変換を繰り返す
        for(int i(0); i < m - 1; i++){
          typename builder_t::template resize_t<3, 1, 0, 0>::assignable_t omega(3, 1);
          if(i == 0){ // calculate double shift of initial Householder transformation
            // @see https://people.inf.ethz.ch/arbenz/ewp/Lnotes/chapter4.pdf
            T trace_22(A(m-2, m-2) + A(m-1, m-1));
            T det_22(A(m-2, m-2) * A(m-1, m-1) - A(m-2, m-1) * A(m-1, m-2));
            omega(0, 0) = A(0, 0) * A(0, 0)  + A(0, 1) * A(1, 0) - trace_22 * A(0, 0) + det_22;
            omega(1, 0) = A(1, 0) * (A(0, 0) + A(1, 1) - trace_22);
            omega(2, 0) = A(2, 1) * A(1, 0);
          }else{
            omega(0, 0) = A(i, i - 1);
            omega(1, 0) = A(i + 1, i - 1);
            omega(2, 0) = (i == m - 2 ? T(0) : A(i + 2, i - 1));
            // caution: omega size is 3x3 if i in [0, m-2), however, 2x2 when i == m-2
          }

          typename value_t::real_t omega_abs2(householder_vector(omega));
          if(omega_abs2 == 0){continue;}
          //std::cout << "omega_abs(" << m << ") " << omega_abs << std::endl;

          if(false){ // as definition
            typename builder_t::assignable_t P(getI(_rows));
            P.pivotMerge(i, i, omega * omega.adjoint() * -2 / omega_abs2);
            A = P * A * P;
          }else{ // optimized
            if(i < m - 2){
              typename builder_t::template resize_t<3, 3, 0, 0>::assignable_t P(
                  (omega * omega.adjoint() * -2 / omega_abs2) + 1);
              // P multiplication from left
              unsigned i2((i <= 1) ? 0 : i - 2);
              typename builder_t::template resize_t<3, 0, 0, 1>::assignable_t PX(
                  P * A.partial(3, m - i2, i, i2));
              A.partial(3, m - i2, i, i2).replace(PX, false);
              // P multiplication from right
              unsigned i3((i >= m - 3) ? (i + 3) : (i + 4));
              typename builder_t::template resize_t<0, 3, 1, 0>::assignable_t PXP(
                  A.partial(i3, 3, 0, i) * P);
              A.partial(i3, 3, 0, i).replace(PXP, false);
            }else{ // i == m - 2
              typename builder_t::template resize_t<2, 2, 0, 0>::assignable_t P(
                  ((omega * omega.adjoint()).partial(2, 2) * -2 / omega_abs2) + 1);
              typename builder_t::template resize_t<2, 3, 0, 0>::assignable_t PX(
                  P * A.partial(2, 3, i, i - 1));
              A.partial(2, 3, i, i - 1).replace(PX, false);
              typename builder_t::template resize_t<0, 2, 1, 0>::assignable_t PXP(
                  A.partial(m, 2, 0, i) * P);
              A.partial(m, 2, 0, i).replace(PXP, false);
            }
          }
        }
        //std::cout << "A_scl(" << m << ") " << A(m-1,m-2) << std::endl;

        if(value_t::is_nan_or_infinite(A(m-1,m-2))){
          throw std::runtime_error("eigenvalues calculation failed");
        }

        // Convergence test; 収束判定
        typename value_t::real_t
            A_m2_abs(value_t::get_abs(A(m-2, m-2))),
            A_m1_abs(value_t::get_abs(A(m-1, m-1)));
        typename value_t::real_t epsilon(opt.threshold_abs
          + opt.threshold_rel * ((A_m2_abs < A_m1_abs) ? A_m2_abs : A_m1_abs));

        //std::cout << "epsil(" << m << ") " << epsilon << std::endl;

        if(value_t::get_abs(A(m-1, m-2)) < epsilon){
          --m;
          lambda(m) = A(m, m);
        }else if(value_t::get_abs(A(m-2, m-3)) < epsilon){
          A.eigen22(m-2, m-2, lambda(m-1), lambda(m-2));
          m -= 2;
        }
      }

#if defined(MATRIX_EIGENVEC_SIMPLE)
      // 固有ベクトルの計算
      cmat_t x(_rows, _rows);  // 固有ベクトル
      A = A_;

      for(unsigned int j(0); j < _rows; j++){
        unsigned int n = _rows;
        for(unsigned int i(0); i < j; i++){
          if((lambda(j) - lambda(i)).abs() <= threshold_abs){--n;}
        }
        //std::cout << n << ", " << lambda(j) << std::endl;
        x(--n, j) = 1;
        while(n-- > 0){
          x(n, j) = x(n+1, j) * (lambda(j) - A(n+1, n+1));
          for(unsigned int i(n+2); i < _rows; i++){
            x(n, j) -= x(i, j) * A(n+1, i);
          }
          if(A(n+1, n)){x(n, j) /= A(n+1, n);}
        }
        //std::cout << x.partial(_rows, 1, 0, j).transpose() << std::endl;
      }
#else
      // Inverse iteration to compute eigenvectors; 固有ベクトルの計算(逆反復法)
      cmat_t x(cmat_t::getI(_rows));  //固有ベクトル

      for(unsigned int j(0); j < _rows; j++){
        /* https://web.archive.org/web/20120702040824/http://www.prefield.com/algorithm/math/eigensystem.html
         * (Previously, http://www.prefield.com/algorithm/math/eigensystem.html)
         * is referred, and Numerical receipt
         * http://www.nrbook.com/a/bookcpdf/c11-7.pdf
         * is also utilized in case some eigenvalues are identical.
         */
        typename value_t::complex_t approx_lambda(lambda(j));
        approx_lambda += value_t::get_abs(approx_lambda) * 1E-4; // 0.01%
        typename MatrixBuilder<cmat_t>::template resize_t<0, 0, 1, 2>::assignable_t
            A_C_lambda_LU((A_.complex() - approx_lambda).decomposeLU(false));

        cvec_t target_x(cvec_t::blank(_rows, 1));
        target_x.replace(x.columnVector(j), false);
        for(unsigned int loop(0); true; loop++){
          cvec_t target_x_new(
              A_C_lambda_LU.solve_linear_eq_with_LU(target_x, false));
          typename value_t::complex_t mu((target_x_new.adjoint() * target_x)(0, 0)); // inner product
          typename value_t::real_t v2(target_x_new.norm2F());
          target_x.replace(target_x_new / std::sqrt(v2), false);
          //std::cout << j << ": " << target_x.transpose() << ", " << mu << ", " << v2 << std::endl;
          if(std::abs(mu.abs2() / v2 - 1) < opt.threshold_abs){
            x.columnVector(j).replace(target_x, false);
            break;
          }
          if(loop > opt.inverse_power_max_iter){
            throw std::runtime_error("eigen vectors calculation failed");
          }
        }
      }
#endif

      /*res_t lambda2(_rows, _rows);
      for(unsigned int i(0); i < _rows; i++){
        lambda2(i, i) = lambda(i);
      }

      std::cout << "A:" << A << std::endl;
      //std::cout << "x * x^-1" << x * x.inverse() << std::endl;
      std::cout << "x * lambda * x^-1:" << x * lambda2 * x.inverse() << std::endl;*/

      // Register eigenvectors; 結果の格納
      result.partial(_rows, _rows).replace(transform.complex() * x, false);
      // result.partial(_rows, _rows).replace(transform * x, false); is desireable,
      // however, (real) * (complex) may occur and fail to build.

#if 0
      // Normalization(正規化) is skipable due to transform matrix is unitary
      for(unsigned int j(0), j_end(x.columns()); j < j_end; j++){
        result.columnVector(j) /= value_t::get_sqrt(result.columnVector(j).norm2F());
      }
#endif
#undef lambda

      return result;
    }

  protected:
    /**
     * Calculate square root of a matrix
     *
     * If matrix (A) can be decomposed as
     * @f[
     *    A = V D V^{-1},
     * @f]
     * where D and V are diagonal matrix consisting of eigenvalues and eigenvectors, respectively,
     * the square root A^{1/2} is
     * @f[
     *    A^{1/2} = V D^{1/2} V^{-1}.
     * @f]
     *
     * @param eigen_mat result of eigen()
     * @return square root
     * @see eiegn(const T &, const T &)
     */
    template <class MatrixT>
    static typename MatrixBuilder<MatrixT>::template resize_t<0, -1>::assignable_t sqrt(
        const MatrixT &eigen_mat){
      unsigned int n(eigen_mat.rows());
      typename MatrixT::partial_offsetless_t VsD(eigen_mat.partial(n, n));
      typename MatrixBuilder<MatrixT>::template resize_t<0, -1>::assignable_t nV(VsD.inverse());
      for(unsigned int i(0); i < n; i++){
        nV.partial(1, n, i, 0) *= (eigen_mat(i, n).sqrt());
      }

      return (typename MatrixBuilder<MatrixT>::template resize_t<0, -1>::assignable_t)(VsD * nV);
    }

  public:
    /**
     * Calculate square root of a matrix
     *
     * @param opt option to calculate eigenvalue/eigenvector
     * @return square root (assignable complex matrix)
     * @see eigen(const opt_eigen_t &)
     */
    typename builder_t::template cast_t<typename value_t::complex_t>::assignable_t sqrt(
        const opt_eigen_t &opt = opt_eigen_t()) const {
      return sqrt(eigen(opt));
    }

    /**
     * Print matrix
     *
     */
    template<class CharT, class Traits>
    friend std::basic_ostream<CharT, Traits> &operator<<(
        std::basic_ostream<CharT, Traits> &out, const self_t &matrix){
      const unsigned int i_end(matrix.rows()), j_end(matrix.columns());
      out << "{";
      for(unsigned int i(0); i < i_end; i++){
        out << (i == 0 ? "" : ",") << std::endl << "{";
        for(unsigned int j(0); j < j_end; j++){
          out << (j == 0 ? "" : ",") << matrix(i, j);
        }
        out << "}";
      }
      out << std::endl << "}";
      return out;
    }

    struct inspect_t {
      self_t mat;
      inspect_t(const self_t &target) : mat(target){}

      template <class CharT, class Traits>
      struct format_t {
        std::basic_ostream<CharT, Traits> &out;
        format_t(std::basic_ostream<CharT, Traits> &_out) : out(_out) {}

        template <class U>
        format_t &operator<<(const U &u){
          out << u;
          return *this;
        }
        format_t &operator<<(std::basic_ostream<CharT, Traits> &(*f)(std::basic_ostream<CharT, Traits> &)){
          // for std::endl, which is defined with template<class CharT, class Traits>
          out << f;
          return *this;
        }

        template <class T2, class Array2D_Type2, class View_Type2>
        format_t &operator<<(const Matrix_Frozen<T2, Array2D_Type2, View_Type2> &m){
          (*this) << "M"
              << (MatrixViewProperty<View_Type2>::conjugated ? "~" : "")
              << (MatrixViewProperty<View_Type2>::transposed ? "t" : "")
              << (MatrixViewProperty<View_Type2>::variable_size ? "p" : "")
              << "(" << m.rows() << "," << m.columns() << ")";
          return *this;
        }

        template <class LHS_T, class RHS_T, bool rhs_positive>
        format_t &operator<<(const Array2D_Operator_Add<LHS_T, RHS_T, rhs_positive> &op){
          return (*this) << op.lhs << ", " << op.rhs;
        }
        template <class LHS_T, class RHS_T>
        format_t &operator<<(const Array2D_Operator_EntrywiseMultiply<LHS_T, RHS_T> &op){
          return (*this) << op.lhs << ", " << op.rhs;
        }
        template <class LHS_T, class RHS_T>
        format_t &operator<<(const Array2D_Operator_Multiply_by_Scalar<LHS_T, RHS_T> &op){
          return (*this) << op.lhs << ", " << op.rhs;
        }
        template <class LHS_T, class RHS_T>
        format_t &operator<<(const Array2D_Operator_Multiply_by_Matrix<LHS_T, RHS_T> &op){
          return (*this) << op.lhs << ", " << op.rhs;
        }
        template <class LHS_T, class RHS_T, bool rhs_horizontal>
        format_t &operator<<(const Array2D_Operator_Stack<LHS_T, RHS_T, rhs_horizontal> &op){
          return (*this) << op.lhs << ", " << op.rhs;
        }

        template <class T2, class T2_op, class OperatorT, class View_Type2>
        format_t &operator<<(
            const Matrix_Frozen<T2, Array2D_Operator<T2_op, OperatorT>, View_Type2> &m){
          const char *symbol = "";
          switch(OperatorProperty<
              Matrix_Frozen<T2, Array2D_Operator<T2_op, OperatorT>, View_Type2> >::tag){
            case OPERATOR_2_Multiply_Matrix_by_Scalar:
            case OPERATOR_2_Multiply_Matrix_by_Matrix:
              symbol = "*"; break;
            case OPERATOR_2_Add_Matrix_to_Matrix:
              symbol = "+"; break;
            case OPERATOR_2_Subtract_Matrix_from_Matrix:
              symbol = "-"; break;
            case OPERATOR_2_Entrywise_Multiply_Matrix_by_Matrix:
              symbol = ".*"; break;
            case OPERATOR_2_Stack_Horizontal:
              symbol = "H"; break;
            case OPERATOR_2_Stack_Vertical:
              symbol = "V"; break;
            default:
              return (*this) << "(?)";
          }
          return (*this) << "(" << symbol << ", "
              << m.storage.op << ")"
              << (MatrixViewProperty<View_Type2>::transposed ? "t" : "")
              << (MatrixViewProperty<View_Type2>::variable_size ? "p" : "");
        }
      };

      template<class CharT, class Traits>
      std::basic_ostream<CharT, Traits> &operator()(std::basic_ostream<CharT, Traits> &out) const {
        format_t<CharT, Traits>(out)
            << "prop: {" << std::endl
            << "  *(R,C): (" << mat.rows() << "," << mat.columns() << ")" << std::endl
            << "  *view: " << mat.view << std::endl
            << "  *storage: " << mat << std::endl
            << "}";
        return out;
      }
    };
    template<class CharT, class Traits>
    friend std::basic_ostream<CharT, Traits> &operator<<(std::basic_ostream<CharT, Traits> &out, const inspect_t &inspector){
      return inspector(out);
    }
    inspect_t inspect() const {
      return inspect_t(*this);
    }
};

template <
    class T, class Array2D_Type, class ViewType,
    class RHS_T>
struct Array2D_Operator_Multiply_by_Scalar<Matrix_Frozen<T, Array2D_Type, ViewType>, RHS_T>
    : public Array2D_Operator_Binary<Matrix_Frozen<T, Array2D_Type, ViewType>, RHS_T>{
  typedef Matrix_Frozen<T, Array2D_Type, ViewType> lhs_t;
  static const int tag = lhs_t::OPERATOR_2_Multiply_Matrix_by_Scalar;
  typedef Array2D_Operator_Binary<lhs_t, RHS_T> super_t;
  Array2D_Operator_Multiply_by_Scalar(const lhs_t &_lhs, const RHS_T &_rhs) noexcept
        : super_t(_lhs, _rhs) {}
  T operator()(const unsigned int &row, const unsigned int &column) const noexcept {
    return super_t::lhs(row, column) * super_t::rhs;
  }
};

template <
    class T, class Array2D_Type, class ViewType,
    class RHS_T>
struct Matrix_multiplied_by_Scalar<Matrix_Frozen<T, Array2D_Type, ViewType>, RHS_T> {
  typedef Matrix_Frozen<T, Array2D_Type, ViewType> lhs_t;
  typedef RHS_T rhs_t;
  typedef Array2D_Operator_Multiply_by_Scalar<lhs_t, rhs_t> impl_t;
  typedef Matrix_Frozen<T, Array2D_Operator<T, impl_t> > mat_t;
  static mat_t generate(const lhs_t &mat, const rhs_t &scalar) {
    return mat_t(
        typename mat_t::storage_t(
          mat.rows(), mat.columns(), impl_t(mat, scalar)));
  }
};

template <class LHS_T, class RHS_T>
struct Matrix_multiplied_by_Scalar<Matrix_Frozen<LHS_T, Array2D_ScaledUnit<LHS_T> >, RHS_T> {
  typedef Matrix_Frozen<LHS_T, Array2D_ScaledUnit<LHS_T> > lhs_t;
  typedef RHS_T rhs_t;
  typedef lhs_t mat_t;
  static mat_t generate(const lhs_t &mat, const rhs_t &scalar) {
    return mat_t(mat.rows(), mat(0, 0) * scalar);
  }
};

template <
    class T, class Array2D_Type, class ViewType,
    class T2, class Array2D_Type2, class ViewType2,
    bool rhs_positive>
struct Array2D_Operator_Add<
      Matrix_Frozen<T, Array2D_Type, ViewType>,
      Matrix_Frozen<T2, Array2D_Type2, ViewType2>,
      rhs_positive>
    : public Array2D_Operator_Binary<
        Matrix_Frozen<T, Array2D_Type, ViewType>,
        Matrix_Frozen<T2, Array2D_Type2, ViewType2> >{
  typedef Array2D_Operator_Binary<
      Matrix_Frozen<T, Array2D_Type, ViewType>,
      Matrix_Frozen<T2, Array2D_Type2, ViewType2> > super_t;
  static const int tag = rhs_positive
      ? super_t::lhs_t::OPERATOR_2_Add_Matrix_to_Matrix
      : super_t::lhs_t::OPERATOR_2_Subtract_Matrix_from_Matrix;
  Array2D_Operator_Add(
      const typename super_t::lhs_t &_lhs,
      const typename super_t::rhs_t &_rhs) noexcept
      : super_t(_lhs, _rhs) {}
  T operator()(const unsigned int &row, const unsigned int &column) const noexcept {
    if(rhs_positive){
      return super_t::lhs(row, column) + super_t::rhs(row, column);
    }else{
      return super_t::lhs(row, column) - super_t::rhs(row, column);
    }
  }
};

template <
    class T, class Array2D_Type, class ViewType,
    class T2, class Array2D_Type2, class ViewType2>
struct Array2D_Operator_EntrywiseMultiply<
      Matrix_Frozen<T, Array2D_Type, ViewType>,
      Matrix_Frozen<T2, Array2D_Type2, ViewType2> >
    : public Array2D_Operator_Binary<
        Matrix_Frozen<T, Array2D_Type, ViewType>,
        Matrix_Frozen<T2, Array2D_Type2, ViewType2> >{
  typedef Array2D_Operator_Binary<
      Matrix_Frozen<T, Array2D_Type, ViewType>,
      Matrix_Frozen<T2, Array2D_Type2, ViewType2> > super_t;
  static const int tag = super_t::lhs_t::OPERATOR_2_Entrywise_Multiply_Matrix_by_Matrix;
  Array2D_Operator_EntrywiseMultiply(
      const typename super_t::lhs_t &_lhs,
      const typename super_t::rhs_t &_rhs) noexcept
      : super_t(_lhs, _rhs) {}
  T operator()(const unsigned int &row, const unsigned int &column) const noexcept {
    return super_t::lhs(row, column) * super_t::rhs(row, column);
  }
};

template <
    class T, class Array2D_Type, class ViewType,
    class T2, class Array2D_Type2, class ViewType2,
    bool rhs_horizontal>
struct Array2D_Operator_Stack<
      Matrix_Frozen<T, Array2D_Type, ViewType>,
      Matrix_Frozen<T2, Array2D_Type2, ViewType2>,
      rhs_horizontal>
    : public Array2D_Operator_Binary<
        Matrix_Frozen<T, Array2D_Type, ViewType>,
        Matrix_Frozen<T2, Array2D_Type2, ViewType2> >{
  typedef Array2D_Operator_Binary<
      Matrix_Frozen<T, Array2D_Type, ViewType>,
      Matrix_Frozen<T2, Array2D_Type2, ViewType2> > super_t;
  static const int tag = rhs_horizontal
      ? super_t::lhs_t::OPERATOR_2_Stack_Horizontal
      : super_t::lhs_t::OPERATOR_2_Stack_Vertical;
  const unsigned int threshold;
  Array2D_Operator_Stack(
      const typename super_t::lhs_t &_lhs,
      const typename super_t::rhs_t &_rhs) noexcept
      : super_t(_lhs, _rhs),
      threshold(rhs_horizontal ? _lhs.columns() : _lhs.rows()) {}
  T operator()(const unsigned int &row, const unsigned int &column) const noexcept {
    if(rhs_horizontal){
      return (column < threshold) ? super_t::lhs(row, column) : super_t::rhs(row, column - threshold);
    }else{
      return (row < threshold) ? super_t::lhs(row, column) : super_t::rhs(row - threshold, column);
    }
  }
};

template <
    class T, class Array2D_Type, class ViewType,
    class T2, class Array2D_Type2, class ViewType2>
struct Array2D_Operator_Multiply_by_Matrix<
      Matrix_Frozen<T, Array2D_Type, ViewType>,
      Matrix_Frozen<T2, Array2D_Type2, ViewType2> >
    : public Array2D_Operator_Binary<
        Matrix_Frozen<T, Array2D_Type, ViewType>,
        Matrix_Frozen<T2, Array2D_Type2, ViewType2> >{
  typedef Matrix_Frozen<T, Array2D_Type, ViewType> lhs_t;
  typedef Matrix_Frozen<T2, Array2D_Type2, ViewType2> rhs_t;
  typedef Array2D_Operator_Multiply_by_Matrix<lhs_t, rhs_t> self_t;
  typedef Array2D_Operator_Binary<lhs_t, rhs_t> super_t;
  static const int tag = lhs_t::OPERATOR_2_Multiply_Matrix_by_Matrix;
  Array2D_Operator_Multiply_by_Matrix(const lhs_t &_lhs, const rhs_t &_rhs) noexcept
      : super_t(_lhs, _rhs) {}
  T operator()(const unsigned int &row, const unsigned int &column) const noexcept {
    T res(0);
    for(unsigned int i(0), i_end(super_t::lhs.columns()); i < i_end; ++i){
      res += super_t::lhs(row, i) * super_t::rhs(i, column);
    }
    return res;
  }
  typedef Matrix_Frozen<T, Array2D_Operator<T, self_t> > mat_t;
  static mat_t generate(const lhs_t &mat1, const rhs_t &mat2) {
    return mat_t(
        typename mat_t::storage_t(
          mat1.rows(), mat2.columns(), self_t(mat1, mat2)));
  }
};

template <
    class T, class Array2D_Type, class ViewType,
    class T2>
struct Array2D_Operator_Multiply_by_Matrix<
      Matrix_Frozen<T, Array2D_Type, ViewType>,
      Matrix_Frozen<T2, Array2D_ScaledUnit<T2> > >
    : public Matrix_multiplied_by_Scalar<Matrix_Frozen<T, Array2D_Type, ViewType>, T2> {
  typedef Matrix_multiplied_by_Scalar<Matrix_Frozen<T, Array2D_Type, ViewType>, T2> super_t;
  static typename super_t::mat_t generate(
      const typename super_t::lhs_t &mat1, const Matrix_Frozen<T2, Array2D_ScaledUnit<T2> > &mat2) {
    return super_t::generate(mat1, mat2(0, 0));
  }
};

/*
 * Downcast rules including operation
 * When multiplication of multiple matrices, the most left and right hand terms are extracted intermediately.
 * For example, type((M1 * M2) * M3) will be type(M1 * M3).
 * type((M1 * M2) * (M3 * M4)) will be (M1 * M4).
 * In addition, a type of the most left hand term is used for any operation.
 * Please see the following examples:
 * type((M1 + M2) + M3) will be type(M1 + M2), then type(M1).
 * type(((M1 * M2) * M3) + M4) will be type((M1 * M2) * M3), then type(M1 * M3), and finally type(M1).
 */
template <
    class T, class T_op, class OperatorT, class ViewType>
struct MatrixBuilder_Dependency<
    Matrix_Frozen<T, Array2D_Operator<T_op, OperatorT>, ViewType> > {

private:
  template <class OperatorT2>
  struct unpack_op_t {
    // consequently, M * S, M + M are captured
    // (op, M1, M2, ...) => M1, then apply ViewType
    typedef typename MatrixBuilder<typename OperatorT2::first_t::frozen_t>
        ::template view_apply_t<ViewType>::applied_t mat_t;
  };
  template <class LHS_T, class RHS_T>
  struct unpack_op_t<Array2D_Operator_Multiply_by_Matrix<LHS_T, RHS_T> > { // M * M
    template <
        class OperatorT_L = typename LHS_T::template OperatorProperty<>::operator_t,
        class OperatorT_R = typename RHS_T::template OperatorProperty<>::operator_t,
        class U = void>
    struct check_op_t {
      typedef Matrix_Frozen<T, Array2D_Operator<T, Array2D_Operator_Multiply_by_Matrix<
          typename MatrixBuilder<LHS_T>::assignable_t::frozen_t,
          typename MatrixBuilder<RHS_T>::assignable_t::frozen_t> >, ViewType> res_t;
    };
    template <class U>
    struct check_op_t<void, void, U> {
      // active when both left and right hand side terms are none operator
      // This may be overwritten by (M * M) if its MatrixBuilder specialization exists
      typedef typename MatrixBuilder<LHS_T>
          ::template view_apply_t<ViewType>::applied_t res_t;
    };
    typedef typename check_op_t<>::res_t mat_t;
  };
  template <class LHS_T, class RHS_T, bool rhs_horizontal>
  struct unpack_op_t<Array2D_Operator_Stack<LHS_T, RHS_T, rhs_horizontal> > { // (H/V, M, M)
    template <
        class OperatorT_L = typename LHS_T::template OperatorProperty<>::operator_t,
        class OperatorT_R = typename RHS_T::template OperatorProperty<>::operator_t,
        class U = void>
    struct check_op_t {
      typedef Matrix_Frozen<T, Array2D_Operator<T, Array2D_Operator_Stack<
          typename MatrixBuilder<LHS_T>::assignable_t::frozen_t,
          typename MatrixBuilder<RHS_T>::assignable_t::frozen_t,
          rhs_horizontal> >, ViewType> res_t;
    };
    template <class U>
    struct check_op_t<void, void, U> {
      // active when both left and right hand side terms are none operator
      // This may be overwritten by (H/V, M, M) if its MatrixBuilder specialization exists
      typedef typename MatrixBuilder<LHS_T>
          ::template view_apply_t<ViewType>::applied_t res_t;
    };
    typedef typename check_op_t<>::res_t mat_t;
  };

  typedef MatrixBuilder<typename unpack_op_t<OperatorT>::mat_t> gen_t;
public:
  static const int row_buffer = gen_t::row_buffer;
  static const int column_buffer = gen_t::column_buffer;

  typedef typename gen_t::assignable_t assignable_t;

  template <class T2>
  struct cast_t {
    typedef typename gen_t::template cast_t<T2>::assignable_t assignable_t;
  };

  template <int nR_add = 0, int nC_add = 0, int nR_multiply = 1, int nC_multiply = 1>
  struct resize_t {
    typedef typename gen_t::template resize_t<nR_add, nC_add, nR_multiply, nC_multiply>::assignable_t assignable_t;
  };
};


/**
 * @brief Matrix
 *
 * Most of useful matrix operations are defined.
 *
 * Special care when you want to make copy;
 * The copy constructor(s) and change functions of view such as
 * transpose() are implemented by using shallow copy, which means
 * these return values are linked to their original operand.
 * If you unlink the relation between the original and returned matrices,
 * you have to use copy(), which makes a deep copy explicitly,
 * for example, mat.transpose().copy().
 *
 * @param T precision such as double
 * @param Array2D_Type Storage type. The default is Array2D_Dense
 * @param ViewType View type. The default is void, which means no view, i.e. direct access.
 */
template <class T, class Array2D_Type, class ViewType>
class Matrix : public Matrix_Frozen<T, Array2D_Type, ViewType> {
  public:
    typedef Matrix_Frozen<T, Array2D_Type, ViewType> super_t;

#if defined(__GNUC__) && (__GNUC__ < 5)
    typedef typename super_t::storage_t storage_t;
#else
    using typename super_t::storage_t;
#endif

    typedef Matrix<T, Array2D_Type, ViewType> self_t;
    typedef MatrixViewProperty<ViewType> view_property_t;
    typedef MatrixBuilder<self_t> builder_t;

    typedef typename builder_t::assignable_t clone_t;

    template <class T2, class Array2D_Type2, class ViewType2>
    friend class Matrix_Frozen;

    template <class T2, class Array2D_Type2, class ViewType2>
    friend class Matrix;

    template <class ImplementedT>
    static char (&check_storage(Array2D<T, ImplementedT> *) )[1];
    static const int storage_t_should_be_derived_from_Array2D
        = sizeof(check_storage(static_cast<storage_t *>(0)));

  protected:
    /**
     * Constructor with storage
     *
     * @param new_storage new storage
     */
    Matrix(const storage_t &new_storage) : super_t(new_storage) {}

    using super_t::storage;

  public:
    /**
     * Return matrix element of specified indices.
     *
     * @param row Row index starting from 0.
     * @param column Column index starting from 0.
     * @return element
     */
    using super_t::operator();
    T &operator()(
        const unsigned int &row,
        const unsigned int &column){
      return super_t::view.DELETE_IF_MSC(template) operator()<T &, Array2D_Type>(storage, row, column);
    }

    using super_t::rows;
    using super_t::columns;

    template <template <typename> class MapperT = super_t::template iterator_base_t>
    class iterator_skelton_t : public MapperT<iterator_skelton_t<MapperT> > {
      public:
        typedef T value_type;
        typedef T& reference;
        typedef T* pointer;
      protected:
        typedef MapperT<iterator_skelton_t<MapperT> > base_t;
        self_t mat;
      public:
        reference operator*() {return mat(base_t::r, base_t::c);}
        pointer operator->() {return &(operator*());}
        iterator_skelton_t(const self_t &mat_, const typename base_t::difference_type &idx_ = 0)
            : base_t(mat_, idx_), mat(mat_) {}
        iterator_skelton_t()
            : base_t(), mat() {}
        reference operator[](const typename base_t::difference_type &n){
          return *((*this) + n);
        }
    };
    typedef iterator_skelton_t<super_t::iterator_mapper_t::template all_t> iterator;
    using super_t::begin;
    iterator begin() {return iterator(*this);}
    typename super_t::const_iterator cbegin() const {return super_t::begin();}
    using super_t::end;
    iterator end() {return iterator(*this).tail();}
    typename super_t::const_iterator cend() const {return super_t::end();}

    template <template <typename> class MapperT>
    iterator_skelton_t<MapperT> begin() {
      return iterator_skelton_t<MapperT>(*this);
    }
    template <template <typename> class MapperT>
    typename super_t::template const_iterator_skelton_t<MapperT> cbegin() const {
      return super_t::template begin<MapperT>();
    }
    template <template <typename> class MapperT>
    iterator_skelton_t<MapperT> end() {
      return iterator_skelton_t<MapperT>(*this).tail();
    }
    template <template <typename> class MapperT>
    typename super_t::template const_iterator_skelton_t<MapperT> cend() const {
      return super_t::template end<MapperT>();
    }

    /**
     * Clear elements.
     *
     */
    void clear(){
      if(view_property_t::variable_size){
        for(unsigned int i(0), i_end(rows()); i < i_end; i++){
          for(unsigned int j(0), j_end(columns()); j < j_end; j++){
            (*this)(i, j) = T(0);
          }
        }
      }else{
        storage.storage_t::clear();
      }
    }

    /**
     * Constructor without storage.
     *
     */
    Matrix() : super_t(){}

    /**
     * Constructor with specified row and column numbers.
     * The storage will be assigned with the size.
     * The elements will be cleared with T(0).
     *
     * @param rows Row number
     * @param columns Column number
     */
    Matrix(
        const unsigned int &rows,
        const unsigned int &columns)
        : super_t(Array2D_Type(rows, columns)){
      /* Array2D_Type is intentionally used instead of storage_t due to VC2010(C2514) */
      clear();
    }

    /**
     * Constructor with specified row and column numbers, and values.
     * The storage will be assigned with the size.
     * The elements will be initialized with specified values。
     *
     * @param rows Row number
     * @param columns Column number
     * @param serialized Initial values of elements
     */
    Matrix(
        const unsigned int &rows,
        const unsigned int &columns,
        const T *serialized)
        : super_t(Array2D_Type(rows, columns, serialized)){
      /* Array2D_Type is intentionally used instead of storage_t due to VC2010(C2514) */
    }

    /**
     * Copy constructor generating shallow copy linking to source matrix
     *
     * @param another source matrix
     */
    Matrix(const self_t &another)
        : super_t(another){}

    /**
     * Constructor with different storage type
     * This will be used as Matrix x(Matrix + Matrix) to get calculation results,
     * where most of calculations are returned in a Matrix_Frozen due to expression template technique.
     * Another example is Matrix<Complex<double> > (Matrix<double>::getI(N))
     * to cast real to complex type.
     */
    template <class T2, class Array2D_Type2>
    Matrix(const Matrix_Frozen<T2, Array2D_Type2, ViewType> &matrix)
        : super_t(matrix) {}
  protected:
    /**
     * Constructor with different view
     * This will be used for view change such as transpose().
     */
    template <class ViewType2>
    Matrix(const Matrix<T, Array2D_Type, ViewType2> &matrix)
        : super_t(matrix){}

  public:
    /**
     * Destructor
     */
    virtual ~Matrix(){}


    /**
     * Matrix generator with specified row and column numbers.
     * The storage will be assigned with the size,
     * however, initialization of elements will NOT be performed.
     * In addition, its view is none.
     *
     * @param new_rows Row number
     * @param new_columns Column number
     */
    static typename builder_t::assignable_t blank(
        const unsigned int &new_rows,
        const unsigned int &new_columns){
      typedef typename builder_t::assignable_t res_t;
      typedef typename res_t::storage_t s_t; // work around of VC2010(C2514)
      return res_t(s_t(new_rows, new_columns));
    }

  protected:
    clone_t blank_copy() const {
      return clone_t::blank(rows(), columns());
    }

  public:
    /**
     * Assigner for the same type matrix
     * After this operation, another variable which shared the buffer before the operation will be unlinked.
     * Its modification strategy is dependent on the implementation of storage=(another.storage),
     * which is called in frozen_t::operator=(const frozen_t &).
     * For example, storage is a Array2D_Storage, then shallow copy is conducted.
     *
     * @return myself
     */
    self_t &operator=(const self_t &another){
      super_t::operator=(another); // frozen_t::operator=(const frozen_t &) is exactly called
      return *this;
    }
    /**
     * Assigner for matrix having a different storage type
     * After this operation, another variable which shared the buffer before the operation will be unlinked.
     * Its modification strategy is dependent on the implementation of storage=(another.storage),
     * which is called in frozen_t::operator=(const another_frozen_t &).
     * For example, storage is a Array2D_Storage, then deep copy is conducted.
     * This will be used as
     *   Matrix<Complex<double> > mat_c;
     *   mat_c = Matrix<double>::getI(N);
     *
     * @return myself
     */
    template <class T2, class Array2D_Type2>
    self_t &operator=(const Matrix<T2, Array2D_Type2, ViewType> &matrix){
      super_t::operator=(matrix); // frozen_t::operator=(const another_frozen_t &) is exactly called
      return *this;
    }

  protected:
    template <bool clone_storage = false, class U = void>
    struct copy_t {
      static clone_t run(const self_t &self){
        clone_t res(self.blank_copy());
        builder_t::copy_value(res, self);
        return res;
      }
    };
    template <class U>
    struct copy_t<true, U> {
      static clone_t run(const self_t &self){
        return clone_t(self.storage.storage_t::copy(true));
      }
    };

  public:
    /**
     * Perform (deep) copy
     *
     * @return (clone_t)
     */
    clone_t copy() const {
      // To avoid undefined reference of (future defined, such as Matrix_Fixed) downcast in GCC,
      // template is used.
      return copy_t<view_property_t::viewless>::run(*this);
    }

  protected:
    /**
     * Cast to another Matrix defined in Matrix_Frozen is intentionally protected.
     *
     * Use copy() for Matrix
     */
    operator clone_t() const;

  public:
    typedef typename builder_t::transpose_t transpose_t;
    /**
     * Generate transpose matrix
     * Be careful, the return value is linked to the original matrix.
     * In order to unlink, do transpose().copy().
     *
     * @return Transpose matrix
     */
    transpose_t transpose() const noexcept {
      return transpose_t(*this);
    }

    typedef typename builder_t::partial_t partial_t;
    /**
     * Generate partial matrix
     * Be careful, the return value is linked to the original matrix.
     * In order to unlink, do partial().copy().
     *
     * @param rowSize Row number
     * @param columnSize Column number
     * @param rowOffset Upper row index of original matrix for partial matrix
     * @param columnOffset Left column index of original matrix for partial matrix
     * @throw std::out_of_range When either row or column size exceeds original one
     * @return partial matrix
     *
     */
    partial_t partial(
        const unsigned int &new_rows,
        const unsigned int &new_columns,
        const unsigned int &row_offset,
        const unsigned int &column_offset) const {
      return super_t::partial_internal(*this,
          new_rows, new_columns, row_offset, column_offset);
    }

    typedef typename builder_t::partial_offsetless_t partial_offsetless_t;
    /**
     * Generate partial matrix by just reducing its size;
     * The origins and direction of original and return matrices are the same.
     * Be careful, the return value is linked to the original matrix.
     * In order to unlink, do partial().copy().
     *
     * @param rowSize Row number
     * @param columnSize Column number
     * @throw std::out_of_range When either row or column size exceeds original one
     * @return partial matrix
     */
    partial_offsetless_t partial(
        const unsigned int &new_rows,
        const unsigned int &new_columns) const {
      return super_t::partial_internal(*this, new_rows, new_columns);
    }

    using super_t::circular;

    typedef typename builder_t::circular_bijective_t circular_bijective_t;
    /**
     * Generate matrix with circular view, keeping original size version.
     * This version is still belonged into Matrix class.
     * Another size variable version returns Matrix_Frozen.
     *
     * @param row_offset Upper row index of original matrix for circular matrix
     * @param column_offset Left column index of original matrix for circular matrix
     * @return matrix with circular view
     * @see circular(
     *    const unsigned int &, const unsigned int &,
     *    const unsigned int &, const unsigned int &)
     */
    circular_bijective_t circular(
        const unsigned int &row_offset,
        const unsigned int &column_offset) const noexcept {
      return super_t::circular_internal(*this, row_offset, column_offset);
    }

    /**
     * Generate row vector by using partial()
     *
     * @param row Row index of original matrix for row vector
     * @return Row vector
     * @see partial()
     */
    partial_t rowVector(const unsigned int &row) const {
      return partial(1, columns(), row, 0);
    }
    /**
     * Generate column vector by using partial()
     *
     * @param column Column index of original matrix for column vector
     * @return Column vector
     * @see partial()
     */
    partial_t columnVector(const unsigned int &column) const {
      return partial(rows(), 1, 0, column);
    }

    /**
     * Swap rows (bang method).
     * This operation has a side effect to another variables sharing the buffer.
     *
     * @param row1 Target row (1)
     * @param row2 Target row (2)
     * @return myself
     * @throw std::out_of_range When row1 or row2 exceeds bound
     */
    self_t &swapRows(
        const unsigned int &row1, const unsigned int &row2){
      if(row1 >= rows() || row2 >= rows()){
        throw std::out_of_range("Row index incorrect");
      }
      T temp;
      for(unsigned int j(0), j_end(columns()); j < j_end; j++){
        temp = (*this)(row1, j);
        (*this)(row1, j) = (*this)(row2, j);
        (*this)(row2, j) = temp;
      }
      return *this;
    }

    /**
     * Swap columns (bang method).
     * This operation has a side effect to another variables sharing the buffer.
     *
     * @param column1 Target column (1)
     * @param column2 Target column (2)
     * @return myself
     * @throw std::out_of_range When column1 or column2 exceeds bound
     */
    self_t &swapColumns(
        const unsigned int &column1, const unsigned int &column2){
      if(column1 >= columns() || column2 >= columns()){
        throw std::out_of_range("Column index incorrect");
      }
      T temp;
      for(unsigned int i(0), i_end(rows()); i < i_end; i++){
        temp = (*this)(i, column1);
        (*this)(i, column1) = (*this)(i, column2);
        (*this)(i, column2) = temp;
      }
      return *this;
    }

    /**
     * Replace content
     * This operation has a side effect to another variable sharing the buffer.
     *
     * @param matrix matrix to be replaced to
     * @param do_check Check matrix size property. The default is true
     * @return matrix with new content
     */
    template <class T2, class Array2D_Type2, class ViewType2>
    self_t &replace(
        const Matrix_Frozen<T2, Array2D_Type2, ViewType2> &matrix,
        const bool &do_check = true){
      if(do_check && isDifferentSize(matrix)){
        throw std::invalid_argument("Incorrect size");
      }
      return Matrix_Frozen<T2, Array2D_Type2, ViewType2>::builder_t::copy_value(*this, matrix);
    }

    using super_t::isSquare;
    using super_t::isDiagonal;
    using super_t::isSymmetric;
    using super_t::isLowerTriangular;
    using super_t::isUpperTriangular;
    using super_t::isHermitian;
    using super_t::isDifferentSize;
    using super_t::isLU;
    using super_t::isSkewSymmetric;
    using super_t::isNormal;
    using super_t::isOrthogonal;
    using super_t::isUnitary;

    /*
     * operator+=, operator-=, operator*=, operator/= are shortcuts of this->replace((*this) op another).
     * Be careful, they affect another variable whose referenced buffer is the same as (*this).
     * They are different from (*this) = (this_type)((*this) op another),
     * which does not affect another variable whose referenced buffer was the same as (*this) before the operation.
     */

    /**
     * Multiply matrix by scalar (bang method)
     *
     * @param scalar
     * @return myself
     */
    self_t &operator*=(const T &scalar) noexcept {
      return replace((*this) * scalar, false);
    }

    /**
     * Divide matrix by scalar (bang method)
     *
     * @param scalar
     * @return myself
     */
    self_t &operator/=(const T &scalar) noexcept {
      return operator*=(T(1) / scalar);
    }
    
    /**
     * Add matrix to matrix (bang method)
     *
     * @param matrix Matrix to add
     * @return myself
     */
    template <class T2, class Array2D_Type2, class ViewType2>
    self_t &operator+=(const Matrix_Frozen<T2, Array2D_Type2, ViewType2> &matrix){
      return replace((*this) + matrix, false);
    }
    
    /**
     * Subtract matrix from matrix (bang method)
     *
     * @param matrix Matrix to subtract
     * @return myself
     */
    template <class T2, class Array2D_Type2, class ViewType2>
    self_t &operator-=(const Matrix_Frozen<T2, Array2D_Type2, ViewType2> &matrix){
      return replace((*this) - matrix, false);
    }

    /**
     * Add scalar to matrix (bang method)
     *
     * @param scalar scalar to add
     * @return myself
     */
    self_t &operator+=(const T &scalar){
      return replace((*this) + scalar, false);
    }

    /**
     * Subtract scalar from matrix (bang method)
     *
     * @param scalar scalar to subtract
     * @return myself
     */
    self_t &operator-=(const T &scalar){
      return replace((*this) - scalar, false);
    }

    /**
     * Multiply matrix by matrix (bang method)
     *
     * @param matrix Matrix to multiply
     * @return myself
     */
    template <class T2, class Array2D_Type2, class ViewType2>
    self_t &operator*=(const Matrix_Frozen<T2, Array2D_Type2, ViewType2> &matrix){
      return replace((clone_t)(*this * matrix));
    }

    /**
     * Divide matrix by matrix, in other words, multiply matrix by inverse matrix. (bang method)
     *
     * @param matrix Matrix to divide
     * @return myself
     */
    template <class T2, class Array2D_Type2, class ViewType2>
    self_t &operator/=(const Matrix_Frozen<T2, Array2D_Type2, ViewType2> &matrix) {
      return replace((clone_t)(*this / matrix));
    }

    /**
     * Add matrix to matrix with specified pivot (bang method)
     *
     * @param row Upper row index (pivot) of matrix to be added
     * @param column Left column index (pivot) of matrix to be added
     * @param matrix Matrix to add
     * @return myself
     */
    template <class T2, class Array2D_Type2, class ViewType2>
    self_t &pivotMerge(
        const int &row, const int &column,
        const Matrix_Frozen<T2, Array2D_Type2, ViewType2> &matrix){

      unsigned int i_min(row < 0 ? 0 : row), j_min(column < 0 ? 0 : column),
          i_max((row + matrix.rows()) > rows() ? rows() : row + matrix.rows()),
          j_max((column + matrix.columns()) > columns() ? columns() : column + matrix.columns());
      for(unsigned int i(i_min), i2(i_min - row); i < i_max; i++, i2++){
        for(unsigned int j(j_min), j2(j_min - column); j < j_max; j++, j2++){
          (*this)(i, j) += matrix(i2, j2);
        }
      }

      return *this;
    }

};

template <class T, class Array2D_Type, class ViewType, class RHS_T>
struct Matrix_multiplied_by_Scalar<Matrix<T, Array2D_Type, ViewType>, RHS_T>
    : public Matrix_multiplied_by_Scalar<Matrix_Frozen<T, Array2D_Type, ViewType>, RHS_T> {};

template <class LHS_T, class RHS_T>
struct Array2D_Operator_Multiply_by_Matrix
    : public Array2D_Operator_Multiply_by_Matrix<typename LHS_T::frozen_t, typename RHS_T::frozen_t> {};

#undef DELETE_IF_MSC
#undef throws_when_debug
#if (__cplusplus < 201103L) && defined(noexcept)
#undef noexcept
#endif

#endif /* __MATRIX_H */
