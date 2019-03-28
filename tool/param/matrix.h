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
 * 3) to use views for transpose and partial matrices
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
#include <ostream>
#include "param/complex.h"

#if (__cplusplus < 201103L) && !defined(noexcept)
#define noexcept throw()
#endif
#if defined(DEBUG) && !defined(throws_when_debug)
#define throws_when_debug
#else
#define throws_when_debug noexcept
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
     * Assigner, which performs shallow copy.
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
     * @param is_deep If true, return deep copy, otherwise return shallow copy (just link).
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
    typedef Array2D<T, self_t> root_t;
    
    template <class T2>
    struct family_t {
      typedef Array2D_Dense<T2> res_t;
    };

    using root_t::rows;
    using root_t::columns;

  protected:
    T *values; ///< array for values
    int *ref;  ///< reference counter

    template <class T2>
    static void copy_raw(Array2D_Dense<T2> &dist, const T2 *src){
      std::memcpy(dist.values, src, sizeof(T2) * dist.rows() * dist.columns());
    }

    template <class T2>
    static void clear_raw(Array2D_Dense<T2> &target){
      std::memset(target.values, 0, sizeof(T2) * target.rows() * target.columns());
    }

  public:
    Array2D_Dense() : super_t(0, 0), values(NULL), ref(NULL) {
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
        values(new T[rows * columns]), ref(new int(1)) {
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
        values(new T[rows * columns]), ref(new int(1)) {
      copy_raw(*this, serialized);
    }
    /**
     * Copy constructor, which performs shallow copy.
     *
     * @param array another one
     */
    Array2D_Dense(const self_t &array)
        : super_t(array.m_rows, array.m_columns){
      if(values = array.values){(*(ref = array.ref))++;}
    }
    /**
     * Constructor based on another type array, which performs deep copy.
     *
     * @param array another one
     */
    template <class T2>
    Array2D_Dense(const Array2D_Frozen<T2> &array)
        : values(new T[array.rows() * array.columns()]), ref(new int(1)) {
      T *buf(values);
      for(unsigned int i(0); i < array.rows(); ++i){
        for(unsigned int j(0); j < array.rows(); ++j){
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
      if(ref && ((--(*ref)) <= 0)){
        delete [] values;
        delete ref;
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
        if(ref && ((--(*ref)) <= 0)){delete ref; delete [] values;}
        ref = NULL;
        if(values = array.values){
          super_t::m_rows = array.m_rows;
          super_t::m_columns = array.m_columns;
          (*(ref = array.ref))++;
        }
      }
      return *this;
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
      clear_raw(*this);
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
    typedef Array2D_Frozen<T> root_t;

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
    typedef Array2D_Operator<T, OperatorT> self_t;
    typedef Array2D_Frozen<T> super_t;
    typedef Array2D_Frozen<T> root_t;

    const OperatorT op;

    /**
     * Constructor
     *
     * @param rows Rows
     * @param columns Columns
     */
    Array2D_Operator(
        const unsigned int &r, const unsigned int &c,
        const OperatorT &_op)
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
  typedef LHS_T lhs_t;
  typedef RHS_T rhs_t;
  lhs_t lhs; ///< Left hand side value
  rhs_t rhs; ///< Right hand side value
  Array2D_Operator_Binary(const lhs_t &_lhs, const RHS_T &_rhs) noexcept
      : lhs(_lhs), rhs(_rhs) {}
};


template <class BaseView = void>
struct MatrixViewBase {
  typedef MatrixViewBase self_t;

  template<class CharT, class Traits>
  friend std::basic_ostream<CharT, Traits> &operator<<(
      std::basic_ostream<CharT, Traits> &out, const self_t &view){
    return out << " [V]";
  }

  inline const unsigned int rows(
      const unsigned int &_rows, const unsigned int &_columns) const noexcept {
    return _rows;
  }
  inline const unsigned int columns(
      const unsigned int &_rows, const unsigned int &_columns) const noexcept {
    return _columns;
  }
  inline unsigned int i(
      const unsigned int &i, const unsigned int &j) const noexcept {
    return i;
  }
  inline unsigned int j(
      const unsigned int &i, const unsigned int &j) const noexcept {
    return j;
  }
};

template <class BaseView>
struct MatrixViewTranspose;

template <class BaseView>
struct MatrixViewPartial;

template <class View>
struct MatrixViewProperty {
  typedef View self_t;
  static const bool viewless = true;

  template <template <class> class TargetView>
  struct check_of_t {
    static const bool res = false;
  };

  static const bool transposed = false;
  static const bool partialized = false;

  static const bool truncated = false;
};

template <class V1, template <class> class V2>
struct MatrixViewProperty<V2<V1> > {
  typedef V2<V1> self_t;
  static const bool viewless = false;

  template <template <class> class TargetView>
  struct check_of_t {
    template <template <class> class T, class U = void>
    struct check_t {
      static const bool res = MatrixViewProperty<V1>::template check_of_t<TargetView>::res;
    };
    template <class U>
    struct check_t<TargetView, U> {
        static const bool res = true;
    };
    static const bool res = check_t<V2>::res;
  };

  static const bool transposed = check_of_t<MatrixViewTranspose>::res;
  static const bool partialized = check_of_t<MatrixViewPartial>::res;

  static const bool truncated = partialized;
};

template <class View>
struct MatrixViewBuilder {
  typedef MatrixViewProperty<View> property_t;

  template <template <class> class V, class U = void>
  struct priority_t {
    static const int priority = -1;
  };
  enum {
    Partial = 0,
    Transpose,
  };
#define make_priority_table(name) \
template <class U> \
struct priority_t<MatrixView ## name, U> { \
  static const int priority = name; \
};
  make_priority_table(Partial)
  make_priority_table(Transpose)
#undef make_priority_table

  template <template <class> class V1, class V2>
  struct sort_t {
    typedef V1<V2> res_t;
  };
  template <template <class> class V1, template <class> class V2, class V3>
  struct sort_t<V1, V2<V3> > {
    template <bool, class U = void>
    struct rebuild_t {
      typedef V1<V2<V3> > res_t;
    };
    template <class U>
    struct rebuild_t<false, U> {
      typedef V2<V1<V3> > res_t;
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

  template <template <class> class AddView>
  struct add_t {
    template <class V>
    struct rebuild_t {
      typedef AddView<V> res_t;
    };
    template <class V1, template <class> class V2>
    struct rebuild_t<V2<V1> > {
      typedef typename sort_t<V2, typename rebuild_t<V1>::res_t>::res_t res_t;
    };
    typedef typename rebuild_t<View>::res_t res_t;
  };

  template <template <class> class SwitchView>
  struct switch_t { // off => on => off => ...
    template <class V>
    struct rebuild_t {
      template <class V1>
      struct check_t {
        typedef SwitchView<V1> res_t;
      };
      template <class V1, template <class> class V2>
      struct check_t<V2<V1> > {
        typedef typename sort_t<V2, typename rebuild_t<V1>::res_t>::res_t res_t;
      };
      typedef typename check_t<V>::res_t res_t;
    };
    template <class V>
    struct rebuild_t<SwitchView<V> > {
      typedef typename MatrixViewBuilder<V>
          ::template remove_t<SwitchView>::res_t res_t; // remove all subsequential SwitchView
    };
    typedef typename rebuild_t<View>::res_t res_t;
  };

  template <template <class> class OnceView>
  struct once_t { // off => on => on => ...
    typedef typename MatrixViewBuilder<
        typename remove_t<OnceView>::res_t> // remove all OnceView, then add an OnceView
            ::template add_t<OnceView>::res_t res_t;
  };

  typedef typename switch_t<MatrixViewTranspose>::res_t transpose_t;
  typedef typename once_t<MatrixViewPartial>::res_t partial_t;


  template <class View2>
  struct copy_t {
    typedef property_t dist_prop_t;
    typedef MatrixViewProperty<View2> orig_prop_t;

    template <bool, class U = void>
    struct partial_t {
      static void run(View &dist, const View2 &orig){}
    };
    template <class U>
    struct partial_t<true, U> {
      static void run(View &dist, const View2 &orig){
        dist.partial_prop = orig.partial_prop;
      }
    };
    static void run(View &dist, const View2 &orig){
      partial_t<
          dist_prop_t::partialized
          && orig_prop_t::partialized>::run(dist, orig);
    }
  };
  template <class View2>
  static void copy(View &dist, const View2 &orig){
    copy_t<View2>::run(dist, orig);
  }

  /* Using template because of inhibit generation of method
   * in case View does not have partial attribute.
   */
  template <class View2>
  static void set_partial(
      View2 &view,
      const unsigned int &new_rows,
      const unsigned int &new_columns,
      const unsigned int &row_offset,
      const unsigned int &column_offset){
    if(property_t::transposed){
      view.partial_prop.rows = new_columns;
      view.partial_prop.columns = new_rows;
      view.partial_prop.row_offset += column_offset;
      view.partial_prop.column_offset += row_offset;
    }else{
      view.partial_prop.rows = new_rows;
      view.partial_prop.columns = new_columns;
      view.partial_prop.row_offset += row_offset;
      view.partial_prop.column_offset += column_offset;
    }

  }
};

template <class BaseView>
struct MatrixViewTranspose : protected BaseView {
  typedef MatrixViewTranspose<BaseView> self_t;

  MatrixViewTranspose() : BaseView() {}
  MatrixViewTranspose(const self_t &view)
      : BaseView((const BaseView &)view) {}

  template <class View>
  friend struct MatrixViewBuilder;

  template<class CharT, class Traits>
  friend std::basic_ostream<CharT, Traits> &operator<<(
      std::basic_ostream<CharT, Traits> &out, const MatrixViewTranspose<BaseView> &view){
    return out << " [T]" << (const BaseView &)view;
  }

  inline const unsigned int rows(
      const unsigned int &_rows, const unsigned int &_columns) const noexcept {
    return BaseView::columns(_rows, _columns);
  }
  inline const unsigned int columns(
      const unsigned int &_rows, const unsigned int &_columns) const noexcept {
    return BaseView::rows(_rows, _columns);
  }
  inline unsigned int i(
      const unsigned int &i, const unsigned int &j) const noexcept {
    return BaseView::i(j, i);
  }
  inline unsigned int j(
      const unsigned int &i, const unsigned int &j) const noexcept {
    return BaseView::j(j, i);
  }
};

template <class BaseView>
struct MatrixViewPartial : protected BaseView {
  typedef MatrixViewPartial<BaseView> self_t;

  struct partial_prop_t {
    unsigned int rows, row_offset;
    unsigned int columns, column_offset;
    partial_prop_t()
        : rows(0), row_offset(0), columns(0), column_offset(0){}
    partial_prop_t(const partial_prop_t &prop)
        : rows(prop.rows), row_offset(prop.row_offset),
        columns(prop.columns), column_offset(prop.column_offset){}
  } partial_prop;

  MatrixViewPartial() : BaseView(), partial_prop() {}
  MatrixViewPartial(const self_t &view)
      : BaseView((const BaseView &)view), partial_prop(view.partial_prop) {
  }

  template <class View>
  friend struct MatrixViewBuilder;

  template<class CharT, class Traits>
  friend std::basic_ostream<CharT, Traits> &operator<<(
      std::basic_ostream<CharT, Traits> &out, const MatrixViewPartial<BaseView> &view){
    return out << " [P]("
         << view.partial_prop.rows << ","
         << view.partial_prop.columns << ","
         << view.partial_prop.row_offset << ","
         << view.partial_prop.column_offset << ")"
         << (const BaseView &)view;
  }

  inline const unsigned int rows(
      const unsigned int &_rows, const unsigned int &_columns) const noexcept {
    return partial_prop.rows;
  }
  inline const unsigned int columns(
      const unsigned int &_rows, const unsigned int &_columns) const noexcept {
    return partial_prop.columns;
  }
  inline unsigned int i(
      const unsigned int &i, const unsigned int &j) const noexcept {
    return BaseView::i(i + partial_prop.row_offset, j + partial_prop.column_offset);
  }
  inline unsigned int j(
      const unsigned int &i, const unsigned int &j) const noexcept {
    return BaseView::j(i + partial_prop.row_offset, j + partial_prop.column_offset);
  }
};


template <
    class T,
    class Array2D_Type = Array2D_Dense<T>,
    class ViewType = MatrixViewBase<> >
class Matrix;

/**
 * @brief Matrix for fixed content
 *
 * @see Matrix
 */
template <
    class T, class Array2D_Type,
    class ViewType = MatrixViewBase<> >
class Matrix_Frozen {
  public:
    typedef Array2D_Type storage_t;
    typedef Matrix_Frozen<T, Array2D_Type, ViewType> self_t;

    typedef MatrixViewProperty<ViewType> view_property_t;
    typedef typename view_property_t::self_t view_t;
    typedef MatrixViewBuilder<view_t> view_builder_t;

    typedef Matrix_Frozen<T, Array2D_Type,
        typename view_builder_t::transpose_t> transpose_t;
    typedef Matrix_Frozen<T, Array2D_Type,
        typename view_builder_t::partial_t> partial_t;

    typedef Matrix<T> downcast_default_t;

    template <class T2, class Array2D_Type2, class ViewType2>
    friend class Matrix_Frozen;

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
     * @param storage new storage
     */
    Matrix_Frozen(const Array2D_Frozen<T> &new_storage)
        : storage(static_cast<const storage_t &>(new_storage)),
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
      return storage.storage_t::operator()(
          view.i(row, column), view.j(row, column));
    }

    /**
     * Copy constructor generating shallow copy.
     *
     * @param matrix original
     */
    Matrix_Frozen(const self_t &matrix)
        : storage(matrix.storage),
        view(matrix.view){}

    template <class T2, class Array2D_Type2>
    Matrix_Frozen(const Matrix_Frozen<T2, Array2D_Type2, ViewType> &matrix)
        : storage(matrix.storage),
        view(matrix.view) {}
  protected:
    template <class ViewType2>
    Matrix_Frozen(const Matrix_Frozen<T, Array2D_Type, ViewType2> &matrix)
        : storage(matrix.storage),
        view() {
      view_builder_t::copy(view, matrix.view);
    }

  public:
    /**
     * Destructor
     */
    virtual ~Matrix_Frozen(){}

    /**
     * Generate scalar matrix
     *
     * @param size Row and column number
     * @param scalar
     */
    static Matrix_Frozen<T, Array2D_ScaledUnit<T> > getScalar(
        const unsigned int &size, const T &scalar){
      return Matrix_Frozen<T, Array2D_ScaledUnit<T> >(
          Array2D_ScaledUnit<T>(size, scalar));
    }

    /**
     * Generate unit matrix
     *
     * @param size Row and column number
     */
    static Matrix_Frozen<T, Array2D_ScaledUnit<T> > getI(const unsigned int &size){
      return getScalar(size, T(1));
    }

    /**
     * Down cast to Matrix by creating deep copy to make its content changeable
     *
     */
    template <class T2, class Array2D_Type2>
    operator Matrix<T2, Array2D_Type2>() const {
      return Matrix<T2, Array2D_Type2>::blank(rows(), columns()).replace_internal(*this);
    }

  protected:
    self_t &operator=(const self_t &matrix){
      if(this != &matrix){
        storage = matrix.storage;
        view = matrix.view;
      }
      return *this;
    }
    template <class T2, class Array2D_Type2>
    self_t &operator=(const Matrix_Frozen<T2, Array2D_Type2, ViewType> &matrix){
      storage = storage_t(matrix.storage);
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
      if(this == &matrix){return true;}
      if((rows() != matrix.rows())
          || columns() != matrix.columns()){
        return false;
      }
      for(unsigned int i(0); i < rows(); i++){
        for(unsigned int j(0); j < columns(); j++){
          if((*this)(i, j) != matrix(i, j)){
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
     * Test whether matrix is diagonal
     *
     * @return true when diagonal, otherwise false.
     */
    bool isDiagonal() const noexcept {
      if(isSquare()){
        for(unsigned int i(0); i < rows(); i++){
          for(unsigned int j(i + 1); j < columns(); j++){
            if(((*this)(i, j) != T(0)) || ((*this)(j, i) != T(0))){
              return false;
            }
          }
        }
        return true;
      }else{return false;}
    }

    /**
     * Test whether matrix is symmetric
     *
     * @return true when symmetric, otherwise false.
     */
    bool isSymmetric() const noexcept {
      if(isSquare()){
        for(unsigned int i(0); i < rows(); i++){
          for(unsigned int j(i + 1); j < columns(); j++){
            if((*this)(i, j) != (*this)(j, i)){return false;}
          }
        }
        return true;
      }else{return false;}
    }

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
      for(unsigned i(0); i < rows(); i++){
        tr += (*this)(i, i);
      }
      return tr;
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
      if(rows() * 2 != columns()){return false;}
      for(unsigned int i(0), i_U(rows()); i < rows() - 1; i++, i_U++){
        for(unsigned int j(i + 1); j < rows(); j++){
          if((*this)(i, j) != T(0)){return false;} // check L
          if((*this)(j, i_U) != T(0)){return false;} // check U
        }
      }
      return true;
    }

    /**
     * Generate transpose matrix
     *
     * @return Transpose matrix
     */
    transpose_t transpose() const noexcept {
      return transpose_t(*this);
    }

  protected:
    template <class MatrixT>
    static typename MatrixT::partial_t partial_internal(
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
      typename MatrixT::partial_t res(self);
      MatrixT::partial_t::view_builder_t::set_partial(
          res.view,
          new_rows, new_columns, row_offset, column_offset);
      return res;
    }

  public:
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

    enum {
      OPERATOR_2_Multiply_Matrix_by_Scalar,
      OPERATOR_2_Add_Matrix_to_Matrix,
      OPERATOR_2_Multiply_Matrix_by_Matrix,
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
        template <class OperatorT>
        struct check2_t<Array2D_Operator<T2, OperatorT> >{
          static const int tag = OperatorT::tag;
          typedef OperatorT operator_t;
        };
        static const int tag = check2_t<Array2D_Type2>::tag;
        typedef typename check2_t<Array2D_Type2>::operator_t operator_t;
      };
      static const int tag = check1_t<MatrixT>::tag;
      typedef typename check1_t<MatrixT>::operator_t operator_t;
    };

    template <class RHS_T>
    struct Multiply_Matrix_by_Scalar {
      struct op_t : public Array2D_Operator_Binary<self_t, RHS_T> {
        static const int tag = OPERATOR_2_Multiply_Matrix_by_Scalar;
        typedef Array2D_Operator_Binary<self_t, RHS_T> super_t;
        op_t(const self_t &mat, const RHS_T &scalar) noexcept
            : super_t(mat, scalar) {}
        T operator()(const unsigned int &row, const unsigned int &column) const noexcept {
          return super_t::lhs(row, column) * super_t::rhs;
        }
      };

      /*
       * Optimization policy: If upper pattern is M * scalar, then reuse it.
       * For example, (M * scalar) * scalar is transformed to M * (scalar * scalar).
       */

      template <bool, class U = void>
      struct optimizer_t {
        typedef Matrix_Frozen<T, Array2D_Operator<T, op_t> > res_t;
        static res_t generate(const self_t &mat, const RHS_T &scalar) noexcept {
          return res_t(
              typename res_t::storage_t(
                mat.rows(), mat.columns(), op_t(mat, scalar)));
        }
      };
#if 1 // 0 = Remove optimization
      template <class U>
      struct optimizer_t<true, U> {
        typedef self_t res_t;
        static res_t generate(const self_t &mat, const RHS_T &scalar) noexcept {
          return res_t(
              typename res_t::storage_t(
                mat.rows(), mat.columns(),
                typename OperatorProperty<res_t>::operator_t(
                  mat.storage.op.lhs, mat.storage.op.rhs * scalar)));
        }
      };
#endif

      typedef optimizer_t<
          OperatorProperty<self_t>::tag
            == OPERATOR_2_Multiply_Matrix_by_Scalar> opt_t;
      typedef typename opt_t::res_t mat_t;
      static mat_t generate(const self_t &mat, const RHS_T &scalar) noexcept {
        return opt_t::generate(mat, scalar);
      }
    };
    typedef Multiply_Matrix_by_Scalar<T> mul_mat_scalar_t;

    /**
     * Multiply by scalar
     *
     * @param scalar
     * @return multiplied matrix
     */
    typename mul_mat_scalar_t::mat_t operator*(const T &scalar) const noexcept {
      return mul_mat_scalar_t::generate(*this, scalar);
    }

    /**
     * Multiply by scalar
     *
     * @param scalar
     * @param matrix
     * @return multiplied matrix
     */
    friend typename mul_mat_scalar_t::mat_t operator*(const T &scalar, const self_t &matrix) noexcept {
      return matrix * scalar;
    }

    /**
     * Divide by scalar
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
      struct op_t : public Array2D_Operator_Binary<self_t, RHS_MatrixT> {
        static const int tag = OPERATOR_2_Add_Matrix_to_Matrix;
        typedef Array2D_Operator_Binary<self_t, RHS_MatrixT> super_t;
        op_t(const self_t &mat1, const RHS_MatrixT &mat2) noexcept
            : super_t(mat1, mat2) {}
        T operator()(const unsigned int &row, const unsigned int &column) const noexcept {
          if(rhs_positive){
            return super_t::lhs(row, column) + super_t::rhs(row, column);
          }else{
            return super_t::lhs(row, column) - super_t::rhs(row, column);
          }
        }
      };
      typedef Matrix_Frozen<T, Array2D_Operator<T, op_t> > mat_t;
      static mat_t generate(const self_t &mat1, const RHS_MatrixT &mat2){
        if(mat1.isDifferentSize(mat2)){throw std::invalid_argument("Incorrect size");}
        return mat_t(
            typename mat_t::storage_t(
              mat1.rows(), mat1.columns(), op_t(mat1, mat2)));
      }
    };

    /**
     * Add to matrix
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
     * Subtract from matrix
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


    template <class RHS_MatrixT>
    struct Multiply_Matrix_by_Matrix {

      template <class MatrixT, int tag = OperatorProperty<MatrixT>::tag>
      struct check_t {
        template <bool is_binary, class U = void>
        struct check_binary_t {
          static const bool has_multi_mat_by_mat = false;
        };
        template <class U>
        struct check_binary_t<true, U> {
          static const bool has_multi_mat_by_mat
              = (check_t<typename OperatorProperty<MatrixT>::operator_t::lhs_t>
                  ::has_multi_mat_by_mat)
                || (check_t<typename OperatorProperty<MatrixT>::operator_t::rhs_t>
                  ::has_multi_mat_by_mat);
        };
        static const bool has_multi_mat_by_mat
            = (tag == OPERATOR_2_Multiply_Matrix_by_Matrix)
              || check_binary_t<
                (tag == OPERATOR_2_Multiply_Matrix_by_Scalar)
                || (tag == OPERATOR_2_Add_Matrix_to_Matrix)
                || (tag == OPERATOR_2_Multiply_Matrix_by_Matrix)
                >::has_multi_mat_by_mat;
        static const bool is_multi_mat_by_scalar
            = (tag == OPERATOR_2_Multiply_Matrix_by_Scalar);
      };

      template <class LHS_T = self_t, class RHS_T = RHS_MatrixT>
      struct op_gen_t {

        /*
         * [Optimization policy 1]
         * If each side include M * M, then use cache.
         * For example, (M * M) * M, and (M * M + M) * M use cache for the first parenthesis terms.
         * (M * M + M) * (M * M + M) uses cache for the first and second parenthesis terms.
         */

        template <class MatrixT, bool cache_on = check_t<MatrixT>::has_multi_mat_by_mat>
        struct optimizer1_t {
          typedef MatrixT res_t;
        };
#if 1 // 0 = remove optimizer
        template <class MatrixT>
        struct optimizer1_t<MatrixT, true> {
          typedef typename MatrixT::downcast_default_t res_t;
        };
#endif
        typedef typename optimizer1_t<LHS_T>::res_t lhs_opt_t;
        typedef typename optimizer1_t<RHS_T>::res_t rhs_opt_t;

        struct op_t : public Array2D_Operator_Binary<LHS_T, RHS_T> {
          static const int tag = OPERATOR_2_Multiply_Matrix_by_Matrix;
          typedef Array2D_Operator_Binary<LHS_T, RHS_T> super_t;
          lhs_opt_t lhs_opt;
          rhs_opt_t rhs_opt;
          op_t(const LHS_T &mat1, const RHS_T &mat2) noexcept
              : super_t(mat1, mat2), // Preserve original operation on superclass
              lhs_opt(super_t::lhs), rhs_opt(super_t::rhs) {}
          T operator()(const unsigned int &row, const unsigned int &column) const noexcept {
            T res(0);
            for(unsigned int i(0); i < lhs_opt.columns(); ++i){
              res += lhs_opt(row, i) * rhs_opt(i, column);
            }
            return res;
          }
        };
      };

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
        typedef Matrix_Frozen<T, Array2D_Operator<T, typename op_gen_t<>::op_t> > res_t;
        static res_t generate(const self_t &mat1, const RHS_MatrixT &mat2){
          return res_t(
              typename res_t::storage_t(
                mat1.rows(), mat2.columns(),
                typename OperatorProperty<res_t>::operator_t(mat1, mat2)));
        }
      };
#if 1 // 0 = remove optimizer
      template <class U>
      struct optimizer2_t<true, false, U> {
        // (M * S) * M => (M * M) * S
        typedef typename OperatorProperty<self_t>::operator_t::lhs_t
            ::template Multiply_Matrix_by_Matrix<RHS_MatrixT>::mat_t stage1_t;
        typedef typename stage1_t
            ::template Multiply_Matrix_by_Scalar<
              typename OperatorProperty<self_t>::operator_t::rhs_t>::mat_t res_t;
        static res_t generate(const self_t &mat1, const RHS_MatrixT &mat2){
          stage1_t mat_stage1(
              typename stage1_t::storage_t(
                mat1.rows(), mat2.columns(),
                typename OperatorProperty<stage1_t>::operator_t(
                  mat1.storage.op.lhs, mat2)));
          return res_t(
              typename res_t::storage_t(
                mat1.rows(), mat2.columns(),
                typename OperatorProperty<res_t>::operator_t(
                  mat_stage1, mat1.storage.op.rhs)));
        }
      };
      template <class U>
      struct optimizer2_t<false, true, U> {
        // M * (M * S) => (M * M) * S
        typedef typename self_t
            ::template Multiply_Matrix_by_Matrix<
              typename OperatorProperty<RHS_MatrixT>::operator_t::lhs_t>::mat_t stage1_t;
        typedef typename stage1_t
            ::template Multiply_Matrix_by_Scalar<
              typename OperatorProperty<RHS_MatrixT>::operator_t::rhs_t>::mat_t res_t;
        static res_t generate(const self_t &mat1, const RHS_MatrixT &mat2){
          stage1_t mat_stage1(
              typename stage1_t::storage_t(
                mat1.rows(), mat2.columns(),
                typename OperatorProperty<stage1_t>::operator_t(
                  mat1, mat2.storage.op.lhs)));
          return res_t(
              typename res_t::storage_t(
                mat1.rows(), mat2.columns(),
                typename OperatorProperty<res_t>::operator_t(
                  mat_stage1, mat2.storage.op.rhs)));
        }
      };
      template <class U>
      struct optimizer2_t<true, true, U> {
        // (M * S) * (M * S) => (M * M) * (S * S)
        typedef typename OperatorProperty<self_t>::operator_t::lhs_t
            ::template Multiply_Matrix_by_Matrix<
              typename OperatorProperty<RHS_MatrixT>::operator_t::lhs_t>::mat_t stage1_t;
        typedef typename stage1_t
            ::template Multiply_Matrix_by_Scalar<
              typename OperatorProperty<self_t>::operator_t::rhs_t>::mat_t res_t;
        static res_t generate(const self_t &mat1, const RHS_MatrixT &mat2){
          stage1_t mat_stage1(
              typename stage1_t::storage_t(
                mat1.rows(), mat2.columns(),
                typename OperatorProperty<stage1_t>::operator_t(
                  mat1.storage.op.lhs, mat2.storage.op.lhs)));
          return res_t(
              typename res_t::storage_t(
                mat1.rows(), mat2.columns(),
                typename OperatorProperty<res_t>::operator_t(
                  mat_stage1, mat1.storage.op.rhs * mat2.storage.op.rhs)));
        }
      };
#endif

      typedef typename optimizer2_t<>::res_t mat_t;
      static mat_t generate(const self_t &mat1, const RHS_MatrixT &mat2){
        if(mat1.columns() != mat2.rows()){throw std::invalid_argument("Incorrect size");}
        return optimizer2_t<>::generate(mat1, mat2);
      }
    };

    /**
     * Multiply by matrix
     *
     * @param matrix matrix to multiply
     * @return multiplied matrix
     * @throw std::invalid_argument When operation is undefined
     */
    template <class T2, class Array2D_Type2, class ViewType2>
    typename Multiply_Matrix_by_Matrix<Matrix_Frozen<T2, Array2D_Type2, ViewType2> >::mat_t
        operator*(const Matrix_Frozen<T2, Array2D_Type2, ViewType2> &matrix) const {
      return Multiply_Matrix_by_Matrix<Matrix_Frozen<T2, Array2D_Type2, ViewType2> >::generate(*this, matrix);
    }


    downcast_default_t matrix_for_minor(
        const unsigned int &row,
        const unsigned int &column) const noexcept {
      return ((downcast_default_t)*this).matrix_for_minor(row, column);
    }

    T determinant_minor(const bool &do_check = true) const {
      return ((downcast_default_t)*this).determinant_minor(do_check);
    }

    downcast_default_t decomposeLUP(
        unsigned int &pivot_num,
        unsigned int *pivot = NULL,
        const bool &do_check = true) const {
      return ((downcast_default_t)*this).decomposeLUP(pivot_num, pivot, do_check);
    }

    downcast_default_t decomposeLU(const bool &do_check = true) const {
      return ((downcast_default_t)*this).decomposeLU(do_check);
    }

    T determinant_LU(const bool &do_check = true) const {
      return ((downcast_default_t)*this).determinant_LU(do_check);
    }

    T determinant(const bool &do_check = true) const {
      return ((downcast_default_t)*this).determinant(do_check);
    }

    downcast_default_t decomposeUD(const bool &do_check = true) const {
      return ((downcast_default_t)*this).decomposeUD(do_check);
    }

    downcast_default_t inverse() const {
      return ((downcast_default_t)*this).inverse();
    }

    /**
     * Divide by matrix, in other words, multiply by inverse matrix
     *
     * @param matrix Matrix to divide
     * @return divided matrix
     */
    template <class T2, class Array2D_Type2, class ViewType2>
    typename Multiply_Matrix_by_Matrix<
          typename Matrix_Frozen<T2, Array2D_Type2, ViewType2>::downcast_default_t>::mat_t
        operator/(const Matrix_Frozen<T2, Array2D_Type2, ViewType2> &matrix) const {
      return Multiply_Matrix_by_Matrix<
            typename Matrix_Frozen<T2, Array2D_Type2, ViewType2>::downcast_default_t>::generate(
          *this, matrix.inverse());
    }

    /**
     * Divide by matrix, in other words, multiply by inverse matrix
     *
     * @param matrix Matrix to divide
     * @return divided matrix
     */
    template <class T2, class Array2D_Type2, class ViewType2>
    typename Multiply_Matrix_by_Matrix<
          typename Matrix<T2, Array2D_Type2, ViewType2>::viewless_t>::mat_t
        operator/(const Matrix<T2, Array2D_Type2, ViewType2> &matrix) const {
      return Multiply_Matrix_by_Matrix<
            typename Matrix<T2, Array2D_Type2, ViewType2>::viewless_t>::generate(
          *this, matrix.inverse());
    }

    /**
     * Print matrix
     *
     */
    template<class CharT, class Traits>
    friend std::basic_ostream<CharT, Traits> &operator<<(
        std::basic_ostream<CharT, Traits> &out, const self_t &matrix){
      out << "{";
      for(unsigned int i(0); i < matrix.rows(); i++){
        out << (i == 0 ? "" : ",") << std::endl << "{";
        for(unsigned int j(0); j < matrix.columns(); j++){
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
          return (*this) << "M"
              << (MatrixViewProperty<View_Type2>::transposed ? "t" : "")
              << (MatrixViewProperty<View_Type2>::partialized ? "p" : "")
              << "(" << m.rows() << "," << m.columns() << ")";
        }
        template <class LHS_T, class RHS_T>
        format_t &operator<<(const Array2D_Operator_Binary<LHS_T, RHS_T> &op){
          return (*this) << op.lhs << ", " << op.rhs;
        }
        template <class T2, class OperatorT, class View_Type2>
        format_t &operator<<(
            const Matrix_Frozen<T2, Array2D_Operator<T2, OperatorT>, View_Type2> &m){
          const char *symbol = "";
          switch(OperatorProperty<
              Matrix_Frozen<T2, Array2D_Operator<T2, OperatorT>, View_Type2> >::tag){
            case OPERATOR_2_Multiply_Matrix_by_Scalar:
            case OPERATOR_2_Multiply_Matrix_by_Matrix:
              symbol = "*"; break;
            case OPERATOR_2_Add_Matrix_to_Matrix:
              symbol = "+"; break;
            default:
              return (*this) << "(?)";
          }
          return (*this) << "(" << symbol << ", "
              << (typename OperatorT::super_t)m.storage.op << ")"; // cast to Array2D_Operator_Binary
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
    typedef typename super_t::view_builder_t view_builder_t;
    typedef typename super_t::view_property_t view_property_t;
#else
    using typename super_t::storage_t;
    using typename super_t::view_builder_t;
    using typename super_t::view_property_t;
#endif

    typedef Matrix<T, Array2D_Type, ViewType> self_t;

    typedef Matrix<T, Array2D_Type> viewless_t;
    typedef Matrix<T, Array2D_Type,
        typename view_builder_t::transpose_t> transpose_t;
    typedef Matrix<T, Array2D_Type,
        typename view_builder_t::partial_t> partial_t;

    template <class T2, class Array2D_Type2, class ViewType2>
    friend class Matrix_Frozen;

    template <class T2, class Array2D_Type2, class ViewType2>
    friend class Matrix;

  protected:
    /**
     * Constructor with storage
     *
     * @param storage new storage
     */
    Matrix(const Array2D<T, Array2D_Type> &new_storage) : super_t(new_storage) {}

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
      return storage.storage_t::operator()(
            super_t::view.i(row, column), super_t::view.j(row, column));
    }

    using super_t::rows;
    using super_t::columns;

    /**
     * Clear elements.
     *
     */
    void clear(){
      if(view_property_t::truncated){
        for(unsigned int i(0); i < rows(); i++){
          for(unsigned int j(0); j < columns(); j++){
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
      /* Array2D_Type is intentionally used instead of storage_t due to VC10(C2514) */
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
      /* Array2D_Type is intentionally used instead of storage_t due to VC10(C2514) */
    }

    /**
     * Copy constructor generating shallow copy.
     *
     * @param matrix original
     */
    Matrix(const self_t &matrix)
        : super_t(matrix){}

    template <class T2, class Array2D_Type2>
    Matrix(const Matrix<T2, Array2D_Type2, ViewType> &matrix)
        : super_t(matrix) {}
  protected:
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
    static viewless_t blank(
        const unsigned int &new_rows,
        const unsigned int &new_columns){
      /* Array2D_Type is intentionally used instead of storage_t due to VC10(C2597) */
      return viewless_t(Array2D_Type(new_rows, new_columns));
    }

  protected:
    viewless_t blank_copy() const {
      return blank(rows(), columns());
    }

    template <class T2, class Array2D_Type2, class ViewType2>
    self_t &replace_internal(const Matrix_Frozen<T2, Array2D_Type2, ViewType2> &matrix){
      for(unsigned int i(0); i < rows(); ++i){
        for(unsigned int j(0); j < columns(); ++j){
          (*this)(i, j) = (T)matrix(i, j);
        }
      }
      return *this;
    }

  public:
    /**
     * Assign operator performing shallow copy.
     *
     * @return myself
     */
    self_t &operator=(const self_t &matrix){
      super_t::operator=(matrix);
      return *this;
    }
    template <class T2, class Array2D_Type2>
    self_t &operator=(const Matrix<T2, Array2D_Type2, ViewType> &matrix){
      super_t::operator=(matrix);
      return *this;
    }

    /**
     * Perform (deep) copy
     *
     * @return (viewless_t)
     */
    viewless_t copy() const {
      if(view_property_t::viewless){
        return viewless_t(storage.storage_t::copy(true));
      }else{
        return blank_copy().replace_internal(*this);
      }
    }

  protected:
    /**
     * Cast to another Matrix defined in Matrix_Frozen is intentionally protected.
     *
     * Use copy() for Matrix
     */
    template <class T2, class Array2D_Type2>
    operator Matrix<T2, Array2D_Type2>() const;

  public:
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
     * Exchange rows (bang method).
     *
     * @param row1 Target row (1)
     * @param row2 Target row (2)
     * @return myself
     * @throw std::out_of_range When row1 or row2 exceeds bound
     */
    self_t &exchangeRows(
        const unsigned int &row1, const unsigned int &row2){
      if(row1 >= rows() || row2 >= rows()){
        throw std::out_of_range("Row index incorrect");
      }
      T temp;
      for(unsigned int j(0); j < columns(); j++){
        temp = (*this)(row1, j);
        (*this)(row1, j) = (*this)(row2, j);
        (*this)(row2, j) = temp;
      }
      return *this;
    }

    /**
     * Exchange columns (bang method).
     *
     * @param column1 Target column (1)
     * @param column2 Target column (2)
     * @return myself
     * @throw std::out_of_range When column1 or column2 exceeds bound
     */
    self_t &exchangeColumns(
        const unsigned int &column1, const unsigned int &column2){
      if(column1 >= columns() || column2 >= columns()){
        throw std::out_of_range("Column index incorrect");
      }
      T temp;
      for(unsigned int i(0); i < rows(); i++){
        temp = (*this)(i, column1);
        (*this)(i, column1) = (*this)(i, column2);
        (*this)(i, column2) = temp;
      }
      return *this;
    }

    /**
     * Replace content
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
      return replace_internal(matrix);
    }

    using super_t::isSquare;
    using super_t::isDiagonal;
    using super_t::isSymmetric;
    using super_t::isDifferentSize;
    using super_t::isLU;

    /**
     * Multiply by scalar (bang method)
     *
     * @param scalar
     * @return myself
     */
    self_t &operator*=(const T &scalar) noexcept {
      return replace_internal((*this) * scalar);
    }

    /**
     * Divide by scalar (bang method)
     *
     * @param scalar
     * @return myself
     */
    self_t &operator/=(const T &scalar) noexcept {
      return operator*=(T(1) / scalar);
    }
    
    /**
     * Add to matrix (bang method)
     *
     * @param matrix Matrix to add
     * @return myself
     */
    template <class T2, class Array2D_Type2, class ViewType2>
    self_t &operator+=(const Matrix_Frozen<T2, Array2D_Type2, ViewType2> &matrix){
      return replace_internal((*this) + matrix);
    }
    
    /**
     * Subtract from matrix (bang method)
     *
     * @param matrix Matrix to subtract
     * @return myself
     */
    template <class T2, class Array2D_Type2, class ViewType2>
    self_t &operator-=(const Matrix_Frozen<T2, Array2D_Type2, ViewType2> &matrix){
      return replace_internal((*this) - matrix);
    }
    
    /**
     * Multiply by matrix (bang method)
     *
     * @param matrix Matrix to multiply
     * @return myself
     */
    template <class T2, class Array2D_Type2, class ViewType2>
    self_t &operator*=(const Matrix_Frozen<T2, Array2D_Type2, ViewType2> &matrix){
      return operator=(*this * matrix);
    }

    /**
     * Generate a matrix in which i-th row and j-th column are removed to calculate minor (determinant)
     *
     * @param row Row to be removed
     * @param column Column to be removed
     * @return Removed matrix
     */
    viewless_t matrix_for_minor(
        const unsigned int &row,
        const unsigned int &column) const noexcept {
      viewless_t res(blank(rows() - 1, columns() - 1));
      unsigned int i(0), i2(0);
      for( ; i < row; ++i, ++i2){
        unsigned int j(0), j2(0);
        for( ; j < column; ++j, ++j2){
          res(i, j) = operator()(i2, j2);
        }
        ++j2;
        for( ; j < res.columns(); ++j, ++j2){
          res(i, j) = operator()(i2, j2);
        }
      }
      ++i2;
      for( ; i < res.rows(); ++i, ++i2){
        unsigned int j(0), j2(0);
        for( ; j < column; ++j, ++j2){
          res(i, j) = operator()(i2, j2);
        }
        ++j2;
        for( ; j < res.columns(); ++j, ++j2){
          res(i, j) = operator()(i2, j2);
        }
      }
      return res;
    }

    /**
     * Calculate determinant by using minor
     *
     * @param do_check Whether check size property. The default is true.
     * @return Determinant
     * @throw std::logic_error When operation is undefined
     */
    T determinant_minor(const bool &do_check = true) const {
      if(do_check && !isSquare()){throw std::logic_error("rows() != columns()");}
      if(rows() == 1){
        return (*this)(0, 0);
      }else{
        T sum(0);
        T sign(1);
        for(unsigned int i(0); i < rows(); i++){
          if((*this)(i, 0) != T(0)){
            sum += (*this)(i, 0) * (matrix_for_minor(i, 0).determinant(false)) * sign;
          }
          sign = -sign;
        }
        return sum;
      }
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
    typename Matrix<T2, Array2D_Type2, ViewType2>::viewless_t
        solve_linear_eq_with_LU(
            const Matrix<T2, Array2D_Type2, ViewType2> &y, const bool &do_check = true)
            const {
      if(do_check && (!isLU())){
        throw std::logic_error("Not LU decomposed matrix!!");
      }
      if((y.columns() != 1)
          || (y.rows() != rows())){
        throw std::invalid_argument("Incorrect y size");
      }

      partial_t
          L(partial(rows(), rows(), 0, 0)),
          U(partial(rows(), rows(), 0, rows()));
      typedef typename Matrix<T2, Array2D_Type2>::viewless_t y_t;
      // L(Ux) = y で y' = (Ux)をまず解く
      y_t y_copy(y.copy());
      y_t y_prime(y_t::blank(y.rows(), 1));
      for(unsigned i(0); i < rows(); i++){
        y_prime(i, 0) = y_copy(i, 0) / L(i, i);
        for(unsigned j(i + 1); j < rows(); j++){
          y_copy(j, 0) -= L(j, i) * y_prime(i, 0);
        }
      }

      // 続いてUx = y'で xを解く
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
     * Perform decomposition of LUP
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
    viewless_t decomposeLUP(
        unsigned int &pivot_num,
        unsigned int *pivot = NULL,
        const bool &do_check = true) const {
      if(do_check && !isSquare()){throw std::logic_error("rows() != columns()");}
      viewless_t LU(blank(rows(), columns() * 2));
#define L(i, j) LU(i, j)
#define U(i, j) LU(i, j + columns())
      for(unsigned int i(0); i < rows(); ++i){
        U(i, i) = (*this)(i, i);
        L(i, i) = T(1);
        for(unsigned int j(i + 1); j < rows(); ++j){
          U(i, j) = (*this)(i, j);
          U(j, i) = (*this)(j, i); // U is full copy
          L(i, j) = T(0);
        }
      }
      pivot_num = 0;
      if(pivot){
        for(unsigned int i(0); i < rows(); ++i){
          pivot[i] = i;
        }
      }
      // apply Gaussian elimination
      for(unsigned int i(0); i < rows(); ++i){
        if(U(i, i) == T(0)){ // check (i, i) is not zero
          unsigned int j(i);
          do{
            if(++j == rows()){
              throw std::runtime_error("LU decomposition cannot be performed");
            }
          }while(U(i, j) == T(0));
          for(unsigned int i2(0); i2 < rows(); ++i2){ // exchange i-th and j-th columns
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
        for(unsigned int i2(i + 1); i2 < rows(); ++i2){
          L(i2, i) = U(i2, i) / U(i, i);
          U(i2, i) = T(0);
          for(unsigned int j2(i + 1); j2 < rows(); ++j2){
            U(i2, j2) -= L(i2, i) * U(i, j2);
          }
        }
      }
#undef L
#undef U

      return LU;
    }

    viewless_t decomposeLU(const bool &do_check = true) const {
      unsigned int pivot_num;
      return decomposeLUP(pivot_num, NULL, do_check);
    }

    /**
     * Calculate determinant by using LU decomposition
     *
     * @param do_check Whether check size property. The default is true.
     * @return Determinant
     */
    T determinant_LU(const bool &do_check = true) const {
      unsigned int pivot_num;
      viewless_t LU(decomposeLUP(pivot_num, NULL, do_check));
      T res((pivot_num % 2 == 0) ? 1 : -1);
      for(unsigned int i(0), j(rows()); i < rows(); ++i, ++j){
        res *= LU(i, i) * LU(i, j);
      }
      return res;
    }

    T determinant(const bool &do_check = true) const {
      return determinant_LU(do_check);
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
    viewless_t decomposeUD(const bool &do_check = true) const {
      if(do_check && !isSymmetric()){throw std::logic_error("not symmetric");}
      viewless_t P(copy());
      viewless_t UD(rows(), columns() * 2);
#define U(i, j) UD(i, j)
#define D(i, j) UD(i, j + columns())
      for(int i(rows() - 1); i >= 0; i--){
        D(i, i) = P(i, i);
        U(i, i) = T(1);
        for(unsigned int j(0); j < i; j++){
          U(j, i) = P(j, i) / D(i, i);
          for(unsigned int k(0); k <= j; k++){
            P(k, j) -= U(k, i) * D(i, i) * U(j, i);
          }
        }
      }
#undef U
#undef D
      return UD;
    }

    /**
     * Calculate inverse matrix
     *
     * @return Inverse matrix
     * @throw std::logic_error When operation is undefined
     * @throw std::runtime_error When operation is unavailable
     */
    viewless_t inverse() const {

      if(!isSquare()){throw std::logic_error("rows() != columns()");}

      //クラメール(遅い)
      /*
      viewless_t result(rows(), columns());
      T det;
      if((det = determinant()) == 0){throw std::runtime_error("Operation void!!");}
      for(unsigned int i(0); i < rows(); i++){
        for(unsigned int j(0); j < columns(); j++){
          result(i, j) = coMatrix(i, j).determinant() * ((i + j) % 2 == 0 ? 1 : -1);
        }
      }
      return result.transpose() / det;
      */

      //ガウス消去法
      viewless_t left(copy());
      viewless_t right(viewless_t::getI(rows()));
      for(unsigned int i(0); i < rows(); i++){
        if(left(i, i) == T(0)){
          unsigned int i2(i);
          do{
            if(++i2 == rows()){
              throw std::runtime_error("invert matrix not exist");
            }
          }while(left(i2, i) == T(0));
          // exchange i-th and i2-th rows
          for(unsigned int j(i); j < columns(); ++j){
            T temp(left(i, j));
            left(i, j) = left(i2, j);
            left(i2, j) = temp;
          }
          right.exchangeRows(i, i2);
        }
        if(left(i, i) != T(1)){
          for(unsigned int j(0); j < columns(); j++){right(i, j) /= left(i, i);}
          for(unsigned int j(i+1); j < columns(); j++){left(i, j) /= left(i, i);}
          left(i, i) = T(1);
        }
        for(unsigned int k(0); k < rows(); k++){
          if(k == i){continue;}
          if(left(k, i) != T(0)){
            for(unsigned int j(0); j < columns(); j++){right(k, j) -= right(i, j) * left(k, i);}
            for(unsigned int j(i+1); j < columns(); j++){left(k, j) -= left(i, j) * left(k, i);}
            left(k, i) = T(0);
          }
        }
      }
      //std::cout << "L:" << left << std::endl;
      //std::cout << "R:" << right << std::endl;

      return right;

      //LU分解
      /*
      */
    }

    /**
     * Scalar divided by matrix, which is equivalent to inverted matrix multiplied by scalar
     *
     * @param scalar
     * @param matrix
     * @return result matrix
     */
    friend typename viewless_t::super_t::mul_mat_scalar_t::mat_t
        operator/(const T &scalar, const self_t &matrix) {
      return matrix.inverse() * scalar;
    }

    /**
     * Divide by matrix, in other words, multiply by inverse matrix. (bang method)
     *
     * @param matrix Matrix to divide
     * @return myself
     */
    template <class T2, class Array2D_Type2, class ViewType2>
    self_t &operator/=(const Matrix_Frozen<T2, Array2D_Type2, ViewType2> &matrix) {
      return operator=(*this / matrix);
    }

    /**
     * Divide by matrix, in other words, multiply by inverse matrix. (bang method)
     *
     * @param matrix Matrix to divide
     * @return myself
     */
    template <class T2, class Array2D_Type2, class ViewType2>
    self_t &operator/=(const Matrix<T2, Array2D_Type2, ViewType2> &matrix) {
      return operator=(*this / matrix);
    }

    /**
     * Add by matrix with specified pivot (bang method)
     *
     * @param row Upper row index (pivot) of matrix to be added
     * @param column Left column index (pivot) of matrix to be added
     * @param matrix Matrix to add
     * @return myself
     */
    template <class T2, class Array2D_Type2, class ViewType2>
    self_t &pivotMerge(
        const unsigned int &row, const unsigned int &column,
        const Matrix_Frozen<T2, Array2D_Type2, ViewType2> &matrix){
      for(int i(0); i < matrix.rows(); i++){
        if(row + i < 0){continue;}
        else if(row + i >= rows()){break;}
        for(int j(0); j < matrix.columns(); j++){
          if(column + j < 0){continue;}
          else if(column + j >= columns()){break;}
          (*this)(row + i, column + j) += matrix(i, j);
        }
      }
      return *this;
    }

    /**
     * Add by matrix with specified pivot
     *
     * @param row Upper row index (pivot) of matrix to be added
     * @param column Left column index (pivot) of matrix to be added
     * @param matrix Matrix to add
     * @return added (deep) copy
     */
    template <class T2, class Array2D_Type2, class ViewType2>
    viewless_t pivotAdd(
        const unsigned int &row, const unsigned int &column,
        const Matrix_Frozen<T2, Array2D_Type2, ViewType2> &matrix) const{
      return copy().pivotMerge(row, column, matrix);
    }

    /**
     * Calculate Hessenberg matrix by performing householder conversion
     *
     * @param transform Pointer to store multiplication of matrices used for the conversion.
     * If NULL is specified, the store will not be performed, The default is NULL.
     * @return Hessenberg matrix
     * @throw std::logic_error When operation is undefined
     */
    viewless_t hessenberg(viewless_t *transform = NULL) const {
      if(!isSquare()){throw std::logic_error("rows() != columns()");}

      viewless_t result(copy());
      for(unsigned int j(0); j < columns() - 2; j++){
        T t(0);
        for(unsigned int i(j + 1); i < rows(); i++){
          t += pow(result(i, j), 2);
        }
        T s = ::sqrt(t);
        if(result(j + 1, j) < 0){s *= -1;}

        viewless_t omega(blank(rows() - (j+1), 1));
        {
          for(unsigned int i(0); i < omega.rows(); i++){
            omega(i, 0) = result(j+i+1, j);
          }
          omega(0, 0) += s;
        }

        viewless_t P(viewless_t::getI(rows()));
        T denom(t + result(j + 1, j) * s);
        if(denom){
          P.pivotMerge(j+1, j+1, -(omega * omega.transpose() / denom));
        }

        result = P * result * P;
        if(transform){(*transform) *= P;}
      }

      //ゼロ処理
      bool sym = isSymmetric();
      for(unsigned int j(0); j < columns() - 2; j++){
        for(unsigned int i(j + 2); i < rows(); i++){
          result(i, j) = T(0);
          if(sym){result(j, i) = T(0);}
        }
      }

      return result;
    }


    struct complex_t {
      template <class T2>
      struct check_t {
        static const bool hit = false;
        typedef Complex<T2> res_t;
      };
      template <class T2>
      struct check_t<Complex<T2> > {
        static const bool hit = true;
        typedef Complex<T2> res_t;
      };
      static const bool is_complex = check_t<T>::hit;
      typedef typename check_t<T>::res_t v_t;
      typedef typename Matrix<
          v_t,
          typename Array2D_Type::template family_t<v_t>::res_t,
          ViewType>::viewless_t m_t;
    };

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
        typename complex_t::v_t &upper, typename complex_t::v_t &lower) const {
      T a((*this)(row, column)),
        b((*this)(row, column + 1)),
        c((*this)(row + 1, column)),
        d((*this)(row + 1, column + 1));
      T root2(pow((a - d), 2) + b * c * 4);
      if(complex_t::is_complex || (root2 > 0)){
        T root(::sqrt(root2));
        upper = ((a + d + root) / 2);
        lower = ((a + d - root) / 2);
      }else{
        T root(::sqrt(root2 * -1));
        upper = typename complex_t::v_t((a + d) / 2, root / 2);
        lower = typename complex_t::v_t((a + d) / 2, root / 2 * -1);
      }
    }

    /**
     * Calculate eigenvalues and eigenvectors.
     * The return matrix consists of
     * (0,j)-(n-1,j): Eigenvector (j) (0 <= j <= n-1)
     * (j,n)-(j,n): Eigenvalue (j)
     *
     * @param threshold_abs Absolute error to be used for convergence determination
     * @param threshold_rel Relative error to be used for convergence determination
     * @return Eigenvalues and eigenvectors
     * @throw std::logic_error When operation is undefined
     * @throw std::runtime_error When operation is unavailable
     */
    typename complex_t::m_t eigen(
        const T &threshold_abs = 1E-10,
        const T &threshold_rel = 1E-7) const {

      typedef typename complex_t::m_t res_t;

      if(!isSquare()){throw std::logic_error("rows() != columns()");}

      //パワー法(べき乗法)
      /*viewless_t result(rows(), rows() + 1);
      viewless_t source = copy();
      for(unsigned int i(0); i < columns(); i++){result(0, i) = T(1);}
      for(unsigned int i(0); i < columns(); i++){
        while(true){
          viewless_t approxVec = source * result.columnVector(i);
          T approxVal(0);
          for(unsigned int j(0); j < approxVec.rows(); j++){approxVal += pow(approxVec(j, 0), 2);}
          approxVal = sqrt(approxVal);
          for(unsigned int j(0); j < approxVec.rows(); j++){result(j, i) = approxVec(j, 0) / approxVal;}
          T before = result(i, rows());
          if(abs(before - (result(i, rows()) = approxVal)) < threshold){break;}
        }
        for(unsigned int j(0); (i < rows() - 1) && (j < rows()); j++){
          for(unsigned int k(0); k < rows(); k++){
            source(j, k) -= result(i, rows()) * result(j, i) * result(k, i);
          }
        }
      }
      return result;*/

      //ダブルQR法
      /* <手順>
       * ハウスホルダー法を適用して、上ヘッセンベルク行列に置換後、
       * ダブルQR法を適用。
       * 結果、固有値が得られるので、固有ベクトルを計算。
       */

      const unsigned int &_rows(rows());

      //結果の格納用の行列
      res_t result(_rows, _rows + 1);

      //固有値の計算
#define lambda(i) result(i, _rows)

      T mu_sum(0), mu_multi(0);
      typename complex_t::v_t p1, p2;
      int m = _rows;
      bool first = true;

      viewless_t transform(super_t::getI(_rows));
      viewless_t A(hessenberg(&transform));
      viewless_t A_(A);

      while(true){

        //m = 1 or m = 2
        if(m == 1){
          lambda(0) = A(0, 0);
          break;
        }else if(m == 2){
          A.eigen22(0, 0, lambda(0), lambda(1));
          break;
        }

        //μ、μ*の更新(4.143)
        {
          typename complex_t::v_t p1_new, p2_new;
          A.eigen22(m-2, m-2, p1_new, p2_new);
          if(first ? (first = false) : true){
            if((p1_new - p1).abs() > p1_new.abs() / 2){
              if((p2_new - p2).abs() > p2_new.abs() / 2){
                mu_sum = (p1 + p2).real();
                mu_multi = (p1 * p2).real();
              }else{
                mu_sum = p2_new.real() * 2;
                mu_multi = pow(p2_new.real(), 2);
              }
            }else{
              if((p2_new - p2).abs() > p2_new.abs() / 2){
                mu_sum = p1_new.real() * 2;
                mu_multi = p1_new.real() * p1_new.real();
              }else{
                mu_sum = (p1_new + p2_new).real();
                mu_multi = (p1_new * p2_new).real();
              }
            }
          }
          p1 = p1_new, p2 = p2_new;
        }

        //ハウスホルダー変換を繰り返す
        T b1, b2, b3, r;
        for(int i(0); i < m - 1; i++){
          if(i == 0){
            b1 = A(0, 0) * A(0, 0) - mu_sum * A(0, 0) + mu_multi + A(0, 1) * A(1, 0);
            b2 = A(1, 0) * (A(0, 0) + A(1, 1) - mu_sum);
            b3 = A(2, 1) * A(1, 0);
          }else{
            b1 = A(i, i - 1);
            b2 = A(i + 1, i - 1);
            b3 = (i == m - 2 ? T(0) : A(i + 2, i - 1));
          }

          r = ::sqrt((b1 * b1) + (b2 * b2) + (b3 * b3));

          viewless_t omega(3, 1);
          {
            omega(0, 0) = b1 + r * (b1 >= T(0) ? 1 : -1);
            omega(1, 0) = b2;
            if(b3 != T(0)){omega(2, 0) = b3;}
          }
          viewless_t P(viewless_t::getI(_rows));
          T denom((omega.transpose() * omega)(0, 0));
          if(denom){
            P.pivotMerge(i, i, omega * omega.transpose() * -2 / denom);
          }
          //std::cout << "denom(" << m << ") " << denom << std::endl;

          A = P * A * P;
        }
        //std::cout << "A_scl(" << m << ") " << A(m-1,m-2) << std::endl;

#if defined(_MSC_VER)
        if(_isnan(A(m-1,m-2)) || !_finite(A(m-1,m-2))){
#else
        if(std::isnan(A(m-1,m-2)) || !std::isfinite(A(m-1,m-2))){
#endif
          throw std::runtime_error("eigen values calculation failed");
        }

        //収束判定
#define _abs(x) ((x) >= 0 ? (x) : -(x))
        T A_m2_abs(_abs(A(m-2, m-2))), A_m1_abs(_abs(A(m-1, m-1)));
        T epsilon(threshold_abs
          + threshold_rel * ((A_m2_abs < A_m1_abs) ? A_m2_abs : A_m1_abs));

        //std::cout << "epsil(" << m << ") " << epsilon << std::endl;

        if(_abs(A(m-1, m-2)) < epsilon){
          --m;
          lambda(m) = A(m, m);
        }else if(_abs(A(m-2, m-3)) < epsilon){
          A.eigen22(m-2, m-2, lambda(m-1), lambda(m-2));
          m -= 2;
        }
      }
#undef _abs

#if defined(MATRIX_EIGENVEC_SIMPLE)
      //固有ベクトルの計算
      res_t x(_rows, _rows);  //固有ベクトル
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
      //固有ベクトルの計算(逆反復法)
      res_t x(res_t::getI(_rows));  //固有ベクトル
      A = A_;
      res_t A_C(_rows, _rows);
      for(unsigned int i(0); i < _rows; i++){
        for(unsigned int j(0); j < columns(); j++){
          A_C(i, j) = A(i, j);
        }
      }

      for(unsigned int j(0); j < _rows; j++){
        // http://www.prefield.com/algorithm/math/eigensystem.html を参考に
        // かつ、固有値が等しい場合の対処方法として、
        // http://www.nrbook.com/a/bookcpdf/c11-7.pdf
        // を参考に、値を振ってみることにした
        res_t A_C_lambda(A_C.copy());
        typename complex_t::v_t approx_lambda(lambda(j));
        if((A_C_lambda(j, j) - approx_lambda).abs() <= 1E-3){
          approx_lambda += 2E-3;
        }
        for(unsigned int i(0); i < _rows; i++){
          A_C_lambda(i, i) -= approx_lambda;
        }
        res_t A_C_lambda_LU(A_C_lambda.decomposeLU());

        res_t target_x(res_t::blank(_rows, 1));
        for(unsigned i(0); i < _rows; ++i){
          target_x(i, 0) = x(i, j);
        }
        for(unsigned loop(0); true; loop++){
          res_t target_x_new(
              A_C_lambda_LU.solve_linear_eq_with_LU(target_x, false));
          T mu((target_x_new.transpose() * target_x)(0, 0).abs2()),
            v2((target_x_new.transpose() * target_x_new)(0, 0).abs2()),
            v2s(::sqrt(v2));
          for(unsigned i(0); i < _rows; ++i){
            target_x(i, 0) = target_x_new(i, 0) / v2s;
          }
          //std::cout << mu << ", " << v2 << std::endl;
          //std::cout << target_x.transpose() << std::endl;
          if((T(1) - (mu * mu / v2)) < T(1.1)){
            for(unsigned i(0); i < _rows; ++i){
              x(i, j) = target_x(i, 0);
            }
            break;
          }
          if(loop > 100){
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

      //結果の格納
      for(unsigned int j(0); j < x.columns(); j++){
        for(unsigned int i(0); i < x.rows(); i++){
          for(unsigned int k(0); k < transform.columns(); k++){
            result(i, j) += transform(i, k) * x(k, j);
          }
        }

        //正規化
        typename complex_t::v_t _norm;
        for(unsigned int i(0); i < _rows; i++){
          _norm += result(i, j).abs2();
        }
        T norm = ::sqrt(_norm.real());
        for(unsigned int i(0); i < _rows; i++){
          result(i, j) /= norm;
        }
        //std::cout << result.partial(_rows, 1, 0, j).transpose() << std::endl;
      }
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
    static typename complex_t::m_t sqrt(
        const typename complex_t::m_t &eigen_mat){
      unsigned int n(eigen_mat.rows());
      typename complex_t::m_t::partial_t VsD(eigen_mat.partial(n, n, 0, 0));
      typename complex_t::m_t nV(VsD.inverse());
      for(unsigned int i(0); i < n; i++){
        VsD.partial(n, 1, 0, i) *= (eigen_mat(i, n).sqrt());
      }

      return VsD * nV;
    }

  public:
    /**
     * Calculate square root of a matrix
     *
     * @param threshold_abs Absolute error to be used for convergence determination of eigenvalue calculation
     * @param threshold_abs Relative error to be used for convergence determination of eigenvalue calculation
     * @return square root
     * @see eigen(const T &, const T &)
     */
    typename complex_t::m_t sqrt(
        const T &threshold_abs,
        const T &threshold_rel) const {
      return sqrt(eigen(threshold_abs, threshold_rel));
    }

    /**
     * Calculate square root
     *
     * @return square root
     */
    typename complex_t::m_t sqrt() const {
      return sqrt(eigen());
    }
};

/**
 * Scalar divided by matrix, which is equivalent to inverted matrix multiplied by scalar
 *
 * @param scalar
 * @param matrix
 * @return result matrix
 */
template <class T, class Array2D_Type, class ViewType>
typename Matrix_Frozen<T, Array2D_Type, ViewType>::downcast_default_t::super_t::mul_mat_scalar_t::mat_t operator/(
    const T &scalar, const Matrix_Frozen<T, Array2D_Type, ViewType> &matrix) {
  return scalar / (typename Matrix_Frozen<T, Array2D_Type, ViewType>::downcast_default_t)matrix;
}

#undef throws_when_debug
#if (__cplusplus < 201103L) && defined(noexcept)
#undef noexcept
#endif

#endif /* __MATRIX_H */
