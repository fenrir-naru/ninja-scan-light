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
#include <limits>
#include "param/complex.h"

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
    
    template <class T2>
    struct family_t {
      typedef Array2D_Dense<T2> res_t;
    };

    using super_t::rows;
    using super_t::columns;

  protected:
    T *values; ///< array for values
    int *ref;  ///< reference counter

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
      setup_t<T>::copy(*this, serialized);
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
        : super_t(array.rows(), array.columns()),
        values(new T[array.rows() * array.columns()]), ref(new int(1)) {
      T *buf(values);
      for(unsigned int i(0); i < array.rows(); ++i){
        for(unsigned int j(0); j < array.columns(); ++j){
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

    template <class T2>
    struct family_t {
      typedef Array2D_ScaledUnit<T2> res_t;
    };

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
  typedef LHS_T first_t;
  typedef LHS_T lhs_t;
  typedef RHS_T rhs_t;
  lhs_t lhs; ///< Left hand side value
  rhs_t rhs; ///< Right hand side value
  Array2D_Operator_Binary(const lhs_t &_lhs, const rhs_t &_rhs) noexcept
      : lhs(_lhs), rhs(_rhs) {}
};

template <
    class LHS_T, class RHS_T,
    class LHS_BufferT = LHS_T, class RHS_BufferT = RHS_T>
struct Array2D_Operator_Multiply;

template <class LHS_T, class RHS_T, bool rhs_positive>
struct Array2D_Operator_Add;


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
  inline T operator()(const Array2D_Type &storage, const unsigned int &i, const unsigned int &j) const {
    return storage.Array2D_Type::operator()(i, j); // direct call instead of via vtable
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
      Array2D_Type storage, const unsigned int &i, const unsigned int &j) const {
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
      Array2D_Type storage, const unsigned int &i, const unsigned int &j) const {
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
      Array2D_Type storage, const unsigned int &i, const unsigned int &j) const {
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
    for(unsigned int i(0); i < src.rows(); ++i){
      for(unsigned int j(0); j < src.columns(); ++j){
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

template <
    class MatrixT,
    int nR_add = 0, int nC_add = 0, int nR_multiply = 1, int nC_multiply = 1>
struct MatrixBuilder;

template <
    template <class, class, class> class MatrixT,
    class T, class Array2D_Type, class ViewType>
struct MatrixBuilder_Dependency<MatrixT<T, Array2D_Type, ViewType> > {
  template <class T2, bool is_writable_array = Array2D_Type::writable>
  struct assignable_matrix_t {
    typedef Matrix<T2> res_t;
  };
  template <class T2>
  struct assignable_matrix_t<T2, true> {
    typedef Matrix<T2, typename Array2D_Type::template family_t<T2>::res_t> res_t;
  };
  typedef typename assignable_matrix_t<T>::res_t assignable_t;

  template <class T2>
  struct family_t {
    typedef typename MatrixBuilder<
        typename assignable_matrix_t<T2>::res_t>::assignable_t assignable_t;
  };
};

template <
    template <class, class, class> class MatrixT,
    class T, class Array2D_Type, class ViewType,
    int nR_add, int nC_add, int nR_multiply, int nC_multiply>
struct MatrixBuilder<
    MatrixT<T, Array2D_Type, ViewType>,
    nR_add, nC_add, nR_multiply, nC_multiply>
    : public MatrixBuilderBase<MatrixT<T, Array2D_Type, ViewType> > {
};

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

    typedef MatrixBuilder<self_t> builder_t;

    template <class T2, class Array2D_Type2, class ViewType2>
    friend class Matrix_Frozen;

    static char (&check_storage(Array2D_Frozen<T> *) )[1];
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
     * @param storage new storage
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
      return view.DELETE_IF_MSC(template) operator()<T>((const storage_t &)storage, row, column);
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
     * Down cast to Matrix by creating deep copy to make its content changeable
     *
     */
    operator typename builder_t::assignable_t() const {
      typedef typename builder_t::assignable_t res_t;
      res_t res(res_t::blank(rows(), columns()));
      builder_t::copy_value(res, *this);
      return res;
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
    typename builder_t::transpose_t transpose() const noexcept {
      return typename builder_t::transpose_t(*this);
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
    typename builder_t::partial_t partial(
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
    typename builder_t::partial_t rowVector(const unsigned int &row) const {
      return partial(1, columns(), row, 0);
    }
    /**
     * Generate column vector by using partial()
     *
     * @param column Column index of original matrix for column vector
     * @return Column vector
     * @see partial()
     */
    typename builder_t::partial_t columnVector(const unsigned int &column) const {
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
    /**
     * Generate partial matrix by just reducing its size;
     * The origins and direction of original and return matrices are the same.
     *
     * @param rowSize Row number
     * @param columnSize Column number
     * @throw std::out_of_range When either row or column size exceeds original one
     * @return partial matrix
     */
    typename builder_t::partial_offsetless_t partial(
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
    /**
     * Generate matrix with circular view
     * "circular" means its index is treated with roll over correction, for example,
     * if specified index is 5 to a matrix of 4 rows, then it will be treated
     * as the same as index 1 (= 5 % 4) is selected.
     *
     * Another example; [4x3].circular(1,2,5,6) is
     *  00 01 02  =>  12 10 11 12 10
     *  10 11 12      22 20 21 22 20
     *  20 21 22      32 30 31 32 30
     *  30 31 32      02 00 01 02 00
     *                12 10 11 12 10
     *
     * @param row_offset Upper row index of original matrix for circular matrix
     * @param column_offset Left column index of original matrix for circular matrix
     * @param new_rows Row number
     * @param new_columns Column number
     * @throw std::out_of_range When either row or column loop exceeds original size
     * @return matrix with circular view
     */
    typename builder_t::circular_t circular(
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
      typename MatrixBuilder<MatrixT>::circular_t res(self);
      res.view.update_loop(self.rows(), self.columns());
      res.view.update_size(self.rows(), self.columns());
      res.view.update_offset(row_offset, column_offset);
      return res;
    }
  public:
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
    typename builder_t::circular_bijective_t circular(
        const unsigned int &row_offset,
        const unsigned int &column_offset) const noexcept {
      return circular_internal(*this, row_offset, column_offset);
    }


    enum {
      OPERATOR_2_Multiply_Matrix_by_Scalar,
      OPERATOR_2_Add_Matrix_to_Matrix,
      OPERATOR_2_Subtract_Matrix_from_Matrix,
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

    template <class RHS_T, class LHS_MatrixT = self_t>
    struct Multiply_Matrix_by_Scalar {

      template <
          bool is_lhs_multi_mat_by_scalar
            = (OperatorProperty<self_t>::tag == OPERATOR_2_Multiply_Matrix_by_Scalar),
          class U = void>
      struct check_lhs_t {
        typedef Array2D_Operator_Multiply<self_t, RHS_T, LHS_MatrixT> op_t;
        typedef Matrix_Frozen<T, Array2D_Operator<T, op_t> > res_t;
        static res_t generate(const LHS_MatrixT &mat, const RHS_T &scalar) noexcept {
          return res_t(
              typename res_t::storage_t(
                mat.rows(), mat.columns(), op_t(mat, scalar)));
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


    template <class RHS_MatrixT, class LHS_MatrixT = self_t>
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
                || (tag == OPERATOR_2_Subtract_Matrix_from_Matrix)
                || (tag == OPERATOR_2_Multiply_Matrix_by_Matrix)
                >::has_multi_mat_by_mat;
        static const bool is_multi_mat_by_scalar
            = (tag == OPERATOR_2_Multiply_Matrix_by_Scalar);
      };

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
        typedef typename MatrixT::builder_t::assignable_t res_t;
      };
#endif
      typedef typename optimizer1_t<self_t>::res_t lhs_opt_t;
      typedef typename optimizer1_t<RHS_MatrixT>::res_t rhs_opt_t;

      typedef Array2D_Operator_Multiply<
          self_t, typename RHS_MatrixT::frozen_t, lhs_opt_t, rhs_opt_t> op_t;

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
        typedef Matrix_Frozen<T, Array2D_Operator<T, op_t> > res_t;
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
     * If this matrix is scalar matrix, then right hand side matrix multiplied by this will be returned.
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
     * Multiply matrix by scalar matrix
     *
     * @param matrix scalar matrix to multiply
     * @return multiplied matrix
     * @throw std::invalid_argument When operation is undefined
     */
    template <class T2>
    typename Multiply_Matrix_by_Scalar<T2>::mat_t
        operator*(const /*typename Matrix<T2>::scalar_matrix_t*/ Matrix_Frozen<T2, Array2D_ScaledUnit<T2> > &matrix) const {
      if(columns() != matrix.rows()){throw std::invalid_argument("Incorrect size");}
      return Multiply_Matrix_by_Scalar<T2>::generate(*this, matrix(0,0));
    }


    /**
     * Generate a matrix in which i-th row and j-th column are removed to calculate minor (determinant)
     *
     * @param row Row to be removed
     * @param column Column to be removed
     * @return Removed matrix
     */
    typename MatrixBuilder<self_t, -1, -1>::assignable_t matrix_for_minor(
        const unsigned int &row,
        const unsigned int &column) const noexcept {
      typename MatrixBuilder<self_t, -1, -1>::assignable_t res(
          MatrixBuilder<self_t, -1, -1>::assignable_t::blank(rows() - 1, columns() - 1));
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
    typename MatrixBuilder<self_t, 0, 0, 1, 2>::assignable_t decomposeLUP(
        unsigned int &pivot_num,
        unsigned int *pivot = NULL,
        const bool &do_check = true) const {
      if(do_check && !isSquare()){throw std::logic_error("rows() != columns()");}

      typedef typename MatrixBuilder<self_t, 0, 0, 1, 2>::assignable_t res_t;
      res_t LU(res_t::blank(rows(), columns() * 2));

      typename res_t::partial_offsetless_t L(LU.partial(rows(), columns()));
      typename res_t::partial_t U(LU.partial(rows(), columns(), 0, columns()));
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
      return LU;
    }

    typename MatrixBuilder<self_t, 0, 0, 1, 2>::assignable_t decomposeLU(
        const bool &do_check = true) const {
      unsigned int pivot_num;
      return decomposeLUP(pivot_num, NULL, do_check);
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
      for(unsigned i(0); i < rows(); i++){
        y_prime(i, 0) = y_copy(i, 0) / L(i, i);
        for(unsigned j(i + 1); j < rows(); j++){
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
      typename MatrixBuilder<self_t, 0, 0, 1, 2>::assignable_t LU(decomposeLUP(pivot_num, NULL, do_check));
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
    typename MatrixBuilder<self_t, 0, 0, 1, 2>::assignable_t decomposeUD(const bool &do_check = true) const {
      if(do_check && !isSymmetric()){throw std::logic_error("not symmetric");}
      typename builder_t::assignable_t P(this->operator typename builder_t::assignable_t());
      typedef typename MatrixBuilder<self_t, 0, 0, 1, 2>::assignable_t res_t;
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
        mat_t result(rows(), columns());
        T det;
        if((det = mat.determinant()) == 0){throw std::runtime_error("Operation void!!");}
        for(unsigned int i(0); i < mat.rows(); i++){
          for(unsigned int j(0); j < mat.columns(); j++){
            result(i, j) = mat.matrix_for_minor(i, j).determinant() * ((i + j) % 2 == 0 ? 1 : -1);
          }
        }
        return result.transpose() / det;
#endif

        // Gaussian elimination; ガウス消去法
        mat_t left(mat.operator mat_t());
        mat_t right(getI(mat.rows()));
        for(unsigned int i(0); i < left.rows(); i++){
          if(left(i, i) == T(0)){
            unsigned int i2(i);
            do{
              if(++i2 == left.rows()){
                throw std::runtime_error("invert matrix not exist");
              }
            }while(left(i2, i) == T(0));
            // exchange i-th and i2-th rows
            for(unsigned int j(i); j < left.columns(); ++j){
              T temp(left(i, j));
              left(i, j) = left(i2, j);
              left(i2, j) = temp;
            }
            right.exchangeRows(i, i2);
          }
          if(left(i, i) != T(1)){
            for(unsigned int j(0); j < left.columns(); j++){right(i, j) /= left(i, i);}
            for(unsigned int j(i+1); j < left.columns(); j++){left(i, j) /= left(i, i);}
            left(i, i) = T(1);
          }
          for(unsigned int k(0); k < left.rows(); k++){
            if(k == i){continue;}
            if(left(k, i) != T(0)){
              for(unsigned int j(0); j < left.columns(); j++){right(k, j) -= right(i, j) * left(k, i);}
              for(unsigned int j(i+1); j < left.columns(); j++){left(k, j) -= left(i, j) * left(k, i);}
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
          typename Inverse_Matrix<Matrix_Frozen<T2, Array2D_Type2, ViewType2> >::mat_t>::mat_t
        operator/(const Matrix_Frozen<T2, Array2D_Type2, ViewType2> &matrix) const {
      return Multiply_Matrix_by_Matrix<
            typename Inverse_Matrix<Matrix_Frozen<T2, Array2D_Type2, ViewType2> >::mat_t>
          ::generate(*this, matrix.inverse()); // equal to (*this) * matrix.inverse()
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

    /**
     * Calculate Hessenberg matrix by performing householder conversion
     *
     * @param transform Pointer to store multiplication of matrices used for the conversion.
     * If NULL is specified, the store will not be performed, The default is NULL.
     * @return Hessenberg matrix
     * @throw std::logic_error When operation is undefined
     */
    template <class T2, class Array2D_Type2, class ViewType2>
    typename builder_t::assignable_t hessenberg(
        Matrix<T2, Array2D_Type2, ViewType2> *transform = NULL) const {
      if(!isSquare()){throw std::logic_error("rows() != columns()");}

      typename builder_t::assignable_t result(this->operator typename builder_t::assignable_t());
      typedef typename MatrixBuilder<self_t, 0, 1, 1, 0>::assignable_t omega_buf_t;
      omega_buf_t omega_buf(omega_buf_t::blank(rows(), 1));
      for(unsigned int j(0); j < columns() - 2; j++){
        T t(0);
        for(unsigned int i(j + 1); i < rows(); i++){
          t += pow(result(i, j), 2);
        }
        T s = ::sqrt(t);
        if(result(j + 1, j) < 0){s *= -1;}

        typename omega_buf_t::partial_offsetless_t omega(omega_buf.partial(rows() - (j+1), 1));
        {
          for(unsigned int i(0); i < omega.rows(); i++){
            omega(i, 0) = result(j+i+1, j);
          }
          omega(0, 0) += s;
        }

        typename builder_t::assignable_t P(getI(rows()));
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
      typedef typename builder_t::template family_t<v_t>::assignable_t m_t;
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
     * The return (n, n+1) matrix consists of
     * (0,j)-(n-1,j): Eigenvector (j) (0 <= j <= n-1)
     * (j,n): Eigenvalue (j)
     *
     * @param threshold_abs Absolute error to be used for convergence determination
     * @param threshold_rel Relative error to be used for convergence determination
     * @return Eigenvalues and eigenvectors
     * @throw std::logic_error When operation is undefined
     * @throw std::runtime_error When operation is unavailable
     */
    typename MatrixBuilder<typename complex_t::m_t, 0, 1>::assignable_t eigen(
        const T &threshold_abs = 1E-10,
        const T &threshold_rel = 1E-7) const {

      typedef typename complex_t::m_t cmat_t;
      typedef typename MatrixBuilder<cmat_t, 0, 1, 1, 0>::assignable_t cvec_t;
      typedef typename MatrixBuilder<cmat_t, 0, 1>::assignable_t res_t;

      if(!isSquare()){throw std::logic_error("rows() != columns()");}

#if 0
      //パワー法(べき乗法)
      typename MatrixBuilder<self_t, 0, 1>::assignable_t result(rows(), rows() + 1);
      typename builder_t::assignable_t source(this->operator typename builder_t::assignable_t());
      for(unsigned int i(0); i < columns(); i++){result(0, i) = T(1);}
      for(unsigned int i(0); i < columns(); i++){
        while(true){
          typename MatrixBuilder<self_t, 0, 1>::assignable_t approxVec(source * result.columnVector(i));
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
      return result;
#endif

      // Double QR method
      /* <Procedure>
       * 1) Transform upper Hessenburg's matrix by using Householder's method
       * ハウスホルダー法を適用して、上ヘッセンベルク行列に置換後
       * 2) Then, Apply double QR method to get eigenvalues
       * ダブルQR法を適用。
       * 3) Finally, compute eigenvectors
       * 結果、固有値が得られるので、固有ベクトルを計算。
       */

      const unsigned int &_rows(rows());

      // 結果の格納用の行列
      res_t result(_rows, _rows + 1);

      // 固有値の計算
#define lambda(i) result(i, _rows)

      T mu_sum(0), mu_multi(0);
      typename complex_t::v_t p1, p2;
      int m = _rows;
      bool first = true;

      typename builder_t::assignable_t transform(getI(_rows));
      typename builder_t::assignable_t A(hessenberg(&transform));
      typename builder_t::assignable_t A_(A);

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

          typename MatrixBuilder<self_t, 3, 1, 0, 0>::assignable_t omega(3, 1);
          {
            omega(0, 0) = b1 + r * (b1 >= T(0) ? 1 : -1);
            omega(1, 0) = b2;
            if(b3 != T(0)){omega(2, 0) = b3;}
          }
          typename builder_t::assignable_t P(getI(_rows));
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

        // Convergence test; 収束判定
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
      // Inverse Iteration to compute eigenvectors; 固有ベクトルの計算(逆反復法)
      cmat_t x(cmat_t::getI(_rows));  //固有ベクトル
      A = A_;
      cmat_t A_C(_rows, _rows);
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
        cmat_t A_C_lambda(A_C.copy());
        typename complex_t::v_t approx_lambda(lambda(j));
        if((A_C_lambda(j, j) - approx_lambda).abs() <= 1E-3){
          approx_lambda += 2E-3;
        }
        for(unsigned int i(0); i < _rows; i++){
          A_C_lambda(i, i) -= approx_lambda;
        }
        typename MatrixBuilder<typename complex_t::m_t, 0, 0, 1, 2>::assignable_t
            A_C_lambda_LU(A_C_lambda.decomposeLU());

        cvec_t target_x(cvec_t::blank(_rows, 1));
        for(unsigned i(0); i < _rows; ++i){
          target_x(i, 0) = x(i, j);
        }
        for(unsigned loop(0); true; loop++){
          cvec_t target_x_new(
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

      // 結果の格納
      for(unsigned int j(0); j < x.columns(); j++){
        for(unsigned int i(0); i < x.rows(); i++){
          for(unsigned int k(0); k < transform.columns(); k++){
            result(i, j) += transform(i, k) * x(k, j);
          }
        }

        // Normalization; 正規化
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
    template <class MatrixT>
    static typename MatrixBuilder<MatrixT, 0, -1>::assignable_t sqrt(
        const MatrixT &eigen_mat){
      unsigned int n(eigen_mat.rows());
      typename MatrixT::partial_offsetless_t VsD(eigen_mat.partial(n, n));
      typename MatrixBuilder<MatrixT, 0, -1>::assignable_t nV(VsD.inverse());
      for(unsigned int i(0); i < n; i++){
        nV.partial(1, n, i, 0) *= (eigen_mat(i, n).sqrt());
      }

      return (typename MatrixBuilder<MatrixT, 0, -1>::assignable_t)(VsD * nV);
    }

  public:
    /**
     * Calculate square root of a matrix
     *
     * @param threshold_abs Absolute error to be used for convergence determination of eigenvalue calculation
     * @param threshold_rel Relative error to be used for convergence determination of eigenvalue calculation
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
        format_t &operator<<(const Matrix_Frozen<T2, Array2D_Type2, View_Type2> *m){
          (*this) << "M"
              << (MatrixViewProperty<View_Type2>::transposed ? "t" : "")
              << (MatrixViewProperty<View_Type2>::variable_size ? "p" : "");
          if(m){
            (*this) << "(" << m->rows() << "," << m->columns() << ")";
          }
          return *this;
        }

        template <class LHS_T, class RHS_T, bool rhs_positive>
        format_t &operator<<(const Array2D_Operator_Add<LHS_T, RHS_T, rhs_positive> *op){
          return (*this) << (const LHS_T *)(op ? &(op->lhs) : 0) << ", " << (const RHS_T *)(op ? &(op->rhs) : 0);
        }

        template <class U>
        static const U &check_scalar(const U *u){return *u;}
        template <class T2, class Array2D_Type2, class View_Type2>
        static const Matrix_Frozen<T2, Array2D_Type2, View_Type2> *check_scalar(
            const Matrix_Frozen<T2, Array2D_Type2, View_Type2> *m){
          return m;
        }

        template <class LHS_T, class RHS_T, class LHS_BufferT, class RHS_BufferT>
        format_t &operator<<(const Array2D_Operator_Multiply<LHS_T, RHS_T, LHS_BufferT, RHS_BufferT> *){
          return (*this) << (const LHS_T *)0 << ", " << (const RHS_T *)0;
        }
        template <class LHS_T, class RHS_T, class LHS_BufferT>
        format_t &operator<<(const Array2D_Operator_Multiply<LHS_T, RHS_T, LHS_BufferT, RHS_T> *op){
          return (*this)
              << (const LHS_T *)0 << ", "
              << check_scalar((const RHS_T *)(op ? &(op->rhs) : 0));
        }
        template <class LHS_T, class RHS_T, class RHS_BufferT>
        format_t &operator<<(const Array2D_Operator_Multiply<LHS_T, RHS_T, LHS_T, RHS_BufferT> *op){
          return (*this) << (const LHS_T *)(op ? &(op->lhs) : 0) << ", " << (const RHS_T *)0;
        }
        template <class LHS_T, class RHS_T>
        format_t &operator<<(const Array2D_Operator_Multiply<LHS_T, RHS_T, LHS_T, RHS_T> *op){
          return (*this)
              << (const LHS_T *)(op ? &(op->lhs) : 0) << ", "
              << check_scalar((const RHS_T *)(op ? &(op->rhs) : 0));
        }

        template <class T2, class OperatorT, class View_Type2>
        format_t &operator<<(
            const Matrix_Frozen<T2, Array2D_Operator<T2, OperatorT>, View_Type2> *m){
          const char *symbol = "";
          switch(OperatorProperty<
              Matrix_Frozen<T2, Array2D_Operator<T2, OperatorT>, View_Type2> >::tag){
            case OPERATOR_2_Multiply_Matrix_by_Scalar:
            case OPERATOR_2_Multiply_Matrix_by_Matrix:
              symbol = "*"; break;
            case OPERATOR_2_Add_Matrix_to_Matrix:
              symbol = "+"; break;
            case OPERATOR_2_Subtract_Matrix_from_Matrix:
              symbol = "-"; break;
            default:
              return (*this) << "(?)";
          }
          return (*this) << "(" << symbol << ", "
              << (const OperatorT *)(m ? &(m->storage.op) : 0) << ")"
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
            << "  *storage: " << &mat << std::endl
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
    class RHS_T,
    class LHS_BufferT, class RHS_BufferT>
struct Array2D_Operator_Multiply<
      Matrix_Frozen<T, Array2D_Type, ViewType>,
      RHS_T,
      LHS_BufferT, RHS_BufferT>
    : public Array2D_Operator_Binary<LHS_BufferT, RHS_BufferT>{
  typedef Array2D_Operator_Binary<LHS_BufferT, RHS_BufferT> super_t;
  static const int tag = super_t::lhs_t::OPERATOR_2_Multiply_Matrix_by_Scalar;
  Array2D_Operator_Multiply(
      const typename super_t::lhs_t &_lhs,
      const typename super_t::rhs_t &_rhs) noexcept
      : super_t(_lhs, _rhs) {}
  T operator()(const unsigned int &row, const unsigned int &column) const noexcept {
    return super_t::lhs(row, column) * super_t::rhs;
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
    class T2, class Array2D_Type2, class ViewType2,
    class LHS_BufferT, class RHS_BufferT>
struct Array2D_Operator_Multiply<
      Matrix_Frozen<T, Array2D_Type, ViewType>,
      Matrix_Frozen<T2, Array2D_Type2, ViewType2>,
      LHS_BufferT, RHS_BufferT>
    : public Array2D_Operator_Binary<LHS_BufferT, RHS_BufferT>{
  typedef Array2D_Operator_Binary<LHS_BufferT, RHS_BufferT> super_t;
  static const int tag = super_t::lhs_t::OPERATOR_2_Multiply_Matrix_by_Matrix;
  Array2D_Operator_Multiply(
      const typename super_t::lhs_t &_lhs,
      const typename super_t::rhs_t &_rhs) noexcept
      : super_t(_lhs, _rhs) {}
  T operator()(const unsigned int &row, const unsigned int &column) const noexcept {
    T res(0);
    for(unsigned int i(0); i < super_t::lhs.columns(); ++i){
      res += super_t::lhs(row, i) * super_t::rhs(i, column);
    }
    return res;
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
    class T, class OperatorT, class ViewType,
    int nR_add, int nC_add, int nR_multiply, int nC_multiply>
struct MatrixBuilder<
    Matrix_Frozen<T, Array2D_Operator<T, OperatorT>, ViewType>,
    nR_add, nC_add, nR_multiply, nC_multiply>
    : public MatrixBuilderBase<
      Matrix_Frozen<T, Array2D_Operator<T, OperatorT>, ViewType> > {

  template <class MatrixT>
  struct unpack_mat_t {
    typedef MatrixT mat_t;
  };
  template <class T2, class OperatorT2, class ViewType2>
  struct unpack_mat_t<Matrix_Frozen<T2, Array2D_Operator<T2, OperatorT2>, ViewType2> > {
    typedef typename MatrixBuilder<Matrix_Frozen<T2, Array2D_Operator<T2, OperatorT2>, ViewType2> >
        ::assignable_t::frozen_t mat_t;
  };

  template <class OperatorT2>
  struct unpack_op_t {
    // consequently, M * S, M + M are captured
    // (op, M1, M2, ...) => M1, then apply ViewType
    typedef typename MatrixBuilder<typename OperatorT2::first_t::frozen_t>
        ::template view_apply_t<ViewType>::applied_t mat_t;
  };
  template <
      class LHS_T,
      class T_R, class Array2D_Type_R, class ViewType_R,
      class LHS_BufferT, class RHS_BufferT>
  struct unpack_op_t<Array2D_Operator_Multiply<
      LHS_T,
      Matrix_Frozen<T_R, Array2D_Type_R, ViewType_R>,
      LHS_BufferT, RHS_BufferT> > { // M * M
    typedef LHS_T mat1_t;
    typedef Matrix_Frozen<T_R, Array2D_Type_R, ViewType_R> mat2_t;
    
    template <
        class OperatorT_L = typename mat1_t::template OperatorProperty<>::operator_t,
        class OperatorT_R = typename mat2_t::template OperatorProperty<>::operator_t,
        class U = void>
    struct check_op_t {
      typedef Matrix_Frozen<T, Array2D_Operator<T, Array2D_Operator_Multiply<
          typename unpack_mat_t<mat1_t>::mat_t,
          typename unpack_mat_t<mat2_t>::mat_t> >, ViewType> res_t;
    };
    template <class U>
    struct check_op_t<void, void, U> {
      // active when both left and right hand side terms are none operator
      // This may be overwritten by (M * M) if its MatrixBuilder specialization exists
      typedef typename MatrixBuilder<mat1_t>::template view_apply_t<ViewType>::applied_t res_t;
    };
    typedef typename check_op_t<>::res_t mat_t;
  };

  typedef typename MatrixBuilder<
      typename unpack_op_t<OperatorT>::mat_t,
      nR_add, nC_add, nR_multiply, nC_multiply>::assignable_t assignable_t;
};
// Remove default assignable_t, and make family_t depend on assignable_t defined in sub class
template <class T, class OperatorT, class ViewType>
struct MatrixBuilder_Dependency<
    Matrix_Frozen<T, Array2D_Operator<T, OperatorT>, ViewType> > {

  template <class T2>
  struct family_t {
    typedef typename MatrixBuilder<
        typename MatrixBuilder<Matrix_Frozen<T, Array2D_Operator<T, OperatorT>, ViewType> >::assignable_t>
        ::template family_t<T2>::assignable_t assignable_t;
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
    typedef typename builder_t::transpose_t transpose_t;
    typedef typename builder_t::partial_offsetless_t partial_offsetless_t;
    typedef typename builder_t::partial_t partial_t;
    typedef typename builder_t::circular_bijective_t circular_bijective_t;
    typedef typename builder_t::circular_t circular_t;

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
     * @param storage new storage
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
      return super_t::view.DELETE_IF_MSC(template) operator()<T &>(
          (storage_t &)storage, row, column);
    }

    using super_t::rows;
    using super_t::columns;

    /**
     * Clear elements.
     *
     */
    void clear(){
      if(view_property_t::variable_size){
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
     * Copy constructor generating shallow copy.
     *
     * @param matrix original
     */
    Matrix(const self_t &matrix)
        : super_t(matrix){}

    template <class T2, class Array2D_Type2>
    Matrix(const Matrix_Frozen<T2, Array2D_Type2, ViewType> &matrix)
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
     * Assign operator performing shallow copy.
     *
     * @return myself
     */
    self_t &operator=(const self_t &matrix){
      super_t::operator=(matrix); // frozen_t::operator=(const frozen_t &) is exactly called
      return *this;
    }
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
      return Matrix_Frozen<T2, Array2D_Type2, ViewType2>::builder_t::copy_value(*this, matrix);
    }

    using super_t::isSquare;
    using super_t::isDiagonal;
    using super_t::isSymmetric;
    using super_t::isDifferentSize;
    using super_t::isLU;

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
      return operator=((clone_t)(*this * matrix));
    }

    /**
     * Divide matrix by matrix, in other words, multiply matrix by inverse matrix. (bang method)
     *
     * @param matrix Matrix to divide
     * @return myself
     */
    template <class T2, class Array2D_Type2, class ViewType2>
    self_t &operator/=(const Matrix_Frozen<T2, Array2D_Type2, ViewType2> &matrix) {
      return operator=((clone_t)(*this / matrix));
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

#undef DELETE_IF_MSC
#undef throws_when_debug
#if (__cplusplus < 201103L) && defined(noexcept)
#undef noexcept
#endif

#endif /* __MATRIX_H */
