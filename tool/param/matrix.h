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
 * @brief 行列ライブラリ
 *
 * お手製の行列を定義したライブラリ。
 * シャローコピーを積極的に利用しているので転置や部分行列ではコピーを生成しない分、
 * 早くなっていると思われます。
 *
 * しかし演算速度を最重視する場合は、ETを活用したublas等を
 * 利用することを検討してください。
 * インターフェイスは大体似ていると思うので、移植は簡単なはずです。
 *
 * @see <a href="http://www.boost.org/libs/numeric/ublas/doc/index.htm">boost::ublas</a>
 */

#define USE_ARRAY2D_ITERATOR

#include <string>
#include <exception>

/**
 * @brief 行列に関わる例外
 *
 * Matrixクラスの例外クラス。
 * 例として、演算が成立しない場合など。
 *
 */
class MatrixException: public std::exception{
  private:
    std::string what_str;
  public:
    /**
     * コンストラクタ。
     *
     * @param what_arg エラー内容
     */
    MatrixException(const std::string &what_arg) : what_str(what_arg){}
    /**
     * デストラクタ。
     *
     */
    ~MatrixException() throw(){}
    /**
     * エラー内容を取得します。
     *
     * @return (chsr *) エラー内容
     */
    const char *what() const throw(){
      return what_str.c_str();
    }
};

/**
 * @brief 行列の保管方法に関する例外
 *
 * 保存クラスの例外クラス。
 * 例として、メモリが足りない場合など。
 *
 */
class StorageException: public MatrixException{
  public:
    /**
     * コンストラクタ。
     *
     * @param what_arg エラー内容
     */
    StorageException(const std::string &what_arg)
        : MatrixException(what_arg) {}
    /**
     * デストラクタ。
     *
     */
    ~StorageException() throw(){}
};

#include <cstring>
#include <cmath>
#include <ostream>
#include <iterator>
#include "param/complex.h"

template <class T>
class Array2D_Dense;

template <class T>
class Matrix;

template <class T>
void array2d_clear(T &t){t = T(0);}

/**
 * 行列内行列用の特殊化
 * 
 */
template <class T>
void array2d_clear(Matrix<T> &t){t = Matrix<T>();}

/**
 * @brief 2次元配列の抽象クラス
 *
 * 2次元配列の抽象クラスです。
 * 2次元配列における要素数、または要素へのアクセスなどのインターフェイスを定義しています。
 *
 * @param T 演算精度、doubleなど。
 */
template<class T>
class Array2D{
  protected:
    unsigned int m_rows;    ///< 行数
    unsigned int m_columns; ///< 列数
    
    typedef Array2D<T> self_t;
    typedef Array2D<T> root_t;
    typedef Array2D_Dense<T> dense_t;
    
  public:
    typedef T content_t;

    /**
     * シャローコピーをします。
     *
     * @return (Array2D *) 自分自身
     */
    virtual self_t *shallow_copy() const = 0;

    /**
     * Array2Dクラスのコンストラクタ。
     * 指定の行サイズ、指定の列サイズで仮想的な2次元配列を生成します。
     *
     * @param rows 行数
     * @param columns 列数
     */
    Array2D(const unsigned int &rows, const unsigned int &columns)
        : m_rows(rows), m_columns(columns){}

    /**
     * Array2Dクラスのデストラクタ。

     */
    virtual ~Array2D(){/*std::cout << "~Array2D() called" << std::std::endl;*/}

    /**
     * 2次元配列を複製(ディープコピー)します。
     *
     * @return storage_t>) コピー
     */
    virtual root_t *copy() const throw(StorageException) = 0;

    /**
     * 密2次元配列に落とし込みます。
     *
     * @return (Array2D_Dense<T>)
     */
    virtual dense_t dense() const = 0;

    /**
     * 2次元配列のサイズを調整します。
     *
     */
    /*virtual void *resize(const unsigned int &rows,
                          const unsigned int &columns) const throw(StorageException){
      m_rows = rows;
      m_columns = columns;
    }*/

    /**
     * 行数を返します。
     *
     * @return (int) 行数
     */
    unsigned int rows() const{return m_rows;}
    /**
     * 列数を返します。
     *
     * @return (int) 列数
     */
    unsigned int columns() const{return m_columns;}

    /**
     * 指定した行列成分を返します。
     *
     * @param row 行インデックス(開始番号は0～)
     * @param column 列インデックス(開始番号は0～)
     * @return (T) 成分
     */
    virtual T &operator()(
        const unsigned int &row,
        const unsigned int &column) throw(StorageException) = 0;
    
    template <class IteratorGenerator>
    void iterate_operation(const IteratorGenerator &gen, void (*op)(T &t)) {
      for(typename IteratorGenerator::iterator it(gen.begin());
          it != gen.end();
          ++it){
        op(*it);
      }
    }
    
    struct IterateOperator {
      virtual void operator()(T &) = 0;
    };
    
    template <class IteratorGenerator>
    void iterate_operation(const IteratorGenerator &gen, IterateOperator &op) {
      for(typename IteratorGenerator::iterator it(gen.begin());
          it != gen.end();
          ++it){
        op(*it);
      }
    }
    
    struct IterateOperator2 {
      virtual void operator()(
          T &, unsigned int, unsigned int) = 0;
    };
    
    template <class IteratorGenerator>
    void iterate_operation(const IteratorGenerator &gen, IterateOperator2 &op) {
      for(typename IteratorGenerator::iterator it(gen.begin());
          it != gen.end();
          ++it){
        op(*it, it.row(), it.column());
      }
    }
    
    struct AllElements {
      typedef AllElements iterator;
      self_t &array2d;
      unsigned int r, c;
      AllElements(
          self_t &_array2d,
          const unsigned int _row = 0, const unsigned int _column = 0) 
          : array2d(_array2d),
          r(_row), c(_column) {}
      AllElements(
          const AllElements &orig) 
          : array2d(orig.array2d),
          r(orig.r), c(orig.c) {}
      ~AllElements(){}
      AllElements begin() const {return AllElements(array2d, 0, 0);}
      AllElements end() const {return AllElements(array2d, array2d.rows(), 0);}
      AllElements &operator++(){
        if(++c >= array2d.columns()){
          c = 0;
          ++r;
        }
        return *this;
      }
      bool operator!=(const AllElements &another){
        return (r != another.r) || (c != another.c);
      }
      T &operator*(){return array2d(r, c);}
      unsigned int row() {return r;}
      unsigned int column() {return c;}
    };
    
    virtual void all_elements(void (*op)(T &t)){
      this->iterate_operation(AllElements(*this), op);
    }
    
    virtual void all_elements(IterateOperator &op){
      this->iterate_operation(AllElements(*this), op);
    }
    
    virtual void all_elements(IterateOperator2 &op){
      this->iterate_operation(AllElements(*this), op);
    }
    
    /**
     * 要素のゼロクリアを行います。
     *
     */
    void clear(){
#ifdef USE_ARRAY2D_ITERATOR
      all_elements(array2d_clear);
#else
      for(unsigned int i = 0; i < rows(); i++){
        for(unsigned int j = 0; j < columns(); j++){
          array2d_clear(this->operator()(i, j));
        }
      }
#endif
    }
};

template <class T>
void array2d_copy(Array2D_Dense<T> *dist, const T *src);

/**
 * @brief 中身が詰まった2次元配列
 *
 * 中身が詰まった2次元配列を表現するクラス。
 * 内部的に1次元配列でこれらのメモリ領域は確保されています。
 * すなわちi行j列成分(i, jが0から数えるとして)は、[i * rows + j]が返却されます。
 *
 * @param T 演算精度、doubleなど。
 */
template <class T>
class Array2D_Dense : public Array2D<T> {
  protected:
    typedef Array2D_Dense<T> self_t;
    typedef Array2D<T> super_t;
    typedef Array2D<T> root_t;
    
    T *m_Values; ///< 確保したメモリ
    int *ref;   ///< 参照カウンタ

  public:
    /**
     * 単純配列化したものを返します。
     *
     * @return (T *) 単純配列
     */
    T *buffer() const {return m_Values;}
    
    using root_t::rows;
    using root_t::columns;
    
    /**
     * 2次元配列を複製(ディープコピー)します。
     *
     * @return (Array2D<T>) コピー
     * @throw StorageException メモリの確保に失敗したとき
     */
    root_t *copy() const throw(StorageException){
      self_t *array(
          new self_t(rows(), columns()));
      array2d_copy(array, m_Values);
      return array;
    }

    /**
     * 密2次元配列に落とし込みます。
     *
     * @return (Array2D_Dense<T>)
     */
    self_t dense() const {return self_t(*this);}

    /**
     * シャローコピーをします。
     * 参照カウンタのインクリメントも同時に行います。
     *
     * @return (Array2D_Dense *)自分自身
     */
    root_t *shallow_copy() const {return new self_t(*this);}

    /**
     * 指定した行列成分を返します。
     *
     * @param row 行インデックス(開始番号は0～)
     * @param column 列インデックス(開始番号は0～)
     * @return (T) 成分
     * @throw StorageException 参照インデックスが不正の場合
     */
    T &operator()(
        const unsigned int &row,
        const unsigned int &column) throw(StorageException){
      if((row >= rows()) || (column >= columns())){
        throw StorageException("Index incorrect");
      }
      return *(m_Values + (row * columns()) + column);
    }

    /**
     * Array2D_Denseクラスのコンストラクタ。
     * 指定の行サイズ、指定の列サイズで2次元配列を生成します。
     *
     * @param rows 行数
     * @param columns 列数
     * @throw StorageException メモリが確保できなかった場合
     */
    Array2D_Dense<T>(
        const unsigned int &rows,
        const unsigned int &columns) throw(StorageException)
        : super_t(rows, columns),
        m_Values(new T[rows * columns]),
        ref(new int(0)) {
      // bad_alloc例外がでるので調べる必要はない
      // if(!m_Values || !ref){throw StorageException("Lack of memory!!");}
      (*ref)++;
    }

    /**
     * Array2D_Denseクラスのコンストラクタ。
     * 指定の行サイズ、指定の列サイズで2次元配列を生成します。
     * また成分はserilaizedによって指定された値で生成されます
     *
     * @param rows 行数
     * @param columns 列数
     * @param serialized 成分
     * @throw StorageException メモリが確保できなかった場合
     */
    Array2D_Dense<T>(
        const unsigned int &rows,
        const unsigned int &columns,
        const T *serialized) throw(StorageException)
        : super_t(rows, columns),
        m_Values(new T[rows * columns]), ref(new int(0)) {
      // bad_alloc例外がでるので調べる必要はない
      //if(!m_Values || !ref){throw StorageException("Lack of memory!!");}
      (*ref)++;
      array2d_copy(this, serialized);
    }

    /**
     * コピーコンストラクタ
     *
     * @param array コピー元
     */
    Array2D_Dense(const self_t &array)
        : super_t(array.m_rows, array.m_columns){
      if(m_Values = array.m_Values){(*(ref = array.ref))++;}
    }
    /**
     * デストラクタ。
     * 参照カウンタを減算すると共に、もしカウンタが0の場合、
     * 確保したメモリをdeleteによって解放します。
     */
    ~Array2D_Dense(){
      if(ref && ((--(*ref)) <= 0)){
        //std::cout << "Trashed" << std::endl;
        delete [] m_Values;
        delete ref;
      }
    }

    /**
     * 代入演算子。
     * 高速なシャローコピーを行います。
     *
     * @return (Array2D_Dense<T>) 自分自身
     */
    self_t &operator=(const self_t &array){
      if(this != &array){
        if(ref && ((--(*ref)) <= 0)){delete ref; delete [] m_Values;}
        if(m_Values = array.m_Values){
          super_t::m_rows = array.m_rows;
          super_t::m_columns = array.m_columns;
          (*(ref = array.ref))++;
        }
      }
      return *this;
    }
    
  public:
    struct AllElements {
      typedef AllElements iterator;
      self_t &array2d;
      unsigned int index;
      AllElements(self_t &_array2d, const unsigned int _index = 0) 
          : array2d(_array2d), index(_index) {}
      AllElements(const AllElements &orig) 
          : array2d(orig.array2d), index(orig.index) {}
      ~AllElements(){}
      AllElements begin() const {return AllElements(array2d, 0);}
      AllElements end() const {return AllElements(array2d, array2d.rows() * array2d.columns());}
      AllElements &operator++(){
        ++index;
        return *this;
      }
      bool operator!=(const AllElements &another){
        return (index != another.index);
      }
      T &operator*(){return *(array2d.buffer() + index);}
      unsigned int row() {return index / array2d.columns();}
      unsigned int column() {return index % array2d.columns();}
    };
    
    void all_elements(void (*op)(T &)){
      this->iterate_operation(AllElements(*this), op);
    }
    
    void all_elements(typename super_t::IterateOperator &op){
      this->iterate_operation(AllElements(*this), op);
    }
    
    void all_elements(typename super_t::IterateOperator2 &op){
      this->iterate_operation(AllElements(*this), op);
    }
};

template <class T>
void array2d_copy(Array2D_Dense<T> *dist, const T *src){
  memcpy(dist->buffer(), src,
      sizeof(T) * dist->rows() * dist->columns());
}

/**
 * 行列内行列用の2次元配列をコピーするための特殊化
 *
 */
template <class T>
void array2d_copy(
    Array2D_Dense<Matrix<T> > *dist,
    const Matrix<T> *src){
  Matrix<T> *dist_buffer(dist->buffer());
  for(unsigned int i(0); i < dist->rows() * dist->columns(); i++){
    (*(dist_buffer++)) = (src++)->copy();
  }
}

/**
 * @brief 別の2次元配列に委譲を行う2次元配列
 *
 * 別の2次元配列に委譲を行う2次元配列を定義したクラス。
 * 例えば転置行列などはインデックスを転置するのみで実態をコピーしなおすことは
 * オーバーヘッドとなります。
 * そこで、このような行列の操作のみを入れ替えるクラスを間に中継させることによって
 * 演算を高速に処理することが可能になります。
 *
 * @param T 演算精度、doubleなど。
 */
template<class T>
class Array2D_Delegate : public Array2D<T>{
  protected:
    typedef Array2D_Delegate<T> self_t;
    typedef Array2D<T> super_t;
    typedef Array2D<T> root_t;
    typedef Array2D_Dense<T> dense_t;
  
  private:
    root_t *m_target;

  protected:
    /**
     * 委譲先を返します。
     *
     * @return (Array2D<T>) 委譲先
     */
    root_t &getTarget() const{return *m_target;}

  public:
    /**
     * 委譲先を返します。
     *
     * @return (Array2D<T>) 委譲先
     */
    const root_t *getParent() const{return m_target;}

    /**
     * Array2D_Partial(部分2次元配列)クラスのコンストラクタ。
     *
     * @param rows 行数
     * @param columns 列数
     * @param array 元の配列
     */
    Array2D_Delegate(
        const unsigned int &rows, const unsigned int &columns,
        const root_t &array) throw(StorageException)
        : super_t(rows, columns), m_target(array.shallow_copy()){}

    /**
     * コピーコンストラクタ
     *
     * @param array コピー元
     * @throw StorageException コピーにおいてなんらかの例外が発生した場合
     */
    Array2D_Delegate(
        const self_t &array) throw(StorageException)
        : super_t(array.rows(), array.columns()),
        m_target(array.m_target->shallow_copy()){}

    /**
     * デストラクタ。
     */
    ~Array2D_Delegate(){delete m_target;}

    /**
     * 指定した行列成分を返します。
     *
     * @param row 行インデックス(開始番号は0～)
     * @param column 列インデックス(開始番号は0～)
     * @return (T) 成分
     * @throw StorageException インデックスが不正の場合
     */
    T &operator()(
        const unsigned int &row,
        const unsigned int &column) throw(StorageException){
      return m_target->operator()(row, column);
    }
    
    using super_t::rows;
    using super_t::columns;
    
    struct DenseCopier : public super_t::IterateOperator2{
      root_t &src;
      DenseCopier(const root_t &_src) 
          : src(const_cast<root_t &>(_src)) {}
      ~DenseCopier() {}
      void operator()(
          T &t, unsigned int row, unsigned int column){
        t = src(row, column);
      }
    };
    
    /**
     * 密2次元配列に落とし込みます。
     *
     * @return (Array2D_Dense<T>)
     */
    dense_t dense() const {
      dense_t array(rows(), columns());
#ifdef USE_ARRAY2D_ITERATOR
      DenseCopier op(*this);
      array.all_elements(op);
#else
      for(unsigned int i(0); i < rows(); i++){
        for(unsigned int j(0); j < columns(); j++){
          array(i, j)
              = (const_cast<Array2D_Delegate *>(this))->operator()(i, j);
        }
      }
#endif
      return array;
    }

    /**
     * 複製(ディープコピー)します。
     * 複製後は単純な密2次元配列になります。
     *
     * @return (root_t) コピー
     * @throw StorageException メモリが確保できなかった場合等
     */
    root_t *copy() const throw(StorageException){
      return dense().shallow_copy();
    }
};

/**
 * @brief 余因子2次元配列
 *
 * 余因子2次元配列をあらわすクラス。
 *
 * @param T 演算精度、doubleなど
 */
template<class T>
class Array2D_CoFactor : public Array2D_Delegate<T>{

  private:
    unsigned int m_RowCoFactor;     ///< 余因子を指定する元の行列での行インデックス
    unsigned int m_ColumnCoFactor;  ///< 余因子を指定する元の行列での列インデックス

  protected:
    /**
     * 余因子行を返します。
     *
     * @return (int) 行インデックス
     */
    unsigned int row_cofactor() const{return m_RowCoFactor;}
    /**
     * 余因子列を返します。
     *
     * @return (int) 列インデックス
     */
    unsigned int column_cofactor() const{return m_ColumnCoFactor;}

  public:
    /**
     * シャローコピーをします。
     *
     * @return (Array2D *)自分自身
     */
    Array2D<T> *shallow_copy() const{return new Array2D_CoFactor(*this);}

    /**
     * Array2D_Partial(部分2次元配列)クラスのコンストラクタ。
     *
     * @param array 元の配列
     * @param rowCoFactor 余因子となる元の2次元配列上での行インデックス
     * @param columnCoFactor 同じく列インデックス
     * @throw StorageException 何らかの例外が発生した場合
     */
    Array2D_CoFactor(const Array2D<T> &array,
                    const unsigned int &rowCoFactor,
                    const unsigned int &columnCoFactor) throw(StorageException)
                       : Array2D_Delegate<T>(array.rows() - 1, array.columns() - 1, array),
                         m_RowCoFactor(rowCoFactor), m_ColumnCoFactor(columnCoFactor){}

    /**
     * コピーコンストラクタ。
     *
     * @param array コピー元
     * @throw StorageException
     */
    Array2D_CoFactor(const Array2D_CoFactor &array) throw(StorageException)
      : Array2D_Delegate<T>(array),
        m_RowCoFactor(array.row_cofactor()), m_ColumnCoFactor(array.column_cofactor()){}

    /**
     * 指定した行列成分を返します。
     *
     * @param row 行インデックス(開始番号は0～)
     * @param column 列インデックス(開始番号は0～)
     * @return (T) 成分
     * @throw StorageException インデックスが不正な場合など
     */
    T &operator()(
        const unsigned int &row,
        const unsigned int &column) throw(StorageException){
      return Array2D_Delegate<T>::operator()(
          (row < row_cofactor() ? row : row + 1), (column < column_cofactor() ? column : column + 1));
    }
};

/**
 * @brief 転置2次元配列
 *
 * 転置2次元配列をあらわすクラス
 * 要素を求める際に元の行列の行と列を入れ替えて要素を抽出することによって
 * 転置表現を高速に実現しています。
 *
 * @param T 演算精度
 */
template<class T>
class Array2D_Transpose : public Array2D_Delegate<T>{

  public:
    /**
     * シャローコピーをします。
     *
     * @return (Array2D *)自分自身
     */
    Array2D<T> *shallow_copy() const{return new Array2D_Transpose(*this);}

    /**
     * Array2D_Transpose(転置2次元配列)クラスのコンストラクタ。
     *
     * @param array 元の配列
     * @throw StorageException
     */
    Array2D_Transpose(const Array2D<T> &array) throw(StorageException)
                       : Array2D_Delegate<T>(array.columns(), array.rows(), array){}

    /**
     * コピーコンストラクタ
     *
     * @param array コピー元
     * @throw StorageException
     */
    Array2D_Transpose(const Array2D_Transpose &array) throw(StorageException)
      : Array2D_Delegate<T>(array){}

    /**
     * 指定した行列成分を返します。
     *
     * @param row 行インデックス(開始番号は0～)
     * @param column 列インデックス(開始番号は0～)
     * @return (T) 成分
     * @throw StorageException インデックスが不正な場合など
     */
    T &operator()(
        const unsigned int &row,
        const unsigned int &column) throw(StorageException){
      return Array2D_Delegate<T>::operator()(column, row);
    }
    
    /**
     * 密2次元配列に落とし込みます。
     * 実際は規定クラスの関数 Array2D_Delegate<T>::dense() を呼び出しているだけで、
     * 特殊化のためのエントリポイントです。
     *
     * @return (Array2D_Dense<T> *)
     */
    Array2D_Dense<T> dense() const {
      return Array2D_Delegate<T>::dense();
    }
};

/**
 * @brief 部分2次元配列
 *
 * 部分2次元配列をあらわすクラス。
 *
 * @param T 演算精度、doubleなど
 */
template<class T>
class Array2D_Partial : public Array2D_Delegate<T>{

  private:
    unsigned int m_RowOffset;     ///< 部分行列が開始する元の行列での行インデックス
    unsigned int m_ColumnOffset;  ///< 部分行列が開始する元の行列での列インデックス

  protected:
    /**
     * オフセット行を返します。
     *
     * @return (int) 行数
     */
    unsigned int row_offset() const{return m_RowOffset;}
    /**
     * オフセット列を返します。
     *
     * @return (int) 列数
     */
    unsigned int column_offset() const{return m_ColumnOffset;}

  public:
    /**
     * シャローコピーをします。
     *
     * @return (Array2D *)自分自身
     */
    Array2D<T> *shallow_copy() const{return new Array2D_Partial(*this);}

    /**
     * Array2D_Partial(部分2次元配列)クラスのコンストラクタ。
     *
     * @param rows 行数
     * @param columns 列数
     * @param array 元の配列
     * @param rowOffset 部分2次元配列で(0,0)となる元の2次元配列のピボットの行インデックス
     * @param columnOffset 同じく列インデックス
     * @throw StorageException 何らかの例外が発生した場合
     */
    Array2D_Partial(const unsigned int &rows,
                    const unsigned int &columns,
                    const Array2D<T> &array,
                    const unsigned int &rowOffset,
                    const unsigned int &columnOffset) throw(StorageException)
                       : Array2D_Delegate<T>(rows, columns, array),
                         m_RowOffset(rowOffset), m_ColumnOffset(columnOffset){}

    /**
     * コピーコンストラクタ。
     *
     * @param array コピー元
     * @throw StorageException
     */
    Array2D_Partial(const Array2D_Partial &array) throw(StorageException)
      : Array2D_Delegate<T>(array),
        m_RowOffset(array.row_offset()), m_ColumnOffset(array.column_offset()){}

    /**
     * 指定した行列成分を返します。
     *
     * @param row 行インデックス(開始番号は0～)
     * @param column 列インデックス(開始番号は0～)
     * @return (T) 成分
     * @throw StorageException インデックスが不正な場合など
     */
    T &operator()(
        const unsigned int &row,
        const unsigned int &column) throw(StorageException){
      return Array2D_Delegate<T>::operator()(
          row + row_offset(), column + column_offset());
    }
};

template <class T>
class CoMatrix;

template <class T>
class TransposedMatrix;

template <class T>
class PartialMatrix;

/**
 * @brief 行列
 *
 * 行列を定義したクラス。
 * 様々な行列の演算を定義しています。
 *
 * なお、内部的に参照カウンタを利用したライトウエイトな実装になっているため、
 * メモリや演算回数が節約されることが機体されます。
 * そのために明示的にcopy()メソッドを使用しないとディープコピーがされないので、
 * 再利用を行う際は注意してください。
 *
 * @see Array2D 内部的に利用する2次元配列を定義したクラス。
 * @param T 演算精度
 */
template <class T>
class Matrix{
  public:
    typedef Matrix<T> self_t;
    typedef Array2D<T> storage_t;

  protected:
    storage_t *m_Storage; ///< 内部的に利用する2次元配列のメモリ

    /**
     * Matrixクラスのコンストラクタ。
     * ストレージを指定して新しく行列を作ります。
     *
     * @param storage ストレージ
     */
    Matrix(const storage_t *storage) : m_Storage(const_cast<storage_t *>(storage)){}
    
    static self_t make_instance(const storage_t *storage){
      return self_t(storage);
    }

    /**
     * Matrixクラスを作成するヘルパ関数。
     * 指定の行数、指定の列数で行列を生成しますが、
     * 成分については初期化を行わないため不定です。
     *
     * @param rows 行数
     * @param columns 列数
     * @throw MatrixException
     */
    static self_t naked(
        const unsigned int &rows,
        const unsigned int &columns) throw(MatrixException){
      return Matrix(new Array2D_Dense<T>(rows, columns));
    }

  public:

    /**
     * 内部的な保存形式を返します。
     *
     * @return (const storage_t *) ストレージ
     */
    const storage_t *storage() const{return m_Storage;}

    /**
     * Matrixクラスのコンストラクタ。
     * 内部的な2次元配列用のメモリを確保しない状態で初期化を完了します。
     *
     */
    Matrix() : m_Storage(NULL){}

    /**
     * Matrixクラスのコンストラクタ。
     * 指定の行数、指定の列数で行列を生成します。
     * また成分はすべてT(0)で初期化されます。
     *
     * @param rows 行数
     * @param columns 列数
     * @throw MatrixException
     */
    Matrix(
        const unsigned int &rows,
        const unsigned int &columns) throw(MatrixException)
        : m_Storage(new Array2D_Dense<T>(rows, columns)){m_Storage->clear();}

    /**
     * Matrixクラスのコンストラクタ。
     * 指定の行数、指定の列数で行列を生成します。
     * また成分はseializedで復元されます。
     *
     * @param rows 行数
     * @param columns 列数
     * @param serialized 成分
     * @throw MatrixException
     */
    Matrix(
        const unsigned int &rows,
        const unsigned int &columns,
        const T *serialized) throw(MatrixException)
        : m_Storage(new Array2D_Dense<T>(rows, columns, serialized)){}

    /**
     * コピーコンストラクタ。
     * シャローコピーを生成します。
     *
     * @param matrix コピー元
     */
    Matrix(const self_t &matrix) : m_Storage(matrix.m_Storage->shallow_copy()){}
    /**
     * デストラクタ。
     */
    virtual ~Matrix(){delete m_Storage;}

  protected:
    /**
     * 代入演算子をサポートするための関数
     * 内部的にはシャローコピーを行っています。
     *
     * @return (self_t) 自分自身
     */
    virtual self_t &substitute(const self_t &matrix){
      if(this != &matrix){
        delete m_Storage;
        if(matrix.m_Storage){
          m_Storage = matrix.m_Storage->shallow_copy();
        }
      }
      return *this;
    }

  public:
    /**
     * 代入演算子。
     *
     * @return (self_t) 自分自身
     */
    self_t &operator=(const self_t &matrix){
      return substitute(matrix);
    }

    /**
     * 行列を複製(ディープコピー)します。
     *
     * @return (self_t) コピー
     * @throw MatrixException
     */
    self_t copy() const throw(MatrixException){
      return self_t(m_Storage->copy());
    }

    /**
     * 行数を返します。
     *
     * @return (int) 行数
     */
    unsigned int rows() const{return m_Storage->rows();}
    /**
     * 列数を返します。
     *
     * @return (int) 列数
     */
    unsigned int columns() const{return m_Storage->columns();}

    /**
     * 指定した行列成分を返します。
     *
     * @param row 行インデックス(開始番号は0～)
     * @param column 列インデックス(開始番号は0～)
     * @return (T) 成分
     * @throw MatrixException インデックスが不正な場合など
     */
    T &operator()(
        const unsigned int &row,
        const unsigned int &column) throw(MatrixException){
      return m_Storage->operator()(row, column);
    }
    /**
     * 指定した行列成分を返します。(Matlab風味)
     *
     * @param row 行番号(開始番号は1～)
     * @param column 列番号(開始番号は1～)
     * @return (T) 成分
     * @throw MatrixException 番号が不正な場合など
     */
    T &index(const int &row, const int &column) throw(MatrixException){
      return (*this)(row - 1, column - 1);
    }
    
    /**
     * 行列の内容が等しいか、調べます
     * 
     * @param matrix 比較する別の行列
     * @return (bool) 行列が等しい場合true、以外false
     */
    bool operator==(const self_t &matrix) const {
      if(m_Storage != matrix.m_Storage){
        if((rows() != matrix.rows())
            || columns() != matrix.columns()){
          return false;
        }
        for(unsigned int i(0); i < rows(); i++){
          for(unsigned int j(0); j < columns(); j++){
            if(const_cast<self_t &>(*this)(i, j)
                != const_cast<self_t &>(matrix)(i, j)){
              return false;
            }
          }
        }
      }
      return true;
    }
    
    bool operator!=(const self_t &matrix) const {
      return !(operator==(matrix));
    }

    /**
     * 要素をゼロクリアします
     *
     * @return (self_t) ゼロクリアされた自分自身
     */
    self_t &clear(){
      m_Storage->clear();
      return *this;
    }

    /**
     * 指定のスカラー行列を生成します。
     *
     * @param size 指定の行数(列数)
     * @param scalar 値
     */
    static self_t getScalar(const unsigned int &size, const T &scalar){
      self_t result(size, size);
      for(unsigned int i = 0; i < size; i++){result(i, i) = scalar;}
      return result;
    }

    /**
     * 指定の単位行列を生成します。
     *
     * @param size 指定の行数(列数)
     */
    static self_t getI(const unsigned int &size){
      return getScalar(size, T(1));
    }

    /**
     * 行列を転置します。
     * 転置された行列はもとの行列とリンクしています。
     * もとの行列との切り離しを行うにはtranspose().copy()としてください。
     *
     * @return (TransposedMatrix<T>) 転置行列
     */
    TransposedMatrix<T> transpose() const{
      return TransposedMatrix<T>(*this);
    }

    /**
     * 指定した部分行列を返します。
     *
     * @param rowSize 行サイズ
     * @param columnSize 列サイズ
     * @param rowOffset 開始行インデックス
     * @param columnOffset 開始列インデックス
     * @return (PartialMatrix<T>) 部分行列
     * @throw MatrixException インデックスが不正な場合
     *
     */
    PartialMatrix<T> partial(
          const unsigned int &rowSize,
          const unsigned int &columnSize,
          const unsigned int &rowOffset,
          const unsigned int &columnOffset) const throw(MatrixException){
      if((rowOffset < 0) || (columnOffset < 0)
          || (rowSize + rowOffset > rows()) || (columnSize + columnOffset > columns())){
        throw MatrixException("Index Incorrect!!");
      }
      return PartialMatrix<T>(*this, rowSize, columnSize, rowOffset, columnOffset);
    }

    /**
     * 指定した行の行ベクトルを返します。
     *
     * @param row 行インデックス
     * @return (self_t) 行ベクトル
     * @throw MatrixException インデックスが不正な場合
     */
    PartialMatrix<T> rowVector(const unsigned int &row) const throw(MatrixException){
      if(row >= rows()){throw MatrixException("Index Incorrect!!");}
      return PartialMatrix<T>(*this, 1, columns(), row, 0);
    }
    /**
     * 指定した列の列ベクトルを返します。
     *
     * @param column 列インデックス
     * @return (self_t) 列ベクトル
     * @throw MatrixException インデックスが不正な場合
     */
    PartialMatrix<T> columnVector(const unsigned int &column) const throw(MatrixException){
      if(column >= columns()){throw MatrixException("Index Incorrect!!");}
      return PartialMatrix<T>(*this, rows(), 1, 0, column);
    }

    /**
     * 行を入れ替えます。破壊的メソッドです。
     *
     * @param row1 行インデックス1
     * @param row2 行インデックス2
     * @return (self_t) 自分自身
     * @throw MatrixException インデックスが不正な場合
     */
    self_t &exchangeRows(const unsigned int &row1, const unsigned int &row2) throw(MatrixException){
      if(row1 >= rows() || row2 >= rows()){throw MatrixException("Index incorrect");}
      T temp;
      for(unsigned int j = 0; j < columns(); j++){
      	temp = (*this)(row1, j);
      	(*this)(row1, j) = (*this)(row2, j);
      	(*this)(row2, j) = temp;
      }
      return *this;
    }

    /**
     * 列を入れ替えます。破壊的メソッドです。
     *
     * @param column1 列インデックス1
     * @param column2 列インデックス2
     * @return (self_t) 自分自身
     * @throw MatrixException インデックスが不正な場合
     */
    self_t &exchangeColumns(const unsigned int &column1, const unsigned int &column2){
      if(column1 >= columns() || column2 >= columns()){throw MatrixException("Index incorrect");}
      T temp;
      for(unsigned int i = 0; i < rows(); i++){
				temp = (*this)(i, column1);
      	(*this)(i, column1) = (*this)(i, column2);
      	(*this)(i, column2) = temp;
      }
      return *this;
    }

    /**
     * 正方行列かどうか調べます。
     *
     * @return (bool) 正方行列である場合true、それ以外の場合false
     */
    bool isSquare() const{return rows() == columns();}

    /**
     * 対角行列かどうか調べます
     *
     * @return (bool) 対角行列である場合true、それ以外の場合false
     */
    bool isDiagonal() const{
      if(isSquare()){
        for(unsigned int i = 0; i < rows(); i++){
          for(unsigned int j = i + 1; j < columns(); j++){
            if((const_cast<self_t &>(*this)(i, j) != T(0))
                || (const_cast<self_t &>(*this)(j, i) != T(0))){
              return false;
            }
          }
        }
        return true;
      }else{return false;}
    }

    /**
     * 対称行列かどうか調べます。
     *
     * @return (bool) 対称行列である場合true、それ以外の場合false
     */
    bool isSymmetric() const{
    	if(isSquare()){
    		for(unsigned int i = 0; i < rows(); i++){
    			for(unsigned int j = i + 1; j < columns(); j++){
    				if((*const_cast<Matrix *>(this))(i, j) != (*const_cast<Matrix *>(this))(j, i)){return false;}
    			}
    		}
    		return true;
    	}else{return false;}
    }

    /**
     * 行列の大きさが異なるか調べる
     *
     * @param matrix 比較対象
     * @return (bool) 異なっている場合true
     */
    bool isDifferentSize(const self_t &matrix) const{
      return (rows() != matrix.rows()) || (columns() != matrix.columns());
    }

    /**
     * 行列のトレースを返します。
     *
     * @param do_check 正方行列かを調べる、デフォルトtrue
     * @return (T) トレース
     */
    T trace(bool do_check = true) const throw(MatrixException) {
      if(do_check && !isSquare()){throw MatrixException("Operation void!!");}
      T tr(0);
      for(unsigned i(0); i < rows(); i++){
        tr += (*const_cast<self_t *>(this))(i, i);
      }
      return tr;
    }
    
    struct ScalarMultiplier : public storage_t::IterateOperator {
      T scalar;
      ScalarMultiplier(const T &_scalar) : scalar(_scalar) {}
      ~ScalarMultiplier(){}
      void operator()(T &t){t *= scalar;}
    };

    /**
     * 行列の成分全てを指定倍します。破壊的メソッドです。
     *
     * @param scalar 倍数
     * @return (self_t) 自分自身
     */
    self_t &operator*=(const T &scalar){
#ifdef USE_ARRAY2D_ITERATOR
      ScalarMultiplier op(scalar);
      m_Storage->all_elements(op);
#else
      for(unsigned int i = 0; i < rows(); i++){
        for(unsigned int j = 0; j < columns(); j++){
          (*this)(i, j) *= scalar;
        }
      }
#endif
      return *this;
    }
    /**
     * 行列の成分全てを指定倍します。
     *
     * @param scalar 倍数
     * @return (self_t) 結果
     */
    self_t operator*(const T &scalar) const{return (copy() *= scalar);}
    /**
     * 行列の成分全てを指定倍します。
     *
     * @param scalar 倍数
     * @param matrix 行列
     * @return (self_t) 結果
     */
    friend self_t operator*(const T &scalar, const self_t &matrix){return matrix * scalar;}
    /**
     * 行列の成分全てを除算します。破壊的メソッドです。
     *
     * @param scalar 倍数
     * @return (self_t) 自分自身
     */
    self_t &operator/=(const T &scalar){return (*this) *= (1 / scalar);}
    /**
     * 行列の成分全てを除算します。
     *
     * @param scalar 倍数
     * @return (self_t) 結果
     */
    self_t operator/(const T &scalar) const{return (copy() /= scalar);}
    /**
     * 行列の成分全てを除算します。
     *
     * @param scalar 倍数
     * @param matrix 行列
     * @return (self_t) 結果
     */
    friend self_t operator/(const T &scalar, const self_t &matrix){return matrix / scalar;}
    
    struct PlusEqual : public storage_t::IterateOperator2 {
      self_t &dist;
      PlusEqual(self_t &_dist) : dist(_dist) {}
      ~PlusEqual() {}
      void operator()(
          T &t, unsigned int row, unsigned int column){
        dist(row, column) += t;
      }
    };
    
    /**
     * 行列を成分ごとに加算します。破壊的メソッドです。
     *
     * @param matrix 加える行列
     * @return (self_t) 自分自身
     */
    self_t &operator+=(const self_t &matrix){
      if(isDifferentSize(matrix)){throw MatrixException("Operation void!!");}
#ifdef USE_ARRAY2D_ITERATOR
      PlusEqual op(*this);
      matrix.m_Storage->all_elements(op);
#else
      for(unsigned int i = 0; i < rows(); i++){
    		for(unsigned int j = 0; j < columns(); j++){
          (*this)(i, j) += (const_cast<self_t &>(matrix))(i, j);
    		}
  		}
#endif
  		return *this;
    }

    /**
     * 行列を成分ごとに加算します。
     *
     * @param matrix 加える行列
     * @return (self_t) 結果
     */
    self_t operator+(const self_t &matrix) const{return (copy() += matrix);}
    
    struct MinusEqual : public storage_t::IterateOperator2 {
      self_t &dist;
      MinusEqual(self_t &_dist) : dist(_dist) {}
      ~MinusEqual() {}
      void operator()(
          T &t, unsigned int row, unsigned int column){
        dist(row, column) -= t;
      }
    };
    
    /**
     * 行列を成分ごとに減算します。
     *
     * @param matrix 引く行列
     * @return (self_t) 自分自身
     */
    self_t &operator-=(const self_t &matrix){
    	if(isDifferentSize(matrix)){throw MatrixException("Operation void!!");}
#ifdef USE_ARRAY2D_ITERATOR
		  MinusEqual op(*this);
      matrix.m_Storage->all_elements(op);
#else
		  for(unsigned int i = 0; i < rows(); i++){
        for(unsigned int j = 0; j < columns(); j++){
          (*this)(i, j) -= (const_cast<self_t &>(matrix))(i, j);
        }
      }
#endif
  		return *this;
    }

    /**
     * 行列を成分ごとに減算します。
     *
     * @param matrix 引く行列
     * @return (self_t) 結果
     */
    self_t operator-(const self_t &matrix) const{return (copy() -= matrix);}

    /**
     * 行列を乗算します。
     *
     * @param matrix かける行列
     * @return (self_t) 結果
     * @throw MatrixException 行列の積算が成立しない場合(オペランド行列の列数が引数行列の行数と等しくない)
     */
    self_t operator*(const self_t &matrix) const throw(MatrixException){
      if(columns() != matrix.rows()){
        throw MatrixException("Operation void!!");
      }
      self_t result(self_t::naked(rows(), matrix.columns()));
      for(unsigned int i = 0; i < result.rows(); i++){
        for(unsigned int j = 0; j < result.columns(); j++){
          result(i, j)
              = ((*const_cast<self_t *>(this))(i, 0)
                * (const_cast<self_t &>(matrix))(0, j));
          for(unsigned int k(1); k < columns(); k++){
            result(i, j)
                += ((*const_cast<self_t *>(this))(i, k)
                  * (const_cast<self_t &>(matrix))(k, j));
          }
        }
      }
      return result;
    }
    
    /**
     * 行列を乗算します。(転置行列バージョン)
     *
     * @param matrix かける行列
     * @return (self_t) 結果
     * @throw MatrixException 行列の積算が成立しない場合(オペランド行列の列数が引数行列の行数と等しくない)
     */
    self_t operator*(const TransposedMatrix<T> &matrix) const throw(MatrixException){
      return operator*((const self_t &)matrix);
    }
    
    /**
     * 行列を乗算します。破壊的メソッドです。
     *
     * @param matrix かける行列
     * @return (self_t) 自分自身
     * @throw MatrixException 行列の積算が成立しない場合(オペランド行列の列数が引数行列の行数と等しくない)
     */
    template <class RhsMatrix>
    self_t &operator*=(const RhsMatrix &matrix) throw(MatrixException){
      return (*this = (*this * matrix));
    }

    /**
     * 単項演算子-。
     * 効果は matrix * -1と同じです。
     *
     * @return (self_t) -matrix
     */
    self_t operator-() const{return (copy() *= -1);}

    /**
     * 補行列(余因子行列)を求めます。
     *
     * @param row 行インデックス
     * @param column 列インデックス
     * @return (CoMatrix) 補行列
     */
    CoMatrix<T> coMatrix(
        const unsigned int &row,
        const unsigned int &column) const throw(MatrixException){
      if(row < 0 && row >= rows() && column < 0 && column >= columns()){
        throw MatrixException("Index incorrect");
      }
      return CoMatrix<T>(*this, row, column);
    }

    /**
     * 行列式を計算します。
     *
     * @param do_check 正方行列チェックを行うか(デフォルトtrue)
     * @return (T) 結果
     * @throw MatrixException 正方行列ではなく行列式を計算することができない場合
     */
    T determinant(bool do_check = true) const throw(MatrixException){
      if(do_check && !isSquare()){throw MatrixException("Operation void!!");}
      if(rows() == 1){
        return (*const_cast<self_t *>(this))(0, 0);
      }else{
        T sum(0);
        T sign(1);
        for(unsigned int i = 0; i < rows(); i++){
          if((*const_cast<self_t *>(this))(i, 0) != T(0)){
            sum += (*const_cast<self_t *>(this))(i, 0)
                * (coMatrix(i, 0).determinant(false)) * sign;
          }
          sign = -sign;
        }
        return sum;
      }
    }

    /**
     * LU行列であること利用して線型方程式(Ax = y)のxを解きます。
     *
     * @param y 右辺
     * @param do_check LU分解済み行列の定義を満たしているか、確認する
     * @return (Matrix<T2>) x(解)
     */
    template <class T2>
    Matrix<T2> solve_linear_eq_with_LU(
        const Matrix<T2> &y, bool do_check = true)
        const throw(MatrixException) {
      bool not_LU(false);
      if(do_check){
        if(rows() * 2 != columns()){not_LU = true;}
      }
      self_t L(partial(rows(), rows(), 0, 0)),
             U(partial(rows(), rows(), 0, rows()));
      if(do_check){
        for(unsigned i(1); i < rows(); i++){
          for(unsigned j(0); j < i; j++){
            if(U(i, j) != T(0)){not_LU = true;}
            if(L(j, i) != T(0)){not_LU = true;}
          }
        }
      }
      if(not_LU){
        throw MatrixException("Not LU decomposed matrix!!");
      }
      if((y.columns() != 1)
          || (y.rows() != rows())){
        throw MatrixException("Operation void!!");
      }


      // L(Ux) = y で y' = (Ux)をまず解く
      Matrix<T2> y_copy(y.copy());
      Matrix<T2> y_prime(Matrix<T2>::naked(y.rows(), 1));
      for(unsigned i(0); i < rows(); i++){
        y_prime(i, 0) = y_copy(i, 0) / L(i, i);
        for(unsigned j(i + 1); j < rows(); j++){
          y_copy(j, 0) -= L(j, i) * y_prime(i, 0);
        }
      }

      // 続いてUx = y'で xを解く
      Matrix<T2> x(Matrix<T2>::naked(y.rows(), 1));
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
     * LU分解をします。
     * (0, 0)～(n-1, n-1):  L行列
     * (0, n)～(n-1, 2n-1): U行列
     *
     * @param do_check 対称行列チェックを行うか(デフォルトtrue)
     * @return (self_t) LU分解
     * @throw MatrixException 正方行列ではなくLU分解を計算することができない場合
     */
   	self_t decomposeLU(bool do_check = true) const throw(MatrixException){
   		if(do_check && !isSquare()){throw MatrixException("Operation void!!");}
   		self_t LU(self_t::naked(rows(), columns() * 2));
#define L(i, j) LU(i, j)
#define U(i, j) LU(i, j + columns())
   		for(unsigned int i = 0; i < rows(); i++){
   			for(unsigned int j = 0; j < columns(); j++){
   				if(i >= j){
            U(i, j) = T(i == j ? 1 : 0);
   					L(i, j) = (*const_cast<Matrix *>(this))(i, j);
   					for(unsigned int k = 0; k < j; k++){
   						L(i, j) -= (L(i, k) * U(k, j));
   					}
   				}else{
            L(i, j) = T(0);
   					U(i, j) = (*const_cast<Matrix *>(this))(i, j);
   					for(unsigned int k = 0; k < i; k++){
   						U(i, j) -= (L(i, k) * U(k, j));
   					}
   					U(i, j) /= L(i, i);
   				}
   			}
   		}
#undef L
#undef U
   		return LU;
   	}

    /**
     * UD分解をします。
     * (0, 0)～(n-1,n-1):  U行列
     * (0, n)～(n-1,2n-1): D行列
     *
     * @param do_check 対称行列チェックを行うか(デフォルトtrue)
     * @return (self_t) UD分解
     * @throw MatrixException 対称行列ではなくUD分解を計算することができない場合
     */
    self_t decomposeUD(bool do_check = true) const throw(MatrixException){
      if(do_check && !isSymmetric()){throw MatrixException("Operation void");}
      self_t P(copy());
      self_t UD(rows(), columns() * 2);
#define U(i, j) UD(i, j)
#define D(i, j) UD(i, j + columns())
      for(int i = rows() - 1; i >= 0; i--){
        D(i, i) = P(i, i);
        U(i, i) = T(1);
        for(int j = 0; j < i; j++){
          U(j, i) = P(j, i) / D(i, i);
          for(int k = 0; k <= j; k++){
            P(k, j) -= U(k, i) * D(i, i) * U(j, i);
          }
        }
      }
#undef U
#undef D
      return UD;
    }

    /**
     * 逆行列を求めます。
     *
     * @return (self_t) 逆行列
     * @throw MatrixException 正方行列ではなく逆行列を計算することができない場合
     */
    self_t inverse() const throw(MatrixException){

      if(!isSquare()){throw MatrixException("Operation void!!");}

      //クラメール(遅い)
      /*
      self_t result(rows(), columns());
      T det;
      if((det = determinant()) == 0){throw MatrixException("Operation void!!");}
      for(int i = 0; i < rows(); i++){
        for(int j = 0; j < columns(); j++){
          result(i, j) = coMatrix(i, j).determinant() * ((i + j) % 2 == 0 ? 1 : -1);
        }
      }
      return result.transpose() / det;
      */

      //ガウス消去法

      self_t left(copy());
      self_t right(self_t::getI(rows()));
      for(unsigned int i = 0; i < rows(); i++){
        if(left(i, i) == T(0)){ //(i, i)が存在するように並べ替え
          for(unsigned int j = i+1; j <= rows(); j++){
            if(j == rows()){throw MatrixException("Operation void!! ; Invert matrix not exist");}
            if(left(j, i) != T(0)){
              left.exchangeRows(j, i);
              right.exchangeRows(j, i);
              break;
            }
          }
        }
        if(left(i, i) != T(1)){
          for(unsigned int j = 0; j < columns(); j++){right(i, j) /= left(i, i);}
          for(unsigned int j = i+1; j < columns(); j++){left(i, j) /= left(i, i);}
          left(i, i) = T(1);
        }
        for(unsigned int k = 0; k < rows(); k++){
          if(k == i){continue;}
          if(left(k, i) != T(0)){
            for(unsigned int j = 0; j < columns(); j++){right(k, j) -= right(i, j) * left(k, i);}
            for(unsigned int j = i+1; j < columns(); j++){left(k, j) -= left(i, j) * left(k, i);}
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
     * 逆行列をかけます。破壊的メソッドです。
     *
     * @param matrix 行列
     * @return (self_t) 自分自身
     * @throw MatrixException 逆行列を計算することができない場合
     */
    self_t &operator/=(const self_t &matrix) throw(MatrixException){return (*this) *= matrix.inverse();}
    /**
     * 逆行列をかけます。
     *
     * @param matrix 行列
     * @return (self_t) 結果
     * @throw MatrixException 逆行列を計算することができない場合
     */
    self_t operator/(const self_t &matrix) const throw(MatrixException){return (copy() /= matrix);}

    /**
     * ピボットを指定して、加算します。
     * 破壊的です。
     *
     * @param row 行インデックス
     * @param column 列インデックス
     * @param matrix 足す行列
     */
    self_t &pivotMerge(const int &row, const int &column, const self_t &matrix){
      for(unsigned int i = 0; i < matrix.rows(); i++){
      	if(row + i < 0){continue;}
        else if(row + i >= rows()){break;}
        for(unsigned int j = 0; j < matrix.columns(); j++){
        	if(column + j < 0){continue;}
          else if(column + j >= columns()){break;}
          (*this)(row + i, column + j) += (const_cast<self_t &>(matrix))(i, j);
        }
      }
      return *this;
    }

    /**
     * ピボットを指定して、加算します。
     *
     * @param row 行インデックス
     * @param column 列インデックス
     * @param matrix 足す行列
     */
    self_t pivotAdd(const int &row, const int &column, const self_t &matrix) const{
      return copy().pivotMerge(row, column, matrix);
    }

    /**
     * ハウスホルダー変換をしてヘッセンベルク行列を得ます。
     *
     * @param transform 変換に用いた行列の積を格納するポインタ
     * @return (self_t) ヘッセンベルク行列
     * @throw MatrixException 正方行列ではなく計算することができない場合
     */
    self_t hessenberg(Matrix *transform) const throw(MatrixException){
    	if(!isSquare()){throw MatrixException("Operation void!!");}

    	self_t result(copy());
    	for(unsigned int j = 0; j < columns() - 2; j++){
    		T t(0);
    		for(unsigned int i = j + 1; i < rows(); i++){
    			t += pow(result(i, j), 2);
    		}
        T s = ::sqrt(t);
        if(result(j + 1, j) < 0){s *= -1;}

    		self_t omega(self_t::naked(rows() - (j+1), 1));
    		{
    			for(unsigned int i = 0; i < omega.rows(); i++){
    				omega(i, 0) = result(j+i+1, j);
    			}
    			omega(0, 0) += s;
    		}

        self_t P(self_t::getI(rows()));
        T denom(t + result(j + 1, j) * s);
        if(denom){
          P.pivotMerge(j+1, j+1, -(omega * omega.transpose() / denom));
        }

        result = P * result * P;
        if(transform){(*transform) *= P;}
    	}

    	//ゼロ処理
    	bool sym = isSymmetric();
    	for(unsigned int j = 0; j < columns() - 2; j++){
    		for(unsigned int i = j + 2; i < rows(); i++){
    			result(i, j) = T(0);
    			if(sym){result(j, i) = T(0);}
    		}
    	}

    	return result;
    }

    /**
     * ハウスホルダー変換をしてヘッセンベルク行列を得ます。
     *
     * @return (self_t) ヘッセンベルク行列
     */
    self_t hessenberg() const throw(MatrixException){return hessenberg(NULL);}

    /**
     * 2次小行列の固有値を求めます。
     *
     * @param row 2次小行列の左上項の行インデックス
     * @param column 2次小行列の左上項の列インデックス
     * @param upper 結果(固有値1)
     * @param lower 結果(固有値2)
     */
    void eigen22(const int &row, const int &column, Complex<T> &upper, Complex<T> &lower) const throw(MatrixException){
      T a((*const_cast<Matrix *>(this))(row, column)),
        b((*const_cast<Matrix *>(this))(row, column + 1)),
        c((*const_cast<Matrix *>(this))(row + 1, column)),
        d((*const_cast<Matrix *>(this))(row + 1, column + 1));
      T root(pow((a - d), 2) + b * c * 4);
      if(root >= T(0)){
        root = ::sqrt(root);
        upper = Complex<T>((a + d + root) / 2);
        lower = Complex<T>((a + d - root) / 2);
      }else{
        root = ::sqrt(root * -1);
        upper = Complex<T>((a + d) / 2, root / 2);
        lower = Complex<T>((a + d) / 2, root / 2 * -1);
      }
    }

    /**
     * 固有値、固有ベクトルを求めます。
     * 返却値はMatrix<Complex<T> >型で、
     * (0,0)～(n-1,n-1)要素が固有ベクトルの行列
     * (0,n)～(n-1,n)要素が対応する固有値の列ベクトル
     * になっています。
     * (固有ベクトル1,固有ベクトル2,…,固有ベクトルn,固有値)のn×(n+1)行列
     *
     * @param threshold_abs 収束判定に用いる絶対誤差
     * @param threshold_rel 収束判定に用いる相対誤差
     * @return (Matrix<Complex<T> >) 固有値、固有ベクトル
     */
    Matrix<Complex<T> > eigen(
        const T &threshold_abs,
        const T &threshold_rel) const throw(MatrixException){

    	if(!isSquare()){throw MatrixException("Operation void!!");}

      //パワー法(べき乗法)
      /*self_t result(rows(), rows() + 1);
      self_t source = copy();
      for(int i = 0; i < columns(); i++){result(0, i) = T(1);}
      for(int i = 0; i < columns(); i++){
        while(true){
          self_t approxVec = source * result.columnVector(i);
          T approxVal(0);
          for(int j = 0; j < approxVec.rows(); j++){approxVal += pow(approxVec(j, 0), 2);}
          approxVal = sqrt(approxVal);
          for(int j = 0; j < approxVec.rows(); j++){result(j, i) = approxVec(j, 0) / approxVal;}
          T before = result(i, rows());
          if(abs(before - (result(i, rows()) = approxVal)) < threshold){break;}
        }
        for(int j = 0; (i < rows() - 1) && (j < rows()); j++){
          for(int k = 0; k < rows(); k++){
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

      //結果の格納用の行列
      Matrix<Complex<T> > result(rows(), rows() + 1);

      //固有値の計算
      Matrix<Complex<T> > lambda(result.partial(rows(), 1, 0, rows()));   //固有値

      T mu_sum(0), mu_multi(0);
      Complex<T> p1, p2;
      int m = rows();
      bool first = true;

      Matrix transform(getI(rows()));
      Matrix A(hessenberg(&transform));
      Matrix A_(A);

      while(true){

        //m = 1 or m = 2
        if(m == 1){
          lambda(0, 0) = A(0, 0);
          break;
        }else if(m == 2){
          A.eigen22(0, 0, lambda(0, 0), lambda(1, 0));
          break;
        }

        //μ、μ*の更新(4.143)
        {
          Complex<T> p1_new, p2_new;
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
        for(int i = 0; i < m - 1; i++){
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

          Matrix omega(3, 1);
          {
    	  	  omega(0, 0) = b1 + r * (b1 >= T(0) ? 1 : -1);
        		omega(1, 0) = b2;
        		if(b3 != T(0)){omega(2, 0) = b3;}
          }
          Matrix P(Matrix::getI(rows()));
          T denom((omega.transpose() * omega)(0, 0));
          if(denom){
            P.pivotMerge(i, i, omega * omega.transpose() * -2 / denom);
          }
          //std::cout << "denom(" << m << ") " << denom << std::endl;

          A = P * A * P;
        }
        //std::cout << "A_scl(" << m << ") " << A(m-1,m-2) << std::endl;

        if(std::isnan(A(m-1,m-2)) || !std::isfinite(A(m-1,m-2))){
          throw MatrixException("Cannot calc eigen values!!");
        }

        //収束判定

        T A_m2_abs(_abs(A(m-2, m-2))), A_m1_abs(_abs(A(m-1, m-1)));
        T epsilon(threshold_abs
          + threshold_rel * ((A_m2_abs < A_m1_abs) ? A_m2_abs : A_m1_abs));

        //std::cout << "epsil(" << m << ") " << epsilon << std::endl;

        if(_abs(A(m-1, m-2)) < epsilon){
          --m;
          lambda(m, 0) = A(m, m);
        }else if(_abs(A(m-2, m-3)) < epsilon){
          A.eigen22(m-2, m-2, lambda(m-1, 0), lambda(m-2, 0));
          m -= 2;
        }
      }

#ifdef MATRIX_EIGENVEC_SIMPLE
      //固有ベクトルの計算
      Matrix<Complex<T> > x(rows(), rows());  //固有ベクトル
      A = A_;

      for(unsigned int j = 0; j < rows(); j++){
        int n = rows();
        for(unsigned int i = 0; i < j; i++){
          if((lambda(j, 0) - lambda(i, 0)).abs() <= threshold_abs){--n;}
        }
        //std::cout << n << ", " << lambda(j, 0) << std::endl;
        x(--n, j) = 1;
        while(n-- > 0){
          x(n, j) = x(n+1, j) * (lambda(j, 0) - A(n+1, n+1));
          for(unsigned int i = n+2; i < rows(); i++){
          	x(n, j) -= x(i, j) * A(n+1, i);
          }
          if(A(n+1, n)){x(n, j) /= A(n+1, n);}
        }
        //std::cout << x.partial(rows(), 1, 0, j).transpose() << std::endl;
      }
#else
      //固有ベクトルの計算(逆反復法)
      Matrix<Complex<T> > x(Matrix<Complex<T> >::getI(rows()));  //固有ベクトル
      A = A_;
      Matrix<Complex<T> > A_C(rows(), rows());
      for(unsigned int i = 0; i < rows(); i++){
        for(unsigned int j = 0; j < columns(); j++){
          A_C(i, j) = A(i, j);
        }
      }

      for(unsigned int j = 0; j < rows(); j++){
        // http://www.prefield.com/algorithm/math/eigensystem.html を参考に
        // かつ、固有値が等しい場合の対処方法として、
        // http://www.nrbook.com/a/bookcpdf/c11-7.pdf
        // を参考に、値を振ってみることにした
        Matrix<Complex<T> > A_C_lambda(A_C.copy());
        Complex<T> approx_lambda(lambda(j, 0));
        if((A_C_lambda(j, j) - approx_lambda).abs() <= 1E-3){
          approx_lambda += 2E-3;
        }
        for(unsigned int i = 0; i < rows(); i++){
          A_C_lambda(i, i) -= approx_lambda;
        }
        Matrix<Complex<T> > A_C_lambda_LU(A_C_lambda.decomposeLU());

        Matrix<Complex<T> > target_x(x.partial(rows(), 1, 0, j));
        for(unsigned loop(0); true; loop++){
          Matrix<Complex<T> > target_x_new(
              A_C_lambda_LU.solve_linear_eq_with_LU(target_x, false));
          T mu((target_x_new.transpose() * target_x)(0, 0).abs2()),
            v2((target_x_new.transpose() * target_x_new)(0, 0).abs2()),
            v2s(::sqrt(v2));
          for(unsigned j(0); j < rows(); ++j){
            target_x(j, 0) = target_x_new(j, 0) / v2s;
          }
          //std::cout << mu << ", " << v2 << std::endl;
          //std::cout << target_x.transpose() << std::endl;
          if((T(1) - (mu * mu / v2)) < T(1.1)){break;}
          if(loop > 100){
            throw MatrixException("Cannot calc eigen vectors!!");
          }
        }
      }
#endif

      /*Matrix<Complex<T> > lambda2(rows(), rows());
      for(int i = 0; i < rows(); i++){
      	lambda2(i, i) = lambda(i, 0);
      }

      std::cout << "A:" << A << std::endl;
      //std::cout << "x * x^-1" << x * x.inverse() << std::endl;
      std::cout << "x * lambda * x^-1:" << x * lambda2 * x.inverse() << std::endl;*/

      //結果の格納
      for(unsigned int j = 0; j < x.columns(); j++){
        for(unsigned int i = 0; i < x.rows(); i++){
          for(unsigned int k = 0; k < transform.columns(); k++){
            result(i, j) += transform(i, k) * x(k, j);
          }
        }

        //正規化
        Complex<T> _norm;
        for(unsigned int i = 0; i < rows(); i++){
        	_norm += result(i, j).abs2();
        }
        T norm = ::sqrt(_norm.real());
        for(unsigned int i = 0; i < rows(); i++){
        	result(i, j) /= norm;
        }
        //std::cout << result.partial(rows(), 1, 0, j).transpose() << std::endl;
      }

      return result;
    }

    /**
     * 固有値、固有ベクトルを求めます。
     * 返却値はMatrix型で、
     * (0,0)～(n-1,n-1)要素が固有ベクトルの行列
     * (0,n)～(n-1,n)要素が対応する固有値の列ベクトル
     * になっています。
     * (固有ベクトル1,固有ベクトル2,…,固有ベクトルn,固有値)のn×(n+1)行列
     *
     * @return (Matrix<Complex<T> >) 固有値、固有ベクトル
     */
    Matrix<Complex<T> > eigen() const throw(MatrixException){return eigen(1E-10, 1E-7);}

  protected:
    /**
     * 行列の平方根を求めます。
     * 返却値はMatrix型です。
     *
     * 行列Aが
     * @f[
     *    A = V D V^{-1}
     * @f]
     * と固有値(D)と固有ベクトル(V)に分解できれば
     * @f[
     *    A^{1/2} = V D^{1/2} V^{-1}
     * @f]
     * である。
     *
     * @param eigen_mat 固有値、固有ベクトルが入った(n,n+1)の行列
     * @return (Matrix<Complex<T> >) 平方根
     */
    Matrix<Complex<T> > sqrt(const Matrix<Complex<T> > &eigen_mat) const throw(MatrixException) {
      int n(eigen_mat.rows());
      Matrix<Complex<T> > VsD(eigen_mat.partial(n, n, 0, 0));
      Matrix<Complex<T> > nV(VsD.inverse());
      for(int i(0); i < n; i++){
        VsD.partial(n, 1, 0, i) *= ::sqrt(const_cast<Matrix<Complex<T> > &>(eigen_mat)(i, n));
      }

      return VsD * nV;
    }

  public:
    /**
     * 行列の平方根を求めます。
     * 返却値はMatrix型です。
     *
     * @param threshold_abs 固有値、固有ベクトル求める際に収束判定に用いる絶対誤差
     * @param threshold_abs 固有値、固有ベクトル求める際に収束判定に用いる相対誤差
     * @return (Matrix<Complex<T> >) 平方根
     */
    Matrix<Complex<T> > sqrt(
        const T &threshold_abs,
        const T &threshold_rel) const throw(MatrixException){
      return sqrt(eigen(threshold_abs, threshold_rel));
    }

    /**
     * 行列の平方根を求めます。
     * 返却値はMatrix型です。
     *
     * @return (Matrix<Complex<T> >) 平方根
     */
    Matrix<Complex<T> > sqrt() const throw(MatrixException){
      return sqrt(eigen());
    }

    /**
     * 行列を見やすい形で出力します。
     *
     */
    friend std::ostream &operator<<(std::ostream &out, const self_t &matrix){
    	if(matrix.m_Storage){
    		out << "{";
	      for(unsigned int i = 0; i < matrix.rows(); i++){
  	      out << (i == 0 ? "" : ",") << std::endl << "{";
    	    for(unsigned int j = 0; j < matrix.columns(); j++){
      	    out << (j == 0 ? "" : ",") << (const_cast<self_t &>(matrix))(i, j);
        	}
	        out << "}";
  	    }
    	  out << std::endl << "}";
    	}
      return out;
    }
};

/**
 * @brief 委譲された行列
 *
 * 委譲された行列(部分行列、転置行列などの派生行列)の基底クラス
 * 元の行列クラスではライトウエイト実装(代入時、シャローコピーによる解決)であるため、
 * そのままでは派生クラスにおいて弊害が発生します。
 * 当該項目を解消するのが、このクラスの役割です。
 */
template <class T>
class DelegatedMatrix : public Matrix<T>{
  protected:
    typedef Matrix<T> super_t;
    typedef DelegatedMatrix<T> self_t;

    super_t &substitute(const super_t &matrix){
      if((this != &matrix) && (super_t::m_Storage)){
        for(unsigned int i(0); i < (std::min)(super_t::rows(), matrix.rows()); i++){
          for(unsigned int j(0); j < (std::min)(super_t::columns(), matrix.columns()); j++){
            (*this)(i, j) = (const_cast<super_t &>(matrix))(i, j);
          }
        }
      }
      return *this;
    }

    DelegatedMatrix(const typename super_t::storage_t *storage)
        : super_t(storage){}
    virtual ~DelegatedMatrix(){}

    /**
     * 代入演算子。
     * 基底クラスのStorageを変更する操作と異なり、成分どおしの代入を行います。
     *
     * @param matrix 代入する行列
     */
    self_t &operator=(const super_t &matrix){
      return static_cast<self_t &>(self_t::substitute(matrix));
    }

    self_t &operator=(const self_t &matrix){
      return static_cast<self_t &>(self_t::substitute(matrix));
    }

  public:
    Matrix<T> original() const{
      return super_t::make_instance(
          static_cast<Array2D_Delegate<T> *>(Matrix<T>::m_Storage)
              ->getParent()->shallow_copy());
    }
};

/**
 * @brief 余因子行列(補行列)
 *
 * 余因子行列をあらわすクラス
 * 余因子2次元配列を表現したArray2D_CoFactorと協力して部分行列を実現しています。
 *
 * @see Array2D_Partial 部分2次元配列
 */
template <class T>
class CoMatrix : public DelegatedMatrix<T>{
  protected:
    typedef Matrix<T> root_t;
    typedef DelegatedMatrix<T> super_t;
    typedef CoMatrix<T> self_t;

  public:
    /**
     * PartialMatrix(部分行列)クラスのコンストラクタ。
     *
     * @param matrix 元の行列
     * @param rowOffset 余因子となる行インデックス
     * @param columnOffset 同じく列インデックス
     * @throw MatrixException
     */
    CoMatrix(
        const root_t &matrix,
        const unsigned int &rowCoFactor,
        const unsigned int &columnCoFactor) throw(MatrixException)
            : super_t(new Array2D_CoFactor<T>(
                *(matrix.storage()),
                rowCoFactor, columnCoFactor)){}

    /**
     * デストラクタ。
     */
    ~CoMatrix(){}

    self_t &operator=(const root_t &matrix){
      return static_cast<self_t &>(super_t::substitute(matrix));
    }
};

/**
 * @brief 転置行列
 *
 * 転置行列をあらわすクラス。
 * クラス内クラスとして定義。
 * 転置2次元配列を表現したArray2D_Transposeと協力して転置行列を実現しています。
 *
 * @see Array2D_Transpose 転置2次元配列
 */
template <class T>
class TransposedMatrix : public DelegatedMatrix<T>{
  public:
    typedef Matrix<T> root_t;
    typedef TransposedMatrix<T> self_t;
  protected:
    typedef DelegatedMatrix<T> super_t;
    

  public:
    /**
     * TransposedMatrix(転置行列)クラスのコンストラクタ。
     *
     * @param matrix 元の行列
     * @throw MatrixException
     */
    TransposedMatrix(const root_t &matrix) throw(MatrixException)
        : super_t(new Array2D_Transpose<T>(*(matrix.storage()))){}

    /**
     * デストラクタ。
     */
    ~TransposedMatrix(){}

    /**
     * 転置行列を転置して元の行列に戻します。
     * 返却される行列はもとの行列とリンクしています。
     * もとの行列との切り離しを行うにはtranspose().copy()としてください。
     *
     * @return (Matrix<T>) 転置行列
     */
    inline root_t untranspose() const{
      return super_t::original();
    }

    self_t &operator=(const root_t &matrix){
      return static_cast<self_t &>(super_t::substitute(matrix));
    }
    
    /**
     * 行列を乗算します。(転置 * 非転置)
     *
     * @param matrix かける行列
     * @return (root_t) 結果
     * @throw MatrixException 行列の積算が成立しない場合(オペランド行列の列数が引数行列の行数と等しくない)
     */
    root_t operator*(const root_t &matrix) const throw(MatrixException){
      return super_t::operator*(matrix);
    }
    
    /**
     * 行列を乗算します。(転置 * 転置)
     *
     * @param matrix かける行列
     * @return (root_t) 結果
     * @throw MatrixException 行列の積算が成立しない場合(オペランド行列の列数が引数行列の行数と等しくない)
     */
    root_t operator*(const self_t &matrix) const throw(MatrixException){
      return operator*((const root_t &)matrix);
    }
};

/**
 * @brief 部分行列
 *
 * 部分行列をあらわすクラス
 * クラス内クラスとして定義。
 * 部分2次元配列を表現したArray2D_Partialと協力して部分行列を実現しています。
 *
 * @see Array2D_Partial 部分2次元配列
 */
template <class T>
class PartialMatrix : public DelegatedMatrix<T>{
  protected:
    typedef Matrix<T> root_t;
    typedef DelegatedMatrix<T> super_t;
    typedef PartialMatrix<T> self_t;

  public:
    /**
     * PartialMatrix(部分行列)クラスのコンストラクタ。
     *
     * @param matrix 元の行列
     * @param rows 行数
     * @param columns 列数
     * @param rowOffset 部分行列で(0,0)となる元の行列のピボットの行インデックス
     * @param columnOffset 同じく列インデックス
     * @throw MatrixException
     */
    PartialMatrix(
        const root_t &matrix,
        const unsigned int &rows,
        const unsigned int &columns,
        const unsigned int &rowOffset,
        const unsigned int &columnOffset) throw(MatrixException)
            : super_t(new Array2D_Partial<T>(
                rows, columns,
                *(matrix.storage()),
                rowOffset, columnOffset)){}

    /**
     * デストラクタ。
     */
    ~PartialMatrix(){}

    self_t &operator=(const root_t &matrix){
      return static_cast<self_t &>(super_t::substitute(matrix));
    }
};

#endif /* __MATRIX_H */
