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

#if defined(DEBUG)
#define throw_when_debug(e) throw(e)
#else
#define throw_when_debug(e) throw()
#endif

#include <cstring>
#include <cmath>
#include <ostream>
#include "param/complex.h"

template <class T>
class Array2D_Dense;

/**
 * @brief 2D array abstract class
 *
 * This class provides basic interface of 2D array, such as row and column numbers,
 * accessor for element.
 *
 * @param T precision, for example, double
 */
template<class T>
class Array2D{
  public:
    typedef Array2D<T> self_t;
    typedef Array2D<T> root_t;

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
    Array2D(const unsigned int &rows, const unsigned int &columns)
        : m_rows(rows), m_columns(columns){}

    /**
     * Destructor of Array2D
     */
    virtual ~Array2D(){}

    /**
     * Return rows
     *
     * @return (unsigned int) Rows
     */
    const unsigned int &rows() const{return m_rows;}
    /**
     * Return columns
     *
     * @return (int) Columns
     */
    const unsigned int &columns() const{return m_columns;}

    /**
     * Accessor for element
     *
     * @param row Row index (the first row is zero)
     * @param column Column index (the first column is zero)
     * @return (T) 成分
     */
    virtual const T &operator()(
        const unsigned int &row,
        const unsigned int &column) const = 0;
    T &operator()(
        const unsigned int &row,
        const unsigned int &column) {
      return const_cast<T &>(const_cast<const self_t &>(*this)(row, column));
    }
    
    /**
     * Perform zero clear
     *
     */
    virtual void clear() = 0;

    /**
     * Perform copy
     *
     * @param is_deep If true, return deep copy, otherwise return shallow copy (just link).
     * @return root_t Copy
     */
    virtual root_t *copy(const bool &is_deep = false) const = 0;
};

/**
 * @brief Array2D whose elements are dense, and are stored in sequential 1D array.
 * In other words, (i, j) element is mapped to [i * rows + j].
 *
 * @param T precision, for example, double
 */
template <class T>
class Array2D_Dense : public Array2D<T> {
  public:
    typedef Array2D_Dense<T> self_t;
    typedef Array2D<T> super_t;
    typedef Array2D<T> root_t;
    
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
    Array2D_Dense(const Array2D<T2> &array)
        : values(new T[array.rows() * array.columns()]), ref(new int(1)) {
      T *buf;
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
        if(values = array.values){
          super_t::m_rows = array.m_rows;
          super_t::m_columns = array.m_columns;
          (*(ref = array.ref))++;
        }
      }
      return *this;
    }

    /**
     * Accessor for element
     *
     * @param row Row index
     * @param column Column Index
     * @return (const T &) Element
     * @throw StorageException It will be thrown when the indices are incorrect.
     */
    const T &operator()(
        const unsigned int &row,
        const unsigned int &column) const throw_when_debug(StorageException) {
#if defined(DEBUG)
      if((row >= rows()) || (column >= columns())){
        throw StorageException("Index incorrect");
      }
#endif
      return values[(row * columns()) + column];
    }

    void clear(){
      clear_raw(*this);
    }

    /**
     * Perform copy
     *
     * @aparm is_deep If true, return deep copy, otherwise return shallow copy (just link).
     * @return (root_t) copy
     */
    root_t *copy(const bool &is_deep = false) const {
      return is_deep ? new self_t(rows(), columns(), values) : new self_t(*this);
    }
};

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
 * @param Array2D_Type
 */
template <class T, template <class> class Array2D_Type = Array2D_Dense>
class Matrix{
  public:
    typedef Array2D_Type<T> storage_t;
    typedef Matrix<T, Array2D_Type> self_t;

  protected:
    Array2D<T> *storage; ///< 内部的に利用する2次元配列のメモリ

    /**
     * Matrixクラスのコンストラクタ。
     * ストレージを指定して新しく行列を作ります。
     *
     * @param storage ストレージ
     */
    Matrix(Array2D<T> *new_storage) : storage(new_storage) {}
    
    inline const storage_t *array2d() const{
      return static_cast<const storage_t *>(storage);
    }
    inline storage_t *array2d() {
      return const_cast<storage_t *>(const_cast<const self_t *>(this)->array2d());
    }

  public:
    /**
     * Matrixクラスのコンストラクタ。
     * 内部的な2次元配列用のメモリを確保しない状態で初期化を完了します。
     *
     */
    Matrix() : storage(NULL){}

    /**
     * Matrixクラスのコンストラクタ。
     * 指定の行数、指定の列数で行列を生成します。
     * また成分はすべてT(0)で初期化されます。
     *
     * @param rows 行数
     * @param columns 列数
     */
    Matrix(
        const unsigned int &rows,
        const unsigned int &columns)
        : storage(new storage_t(rows, columns)){
      array2d()->storage_t::clear();
    }

    /**
     * Matrixクラスのコンストラクタ。
     * 指定の行数、指定の列数で行列を生成します。
     * また成分はserializedで復元されます。
     *
     * @param rows 行数
     * @param columns 列数
     * @param serialized 成分
     */
    Matrix(
        const unsigned int &rows,
        const unsigned int &columns,
        const T *serialized)
        : storage(new storage_t(rows, columns, serialized)){
    }

    /**
     * コピーコンストラクタ。
     * シャローコピーを生成します。
     *
     * @param matrix コピー元
     */
    Matrix(const self_t &matrix)
        : storage(matrix.storage
            ? matrix.array2d()->storage_t::copy(false)
            : NULL){}

    template <class T2, template <class> class Array2D_Type2>
    Matrix(const Matrix<T2, Array2D_Type2> &matrix)
        : storage(matrix.storage
            ? new storage_t(matrix.storage)
            : NULL){}
    /**
     * デストラクタ。
     */
    virtual ~Matrix(){delete storage;}


    /**
     * Matrixクラスを作成するヘルパ関数。
     * 指定の行数、指定の列数で行列を生成しますが、
     * 成分については初期化を行わないため不定です。
     *
     * @param new_rows 行数
     * @param new_columns 列数
     */
    static self_t blank(
        const unsigned int &new_rows,
        const unsigned int &new_columns){
      return self_t(new storage_t(new_rows, new_columns));
    }

  protected:
    self_t blank_copy() const {
      return storage ? blank(storage->rows(), storage->columns()) : self_t((storage_t)NULL);
    }

  public:
    /**
     * 代入演算子。
     *
     * @return (self_t) 自分自身
     */
    self_t &operator=(const self_t &matrix){
      if(this != &matrix){
        delete storage;
        if(matrix.storage){
          storage = matrix.array2d()->storage_t::copy(false);
        }
      }
      return *this;
    }
    template <class T2, template <class> class Array2D_Type2>
    self_t &operator=(const Matrix<T2, Array2D_Type2> &matrix){
      delete storage;
      storage = new storage_t(*matrix.storage);
      return *this;
    }

    /**
     * 行列を複製(ディープコピー)します。
     *
     * @return (self_t) コピー
     */
    self_t copy() const {
      return self_t(array2d()->storage_t::copy(true));
    }

    /**
     * 行数を返します。
     *
     * @return (int) 行数
     */
    const unsigned int &rows() const{return storage->rows();}
    /**
     * 列数を返します。
     *
     * @return (int) 列数
     */
    const unsigned int &columns() const{return storage->columns();}

    /**
     * 指定した行列成分を返します。
     *
     * @param row 行インデックス(開始番号は0～)
     * @param column 列インデックス(開始番号は0～)
     * @return (const T &) 成分
     */
    const T &operator()(
        const unsigned int &row,
        const unsigned int &column) const {
      return array2d()->storage_t::operator()(row, column);
    }
    T &operator()(
        const unsigned int &row,
        const unsigned int &column){
      return const_cast<T &>(const_cast<const self_t &>(*this)(row, column));
    }
    
    /**
     * 行列の内容が等しいか、調べます
     * 
     * @param matrix 比較する別の行列
     * @return (bool) 行列が等しい場合true、以外false
     */
    template <class T2, template <class> class Array2D_Type2>
    bool operator==(const Matrix<T2, Array2D_Type2> &matrix) const {
      if(storage != matrix.storage){
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
      }
      return true;
    }
    
    template <class T2, template <class> class Array2D_Type2>
    bool operator!=(const Matrix<T2, Array2D_Type2> &matrix) const {
      return !(operator==(matrix));
    }

    /**
     * 要素をゼロクリアします
     *
     * @return (self_t) ゼロクリアされた自分自身
     */
    self_t &clear(){
      array2d()->storage_t::clear();
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
      for(unsigned int i(0); i < size; i++){result(i, i) = scalar;}
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
     * TODO
     *
     * 行列を転置します。
     * 転置された行列はもとの行列とリンクしています。
     * もとの行列との切り離しを行うにはtranspose().copy()としてください。
     *
     * @return (TransposedMatrix<T>) 転置行列
     */
#if 0
    TransposedMatrix<T> transpose() const{
      return TransposedMatrix<T>(*this);
    }
#else
    self_t transpose() const {
      self_t res(self_t::blank(columns(), rows()));
      for(unsigned int i(0); i < res.rows(); ++i){
        for(unsigned int j(0); j < res.columns(); ++j){
          res(i, j) = operator()(j, i);
        }
      }
      return res;
    }
#endif

    /**
     * TODO
     *
     * 指定した部分行列を返します。
     *
     * @param rowSize 行サイズ
     * @param columnSize 列サイズ
     * @param rowOffset 開始行インデックス
     * @param columnOffset 開始列インデックス
     * @return (PartialMatrix<T>) 部分行列
     *
     */
    self_t partial(
        const unsigned int &new_rows,
        const unsigned int &new_columns,
        const unsigned int &row_offset,
        const unsigned int &column_offset) const {
      self_t res(self_t::blank(new_rows, new_columns));
      for(unsigned int i_dst(0), i_src(row_offset); i_dst < res.rows();  ++i_src, ++i_dst){
        for(unsigned int j_dst(0), j_src(column_offset); j_dst < res.columns(); ++j_src, ++j_dst){
          res(i_dst, j_dst) = operator()(i_src, j_src);
        }
      }
      return res;
    }

    /**
     * TODO
     *
     * 指定した行の行ベクトルを返します。
     *
     * @param row 行インデックス
     * @return (self_t) 行ベクトル
     */
    self_t rowVector(const unsigned int &row) const {
      return partial(1, columns(), row, 0);
    }
    /**
     * TODO
     *
     * 指定した列の列ベクトルを返します。
     *
     * @param column 列インデックス
     * @return (self_t) 列ベクトル
     */
    self_t columnVector(const unsigned int &column) const {
      return partial(rows(), 1, 0, column);
    }

    /**
     * 行を入れ替えます。破壊的メソッドです。
     *
     * @param row1 行インデックス1
     * @param row2 行インデックス2
     * @return (self_t) 自分自身
     * @throw MatrixException インデックスが不正な場合
     */
    self_t &exchangeRows(
        const unsigned int &row1, const unsigned int &row2) throw(MatrixException){
      if(row1 >= rows() || row2 >= rows()){throw MatrixException("Index incorrect");}
      T temp;
      for(unsigned int j(0); j < columns(); j++){
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
    self_t &exchangeColumns(
        const unsigned int &column1, const unsigned int &column2){
      if(column1 >= columns() || column2 >= columns()){throw MatrixException("Index incorrect");}
      T temp;
      for(unsigned int i(0); i < rows(); i++){
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
     * 対称行列かどうか調べます。
     *
     * @return (bool) 対称行列である場合true、それ以外の場合false
     */
    bool isSymmetric() const{
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
     * 行列の大きさが異なるか調べる
     *
     * @param matrix 比較対象
     * @return (bool) 異なっている場合true
     */
    template <class T2, template <class> class Array2D_Type2>
    bool isDifferentSize(const Matrix<T2, Array2D_Type2> &matrix) const{
      return (rows() != matrix.rows()) || (columns() != matrix.columns());
    }

    /**
     * 行列のトレースを返します。
     *
     * @param do_check 正方行列かを調べる、デフォルトtrue
     * @return (T) トレース
     */
    T trace(const bool &do_check = true) const throw(MatrixException) {
      if(do_check && !isSquare()){throw MatrixException("Operation void!!");}
      T tr(0);
      for(unsigned i(0); i < rows(); i++){
        tr += (*this)(i, i);
      }
      return tr;
    }

    /**
     * 行列の成分全てを指定倍します。破壊的メソッドです。
     *
     * @param scalar 倍数
     * @return (self_t) 自分自身
     */
    self_t &operator*=(const T &scalar){
      for(unsigned int i(0); i < rows(); i++){
        for(unsigned int j(0); j < columns(); j++){
          (*this)(i, j) *= scalar;
        }
      }
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
    
    /**
     * 行列を成分ごとに加算します。破壊的メソッドです。
     *
     * @param matrix 加える行列
     * @return (self_t) 自分自身
     */
    template <class T2, template <class> class Array2D_Type2>
    self_t &operator+=(const Matrix<T2, Array2D_Type2> &matrix) throw(MatrixException) {
      if(isDifferentSize(matrix)){throw MatrixException("Operation void!!");}
      for(unsigned int i(0); i < rows(); i++){
        for(unsigned int j(0); j < columns(); j++){
          (*this)(i, j) += matrix(i, j);
        }
      }
      return *this;
    }

    /**
     * 行列を成分ごとに加算します。
     *
     * @param matrix 加える行列
     * @return (self_t) 結果
     */
    template <class T2, template <class> class Array2D_Type2>
    self_t operator+(const Matrix<T2, Array2D_Type2> &matrix) const{return (copy() += matrix);}
    
    /**
     * 行列を成分ごとに減算します。
     *
     * @param matrix 引く行列
     * @return (self_t) 自分自身
     */
    template <class T2, template <class> class Array2D_Type2>
    self_t &operator-=(const Matrix<T2, Array2D_Type2> &matrix) throw(MatrixException) {
      if(isDifferentSize(matrix)){throw MatrixException("Operation void!!");}
      for(unsigned int i(0); i < rows(); i++){
        for(unsigned int j(0); j < columns(); j++){
          (*this)(i, j) -= matrix(i, j);
        }
      }
      return *this;
    }

    /**
     * 行列を成分ごとに減算します。
     *
     * @param matrix 引く行列
     * @return (self_t) 結果
     */
    template <class T2, template <class> class Array2D_Type2>
    self_t operator-(const Matrix<T2, Array2D_Type2> &matrix) const{return (copy() -= matrix);}

    /**
     * 行列を乗算します。
     *
     * @param matrix かける行列
     * @return (self_t) 結果
     * @throw MatrixException 行列の積算が成立しない場合(オペランド行列の列数が引数行列の行数と等しくない)
     */
    template <class T2, template <class> class Array2D_Type2>
    self_t operator*(const Matrix<T2, Array2D_Type2> &matrix) const throw(MatrixException){
      if(columns() != matrix.rows()){
        throw MatrixException("Operation void!!");
      }
      self_t result(self_t::blank(rows(), matrix.columns()));
      for(unsigned int i(0); i < result.rows(); i++){
        for(unsigned int j(0); j < result.columns(); j++){
          result(i, j) = (*this)(i, 0) * matrix(0, j);
          for(unsigned int k(1); k < columns(); k++){
            result(i, j) += ((*this)(i, k) * matrix(k, j));
          }
        }
      }
      return result;
    }
    
    /**
     * 行列を乗算します。破壊的メソッドです。
     *
     * @param matrix かける行列
     * @return (self_t) 自分自身
     * @throw MatrixException 行列の積算が成立しない場合(オペランド行列の列数が引数行列の行数と等しくない)
     */
    template <class T2, template <class> class Array2D_Type2>
    self_t &operator*=(const Matrix<T2, Array2D_Type2> &matrix) throw(MatrixException){
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
     * TODO
     *
     * 補行列(余因子行列)を求めます。
     *
     * @param row 行インデックス
     * @param column 列インデックス
     * @return (CoMatrix) 補行列
     */
#if 0
    CoMatrix<T> coMatrix(
        const unsigned int &row,
        const unsigned int &column) const {
      return CoMatrix<T>(*this, row, column);
    }
#else
    self_t coMatrix(
        const unsigned int &row,
        const unsigned int &column) const {
      self_t res(self_t::blank(rows() - 1, columns() - 1));
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
#endif

    /**
     * 行列式を計算します。
     *
     * @param do_check 正方行列チェックを行うか(デフォルトtrue)
     * @return (T) 結果
     * @throw MatrixException 正方行列ではなく行列式を計算することができない場合
     */
    T determinant(const bool &do_check = true) const throw(MatrixException){
      if(do_check && !isSquare()){throw MatrixException("Operation void!!");}
      if(rows() == 1){
        return (*this)(0, 0);
      }else{
        T sum(0);
        T sign(1);
        for(unsigned int i(0); i < rows(); i++){
          if((*this)(i, 0) != T(0)){
            sum += (*this)(i, 0) * (coMatrix(i, 0).determinant(false)) * sign;
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
    template <class T2, template <class> class Array2D_Type2>
    Matrix<T2, Array2D_Type2> solve_linear_eq_with_LU(
        const Matrix<T2, Array2D_Type2> &y, const bool &do_check = true)
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


      typedef Matrix<T2, Array2D_Type2> y_t;
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
     * LU分解をします。
     * (0, 0)～(n-1, n-1):  L行列
     * (0, n)～(n-1, 2n-1): U行列
     *
     * @param do_check 対称行列チェックを行うか(デフォルトtrue)
     * @return (self_t) LU分解
     * @throw MatrixException 正方行列ではなくLU分解を計算することができない場合
     */
    self_t decomposeLU(const bool &do_check = true) const throw(MatrixException){
      if(do_check && !isSquare()){throw MatrixException("Operation void!!");}
      self_t LU(self_t::blank(rows(), columns() * 2));
#define L(i, j) LU(i, j)
#define U(i, j) LU(i, j + columns())
      for(unsigned int i(0); i < rows(); i++){
        for(unsigned int j(0); j < columns(); j++){
          if(i >= j){
           U(i, j) = T(i == j ? 1 : 0);
            L(i, j) = (*this)(i, j);
            for(unsigned int k(0); k < j; k++){
              L(i, j) -= (L(i, k) * U(k, j));
            }
          }else{
           L(i, j) = T(0);
            U(i, j) = (*this)(i, j);
            for(unsigned int k(0); k < i; k++){
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
    self_t decomposeUD(const bool &do_check = true) const throw(MatrixException){
      if(do_check && !isSymmetric()){throw MatrixException("Operation void");}
      self_t P(copy());
      self_t UD(rows(), columns() * 2);
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
      for(unsigned int i(0); i < rows(); i++){
        for(unsigned int j(0); j < columns(); j++){
          result(i, j) = coMatrix(i, j).determinant() * ((i + j) % 2 == 0 ? 1 : -1);
        }
      }
      return result.transpose() / det;
      */

      //ガウス消去法

      self_t left(copy());
      self_t right(self_t::getI(rows()));
      for(unsigned int i(0); i < rows(); i++){
        if(left(i, i) == T(0)){ //(i, i)が存在するように並べ替え
          for(unsigned int j(i + 1); j <= rows(); j++){
            if(j == rows()){throw MatrixException("Operation void!! ; Invert matrix not exist");}
            if(left(j, i) != T(0)){
              left.exchangeRows(j, i);
              right.exchangeRows(j, i);
              break;
            }
          }
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
     * 逆行列をかけます。破壊的メソッドです。
     *
     * @param matrix 行列
     * @return (self_t) 自分自身
     */
    template <class T2, template <class> class Array2D_Type2>
    self_t &operator/=(const Matrix<T2, Array2D_Type2> &matrix) {
        return (*this) *= matrix.inverse();
    }
    /**
     * 逆行列をかけます。
     *
     * @param matrix 行列
     * @return (self_t) 結果
     */
    template <class T2, template <class> class Array2D_Type2>
    self_t operator/(const Matrix<T2, Array2D_Type2> &matrix) const {
      return (copy() /= matrix);
    }

    /**
     * ピボットを指定して、加算します。
     * 破壊的です。
     *
     * @param row 行インデックス
     * @param column 列インデックス
     * @param matrix 足す行列
     */
    template <class T2, template <class> class Array2D_Type2>
    self_t &pivotMerge(
        const unsigned int &row, const unsigned int &column,
        const Matrix<T2, Array2D_Type2> &matrix){
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
     * ピボットを指定して、加算します。
     *
     * @param row 行インデックス
     * @param column 列インデックス
     * @param matrix 足す行列
     */
    template <class T2, template <class> class Array2D_Type2>
    self_t pivotAdd(
        const unsigned int &row, const unsigned int &column,
        const Matrix<T2, Array2D_Type2> &matrix) const{
      return copy().pivotMerge(row, column, matrix);
    }

    /**
     * ハウスホルダー変換をしてヘッセンベルク行列を得ます。
     *
     * @param transform 変換に用いた行列の積を格納するポインタ
     * @return (self_t) ヘッセンベルク行列
     * @throw MatrixException 正方行列ではなく計算することができない場合
     */
    self_t hessenberg(self_t *transform = NULL) const throw(MatrixException){
      if(!isSquare()){throw MatrixException("Operation void!!");}

      self_t result(copy());
      for(unsigned int j(0); j < columns() - 2; j++){
        T t(0);
        for(unsigned int i(j + 1); i < rows(); i++){
          t += pow(result(i, j), 2);
        }
        T s = ::sqrt(t);
        if(result(j + 1, j) < 0){s *= -1;}

        self_t omega(self_t::blank(rows() - (j+1), 1));
        {
          for(unsigned int i(0); i < omega.rows(); i++){
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
      for(unsigned int j(0); j < columns() - 2; j++){
        for(unsigned int i(j + 2); i < rows(); i++){
          result(i, j) = T(0);
          if(sym){result(j, i) = T(0);}
        }
      }

      return result;
    }

    template <class T2>
    struct complex_t {
      static const bool is_complex = false;
      typedef Complex<T2> v_t;
      typedef Matrix<Complex<T2>, Array2D_Type> m_t;
    };
    template <class T2>
    struct complex_t<Complex<T2> > {
        static const bool is_complex = true;
      typedef Complex<T2> v_t;
      typedef Matrix<Complex<T2>, Array2D_Type> m_t;
    };

    /**
     * 2次小行列の固有値を求めます。
     *
     * @param row 2次小行列の左上項の行インデックス
     * @param column 2次小行列の左上項の列インデックス
     * @param upper 結果(固有値1)
     * @param lower 結果(固有値2)
     */
    void eigen22(
        const unsigned int &row, const unsigned int &column,
        typename complex_t<T>::v_t &upper, typename complex_t<T>::v_t &lower) const {
      T a((*this)(row, column)),
        b((*this)(row, column + 1)),
        c((*this)(row + 1, column)),
        d((*this)(row + 1, column + 1));
      T root2(pow((a - d), 2) + b * c * 4);
      if(complex_t<T>::is_complex || (root2 > 0)){
        T root(::sqrt(root2));
        upper = ((a + d + root) / 2);
        lower = ((a + d - root) / 2);
      }else{
        T root(::sqrt(root2 * -1));
        upper = typename complex_t<T>::v_t((a + d) / 2, root / 2);
        lower = typename complex_t<T>::v_t((a + d) / 2, root / 2 * -1);
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
    typename complex_t<T>::m_t eigen(
        const T &threshold_abs = 1E-10,
        const T &threshold_rel = 1E-7) const throw(MatrixException){

      typedef typename complex_t<T>::m_t res_t;

      if(!isSquare()){throw MatrixException("Operation void!!");}

      //パワー法(べき乗法)
      /*self_t result(rows(), rows() + 1);
      self_t source = copy();
      for(unsigned int i(0); i < columns(); i++){result(0, i) = T(1);}
      for(unsigned int i(0); i < columns(); i++){
        while(true){
          self_t approxVec = source * result.columnVector(i);
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
      typename complex_t<T>::v_t p1, p2;
      int m = _rows;
      bool first = true;

      self_t transform(getI(_rows));
      self_t A(hessenberg(&transform));
      self_t A_(A);

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
          typename complex_t<T>::v_t p1_new, p2_new;
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

          self_t omega(3, 1);
          {
            omega(0, 0) = b1 + r * (b1 >= T(0) ? 1 : -1);
            omega(1, 0) = b2;
            if(b3 != T(0)){omega(2, 0) = b3;}
          }
          self_t P(Matrix::getI(_rows));
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
        typename complex_t<T>::v_t approx_lambda(lambda(j));
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
            throw MatrixException("Cannot calc eigen vectors!!");
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
        typename complex_t<T>::v_t _norm;
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
    static typename complex_t<T>::m_t sqrt(
        const typename complex_t<T>::m_t &eigen_mat){
      unsigned int n(eigen_mat.rows());
      typename complex_t<T>::m_t VsD(eigen_mat.partial(n, n, 0, 0));
      typename complex_t<T>::m_t nV(VsD.inverse());
      for(unsigned int i(0); i < n; i++){
        VsD.partial(n, 1, 0, i) *= (eigen_mat(i, n).sqrt());
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
    typename complex_t<T>::m_t sqrt(
        const T &threshold_abs,
        const T &threshold_rel) const {
      return sqrt(eigen(threshold_abs, threshold_rel));
    }

    /**
     * 行列の平方根を求めます。
     * 返却値はMatrix型です。
     *
     * @return (Matrix<Complex<T> >) 平方根
     */
    typename complex_t<T>::m_t sqrt() const {
      return sqrt(eigen());
    }

    /**
     * 行列を見やすい形で出力します。
     *
     */
    friend std::ostream &operator<<(std::ostream &out, const self_t &matrix){
      if(matrix.storage){
        out << "{";
        for(unsigned int i(0); i < matrix.rows(); i++){
          out << (i == 0 ? "" : ",") << std::endl << "{";
          for(unsigned int j(0); j < matrix.columns(); j++){
            out << (j == 0 ? "" : ",") << matrix(i, j);
          }
          out << "}";
        }
        out << std::endl << "}";
      }
      return out;
    }
};

#undef throw_when_debug

#endif /* __MATRIX_H */
