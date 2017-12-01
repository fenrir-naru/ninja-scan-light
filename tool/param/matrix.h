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
 * @brief �s�񃉃C�u����
 *
 * ���萻�̍s����`�������C�u�����B
 * �V�����[�R�s�[��ϋɓI�ɗ��p���Ă���̂œ]�u�╔���s��ł̓R�s�[�𐶐����Ȃ����A
 * �����Ȃ��Ă���Ǝv���܂��B
 *
 * ���������Z���x���ŏd������ꍇ�́AET�����p����ublas����
 * ���p���邱�Ƃ��������Ă��������B
 * �C���^�[�t�F�C�X�͑�̎��Ă���Ǝv���̂ŁA�ڐA�͊ȒP�Ȃ͂��ł��B
 *
 * @see <a href="http://www.boost.org/libs/numeric/ublas/doc/index.htm">boost::ublas</a>
 */

#include <string>
#include <exception>

/**
 * @brief �s��Ɋւ���O
 *
 * Matrix�N���X�̗�O�N���X�B
 * ��Ƃ��āA���Z���������Ȃ��ꍇ�ȂǁB
 *
 */
class MatrixException: public std::exception{
  private:
    std::string what_str;
  public:
    /**
     * �R���X�g���N�^�B
     *
     * @param what_arg �G���[���e
     */
    MatrixException(const std::string &what_arg) : what_str(what_arg){}
    /**
     * �f�X�g���N�^�B
     *
     */
    ~MatrixException() throw(){}
    /**
     * �G���[���e���擾���܂��B
     *
     * @return (chsr *) �G���[���e
     */
    const char *what() const throw(){
      return what_str.c_str();
    }
};

/**
 * @brief �s��̕ۊǕ��@�Ɋւ����O
 *
 * �ۑ��N���X�̗�O�N���X�B
 * ��Ƃ��āA������������Ȃ��ꍇ�ȂǁB
 *
 */
class StorageException: public MatrixException{
  public:
    /**
     * �R���X�g���N�^�B
     *
     * @param what_arg �G���[���e
     */
    StorageException(const std::string &what_arg)
        : MatrixException(what_arg) {}
    /**
     * �f�X�g���N�^�B
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
     * @return (T) ����
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
 * @brief �s��
 *
 * �s����`�����N���X�B
 * �l�X�ȍs��̉��Z���`���Ă��܂��B
 *
 * �Ȃ��A�����I�ɎQ�ƃJ�E���^�𗘗p�������C�g�E�G�C�g�Ȏ����ɂȂ��Ă��邽�߁A
 * �������≉�Z�񐔂��ߖ񂳂�邱�Ƃ��@�̂���܂��B
 * ���̂��߂ɖ����I��copy()���\�b�h���g�p���Ȃ��ƃf�B�[�v�R�s�[������Ȃ��̂ŁA
 * �ė��p���s���ۂ͒��ӂ��Ă��������B
 *
 * @see Array2D �����I�ɗ��p����2�����z����`�����N���X�B
 * @param T ���Z���x
 * @param Array2D_Type
 */
template <class T, template <class> class Array2D_Type = Array2D_Dense>
class Matrix{
  public:
    typedef Array2D_Type<T> storage_t;
    typedef Matrix<T, Array2D_Type> self_t;

  protected:
    Array2D<T> *storage; ///< �����I�ɗ��p����2�����z��̃�����

    /**
     * Matrix�N���X�̃R���X�g���N�^�B
     * �X�g���[�W���w�肵�ĐV�����s������܂��B
     *
     * @param storage �X�g���[�W
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
     * Matrix�N���X�̃R���X�g���N�^�B
     * �����I��2�����z��p�̃��������m�ۂ��Ȃ���Ԃŏ��������������܂��B
     *
     */
    Matrix() : storage(NULL){}

    /**
     * Matrix�N���X�̃R���X�g���N�^�B
     * �w��̍s���A�w��̗񐔂ōs��𐶐����܂��B
     * �܂������͂��ׂ�T(0)�ŏ���������܂��B
     *
     * @param rows �s��
     * @param columns ��
     */
    Matrix(
        const unsigned int &rows,
        const unsigned int &columns)
        : storage(new storage_t(rows, columns)){
      array2d()->storage_t::clear();
    }

    /**
     * Matrix�N���X�̃R���X�g���N�^�B
     * �w��̍s���A�w��̗񐔂ōs��𐶐����܂��B
     * �܂�������serialized�ŕ�������܂��B
     *
     * @param rows �s��
     * @param columns ��
     * @param serialized ����
     */
    Matrix(
        const unsigned int &rows,
        const unsigned int &columns,
        const T *serialized)
        : storage(new storage_t(rows, columns, serialized)){
    }

    /**
     * �R�s�[�R���X�g���N�^�B
     * �V�����[�R�s�[�𐶐����܂��B
     *
     * @param matrix �R�s�[��
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
     * �f�X�g���N�^�B
     */
    virtual ~Matrix(){delete storage;}


    /**
     * Matrix�N���X���쐬����w���p�֐��B
     * �w��̍s���A�w��̗񐔂ōs��𐶐����܂����A
     * �����ɂ��Ă͏��������s��Ȃ����ߕs��ł��B
     *
     * @param new_rows �s��
     * @param new_columns ��
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
     * ������Z�q�B
     *
     * @return (self_t) �������g
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
     * �s��𕡐�(�f�B�[�v�R�s�[)���܂��B
     *
     * @return (self_t) �R�s�[
     */
    self_t copy() const {
      return self_t(array2d()->storage_t::copy(true));
    }

    /**
     * �s����Ԃ��܂��B
     *
     * @return (int) �s��
     */
    const unsigned int &rows() const{return storage->rows();}
    /**
     * �񐔂�Ԃ��܂��B
     *
     * @return (int) ��
     */
    const unsigned int &columns() const{return storage->columns();}

    /**
     * �w�肵���s�񐬕���Ԃ��܂��B
     *
     * @param row �s�C���f�b�N�X(�J�n�ԍ���0�`)
     * @param column ��C���f�b�N�X(�J�n�ԍ���0�`)
     * @return (const T &) ����
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
     * �s��̓��e�����������A���ׂ܂�
     * 
     * @param matrix ��r����ʂ̍s��
     * @return (bool) �s�񂪓������ꍇtrue�A�ȊOfalse
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
     * �v�f���[���N���A���܂�
     *
     * @return (self_t) �[���N���A���ꂽ�������g
     */
    self_t &clear(){
      array2d()->storage_t::clear();
      return *this;
    }

    /**
     * �w��̃X�J���[�s��𐶐����܂��B
     *
     * @param size �w��̍s��(��)
     * @param scalar �l
     */
    static self_t getScalar(const unsigned int &size, const T &scalar){
      self_t result(size, size);
      for(unsigned int i(0); i < size; i++){result(i, i) = scalar;}
      return result;
    }

    /**
     * �w��̒P�ʍs��𐶐����܂��B
     *
     * @param size �w��̍s��(��)
     */
    static self_t getI(const unsigned int &size){
      return getScalar(size, T(1));
    }

    /**
     * TODO
     *
     * �s���]�u���܂��B
     * �]�u���ꂽ�s��͂��Ƃ̍s��ƃ����N���Ă��܂��B
     * ���Ƃ̍s��Ƃ̐؂藣�����s���ɂ�transpose().copy()�Ƃ��Ă��������B
     *
     * @return (TransposedMatrix<T>) �]�u�s��
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
     * �w�肵�������s���Ԃ��܂��B
     *
     * @param rowSize �s�T�C�Y
     * @param columnSize ��T�C�Y
     * @param rowOffset �J�n�s�C���f�b�N�X
     * @param columnOffset �J�n��C���f�b�N�X
     * @return (PartialMatrix<T>) �����s��
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
     * �w�肵���s�̍s�x�N�g����Ԃ��܂��B
     *
     * @param row �s�C���f�b�N�X
     * @return (self_t) �s�x�N�g��
     */
    self_t rowVector(const unsigned int &row) const {
      return partial(1, columns(), row, 0);
    }
    /**
     * TODO
     *
     * �w�肵����̗�x�N�g����Ԃ��܂��B
     *
     * @param column ��C���f�b�N�X
     * @return (self_t) ��x�N�g��
     */
    self_t columnVector(const unsigned int &column) const {
      return partial(rows(), 1, 0, column);
    }

    /**
     * �s�����ւ��܂��B�j��I���\�b�h�ł��B
     *
     * @param row1 �s�C���f�b�N�X1
     * @param row2 �s�C���f�b�N�X2
     * @return (self_t) �������g
     * @throw MatrixException �C���f�b�N�X���s���ȏꍇ
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
     * ������ւ��܂��B�j��I���\�b�h�ł��B
     *
     * @param column1 ��C���f�b�N�X1
     * @param column2 ��C���f�b�N�X2
     * @return (self_t) �������g
     * @throw MatrixException �C���f�b�N�X���s���ȏꍇ
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
     * �����s�񂩂ǂ������ׂ܂��B
     *
     * @return (bool) �����s��ł���ꍇtrue�A����ȊO�̏ꍇfalse
     */
    bool isSquare() const{return rows() == columns();}

    /**
     * �Ίp�s�񂩂ǂ������ׂ܂�
     *
     * @return (bool) �Ίp�s��ł���ꍇtrue�A����ȊO�̏ꍇfalse
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
     * �Ώ̍s�񂩂ǂ������ׂ܂��B
     *
     * @return (bool) �Ώ̍s��ł���ꍇtrue�A����ȊO�̏ꍇfalse
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
     * �s��̑傫�����قȂ邩���ׂ�
     *
     * @param matrix ��r�Ώ�
     * @return (bool) �قȂ��Ă���ꍇtrue
     */
    template <class T2, template <class> class Array2D_Type2>
    bool isDifferentSize(const Matrix<T2, Array2D_Type2> &matrix) const{
      return (rows() != matrix.rows()) || (columns() != matrix.columns());
    }

    /**
     * �s��̃g���[�X��Ԃ��܂��B
     *
     * @param do_check �����s�񂩂𒲂ׂ�A�f�t�H���gtrue
     * @return (T) �g���[�X
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
     * �s��̐����S�Ă��w��{���܂��B�j��I���\�b�h�ł��B
     *
     * @param scalar �{��
     * @return (self_t) �������g
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
     * �s��̐����S�Ă��w��{���܂��B
     *
     * @param scalar �{��
     * @return (self_t) ����
     */
    self_t operator*(const T &scalar) const{return (copy() *= scalar);}
    /**
     * �s��̐����S�Ă��w��{���܂��B
     *
     * @param scalar �{��
     * @param matrix �s��
     * @return (self_t) ����
     */
    friend self_t operator*(const T &scalar, const self_t &matrix){return matrix * scalar;}
    /**
     * �s��̐����S�Ă����Z���܂��B�j��I���\�b�h�ł��B
     *
     * @param scalar �{��
     * @return (self_t) �������g
     */
    self_t &operator/=(const T &scalar){return (*this) *= (1 / scalar);}
    /**
     * �s��̐����S�Ă����Z���܂��B
     *
     * @param scalar �{��
     * @return (self_t) ����
     */
    self_t operator/(const T &scalar) const{return (copy() /= scalar);}
    /**
     * �s��̐����S�Ă����Z���܂��B
     *
     * @param scalar �{��
     * @param matrix �s��
     * @return (self_t) ����
     */
    friend self_t operator/(const T &scalar, const self_t &matrix){return matrix / scalar;}
    
    /**
     * �s��𐬕����Ƃɉ��Z���܂��B�j��I���\�b�h�ł��B
     *
     * @param matrix ������s��
     * @return (self_t) �������g
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
     * �s��𐬕����Ƃɉ��Z���܂��B
     *
     * @param matrix ������s��
     * @return (self_t) ����
     */
    template <class T2, template <class> class Array2D_Type2>
    self_t operator+(const Matrix<T2, Array2D_Type2> &matrix) const{return (copy() += matrix);}
    
    /**
     * �s��𐬕����ƂɌ��Z���܂��B
     *
     * @param matrix �����s��
     * @return (self_t) �������g
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
     * �s��𐬕����ƂɌ��Z���܂��B
     *
     * @param matrix �����s��
     * @return (self_t) ����
     */
    template <class T2, template <class> class Array2D_Type2>
    self_t operator-(const Matrix<T2, Array2D_Type2> &matrix) const{return (copy() -= matrix);}

    /**
     * �s�����Z���܂��B
     *
     * @param matrix ������s��
     * @return (self_t) ����
     * @throw MatrixException �s��̐ώZ���������Ȃ��ꍇ(�I�y�����h�s��̗񐔂������s��̍s���Ɠ������Ȃ�)
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
     * �s�����Z���܂��B�j��I���\�b�h�ł��B
     *
     * @param matrix ������s��
     * @return (self_t) �������g
     * @throw MatrixException �s��̐ώZ���������Ȃ��ꍇ(�I�y�����h�s��̗񐔂������s��̍s���Ɠ������Ȃ�)
     */
    template <class T2, template <class> class Array2D_Type2>
    self_t &operator*=(const Matrix<T2, Array2D_Type2> &matrix) throw(MatrixException){
      return (*this = (*this * matrix));
    }

    /**
     * �P�����Z�q-�B
     * ���ʂ� matrix * -1�Ɠ����ł��B
     *
     * @return (self_t) -matrix
     */
    self_t operator-() const{return (copy() *= -1);}

    /**
     * TODO
     *
     * ��s��(�]���q�s��)�����߂܂��B
     *
     * @param row �s�C���f�b�N�X
     * @param column ��C���f�b�N�X
     * @return (CoMatrix) ��s��
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
     * �s�񎮂��v�Z���܂��B
     *
     * @param do_check �����s��`�F�b�N���s����(�f�t�H���gtrue)
     * @return (T) ����
     * @throw MatrixException �����s��ł͂Ȃ��s�񎮂��v�Z���邱�Ƃ��ł��Ȃ��ꍇ
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
     * LU�s��ł��邱�Ɨ��p���Đ��^������(Ax = y)��x�������܂��B
     *
     * @param y �E��
     * @param do_check LU�����ςݍs��̒�`�𖞂����Ă��邩�A�m�F����
     * @return (Matrix<T2>) x(��)
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
      // L(Ux) = y �� y' = (Ux)���܂�����
      y_t y_copy(y.copy());
      y_t y_prime(y_t::blank(y.rows(), 1));
      for(unsigned i(0); i < rows(); i++){
        y_prime(i, 0) = y_copy(i, 0) / L(i, i);
        for(unsigned j(i + 1); j < rows(); j++){
          y_copy(j, 0) -= L(j, i) * y_prime(i, 0);
        }
      }

      // ������Ux = y'�� x������
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
     * LU���������܂��B
     * (0, 0)�`(n-1, n-1):  L�s��
     * (0, n)�`(n-1, 2n-1): U�s��
     *
     * @param do_check �Ώ̍s��`�F�b�N���s����(�f�t�H���gtrue)
     * @return (self_t) LU����
     * @throw MatrixException �����s��ł͂Ȃ�LU�������v�Z���邱�Ƃ��ł��Ȃ��ꍇ
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
     * UD���������܂��B
     * (0, 0)�`(n-1,n-1):  U�s��
     * (0, n)�`(n-1,2n-1): D�s��
     *
     * @param do_check �Ώ̍s��`�F�b�N���s����(�f�t�H���gtrue)
     * @return (self_t) UD����
     * @throw MatrixException �Ώ̍s��ł͂Ȃ�UD�������v�Z���邱�Ƃ��ł��Ȃ��ꍇ
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
     * �t�s������߂܂��B
     *
     * @return (self_t) �t�s��
     * @throw MatrixException �����s��ł͂Ȃ��t�s����v�Z���邱�Ƃ��ł��Ȃ��ꍇ
     */
    self_t inverse() const throw(MatrixException){

      if(!isSquare()){throw MatrixException("Operation void!!");}

      //�N�����[��(�x��)
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

      //�K�E�X�����@

      self_t left(copy());
      self_t right(self_t::getI(rows()));
      for(unsigned int i(0); i < rows(); i++){
        if(left(i, i) == T(0)){ //(i, i)�����݂���悤�ɕ��בւ�
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

      //LU����
      /*
      */
    }
    /**
     * �t�s��������܂��B�j��I���\�b�h�ł��B
     *
     * @param matrix �s��
     * @return (self_t) �������g
     */
    template <class T2, template <class> class Array2D_Type2>
    self_t &operator/=(const Matrix<T2, Array2D_Type2> &matrix) {
        return (*this) *= matrix.inverse();
    }
    /**
     * �t�s��������܂��B
     *
     * @param matrix �s��
     * @return (self_t) ����
     */
    template <class T2, template <class> class Array2D_Type2>
    self_t operator/(const Matrix<T2, Array2D_Type2> &matrix) const {
      return (copy() /= matrix);
    }

    /**
     * �s�{�b�g���w�肵�āA���Z���܂��B
     * �j��I�ł��B
     *
     * @param row �s�C���f�b�N�X
     * @param column ��C���f�b�N�X
     * @param matrix �����s��
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
     * �s�{�b�g���w�肵�āA���Z���܂��B
     *
     * @param row �s�C���f�b�N�X
     * @param column ��C���f�b�N�X
     * @param matrix �����s��
     */
    template <class T2, template <class> class Array2D_Type2>
    self_t pivotAdd(
        const unsigned int &row, const unsigned int &column,
        const Matrix<T2, Array2D_Type2> &matrix) const{
      return copy().pivotMerge(row, column, matrix);
    }

    /**
     * �n�E�X�z���_�[�ϊ������ăw�b�Z���x���N�s��𓾂܂��B
     *
     * @param transform �ϊ��ɗp�����s��̐ς��i�[����|�C���^
     * @return (self_t) �w�b�Z���x���N�s��
     * @throw MatrixException �����s��ł͂Ȃ��v�Z���邱�Ƃ��ł��Ȃ��ꍇ
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

      //�[������
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
     * 2�����s��̌ŗL�l�����߂܂��B
     *
     * @param row 2�����s��̍��㍀�̍s�C���f�b�N�X
     * @param column 2�����s��̍��㍀�̗�C���f�b�N�X
     * @param upper ����(�ŗL�l1)
     * @param lower ����(�ŗL�l2)
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
     * �ŗL�l�A�ŗL�x�N�g�������߂܂��B
     * �ԋp�l��Matrix<Complex<T> >�^�ŁA
     * (0,0)�`(n-1,n-1)�v�f���ŗL�x�N�g���̍s��
     * (0,n)�`(n-1,n)�v�f���Ή�����ŗL�l�̗�x�N�g��
     * �ɂȂ��Ă��܂��B
     * (�ŗL�x�N�g��1,�ŗL�x�N�g��2,�c,�ŗL�x�N�g��n,�ŗL�l)��n�~(n+1)�s��
     *
     * @param threshold_abs ��������ɗp�����Ό덷
     * @param threshold_rel ��������ɗp���鑊�Ό덷
     * @return (Matrix<Complex<T> >) �ŗL�l�A�ŗL�x�N�g��
     */
    typename complex_t<T>::m_t eigen(
        const T &threshold_abs = 1E-10,
        const T &threshold_rel = 1E-7) const throw(MatrixException){

      typedef typename complex_t<T>::m_t res_t;

      if(!isSquare()){throw MatrixException("Operation void!!");}

      //�p���[�@(�ׂ���@)
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

      //�_�u��QR�@
      /* <�菇>
       * �n�E�X�z���_�[�@��K�p���āA��w�b�Z���x���N�s��ɒu����A
       * �_�u��QR�@��K�p�B
       * ���ʁA�ŗL�l��������̂ŁA�ŗL�x�N�g�����v�Z�B
       */

      const unsigned int &_rows(rows());

      //���ʂ̊i�[�p�̍s��
      res_t result(_rows, _rows + 1);

      //�ŗL�l�̌v�Z
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

        //�ʁA��*�̍X�V(4.143)
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

        //�n�E�X�z���_�[�ϊ����J��Ԃ�
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

        //��������
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
      //�ŗL�x�N�g���̌v�Z
      res_t x(_rows, _rows);  //�ŗL�x�N�g��
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
      //�ŗL�x�N�g���̌v�Z(�t�����@)
      res_t x(res_t::getI(_rows));  //�ŗL�x�N�g��
      A = A_;
      res_t A_C(_rows, _rows);
      for(unsigned int i(0); i < _rows; i++){
        for(unsigned int j(0); j < columns(); j++){
          A_C(i, j) = A(i, j);
        }
      }

      for(unsigned int j(0); j < _rows; j++){
        // http://www.prefield.com/algorithm/math/eigensystem.html ���Q�l��
        // ���A�ŗL�l���������ꍇ�̑Ώ����@�Ƃ��āA
        // http://www.nrbook.com/a/bookcpdf/c11-7.pdf
        // ���Q�l�ɁA�l��U���Ă݂邱�Ƃɂ���
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

      //���ʂ̊i�[
      for(unsigned int j(0); j < x.columns(); j++){
        for(unsigned int i(0); i < x.rows(); i++){
          for(unsigned int k(0); k < transform.columns(); k++){
            result(i, j) += transform(i, k) * x(k, j);
          }
        }

        //���K��
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
     * �s��̕����������߂܂��B
     * �ԋp�l��Matrix�^�ł��B
     *
     * �s��A��
     * @f[
     *    A = V D V^{-1}
     * @f]
     * �ƌŗL�l(D)�ƌŗL�x�N�g��(V)�ɕ����ł����
     * @f[
     *    A^{1/2} = V D^{1/2} V^{-1}
     * @f]
     * �ł���B
     *
     * @param eigen_mat �ŗL�l�A�ŗL�x�N�g����������(n,n+1)�̍s��
     * @return (Matrix<Complex<T> >) ������
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
     * �s��̕����������߂܂��B
     * �ԋp�l��Matrix�^�ł��B
     *
     * @param threshold_abs �ŗL�l�A�ŗL�x�N�g�����߂�ۂɎ�������ɗp�����Ό덷
     * @param threshold_abs �ŗL�l�A�ŗL�x�N�g�����߂�ۂɎ�������ɗp���鑊�Ό덷
     * @return (Matrix<Complex<T> >) ������
     */
    typename complex_t<T>::m_t sqrt(
        const T &threshold_abs,
        const T &threshold_rel) const {
      return sqrt(eigen(threshold_abs, threshold_rel));
    }

    /**
     * �s��̕����������߂܂��B
     * �ԋp�l��Matrix�^�ł��B
     *
     * @return (Matrix<Complex<T> >) ������
     */
    typename complex_t<T>::m_t sqrt() const {
      return sqrt(eigen());
    }

    /**
     * �s������₷���`�ŏo�͂��܂��B
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
