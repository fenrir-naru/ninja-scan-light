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

#define USE_ARRAY2D_ITERATOR

#include <string>
#include <cstring>
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

#include <cstring>
#include <cmath>
#include <ostream>
#include <iterator>
#include <algorithm>
#include "param/complex.h"

template <class T>
class Array2D_Dense;

template <class T>
class Matrix;

template <class T>
void array2d_clear(T &t){t = T(0);}

/**
 * �s����s��p�̓��ꉻ
 * 
 */
template <class T>
void array2d_clear(Matrix<T> &t){t = Matrix<T>();}

/**
 * @brief 2�����z��̒��ۃN���X
 *
 * 2�����z��̒��ۃN���X�ł��B
 * 2�����z��ɂ�����v�f���A�܂��͗v�f�ւ̃A�N�Z�X�Ȃǂ̃C���^�[�t�F�C�X���`���Ă��܂��B
 *
 * @param T ���Z���x�Adouble�ȂǁB
 */
template<class T>
class Array2D{
  protected:
    unsigned int m_rows;    ///< �s��
    unsigned int m_columns; ///< ��
    
    typedef Array2D<T> self_t;
    typedef Array2D<T> root_t;
    typedef Array2D_Dense<T> dense_t;
    
  public:
    typedef T content_t;

    /**
     * �V�����[�R�s�[�����܂��B
     *
     * @return (Array2D *) �������g
     */
    virtual self_t *shallow_copy() const = 0;

    /**
     * Array2D�N���X�̃R���X�g���N�^�B
     * �w��̍s�T�C�Y�A�w��̗�T�C�Y�ŉ��z�I��2�����z��𐶐����܂��B
     *
     * @param rows �s��
     * @param columns ��
     */
    Array2D(const unsigned int &rows, const unsigned int &columns)
        : m_rows(rows), m_columns(columns){}

    /**
     * Array2D�N���X�̃f�X�g���N�^�B

     */
    virtual ~Array2D(){/*std::cout << "~Array2D() called" << std::std::endl;*/}

    /**
     * 2�����z��𕡐�(�f�B�[�v�R�s�[)���܂��B
     *
     * @return storage_t>) �R�s�[
     */
    virtual root_t *copy() const throw(StorageException) = 0;

    /**
     * ��2�����z��ɗ��Ƃ����݂܂��B
     *
     * @return (Array2D_Dense<T>)
     */
    virtual dense_t dense() const = 0;

    /**
     * 2�����z��̃T�C�Y�𒲐����܂��B
     *
     */
    /*virtual void *resize(const unsigned int &rows,
                          const unsigned int &columns) const throw(StorageException){
      m_rows = rows;
      m_columns = columns;
    }*/

    /**
     * �s����Ԃ��܂��B
     *
     * @return (int) �s��
     */
    unsigned int rows() const{return m_rows;}
    /**
     * �񐔂�Ԃ��܂��B
     *
     * @return (int) ��
     */
    unsigned int columns() const{return m_columns;}

    /**
     * �w�肵���s�񐬕���Ԃ��܂��B
     *
     * @param row �s�C���f�b�N�X(�J�n�ԍ���0�`)
     * @param column ��C���f�b�N�X(�J�n�ԍ���0�`)
     * @return (T) ����
     */
    virtual const T &operator()(
        const unsigned int &row,
        const unsigned int &column) const throw(StorageException) = 0;
    virtual T &operator()(
        const unsigned int &row,
        const unsigned int &column) throw(StorageException){
      return const_cast<T &>(static_cast<const self_t &>(*this)(row, column));
    }
    
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
     * �v�f�̃[���N���A���s���܂��B
     *
     */
    void clear(){
#ifdef USE_ARRAY2D_ITERATOR
      all_elements(array2d_clear);
#else
      for(unsigned int i(0); i < rows(); i++){
        for(unsigned int j(0); j < columns(); j++){
          array2d_clear(this->operator()(i, j));
        }
      }
#endif
    }
};

template <class T>
void array2d_copy(Array2D_Dense<T> *dist, const T *src);

/**
 * @brief ���g���l�܂���2�����z��
 *
 * ���g���l�܂���2�����z���\������N���X�B
 * �����I��1�����z��ł����̃������̈�͊m�ۂ���Ă��܂��B
 * ���Ȃ킿i�sj�񐬕�(i, j��0���琔����Ƃ���)�́A[i * rows + j]���ԋp����܂��B
 *
 * @param T ���Z���x�Adouble�ȂǁB
 */
template <class T>
class Array2D_Dense : public Array2D<T> {
  protected:
    typedef Array2D_Dense<T> self_t;
    typedef Array2D<T> super_t;
    typedef Array2D<T> root_t;
    
    T *m_Values; ///< �m�ۂ���������
    int *ref;   ///< �Q�ƃJ�E���^

  public:
    /**
     * �P���z�񉻂������̂�Ԃ��܂��B
     *
     * @return (T *) �P���z��
     */
    T *buffer() const {return m_Values;}
    
    using root_t::rows;
    using root_t::columns;
    
    /**
     * 2�����z��𕡐�(�f�B�[�v�R�s�[)���܂��B
     *
     * @return (Array2D<T>) �R�s�[
     * @throw StorageException �������̊m�ۂɎ��s�����Ƃ�
     */
    root_t *copy() const throw(StorageException){
      self_t *array(
          new self_t(rows(), columns()));
      array2d_copy(array, m_Values);
      return array;
    }

    /**
     * ��2�����z��ɗ��Ƃ����݂܂��B
     *
     * @return (Array2D_Dense<T>)
     */
    self_t dense() const {return self_t(*this);}

    /**
     * �V�����[�R�s�[�����܂��B
     * �Q�ƃJ�E���^�̃C���N�������g�������ɍs���܂��B
     *
     * @return (Array2D_Dense *)�������g
     */
    root_t *shallow_copy() const {return new self_t(*this);}

    /**
     * �w�肵���s�񐬕���Ԃ��܂��B
     *
     * @param row �s�C���f�b�N�X(�J�n�ԍ���0�`)
     * @param column ��C���f�b�N�X(�J�n�ԍ���0�`)
     * @return (const T &) ����
     * @throw StorageException �Q�ƃC���f�b�N�X���s���̏ꍇ
     */
    const T &operator()(
        const unsigned int &row,
        const unsigned int &column) const throw(StorageException){
      if((row >= rows()) || (column >= columns())){
        throw StorageException("Index incorrect");
      }
      return *(m_Values + (row * columns()) + column);
    }

    /**
     * Array2D_Dense�N���X�̃R���X�g���N�^�B
     * �w��̍s�T�C�Y�A�w��̗�T�C�Y��2�����z��𐶐����܂��B
     *
     * @param rows �s��
     * @param columns ��
     * @throw StorageException ���������m�ۂł��Ȃ������ꍇ
     */
    Array2D_Dense<T>(
        const unsigned int &rows,
        const unsigned int &columns) throw(StorageException)
        : super_t(rows, columns),
        m_Values(new T[rows * columns]),
        ref(new int(0)) {
      // bad_alloc��O���ł�̂Œ��ׂ�K�v�͂Ȃ�
      // if(!m_Values || !ref){throw StorageException("Lack of memory!!");}
      (*ref)++;
    }

    /**
     * Array2D_Dense�N���X�̃R���X�g���N�^�B
     * �w��̍s�T�C�Y�A�w��̗�T�C�Y��2�����z��𐶐����܂��B
     * �܂�������serilaized�ɂ���Ďw�肳�ꂽ�l�Ő�������܂�
     *
     * @param rows �s��
     * @param columns ��
     * @param serialized ����
     * @throw StorageException ���������m�ۂł��Ȃ������ꍇ
     */
    Array2D_Dense<T>(
        const unsigned int &rows,
        const unsigned int &columns,
        const T *serialized) throw(StorageException)
        : super_t(rows, columns),
        m_Values(new T[rows * columns]), ref(new int(0)) {
      // bad_alloc��O���ł�̂Œ��ׂ�K�v�͂Ȃ�
      //if(!m_Values || !ref){throw StorageException("Lack of memory!!");}
      (*ref)++;
      array2d_copy(this, serialized);
    }

    /**
     * �R�s�[�R���X�g���N�^
     *
     * @param array �R�s�[��
     */
    Array2D_Dense(const self_t &array)
        : super_t(array.m_rows, array.m_columns){
      if(m_Values = array.m_Values){(*(ref = array.ref))++;}
    }
    /**
     * �f�X�g���N�^�B
     * �Q�ƃJ�E���^�����Z����Ƌ��ɁA�����J�E���^��0�̏ꍇ�A
     * �m�ۂ�����������delete�ɂ���ĉ�����܂��B
     */
    ~Array2D_Dense(){
      if(ref && ((--(*ref)) <= 0)){
        //std::cout << "Trashed" << std::endl;
        delete [] m_Values;
        delete ref;
      }
    }

    /**
     * ������Z�q�B
     * �����ȃV�����[�R�s�[���s���܂��B
     *
     * @return (Array2D_Dense<T>) �������g
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
  std::memcpy(dist->buffer(), src,
      sizeof(T) * dist->rows() * dist->columns());
}

/**
 * �s����s��p��2�����z����R�s�[���邽�߂̓��ꉻ
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
 * @brief �ʂ�2�����z��ɈϏ����s��2�����z��
 *
 * �ʂ�2�����z��ɈϏ����s��2�����z����`�����N���X�B
 * �Ⴆ�Γ]�u�s��Ȃǂ̓C���f�b�N�X��]�u����݂̂Ŏ��Ԃ��R�s�[���Ȃ������Ƃ�
 * �I�[�o�[�w�b�h�ƂȂ�܂��B
 * �����ŁA���̂悤�ȍs��̑���݂̂����ւ���N���X���Ԃɒ��p�����邱�Ƃɂ����
 * ���Z�������ɏ������邱�Ƃ��\�ɂȂ�܂��B
 *
 * @param T ���Z���x�Adouble�ȂǁB
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
     * �Ϗ����Ԃ��܂��B
     *
     * @return (Array2D<T>) �Ϗ���
     */
    root_t &getTarget() const{return *m_target;}

  public:
    /**
     * �Ϗ����Ԃ��܂��B
     *
     * @return (Array2D<T>) �Ϗ���
     */
    const root_t *getParent() const{return m_target;}

    /**
     * Array2D_Partial(����2�����z��)�N���X�̃R���X�g���N�^�B
     *
     * @param rows �s��
     * @param columns ��
     * @param array ���̔z��
     */
    Array2D_Delegate(
        const unsigned int &rows, const unsigned int &columns,
        const root_t &array) throw(StorageException)
        : super_t(rows, columns), m_target(array.shallow_copy()){}

    /**
     * �R�s�[�R���X�g���N�^
     *
     * @param array �R�s�[��
     * @throw StorageException �R�s�[�ɂ����ĂȂ�炩�̗�O�����������ꍇ
     */
    Array2D_Delegate(
        const self_t &array) throw(StorageException)
        : super_t(array.rows(), array.columns()),
        m_target(array.m_target->shallow_copy()){}

    /**
     * �f�X�g���N�^�B
     */
    ~Array2D_Delegate(){delete m_target;}

    /**
     * �w�肵���s�񐬕���Ԃ��܂��B
     *
     * @param row �s�C���f�b�N�X(�J�n�ԍ���0�`)
     * @param column ��C���f�b�N�X(�J�n�ԍ���0�`)
     * @return (const T &) ����
     * @throw StorageException �C���f�b�N�X���s���̏ꍇ
     */
    const T &operator()(
        const unsigned int &row,
        const unsigned int &column) const throw(StorageException){
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
     * ��2�����z��ɗ��Ƃ����݂܂��B
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
     * ����(�f�B�[�v�R�s�[)���܂��B
     * ������͒P���Ȗ�2�����z��ɂȂ�܂��B
     *
     * @return (root_t) �R�s�[
     * @throw StorageException ���������m�ۂł��Ȃ������ꍇ��
     */
    root_t *copy() const throw(StorageException){
      return dense().shallow_copy();
    }
};

/**
 * @brief �]���q2�����z��
 *
 * �]���q2�����z�������킷�N���X�B
 *
 * @param T ���Z���x�Adouble�Ȃ�
 */
template<class T>
class Array2D_CoFactor : public Array2D_Delegate<T>{

  private:
    unsigned int m_RowCoFactor;     ///< �]���q���w�肷�錳�̍s��ł̍s�C���f�b�N�X
    unsigned int m_ColumnCoFactor;  ///< �]���q���w�肷�錳�̍s��ł̗�C���f�b�N�X

  protected:
    /**
     * �]���q�s��Ԃ��܂��B
     *
     * @return (int) �s�C���f�b�N�X
     */
    unsigned int row_cofactor() const{return m_RowCoFactor;}
    /**
     * �]���q���Ԃ��܂��B
     *
     * @return (int) ��C���f�b�N�X
     */
    unsigned int column_cofactor() const{return m_ColumnCoFactor;}

  public:
    /**
     * �V�����[�R�s�[�����܂��B
     *
     * @return (Array2D *)�������g
     */
    Array2D<T> *shallow_copy() const{return new Array2D_CoFactor(*this);}

    /**
     * Array2D_Partial(����2�����z��)�N���X�̃R���X�g���N�^�B
     *
     * @param array ���̔z��
     * @param rowCoFactor �]���q�ƂȂ錳��2�����z���ł̍s�C���f�b�N�X
     * @param columnCoFactor ��������C���f�b�N�X
     * @throw StorageException ���炩�̗�O�����������ꍇ
     */
    Array2D_CoFactor(const Array2D<T> &array,
        const unsigned int &rowCoFactor,
        const unsigned int &columnCoFactor) throw(StorageException)
        : Array2D_Delegate<T>(array.rows() - 1, array.columns() - 1, array),
        m_RowCoFactor(rowCoFactor), m_ColumnCoFactor(columnCoFactor){}

    /**
     * �R�s�[�R���X�g���N�^�B
     *
     * @param array �R�s�[��
     * @throw StorageException
     */
    Array2D_CoFactor(const Array2D_CoFactor &array) throw(StorageException)
        : Array2D_Delegate<T>(array),
        m_RowCoFactor(array.row_cofactor()), m_ColumnCoFactor(array.column_cofactor()){}

    /**
     * �w�肵���s�񐬕���Ԃ��܂��B
     *
     * @param row �s�C���f�b�N�X(�J�n�ԍ���0�`)
     * @param column ��C���f�b�N�X(�J�n�ԍ���0�`)
     * @return (const T &) ����
     * @throw StorageException �C���f�b�N�X���s���ȏꍇ�Ȃ�
     */
    const T &operator()(
        const unsigned int &row,
        const unsigned int &column) const throw(StorageException){
      return Array2D_Delegate<T>::operator()(
          (row < row_cofactor() ? row : row + 1), (column < column_cofactor() ? column : column + 1));
    }
};

/**
 * @brief �]�u2�����z��
 *
 * �]�u2�����z�������킷�N���X
 * �v�f�����߂�ۂɌ��̍s��̍s�Ɨ�����ւ��ėv�f�𒊏o���邱�Ƃɂ����
 * �]�u�\���������Ɏ������Ă��܂��B
 *
 * @param T ���Z���x
 */
template<class T>
class Array2D_Transpose : public Array2D_Delegate<T>{

  public:
    /**
     * �V�����[�R�s�[�����܂��B
     *
     * @return (Array2D *)�������g
     */
    Array2D<T> *shallow_copy() const{return new Array2D_Transpose(*this);}

    /**
     * Array2D_Transpose(�]�u2�����z��)�N���X�̃R���X�g���N�^�B
     *
     * @param array ���̔z��
     * @throw StorageException
     */
    Array2D_Transpose(const Array2D<T> &array) throw(StorageException)
                       : Array2D_Delegate<T>(array.columns(), array.rows(), array){}

    /**
     * �R�s�[�R���X�g���N�^
     *
     * @param array �R�s�[��
     * @throw StorageException
     */
    Array2D_Transpose(const Array2D_Transpose &array) throw(StorageException)
      : Array2D_Delegate<T>(array){}

    /**
     * �w�肵���s�񐬕���Ԃ��܂��B
     *
     * @param row �s�C���f�b�N�X(�J�n�ԍ���0�`)
     * @param column ��C���f�b�N�X(�J�n�ԍ���0�`)
     * @return (const T &) ����
     * @throw StorageException �C���f�b�N�X���s���ȏꍇ�Ȃ�
     */
    const T &operator()(
        const unsigned int &row,
        const unsigned int &column) const throw(StorageException){
      return Array2D_Delegate<T>::operator()(column, row);
    }
    
    /**
     * ��2�����z��ɗ��Ƃ����݂܂��B
     * ���ۂ͋K��N���X�̊֐� Array2D_Delegate<T>::dense() ���Ăяo���Ă��邾���ŁA
     * ���ꉻ�̂��߂̃G���g���|�C���g�ł��B
     *
     * @return (Array2D_Dense<T> *)
     */
    Array2D_Dense<T> dense() const {
      return Array2D_Delegate<T>::dense();
    }
};

/**
 * @brief ����2�����z��
 *
 * ����2�����z�������킷�N���X�B
 *
 * @param T ���Z���x�Adouble�Ȃ�
 */
template<class T>
class Array2D_Partial : public Array2D_Delegate<T>{

  private:
    unsigned int m_RowOffset;     ///< �����s�񂪊J�n���錳�̍s��ł̍s�C���f�b�N�X
    unsigned int m_ColumnOffset;  ///< �����s�񂪊J�n���錳�̍s��ł̗�C���f�b�N�X

  protected:
    /**
     * �I�t�Z�b�g�s��Ԃ��܂��B
     *
     * @return (int) �s��
     */
    unsigned int row_offset() const{return m_RowOffset;}
    /**
     * �I�t�Z�b�g���Ԃ��܂��B
     *
     * @return (int) ��
     */
    unsigned int column_offset() const{return m_ColumnOffset;}

  public:
    /**
     * �V�����[�R�s�[�����܂��B
     *
     * @return (Array2D *)�������g
     */
    Array2D<T> *shallow_copy() const{return new Array2D_Partial(*this);}

    /**
     * Array2D_Partial(����2�����z��)�N���X�̃R���X�g���N�^�B
     *
     * @param rows �s��
     * @param columns ��
     * @param array ���̔z��
     * @param rowOffset ����2�����z���(0,0)�ƂȂ錳��2�����z��̃s�{�b�g�̍s�C���f�b�N�X
     * @param columnOffset ��������C���f�b�N�X
     * @throw StorageException ���炩�̗�O�����������ꍇ
     */
    Array2D_Partial(
        const unsigned int &rows, const unsigned int &columns,
        const Array2D<T> &array,
        const unsigned int &rowOffset, const unsigned int &columnOffset)
        throw(StorageException)
        : Array2D_Delegate<T>(rows, columns, array),
        m_RowOffset(rowOffset), m_ColumnOffset(columnOffset){}

    /**
     * �R�s�[�R���X�g���N�^�B
     *
     * @param array �R�s�[��
     * @throw StorageException
     */
    Array2D_Partial(const Array2D_Partial &array) throw(StorageException)
        : Array2D_Delegate<T>(array),
        m_RowOffset(array.row_offset()), m_ColumnOffset(array.column_offset()){}

    /**
     * �w�肵���s�񐬕���Ԃ��܂��B
     *
     * @param row �s�C���f�b�N�X(�J�n�ԍ���0�`)
     * @param column ��C���f�b�N�X(�J�n�ԍ���0�`)
     * @return (const T &) ����
     * @throw StorageException �C���f�b�N�X���s���ȏꍇ�Ȃ�
     */
    const T &operator()(
        const unsigned int &row,
        const unsigned int &column) const throw(StorageException){
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
 */
template <class T>
class Matrix{
  public:
    typedef Matrix<T> self_t;
    typedef Array2D<T> storage_t;

  protected:
    storage_t *m_Storage; ///< �����I�ɗ��p����2�����z��̃�����

    /**
     * Matrix�N���X�̃R���X�g���N�^�B
     * �X�g���[�W���w�肵�ĐV�����s������܂��B
     *
     * @param storage �X�g���[�W
     */
    Matrix(const storage_t *storage) : m_Storage(const_cast<storage_t *>(storage)){}
    
    static self_t make_instance(const storage_t *storage){
      return self_t(storage);
    }

    /**
     * Matrix�N���X���쐬����w���p�֐��B
     * �w��̍s���A�w��̗񐔂ōs��𐶐����܂����A
     * �����ɂ��Ă͏��������s��Ȃ����ߕs��ł��B
     *
     * @param rows �s��
     * @param columns ��
     * @throw MatrixException
     */
    static self_t naked(
        const unsigned int &rows,
        const unsigned int &columns) throw(MatrixException){
      return Matrix(new Array2D_Dense<T>(rows, columns));
    }

  public:

    /**
     * �����I�ȕۑ��`����Ԃ��܂��B
     *
     * @return (const storage_t *) �X�g���[�W
     */
    const storage_t *storage() const{return m_Storage;}

    /**
     * Matrix�N���X�̃R���X�g���N�^�B
     * �����I��2�����z��p�̃��������m�ۂ��Ȃ���Ԃŏ��������������܂��B
     *
     */
    Matrix() : m_Storage(NULL){}

    /**
     * Matrix�N���X�̃R���X�g���N�^�B
     * �w��̍s���A�w��̗񐔂ōs��𐶐����܂��B
     * �܂������͂��ׂ�T(0)�ŏ���������܂��B
     *
     * @param rows �s��
     * @param columns ��
     * @throw MatrixException
     */
    Matrix(
        const unsigned int &rows,
        const unsigned int &columns) throw(MatrixException)
        : m_Storage(new Array2D_Dense<T>(rows, columns)){m_Storage->clear();}

    /**
     * Matrix�N���X�̃R���X�g���N�^�B
     * �w��̍s���A�w��̗񐔂ōs��𐶐����܂��B
     * �܂�������seialized�ŕ�������܂��B
     *
     * @param rows �s��
     * @param columns ��
     * @param serialized ����
     * @throw MatrixException
     */
    Matrix(
        const unsigned int &rows,
        const unsigned int &columns,
        const T *serialized) throw(MatrixException)
        : m_Storage(new Array2D_Dense<T>(rows, columns, serialized)){}

    /**
     * �R�s�[�R���X�g���N�^�B
     * �V�����[�R�s�[�𐶐����܂��B
     *
     * @param matrix �R�s�[��
     */
    Matrix(const self_t &matrix) : m_Storage(matrix.m_Storage->shallow_copy()){}
    /**
     * �f�X�g���N�^�B
     */
    virtual ~Matrix(){delete m_Storage;}

  protected:
    /**
     * ������Z�q���T�|�[�g���邽�߂̊֐�
     * �����I�ɂ̓V�����[�R�s�[���s���Ă��܂��B
     *
     * @return (self_t) �������g
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
     * ������Z�q�B
     *
     * @return (self_t) �������g
     */
    self_t &operator=(const self_t &matrix){
      return substitute(matrix);
    }

    /**
     * �s��𕡐�(�f�B�[�v�R�s�[)���܂��B
     *
     * @return (self_t) �R�s�[
     * @throw MatrixException
     */
    self_t copy() const throw(MatrixException){
      return self_t(m_Storage->copy());
    }

    /**
     * �s����Ԃ��܂��B
     *
     * @return (int) �s��
     */
    unsigned int rows() const{return m_Storage->rows();}
    /**
     * �񐔂�Ԃ��܂��B
     *
     * @return (int) ��
     */
    unsigned int columns() const{return m_Storage->columns();}

    /**
     * �w�肵���s�񐬕���Ԃ��܂��B
     *
     * @param row �s�C���f�b�N�X(�J�n�ԍ���0�`)
     * @param column ��C���f�b�N�X(�J�n�ԍ���0�`)
     * @return (const T &) ����
     * @throw MatrixException �C���f�b�N�X���s���ȏꍇ�Ȃ�
     */
    const T &operator()(
        const unsigned int &row,
        const unsigned int &column) const throw(MatrixException){
      return m_Storage->operator()(row, column);
    }
    T &operator()(
        const unsigned int &row,
        const unsigned int &column) throw(MatrixException){
      return const_cast<T &>(static_cast<const self_t &>(*this)(row, column));
    }
    /**
     * �w�肵���s�񐬕���Ԃ��܂��B(Matlab����)
     *
     * @param row �s�ԍ�(�J�n�ԍ���1�`)
     * @param column ��ԍ�(�J�n�ԍ���1�`)
     * @return (T) ����
     * @throw MatrixException �ԍ����s���ȏꍇ�Ȃ�
     */
    T &index(const int &row, const int &column) throw(MatrixException){
      return (*this)(row - 1, column - 1);
    }
    
    /**
     * �s��̓��e�����������A���ׂ܂�
     * 
     * @param matrix ��r����ʂ̍s��
     * @return (bool) �s�񂪓������ꍇtrue�A�ȊOfalse
     */
    bool operator==(const self_t &matrix) const {
      if(m_Storage != matrix.m_Storage){
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
    
    bool operator!=(const self_t &matrix) const {
      return !(operator==(matrix));
    }

    /**
     * �v�f���[���N���A���܂�
     *
     * @return (self_t) �[���N���A���ꂽ�������g
     */
    self_t &clear(){
      m_Storage->clear();
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
     * �s���]�u���܂��B
     * �]�u���ꂽ�s��͂��Ƃ̍s��ƃ����N���Ă��܂��B
     * ���Ƃ̍s��Ƃ̐؂藣�����s���ɂ�transpose().copy()�Ƃ��Ă��������B
     *
     * @return (TransposedMatrix<T>) �]�u�s��
     */
    TransposedMatrix<T> transpose() const{
      return TransposedMatrix<T>(*this);
    }

    /**
     * �w�肵�������s���Ԃ��܂��B
     *
     * @param rowSize �s�T�C�Y
     * @param columnSize ��T�C�Y
     * @param rowOffset �J�n�s�C���f�b�N�X
     * @param columnOffset �J�n��C���f�b�N�X
     * @return (PartialMatrix<T>) �����s��
     * @throw MatrixException �C���f�b�N�X���s���ȏꍇ
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
     * �w�肵���s�̍s�x�N�g����Ԃ��܂��B
     *
     * @param row �s�C���f�b�N�X
     * @return (self_t) �s�x�N�g��
     * @throw MatrixException �C���f�b�N�X���s���ȏꍇ
     */
    PartialMatrix<T> rowVector(const unsigned int &row) const throw(MatrixException){
      if(row >= rows()){throw MatrixException("Index Incorrect!!");}
      return PartialMatrix<T>(*this, 1, columns(), row, 0);
    }
    /**
     * �w�肵����̗�x�N�g����Ԃ��܂��B
     *
     * @param column ��C���f�b�N�X
     * @return (self_t) ��x�N�g��
     * @throw MatrixException �C���f�b�N�X���s���ȏꍇ
     */
    PartialMatrix<T> columnVector(const unsigned int &column) const throw(MatrixException){
      if(column >= columns()){throw MatrixException("Index Incorrect!!");}
      return PartialMatrix<T>(*this, rows(), 1, 0, column);
    }

    /**
     * �s�����ւ��܂��B�j��I���\�b�h�ł��B
     *
     * @param row1 �s�C���f�b�N�X1
     * @param row2 �s�C���f�b�N�X2
     * @return (self_t) �������g
     * @throw MatrixException �C���f�b�N�X���s���ȏꍇ
     */
    self_t &exchangeRows(const unsigned int &row1, const unsigned int &row2) throw(MatrixException){
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
    self_t &exchangeColumns(const unsigned int &column1, const unsigned int &column2){
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
    bool isDifferentSize(const self_t &matrix) const{
      return (rows() != matrix.rows()) || (columns() != matrix.columns());
    }

    /**
     * �s��̃g���[�X��Ԃ��܂��B
     *
     * @param do_check �����s�񂩂𒲂ׂ�A�f�t�H���gtrue
     * @return (T) �g���[�X
     */
    T trace(bool do_check = true) const throw(MatrixException) {
      if(do_check && !isSquare()){throw MatrixException("Operation void!!");}
      T tr(0);
      for(unsigned i(0); i < rows(); i++){
        tr += (*this)(i, i);
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
     * �s��̐����S�Ă��w��{���܂��B�j��I���\�b�h�ł��B
     *
     * @param scalar �{��
     * @return (self_t) �������g
     */
    self_t &operator*=(const T &scalar){
#ifdef USE_ARRAY2D_ITERATOR
      ScalarMultiplier op(scalar);
      m_Storage->all_elements(op);
#else
      for(unsigned int i(0); i < rows(); i++){
        for(unsigned int j(0); j < columns(); j++){
          (*this)(i, j) *= scalar;
        }
      }
#endif
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
     * �s��𐬕����Ƃɉ��Z���܂��B�j��I���\�b�h�ł��B
     *
     * @param matrix ������s��
     * @return (self_t) �������g
     */
    self_t &operator+=(const self_t &matrix){
      if(isDifferentSize(matrix)){throw MatrixException("Operation void!!");}
#ifdef USE_ARRAY2D_ITERATOR
      PlusEqual op(*this);
      matrix.m_Storage->all_elements(op);
#else
      for(unsigned int i(0); i < rows(); i++){
        for(unsigned int j(0); j < columns(); j++){
          (*this)(i, j) += matrix(i, j);
        }
      }
#endif
      return *this;
    }

    /**
     * �s��𐬕����Ƃɉ��Z���܂��B
     *
     * @param matrix ������s��
     * @return (self_t) ����
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
     * �s��𐬕����ƂɌ��Z���܂��B
     *
     * @param matrix �����s��
     * @return (self_t) �������g
     */
    self_t &operator-=(const self_t &matrix){
      if(isDifferentSize(matrix)){throw MatrixException("Operation void!!");}
#ifdef USE_ARRAY2D_ITERATOR
      MinusEqual op(*this);
      matrix.m_Storage->all_elements(op);
#else
      for(unsigned int i(0); i < rows(); i++){
        for(unsigned int j(0); j < columns(); j++){
          (*this)(i, j) -= matrix(i, j);
        }
      }
#endif
      return *this;
    }

    /**
     * �s��𐬕����ƂɌ��Z���܂��B
     *
     * @param matrix �����s��
     * @return (self_t) ����
     */
    self_t operator-(const self_t &matrix) const{return (copy() -= matrix);}

    /**
     * �s�����Z���܂��B
     *
     * @param matrix ������s��
     * @return (self_t) ����
     * @throw MatrixException �s��̐ώZ���������Ȃ��ꍇ(�I�y�����h�s��̗񐔂������s��̍s���Ɠ������Ȃ�)
     */
    self_t operator*(const self_t &matrix) const throw(MatrixException){
      if(columns() != matrix.rows()){
        throw MatrixException("Operation void!!");
      }
      self_t result(self_t::naked(rows(), matrix.columns()));
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
     * �s�����Z���܂��B(�]�u�s��o�[�W����)
     *
     * @param matrix ������s��
     * @return (self_t) ����
     * @throw MatrixException �s��̐ώZ���������Ȃ��ꍇ(�I�y�����h�s��̗񐔂������s��̍s���Ɠ������Ȃ�)
     */
    self_t operator*(const TransposedMatrix<T> &matrix) const throw(MatrixException){
      return operator*((const self_t &)matrix);
    }
    
    /**
     * �s�����Z���܂��B�j��I���\�b�h�ł��B
     *
     * @param matrix ������s��
     * @return (self_t) �������g
     * @throw MatrixException �s��̐ώZ���������Ȃ��ꍇ(�I�y�����h�s��̗񐔂������s��̍s���Ɠ������Ȃ�)
     */
    template <class RhsMatrix>
    self_t &operator*=(const RhsMatrix &matrix) throw(MatrixException){
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
     * ��s��(�]���q�s��)�����߂܂��B
     *
     * @param row �s�C���f�b�N�X
     * @param column ��C���f�b�N�X
     * @return (CoMatrix) ��s��
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
     * �s�񎮂��v�Z���܂��B
     *
     * @param do_check �����s��`�F�b�N���s����(�f�t�H���gtrue)
     * @return (T) ����
     * @throw MatrixException �����s��ł͂Ȃ��s�񎮂��v�Z���邱�Ƃ��ł��Ȃ��ꍇ
     */
    T determinant(bool do_check = true) const throw(MatrixException){
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


      // L(Ux) = y �� y' = (Ux)���܂�����
      Matrix<T2> y_copy(y.copy());
      Matrix<T2> y_prime(Matrix<T2>::naked(y.rows(), 1));
      for(unsigned i(0); i < rows(); i++){
        y_prime(i, 0) = y_copy(i, 0) / L(i, i);
        for(unsigned j(i + 1); j < rows(); j++){
          y_copy(j, 0) -= L(j, i) * y_prime(i, 0);
        }
      }

      // ������Ux = y'�� x������
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
     * LU���������܂��B
     * (0, 0)�`(n-1, n-1):  L�s��
     * (0, n)�`(n-1, 2n-1): U�s��
     *
     * @param do_check �Ώ̍s��`�F�b�N���s����(�f�t�H���gtrue)
     * @return (self_t) LU����
     * @throw MatrixException �����s��ł͂Ȃ�LU�������v�Z���邱�Ƃ��ł��Ȃ��ꍇ
     */
     self_t decomposeLU(bool do_check = true) const throw(MatrixException){
       if(do_check && !isSquare()){throw MatrixException("Operation void!!");}
       self_t LU(self_t::naked(rows(), columns() * 2));
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
    self_t decomposeUD(bool do_check = true) const throw(MatrixException){
      if(do_check && !isSymmetric()){throw MatrixException("Operation void");}
      self_t P(copy());
      self_t UD(rows(), columns() * 2);
#define U(i, j) UD(i, j)
#define D(i, j) UD(i, j + columns())
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
      for(int i(0); i < rows(); i++){
        for(int j(0); j < columns(); j++){
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
     * @throw MatrixException �t�s����v�Z���邱�Ƃ��ł��Ȃ��ꍇ
     */
    self_t &operator/=(const self_t &matrix) throw(MatrixException){return (*this) *= matrix.inverse();}
    /**
     * �t�s��������܂��B
     *
     * @param matrix �s��
     * @return (self_t) ����
     * @throw MatrixException �t�s����v�Z���邱�Ƃ��ł��Ȃ��ꍇ
     */
    self_t operator/(const self_t &matrix) const throw(MatrixException){return (copy() /= matrix);}

    /**
     * �s�{�b�g���w�肵�āA���Z���܂��B
     * �j��I�ł��B
     *
     * @param row �s�C���f�b�N�X
     * @param column ��C���f�b�N�X
     * @param matrix �����s��
     */
    self_t &pivotMerge(const int &row, const int &column, const self_t &matrix){
      for(unsigned int i(0); i < matrix.rows(); i++){
        if(row + i < 0){continue;}
        else if(row + i >= rows()){break;}
        for(unsigned int j(0); j < matrix.columns(); j++){
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
    self_t pivotAdd(const int &row, const int &column, const self_t &matrix) const{
      return copy().pivotMerge(row, column, matrix);
    }

    /**
     * �n�E�X�z���_�[�ϊ������ăw�b�Z���x���N�s��𓾂܂��B
     *
     * @param transform �ϊ��ɗp�����s��̐ς��i�[����|�C���^
     * @return (self_t) �w�b�Z���x���N�s��
     * @throw MatrixException �����s��ł͂Ȃ��v�Z���邱�Ƃ��ł��Ȃ��ꍇ
     */
    self_t hessenberg(Matrix *transform) const throw(MatrixException){
      if(!isSquare()){throw MatrixException("Operation void!!");}

      self_t result(copy());
      for(unsigned int j(0); j < columns() - 2; j++){
        T t(0);
        for(unsigned int i(j + 1); i < rows(); i++){
          t += pow(result(i, j), 2);
        }
        T s = ::sqrt(t);
        if(result(j + 1, j) < 0){s *= -1;}

        self_t omega(self_t::naked(rows() - (j+1), 1));
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

    /**
     * �n�E�X�z���_�[�ϊ������ăw�b�Z���x���N�s��𓾂܂��B
     *
     * @return (self_t) �w�b�Z���x���N�s��
     */
    self_t hessenberg() const throw(MatrixException){return hessenberg(NULL);}

    /**
     * 2�����s��̌ŗL�l�����߂܂��B
     *
     * @param row 2�����s��̍��㍀�̍s�C���f�b�N�X
     * @param column 2�����s��̍��㍀�̗�C���f�b�N�X
     * @param upper ����(�ŗL�l1)
     * @param lower ����(�ŗL�l2)
     */
    void eigen22(const int &row, const int &column, Complex<T> &upper, Complex<T> &lower) const throw(MatrixException){
      T a((*this)(row, column)),
        b((*this)(row, column + 1)),
        c((*this)(row + 1, column)),
        d((*this)(row + 1, column + 1));
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
    Matrix<Complex<T> > eigen(
        const T &threshold_abs,
        const T &threshold_rel) const throw(MatrixException){

      if(!isSquare()){throw MatrixException("Operation void!!");}

      //�p���[�@(�ׂ���@)
      /*self_t result(rows(), rows() + 1);
      self_t source = copy();
      for(int i(0); i < columns(); i++){result(0, i) = T(1);}
      for(int i(0); i < columns(); i++){
        while(true){
          self_t approxVec = source * result.columnVector(i);
          T approxVal(0);
          for(int j(0); j < approxVec.rows(); j++){approxVal += pow(approxVec(j, 0), 2);}
          approxVal = sqrt(approxVal);
          for(int j(0); j < approxVec.rows(); j++){result(j, i) = approxVec(j, 0) / approxVal;}
          T before = result(i, rows());
          if(abs(before - (result(i, rows()) = approxVal)) < threshold){break;}
        }
        for(int j(0); (i < rows() - 1) && (j < rows()); j++){
          for(int k(0); k < rows(); k++){
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

      //���ʂ̊i�[�p�̍s��
      Matrix<Complex<T> > result(rows(), rows() + 1);

      //�ŗL�l�̌v�Z
      Matrix<Complex<T> > lambda(result.partial(rows(), 1, 0, rows()));   //�ŗL�l

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

        //�ʁA��*�̍X�V(4.143)
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

        //��������

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
      //�ŗL�x�N�g���̌v�Z
      Matrix<Complex<T> > x(rows(), rows());  //�ŗL�x�N�g��
      A = A_;

      for(unsigned int j(0); j < rows(); j++){
        int n = rows();
        for(unsigned int i(0); i < j; i++){
          if((lambda(j, 0) - lambda(i, 0)).abs() <= threshold_abs){--n;}
        }
        //std::cout << n << ", " << lambda(j, 0) << std::endl;
        x(--n, j) = 1;
        while(n-- > 0){
          x(n, j) = x(n+1, j) * (lambda(j, 0) - A(n+1, n+1));
          for(unsigned int i(n+2); i < rows(); i++){
            x(n, j) -= x(i, j) * A(n+1, i);
          }
          if(A(n+1, n)){x(n, j) /= A(n+1, n);}
        }
        //std::cout << x.partial(rows(), 1, 0, j).transpose() << std::endl;
      }
#else
      //�ŗL�x�N�g���̌v�Z(�t�����@)
      Matrix<Complex<T> > x(Matrix<Complex<T> >::getI(rows()));  //�ŗL�x�N�g��
      A = A_;
      Matrix<Complex<T> > A_C(rows(), rows());
      for(unsigned int i(0); i < rows(); i++){
        for(unsigned int j(0); j < columns(); j++){
          A_C(i, j) = A(i, j);
        }
      }

      for(unsigned int j(0); j < rows(); j++){
        // http://www.prefield.com/algorithm/math/eigensystem.html ���Q�l��
        // ���A�ŗL�l���������ꍇ�̑Ώ����@�Ƃ��āA
        // http://www.nrbook.com/a/bookcpdf/c11-7.pdf
        // ���Q�l�ɁA�l��U���Ă݂邱�Ƃɂ���
        Matrix<Complex<T> > A_C_lambda(A_C.copy());
        Complex<T> approx_lambda(lambda(j, 0));
        if((A_C_lambda(j, j) - approx_lambda).abs() <= 1E-3){
          approx_lambda += 2E-3;
        }
        for(unsigned int i(0); i < rows(); i++){
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
      for(int i(0); i < rows(); i++){
        lambda2(i, i) = lambda(i, 0);
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
        Complex<T> _norm;
        for(unsigned int i(0); i < rows(); i++){
          _norm += result(i, j).abs2();
        }
        T norm = ::sqrt(_norm.real());
        for(unsigned int i(0); i < rows(); i++){
          result(i, j) /= norm;
        }
        //std::cout << result.partial(rows(), 1, 0, j).transpose() << std::endl;
      }

      return result;
    }

    /**
     * �ŗL�l�A�ŗL�x�N�g�������߂܂��B
     * �ԋp�l��Matrix�^�ŁA
     * (0,0)�`(n-1,n-1)�v�f���ŗL�x�N�g���̍s��
     * (0,n)�`(n-1,n)�v�f���Ή�����ŗL�l�̗�x�N�g��
     * �ɂȂ��Ă��܂��B
     * (�ŗL�x�N�g��1,�ŗL�x�N�g��2,�c,�ŗL�x�N�g��n,�ŗL�l)��n�~(n+1)�s��
     *
     * @return (Matrix<Complex<T> >) �ŗL�l�A�ŗL�x�N�g��
     */
    Matrix<Complex<T> > eigen() const throw(MatrixException){return eigen(1E-10, 1E-7);}

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
    Matrix<Complex<T> > sqrt(const Matrix<Complex<T> > &eigen_mat) const throw(MatrixException) {
      int n(eigen_mat.rows());
      Matrix<Complex<T> > VsD(eigen_mat.partial(n, n, 0, 0));
      Matrix<Complex<T> > nV(VsD.inverse());
      for(int i(0); i < n; i++){
        VsD.partial(n, 1, 0, i) *= std::sqrt(eigen_mat(i, n));
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
    Matrix<Complex<T> > sqrt(
        const T &threshold_abs,
        const T &threshold_rel) const throw(MatrixException){
      return sqrt(eigen(threshold_abs, threshold_rel));
    }

    /**
     * �s��̕����������߂܂��B
     * �ԋp�l��Matrix�^�ł��B
     *
     * @return (Matrix<Complex<T> >) ������
     */
    Matrix<Complex<T> > sqrt() const throw(MatrixException){
      return sqrt(eigen());
    }

    /**
     * �s������₷���`�ŏo�͂��܂��B
     *
     */
    friend std::ostream &operator<<(std::ostream &out, const self_t &matrix){
      if(matrix.m_Storage){
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

/**
 * @brief �Ϗ����ꂽ�s��
 *
 * �Ϗ����ꂽ�s��(�����s��A�]�u�s��Ȃǂ̔h���s��)�̊��N���X
 * ���̍s��N���X�ł̓��C�g�E�G�C�g����(������A�V�����[�R�s�[�ɂ�����)�ł��邽�߁A
 * ���̂܂܂ł͔h���N���X�ɂ����ĕ��Q���������܂��B
 * ���Y���ڂ���������̂��A���̃N���X�̖����ł��B
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
            (*this)(i, j) = matrix(i, j);
          }
        }
      }
      return *this;
    }

    DelegatedMatrix(const typename super_t::storage_t *storage)
        : super_t(storage){}
    virtual ~DelegatedMatrix(){}

    /**
     * ������Z�q�B
     * ���N���X��Storage��ύX���鑀��ƈقȂ�A�����ǂ����̑�����s���܂��B
     *
     * @param matrix �������s��
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
 * @brief �]���q�s��(��s��)
 *
 * �]���q�s�������킷�N���X
 * �]���q2�����z���\������Array2D_CoFactor�Ƌ��͂��ĕ����s����������Ă��܂��B
 *
 * @see Array2D_Partial ����2�����z��
 */
template <class T>
class CoMatrix : public DelegatedMatrix<T>{
  protected:
    typedef Matrix<T> root_t;
    typedef DelegatedMatrix<T> super_t;
    typedef CoMatrix<T> self_t;

  public:
    /**
     * PartialMatrix(�����s��)�N���X�̃R���X�g���N�^�B
     *
     * @param matrix ���̍s��
     * @param rowOffset �]���q�ƂȂ�s�C���f�b�N�X
     * @param columnOffset ��������C���f�b�N�X
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
     * �f�X�g���N�^�B
     */
    ~CoMatrix(){}

    self_t &operator=(const root_t &matrix){
      return static_cast<self_t &>(super_t::substitute(matrix));
    }
};

/**
 * @brief �]�u�s��
 *
 * �]�u�s�������킷�N���X�B
 * �N���X���N���X�Ƃ��Ē�`�B
 * �]�u2�����z���\������Array2D_Transpose�Ƌ��͂��ē]�u�s����������Ă��܂��B
 *
 * @see Array2D_Transpose �]�u2�����z��
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
     * TransposedMatrix(�]�u�s��)�N���X�̃R���X�g���N�^�B
     *
     * @param matrix ���̍s��
     * @throw MatrixException
     */
    TransposedMatrix(const root_t &matrix) throw(MatrixException)
        : super_t(new Array2D_Transpose<T>(*(matrix.storage()))){}

    /**
     * �f�X�g���N�^�B
     */
    ~TransposedMatrix(){}

    /**
     * �]�u�s���]�u���Č��̍s��ɖ߂��܂��B
     * �ԋp�����s��͂��Ƃ̍s��ƃ����N���Ă��܂��B
     * ���Ƃ̍s��Ƃ̐؂藣�����s���ɂ�transpose().copy()�Ƃ��Ă��������B
     *
     * @return (Matrix<T>) �]�u�s��
     */
    inline root_t untranspose() const{
      return super_t::original();
    }

    self_t &operator=(const root_t &matrix){
      return static_cast<self_t &>(super_t::substitute(matrix));
    }
    
    /**
     * �s�����Z���܂��B(�]�u * ��]�u)
     *
     * @param matrix ������s��
     * @return (root_t) ����
     * @throw MatrixException �s��̐ώZ���������Ȃ��ꍇ(�I�y�����h�s��̗񐔂������s��̍s���Ɠ������Ȃ�)
     */
    root_t operator*(const root_t &matrix) const throw(MatrixException){
      return super_t::operator*(matrix);
    }
    
    /**
     * �s�����Z���܂��B(�]�u * �]�u)
     *
     * @param matrix ������s��
     * @return (root_t) ����
     * @throw MatrixException �s��̐ώZ���������Ȃ��ꍇ(�I�y�����h�s��̗񐔂������s��̍s���Ɠ������Ȃ�)
     */
    root_t operator*(const self_t &matrix) const throw(MatrixException){
      return operator*((const root_t &)matrix);
    }
};

/**
 * @brief �����s��
 *
 * �����s�������킷�N���X
 * �N���X���N���X�Ƃ��Ē�`�B
 * ����2�����z���\������Array2D_Partial�Ƌ��͂��ĕ����s����������Ă��܂��B
 *
 * @see Array2D_Partial ����2�����z��
 */
template <class T>
class PartialMatrix : public DelegatedMatrix<T>{
  protected:
    typedef Matrix<T> root_t;
    typedef DelegatedMatrix<T> super_t;
    typedef PartialMatrix<T> self_t;

  public:
    /**
     * PartialMatrix(�����s��)�N���X�̃R���X�g���N�^�B
     *
     * @param matrix ���̍s��
     * @param rows �s��
     * @param columns ��
     * @param rowOffset �����s���(0,0)�ƂȂ錳�̍s��̃s�{�b�g�̍s�C���f�b�N�X
     * @param columnOffset ��������C���f�b�N�X
     * @throw MatrixException
     */
    PartialMatrix(
        const root_t &matrix,
        const unsigned int &rows, const unsigned int &columns,
        const unsigned int &rowOffset, const unsigned int &columnOffset)
        throw(MatrixException)
        : super_t(new Array2D_Partial<T>(
        rows, columns,
        *(matrix.storage()),
        rowOffset, columnOffset)){}

    /**
     * �f�X�g���N�^�B
     */
    ~PartialMatrix(){}

    self_t &operator=(const root_t &matrix){
      return static_cast<self_t &>(super_t::substitute(matrix));
    }
};

#endif /* __MATRIX_H */
