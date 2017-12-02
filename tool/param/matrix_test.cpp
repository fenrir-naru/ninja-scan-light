#include <string>
#include <cstring>
#include <cmath>
#include <iostream>
#include <exception>

#include "param/complex.h"

#if 1
#include "param/matrix.h"
#else
class MatrixException: public std::exception{
  private:
    std::string what_str;
  public:
    MatrixException(const std::string &what_arg) : what_str(what_arg){}
    ~MatrixException() {}
    const char *what() const throw(){
      return what_str.c_str();
    }
};

class StorageException: public MatrixException{
  public:
    StorageException(const std::string &what_arg)
        : MatrixException(what_arg) {}
    ~StorageException() {}
};

template <class T>
class Array2D_Dense;

template<class T>
class Array2D {
  protected:
    unsigned int m_rows;    ///< Rows
    unsigned int m_columns; ///< Columns
    
    typedef Array2D<T> self_t;
    typedef Array2D<T> root_t;
    typedef Array2D_Dense<T> dense_t;
    
  public:
    typedef T content_t;

    Array2D(const unsigned int &rows, const unsigned int &columns)
        : m_rows(rows), m_columns(columns){}
    virtual ~Array2D(){}

    const unsigned int &rows() const{return m_rows;}
    const unsigned int &columns() const{return m_columns;}

    virtual const T &operator()(
        const unsigned int &row,
        const unsigned int &column) const = 0;
    T &operator()(
        const unsigned int &row,
        const unsigned int &column){
      return const_cast<T &>(const_cast<const self_t &>(*this)(row, column));
    }
    
    virtual void clear() = 0;

    virtual root_t *copy(const bool &is_deep = false) const = 0;
};

template <class T>
class Array2D_Dense : public Array2D<T> {
  protected:
    typedef Array2D_Dense<T> self_t;
    typedef Array2D<T> super_t;
    using typename super_t::root_t;
    
    using root_t::rows;
    using root_t::columns;

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
    Array2D_Dense(
        const unsigned int &rows,
        const unsigned int &columns)
        : super_t(rows, columns),
        values(new T[rows * columns]), ref(new int(1)) {
    }
    Array2D_Dense(
        const unsigned int &rows,
        const unsigned int &columns,
        const T *serialized)
        : super_t(rows, columns),
        values(new T[rows * columns]), ref(new int(1)) {
      copy_raw(*this, serialized);
    }
    Array2D_Dense(const self_t &array)
        : super_t(array.m_rows, array.m_columns){
      if(values = array.values){(*(ref = array.ref))++;}
    }
    Array2D_Dense(const root_t &array)
        : values(new T[rows * columns]), ref(new int(1)) {
      T *buf;
      for(unsigned int i(0); i < array.rows(); ++i){
        for(unsigned int j(0); j < array.rows(); ++j){
          *(buf++) = array(i, j);
        }
      }
    }
    ~Array2D_Dense(){
      if(ref && ((--(*ref)) <= 0)){
        delete [] values;
        delete ref;
      }
    }
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
    
    const T &operator()(
        const unsigned int &row,
        const unsigned int &column) const throw(StorageException){
      if((row >= rows()) || (column >= columns())){
        throw StorageException("Index incorrect");
      }
      return values[(row * columns()) + column];
    }
    
    void clear(){
      clear_raw(*this);
    }

    root_t *copy(const bool &is_deep = false) const {
      return is_deep ? new self_t(rows(), columns(), values) : new self_t(*this);
    }
};

template <class T, template <class> class Array2DType = Array2D_Dense>
class Matrix {
  public:
    typedef Array2DType<T> storage_t;
    typedef Matrix<T, Array2DType> self_t;

  protected:
    Array2D<T> *storage;
    Matrix(Array2D<T> *new_storage) : storage(new_storage) {}
    inline const storage_t *array2d() const{
      return static_cast<const storage_t *>(storage);
    }
    inline storage_t *array2d() {
      return const_cast<storage_t *>(const_cast<const self_t *>(this)->array2d());
    }

  public:
    Matrix() : storage(NULL){}
    Matrix(
        const unsigned int &rows,
        const unsigned int &columns)
        : storage(new storage_t(rows, columns)){
      array2d()->storage_t::clear();
    }
    Matrix(const self_t &matrix)
        : storage(matrix.storage
            ? matrix.array2d()->storage_t::copy(false)
            : NULL){}
    template <template <class> class Array2DAnotherType>
    Matrix(const Matrix<T, Array2DAnotherType> &matrix)
        : storage(matrix.storage
            ? new storage_t(matrix.storage)
            : NULL){}
    virtual ~Matrix(){delete storage;}

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
    self_t &operator=(const self_t &matrix){
      if(this != &matrix){
        delete storage;
        if(matrix.storage){
          storage = matrix.array2d()->storage_t::copy(false);
        }
      }
      return *this;
    }
    template <template <class> class Array2DAnotherType>
    self_t &operator=(const Matrix<T, Array2DAnotherType> &matrix){
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
    template <template <class> class Array2DAnotherType>
    bool operator==(const Matrix<T, Array2DAnotherType> &matrix) const {
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
    
    template <template <class> class Array2DAnotherType>
    bool operator!=(const Matrix<T, Array2DAnotherType> &matrix) const {
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
      // TODO
      self_t res(self_t::blank(new_rows, new_columns));
      for(unsigned int i_dst(0), i_src(row_offset); i_dst < res.rows();  ++i_src, ++i_dst){
        for(unsigned int j_dst(0), j_src(column_offset); j_dst < res.columns(); ++j_src, ++j_dst){
          res(i_dst, j_dst) = operator()(i_src, j_src);
        }
      }
      return res;
    }

    /**
     * 指定した行の行ベクトルを返します。
     *
     * @param row 行インデックス
     * @return (self_t) 行ベクトル
     */
    self_t rowVector(const unsigned int &row) const {
      return partial(1, columns(), row, 0);
    }
    /**
     * 指定した列の列ベクトルを返します。
     *
     * @param column 列インデックス
     * @return (self_t) 列ベクトル
     * @throw MatrixException インデックスが不正な場合
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
    template <template <class> class Array2DAnotherType>
    bool isDifferentSize(const Matrix<T, Array2DAnotherType> &matrix) const{
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
    template <template <class> class Array2DAnotherType>
    self_t &operator+=(const Matrix<T, Array2DAnotherType> &matrix) throw(MatrixException) {
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
    template <template <class> class Array2DAnotherType>
    self_t operator+(const Matrix<T, Array2DAnotherType> &matrix) const{return (copy() += matrix);}
    
    /**
     * 行列を成分ごとに減算します。
     *
     * @param matrix 引く行列
     * @return (self_t) 自分自身
     */
    template <template <class> class Array2DAnotherType>
    self_t &operator-=(const Matrix<T, Array2DAnotherType> &matrix) throw(MatrixException) {
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
    template <template <class> class Array2DAnotherType>
    self_t operator-(const Matrix<T, Array2DAnotherType> &matrix) const{return (copy() -= matrix);}

    /**
     * 行列を乗算します。
     *
     * @param matrix かける行列
     * @return (self_t) 結果
     * @throw MatrixException 行列の積算が成立しない場合(オペランド行列の列数が引数行列の行数と等しくない)
     */
    template <template <class> class Array2DAnotherType>
    self_t operator*(const Matrix<T, Array2DAnotherType> &matrix) const throw(MatrixException){
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
    template <class T2, template <class> class Array2DAnotherType>
    Matrix<T2, Array2DAnotherType> solve_linear_eq_with_LU(
        const Matrix<T2, Array2DAnotherType> &y, const bool &do_check = true)
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


      typedef Matrix<T2, Array2DAnotherType> y_t;
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
      for(unsigned int i(rows() - 1); i >= 0; i--){
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
    template <template <class> class Array2DAnotherType>
    self_t &operator/=(const Matrix<T, Array2DAnotherType> &matrix) {
        return (*this) *= matrix.inverse();
    }
    /**
     * 逆行列をかけます。
     *
     * @param matrix 行列
     * @return (self_t) 結果
     */
    template <template <class> class Array2DAnotherType>
    self_t operator/(const Matrix<T, Array2DAnotherType> &matrix) const {
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
    template <template <class> class Array2DAnotherType>
    self_t &pivotMerge(
        const unsigned int &row, const unsigned int &column,
        const Matrix<T, Array2DAnotherType> &matrix){
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
    template <template <class> class Array2DAnotherType>
    self_t pivotAdd(
        const unsigned int &row, const unsigned int &column,
        const Matrix<T, Array2DAnotherType> &matrix) const{
      return copy().pivotMerge(row, column, matrix);
    }

    /**
     * ハウスホルダー変換をしてヘッセンベルク行列を得ます。
     *
     * @param transform 変換に用いた行列の積を格納するポインタ
     * @return (self_t) ヘッセンベルク行列
     * @throw MatrixException 正方行列ではなく計算することができない場合
     */
    template <template <class> class Array2DAnotherType>
    self_t hessenberg(Matrix<T, Array2DAnotherType> *transform) const throw(MatrixException){
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

    /**
     * ハウスホルダー変換をしてヘッセンベルク行列を得ます。
     *
     * @return (self_t) ヘッセンベルク行列
     */
    self_t hessenberg() const {return hessenberg(NULL);}

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
        Complex<T> &upper, Complex<T> &lower) const {
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
    Matrix<Complex<T>, Array2DType> eigen(
        const T &threshold_abs = 1E-10,
        const T &threshold_rel = 1E-7) const throw(MatrixException){

      typedef Matrix<Complex<T>, Array2DType> res_t;

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
      Complex<T> p1, p2;
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
        Complex<T> approx_lambda(lambda(j));
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
        Complex<T> _norm;
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
    static Matrix<Complex<T>, Array2DType> sqrt(
        const Matrix<Complex<T>, Array2DType> &eigen_mat){
      unsigned int n(eigen_mat.rows());
      Matrix<Complex<T>, Array2DType> VsD(eigen_mat.partial(n, n, 0, 0));
      Matrix<Complex<T>, Array2DType> nV(VsD.inverse());
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
    Matrix<Complex<T>, Array2DType> sqrt(
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
    Matrix<Complex<T>, Array2DType> sqrt() const {
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
#endif

#include <iostream>
#include <sstream>
#include <cmath>
#include <ctime>
#include <set>

#include <cpptest.h>
#include <cpptest-suite.h>

#include "util/util.h"


#define SIZE 8
#define accuracy double
#define ACCEPTABLE_DELTA 1E-10

using namespace std;

#ifndef DEBUG_PRINT
#define DEBUG_PRINT false
#endif
#define dbg(exp, force) \
if((force) || DEBUG_PRINT){cerr << endl << exp;} \
else{stringstream ss; ss << exp;}

accuracy k_delta(unsigned i, unsigned j){
  return (i == j) ? 1 : 0;
}

class MatrixTestSuite : public Test::Suite{
  public:
    typedef Matrix<accuracy> matrix_t;
    typedef Matrix<Complex<accuracy> > cmatrix_t;
    MatrixTestSuite(){
      TEST_ADD(MatrixTestSuite::test_init);
      TEST_ADD(MatrixTestSuite::test_op_equal);
      TEST_ADD(MatrixTestSuite::test_copy);
      TEST_ADD(MatrixTestSuite::test_properties);
      TEST_ADD(MatrixTestSuite::test_getI);
      TEST_ADD(MatrixTestSuite::test_exchange);
      TEST_ADD(MatrixTestSuite::test_check);
      TEST_ADD(MatrixTestSuite::test_scalar_op);
      TEST_ADD(MatrixTestSuite::test_matrix_op);
      TEST_ADD(MatrixTestSuite::test_matrix_op_loop);
      //TEST_ADD(MatrixTestSuite::test_partialMatrix);
      TEST_ADD(MatrixTestSuite::test_decompose);
    }

  protected:
    matrix_t *A, *B;
    accuracy A_array[SIZE][SIZE], B_array[SIZE][SIZE];

    virtual void setup(){
      dbg(endl, false);
      A = new matrix_t(SIZE, SIZE);
      for(int i = 0; i < A->rows(); i++){
        A_array[i][i] = (*A)(i, i) = rand_regularized(0, 1);
        for(int j = i + 1; j < A->columns(); j++){
          A_array[i][j] = A_array[j][i]
              = (*A)(i, j) = (*A)(j, i) = rand_regularized(0, 1);
        }
      }
      B = new matrix_t(SIZE, SIZE);
      for(int i = 0; i < B->rows(); i++){
        B_array[i][i] = (*B)(i, i) = rand_regularized(0, 1);
        for(int j = i + 1; j < B->columns(); j++){
          B_array[i][j] = B_array[j][i]
              = (*B)(i, j) = (*B)(j, i) = rand_regularized(0, 1);
        }
      }
      dbg("A:" << *A << endl, false);
      dbg("B:" << *B << endl, false);
    }

    virtual void tear_down(){
      delete A;
      delete B;
    }

    template<class U, class V, class T>
    void matrix_compare_delta(
        const U &u, const V &v,
        const T &delta = ACCEPTABLE_DELTA){
      for(unsigned i(0); i < v.rows(); i++){
        for(unsigned j(0); j < v.columns(); j++){
          TEST_ASSERT_DELTA(u(i, j), v(i, j), delta);
        }
      }
    }

    template<class U, class T>
    void matrix_compare_delta(
        const Matrix<U> &m1, const Matrix<U> &m2,
        const T &delta = ACCEPTABLE_DELTA){
      TEST_ASSERT(m1.rows() == m2.rows());
      TEST_ASSERT(m1.columns() == m2.columns());
      for(unsigned i(0); i < m1.rows(); i++){
        for(unsigned j(0); j < m1.columns(); j++){
          TEST_ASSERT_DELTA(m1(i, j), m2(i, j), delta);
        }
      }
    }

    template<class U, class T>
    void matrix_compare_delta(
        const Matrix<U> &m1, const Matrix<Complex<U> > &m2,
        const T &delta = ACCEPTABLE_DELTA){
      TEST_ASSERT(m1.rows() == m2.rows());
      TEST_ASSERT(m1.columns() == m2.columns());
      for(unsigned i(0); i < m1.rows(); i++){
        for(unsigned j(0); j < m1.columns(); j++){
          TEST_ASSERT_DELTA(m1(i, j), m2(i, j).real(), delta);
        }
      }
    }

    template<class U, class T>
    void matrix_compare_delta(
        const Matrix<Complex<U> > &m1, const Matrix<Complex<U> > &m2,
        const T &delta = ACCEPTABLE_DELTA){
      TEST_ASSERT(m1.rows() == m2.rows());
      TEST_ASSERT(m1.columns() == m2.columns());
      for(unsigned i(0); i < m1.rows(); i++){
        for(unsigned j(0); j < m1.columns(); j++){
          TEST_ASSERT_DELTA(m1(i, j).real(), m2(i, j).real(), delta);
          TEST_ASSERT_DELTA(m1(i, j).imaginary(), m2(i, j).imaginary(), delta);
        }
      }
    }

    template<class U, class T>
    void matrix_compare_delta(
        U (*func)(const unsigned &, const unsigned &), const Matrix<U> &m,
        const T &delta = ACCEPTABLE_DELTA){
      for(unsigned i(0); i < m.rows(); i++){
        for(unsigned j(0); j < m.columns(); j++){
          TEST_ASSERT_DELTA(func(i, j), m(i, j), delta);
        }
      }
    }

    template<class U, class V>
    void matrix_compare(
        const U &u, const V &v){
      matrix_compare_delta(u, v, accuracy(0));
    }

    template<class U>
    void matrix_compare(
        U (*func)(const unsigned &, const unsigned &), const Matrix<U> &m){
      matrix_compare_delta(func, m, accuracy(0));
    }

  private:
    void test_init(){
      matrix_t _A(*A);
      dbg("init:" << _A << endl, false);
      matrix_compare(*A, _A);
    }
    void test_op_equal(){
      matrix_t _A = *A;
      dbg("=:" << _A << endl, false);
      matrix_compare(*A, _A);
    }

    void test_copy(){
      matrix_t _A(A->copy());
      dbg("copy:" << _A << endl, false);
      matrix_compare(*A, _A);
    }

    void test_properties(){
      dbg("rows:" << A->rows() << endl, false);
      TEST_ASSERT(SIZE == A->rows());
      dbg("columns:" << A->columns() << endl, false);
      TEST_ASSERT(SIZE == A->columns());
    }

    void test_getI(){
      matrix_t _A(matrix_t::getI(SIZE));
      dbg("I:" << _A << endl, false);
      matrix_compare(k_delta, _A);
    }

    void test_exchange(){
      struct A1_compared {
        const matrix_t m;
        A1_compared(const matrix_t &_m) : m(_m.copy()) {}
        ~A1_compared(){};
        accuracy operator()(const unsigned &i, const unsigned &j) const {
          switch(i){
            case 0:
              return m(1, j);
            case 1:
              return m(0, j);
            default:
              return m(i, j);
          }
        }
      } a1(*A);
      matrix_t _A1(A->exchangeRows(0, 1));
      dbg("ex_rows:" << _A1 << endl, false);
      matrix_compare(a1, _A1);
      struct A2_compared {
        const matrix_t m;
        A2_compared(const matrix_t &_m) : m(_m.copy()) {}
        ~A2_compared(){};
        accuracy operator()(const unsigned &i, const unsigned &j) const {
          switch(j){
            case 0:
              return m(i, 1);
            case 1:
              return m(i, 0);
            default:
              return m(i, j);
          }
        }
      } a2(*A);
      matrix_t _A2(A->exchangeColumns(0, 1));
      dbg("ex_columns:" << _A2 << endl, false);
      matrix_compare(a2, _A2);
    }

    void test_check(){
      dbg("square?:" << A->isSquare() << endl, false);
      TEST_ASSERT(true == A->isSquare());
      dbg("sym?:" << A->isSymmetric() << endl, false);
      TEST_ASSERT(true == A->isSymmetric());
    }

    void test_scalar_op(){
      struct A1_compared {
        const matrix_t &m;
        A1_compared(const matrix_t &_m) : m(_m) {}
        ~A1_compared(){};
        accuracy operator()(const unsigned &i, const unsigned &j) const {
          return m(i, j) * 2;
        }
      } a1(*A);
      struct A2_compared {
        const matrix_t &m;
        A2_compared(const matrix_t &_m) : m(_m) {}
        ~A2_compared(){};
        accuracy operator()(const unsigned &i, const unsigned &j) const {
          return m(i, j) / 2;
        }
      } a2(*A);
      struct A3_compared {
        const matrix_t &m;
        A3_compared(const matrix_t &_m) : m(_m) {}
        ~A3_compared(){};
        accuracy operator()(const unsigned &i, const unsigned &j) const {
          return -m(i, j);
        }
      } a3(*A);
      matrix_t _A1((*A) * 2.);
      matrix_t _A2((*A) / 2.);
      matrix_t _A3(-(*A));
      dbg("*:" << _A1 << endl, false);
      dbg("/:" << _A2 << endl, false);
      dbg("-():" << _A3 << endl, false);
      matrix_compare(a1, _A1);
      matrix_compare(a2, _A2);
      matrix_compare(a3, _A3);
    }

    void test_matrix_op(){
      {
        struct A_compared {
          const matrix_t &m1, &m2;
          A_compared(const matrix_t &_m1, const matrix_t &_m2) : m1(_m1), m2(_m2) {}
          ~A_compared(){};
          accuracy operator()(const unsigned &i, const unsigned &j) const {
            return (m1)(i, j) + (m2)(i, j);
          }
        } a(*A, *B);
        matrix_t _A((*A) + (*B));
        dbg("+:" << _A << endl, false);
        matrix_compare_delta(a, _A, ACCEPTABLE_DELTA);
      }
      {
        struct A_compared {
          matrix_t &m1, &m2;
          A_compared(matrix_t &_m1, matrix_t &_m2) : m1(_m1), m2(_m2) {}
          ~A_compared(){};
          accuracy operator()(unsigned i, unsigned j) const {
            accuracy sum(0);
            for(unsigned k(0); k < m1.rows(); k++){
              sum += m1(i, k) * m2(k, j);
            }
            return sum;
          }
        } a(*A, *B);
        matrix_t _A((*A) * (*B));
        dbg("*:" << _A << endl, false);
        matrix_compare_delta(a, _A, ACCEPTABLE_DELTA);
      }
      {
        matrix_t _A(A->inverse());
        dbg("inv:" << _A << endl, false);
        matrix_compare_delta(matrix_t::getI(SIZE), (*A) * _A, ACCEPTABLE_DELTA);
      }
      {
        struct A_compared {
          const matrix_t &m;
          A_compared(const matrix_t &_m) : m(_m) {}
          ~A_compared(){};
          accuracy operator()(const unsigned &i, const unsigned &j) const {
            return m(j, i);
          }
        } a(*A);
        matrix_t _A(A->transpose());
        dbg("trans:" << _A << endl, false);
        matrix_compare(a, _A);
      }
      {
        struct A_compared {
          const matrix_t &m;
          A_compared(const matrix_t &_m) : m(_m) {}
          ~A_compared(){};
          accuracy operator()(const unsigned &i, const unsigned &j) const {
            return m(i+1, j+1);
          }
        } a(*A);
        matrix_t _A(A->coMatrix(0, 0));
        dbg("coMatrix:" << _A << endl, false);
        matrix_compare(a, _A);
      }
      dbg("det:" << A->determinant() << endl, false);
      {
        struct A_compared {
          matrix_t m1, m2;
          A_compared(const matrix_t &_m1, const matrix_t &_m2)
              : m1(_m1.copy()), m2(_m2.copy()) {}
          ~A_compared(){};
          accuracy operator()(const unsigned &i, const unsigned &j) const {
            return m1(i, j) + m2(i, j);
          }
        } a(*A, *B);
        matrix_t _A(A->pivotMerge(0, 0, *B));
        dbg("pivotMerge:" << _A << endl, false);
        matrix_compare_delta(a, _A, ACCEPTABLE_DELTA);
      }
      {
        struct A_compared {
          matrix_t m1, m2;
          A_compared(const matrix_t &_m1, const matrix_t &_m2)
              : m1(_m1.copy()), m2(_m2.copy()) {}
          ~A_compared(){};
          accuracy operator()(const unsigned &i, const unsigned &j) const {
            //std::cerr << i << "," << j << ": " << (m1(i, j) + m2(i, j)) << std::endl;
            return m1(i, j) + m2(i, j);
          }
        } a(*A, *B);
        matrix_t _A(A->pivotAdd(0, 0, *B));
        dbg("pivotAdd:" << _A << endl, false);
        matrix_compare_delta(a, _A, ACCEPTABLE_DELTA);
      }
      dbg("eigen22" << *A << endl, false);

      try{
        cmatrix_t A_copy(A->rows(), A->columns());
        for(unsigned i(0); i < A_copy.rows(); i++){
          for(unsigned j(0); j < A_copy.columns(); j++){
            A_copy(i, j).real() = (*A)(i, j);
          }
        }
        cmatrix_t _A(A->eigen());
        dbg("eigen:" << _A << endl, false);
        for(unsigned i(0); i < A->rows(); i++){
          matrix_compare_delta(A_copy * _A.partial(A->rows(), 1, 0, i),
              _A.partial(A->rows(), 1, 0, i) * _A(i, A->rows()), ACCEPTABLE_DELTA);
        }
      }catch(exception &e){
        dbg("eigen_error:" << e.what() << endl, true);
      }

      /*{
        cmatrix_t _A(A->sqrt());
        dbg("sqrt:" << _A << endl, false);
        matrix_compare_delta(*A, _A * _A, ACCEPTABLE_DELTA);
      }*/
    }

    void test_matrix_op_loop(){
      for(unsigned i(0); i < 300; i++){
        if(i % 100 == 0){
          cout << "loop(" << i << ") ..." << endl;
        }
        try{
          test_matrix_op();
        }catch(exception &e){
          cout << e.what() << endl;
        }
      }
    }

#if 0
    void test_partialMatrix(){
      PartialMatrix<accuracy> _A(A->partial(3, 3, 1, 1));

      dbg("A:" << *A << endl, false);
      dbg("_A:" << _A << endl, false);

      dbg("rows:" << _A.rows() << endl, false);
      dbg("columns:" << _A.columns() << endl, false);

      dbg("_A.copy():" << _A.copy() << endl, false);
      dbg("(*_A)^{-1}:" << _A.inverse() << endl, false);
      dbg("_A.eigen()" << _A.eigen() << endl, false);
      dbg("_A.rowVector():" << _A.rowVector(0) << endl, false);
      dbg("_A.columnVector():" << _A.columnVector(0) << endl, false);

      dbg("A:" << *A << endl, false);
      dbg("_A:" << _A << endl, false);
      _A = A->partial(3, 3, 2, 2);
      dbg("_A => A:" << *A << endl, false);

      dbg("A:" << *A << endl, false);
      Matrix<accuracy> __A(A->partial(3, 3, 1, 1));
      dbg("__A:" << __A << endl, false);
      __A = A->partial(3, 3, 3, 3);
      dbg("__A => A:" << *A << endl, false);

      dbg("A:" << *A << endl, false);
      PartialMatrix<accuracy> ___A_(A->partial(3, 3, 1, 1));
      PartialMatrix<accuracy> &___A(___A_);
      dbg("___A:" << ___A << endl, false);
      ___A = A->partial(3, 3, 4, 4);
      dbg("___A => A:" << *A << endl, false);


      dbg("A:" << *A << endl, false);
      PartialMatrix<accuracy> ____A_(A->partial(3, 3, 1, 1));
      Matrix<accuracy> &____A(____A_);
      dbg("____A:" << ____A << endl, false);
      ____A = A->partial(3, 3, 5, 5);
      dbg("____A => A:" << *A << endl, false);
    }
#endif

    void test_decompose(){

      {
        Matrix<accuracy> LU(A->decomposeLU());
        Matrix<accuracy>
            L(LU.partial(LU.rows(), LU.rows(), 0, 0)),
            U(LU.partial(LU.rows(), LU.rows(), 0, LU.rows()));
        dbg("LU(L):" << L << endl, false);
        dbg("LU(U):" << U << endl, false);

        for(unsigned i(0); i < A->rows(); i++){
          for(unsigned j(i+1); j < A->columns(); j++){
            TEST_ASSERT(L(i, j) == 0);
            TEST_ASSERT(U(j, i) == 0);
          }
        }

        Matrix<accuracy> _A(L * U);
        dbg("LU(L) * LU(U):" << _A << endl, false);
        // GSLのLU分解では行の入れ替えが行われている
        set<unsigned> unused_rows;
        for(unsigned i(0); i < A->rows(); i++){
          unused_rows.insert(i);
        }
        for(unsigned i(0); i < A->rows(); i++){
          for(set<unsigned>::iterator it(unused_rows.begin());
              ;
              ++it){
            if(it == unused_rows.end()){TEST_FAIL("L * U != A");}
            //cout << *it << endl;
            bool matched(true);
            for(unsigned j(0); j < A->columns(); j++){
              accuracy delta((*A)(i, j) - _A(*it, j));
              //cout << delta << endl;
              if(delta < 0){delta *= -1;}
              if(delta > ACCEPTABLE_DELTA){matched = false;}
            }
            if(matched){
              //cout << "matched: " << *it << endl;
              unused_rows.erase(it);
              break;
            }
          }
        }
      }

      {
        matrix_t U(matrix_t::getI(A->rows()));
        matrix_t H(A->hessenberg(&U));

        dbg("hessen(H):" << H << endl, false);
        for(unsigned i(2); i < H.rows(); i++){
          for(unsigned j(0); j < (i - 1); j++){
            TEST_ASSERT(H(i, j) == 0);
          }
        }
        matrix_t _A(U * H * U.transpose());
        dbg("U * H * U^{T}:" << _A << endl, false);
        matrix_compare_delta(*A, _A, ACCEPTABLE_DELTA);
      }

      {

        Matrix<accuracy> UD(A->decomposeUD());
        Matrix<accuracy>
            U(UD.partial(UD.rows(), UD.rows(), 0, 0)),
            D(UD.partial(UD.rows(), UD.rows(), 0, UD.rows()));
        dbg("UD(U):" << U << endl, false);
        dbg("UD(D):" << D << endl, false);

        for(unsigned i(0); i < A->rows(); i++){
          for(unsigned j(i+1); j < A->columns(); j++){
            TEST_ASSERT(U(j, i) == 0);
            TEST_ASSERT(D(i, j) == 0);
            TEST_ASSERT(D(j, i) == 0);
          }
        }

        Matrix<accuracy> _A(U * D * U.transpose());
        dbg("U * D * U^{T}:" << _A << endl, false);
        matrix_compare_delta(*A, _A, ACCEPTABLE_DELTA);
      }
    }

#if 0
    template <class FloatT>
    void mat_mul(FloatT *x, const int r1, const int c1,
        FloatT *y, const int c2,
        FloatT *r,
        bool x_trans = false, bool y_trans = false){
      int indx_c, indy_c; // 列方向への移動
      int indx_r, indy_r; // 行方向への移動
      if(x_trans){
        indx_c = r1;
        indx_r = 1;
      }else{
        indx_c = 1;
        indx_r = c1;
      }
      if(y_trans){
        indy_c = c1;
        indy_r = 1;
      }else{
        indy_c = 1;
        indy_r = c2;
      }
      int indx, indy, indr(0);
      for(unsigned int i(0); i < r1; i++){
        for(unsigned int j(0); j < c2; j++){
          indx = i * indx_r;
          indy = j * indy_c;
          r[indr] = FloatT(0);
          for(unsigned int k(0); k < c1; k++){
            r[indr] += x[indx] * y[indy];
            indx += indx_c;
            indy += indy_r;
          }
          indr++;
        }
      }
    }

    template <class FloatT>
    void mat_mul_unrolled(FloatT *x, const int r1, const int c1,
        FloatT *y, const int c2,
        FloatT *r,
        bool x_trans = false, bool y_trans = false){
      if((r1 > 1) && (c1 > 1) && (c2 > 1)
          && (r1 % 2 == 0) && (c1 % 2 == 0) && (c2 % 2 == 0)){
        return mat_mul(x, r1, c1, y, c2, r, x_trans, y_trans);
      }
      int indx_c, indy_c; // 列方向への移動
      int indx_r, indy_r; // 行方向への移動
      if(x_trans){
        indx_c = r1;
        indx_r = 1;
      }else{
        indx_c = 1;
        indx_r = c1;
      }
      if(y_trans){
        indy_c = c1;
        indy_r = 1;
      }else{
        indy_c = 1;
        indy_r = c2;
      }
      int indx, indy, indr(0);
      // ループ展開バージョン
      for(int i(0); i < r1; i += 2){
        for(int j(0); j < c2; j += 2){
          indx = i * indx_r;
          indy = j * indy_c;
          FloatT sum00(0), sum01(0), sum10(0), sum11(0);
          for(int k(c1); k > 0; k -= 2){
            sum00 += x[indx]                   * y[indy];
            sum01 += x[indx]                   * y[indy + indy_c];
            sum00 += x[indx + indx_c]          * y[indy + indy_r];
            sum01 += x[indx + indx_c]          * y[indy + indy_c + indy_r];
            sum10 += x[indx + indx_r]          * y[indy];
            sum11 += x[indx + indx_r]          * y[indy + indy_c];
            sum10 += x[indx + indx_c + indx_r] * y[indy + indy_r];
            sum11 += x[indx + indx_c + indx_r] * y[indy + indy_c + indy_r];
            indx += (indx_c * 2);
            indy += (indy_r * 2);
          }
          r[indr] = sum00;
          r[indr + 1] = sum01;
          r[indr + c2] = sum10;
          r[indr + c2 + 1] = sum11;
          indr += 2;
        }
        indr += c2;
      }
    }

    void test_unrolled_product(){
      // 非対称行列化
      for(int i(0); i < A->rows(); i++){
        for(int j(i); j < A->columns(); j++){
          A_array[i][j] = (*A)(i, j) = rand_regularized(0, 1);
        }
      }
      for(int i(0); i < B->rows(); i++){
        for(int j(i); j < B->columns(); j++){
          B_array[i][j] = (*B)(i, j) = rand_regularized(0, 1);
        }
      }
      accuracy *AB_array(new accuracy[A->rows() * B->columns()]);
      {
        // 非転置 * 非転置
        mat_mul_unrolled((accuracy *)A_array, A->rows(), A->columns(),
            (accuracy *)B_array, B->columns(),
            (accuracy *)AB_array);
        matrix_t AB((*A) * (*B));
        accuracy *AB_array_target((accuracy *)AB_array);
        for(int i(0); i < AB.rows(); i++){
          for(int j(0); j < AB.columns(); j++){
            //cerr << AB(i, j) << "," << *AB_array_target << endl;
            TEST_ASSERT_DELTA(AB(i, j), *AB_array_target, accuracy(0));
            AB_array_target++;
          }
        }
      }
      {
        // 非転置 * 転置
        mat_mul_unrolled((accuracy *)A_array, A->rows(), A->columns(),
            (accuracy *)B_array, B->columns(),
            (accuracy *)AB_array, false, true);
        matrix_t AB((*A) * B->transpose());
        accuracy *AB_array_target((accuracy *)AB_array);
        for(int i(0); i < AB.rows(); i++){
          for(int j(0); j < AB.columns(); j++){
            //cerr << AB(i, j) << "," << *AB_array_target << endl;
            TEST_ASSERT_DELTA(AB(i, j), *AB_array_target, accuracy(0));
            AB_array_target++;
          }
        }
      }
      {
        // 転置 * 非転置
        mat_mul_unrolled((accuracy *)A_array, A->rows(), A->columns(),
            (accuracy *)B_array, B->columns(),
            (accuracy *)AB_array, true, false);
        matrix_t AB(A->transpose() * (*B));
        accuracy *AB_array_target((accuracy *)AB_array);
        for(int i(0); i < AB.rows(); i++){
          for(int j(0); j < AB.columns(); j++){
            //cerr << AB(i, j) << "," << *AB_array_target << endl;
            TEST_ASSERT_DELTA(AB(i, j), *AB_array_target, accuracy(0));
            AB_array_target++;
          }
        }
      }
      {
        // 転置 * 転置
        mat_mul_unrolled((accuracy *)A_array, A->rows(), A->columns(),
            (accuracy *)B_array, B->columns(),
            (accuracy *)AB_array, true, true);
        matrix_t AB(A->transpose() * B->transpose());
        accuracy *AB_array_target((accuracy *)AB_array);
        for(int i(0); i < AB.rows(); i++){
          for(int j(0); j < AB.columns(); j++){
            //cerr << AB(i, j) << "," << *AB_array_target << endl;
            TEST_ASSERT_DELTA(AB(i, j), *AB_array_target, accuracy(0));
            AB_array_target++;
          }
        }
      }
      delete [] AB_array;
    }
#endif
};

bool run_tests(){
  MatrixTestSuite test_suits;
  Test::TextOutput output(Test::TextOutput::Verbose);
  return test_suits.run(output, false); // Note the 'false' parameter
}


int main(){
  run_tests();
  return 0;
}
