#include <string>
#include <cstring>
#include <cmath>
#include <ctime>
#include <iostream>
#include <exception>
#include <set>

#include "param/complex.h"
#include "param/matrix.h"

#include <cpptest.h>
#include <cpptest-suite.h>

#include <boost/random.hpp>
#include <boost/random/random_device.hpp>

#define SIZE 8
#define accuracy double
#define ACCEPTABLE_DELTA 1E-10

using namespace std;

struct rand_t {
  boost::random::mt19937 gen;
  boost::random::normal_distribution<> dist;
  rand_t()
      : gen(static_cast<unsigned long>(time(0))), dist(0, 1.0){
  }
  accuracy operator()(){
    return (accuracy)dist(gen);
  }
} gen_rand;


#ifndef DEBUG_PRINT
#define DEBUG_PRINT false
#endif
#define dbg(exp, force) \
if((force) || DEBUG_PRINT){cerr << endl << exp;} \
else{ostream(NULL) << exp;}

accuracy k_delta(const unsigned &i, const unsigned &j){
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
      TEST_ADD(MatrixTestSuite::test_exchange_row);
      TEST_ADD(MatrixTestSuite::test_exchange_column);
      TEST_ADD(MatrixTestSuite::test_check_square);
      TEST_ADD(MatrixTestSuite::test_check_symmetric);
      TEST_ADD(MatrixTestSuite::test_scalar_mul);
      TEST_ADD(MatrixTestSuite::test_scalar_div);
      TEST_ADD(MatrixTestSuite::test_scalar_minus);
      TEST_ADD(MatrixTestSuite::test_matrix_add);
      TEST_ADD(MatrixTestSuite::test_matrix_mul);
      TEST_ADD(MatrixTestSuite::test_inv);
      TEST_ADD(MatrixTestSuite::test_trans);
      TEST_ADD(MatrixTestSuite::test_det);
      TEST_ADD(MatrixTestSuite::test_pivot_merge);
      TEST_ADD(MatrixTestSuite::test_pivot_add);
      TEST_ADD(MatrixTestSuite::test_eigen);
      TEST_ADD(MatrixTestSuite::test_sqrt);
      TEST_ADD(MatrixTestSuite::test_partial);
      TEST_ADD(MatrixTestSuite::test_LU);
      TEST_ADD(MatrixTestSuite::test_UH);
      TEST_ADD(MatrixTestSuite::test_UD);
      //TEST_ADD(MatrixTestSuite::test_matrix_op_loop);
    }

  protected:
    matrix_t *A, *B;
    accuracy A_array[SIZE][SIZE], B_array[SIZE][SIZE];

    virtual void setup(){
      dbg(endl, false);
      A = new matrix_t(SIZE, SIZE);
      for(int i = 0; i < A->rows(); i++){
        A_array[i][i] = (*A)(i, i) = gen_rand();
        for(int j = i + 1; j < A->columns(); j++){
          A_array[i][j] = A_array[j][i]
              = (*A)(i, j) = (*A)(j, i) = gen_rand();
        }
      }
      B = new matrix_t(SIZE, SIZE);
      for(int i = 0; i < B->rows(); i++){
        B_array[i][i] = (*B)(i, i) = gen_rand();
        for(int j = i + 1; j < B->columns(); j++){
          B_array[i][j] = B_array[j][i]
              = (*B)(i, j) = (*B)(j, i) = gen_rand();
        }
      }
      dbg("A:" << *A << endl, false);
      dbg("B:" << *B << endl, false);
    }

    virtual void tear_down(){
      delete A;
      delete B;
    }

    template<class T1, class T2, class T3>
    void element_compare_delta(
        const T1 &v1, const T2 &v2,
        const T3 &delta){
      TEST_ASSERT_DELTA(v1, v2, delta);
    }

    template<class T1, class T2, class T3>
    void element_compare_delta(
        const T1 &v1, const Complex<T2> &v2,
        const T3 &delta){
      TEST_ASSERT_DELTA(v1, v2.real(), delta);
    }

    template<class T1, class T2, class T3>
    void element_compare_delta(
        const Complex<T1> &v1, const Complex<T2> &v2,
        const T3 &delta){
      TEST_ASSERT_DELTA(v1.real(), v2.real(), delta);
      TEST_ASSERT_DELTA(v1.imaginary(), v2.imaginary(), delta);
    }

    template<
        class U,
        class T2, template <class> class Array2D_Type2, class ViewType2,
        class T3>
    void matrix_compare_delta(
        const U &u,
        const Matrix<T2, Array2D_Type2, ViewType2> &m,
        const T3 &delta){
      for(unsigned i(0); i < m.rows(); i++){
        for(unsigned j(0); j < m.columns(); j++){
          element_compare_delta(u(i, j), m(i, j), delta);
        }
      }
    }

    template<
        class T1, template <class> class Array2D_Type1, class ViewType1,
        class T2, template <class> class Array2D_Type2, class ViewType2,
        class T3>
    void matrix_compare_delta(
        const Matrix<T1, Array2D_Type1, ViewType1> &m1,
        const Matrix<T2, Array2D_Type2, ViewType2> &m2,
        const T3 &delta){
      TEST_ASSERT(m1.rows() == m2.rows());
      TEST_ASSERT(m1.columns() == m2.columns());
      matrix_compare_delta<
          Matrix<T1, Array2D_Type1, ViewType1>,
          T2, Array2D_Type2, ViewType2,
          T3>(m1, m2, delta);
    }

    template<class U, class V>
    void matrix_compare(
        const U &u, const V &v){
      matrix_compare_delta(u, v, accuracy(0));
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

    void test_exchange_row(){
      struct {
        const matrix_t m;
        const accuracy &operator()(const unsigned &i, const unsigned &j) const {
          return m((i == 0 ? 1 : (i == 1 ? 0 : i)), j);
        }
      } a1 = {A->copy()};
      matrix_t _A1(A->exchangeRows(0, 1));
      dbg("ex_rows:" << _A1 << endl, false);
      matrix_compare(a1, _A1);
    }
    void test_exchange_column(){
      struct A2_compared {
        const matrix_t m;
        const accuracy &operator()(const unsigned &i, const unsigned &j) const {
          return m(i, (j == 0 ? 1 : (j == 1 ? 0 : j)));
        }
      } a2 = {A->copy()};
      matrix_t _A2(A->exchangeColumns(0, 1));
      dbg("ex_columns:" << _A2 << endl, false);
      matrix_compare(a2, _A2);
    }

    void test_check_square(){
      dbg("square?:" << A->isSquare() << endl, false);
      TEST_ASSERT(true == A->isSquare());
    }
    void test_check_symmetric(){
      dbg("sym?:" << A->isSymmetric() << endl, false);
      TEST_ASSERT(true == A->isSymmetric());
    }

    void test_scalar_mul(){
      struct {
        const matrix_t &m;
        accuracy operator()(const unsigned &i, const unsigned &j) const {
          return m(i, j) * 2;
        }
      } a1 = {*A};
      matrix_t _A1((*A) * 2.);
      dbg("*:" << _A1 << endl, false);
      matrix_compare(a1, _A1);
    }
    void test_scalar_div(){
      struct {
        const matrix_t &m;
        accuracy operator()(const unsigned &i, const unsigned &j) const {
          return m(i, j) / 2;
        }
      } a2 = {*A};
      matrix_t _A2((*A) / 2.);
      dbg("/:" << _A2 << endl, false);
      matrix_compare(a2, _A2);
    }
    void test_scalar_minus(){
      struct A3_compared {
        const matrix_t &m;
        accuracy operator()(const unsigned &i, const unsigned &j) const {
          return -m(i, j);
        }
      } a3 = {*A};
      matrix_t _A3(-(*A));
      dbg("-():" << _A3 << endl, false);
      matrix_compare(a3, _A3);
    }

    void test_matrix_add(){
      struct {
        const matrix_t &m1, &m2;
        accuracy operator()(const unsigned &i, const unsigned &j) const {
          return (m1)(i, j) + (m2)(i, j);
        }
      } a = {*A, *B};
      matrix_t _A((*A) + (*B));
      dbg("+:" << _A << endl, false);
      matrix_compare_delta(a, _A, ACCEPTABLE_DELTA);
    }
    void test_matrix_mul(){
      struct {
        matrix_t &m1, &m2;
        accuracy operator()(const unsigned &i, const unsigned &j) const {
          accuracy sum(0);
          for(unsigned k(0); k < m1.rows(); k++){
            sum += m1(i, k) * m2(k, j);
          }
          return sum;
        }
      } a = {*A, *B};
      matrix_t _A((*A) * (*B));
      dbg("*:" << _A << endl, false);
      matrix_compare_delta(a, _A, ACCEPTABLE_DELTA);
    }
    void test_inv(){
      matrix_t _A(A->inverse());
      dbg("inv:" << _A << endl, false);
      matrix_compare_delta(matrix_t::getI(SIZE), (*A) * _A, ACCEPTABLE_DELTA);
    }
    void test_trans(){
      struct {
        const matrix_t &m;
        const accuracy &operator()(const unsigned &i, const unsigned &j) const {
          return m(j, i);
        }
      } a = {*A};
      matrix_t::transposed_t _A(A->transpose());
      dbg("trans:" << _A << endl, false);
      matrix_compare(a, _A);
      matrix_t __A(_A.copy());
      dbg("trans.copy:" << __A << endl, false);
      matrix_compare(a, __A);
      matrix_t ___A(_A.transpose());
      dbg("trans.trans:" << ___A << endl, false);
      matrix_compare(*A, ___A);
    }
    void test_det(){
      struct {
        const matrix_t &m;
        const accuracy &operator()(const unsigned &i, const unsigned &j) const {
          return m(i+1, j+1);
        }
      } a = {*A};
      matrix_t _A(A->matrix_for_minor(0, 0));
      dbg("matrix_for_minor:" << _A << endl, false);
      matrix_compare(a, _A);
      dbg("det:" << A->determinant() << endl, false);
    }

    void test_pivot_merge(){
      struct {
        matrix_t m1, m2;
        accuracy operator()(const unsigned &i, const unsigned &j) const {
          return m1(i, j) + m2(i, j);
        }
      } a = {A->copy(), B->copy()};
      matrix_t _A(A->pivotMerge(0, 0, *B));
      dbg("pivotMerge:" << _A << endl, false);
      matrix_compare_delta(a, _A, ACCEPTABLE_DELTA);
    }
    void test_pivot_add(){
      struct {
        const matrix_t &m1, &m2;
        accuracy operator()(const unsigned &i, const unsigned &j) const {
          //std::cerr << i << "," << j << ": " << (m1(i, j) + m2(i, j)) << std::endl;
          return m1(i, j) + m2(i, j);
        }
      } a = {*A, *B};
      matrix_t _A(A->pivotAdd(0, 0, *B));
      dbg("pivotAdd:" << _A << endl, false);
      matrix_compare_delta(a, _A, ACCEPTABLE_DELTA);
    }

    void test_eigen(){
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
          //dbg("eigen(" << i << "):" << A_copy * _A.partial(A->rows(), 1, 0, i) << endl, false);
          //dbg("eigen(" << i << "):" << _A.partial(A->rows(), 1, 0, i) * _A(i, A->rows()) << endl, false);
          matrix_compare_delta(A_copy * _A.partial(A->rows(), 1, 0, i),
              _A.partial(A->rows(), 1, 0, i) * _A(i, A->rows()), ACCEPTABLE_DELTA);
        }
      }catch(exception &e){
        dbg("eigen_error:" << e.what() << endl, true);
      }
    }

    void test_sqrt(){
      cmatrix_t _A(A->sqrt());
      dbg("sqrt:" << _A << endl, false);
      matrix_compare_delta(*A, _A * _A, ACCEPTABLE_DELTA);
    }

#if 0
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
#endif

    void test_partial(){
      struct {
        const matrix_t &m;
        unsigned int i_offset, j_offset;
        const accuracy &operator()(const unsigned &i, const unsigned &j) const {
          return m(i + i_offset, j + j_offset);
        }
      } a = {*A, 0, 0};

      a.i_offset = a.j_offset = 1;
      matrix_t::partial_t _A(A->partial(3, 3, a.i_offset, a.j_offset));
      dbg("_A:" << _A << endl, false);
      matrix_compare(a, _A);

      dbg("_A.copy():" << _A.copy() << endl, false);
      dbg("(*_A)^{-1}:" << _A.inverse() << endl, false);
      dbg("_A.eigen()" << _A.eigen() << endl, false);
      dbg("_A.rowVector():" << _A.rowVector(0) << endl, false);
      dbg("_A.columnVector():" << _A.columnVector(0) << endl, false);

      a.i_offset = a.j_offset = 2;
      _A = A->partial(3, 3, a.i_offset, a.j_offset);
      dbg("_A:" << _A << endl, false);
      matrix_compare(a, _A);
    }

    void test_LU(){
      matrix_t LU(A->decomposeLU());
      matrix_t::partial_t
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

      matrix_t _A(L * U);
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

    void test_UH(){
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

    void test_UD(){
      matrix_t UD(A->decomposeUD());
      matrix_t::partial_t
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

      dbg("UD(U):" << U.transpose() << endl, false);
      matrix_t _A(U * D * U.transpose());
      dbg("U * D * U^{T}:" << _A << endl, false);
      matrix_compare_delta(*A, _A, ACCEPTABLE_DELTA);
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
          A_array[i][j] = (*A)(i, j) = gen_rand();
        }
      }
      for(int i(0); i < B->rows(); i++){
        for(int j(i); j < B->columns(); j++){
          B_array[i][j] = (*B)(i, j) = gen_rand();
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
