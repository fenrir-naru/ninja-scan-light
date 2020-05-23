#include <string>
#include <cstring>
#include <cmath>
#include <ctime>
#include <iostream>
#include <stdexcept>
#include <set>
#include <deque>
#include <algorithm>

#include "param/complex.h"
#include "param/matrix.h"
#include "param/matrix_fixed.h"

#include <boost/random.hpp>
#include <boost/random/random_device.hpp>

#include <boost/type_traits/is_same.hpp>

#include <boost/format.hpp>

#define BOOST_TEST_MAIN
#include <boost/test/included/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/output_test_stream.hpp>

#if !defined(BOOST_VERSION)
#define BOOST_FIXTURE_TEST_SUITE(name, fixture)
#define BOOST_AUTO_TEST_SUITE_END()
#define BOOST_AUTO_TEST_CASE(name) void name()
#define BOOST_AUTO_TEST_CASE_MAY_FAILURES(name, failures) void name()
#elif defined(BOOST_VERSION) && (BOOST_VERSION < 105900)
#define BOOST_AUTO_TEST_CASE_MAY_FAILURES(name, failures) \
    BOOST_AUTO_TEST_CASE_EXPECTED_FAILURES(name, failures) \
    BOOST_AUTO_TEST_CASE(name)
#else
#define BOOST_AUTO_TEST_CASE_MAY_FAILURES(name, failures) \
    BOOST_AUTO_TEST_CASE(name, * boost::unit_test::expected_failures(failures))
#endif

#define SIZE 8
#define ACCEPTABLE_DELTA_DEFAULT 1E-8

using namespace std;

namespace br = boost::random;
struct rand_t {
  br::mt19937 gen;
  br::normal_distribution<> dist;
  rand_t()
      : gen(static_cast<unsigned long>(time(0))), dist(0, 1.0){
  }
  rand_t(const unsigned long &seed)
      : gen(seed), dist(0, 1.0){
  }
  double operator()(){
    return (double)dist(gen);
  }
};

int k_delta(const unsigned &i, const unsigned &j){
  return (i == j) ? 1 : 0;
}

typedef double content_t;
typedef Matrix<content_t> matrix_t;
typedef Matrix<Complex<content_t> > cmatrix_t;

struct Fixture {
  rand_t gen_rand;
  matrix_t *A, *B;
  content_t A_array[SIZE][SIZE], B_array[SIZE][SIZE];

  void assign_symmetric(){
    for(unsigned int i(0); i < A->rows(); i++){
      A_array[i][i] = (*A)(i, i) = gen_rand();
      for(unsigned int j(i + 1); j < A->columns(); j++){
        A_array[i][j] = A_array[j][i]
            = (*A)(i, j) = (*A)(j, i) = gen_rand();
      }
    }
    for(unsigned int i(0); i < B->rows(); i++){
      B_array[i][i] = (*B)(i, i) = gen_rand();
      for(unsigned int j(i + 1); j < B->columns(); j++){
        B_array[i][j] = B_array[j][i]
            = (*B)(i, j) = (*B)(j, i) = gen_rand();
      }
    }
  }
  void assign_unsymmetric(){
    for(unsigned int i(0); i < A->rows(); i++){
      for(unsigned int j(i); j < A->columns(); j++){
        A_array[i][j] = (*A)(i, j) = gen_rand();
      }
    }
    for(unsigned int i(0); i < B->rows(); i++){
      for(unsigned int j(i); j < B->columns(); j++){
        B_array[i][j] = (*B)(i, j) = gen_rand();
      }
    }
  }
  void assign_intermediate_zeros(const unsigned &row = 1){
    if(row > A->rows()){return;}
    for(unsigned int j(0); j < row; j++){
      A_array[row][j] = (*A)(row, j) = content_t(0);
    }
    for(unsigned int j(0); j < row; j++){
      B_array[row][j] = (*B)(row, j) = content_t(0);
    }
  }
  void assign_linear(){
    int counter(0);
    for(unsigned int i(0); i < A->rows(); i++){
      for(unsigned int j(0); j < A->columns(); j++){
        A_array[i][j] = (*A)(i, j) = counter++;
      }
    }
    for(unsigned int i(0); i < B->rows(); i++){
      for(unsigned int j(0); j < B->columns(); j++){
        B_array[i][j] = (*B)(i, j) = counter++;
      }
    }
  }
  void prologue_print(){
    BOOST_TEST_MESSAGE("A:" << *A);
    BOOST_TEST_MESSAGE("B:" << *B);
  }

  Fixture()
      : gen_rand(), A(new matrix_t(SIZE, SIZE)), B(new matrix_t(SIZE, SIZE)){
    assign_symmetric();
  }

  ~Fixture(){
    delete A;
    delete B;
  }
};

template<class T1, class T2, class T3>
void element_compare_delta(
    T1 v1, T2 v2,
    T3 delta){
  BOOST_CHECK_SMALL(v1 - v2, delta);
}
template<class T1, class T2>
void element_compare_delta(
    T1 v1, T2 v2,
    const bool &){
  BOOST_CHECK_EQUAL(v1, v2);
}

template<class T1, class T2, class T3>
void element_compare_delta(
    T1 v1, const Complex<T2> &v2,
    T3 delta){
  BOOST_CHECK_SMALL(v1 - v2.real(), delta);
  BOOST_CHECK_SMALL(0 - v2.imaginary(), delta);
}
template<class T1, class T2>
void element_compare_delta(
    T1 v1, const Complex<T2> &v2,
    const bool &){
  BOOST_CHECK_EQUAL(v1, v2.real());
  BOOST_CHECK_EQUAL(0, v2.imaginary());
}

template<class T1, class T2, class T3>
void element_compare_delta(
    const Complex<T1> &v1, const Complex<T2> &v2,
    T3 delta){
  BOOST_CHECK_SMALL(v1.real() - v2.real(), delta);
  BOOST_CHECK_SMALL(v1.imaginary() - v2.imaginary(), delta);
}
template<class T1, class T2>
void element_compare_delta(
    const Complex<T1> &v1, const Complex<T2> &v2,
    const bool &){
  BOOST_CHECK_EQUAL(v1.real(), v2.real());
  BOOST_CHECK_EQUAL(v1.imaginary(), v2.imaginary());
}

template<
    class U,
    class T2, class Array2D_Type2, class ViewType2,
    class T3>
void matrix_compare_delta(
    U u,
    const Matrix_Frozen<T2, Array2D_Type2, ViewType2> &m,
    const T3 &delta = ACCEPTABLE_DELTA_DEFAULT){
  for(unsigned i(0); i < m.rows(); i++){
    for(unsigned j(0); j < m.columns(); j++){
      element_compare_delta(u(i, j), m(i, j), delta);
    }
  }
}

template<
    class T1, class Array2D_Type1, class ViewType1,
    class T2, class Array2D_Type2, class ViewType2,
    class T3>
void matrix_compare_delta(
    const Matrix_Frozen<T1, Array2D_Type1, ViewType1> &m1,
    const Matrix_Frozen<T2, Array2D_Type2, ViewType2> &m2,
    const T3 &delta = ACCEPTABLE_DELTA_DEFAULT){
  BOOST_REQUIRE_EQUAL(m1.rows(), m2.rows());
  BOOST_REQUIRE_EQUAL(m1.columns(), m2.columns());
  for(unsigned i(0); i < m1.rows(); i++){
    for(unsigned j(0); j < m1.columns(); j++){
      element_compare_delta(m1(i, j), m2(i, j), delta);
    }
  }
}

template<
    class T1,
    class Array2D_Type2, class ViewType2,
    class T3>
void matrix_compare_delta(
    T1 *t,
    const Matrix_Frozen<T1, Array2D_Type2, ViewType2> &m,
    const T3 &delta = ACCEPTABLE_DELTA_DEFAULT){
  for(unsigned i(0); i < m.rows(); i++){
    for(unsigned j(0); j < m.columns(); j++){
      element_compare_delta(*(t++), m(i, j), delta);
    }
  }
}

template<class U, class V>
void matrix_compare(
    U u, V v){
  matrix_compare_delta(u, v, false);
}

struct direct_t {
  matrix_t m;
  content_t scalar;
  deque<unsigned int> row, column;
  bool trans;
  unsigned int i_offset, j_offset;
  direct_t &reset(){
    scalar = 1;
    trans = false;
    i_offset = j_offset = 0;
    row.clear();
    for(unsigned int i(0); i < m.rows(); ++i){
      row.push_back(i);
    }
    column.clear();
    for(unsigned int j(0); j < m.columns(); ++j){
      column.push_back(j);
    }
    return *this;
  }
  direct_t &shrink_map(const int &rows, const int &columns){
    row.erase(row.begin() + rows, row.end());
    column.erase(column.begin() + columns, column.end());
    return *this;
  }
  direct_t &rotate_map(const int &row_shift, const int &column_shift){
    rotate(row.begin(), row.begin() + row_shift, row.end());
    rotate(column.begin(), column.begin() + column_shift, column.end());
    return *this;
  }
  direct_t(const matrix_t &_m)
      : m(_m), row(), column() {
    reset();
  }
  content_t operator()(const unsigned &i, const unsigned &j) const {
    // strength: modification of row[], column[] > i_offset, j_offset > trans
    return m(
        row[(trans ? j : i) + i_offset],
        column[(trans ? i : j) + j_offset]) * scalar;
  }
};

BOOST_FIXTURE_TEST_SUITE(matrix, Fixture)

BOOST_AUTO_TEST_CASE(init){
  prologue_print();
  matrix_t _A(*A);
  BOOST_TEST_MESSAGE("init:" << _A);
  matrix_compare(*A, _A);
}

BOOST_AUTO_TEST_CASE(equal){
  prologue_print();
  matrix_t _A = *A;
  BOOST_TEST_MESSAGE("=:" << _A);
  matrix_compare(*A, _A);
}

BOOST_AUTO_TEST_CASE(copy){
  prologue_print();
  matrix_t _A(A->copy());
  BOOST_TEST_MESSAGE("copy:" << _A);
  matrix_compare(*A, _A);
}

BOOST_AUTO_TEST_CASE(assign_null_matrix){
  matrix_t _A(A->copy()), __A = *A;
  __A = matrix_t();
  __A = matrix_t();
  matrix_compare(*A, _A);
}

BOOST_AUTO_TEST_CASE(properties){
  prologue_print();
  BOOST_TEST_MESSAGE("rows:" << A->rows());
  BOOST_REQUIRE_EQUAL(SIZE, A->rows());
  BOOST_TEST_MESSAGE("columns:" << A->columns());
  BOOST_REQUIRE_EQUAL(SIZE, A->columns());
}

BOOST_AUTO_TEST_CASE(getI){
  prologue_print();
  matrix_t _A(matrix_t::getI(SIZE));
  BOOST_TEST_MESSAGE("I:" << _A);
  matrix_compare(k_delta, _A);
}

BOOST_AUTO_TEST_CASE(exchange_row){
  assign_linear();
  prologue_print();
  direct_t a(A->copy());
  a.row[0] = 1;
  a.row[1] = 0;
  matrix_t _A(A->exchangeRows(0, 1));
  BOOST_TEST_MESSAGE("ex_rows:" << _A);
  matrix_compare(a, _A);
}
BOOST_AUTO_TEST_CASE(exchange_column){
  assign_linear();
  prologue_print();
  direct_t a(A->copy());
  a.column[1] = 0;
  a.column[0] = 1;
  matrix_t _A(A->exchangeColumns(0, 1));
  BOOST_TEST_MESSAGE("ex_columns:" << _A);
  matrix_compare(a, _A);
}

BOOST_AUTO_TEST_CASE(check_square){
  prologue_print();
  BOOST_TEST_MESSAGE("square?:" << A->isSquare());
  BOOST_REQUIRE_EQUAL(true, A->isSquare());
}
BOOST_AUTO_TEST_CASE(check_symmetric){
  prologue_print();
  BOOST_TEST_MESSAGE("sym?:" << A->isSymmetric());
  BOOST_REQUIRE_EQUAL(true, A->isSymmetric());
}

BOOST_AUTO_TEST_CASE(scalar_mul){
  prologue_print();
  direct_t a(*A);
  a.scalar = 2.;
  matrix_t _A((*A) * a.scalar);
  BOOST_TEST_MESSAGE("*:" << _A);
  matrix_compare(a, _A);

  matrix_t _A2((*A) * a.scalar * a.scalar);
  a.scalar *= a.scalar;
  BOOST_TEST_MESSAGE("*:" << _A2);
  matrix_compare(a, _A2);
}
BOOST_AUTO_TEST_CASE(scalar_div){
  prologue_print();
  direct_t a(*A);
  int div(2);
  a.scalar = (1.0 / div);
  matrix_t _A((*A) / div);
  BOOST_TEST_MESSAGE("/:" << _A);
  matrix_compare(a, _A);
}
BOOST_AUTO_TEST_CASE(scalar_minus){
  prologue_print();
  direct_t a(*A);
  a.scalar = -1;
  matrix_t _A(-(*A));
  BOOST_TEST_MESSAGE("-():" << _A);
  matrix_compare(a, _A);
}

struct mat_add_t {
  matrix_t m1, m2;
  content_t operator()(const unsigned &i, const unsigned &j) const {
    return (m1)(i, j) + (m2)(i, j);
  }
};

BOOST_AUTO_TEST_CASE(matrix_add){
  prologue_print();
  mat_add_t a = {*A, *B};
  matrix_t _A((*A) + (*B));
  BOOST_TEST_MESSAGE("+:" << _A);
  matrix_compare_delta(a, _A, ACCEPTABLE_DELTA_DEFAULT);
}

struct mat_mul_t {
  matrix_t m1, m2;
  content_t operator()(const unsigned &i, const unsigned &j) const {
    content_t sum(0);
    for(unsigned k(0); k < m1.rows(); k++){
      sum += m1(i, k) * m2(k, j);
    }
    return sum;
  }
};

BOOST_AUTO_TEST_CASE(matrix_mul){
  prologue_print();
  mat_mul_t a = {*A, *B};
  matrix_t _A((*A) * (*B));
  BOOST_TEST_MESSAGE("*:" << _A);
  matrix_compare_delta(a, _A, ACCEPTABLE_DELTA_DEFAULT);

  mat_mul_t a2 = {(*A) * 2, *B};
  matrix_t _A2((*A) * 2 * (*B));
  BOOST_TEST_MESSAGE("*:" << _A2);
  matrix_compare_delta(a2, _A2, ACCEPTABLE_DELTA_DEFAULT);

  mat_mul_t a3 = {*A, (*B) * 2};
  matrix_t _A3((*A) * ((*B) * 2));
  BOOST_TEST_MESSAGE("*:" << _A3);
  matrix_compare_delta(a3, _A3, ACCEPTABLE_DELTA_DEFAULT);

  mat_mul_t a4 = {(*A) * 2, (*B) * 2};
  matrix_t _A4(((*A) * 2) * ((*B) * 2));
  BOOST_TEST_MESSAGE("*:" << _A4);
  matrix_compare_delta(a4, _A4, ACCEPTABLE_DELTA_DEFAULT);
}

template<class T, class Array2D_Type, class ViewType, class U>
void matrix_inspect_contains(
    const Matrix_Frozen<T, Array2D_Type, ViewType> &m,
    const U &cmp){
  boost::test_tools::output_test_stream os;
  os << m.inspect();
  BOOST_TEST_MESSAGE(os.str());
  BOOST_CHECK(os.str().find(cmp) != std::string::npos);
}
BOOST_AUTO_TEST_CASE(matrix_inspect){
  using boost::format;

  matrix_inspect_contains(
      *A,
      (format("*storage: M(%1%,%1%)") % SIZE).str());
  matrix_inspect_contains(
      A->transpose(),
      (format("*storage: Mt(%1%,%1%)") % SIZE).str());
  matrix_inspect_contains(
      A->partial(2, 3, 1, 1),
      "*storage: Mp(2,3)");
  matrix_inspect_contains(
      (*A * 2),
      (format("*storage: (*, M(%1%,%1%), 2)") % SIZE).str());
  matrix_inspect_contains(
      (2 * (*A)),
      (format("*storage: (*, M(%1%,%1%), 2)") % SIZE).str());
  matrix_inspect_contains(
      (-(*A)),
      (format("*storage: (*, M(%1%,%1%), -1)") % SIZE).str());
  matrix_inspect_contains(
      ((*A) + (*B)),
      (format("*storage: (+, M(%1%,%1%), M(%1%,%1%))") % SIZE).str());
  matrix_inspect_contains(
      ((*A) - (*B)),
      (format("*storage: (-, M(%1%,%1%), M(%1%,%1%))") % SIZE).str());
  matrix_inspect_contains(
      ((*A) * (*B)),
      (format("*storage: (*, M(%1%,%1%), M(%1%,%1%))") % SIZE).str());
  matrix_inspect_contains(
      ((*A) * (*B) * (*A)),
      (format("*storage: (*, (*, M(%1%,%1%), M(%1%,%1%)), M(%1%,%1%))") % SIZE).str());
  matrix_inspect_contains(
      (((*A) * (*B)) + (*A)),
      (format("*storage: (+, (*, M(%1%,%1%), M(%1%,%1%)), M(%1%,%1%))") % SIZE).str());
  matrix_inspect_contains(
      (((A->partial(2, 3, 1, 1).transpose()) * (B->partial(2, 3, 1, 1))) + (A->partial(3, 3, 1, 2))),
      "*storage: (+, (*, Mtp(3,2), Mp(2,3)), Mp(3,3))");

  // optimized cases
  matrix_inspect_contains(
      ((*A * 2) * 2), // should be *A * 4
      (format("*storage: (*, M(%1%,%1%), 4)") % SIZE).str());
  matrix_inspect_contains(
      (*A * (*B * 2)), // should be (*A * (*B)) * 2
      (format("*storage: (*, (*, M(%1%,%1%), M(%1%,%1%)), 2)") % SIZE).str());
  matrix_inspect_contains(
      ((*A * 2) * (*B * 2)), // should be (*A * (*B)) * 4
      (format("*storage: (*, (*, M(%1%,%1%), M(%1%,%1%)), 4)") % SIZE).str());
}

void check_inv(const matrix_t &mat){
  try{
    matrix_t inv(mat.inverse());
    BOOST_TEST_MESSAGE("inv:" << inv);
    matrix_compare_delta(matrix_t::getI(SIZE), mat * inv, 1E-5);
    matrix_compare_delta(mat, matrix_t::getI(SIZE) / inv, 1E-5);
    matrix_t inv2(1 / mat);
    BOOST_CHECK(inv == inv2);
  }catch(std::runtime_error &e){
    BOOST_ERROR("inv_error:" << e.what());
  }
}

BOOST_AUTO_TEST_CASE_MAY_FAILURES(inv, 2){
  prologue_print();
  check_inv(*A);
  assign_intermediate_zeros();
  prologue_print();
  check_inv(*A);
}
BOOST_AUTO_TEST_CASE(view){
  BOOST_CHECK((boost::is_same<
      matrix_t::transpose_t::view_t,
      MatrixViewTranspose<MatrixViewBase<> > >::value));
  BOOST_CHECK((boost::is_same<
      matrix_t::transpose_t::transpose_t::view_t,
      MatrixViewBase<> >::value));
  BOOST_CHECK((boost::is_same<
      matrix_t::transpose_t::partial_t::view_t,
      MatrixViewTranspose<MatrixViewSizeVariable<MatrixViewOffset<MatrixViewBase<> > > > >::value));
  BOOST_CHECK((boost::is_same<
      matrix_t::transpose_t::partial_t::transpose_t::view_t,
      MatrixViewSizeVariable<MatrixViewOffset<MatrixViewBase<> > > >::value));
  BOOST_CHECK((boost::is_same<
      matrix_t::partial_t::partial_t::view_t,
      MatrixViewSizeVariable<MatrixViewOffset<MatrixViewBase<> > > >::value));
  BOOST_CHECK((boost::is_same<
      matrix_t::partial_t::transpose_t::view_t,
      MatrixViewTranspose<MatrixViewSizeVariable<MatrixViewOffset<MatrixViewBase<> > > > >::value));
  BOOST_CHECK((boost::is_same<
      matrix_t::partial_t::transpose_t::transpose_t::view_t,
      MatrixViewSizeVariable<MatrixViewOffset<MatrixViewBase<> > > >::value));
  BOOST_CHECK((boost::is_same<
      matrix_t::partial_t::transpose_t::transpose_t::partial_t::view_t,
      MatrixViewSizeVariable<MatrixViewOffset<MatrixViewBase<> > > >::value));

  BOOST_CHECK((boost::is_same<
      matrix_t::circular_t::view_t,
      MatrixViewSizeVariable<MatrixViewOffset<MatrixViewLoop<MatrixViewBase<> > > > >::value));
  BOOST_CHECK((boost::is_same<
      matrix_t::partial_t::circular_t::view_t,
      MatrixViewSizeVariable<MatrixViewOffset<MatrixViewLoop<MatrixViewOffset<MatrixViewBase<> > > > > >::value));
  BOOST_CHECK((boost::is_same<
      matrix_t::circular_t::builder_t::partial_t::view_t,
      MatrixViewSizeVariable<MatrixViewOffset<MatrixViewLoop<MatrixViewBase<> > > > >::value));
  BOOST_CHECK((boost::is_same<
      matrix_t::circular_t::builder_t::circular_t::view_t,
      MatrixViewSizeVariable<MatrixViewOffset<MatrixViewLoop<
        MatrixViewOffset<MatrixViewLoop<MatrixViewBase<> > > > > > >::value));
  BOOST_CHECK((boost::is_same<
      matrix_t::circular_bijective_t::view_t,
      MatrixViewOffset<MatrixViewLoop<MatrixViewBase<> > > >::value));
  BOOST_CHECK((boost::is_same<
      matrix_t::partial_t::circular_bijective_t::view_t,
      MatrixViewSizeVariable<MatrixViewOffset<MatrixViewLoop<MatrixViewOffset<MatrixViewBase<> > > > > >::value));
  BOOST_CHECK((boost::is_same<
      matrix_t::circular_bijective_t::partial_t::view_t,
      MatrixViewSizeVariable<MatrixViewOffset<MatrixViewLoop<MatrixViewBase<> > > > >::value));
  BOOST_CHECK((boost::is_same<
      matrix_t::circular_bijective_t::circular_bijective_t::view_t,
      MatrixViewOffset<MatrixViewLoop<MatrixViewOffset<MatrixViewLoop<MatrixViewBase<> > > > > >::value));

  BOOST_CHECK((boost::is_same<
      typename matrix_t::builder_t::template view_merge_t<typename matrix_t::transpose_t::view_t>::merged_t::view_t,
      MatrixViewTranspose<MatrixViewBase<> > >::value));
  BOOST_CHECK((boost::is_same<
      typename matrix_t::transpose_t::builder_t::template view_merge_t<typename matrix_t::transpose_t::view_t>::merged_t::view_t,
      MatrixViewTranspose<MatrixViewBase<> > >::value));
  BOOST_CHECK((boost::is_same<
      typename matrix_t::builder_t::template view_merge_t<typename matrix_t::partial_t::view_t>::merged_t::view_t,
      MatrixViewSizeVariable<MatrixViewOffset<MatrixViewBase<> > > >::value));
  BOOST_CHECK((boost::is_same<
      typename matrix_t::partial_t::builder_t::template view_merge_t<typename matrix_t::partial_t::view_t>::merged_t::view_t,
      MatrixViewSizeVariable<MatrixViewOffset<MatrixViewBase<> > > >::value));
  BOOST_CHECK((boost::is_same<
      typename matrix_t::transpose_t::partial_t::builder_t::template view_merge_t<
        typename matrix_t::transpose_t::partial_t::view_t>::merged_t::view_t,
        MatrixViewTranspose<MatrixViewSizeVariable<MatrixViewOffset<MatrixViewBase<> > > > >::value));
}
BOOST_AUTO_TEST_CASE(view_inspect){
  boost::test_tools::output_test_stream os;

  os << matrix_t::partial_t::circular_t::builder_t::transpose_t::view_t();
  BOOST_TEST_MESSAGE(os.str());
  BOOST_CHECK(os.is_equal("[T] [Size](0,0) [Offset](0,0) [Loop](0,0) [Offset](0,0) [Base]"));
}
BOOST_AUTO_TEST_CASE(trans){
  assign_linear();
  prologue_print();
  direct_t a(*A);
  a.trans = true;
  matrix_t::transpose_t _A(A->transpose());
  BOOST_TEST_MESSAGE("trans:" << _A);
  matrix_compare(a, _A);
  matrix_t __A(_A.copy());
  BOOST_TEST_MESSAGE("trans.copy:" << __A);
  matrix_compare(a, __A);
  matrix_t::transpose_t::transpose_t ___A(_A.transpose()); // matrix_t::transpose_t::transpose_t = matrix_t
  BOOST_TEST_MESSAGE("trans.trans:" << ___A);
  matrix_compare(*A, ___A);
}
BOOST_AUTO_TEST_CASE(partial){
  assign_linear();
  prologue_print();
  direct_t a(*A);

  BOOST_CHECK_THROW(A->partial(A->rows() + 1, 0, 0, 0), std::logic_error);
  BOOST_CHECK_THROW(A->partial(0, A->columns() + 1, 0, 0), std::logic_error);
  BOOST_CHECK_THROW(A->partial(A->rows(), 0, 1, 0), std::logic_error);
  BOOST_CHECK_THROW(A->partial(0, A->columns(), 0, 1), std::logic_error);

  BOOST_CHECK_THROW(
      A->partial(A->rows() / 2, A->columns() / 2, 0, 0)
        .partial(A->rows() / 2 + 1, A->columns() / 2, 0, 0), std::logic_error);
  BOOST_CHECK_THROW(
      A->partial(A->rows() / 2, A->columns() / 2, 0, 0)
        .partial(A->rows() / 2, A->columns() / 2 + 1, 0, 0), std::logic_error);
  BOOST_CHECK_THROW(
      A->partial(A->rows() / 2, A->columns() / 2, 0, 0)
        .partial(A->rows() / 2, A->columns() / 2, 1, 0), std::logic_error);
  BOOST_CHECK_THROW(
      A->partial(A->rows() / 2, A->columns() / 2, 0, 0)
        .partial(A->rows() / 2, A->columns() / 2, 0, 1), std::logic_error);

  a.i_offset = a.j_offset = 1;
  matrix_t::partial_t _A(A->partial(3, 3, a.i_offset, a.j_offset));
  BOOST_TEST_MESSAGE("_A:" << _A);
  matrix_compare(a, _A);
  matrix_compare(a, _A.copy());

  unsigned int i_offset2, j_offset2;
  i_offset2 = j_offset2 = 1;

  _A = _A.partial(
      _A.rows() - i_offset2, _A.columns() - j_offset2,
      i_offset2, j_offset2);
  a.i_offset += i_offset2;
  a.j_offset += j_offset2;
  BOOST_TEST_MESSAGE("_A:" << _A);
  matrix_compare(a, _A);
}
BOOST_AUTO_TEST_CASE(trans_partial){
  assign_linear();
  prologue_print();
  direct_t a(*A);

  a.trans = true;   a.i_offset = 4; a.j_offset = 3;
  matrix_compare(a, A->transpose().partial(2,3,3,4));

  a.trans = false;  a.i_offset = 4; a.j_offset = 3;
  matrix_compare(a, A->transpose().partial(2,3,3,4).transpose());

  a.trans = true;   a.i_offset = 6; a.j_offset = 4;
  matrix_compare(a, A->transpose().partial(2,3,3,4).partial(1,1,1,2));

  a.trans = true;   a.i_offset = 3; a.j_offset = 4;
  matrix_compare(a, A->partial(2,3,3,4).transpose());

  a.trans = false;  a.i_offset = 3; a.j_offset = 4;
  matrix_compare(a, A->partial(2,3,3,4).transpose().transpose());

  a.trans = false;  a.i_offset = 4; a.j_offset = 6;
  matrix_compare(a, A->partial(2,3,3,4).transpose().transpose().partial(1,1,1,2));

  a.trans = false;  a.i_offset = 5; a.j_offset = 5;
  matrix_compare(a, A->partial(3,4,3,4).transpose().partial(3,1,1,2).transpose());
}
BOOST_AUTO_TEST_CASE(circular){
  assign_linear();
  prologue_print();
  direct_t a(*A);

#define print_then_compare(mat) \
BOOST_TEST_MESSAGE(#mat ": " << (mat)); \
matrix_compare(a, mat)
  a.reset().rotate_map(1,2);
  print_then_compare(A->circular(1,2));

  a.reset().rotate_map(2,1); a.trans = true;
  print_then_compare(A->transpose().circular(1,2));

  a.reset().rotate_map(5,6).rotate_map(1,2);
  print_then_compare(A->circular(5,6).partial(6,5,1,2));

  a.reset().rotate_map(5,6).rotate_map(2,1); a.trans = true;
  print_then_compare(A->circular(5,6).transpose().partial(6,5,1,2));

  a.reset().rotate_map(1,2).shrink_map(6,5).rotate_map(3,2);
  print_then_compare(A->partial(6,5,1,2).circular(3,2));

  a.reset().rotate_map(1,2).shrink_map(6,5).rotate_map(2,3); a.trans = true;
  print_then_compare(A->partial(6,5,1,2).transpose().circular(3,2));


  a.reset().rotate_map(1,2);
  print_then_compare(A->circular(1,2,3,4));

  a.reset().rotate_map(2,1); a.trans = true;
  print_then_compare(A->transpose().circular(1,2,3,4));

  a.reset().rotate_map(5,6).shrink_map(6,7).rotate_map(1,2);
  print_then_compare(A->circular(5,6,6,7).partial(4,3,1,2));

  a.reset().rotate_map(5,6).shrink_map(6,7).rotate_map(2,1); a.trans = true;
  print_then_compare(A->circular(5,6,6,7).transpose().partial(4,3,1,2));

  a.reset().rotate_map(1,2).shrink_map(6,5).rotate_map(3,2);
  print_then_compare(A->partial(6,5,1,2).circular(3,2,4,3));

  a.reset().rotate_map(1,2).shrink_map(6,5).rotate_map(2,3); a.trans = true;
  print_then_compare(A->partial(6,5,1,2).transpose().circular(3,2,4,3));
#undef print_then_compare
}

BOOST_AUTO_TEST_CASE(view_downcast){
  assign_linear();
  prologue_print();

  direct_t a(*A);
  a.trans = true;   a.i_offset = 4; a.j_offset = 3;

  typedef matrix_t::transpose_t::partial_t mat_tp_t;
  mat_tp_t mat_tp(A->transpose().partial(2,3,3,4));

  matrix_t _A(static_cast<mat_tp_t::super_t &>(mat_tp)); // downcast (internally deep copy) is available
  //matrix_t _A2(mat_tp); // compile error due to protected
  matrix_compare(a, _A);
}

BOOST_AUTO_TEST_CASE(replace){
  assign_linear();
  prologue_print();

  direct_t a(*A), b(B->copy());

  B->transpose().partial(2,3,3,4).replace(A->transpose().partial(2,3,3,4));

  // replaced part
  a.i_offset = 4; a.j_offset = 3; matrix_compare(a, B->partial(3,2,4,3));

  // other parts
  b.i_offset = 0; b.j_offset = 0; matrix_compare(b, B->partial(B->rows(),3,0,0)); // left
  b.i_offset = 0; b.j_offset = 5; matrix_compare(b, B->partial(B->rows(),3,0,5)); // right
  b.i_offset = 0; b.j_offset = 0; matrix_compare(b, B->partial(4,B->columns(),0,0)); // top
  b.i_offset = 7; b.j_offset = 0; matrix_compare(b, B->partial(1,B->columns(),7,0)); // bottom
}

BOOST_AUTO_TEST_CASE(minor){
  assign_linear();
  prologue_print();
  for(unsigned i(0); i < A->rows(); ++i){
    for(unsigned j(0); j < A->columns(); ++j){
      direct_t a(*A);
      a.row.erase(a.row.begin() + i);
      a.column.erase(a.column.begin() + j);
      matrix_t _A(A->matrix_for_minor(i, j));
      BOOST_TEST_MESSAGE("matrix_for_minor:" << _A);
      matrix_compare(a, _A);
    }
  }
}

BOOST_AUTO_TEST_CASE(det){
  prologue_print();
  BOOST_CHECK_SMALL(A->determinant_minor() - A->determinant(), ACCEPTABLE_DELTA_DEFAULT);
  BOOST_TEST_MESSAGE("det:" << A->determinant());

  assign_unsymmetric();
  assign_intermediate_zeros();
  prologue_print();
  BOOST_CHECK_SMALL(A->determinant_minor() - A->determinant(), ACCEPTABLE_DELTA_DEFAULT);
  BOOST_TEST_MESSAGE("det:" << A->determinant());
}

BOOST_AUTO_TEST_CASE(pivot_merge){
  prologue_print();
  for(int i(-(int)(A->rows()) + 1); i < (int)A->rows(); ++i){
    for(int j(-(int)(A->columns()) + 1); j < (int)A->columns(); ++j){
      matrix_t A_copy(A->copy());
      matrix_t _A(A->pivotMerge(i, j, *B));
      BOOST_TEST_MESSAGE("pivotMerge:" << _A);

      // make [[X, 0], [0, 0]], [[0, X], [0, 0]], [[0, 0], [X, 0]], or [[0, 0], [0, X]]
      unsigned int
          i_B(i < 0 ? 0 : i), j_B(j < 0 ? 0 : j),
          i2_B(i < 0 ? -i : 0), j2_B(j < 0 ? -j : 0),
          rows_B(B->rows() + (i < 0 ? i : -i)),
          columns_B(B->columns() + (j < 0 ? j : -j));
      matrix_t B_cutout(matrix_t::getScalar(B->rows(), 0));
      B_cutout.partial(rows_B, columns_B, i_B, j_B).replace(
          B->partial(rows_B, columns_B, i2_B, j2_B));
      mat_add_t a = {A_copy, B_cutout};
      matrix_compare_delta(a, _A, ACCEPTABLE_DELTA_DEFAULT);
    }
  }
}
BOOST_AUTO_TEST_CASE(pivot_add){
  prologue_print();
  for(int i(-(int)(A->rows()) + 1); i < (int)A->rows(); ++i){
    for(int j(-(int)(A->columns()) + 1); j < (int)A->columns(); ++j){
      matrix_t _A(A->pivotAdd(i, j, *B));
      BOOST_TEST_MESSAGE("pivotAdd:" << _A);

      // make [[X, 0], [0, 0]], [[0, X], [0, 0]], [[0, 0], [X, 0]], or [[0, 0], [0, X]]
      unsigned int
          i_B(i < 0 ? 0 : i), j_B(j < 0 ? 0 : j),
          i2_B(i < 0 ? -i : 0), j2_B(j < 0 ? -j : 0),
          rows_B(B->rows() + (i < 0 ? i : -i)),
          columns_B(B->columns() + (j < 0 ? j : -j));
      matrix_t B_cutout(matrix_t::getScalar(B->rows(), 0));
      B_cutout.partial(rows_B, columns_B, i_B, j_B).replace(
          B->partial(rows_B, columns_B, i2_B, j2_B));
      mat_add_t a = {*A, B_cutout};
      matrix_compare_delta(a, _A, ACCEPTABLE_DELTA_DEFAULT);
    }
  }
}

BOOST_AUTO_TEST_CASE_MAY_FAILURES(eigen, 1){
  prologue_print();
  try{
    cmatrix_t A_copy(A->rows(), A->columns());
    for(unsigned i(0); i < A_copy.rows(); i++){
      for(unsigned j(0); j < A_copy.columns(); j++){
        A_copy(i, j).real() = (*A)(i, j);
      }
    }
    cmatrix_t _A(A->eigen());
    BOOST_TEST_MESSAGE("eigen:" << _A);
    for(unsigned i(0); i < A->rows(); i++){
      BOOST_TEST_MESSAGE("eigen(" << i << "):" << A_copy * _A.partial(A->rows(), 1, 0, i));
      BOOST_TEST_MESSAGE("eigen(" << i << "):" << _A.partial(A->rows(), 1, 0, i) * _A(i, A->rows()));
      matrix_compare_delta(A_copy * _A.partial(A->rows(), 1, 0, i),
          _A.partial(A->rows(), 1, 0, i) * _A(i, A->rows()), 1E-4);
    }
  }catch(std::runtime_error &e){
    BOOST_ERROR("eigen_error:" << e.what());
  }
}

BOOST_AUTO_TEST_CASE_MAY_FAILURES(sqrt, 1){
  prologue_print();
  try{
    cmatrix_t _A(A->sqrt());
    BOOST_TEST_MESSAGE("sqrt:" << _A);
    matrix_compare_delta(*A, _A * _A, 1E-4);
  }catch(std::runtime_error &e){
    BOOST_ERROR("sqrt_error:" << e.what());
  }
}

void check_LU(const matrix_t &mat){
  struct pivot_t {
    unsigned num;
    unsigned *indices;
    pivot_t(const unsigned &size) : indices(new unsigned [size]) {}
    ~pivot_t(){
      delete [] indices;
    }
  } pivot(mat.rows());

  matrix_t LU(mat.decomposeLUP(pivot.num, pivot.indices)), LU2(mat.decomposeLU());
  matrix_compare_delta(LU, LU2, ACCEPTABLE_DELTA_DEFAULT);
  matrix_t::partial_t
      L(LU.partial(LU.rows(), LU.rows(), 0, 0)),
      U(LU.partial(LU.rows(), LU.rows(), 0, LU.rows()));
  BOOST_TEST_MESSAGE("LU(L):" << L);
  BOOST_TEST_MESSAGE("LU(U):" << U);

  for(unsigned i(0); i < mat.rows(); i++){
    for(unsigned j(i+1); j < mat.columns(); j++){
      BOOST_CHECK_EQUAL(L(i, j), 0);
      BOOST_CHECK_EQUAL(U(j, i), 0);
    }
  }
  BOOST_REQUIRE_EQUAL(true, LU.isLU());

  matrix_t LU_multi(L * U);
  BOOST_TEST_MESSAGE("LU(L) * LU(U):" << LU_multi);
  // column permutation will possibly be performed
  set<unsigned> unused_columns;
  for(unsigned j(0); j < mat.columns(); j++){
    unused_columns.insert(j);
  }
  for(unsigned j(0); j < mat.columns(); j++){
    for(set<unsigned>::iterator it(unused_columns.begin());
        ;
        ++it){
      if(it == unused_columns.end()){BOOST_FAIL("L * U != A");}
      //cout << *it << endl;
      bool matched(true);
      for(unsigned i(0); i < mat.rows(); i++){
        content_t delta(mat(i, j) - LU_multi(i, *it));
        //cerr << delta << endl;
        if(delta < 0){delta *= -1;}
        if(delta > ACCEPTABLE_DELTA_DEFAULT){matched = false;}
      }
      if(matched && (pivot.indices[j] == *it)){
        //cerr << "matched: " << *it << endl;
        unused_columns.erase(it);
        break;
      }
    }
  }
}

BOOST_AUTO_TEST_CASE(LU){
  prologue_print();
  check_LU(*A);
  assign_intermediate_zeros();
  prologue_print();
  check_LU(*A);
}

BOOST_AUTO_TEST_CASE(UH){
  prologue_print();
  matrix_t U(matrix_t::getI(A->rows()));
  matrix_t H(A->hessenberg(&U));

  BOOST_TEST_MESSAGE("hessen(H):" << H);
  for(unsigned i(2); i < H.rows(); i++){
    for(unsigned j(0); j < (i - 1); j++){
      BOOST_CHECK_EQUAL(H(i, j), 0);
    }
  }
  matrix_t _A(U * H * U.transpose());
  BOOST_TEST_MESSAGE("U * H * U^{T}:" << _A);
  matrix_compare_delta(*A, _A, ACCEPTABLE_DELTA_DEFAULT);
}

BOOST_AUTO_TEST_CASE(UD){
  prologue_print();
  matrix_t UD(A->decomposeUD());
  matrix_t::partial_t
      U(UD.partial(UD.rows(), UD.rows(), 0, 0)),
      D(UD.partial(UD.rows(), UD.rows(), 0, UD.rows()));
  BOOST_TEST_MESSAGE("UD(U):" << U);
  BOOST_TEST_MESSAGE("UD(D):" << D);

  for(unsigned i(0); i < A->rows(); i++){
    for(unsigned j(i+1); j < A->columns(); j++){
      BOOST_CHECK_EQUAL(U(j, i), 0);
      BOOST_CHECK_EQUAL(D(i, j), 0);
      BOOST_CHECK_EQUAL(D(j, i), 0);
    }
  }

  BOOST_TEST_MESSAGE("UD(U):" << U.transpose());
  matrix_t _A(U * D * U.transpose());
  BOOST_TEST_MESSAGE("U * D * U^{T}:" << _A);
  matrix_compare_delta(*A, _A, ACCEPTABLE_DELTA_DEFAULT);
}

template <class FloatT>
void mat_mul(FloatT *x, const int &r1, const int &c1,
    FloatT *y, const int &c2,
    FloatT *r,
    const bool &x_trans = false, const bool &y_trans = false){
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
  for(int i(0); i < r1; i++){
    for(int j(0); j < c2; j++){
      indx = i * indx_r;
      indy = j * indy_c;
      r[indr] = FloatT(0);
      for(int k(0); k < c1; k++){
        r[indr] += x[indx] * y[indy];
        indx += indx_c;
        indy += indy_r;
      }
      indr++;
    }
  }
}

template <class FloatT>
void mat_mul_unrolled(FloatT *x, const int &r1, const int &c1,
    FloatT *y, const int &c2,
    FloatT *r,
    const bool &x_trans = false, const bool &y_trans = false){
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

BOOST_AUTO_TEST_CASE(unrolled_product){ // This test is experimental for SIMD such as DSP
  assign_unsymmetric();
  prologue_print();
  content_t *AB_array(new content_t[A->rows() * B->columns()]);
  {
    // normal * normal
    mat_mul_unrolled((content_t *)A_array, A->rows(), A->columns(),
        (content_t *)B_array, B->columns(),
        (content_t *)AB_array);
    matrix_t AB((*A) * (*B));
    matrix_compare_delta(AB_array, AB, ACCEPTABLE_DELTA_DEFAULT);
  }
  {
    // normal * transpose
    mat_mul_unrolled((content_t *)A_array, A->rows(), A->columns(),
        (content_t *)B_array, B->columns(),
        (content_t *)AB_array, false, true);
    matrix_t AB((*A) * B->transpose());
    matrix_compare_delta(AB_array, AB, ACCEPTABLE_DELTA_DEFAULT);
  }
  {
    // transpose * normal
    mat_mul_unrolled((content_t *)A_array, A->rows(), A->columns(),
        (content_t *)B_array, B->columns(),
        (content_t *)AB_array, true, false);
    matrix_t AB(A->transpose() * (*B));
    matrix_compare_delta(AB_array, AB, ACCEPTABLE_DELTA_DEFAULT);
  }
  {
    // transpose * transpose
    mat_mul_unrolled((content_t *)A_array, A->rows(), A->columns(),
        (content_t *)B_array, B->columns(),
        (content_t *)AB_array, true, true);
    matrix_t AB(A->transpose() * B->transpose());
    matrix_compare_delta(AB_array, AB, ACCEPTABLE_DELTA_DEFAULT);
  }
  delete [] AB_array;
}

#if 1
BOOST_AUTO_TEST_CASE_MAY_FAILURES(fixed, 1){
  prologue_print();
  typedef Matrix_Fixed<content_t, SIZE> fixed_t;
  fixed_t _A(fixed_t::blank(SIZE, SIZE).replace(*A));
  matrix_compare_delta(*A, _A, ACCEPTABLE_DELTA_DEFAULT);

  typedef Matrix_Fixed<Complex<content_t>, SIZE> cfixed_t;
  cfixed_t _Ac1(cfixed_t::blank(SIZE, SIZE).replace(*A));
  cfixed_t _Ac2(_Ac1.copy());
  matrix_compare_delta(_A, _Ac1, ACCEPTABLE_DELTA_DEFAULT);
  matrix_compare_delta(_A, _Ac2, ACCEPTABLE_DELTA_DEFAULT);

  try{
    matrix_compare_delta(A->eigen(), _A.eigen(), ACCEPTABLE_DELTA_DEFAULT);
    matrix_compare_delta(
        A->partial(SIZE - 1, SIZE - 1, 0, 0).eigen(),
        _A.partial(SIZE - 1, SIZE - 1, 0, 0).eigen(),
        ACCEPTABLE_DELTA_DEFAULT);
  }catch(std::runtime_error &e){
    BOOST_ERROR("eigen_error:" << e.what());
  }
}

BOOST_AUTO_TEST_CASE(fixed_types){
  BOOST_CHECK((boost::is_same<
      Matrix_Frozen<content_t, Array2D_Operator<content_t, Array2D_Operator_Multiply<
        Matrix_Frozen<content_t,Array2D_Fixed<content_t, 2, 4> >,
        Matrix_Frozen<content_t, Array2D_Operator<content_t, Array2D_Operator_Multiply<
          Matrix_Frozen<content_t,Array2D_Fixed<content_t, 4, 8> >,
          Matrix_Frozen<content_t,Array2D_Fixed<content_t, 8, 16> > > > > > > >::builder_t::assignable_t,
      Matrix_Fixed<content_t, 2, 16> >::value));
  BOOST_CHECK((boost::is_same<
      Matrix_Frozen<content_t, Array2D_Operator<content_t, Array2D_Operator_Multiply<
        Matrix_Frozen<content_t, Array2D_Operator<content_t, Array2D_Operator_Multiply<
          Matrix_Frozen<content_t,Array2D_Fixed<content_t, 2, 4> >,
          Matrix_Frozen<content_t,Array2D_Fixed<content_t, 4, 8> > > > >,
        Matrix_Frozen<content_t,Array2D_Fixed<content_t, 8, 16> > > > >::builder_t::assignable_t,
      Matrix_Fixed<content_t, 2, 16> >::value));
  BOOST_CHECK((boost::is_same<
      Matrix_Fixed<content_t, 2, 4>::super_t::super_t
        ::template Multiply_Matrix_by_Matrix<Matrix_Fixed<content_t, 4, 8>::super_t::super_t>::mat_t
        ::template Multiply_Matrix_by_Matrix<Matrix_Fixed<content_t, 16, 8>::super_t::super_t::builder_t::transpose_t>::mat_t
        ::builder_t::transpose_t::builder_t::assignable_t,
      Matrix_Fixed<content_t, 16, 2> >::value));
  BOOST_CHECK((boost::is_same<
      Matrix_Fixed<content_t, 2, 4>::super_t::super_t
        ::template Multiply_Matrix_by_Matrix<Matrix_Fixed<content_t, 4, 8>::super_t::super_t>::mat_t
        ::template Add_Matrix_to_Matrix<Matrix_Fixed<content_t, 3, 7>::super_t::super_t>::mat_t
        ::template Multiply_Matrix_by_Scalar<int>::mat_t
        ::template Multiply_Matrix_by_Matrix<Matrix_Fixed<content_t, 8, 16>::super_t::super_t>::mat_t
        ::builder_t::assignable_t,
      Matrix_Fixed<content_t, 2, 16> >::value));
}

#endif

BOOST_AUTO_TEST_SUITE_END()
