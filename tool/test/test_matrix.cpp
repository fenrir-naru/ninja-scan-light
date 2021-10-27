#include <string>
#include <cstring>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <set>
#include <deque>
#include <algorithm>
#include <numeric>

#include <boost/type_traits/is_same.hpp>

#include <boost/format.hpp>

#define BOOST_TEST_MAIN
#include <boost/test/included/unit_test.hpp>
#include "test_matrix/common.h"

using namespace std;

int k_delta(const unsigned &i, const unsigned &j){
  return (i == j) ? 1 : 0;
}

typedef double content_t;
typedef Matrix<content_t> matrix_t;
typedef Matrix<Complex<content_t> > cmatrix_t;

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

struct split_r_i_t {
  matrix_t mat_r, mat_i;
  template <class T, class Array2D_Type, class ViewType>
  split_r_i_t(const Matrix_Frozen<Complex<T>, Array2D_Type, ViewType> &mat)
      : mat_r(mat.rows(), mat.columns()), mat_i(mat.rows(), mat.columns()) {
    for(unsigned int i(0); i < mat.rows(); ++i){
      for(unsigned int j(0); j < mat.columns(); ++j){
        mat_r(i, j) = mat(i, j).real();
        mat_i(i, j) = mat(i, j).imaginary();
      }
    }
  }
};

BOOST_FIXTURE_TEST_SUITE(matrix, Fixture<content_t>)

BOOST_AUTO_TEST_CASE(copy_ctor){
  prologue_print();
  matrix_t _A(*A);
  BOOST_TEST_MESSAGE("copy_ctor:" << _A);
  matrix_compare(*A, _A);

  cmatrix_t _Ac(*A);
  BOOST_TEST_MESSAGE("copy_ctor(real->complex):" << _Ac);
  matrix_compare(*A, _Ac);
}

BOOST_AUTO_TEST_CASE(null_copy){
  // check no access violation derived from double delete
  matrix_t _A, __A(_A);
}

BOOST_AUTO_TEST_CASE(assign){
  prologue_print();
  matrix_t _A;
  _A = *A; // expect shallow copy due to same type
  BOOST_TEST_MESSAGE("=:" << _A);
  matrix_compare(*A, _A);

  cmatrix_t _Ac;
  _Ac = _A; // expect deep copy due to same type
  matrix_compare(_Ac, _A);

  _A(0, 0) += 1;
  matrix_compare(*A, _A);
  for(unsigned int i(0), i_end(A->rows()); i < i_end; ++i){
    for(unsigned int j(0), j_end(A->columns()); j < j_end; ++j){
      BOOST_REQUIRE((_Ac(i, j) == (*A)(i, j)) ^ ((i == 0) && (j == 0)));
    }
  }
}

BOOST_AUTO_TEST_CASE(copy){
  prologue_print();
  matrix_t _A(A->copy());
  BOOST_TEST_MESSAGE("copy:" << _A);
  matrix_compare(*A, _A);
  _A(0, 0) += 1;
  BOOST_TEST_MESSAGE("copy_mod:" << _A);
  for(unsigned int i(0), i_end(A->rows()); i < i_end; ++i){
    for(unsigned int j(0), j_end(A->columns()); j < j_end; ++j){
      BOOST_REQUIRE((_A(i, j) == (*A)(i, j)) ^ ((i == 0) && (j == 0)));
    }
  }
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
  matrix_t _A(matrix_t::getI(SIZE));
  BOOST_TEST_MESSAGE("I:" << _A);
  matrix_compare(k_delta, _A);

  // type(getI()) == type(getI().transpose)
  BOOST_CHECK((boost::is_same<
      Matrix_Frozen<content_t, Array2D_ScaledUnit<content_t> >,
      Matrix_Frozen<content_t, Array2D_ScaledUnit<content_t> >::builder_t::transpose_t>::value));
}

BOOST_AUTO_TEST_CASE(swap_row){
  assign_linear();
  prologue_print();
  direct_t a(A->copy());
  a.row[0] = 1;
  a.row[1] = 0;
  matrix_t _A(A->swapRows(0, 1));
  BOOST_TEST_MESSAGE("ex_rows:" << _A);
  matrix_compare(a, _A);
}
BOOST_AUTO_TEST_CASE(swap_column){
  assign_linear();
  prologue_print();
  direct_t a(A->copy());
  a.column[1] = 0;
  a.column[0] = 1;
  matrix_t _A(A->swapColumns(0, 1));
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

BOOST_AUTO_TEST_CASE(sum){
  prologue_print();
  BOOST_TEST_MESSAGE("sum:" << A->sum());
  BOOST_REQUIRE_EQUAL(std::accumulate(A->begin(), A->end(), content_t(0)), A->sum());
}
BOOST_AUTO_TEST_CASE(trace){
  for(matrix_t::iterator it(A->begin()), it_end(A->end()); it != it_end; ++it){
    if(it.row() != it.column()){*it = 0;} // Remove nondiagonal elements
  }
  prologue_print();
  BOOST_TEST_MESSAGE("trace:" << A->trace());
  BOOST_REQUIRE_EQUAL(A->sum(), A->trace());
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
  mat_add_t a1 = {*A, *B};
  matrix_t A1((*A) + (*B));
  BOOST_TEST_MESSAGE("+:" << A1);
  matrix_compare_delta(a1, A1, ACCEPTABLE_DELTA_DEFAULT);

  mat_add_t a2 = {*A, matrix_t::getI(A->rows())};
  matrix_t A2((*A) + 1);
  BOOST_TEST_MESSAGE("+1:" << A2);
  matrix_compare_delta(a2, A2, ACCEPTABLE_DELTA_DEFAULT);

  matrix_t A2_(1 + (*A));
  BOOST_TEST_MESSAGE("1+:" << A2_);
  matrix_compare_delta(a2, A2_, ACCEPTABLE_DELTA_DEFAULT);

  mat_add_t a3 = {*A, matrix_t::getScalar(A->rows(), -1)};
  matrix_t A3((*A) - 1);
  BOOST_TEST_MESSAGE("-1:" << A3);
  matrix_compare_delta(a3, A3, ACCEPTABLE_DELTA_DEFAULT);

  mat_add_t a4 = {matrix_t::getI(A->rows()), -(*A)};
  matrix_t A4(1 - (*A));
  BOOST_TEST_MESSAGE("1-:" << A4);
  matrix_compare_delta(a4, A4, ACCEPTABLE_DELTA_DEFAULT);
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

BOOST_AUTO_TEST_CASE(matrix_inspect){
  using boost::format;

  matrix_inspect_contains(
      *A,
      (format("*storage: M(%1%,%1%)") % SIZE).str());
  matrix_inspect_contains(
      A->transpose(),
      (format("*storage: Mt(%1%,%1%)") % SIZE).str());
  matrix_inspect_contains(
      rAiB->conjugate(),
      (format("*storage: M~(%1%,%1%)") % SIZE).str());
  matrix_inspect_contains(
      rAiB->adjoint(),
      (format("*storage: M~t(%1%,%1%)") % SIZE).str());
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
      (*A * matrix_t::getScalar(A->columns(), 2)),
      (format("*storage: (*, M(%1%,%1%), 2)") % SIZE).str());
  matrix_inspect_contains(
      (matrix_t::getScalar(A->rows(), 2) * (*A)),
      (format("*storage: (*, M(%1%,%1%), 2)") % SIZE).str());
  matrix_inspect_contains(
      (matrix_t::getScalar(A->rows(), 2) * 2), // should be 4_I
      (format("*storage: M(%1%,%1%)") % SIZE).str());
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
      (((*A) * (*B)) + (*A)),
      (format("*storage: (+, (*, M(%1%,%1%), M(%1%,%1%)), M(%1%,%1%))") % SIZE).str());
  matrix_inspect_contains(
      (((A->partial(2, 3, 1, 1).transpose()) * (B->partial(2, 3, 1, 1))) + (A->partial(3, 3, 1, 2))).transpose(),
      "*storage: (+, (*, Mtp(3,2), Mp(2,3)), Mp(3,3))t");

  // optimized cases
  matrix_inspect_contains(
      (matrix_t::getScalar(A->rows(), 2) * matrix_t::getScalar(A->rows(), 2)), // should be 4_I
      (format("*storage: M(%1%,%1%)") % SIZE).str());
  matrix_inspect_contains(
      ((*A * 2) * 2), // should be *A * 4
      (format("*storage: (*, M(%1%,%1%), 4)") % SIZE).str());
  matrix_inspect_contains(
      (*A * (*B * 2)), // should be (*A * (*B)) * 2
      (format("*storage: (*, (*, M(%1%,%1%), M(%1%,%1%)), 2)") % SIZE).str());
  matrix_inspect_contains(
      ((*A * 2) * (*B * 2)), // should be (*A * (*B)) * 4
      (format("*storage: (*, (*, M(%1%,%1%), M(%1%,%1%)), 4)") % SIZE).str());
  matrix_inspect_contains(
      ((*A * 2) * (matrix_t::getScalar(B->rows(), 2) * 2)), // should be *A * 8
      (format("*storage: (*, M(%1%,%1%), 8)") % SIZE).str());
  matrix_inspect_contains(
      ((*A) * (*B) * (*A)),
      (format("*storage: (*, M(%1%,%1%), M(%1%,%1%))") % SIZE).str()); // should be M * M
  matrix_inspect_contains(
      ((*A) / matrix_t::getScalar(A->rows(), 2)),
      (format("*storage: (*, M(%1%,%1%), 0.5)") % SIZE).str()); // should be M * 0.5
}

void check_inv(const matrix_t &mat){
  try{
    matrix_t inv(mat.inverse());
    BOOST_TEST_MESSAGE("inv:" << inv);
    matrix_compare_delta(matrix_t::getI(SIZE), mat * inv, 1E-5);
    matrix_compare_delta(mat, matrix_t::getI(SIZE) / inv, 1E-5);
    matrix_compare_delta(mat, mat / matrix_t::getI(SIZE), 1E-5);
    matrix_compare_delta(matrix_t::getI(SIZE), inv / inv, 1E-5);
    matrix_compare_delta(matrix_t::getI(SIZE), (inv * inv) / (inv * inv), 1E-5);
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
BOOST_AUTO_TEST_CASE(inv_scalar){
  matrix_t::scalar_matrix_t a(matrix_t::getScalar(A->rows(), 2));
  BOOST_TEST_MESSAGE("scalar:" << a);
  check_inv(a);
}
BOOST_AUTO_TEST_CASE(view_property){
  BOOST_CHECK(true  == MatrixViewProperty<matrix_t::view_t>::viewless);
  BOOST_CHECK(false == MatrixViewProperty<matrix_t::view_t>::transposed);
  BOOST_CHECK(false == MatrixViewProperty<matrix_t::view_t>::conjugated);
  BOOST_CHECK(false == MatrixViewProperty<matrix_t::view_t>::offset);
  BOOST_CHECK(false == MatrixViewProperty<matrix_t::view_t>::variable_size);

  BOOST_CHECK(false == MatrixViewProperty<matrix_t::transpose_t::view_t>::viewless);
  BOOST_CHECK(true  == MatrixViewProperty<matrix_t::transpose_t::view_t>::transposed);
  BOOST_CHECK(false == MatrixViewProperty<matrix_t::transpose_t::view_t>::conjugated);
  BOOST_CHECK(false == MatrixViewProperty<matrix_t::transpose_t::view_t>::offset);
  BOOST_CHECK(false == MatrixViewProperty<matrix_t::transpose_t::view_t>::variable_size);

  BOOST_CHECK(false == MatrixViewProperty<matrix_t::partial_offsetless_t::view_t>::viewless);
  BOOST_CHECK(false == MatrixViewProperty<matrix_t::partial_offsetless_t::view_t>::transposed);
  BOOST_CHECK(false == MatrixViewProperty<matrix_t::partial_offsetless_t::view_t>::conjugated);
  BOOST_CHECK(false == MatrixViewProperty<matrix_t::partial_offsetless_t::view_t>::offset);
  BOOST_CHECK(true  == MatrixViewProperty<matrix_t::partial_offsetless_t::view_t>::variable_size);

  BOOST_CHECK(false == MatrixViewProperty<matrix_t::partial_t::view_t>::viewless);
  BOOST_CHECK(false == MatrixViewProperty<matrix_t::partial_t::view_t>::transposed);
  BOOST_CHECK(false == MatrixViewProperty<matrix_t::partial_t::view_t>::conjugated);
  BOOST_CHECK(true  == MatrixViewProperty<matrix_t::partial_t::view_t>::offset);
  BOOST_CHECK(true  == MatrixViewProperty<matrix_t::partial_t::view_t>::variable_size);

  BOOST_CHECK(false == MatrixViewProperty<matrix_t::circular_bijective_t::view_t>::viewless);
  BOOST_CHECK(false == MatrixViewProperty<matrix_t::circular_bijective_t::view_t>::transposed);
  BOOST_CHECK(false == MatrixViewProperty<matrix_t::circular_bijective_t::view_t>::conjugated);
  BOOST_CHECK(true  == MatrixViewProperty<matrix_t::circular_bijective_t::view_t>::offset);
  BOOST_CHECK(false == MatrixViewProperty<matrix_t::circular_bijective_t::view_t>::variable_size);

  BOOST_CHECK(false == MatrixViewProperty<matrix_t::circular_t::view_t>::viewless);
  BOOST_CHECK(false == MatrixViewProperty<matrix_t::circular_t::view_t>::transposed);
  BOOST_CHECK(false == MatrixViewProperty<matrix_t::circular_t::view_t>::conjugated);
  BOOST_CHECK(true  == MatrixViewProperty<matrix_t::circular_t::view_t>::offset);
  BOOST_CHECK(true  == MatrixViewProperty<matrix_t::circular_t::view_t>::variable_size);

  BOOST_CHECK(false == MatrixViewProperty<matrix_t::conjugate_t::view_t>::conjugated);
  BOOST_CHECK(true  == MatrixViewProperty<cmatrix_t::conjugate_t::view_t>::conjugated);
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
      matrix_t::conjugate_t::view_t,
      MatrixViewBase<> >::value));
  BOOST_CHECK((boost::is_same<
      cmatrix_t::conjugate_t::view_t,
      MatrixViewConjugate<MatrixViewBase<> > >::value));
  BOOST_CHECK((boost::is_same<
      cmatrix_t::conjugate_t::conjugate_t::view_t,
      MatrixViewBase<> >::value));
  BOOST_CHECK((boost::is_same<
      cmatrix_t::conjugate_t::transpose_t::conjugate_t::view_t,
      MatrixViewTranspose<MatrixViewBase<> > >::value));

  BOOST_CHECK((boost::is_same<
      typename matrix_t::builder_t::template view_apply_t<typename matrix_t::transpose_t::view_t>::applied_t::view_t,
      MatrixViewTranspose<MatrixViewBase<> > >::value));
  BOOST_CHECK((boost::is_same<
      typename matrix_t::transpose_t::builder_t::template view_apply_t<typename matrix_t::transpose_t::view_t>::applied_t::view_t,
      MatrixViewBase<> >::value));
  BOOST_CHECK((boost::is_same<
      typename matrix_t::builder_t::template view_apply_t<typename matrix_t::partial_t::view_t>::applied_t::view_t,
      MatrixViewSizeVariable<MatrixViewOffset<MatrixViewBase<> > > >::value));
  BOOST_CHECK((boost::is_same<
      typename matrix_t::partial_t::builder_t::template view_apply_t<typename matrix_t::partial_t::view_t>::applied_t::view_t,
      MatrixViewSizeVariable<MatrixViewOffset<MatrixViewBase<> > > >::value));
  BOOST_CHECK((boost::is_same<
      typename matrix_t::transpose_t::partial_t::builder_t::template view_apply_t<
        typename matrix_t::transpose_t::partial_t::view_t>::applied_t::view_t,
        MatrixViewSizeVariable<MatrixViewOffset<MatrixViewBase<> > > >::value));
}
BOOST_AUTO_TEST_CASE(view_inspect){
  boost::test_tools::output_test_stream os;

  os << MatrixViewProperty<matrix_t::partial_t::circular_t::builder_t::transpose_t::view_t>::inspect();
  BOOST_TEST_MESSAGE(os.str());
  BOOST_CHECK(os.is_equal("[T] [Size] [Offset] [Loop] [Offset] [Base]"));

  os << MatrixViewProperty<
      matrix_t::partial_t::circular_t::builder_t::transpose_t
        ::builder_t::view_builder_t::reverse_t>::inspect();
  BOOST_TEST_MESSAGE(os.str());
  BOOST_CHECK(os.is_equal("[Base] [Offset] [Loop] [Offset] [Size] [T]"));

  os << matrix_t::partial_t::circular_t::builder_t::transpose_t::view_t();
  BOOST_TEST_MESSAGE(os.str());
  BOOST_CHECK(os.is_equal("[T] [Size](0,0) [Offset](0,0) [Loop](0,0) [Offset](0,0) [Base]"));
}
BOOST_AUTO_TEST_CASE(transpose){
  assign_linear();
  prologue_print();
  direct_t a(*A);
  a.trans = true;
  matrix_t::transpose_t _A(A->transpose());
  BOOST_TEST_MESSAGE("t:" << _A);
  matrix_compare(a, _A);
  matrix_t __A(_A.copy());
  BOOST_TEST_MESSAGE("t.copy:" << __A);
  matrix_compare(a, __A);
  matrix_t::transpose_t::transpose_t ___A(_A.transpose()); // matrix_t::transpose_t::transpose_t = matrix_t
  BOOST_TEST_MESSAGE("t.trans:" << ___A);
  matrix_compare(*A, ___A);
}
BOOST_AUTO_TEST_CASE(conj){
  assign_linear();
  prologue_print();
  direct_t a(*A), b(*B);
  cmatrix_t::conjugate_t _AB(rAiB->conjugate());
  BOOST_TEST_MESSAGE("conj:" << _AB);

  split_r_i_t _AB_ri(_AB);
  matrix_compare(a, _AB_ri.mat_r);
  b.scalar = -1;
  matrix_compare(b, _AB_ri.mat_i);

  cmatrix_t::conjugate_t::conjugate_t __AB(_AB.conjugate()); // cmatrix_t::conjugate_t::conjugate_t = matrix_t
  BOOST_TEST_MESSAGE("conj.conj:" << __AB);
  matrix_compare(*rAiB, __AB);
}
BOOST_AUTO_TEST_CASE(adjoint){
  assign_linear();
  prologue_print();
  direct_t a(*A), b(*B);
  cmatrix_t::adjoint_t _AB(rAiB->adjoint());
  BOOST_TEST_MESSAGE("adj:" << _AB);

  split_r_i_t _AB_ri(_AB);
  a.trans = true;
  matrix_compare(a, _AB_ri.mat_r);
  b.scalar = -1; b.trans = true;
  matrix_compare(b, _AB_ri.mat_i);

  cmatrix_t::adjoint_t::adjoint_t __AB(_AB.adjoint()); // cmatrix_t::adjoint_t::adjoint_t = matrix_t
  BOOST_TEST_MESSAGE("conj.conj:" << __AB);
  matrix_compare(*rAiB, __AB);
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

BOOST_AUTO_TEST_CASE(eigen22){
  prologue_print();
  Complex<content_t> sigma[2];

  for(unsigned i(0); i < A->rows() - 1; ++i){
    A->eigen22(i, i, sigma[0], sigma[1]);
    cmatrix_t cmat(2, 2);
    cmat.replace(A->partial(2, 2, i, i));
    Complex<content_t> det[2] = {
        (cmat - sigma[0]).determinant_minor(),
        (cmat - sigma[1]).determinant_minor()};
    BOOST_CHECK_SMALL(det[0].abs(), ACCEPTABLE_DELTA_DEFAULT);
    BOOST_CHECK_SMALL(det[1].abs(), ACCEPTABLE_DELTA_DEFAULT);
  }

  for(unsigned i(0); i < rAiB->rows() - 1; ++i){
    rAiB->eigen22(i, i, sigma[0], sigma[1]);
    Complex<content_t> det[2] = {
        (rAiB->partial(2, 2, i, i) - sigma[0]).determinant_minor(),
        (rAiB->partial(2, 2, i, i) - sigma[1]).determinant_minor()};
    BOOST_CHECK_SMALL(det[0].abs(), ACCEPTABLE_DELTA_DEFAULT);
    BOOST_CHECK_SMALL(det[1].abs(), ACCEPTABLE_DELTA_DEFAULT);
  }
}

BOOST_AUTO_TEST_CASE_MAY_FAILURES(eigen, 1){
  assign_unsymmetric();
  prologue_print();
  try{
    cmatrix_t eig(A->eigen());
    BOOST_TEST_MESSAGE("eigen:" << eig);
    for(unsigned i(0); i < A->rows(); i++){
      cmatrix_t::partial_t vec(eig.partial(A->rows(), 1, 0, i));
      BOOST_CHECK_SMALL((vec.adjoint() * vec)(0, 0).abs() - 1, ACCEPTABLE_DELTA_DEFAULT);
      BOOST_TEST_MESSAGE("eigen(" << i << "):" << (vec.transpose() * A->transpose()).transpose());
      BOOST_TEST_MESSAGE("eigen(" << i << "):" << vec * eig(i, A->rows()));
      matrix_compare_delta((vec.transpose() * A->transpose()).transpose(), vec * eig(i, A->rows()), 1E-6);
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

template <class MatrixT>
void check_LU(const MatrixT &mat){
  struct pivot_t {
    unsigned num;
    unsigned *indices;
    pivot_t(const unsigned &size) : indices(new unsigned [size]) {}
    ~pivot_t(){
      delete [] indices;
    }
  } pivot(mat.rows());

  typename MatrixT::builder_t::template resize_t<0, 0, 1, 2>::assignable_t
      LU(mat.decomposeLUP(pivot.num, pivot.indices)),
      LU2(mat.decomposeLU());
  matrix_compare_delta(LU, LU2, ACCEPTABLE_DELTA_DEFAULT);
  typename MatrixT::builder_t::template resize_t<0, 0, 1, 2>::assignable_t::partial_t
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

  typename MatrixT::builder_t::assignable_t LU_multi(L * U);
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
        if(std::abs(mat(i, j) - LU_multi(i, *it)) > ACCEPTABLE_DELTA_DEFAULT){matched = false;}
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
  check_LU(*rAiB);
  assign_intermediate_zeros();
  prologue_print();
  check_LU(*A);
  check_LU(*rAiB);
}

BOOST_AUTO_TEST_CASE(QR){
  prologue_print();

  {
    matrix_t::opt_decomposeQR_t opt;
    opt.force_zeros = false;
    matrix_t QR(A->decomposeQR(opt));
    matrix_t::partial_t
        Q(QR.partial(QR.rows(), QR.rows(), 0, 0)),
        R(QR.partial(QR.rows(), QR.columns() - QR.rows(), 0, QR.rows()));
    BOOST_TEST_MESSAGE("Q:" << Q);
    BOOST_TEST_MESSAGE("R:" << R);

    matrix_compare_delta(matrix_t::getI(A->rows()), Q * Q.transpose(), ACCEPTABLE_DELTA_DEFAULT);
    for(unsigned i(1); i < R.rows(); i++){
      for(unsigned j(0); j < i; j++){
        BOOST_CHECK_SMALL(std::abs(R(i, j)), ACCEPTABLE_DELTA_DEFAULT);
      }
    }
    matrix_compare_delta(*A, Q * R, ACCEPTABLE_DELTA_DEFAULT);
  }

  {
    cmatrix_t::opt_decomposeQR_t opt;
    opt.force_zeros = false;
    cmatrix_t QR(rAiB->decomposeQR(opt));
    cmatrix_t::partial_t
        Q(QR.partial(QR.rows(), QR.rows(), 0, 0)),
        R(QR.partial(QR.rows(), QR.columns() - QR.rows(), 0, QR.rows()));
    BOOST_TEST_MESSAGE("Q:" << Q);
    BOOST_TEST_MESSAGE("R:" << R);

    matrix_compare_delta(cmatrix_t::getI(rAiB->rows()), Q * Q.adjoint(), ACCEPTABLE_DELTA_DEFAULT);
    for(unsigned i(1); i < R.rows(); i++){
      for(unsigned j(0); j < i; j++){
        BOOST_CHECK_SMALL(std::abs(R(i, j)), ACCEPTABLE_DELTA_DEFAULT);
      }
    }
    matrix_compare_delta(*rAiB, Q * R, ACCEPTABLE_DELTA_DEFAULT);
  }
}

BOOST_AUTO_TEST_CASE(UH){
  prologue_print();
  matrix_t U(matrix_t::getI(A->rows()));
  matrix_t::opt_hessenberg_t opt_H;
  opt_H.force_zeros = false;
  matrix_t H(A->hessenberg(&U, opt_H));

  BOOST_TEST_MESSAGE("hessen(H):" << H);
  for(unsigned i(2); i < H.rows(); i++){
    for(unsigned j(0); j < (i - 1); j++){
      BOOST_CHECK_SMALL(std::abs(H(i, j)), ACCEPTABLE_DELTA_DEFAULT);
    }
  }
  matrix_t _A(U * H * U.transpose());
  BOOST_TEST_MESSAGE("U * H * U^{T}:" << _A);
  matrix_compare_delta(*A, _A, ACCEPTABLE_DELTA_DEFAULT);

  matrix_t H_(A->hessenberg()); // zero clear version with default options
  matrix_compare_delta(H, H_, ACCEPTABLE_DELTA_DEFAULT);

  cmatrix_t Uc(cmatrix_t::getI(A->rows()));
  cmatrix_t::opt_hessenberg_t opt_Hc;
  opt_Hc.force_zeros = false;
  cmatrix_t Hc(rAiB->hessenberg(&Uc, opt_Hc));

  BOOST_TEST_MESSAGE("hessen(Hc):" << Hc);
  for(unsigned i(2); i < Hc.rows(); i++){
    for(unsigned j(0); j < (i - 1); j++){
      BOOST_CHECK_SMALL(Hc(i, j).abs(), ACCEPTABLE_DELTA_DEFAULT);
    }
  }

  cmatrix_t _Ac(Uc * Hc * Uc.adjoint());
  BOOST_TEST_MESSAGE("U * H * U^{*}:" << _Ac);
  matrix_compare_delta(*rAiB, _Ac, ACCEPTABLE_DELTA_DEFAULT);
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

BOOST_AUTO_TEST_CASE(iterator){
  assign_unsymmetric();
  prologue_print();
  {
    const matrix_t *_A(A);
    matrix_t::const_iterator it(std::max_element(_A->begin(), _A->end()));
#if defined(__cplusplus) && (__cplusplus >= 201103L)
    content_t
        *it_cmp_begin(std::begin((content_t (&)[SIZE * SIZE])A_array)),
        *it_cmp_end(std::end((content_t (&)[SIZE * SIZE])A_array));
#else
    content_t *it_cmp_begin(&A_array[0][0]), *it_cmp_end(&A_array[SIZE][0]);
#endif
    content_t *it_cmp(std::max_element(it_cmp_begin, it_cmp_end));
    BOOST_TEST_MESSAGE("max:" << *(it) << " @ (" << it.row() << "," << it.column() << ")");
    BOOST_CHECK_EQUAL(*it, *it_cmp);
    BOOST_CHECK_EQUAL(std::distance(_A->begin(), it), std::distance(it_cmp_begin, it_cmp));
  }
  {
    const matrix_t *_A(A);
    matrix_t::const_iterator it(std::min_element(_A->begin(), _A->end()));
#if defined(__cplusplus) && (__cplusplus >= 201103L)
    content_t
        *it_cmp_begin(std::begin((content_t (&)[SIZE * SIZE])A_array)),
        *it_cmp_end(std::end((content_t (&)[SIZE * SIZE])A_array));
#else
    content_t *it_cmp_begin(&A_array[0][0]), *it_cmp_end(&A_array[SIZE][0]);
#endif
    content_t *it_cmp(std::min_element(it_cmp_begin, it_cmp_end));
    BOOST_TEST_MESSAGE("min:" << *(it) << " @ (" << it.row() << "," << it.column() << ")");
    BOOST_CHECK_EQUAL(*it, *it_cmp);
    BOOST_CHECK_EQUAL(std::distance(_A->begin(), it), std::distance(it_cmp_begin, it_cmp));
  }
  {
    const matrix_t *_A(A);
    matrix_t __A(_A->copy());
    std::sort(__A.begin(), __A.end());
    BOOST_TEST_MESSAGE("sort:" << __A);
#if defined(__cplusplus) && (__cplusplus >= 201103L)
    BOOST_CHECK(std::all_of(__A.cbegin(), __A.cend(), [_A](const content_t &v){
      return std::find(_A->begin(), _A->end(), v) != _A->end();
    }));
#else
    for(matrix_t::const_iterator it(__A.cbegin()), it_end(__A.cend()); it != it_end; ++it){
      BOOST_CHECK(std::find(_A->begin(), _A->end(), *it) != _A->end());
    }
#endif
    for(matrix_t::const_iterator it(__A.cbegin()), it_end(__A.cend()), it2(it + 1);
        it2 != it_end; ++it, ++it2){
      BOOST_CHECK((*it) <= (*it2));
    }
  }
}

BOOST_AUTO_TEST_SUITE_END()
