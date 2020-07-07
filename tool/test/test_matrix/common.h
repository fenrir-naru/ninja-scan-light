#ifndef __COMMON_H__
#define __COMMON_H__

#include "param/complex.h"
#include "param/matrix.h"

#include <boost/version.hpp>

#include <ctime>
#include <boost/random.hpp>
#include <boost/random/random_device.hpp>

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

template <class ElementT = double>
struct Fixture {
  struct rand_t {
    boost::random::mt19937 gen;
    boost::random::normal_distribution<> dist;
    rand_t()
        : gen(static_cast<unsigned long>(time(0))), dist(0, 1.0){
    }
    rand_t(const unsigned long &seed)
        : gen(seed), dist(0, 1.0){
    }
    ElementT operator()(){
      return (ElementT)dist(gen);
    }
  } gen_rand;

  typedef Matrix<ElementT> matrix_t;
  matrix_t *A, *B;
  ElementT A_array[SIZE][SIZE], B_array[SIZE][SIZE];

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
      A_array[row][j] = (*A)(row, j) = ElementT(0);
    }
    for(unsigned int j(0); j < row; j++){
      B_array[row][j] = (*B)(row, j) = ElementT(0);
    }
  }
  void assign_linear(const int &base = 0){
    int counter(base);
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

template<class T, class Array2D_Type, class ViewType, class U>
void matrix_inspect_contains(
    const Matrix_Frozen<T, Array2D_Type, ViewType> &m,
    const U &cmp,
    const bool &is_negative = false){
  boost::test_tools::output_test_stream os;
  os << m.inspect();
  BOOST_TEST_MESSAGE(os.str());
  if(is_negative){
    BOOST_CHECK(os.str().find(cmp) == std::string::npos);
  }else{
    BOOST_CHECK(os.str().find(cmp) != std::string::npos);
  }
}

#endif /* __COMMON_H__ */
