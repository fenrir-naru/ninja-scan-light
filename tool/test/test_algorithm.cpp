#include <iostream>
#include <string>
#include <sstream>
#include <vector>

#include "algorithm/interpolate.h"
#include "param/matrix.h"

#include <boost/version.hpp>

#include <ctime>
#include <boost/random.hpp>
#include <boost/random/random_device.hpp>

#include <boost/format.hpp>
#include <boost/cstdint.hpp>
#include <boost/xpressive/xpressive.hpp>

#define BOOST_TEST_MAIN
#include <boost/test/included/unit_test.hpp>

using namespace std;
using boost::format;

struct Fixture {
  struct rand_t {
    boost::random::mt19937 gen;
    boost::random::uniform_real_distribution<> dist;
    rand_t()
        : gen(static_cast<unsigned long>(time(0))), dist(0, 1){
    }
    rand_t(const unsigned long &seed)
        : gen(seed), dist(0, 1){
    }
    double operator()(){
      return dist(gen);
    }
  } gen_rand;
  Fixture() : gen_rand() {}
  ~Fixture(){}
};

BOOST_FIXTURE_TEST_SUITE(algorithm, Fixture)

BOOST_AUTO_TEST_CASE(interpolate_neville){
  static const int order_max(9);

  double coef[order_max + 1];
  double x_given[order_max + 1], y_given[order_max + 1];
  double y_buf[order_max], dy_buf[order_max][order_max];

  for(int order(0); order <= order_max; ++order){
    for(int i(0); i <= order; ++i){
      coef[i] = gen_rand();
      BOOST_TEST_MESSAGE("coef[" << i << "] = " << coef[i]);
    }

    // generate (x, y)_given
    for(int i(0); i <= order; ++i){
      x_given[i] = gen_rand();
      y_given[i] = 0;
      for(int j(0); j <= order; ++j){
        y_given[i] += coef[j] * std::pow(x_given[i], j);
      }
      BOOST_TEST_MESSAGE("(x,y)_given[" << i << "] = " << x_given[i] << ", " << y_given[i]);
    }

    { // (additional) solution by using linear algebra
      Matrix<double> x_mat(order + 1, order + 1), y_mat(order + 1, 1);
      for(int i(0); i <= order; ++i){
        for(int j(0); j <= order; ++j){
          x_mat(i, j) = std::pow(x_given[i], j);
          y_mat(i, 0) = y_given[i];
        }
      }
      Matrix<double> coef_mat(x_mat.inverse() * y_mat);
      for(int i(0); i <= order; ++i){
        if(std::abs(coef[i]) < 1E-8){
          BOOST_WARN_SMALL(std::abs(coef_mat(i, 0)), 1E-8);
        }else{
          BOOST_WARN_CLOSE(coef_mat(i, 0), coef[i], 1E-2); // 0.01%
        }
      }
    }

    // vector version
    std::vector<double> x_vec, y_vec;
    for(int i(0); i <= order; ++i){
      x_vec.push_back(x_given[i]);
      y_vec.push_back(y_given[i]);
    }

    // check new (x, y)
    for(int i(0); i < 10; ++i){
      double x(gen_rand());

      double y(0); // truth
      for(int j(0); j <= order; ++j){
        y += coef[j] * std::pow(x, j);
      }
      double y2(interpolate_Neville(x_given, y_given, x, y_buf, order)[0]);
      double y3(interpolate_Neville<double>(x_vec, y_vec, x, order));
      double y4(interpolate_Neville<double>(x_vec.begin(), y_vec.begin(), x, order));

      BOOST_TEST_MESSAGE(
          "(x,y,y2,y3,y4)[" << i << "] = "
          << x << ", " << y << ", " << y2 << ", " << y3 << ", " << y4);
      BOOST_CHECK_SMALL(y - y2, 1E-8);
      BOOST_CHECK_EQUAL(y2, y3);
      BOOST_CHECK_EQUAL(y2, y4);

      // derivatives
      interpolate_Neville(x_given, y_given, x, y_buf, order, dy_buf, order);
      for(int j(1); j <= order; ++j){
        double dy(0); // truth
        for(int k(j); k <= order; ++k){
          int fact(1); // n!
          for(int k2(k); k2 > (k - j); k2--){fact *= k2;}
          dy += coef[k] * std::pow(x, k - j) * fact;
        }
        BOOST_TEST_MESSAGE(
            "(x,d(" << j << ")y,d(" << j << ")y2)[" << i << "] = "
            << x << ", " << dy << ", " << dy_buf[j - 1][0]);
        BOOST_CHECK_CLOSE(dy, dy_buf[j - 1][0], 1E-2);
      }
    }
  }
}

BOOST_AUTO_TEST_CASE(interpolatable_set){
  typedef InterpolatableSet<int, int> target_t;
  target_t target;
  std::map<int, int> src;
  for(int i(0); i < 10; ++i){
    src[i * 10] = 0;
  }

  target_t::condition_t cnd = {5, 100};

  // case: -10=x0, 0, 10, 20, 30, 40
  target.update(-10, src, cnd);
  for(std::size_t i(0); i < cnd.max_x_size; ++i){
    BOOST_CHECK(find(target.dx.begin(), target.dx.end(), 10 + i * 10) != target.dx.end());
  }
  BOOST_CHECK_EQUAL(target.x_lower, -10);
  BOOST_CHECK_EQUAL(target.x_upper, 5);

  // case: 0, 2=x0, 10, 20, 30, 40
  target.update(2, src, cnd);
  for(std::size_t i(0); i < cnd.max_x_size; ++i){
    BOOST_CHECK(find(target.dx.begin(), target.dx.end(), 10 + i * 10) != target.dx.end());
  }
  BOOST_CHECK_EQUAL(target.x_lower, -10);
  BOOST_CHECK_EQUAL(target.x_upper, 5);

  // case: 0, 10, 20, 22=x0, 30, 40
  target.update(22, src, cnd);
  for(std::size_t i(0); i < cnd.max_x_size; ++i){
    BOOST_CHECK(find(target.dx.begin(), target.dx.end(), -22 + i * 10) != target.dx.end());
  }
  BOOST_CHECK_EQUAL(target.x_lower, 20);
  BOOST_CHECK_EQUAL(target.x_upper, 25);

  // case: 10, 20, 28=x0, 30, 40, 50
  target.update(28, src, cnd);
  for(std::size_t i(0); i < cnd.max_x_size; ++i){
    BOOST_CHECK(find(target.dx.begin(), target.dx.end(), -28 + i * 10 + 10) != target.dx.end());
  }
  BOOST_CHECK_EQUAL(target.x_lower, 25);
  BOOST_CHECK_EQUAL(target.x_upper, 30);

  // case: 50, 60, 70, 80, 88=x0, 90
  target.update(88, src, cnd);
  for(std::size_t i(0); i < cnd.max_x_size; ++i){
    BOOST_CHECK(find(target.dx.begin(), target.dx.end(), -88 + i * 10 + 50) != target.dx.end());
  }
  BOOST_CHECK_EQUAL(target.x_lower, 85);
  BOOST_CHECK_EQUAL(target.x_upper, 88);

  // case: 50, 60, 70, 80, 90, 100=x0
  target.update(100, src, cnd);
  for(std::size_t i(0); i < cnd.max_x_size; ++i){
    BOOST_CHECK(find(target.dx.begin(), target.dx.end(), -100 + i * 10 + 50) != target.dx.end());
  }
  BOOST_CHECK_EQUAL(target.x_lower, 85);
  BOOST_CHECK_EQUAL(target.x_upper, 100);

  // case: 0, 2=x0, 10, 20, 30, 40
  target.update(2, src, cnd);
  for(std::size_t i(0); i < cnd.max_x_size; ++i){
    BOOST_CHECK(find(target.dx.begin(), target.dx.end(), -2 + i * 10) != target.dx.end());
  }
  BOOST_CHECK_EQUAL(target.x_lower, 2);
  BOOST_CHECK_EQUAL(target.x_upper, 5);
}

BOOST_AUTO_TEST_SUITE_END()
