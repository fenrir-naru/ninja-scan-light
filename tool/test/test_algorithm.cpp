#include <iostream>
#include <string>
#include <sstream>
#include <vector>

#include "algorithm/interpolate.h"

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
  double y_buf[order_max];

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
      interpolate_Neville(x_given, y_given, x, y_buf, order);
      double y2(y_buf[0]);
      double y3(interpolate_Neville<double>(x_vec, y_vec, x, order));
      double y4(interpolate_Neville<double>(x_vec.begin(), y_vec.begin(), x, order));

      BOOST_TEST_MESSAGE("(x,y,y2,y3,y4)[" << i << "] = "
          << x << ", " << y << ", " << y2 << ", " << y3 << ", " << y4);
      BOOST_CHECK_SMALL(y - y2, 1E-8);
      BOOST_CHECK_EQUAL(y2, y3);
      BOOST_CHECK_EQUAL(y2, y4);
    }
  }
}

BOOST_AUTO_TEST_SUITE_END()
