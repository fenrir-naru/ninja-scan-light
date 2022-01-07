#if defined(_MSC_VER)
#define _USE_MATH_DEFINES
#endif

#include <ctime>
#include <climits>
#include <cmath>
#include <iostream>
#include <bitset>
#include <string>
#include <algorithm>
#include <vector>

#include "navigation/GLONASS.h"

#include <boost/random.hpp>
#include <boost/random/random_device.hpp>

#include <boost/format.hpp>
#include <boost/cstdint.hpp>

#define BOOST_TEST_MAIN
#include <boost/test/included/unit_test.hpp>

using namespace std;
using boost::format;

typedef GLONASS_SpaceNode<double> space_node_t;

BOOST_AUTO_TEST_SUITE(GLONASS)

struct Fixture {
  boost::random::mt19937 gen;
  boost::random::uniform_int_distribution<> bin_dist;

  Fixture()
      : gen(0), //static_cast<unsigned long>(time(0))
      bin_dist(0, 1)
      {}
  ~Fixture(){}

  bool get_bool(){
    return bin_dist(gen) == 1;
  }
};

BOOST_AUTO_TEST_CASE(ICD_CDMA_2016_Appendix_K){
  space_node_t::TimeProperties::raw_t raw = {0};
  raw.N_4 = 5;
  raw.NA = 251;
  space_node_t::TimeProperties t(raw);
  std::tm t_tm(t.date.c_tm());
  BOOST_REQUIRE_EQUAL(t_tm.tm_year, 2012 - 1900);
  BOOST_REQUIRE_EQUAL(t_tm.tm_mon, 9 - 1);
  BOOST_REQUIRE_EQUAL(t_tm.tm_mday, 7);

  { // additional tests to Appendix.K
    space_node_t::TimeProperties::date_t date(
        space_node_t::TimeProperties::date_t::from_c_tm(t_tm));
    BOOST_REQUIRE_EQUAL(t.date.year, date.year);
    BOOST_REQUIRE_EQUAL(t.date.day_of_year, date.day_of_year);
  }

  double jd0(space_node_t::TimeProperties::date_t::julian_day(t_tm));
  BOOST_REQUIRE_CLOSE(jd0, 2456177.5, 1E-8);

  double gmst_deg(std::fmod(
      space_node_t::TimeProperties::date_t::Greenwich_sidereal_time_deg(t_tm),
      360));
  BOOST_REQUIRE_CLOSE(
      gmst_deg, std::fmod(29191.442830, M_PI * 2) / M_PI * 180, 1E-3);
}

BOOST_AUTO_TEST_SUITE_END()
