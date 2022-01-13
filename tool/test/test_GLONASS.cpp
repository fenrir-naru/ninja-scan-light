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

BOOST_AUTO_TEST_CASE(ICD_CDMA_2016_Appendix_J){
  space_node_t::SatelliteProperties::Ephemeris eph;
  space_node_t::TimeProperties t;
  {
    space_node_t::SatelliteProperties::Ephemeris::raw_t eph_raw = {0};
    space_node_t::TimeProperties::raw_t t_raw = {0};

    eph_raw.N_T = t_raw.NA = 251;
    t_raw.N_4 = 5;

    eph = (space_node_t::SatelliteProperties::Ephemeris)eph_raw;
    t = (space_node_t::TimeProperties)t_raw;

    eph.t_b = 11700;
    eph.xn = 7003.008789E3; eph.yn = -12206.626953E3; eph.zn = 21280.765625E3;
    eph.xn_dot = 0.7835417E3; eph.yn_dot = 2.8042530E3; eph.zn_dot = 1.3525150E3;
    eph.xn_ddot = 0; eph.yn_ddot = 1.7E-6; eph.zn_ddot = -5.41E-6;
  }

  space_node_t::SatelliteProperties::Ephemeris_with_Time eph_t(eph, t);

  space_node_t::SatelliteProperties::Ephemeris::constellation_t pos_vel(
      eph_t.calculate_constellation(12300 - 11700));

  // precise (J.1)
  // 7523.174819 km, -10506.961965 km, 21999.239413 km
  // 0.950126007 km/s, 2.855687825 km/s, 1.040679862 km/s
  // Jx0m = -5.035590E-10 km/s2, Jy0m = 7.379024E-10 km/s2, Jz0m = -1.648033E-9 km/s2
  // Jx0s = 4.428827E-10 km/s2, Jy0s = 3.541631E-10 km/s2, Jz0s = -8.911601E-10 km/s2

  // simplified (J.2)
  // 7523.174853 km, -10506.962176 km, 21999.239866 km
  // 0.95012609 km/s, 2.85568710 km/s, 1.04068137 km/s

  BOOST_REQUIRE_CLOSE(pos_vel.position[0], 7523.174819E3, 1E0); // 1m
  BOOST_REQUIRE_CLOSE(pos_vel.position[1], -10506.961965E3, 1E0);
  BOOST_REQUIRE_CLOSE(pos_vel.position[2], 21999.239413E3, 1E0);

  BOOST_REQUIRE_CLOSE(pos_vel.velocity[0], 0.950126007E3, 1E-1); // 0.1m/s
  BOOST_REQUIRE_CLOSE(pos_vel.velocity[1], 2.855687825E3, 1E-1);
  BOOST_REQUIRE_CLOSE(pos_vel.velocity[2], 1.040679862E3, 1E-1);
}

BOOST_AUTO_TEST_CASE(ICD_CDMA_2016_Appendix_J_sun_moon){
  // TODO check precise method, J_m, J_s differences are not negligible.

  double t(11700); // [s]

  space_node_t::SatelliteProperties::Ephemeris::constellation_t pos_vel = {
    {7523.174819E3, -10506.961965E3, 21999.239413E3},
    {0.950126007E3, 2.855687825E3, 1.040679862E3},
  };

  space_node_t::SatelliteProperties::Ephemeris::constellation_t pos_vel_abs(
      pos_vel.abs_corrdinate(
        29191.442830 + M_PI * 2 * (t - 10800) / 86400)); // PZ-90 => O-XYZ

  space_node_t::SatelliteProperties::Ephemeris::differential2_t diff2_2000 = {
    space_node_t::SatelliteProperties::Ephemeris::lunar_solar_perturbations_t::base2000(
        2456177.5, t),
  };

  double Jm_2000[3], Js_2000[3];
  diff2_2000.calculate_Jm(pos_vel_abs.position, Jm_2000);
  diff2_2000.calculate_Js(pos_vel_abs.position, Js_2000);

  space_node_t::SatelliteProperties::Ephemeris::differential2_t diff2_1975 = {
    space_node_t::SatelliteProperties::Ephemeris::lunar_solar_perturbations_t::base1975(
        13704, t), // 13704 equals to interval days between 2012/7/9 and 1975/1/1
  };

  double Jm_1975[3], Js_1975[3];
  diff2_1975.calculate_Jm(pos_vel_abs.position, Jm_1975);
  diff2_1975.calculate_Js(pos_vel_abs.position, Js_1975);
}

BOOST_AUTO_TEST_SUITE_END()
