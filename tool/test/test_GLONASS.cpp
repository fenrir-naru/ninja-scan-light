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

struct rtklib_t {
  // https://github.com/tomojitakasu/RTKLIB/blob/03fb5a2da9ad4294a262b6ed8c4287dafdef4a74/src/rcvraw.c#L405
  static int test_glostr(const unsigned char *buff){
    static const unsigned char xor_8bit[256]={ /* xor of 8 bits */
      0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,
      1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,
      1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,
      0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,
      1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,
      0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,
      0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,
      1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0
    };
    static const unsigned char mask_hamming[][11]={ /* mask of hamming codes */
      {0x55,0x55,0x5A,0xAA,0xAA,0xAA,0xB5,0x55,0x6A,0xD8,0x08},
      {0x66,0x66,0x6C,0xCC,0xCC,0xCC,0xD9,0x99,0xB3,0x68,0x10},
      {0x87,0x87,0x8F,0x0F,0x0F,0x0F,0x1E,0x1E,0x3C,0x70,0x20},
      {0x07,0xF8,0x0F,0xF0,0x0F,0xF0,0x1F,0xE0,0x3F,0x80,0x40},
      {0xF8,0x00,0x0F,0xFF,0xF0,0x00,0x1F,0xFF,0xC0,0x00,0x80},
      {0x00,0x00,0x0F,0xFF,0xFF,0xFF,0xE0,0x00,0x00,0x01,0x00},
      {0xFF,0xFF,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x00},
      {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xF8}
    };
    unsigned char cs; /* checksum */
    int i,j,n=0;
    for (i=0;i<8;i++) {
      for (j=0,cs=0;j<11;j++) {
        cs^=xor_8bit[buff[j]&mask_hamming[i][j]];
      }
      if (cs) n++;
    }
    return n==0||(n==2&&cs);
  }
  template <std::size_t N>
  static bool test_glostr(const boost::uint32_t (&buff)[N]){
    unsigned char temp[4 * N];
    for(std::size_t i(0), j(0); i < N; ++i){
      temp[j++] = (buff[i] >> 24) & 0xFF;
      temp[j++] = (buff[i] >> 16) & 0xFF;
      temp[j++] = (buff[i] >> 8) & 0xFF;
      temp[j++] = buff[i] & 0xFF;
    }
    return test_glostr(temp) != 0;
  }
};

BOOST_AUTO_TEST_CASE(dump_with_parity_check){
  typedef boost::uint32_t u32_t;
  static const u32_t packet[][4] = {
    // UBX 20210531/log.ubx
    {0x7839e8b8, 0x0d34c028, 0x97930800, 0x07a10002},
    {0x7839e8b8, 0x0d34c028, 0x979f1000, 0x07a10002},
    {0x08255533, 0x9cd061d6, 0x45355000, 0x07a10003},
    {0x082550fd, 0xd9f0f3cc, 0xf9511000, 0x07a10003},
    {0x08255521, 0x19e8af12, 0x4845b800, 0x07a10003},
    {0x08255400, 0x5c0136ff, 0x29fd9000, 0x07a10003},
    {0x08255407, 0x34d00255, 0x00ac6800, 0x07a10003},
    {0x08255023, 0x619865f3, 0xc02d0800, 0x07a10003},
    {0x10a500c4, 0x4111d648, 0xa8abe000, 0x07a10003},
    {0x10a504d0, 0xf090a084, 0x008eb000, 0x07a10003},
    {0x10a50016, 0xe050499e, 0x57b3c800, 0x07a10003},
    // UBX f9p_ppp_1224/rover.ubx
    {0x654c07e3, 0xc5804cec, 0x2ed2f000, 0x0b4c0002},
    {0x654c09e3, 0xc5804cec, 0x2ed71000, 0x0b4c0002},
    {0x6f7c185c, 0xf774bf87, 0xc9e0c000, 0x0b4c0002},
    {0x755022ff, 0x49008c38, 0x36413000, 0x0b4c0002},
    {0x783990f3, 0xeb34bfe9, 0xcb95b800, 0x0b4c0002},
    {0x0801cc1b, 0x240c273a, 0x659d1000, 0x0b4c0003},
    {0x0801c954, 0xaf28622b, 0x0e39a800, 0x0b4c0003},
    {0x0801c8a3, 0xe6e470fb, 0xb1206800, 0x0b4c0003},
    {0x0801cc10, 0xb5342aa5, 0xc960f000, 0x0b4c0003},
    {0x0801cc12, 0xdfc80221, 0x76ec1000, 0x0b4c0003},
    {0x0801cd02, 0x905912b3, 0x78748000, 0x0b4c0003},
  };
  for(std::size_t i(0); i < sizeof(packet) / sizeof(packet[0]); ++i){
    bool res_rtklib(rtklib_t::test_glostr(packet[i]));
    BOOST_CHECK(res_rtklib);

    typedef space_node_t::BroadcastedMessage<u32_t> msg_t;
    u32_t copy[4];
    std::memcpy(copy, packet[i], sizeof(copy));
#define pingpong(str_num, key) { \
  u32_t copy2[4] = {0}; \
  msg_t::String ## str_num::key ## _set(copy2, msg_t::String ## str_num::key(copy)); \
  BOOST_REQUIRE_EQUAL(msg_t::String ## str_num::key(copy), msg_t::String ## str_num::key(copy2)); \
}
    pingpong(1, P1);
    pingpong(1, t_k);
    pingpong(1, xn_dot);
    pingpong(1, xn_ddot);
    pingpong(1, xn);
    pingpong(2, B_n);
    pingpong(2, P2);
    pingpong(2, t_b);
    pingpong(2, yn_dot);
    pingpong(2, yn_ddot);
    pingpong(2, yn);
    pingpong(3, P3);
    pingpong(3, gamma_n);
    pingpong(3, p);
    pingpong(3, l_n);
    pingpong(3, zn_dot);
    pingpong(3, zn_ddot);
    pingpong(3, zn);
    pingpong(4, tau_n);
    pingpong(4, delta_tau_n);
    pingpong(4, E_n);
    pingpong(4, P4);
    pingpong(4, F_T);
    pingpong(4, N_T);
    pingpong(4, n);
    pingpong(4, M);
    pingpong(5_Almanac, NA);
    pingpong(5_Almanac, tau_c);
    pingpong(5_Almanac, N_4);
    pingpong(5_Almanac, tau_GPS);
    pingpong(5_Almanac, l_n);
    pingpong(6_Almanac, C_n);
    pingpong(6_Almanac, M_n);
    pingpong(6_Almanac, nA);
    pingpong(6_Almanac, tauA_n);
    pingpong(6_Almanac, lambdaA_n);
    pingpong(6_Almanac, delta_iA_n);
    pingpong(6_Almanac, epsilonA_n);
    pingpong(7_Almanac, omegaA_n);
    pingpong(7_Almanac, tA_lambda_n);
    pingpong(7_Almanac, delta_TA_n);
    pingpong(7_Almanac, delta_TA_dot_n);
    pingpong(7_Almanac, HA_n);
    pingpong(7_Almanac, l_n);
#undef pingpong
    msg_t::KX_set(copy, 0);
    msg_t::KX_set(copy);
    BOOST_CHECK_EQUAL(msg_t::KX(packet[i]), msg_t::KX(copy));
  }
}

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
