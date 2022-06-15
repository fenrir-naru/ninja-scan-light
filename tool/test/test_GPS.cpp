#include <ctime>
#include <climits>
#include <iostream>
#include <bitset>
#include <string>
#include <algorithm>
#include <vector>

#include "navigation/GPS.h"
#include "navigation/GPS_Solver_Base.h"

#include <boost/random.hpp>
#include <boost/random/random_device.hpp>

#include <boost/format.hpp>
#include <boost/cstdint.hpp>

#define BOOST_TEST_MAIN
#include <boost/test/included/unit_test.hpp>

using namespace std;
using boost::format;

typedef GPS_SpaceNode<double> space_node_t;
typedef GPS_Solver_Base<double> solver_base_t;

BOOST_AUTO_TEST_SUITE(GPS)

template <int PaddingMSB, int PaddingLSB,
    size_t N_bitset, class BufferT, size_t N_buf>
void fill_buffer(const bitset<N_bitset> &b, BufferT (&buf)[N_buf]){
  static const int effectiveBits((int)sizeof(BufferT) * CHAR_BIT - PaddingMSB - PaddingLSB);
  static const int N((N_bitset + effectiveBits - 1) / effectiveBits);
  BOOST_REQUIRE_LE(N, (int)N_buf);
  unsigned int b_index(0);
  for(int i(0); i < (N - 1); ++i){
    //buf[i] = 0;
    for(int j(0); j < effectiveBits; ++j){
      buf[i] <<= 1;
      buf[i] |= (b[b_index++] ? 1 : 0);
    }
    (PaddingLSB >= 0) ? (buf[i] <<= PaddingLSB) : (buf[i] >>= -PaddingLSB);
  }
  { // Last
    //buf[N - 1] = 0;
    while(b_index < N_bitset){
      buf[N - 1] <<= 1;
      buf[N - 1] |= (b[b_index++] ? 1 : 0);
    }
    static const int lash_shift((N_bitset % effectiveBits == 0)
        ? PaddingLSB
        : (effectiveBits - ((int)N_bitset % effectiveBits) + PaddingLSB));
    (lash_shift >= 0) ? (buf[N - 1] <<= lash_shift) : (buf[N - 1] >>= -lash_shift);
  }


  BOOST_TEST_MESSAGE("Buffer =>");
  for(int i(0); i < N; ++i){ // debug print
    std::string msg;
    BufferT buf2(buf[i]);
    for(int j(sizeof(BufferT) * CHAR_BIT); j > 0; j -= 8){
      msg.insert(0, std::bitset<8>((unsigned long long)(buf2 & 0xFF)).to_string());
      msg.insert(0, " ");
      buf2 >>= 8;
    }
    BOOST_TEST_MESSAGE(msg);
  }
}

template <int PaddingMSB, int PaddingLSB,
    size_t N_bitset, class BufferT>
void check_parse(const bitset<N_bitset> &b, const BufferT *buf){
  typedef space_node_t::BroadcastedMessage<
      BufferT, (int)sizeof(BufferT) * CHAR_BIT - PaddingMSB - PaddingLSB, PaddingMSB> msg_t;

#define each2(offset, bits, shift, func) { \
  boost::uint32_t res((boost::uint32_t)msg_t::func(buf)); \
  res >>= shift; \
  BOOST_TEST_MESSAGE(format("%s => 0x%08x") % #func % res); \
  for(int i(0), j(offset + bits - 1); i < bits; ++i, --j, res >>= 1){ \
    BOOST_REQUIRE_EQUAL(b[j], ((res & 0x1) == 1)); \
  } \
}
#define each(offset, bits, func) each2(offset, bits, 0, func)
  each(   0,  8, preamble);
  each(  30, 24, how);
  each(  49,  3, subframe_id);

  each(  60, 10, SubFrame1::WN);
  each(  72,  4, SubFrame1::URA);
  each(  76,  6, SubFrame1::SV_health);
  each2( 82,  2, 8, SubFrame1::iodc);
  each( 196,  8, SubFrame1::t_GD);
  each( 210,  8, SubFrame1::iodc);
  each( 218, 16, SubFrame1::t_oc);
  each( 240,  8, SubFrame1::a_f2);
  each( 248, 16, SubFrame1::a_f1);
  each( 270, 22, SubFrame1::a_f0);

  each(  60,  8, SubFrame2::iode);
  each(  68, 16, SubFrame2::c_rs);
  each(  90, 16, SubFrame2::delta_n);
  each2(106,  8, 24, SubFrame2::M0);
  each( 120, 24, SubFrame2::M0);
  each( 150, 16, SubFrame2::c_uc);
  each2(166,  8, 24, SubFrame2::e);
  each( 180, 24, SubFrame2::e);
  each( 210, 16, SubFrame2::c_us);
  each2(226,  8, 24, SubFrame2::sqrt_A);
  each( 240, 24, SubFrame2::sqrt_A);
  each( 270, 16, SubFrame2::t_oe);
  each( 286,  1, SubFrame2::fit);

  each(  60, 16, SubFrame3::c_ic);
  each2( 76,  8, 24, SubFrame3::Omega0);
  each(  90, 24, SubFrame3::Omega0);
  each( 120, 16, SubFrame3::c_is);
  each2(136,  8, 24, SubFrame3::i0);
  each( 150, 24, SubFrame3::i0);
  each( 180, 16, SubFrame3::c_rc);
  each2(196,  8, 24, SubFrame3::omega);
  each( 210, 24, SubFrame3::omega);
  each( 240, 24, SubFrame3::dot_Omega0);
  each( 270,  8, SubFrame3::iode);
  each( 278, 14, SubFrame3::dot_i0);

  each(  62,  6, sv_page_id);

  each(  68, 16, SubFrame4_5_Alnamac::e);
  each(  90,  8, SubFrame4_5_Alnamac::t_oa);
  each(  98, 16, SubFrame4_5_Alnamac::delta_i);
  each( 120, 16, SubFrame4_5_Alnamac::dot_Omega0);
  each( 128,  8, SubFrame4_5_Alnamac::SV_health);
  each( 150, 24, SubFrame4_5_Alnamac::sqrt_A);
  each( 180, 24, SubFrame4_5_Alnamac::Omega0);
  each( 210, 24, SubFrame4_5_Alnamac::omega);
  each( 240, 24, SubFrame4_5_Alnamac::M0);
  each2(270,  8,  3, SubFrame4_5_Alnamac::a_f0);
  each( 289,  3, SubFrame4_5_Alnamac::a_f0);
  each( 278, 11, SubFrame4_5_Alnamac::a_f1);

  each(  68,  8, SubFrame4_Page18::alpha0);
  each(  76,  8, SubFrame4_Page18::alpha1);
  each(  90,  8, SubFrame4_Page18::alpha2);
  each(  98,  8, SubFrame4_Page18::alpha3);
  each( 106,  8, SubFrame4_Page18::beta0);
  each( 120,  8, SubFrame4_Page18::beta1);
  each( 128,  8, SubFrame4_Page18::beta2);
  each( 136,  8, SubFrame4_Page18::beta3);
  each( 150, 24, SubFrame4_Page18::A1);
  each2(180, 24,  8, SubFrame4_Page18::A0);
  each( 210,  8, SubFrame4_Page18::A0);
  each( 218,  8, SubFrame4_Page18::t_ot);
  each( 240,  8, SubFrame4_Page18::delta_t_LS);
  each( 226,  8, SubFrame4_Page18::WN_t);
  each( 248,  8, SubFrame4_Page18::WN_LSF);
  each( 256,  8, SubFrame4_Page18::DN);
  each( 270,  8, SubFrame4_Page18::delta_t_LSF);
#undef each
#undef each2
}

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

BOOST_FIXTURE_TEST_CASE(data_parse, Fixture){
  typedef boost::uint8_t u8_t;
  typedef boost::uint32_t u32_t;

  for(int loop(0); loop < 0x1000; loop++){
    bitset<300> b;
    for(unsigned int i(0); i < b.size(); ++i){
      b.set(i, get_bool());
    }
    string b_str(b.to_string());
    reverse(b_str.begin(), b_str.end());
    BOOST_TEST_MESSAGE(format("Origin(%d) => ") % loop);
    for(int i(0); i < 300; i += 30){
      BOOST_TEST_MESSAGE(format("%3d ") % i << b_str.substr(i, 24) << ' ' << b_str.substr(i + 24, 6));
    }

    static const int u8_bits(sizeof(u8_t) * CHAR_BIT);
    {
      BOOST_TEST_MESSAGE("u8_t container without padding");
      u8_t buf[(300 + u8_bits - 1) / u8_bits];
      fill_buffer<0, 0>(b, buf);
      check_parse<0, 0>(b, buf);
    }
    {
      BOOST_TEST_MESSAGE("u8_t container with padding (2, 0)");
      u8_t buf[(300 + u8_bits - 2 - 1) / (u8_bits - 2)];
      fill_buffer<2, 0>(b, buf);
      check_parse<2, 0>(b, buf);
    }
    {
      BOOST_TEST_MESSAGE("u8_t container with padding (0, 2)");
      u8_t buf[(300 + u8_bits - 2 - 1) / (u8_bits - 2)];
      fill_buffer<0, 2>(b, buf);
      check_parse<0, 2>(b, buf);
    }
    {
      BOOST_TEST_MESSAGE("u8_t container with padding (2, 2)");
      u8_t buf[(300 + u8_bits - 4 - 1) / (u8_bits - 4)];
      fill_buffer<2, 2>(b, buf);
      check_parse<2, 2>(b, buf);
    }
    static const int u32_bits(sizeof(u32_t) * CHAR_BIT);
    {
      BOOST_TEST_MESSAGE("u32_t container without padding");
      u32_t buf[(300 + u32_bits - 1) / u32_bits];
      fill_buffer<0, 0>(b, buf);
      check_parse<0, 0>(b, buf);
    }
    {
      BOOST_TEST_MESSAGE("u32_t container with padding (2, 0)");
      u32_t buf[(300 + u32_bits - 2 - 1) / (u32_bits - 2)];
      fill_buffer<2, 0>(b, buf);
      check_parse<2, 0>(b, buf);
    }
    {
      BOOST_TEST_MESSAGE("u32_t container with padding (0, 2)");
      u32_t buf[(300 + u32_bits - 2 - 1) / (u32_bits - 2)];
      fill_buffer<0, 2>(b, buf);
      check_parse<0, 2>(b, buf);
    }
    {
      BOOST_TEST_MESSAGE("u32_t container with padding (2, 2)");
      u32_t buf[(300 + u32_bits - 4 - 1) / (u32_bits - 4)];
      fill_buffer<2, 2>(b, buf);
      check_parse<2, 2>(b, buf);
    }

    { // special case for u-blox 6 RXM-EPH
      BOOST_TEST_MESSAGE("u32_t container with padding (8, -6)");
      u32_t buf[(300 + u32_bits - 2 - 1) / (u32_bits - 2)];
      fill_buffer<8, -6>(b, buf);
      check_parse<8, -6>(b, buf);
    }
  }
}

BOOST_AUTO_TEST_CASE(gps_time){
  typedef space_node_t::gps_time_t gpst_t;
  { // roll over check
    gpst_t t(1000, 5);
    BOOST_REQUIRE_EQUAL((t - 10).week, t.week - 1);
    BOOST_REQUIRE_CLOSE((t - 10).seconds, t.seconds - 10 + gpst_t::seconds_week, 1E-8);
    BOOST_REQUIRE_EQUAL(((t - 10) + 10).week, t.week);
    BOOST_REQUIRE_CLOSE(((t - 10) + 10).seconds, t.seconds, 1E-8);
  }
  for(const gpst_t::leap_second_event_t *t(&gpst_t::leap_second_events[0]); t->leap_seconds > 0; ++t){
    gpst_t t_gps(t->uncorrected.week, t->uncorrected.seconds);
    std::tm t_tm(t_gps.c_tm()); // tm => gps_time => tm
    BOOST_REQUIRE_EQUAL(t_tm.tm_year, t->tm_year);
    BOOST_REQUIRE_EQUAL(t_tm.tm_mon, t->tm_mon);
    BOOST_REQUIRE_EQUAL(t_tm.tm_mday, t->tm_mday);
    BOOST_REQUIRE_EQUAL(gpst_t::guess_leap_seconds(t_tm), t->leap_seconds);
    BOOST_REQUIRE_EQUAL(gpst_t::guess_leap_seconds(t_gps), t->leap_seconds);
  }
  {
    // UTC, julian date
    std::tm tm2000 = {0, 0, 12, 1, 0, 2000 - 1900};
    gpst_t t2000(tm2000, 13);
    BOOST_REQUIRE_EQUAL(t2000.leap_seconds(), 13);
    {
      std::tm utc(t2000.utc());
      BOOST_REQUIRE_SMALL(std::abs(
          std::difftime(std::mktime(&utc), std::mktime(&tm2000))), 1E-8);
    }

    BOOST_REQUIRE_CLOSE(gpst_t(0, 0).julian_date(), 2444244.5, 1E-8);
    BOOST_REQUIRE_CLOSE(t2000.julian_date(), 2451545.0, 1E-8);
    BOOST_REQUIRE_SMALL(std::abs(t2000.julian_date_2000()), 1E-8);
  }
}

BOOST_AUTO_TEST_CASE(satellite_ephemeris){
  typedef space_node_t::Satellite sat_t;
  sat_t sat;
  for(int i(1); i < 3; ++i){
    sat_t::eph_t eph;
    eph.WN = 2000;
    eph.t_oc = eph.t_oe = (2 * i + 1) * 60 * 60; // every 2 hr; 3, 5
    eph.fit_interval = 4 * 60 * 60; // 4 hr
    sat.register_ephemeris(eph);
  }

  struct {
    space_node_t::float_t hr_in, hr_out;
  } target[] = {
    {0, 0},
    {1, 3}, {2, 3}, {2.9, 3}, {3, 3}, {3.9, 3}, {4, 3},
    {4.1, 5}, {5, 5}, {6, 5}, {7, 5},
  };
  for(unsigned int i(0); i < sizeof(target) / sizeof(target[0]); ++i){
    sat.select_ephemeris(space_node_t::gps_time_t(2000, target[i].hr_in * 60 * 60));
    BOOST_TEST_MESSAGE(target[i].hr_in);
    BOOST_REQUIRE_EQUAL(sat.ephemeris().t_oc, target[i].hr_out * 60 * 60);
  }
}

BOOST_AUTO_TEST_SUITE_END()
