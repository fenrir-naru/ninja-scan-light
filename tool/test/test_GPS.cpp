#include <ctime>
#include <iostream>
#include <bitset>
#include <string>
#include <algorithm>

#include "navigation/GPS.h"

#include <boost/random.hpp>
#include <boost/random/random_device.hpp>

#include <boost/format.hpp>
#include <boost/cstdint.hpp>

#define BOOST_TEST_MAIN
#include <boost/test/included/unit_test.hpp>

using namespace std;
using boost::format;

typedef GPS_SpaceNode<double> space_node_t;

BOOST_AUTO_TEST_SUITE(GPS)

template <int Padding, size_t N_bitset, class BufferT, size_t N>
void fill(const bitset<N_bitset> &b, BufferT (&buf)[N]){
  int b_index(0);
  for(int i(0); i < (N - 1); ++i){
    //buf[i] = 0;
    for(int j(0); j < (sizeof(BufferT) * 8 - Padding); ++j){
      buf[i] <<= 1;
      buf[i] |= (b[b_index++] ? 1 : 0);
    }
  }
  // Last
  {
    int j(0);
    //buf[N - 1] = 0;
    for(; b_index < b.size(); ++j){
      buf[N - 1] <<= 1;
      buf[N - 1] |= (b[b_index++] ? 1 : 0);
    }
    buf[N - 1] <<= (sizeof(BufferT) * 8 - Padding - j);
  }
}

template <int Padding, size_t N_bitset, class BufferT>
void check(const bitset<N_bitset> &b, const BufferT *buf){
  typedef space_node_t::BroadcastedMessage<
      BufferT, sizeof(BufferT) * 8 - Padding, Padding> msg_t;

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

BOOST_AUTO_TEST_CASE(data_parse){
  namespace br = boost::random;
  br::mt19937 gen(0); //static_cast<unsigned long>(time(0)));
  br::uniform_int_distribution<> dist(0, 1);

  typedef boost::uint8_t u8_t;
  typedef boost::uint32_t u32_t;

  for(int loop(0); loop < 0x1000; loop++){
    bitset<300> b;
    for(int i(0); i < b.size(); ++i){
      b.set(i, dist(gen) == 1);
    }
    string b_str(b.to_string());
    reverse(b_str.begin(), b_str.end());
    BOOST_TEST_MESSAGE("Origin => ");
    for(int i(0); i < 300; i += 30){
      BOOST_TEST_MESSAGE(format("%3d ") % i << b_str.substr(i, 24) << ' ' << b_str.substr(i + 24, 6));
    }

    {
      BOOST_TEST_MESSAGE("u8_t container without padding");
      u8_t buf[(300 + sizeof(u8_t) * 8 - 1) / (sizeof(u8_t) * 8)];
      fill<0>(b, buf);
      check<0>(b, buf);
    }
    {
      BOOST_TEST_MESSAGE("u8_t container with padding 2");
      u8_t buf[(300 + sizeof(u8_t) * 8 - 2 - 1) / (sizeof(u8_t) * 8 - 2)];
      fill<2>(b, buf);
      check<2>(b, buf);
    }
    {
      BOOST_TEST_MESSAGE("u32_t container without padding");
      u32_t buf[(300 + sizeof(u32_t) * 8 - 1) / (sizeof(u32_t) * 8)];
      fill<0>(b, buf);
      check<0>(b, buf);
    }
    {
      BOOST_TEST_MESSAGE("u32_t container with padding 2");
      u32_t buf[(300 + sizeof(u32_t) * 8 - 2 - 1) / (sizeof(u32_t) * 8 - 2)];
      fill<2>(b, buf);
      check<2>(b, buf);
    }
  }
}

BOOST_AUTO_TEST_SUITE_END()
