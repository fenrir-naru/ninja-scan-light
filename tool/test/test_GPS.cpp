#define _USE_MATH_DEFINES
#include <cmath>

#include <ctime>
#include <climits>
#include <iostream>
#include <bitset>
#include <string>
#include <algorithm>
#include <vector>

#include "navigation/GPS.h"
#include "navigation/GPS_Solver_Base.h"
#include "navigation/coordinate.h"

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
    size_t N_bitset, class BufferT, std::size_t N_buf>
void check_parse(const bitset<N_bitset> &b, const BufferT (&buf)[N_buf]){
  typedef space_node_t::BroadcastedMessage<
      BufferT, (int)sizeof(BufferT) * CHAR_BIT - PaddingMSB - PaddingLSB, PaddingMSB> msg_t;
  BufferT buf2[N_buf];
#define each2(offset, bits, shift, func) { \
  boost::uint32_t res((boost::uint32_t)msg_t::func(buf)); \
  res >>= shift; \
  BOOST_TEST_MESSAGE(format("%s => 0x%08x") % #func % res); \
  for(int i(0), j(offset + bits - 1); i < bits; ++i, --j, res >>= 1){ \
    BOOST_REQUIRE_EQUAL(b[j], ((res & 0x1) == 1)); \
  } \
}
#define each(offset, bits, func) { \
  each2(offset, bits, 0, func); \
  msg_t::func ## _set(buf2, msg_t::func(buf)); \
  BOOST_REQUIRE_EQUAL(msg_t::func(buf), msg_t::func(buf2)); \
}
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

  each(  68, 16, SubFrame4_5_Almanac::e);
  each(  90,  8, SubFrame4_5_Almanac::t_oa);
  each(  98, 16, SubFrame4_5_Almanac::delta_i);
  each( 120, 16, SubFrame4_5_Almanac::dot_Omega0);
  each( 136,  8, SubFrame4_5_Almanac::SV_health);
  each( 150, 24, SubFrame4_5_Almanac::sqrt_A);
  each( 180, 24, SubFrame4_5_Almanac::Omega0);
  each( 210, 24, SubFrame4_5_Almanac::omega);
  each( 240, 24, SubFrame4_5_Almanac::M0);
  each2(270,  8,  3, SubFrame4_5_Almanac::a_f0);
  each( 289,  3, SubFrame4_5_Almanac::a_f0);
  each( 278, 11, SubFrame4_5_Almanac::a_f1);

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
  boost::random::normal_distribution<> norm_dist;

  Fixture()
      : gen(0), //static_cast<unsigned long>(time(0))
      bin_dist(0, 1), norm_dist(0, 1)
      {}
  ~Fixture(){}

  bool get_bool(){
    return bin_dist(gen) == 1;
  }
  double get_double(){
    return norm_dist(gen);
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

BOOST_AUTO_TEST_CASE(parity_calculation){
  typedef boost::uint32_t u32_t;
  static const u32_t packet[][10] = {
    // UBX 20210531/log.ubx
    {0x22c06f12, 0x96f4ab18, 0x000567ba, 0x8da593e4, 0x000a09e8, 0x1fae2945, 0x85f04883, 0x0f2a68f3, 0x3fea7f5c, 0x010910af},
    {0x22c06f12, 0x96f4ab18, 0x3ff9c815, 0x803a7f60, 0x3ff909f9, 0x8031c995, 0x88675bbe, 0xa80227e4, 0x3fe94d5d, 0x8fc19590},
    {0x22c06f12, 0x96f4ab18, 0x002c92c3, 0x15fff712, 0x80184a3a, 0x8051194a, 0x8c60701e, 0x897ca876, 0xbfe9aa21, 0xa07cea48},
    {0x22c06f12, 0x96f4ab18, 0x3fd6c8bd, 0x812a4a14, 0x003009c0, 0x28fc6911, 0x88eb726f, 0x06513976, 0xbfe981a6, 0x8cc1cd5b},
    {0x22c06f12, 0x96f4ab18, 0x0002935e, 0x8bd07e54, 0x3fe4c9fc, 0x3c84d1c5, 0x8bea1297, 0x3e8dec39, 0xbfe8f2cc, 0x047c1d63},
    {0x22c06f12, 0x96f4ab18, 0x3fe95cc1, 0xbd925a10, 0x3fee8a15, 0x87ba8819, 0x85bdc8a6, 0xa1efc191, 0xbfeb354a, 0x85bfb824},
    {0x22c06f12, 0x96f4ab18, 0x000626bc, 0x0a58052c, 0x0016898e, 0x86188e34, 0x04e6f636, 0xa45e4fd5, 0xbfea0780, 0x0709df53},
    {0x22c06f12, 0x96f4ab18, 0x3ffd9ccb, 0x28109433, 0x000a4a0d, 0x864db26e, 0x8623763e, 0xa3868c2e, 0xbfeb7fbd, 0x94c001b8},
    {0x22c06f12, 0x96f4ab18, 0x000032a7, 0x10172e7a, 0xbffcc9c7, 0x07385330, 0x08b9223a, 0xb348e024, 0x3fe98a8b, 0x0afd90cb},
    {0x22c06f12, 0x96f4ab18, 0x3fef1c3c, 0x23642bd4, 0x004e89c2, 0x80b73380, 0x05203447, 0x305bdde9, 0xbfea8c9b, 0x04bf1ae3},
    // UBX f9p_ppp_1224/rover.ubx
    {0x22c0593c, 0x226aea68, 0x393f01c1, 0x8ce67a0e, 0x8a40fa3a, 0xbf22802d, 0xa0ee0f0f, 0x03c82852, 0x83326c66, 0x99ed9f33},
    {0x22c0593c, 0x226aea68, 0x043efa55, 0x8d69c330, 0x0b6bd821, 0xbf084064, 0x02a67db6, 0x8391a845, 0x832eb57d, 0x99ed9f60},
    {0x22c0593c, 0x226aea68, 0x03823a77, 0x0c2d07d0, 0x2d867fb2, 0x821841b4, 0x06b84750, 0x02642871, 0x833f3474, 0x19ed9d0c},
    {0x22c0593c, 0x226aea68, 0x1102a2e7, 0x0e21d1bc, 0x3a021ed0, 0x0263809c, 0x3432e07f, 0x0297e850, 0x0375ba33, 0x19ed9fbf},
    {0x22c0593c, 0x226aea68, 0x1a001d7f, 0x0c0b9c5e, 0x919ff8f3, 0x001c4077, 0x2a88238b, 0x03a8e849, 0x835cb857, 0x19ed9fec},
    {0x22c0593c, 0x226aea68, 0x16fe4a01, 0x8c40bf73, 0x2d5a7247, 0x3e6ec0cf, 0x0320988e, 0x8232a86c, 0x03890814, 0x19ed9fec},
    {0x22c0593c, 0x226aea68, 0x09fe4c30, 0x0bf17eb2, 0xb67239b3, 0x3e830104, 0x1db8a628, 0x0247a86a, 0x835954a5, 0x99ed9f60},
    {0x22c0593c, 0x226aea68, 0x00814ad3, 0x0c7fa193, 0x307b3abb, 0x010c41cd, 0x9461038d, 0x857fe85c, 0x033ea986, 0x99ed47ff},
    {0x22c0593c, 0x226aea68, 0x0e803584, 0x0e4bf44d, 0x8f013410, 0x0037c0f8, 0x1f0267d9, 0x83c6a850, 0x034909ad, 0x99ed9f33},
    {0x22c0593c, 0x226b0b9c, 0x00054e8e, 0x9dc8b839, 0xbffdc9f9, 0x86bd6b9a, 0x876fa0b1, 0xafe67609, 0xbfe9dbc6, 0xb93d9734},
#if 0
    // UBX https://portal.u-blox.com/s/question/0D52p00008HKDgdCAH/gps-paritycheck-in-the-nav-messfrbxdoesnt-work-if-d301does-ucenter-change-anything-after-checking-parity-itself
    {0x22C17924, 0x0C4B09FC, 0x17940031, 0x3532C719, 0x396D6953, 0x06CFCAFA, 0x0CAA7AAF, 0x0836C543, 0x3FC001F7, 0x3F879C24},
    {0x22C17924, 0x0C4B2A94, 0x37FE1361, 0x33C05CA8, 0x0F066CD1, 0x01B57E70, 0x02EA389B, 0x3B1257B7, 0x3CA0A933, 0x36C56404},
    {0x22C17924, 0x0C4B4BE4, 0x00110F66, 0x1C6FB753, 0x0018760E, 0x0894420B, 0x39E396DC, 0x32D6F917, 0x00156E12, 0x37FF9990},
    {0x22C17924, 0x0C4B6CA0, 0x1E4D24FA, 0x1E96DE7A, 0x28CD0852, 0x1E0D4400, 0x118653CB, 0x2434BF90, 0x010692AC, 0x2AAAAABC},
    {0x22C17924, 0x0C4B8D9C, 0x12CCAE4C, 0x13B86A04, 0x3F43002F, 0x17BCBC05, 0x1D21EA30, 0x04335349, 0x05795D1A, 0x0BC001E0},
    {0x22C17924, 0x0C4BA960, 0x17940031, 0x3532C719, 0x396D6953, 0x06CFCAFA, 0x0CAA7AAF, 0x0836C543, 0x3FC001F7, 0x3F879C24},
#endif
  };
  for(std::size_t i(0); i < sizeof(packet) / sizeof(packet[0]); ++i){
    typedef space_node_t::BroadcastedMessage<u32_t, 30> msg_t;
    u32_t copy[10];
    std::memcpy(copy, packet[i], sizeof(copy));
    for(int j(0); j < 10; ++j){
      msg_t::parity_update(copy, j);
      /*
       * UBX-SFRBX contains D29* and D30* in the leading two bits
       * @see https://portal.u-blox.com/s/question/0D52p00009HHGpXCAX/manipulated-parity-bits-in-the-gps-lnav-sfrbx
       * @see https://portal.u-blox.com/s/question/0D52p00008HKDgdCAH/gps-paritycheck-in-the-nav-messfrbxdoesnt-work-if-d301does-ucenter-change-anything-after-checking-parity-itself
       */
      static const bool D29_30[][2] = { // check all candidates
        {false, false}, {false, true}, {true, false}, {true, true}
      };
      {
        std::size_t k(0);
        for(; k < sizeof(D29_30) / sizeof(D29_30[0]); ++k){
          u32_t copy2(copy[j]);
          if(D29_30[k][0]){copy2 ^= 0x29;} // D29*; bit 29 of previous transmitted word
          if(D29_30[k][1]){copy2 ^= 0x16;} // D30*; bit 30
          BOOST_TEST_MESSAGE(
              format("%d: 0x%08X; 0x%02X <=> 0x%02X (%d, %d)")
                % j % packet[i][j] % (packet[i][j] & 0x3F) % (copy2 & 0x3F)
                % (D29_30[k][0] ? 1 : 0) % (D29_30[k][1] ? 1 : 0));
          if(packet[i][j] == copy2){break;}
        }
        if(k == sizeof(D29_30) / sizeof(D29_30[0])){
          BOOST_FAIL("parity check failed.");
        }
        BOOST_WARN_MESSAGE(
            (((packet[i][j] & 0x80000000) == (D29_30[k][0] ? 0x80000000 : 0))
            && ((packet[i][j] & 0x40000000) == (D29_30[k][1] ? 0x40000000 : 0))),
            "UBX-SFRBX problem of D29* or D30*?");
      }
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
  std::tm tm2000 = {0, 0, 12, 1, 0, 2000 - 1900};
  gpst_t t2000(tm2000, 13);
  {
    // UTC, julian date
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

  std::tm t_tm[] = {
    {0, 0, 0, 1, 0, 2019 - 1900},
    {0, 0, 8, 1, 0, 2019 - 1900},
  };
  gpst_t t_gps[] = {
    gpst_t(t_tm[0], 18),
    gpst_t(t_tm[1], 18),
  };

#define HMS2S(hh, mm, ss) ((3600.0 * (hh)) + (60.0 * (mm)) + (ss))
#define DMS2RAD(dd, mm, ss) (M_PI / 180 * HMS2S(dd, mm, ss) / 3600)
  { // ERA (Earth rotation angle)
    // @see https://dc.zah.uni-heidelberg.de/apfs/times/q/form
    struct cmp_t {
      static void cmp(const double &a, const double &b, const double &delta){
        double a_(a - M_PI * 2 * std::floor(a / (M_PI * 2))); // [0, 2pi)
        double b_(b - M_PI * 2 * std::floor(b / (M_PI * 2)));
        BOOST_REQUIRE_SMALL(std::abs(a_ - b_), delta);
      }
    };
    cmp_t::cmp(t2000.earth_rotation_angle(), DMS2RAD(280, 27, 38.227), DMS2RAD(0, 0, 1));
    cmp_t::cmp(t_gps[0].earth_rotation_angle(), DMS2RAD(100, 7, 1.530), DMS2RAD(0, 0, 1));
    cmp_t::cmp(t_gps[1].earth_rotation_angle(), DMS2RAD(220, 26, 44.265), DMS2RAD(0, 0, 1));
  }
  { // sidereal time
    // @see (1) https://astronomy.stackexchange.com/questions/21002/how-to-find-greenwich-mean-sideral-time
    double t1_gmst(t_gps[0].greenwich_mean_sidereal_time_sec_ires1996());
    BOOST_REQUIRE_SMALL(std::abs(t1_gmst - 1665686.527373333), 1E-4);

    double t2_gmst(t_gps[1].greenwich_mean_sidereal_time_sec_ires1996());
    BOOST_REQUIRE_SMALL(
        std::abs(t2_gmst - (1665686.527373333 + 28878.85178960284)),
        1E-4);

    // @see (2) https://eco.mtk.nao.ac.jp/cgi-bin/koyomi/cande/gst_en.cgi
    // @see (3) https://dc.zah.uni-heidelberg.de/apfs/times/q/form
    struct cmp_t {
      static void cmp(const double &a, const double &b, const double &delta){
        double a_(a - std::floor(a / (24 * 3600)) * (24 * 3600)); // [0, 86400)
        double b_(b - std::floor(b / (24 * 3600)) * (24 * 3600));
        BOOST_REQUIRE_SMALL(std::abs(a_ - b_), delta);
      }
    };
    cmp_t::cmp(
        t2000.greenwich_mean_sidereal_time_sec_ires2010(),
        HMS2S(18, 41, 50.5494),
        1E-3);
    cmp_t::cmp(
        t_gps[0].greenwich_mean_sidereal_time_sec_ires2010(),
        HMS2S(6, 41, 26.5248),
        1E-3);
    cmp_t::cmp(
        t_gps[1].greenwich_mean_sidereal_time_sec_ires2010(),
        HMS2S(14, 42, 45.3766),
        1E-3);
  }
#undef DMS2RAD
#undef HMS2S
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

BOOST_FIXTURE_TEST_CASE(rotation, Fixture){
  typedef space_node_t::xyz_t xyz_t;
  typedef xyz_t::Rotation rot_t;
  typedef Vector3<double> vec3_t;
  double xyz[][3] = {
      {1, 0, 0}, {0, 1, 0}, {0, 0, 1},
      {get_double(), get_double(), get_double()},
  };
  vec3_t xyz_v[sizeof(xyz) / sizeof(xyz[0])];
  for(std::size_t j(0); j < sizeof(xyz) / sizeof(xyz[0]); ++j){
    xyz_v[j] = vec3_t(xyz[j]);
  }
  for(int i(0); i < 0x100; ++i){
    rot_t rot(rot_t().then_x(get_double()).then_y(get_double()).then_z(get_double()));
    for(std::size_t j(0); j < sizeof(xyz) / sizeof(xyz[0]); ++j){
      xyz_t xyz_(xyz_v[j]);
      rot.apply(xyz_);
      vec3_t delta((vec3_t)xyz_ - (rot.q.conj() * xyz_v[j] * rot.q).vector());
      BOOST_REQUIRE_SMALL(delta.abs2(), 1E-8);
    }
  }
}

BOOST_AUTO_TEST_SUITE_END()
