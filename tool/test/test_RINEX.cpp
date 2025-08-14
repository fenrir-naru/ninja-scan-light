#if defined(_MSC_VER)
#define _USE_MATH_DEFINES
#endif
#include <iostream>
#include <string>
#include <sstream>

#include "navigation/RINEX.h"
#include "navigation/RINEX_Clock.h"
#include "navigation/GPS.h"
#include "navigation/GLONASS.h"

#include <boost/random.hpp>
#include <boost/random/random_device.hpp>

#include <boost/format.hpp>
#include <boost/cstdint.hpp>
#include <boost/xpressive/xpressive.hpp>

#define BOOST_TEST_MAIN
#include <boost/test/included/unit_test.hpp>

using namespace std;
using boost::format;

typedef double fnum_t;
typedef GPS_SpaceNode<fnum_t> gps_t;
typedef SBAS_SpaceNode<fnum_t> sbas_t;
typedef GLONASS_SpaceNode<fnum_t> glonass_t;

BOOST_AUTO_TEST_SUITE(RINEX)

template <class T1, class T2>
void compare_lines(const T1 &a, const T2 &b, const int &skip_lines = 0){
  std::stringstream ss_a(a), ss_b(b);
  char buf_a[0x100], buf_b[0x100];
  for(int i(0); true; i++){
    if(ss_a.eof() && ss_b.eof()){break;}
    if(ss_a.eof() ^ ss_b.eof()){
      BOOST_FAIL("Line numbers are different!");
      break;
    }
    ss_a.getline(buf_a, sizeof(buf_a));
    ss_b.getline(buf_b, sizeof(buf_b));
    if(i < skip_lines){continue;}
    BOOST_REQUIRE_EQUAL(std::string(buf_a), std::string(buf_b));
  }
}

static std::string modify_trailer(
    const std::string &src,
    const bool &remove_blank = false, const char *new_eol = "\n"){
  using namespace boost::xpressive;
  return regex_replace(src,
      sregex::compile("( *)(?:\r\n?|\n)"),
      std::string(remove_blank ? "" : "$1").append(new_eol));
}

template <class ReaderT>
static void check_reader_versatility_to_input(const char *src){
  struct opt_t {
    bool no_trailing_blank;
    const char *eol;
  } opts[] = {
    {false, "\n"}, {false, "\r"}, {false, "\r\n"},
    {true,  "\n"}, {true,  "\r"}, {true,  "\r\n"},
  };
  for(unsigned i(0); i < sizeof(opts) / sizeof(opts[0]); ++i){
    std::stringbuf sbuf(modify_trailer(src, opts[i].no_trailing_blank, opts[i].eol));
    std::istream in(&sbuf);
    ReaderT reader(in);
    while(reader.has_next()){reader.next();}
  }
}

BOOST_AUTO_TEST_CASE(nav_GPS_v2){
  const char *src = \
      "     2.10           N: GPS NAV DATA                         RINEX VERSION / TYPE\n"
      "XXRINEXN V2.10      AIUB                 3-SEP-99 15:22     PGM / RUN BY / DATE \n"
      "EXAMPLE OF VERSION 2.10 FORMAT                              COMMENT             \n"
      "     .1676D-07   .2235D-07  -.1192D-06  -.1192D-06          ION ALPHA           \n"
      "     .1208D+06   .1310D+06  -.1310D+06  -.1966D+06          ION BETA            \n"
      "     .133179128170D-06  .107469588780D-12   552960     1025 DELTA-UTC: A0,A1,T,W\n"
      "    13                                                      LEAP SECONDS        \n"
      "                                                            END OF HEADER       \n"
      " 6 99  9  2 17 51 44.0 -.839701388031D-03 -.165982783074D-10  .000000000000D+00 \n"
      "     .910000000000D+02  .934062500000D+02  .116040547840D-08  .162092304801D+00 \n"
      "     .484101474285D-05  .626740418375D-02  .652112066746D-05  .515365489006D+04 \n"
      "     .409904000000D+06 -.242143869400D-07  .329237003460D+00 -.596046447754D-07 \n"
      "     .111541663136D+01  .326593750000D+03  .206958726335D+01 -.638312302555D-08 \n"
      "     .307155651409D-09  .000000000000D+00  .102500000000D+04  .000000000000D+00 \n"
      "     .000000000000D+00  .000000000000D+00  .000000000000D+00  .910000000000D+02 \n"
      "     .406800000000D+06  .000000000000D+00                                       \n";
     //----|---1|0---|---2|0---|---3|0---|---4|0---|---5|0---|---6|0---|---7|0---|---8|

  typedef RINEX_NAV_Reader<fnum_t> reader_t;
  typedef RINEX_NAV_Writer<fnum_t> writer_t;

  check_reader_versatility_to_input<reader_t>(src);

  gps_t gps;
  {
    std::stringbuf sbuf(src);
    std::istream in(&sbuf);
    reader_t::read_all(in, gps);
  }

  {
    BOOST_REQUIRE(gps.is_valid_iono_utc());
    /*typename*/ gps_t::Ionospheric_UTC_Parameters iono_utc(gps.iono_utc());
    static const fnum_t
        alpha[4] = {.1676E-07, .2235E-07, -.1192E-06, -.1192E-06},
        beta[4] = {.1208E+06, .1310E+06, -.1310E+06, -.1966E+06},
        A0(.133179128170E-06), A1(.107469588780E-12);
    static const int t_ot(552960), WN_t(1025);
    for(int i(0); i < 4; ++i){
      BOOST_CHECK_SMALL(std::abs(iono_utc.alpha[i] - alpha[i]), 1E-8);
      BOOST_CHECK_SMALL(std::abs(iono_utc.beta[i] - beta[i]), 1E-7);
    }
    BOOST_CHECK_SMALL(std::abs(iono_utc.A0 - A0), 1E-7);
    BOOST_CHECK_SMALL(std::abs(iono_utc.A1 - A1), 1E-7);
    BOOST_CHECK_EQUAL(iono_utc.t_ot, t_ot);
    BOOST_CHECK_EQUAL(iono_utc.WN_t, WN_t);
  }

  {
    std::tm t_oc_tm = {44, 51, 17, 2, 9 - 1, 1999 - 1900};
    /*typename*/ gps_t::gps_time_t t_oc(t_oc_tm);
    gps.satellite(6).select_ephemeris(t_oc);
    const /*typename*/ gps_t::Satellite::Ephemeris &eph(gps.satellite(6).ephemeris());

    BOOST_CHECK_SMALL(std::abs(-.839701388031E-03 - eph.a_f0), 1E-15);
    BOOST_CHECK_SMALL(std::abs(-.165982783074E-10 - eph.a_f1), 1E-22);
    BOOST_CHECK_SMALL(std::abs( .000000000000E+00 - eph.a_f2), 1E-12);

    BOOST_CHECK_SMALL(std::abs( .910000000000E+02 - eph.iode),    1E-10);
    BOOST_CHECK_SMALL(std::abs( .934062500000E+02 - eph.c_rs),    1E-10);
    BOOST_CHECK_SMALL(std::abs( .116040547840E-08 - eph.delta_n), 1E-20);
    BOOST_CHECK_SMALL(std::abs( .162092304801E+00 - eph.M0),      1E-12);

    BOOST_CHECK_SMALL(std::abs( .484101474285E-05 - eph.c_uc),    1E-17);
    BOOST_CHECK_SMALL(std::abs( .626740418375E-02 - eph.e),       1E-14);
    BOOST_CHECK_SMALL(std::abs( .652112066746E-05 - eph.c_us),    1E-17);
    BOOST_CHECK_SMALL(std::abs( .515365489006E+04 - eph.sqrt_A),  1E-8);

    BOOST_CHECK_SMALL(std::abs( .409904000000E+06 - eph.t_oe),    1E-6);
    BOOST_CHECK_SMALL(std::abs(-.242143869400E-07 - eph.c_ic),    1E-19);
    BOOST_CHECK_SMALL(std::abs( .329237003460E+00 - eph.Omega0),  1E-12);
    BOOST_CHECK_SMALL(std::abs(-.596046447754E-07 - eph.c_is),    1E-19);

    BOOST_CHECK_SMALL(std::abs( .111541663136E+01 - eph.i0),          1E-11);
    BOOST_CHECK_SMALL(std::abs( .326593750000E+03 - eph.c_rc),        1E-9);
    BOOST_CHECK_SMALL(std::abs( .206958726335E+01 - eph.omega),       1E-11);
    BOOST_CHECK_SMALL(std::abs(-.638312302555E-08 - eph.dot_Omega0),  1E-20);

    BOOST_CHECK_SMALL(std::abs( .307155651409E-09 - eph.dot_i0), 1E-21);

    BOOST_CHECK_EQUAL(0, eph.URA);
    BOOST_CHECK_EQUAL(0, eph.SV_health);
    BOOST_CHECK_SMALL(std::abs( .000000000000E+00 - eph.t_GD), 1E-12);
    BOOST_CHECK_SMALL(std::abs( .910000000000E+02 - eph.iodc), 1E-10);

    BOOST_CHECK_SMALL(std::abs(60 * 60 * 4 - eph.fit_interval), 1E-12);
  }

  std::string dist;
  {
    std::stringstream ss;
    writer_t writer(ss);
    writer.header()["PGM / RUN BY / DATE"] = "XXRINEXN V2.10      AIUB                 3-SEP-99 15:22";
    writer.header()["COMMENT"] = "EXAMPLE OF VERSION 2.10 FORMAT";
    writer.write_all(gps, 210);
    dist = ss.str();
    dist.replace(14 * 81 +  4, 18, " .000000000000D+00"); // overwrite SV accuracy
    dist.replace(15 * 81 +  4, 18, " .406800000000D+06"); // overwrite Transmission time
    dist.replace(15 * 81 + 23, 18, " .000000000000D+00"); // overwrite Fit interval
  }

  compare_lines(src, dist);
}

BOOST_AUTO_TEST_CASE(nav_GPS_v3){
  const char *src = \
      "     3.02           N: GNSS NAV DATA    G: GPS              RINEX VERSION / TYPE\n"
      "NetR9 5.45          Receiver Operator   20210526 000001 UTC PGM / RUN BY / DATE \n"
      "GPSA    .6519D-08   .2235D-07  -.5960D-07  -.1192D-06       IONOSPHERIC CORR    \n"
      "GPSB    .8602D+05   .9830D+05  -.6554D+05  -.5243D+06       IONOSPHERIC CORR    \n"
      "GPUT   .9313225746D-09 -.888178420D-15 405504 2159          TIME SYSTEM CORR    \n"
      "    18    18  2185     7                                    LEAP SECONDS        \n"
      "                                                            END OF HEADER       \n"
      "G18 2021 05 26 00 00 00  .349333044142D-03 -.125055521494D-11  .000000000000D+00\n"
      "      .153000000000D+03 -.127968750000D+03  .412802909184D-08 -.186204225475D+01\n"
      "     -.674836337566D-05  .142880436033D-02  .113956630230D-04  .515366753769D+04\n"
      "      .259200000000D+06 -.372529029846D-08  .285810954536D+01 -.353902578354D-07\n"
      "      .968418001400D+00  .168687500000D+03  .290532037240D+01 -.775818030221D-08\n"
      "      .832177520677D-10  .000000000000D+00  .215900000000D+04  .000000000000D+00\n"
      "      .200000000000D+01  .000000000000D+00 -.838190317154D-08  .921000000000D+03\n"
      "      .252018000000D+06  .400000000000D+01                                      \n";
     //----|---1|0---|---2|0---|---3|0---|---4|0---|---5|0---|---6|0---|---7|0---|---8|

  typedef RINEX_NAV_Reader<fnum_t> reader_t;
  typedef RINEX_NAV_Writer<fnum_t> writer_t;

  check_reader_versatility_to_input<reader_t>(src);

  gps_t gps;
  {
    std::stringbuf sbuf(src);
    std::istream in(&sbuf);
    reader_t::read_all(in, gps);
  }

  {
    std::tm t_oc_tm = {0, 0, 0, 26, 5 - 1, 2021 - 1900};
    /*typename*/ gps_t::gps_time_t t_oc(t_oc_tm);
    gps.satellite(18).select_ephemeris(t_oc);
    const /*typename*/ gps_t::Satellite::Ephemeris &eph(gps.satellite(18).ephemeris());

    BOOST_CHECK_SMALL(std::abs( .349333044142E-03 - eph.a_f0), 1E-15);
    BOOST_CHECK_SMALL(std::abs(-.125055521494E-11 - eph.a_f1), 1E-23);
    BOOST_CHECK_SMALL(std::abs( .000000000000E+00 - eph.a_f2), 1E-12);

    BOOST_CHECK_SMALL(std::abs( .153000000000E+03 - eph.iode),    1E-9);
    BOOST_CHECK_SMALL(std::abs(-.127968750000E+03 - eph.c_rs),    1E-9);
    BOOST_CHECK_SMALL(std::abs( .412802909184E-08 - eph.delta_n), 1E-20);
    BOOST_CHECK_SMALL(std::abs(-.186204225475E+01 - eph.M0),      1E-11);

    BOOST_CHECK_SMALL(std::abs(-.674836337566E-05 - eph.c_uc),    1E-17);
    BOOST_CHECK_SMALL(std::abs( .142880436033E-02 - eph.e),       1E-14);
    BOOST_CHECK_SMALL(std::abs( .113956630230E-04 - eph.c_us),    1E-16);
    BOOST_CHECK_SMALL(std::abs( .515366753769E+04 - eph.sqrt_A),  1E-8);

    BOOST_CHECK_SMALL(std::abs( .259200000000E+06 - eph.t_oe),    1E-6);
    BOOST_CHECK_SMALL(std::abs(-.372529029846E-08 - eph.c_ic),    1E-20);
    BOOST_CHECK_SMALL(std::abs( .285810954536E+01 - eph.Omega0),  1E-11);
    BOOST_CHECK_SMALL(std::abs(-.353902578354E-07 - eph.c_is),    1E-19);

    BOOST_CHECK_SMALL(std::abs( .968418001400E+00 - eph.i0),          1E-12);
    BOOST_CHECK_SMALL(std::abs( .168687500000E+03 - eph.c_rc),        1E-9);
    BOOST_CHECK_SMALL(std::abs( .290532037240E+01 - eph.omega),       1E-11);
    BOOST_CHECK_SMALL(std::abs(-.775818030221E-08 - eph.dot_Omega0),  1E-20);

    BOOST_CHECK_SMALL(std::abs( .832177520677E-10 - eph.dot_i0), 1E-22);

    BOOST_CHECK_SMALL(std::abs( .200000000000E+01 - eph.URA),  1E-12);
    BOOST_CHECK_EQUAL(0, eph.SV_health);
    BOOST_CHECK_SMALL(std::abs(-.838190317154E-08 - eph.t_GD), 1E-20);
    BOOST_CHECK_SMALL(std::abs( .921000000000E+03 - eph.iodc), 1E-9);

    BOOST_CHECK_SMALL(std::abs(60 * 60 * 4 - eph.fit_interval), 1E-12);
  }

  std::string dist;
  {
    std::stringstream ss;
    writer_t writer(ss);
    //writer.header()["PGM / RUN BY / DATE"] = "NetR9 5.45          Receiver Operator   20210526 000001 UTC";
    std::tm t_header = {1, 0, 0, 26, 5 - 1, 2021 - 1900};
    writer.pgm_runby_date("NetR9 5.45", "Receiver Operator", t_header, "UTC");
    writer.write_all(gps, 302);
    dist = ss.str();
    dist.replace(13 * 81 +  5, 18, " .200000000000D+01"); // overwrite SV accuracy
    dist.replace(14 * 81 +  5, 18, " .252018000000D+06"); // overwrite Transmission time
  }

  compare_lines(src, dist);
}

BOOST_AUTO_TEST_CASE(nav_SBAS_v2){
  const char *src = \
      "     2.11           H: GEO NAV MSG DATA                     RINEX VERSION / TYPE\n"
      "SBAS2RINEX 2.0      CNES                20-Oct-03 14:01     PGM / RUN BY / DATE \n"
      "0.133179128170D-06-0.107469588780D-12 518400 1240 EGNOS  5  D-UTC A0,A1,T,W,S,U \n"
      "    13                                                      LEAP SECONDS        \n"
      "This file contains navigation message data from a SBAS      COMMENT             \n"
      "(geostationary) satellite, here AOR-W (PRN 122 = # 22)      COMMENT             \n"
      "                                                            END OF HEADER       \n"
      "22 03 10 18  0  1  4.0-1.005828380585D-07 6.366462912410D-12 5.184420000000D+05 \n"
      "    2.482832392000D+04-3.593750000000D-04-1.375000000000D-07 0.000000000000D+00 \n"
      "   -3.408920872000D+04-1.480625000000D-03-5.000000000000D-08 4.000000000000D+00 \n"
      "   -1.650560000000D+01 8.360000000000D-04 6.250000000000D-08 2.300000000000D+01 \n";
     //----|---1|0---|---2|0---|---3|0---|---4|0---|---5|0---|---6|0---|---7|0---|---8|

  typedef RINEX_NAV_Reader<fnum_t> reader_t;
  typedef RINEX_NAV_Writer<fnum_t> writer_t;

  check_reader_versatility_to_input<reader_t>(src);

  sbas_t sbas;
  {
    std::stringbuf sbuf(src);
    std::istream in(&sbuf);
    reader_t::space_node_list_t space_nodes = {NULL};
    space_nodes.sbas = &sbas;
    reader_t::read_all(in, space_nodes);
  }

  {
    std::tm t_tm = {4, 1, 0, 18, 10 - 1, 2003 - 1900};
    gps_t::gps_time_t t(t_tm);
    sbas.satellite(122).select_ephemeris(t);
    const sbas_t::Satellite::Ephemeris &eph(sbas.satellite(122).ephemeris());

    BOOST_CHECK_SMALL(std::abs(-1.005828380585E-07 - eph.a_Gf0), 1E-19);
    BOOST_CHECK_SMALL(std::abs( 6.366462912410E-12 - eph.a_Gf1), 1E-24);
    //BOOST_CHECK_SMALL(std::abs( 5.184420000000E+05 - TODO),     1E-10); // transmission time

    BOOST_CHECK_SMALL(std::abs( 2.482832392000E+07 - eph.x),     1E-05); // km => m
    BOOST_CHECK_SMALL(std::abs(-3.593750000000E-01 - eph.dx),    1E-13); // km/s => m/s
    BOOST_CHECK_SMALL(std::abs(-1.375000000000E-04 - eph.ddx),   1E-17); // km/s^2 => m/s^2
    //BOOST_CHECK_SMALL(std::abs(0.000000000000E+00 - TODO),      1E-12); // health

    BOOST_CHECK_SMALL(std::abs(-3.408920872000E+07 - eph.y),     1E-05);
    BOOST_CHECK_SMALL(std::abs(-1.480625000000E-00 - eph.dy),    1E-12);
    BOOST_CHECK_SMALL(std::abs(-5.000000000000E-05 - eph.ddy),   1E-17);
    BOOST_CHECK_SMALL(std::abs( 4.000000000000E+00 - eph.URA),   1E-12);

    BOOST_CHECK_SMALL(std::abs(-1.650560000000E+04 - eph.z),     1E-08);
    BOOST_CHECK_SMALL(std::abs( 8.360000000000E-01 - eph.dz),    1E-13);
    BOOST_CHECK_SMALL(std::abs( 6.250000000000E-05 - eph.ddz),   1E-17);
    //BOOST_CHECK_SMALL(std::abs( 2.300000000000E+01 - TODO),     1E-11); // iodn
  }

  std::string dist;
  {
    std::stringstream ss;
    writer_t writer(ss);
    writer.header()["PGM / RUN BY / DATE"]
        = "SBAS2RINEX 2.0      CNES                20-Oct-03 14:01";
    writer_t::space_node_list_t space_nodes = {NULL};
    space_nodes.sbas = &sbas;
    writer.write_all(space_nodes, 211);
    dist = ss.str();
  }

  //compare_lines(src, dist, 5); // TODO check header
}

BOOST_AUTO_TEST_CASE(nav_SBAS_v3){
  const char *src = \
      "     3.04           N: GNSS NAV DATA    S: SBAS             RINEX VERSION / TYPE\n"
      "SBAS2RINEX 3.0      CNES                20031018 140100     PGM / RUN BY / DATE \n"
      "EXAMPLE OF VERSION 3.04 FORMAT                              COMMENT             \n"
      "SBUT  -.1331791282D-06 -.107469589D-12 552960 1025 EGNOS  5 TIME SYSTEM CORR    \n"
      "    13                                                      LEAP SECONDS        \n"
      "This file contains navigation message data from a SBAS      COMMENT             \n"
      "(geostationary) satellite, here AOR-W (PRN 122 = # S22)     COMMENT             \n"
      "                                                            END OF HEADER       \n"
      "S22 2003 10 18  0  1  4-1.005828380585D-07 6.366462912410D-12 5.184420000000D+05\n"
      "     2.482832392000D+04-3.593750000000D-04-1.375000000000D-07 0.000000000000D+00\n"
      "    -3.408920872000D+04-1.480625000000D-03-5.000000000000D-08 4.000000000000D+00\n"
      "    -1.650560000000D+01 8.360000000000D-04 6.250000000000D-08 2.300000000000D+01\n";
     //----|---1|0---|---2|0---|---3|0---|---4|0---|---5|0---|---6|0---|---7|0---|---8|

  typedef RINEX_NAV_Reader<fnum_t> reader_t;
  typedef RINEX_NAV_Writer<fnum_t> writer_t;

  check_reader_versatility_to_input<reader_t>(src);

  sbas_t sbas;
  {
    std::stringbuf sbuf(src);
    std::istream in(&sbuf);
    reader_t::space_node_list_t space_nodes = {NULL};
    space_nodes.sbas = &sbas;
    reader_t::read_all(in, space_nodes);
  }

  {
    std::tm t_oc_tm = {4, 1, 0, 18, 10 - 1, 2003 - 1900};
    gps_t::gps_time_t t_oc(t_oc_tm);
    sbas.satellite(122).select_ephemeris(t_oc);
    const sbas_t::Satellite::Ephemeris &eph(sbas.satellite(122).ephemeris());

    BOOST_CHECK_SMALL(std::abs(-1.005828380585E-07 - eph.a_Gf0), 1E-19);
    BOOST_CHECK_SMALL(std::abs( 6.366462912410E-12 - eph.a_Gf1), 1E-24);
    //BOOST_CHECK_SMALL(std::abs( 5.184420000000E+05 - TODO),     1E-10); // transmission time

    BOOST_CHECK_SMALL(std::abs( 2.482832392000E+07 - eph.x),     1E-05); // km => m
    BOOST_CHECK_SMALL(std::abs(-3.593750000000E-01 - eph.dx),    1E-13); // km/s => m/s
    BOOST_CHECK_SMALL(std::abs(-1.375000000000E-04 - eph.ddx),   1E-17); // km/s^2 => m/s^2
    //BOOST_CHECK_SMALL(std::abs(0.000000000000E+00 - TODO),      1E-12); // health

    BOOST_CHECK_SMALL(std::abs(-3.408920872000E+07 - eph.y),     1E-05);
    BOOST_CHECK_SMALL(std::abs(-1.480625000000E-00 - eph.dy),    1E-12);
    BOOST_CHECK_SMALL(std::abs(-5.000000000000E-05 - eph.ddy),   1E-17);
    BOOST_CHECK_SMALL(std::abs( 4.000000000000E+00 - eph.URA),   1E-12);

    BOOST_CHECK_SMALL(std::abs(-1.650560000000E+04 - eph.z),     1E-08);
    BOOST_CHECK_SMALL(std::abs( 8.360000000000E-01 - eph.dz),    1E-13);
    BOOST_CHECK_SMALL(std::abs( 6.250000000000E-05 - eph.ddz),   1E-17);
    //BOOST_CHECK_SMALL(std::abs( 2.300000000000E+01 - TODO),     1E-11); // iodn
  }

  std::string dist;
  {
    std::stringstream ss;
    writer_t writer(ss);
    //writer.header()["PGM / RUN BY / DATE"]
    //    = "XXRINEXN V3         AIUB                20061002 000123 UTC";
    std::tm t_header = {0, 1, 14, 18, 10 - 1, 2003 - 1900};
    writer.pgm_runby_date("SBAS2RINEX 3.0", "CNES", t_header, "");
    writer.header()["COMMENT"] << "EXAMPLE OF VERSION 3.04 FORMAT";
    writer.header()["TIME SYSTEM CORR"]
        << "SBUT  -.1331791282D-06 -.107469589D-12 552960 1025 EGNOS  5";
    writer.header()["LEAP SECONDS"] = "    13"; // writer.leap_seconds(gps);

    writer_t::space_node_list_t space_nodes = {NULL};
    space_nodes.sbas = &sbas;
    writer.write_all(space_nodes, 304);
    dist = ss.str();
  }

  //compare_lines(src, dist, 8); // TODO check header
}

BOOST_AUTO_TEST_CASE(nav_QZSS_v3){
  const char *src = \
      "     3.04           N: GNSS NAV DATA    J: QZSS             RINEX VERSION / TYPE\n"
      "GR25 V3.08                              20140513 072944 UTC PGM / RUN BY / DATE \n"
      "    16        1694     7                                    LEAP SECONDS        \n"
      "                                                            END OF HEADER       \n"
      "J01 2014 05 13 08 15 12 3.323303535581D-04-1.818989403546D-11 0.000000000000D+00\n"
      "     6.900000000000D+01-4.927812500000D+02 2.222949737636D-09 7.641996743610D-01\n"
      "    -1.654587686062D-05 7.542252133135D-02 1.197867095470D-05 6.492895933151D+03\n"
      "     2.025120000000D+05-8.381903171539D-07-9.211997910060D-01-2.041459083557D-06\n"
      "     7.082252892260D-01-1.558437500000D+02-1.575843337115D+00-2.349740733276D-09\n"
      "    -6.793140104410D-10 2.000000000000D+00 1.792000000000D+03 1.000000000000D+00\n"
      "     2.000000000000D+00 1.000000000000D+00-4.656612873077D-09 6.900000000000D+01\n"
      "     1.989000000000D+05 0.000000000000D+00                                      \n";
     //----|---1|0---|---2|0---|---3|0---|---4|0---|---5|0---|---6|0---|---7|0---|---8|

  typedef RINEX_NAV_Reader<fnum_t> reader_t;
  typedef RINEX_NAV_Writer<fnum_t> writer_t;

  check_reader_versatility_to_input<reader_t>(src);

  gps_t gps;
  {
    std::stringbuf sbuf(src);
    std::istream in(&sbuf);
    reader_t::space_node_list_t space_nodes = {NULL};
    space_nodes.qzss = &gps;
    reader_t::read_all(in, space_nodes);
  }

  {
    std::tm t_oc_tm = {12, 15, 8, 13, 5 - 1, 2014 - 1900};
    /*typename*/ gps_t::gps_time_t t_oc(t_oc_tm);
    gps.satellite(1 + 192).select_ephemeris(t_oc);
    const /*typename*/ gps_t::Satellite::Ephemeris &eph(gps.satellite(1 + 192).ephemeris());

    BOOST_CHECK_SMALL(std::abs( 3.323303535581E-04 - eph.a_f0), 1E-16);
    BOOST_CHECK_SMALL(std::abs(-1.818989403546E-11 - eph.a_f1), 1E-23);
    BOOST_CHECK_SMALL(std::abs( 0.000000000000E+00 - eph.a_f2), 1E-12);

    BOOST_CHECK_SMALL(std::abs( 6.900000000000E+01 - eph.iode),    1E-11);
    BOOST_CHECK_SMALL(std::abs(-4.927812500000E+02 - eph.c_rs),    1E-10);
    BOOST_CHECK_SMALL(std::abs( 2.222949737636E-09 - eph.delta_n), 1E-21);
    BOOST_CHECK_SMALL(std::abs( 7.641996743610E-01 - eph.M0),      1E-13);

    BOOST_CHECK_SMALL(std::abs(-1.654587686062E-05 - eph.c_uc),    1E-17);
    BOOST_CHECK_SMALL(std::abs( 7.542252133135E-02 - eph.e),       1E-14);
    BOOST_CHECK_SMALL(std::abs( 1.197867095470E-05 - eph.c_us),    1E-17);
    BOOST_CHECK_SMALL(std::abs( 6.492895933151E+03 - eph.sqrt_A),  1E-9);

    BOOST_CHECK_SMALL(std::abs( 2.025120000000E+05 - eph.t_oe),    1E-7);
    BOOST_CHECK_SMALL(std::abs(-8.381903171539E-07 - eph.c_ic),    1E-19);
    BOOST_CHECK_SMALL(std::abs(-9.211997910060E-01 - eph.Omega0),  1E-13);
    BOOST_CHECK_SMALL(std::abs(-2.041459083557E-06 - eph.c_is),    1E-18);

    BOOST_CHECK_SMALL(std::abs( 7.082252892260E-01 - eph.i0),          1E-13);
    BOOST_CHECK_SMALL(std::abs(-1.558437500000E+02 - eph.c_rc),        1E-10);
    BOOST_CHECK_SMALL(std::abs(-1.575843337115E+00 - eph.omega),       1E-12);
    BOOST_CHECK_SMALL(std::abs(-2.349740733276E-09 - eph.dot_Omega0),  1E-21);

    BOOST_CHECK_SMALL(std::abs(-6.793140104410E-10 - eph.dot_i0), 1E-22);

    BOOST_CHECK_SMALL(std::abs( 2.000000000000E+00 - eph.URA),  1E-13);
    BOOST_CHECK_EQUAL(1, eph.SV_health);
    BOOST_CHECK_SMALL(std::abs(-4.656612873077E-09 - eph.t_GD), 1E-21);
    BOOST_CHECK_SMALL(std::abs( 6.900000000000E+01 - eph.iodc), 1E-11);

    BOOST_CHECK_SMALL(std::abs(60 * 60 * 2 - eph.fit_interval), 1E-12);
  }

  std::string dist;
  {
    std::stringstream ss;
    writer_t writer(ss);
    writer.set_version(304);
    std::tm t_header = {44, 29, 7, 13, 5 - 1, 2014 - 1900};
    writer.pgm_runby_date("GR25 V3.08", "", t_header, "UTC");
    writer.leap_seconds(gps);
    writer_t::space_node_list_t space_nodes = {NULL};
    space_nodes.qzss = &gps;
    writer.write_all(space_nodes, 304);
    dist = ss.str();
    dist.replace(10 * 81 +  5, 18, "2.000000000000D+00"); // overwrite SV accuracy
    dist.replace(11 * 81 +  5, 18, "1.989000000000D+05"); // overwrite Transmission time
  }

  //compare_lines(src, dist);
}

BOOST_AUTO_TEST_CASE(nav_GLONASS_v2){
  const char *src = \
      "     2.11           GLONASS NAV DATA                        RINEX VERSION / TYPE\n"
      "ASRINEXG V1.1.0 VM  AIUB                19-FEB-98 10:42     PGM / RUN BY / DATE \n"
      "STATION ZIMMERWALD                                          COMMENT             \n"
      "  1998     2    16    0.379979610443D-06                    CORR TO SYSTEM TIME \n"
      "                                                            END OF HEADER       \n"
      " 3 98  2 15  0 15  0.0 0.163525342941D-03 0.363797880709D-11 0.108000000000D+05 \n"
      "    0.106275903320D+05-0.348924636841D+00 0.931322574615D-09 0.000000000000D+00 \n"
      "   -0.944422070313D+04 0.288163375854D+01 0.931322574615D-09 0.210000000000D+02 \n"
      "    0.212257280273D+05 0.144599342346D+01-0.186264514923D-08 0.300000000000D+01 \n";
     //----|---1|0---|---2|0---|---3|0---|---4|0---|---5|0---|---6|0---|---7|0---|---8|

  typedef RINEX_NAV_Reader<fnum_t> reader_t;
  typedef RINEX_NAV_Writer<fnum_t> writer_t;

  check_reader_versatility_to_input<reader_t>(src);

  glonass_t glonass;
  {
    std::stringbuf sbuf(src);
    std::istream in(&sbuf);
    reader_t::space_node_list_t space_nodes = {NULL};
    space_nodes.glonass = &glonass;
    reader_t::read_all(in, space_nodes);
  }

  {
    std::tm t_tm = {0, 15, 0, 15, 2 - 1, 1998 - 1900};
    /*typename*/ gps_t::gps_time_t t(t_tm, 12);
    glonass.satellite(3).select_ephemeris(t);
    const /*typename*/ glonass_t::Satellite::Ephemeris &eph(glonass.satellite(3).ephemeris());

    BOOST_CHECK_SMALL(std::abs(-.163525342941E-03 - eph.tau_n),   1E-15); // tau_n is inverted
    BOOST_CHECK_SMALL(std::abs( .363797880709E-11 - eph.gamma_n), 1E-23);
    BOOST_CHECK_SMALL(std::abs( .108000000000E+05 - eph.t_k),     1E-07);

    BOOST_CHECK_SMALL(std::abs( .106275903320E+08 - eph.xn),      1E-04);
    BOOST_CHECK_SMALL(std::abs(-.348924636841E+03 - eph.xn_dot),  1E-09);
    BOOST_CHECK_SMALL(std::abs( .931322574615E-06 - eph.xn_ddot), 1E-18);
    BOOST_CHECK_SMALL(std::abs( .000000000000E+00 - eph.B_n),     1E-12);

    BOOST_CHECK_SMALL(std::abs(-.944422070313E+07 - eph.yn),      1E-05);
    BOOST_CHECK_SMALL(std::abs( .288163375854E+04 - eph.yn_dot),  1E-08);
    BOOST_CHECK_SMALL(std::abs( .931322574615E-06 - eph.yn_ddot), 1E-18);
    BOOST_CHECK_SMALL(std::abs( .210000000000E+02 - eph.freq_ch), 1E-08);

    BOOST_CHECK_SMALL(std::abs( .212257280273E+08 - eph.zn),      1E-04);
    BOOST_CHECK_SMALL(std::abs( .144599342346E+04 - eph.zn_dot),  1E-08);
    BOOST_CHECK_SMALL(std::abs(-.186264514923E-05 - eph.zn_ddot), 1E-17);
    BOOST_CHECK_SMALL(std::abs( .300000000000E+01 - eph.E_n),     1E-11);
  }

  std::string dist;
  {
    std::stringstream ss;
    writer_t writer(ss);
    writer.header()["PGM / RUN BY / DATE"]
        = "ASRINEXG V1.1.0 VM  AIUB                19-FEB-98 10:42";
    writer.header()["COMMENT"] = "STATION ZIMMERWALD";
    //writer.header()["CORR TO SYSTEM TIME"] << "  1998     2    16    0.379979610443D-06";

    writer_t::space_node_list_t space_nodes = {NULL};
    space_nodes.glonass = &glonass;
    writer.write_all(space_nodes, 211);
    dist = ss.str();
  }

  //compare_lines(src, dist, 5); // TODO check header
}

BOOST_AUTO_TEST_CASE(nav_GLONASS_v3){
  const char *src = \
      "     3.04           N: GNSS NAV DATA    M: MIXED            RINEX VERSION / TYPE\n"
      "XXRINEXN V3         AIUB                20061002 000123 UTC PGM / RUN BY / DATE \n"
      "EXAMPLE OF VERSION 3.04 FORMAT                              COMMENT             \n"
      "GPSA   0.1025E-07  0.7451E-08 -0.5960E-07  -0.5960E-07      IONOSPHERIC CORR    \n"
      "GPSB   0.8806E+05  0.0000E+00 -0.1966E+06  -0.6554E+05      IONOSPHERIC CORR    \n"
      "GPUT  0.2793967723E-08 0.000000000E+00 147456 1395    G10 2 TIME SYSTEM CORR    \n"
      "GLUT  0.7823109626E-06 0.000000000E+00      0 1395    R10 0 TIME SYSTEM CORR    \n"
      "    14                                                      LEAP SECONDS        \n"
      "                                                            END OF HEADER       \n"
      "R01 2006 10 01 00 15 00-0.137668102980E-04-0.454747350886E-11 0.900000000000E+02\n"
      "     0.157594921875E+05-0.145566368103E+01 0.000000000000E+00 0.000000000000E+00\n"
      "    -0.813711474609E+04 0.205006790161E+01 0.931322574615E-09 0.700000000000E+01\n"
      "     0.183413398438E+05 0.215388488770E+01-0.186264514923E-08 0.100000000000E+01\n";
     //----|---1|0---|---2|0---|---3|0---|---4|0---|---5|0---|---6|0---|---7|0---|---8|

  typedef RINEX_NAV_Reader<fnum_t> reader_t;
  typedef RINEX_NAV_Writer<fnum_t> writer_t;

  check_reader_versatility_to_input<reader_t>(src);

  gps_t gps;
  glonass_t glonass;
  {
    std::stringbuf sbuf(src);
    std::istream in(&sbuf);
    reader_t::space_node_list_t space_nodes = {&gps};
    space_nodes.glonass = &glonass;
    reader_t::read_all(in, space_nodes);
  }

  {
    std::tm t_oc_tm = {0, 15, 0, 1, 10 - 1, 2006 - 1900};
    /*typename*/ gps_t::gps_time_t t_oc(t_oc_tm, 14);
    glonass.satellite(1).select_ephemeris(t_oc);
    const /*typename*/ glonass_t::Satellite::Ephemeris &eph(glonass.satellite(1).ephemeris());

    BOOST_CHECK_SMALL(std::abs( .137668102980E-04 - eph.tau_n),   1E-16); // tau_n is inverted
    BOOST_CHECK_SMALL(std::abs(-.454747350886E-11 - eph.gamma_n), 1E-23);
    BOOST_CHECK_SMALL(std::abs( .900000000000E+02 - eph.t_k),     1E-10);

    BOOST_CHECK_SMALL(std::abs( .157594921875E+08 - eph.xn),      1E-04);
    BOOST_CHECK_SMALL(std::abs(-.145566368103E+04 - eph.xn_dot),  1E-08);
    BOOST_CHECK_SMALL(std::abs( .000000000000E+03 - eph.xn_ddot), 1E-09);
    BOOST_CHECK_SMALL(std::abs( .000000000000E+00 - eph.B_n),     1E-12);

    BOOST_CHECK_SMALL(std::abs(-.813711474609E+07 - eph.yn),      1E-05);
    BOOST_CHECK_SMALL(std::abs( .205006790161E+04 - eph.yn_dot),  1E-08);
    BOOST_CHECK_SMALL(std::abs( .931322574615E-06 - eph.yn_ddot), 1E-18);
    BOOST_CHECK_SMALL(std::abs( .700000000000E+01 - eph.freq_ch), 1E-11);

    BOOST_CHECK_SMALL(std::abs( .183413398438E+08 - eph.zn),      1E-04);
    BOOST_CHECK_SMALL(std::abs( .215388488770E+04 - eph.zn_dot),  1E-08);
    BOOST_CHECK_SMALL(std::abs(-.186264514923E-05 - eph.zn_ddot), 1E-17);
    BOOST_CHECK_SMALL(std::abs( .100000000000E+01 - eph.E_n),     1E-11);

    // default setting until ver.3.05
    BOOST_CHECK_EQUAL(1, eph.M); // GLONASS-M
    BOOST_CHECK_EQUAL(60 * 60, eph.P1); // 1 hour
    BOOST_CHECK_EQUAL(1024, eph.F_T); // invalid range accuracy, thus max(512) * 2 = 1024 will be returned.
    BOOST_CHECK_EQUAL(0, eph.delta_tau_n);
  }

  std::string dist;
  {
    std::stringstream ss;
    writer_t writer(ss);
    //writer.header()["PGM / RUN BY / DATE"]
    //    = "XXRINEXN V3         AIUB                20061002 000123 UTC";
    std::tm t_header = {23, 1, 0, 2, 10 - 1, 2006 - 1900};
    writer.pgm_runby_date("XXRINEXN V3", "AIUB", t_header, "UTC");
    writer.header()["COMMENT"] << "EXAMPLE OF VERSION 3.04 FORMAT";
#if 0
    writer.header()["TIME SYSTEM CORR"]
        << "GPUT  0.2793967723E-08 0.000000000E+00 147456 1395    G10 2" //writer.utc_params(gps);
        << "GLUT  0.7823109626E-06 0.000000000E+00      0 1395    R10 0";
    writer.header()["LEAP SECONDS"] = "    14"; // writer.leap_seconds(gps);
#endif

    writer_t::space_node_list_t space_nodes = {&gps};
    space_nodes.glonass = &glonass;
    writer.write_all(space_nodes, 304);
    dist = ss.str();
  }

  //compare_lines(src, dist, 9); // TODO check header
}

BOOST_AUTO_TEST_CASE(nav_GLONASS_v305){
  const char *src = \
      "     3.05           N: GNSS NAV DATA    M: MIXED            RINEX VERSION / TYPE\n"
      "XXRINEXN V3         AIUB                20061002 000123 UTC PGM / RUN BY / DATE \n"
      "EXAMPLE OF VERSION 3.05 FORMAT                              COMMENT             \n"
      "GPSA   0.1025E-07  0.7451E-08 -0.5960E-07  -0.5960E-07      IONOSPHERIC CORR    \n"
      "GPSB   0.8806E+05  0.0000E+00 -0.1966E+06  -0.6554E+05      IONOSPHERIC CORR    \n"
      "GPUT  0.2793967723E-08 0.000000000E+00 147456 1395    G10 2 TIME SYSTEM CORR    \n"
      "GLUT  0.7823109626E-06 0.000000000E+00      0 1395    R10 0 TIME SYSTEM CORR    \n"
      "    14                                                      LEAP SECONDS        \n"
      "                                                            END OF HEADER       \n"
      "R01 2006 10 01 00 15 00-0.137668102980E-04-0.454747350886E-11 0.900000000000E+02\n"
      "     0.157594921875E+05-0.145566368103E+01 0.000000000000E+00 0.000000000000E+00\n"
      "    -0.813711474609E+04 0.205006790161E+01 0.931322574615E-09 0.700000000000E+01\n"
      "     0.183413398438E+05 0.215388488770E+01-0.186264514923E-08 0.100000000000E+01\n"
      "     1.790000000000E+02 8.381903171539E-09 2.000000000000E+00 3.000000000000E+00\n";
     //----|---1|0---|---2|0---|---3|0---|---4|0---|---5|0---|---6|0---|---7|0---|---8|

  typedef RINEX_NAV_Reader<fnum_t> reader_t;
  typedef RINEX_NAV_Writer<fnum_t> writer_t;

  check_reader_versatility_to_input<reader_t>(src);

  gps_t gps;
  glonass_t glonass;
  {
    std::stringbuf sbuf(src);
    std::istream in(&sbuf);
    reader_t::space_node_list_t space_nodes = {&gps};
    space_nodes.glonass = &glonass;
    reader_t::read_all(in, space_nodes);
  }

  {
    std::tm t_oc_tm = {0, 15, 0, 1, 10 - 1, 2006 - 1900};
    /*typename*/ gps_t::gps_time_t t_oc(t_oc_tm);
    glonass.satellite(1).select_ephemeris(t_oc);
    const /*typename*/ glonass_t::Satellite::Ephemeris &eph(glonass.satellite(1).ephemeris());

    BOOST_CHECK_SMALL(std::abs( .137668102980E-04 - eph.tau_n),   1E-16); // tau_n is inverted
    BOOST_CHECK_SMALL(std::abs(-.454747350886E-11 - eph.gamma_n), 1E-23);
    BOOST_CHECK_SMALL(std::abs( .900000000000E+02 - eph.t_k),     1E-10);

    BOOST_CHECK_SMALL(std::abs( .157594921875E+08 - eph.xn),      1E-04);
    BOOST_CHECK_SMALL(std::abs(-.145566368103E+04 - eph.xn_dot),  1E-08);
    BOOST_CHECK_SMALL(std::abs( .000000000000E+03 - eph.xn_ddot), 1E-09);
    BOOST_CHECK_SMALL(std::abs( .000000000000E+00 - eph.B_n),     1E-12);

    BOOST_CHECK_SMALL(std::abs(-.813711474609E+07 - eph.yn),      1E-05);
    BOOST_CHECK_SMALL(std::abs( .205006790161E+04 - eph.yn_dot),  1E-08);
    BOOST_CHECK_SMALL(std::abs( .931322574615E-06 - eph.yn_ddot), 1E-18);
    BOOST_CHECK_SMALL(std::abs( .700000000000E+01 - eph.freq_ch), 1E-11);

    BOOST_CHECK_SMALL(std::abs( .183413398438E+08 - eph.zn),      1E-04);
    BOOST_CHECK_SMALL(std::abs( .215388488770E+04 - eph.zn_dot),  1E-08);
    BOOST_CHECK_SMALL(std::abs(-.186264514923E-05 - eph.zn_ddot), 1E-17);
    BOOST_CHECK_SMALL(std::abs( .100000000000E+01 - eph.E_n),     1E-11);

    BOOST_CHECK_EQUAL(1, eph.M); // GLONASS-M
    BOOST_CHECK_EQUAL(0, eph.P1);
    BOOST_CHECK_EQUAL(2, eph.F_T_index());
    BOOST_CHECK_SMALL(std::abs(8.381903171539E-09 - eph.delta_tau_n), 1E-22);
  }

  std::string dist;
  {
    std::stringstream ss;
    writer_t writer(ss);
    //writer.header()["PGM / RUN BY / DATE"]
    //    = "XXRINEXN V3         AIUB                20061002 000123 UTC";
    std::tm t_header = {23, 1, 0, 2, 10 - 1, 2006 - 1900};
    writer.pgm_runby_date("XXRINEXN V3", "AIUB", t_header, "UTC");
    writer.header()["COMMENT"] << "EXAMPLE OF VERSION 3.05 FORMAT";

    writer_t::space_node_list_t space_nodes = {&gps};
    space_nodes.glonass = &glonass;
    writer.write_all(space_nodes, 305);
    dist = ss.str();
  }

  //compare_lines(src, dist, 9); // TODO check header
}

BOOST_AUTO_TEST_CASE(obs_GPS_v2){
  const char *src = \
      "     2.11           OBSERVATION DATA    M (MIXED)           RINEX VERSION / TYPE\n"
      "BLANK OR G = GPS,  R = GLONASS,  E = GALILEO,  M = MIXED    COMMENT             \n"
      "XXRINEXO V9.9       AIUB                24-MAR-01 14:43     PGM / RUN BY / DATE \n"
      "EXAMPLE OF A MIXED RINEX FILE (NO FEATURES OF V 2.11)       COMMENT             \n"
      "A 9080                                                      MARKER NAME         \n"
      "9080.1.34                                                   MARKER NUMBER       \n"
      "BILL SMITH          ABC INSTITUTE                           OBSERVER / AGENCY   \n"
      "X1234A123           XX                  ZZZ                 REC # / TYPE / VERS \n"
      "234                 YY                                      ANT # / TYPE        \n"
      "  4375274.       587466.      4589095.                      APPROX POSITION XYZ \n"
      "         .9030         .0000         .0000                  ANTENNA: DELTA H/E/N\n"
      "     1     1                                                WAVELENGTH FACT L1/2\n"
      "     1     2     6   G14   G15   G16   G17   G18   G19      WAVELENGTH FACT L1/2\n"
      "     0                                                      RCV CLOCK OFFS APPL \n"
      "     5    P1    L1    L2    P2    L5                        # / TYPES OF OBSERV \n"
      "    18.000                                                  INTERVAL            \n"
      "  2005     3    24    13    10   36.0000000                 TIME OF FIRST OBS   \n"
      "                                                            END OF HEADER       \n"
      " 05  3 24 13 10 36.0000000  0  4G12G09G06E11                         -.123456789\n"
      "  23629347.915            .300 8         -.353    23629364.158                  \n"
      "  20891534.648           -.120 9         -.358    20891541.292                  \n"
      "  20607600.189           -.430 9          .394    20607605.848                  \n"
      "                          .324 8                                          .178 7\n"
      " 05  3 24 13 10 50.0000000  4  4                                                \n"
      "     1     2     2   G 9   G12                              WAVELENGTH FACT L1/2\n"
      "  *** WAVELENGTH FACTOR CHANGED FOR 2 SATELLITES ***        COMMENT             \n"
      "      NOW 8 SATELLITES HAVE WL FACT 1 AND 2!                COMMENT             \n"
      "                                                            COMMENT             \n"
      " 05  3 24 13 10 54.0000000  0  6G12G09G06R21R22E11                   -.123456789\n"
      "  23619095.450      -53875.632 8    -41981.375    23619112.008                  \n"
      "  20886075.667      -28688.027 9    -22354.535    20886082.101                  \n"
      "  20611072.689       18247.789 9     14219.770    20611078.410                  \n"
      "  21345678.576       12345.567 5                                                \n"
      "  22123456.789       23456.789 5                                                \n"
      "                     65432.123 5                                     48861.586 7\n";
     //----|---1|0---|---2|0---|---3|0---|---4|0---|---5|0---|---6|0---|---7|0---|---8|

  typedef RINEX_OBS_Reader<fnum_t> reader_t;
  typedef RINEX_OBS_Writer<fnum_t> writer_t;

  check_reader_versatility_to_input<reader_t>(src);

  {
    std::stringbuf sbuf(src);
    std::istream in(&sbuf);
    reader_t reader(in);

    {
      static const char *types[] = {"P1", "L1", "L2", "P2", "L5"};
      for(unsigned i(0); i < sizeof(types)/ sizeof(types[0]); ++i){
        BOOST_CHECK_EQUAL(reader.observed_index(types[i]), i);
      }
    }

    BOOST_CHECK(reader.has_next());
    {
      reader_t::observation_t obs(reader.next());

      std::tm t(obs.t_epoch.c_tm());
      BOOST_CHECK_EQUAL(t.tm_year, 2005 - 1900);
      BOOST_CHECK_EQUAL(t.tm_mon,  3 - 1);
      BOOST_CHECK_EQUAL(t.tm_mday, 24);
      BOOST_CHECK_EQUAL(t.tm_hour, 13);
      BOOST_CHECK_EQUAL(t.tm_min, 10);
      BOOST_CHECK_EQUAL(t.tm_sec, 36);

      BOOST_CHECK_SMALL(std::abs(-.123456789 - obs.receiver_clock_error), 1E-10);
      BOOST_CHECK_EQUAL(obs.per_satellite.size(), 4);
      BOOST_CHECK_EQUAL(obs.per_satellite[12].size(), 5);
      BOOST_CHECK_EQUAL(obs.per_satellite[ 9].size(), 5);
      BOOST_CHECK_EQUAL(obs.per_satellite[ 6].size(), 5);
      BOOST_CHECK_EQUAL(obs.per_satellite[11 + 0x200].size(), 5);

      BOOST_CHECK(obs.per_satellite[12][0].valid);
      BOOST_CHECK(obs.per_satellite[12][1].valid);
      BOOST_CHECK_SMALL(std::abs(.300 - obs.per_satellite[12][1].value), 1E-4);
      BOOST_CHECK_EQUAL(obs.per_satellite[12][1].lli, 0);
      BOOST_CHECK_EQUAL(obs.per_satellite[12][1].ss, 8);
      BOOST_CHECK(obs.per_satellite[12][2].valid);
      BOOST_CHECK(obs.per_satellite[12][3].valid);
      BOOST_CHECK(!obs.per_satellite[12][4].valid);

      std::stringstream ss;
      writer_t writer(ss);
      writer.set_version(210);
      writer << obs;
      compare_lines(
          " 05  3 24 13 10 36.0000000  0  4G06G09G12E11                         -.123456789\n"
          "  20607600.189           -.430 9          .394    20607605.848                  \n"
          "  20891534.648           -.120 9         -.358    20891541.292                  \n"
          "  23629347.915            .300 8         -.353    23629364.158                  \n"
          "                          .324 8                                          .178 7\n",
          ss.str());
    }
    BOOST_CHECK(reader.has_next());
    {
      reader_t::observation_t obs(reader.next());

      std::tm t(obs.t_epoch.c_tm());
      BOOST_CHECK_EQUAL(t.tm_year, 2005 - 1900);
      BOOST_CHECK_EQUAL(t.tm_mon,  3 - 1);
      BOOST_CHECK_EQUAL(t.tm_mday, 24);
      BOOST_CHECK_EQUAL(t.tm_hour, 13);
      BOOST_CHECK_EQUAL(t.tm_min, 10);
      BOOST_CHECK_EQUAL(t.tm_sec, 54);

      BOOST_CHECK_EQUAL(obs.per_satellite.size(), 6);
      BOOST_CHECK_EQUAL(obs.per_satellite[12].size(), 5);
      BOOST_CHECK_EQUAL(obs.per_satellite[ 9].size(), 5);
      BOOST_CHECK_EQUAL(obs.per_satellite[ 6].size(), 5);
      BOOST_CHECK_EQUAL(obs.per_satellite[21 + 0x100].size(), 5);
      BOOST_CHECK_EQUAL(obs.per_satellite[22 + 0x100].size(), 5);
      BOOST_CHECK_EQUAL(obs.per_satellite[11 + 0x200].size(), 5);
    }
    BOOST_CHECK(!reader.has_next());
  }
}

BOOST_AUTO_TEST_CASE(obs_GPS_v3){
  const char *src = \
      "     3.04           OBSERVATION DATA    M                   RINEX VERSION / TYPE\n"
      "G = GPS  R = GLONASS  E = GALILEO  S = GEO  M = MIXED       COMMENT             \n"
      "XXRINEXO V9.9       AIUB                20060324 144333 UTC PGM / RUN BY / DATE \n"
      "EXAMPLE OF A MIXED RINEX FILE VERSION 3.04                  COMMENT             \n"
      "The file contains L1 pseudorange and phase data of the      COMMENT             \n"
      "geostationary AOR-E satellite (PRN 120 = S20)               COMMENT             \n"
      "A 9080                                                      MARKER NAME         \n"
      "9080.1.34                                                   MARKER NUMBER       \n"
      "BILL SMITH          ABC INSTITUTE                           OBSERVER / AGENCY   \n"
      "X1234A123           GEODETIC            1.3.1               REC # / TYPE / VERS \n"
      "G1234               ROVER                                   ANT # / TYPE        \n"
      "  4375274.       587466.      4589095.                      APPROX POSITION XYZ \n"
      "         .9030         .0000         .0000                  ANTENNA: DELTA H/E/N\n"
      "     0                                                      RCV CLOCK OFFS APPL \n"
      "G    5 C1C L1W L2W C1W S2W                                  SYS / # / OBS TYPES \n"
      "R    2 C1C L1C                                              SYS / # / OBS TYPES \n"
      "E    2 L1B L5I                                              SYS / # / OBS TYPES \n"
      "S    2 C1C L1C                                              SYS / # / OBS TYPES \n"
      "    18.000                                                  INTERVAL            \n"
      "G APPL_DCB          xyz.uvw.abc//pub/dcb_gps.dat            SYS / DCBS APPLIED  \n"
      "DBHZ                                                        SIGNAL STRENGTH UNIT\n"
      "  2006    03    24    13    10   36.0000000     GPS         TIME OF FIRST OBS   \n"
      " 18 R01  1 R02  2 R03  3 R04  4 R05  5 R06 -6 R07 -5 R08 -4 GLONASS SLOT / FRQ #\n"
      "    R09 -3 R10 -2 R11 -1 R12  0 R13  1 R14  2 R15  3 R16  4 GLONASS SLOT / FRQ #\n"
      "    R17  5 R18 -5                                           GLONASS SLOT / FRQ #\n"
      "G L1C                                                       SYS / PHASE SHIFT   \n"
      "G L1W  0.00000                                              SYS / PHASE SHIFT   \n"
      "G L2W                                                       SYS / PHASE SHIFT   \n"
      "R L1C                                                       SYS / PHASE SHIFT   \n"
      "E L1B                                                       SYS / PHASE SHIFT   \n"
      "E L5I                                                       SYS / PHASE SHIFT   \n"
      "S L1C                                                       SYS / PHASE SHIFT   \n"
      " C1C  -10.000 C1P  -10.123 C2C  -10.432 C2P  -10.634        GLONASS COD/PHS/BIS \n"
      "                                                            END OF HEADER       \n"
      "> 2006 03 24 13 10 36.0000000  0  5      -0.123456789012                        \n"
      "G06  23629347.915            .300 8         -.353 4  23629347.158          24.158  \n"
      "G09  20891534.648           -.120 9         -.358 6  20891545.292          38.123  \n"
      "G12  20607600.189           -.430 9          .394 5  20607600.848          35.234  \n"
      "E11          .324 8          .178 7\n"
      "S20  38137559.506      335849.135 9\n"
      "> 2006 03 24 13 10 54.0000000  0  7      -0.123456789210                        \n"
      "G06  23619095.450      -53875.632 8    -41981.375 4  23619095.008          25.234  \n"
      "G09  20886075.667      -28688.027 9    -22354.535 7  20886076.101          42.231  \n"
      "G12  20611072.689       18247.789 9     14219.770 6  20611072.410          36.765  \n"
      "R21  21345678.576       12345.567 5\n"
      "R22  22123456.789       23456.789 5\n"
      "E11     65432.123 5     48861.586 7\n"
      "S20  38137559.506      335849.135 9\n"
      "> 2006 03 24 13 11 12.0000000  2  2                                             \n"
      "      *** FROM NOW ON KINEMATIC DATA! ***                   COMMENT             \n"
      "      TWO COMMENT LINES FOLLOW DIRECTLY THE EVENT RECORD    COMMENT             \n";
     //----|---1|0---|---2|0---|---3|0---|---4|0---|---5|0---|---6|0---|---7|0---|---8|

  typedef RINEX_OBS_Reader<fnum_t> reader_t;
  typedef RINEX_OBS_Writer<fnum_t> writer_t;

  check_reader_versatility_to_input<reader_t>(src);

  {
    std::stringbuf sbuf(src);
    std::istream in(&sbuf);
    reader_t reader(in);

    {
      static const char *types[] = {"C1C", "L1W", "L2W", "C1W", "S2W"};
      for(unsigned i(0); i < sizeof(types)/ sizeof(types[0]); ++i){
        BOOST_CHECK_EQUAL(reader.observed_index(types[i], 'G'), i);
      }
    }
    {
      static const char *types[] = {"C1C", "L1C"};
      for(unsigned i(0); i < sizeof(types)/ sizeof(types[0]); ++i){
        BOOST_CHECK_EQUAL(reader.observed_index(types[i], 'R'), i);
      }
    }
    {
      static const char *types[] = {"L1B", "L5I"};
      for(unsigned i(0); i < sizeof(types)/ sizeof(types[0]); ++i){
        BOOST_CHECK_EQUAL(reader.observed_index(types[i], 'E'), i);
      }
    }
    {
      static const char *types[] = {"C1C", "L1C"};
      for(unsigned i(0); i < sizeof(types)/ sizeof(types[0]); ++i){
        BOOST_CHECK_EQUAL(reader.observed_index(types[i], 'S'), i);
      }
    }

    BOOST_CHECK(reader.has_next());
    {
      reader_t::observation_t obs(reader.next());

      std::tm t(obs.t_epoch.c_tm());
      BOOST_CHECK_EQUAL(t.tm_year, 2006 - 1900);
      BOOST_CHECK_EQUAL(t.tm_mon,  3 - 1);
      BOOST_CHECK_EQUAL(t.tm_mday, 24);
      BOOST_CHECK_EQUAL(t.tm_hour, 13);
      BOOST_CHECK_EQUAL(t.tm_min, 10);
      BOOST_CHECK_EQUAL(t.tm_sec, 36);

      BOOST_CHECK_SMALL(std::abs(-0.123456789012 - obs.receiver_clock_error), 1E-14);
      BOOST_CHECK_EQUAL(obs.per_satellite.size(), 5);
      BOOST_CHECK_EQUAL(obs.per_satellite[ 6].size(), 5);
      BOOST_CHECK_EQUAL(obs.per_satellite[ 9].size(), 5);
      BOOST_CHECK_EQUAL(obs.per_satellite[12].size(), 5);
      BOOST_CHECK_EQUAL(obs.per_satellite[11 + 0x200].size(), 2);
      BOOST_CHECK_EQUAL(obs.per_satellite[120].size(), 2);

      BOOST_CHECK(obs.per_satellite[6][0].valid);
      BOOST_CHECK(obs.per_satellite[6][1].valid);
      BOOST_CHECK_SMALL(std::abs(.300 - obs.per_satellite[6][1].value), 1E-4);
      BOOST_CHECK_EQUAL(obs.per_satellite[6][1].lli, 0);
      BOOST_CHECK_EQUAL(obs.per_satellite[6][1].ss, 8);
      BOOST_CHECK(obs.per_satellite[6][2].valid);
      BOOST_CHECK(obs.per_satellite[6][3].valid);
      BOOST_CHECK(obs.per_satellite[6][4].valid);
      BOOST_CHECK_SMALL(std::abs(24.158 - obs.per_satellite[6][4].value), 1E-4);

      std::stringstream ss;
      writer_t writer(ss);
      writer.set_version(304);
      writer << obs;
      compare_lines(
          "> 2006 03 24 13 10 36.0000000  0  5       -.123456789012                        \n"
          "G06  23629347.915            .300 8         -.353 4  23629347.158          24.158  \n"
          "G09  20891534.648           -.120 9         -.358 6  20891545.292          38.123  \n"
          "G12  20607600.189           -.430 9          .394 5  20607600.848          35.234  \n"
          "S20  38137559.506      335849.135 9\n"
          "E11          .324 8          .178 7\n",
          ss.str());
    }
    BOOST_CHECK(reader.has_next());
    {
      reader_t::observation_t obs(reader.next());

      std::tm t(obs.t_epoch.c_tm());
      BOOST_CHECK_EQUAL(t.tm_year, 2006 - 1900);
      BOOST_CHECK_EQUAL(t.tm_mon,  3 - 1);
      BOOST_CHECK_EQUAL(t.tm_mday, 24);
      BOOST_CHECK_EQUAL(t.tm_hour, 13);
      BOOST_CHECK_EQUAL(t.tm_min, 10);
      BOOST_CHECK_EQUAL(t.tm_sec, 54);

      BOOST_CHECK_EQUAL(obs.per_satellite.size(), 7);
      BOOST_CHECK_EQUAL(obs.per_satellite[ 6].size(), 5);
      BOOST_CHECK_EQUAL(obs.per_satellite[ 9].size(), 5);
      BOOST_CHECK_EQUAL(obs.per_satellite[12].size(), 5);
      BOOST_CHECK_EQUAL(obs.per_satellite[21 + 0x100].size(), 2);
      BOOST_CHECK_EQUAL(obs.per_satellite[22 + 0x100].size(), 2);
      BOOST_CHECK_EQUAL(obs.per_satellite[11 + 0x200].size(), 2);
      BOOST_CHECK_EQUAL(obs.per_satellite[120].size(), 2);
    }
    BOOST_CHECK(!reader.has_next());
  }
}

BOOST_AUTO_TEST_CASE(obs_GPS_v3_2){
  // GEONET Setagaya from https://terras.gsi.go.jp/data_service.php#11/35.663712/139.630394
  const char *src = \
      "     3.02           OBSERVATION DATA    M (MIXED)           RINEX VERSION / TYPE\n"
      "BINEX2RINEX  2.09   GSI, JAPAN          20211109 18:00:47UTCPGM / RUN BY / DATE\n"
      "0228                                                        MARKER NAME\n"
      "GEODETIC                                                    MARKER TYPE\n"
      "GSI, JAPAN          GEOSPATIAL INFORMATION AUTHORITY OF JAPAOBSERVER / AGENCY\n"
      "00000               TRIMBLE NETR9       5.37,21/SEP/2018    REC # / TYPE / VERS\n"
      "                    TRM59800.80     GSI                     ANT # / TYPE\n"
      " -3952590.4826  3360273.8960  3697987.2471                  APPROX POSITION XYZ\n"
      "        0.0000        0.0000        0.0000                  ANTENNA: DELTA H/E/N\n"
      "G   16 C1C L1C D1C S1C C2W L2W D2W S2W C2X L2X D2X S2X C5X  SYS / # / OBS TYPES\n"
      "       L5X D5X S5X                                          SYS / # / OBS TYPES\n"
      "R   16 C1C L1C D1C S1C C1P L1P D1P S1P C2C L2C D2C S2C C2P  SYS / # / OBS TYPES\n"
      "       L2P D2P S2P                                          SYS / # / OBS TYPES\n"
      "E   16 C1X L1X D1X S1X C5X L5X D5X S5X C7X L7X D7X S7X C8X  SYS / # / OBS TYPES\n"
      "       L8X D8X S8X                                          SYS / # / OBS TYPES\n"
      "J   16 C1C L1C D1C S1C C1X L1X D1X S1X C2X L2X D2X S2X C5X  SYS / # / OBS TYPES\n"
      "       L5X D5X S5X                                          SYS / # / OBS TYPES\n"
      "  2021    11     8     0     0    0.0000000     GPS         TIME OF FIRST OBS\n"
      "G L1C  0.00000  9 G02 G05 G06 G07 G09 G13 G15 G20 G30       SYS / PHASE SHIFT\n"
      "G L2W  0.00000  9 G02 G05 G06 G07 G09 G13 G15 G20 G30       SYS / PHASE SHIFT\n"
      "G L2X  0.00000  6 G05 G06 G07 G09 G15 G30                   SYS / PHASE SHIFT\n"
      "G L5X  0.00000  3 G06 G09 G30                               SYS / PHASE SHIFT\n"
      "R L1C  0.00000  9 R06 R07 R08 R09 R10 R16 R21 R22 R23       SYS / PHASE SHIFT\n"
      "R L1P  0.00000  9 R06 R07 R08 R09 R10 R16 R21 R22 R23       SYS / PHASE SHIFT\n"
      "R L2C  0.00000  5 R07 R08 R09 R21 R22                       SYS / PHASE SHIFT\n"
      "R L2P  0.00000  6 R07 R08 R09 R16 R21 R22                   SYS / PHASE SHIFT\n"
      "J L1C  0.00000  3 J01 J03 J07                               SYS / PHASE SHIFT\n"
      "J L1X  0.00000  3 J01 J03 J07                               SYS / PHASE SHIFT\n"
      "J L2X  0.00000  3 J01 J03 J07                               SYS / PHASE SHIFT\n"
      "J L5X  0.00000  3 J01 J03 J07                               SYS / PHASE SHIFT\n"
      "E L1X  0.00000  5 E03 E07 E08 E13 E15                       SYS / PHASE SHIFT\n"
      "E L5X  0.00000  5 E03 E07 E08 E13 E15                       SYS / PHASE SHIFT\n"
      "E L7X  0.00000  5 E03 E07 E08 E13 E15                       SYS / PHASE SHIFT\n"
      "E L8X  0.00000  5 E03 E07 E08 E13 E15                       SYS / PHASE SHIFT\n"
      " 23 R01  1 R02 -4 R03  5 R04  6 R05  1 R06 -4 R07  5 R08  6 GLONASS SLOT / FRQ #\n"
      "    R09 -2 R10 -7 R12 -1 R13 -2 R14 -7 R15  0 R16 -1 R17  4 GLONASS SLOT / FRQ #\n"
      "    R18 -3 R19  3 R20  2 R21  4 R22 -3 R23  3 R24  2        GLONASS SLOT / FRQ #\n"
      "                                                            GLONASS COD/PHS/BIS\n"
      "    30.000                                                  INTERVAL\n"
      "    18                                                      LEAP SECONDS\n"
      "smtt off - ms jumps in time tag (smooth phase and range)    COMMENT\n"
      "                                                            END OF HEADER\n"
      "> 2021 11 08 00 00  0.0000000  0 23\n"
      "G15  24141250.766   126863140.727 6      3509.254          39.700    24141258.2934   98854419.28945      2734.4844         33.1004   24141257.641    98854349.270 6      2734.484          36.400\n"
      "G13  21592361.766   113468747.405 7      2796.309          45.600    21592368.1684   88417155.44245      2178.9414         33.6004\n"
      "R08  21141687.664   113212791.929 6      1817.027          37.700    21141685.727   113212789.916 5      1817.027          34.900    21141689.770    88054417.588 5      1413.246          35.500    21141690.164    88054403.595 5      1413.246          35.000\n"
      "G30  22607567.063   118803569.517 6      1557.680          41.900    22607576.0204   92574267.45145      1213.7774         31.6004   22607576.523    92574238.467 7      1213.777          42.700    22607576.340    88716979.487 8      1163.203          49.500\n"
      "G07  23150925.211   121658956.840 7      -212.336          42.100    23150931.2544   94799227.45344      -165.4574         29.7004   23150931.125    94799165.467 6      -165.457          39.000\n"
      "G06  22171713.844   116513335.852 7     -2697.594          44.300    22171722.9304   90789462.20645     -2102.0204         34.5004   22171722.813    90789433.204 7     -2102.020          43.900    22171722.887    87006536.290 8     -2014.438          49.900\n"
      "R06  22209856.188   118516065.958 5     -3022.430          33.700    22209854.203   118516031.903 5     -3022.430          32.600\n"
      "J07  37300333.188   196014561.708 6         3.633          39.400    37300332.559   196014557.713 7         3.633          42.000    37300338.539   152738613.258 7         2.832          46.400    37300341.008   146374506.636 8         2.711          50.500\n"
      "J03  37061460.211   194759309.656 7       205.613          45.200    37061460.328   194759310.653 8       205.613          48.600    37061466.102   151760510.174 7       160.219          47.500    37061468.117   145437155.656 8       153.543          50.400\n"
      "J01  38349247.477   201526613.660 7      -635.609          45.800    38349248.148   201526661.650 8      -635.609          49.600    38349252.371   157033762.474 7      -495.281          46.700    38349256.379   150490689.061 8      -474.645          52.700\n"
      "G20  20448946.719   107460186.730 7       543.793          47.000    20448952.4884   83735046.00746       423.7344         40.3004\n"
      "G09  24515946.336   128832135.952 5     -3411.457          32.900    24515954.8524  100388697.06742     -2658.2774         16.9004   24515955.281   100388676.071 6     -2658.277          36.400    24515954.770    96205810.407 6     -2547.516          39.900\n"
      "R09  23958778.672   127938625.792 5      3149.793          35.400    23958778.984   127938569.764 5      3149.793          32.800    23958787.727    99507767.240 6      2449.836          38.800    23958787.813    99507781.234 6      2449.836          36.800\n"
      "G05  21540425.477   113195854.185 6      1667.867          41.700    21540431.6054   88204464.49946      1299.6374         37.7004   21540431.516    88204451.480 6      1299.637          39.200\n"
      "R07  19818698.492   106091061.520 8      -959.352          49.800    19818698.211   106091032.523 7      -959.352          47.000    19818702.402    82515254.409 7      -746.164          45.900    19818702.422    82515267.417 7      -746.164          45.200\n"
      "R22  19361400.602   103352467.484 6      -574.445          40.200    19361400.547   103352472.496 6      -574.445          39.000    19361405.000    80385311.919 6      -446.793          37.400    19361403.922    80385262.922 6      -446.793          37.100\n"
      "R21  21392541.445   114475803.643 7     -3886.512          47.500    21392540.141   114475784.654 7     -3886.512          43.300    21392544.566    89036721.583 6     -3022.844          41.400    21392544.973    89036717.591 6     -3022.844          41.100\n"
      "G02  20778663.938   109192674.650 8        56.312          49.200    20778667.4774   85085170.82247        43.8794         45.6004\n"
      "E13  23897972.344   125584693.247 7      1406.211          46.300    23897977.613    93780777.032 8      1050.094          50.800    23897976.484    96227231.990 8      1077.484          50.700    23897976.453    95004004.610 8      1063.789          53.700\n"
      "E15  23496321.383   123473982.577 8     -1379.035          48.500    23496327.297    92204612.387 8     -1029.797          51.200    23496326.746    94609947.145 8     -1056.664          51.700    23496326.844    93407278.370 9     -1043.230          54.400\n"
      "E08  21700316.023   114035932.451 8       613.961          50.100    21700319.844    85156715.590 8       458.477          51.500    21700319.078    87378196.565 8       470.438          52.200    21700319.105    86267453.182 9       464.457          54.800\n"
      "E07  26957428.852   141662246.158 6      2373.805          38.300    26957436.270   105786772.873 6      1772.645          40.800    26957433.875   108546424.129 6      1818.887          40.600    26957435.078   107166597.106 7      1795.766          43.400\n"
      "E03  24600836.375   129278261.655 7     -1785.637          47.500    24600842.266    96538976.110 8     -1333.430          49.700    24600841.684    99057372.918 8     -1368.215          50.300    24600841.180    97798176.121 8     -1350.820          52.900\n"
      "> 2021 11 08 00 00 30.0000000  0 23\n"
      "G15  24121230.914   126757939.545 6      3504.062          39.200    24121238.9104   98772444.38745      2730.4384         34.6004   24121238.426    98772374.369 6      2730.438          36.800\n"
      "G13  21576434.758   113385050.719 7      2783.562          45.500    21576441.3524   88351937.26045      2169.0084         33.6004\n"
      "R08  21131541.047   113158456.855 5      1805.328          33.700    21131538.480   113158454.834 5      1805.328          33.600    21131543.898    88012156.992 5      1404.145          35.800    21131543.188    88012142.986 5      1404.145          35.400\n"
      "G30  22598714.820   118757048.558 7      1543.625          46.000    22598724.0234   92538017.38445      1202.8244         33.3004   22598723.453    92537988.392 7      1202.824          44.500    22598723.578    88682239.862 8      1152.707          49.500\n"
      "G07  23152183.281   121665566.317 7      -228.418          44.000    23152188.3204   94804377.72345      -177.9884         30.2004   23152189.301    94804315.739 6      -177.988          39.200\n"
      "G06  22187144.461   116594431.721 7     -2708.754          43.100    22187155.3284   90852653.74345     -2110.7194         33.8004   22187154.828    90852624.749 7     -2110.719          44.800    22187155.465    87067094.812 8     -2022.770          50.100\n"
      "R06  22226849.156   118606745.289 5     -3022.691          35.200    22226849.117   118606711.326 5     -3022.691          33.400\n"
      "J07  37300312.398   196014452.751 6         3.680          39.000    37300311.727   196014448.760 7         3.680          42.300    37300317.621   152738528.350 7         2.867          46.600    37300320.199   146374425.259 8         2.746          50.500\n"
      "J03  37060293.117   194753175.578 7       203.258          44.900    37060292.984   194753176.578 8       203.258          48.700    37060298.727   151755730.372 7       158.383          47.300    37060300.949   145432575.012 8       151.785          50.500\n"
      "J01  38352873.492   201545667.512 7      -634.590          46.500    38352873.902   201545715.509 8      -634.590          49.600    38352877.934   157048609.630 7      -494.484          46.900    38352882.098   150504917.591 8      -473.883          52.700\n"
      "G20  20445863.383   107443982.586 7       536.562          47.500    20445868.7464   83722419.40046       418.1024         40.5004\n"
      "G09  24535425.164   128934496.990 5     -3412.754          34.800    24535433.6254  100468458.86143     -2659.2894         18.3004   24535434.020   100468437.908 6     -2659.289          36.500    24535433.430    96282248.851 6     -2548.484          41.000\n"
      "R09  23941095.367   127844188.260 6      3146.020          36.800    23941094.121   127844132.272 5      3146.020          34.000    23941100.992    99434315.852 6      2446.902          39.500    23941102.094    99434329.873 6      2446.902          38.400\n"
      "G05  21530920.023   113145902.593 7      1662.383          42.000    21530926.0204   88165541.20446      1295.3634         39.6004   21530925.715    88165528.207 6      1295.363          40.000\n"
      "R07  19824105.828   106120005.244 8      -970.172          50.500    19824105.695   106119976.261 7      -970.172          47.900    19824109.859    82537766.187 7      -754.578          46.600    19824108.941    82537779.194 7      -754.578          45.500\n"
      "R22  19364706.469   103370109.061 6      -601.539          41.000    19364705.574   103370114.073 6      -601.539          39.600    19364708.832    80399033.133 6      -467.863          36.900    19364708.883    80398984.149 6      -467.863          37.100\n"
      "R21  21414368.484   114592607.200 7     -3900.359          46.300    21414366.914   114592588.204 7     -3900.359          44.700    21414372.340    89127568.749 7     -3033.613          42.300    21414371.723    89127564.752 7     -3033.613          42.100\n"
      "G02  20778385.664   109191212.139 8        41.199          49.600    20778388.7704   85084031.17947        32.1024         45.5004\n"
      "E13  23889973.133   125542656.512 7      1396.105          46.400    23889978.098    93749385.957 8      1042.547          50.600    23889977.137    96195022.015 8      1069.742          50.800    23889977.109    94972204.086 8      1056.145          53.500\n"
      "E15  23504227.625   123515530.175 8     -1390.703          48.300    23504233.805    92235638.156 8     -1038.512          51.400    23504232.371    94641782.295 8     -1065.602          51.700    23504233.094    93438708.829 9     -1052.059          54.400\n"
      "E08  21696838.961   114017660.130 8       604.328          48.800    21696842.512    85143070.675 8       451.285          51.700    21696842.145    87364195.698 8       463.055          52.400    21696841.996    86253630.291 9       457.168          54.900\n"
      "E07  26943892.438   141591112.470 6      2368.535          40.000    26943899.965   105733653.609 6      1768.711          41.400    26943898.121   108491919.110 6      1814.852          41.800    26943898.691   107112784.964 7      1791.781          44.700\n"
      "E03  24611044.516   129331904.875 7     -1790.676          47.500    24611049.594    96579034.358 8     -1337.195          49.400    24611049.215    99098476.154 8     -1372.078          50.100    24611049.168    97838756.863 8     -1354.637          52.600\n";
     //----|---1|0---|---2|0---|---3|0---|---4|0---|---5|0---|---6|0---|---7|0---|---8|

  typedef RINEX_OBS_Reader<fnum_t> reader_t;
  //typedef RINEX_OBS_Writer<fnum_t> writer_t;

  check_reader_versatility_to_input<reader_t>(src);
}

BOOST_AUTO_TEST_CASE(clk_GPS_v3){
  const char *src = \
      "     3.00           CLOCK DATA          GPS                 RINEX VERSION / TYPE\n"
      "TORINEXC V9.9       USNO                19960403 001000 UTC PGM / RUN BY / DATE \n"
      "EXAMPLE OF A CLOCK DATA ANALYSIS FILE                       COMMENT             \n"
      "IN THIS CASE ANALYSIS RESULTS FROM GPS ONLY ARE INCLUDED    COMMENT             \n"
      "No re-alignment of the clocks has been applied.             COMMENT             \n"
      "G    4 C1W L1W C2W L2W                                      SYS / # / OBS TYPES \n"
      "   GPS                                                      TIME SYSTEM ID      \n"
      "    10                                                      LEAP SECONDS        \n"
      "G CC2NONCC          p1c1bias.hist @ goby.nrl.navy.mil       SYS / DCBS APPLIED  \n"
      "G PAGES             igs05.atx @ igscb.jpl.nasa.gov          SYS / PCVS APPLIED  \n"
      "     2    AS    AR                                          # / TYPES OF DATA   \n"
      "USN  USNO USING GIPSY/OASIS-II                              ANALYSIS CENTER     \n"
      "     1 1994 07 14  0  0  0.000000 1994 07 14 20 59  0.000000# OF CLK REF        \n"
      "USNO 40451S003                           -.123456789012E+00 ANALYSIS CLK REF    \n"
      "     1 1994 07 14 21  0  0.000000 1994 07 14 21 59  0.000000# OF CLK REF        \n"
      "TIDB 50103M108                          -0.123456789012E+00 ANALYSIS CLK REF    \n"
      "     4    ITRF96                                            # OF SOLN STA / TRF \n"
      "GOLD 40405S031            1234567890 -1234567890 -1234567890SOLN STA NAME / NUM \n"
      "AREQ 42202M005           -1234567890  1234567890 -1234567890SOLN STA NAME / NUM \n"
      "TIDB 50103M108            1234567890 -1234567890  1234567890SOLN STA NAME / NUM \n"
      "HARK 30302M007           -1234567890  1234567890 -1234567890SOLN STA NAME / NUM \n"
      "USNO 40451S003            1234567890 -1234567890 -1234567890SOLN STA NAME / NUM \n"
      "    27                                                      # OF SOLN SATS      \n"
      "G01 G02 G03 G04 G05 G06 G07 G08 G09 G10 G13 G14 G15 G16 G17 PRN LIST            \n"
      "G18 G19 G21 G22 G23 G24 G25 G26 G27 G29 G30 G31             PRN LIST            \n"
      "                                                            END OF HEADER       \n"
      "AR AREQ 1994 07 14 20 59  0.000000  6   -0.123456789012E+00 -0.123456789012E+01 \n"
      "-0.123456789012E+02 -0.123456789012E+03 -0.123456789012E+04 -0.123456789012E+05 \n"
      "AS G16  1994 07 14 20 59  0.000000  2    -.123456789012E+00  -.123456789012E-01 \n"
      "AR GOLD 1994 07 14 20 59  0.000000  4    -.123456789012E-01  -.123456789012E-02 \n"
      " -.123456789012E-03  -.123456789012E-04                                         \n"
      "AR HARK 1994 07 14 20 59  0.000000  2     .123456789012E+00   .123456789012E+00 \n"
      "AR TIDB 1994 07 14 20 59  0.000000  6     .123456789012E+00   .123456789012E+00 \n"
      "  .123456789012E+00   .123456789012E+00   .123456789012E+00   .123456789012E+00 \n";
     //----|---1|0---|---2|0---|---3|0---|---4|0---|---5|0---|---6|0---|---7|0---|---8|

  typedef RINEX_CLK_Reader<fnum_t> reader_t;

  check_reader_versatility_to_input<reader_t>(src);

  {
    std::stringbuf sbuf(src);
    std::istream in(&sbuf);
    RINEX_CLK<fnum_t>::collection_t collection;
    BOOST_CHECK_EQUAL(reader_t::read_all(in, collection), 5);
    RINEX_CLK<fnum_t>::satellites_t sats;
    BOOST_CHECK_EQUAL(reader_t::read_all(in.seekg(0), sats), 1);
  }
}

BOOST_AUTO_TEST_CASE(clk_GPS_v304){
  const char *src = \
      "3.04                 C                    G                      RINEX VERSION / TYPE\n"
      "TORINEXC V9.9        USNO                 19960403  001000 UTC   PGM / RUN BY / DATE\n"
      "EXAMPLE OF A CLOCK DATA ANALYSIS FILE                            COMMENT\n"
      "IN THIS CASE ANALYSIS RESULTS FROM GPS ONLY ARE INCLUDED         COMMENT\n"
      "No re-alignment of the clocks has been applied.                  COMMENT\n"
      "G    4  C1W L1W C2W L2W                                          SYS / # / OBS TYPES\n"
      "   GPS                                                           TIME SYSTEM ID\n"
      "    10                                                           LEAP SECONDS\n"
      "G CC2NONCC          p1c1bias.hist @ goby.nrl.navy.mil            SYS / DCBS APPLIED\n"
      "G PAGES             igs05.atx @ igscb.jpl.nasa.gov               SYS / PCVS APPLIED \n"
      "     2    AS    AR                                               # / TYPES OF DATA\n"
      "USN  USNO USING GIPSY/OASIS-II                                   ANALYSIS CENTER\n"
      "     1 1994 07 14  0  0  0.000000 1994 07 14 20 59  0.000000     # OF CLK REF\n"
      "USNO      40451S003                           -.123456789012E+00 ANALYSIS CLK REF\n"
      "     1 1994 07 14 21  0  0.000000 1994 07 14 21 59  0.000000     # OF CLK REF\n"
      "TIDB      50103M108                          -0.123456789012E+00 ANALYSIS CLK REF\n"
      "     4    ITRF96                                                 # OF SOLN STA / TRF\n"
      "GOLD      40405S031            1234567890 -1234567890 -1234567890SOLN STA NAME / NUM\n"
      "AREQ      42202M005           -1234567890  1234567890 -1234567890SOLN STA NAME / NUM\n"
      "TIDB      50103M108            1234567890 -1234567890  1234567890SOLN STA NAME / NUM\n"
      "HARK      30302M007           -1234567890  1234567890 -1234567890SOLN STA NAME / NUM\n"
      "USNO      40451S003            1234567890 -1234567890 -1234567890SOLN STA NAME / NUM\n"
      "    27                                                           # OF SOLN SATS \n"
      "G01 G02 G03 G04 G05 G06 G07 G08 G09 G10 G13 G14 G15 G16 G17 G18  PRN LIST\n"
      "G19 G21 G22 G23 G24 G25 G26 G27 G29 G30 G31                      PRN LIST\n"
      "                                                                 END OF HEADER\n"
      "AR AREQ00USA 1994 07 14 20 59  0.000000  6   -0.123456789012E+00  -0.123456789012E+01\n"
      "   -0.123456789012E+02  -0.123456789012E+03  -0.123456789012E+04  -0.123456789012E+05\n"
      "AS G16       1994 07 14 20 59  0.000000  2   -0.123456789012E+00  -0.123456789012E-01\n"
      "AR GOLD      1994 07 14 20 59  0.000000  4   -0.123456789012E-01  -0.123456789012E-02\n"
      "   -0.123456789012E-03  -0.123456789012E-04\n"
      "AR HARK      1994 07 14 20 59  0.000000  2    0.123456789012E+00   0.123456789012E+00\n"
      "AR TIDB      1994 07 14 20 59  0.000000  6    0.123456789012E+00   0.123456789012E+00\n"
      "    0.123456789012E+00   0.123456789012E+00   0.123456789012E+00   0.123456789012E+00\n";
     //----|---1|0---|---2|0---|---3|0---|---4|0---|---5|0---|---6|0---|---7|0---|---8|

  typedef RINEX_CLK_Reader<fnum_t> reader_t;

  check_reader_versatility_to_input<reader_t>(src);

  {
    std::stringbuf sbuf(src);
    std::istream in(&sbuf);
    RINEX_CLK<fnum_t>::collection_t collection;
    BOOST_CHECK_EQUAL(reader_t::read_all(in, collection), 5);
    RINEX_CLK<fnum_t>::satellites_t sats;
    BOOST_CHECK_EQUAL(reader_t::read_all(in.seekg(0), sats), 1);
  }
}

BOOST_AUTO_TEST_SUITE_END()
