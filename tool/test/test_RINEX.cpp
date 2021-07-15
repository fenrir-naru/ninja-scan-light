#include <iostream>
#include <string>
#include <sstream>

#include "navigation/RINEX.h"
#include "navigation/GPS.h"

#include <boost/random.hpp>
#include <boost/random/random_device.hpp>

#include <boost/format.hpp>
#include <boost/cstdint.hpp>

#define BOOST_TEST_MAIN
#include <boost/test/included/unit_test.hpp>

using namespace std;
using boost::format;

typedef double float_t;
typedef GPS_SpaceNode<float_t> gps_t;

BOOST_AUTO_TEST_SUITE(RINEX)

template <class T1, class T2>
void compare_lines(const T1 &a, const T2 &b){
  std::stringstream ss_a(a), ss_b(b);
  char buf_a[0x100], buf_b[0x100];
  while(true){
    if(ss_a.eof() && ss_b.eof()){break;}
    if(ss_a.eof() ^ ss_b.eof()){
      break; // error!
    }
    ss_a.getline(buf_a, sizeof(buf_a));
    ss_b.getline(buf_b, sizeof(buf_b));
    BOOST_REQUIRE_EQUAL(std::string(buf_a), std::string(buf_b));
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

  typedef RINEX_NAV_Reader<float_t> reader_t;
  typedef RINEX_NAV_Writer<float_t> writer_t;

  gps_t gps;
  {
    std::stringbuf sbuf(src);
    std::istream in(&sbuf);
    reader_t::read_all(in, gps);
  }

  {
    BOOST_REQUIRE(gps.is_valid_iono_utc());
    /*typename*/ gps_t::Ionospheric_UTC_Parameters iono_utc(gps.iono_utc());
    static const float_t
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

  typedef RINEX_NAV_Reader<float_t> reader_t;
  typedef RINEX_NAV_Writer<float_t> writer_t;

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

    BOOST_CHECK_EQUAL(0, eph.URA); // .200000000000E+01
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

  typedef RINEX_OBS_Reader<float_t> reader_t;
  typedef RINEX_OBS_Writer<float_t> writer_t;

  {
    std::stringbuf sbuf(src);
    std::istream in(&sbuf);
    reader_t reader(in);

    {
      static const char *types[] = {"P1", "L1", "L2", "P2", "L5"};
      for(int i(0); i < sizeof(types)/ sizeof(types[0]); ++i){
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

  typedef RINEX_OBS_Reader<float_t> reader_t;
  typedef RINEX_OBS_Writer<float_t> writer_t;

  {
    std::stringbuf sbuf(src);
    std::istream in(&sbuf);
    reader_t reader(in);

    {
      static const char *types[] = {"C1C", "L1W", "L2W", "C1W", "S2W"};
      for(int i(0); i < sizeof(types)/ sizeof(types[0]); ++i){
        BOOST_CHECK_EQUAL(reader.observed_index(types[i], 'G'), i);
      }
    }
    {
      static const char *types[] = {"C1C", "L1C"};
      for(int i(0); i < sizeof(types)/ sizeof(types[0]); ++i){
        BOOST_CHECK_EQUAL(reader.observed_index(types[i], 'R'), i);
      }
    }
    {
      static const char *types[] = {"L1B", "L5I"};
      for(int i(0); i < sizeof(types)/ sizeof(types[0]); ++i){
        BOOST_CHECK_EQUAL(reader.observed_index(types[i], 'E'), i);
      }
    }
    {
      static const char *types[] = {"C1C", "L1C"};
      for(int i(0); i < sizeof(types)/ sizeof(types[0]); ++i){
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

BOOST_AUTO_TEST_SUITE_END()
