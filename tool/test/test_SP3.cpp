#include <iostream>
#include <string>
#include <sstream>

#include "navigation/SP3.h"

#include <boost/random.hpp>
#include <boost/random/random_device.hpp>

#define BOOST_TEST_MAIN
#include <boost/test/included/unit_test.hpp>

using namespace std;

typedef double fnum_t;
typedef SP3_Reader<fnum_t> reader_t;
typedef SP3_Writer<fnum_t> writer_t;

BOOST_AUTO_TEST_SUITE(SP3)

BOOST_AUTO_TEST_CASE(SP3_d){
  const char *src = \
      "#dP2013  4  3  0  0  0.00000000      96 ORBIT WGS84 BCT MGEX\n" // 1
      "## 1734 259200.00000000   900.00000000 56385 0.0000000000000\n" // 2
      "+  140   G01G02G03G04G05G06G07G08G09G10G11G12G13G14G15G16G17\n" // 3
      "+        G18G19G20G21G22G23G24G25G26G27G28G29G30G31G32R01R02\n" // 4
      "+        R03R04R05R06R07R08R09R10R11R12R13R14R15R16R17R18R19\n" // 5
      "+        R20R21R22R23R24E01E02E03E04E05E06E07E08E09E10E11E12\n" // 6
      "+        E13E14E15E16E17E18E19E20E21E22E23E24E25E26E27E28E29\n" // 7
      "+        E30C01C02C03C04C05C06C07C08C09C10C11C12C13C14C15C16\n" // 8
      "+        C17C18C19C20C21C22C23C24C25C26C27C28C29C30C31C32C33\n" // 9
      "+        C34C35J01J02J03I01I02I03I04I05I06I07S20S24S27S28S29\n" // 10
      "+        S33S35S37S38  0  0  0  0  0  0  0  0  0  0  0  0  0\n" // 11
      "++         0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0\n" // 12
      "++         0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0\n" // 13
      "++         0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0\n" // 14
      "++         0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0\n" // 15
      "++         0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0\n" // 16
      "++         0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0\n" // 17
      "++         0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0\n" // 18
      "++         0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0\n" // 19
      "++         0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0\n" // 20
      "%c M  cc GPS ccc cccc cccc cccc cccc ccccc ccccc ccccc ccccc\n" // 21
      "%c cc cc ccc ccc cccc cccc cccc cccc ccccc ccccc ccccc ccccc\n" // 22
      "%f  1.2500000  1.025000000  0.00000000000  0.000000000000000\n" // 23
      "%f  0.0000000  0.000000000  0.00000000000  0.000000000000000\n" // 24
      "%i    0    0    0    0      0      0      0      0         0\n" // 25
      "%i    0    0    0    0      0      0      0      0         0\n" // 26
      "/* Note: This is a simulated file, meant to illustrate what an SP3-d header\n" // 27
      "/* might look like with more than 85 satellites. Source for GPS and SBAS satel-\n" // 28
      "/* lite positions: BRDM0930.13N. G=GPS,R=GLONASS,E=Galileo,C=BeiDou,J=QZSS,\n" // 29
      "/* I=IRNSS,S=SBAS. For definitions of SBAS satellites, refer to the website:\n" // 30
      "/* http://igs.org/mgex/status-SBAS\n" // 31
      "*  2013  4  3  0  0  0.00000000                             \n" // 32
      "PG01   5783.206741 -18133.044484 -18510.756016     12.734450\n" // 33
      "PG02 -22412.401440  13712.162332    528.367722    425.364822\n"
      "PG03  10114.112309 -17446.189044  16665.051308    189.049475\n"
      "PG04 -24002.325710   4250.313148 -11163.577756    179.333612\n"
      "PG05 -15087.153141   8034.886396  20331.626539   -390.251167\n"
      "PG06  13855.140409 -11053.269706  19768.346019    289.556712\n";
     //----|---1|0---|---2|0---|---3|0---|---4|0---|---5|0---|---6|0---|---7|0---|---8|

  {
    std::stringbuf sbuf(src), sbuf2(src);
    std::istream in(&sbuf), in2(&sbuf2);
    reader_t reader(in);
    while(reader.has_next()){
      reader_t::parsed_t parsed(reader.parse_line());
      switch(parsed.type){
        case reader_t::parsed_t::EPOCH:
          BOOST_CHECK_EQUAL(parsed.item.epoch.year_start, 2013);
          BOOST_CHECK_EQUAL(parsed.item.epoch.month_start, 4);
          BOOST_CHECK_EQUAL(parsed.item.epoch.day_of_month_st, 3);
          BOOST_CHECK_EQUAL(parsed.item.epoch.hour_start, 0);
          BOOST_CHECK_EQUAL(parsed.item.epoch.minute_start, 0);
          BOOST_CHECK_EQUAL(parsed.item.epoch.second_start, 0);
          break;
        case reader_t::parsed_t::POSITION_CLOCK: {
          static const fnum_t cmp[6][4] = {
            {  5783.206741, -18133.044484, -18510.756016,     12.734450},
            {-22412.401440,  13712.162332,    528.367722,    425.364822},
            { 10114.112309, -17446.189044,  16665.051308,    189.049475},
            {-24002.325710,   4250.313148, -11163.577756,    179.333612},
            {-15087.153141,   8034.886396,  20331.626539,   -390.251167},
            { 13855.140409, -11053.269706,  19768.346019,    289.556712},
          };
          for(int i(0); i < 3; ++i){
            BOOST_CHECK_SMALL(
                parsed.item.position_clock.coordinate_km[i]
                  - cmp[parsed.item.position_clock.vehicle_id - 1][i],
                1E-6);
          }
          BOOST_CHECK_SMALL(
              parsed.item.position_clock.clock_us
                - cmp[parsed.item.position_clock.vehicle_id - 1][3],
              1E-6);
          break;
        }
        default: break;
      }
      {
        char buf[0x100] = {0};
        in2.getline(buf, sizeof(buf));
        switch(parsed.type){
          case reader_t::parsed_t::L3_11:
          case reader_t::parsed_t::COMMENT:
            break; // TODO
          default:
            BOOST_TEST(buf == writer_t::print_line(parsed));
            break;
        }
      }
    }
  }
}

BOOST_AUTO_TEST_CASE(SP3_d_GPS_only){
  const char *src = \
      "#dV2001  8  8  0  0  0.00000000     192 ORBIT IGS97 HLM MGEX\n" // 1
      "## 1126 259200.00000000   900.00000000 52129 0.0000000000000\n" // 2
      "+   26   G01G02G03G04G05G06G07G08G09G10G11G13G14G17G18G20G21\n" // 3
      "+        G23G24G25G26G27G28G29G30G31  0  0  0  0  0  0  0  0\n" // 4
      "+          0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0\n" // 5
      "+          0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0\n" // 6
      "+          0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0\n" // 7
      "++         7  8  7  8  6  7  7  7  7  7  7  7  7  8  8  7  9\n" // 8
      "++         9  8  6  8  7  7  6  7  7  0  0  0  0  0  0  0  0\n" // 9
      "++         0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0\n" // 10
      "++         0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0\n" // 11
      "++         0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0\n" // 12
      "%c G  cc GPS ccc cccc cccc cccc cccc ccccc ccccc ccccc ccccc\n" // 13
      "%c cc cc ccc ccc cccc cccc cccc cccc ccccc ccccc ccccc ccccc\n" // 14
      "%f  1.2500000  1.025000000  0.00000000000  0.000000000000000\n" // 15
      "%f  0.0000000  0.000000000  0.00000000000  0.000000000000000\n" // 16
      "%i    0    0    0    0      0      0      0      0         0\n" // 17
      "%i    0    0    0    0      0      0      0      0         0\n" // 18
      "/* AN EXAMPLE ULTRA RAPID ORBIT, GPS ONLY.\n" // 19
      "/* NOTE THE 'PREDICTED DATA' FLAGS FOR THE LAST EPOCH (IN COLUMNS 76 and 80).\n" // 20
      "/* CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC\n" // 21
      "/* CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC\n" // 22
      "*  2001  8  8  0  0  0.00000000                             \n" // 23
      "PG01 -11044.805800 -10475.672350  21929.418200    189.163300 18 18 18 219       \n" // 24
      "EP    55   55   55     222  1234567 -1234567  5999999      -30       21 -1230000\n" // 25
      "VG01  20298.880364 -18462.044804   1381.387685     -4.534317 14 14 14 191       \n" // 26
      "EV    22   22   22     111  1234567  1234567  1234567  1234567  1234567  1234567\n" // 27
      "PG02 -12593.593500  10170.327650 -20354.534400    -55.976000 18 18 18 219     M \n"
      "EP    55   55   55     222  1234567 -1234567  5999999      -30       21 -1230000\n"
      "VG02  -9481.923808 -25832.652567  -7277.160056      8.801258 14 14 14 191       \n"
      "EV    22   22   22     111  1234567  1234567  1234567  1234567  1234567  1234567\n";
     //----|---1|0---|---2|0---|---3|0---|---4|0---|---5|0---|---6|0---|---7|0---|---8|
  {
    std::stringbuf sbuf(src), sbuf2(src);
    std::istream in(&sbuf), in2(&sbuf2);
    reader_t reader(in);
    while(reader.has_next()){
      reader_t::parsed_t parsed(reader.parse_line());
      switch(parsed.type){
        case reader_t::parsed_t::EPOCH:
          BOOST_CHECK_EQUAL(parsed.item.epoch.year_start, 2001);
          BOOST_CHECK_EQUAL(parsed.item.epoch.month_start, 8);
          BOOST_CHECK_EQUAL(parsed.item.epoch.day_of_month_st, 8);
          BOOST_CHECK_EQUAL(parsed.item.epoch.hour_start, 0);
          BOOST_CHECK_EQUAL(parsed.item.epoch.minute_start, 0);
          BOOST_CHECK_EQUAL(parsed.item.epoch.second_start, 0);
          break;
        case reader_t::parsed_t::POSITION_CLOCK: {
          static const fnum_t cmp[2][4] = {
            {-11044.805800, -10475.672350,  21929.418200,    189.163300},
            {-12593.593500,  10170.327650, -20354.534400,    -55.976000},
          };
          for(int i(0); i < 3; ++i){
            BOOST_CHECK_SMALL(
                parsed.item.position_clock.coordinate_km[i]
                  - cmp[parsed.item.position_clock.vehicle_id - 1][i],
                1E-6);
          }
          BOOST_CHECK_SMALL(
              parsed.item.position_clock.clock_us
                - cmp[parsed.item.position_clock.vehicle_id - 1][3],
              1E-6);
          break;
        }
        case reader_t::parsed_t::VELOCITY_RATE: {
          static const fnum_t cmp[2][4] = {
            {20298.880364, -18462.044804,   1381.387685,     -4.534317},
            {-9481.923808, -25832.652567,  -7277.160056,      8.801258},
          };
          for(int i(0); i < 3; ++i){
            BOOST_CHECK_SMALL(
                parsed.item.velocity_rate.velocity_dm_s[i]
                  - cmp[parsed.item.velocity_rate.vehicle_id - 1][i],
                1E-6);
          }
          BOOST_CHECK_SMALL(
              parsed.item.velocity_rate.clock_rate_chg_100ps_s
                - cmp[parsed.item.velocity_rate.vehicle_id - 1][3],
              1E-6);
          break;
        }
        default: break;
      }
      {
        char buf[0x100] = {0};
        in2.getline(buf, sizeof(buf));
        switch(parsed.type){
          case reader_t::parsed_t::COMMENT:
            break; // TODO
          default:
            BOOST_TEST(buf == writer_t::print_line(parsed));
            break;
        }
      }
    }
  }
}

BOOST_AUTO_TEST_SUITE_END()
