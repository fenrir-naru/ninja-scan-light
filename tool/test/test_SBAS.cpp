#include <iostream>
#include <cmath>
#include <vector>
#include <bitset>

#include "navigation/SBAS.h"

#include <boost/cstdint.hpp>
#include <boost/format.hpp>
#define BOOST_TEST_MAIN
#include <boost/test/included/unit_test.hpp>

#if !defined(BOOST_VERSION)
#define BOOST_FIXTURE_TEST_SUITE(name, fixture)
#define BOOST_AUTO_TEST_SUITE_END()
#define BOOST_AUTO_TEST_CASE(name) void name()
#endif

using namespace std;

typedef SBAS_SpaceNode<double> space_node_t;
typedef space_node_t::IonosphericGridPoints igp_t;

typedef vector<igp_t::trapezoid_t> trapezoid_list_t;

trapezoid_list_t trapezoid_list;
ostream &operator<<(ostream &out, const igp_t::trapezoid_t &target){
  out << trapezoid_list.size() << ": ";
  for(int i(0); i < 4; i++){
    if(i != 0){out << ", ";}
    out << boost::format("(% 3d, % 4d)") % target.igp[i].latitude_deg % target.igp[i].longitude_deg;
  }
  return out;
}

template <>
template <>
int SBAS_SpaceNode<double>::IonosphericGridPoints::check_availability_hook<int>(
    SBAS_SpaceNode<double>::IonosphericGridPoints::trapezoid_t &in, const int &out) const {
  BOOST_TEST_MESSAGE(in);
  trapezoid_list.push_back(in);
  in.checked[2] = in.checked[3] = true;
  return 0; // for test
}

BOOST_AUTO_TEST_SUITE(SBAS)

#define LAT_75S_85N(x) \
    {-75, x}, {-65, x}, {-55, x}, {-50, x}, {-45, x}, {-40, x}, {-35, x}, \
    {-30, x}, {-25, x}, {-20, x}, {-15, x}, {-10, x}, { -5, x}, {  0, x}, \
    {  5, x}, { 10, x}, { 15, x}, { 20, x}, { 25, x}, { 30, x}, { 35, x}, \
    { 40, x}, { 45, x}, { 50, x}, { 55, x}, { 65, x}, { 75, x}, { 85, x}
#define LAT_55S_55N(x) \
    {-55, x}, {-50, x}, {-45, x}, {-40, x}, {-35, x}, {-30, x}, {-25, x}, \
    {-20, x}, {-15, x}, {-10, x}, { -5, x}, {  0, x}, \
    {  5, x}, { 10, x}, { 15, x}, { 20, x}, \
    { 25, x}, { 30, x}, { 35, x}, { 40, x}, { 45, x}, { 50, x}, { 55, x}
#define LAT_75S_75N(x) \
    {-75, x}, {-65, x}, {-55, x}, {-50, x}, {-45, x}, {-40, x}, {-35, x}, \
    {-30, x}, {-25, x}, {-20, x}, {-15, x}, {-10, x}, { -5, x}, {  0, x}, \
    {  5, x}, { 10, x}, { 15, x}, { 20, x}, { 25, x}, { 30, x}, { 35, x}, \
    { 40, x}, { 45, x}, { 50, x}, { 55, x}, { 65, x}, { 75, x}
#define LAT_85S_75N(x) \
    {-85, x}, {-75, x}, {-65, x}, {-55, x}, {-50, x}, {-45, x}, {-40, x}, \
    {-35, x}, {-30, x}, {-25, x}, {-20, x}, {-15, x}, {-10, x}, { -5, x}, \
    {  0, x}, {  5, x}, { 10, x}, { 15, x}, { 20, x}, { 25, x}, { 30, x}, \
    { 35, x}, { 40, x}, { 45, x}, { 50, x}, { 55, x}, { 65, x}, { 75, x}

#define LNG_180W_175E(x) \
    {x, -180}, {x, -175}, {x, -170}, {x, -165}, {x, -160}, {x, -155}, {x, -150}, {x, -145}, \
    {x, -140}, {x, -135}, {x, -130}, {x, -125}, {x, -120}, {x, -115}, {x, -110}, {x, -105}, \
    {x, -100}, {x,  -95}, {x,  -90}, {x,  -85}, {x,  -80}, {x,  -75}, {x,  -70}, {x,  -65}, \
    {x,  -60}, {x,  -55}, {x,  -50}, {x,  -45}, {x,  -40}, {x,  -35}, {x,  -30}, {x,  -25}, \
    {x,  -20}, {x,  -15}, {x,  -10}, {x,   -5}, {x,    0}, {x,    5}, {x,   10}, {x,   15}, \
    {x,   20}, {x,   25}, {x,   30}, {x,   35}, {x,   40}, {x,   45}, {x,   50}, {x,   55}, \
    {x,   60}, {x,   65}, {x,   70}, {x,   75}, {x,   80}, {x,   85}, {x,   90}, {x,   95}, \
    {x,  100}, {x,  105}, {x,  110}, {x,  115}, {x,  120}, {x,  125}, {x,  130}, {x,  135}, \
    {x,  140}, {x,  145}, {x,  150}, {x,  155}, {x,  160}, {x,  165}, {x,  170}, {x,  175}
#define LNG_180W_170E(x) \
    {x, -180}, {x, -170}, {x, -160}, {x, -150}, {x, -140}, {x, -130}, {x, -120}, {x, -110}, \
    {x, -100}, {x,  -90}, {x,  -80}, {x,  -70}, {x,  -60}, {x,  -50}, {x,  -40}, {x,  -30}, \
    {x,  -20}, {x,  -10}, {x,    0}, {x,   10}, {x,   20}, {x,   30}, {x,   40}, {x,   50}, \
    {x,   60}, {x,   70}, {x,   80}, {x,   90}, {x,  100}, {x,  110}, {x,  120}, {x,  130}, \
    {x,  140}, {x,  150}, {x,  160}, {x,  170}
#define LNG_180W_150E(x) \
    {x, -180}, {x, -150}, {x, -120}, {x,  -90}, {x,  -60}, {x,  -30}, {x,    0}, {x,   30}, \
    {x,   60}, {x,   90}, {x,  120}, {x,  150}
#define LNG_170W_160E(x) \
    {x, -170}, {x, -140}, {x, -110}, {x,  -80}, {x,  -50}, {x,  -20}, {x,   10}, {x,   40}, \
    {x,   70}, {x,  100}, {x,  130}, {x,  160}

static const int igp_lat_lng[][201][2] = {
  { // 0
    LAT_75S_85N(-180),  // !
    LAT_55S_55N(-175),
    LAT_75S_75N(-170),
    LAT_55S_55N(-165),
    LAT_75S_75N(-160),
    LAT_55S_55N(-155),
    LAT_75S_75N(-150),
    LAT_55S_55N(-145),
  }, { // 1
    LAT_85S_75N(-140),  // !
    LAT_55S_55N(-135),
    LAT_75S_75N(-130),
    LAT_55S_55N(-125),
    LAT_75S_75N(-120),
    LAT_55S_55N(-115),
    LAT_75S_75N(-110),
    LAT_55S_55N(-105),
  }, { // 2
    LAT_75S_75N(-100),
    LAT_55S_55N(-95),
    LAT_75S_85N(-90),   // !
    LAT_55S_55N(-85),
    LAT_75S_75N(-80),
    LAT_55S_55N(-75),
    LAT_75S_75N(-70),
    LAT_55S_55N(-65),
  }, { // 3
    LAT_75S_75N(-60),
    LAT_55S_55N(-55),
    LAT_85S_75N(-50),   // !
    LAT_55S_55N(-45),
    LAT_75S_75N(-40),
    LAT_55S_55N(-35),
    LAT_75S_75N(-30),
    LAT_55S_55N(-25),
  }, { // 4
    LAT_75S_75N(-20),
    LAT_55S_55N(-15),
    LAT_75S_75N(-10),
    LAT_55S_55N(-5),
    LAT_75S_85N(0),     // !
    LAT_55S_55N(5),
    LAT_75S_75N(10),
    LAT_55S_55N(15),
  }, { // 5
    LAT_75S_75N(20),
    LAT_55S_55N(25),
    LAT_75S_75N(30),
    LAT_55S_55N(35),
    LAT_85S_75N(40),    // !
    LAT_55S_55N(45),
    LAT_75S_75N(50),
    LAT_55S_55N(55),
  }, { // 6
    LAT_75S_75N(60),
    LAT_55S_55N(65),
    LAT_75S_75N(70),
    LAT_55S_55N(75),
    LAT_75S_75N(80),
    LAT_55S_55N(85),
    LAT_75S_85N(90),    // !
    LAT_55S_55N(95),
  }, { // 7
    LAT_75S_75N(100),
    LAT_55S_55N(105),
    LAT_75S_75N(110),
    LAT_55S_55N(115),
    LAT_75S_75N(120),
    LAT_55S_55N(125),
    LAT_85S_75N(130),   // !
    LAT_55S_55N(135),
  }, { // 8
    LAT_75S_75N(140),
    LAT_55S_55N(145),
    LAT_75S_75N(150),
    LAT_55S_55N(155),
    LAT_75S_75N(160),
    LAT_55S_55N(165),
    LAT_75S_75N(170),
    LAT_55S_55N(175),
  }, { // 9
    LNG_180W_175E(60),
    LNG_180W_170E(65),
    LNG_180W_170E(70),
    LNG_180W_170E(75),
    LNG_180W_150E(85),
  }, { // 10
    LNG_180W_175E(-60),
    LNG_180W_170E(-65),
    LNG_180W_170E(-70),
    LNG_180W_170E(-75),
    LNG_170W_160E(-85),
  },
};

#undef LAT_75S_85N
#undef LAT_55S_55N
#undef LAT_75S_75N
#undef LAT_85S_75N

#undef LNG_180W_175E
#undef LNG_180W_170E
#undef LNG_180W_150E
#undef LNG_170W_160E

BOOST_AUTO_TEST_CASE(igp_band_mask){
  for(unsigned band(0); band < 11; ++band){
    for(unsigned mask(space_node_t::DataBlock::Type18::mask_bits(band)); mask < 201; ++mask){
      BOOST_REQUIRE_EQUAL(igp_lat_lng[band][mask][0], 0); // out of bands => check default value(0)
      BOOST_REQUIRE_EQUAL(igp_lat_lng[band][mask][1], 0);
    }
  }
}

BOOST_AUTO_TEST_CASE(igp_pos_cast){
  for(unsigned band(0); band < 11; ++band){
    for(unsigned mask(0); mask < space_node_t::DataBlock::Type18::mask_bits(band); ++mask){
      igp_t::position_t pos = {igp_lat_lng[band][mask][0], igp_lat_lng[band][mask][1]};
      BOOST_TEST_MESSAGE("(band, mask) = (" << band << ", " << mask << ") => " << pos.latitude_deg << ", " << pos.longitude_deg);
      igp_t::position_t pos2(((igp_t::position_index_t)pos));
      BOOST_REQUIRE_EQUAL(pos2.latitude_deg, pos.latitude_deg);
      BOOST_REQUIRE_EQUAL(pos2.longitude_deg, pos.longitude_deg);
    }
  }
}

BOOST_AUTO_TEST_CASE(igp_pos){
  for(unsigned band(0); band < 11; ++band){
    for(unsigned mask(0); mask < space_node_t::DataBlock::Type18::mask_bits(band); ++mask){
      igp_t::position_t pos(igp_t::position(band, mask));
      BOOST_TEST_MESSAGE("(band, mask) = (" << band << ", " << mask << ") => " << pos.latitude_deg << ", " << pos.longitude_deg);
      BOOST_REQUIRE(pos.is_predefined());
      BOOST_REQUIRE_EQUAL(pos.latitude_deg, igp_lat_lng[band][mask][0]);
      BOOST_REQUIRE_EQUAL(pos.longitude_deg, igp_lat_lng[band][mask][1]);
    }
  }
}

void check_pivot_of_grid_point(const igp_t::pivot_t &pivot, const unsigned &band, const unsigned &mask){

  BOOST_TEST_MESSAGE("(band, mask) = (" << band << ", " << mask << ") => "
      << pivot.igp.latitude_deg << ", " << pivot.igp.longitude_deg);

  int delta_lat(0);
  if(igp_lat_lng[band][mask][0] == 85){
    delta_lat = -10;
  }else if(igp_lat_lng[band][mask][0] == -85){
    delta_lat = 10;
  }else if(igp_lat_lng[band][mask][0] > 0){
    delta_lat = -5;
  }else if(igp_lat_lng[band][mask][0] < 0){
    delta_lat = 5;
  }

  BOOST_REQUIRE_EQUAL(pivot.igp.latitude_deg, igp_lat_lng[band][mask][0] + delta_lat);
  BOOST_REQUIRE_EQUAL(pivot.igp.longitude_deg, igp_lat_lng[band][mask][1]);
  BOOST_REQUIRE_EQUAL(pivot.delta.latitude_deg, -delta_lat);
  BOOST_REQUIRE_EQUAL(pivot.delta.longitude_deg, 0);
}

BOOST_AUTO_TEST_CASE(igp_pivot){
  typedef igp_t::pivot_t res_t;

  for(unsigned band(0); band < 11; ++band){
    for(unsigned mask(0); mask < space_node_t::DataBlock::Type18::mask_bits(band); ++mask){

      { // on grid
        check_pivot_of_grid_point(
            igp_t::get_pivot(igp_lat_lng[band][mask][0], igp_lat_lng[band][mask][1]),
            band, mask);
      }
      { // near grid
        double delta_lat(igp_lat_lng[band][mask][0] >= 0 ? 0.5 : -0.5), delta_lng(0.5);
        double
            lat(delta_lat + igp_lat_lng[band][mask][0]),
            lng(delta_lng + igp_lat_lng[band][mask][1]);
        res_t res(igp_t::get_pivot(lat, lng));
        BOOST_TEST_MESSAGE("(band, mask) = (" << band << ", " << mask << ") => "
            << res.igp.latitude_deg << "(" << lat << "), "
            << res.igp.longitude_deg << "(" << lng << ")");

        BOOST_REQUIRE_EQUAL(res.igp.latitude_deg, igp_lat_lng[band][mask][0]);
        BOOST_REQUIRE_EQUAL(res.delta.latitude_deg, delta_lat);

        if(lat > 85){
          int lng_grid(90 * floor(lng / 90));
          if(lng_grid == 180){lng_grid = -180;}
          double delta_lng2(delta_lng + igp_lat_lng[band][mask][1] - lng_grid);
          BOOST_REQUIRE_EQUAL(res.igp.longitude_deg, lng_grid);
          BOOST_REQUIRE_EQUAL(res.delta.longitude_deg, delta_lng2);
        }else if(lat < -85){
          int lng_grid(40 + 90 * floor((lng - 40) / 90));
          if(lng_grid == -230){lng_grid = 130;}
          double delta_lng2(delta_lng + igp_lat_lng[band][mask][1] - lng_grid);
          if(delta_lng2 < -180){delta_lng2 += 360;}
          BOOST_REQUIRE_EQUAL(res.igp.longitude_deg, lng_grid);
          BOOST_REQUIRE_EQUAL(res.delta.longitude_deg, delta_lng2);
        }else if((lat > 60) || (lat < -60)){
          int lng_grid(10 * floor(lng / 10));
          double delta_lng2(delta_lng + igp_lat_lng[band][mask][1] - lng_grid);
          if(delta_lng2 < -180){delta_lng2 += 360;}
          BOOST_REQUIRE_EQUAL(res.igp.longitude_deg, lng_grid);
          BOOST_REQUIRE_EQUAL(res.delta.longitude_deg, delta_lng2);
        }else{
          BOOST_REQUIRE_EQUAL(res.igp.longitude_deg, igp_lat_lng[band][mask][1]);
          BOOST_REQUIRE_EQUAL(res.delta.longitude_deg, delta_lng);
        }
      }

      // on grid, roll over (int)
      static const int roll_over_i[] = {-360, 360};
      for(unsigned int i(0); i < sizeof(roll_over_i) / sizeof(roll_over_i[0]); ++i){ // +/- roll over
        check_pivot_of_grid_point(
            igp_t::get_pivot(
              igp_lat_lng[band][mask][0],
              roll_over_i[i] + igp_lat_lng[band][mask][1]),
            band, mask);
      }

      // on grid, roll over (double)
      static const double roll_over_f[] = {-360.0, 360.0};
      for(unsigned int i(0); i < sizeof(roll_over_f) / sizeof(roll_over_f[0]); ++i){ // +/- roll over
        check_pivot_of_grid_point(
            igp_t::get_pivot(
              (double)igp_lat_lng[band][mask][0],
              roll_over_f[i] + igp_lat_lng[band][mask][1]),
            band, mask);
      }
    }
  }
}

/**
 * @param in_lat [-90, 90]
 * @param in_lng [-180, 180)
 */
void igp_interpolate_check(const double &in_lat, const double &in_lng){
  bool north_hemisphere(in_lat >= 0);

  static igp_t igp;
  trapezoid_list.clear();
  igp.interpolate(in_lat, in_lng);

  // simple check
  for(trapezoid_list_t::const_iterator it(trapezoid_list.begin());
      it != trapezoid_list.end(); ++it){
    for(int i(0); i < 4; ++i){
      BOOST_REQUIRE(it->igp[i].is_predefined()); // grids are predefined?
    }
    { // trapezoid?
      BOOST_REQUIRE_EQUAL(it->igp[0].latitude_deg, it->igp[1].latitude_deg);
      BOOST_REQUIRE_EQUAL(it->igp[2].latitude_deg, it->igp[3].latitude_deg);
    }

    { // check igp[2]
      if(in_lat == 0){
        BOOST_REQUIRE_EQUAL(it->igp[2].latitude_deg, 0);
      }else if(north_hemisphere){
        BOOST_REQUIRE_LT(it->igp[2].latitude_deg, in_lat);
      }else{
        BOOST_REQUIRE_GT(it->igp[2].latitude_deg, in_lat);
      }

      if((trapezoid_list.front().igp[2].latitude_deg == -85)
          && (in_lng < -140)){
        BOOST_REQUIRE_EQUAL(it->igp[2].longitude_deg, 130);
      }else if((trapezoid_list.front().igp[2].longitude_deg == -180)
          && (it->igp[2].longitude_deg > 0)){
        if(trapezoid_list.front().igp[2].latitude_deg * (north_hemisphere ? 1 : -1) <= 55){
          BOOST_REQUIRE_EQUAL(it->igp[2].longitude_deg, 175);
        }else{
          BOOST_REQUIRE(
              (trapezoid_list.front().igp[2].latitude_deg == -75)
              && ((it->igp[2].longitude_deg == 160)
                  || (it->igp[2].longitude_deg == 130)));
        }
      }else if((trapezoid_list.front().igp[2].longitude_deg == -170)
          && (it->igp[2].longitude_deg > 0)){
        BOOST_REQUIRE(
            (trapezoid_list.front().igp[2].latitude_deg == -85)
            && (it->igp[2].longitude_deg == 130));
      }else{
        BOOST_REQUIRE_LE(it->igp[2].longitude_deg, in_lng);
      }
    }
  }

  // precise check
  static const struct {
    int lat_max;
    int num_min, num_max;
    struct {
      int lat, lng;
    } spacing[5];
  } props[] = {
    55, 5, 5, {{ 5,  5}, {10, 10}, {10, 10}, {10, 10}, {10, 10}},
    60, 4, 4, {{ 5,  5}, {10, 10}, {10, 10}, {10, 10}},
    70, 3, 3, {{ 5, 10}, {10, 10}, {10, 10}},
    75, 2, 2, {{ 5, 10}, {10, 10}},
    85, 1, 2, {{10, 10}, {10, 10}},
  };

  for(unsigned int i(0); i < sizeof(props) / sizeof(props[0]); i++){ // lat = [-75, 75]
    if(abs(in_lat) > props[i].lat_max){continue;}

    BOOST_REQUIRE_GE(trapezoid_list.size(), props[i].num_min); // number of candidates
    BOOST_REQUIRE_LE(trapezoid_list.size(), props[i].num_max); // number of candidates
    for(unsigned int i2(0); i2 < trapezoid_list.size(); ++i2){
      { // check latitude
        { // spacing
          BOOST_REQUIRE_EQUAL(
              trapezoid_list[i2].igp[1].latitude_deg - trapezoid_list[i2].igp[2].latitude_deg,
              props[i].spacing[i2].lat * (north_hemisphere ? 1 : -1));
          BOOST_REQUIRE_EQUAL(
              trapezoid_list[i2].igp[0].latitude_deg - trapezoid_list[i2].igp[3].latitude_deg,
              props[i].spacing[i2].lat * (north_hemisphere ? 1 : -1));
        }

        // opposite of igp[2] equals igp[0]
        if(north_hemisphere){
          BOOST_REQUIRE_GE(trapezoid_list[i2].igp[0].latitude_deg, in_lat);
        }else{
          BOOST_REQUIRE_LE(trapezoid_list[i2].igp[0].latitude_deg, in_lat);
        }
      }
      { // check longitude; rectangle and spacing
        if(props[i].lat_max < 85){
          BOOST_REQUIRE_EQUAL(
              trapezoid_list[i2].igp[0].longitude_deg,
              trapezoid_list[i2].igp[3].longitude_deg);
          BOOST_REQUIRE_EQUAL(
              trapezoid_list[i2].igp[1].longitude_deg,
              trapezoid_list[i2].igp[2].longitude_deg);
          BOOST_REQUIRE_EQUAL(
              trapezoid_list[i2].igp[0].delta_lng(trapezoid_list[i2].igp[1]),
              props[i].spacing[i2].lng);
        }else{
          BOOST_REQUIRE_LE(
              trapezoid_list[i2].igp[0].delta_lng(trapezoid_list[i2].igp[1]),
              90);
        }
        BOOST_REQUIRE_EQUAL(
            trapezoid_list[i2].igp[3].delta_lng(trapezoid_list[i2].igp[2]),
            props[i].spacing[i2].lng);

        if(trapezoid_list[i2].igp[3].longitude_deg <= -175){
          BOOST_REQUIRE_GT(trapezoid_list[i2].igp[3].longitude_deg + 360, in_lng);
        }else{
          BOOST_REQUIRE_GT(trapezoid_list[i2].igp[3].longitude_deg, in_lng);
        }
      }
    }
    return;
  }

  { // pole
    BOOST_REQUIRE_EQUAL(trapezoid_list.size(), 1); // number of candidates
    BOOST_REQUIRE_EQUAL(
        trapezoid_list.front().igp[1].latitude_deg,
        trapezoid_list.front().igp[2].latitude_deg);
  }
}

BOOST_AUTO_TEST_CASE(igp_interpolate_near_grid){
  for(unsigned band(0); band < 11; ++band){
    for(unsigned mask(0); mask < space_node_t::DataBlock::Type18::mask_bits(band); ++mask){

      { // near grid
        double delta_lat(igp_lat_lng[band][mask][0] >= 0 ? 0.5 : -0.5), delta_lng(0.5);
        double
            lat(delta_lat + igp_lat_lng[band][mask][0]),
            lng(delta_lng + igp_lat_lng[band][mask][1]);
        BOOST_TEST_MESSAGE("(band, mask) = (" << band << ", " << mask << ") => "
            << "(" << lat << "), " << "(" << lng << ")");
        igp_interpolate_check(lat, lng);
      }
    }
  }
}

BOOST_AUTO_TEST_CASE(data_block_decorder){
  for(unsigned int i(0); i < 0x100; i++){
    for(unsigned int j(0); j < 0x100; j++){
      BOOST_TEST_MESSAGE( "Calling (i,j)=(" << i << "," << j << ")");
      unsigned char buf[] = {(unsigned char)i, (unsigned char)j, (unsigned char)i};
      boost::uint16_t buf2[] = {(boost::uint16_t)((i << 8) + j)};
      unsigned char buf3[] = { // (padding 2, effective 4, padding 2)
          (unsigned char)((i & 0xF0) >> 2), (unsigned char)((i & 0xF) << 2),
          (unsigned char)((j & 0xF0) >> 2), (unsigned char)((j & 0xF) << 2),
          (unsigned char)((i & 0xF0) >> 2), (unsigned char)((i & 0xF) << 2),};
      boost::uint16_t buf4[] = { // (padding 2, effective 12, padding 2)
          (boost::uint16_t)((((i << 8) + j) & 0xFFF0) >> 2),
          (boost::uint16_t)((((i << 8) + j) & 0x000F) << (16 - 2 - 4)),};
      bitset<24> b((unsigned long long)((i << 16) + (j << 8) + i));
      for(unsigned int offset(0); offset <= 8; offset++){
        unsigned char res(space_node_t::DataBlock::bits2num<unsigned char>(buf, offset)); // in == out
        unsigned char res2(space_node_t::DataBlock::bits2num<unsigned char>(buf2, offset)); // in > out
        boost::uint16_t res3(space_node_t::DataBlock::bits2num<boost::uint16_t>(buf, offset)); // in < out
        unsigned long b2((b << offset).to_ulong());
        BOOST_REQUIRE_EQUAL(((b2 & 0xFF0000) >> 16), res);
        BOOST_REQUIRE_EQUAL(((b2 & 0xFF0000) >> 16), res2);
        BOOST_REQUIRE_EQUAL(((b2 & 0xFFFF00) >> 8), res3);
        unsigned char res4(space_node_t::DataBlock::bits2num<unsigned char, 4, 2>(buf3, offset)); // in < out
        unsigned char res5(space_node_t::DataBlock::bits2num<unsigned char, 12, 2>(buf4, offset)); // in > out
        BOOST_REQUIRE_EQUAL(((b2 & 0xFF0000) >> 16), res4);
        BOOST_REQUIRE_EQUAL(((b2 & 0xFF0000) >> 16), res5);
      }
    }
  }
}

BOOST_AUTO_TEST_CASE(data_block_type18){
  static const int mask_max(16); // check for [0,15] mask
  for(int i(0); i < (1 << mask_max); ++i){
    unsigned char buf[(250 + 7) / 8] = {0};
    buf[3] = (unsigned char)(i & 0xFF);
    buf[4] = (unsigned char)((i >> 8) & 0xFF);

    space_node_t::DataBlock::Type18::mask_t res(space_node_t::DataBlock::Type18::mask(buf));

    BOOST_TEST_MESSAGE(i << ", valid(" << (int)res.valid << ")");
    int res_offset(0);
    for(int j(0); j < mask_max; ++j){
      div_t k(div(j, 8));
      if((res_offset < res.valid) && (res.linear[res_offset] == j)){
        res_offset++;
        BOOST_TEST_MESSAGE(i << ", mask(" << j << ")");
        BOOST_REQUIRE(i & (1 << ((k.quot * 8) + (7 - k.rem))));
      }else{
        BOOST_REQUIRE(!(i & (1 << ((k.quot * 8) + (7 - k.rem)))));
      }
    }
  }
}

BOOST_AUTO_TEST_CASE(known_satellites){
  {
    int prn(0);
    for(space_node_t::KnownSatellites::list_t::const_iterator it(
          space_node_t::KnownSatellites::prn_ordered.begin());
        it != space_node_t::KnownSatellites::prn_ordered.end();
        ++it){
      BOOST_TEST_MESSAGE("prn: " << (*it)->prn);
      BOOST_REQUIRE(prn <= (*it)->prn);
      prn = (*it)->prn;
    }
  }
  {
    double lng_deg(-180);
    for(space_node_t::KnownSatellites::list_t::const_iterator it(
          space_node_t::KnownSatellites::longitude_ordered.begin());
        it != space_node_t::KnownSatellites::longitude_ordered.end();
        ++it){
      BOOST_TEST_MESSAGE("lng: " << (*it)->lng_deg);
      BOOST_REQUIRE(lng_deg <= (*it)->lng_deg);
      lng_deg = (*it)->lng_deg;
    }
  }
  for(double base_lng_deg(-180); base_lng_deg < 180; base_lng_deg += 10){
    double delta_lng_deg(0);
    space_node_t::KnownSatellites::list_t sats(
        space_node_t::KnownSatellites::nearest_ordered(base_lng_deg));
    for(space_node_t::KnownSatellites::list_t::const_iterator it(sats.begin());
        it != sats.end();
        ++it){
      BOOST_TEST_MESSAGE("near(" << base_lng_deg << "): " << (*it)->lng_deg);
      double delta_lng2_deg((*it)->lng_deg - base_lng_deg);
      if(delta_lng2_deg < 0){delta_lng2_deg *= -1;}
      if(delta_lng2_deg > 180){delta_lng2_deg = -delta_lng2_deg + 360;}
      BOOST_REQUIRE(delta_lng_deg <= delta_lng2_deg);
      delta_lng_deg = delta_lng2_deg;
    }
  }
}

namespace rtklib {
unsigned int crc24q(const unsigned char *buff, int len){
  static const unsigned int poly(0x1864CFBu); /* 1100001100100110011111011 */
  unsigned int crc(0);
  for(int i(0); i < len; i++) {
    crc^=(unsigned int)buff[i] << 16;
    for(int j(0); j < 8; j++) if ((crc <<= 1) & 0x1000000) crc ^= poly;
  }
  return crc;
}
}

namespace wikipedia {
// @see https://en.wikipedia.org/wiki/Cyclic_redundancy_check
const unsigned int poly = 0x1864CFBu; /* 1100001100100110011111011 */
unsigned int crc24q(const unsigned char *buf, const int &bits, unsigned int crc = 0){
  if(crc != 0){
    for(int i(0); i < 24; i++){ // invert operation for padding
      if(crc & 0x1){crc ^= poly;}
      crc >>= 1;
    }
  }
  unsigned char mask(0x80);
  for(int i(0), buf_idx(0); i < bits; i++, mask >>= 1){
    if(mask == 0){
      mask = 0x80;
      ++buf_idx;
    }
    crc <<= 1;
    if(buf[buf_idx] & mask){crc |= 1;}
    if(crc & 0x1000000u){crc ^= poly;}
  }
  for(int i(0); i < 24; i++){ // padding
    crc <<= 1;
    if(crc & 0x1000000u){crc ^= poly;}
  }
  return crc;
}

unsigned int crc24q_tbl(const unsigned char *buf, const int &bits, unsigned int crc = 0){
  static const struct tbl_t {
    unsigned int fw[0x100], bk[0x100];
    void gen_forward() {
      for(int i(0); i < 0x100; ++i){
        fw[i] = i << 16;
        for(int j(0); j < 8; ++j){
          fw[i] <<= 1;
          if(fw[i] & 0x1000000u){fw[i] ^= poly;}
        }
      }
    }
    void gen_backward() {
      for(int i(0); i < 0x100; ++i){
        bk[i] = i;
        for(int j(0); j < 8; ++j){
          if(bk[i] & 0x1){bk[i] ^= poly;}
          bk[i] >>= 1;
        }
      }
    }
    tbl_t() {
      gen_forward();
      gen_backward();
    }
  } tbl;

  if(crc != 0){
    for(int i(0); i < 3; i++){ // invert operation for padding
      unsigned int cache(tbl.bk[crc & 0xFF]);
      crc = (cache & 0xFF0000) | ((cache & 0xFFFF) ^ (crc >> 8));
    }
  }
  int bit_idx(0), buf_idx(0);
  for(int bytes(bits >> 3); buf_idx < bytes; ++buf_idx, bit_idx += 8){
    crc = (((crc << 8) & 0xFFFF00) | buf[buf_idx]) ^ tbl.fw[(crc >> 16) & 0xFF];
  }
  for(unsigned char mask(0x80); bit_idx < bits; bit_idx++, mask >>= 1){
    crc <<= 1;
    if(buf[buf_idx] & mask){crc |= 1;}
    if(crc & 0x1000000u){crc ^= poly;}
  }
  for(int i(0); i < 3; i++){ // forward operation for padding
    unsigned int cache(tbl.fw[(crc >> 16) & 0xFF]);
    crc = (((crc << 8) ^ cache) & 0xFFFF00) | (cache & 0xFF);
  }
  return crc;
}
}

namespace gpsd {
// @see https://github.com/ukyg9e5r6k7gubiekd6/gpsd/blob/master/crc24q.c#L80

static const int unsigned crc24q[256] = {
  0x00000000u, 0x01864CFBu, 0x028AD50Du, 0x030C99F6u,
  0x0493E6E1u, 0x0515AA1Au, 0x061933ECu, 0x079F7F17u,
  0x08A18139u, 0x0927CDC2u, 0x0A2B5434u, 0x0BAD18CFu,
  0x0C3267D8u, 0x0DB42B23u, 0x0EB8B2D5u, 0x0F3EFE2Eu,
  0x10C54E89u, 0x11430272u, 0x124F9B84u, 0x13C9D77Fu,
  0x1456A868u, 0x15D0E493u, 0x16DC7D65u, 0x175A319Eu,
  0x1864CFB0u, 0x19E2834Bu, 0x1AEE1ABDu, 0x1B685646u,
  0x1CF72951u, 0x1D7165AAu, 0x1E7DFC5Cu, 0x1FFBB0A7u,
  0x200CD1E9u, 0x218A9D12u, 0x228604E4u, 0x2300481Fu,
  0x249F3708u, 0x25197BF3u, 0x2615E205u, 0x2793AEFEu,
  0x28AD50D0u, 0x292B1C2Bu, 0x2A2785DDu, 0x2BA1C926u,
  0x2C3EB631u, 0x2DB8FACAu, 0x2EB4633Cu, 0x2F322FC7u,
  0x30C99F60u, 0x314FD39Bu, 0x32434A6Du, 0x33C50696u,
  0x345A7981u, 0x35DC357Au, 0x36D0AC8Cu, 0x3756E077u,
  0x38681E59u, 0x39EE52A2u, 0x3AE2CB54u, 0x3B6487AFu,
  0x3CFBF8B8u, 0x3D7DB443u, 0x3E712DB5u, 0x3FF7614Eu,
  0x4019A3D2u, 0x419FEF29u, 0x429376DFu, 0x43153A24u,
  0x448A4533u, 0x450C09C8u, 0x4600903Eu, 0x4786DCC5u,
  0x48B822EBu, 0x493E6E10u, 0x4A32F7E6u, 0x4BB4BB1Du,
  0x4C2BC40Au, 0x4DAD88F1u, 0x4EA11107u, 0x4F275DFCu,
  0x50DCED5Bu, 0x515AA1A0u, 0x52563856u, 0x53D074ADu,
  0x544F0BBAu, 0x55C94741u, 0x56C5DEB7u, 0x5743924Cu,
  0x587D6C62u, 0x59FB2099u, 0x5AF7B96Fu, 0x5B71F594u,
  0x5CEE8A83u, 0x5D68C678u, 0x5E645F8Eu, 0x5FE21375u,
  0x6015723Bu, 0x61933EC0u, 0x629FA736u, 0x6319EBCDu,
  0x648694DAu, 0x6500D821u, 0x660C41D7u, 0x678A0D2Cu,
  0x68B4F302u, 0x6932BFF9u, 0x6A3E260Fu, 0x6BB86AF4u,
  0x6C2715E3u, 0x6DA15918u, 0x6EADC0EEu, 0x6F2B8C15u,
  0x70D03CB2u, 0x71567049u, 0x725AE9BFu, 0x73DCA544u,
  0x7443DA53u, 0x75C596A8u, 0x76C90F5Eu, 0x774F43A5u,
  0x7871BD8Bu, 0x79F7F170u, 0x7AFB6886u, 0x7B7D247Du,
  0x7CE25B6Au, 0x7D641791u, 0x7E688E67u, 0x7FEEC29Cu,
  0x803347A4u, 0x81B50B5Fu, 0x82B992A9u, 0x833FDE52u,
  0x84A0A145u, 0x8526EDBEu, 0x862A7448u, 0x87AC38B3u,
  0x8892C69Du, 0x89148A66u, 0x8A181390u, 0x8B9E5F6Bu,
  0x8C01207Cu, 0x8D876C87u, 0x8E8BF571u, 0x8F0DB98Au,
  0x90F6092Du, 0x917045D6u, 0x927CDC20u, 0x93FA90DBu,
  0x9465EFCCu, 0x95E3A337u, 0x96EF3AC1u, 0x9769763Au,
  0x98578814u, 0x99D1C4EFu, 0x9ADD5D19u, 0x9B5B11E2u,
  0x9CC46EF5u, 0x9D42220Eu, 0x9E4EBBF8u, 0x9FC8F703u,
  0xA03F964Du, 0xA1B9DAB6u, 0xA2B54340u, 0xA3330FBBu,
  0xA4AC70ACu, 0xA52A3C57u, 0xA626A5A1u, 0xA7A0E95Au,
  0xA89E1774u, 0xA9185B8Fu, 0xAA14C279u, 0xAB928E82u,
  0xAC0DF195u, 0xAD8BBD6Eu, 0xAE872498u, 0xAF016863u,
  0xB0FAD8C4u, 0xB17C943Fu, 0xB2700DC9u, 0xB3F64132u,
  0xB4693E25u, 0xB5EF72DEu, 0xB6E3EB28u, 0xB765A7D3u,
  0xB85B59FDu, 0xB9DD1506u, 0xBAD18CF0u, 0xBB57C00Bu,
  0xBCC8BF1Cu, 0xBD4EF3E7u, 0xBE426A11u, 0xBFC426EAu,
  0xC02AE476u, 0xC1ACA88Du, 0xC2A0317Bu, 0xC3267D80u,
  0xC4B90297u, 0xC53F4E6Cu, 0xC633D79Au, 0xC7B59B61u,
  0xC88B654Fu, 0xC90D29B4u, 0xCA01B042u, 0xCB87FCB9u,
  0xCC1883AEu, 0xCD9ECF55u, 0xCE9256A3u, 0xCF141A58u,
  0xD0EFAAFFu, 0xD169E604u, 0xD2657FF2u, 0xD3E33309u,
  0xD47C4C1Eu, 0xD5FA00E5u, 0xD6F69913u, 0xD770D5E8u,
  0xD84E2BC6u, 0xD9C8673Du, 0xDAC4FECBu, 0xDB42B230u,
  0xDCDDCD27u, 0xDD5B81DCu, 0xDE57182Au, 0xDFD154D1u,
  0xE026359Fu, 0xE1A07964u, 0xE2ACE092u, 0xE32AAC69u,
  0xE4B5D37Eu, 0xE5339F85u, 0xE63F0673u, 0xE7B94A88u,
  0xE887B4A6u, 0xE901F85Du, 0xEA0D61ABu, 0xEB8B2D50u,
  0xEC145247u, 0xED921EBCu, 0xEE9E874Au, 0xEF18CBB1u,
  0xF0E37B16u, 0xF16537EDu, 0xF269AE1Bu, 0xF3EFE2E0u,
  0xF4709DF7u, 0xF5F6D10Cu, 0xF6FA48FAu, 0xF77C0401u,
  0xF842FA2Fu, 0xF9C4B6D4u, 0xFAC82F22u, 0xFB4E63D9u,
  0xFCD11CCEu, 0xFD575035u, 0xFE5BC9C3u, 0xFFDD8538u,
};

unsigned int crc24q_hash(const unsigned char *data, const int &len, unsigned int crc = 0){
  for(int i(0); i < len; i++){
    crc = (crc << 8) ^ crc24q[data[i] ^ (unsigned char)(crc >> 16)];
  }
  return (crc & 0x00FFFFFFu);
}
};

BOOST_AUTO_TEST_CASE(parity_calculation){
  typedef boost::uint32_t u32_t;
  static const u32_t packet[][9] = {
    // UBX f9p_ppp_1224/rover.ubx
    {0x9a663028, 0x02006014, 0x017f8000, 0x80cb89aa, 0x87f6bf17, 0xd504601f, 0xffc012e2, 0x50c48d31, 0xbf000000},
    {0xc6fc0000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x085c16d4, 0xff000000},
    {0xc6fc0000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x085c16d4, 0xc1ffffff},
    {0x53fc0000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x018036e6, 0x8217ff00},
    {0x5307ffff, 0xfffc0000, 0x00000000, 0x00000000, 0x0000a100, 0x00000000, 0x00000000, 0x65232ce6, 0x8297ff00},
    {0x9a085ffc, 0x00bfffff, 0xbfffffdf, 0xffffbff9, 0xffdffdff, 0xdfff9956, 0x1bd97bbb, 0x9eb4ca31, 0x8317ff7f},
    {0x9a0a5ffc, 0x004007ff, 0xbfffffdf, 0xfffffff9, 0xffdffdff, 0xdfff9956, 0x1bd97bbb, 0xac8d2f71, 0x8397ff7f},
    {0x9a085ffc, 0x003fffff, 0xbffc001f, 0xffffbffd, 0xffdffdff, 0xdfff9956, 0x1bd97bbb, 0x81595ef1, 0x8357ff7f},
    {0xc60c5ffd, 0xffc005ff, 0xc001ffdf, 0xfdffc001, 0xffdffdff, 0xc00bb97a, 0xbbb97bb9, 0x6585bc14, 0xc490037f},
    {0xc60e5ffd, 0xffc009ff, 0xc005ffdf, 0xfdffc001, 0xffdffdff, 0xc00bb97a, 0xbbb97bb9, 0x7523b594, 0xc450027f},
    {0xc60d5ffd, 0xffc009ff, 0xc009ffdf, 0xfdffc001, 0xffdffdff, 0xc00bb97a, 0xbbb97bb9, 0x7db79854, 0xc410037f},
    {0x5312400d, 0xfffffbff, 0xbffdffff, 0xe7fefff5, 0xffdffdff, 0xdffd79e1, 0x7aaabfff, 0xed3debe6, 0x9c492f96},
    {0x53114009, 0xfffffbff, 0x7ffdffff, 0xe7fefff5, 0xffdffdff, 0xdffd79e1, 0x7aaabfff, 0xcfd0e8a6, 0x9c520fb6},
    {0x5310400d, 0xffffffff, 0x4001ffff, 0xe7fefff5, 0xffdffdff, 0xdffd79e1, 0x7aaabfff, 0xdbfed9a6, 0x9c482da6},
    {0x9a7124be, 0x58319080, 0xa1e1189d, 0xc90591ba, 0x54537588, 0x406eecba, 0xfd5fa7e2, 0xc1bf63b1, 0x99a01dff},
    {0x9a71483e, 0xda0d4107, 0x8743f49e, 0x4f7fe2c7, 0x53a3f130, 0x05e40c1f, 0xefd43a01, 0x2f0818f1, 0x99a01dff},
    {0x9a7120b6, 0x9bb7b087, 0xfc16fdb7, 0x50605ec5, 0x65c2a0f0, 0x0564079d, 0x2f581b00, 0xe8650e31, 0x99a01dff},
    {0xc6668077, 0xfc008024, 0x00000000, 0x004b75be, 0x0600005f, 0xfe004000, 0x200032dd, 0x5ba749d4, 0xc714ffff},
    {0xc6668077, 0xfc008027, 0xff808000, 0x004b75be, 0x0600005f, 0xfe004020, 0x200012dd, 0x6834c714, 0xd2528000},
    {0xc6668077, 0xfc008027, 0xff808000, 0x004b75be, 0x0600005f, 0xfe004020, 0x200012dd, 0x6834c714, 0xff000000},
    {0x531c53ff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xc0000000, 0x00000000, 0x179520e6, 0x8257ff00},
  };
  for(std::size_t i(0); i < sizeof(packet) / sizeof(packet[0]); ++i){

    unsigned char temp[sizeof(packet[0])];
    for(std::size_t j(0), k(0); j < sizeof(packet[0]) / sizeof(packet[0][0]); ++j){
      temp[k++] = (packet[i][j] >> 24) & 0xFF;
      temp[k++] = (packet[i][j] >> 16) & 0xFF;
      temp[k++] = (packet[i][j] >> 8) & 0xFF;
      temp[k++] = packet[i][j] & 0xFF;
    }
    BOOST_CHECK_EQUAL(rtklib::crc24q(temp, 28), gpsd::crc24q_hash(temp, 28));
    BOOST_CHECK_EQUAL(rtklib::crc24q(temp, 28), gpsd::crc24q_hash(&temp[27], 1, gpsd::crc24q_hash(temp, 27)));
    BOOST_CHECK_EQUAL(wikipedia::crc24q(temp, 224), gpsd::crc24q_hash(temp, 28));
    BOOST_CHECK_EQUAL(wikipedia::crc24q_tbl(temp, 224), gpsd::crc24q_hash(temp, 28));

    BOOST_CHECK_EQUAL(wikipedia::crc24q(temp, 226), (packet[i][7] & 0x3FFFFFC0) >> 6);
    BOOST_CHECK_EQUAL(wikipedia::crc24q_tbl(&temp[28], 2, gpsd::crc24q_hash(temp, 28)), (packet[i][7] & 0x3FFFFFC0) >> 6);
    BOOST_CHECK_EQUAL(wikipedia::crc24q_tbl(temp, 226), (packet[i][7] & 0x3FFFFFC0) >> 6);

    {
      u32_t copy[sizeof(packet[0]) / sizeof(packet[0][0])];
      std::memcpy(copy, packet[i], sizeof(copy));
      space_node_t::DataBlock::parity_set(copy);
      BOOST_CHECK_EQUAL(copy[7], packet[i][7]);
    }
  }
}

BOOST_AUTO_TEST_CASE(dump_ephemeris){
  typedef boost::uint32_t u32_t;
  static const u32_t packet[][9] = {
    // UBX f9p_ppp_1224/rover.ubx
    {0x5327365b, 0x81b53961, 0x54406ac7, 0x1ff085b0, 0x2f77fa4c, 0x0048003f, 0xe00806be, 0xe1b9a1a6, 0x8297ff00},
    {0x5325025b, 0x818de1eb, 0xf480555a, 0x8000bfd0, 0x02a001cf, 0xffff0040, 0x00000040, 0x65122de6, 0x8217ff00},
    {0x53247a5b, 0x816149ef, 0x6cf49f47, 0x3ff158c0, 0x0c17f6c3, 0xffe3ff80, 0x000bfe3f, 0x91a38166, 0x8257ff00},
    {0x53273a5d, 0x81b539c0, 0x34406a6a, 0x9ff091e0, 0x2f77fa20, 0x0050003f, 0xe00805fe, 0xf957c1a6, 0x8297ff00},
    {0x5325065d, 0x818de1f1, 0x5c805577, 0x4000bfa0, 0x02c001cf, 0xffff0040, 0x00000040, 0x16870426, 0x8257ff00},
    {0x53247e5d, 0x81614a07, 0x3cf49eb3, 0x3ff154c0, 0x0bb7f6cf, 0xffebff80, 0x000bfdff, 0x087a6f66, 0x8297ff00},
    {0xc6273a5d, 0x81b539c0, 0x34406a6a, 0x9ff091e0, 0x2f77fa20, 0x0050003f, 0xe00805fe, 0xf08be194, 0xff000000},
    {0xc625065d, 0x818de1f1, 0x5c805577, 0x4000bfa0, 0x02c001cf, 0xffff0040, 0x00000040, 0x1f5b2414, 0xd2528000},
    {0xc6247e5d, 0x81614a07, 0x3cf49eb3, 0x3ff154c0, 0x0bb7f6cf, 0xffebff80, 0x000bfdff, 0x01a64f54, 0xff000000},
    {0xc6273a5d, 0x81b539c0, 0x34406a6a, 0x9ff091e0, 0x2f77fa20, 0x0050003f, 0xe00805fe, 0xf08be194, 0xd9a01dff},
    {0xc625065d, 0x818de1f1, 0x5c805577, 0x4000bfa0, 0x02c001cf, 0xffff0040, 0x00000040, 0x1f5b2414, 0xd2508000},
    {0x9a273e5f, 0x81b53a1f, 0x14406a0b, 0x5ff09f50, 0x2f67f9ec, 0x0058003f, 0xe008053e, 0xcf121c71, 0x9c482db6},
    {0x9a250a5f, 0x818de1f7, 0x04805594, 0x0000bf70, 0x02e001cb, 0xfffe0040, 0x0000007f, 0xf6848f71, 0x9c462ae9},
    {0x9a24825f, 0x81614a1e, 0x4cf49e1f, 0xfff15200, 0x0b57f6cf, 0xfff2ff80, 0x000bfd7f, 0x1c5b4a31, 0xbf000000},
    {0x9a273e5f, 0x81b53a1f, 0x14406a0b, 0x5ff09f50, 0x2f67f9ec, 0x0058003f, 0xe008053e, 0xcf121c71, 0x9120c031},
    {0xc6250a5f, 0x818de1f7, 0x04805594, 0x0000bf70, 0x02e001cb, 0xfffe0040, 0x0000007f, 0xe25feed4, 0xd9ba8c01},
    {0x5324825f, 0x81614a1e, 0x4cf49e1f, 0xfff15200, 0x0b57f6cf, 0xfff2ff80, 0x000bfd7f, 0x015c0ba6, 0x8297ff00},
    {0xc6273e5f, 0x81b53a1f, 0x14406a0b, 0x5ff09f50, 0x2f67f9ec, 0x0058003f, 0xe008053e, 0xdbc97dd4, 0xd989c9ff},
    {0xc6250a5f, 0x818de1f7, 0x04805594, 0x0000bf70, 0x02e001cb, 0xfffe0040, 0x0000007f, 0xe25feed4, 0xd989c9ff},
    {0xc624825f, 0x81614a1e, 0x4cf49e1f, 0xfff15200, 0x0b57f6cf, 0xfff2ff80, 0x000bfd7f, 0x08802b94, 0xd989c9ff},
    {0x9a274261, 0x81b53a7d, 0xd44069a8, 0xdff0ae10, 0x2f57f9c0, 0x0060003f, 0xe008047f, 0x08997571, 0x8714ffff},
  };
  for(std::size_t i(0); i < sizeof(packet) / sizeof(packet[0]); ++i){
    BOOST_CHECK_EQUAL(space_node_t::DataBlock::message_type(packet[i]), space_node_t::GEO_NAVIGATION);

    u32_t copy[sizeof(packet[0]) / sizeof(packet[0][0])];
    std::memcpy(copy, packet[i], sizeof(copy));

    typedef space_node_t::SatelliteProperties::Ephemeris eph_t;
    eph_t::raw_t raw = eph_t::raw_t::fetch(packet[i]); // parse raw
    raw = (eph_t)raw; // raw -> ephemeris -> raw
    raw.dump(copy);

    BOOST_CHECK_EQUAL(std::memcmp(packet[i], copy, sizeof(copy)), 0);
  }
}

BOOST_AUTO_TEST_SUITE_END()
