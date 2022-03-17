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

BOOST_AUTO_TEST_SUITE_END()