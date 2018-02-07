#include <iostream>
#include <cmath>

#include "navigation/SBAS.h"

#define BOOST_TEST_MAIN
#include <boost/test/included/unit_test.hpp>

#if !defined(BOOST_VERSION)
#define BOOST_FIXTURE_TEST_SUITE(name, fixture)
#define BOOST_AUTO_TEST_SUITE_END()
#define BOOST_AUTO_TEST_CASE(name) void name()
#endif

using namespace std;

typedef SBAS_SpaceNode<double> space_node_t;

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

BOOST_AUTO_TEST_CASE(igp_pos){
  for(unsigned band(0); band < 11; ++band){
    for(unsigned mask(0); mask < 201; ++mask){
      if((band == 8) && (mask >= 200)){
        break;
      }else if((band >= 9) && (mask >= 192)){
        break;
      }
      space_node_t::igp_pos_t pos(space_node_t::igp_pos(band, mask));
      BOOST_TEST_MESSAGE("(band, mask) = (" << band << ", " << mask << ") => " << pos.latitude_deg << ", " << pos.longitude_deg);
      BOOST_REQUIRE_EQUAL(pos.latitude_deg, igp_lat_lng[band][mask][0]);
      BOOST_REQUIRE_EQUAL(pos.longitude_deg, igp_lat_lng[band][mask][1]);
    }
  }
}

BOOST_AUTO_TEST_CASE(igp_pos_index){
  for(unsigned band(0); band < 11; ++band){
    for(unsigned mask(0); mask < 201; ++mask){
      if((band == 8) && (mask >= 200)){
        break;
      }else if((band >= 9) && (mask >= 192)){
        break;
      }
      { // on grid
        space_node_t::igp_pos_index_t idx(space_node_t::igp_pos_index(igp_lat_lng[band][mask][0], igp_lat_lng[band][mask][1]));
        BOOST_TEST_MESSAGE("(band, mask) = (" << band << ", " << mask << ") => " << idx.lat_index << ", " << idx.lng_index);
        BOOST_REQUIRE_EQUAL(((space_node_t::igp_pos_t)idx).latitude_deg, igp_lat_lng[band][mask][0]);
        BOOST_REQUIRE_EQUAL(((space_node_t::igp_pos_t)idx).longitude_deg, igp_lat_lng[band][mask][1]);
      }
      { // near grid
        double
            lat((igp_lat_lng[band][mask][0] >= 0 ? 0.5 : -0.5) + igp_lat_lng[band][mask][0]), // little north,
            lng(0.5 + igp_lat_lng[band][mask][1]); // little east
        space_node_t::igp_pos_index_t idx(space_node_t::igp_pos_index(lat, lng));
        BOOST_TEST_MESSAGE("(band, mask) = (" << band << ", " << mask << ") => "
            << idx.lat_index << "(" << lat << "), "
            << idx.lng_index << "(" << lng << ")");
        BOOST_REQUIRE_EQUAL(((space_node_t::igp_pos_t)idx).latitude_deg, igp_lat_lng[band][mask][0]);
        if(lat > 85){
          int lng_grid(90 * floor(lng / 90));
          if(lng_grid == 180){lng_grid = -180;}
          BOOST_REQUIRE_EQUAL(((space_node_t::igp_pos_t)idx).longitude_deg, lng_grid);
        }else if(lat < -85){
          int lng_grid(40 + 90 * floor((lng - 40) / 90));
          if(lng_grid == -230){lng_grid = 130;}
          BOOST_REQUIRE_EQUAL(((space_node_t::igp_pos_t)idx).longitude_deg, lng_grid);
        }else{
          BOOST_REQUIRE_EQUAL(((space_node_t::igp_pos_t)idx).longitude_deg, igp_lat_lng[band][mask][1]);
        }
      }

      static const int roll_over_i[] = {-360, 360};
      for(int i(0); i < sizeof(roll_over_i) / sizeof(roll_over_i[0]); ++i){ // +/- roll over
        space_node_t::igp_pos_index_t idx(space_node_t::igp_pos_index(
            igp_lat_lng[band][mask][0],
            roll_over_i[i] + igp_lat_lng[band][mask][1]));
        BOOST_TEST_MESSAGE("(band, mask) = (" << band << ", " << mask << ") => " << idx.lat_index << ", " << idx.lng_index);
        BOOST_REQUIRE_EQUAL(((space_node_t::igp_pos_t)idx).latitude_deg, igp_lat_lng[band][mask][0]);
        BOOST_REQUIRE_EQUAL(((space_node_t::igp_pos_t)idx).longitude_deg, igp_lat_lng[band][mask][1]);
      }

      static const double roll_over_f[] = {-360.0, 360.0};
      for(int i(0); i < sizeof(roll_over_f) / sizeof(roll_over_f[0]); ++i){ // +/- roll over
        space_node_t::igp_pos_index_t idx(space_node_t::igp_pos_index(
            (double)igp_lat_lng[band][mask][0],
            roll_over_f[i] + igp_lat_lng[band][mask][1]));
        BOOST_TEST_MESSAGE("(band, mask) = (" << band << ", " << mask << ") => " << idx.lat_index << ", " << idx.lng_index);
        BOOST_REQUIRE_EQUAL(((space_node_t::igp_pos_t)idx).latitude_deg, igp_lat_lng[band][mask][0]);
        BOOST_REQUIRE_EQUAL(((space_node_t::igp_pos_t)idx).longitude_deg, igp_lat_lng[band][mask][1]);
      }
    }
  }
}

BOOST_AUTO_TEST_SUITE_END()
