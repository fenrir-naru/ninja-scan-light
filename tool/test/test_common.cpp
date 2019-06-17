#include "analyze_common.h"

#define BOOST_TEST_MAIN
#include <boost/test/included/unit_test.hpp>

using namespace std;

BOOST_AUTO_TEST_SUITE(common)

BOOST_AUTO_TEST_CASE(roll_over_monitor){
  typedef CalendarTime<double>::Converter::roll_over_monitor_t monitor_t;
  monitor_t monitor;

  monitor.reset();
  for(int i(0); i < 3; i++){
    for(double itow(0); itow < monitor_t::one_week; itow += monitor_t::threshold / 2){
      monitor(itow);
      BOOST_CHECK_EQUAL(false, monitor.abnormal_jump_detected);
      BOOST_CHECK_EQUAL(i * monitor_t::one_week, monitor.roll_over_offset);
    }
  }

  monitor.reset();
  for(int i(-1); i > -3; i--){
    for(double itow(monitor_t::one_week - (monitor_t::threshold / 2)); itow > 0; itow -= monitor_t::threshold / 2){
      monitor(itow);
      BOOST_CHECK_EQUAL(false, monitor.abnormal_jump_detected);
      BOOST_CHECK_EQUAL(i * monitor_t::one_week, monitor.roll_over_offset);
    }
  }

  monitor.reset();
  monitor(monitor_t::threshold);
  BOOST_CHECK_EQUAL(true, monitor.abnormal_jump_detected);

  monitor.reset();
  monitor(monitor_t::one_week - monitor_t::threshold);
  BOOST_CHECK_EQUAL(true, monitor.abnormal_jump_detected);
}

BOOST_AUTO_TEST_SUITE_END()
