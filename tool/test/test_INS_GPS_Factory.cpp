#include <iostream>

#include "navigation/INS_GPS_Factory.h"

#include <boost/type_traits/is_same.hpp>

#define BOOST_TEST_MAIN
#include <boost/test/included/unit_test.hpp>

#if !defined(BOOST_VERSION)
#define BOOST_FIXTURE_TEST_SUITE(name, fixture)
#define BOOST_AUTO_TEST_SUITE_END()
#define BOOST_AUTO_TEST_CASE(name) void name()
#endif

using namespace std;

#ifndef DEBUG_PRINT
#define DEBUG_PRINT false
#endif
#define dbg(exp, force) \
if((force) || DEBUG_PRINT){cerr << endl << exp;} \
else{ostream(NULL) << exp;}

typedef INS_GPS_Factory_Options opt_t;
typedef INS_GPS_Factory<> factory_t;

template <class T>
struct test_t {
  static void print(ostream &out){
    out << " [end]" << endl;
  }
};


template <class T, template <class> class Filter>
struct test_t<typename opt_t::kf_t<T, Filter> >{
  static void print(ostream &out){
    out << " KF";
    test_t<T>::print(out);
  }
};

template <class T, class EGM>
struct test_t<typename opt_t::egm_t<T, EGM> >{
  static void print(ostream &out){
    out << " EGM";
    test_t<T>::print(out);
  }
};

template <class T>
struct test_t<typename opt_t::bias_t<T> >{
  static void print(ostream &out){
    out << " Bias";
    test_t<T>::print(out);
  }
};

BOOST_AUTO_TEST_SUITE(Factory)

BOOST_AUTO_TEST_CASE(option){

  // generated with test_INS_GPS_Factory_option_gen.rb
  BOOST_CHECK((boost::is_same<
      typename factory_t::template egm<void>::options_t,
      typename opt_t::egm_t<void, void> >::value));
  BOOST_CHECK((boost::is_same<
      typename factory_t::template kf<KalmanFilter>::options_t,
      typename opt_t::kf_t<void, KalmanFilter> >::value));
  BOOST_CHECK((boost::is_same<
      typename factory_t::template bias<>::options_t,
      typename opt_t::bias_t<void > >::value));
  BOOST_CHECK((boost::is_same<
      typename factory_t::template egm<void>::template kf<KalmanFilter>::options_t,
      typename opt_t::kf_t<typename opt_t::egm_t<void, void>, KalmanFilter> >::value));
  BOOST_CHECK((boost::is_same<
      typename factory_t::template egm<void>::template bias<>::options_t,
      typename opt_t::bias_t<typename opt_t::egm_t<void, void> > >::value));
  BOOST_CHECK((boost::is_same<
      typename factory_t::template kf<KalmanFilter>::template egm<void>::options_t,
      typename opt_t::kf_t<typename opt_t::egm_t<void, void>, KalmanFilter> >::value));
  BOOST_CHECK((boost::is_same<
      typename factory_t::template kf<KalmanFilter>::template bias<>::options_t,
      typename opt_t::bias_t<typename opt_t::kf_t<void, KalmanFilter> > >::value));
  BOOST_CHECK((boost::is_same<
      typename factory_t::template bias<>::template egm<void>::options_t,
      typename opt_t::bias_t<typename opt_t::egm_t<void, void> > >::value));
  BOOST_CHECK((boost::is_same<
      typename factory_t::template bias<>::template kf<KalmanFilter>::options_t,
      typename opt_t::bias_t<typename opt_t::kf_t<void, KalmanFilter> > >::value));
  BOOST_CHECK((boost::is_same<
      typename factory_t::template egm<void>::template kf<KalmanFilter>::template egm<void>::options_t,
      typename opt_t::kf_t<typename opt_t::egm_t<void, void>, KalmanFilter> >::value));
  BOOST_CHECK((boost::is_same<
      typename factory_t::template egm<void>::template kf<KalmanFilter>::template bias<>::options_t,
      typename opt_t::bias_t<typename opt_t::kf_t<typename opt_t::egm_t<void, void>, KalmanFilter> > >::value));
  BOOST_CHECK((boost::is_same<
      typename factory_t::template egm<void>::template bias<>::template egm<void>::options_t,
      typename opt_t::bias_t<typename opt_t::egm_t<void, void> > >::value));
  BOOST_CHECK((boost::is_same<
      typename factory_t::template egm<void>::template bias<>::template kf<KalmanFilter>::options_t,
      typename opt_t::bias_t<typename opt_t::kf_t<typename opt_t::egm_t<void, void>, KalmanFilter> > >::value));
  BOOST_CHECK((boost::is_same<
      typename factory_t::template kf<KalmanFilter>::template egm<void>::template kf<KalmanFilter>::options_t,
      typename opt_t::kf_t<typename opt_t::egm_t<void, void>, KalmanFilter> >::value));
  BOOST_CHECK((boost::is_same<
      typename factory_t::template kf<KalmanFilter>::template egm<void>::template bias<>::options_t,
      typename opt_t::bias_t<typename opt_t::kf_t<typename opt_t::egm_t<void, void>, KalmanFilter> > >::value));
  BOOST_CHECK((boost::is_same<
      typename factory_t::template kf<KalmanFilter>::template bias<>::template egm<void>::options_t,
      typename opt_t::bias_t<typename opt_t::kf_t<typename opt_t::egm_t<void, void>, KalmanFilter> > >::value));
  BOOST_CHECK((boost::is_same<
      typename factory_t::template kf<KalmanFilter>::template bias<>::template kf<KalmanFilter>::options_t,
      typename opt_t::bias_t<typename opt_t::kf_t<void, KalmanFilter> > >::value));
  BOOST_CHECK((boost::is_same<
      typename factory_t::template bias<>::template egm<void>::template kf<KalmanFilter>::options_t,
      typename opt_t::bias_t<typename opt_t::kf_t<typename opt_t::egm_t<void, void>, KalmanFilter> > >::value));
  BOOST_CHECK((boost::is_same<
      typename factory_t::template bias<>::template egm<void>::template bias<>::options_t,
      typename opt_t::bias_t<typename opt_t::egm_t<void, void> > >::value));
  BOOST_CHECK((boost::is_same<
      typename factory_t::template bias<>::template kf<KalmanFilter>::template egm<void>::options_t,
      typename opt_t::bias_t<typename opt_t::kf_t<typename opt_t::egm_t<void, void>, KalmanFilter> > >::value));
  BOOST_CHECK((boost::is_same<
      typename factory_t::template bias<>::template kf<KalmanFilter>::template bias<>::options_t,
      typename opt_t::bias_t<typename opt_t::kf_t<void, KalmanFilter> > >::value));
  BOOST_CHECK((boost::is_same<
      typename factory_t::template egm<void>::template kf<KalmanFilter>::template egm<void>::template kf<KalmanFilter>::options_t,
      typename opt_t::kf_t<typename opt_t::egm_t<void, void>, KalmanFilter> >::value));
  BOOST_CHECK((boost::is_same<
      typename factory_t::template egm<void>::template kf<KalmanFilter>::template egm<void>::template bias<>::options_t,
      typename opt_t::bias_t<typename opt_t::kf_t<typename opt_t::egm_t<void, void>, KalmanFilter> > >::value));
  BOOST_CHECK((boost::is_same<
      typename factory_t::template egm<void>::template kf<KalmanFilter>::template bias<>::template egm<void>::options_t,
      typename opt_t::bias_t<typename opt_t::kf_t<typename opt_t::egm_t<void, void>, KalmanFilter> > >::value));
  BOOST_CHECK((boost::is_same<
      typename factory_t::template egm<void>::template kf<KalmanFilter>::template bias<>::template kf<KalmanFilter>::options_t,
      typename opt_t::bias_t<typename opt_t::kf_t<typename opt_t::egm_t<void, void>, KalmanFilter> > >::value));
  BOOST_CHECK((boost::is_same<
      typename factory_t::template egm<void>::template bias<>::template egm<void>::template kf<KalmanFilter>::options_t,
      typename opt_t::bias_t<typename opt_t::kf_t<typename opt_t::egm_t<void, void>, KalmanFilter> > >::value));
  BOOST_CHECK((boost::is_same<
      typename factory_t::template egm<void>::template bias<>::template egm<void>::template bias<>::options_t,
      typename opt_t::bias_t<typename opt_t::egm_t<void, void> > >::value));
  BOOST_CHECK((boost::is_same<
      typename factory_t::template egm<void>::template bias<>::template kf<KalmanFilter>::template egm<void>::options_t,
      typename opt_t::bias_t<typename opt_t::kf_t<typename opt_t::egm_t<void, void>, KalmanFilter> > >::value));
  BOOST_CHECK((boost::is_same<
      typename factory_t::template egm<void>::template bias<>::template kf<KalmanFilter>::template bias<>::options_t,
      typename opt_t::bias_t<typename opt_t::kf_t<typename opt_t::egm_t<void, void>, KalmanFilter> > >::value));
  BOOST_CHECK((boost::is_same<
      typename factory_t::template kf<KalmanFilter>::template egm<void>::template kf<KalmanFilter>::template egm<void>::options_t,
      typename opt_t::kf_t<typename opt_t::egm_t<void, void>, KalmanFilter> >::value));
  BOOST_CHECK((boost::is_same<
      typename factory_t::template kf<KalmanFilter>::template egm<void>::template kf<KalmanFilter>::template bias<>::options_t,
      typename opt_t::bias_t<typename opt_t::kf_t<typename opt_t::egm_t<void, void>, KalmanFilter> > >::value));
  BOOST_CHECK((boost::is_same<
      typename factory_t::template kf<KalmanFilter>::template egm<void>::template bias<>::template egm<void>::options_t,
      typename opt_t::bias_t<typename opt_t::kf_t<typename opt_t::egm_t<void, void>, KalmanFilter> > >::value));
  BOOST_CHECK((boost::is_same<
      typename factory_t::template kf<KalmanFilter>::template egm<void>::template bias<>::template kf<KalmanFilter>::options_t,
      typename opt_t::bias_t<typename opt_t::kf_t<typename opt_t::egm_t<void, void>, KalmanFilter> > >::value));
  BOOST_CHECK((boost::is_same<
      typename factory_t::template kf<KalmanFilter>::template bias<>::template egm<void>::template kf<KalmanFilter>::options_t,
      typename opt_t::bias_t<typename opt_t::kf_t<typename opt_t::egm_t<void, void>, KalmanFilter> > >::value));
  BOOST_CHECK((boost::is_same<
      typename factory_t::template kf<KalmanFilter>::template bias<>::template egm<void>::template bias<>::options_t,
      typename opt_t::bias_t<typename opt_t::kf_t<typename opt_t::egm_t<void, void>, KalmanFilter> > >::value));
  BOOST_CHECK((boost::is_same<
      typename factory_t::template kf<KalmanFilter>::template bias<>::template kf<KalmanFilter>::template egm<void>::options_t,
      typename opt_t::bias_t<typename opt_t::kf_t<typename opt_t::egm_t<void, void>, KalmanFilter> > >::value));
  BOOST_CHECK((boost::is_same<
      typename factory_t::template kf<KalmanFilter>::template bias<>::template kf<KalmanFilter>::template bias<>::options_t,
      typename opt_t::bias_t<typename opt_t::kf_t<void, KalmanFilter> > >::value));
  BOOST_CHECK((boost::is_same<
      typename factory_t::template bias<>::template egm<void>::template kf<KalmanFilter>::template egm<void>::options_t,
      typename opt_t::bias_t<typename opt_t::kf_t<typename opt_t::egm_t<void, void>, KalmanFilter> > >::value));
  BOOST_CHECK((boost::is_same<
      typename factory_t::template bias<>::template egm<void>::template kf<KalmanFilter>::template bias<>::options_t,
      typename opt_t::bias_t<typename opt_t::kf_t<typename opt_t::egm_t<void, void>, KalmanFilter> > >::value));
  BOOST_CHECK((boost::is_same<
      typename factory_t::template bias<>::template egm<void>::template bias<>::template egm<void>::options_t,
      typename opt_t::bias_t<typename opt_t::egm_t<void, void> > >::value));
  BOOST_CHECK((boost::is_same<
      typename factory_t::template bias<>::template egm<void>::template bias<>::template kf<KalmanFilter>::options_t,
      typename opt_t::bias_t<typename opt_t::kf_t<typename opt_t::egm_t<void, void>, KalmanFilter> > >::value));
  BOOST_CHECK((boost::is_same<
      typename factory_t::template bias<>::template kf<KalmanFilter>::template egm<void>::template kf<KalmanFilter>::options_t,
      typename opt_t::bias_t<typename opt_t::kf_t<typename opt_t::egm_t<void, void>, KalmanFilter> > >::value));
  BOOST_CHECK((boost::is_same<
      typename factory_t::template bias<>::template kf<KalmanFilter>::template egm<void>::template bias<>::options_t,
      typename opt_t::bias_t<typename opt_t::kf_t<typename opt_t::egm_t<void, void>, KalmanFilter> > >::value));
  BOOST_CHECK((boost::is_same<
      typename factory_t::template bias<>::template kf<KalmanFilter>::template bias<>::template egm<void>::options_t,
      typename opt_t::bias_t<typename opt_t::kf_t<typename opt_t::egm_t<void, void>, KalmanFilter> > >::value));
  BOOST_CHECK((boost::is_same<
      typename factory_t::template bias<>::template kf<KalmanFilter>::template bias<>::template kf<KalmanFilter>::options_t,
      typename opt_t::bias_t<typename opt_t::kf_t<void, KalmanFilter> > >::value));
}

BOOST_AUTO_TEST_SUITE_END()
