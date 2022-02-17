#include <ctime>
#include <climits>
#include <iostream>
#include <bitset>
#include <string>
#include <algorithm>
#include <vector>

#include "param/bit_array.h"

#include <boost/random.hpp>
#include <boost/random/random_device.hpp>

#include <boost/format.hpp>
#include <boost/cstdint.hpp>

#define BOOST_TEST_MAIN
#include <boost/test/included/unit_test.hpp>

using namespace std;
using boost::format;

BOOST_AUTO_TEST_SUITE(bit_array)

struct Fixture {
  boost::random::mt19937 gen;
  boost::random::uniform_int_distribution<> bin_dist;

  Fixture()
      : gen(0), //static_cast<unsigned long>(time(0))
      bin_dist(0, 1)
      {}
  ~Fixture(){}

  bool get_bool(){
    return bin_dist(gen) == 1;
  }
};

template <size_t N, class BitArrayT>
void check_bit_array(const bitset<N> &b, const BitArrayT &bit_array){
  for(int i(0); i < (int)b.size(); ++i){
    int j(i + (sizeof(unsigned int) * CHAR_BIT) - 1);
    if(j >= (int)b.size()){j = (int)b.size() - 1;}
    for(; j >= i; --j){
      unsigned int pattern(bit_array.pattern(i, j));
      BOOST_TEST_MESSAGE(format("(%d, %d) => 0x%08x") % i % j % pattern);
      for(int k1(i), k2(0); k1 <= j; ++k1, ++k2){
        BOOST_REQUIRE_EQUAL((pattern & 0x1), (b[k1] ? 1 : 0));
        pattern >>= 1;
      }
    }
  }
}

BOOST_FIXTURE_TEST_CASE(bit_array, Fixture){
  for(int loop(0); loop < 0x400; loop++){
    bitset<64> b;
    std::vector<int> ones;
    for(unsigned int i(0); i < b.size(); ++i){
      b.set(i, get_bool());
      if(b[i]){ones.push_back((int)i);}
    }
    string b_str(b.to_string());
    reverse(b_str.begin(), b_str.end());
    string b_str2("");
    for(int i(0); i < 64; i += 8){
      b_str2.append(" ").append(b_str.substr(i, 8));
    }
    BOOST_TEST_MESSAGE(format("Origin(%d) LSB => MSB:%s") % loop % b_str2);

    {
      BOOST_TEST_MESSAGE("u8_t container");
      typedef BitArray<64, boost::uint8_t> bit_array_t;
      bit_array_t bit_array;
      for(unsigned int i(0); i < b.size(); ++i){
        bit_array.set(i, b[i]);
      }
      check_bit_array(b, bit_array);

      std::vector<int> ones2(bit_array.indices_one());
      BOOST_REQUIRE(std::equal(ones.begin(), ones.end(), ones2.begin()));
    }

    {
      BOOST_TEST_MESSAGE("u32_t container");
      typedef BitArray<64, boost::uint32_t> bit_array_t;
      bit_array_t bit_array;
      for(unsigned int i(0); i < b.size(); ++i){
        bit_array.set(i, b[i]);
      }
      check_bit_array(b, bit_array);

      std::vector<int> ones2(bit_array.indices_one());
      BOOST_REQUIRE(std::equal(ones.begin(), ones.end(), ones2.begin()));
    }

    {
      BOOST_TEST_MESSAGE("u64_t container");
      typedef BitArray<64, boost::uint64_t> bit_array_t;
      bit_array_t bit_array;
      for(unsigned int i(0); i < b.size(); ++i){
        bit_array.set(i, b[i]);
      }
      check_bit_array(b, bit_array);

      std::vector<int> ones2(bit_array.indices_one());
      BOOST_REQUIRE(std::equal(ones.begin(), ones.end(), ones2.begin()));
    }

    {
      BOOST_TEST_MESSAGE("container(u32_t) bigger than required capacity(24)");
      typedef BitArray<24, boost::uint32_t> bit_array_t;
      bit_array_t bit_array;
      for(unsigned int i(0); i < bit_array_t::max_size; ++i){
        bit_array.set(i, b[i]);
      }

      std::vector<int> ones2(bit_array.indices_one());
#if defined(__cplusplus) && (__cplusplus >= 201103L)
      BOOST_REQUIRE(std::equal(
          ones.begin(),
          std::remove_if(ones.begin(), ones.end(),
              [](const int &i)->bool{return i >= bit_array_t::max_size;}),
          ones2.begin()));
#else
      struct {
        bool operator()(const int &i) const {return i >= bit_array_t::max_size;}
      } predicate;
      BOOST_REQUIRE(std::equal(
          ones.begin(),
          std::remove_if(ones.begin(), ones.end(), predicate),
          ones2.begin()));
#endif
    }
  }
}

BOOST_AUTO_TEST_SUITE_END()
