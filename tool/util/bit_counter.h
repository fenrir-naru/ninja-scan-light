#ifndef __BIT_COUNTER__
#define __BIT_COUNTER__

#include <climits>

using namespace std;

/**
 * Counting bits utility
 */
template <class T>
struct BitCounter {

  static const int num_of_bits = CHAR_BIT * sizeof(T);

  private:
    template <int Interval>
    static T make_mask(){
      /*
       * @see http://www.nminoru.jp/~nminoru/programming/bitcount.html
       * 
       * 01010101010101010101010101010101  1 32
       * 00110011001100110011001100110011  2 32
       * 00001111000011110000111100001111  4 32
       * 00000000111111110000000011111111  8 32
       * 00000000000000001111111111111111 16 32 
       */

      T mask(1);
      for(int i(1); i < Interval; i++){
        mask <<= 1;
        mask |= 0x01;
      }
      /*
      // Equivalent to the following if T is built-in type
      mask <<= Interval;
      mask = (mask & (-mask)) - 1;
      */ 

      T res(mask);
      for(unsigned int i(1); i < num_of_bits / Interval / 2; i++){
        res <<= Interval;
        res <<= Interval;
        res |= mask;
      }
      return res;
    }

    template <int Interval, class U = void>
    struct count_loop {
      static T run(T bits){
        bits = count_loop<(Interval >> 1)>::run(bits);
        static const T mask(make_mask<Interval>());
        // bit shift and add operators are required at least
        return (bits & mask) + ((bits >> Interval) & mask);
      }
    };
    template <class U>
    struct count_loop<1, U> {
      static T run(T bits){
        static const T mask(make_mask<1>());
        return (bits & mask) + ((bits >> 1) & mask);
      }
    };

  public:
    /**
     * Count bits in a value
     * @param results
     */
    static T count(const T &v) {
      return count_loop<(num_of_bits >> 1)>::run(v);
    }

    /**
     * Count rightmost zeros before the first one (Number of trailing zeros)
     * Be careful, if input equals to 0, then total number of corresponding type bits
     * will be returned like (unsigned int)0 => 32.
     * @param bits results
     */
    static T ntz(const T &v) {
      return count((T)((~v) & (v - 1)));
    }
};

#endif /* __BIT_COUNTER__ */
