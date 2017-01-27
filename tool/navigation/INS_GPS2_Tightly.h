/*
 *  INS_GPS2.h, header file to perform calculation of integration of INS and GPS.
 *  Copyright (C) 2015 M.Naruoka (fenrir)
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __INS_GPS2_TIGHTLY_H__
#define __INS_GPS2_TIGHTLY_H__

#include "INS.h"
#include "Filtered_INS2.h"
#include "GPS_SP.h"

template <
    typename BaseINS = INS<> >
class INS_ClockErrorEstimated : public BaseINS {
  public:
#if defined(__GNUC__) && (__GNUC__ < 5)
    typedef typename BaseINS::float_t float_t;
#else
    using typename BaseINS::float_t;
#endif

  protected:
    float_t m_clock_error;

  public:
    static const unsigned STATE_VALUES_WITHOUT_CLOCK_ERROR = BaseINS::STATE_VALUES;
    static const unsigned STATE_VALUES_CLOCK_ERROR = 1;
    static const unsigned STATE_VALUES; ///< Number of state values
    virtual unsigned state_values() const {return STATE_VALUES;}

    INS_ClockErrorEstimated()
        : BaseINS(),
          m_clock_error(0) {
    }

    INS_ClockErrorEstimated(const INS_ClockErrorEstimated &orig, const bool deepcopy = false)
        : BaseINS(orig, deepcopy),
          m_clock_error(orig.m_clock_error) {
    }

    virtual ~INS_ClockErrorEstimated(){}

    float_t &clock_error(){return m_clock_error;}

    using BaseINS::operator[];

    const float_t &operator[](const unsigned &index) const {
      switch(index){
        case STATE_VALUES_WITHOUT_CLOCK_ERROR: return m_clock_error;
        default: return BaseINS::operator[](index);
      }
    }
};

template <
    typename BaseINS>
const unsigned INS_ClockErrorEstimated<BaseINS>::STATE_VALUES
    = STATE_VALUES_WITHOUT_CLOCK_ERROR + STATE_VALUES_CLOCK_ERROR;

template <class BaseINS>
class Filtered_INS2_Property<INS_ClockErrorEstimated<BaseINS> >
    : public Filtered_INS2_Property<BaseINS> {
  public:
    static const unsigned P_SIZE_WITHOUT_CLOCK_ERROR
#if defined(_MSC_VER)
        = Filtered_INS2_Property<BaseINS>::P_SIZE
#endif
        ;
    static const unsigned Q_SIZE_WITHOUT_CLOCK_ERROR
#if defined(_MSC_VER)
        = Filtered_INS2_Property<BaseINS>::Q_SIZE
#endif
        ;
    static const unsigned P_SIZE
#if defined(_MSC_VER)
        = P_SIZE_WITHOUT_CLOCK_ERROR + INS_ClockErrorEstimated<BaseINS>::STATE_VALUES_BIAS
#endif
        ;
    static const unsigned Q_SIZE
#if defined(_MSC_VER)
        = Q_SIZE_WITHOUT_CLOCK_ERROR + INS_ClockErrorEstimated<BaseINS>::STATE_VALUES_BIAS;
#endif
        ;
};

#if !defined(_MSC_VER)
template <class BaseINS>
const unsigned Filtered_INS2_Property<INS_ClockErrorEstimated<BaseINS> >::P_SIZE_WITHOUT_CLOCK_ERROR
    = Filtered_INS2_Property<BaseINS>::P_SIZE;

template <class BaseINS>
const unsigned Filtered_INS2_Property<INS_ClockErrorEstimated<BaseINS> >::Q_SIZE_WITHOUT_CLOCK_ERROR
    = Filtered_INS2_Property<BaseINS>::Q_SIZE;

template <class BaseINS>
const unsigned Filtered_INS2_Property<INS_ClockErrorEstimated<BaseINS> >::P_SIZE
    = P_SIZE_WITHOUT_CLOCK_ERROR + INS_ClockErrorEstimated<BaseINS>::STATE_VALUES_CLOCK_ERROR;

template <class BaseINS>
const unsigned Filtered_INS2_Property<INS_ClockErrorEstimated<BaseINS> >::Q_SIZE
    = Q_SIZE_WITHOUT_CLOCK_ERROR + INS_ClockErrorEstimated<BaseINS>::STATE_VALUES_CLOCK_ERROR;
#endif

#endif /* __INS_GPS2_TIGHTLY_H__ */
