/*
 *  INS_EGM.h, header file to perform calculation of inertial navigation system
 *  with Earth gravity model instead of the default WGS84 simplified gravity model.
 *  Copyright (C) 2017 M.Naruoka (fenrir)
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

#ifndef __INS_EGM_H__
#define __INS_EGM_H__

/** @file
 * @brief Inertial navigation system with Earth gravity model
 *
 */

#include "INS.h"
#include "EGM.h"

/**
 * @brief INS including EGM
 *
 * @param FloatT precision, default is double
 * @param EGM
 */
template <
    class FloatT = double,
    class EGM = EGM2008_70_Generic<FloatT> >
class INS_EGM : public INS<FloatT> {
  public:
    typedef INS<FloatT> super_t;
#if defined(__GNUC__) && (__GNUC__ < 5)
    typedef typename super_t::vec3_t vec3_t;
#else
    using typename super_t::vec3_t;
#endif
  public:
    /**
     * Constructor
     *
     */
    INS_EGM() : super_t() {}

    /**
     * Copy constructor
     *
     * @param orig source
     * @param deepcopy if true, perform deep copy
     */
    INS_EGM(const INS_EGM &orig, const bool &deepcopy = false)
        : super_t(orig, deepcopy){

    }

    /**
     * Destructor
     *
     */
    virtual ~INS_EGM(){}

    /**
     * Return the total gravity vector in accordance to current position.
     * The total gravity is derivative of the Earth's total potential,
     * which includes both the gravitational potential and the potential due to the Earths rotation.
     *
     * @return (vec3_t) total gravity in the navigation frame
     */
    virtual vec3_t gravity_total() const {
      vec3_t res(0, 0, super_t::Earth::gravity(super_t::phi, super_t::h));
      //EGM::gravity_r();
      //res -= super_t::gravity_rotational();

      return res;
    }
};

#endif /* __INS_EGM_H__ */
