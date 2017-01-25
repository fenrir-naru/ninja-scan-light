/*
 *  INS_GPS_BE.h, header file to perform calculation of integration of INS and GPS with bias drift estimation.
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

#ifndef __INS_GPS_BE_H__
#define __INS_GPS_BE_H__

#include "algorithm/kalman.h"
#include "Filtered_INS_BE.h"
#include "INS_GPS2.h"

template <
  class FloatT, 
  template <class> class Filter = KalmanFilterUD,
  typename BaseFINS = Filtered_INS_BiasEstimated<FloatT, Filter>
>
class INS_GPS2_BiasEstimated
  : public INS_GPS2<FloatT, Filter, BaseFINS>{
  public:
    INS_GPS2_BiasEstimated() 
        : INS_GPS2<FloatT, Filter, BaseFINS>(){}
    
    /**
     * コピーコンストラクタ
     * 
     * @param orig コピー元
     * @param deepcopy ディープコピーを作成するかどうか
     */
    INS_GPS2_BiasEstimated(const INS_GPS2_BiasEstimated &orig, const bool deepcopy = false)
        : INS_GPS2<FloatT, Filter, BaseFINS>(orig, deepcopy){
      
    }
    
    ~INS_GPS2_BiasEstimated(){}
};

#endif /* __INS_GPS_BE_H__ */
