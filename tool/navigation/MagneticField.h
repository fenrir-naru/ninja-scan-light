/*
 * Copyright (c) 2015, M.Naruoka (fenrir)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * - Neither the name of the naruoka.org nor the names of its contributors
 *   may be used to endorse or promote products derived from this software
 *   without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __MAGNETIC_FIELD_H__
#define __MAGNETIC_FIELD_H__

#define _USE_MATH_DEFINES
#include <cmath>

#include "WGS84.h"

template <class FloatT>
class MagneticFieldGeneric {
  public:
    struct model_t {
      const char *name;
      FloatT year;
      int dof;
      FloatT coef[256];
    };
    
    static void model_inter_extra_polation(
        model_t &model_new,
        const model_t &model_early, const model_t &model_late){
      FloatT factor((model_new.year - model_early.year)
          / (model_late.year - model_early.year));
      
      model_new.name = "(generated)";
      int coef_common;
      
      if(model_early.dof >= model_late.dof){
        coef_common =  model_late.dof * (model_late.dof + 2);
        model_new.dof = model_early.dof;
        
        for(int i(coef_common); i < model_early.dof * (model_early.dof + 2); i--){
          model_new.coef[i] = model_early.coef[i] * (1.0 - factor);
        }
      }else{
        coef_common =  model_early.dof * (model_early.dof + 2);
        model_new.dof = model_late.dof;
        
        for(int i(coef_common); i < model_late.dof * (model_late.dof + 2); i--){
          model_new.coef[i] = model_late.coef[i] * factor;
        }
      }
      for(int i(0); i < coef_common; i++){
        model_new.coef[i] = model_early.coef[i] 
            + factor * (model_late.coef[i] - model_early.coef[i]);
      }
    }
    
    static void make_model(
        model_t &model_new, const model_t *models[], const unsigned int &models_size){
      if(models_size < 1){
        return;
      }else if(models_size == 1){
        model_new = *models[0];
        return;
      }
      const model_t *a(models[0]), *b(models[1]);
      FloatT a_dist(std::abs(model_new.year - a->year)), b_dist(std::abs(model_new.year - b->year));
      for(unsigned int i(2); i < models_size; ++i){
        FloatT dist(std::abs(model_new.year - models[i]->year));
        if(dist < a_dist){
          if(a_dist < b_dist){
            b = models[i];
            b_dist = dist;
          }else{
            a = models[i];
            a_dist = dist;
          }
        }else if(dist < b_dist){
          b = models[i];
          b_dist = dist;
        }
      }
      model_inter_extra_polation(model_new, *a, *b);
    }

    struct field_components_res_t {FloatT north, east, down;};
    static field_components_res_t field_components(
        const model_t &model,
        const FloatT &latitude_rad, const FloatT &longitude_rad,
        const FloatT &height_meter_wgs84){
      using std::cos;
      using std::sin;
      using std::pow;
      using std::sqrt;
      field_components_res_t res;
      
      FloatT slat(sin(latitude_rad));
      FloatT clat(cos(latitude_rad));
      // 緯度の補正
      {
        FloatT latitude_deg(latitude_rad / M_PI * 180);
        if((90.0 - latitude_deg) < 1E-3){
          clat = cos((90.0 - 1E-3) / 180 * M_PI); // 300 ft. from North pole
        }else if((90.0 + latitude_deg) < 1E-3){
          clat = cos((-90.0 + 1E-3) / 180 * M_PI); // 300 ft. from South pole
        }
      }
      
      FloatT sl[13] = {sin(longitude_rad)};
      FloatT cl[13] = {cos(longitude_rad)};
      
      res.north = res.east = res.down = 0;
      
      FloatT sd(0), cd(1);
      FloatT aa, bb, cc, dd, r, rr;
      
      // TODO: WGS84高度から地球中心距離への変換(だと思う)
      {
        static const FloatT a2(40680631.59);            /* WGS84 */
        static const FloatT b2(40408299.98);            /* WGS84 */
        aa = a2 * clat * clat;
        bb = b2 * slat * slat;
        cc = aa + bb;
        dd = sqrt(cc);
        FloatT height_km_wgs84(1E-3 * height_meter_wgs84);
        r = sqrt(height_km_wgs84 * (height_km_wgs84 + 2.0 * dd) + (a2 * aa + b2 * bb) / cc);
        cd = (height_km_wgs84 + dd) / r;
        sd = (a2 - b2) / dd * slat * clat / r;
        slat = slat * cd - clat * sd;
        clat = clat * cd + slat * sd;
      }
      
      aa = sqrt(3.0);
      FloatT p[118] = {
          2.0 * slat,
          2.0 * clat,
          4.5 * slat * slat - 1.5,
          3.0 * aa * clat * slat};
      FloatT q[118] = {
          -clat,
          slat,
          -3.0 * clat * slat,
          aa * (slat * slat - clat * clat)};
      
      for(int k(0), l(0), m(0), n(-1); k < ((model.dof * (model.dof + 3)) / 2); k++, m++){
        if(m > n){
          m = -1;
          n++;
          static const FloatT earths_radius(6371.2);
          rr = pow(earths_radius / r, n + 3);
        }
        FloatT fm(m + 1), fn(n + 1);
        if (k > 3){
          if (m == n){
            aa = sqrt(1.0 - 0.5 / fm);
            int j(k - n - 2);
            p[k] = (1.0 + 1.0 / fm) * aa * clat * p[j];
            q[k] = aa * (clat * q[j] + slat/fm * p[j]);
            sl[m] = sl[m-1] * cl[0] + cl[m-1] * sl[0];
            cl[m] = cl[m-1] * cl[0] - sl[m-1] * sl[0];
          }else{
            aa = sqrt(fn * fn - fm * fm);
            bb = sqrt(((fn - 1.0) * (fn - 1.0)) - (fm * fm)) / aa;
            cc = (2.0 * fn - 1.0) / aa;
            int i(k - n - 1), j(k - 2 * n - 1);
            p[k] = (fn + 1.0) * (cc * slat / fn * p[i] - bb / (fn - 1.0) * p[j]);
            q[k] = cc * (slat * q[i] - clat / fn * p[i]) - bb * q[j];
          }
        }
        aa = rr * model.coef[l];
        
        if(m < 0){
          res.north += aa * q[k];
          res.down -= aa * p[k];
          l++;
        }else{
          bb = rr * model.coef[l + 1];
          cc = aa * cl[m] + bb * sl[m];
          res.north += cc * q[k];
          res.down -= cc * p[k];
          if (clat > 0){
            res.east += (aa * sl[m] - bb * cl[m]) * fm * p[k]/((fn + 1.0) * clat);
          }else{
            res.east += (aa * sl[m] - bb * cl[m]) * q[k] * slat;
          }
          l += 2;
        }
      }
  
      aa = res.north;
      res.north = aa * cd + res.down * sd;
      res.down = res.down * cd - aa * sd;
      
      return res;
    }

    struct latlng_t {
      FloatT latitude, longitude;
    };
    static latlng_t geomagnetic_latlng(
        const model_t &model,
        const FloatT &geocentric_latitude, const FloatT &longitude){
      const FloatT &g10(model.coef[0]), &g11(model.coef[1]), &h11(model.coef[2]);
      FloatT m[] = {g11, h11, g10};
      FloatT b0(-std::sqrt(std::pow(m[0], 2) + std::pow(m[1], 2) + std::pow(m[2], 2)));
      FloatT z_cd[] = {m[0] / b0, m[1] / b0, m[2] / b0};
      FloatT y_cd_denom(-std::sqrt((h11 * h11) + (g11 * g11)));
      FloatT y_cd[] = {-h11 / y_cd_denom, g11 / y_cd_denom, 0}; /* = [0,0,1] * z_cd */
      FloatT x_cd[] = { /* y_cd * z_cd */
        y_cd[1] * z_cd[2] - y_cd[2] * z_cd[1],
        y_cd[2] * z_cd[0] - y_cd[0] * z_cd[2],
        y_cd[0] * z_cd[1] - y_cd[1] * z_cd[0],
      };
      FloatT
          clat(std::cos(geocentric_latitude)), slat(std::sin(geocentric_latitude)),
          clng(std::cos(longitude)), slng(std::sin(longitude));
      FloatT vec_geoc[] = {clat * clng, clat * slng, slat};
      FloatT vec_geom[] = {
        x_cd[0] * vec_geoc[0] + x_cd[1] * vec_geoc[1] + x_cd[2] * vec_geoc[2],
        y_cd[0] * vec_geoc[0] + y_cd[1] * vec_geoc[1] + y_cd[2] * vec_geoc[2],
        z_cd[0] * vec_geoc[0] + z_cd[1] * vec_geoc[1] + z_cd[2] * vec_geoc[2],
      };
      latlng_t res = {
        std::asin(vec_geom[2]),
        std::atan2(vec_geom[1], vec_geom[0])
      };
      return res;
    }
};

typedef MagneticFieldGeneric<double> MagneticField;

/* IGRF11 is the eleventh generation standard main field model adopted
 * by the International Association of Geomagnetism and Aeronomy (IAGA).
 * This is a degree and order 10 model from 1900 to 1995 and a degree and
 * order 13 model from 2000 to 2015, providing estimates of the main field
 * for dates between January 1, 1900 and January 1, 2015. For more information
 * on the IGRF and IAGA, visit the IAGA Working Group V-MOD Web site at:
 *         http://www.ngdc.noaa.gov/IAGA/vmod/
 */

template <class FloatT>
class IGRF11Generic : public MagneticFieldGeneric<FloatT> {
  public:
    static const typename MagneticFieldGeneric<FloatT>::model_t DGRF2000;
    static const typename MagneticFieldGeneric<FloatT>::model_t DGRF2005;
    static const typename MagneticFieldGeneric<FloatT>::model_t DGRF45;
    static const typename MagneticFieldGeneric<FloatT>::model_t DGRF50;
    static const typename MagneticFieldGeneric<FloatT>::model_t DGRF55;
    static const typename MagneticFieldGeneric<FloatT>::model_t DGRF60;
    static const typename MagneticFieldGeneric<FloatT>::model_t DGRF65;
    static const typename MagneticFieldGeneric<FloatT>::model_t DGRF70;
    static const typename MagneticFieldGeneric<FloatT>::model_t DGRF75;
    static const typename MagneticFieldGeneric<FloatT>::model_t DGRF80;
    static const typename MagneticFieldGeneric<FloatT>::model_t DGRF85;
    static const typename MagneticFieldGeneric<FloatT>::model_t DGRF90;
    static const typename MagneticFieldGeneric<FloatT>::model_t DGRF95;
    static const typename MagneticFieldGeneric<FloatT>::model_t IGRF00;
    static const typename MagneticFieldGeneric<FloatT>::model_t IGRF05;
    static const typename MagneticFieldGeneric<FloatT>::model_t IGRF10;
    static const typename MagneticFieldGeneric<FloatT>::model_t IGRF15;
    static const typename MagneticFieldGeneric<FloatT>::model_t IGRF20;
    static const typename MagneticFieldGeneric<FloatT>::model_t IGRF2010;
    static const typename MagneticFieldGeneric<FloatT>::model_t IGRF25;
    static const typename MagneticFieldGeneric<FloatT>::model_t IGRF30;
    static const typename MagneticFieldGeneric<FloatT>::model_t IGRF35;
    static const typename MagneticFieldGeneric<FloatT>::model_t IGRF40;

    static typename MagneticFieldGeneric<FloatT>::model_t get_model(const FloatT &year){
      static const typename MagneticFieldGeneric<FloatT>::model_t *models[] = {
        &IGRF00,
        &IGRF05,
        &IGRF10,
        &IGRF15,
        &IGRF20,
        &IGRF25,
        &IGRF30,
        &IGRF35,
        &IGRF40,
        &DGRF45,
        &DGRF50,
        &DGRF55,
        &DGRF60,
        &DGRF65,
        &DGRF70,
        &DGRF75,
        &DGRF80,
        &DGRF85,
        &DGRF90,
        &DGRF95,
        &DGRF2000,
        &DGRF2005,
        &IGRF2010,
      };
      typename MagneticFieldGeneric<FloatT>::model_t res;
      res.year = year;
      MagneticFieldGeneric<FloatT>::make_model(res, models, sizeof(models) / sizeof(models[0]));
      return res;
    }
};

typedef IGRF11Generic<double> IGRF11;

template <class FloatT>
const typename MagneticFieldGeneric<FloatT>::model_t IGRF11Generic<FloatT>::DGRF2000 = {
    "DGRF2000",
    2000.0,
    13,
    {-29619.4, -1728.2, 5186.1, // 1
        -2267.7, 3068.4, -2481.6, 1670.9, -458.0, // 2
        1339.6, -2288.0, -227.6, 1252.1, 293.4, 714.5, -491.1, // 3
        932.3, 786.8, 272.6, 250.0, -231.9, -403.0, 119.8, 111.3, -303.8, // 4
        -218.8, 351.4, 43.8, 222.3, 171.9, -130.4, -133.1, -168.6, -39.3, -12.9, 106.3, // 5
        72.3, 68.2, -17.4, 74.2, 63.7, -160.9, 65.1, -5.9, -61.2, 16.9, 0.7, -90.4, 43.8, // 6
        79.0, -74.0, -64.6, 0.0, -24.2, 33.3, 6.2, 9.1, 24.0, 6.9, 14.8, 7.3, -25.4, -1.2, -5.8, // 7
        24.4, 6.6, 11.9, -9.2, -21.5, -7.9, 8.5, -16.6, -21.5, 9.1, 15.5, 7.0, 8.9, -7.9, -14.9, -7.0, -2.1, // 8
        5.0, 9.4, -19.7, 3.0, 13.4, -8.4, 12.5, 6.3, -6.2, -8.9, -8.4, -1.5, 8.4, 9.3, 3.8, -4.3, -8.2, -8.2, 4.8, // 9
        -2.6, -6.0, 1.7, 1.7, 0.0, -3.1, 4.0, -0.5, 4.9, 3.7, -5.9, 1.0, -1.2, 2.0, -2.9, 4.2, 0.2, 0.3, -2.2, -1.1, -7.4, // 10
        2.7, -1.7, 0.1, -1.9, 1.3, 1.5, -0.9, -0.1, -2.6, 0.1, 0.9, -0.7, -0.7, 0.7, -2.8, 1.7, -0.9, 0.1, -1.2, 1.2, -1.9, 4.0, -0.9, // 11
        -2.2, -0.3, -0.4, 0.2, 0.3, 0.9, 2.5, -0.2, -2.6, 0.9, 0.7, -0.5, 0.3, 0.3, 0.0, -0.3, 0.0, -0.4, 0.3, -0.1, -0.9, -0.2, -0.4, -0.4, 0.8, // 12
        -0.2, -0.9, -0.9, 0.3, 0.2, 0.1, 1.8, -0.4, -0.4, 1.3, -1.0, -0.4, -0.1, 0.7, 0.7, -0.4, 0.3, 0.3, 0.6, -0.1, 0.3, 0.4, -0.2, 0.0, -0.5, 0.1, -0.9, } // 13
    };
template <class FloatT>
const typename MagneticFieldGeneric<FloatT>::model_t IGRF11Generic<FloatT>::DGRF2005 = {
    "DGRF2005",
    2005.0,
    13,
    {-29554.63, -1669.05, 5077.99, // 1
        -2337.24, 3047.69, -2594.5, 1657.76, -515.43, // 2
        1336.3, -2305.83, -198.86, 1246.39, 269.72, 672.51, -524.72, // 3
        920.55, 797.96, 282.07, 210.65, -225.23, -379.86, 145.15, 100.0, -305.36, // 4
        -227.0, 354.41, 42.72, 208.95, 180.25, -136.54, -123.45, -168.05, -19.57, -13.55, 103.85, // 5
        73.6, 69.56, -20.33, 76.74, 54.75, -151.34, 63.63, -14.58, -63.53, 14.58, 0.24, -86.36, 50.94, // 6
        79.88, -74.46, -61.14, -1.65, -22.57, 38.73, 6.82, 12.3, 25.35, 9.37, 10.93, 5.42, -26.32, 1.94, -4.64, // 7
        24.8, 7.62, 11.2, -11.73, -20.88, -6.88, 9.83, -18.11, -19.71, 10.17, 16.22, 9.36, 7.61, -11.25, -12.76, -4.87, -0.06, // 8
        5.58, 9.76, -20.11, 3.58, 12.69, -6.94, 12.67, 5.01, -6.72, -10.76, -8.16, -1.25, 8.1, 8.76, 2.92, -6.66, -7.73, -9.22, 6.01, // 9
        -2.17, -6.12, 2.19, 1.42, 0.1, -2.35, 4.46, -0.15, 4.76, 3.06, -6.58, 0.29, -1.01, 2.06, -3.47, 3.77, -0.86, -0.21, -2.31, -2.09, -7.93, // 10
        2.95, -1.6, 0.26, -1.88, 1.44, 1.44, -0.77, -0.31, -2.27, 0.29, 0.9, -0.79, -0.58, 0.53, -2.69, 1.8, -1.08, 0.16, -1.58, 0.96, -1.9, 3.99, -1.39, // 11
        -2.15, -0.29, -0.55, 0.21, 0.23, 0.89, 2.38, -0.38, -2.63, 0.96, 0.61, -0.3, 0.4, 0.46, 0.01, -0.35, 0.02, -0.36, 0.28, 0.08, -0.87, -0.49, -0.34, -0.08, 0.88, // 12
        -0.16, -0.88, -0.76, 0.3, 0.33, 0.28, 1.72, -0.43, -0.54, 1.18, -1.07, -0.37, -0.04, 0.75, 0.63, -0.26, 0.21, 0.35, 0.53, -0.05, 0.38, 0.41, -0.22, -0.1, -0.57, -0.18, -0.82, } // 13
    };
template <class FloatT>
const typename MagneticFieldGeneric<FloatT>::model_t IGRF11Generic<FloatT>::DGRF45 = {
    "DGRF45",
    1945.0,
    10,
    {-30594.0, -2285.0, 5810.0, // 1
        -1244.0, 2990.0, -1702.0, 1578.0, 477.0, // 2
        1282.0, -1834.0, -499.0, 1255.0, 186.0, 913.0, -11.0, // 3
        944.0, 776.0, 144.0, 544.0, -276.0, -421.0, -55.0, 304.0, -178.0, // 4
        -253.0, 346.0, -12.0, 194.0, 95.0, -20.0, -67.0, -142.0, -119.0, -82.0, 82.0, // 5
        59.0, 57.0, 6.0, 6.0, 100.0, -246.0, 16.0, -25.0, -9.0, 21.0, -16.0, -104.0, -39.0, // 6
        70.0, -40.0, -45.0, 0.0, -18.0, 0.0, 2.0, -29.0, 6.0, -10.0, 28.0, 15.0, -17.0, 29.0, -22.0, // 7
        13.0, 7.0, 12.0, -8.0, -21.0, -5.0, -12.0, 9.0, -7.0, 7.0, 2.0, -10.0, 18.0, 7.0, 3.0, 2.0, -11.0, // 8
        5.0, -21.0, -27.0, 1.0, 17.0, -11.0, 29.0, 3.0, -9.0, 16.0, 4.0, -3.0, 9.0, -4.0, 6.0, -3.0, 1.0, -4.0, 8.0, // 9
        -3.0, 11.0, 5.0, 1.0, 1.0, 2.0, -20.0, -5.0, -1.0, -1.0, -6.0, 8.0, 6.0, -1.0, -4.0, -3.0, -2.0, 5.0, 0.0, -2.0, -2.0, } // 10
    };
template <class FloatT>
const typename MagneticFieldGeneric<FloatT>::model_t IGRF11Generic<FloatT>::DGRF50 = {
    "DGRF50",
    1950.0,
    10,
    {-30554.0, -2250.0, 5815.0, // 1
        -1341.0, 2998.0, -1810.0, 1576.0, 381.0, // 2
        1297.0, -1889.0, -476.0, 1274.0, 206.0, 896.0, -46.0, // 3
        954.0, 792.0, 136.0, 528.0, -278.0, -408.0, -37.0, 303.0, -210.0, // 4
        -240.0, 349.0, 3.0, 211.0, 103.0, -20.0, -87.0, -147.0, -122.0, -76.0, 80.0, // 5
        54.0, 57.0, -1.0, 4.0, 99.0, -247.0, 33.0, -16.0, -12.0, 12.0, -12.0, -105.0, -30.0, // 6
        65.0, -55.0, -35.0, 2.0, -17.0, 1.0, 0.0, -40.0, 10.0, -7.0, 36.0, 5.0, -18.0, 19.0, -16.0, // 7
        22.0, 15.0, 5.0, -4.0, -22.0, -1.0, 0.0, 11.0, -21.0, 15.0, -8.0, -13.0, 17.0, 5.0, -4.0, -1.0, -17.0, // 8
        3.0, -7.0, -24.0, -1.0, 19.0, -25.0, 12.0, 10.0, 2.0, 5.0, 2.0, -5.0, 8.0, -2.0, 8.0, 3.0, -11.0, 8.0, -7.0, // 9
        -8.0, 4.0, 13.0, -1.0, -2.0, 13.0, -10.0, -4.0, 2.0, 4.0, -3.0, 12.0, 6.0, 3.0, -3.0, 2.0, 6.0, 10.0, 11.0, 3.0, 8.0, } // 10
    };
template <class FloatT>
const typename MagneticFieldGeneric<FloatT>::model_t IGRF11Generic<FloatT>::DGRF55 = {
    "DGRF55",
    1955.0,
    10,
    {-30500.0, -2215.0, 5820.0, // 1
        -1440.0, 3003.0, -1898.0, 1581.0, 291.0, // 2
        1302.0, -1944.0, -462.0, 1288.0, 216.0, 882.0, -83.0, // 3
        958.0, 796.0, 133.0, 510.0, -274.0, -397.0, -23.0, 290.0, -230.0, // 4
        -229.0, 360.0, 15.0, 230.0, 110.0, -23.0, -98.0, -152.0, -121.0, -69.0, 78.0, // 5
        47.0, 57.0, -9.0, 3.0, 96.0, -247.0, 48.0, -8.0, -16.0, 7.0, -12.0, -107.0, -24.0, // 6
        65.0, -56.0, -50.0, 2.0, -24.0, 10.0, -4.0, -32.0, 8.0, -11.0, 28.0, 9.0, -20.0, 18.0, -18.0, // 7
        11.0, 9.0, 10.0, -6.0, -15.0, -14.0, 5.0, 6.0, -23.0, 10.0, 3.0, -7.0, 23.0, 6.0, -4.0, 9.0, -13.0, // 8
        4.0, 9.0, -11.0, -4.0, 12.0, -5.0, 7.0, 2.0, 6.0, 4.0, -2.0, 1.0, 10.0, 2.0, 7.0, 2.0, -6.0, 5.0, 5.0, // 9
        -3.0, -5.0, -4.0, -1.0, 0.0, 2.0, -8.0, -3.0, -2.0, 7.0, -4.0, 4.0, 1.0, -2.0, -3.0, 6.0, 7.0, -2.0, -1.0, 0.0, -3.0, } // 10
    };
template <class FloatT>
const typename MagneticFieldGeneric<FloatT>::model_t IGRF11Generic<FloatT>::DGRF60 = {
    "DGRF60",
    1960.0,
    10,
    {-30421.0, -2169.0, 5791.0, // 1
        -1555.0, 3002.0, -1967.0, 1590.0, 206.0, // 2
        1302.0, -1992.0, -414.0, 1289.0, 224.0, 878.0, -130.0, // 3
        957.0, 800.0, 135.0, 504.0, -278.0, -394.0, 3.0, 269.0, -255.0, // 4
        -222.0, 362.0, 16.0, 242.0, 125.0, -26.0, -117.0, -156.0, -114.0, -63.0, 81.0, // 5
        46.0, 58.0, -10.0, 1.0, 99.0, -237.0, 60.0, -1.0, -20.0, -2.0, -11.0, -113.0, -17.0, // 6
        67.0, -56.0, -55.0, 5.0, -28.0, 15.0, -6.0, -32.0, 7.0, -7.0, 23.0, 17.0, -18.0, 8.0, -17.0, // 7
        15.0, 6.0, 11.0, -4.0, -14.0, -11.0, 7.0, 2.0, -18.0, 10.0, 4.0, -5.0, 23.0, 10.0, 1.0, 8.0, -20.0, // 8
        4.0, 6.0, -18.0, 0.0, 12.0, -9.0, 2.0, 1.0, 0.0, 4.0, -3.0, -1.0, 9.0, -2.0, 8.0, 3.0, 0.0, -1.0, 5.0, // 9
        1.0, -3.0, 4.0, 4.0, 1.0, 0.0, 0.0, -1.0, 2.0, 4.0, -5.0, 6.0, 1.0, 1.0, -1.0, -1.0, 6.0, 2.0, 0.0, 0.0, -7.0, } // 10
    };
template <class FloatT>
const typename MagneticFieldGeneric<FloatT>::model_t IGRF11Generic<FloatT>::DGRF65 = {
    "DGRF65",
    1965.0,
    10,
    {-30334.0, -2119.0, 5776.0, // 1
        -1662.0, 2997.0, -2016.0, 1594.0, 114.0, // 2
        1297.0, -2038.0, -404.0, 1292.0, 240.0, 856.0, -165.0, // 3
        957.0, 804.0, 148.0, 479.0, -269.0, -390.0, 13.0, 252.0, -269.0, // 4
        -219.0, 358.0, 19.0, 254.0, 128.0, -31.0, -126.0, -157.0, -97.0, -62.0, 81.0, // 5
        45.0, 61.0, -11.0, 8.0, 100.0, -228.0, 68.0, 4.0, -32.0, 1.0, -8.0, -111.0, -7.0, // 6
        75.0, -57.0, -61.0, 4.0, -27.0, 13.0, -2.0, -26.0, 6.0, -6.0, 26.0, 13.0, -23.0, 1.0, -12.0, // 7
        13.0, 5.0, 7.0, -4.0, -12.0, -14.0, 9.0, 0.0, -16.0, 8.0, 4.0, -1.0, 24.0, 11.0, -3.0, 4.0, -17.0, // 8
        8.0, 10.0, -22.0, 2.0, 15.0, -13.0, 7.0, 10.0, -4.0, -1.0, -5.0, -1.0, 10.0, 5.0, 10.0, 1.0, -4.0, -2.0, 1.0, // 9
        -2.0, -3.0, 2.0, 2.0, 1.0, -5.0, 2.0, -2.0, 6.0, 4.0, -4.0, 4.0, 0.0, 0.0, -2.0, 2.0, 3.0, 2.0, 0.0, 0.0, -6.0, } // 10
    };
template <class FloatT>
const typename MagneticFieldGeneric<FloatT>::model_t IGRF11Generic<FloatT>::DGRF70 = {
    "DGRF70",
    1970.0,
    10,
    {-30220.0, -2068.0, 5737.0, // 1
        -1781.0, 3000.0, -2047.0, 1611.0, 25.0, // 2
        1287.0, -2091.0, -366.0, 1278.0, 251.0, 838.0, -196.0, // 3
        952.0, 800.0, 167.0, 461.0, -266.0, -395.0, 26.0, 234.0, -279.0, // 4
        -216.0, 359.0, 26.0, 262.0, 139.0, -42.0, -139.0, -160.0, -91.0, -56.0, 83.0, // 5
        43.0, 64.0, -12.0, 15.0, 100.0, -212.0, 72.0, 2.0, -37.0, 3.0, -6.0, -112.0, 1.0, // 6
        72.0, -57.0, -70.0, 1.0, -27.0, 14.0, -4.0, -22.0, 8.0, -2.0, 23.0, 13.0, -23.0, -2.0, -11.0, // 7
        14.0, 6.0, 7.0, -2.0, -15.0, -13.0, 6.0, -3.0, -17.0, 5.0, 6.0, 0.0, 21.0, 11.0, -6.0, 3.0, -16.0, // 8
        8.0, 10.0, -21.0, 2.0, 16.0, -12.0, 6.0, 10.0, -4.0, -1.0, -5.0, 0.0, 10.0, 3.0, 11.0, 1.0, -2.0, -1.0, 1.0, // 9
        -3.0, -3.0, 1.0, 2.0, 1.0, -5.0, 3.0, -1.0, 4.0, 6.0, -4.0, 4.0, 0.0, 1.0, -1.0, 0.0, 3.0, 3.0, 1.0, -1.0, -4.0, } // 10
    };
template <class FloatT>
const typename MagneticFieldGeneric<FloatT>::model_t IGRF11Generic<FloatT>::DGRF75 = {
    "DGRF75",
    1975.0,
    10,
    {-30100.0, -2013.0, 5675.0, // 1
        -1902.0, 3010.0, -2067.0, 1632.0, -68.0, // 2
        1276.0, -2144.0, -333.0, 1260.0, 262.0, 830.0, -223.0, // 3
        946.0, 791.0, 191.0, 438.0, -265.0, -405.0, 39.0, 216.0, -288.0, // 4
        -218.0, 356.0, 31.0, 264.0, 148.0, -59.0, -152.0, -159.0, -83.0, -49.0, 88.0, // 5
        45.0, 66.0, -13.0, 28.0, 99.0, -198.0, 75.0, 1.0, -41.0, 6.0, -4.0, -111.0, 11.0, // 6
        71.0, -56.0, -77.0, 1.0, -26.0, 16.0, -5.0, -14.0, 10.0, 0.0, 22.0, 12.0, -23.0, -5.0, -12.0, // 7
        14.0, 6.0, 6.0, -1.0, -16.0, -12.0, 4.0, -8.0, -19.0, 4.0, 6.0, 0.0, 18.0, 10.0, -10.0, 1.0, -17.0, // 8
        7.0, 10.0, -21.0, 2.0, 16.0, -12.0, 7.0, 10.0, -4.0, -1.0, -5.0, -1.0, 10.0, 4.0, 11.0, 1.0, -3.0, -2.0, 1.0, // 9
        -3.0, -3.0, 1.0, 2.0, 1.0, -5.0, 3.0, -2.0, 4.0, 5.0, -4.0, 4.0, -1.0, 1.0, -1.0, 0.0, 3.0, 3.0, 1.0, -1.0, -5.0, } // 10
    };
template <class FloatT>
const typename MagneticFieldGeneric<FloatT>::model_t IGRF11Generic<FloatT>::DGRF80 = {
    "DGRF80",
    1980.0,
    10,
    {-29992.0, -1956.0, 5604.0, // 1
        -1997.0, 3027.0, -2129.0, 1663.0, -200.0, // 2
        1281.0, -2180.0, -336.0, 1251.0, 271.0, 833.0, -252.0, // 3
        938.0, 782.0, 212.0, 398.0, -257.0, -419.0, 53.0, 199.0, -297.0, // 4
        -218.0, 357.0, 46.0, 261.0, 150.0, -74.0, -151.0, -162.0, -78.0, -48.0, 92.0, // 5
        48.0, 66.0, -15.0, 42.0, 93.0, -192.0, 71.0, 4.0, -43.0, 14.0, -2.0, -108.0, 17.0, // 6
        72.0, -59.0, -82.0, 2.0, -27.0, 21.0, -5.0, -12.0, 16.0, 1.0, 18.0, 11.0, -23.0, -2.0, -10.0, // 7
        18.0, 6.0, 7.0, 0.0, -18.0, -11.0, 4.0, -7.0, -22.0, 4.0, 9.0, 3.0, 16.0, 6.0, -13.0, -1.0, -15.0, // 8
        5.0, 10.0, -21.0, 1.0, 16.0, -12.0, 9.0, 9.0, -5.0, -3.0, -6.0, -1.0, 9.0, 7.0, 10.0, 2.0, -6.0, -5.0, 2.0, // 9
        -4.0, -4.0, 1.0, 2.0, 0.0, -5.0, 3.0, -2.0, 6.0, 5.0, -4.0, 3.0, 0.0, 1.0, -1.0, 2.0, 4.0, 3.0, 0.0, 0.0, -6.0, } // 10
    };
template <class FloatT>
const typename MagneticFieldGeneric<FloatT>::model_t IGRF11Generic<FloatT>::DGRF85 = {
    "DGRF85",
    1985.0,
    10,
    {-29873.0, -1905.0, 5500.0, // 1
        -2072.0, 3044.0, -2197.0, 1687.0, -306.0, // 2
        1296.0, -2208.0, -310.0, 1247.0, 284.0, 829.0, -297.0, // 3
        936.0, 780.0, 232.0, 361.0, -249.0, -424.0, 69.0, 170.0, -297.0, // 4
        -214.0, 355.0, 47.0, 253.0, 150.0, -93.0, -154.0, -164.0, -75.0, -46.0, 95.0, // 5
        53.0, 65.0, -16.0, 51.0, 88.0, -185.0, 69.0, 4.0, -48.0, 16.0, -1.0, -102.0, 21.0, // 6
        74.0, -62.0, -83.0, 3.0, -27.0, 24.0, -2.0, -6.0, 20.0, 4.0, 17.0, 10.0, -23.0, 0.0, -7.0, // 7
        21.0, 6.0, 8.0, 0.0, -19.0, -11.0, 5.0, -9.0, -23.0, 4.0, 11.0, 4.0, 14.0, 4.0, -15.0, -4.0, -11.0, // 8
        5.0, 10.0, -21.0, 1.0, 15.0, -12.0, 9.0, 9.0, -6.0, -3.0, -6.0, -1.0, 9.0, 7.0, 9.0, 1.0, -7.0, -5.0, 2.0, // 9
        -4.0, -4.0, 1.0, 3.0, 0.0, -5.0, 3.0, -2.0, 6.0, 5.0, -4.0, 3.0, 0.0, 1.0, -1.0, 2.0, 4.0, 3.0, 0.0, 0.0, -6.0, } // 10
    };
template <class FloatT>
const typename MagneticFieldGeneric<FloatT>::model_t IGRF11Generic<FloatT>::DGRF90 = {
    "DGRF90",
    1990.0,
    10,
    {-29775.0, -1848.0, 5406.0, // 1
        -2131.0, 3059.0, -2279.0, 1686.0, -373.0, // 2
        1314.0, -2239.0, -284.0, 1248.0, 293.0, 802.0, -352.0, // 3
        939.0, 780.0, 247.0, 325.0, -240.0, -423.0, 84.0, 141.0, -299.0, // 4
        -214.0, 353.0, 46.0, 245.0, 154.0, -109.0, -153.0, -165.0, -69.0, -36.0, 97.0, // 5
        61.0, 65.0, -16.0, 59.0, 82.0, -178.0, 69.0, 3.0, -52.0, 18.0, 1.0, -96.0, 24.0, // 6
        77.0, -64.0, -80.0, 2.0, -26.0, 26.0, 0.0, -1.0, 21.0, 5.0, 17.0, 9.0, -23.0, 0.0, -4.0, // 7
        23.0, 5.0, 10.0, -1.0, -19.0, -10.0, 6.0, -12.0, -22.0, 3.0, 12.0, 4.0, 12.0, 2.0, -16.0, -6.0, -10.0, // 8
        4.0, 9.0, -20.0, 1.0, 15.0, -12.0, 11.0, 9.0, -7.0, -4.0, -7.0, -2.0, 9.0, 7.0, 8.0, 1.0, -7.0, -6.0, 2.0, // 9
        -3.0, -4.0, 2.0, 2.0, 1.0, -5.0, 3.0, -2.0, 6.0, 4.0, -4.0, 3.0, 0.0, 1.0, -2.0, 3.0, 3.0, 3.0, -1.0, 0.0, -6.0, } // 10
    };
template <class FloatT>
const typename MagneticFieldGeneric<FloatT>::model_t IGRF11Generic<FloatT>::DGRF95 = {
    "DGRF95",
    1995.0,
    10,
    {-29692.0, -1784.0, 5306.0, // 1
        -2200.0, 3070.0, -2366.0, 1681.0, -413.0, // 2
        1335.0, -2267.0, -262.0, 1249.0, 302.0, 759.0, -427.0, // 3
        940.0, 780.0, 262.0, 290.0, -236.0, -418.0, 97.0, 122.0, -306.0, // 4
        -214.0, 352.0, 46.0, 235.0, 165.0, -118.0, -143.0, -166.0, -55.0, -17.0, 107.0, // 5
        68.0, 67.0, -17.0, 68.0, 72.0, -170.0, 67.0, -1.0, -58.0, 19.0, 1.0, -93.0, 36.0, // 6
        77.0, -72.0, -69.0, 1.0, -25.0, 28.0, 4.0, 5.0, 24.0, 4.0, 17.0, 8.0, -24.0, -2.0, -6.0, // 7
        25.0, 6.0, 11.0, -6.0, -21.0, -9.0, 8.0, -14.0, -23.0, 9.0, 15.0, 6.0, 11.0, -5.0, -16.0, -7.0, -4.0, // 8
        4.0, 9.0, -20.0, 3.0, 15.0, -10.0, 12.0, 8.0, -6.0, -8.0, -8.0, -1.0, 8.0, 10.0, 5.0, -2.0, -8.0, -8.0, 3.0, // 9
        -3.0, -6.0, 1.0, 2.0, 0.0, -4.0, 4.0, -1.0, 5.0, 4.0, -5.0, 2.0, -1.0, 2.0, -2.0, 5.0, 1.0, 1.0, -2.0, 0.0, -7.0, } // 10
    };
template <class FloatT>
const typename MagneticFieldGeneric<FloatT>::model_t IGRF11Generic<FloatT>::IGRF00 = {
    "IGRF00",
    1900.0,
    10,
    {-31543.0, -2298.0, 5922.0, // 1
        -677.0, 2905.0, -1061.0, 924.0, 1121.0, // 2
        1022.0, -1469.0, -330.0, 1256.0, 3.0, 572.0, 523.0, // 3
        876.0, 628.0, 195.0, 660.0, -69.0, -361.0, -210.0, 134.0, -75.0, // 4
        -184.0, 328.0, -210.0, 264.0, 53.0, 5.0, -33.0, -86.0, -124.0, -16.0, 3.0, // 5
        63.0, 61.0, -9.0, -11.0, 83.0, -217.0, 2.0, -58.0, -35.0, 59.0, 36.0, -90.0, -69.0, // 6
        70.0, -55.0, -45.0, 0.0, -13.0, 34.0, -10.0, -41.0, -1.0, -21.0, 28.0, 18.0, -12.0, 6.0, -22.0, // 7
        11.0, 8.0, 8.0, -4.0, -14.0, -9.0, 7.0, 1.0, -13.0, 2.0, 5.0, -9.0, 16.0, 5.0, -5.0, 8.0, -18.0, // 8
        8.0, 10.0, -20.0, 1.0, 14.0, -11.0, 5.0, 12.0, -3.0, 1.0, -2.0, -2.0, 8.0, 2.0, 10.0, -1.0, -2.0, -1.0, 2.0, // 9
        -3.0, -4.0, 2.0, 2.0, 1.0, -5.0, 2.0, -2.0, 6.0, 6.0, -4.0, 4.0, 0.0, 0.0, -2.0, 2.0, 4.0, 2.0, 0.0, 0.0, -6.0, } // 10
    };
template <class FloatT>
const typename MagneticFieldGeneric<FloatT>::model_t IGRF11Generic<FloatT>::IGRF05 = {
    "IGRF05",
    1905.0,
    10,
    {-31464.0, -2298.0, 5909.0, // 1
        -728.0, 2928.0, -1086.0, 1041.0, 1065.0, // 2
        1037.0, -1494.0, -357.0, 1239.0, 34.0, 635.0, 480.0, // 3
        880.0, 643.0, 203.0, 653.0, -77.0, -380.0, -201.0, 146.0, -65.0, // 4
        -192.0, 328.0, -193.0, 259.0, 56.0, -1.0, -32.0, -93.0, -125.0, -26.0, 11.0, // 5
        62.0, 60.0, -7.0, -11.0, 86.0, -221.0, 4.0, -57.0, -32.0, 57.0, 32.0, -92.0, -67.0, // 6
        70.0, -54.0, -46.0, 0.0, -14.0, 33.0, -11.0, -41.0, 0.0, -20.0, 28.0, 18.0, -12.0, 6.0, -22.0, // 7
        11.0, 8.0, 8.0, -4.0, -15.0, -9.0, 7.0, 1.0, -13.0, 2.0, 5.0, -8.0, 16.0, 5.0, -5.0, 8.0, -18.0, // 8
        8.0, 10.0, -20.0, 1.0, 14.0, -11.0, 5.0, 12.0, -3.0, 1.0, -2.0, -2.0, 8.0, 2.0, 10.0, 0.0, -2.0, -1.0, 2.0, // 9
        -3.0, -4.0, 2.0, 2.0, 1.0, -5.0, 2.0, -2.0, 6.0, 6.0, -4.0, 4.0, 0.0, 0.0, -2.0, 2.0, 4.0, 2.0, 0.0, 0.0, -6.0, } // 10
    };
template <class FloatT>
const typename MagneticFieldGeneric<FloatT>::model_t IGRF11Generic<FloatT>::IGRF10 = {
    "IGRF10",
    1910.0,
    10,
    {-31354.0, -2297.0, 5898.0, // 1
        -769.0, 2948.0, -1128.0, 1176.0, 1000.0, // 2
        1058.0, -1524.0, -389.0, 1223.0, 62.0, 705.0, 425.0, // 3
        884.0, 660.0, 211.0, 644.0, -90.0, -400.0, -189.0, 160.0, -55.0, // 4
        -201.0, 327.0, -172.0, 253.0, 57.0, -9.0, -33.0, -102.0, -126.0, -38.0, 21.0, // 5
        62.0, 58.0, -5.0, -11.0, 89.0, -224.0, 5.0, -54.0, -29.0, 54.0, 28.0, -95.0, -65.0, // 6
        71.0, -54.0, -47.0, 1.0, -14.0, 32.0, -12.0, -40.0, 1.0, -19.0, 28.0, 18.0, -13.0, 6.0, -22.0, // 7
        11.0, 8.0, 8.0, -4.0, -15.0, -9.0, 6.0, 1.0, -13.0, 2.0, 5.0, -8.0, 16.0, 5.0, -5.0, 8.0, -18.0, // 8
        8.0, 10.0, -20.0, 1.0, 14.0, -11.0, 5.0, 12.0, -3.0, 1.0, -2.0, -2.0, 8.0, 2.0, 10.0, 0.0, -2.0, -1.0, 2.0, // 9
        -3.0, -4.0, 2.0, 2.0, 1.0, -5.0, 2.0, -2.0, 6.0, 6.0, -4.0, 4.0, 0.0, 0.0, -2.0, 2.0, 4.0, 2.0, 0.0, 0.0, -6.0, } // 10
    };
template <class FloatT>
const typename MagneticFieldGeneric<FloatT>::model_t IGRF11Generic<FloatT>::IGRF15 = {
    "IGRF15",
    1915.0,
    10,
    {-31212.0, -2306.0, 5875.0, // 1
        -802.0, 2956.0, -1191.0, 1309.0, 917.0, // 2
        1084.0, -1559.0, -421.0, 1212.0, 84.0, 778.0, 360.0, // 3
        887.0, 678.0, 218.0, 631.0, -109.0, -416.0, -173.0, 178.0, -51.0, // 4
        -211.0, 327.0, -148.0, 245.0, 58.0, -16.0, -34.0, -111.0, -126.0, -51.0, 32.0, // 5
        61.0, 57.0, -2.0, -10.0, 93.0, -228.0, 8.0, -51.0, -26.0, 49.0, 23.0, -98.0, -62.0, // 6
        72.0, -54.0, -48.0, 2.0, -14.0, 31.0, -12.0, -38.0, 2.0, -18.0, 28.0, 19.0, -15.0, 6.0, -22.0, // 7
        11.0, 8.0, 8.0, -4.0, -15.0, -9.0, 6.0, 2.0, -13.0, 3.0, 5.0, -8.0, 16.0, 6.0, -5.0, 8.0, -18.0, // 8
        8.0, 10.0, -20.0, 1.0, 14.0, -11.0, 5.0, 12.0, -3.0, 1.0, -2.0, -2.0, 8.0, 2.0, 10.0, 0.0, -2.0, -1.0, 2.0, // 9
        -3.0, -4.0, 2.0, 2.0, 1.0, -5.0, 2.0, -2.0, 6.0, 6.0, -4.0, 4.0, 0.0, 0.0, -2.0, 1.0, 4.0, 2.0, 0.0, 0.0, -6.0, } // 10
    };
template <class FloatT>
const typename MagneticFieldGeneric<FloatT>::model_t IGRF11Generic<FloatT>::IGRF20 = {
    "IGRF20",
    1920.0,
    10,
    {-31060.0, -2317.0, 5845.0, // 1
        -839.0, 2959.0, -1259.0, 1407.0, 823.0, // 2
        1111.0, -1600.0, -445.0, 1205.0, 103.0, 839.0, 293.0, // 3
        889.0, 695.0, 220.0, 616.0, -134.0, -424.0, -153.0, 199.0, -57.0, // 4
        -221.0, 326.0, -122.0, 236.0, 58.0, -23.0, -38.0, -119.0, -125.0, -62.0, 43.0, // 5
        61.0, 55.0, 0.0, -10.0, 96.0, -233.0, 11.0, -46.0, -22.0, 44.0, 18.0, -101.0, -57.0, // 6
        73.0, -54.0, -49.0, 2.0, -14.0, 29.0, -13.0, -37.0, 4.0, -16.0, 28.0, 19.0, -16.0, 6.0, -22.0, // 7
        11.0, 7.0, 8.0, -3.0, -15.0, -9.0, 6.0, 2.0, -14.0, 4.0, 5.0, -7.0, 17.0, 6.0, -5.0, 8.0, -19.0, // 8
        8.0, 10.0, -20.0, 1.0, 14.0, -11.0, 5.0, 12.0, -3.0, 1.0, -2.0, -2.0, 9.0, 2.0, 10.0, 0.0, -2.0, -1.0, 2.0, // 9
        -3.0, -4.0, 2.0, 2.0, 1.0, -5.0, 2.0, -2.0, 6.0, 6.0, -4.0, 4.0, 0.0, 0.0, -2.0, 1.0, 4.0, 3.0, 0.0, 0.0, -6.0, } // 10
    };
template <class FloatT>
const typename MagneticFieldGeneric<FloatT>::model_t IGRF11Generic<FloatT>::IGRF2010 = {
    "IGRF2010",
    2010.0,
    13,
    {-29496.5, -1585.9, 4945.1, // 1
        -2396.6, 3026.0, -2707.7, 1668.6, -575.4, // 2
        1339.7, -2326.3, -160.5, 1231.7, 251.7, 634.2, -536.8, // 3
        912.6, 809.0, 286.4, 166.6, -211.2, -357.1, 164.4, 89.7, -309.2, // 4
        -231.1, 357.2, 44.7, 200.3, 188.9, -141.2, -118.1, -163.1, 0.1, -7.7, 100.9, // 5
        72.8, 68.6, -20.8, 76.0, 44.2, -141.4, 61.5, -22.9, -66.3, 13.1, 3.1, -77.9, 54.9, // 6
        80.4, -75.0, -57.8, -4.7, -21.2, 45.3, 6.6, 14.0, 24.9, 10.4, 7.0, 1.6, -27.7, 4.9, -3.4, // 7
        24.3, 8.2, 10.9, -14.5, -20.0, -5.7, 11.9, -19.3, -17.4, 11.6, 16.7, 10.9, 7.1, -14.1, -10.8, -3.7, 1.7, // 8
        5.4, 9.4, -20.5, 3.4, 11.6, -5.3, 12.8, 3.1, -7.2, -12.4, -7.4, -0.8, 8.0, 8.4, 2.2, -8.4, -6.1, -10.1, 7.0, // 9
        -2.0, -6.3, 2.8, 0.9, -0.1, -1.1, 4.7, -0.2, 4.4, 2.5, -7.2, -0.3, -1.0, 2.2, -4.0, 3.1, -2.0, -1.0, -2.0, -2.8, -8.3, // 10
        3.0, -1.5, 0.1, -2.1, 1.7, 1.6, -0.6, -0.5, -1.8, 0.5, 0.9, -0.8, -0.4, 0.4, -2.5, 1.8, -1.3, 0.2, -2.1, 0.8, -1.9, 3.8, -1.8, // 11
        -2.1, -0.2, -0.8, 0.3, 0.3, 1.0, 2.2, -0.7, -2.5, 0.9, 0.5, -0.1, 0.6, 0.5, 0.0, -0.4, 0.1, -0.4, 0.3, 0.2, -0.9, -0.8, -0.2, 0.0, 0.8, // 12
        -0.2, -0.9, -0.8, 0.3, 0.3, 0.4, 1.7, -0.4, -0.6, 1.1, -1.2, -0.3, -0.1, 0.8, 0.5, -0.2, 0.1, 0.4, 0.5, 0.0, 0.4, 0.4, -0.2, -0.3, -0.5, -0.3, -0.8, } // 13
    };
template <class FloatT>
const typename MagneticFieldGeneric<FloatT>::model_t IGRF11Generic<FloatT>::IGRF25 = {
    "IGRF25",
    1925.0,
    10,
    {-30926.0, -2318.0, 5817.0, // 1
        -893.0, 2969.0, -1334.0, 1471.0, 728.0, // 2
        1140.0, -1645.0, -462.0, 1202.0, 119.0, 881.0, 229.0, // 3
        891.0, 711.0, 216.0, 601.0, -163.0, -426.0, -130.0, 217.0, -70.0, // 4
        -230.0, 326.0, -96.0, 226.0, 58.0, -28.0, -44.0, -125.0, -122.0, -69.0, 51.0, // 5
        61.0, 54.0, 3.0, -9.0, 99.0, -238.0, 14.0, -40.0, -18.0, 39.0, 13.0, -103.0, -52.0, // 6
        73.0, -54.0, -50.0, 3.0, -14.0, 27.0, -14.0, -35.0, 5.0, -14.0, 29.0, 19.0, -17.0, 6.0, -21.0, // 7
        11.0, 7.0, 8.0, -3.0, -15.0, -9.0, 6.0, 2.0, -14.0, 4.0, 5.0, -7.0, 17.0, 7.0, -5.0, 8.0, -19.0, // 8
        8.0, 10.0, -20.0, 1.0, 14.0, -11.0, 5.0, 12.0, -3.0, 1.0, -2.0, -2.0, 9.0, 2.0, 10.0, 0.0, -2.0, -1.0, 2.0, // 9
        -3.0, -4.0, 2.0, 2.0, 1.0, -5.0, 2.0, -2.0, 6.0, 6.0, -4.0, 4.0, 0.0, 0.0, -2.0, 1.0, 4.0, 3.0, 0.0, 0.0, -6.0, } // 10
    };
template <class FloatT>
const typename MagneticFieldGeneric<FloatT>::model_t IGRF11Generic<FloatT>::IGRF30 = {
    "IGRF30",
    1930.0,
    10,
    {-30805.0, -2316.0, 5808.0, // 1
        -951.0, 2980.0, -1424.0, 1517.0, 644.0, // 2
        1172.0, -1692.0, -480.0, 1205.0, 133.0, 907.0, 166.0, // 3
        896.0, 727.0, 205.0, 584.0, -195.0, -422.0, -109.0, 234.0, -90.0, // 4
        -237.0, 327.0, -72.0, 218.0, 60.0, -32.0, -53.0, -131.0, -118.0, -74.0, 58.0, // 5
        60.0, 53.0, 4.0, -9.0, 102.0, -242.0, 19.0, -32.0, -16.0, 32.0, 8.0, -104.0, -46.0, // 6
        74.0, -54.0, -51.0, 4.0, -15.0, 25.0, -14.0, -34.0, 6.0, -12.0, 29.0, 18.0, -18.0, 6.0, -20.0, // 7
        11.0, 7.0, 8.0, -3.0, -15.0, -9.0, 5.0, 2.0, -14.0, 5.0, 5.0, -6.0, 18.0, 8.0, -5.0, 8.0, -19.0, // 8
        8.0, 10.0, -20.0, 1.0, 14.0, -12.0, 5.0, 12.0, -3.0, 1.0, -2.0, -2.0, 9.0, 3.0, 10.0, 0.0, -2.0, -2.0, 2.0, // 9
        -3.0, -4.0, 2.0, 2.0, 1.0, -5.0, 2.0, -2.0, 6.0, 6.0, -4.0, 4.0, 0.0, 0.0, -2.0, 1.0, 4.0, 3.0, 0.0, 0.0, -6.0, } // 10
    };
template <class FloatT>
const typename MagneticFieldGeneric<FloatT>::model_t IGRF11Generic<FloatT>::IGRF35 = {
    "IGRF35",
    1935.0,
    10,
    {-30715.0, -2306.0, 5812.0, // 1
        -1018.0, 2984.0, -1520.0, 1550.0, 586.0, // 2
        1206.0, -1740.0, -494.0, 1215.0, 146.0, 918.0, 101.0, // 3
        903.0, 744.0, 188.0, 565.0, -226.0, -415.0, -90.0, 249.0, -114.0, // 4
        -241.0, 329.0, -51.0, 211.0, 64.0, -33.0, -64.0, -136.0, -115.0, -76.0, 64.0, // 5
        59.0, 53.0, 4.0, -8.0, 104.0, -246.0, 25.0, -25.0, -15.0, 25.0, 4.0, -106.0, -40.0, // 6
        74.0, -53.0, -52.0, 4.0, -17.0, 23.0, -14.0, -33.0, 7.0, -11.0, 29.0, 18.0, -19.0, 6.0, -19.0, // 7
        11.0, 7.0, 8.0, -3.0, -15.0, -9.0, 5.0, 1.0, -15.0, 6.0, 5.0, -6.0, 18.0, 8.0, -5.0, 7.0, -19.0, // 8
        8.0, 10.0, -20.0, 1.0, 15.0, -12.0, 5.0, 11.0, -3.0, 1.0, -3.0, -2.0, 9.0, 3.0, 11.0, 0.0, -2.0, -2.0, 2.0, // 9
        -3.0, -4.0, 2.0, 2.0, 1.0, -5.0, 2.0, -2.0, 6.0, 6.0, -4.0, 4.0, 0.0, 0.0, -1.0, 2.0, 4.0, 3.0, 0.0, 0.0, -6.0, } // 10
    };
template <class FloatT>
const typename MagneticFieldGeneric<FloatT>::model_t IGRF11Generic<FloatT>::IGRF40 = {
    "IGRF40",
    1940.0,
    10,
    {-30654.0, -2292.0, 5821.0, // 1
        -1106.0, 2981.0, -1614.0, 1566.0, 528.0, // 2
        1240.0, -1790.0, -499.0, 1232.0, 163.0, 916.0, 43.0, // 3
        914.0, 762.0, 169.0, 550.0, -252.0, -405.0, -72.0, 265.0, -141.0, // 4
        -241.0, 334.0, -33.0, 208.0, 71.0, -33.0, -75.0, -141.0, -113.0, -76.0, 69.0, // 5
        57.0, 54.0, 4.0, -7.0, 105.0, -249.0, 33.0, -18.0, -15.0, 18.0, 0.0, -107.0, -33.0, // 6
        74.0, -53.0, -52.0, 4.0, -18.0, 20.0, -14.0, -31.0, 7.0, -9.0, 29.0, 17.0, -20.0, 5.0, -19.0, // 7
        11.0, 7.0, 8.0, -3.0, -14.0, -10.0, 5.0, 1.0, -15.0, 6.0, 5.0, -5.0, 19.0, 9.0, -5.0, 7.0, -19.0, // 8
        8.0, 10.0, -21.0, 1.0, 15.0, -12.0, 5.0, 11.0, -3.0, 1.0, -3.0, -2.0, 9.0, 3.0, 11.0, 1.0, -2.0, -2.0, 2.0, // 9
        -3.0, -4.0, 2.0, 2.0, 1.0, -5.0, 2.0, -2.0, 6.0, 6.0, -4.0, 4.0, 0.0, 0.0, -1.0, 2.0, 4.0, 3.0, 0.0, 0.0, -6.0, } // 10
    };

template <class FloatT>
class IGRF12Generic : public IGRF11Generic<FloatT> {
  public:
    static const typename MagneticFieldGeneric<FloatT>::model_t DGRF2010;
    static const typename MagneticFieldGeneric<FloatT>::model_t IGRF2015;

    static typename MagneticFieldGeneric<FloatT>::model_t get_model(const FloatT &year){
      static const typename MagneticFieldGeneric<FloatT>::model_t *models[] = {
        &IGRF11Generic<FloatT>::IGRF00,
        &IGRF11Generic<FloatT>::IGRF05,
        &IGRF11Generic<FloatT>::IGRF10,
        &IGRF11Generic<FloatT>::IGRF15,
        &IGRF11Generic<FloatT>::IGRF20,
        &IGRF11Generic<FloatT>::IGRF25,
        &IGRF11Generic<FloatT>::IGRF30,
        &IGRF11Generic<FloatT>::IGRF35,
        &IGRF11Generic<FloatT>::IGRF40,
        &IGRF11Generic<FloatT>::DGRF45,
        &IGRF11Generic<FloatT>::DGRF50,
        &IGRF11Generic<FloatT>::DGRF55,
        &IGRF11Generic<FloatT>::DGRF60,
        &IGRF11Generic<FloatT>::DGRF65,
        &IGRF11Generic<FloatT>::DGRF70,
        &IGRF11Generic<FloatT>::DGRF75,
        &IGRF11Generic<FloatT>::DGRF80,
        &IGRF11Generic<FloatT>::DGRF85,
        &IGRF11Generic<FloatT>::DGRF90,
        &IGRF11Generic<FloatT>::DGRF95,
        &IGRF11Generic<FloatT>::DGRF2000,
        &IGRF11Generic<FloatT>::DGRF2005,
        &DGRF2010,
        &IGRF2015,
      };
      typename MagneticFieldGeneric<FloatT>::model_t res;
      res.year = year;
      MagneticFieldGeneric<FloatT>::make_model(res, models, sizeof(models) / sizeof(models[0]));
      return res;
    }
};

typedef IGRF12Generic<double> IGRF12;

template <class FloatT>
const typename MagneticFieldGeneric<FloatT>::model_t IGRF12Generic<FloatT>::DGRF2010 = {
    "DGRF2010",
    2010.0,
    13,
    {-29496.57, -1586.42, 4944.26, // 1
        -2396.06, 3026.34, -2708.54, 1668.17, -575.73, // 2
        1339.85, -2326.54, -160.4, 1232.1, 251.75, 633.73, -537.03, // 3
        912.66, 808.97, 286.48, 166.58, -211.03, -356.83, 164.46, 89.4, -309.72, // 4
        -230.87, 357.29, 44.58, 200.26, 189.01, -141.05, -118.06, -163.17, -0.01, -8.03, 101.04, // 5
        72.78, 68.69, -20.9, 75.92, 44.18, -141.4, 61.54, -22.83, -66.26, 13.1, 3.02, -78.09, 55.4, // 6
        80.44, -75.0, -57.8, -4.55, -21.2, 45.24, 6.54, 14.0, 24.96, 10.46, 7.03, 1.64, -27.61, 4.92, -3.28, // 7
        24.41, 8.21, 10.84, -14.5, -20.03, -5.59, 11.83, -19.34, -17.41, 11.61, 16.71, 10.85, 6.96, -14.05, -10.74, -3.54, 1.64, // 8
        5.5, 9.45, -20.54, 3.45, 11.51, -5.27, 12.75, 3.13, -7.14, -12.38, -7.42, -0.76, 7.97, 8.43, 2.14, -8.42, -6.08, -10.08, 7.01, // 9
        -1.94, -6.24, 2.73, 0.89, -0.1, -1.07, 4.71, -0.16, 4.44, 2.45, -7.22, -0.33, -0.96, 2.13, -3.95, 3.09, -1.99, -1.03, -1.97, -2.8, -8.31, // 10
        3.05, -1.48, 0.13, -2.03, 1.67, 1.65, -0.66, -0.51, -1.76, 0.54, 0.85, -0.79, -0.39, 0.37, -2.51, 1.79, -1.27, 0.12, -2.11, 0.75, -1.94, 3.75, -1.86, // 11
        -2.12, -0.21, -0.87, 0.3, 0.27, 1.04, 2.13, -0.63, -2.49, 0.95, 0.49, -0.11, 0.59, 0.52, 0.0, -0.39, 0.13, -0.37, 0.27, 0.21, -0.86, -0.77, -0.23, 0.04, 0.87, // 12
        -0.09, -0.89, -0.87, 0.31, 0.3, 0.42, 1.66, -0.45, -0.59, 1.08, -1.14, -0.31, -0.07, 0.78, 0.54, -0.18, 0.1, 0.38, 0.49, 0.02, 0.44, 0.42, -0.25, -0.26, -0.53, -0.26, -0.79, } // 13
    };
template <class FloatT>
const typename MagneticFieldGeneric<FloatT>::model_t IGRF12Generic<FloatT>::IGRF2015 = {
    "IGRF2015",
    2015.0,
    13,
    {-29442.0, -1501.0, 4797.1, // 1
        -2445.1, 3012.9, -2845.6, 1676.7, -641.9, // 2
        1350.7, -2352.3, -115.3, 1225.6, 244.9, 582.0, -538.4, // 3
        907.6, 813.7, 283.3, 120.4, -188.7, -334.9, 180.9, 70.4, -329.5, // 4
        -232.6, 360.1, 47.3, 192.4, 197.0, -140.9, -119.3, -157.5, 16.0, 4.1, 100.2, // 5
        70.0, 67.7, -20.8, 72.7, 33.2, -129.9, 58.9, -28.9, -66.7, 13.2, 7.3, -70.9, 62.6, // 6
        81.6, -76.1, -54.1, -6.8, -19.5, 51.8, 5.7, 15.0, 24.4, 9.4, 3.4, -2.8, -27.4, 6.8, -2.2, // 7
        24.2, 8.8, 10.1, -16.9, -18.3, -3.2, 13.3, -20.6, -14.6, 13.4, 16.2, 11.7, 5.7, -15.9, -9.1, -2.0, 2.1, // 8
        5.4, 8.8, -21.6, 3.1, 10.8, -3.3, 11.8, 0.7, -6.8, -13.3, -6.9, -0.1, 7.8, 8.7, 1.0, -9.1, -4.0, -10.5, 8.4, // 9
        -1.9, -6.3, 3.2, 0.1, -0.4, 0.5, 4.6, -0.5, 4.4, 1.8, -7.9, -0.7, -0.6, 2.1, -4.2, 2.4, -2.8, -1.8, -1.2, -3.6, -8.7, // 10
        3.1, -1.5, -0.1, -2.3, 2.0, 2.0, -0.7, -0.8, -1.1, 0.6, 0.8, -0.7, -0.2, 0.2, -2.2, 1.7, -1.4, -0.2, -2.5, 0.4, -2.0, 3.5, -2.4, // 11
        -1.9, -0.2, -1.1, 0.4, 0.4, 1.2, 1.9, -0.8, -2.2, 0.9, 0.3, 0.1, 0.7, 0.5, -0.1, -0.3, 0.3, -0.4, 0.2, 0.2, -0.9, -0.9, -0.1, 0.0, 0.7, // 12
        0.0, -0.9, -0.9, 0.4, 0.4, 0.5, 1.6, -0.5, -0.5, 1.0, -1.2, -0.2, -0.1, 0.8, 0.4, -0.1, -0.1, 0.3, 0.4, 0.1, 0.5, 0.5, -0.3, -0.4, -0.4, -0.3, -0.8, } // 13
    };

/*
 * WMM2010 is the standard model for the U.S. and U.K. Departments of Defense  
 * and for NATO, also used widely in civilian navigation systems. This is a
 * degree and order 12 main field model for 2010.0 and degree and order 12
 * secular variation model for 2010 - 2015. For more information on the WMM or
 * to download the Technical Report, visit the WMM Web site at:
 *        http://www.ngdc.noaa.gov/geomag/WMM/
 */

template <class FloatT>
class WMM2010Generic : public MagneticFieldGeneric<FloatT> {
  public:
    static const typename MagneticFieldGeneric<FloatT>::model_t WMM;

    static typename MagneticFieldGeneric<FloatT>::model_t get_model(const FloatT &year){
      return WMM;
    }
};

typedef WMM2010Generic<double> WMM2010;

template <class FloatT>
const typename MagneticFieldGeneric<FloatT>::model_t WMM2010Generic<FloatT>::WMM = {
    "WMM2010",
    2010.0,
    12,
    {-29496.6, -1586.3, 4944.4, // 1
        -2396.6, 3026.1, -2707.7, 1668.6, -576.1, // 2
        1340.1, -2326.2, -160.2, 1231.9, 251.9, 634.0, -536.6, // 3
        912.6, 808.9, 286.4, 166.7, -211.2, -357.1, 164.3, 89.4, -309.1, // 4
        -230.9, 357.2, 44.6, 200.3, 188.9, -141.1, -118.2, -163.0, 0.0, -7.8, 100.9, // 5
        72.8, 68.6, -20.8, 76.0, 44.1, -141.4, 61.5, -22.8, -66.3, 13.2, 3.1, -77.9, 55.0, // 6
        80.5, -75.1, -57.9, -4.7, -21.1, 45.3, 6.5, 13.9, 24.9, 10.4, 7.0, 1.7, -27.7, 4.9, -3.3, // 7
        24.4, 8.1, 11.0, -14.5, -20.0, -5.6, 11.9, -19.3, -17.4, 11.5, 16.7, 10.9, 7.0, -14.1, -10.8, -3.7, 1.7, // 8
        5.4, 9.4, -20.5, 3.4, 11.5, -5.2, 12.8, 3.1, -7.2, -12.4, -7.4, -0.7, 8.0, 8.4, 2.1, -8.5, -6.1, -10.1, 7.0, // 9
        -2.0, -6.3, 2.8, 0.9, -0.1, -1.1, 4.7, -0.2, 4.4, 2.5, -7.2, -0.3, -1.0, 2.2, -3.9, 3.1, -2.0, -1.0, -2.0, -2.8, -8.3, // 10
        3.0, -1.5, 0.2, -2.1, 1.7, 1.7, -0.6, -0.5, -1.8, 0.5, 0.9, -0.8, -0.4, 0.4, -2.5, 1.8, -1.3, 0.1, -2.1, 0.7, -1.9, 3.8, -1.8, // 11
        -2.2, -0.2, -0.9, 0.3, 0.3, 1.0, 2.1, -0.6, -2.5, 0.9, 0.5, -0.1, 0.6, 0.5, -0.0, -0.4, 0.1, -0.4, 0.3, 0.2, -0.9, -0.8, -0.2, 0.0, 0.9, } // 12
    };

#endif /* __MAGNETIC_FIELD_H__ */
