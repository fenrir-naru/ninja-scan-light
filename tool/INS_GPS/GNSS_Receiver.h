/*
 * Copyright (c) 2020, M.Naruoka (fenrir)
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

#ifndef __GNSS_RECEIVER_H__
#define __GNSS_RECEIVER_H__

#include <iostream>
#include <cmath>
#include <cstring>

#include "navigation/GPS.h"
#include "navigation/SBAS.h"
#include "navigation/GPS_Solver.h"
#include "navigation/SBAS_Solver.h"
#include "navigation/RINEX.h"

#include "navigation/INS_GPS2_Tightly.h"

#include "INS_GPS/GNSS_Data.h"
#include "SylphideProcessor.h"

#include "analyze_common.h"

#if !defined(BUILD_WITHOUT_GNSS_RAIM)
#include "navigation/GPS_Solver_RAIM.h"
#endif

#if !defined(BUILD_WITHOUT_GNSS_MULTI_FREQUENCY)
#include "navigation/GPS_Solver_MultiFrequency.h"
#endif

#if !defined(BUILD_WITHOUT_SP3)
#include "navigation/SP3.h"
#include "navigation/ANTEX.h"
#endif

#if !defined(BUILD_WITHOUT_RINEX_CLK)
#include "navigation/RINEX_Clock.h"
#endif

template <class FloatT>
struct GNSS_Receiver {
  typedef GPS_SpaceNode<FloatT> gps_space_node_t;

#if !defined(BUILD_WITHOUT_GNSS_RAIM)
#if defined(BUILD_WITH_GNSS_DEBUG)
  typedef GPS_Solver_RAIM_LSR<FloatT, GPS_Solver_Base_Debug<FloatT> > solver_base_t;
#else
  typedef GPS_Solver_RAIM_LSR<FloatT> solver_base_t;
#endif
#else
#if defined(BUILD_WITH_GNSS_DEBUG)
  typedef GPS_Solver_Base_Debug<FloatT> solver_base_t;
#else
  typedef GPS_Solver_Base<FloatT> solver_base_t;
#endif
#endif

#if !defined(BUILD_WITHOUT_GNSS_MULTI_FREQUENCY)
  typedef GPS_Solver_MultiFrequency<GPS_SinglePositioning<FloatT, solver_base_t> > gps_solver_t;
#else
  typedef GPS_SinglePositioning<FloatT, solver_base_t> gps_solver_t;
#endif

  typedef SBAS_SpaceNode<FloatT> sbas_space_node_t;
  typedef SBAS_SinglePositioning<FloatT, solver_base_t> sbas_solver_t;

  struct system_t;

  struct data_t {
    struct {
      gps_space_node_t space_node;
      typename gps_solver_t::options_t solver_options;
    } gps;
    struct {
      sbas_space_node_t space_node;
      typename sbas_solver_t::options_t solver_options;
    } sbas;
    std::ostream *out_rinex_nav;
#if !defined(BUILD_WITHOUT_SP3)
    typedef SP3_Product<FloatT> sp3_t;
    sp3_t sp3;
#endif
#if !defined(BUILD_WITHOUT_RINEX_CLK)
    typedef typename RINEX_CLK<FloatT>::satellites_t clk_t;
    clk_t clk;
#endif
    data_t() : gps(), sbas(), out_rinex_nav(NULL) {}
    ~data_t(){
      if(out_rinex_nav){
        typename RINEX_NAV_Writer<FloatT>::space_node_list_t list = {&gps.space_node};
        list.sbas = &sbas.space_node;
        RINEX_NAV_Writer<FloatT>::write_all(*out_rinex_nav, list);
      }
    }
  } data;

  struct solver_t : public solver_base_t {
    typedef solver_base_t base_t;
    gps_solver_t gps;
    sbas_solver_t sbas;
    struct measurement_items_t : public gps_solver_t::measurement_items_t {
      // TODO
    };
    solver_t(const GNSS_Receiver &rcv)
        : base_t(),
        gps(rcv.data.gps.space_node),
        sbas(rcv.data.sbas.space_node) {
#if !defined(BUILD_WITHOUT_GNSS_MULTI_CONSTELLATION)
      // ionospheric and tropospheric correction methods
      typename base_t::range_correction_t ionospheric, tropospheric;
      ionospheric.push_back(&sbas.ionospheric_sbas);
      ionospheric.push_back(&gps.ionospheric_klobuchar);
      tropospheric.push_back(&sbas.tropospheric_sbas);
      tropospheric.push_back(&gps.tropospheric_simplified);
      gps.ionospheric_correction
          = sbas.ionospheric_correction
          = ionospheric;
      gps.tropospheric_correction
          = sbas.tropospheric_correction
          = tropospheric;
#endif
    }

    // Proxy functions
    const base_t &select(const typename base_t::prn_t &serial) const {
      switch(system_t::serial2system(serial)){
        case system_t::GPS: 	return gps;
        case system_t::SBAS:	return sbas;
      }
      return *this;
    }

    void copy_range_correction(const solver_t &another){
#define MAKE_ENTRY(target) {&(another.target), &(target)}
      struct {
        const typename base_t::range_corrector_t *src;
        typename base_t::range_corrector_t *dst;
      } item[] = {
        {&base_t::no_correction,
            const_cast<typename base_t::no_correction_t *>(&base_t::no_correction)},
        MAKE_ENTRY(gps.ionospheric_klobuchar),
        MAKE_ENTRY(gps.ionospheric_ntcm_gl),
        MAKE_ENTRY(gps.tropospheric_simplified),
        MAKE_ENTRY(sbas.ionospheric_sbas),
        MAKE_ENTRY(sbas.tropospheric_sbas),
      };
      struct {
        const typename base_t::range_correction_t *src;
        typename base_t::range_correction_t *dst;
      } root[] = {
        MAKE_ENTRY(gps.ionospheric_correction),
        MAKE_ENTRY(gps.tropospheric_correction),
        MAKE_ENTRY(sbas.ionospheric_correction),
        MAKE_ENTRY(sbas.tropospheric_correction),
      };
#undef MAKE_ENTRY
      for(std::size_t i(0); i < sizeof(root) / sizeof(root[0]); ++i){
        root[i].dst->clear();
        for(typename base_t::range_correction_t::const_iterator
              it(root[i].src->begin()), it_end(root[i].src->end());
            it != it_end; ++it){
          for(std::size_t j(0); j < sizeof(item) / sizeof(item[0]); ++j){
            if(*it != item[j].src){continue;}
            root[i].dst->push_back(item[j].dst);
            break;
          }
        }
      }
    }

    void update_ephemeris_source(const data_t &data){
#if !defined(BUILD_WITHOUT_SP3)
      typename data_t::sp3_t::satellite_count_t cnt(data.sp3.satellite_count());
      if(cnt.gps > 0){data.sp3.push(gps.satellites, SP3_Product<FloatT>::SYSTEM_GPS);}
      if(cnt.sbas > 0){data.sp3.push(sbas.satellites, SP3_Product<FloatT>::SYSTEM_SBAS);}
#endif
#if !defined(BUILD_WITHOUT_RINEX_CLK)
      // RINEX clock has higher priority to be applied than SP3
      typename data_t::clk_t::count_t cnt2(data.clk.count());
      if(cnt2.gps > 0){data.clk.push(gps.satellites, data_t::clk_t::SYSTEM_GPS);}
      if(cnt2.sbas > 0){data.clk.push(sbas.satellites, data_t::clk_t::SYSTEM_SBAS);}
#endif
    }
  } solver_GNSS;

  GNSS_Receiver() : data(), solver_GNSS(*this) {}
  GNSS_Receiver(const GNSS_Receiver &another)
      : data(another.data), solver_GNSS(*this) {
    solver_GNSS.copy_range_correction(another.solver_GNSS);
    solver_GNSS.update_ephemeris_source(data);
  }
  GNSS_Receiver &operator=(const GNSS_Receiver &another){
    data = another.data;
    solver_GNSS.copy_range_correction(another.solver_GNSS);
    solver_GNSS.update_ephemeris_source(data);
    return *this;
  }

  void setup(typename GNSS_Data<FloatT>::Loader &loader) const {
    loader.gps = &const_cast<gps_space_node_t &>(data.gps.space_node);
    loader.sbas = &const_cast<sbas_space_node_t &>(data.sbas.space_node);
  }

  const solver_base_t &solver() const {
#if !defined(BUILD_WITHOUT_GNSS_MULTI_CONSTELLATION)
    return solver_GNSS;
#else
    return solver_GNSS.gps;
#endif
  }

  void adjust(const GPS_Time<FloatT> &t){
    // Select most preferable ephemeris
    data.gps.space_node.update_all_ephemeris(t);
    data.sbas.space_node.update_all_ephemeris(t);

    // Solver update mainly for preferable ionospheric model selection
    // based on their availability
    solver_GNSS.gps.update_options(data.gps.solver_options);
    solver_GNSS.sbas.update_options(data.sbas.solver_options);
  }

  struct system_t {
    enum type_t {
#define make_entry(key) dummy_ ## key, key = ((dummy_ ## key + 0xFF) & ~0xFF), \
  key ## _SHIFT = key >> 8, dummy2_ ## key = key
      make_entry(GPS),      // 0x000
      SBAS,                 // 0x001
      QZSS,                 // 0x002
      make_entry(GLONASS),  // 0x100
      make_entry(Galileo),  // 0x200
      make_entry(Beido),    // 0x300
      make_entry(Unknown),  // 0x400
#undef make_entry
    };
    static type_t serial2system(const typename solver_t::prn_t &serial){
      if(serial <= 0){return Unknown;}
      switch(serial >> 8){
        case GPS_SHIFT:
          if(serial <= 32){return GPS;}
          else if(serial < 120){break;}
          else if(serial <= 158){return SBAS;}
          else if(serial < 193){break;}
          else if(serial <= 202){return QZSS;}
          else {break;}
        case GLONASS_SHIFT: return GLONASS;
        case Galileo_SHIFT: return Galileo;
        case Beido_SHIFT:   return Beido;
      }
      return Unknown;
    }
    static const struct type_str_t {
      type_t type;
      const char *str;
    } type_str_list[];
    static const int type_str_list_length;
    static const char *system2str(const type_t &type){
      for(int i(0); i < type_str_list_length; ++i){
        if(type == type_str_list[i].type){
          return type_str_list[i].str;
        }
      }
      return "Unknown";
    }
    static type_t str2system(const char *system_str){
      for(int i(0); i < type_str_list_length; ++i){
        if(std::strcmp(system_str, type_str_list[i].str) == 0){
          return type_str_list[i].type;
        }
      }
      return Unknown;
    }
  };

  struct satellite_id_t {
    const typename system_t::type_t type;
    const int sv_id;
    satellite_id_t(const typename solver_t::prn_t &serial)
        : type(system_t::serial2system(serial)),
        sv_id((type != system_t::Unknown) ? (serial & 0xFF) : 0)
        {}
    satellite_id_t(const typename system_t::type_t &_type, const unsigned int &_sv_id)
        : type(_type),
        sv_id((type != system_t::Unknown) ? ((int)_sv_id & 0xFF) : 0)
        {}
    operator typename solver_t::prn_t() const {
      return ((type & ~0xFF) | sv_id); // 0x(GNSS_type)_(8bits:SVID)
    }
    friend std::ostream &operator<<(std::ostream &out, const satellite_id_t &id){
      out << system_t::system2str(id.type);
      if(id.type != system_t::Unknown){
        out << '(' << id.sv_id << ')';
      }
      return out;
    }
  };

  /**
   * Generate satellite unique serial from UBX GNSS ID and SV ID
   * @param gnss_id GNSS ID defined in UBX protocol
   * @param sv_id Space vehicle ID defined in UBX protocol
   * @return unique satellite serial. If a satellite system uses PRN such as GPS and SBAS,
   * the return value is identical to PRN code.
   */
  static typename solver_t::prn_t satellite_serial(
      const unsigned int &gnss_id,
      unsigned int sv_id){
    typedef G_Packet_Observer<FloatT> decorder_t;
    typename system_t::type_t type(system_t::Unknown);
    switch(gnss_id){
      case decorder_t::gnss_svid_t::GPS:      type = system_t::GPS;     break;
      case decorder_t::gnss_svid_t::SBAS:     type = system_t::SBAS;    break;
      case decorder_t::gnss_svid_t::QZSS:     type = system_t::QZSS;
        sv_id = typename decorder_t::gnss_svid_t(gnss_id, sv_id); // convert to legacy svid (1-5 => 193-197)
        break;
      case decorder_t::gnss_svid_t::Galileo:  type = system_t::Galileo; break;
      case decorder_t::gnss_svid_t::BeiDou:   type = system_t::Beido;   break;
      case decorder_t::gnss_svid_t::GLONASS:  type = system_t::GLONASS; break;
    }
    return satellite_id_t(type, sv_id);
  }

  /**
   * Check whether combination of GNSS and signal is supported
   * @param gnss_id GNSS ID defined in UBX protocol
   * @param signal_id Signal ID defined in UBX protocol
   * @return If a combination is supported, the corresponding pointer of
   * measurement_item_set_t, which contains indices of measurement items,
   * is returned. Otherwise, NULL.
   */
  static const typename solver_t::measurement_item_set_t *is_supported(
      const unsigned int &gnss_id,
      const unsigned int &signal_id){
    typedef G_Packet_Observer<FloatT> decorder_t;
    switch(decorder_t::gnss_signal_t::decode(gnss_id, signal_id)){
      case decorder_t::gnss_signal_t::GPS_L1CA:
        // GPS L1 C/A (SBAS and QZSS are included because of same signal)
        return &(gps_solver_t::L1CA);
#if !defined(BUILD_WITHOUT_GNSS_MULTI_FREQUENCY)
      case decorder_t::gnss_signal_t::GPS_L2CM:
        return &(gps_solver_t::L2CM);
      case decorder_t::gnss_signal_t::GPS_L2CL:
        return &(gps_solver_t::L2CL);
#endif
    }
    return NULL; // TODO support other GNSS, signals
  }

  typedef GlobalOptions<FloatT> runtime_opt_t;

  bool check_spec(
      runtime_opt_t &options,
      const char *spec, const bool &dry_run = false){
    const char *value;

    if(value = runtime_opt_t::get_value(spec, "rinex_nav", false)){
      if(dry_run){return true;}
      std::cerr << "RINEX Navigation file (" << value << ") reading..." << std::endl;
      std::istream &in(options.spec2istream(value));
      typename RINEX_NAV_Reader<FloatT>::space_node_list_t list = {&data.gps.space_node};
      list.sbas = &data.sbas.space_node;
      int ephemeris(RINEX_NAV_Reader<FloatT>::read_all(in, list));
      if(ephemeris < 0){
        std::cerr << "(error!) Invalid format!" << std::endl;
        return false;
      }else{
        std::cerr << "rinex_nav: " << ephemeris << " items captured." << std::endl;
      }
      return true;
    }
    if(value = runtime_opt_t::get_value(spec, "out_rinex_nav", false)){
      if(dry_run){return true;}
      data.out_rinex_nav = &options.spec2ostream(value);
      std::cerr << "out_rinex_nav: " << value << std::endl;
      return true;
    }

#if !defined(BUILD_WITHOUT_SP3)
    if(value = runtime_opt_t::get_value(spec, "sp3", false)){
      if(dry_run){return true;}
      std::cerr << "SP3 file (" << value << ") reading..." << std::endl;
      std::istream &in(options.spec2istream(value));
      int entries(SP3_Reader<FloatT>::read_all(in, data.sp3));
      if(entries < 0){
        std::cerr << "(error!) Invalid format!" << std::endl;
        return false;
      }else{
        std::cerr << "sp3: " << entries << " items captured." << std::endl;
        typename data_t::sp3_t::satellite_count_t cnt(data.sp3.satellite_count());
        if(cnt.gps > 0){std::cerr << "SP3 GPS satellites: " << cnt.gps << std::endl;}
        if(cnt.sbas > 0){std::cerr << "SP3 SBAS satellites: " << cnt.sbas << std::endl;}
      }
      solver_GNSS.update_ephemeris_source(data);
      return true;
    }
    if(value = runtime_opt_t::get_value(spec, "antex", false)){
      if(dry_run){return true;}
      std::cerr << "ANTEX file (" << value << ") reading..." << std::endl;
      std::istream &in(options.spec2istream(value));
      ANTEX_Product<FloatT> atx;
      int entries(ANTEX_Reader<FloatT>::read_all(in, atx));
      if(entries < 0){
        std::cerr << "(error!) Invalid format!" << std::endl;
        return false;
      }else{
        std::cerr << "antex: " << entries << " items captured." << std::endl;
        int moved(atx.move_to_antenna_position(data.sp3));
        std::cerr << "Moved SP3 positions: " << moved << std::endl;
      }
      return true;
    }
#endif

#if !defined(BUILD_WITHOUT_RINEX_CLK)
    if(value = runtime_opt_t::get_value(spec, "rinex_clk", false)){
      if(dry_run){return true;}
      std::cerr << "RINEX clock file (" << value << ") reading..." << std::endl;
      std::istream &in(options.spec2istream(value));
      int entries(RINEX_CLK_Reader<FloatT>::read_all(in, data.clk));
      if(entries < 0){
        std::cerr << "(error!) Invalid format!" << std::endl;
        return false;
      }else{
        std::cerr << "rinex_clk: " << entries << " items captured." << std::endl;
        typename data_t::clk_t::count_t cnt(data.clk.count());
        if(cnt.gps > 0){std::cerr << "RINEX clock GPS satellites: " << cnt.gps << std::endl;}
        if(cnt.sbas > 0){std::cerr << "RINEX clock SBAS satellites: " << cnt.sbas << std::endl;}
      }
      solver_GNSS.update_ephemeris_source(data);
      return true;
    }
#endif

#define option_apply(expr) \
data.gps.solver_options. expr; \
data.sbas.solver_options. expr
    if(value = runtime_opt_t::get_value(spec, "GNSS_elv_mask_deg", false)){
      if(dry_run){return true;}
      FloatT mask_deg(std::atof(value));
      std::cerr << "GNSS_elv_mask: reject when (elv) < " << mask_deg << " [deg]" << std::endl;
      option_apply(elevation_mask = deg2rad(mask_deg));
      return true;
    }
    if(value = runtime_opt_t::get_value(spec, "GNSS_delta_rho_mask", false)){
      if(dry_run){return true;}
      FloatT mask_meter(std::atof(value));
      std::cerr << "GNSS_delta_rho_mask: reject when (delta_rho) > " << mask_meter << " [m]" << std::endl;
      option_apply(residual_mask = mask_meter);
      return true;
    }
#undef option_apply

    if(value = runtime_opt_t::get_value(spec, "F10.7", false)){
      if(dry_run){return true;}
      FloatT f_10_7(std::atof(value));
      if((f_10_7 <= 0) || (f_10_7 > 1E3)){
        std::cerr << "(error!) Abnormal F10.7!" << value << std::endl;
        return false;
      }
      std::cerr << "F10.7: " << f_10_7 << std::endl;
      solver_GNSS.gps.ionospheric_ntcm_gl.f_10_7 = f_10_7;
      solver_GNSS.gps.ionospheric_correction.add(&solver_GNSS.gps.ionospheric_ntcm_gl);
      solver_GNSS.sbas.ionospheric_correction.add(&solver_GNSS.gps.ionospheric_ntcm_gl);
      return true;
    }

    if(value = runtime_opt_t::get_value(spec, "SBAS_IGP")){
      if(dry_run){return true;}
      if(runtime_opt_t::is_true(value)){
        std::cerr << "SBAS_IGP: on" << std::endl;
        solver_GNSS.gps.ionospheric_correction.add(&solver_GNSS.sbas.ionospheric_sbas);
        solver_GNSS.sbas.ionospheric_correction.add(&solver_GNSS.sbas.ionospheric_sbas);
      }else{
        std::cerr << "SBAS_IGP: off" << std::endl;
        solver_GNSS.gps.ionospheric_correction.remove(&solver_GNSS.sbas.ionospheric_sbas);
        solver_GNSS.sbas.ionospheric_correction.remove(&solver_GNSS.sbas.ionospheric_sbas);
      }
      return true;
    }

    /* --GNSS_with[out]=(system|[system:][+-]sat_id)
     *    --GNSS_without=GPS excludes all GPS satellites
     *    --GNSS_without=GPS:4, --GNSS_with=GPS:-4, or --GNSS_without=4 exclude GPS(4).
     */
    while(true){
      bool without(true);
      if(!(value = runtime_opt_t::get_value(spec, "GNSS_without", false))){
        without = false;
        if(!(value = runtime_opt_t::get_value(spec, "GNSS_with", false))){break;}
      }
      if(dry_run){return true;}

      typename system_t::type_t sys(system_t::Unknown);
      int sv_id(0);
      bool select_all(false);
      do{
        char sys_str[8];
        int tmp;
        if((tmp = std::sscanf(value, "%7[A-Z]:%i", sys_str, &sv_id)) >= 1){
          // Specific system (with optional satellite) selected
          // system string check
          sys = system_t::str2system(sys_str);
          if(select_all = (tmp == 1)){break;}
          if((sys == system_t::QZSS) && (std::abs(sv_id) <= 10)){
            // For QZSS, alternatively QZSS:1-10 available instead of 193-202
            sv_id = (sv_id < 0 ? -1 : 1) * (std::abs(sv_id) + 192);
          }
        }else if((tmp = std::atoi(value)) != 0){ // For QZSS, use 193-197
          // Specific satellite selected
          satellite_id_t id(std::abs(tmp));
          sys = id.type;
          sv_id = (tmp < 0 ? -1 : 1) * id.sv_id;
        }
      }while(false);

      if(sv_id < 0){ // check polarity
        sv_id *= -1;
        without = !without;
      }

      switch(sys){
        case system_t::GPS:
          select_all
              ? data.gps.solver_options.exclude_prn.set(without)
              : data.gps.solver_options.exclude_prn.set(sv_id, without);
          break;
        case system_t::SBAS:
          select_all
              ? data.sbas.solver_options.exclude_prn.set(without)
              : data.sbas.solver_options.exclude_prn.set(sv_id, without);
          break;
        default:
          std::cerr << "(error!) Unsupported satellite! [" << value << "]" << std::endl;
          return false;
      }

      std::cerr << "GNSS_" << (without ? "without" : "with") << ": " << system_t::system2str(sys);
      if(!select_all){std::cerr << "(" << sv_id << ")";}
      std::cerr << std::endl;
      return true;
    }

#if !defined(BUILD_WITHOUT_GNSS_MULTI_FREQUENCY)
    if(value = runtime_opt_t::get_value(spec, "GNSS_L2", true)){
      if(dry_run){return true;}
      bool use(runtime_opt_t::is_true(value));
      std::cerr << "GNSS_L2: " << (use ? "on" : "off") << std::endl;
      data.gps.solver_options.exclude_L2C = !use;
      return true;
    }
#endif

    return false;
  }

  struct pvt_printer_t {
    typedef typename solver_base_t::user_pvt_t pvt_t;
    const pvt_t &pvt;
    pvt_printer_t(const pvt_t &_pvt) : pvt(_pvt) {}
    static struct label_t {
      static std::ostream &print(std::ostream &out, const typename GPS_Solver_Base<FloatT>::user_pvt_t *){
        return out << "week"
            << ',' << "itow_rcv"
            << ',' << "receiver_clock_error_meter"
            << ',' << "longitude"
            << ',' << "latitude"
            << ',' << "height"
            << ',' << "gdop"
            << ',' << "pdop"
            << ',' << "hdop"
            << ',' << "vdop"
            << ',' << "tdop"
            << ',' << "v_north"
            << ',' << "v_east"
            << ',' << "v_down"
            << ',' << "receiver_clock_error_dot_ms"
            << ',' << "used_satellites"
            << ',' << "GPS_PRN(1-32)"
#if !defined(BUILD_WITHOUT_GNSS_MULTI_CONSTELLATION)
            << ',' << "SBAS_PRN(120-158)"
#endif
            ;
      }
      template <class PVT_BaseT>
      static std::ostream &print(std::ostream &out, const GPS_PVT_Debug<FloatT, PVT_BaseT> *){
        print(out, static_cast<const PVT_BaseT *>(0));
        for(int i(1); i <= 32; ++i){
          out << ',' << "range_residual(GPS:" << i << ')'
              << ',' << "weight(GPS:" << i << ')';
        }
#if !defined(BUILD_WITHOUT_GNSS_MULTI_CONSTELLATION)
        for(int i(120); i <= 158; ++i){
          out << ',' << "range_residual(SBAS:" << i << ')'
              << ',' << "weight(SBAS:" << i << ')';
        }
#endif
        return out;
      }
#if !defined(BUILD_WITHOUT_GNSS_RAIM)
      template <class PVT_BaseT>
      static std::ostream &print(std::ostream &out, const GPS_PVT_RAIM_LSR<FloatT, PVT_BaseT> *){
        return print(out, static_cast<const PVT_BaseT *>(0))
            << ',' << "wssr" << ',' << "wssr_sf" << ',' << "weight_max"
            << ',' << "slopeH_max" << ',' << "slopeH_max_PRN"
            << ',' << "slopeV_max" << ',' << "slopeV_max_PRN"
            << ',' << "wssr_FDE_min" << ',' << "wssr_FDE_min_PRN"
            << ',' << "wssr_FDE_2nd" << ',' << "wssr_FDE_2nd_PRN";
      }
#endif
      friend std::ostream &operator<<(std::ostream &out, const label_t &label){
        return print(out, static_cast<const pvt_t *>(0));
      }
    } label;

    struct mask_printer_t {
      const typename pvt_t::satellite_mask_t &mask;
      const int prn_lsb, prn_msb;
      mask_printer_t(
          const typename pvt_t::satellite_mask_t &_mask,
          const int &_prn_lsb, const int &_prn_msb)
          : mask(_mask), prn_lsb(_prn_lsb), prn_msb(_prn_msb) {}
      friend std::ostream &operator<<(std::ostream &out, const mask_printer_t &p){
        if(p.prn_msb < p.prn_lsb){return out;}
        int prn(p.prn_lsb % 8);
        prn = ((p.prn_msb - prn) / 8) * 8 + prn; // lower prn of each chunk is aligned to input prn_lsb modulo
#if defined(_MSC_VER) && (_MSC_VER >= 1600) && (_MSC_VER < 1700) // work around of C2668
#define print_bits(x) std::bitset<8>((unsigned long long)x)
#else
#define print_bits(x) std::bitset<8>(x)
#endif
        out << print_bits(p.mask.pattern(prn, p.prn_msb));
        for(int i((prn - p.prn_lsb) / 8); i > 0; --i){
          prn -= 8;
          out << '_' << print_bits(p.mask.pattern(prn, prn + 7));
        }
#undef print_bits
        return out;
      }
    };

    static std::ostream &print(std::ostream &out, const typename GPS_Solver_Base<FloatT>::user_pvt_t &src){
      out << src.receiver_time.week
          << ',' << src.receiver_time.seconds;
      if(src.position_solved()){
        out << ',' << src.receiver_error
            << ',' << rad2deg(src.user_position.llh.longitude())
            << ',' << rad2deg(src.user_position.llh.latitude())
            << ',' << src.user_position.llh.height()
            << ',' << src.dop.g
            << ',' << src.dop.p
            << ',' << src.dop.h
            << ',' << src.dop.v
            << ',' << src.dop.t;
      }else{
        out << ",,,,,,,,,";
      }
      if(src.velocity_solved()){
        out << ',' << src.user_velocity_enu.north()
            << ',' << src.user_velocity_enu.east()
            << ',' << -src.user_velocity_enu.up()
            << ',' << src.receiver_error_rate;
      }else{
        out << ",,,,";
      }
      if(src.position_solved()){
        out << ',' << src.used_satellites
            << ',' << mask_printer_t(src.used_satellite_mask, 1, 32)
#if !defined(BUILD_WITHOUT_GNSS_MULTI_CONSTELLATION)
            << ',' << mask_printer_t(src.used_satellite_mask, 120, 158) // SBAS
#endif
            ;
      }else{
        out << ",,"
#if !defined(BUILD_WITHOUT_GNSS_MULTI_CONSTELLATION)
            << ","
#endif
            ;
      }
      return out;
    }
    template <class PVT_BaseT>
    static std::ostream &print(std::ostream &out, const GPS_PVT_Debug<FloatT, PVT_BaseT> &src){
      print(out, static_cast<const PVT_BaseT &>(src));
      std::vector<int> used_sats(src.used_satellite_mask.indices_one());
      typename std::vector<int>::const_iterator it(used_sats.begin()), it_end(used_sats.end());
      if(!src.position_solved()){it = it_end;}
      static const struct {
        int prn_first, prn_last;
      } range[] = {
        {1, 32}, // GPS
#if !defined(BUILD_WITHOUT_GNSS_MULTI_CONSTELLATION)
        {120, 158}, // SBAS
#endif
      };
      unsigned int i_row(0);
      for(std::size_t i(0); i < sizeof(range) / sizeof(range[0]); ++i){
        for(int prn(range[i].prn_first); prn <= range[i].prn_last; ++prn){
          if((it == it_end) || (prn != *it)){
            out << ",,";
            continue;
          }
          out << ',' << src.delta_r(i_row, 0)
              << ',' << src.W(i_row, i_row);
          ++it;
          ++i_row;
        }
      }
      return out;
    }
#if !defined(BUILD_WITHOUT_GNSS_RAIM)
    template <class PVT_BaseT>
    static std::ostream &print(std::ostream &out, const GPS_PVT_RAIM_LSR<FloatT, PVT_BaseT> &src){
      print(out, static_cast<const PVT_BaseT &>(src));
      if(src.position_solved() && src.FD.valid){
        out << ',' << src.FD.wssr << ',' << src.FD.wssr_sf << ',' << src.FD.weight_max
            << ',' << src.FD.slope_HV[0].max << ',' << src.FD.slope_HV[0].prn
            << ',' << src.FD.slope_HV[1].max << ',' << src.FD.slope_HV[1].prn;
        if(src.FDE_min.valid){
          out << ',' << src.FDE_min.wssr << ',' << src.FDE_min.excluded;
          if(src.FDE_2nd.valid){
            out << ',' << src.FDE_2nd.wssr << ',' << src.FDE_2nd.excluded;
          }else{
            out << ",,";
          }
        }else{
          out << ",,,,";
        }
      }else{
        out << ",,,,,,,,,,,";
      }
      return out;
    }
#endif
    friend std::ostream &operator<<(std::ostream &out, const pvt_printer_t &p){
      return print(out, p.pvt);
    }
  };

  struct raw_data_printer_t {
    typedef GPS_RawData<FloatT, solver_base_t> raw_t;
    const raw_t &raw;
    typedef typename solver_t::xyz_t xyz_t;
    const xyz_t *xyz_base;
    raw_data_printer_t(const raw_t &_raw) : raw(_raw), xyz_base(NULL) {}
    raw_data_printer_t(const raw_t &_raw, const xyz_t &_xyz_base)
        : raw(_raw), xyz_base(&_xyz_base) {}
    static struct label_t {
      friend std::ostream &operator<<(std::ostream &out, const label_t &label){
        out << "clock_index";
        for(int i(1); i <= 32; ++i){
          out << ',' << "L1_range(GPS:" << i << ')'
#if !defined(BUILD_WITHOUT_GNSS_MULTI_FREQUENCY)
              << ',' << "L2_range(GPS:" << i << ')'
#endif
              << ',' << "L1_rate(GPS:" << i << ')'
              << ',' << "azimuth(GPS:" << i << ')'
              << ',' << "elevation(GPS:" << i << ')';
        }
#if !defined(BUILD_WITHOUT_GNSS_MULTI_CONSTELLATION)
        for(typename sbas_space_node_t::KnownSatellites::list_t::const_iterator it(
              sbas_space_node_t::KnownSatellites::name_ordered.begin());
            it != sbas_space_node_t::KnownSatellites::name_ordered.end();
            ++it){
          out << ',' << "L1_range(SBAS:" << (*it)->prn << ')'
              << ',' << "L1_rate(SBAS:" << (*it)->prn << ')'
              << ',' << "azimuth(SBAS:" << (*it)->prn << ')'
              << ',' << "elevation(SBAS:" << (*it)->prn << ')';
        }
#endif
        return out;
      }
    } label;

    struct cmd_t {
      int item;
      bool anchor;
      FloatT *(*conv)(FloatT &orig, const typename raw_t::measurement_t::mapped_type &k_v);
    };
    struct print_t {
      typedef typename raw_t::measurement_t msr_t;
      const msr_t &msr;
      const int &prn;
      const cmd_t *cmd;
      const int cmd_length;
      friend std::ostream &operator<<(std::ostream &out, const print_t &target){
        typename msr_t::const_iterator it(target.msr.find(target.prn));
        bool has_info(it != target.msr.end()), value_found(false);
        FloatT v;
        for(int i(0); i < target.cmd_length; ++i){
          if(has_info && (!value_found)
              && solver_t::find_value(it->second, target.cmd[i].item, v)){
            if((target.cmd[i].conv) && !(target.cmd[i].conv(v, it->second))){break;}
            out << v;
            value_found = true;
          }
          if(!target.cmd[i].anchor){continue;}
          out << ',';
          value_found = false;
        }
        return out;
      }
      static FloatT *l1_doppler2rate_with_freq(
          FloatT &orig, const typename msr_t::mapped_type &k_v){
        FloatT freq;
        if(solver_t::find_value(k_v, solver_t::measurement_items_t::L1_FREQUENCY, freq)){
          return &(orig *= -(gps_space_node_t::light_speed / freq));
        }
        return NULL;
      }
      static FloatT *l1_doppler2rate(
          FloatT &orig, const typename msr_t::mapped_type &k_v){
        return &(orig *= -gps_space_node_t::L1_WaveLength());
      }
    };
    template <int N>
    print_t operator()(const int &prn, const cmd_t (&cmd)[N]) const {
      print_t res = {raw.measurement, prn, cmd, N};
      return res;
    }
    struct print_az_el_t {
      bool valid;
      typename solver_t::enu_t enu_sat;
      friend std::ostream &operator<<(std::ostream &out, const print_az_el_t &target){
        if(!target.valid){return out << ',';}
        return out << rad2deg(target.enu_sat.azimuth())
            << ',' << rad2deg(target.enu_sat.elevation());
      }
    };
    print_az_el_t az_el(const int &prn) const {
      print_az_el_t res = {false};
      if(xyz_base){
        typename solver_t::satellite_t sat(
            raw.solver->select(prn).select_satellite(prn, raw.gpstime));
        if(sat.is_available()){
          res.valid = true;
          res.enu_sat = solver_t::enu_t::relative(sat.position(raw.gpstime), *xyz_base);
        }
      }
      return res;
    }
    friend std::ostream &operator<<(std::ostream &out, const raw_data_printer_t &p){
      typedef typename solver_t::measurement_items_t items_t;
      static const cmd_t cmd_gps[] = {
        {items_t::L1_PSEUDORANGE, true}, // range
#if !defined(BUILD_WITHOUT_GNSS_MULTI_FREQUENCY)
        {items_t::L2CM_PSEUDORANGE, false}, // range(L2C)
        {items_t::L2CL_PSEUDORANGE, true},
#endif
        {items_t::L1_RANGE_RATE,  false}, // rate
        {items_t::L1_DOPPLER,     false, print_t::l1_doppler2rate}, // fallback to using doppler
      };
      out << p.raw.clock_index;
      for(int i(1); i <= 32; ++i){
        out << ',' << p(i, cmd_gps) << ',' << p.az_el(i);
      }
#if !defined(BUILD_WITHOUT_GNSS_MULTI_CONSTELLATION)
      static const cmd_t cmd_sbas[] = {
        {items_t::L1_PSEUDORANGE, true}, // range
        {items_t::L1_RANGE_RATE,  false}, // rate
        {items_t::L1_DOPPLER,     false, print_t::l1_doppler2rate}, // fallback to using doppler
      };
      for(typename sbas_space_node_t::KnownSatellites::list_t::const_iterator
            it(sbas_space_node_t::KnownSatellites::name_ordered.begin()),
            it_end(sbas_space_node_t::KnownSatellites::name_ordered.end());
          it != it_end; ++it){
        out << ',' << p((*it)->prn, cmd_sbas) << ',' << p.az_el((*it)->prn);
      }
#endif
      return out;
    }
  };
};

template <class FloatT>
const typename GNSS_Receiver<FloatT>::system_t::type_str_t GNSS_Receiver<FloatT>::system_t::type_str_list[] = {
#define make_entry(x) {x, #x}
  make_entry(GPS), make_entry(SBAS), make_entry(QZSS),
  make_entry(GLONASS), make_entry(Galileo), make_entry(Beido),
  make_entry(Unknown),
#undef make_entry
};

template <class FloatT>
const int GNSS_Receiver<FloatT>::system_t::type_str_list_length
    = sizeof(type_str_list) / sizeof(type_str_list[0]);

template <class FloatT>
typename GNSS_Receiver<FloatT>::pvt_printer_t::label_t GNSS_Receiver<FloatT>::pvt_printer_t::label;

template <class FloatT>
typename GNSS_Receiver<FloatT>::raw_data_printer_t::label_t GNSS_Receiver<FloatT>::raw_data_printer_t::label;

#endif /* __GNSS_RECEIVER_H__ */
