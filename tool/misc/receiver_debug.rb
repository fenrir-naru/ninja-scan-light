#!/usr/bin/ruby
# coding: cp932

=begin
Debug GPS receiver with Ruby via SWIG interface
=end

[
  File::join(File::dirname(__FILE__), '..', 'swig', 'build_SWIG'),
].each{|dir|
  $: << dir unless $:.include?(dir)
}
require 'GPS.so'

class GPS_Receiver
  OUTPUT_PVT_ITEMS = [[
    [:week, :itow_rcv, :year, :month, :mday, :hour, :min, :sec],
    proc{|pvt|
      [:week, :seconds, :c_tm].collect{|f| pvt.receiver_time.send(f)}.flatten
    }
  ]] + [[
    [:receiver_clock_error_meter, :longitude, :latitude, :height, :rel_E, :rel_N, :rel_U],
    proc{|pvt|
      next [nil] * 7 unless pvt.position_solved?
      [
        pvt.receiver_error,
        pvt.llh.lng / Math::PI * 180,
        pvt.llh.lat / Math::PI * 180,
        pvt.llh.alt,
      ] + (pvt.rel_ENU.to_a rescue [nil] * 3)
    } 
  ]] + [proc{
    labels = [:g, :p, :h, :v, :t].collect{|k| "#{k}dop".to_sym}
    [
      labels,
      proc{|pvt|
        next [nil] * 5 unless pvt.position_solved?
        labels.collect{|k| pvt.send(k)}
      }
    ]
  }.call] + [[
    [:v_north, :v_east, :v_down, :receiver_clock_error_dot_ms],
    proc{|pvt|
      next [nil] * 4 unless pvt.velocity_solved?
      [:north, :east, :down].collect{|k| pvt.velocity.send(k)} \
          + [pvt.receiver_error_rate] 
    }
  ]] + [
    [:used_satellites, proc{|pvt| pvt.used_satellites}],
    [:PRN, proc{|pvt|
      ("%32s"%[pvt.used_satellite_list.collect{|i|
        1 << (i - 1)
      }.inject(0){|res, v| res | v}.to_s(2)]).scan(/.{8}/).collect{|str|
        str.gsub(' ', '0')
      }.join('_')
    }],
  ] + [[
    (1..32).collect{|prn|
      [:range_residual, :weight, :azimuth, :elevation, :slopeH, :slopeV].collect{|str| "#{str}(#{prn})"}
    }.flatten,
    proc{|pvt|
      next ([nil] * 6 * 32) unless pvt.position_solved?
      sats = pvt.used_satellite_list
      r, w = [:delta_r, :W].collect{|f| pvt.send(f)}
      (1..32).collect{|i|
        next ([nil] * 6) unless i2 = sats.index(i)
        [r[i2, 0], w[i2, i2]] +
            [:azimuth, :elevation].collect{|f|
              pvt.send(f)[i] / Math::PI * 180
            } + [pvt.slopeH[i], pvt.slopeV[i]]
      }.flatten
    },
  ]] + [[
    [:wssr, :wssr_sf, :weight_max,
        :slopeH_max, :slopeH_max_PRN, :slopeH_max_elevation,
        :slopeV_max, :slopeV_max_PRN, :slopeV_max_elevation],
    proc{|pvt|
      next [nil] * 9 unless fd = pvt.fd
      el_deg = [4, 6].collect{|i| pvt.elevation[fd[i]] / Math::PI * 180}
      fd[0..4] + [el_deg[0]] + fd[5..6] + [el_deg[1]]
    }
  ]] + [[
    [:wssr_FDE_min, :wssr_FDE_min_PRN, :wssr_FDE_2nd, :wssr_FDE_2nd_PRN],
    proc{|pvt|
      [:fde_min, :fde_2nd].collect{|f|
        info = pvt.send(f)
        next ([nil] * 2) if (!info) || info.empty?
        [info[0], info[-3]] 
      }.flatten
    }
  ]]

  OUTPUT_MEAS_ITEMS = [[
    (1..32).collect{|prn|
      [:L1_range, :L1_rate].collect{|str| "#{str}(#{prn})"}
    }.flatten,
    proc{|meas|
      meas_hash = Hash[*(meas.collect{|prn, k, v| [[prn, k], v]}.flatten(1))]
      (1..32).collect{|prn|
        [:L1_PSEUDORANGE, [:L1_DOPPLER, GPS::SpaceNode.L1_WaveLength]].collect{|k, sf|
          meas_hash[[prn, GPS::Measurement.const_get(k)]] * (sf || 1) rescue nil
        }
      }
    }
  ]]
  
  def self.header
    (OUTPUT_PVT_ITEMS + OUTPUT_MEAS_ITEMS).transpose[0].flatten.join(',')
  end
    
  attr_accessor :solver
  attr_accessor :base_station

  def initialize(options = {})
    @solver = GPS::Solver::new
    @solver.hooks[:relative_property] = proc{|prn, rel_prop, rcv_e, t_arv, usr_pos, usr_vel|
      rel_prop[0] = 1 if rel_prop[0] > 0 # weight = 1
      rel_prop
    }
    options = options.reject{|k, v|
      case k
      when :weight
        case v.to_sym
        when :elevation # (same as underneath C++ library)
          @solver.hooks[:relative_property] = proc{|prn, rel_prop, rcv_e, t_arv, usr_pos, usr_vel|
            if rel_prop[0] > 0 then
              elv = Coordinate::ENU::relative_rel(
                  Coordinate::XYZ::new(*rel_prop[4..6]), usr_pos).elevation
              rel_prop[0] = (Math::sin(elv)/0.8)**2
            end
            rel_prop
          }
          next true
        when :identical # same as default
          next true
        end
      when :base_station
        crd, sys = v.split(/ *, */).collect.with_index{|item, i|
          case item
          when /^([\+-]?\d+\.?\d*)([XYZNEDU]?)$/ # ex) meter[X], degree[N]
            [$1.to_f, ($2 + "XY?"[i])[0]]
          when /^([\+-]?\d+)_(?:(\d+)_(\d+\.?\d*)|(\d+\.?\d*))([NE])$/ # ex) deg_min_secN
            [$1.to_f + ($2 || $4).to_f / 60 + ($3 || 0).to_f / 3600, $5]
          else
            raise "Unknown coordinate spec.: #{item}"
          end
        }.transpose
        raise "Unknown base station: #{v}" if crd.size != 3
        @base_station = case (sys = sys.join.to_sym)
        when :XYZ, :XY?
          Coordinate::XYZ::new(*crd)
        when :NED, :ENU, :NE?, :EN? # :NE? => :NEU, :EN? => :ENU
          (0..1).each{|i| crd[i] *= (Math::PI / 180)}
          ([:NED, :NE?].include?(sys) ?
              Coordinate::LLH::new(crd[0], crd[1], crd[2] * (:NED == sys ? -1 : 1)) :
              Coordinate::LLH::new(crd[1], crd[0], crd[2])).xyz
        else
          raise "Unknown coordinate system: #{sys}"
        end
        $stderr.puts "Base station (LLH): #{
          llh = @base_station.llh.to_a
          llh[0..1].collect{|rad| rad / Math::PI * 180} + [llh[2]]
        }"
        next true
      end
      false
    }
    raise "Unknown receiver options: #{options.inspect}" unless options.empty?
    proc{|opt|
      opt.elevation_mask = 0.0 / 180 * Math::PI # 0 deg
      opt.residual_mask = 1E4 # 10 km
    }.call(@solver.gps_options)
  end

  GPS::Measurement.class_eval{
    proc{
      key2sym = []
      GPS::Measurement.constants.each{|k|
        i = GPS::Measurement.const_get(k)
        key2sym[i] = k if i.kind_of?(Integer)
      }
      define_method(:to_a2){
        to_a.collect{|prn, k, v| [prn, key2sym[k] || k, v]}
      }
      define_method(:to_hash2){
        Hash[*(to_hash.collect{|prn, k_v|
          [prn, Hash[*(k_v.collect{|k, v| [key2sym[k] || k, v]}.flatten(1))]]
        }.flatten(1))]
      }
    }.call
    alias_method(:add_orig, :add)
    define_method(:add){|prn, key, value|
      add_orig(prn, key.kind_of?(Symbol) ? GPS::Measurement.const_get(key) : key, value)
    }
  }

  def run(meas, t_meas, ref_pos = @base_station)
=begin
    meas.to_a.collect{|prn, k, v| prn}.uniq.each{|prn|
      eph = @solver.gps_space_node.ephemeris(prn)
      $stderr.puts "XYZ(PRN:#{prn}): #{eph.constellation(t_meas)[0].to_a} (iodc: #{eph.iodc}, iode: #{eph.iode})"
    }
=end

    #@solver.gps_space_node.update_all_ephemeris(t_meas) # internally called in the following solver.solve
    pvt = @solver.solve(meas, t_meas)
    pvt.define_singleton_method(:rel_ENU){
      Coordinate::ENU::relative(xyz, ref_pos)
    } if (ref_pos && pvt.position_solved?)
    pvt.define_singleton_method(:to_s){
      (OUTPUT_PVT_ITEMS.transpose[1].collect{|task|
        task.call(pvt)
      } + OUTPUT_MEAS_ITEMS.transpose[1].collect{|task|
        task.call(meas)
      }).flatten.join(',')
    }
    pvt
  end

  GPS::PVT.class_eval{
    define_method(:post_solution){|target|
      sats, az, el = proc{|g|
        self.used_satellite_list.collect.with_index{|prn, i|
          # G_enu is measured in the direction from satellite to user positions
          [prn,
              Math::atan2(-g[i, 0], -g[i, 1]),
              Math::asin(-g[i, 2])]
        }.transpose
      }.call(self.G_enu) rescue [[], [], []]
      [[:@azimuth, az], [:@elevation, el]].each{|k, values|
        self.instance_variable_set(k, Hash[*(sats.zip(values).flatten(1))])
      }
      [:@slopeH, :@slopeV] \
          .zip((self.slope_HV_enu.to_a.transpose rescue [nil, nil])) \
          .each{|k, values|
        self.instance_variable_set(k,
            Hash[*(values ? sats.zip(values).flatten(1) : [])])
      }
      instance_variable_get(target)
    }
    [:azimuth, :elevation, :slopeH, :slopeV].each{|k|
      eval("define_method(:#{k}){@#{k} || self.post_solution(:@#{k})}")
    }
  }
  
  proc{
    eph_list = Hash[*(1..32).collect{|prn|
      eph = GPS::Ephemeris::new
      eph.svid = prn
      [prn, eph]
    }.flatten(1)]
    define_method(:register_ephemeris){|t_meas, prn, bcast_data|
      next unless eph = eph_list[prn]
      sn = @solver.gps_space_node
      subframe, iodc_or_iode = eph.parse(bcast_data)
      if iodc_or_iode < 0 then
        begin
          sn.update_iono_utc(
              GPS::Ionospheric_UTC_Parameters::parse(bcast_data))
          [:alpha, :beta].each{|k|
            $stderr.puts "Iono #{k}: #{sn.iono_utc.send(k)}"
          } if false
        rescue
        end
        next
      end
      if t_meas and eph.consistent? then
        eph.WN = ((t_meas.week / 1024).to_i * 1024) + (eph.WN % 1024)
        sn.register_ephemeris(prn, eph)
        eph.invalidate
      end
    }
  }.call
  
  def parse_ubx(ubx_fname, &b)
    $stderr.print "Reading UBX file (%s) "%[ubx_fname]
    require_relative 'ubx'
  
    ubx = UBX::new(open(ubx_fname))  
    ubx_kind = Hash::new(0)
    
    after_run = b || proc{|pvt| puts pvt.to_s}
    
    t_meas = nil
    ubx.each_packet.with_index(1){|packet, i|
      $stderr.print '.' if i % 1000 == 0
      ubx_kind[packet[2..3]] += 1
      case packet[2..3]
      when [0x02, 0x10] # RXM-RAW
        msec, week = [[0, 4, "V"], [4, 2, "v"]].collect{|offset, len, str|
          packet.slice(6 + offset, len).pack("C*").unpack(str)[0]
        }
        t_meas = GPS::Time::new(week, msec.to_f / 1000)
        meas = GPS::Measurement::new
        packet[6 + 6].times{|i|
          loader = proc{|offset, len, str|
            ary = packet.slice(6 + offset + (i * 24), len)
            str ? ary.pack("C*").unpack(str)[0] : ary
          }
          prn = loader.call(28, 1)[0]
          {
            :L1_PSEUDORANGE => [16, 8, "E"],
            :L1_DOPPLER => [24, 4, "e"],
            :L1_CARRIER_PHASE => [8, 8, "E"],
            :L1_SIGNAL_STRENGTH_dBHz => [30, 1, "c"],
          }.each{|k, prop|
            meas.add(prn, k, loader.call(*prop))
          }
          # bit 0 of RINEX LLI (loss of lock indicator) shows lost lock
          # between previous and current observation, which maps negative lock seconds
          meas.add(prn, :L1_LOCK_SEC,
              (packet[6 + 31 + (i * 24)] & 0x01 == 0x01) ? -1 : 0)
        }
        after_run.call(run(meas, t_meas), [meas, t_meas])
      when [0x02, 0x15] # RXM-RAWX
        sec, week = [[0, 8, "E"], [8, 2, "v"]].collect{|offset, len, str|
          packet.slice(6 + offset, len).pack("C*").unpack(str)[0]
        }
        t_meas = GPS::Time::new(week, sec)
        meas = GPS::Measurement::new
        packet[6 + 11].times{|i|
          loader = proc{|offset, len, str, post|
            v = packet.slice(6 + offset + (i * 32), len)
            v = str ? v.pack("C*").unpack(str)[0] : v
            v = post.call(v) if post
            v
          }
          next unless (gnss = loader.call(36, 1)[0]) == 0
          svid = loader.call(37, 1)[0]
          trk_stat = loader.call(46, 1)[0]
          {
            :L1_PSEUDORANGE => [16, 8, "E", proc{|v| (trk_stat & 0x1 == 0x1) ? v : nil}],
            :L1_PSEUDORANGE_SIGMA => [43, 1, nil, proc{|v|
              (trk_stat & 0x1 == 0x1) ? (1E-2 * (v[0] & 0xF)) : nil
            }],
            :L1_DOPPLER => [32, 4, "e"],
            :L1_DOPPLER_SIGMA => [45, 1, nil, proc{|v| 2E-3 * (v[0] & 0xF)}],
            :L1_CARRIER_PHASE => [24, 8, "E", proc{|v| (trk_stat & 0x2 == 0x2) ? v : nil}],
            :L1_CARRIER_PHASE_SIGMA => [44, 1, nil, proc{|v|
              (trk_stat & 0x2 == 0x2) ? (0.004 * (v[0] & 0xF)) : nil
            }],
            :L1_SIGNAL_STRENGTH_dBHz => [42, 1, "C"],
            :L1_LOCK_SEC => [40, 2, "v", proc{|v| 1E-3 * v}],
          }.each{|k, prop|
            next unless v = loader.call(*prop)
            meas.add(svid, k, v)
          }
        }
        after_run.call(run(meas, t_meas), [meas, t_meas])
      when [0x02, 0x11] # RXM-SFRB
        register_ephemeris(
            t_meas,
            packet[6 + 1],
            packet.slice(6 + 2, 40).each_slice(4).collect{|v|
              (v.pack("C*").unpack("V")[0] & 0xFFFFFF) << 6
            })
      when [0x02, 0x13] # RXM-SFRBX
        next unless (gnss = packet[6]) == 0
        register_ephemeris(
            t_meas,
            packet[6 + 1],
            packet.slice(6 + 8, 4 * packet[6 + 4]).each_slice(4).collect{|v|
              v.pack("C*").unpack("V")[0]
            })
      end
    }
    $stderr.puts ", found packets are %s"%[ubx_kind.inspect]
  end
  
  def parse_rinex_nav(fname)
    $stderr.puts "Read RINEX NAV file (%s): %d items."%[
        fname, @solver.gps_space_node.read(fname)]
  end
  
  def parse_rinex_obs(fname, &b)
    after_run = b || proc{|pvt| puts pvt.to_s}
    $stderr.print "Reading RINEX observation file (%s)"%[fname]
    types = nil
    count = 0
    GPS::RINEX_Observation::read(fname){|item|
      $stderr.print '.' if (count += 1) % 1000 == 0
      t_meas = item[:time]
      
      meas = GPS::Measurement::new
      types ||= (item[:meas_types]['G'] || item[:meas_types][' ']).collect.with_index{|type_, i|
        case type_
        when "C1", "C1C"
          [i, :L1_PSEUDORANGE]
        when "L1", "L1C"
          [i, :L1_CARRIER_PHASE]
        when "D1", "D1C"
          [i, :L1_DOPPLER]
        when "S1", "S1C"
          [i, :L1_SIGNAL_STRENGTH_dBHz]
        else
          nil 
        end
      }.compact
      item[:meas].each{|k, v|
        sys, prn = k
        next unless sys == 'G' # GPS only
        types.each{|i, type_|
          meas.add(prn, type_, v[i][0]) if v[i]
        }
      }
      after_run.call(run(meas, t_meas), [meas, t_meas])
    }
    $stderr.puts ", %d epochs."%[count] 
  end
end

if __FILE__ == $0 then
  options = {}

  # check options
  ARGV.reject!{|arg|
    next false unless arg =~ /^--([^=]+)=?/
    options[$1.to_sym] = $'
    true
  }

  # Check file existence
  ARGV.each{|arg|
    raise "File not found: #{arg}" unless File::exist?(arg)
  }

  rcv = GPS_Receiver::new(options)
  
  puts GPS_Receiver::header

  # parse RINEX NAV
  ARGV.reject!{|arg|
    next false unless arg =~ /\.\d{2}n$/
    rcv.parse_rinex_nav(arg)
  }
  
  # other files
  ARGV.each{|arg|
    case arg
    when /\.ubx$/
      rcv.parse_ubx(arg)
    when /\.\d{2}o$/
      rcv.parse_rinex_obs(arg)
    end
  }
end
