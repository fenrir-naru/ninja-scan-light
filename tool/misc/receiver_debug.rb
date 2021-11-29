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

OUTPUT_PVT_ITEMS = [
  [:week, proc{|pvt| pvt.receiver_time.week}],
  [:itow_rcv, proc{|pvt| pvt.receiver_time.seconds}],
] + [[
  [:receiver_clock_error_meter, :longitude, :latitude, :height],
  proc{|pvt|
    next [nil] * 4 unless pvt.position_solved? 
    [
      pvt.receiver_error,
      pvt.llh.lng / Math::PI * 180,
      pvt.llh.lat / Math::PI * 180,
      pvt.llh.alt,
    ]
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
    [:range_residual, :weight, :azimuth, :elevation].collect{|str| "#{str}(#{prn})"}
  }.flatten,
  proc{|pvt|
    next ([nil] * 4 * 32) unless pvt.position_solved?
    sats = pvt.used_satellite_list
    r, w = [:delta_r, :W].collect{|f| pvt.send(f)}
    (1..32).collect{|i|
      next ([nil] * 4) unless i2 = sats.index(i)
      [r[i2, 0], w[i2, i2]] +
          [:azimuth, :elevation].collect{|f|
            pvt.send(f)[i] / Math::PI * 180
          }
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

puts (OUTPUT_PVT_ITEMS + OUTPUT_MEAS_ITEMS).transpose[0].flatten.join(',')

solver = GPS::Solver::new
solver.hooks[:relative_property] = proc{|prn, rel_prop, rcv_e, t_arv, usr_ps, us_vel|
  rel_prop[0] = 1 if rel_prop[0] > 0 # weight = 1
  rel_prop
}
sn = solver.gps_space_node
proc{|opt|
  opt.elevation_mask = 0.0 / 180 * Math::PI # 0 deg
  opt.residual_mask = 1E4 # 10 km
}.call(solver.gps_options)

run_solver = proc{|meas, t_meas|
  #$stderr.puts "Measurement time: #{t_meas.to_a} (a.k.a #{"%d/%d/%d %d:%d:%d UTC"%[*t_meas.c_tm]})"
  sn.update_all_ephemeris(t_meas)
  meas.to_a.collect{|prn, k, v| prn}.uniq.each{|prn|
    eph = sn.ephemeris(prn)
    $stderr.puts "XYZ(PRN:#{prn}): #{eph.constellation(t_meas)[0].to_a} (iodc: #{eph.iodc}, iode: #{eph.iode})"
  } if false 

  pvt = solver.solve(meas, t_meas)
  sats, az, el = proc{|g|
    pvt.used_satellite_list.collect.with_index{|prn, i|
      # G_enu is measured in the direction from satellite to user positions
      [prn,
          Math::atan2(-g[i, 0], -g[i, 1]),
          Math::asin(-g[i, 2])]
    }.transpose
  }.call(pvt.G_enu) rescue [[], [], []]
  [[:azimuth, az], [:elevation, el]].each{|f, values|
    pvt.define_singleton_method(f){Hash[*(sats.zip(values).flatten(1))]}
  }

  puts (OUTPUT_PVT_ITEMS.transpose[1].collect{|task|
    task.call(pvt)
  } + OUTPUT_MEAS_ITEMS.transpose[1].collect{|task|
    task.call(meas)
  }).flatten.join(',')
}

register_ephemeris = proc{
  eph_list = Hash[*(1..32).collect{|prn|
    eph = GPS::Ephemeris::new
    eph.svid = prn
    [prn, eph]
  }.flatten(1)]
  proc{|t_meas, prn, bcast_data|
    next unless eph = eph_list[prn]
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

parse_ubx = proc{|ubx_fname|
  $stderr.print "Reading UBX file (%s) "%[ubx_fname]
  require_relative 'ubx'

  ubx = UBX::new(open(ubx_fname))  
  ubx_kind = Hash::new(0)
  
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
        }.each{|k, prop|
          meas.add(prn, GPS::Measurement.const_get(k), loader.call(*prop))
        }
      }
      run_solver.call(meas, t_meas)
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
        }.each{|k, prop|
          next unless v = loader.call(*prop)
          meas.add(svid, GPS::Measurement.const_get(k), v)
        }
      }
      run_solver.call(meas, t_meas)
    when [0x02, 0x11] # RXM-SFRB
      register_ephemeris.call(
          t_meas,
          packet[6 + 1],
          packet.slice(6 + 2, 40).each_slice(4).collect{|v|
            (v.pack("C*").unpack("V")[0] & 0xFFFFFF) << 6
          })
    when [0x02, 0x13] # RXM-SFRBX
      next unless (gnss = packet[6]) == 0
      register_ephemeris.call(
          t_meas,
          packet[6 + 1],
          packet.slice(6 + 8, 4 * packet[6 + 4]).each_slice(4).collect{|v|
            v.pack("C*").unpack("V")[0]
          })
    end
  }
  $stderr.puts ", found packets are %s"%[ubx_kind.inspect]
}

parse_rinex_obs = proc{|fname|
  $stderr.print "Reading RINEX observation file (%s)"%[fname]
  types = nil
  count = 0
  GPS::RINEX_Observation::read(fname){|item|
    $stderr.print '.' if (count += 1) % 1000 == 0
    t_meas = item[:time]
    sn.update_all_ephemeris(t_meas)
    
    meas = GPS::Measurement::new
    types ||= (item[:meas_types]['G'] || item[:meas_types][' ']).collect.with_index{|type_, i|
      case type_
      when "C1", "C1C"
        [i, GPS::Measurement::L1_PSEUDORANGE]
      when "D1", "D1C"
        [i, GPS::Measurement::L1_DOPPLER]
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
    run_solver.call(meas, t_meas)
  }
  $stderr.puts ", %d epochs."%[count] 
}

# check options
ARGV.reject!{|arg|
  next false unless arg =~ /^--[^=]+=?/
  true
}
ARGV.each{|arg|
  raise "File not found: #{arg}" unless File::exist?(arg)
}

# parse RINEX NAV
ARGV.reject!{|arg|
  next false unless arg =~ /\.\d{2}n$/
  $stderr.puts "Read RINEX NAV file (%s): %d items."%[arg, sn.read(arg)]
}

# other files
ARGV.each{|arg|
  case arg
  when /\.ubx$/
    parse_ubx.call(arg)
  when /\.\d{2}o$/
    parse_rinex_obs.call(arg)
  end
}
