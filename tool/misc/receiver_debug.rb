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

require_relative 'ubx'

ubx_fname = File::join(File::dirname(__FILE__), '..', 'test_log', '150616_bicycle.ubx')
ubx = UBX::new(open(ubx_fname))

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
    [:range_residual, :weight].collect{|str| "#{str}(#{prn})"} # TODO :azimuth, :elevation
  }.flatten,
  proc{|pvt|
    next ([nil] * 64) unless pvt.position_solved?
    sats = pvt.used_satellite_list
    r, w = [:delta_r, :W].collect{|f| pvt.send(f)}
    (1..32).collect{|i|
      next [nil, nil] unless i2 = sats.index(i)
      [r[i2, 0], w[i2, i2]]
    }.flatten
  },
]] + [[
  [:wssr, :wssr_sf, :weight_max, :slopeH_max, :slopeH_max_PRN, :slopeV_max, :slopeV_max_PRN],
  proc{|pvt| pvt.fd || ([nil] * 7)}
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

C1C = GPS::Measurement::L1_PSEUDORANGE
D1C = GPS::Measurement::L1_DOPPLER
R1C = GPS::Measurement::L1_RANGE_RATE

OUTPUT_MEAS_ITEMS = [[
  (1..32).collect{|prn|
    [:L1_range, :L1_rate].collect{|str| "#{str}(#{prn})"}
  }.flatten,
  proc{|meas|
    meas_hash = Hash[*(meas.collect{|prn, k, v| [[prn, k], v]}.flatten(1))]
    (1..32).collect{|prn|
      [
        meas_hash[[prn, C1C]], 
        meas_hash[[prn, R1C]] || ((GPS::SpaceNode.L1_WaveLength * meas_hash[[prn, D1C]]) rescue nil),
      ]
    }
  }
]]

puts (OUTPUT_PVT_ITEMS + OUTPUT_MEAS_ITEMS).transpose[0].flatten.join(',')

solver = GPS::Solver::new
sn = solver.gps_space_node

run_solver = proc{|meas, t_meas|
  #$stderr.puts "Measurement time: #{t_meas.to_a} (a.k.a #{"%d/%d/%d %d:%d:%d UTC"%[*t_meas.c_tm]})"
  sn.update_all_ephemeris(t_meas)
  meas.to_a.collect{|prn, k, v| prn}.uniq.each{|prn|
    eph = sn.ephemeris(prn)
    $stderr.puts "XYZ(PRN:#{prn}): #{eph.constellation(t_meas)[0].to_a} (iodc: #{eph.iodc}, iode: #{eph.iode})"
  } if false 
  pvt = solver.solve(meas, t_meas)
  puts (OUTPUT_PVT_ITEMS.transpose[1].collect{|task|
    task.call(pvt)
  } + OUTPUT_MEAS_ITEMS.transpose[1].collect{|task|
    task.call(meas)
  }).flatten.join(',')
}

eph_list = Hash[*(1..32).collect{|prn|
  eph = GPS::Ephemeris::new
  eph.svid = prn
  [prn, eph]
}.flatten(1)]
ubx_kind = Hash::new(0)

t_meas = nil
while packet = ubx.read_packet
  ubx_kind[packet[2..3]] += 1
  case packet[2..3]
  when [0x02, 0x10] # RXM-RAW
    msec, week = [[0, 4, "V"], [4, 2, "v"]].collect{|offset, len, str|
      packet.slice(6 + offset, len).pack("C*").unpack(str)[0]
    }
    t_meas = GPS::Time::new(week, msec.to_f / 1000)
    meas = GPS::Measurement::new
    packet[6 + 6].times{|i|
      prange, doppler, prn = [[16, 8, "d"], [24, 4, "f"], [28, 1, "C"]].collect{|offset, len, str|
        packet.slice(6 + offset + (i * 24), len).pack("C*").unpack(str)[0]
      }
      meas.add(prn, C1C, prange)
      meas.add(prn, D1C, doppler)
    }
    run_solver.call(meas, t_meas)
  when [0x02, 0x15] # RXM-RAWX
  when [0x02, 0x11] # RXM-SFRB
    prn = packet[6 + 1]
    next unless eph = eph_list[prn]
    bcast_data = packet.slice(6 + 2, 40).each_slice(4).collect{|v|
      (v.pack("C*").unpack("V")[0] & 0xFFFFFF) << 6
    }
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
  when [0x02, 0x13] # RXM-SFRBX
  end
end
$stderr.puts ubx_kind.inspect

