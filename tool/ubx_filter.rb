#!/usr/bin/ruby

# U-blox filter

# Copyright (c) 2020 M.Naruoka (fenrir)
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
#
# - Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# - Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# - Neither the name of the naruoka.org nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
# BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
# OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

UTIL_DIR = File::join(File::dirname(__FILE__), 'misc')
$: << UTIL_DIR unless $:.include?(UTIL_DIR)
require 'ubx'

class UBX_Filter
  FILTERS = {
    :pass_all => proc{|packet, prop| packet},
    :drop_all => proc{|packet, prop| nil},
    :drop_measurement => proc{|drop_satellites|
      drop_satellites = drop_satellites.collect{|item|
        next item unless item.kind_of?(Array) # PRN only
        gnss = item[0]
        if gnss.kind_of?(Integer)
          gnss = UBX::GNSS_ID.invert[gnss]
        else
          item[0] = UBX::GNSS_ID[gnss] # Symbol => int
        end
        item[2] = UBX::SIGNAL_ID[gnss][item[2]] if item[2] && item[2].kind_of?(Symbol) # Symbol => int
        item
      }.collect{|item|
        # Handle special keyword :all
        if item == :all then # global :all
          Class::new{def ==(another); true; end}::new
        elsif item.kind_of?(Array) && item[1] == :all  # :all for each GNSS
          Class::new{GNSS = item[0]; def ==(another); another[0] == GNSS; end}::new
        else
          item
        end
      }
      proc{|packet, prop|
        idx_measurements = nil
        len_per_meas = nil
        dropped = case prop[:class_id]
            when [0x02, 0x10] # RXM-RAW
              idx_measurements = 6 + 6
              len_per_meas = 24
              packet[idx_measurements].times.collect{|i|
                next nil unless drop_satellites.include?(packet[6 + 28 + (i * len_per_meas)]) # check PRN
                (6 + 8 + (i * len_per_meas))
              }.compact
            when [0x02, 0x15] # RXM-RAWX
              idx_measurements = 6 + 11
              len_per_meas = 32
              packet[idx_measurements].times.collect{|i|
                gnss_sv_sig_freq = packet[6 + 36 + (i * len_per_meas), 4] # check gnssID, svID, signalID, freqID
                next nil unless (drop_satellites.include?(gnss_sv_sig_freq[0..1]) ||
                    drop_satellites.include?(gnss_sv_sig_freq[0..2]) ||
                    drop_satellites.include?(gnss_sv_sig_freq))
                (6 + 16 + (i * len_per_meas))
              }.compact
            else
              []
            end
        next packet if dropped.empty?
        dropped.reverse.each{|idx| # Remove information in reverse order
          packet.slice!(idx, len_per_meas)
        }
        packet[idx_measurements] -= dropped.size
        UBX::update(packet)
      }
    },
    :drop_ubx => proc{|class_id_list|
      any = Class::new{def ==(another); true; end}::new
      class_id_list.collect!{|item|
        case item
        when "all"; any
        else
          res = Integer(item)
          (res >= 0x100) ? res.divmod(0x100) : res
        end
      }
      proc{|packet, prop|
        next nil if class_id_list.include?(prop[:class_id]) \
            || class_id_list.include?(prop[:class_id][0])
        packet
      }
    },
  }
  
  def initialize(io, opt = {})
    @ubx = UBX::new(io)
    @prop = {
      :itow => nil, :week => nil, 
      :gates => opt[:gates] || [FILTERS[:pass_all]] # default: accept all packets
    }
  end
  
  ITOW_PARSER = proc{
    res = {}
    {
      0x01 => [0x01, 0x02, 0x03, 0x04, 0x06, 0x08, 0x11, 0x12, 0x20, 0x21, 0x22, 0x30, 0x31, 0x32],
      0x02 => [0x10, 0x20],
    }.collect{|class_, id_list|
      parser = proc{|packet| (1E-3 * packet[6..9].pack("C*").unpack('V')[0])}
      id_list.each{|id_|
        res[[class_, id_]] = parser
      }
    }
    {
      0x02 => [0x15],
    }.collect{|class_, id_list|
      parser = proc{|packet| packet[6..13].pack("C*").unpack('E')[0]}
      id_list.each{|id_|
        res[[class_, id_]] = parser
      }
    }
    res
  }.call
  
  WEEK_PARSER = Hash[*({
        10 => [[0x02, 0x10]],
        14 => [[0x01, 0x20], [0x02, 0x15]],
      }.collect{|idx_start, class_id_list|
        parser = proc{|packet| packet[idx_start..(idx_start + 1)].pack("C*").unpack('v')[0]}
        class_id_list.collect{|class_id| [class_id, parser]}
      }.flatten(2))]
  
  def run(&b)
    while true
      break unless (@prop[:packet] = @ubx.read_packet)
      @prop[:class_id] = @prop[:packet][2..3]
      @prop[:itow] = (ITOW_PARSER[@prop[:class_id]].call(@prop[:packet]) rescue @prop[:itow])
      @prop[:week] = (WEEK_PARSER[@prop[:class_id]].call(@prop[:packet]) rescue @prop[:week])
      b.call(@prop) if b
      return "" unless filtered = @prop[:gates].inject(@prop[:packet]){|packet, gate|
        break nil unless packet
        gate.call(packet, @prop)
      }
      return filtered.pack("C*")
    end
    nil
  end
end

if $0 == __FILE__ then

$stderr.puts <<-__STRING__
UBX filter
  Usage: #{__FILE__} [--option_key[=option_value]] [original.ubx, otherwise stdin] > filtered.ubx
    --cmdfile=cmdfile.txt is the most useful option.
    Example cmdfile.txt content is the following.

      # The default filter is to pass all packets
      drop,GPS:01 # (text after # is comment) always drop GPS 1
      drop,GPS:04,GPS:05 # always drop GPS 4, 5 (multiple satellites in a line)
      drop,QZSS:all # always drop any QZSS
      event,1000,drop,GPS:02 
          # after GPS time 1000[s] (any week) (i.e., GPS_sec > 1000), dropping GPS:02 is activated
          # Warning: event lines must be sorted in time order
          # Note: GPS 1,4,5, and any QZSS has still been dropped
          # Note2: If this filter is intended to be active no less than 1000 (i.e, >= 1000),
          # pleas use smaller seconds such as 999.9.
      event,2000,drop,GPS:03
          # after GPS time 2000[s] (any week), dropping GPS:03 is activated
          # In addition, all previous event commands are cleared, 
          # which means stop dropping GPS:02.
          # Note: GPS 1,4,5, and any QZSS has still been dropped
      event,3000,pass,all
          # after GPS time 3000[s] (any week), 
          # pass all except for GPS 1,4,5, and any QZSS 
      event,2100:2000,drop,GPS:03 
          # start dropping GPS 3 after GPS time 2100[week] 20000[s]

      Selection with ubx packet type is performed with drop_ubx, the below is example;
      drop_ubx,all # drop All packets
      drop_ubx,0x01 # drop All NAV packets
      drop_ubx,0x0102,0x0112 # drop NAV-POSLLH,VELNED packets

    Note: For old ubx, just PRN number (without satellite system) like "1" (<-instead of "GPS:01") works well. 
__STRING__

options = {
  :out => nil, 
  :cmd => [],
}
ARGV.reject!{|arg|
  if arg =~ /--([^=]+)=?/ then
    k, v = [$1.to_sym, $']
    case k
    when :cmd
      act, *specs = v.split(/ *, */)
      options[:cmd] << [act.to_sym, specs]
    when :drop, :drop_ubx, :pass, :event
      options[:cmd] << [k, v.split(/ *, */)]
    when :cmdfile
      open(v, 'r').each{|line|
        line = line.sub(/^\s*/, '').sub(/\s*$/, '').sub(/#.+$/, '') # text followed by '#' means comments
        next if line =~ /^\s*$/ # skip of empty line
        act, *specs = line.split(/ *, */)
        options[:cmd] << [act.to_sym, specs]
      }
    when :out
      options[k] = (v == "" ? true : v)
    else
      raise "Unknown option: #{arg}"
    end
    true
  else
    false
  end
}

gates = []
scenario = nil

proc{ # Build filter elements
  parse_sat = proc{|spec|
    spec =~ /^ *(?:([A-Za-z]+):?)?(\d+|all)/ # each satellite, (GNSS:)sat and special keyword, "all"
    gnss, sat = [$1, ($2 == 'all') ? :all : $2.to_i]
    next sat unless gnss
    raise "Unknown satellites: #{spec}" unless gnss = {
      'G' => :GPS, 'S' => :SBAS, 'E' => :Galileo, 'B' => :Beisou, 'Q' => :QZSS, 'R' => :GLONASS,
      'GPS' => :GPS, 'SBAS' => :SBAS, 'GALILEO' => :Galileo, 
      'BEIDOU' => :Beisou, 'QZSS' => :QZSS, 'GLONASS' => :GLONASS,
    }[gnss.upcase]
    [gnss, sat]
  }
  
  make_gates = proc{|act, specs|
    case act
    when :drop
      [UBX_Filter::FILTERS[:drop_measurement].call(specs.collect{|spec| parse_sat.call(spec)})]
    when :drop_ubx
      [UBX_Filter::FILTERS[:drop_ubx].call(specs)]
    #when :pass # ignore, because default is to accept all packets
    else
      []
    end
  }
  
  events = []
  options[:cmd].each{|act, specs|
    case act
    when :event
      t = case specs[0]
      when /(\d+:)?(\d+\.?\d+)/ # GPSTime == (week:)sec
        $1 ? [$1.to_i, $2.to_f] : $2.to_f
      else # Other time spec
        GPSTime::new(specs[0]).to_a
      end
      events << [t, *make_gates.call(specs[1].to_sym, specs[2..-1])]
    else
      gates += make_gates.call(act, specs)
    end
  }
  $stderr.puts events.inspect

  generate_scenario = proc{|event|
    next nil unless event
    cmp = event[0].kind_of?(Array) \
        ? proc{|prop|
          delta_week = event[0][0] - prop[:week]
          next false if delta_week < 0
          delta_week == 0 ? (event[0][1] > prop[:itow]) : false
        } \
        : proc{|prop| event[0] > prop[:itow]}
    proc{|prop|
      next if (cmp.call(prop) rescue true)
      $stderr.puts "Check point, GPS Time #{[:week, :itow].collect{|k| prop[k]}.join(':')} ..."
      prop[:gates] = event[1..-1] + gates
      scenario = generate_scenario.call(events.shift)
    }
  }
  scenario = generate_scenario.call(events.shift)
}.call

gates << UBX_Filter::FILTERS[:pass_all] # default; accept all packets

filter = UBX_Filter::new(proc{
  src = ARGV.shift
  case src
  when nil, '-'
    $stderr.puts "Reading from $stdin ..."
    STDIN.binmode
    $stdin
  else
    $stderr.puts "Reading from #{src} ..."
    open(src, 'rb')
  end
}.call, {:gates => gates})

dest = if options[:out] then
  $stderr.puts "#{File::exist?(options[:out]) ? 'Appending' : 'Writing'} to #{options[:out]} ..."
  open(options[:out], 'wb+')
else
  $stderr.puts "Writing to $stdout ..."
  STDOUT.binmode
  $stdout
end

while packet = filter.run(&scenario)
  dest << packet
end

end
