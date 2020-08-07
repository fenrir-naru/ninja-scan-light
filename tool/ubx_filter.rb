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
  def initialize(io, opt = {})
    @ubx = UBX::new(io)
    @prop = {:itow => nil, :week => nil}
    @gates = opt[:gates] || []
    @gates << proc{|packet, prop| packet} # Anchor to accept all packets
  end
  
  ITOW_PARSER = proc{
    res = {}
    {
      0x01 => [0x01, 0x02, 0x03, 0x04, 0x06, 0x08, 0x11, 0x12, 0x20, 0x21, 0x22, 0x30, 0x31, 0x32],
      0x02 => [0x10, 0x20],
    }.collect{|class_, id_list|
      parser = proc{|packet| (1E-3 * packet[6..9].unpack('V')[0])}
      id_list.each{|id_|
        res[[class_, id_]] = parser
      }
    }
    {
      0x02 => [0x15],
    }.collect{|class_, id_list|
      parser = proc{|packet| packet[6..13].unpack('E')[0]}
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
        parser = proc{|packet| packet[idx_start..(idx_start + 1)].unpack('v')[0]}
        class_id_list.collect{|class_id| [class_id, parser]}
      }.flatten(2))]
  
  def run
    while true
      break unless (packet = @ubx.read_packet)      
      @prop[:class_id] = packet[2..3]
      @prop[:packet] = packet.pack('C*')
      @prop[:itow] = (ITOW_PARSER[packet[2..3]].call(@prop[:packet]) rescue @prop[:itow])
      @prop[:week] = (WEEK_PARSER[packet[2..3]].call(@prop[:packet]) rescue @prop[:week])
      next unless filtered_bin = @gates.inject(@prop[:packet]){|arg, gate|
        break nil unless arg
        gate.call(arg, @prop)
      }
      return filtered_bin
    end
    nil
  end
end

if $0 == __FILE__ then

$stderr.puts <<-__STRING__
UBX filter
  Usage: #{__FILE__} [--option_key[=option_value]] [original.ubx, otherwise stdin] > filtered.ubx
__STRING__

options = {
  :out => nil,
  :gates => [], # default is empty, which is identical to accept all packets
}
ARGV.reject{|arg|
  if arg =~ /--([^=]+)=?/ then
    k, v = [$1.to_sym, $']
    options[k] = v
    true
  else
    false
  end
}

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
}.call, options)

dest = if options[:out] then
  $stderr.puts "#{File::exist?(options[:out]) ? 'Appending' : 'Writing'} to #{options[:out]} ..."
  open(options[:out], 'wb+')
else
  $stderr.puts "Writing to $stdout ..."
  STDOUT.binmode
  $stdout
end

while packet = filter.run
  dest << packet
end

end
