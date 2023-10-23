#!/usr/bin/ruby

# U-blox file utilities

# Copyright (c) 2016, M.Naruoka (fenrir)
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

class UBX
  def initialize(io)
    @io = io
    @buf = []
  end
  def UBX.checksum(packet, range = 2..-3)
    ck_a, ck_b = [0, 0]
    packet[range].each{|b|
      ck_a += b
      ck_b += ck_a
    }
    ck_a &= 0xFF
    ck_b &= 0xFF
    [ck_a, ck_b]
  end
  def UBX.update_checksum(packet)
    packet[-2..-1] = checksum(packet)
    packet
  end
  def UBX.update_size(packet, size = nil)
    size ||= packet.size - 8
    size = size.divmod(0x100)
    packet[4] = size[1]
    packet[5] = size[0]
    packet
  end
  def UBX.update(packet)
    [:update_size, :update_checksum].inject(packet){|arg, f|
      UBX.send(f, arg)
    }
  end
  def read_packet
    while !@io.eof?
      if @buf.size < 8 then
        @buf += @io.read(8 - @buf.size).unpack('C*')
        return nil if @buf.size < 8
      end
      
      if @buf[0] != 0xB5 then
        @buf.shift
        next
      elsif @buf[1] != 0x62 then
        @buf = @buf[2..-1]
        next
      end
      
      len = (@buf[5] << 8) + @buf[4]
      if @buf.size < len + 8 then
        @buf += @io.read(len + 8 - @buf.size).unpack('C*')
        return nil if @buf.size < len + 8
      end
      
      ck_a, ck_b = UBX::checksum(@buf, 2..(len + 5))
      if (@buf[len + 6] != ck_a) || (@buf[len + 7] != ck_b) then
        @buf = @buf[2..-1]
        next
      end
      
      packet = @buf[0..(len + 7)]
      @buf = @buf[(len + 8)..-1]
      
      return packet
    end
    return nil
  end
  
  GNSS_ID = {
    :GPS => 0,
    :SBAS => 1,
    :Galileo => 2,
    :BeiDou => 3,
    :QZSS => 5,
    :GLONASS => 6,
  }
  
  SIGNAL_ID = {
    :GPS      => {:L1CA => 0, :L2CL => 3, :L2CM => 4},
    :SBAS     => {:L1CA => 0},
    :Galileo  => {:E1C => 0, :E1B => 1, :E5_bI => 5, :E5_bQ => 6},
    :BeiDou   => {:B1I_D1 => 0, :B1I_D2 => 1, :B2I_D1 => 2, :B2I_D2 => 3},
    :QZSS     => {:L1CA => 0, :L2CL => 4, :L2CM => 5},
    :GLONASS  => {:L1OF => 0, :L2OF => 2},
  }
  
  def UBX.svid(id, gnss = :GPS)
    gnss = GNSS_ID[gnss] if gnss.kind_of?(Symbol)
    case gnss
    when GNSS_ID[:GPS], GNSS_ID[:SBAS]
      id
    when GNSS_ID[:Galileo]
      id + 210
    when GNSS_ID[:Beido]
      (id < 6) ? (id + 158) : (id + 27)
    when GNSS_ID[:QZSS]
      id + 192
    when GNSS_ID[:GLONASS]
      id + 64
    else
      nil
    end
  end
  
  def UBX.gnss_svid(legacy_svid)
    case legacy_svid
    when 1..32;     [:GPS, legacy_svid]
    when 120..158;  [:SBAS, legacy_svid]
    when 211..246;  [:Galileo, legacy_svid - 210]
    when 159..163;  [:Beido, legacy_svid - 158]
    when 33..64;    [:Beido, legacy_svid - 27]
    #when 173..182 # IMES
    when 193..197;  [:QZSS, legacy_svid - 192]
    when 65..96;    [:GLONASS, legacy_svid - 64]
    when 255;       [:GLONASS, nil]
    else; nil
    end
  end
end
