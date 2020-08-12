#!/usr/bin/env ruby

# GPS Time

# Copyright (c) 2020, M.Naruoka (fenrir)
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

class GPSTime
  SEC_PER_WEEK = 7 * 24 * 60 * 60
  WEEK_PER_CYCLE = 1024
  ZERO = Time::gm(1980, 1, 6, 0, 0, 0)
  
  # based on https://ja.wikipedia.org/wiki/%E9%96%8F%E7%A7%92
  LEAP_SEC_LIST = [
    [Time::gm(1981, 7, 1, 0, 0, 0), 1],
    [Time::gm(1982, 7, 1, 0, 0, 0), 1],
    [Time::gm(1983, 7, 1, 0, 0, 0), 1],
    [Time::gm(1985, 7, 1, 0, 0, 0), 1],
    [Time::gm(1988, 1, 1, 0, 0, 0), 1],
    [Time::gm(1990, 1, 1, 0, 0, 0), 1],
    [Time::gm(1991, 1, 1, 0, 0, 0), 1],
    [Time::gm(1992, 7, 1, 0, 0, 0), 1],
    [Time::gm(1993, 7, 1, 0, 0, 0), 1],
    [Time::gm(1994, 7, 1, 0, 0, 0), 1],
    [Time::gm(1996, 1, 1, 0, 0, 0), 1],
    [Time::gm(1997, 7, 1, 0, 0, 0), 1],
    [Time::gm(1999, 1, 1, 0, 0, 0), 1],
    [Time::gm(2006, 1, 1, 0, 0, 0), 1],
    [Time::gm(2009, 1, 1, 0, 0, 0), 1],
    [Time::gm(2012, 7, 1, 0, 0, 0), 1],
    [Time::gm(2015, 7, 1, 0, 0, 0), 1],
    [Time::gm(2017, 1, 1, 0, 0, 0), 1],
  ]
  
  UTC2GPST_DELTA = proc{|list|
    delta_sum = 0
    list.collect{|utc, delta|
      [utc, delta_sum += delta]
    }.sort{|a, b| b[0] <=> a[0]} # Latest first
  }.call(LEAP_SEC_LIST)
  
  attr_accessor :week, :sec
  
  def GPSTime.time2gpssec(t)
    sec = t - ZERO
    UTC2GPST_DELTA.each{|check_t, delta| 
      return sec + delta if t > check_t
    }
    sec
  end
  
  def initialize(t = Time::now)
    case t
    when Array
      @seek, @sec = t[0..1]
      return
    when Numeric
      @sec = t
    when Time
      @sec = GPSTime::time2gpssec(t)
    when String
      require 'time'
      @sec = GPSTime::time2gpssec(Time::parse(t))
    else
      raise "Unsupported input: #{t}"
    end
    @week, @sec = @sec.divmod(SEC_PER_WEEK)
  end
  
  def to_a
    [@seek, @sec]
  end
  
  def cycle_week
    @week.divmod(WEEK_PER_CYCLE)
  end
  
  GPST2UTC_DELTA = UTC2GPST_DELTA.collect{|utc, delta|
    [utc - ZERO + delta, delta]
  }
  
  def utc
    sec = @week * SEC_PER_WEEK + @sec
    GPST2UTC_DELTA.each{|check_t, delta| 
      if sec > check_t then
        sec -= delta
        break
      end
    }
    ZERO + sec
  end
  
  def <=>(another)
    another = GPSTime::new(another) unless another.kind_of?(GPSTime)
    [@week, @sec] <=> [another.week, another.sec]
  end
end

if $0 == __FILE__ then
  options = {}
  ARGV.reject!{|arg|
    if arg =~ /--([^=]+)=?/ then
      options[$1.to_sym] = $' || true
      true
    else
      false
    end
  }
  
  target = ARGV.empty? ? Time::now : ARGV.join(' ')
  gpstime = GPSTime::new(target)
  
  $stderr.puts(<<-__STRING__)
Input: #{target}
GPS time: #{gpstime.week} weeks #{gpstime.sec} seconds (#{
  gpstime.cycle_week.zip(["cycles,", "weeks"]).flatten.join(' ')
}) 
UTC from GPS time: #{gpstime.utc} 
  __STRING__
end
