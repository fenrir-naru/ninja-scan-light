#!/usr/bin/ruby
# coding: cp932

# generating unified CSV from log.dat

# Copyright (c) 2019, M.Naruoka (fenrir)
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

class Unified_CSV
  OPTIONS_DEFAULT = {
    :tool_dirs => [
        File::dirname($0), 
        File::join(File::dirname($0), "build_GCC"),
        File::join(File::dirname($0), "build_VC", "Release"),],
    :page => [],
  }
  
  def initialize(options)
    @options = options
    @options[:bin] ||= proc{
      bin = "log_CSV.#{RUBY_PLATFORM =~ /mswin|mingw|bccwin/ ? :exe : :out}"  
      options[:tool_dirs].collect{|dir|
        path = File::join(dir, bin)
        File::exist?(path) ? path : nil
      }.compact[0]
    }.call
  end
  
  def generate(log_dat)
    $stderr.puts "Generating unified CSV of #{log_dat} with #{@options} ..."
    log_CSV_opts = (@options.keys - [:tool_dirs, :bin, :page]).collect{|k|
      "--#{k}#{@options[k] == true ? '' : "=#{@options[k]}"}"
    }
    data = @options[:page].collect{|page|
      log_CSV_args = [@options[:bin], "--page=#{page}"] + log_CSV_opts + [log_dat]
      range = "AF".include?(page) ? (1..-1) : (0..-1)
      items = IO::popen(log_CSV_args.join(' '), 'r').collect{|line|
        line.chomp.split(/ *, */)[range]
      }
      if @options[:calendar_time] # parse time and [0] will be time
        items.collect!{|values|
          [values[0..4].collect{|v| v.to_i} << values[5].to_f] + values[6..-1] 
        }
      else
        items.collect!{|values|
          [values[0].to_f] + values[1..-1]
        }
      end
      if "MP".include?(page) then # having index and fix time
        # determine interval
        sample_top = (items.size / 2).to_i
        t_i = items[sample_top..(sample_top + 200)].collect{|values|
          [values[0], values[1].to_i]
        }
        time_stamps = t_i.select{|t, i| i == 0}.transpose[0]
        t_minmax = time_stamps.minmax
        t_minmax.collect!{|t| Time::local(*(t[0..4] + (t[5] * 1E6).to_i.divmod(1_000_000)))} if @options[:calendar_time]
        t_interval_zero = (t_minmax[1] - t_minmax[0]) / (time_stamps.size - 1)
        i_minmax = t_i.transpose[1].minmax
        t_interval = (t_interval_zero / (i_minmax[1] - i_minmax[0] + 1)).round(2) # approximate every 10 ms
        
        # fix time
        fix_time = @options[:calendar_time] ? proc{|t, delta|
          t[5] = (t[5] + delta).round(2); next t if t[5] >= 0 # sec
          t[5] += 60; t[4] -= 1; next t if t[4] >= 0 # min
          t[4] += 60; t[3] -= 1; next t if t[3] >= 0 # hr
          t[3] += 24; t[2] -= 1; t # day (accept zeroth day)
        } : proc{|t, delta| (t + delta).round(2)}
        items.collect!{|values|
          [fix_time.call(values[0], t_interval * values[1].to_i)] + values[2..-1]
        }
      end
      items
    }
    template = data.collect{|items| [nil] * (items[0].size - 1)}
    data_indices = data.size.times.to_a
    
    # sort and output
    
    # t_sorted = [[t0, i2], [t1, i0], [t1, i3], [t2, i1], ...]
    t_sorted = data.collect.with_index{|items, i| [items[0][0], i]}.sort{|a, b| a[0] <=> b[0]}
    while !t_sorted.empty?
      #p t_sorted
      t_i = t_sorted.shift
      oldest = [t_i[1]]
      while !t_sorted.empty? and (t_i[0] == t_sorted[0][0]) # find same time stamp
        oldest << t_sorted.shift[1]
      end
      output = template.clone
      oldest.each{|i|
        output[i] = data[i].shift[1..-1]
        next if data[i].empty?
        # TODO check super jump
        t_sorted.insert(t_sorted.find_index{|t_i2|
          (data[i][0][0] <=> t_i2[0]) <= 0
        } || -1, [data[i][0][0], i])
      }
      $stdout.puts ([t_i[0]] + output).join(',')
    end
  end
end

if $0 == __FILE__ then

options = Unified_CSV::OPTIONS_DEFAULT.clone
ARGV.reject!{|arg|
  next false unless arg =~ /--([^=]+)=?/
  k, v = [$1.to_sym, $' == "" ? true : $']
  case k
  when :page
    options[k] << v.upcase[0]
  else
    options[k] = v
  end
  true
}

$stderr.print <<__TEXT__
Generate unified CSV by using log_CSV
Usage: #{__FILE__} [options] log.dat > log_unified.csv
[options] = mostly same as log_CSV ones represented by --page=[A|G|P|M]
__TEXT__

raise "log_CSV is not specified!" if ARGV.empty?

Unified_CSV::new(options).generate(ARGV.shift)

end