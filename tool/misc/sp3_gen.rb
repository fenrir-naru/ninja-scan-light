#!/usr/bin/ruby
# coding: cp932

require 'stringio'

def parse(str)
  res = str.each_line.collect{|line|
    line.chomp!
    col, desc, exp, fmt = [[0, 20], [20, 19], [39, 20], 59..-1].collect{|range|
      line[*([range].flatten)]
    }
    #$stderr.puts [col, desc, exp, fmt].inspect
    next nil unless col =~ /(\d+)(?:\-(\d+))?/
    idx_begin = (Integer($1) - 1)
    idx_end = $2 ? (Integer($2) - 1) : idx_begin
    k = desc.sub(/^\s*/, '').sub(/\s*$/, '') \
        .sub(/^(\d+)/, "_\\1").gsub(/[\(\)\.]/, '').gsub(/[ \-\/\*]/, '_') \
        .downcase.to_sym
    next nil if k == :unused
    fmt = fmt.sub(/^\s*/, '').sub(/\s*$/, '').gsub(/ /, '_')
    [idx_begin, idx_end - idx_begin + 1, k, fmt]
  }.compact
  res.define_singleton_method(:add_last!){|num_times|
    (num_times || 1).times{
      col = self[-1].clone
      col[0] += col[1]
      self << col
    }
    self
  }
  res.define_singleton_method(:variables){
    list = {}
    self.each{|idx, len, k, fmt|
      if list[k] then
        raise unless list[k][1] == fmt
        list[k][0] += 1
      else
        list[k] = [1, fmt]
      end
    }
    list
  }
  res.define_singleton_method(:c_str){|opt|
    opt ||= {}
    proto = "struct #{opt[:var_type]}"
    next "#{proto};" if opt[:prototype]

    sio = StringIO::new
    sio.puts "#{proto}{"
    self.variables.each{|k, len_fmt|
      len1 = len_fmt[0]
      type_, len2 = case len_fmt[1]
      when "A1,I2.2"; [:int]
      when /^A(\d+)/; [:char, $1.to_i]
      when /^I/; [:int]
      when /^F/; [:double]
      end
      sio.puts "  #{type_} #{k}#{"[#{len1}]" if len1 > 1}#{"[#{len2}]" if len2};"
    }
    sio.puts "};"
    sio.string
  }
  res.define_singleton_method(:c_list){|opt|
    opt ||= {}
    proto = "#{opt[:var_name]}[#{self.size}]"
    next "#{proto};" if opt[:prototype]

    array_keys = self.variables.select{|k, v| v[0] > 1}.keys
    array_idx = Hash[*(array_keys.zip([0] * array_keys.size).flatten)]
    sio = StringIO::new
    sio.puts "#{proto} = {"
    self.each{|idx, len, k, fmt|
      type_, misc = case fmt
      when "A1,I2.2"; [:sat]
      when /^A/; [:C]
      when /^I/; [:I]
      when /^F\d+\.(\d+)/; [:E, $1.to_i]
      end
      if i = array_idx[k] then
        array_idx[k] += 1
        k = "#{k}[#{i}]"
      end
      sio.puts "  GEN_#{type_}(#{([idx, len, opt[:var_type], k] + [misc]).flatten.compact.join(', ')}),"
    }
    sio.puts "};"
    sio.string
  }
  res
end


# In the followings, description is based on ver.D

SP3 = {}
SP3[:L1] = parse(<<-__TEXT__)
Columns 1-2         Version Symbol     #c                  A2
Column  3           Pos or Vel Flag    P or V              A1
Columns 4-7         Year Start         2001                I4
Column  8           Unused             _                   blank
Columns 9-10        Month Start        _8                  I2
Column  11          Unused             _                   blank 
Columns 12-13       Day of Month St    _8                  I2
Column  14          Unused             _                   blank
Columns 15-16       Hour Start         _0                  I2
Column  17          Unused             _                   blank
Columns 18-19       Minute Start       _0                  I2
Column  20          Unused             _                   blank 
Columns 21-31       Second Start       _0.00000000         F11.8
Column  32          Unused             _                   blank
Columns 33-39       Number of Epochs   ____192             I7
Column  40          Unused             _                   blank
Columns 41-45       Data Used          ____d               A5
Column  46          Unused             _                   blank
Columns 47-51       Coordinate Sys     ITR97               A5
Column  52          Unused             _                   blank
Columns 53-55       Orbit Type         FIT                 A3
Colusmn 56          Unused             _                   blank
Columns 57-60       Agency             _NGS                A4
__TEXT__

SP3[:L2] = parse(<<-__TEXT__)
Columns 1-2         Symbols            ##                  A2
Column  3           Unused             _                   blank
Columns 4-7         GPS Week           1126                I4
Column  8           Unused             _                   blank
Columns 9-23        Seconds of Week    259200.00000000     F15.8
Column  24          Unused             _                   blank
Columns 25-38       Epoch Interval     __900.00000000      F14.8
Column  39          Unused             _                   blank
Columns 40-44       Mod Jul Day St     52129               I5
Column  45          Unused             _                   blank
Columns 46-60       Fractional Day     0.0000000000000     F15.13
__TEXT__

SP3[:L3_11] = proc{
  res = parse((<<-__TEXT__).lines[0, 5].join)
Columns 1-2         Symbols            +_                  A2
Column  3-4         Unused             __                  2 blanks
Columns 5-6         Number of Sats     26                  I2
Column  7-9         Unused             ___                 3 blanks
Columns 10-12       Sat #1 Id          G01                 A1,I2.2
Columns 13-15       Sat #2 Id          G02                 A1,I2.2
       *
       *
       *
Columns 58-60       Sat #17 Id         G21                 A1,I2.2
__TEXT__
  res[-1][2] = :sat_id
  res.add_last!(16)
}.call

SP3[:L12_20] = proc{
  res = parse((<<-__TEXT__).lines[0, 3].join)
Columns 1-2         Symbols            ++                  A2
Columns 3-9         Unused             _______             7 blanks
Columns 10-12       Sat #1 Accuracy    __7                 I3
Columns 13-15       Sat #2 Accuracy    __8                 I3
       *
       *
       *
Columns 58-60       Sat #17 Accuracy   __9                 I3
__TEXT__
  res[-1][2] = :sat_accuracy
  res.add_last!(16)
}.call

SP3[:L21_22] = parse(<<-__TEXT__)
Columns 1-2         Symbols            %c                  A2
Column  3           Unused             _                   blank
Columns 4-5         File Type          G_                  A2
Column  6           Unused             _                   blank
Columns 7-8         2 characters       cc                  A2
Column  9           Unused             _                   blank
Columns 10-12       Time System        GPS                 A3 
Column  13          Unused             _                   blank
Columns 14-16       3 characters       ccc                 A3 
Column  17          Unused             _                   blank
Columns 18-21       4 characters       cccc                A4
Column  22          Unused             _                   blank
Columns 23-26       4 characters       cccc                A4
Column  27          Unused             _                   blank
Columns 28-31       4 characters       cccc                A4
Column  32          Unused             _                   blank
Columns 33-36       4 characters       cccc                A4
Column  37          Unused             _                   blank
Columns 38-42       5 characters       ccccc               A5
Column  43          Unused             _                   blank
Columns 44-48       5 characters       ccccc               A5
Column  49          Unused             _                   blank
Columns 50-54       5 characters       ccccc               A5
Column  55          Unused             _                   blank
Columns 56-60       5 characters       ccccc               A5
__TEXT__

SP3[:L23_24] = parse(<<-__TEXT__)
Columns 1-2         Symbols            %f                  A2
Column  3           Unused             _                   blank
Columns 4-13        Base for Pos/Vel   _1.2500000          F10.7
                    (mm or 10**-4 mm/sec)
Column  14          Unused             _                   blank
Columns 15-26       Base for Clk/Rate  _1.025000000        F12.9
                    (psec or 10**-4 psec/sec)
Column  27          Unused             _                   blank
Columns 28-41       14-column float    _0.00000000000      F14.11
Column  42          Unused             _                   blank
Columns 43-60       18-column float    _0.000000000000000  F18.15
__TEXT__

SP3[:L25_26] = parse(<<-__TEXT__)
Columns 1-2         Symbols            %i                  A2
Column  3           Unused             _                   blank
Columns 4-7         4-column int       ___0                I4
Column  8           Unused             _                   blank
Columns 9-12        4-column int       ___0                I4
Column  13          Unused             _                   blank
Columns 14-17       4-column int       ___0                I4
Column  18          Unused             _                   blank
Columns 19-22       4-column int       ___0                I4
Column  23          Unused             _                   blank
Columns 24-29       6-column int       _____0              I6
Column  30          Unused             _                   blank
Columns 31-36       6-column int       _____0              I6
Column  37          Unused             _                   blank
Columns 38-43       6-column int       _____0              I6
Column  44          Unused             _                   blank
Columns 45-50       6-column int       _____0              I6
Column  51          Unused             _                   blank
Columns 52-60       9-column int       ________0           I9
__TEXT__

SP3[:COMMENT] = parse(<<-__TEXT__)
Columns 1-2         Symbols            /*                  A2
Column  3           Unused             _                   blank
Columns 4-60        Comment            CC...CC             A57
__TEXT__

SP3[:EPOCH] = parse(<<-__TEXT__)
Columns 1-2         Symbols            *_                  A2
Column  3           Unused             _                   blank
Columns 4-7         Year Start         2001                I4
Column  8           Unused             _                   blank
Columns 9-10        Month Start        _8                  I2
Column  11          Unused             _                   blank
Columns 12-13       Day of Month St    _8                  I2
Column  14          Unused             _                   blank
Columns 15-16       Hour Start         _0                  I2
Column  17          Unused             _                   blank
Columns 18-19       Minute Start       _0                  I2
Column  20          Unused             _                   blank
Columns 21-31       Second Start       _0.00000000         F11.8
__TEXT__

SP3[:POSITION_CLOCK] = parse(<<-__TEXT__)
Column  1           Symbol             P                   A1
Columns 2-4         Vehicle Id.        G01                 A1,I2.2
Columns 5-18        x-coordinate(km)   _-11044.805800      F14.6 
Columns 19-32       y-coordinate(km)   _-10475.672350      F14.6
Columns 33-46       z-coordinate(km)   __21929.418200      F14.6
Columns 47-60       clock (microsec)   ____189.163300      F14.6
Column  61          Unused             _                   blank
Columns 62-63       x-sdev (b**n mm)   18                  I2
Column  64          Unused             _                   blank
Columns 65-66       y-sdev (b**n mm)   18                  I2
Column  67          Unused             _                   blank
Columns 68-69       z-sdev (b**n mm)   18                  I2
Column  70          Unused             _                   blank
Columns 71-73       c-sdev (b**n psec) 219                 I3
Column  74          Unused             _                   blank
Column  75          Clock Event Flag   E                   A1
Column  76          Clock Pred. Flag   P                   A1
Columns 77-78       Unused             __                  2 blanks
Column  79          Maneuver Flag      M                   A1
Column  80          Orbit Pred. Flag   P                   A1
__TEXT__

SP3[:POSITION_CLOCK_CORRELATION] = parse(<<-__TEXT__)
Columns 1-2         Symbols            EP                  A2
Columns 3-4         Unused             __                  2 blanks
Columns 5-8         x-sdev (mm)        __55                I4
Column  9           Unused             _                   blank
Columns 10-13       y-sdev (mm)        __55                I4
Column  14          Unused             _                   blank
Columns 15-18       z-sdev (mm)        __55                I4
Column  19          Unused             _                   blank
Columns 20-26       clk-sdev (psec)    ____222             I7
Column  27          Unused             _                   blank
Columns 28-35       xy-correlation     _1234567            I8
Column  36          Unused             _                   blank
Columns 37-44       xz-correlation     -1234567            I8
Column  45          Unused             _                   blank
Columns 46-53       xc-correlation     _5999999            I8
Column  54          Unused             _                   blank
Columns 55-62       yz-correlation     _____-30            I8
Column  63          Unused             _                   blank
Columns 64-71       yc-correlation     ______21            I8
Column  72          Unused             _                   blank
Columns 73-80       zc-correlation     -1230000            I8
__TEXT__

SP3[:VELOCITY_RATE] = parse(<<-__TEXT__)
Column 1            Symbol             V                   A1
Columns 2-4         Vehicle Id.        G01                 A1,I2.2
Columns 5-18        x-velocity(dm/s)   __20298.880364      F14.6
Columns 19-32       y-velocity(dm/s)   _-18462.044804      F14.6
Columns 33-46       z-velocity(dm/s)   ___1381.387685      F14.6
Columns 47-60       clock rate-chg     _____-4.534317      F14.6
Column  61          Unused             _                   blank
Columns 62-63       xvel-sdev          14                  I2
                    (b**n 10**-4 mm/sec)
Column  64          Unused             _                   blank
Columns 65-66       yvel-sdev          14                  I2
                    (b**n 10**-4 mm/sec)
Column  67          Unused             _                   blank
Columns 68-69       zvel-sdev          14                  I2
                    (b**n 10**-4 mm/sec)
Column  70          Unused             _                   blank
Columns 71-73       clkrate-sdev       191                 I3
                    (b**n 10**-4 psec/sec)
Columns 74-80       Unused             _______             7 blanks
__TEXT__

SP3[:VELOCITY_RATE_CORRELATION] = parse(<<-__TEXT__)
Columns 1-2         Symbols            EV                  A2
Columns 3-4         Unused             __                  2 blanks
Columns 5-8         xvel-sdev          __22                I4
                    (10**-4 mm/sec)
Column  9           Unused             _                   blank
Columns 10-13       yvel-sdev          __22                I4
                    (10**-4 mm/sec)
Column  14          Unused             _                   blank
Columns 15-18       zvel-sdev          __22                I4
                    (10**-4 mm/sec)
Column  19          Unused             _                   blank
Columns 20-26       clkrate-sdev       ____111             I7
                    (10**-4 psec/sec)
Column  27          Unused             _                   blank
Columns 28-35       xy-correlation     _1234567            I8
Column  36          Unused             _                   blank
Columns 37-44       xz-correlation     _1234567            I8
Column  45          Unused             _                   blank
Columns 46-53       xc-correlation     _1234567            I8
Column  54          Unused             _                   blank
Columns 55-62       yz-correlation     _1234567            I8
Column  63          Unused             _                   blank
Columns 64-71       yc-correlation     _1234567            I8
Column  72          Unused             _                   blank
Columns 73-80       zc-correlation     _1234567            I8
__TEXT__

if $0 == __FILE__ then
  SP3.collect{|k, v|
    opt = {
      :var_type => "#{k.to_s.downcase}_t",
      :var_name => "#{k.to_s.downcase}_items",
    }
    [v.c_str(opt), v.c_str(opt.merge({:prototype => true})),
      v.c_list(opt.merge({:prototype => true})), v.c_list(opt)]
  }.transpose.flatten.each{|item|
    puts item
  }
  exit(0)
end