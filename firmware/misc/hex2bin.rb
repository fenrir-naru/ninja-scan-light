#!/usr/bin/ruby

$stderr.puts(<<__TEXT__)
Usage:
#{__FILE__} hex1 hex2 ... > *.cfg
(hexN can be combination of '-f file_which_has_hex_string_separated_by_spaces')
or
echo hex1 hex2 ... | #{__FILE__} > *.cfg
__TEXT__

def hex2num(hex)
  v = Integer(hex, 16)
  raise "out of bounds => \"#{hex}\"" if (v < 0) || (v > 0xFF)
  v
end

is_file = false
print (if ARGV.empty? then
  $stdin.read.split(/\s+/)
else
  ARGV
end).collect{|str|
  if is_file then
    is_file = false
    next open(str).collect{|line|
      line.scrub.scan(/^[^#]*/)[0].split(/\s+/).collect{|str|
        hex2num(str)
      }
    }
  elsif str == '-f' then
    is_file = true
    next nil
  end
  hex2num(str)
}.compact.flatten.pack("C*")
