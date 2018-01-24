#!/usr/bin/ruby

$stderr.puts(<<__TEXT__)
Usage:
#{__FILE__} hex1 hex2 ... > *.cfg
or
echo hex1 hex2 ... | #{__FILE__} > *.cfg

__TEXT__

print (if ARGV.empty? then
  $stdin.read.split(/\s+/)
else
  ARGV
end).collect{|str|
  v = Integer(str, 16)
  raise "out of bounds => \"#{str}\"" if (v < 0) || (v > 0xFF)
  v
}.pack("C*")
