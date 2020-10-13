#!/usr/bin/ruby

require 'serialport' # gem install serialport
require 'io/console'

opt = {
  :port => {
    :bps => 9600,
    :nbits => 8,
    :stopb => 1,
    :parity => SerialPort::NONE,
  },
  :begin_sector => 0,
  :sectors => 1024 * 1024, # 512MB
  :end_sector => -1, # Auto
  :each_sectors => 0x100,
}

if ARGV.size < 1
  $stderr.puts "Usage: #{$0} Serial_port_number [options]"
  exit(1)
end

# set port number
opt[:port][:num] = proc{
  res = ARGV.shift
  begin
    res = Integer(res)
    res = "/dev/ttyS#{res}" if ((`uname -a` =~ /cygwin/i) rescue false)
  rescue
  end
  res
}.call

ARGV.reject!{|arg|
  next false if arg !~ /--([^=]+)=?/
  k, v = [$1.to_sym, $' == "" ? true : $']
  case opt[k]
  when Integer
    v = if (k.to_s =~ /sector/) && (v =~ /(\d+)([GMK])/) then
      $1.to_i * {:G => 0x40000000, :M => 0x100000, :K => 0x400}[$2.to_sym]
    else
      Integer(v) rescue eval(v)
    end
  when true, false
    v = (v == "true") if v.kind_of?(String)
  end
  opt[k] = v
  true
}
$stderr.puts "Error! Unknown args: #{ARGV}" unless ARGV.empty?

opt[:port][:name] = proc{
  next opt[:port][:num] unless opt[:port][:num].kind_of?(Integer)
  RUBY_PLATFORM =~ /win32/ ? "\\\\.\\COM#{opt[:port][:num]}" : "tty#{opt[:port][:num]}"
}.call

opt[:dst_fname] ||= File::basename("#{opt[:port][:name]}_#{Time::now.strftime("%Y%d%m_%H%M%S")}.bin")

open_proc = proc{
  io = SerialPort.new(*([:num, :bps, :nbits, :stopb, :parity].collect{|k| opt[:port][k]}))
  io.read_timeout = 100
  io.fsync
  #io.readpartial(0x20000) rescue nil
  io
}
  
sp = open_proc.call
$stderr.puts "#{opt[:port][:name]} opened."

read_prop = proc{|k|
  sp.write(k)
  sp.fsync
  next $&.to_i if sp.gets.force_encoding(Encoding::ASCII_8BIT) =~ /\d+/
  $stderr.puts "Unknown property(#{k})!"
  exit(-1)
}

property = {
  :count => read_prop.call("count"),
  :size => read_prop.call("size"),
}
$stderr.puts "Storage property: #{property.inspect}"

read_block = proc{|sector_start, sectors|
  sectors ||= 1
  sp.write([sector_start, sectors].join(' '))
  sp.fsync
  raise if sp.gets !~ /READ\((\d+),(\d+)\)/
  raise if ($1.to_i != sector_start) || ($2.to_i != sectors)
  bytes = property[:size] * sectors
  res = ""
  err_cnt = 0
  begin
    rx = sp.read(bytes)
    res += rx
    raise if (bytes -= rx.size) > 0
  rescue
    raise if (err_cnt += 1) > 8
    $stderr.print 'e'
    retry
  end
  res
}

$stderr.puts "Save to #{opt[:dst_fname]}."
open(opt[:dst_fname], "a+" + (/mswin|mingw/ =~ RUBY_PLATFORM ? 'b' : '')){|dst|
  bs, es, sectors = [:begin_sector, :end_sector, :sectors].collect{|k| opt[k]}
  es = bs + sectors if es < 0
  es = [property[:count], es].min
  sectors = es - bs
  each_sectors = opt[:each_sectors]
  (bs...es).step(each_sectors){|i|
    i_rel = i - bs
    $stderr.print "Reading #{sprintf("%8d (%8d of %8d, %5.1f%%)", i, i_rel, sectors, 100.0 * i_rel / sectors)}... "
    begin
      dst.write(read_block.call(i, [each_sectors, es - i].min))
    rescue
      sp.close
      sleep(1000)
      sp = open_proc.call
      $stderr.print 'r'
      retry
    end
    $stderr.puts " #{Time::now.strftime("%T")}"
  }
  $stderr.puts "done."
}

sp.close