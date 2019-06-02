#!/usr/bin/ruby

require 'serialport' # gem install serialport

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
  res = ARGV[0]
  begin
    res = Integer(res)
    res = "/dev/ttyS#{res}" if ((`uname -a` =~ /cygwin/i) rescue false)
  rescue
  end
  res
}.call

ARGV[1..-1].each{|arg|
  next false if arg !~ /--([^=]+)=?/
  k, v = [$1.to_sym, $' == "" ? true : $']
  case opt[k]
  when Fixnum
    v = Integer(v) rescue eval(v)
  when true, false
    v = (v == "true") if v.kind_of?(String)
  end
  opt[k] = v
  true
}

opt[:port][:name] = proc{
  next opt[:port][:num] unless opt[:port][:num].kind_of?(Integer)
  RUBY_PLATFORM =~ /win32/ ? "\\\\.\\COM#{opt[:port][:num]}" : "tty#{opt[:port][:num]}"
}.call

opt[:dst_fname] ||= File::basename("#{opt[:port][:name]}_#{Time::now.strftime("%Y%d%m_%H%M%S")}.bin")

sp = SerialPort.new(*([:num, :bps, :nbits, :stopb, :parity].collect{|k| opt[:port][k]}))
sp.read_timeout = 100
$stderr.puts "#{opt[:port][:name]} opened."

read_prop = proc{|k|
  sp.write(k)
  next $&.to_i if sp.gets =~ /\d+/
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
  raise if sp.gets !~ /READ\((\d+),(\d+)\)/
  raise if ($1.to_i != sector_start) || ($2.to_i != sectors)
  res = sp.read(property[:size] * sectors)
  raise if res.size != (property[:size] * sectors)
  res
}

$stderr.puts "Save to #{opt[:dst_fname]}."
open(opt[:dst_fname], 'a+'){|dst|
  bs, es, sectors = [:begin_sector, :end_sector, :sectors].collect{|k| opt[k]}
  es = bs + sectors if es < 0
  es = [property[:count], es].min
  sectors = es - bs
  each_sectors = opt[:each_sectors]
  (bs...es).step(each_sectors){|i|
    i_rel = i - bs
    $stderr.puts "Reading #{sprintf("%8d (%8d of %8d, %5.1f%%)", i, i_rel, sectors, 100.0 * i_rel / sectors)} #{Time::now.strftime("%T")} ..."
    dst.write(read_block.call(i, [each_sectors, es - i].min))
  }
  $stderr.puts "done."
}

sp.close