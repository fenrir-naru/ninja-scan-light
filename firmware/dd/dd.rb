#!/usr/bin/ruby

require 'serialport' # gem install serialport

opt = {
  :port => {
    :bps => 9600,
    :nbits => 8,
    :stopb => 1,
  },
  :begin_sector => 0,
  :end_sector => 1024 * 1024, # 512MB
  :each_sectors => 0x100,
}

ARGV.reject!{|arg|
  next false if arg !~ /--([^=]+)=?/
  k, v = [$1.to_sym, $' == "" ? true : $']
  case opt[k]
  when Fixnum
    v = Integer(v)
  when true, false
    v = (v == "true") if v.kind_of?(String)
  end
  opt[k] = v
  true
}

# set port number
if ARGV.size < 1
  $stderr.puts "Usage: ruby #{$0} num_port [options]"
  exit(1)
end
opt[:port][:num] = ARGV[0].to_i - (((`uname -a` =~ /cygwin/i) rescue false) ? 1 : 0)

def port_name(port_num)
  RUBY_PLATFORM =~ /win32/ ? "COM#{port_num}" : "tty#{port_num}"
end

opt[:dst_fname] ||= "#{port_name(opt[:port][:num])}_#{Time::now.strftime("%Y%d%m_%H%M%S")}.bin"

sp = SerialPort.new(*([:num, :bps, :nbits, :stopb].collect{|k| opt[:port][k]} + [SerialPort::NONE]))
$stderr.puts "#{port_name(opt[:port][:num])} opened."

read_prop = proc{|k|
  sp.write(k)
  if sp.gets !~ /\d+/
    $stderr.puts "Unknown property(#{k})!"
    exit(-1)
  end
  $&.to_i
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
  sp.read(property[:size] * sectors)
}

$stderr.puts "Save to #{opt[:dst_fname]}."
open(opt[:dst_fname], 'a+'){|dst|
  bs, es = [opt[:begin_sector], [property[:count], opt[:end_sector]].min]
  sectors = es - bs
  (bs..es).step(opt[:each_sectors]){|i|
    i_rel = i - bs
    $stderr.puts "Reading #{sprintf("%8d (%8d of %8d, %5.1f%%)", i, i_rel, sectors, 100.0 * i_rel / sectors)} ..."
    dst.write(read_block.call(i, opt[:each_sectors]))
  }
  $stderr.puts "done."
}

sp.close