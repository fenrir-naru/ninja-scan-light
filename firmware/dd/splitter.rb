#!/usr/bin/ruby

$stderr.puts "Usage: begin end [log.dat or $stdin]"
exit(-1) if ARGV.size < 2

i, j = ARGV[0..1].collect{|arg|
  res = eval(arg)
  raise "Invalid parameter: #{arg}" unless (res.kind_of?(Integer) and res >= 0)
  res
}

src = (ARGV.size == 3) ? ARGV[2] : $stdin
$stderr.puts(<<__TEXT__)
Run equivalent: cat #{src.kind_of?(String) ? src : '-'} | head -c #{j} | tail -c +#{i + 1}
(if needed, to check) | od -A x -t x1 | less
__TEXT__

filter_proc = proc{|io|
  io.read(i)
  $stdout.write(io.read(j - i))
}

src.kind_of?(IO) \
    ? filter_proc.call(src) \
    : open(src, 'r'){|io|
  filter_proc.call(io)
}