#!/usr/bin/ruby
# coding: cp932

OPTIONS = [[:egm, [:void]], [:kf, [:KalmanFilter]], [:tightly, [1]], [:bias]]
PRIORITY = {
  :egm => 1,
  :kf => 2,
  :tightly => 3,
  :bias => 4,
}

nested_gen = proc{|array| 
  "typename opt_t::#{array[0][0]}_t<%s#{array[0][1] ? ", #{array[0][1].join(',')}" : ' '}>"%[
      (array.size <= 1) ? :void : "#{nested_gen.call(array[1..-1])}"]
}

(OPTIONS.size + 1).times{|i|
  OPTIONS.product(*([OPTIONS] * i)).each{|opt|
    if i > 0 then
      flag = false
      opt[0..-2].zip(opt[1..-1]).each{|a, b|
        if a == b then
          flag = true
          break
        end
      }
      next if flag
    end
    comp = [
        "typename factory_t#{opt.collect{|v| "::template #{v[0]}<#{(v[1] || []).join(', ')}>"}.join}::options_t",
        nested_gen.call(opt.uniq.sort{|a, b| PRIORITY[b[0]] <=> PRIORITY[a[0]]})]
    print(<<__TEXT__)
BOOST_CHECK((boost::is_same<
    #{comp[0]},
    #{comp[1]} >::value));
__TEXT__
  }
}
