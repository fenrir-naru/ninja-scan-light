#!/usr/bin/ruby
# coding: cp932

require 'stringio'

# derivative
class Array
  def derivative!
    self.size.times{|i|
      self[i] *= i
    }
    self.shift
    self
  end
end

# @return [coefficient(c) of sin^0, c of sin^1, ...]
def legendre(n, cache = {})
  return n if cache.include?(n)
  
  res = case n
  when 0
    [Rational(1)]
  when 1
    [Rational(0), Rational(1)]
  else
    ([Rational(0)] + legendre(n - 1).collect{|v|
      v / n * (n * 2 - 1)
    }).zip(legendre(n - 2).collect{|v|
      v / n * (n - 1)
    }).collect{|a, b|
      b ? a - b : a
    }
  end
  cache[n] = res
  return res
end

print_proc = proc{|n_max, opt|
  legendre_cache = {}
  decl = []
  upper = []
  lower = []
  (2..n_max).each{|n|
    p_n = legendre(n, legendre_cache)
    p_nm = [p_n]
    p_nm_bar_coef = [(n * 2 + 1).to_f]
    n.times{|m|
      p_nm << p_nm[-1].clone.derivative!
      p_nm_bar_coef << p_nm_bar_coef[-1] / ((n - m) * (n + 1 + m))
    }
    p_nm.each_with_index{|f, m|
      c = Math::sqrt(p_nm_bar_coef[m] * (m == 0 ? 1 : 2))
      c_bar, s_bar = (opt[:CS_Bar][[n, m]] rescue ["C_BAR_#{n}_#{m}", "S_BAR_#{n}_#{m}"])
      decl << "static const FloatT P_#{n}_#{m}[];"
      upper << "template<class FloatT> const FloatT EGM<FloatT>::P_#{n}_#{m}[] = {
    #{f.collect{|v| "%a"%[v.to_f]}.join(', ')}};"
      lower << "{#{n}, #{m}, #{c_bar}, #{s_bar}, #{"%a"%[c]}, P_#{n}_#{m}},"
    }
  }
  puts(<<__TEXT__)
template <class FloatT>
struct EGM {
  static const struct coefficients_t {
    const int n;
    const int m;
    const FloatT c_bar;
    const FloatT s_bar;
    const FloatT p_nm_bar;
    const FloatT *p_nm;
  } coefficients[];
  #{decl.join("\n  ")}
};

#{upper.join("\n")}

template<class FloatT>
const typename EGM<FloatT>::coefficients_t EGM<FloatT>::coefficients[] = {
  #{lower.join("\n  ")}
};
__TEXT__
}

read_coef_file = proc{|f|
  next nil unless f
  require 'open-uri' if f =~ /^https?:\/\//
  data = open(f){|io|
    case f
    when /\.z$/
      require 'tempfile'
      Tempfile.open(File::basename(__FILE__, '.*')){|f_tmp|
        f_tmp.write(io.read)
        IO::popen("gunzip -c #{f_tmp.path}", 'r'){|io2|
          io2.read
        }
      }
    else
      io.read
    end
  }
  res = {}
  data.each_line{|line|
    values = line.chomp.sub(/^ +/, '').split(/ +/)
    res[[0, 1].collect{|i| values[i].to_i}] = [2, 3].collect{|i| values[i]}
  }
  res
}

$stderr.puts "Usage #{$0} n [CS_bar_file]"
$stderr.puts "ex) #{$0} 10 http://earth-info.nga.mil/GandG/wgs84/gravitymod/egm96/egm96.z"
print_proc.call(ARGV.shift.to_i, {:CS_Bar => read_coef_file.call(ARGV.shift)})
