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
      decl << "static const FloatT P_#{n}_#{m}[];"
      upper << "template<class FloatT> const FloatT EGM<FloatT>::P_#{n}_#{m}[] = {
    #{f.collect{|v| "%a"%[v.to_f]}.join(', ')}};"
      lower << "{#{n}, #{m}, #{0}, #{0}, #{"%a"%[c]}, P_#{n}_#{m}},"
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

print_proc.call(3)