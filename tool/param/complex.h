/*
 * Copyright (c) 2015, M.Naruoka (fenrir)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * - Neither the name of the naruoka.org nor the names of its contributors
 *   may be used to endorse or promote products derived from this software
 *   without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __COMPLEX_H
#define __COMPLEX_H

#include <exception>
#include <string>

#include <cmath>

#if (__cplusplus < 201103L) && !defined(noexcept)
#define noexcept throw()
#endif

/** @file
 * @brief 複素数ライブラリ
 *
 * 複素数を定義したライブラリ。
 * 多分誰が作っても似たような感じになると思われる。
 */

/**
 * @brief 複素数に関わる例外
 *
 * Complexクラスの例外クラス。
 * 例として、演算が成立しない場合など。
 *
 */
class ComplexException: public std::exception{
  private:
    std::string what_str;  ///< エラー内容
  public:
    /**
     * コンストラクタ
     *
     * @param what_arg エラー内容
     */
    ComplexException(const std::string &what_arg) noexcept : what_str(what_arg){}
    /**
     * デストラクタ
     *
     */
    ~ComplexException() noexcept {}
    /**
     * エラー内容を取得します。
     *
     * @return (chsr *) エラー内容
     */
    const char *what() const noexcept {
      return what_str.c_str();
    }
};

/**
 * @brief 複素数
 *
 * 複素数をあらわすクラスです。
 *
 * @param FloatT 演算精度、doubleなど
 */
template <class FloatT>
class Complex{
  private:
    FloatT m_Real; ///< 実部
    FloatT m_Imaginary; ///< 虚部
  public:
   /**
    * コンストラクタ。
    *
    * @param real 実数部
    * @param imaginary 虚数部
    */
    Complex(const FloatT &real, const FloatT &imaginary) noexcept
         : m_Real(real), m_Imaginary(imaginary){}

    /**
     * コンストラクタ。
     * 虚数部は0で初期化されます。
     *
     * @param real 実数部
     */
    Complex(const FloatT &real) noexcept : m_Real(real), m_Imaginary(0){}

    /**
     * コンストラクタ。
     * 実数部、虚数部ともに0に初期化されます。
     *
     */
    Complex() noexcept : m_Real(0), m_Imaginary(0){}

    /**
     * デストラクタ。
     *
     */
    ~Complex() noexcept {}

    /**
     * 実数部を返します。
     *
     * @return (FloatT) 実数部
     */
    const FloatT &real() const noexcept {return m_Real;}
    FloatT &real() noexcept {
      return const_cast<FloatT &>(static_cast<const Complex &>(*this).real());
    }
    /**
     * 虚数部を返します。
     *
     * @return (FloatT) 虚数部
     */
    const FloatT &imaginary() const noexcept {return m_Imaginary;}
    FloatT &imaginary() noexcept {
      return const_cast<FloatT &>(static_cast<const Complex &>(*this).imaginary());
    }

    Complex<FloatT> &operator=(const Complex<FloatT> &another) noexcept {
      m_Real = another.m_Real;
      m_Imaginary = another.m_Imaginary;
      return *this;
    }
    Complex<FloatT> &operator=(const FloatT &another) noexcept {
      m_Real = another;
      m_Imaginary = 0;
      return *this;
    }

    /**
     * 絶対値の二乗を返します。
     * pow(real(), 2) + pow(imaginary(), 2)をしています。
     *
     * @return (FloatT) 絶対値の二乗
     * @see pow(FloatT, FloatT)
     */
    FloatT abs2() const noexcept {return pow(m_Real, 2) + pow(m_Imaginary, 2);}
    /**
     * 絶対値を返します。
     * sqrt(abs2())をしています。
     *
     * @return (FloatT) 絶対値
     * @see abs2()
     * @see sqrt()
     */
    FloatT abs() const {return ::sqrt(abs2());}
    /**
     * 偏角を返します。
     *
     * @return (double) 偏角
     */
    double arg() const {
      if((m_Real == FloatT(0)) && (m_Imaginary == FloatT(0))){
        return 0;
      }
      return atan2(m_Imaginary, m_Real);
    }
    /**
     * 指定乗します。
     * ただし定義として、複素数が
     * @f[
     *    a + b i = r e^{i \theta}
     * @f]
     * とあらわされるとき、
     * @f[
     *    (a + b i)^{n} = r^{n} e^{i \theta n}
     * @f]
     * とします。
     *
     * @param factor 乗数
     * @return (Complex<FloatT>) 結果
     */
    Complex<FloatT> power(const FloatT &factor) const {
      if((m_Imaginary == FloatT(0)) && (m_Real >= FloatT(0))){
        return Complex(pow(m_Real, factor));
      }else{
        FloatT _abs(pow(abs(), factor));
        double _arg(arg() * factor);
        return Complex(_abs * std::cos(_arg), _abs * std::sin(_arg));
      }
    }

    /**
     * 平方根を求めます。定義はpowerに従います。
     *
     * @return (Complex<FloatT>) 結果
     * @see power
     */
    Complex<FloatT> sqrt() const {
      return power(FloatT(0.5));
    }

    /**
     * 共役複素数を返します。
     *
     * @return (Complex<FloatT>) 共役複素数
     */
    Complex<FloatT> conjugate() const noexcept {
      Complex<FloatT> result = *this;
      result.imaginary() *= -1;
      return result;
    }

    /**
     * 等しいかどうか調べます。
     *
     * @return (bool) 等しい場合true
     */
    bool operator==(const Complex<FloatT> &complex) const noexcept {
      return m_Real == complex.real()
          ? m_Imaginary == complex.imaginary()
          : false;
    }
    /**
     * 等しくないか調べます。
     *
     * @return (bool) 等しくない場合true
     */
    bool operator!=(const Complex<FloatT> &complex) const noexcept {
      return !(*this == complex);
    }

    /**
     * 加算を行います。破壊的です。
     *
     * @return (Complex<FloatT>) 加算結果
     */
    Complex<FloatT> &operator+=(const FloatT &scalar) noexcept {
      m_Real += scalar;
      return *this;
    }
    /**
     * 加算を行います。
     *
     * @return (Complex<FloatT>) 加算結果
     */
    Complex<FloatT> operator+(const FloatT &scalar) const noexcept {
      Complex<FloatT> result = *this;
      return (result += scalar);
    }
    /**
     * 加算を行います。
     *
     * @return (Complex<FloatT>) 加算結果
     */
    friend Complex<FloatT> operator+(const FloatT &scalar, const Complex<FloatT> complex) noexcept {return (complex + scalar);}

    /**
     * 減算を行います。破壊的です。
     *
     * @return (Complex<FloatT>) 減算結果
     */
    Complex<FloatT> &operator-=(const FloatT &scalar) noexcept {return (*this) += (-scalar);}
    /**
     * 減算を行います。
     *
     * @return (Complex<FloatT>) 減算結果
     */
    Complex<FloatT> operator-(const FloatT &scalar) const noexcept {return (*this) + (-scalar);}
    /**
     * 減算を行います。
     *
     * @return (Complex<FloatT>) 減算結果
     */
    friend Complex<FloatT> operator-(const FloatT &scalar, const Complex<FloatT> complex) noexcept {return (complex - scalar);}

    /**
     * 乗算を行います。破壊的です。
     *
     * @return (Complex<FloatT>) 乗算結果
     */
    Complex<FloatT> &operator *=(const FloatT &scalar) noexcept {
      m_Real *= scalar;
      m_Imaginary *= scalar;
      return *this;
    }
    /**
     * 乗算を行います。
     *
     * @return (Complex<FloatT>) 乗算結果
     */
    Complex<FloatT> operator*(const FloatT &scalar) const noexcept {
      Complex<FloatT> result(*this);
      return (result *= scalar);
    }
    /**
     * 乗算を行います。
     *
     * @return (Complex<FloatT>) 乗算結果
     */
    friend Complex<FloatT> operator*(const FloatT &scalar, const Complex<FloatT> complex) noexcept {return (complex * scalar);}

    /**
     * 除算を行います。破壊的です。
     *
     * @return (Complex<FloatT>) 除算結果
     */
    Complex<FloatT> &operator/=(const FloatT &scalar){return (*this) *= (1/scalar);}
    /**
     * 除算を行います。
     *
     * @return (Complex<FloatT>) 除算結果
     */
    Complex<FloatT> operator/(const FloatT &scalar) const{return (*this) * (1/scalar);}

    /**
     * 単項マイナスオペレータ。
     *
     * @return (Complex<FloatT>) 結果
     */
    Complex<FloatT> operator -() const noexcept {return ((*this) * -1);}

    /**
     * 加算を行います。破壊的です。
     *
     * @return (Complex<FloatT>) 加算結果
     */
    Complex<FloatT> &operator+=(const Complex<FloatT> &complex) noexcept {
      m_Real += complex.real();
      m_Imaginary += complex.imaginary();
      return *this;
    }
    /**
     * 加算を行います。
     *
     * @return (Complex<FloatT>) 加算結果
     */
    Complex<FloatT> operator+(const Complex<FloatT> &complex) const noexcept {
      Complex<FloatT> result = *this;
      return (result += complex);
    }

    /**
     * 減算を行います。破壊的です。
     *
     * @return (Complex<FloatT>) 減算結果
     */
    Complex<FloatT> &operator-=(const Complex<FloatT> &complex) noexcept {
      return ((*this) += (-complex));
    }
    /**
     * 減算を行います。
     *
     * @return (Complex<FloatT>) 減算結果
     */
    Complex<FloatT> operator-(const Complex<FloatT> &complex) const noexcept {
      return ((-complex) += (*this));
    }

    /**
     * 乗算を行います。
     *
     * @return (Complex<FloatT>) 乗算結果
     */
    Complex<FloatT> operator*(const Complex<FloatT> &complex) const noexcept {
      return Complex<FloatT>(
          m_Real * complex.real()      - m_Imaginary * complex.imaginary(),
          m_Real * complex.imaginary() + m_Imaginary * complex.real());
    }
    /**
     * 乗算を行います。破壊的です。
     *
     * @return (Complex<FloatT>) 乗算結果
     */
    Complex<FloatT> &operator*=(const Complex<FloatT> &complex) noexcept {
      Complex<FloatT> copy = *this;
      m_Real =
        copy.real() * complex.real()
        - copy.imaginary() * complex.imaginary();
      m_Imaginary =
        copy.real() * complex.imaginary()
        + copy.imaginary() * complex.real();
      return *this;
    }

    /**
     * 除算を行います。破壊的です。
     *
     * @return (Complex<FloatT>) 除算結果
     */
    Complex<FloatT> &operator/=(const Complex<FloatT> &complex){
      return (*this) *= (complex.conjugate() / complex.abs2());
    }
    /**
     * 除算を行います。
     *
     * @return (Complex<FloatT>) 除算結果
     */
    Complex<FloatT> operator/(const Complex<FloatT> &complex) const{
      Complex<FloatT> copy = (*this);
      return (copy /= complex);
    }

    /**
     * 除算を行います。
     *
     * @return (Complex<FloatT>) 除算結果
     */
    friend Complex<FloatT> operator/(const FloatT &scalar, const Complex<FloatT> complex){
      return Complex<FloatT>(scalar) / complex;
    }

    /**
     * みやすい形で複素数を出力します。
     *
     * @param out 出力ストリーム
     * @param complex 出力する複素数
     */
    friend std::ostream &operator<<(std::ostream &out, const Complex<FloatT> &complex){
      out << complex.real() << " + "
          << complex.imaginary() << "i";
      return out;
    }

    /**
     * eの指数乗を求めます。
     *
     * @param imaginary 虚部
     * @return (Complex<FloatT>) 結果
     */
    static Complex<FloatT> exp(const FloatT &imaginary) noexcept {
      return Complex<FloatT>(std::cos(imaginary), std::sin(imaginary));
    }

    /**
     * eの指数乗を求めます。
     *
     * @param real 実部
     * @param imaginary 虚部
     * @return (Complex<FloatT>) 結果
     */
    static Complex<FloatT> exp(const FloatT &real, const FloatT &imaginary) noexcept {
      return Complex<FloatT>::exp(imaginary) *= std::exp(real);
    }

    /**
     * eの指数乗を求めます。
     *
     * @param complex 複素数
     * @return (Complex<FloatT>) 結果
     */
    static Complex<FloatT> exp(const Complex<FloatT> &complex) noexcept {
      return Complex<FloatT>::exp(
          complex.real(), complex.imaginary());
    }
};

/**
 * expの複素数拡張
 *
 * @param real 実部
 * @param imaginary 虚部
 * @param FloatT 演算精度
 * @return (Complex<FloatT>) 結果
 */
template <class FloatT>
inline Complex<FloatT> iexp(const FloatT &real, const FloatT &imaginary) noexcept {
  return Complex<FloatT>::exp(real, imaginary);
}

/**
 * expの複素数拡張
 * @param imaginary 虚部
 * @param FloatT 演算精度
 * @return (Complex<FloatT>) 結果
 */
template <class FloatT>
inline Complex<FloatT> iexp(const FloatT &imaginary) noexcept {
  return Complex<FloatT>::exp(imaginary);
}

/**
 * expの複素数拡張
 *
 * @param complex 複素数
 * @param FloatT 演算精度
 * @return (Complex<FloatT>) 結果
 */
template <class FloatT>
inline Complex<FloatT> exp(const Complex<FloatT> &complex) noexcept {
  return Complex<FloatT>::exp(complex);
}

/**
 * powの複素数拡張
 *
 * @param complex 複素数
 * @param FloatT 演算精度
 * @return (Complex<FloatT>) 結果
 */
template <class FloatT>
inline Complex<FloatT> pow(const Complex<FloatT> &complex, const FloatT &factor) noexcept {
  return complex.power(factor);
}

/**
 * sqrtの複素数拡張
 *
 * @param complex 複素数
 * @param FloatT 演算精度
 * @return (Complex<FloatT>) 結果
 */
template <class FloatT>
inline Complex<FloatT> sqrt(const Complex<FloatT> &complex){
  return complex.sqrt();
}

#if (__cplusplus < 201103L) && defined(noexcept)
#undef noexcept
#endif

#endif /* __COMPLEX_H */
