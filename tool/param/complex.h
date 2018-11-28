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
 * @brief ���f�����C�u����
 *
 * ���f�����`�������C�u�����B
 * �����N������Ă������悤�Ȋ����ɂȂ�Ǝv����B
 */

/**
 * @brief ���f���Ɋւ���O
 *
 * Complex�N���X�̗�O�N���X�B
 * ��Ƃ��āA���Z���������Ȃ��ꍇ�ȂǁB
 *
 */
class ComplexException: public std::exception{
  private:
    std::string what_str;  ///< �G���[���e
  public:
    /**
     * �R���X�g���N�^
     *
     * @param what_arg �G���[���e
     */
    ComplexException(const std::string &what_arg) noexcept : what_str(what_arg){}
    /**
     * �f�X�g���N�^
     *
     */
    ~ComplexException() noexcept {}
    /**
     * �G���[���e���擾���܂��B
     *
     * @return (chsr *) �G���[���e
     */
    const char *what() const noexcept {
      return what_str.c_str();
    }
};

/**
 * @brief ���f��
 *
 * ���f��������킷�N���X�ł��B
 *
 * @param FloatT ���Z���x�Adouble�Ȃ�
 */
template <class FloatT>
class Complex{
  private:
    FloatT m_Real; ///< ����
    FloatT m_Imaginary; ///< ����
  public:
   /**
    * �R���X�g���N�^�B
    *
    * @param real ������
    * @param imaginary ������
    */
    Complex(const FloatT &real, const FloatT &imaginary) noexcept
         : m_Real(real), m_Imaginary(imaginary){}

    /**
     * �R���X�g���N�^�B
     * ��������0�ŏ���������܂��B
     *
     * @param real ������
     */
    Complex(const FloatT &real) noexcept : m_Real(real), m_Imaginary(0){}

    /**
     * �R���X�g���N�^�B
     * �������A�������Ƃ���0�ɏ���������܂��B
     *
     */
    Complex() noexcept : m_Real(0), m_Imaginary(0){}

    /**
     * �f�X�g���N�^�B
     *
     */
    ~Complex() noexcept {}

    /**
     * ��������Ԃ��܂��B
     *
     * @return (FloatT) ������
     */
    const FloatT &real() const noexcept {return m_Real;}
    FloatT &real() noexcept {
      return const_cast<FloatT &>(static_cast<const Complex &>(*this).real());
    }
    /**
     * ��������Ԃ��܂��B
     *
     * @return (FloatT) ������
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
     * ��Βl�̓���Ԃ��܂��B
     * pow(real(), 2) + pow(imaginary(), 2)�����Ă��܂��B
     *
     * @return (FloatT) ��Βl�̓��
     * @see pow(FloatT, FloatT)
     */
    FloatT abs2() const noexcept {return pow(m_Real, 2) + pow(m_Imaginary, 2);}
    /**
     * ��Βl��Ԃ��܂��B
     * sqrt(abs2())�����Ă��܂��B
     *
     * @return (FloatT) ��Βl
     * @see abs2()
     * @see sqrt()
     */
    FloatT abs() const {return ::sqrt(abs2());}
    /**
     * �Ίp��Ԃ��܂��B
     *
     * @return (double) �Ίp
     */
    double arg() const {
      if((m_Real == FloatT(0)) && (m_Imaginary == FloatT(0))){
        return 0;
      }
      return atan2(m_Imaginary, m_Real);
    }
    /**
     * �w��悵�܂��B
     * ��������`�Ƃ��āA���f����
     * @f[
     *    a + b i = r e^{i \theta}
     * @f]
     * �Ƃ���킳���Ƃ��A
     * @f[
     *    (a + b i)^{n} = r^{n} e^{i \theta n}
     * @f]
     * �Ƃ��܂��B
     *
     * @param factor �搔
     * @return (Complex<FloatT>) ����
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
     * �����������߂܂��B��`��power�ɏ]���܂��B
     *
     * @return (Complex<FloatT>) ����
     * @see power
     */
    Complex<FloatT> sqrt() const {
      return power(FloatT(0.5));
    }

    /**
     * ���𕡑f����Ԃ��܂��B
     *
     * @return (Complex<FloatT>) ���𕡑f��
     */
    Complex<FloatT> conjugate() const noexcept {
      Complex<FloatT> result = *this;
      result.imaginary() *= -1;
      return result;
    }

    /**
     * ���������ǂ������ׂ܂��B
     *
     * @return (bool) �������ꍇtrue
     */
    bool operator==(const Complex<FloatT> &complex) const noexcept {
      return m_Real == complex.real()
          ? m_Imaginary == complex.imaginary()
          : false;
    }
    /**
     * �������Ȃ������ׂ܂��B
     *
     * @return (bool) �������Ȃ��ꍇtrue
     */
    bool operator!=(const Complex<FloatT> &complex) const noexcept {
      return !(*this == complex);
    }

    /**
     * ���Z���s���܂��B�j��I�ł��B
     *
     * @return (Complex<FloatT>) ���Z����
     */
    Complex<FloatT> &operator+=(const FloatT &scalar) noexcept {
      m_Real += scalar;
      return *this;
    }
    /**
     * ���Z���s���܂��B
     *
     * @return (Complex<FloatT>) ���Z����
     */
    Complex<FloatT> operator+(const FloatT &scalar) const noexcept {
      Complex<FloatT> result = *this;
      return (result += scalar);
    }
    /**
     * ���Z���s���܂��B
     *
     * @return (Complex<FloatT>) ���Z����
     */
    friend Complex<FloatT> operator+(const FloatT &scalar, const Complex<FloatT> complex) noexcept {return (complex + scalar);}

    /**
     * ���Z���s���܂��B�j��I�ł��B
     *
     * @return (Complex<FloatT>) ���Z����
     */
    Complex<FloatT> &operator-=(const FloatT &scalar) noexcept {return (*this) += (-scalar);}
    /**
     * ���Z���s���܂��B
     *
     * @return (Complex<FloatT>) ���Z����
     */
    Complex<FloatT> operator-(const FloatT &scalar) const noexcept {return (*this) + (-scalar);}
    /**
     * ���Z���s���܂��B
     *
     * @return (Complex<FloatT>) ���Z����
     */
    friend Complex<FloatT> operator-(const FloatT &scalar, const Complex<FloatT> complex) noexcept {return (complex - scalar);}

    /**
     * ��Z���s���܂��B�j��I�ł��B
     *
     * @return (Complex<FloatT>) ��Z����
     */
    Complex<FloatT> &operator *=(const FloatT &scalar) noexcept {
      m_Real *= scalar;
      m_Imaginary *= scalar;
      return *this;
    }
    /**
     * ��Z���s���܂��B
     *
     * @return (Complex<FloatT>) ��Z����
     */
    Complex<FloatT> operator*(const FloatT &scalar) const noexcept {
      Complex<FloatT> result(*this);
      return (result *= scalar);
    }
    /**
     * ��Z���s���܂��B
     *
     * @return (Complex<FloatT>) ��Z����
     */
    friend Complex<FloatT> operator*(const FloatT &scalar, const Complex<FloatT> complex) noexcept {return (complex * scalar);}

    /**
     * ���Z���s���܂��B�j��I�ł��B
     *
     * @return (Complex<FloatT>) ���Z����
     */
    Complex<FloatT> &operator/=(const FloatT &scalar){return (*this) *= (1/scalar);}
    /**
     * ���Z���s���܂��B
     *
     * @return (Complex<FloatT>) ���Z����
     */
    Complex<FloatT> operator/(const FloatT &scalar) const{return (*this) * (1/scalar);}

    /**
     * �P���}�C�i�X�I�y���[�^�B
     *
     * @return (Complex<FloatT>) ����
     */
    Complex<FloatT> operator -() const noexcept {return ((*this) * -1);}

    /**
     * ���Z���s���܂��B�j��I�ł��B
     *
     * @return (Complex<FloatT>) ���Z����
     */
    Complex<FloatT> &operator+=(const Complex<FloatT> &complex) noexcept {
      m_Real += complex.real();
      m_Imaginary += complex.imaginary();
      return *this;
    }
    /**
     * ���Z���s���܂��B
     *
     * @return (Complex<FloatT>) ���Z����
     */
    Complex<FloatT> operator+(const Complex<FloatT> &complex) const noexcept {
      Complex<FloatT> result = *this;
      return (result += complex);
    }

    /**
     * ���Z���s���܂��B�j��I�ł��B
     *
     * @return (Complex<FloatT>) ���Z����
     */
    Complex<FloatT> &operator-=(const Complex<FloatT> &complex) noexcept {
      return ((*this) += (-complex));
    }
    /**
     * ���Z���s���܂��B
     *
     * @return (Complex<FloatT>) ���Z����
     */
    Complex<FloatT> operator-(const Complex<FloatT> &complex) const noexcept {
      return ((-complex) += (*this));
    }

    /**
     * ��Z���s���܂��B
     *
     * @return (Complex<FloatT>) ��Z����
     */
    Complex<FloatT> operator*(const Complex<FloatT> &complex) const noexcept {
      return Complex<FloatT>(
          m_Real * complex.real()      - m_Imaginary * complex.imaginary(),
          m_Real * complex.imaginary() + m_Imaginary * complex.real());
    }
    /**
     * ��Z���s���܂��B�j��I�ł��B
     *
     * @return (Complex<FloatT>) ��Z����
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
     * ���Z���s���܂��B�j��I�ł��B
     *
     * @return (Complex<FloatT>) ���Z����
     */
    Complex<FloatT> &operator/=(const Complex<FloatT> &complex){
      return (*this) *= (complex.conjugate() / complex.abs2());
    }
    /**
     * ���Z���s���܂��B
     *
     * @return (Complex<FloatT>) ���Z����
     */
    Complex<FloatT> operator/(const Complex<FloatT> &complex) const{
      Complex<FloatT> copy = (*this);
      return (copy /= complex);
    }

    /**
     * ���Z���s���܂��B
     *
     * @return (Complex<FloatT>) ���Z����
     */
    friend Complex<FloatT> operator/(const FloatT &scalar, const Complex<FloatT> complex){
      return Complex<FloatT>(scalar) / complex;
    }

    /**
     * �݂₷���`�ŕ��f�����o�͂��܂��B
     *
     * @param out �o�̓X�g���[��
     * @param complex �o�͂��镡�f��
     */
    friend std::ostream &operator<<(std::ostream &out, const Complex<FloatT> &complex){
      out << complex.real() << " + "
          << complex.imaginary() << "i";
      return out;
    }

    /**
     * e�̎w��������߂܂��B
     *
     * @param imaginary ����
     * @return (Complex<FloatT>) ����
     */
    static Complex<FloatT> exp(const FloatT &imaginary) noexcept {
      return Complex<FloatT>(std::cos(imaginary), std::sin(imaginary));
    }

    /**
     * e�̎w��������߂܂��B
     *
     * @param real ����
     * @param imaginary ����
     * @return (Complex<FloatT>) ����
     */
    static Complex<FloatT> exp(const FloatT &real, const FloatT &imaginary) noexcept {
      return Complex<FloatT>::exp(imaginary) *= std::exp(real);
    }

    /**
     * e�̎w��������߂܂��B
     *
     * @param complex ���f��
     * @return (Complex<FloatT>) ����
     */
    static Complex<FloatT> exp(const Complex<FloatT> &complex) noexcept {
      return Complex<FloatT>::exp(
          complex.real(), complex.imaginary());
    }
};

/**
 * exp�̕��f���g��
 *
 * @param real ����
 * @param imaginary ����
 * @param FloatT ���Z���x
 * @return (Complex<FloatT>) ����
 */
template <class FloatT>
inline Complex<FloatT> iexp(const FloatT &real, const FloatT &imaginary) noexcept {
  return Complex<FloatT>::exp(real, imaginary);
}

/**
 * exp�̕��f���g��
 * @param imaginary ����
 * @param FloatT ���Z���x
 * @return (Complex<FloatT>) ����
 */
template <class FloatT>
inline Complex<FloatT> iexp(const FloatT &imaginary) noexcept {
  return Complex<FloatT>::exp(imaginary);
}

/**
 * exp�̕��f���g��
 *
 * @param complex ���f��
 * @param FloatT ���Z���x
 * @return (Complex<FloatT>) ����
 */
template <class FloatT>
inline Complex<FloatT> exp(const Complex<FloatT> &complex) noexcept {
  return Complex<FloatT>::exp(complex);
}

/**
 * pow�̕��f���g��
 *
 * @param complex ���f��
 * @param FloatT ���Z���x
 * @return (Complex<FloatT>) ����
 */
template <class FloatT>
inline Complex<FloatT> pow(const Complex<FloatT> &complex, const FloatT &factor) noexcept {
  return complex.power(factor);
}

/**
 * sqrt�̕��f���g��
 *
 * @param complex ���f��
 * @param FloatT ���Z���x
 * @return (Complex<FloatT>) ����
 */
template <class FloatT>
inline Complex<FloatT> sqrt(const Complex<FloatT> &complex){
  return complex.sqrt();
}

#if (__cplusplus < 201103L) && defined(noexcept)
#undef noexcept
#endif

#endif /* __COMPLEX_H */
