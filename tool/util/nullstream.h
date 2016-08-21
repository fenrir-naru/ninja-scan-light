/*
 * Copyright (c) 2016, M.Naruoka (fenrir)
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

#ifndef __NULLSTREAM_H__
#define __NULLSTREAM_H__

#include <streambuf>
#include <iostream>
#include <string>

template<
    class _Elem, 
    class _Traits>
class basic_NullStreambuf : public std::basic_streambuf<_Elem, _Traits> {
  protected:
    typedef std::basic_streambuf<_Elem, _Traits> super_t;
    typedef std::streamsize streamsize;
    typedef typename super_t::int_type int_type;
  public:
    basic_NullStreambuf() throw(std::ios_base::failure) : super_t() {}
    virtual ~basic_NullStreambuf() {}
  protected:
    int_type overflow(int_type c = _Traits::eof()){
      return _Traits::not_eof(c);
    }
    streamsize xsputn(const _Elem *s, streamsize n){
      return (streamsize)n;
    }
    
    streamsize xsgetn(_Elem *s, streamsize n){
      return (streamsize)0;
    }
    int_type underflow(){
      return _Traits::eof();
    }
    int_type uflow(){
      return _Traits::eof();
    }
};

typedef basic_NullStreambuf<char, std::char_traits<char> > NullStreambuf;

class NullStream : public std::iostream{
  public:
    typedef NullStreambuf buf_t;
  protected:
    typedef std::iostream super_t;
    buf_t buf;
  public:
    NullStream() throw(std::ios_base::failure) : buf(), super_t(&buf){}
    ~NullStream(){}
};

#endif /* __NULLSTREAM_H__ */
