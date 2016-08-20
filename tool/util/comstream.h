/*
 * Copyright (c) 2013, M.Naruoka (fenrir)
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

#ifndef __COMSTREAM_H__
#define __COMSTREAM_H__

#include <streambuf>
#include <iostream>
#include <string>

#include <cstring>

#ifdef _WIN32
#include <windows.h>
#include <cstdlib>
#else
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <cstdio>
#endif

/**
 * Blocking streambuf for serial/tty port
 * 
 */
template<
    class _Elem, 
    class _Traits>
class basic_ComportStreambuf : public std::basic_streambuf<_Elem, _Traits> {
  public:
#ifdef _WIN32
    typedef HANDLE handle_t;
#else
    typedef int handle_t;
#endif
  protected:
    typedef std::basic_streambuf<_Elem, _Traits> super_t;
    typedef std::streamsize streamsize;
    typedef typename super_t::int_type int_type;
    handle_t handle;
    int_type in_buf;
    bool in_buf_ready;
    static handle_t spec2handle(const char *port_spec) throw(std::ios_base::failure) {
      std::string regular_name(port_spec);
#ifdef _WIN32
      // When Windows COM port number is greater than 9, we must change name format.
      if(port_spec == strstr(port_spec, "COM")){
        regular_name = std::string("\\\\.\\").append(port_spec);
      }
#ifdef _UNICODE
      wchar_t wname[512];
      mbstowcs(wname, regular_name.c_str(), sizeof(wname));
#endif
      handle_t res = CreateFile( 
#ifdef _UNICODE
          wname,
#else
          regular_name.c_str(),
#endif
          GENERIC_READ | GENERIC_WRITE,
          0,
          0,
          OPEN_EXISTING,
          FILE_ATTRIBUTE_NORMAL,
          0);
      if(res == INVALID_HANDLE_VALUE){
        throw std::ios_base::failure(std::string("Could not open ").append(port_spec));
      }
#else
      // Port open
      handle_t res;
      if((res = open(port_spec, O_RDWR | O_NOCTTY /*| O_NONBLOCK*/)) == -1){
        // O_RDWR:For both read and write O_NOCTTY:No tty control
        perror("open");
        throw std::ios_base::failure(std::string("Could not open ").append(port_spec));
      }
      /*if(ioctl(res, TIOCEXCL) == -1){
        perror("TIOCEXCL");
        close(res);
        throw std::ios_base::failure(std::string("TIOCEXCL ").append(port_spec));
      }
      if(fcntl(res, F_SETFL, 0) == -1){
        perror("clear O_NONBLOCK");
        close(res);
        throw std::ios_base::failure(std::string("clear O_NONBLOCK ").append(port_spec));
      }*/
#endif
      return res;
    }
    
  public:
#ifdef _WIN32
    virtual void config_dcb(){
      DCB dcb;
      GetCommState(handle, &dcb);
      dcb.BaudRate = CBR_9600;
      dcb.ByteSize = 8;
      dcb.Parity = NOPARITY;
      dcb.fParity = FALSE;
      dcb.StopBits = ONESTOPBIT;
      dcb.fBinary = TRUE; // Binary mode
      dcb.fNull = FALSE;  // Don't throw away NULL
      dcb.fOutX = FALSE;  // No XON
      dcb.fInX = FALSE;   // No XOFF
      dcb.fOutxCtsFlow = FALSE;   // No CTS flow control
      dcb.fOutxDsrFlow = FALSE;   // No DSR flow control
      dcb.fDtrControl = DTR_CONTROL_DISABLE; // No DSR control
      dcb.fDsrSensitivity = FALSE;
      dcb.fRtsControl = RTS_CONTROL_DISABLE;
      dcb.fAbortOnError = FALSE;
      SetCommState(handle, &dcb);
    }
    void config_dcb(void (*setter)(DCB &)){
      DCB dcb;
      GetCommState(handle, &dcb);
      setter(dcb);
      SetCommState(handle, &dcb);
    }
    virtual void config_timeout(){
      COMMTIMEOUTS tout;
      GetCommTimeouts(handle, &tout);
      // Wait RX/TX forever
      tout.ReadIntervalTimeout = 0;
      tout.ReadTotalTimeoutConstant = 0;
      tout.ReadTotalTimeoutMultiplier = 0;
      tout.WriteTotalTimeoutConstant = 0;
      tout.WriteTotalTimeoutMultiplier = 0;
      SetCommTimeouts(handle, &tout);
    }
    void config_timeout(void (*setter)(COMMTIMEOUTS &)){
      COMMTIMEOUTS tout;
      GetCommTimeouts(handle, &tout);
      setter(tout);
      SetCommTimeouts(handle, &tout);
    }
    virtual void config(){
      config_dcb();
      config_timeout();
    }
    int set_baudrate(const int &request_speed){
      DWORD new_speed(0);
      DCB dcb;
      GetCommState(handle, &dcb);
      switch(request_speed){
        case 1200: new_speed = CBR_1200; break;
        case 2400: new_speed = CBR_2400; break;
        case 4800: new_speed = CBR_4800; break;
        case 9600: new_speed = CBR_9600; break;
        case 19200: new_speed = CBR_19200; break;
        case 38400: new_speed = CBR_38400; break;
        case 57600: new_speed = CBR_57600; break;
        case 115200: new_speed = CBR_115200; break;
        default: return -1;
      }
      dcb.BaudRate = new_speed;
      SetCommState(handle, &dcb);
      return request_speed;
    }
    void clear_error(
        void (*error_handler)(const DWORD &, const COMSTAT &) = NULL){
      DWORD errors;
      COMSTAT stat;
      ClearCommError(handle, &errors, &stat);
      if(error_handler){error_handler(errors, stat);}
      SetCommBreak(handle);
      ClearCommBreak(handle);
    }
#else
    static int speed_to_num(const speed_t &speed){
      static const struct {
        int num;
        speed_t speed;
      } speed_map[] = {
        { 0,      B0 },
        { 50,     B50 },
        { 75,     B75 }, 
        { 110,    B110 },
        { 134,    B134 },
        { 150,    B150 },
        { 200,    B200 },
        { 300,    B300 },
        { 600,    B600 },
        { 1200,   B1200 },
        { 1800,   B1800 },
        { 2400,   B2400 },
        { 4800,   B4800 },
        { 9600,   B9600 },
        { 19200,  B19200 },
        { 38400,  B38400 },
        { 57600,  B57600 },
        { 115200, B115200 },
      };
      for(int i(0); i < sizeof(speed_map) / sizeof(speed_map[0]); i++){
        if(speed_map[i].speed == speed){
          return speed_map[i].num;
        }
      }
      return speed;
    }
    static void print_status(const struct termios &tio){
      std::cerr << " ispeed " << speed_to_num(cfgetispeed(&tio));
      std::cerr << " ospeed " << speed_to_num(cfgetospeed(&tio));
      std::cerr << std::endl;
      std::cerr << ((tio.c_iflag & IGNBRK) ? '+' : '-') << "IGNBRK ";
      std::cerr << ((tio.c_iflag & BRKINT) ? '+' : '-') << "BRKINT ";
      std::cerr << ((tio.c_iflag & IGNPAR) ? '+' : '-') << "IGNPAR ";
      std::cerr << ((tio.c_iflag & PARMRK) ? '+' : '-') << "PARMRK ";
      std::cerr << ((tio.c_iflag & INPCK) ? '+' : '-') << "INPCK ";
      std::cerr << ((tio.c_iflag & ISTRIP) ? '+' : '-') << "ISTRIP ";
      std::cerr << ((tio.c_iflag & INLCR) ? '+' : '-') << "INLCR ";
      std::cerr << ((tio.c_iflag & IGNCR) ? '+' : '-') << "IGNCR ";
      std::cerr << ((tio.c_iflag & ICRNL) ? '+' : '-') << "ICRNL ";
      std::cerr << ((tio.c_iflag & IXON) ? '+' : '-') << "IXON ";
      std::cerr << ((tio.c_iflag & IXOFF) ? '+' : '-') << "IXOFF ";
      std::cerr << ((tio.c_iflag & IXANY) ? '+' : '-') << "IXANY ";
      std::cerr << ((tio.c_iflag & IMAXBEL) ? '+' : '-') << "IMAXBEL ";
      std::cerr << std::endl;
      std::cerr << ((tio.c_oflag & OPOST) ? '+' : '-') << "OPOST ";
      std::cerr << ((tio.c_oflag & ONLCR) ? '+' : '-') << "ONLCR ";
#ifdef OXTABS
      std::cerr << ((tio.c_oflag & OXTABS) ? '+' : '-') << "OXTABS ";
#endif
#if defined(TABDLY) && defined(XTABS) /* linux */
      std::cerr << ((tio.c_oflag & TABDLY) == XTABS ? '+' : '-') << "TABDLY ";
#endif
#ifdef ONOEOT
      std::cerr << ((tio.c_oflag & ONOEOT) ? '+' : '-') << "ONOEOT ";
#endif
      std::cerr << std::endl;
      {
        char csstr;
        switch (tio.c_cflag & CSIZE) {
          case CS5: csstr = '5'; break;
          case CS6: csstr = '6'; break;
          case CS7: csstr = '7'; break;
          case CS8: csstr = '8'; break;
          default: csstr = '?'; break;
        }
        std::cerr << "CS" << csstr << " ";
      }
      std::cerr << ((tio.c_cflag & CSTOPB) ? '+' : '-') << "CSTOPB ";
      std::cerr << ((tio.c_cflag & CREAD) ? '+' : '-') << "CREAD ";
      std::cerr << ((tio.c_cflag & PARENB) ? '+' : '-') << "PARENB ";
      std::cerr << ((tio.c_cflag & PARODD) ? '+' : '-') << "PARODD ";
      std::cerr << ((tio.c_cflag & HUPCL) ? '+' : '-') << "HUPCL ";
      std::cerr << ((tio.c_cflag & CLOCAL) ? '+' : '-') << "CLOCAL ";
#ifdef CCTS_OFLOW
      std::cerr << ((tio.c_cflag & CCTS_OFLOW) ? '+' : '-') << "CCTS_OFLOW ";
#endif
      std::cerr << ((tio.c_cflag & CRTSCTS) ? '+' : '-') << "CRTSCTS ";
#ifdef CRTS_IFLOW
      std::cerr << ((tio.c_cflag & CRTS_IFLOW) ? '+' : '-') << "CRTS_IFLOW ";
#endif
#ifdef MDMBUF
      std::cerr << ((tio.c_cflag & MDMBUF) ? '+' : '-') << "MDMBUF ";
#endif
      std::cerr << ((tio.c_lflag & ECHOKE) ? '+' : '-') << "ECHOKE ";
      std::cerr << ((tio.c_lflag & ECHOE) ? '+' : '-') << "ECHOE ";
      std::cerr << ((tio.c_lflag & ECHO) ? '+' : '-') << "ECHO ";
      std::cerr << ((tio.c_lflag & ECHONL) ? '+' : '-') << "ECHONL ";
#ifdef ECHOPRT
      std::cerr << ((tio.c_lflag & ECHOPRT) ? '+' : '-') << "ECHOPRT ";
#endif
      std::cerr << ((tio.c_lflag & ECHOCTL) ? '+' : '-') << "ECHOCTL ";
      std::cerr << ((tio.c_lflag & ISIG) ? '+' : '-') << "ISIG ";
      std::cerr << ((tio.c_lflag & ICANON) ? '+' : '-') << "ICANON ";
#ifdef ALTWERASE
      std::cerr << ((tio.c_lflag & ALTWERASE) ? '+' : '-') << "ALTWERASE ";
#endif
      std::cerr << ((tio.c_lflag & IEXTEN) ? '+' : '-') << "IEXTEN ";
      std::cerr << std::endl;
#ifdef EXTPROC
      std::cerr << ((tio.c_lflag & EXTPROC) ? '+' : '-') << "EXTPROC ";
#endif
      std::cerr << ((tio.c_lflag & TOSTOP) ? '+' : '-') << "TOSTOP ";
      std::cerr << ((tio.c_lflag & FLUSHO) ? '+' : '-') << "FLUSHO ";
#ifdef NOKERNINFO
      std::cerr << ((tio.c_lflag & NOKERNINFO) ? '+' : '-') << "NOKERNINFO ";
#endif
#ifdef PENDIN
      std::cerr << ((tio.c_lflag & PENDIN) ? '+' : '-') << "PENDIN ";
#endif
      std::cerr << ((tio.c_lflag & NOFLSH) ? '+' : '-') << "NOFLSH ";
      std::cerr << std::endl;
      
      std::cerr << "c_cc[VTIME]: " << (int)(tio.c_cc[VTIME]) << std::endl;  // キャラクタ間タイマ
      std::cerr << "c_cc[VMIN]: " << (int)(tio.c_cc[VMIN]) << std::endl;    // n文字受け取るまでブロックする
    }
    virtual void config(){
      struct termios config_data;
      
      if(!isatty(handle)){return;}
      
      tcgetattr(handle, &config_data); // Get current configuration
#ifdef DEBUG
      print_status(config_data);
#endif
      
      /*
       * rawモード
       */
#if defined(HAVE_CFMAKERAW) || !defined(__CYGWIN__)
      cfmakeraw(&config_data);
#else
      config_data.c_iflag 
          &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
      config_data.c_oflag &= ~OPOST;
      config_data.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
      config_data.c_cflag &= ~(CSIZE | PARENB);
      config_data.c_cflag |= CS8;
#endif
      tcsetattr(handle, TCSANOW, &config_data); // Set configuration
      
      /*
       * CRTSCTS: has control
       * 115200bps, CS8:8bit, no parity, 1 stop bit
       */
      cfsetispeed(&config_data, B115200);
      cfsetospeed(&config_data, B115200);
      config_data.c_cflag = (config_data.c_cflag & ~CSIZE) | CS8;
      config_data.c_cflag &= ~PARENB;
      config_data.c_cflag &= ~CSTOPB;
      /*
       * no other flow control
       */
      config_data.c_cflag &= ~CRTSCTS;
      config_data.c_iflag &= ~(IXON | IXOFF);
      /*
       * CLOCAL:ignore modem status, CREAD:receivable
       */
      config_data.c_cflag |= (CLOCAL | CREAD);
      
      config_data.c_cc[VTIME] = 0;  // timer between two characters
      config_data.c_cc[VMIN] = 1;   // blocking until n characters recption
      
      tcsetattr(handle, TCSANOW, &config_data); // 設定を保存
      
#ifdef DEBUG
      print_status(config_data);
#endif
    }
    void config(void (*setter)(struct termios &)){
      struct termios config_data;
      tcgetattr(handle, &config_data);
      setter(config_data);
      tcsetattr(handle, TCSANOW, &config_data);
    }
    int set_baudrate(const int &request_speed){
      speed_t new_speed(0);
      struct termios config_data;
      tcgetattr(handle, &config_data);
      switch(request_speed){
        case 1200: new_speed = B1200; break;
        case 2400: new_speed = B2400; break;
        case 4800: new_speed = B4800; break;
        case 9600: new_speed = B9600; break;
        case 19200: new_speed = B19200; break;
        case 38400: new_speed = B38400; break;
        case 57600: new_speed = B57600; break;
        case 115200: new_speed = B115200; break;
        default: return -1;
      }
      cfsetispeed(&config_data, new_speed);
      cfsetospeed(&config_data, new_speed);
      tcsetattr(handle, TCSANOW, &config_data);
      return request_speed;
    }
    void clear_error(){
      tcflush(handle, TCIFLUSH);
    }
#endif
    handle_t get_handle() const {
      return handle;
    }
    basic_ComportStreambuf(const char *port_spec) throw(std::ios_base::failure)
        : super_t(), handle(spec2handle(port_spec)), in_buf_ready(false) {
      config();
      clear_error();
    }
    virtual ~basic_ComportStreambuf() {
#ifdef _WIN32
      CloseHandle(handle);
#else
      close(handle);
#endif
      //std::cerr << "~()" << std::endl;
    }
    void update_in_buf(){
      in_buf_ready = true;
#ifdef _WIN32
      DWORD received;
      //static int seq_num(0);
      //seq_num++;
      //std::cerr << "update_in_buf() : " << seq_num << std::endl;
      if(ReadFile(handle, (LPVOID)&in_buf, 1, &received, NULL)
          && (received > 0)){
        //std::cerr << "received!" << std::endl;
        return;
      }
      //std::cerr << "return update_in_buf() : " << seq_num << std::endl;
#else
      if(read(handle, (void *)&in_buf, 1)){
        return;
      }
#endif
      in_buf = _Traits::eof();
    }
    
  protected:
    
    /**
     * Get number of characters available in the sequence
     * 
     * This member function (to be read s-how-many-c) is called to get an 
     * estimate on the number of characters available in the associated 
     * input sequence when the get pointer has reached the apparent end of 
     * the associated sequence (still, characters may be available 
     * after an underflow, and their count is what the return value of 
     * this function is expected to be). 
     * 
     * The public member function in_avail calls this protected member 
     * function to perform this action when the get pointer has reached 
     * the end pointer (or when it is set to null).
     * 
     * @return An estimate on the number of characters remaining to 
     * be read in the associated character sequence after an underflow 
     * when no read positions are available at the get pointer.
     */
#ifdef _WIN32
    streamsize showmanyc(){
      //std::cerr << "showmanyc()" << std::endl;
      DWORD dwerrors;
      COMSTAT comstat;
      ClearCommError(handle, &dwerrors, &comstat);
      return comstat.cbInQue;
    }
#else
    /* TODO: unistd.hの低水準I/Oはバッファリングなしのため、
     * streambufの標準挙動である0返しでよい?
     * @see http://sato-www.cs.titech.ac.jp/pro3/slides/unix2-4.pdf
     * @see http://www.cplusplus.com/reference/iostream/streambuf/showmanyc/
     */
#endif
    
    /**
     * Write character in the case of overflow
     * 
     * For the purpose of the streambuf class, overflows happen when 
     * a new character is to be written at the put pointer pptr position, 
     * but this has reached the end pointer epptr, indicating that 
     * apparently no room is currently available at the internal output array.
     * 
     * This function is expected either to modify the pbase, pptr and epptr 
     * pointers that define the internal output array in such a way that 
     * room is made available for more characters, or otherwise fail. 
     * It is also responsibility of the function to write the character 
     * passed as argument.
     * 
     * The specific behavior depends on each derived class, but it normally 
     * attempts to write some of the characters to the controlled output 
     * sequence to make room for more characters in the internal output array. 
     * 
     * This protected member function is automatically called by sputc and 
     * sputn member functions when overflows happen.
     * 
     * @param c Character to be written.
     * @return A value different than EOF (or traits::eof() for other traits) 
     * signals success. If the function fails, either EOF (or traits::eof() 
     * for other traits) is returned.
     */
    int_type overflow(int_type c = _Traits::eof()){
      //std::cerr << "overflow()" << std::endl;
#ifdef _WIN32
      DWORD transmitted;
      if((c != _Traits::eof())
          && WriteFile(handle, (LPCVOID)&c, 1, &transmitted, NULL)
          && (transmitted > 0)){
        return true;
      }
#else
      if((c != _Traits::eof())
          && write(handle, (const void *)&c, 1)){
        return true;
      }
#endif
      return (_Traits::eof());
    }
    
    /**
     * Write sequence of characters
     * 
     * Writes up to n characters from the array pointed by s to the output 
     * sequence controlled by the stream buffer. If less than n characters 
     * can be written to the output sequence the function stops and leaves 
     * the put pointer pptr in the same state as if successive calls to 
     * sputc were made until an EOF (or traits::eof() for other traits) 
     * was returned. 
     * 
     * @param s pointer to the sequence of characters to be output
     * @param n number of character to be put
     * @return the number of characters written
     */
#ifdef _WIN32
    streamsize xsputn(const _Elem *s, streamsize n){
      //std::cerr << "xsputn()" << std::endl;

      DWORD transmitted;
      WriteFile(handle, s, (DWORD)n, &transmitted, NULL);
      return (streamsize)transmitted;
    }
#else
    /* TODO: unistd.hの低水準I/Oはバッファリングなしのため、
     * 複数文字を引数にとるwriteを呼び出しだところで、1文字書きだけで返ってきてしまう?
     * 安全のため、streambufの標準挙動であるsputc複数回呼び出しで対応
     * @see http://www.cplusplus.com/reference/iostream/streambuf/xsputn/
     */
    /*streamsize xsputn(const _Elem *s, streamsize n){
      return write(handle, (const void *)s, n);
    }*/
#endif
    
    /**
     * Get sequence of characters
     * 
     * Gets up to n characters from the input sequence and stores them 
     * in the array pointed by s. If less than n characters are available 
     * in the input sequence the function returns all the available 
     * characters, as if successive calls to sbumpc were made until an EOF 
     * (or traits::eof() for other traits) was returned.
     * 
     * Its default behavior in streambuf is to perform the expected behavior 
     * by calling repeatedly the member function sbumpc.
     * 
     * @param s Pointer to a block of memory where the character sequence 
     * is to be stored. 
     * @param n Number of characters to be gotten. This is an integer 
     * value of type streamsize.
     * @return The number of characters gotten
     */
#ifdef _WIN32
    streamsize xsgetn(_Elem *s, streamsize n){
      //std::cerr << "xsgetn()" << std::endl;
      DWORD received;
      ReadFile(handle, s, (DWORD)n, &received, NULL);
      return (streamsize)received;
    }
#else
    /* TODO: unistd.hの低水準I/Oはバッファリングなしのため、
     * 複数文字を引数にとるreadを呼び出しだところで、1文字読みだけで返ってきてしまう
     * streambufの標準挙動であるsbumpc複数回呼び出しで対応
     * @see http://www.cplusplus.com/reference/iostream/streambuf/xsgetn/
     */
    /*streamsize xsgetn(_Elem *s, streamsize n){
      return read(handle, (void *)s, n);
    }*/
#endif
    
    /**
     * Get character in the case of underflow
     * 
     * For the purpose of the streambuf class, underflows happen when a new 
     * character is to be read at the get pointer gptr, but this has reached 
     * the end pointer egptr, indicating that apparently no more characters 
     * are available in the internal input array. 
     * 
     * This function is expected to modify the eback, gptr and egptr pointers 
     * that define the internal input array in such a way that if there are 
     * more characters available in the controlled input sequence after the 
     * location represented by streambuf::egptr, at least some of them are 
     * made available through this internal input array and the new character 
     * available at the get pointer's position itself is returned. Otherwise, 
     * if there are no more characters available in the controlled input 
     * sequence after the one represented by egptr, the function returns EOF 
     * (or traits::eof() for other traits).
     * 
     * Its default behavior in streambuf is to do nothing and return EOF 
     * (or traits::eof() for other traits).
     * 
     * This protected member function is automatically called by sgetc member 
     * function when an underflow happens.
     * 
     * The definition of the member function uflow in streambuf relies on 
     * this member function. The behavior of uflow is the same as the one of 
     * underflow, except that it also advances the get pointer.
     * 
     * @return The new character available at the get pointer position, 
     * if any. Otherwise, EOF (or traits::eof() for other traits) is returned. 
     */
    int_type underflow(){
      //std::cerr << "underflow()" << std::endl;
      if(!in_buf_ready){update_in_buf();}
      return in_buf;
    }
    
    /**
     * Get character in the case of underflow and advance get pointer
     * 
     * For the purpose of the streambuf class, underflows happen when a new 
     * character is to be read at the get pointer gptr, but this has reached 
     * the end pointer egptr, indicating that no more characters are apparently 
     * available in the internal input array.
     * 
     * This function is expected to modify the eback, gptr and egptr pointers 
     * that define the internal input array in such a way that if there are 
     * more characters available in the controlled input sequence after the 
     * location represented by streambuf::egptr, at least some of them are 
     * made available through this internal input array. In this case, the 
     * function returns the new character pointed by the get pointer and 
     * advances it one position. Otherwise, if there are no more characters 
     * available in the controlled input sequence after the one represented by 
     * egptr, the function returns EOF (or traits::eof() for other traits).
     * 
     * Its default behavior in streambuf is to call its sibling virtual member 
     * function underflow and return its value and advance the get pointer in case 
     * of success, or return EOF (or traits::eof() for other traits) otherwise.
     * 
     * This protected member function is automatically called by sbumpc and snextc 
     * member functions when underflows happen.
     * 
     * The behavior of this member function is the same as the one of underflow 
     * except that this function also advances the get pointer by one position.
     * 
     * @return The new character available at the current get pointer position, 
     * if any. Otherwise, EOF (or [=traits::eof() for other traits) is returned. 
     */
    int_type uflow(){
      //std::cerr << "uflow()" << std::endl;
      update_in_buf();
      return in_buf;
    }
};

typedef basic_ComportStreambuf<char, std::char_traits<char> > ComportStreambuf;

class ComportStream : public std::iostream{
  public:
    typedef ComportStreambuf buf_t;
  protected:
    typedef std::iostream super_t;
    buf_t buf;
  public:
    ComportStream(const char *port_spec) throw(std::ios_base::failure)
        : buf(port_spec), super_t(&buf){}
    ~ComportStream(){}
    buf_t &buffer(){return buf;}
};

#endif /* __COMSTREAM_H__ */
