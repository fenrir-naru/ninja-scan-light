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

#ifndef __SYLPHIDE_STREAM_H__
#define __SYLPHIDE_STREAM_H__

#include <cstddef>

#include "std.h"
#include "util/endian.h"
#include "util/crc.h"

struct SylphideProtocol {
  
  static const unsigned char header[];
  static const unsigned int header_size;
  static const unsigned int capsule_head_size;
  static const unsigned int capsule_tail_size;
  static const unsigned int capsule_size;
  static const unsigned int payload_fixed_length = 0x20;
  
  typedef Uint16 v_u16_t;
  
  template<class Container>
  static v_u16_t calc_crc16(
      Container &target,
      const unsigned int &size,
      const unsigned int &offset,
      v_u16_t crc_u16 = 0){
    
    unsigned int _offset(offset);
    for(int i(0); i < size; i++){
      crc_u16 = CRC16::crc16_generic(target[_offset++], crc_u16);
    }
    return crc_u16;
  }
  
  template<typename T, std::size_t N>
  static v_u16_t calc_crc16(
      T (&target)[N],
      const unsigned int &size,
      const unsigned int &offset,
      v_u16_t crc_u16 = 0){
    return CRC16::crc16((unsigned char *)&target[offset], size, crc_u16);
  }
    
  /**
   * �G���R�[�h���s���֐�
   * 
   */
  struct Encoder {
    /**
     * �G���R�[�h���s�������ʂ̃p�P�b�g�T�C�Y�����߂�
     * 
     * @param payload_size �y�C���[�h�̃T�C�Y
     * @return (unsigned int) �p�P�b�g�̃T�C�Y
     */
    static const unsigned int packet_size(
        const unsigned int &payload_size){
      
      // �y�C���[�h�T�C�Y�m�F
      switch(payload_size){
        case 0: // �y�C���[�h�񏊗L�̏ꍇ�͌Œ蒷(0bytes)
          return capsule_size;
        case payload_fixed_length: // �y�C���[�h���L�A�Œ蒷
          return capsule_size + payload_fixed_length;
        default: // �y�C���[�h���L�A�ϒ�
          return capsule_size + 2 + payload_size;
      }   
    }
    
    /**
     * �G���R�[�h�̑O����
     * 
     * @param target �G���R�[�h���ʂ��i�[�����
     * @param payload_size �y�C���[�h�̃T�C�Y
     * @param request_ack ACK�v����K�v�Ƃ��邩
     * @param ack_reply ACK�ւ̕ԓ���
     * @return (unsigne int) �y�C���[�h�̐擪�ƂȂ�ׂ��I�t�Z�b�g��
     */
    template<class Container>
    static unsigned int preprocess(Container &target,
        const unsigned int &payload_size = payload_fixed_length,
        const bool &request_ack = false,
        const bool &ack_reply = false){
      
      unsigned int payload_start_offset(capsule_head_size);
      
      // �w�b�_�̏�������
      for(int i(0); i < header_size; i++){
        target[i] = header[i];
      }
      
      bool add_payload_size(false);
      switch(payload_size){
        case 0:
          target[header_size - 1] |= 0x02;
          break;
        case payload_fixed_length:
          break;
        default:
          target[header_size - 1] |= 0x01;
          add_payload_size = true;
          break;
      }
      if(request_ack){target[header_size - 1] |= 0x08;}
      if(ack_reply){target[header_size - 1] |= 0x04;}
      
      // �y�C���[�h�T�C�Y���ς̏ꍇ�̓T�C�Y�������Ă���
      if(add_payload_size){
        v_u16_t payload_size_u16(payload_size);
        v_u16_t payload_size_u16_le(
            le_char2_2_num<v_u16_t>(*(char *)&payload_size_u16));
        target[payload_start_offset] 
            = *(unsigned char *)&payload_size_u16_le;
        target[payload_start_offset + 1] 
            = *(((unsigned char *)&payload_size_u16_le) + 1);
        payload_start_offset += sizeof(v_u16_t);
      }
      
      return payload_start_offset;
    }
    
    /**
     * �V�[�P���X�ԍ��𖄂ߍ���
     *
     * @param target �G���R�[�h���ʂ̊i�[��
     * @param sequence_num �V�[�P���X�ԍ�
     */
    template<class Container>
    static void embed_sequence_num(Container &target,
        const unsigned int &sequence_num){

      // �V�[�P���X�ԍ��̏�������
      v_u16_t sequence_num_u16(sequence_num);
      v_u16_t sequence_num_u16_le(
          le_char2_2_num<v_u16_t>(*(char *)&sequence_num_u16));
      target[header_size]
          = *(unsigned char *)&sequence_num_u16_le;
      target[header_size + 1]
          = *(((unsigned char *)&sequence_num_u16_le) + 1);
    }

    /**
     * �G���R�[�h���đ��M����
     * 
     * @param stream ���M�^�[�Q�b�g
     * @param sequence_num �V�[�P���X�ԍ�
     * @param payload �y�C���[�h
     * @param payload_size �y�C���[�h�T�C�Y
     * @param request_ack ACK�v����K�v�Ƃ��邩
     * @param ack_reply ACK�ւ̕ԓ���
     */
    template<class Stream, class Container>
    static void send(
        Stream &stream,
        const unsigned int &sequence_num,
        Container &payload,
        const unsigned int &payload_size = payload_fixed_length,
        const bool &request_ack = false,
        const bool &ack_reply = false){

      unsigned char header_buf[8];

      unsigned int payload_offset(
          preprocess(header_buf, payload_size, request_ack, ack_reply));

      // �V�[�P���X�ԍ��̏�������
      embed_sequence_num(header_buf, sequence_num);

      // �w�b�_(+�V�[�P���X�ԍ�(+�T�C�Y))���M
      stream(header_buf, payload_offset);

      // �y�C���[�h���M
      stream(payload, payload_size);

      // CRC�̕t��
      v_u16_t crc_u16(calc_crc16(
          payload, payload_size, 0,
          calc_crc16(header_buf, payload_offset - header_size, header_size)));

      v_u16_t crc_u16_le(
          le_char2_2_num<v_u16_t>(*(char *)&crc_u16));

      // �t�b�^���M
      stream((unsigned char *)&crc_u16_le, sizeof(crc_u16_le));
    }

    /**
     * �G���R�[�h�̌㏈��
     *
     * @param target �G���R�[�h���ʂ̊i�[��
     * @param sequence_num �V�[�P���X�ԍ�
     * @param whole_size �G���R�[�h�������ʂ̃p�P�b�g�T�C�Y
     */
    template<class Container>
    static void postprocess(Container &target,
        const unsigned int &sequence_num,
        const unsigned int &whole_size){
      
      // �V�[�P���X�ԍ��̏�������
      embed_sequence_num(target, sequence_num);
      
      // CRC�̕t��
      v_u16_t crc_u16(calc_crc16(
          target, whole_size - capsule_tail_size - header_size, header_size));
      
      v_u16_t crc_u16_le(
          le_char2_2_num<v_u16_t>(*(char *)&crc_u16));
      target[whole_size - sizeof(v_u16_t)]
          = *(unsigned char *)&crc_u16_le;
      target[whole_size - sizeof(v_u16_t) + 1]
          = *(((unsigned char *)&crc_u16_le) + 1);
      
#if DEBUG > 3
      std::cerr << sequence_num_u16 << "=>" << sequence_num_u16_le << std::endl;
      std::cerr << crc_u16 << "=>" << crc_u16_le << std::endl;
      
      std::cerr << "--write--" << std::endl;
      std::cerr << hex;
      for(int i = 0; i < whole_size; i++){
        std::cerr 
          << setfill('0') 
          << setw(2)
          << (unsigned int)((unsigned char)target[i]) << ' ';
      }
      std::cerr << std::endl << dec;
#endif
    }
  };
  
  struct Decorder {
    
    /**
     * �擪�`�F�b�N���s��
     * 
     * @param target ���̓X�g���[��
     * @return (bool) �擪�`�F�b�N�����������ꍇtrue
     */
    template<class Container>
    static bool valid_head(Container &target){
      // �ŏ��T�C�Y�ŃT�C�Y�m�F
      if(capsule_size > target.stored()){
        return false;
      }
      // �w�b�_�m�F
      if(((unsigned char)target[0] != header[0])
          || (((unsigned char)target[1] & 0xF0) != header[1])){
        return false;
      }
      return true;
    }
    
    /**
     * �y�C���[�h�̃T�C�Y�����߂�
     * 
     * @param target ���̓X�g���[��
     * @return (unsigned int) �y�C���[�h�̃T�C�Y
     */
    template<class Container>
    static unsigned int payload_size(Container &target){
      
      // �y�C���[�h�T�C�Y�m�F
      switch((unsigned char)target[1] & 0x03){
        case 1: { // �y�C���[�h���L�A�ϒ�
          unsigned char buf[] = {
              (unsigned char)target[4], (unsigned char)target[5]};
          return le_char2_2_num<v_u16_t>(*(const char *)buf);
        }
        case 2: // �y�C���[�h�񏊗L�A�Œ蒷(0bytes)
          return 0;
        case 3: // �y�C���[�h�񏊗L�A�ϒ�(0bytes)
          return 0;
        default: // �y�C���[�h���L�A�Œ蒷(32bytes)
          return payload_fixed_length;
      }   
    }
    
    /**
     * �p�P�b�g�̃T�C�Y�����߂�
     * 
     * @param target ���̓X�g���[��
     * @return (unsigned int) �p�P�b�g�̃T�C�Y
     */
    template<class Container>
    static unsigned int packet_size(Container &target){
      
      // �y�C���[�h�T�C�Y�m�F
      switch((unsigned char)target[1] & 0x03){
        case 1: { // �y�C���[�h���L�A�ϒ�
          unsigned char buf[] = {
              (unsigned char)target[4], (unsigned char)target[5]};
          return le_char2_2_num<v_u16_t>(*(const char *)buf) + capsule_size + 2;
        }
        case 2: // �y�C���[�h�񏊗L�A�Œ蒷(0bytes)
          return capsule_size;
        case 3: // �y�C���[�h�񏊗L�A�ϒ�(0bytes)
          return capsule_size + 2;
        default: // �y�C���[�h���L�A�Œ蒷(32bytes)
          return payload_fixed_length + capsule_size;
      }
    }
    
    /**
     * �T�C�Y�`�F�b�N���s��
     * 
     * @param target ���̓X�g���[��
     * @return (bool) �擪�`�F�b�N�����������ꍇtrue
     */
    template<class Container>
    static bool valid_size(Container &target){
      // �ŏ��T�C�Y�ŃT�C�Y�m�F
      return packet_size(target) <= target.stored();
    }
    
    /**
     * �V�[�P���X�ԍ������߂�
     * 
     * @param target ���̓X�g���[��
     * @return (unsigned int) �V�[�P���X�ԍ�
     */
    template<class Container>
    static unsigned int sequence_num(Container &target){
          
      // �V�[�P���X�ԍ�
      unsigned char buf[] = {
          (unsigned char)target[2], (unsigned char)target[3]};
      return le_char2_2_num<v_u16_t>(*(const char *)buf);    
    }
    
    /**
     * ACK�v����K�v�Ƃ��Ă��邩�ǂ���
     * 
     * @param target ���̓X�g���[��
     * @return (bool) ACK�v����K�v�Ƃ��Ă���ꍇtrue
     */
    template<class Container>
    static bool is_request_ack(Container &target){
      
      return (target[header_size - 1] & 0x08);
    }
    
    /**
     * ACK�ւ̕ԓ����ǂ���
     * 
     * @param target ���̓X�g���[��
     * @return (bool) ACK�ւ̕ԓ��̏ꍇtrue
     */
    template<class Container>
    static bool is_ack_reply(Container &target){
      
      return (target[header_size - 1] & 0x04);
    }
    
    /**
     * �y�C���[�h�����o��
     * 
     * @param target ���̓X�g���[��
     * @param buf �R�s�[��
     * @param whole_size �p�P�b�g�S�̂̃T�C�Y
     * @param extract_size �y�C���[�h�T�C�Y
     */
    template<class Container, class BufferT>
    static void extract_payload(
        Container &target, BufferT &buf,
        const unsigned int &whole_size, const unsigned int &extract_size){
      
      for(int i(0), j(whole_size - extract_size - capsule_tail_size); 
          i < extract_size; 
          i++, j++){
        buf[i] = target[j];
      }
    }
    
    /**
     * �p�P�b�g���L�����ǂ����m�F����
     * 
     * @param target ���̓X�g���[��
     * @return (bool) �L���̏ꍇtrue
     */
    template<class Container>
    static bool ready(Container &target){
      // �w�b�_�̊m�F
      return valid_head(target) && valid_size(target);
    }
    
    /**
     * �f�[�^�̑Ó��������؂���
     * 
     * @param target ���̓X�g���[��
     * @return (bool) �y�C���[�h�̃T�C�Y
     */
    template<class Container>
    static bool validate(Container &target){
      const unsigned int whole_size(packet_size(target));
        
      // CRC�m�F
      v_u16_t crc16_orig, crc16_calc;
      {
        unsigned char buf[] = {
            (unsigned char)target[whole_size - capsule_tail_size], 
            (unsigned char)target[whole_size - capsule_tail_size + 1]};
        crc16_orig = le_char2_2_num<v_u16_t>(*(const char *)buf);
        
        crc16_calc = calc_crc16(
            target, whole_size - capsule_tail_size - header_size, header_size);
      }
        
#if DEBUG > 3
      std::cerr << "! " << hex
          << setfill('0') << setw(4) << (unsigned int)crc16_orig 
          << " vs " 
          << setfill('0') << setw(4) << (unsigned int)crc16_calc
          << std::endl;
      
      std::cerr << "--read--" << std::endl;
      std::cerr << hex;
      for(int i(0); i < whole_size; i++){
        std::cerr 
          << setfill('0') 
          << setw(2)
          << (unsigned int)((unsigned char)target[i]) << ' ';
      }
      std::cerr << std::endl << dec;
#endif
      
      return crc16_orig == crc16_calc;
    }
  };
};

const unsigned char SylphideProtocol::header[] 
    = {0xF7, 0xE0};

const unsigned int SylphideProtocol::header_size 
    = sizeof(SylphideProtocol::header);

const unsigned int SylphideProtocol::capsule_head_size 
    = SylphideProtocol::header_size 
        + sizeof(SylphideProtocol::v_u16_t);

const unsigned int SylphideProtocol::capsule_tail_size 
    = sizeof(SylphideProtocol::v_u16_t);

const unsigned int SylphideProtocol::capsule_size 
    = SylphideProtocol::capsule_head_size
        + SylphideProtocol::capsule_tail_size;

#ifndef __TI_COMPILER_VERSION__
// DSP�����̏ꍇ�Astream/STL�֘A�̋@�\�̓J�b�g���Ă���

#include <streambuf>
#include <istream>
#include <ostream>
#include <cstring>

#include <deque>

template<
    class _Elem, 
    class _Traits>
class basic_SylphideStreambuf_out : public std::basic_streambuf<_Elem, _Traits>{
  
  protected:
    typedef std::basic_streambuf<_Elem, _Traits> super_t;
    typedef std::streamsize streamsize;
    typedef typename super_t::int_type int_type;
    
    std::ostream &out;
    unsigned int payload_size, packet_size;
    _Elem *packet;
    unsigned int packet_bufsize;
    unsigned int sequence_num;
    bool sequence_num_lock;
    
    using super_t::pbase;
    using super_t::pptr;
    using super_t::epptr;
    using super_t::pbump;
    using super_t::setp;
    
    int_type overflow(int_type c = _Traits::eof()){
      // �G���R�[�h��S��
      //std::cerr << "overflow()" << std::endl;
      
      if(c != _Traits::eof()){
        *epptr() = _Traits::to_char_type(c);
        SylphideProtocol::Encoder::postprocess(
            packet, sequence_num, packet_size);
      
        // �w�b�_�A�V�[�P���X�ԍ��A�{���ACRC16�̏��ɑ��M����
        out.write(packet, packet_size);
        if(out.good()){
          setp(pbase(), epptr());
          if(!sequence_num_lock){sequence_num++;}
          return true;
        }
      }
      return _Traits::eof();
    }
    
  public:
    void set_payload_size(const unsigned int &new_size){
      payload_size = new_size;
      packet_size = SylphideProtocol::Encoder::packet_size(payload_size);
      
      if(packet_bufsize < packet_size){
        delete [] packet;
        packet_bufsize = packet_size;
        packet = new _Elem[packet_bufsize];
      }
      
      _Elem *payload_head(packet
          + SylphideProtocol::Encoder::preprocess(
              packet, payload_size));
      setp(payload_head, payload_head + payload_size - 1);
    }
    
    /**
     * �R���X�g���N�^
     * 
     * �o�̓t�B���^�Ƃ��Ďg�p����ꍇ�B���̎��A�G���R�[�_������������B
     * @param out �o�̓X�g���[��
     */
    basic_SylphideStreambuf_out(
        std::ostream &_out,
        const unsigned int &size = SylphideProtocol::payload_fixed_length)
        : out(_out), payload_size(0), packet_size(0),
        packet(NULL), packet_bufsize(0),
        sequence_num(0), sequence_num_lock(false) {
      set_payload_size(size);
    }
    ~basic_SylphideStreambuf_out(){
      delete [] packet;
    }
    
    unsigned int &sequence_number(){
      return sequence_num;
    }
    bool &sequence_number_lock(){
      return sequence_num_lock;
    }
};

template<
    class _Elem, 
    class _Traits>
class basic_SylphideStreambuf_in : public std::basic_streambuf<_Elem, _Traits>{
  
  public:
    class container_t : public std::deque<_Elem> {
      typedef std::deque<_Elem> super_t;
      protected:
        std::istream &in;
      public:
        container_t(std::istream &_in) : super_t(), in(_in) {}
        ~container_t(){}
        bool pull(unsigned int n){
          while(n--){
            _Elem c;
            in.get(c);
            if(!in.good()){return false;}
            super_t::push_back(c);
          }
          return true;
        }
        void skip(unsigned int n){
          typename super_t::iterator it1(super_t::begin()), it2(it1);
          std::advance(it2, n);
          super_t::erase(it1, it2);
        }
        typename super_t::size_type stored() const {
          return super_t::size();
        }
    };
  
  protected:
    typedef std::basic_streambuf<_Elem, _Traits> super_t;
    typedef std::streamsize streamsize;
    typedef typename super_t::int_type int_type;
    
    container_t buffer;
    bool mode_fixed_size; ///< ���܂��������̃p�P�b�g�����E��Ȃ��悤�ɂ��郂�[�h
    unsigned int payload_size;
    _Elem *payload;
    unsigned int payload_bufsize;
    unsigned int sequence_num;
    bool sequence_num_lock;
    
    void regulate_payload(unsigned int min_size){
      if(payload_bufsize < min_size){
        delete [] payload;
        payload_bufsize = min_size;
        payload = new _Elem[payload_bufsize];
      }
    }
    
    using super_t::eback;
    using super_t::gptr;
    using super_t::egptr;
    using super_t::setg;
    
  protected:
    int_type underflow(){
      // �f�R�[�h��S��
      //std::cerr << "underflow()" << std::endl;
      
      unsigned int buffer_size_min(SylphideProtocol::capsule_size);
      bool header_checked(false);
      while(true){
        if(buffer.stored() < buffer_size_min){
          if(!buffer.pull(buffer_size_min - buffer.stored())){
            return _Traits::eof();
          }
        }
        if(!header_checked){
          if(!SylphideProtocol::Decorder::valid_head(buffer)){
            buffer.skip(1);
          }else{
            header_checked = true;
            buffer_size_min = SylphideProtocol::Decorder::packet_size(buffer);
          }
          continue;
        }
        
        if(SylphideProtocol::Decorder::validate(buffer)){
          unsigned int new_payload_size(
              SylphideProtocol::Decorder::payload_size(buffer));
          if(new_payload_size){
            if(mode_fixed_size){
              if(payload_size == new_payload_size){break;}
            }else{
              payload_size = new_payload_size;
              break;
            }
          }
          buffer.skip(buffer_size_min);
        }else{
          buffer.skip(1);
        }
        buffer_size_min = SylphideProtocol::capsule_size;
        header_checked = false;
      }
      
      if(!sequence_num_lock){
        sequence_num 
            = SylphideProtocol::Decorder::sequence_num(buffer);
      }
      regulate_payload(payload_size);
      
      SylphideProtocol::Decorder::extract_payload(
          buffer, payload, buffer_size_min, payload_size);
      
      setg(payload, payload, payload + payload_size);
      buffer.skip(buffer_size_min);
      
      return _Traits::to_int_type(*gptr());
    }
    
  public:
    /**
     * �R���X�g���N�^
     * 
     * �f�R�[�_�Ƌ�������
     * 
     * @param in ���̓X�g���[�� 
     */
    basic_SylphideStreambuf_in(std::istream &_in)
        : buffer(_in), payload_size(0), 
        payload(NULL), payload_bufsize(0),
        sequence_num(0), sequence_num_lock(false), mode_fixed_size(false) {
      setg(payload, payload, payload);
    }
    /**
     * �R���X�g���N�^
     * 
     * �f�R�[�_�Ƌ�������A�����ē���̒����̃p�P�b�g�����E��Ȃ��悤�ɂ���
     * 
     * @param in ���̓X�g���[��
     * @param payload_size �p�P�b�g��̃y�C���[�h�T�C�Y(0���w�肷��Ɨl�X�Ȓ����̃p�P�b�g���E��) 
     */
    basic_SylphideStreambuf_in(
        std::istream &_in, const unsigned int size)
        : buffer(_in), payload_size(size), 
        payload(NULL), payload_bufsize(0),
        sequence_num(0), sequence_num_lock(false), mode_fixed_size(size > 0) {
      setg(payload, payload, payload);
    }
    ~basic_SylphideStreambuf_in(){
      delete [] payload;
    }
    
    unsigned int sequence_number() const {
      return sequence_num;
    }
    bool &sequence_number_lock(){
      return sequence_num_lock;
    }
};

template<class _Elem, class _Traits>
class basic_SylphideIStream : public std::istream {
  public:
    typedef basic_SylphideStreambuf_in<_Elem, _Traits> buf_t;
  protected:
    typedef std::istream super_t;
    buf_t buf;
  public:
    basic_SylphideIStream(std::istream &in)
        : buf(in), super_t(&buf){}
    basic_SylphideIStream(std::istream &in, const unsigned int payload_size)
        : buf(in, payload_size), super_t(&buf){}
    ~basic_SylphideIStream(){}
    unsigned int current_sequence() const {
      return const_cast<buf_t &>(buf).sequence_number();
    }
};

template<class _Elem, class _Traits>
class basic_SylphideOStream : public std::ostream {
  public:
    typedef basic_SylphideStreambuf_out<_Elem, _Traits> buf_t;
  protected:
    typedef std::ostream super_t;
    buf_t buf;
  public:
    basic_SylphideOStream(std::ostream &out,
        const unsigned int &payload_size = SylphideProtocol::payload_fixed_length)
        : buf(out, payload_size), super_t(&buf){}
    ~basic_SylphideOStream(){}
    void set_payload_size(const unsigned int &new_size){
      buf.set_payload_size(new_size);
    }
    unsigned int &sequence() {
      return buf.sequence_number();
    }
    bool &sequence_lock() {
      return buf.sequence_number_lock();
    }
};

typedef
    basic_SylphideIStream<char, std::char_traits<char> >
    SylphideIStream;

typedef
    basic_SylphideOStream<char, std::char_traits<char> >
    SylphideOStream;


#endif

#endif /* __SYLPHIDE_STREAM_H__ */
