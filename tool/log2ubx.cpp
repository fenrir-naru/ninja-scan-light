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

#if defined(_MSC_VER) && _MSC_VER >= 1400
#define _USE_MATH_DEFINES
//#define _CRT_SECURE_NO_WARNINGS // sprintf_s等への推奨を止める
#endif

#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <exception>
#include <cstring>

#define DEBUG 1

#define IS_LITTLE_ENDIAN 1
#include "SylphideProcessor.h"
#include "SylphideStream.h"

typedef double float_sylph_t;

#include "analyze_common.h"

struct Options : public GlobalOptions<float_sylph_t> {
  typedef GlobalOptions<float_sylph_t> super_t;
  bool log_is_ubx; ///< ubx2ubxを実現するためのフラグ
  
  Options()
      : super_t(), log_is_ubx(false) {}
  ~Options(){}
  
  /**
   * コマンドに与えられた設定を読み解く
   * 
   * @param spec コマンド
   * @return (bool) 解読にヒットした場合はtrue、さもなければfalse
   */
  bool check_spec(const char *spec){
    using std::cerr;
    using std::endl;
    
    static const char *available_keys[] = {
        "start_gpst", "start-gpst",
        "end_gpst", "end-gpst",
        "out",
        "in_sylphide"};
    
    const char *value;
    if(value = get_value(spec, "log_is_ubx")){
      log_is_ubx = is_true(value);
      std::cerr << "log_is_ubx" << ": " << (log_is_ubx ? "true" : "false") << std::endl;
      return true;
    }

    for(int i(0); 
        i < sizeof(available_keys) / sizeof(available_keys[0]);
        i++){
      if(value = get_value(spec, available_keys[i])){
        return super_t::check_spec(spec);
      }
    }
    
    return false;
  }
} options;

using namespace std;

typedef SylphideProcessor<> Processor_t;
typedef Processor_t::G_Observer_t G_Observer_t;

#define BUFFER_SIZE (PAGE_SIZE * 1) // 32
#define OBSERVER_SIZE (PAGE_SIZE * 64) // 2048

int good_packet(0);
int bad_packet(0);

double itow_0x0106(0);
bool read_continue(true);

/**
 * Gページ(u-bloxのGPS)の処理用関数
 * Gページの内容が正しいかvalidateで確認した後、実処理を行うこと。
 * {class, id} = {0x01, 0x02}のとき位置情報
 * {class, id} = {0x01, 0x12}のとき速度情報
 * {class, id} = {0x01, 0x03}のとき受信機状態
 * {class, id} = {0x01, 0x04}のときDOP
 * {class, id} = {0x01, 0x06}のとき解情報
 * {class, id} = {0x01, 0x30}のとき衛星情報
 * {class, id} = {0x02, 0x10}のとき生情報
 * 
 * @param obsrever Gページのオブザーバー
 */
void g_packet_handler(const G_Observer_t &observer){
  if(!observer.validate()){
    bad_packet++;
    return;
  }

  G_Observer_t::packet_type_t packet_type(observer.packet_type());
  if((packet_type.mclass == 0x01) && (packet_type.mid == 0x06)){
    G_Observer_t::solution_t solution(observer.fetch_solution());
    if(solution.status_flags & G_Observer_t::solution_t::TOW_VALID){
      itow_0x0106 = observer.fetch_ITOW();
    }
    if(itow_0x0106 >= options.end_gpstime){
      read_continue = false;
      return;
    }
  }

  if(itow_0x0106 < options.start_gpstime){return;}
  good_packet++;
  for(int i = 0; i < observer.current_packet_size(); i++){
    options.out() << observer[i];
  }
}

/**
 * ファイル等のストリームからページ単位で切り出す関数
 * 
 * @param in ストリーム
 */
void stream_processor(istream &in){
  char buffer[PAGE_SIZE];
  char *buffer_head(buffer);
  int read_count_max(sizeof(buffer));
  
  if(options.log_is_ubx){
    buffer[0] = 'G';
    buffer_head++;
    read_count_max--;
  }

  Processor_t processor(OBSERVER_SIZE);       // ストリーム処理機を生成
  processor.set_g_handler(g_packet_handler);  // Gページの際の処理を登録
  
  int count(0);

  while(read_continue && (!in.eof())){
    count++;

    in.read(buffer_head, read_count_max);
    if(in.gcount() < read_count_max){
      continue;
    }

    processor.process(buffer, sizeof(buffer));
  }
}

int main(int argc, char *argv[]){

  cerr << "NinjaScan converter to make ubx format GPS data." << endl;
  cerr << "Usage: (exe) [options] log.dat" << endl;
  if(argc < 2){
    cerr << "(error!) Too few arguments; " << argc << " < min(2)" << endl;
    return -1;
  }
  
  int log_index(1); // log file name is assumed to be given by argv[1]

  options._out = NULL;

  // Check options
  for(int i(1); i < argc; i++){
    if(options.check_spec(argv[i])){continue;}
    // if arg is not an option, assume arg as log file name
    if(log_index != 1){ // Detect unknown option by multiple substitution to log_index.
      cerr << "(error!) Unknown option!! : " << argv[i] << endl;
      return -1;
    }
    log_index = i;
  }
  
  // Setup default output if output is not specified
  if(!options._out){
    string out_fname(argv[log_index]);
    string::size_type index = out_fname.find_last_of('.');
    if(index != string::npos){
      out_fname.erase(index);
    }
    out_fname.append(".ubx");
    cerr << "Output: ";
    options._out = &(options.spec2ostream(out_fname.c_str(), true));
  }
  
  // Input is in SylphideProtocol or log.dat
  if(options.in_sylphide){
    SylphideIStream sylphide_in(options.spec2istream(argv[log_index]), PAGE_SIZE);
    stream_processor(sylphide_in);
  }else{
    stream_processor(options.spec2istream(argv[log_index]));
  }
  
  cerr << "Good, Bad = " 
       << good_packet << ", " << bad_packet << endl;
  
  return 0;
}
