require 'rspec'
require 'inline'
require 'date'

module Miscellaneous
  inline{|builder|
    builder.add_type_converter('unsigned char', 'NUM2CHR', 'CHR2FIX')
    builder.include("<time.h>") #builder.prefix("#include <time.h>")
    builder.c_singleton <<-__C_CODE__
      unsigned char dow(unsigned char y1900, unsigned char m1, unsigned char d){
        unsigned char y = y1900 - 76; // y > 1980
        static const unsigned char tbl[] = {5, 1, 0, 3, 5, 1, 3, 6, 2, 4, 0, 2};
        if(m1 < 2){y -= 1;}
        return ((y + y/4 + tbl[m1] + d) % 7);
      }
    __C_CODE__
    builder.c_singleton <<-__C_CODE__
      unsigned int dow2(unsigned int y1900, unsigned int m1, unsigned int d){
        struct tm t_tm = {0};
        time_t rawtime;
        t_tm.tm_year = y1900;
        t_tm.tm_mon = m1;
        t_tm.tm_mday = d;
        rawtime = mktime(&t_tm);
        return (t_tm = *localtime(&rawtime)).tm_wday;
      }
    __C_CODE__
  }
end

describe Miscellaneous do
  it "dow" do
    (1980..2099).each{|y|
      (1..12).each{|m|
        wday = Date::new(y, m, 1).wday
        expect(Miscellaneous::dow(y - 1900, m - 1, 1)).to equal(wday)
        expect(Miscellaneous::dow2(y - 1900, m - 1, 1)).to equal(wday)
      }
    }
  end
end
