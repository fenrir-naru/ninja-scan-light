require 'rspec'
require 'inline'
require 'date'

module Miscellaneous
  inline{|builder|
    #builder.prefix("")
    builder.c_singleton <<-__C_CODE__
      unsigned int dow(unsigned int y1900, unsigned int m1, unsigned int d){
        unsigned int y = y1900 - 76; // y > 1980
        static const unsigned int tbl[] = {5, 1, 0, 3, 5, 1, 3, 6, 2, 4, 0, 2};
        if(m1 < 2){y -= 1;}
        return ((y + y/4 + tbl[m1] + d) % 7);
      }
    __C_CODE__
  }
end

describe Miscellaneous do
  let "dow" do
    (1980..2099).each{|y|
      (1..12).each{|m|
        expect(Miscellaneous::dow(y - 1900, m - 1, 1)).to equal(Date::new(y, m, 1).wday)
      }
    }
  end
end
