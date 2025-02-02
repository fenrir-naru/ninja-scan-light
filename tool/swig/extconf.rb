require "mkmf"
cflags = " -Wall -I../../.. -O3" # -march=native
RE_optflags = /(?<=^|\s)-O(?:[0-3sgz]|fast)?/
if RE_optflags =~ cflags then
  $CFLAGS.gsub!(RE_optflags, '')
  $CXXFLAGS.gsub!(RE_optflags, '')
end
$CFLAGS << cflags
$CXXFLAGS << cflags
$LOCAL_LIBS += " -lstdc++ "
