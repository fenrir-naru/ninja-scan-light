require "mkmf"
cflags = " -Wall -I../../.."
$CFLAGS += cflags
$CPPFLAGS += cflags if RUBY_VERSION >= "2.0.0"
$LOCAL_LIBS += " -lstdc++ "
