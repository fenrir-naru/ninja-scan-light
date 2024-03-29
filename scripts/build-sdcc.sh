#!/bin/bash
set -ev
: ${SDCC:=3.3.0}
: ${SDCC_DIR:=/usr/local}

if [[ $(sdcc --version) =~ "${SDCC}" ]]; then
  echo "Found sdcc-${SDCC}, build skipped."
  exit 0
fi

if [ ! -d sdcc-${SDCC} ]; then
  SDCC_SRC=sdcc-src-${SDCC}.tar.bz2
  if [ -f $(dirname "$0")/src/${SDCC_SRC} ]; then
    SDCC_SRC=$(dirname "$0")/src/${SDCC_SRC}
  elif [ ! -f ${SDCC_SRC} ]; then
    # dead link?
    wget "https://downloads.sourceforge.net/project/sdcc/sdcc/${SDCC}/${SDCC_SRC}" -O ${SDCC_SRC}
  fi
  mkdir sdcc-${SDCC} && tar jvxf ${SDCC_SRC} -C sdcc-${SDCC} --strip-components 1
fi

cd sdcc-${SDCC}

case "${SDCC}" in
3.[34].*)
(cd sdas/linksrc && patch -u -N -p1 <<'__PATCH_LINE__' || let "$?<=1")
diff -uprN linksrc.orig/lkar.c linksrc/lkar.c
--- linksrc.orig/lkar.c 2019-09-16 03:01:13.278101100 +0900
+++ linksrc/lkar.c      2019-09-16 02:59:47.520475600 +0900
@@ -47,7 +47,7 @@ along with this program.  If not, see <h


 char *
-strndup (const char *str, size_t len)
+strndup_ (const char *str, size_t len)
 {
   char *s = (char *) malloc (len + 1);
   memcpy (s, str, len);
@@ -98,7 +98,7 @@ get_long_name (const char *name)
                 while (*++n != '\n')
                   assert (n < &str_tab[str_tab_size]);

-              return strndup (name, n - name);
+              return strndup_ (name, n - name);
             }
         }
     }
@@ -164,7 +164,7 @@ get_member_name (char *name, size_t *p_s
               while (name[++len] == ' ')
                 ;
               if (len == AR_NAME_LEN)
-                return strndup (name, p - name);
+                return strndup_ (name, p - name);
             }
           else
             {
@@ -173,7 +173,7 @@ get_member_name (char *name, size_t *p_s
               p = name + AR_NAME_LEN;
               while (*--p == ' ' && p >= name)
                 ;
-              return strndup (name, p - name + 1);
+              return strndup_ (name, p - name + 1);
             }
         }

__PATCH_LINE__
;;
esac

case "${SDCC}" in
3.[34].*)
patch -u -N -p1 <<'__PATCH_LINE__' || let "$?<=1"
Index: sdcc-3.4.0/support/sdbinutils/bfd/bfd-in.h
===================================================================
--- sdcc-3.4.0.orig/support/sdbinutils/bfd/bfd-in.h  2012-11-05 21:26:25.000000000 +0800
+++ sdcc-3.4.0/support/sdbinutils/bfd/bfd-in.h 2014-06-19 16:40:09.274149785 +0800
@@ -294,9 +294,9 @@
 
 #define bfd_is_com_section(ptr) (((ptr)->flags & SEC_IS_COMMON) != 0)
 
-#define bfd_set_section_vma(bfd, ptr, val) (((ptr)->vma = (ptr)->lma = (val)), ((ptr)->user_set_vma = TRUE), TRUE)
-#define bfd_set_section_alignment(bfd, ptr, val) (((ptr)->alignment_power = (val)),TRUE)
-#define bfd_set_section_userdata(bfd, ptr, val) (((ptr)->userdata = (val)),TRUE)
+#define bfd_set_section_vma(bfd, ptr, val) (((ptr)->vma = (ptr)->lma = (val)), ((ptr)->user_set_vma = TRUE))
+#define bfd_set_section_alignment(bfd, ptr, val) ((ptr)->alignment_power = (val))
+#define bfd_set_section_userdata(bfd, ptr, val) ((ptr)->userdata = (val))
 /* Find the address one past the end of SEC.  */
 #define bfd_get_section_limit(bfd, sec) \
   (((bfd)->direction != write_direction && (sec)->rawsize != 0 \
@@ -519,7 +519,7 @@
 
 #define bfd_get_symbol_leading_char(abfd) ((abfd)->xvec->symbol_leading_char)
 
-#define bfd_set_cacheable(abfd,bool) (((abfd)->cacheable = bool), TRUE)
+#define bfd_set_cacheable(abfd,bool) ((abfd)->cacheable = bool)
 
 extern bfd_boolean bfd_cache_close
   (bfd *abfd);
Index: sdcc-3.4.0/support/sdbinutils/bfd/bfd-in2.h
===================================================================
--- sdcc-3.4.0.orig/support/sdbinutils/bfd/bfd-in2.h 2012-11-05 21:26:25.000000000 +0800
+++ sdcc-3.4.0/support/sdbinutils/bfd/bfd-in2.h  2014-06-19 16:42:09.110092421 +0800
@@ -301,9 +301,9 @@
 
 #define bfd_is_com_section(ptr) (((ptr)->flags & SEC_IS_COMMON) != 0)
 
-#define bfd_set_section_vma(bfd, ptr, val) (((ptr)->vma = (ptr)->lma = (val)), ((ptr)->user_set_vma = TRUE), TRUE)
-#define bfd_set_section_alignment(bfd, ptr, val) (((ptr)->alignment_power = (val)),TRUE)
-#define bfd_set_section_userdata(bfd, ptr, val) (((ptr)->userdata = (val)),TRUE)
+#define bfd_set_section_vma(bfd, ptr, val) (((ptr)->vma = (ptr)->lma = (val)), ((ptr)->user_set_vma = TRUE))
+#define bfd_set_section_alignment(bfd, ptr, val) ((ptr)->alignment_power = (val))
+#define bfd_set_section_userdata(bfd, ptr, val) ((ptr)->userdata = (val))
 /* Find the address one past the end of SEC.  */
 #define bfd_get_section_limit(bfd, sec) \
   (((bfd)->direction != write_direction && (sec)->rawsize != 0 \
@@ -526,7 +526,7 @@
 
 #define bfd_get_symbol_leading_char(abfd) ((abfd)->xvec->symbol_leading_char)
 
-#define bfd_set_cacheable(abfd,bool) (((abfd)->cacheable = bool), TRUE)
+#define bfd_set_cacheable(abfd,bool) ((abfd)->cacheable = bool)
 
 extern bfd_boolean bfd_cache_close
   (bfd *abfd);
__PATCH_LINE__
;;
esac

case "${SDCC}" in
3.3.*)
patch -u -N -p1 <<'__PATCH_LINE__' || let "$?<=1"
diff -uprN sdcc-3.3.0.orig/support/sdbinutils/bfd/Makefile.in sdcc-3.3.0/support/sdbinutils/bfd/Makefile.in
--- sdcc-3.3.0.orig/support/sdbinutils/bfd/Makefile.in  2012-11-08 16:44:00.000000000 +0900
+++ sdcc-3.3.0/support/sdbinutils/bfd/Makefile.in 2023-08-15 15:42:47.661249300 +0900
@@ -239,7 +239,7 @@ STRIP = @STRIP@
 TDEFINES = @TDEFINES@
 USE_NLS = @USE_NLS@
 VERSION = @VERSION@
-WARN_CFLAGS = @WARN_CFLAGS@
+WARN_CFLAGS = @WARN_CFLAGS@ -Wno-format-overflow -Wno-pointer-compare -Wno-implicit-fallthrough -Wno-unused-const-variable -Wno-cast-function-type
 XGETTEXT = @XGETTEXT@
 abs_builddir = @abs_builddir@
 abs_srcdir = @abs_srcdir@
diff -uprN sdcc-3.3.0.orig/support/sdbinutils/binutils/Makefile.in sdcc-3.3.0/support/sdbinutils/binutils/Makefile.in
--- sdcc-3.3.0.orig/support/sdbinutils/binutils/Makefile.in 2012-11-08 23:22:15.000000000 +0900
+++ sdcc-3.3.0/support/sdbinutils/binutils/Makefile.in  2023-08-15 15:44:30.335377200 +0900
@@ -264,7 +264,7 @@ SHELL = @SHELL@
 STRIP = @STRIP@
 USE_NLS = @USE_NLS@
 VERSION = @VERSION@
-WARN_CFLAGS = @WARN_CFLAGS@
+WARN_CFLAGS = @WARN_CFLAGS@ -Wno-format-overflow -Wno-pointer-compare -Wno-implicit-fallthrough -Wno-unused-const-variable -Wno-cast-function-type
 XGETTEXT = @XGETTEXT@
 YACC = `if [ -f ../bison/bison ]; then echo ../bison/bison -y -L$(srcdir)/../bison/; else echo @YACC@; fi`
 YFLAGS = -d
__PATCH_LINE__
;;
esac

DISABLE_DEVICES="z80 z180 r2k r3ka gbz80 ds390 ds400 pic14 pic16 hc08 s08"
CONFIGURE_OPT="--prefix=${SDCC_DIR} --disable-ucsim --disable-sdcdb --disable-non-free" 
case "${SDCC}" in
3.[45678].*)
DISABLE_DEVICES+=" tlcs90 stm8";;
3.[9].* | 4.0.*)
DISABLE_DEVICES+=" tlcs90 stm8 ez80_z80 pdk13 pdk14 pdk15";;
4.1.*)
DISABLE_DEVICES+=" tlcs90 stm8 ez80_z80 pdk13 pdk14 pdk15 r2ka z80n";;
esac
for dev in ${DISABLE_DEVICES}; do
  CONFIGURE_OPT+=" --disable-${dev}-port";
done

./configure ${CONFIGURE_OPT} \
&& make -j \
&& make install