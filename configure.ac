#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

# to generate ./configure run:
# 	$ autoconf

# to clean up autoconf files, run:
#	$ make realclean

AC_INIT(mpt3lowcom, 1.0, michal.kvasnica@stuba.sk)
# no config.log please
AC_RUN_LOG(:)

# check if the configuration is complete
AC_CONFIG_SRCDIR(configure.ac)

# Checks for programs.
AC_PROG_SED
AC_PROG_GREP
AC_PROG_MAKE_SET
AC_PROG_LN_S

AC_CHECK_PROG(CURL, curl, curl, no)
if test "$CURL" = no ; then
	AC_MSG_ERROR(Cannot find "curl".)
fi

AC_CHECK_PROG(WGET, wget, wget, no)
if test "$WGET" = no ; then
	AC_MSG_ERROR(Cannot find wget. Please, install "wget" program.)
fi

AC_CHECK_PROG(UNZIP, unzip, unzip, no)
if test "$UNZIP" = no ; then
	AC_MSG_ERROR(Cannot find unzip.)
fi

AC_CHECK_PROG(GUNZIP, gunzip, gunzip, no)
if test "$GUNZIP" = no ; then
	AC_MSG_ERROR(Cannot find gunzip.)
fi

AC_CHECK_PROG(GZIP, gzip, gzip, no)
if test "$GZIP" = no ; then
	AC_MSG_ERROR(Cannot find gzip.)
fi

AC_CHECK_PROG(SCP, scp, scp, no)
if test "$SCP" = no ; then
	AC_MSG_ERROR(Cannot find scp. Please, install "openssh" package.)
fi

AC_CHECK_PROG(RSYNC, rsync, rsync, no)
if test "$RSYNC" = no ; then
	AC_MSG_ERROR(Cannot find "rsync". Please, install "rsync" package.)
fi

# Checks for libraries.

# Checks for header files.

# Checks for typedefs, structures, and compiler characteristics.

# Checks for library functions.

AC_CONFIG_FILES([Makefile])
AC_OUTPUT
