# FATFS files.
FATFSSRC = ${CHIBIOS}/os/various/fatfs_bindings/fatfs_diskio.c \
           ${CHIBIOS}/os/various/fatfs_bindings/fatfs_syscall.c \
           ${FATFS}/src/ff.c \
           ${FATFS}/src/option/unicode.c

FATFSINC = ${FATFS}/src
