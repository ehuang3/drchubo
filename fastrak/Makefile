# Project Name
PROJECT := fastrak

# Project Version
VERSION := 20101115

# Binary Files
BINFILES := fastrakd

# Library files
#SHAREDLIBS :=

all: default

CC := g++
include /usr/share/make-common/common.1.mk

CPPFLAGS +=  -O0
CFLAGS += --std=gnu99 -O0

default: $(LIBFILES) $(BINFILES)

$(call LINKBIN, fastrakd, fastrak.o fastrakd.o, amino somatic stdc++ ach protobuf-c)

.PHONY: default clean


clean:
	rm -fr *.o $(BINFILES) $(LIBFILES) *.o debian *.deb *.lzma
depclean: clean
	rm -fr .deps
