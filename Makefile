################################################################################
#author:zyh
#date:2022-11-17
#func:
################################################################################

TARGET:=libv2x_asn.so

ifneq ($(wildcard /home/share/06-cross_tools/01-Jeson-NANO/tool/bin/aarch64-linux-gnu-gcc),)
 CC=/home/share/06-cross_tools/01-Jeson-NANO/tool/bin/aarch64-linux-gnu-gcc
else
 CC=gcc
endif

SRCS:= $(wildcard v2x_asn/*.c)
SRCS+= $(wildcard v2x_api/*.c)

OBJS:= $(SRCS:.c=.o)

INCS:= -I v2x_asn/ -I v2x_api/

#CFLAGS:= -Wall -O2
CFLAGS:= -Wall

LIBS:=


all:$(TARGET)
%.o:%.c
	$(CC) -fpic -c $< -o $@ $(INCS) $(CFLAGS)
	
$(TARGET):$(OBJS)
	$(CC) -shared -o $@ $^ $(LIBS)
	
clean:
	rm -f $(OBJS) $(TARGET)
	
test:
	rm -f test1
	$(CC) examples/test1.c -o test1 $(CFLAGS) $(INCS) -L ./ -lv2x_asn