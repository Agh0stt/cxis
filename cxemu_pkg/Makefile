CC     = gcc
CFLAGS = -O2 -std=c11 -Wall -Wextra -Iinclude
TARGET = cxemu

all: $(TARGET)

$(TARGET): cxemu.c include/abios.h include/cxis.h include/cxe.h
	$(CC) $(CFLAGS) -o $@ cxemu.c

# create a blank 1MB disk image
disk.img:
	dd if=/dev/zero of=disk.img bs=512 count=2048

clean:
	rm -f $(TARGET) *.o

.PHONY: all clean
