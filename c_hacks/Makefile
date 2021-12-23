CC = arm-linux-gnueabihf-gcc
CFLAGS = -O0 -g3
LDFLAGS = -lmmal_core -lmmal_util -lmmal_vc_client -lvcos -lbcm_host -lpigpio

all: lt6911uxc tc358743

%.o: %.c
	$(CC) $< $(CFLAGS) -c -o $@

lt6911uxc: lt6911uxc.o
	$(CC) $< $(CFLAGS) $(LDFLAGS) -o $@

tc358743: tc358743.o
	$(CC) $< $(CFLAGS) $(LDFLAGS) -o $@

clean:
	rm -f *.o lt6911uxc tc358743

.PHONY: all clean
