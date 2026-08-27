

#CROSS_COMPILE=


all: uart_tof uart

clean:
	rm uart_tof uart *.o


uart_tof: uart_tof.c
	${CROSS_COMPILE}gcc -O2 -Wall -o $@ $<

uart: uart.cpp
	${CROSS_COMPILE}gcc -O2 -Wall -D__MAIN__ -o $@ $< -lm
