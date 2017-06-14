# MB1502-raspberry-pi
Software for controlling MB1502 PLL synthesizer with raspberry pi board.

Build instructions
------------------
```bash
$ sudo apt-get install wiringpi
```
```bash
$ gcc main.c -o spi_synth -lwiringPi
```
Using
------------------
Gpio22 receiver PLL latch enable (LE) pin.
Gpio23 transmitter PLL latch enable (LE) pin.
```bash
# ./spi_synth -o 12800000 -r 431500000 -t 439100000 -i 71000000

-o Reference oscilator frequency.
-r Receiver frequency.
-t Transmitter freguency.
-i Intermediate receiver frequency.
```
