/*
 * SPI testing utility (using spidev driver)
 *
 * Copyright (c) 2007  MontaVista Software, Inc.
 * Copyright (c) 2007  Anton Vorontsov
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 * Cross-compile with cross-gcc -I/path/to/cross-kernel/include
 *
 */

/*
 * Software for controlling MB1502 PLL synthesizer with raspberry pi board. 
 *
 */

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <wiringPi.h>


static const uint32_t f_step_hz = 12500;
//static uint16_t delay;
static uint8_t tx_R[2];
static uint8_t tx_D[3];
static uint32_t Fosc_hz;
static uint8_t mode = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH;
static uint8_t bits = 8;
static uint32_t spi_speed = 100000;
static uint32_t intermediate_freq_hz;

struct sint_dev
{
	const char *dev_alias;
	uint32_t freq_hz;
	uint32_t Fvco_hz;
	uint32_t P;
	uint32_t N;
	uint32_t A;
	uint32_t R;
	uint32_t Ntot;
};

static void pabort(const char *s)
{
	perror(s);
	abort();
}

static struct sint_dev rx_sint_dev =
{
	.dev_alias = "/dev/spidev0.0",
};

static struct sint_dev tx_sint_dev =
{
	.dev_alias = "/dev/spidev0.1",
};

static void transfer(int fd, uint8_t *tx_p, uint32_t size)
{
	int ret;

	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx_p,
		//.rx_buf = (unsigned long)rx,
		.rx_buf = (unsigned long)NULL,
		.len = size,
		.delay_usecs = 10,
		.speed_hz = 0,
		.bits_per_word = 0,
	};

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret == 1)
		pabort("can't send spi message");
}

int main(int argc, char *argv[])
{
	int ret = 0;
	int fd_rx_synt, fd_tx_synt;
	uint16_t temp;
	uint8_t i;

	if(argc < 5)
	{
		printf("Too many arguments!\n");
		printf("-o fosc -r frx -t ftx -i fim\n");
		pabort("");
		return -1;
	}

	for(i = 1; i < argc; i++)
	{
		if(strcmp(&argv[i][0], "-o") == 0)
		{
			Fosc_hz = (uint32_t)atoi(&argv[i + 1][0]);
			printf("Fosc -> %d\n", Fosc_hz);
		}

		if(strcmp(&argv[i][0], "-r") == 0)
                {
                        rx_sint_dev.freq_hz = (uint32_t)atoi(&argv[i + 1][0]);
                        printf("RX freq. -> %d\n", rx_sint_dev.freq_hz);
                }


		if(strcmp(&argv[i][0], "-t") == 0)
                {
                        tx_sint_dev.freq_hz = (uint32_t)atoi(&argv[i + 1][0]);
                        printf("TX freq. -> %d\n", tx_sint_dev.freq_hz);
                }


		if(strcmp(&argv[i][0], "-i") == 0)
                {
                        intermediate_freq_hz = (uint32_t)atoi(&argv[i + 1][0]);
                        printf("IF -> %d\n", intermediate_freq_hz);
                }

//		printf("i %d -> %s\n", i, &argv[i][0]);
	}

	 if (wiringPiSetup () == -1)
    		return 1;

	pinMode (3, OUTPUT) ;         // aka BCM_GPIO pin 22 gpio3
	pinMode (4, OUTPUT) ;         // aka BCM_GPIO pin 23 gpio4

	digitalWrite (3, 0) ;       // Off
	digitalWrite (4, 0) ;       // Off

	//delay = 10; // 10us.

	fd_rx_synt = open(rx_sint_dev.dev_alias, O_RDWR);
	if (fd_rx_synt < 0)
		pabort("can't open device");

	fd_tx_synt = open(tx_sint_dev.dev_alias, O_RDWR);
        if (fd_tx_synt < 0)
                pabort("can't open device");

	/*
	 * spi mode
	 */
	ret = ioctl(fd_rx_synt, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
		pabort("can't set spi mode");

	ret = ioctl(fd_rx_synt, SPI_IOC_RD_MODE, &mode);
	if (ret == -1)
		pabort("can't get spi mode");

	ret = ioctl(fd_tx_synt, SPI_IOC_WR_MODE, &mode);
        if (ret == -1)
                pabort("can't set spi mode");

        ret = ioctl(fd_tx_synt, SPI_IOC_RD_MODE, &mode);
        if (ret == -1)
                pabort("can't get spi mode");

	/*
	 * bits per word
	 */
	ret = ioctl(fd_rx_synt, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't set bits per word");

	ret = ioctl(fd_rx_synt, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't get bits per word");

	ret = ioctl(fd_tx_synt, SPI_IOC_WR_BITS_PER_WORD, &bits);
        if (ret == -1)
                pabort("can't set bits per word");

        ret = ioctl(fd_tx_synt, SPI_IOC_RD_BITS_PER_WORD, &bits);
        if (ret == -1)
                pabort("can't get bits per word");

	/*
	 * max speed hz
	 */
	ret = ioctl(fd_rx_synt, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);
	if (ret == -1)
		pabort("can't set max speed hz");

	ret = ioctl(fd_rx_synt, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed);
	if (ret == -1)
		pabort("can't get max speed hz");

	ret = ioctl(fd_tx_synt, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);
        if (ret == -1)
                pabort("can't set max speed hz");

        ret = ioctl(fd_tx_synt, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed);
        if (ret == -1)
                pabort("can't get max speed hz");


	printf("RX Synt. spi mode: %d\n", mode);
	printf("RX Synt. bits per word: %d\n", bits);
	printf("RX Synt. max speed: %d Hz (%d KHz)\n", spi_speed, spi_speed / 1000);

        printf("TX Synt. spi mode: %d\n", mode);
        printf("TX Synt. bits per word: %d\n", bits);
        printf("TX Synt. max speed: %d Hz (%d KHz)\n", spi_speed, spi_speed / 1000);

        tx_R[0] = 0x00;
        tx_R[1] = 0x00;
        tx_D[0] = 0x00;
        tx_D[1] = 0x00;
        tx_D[2] = 0x00;

        rx_sint_dev.P = 128;
        rx_sint_dev.R = (uint32_t)(Fosc_hz / f_step_hz);
        if((rx_sint_dev.R < 8) || (rx_sint_dev.R > 16383))
                pabort("R register value is out of range!");

        temp = (rx_sint_dev.R << 1);
        tx_R[0] = (temp >> 8) & 0xFF;
        tx_R[1] = temp & 0xFF;
        if(rx_sint_dev.P == 64)
                tx_R[0] |= 0x80;

        tx_R[1] |= 0x01;

        rx_sint_dev.Fvco_hz = rx_sint_dev.freq_hz + intermediate_freq_hz;
        rx_sint_dev.Ntot = (uint32_t)(rx_sint_dev.Fvco_hz / f_step_hz);
        rx_sint_dev.N = (uint32_t)(rx_sint_dev.Ntot / rx_sint_dev.P);
        if((rx_sint_dev.N < 16) || (rx_sint_dev.N > 2047))
                pabort("N register value is out of range!");
        rx_sint_dev.A = (uint32_t)(rx_sint_dev.Ntot - (rx_sint_dev.N * rx_sint_dev.P));
        if((rx_sint_dev.A < 0) || (rx_sint_dev.A > 127) || (rx_sint_dev.A > rx_sint_dev.N))
                pabort("A register value is out of range!");

        tx_D[2] = (rx_sint_dev.A << 1) & 0xFE;
        tx_D[1] = rx_sint_dev.N & 0xFF;
        tx_D[0] = (rx_sint_dev.N >> 8) & 0xFF;

	printf("R -> %d\n", rx_sint_dev.R);
	printf("Fvco -> %d\n", rx_sint_dev.Fvco_hz);
	printf("Ntot -> %d\n", rx_sint_dev.Ntot);
	printf("N -> %d\n", rx_sint_dev.N);
	printf("A -> %d\n", rx_sint_dev.A);

	transfer(fd_rx_synt, tx_R, sizeof(tx_R));
	digitalWrite (3, 1) ;       // On
        usleep(10);
	digitalWrite (3, 0) ;       // Off
	transfer(fd_rx_synt, tx_D, sizeof(tx_D));
	digitalWrite (3, 1) ;       // On
	usleep(10);
	digitalWrite (3, 0) ;       // Off

	tx_R[0] = 0x00;
        tx_R[1] = 0x00;
        tx_D[0] = 0x00;
        tx_D[1] = 0x00;
        tx_D[2] = 0x00;

        tx_sint_dev.P = 128;
        tx_sint_dev.R = (uint32_t)(Fosc_hz / f_step_hz);
        if((tx_sint_dev.R < 8) || (tx_sint_dev.R > 16383))
                pabort("R register value is out of range!");

        temp = (tx_sint_dev.R << 1);
        tx_R[0] = (temp >> 8) & 0xFF;
        tx_R[1] = temp & 0xFF;
        if(tx_sint_dev.P == 64)
                tx_R[0] |= 0x80;

        tx_R[1] |= 0x01;

        tx_sint_dev.Fvco_hz = tx_sint_dev.freq_hz;
        tx_sint_dev.Ntot = (uint32_t)(tx_sint_dev.Fvco_hz / f_step_hz);
        tx_sint_dev.N = (uint32_t)(tx_sint_dev.Ntot / tx_sint_dev.P);
        if((tx_sint_dev.N < 16) || (tx_sint_dev.N > 2047))
                pabort("N register value is out of range!");
        tx_sint_dev.A = (uint32_t)(tx_sint_dev.Ntot - (tx_sint_dev.N * tx_sint_dev.P));
        if((tx_sint_dev.A < 0) || (tx_sint_dev.A > 127) || (tx_sint_dev.A > tx_sint_dev.N))
                pabort("A register value is out of range!");

        tx_D[2] = (tx_sint_dev.A << 1) & 0xFE;
        tx_D[1] = tx_sint_dev.N & 0xFF;
        tx_D[0] = (tx_sint_dev.N >> 8) & 0xFF;

	printf("R -> %d\n", tx_sint_dev.R);
        printf("Fvco -> %d\n", tx_sint_dev.Fvco_hz);
        printf("Ntot -> %d\n", tx_sint_dev.Ntot);
        printf("N -> %d\n", tx_sint_dev.N);
        printf("A -> %d\n", tx_sint_dev.A);

        transfer(fd_tx_synt, tx_R, sizeof(tx_R));
	digitalWrite (4, 1) ;       // On
        usleep(10);
	digitalWrite (4, 0) ;       // Off
        transfer(fd_tx_synt, tx_D, sizeof(tx_D));
	digitalWrite (4, 1) ;       // On
        usleep(10);
	digitalWrite (4, 0) ;       // Off

	close(fd_rx_synt);
	close(fd_tx_synt);

	return ret;
}
