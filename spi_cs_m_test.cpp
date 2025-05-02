#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <string.h>
#include "rp1-gpio.h"

#define SPI_DEV "/dev/spidev1.0"
#define CS_GPIO 18  // GPIO18 -> 物理ピン12

#define SPI_SPEED 20000000
#define WRITE 0x02
#define READ  0x03

int main() {
    RP1_GPIO gpio;
    gpio.begin();

    // --- CSピン設定 ---
    gpio.pinMode(CS_GPIO, gpio.OUTPUT);
    gpio.digitalWrite(CS_GPIO, gpio.HIGH);

    // --- SPI設定 ---
    int spi_fd = open(SPI_DEV, O_RDWR);
    if (spi_fd < 0) {
        perror("SPI open error");
        return 1;
    }

    uint8_t mode = SPI_MODE_0;
    uint8_t bits = 8;
    uint32_t speed = SPI_SPEED;
    ioctl(spi_fd, SPI_IOC_WR_MODE, &mode);
    ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

    // --- WRMR: シーケンシャルモード 0x40 ---
    uint8_t wrmr[2] = {0x01, 0x40};
    struct spi_ioc_transfer tr_wr = {0};
    tr_wr.tx_buf = (unsigned long)wrmr;
    tr_wr.len = 2;

    gpio.digitalWrite(CS_GPIO, gpio.LOW);
    ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr_wr);
    gpio.digitalWrite(CS_GPIO, gpio.HIGH);
    usleep(50);

    // --- モード確認 ---
    uint8_t rdmr_cmd = 0x05;
    uint8_t mode_val = 0xFF;
    struct spi_ioc_transfer tr_rd[2] = {0};
    tr_rd[0].tx_buf = (unsigned long)&rdmr_cmd;
    tr_rd[0].len = 1;
    tr_rd[1].rx_buf = (unsigned long)&mode_val;
    tr_rd[1].len = 1;

    gpio.digitalWrite(CS_GPIO, gpio.LOW);
    ioctl(spi_fd, SPI_IOC_MESSAGE(2), tr_rd);
    gpio.digitalWrite(CS_GPIO, gpio.HIGH);
    printf("SRAM Mode Register: 0x%02X\n", mode_val);

    // --- 書き込み（65535バイト） ---
    const int DATA_LEN = 0xffff;
    uint8_t write_data[4 + DATA_LEN];
    write_data[0] = WRITE;
    write_data[1] = 0x00;
    write_data[2] = 0x00;
    write_data[3] = 0x00;
    for (int i = 0; i < DATA_LEN; ++i) {
        write_data[4 + i] = i & 0xFF;
    }

    gpio.digitalWrite(CS_GPIO, gpio.LOW);
    write(spi_fd, write_data, sizeof(write_data));
    gpio.digitalWrite(CS_GPIO, gpio.HIGH);
    usleep(100);

    // --- 読み出し ---
    uint8_t tx_buf[4 + DATA_LEN] = { READ, 0x00, 0x00, 0x00 };
    uint8_t rx_buf[4 + DATA_LEN] = { 0 };
    struct spi_ioc_transfer tr = {0};
    tr.tx_buf = (unsigned long)tx_buf;
    tr.rx_buf = (unsigned long)rx_buf;
    tr.len = sizeof(tx_buf);
    tr.speed_hz = SPI_SPEED;
    tr.bits_per_word = 8;

    gpio.digitalWrite(CS_GPIO, gpio.LOW);
    ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    gpio.digitalWrite(CS_GPIO, gpio.HIGH);

    // --- 検証 ---
    int error_count = 0;
    for (int i = 0; i < DATA_LEN; ++i) {
        uint8_t expected = i & 0xFF;
        uint8_t actual = rx_buf[4 + i];
        if (expected != actual) {
            if (error_count < 10) {
                printf("Mismatch at %04X: wrote %02X, read %02X\n", i, expected, actual);
            }
            error_count++;
        }
    }

    if (error_count == 0) {
        printf("\n✅ 全 %d バイト一致（書き込み／読み出し OK）\n", DATA_LEN);
    } else {
        printf("\n❌ 不一致: %d 箇所のエラーが検出されました\n", error_count);
    }

    // 終了処理
    close(spi_fd);
    gpio.end();
    return 0;
}
