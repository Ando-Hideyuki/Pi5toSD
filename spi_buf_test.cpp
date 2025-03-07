#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <string.h>
#include <errno.h>

#define SPI_DEVICE "/dev/spidev1.0"  // SPIデバイス（CE0を使用）
#define SPI_SPEED 20000000  // SPIクロック速度（20MHz）
#define WRITE 0x02  // 書き込みコマンド
#define READ  0x03  // 読み込みコマンド
#define BUFFER_SIZE 65535  // 65535 バイトのデータ

int spi_fd;  // SPIデバイスのファイルディスクリプタ
uint8_t write_data[BUFFER_SIZE];  // 書き込み用データ
uint8_t read_data[BUFFER_SIZE];   // 読み込み用データ

// **SPIの初期化**
int spi_init() {
    spi_fd = open(SPI_DEVICE, O_RDWR);
    if (spi_fd < 0) {
        perror("SPIデバイスを開けませんでした");
        return -1;
    }

    uint8_t mode = 0;
    uint8_t bits = 8;
    uint32_t speed = SPI_SPEED;

    ioctl(spi_fd, SPI_IOC_WR_MODE, &mode);
    ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

    return 0;
}

// **SRAMに65535バイトを書き込み、正しく書き込まれているか検証**
void sram_write_verify() {
    int i, result;
    uint8_t cmd[4];  // コマンドバイト（WRITE/READ + アドレス3バイト）
    
    // **書き込むデータを生成**
    for (i = 0; i < BUFFER_SIZE; i++) {
        write_data[i] = i & 0xFF;  // 簡単なパターン (0x00, 0x01, ..., 0xFF)
    }

    // **SRAMに書き込み**
    cmd[0] = WRITE;  // 書き込みコマンド
    cmd[1] = 0x00;   // アドレス上位バイト
    cmd[2] = 0x00;   // アドレス中位バイト
    cmd[3] = 0x00;   // アドレス下位バイト

    struct iovec tx[] = {
        { cmd, 4 },  // コマンド（4バイト）
        { write_data, BUFFER_SIZE }  // データ
    };
    result = writev(spi_fd, tx, 2);
    if (result < 0) {
        perror("SRAM 書き込みエラー");
        return;
    }

    printf("SRAM に 65535 バイトを書き込みました\n");

    // **SRAMからデータを読み込み**
    cmd[0] = READ;  // 読み込みコマンド
    struct spi_ioc_transfer tr[2] = {
        {
            .tx_buf = (unsigned long)cmd,
            .rx_buf = 0,
            .len = 4,
            .speed_hz = SPI_SPEED,
            .bits_per_word = 8,
        },
        {
            .tx_buf = 0,
            .rx_buf = (unsigned long)read_data,
            .len = BUFFER_SIZE,
            .speed_hz = SPI_SPEED,
            .bits_per_word = 8,
        }
    };

    if (ioctl(spi_fd, SPI_IOC_MESSAGE(2), &tr) < 0) {
        perror("SRAM 読み込みエラー");
        return;
    }

    printf("SRAM から 65535 バイトを読み込みました\n");

    // **書き込んだデータと読み込んだデータを比較**
    for (i = 0; i < BUFFER_SIZE; i++) {
        if (write_data[i] != read_data[i]) {
            printf("エラー: アドレス 0x%X でデータ不一致 (書き込んだ値: 0x%X, 読み込んだ値: 0x%X)\n",
                   i, write_data[i], read_data[i]);
            return;
        }
    }

    printf("SRAM のデータ検証成功: 全データ一致！\n");
}

int main() {
    if (spi_init() < 0) {
        return -1;
    }

    sram_write_verify();

    close(spi_fd);
    return 0;
}