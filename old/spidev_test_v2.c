#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <string.h>
#include <errno.h>

#define SPI_DEVICE0 "/dev/spidev1.0"  // SPIデバイス（CE0を使用）
#define SPI_DEVICE1 "/dev/spidev1.1"  // SPIデバイス（CE1を使用）
#define SPI_DEVICE2 "/dev/spidev1.2"  // SPIデバイス（CE2を使用）
#define SPI_SPEED 30000000  // SPIクロック速度（30MHz）

#define WRITE 0x02  // 書き込みコマンド
#define READ  0x03  // 読み込みコマンド
#define BUFFER_SIZE 256  // 256バイトのデータ

int spi_fd;  // SPIデバイスのファイルディスクリプタ

uint8_t write_data[BUFFER_SIZE];  // 書き込み用データ

// **SPIの初期化**
int spi_init() {
    spi_fd = open(SPI_DEVICE2, O_RDWR);
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


// **SRAMに 256バイトのデータを書き込む**
void sram_write_256(uint32_t addr, uint8_t *data_buffer) {
    uint8_t tx_buf[3 + BUFFER_SIZE] = {WRITE, 
                                       (addr >> 16) & 0xFF,  // 上位バイト
                                       (addr >> 8)  & 0xFF,  // 中位バイト
                                       addr & 0xFF};         // 下位バイト
    memcpy(&tx_buf[3], data_buffer, BUFFER_SIZE);  // データを追加

    int result = write(spi_fd, tx_buf, sizeof(tx_buf));
    if (result < 0) {
        perror("SRAM 256バイト書き込みエラー");
    }
}

// SRAMにデータを書き込む
uint8_t buff[256];
void sram_write() {
    uint8_t buff[256+4];
    int i;
    buff[0]=0x02;
    buff[1]=0x00;
    buff[2]=0x00;
    buff[3]=0x00;
    for(i=0;i<256;i++){
        buff[i+4]=255-i;
        write_data[i]=buff[i+4];
    }
    int result = write(spi_fd, buff, 4+256);
    if (result < 0) {
        perror("SRAM 書き込みエラー");
    }

    printf("送信データ: ");
    for (i = 0; i < (256+4); i++) {
        printf("%02X ", buff[i]);
    }
}

// **SRAMから 256バイト を一気に読み込む**
void sram_read_256(uint32_t addr, uint8_t *data_buffer) {
    uint8_t tx_buf[4 + BUFFER_SIZE] = {READ, 
                                       (addr >> 16) & 0xFF,  // 上位バイト
                                       (addr >> 8)  & 0xFF,  // 中位バイト
                                       addr & 0xFF};         // 下位バイト
    uint8_t rx_buf[4 + BUFFER_SIZE] = {0};  // 受信用バッファ（ヘッダ＋データ）

    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx_buf,
        .rx_buf = (unsigned long)rx_buf,
        .len = sizeof(tx_buf),
        .speed_hz = SPI_SPEED,
        .bits_per_word = 8,
    };

    if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
        perror("SRAM 256バイト読み込みエラー");
    }

    memcpy(data_buffer, &rx_buf[4], BUFFER_SIZE);  // 受信データをコピー
}

int main() {
    if (spi_init() < 0) {
        return -1;
    }

    uint32_t test_addr = 0x000000;  // 読み書きするアドレス
    uint8_t read_data[BUFFER_SIZE] = {0};  // 読み取り用バッファ

    printf("SRAMに 0x00 ～ 0xFF のデータを書き込み...\n");
    //sram_write_256(test_addr, write_data);
    sram_write(); 

    printf("SRAMから 256バイト を読み込み...\n");
    sram_read_256(test_addr, read_data);

    printf("\n読み込んだデータ:\n");
    for (int i = 0; i < BUFFER_SIZE; i++) {
        printf("%02X ", read_data[i]);
        if ((i + 1) % 16 == 0) printf("\n");
    }

    // **データチェック**
    int mismatch = 0;
    for (int i = 0; i < BUFFER_SIZE; i++) {
        if (write_data[i] != read_data[i]) {
            printf("エラー: アドレス %d で 0x%02X を書いたが 0x%02X を読んだ\n", i, write_data[i], read_data[i]);
            mismatch = 1;
        }
    }

    if (!mismatch) {
        printf("\n✅ SRAM 書き込み & 読み込み成功！データが一致しました！\n");
    } else {
        printf("\n❌ SRAM 読み書きに失敗しました...\n");
    }

    close(spi_fd);
    return 0;
}