#include <stdio.h>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <unistd.h>

int main() {
    int spi_fd = open("/dev/spidev1.0", O_RDWR);
    if (spi_fd < 0) {
        perror("SPIデバイスを開けませんでした");
        return -1;
    }

    uint32_t max_speed;
    if (ioctl(spi_fd, SPI_IOC_RD_MAX_SPEED_HZ, &max_speed) == -1) {
        perror("SPI_IOC_RD_MAX_SPEED_HZ 取得エラー");
    } else {
        printf("SPI 最大クロック速度: %u Hz\n", max_speed);
    }

    close(spi_fd);
    return 0;
}
