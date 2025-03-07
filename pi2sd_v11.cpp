/*
20250307 Ando Hideyuki
Pi5　ー＞　サッカードディスプレイ　駆動のためのプログラム
まだ作りかけ、現状だとSRAMのテストができる
*/
#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <string.h>
#include <errno.h>
#include "rp1-gpio.h"
#include <stdlib.h>
#include "libbmp.h"
#include <time.h>

unsigned char R[256],G[256],B[256];
unsigned char PicDat_r[128*256/8*16],PicDat_g[128*256/8*16],PicDat_b[128*256/8*16];

//debug用
unsigned char read_data[0x1ffff];

#define SPI_DEVICE0 "/dev/spidev1.0"  // SPIデバイス（CE0を使用）
#define SPI_DEVICE1 "/dev/spidev1.1"  // SPIデバイス（CE1を使用）
#define SPI_DEVICE2 "/dev/spidev1.2"  // SPIデバイス（CE2を使用）
#define SPI_SPEED 30000000  // SPIクロック速度（30MHz）

#define WRITE 0x02  // 書き込みコマンド
#define READ  0x03  // 読み込みコマンド

int spi_fd0,spi_fd1,spi_fd2;  // SPIデバイスのファイルディスクリプタ

RP1_GPIO rp1;


// **SPIの初期化**
#define SPI_SRAM_CMD_WRMR  0x01  // Write Mode Register
#define SPI_SRAM_SEQ_MODE 0x40  // シーケンシャルモード（デフォルト）
void reset_sram_mode(int spi_fd) {
    uint8_t cmd[2] = {SPI_SRAM_CMD_WRMR, SPI_SRAM_SEQ_MODE};
    struct spi_ioc_transfer transfer;
    memset(&transfer, 0, sizeof(transfer)); // すべてのメンバをゼロ初期化
    transfer.tx_buf = (unsigned long)cmd;
    transfer.len = 2;
    
    ioctl(spi_fd, SPI_IOC_MESSAGE(1), &transfer);
}


#define SPI_SRAM_CMD_RDMR  0x05  // Read Mode Register
uint8_t check_sram_mode(int spi_fd) {
    uint8_t cmd = SPI_SRAM_CMD_RDMR;
    uint8_t mode = 0xFF;
    struct spi_ioc_transfer transfer[2];
    memset(&transfer, 0, sizeof(transfer)); // 配列全体をゼロ初期化
    transfer[0].tx_buf = (unsigned long)&cmd;
    transfer[0].len = 1;
    
    transfer[1].rx_buf = (unsigned long)&mode;
    transfer[1].len = 1;
    ioctl(spi_fd, SPI_IOC_MESSAGE(2), transfer);
    return mode;
}



int spi_init() {
    spi_fd0 = open(SPI_DEVICE0, O_RDWR);
    if (spi_fd0 < 0) {
        perror("SPIデバイスを開けませんでした");
        return -1;
    }

    uint8_t mode = 0;
    uint8_t bits = 8;
    uint32_t speed = SPI_SPEED;

    ioctl(spi_fd0, SPI_IOC_WR_MODE, &mode);
    ioctl(spi_fd0, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ioctl(spi_fd0, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

    spi_fd1 = open(SPI_DEVICE1, O_RDWR);
    if (spi_fd1 < 0) {
        perror("SPIデバイスを開けませんでした");
        return -1;
    }

    ioctl(spi_fd1, SPI_IOC_WR_MODE, &mode);
    ioctl(spi_fd1, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ioctl(spi_fd1, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

    spi_fd2 = open(SPI_DEVICE2, O_RDWR);
    if (spi_fd2 < 0) {
        perror("SPIデバイスを開けませんでした");
        return -1;
    }

    ioctl(spi_fd2, SPI_IOC_WR_MODE, &mode);
    ioctl(spi_fd2, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ioctl(spi_fd2, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

    return 0;
}
void sram_init(){
    uint8_t mode;
    reset_sram_mode(spi_fd0);
    mode = check_sram_mode(spi_fd0);
    printf("Sram1 Mode Register: 0x%02X\n", mode);
    reset_sram_mode(spi_fd1);
    mode = check_sram_mode(spi_fd1);
    printf("Sram2 Mode Register: 0x%02X\n", mode);
    reset_sram_mode(spi_fd2);
    mode = check_sram_mode(spi_fd2);
    printf("Sram3 Mode Register: 0x%02X\n", mode);
}

// SRAMにデータを書き込む
uint8_t buff[0xFFFF+4 +1];
void sram_write() {
    int i;
    int result;
    buff[0]=0x02;
    buff[1]=0x00;
    buff[2]=0x00;
    buff[3]=0x00;

    /*
    for(i=0;i<0xFFFF;i++){
            buff[i+4]=PicDat_r[i];
   
    }
    */
    memcpy(&buff[4], PicDat_r, 0xFFFF);
    result = write(spi_fd0, buff, 4 + 0xffff + 1);
    if (result < 0) {
        perror("SRAM 書き込みエラー");
    }


/*
    //------------
    // 計測開始
    struct timespec start, end;
    clock_gettime(CLOCK_MONOTONIC, &start);



    clock_gettime(CLOCK_MONOTONIC, &end);

    // 経過時間を計算（ナノ秒単位）
    double elapsed_time = (end.tv_sec - start.tv_sec) +
                          (end.tv_nsec - start.tv_nsec) / 1e9;

    printf("処理時間: %.9f マイクロ秒\n", elapsed_time*1000*1000);
//-------------
*/


    for(i=0;i<0xFFFF;i++){
            buff[i+4]=PicDat_g[i];
   
    }
    result = write(spi_fd1, buff, 4 + 0xffff + 1);
    if (result < 0) {
        perror("SRAM 書き込みエラー");
    }

    for(i=0;i<0xFFFF;i++){
        buff[i+4]=PicDat_b[i];

    }
    result = write(spi_fd2, buff, 4 + 0xffff + 1);
    if (result < 0) {
        perror("SRAM 書き込みエラー");
    }

}

#define BUFFER_SIZE 0xffff  
void sram_read(uint32_t addr) {
    uint8_t tx_buf[4 + BUFFER_SIZE + 1] = {READ, 
                                       (uint8_t)((addr >> 16) & 0xFF),  // 上位バイト
                                       (uint8_t)((addr >> 8)  & 0xFF),  // 中位バイト
                                       (uint8_t)(addr & 0xFF)};         // 下位バイト
    uint8_t rx_buf[4 + BUFFER_SIZE + 1] = {0};  // 受信用バッファ（ヘッダ＋データ）

    struct spi_ioc_transfer tr;
        memset(&tr, 0, sizeof(tr)); // すべてのメンバをゼロ初期化
        tr.tx_buf = (unsigned long)tx_buf;
        tr.rx_buf = (unsigned long)rx_buf;
        tr.len = sizeof(tx_buf);
        tr.speed_hz = SPI_SPEED;
        tr.bits_per_word = 8;

    if (ioctl(spi_fd0, SPI_IOC_MESSAGE(1), &tr) < 0) {
        perror("SRAM 読み込みエラー");
    }
    memcpy(read_data, &rx_buf[4], BUFFER_SIZE);  // 受信データをコピー


}



//--------PWM用に分解
void SetDat(int line){
    int c1,c2,j,k;
    unsigned char Pdat_r,Pdat_g,Pdat_b;   
    //変換
    c1=c2=Pdat_r=Pdat_g=Pdat_b=0;
    for(k=0;k<16;k++){  
        for(j=0;j<256;j++) {
          
          if( 0 < R[j]){
            R[j]--;
            Pdat_r++;
          };
  
          if( 0 < G[j]){
            G[j]--;
            Pdat_g++;
          };
  
          if( 0 < B[j]){
            B[j]--;
            Pdat_b++;
          };
  
          if(c2 == 7){
            c2=0;
            PicDat_r[line*512+c1]=Pdat_r;
            PicDat_g[line*512+c1]=Pdat_g;
            PicDat_b[line*512+c1]=Pdat_b;
            Pdat_r=Pdat_g=Pdat_b=0;
            c1++;
          }else{
            c2++;
          }
  
          Pdat_r = Pdat_r<<1;
          Pdat_g = Pdat_g<<1;
          Pdat_b = Pdat_b<<1;
        }
       
    }
  }
void bitmap_process() {
    bmp_img img;
    bmp_img_read(&img, "input.bmp");

    int width = img.img_header.biWidth;
    int height = abs(img.img_header.biHeight); // 高さは絶対値で扱う

    printf("画像サイズ: 幅=%d, 高さ=%d\n", width, height);
	int x,line=0;

	for(x = 0; x < width; x=x+2){
		//printf("左上から下方向にRGBを取得 (X=%d の列)\n", x);
		//データをR［］，G［］，B［］に変換	
		for (int y = 0; y < height; y++) {
				bmp_pixel p = img.img_pixels[y][x];
				//printf("TD_Y=%d → R:%3d G:%3d B:%3d\n", y, p.red, p.green, p.blue);
				R[y] = p.red >> 4;
				G[y] = p.green >> 4;
				B[y] = p.blue >> 4 ;
		}
		//R［］，G［］，B［］-> PicDat_r,g,b （PWM）に変換	
		SetDat(line);
		line++;
	}
    bmp_img_free(&img);
}
void gpio_init(){
    //GPIOの設定
    rp1.begin();

    //おまじない gpio22 <- pic input
    rp1.pinMode(22, rp1.OUTPUT);
    rp1.digitalWrite(22, rp1.LOW);//LOW HIGH TOGGLE
    rp1.pinMode(22, rp1.INPUT);
  
    rp1.pinMode(26, rp1.OUTPUT); //HC157のS HのときSram0, LのときSram1に接続

    rp1.pinMode(4, rp1.OUTPUT);  //GPIO4 -> PIC
    rp1.pinMode(23, rp1.INPUT);
  

    rp1.digitalWrite(26, rp1.LOW);//HC157のS LのときAつまりSram0に接続
    //rp1.digitalWrite(26, rp1.HIGH);//HC157のS HのときAつまりSram1に接続
    rp1.digitalWrite(4, rp1.HIGH);//PICを動作させる  
    //rp1.digitalWrite(4, rp1.LOW);//PICを停止させる  
}

void Datachk(){
    // **データチェック**
    int mismatch = 0;
    for (int i = 0; i < BUFFER_SIZE; i++) {
        if (PicDat_r[i] != read_data[i]) {
            printf("エラー: アドレス %d で 0x%02X を書いたが 0x%02X を読んだ\n", i, PicDat_r[i], read_data[i]);
            mismatch = 1;
        }
    }

    if (!mismatch) {
        printf("\n✅ SRAM 書き込み & 読み込み成功！データが一致しました！\n");
    } else {
        printf("\n❌ SRAM 読み書きに失敗しました...\n");
    }
}
int main() {
    //BMPを開いてPicDat_r[128][256/8*16],PicDat_g[128][256/8*16],PicDat_b[128][256/8*16]にセットする
    bitmap_process();
    gpio_init();
    spi_init();
    sram_init();//171us
    sram_write(); //20MHzで79ms, 30MHzで63ms
    sram_read(0);
    Datachk();

    rp1.digitalWrite(26, rp1.HIGH);//HC157のS HのときAつまりSram1に接続

    close(spi_fd0); close(spi_fd1); close(spi_fd2);
    rp1.end();
    return 0;
}

/*
    //------------
    // 計測開始
    struct timespec start, end;
    clock_gettime(CLOCK_MONOTONIC, &start);



    clock_gettime(CLOCK_MONOTONIC, &end);

    // 経過時間を計算（ナノ秒単位）
    double elapsed_time = (end.tv_sec - start.tv_sec) +
                          (end.tv_nsec - start.tv_nsec) / 1e9;

    printf("処理時間: %.9f マイクロ秒\n", elapsed_time*1000*1000);
//-------------
*/