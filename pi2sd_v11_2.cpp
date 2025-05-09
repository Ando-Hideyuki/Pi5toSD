/*
20250307 Ando Hideyuki
Pi5　ー＞　サッカードディスプレイ　駆動のためのプログラム
まだ作りかけ、現状だとSRAMのテストができる
CEピンを汎用GPIOに設定
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

#define CS_R 18  // GPIO18 -> 物理ピン12
#define CS_G 17  // GPIO17 -> 物理ピン11
#define CS_B 16  // GPIO16 -> 物理ピン36
#define SPI_DEV "/dev/spidev1.0"
#define SPI_SPEED 30000000  // SPIクロック速度（30MHz）
#define WRITE 0x02  // 書き込みコマンド
#define READ  0x03  // 読み込みコマンド

int spi_fd; // グローバルで1つに統合
RP1_GPIO rp1;

void gpio_init(){
    //GPIOの設定
    rp1.begin();

    //おまじない gpio22 <- pic input
    rp1.pinMode(22, rp1.OUTPUT);
    rp1.digitalWrite(22, rp1.LOW);//LOW HIGH TOGGLE
    rp1.pinMode(22, rp1.INPUT);
  
    rp1.pinMode(26, rp1.OUTPUT); //HC157のS HのときSram0, LのときSram1に接続

    rp1.pinMode(4, rp1.OUTPUT);  //GPIO4 -> PIC
    rp1.pinMode(23, rp1.INPUT);  //PIC -> GPIO23   
  
    rp1.digitalWrite(26, rp1.LOW);//HC157のS LのときAつまりSram0に接続
    //rp1.digitalWrite(26, rp1.HIGH);//HC157のS HのときAつまりSram1に接続
    rp1.digitalWrite(4, rp1.HIGH);//PICを動作させる  
    //rp1.digitalWrite(4, rp1.LOW);//PICを停止させる  

    //CSピンの設定
    rp1.pinMode(CS_R, rp1.OUTPUT);
    rp1.pinMode(CS_G, rp1.OUTPUT);
    rp1.pinMode(CS_B, rp1.OUTPUT);
    rp1.digitalWrite(CS_R, rp1.HIGH); // 非選択状態にしておく
    rp1.digitalWrite(CS_G, rp1.HIGH);
    rp1.digitalWrite(CS_B, rp1.HIGH);

}

int spi_init() {
    spi_fd = open(SPI_DEV, O_RDWR);
    if (spi_fd < 0) {
        perror("SPIデバイスを開けませんでした");
        return -1;
    }

    // SPI設定
    uint8_t mode = SPI_MODE_0;
    uint8_t bits = 8;
    uint32_t speed = SPI_SPEED;
    ioctl(spi_fd, SPI_IOC_WR_MODE, &mode);
    ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

    return 0;
}
// **SPIの初期化**
#define SPI_SRAM_CMD_WRMR  0x01  // Write Mode Register
#define SPI_SRAM_SEQ_MODE 0x40  // シーケンシャルモード（デフォルト）
void reset_sram_mode(int CS_PIN) {
    uint8_t cmd[2] = {SPI_SRAM_CMD_WRMR, SPI_SRAM_SEQ_MODE};
    struct spi_ioc_transfer transfer;
    memset(&transfer, 0, sizeof(transfer)); // すべてのメンバをゼロ初期化
    transfer.tx_buf = (unsigned long)cmd;
    transfer.len = 2;
    rp1.digitalWrite(CS_PIN, rp1.LOW);
    ioctl(spi_fd, SPI_IOC_MESSAGE(1), &transfer);
    rp1.digitalWrite(CS_PIN, rp1.HIGH);
}

#define SPI_SRAM_CMD_RDMR  0x05  // Read Mode Register
uint8_t check_sram_mode(int CS_PIN) {
    uint8_t cmd = SPI_SRAM_CMD_RDMR;
    uint8_t mode = 0xFF;
    struct spi_ioc_transfer transfer[2];
    memset(&transfer, 0, sizeof(transfer)); // 配列全体をゼロ初期化
    transfer[0].tx_buf = (unsigned long)&cmd;
    transfer[0].len = 1;
    transfer[1].rx_buf = (unsigned long)&mode;
    transfer[1].len = 1;
    rp1.digitalWrite(CS_PIN, rp1.LOW);
    ioctl(spi_fd, SPI_IOC_MESSAGE(2), transfer);
    rp1.digitalWrite(CS_PIN, rp1.HIGH);
    return mode;
}

void sram_init(){
    uint8_t mode;

    // SRAM R
    reset_sram_mode(CS_R);
    mode = check_sram_mode(CS_R);
    printf("Sram_R Mode Register: 0x%02X\n", mode);
   
    // SRAM G
    reset_sram_mode(CS_G);

    // SRAM B
    reset_sram_mode(CS_B);
}

/*
void reset_sram_mode(int spi_fd) {
    uint8_t cmd[2] = {SPI_SRAM_CMD_WRMR, SPI_SRAM_SEQ_MODE};
    struct spi_ioc_transfer transfer;
    memset(&transfer, 0, sizeof(transfer)); // すべてのメンバをゼロ初期化
    transfer.tx_buf = (unsigned long)cmd;
    transfer.len = 2;
    
    ioctl(spi_fd, SPI_IOC_MESSAGE(1), &transfer);
}
*/

// SRAMにデータを書き込む
uint8_t buff[0xFFFF+4 +1];
void sram_write() {
    int result;
    buff[0]=WRITE;
    buff[1]=0x00;
    buff[2]=0x00;
    buff[3]=0x00;

    // --- R用SRAM書き込み ---
    memcpy(&buff[4], PicDat_r, 0xFFFF);
    rp1.digitalWrite(CS_R, rp1.LOW);
    result = write(spi_fd, buff, 4 + 0xFFFF + 1);
    rp1.digitalWrite(CS_R, rp1.HIGH);
    //usleep(100);
    if (result < 0) {
        perror("SRAM_R 書き込みエラー");
    }

    // --- G用SRAM書き込み ---
    memcpy(&buff[4], PicDat_g, 0xFFFF);
    rp1.digitalWrite(CS_G, rp1.LOW);
    write(spi_fd, buff, 4 + 0xFFFF + 1);
    rp1.digitalWrite(CS_G, rp1.HIGH);
    //usleep(100);

    // --- B用SRAM書き込み ---
    memcpy(&buff[4], PicDat_b, 0xFFFF);
    rp1.digitalWrite(CS_B, rp1.LOW);
    write(spi_fd, buff, 4 + 0xFFFF + 1);
    rp1.digitalWrite(CS_B, rp1.HIGH);
    //usleep(100);
}

void sram_write_single(const uint8_t *data, int cs_pin) {
    uint8_t cmd[4] = { WRITE, 0x00, 0x00, 0x00 };
    struct spi_ioc_transfer xfer[2] = {0};

    xfer[0].tx_buf = (unsigned long)cmd;
    xfer[0].len = 4;

    xfer[1].tx_buf = (unsigned long)data;
    xfer[1].len = 0xFFFF;

    rp1.digitalWrite(cs_pin, rp1.LOW);
    ioctl(spi_fd, SPI_IOC_MESSAGE(2), xfer);
    rp1.digitalWrite(cs_pin, rp1.HIGH);
}

#define BUFFER_SIZE 0xffff  
void sram_read() {
    const int DATA_LEN = 0xffff;
    // --- 読み出し ---
    uint8_t tx_buf[4 + DATA_LEN] = { READ, 0x00, 0x00, 0x00 };
    uint8_t rx_buf[4 + DATA_LEN] = { 0 };
    struct spi_ioc_transfer tr = {0};
    tr.tx_buf = (unsigned long)tx_buf;
    tr.rx_buf = (unsigned long)rx_buf;
    tr.len = sizeof(tx_buf);
    tr.speed_hz = SPI_SPEED;
    tr.bits_per_word = 8;
    
    rp1.digitalWrite(CS_R, rp1.LOW);
    ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    rp1.digitalWrite(CS_R, rp1.HIGH);
    
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
void bitmap_process(const char *filename) {
    bmp_img img;
    //bmp_img_read(&img, "input.bmp");
    bmp_img_read(&img, filename);

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

void PicRes0to1(){//0->1
    rp1.digitalWrite(4, rp1.LOW);//PICを停止させる
    while((rp1.digitalRead(23))==1){}
    rp1.digitalWrite(26, rp1.HIGH);//HC157のS HのときAつまりSram1に接続
    rp1.digitalWrite(4, rp1.HIGH);//PICを動作させる
}
void PicRes1to0(){//1->0
    rp1.digitalWrite(4, rp1.LOW);//PICを停止させる
    while((rp1.digitalRead(23))==1){}
    rp1.digitalWrite(26, rp1.LOW);//HC157のS LのときAつまりSram0に接続
    rp1.digitalWrite(4, rp1.HIGH);//PICを動作させる
}


void Datachk(){
    // **データチェック**
    int mismatch = 0;
    for (int i = 0; i < BUFFER_SIZE; i++) {
        if (PicDat_r[i] != read_data[i]) {
            //printf("エラー: アドレス %d で 0x%02X を書いたが 0x%02X を読んだ\n", i, PicDat_r[i], read_data[i]);
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
    bitmap_process("input.bmp");
    gpio_init();
    spi_init();

    struct timespec start, end;
    clock_gettime(CLOCK_MONOTONIC, &start);

    sram_init();
    //sram_write(); //20MHzで79ms, 30MHzで63ms

    sram_write_single(PicDat_r, CS_R);
    sram_write_single(PicDat_g, CS_G);
    sram_write_single(PicDat_b, CS_B);

    clock_gettime(CLOCK_MONOTONIC, &end);

    // 経過時間を計算（ナノ秒単位）
    double elapsed_time = (end.tv_sec - start.tv_sec) +
                          (end.tv_nsec - start.tv_nsec) / 1e9;

    printf("処理時間: %.9f マイクロ秒\n", elapsed_time*1000*1000);

    //sram_read();
    PicRes0to1();//PICの処理を待ってSRAMバンクを0から1に切り替える

    /*
    //BMPを開いてPicDat_r[128][256/8*16],PicDat_g[128][256/8*16],PicDat_b[128][256/8*16]にセットする
    //bitmap_process("input.bmp");
    //bitmap_process("rainbow.bmp");
     sram_init();//171us
    sram_write(); //20MHzで79ms, 30MHzで63ms
    sram_read(0);
    Datachk();

    PicRes0to1();//PICの処理を待ってSRAMバンクを0から1に切り替える
    */

    close(spi_fd);
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