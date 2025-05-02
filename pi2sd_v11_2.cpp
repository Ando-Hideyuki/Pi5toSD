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

#define CS_R 18  // GPIO18 -> 物理ピン12
#define CS_G 17  // GPIO17 -> 物理ピン11
#define CS_B 16  // GPIO16 -> 物理ピン36
#define SPI_DEV "/dev/spidev1.0"

#define SPI_SPEED 20000000  // SPIクロック速度（30MHz）

#define WRITE 0x02  // 書き込みコマンド
#define READ  0x03  // 読み込みコマンド

int spi_fd; // グローバルで1つに統合

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

void select_sram(int gpio_pin) {
    rp1.digitalWrite(CS_R, rp1.HIGH);
    rp1.digitalWrite(CS_G, rp1.HIGH);
    rp1.digitalWrite(CS_B, rp1.HIGH);
    rp1.digitalWrite(gpio_pin, rp1.LOW); // 指定したSRAMを選択
}
void deselect_all_sram() {
    rp1.digitalWrite(CS_R, rp1.HIGH);
    rp1.digitalWrite(CS_G, rp1.HIGH);
    rp1.digitalWrite(CS_B, rp1.HIGH);
}

int spi_init() {
    spi_fd = open(SPI_DEV, O_RDWR);
    if (spi_fd < 0) {
        perror("SPIデバイスを開けませんでした");
        return -1;
    }

    // GPIO CSピン初期化
    rp1.pinMode(CS_R, rp1.OUTPUT);
    rp1.pinMode(CS_G, rp1.OUTPUT);
    rp1.pinMode(CS_B, rp1.OUTPUT);
    rp1.digitalWrite(CS_R, rp1.HIGH); // 非選択状態にしておく
    rp1.digitalWrite(CS_G, rp1.HIGH);
    rp1.digitalWrite(CS_B, rp1.HIGH);

    // SPI設定
    uint8_t mode = 0;
    uint8_t bits = 8;
    uint32_t speed = SPI_SPEED;

    ioctl(spi_fd, SPI_IOC_WR_MODE, &mode);
    ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

    return 0;
}

void sram_init(){
    uint8_t mode;

    // SRAM R
    rp1.digitalWrite(CS_R, rp1.LOW);
    reset_sram_mode(spi_fd);
    mode = check_sram_mode(spi_fd);
    rp1.digitalWrite(CS_R, rp1.HIGH);
    printf("Sram_R Mode Register: 0x%02X\n", mode);

    // SRAM G
    rp1.digitalWrite(CS_G, rp1.LOW);
    reset_sram_mode(spi_fd);
    mode = check_sram_mode(spi_fd);
    rp1.digitalWrite(CS_G, rp1.HIGH);
    printf("Sram_G Mode Register: 0x%02X\n", mode);

    // SRAM B
    rp1.digitalWrite(CS_B, rp1.LOW);
    reset_sram_mode(spi_fd);
    mode = check_sram_mode(spi_fd);
    rp1.digitalWrite(CS_B, rp1.HIGH);
    printf("Sram_B Mode Register: 0x%02X\n", mode);
}

// SRAMにデータを書き込む
uint8_t buff[0xFFFF+4 +1];
void sram_write() {
    int result;
    buff[0]=0x02;
    buff[1]=0x00;
    buff[2]=0x00;
    buff[3]=0x00;

    // --- R用SRAM書き込み ---
    memcpy(&buff[4], PicDat_r, 0xFFFF);
    rp1.digitalWrite(CS_R, rp1.LOW);
    result = write(spi_fd, buff, 4 + 0xFFFF + 1);
    rp1.digitalWrite(CS_R, rp1.HIGH);
    if (result < 0) {
        perror("SRAM_R 書き込みエラー");
    }

    // --- G用SRAM書き込み ---
    memcpy(&buff[4], PicDat_g, 0xFFFF);
    rp1.digitalWrite(CS_G, rp1.LOW);
    result = write(spi_fd, buff, 4 + 0xFFFF + 1);
    rp1.digitalWrite(CS_G, rp1.HIGH);
    if (result < 0) {
        perror("SRAM_G 書き込みエラー");
    }

    // --- B用SRAM書き込み ---
    memcpy(&buff[4], PicDat_b, 0xFFFF);
    rp1.digitalWrite(CS_B, rp1.LOW);
    result = write(spi_fd, buff, 4 + 0xFFFF + 1);
    rp1.digitalWrite(CS_B, rp1.HIGH);
    if (result < 0) {
        perror("SRAM_B 書き込みエラー");
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


}

#define BUFFER_SIZE 0xffff  
void sram_read(uint32_t addr) {
    uint8_t tx_buf[4 + BUFFER_SIZE + 1] = {
        READ,
        (uint8_t)((addr >> 16) & 0xFF),
        (uint8_t)((addr >> 8)  & 0xFF),
        (uint8_t)(addr & 0xFF)
    };
    uint8_t rx_buf[4 + BUFFER_SIZE + 1] = {0};

    struct spi_ioc_transfer tr;
    memset(&tr, 0, sizeof(tr));
    tr.tx_buf = (unsigned long)tx_buf;
    tr.rx_buf = (unsigned long)rx_buf;
    tr.len = sizeof(tx_buf);
    tr.speed_hz = SPI_SPEED;
    tr.bits_per_word = 8;

    // 読み出しは R のみ（テスト用）
    rp1.digitalWrite(CS_R, rp1.LOW);
    if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
        perror("SRAM_R 読み込みエラー");
    }
    rp1.digitalWrite(CS_R, rp1.HIGH);

    memcpy(read_data, &rx_buf[4], BUFFER_SIZE);

    // デバッグ: 最初の16バイトだけ表示
    printf("READ DUMP:\n");
    for (int i = 0; i < 16; i++) {
        printf("%02X ", read_data[i]);
    }
    printf("\n");

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
    //BMPを開いてPicDat_r[128][256/8*16],PicDat_g[128][256/8*16],PicDat_b[128][256/8*16]にセットする
    //bitmap_process("input.bmp");
    //bitmap_process("rainbow.bmp");

    gpio_init();
    spi_init();
    sram_init();

 

    sram_init();//171us
    sram_write(); //20MHzで79ms, 30MHzで63ms
    sram_read(0);
    Datachk();

    PicRes0to1();//PICの処理を待ってSRAMバンクを0から1に切り替える

 /*
    bitmap_process("star1.bmp");
    sram_init();//171us
    sram_write(); //20MHzで79ms, 30MHzで63ms

    PicRes1to0();//PICの処理を待ってSRAMバンクを1から0に切り替える

    //PicRes0to1();//PICの処理を待ってSRAMバンクを0から1に切り替える
*/
    //close(spi_fd);
    //rp1.end();
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