
/*
20250307 Ando Hideyuki
Pi5　ー＞　サッカードディスプレイ　駆動のためのプログラム
受動的画像切り替え版 - Processing側からの書き込み検知で切り替え

20250511
２枚の基盤をカスケード、PICを同期　書き込みに　160msくらいかかる
２枚の絵が交互に現れるようにする。

*/
#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <linux/spi/spidev.h>
#include <string.h>
#include <errno.h>
#include "rp1-gpio.h"
#include <stdlib.h>
#include "libbmp.h"
#include <time.h>

unsigned char R[256],G[256],B[256];
unsigned char PicDat_r1[128*256/8*16],PicDat_g1[128*256/8*16],PicDat_b1[128*256/8*16];
unsigned char PicDat_r2[128*256/8*16],PicDat_g2[128*256/8*16],PicDat_b2[128*256/8*16];

//debug用
unsigned char read_data[0x1ffff];

#define CS_R1 18  // GPIO18 -> 物理ピン12
#define CS_G1 17  // GPIO17 -> 物理ピン11
#define CS_B1 16  // GPIO16 -> 物理ピン36

#define CS_R2 6   // GPIO6 -> 物理ピン31
#define CS_G2 12  // GPIO12 -> 物理ピン32
#define CS_B2 13  // GPIO13 -> 物理ピン33

#define SPI_DEV "/dev/spidev1.0"
#define SPI_SPEED 20000000  // SPIクロック速度（30MHz）
#define WRITE 0x02  // 書き込みコマンド
#define READ  0x03  // 読み込みコマンド

int spi_fd; // グローバルで1つに統合
RP1_GPIO rp1;

// 現在表示中のSRAMバンク (0 or 1)
int current_bank = 0;

// ファイル監視用変数
time_t last_mtime_A1 = 0, last_mtime_A2 = 0;
time_t last_mtime_B1 = 0, last_mtime_B2 = 0;

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
    rp1.digitalWrite(4, rp1.HIGH);//PICを動作させる  

    //CSピンの設定
    rp1.pinMode(CS_R1, rp1.OUTPUT);
    rp1.pinMode(CS_G1, rp1.OUTPUT);
    rp1.pinMode(CS_B1, rp1.OUTPUT);
    rp1.digitalWrite(CS_R1, rp1.HIGH); // 非選択状態にしておく
    rp1.digitalWrite(CS_G1, rp1.HIGH);
    rp1.digitalWrite(CS_B1, rp1.HIGH);
    rp1.pinMode(CS_R2, rp1.OUTPUT);
    rp1.pinMode(CS_G2, rp1.OUTPUT);
    rp1.pinMode(CS_B2, rp1.OUTPUT);
    rp1.digitalWrite(CS_R2, rp1.HIGH); // 非選択状態にしておく
    rp1.digitalWrite(CS_G2, rp1.HIGH);
    rp1.digitalWrite(CS_B2, rp1.HIGH);
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
    reset_sram_mode(CS_R1);
    mode = check_sram_mode(CS_R1);
    printf("Sram_R Mode Register: 0x%02X\n", mode);
   
    // SRAM G
    reset_sram_mode(CS_G1);

    // SRAM B
    reset_sram_mode(CS_B1);

    reset_sram_mode(CS_R2);
    reset_sram_mode(CS_G2);
    reset_sram_mode(CS_B2);
}

void sram_write_single(const uint8_t *data, int cs_pin) {
    uint8_t cmd[4] = { WRITE, 0x00, 0x00, 0x00 };
    struct spi_ioc_transfer xfer[2] = {0};
    xfer[0].tx_buf = (unsigned long)cmd;
    xfer[0].len = 4;
    xfer[1].tx_buf = (unsigned long)data;
    xfer[1].len = 0x10000;
    rp1.digitalWrite(cs_pin, rp1.LOW);
    ioctl(spi_fd, SPI_IOC_MESSAGE(2), xfer);
    rp1.digitalWrite(cs_pin, rp1.HIGH);
}

//--------PWM用に分解
void SetDat_A(int line){
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
            PicDat_r1[line*512+c1]=Pdat_r;
            PicDat_g1[line*512+c1]=Pdat_g;
            PicDat_b1[line*512+c1]=Pdat_b;
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

//--------PWM用に分解
void SetDat_B(int line){
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
            PicDat_r2[line*512+c1]=Pdat_r;
            PicDat_g2[line*512+c1]=Pdat_g;
            PicDat_b2[line*512+c1]=Pdat_b;
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

int bitmap_process_A(const char *filename) {
    bmp_img img;
    
    // ファイル存在チェック
    if(access(filename, R_OK) != 0) {
        printf("ファイル読み込み不可: %s (処理続行)\n", filename);
        return -1;
    }
    
    // BMP読み込み
    if(bmp_img_read(&img, filename) != 0) {
        printf("BMP読み込み失敗: %s (処理続行)\n", filename);
        return -1;
    }
    
    // サイズチェック
    int width = img.img_header.biWidth;
    int height = abs(img.img_header.biHeight);
    
    if(width != 256 || height != 256) {
        printf("サイズエラー: %dx%d (期待値: 256x256) in %s (処理続行)\n", 
               width, height, filename);
        bmp_img_free(&img);
        return -1;
    }
    
    printf("画像サイズ: 幅=%d, 高さ=%d\n", width, height);
    
    // 既存処理（width, heightの再宣言を削除）
    int x, line = 0;
    for(x = 0; x < width; x += 2){
        for (int y = 0; y < height; y++) {
            bmp_pixel p = img.img_pixels[y][x];
            R[y] = p.red >> 4;
            G[y] = p.green >> 4;
            B[y] = p.blue >> 4;
        }
        SetDat_A(line);
        line++;
    }
    
    bmp_img_free(&img);
    return 0;  // 成功
}

int bitmap_process_B(const char *filename) {
    bmp_img img;
    
    if(access(filename, R_OK) != 0) {
        printf("ファイル読み込み不可: %s (処理続行)\n", filename);
        return -1;
    }
    
    if(bmp_img_read(&img, filename) != 0) {
        printf("BMP読み込み失敗: %s (処理続行)\n", filename);
        return -1;
    }
    
    int width = img.img_header.biWidth;
    int height = abs(img.img_header.biHeight);
    
    if(width != 256 || height != 256) {
        printf("サイズエラー: %dx%d (期待値: 256x256) in %s (処理続行)\n", 
               width, height, filename);
        bmp_img_free(&img);
        return -1;
    }
    
    printf("画像サイズ: 幅=%d, 高さ=%d\n", width, height);
    
    int x, line = 0;
    for(x = 0; x < width; x += 2){
        for (int y = 0; y < height; y++) {
            bmp_pixel p = img.img_pixels[y][x];
            R[y] = p.red >> 4;
            G[y] = p.green >> 4;
            B[y] = p.blue >> 4;
        }
        SetDat_B(line);
        line++;
    }
    
    bmp_img_free(&img);
    return 0;
}

void switch_to_bank(int bank) {
    printf("SRAMバンク切り替え: %d -> %d\n", current_bank, bank);
    
    rp1.digitalWrite(4, rp1.LOW);  // PICを停止
    while((rp1.digitalRead(23)) == 1) {
        usleep(1000);  // 1ms待機
    }
    
    if (bank == 0) {
        rp1.digitalWrite(26, rp1.LOW);   // SRAM0に接続
    } else {
        rp1.digitalWrite(26, rp1.HIGH);  // SRAM1に接続
    }
    
    rp1.digitalWrite(4, rp1.HIGH);  // PIC再開
    current_bank = bank;
    
    printf("SRAMバンク切り替え完了: バンク%d\n", current_bank);
}

// ファイル更新をチェックする関数
int check_file_updates() {
    struct stat st;
    int updates = 0;
    
    // A1.bmp チェック
    if (stat("A1.bmp", &st) == 0) {
        if (st.st_mtime > last_mtime_A1) {
            printf("ファイル更新検知: A1.bmp\n");
            last_mtime_A1 = st.st_mtime;
            updates |= 1;  // bit 0
        }
    }
    
    // A2.bmp チェック
    if (stat("A2.bmp", &st) == 0) {
        if (st.st_mtime > last_mtime_A2) {
            printf("ファイル更新検知: A2.bmp\n");
            last_mtime_A2 = st.st_mtime;
            updates |= 2;  // bit 1
        }
    }
    
    // B1.bmp チェック
    if (stat("B1.bmp", &st) == 0) {
        if (st.st_mtime > last_mtime_B1) {
            printf("ファイル更新検知: B1.bmp\n");
            last_mtime_B1 = st.st_mtime;
            updates |= 4;  // bit 2
        }
    }
    
    // B2.bmp チェック
    if (stat("B2.bmp", &st) == 0) {
        if (st.st_mtime > last_mtime_B2) {
            printf("ファイル更新検知: B2.bmp\n");
            last_mtime_B2 = st.st_mtime;
            updates |= 8;  // bit 3
        }
    }
    
    return updates;
}

void process_and_display_A() {
    printf("=== A画像処理開始 ===\n");
    
    if (bitmap_process_A("A1.bmp") == 0 && bitmap_process_B("A2.bmp") == 0) {
        // SRAM書き込み
        sram_write_single(PicDat_r1, CS_R1);
        sram_write_single(PicDat_g1, CS_G1);
        sram_write_single(PicDat_b1, CS_B1);
        sram_write_single(PicDat_r2, CS_R2);
        sram_write_single(PicDat_g2, CS_G2);
        sram_write_single(PicDat_b2, CS_B2);
        
        // バンク0に切り替え（A画像表示）
        switch_to_bank(0);
        
        printf("=== A画像表示中 ===\n");
    } else {
        printf("A画像の処理に失敗しました\n");
    }
}

void process_and_display_B() {
    printf("=== B画像処理開始 ===\n");
    
    if (bitmap_process_A("B1.bmp") == 0 && bitmap_process_B("B2.bmp") == 0) {
        // SRAM書き込み
        sram_write_single(PicDat_r1, CS_R1);
        sram_write_single(PicDat_g1, CS_G1);
        sram_write_single(PicDat_b1, CS_B1);
        sram_write_single(PicDat_r2, CS_R2);
        sram_write_single(PicDat_g2, CS_G2);
        sram_write_single(PicDat_b2, CS_B2);
        
        // バンク1に切り替え（B画像表示）
        switch_to_bank(1);
        
        printf("=== B画像表示中 ===\n");
    } else {
        printf("B画像の処理に失敗しました\n");
    }
}

int main() {
    printf("受動的画像切り替えプログラム開始\n");
    
    // 初期化（1回のみ）
    gpio_init();
    if (spi_init() < 0) {
        return -1;
    }
    sram_init();
    
    // 初期表示（A画像）
    process_and_display_A();
    
    printf("ファイル更新監視開始...\n");
    
    while(1) {
        int updates = check_file_updates();
        
        if (updates & 3) {  // A1.bmp または A2.bmp が更新された
            process_and_display_A();
        }
        else if (updates & 12) {  // B1.bmp または B2.bmp が更新された
            process_and_display_B();
        }
        
        // 100ms待機
        usleep(100000);
    }
    
    close(spi_fd);
    rp1.end();
    return 0;
}