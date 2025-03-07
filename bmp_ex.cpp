#include <stdio.h>
#include <stdlib.h>
#include "libbmp.h"

unsigned char R[256],G[256],B[256];
unsigned PicDat_r[128][256/8*16],PicDat_g[128][256/8*16],PicDat_b[128][256/8*16];


//--------PWM用に分解
void SetDat(int line){
  int c1,c2,i,j,k;
  char Pdat_r,Pdat_g,Pdat_b;   
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
          PicDat_r[line][c1]=Pdat_r;
          PicDat_g[line][c1]=Pdat_g;
          PicDat_b[line][c1]=Pdat_b;
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


int main() {
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
    return 0;
}


/*
int main() {
    bmp_img img;
    bmp_img_read(&img, "input.bmp");

    int new_width = img.img_header.biWidth / 2;
    int height = img.img_header.biHeight;

    // 新しいBMP画像を作成
    bmp_img shrunk_img;
    bmp_img_init_df(&shrunk_img, new_width, height);

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < new_width; x++) {
            // 元画像の左側のピクセルをコピー（最近傍法）
            shrunk_img.img_pixels[y][x] = img.img_pixels[y][x * 2];
        }
    }

    bmp_img_write(&shrunk_img, "output.bmp");
    bmp_img_free(&shrunk_img);
    bmp_img_free(&img);
    
    printf("横幅を半分にシュリンク完了！\n");
    return 0;
}
*/
/*
int main() {
    bmp_img img;
    bmp_img_read(&img, "input.bmp");

    for (int y = 0; y < img.img_header.biHeight; y++) {
        for (int x = 0; x < img.img_header.biWidth / 2; x++) {
            bmp_pixel temp = img.img_pixels[y][x];
            img.img_pixels[y][x] = img.img_pixels[y][img.img_header.biWidth - 1 - x];
            img.img_pixels[y][img.img_header.biWidth - 1 - x] = temp;
        }
    }

    bmp_img_write(&img, "output.bmp");
    bmp_img_free(&img);
    printf("左右反転完了！\n");
    return 0;
}
*/

