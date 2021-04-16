#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "fatfs.h"
#include "jpeglib.h"
#include "jdata_conf.h"
#include "stm32_adafruit_lcd.h"

typedef struct RGB
{
  uint8_t B;
  uint8_t G;
  uint8_t R;
}RGB_typedef;

RGB_typedef *RGB_matrix;
uint32_t line_counter = 0;

/* This struct contains the JPEG decompression parameters */
struct jpeg_decompress_struct cinfo;
/* This struct represents a JPEG error handler */
struct jpeg_error_mgr jerr;

uint16_t *RGB16Buffer;

// ----------------------------------------------------------------------------
static uint8_t Jpeg_CallbackFunction(uint8_t* Row, uint32_t DataLength) {
  uint32_t i = 0;
  RGB_matrix =  (RGB_typedef*)Row;

  for(i = 0; i < cinfo.image_width; i++) {
    RGB16Buffer[i] = (uint16_t)
    (
     ((RGB_matrix[i].R & 0x00F8) >> 3)|
     ((RGB_matrix[i].G & 0x00FC) << 3)|
     ((RGB_matrix[i].B & 0x00F8) << 8)
    );
  }
  BSP_LCD_DrawRGB16Image(0, line_counter, cinfo.image_width, 1, RGB16Buffer);
  line_counter++;
  return 0;
}

// ----------------------------------------------------------------------------
uint32_t jpeg_decode(JFILE *file, uint32_t width, uint8_t * buff, uint8_t (*callback)(uint8_t*, uint32_t)) {
  /* Decode JPEG Image */
  uint32_t ret = 0;
  JSAMPROW buffer[2] = {0}; /* Output row buffer */
  uint32_t row_stride = 0; /* physical row width in image buffer */

  buffer[0] = buff;

  /* Step 1: allocate and initialize JPEG decompression object */
  cinfo.err = jpeg_std_error(&jerr);

  /* Initialize the JPEG decompression object */
  jpeg_create_decompress(&cinfo);

  jpeg_stdio_src (&cinfo, file);

  /* Step 3: read image parameters with jpeg_read_header() */
  jpeg_read_header(&cinfo, TRUE);

  /* Step 4: set parameters for decompression */
  cinfo.dct_method = JDCT_FLOAT;

  if((cinfo.image_width <= BSP_LCD_GetXSize()) && (cinfo.image_height <= BSP_LCD_GetYSize())) {
    BSP_LCD_Clear(LCD_COLOR_BLACK);
    RGB16Buffer = JMALLOC(width * 2);
    /* Step 5: start decompressor */
    jpeg_start_decompress(&cinfo);

    row_stride = width * 3;
    while (cinfo.output_scanline < cinfo.output_height) {
      (void) jpeg_read_scanlines(&cinfo, buffer, 1);

      if (callback(buffer[0], row_stride) != 0) {
        break;
      }
    }
    /* Step 6: Finish decompression */
    jpeg_finish_decompress(&cinfo);

    JFREE(RGB16Buffer);
    ret = 1;
  }

  /* Step 7: Release JPEG decompression object */
  jpeg_destroy_decompress(&cinfo);
  return ret;
}

// ----------------------------------------------------------------------------
void jpg_view(char* fn) {
  FIL sfile;
  static uint8_t _aucLine[2048];

  if(f_open(&sfile, fn, FA_READ) == FR_OK) {
    line_counter = 0;
    if(jpeg_decode(&sfile, BSP_LCD_GetXSize(), _aucLine, Jpeg_CallbackFunction)) {
      printf("Jpg view: '%s'\r\n", fn);
    }
    f_close(&sfile);
  }
}
