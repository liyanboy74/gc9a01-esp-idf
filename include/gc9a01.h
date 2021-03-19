//--------------------------------------------------------------------------------------------------------
// Nadyrshin Ruslan - [YouTube-channel: https://www.youtube.com/channel/UChButpZaL5kUUl_zTyIDFkQ]
// Liyanboy74
//--------------------------------------------------------------------------------------------------------
#ifndef _GC9A01_H
#define _GC9A01_H

#define GC9A01_Width	CONFIG_GC9A01_Width
#define GC9A01_Height 	CONFIG_GC9A01_Height

uint16_t GC9A01_GetWidth();
uint16_t GC9A01_GetHeight();

void GC9A01_Init();
void GC9A01_SleepMode(uint8_t Mode);
void GC9A01_DisplayPower(uint8_t On);
void GC9A01_DrawPixel(int16_t x, int16_t y, uint16_t color);
void GC9A01_FillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void GC9A01_Update();
void GC9A01_SetBL(uint8_t Value);
uint16_t GC9A01_GetPixel(int16_t x, int16_t y);

void GC9A01_Screen_Shot(uint16_t x,uint16_t y,uint16_t width ,uint16_t height,uint16_t * Buffer);
void GC9A01_Screen_Load(uint16_t x,uint16_t y,uint16_t width ,uint16_t height,uint16_t * Buffer);

#endif
