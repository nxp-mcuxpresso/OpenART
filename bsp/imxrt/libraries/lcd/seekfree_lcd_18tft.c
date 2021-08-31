#include <rtthread.h>
#include <rtdevice.h>
#include "drivers/spi.h"

#define TFT_DISPLAY_DIR 3

#if (0==TFT_DISPLAY_DIR || 1==TFT_DISPLAY_DIR)
#define	TFT_X_MAX	128
#define TFT_Y_MAX	160
     
#elif (2==TFT_DISPLAY_DIR || 3==TFT_DISPLAY_DIR)
#define	TFT_X_MAX	160	
#define TFT_Y_MAX	128 
     
#else
#error "TFT_DISPLAY_DIR error"
     
#endif

#define RED          	0xF800
#define BLUE         	0x001F
#define YELLOW       	0xFFE0
#define GREEN        	0x07E0
#define WHITE        	0xFFFF
#define BLACK        	0x0000
#define GRAY  			0X8430
#define BROWN 			0XBC40
#define PURPLE    		0XF81F
#define PINK    		0XFE19

#define SPI_MASTER_NAME "spi30"

static struct rt_spi_device *spi_dev_lcd = NULL;
static uint16_t disp_buffer[2][TFT_X_MAX*TFT_Y_MAX];

__attribute__((naked)) static void LCDMonitor_UpdateLineRGB565(uint16_t *pLcdFB, uint16_t *pCamFB, uint32_t u64Cnt){
	__asm volatile(
		"	push {r3, r4} \n"
		"loop: \n"
		"	ldrd r3, r4, [r1], #8 \n"
		"	rev16 r3, r3 \n"
		"	rev16 r4, r4 \n"
		"	strd r3, r4, [r0], #8 \n"
		"	subs r2, #1 \n"
		"	bne loop \n"
		"	pop {r3, r4} \n"
		"	bx lr ");
	
}

static void lcd_writeIndex(uint8_t data)
{
    board_lcd_set_dc(0);

    rt_spi_send(spi_dev_lcd,&data,1);
}

static void lcd_writeData(uint8_t data)
{
    board_lcd_set_dc(1);

    rt_spi_send(spi_dev_lcd,&data,1);
}

void lcd_writedata_16bit(uint16_t dat)
{
    uint8_t dat1[2];
    dat1[0] = dat >> 8;
    dat1[1] = (uint8_t)dat;
    
    board_lcd_set_dc(1);
    rt_spi_send(spi_dev_lcd,&dat1,2);
    
}


void lcd_set_region(unsigned int x_start,unsigned int y_start,unsigned int x_end,unsigned int y_end)
{	
    assert(TFT_X_MAX>x_start);
    assert(TFT_Y_MAX>y_start);
    
    assert(TFT_X_MAX>x_end);
    assert(TFT_Y_MAX>y_end);
    
#if (0 == TFT_DISPLAY_DIR || 1 == TFT_DISPLAY_DIR)
    lcd_writeIndex(0x2a);
	lcd_writeData(0x00);
	lcd_writeData(x_start+2);
	lcd_writeData(0x00);
	lcd_writeData(x_end+2);

	lcd_writeIndex(0x2b);
	lcd_writeData(0x00);
	lcd_writeData(y_start+1);
	lcd_writeData(0x00);
	lcd_writeData(y_end+1);	

#elif(2 == TFT_DISPLAY_DIR || 3 == TFT_DISPLAY_DIR)
    lcd_writeIndex(0x2a);
	lcd_writeData(0x00);
	lcd_writeData(x_start+1);
	lcd_writeData(0x0);
	lcd_writeData(x_end+1);

	lcd_writeIndex(0x2b);
	lcd_writeData(0x00);
	lcd_writeData(y_start+2);
	lcd_writeData(0x00);
	lcd_writeData(y_end+2);	

#endif
    
    lcd_writeIndex(0x2c);
}

void lcd_clear(int color)
{
 	uint8_t i,j;
	lcd_set_region(0,0,TFT_X_MAX-1,TFT_Y_MAX-1);
 	for (i=0;i<TFT_Y_MAX;i++)
    	for (j=0;j<TFT_X_MAX;j++)
        	lcd_writedata_16bit(color);
}

void lcd_clear_ex(void* buf)
{
	lcd_set_region(0,0,TFT_X_MAX-1,TFT_Y_MAX-1);
    board_lcd_set_dc(1);
    rt_spi_send(spi_dev_lcd,buf,TFT_X_MAX*TFT_Y_MAX*2);
}

void LCDMonitor_Update(uint32_t fbNdx, uint8_t isGray,uint32_t wndH, uint32_t wndW, uint32_t pixels_addr)
{
    
}

void LCDMonitor_GetDispSize(int *disp_w, int *disp_h)
{
	*disp_w = TFT_X_MAX;
	*disp_h = TFT_Y_MAX;
}

void Update_FrameBuffer(void* buf, uint32_t w, uint32_t h, uint32_t bpp, void* handler)
{
    int i,j;
    static int idx = 0;
	uint16_t* pLcd = (uint16_t*)disp_buffer[idx];
	uint32_t t1 = w * 2 / 8;
	
	uint16_t*  pFrame = (uint16_t*)buf;
//	for (uint32_t y=0; y< h; y++) {
//		LCDMonitor_UpdateLineRGB565(pLcd, pFrame, t1);
//		
//		pLcd += w;
//		pFrame += w;
//	}
	
    lcd_set_region(0,0,w-1,h-1);
	
 	board_lcd_set_dc(1);
    rt_spi_send(spi_dev_lcd,(const void*)buf,w*h*2);
	idx ^= 1;
}

void seekfree_18tft_lcd_init()
{
    spi_dev_lcd = (struct rt_spi_device *)rt_device_find(SPI_MASTER_NAME);
    if (spi_dev_lcd == NULL)
        return;

    {
        struct rt_spi_configuration cfg;
        cfg.data_width = 8;
        cfg.mode = RT_SPI_MODE_0 | RT_SPI_MSB; /* SPI Compatible Modes 0 */
        cfg.max_hz = 30 * 1000 * 1000; /* SPI Interface with Clock Speeds Up to 20 MHz */
        rt_spi_configure(spi_dev_lcd, &cfg);
    } /* config spi */

    
    board_lcd_set_blk(1);
    board_lcd_set_dc(0);

    board_lcd_set_reset(0);
    rt_thread_mdelay(10);
    board_lcd_set_reset(1);
    
    rt_thread_mdelay(120);
    lcd_writeIndex(0x11);
    rt_thread_mdelay(120);

    lcd_writeIndex(0xB1); 
    lcd_writeData(0x01); 
    lcd_writeData(0x2C); 
    lcd_writeData(0x2D); 
    
    lcd_writeIndex(0xB2);
    lcd_writeData(0x01); 
    lcd_writeData(0x2C); 
    lcd_writeData(0x2D); 
    
    lcd_writeIndex(0xB3); 
    lcd_writeData(0x01); 
    lcd_writeData(0x2C); 
    lcd_writeData(0x2D); 
    lcd_writeData(0x01); 
    lcd_writeData(0x2C); 
    lcd_writeData(0x2D); 
    
    lcd_writeIndex(0xB4);
    lcd_writeData(0x07); 

    lcd_writeIndex(0xC0); 
    lcd_writeData(0xA2); 
    lcd_writeData(0x02); 
    lcd_writeData(0x84); 
    lcd_writeIndex(0xC1); 
    lcd_writeData(0xC5); 
    
    lcd_writeIndex(0xC2); 
    lcd_writeData(0x0A); 
    lcd_writeData(0x00); 
    
    lcd_writeIndex(0xC3); 
    lcd_writeData(0x8A); 
    lcd_writeData(0x2A); 
    lcd_writeIndex(0xC4); 
    lcd_writeData(0x8A); 
    lcd_writeData(0xEE); 
    
    lcd_writeIndex(0xC5);
    lcd_writeData(0x0E); 
    
    lcd_writeIndex(0x36);
    switch(TFT_DISPLAY_DIR)//y x v
    {
        case 0: lcd_writeData(1<<7 | 1<<6 | 0<<5);  break;
        case 1: lcd_writeData(0<<7 | 0<<6 | 0<<5);  break;
        case 2: lcd_writeData(1<<7 | 0<<6 | 1<<5);  break;
        case 3: lcd_writeData(0<<7 | 1<<6 | 1<<5);  break;
    }

    lcd_writeIndex(0xe0); 
    lcd_writeData(0x0f); 
    lcd_writeData(0x1a); 
    lcd_writeData(0x0f); 
    lcd_writeData(0x18); 
    lcd_writeData(0x2f); 
    lcd_writeData(0x28); 
    lcd_writeData(0x20); 
    lcd_writeData(0x22); 
    lcd_writeData(0x1f); 
    lcd_writeData(0x1b); 
    lcd_writeData(0x23); 
    lcd_writeData(0x37); 
    lcd_writeData(0x00); 	
    lcd_writeData(0x07); 
    lcd_writeData(0x02); 
    lcd_writeData(0x10); 
    
    lcd_writeIndex(0xe1); 
    lcd_writeData(0x0f); 
    lcd_writeData(0x1b); 
    lcd_writeData(0x0f); 
    lcd_writeData(0x17); 
    lcd_writeData(0x33); 
    lcd_writeData(0x2c); 
    lcd_writeData(0x29); 
    lcd_writeData(0x2e); 
    lcd_writeData(0x30); 
    lcd_writeData(0x30); 
    lcd_writeData(0x39); 
    lcd_writeData(0x3f); 
    lcd_writeData(0x00); 
    lcd_writeData(0x07); 
    lcd_writeData(0x03); 
    lcd_writeData(0x10);  
    
    lcd_writeIndex(0x2a);
    lcd_writeData(0x00);
    lcd_writeData(0x00+2);
    lcd_writeData(0x00);
    lcd_writeData(0x80+2);
    
    lcd_writeIndex(0x2b);
    lcd_writeData(0x00);
    lcd_writeData(0x00+3);
    lcd_writeData(0x00);
    lcd_writeData(0x80+3);
    
    lcd_writeIndex(0xF0); 
    lcd_writeData(0x01); 
    lcd_writeIndex(0xF6);
    lcd_writeData(0x00); 
    
    lcd_writeIndex(0x3A);
    lcd_writeData(0x05); 
    lcd_writeIndex(0x29);
	
    for(int i=0;i<TFT_X_MAX*TFT_Y_MAX;i++)
        disp_buffer[0][i] = GRAY;

	lcd_clear_ex(&disp_buffer[0]);
}




INIT_DEVICE_EXPORT(seekfree_18tft_lcd_init);