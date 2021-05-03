# GC9A01 ESP-IDF Component    

Clone to `components` folder and run `idf.py menuconfig`

![ESP-IDF_menuconfig](https://user-images.githubusercontent.com/64005694/111914456-43582400-8a87-11eb-9173-375262b5a261.jpg)

**Add as submodule:**

`git submodule add https://github.com/liyanboy74/gc9a01-esp-idf.git components/gc9a01`

**Example Test:**

```c
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#include "gc9a01.h"

#define STACK_SIZE              2048

void LCD(void * arg)
{
    uint16_t Color;
    GC9A01_Init();
    for(;;)
    {
        Color=rand();
        GC9A01_FillRect(0,0,239,239,Color);
        GC9A01_Update();
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    TaskHandle_t LCDHandle;

    xTaskCreate(LCD,"Test LCD",STACK_SIZE,NULL,tskIDLE_PRIORITY,&LCDHandle);
    configASSERT(LCDHandle);
}

```

- If you succeed, it's time to go one layer higher! Try [Dispcolor](https://github.com/liyanboy74/dispcolor)
- You can also use the [BMP24 to RGB565](https://github.com/liyanboy74/bmp24-to-rgb565) tools to convert and display images

