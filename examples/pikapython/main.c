#include <stdio.h>
#include <stdlib.h>
#include "./pikapython/pikascript-lib/PikaStdDevice/pika_hal.h"
#include "bflb_flash.h"
#include "bflb_gpio.h"
#include "bflb_uart.h"
#include "board.h"
#include "log.h"
#include "usbd_cdc_user.h"

#include "lv_conf.h"
#include "lvgl.h"
// #include "demos/lv_demos.h"

#include "lcd.h"

#include "lv_port_disp.h"
#include "lv_port_indev.h"
// #include "lwip/init.h"
#include "pikaScript.h"
#if PIKA_FREERTOS_ENABLE
#include "FreeRTOS.h"
#include "task.h"
#endif

/* bsp config for PikaPython */
#define USING_USB_CDC 1
#define USING_LVGL 1
#define USING_FLASH_READ 1

#define USING_KEY_ERAISE 0
#define USING_FORCE_ERASE 0

/* valid check for bsp config */
#if USING_KEY_ERAISE && USING_LVGL
#error "Using key eraise and lvgl at the same time is not supported"
#endif

#if defined(BL616)
#include "bl616_glb.h"
#elif defined(BL602)
#include "bl602_glb.h"
#elif defined(BL702)
#include "bl702_glb.h"
#elif defined(BL808)
#include "bl808_glb.h"
#endif


struct bflb_device_s* uartx = NULL;
// static uint8_t freertos_heap[configTOTAL_HEAP_SIZE];

volatile FILE g_pika_app_flash_file = {0};
volatile int g_pika_app_flash_pos = 0;
#define _PIKA_APP_FLASH_ADDR 0x100000   // 1M
#define _PIKA_APP_FLASH_SIZE 24 * 1024  // 24K

#define _PIKA_APP_FLASH_INITED 0xFE
#define _PIKA_APP_FLASH_VOID 0xFF
#define _PIKA_APP_FLASH_SAVED 0x0F

// static HeapRegion_t xHeapRegions[] = {
//     {(uint8_t*)freertos_heap, 0},
//     {NULL, 0}, /* Terminates the array. */
//     {NULL, 0}  /* Terminates the array. */
// };

static int _pika_app_check(void) {
    uint8_t buf = {0};
    FILE* f = pika_platform_fopen("app.pika", "rb");
    pika_platform_fread(&buf, 1, 1, f);
    if (buf == _PIKA_APP_FLASH_SAVED) {
        return 1;
    }
    return 0;
}

#define ERAISE_BATCH_SIZE (_PIKA_APP_FLASH_SIZE / 1)

static int _do_eraise_app(void){
    for (uint32_t i = 0; i < (_PIKA_APP_FLASH_SIZE / ERAISE_BATCH_SIZE); i++) {
        int ret = 0;
        ret = bflb_flash_erase(
            _PIKA_APP_FLASH_ADDR + i * ERAISE_BATCH_SIZE,
            ERAISE_BATCH_SIZE);
        if (ret != 0) {
            pika_platform_printf("Erase app.pika failed\r\n");
            return -1;
        }
    }
    return 0;
}

static void _eraise_app(void){
    pika_platform_printf("Erasing app.pika...\r\n");
    // pika_platform_printf("Please release the button\r\n");
    _do_eraise_app();
    pika_platform_printf("Erase app.pika done\r\n");
}

uint8_t _pika_app_buf[_PIKA_APP_FLASH_SIZE] = {0};
static void consumer_task(void* pvParameters) {
    #if USING_USB_CDC
    cdc_acm_init();
    #endif
    vTaskDelay(1000);
    #if USING_KEY_ERAISE
    struct bflb_device_s* gpio = bflb_device_get_by_name("gpio");
    bflb_gpio_init(gpio, GPIO_PIN_33,
                   GPIO_INPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_0);
    vTaskDelay(100);
    if (bflb_gpio_read(gpio, GPIO_PIN_33) == 0){
        _eraise_app();
    }
    #endif

    #if USING_FORCE_ERASE
    _eraise_app();
    #endif

    PikaObj* root = newRootObj("root", New_PikaMain);
    if (_pika_app_check()) {
        pika_platform_printf("Load app.pika from flash\r\n");
        FILE* f = pika_platform_fopen("app.pika", "rb");
        pika_platform_fread(_pika_app_buf, 1, _PIKA_APP_FLASH_SIZE, f);
        obj_linkLibrary(root, (uint8_t*)_pika_app_buf);
        obj_runModule(root, "main");
    } else {
        pika_platform_printf("Load app.pika from flash failed\r\n");
        extern unsigned char pikaModules_py_a[];
        obj_linkLibrary(root, pikaModules_py_a);
        obj_runModule(root, "main");
    }
    pikaScriptShell(root);
    while (1) {
    }
}

static void _erase_app_task(void* pvParameters) {
    struct bflb_device_s* gpio = bflb_device_get_by_name("gpio");
    bflb_gpio_init(gpio, GPIO_PIN_2,
                   GPIO_INPUT | GPIO_FLOAT | GPIO_SMT_EN | GPIO_DRV_0);
    int time = 0;
    while (1) {
        if (bflb_gpio_read(gpio, GPIO_PIN_2) == 1) {
            time++;
            if (time > 100) {
                _eraise_app();
                pika_platform_reboot();
            }
        } else {
            time = 0;
        }
        vTaskDelay(10);
    }
}

void lv_log_print_g_cb(const char *buf)
{
    printf("[LVGL] %s", buf);
}

static void lvgl_task(void* pvParameters) {
    // lv_port_indev_init();

    // lv_demo_benchmark();
    // lv_demo_stress();

    while (1) {
        lv_task_handler();
        // bflb_mtimer_delay_ms(1);
        vTaskDelay(1);
    }
}

static void usb_cdc_fflush_task(void* pvParameters) {
    while (1) {
        vTaskDelay(30);
        usb_cdc_fflush();
    }
}


int main(void) {
    board_init();
#if USING_LVGL
    /* lvgl init */
    lv_log_register_print_cb(lv_log_print_g_cb);
    lv_init();
    lv_port_disp_init();
    printf("lvgl success\r\n");
#endif
    #if !USING_USB_CDC
    uartx = bflb_device_get_by_name("uart0");
    // bflb_uart_feature_control(uartx, UART_CMD_SET_BAUD_RATE, 115200);
    #endif
    // xHeapRegions[0].xSizeInBytes = configTOTAL_HEAP_SIZE;
    // vPortDefineHeapRegions(xHeapRegions);
    // printf("Heap size: %d\r\n", configTOTAL_HEAP_SIZE);


#if PIKA_FREERTOS_ENABLE

    #if USING_KEY_ERAISE
    xTaskCreate(_erase_app_task, (char*)"erase_app_task", 8192, NULL,
                configMAX_PRIORITIES - 1, NULL);
    #endif
    xTaskCreate(consumer_task, (char*)"consumer_task", 8 * 1024, NULL, 3, NULL);
    #if USING_USB_CDC
    xTaskCreate(usb_cdc_fflush_task, (char*)"usb_cdc_fflush_task", 1024, NULL,
               1, NULL);
    #endif
    #if USING_LVGL
    xTaskCreate(lvgl_task, (char*)"lvgl_task", 8 * 1024, NULL, 2, NULL);
    #endif
    vTaskStartScheduler();
#else
    consumer_task(NULL);
#endif

    while (1) {
    }
}

/* Platform Porting */

char pika_platform_getchar(void) {
    #if !USING_USB_CDC
    while (1) {
        int c = bflb_uart_getchar(uartx);
        if (c != -1) {
            return c;
        }
    }
    #else 
    return usb_cdc_user_getchar();
    #endif
}

int pika_platform_putchar(char ch) {
    #if !USING_USB_CDC
    bflb_uart_putchar(uartx, ch);
    return 0;
    #else
    return usb_cdc_user_putchar(ch);
    #endif
}

void pika_platform_reboot(void) {
    GLB_SW_System_Reset();
}

void* pika_platform_malloc(size_t size) {
#if PIKA_FREERTOS_ENABLE
    return pvPortMalloc(size);
#else
    return malloc(size);
#endif
}

void pika_platform_free(void* ptr) {
#if PIKA_FREERTOS_ENABLE
    return vPortFree(ptr);
#else
    free(ptr);
#endif
}

/* fopen */
FILE* pika_platform_fopen(const char* filename, const char* modes) {
    if (strcmp("app.pika", filename) == 0) {
        g_pika_app_flash_pos = 0;
        FILE* fp = (FILE*)&g_pika_app_flash_file;
        if (strchr(modes, 'w') == NULL) {
            return fp;
        }
        _do_eraise_app();
        return fp;
    }
    return NULL;
}

/* fwrite */
size_t pika_platform_fwrite(const void* ptr,
                            size_t size,
                            size_t n,
                            FILE* stream) {
    if (stream == (FILE*)&g_pika_app_flash_file) {
        bflb_flash_write(_PIKA_APP_FLASH_ADDR + g_pika_app_flash_pos,
                         (uint8_t*)ptr, size * n);
        g_pika_app_flash_pos += size * n;
        return size * n;
    }
    return 0;
}

size_t pika_platform_fread(void* ptr, size_t size, size_t n, FILE* stream) {
    if (stream == (FILE*)&g_pika_app_flash_file) {
#if USING_FLASH_READ
        bflb_flash_read(_PIKA_APP_FLASH_ADDR + g_pika_app_flash_pos,
                    (uint8_t*)ptr, size * n);
#else
        memcpy(ptr, (const void*)(_PIKA_APP_FLASH_ADDR + g_pika_app_flash_pos), size * n);
#endif
        g_pika_app_flash_pos += size * n;
        return size * n;
    }
    return 0;
}

/* fclose */
int pika_platform_fclose(FILE* stream) {
    if (stream == (FILE*)&g_pika_app_flash_file) {
        for (uint32_t i = 1; i < 32; i++) {
            /* add EOF */
            pika_platform_fwrite("\0", 1, 1, stream);
        }
        return 0;
    }
    return -1;
}
