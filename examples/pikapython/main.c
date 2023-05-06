// C标准库
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

// Pikascript相关
#include "./pikapython/pikascript-lib/PikaStdDevice/pika_hal.h"
#include "pikaScript.h"

#include <lwip/tcpip.h>
#include <lwip/sockets.h>
#include <lwip/netdb.h>

// 板级驱动
#include "bflb_adc.h"
#include "bflb_cam.h"
#include "bflb_dma.h"
#include "bflb_flash.h"
#include "bflb_gpio.h"
#include "bflb_l1c.h"
#include "bflb_mtimer.h"
#include "bflb_uart.h"
#include "board.h"
#include "csi_rv32_gcc.h"
#include "image_sensor.h"
#include "touch.h"

// USB驱动
#include "usbd_cdc_user.h"

// Log库
#include "log.h"

// LVGL图形库
#include "lv_conf.h"
#include "lv_port_disp.h"
#include "lv_port_indev.h"
#include "lvgl.h"

// LCD驱动
#include "lcd.h"

// FreeRTOS库（可选）
#if PIKA_FREERTOS_ENABLE
#include "FreeRTOS.h"
#include "task.h"
#endif

// 日志库
#include "log.h"

// FatFS库
#include "fatfs_diskio_register.h"
#include "ff.h"

/* bsp config for PikaPython */
#define USING_USB_CDC 1
#define USING_LVGL 1
#define USING_TOUCH 1
#define USING_APP_XIP 1

#define USING_KEY_ERAISE 0
#define USING_FORCE_ERASE 0

/* valid check for bsp config */
#if USING_KEY_ERAISE && USING_LVGL
#error "Using key eraise and lvgl at the same time is not supported"
#endif

#if !USING_LVGL && USING_TOUCH
#warning "Using touch and not using lvgl at the same time is not supported"
#undef USING_TOUCH
#define USING_TOUCH 0
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

volatile FILE g_pika_app_flash_file = {0};
volatile int g_pika_app_flash_pos = 0;
static volatile int g_usb_cdc_init = 0;
int wifi_start_firmware_task(void);
#define _PIKA_APP_FLASH_ADDR 0x200000   // 2M

#if USING_APP_XIP
#define _PIKA_APP_FLASH_SIZE 1024 * 128
#else
#define _PIKA_APP_FLASH_SIZE 1024 * 24
#endif

#if !USING_APP_XIP
uint8_t g_pika_app_flash_buf[_PIKA_APP_FLASH_SIZE] = {0};
#endif

#define _PIKA_APP_FLASH_INITED 0xFE
#define _PIKA_APP_FLASH_VOID 0xFF
#define _PIKA_APP_FLASH_SAVED 0x0F

static struct bflb_device_s* adc;

static int filesystem_init(void);

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

static int _do_eraise_app(void) {
    for (uint32_t i = 0; i < (_PIKA_APP_FLASH_SIZE / ERAISE_BATCH_SIZE); i++) {
        int ret = 0;
        ret = bflb_flash_erase(_PIKA_APP_FLASH_ADDR + i * ERAISE_BATCH_SIZE,
                               ERAISE_BATCH_SIZE);
        if (ret != 0) {
            pika_platform_printf("Erase app.pika failed\r\n");
            return -1;
        }
    }
    return 0;
}

static void _eraise_app(void) {
    pika_platform_printf("Erasing app.pika...\r\n");
    // pika_platform_printf("Please release the button\r\n");
    _do_eraise_app();
    pika_platform_printf("Erase app.pika done\r\n");
}

static void consumer_task(void* pvParameters) {
#if USING_USB_CDC
    cdc_acm_init();
#endif
    vTaskDelay(1000);
    g_usb_cdc_init = 1;
    // init_cam(bflb_device_get_by_name("gpio"));

#if USING_KEY_ERAISE
    struct bflb_device_s* gpio = bflb_device_get_by_name("gpio");
    bflb_gpio_init(gpio, GPIO_PIN_33,
                   GPIO_INPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_0);
    vTaskDelay(100);
    if (bflb_gpio_read(gpio, GPIO_PIN_33) == 0) {
        _eraise_app();
    }
#endif

#if USING_FORCE_ERASE
    _eraise_app();
#endif

    PikaObj* root = newRootObj("root", New_PikaMain);
    pks_printVersion();
    if (_pika_app_check()) {
        uint8_t* lib_buf = NULL;
        #if USING_APP_XIP
            lib_buf = (uint8_t*)FLASH_XIP_BASE + _PIKA_APP_FLASH_ADDR - bflb_flash_get_image_offset();
        #else
            lib_buf = g_pika_app_flash_buf;
            bflb_flash_read(_PIKA_APP_FLASH_ADDR, lib_buf, _PIKA_APP_FLASH_SIZE);
        #endif
        pika_platform_printf("Load app.pika from flash\r\n");
        obj_linkLibrary(root, lib_buf);
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

void lv_log_print_g_cb(const char* buf) {
    printf("[LVGL] %s", buf);
}

static void lvgl_task(void* pvParameters) {
    while (1) {
        lv_task_handler();
        vTaskDelay(1);
    }
}

static void usb_cdc_fflush_task(void* pvParameters) {
    while (1) {
        vTaskDelay(30);
        usb_cdc_fflush();
    }
}

static void init_filesystem(void) {
    filesystem_init();
}

static void init_gpio(struct bflb_device_s **gpio) {
    *gpio = bflb_device_get_by_name("gpio");
    /* backlight pin */
    bflb_gpio_init(*gpio, GPIO_PIN_2, GPIO_OUTPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_0);
    bflb_gpio_set(*gpio, GPIO_PIN_2);
}

static void init_lvgl(void) {
    /* lvgl init */
    struct bflb_device_s *gpio;
    init_gpio(&gpio);
    lv_log_register_print_cb(lv_log_print_g_cb);
    lv_init();
    lv_port_disp_init();
    uint8_t point_num = 0;

#if USING_TOUCH
    touch_coord_t touch_max_point = {
        .coord_x = LCD_W,
        .coord_y = LCD_H,
    };
    int ret = touch_init(&touch_max_point);
    touch_coord_t touch_coord;
    if (ret == 0){
        ret = touch_read(&point_num, &touch_coord, 1);
    }
    printf("touch_read ret:%d\r\n", ret);
    /* check touch is connected */
    if(ret == 0){
        lv_port_indev_init();
    }
#endif

    LOG_I("lvgl success\r\n");
}

static void create_tasks(void) {
#if 1
    xTaskCreate(consumer_task, (char *)"consumer_task", 4 * 1024, NULL, 3, NULL);
#endif
#if USING_KEY_ERAISE
    xTaskCreate(_erase_app_task, (char *)"erase_app_task", 8192, NULL,
                configMAX_PRIORITIES - 1, NULL);
#endif
#if USING_USB_CDC && 1
    xTaskCreate(usb_cdc_fflush_task, (char *)"usb_cdc_fflush_task", 128, NULL,
                1, NULL);
#endif
#if USING_LVGL && 1
    xTaskCreate(lvgl_task, (char *)"lvgl_task", 4 * 1024, NULL,
                configMAX_PRIORITIES - 16, NULL);
#endif
}

int main(void) {
    board_init();

    init_filesystem();

#if USING_LVGL
    init_lvgl();
#endif

    create_tasks();

    // wifi_start_firmware_task();

    vTaskStartScheduler();

    while (1) {
        /* delay */
#if PIKA_FREERTOS_ENABLE
        vTaskDelay(10);
#else
        bflb_mtimer_delay_ms(10);
#endif
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
        bflb_flash_read(_PIKA_APP_FLASH_ADDR + g_pika_app_flash_pos, 
            (uint8_t*)ptr, size * n);
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

static int filesystem_init(void) {
    static FATFS fs;
    static __attribute((aligned(8))) uint32_t workbuf[4 * 1024];

    MKFS_PARM fs_para = {
        .fmt = FM_FAT32,     /* Format option (FM_FAT, FM_FAT32, FM_EXFAT and
                                FM_SFD) */
        .n_fat = 1,          /* Number of FATs */
        .align = 0,          /* Data area alignment (sector) */
        .n_root = 1,         /* Number of root directory entries */
        .au_size = 512 * 32, /* Cluster size (byte) */
    };

    FRESULT ret;

    board_sdh_gpio_init();

    fatfs_sdh_driver_register();

    ret = f_mount(&fs, "/sd", 1);

    if (ret == FR_NO_FILESYSTEM) {
        LOG_W("No filesystem yet, try to be formatted...\r\n");

        ret = f_mkfs("/sd", &fs_para, workbuf, sizeof(workbuf));

        if (ret != FR_OK) {
            LOG_F("fail to make filesystem\r\n");
            return ret;
        }

        if (ret == FR_OK) {
            LOG_I("done with formatting.\r\n");
            LOG_I("first start to unmount.\r\n");
            ret = f_mount(NULL, "/sd", 1);
            LOG_I("then start to remount.\r\n");
        }
    } else if (ret != FR_OK) {
        LOG_F("fail to mount filesystem,error= %d\r\n", ret);
        LOG_F("SD card might fail to initialise.\r\n");
        return ret;
    } else {
        LOG_D("Succeed to mount filesystem\r\n");
    }

    if (ret == FR_OK) {
        LOG_I("FileSystem cluster size:%d-sectors (%d-Byte)\r\n", fs.csize,
              fs.csize * 512);
    }

    return ret;
}

void pika_platform_panic_handle() {
}
