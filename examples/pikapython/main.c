// C标准库
#include <stdio.h>
#include <stdlib.h>

// Pikascript相关
#include "./pikapython/pikascript-lib/PikaStdDevice/pika_hal.h"
#include "pikaScript.h"

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
#define DBG_TAG "main"
#include "log.h"

// FatFS库
#include "fatfs_diskio_register.h"
#include "ff.h"

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

volatile FILE g_pika_app_flash_file = {0};
volatile int g_pika_app_flash_pos = 0;
#define _PIKA_APP_FLASH_ADDR 0x100000   // 1M
#define _PIKA_APP_FLASH_SIZE 24 * 1024  // 24K

#define _PIKA_APP_FLASH_INITED 0xFE
#define _PIKA_APP_FLASH_VOID 0xFF
#define _PIKA_APP_FLASH_SAVED 0x0F

static struct bflb_device_s* cam0;
static struct bflb_device_s* adc;

static int filesystem_init(void);
static void cam_init(void);
static void adc_init(void);

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
    if (bflb_gpio_read(gpio, GPIO_PIN_33) == 0) {
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
#if USING_LVGL
    /* lvgl init */
    lv_log_register_print_cb(lv_log_print_g_cb);
    lv_init();
    lv_port_disp_init();
    lv_port_indev_init();

    LOG_I("lvgl success\r\n");
#endif
}

static void init_adc(struct bflb_device_s *gpio) {
    /* ADC_CH0 */
    bflb_gpio_init(gpio, GPIO_PIN_20, GPIO_ANALOG | GPIO_SMT_EN | GPIO_DRV_0);
    /* ADC_CH3 */
    bflb_gpio_init(gpio, GPIO_PIN_3, GPIO_ANALOG | GPIO_SMT_EN | GPIO_DRV_0);
    /* adc init */
    adc_init();
}

static void init_cam(struct bflb_device_s *gpio) {
    /* DVP0 GPIO init */
    bflb_gpio_init(gpio, GPIO_PIN_24,
                   GPIO_FUNC_CAM | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN |
                       GPIO_DRV_1);
    bflb_gpio_init(gpio, GPIO_PIN_25,
                   GPIO_FUNC_CAM | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN |
                       GPIO_DRV_1);
    bflb_gpio_init(gpio, GPIO_PIN_26,
                   GPIO_FUNC_CAM | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN |
                       GPIO_DRV_1);
    bflb_gpio_init(gpio, GPIO_PIN_27,
                   GPIO_FUNC_CAM | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN |
                       GPIO_DRV_1);
    bflb_gpio_init(gpio, GPIO_PIN_28,
                   GPIO_FUNC_CAM | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN |
                       GPIO_DRV_1);
    bflb_gpio_init(gpio, GPIO_PIN_29,
                   GPIO_FUNC_CAM | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN |
                       GPIO_DRV_1);
    bflb_gpio_init(gpio, GPIO_PIN_30,
                   GPIO_FUNC_CAM | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN |
                       GPIO_DRV_1);
    bflb_gpio_init(gpio, GPIO_PIN_31,
                   GPIO_FUNC_CAM | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN |
                       GPIO_DRV_1);
    bflb_gpio_init(gpio, GPIO_PIN_32,
                   GPIO_FUNC_CAM | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN |
                       GPIO_DRV_1);
    bflb_gpio_init(gpio, GPIO_PIN_33,
                   GPIO_FUNC_CAM | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN |
                       GPIO_DRV_1);
    bflb_gpio_init(gpio, GPIO_PIN_34,
                   GPIO_FUNC_CAM | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN |
                       GPIO_DRV_1);

    cam_init();
}

static void init_uart(void) {
#if !USING_USB_CDC
    uartx = bflb_device_get_by_name("uart0");
#endif
}

static void create_tasks(void) {
#if PIKA_FREERTOS_ENABLE
#if USING_KEY_ERAISE
    xTaskCreate(_erase_app_task, (char *)"erase_app_task", 8192, NULL,
                configMAX_PRIORITIES - 1, NULL);
#endif
    xTaskCreate(consumer_task, (char *)"consumer_task", 8 * 1024, NULL, 3, NULL);
#if USING_USB_CDC
    xTaskCreate(usb_cdc_fflush_task, (char *)"usb_cdc_fflush_task", 1024, NULL,
                1, NULL);
#endif
#if USING_LVGL
    xTaskCreate(lvgl_task, (char *)"lvgl_task", 8 * 1024, NULL,
                configMAX_PRIORITIES - 2, NULL);
#endif
    vTaskStartScheduler();
#endif
}

int main(void) {
    board_init();

    init_filesystem();

    struct bflb_device_s *gpio;
    init_gpio(&gpio);

    init_lvgl();

    init_adc(gpio);

    init_cam(gpio);

    init_uart();

    create_tasks();

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
#if USING_FLASH_READ
        bflb_flash_read(_PIKA_APP_FLASH_ADDR + g_pika_app_flash_pos,
                        (uint8_t*)ptr, size * n);
#else
        memcpy(ptr, (const void*)(_PIKA_APP_FLASH_ADDR + g_pika_app_flash_pos),
               size * n);
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

static void cam_isr(int irq, void* arg);

static void cam_init(void) {
    struct bflb_cam_config_s cam_config;
    struct image_sensor_config_s* sensor_config;
    struct bflb_device_s* i2c0;

    i2c0 = bflb_device_get_by_name("i2c0");
    cam0 = bflb_device_get_by_name("cam0");

    if (image_sensor_scan(i2c0, &sensor_config)) {
        LOG_I("\r\nSensor name: %s\r\n", sensor_config->name);
    } else {
        LOG_E("\r\nError! Can't identify sensor!\r\n");
        cam0 = NULL;
        return;
    }

    bflb_cam_int_mask(cam0, CAM_INTMASK_NORMAL, false);
    bflb_irq_attach(cam0->irq_num, cam_isr, NULL);
    bflb_irq_enable(cam0->irq_num);

    memcpy(&cam_config, sensor_config, IMAGE_SENSOR_INFO_COPY_SIZE);
    cam_config.with_mjpeg = false;
    cam_config.output_format = CAM_OUTPUT_FORMAT_AUTO;
    static lv_color_t cam_buffer[4][320 * 240] __section(".psmram_data");
    cam_config.output_bufaddr = (uint32_t)(uintptr_t)(void*)cam_buffer;
    cam_config.output_bufsize = sizeof(cam_buffer);

    bflb_cam_init(cam0, &cam_config);
    bflb_cam_start(cam0);

    // bflb_cam_stop(cam0);
}

#define UPDATE_FREQ 2
#define TEST_COUNT 64
#define TEST_ADC_CHANNELS 2

static ATTR_NOCACHE_NOINIT_RAM_SECTION uint32_t
    adc_raw_data[2][TEST_ADC_CHANNELS * TEST_COUNT];
static void dma0_ch0_isr(void* arg);

static void adc_init(void) {
    struct bflb_adc_channel_s chan[] = {
        {.pos_chan = ADC_CHANNEL_0, .neg_chan = ADC_CHANNEL_GND},
        {.pos_chan = ADC_CHANNEL_3, .neg_chan = ADC_CHANNEL_GND},
    };

    adc = bflb_device_get_by_name("adc");

    /**
     *  adc clock = XCLK /  2 / 32 / 128 (14B) =  4.882K
     *                             /  60 (12B) = 10.38276K
     *  adc clock = XCLK / 1 /  20 / 128 (14B) = 15.625K
     */
    struct bflb_adc_config_s cfg;
    cfg.clk_div = ADC_CLK_DIV_20;
    cfg.scan_conv_mode = true;
    cfg.continuous_conv_mode = true;
    cfg.differential_mode = false;
    cfg.resolution = ADC_RESOLUTION_14B;
    cfg.vref = ADC_VREF_2P0V;

    bflb_adc_init(adc, &cfg);
    bflb_adc_channel_config(adc, chan, sizeof(chan) / sizeof(chan[0]));
    bflb_adc_link_rxdma(adc, true);

    struct bflb_device_s* dma0_ch0;
    dma0_ch0 = bflb_device_get_by_name("dma0_ch0");

    struct bflb_dma_channel_config_s config;

    config.direction = DMA_PERIPH_TO_MEMORY;
    config.src_req = DMA_REQUEST_ADC;
    config.dst_req = DMA_REQUEST_NONE;
    config.src_addr_inc = DMA_ADDR_INCREMENT_DISABLE;
    config.dst_addr_inc = DMA_ADDR_INCREMENT_ENABLE;
    config.src_burst_count = DMA_BURST_INCR1;
    config.dst_burst_count = DMA_BURST_INCR1;
    config.src_width = DMA_DATA_WIDTH_32BIT;
    config.dst_width = DMA_DATA_WIDTH_32BIT;
    bflb_dma_channel_init(dma0_ch0, &config);

    bflb_dma_channel_irq_attach(dma0_ch0, dma0_ch0_isr, NULL);

    static struct bflb_dma_channel_lli_pool_s
        lli[20]; /* max trasnfer size 4064 * 20 */
    static struct bflb_dma_channel_lli_transfer_s transfers[2];

    transfers[0].src_addr = (uint32_t)DMA_ADDR_ADC_RDR;
    transfers[0].dst_addr = (uint32_t)adc_raw_data[0];
    transfers[0].nbytes = sizeof(adc_raw_data[0]);

    transfers[1].src_addr = (uint32_t)DMA_ADDR_ADC_RDR;
    transfers[1].dst_addr = (uint32_t)adc_raw_data[1];
    transfers[1].nbytes = sizeof(adc_raw_data[1]);

    int used_count =
        bflb_dma_channel_lli_reload(dma0_ch0, lli, 20, transfers, 2);
    bflb_dma_channel_lli_link_head(dma0_ch0, lli, used_count);
    bflb_dma_channel_start(dma0_ch0);

    // bflb_adc_start_conversion(adc);

    bflb_adc_stop_conversion(adc);
}

static void cam_isr(int irq, void* arg) {
    uint16_t* pic_addr;
    uint32_t pic_size;
    // static volatile uint32_t cam_int_cnt = 0;

    bflb_cam_int_clear(cam0, CAM_INTCLR_NORMAL);
    pic_size = bflb_cam_get_frame_info(cam0, (void*)&pic_addr);
    bflb_cam_pop_one_frame(cam0);
    // printf("CAM interrupt, pop picture %d: 0x%08x, len: %d\r\n",
    // cam_int_cnt++, (uint32_t)pic_addr, pic_size);
    for (size_t i = 0; i < pic_size / sizeof(uint16_t); i++) {
        pic_addr[i] = __bswap16(pic_addr[i]);
    }
    // canvas_cam_update(pic_addr);
}

static void dma0_ch0_isr(void* arg) {
    static uint32_t dma_tc_flag0 = 0;
    dma_tc_flag0++;
    // printf("[%d]tc done\r\n", dma_tc_flag0);

    struct bflb_adc_result_s result[TEST_ADC_CHANNELS * TEST_COUNT];
    bflb_adc_parse_result(adc, adc_raw_data[!(dma_tc_flag0 & 0x1)], result,
                          TEST_ADC_CHANNELS * TEST_COUNT);

    int16_t results_temp[TEST_COUNT];
    uint32_t btn_adc_val_avg = 0;
    for (size_t j = 0, k = 0; j < TEST_ADC_CHANNELS * TEST_COUNT; j++) {
        // printf("raw data:%08x\r\n", adc_raw_data[!(dma_tc_flag0 & 0x1)][j]);
        // ADC_CHANNEL_0 min:923mv nor:952mv max:980mv
        // printf("pos chan %d,%d mv \r\n", result[j].pos_chan,
        // result[j].millivolt);
        if (ADC_CHANNEL_3 == result[j].pos_chan) {
            btn_adc_val_avg += result[j].millivolt;
        } else if (ADC_CHANNEL_0 == result[j].pos_chan) {
            results_temp[k++] = result[j].millivolt;  // - 952;
        }
    }
    btn_adc_val_avg /= TEST_COUNT;

    // chart_mic_append_data(results_temp, TEST_COUNT);
    // label_adc_btn_update(btn_adc_val_avg);
}

void btn_cam_event_handled(lv_event_t* e) {
    lv_event_code_t code = lv_event_get_code(e);

    lv_obj_t* btn = e->target;
    lv_label_t* label = (lv_label_t*)lv_obj_get_child(btn, 0);

    bool is_btn_checked = (lv_obj_get_state(btn) & LV_STATE_CHECKED);

    if (code == LV_EVENT_VALUE_CHANGED) {
        // LV_LOG_USER("%s checked %u", label->text, is_btn_checked);
        if (cam0) {
            (void (*[])(struct bflb_device_s*)){
                bflb_cam_stop, bflb_cam_start}[is_btn_checked](cam0);
        }
    }
}

void btn_adc_event_handled(lv_event_t* e) {
    lv_event_code_t code = lv_event_get_code(e);

    lv_obj_t* btn = e->target;
    lv_label_t* label = (lv_label_t*)lv_obj_get_child(btn, 0);

    bool is_btn_checked = (lv_obj_get_state(btn) & LV_STATE_CHECKED);

    if (code == LV_EVENT_VALUE_CHANGED) {
        // LV_LOG_USER("%s checked %u", label->text, is_btn_checked);
        if (adc) {
            (void (*[])(struct bflb_device_s*)){
                bflb_adc_stop_conversion,
                bflb_adc_start_conversion}[is_btn_checked](adc);
            if (!is_btn_checked) {
                lv_label_set_text(label, "ADC");
            }
        }
    }
}
