#include "bflb_adc.h"
#include "bflb_dma.h"
#include "bflb_gpio.h"
#include "lvgl.h"
#include "bflb_Microphone.h"

#define TEST_ADC_CHANNELS 2
#define TEST_COUNT 80

static lv_obj_t* chart_mic = NULL;
static lv_coord_t ecg_sample[160];
static struct bflb_device_s* adc;
static uint32_t adc_raw_data[2][TEST_ADC_CHANNELS * TEST_COUNT];

static lv_obj_t* chart_mic_create(lv_obj_t* parent);
static void adc_init(void);
static void dma0_ch0_isr(void* arg);

static uint8_t g_mic_inited = 0;

static void microphone_init(void) {
    adc_init();
}

void microphone_demo(void) {
    chart_mic = chart_mic_create(lv_scr_act());
    bflb_adc_start_conversion(adc);
}

static void adc_init(void) {
    /* ADC_CH0 */
    struct bflb_device_s* gpio = bflb_device_get_by_name("gpio");
    bflb_gpio_init(gpio, GPIO_PIN_20, GPIO_ANALOG | GPIO_SMT_EN | GPIO_DRV_0);
    /* ADC_CH3 */
    bflb_gpio_init(gpio, GPIO_PIN_3, GPIO_ANALOG | GPIO_SMT_EN | GPIO_DRV_0);
        struct bflb_adc_channel_s chan[] = {
        { .pos_chan = ADC_CHANNEL_0,
          .neg_chan = ADC_CHANNEL_GND },
        { .pos_chan = ADC_CHANNEL_3,
          .neg_chan = ADC_CHANNEL_GND },
    };

    adc = bflb_device_get_by_name("adc");

    /**
     *  adc clock = XCLK / 2 / 20 / 64(14B) = 15.625K
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

    struct bflb_device_s *dma0_ch0;
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

    static struct bflb_dma_channel_lli_pool_s lli[20]; /* max trasnfer size 4064 * 20 */
    static struct bflb_dma_channel_lli_transfer_s transfers[2];

    transfers[0].src_addr = (uint32_t)DMA_ADDR_ADC_RDR;
    transfers[0].dst_addr = (uint32_t)adc_raw_data[0];
    transfers[0].nbytes = sizeof(adc_raw_data[0]);

    transfers[1].src_addr = (uint32_t)DMA_ADDR_ADC_RDR;
    transfers[1].dst_addr = (uint32_t)adc_raw_data[1];
    transfers[1].nbytes = sizeof(adc_raw_data[1]);

    int used_count = bflb_dma_channel_lli_reload(dma0_ch0, lli, 20, transfers, 2);
    bflb_dma_channel_lli_link_head(dma0_ch0, lli, used_count);
    bflb_dma_channel_start(dma0_ch0);

    // bflb_adc_start_conversion(adc);

    bflb_adc_stop_conversion(adc);
}

void chart_mic_append_data(int16_t *data, uint16_t len)
{
    uint32_t pcnt = sizeof(ecg_sample) / sizeof(ecg_sample[0]);
    if (!chart_mic || !data || !len) {
        return;
    }
    if (len > pcnt)
        len = pcnt;

    lv_obj_t *chart = chart_mic;
    lv_chart_set_point_count(chart, 0);

    memcpy(ecg_sample, ecg_sample + len, (pcnt - len) * sizeof(lv_coord_t));
    memcpy(ecg_sample + pcnt - len, data, len * sizeof(lv_coord_t));

    lv_chart_set_point_count(chart, pcnt);
}


static void dma0_ch0_isr(void* arg) {
    pika_debug("dma0_ch0_isr");
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

    chart_mic_append_data(results_temp, TEST_COUNT);
    // label_adc_btn_update(btn_adc_val_avg);
}

static lv_obj_t* chart_mic_create(lv_obj_t* parent) {
    /* Create a chart */ 
    lv_obj_t* chart;
    chart = lv_chart_create(parent);
    pika_debug("create chart: %p", chart);
    lv_obj_set_size(chart, 320, 80);
    lv_obj_align(chart, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, 0, 2000);
    /*Do not display points on the data*/
    lv_obj_set_style_size(chart, 0, LV_PART_INDICATOR);

    lv_chart_series_t* ser = lv_chart_add_series(
        chart, lv_palette_main(LV_PALETTE_RED), LV_CHART_AXIS_PRIMARY_Y);
    LV_ASSERT(ser == lv_chart_get_series_next(chart, NULL));
    pika_assert(ser == lv_chart_get_series_next(chart, NULL));

    uint32_t pcnt = sizeof(ecg_sample) / sizeof(ecg_sample[0]);
    lv_chart_set_point_count(chart, pcnt);
    lv_chart_set_ext_y_array(chart, lv_chart_get_series_next(chart, NULL),
                             (lv_coord_t*)ecg_sample);

    lv_chart_set_zoom_x(chart, LV_IMG_ZOOM_NONE);
    lv_chart_set_zoom_y(chart, LV_IMG_ZOOM_NONE);

    pika_debug("create chart OK");
    return chart;
}

void bflb_Microphone___init__(PikaObj *self){
    if (g_mic_inited){
        return;
    }
    pika_debug("microphone init");
    microphone_init();
    g_mic_inited = 1;
}

void bflb_Microphone_demo(PikaObj *self){
    pika_debug("microphone demo");
    microphone_demo();
}
