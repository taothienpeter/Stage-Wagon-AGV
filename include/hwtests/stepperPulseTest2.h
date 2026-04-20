#include "driver/rmt.h"

#define RMT_TX_CHANNEL RMT_CHANNEL_0
#define RMT_TX_GPIO    18

void setup() {

    rmt_config_t config;

    config.rmt_mode = RMT_MODE_TX;
    config.channel = RMT_TX_CHANNEL;
    config.gpio_num = (gpio_num_t)RMT_TX_GPIO;
    config.clk_div = 80;   // 1 tick = 1 us
    config.mem_block_num = 1;

    config.tx_config.loop_en = false;
    config.tx_config.carrier_en = false;
    config.tx_config.idle_output_en = true;
    config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;

    rmt_config(&config);
    rmt_driver_install(config.channel, 0, 0);

    rmt_item32_t item;

    item.level0 = 1;
    item.duration0 = 10;   // HIGH 10 us
    item.level1 = 0;
    item.duration1 = 20;   // LOW 20 us

    rmt_write_items(RMT_TX_CHANNEL, &item, 1, true);
}

void loop() {}