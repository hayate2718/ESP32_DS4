#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "driver/spi_slave.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include <Arduino.h>
#include <SPI.h>

#define GPIO_MOSI           12
#define GPIO_MISO           13
#define GPIO_SCLK           15
#define GPIO_CS             14

#define GPIO_MOSI GPIO_NUM_12
#define GPIO_MISO GPIO_NUM_13
#define GPIO_SCLK GPIO_NUM_15
#define GPIO_CS GPIO_NUM_14

spi_device_handle_t handle;
float rxdata = 0;
uint8_t txdata[255] = {0};


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  int ret = 0; //診断用バッファ
  

  spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_MOSI,
        .miso_io_num = GPIO_MISO,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    spi_device_interface_config_t spicfg = {
      .command_bits = 0,
      .address_bits = 0,
      .dummy_bits = 0,
      .mode = 0,
      .duty_cycle_pos = 128,
      .clock_speed_hz = 1000000,
      .input_delay_ns = 0,
      .spics_io_num = GPIO_CS,
      .queue_size = 3,
    };

    buscfg.max_transfer_sz = 128;

    ret = spi_bus_initialize(SPI2_HOST, &buscfg, 1);
    assert(ret == ESP_OK);
    ret = spi_bus_add_device(SPI2_HOST, &spicfg,&handle);
    assert(ret == ESP_OK);

}

void send_data(void* txdata,uint8_t length){
  spi_transaction_t t;
  spi_transaction_t* _t;

  memset(&t,0,sizeof(t));

  t.length = 8 * length; //length specify Byte unit (max 255 Bytes)
  t.tx_buffer = txdata;

  assert(spi_device_queue_trans(handle, &t,portMAX_DELAY) == ESP_OK);
  assert(spi_device_get_trans_result(handle, &_t, portMAX_DELAY) == ESP_OK);

}

void loop() {
  // put your main code here, to run repeatedly:
    int ret = 0;
    int count = 0;
    int flag = 0;

    
    for(int i=0;i<sizeof(txdata);i++){
      txdata[i] = i;
    }
    

    while(1){
      count++;
      flag = count % 2;

    send_data(txdata,sizeof(txdata));

    //assert(spi_device_polling_transmit(handle, &t) == ESP_OK );
 
      /*
      if(flag == 1){
        txdata[0] = 15;
        txdata[1] = 17;
        txdata[2] = 255;
        txdata[3] = 2;
        ret = spi_device_transmit(handle,&t);
      
      txdata[0] = 1;
      txdata[1] = 4;
      txdata[2] = 5;
      txdata[3] = 6;        
      ret = spi_device_transmit(handle,&t);

      }else if(flag == 0){
      
      txdata[0] = 15;
      txdata[1] = 4;
      txdata[2] = 5;
      txdata[3] = 6;        
      ret = spi_device_transmit(handle,&t);
      
      txdata[0] = 16;
        txdata[1] = 17;
        txdata[2] = 255;
        txdata[3] = 2;
        ret = spi_device_transmit(handle,&t);
      }
      */
      
      Serial.printf("txdata = %d\n count = %d\n flag = %d\n",sizeof(txdata),count,flag);

      delay(1000);
    }
}
