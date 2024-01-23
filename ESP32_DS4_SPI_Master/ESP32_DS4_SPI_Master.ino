 /*
  ESPRESSIF社公式より
  Misxellaneous System APIs
  https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/misc_system_api.html#_CPPv414esp_mac_type_t
  */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "driver/spi_slave.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include <Arduino.h>
#include <SPI.h>
#include <PS4Controller.h>
#include <stdio.h>

//define

#define GPIO_MOSI           12
#define GPIO_MISO           13
#define GPIO_SCLK           15
#define GPIO_CS             14

#define GPIO_MOSI GPIO_NUM_12
#define GPIO_MISO GPIO_NUM_13
#define GPIO_SCLK GPIO_NUM_15
#define GPIO_CS GPIO_NUM_14

//global variables declaration

spi_device_handle_t handle;

typedef union{
			  int8_t signed_data;
		    uint8_t unsigned_data;
}analog_data; //符号付き、符号なしを同様に扱うための共用体

typedef union{
  struct{
      uint8_t bit_0 : 1; //right
      uint8_t bit_1 : 1; //down
      uint8_t bit_2 : 1; //up
      uint8_t bit_3 : 1; //left
      uint8_t bit_4 : 1; //square
      uint8_t bit_5 : 1; //Cross
      uint8_t bit_6 : 1; //Circle
      uint8_t bit_7 : 1; //Triangle
      uint8_t bit_8 : 1; //L1
      uint8_t bit_9 : 1; //R1
      uint8_t bit_A : 1; //L3
      uint8_t bit_B : 1; //R3
      uint8_t bit_C : 1; //Share
      uint8_t bit_D : 1; //Options
      uint8_t bit_E : 1; //none
      uint8_t bit_F : 1; //none
  };
	uint32_t all_data : 14;
}digital_data; //スイッチの数は14個なので14bitsで状態を表す

typedef union{
  uint8_t data[8];
  uint64_t rawdata;

}rawdata_64bits;

//function declaration

void SPI_Send_Data(void* _txdata,uint8_t length){ //SPI send data function
  spi_transaction_t t;
  spi_transaction_t* _t;

  memset(&t,0,sizeof(t));

  t.length = 8 * length; //length specify Byte unit (max 255 Bytes)
  t.tx_buffer = _txdata;

  assert(spi_device_queue_trans(handle, &t,portMAX_DELAY) == ESP_OK);
  assert(spi_device_get_trans_result(handle, &_t, portMAX_DELAY) == ESP_OK);

}

void DS4_Recive_Data(void * _Data ){//64bits以上のメモリが必要
  digital_data DS4_switch_stat;
  analog_data DS4_analog_stat[6];

  DS4_switch_stat.all_data = 0;

  if (PS4.isConnected()) {

    DS4_switch_stat.bit_0 = PS4.Right();
    DS4_switch_stat.bit_1 = PS4.Down();
    DS4_switch_stat.bit_2 = PS4.Up();
    DS4_switch_stat.bit_3 = PS4.Left();

    DS4_switch_stat.bit_4 = PS4.Square();
    DS4_switch_stat.bit_5 = PS4.Cross();
    DS4_switch_stat.bit_6 = PS4.Circle();
    DS4_switch_stat.bit_7 = PS4.Triangle();

    DS4_switch_stat.bit_8 = PS4.L1();
    DS4_switch_stat.bit_9 = PS4.R1();

    DS4_switch_stat.bit_A = PS4.L3();
    DS4_switch_stat.bit_B = PS4.R3();

    DS4_switch_stat.bit_C = PS4.Share();
    DS4_switch_stat.bit_D = PS4.Options();

    DS4_analog_stat[0].unsigned_data = PS4.L2Value(); //符号なし1Byte
    DS4_analog_stat[1].unsigned_data = PS4.R2Value(); //符号なし1Byte
    DS4_analog_stat[2].signed_data = PS4.LStickX(); //符号あり1Byte
    DS4_analog_stat[3].signed_data = PS4.LStickY(); //符号あり1Byte
    DS4_analog_stat[4].signed_data = PS4.RStickX(); //符号あり1Byte
    DS4_analog_stat[5].signed_data = PS4.RStickY(); //符号あり1Byte

    memcpy(_Data,&DS4_switch_stat,sizeof(DS4_switch_stat));
    memcpy(_Data+sizeof(DS4_switch_stat),&DS4_analog_stat,sizeof(DS4_analog_stat));

  }

}

// setup function

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  uint8_t ESP_bt_mac[6] = {0};
  char DS4_bt_mac[30];

  PS4.begin(DS4_bt_mac);
  
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

//main routine

void loop() {
  // put your main code here, to run repeatedly:
  
    uint8_t txdata[8] = {0};
    uint64_t *_txdata_64bits;
    _txdata_64bits = (uint64_t *)txdata;

    while(1){

      DS4_Recive_Data(txdata);
      SPI_Send_Data(txdata,sizeof(txdata));

      
      Serial.printf("raw txdata = %ld\n txdata size = %d\n",*_txdata_64bits,sizeof(txdata));

      delay(1000);
    }
}
