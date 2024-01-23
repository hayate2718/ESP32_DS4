# ESP32_DS4
ESP32でDualShock4をつかうプログラム
## 仕様
DS4の信号をBluetoothでESP32に送る。<br>
ESP32はDS4とメイン制御ボードとのインターフェースにする。<br>
ESP32と制御ボードの通信はSPIでおこなう。<br>
`#define dbug_mode`を定義すると、デバッグモードになる。シリアルモニタでコントローラの状態を確認可能<br>


|ESP32|SPI_Master|
|---|---|
|||
|pin_num|func|
|12|GPIO_MOSI|
|13|GPIO_MISO|
|15|GPIO_SCLK|
|14|GPIO_CS|
|||
|bps|1Mbps|
|Data_size|64bits|
|||
|DS4_stat|bit_num|
|十字キー右|bit0|
|十字キー下|bit1|
|十字キー上|bit2|
|十字キー左|bit3|
|四角|bit4|
|バツ|bit5|
|マル|bit6|
|三角|bit7|
|L1|bit8|
|R1|bit9|
|L3|bit10|
|R3|bit11|
|Share|bit12|
|Option|bit13|
|None(Read = 0)|bit14|
|None(Read = 0)|bit15|
|L2(unsigned)|bit16 ~ bit23|
|R2(unsigned)|bit24 ~ bit31|
|L_Stick_X(signed)|bit32 ~ bit39|
|L_Stick_Y(signed)|bit40 ~ bit47|
|R_Stick_X(signed)|bit48 ~ bit55|
|R_Stick_Y(signed)|bit56 ~ bit63|
