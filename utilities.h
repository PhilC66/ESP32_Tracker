/* definition pour LILYGO*/
#define uS_TO_S_FACTOR      1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP       30          /* Time ESP32 will go to sleep (in seconds) */

#define UART_BAUD           115200

#define MODEM_TX            27
#define MODEM_RX            26
#define MODEM_PWRKEY        4
#define MODEM_DTR           32
#define MODEM_RI            33
#define MODEM_FLIGHT        25
#define MODEM_STATUS        34

#define SD_MISO             2
#define SD_MOSI             15
#define SD_SCLK             14
#define SD_CS               13

#define LED_PIN             12

/*
PIN 35 VBAT 
PIN 36 IO S_VP  V SOLAR IN

PIN 39 free S_VN
PIN 5  free SPI CS
PIN 18 free SPI SCK
PIN 19 free SPI MISO
PIN 23 free SPI MOSI
PIN 21 free I2C SDA
PIN 22 free I2C SCL

*/