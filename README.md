# EnviroSenseCube v1.0
Monitor environment by Vincenzo Rivelli.

Features:
- Temperature, Relative humidity, Barometric pressure && Air Quality.
- Low power optimization && battery monitor.
- E-Paper Display.
- ThingSpeak channel over WiFi.

Hardware:
- Adafruit Huzzah ESP8266 ( 3.3volt logic ).
- Adafruit PowerBoost 500 Charger.
- LiPo battery 1800mAh ( EEMB Cell ).
- Adafruit BME688 ( I2C address: Default 0x77, 0x76 with closed bridge ).
- Adafruit LC709203F LiPoly/LiIon Fuel Gauge. ( I2C address fixed: 0x0B ). Need the same power as the logic level of your microcontroller ( 3.3v ).
- Adafruit On-Off Power Button ADA1683 ( Power Switch ).
- Aceinnolab Universal E-Paper driver board for 24-pin SPI.
- GoodDisplay GDEW0097T50 E-Paper Mini 0.97inc 184x88 24pin SPI. Min.FULL upd.: 180sec(3min.), Min.PARTIAL upd.: 5sec. https://www.good-display.com/product/434.html https://www.good-display.com/news/80.html#precaution
- Acrilic Case.

GPIO Pin map: #0, #2, #4, #5, #12, #13, #14, #15, #16.
- HUZZAH GPIO #16 Pin             -> HUZZAH GPIO Rst. Pin. ( For the DeepSleep awake ).
- HUZZAH TX Pin                   -> UART Cable WHITE.
- HUZZAH RX Pin                   -> UART Cable GREEN.
- HUZZAH V+ Pin                   -> UART Cable RED.
- HUZZAH GND Pin                  -> UART Cable BLACK.
- HUZZAH GPIO #13 Pin             -> E-Paper driver SPI SDI.
- HUZZAH GPIO #14 Pin             -> E-Paper driver SPI CLK.
- HUZZAH GPIO #2 Pin              -> E-Paper driver SPI CS.
- HUZZAH GPIO #15 Pin             -> E-Paper driver SPI DC.
- HUZZAH GPIO #0 Pin              -> E-Paper driver SPI RST.
- HUZZAH GPIO #12 Pin             -> E-Paper driver SPI BUSY.
- HUZZAH GPIO #4 Pin (i2c SDA)    -> BME688 SDI Pin    -> LC709203F SDA Pin (JST-SH Blue).
- HUZZAH GPIO #5 Pin (i2c SCL)    -> BME688 SCK Pin    -> LC709203F SCL Pin (JST-SH Yellow).

Power management:
- Charging time: 6 hours and 15 minutes. (Yellow LED (near pins): Charging; Green LED (near screw): Charge complete). 
- Autonomy: 11 days 4 hours and 40 minutes. ( SLEEP_TIME = 5m ).

![IMG_2259](https://github.com/user-attachments/assets/81306cf2-93c1-4d25-b090-27cabb6a0408)
![IMG_1986](https://github.com/user-attachments/assets/09794eaf-fb2d-4d55-a748-8d1918120da7)

