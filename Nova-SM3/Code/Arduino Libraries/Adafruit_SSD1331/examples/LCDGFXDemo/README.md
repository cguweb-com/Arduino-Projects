This is a hello world GFX demo written and ported to multiple GFX supported backends.

It is not fully optimized for SD1331 but simply meant to show how it works on that backend, including the issue of clear() being slow. Pixmap sync was changed to work with and without full frame clear. In real life, code for these panels should be rewritten to only clear an area around the area being redrawn to avoid flashing and slower refreshes (HWSPI at full 80Mhz speed on ESP8266 can reach over 80fps but still shows noticeable flashing if you clear the whole screen before displaying the next frame).

A lot of the demo code was meant for lower resolution displays (down to 8x8 or 16x8), see http://marc.merlins.org/perso/arduino/post_2017-04-24_Adafruit-GFX-on-NeoMatrix-and-RGB-Matrix-Panel-Demo.html , but it's interesting to see how the code and the GFX library scales from 8x8 to 96x64.

![image](https://user-images.githubusercontent.com/1369412/57587320-32682000-74b8-11e9-92c0-577df34d4447.png)
![image](https://user-images.githubusercontent.com/1369412/57587336-50ce1b80-74b8-11e9-9020-043a0eeaa01d.png)


This same demo is already present in these trees:
* https://github.com/adafruit/Adafruit_NeoMatrix/tree/master/examples/MatrixGFXDemo
* https://github.com/adafruit/RGB-matrix-Panel/blob/master/examples/PanelGFXDemo_16x32/PanelGFXDemo_16x32.ino
* https://github.com/marcmerlin/FastLED_NeoMatrix/tree/master/examples/MatrixGFXDemo
* https://github.com/marcmerlin/LED-Matrix/blob/master/examples/directmatrix8x8_tricolor_direct_sr/directmatrix8x8_tricolor_direct_sr.ino
* https://github.com/mrfaptastic/ESP32-RGB64x32MatrixPanel-I2S-DMA/blob/master/examples/PanelGFXDemo/PanelGFXDemo.ino
* https://github.com/marcmerlin/SmartMatrix_GFX/tree/master/examples/MatrixGFXDemo
