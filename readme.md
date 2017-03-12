# oled4linux

Tiny project to make a status display with a [0.96 i2c OLED display](https://es.aliexpress.com/item/0-96-Inch-Yellow-and-Blue-I2C-IIC-Serial-128X64-OLED-LCD-LED-Display-Module-for/2053302733.html?isOrigTitle=true) and an [Arduino Pro Micro](https://www.sparkfun.com/products/12640). I used [this 3D model](http://www.thingiverse.com/thing:857858) as enclosure.

![Image of the enclosure](http://thingiverse-production-new.s3.amazonaws.com/renders/ea/08/2f/a4/26/CSC_2699_preview_featured.JPG)

Now it also has some fancy terminal output in case you would like to use it with some tiny tft screens like [this one](https://www.adafruit.com/product/1601) or [this one for the odroid](http://www.hardkernel.com/main/products/prdt_info.php?g_code=G143599699669). The idea is to offer an overview about the status of your linux board without ssh it.

![terminal_gif](https://raw.githubusercontent.com/maesoser/oled4linux/master/imgs/animation.gif)

### Installation for the serial output

1. Install the [Adafruit gfx library](https://github.com/adafruit/Adafruit-GFX-Library) and the [Adafruit_SSD1306 library](https://github.com/adafruit/Adafruit_SSD1306). If you don't know how to do it, maybe you should read [this](https://www.arduino.cc/en/Guide/Libraries).

2. Burn the .ino file into the Arduino Micro Pro

3. Connect the board with the Oled this way. If you do it correctly you should see a triangle on the center of the screen:
```
ARDUINO ----- OLED
D2	----- SDA
D3	----- SCL
VCC	----- VCC
GND	----- GND		

```

4. Compile the host program and run it. If you've doubts about why I'm writting this arguments, you can always do `oled4linux -h` to see the help.:
```
make
oled4linux -v -t 5 -s /dev/ttyACM0
```

### Bibliography

[htop explained](https://peteris.rocks/blog/htop/)

### Some (shitty) photos

![photo_black](https://raw.githubusercontent.com/maesoser/oled4linux/master/imgs/photo1.jpg)

![photo_white](https://raw.githubusercontent.com/maesoser/oled4linux/master/imgs/photo2.jpg)
