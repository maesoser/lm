#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2

#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

typedef struct{
  uint32_t rtotal;
  uint32_t rfree;

  uint32_t upt_days;
  uint32_t upt_hours;
  uint32_t upt_mins;
  uint32_t upt_secs;

  uint8_t s;
  uint8_t m;
  uint8_t h;
  uint8_t day;
  uint8_t month;
  uint8_t year;

  uint16_t load[3];
  uint8_t ip[4];
}serial_pkt;

serial_pkt recvpkt;
boolean dcolor = false;

void setup()   {
  Serial.begin(115200);

  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
  // init done
  display.ssd1306_command(SSD1306_SETCONTRAST);
  display.ssd1306_command(0x01);
  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.invertDisplay(false);
  display.display();
  display.setTextColor(WHITE);
  display.clearDisplay();

  for (int16_t i=0; i<min(display.width(),display.height())/2; i+=5) {
    display.drawTriangle(display.width()/2, display.height()/2-i,
                     display.width()/2-i, display.height()/2+i,
                     display.width()/2+i, display.height()/2+i, WHITE);
    display.display();
  }

}

boolean flip(boolean b){
  if(b) return false;
  return true;
}

void loop() {

  if (Serial.available() == sizeof(serial_pkt)) {
    Serial.readBytes((char*)&recvpkt, sizeof(serial_pkt));
    display.clearDisplay();
    if(dcolor) dcolor=false;
    else dcolor=true;
    display.invertDisplay(dcolor);

    //RAM
    display.setTextSize(1);
    display.setCursor(1,0);
    display.println("RAM");
    display.setCursor(32,0);
    display.println(recvpkt.rfree);
    display.setCursor(64,0);
    display.println(recvpkt.rtotal);
    display.setCursor(100,0);
    display.println("MB");

    // LOAD
    display.setCursor(1,8);
    display.println("LOAD");
    display.setCursor(36,8);
    display.println(recvpkt.load[0]/100.0);
    display.setCursor(70,8);
    display.println(recvpkt.load[1]/100.0);
    display.setCursor(100,8);
    display.println(recvpkt.load[2]/100.0);

    // TIME
    display.setTextSize(3);
    display.setCursor(2,20);
    display.println(recvpkt.h);
    display.setCursor(32,20);
    display.println(":");
    display.setCursor(44,20);
    display.println(recvpkt.m);
    display.setCursor(74,20);
    display.println(":");
    display.setCursor(90,20);
    display.println(recvpkt.s);

    display.setTextSize(1);
    display.setCursor(1,46);
    display.println("ADDR");
    display.setCursor(40,46);
    display.println(recvpkt.ip[0]);
    display.setCursor(64,46);
    display.println(recvpkt.ip[1]);
    display.setCursor(88,46);
    display.println(recvpkt.ip[2]);
    display.setCursor(110,46);
    display.println(recvpkt.ip[3]);
    
    //UPTIME
    display.setCursor(1,56);
    display.println(recvpkt.upt_days);
    display.setCursor(18,56);
    display.println(recvpkt.upt_hours);
    display.setCursor(36,56);
    display.println(recvpkt.upt_mins);
    display.setCursor(54,56);
    display.println(recvpkt.upt_secs);
    display.setCursor(74,56);
    display.println("D H M S");

    display.display();

  }

    // Clear the buffer.
}
