#include "TouchScreen.h" // only when you want to use touch screen 
#include <Adafruit_GFX.h>
#include <MCUFRIEND_kbv.h>
#include "SPI.h"  // using sdcard for display bitmap image

MCUFRIEND_kbv tft(A3, A2, A1, A0, A4);  

#define NAVY 0x000F
#define DARKGREEN 0x03E0
#define DARKCYAN 0x03EF
#define MAROON 0x7800
#define PURPLE 0x780F
#define OLIVE 0x7BE0
#define LIGHTGREY 0xC618
#define DARKGREY 0x7BEF
#define BLUE 0x001F
#define GREEN 0x07E0
#define CYAN 0x07FF
#define RED 0xF800
#define MAGENTA 0xF81F
#define YELLOW 0xFFE0
#define WHITE 0xFFFF
#define ORANGE 0xFD20
#define GREENYELLOW 0xAFE5
#define PINK 0xF81F
#define BLACK   0x0000
#define MINPRESSURE 0
#define MAXPRESSURE 1000

uint16_t screenWidth;
uint16_t screenHeight;
const int XP=7,XM=A1,YP=A2,YM=6; //ID=0x9341
const int TS_LEFT=922,TS_RT=167,TS_TOP=167,TS_BOT=922;

TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);

//tft.reset resets the screen
int BaudRate = 9600;
//reset button at (0,0)  (50, 30)

void ScreenClear(MCUFRIEND_kbv& disp, bool doClear = true){
  disp.fillScreen(BLACK);
  disp.fillRect(0, 0, disp.width(), 30, NAVY);
  disp.fillRect(0, disp.height() - 30, disp.width(), 30, DARKGREEN);
  disp.setCursor(50, 0);
  disp.setTextSize(4);
  disp.print("CLEAR");
  disp.setCursor(70, disp.height() - 30);
  disp.print("SEND");
  if(doClear){
    char message [5] = {'C', 'L', 'E', 'A', 'R'}; 
    Serial.write(message, 5);
  }
}

float radius = 1.0f;

uint16_t pixel_x = 0x0000;
uint16_t pixel_y = 0x0000;
bool wasPressed = false;
void getTouch(){
    TSPoint p = ts.getPoint();
    pinMode(YP, OUTPUT);      //restore shared pins
    pinMode(XM, OUTPUT);
    bool pressed = (p.z > MINPRESSURE && p.z < MAXPRESSURE);
    if (pressed) {
        pixel_x = map(p.y, TS_TOP, TS_BOT, 0, tft.width()); //.kbv makes sense to me
        pixel_y = map(p.x, TS_LEFT, TS_RT, 0, tft.height());
        pixel_y = tft.height() - pixel_y;
        if(pixel_y < 30){
          ScreenClear(tft); 
        }
        else if(pixel_y > (tft.height() - 30)){
          ScreenClear(tft, false);
          char message [5] {'S', 'E', 'N', 'D', 'K'}; 
          Serial.write(message, 5);
        }
        
        else{
          tft.fillCircle(pixel_x,pixel_y, radius,WHITE);
          byte xChar1 = lowByte(pixel_x);
          byte xChar2 = highByte(pixel_x);
          byte yChar1 = lowByte(pixel_y);
          byte yChar2 = highByte(pixel_y);
          byte zeroChar = 0x00;
          byte message [5] = {zeroChar, xChar1, xChar2, yChar1, yChar2};
          /*
          Serial.print("X: ");
          Serial.print(pixel_x);
          Serial.print(" Y: ");
          Serial.print(pixel_y);
          Serial.print('\n');
          */
          Serial.write(message, 5);
          wasPressed = true;
        }
    }
    else if(!pressed && wasPressed){
      byte xChar1 = lowByte(pixel_x);
      byte xChar2 = highByte(pixel_x);
      byte yChar1 = lowByte(pixel_y);
      byte yChar2 = highByte(pixel_y);
      //Tell the computer that there is no pressure
      char message [5] = {'E',  xChar1, xChar2, yChar1, yChar2}; 
      Serial.write(message, 5);
      wasPressed = false;
    }
}

void setup() {
  delay(5);
  Serial.begin(BaudRate);
  // put your setup code here, to run once:
  uint16_t ID = tft.readID();
  if (ID == 0xD3D3) ID = 0x9486; // write-only shield
  tft.begin(ID);
  tft.fillScreen(BLACK);
  tft.setRotation(1);            //LANDSCAPE

  ScreenClear(tft, false);
}



void loop() {
  //detect button press
  //getTouch();
  //  tft.fillScreen(BLACK);
  //Draw at pressure point
  getTouch();
}
