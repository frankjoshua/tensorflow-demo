/***************************************************************************
  This is a library for the AMG88xx GridEYE 8x8 IR camera

  This sketch makes a 64 pixel thermal camera with the GridEYE sensor
  and a 128x128 tft screen https://www.adafruit.com/product/2088

  Designed specifically to work with the Adafruit AMG88 breakout
  ----> http://www.adafruit.com/products/3538

  These sensors use I2C to communicate. The device's I2C address is 0x69

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Dean Miller & James DeVito for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ILI9341.h>
#include <SPI.h>
#include "SD.h"

#include <Wire.h>
#include <Adafruit_AMG88xx.h>
#include <Adafruit_STMPE610.h>

#ifdef ESP8266
   #define STMPE_CS 16
   #define TFT_CS   0
   #define TFT_DC   15
   #define SD_CS    2
#endif
#ifdef ESP32
   #define STMPE_CS 32
   #define TFT_CS   15
   #define TFT_DC   33
   #define SD_CS    14
#endif
#ifdef TEENSYDUINO
   #define TFT_DC   10
   #define TFT_CS   4
   #define STMPE_CS 3
   #define SD_CS    8
#endif
#ifdef ARDUINO_STM32_FEATHER
   #define TFT_DC   PB4
   #define TFT_CS   PA15
   #define STMPE_CS PC7
   #define SD_CS    PC5
#endif
#ifdef ARDUINO_NRF52832_FEATHER /* BSP 0.6.5 and higher! */
   #define TFT_DC   11
   #define TFT_CS   31
   #define STMPE_CS 30
   #define SD_CS    27
#endif
#if defined(ARDUINO_MAX32620FTHR) || defined(ARDUINO_MAX32630FTHR)
   #define TFT_DC   P5_4
   #define TFT_CS   P5_3
   #define STMPE_CS P3_3
   #define SD_CS    P3_2
#endif

// Anything else!
#if defined (__AVR_ATmega32U4__) || defined(ARDUINO_SAMD_FEATHER_M0) || defined (__AVR_ATmega328P__) || \
    defined(ARDUINO_SAMD_ZERO) || defined(__SAMD51__) || defined(__SAM3X8E__) || defined(ARDUINO_NRF52840_FEATHER)
   #define STMPE_CS 6
   #define TFT_CS   9
   #define TFT_DC   10
   #define SD_CS    5
#endif

//Comment this out to remove the text overlay
#define SHOW_TEMP_TEXT

//low range of the sensor (this will be blue on the screen)
#define MINTEMP 10

//high range of the sensor (this will be red on the screen)
#define MAXTEMP 20

//the colors we will be using
const uint16_t camColors[] = {0x480F,
0x400F,0x400F,0x400F,0x4010,0x3810,0x3810,0x3810,0x3810,0x3010,0x3010,
0x3010,0x2810,0x2810,0x2810,0x2810,0x2010,0x2010,0x2010,0x1810,0x1810,
0x1811,0x1811,0x1011,0x1011,0x1011,0x0811,0x0811,0x0811,0x0011,0x0011,
0x0011,0x0011,0x0011,0x0031,0x0031,0x0051,0x0072,0x0072,0x0092,0x00B2,
0x00B2,0x00D2,0x00F2,0x00F2,0x0112,0x0132,0x0152,0x0152,0x0172,0x0192,
0x0192,0x01B2,0x01D2,0x01F3,0x01F3,0x0213,0x0233,0x0253,0x0253,0x0273,
0x0293,0x02B3,0x02D3,0x02D3,0x02F3,0x0313,0x0333,0x0333,0x0353,0x0373,
0x0394,0x03B4,0x03D4,0x03D4,0x03F4,0x0414,0x0434,0x0454,0x0474,0x0474,
0x0494,0x04B4,0x04D4,0x04F4,0x0514,0x0534,0x0534,0x0554,0x0554,0x0574,
0x0574,0x0573,0x0573,0x0573,0x0572,0x0572,0x0572,0x0571,0x0591,0x0591,
0x0590,0x0590,0x058F,0x058F,0x058F,0x058E,0x05AE,0x05AE,0x05AD,0x05AD,
0x05AD,0x05AC,0x05AC,0x05AB,0x05CB,0x05CB,0x05CA,0x05CA,0x05CA,0x05C9,
0x05C9,0x05C8,0x05E8,0x05E8,0x05E7,0x05E7,0x05E6,0x05E6,0x05E6,0x05E5,
0x05E5,0x0604,0x0604,0x0604,0x0603,0x0603,0x0602,0x0602,0x0601,0x0621,
0x0621,0x0620,0x0620,0x0620,0x0620,0x0E20,0x0E20,0x0E40,0x1640,0x1640,
0x1E40,0x1E40,0x2640,0x2640,0x2E40,0x2E60,0x3660,0x3660,0x3E60,0x3E60,
0x3E60,0x4660,0x4660,0x4E60,0x4E80,0x5680,0x5680,0x5E80,0x5E80,0x6680,
0x6680,0x6E80,0x6EA0,0x76A0,0x76A0,0x7EA0,0x7EA0,0x86A0,0x86A0,0x8EA0,
0x8EC0,0x96C0,0x96C0,0x9EC0,0x9EC0,0xA6C0,0xAEC0,0xAEC0,0xB6E0,0xB6E0,
0xBEE0,0xBEE0,0xC6E0,0xC6E0,0xCEE0,0xCEE0,0xD6E0,0xD700,0xDF00,0xDEE0,
0xDEC0,0xDEA0,0xDE80,0xDE80,0xE660,0xE640,0xE620,0xE600,0xE5E0,0xE5C0,
0xE5A0,0xE580,0xE560,0xE540,0xE520,0xE500,0xE4E0,0xE4C0,0xE4A0,0xE480,
0xE460,0xEC40,0xEC20,0xEC00,0xEBE0,0xEBC0,0xEBA0,0xEB80,0xEB60,0xEB40,
0xEB20,0xEB00,0xEAE0,0xEAC0,0xEAA0,0xEA80,0xEA60,0xEA40,0xF220,0xF200,
0xF1E0,0xF1C0,0xF1A0,0xF180,0xF160,0xF140,0xF100,0xF0E0,0xF0C0,0xF0A0,
0xF080,0xF060,0xF040,0xF020,0xF800,};

// This is calibration data for the raw touch data to the screen coordinates
#define TS_MINX 150
#define TS_MINY 130
#define TS_MAXX 3800
#define TS_MAXY 4000

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

Adafruit_STMPE610 ts = Adafruit_STMPE610(STMPE_CS);

Adafruit_AMG88xx amg;
unsigned long delayTime;
float pixels[AMG88xx_PIXEL_ARRAY_SIZE];
uint16_t displayPixelWidth, displayPixelHeight;

struct Button {
  int id;
  int x, y, w, h;
  String text;
  int color;
  boolean isTouched;
};
#define NUM_OF_BUTTONS 4
Button buttons[NUM_OF_BUTTONS];
int colors[6] = {
  ILI9341_RED,
  ILI9341_YELLOW,
  ILI9341_GREEN,
  ILI9341_CYAN,
  ILI9341_BLUE,
  ILI9341_MAGENTA
};

void appendFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        drawMessage("Failed to open file for appending");
        return;
    }
    if(file.print(message)){
        Serial.println("Message appended");
    } else {
        drawMessage("Append failed");
    }
    file.close();
}

void writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        drawMessage("Failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Serial.println("File written");
    } else {
        drawMessage("Write failed");
    }
    file.close();
}

void createDir(fs::FS &fs, const char * path){
    Serial.printf("Creating Dir: %s\n", path);
    if(fs.mkdir(path)){
        Serial.println("Dir created");
    } else {
        drawMessage("mkdir failed");
    }
}

String idToName(int id){
  if(id == 0){
    return "pee";
  }
  if(id == 1){
    return "turd";
  }
  if(id == 2){
    return "dry";
  }
  if(id == 3){
    return "wet";
  }
  return "other";
}

void setup() {
  delay(500);
  Serial.begin(115200);
    Serial.println(F("AMG88xx thermal camera!"));

    tft.begin();
    tft.fillScreen(ILI9341_BLACK);

    displayPixelWidth = tft.width() / 8;
    displayPixelHeight = tft.width() / 8; //Keep pixels square 

    tft.setRotation(0);
    
    bool status;
    
    // default settings
    status = amg.begin();
    if (!status) {
        drawMessage("No AMG88xx sensor.");
        while (1);
    }
    
    Serial.println("-- Thermal Camera Test --");
    delay(100); // let sensor boot up

    if (!ts.begin()) {
      drawMessage("Couldn't start touchscreen controller");
      while (1);
    }

    if(!SD.begin(14)){
      drawMessage("Card Mount Failed");
      while (1);
    }
    randomSeed(analogRead(0));
    
    //Create buttons
    int width = tft.width() / NUM_OF_BUTTONS;
    for(int i = 0; i < NUM_OF_BUTTONS; i++){
      int x = width * i;
      int y = tft.height() - width;
      buttons[i] = {
        i,
        x,y,
        width,width,
        idToName(i),
        false
        };
        
        //Create folder
        // Length (with one extra character for the null terminator)
        int str_len = buttons[i].text.length() + 2; 
        
        // Prepare the character array (the buffer) 
        char char_array[str_len];
        
        // Copy it over 
        ("/" + buttons[i].text).toCharArray(char_array, str_len);
        createDir(SD, char_array);
    }   

    drawButtons();

}

void drawMessage(String msg){
  tft.fillRect(0, 0, tft.width(), 50, ILI9341_BLACK);
  tft.setCursor(0,0);
  tft.setTextColor(ILI9341_WHITE);  
  tft.setTextSize(2);
  tft.print(msg);
}

void drawButtons(){
  for(int i = 0; i < NUM_OF_BUTTONS; i++){
    //Draw button
    tft.fillRect(buttons[i].x, buttons[i].y, buttons[i].w, buttons[i].h, colors[i]);
    tft.setCursor(buttons[i].x, buttons[i].y + buttons[i].h / 2);
    tft.setTextColor(ILI9341_BLACK);  
    tft.setTextSize(2);
    tft.print(buttons[i].text);
  }
}

void loop() {
  //read all the pixels
  amg.readPixels(pixels);
  
  for(int i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++){

    int colorTemp;
    if(pixels[i] >= MAXTEMP) colorTemp = MAXTEMP;
    else if(pixels[i] <= MINTEMP) colorTemp = MINTEMP;
    else colorTemp = pixels[i];
    
    uint8_t colorIndex = map(colorTemp, MINTEMP, MAXTEMP, 0, 255);
    
    colorIndex = constrain(colorIndex, 0, 255);
    //draw the pixels!
    tft.fillRect(displayPixelHeight * floor(i / 8), 40 + displayPixelWidth * ((AMG88xx_PIXEL_ARRAY_SIZE - i - 1) % 8),
        displayPixelHeight, displayPixelWidth, camColors[colorIndex]);
        
    #ifdef SHOW_TEMP_TEXT
      tft.setCursor( displayPixelHeight * floor(i / 8) + displayPixelHeight/2 - 12, 
                     40 + displayPixelWidth * ((AMG88xx_PIXEL_ARRAY_SIZE - i - 1) % 8) + displayPixelHeight/2 - 4);
      tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(1);
      tft.print(pixels[i],1);
    #endif
    
  } 

  //Check for touches
  if (!ts.bufferEmpty()) {
    // Retrieve a point  
    TS_Point p = ts.getPoint();
    // Scale from ~0->4000 to tft.width using the calibration #'s
    p.x = map(p.x, TS_MINX, TS_MAXX, tft.width(), 0);
    p.y = map(p.y, TS_MINY, TS_MAXY, 0, tft.height());
    Serial.print(p.x);
    Serial.print(" ");
    Serial.println(p.y);
    for(int i = 0; i < NUM_OF_BUTTONS; i++){
      Button *b = &buttons[i];
      //update touched state of button
      boolean touched = b->x < p.x && b->x + b->w > p.x &&
        b->y < p.y && b->y + b->h > p.y;
      
      if(!b->isTouched && touched){
        b->isTouched = true;
        onButtonTouched(b);
      } else if (b->isTouched && !touched){
        b->isTouched = false;
      }
    }
  }



}

void onButtonTouched(Button *button){

  Serial.print(button->id);
  String name = String("/") + button->text + "/" + String(random(2147483647)) + ".CSV";
  // Length (with one extra character for the null terminator)
  int str_len = name.length() + 1; 
  // Prepare the character array (the buffer) 
  char fileName[str_len];
  // Copy it over 
  name.toCharArray(fileName, str_len);
  
  String file = "";
  for(int i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++){
    file += String(pixels[i]);
    file += ",";
  }
  Serial.println(" Button touched");
  Serial.println(name);
  Serial.println(file);
  str_len = file.length() + 1; 
  // Prepare the character array (the buffer) 
  char fileContents[str_len];
  // Copy it over 
  file.toCharArray(fileContents, str_len);
  appendFile(SD, fileName, fileContents); 
  
  drawMessage(name);
}



