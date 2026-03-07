#pragma once
#include <Adafruit_SSD1306.h>
#include <Wire.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
#define S_SDA 21
#define S_SCL 22

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void initDisplay()
{
  Wire.begin(S_SDA, S_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
  {
    Serial.println("SSD1306 init failed");
    return;
  }
  display.clearDisplay();
  display.setTextSize(1);  // 6x8 px font -> 21 chars x 4 lines
  display.setTextColor(SSD1306_WHITE);
  display.display();
}

// Write up to 4 lines to the display. Pass "" for unused lines.
void displayStatus(
  const char * line0, const char * line1 = "", const char * line2 = "", const char * line3 = "")
{
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(line0);
  display.println(line1);
  display.println(line2);
  display.println(line3);
  display.display();
}
