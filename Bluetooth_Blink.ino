// Arduino SD image viewer
// Written by Stanley Huang <stanleyhuangyc@gmail.com>
//
// This program requires the UTFT library.
//

#include <UTFT.h>
#include <SD.h>

// Declare which fonts we will be using
extern uint8_t SmallFont[];
extern uint8_t BigFont[];

// for Arduino 2009/Uno
UTFT myGLCD(ILI9481,38,39,40,41);   // Remember to change the model parameter to suit your display module!

// for Arduino Mega
//UTFT myGLCD(ITDB32S,38,39,40,41);   // Remember to change the model parameter to suit your display module!

#define SD_PIN 4

File root;

#define SCREEN_WIDTH 480
#define SCREEN_HEIGHT 320

int cmd = 'a';
int i = 0;

void ShowMessage(String msg1, const char* msg2 = 0)
{
    myGLCD.setColor(255, 0, 0);
    myGLCD.fillRoundRect(50, 190, 270, 230);
    myGLCD.setColor(255, 255, 255);
    myGLCD.setBackColor(255, 0, 0);
    myGLCD.print(msg1, CENTER, 196);
    if (msg2) myGLCD.print(msg2, CENTER, 210);
}

void setupBlueToothConnection(){
  Serial.begin(38400); //Set BluetoothBee BaudRate to default baud rate 38400
  Serial.print("\r\n+STWMOD=1\r\n"); //set the bluetooth work in slave mode
  Serial.print("\r\n+STNA=Wall-e\r\n"); //set the bluetooth name as "Wall-e"
  Serial.print("\r\n+STPIN=1234\r\n");
  Serial.print("\r\n+STOAUT=1\r\n"); // Permit Paired device to connect me
  Serial.print("\r\n+STAUTO=0\r\n"); // Auto-connection should be forbidden here
  delay(2000); // This delay is required.
  Serial.print("\r\n+INQ=1\r\n"); //make the slave bluetooth inquirable
  //Serial.println("The slave bluetooth is inquirable!");
  delay(2000); // This delay is required.
  Serial.print("r\n+CONN=98,D3,31,50,4A,D2\r\n");
  Serial.print("\r\n+RTPIN=1234\r\n");
  
  Serial.flush();
}


void LoadImage(File& file)
{
    for (int y = 0; y < SCREEN_HEIGHT && file.available(); y++) {
        uint16_t buf[SCREEN_WIDTH];
        for (int x = SCREEN_WIDTH - 1; x >= 0; x--) {
            byte l = file.read();
            byte h = file.read();
            buf[x] = ((uint16_t)h << 8) | l;
        }
        myGLCD.drawPixelLine(0, y, SCREEN_WIDTH, buf);
        //myGLCD.print("loading",200,200,0);
    }
    //myGLCD.print("done",200,200,0);
}

void setup() {
  setupBlueToothConnection();
  pinMode(13,OUTPUT);
  Serial.flush();

  // Setup the LCD
    myGLCD.InitLCD();
    myGLCD.setFont(SmallFont);
    myGLCD.fillScr(0, 0, 255);
    myGLCD.print("Hello",200,200,0);

    pinMode(SS,OUTPUT);
    if (!SD.begin(SD_PIN)) {
        ShowMessage("SD not ready");
        return;
    }
    pinMode(13, OUTPUT);
}
void loop() {
  String imageIs;
  cmd = 'd';
  if (Serial.available() > 0) {
    cmd = Serial.read();
    //Serial.write(cmd);
  }
  if (cmd == 'a') {
    digitalWrite(13,LOW);
  }
  if (cmd == 'e') {
    digitalWrite(13,HIGH);
    for(int k=0;k<40;k++)
    {
      Serial.write('J');
      Serial.write('e');
      Serial.write('l');
      Serial.write('l');
      Serial.write('o');
    }
    pinMode(SS, OUTPUT);
//    if (!SD.begin(10)) {
//        ShowMessage("SD not ready");
//        return;
//    }

    delay(1000);
    imageIs = "/PICTURES/" + String(i) + ".RAW";
    //ShowMessage(imageIs);
    root = SD.open("/PICTURES/"+String(i)+".RAW");
    LoadImage(root);
    root.close();
    pinMode(13, OUTPUT);
    i ++;
  }
}
