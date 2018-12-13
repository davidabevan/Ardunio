#include <memorysaver.h>
#include <UTFT.h>
//for Compass
int x, y;
int windGaugePanelX=320;//671;
int windGaugePanelY=120;
//extern uint8_t Inconsola[];
extern uint8_t Inconsola[9124];
char* conds[]={"\"local_epoch\":","\"weather\":","\"temp_c\":","\"relative_humidity\":","\"wind_dir\":","\"wind_kph\":","\"pressure_mb\":","\"feelslike_c\":"};
extern uint8_t SmallFont[];
UTFT myGLCD ( ILI9341_S5P, 5, 4, 2 );
void setup() {
  // put your setup code here, to run once:

   myGLCD.InitLCD();
   myGLCD.clrScr();
   myGLCD.setFont(Inconsola);
   myGLCD.setColor(VGA_RED);
}

void loop() {
  // put your main code here, to run repeatedly:
//drawArrow;
//drawGaugePanel;
compass;
//drawGaugeLines;
}
void drawArrow(float m)
{
  float x1, y1, x2, y2, x3, y3, x4, y4;
  int pm = m-1;

  myGLCD.setColor(0, 0, 0);
  if (pm==-1)
  pm=59;
  pm=pm*6;
  pm=pm+270;
  
  x1=80*cos(pm*0.0175);
  y1=80*sin(pm*0.0175);
  x2=5*cos(pm*0.0175);
  y2=5*sin(pm*0.0175);
  x3=30*cos((pm+4)*0.0175);
  y3=30*sin((pm+4)*0.0175);
  x4=30*cos((pm-4)*0.0175);
  y4=30*sin((pm-4)*0.0175);
  
  

  myGLCD.setColor(0, 255, 0);
  m=m*6;
  m=m+270;
  
  x1=80*cos(m*0.0175);
  y1=80*sin(m*0.0175);
  x2=5*cos(m*0.0175);
  y2=5*sin(m*0.0175);
  x3=30*cos((m+4)*0.0175);
  y3=30*sin((m+4)*0.0175);
  x4=30*cos((m-4)*0.0175);
  y4=30*sin((m-4)*0.0175);
  
  myGLCD.drawLine(x1+windGaugePanelX, y1+windGaugePanelY, x3+windGaugePanelX, y3+windGaugePanelY);
  myGLCD.drawLine(x3+windGaugePanelX, y3+windGaugePanelY, x2+windGaugePanelX, y2+windGaugePanelY);
  myGLCD.drawLine(x2+windGaugePanelX, y2+windGaugePanelY, x4+windGaugePanelX, y4+windGaugePanelY);
  myGLCD.drawLine(x4+windGaugePanelX, y4+windGaugePanelY, x1+windGaugePanelX, y1+windGaugePanelY);
}


//Wind Panel (Wind direction print)
 void compass(){
  char *wind_dir="North";
  int *wind_kph;
if (strcmp (wind_dir, "North")==0) {drawArrow(0) ;}
if (strcmp (wind_dir, "NNE")==0) {drawArrow(4.5) ;}
if (strcmp (wind_dir, "NE")==0) {drawArrow(7.5);}
if (strcmp (wind_dir, "ENE")==0) {drawArrow(12);}
if (strcmp (wind_dir, "East")==0) {drawArrow(15);}
if (strcmp (wind_dir, "ESE")==0) {drawArrow(19.5);}
if (strcmp (wind_dir, "SE")==0) {drawArrow(22.5);}
if (strcmp (wind_dir, "SSE")==0) {drawArrow(27);}
if (strcmp (wind_dir, "South")==0) {drawArrow(30);}
if (strcmp (wind_dir, "SSW")==0) {drawArrow(34);}
if (strcmp (wind_dir, "SW")==0) {drawArrow(37.5);}
if (strcmp (wind_dir, "WSW")==0) {drawArrow(41.5);}
if (strcmp (wind_dir, "West")==0) {drawArrow(45);}
if (strcmp (wind_dir, "WNW")==0) {drawArrow(49.5);}
if (strcmp (wind_dir, "NW")==0) {drawArrow(53);}
if (strcmp (wind_dir, "NNW")==0) {drawArrow(56) ;}
Serial.println(wind_dir);//me
myGLCD.setColor(0, 255, 0);

//if ((wind_kph) < 10) {myGLCD.print("  ", 596, 242);
//  myGLCD.printNumI(wind_kph, 620, 242);
//}
//else {
//myGLCD.printNumI(wind_kph, 596, 242);}
myGLCD.print("Km/h", 660, 242);
 }

void drawGaugeLines(float s)
{
  s = (s-0.1);
  float x1, y1, x2, y2;
  int ps = s-1;
  
  myGLCD.setColor(0, 0, 0);
  if (ps==-1)
  ps=59;
  ps=ps*6;
  ps=ps+270;
  
  x1=95*cos(ps*0.0175);
  y1=95*sin(ps*0.0175);
  x2=80*cos(ps*0.0175);
  y2=80*sin(ps*0.0175);
  
  

  myGLCD.setColor(0, 0, 0);
  s=s*6;
  s=s+270;
  
  x1=65*cos(s*0.0175);
  y1=65*sin(s*0.0175);
  x2=30*cos(s*0.0175);
  y2=30*sin(s*0.0175);
  
  myGLCD.drawLine(x1+windGaugePanelX, y1+windGaugePanelY, x2+windGaugePanelX, y2+windGaugePanelY);
}
void drawGaugePanel()
{
  

//Wind gauge panel
myGLCD.setColor(VGA_NAVY);
myGLCD.fillCircle(670, 120, 110);
myGLCD.setColor(VGA_RED);
myGLCD.fillCircle(670, 120, 70);
myGLCD.setColor(VGA_SILVER);
myGLCD.fillCircle(670, 120, 5);

for (float i = 0; i < 60; i+=7.5)
{
  drawGaugeLines(i);
}


myGLCD.setBackColor(VGA_TRANSPARENT);
myGLCD.setColor(VGA_SILVER);
myGLCD.setFont(Inconsola);
myGLCD.print("N", 660,18);
myGLCD.print("NE", 728,38,47);
myGLCD.print("E", 750,110);
myGLCD.print("SE", 705,195,-48);
myGLCD.print("S", 660,200);
myGLCD.print("SW", 600,160,47);
myGLCD.print("W", 568,110);
myGLCD.print("NW", 586,58,-42);
}
        
