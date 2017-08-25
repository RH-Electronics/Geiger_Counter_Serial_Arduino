/*
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*

  ACSi Geiger counter, by Dave Knight  20170622

As customized for use with:

    RHelectronics.net DIY geiger counter board: ($43, incl s&h)
     http://www.ebay.com/itm/SOLDERED-DIY-Geiger-Counter-Kit-Nuclear-Radiation-Detector-for-Arduino-iPhone-/321499888801?rmvSB=true

    STS-5 Geiger Muller tube:  $15 various sebay sellers

    MPJA's LCD shield: ($12 + s&h)
     http://www.mpja.com/LCD-Display-Arduino-Compatible-Shield/productinfo/31059+MP/

    Arduino Uno R3 clone: ($11 free s&h for Prime members))
     https://www.amazon.com/SainSmart-Arduino-ATmega328P-Development-Board/dp/B00E5WJSHK

    total cost for major parts: about $80

Note: Arduino Uno clones and LCD/keypad shields are also available from
various Chinese outfits on Ebay for $3 each or less.

NOTE: prices & links valid on 20170622

==========================================================================

Interconnection: RH Electronics DIY Geiger counter + Arduino/LCD-Keypad

    RH ELecronics board has 3 pins labeled +5v, INT, GND, on the left end.
    These pins connect to the Arduino pins: +5v, D02, GND, respectively.

    The LCD-Keypad shield installs on top of the Arduino Uno board.  The 
    pinouts on the top right are:  D13, R12, D11, D03, D02, D01, D00.  If
    the LCD does not display letters on newGeiger startup, the contrast
    may be too low.  Use a small flat screwdriver to turn the little brass
    screw on the trim pot clockwise until the white letters are displayed.

==========================================================================

  Display: line0: CPM (instantaneous) | Average CPM
           line1: mR/h:x.xx  uSv/h

Calibration Option: 

  calFactor is a multiplier applied to CPM to account for GM tube
           variations and/or deterioration over time.
	   
	   To establish the initial "knownSourceCount" for a GM tube:
              - set default calFactor (1.0) using the Serial/LCD UI
              - get an 5 minute average CPM for a known calibration source*
	      - record this CPM with the GM tube.  

	   Test the counter periodically with the same known source.
	   If the CPM consistently differs from the original CPM, use
	   calFactor to bring the CPM into "calbration":
	      - calculate:

                    calFactor = knownSourceCount / averageCPM

              - enter calFactor via the Serial/LCD UI
              - SAVE EEPROM.

  * A convenient "known source" is the ion chamber from a smoke detector.
  Typically, these ion chambers contain about 1 uCi of Americium,
  which is primarily an Alpha emitter, shielded by an aluminum cap.
  Americium also emits some low-energy Beta rays, which can be measured
  by common GM tubes like the STS-5 and SBM-20.  Most ion chambers, when
  placed very close to the GM tube will register between 120 and 200 CPM.

  DO NOT ATTEMPT to use the calFactor to "cross calibrate" different
  GM tubes!  The alpha, beta & gamma sensitivity differences among GM
  tubes are neither linear nor proportional.

*/

//
//  $Revision: 1.31.2.4 $   $Name: RHelec-01_dmk $
//

#define TRUE 1
#define FALSE 0
#include <Wire.h>
#include <avr/eeprom.h>
#include <EEPROM.h>
#include <LiquidCrystal.h>
#include <stdlib.h>

int uiState = 0;  // lcdUI state variable
int hitCount = 0;

void usage(void);
int read_LCD_buttons();

// Uncomment the #define DBUG to enable verbose debug output
//      (adds about 1.5K code + 100 bytes of RAM for msgs)
//#define DBUG 
char dbug = 0;  // toggle 0/1 via serialUI "d" cmd
//char plotOn = 0;  // toggle 0/1 via serialUI "p" cmd
char plotOn = 1;  // startup to watch CPM & avg with IDE Serial Plotter

char buf[30];   // short general output buffer (for sprintf, etc)
char cmd[20];   // serial UI input buffer

//
// LCD Pins mandated by MPJA 31059 LCD Shield:
//
#define LCD_RS 8
#define LCD_EN 9
#define LCD_D4 4
#define LCD_D5 5
#define LCD_D6 6
#define LCD_D7 7

LiquidCrystal lcd(LCD_RS,LCD_EN,LCD_D4,LCD_D5,LCD_D6,LCD_D7); 

//
//  text msgs, may put in PROGMEM if RAM gets crowded
//
//char acsi[] = "ACSi Geiger ctr";
char rev[] = "$Revision: 1.31.2.4 $"; 
char *revNo = 0;
//char units0[] = "CPM inst:average";
//char units1[] = " mR/h     uSv/h ";
//char ready[] = "Ready";

//
// displayed variables & control
//
unsigned long cps = 0;
unsigned long cpm = 0;
unsigned long plotCpm = 0;
unsigned long secStart = 0;
unsigned long a60cpm = 0;
unsigned long lct = 0;
float mRph = 0;
float uSph = 0;

//  plot period, milliseconds
#define PLOT_PERIOD 100
unsigned long nextPlot = 0;
//
// Configurable values (see serial UI code)
//
#define DEFAULT_ID "ACSi-RH"
#define DEFAULT_CALFACTOR 1.0
float calFactor = DEFAULT_CALFACTOR;  // calibration factor
// NO_CLICKS is the time during which there should be at least one GM hit
//           if NO_CLICKS milliseconds passes without any counts, the GM tube
//           is probably not working or radiation is so high that it has
//           ceased to function.  Either way, some action needs to be taken
//           e.g. RUN AWAY or make repairs of some sort.  In the event of
//           this error, post "COUNT FAILURE" to LCD line1.
//
#define NO_CLICKS  60000

//
//  EEPROM layout - set eeOffset > 0 if location in eeprom changed due to
//    errors use these definitions in calls to:
//
//         EEPROM.[put|get](eeLoc, data)
//
int eeOffset = 0;
struct eeData
{
  char ID[8];  // program ID = "ACSi.gc" - 7 chars + NULL
  float calFactor;
} ee; 
#define EE_ID (eeOffset + (int)&ee.ID - (int)&ee)
#define EE_CALFACTOR (eeOffset + (int)&ee.calFactor - (int)&ee)

//
//  EEPROM data access functions
//
void initEEPROM (void)
{   // used to init EEPROM on a new AVR chip
  strcpy(ee.ID, DEFAULT_ID);
  ee.calFactor = DEFAULT_CALFACTOR;
  EEPROM.put (eeOffset, ee);
}

void readEEPROM (void)
{
  EEPROM.get (eeOffset, ee);  // retrieve the whole struct
  if (strcmp(ee.ID, DEFAULT_ID) != 0 ) {  // EEPROM set for this app?
    initEEPROM();  // no, initialize it now
  }
}

//  NOTE: using the F(...) "flash-helper" macro puts "print strings"
//        in PROGMEM & saves RAM for variable use.  This works for both
//        Serial.print() and lcd.print().  Overall, using F() for as
//        many strings as possible freed more than 500 bytes of RAM in
//        this sketch.

void showEEPROM (void)
{
  readEEPROM();
  Serial.println(F("EEPROM:"));
  Serial.print(F("ID:"));
  Serial.println(ee.ID);
  Serial.print(F("calFactor:"));
  Serial.println (ee.calFactor);
}

void saveEEPROM (void)
{   
    ee.calFactor = calFactor;
    EEPROM.put(eeOffset, ee);  // update struct in EEPROM
    showEEPROM();
}
//
// user interface stuff
//
#define detectPin 2
//

volatile byte GMhits = 0;

unsigned long now = 0;
unsigned long seconds = 0;
unsigned long prevScount = 0;
unsigned long lastClick = 0;

//   
// ISR - counts geiger hits 
//
void bumpCount(){
  GMhits++;
}

//
//  LCD funcs
//
void clearBuf(){
  int m;
  for(m = 0;m < (signed)sizeof(buf);m++) {
    buf[m] = 0;
  }
}


void setup(){
  Serial.begin(9600);
  //
  // generate & display startup splash -> LCD
  //
  // Pull revNo from RCS $Revision: 1.31.2.4 $ expansion
  strtok(rev," "); // ignore 1st token
  revNo = strtok(0," ");   // 2nd token is revision number
  clearBuf();
  Serial.print(F("ACSi Geiger Counter"));
  sprintf(buf,"Rev %s", revNo);  // buf used here and below for lcd splash
  Serial.println(buf);
  //
  // Get saved settings from EEPROM
  //
  readEEPROM ();
  calFactor = ee.calFactor;
  if (calFactor <= 0) initEEPROM();

  //
  // Serial monitor splash: usage message to serialUI @ restart (e.g. when
  // serial monitor, etc. connects.)
  //
  usage();  

  //
  // display LCD splash screens
  //
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0,0);
  clearLine(0);
  lcd.print(F("ACSi Geiger ctr"));
  showLine(1,buf); // Rev set in buf above
  delay(2000);
  //showLine(1,ready);
  clearLine(1);
  lcd.print(F("Ready"));
  delay(1000);
  clearLine(0);
  lcd.print(F("CPM inst:average"));
  clearLine(1);
  lcd.print(F(" mR/h     uSv/h "));
  delay(5000);
  //
  //  set up lcd UI banner
  //
  if (dbug > 0 ) {
    setHitsBanner();
  } else {
    setStdBanner();
  }
  //
  // setup  GM hit detector ISR, using std hardware interrupt
  //    on Arduino UNO pin D02 (interrupt 0) or Versalino pin A/D1
  //
  //    Note that the pulse from GM board is a negative pulse, so
  //    the normal (PULL_UP) state of detectPin is HIGH (+5v) and we want to
  //    detect the only leading edge (FALLING).
  //    
  GMhits = 0;
  pinMode(detectPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(detectPin), bumpCount,  FALLING);
  lastClick = millis();  // init lastClick to detect COUNT FAILURE
}

//
// simple moving average of 60 samples
//

#define MAX_SAMPLES 60
unsigned long sumit = 0;
unsigned long sma( unsigned long s ){
  static int samples = 0;
  if ( samples < MAX_SAMPLES ) samples++;
  sumit = (sumit + s) - (sumit / samples);
  return sumit / samples;
}

//
// main loop 
//
void loop(){
  lct++;
  now = millis();

  // 
  // manage serial plotter output
  //
  if ( plotOn ) {
    if (now >= nextPlot) {
      nextPlot = now + PLOT_PERIOD;
      Serial.print(plotCpm);
      Serial.print('\t');
      Serial.println(a60cpm);
      plotCpm = 0;  // reset
    }
  }
  //
  //   save total geiger tube hits (incremented by ISR)
  // 
  if(dbug && GMhits){
    Serial.print("GMhits:");
    Serial.println(GMhits);
  }
  if (GMhits) {
    unsigned char newHits = 0;
    lastClick = now;
    // may not be necessary to disble/enable interrupts
    // if GMhits variable is only 8 bits
    noInterrupts();
    newHits = GMhits;
    GMhits = 0;
    interrupts();
    prevScount += newHits;
    if(dbug){
      Serial.print("prevScount/newHits:");
      Serial.print(prevScount);
      Serial.print("/");
      Serial.println(newHits);
    }
  }
  
  //
  // dispatch serial port User Interface task
  //
  serialUI();

  //
  //  dispatch the lcd UI if necessary
  //
  if (hitCount == 0 ) {
    // skip buttonCheck until post uiFSM() cleanup
    hitCount = buttonCheck();
    if ( hitCount ) delay(200);  // a debounce of sorts
  }
  processHits();  // process hits, if any
  showBanner();

  // ---------------------------------------------
  // Now do the once/second stuff
  // ---------------------------------------------
#ifdef DBUG
    if ( dbug ) {
	Serial.print("calFactor:");
	Serial.println(calFactor);
      sprintf(buf,"now/secStart:%lu/%lu\n", now, secStart);
      Serial.print(buf);
    }
#endif

  if ( now >= secStart + 1000) {
#ifdef DBUG
    if ( dbug ) {
      sprintf(buf,"l/s:%lu\n", lct);
      Serial.print(buf);
    }
#endif
    lct=0;
    secStart = now;
    seconds++;  // bump seconds

// do not move these calculations!
    cps = prevScount;
    cpm = cps * 60 * calFactor;
    plotCpm = cpm;   // instantaneous cpm for serial plotter
    a60cpm = sma(cpm);  // simple moving average for 60 seconds
//
#ifdef DBUG
    { char tbuf[60];  //use larger temp buffer to avoid overrun of buf[30]
      if ( dbug ) {
        sprintf(tbuf,"@ %lu cps:%lu cpm:%lu a60cpm:%lu ",
           seconds, cps, cpm, a60cpm);
        Serial.print(tbuf);
      }
    }
#endif

// -------------------------------------------------------------------------
//
//  Line0: instantaneous CPM and average CPM
//  Line1: calculated mR/h and uSv/h
//
//  approximate Calculations:   cpm/1500 -> mRph (aka cpm * .0007)
//                              mRph*10 -> uSvph  (aka cpm * .007)
//
//  Note that multiply is much faster than divide on an Arduino
//
// -------------------------------------------------------------------------

    // report instantaneous mR/h
    mRph = cpm * 0.0007;  //  1/1500 is 0.0007
    //  calculate uSv/hr 
    uSph = cpm * 0.007;  //  1/150 is 0.007
#ifdef DBUG
    if ( dbug ) {
      Serial.print(F("mR/hr:"));
      Serial.println(mRph);
      Serial.print(F("uS/hr:"));
      Serial.println(uSph);
    }
#endif
    //
    if ( (now - lastClick) < NO_CLICKS ) {
      if (uiState == 0){
        sprintf(buf,"cpm%6lu:%6lu", cpm, a60cpm);
        showLine(0,buf);
        clearLine(1);
        // use lcd.print for floats
        lcd.setCursor(0,1);
        lcd.print(F("mR/h "));
        lcd.print(mRph);
        lcd.print(F(":"));
        lcd.print(uSph);
      }
      prevScount = 0;
    } else {
      // no GM hits in NO_CLICKS seconds 
      // post error message on LCD
      if (uiState == 0 ){
        showLine(1, "COUNT FAILURE!");  // display error msg
      }
    }
  }
}  // end of loop()

/*
*
*  serialUI - a serial port user interface for the geiger counter
* 
*/

// Implementing usage() as a for loop, printing an array of char pointers
// (vs. a series of Serial.println("msg") calls) can increase or decrease
// the binary size, depending on where the char * array is declared.
// When this char * array is global, executable size is reduced by 32 bytes;
// when declared inside the function, binary size is reduced by 44 bytes.
// 20120115-DMK
/*
  const char *um[] = {
    "Usage: command [arg1]",
    " Commands:",
    "  h,? - help",
    "  show - settings",
    "  reset - set defaults",
    "  calFactor [v]", 
    "   where: v = factor",
    "  When v omitted, set default",
    "  SAVE - to EEPROM",
    "  d - dbug on/off",
    "  p - plot on/off",
    NULL
  };

//
// usage() - print strings from um array until string ptr is NULL
//
void usage(){
  const char **ump = um;
  while (*ump != NULL ) {
    Serial.println(*ump);  
    ump++;
  }
}
*/

//
//   NOTE: moving usage() messages to PROGMEM with F(...) to free up RAM
//   increased the sketch/flash size by 112 bytes and reduced RAM use by
//   282 bytes (curious).
//
void usage(){
    Serial.println(F("Usage: command [arg1]"));
    Serial.println(F(" Commands:"));
    Serial.println(F("  h,? - help"));
    Serial.println(F("  show - settings"));
    Serial.println(F("  reset - set defaults"));
    Serial.println(F("  calFactor [v]")); 
    Serial.println(F("   where: v = factor"));
    Serial.println(F("  When v omitted, set default"));
    Serial.println(F("  SAVE - to EEPROM"));
    Serial.println(F("  d - dbug on/off"));
    Serial.println(F("  p - plot on/off"));
}

//
//  A simple serial command interpreter
//
int nn = 0;          // cmd string index
char logit = 0;      
//     enable/1 disable/0 cmd parsing debug output w/UI b/e cmds

int state = 0;  // serialUI thread state variable

#define MAX_TOKENS 20

void showParms(void) {
      char sep[] = "\t";
      Serial.println(F("Parm\t\tCur\tDef"));
      Serial.print(F("calFactor"));
      Serial.print(sep);
      Serial.print(calFactor);
      Serial.print(sep);
      Serial.println(DEFAULT_CALFACTOR);
}

void serialUI(void) {
  char * tp = NULL;
  int tc = 0;  // token count this cmd
  char * tok[MAX_TOKENS];  // token ptr array
  int tn = 0;  // working token index

  switch (state) {
  case 0: // seaching for a newline (0x0a), ignore CR/0x0d
    if (Serial.available()) {  // new character ready
      char c = Serial.read();  // read it
      if ( c == 0x0d ) break;
      if ( c == 0x0a ) {  // NL signals end of command
        // log the command buffer
        Serial.print(F("cmd:"));
        Serial.println(cmd);
        state = 1;  // advance to parsing state
      } 
      else {
        cmd[nn++] = (char) c;  // add char to cmd string
        cmd[nn] = 0;
        if (logit) {
          // show partial cmd as it is received
          Serial.print(F(" cmd["));
          Serial.print(cmd);
          Serial.println(F("]"));
        }
      }
      if ( nn >= (signed)sizeof(cmd)) {
        state = 1;  // preclude cmd buffer overrun
      }
    }
    break;
  case 1:  // process the cmd string
    for (tn = 0;tn < MAX_TOKENS;tn++){
      tok[tn] = NULL;  // clear token ptr array
    }
    tc = 0;
    tp = NULL;
    tp = strtok(cmd," "); // start parsing out tokens
    while (tp) {
      if (logit ) {
        Serial.print(F("token "));
        Serial.print(tc);
        Serial.print(F(": "));
        Serial.println(tp);
      }
      tok[tc++] = tp;  // save token in token array
      tp = strtok(NULL," ");  // get next token
      if (tc >= MAX_TOKENS ) break;  // preclude tok ptr array overrun
    }
    // ============================================
    tn = 0; // index 1st token
    tp = tok[0];  // set tp to point to 1st token
    if ( strcmp(tp,"h") == 0 || 
          strcmp(tp,"?") == 0 ) {  // h (Help) cmd
      usage();
    }
    else if ( strcmp(tp,"b") == 0 ) {
      logit = 1;
      Serial.println(F("log on"));
    }
    else if ( strcmp(tp,"d") == 0 ) {
      dbug = !dbug;  // toggle dbug
      sprintf(buf,"dbug:%d\n",dbug);
      Serial.println(buf);
    }
    else if ( strcmp(tp,"p") == 0 ) {
      plotOn = !plotOn;  // toggle plotOn
      sprintf(buf,"plotOn:%d\n",plotOn);
      Serial.println(buf);
    }
    else if ( strcmp(tp,"e") == 0 ) {
      logit = 0;
      Serial.println(F("log off"));
    }
    else if ( strcmp(tp,"calFactor") == 0 ) {
      calFactor = DEFAULT_CALFACTOR; //   default 
      if (tok[1] != NULL ){
        calFactor = atof(tok[1]);  // user provided value
      } 
      Serial.print(F("calFactor set to  "));
      Serial.println(calFactor);
      showParms();
    }
    else if ( strcmp(tp,"show") == 0 ) {
      showParms();
    }
    else if ( strcmp(tp,"SAVE") == 0 ) {
      saveEEPROM();
    }
    else if ( strcmp(tp,"reset") == 0 ) {
      calFactor = DEFAULT_CALFACTOR;
      showParms();
    }
    else {
      Serial.print(F("bad cmd:"));
      Serial.println(tp);
    }
    clearBuf();
    nn = state = 0;  // ready for next cmd
    break;
  default:
    Serial.print(F("bad state:"));
    Serial.println(state);
    nn = state = 0;  // get next cmd
    break;
  }
}

//
//  ----------------  LCD UI ---------------------
//
// define some values used by the panel and buttons
int lcd_key     = 0;
int adc_key_in  = 0;

#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

int uiFSM(void);

struct buttonHits {
  int right;
  int left;
  int up;
  int down;
  int select;
} hits = {0, 0, 0, 0, 0};

// read_LCD_buttons() is lifted from the MPJA sample code.
//
// read the buttons - use the threshold list that works
// for the installed LCD panel
//
// my buttons when read are centered at these valies: 0, 144, 329, 504, 741
// we add approx 50 to those values and check to see if we are close
//
int read_LCD_buttons() {
  adc_key_in = analogRead(0);      // read the value from the sensor
  // 1st option for speed, i.e. most likely result
  if (adc_key_in > 1000) return btnNONE;
  // For V1.1 use this threshold
  /*
    if (adc_key_in < 50)   return btnRIGHT;
    if (adc_key_in < 250)  return btnUP;
    if (adc_key_in < 450)  return btnDOWN;
    if (adc_key_in < 650)  return btnLEFT;
    if (adc_key_in < 850)  return btnSELECT;
  */
  // For V1.0 boards, comment the other threshold and use the one below:
  if (adc_key_in < 50)   return btnRIGHT;
  if (adc_key_in < 195)  return btnUP;
  if (adc_key_in < 380)  return btnDOWN;
  if (adc_key_in < 555)  return btnLEFT;
  if (adc_key_in < 790)  return btnSELECT;
  return btnNONE;  // when all others fail, return this...
}

//
// LCD display management
//
int in_lcdUI = 0;

void clearLine(int lno){
  int n = 0;
  lcd.setCursor(0, lno);
  for (n=0;n<sizeof(buf);n++)
    lcd.print(" ");
  lcd.setCursor(0, lno);
}

void showLine(int lno, char *msg){
  clearLine(lno);
  lcd.print(msg);
}

//
// -------------------  LCD User Interface --------------
//
//   NOTE: use of delay() in parts of the lcd UI simplifies the UI 
//   display and does not impact the interrupt-driven operation of
//   the counter.
//
char *banner;

void showBanner(){
   if (uiState > 0 ) {
     clearLine(0);
     lcd.print(banner);
   }
}

void setStdBanner(){
   banner = "";
}

void updateHitsBanner(){ // config banner or hits - mostly for debugging
   clearBuf();
   sprintf(buf, "h:%d %d %d %d %d|%d", hits.right, hits.left,
        hits.up, hits.down, hits.select, uiState);
}

void setHitsBanner(){
  banner = buf;
}

void setConfigBanner(){
   banner = "Configuration UI";
}

void processHits(void){
  int rc;
  rc = uiFSM();
  // cleanup hits struct
  hits.up = hits.down = hits.left = hits.right = hits.select = 0;
  hitCount = 0;
  return rc;
}

//
//  button check - returns 0 when no button hit, otherwise, non-zero
//
int buttonCheck(void) {
  int rc = 0;

  lcd_key = read_LCD_buttons();  // read the buttons
  switch (lcd_key)
  {
    case btnRIGHT:
      {
        hits.right = 1;
	rc++;
        break;
      }
    case btnLEFT:
      {
        hits.left = 1;
	rc++;
        break;
      }
    case btnUP:
      {
        hits.up = 1;
	rc++;
        break;
      }
    case btnDOWN:
      {
        hits.down = 1;
	rc++;
        break;
      }
    case btnSELECT:
      {
        hits.select = 1;
	rc++;
        break;
      }
    case btnNONE:
      {
        break;
      }
  }
  updateHitsBanner(); // update banner
  return rc;
}


//
// lcd UI menu
//
//  move acts[] to PROGMEM if RAM gets tight
//

struct actions {
  char *prompt;
  int (*function)();
} acts[] = {
   "Show Settings", &a_show,
   "Set Defaults", &a_defaults,
   "Set calFactor", &a_calFactor,
   "SAVE settings", &a_SAVE,
   "Done, EXIT UI", &a_done,
   "Toggle dbug", &a_debug,
   "Toggle plot", &a_plot
};
#define asize sizeof(acts[0])
int aSubMax = sizeof(acts) / asize - 1;
int aSub = 0;

//
// lcd UI action functions
//
int a_show(){
    showLine(1,"calFactor ");
    lcd.print(calFactor);  // because sprintf() doesn't do floats
    delay(2000);
    uiState = 1;  // back to menu list
    return;
}

int a_defaults(){  // restore default settings
    calFactor = DEFAULT_CALFACTOR;
    showLine(1,"done"); delay(2000);
    uiState = 1;  // back to menu list
    return;
}

int a_calFactor(){
    showLine(1,"calFactor ");
    lcd.print(calFactor);
    // process UP DOWN LEFT & SELECT
    if (hits.up == 1 ) calFactor += 0.1;
    if (hits.down == 1 ) calFactor -= 0.1;
    if (hits.select == 1 ) uiState = 1;  // ACCEPT value, return to menus
    if (hits.left == 1 ) uiState = 3;  // go BACK - exit UI
    return;
}

int a_SAVE(){
    showLine(1,"saved"); delay(2000);
    saveEEPROM();
    uiState = 1;
    return;
}

int a_done(){
    showLine(1,"Bye"); delay(2000);
    uiState = 3;  // exit UI
    return;
}

int a_debug(){
    dbug == 0 ? dbug = 1 : dbug = 0;
    showLine(1,"dbug: ");
    lcd.print((int)dbug);
    delay(2000);
    uiState = 1;  
    return;
}

int a_plot(){
    plotOn == 0 ? plotOn = 1 : plotOn = 0;
    showLine(1,"plotOn: ");
    lcd.print((int)plotOn);
    delay(2000);
    uiState = 1;  
    return;
}

//
//  the lcd UI finite state machine
//
int uiFSM(void){
  int rc = 0;
  switch (uiState) {
  case 0:    // state 0 - looking for SELECT to enter lcd UI
    if (hits.select == 1) {
      hits.select = 0;
      uiState = 1;
      setConfigBanner();
      aSub = 0;  //set to start of menu list
    }
    break;
  case 1:   // State 1: lcd UI - looking for user to press buttons
    clearLine(1);
    lcd.print(acts[aSub].prompt);  // display menu item on lcd

    // process UP, DOWN, LEFT and SELECT buttons, ignore others

    if (hits.up == 1 ){  // scroll menu up
      hits.up = 0;
      if (aSub > 0) aSub--;
    }
    if (hits.down == 1) {
      hits.down = 0;    // scroll menu down
      if (aSub < aSubMax) aSub++;
    }
    // new menu will be displayed on next pass thru state 1
    //
    // LEFT button exits lcd UI
    //
    if (hits.left == 1 ) {
      uiState = 3;
    }
    //
    // SELECT select current menu item for further action
    //
    if (hits.select == 1){
      hits.select = 0;
      uiState = 2;
    }
    break;
  case 2:
    //  call the action function (to process more button hits)
    //  until it sets uiState back to 0, 1 or 3.
    acts[aSub].function();
    break;
  case 3:  // cleanup & UI exit
    if ( dbug > 0 ) setHitsBanner();
    else setStdBanner();
    uiState = 0;
    break;
  }
  return rc;
}

#ident "$Name: RHelec-01_dmk $ $Header: /projRCS/Arduino/newGeiger/newGeiger.ino,v 1.31.2.4 2017/08/20 16:30:30 dmk%eMach Exp $"
