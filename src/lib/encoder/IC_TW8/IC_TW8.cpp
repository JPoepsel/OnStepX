// ICHaus TW8 1Vss to SPI converter/interpolator
// jp  03.2023

#include "IC_TW8.h"

#if AXIS1_ENCODER == TW8_SPI || AXIS2_ENCODER == TW8_SPI || AXIS3_ENCODER == TW8_SPI || \
    AXIS4_ENCODER == TW8_SPI || AXIS5_ENCODER == TW8_SPI || AXIS6_ENCODER == TW8_SPI || \
    AXIS7_ENCODER == TW8_SPI || AXIS8_ENCODER == TW8_SPI || AXIS9_ENCODER == TW8_SPI

// designed according protocol description found in as38-H39e-b-an100.pdf

#include<SPI.h>

#define SPI_SETTINGS SPISettings(1000000,MSBFIRST,SPI_MODE0)


bool TW8WriteByte(uint8_t cs, word Addr, byte ByteToWrite) {
  digitalWrite(cs, 0);
  SPI.beginTransaction(SPI_SETTINGS);

  SPI.write(0x90);
  SPI.write16(Addr);
  SPI.write(ByteToWrite);

  digitalWrite(cs, 1);
  SPI.endTransaction();
  return true;
}

bool TW8WriteWord(uint8_t cs, word Addr, word WordToWrite) {
  if (Addr & 1) return false; // Addr must be even!
  digitalWrite(cs, 0);
  SPI.beginTransaction(SPI_SETTINGS);

  SPI.write(0x80);
  SPI.write16(Addr);
  SPI.write16(WordToWrite);

  digitalWrite(cs, 1);
  SPI.endTransaction();
  return true;
}

bool TW8ReadByte(uint8_t cs, word Addr, byte* ByteRead) {
  *ByteRead=0;
  digitalWrite(cs, 0);
  SPI.beginTransaction(SPI_SETTINGS);

  SPI.write(0xD0);
  SPI.write16(Addr);
  SPI.write(0);
  *ByteRead=SPI.transfer(0);

  digitalWrite(cs, 1);
  SPI.endTransaction();
  return true;
}

bool TW8ReadWord(uint8_t cs, word Addr, word* WordRead) {
  *WordRead=0;
  if (Addr & 1) return false; // Addr must be even!
  digitalWrite(cs, 0);
  SPI.beginTransaction(SPI_SETTINGS);

  SPI.write(0xC0);
  SPI.write16(Addr);
  SPI.write(0);
  *WordRead=SPI.transfer16(0);

  digitalWrite(cs, 1);
  SPI.endTransaction();
  return true;
}

signed long TW8PositionRead(uint8_t cs) {
  signed long PosVal=0;
  digitalWrite(cs, 0);
  SPI.beginTransaction(SPI_SETTINGS);

  SPI.write(0xE0);
  SPI.write(0);
  word LS=SPI.transfer16(0);
  word HS=SPI.transfer16(0);
  digitalWrite(cs, 1);
  SPI.endTransaction();
  PosVal=(signed long)((((unsigned long)HS)<<16)+LS);
  return PosVal;
}

void ftoDMSstr(float Ang, char* s) {
  bool aneg=Ang<0.0f; float aa=aneg?-Ang:Ang;  
  float deg=floor(aa); float min=floor((aa-deg)*60.0); float sec=(aa-deg-min/60.0)*3600.0; if (aneg) deg=-deg;
  sprintf(s,"%+04.0f %02.0f' %05.2f\"",deg,min,sec);
}

IcTw8::IcTw8(int16_t ChipSelectPin) {
  initialized = false;
  TheChipSelectPin=ChipSelectPin;
}


static word RChipID=0;
static word DChipID=0;
static word RInter=0;
static word DInter=0;


void IcTw8::init() {
  SPI.begin();
  pinMode(TheChipSelectPin, OUTPUT);
  digitalWrite(TheChipSelectPin, 1);
  
  #define CHIP_ID 0xC002
  #define MAIN_INTER 0x8046

  word chip_id; TW8ReadWord(TheChipSelectPin,CHIP_ID,&chip_id);
  // Debug out does not work since this function is called in th constructor of a global var, even before Serial is set up...
  // VF("MSG: TW8 encoder, CHIP_ID is ");  V(chip_id); VL("(if this is not 1032, you may be in trouble...)");
  TW8ReadWord(TheChipSelectPin,MAIN_INTER,&inter); 
  // VF("MSG: TW8 encoder, MAIN_INTER is ");  VL(inter); 
  // VF("MSG: You should set the AXIS?_STEPS_PER_DEGREE define accordingly (encoder_ticks_per_rev*this value/360)!"); 

  if (TheChipSelectPin==AXIS1_ENCODER_A_PIN) {
    RChipID=chip_id;
    RInter=inter;
  } else { 
    DChipID=chip_id;
    DInter=inter;
  }
 
  initialized=chip_id==0x408;    // only works, if correct chip id returned
}



// read encoder count
int32_t IcTw8::read() {
  if (!initialized) {
    if (TheChipSelectPin==AXIS1_ENCODER_A_PIN) {
      V("MSG: !!!!!!!!! ERROR !!!!!!!!!: RA not initialized, returned ID="); V(RChipID); V(", inter="); VL(RInter);  
    } else {
      V("MSG: !!!!!!!!! ERROR !!!!!!!!!: DEC not initialized, returned ID="); V(DChipID); V(", inter="); VL(DInter); 
    }
    return 0;
  }
  int32_t TW8Val=TW8PositionRead(TheChipSelectPin);
  // VF("MSG: TW8, Position is "); VL(TW8Val);
  
  if (TheChipSelectPin==AXIS1_ENCODER_A_PIN) {

    static unsigned long lasttime=0;
    static unsigned long lasttimeT=0;
    unsigned long now=micros();

    unsigned long nowT=millis();
    bool WantReset=((nowT/2000) > (lasttimeT/2000));
    lasttimeT=nowT;

    // if (WantReset) VL("MSG: ****** reset Max");

    static int32_t LastTW8Val=0;
    static int32_t MaxDTW8Val=0;
    static float   MaxDTW8ValV=0;

    int32_t dVal=TW8Val-LastTW8Val;
    if (dVal<0) dVal=-dVal;
    if (dVal>MaxDTW8Val) {
      // VF("MSG: ****** new max Ang Delta = "); VL(dVal); 
      MaxDTW8Val=dVal;
      if (WantReset) MaxDTW8Val=dVal;
    }
  
    float dt=(now-lasttime)/1000000.0;
    if (dt>0.0f) {
      #define RESM_RES 36000
      float Ang=((float)TW8Val)/(float)RESM_RES/(float)inter*360.0f;
      // char sa[100];  ftoDMSstr(Ang,sa); VF("MSG: Ra Ang= "); VL(sa);
      float LastAng=((float)LastTW8Val)/(float)RESM_RES/(float)inter*360.0f;
      float dValV=(Ang-LastAng)/dt; // deg/s
      if (dValV>MaxDTW8ValV) {
        if (dValV>4.0f) {
          char s[100];
          ftoDMSstr(dValV,s);
          VF("MSG: ****** RA new max Ang Velocity = "); VL(s); 
          VL("!!!!!!!!!!!!!!!!!! Ra: >4 Grad/s, das schafft der Motor gar nicht !!!!!!!!!!!!!!!");
        }
        MaxDTW8ValV=dValV;
      }
      if (WantReset) MaxDTW8ValV=dValV;
    }

    LastTW8Val=TW8Val;
    lasttime=now;
  }

  if (TheChipSelectPin==AXIS2_ENCODER_A_PIN) {

    static unsigned long lasttime=0;
    static unsigned long lasttimeT=0;
    unsigned long now=micros();

    unsigned long nowT=millis();
    bool WantReset=((nowT/2000) > (lasttimeT/2000));
    lasttimeT=nowT;

    // if (WantReset) VL("MSG: ****** reset Max");

    static int32_t LastTW8Val=0;
    static int32_t MaxDTW8Val=0;
    static float   MaxDTW8ValV=0;

    int32_t dVal=TW8Val-LastTW8Val;
    if (dVal<0) dVal=-dVal;
    if (dVal>MaxDTW8Val) {
      // VF("MSG: ****** new max Ang Delta = "); VL(dVal); 
      MaxDTW8Val=dVal;
      if (WantReset) MaxDTW8Val=dVal;
    }
  
    float dt=(now-lasttime)/1000000.0;
    if (dt>0.0f) {
      #define RESM_RES 36000
      float Ang=((float)TW8Val)/(float)RESM_RES/(float)inter*360.0f;
      // char sa[100];  ftoDMSstr(Ang,sa); VF("MSG: DEC Ang= "); VL(sa);
      float LastAng=((float)LastTW8Val)/(float)RESM_RES/(float)inter*360.0f;
      float dValV=(Ang-LastAng)/dt; // deg/s
      if (dValV>MaxDTW8ValV) {
        if (dValV>4.0f) {
          char s[100];
          ftoDMSstr(dValV,s);
          VF("MSG: ****** DEC new max Ang Velocity = "); VL(s); 
          VL("!!!!!!!!!!!!!!!!!! DEC: >4 Grad/s, das schafft der Motor gar nicht !!!!!!!!!!!!!!!");
        }
        MaxDTW8ValV=dValV;
      }
      if (WantReset) MaxDTW8ValV=dValV;
    }

    LastTW8Val=TW8Val;
    lasttime=now;
  }


  

  // VF("MSG: TW8 read, corrected Position is "); VL(TW8Val-offsetPosition);
  return TW8Val-offsetPosition; 
}

// write encoder count
void IcTw8::write(int32_t count) {
  if (!initialized) return;
  int32_t TW8Val=TW8PositionRead(TheChipSelectPin);
  offsetPosition=TW8Val-count;  
  // Debug out does not work since this function is called in th constructor of a global var, even before Serial is set up...
  // VF("MSG: TW8, position set to "); VL(offsetPosition);
}
 
#endif
