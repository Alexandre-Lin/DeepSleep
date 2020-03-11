#include <Arduino.h>
#include <Sodaq_RN2483.h>
#include <Sodaq_wdt.h>
#include <SPI.h>
#include <RN487x_BLE.h>
#include <RTCTimer.h>
#include <RTCZero.h>
#define CONSOLE_STREAM  SERIAL_PORT_MONITOR
#define debugSerial SerialUSB // debug on USB
#define bleSerial Serial1// Bluetooth module Serial
#define loraSerial Serial2 // LoRa module Serial
#define LORA_BAUD 57600 #define DEBUG_BAUD 57600
#define NIBBLE_TO_HEX_CHAR( i ) (( i  <= 9) ? ( ’ 0 ’ + i ) : ( ’A ’  10 +  i ))
#define HIGH_NIBBLE( i )  (( i  >>  4)  & 0x0F )
#define LOW_NIBBLE( i ) ( i & 0x0F )
RTCZero rtc ;
RTCTimer  timer ;

volatile bool minuteFlag ;
//
// setup your c o n s t a n t s here ! !
//
const uint8_t records_to_send = 2 ; // s e t t h i s to change the amount o f r e c o r d s to send
const uint8_t record_every_x_minutes = 1 ; // s e t t h i s to the d e s i r e d i n t e r v a l i n minu tes
const uint8_t spreading_factor = 7 ; // s e t t h i s to the d e s i r e d LoRa sp r e ading f a c t o r
// *****************************************************************************************
// LoRa communication se tup !
// true : use OTAA
// false : use ABP
bool OTAA = false ;
// ABP setup ( d ev ic e add ress )
// USE <OUR OWN KE<S !
const uint8_t devAddr[4] =
{
  0x00 , 0x00 , 0x00 , 0x00
} ;
// applicationsessionkey
// USE <OUR OWN KE<S !
const uint8_t appSKey[16] =
{
  0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00
} ;
// network s e s s i o n key
// USE <OUR OWN KE<S !
const uint8_t nwkSKey[16] =
{
  0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00
} ;
// OTAA ( d ev ic e EUI )
// With using the GetHWEUI ( ) f u n c t i o n the HWEUI w i l l be used
static uint8_t DevEUI[8]
{
  0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00
} ;
const uint8_t AppEUI[8] =
{
  0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00
} ;
const uint8_t AppKey[16] =
{
  0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00
} ;
// *****************************************************************************************
// se tup
bool LoRa_sleeps = false ;
uint8_t message [records_to_send * 2] ;
void setup()
{
  sodaq_wdt_enable (WDT_PERIOD_8X ) ; // Enable the wdt a t maximum i n t e r v a l
  sodaq_wdt_reset() ;
  sodaq_wdt_safe_delay(5000) ;
  pinMode (TEMP_SENSOR , INPUT ) ;
  initRtc();


  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk ; // s e t s SAMD s l e e p mode to deep s l e e p
  // ne tworks
  loraSerial.begin(LoRaBee.getDefaultBaudRate());
  LoRaBee.init(loraSerial, LORA_RESET ) ;
  setupLoRa(); // s e t the gea rs i n motion
  initRtcTimer(); // t im e r i n t e r r u p t > 1 minute i n t e r v a l
  sodaq_wdt_reset();
  SerialUSB.flush();
  SerialUSB.end();
  USBDevice.detach() ;
  USB->DEVICE.CTRLA.reg &= ~USB_CTRLA_ENABLE ; // D i s a b l e USB
  sleep_setup();
}
// *****************************************************************************************
// loop
// a r r ay f o r tempe ra tu re we wish to send ;
// here we ’ re t a k i n g a 6ዅv alu e (12 by t es ) a r r ay to send a t once
// in t end ed to measure eve ry n*60 seconds , send eve ry x messages .
const uint8_t measurements_to_send = 6 ;
int temperature_array[measurements_to_send ] ;
uint8_t list_iter = 0; // v a r i a b l e to keep t r a c k o f a r r ay ind ex and when to send
void loop ( )
{
  if (sodaq_wdt_flag) {
    sodaq_wdt_reset();
    sodaq_wdt_flag = false ;
  }
  if ( minuteFlag ) {
    timer.update();
    minuteFlag = false ;
  }
  systemSleep();
}
// *****************************************************************************************
// Sleep commands
void BT_powerdown ( )
{
  rn487xBle.hwInit();
  bleSerial.begin(rn487xBle.getDefaultBaudRate()) ;
  rn487xBle.initBleStream(&bleSerial) ;
  rn487xBle.enterCommandMode();
  rn487xBle.dormantMode();
  bleSerial.end() ;
}
void sleep_setup()
{
  // s e t FLASH to deep s l e e p & r e s e t SPI p i n s f o r min . energy consumption
  DFlashUltraDeepSleep();
  sleep_LoRa() ;
  // RN4871 BT / BLE module s l e e p
  BT_powerdown() ;
}
void systemSleep()// Sinc e o nly LoRa and MCU awake , o nly s e t those to s l e e p
{
  if (!LoRa_sleeps) // Ski p i f LoRa i s a sl e e p
  {
    sleep_LoRa() ;
  }
  noInterrupts();
  if (!(sodaq_wdt_flag || minuteFlag)) {
    interrupts();
    debugSerial.println("Sleeping");
    __WFI(); // SAMD s l e e p
  }
  interrupts();
}
void sleep_LoRa() {
  loraSerial.flush();
  LoRaBee.sleep();
  LoRa_sleeps = true ;
  sodaq_wdt_safe_delay(5); // wi t h o u t t h i s , i t doesn ’ t s l e e p . . don ’ t know why
}
void wake_LoRa()
{
  LoRa_sleeps = false ;
  LoRaBee.wakeUp();
}
// *****************************************************************************************
// SST25PF040C F l a s h f u n c t i o n s ( SPI )
void DFlashUltraDeepSleep()
{
  static const uint8_t SS_DFLASH = 44 ;
  // SPI i n i t i a l i s a t i o n
  SPI.begin();
  // I n i t i a l i s e the CS p i n f o r the da ta f l a s h
  pinMode(SS_DFLASH, OUTPUT);
  digitalWrite(SS_DFLASH, HIGH);
  transmit(0xB9);
  SPI.end();
  // Rese ts the p i n s used
  resetSPIPins();
}
void transmit(uint8_t val)
{
  SPISettings settings;
  digitalWrite(SS_DFLASH, LOW) ;
  SPI.beginTransaction(settings);
  SPI.transfer(val);
  SPI.endTransaction();
  digitalWrite(SS_DFLASH, HIGH);
  delayMicroseconds(1000);
}
void resetSPIPins()
{
  resetPin(MISO);
  resetPin(MOSI);
  resetPin(SCK);
  resetPin(SS_DFLASH);
}
void resetPin(uint8_t pin)
{
  PORT->Group[g_APinDescription[pin].ulPort].
  PINCFG[g_APinDescription[pin].ulPin].reg = (uint8_t)(0);
  PORT->Group[g_APinDescription[pin].ulPort].
  DIRCLR.reg = (uint32_t)(1 << g_APinDescription[pin].ulPin);
  PORT->Group[g_APinDescription[pin].ulPort].
  OUTCLR.reg = (uint32_t)(1 << g_APinDescription[pin].ulPin);
}
// *****************************************************************************************
// RN2483 / RN2903 LoRa commands
// f o r RN2903 : uncomment the se t FsbCh annels l i n e .
void setupLoRa()
{
  getHWEUI();
  if (!OTAA) {
    setupLoRaABP(); // ABP se tup
  } else {
    setupLoRaOTAA(); // OTAA se tup
  }
  // Uncomment the f o l l o w i n g l i n e to f o r the RN2903 wi th the A c t i l i t y Network .
  // Fo r OTAA , update the DEFAULT_FSB i n the l i b r a r y
  // LoRaBee.setFsbChannels(1);
  LoRaBee.setSpreadingFactor(spreading_factor);
}
void setupLoRaABP() {
  if (LoRaBee.initABP(loraSerial, devAddr, appSKey, nwkSKey, true))
  {
    debugSerial.println("Communication␣t ␣LoRaBEE␣successful.");
  }
  else
  {
    debugSerial.println("Communication␣to␣LoRaBEE␣failed!");
  }
}
void setupLoRaOTAA() {
  if (LoRaBee.initOTA(loraSerial, DevEUI , AppEUI , AppKey , true ) )
  {
    debugSerial.println("Network␣connection␣successful.");
  }
  else
  {
    debugSerial.println("Network␣connection␣failed!");
  }
}
static void getHWEUI() // g e ts + s t o r e s HWEUI
{
  uint8_t len = LoRaBee.getHWEUI(DevEUI , sizeof ( DevEUI ) ) ;
}
void send_message(uint8_t* val, size_t val_size) {
  wake_LoRa ( ) ;
  // s i n c e the debug p o r t i s no t enabled i n t h i s example , the debug message i s no t p r i n t e d
  switch (LoRaBee.send(1, (uint8_t*) val , val_size) ) // send ( po r t , payload , l e n g t h )
  {
    case NoError :
      debugSerial.println("Successful");
      break;
    case NoResponse :
      debugSerial.println("No␣Response" ) ;
      break;
    case Timeout :
      debugSerial.println(" Timeout.␣starting␣20 second␣delay " ) ;
      delay(20000);
      break;
    case PayloadSizeError:
      debugSerial.println("Payload␣too␣large !") ;
      break;
    case InternalError:
      debugSerial.println("Internal␣Error; resetting␣module");
      setupLoRa();
      break;
    case Busy :
      debugSerial.println("LoRa␣module␣active");
      delay(10000);
      break;
    case NetworkFatalError:
      debugSerial.println( "Network␣connection␣error; ␣resetting␣module");
      setupLoRa();
      break;
    case NotConnected :
      debugSerial.println(" Not␣connec ted ; ␣resetting␣module " ) ;
      setupLoRa();
      break;
    case NoAcknowledgment :
      debugSerial.println( "No␣acknowledgement␣received");
      break;
    default :
      break;
  }
  sleep_LoRa();
}
// *****************************************************************************************
// Temperature Sensor f u n c t i o n s
// ou tpu t : 2ዅby te i n t c e n t i ዅc e l c i u s ( d i v i d e by 100 to type f l o a t f o r c o r r e c t v alu e ) .
void getTemperature() {
  int int_temp ;
  uint8_t negativeFlag;
  float mVolts = (float)analogRead(TEMP_SENSOR)*3300.0/1024.0 ;
  float temp = (mVolts - 500.0 ) / 10.0 ;
  temp *= 100;
  if (temp < 0) negativeFlag = 0x80 ;
  else negativeFlag = 0x00 ;
  int_temp = abs((int)temp ) ;
  message[list_iter * 2] = (int_temp >> 8) | negativeFlag;
  message[list_iter * 2 + 1] = int_temp & 0xFF ;
}
// *****************************************************************************************
// RTC f u n c t i o n s
// I n i t i a l i z e s the RTC
void initRtc()
{
  rtc.begin();
  // Schedule the wakeup i n t e r r u p t f o r eve ry minute
  // Alarm is triggered1 cycle after match
  rtc.setAlarmSeconds(59);
  rtc.enableAlarm(RTCZero::MATCH_SS); // alarm eve ry minute
  // A t t ac h h a n dl e r
  rtc.attachInterrupt(rtcAlarmHandler);
  // T h i s s e t s i t to 2000-01-01
  rtc.setEpoch(0);
}
// Runs eve ry minute by the r t c alarm .
void rtcAlarmHandler()
{
  minuteFlag = true ;
}
// I n i t i a l i z e s the RTC Timer
void initRtcTimer()
{
  debugSerial.print("init␣rtc␣timer");
  timer.setNowCallback(getNow); // s e t how to ge t the c u r r e n t time
  timer.allowMultipleEvents();
  resetRtcTimerEvents();
}
void resetRtcTimerEvents()
{

  // Schedule the d e f a u l t f i x even t ( i f a p p l i c a b l e )
  timer.every(record_every_x_minutes * 60 , measureTemperature);
  debugSerial.println("event_set");
}
// Re tu rns c u r r e n t da te time i n seconds s i n c e epoch
uint32_t getNow()
{
  return rtc.getEpoch();
}
// D e f a u l t even t parame ter
void measureTemperature ( uint32_t now )
{
  getTemperature();
  list_iter++;
  if (!(list_iter < records_to_send)) {
    send_message((uint8_t*) &message, sizeof ( message));
    list_iter = 0;
  }
}
