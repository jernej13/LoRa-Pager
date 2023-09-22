/* MATURITETNA NALOGA
ELEKTROTEHNIŠKO-RAČUNALNIŠKA STROKOVNA ŠOLA IN GIMNAZIJA LJUBLJANA

Mentor: Janez Omahen, prof.
Avtor: Jernej Leskovec, E 4. B
Ljubljana, marec 2023

>>> LORA POZIVNIK <<<

Programska koda je napisana za ploščico LilyGO TTGO T-Beam. Na headerje 15, 35, 32
so vezane tipke, na 25 je vezan piezo zvočnik in na 22 in 21 je vezan OLED display.
Dodana je še 18650 baterija kapacitete okoli 2700 mAh. Koda ima več napak in se
večkrat zatakne, nevem še zakaj in kje, tako da je potrebno odpravljanje hroščev .

Knjižnice v celoti pripadajo njihovima avtorjem in se ne zavezam, da so moje
avtorsko delo. Koda spodaje je pa v celoti moje avtorsko delo, kodo lahko uporabljate
v kakršne koli namene in nisem odgovoren za poškodbe vezji itd.

(Koda je mišlena kot priloga poročilu maturitetnega izdelka.)

*/



/*KNJIŽNICE    
*/
#include "axp20x.h"
#include <LoRa.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <U8g2lib.h>

/*SISTEMSKE DEKLARACIJE
    Deklaracije uporabljene za delovanje sistema.
*/

/* Maska za določanje pinov na ESPju ki ga bodo budile iz spanja
maska binarno je pozitivna samo na pinih, ki bodo zbudile ESP iz spanja
desna je 0; 1000 0010 0000 0000 0000 0000 0000 0000
*/

#define BUTTON_PIN_BITMASK 0x104000000

// Pini ki komunicirajo iz LoRa čipom
#define ss 18
#define rst 23
#define dio0 26

// LoRa nastavitve
uint8_t txPower = 2;
uint8_t spreadingFactor = 12;
long Bandwidth = 62.5E3;
uint8_t codingRateDenominator = 8;
long preambleLength = 10;

// Naslov ekrana
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);    //prekliks za OLED

// Klasa in pini za komunikacijo iz čipom ki upravlja napajanje
AXP20X_Class axp;
const uint8_t i2c_sda = 21;
const uint8_t i2c_scl = 22;
const uint8_t slave_address = AXP192_SLAVE_ADDRESS;     //nastavimo i2c naslov

// Pini in klasa za upravljanja iz GNSSom
SoftwareSerial mySerial(34, 12);
SFE_UBLOX_GNSS myGNSS;                  //nastavimo prekliks za GNSS

/*GLOBALNE DEKLARACIJE IN SPREMENLJIVKE
    Obstajajo za ključno, lažje delovanje programa in simplifikacijo.
*/
// LoRa sprejem:
String MSGstring = "0";
uint32_t MSG = 0;
int16_t RSSI = 0;
uint8_t MSGlenght = 0;

// Za interrupte:
bool counter = 0;
bool onRcounter = 0;

// Števci za izklapljanje:
uint8_t whilecounter = 0;
uint16_t timeouttime = 30; // s
unsigned long previousTime = 0;
unsigned long previousTime1 = 0;
unsigned long previousTime2 = 0;

// Inputi in outputi espja
const uint8_t tipka1pin = 15;
const uint8_t tipka3pin = 33;
const uint8_t piezo = 25;

// Spremenljivke za GNSS
long gnssLAT = 0;
long gnssLON = 0;
long gnssALT = 0;
uint8_t SIV = 0;
int32_t ACC = 0;
int32_t Speed = 0;
bool GNSSisOFF = 0;
uint8_t GNSSsleepSETTIING = 0;

// Spremenljivke za napajanje
float batteryPercantage = 0;
float batteryMV = 0;

// Globalne spremenljivke, ampak uporabljene za oled
// Stringi za tekst v menuju
String TITLE;
String Screen1Row1;
String Screen1Row2;
String Screen1Row3;
String Screen1Row4;
String Screen2Row1;
String Screen2Row2;
String Screen2Row3;
String Screen2Row4;
String Screen2Row5;
String Screen2Row6;
String Screen3Row1;
String Screen3Row2;
String Screen3Row3;
String Screen3Row4;
String Screen3Row5;
String Screen3Row6;

// Dolžina zgornjih stringow
uint8_t TITLElenght;
uint8_t Screen1Row1lenght;
uint8_t Screen1Row2lenght;
uint8_t Screen1Row3lenght;
uint8_t Screen1Row4lenght;
uint8_t Screen2Row1lenght;
uint8_t Screen2Row2lenght;
uint8_t Screen2Row3lenght;
uint8_t Screen2Row4lenght;
uint8_t Screen2Row5lenght;
uint8_t Screen2Row6lenght;
uint8_t Screen3Row1lenght;
uint8_t Screen3Row2lenght;
uint8_t Screen3Row3lenght;
uint8_t Screen3Row4lenght;
uint8_t Screen3Row5lenght;
uint8_t Screen3Row6lenght;

// za pomikanje nazaj po menuju
bool m1 = 0;
bool m2 = 0;
bool m3 = 0;

// XBM Bitmap slikice
#define satlogo_h 14
#define satlogo_w 14
static const unsigned char satlogo_bits[] U8X8_PROGMEM = {
    0x00,
    0x18,
    0x00,
    0x3C,
    0x00,
    0x3E,
    0x00,
    0x1F,
    0x60,
    0x0F,
    0xF0,
    0x06,
    0xF0,
    0x01,
    0xE0,
    0x03,
    0xD8,
    0x01,
    0xBC,
    0x2A,
    0x3E,
    0x28,
    0x1F,
    0x16,
    0x0F,
    0x18,
    0x06,
    0x06,
};
#define satlogoz_h 14
#define satlogoz_w 14
static const unsigned char satlogoz_bits[] U8X8_PROGMEM = {
    0x0F,
    0x18,
    0x04,
    0x3C,
    0x02,
    0x3E,
    0x0F,
    0x1F,
    0x60,
    0x0F,
    0xF0,
    0x06,
    0xF0,
    0x01,
    0xE0,
    0x03,
    0xD8,
    0x01,
    0xBC,
    0x2A,
    0x3E,
    0x28,
    0x1F,
    0x16,
    0x0F,
    0x18,
    0x06,
    0x06,
};

// Poziv in dolžina poziva
String page;
uint8_t pagelen;

// Spremenljivke za flipflope in ostalo
int16_t a;
uint16_t selecta;
uint16_t page_select;
int acap;
bool b;
bool c;
bool d;
bool e;
bool f;
bool g;
bool h;
bool fdecl;

/*PROGRAM
    Program je sestavljen iz 24 funkciji. Na začetku sta deklarirane dve perkinitvene
    funkcije ali interrupti, naslednja je funkcija setup() in nakoncu kode je funkcija
    glavne zanke loop(). Vmes so funkcije, ki se izvajajo v glavni zanki.
*/

// Interupti
// Interupt za sprejemanje
void onReceive(int packetSize) {
    MSGstring = LoRa.readString();      //Preberemo sprejet paket
    RSSI = LoRa.packetRssi();           //Preberemo moč paketa
    onRcounter = 1;                     //Nastavimo pomnilno celico na 1
    MSGlenght = MSGstring.length();     //Določimo dolžino niza, ki smo ga sprejeli
    MSG = MSGstring.toInt();            //Niz pretvorimo v naravno število
}

// Interupt za zbujanje in oddajanje
void MENUInterrupt() {
    counter = 1;                        //pomnilno celico nastavimo na 1
}

// Začetna koda, ki initalizira program
void setup() {

    // Initilizacija komunikacije iz UBLOX čipom
    mySerial.begin(38400);              //GNSS serial je nazačetku 9600 in ga je potrebno nastaviti z funkcijo prefliks.setSerialRate(38400);
    myGNSS.begin(mySerial);

    // Initilizacija komunikacije iz čipom za napajanje
    Wire.begin(i2c_sda, i2c_scl);
    axp.begin(Wire, slave_address);
    axp.setChgLEDMode(AXP20X_LED_OFF);      //izključimo ledico, ki signalizira polnjenje


    // Initilizacija oled ekrana
    u8g2.begin();
    u8g2.setFont(u8g2_font_7x13_mf);
    u8g2.setFontMode(0);

    // Initilizacija LoRa čipa
    LoRa.setPins(ss, rst, dio0);

    // Nastavitev IO pinov
    pinMode(tipka1pin, INPUT_PULLDOWN);
    pinMode(tipka3pin, INPUT_PULLDOWN);
    pinMode(piezo, OUTPUT);

    // Koda za prikaz napake v komunikaciji iz LoRo
    if (!LoRa.begin(433E6)) {
        u8g2.clearBuffer();
        u8g2.drawStr(0, 10, "Starting LoRa failed!");
        u8g2.sendBuffer();
        while (1)
            ;
    }

    // LoRa nastavitve
    LoRa.setTxPower(txPower);
    LoRa.setSpreadingFactor(spreadingFactor);
    LoRa.setSignalBandwidth(Bandwidth);
    LoRa.setCodingRate4(codingRateDenominator);
    LoRa.setPreambleLength(preambleLength);

    // Bujenje ESPja in nastavitev interupta
    esp_sleep_enable_ext1_wakeup(BUTTON_PIN_BITMASK, ESP_EXT1_WAKEUP_ANY_HIGH);         //Koda, ki nastavi pine, za zbujanje ESPja
    attachInterrupt(digitalPinToInterrupt(32), MENUInterrupt, RISING);                  //Interrupt za meni

    // LoRa postavitev na sprejem
    LoRa.onReceive(onReceive);          //Interrupt za sprejemanje
    LoRa.receive();

    // Nastavitev GNSSa
    myGNSS.setUART1Output(COM_TYPE_UBX);        //Limitiramo Ublox čipu, da nam podaja podatke samo v obliki UBX
    myGNSS.saveConfiguration();

    // Nastavitev hitrosti procesorja
    setCpuFrequencyMhz(240);

    // Postavitev GNSSa v spanje
    GNSSisOFF = 1;
    if (myGNSS.powerOffWithInterrupt(0, VAL_RXM_PMREQ_WAKEUPSOURCE_UARTRX) == 0) {      //Preverimo, če je od prej Ublox čip še v spanju
        GNSSisOFF = 0;
    }

    // Lil stuff to end idk
    tone(piezo, 4098, 1000);
    u8g2.drawStr(25, 25, "Initialized");
    u8g2.sendBuffer();
    delay(1000);
}
/*FUNKCIJA, KI ŠTEJE, GLEDE NA MILLISE
    Funkcija nam poda novo vrednost pristevnaca, doda jo 1 vsah interval (ms)
*/
uint16_t millisTimeout(uint16_t interval, uint16_t pristevanec) {

    unsigned long currentMillis = millis();
    if (currentMillis - previousTime >= interval) {
        pristevanec++;
        u8g2.sendBuffer();          // Pride prav v vecini primerow
        previousTime = currentMillis;
    }
    return pristevanec;
}
uint16_t millisTimeout1(uint16_t interval, uint16_t pristevanec) {

    unsigned long currentMillis = millis();
    if (currentMillis - previousTime1 >= interval) {
        pristevanec++;
        u8g2.sendBuffer();
        previousTime1 = currentMillis;
    }
    return pristevanec;
}

/*Funkcija za prekinitev zank, če je negdo pritisnil tipko ali v primeru
 sprejetja paketa.
 0 za counter (MENU), 1 za onRcounter(sprejem)
*/
void check(bool interupt_stevec) {
    if (interupt_stevec == 0) {
        if (counter == 1) {
            m1 = 1;
            m2 = 1;
            m3 = 1;
            whilecounter = timeouttime;
        }
    }
    if (interupt_stevec == 1) {
        if (onRcounter == 1) {
            whilecounter = timeouttime;
            m1 = 1;
            m2 = 1;
            m3 = 1;
        }
    }
}

/*Funkcija za poenostavljanje spremenljivk
 x >> 0 za poenostavitev števcev, 1 za poenastavitev spremenljivk
 Delay >> Koliko delaya dodamo za debounce*/
void reset(bool x, uint16_t Delay) {

    delay(Delay);
    if (x == 1) {
        TITLE = "";         // Zaradi določanja dolžine in ustavljanja v level funkciji
        Screen1Row1 = "";
        Screen1Row2 = "";
        Screen1Row3 = "";
        Screen1Row4 = "";
        Screen2Row1 = "";
        Screen2Row2 = "";
        Screen2Row3 = "";
        Screen2Row4 = "";
        Screen2Row5 = "";
        Screen2Row6 = "";
        Screen3Row1 = "";
        Screen3Row2 = "";
        Screen3Row3 = "";
        Screen3Row4 = "";
        Screen3Row5 = "";
        Screen3Row6 = "";

        a = 0;
        b = 0;
        c = 0;
        d = 0;
        e = 0;
        f = 0;
        g = 0;
        h = 0;
        selecta = 1;
        page_select = 0;
    }
    if (x == 0) {
        counter = 0;
        onRcounter = 0;
        whilecounter = 0;
    }
}

/*GNSS parser je funkcija, ki iz pomočjo zaporedji sestevi besedo iz podatkov in jo poslje,
    ali sprejme in raztavi na uporabne elemente.
    Recieved >> 1 če raztavljamo oz smo sprejeli GNSS, 0 če pošiljamo GNSS
    latitude >> damo podatek iz gnssja
    longitude >> damo podatek iz gnssja
    altitude >> damo podatek iz gnssja
    returntype >> kateri podatek bomo dobivali, če sprejemamo 1>latitude 2>longitude 3>altitude
    Za uspešno sprejet funkcija vrne 10, če karkoli gre narobe pa 0.
*/
float LoRa_GNSS_Parser(bool recieved, uint32_t latitude, uint32_t longitude, uint32_t altitude, uint8_t returntype) {
    char buffer[32];
    char recievedbuffer[32];

    int32_t Slat = 0;
    int32_t Slon = 0;
    int32_t Salt = 0;

    float gpsfloatLAT;
    float gpsfloatLON;
    float gpsfloatALT;

    String StringToSend = "";

    if (recieved == 0) {
        sprintf(buffer, "%d %d %d", latitude, longitude, altitude);     //Zapakiramo v zbirko
        StringToSend = String(buffer);                      //Spremenimo array v niz
        LoRa.beginPacket();
        LoRa.print(StringToSend);
        LoRa.endPacket(true);           //Pošljemo paket brez zahteve za stabilno povezavo
        delay(4000);
        LoRa.receive();
        return 10;
    }
    if ((recieved == 1) && (MSGlenght > 5)) {
        MSGstring.toCharArray(recievedbuffer, 32);              //Sprejeti niz pretvorimo v zbirko
        sscanf(recievedbuffer, "%d %d %d", &Slat, &Slon, &Salt);        //Raztavimo zbirko

        gpsfloatLAT = Slat / 10000000.;
        gpsfloatLON = Slon / 10000000.;
        gpsfloatALT = Salt / 1000.;

        if (returntype == 1) {
            return gpsfloatLAT;
        }
        if (returntype == 2) {
            return gpsfloatLON;
        }
        if (returntype == 3) {
            return gpsfloatALT;
        }
    }
    return 0;
}

/*Funcije za display in pisanje na display:
Nize, ki jih hočemo prikazati na OLEDU navajamo kot Screen1Row1 = "";
Priporočeno je pred to funkcijo klicati tudi reset funkcijo.
*/
void astop(uint8_t x) {     //Za določanje gdaj se ustavi kazalec
    if (x > 1) {
        acap += 10;
    }
}

void level() {
    TITLElenght = TITLE.length();           //Določimo dolžine vseh nizov
    Screen1Row1lenght = Screen1Row1.length();
    Screen1Row2lenght = Screen1Row2.length();
    Screen1Row3lenght = Screen1Row3.length();
    Screen1Row4lenght = Screen1Row4.length();
    Screen2Row1lenght = Screen2Row1.length();
    Screen2Row2lenght = Screen2Row2.length();
    Screen2Row3lenght = Screen2Row3.length();
    Screen2Row4lenght = Screen2Row4.length();
    Screen2Row5lenght = Screen2Row5.length();
    Screen2Row6lenght = Screen2Row6.length();
    Screen3Row1lenght = Screen3Row1.length();
    Screen3Row2lenght = Screen3Row2.length();
    Screen3Row3lenght = Screen3Row3.length();
    Screen3Row4lenght = Screen3Row4.length();
    Screen3Row5lenght = Screen3Row5.length();
    Screen3Row6lenght = Screen3Row6.length();

    astop(Screen1Row1lenght);       //Določimo gdaj se ustavi
    astop(Screen1Row2lenght);
    astop(Screen1Row3lenght);
    astop(Screen1Row4lenght);
    astop(Screen2Row1lenght);
    astop(Screen2Row2lenght);
    astop(Screen2Row3lenght);
    astop(Screen2Row4lenght);
    astop(Screen2Row5lenght);
    astop(Screen2Row6lenght);
    astop(Screen3Row1lenght);
    astop(Screen3Row2lenght);
    astop(Screen3Row3lenght);
    astop(Screen3Row4lenght);
    astop(Screen3Row5lenght);
    astop(Screen3Row6lenght);

    if (a < 0) {
        u8g2.clearBuffer();
        a = acap - 10;
        selecta = (acap - 10) / 10 + 1;
    }

    if ((a < 35) && (d == 1)) {
        u8g2.clearBuffer();
    }

    if ((a < 95) && (e == 1)) {
        u8g2.clearBuffer();
    }

    if ((a < 35) || (Screen2Row1lenght < 1)) {  //Prva stran

        if (a > (acap - 10)) {
            a = 0;
            selecta = 1;
            f = 1;
            u8g2.clearBuffer();
        }

        u8g2.setFont(u8g2_font_7x13_mf);            
        u8g2.setCursor((128 - (TITLElenght)*7) / 2, 12);        //Izpišemo na ekran
        u8g2.print(TITLE);
        u8g2.setFont(u8g2_font_6x10_mf);
        u8g2.setCursor((128 - (Screen1Row1lenght)*6) / 2, 30);
        u8g2.print(Screen1Row1);
        u8g2.setCursor((128 - (Screen1Row2lenght)*6) / 2, 40);
        u8g2.print(Screen1Row2);
        u8g2.setCursor((128 - (Screen1Row3lenght)*6) / 2, 50);
        u8g2.print(Screen1Row3);
        u8g2.setCursor((128 - (Screen1Row4lenght)*6) / 2, 60);
        u8g2.print(Screen1Row4);

        if (a > 5) {
            u8g2.drawStr(0, 20 + a, " ");
            u8g2.drawStr(120, 20 + a, " ");
        }

        u8g2.drawLine(0, 16, 128, 16);

        if (((a < 25) && (Screen2Row1lenght > 0)) || (Screen2Row1lenght == 0)) {    //Kazalci
            u8g2.drawStr(0, 30 + a, ">");
            u8g2.drawStr(120, 30 + a, "<");
        }
        if ((Screen2Row1lenght > 0) && (a > 25)) {
            u8g2.drawStr(0, 30 + a, "v");
            u8g2.drawStr(120, 30 + a, "v");
        }

        u8g2.drawStr(0, 40 + a, " ");
        u8g2.drawStr(120, 40 + a, " ");

        d = 0;
    }

    if ((digitalRead(tipka3pin) == 1) && (b == 0)) {    //Preberemo ali je tipka pritisnjena in če je izvršimo premik
        a += 10;
        whilecounter = 0;
        b = 1;
        selecta++;
    }
    if (digitalRead(tipka3pin) == 0) {
        b = 0;
    }
    if ((digitalRead(tipka1pin) == 1) && (c == 0)) {
        a -= 10;
        whilecounter = 0;
        c = 1;
        selecta--;
    }
    if (digitalRead(tipka1pin) == 0) {
        c = 0;
    }

    if ((a > 35) && (d == 0) && (Screen2Row1lenght > 1)) {
        u8g2.clearBuffer();
    }

    if ((a > 95) && (e == 0)) {
        u8g2.clearBuffer();
    }

    if (f == 1) {
        u8g2.drawStr(0, 60, " ");
        u8g2.drawStr(120, 60, " ");
        f = 0;
    }

    if (((a > 35) && (Screen2Row1lenght > 1) && (a < 95)) || ((Screen3Row1lenght < 1) && (d == 1))) {       //Druga stran

        if (a > (acap - 10)) {
            a = 0;
            selecta = 1;
            f = 1;
        }

        u8g2.setFont(u8g2_font_6x10_mf);
        u8g2.setCursor((128 - (Screen2Row1lenght)*6) / 2, 10);      //Izpišemo na ekran
        u8g2.print(Screen2Row1);
        u8g2.setCursor((128 - (Screen2Row2lenght)*6) / 2, 20);
        u8g2.print(Screen2Row2);
        u8g2.setCursor((128 - (Screen2Row3lenght)*6) / 2, 30);
        u8g2.print(Screen2Row3);
        u8g2.setCursor((128 - (Screen2Row4lenght)*6) / 2, 40);
        u8g2.print(Screen2Row4);
        u8g2.setCursor((128 - (Screen2Row5lenght)*6) / 2, 50);
        u8g2.print(Screen2Row5);
        u8g2.setCursor((128 - (Screen2Row6lenght)*6) / 2, 60);
        u8g2.print(Screen2Row6);

        u8g2.drawStr(0, -40 + a, " ");
        u8g2.drawStr(120, -40 + a, " ");

        if (((a < 85) && (Screen3Row1lenght > 0)) || ((Screen3Row1lenght == 0) && (a > 45))) {      //Kazalec
            u8g2.drawStr(0, -30 + a, ">");
            u8g2.drawStr(120, -30 + a, "<");
        }
        if ((Screen3Row1lenght > 0) && (a > 85)) {
            u8g2.drawStr(0, -30 + a, "v");
            u8g2.drawStr(120, -30 + a, "v");
            u8g2.drawStr(0, 10, " ");
            u8g2.drawStr(120, 10, " ");
        }
        if (a < 45) {
            u8g2.drawStr(0, -30 + a, "^");
            u8g2.drawStr(120, -30 + a, "^");
        }

        u8g2.drawStr(0, -20 + a, " ");
        u8g2.drawStr(120, -20 + a, " ");
        d = 1;
        e = 0;
    }

    if ((a > 95) && (Screen3Row1lenght > 1)) {      //Tretja stran

        if (a > (acap - 10)) {
            a = 0;
            selecta = 1;
            f = 1;
        }

        u8g2.setFont(u8g2_font_6x10_mf);
        u8g2.setCursor((128 - (Screen3Row1lenght)*6) / 2, 10);      //Izpišemo
        u8g2.print(Screen3Row1);
        u8g2.setCursor((128 - (Screen3Row2lenght)*6) / 2, 20);
        u8g2.print(Screen3Row2);
        u8g2.setCursor((128 - (Screen3Row3lenght)*6) / 2, 30);
        u8g2.print(Screen3Row3);
        u8g2.setCursor((128 - (Screen3Row4lenght)*6) / 2, 40);
        u8g2.print(Screen3Row4);
        u8g2.setCursor((128 - (Screen3Row5lenght)*6) / 2, 50);
        u8g2.print(Screen3Row5);
        u8g2.setCursor((128 - (Screen3Row6lenght)*6) / 2, 60);
        u8g2.print(Screen3Row6);

        u8g2.drawStr(0, -100 + a, " ");
        u8g2.drawStr(120, -100 + a, " ");

        if (a > 105) {
            u8g2.drawStr(0, -90 + a, ">");      //Kazalec
            u8g2.drawStr(120, -90 + a, "<");
        }
        if (a < 105) {
            u8g2.drawStr(0, -90 + a, "^");
            u8g2.drawStr(120, -90 + a, "^");
        }

        u8g2.drawStr(0, -80 + a, " ");
        u8g2.drawStr(120, -80 + a, " ");
        e = 1;
    }
    acap = 0;
    u8g2.setFont(u8g2_font_7x13_mf);
    u8g2.sendBuffer();                  //Pošjemo na ekran
}

/*Funkcije za izpisovanje sprejetih pozivov in tistih ki jih pošiljamo.
Funkcija potrebuje data bazo, ki je navedena spodaj. V spremenljivko poziv 
vstavimo spremenljivko iz data baze.
 */
void menuPage(uint16_t poziv) {

    u8g2.clearBuffer();
    counter = 0;
    delay(50);

    pagelen = page.length();        //Določimo dolžino poziva

    while (whilecounter < timeouttime) {
        u8g2.setFont(u8g2_font_6x10_mf);            //Izpišemo grfiko
        u8g2.drawStr(25, 10, "Send message:");
        u8g2.setFont(u8g2_font_7x13_mf);
        u8g2.setCursor((128 - (pagelen)*7) / 2, 30);
        u8g2.print(page);
        u8g2.setFont(u8g2_font_4x6_mf);
        u8g2.drawStr(6, 60, "Press middle button to send.");

        whilecounter = millisTimeout(1000, whilecounter);

        if (counter == 1) {     //Če se je med tem izvršil interupt tipke pošljemo
            u8g2.setFont(u8g2_font_6x10_mf);
            LoRa.beginPacket();
            LoRa.print(poziv);
            LoRa.endPacket(true);
            u8g2.clearBuffer();
            u8g2.clearDisplay();
            u8g2.drawStr(40, 32, "Sending...");
            u8g2.sendBuffer();
            tone(piezo, 3699, 1000);
            delay(1600);
            u8g2.clearDisplay();
            LoRa.receive();
            whilecounter = timeouttime;
        }
        if ((digitalRead(tipka1pin) == 1) || (digitalRead(tipka3pin) == 1)) {       //Če pritisnemo stranske tipke nas vrže ven
            whilecounter = timeouttime;
            u8g2.clearBuffer();
        }
        counter = 0;
    }
}

void recievedPage() {           //Funkcija, ki se izvrši v primeru sprejemanja poziva
    uint16_t tonetime = 1500;
    tone(piezo, 4093, tonetime);
    while ((whilecounter < timeouttime)) {
        whilecounter = millisTimeout(1000, whilecounter);
        pagelen = page.length();
        u8g2.setFont(u8g2_font_6x10_mf);
        u8g2.drawStr(13, 10, "MESSAGE RECIEVED:");      //Izpišemo grafiko
        u8g2.setFont(u8g2_font_7x13_mf);
        u8g2.setCursor((128 - (pagelen)*7) / 2, 30);
        u8g2.print(page);
        u8g2.setFont(u8g2_font_4x6_mf);
        u8g2.drawStr(0, 50, "RSSI:");       //in moč
        u8g2.setCursor(22, 50);
        u8g2.print(RSSI);
        u8g2.sendBuffer();

        check(0);
    }
}

/*SVETILNIK V SILI
Svetilnik v sili je funkcija, ki vsakih 10 sekund pošlje poziv, ki na drugi napravi
sproži sprejemanje v sili in alarm. Vklopi GNSS sprejemnik in če dobimo fiks pošlje še GNSS.
 */
void emergency_beacon() {
    uint16_t paketNO = 0;
    reset(0, 100);
    u8g2.clearBuffer();
    while (counter == 0) {
        u8g2.setFont(u8g2_font_7x13_mf);
        u8g2.drawStr(8, 13, "EMERGENCY BEACON");        //Izpišemo na ekran
        myGNSS.powerSaveMode(0);
        u8g2.setFont(u8g2_font_4x6_mf);
        u8g2.drawStr(0, 20, "Sending beacon signal every 10s!");
        unsigned long currentMillis = millis();
        u8g2.sendBuffer();
        if (currentMillis - previousTime2 >= 10000) {       //Se izvrši vsakih 10 sekund
            paketNO++;
            LoRa.beginPacket();
            LoRa.print(1);
            LoRa.endPacket(true);           //Pošljemo paket
            delay(1500);
            u8g2.setFont(u8g2_font_6x10_mf);
            u8g2.drawStr(0, 40, "Sent packet:");
            u8g2.drawStr(0, 50, "GNSS unavailable!");
            u8g2.setCursor(75, 40);
            u8g2.print(paketNO);

            if (myGNSS.getGnssFixOk() == 1) {           //Se izvrši, če imamo GNSS fiks
                u8g2.drawStr(0, 50, "Sending GNSS     ");

                LoRa_GNSS_Parser(0, myGNSS.getLatitude(), myGNSS.getLongitude(), myGNSS.getAltitude(), 0);      //Pošljemo GNSS podatke, preko uporabe funkcije parser
            }
            previousTime2 = currentMillis;
            u8g2.sendBuffer();
        }
    }
    u8g2.clearBuffer();
    reset(0, 100);
    GNSSisOFF = 0;
}

//Sprejemanje klica v sili 
void emergency_rec() {
    unsigned long time = millis();
    unsigned long seconds = 0;
    unsigned long beforetime = 0;
    reset(0, 100);
    u8g2.clearBuffer();
    float GNSSLAT;
    float GNSSLON;
    float GNSSALT;
    uint16_t zvonenje = 0;
    uint16_t zvonenje2 = 0;
    while ((digitalRead(tipka1pin) == 0) && (digitalRead(tipka3pin) == 0)) {    //Izhod če pritisnemo stranske tipke

        if (onRcounter == 1) {
            onRcounter = 0;
            if (MSGlenght > 5) {
                GNSSLAT = LoRa_GNSS_Parser(1, 0, 0, 0, 1);      //Preberome GNSS podatke, če jih sprejmemo
                GNSSLON = LoRa_GNSS_Parser(1, 0, 0, 0, 2);
                GNSSALT = LoRa_GNSS_Parser(1, 0, 0, 0, 3);
            }

            if ((MSG > 1) && (MSG < 1000)) {
                counter = 1;
            }

            seconds = 0;
        }

        if (zvonenje2 < zvonenje) {
            tone(piezo, 4000, 500);     //Alarm
            zvonenje2 = zvonenje;
        }

        time = millis();
        if (time - beforetime >= 1000) {
            seconds++;
            u8g2.clearBuffer();

            u8g2.setFont(u8g2_font_6x10_mf);
            u8g2.drawStr(1, 10, "RECIEVING SOS BEACON");        //Grafično pokažemo
            u8g2.drawStr(0, 20, "LAST RECIEVE:");
            u8g2.drawStr(0, 30, "LAT:");
            u8g2.drawStr(0, 40, "LON:");
            u8g2.drawStr(0, 50, "ALT:");
            u8g2.drawStr(0, 60, "RSSI:");
            u8g2.setCursor(25, 30);
            u8g2.print(GNSSLAT, 5);
            u8g2.setCursor(25, 40);
            u8g2.print(GNSSLON, 5);
            u8g2.setCursor(25, 50);
            u8g2.print(GNSSALT, 5);
            u8g2.setCursor(85, 20);
            u8g2.print(seconds);
            u8g2.drawStr(100, 20, "(s)");

            u8g2.setCursor(32, 60);
            u8g2.print(RSSI);
            MSG = 0;
            beforetime = time;
            u8g2.sendBuffer();
            zvonenje++;
            tone(piezo, 3500, 500);
        }
    }

    u8g2.clearBuffer();
    reset(0, 100);
    reset(1, 100);
}

/*PING
Funkcija ping služi testu dvostranske povezave. Oddajnik odda poziv, nato ga sprejemnik sprejme
in nazaj odda poziv. Oddajnik sprejeti poziv izpiše in to pomeni, da smo uspešno naredilo vezo
v obe strani.
*/
void ping() {
    reset(0, 200);
    u8g2.clearBuffer();
    bool again = 0;
    uint8_t NoRec = 0;

    while ((whilecounter < timeouttime) || (again == 1)) {

        if (again == 1) {
            whilecounter = 0;
            again = 0;
        }

        whilecounter = millisTimeout(1000, whilecounter);
        u8g2.setFont(u8g2_font_6x10_mf);
        u8g2.drawStr(0, 10, "PRESS TO PING!");
        u8g2.sendBuffer();
        if ((digitalRead(tipka1pin) == 1) || (digitalRead(tipka3pin) == 1)) {       //Izhod, če pritisnjemo stranske tipke
            whilecounter = timeouttime;
            u8g2.clearBuffer();
        }

        if (counter == 1) {
            counter = 0;
            u8g2.drawStr(0, 20, "Pinging...");
            u8g2.sendBuffer();
            LoRa.beginPacket();
            LoRa.print(2);
            LoRa.endPacket(true);
            delay(1500);
            LoRa.receive();
            uint8_t chech = 0;

            while (whilecounter < timeouttime) {                    //Čakamo na vrnjen poziv
                whilecounter = millisTimeout(1000, whilecounter);
                NoRec = millisTimeout1(1000, NoRec);

                if (onRcounter == 1) {
                    MSG = MSGstring.toInt();
                    if (MSG == 2) {
                        reset(0, 200);
                        u8g2.drawStr(0, 40, "Ping successful!");    //Se izpiše če je uspešen
                        u8g2.drawStr(0, 50, "RSSI:");
                        u8g2.setCursor(35, 50);
                        u8g2.print(RSSI);       //Izpišemo moč signala
                        u8g2.sendBuffer();
                        again = 1;
                    }
                    if (MSG != 2) {
                        whilecounter = timeouttime;
                        counter = 0;
                    }
                }
                if ((onRcounter == 0) && (NoRec > 10) && (a == 0)) {
                    u8g2.drawStr(0, 40, "Ping unsuccessful!");          //Se izpiše po 10 sekunda, če je neuspešen
                    u8g2.sendBuffer();
                    again = 1;
                }
                if ((digitalRead(tipka1pin) == 1) || (digitalRead(tipka3pin) == 1)) {
                    whilecounter = timeouttime;
                    u8g2.clearBuffer();
                }
                if ((counter == 1) && (again == 1)) {
                    counter = 0;
                    again = 0;
                    onRcounter = 0;
                    whilecounter = timeouttime;
                    NoRec = 0;
                    u8g2.clearBuffer();
                }
            }
        }
    }
    reset(0, 100);
    u8g2.clearBuffer();
}

void pingREC() {        //Sprejme in odda ping nazaj
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_mf);
    u8g2.drawStr(0, 10, "Returning ping!");
    u8g2.drawStr(0, 21, "Please wait!");
    u8g2.sendBuffer();
    delay(1500);
    LoRa.beginPacket();
    LoRa.print(2);
    LoRa.endPacket(true);
    delay(1500);
    LoRa.receive();
    u8g2.clearBuffer();
    whilecounter = timeouttime;
}

/*Databaza pozivov
    Switch >> v switch specificiras spemenljivko poziva
    */
void functionswitch(uint16_t x) {
    if (fdecl == 1) {
        menuPage(x);
    } else if (fdecl == 0) {
        recievedPage();
    }
}

void pages(uint16_t Switch) {

    switch (Switch) {
    case 0:
        break;
    case 1:
        emergency_rec();
        break;
    case 2:
        pingREC();
        break;
    case 3:
        page = "Where are you?";
        functionswitch(Switch);
        break;
    case 4:
        page = "GPSn Smwhr N";
        functionswitch(Switch);
        break;
    case 5:
        page = "GPSn Smwhr NE";
        functionswitch(Switch);
        break;
    case 6:
        page = "GPSn Smwhr E";
        functionswitch(Switch);
        break;
    case 7:
        page = "GPSn Smwhr SE";
        functionswitch(Switch);
        break;
    case 8:
        page = "GPSn Smwhr S";
        functionswitch(Switch);
        break;
    case 9:
        page = "GPSn Smwhr SW";
        functionswitch(Switch);
        break;
    case 10:
        page = "GPSn Smwhr W";
        functionswitch(Switch);
        break;
    case 11:
        page = "GPSn Smwhr NW";
        functionswitch(Switch);
        break;
    case 12:
        page = "GPSn Peak";
        functionswitch(Switch);
        break;
    case 13:
        page = "Right Behind You";
        functionswitch(Switch);
        break;
    case 14:
        page = "Heading N";
        functionswitch(Switch);
        break;
    case 15:
        page = "Heading NE";
        functionswitch(Switch);
        break;
    case 16:
        page = "Heading E";
        functionswitch(Switch);
        break;
    case 17:
        page = "Heading SE";
        functionswitch(Switch);
        break;
    case 18:
        page = "Heading S";
        functionswitch(Switch);
        break;
    case 19:
        page = "Heading SW";
        functionswitch(Switch);
        break;
    case 20:
        page = "Heading W";
        functionswitch(Switch);
        break;
    case 21:
        page = "Heading NW";
        functionswitch(Switch);
        break;
    case 22:
        page = "Heading Peak";
        functionswitch(Switch);
        break;
    case 23:
        page = "Heading Upstr.";
        functionswitch(Switch);
        break;
    case 24:
        page = "Heading Downstr.";
        functionswitch(Switch);
        break;
    case 25:
        page = "Mountain Side";
        functionswitch(Switch);
        break;
    case 26:
        page = "Grass Field";
        functionswitch(Switch);
        break;
    case 27:
        page = "Spruce Forest";
        functionswitch(Switch);
        break;
    case 28:
        page = "Birch Forest";
        functionswitch(Switch);
        break;
    case 29:
        page = "Beech Forest";
        functionswitch(Switch);
        break;
    case 30:
        page = "Oak Forest";
        functionswitch(Switch);
        break;
    case 31:
        page = "Pine Forest";
        functionswitch(Switch);
        break;
    case 32:
        page = "Linden Forest";
        functionswitch(Switch);
        break;
    case 33:
        page = "Hazel Forest";
        functionswitch(Switch);
        break;
    case 34:
        page = "Low Hills";
        functionswitch(Switch);
        break;
    case 35:
        page = "Medium Hills";
        functionswitch(Switch);
        break;
    case 36:
        page = "Tall Hills";
        functionswitch(Switch);
        break;
    case 37:
        page = "V Valley";
        functionswitch(Switch);
        break;
    case 38:
        page = "U Valley";
        functionswitch(Switch);
        break;
    case 39:
        page = "Canyon";
        functionswitch(Switch);
        break;
    case 40:
        page = "Camp Visible";
        functionswitch(Switch);
        break;
    case 41:
        page = "500 from Camp";
        functionswitch(Switch);
        break;
    case 42:
        page = "1k frm Camp";
        functionswitch(Switch);
        break;
    case 43:
        page = "2k frm Camp";
        functionswitch(Switch);
        break;
    case 44:
        page = "5k frm Camp";
        functionswitch(Switch);
        break;
    case 45:
        page = "10k frm Camp";
        functionswitch(Switch);
        break;
    case 46:
        page = "Near Church";
        functionswitch(Switch);
        break;
    case 47:
        page = "Near Sinkhole";
        functionswitch(Switch);
        break;
    case 48:
        page = "Near Ruins";
        functionswitch(Switch);
        break;
    case 49:
        page = "Near Lake";
        functionswitch(Switch);
        break;
    case 50:
        page = "Near Sea";
        functionswitch(Switch);
        break;
    case 51:
        page = "Near River";
        functionswitch(Switch);
        break;
    case 52:
        page = "River in eyesight ";
        functionswitch(Switch);
        break;
    case 53:
        page = "River 100m";
        functionswitch(Switch);
        break;
    case 54:
        page = "River 500m";
        functionswitch(Switch);
        break;
    case 55:
        page = "Physical status?";
        functionswitch(Switch);
        break;
    case 56:
        page = "Great";
        functionswitch(Switch);
        break;
    case 57:
        page = "Good";
        functionswitch(Switch);
        break;
    case 58:
        page = "Okay";
        functionswitch(Switch);
        break;
    case 59:
        page = "Little Hungry";
        functionswitch(Switch);
        break;
    case 60:
        page = "Mod. Hungry";
        functionswitch(Switch);
        break;
    case 61:
        page = "Very Hungry";
        functionswitch(Switch);
        break;
    case 62:
        page = "Critical Starving";
        functionswitch(Switch);
        break;
    case 63:
        page = "Little Thirsty";
        functionswitch(Switch);
        break;
    case 64:
        page = "Mod. Thirsty";
        functionswitch(Switch);
        break;
    case 65:
        page = "Very Thirsty";
        functionswitch(Switch);
        break;
    case 66:
        page = "Critical Thirsty";
        functionswitch(Switch);
        break;
    case 67:
        page = "Tired";
        functionswitch(Switch);
        break;
    case 68:
        page = "Very Tired";
        functionswitch(Switch);
        break;
    case 69:
        page = "Fatigued";
        functionswitch(Switch);
        break;
    case 70:
        page = "Lightly hurt";
        functionswitch(Switch);
        break;
    case 71:
        page = "Mod. hurt";
        functionswitch(Switch);
        break;
    case 72:
        page = "Very hurt";
        functionswitch(Switch);
        break;
    case 73:
        page = "Broken bone";
        functionswitch(Switch);
        break;
    case 74:
        page = "Sprained joint ";
        functionswitch(Switch);
        break;
    case 75:
        page = "Light Bleeding";
        functionswitch(Switch);
        break;
    case 76:
        page = "Mod. Bleeding";
        functionswitch(Switch);
        break;
    case 77:
        page = "Critical Bleed.";
        functionswitch(Switch);
        break;
    case 78:
        page = "Immobile";
        functionswitch(Switch);
        break;
    case 79:
        page = "Breedable";
        functionswitch(Switch);
        break;
    case 80:
        page = "Situation?";
        functionswitch(Switch);
        break;
    case 81:
        page = "Falling back";
        functionswitch(Switch);
        break;
    case 82:
        page = "In Position";
        functionswitch(Switch);
        break;
    case 83:
        page = "Not ready";
        functionswitch(Switch);
        break;
    case 84:
        page = "Idling";
        functionswitch(Switch);
        break;
    case 85:
        page = "Busy";
        functionswitch(Switch);
        break;
    case 86:
        page = "Compleated";
        functionswitch(Switch);
        break;
    case 87:
        page = "In Progress";
        functionswitch(Switch);
        break;
    case 88:
        page = "Setting up camp";
        functionswitch(Switch);
        break;
    case 89:
        page = "Get. R. for dark";
        functionswitch(Switch);
        break;
    case 90:
        page = "Setting up fire";
        functionswitch(Switch);
        break;
    case 91:
        page = "Following";
        functionswitch(Switch);
        break;
    case 92:
        page = "Supplies?";
        functionswitch(Switch);
        break;
    case 93:
        page = "Supp. Surplus";
        functionswitch(Switch);
        break;
    case 94:
        page = "Supp. Good";
        functionswitch(Switch);
        break;
    case 95:
        page = "Supp. Lack";
        functionswitch(Switch);
        break;
    case 96:
        page = "Supp. Insufficient";
        functionswitch(Switch);
        break;
    case 97:
        page = "Hazard?";
        functionswitch(Switch);
        break;
    case 98:
        page = "Fire Hazard";
        functionswitch(Switch);
        break;
    case 99:
        page = "Water Hazard";
        functionswitch(Switch);
        break;
    case 100:
        page = "Gas Hazzard";
        functionswitch(Switch);
        break;
    case 101:
        page = "RadioA. Hazzard";
        functionswitch(Switch);
        break;
    case 102:
        page = "Height Hazzard";
        functionswitch(Switch);
        break;
    case 103:
        page = "Contamination";
        functionswitch(Switch);
        break;
    case 104:
        page = "Phone Signal?";
        functionswitch(Switch);
        break;
    case 105:
        page = "P. Signal Strong";
        functionswitch(Switch);
        break;
    case 106:
        page = "P. Signal Mod.";
        functionswitch(Switch);
        break;
    case 107:
        page = "P. Signal Weak";
        functionswitch(Switch);
        break;
    case 108:
        page = "No P. Signal";
        functionswitch(Switch);
        break;
    case 109:
        page = "RSSI?";
        functionswitch(Switch);
        break;
    case 110:
        page = "RSSI < 50";
        functionswitch(Switch);
        break;
    case 111:
        page = "RSSI 50-100";
        functionswitch(Switch);
        break;
    case 112:
        page = "RSSI 100-110";
        functionswitch(Switch);
        break;
    case 113:
        page = "RSSI 110-120";
        functionswitch(Switch);
        break;
    case 114:
        page = "RSSI 120-130";
        functionswitch(Switch);
        break;
    case 115:
        page = "RSSI  >130";
        functionswitch(Switch);
        break;

    default:
        if (fdecl == 0) {
            page = "Unknown msg";
            functionswitch(Switch);
        }
        break;
    }
}

// back
bool back(bool x) {
    if (x == 1) {
        u8g2.clearBuffer();
        whilecounter = 0;
        return 0;
    }
    return (0);
}
/*Funkcija NASTAVITEV
S to funkcijo si lahko preko menija z level() funkcijo spremenimo nastavitve LoRa, GNSSa 
in navadne sistemske nastavitve, lahko tudi izključimo pozivnik ali ga resetiramo.*/
void SET() {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_7x13_mf);
    u8g2.drawStr(50, 38, "SET!");
    u8g2.sendBuffer();
    delay(750);
    u8g2.clearBuffer();
}
void settings() {
    reset(1, 100);
    reset(0, 100);
    u8g2.clearBuffer();
    while ((whilecounter < timeouttime) && (m1 == 0)) {
        m2 = back(m2);
        whilecounter = millisTimeout(1000, whilecounter);
        TITLE = "SETTINGS";
        Screen1Row1 = "GNSS Settings";
        Screen1Row2 = "LoRa Settings";
        Screen1Row3 = "Timeout Time";
        Screen1Row4 = "Power Menu";
        Screen2Row1 = "Credits";
        Screen2Row2 = "back";
        level();
        check(1);

        if (counter == 1) {
            counter = 0;

            switch (selecta) {
            case 1:
                u8g2.clearBuffer();
                reset(1, 100);
                reset(0, 100);
                while ((whilecounter < timeouttime) && (m2 == 0)) {
                    m3 = back(m3);
                    whilecounter = millisTimeout(1000, whilecounter);
                    TITLE = "GNSS SETTINGS";
                    Screen1Row1 = "Set Sleep Mode";
                    Screen1Row2 = "Power Save Mode";
                    Screen1Row3 = "back";
                    level();
                    check(1);

                    if (counter == 1) {
                        counter = 0;
                        switch (selecta) {
                        case 1:
                            reset(1, 100);
                            reset(0, 100);
                            u8g2.clearBuffer();
                            while ((whilecounter < timeouttime) && (m3 == 0)) {
                                whilecounter = millisTimeout(1000, whilecounter);
                                TITLE = "SLEEP MODE";
                                Screen1Row1 = "Always Sleep";
                                Screen1Row2 = "Sleep when OFF(def)";
                                Screen1Row3 = "Never Sleep";
                                Screen1Row4 = "back";
                                level();
                                check(1);

                                if (counter == 1) {
                                    counter = 0;
                                    switch (selecta) {
                                    case 1:
                                        GNSSsleepSETTIING = 1;
                                        SET();
                                        break;
                                    case 2:
                                        GNSSsleepSETTIING = 2;
                                        SET();
                                        break;
                                    case 3:
                                        GNSSsleepSETTIING = 3;
                                        myGNSS.getGnssFixOk();
                                        GNSSisOFF = 0;
                                        SET();
                                        break;
                                    case 4:
                                        m3 = 1;
                                        u8g2.clearBuffer();
                                        reset(1, 100);
                                        reset(0, 100);
                                        break;
                                    }
                                }
                            }
                            break;
                        case 2:
                            reset(1, 100);
                            reset(0, 100);
                            u8g2.clearBuffer();
                            while ((whilecounter < timeouttime) && (m3 == 0)) {
                                whilecounter = millisTimeout(1000, whilecounter);
                                TITLE = "POWER SAVE MODE";
                                Screen1Row1 = "ON(def)";
                                Screen1Row2 = "OFF";
                                Screen1Row3 = "back";
                                level();
                                check(1);

                                if (counter == 1) {
                                    counter = 0;
                                    switch (selecta) {
                                    case 1:
                                        myGNSS.powerSaveMode(1);
                                        GNSSisOFF = 0;
                                        SET();
                                        break;
                                    case 2:
                                        myGNSS.powerSaveMode(0);
                                        GNSSisOFF = 0;
                                        SET();
                                        break;
                                    case 3:
                                        m3 = 1;
                                        u8g2.clearBuffer();
                                        reset(1, 100);
                                        reset(0, 100);
                                        break;
                                    }
                                }
                            }
                            break;
                        case 3:
                            m2 = 1;
                            u8g2.clearBuffer();
                            reset(1, 100);
                            reset(0, 100);
                            break;
                        }
                    }
                }
                break;
            case 2:
                u8g2.clearBuffer();
                reset(1, 100);
                reset(0, 100);
                while ((whilecounter < timeouttime) && (m2 == 0)) {
                    m3 = back(m3);
                    whilecounter = millisTimeout(1000, whilecounter);
                    TITLE = "LoRa SETTINGS";
                    Screen1Row1 = "TX power";
                    Screen1Row2 = "Spredaing factor";
                    Screen1Row3 = "Bandwidth";
                    Screen1Row4 = "CD Denominator";
                    Screen2Row1 = "CRC";
                    Screen2Row2 = "LNA Gain";
                    Screen2Row3 = "back";
                    level();
                    check(1);

                    if (counter == 1) {
                        counter = 0;
                        switch (selecta) {
                        case 1:
                            u8g2.clearBuffer();
                            reset(1, 100);
                            reset(0, 100);
                            while ((whilecounter < timeouttime) && (m3 == 0)) {
                                whilecounter = millisTimeout(1000, whilecounter);
                                TITLE = "TX POWER";
                                Screen1Row1 = "2 dB";
                                Screen1Row2 = "4 dB";
                                Screen1Row3 = "6 dB";
                                Screen1Row4 = "8 dB";
                                Screen2Row1 = "10 dB";
                                Screen2Row2 = "12 dB";
                                Screen2Row3 = "14 dB";
                                Screen2Row4 = "16 dB";
                                Screen2Row5 = "18 dB";
                                Screen2Row6 = "20 dB(def)";
                                Screen3Row1 = "back";
                                level();
                                check(1);

                                if (counter == 1) {
                                    counter = 0;
                                    switch (selecta) {
                                    case 1:
                                        LoRa.setTxPower(2);
                                        SET();
                                        break;
                                    case 2:
                                        LoRa.setTxPower(4);
                                        SET();
                                        break;
                                    case 3:
                                        LoRa.setTxPower(6);
                                        SET();
                                        break;
                                    case 4:
                                        LoRa.setTxPower(8);
                                        SET();
                                        break;
                                    case 5:
                                        LoRa.setTxPower(10);
                                        SET();
                                        break;
                                    case 6:
                                        LoRa.setTxPower(12);
                                        SET();
                                        break;
                                    case 7:
                                        LoRa.setTxPower(14);
                                        SET();
                                        break;
                                    case 8:
                                        LoRa.setTxPower(16);
                                        SET();
                                        break;
                                    case 9:
                                        LoRa.setTxPower(18);
                                        SET();
                                        break;
                                    case 10:
                                        LoRa.setTxPower(20);
                                        SET();
                                        break;
                                    case 11:
                                        m3 = 1;
                                        u8g2.clearBuffer();
                                        reset(1, 100);
                                        reset(0, 100);
                                        break;
                                    }
                                }
                            }
                            break;
                        case 2:
                            u8g2.clearBuffer();
                            reset(1, 100);
                            reset(0, 100);
                            while ((whilecounter < timeouttime) && (m3 == 0)) {
                                whilecounter = millisTimeout(1000, whilecounter);
                                TITLE = "SPREADING";
                                Screen1Row1 = "Factor 6";
                                Screen1Row2 = "Factor 7";
                                Screen1Row3 = "Factor 8";
                                Screen1Row4 = "Factor 9";
                                Screen2Row1 = "Factor 10";
                                Screen2Row2 = "Factor 11";
                                Screen2Row3 = "Factor 12(def)";
                                Screen2Row4 = "back";
                                level();
                                check(1);

                                if (counter == 1) {
                                    counter = 0;
                                    switch (selecta) {
                                    case 1:
                                        LoRa.setSpreadingFactor(6);
                                        SET();
                                        break;
                                    case 2:
                                        LoRa.setSpreadingFactor(7);
                                        SET();
                                        break;
                                    case 3:
                                        LoRa.setSpreadingFactor(8);
                                        SET();
                                        break;
                                    case 4:
                                        LoRa.setSpreadingFactor(9);
                                        SET();
                                        break;
                                    case 5:
                                        LoRa.setSpreadingFactor(10);
                                        SET();
                                        break;
                                    case 6:
                                        LoRa.setSpreadingFactor(11);
                                        SET();
                                        break;
                                    case 7:
                                        LoRa.setSpreadingFactor(12);
                                        SET();
                                        break;
                                    case 8:
                                        m3 = 1;
                                        u8g2.clearBuffer();
                                        reset(1, 100);
                                        reset(0, 100);
                                        break;
                                    }
                                }
                            }
                            break;
                        case 3:
                            u8g2.clearBuffer();
                            reset(1, 100);
                            reset(0, 100);
                            while ((whilecounter < timeouttime) && (m3 == 0)) {
                                whilecounter = millisTimeout(1000, whilecounter);
                                TITLE = "BANDWIDTH";
                                Screen1Row1 = "31.25 kHz(unstable)";
                                Screen1Row2 = "41.7 kHz(unstable)";
                                Screen1Row3 = "62.5 kHz(def)";
                                Screen1Row4 = "125 kHz";
                                Screen2Row1 = "250 kHz";
                                Screen2Row2 = "500 kHz";
                                Screen2Row3 = "back";
                                level();
                                check(1);

                                if (counter == 1) {
                                    counter = 0;
                                    switch (selecta) {
                                    case 1:
                                        LoRa.setSignalBandwidth(31.25E3);
                                        SET();
                                        break;
                                    case 2:
                                        LoRa.setSignalBandwidth(41.7E3);
                                        SET();
                                        break;
                                    case 3:
                                        LoRa.setSignalBandwidth(62.5E3);
                                        SET();
                                        break;
                                    case 4:
                                        LoRa.setSignalBandwidth(125E3);
                                        SET();
                                        break;
                                    case 5:
                                        LoRa.setSignalBandwidth(250E3);
                                        SET();
                                        break;
                                    case 6:
                                        LoRa.setSpreadingFactor(500E3);
                                        SET();
                                        break;
                                    case 7:
                                        m3 = 1;
                                        u8g2.clearBuffer();
                                        reset(1, 100);
                                        reset(0, 100);
                                        break;
                                    }
                                }
                            }
                            break;
                        case 4:
                            u8g2.clearBuffer();
                            reset(1, 100);
                            reset(0, 100);
                            while ((whilecounter < timeouttime) && (m3 == 0)) {
                                whilecounter = millisTimeout(1000, whilecounter);
                                TITLE = "CD DENOMINATOR";
                                Screen1Row1 = "4/5";
                                Screen1Row2 = "4/8(def)";
                                Screen1Row3 = "back";
                                level();
                                check(1);

                                if (counter == 1) {
                                    counter = 0;
                                    switch (selecta) {
                                    case 1:
                                        LoRa.setCodingRate4(5);
                                        SET();
                                        break;
                                    case 2:
                                        LoRa.setCodingRate4(8);
                                        SET();
                                        break;
                                    case 3:
                                        m3 = 1;
                                        u8g2.clearBuffer();
                                        reset(1, 100);
                                        reset(0, 100);
                                        break;
                                    }
                                }
                            }
                            break;
                        case 5:
                            u8g2.clearBuffer();
                            reset(1, 100);
                            reset(0, 100);
                            while ((whilecounter < timeouttime) && (m3 == 0)) {
                                whilecounter = millisTimeout(1000, whilecounter);
                                TITLE = "CRC";
                                Screen1Row1 = "ON";
                                Screen1Row2 = "OFF(def)";
                                Screen1Row3 = "back";
                                level();
                                check(1);

                                if (counter == 1) {
                                    counter = 0;
                                    switch (selecta) {
                                    case 1:
                                        LoRa.enableCrc();
                                        SET();
                                        break;
                                    case 2:
                                        LoRa.disableCrc();
                                        SET();
                                        break;
                                    case 3:
                                        m3 = 1;
                                        u8g2.clearBuffer();
                                        reset(1, 100);
                                        reset(0, 100);
                                        break;
                                    }
                                }
                            }
                            break;
                        case 6:
                            u8g2.clearBuffer();
                            reset(1, 100);
                            reset(0, 100);
                            while ((whilecounter < timeouttime) && (m3 == 0)) {
                                whilecounter = millisTimeout(1000, whilecounter);
                                TITLE = "LNA GAIN";
                                Screen1Row1 = "OFF(def)";
                                Screen1Row2 = "6 dB";
                                Screen1Row3 = "5 dB";
                                Screen1Row4 = "4 dB";
                                Screen2Row1 = "3 dB";
                                Screen2Row2 = "2 dB";
                                Screen2Row3 = "1 dB";
                                Screen2Row4 = "back";
                                level();
                                check(1);

                                if (counter == 1) {
                                    counter = 0;
                                    switch (selecta) {
                                    case 1:
                                        LoRa.setGain(0);
                                        SET();
                                        break;
                                    case 2:
                                        LoRa.setGain(6);
                                        SET();
                                        break;
                                    case 3:
                                        LoRa.setGain(5);
                                        SET();
                                        break;
                                    case 4:
                                        LoRa.setGain(4);
                                        SET();
                                        break;
                                    case 5:
                                        LoRa.setGain(3);
                                        SET();
                                        break;
                                    case 6:
                                        LoRa.setGain(2);
                                        SET();
                                        break;
                                    case 7:
                                        LoRa.setGain(1);
                                        SET();
                                        break;
                                    case 8:
                                        m3 = 1;
                                        u8g2.clearBuffer();
                                        reset(1, 100);
                                        reset(0, 100);
                                        break;
                                    }
                                }
                            }
                            break;
                        case 7:
                            m2 = 1;
                            u8g2.clearBuffer();
                            reset(1, 100);
                            reset(0, 100);
                            break;
                        }
                    }
                }
                break;
            case 3:
                u8g2.clearBuffer();
                reset(1, 100);
                reset(0, 100);
                while ((whilecounter < timeouttime) && (m2 == 0)) {
                    whilecounter = millisTimeout(1000, whilecounter);
                    TITLE = "TIMEOUT TIME";
                    Screen1Row1 = "5 Seconds";
                    Screen1Row2 = "10 Seconds";
                    Screen1Row3 = "15 Seconds";
                    Screen1Row4 = "20 Seconds";
                    Screen2Row1 = "30 Seconds";
                    Screen2Row2 = "1 Minute";
                    Screen2Row3 = "2 Minutes";
                    Screen2Row4 = "5 Minutes";
                    Screen2Row5 = "Never";
                    Screen2Row6 = "back";
                    level();
                    check(1);

                    if (counter == 1) {
                        counter = 0;
                        switch (selecta) {
                        case 1:
                            timeouttime = 5;
                            SET();
                            break;
                        case 2:
                            timeouttime = 10;
                            SET();
                            break;
                        case 3:
                            timeouttime = 15;
                            SET();
                            break;
                        case 4:
                            timeouttime = 20;
                            SET();
                            break;
                        case 5:
                            timeouttime = 30;
                            SET();
                            break;
                        case 6:
                            timeouttime = 60;
                            SET();
                            break;
                        case 7:
                            timeouttime = 120;
                            SET();
                            break;
                        case 8:
                            timeouttime = 300;
                            SET();
                            break;
                        case 9:
                            timeouttime = 65534;
                            break;
                        case 10:
                            m2 = 1;
                            u8g2.clearBuffer();
                            reset(1, 100);
                            reset(0, 100);
                            break;
                        }
                    }
                }
                break;
            case 4:
                u8g2.clearBuffer();
                reset(1, 100);
                reset(0, 100);
                while ((whilecounter < timeouttime) && (m2 == 0)) {
                    whilecounter = millisTimeout(1000, whilecounter);
                    TITLE = "POWER MENU";
                    Screen1Row1 = "Sleep";
                    Screen1Row2 = "Power Off";
                    Screen1Row3 = "Restart";
                    Screen1Row4 = "back";
                    level();
                    check(1);

                    if (counter == 1) {
                        counter = 0;
                        switch (selecta) {
                        case 1:
                            whilecounter = timeouttime;
                            break;
                        case 2:
                            axp.setDCDC1Voltage(0);
                            axp.setDCDC2Voltage(0);
                            axp.setDCDC3Voltage(0);
                            break;
                        case 3:
                            ESP.restart();
                            break;
                        case 4:
                            m2 = 1;
                            u8g2.clearBuffer();
                            reset(1, 100);
                            reset(0, 100);
                            break;
                        }
                    }
                }
                break;
            case 5:
                u8g2.clearBuffer();
                reset(1, 100);
                reset(0, 100);
                while ((whilecounter < timeouttime) && (m2 == 0)) {
                    whilecounter = millisTimeout(1000, whilecounter);
                    TITLE = "CREDITS";
                    TITLElenght = TITLE.length();
                    u8g2.drawLine(0, 16, 128, 16);
                    u8g2.setFont(u8g2_font_7x13_mf);
                    u8g2.setCursor((128 - (TITLElenght)*7) / 2, 12);
                    u8g2.print(TITLE);
                    u8g2.setFont(u8g2_font_6x10_mf);
                    u8g2.drawStr(0, 30, "Code written by JERNEJ");
                    u8g2.drawStr(0, 40, "LESKOVEC, for end of  ");
                    u8g2.drawStr(0, 50, "highschool project on ");
                    u8g2.drawStr(0, 60, "VEGOVA Ljubljana.     ");
                    u8g2.sendBuffer();
                    check(1);

                    if ((digitalRead(tipka1pin) == 1) || (digitalRead(tipka3pin) == 1)) {
                        m2 = 1;
                        u8g2.clearBuffer();
                        reset(1, 100);
                        reset(0, 100);
                    }

                    if (counter == 1) {
                        u8g2.clearBuffer();

                        reset(0, 100);
                        while (whilecounter < timeouttime) {
                            whilecounter = millisTimeout(1000, whilecounter);
                            u8g2.setFont(u8g2_font_6x10_mf);
                            u8g2.drawStr(0, 10, "Written in Platformio");
                            u8g2.drawStr(0, 20, "using Arduino archit-");
                            u8g2.drawStr(0, 30, "ecture. Based on LILY");
                            u8g2.drawStr(0, 40, "GO TTGO T-BEAM.      ");
                            u8g2.drawStr(0, 50, "On board there is an ");
                            u8g2.drawStr(0, 60, "ESP32, LoRa Semtech  ");
                            u8g2.sendBuffer();

                            if (counter == 1) {
                                u8g2.clearBuffer();

                                reset(0, 100);
                                while (whilecounter < timeouttime) {
                                    whilecounter = millisTimeout(1000, whilecounter);
                                    u8g2.setFont(u8g2_font_6x10_mf);
                                    u8g2.drawStr(0, 10, "sx1278, UBLOX M8N and");
                                    u8g2.drawStr(0, 20, "AXP20X. Libraries:   ");
                                    u8g2.drawStr(0, 30, "1. AXP202X_Library by");
                                    u8g2.drawStr(0, 40, "lewisxhe");
                                    u8g2.drawStr(0, 50, "2. SparkFun u-blox   ");
                                    u8g2.drawStr(0, 60, "GNSS Arduino Library ");
                                    u8g2.sendBuffer();

                                    if (counter == 1) {
                                        u8g2.clearBuffer();

                                        reset(0, 100);
                                        while (whilecounter < timeouttime) {
                                            whilecounter = millisTimeout(1000, whilecounter);
                                            u8g2.setFont(u8g2_font_6x10_mf);
                                            u8g2.drawStr(0, 10, "3. u8g2 by olikraus  ");
                                            u8g2.drawStr(0, 20, "4. LoRa by           ");
                                            u8g2.drawStr(0, 30, "sandeepmistry        ");
                                            u8g2.drawStr(0, 40, "5. EspSoftwareSerial ");
                                            u8g2.drawStr(0, 50, "by plerup            ");
                                            u8g2.drawStr(0, 60, "                     ");
                                            u8g2.sendBuffer();
                                            if (counter == 1) {
                                                whilecounter = timeouttime;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                break;
            case 6:
                m1 = 1;
                u8g2.clearBuffer();
                reset(1, 100);
                reset(0, 100);
                break;
            }
        }
    }
}

/*GNSS prikazovalnik
Prikazuje GNSS podatke
 */
void GNSSinfo() {
    if (GNSSsleepSETTIING != 1) {
        reset(0, 250);
        reset(1, 250);
        u8g2.clearBuffer();

        bool buffer1 = 0;       //nastavimo spremenljivke
        bool buffer2 = 0;
        bool buffer3 = 0;
        bool buffer4 = 0;

        uint16_t second = 0;
        uint16_t hour = 0;
        uint16_t minute = 0;
        uint16_t day = 0;
        uint16_t month = 0;
        uint16_t year = 0;

        uint8_t fixtype = 0;
        float gpsfloatLAT;
        float gpsfloatLON;
        float gpsfloatALT;
        float gpsfloatACC;

        int16_t gpskmh;
        uint8_t gpskmhlng;

        float angle_rad = 0;
        float x1;
        float x2;
        float y1;
        float y2;

        float x3;
        float x4;
        float y3;
        float y4;

        u8g2.clearBuffer();

        u8g2.setFont(u8g2_font_6x10_mf);
        u8g2.drawStr(0, 10, "GNSS info:");
        u8g2.drawLine(0, 20, 128, 20);
        u8g2.setFont(u8g2_font_4x6_mf);
        u8g2.drawStr(116, 30, "K/H");
        u8g2.drawStr(98, 28, "N");
        u8g2.drawCircle(100, 45, 17);

        u8g2.sendBuffer();

        while ((counter == 0) && (onRcounter == 0)) {   //najprej naredimo vse kalkulacije

            if (myGNSS.getTimeValid() == 0) {
                buffer1 = 0;
            }

            if (myGNSS.getTimeValid() == 1) {
                buffer1 = 1;

                second = myGNSS.getSecond();        //Preberemo čas
                minute = myGNSS.getMinute();
                hour = myGNSS.getHour();
            }

            if (myGNSS.getDateValid() == 0) {
                buffer2 = 0;
            }

            if (myGNSS.getDateValid() == 1) {
                buffer2 = 1;
                unsigned long currentMillis = millis();
                u8g2.setFont(u8g2_font_4x6_mf);

                day = myGNSS.getDay();          //Preberemo datum
                month = myGNSS.getMonth();
                year = myGNSS.getYear();
            }

            if (myGNSS.getGnssFixOk() == 0) {
                buffer3 = 0;
                myGNSS.powerSaveMode(0);
            }

            if (myGNSS.getGnssFixOk() == 1) {
                buffer3 = 1;

                fixtype = myGNSS.getFixType();

                gnssLAT = myGNSS.getLatitude();         //Preberemo koordinate
                gnssLON = myGNSS.getLongitude();
                gnssALT = myGNSS.getAltitudeMSL();
                SIV = myGNSS.getSIV();                  //Preberemo število satelitov
                Speed = myGNSS.getGroundSpeed();        //hitrost
                ACC = myGNSS.getHorizontalAccEst();     //napako

                gpsfloatLAT = gnssLAT / 10000000.;
                gpsfloatLON = gnssLON / 10000000.;
                gpsfloatALT = gnssALT / 1000.;
                gpsfloatACC = ACC / 1000.;

                gpskmh = (Speed / 1000) * 3.6;
                String gpskmhstr = String(gpskmh);
                gpskmhlng = gpskmhstr.length();
                u8g2.setCursor(((128 - (gpskmhlng)*5) / 2) + 36, 48);
                u8g2.print(gpskmh);

                if (gpskmh <= 2) {
                    buffer4 = 0;
                }
                if (gpskmh > 2) {
                    angle_rad = (myGNSS.getHeading() * 10 ^ 5) * M_PI / 180;    //Zračunamo koordinate za risati puščico
                    x1 = (16 * sin(angle_rad)) + 100;
                    y1 = (16 * cos(angle_rad)) + 45;
                    x2 = (-16 * sin(angle_rad)) + 100;
                    y2 = (-16 * cos(angle_rad)) + 45;

                    y3 = (13 * cos(angle_rad - (15 * M_PI / 180))) + 45;
                    y4 = (13 * cos(angle_rad + (15 * M_PI / 180))) + 45;
                    x3 = (13 * sin(angle_rad - (15 * M_PI / 180))) + 100;
                    x4 = (13 * sin(angle_rad + (15 * M_PI / 180))) + 100;
                    buffer4 = 1;
                }
            }

            u8g2.clearBuffer();

            u8g2.setFont(u8g2_font_6x10_mf);
            u8g2.drawStr(0, 10, "GNSS info:");
            u8g2.drawLine(0, 20, 128, 20);
            u8g2.setFont(u8g2_font_4x6_mf);
            u8g2.drawStr(116, 30, "K/H");
            u8g2.drawStr(98, 28, "N");
            u8g2.drawCircle(100, 45, 17);

            if (buffer1 == 0) {                     //Izpišemo vse podatke, ki smo jih prebrali
                u8g2.setFont(u8g2_font_6x10_mf);
                u8g2.drawStr(70, 10, "No Time");
            }
            if (buffer1 == 1) {
                u8g2.setFont(u8g2_font_6x10_mf);
                u8g2.setCursor(70, 10);
                u8g2.print(hour);
                u8g2.setCursor(92, 10);
                u8g2.print(minute);
                u8g2.setCursor(114, 10);
                u8g2.print(second);

                u8g2.drawStr(84, 10, ".");
                u8g2.drawStr(106, 10, ".");
            }
            if (buffer2 == 0) {
                u8g2.setFont(u8g2_font_4x6_mf);
                u8g2.drawStr(73, 18, "No Date");
            }
            if (buffer2 == 1) {
                u8g2.setFont(u8g2_font_4x6_mf);
                u8g2.setCursor(88, 18);
                u8g2.print(day);
                u8g2.setCursor(100, 18);
                u8g2.print(month);
                u8g2.setCursor(112, 18);
                u8g2.print(year);
                u8g2.drawStr(96, 18, "/");
                u8g2.drawStr(108, 18, "/");
            }
            if (buffer3 == 0) {
                u8g2.setFont(u8g2_font_6x10_mf);
                u8g2.drawStr(0, 30, "No GNSS FIX");
                u8g2.drawStr(0, 40, "Searching!");
            }
            if (buffer3 == 1) {
                u8g2.setFont(u8g2_font_4x6_mf);
                u8g2.drawStr(0, 18, "GNSS Locked! @");
                u8g2.setCursor(60, 18);
                u8g2.print(fixtype);
                u8g2.setFont(u8g2_font_6x10_mf);
                u8g2.drawStr(0, 30, "LAT:");
                u8g2.setCursor(30, 30);
                u8g2.print(gpsfloatLAT, 5);
                u8g2.drawStr(0, 40, "LON:");
                u8g2.setCursor(30, 40);
                u8g2.print(gpsfloatLON, 5);
                u8g2.drawStr(0, 50, "ALT:");
                u8g2.setCursor(30, 50);
                u8g2.drawStr(66, 50, "m");
                u8g2.print(gpsfloatALT, 1);
                u8g2.drawStr(0, 60, "SAT:");
                u8g2.setCursor(24, 60);
                u8g2.print(SIV);
                u8g2.drawStr(38, 60, "ACC:");
                u8g2.setCursor(62, 60);
                u8g2.print(gpsfloatACC, 0);

                if (buffer4 == 0) {
                    u8g2.drawCircle(100, 45, 14);
                }
                if (buffer4 == 1) {
                    u8g2.drawLine(x1, y1, x2, y2);      //Narišemo puščico
                    u8g2.drawLine(x1, y1, x3, y3);
                    u8g2.drawLine(x1, y1, x4, y4);
                }

                u8g2.setFont(u8g2_font_5x7_mf);
                u8g2.setCursor(((128 - (gpskmhlng)*5) / 2) + 36, 48);
                u8g2.print(gpskmh);
            }

            u8g2.sendBuffer();
        }
        GNSSisOFF = 0;
        reset(0, 250);
        u8g2.clearBuffer();
    }
}

/*SISTEMSKE INFORMACIJE
Samo prebere informacije o bateriji in ostale in jih izpiše na OLED.*/
void sys_info() {
    reset(1, 100);
    reset(0, 100);

    while ((counter == 0) && (onRcounter == 0)) {

        unsigned long currentMillis = millis();
        if (currentMillis - previousTime1 >= 5000) {
            u8g2.clearBuffer();
            u8g2.setFont(u8g2_font_6x10_mf);
            u8g2.drawStr(0, 10, "SYSTEM info:");        //Grafika
            u8g2.drawStr(0, 20, "Bat.Voltage:");
            u8g2.drawStr(96, 20, "|");
            u8g2.drawStr(120, 20, "%");
            u8g2.drawStr(90, 30, "T:");
            u8g2.drawStr(120, 30, "h");
            u8g2.drawStr(0, 30, "Current:");
            u8g2.drawStr(72, 30, "mA");
            u8g2.drawStr(0, 40, "GNSS:");
            u8g2.drawStr(80, 40, "Ts:");
            u8g2.drawStr(120, 40, "h");
            u8g2.drawStr(0, 50, "Timeout time:");
            u8g2.drawStr(0, 60, "Temp.:");

            batteryMV = axp.getBattVoltage(); // za baterijo
            batteryPercantage = ((batteryMV / 1000) - 3.25) / 0.009;

            whilecounter = millisTimeout(1000, whilecounter);   //Preberemo vse podatke in jih izpišemo
            u8g2.setCursor(72, 20);
            u8g2.print(axp.getBattVoltage(), 0);
            u8g2.setCursor(102, 20);
            u8g2.print(batteryPercantage, 0);

            if (axp.isChargeing() == 1) {
                u8g2.drawStr(48, 30, "+");
                u8g2.setCursor(54, 30);
                u8g2.print(axp.getBattChargeCurrent(), 0);
                u8g2.setCursor(102, 30);
                u8g2.print(((100 - batteryPercantage) * 30) / axp.getBattChargeCurrent(), 0);
            }
            if (axp.isChargeing() == 0) {
                u8g2.drawStr(48, 30, "-");
                u8g2.setCursor(54, 30);
                u8g2.print(axp.getBattDischargeCurrent(), 0);
                u8g2.setCursor(102, 30);
                u8g2.print((batteryPercantage * 30) / axp.getBattDischargeCurrent(), 0);
            }
            if (GNSSisOFF == 0) {
                u8g2.drawStr(33, 40, "Active");
            }
            if (GNSSisOFF == 1) {
                u8g2.drawStr(33, 40, "Asleep");
            }
            u8g2.setCursor(85, 50);
            u8g2.print(timeouttime);
            u8g2.drawStr(97, 50, "s");

            u8g2.setCursor(40, 60);
            u8g2.print(axp.getTemp(), 1);
            u8g2.drawStr(64, 60, "C");
            u8g2.setCursor(98, 40);
            u8g2.print((batteryPercantage * 30) / 17, 0);

            u8g2.sendBuffer();
            previousTime1 = currentMillis;
        }
    }
    u8g2.clearBuffer();
    reset(0, 200);
}

/*Funkcija menu,
    Izbira vsek pozivov oziroma funkciji ki so razpoložlive.
    Deluje na principu funkcije level().
 */

void menu() {
    reset(0, 0);
    reset(1, 200);

    u8g2.clearBuffer();

    batteryMV = axp.getBattVoltage();                           //preberemo baterijo
    batteryPercantage = ((batteryMV / 1000) - 3.25) / 0.009;    //izračunamo procente

    while (whilecounter < timeouttime) {
        m1 = back(m1);

        u8g2.sendBuffer();
        TITLE = "MENU";
        Screen1Row1 = "Pages";
        Screen1Row2 = "GNSS";
        Screen1Row3 = "Emergency";
        Screen1Row4 = "Ping";
        Screen2Row1 = "System info";
        Screen2Row2 = "Settings";
        Screen2Row3 = "Sleep";
        level();
        whilecounter = millisTimeout(1000, whilecounter);
        check(1);
        if (d == 0) {

            if (GNSSisOFF == 0) { // Za slikico od satelita
                u8g2.setBitmapMode(1);
                u8g2.drawXBM(1, 0, satlogo_w, satlogo_h, satlogo_bits);

                unsigned long currentMillis = millis();
                if (currentMillis - previousTime >= 5000) {
                    SIV = myGNSS.getSIV();
                    previousTime = currentMillis;
                }
                u8g2.setFont(u8g2_font_6x10_mf);
                u8g2.setCursor(16, 10);
                u8g2.print(SIV);
            }
            if (GNSSisOFF == 1) {
                u8g2.setBitmapMode(1);
                u8g2.drawXBM(1, 0, satlogoz_w, satlogoz_h, satlogoz_bits);
            }

            u8g2.drawLine(105, 1 + 2, 105 + 15, 1 + 2); // za slikico od baterije
            u8g2.drawLine(105, 8 + 2, 105 + 15, 8 + 2);
            u8g2.drawLine(105, 1 + 2, 105, 8 + 2);
            u8g2.drawLine(105 + 15, 1 + 2, 105 + 15, 2 + 2);
            u8g2.drawLine(105 + 15, 8 + 2, 105 + 15, 6 + 2);
            u8g2.drawLine(105 + 17, 3 + 2, 105 + 17, 6 + 2);
            u8g2.drawLine(105 + 15, 3 + 2, 105 + 17, 3 + 2);
            u8g2.drawLine(105 + 15, 6 + 2, 105 + 17, 6 + 2);
            u8g2.drawBox(105, 1 + 2, batteryPercantage * 0.15, 8);
            u8g2.drawBox(120, 3 + 2, 3, 3);

            if (axp.isChargeing() == 1) {
                u8g2.drawLine(103, 1 + 2, 98, 5 + 2);       //Če se polni
                u8g2.drawLine(98, 5 + 2, 103, 4 + 2);
                u8g2.drawLine(103, 4 + 2, 98, 8 + 2);
            }
            u8g2.sendBuffer();
        }

        if (counter == 1) {
            counter = 0;

            switch (selecta) {

            case 1:
                reset(1, 200);
                reset(0, 0);
                u8g2.clearBuffer();

                while ((whilecounter < timeouttime) && (m1 == 0)) {
                    m2 = back(m2);

                    whilecounter = millisTimeout(1000, whilecounter);
                    TITLE = "Tactical";
                    Screen1Row1 = "Where are you?";
                    Screen1Row2 = "Physical status?";
                    Screen1Row3 = "Situation?";
                    Screen1Row4 = "Supplies?";
                    Screen2Row1 = "Hazards?";
                    Screen2Row2 = "Phone Signal?";
                    Screen2Row3 = "RSSI?";
                    Screen2Row4 = "back";

                    level();
                    check(1);
                    if (counter == 1) {
                        counter = 0;
                        switch (selecta) {
                        case 1:
                            reset(1, 200);
                            reset(0, 0);
                            u8g2.clearBuffer();

                            while ((whilecounter < timeouttime) && (m2 == 0)) {
                                m3 = back(m3);
                                whilecounter = millisTimeout(1000, whilecounter);
                                TITLE = "Where are you?";
                                Screen1Row1 = "Send Question!";
                                Screen1Row2 = "GNSS Unavailable";
                                Screen1Row3 = "Headings";
                                Screen1Row4 = "Biomes";
                                Screen2Row1 = "Near Camp";
                                Screen2Row2 = "Landmarks";
                                Screen2Row3 = "back";

                                level();
                                check(1);
                                if (counter == 1) {
                                    counter = 0;

                                    switch (selecta) {
                                    case 1:
                                        pages(3);
                                        break;
                                    case 2:
                                        reset(1, 200);
                                        reset(0, 0);
                                        u8g2.clearBuffer();

                                        while ((whilecounter < timeouttime) && (m3 == 0)) {
                                            whilecounter = millisTimeout(1000, whilecounter);
                                            TITLE = "GNSS Unavailable";
                                            Screen1Row1 = "Somewhere N";
                                            Screen1Row2 = "Somewhere NE";
                                            Screen1Row3 = "Somewhere E";
                                            Screen1Row4 = "Somewhere SE";
                                            Screen2Row1 = "Somewhere S";
                                            Screen2Row2 = "Somewhere SW";
                                            Screen2Row3 = "Somewhere W";
                                            Screen2Row4 = "Somewhere NW";
                                            Screen2Row5 = "Peak";
                                            Screen2Row6 = "back";

                                            level();
                                            check(1);
                                            if (counter == 1) {
                                                counter = 0;

                                                switch (selecta) {
                                                case 1:
                                                    pages(4);
                                                    break;
                                                case 2:
                                                    pages(5);
                                                    break;
                                                case 3:
                                                    pages(6);
                                                    break;
                                                case 4:
                                                    pages(7);
                                                    break;
                                                case 5:
                                                    pages(8);
                                                    break;
                                                case 6:
                                                    pages(9);
                                                    break;
                                                case 7:
                                                    pages(10);
                                                    break;
                                                case 8:
                                                    pages(11);
                                                    break;
                                                case 9:
                                                    pages(12);
                                                    break;
                                                case 10:
                                                    m3 = 1;
                                                    reset(1, 200);
                                                    u8g2.clearBuffer();
                                                    break;
                                                default:
                                                    break;
                                                }
                                            }
                                        }
                                        break;
                                    case 3:
                                        reset(1, 200);
                                        reset(0, 0);
                                        u8g2.clearBuffer();

                                        while ((whilecounter < timeouttime) && (m3 == 0)) {
                                            whilecounter = millisTimeout(1000, whilecounter);
                                            TITLE = "Headings";
                                            Screen1Row1 = "Right Behind You";
                                            Screen1Row2 = "Heading N";
                                            Screen1Row3 = "Headnig NE";
                                            Screen1Row4 = "Heading E";
                                            Screen2Row1 = "Headnig SE";
                                            Screen2Row2 = "Heading S";
                                            Screen2Row3 = "Headnig SW";
                                            Screen2Row4 = "Heading W";
                                            Screen2Row5 = "Heading NW";
                                            Screen2Row6 = "Heading Peak";
                                            Screen3Row1 = "Heading Upstream";
                                            Screen3Row2 = "Heading Downstream";
                                            Screen3Row3 = "back";

                                            level();
                                            check(1);
                                            if (counter == 1) {
                                                counter = 0;

                                                switch (selecta) {
                                                case 1:
                                                    pages(13);
                                                    break;
                                                case 2:
                                                    pages(14);
                                                    break;
                                                case 3:
                                                    pages(15);
                                                    break;
                                                case 4:
                                                    pages(16);
                                                    break;
                                                case 5:
                                                    pages(17);
                                                    break;
                                                case 6:
                                                    pages(18);
                                                    break;
                                                case 7:
                                                    pages(19);
                                                    break;
                                                case 8:
                                                    pages(20);
                                                    break;
                                                case 9:
                                                    pages(21);
                                                    break;
                                                case 10:
                                                    pages(22);
                                                    break;
                                                case 11:
                                                    pages(23);
                                                    break;
                                                case 12:
                                                    pages(24);
                                                    break;
                                                case 13:
                                                    m3 = 1;
                                                    reset(1, 200);
                                                    u8g2.clearBuffer();
                                                    break;
                                                }
                                            }
                                        }

                                        break;
                                    case 4:
                                        reset(1, 200);
                                        reset(0, 0);
                                        u8g2.clearBuffer();

                                        while ((whilecounter < timeouttime) && (m3 == 0)) {
                                            whilecounter = millisTimeout(1000, whilecounter);
                                            TITLE = "BIOMES";
                                            Screen1Row1 = "Mountain Side";
                                            Screen1Row2 = "Grass Field";
                                            Screen1Row3 = "Spruce Forest";
                                            Screen1Row4 = "Birch Forest";
                                            Screen2Row1 = "Beech Forest";
                                            Screen2Row2 = "Oak Forest";
                                            Screen2Row3 = "Pine Forest";
                                            Screen2Row4 = "Linden Forest";
                                            Screen2Row5 = "Hazel Forest";
                                            Screen2Row6 = "Low Hills";
                                            Screen3Row1 = "Medium Hills";
                                            Screen3Row2 = "Tall Hills";
                                            Screen3Row3 = "V Valley";
                                            Screen3Row4 = "U Valley";
                                            Screen3Row4 = "Canyon";

                                            level();
                                            check(1);
                                            if (counter == 1) {
                                                counter = 0;

                                                switch (selecta) {
                                                case 1:
                                                    pages(25);
                                                    break;
                                                case 2:
                                                    pages(26);
                                                    break;
                                                case 3:
                                                    pages(27);
                                                    break;
                                                case 4:
                                                    pages(28);
                                                    break;
                                                case 5:
                                                    pages(29);
                                                    break;
                                                case 6:
                                                    pages(30);
                                                    break;
                                                case 7:
                                                    pages(31);
                                                    break;
                                                case 8:
                                                    pages(32);
                                                    break;
                                                case 9:
                                                    pages(33);
                                                    break;
                                                case 10:
                                                    pages(34);
                                                    break;
                                                case 11:
                                                    pages(35);
                                                    break;
                                                case 12:
                                                    pages(36);
                                                    break;
                                                case 13:
                                                    m3 = 1;
                                                    reset(1, 200);
                                                    u8g2.clearBuffer();
                                                    break;
                                                }
                                            }
                                        }

                                        break;
                                    case 5:
                                        reset(1, 200);
                                        reset(0, 0);
                                        u8g2.clearBuffer();

                                        while ((whilecounter < timeouttime) && (m3 == 0)) {
                                            whilecounter = millisTimeout(1000, whilecounter);
                                            TITLE = "NEAR CAMP";
                                            Screen1Row1 = "Camp Visible";
                                            Screen1Row2 = "500 from Camp";
                                            Screen1Row3 = "1k From Camp";
                                            Screen1Row4 = "2k From Camp";
                                            Screen2Row1 = "5k From";
                                            Screen2Row2 = "Beech Forest";
                                            Screen2Row3 = "Oak Forest";
                                            Screen2Row4 = "Pine Forest";
                                            Screen2Row5 = "Linden Forest";
                                            Screen2Row6 = "Hazel Forest";
                                            Screen3Row1 = "Low Hills";
                                            Screen3Row2 = "Medium Hills";
                                            Screen3Row3 = "Tall Hills";
                                            Screen3Row4 = "V Valley";
                                            Screen3Row4 = "U Valley";
                                            Screen3Row5 = "Canyon";

                                            level();
                                            check(1);
                                            if (counter == 1) {
                                                counter = 0;

                                                switch (selecta) {
                                                case 1:
                                                    pages(25);
                                                    break;
                                                case 2:
                                                    pages(26);
                                                    break;
                                                case 3:
                                                    pages(27);
                                                    break;
                                                case 4:
                                                    pages(28);
                                                    break;
                                                case 5:
                                                    pages(29);
                                                    break;
                                                case 6:
                                                    pages(30);
                                                    break;
                                                case 7:
                                                    pages(31);
                                                    break;
                                                case 8:
                                                    pages(32);
                                                    break;
                                                case 9:
                                                    pages(33);
                                                    break;
                                                case 10:
                                                    pages(34);
                                                    break;
                                                case 11:
                                                    pages(35);
                                                    break;
                                                case 12:
                                                    pages(36);
                                                    break;
                                                case 13:
                                                    m3 = 1;
                                                    reset(1, 200);
                                                    u8g2.clearBuffer();
                                                    break;
                                                }
                                            }
                                        }

                                        break;
                                    case 6:
                                        reset(1, 200);
                                        reset(0, 0);
                                        u8g2.clearBuffer();

                                        while ((whilecounter < timeouttime) && (m3 == 0)) {
                                            whilecounter = millisTimeout(1000, whilecounter);
                                            TITLE = "LANDMARKS";
                                            Screen1Row1 = "Near Church";
                                            Screen1Row2 = "Near Sinkhole";
                                            Screen1Row3 = "Near Ruins";
                                            Screen1Row4 = "Near Lake";
                                            Screen2Row1 = "Near Sea";
                                            Screen2Row2 = "Near River";
                                            Screen2Row3 = "River in Eyesight";
                                            Screen2Row4 = "River 100m";
                                            Screen2Row5 = "River 500m";
                                            Screen2Row6 = "back";

                                            level();
                                            check(1);
                                            if (counter == 1) {
                                                counter = 0;

                                                switch (selecta) {
                                                case 1:
                                                    pages(46);
                                                    break;
                                                case 2:
                                                    pages(47);
                                                    break;
                                                case 3:
                                                    pages(48);
                                                    break;
                                                case 4:
                                                    pages(49);
                                                    break;
                                                case 5:
                                                    pages(50);
                                                    break;
                                                case 6:
                                                    pages(51);
                                                    break;
                                                case 7:
                                                    pages(52);
                                                    break;
                                                case 8:
                                                    pages(53);
                                                    break;
                                                case 9:
                                                    pages(54);
                                                    break;
                                                case 10:
                                                    m3 = 1;
                                                    reset(1, 200);
                                                    u8g2.clearBuffer();
                                                    break;
                                                }
                                            }
                                        }

                                        break;
                                    case 7:
                                        m2 = 1;
                                        reset(1, 200);
                                        u8g2.clearBuffer();
                                        break;
                                    }
                                }
                            }
                            break;
                        case 2:
                            reset(1, 200);
                            reset(0, 0);
                            u8g2.clearBuffer();

                            while ((whilecounter < timeouttime) && (m2 == 0)) {
                                m3 = back(m3);
                                whilecounter = millisTimeout(1000, whilecounter);
                                TITLE = "PHYSICAL STATUS?";
                                Screen1Row1 = "Send Question!";
                                Screen1Row2 = "Great";
                                Screen1Row3 = "Good";
                                Screen1Row4 = "Okay";
                                Screen2Row1 = "Hunger";
                                Screen2Row2 = "Thirst";
                                Screen2Row3 = "Tired";
                                Screen2Row4 = "Hurt";
                                Screen2Row5 = "back";

                                level();
                                check(1);
                                if (counter == 1) {
                                    counter = 0;

                                    switch (selecta) {
                                    case 1:
                                        pages(55);
                                        break;
                                    case 2:
                                        pages(56);
                                        break;
                                    case 3:
                                        pages(57);
                                        break;
                                    case 4:
                                        pages(58);
                                        break;
                                    case 5:
                                        reset(1, 200);
                                        reset(0, 0);
                                        u8g2.clearBuffer();
                                        while ((whilecounter < timeouttime) && (m3 == 0)) {
                                            whilecounter = millisTimeout(1000, whilecounter);
                                            TITLE = "HUNGER";
                                            Screen1Row1 = "Little hungry?";
                                            Screen1Row2 = "Mod. Hunger";
                                            Screen1Row3 = "Very Hungry";
                                            Screen1Row4 = "Criticle Starving";
                                            Screen1Row4 = "back";

                                            level();
                                            check(1);
                                            if (counter == 1) {
                                                counter = 0;

                                                switch (selecta) {
                                                case 1:
                                                    pages(59);
                                                    break;
                                                case 2:
                                                    pages(60);
                                                    break;
                                                case 3:
                                                    pages(61);
                                                    break;
                                                case 4:
                                                    pages(62);
                                                    break;
                                                case 5:
                                                    m3 = 1;
                                                    reset(1, 200);
                                                    u8g2.clearBuffer();
                                                    break;
                                                }
                                            }
                                        }

                                        break;
                                    case 6:
                                        reset(1, 200);
                                        reset(0, 0);
                                        u8g2.clearBuffer();
                                        while ((whilecounter < timeouttime) && (m3 == 0)) {
                                            whilecounter = millisTimeout(1000, whilecounter);
                                            TITLE = "THIRSTY";
                                            Screen1Row1 = "Little Thirsty";
                                            Screen1Row2 = "Mod. Thirsty";
                                            Screen1Row3 = "Very Thirsty";
                                            Screen1Row4 = "Critical Thirsty";
                                            Screen2Row1 = "back";

                                            level();
                                            check(1);
                                            if (counter == 1) {
                                                counter = 0;

                                                switch (selecta) {
                                                case 1:
                                                    pages(63);
                                                    break;
                                                case 2:
                                                    pages(64);
                                                    break;
                                                case 3:
                                                    pages(65);
                                                    break;
                                                case 4:
                                                    pages(66);
                                                    break;
                                                case 10:
                                                    m3 = 1;
                                                    reset(1, 200);
                                                    u8g2.clearBuffer();
                                                    break;
                                                }
                                            }
                                        }

                                        break;
                                    case 7:
                                        reset(1, 200);
                                        reset(0, 0);
                                        u8g2.clearBuffer();
                                        while ((whilecounter < timeouttime) && (m3 == 0)) {
                                            whilecounter = millisTimeout(1000, whilecounter);
                                            TITLE = "TIRED";
                                            Screen1Row1 = "Tired";
                                            Screen1Row2 = "Very Tired";
                                            Screen1Row3 = "Fatigued";
                                            Screen1Row4 = "back";

                                            level();
                                            check(1);
                                            if (counter == 1) {
                                                counter = 0;

                                                switch (selecta) {
                                                case 1:
                                                    pages(67);
                                                    break;
                                                case 2:
                                                    pages(68);
                                                    break;
                                                case 3:
                                                    pages(69);
                                                    break;
                                                case 4:
                                                    m3 = 1;
                                                    reset(1, 200);
                                                    u8g2.clearBuffer();
                                                    break;
                                                }
                                            }
                                        }
                                        break;
                                    case 8:
                                        reset(1, 200);
                                        reset(0, 0);
                                        u8g2.clearBuffer();
                                        while ((whilecounter < timeouttime) && (m3 == 0)) {
                                            whilecounter = millisTimeout(1000, whilecounter);
                                            TITLE = "HURT";
                                            Screen1Row1 = "Lightly Hurt";
                                            Screen1Row2 = "Mod. Hurt";
                                            Screen1Row3 = "Very Hurt";
                                            Screen1Row4 = "Broken Bone";
                                            Screen2Row1 = "Sprained Joint";
                                            Screen2Row2 = "Light Bleeding";
                                            Screen2Row3 = "Mod. Bleeding";
                                            Screen2Row4 = "Critical Bleed.";
                                            Screen2Row5 = "Immobile";
                                            Screen2Row6 = "Breedable";
                                            Screen3Row1 = "back";

                                            level();
                                            check(1);
                                            if (counter == 1) {
                                                counter = 0;

                                                switch (selecta) {
                                                case 1:
                                                    pages(70);
                                                    break;
                                                case 2:
                                                    pages(71);
                                                    break;
                                                case 3:
                                                    pages(72);
                                                    break;
                                                case 4:
                                                    pages(73);
                                                    break;
                                                case 5:
                                                    pages(74);
                                                    break;
                                                case 6:
                                                    pages(75);
                                                    break;
                                                case 7:
                                                    pages(76);
                                                    break;
                                                case 8:
                                                    pages(77);
                                                    break;
                                                case 9:
                                                    pages(78);
                                                    break;
                                                case 10:
                                                    pages(79);
                                                    break;
                                                case 11:
                                                    m3 = 1;
                                                    reset(1, 200);
                                                    u8g2.clearBuffer();
                                                    break;
                                                }
                                            }
                                        }
                                        break;
                                    case 9:
                                        m2 = 1;
                                        reset(1, 200);
                                        u8g2.clearBuffer();
                                        break;
                                    }
                                }
                            }
                            break;
                        case 3:
                            reset(1, 200);
                            reset(0, 0);
                            u8g2.clearBuffer();
                            while ((whilecounter < timeouttime) && (m2 == 0)) {
                                m3 = back(m3);
                                whilecounter = millisTimeout(1000, whilecounter);
                                TITLE = "SITUATION?";
                                Screen1Row1 = "Send Question!";
                                Screen1Row2 = "Falling Back";
                                Screen1Row3 = "In Position";
                                Screen1Row4 = "Not Ready";
                                Screen2Row1 = "Idling";
                                Screen2Row2 = "Busy";
                                Screen2Row3 = "Compleated";
                                Screen2Row4 = "In Progress";
                                Screen2Row5 = "Setting up Camp";
                                Screen2Row6 = "Get.R. for Dark";
                                Screen3Row1 = "Setting up Fire";
                                Screen3Row2 = "Following";
                                Screen3Row3 = "back";

                                level();
                                check(1);
                                if (counter == 1) {
                                    counter = 0;

                                    switch (selecta) {
                                    case 1:
                                        pages(80);
                                        break;
                                    case 2:
                                        pages(81);
                                        break;
                                    case 3:
                                        pages(82);
                                        break;
                                    case 4:
                                        pages(83);
                                        break;
                                    case 5:
                                        pages(84);
                                        break;
                                    case 6:
                                        pages(85);
                                        break;
                                    case 7:
                                        pages(86);
                                        break;
                                    case 8:
                                        pages(87);
                                        break;
                                    case 9:
                                        pages(88);
                                        break;
                                    case 10:
                                        pages(89);
                                        break;
                                    case 11:
                                        pages(90);
                                        break;
                                    case 12:
                                        pages(91);
                                        break;
                                    case 13:
                                        m2 = 1;
                                        reset(1, 200);
                                        u8g2.clearBuffer();
                                        break;
                                    }
                                }
                            }
                            break;
                        case 4:
                            reset(1, 200);
                            reset(0, 0);
                            u8g2.clearBuffer();
                            while ((whilecounter < timeouttime) && (m2 == 0)) {
                                m3 = back(m3);
                                whilecounter = millisTimeout(1000, whilecounter);
                                TITLE = "SUPPLIES?";
                                Screen1Row1 = "Send Question!";
                                Screen1Row2 = "Supp. Surplus";
                                Screen1Row3 = "Supp. Good";
                                Screen1Row4 = "Supp. Lack";
                                Screen2Row1 = "Supp. Insufficient";
                                Screen2Row2 = "back";

                                level();
                                check(1);
                                if (counter == 1) {
                                    counter = 0;

                                    switch (selecta) {
                                    case 1:
                                        pages(92);
                                        break;
                                    case 2:
                                        pages(93);
                                        break;
                                    case 3:
                                        pages(94);
                                        break;
                                    case 4:
                                        pages(95);
                                        break;
                                    case 5:
                                        pages(96);
                                        break;
                                    case 6:
                                        m2 = 1;
                                        reset(1, 200);
                                        u8g2.clearBuffer();
                                        break;
                                    }
                                }
                            }
                            break;
                        case 5:
                            reset(1, 200);
                            reset(0, 0);
                            u8g2.clearBuffer();
                            while ((whilecounter < timeouttime) && (m2 == 0)) {
                                m3 = back(m3);
                                whilecounter = millisTimeout(1000, whilecounter);
                                TITLE = "HAZARDS?";
                                Screen1Row1 = "Send Question!";
                                Screen1Row2 = "Fire Hazard";
                                Screen1Row3 = "Water Hazard";
                                Screen1Row4 = "Gas Hazzard";
                                Screen2Row1 = "RadioA. Hazzard";
                                Screen2Row2 = "Height Hazzard";
                                Screen2Row3 = "Contamination";
                                Screen2Row4 = "back";

                                level();
                                check(1);
                                if (counter == 1) {
                                    counter = 0;

                                    switch (selecta) {
                                    case 1:
                                        pages(97);
                                        break;
                                    case 2:
                                        pages(98);
                                        break;
                                    case 3:
                                        pages(99);
                                        break;
                                    case 4:
                                        pages(100);
                                        break;
                                    case 5:
                                        pages(101);
                                        break;
                                    case 6:
                                        pages(102);
                                        break;
                                    case 7:
                                        pages(103);
                                        break;
                                    case 8:
                                        m2 = 1;
                                        reset(1, 200);
                                        u8g2.clearBuffer();
                                        break;
                                    }
                                }
                            }
                            break;
                        case 6:
                            reset(1, 200);
                            reset(0, 0);
                            u8g2.clearBuffer();
                            while ((whilecounter < timeouttime) && (m2 == 0)) {
                                m3 = back(m3);
                                whilecounter = millisTimeout(1000, whilecounter);
                                TITLE = "PHONE SIGNAL?";
                                Screen1Row1 = "Send Question!";
                                Screen1Row2 = "Signal Strong";
                                Screen1Row3 = "Signal Mod.";
                                Screen1Row4 = "Signal Weak";
                                Screen2Row1 = "No P. Signal";
                                Screen2Row2 = "back";

                                level();
                                check(1);
                                if (counter == 1) {
                                    counter = 0;

                                    switch (selecta) {
                                    case 1:
                                        pages(104);
                                        break;
                                    case 2:
                                        pages(105);
                                        break;
                                    case 3:
                                        pages(106);
                                        break;
                                    case 4:
                                        pages(107);
                                        break;
                                    case 5:
                                        pages(108);
                                        break;
                                    case 6:
                                        m2 = 1;
                                        reset(1, 200);
                                        u8g2.clearBuffer();
                                        break;
                                    }
                                }
                            }
                            break;
                        case 7:
                            reset(1, 200);
                            reset(0, 0);
                            u8g2.clearBuffer();
                            while ((whilecounter < timeouttime) && (m2 == 0)) {
                                m3 = back(m3);
                                whilecounter = millisTimeout(1000, whilecounter);
                                TITLE = "RSSI?";
                                Screen1Row1 = "Send Question!";
                                Screen1Row2 = "RSSI < 50";
                                Screen1Row3 = "RSSI 50 - 100";
                                Screen1Row4 = "RSSI 100 - 110";
                                Screen2Row1 = "RSSI 110 - 120";
                                Screen2Row2 = "RSSI 120 - 130";
                                Screen2Row3 = "RSSI > 130";
                                Screen2Row2 = "back";

                                level();
                                check(1);
                                if (counter == 1) {
                                    counter = 0;

                                    switch (selecta) {
                                    case 1:
                                        pages(109);
                                        break;
                                    case 2:
                                        pages(110);
                                        break;
                                    case 3:
                                        pages(111);
                                        break;
                                    case 4:
                                        pages(112);
                                        break;
                                    case 5:
                                        pages(113);
                                        break;
                                    case 6:
                                        pages(114);
                                        break;
                                    case 7:
                                        pages(115);
                                        break;
                                    case 8:
                                        m2 = 1;
                                        reset(1, 200);
                                        u8g2.clearBuffer();
                                        break;
                                    }
                                }
                            }
                            break;

                        case 8:
                            m1 = 1;
                            reset(1, 200);
                            u8g2.clearBuffer();
                            break;
                        }
                    }
                }
                break;
            case 2:
                reset(1, 0);
                reset(0, 0);
                u8g2.clearBuffer();
                while ((whilecounter < timeouttime) && (m1 == 0)) {

                    TITLE = "GNSS";
                    if (GNSSisOFF == 0) {
                        Screen1Row1 = "GNSS Stats?";
                    }
                    if (GNSSisOFF == 1) {
                        Screen1Row1 = "Wake GNSS?";
                    }
                    Screen1Row2 = "Send GNSS?";
                    Screen1Row3 = "back";

                    whilecounter = millisTimeout(1000, whilecounter);
                    level();
                    check(1);

                    if (counter == 1) {
                        counter = 0;
                        switch (selecta) {
                        case 1:
                            GNSSinfo();
                            delay(500);
                            break;
                        case 2:
                            if (GNSSisOFF == 1) {
                                break;
                            }
                            if (GNSSisOFF == 0) {
                                if (myGNSS.getGnssFixOk() == 1) {
                                    u8g2.clearBuffer();
                                    u8g2.setFont(u8g2_font_6x10_mf);
                                    u8g2.drawStr(0, 10, "sending...");
                                    u8g2.sendBuffer();
                                    LoRa_GNSS_Parser(0, myGNSS.getLatitude(), myGNSS.getLongitude(), myGNSS.getAltitude(), 0);
                                    u8g2.clearBuffer();
                                    u8g2.setFont(u8g2_font_6x10_mf);
                                    u8g2.drawStr(0, 10, "ok...");
                                    u8g2.sendBuffer();
                                    delay(500);
                                }
                                if (myGNSS.getGnssFixOk() == 0) {
                                }
                            }
                        case 3:
                            m1 = 1;
                            u8g2.clearBuffer();
                            reset(1, 200);
                            break;

                        default:
                            break;
                        }
                    }
                }
                break;
            case 3:
                emergency_beacon();
                break;
            case 4:
                ping();
                break;
            case 5:
                sys_info();
                break;
            case 6:
                settings();
                break;
            case 7:
                whilecounter = timeouttime;
                break;
            }
        }
    }
    reset(1, 100);
}

/*GLAVNA ZANKA
Odgovorna za delovanje vsega. Se izvaja repetativno. Vse zgornje funkcije se izvedejo pod glavno zanko.
 */
void loop() {
    u8g2.setPowerSave(0);
    u8g2.clearDisplay();
    u8g2.clearBuffer();
    delay(500);

    while ((counter != 0) || (onRcounter != 0)) {       //Preverimo gdo je zbudil ESP, a smo sprejeli poziv ali pritisnili tipko
        if ((counter == 1) && (onRcounter == 0)) {      //Se izvrši če smo prritisnili tipko
            fdecl = 1;
            menu();
        } else if ((counter == 0) && (onRcounter == 1)) {   //V drugem primeru se izvrši ta, če smo sprejeli poziv
            reset(0, 100);
            fdecl = 0;
            if (MSGlenght > 5) {
                while (whilecounter < timeouttime) {
                    whilecounter = millisTimeout(1000, whilecounter);

                    if ((whilecounter % 2) == 0) {
                        tone(piezo, 4000, 250);
                    }
                    u8g2.clearBuffer();
                    u8g2.setFont(u8g2_font_7x13_mf);
                    u8g2.drawStr(0, 10, "RECIEVING GPS");   //Izpišemo GNSS podatke, če smo jih sprejeli.
                    u8g2.setFont(u8g2_font_6x10_mf);
                    u8g2.drawStr(0, 30, "LAT:");
                    u8g2.drawStr(0, 40, "LON:");
                    u8g2.drawStr(0, 50, "ALT:");
                    u8g2.drawStr(0, 60, "RSSI:");
                    u8g2.setCursor(25, 30);
                    u8g2.print(LoRa_GNSS_Parser(1, 0, 0, 0, 1), 5);
                    u8g2.setCursor(25, 40);
                    u8g2.print(LoRa_GNSS_Parser(1, 0, 0, 0, 2), 5);
                    u8g2.setCursor(25, 50);
                    u8g2.print(LoRa_GNSS_Parser(1, 0, 0, 0, 3), 5);
                    u8g2.setCursor(32, 60);
                    u8g2.print(RSSI);
                }
            }
            u8g2.clearBuffer();
            pages(MSG);
            reset(0, 100);
        }
    }

    u8g2.clearDisplay();
    u8g2.clearBuffer();
    u8g2.setPowerSave(1);
    if ((GNSSisOFF == 0) && (GNSSsleepSETTIING != 3)) {
        myGNSS.powerOffWithInterrupt(0, VAL_RXM_PMREQ_WAKEUPSOURCE_UARTRX);     //Damo GNSS čip v spanje, če ni v spanju
        GNSSisOFF = 1;
    }
    delay(100);
    esp_light_sleep_start();    //Začnemo lahko esp32 spanje. Naša poraba na sprejemu nam pade na 16mA
}