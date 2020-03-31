
#include <mcp_can.h>
#include <SPI.h>
#include "Servo.h"
// #include <HardwareSerial.h>
// #include <Serial.h>

// SPI :
// pin 49: Digital Pot (logique inversé)
// pin 50: MISO
// pin 51: MOSI
// pin 52: SCLK
// pin 53 Can Bus (logique inversé)
#define PIN_SPI_CS_DP 49
#define PIN_SPI_CS_CAN 53

//Can Bus
// Pin 2: interuption (données disponible)

#define PIN_CB_DATA 2
#define CB_ID_FP 0x00
#define CB_ADDRESS_MA 0x00
#define CB_ADDRESS_FP 0x00
#define CB_ID_BAT 0x00
#define CB_ADDRESS_BAT 0x00

// Commande au volant
// pin 48: A1
// pin 47: A2
// pin 46: A3
// pin 45: B1
// pin 44: B2
// pin 43: B3

// B1 => A3 Bouton Bas
// B1 => B3 Vol -
// B1 => A1 Vol +
// B2 => A3 Btn haut droit
// B2 => B3 Btn haut gauche
// A2 => A3 Molette Cran 1  A2 = blanc
// A2 => B3 Molette Cran 2
// A2 => A1 Molette Cran 3

#define CV_A1 48 // input = Vol + / Molette 3
#define CV_A2 47 // Output =  Molette 1 2 3
#define CV_A3 46 // input = Btn bas / Btn Haut Droite / molette 1
#define CV_B1 45 // Output = Btn bas / vol + / Vol +
#define CV_B2 44 // Output = Btn Haut droite / Btn haut gauche
#define CV_B3 43 // input = Vol + / Molette 3

#define NONE 0
#define CV_BTN_DOWN 1
#define CV_VOL_DOWN 2
#define CV_VOL_UP 4
#define CV_BTN_UP_RGT 8
#define CV_BTN_UP_LFT 16
#define CV_MOL_1 64
#define CV_MOL_2 128
#define CV_MOL_3 256
#define CV_VOL_UP_DOWN 6
#define CV_BTN_UP_LFT_BTN_DOWN 9
#define CV_BTN_UP_LFT_RGT 24
#define CV_MASK_CURSOR 1 + 2 + 4
#define CV_MASK_MOL 64 + 128 + 256
#define CV_MASK_ALL 0xFFFF

// commande autoradio
// pin 42: relais Tip
// pin 41: relais Ring

#define AR_TIP 42
#define AR_RING 41

// source / 2 sec = off => 1.2
// MUTE => 3.5
// display => 5.75
// next => 8
// prev => 11.25
// Vol up => 16
// Vol down => 24
// band => 62.7

#define AR_NEED_RING 256

#define NONE 0
#define AR_SOURCE 3
#define AR_MUTE 9
#define AR_DISPLAY 14
#define AR_NEXT 20
#define AR_PREV 28
#define AR_VOL_UP 41
#define AR_VOL_DOWN 61
#define AR_BAND 160
#define AR_PRESET_UP 20 + AR_NEED_RING
#define AR_PRESET_DOWN 28 + AR_NEED_RING

#define POT0_SEL 0x11
#define POT1_SEL 0x12
#define BOTH_POT_SEL 0x13

#define POT0_SHUTDOWN 0x21
#define POT1_SHUTDOWN 0x22
#define BOTH_POT_SHUTDOWN 0x23

// pin 40: relais frein de parking
// pin 39: relais marche arriere

#define PIN_AR_FP 40
#define PIN_AR_MA 39

//retroviseur
// pin 38: relais ouverture retro
// pin 37: relais fermeture retro

#define PIN_RV_OPEN 38
#define PIN_RV_CLOSE 37
#define RV_TIME 3000 //temp d ouverture et fermeture du retroviseur en ms

//Trape autoradio
// pin 4 (PWM)
// pin 35 relais alime servo

#define PIN_TRAP_PWM 4
#define PIN_TRAP_ALIM 35

#define TRAP_STOP 45
#define TRAP_SPEED 45
#define TRAP_TIME 1000

#define TRAP_UP TRAP_STOP + TRAP_SPEED
#define TRAP_DOWN TRAP_STOP - TRAP_SPEED

//resistance chauffante
// pin 38 : relais
// pin 37 : relais
// pin 36 : relais

#define PIN_RC_1 38
#define PIN_RC_23 37
#define PIN_RC_4 36

#define RC_OFF 0
#define RC_1 1
#define RC_23 2
#define RC_4 4
#define RC_14 5
#define RC_ALL 7

#define DEBOUNCE 20
#define LONG_DEBOUNCE 80

inline bool Debounce(unsigned long time, unsigned long debounce = DEBOUNCE)
{
  return (time + debounce) < millis() ? true : false;
}

//gestion de la commande au volant
//la methode setup() doit etre appele dans la fonction d initialisation
//la methode  loop() doit etre appele dans chaque iteration de le fonction loop()
class CAV
{
public:
  String data;
  //constructeur
  CAV()
  {
    stats = 0;
    cursor = 0;
    debounce = 0;
  };

  //methode d intitialisation a apeler dans setup()
  void setup()
  {
    pinMode(CV_B1, OUTPUT); // set pin to output
    pinMode(CV_B2, OUTPUT); // set pin to output
    pinMode(CV_A2, OUTPUT); // set pin to output
    pinMode(CV_A1, INPUT);  // set pin to input
    pinMode(CV_A3, INPUT);  // set pin to input
    pinMode(CV_B3, INPUT);  // set pin to input

    digitalWrite(CV_A1, HIGH); // turn on pullup resistors /!\ logique inverse
    digitalWrite(CV_A3, HIGH); // turn on pullup resistors /!\ logique inverse
    digitalWrite(CV_B3, HIGH); // turn on pullup resistors /!\ logique inverse

    digitalWrite(CV_B1, HIGH); // /!\ logique inverse
    digitalWrite(CV_B2, HIGH); // /!\ logique inverse
    digitalWrite(CV_A2, HIGH); // /!\ logique inverse
    cursor = 0;
  }

  //methode de traitement a appeler dans chaque iteration de la boucle principal
  int loop()
  {

    static int old_stats = 0;
    bool a1 = !digitalRead(CV_A1);
    bool a3 = !digitalRead(CV_A3);
    bool b3 = !digitalRead(CV_B3);

    // Serial.print("a1 = ");
    // Serial.println(a1);
    // Serial.print("a3 = ");
    // Serial.println(a3);
    // Serial.print("b3 = ");
    // Serial.println(b3);

    // Serial.println("");
    int loop_stats;

    loop_stats = a1 ? 1 : 0;
    // Serial.print("loop_stats = ");
    // Serial.println(loop_stats);
    loop_stats = a3 ? loop_stats + 2 : loop_stats;
    // Serial.print("loop_stats = ");
    // Serial.println(loop_stats);
    loop_stats = b3 ? loop_stats + 4 : loop_stats;

    loop_stats = loop_stats << cursor * 3;
    // Serial.print("loop_stats = ");
    // Serial.println(loop_stats);

    int sup = CV_MASK_CURSOR << cursor * 3;
    // sup = 0b1111111111 ^ sup;
    sup = 0b111111111 ^ sup;
    // Serial.print("sup = ");
    // Serial.println(sup);
    stats = stats & sup;
    // Serial.print("stats = ");
    // Serial.println(stats);
    stats = stats | loop_stats;

    if (stats != old_stats)
    {
      debounce = millis();
      old_stats = stats;
    }

    cursor = cursor == 2 ? 0 : cursor + 1;
    switch (cursor)
    {
    case 0: //CV_B1 (BTN_DOWN / VOL_DOWN / VOL_UP)

      Disable(CV_B2);
      Disable(CV_A2);
      UsePin(CV_B1);
      break;
    case 1: //CV_B2 (BTN_UP_RGT / BTN_UP_LFT)

      Disable(CV_A2);
      Disable(CV_B1);
      UsePin(CV_B2);
      break;
    case 2: //CV_A3 (molette)

      Disable(CV_B1);
      Disable(CV_B2);
      UsePin(CV_A2);
    }
  }

  // recuperation de l etat de l'ensemble des boutons avec anti-rebond
  int get_stats()
  {
    return Debounce(debounce, DEBOUNCE) ? stats : NONE;
  }

  int get_long_stats()
  {
    return Debounce(debounce, LONG_DEBOUNCE) ? stats : NONE;
  }

  // recuperation de l action a effectuer en fonction de l'etat de la commande au volant
  int get_action()
  {
    static int molette = 0;
    int ret = NONE;
    int debounce_stats = get_stats();
    int debounce_long_stats = get_long_stats();

    int new_molette = debounce_stats & CV_MASK_MOL;
    if (new_molette != molette && ((new_molette == CV_MOL_1) || (new_molette == CV_MOL_2) || (new_molette == CV_MOL_3))) // si la molette a ete tournee
    {
      if (molette == CV_MOL_3 && new_molette == CV_MOL_1) // 3 -> 1
      {
        ret = AR_NEXT;
        // Serial.println("Next");
      }
      else if (molette == CV_MOL_1 && new_molette == CV_MOL_3) // 1 -> 3
      {
        ret = AR_PREV;
        // Serial.println("Prev");
      }
      else if (new_molette > molette) // 1 -> 2 ou 2 -> 3
      {
        ret = AR_NEXT;
        // Serial.println("next");
      }
      else if (new_molette < molette) // 3 -> 2 ou 2 -> 1
      {
        ret = AR_PREV;
        // Serial.println("prev");
      }
      molette = new_molette;
    }
    else if ((debounce_stats & CV_VOL_UP_DOWN) == CV_VOL_UP_DOWN) // Bouton VOL+ et VOL-
    {
      ret = AR_MUTE;
      // Serial.println("Mute");
    }
    else if ((debounce_stats & CV_BTN_UP_LFT_RGT) == CV_BTN_UP_LFT_RGT) // Bouton haut droite et gauche
    {
      ret = AR_BAND;
      // Serial.println("Band");
    }
    else if ((debounce_stats & CV_BTN_UP_LFT_BTN_DOWN) == CV_BTN_UP_LFT_BTN_DOWN) // Bouton haut droite et boutons bas
    {
      ret = AR_DISPLAY;
      // Serial.println("Display");
    }
    else if (debounce_long_stats & CV_BTN_DOWN) // Bouton bas
    {
      ret = AR_SOURCE;
      // Serial.println("Source");
    }
    else if (debounce_long_stats & CV_VOL_DOWN) // Volume -
    {
      ret = AR_VOL_DOWN;
      // Serial.println("Vol-");
    }
    else if (debounce_long_stats & CV_VOL_UP) // Volume +
    {
      ret = AR_VOL_UP;
      // Serial.println("Vol+");
    }
    else if (debounce_long_stats & CV_BTN_UP_RGT) // Bouton haut droite
    {
      ret = AR_PRESET_UP;
      // Serial.println("preset +");
    }
    else if (debounce_long_stats & CV_BTN_UP_LFT) // Bouton haut gauche
    {
      ret = AR_PRESET_DOWN;
      // Serial.println("preset-");
    }

    return ret;
  }

private:
  int stats;
  unsigned int cursor;
  unsigned long debounce;

  inline void UsePin(int Pin)
  {
    pinMode(Pin, OUTPUT); // set pin to output
    digitalWrite(Pin, LOW);
  }

  inline void Disable(int Pin)
  {
    pinMode(Pin, INPUT);    // set pin to input
    digitalWrite(Pin, LOW); // turn off pullup resistors
  }
};

class AutoRadio
{
public:
  // AutoRadio() {}

  void setup()
  {
    //audio
    pinMode(AR_TIP, OUTPUT);  // set pin to output
    pinMode(AR_RING, OUTPUT); // set pin to output
    digitalWrite(AR_TIP, LOW);
    digitalWrite(AR_RING, LOW);

    //relais marche arriere et frein de parc
    pinMode(PIN_AR_FP, OUTPUT); // set pin to output
    pinMode(PIN_AR_MA, OUTPUT); // set pin to output
    digitalWrite(PIN_AR_FP, LOW);
    digitalWrite(PIN_AR_MA, LOW);
  }

  void set_audio(int val)
  {
    static int old_val = 0;
    if (old_val != val)
    {
      if (val == NONE)
      {
        digitalWrite(AR_TIP, LOW);
        digitalWrite(AR_RING, LOW);
      }
      else if (val < AR_NEED_RING)
      {
        digitalWrite(AR_TIP, LOW);
        digitalWrite(AR_RING, LOW);
        set_pot(val);
        digitalWrite(AR_TIP, HIGH);
      }
      else if (val >= AR_NEED_RING)
      {
        digitalWrite(AR_TIP, LOW);
        digitalWrite(AR_RING, LOW);
        set_pot(val - AR_NEED_RING);
        digitalWrite(AR_TIP, HIGH);
        digitalWrite(AR_RING, HIGH);
      }

      old_val = val;
    }
  }

  void set_FP(bool value)
  {
    if (value)
    {
      digitalWrite(PIN_AR_FP, HIGH);
    }
    else
    {
      digitalWrite(PIN_AR_FP, LOW);
    }
  }

  void set_MA(bool value)
  {
    if (value)
    {
      digitalWrite(PIN_AR_MA, HIGH);
    }
    else
    {
      digitalWrite(PIN_AR_MA, LOW);
    }
  }

private:
  void set_pot(int val)
  {
    val = constrain(val, 0, 255);
    // set the CS pin to low to select the chip:
    digitalWrite(PIN_SPI_CS_DP, LOW);
    // send the command and value via SPI:
    SPI.transfer(POT0_SEL);
    SPI.transfer(val);
    // Set the CS pin high to execute the command:
    digitalWrite(PIN_SPI_CS_DP, HIGH);
  }
};

class Retroviseur
{
public:
  void setup()
  {
    //audio
    pinMode(PIN_RV_CLOSE, OUTPUT); // set pin to output
    pinMode(PIN_RV_OPEN, OUTPUT);  // set pin to output
    digitalWrite(PIN_RV_CLOSE, LOW);
    digitalWrite(PIN_RV_OPEN, LOW);
    active = false;
  }

  void loop()
  {
    if (active)
    {
      int temp = millis() - l_time;
      l_time = millis();
      if (temp > 0 && l_wait > temp)
      {
        l_wait -= temp;
      }
      else if (temp > 0)
      {
        active = false;
        callback();
      }
    }
  }

  void open(bool value)
  {
    if (value)
    {
      digitalWrite(PIN_RV_CLOSE, LOW);
      digitalWrite(PIN_RV_OPEN, HIGH);
      stop_after(RV_TIME);
    }
    else
    {
      digitalWrite(PIN_RV_CLOSE, LOW);
      digitalWrite(PIN_RV_OPEN, LOW);
    }
  }

  void close(bool value)
  {
    if (value)
    {
      digitalWrite(PIN_RV_OPEN, LOW);
      digitalWrite(PIN_RV_CLOSE, HIGH);
      stop_after(RV_TIME);
    }
    else
    {
      digitalWrite(PIN_RV_CLOSE, LOW);
      digitalWrite(PIN_RV_OPEN, LOW);
    }
  }

  void callback()
  {
    digitalWrite(PIN_RV_CLOSE, LOW);
    digitalWrite(PIN_RV_OPEN, LOW);
  }

  void stop_after(unsigned long int wait)
  {
    l_wait = wait;
    active = true;
    unsigned l_time = millis();
  }

private:
  bool active;
  unsigned long int l_time;
  int l_wait;
  void *l_arg;
};

class Resistance
{
public:
  // AutoRadio() {}

  void setup()
  {
    //audio
    pinMode(PIN_RC_1, OUTPUT);  // set pin to output
    pinMode(PIN_RC_23, OUTPUT); // set pin to output
    pinMode(PIN_RC_4, OUTPUT);  // set pin to output
    digitalWrite(PIN_RC_1, LOW);
    digitalWrite(PIN_RC_23, LOW);
    digitalWrite(PIN_RC_4, LOW);
  }

  //defini quels resistance doive etre allumé
  //
  void set(int mode)
  {
    if (mode & PIN_RC_1)
    {
      digitalWrite(PIN_RC_1, HIGH);
    }
    else
    {
      digitalWrite(PIN_RC_1, LOW);
    }

    if (mode & RC_23)
    {
      digitalWrite(PIN_RC_23, HIGH);
    }
    else
    {
      digitalWrite(PIN_RC_23, LOW);
    }

    if (mode & RC_4)
    {
      digitalWrite(PIN_RC_4, HIGH);
    }
    else
    {
      digitalWrite(PIN_RC_4, LOW);
    }
  }
};

class Trape
{
public:
  // Trape() {}

  void setup()
  {
    servo.attach(PIN_TRAP_PWM);
    servo.write(TRAP_STOP);
    pinMode(PIN_TRAP_ALIM, OUTPUT); // set pin to output
    digitalWrite(PIN_TRAP_ALIM, LOW);
    active = false;
  }

  void loop()
  {
    if (active)
    {
      int temp = millis() - l_time;
      l_time = millis();
      if (temp > 0 && l_wait > temp)
      {
        l_wait -= temp;
      }
      else if (temp > 0)
      {
        active = false;
        stop();
      }
    }
  }

  //defini quels resistance doive etre allumé
  //
  void up()
  {
    digitalWrite(PIN_TRAP_ALIM, HIGH);
    servo.write(TRAP_UP);
    // stop_after();
    call_after(RV_TIME);
  }

  void down()
  {
    digitalWrite(PIN_TRAP_ALIM, HIGH);
    servo.write(TRAP_DOWN);
    // stop_after();
    call_after(RV_TIME);
  }

  void call_after(unsigned long int wait)
  {
    l_wait = wait;
    active = true;
    l_time = millis();
  }

  void stop()
  {
    servo.write(TRAP_STOP);
    digitalWrite(PIN_TRAP_ALIM, LOW);
  }

  Servo servo;

private:
  bool sleep;
  bool active;
  unsigned long int l_time;
  int l_wait;
  void *l_arg;
};

class CAN_ESPACE
{
public:
  CAN_ESPACE() : can(PIN_SPI_CS_CAN)
  {
  }

  void setup()
  {
    can.begin(CAN_250KBPS);
    pinMode(PIN_CB_DATA, INPUT);

    frein_parking = false;
    marche_arriere = false;
    baterie = 0;
  }

  void loop()
  {
    if (!digitalRead(PIN_CB_DATA)) //if data
    {
      while (can.checkReceive() == CAN_MSGAVAIL)
      {
        can.readMsgBuf(&len, rxBuf); // Read data: len = data length, buf = data byte(s)
        rxId = can.getCanId();
        // Serial.print("ID: ");
        // Serial.print(rxId, HEX);
        // Serial.print("  Data: ");
        for (int i = 0; i < len; i++) // Print each byte of the data
        {
          if (rxBuf[i] < 0x10) // If data byte is less than 0x10, add a leading zero
          {
            Serial.print("0");
          }
          Serial.print(rxBuf[i], HEX);
          Serial.print(" ");
        }
        Serial.println();
      }
    }
  };

  bool get_frein_parking() { return frein_parking; };
  bool get_marche_arriere() { return marche_arriere; };
  int get_baterie() { return baterie; };

private:
  MCP_CAN can;

  long unsigned int rxId;
  unsigned char len = 0;
  unsigned char rxBuf[8];

  bool frein_parking;
  bool marche_arriere;
  long int baterie;
};

CAV commande;
AutoRadio ar;
Retroviseur retro;
Resistance resistance;
Trape trape;
CAN_ESPACE canbus;

void setup()
{
  // put your setup code here, to run once:
  pinMode(PIN_SPI_CS_CAN, OUTPUT); // set pin to output
  pinMode(PIN_SPI_CS_DP, OUTPUT);  // set pin to output
  digitalWrite(PIN_SPI_CS_CAN, HIGH);
  digitalWrite(PIN_SPI_CS_DP, HIGH);
  SPI.begin();        //initialize SPI:
  commande.setup();   //initialise la commande au volant
  ar.setup();         //initialise l autoradio (pot + relais)
  retro.setup();      //initialisation des retroviseurs
  resistance.setup(); //initialisation des resistance chauffante
  trape.setup();
  Serial.begin(115200);
  // commande.setSerial(&Serial);

  Serial.println("End INIT");
}

void loop()
{
  // Serial.println("New Loop");
  // put your main code here, to run repeatedly:
  commande.loop();
  retro.loop();
  trape.loop();

  // ar.set_audio(commande.get_action());
  static int old_action = 0;
  int action = commande.get_action();
  if (action != old_action)
  {
    old_action = commande.get_action();
    if (action == NONE)
    {
      Serial.println("action = NONE");
    }
    if (action == AR_BAND)
      Serial.println("action = BAND");
    if (action == AR_DISPLAY)
      Serial.println("action = DISPLAY");
    if (action == AR_MUTE)
    {
      Serial.println("action = MUTE");
      trape.stop();
      Serial.println(trape.servo.read());
    }
    if (action == AR_NEXT)
    {
      Serial.println("action = NEXT");
      trape.up();
      Serial.println(trape.servo.read());
    }
    if (action == AR_PRESET_DOWN)
      Serial.println("action = Preset down");
    if (action == AR_PRESET_UP)
      Serial.println("action = preset UP");
    if (action == AR_PREV)
    {
      Serial.println("action = prev");
      trape.down();
      Serial.println(trape.servo.read());
    }
    if (action == AR_SOURCE)
      Serial.println("action = source");
    if (action == AR_VOL_DOWN)
    {
      Serial.println("action = vol -");
      trape.servo.write(trape.servo.read() - 1);
      Serial.println(trape.servo.read());
      // trape.down();
    }
    if (action == AR_VOL_UP)
    {
      Serial.println("action = Vol +");
      trape.servo.write(trape.servo.read() + 1);
      Serial.println(trape.servo.read());
      // trape.up();
    }
  }
}
