
#include "config.h"
#include <mcp_can.h>
#include <SPI.h>
#include "Servo.h"
// #include "MyTimer.h"
#include "NeoTimer.h"
#include <CLI.h>
// #include <HardwareSerial.h>
// #include <Serial.h>

inline bool Debounce(unsigned long time, unsigned long debounce = DEBOUNCE)
{
  return (time + debounce) < millis() ? true : false;
}

//gestion de la commande au volant
//la methode setup() doit etre appele dans la fonction d initialisation
//la methode  loop() doit etre appele dans chaque iteration de le fonction loop()
class CAV : CLI_Command
{
public:
  String data;
  //constructeur
  CAV(CLI &cli) : CLI_Command(cli,
                              PSTR("cav"),
                              PSTR("Commande au volant"),
                              PSTR("Usage:\t\tcav <commande>\n"
                                   "Where:\t<commande>\t debug, std"))
  {
    stats = 0;
    cursor = 0;
    debounce = 0;
    _debug = 0;
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
    static int debug = NONE;
    static int molette = 0;
    int ret = NONE;
    unsigned long debounce_stats = get_stats();
    unsigned long debounce_long_stats = get_long_stats();

    if (_debug == 1)
    {
      Serial.print("cav = ");
      Serial.println(debounce_stats, BIN);
      Serial.println(_debug);
    }

    int new_molette = debounce_stats & CV_MASK_MOL;
    if (new_molette != molette && ((new_molette == CV_MOL_1) || (new_molette == CV_MOL_2) || (new_molette == CV_MOL_3))) // si la molette a ete tournee
    {
      if (molette == CV_MOL_3 && new_molette == CV_MOL_1) // 3 -> 1
      {
        ret = AR_NEXT;
        if (_debug != 0 && debug != ret)
          Serial.println("cav Next");
        // Serial.println("Next");
      }
      else if (molette == CV_MOL_1 && new_molette == CV_MOL_3) // 1 -> 3
      {
        ret = AR_PREV;
        if (_debug != 0 && debug != ret)
          Serial.println("cav Prev");
      }
      else if (new_molette > molette) // 1 -> 2 ou 2 -> 3
      {
        ret = AR_NEXT;
        if (_debug != 0 && debug != ret)
          Serial.println("cav next");
      }
      else if (new_molette < molette) // 3 -> 2 ou 2 -> 1
      {
        ret = AR_PREV;
        if (_debug != 0 && debug != ret)
          Serial.println("cav prev");
      }
      molette = new_molette;
    }
    else if ((debounce_stats & CV_VOL_UP_DOWN) == CV_VOL_UP_DOWN) // Bouton VOL+ et VOL-
    {
      ret = AR_MUTE;
      if (_debug != 0 && debug != ret)
        Serial.println("cav Mute");
    }
    else if ((debounce_stats & CV_BTN_UP_LFT_RGT) == CV_BTN_UP_LFT_RGT) // Bouton haut droite et gauche
    {
      ret = AR_BAND;
      if (_debug != 0 && debug != ret)
        Serial.println("cav Band");
    }
    else if ((debounce_stats & CV_BTN_UP_LFT_BTN_DOWN) == CV_BTN_UP_LFT_BTN_DOWN) // Bouton haut droite et boutons bas
    {
      ret = AR_DISPLAY;
      if (_debug != 0 && debug != ret)
        Serial.println("cav Display");
    }
    else if (debounce_long_stats & CV_BTN_DOWN) // Bouton bas
    {
      ret = AR_SOURCE;
      if (_debug != 0 && debug != ret)
        Serial.println("cav Source");
    }
    else if (debounce_long_stats & CV_VOL_DOWN) // Volume -
    {
      ret = AR_VOL_DOWN;
      if (_debug != 0 && debug != ret)
        Serial.println("cav Vol-");
    }
    else if (debounce_long_stats & CV_VOL_UP) // Volume +
    {
      ret = AR_VOL_UP;
      if (_debug != 0 && debug != ret)
        Serial.println("cav Vol+");
    }
    else if (debounce_long_stats & CV_BTN_UP_RGT) // Bouton haut droite
    {
      ret = AR_PRESET_UP;
      if (_debug != 0 && debug != ret)
        Serial.println("cav preset +");
    }
    else if (debounce_long_stats & CV_BTN_UP_LFT) // Bouton haut gauche
    {
      ret = AR_PRESET_DOWN;
      if (_debug != 0 && debug != ret)
        Serial.println("cav preset-");
    }

    debug = ret;
    return ret;
  }

  // CLI set parametre
  bool setparams(const char *params)
  {
    _params = params;
    return (params);
  }
  // CLI Execute
  bool execute(CLI &cli)
  {
    if (strcmp(_params, "debug") == 0)
    {
      if (_debug == 1)
      {
        Serial.println("Debug = off");
        _debug = 0;
      }
      else
      {
        Serial.println("Debug = on");
        _debug = 1;
      }
    }
    else if (strcmp(_params, "display") == 0)
    {
      if (_debug == 2)
      {
        Serial.println("display = off");
        _debug = 0;
      }
      else
      {
        Serial.println("display = on");
        _debug = 2;
      }
    }

    // cli.print_P(PSTR("Autoradio "));
    // cli.println(_params);
    return false;
  }

private:
  int stats;
  unsigned int cursor;
  unsigned long debounce;
  const char *_params;
  uint8_t _debug;

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

class AutoRadio : CLI_Command
{
public:
  AutoRadio(CLI &cli) : CLI_Command(cli,
                                    PSTR("ar"),
                                    PSTR("Auto Radio"),
                                    PSTR("Usage:\t\tAR <commande>\n"
                                         "Where:\t<commande>\t vol+, vol-, mute, ..."))
  {
    _debug = 0;
  };

  void setup()
  {
    //audio
    pinMode(AR_TIP, OUTPUT);     // set pin to output
    pinMode(AR_RING, OUTPUT);    // set pin to output
    digitalWrite(AR_TIP, HIGH);  //logique inverse
    digitalWrite(AR_RING, HIGH); //logique inverse

    //relais marche arriere et frein de parc
    pinMode(PIN_AR_FP, OUTPUT);    // set pin to output
    pinMode(PIN_AR_MA, OUTPUT);    // set pin to output
    digitalWrite(PIN_AR_FP, HIGH); //logique inverse
    digitalWrite(PIN_AR_MA, HIGH); //logique inverse
  }

  void loop()
  {
    if (this->_timer.front())
    {
      digitalWrite(AR_TIP, HIGH);
      digitalWrite(AR_RING, HIGH);
      if (_debug == 1)
        Serial.println("None");
    }
  }

  void
  set_audio(int val)
  {
    static int old_val = 0;
    // if (old_val != val)
    {
      // if (val == NONE)
      // {
      //   digitalWrite(AR_TIP, HIGH);
      //   digitalWrite(AR_RING, HIGH);
      // }
      // else
      if (val < AR_NEED_RING && val != NONE)
      {
        digitalWrite(AR_TIP, HIGH);
        digitalWrite(AR_RING, HIGH);
        set_pot(val);
        digitalWrite(AR_TIP, LOW);
        this->_timer.reset();
        this->_timer.start(100);
        if (_debug == 1)
          Serial.println(val);
      }
      else if (val >= AR_NEED_RING && val != NONE)
      {
        digitalWrite(AR_TIP, HIGH);
        digitalWrite(AR_RING, HIGH);
        set_pot(val - AR_NEED_RING);
        digitalWrite(AR_TIP, LOW);
        digitalWrite(AR_RING, LOW);

        this->_timer.reset();
        this->_timer.start(100);
        if (_debug == 1)
          Serial.println(val);
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
  // CLI set parametre
  bool setparams(const char *params)
  {
    _params = params;
    return (params);
  }
  // CLI Execute
  bool execute(CLI &cli)
  {
    if (strcmp(_params, "debug") == 0)
    {
      if (_debug == 1)
      {
        cli.println("Debug = off");
        _debug = 0;
      }
      else
      {
        cli.println("Debug = on");
        _debug = 1;
      }
    }
    if (strcmp(_params, "none") == 0)
    {
      cli.println("action = NONE");
      set_audio(NONE);
    }
    if (strcmp(_params, "band") == 0)
    {
      cli.println("action = BAND");
      set_audio(AR_BAND);
    }
    if (strcmp(_params, "display") == 0)
    {
      cli.println("action = DISPLAY");
      set_audio(AR_DISPLAY);
    }
    if (strcmp(_params, "mute") == 0)
    {
      cli.println("action = MUTE");
      set_audio(AR_MUTE);
    }
    if (strcmp(_params, "next") == 0)
    {
      cli.println("action = NEXT");
      set_audio(AR_NEXT);
    }
    if (strcmp(_params, "preset-") == 0)
    {
      cli.println("action = Preset down");
      set_audio(AR_PRESET_DOWN);
    }
    if (strcmp(_params, "preset+") == 0)
    {
      cli.println("action = preset UP");
      set_audio(AR_PRESET_UP);
    }
    if (strcmp(_params, "prev") == 0)
    {
      cli.println("action = prev");
      set_audio(AR_PREV);
    }
    if (strcmp(_params, "source") == 0)
    {
      cli.println("action = source");
      set_audio(AR_SOURCE);
    }
    if (strcmp(_params, "vol-") == 0)
    {
      cli.println("action = vol -");
      set_audio(AR_VOL_DOWN);
      // trape.down();
    }
    if (strcmp(_params, "vol+") == 0)
    {
      cli.println("action = Vol +");
      set_audio(AR_VOL_UP);
    }

    if (strcmp(_params, "ma") == 0)
    {
      digitalWrite(PIN_AR_MA, !digitalRead(PIN_AR_MA));
      cli.print("marche_arriere = ");
      cli.println(digitalRead(PIN_AR_MA));
    }

    if (strcmp(_params, "fp") == 0)
    {
      digitalWrite(PIN_AR_FP, !digitalRead(PIN_AR_FP));
      cli.print("frein_parking = ");
      cli.println(digitalRead(PIN_AR_FP));
    }

    // cli.print_P(PSTR("Autoradio "));
    // cli.println(_params);
    return false;
  }

private:
  const char *_params;
  uint8_t _debug;
  NeoTimer _timer;
  void set_pot(int val)
  {
    val = constrain(val, 0, 255);
    // set the CS pin to low to select the chip:
    digitalWrite(PIN_SPI_CS_DP, LOW);
    // send the command and value via SPI:
    // SPI.transfer(POT1_SEL);
    // SPI.transfer(val);
    SPI.transfer(BOTH_POT_SEL);
    SPI.transfer(val);
    // Set the CS pin high to execute the command:
    digitalWrite(PIN_SPI_CS_DP, HIGH);
  }
};

class Retroviseur : CLI_Command
{
public:
  Retroviseur(CLI &cli) : CLI_Command(cli,
                                      PSTR("retro"),
                                      PSTR("retroviseur"),
                                      PSTR("Usage:\tretro <commande>\n"
                                           "Where:\t<commande>\t open, close"))
  {
    _open = false;
  };

  void setup()
  {
    pinMode(PIN_RV_CLOSE, OUTPUT); // set pin to output
    pinMode(PIN_RV_OPEN, OUTPUT);  // set pin to output
    digitalWrite(PIN_RV_CLOSE, LOW);
    digitalWrite(PIN_RV_OPEN, LOW);
    // open(); //todo
  }

  void loop()
  {
    if (timer.front())
      stop();
  }

  void open()
  {
    if (!_open)
    {
      digitalWrite(PIN_RV_CLOSE, LOW);
      digitalWrite(PIN_RV_OPEN, HIGH);
      timer.start(RV_TIME);
      _open = true;
    }
  }

  void close()
  {
    if (_open)
    {
      digitalWrite(PIN_RV_OPEN, LOW);
      digitalWrite(PIN_RV_CLOSE, HIGH);
      timer.start(RV_TIME);
      _open = false;
    }
  }

  void stop()
  {
    digitalWrite(PIN_RV_CLOSE, LOW);
    digitalWrite(PIN_RV_OPEN, LOW);
  }

  // CLI set parametre
  bool setparams(const char *params)
  {
    _params = params;
    return (params);
  }
  // CLI Execute
  bool execute(CLI &cli)
  {
    if (strcmp(_params, "open") == 0)
    {
      Serial.println("action = open");
      open();
    }
    else if (strcmp(_params, "close") == 0)
    {
      Serial.println("action = close");
      close();
    }

    // cli.print_P(PSTR("Autoradio "));
    // cli.println(_params);
    return false;
  }

private:
  const char *_params;
  NeoTimer timer;
  bool _open;
  // bool active;
  // unsigned long int l_time;
  // int l_wait;
  void *l_arg;
};

class Resistance : CLI_Command
{
public:
  Resistance(CLI &cli) : CLI_Command(cli,
                                     PSTR("res"),
                                     PSTR("resistance"),
                                     PSTR("Usage:\tresistance <commande>\n"
                                          "Where:\t<commande>\t 0, 1, 2, 3, 23, 4")){};

  void setup()
  {
    pinMode(PIN_I_RC_1, INPUT);      // set pin to input
    pinMode(PIN_I_RC_23, INPUT);     // set pin to input
    pinMode(PIN_I_RC_4, INPUT);      // set pin to input
    digitalWrite(PIN_I_RC_1, HIGH);  //active pullup
    digitalWrite(PIN_I_RC_23, HIGH); //active pullup
    digitalWrite(PIN_I_RC_4, HIGH);  //active pullup

    pinMode(PIN_O_RC_1, OUTPUT);     // set pin to output
    pinMode(PIN_O_RC_23, OUTPUT);    // set pin to output
    pinMode(PIN_O_RC_4, OUTPUT);     // set pin to output
    digitalWrite(PIN_O_RC_1, HIGH);  //logique inverse
    digitalWrite(PIN_O_RC_23, HIGH); //logique inverse
    digitalWrite(PIN_O_RC_4, HIGH);  //logique inverse
  }

  //defini quels resistance doive etre allumé
  //
  void set(int mode)
  {
    if (mode & PIN_O_RC_1)
    {
      digitalWrite(PIN_O_RC_1, LOW);
    }
    else
    {
      digitalWrite(PIN_O_RC_1, HIGH);
    }

    if (mode & RC_23)
    {
      digitalWrite(PIN_O_RC_23, LOW);
    }
    else
    {
      digitalWrite(PIN_O_RC_23, HIGH);
    }

    if (mode & RC_4)
    {
      digitalWrite(PIN_O_RC_4, LOW);
    }
    else
    {
      digitalWrite(PIN_O_RC_4, HIGH);
    }
  } // CLI set parametre
  bool setparams(const char *params)
  {
    _params = String(params).toInt();
    return (params);
  }
  // CLI Execute
  bool execute(CLI &cli)
  {
    if (_params == 0)
    {
      Serial.println("resistance = 0");
      set(RC_OFF);
    }
    else if (_params == 1)
    {
      Serial.println("resistance = 1");
      set(PIN_O_RC_1);
    }
    else if (_params == 2)
    {
      Serial.println("resistance = 23");
      set(PIN_O_RC_23);
    }
    else if (_params == 3)
    {
      Serial.println("resistance = 23");
      set(PIN_O_RC_23);
    }
    else if (_params == 23)
    {
      Serial.println("resistance = 23");
      set(PIN_O_RC_23);
    }
    else if (_params == 4)
    {
      Serial.println("resistance = 4");
      set(PIN_O_RC_4);
    }

    // cli.print_P(PSTR("Autoradio "));
    // cli.println(_params);
    return false;
  }

private:
  int _params;
};

class Trape : CLI_Command
{
public:
  Trape(CLI &cli) : CLI_Command(cli,
                                PSTR("trape"),
                                PSTR("trape"),
                                PSTR("Usage:\ttrape <commande>\n"
                                     "Where:\t<commande>\t open, close")){};

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
    //   if (active)
    //   {
    //     int temp = millis() - l_time;
    //     l_time = millis();
    //     if (temp > 0 && l_wait > temp)
    //     {
    //       l_wait -= temp;
    //     }
    //     else if (temp > 0)
    //     {
    //       active = false;
    //       stop();
    //     }
    //   }
    if (timer.front())
      stop();
  }

  //defini quels resistance doive etre allumé
  //
  void
  up()
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
    // l_wait = wait;
    // active = true;
    // l_time = millis();
    timer.start(wait);
  }

  void stop()
  {
    servo.write(TRAP_STOP);
    digitalWrite(PIN_TRAP_ALIM, LOW);
  }

  // CLI set parametre
  bool setparams(const char *params)
  {
    _params = params;
    return (params);
  }
  // CLI Execute
  bool execute(CLI &cli)
  {
    if (strcmp(_params, "open") == 0)
    {
      Serial.println("action = open");
      up();
    }
    else if (strcmp(_params, "close") == 0)
    {
      Serial.println("action = close");
      down();
    }

    // cli.print_P(PSTR("Autoradio "));
    // cli.println(_params);
    return false;
  }

  Servo servo;

private:
  const char *_params;
  NeoTimer timer;
  bool sleep;
  bool active;
  unsigned long int l_time;
  int l_wait;
  void *l_arg;
};

volatile int Flag_Recv;
/* 
     *  ISR CAN (Routine de Service d'Interruption)
     *  le flag IRQ monte quand au moins un message est reçu
     *  le flag IRQ ne retombe QUE si tous les messages sont lus
     */
static void MCP2515_ISR()
{
  Flag_Recv = 1;
}

class CAN_ESPACE : CLI_Command
{
public:
  CAN_ESPACE(CLI &cli) : can(PIN_SPI_CS_CAN), CLI_Command(cli,
                                                          PSTR("can"),
                                                          PSTR("CAN Bus"),
                                                          PSTR("Usage:\t\tcan <commande>\n"
                                                               "Where:\t<commande>\t debug, std"))

  {
    // _cli = cli;
    _debug = false;
    _init = false;
    frein_parking = false;
    marche_arriere = false;
    embrayage = false;
    vitesse = false;
    baterie = 0;
  };

  void setup()
  {
    pinMode(PIN_CB_DATA, INPUT);
    Serial.println("CAN BUS init !");
    int loop = 3;
    while (loop >= 0)
    {
      Serial.println("CAN BUS init !");
      if (CAN_OK == can.begin(CAN_250KBPS))
      // initialisation du can bus : baudrate = 250k
      {
        Serial.println(F("CAN BUS init ok!"));
        _init = true;
        break; // on sort du while.
      }
      else
      {
        loop--;
        Serial.println(F("CAN BUS init echec !"));
        Serial.println(F("Init CAN BUS a nouveau"));
      }
      delay(100);
    }

    attachInterrupt(0, MCP2515_ISR, FALLING); // interrupt 0 (pin 2)
  }

  // address 766
  //          octet 0 porte 0x8 Av G 0x10 AV D 0x80 coffre 0x20 ar G 0x40 ar D
  //          octet 2 condanation 0x30 telecommand 0x20 interieur
  // address 766
  //          octet 4 Batterie 0.202857x * 8.8714

  void loop()
  {
    if (Flag_Recv) //if data
    {
      if (!_init)
        this->setup();
      // Serial.println("receve interupt");
      Flag_Recv = false;
      while (can.checkReceive() == CAN_MSGAVAIL)
      {
        // Serial.println("receve data");
        can.readMsgBuf(&len, rxBuf); // Read data: len = data length, buf = data byte(s)
        rxId = can.getCanId();

        if (rxId == 0x766 && len == 8)
        {

          porte = rxBuf[0];

          if ((rxBuf[1] & 0x02) != 0)
            pre_contact = true;
          else
            pre_contact = false;

          if ((rxBuf[1] & 0x04) != 0)
            contact = true;
          else
            contact = false;

          if ((rxBuf[2] & 0x10) != 0)
            verou = true;
          else
            verou_tel = false;

          if ((rxBuf[2] & 0x20) != 0)
            verou = true;
          else
            verou = false;

          if ((rxBuf[7] & 0x40) != 0)
            embrayage = true;
          else
            embrayage = false;

          if (contact && (rxBuf[5] & 0x01) != 0)
            vitesse = true;
          else
            vitesse = false;

          if (contact && (rxBuf[6] & 0x10) != 0)
            marche_arriere = true;
          else
            marche_arriere = false;
        }

        if (rxId == 0x711 && len == 8)
        {
          if ((rxBuf[0] & 0x04) != 0)
            frein_parking = true;
          else
            frein_parking = false;
        }

        if (rxId == 0x0FA && len == 7)
        {
          accelerateur = rxBuf[3];
          regime = (rxBuf[0] << 8) + rxBuf[1];
        }

        if (rxId == 0x449 && len == 6)
        {
          baterie = rxBuf[4];
          // Serial.print("baterie = ");
          // Serial.println(rxBuf[4], BIN);
          // Serial.print("baterie = ");
          // Serial.println(rxBuf[4], HEX);
        }

        if (_debug)
        {
          if (rxId == 0x766)
          {
            if (embrayage)
              Serial.println("Embrayage");

            if (vitesse)
              Serial.println("Vitesse");

            if (marche_arriere)
              Serial.println("marche_arriere");
          }

          if (rxId == 0x711)
          {

            if (frein_parking)
              Serial.println("frein_parking");
          }
          if (rxId == 0x0FA && len == 7)
          {
            Serial.print("accel = ");
            Serial.println(get_accelerateur());
            Serial.print("regim = ");
            Serial.println(get_regime());
          }

          if (rxId == 0x449 && len == 6)
          {
            Serial.print("baterie = ");
            Serial.print(get_baterie());
            Serial.println(" V");
          }
        }
      }
    }
  };

  bool get_frein_parking() { return frein_parking; };
  bool get_marche_arriere() { return marche_arriere; };
  bool get_vitesse() { return vitesse; };
  bool get_embrayage() { return embrayage; };
  bool get_contact() { return contact; };
  bool get_pre_contact() { return pre_contact; };
  bool get_verou() { return verou; };
  bool get_verou_tel() { return verou_tel; };
  unsigned int get_porte() { return porte; };
  unsigned int get_accelerateur() { return map(accelerateur, 0x08, 0xFD, 0, 100); };
  unsigned int get_regime() { return regime * 0.125; };
  float get_baterie() { return contact ? (baterie * 0.085789 + 10.8615) : 0; };

  // CLI set parametre
  bool setparams(const char *params)
  {
    _params = params;
    return (params);
  };
  // CLI Execute
  bool execute(CLI &cli)
  {
    if (strcmp(_params, "debug") == 0)
    {
      if (_debug)
      {
        Serial.println("can = debug off");
        _debug = false;
      }
      else
      {
        Serial.println("can = debug on");
        _debug = true;
      }
    }
    else if (strcmp(_params, "get") == 0)
    {
      if (get_frein_parking())
        Serial.println("frein de parking");

      if (get_marche_arriere())
        Serial.println("marche arriere");

      if (get_vitesse())
        Serial.println("vitesse");

      if (get_embrayage())
        Serial.println("embrayage");

      if (get_contact())
        Serial.println("contact");

      if (get_pre_contact())
        Serial.println("pre_contact");

      if (get_verou())
        Serial.println("verou");

      if (get_verou_tel())
        Serial.println("verou_tel");

      if (get_porte())
        Serial.println("porte");

      Serial.print("accelerateur = ");
      Serial.print(get_accelerateur());
      Serial.println(" %");

      Serial.print("regime = ");
      Serial.print(get_regime());
      Serial.println(" t/m");

      Serial.print("batterie = ");
      Serial.print(get_baterie());
      Serial.println(" V");
    }

    return false;
  }

private:
  bool _init;
  const char *_params;
  bool _debug;

  MCP_CAN can;
  CLI &_cli;

  long unsigned int rxId;
  unsigned char len = 0;
  unsigned char rxBuf[8];

  bool frein_parking;
  bool marche_arriere;
  bool vitesse;
  bool embrayage;
  bool contact;
  bool verou;
  bool verou_tel;
  int porte;
  unsigned int accelerateur;
  unsigned int regime;
  bool pre_contact;
  uint8_t baterie;
};

class Pin : CLI_Command
{
public:
  Pin(CLI &cli) : CLI_Command(cli,
                              PSTR("pin"),
                              PSTR("Pin"),
                              PSTR("Usage:\t\tpin <pin> <commande>\n"
                                   "Where:\t<commande>\t 0, 1"))

  {
    _pin = 0;
    _stats = 0;
  };

  // CLI set parametre
  bool setparams(const char *params)
  {
    String val = params;
    int index = val.indexOf(" ");
    _pin = val.substring(0, index).toInt();
    _stats = val.substring(index).toInt();
    return (params);
  }

  // CLI Execute
  bool execute(CLI &cli)
  {
    Serial.print(_pin);
    Serial.print(" = ");
    Serial.println(digitalRead(_pin));
    // cli.print_P(PSTR("Autoradio "));
    // cli.println(_params);
    return false;
  }

private:
  int _pin;
  int _stats;
};

// Initialize the Command Line Interface
const char CLI_banner[] PROGMEM = "ESPACE CLI v1.0 BETA";
CLI CLI(Serial, CLI_banner); // Initialize CLI, telling it to attach to Serial

CAV commande(CLI);
AutoRadio ar(CLI);
Retroviseur retro(CLI);
Resistance resistance(CLI);
Trape trape(CLI);
CAN_ESPACE canbus(CLI);
Pin pin(CLI);

// Hello_Command Hello(CLI);    // Initialize/Register above defined hello command
Help_Command Help(CLI); // Initialize/Register (built-in) help command

void setup()
{
  // put your setup code here, to run once:
  pinMode(PIN_SPI_CS_CAN, OUTPUT); // set pin to output
  pinMode(PIN_SPI_CS_DP, OUTPUT);  // set pin to output
  digitalWrite(PIN_SPI_CS_CAN, HIGH);
  digitalWrite(PIN_SPI_CS_DP, HIGH);

  Serial.begin(115200);
  SPI.begin();        //initialize SPI:
  commande.setup();   //initialise la commande au volant
  ar.setup();         //initialise l autoradio (pot + relais)
  retro.setup();      //initialisation des retroviseurs
  resistance.setup(); //initialisation des resistance chauffante
  canbus.setup();
  trape.setup();
}

void loop()
{
  // Serial.println("New Loop");
  // put your main code here, to run repeatedly:
  commande.loop();
  ar.loop();
  retro.loop();
  trape.loop();
  canbus.loop();

  CLI.process();

  ar.set_audio(commande.get_action());
  // ar.set_FP(canbus.get_frein_parking());
  // ar.set_MA(canbus.get_marche_arriere());

  //gestion retroviseur
  if (canbus.get_verou_tel() && !(canbus.get_contact()))
    retro.close();
  else if (!canbus.get_verou_tel() || canbus.get_contact())
    retro.open();
}
