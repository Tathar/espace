
#include "config.h"
#include <mcp_can.h>
#include <SPI.h>
#include <Servo.h>
// #include "MyTimer.h"
#include "NeoTimer.h"
#include <CLI.h>
#include "cav.h"
#include "AutoRadio.h"
#include "Retroviseur.h"
#include "Resistance.h"
#include "Trape.h"

#include "debounce.h"
#include "TtFront.h"
// #include <HardwareSerial.h>
// #include <Serial.h>

#define IF_MASK(value, mask) ((value & mask) == value)

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
                                                               "Where:\t<commande>\t _debug, std"))
  // CAN_ESPACE() : can(PIN_SPI_CS_CAN)

  {
    _debug = false;
    init = false;
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
    int loop = 5;
    while (loop >= 0)
    {
      Serial.println("CAN BUS init !");
      if (CAN_OK == can.begin(CAN_250KBPS))
      // initialisation du can bus : baudrate = 250k
      {
        Serial.println(F("CAN BUS init ok!"));
        init = true;
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

    if (!init)
      this->setup(); //todo

    if (Flag_Recv) //if data
    {

      // Serial.println("receve interupt");
      Flag_Recv = false;
      while (can.checkReceive() == CAN_MSGAVAIL)
      {
        Serial.println("receve data");
        can.readMsgBuf(&len, rxBuf); // Read data: len = data length, buf = data byte(s)
        rxId = can.getCanId();

        if (rxId == 0x766 && len == 8)
        {
          Serial.println("receve data 0x766");

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
            verou = false;

          if ((rxBuf[2] & 0x20) != 0)
            verou_tel = true;
          else
            verou_tel = false;

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
  bool get_verou() { return this->verou; };
  bool get_verou_tel() { return this->verou_tel && this->verou; };
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

  bool init;

  bool _debug;

private:
  const char *_params;

  MCP_CAN can;
  // CLI &_cli;

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

// Initialize the Command Line Interface
const char CLI_banner[] PROGMEM = "ESPACE CLI v1.0 BETA";
CLI CLI(Serial, CLI_banner); // Initialize CLI, telling it to attach to Serial

CAV commande(CLI);
AutoRadio ar(CLI);
Retroviseur retro(CLI);
Resistance resistance(CLI);
Trape trape(CLI);
CAN_ESPACE canbus(CLI);

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

  // if (commande.get_action() == AR_VOL_UP)
  //   trape.open();
  // if (commande.get_action() == AR_VOL_DOWN)
  //   trape.open();

  // ar.set_FP(!canbus.get_frein_parking());
  // ar.set_MA(canbus.get_marche_arriere());

  //gestion retroviseur

  // static TtFront front_verou_tel(canbus.get_verou_tel());
  if ((canbus.get_verou_tel() && canbus.get_verou()) && !(canbus.get_contact()))
    retro.close();
  else if (!(canbus.get_verou_tel() && canbus.get_verou()) || canbus.get_contact())
    retro.open();

  // // //gestion trape
  static TtFront front_trape(canbus.get_pre_contact() || canbus.get_contact());
  if (front_trape.False(canbus.get_pre_contact() || canbus.get_contact()))
    trape.close();
  else if (front_trape.True(canbus.get_pre_contact() || canbus.get_contact()))
    trape.open();

  //gestion resistance chauffante

  if (canbus.get_contact() && canbus.get_regime() > 500)
  {
    resistance.set(resistance.get());
  }
}
