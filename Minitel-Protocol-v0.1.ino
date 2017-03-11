/* SerialPassthrough sketch modified for Mega2560 & TellyMate & BlueTooth [BK3231 - SlaveOnly] & Matrix Keyboard

  Liste des modules (internes - détail de l'affectation des ports série)

  0 : Serial Port 0 = USB (prise)
  1 : Serial Port 1 = TellyMate (écran)
  2 : Serial Port 2 = BlueTooth (modem)
  3 : N/A (pas de port série) = clavier

 */
/* Rappel des codes contrôles VT52 utilisés avec Tellymate
 *  
 *  http://www.batsocks.co.uk/products/Other/TellyMate_UserGuide_ControlSequences.htm
 *  
   * ESC + E : Clear + Home
   * ESC + x + 4 : Block cursor
   * ESC + e : Cursor on
   * ESC + f : Cursor off
   * ESC + Q : Diag output
   * 0x0A : Line feed
   * ESC + _2 : Double height up
   * ESC + _3 : Double height down
   */
/* Etat d'avancement de l'émulation
 *  
 * ENQRAM1 (0x05)/ENQRAM2 (PRO1/??) non traités, téléchargements RAM non traités (??? je ne connais pas les codes)
 * Mode loupe non traité (??? je ne connais pas les codes)
 * Retournement PRO1/RET1, PRO1/RET2, PRO1/OPPO (bit EC=1 de terminal_status, envoi SEP/50, inhibition PRO1/RET1 et PRO1/RET2), PRO1/OPPORE (fin inhibition RET1/RET2 et envoi status terminal) non traités et acquitements :
 *      SEP/50 (changement vitesse modem à la connexion==Opposition)
 *      SEP/51 (changement vitesse modem en cours connexion)
 *      SEP/58 (retournement)==> Arrêt de PCE en cas de retournement
 *      Pas de distinction SEP/59 (état du relais prise de ligne ??) et SEP/53 (état détection porteuse ??)
 * Connexion ESC9h non traité (module slave only pour le moment)
 * Module téléphonique encore non implémenté ==> Et pas de téléchargement des numéros
 * Mode téléinfo => Blocage protocole en mode téléinfo (et sortie du mode teleinfo par CSI/3F/7B)
 * 
 * Réinitialisations sur réception de PRO1/Reset
 * Gestion du fil PT (terminal prêt) et acquitements (SEP/54)
 * 
 * Certaines (beaucoup de) fonctions sont implémentées mais n'ont aucun effet, 
 *     l'objet en l'état de ce projet n'étant pas de réaliser une émulation Minitel mais une émulation du protocole du Minitel. 
 * Ex : Impression écran, demande de position curseur, vitesse prise
 * 
 * Certaines fonctions n'existent pas dans le protocole mais sont implémentées à des fins de débuggage 
 *    PRO2@[Debug mode - Bit 0=Scroll, Bit 1=DebugProtocole, Bit 2=Trace]
 *    PRO2A[Affichage debugs status sur la prise - Bits 0/1/2 : Affcher Debug status/Aiguilage/statusbis]
 *    PRO2(B ou C)(module)[Désactivation(B)/Activation(C) Filtre en sortie  X=CTRL/Y=ESC      X=PQRS/ Y=XYZ[      (Ecran/Clavier/Modem/Prise)]
 *    PRO2(D ou E)(module)[Désactivation(B)/Activation(C) Filtre en entree  X=CTRL/Y=ESC      X=PQRS/ Y=XYZ[      (Ecran/Clavier/Modem/Prise)]
 *    NB : Pas de commande encore implémentée pour activation/désactivation des filtres d'entrée $$(CTRL) et ##(ESC) ainsi que [SEP] en sortie
 *    -> Copier/coller ou modification de comportement par défaut dans InitProtocol()
 */

#define DEBUG_OPTION    // Existence du code de debuggage (trace, dump variables/status)
#define DEBUG_OPTION_IO // Existence du code de debuggage pour entrée/sortie ($# en entrée et ^[ESC][SEP] en sortie)
//#define SCROLL_OPTION   // Existance du scroll
#define MATRIX_KBD      // Un clavier matriciel est présent
#define PS2_KBD         // Un clavier PS2 est présent

#define TOTALSERIAL 4   // Nous avons 4 modules (dont les 3 premiers sont des vrais ports série)
#define TOTALREALSERIAL 3 // Nombre total de ports série
#define TOTALMODULE 8   // Nombre total (maximal) de modules définissables pour le minitel

#define MODULE_PRISE 0        // Numéro interne du module
#define MODULE_ECRAN 1        // Numéro interne du module
#define MODULE_MODEM 2        // Numéro interne du module
#define MODULE_CLAVIER 3       // Numéro interne du module

  HardwareSerial *MySerial[TOTALREALSERIAL]={&Serial,&Serial1,&Serial2};
  int MySerialSpeed[]={57600,57600,9600};

#ifdef DEBUG_OPTION
  // https://www.arduino.cc/en/Reference/PROGMEM et https://lowpowerlab.com/forum/moteino/saving-memory-with-macros-(put-strings-from-ram-to-progmem)/
  const char Text_Name_Module_None[] PROGMEM ={" <Aucun> "};       // Pour fonctions de débugage protocole
  const char Text_Name_Module_Prise[] PROGMEM ={"Prise"};
  const char Text_Name_Module_Ecran[] PROGMEM ={"Ecran"};
  const char Text_Name_Module_Modem[] PROGMEM ={"Modem"};
  const char Text_Name_Module_Clavier[]PROGMEM  ={"Clavier"};
  const char * const Text_Name_Module[] PROGMEM ={Text_Name_Module_Prise,Text_Name_Module_Ecran,Text_Name_Module_Modem,Text_Name_Module_Clavier};
#endif

  const char InitStr0[] PROGMEM ={0x18,0x1b,'E',0};
  const char InitStr1[] PROGMEM ={0x1b,'x','4',0x1b,'e',0x1b,'Q',0x0d,0x0a,0};    //,0x1b,'Y',' ',0x20+20,'N','o',' ','s','c','r','o','l','l',0x1b,'Y',0x20+17,0x20,0};
  const char InitStr2[] PROGMEM ={0x0d,0x0d,0x0a,'A','T',0x0d,0x0d,0x0a,'A','T','+','V','E','R','S','I','O','N',0x0d,0x0d,0x0a,0};
  //const char InitStr2[] PROGMEM ={0x0d,0x0d,0x0a,'A','T',0x0d,0x0d,0x0a,'A','T',0x0d,0x0d,0x0a,'A','T','+','H','E','L','P',0x0d,0x0d,0x0a,'A','T','+','V','E','R','S','I','O','N',0x0d,0x0d,0x0a,0};
  
//======================== Concerne scroll - (NB : Cette partie (scroll) n'a aucun rapport avec l'émulation protocole du Minitel)
#define ScrollDevice 1      // Numéro du module utilisé pour le scroll - Utilisé aussi pour affichage état modem

#ifdef SCROLL_OPTION
  bool scroll=true;    //Scrolling activé ou non
  #define MaxScroll 3
  #define MaxScroll0 6
 
  byte ScrollText0_Index=0;
  int Scroll_Index[MaxScroll]={0,0,0};
  char ScrollText0_0[]={"                Hello FabLab              - Sur cet ecran, un affichage video PAL directement genere par un simple Arduino pro-mini a 16Mhz sur lequel est installe TellyMate (http://www.batsocks.co.uk/)."};
  char ScrollText0_1[]={"                Une fois le code installe dans l'ATmega328P (un processeur plus petit pouvant faire l'affaire), il ne necessite que l'adjonction de trois resistances et de deux diodes."};
  char ScrollText0_2[]={"                Dans cette configuration, 11 polices de caracteres programmables sont disponibles, TellyMate fournit une emulation VT52 et H19 en 38 colonnes par 25 lignes.."};
  char ScrollText0_3[]={"                Chaque caractère est defini dans une matrice de 8x9 pixels. Ils peuvent etre affiches en double hauteur, double largeur ou double taille."};
  char ScrollText0_4[]={"                L'arduino Mega ci-present, vraiment sous-utilise, transmet en serie a 57600 Bds sur son second port serie les caracteres des scrollings. (Comme au bon vieux temps du commodore 64...)"};
  char ScrollText0_5[]={"                Il agit aussi en passerelle bi-directionnelle entre TellyMate et son port serie principal (USB), en convertissant au passage les codes de controle, mais ceci est une autre histoire."};
  char ScrollText1[]={"                Ici, un autre texte defillant a une autre vitesse, plus lente"};
  char ScrollText2[]={"                                                Et ici plutot vite ...                "};
  char *ScrollText0[MaxScroll0]={ScrollText0_0,ScrollText0_1,ScrollText0_2,ScrollText0_3,ScrollText0_4,ScrollText0_5};
  char *ScrollText[MaxScroll]={ScrollText0[ScrollText0_Index],ScrollText1,ScrollText2};

  long prev_millis[MaxScroll]={0L,0L,0L};
  byte ScrollLine[MaxScroll]={19,21,23};
  long ScrollSpeed[MaxScroll]={150L,200L,50L};

  void SendScroll(byte Scroll, byte MyLine,byte Scroll_Index);
  void GotoScrollLine(byte Scroll, byte MyLine);
  void DoScroll(void);
  void DoInitScroll(void);
#endif
//======================== Concerne status BT
void UpdateStatusBT(char *a,char lin,char col);

#define BT_STATUS_LIN 0
#define BT_STATUS_COL 35
#define BT_CONNECT_COL 25

  #define PIN_STATUS_BT 48                // Pin d'entrée du status BT
  #define PIN_RESET_BT 49                 // Pin de sortie du reset BT
//  #define PIN_RESET_BT 13                 // Pin de sortie du reset BT

  #define Millis_BT_Status_Check 100      // Lire le status tous les 100ms
  #define BT_Status_Unchanged_Limit 20    // Si status BT fixe pendant 1,5 seconde, alors soit connecté, soit dead
  bool Prev_BT_Status=false;              // Précédent status lu
  long Millis_BT_Status=0L;               // Prochain tick pour lire le status BT
  int BT_Status_Unchanged=0;              // Nombre de lectures du status BT avec status inchangé
  byte BT_Status_Wait_Displayed;          // Message "BT_WAIT" déjà affiché (pour ne pas répéter sans cesse)

//======================== Concerne interpretations & filtrages
#ifdef DEBUG_OPTION_IO

  #define FILTER_CTRL_SUBST '$' // En entrée
  #define FILTER_CTRL_DISP  '^' // En sortie
  #define FILTER_ESC_SUBST '#'  // En entrée
  #define FILTER_ESC_DISP  "[ESC]" // En sortie
  #define FILTER_SEP_DISP  "[SEP]" // En sortie
  
  // Traitement $x => Contrôle en entrée
  bool Ser_IntCTRL_in[TOTALSERIAL];     // ={true,true,true} Interpretation des codes contrôle en entrée $<x>
  bool Ser_GotCTRL[TOTALSERIAL];     // ={false,false,false} Précédent code reçu est un code de contrôle
  // Traitement ^x => Contrôle en sortie
  bool Ser_FltCTRL_out[TOTALSERIAL]; // ={false,false,false} Filtrage - Interpretation des codes contrôle en sortie ^<x>
  
  // Traitement #x => ESC en entrée
  bool Ser_IntESC_in[TOTALSERIAL];      // ={true,true,true} Interpretation des codes contrôle en entrée #<x>
  bool Ser_GotESC1[TOTALSERIAL];     // ={false,false,false} Précédent code reçu est un code ESC (lors de l'interprétation du #)
  // Traitement [ESC]x => Escape en sortie
  bool Ser_FltESC_out[TOTALSERIAL];     // ={true,true,true} Filtrage - Interpretation des codes contrôle en sortie [ESC]<x>
  bool Ser_FltSEP_out[TOTALSERIAL];     // ={true,true,true} Filtrage - Interpretation des codes séparateur en sortie [SEP]<x>
#endif
  //======================== Concerne Protocole (Certains codes sont toujours 'en dur' dans le code .... à corriger)
#define CODE_SOH 0x01
#define CODE_STX 0x02
#define CODE_ETX 0x03
#define CODE_EOT 0x04
#define CODE_ENQ 0x05
#define CODE_SEP 0x13
#define CODE_ESC 0x1b
#define CODE_US  0x1f
#define CODE_PRO1 0x39
#define CODE_PRO2 0x3a
#define CODE_PRO3 0x3b

  // Traitement Escape en entrée pour protocole
  bool Ser_GotESC[TOTALSERIAL];      // ={false,false,false} Précédent code reçu est un code ESC
  //
  byte Ser_GotPRO[TOTALSERIAL];                  // {0,0,0}  Précédent code reçu est un code protocole [nb de caractères attendus]
  byte Ser_ProBuf[TOTALSERIAL][4];                        // Au maximum, séquence de 4 charactères
  byte Ser_ProBufIdx[TOTALSERIAL];                        // Index actuel dans le buffer 'Ser_ProBuf' lors de la réception d'une commande protocole
  //
  byte Ser_ProTransp[TOTALSERIAL];              // = {0,0,0} Nombre de caractères en transparence
  bool Ser_ModBlocked[TOTALSERIAL];  // ={false,false,false} Etat bloqué/non bloqué d'un module (RAZ lors du changement d'état de la connexion modem)
  bool Ser_Aiguill[TOTALSERIAL][TOTALSERIAL]; // ={true,true,true},{true,true,true},{true,true,true} Etat actif/inactif des aiguillages d'un module (RAZ lors du changement d'état de la connexion modem)
  bool Ser_NoDif[TOTALSERIAL];                // ={true,true,true} Non diffusion d'acquitement [Si vrai, ne recoit pas la diffusion d'acquitement]
  bool Ser_NoRet[TOTALSERIAL];                // ={false,false,false} Non retour d'acquitement [Si vrai, ne retourne pas d'acquitement]
  
  byte Ser_MapDev[TOTALSERIAL]={0x08,0x01,0x04,0x02};        // Pour status aiguillage - Table de conversion Interne=>Protocole
  byte Ser_MapDev1[TOTALMODULE]={0x01,0x03,0x02,0x00,0xff,0xff,0xff,0xff};       // Pour diffusion/acquitement - Table de conversion Protocole=>Interne - 0xFF=> module inexistant

  // M1b
  // Ser0 = Prise / Serial monitor
  // Ser1 = Ecran / Tellylate
  // Ser2 = Modem / BlueTooth
  // Ser3 = Clavier / Kbd

  byte CODE_ROM[3]={'C','u','<'};
  byte CODE_RAM1[8]={'A','B','C','D','E','F','G','1'};
  byte CODE_RAM2[8]={'A','B','C','D','E','F','G','2'};

void DoInitProtocol(void);                               // Initialisations protocole aux valeurs par défaut
void DoDieseAsEscape(byte b,byte a);                     // interprétation pour #=>ESC
void DoNormalAndControlChar(byte b,byte a);              // Transmettre code controle ou normal
void DoEscapeChar(byte b,byte a);                        // Detecter ESC/protocole
void EnvoieStatusAiguillage(byte b,byte z,bool recepteur); // PRO2 TO ==> PRO3 FROM
void ReponseStatusProtocole(byte b);                     // Envoi ReponseStatusProtocole
void EnvoiStatusClavier(byte b);                    // Envoi status clavier
void EnvoiStatusVitesse(byte b);                    // Envoi status vitesse
void EnvoiStatusFonctionnement(byte b);             // Envoi status fonctionnement
void EnvoiChangementStatus(byte Status);            // Envoie status SEP/xx

void MultiSerialWrite(int From,int chr);                 // Envoie chr depuis 'From' vers tous ses destinataires - Prise en charge des blocages et aiguillages
void SerialWrite(int To,byte chr);                       // Envoie chr vers 'To' sans prise en compte du protocole mais décodage [ESC] et [SEP]

void SerialWriteProgmemString(const char *a);             // Envoie une chaine en PROGMEM vers Serial sans aucune interprétation
void SerialToWriteProgmemString(byte To,const char *a);   // Envoie une chaine en PROGMEM vers Serial[To] sans aucune interprétation
//======================== Concerne debug
#ifdef DEBUG_OPTION
  byte debug_Protocole=3;  
  #define  debug_trace 2
  #define  debug_protocol 1

  char debug_text0[64];
  char debug_text1[64];
  char debug_text2[64];

void DisplayDebugStatus(void);       // Serial.write
void DisplayDebugStatusBis(void);    // Serial.write
void DisplayDebugAiguillage(void);   // Serial.write
void DisplayDeviceName(byte device); // Serial.write
void DisplayHex(byte chr);           // Serial.write 
void DisplayBin(byte chr);           // Serial.write
#endif
//======================== Concerne clavier


#ifdef MATRIX_KBD
  #include <Keypad.h>
  //= MatrixKeypad =========================================
  const byte ROWS = 4; //four rows
  const byte COLS = 3; //three columns
  char keys[ROWS][COLS] = {     // Définition de la matrice clavier
    {'1','2','3'},
    {'4','5','6'},
    {'7','8','9'},
    {'*','0','#'}
  };
  byte rowPins[ROWS] = {32, 33, 34, 35}; //connect to the row pinouts of the keypad
  byte colPins[COLS] = {22, 23, 24}; //connect to the column pinouts of the keypad
  Keypad matrix_keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );
  #define KBD_PRESENT
#endif
#ifdef PS2_KBD
  #include <PS2Keyboard.h>
  const int DataPin = 8;
  const int IRQpin =  2;

  PS2Keyboard keyboard;
  #ifndef KBD_PRESENT
    #define KBD_PRESENT
  #endif
#endif

byte clavier_status=0x00;                           // Clavier pas étendu
//======================== Concerne plusieurs modules
byte fonctionnement_status=0;                       // Majuscules, Pas PCE, Mode page (pas rouleau), 40 colonnes)
bool padx3_status=false;                            // Mode Pad_X3 - Obtenu par FNCT-T, il n'existe pas de commande
//======================== Concerne prise
int  vitesse_prise=9600;
byte vitesse_status=7;
//======================== Concerne modem
byte terminal_status=0;
bool ModemIsConnected;                            // =false Par défaut, le modem n'est pas connecté (sera mis à jour après 1 seconde avec l'état BT fixe ou non)
//#define LoopBackBufferSize  16                  // Taille du buffer loopback modem
//byte LoopBackBuffer[16];                        // Buffer pour loopback (lorsque le modem est déconnecté)
//byte LoopBackBufferIn=0;                        
//byte LoopBackBufferOut=0;
void UpdateModemStatus(char *a,bool StatusModem); // Met à jour l'état modem à l'écran et force éventuellement la déconnexion
void NewStatusModem(bool StatusModem);            // Traitement d'un éventuel changement d'état modem
void DeconnecteModem(void);
//======================== Concerne divers

void SaveCrsr(void);
void RestoreCrsr(void);

long Curr_Millis;

void setup() {
  byte b;

  for (b=0;b<TOTALREALSERIAL;b++) {
    switch(b) {
      case 0:
        MySerial[b]->begin(57600);
        while (!MySerial[b]) {
          ;
        }
        break;
      case 1:
        MySerial[b]->begin(57600);
        while (!MySerial[b]) {
          ;
        }
        break;
      case 2:
        MySerial[b]->begin(9600);
        while (!MySerial[b]) {
          ;
        }
        break;
    }
  }

  SerialToWriteProgmemString(1,InitStr0);
  delay (100);
  SerialToWriteProgmemString(1,InitStr1);
  delay (10);
  SerialToWriteProgmemString(2,InitStr2);

#ifdef SCROLL_OPTION
  if (scroll) {
    DoInitScroll();
  }
#endif

  pinMode(PIN_STATUS_BT,INPUT);
  pinMode(PIN_RESET_BT,OUTPUT);

  DeconnecteModem();  

#ifdef PS2_KBD  
  //keyboard.begin(DataPin, IRQpin, PS2Keymap_US);
  //keyboard.begin(DataPin, IRQpin, PS2Keymap_German);  
  keyboard.begin(DataPin, IRQpin, PS2Keymap_French);
#endif
}

void loop() {
  byte a,b;

  Curr_Millis=millis();

  if (Curr_Millis>Millis_BT_Status) {
    bool BT_Status;
    char *a;
    
    Millis_BT_Status=Curr_Millis+Millis_BT_Status_Check;
    BT_Status=digitalRead(PIN_STATUS_BT);
    if (BT_Status!=Prev_BT_Status) {          // Etat LED BT changé
      Prev_BT_Status=BT_Status;
      switch (Prev_BT_Status) {
        case true:
          a=(char *) &"ON ";
          break;
        case false:
          a=(char *) &"OFF";
          break;
      }
      UpdateStatusBT(a,BT_STATUS_LIN,BT_STATUS_COL);
      BT_Status_Unchanged=0;                  // Reset timer état BT
    }                                         // Fin état BT changé
    else {                                    // Etat LED BT inchangé
      BT_Status_Unchanged++;
      if (BT_Status_Unchanged>=BT_Status_Unchanged_Limit) {
        if (BT_Status_Unchanged==BT_Status_Unchanged_Limit) {
          if (Prev_BT_Status) {                                 // Nouvel état fixe : BT Connecté
            #ifdef DEBUG_OPTION
            if (debug_Protocole & debug_trace) {        // SiDebuggage protocole
              Serial.println(F("BtConnected()"));
            }
            #endif
            a=(char *) &"CONNECTED ";
            BT_Status_Wait_Displayed=1;
            UpdateModemStatus(a,true);
          }
          else {                                                // Nouvel état fixe : BT Dead
            #ifdef DEBUG_OPTION
            if (debug_Protocole & debug_trace) {        // SiDebuggage protocole
              Serial.println(F("BtDead()"));
            }
            #endif
            a=(char *) &"!BT DEAD! ";
            BT_Status_Wait_Displayed=0;
            UpdateModemStatus(a,false);
          }
        }                                                       // Fin Limite atteinte; changement d'état BT
        else {
          BT_Status_Unchanged--;                                // La limite de changement d'état est déjà dépassée; ne pas incrémenter plus
        }
      }                                                         // Fin Limite dépassée; état BT fixe
      else {
        if (BT_Status_Wait_Displayed!=2) {
         #ifdef DEBUG_OPTION
         if (debug_Protocole & debug_trace) {        // SiDebuggage protocole
            Serial.print(F("BtWait() "));
            Serial.println(BT_Status_Wait_Displayed);
          }
          #endif
          BT_Status_Wait_Displayed=2;
          a=(char *) &".BT Wait. ";
          UpdateModemStatus(a,false);
        }
      }
    }                                           // Fin état BT inchangé
  }
#ifdef SCROLL_OPTION
  if (scroll) {
    DoScroll();
  }
#endif
  for (b=0;b<TOTALREALSERIAL;b++) {
    if (MySerial[b]->available()) {      // If anything comes in Serial
      a=MySerial[b]->read();
      DoDieseAsEscape(b,a);
    }
  }
  for (b=TOTALREALSERIAL;b<TOTALSERIAL;b++) {
    ;
    if (b==(TOTALREALSERIAL+0)) {
    // Traitement clavier ......
      #ifdef KBD_PRESENT
        #ifdef PS2_KBD
          a=0;
          if (keyboard.available()) {
            a = keyboard.read();
            if (a)     DoDieseAsEscape(b,a);
          }
        #endif
        #ifdef MATRIX_KBD
          a = matrix_keypad.getKey();  
          if (a){
            switch (a) {
              case '3':
                DisplayDebugStatusBis();
                break;
              case '2':
                DisplayDebugStatus();
                break;
              case '1':
                DisplayDebugAiguillage();
                break;
              default:
                DoDieseAsEscape(b,a);
            }
          }
        #endif
      #else
        ;
      #endif
    }
  }
}


void DoInitProtocol(void){                               // Initialisations protocole aux valeurs par défaut
  byte b,c;

  #ifdef DEBUG_OPTION
  if (debug_Protocole & debug_trace) {        // SiDebuggage protocole
    Serial.println(F("InitProtocol()"));
  }
  #endif

  for (b=0;b<TOTALSERIAL;b++) {
    #ifdef DEBUG_OPTION_IO
    Ser_IntCTRL_in[b]=true;        // Interpretation des codes contrôle en entrée $<x>
    Ser_FltCTRL_out[b]=false;      // Filtrage - Interpretation des codes contrôle en sortie ^<x>
    Ser_GotCTRL[b]=false;          // Précédent code reçu est un code de contrôle
  
    Ser_IntESC_in[b]=true;         // Interpretation des codes contrôle en entrée #<x>
    Ser_FltESC_out[b]=false;       // Filtrage - Interpretation des codes contrôle en sortie [ESC]<x>
    Ser_FltSEP_out[b]=true;        // Filtrage - Interpretation des codes séparateur en sortie [SEP]<x>

    Ser_GotESC1[b]=false;          // Précédent code reçu est un code ESC (lors de l'interprétation du #)
    #endif
    Ser_GotESC[b]=false;           // Précédent code reçu est un code ESC
    Ser_GotPRO[b]=0;               // Précédent code reçu est un code protocole [nb de caractères attendus]

    Ser_ProTransp[b]=0;            // = {0,0,0} Nombre de caractères en transparence
    Ser_ModBlocked[b]=false;       // ={false,false,false} Etat bloqué/non bloqué d'un module (RAZ lors du changement d'état de la connexion modem)
    for (c=0;c<TOTALSERIAL;c++) {
      Ser_Aiguill[b][c]=false;      // ={true,true,true},{true,true,true},{true,true,true} Etat actif/inactif des aiguillages d'un module (RAZ lors du changement d'état de la connexion modem)  }
    }
    Ser_NoDif[b]=true;                 // ={true,true,true} Non diffusion d'acquitement [Si vrai, ne recoit pas la diffusion d'acquitement]
    Ser_NoRet[b]=false;                // ={false,false,false} Non retour d'acquitement [Si vrai, ne reçoit pas d'acquitement à ses propres demandes]
  }
  // M1b
  // Ser0 = Prise / Serial monitor
  // Ser1 = Ecran / Tellylate
  // Ser2 = Modem / BlueTooth
  // Ser3 = Clavier / Kbd
  if (ModemIsConnected) {
    #ifdef DEBUG_OPTION
    if (debug_Protocole) {        // SiDebuggage protocole
      Serial.println(F("InitProtocol():ModemConnected"));
    }
    #endif
    Ser_Aiguill[3][2]=true;   // Clavier->Modem
    Ser_Aiguill[2][1]=true;   // Modem->Ecran
    Ser_Aiguill[2][0]=true;   // Modem->Prise
    Ser_Aiguill[0][2]=true;   // Prise->Modem
  }
  else {
    #ifdef DEBUG_OPTION
    if (debug_Protocole) {        // SiDebuggage protocole
      Serial.println(F("InitProtocol():ModemNotConnected"));
    }
    #endif
    Ser_Aiguill[3][2]=true;   // Clavier->Modem
    Ser_Aiguill[2][1]=true;   // Modem->Ecran
    Ser_Aiguill[3][0]=true;   // Clavier->Prise
    Ser_Aiguill[0][1]=true;   // Prise->Ecran
    // Aiguillages spécifiques BT/Debug ??
    Ser_Aiguill[0][2]=true;   // Prise->Modem
    Ser_Aiguill[2][0]=true;   // Modem->Ecran
  }
}
void DoDieseAsEscape(byte b,byte a){                     // interprétation pour #=>ESC
                          #ifdef DEBUG_OPTION_IO
  if (Ser_IntESC_in[b]) {            // Si Interpretation des codes ESC/# actif
    if (a==FILTER_ESC_SUBST) {       // Si recu '#'
      if (Ser_GotESC1[b]==false) {   // Si pas '#' dans caractère d'avant
        Ser_GotESC1[b]=true;         // Le prochain caractère sera un code ESC, ne pas transmetre '#'
      }
      else {                         // Sinon, '#' dans caractère d'avant, donc reçu '##', transmettre juste '#'
        Ser_GotESC1[b]=false;        // Séquence reçue = "##" ==> Envoyer '#' seulement
        DoNormalAndControlChar(b,a);
      }
    }                                 // FinSi reçu '#'
    else {                            // Si Pas reçu '#' ....
      if (Ser_GotESC1[b]==true) {     // Reçu précédement '#', celui-ci introduit un code ESC et n'est pas un '#'
        Ser_GotESC1[b]=false;         // Séquence reçue = "#"+Code ESC ==> Envoyer ESC + Code (Détecter protocole plus loin)
        DoNormalAndControlChar(b,CODE_ESC);  // Transmettre code ESC
      }
      DoNormalAndControlChar(b,a);  // Transmettre code controle ou normal
    }                                 // FinSi interpretation des codes ESC actif
  }
  else {
    DoNormalAndControlChar(b,a);      // Transmettre code controle ou normal après interprétation pour #=>ESC
  }
  #else
    DoNormalAndControlChar(b,a);
  #endif
}
void DoNormalAndControlChar(byte b,byte a){        // Transmettre code controle ou normal après interprétation pour #=>ESC
                          #ifdef DEBUG_OPTION_IO
  if (Ser_IntCTRL_in[b]) {            // Si Interpretation des codes contrôle actif
    if (a==FILTER_CTRL_SUBST) {       // Si recu '$'
      if (Ser_GotCTRL[b]==false) {    // Si pas '$' dans caractère d'avant
        Ser_GotCTRL[b]=true;          // Le prochain caractère sera un code contrôle, ne pas transmetre '$'
      }
      else {                          // Sinon, '$' dans caractère d'avant, donc reçu '$$', transmettre juste '$'
        Ser_GotCTRL[b]=false;         // Séquence reçue = "$$" ==> Envoyer '$' seulement
        DoEscapeChar(b,a);
      }
    }                                 // FinSi reçu '$'
    else {                            // Si Pas reçu '$' ....
      if (Ser_GotCTRL[b]==true) {     // Reçu précédement '$', celui-ci introduit un code contrôle et n'est pas un '$'
        a-=0x40;                      // Le caractère reçu est transformé en code de contrôle
        Ser_GotCTRL[b]=false;         // Séquence reçue = "$"+Code contrôle affichable ==> Envoyer Code contrôle seulement
      }
      DoEscapeChar(b,a);              // Transmettre tel que
    }
  }                                   // FinSi interpretation des codes contrôle actif
  else {
    DoEscapeChar(b,a);
  }
  #else
    DoEscapeChar(b,a);
  #endif
}
void DoEscapeChar(byte b,byte a) {    // Detecter ESC/protocole
  if ((Ser_ProTransp[b]>0)||(a==0)) {              // = {0,0,0} Nombre de caractères en transparence (le 0 ne compte pas pour la transparence)
    if (a!=0) {
      Ser_ProTransp[b]--;
    }
    MultiSerialWrite(b,a);              // Transmettre sans plus d'interprétation
  }
  else {
    if (a==CODE_ESC) {                  // Code ESC reçu
      if (Ser_GotESC[b]==false) {       // Pas reçu ESC ESC
        Ser_GotESC[b]=true;             // On note de traiter ESX au prochain caractère reçu
      }
      else {                            // Recu ESC ESC
        MultiSerialWrite(b,a);          // Envoyer ESC seul, on traitera le second au prochain passage
      }
    }                                   // FinSi code ESC reçu
    else {                              // Si Pas code ESC reçu
      if (Ser_GotESC[b]==true) {        // Si le précédent était un ESC ==> Protocole ou non ?
        Ser_ProBuf[b][0]=a;
        Ser_ProBufIdx[b]=1;
        Ser_GotESC[b]=false;
        switch(a) {
          case CODE_PRO1:
            Ser_GotPRO[b]=1;
            break;
          case CODE_PRO2:
            Ser_GotPRO[b]=2;
            break;
          case CODE_PRO3:
            Ser_GotPRO[b]=3;
            break;
          case 0x61:                      // a : Demande position curseur ==> Retourne US X/Y au seul module demandeur
            SerialWrite(b,CODE_US);
            SerialWrite(b,0x41);
            SerialWrite(b,0x41);
            break;
          default:
            MultiSerialWrite(b,CODE_ESC); // Transmettre ESC
            MultiSerialWrite(b,a);        // Transmettre code controle ou normal
        }
      }                                   // FinSi le précédent était un ESC
      else {                              // Pas ESC recu et le précédent n'était pas un ESC
        if (Ser_GotPRO[b]>0) {            // Si séquence protocole en cours
          Ser_ProBuf[b][Ser_ProBufIdx[b]++]=a;
          if ((--Ser_GotPRO[b])==0) {     // Fin de séquence protocole ==> Interprétation de la séquence protocole
            #ifdef DEBUG_OPTION
            if (debug_Protocole & debug_protocol) {        // SiDebuggage protocole
              int x,y,z;
              MultiSerialWrite(b,'<');  
              MultiSerialWrite(b,'P');  
              MultiSerialWrite(b,'R');  
              MultiSerialWrite(b,'O');  
              MultiSerialWrite(b,(Ser_ProBuf[b][0])-8);
              for (x=0;x<(Ser_ProBuf[b][0])-0x38;x++) {
                y=Ser_ProBuf[b][x+1];
                MultiSerialWrite(b,' ');  
                MultiSerialWrite(b,'0');  
                MultiSerialWrite(b,'x');  
                z=((y & 0xf0)>>4)+0x30;
                if (z>0x39) { z+=7; }
                MultiSerialWrite(b,z);  
                z=(y & 0x0f)+0x30;
                if (z>0x39) { z+=7; }
                MultiSerialWrite(b,z);  
              }
              MultiSerialWrite(b,'>');  
            }                             // FinSi Debuggage protocole
            #endif
            int x,y,z;
            x=Ser_ProBuf[b][1];
            y=Ser_ProBuf[b][2];
            z=Ser_ProBuf[b][3];
            switch ((Ser_ProBuf[b][0])-0x38) {
              case 1:   // PRO1 (ESC9)
                switch (x) {
                  case 0x7b:        // g : ENQROM
                    SerialWrite(b,CODE_SOH);
                    SerialWrite(b,CODE_ROM[0]);
                    SerialWrite(b,CODE_ROM[1]);
                    if (padx3_status) {
                      SerialWrite(b,CODE_ROM[2]+0x40);                      
                    }
                    else {
                      SerialWrite(b,CODE_ROM[2]);
                    }
                    SerialWrite(b,CODE_EOT);
                    break;
                    
                  case 0x68:        // h : Connexion
                    // Impossible de connecter un module SlaveOnly
                    // AT+ROLE=1 .... KO
                    break;
                  case 0x67:        // g : Déconnexion
                    DeconnecteModem();
                    break;
                  case 0x70:        // p : Demande status tyerminal
                    EnvoiStatusTerminal(b);
                    break;
                  case 0x72:        // r : Demande status fonctionnement
                    EnvoiStatusFonctionnement(b);
                    break;
                  case 0x74:        // t : Demande status vitesse
                    EnvoiStatusVitesse(b);
                    break;
                  case 0x76:        // v : Demande status protocole
                    ReponseStatusProtocole(b);
                    break;
                  case 0x7f:        // : Reset
                    EnvoiChangementStatus(0x5e);
                    break;
                }
                break;
              case 2:   // PRO2 (ESC:)
                switch (x) {
                  #ifdef DEBUG_OPTION
                  case 0x40:              // @ : Debug mode - Bit 0=Scroll, Bit 1=DebugProtocole, Bit 2=Trace
                    #ifdef SCROLL_OPTION
                    if (y&1) {            // Activer scroll
                      scroll=true;
                    }
                    else {
                      scroll=false;
                    }
                    #endif
                    if (y&2) {            // Activer debug protocole
                      debug_Protocole|=0x01;
                    }
                    else {
                      debug_Protocole&=~0x01;
                    }
                    if (y&4) {            // Activer trace
                      debug_Protocole|=0x02;
                    }
                    else {
                      debug_Protocole&=~0x02;
                    }
                    break;
                  case 0x41:              // A : Bits 0/1/2 : Affcher Debug status/Aiguilage/statusbis
                    if (y&1) {            // Afficher debug status
                      DisplayDebugStatus();
                    }
                    if (y&2) {            // Activer debug aiguillage
                      DisplayDebugAiguillage();
                    }                    
                    if (y&4) {            // Activer debug statusbis
                      DisplayDebugStatusBis();
                    }                    
                    break;
                  #endif
                  #ifdef DEBUG_OPTION_IO
                  case 0x42:    // B : (DEBUG) : Filtre en sortie OFF d-X ou Y Non CTRL/Non ESC      X=PQRS/ Y=XYZ[      (Ecran/Clavier/Modem/Prise)
                  case 0x43:    // C : (DEBUG) : Filtre en sortie ON  e-X ou Y CTRL/ESC
                    if ((y>=0x50)&&(y<0x58)) {          // C'est un code émeteur => CTRL
                      y-=0x50;
                      if (Ser_MapDev1[y]!=0xff) {                   // Si le module existe
                        if (x&1) {                                  // Filtrer CTRL x => ^x en sortie
                          Ser_FltCTRL_out[Ser_MapDev1[y]]=true;     // Recoit de nouveau les acquitements (plus éventuellement les diffusions)
                          //ReponseStatusProtocole(b);               // Envoi StatusProtocole
                        }
                        else {                                      // Ne plus filtrer CTRL x => ^x en sortie
                          //if (b==Ser_MapDev1[y]) {                  // Seulement valable pour le module initiateur de la demande
                            Ser_FltCTRL_out[Ser_MapDev1[y]]=false;         // Ne reçoit plus ses acquitements [Même si en diffusion, et pas d'acquitement pour ce changement 
                                                                           // - ne recoit plus non plus les diffusions même si le paramètre est conservé]
                          //}
                        }
                      }
                    }
                    else if ((y>=0x58)&&(y<0x60)) {     // C'est un code récepteur => ESC
                      y-=0x58;
                      if (Ser_MapDev1[y]!=0xff) {                     // Si le module existe
                        if (x&1) {                                    // Filtrer Escape x => [ESC]x en sortie
                          Ser_FltESC_out[Ser_MapDev1[y]]=true;                        
                          //ReponseStatusProtocole(b);                // Envoi StatusProtocole
                        }
                        else {                                        // Ne plus filtrer Escape x => [ESC]x en sortie
                          Ser_FltESC_out[Ser_MapDev1[y]]=false;                        
                          //ReponseStatusProtocole(b);                // Envoi StatusProtocole
                        }
                      }
                    }
                    break;
                  case 0x44:    // D : (DEBUG) : Interpretation en entree OFF d-X ou Y Non CTRL/Non ESC      X=PQRS/ Y=XYZ[      (Ecran/Clavier/Modem/Prise)
                  case 0x45:    // E : (DEBUG) : Interpretation en entree ON  e-X ou Y CTRL/ESC
                    if ((y>=0x50)&&(y<0x58)) {          // C'est un code émeteur => CTRL
                      y-=0x50;
                      if (Ser_MapDev1[y]!=0xff) {                   // Si le module existe
                        if (x&1) {                                  // Filtrer CTRL x => ^x en sortie
                          Ser_IntCTRL_in[Ser_MapDev1[y]]=true;     // Recoit de nouveau les acquitements (plus éventuellement les diffusions)
                          //ReponseStatusProtocole(b);               // Envoi StatusProtocole
                        }
                        else {                                      // Ne plus filtrer CTRL x => ^x en sortie
                          //if (b==Ser_MapDev1[y]) {                  // Seulement valable pour le module initiateur de la demande
                            Ser_IntCTRL_in[Ser_MapDev1[y]]=false;         // Ne reçoit plus ses acquitements [Même si en diffusion, et pas d'acquitement pour ce changement 
                                                                           // - ne recoit plus non plus les diffusions même si le paramètre est conservé]
                          //}
                        }
                      }
                    }
                    else if ((y>=0x58)&&(y<0x60)) {     // C'est un code récepteur => ESC
                      y-=0x58;
                      if (Ser_MapDev1[y]!=0xff) {                     // Si le module existe
                        if (x&1) {                                    // Filtrer Escape x => [ESC]x en sortie
                          Ser_IntCTRL_in[Ser_MapDev1[y]]=true;                        
                          //ReponseStatusProtocole(b);                // Envoi StatusProtocole
                        }
                        else {                                        // Ne plus filtrer Escape x => [ESC]x en sortie
                          Ser_IntCTRL_in[Ser_MapDev1[y]]=false;                        
                          //ReponseStatusProtocole(b);                // Envoi StatusProtocole
                        }
                      }
                    }
                    break;
                  #endif
                  case 0x31:    // 1 : Mode téléinfo (80 cols)
                    if (y==0x7d) {
                      SerialWrite(b,CODE_ESC); // Retourner l'info changement mode
                      SerialWrite(b,']');      // CSI
                      SerialWrite(b,0x3f);     // Envoyer au modem l'info changement mode
                      SerialWrite(b,0x7a);     // [sortie du mode teleinfo ==> Videotext par CSI 0x3f 0x7b qui renvoie SEP/0x5e]
                      // protocole bloqué
                    }
                  case 0x32:    // 2 : Mode mixte
                    switch (y) {
                      case 0x7d:    // Passage de mode videotext en mixte (80 cols)
                        EnvoiChangementStatus(0x70);
                        fonctionnement_status|=0x0b;
                        clavier_status|=0x01;
                        break;
                      case 0x7e:    // Passage de mode mixte en videotext (40 cols)
                        EnvoiChangementStatus(0x71);
                        fonctionnement_status&=0x0b;
                        clavier_status&=0x01;
                        break;
                    }
                    break;
                  case 0x62:    // b-X ou Y TO Demande status aiguillage d'un module    X=PQRS/ Y=XYZ[      (Ecran/Clavier/Modem/Prise)
                    if ((y>=0x50)&&(y<0x58)) {          // C'est un code émeteur => Aiguillages depuis ce module
                      y-=0x50;
                      if (Ser_MapDev1[y]!=0xff) {                   // Si le module existe
                        #ifdef DEBUG_OPTION
                        if (debug_Protocole) {          // SiDebuggage protocole
                          sprintf_P(debug_text0, PSTR("Devs_minitel %x Devs_interne %x"),y,Ser_MapDev1[y]);

                          Serial.print(F("PRO2-TO-Emeteur"));
                          DisplayDeviceName(Ser_MapDev1[y]);
                          Serial.println();
                          Serial.println(debug_text0);
                        }                        
                        #endif
                        EnvoieStatusAiguillage(b,y,false);                        
                      }
                    }
                    else if ((y>=0x58)&&(y<0x60)) {     // C'est un code récepteur => Aiguillages vers ce module
                      y-=0x58;
                      if (Ser_MapDev1[y]!=0xff) {                   // Si le module existe
                        #ifdef DEBUG_OPTION
                        if (debug_Protocole) {          // SiDebuggage protocole
                          sprintf_P(debug_text0, PSTR("Devs_minitel %x Devs_interne %x"),y,Ser_MapDev1[y]);

                          Serial.print(F("PRO2-TO-Recepteur"));
                          DisplayDeviceName(Ser_MapDev1[y]);
                          Serial.println();
                          Serial.println(debug_text0);
                        }
                        #endif
                        EnvoieStatusAiguillage(b,y,true);                        
                      }
                    }
                    break;
                  case 0x64:    // d-X ou Y : Non diffusion/Non retour d'acquitement      X=PQRS/ Y=XYZ[      (Ecran/Clavier/Modem/Prise)
                  case 0x65:    // e-X ou Y : Diffusion/Retour d'acquitement
                    if ((y>=0x50)&&(y<0x58)) {          // C'est un code émeteur => Retour/NonRetour
                      y-=0x50;
                      if (Ser_MapDev1[y]!=0xff) {                   // Si le module existe
                        #ifdef DEBUG_OPTION
                        if (debug_Protocole) {          // SiDebuggage protocole
                          sprintf_P(debug_text0, PSTR("Devs_minitel %x Devs_interne %x"),y,Ser_MapDev1[y]);
                        }
                        #endif
                        if (x&1) {                                  // Retour d'acquitement
                          #ifdef DEBUG_OPTION
                          if (debug_Protocole) {        // SiDebuggage protocole
                            Serial.print(F("PRO2-Retour-"));
                            DisplayDeviceName(Ser_MapDev1[y]);
                            Serial.println();
                            Serial.println(debug_text0);
                          }
                          #endif
                          Ser_NoRet[Ser_MapDev1[y]]=false;          // Recoit de nouveau les acquitements (plus éventuellement les diffusions)
                          ReponseStatusProtocole(b);                // Envoi StatusProtocole
                        }
                        else {                                      // Non retour d'acquitement
                          #ifdef DEBUG_OPTION
                          if (debug_Protocole) {        // SiDebuggage protocole
                            Serial.print(F("PRO2-NonRetour-"));
                            DisplayDeviceName(Ser_MapDev1[y]);
                            Serial.println();
                            Serial.println(debug_text0);
                          }
                          #endif
                          if (b==Ser_MapDev1[y]) {                  // Seulement valable pour le module initiateur de la demande
                            Ser_NoRet[Ser_MapDev1[y]]=true;         // Ne reçoit plus ses acquitements [Même si en diffusion, et pas d'acquitement pour ce changement 
                                                                    // - ne recoit plus non plus les diffusions même si le paramètre est conservé]
                          }
                        }
                      }
                    }
                    else if ((y>=0x58)&&(y<0x60)) {     // C'est un code récepteur => Diffusion/NonDiffusion
                      y-=0x58;
                      if (Ser_MapDev1[y]!=0xff) {                   // Si le module existe
                        #ifdef DEBUG_OPTION
                        if (debug_Protocole) {          // SiDebuggage protocole
                          sprintf_P(debug_text0, PSTR("Devs_minitel %x Devs_interne %x"),y,Ser_MapDev1[y]);
                        }
                        #endif
                        if (x&1) {        // Diffusion
                          #ifdef DEBUG_OPTION
                          if (debug_Protocole) {        // SiDebuggage protocole
                            Serial.print(F("PRO2-Diffusion-"));
                            DisplayDeviceName(Ser_MapDev1[y]);
                            Serial.println();
                            Serial.println(debug_text0);
                          }
                          #endif
                          Ser_NoDif[Ser_MapDev1[y]]=false;                        
                          ReponseStatusProtocole(b);                // Envoi StatusProtocole
                        }
                        else {            // NonDiffusion
                          #ifdef DEBUG_OPTION
                          if (debug_Protocole) {        // SiDebuggage protocole
                            Serial.print(F("PRO2-NonDiffusion-"));
                            DisplayDeviceName(Ser_MapDev1[y]);
                            Serial.println();
                            Serial.println(debug_text0);
                          }
                          #endif
                          Ser_NoDif[Ser_MapDev1[y]]=true;                        
                          ReponseStatusProtocole(b);                // Envoi StatusProtocole
                        }
                      }
                    }
                    break;
                  case 0x66:    // f : Transparence protocole
                    Ser_ProTransp[b]=y;
                    SerialWrite(b,CODE_SEP);
                    SerialWrite(b,0x57);
                    break;
                  case 0x69:    // i : START
                    switch (y) {
                      case 0x43:    // C : Rouleau
                        fonctionnement_status|=0x02;
                        EnvoiStatusFonctionnement(b);
                        break;
                      case 0x44:    // D : PCE
                        if (b==MODULE_MODEM) {  // Si en provenance du modem
                          ;                     // Activer la PCE
                          ;                     // Envoi status fonctionnement au modem
                          ;                     // Envoi SEP 0x56 au périphérique
                          SerialWrite(MODULE_PRISE,CODE_SEP); // Envoyer a la prise l'info changement PCE
                          SerialWrite(MODULE_PRISE,0x56);     //
                          fonctionnement_status|=0x04;
                        }
                        else {                  // Si pas en provenance du modem
                          SerialWrite(MODULE_MODEM,CODE_SEP); // Envoyer au modem la demande d'activation
                          SerialWrite(MODULE_MODEM,0x4a);     //
                        }
                        break;
                      case 0x45:    // E : Minuscules
                        fonctionnement_status|=0x08;
                        EnvoiStatusFonctionnement(b);
                        break;
                    }
                    break;
                  case 0x6a:    // j : STOP
                    switch (y) {
                      case 0x43:    // C : Rouleau
                        fonctionnement_status&=~0x02;
                        EnvoiStatusFonctionnement(b);
                        break;
                      case 0x44:    // D : PCE
                        if (b==MODULE_MODEM) {  // Si en provenance du modem
                          ;                     // Arrêt la PCE
                          ;                     // Envoi status fonctionnement au modem
                          ;                     // Envoi SEP 0x56 au périphérique
                          SerialWrite(MODULE_PRISE,CODE_SEP); // Envoyer a la prise l'info changement PCE
                          SerialWrite(MODULE_PRISE,0x56);     //
                          fonctionnement_status&=~0x04;
                        }
                        else {                  // Si pas en provenance du modem
                          SerialWrite(MODULE_MODEM,CODE_SEP); // Envoyer au modem la demande d'arrêt
                          SerialWrite(MODULE_MODEM,0x4b);     //
                        }
                      break;
                      case 0x45:    // E : Minuscules
                        fonctionnement_status&=~0x08;
                        EnvoiStatusFonctionnement(b);
                        break;
                    }
                    break;
                  case 0x6b:    // k : PROG - Définition vitesse prise
                    if ((y&7)==(y&38)>>3) {  // Vitesse en entrée = vitesse en sortie
                      switch (y&7) {
                      case 1:
                      case 2:
                      case 4:
                      case 6:
                      case 7:
                        switch (y&7) {
                        case 1:
                          vitesse_prise=75;
                          break;
                        case 2:
                          vitesse_prise=300;
                          break;
                        case 4:
                          vitesse_prise=1200;
                          break;
                        case 6:
                          vitesse_prise=4800;
                          break;
                        case 7:
                          vitesse_prise=9600;
                          break;
                        }
                        vitesse_status=y&7;
                        // Changement de vitesse prise
                        break;
                      }
                    }
                    EnvoiStatusVitesse(b);
                    break;
                  case 0x72:    // r : Demande status clavier
                    if (y==0x59) {  // Y : Recepteur clavier
                      EnvoiStatusClavier(b);
                    }
                    break;
                  case 0x7c:    // l : Copie écran
                    if (y==0x6a) {
                      ;   // Copie écran en jeu Français
                    }
                    else if (y==0x6b) {
                      ;   // Copie écran en jeu Américain
                    }
                    break;
                }
              case 3:   // PRO3 (ESC;)
                switch (x) {
                  case 0x60:      // ` OFF - X/Y    X=PQRS/ Y=XYZ[      (Ecran/Clavier/Modem/Prise)
                  case 0x61:      // a ON  - X/Y
                    if ((y>=0x50)&&(y<0x58)&&(z>=0x58)&&(z<0x60)) {       // Si caractères X et Y acceptables
                      y-=0x50;
                      z-=0x58;
                      if ((Ser_MapDev1[y]!=0xff)&&(Ser_MapDev1[z]!=0xff)) { // Si module demandé existant
                        #ifdef DEBUG_OPTION
                        if (debug_Protocole) {          // SiDebuggage protocole
                          sprintf_P(debug_text0, PSTR("Devs_minitel %x-%x Devs_interne %x-%x == %x"),y,z,Ser_MapDev1[y],Ser_MapDev1[z],Ser_Aiguill[Ser_MapDev1[y]][Ser_MapDev1[z]]);
                        }
                        #endif
                        if (y==z) { // Blocage/Déblocage d'un module
                          #ifdef DEBUG_OPTION
                          if (debug_Protocole) {        // SiDebuggage protocole
                            sprintf_P(debug_text1, PSTR(" Ser_ModBlocked[Ser_MapDev1[%x]]= %x"), y,Ser_ModBlocked[Ser_MapDev1[y]]);
                            sprintf_P(debug_text2, PSTR("&Ser_ModBlocked[Ser_MapDev1[%x]] = %04x"), y,&Ser_ModBlocked[Ser_MapDev1[y]]);
                          }
                          #endif
                          if (x&1) {            // Déblocage
                            #ifdef DEBUG_OPTION
                            if (debug_Protocole) {        // SiDebuggage protocole
                              Serial.print(F("PRO3-OFF(UnBlock)-"));
                              DisplayDeviceName(Ser_MapDev1[y]);
                              Serial.println();
                              Serial.println(debug_text0);
                              Serial.println(debug_text1);
                              Serial.println(debug_text2);
                            }
                            #endif
                            Ser_ModBlocked[Ser_MapDev1[y]]=false;
                          }
                          else {                // Blocage
                            #ifdef DEBUG_OPTION
                            if (debug_Protocole) {        // SiDebuggage protocole
                              Serial.print(F("PRO3-ON(Block)-"));
                              DisplayDeviceName(Ser_MapDev1[y]);
                              Serial.println();
                              Serial.println(debug_text0);
                              Serial.println(debug_text1);
                              Serial.println(debug_text2);
                            }
                            #endif
                            Ser_ModBlocked[Ser_MapDev1[y]]=true;
                          }
                          #ifdef DEBUG_OPTION
                          if (debug_Protocole) {        // SiDebuggage protocole
                            sprintf_P(debug_text1, PSTR(" Ser_ModBlocked[Ser_MapDev1[%x]]= %x"), y,Ser_ModBlocked[Ser_MapDev1[y]]);
                            sprintf_P(debug_text2, PSTR("&Ser_ModBlocked[Ser_MapDev1[%x]] = %04x"), y,&Ser_ModBlocked[Ser_MapDev1[y]]);
                            Serial.println(debug_text1);
                            Serial.println(debug_text2);
                          }
                          #endif
                        }
                        else {      // Activation/Désactivation d'un aiguillage
                          #ifdef DEBUG_OPTION
                          if (debug_Protocole) {        // SiDebuggage protocole
                            sprintf_P(debug_text1, PSTR(" Ser_Aiguill[Ser_MapDev1[%x]][Ser_MapDev1[%x]] = %x"), y,z,Ser_Aiguill[Ser_MapDev1[y]][Ser_MapDev1[z]]);
                            sprintf_P(debug_text2, PSTR("&Ser_Aiguill[Ser_MapDev1[%x]][Ser_MapDev1[%x]] = %04x"), y,z,&Ser_Aiguill[Ser_MapDev1[y]][Ser_MapDev1[z]]);
                          }
                          #endif
                          if (x&1) {          // Activation
                            #ifdef DEBUG_OPTION
                            if (debug_Protocole) {        // SiDebuggage protocole
                              Serial.print(F("PRO3-ON-"));
                              DisplayDeviceName(Ser_MapDev1[y]);
                              DisplayDeviceName(Ser_MapDev1[z]);
                              Serial.println();
                              Serial.println(debug_text0);
                              Serial.println(debug_text1);
                              Serial.println(debug_text2);
                            }
                            #endif
                            Ser_Aiguill[Ser_MapDev1[y]][Ser_MapDev1[z]]=true;
                          }
                          else {              // Désactivation
                            #ifdef DEBUG_OPTION
                            if (debug_Protocole) {        // SiDebuggage protocole
                              Serial.print(F("PRO3-OFF-"));
                              DisplayDeviceName(Ser_MapDev1[y]);
                              DisplayDeviceName(Ser_MapDev1[z]);
                              Serial.println();
                              Serial.println(debug_text0);
                              Serial.println(debug_text1);
                              Serial.println(debug_text2);
                            }
                            #endif
                            Ser_Aiguill[Ser_MapDev1[y]][Ser_MapDev1[z]]=false;
                          }
                          #ifdef DEBUG_OPTION
                          if (debug_Protocole) {        // SiDebuggage protocole
                            sprintf_P(debug_text1, PSTR(" Ser_Aiguill[Ser_MapDev1[%x]][Ser_MapDev1[%x]] = %x"), y,z,Ser_Aiguill[Ser_MapDev1[y]][Ser_MapDev1[z]]);
                            sprintf_P(debug_text2, PSTR("&Ser_Aiguill[Ser_MapDev1[%x]][Ser_MapDev1[%x]] = %04x"), y,z,&Ser_Aiguill[Ser_MapDev1[y]][Ser_MapDev1[z]]);
                            Serial.println(debug_text1);
                            Serial.println(debug_text2);
                          }
                          #endif
                        }
                        if (!Ser_NoRet[b]) {                                // Retour_d_acquitement_autorisé[b])
                          EnvoieStatusAiguillage(b,z,true);
                        }
                        //
                        // traiter la diffusion aux autres modules
                        //
                        for (y=0;y<TOTALSERIAL;y++) {   // Pour chaque module
                          if ((b!=MODULE_PRISE)&&(!Ser_NoRet[MODULE_PRISE])&&(!Ser_NoDif[MODULE_PRISE])){ 
                            EnvoieStatusAiguillage(MODULE_PRISE,z,true);
                          }
                        }
                      }                                                     // FinSi module demandé existant
                    }                                                     // FinSi caractères X et Y acceptables
                  break;
                  case 0x69:    // i : START etendu
                    switch (y) {
                    case 0x59:  // Y : Recepteur clavier
                      switch (z) {
                      case 0x41:    // A : Clavier étendu
                        clavier_status|=0x01;
                        EnvoiStatusClavier(b);
                        break;
                      case 0x43:    // C : Curseurs non CSI
                        clavier_status|=0x04;
                        EnvoiStatusClavier(b);
                        break;
                      }
                      break;
                    }
                    break;
                  case 0x6a:    // j : STOP etendu
                    switch (y) {
                    case 0x59:  // Y : Recepteur clavier
                      switch (z) {
                      case 0x41:    // A : Clavier étendu
                        clavier_status&=~0x01;
                        EnvoiStatusClavier(b);
                        break;
                      case 0x43:    // C : Curseurs non CSI
                        clavier_status&=~0x04;
                        EnvoiStatusClavier(b);
                        break;
                      }
                      break;
                    }
                    break;
                }
                break;
            }
          }
        }                                 // FinSi séquence protocole en cours
        else {                            // Si séquence protocole pas en cours
          MultiSerialWrite(b,a);          // Transmettre sans plus d'interprétation
        }
      }
    }                                     // FinSi Pas code ESC reçu
  }                                       // FinSi Pas en cours de transparence
}

void EnvoieStatusAiguillage(byte b,byte z,bool recepteur) { // PRO2 TO ==> PRO3 FROM
  // b=module destination
  // z=module concerné par le status
byte x;
byte y=0x40;
                          
  SerialWrite(b,0x1b);
  SerialWrite(b,0x3b);  // PRO3
  SerialWrite(b,0x63);  // c : FROM
  if (recepteur) {
    SerialWrite(b,0x58+z);// Module demandeur et concerné par l'acquitement
    for (x=0;x<TOTALSERIAL;x++) {   // Pour chaque module
      if (x!=Ser_MapDev1[z]) {                   // Si pas le module originaire de la demande
        if (Ser_Aiguill[x][Ser_MapDev1[z]]) {    // Si l'aiguillage est actif depuis le module balayé vers le module originaire de la demande
          y|=Ser_MapDev[x];
        }
      }
      else {                        // Si le module balayé est le module originaire de la demande ==> Etat bloqué ou non ?
        if (!Ser_ModBlocked[x]) {   // Si non bloqué
          y|=Ser_MapDev[x];         // Indiquer l'état non bloqué du module
        }
      }
    }
  }
  else {
    SerialWrite(b,0x50+z);// Module demandeur et concerné par l'acquitement
    for (x=0;x<TOTALSERIAL;x++) {   // Pour chaque module
      if (x!=Ser_MapDev1[z]) {                   // Si pas le module originaire de la demande
        if (Ser_Aiguill[Ser_MapDev1[z]][x]) {    // Si l'aiguillage est actif depuis le module balayé vers le module originaire de la demande
          y|=Ser_MapDev[x];
        }
      }
      else {                        // Si le module balayé est le module originaire de la demande ==> Etat bloqué ou non ?
        if (!Ser_ModBlocked[x]) {   // Si non bloqué
          y|=Ser_MapDev[x];         // Indiquer l'état non bloqué du module
        }
      }
    }
  }
  SerialWrite(b,y);// Status du module
}
void ReponseStatusProtocole(byte b){                // Envoi ReponseStatusProtocole
  byte y=0x40;
  
#ifdef DEBUG_OPTION
  if (debug_Protocole & debug_trace) {        // SiDebuggage protocole
    Serial.println(F("ReponseStatusProtocole()"));
  }
#endif
  if (Ser_NoDif[MODULE_MODEM]) { // Si acquitements diffusés vers modem
    y|=0x01;
  }
  if (Ser_NoDif[MODULE_PRISE]) { // Si acquitements diffusés vers prise
    y|=0x02;
  }
  if (Ser_NoRet[MODULE_MODEM]) { // Si non retour acquitements diffusés vers modem
    y|=0x04;
  }
  if (Ser_NoRet[MODULE_PRISE]) { // Si non retour acquitements diffusés vers prise
    y|=0x08;
  }
  if (padx3_status) {            // Si mode PAD X3 actif
    y|=0x10;
  }
  SerialWrite(b,0x1b);
  SerialWrite(b,0x3a);
  SerialWrite(b,0x77);    // w
  SerialWrite(b,y);
}
void EnvoiStatusVitesse(byte b){                    // Envoi status vitesse
  SerialWrite(b,0x1b);
  SerialWrite(b,0x3a);
  SerialWrite(b,0x75);    // u
  SerialWrite(b,0x40|vitesse_status|(vitesse_status<<3));
}
void EnvoiStatusClavier(byte b){                    // Envoi status clavier
  SerialWrite(b,0x1b);
  SerialWrite(b,0x3b);    // PRO3
  SerialWrite(b,0x73);    // s
  SerialWrite(b,0x59);    // Y
  SerialWrite(b,0x40|clavier_status);
}
void EnvoiStatusFonctionnement(byte b) {            // Envoi status fonctionnement
  SerialWrite(b,0x1b);
  SerialWrite(b,0x3a);    // PRO2
  SerialWrite(b,0x73);    // s
  SerialWrite(b,0x40|fonctionnement_status);
}
void EnvoiStatusTerminal(byte b){                   // Envoi status terminal
  SerialWrite(b,0x1b);
  SerialWrite(b,0x3a);    // PRO2
  SerialWrite(b,0x71);    // q
  SerialWrite(b,0x40|terminal_status);
}
void EnvoiChangementStatus(byte Status){            // Envoie status SEP/xx aux modules MODEM/PRISE quel que soit l'état de blocage ou d'aiguillage
  SerialWrite(MODULE_PRISE,CODE_SEP);
  SerialWrite(MODULE_PRISE,Status);
  SerialWrite(MODULE_MODEM,CODE_SEP);
  SerialWrite(MODULE_MODEM,Status);
}

void MultiSerialWrite(int From,int chr) {   // Traitement des blocages et aiguillages
  byte x;
/*
  // M1b
  // Ser0 = Prise / Serial monitor
  // Ser1 = Ecran / Tellylate
  // Ser2 = Modem / BlueTooth
  // Ser3 = Clavier / Kbd
  if (ModemIsConnected) {
    Ser_Aiguill[3][2]=true;   // Clavier->Modem
    Ser_Aiguill[2][1]=true;   // Modem->Ecran
    Ser_Aiguill[2][0]=true;   // Modem->Prise
    Ser_Aiguill[0][2]=true;   // Prise->Modem
  }
*/
  if (!Ser_ModBlocked[From]) {              // Si le module source n'est pas bloqué
    for (x=0;x<TOTALSERIAL;x++) {           // Tenter d'envoyer à tous les modules
      if (x!=From) {                        // NE pas renvoyer en echo au module source (a voir avec modem déconnecté ????)
        if (Ser_Aiguill[From][x]==true) {   // Si l'aiguillage est actif
          if (!Ser_ModBlocked[x]) {         // Si le module destinataire n'est pas bloqué
            SerialWrite(x,chr);             // Envoyer vers ce module
          }
        }
      }
    }
  }                             // FinSi le module source n'est pas bloqué
}
void SerialWrite(int To,byte chr){   // Envoie chr vers 'To' sans prise en compte du protocole mais décodage [ESC] et [SEP]
  if (To<TOTALREALSERIAL) {
    #ifdef DEBUG_OPTION_IO
    if (Ser_FltESC_out[To]) {         // Si filtrage des codes ESC en sortie
      if (chr==CODE_ESC) {            // Si code ESC
        byte z=0;
        while (FILTER_ESC_DISP[z+1]) {
          MySerial[To]->write(FILTER_ESC_DISP[z++]);
        }
        chr=FILTER_ESC_DISP[z];
      }                               // FinSi recu code ESC
    }                                 // FinSi filtrage des codes ESC en sortie
    else if (Ser_FltSEP_out[To]) {        // Filtrage - Interpretation des codes séparateur en sortie [SEP]<x>
      if (chr==CODE_SEP) {                // Si code SEP
        byte z=0;
        while (FILTER_SEP_DISP[z+1]) {
          MySerial[To]->write(FILTER_SEP_DISP[z++]);
        }
        chr=FILTER_SEP_DISP[z];
      }                               // FinSi recu code ESC
    }
    else if (Ser_FltCTRL_out[To]) {    // Si filtrage des codes contrôle en sortie
      if (chr<0x20) {                   // Si code contrôle
        MySerial[To]->write(FILTER_CTRL_DISP);
        chr+=0x40;
      }                               // FinSi recu code contrôle
    }                                 // FinSi filtrage des codes contrôle en sortie
    #endif
    MySerial[To]->write(chr);
  }
  else {  // Envoi vers le clavier ?????
    ;
  }
}

void SaveCrsr(void){
//    MySerial[ScrollDevice]->write(0x1b);  // Cursor off
//    MySerial[ScrollDevice]->write('e');
    MySerial[ScrollDevice]->write(0x1b);  // Save cursor location
    MySerial[ScrollDevice]->write('j');
}
void RestoreCrsr(void){
    MySerial[ScrollDevice]->write(0x1b);  // Restore cursor location
    MySerial[ScrollDevice]->write('k');
//    MySerial[ScrollDevice]->write(0x1b);  // Cursor on
//    MySerial[ScrollDevice]->write('f');
}

#ifdef SCROLL_OPTION
void GotoScrollLine(byte Scroll, byte MyLine){
    MySerial[ScrollDevice]->write(0x1b);  // GotoXY
    MySerial[ScrollDevice]->write('Y');
    MySerial[ScrollDevice]->write(0x20+ScrollLine[Scroll]+MyLine);
    MySerial[ScrollDevice]->write(0x20);
}
void SendScroll(byte Scroll,byte MyLine,byte Scroll_Index){
    byte SI=0;

    GotoScrollLine(Scroll,MyLine);
    while ((SI<38) && (ScrollText[Scroll][Scroll_Index + SI])) {
      MySerial[ScrollDevice]->write((ScrollText[Scroll])[Scroll_Index + SI]);
      SI++;
    }
    if (SI<38) {
      MySerial[ScrollDevice]->write(CODE_ESC);  // Erase to EOL
      MySerial[ScrollDevice]->write('K');
    }
}
void DoInitScroll(void) {
    SaveCrsr();
    byte x;
  for (x=0;x< MaxScroll;x++) {
    GotoScrollLine(x,0);
    MySerial[ScrollDevice]->write(0x1b);MySerial[ScrollDevice]->write('_');MySerial[ScrollDevice]->write('2');
    GotoScrollLine(x,1);
    MySerial[ScrollDevice]->write(0x1b);MySerial[ScrollDevice]->write('_');MySerial[ScrollDevice]->write('3');
  }
  RestoreCrsr();
}
void DoScroll (void) {
  byte x;
  char *zz;

  for (x=0;x<MaxScroll;x++) {
    if (Curr_Millis>prev_millis[x]) {
      prev_millis[x]=Curr_Millis+ScrollSpeed[x];
      zz=ScrollText[x];
      if (!(zz[Scroll_Index[x]])) {
        Scroll_Index[x]=0;
        if (x==0) {
          ScrollText0_Index++;
          if (ScrollText0_Index==MaxScroll0) {
            ScrollText0_Index=0;
          }
          ScrollText[0]=ScrollText0[ScrollText0_Index];
        }
      }
      else {
        Scroll_Index[x]++;
      }
      SaveCrsr();
      SendScroll(x,0,Scroll_Index[x]);
      SendScroll(x,1,Scroll_Index[x]);
      RestoreCrsr();
    }
  }
}
#endif

void DeconnecteModem(void){                             // Forcer changement d'état => reset SPP-C
#ifdef DEBUG_OPTION
  if (debug_Protocole & debug_trace) {        // SiDebuggage protocole
    Serial.println(F("DeconnecteModem()"));
  }
#endif
  ModemIsConnected=true;                                // Forcer changement d'état => reset SPP-C
  BT_Status_Unchanged=0;                                // Reset compteur de la limite de changement d'état
  BT_Status_Wait_Displayed=2;
  UpdateModemStatus("DECONNECT ",false);
}
void NewStatusModem(bool StatusModem){          // Traitement changement d'état modem
#ifdef DEBUG_OPTION
  if (debug_Protocole & debug_trace) {        // SiDebuggage protocole
    Serial.println(F("NewStatusModem()"));
  }
#endif
  if (ModemIsConnected==StatusModem) {                          // Changement d'état du modem
    if ((ModemIsConnected==true)&&(StatusModem == false)) {     // Déconnexion
      digitalWrite(PIN_RESET_BT,false);
      delay(100);
      digitalWrite(PIN_RESET_BT,true);
      delay(500);
      terminal_status&=~0x08;
    } else if ((ModemIsConnected==false)&&(StatusModem == true)) {    // Connexion
      terminal_status|=0x08;
    }
    EnvoiChangementStatus(0x59);
    EnvoiChangementStatus(0x53);
  }
  ModemIsConnected=StatusModem;
  DoInitProtocol();
}
void UpdateModemStatus(char *a,bool StatusModem)  { // Met à jour l'état modem à l'écran et force éventuellement la déconnexion
  UpdateStatusBT(a,BT_STATUS_LIN,BT_CONNECT_COL);
  NewStatusModem(StatusModem);
}
void UpdateStatusBT(char *a,char lin,char col) {
  
  SaveCrsr();
  MySerial[ScrollDevice]->write(0x1b);  // GotoXY
  MySerial[ScrollDevice]->write('Y');
  MySerial[ScrollDevice]->write(0x20+lin);
  MySerial[ScrollDevice]->write(0x20+col);
  while (a[0]) {
    MySerial[ScrollDevice]->write(a[0]);
    a++;
  }
  RestoreCrsr();
}
void SerialWriteProgmemString(const char *a){
  byte b;
  b=pgm_read_byte(a);
  while (b) {
    Serial.write(b);
    b=pgm_read_byte(++a);
  }
}
void SerialToWriteProgmemString(byte To,const char *a){ 
  byte b;
  b=pgm_read_byte(a);
  while (b) {
    MySerial[To]->write(b);
    b=pgm_read_byte(++a);
  }
}

#ifdef DEBUG_OPTION
void DisplayDeviceName(byte device) {       // Serial.write
  Serial.write(' ');
  //char *a;

  //a=(char*)pgm_read_word(&(Text_Name_Module[device]));
  SerialWriteProgmemString((const char*)pgm_read_word(&(Text_Name_Module[device])));
/*  char *a;
  a=(char *) Text_Name_Module[device];
  while (a[0]) {
    Serial.write(a[0]);
    a++;
  }
*/
  Serial.write(' ');
}
void DisplayDebugHeader (char *texte) {     // Serial.write
  byte a;

  Serial.println();
  for (a=0;a<strlen(texte);a++) {
    Serial.write('#');
  }
  Serial.println();
  for (a=0;a<strlen(texte);a++) {
    Serial.write(texte[a]);
  }
  Serial.println();
  for (a=0;a<strlen(texte);a++) {
    Serial.write('#');
  }
  Serial.println();
}
void DisplayDebugAiguillage(void) {         // Serial.write
  byte a;

  DisplayDebugHeader ("Etat aiguillages et acquitements.");
  Serial.println(F("Aiguillages"));
  for (a=0;a<TOTALSERIAL;a++) {
    DisplayDeviceName(a);
    Serial.write('-');
    Serial.write('>');
    if (Ser_Aiguill[a][MODULE_PRISE]||Ser_Aiguill[a][MODULE_ECRAN]||Ser_Aiguill[a][MODULE_MODEM]||Ser_Aiguill[a][MODULE_CLAVIER]) {
      if (Ser_Aiguill[a][MODULE_PRISE]) {
        DisplayDeviceName(MODULE_PRISE);
      }
      if (Ser_Aiguill[a][MODULE_ECRAN]) {
        DisplayDeviceName(MODULE_ECRAN);
      }
      if (Ser_Aiguill[a][MODULE_MODEM]) {
        DisplayDeviceName(MODULE_MODEM);
      }
      if (Ser_Aiguill[a][MODULE_CLAVIER]) {
        DisplayDeviceName(MODULE_CLAVIER);
      }
    }
    else {
      //Serial.print(F(" None"));
      SerialWriteProgmemString(Text_Name_Module_None);
    }
    Serial.println();
  }
  //Serial.println();
  Serial.println(F("Blocages"));
  if (Ser_ModBlocked[MODULE_PRISE]||Ser_ModBlocked[MODULE_ECRAN]||Ser_ModBlocked[MODULE_MODEM]||Ser_ModBlocked[MODULE_CLAVIER]) {
    for (a=0;a<TOTALSERIAL;a++) {
      if (Ser_ModBlocked[a]) {
        DisplayDeviceName(a);
      }
    }
  }
  else {
    //Serial.print(F(Text_Name_Module_None));
    SerialWriteProgmemString(Text_Name_Module_None);
  }
  Serial.println();
  Serial.println(F("Non diffusions"));
  if (Ser_NoDif[MODULE_PRISE]||Ser_NoDif[MODULE_ECRAN]||Ser_NoDif[MODULE_MODEM]||Ser_NoDif[MODULE_CLAVIER]) {
    for (a=0;a<TOTALSERIAL;a++) {
      if (Ser_NoDif[a]) {
        DisplayDeviceName(a);
      }
    }
  }
  else {
    //Serial.print(F(Text_Name_Module_None));
    SerialWriteProgmemString(Text_Name_Module_None);
  }
  Serial.println();
  Serial.println("Non retour");
  if (Ser_NoRet[MODULE_PRISE]||Ser_NoRet[MODULE_ECRAN]||Ser_NoRet[MODULE_MODEM]||Ser_NoRet[MODULE_CLAVIER]) {
    for (a=0;a<TOTALSERIAL;a++) {
      if (Ser_NoRet[a]) {
        DisplayDeviceName(a);
      }
    }
  }
  else {
    //Serial.print(F(Text_Name_Module_None));
    SerialWriteProgmemString(Text_Name_Module_None);
  }
  Serial.println();
}
void DisplayDebugStatusBis(void){           // Serial.write
  DisplayDebugHeader ("Etat modes particuliers globaux.");
  {  byte y=0x40;
  
    if (Ser_NoDif[MODULE_MODEM]) { // Si acquitements diffusés vers modem
      y|=0x01;
    }
    if (Ser_NoDif[MODULE_PRISE]) { // Si acquitements diffusés vers prise
      y|=0x02;
    }
    if (Ser_NoRet[MODULE_MODEM]) { // Si non retour acquitements diffusés vers modem
      y|=0x04;
    }
    if (Ser_NoRet[MODULE_PRISE]) { // Si non retour acquitements diffusés vers prise
      y|=0x08;
    }
    if (padx3_status) {            // Si mode PAD X3 actif
      y|=0x10;
    }


    Serial.print(F("Status protocole .... :"));
    Serial.write(y);
    Serial.write(' ');
    DisplayBin(y);
  }
  Serial.println();

  Serial.print(F("Status vitesse ...... :"));
  Serial.write(0x40|vitesse_status|(vitesse_status<<3));
  Serial.write(' ');
  DisplayBin(0x40|vitesse_status|(vitesse_status<<3));
  Serial.println();
  
  Serial.print(F("Status clavier ...... :"));
  Serial.write(0x40|clavier_status);
  Serial.write(' ');
  DisplayBin(0x40|clavier_status);
  Serial.println();

  Serial.print(F("Status fonctionnement :"));
  Serial.write(0x40|fonctionnement_status);
  Serial.write(' ');
  DisplayBin(0x40|fonctionnement_status);
  Serial.println();

  Serial.print(F("Status terminal ..... :"));
  Serial.write(0x40|terminal_status);
  Serial.write(' ');
  DisplayBin(0x40|terminal_status);
  Serial.println();

  Serial.println(F("PRO2-@ABC"));
  Serial.print(F("Status debug protocol :"));
#ifdef SCROLL_OPTION
  Serial.write(0x40|debug_Protocole|(scroll<<7));
  Serial.write(' ');
  DisplayBin(0x40|debug_Protocole|(scroll<<7));
#else
  Serial.write(0x40|debug_Protocole);
  Serial.write(' ');
  DisplayBin(0x40|debug_Protocole);
#endif
  Serial.println();

  Serial.println();
}
void DisplayDebugStatus(void) {             // Serial.write
  byte a;
  
  DisplayDebugHeader ("Etat modes particuliers communs pour tout module.");
#ifdef DEBUG_OPTION_IO
  Serial.println(F("Interpretation des codes controle en entree $<x>"));
  if (Ser_IntCTRL_in[MODULE_PRISE]||Ser_IntCTRL_in[MODULE_ECRAN]||Ser_IntCTRL_in[MODULE_MODEM]||Ser_IntCTRL_in[MODULE_CLAVIER]) {
    for (a=0;a<TOTALSERIAL;a++) {
      if (Ser_IntCTRL_in[a]) {
        DisplayDeviceName(a);
      }
    }
  }
  else {
    //Serial.print(F(Text_Name_Module_None));
    SerialWriteProgmemString(Text_Name_Module_None);
  }
  Serial.println();
  Serial.println(F("Precedent code recu est un code de controle (lors de l'interpretation du $)"));
  if (Ser_GotCTRL[MODULE_PRISE]||Ser_GotCTRL[MODULE_ECRAN]||Ser_GotCTRL[MODULE_MODEM]||Ser_GotCTRL[MODULE_CLAVIER]) {
    for (a=0;a<TOTALSERIAL;a++) {
      if (Ser_GotCTRL[a]) {
        DisplayDeviceName(a);
      }
    }
  }
  else {
    //Serial.print(F(Text_Name_Module_None));
    SerialWriteProgmemString(Text_Name_Module_None);
  }
  Serial.println();
  Serial.println(F("Interpretation des codes controle en sortie ^<x>"));
  if (Ser_FltCTRL_out[MODULE_PRISE]||Ser_FltCTRL_out[MODULE_ECRAN]||Ser_FltCTRL_out[MODULE_MODEM]||Ser_FltCTRL_out[MODULE_CLAVIER]) {
    for (a=0;a<TOTALSERIAL;a++) {
      if (Ser_FltCTRL_out[a]) {
        DisplayDeviceName(a);
      }
    }
  }
  else {
    //Serial.print(F(Text_Name_Module_None));
    SerialWriteProgmemString(Text_Name_Module_None);
  }
  Serial.println();
  Serial.println(F("Interpretation des codes controle en entree #<x>"));
  if (Ser_IntESC_in[MODULE_PRISE]||Ser_IntESC_in[MODULE_ECRAN]||Ser_IntESC_in[MODULE_MODEM]||Ser_IntESC_in[MODULE_CLAVIER]) {
    for (a=0;a<TOTALSERIAL;a++) {
      if (Ser_IntESC_in[a]) {
        DisplayDeviceName(a);
      }
    }
  }
  else {
    //Serial.print(F(Text_Name_Module_None));
    SerialWriteProgmemString(Text_Name_Module_None);
  }
  Serial.println();
  Serial.println(F("Precedent code recu est un code ESC (lors de l'interpretation du #)"));
  if (Ser_GotESC1[MODULE_PRISE]||Ser_GotESC1[MODULE_ECRAN]||Ser_GotESC1[MODULE_MODEM]||Ser_GotESC1[MODULE_CLAVIER]) {
    for (a=0;a<TOTALSERIAL;a++) {
      if (Ser_GotESC1[a]) {
        DisplayDeviceName(a);
      }
    }
  }
  else {
    //Serial.print(F(Text_Name_Module_None));
    SerialWriteProgmemString(Text_Name_Module_None);
  }
  Serial.println();
  Serial.println(F("Interpretation des codes Escape en sortie [ESC]<x>"));
  if (Ser_FltESC_out[MODULE_PRISE]||Ser_FltESC_out[MODULE_ECRAN]||Ser_FltESC_out[MODULE_MODEM]||Ser_FltESC_out[MODULE_CLAVIER]) {
    for (a=0;a<TOTALSERIAL;a++) {
      if (Ser_FltESC_out[a]) {
        DisplayDeviceName(a);
      }
    }
  }
  else {
    //Serial.print(F(Text_Name_Module_None));
    SerialWriteProgmemString(Text_Name_Module_None);
  }
  Serial.println();
  Serial.println(F("Interpretation des codes Separateur en sortie [SEP]<x>"));
  if (Ser_FltSEP_out[MODULE_PRISE]||Ser_FltSEP_out[MODULE_ECRAN]||Ser_FltSEP_out[MODULE_MODEM]||Ser_FltSEP_out[MODULE_CLAVIER]) {
    for (a=0;a<TOTALSERIAL;a++) {
      if (Ser_FltSEP_out[a]) {
        DisplayDeviceName(a);
      }
    }
  }
  else {
    //Serial.print(F(Text_Name_Module_None));
    SerialWriteProgmemString(Text_Name_Module_None);
  }
  Serial.println();
  #endif
  Serial.println(F("Transparence protocole"));
  for (a=0;a<TOTALSERIAL;a++) {
    DisplayDeviceName(a);
    DisplayHex(Ser_ProTransp[a]);
    Serial.println();
  }
  Serial.println();
}
void DisplayBin(byte chr) {     // Serial.write
  int b,c=0x80;
  byte d=8;

  Serial.write('0');
  Serial.write('%');
  while (d) {
    d--;
    b=(chr&c)>>d;
    c>>=1;
    Serial.write(b|0x30);
  }
}
void DisplayHex(byte chr) {     // Serial.write
  int b;

  Serial.write('0');
  Serial.write('x');
  b=((chr&0xf0)>>4)|0x30;
  if (b>0x39) { b+=7; }
  Serial.write(b);
  b=(chr&0x0f)|0x30;
  if (b>0x39) { b+=7; }
  Serial.write(b);
}
#endif
