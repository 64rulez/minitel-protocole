/*
 * OPTIMISATIONS RAM/ROM

Avant msg scroll en progmem
Variables : 3776
program : 18588

Après PS2_Kbd
Variables : 3925
program : 20686

Sans scroll
Variables : 2592
program : 18652

Tous les Serial.print() en serial.print(F()) et sprintf (,,) en sprintf_P(,PSTR(),) sauf DebugHeader()
Variables : 1418
program : 19026

Presque toutes les chaines en PROGMEM
Variables : 1314
program : 18950

Sans KBD (PS2 et Matrix)
Variables : 1025
program : 15026

Sans debug mais avec debug_io ($#)
Variables : 689
program : 7356

Sans debug ni debug_io ($#)
Variables : 649
program : 6980

 */
