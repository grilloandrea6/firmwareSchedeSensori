# FirmwareSchedeSensori

Firmware per il funzionamento delle schede sensori per la piattaforma Robotica Paquitop.

## ToDo

 - configurazione clock

        ```extern "C" void SystemClock_Config(void)  
        {  
        // clock init code here... è weak  
        }```

 - configurazione clock CAN
 - controllare filtri CAN
 - aggiungere invio distanza insieme ad allarmi



/*EEPROM 
 * 0 - byte -> 255 data already written / 0 new data
 * 1 - byte -> my CAN addr
 * 2/3 - byte -> soglia giallo
 * 4/5 - byte -> soglia rosso
 * SISTEMARE FILTRI PER FARE PASSARE TUTTO
  * SISTEMARE COSTANTI DEI COMANDI PER COMBACIARE CON COSO CENTRALE
 * CONFIGURAZIONE CLOCK
 * 
 extern "C" void SystemClock_Config(void)
{
  // clock init code here... `e weak
}
startup
 - read flash -> soglie e id can
 - check adc -> se supero mando allarme
 
 - callback can
	- set id can -> scrivo flash e ricarico variabili
	- set soglie -> scrivo flash e ricarico variabili
	
	- dist request -> leggo e invio
 */



    // 8 byte
    
    // send soglia
    // 1 command, 2 soglia gialla, 2 soglia rossa

    // messaggio allarme
    // 1 allarme tipo soglia, 1 mio id

    // request distance
    // 1 reqDist
    // ans:  1 dist, 1 myid, 2 dist las, 2 dist


startup
 - read flash -> soglie e id can
 - check adc -> se supero mando allarme
 
 - callback can
	- set id can -> scrivo flash e ricarico variabili
	- set soglie -> scrivo flash e ricarico variabili
	
	- dist request -> leggo e invio
	