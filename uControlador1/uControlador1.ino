/*
PRÁCTICA 22449 - Sistemes Encastats.
GESTIÓN DE AFORO Y ADMINISTRACIÓN DEL NIVEL DE OXÍGENO EN UN ESTABLECIMIENTO

AUTORES
	Dilpreet Singh - X9314789L
	Miquel Vives Marcus - 43462572R
*/

/********************************************
                Includes
*********************************************/
#include "HIB.h"
#include "timerConfig.h"
#include "SO.h"
#include <stdio.h>
#include <string.h>
#include <mcp_can_uib.h>
#include <mcp_can_uib_dfs.h>
#include <SPI.h>
#include "Terminal.h"

// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 9;

HIB hib;
SO so;
Terminal term;
MCP_CAN CAN(SPI_CS_PIN);

/********************************************
          DEFINICIÓN DE CONSTANTES
*********************************************/

// Tipos de estados del sistema
#define ESTADO_EXCELENTE 0
#define ESTADO_RIESGO 1
#define ESTADO_PELIGRO 2

#define ID_CONTROLADOR 0
#define ID_LCD 1
#define ID_SIMULACRO 2

/********************************************
 DECLARACIÓN DE VARIABLES GLOBALES COMPARTIDAS
*********************************************/
volatile uint8_t tamany_msg = 0;

//Aforo actual de la tienda
volatile uint8_t aforo = 0; 

// Estado actual de la tienda;
volatile uint8_t estado;

//Última tecla pulsada
//Lo va a leer la ISR ISR_Sensor_Puertas
//key == 1 —> aforo++; key == 2 —> aforo--
volatile uint8_t key = 0;

// Buffers para el CAN
volatile uint32_t rx_id;
volatile uint8_t rx_dlc;
volatile char rx_data [20];
volatile uint8_t rx_estado;

/********************************************
	DEFINICIÓN DE PRIORIDADES DE LAS TAREAS
*********************************************/
#define PRIO_TASK_A_ACTUALIZAR_AFORO 1
#define PRIO_TASK_A_ENVIA_AFORO 2
#define PRIO_TASK_A_MOSTRAR_AFORO_1 2
#define PRIO_TASK_A_MOSTRAR_AFORO_2 2
#define PRIO_TASK_ISR_RECEPCION_CAN 3
#define PRIO_TASK_A_DISPATCHER 4
#define PRIO_TASK_A_PANTALLA_INFORMATIVA 5
#define PRIO_TASK_A_ALARMA_OXIGENO 5

/********************************************
 DECLARACIÓN DE PRIMITIVAS DE SINCRONIZACIÓN
*********************************************/
  // Semáforos
	// Semáforos mutex
	
  
Sem sAforo;											  // Mutex para proteger la variable global aforo
Sem sMutex;											  // Semáforo para proteger la longitud del string a imprimir
Sem sEstado;										  // Semáforo para proteger la variable "estado"

	// Semáforo débil
Sem sEnvia_Aforo;									  // Semáforo débil para bloquear a la tarea A_ENVIAR_AFORO


  // MailBox
MBox mbContador_Aforo_1;							  // A_ACTUALIZAR_AFORO --(entero)--> A_MOSTRAR_AFORO_1
MBox mbContador_Aforo_2;							  // A_ACTUALIZAR_AFORO --(entero)--> A_MOSTRAR_AFORO_2
MBox mbMensajeLCD;									  //  A_DISPATCHER --(String)--> A_PANTALLA_INFORMATIVA

  // Flags y máscaras
Flag fAlarma;										  // Flag para activar para alarma y simulacro de alarma respectivamente
const unsigned char maskAlarm = 0x01; 
const unsigned char maskSimulacro = 0x02; 

Flag fSensorAforo;									  // Flag para la activación de A_ACTUALIZAR_AFORO
const unsigned char maskKeyEvent = 0x01; 

Flag fReciboCAN;									  // Flag para la activación de A_DISPATCHER
const unsigned char canRecieveEvent = 0x01; 


/********************************************
        DECLARACIÓN DE HOOK'S Y ISR's
*********************************************/
void ISR_SENSOR_PUERTAS(uint8_t newKey)
{
  key = newKey; // guardamos en la var. global 'key' la tecla pulsada
  // despertar la tarea A_ACTUALIZAR_AFORO poniendo a '1' los bits de fSensorAforo para maskKeyEvent
  so.setFlag(fSensorAforo, maskKeyEvent);
}

// ISR del bus CAN, al recibir información, la guarda en el buffer y despierta a la tarea de recepción
void ISR_RECEPCION_CAN(){
  char auxSREG;

  // Save the AVR Status Register by software
  // since the micro does not do it automatically
  auxSREG = SREG;


  if (CAN.rxInterrupt())
  {
    // Function readRxMsg() does the following actions:
    // (1) Reads received message from RX buffer and places it
    //     in a software buffer within the CAN object ('within the library')
    // (2) Clears and releases the RX buffer
    // (3) Clears the reception interrupt flag placed at the CAN controller
    CAN.readRxMsg();
    rx_id = CAN.getRxMsgId();
    rx_dlc = CAN.getRxMsgDlc();
    so.waitSem(sMutex);
    tamany_msg = rx_dlc;
    so.signalSem(sMutex);
    
    if(rx_id == 0)
    {
      CAN.getRxMsgData((byte*) &rx_estado);
    }else{
      memset(rx_data, 0, sizeof rx_data);
      CAN.getRxMsgData((byte*) &rx_data);
    }

    // despertar la tarea A_DISPATCHER poniendo a '1' los bits de fReciboCAN para canRecieveEvent
    so.setFlag(fReciboCAN, canRecieveEvent);
  }

  // Restore the AVR Status Register by software
  // since the micro does not do it automatically
  SREG = auxSREG;
}


// Hook FOR TICKISR
void timer5Hook ()
{
  so.updateTime(); // Call SO to update time (tick) and manage tasks
}


/********************************************
		IMPLEMENTACIÓN DE LAS TAREAS
*********************************************/

/*
Activación: mediante la ISR ISR_Sensor_Puertas
Rol: Incrementar o decrementar el aforo en función de la tecla 1 o 2 respectivamente.
*/
void A_ACTUALIZAR_AFORO()
{
  uint8_t unidadesAforo = 0; //Para mostrarlo en el 7-Segment De la derecha
  uint8_t decenasAforo = 0; //Para mostrarlo en el 7-Segment De la izquierda
  
    //Para evitar hacer lecturas sobre la variable aforo con semáforo,
    //copiamos el valor en la var. local aforoLocal
  uint8_t aforoLocal = 0;
  
  while(1)
  {
      // tarea bloqueada hasta que los bits del flag fSensorAforo 
      // sean modificados a '1' respecto bits del maskKeyEvent
    so.waitFlag(fSensorAforo, maskKeyEvent);
      // Hacemos un clear del flag para no procesar dos veces el mismo evento
    so.clearFlag(fSensorAforo, maskKeyEvent);

    //detectar qué boton se ha pulsado
    if(key == 0){
      so.waitSem(sAforo);
        aforo++;
        aforoLocal = aforo; 
      so.signalSem(sAforo);
    }else if (key == 1){
      so.waitSem(sAforo);
        aforo--;
        aforoLocal = aforo;
      so.signalSem(sAforo);
    }
    
    decenasAforo = aforoLocal/10;
    unidadesAforo = aforoLocal - (10*decenasAforo); 


      // Enviamos el aforo en decenas al 7-Segment de izq. (activación meidante mailbox)
    so.signalMBox(mbContador_Aforo_1, (byte*) &decenasAforo);

      // Enviamos el aforo en unidades al 7-Segment de derecha (activación meidante mailbox)
    so.signalMBox(mbContador_Aforo_2, (byte*) &unidadesAforo);
         
    // Se activa la tarea A_ENVIA_AFORO con un semáforo (esta tarea envía aforo por el bus CAN)
    so.signalSem(sEnvia_Aforo);
  }
  
}

/*
Activación: mailbox mbContador_Aforo_1 (A_Actualizar_Aforo -> A_Mostrar_Aforo_1).
Rol: muestra el aforo en decenas en el 7-Segment izq.
*/
void A_MOSTRAR_AFORO_1()
{
  uint8_t decenasAforo;
  uint8_t *decenasAforoMsg;

  while(1)
  {
    so.waitMBox(mbContador_Aforo_1,(byte**) &decenasAforoMsg);
    decenasAforo = *decenasAforoMsg;
    //Display on 7-Segment-Left
    hib.d7sPrintDigit(decenasAforo, hib.LEFT_7SEG_DIS);
  }
}

/*
Activación: mailbox mbContador_Aforo_2 (A_Actualizar_Aforo -> mbContador_Aforo_2).
Rol: muestra el aforo en unidades en el 7-Segment derecho.
*/
void A_MOSTRAR_AFORO_2()
{
  uint8_t unidadesAforo;
  uint8_t *unidadesAforoMsg;
  while(1)
  {
    so.waitMBox(mbContador_Aforo_2, (byte**) &unidadesAforoMsg);
    unidadesAforo = *unidadesAforoMsg;

    //Display on 7-Segment-Right
    hib.d7sPrintDigit(unidadesAforo, hib.RIGHT_7SEG_DIS);
  }
}


/*
Activación: semáforo débil sEnviaAforo
Rol: envia el aforo por el bus CAN al uControlador2
*/
void A_ENVIA_AFORO()
{
  uint8_t auxAforo = 0;
  const uint32_t tx_id = 0x00001000;
  const uint8_t tx_dlc = sizeof(uint8_t);
  while(1)
  {
    so.waitSem(sEnvia_Aforo);
    
    so.waitSem(sAforo);
        auxAforo = aforo; //actualizamos el aforo local
    so.signalSem(sAforo);

    // Check whether or not the TX buffer is available (no Tx still pending)
    // to request transmission of data
    if (CAN.checkPendingTransmission() != CAN_TXPENDING)
    {
      CAN.sendMsgBufNonBlocking(tx_id, CAN_STDID, tx_dlc, (INT8U *) &auxAforo);
    }
  }
}

/*
Activación: ISR_Recepcion_CAN
Rol: recibe datos del bus CAN y acciona de diferentes formas, según el id de recepción
*/
void A_DISPATCHER()
{
  const uint8_t size_buffer = 30; 
  char buff[size_buffer]; //Buffer donde guardaremos los datos recibidos
  while(1)
  {
      // tarea bloqueada hasta que los bits del flag fReciboCAN 
      // sean modificados a '1' respecto bits del canRecieveEvent
    so.waitFlag(fReciboCAN, canRecieveEvent);
      // Clear the flag fReciboCAN to not process the same event twice
    so.clearFlag(fReciboCAN, canRecieveEvent);

    //Procesar datos recibidos del CAN y actuar

      //Activar Pantalla Informative si es necesario
      //Si informa sobre el estado {Excelente = 2, Riesgo = 1, Peligroso = 0} 
      //Activar Alarma Oxigeno si es necessario

    memset(buff,0,30); // Hacemos un clear del buffer donde tenemos que guardar datos
    
      // si rx_id == 0 informa sobre el estado {Excelente = 0, Riesgo = 1, Peligroso = 2}
    if (rx_id == ID_CONTROLADOR) 
    {
      so.waitSem(sMutex);
      tamany_msg = size_buffer; // Cambiamos el tamaño del mensaje porque ahora construimos una cadena de carácteres
      so.signalSem(sMutex);

      so.waitSem(sEstado); 
      estado = rx_estado; // guardamos en una var local el estado recibido
      so.signalSem(sEstado);
      
      switch(rx_estado){ // condicionamos según el estado
        case ESTADO_EXCELENTE:
          memset(buff, 0, sizeof buff);
          strcpy(buff,"Estado tienda: /Excelente");
          so.signalMBox(mbMensajeLCD, (byte*) buff);
          break;

         case ESTADO_RIESGO:
          memset(buff, 0, sizeof buff);
          strcpy(buff,"Estado tienda: /Riesgo");
          so.signalMBox(mbMensajeLCD, (byte*) buff);
          break;

         case ESTADO_PELIGRO:
          memset(buff, 0, sizeof buff);
          strcpy(buff,"Estado tienda: /Peligro");
          so.signalMBox(mbMensajeLCD, (byte*) buff);
          so.setFlag(fAlarma, maskAlarm);
          break;
      }
    } 
    else if (rx_id == ID_LCD) // En este caso solamente Imprime por el LCD

    {
        so.signalMBox(mbMensajeLCD, (byte*) rx_data);
    }
    else if (rx_id == ID_SIMULACRO) // Activa la alarma para Simulacro (Ruido diferente)
    {
        so.setFlag(fAlarma, maskSimulacro);
    }
  }
}

/*
Activación: mailbox mbMensajeLCD (A_DISPATCHER -> mbMensajeLCD)
Rol: imprimir, por la pantalla LCD/Terminal, la cadena de carácteres recibida en el mailbox
*/
void A_PANTALLA_INFORMATIVA()
{
  uint8_t *pMsg; //puntero que apuntará a los datos
  char *aux;
  char temp[30];
  char delim[] = "/";
  
  while(1)
  {
	  so.waitMBox(mbMensajeLCD, (byte**) &pMsg); // Esperamos hasta recibir datos por mbox

  	hib.lcdClear();
  	strcpy(temp,pMsg);
  	aux = strtok(temp,delim); // Creamos tokens del string pasado
  	hib.lcdPrint(aux); // imprimimos el primer token
  	aux = strtok(NULL,delim); // guardamos el siguiente token (no hay más)
  	hib.lcdSetCursorSecondLine(); // posicionamos el cursor de LCD a la segunda fila
  	hib.lcdPrint(aux); // imprimimos el segundo token
      
    memset(temp, 0, sizeof temp);

  }
}

/*
Activación: flag fAlarma, con la máscara maskAlarm o maskSimulacro
Rol: emisión de sonido de buzzer a diferentes frecuencias dependiendo de la máscara de activación.
En el caso que sea una alarma se mantendrá sonando hasta que el estado no sea igual a PELIGRO
*/
void A_ALARMA_OXIGENO()
{
  // máscara que active tanto para alarma como simulacro
  unsigned char mask = (maskAlarm | maskSimulacro);
    // var to read flag value
  unsigned char flagValue;

  uint8_t tempEstado;
  unsigned long nextActivationTick;
  const unsigned char period = 12;

  while(1)
  {
      // tarea bloqueada hasta que los bits del flag fAlarma 
      // sean modificados a '1' respecto bits del mask
    so.waitFlag(fAlarma, mask);

    // leemos el valor del flag
    flagValue = so.readFlag(fAlarma); 
    
    so.clearFlag(fAlarma, mask);

      // miramos si el valor del flag es para alarma o simulacro de alarma
    switch(flagValue)
    {
        case maskAlarm:
          tempEstado = ESTADO_PELIGRO;
          nextActivationTick = so.getTick();

          while(tempEstado == ESTADO_PELIGRO){

            hib.buzzPlay(100,3500);
            hib.buzzPlay(100,4500);

            so.waitSem(sEstado);
            tempEstado = estado;
            so.signalSem(sEstado);

            nextActivationTick = nextActivationTick + period;
            so.delayUntilTick(nextActivationTick); 
          } 
          break;

        case maskSimulacro:
          hib.buzzPlay(100,2000);
          hib.buzzPlay(100,2500);
          hib.buzzPlay(100,2000);
          hib.buzzPlay(100,2500);
          break;
      }
  }
  
}

void setup() {
  hib.begin();
  hib.lcdClear();
  term.begin(115200);
  
  Serial.begin(115200);

  // Init can bus : baudrate = 500k, loopback mode, enable reception and transmission interrupts
  while (CAN.begin(CAN_500KBPS, MODE_NORMAL, true, false) != CAN_OK) {
    Serial.println("CAN BUS Shield init fail");
    Serial.println(" Init CAN BUS Shield again");
    delay(100);
  }


  // Set CAN interrupt handler address in the position of interrupt number 0
  // since the INT output signal of the CAN shield is connected to
  // the Mega 2560 pin num 2, which is associated to that interrupt number.
  attachInterrupt(0, ISR_RECEPCION_CAN, FALLING);

  so.begin();
}

void loop() {
  
  // Inicializacion semáforos
    // Mutex
  sAforo = so.defSem(1);
  sMutex = so.defSem(1);
  sEstado = so.defSem(1);
    // Débil
  sEnvia_Aforo = so.defSem(0); 
  
  // Inicializacion Flags
  fReciboCAN = so.defFlag();
  fSensorAforo = so.defFlag();
  fAlarma = so.defFlag();
  
  // Inicializacion MailBox
  mbContador_Aforo_1 = so.defMBox();
  mbContador_Aforo_2 = so.defMBox();
  mbMensajeLCD = so.defMBox();

  // Inicializacion de las tareas
  so.defTask(A_ACTUALIZAR_AFORO, PRIO_TASK_A_ACTUALIZAR_AFORO);
  so.defTask(A_MOSTRAR_AFORO_1, PRIO_TASK_A_MOSTRAR_AFORO_1);
  so.defTask(A_MOSTRAR_AFORO_2, PRIO_TASK_A_MOSTRAR_AFORO_2);
  so.defTask(A_ENVIA_AFORO, PRIO_TASK_A_ENVIA_AFORO);
  so.defTask(A_DISPATCHER, PRIO_TASK_A_DISPATCHER);
  so.defTask(A_PANTALLA_INFORMATIVA, PRIO_TASK_A_PANTALLA_INFORMATIVA);
  so.defTask(A_ALARMA_OXIGENO, PRIO_TASK_A_ALARMA_OXIGENO);
  
  // Configuramos la interrupación del keypad, a 100ms
  hib.keySetIntDriven(100, ISR_SENSOR_PUERTAS);
  
  // Inicializacion timer
  hib.setUpTimer5(TIMER_TICKS_FOR_50ms, TIMER_PSCALER_FOR_50ms, timer5Hook);

  // Entrar en modo concurrente
  so.enterMultiTaskingEnvironment();  
}
