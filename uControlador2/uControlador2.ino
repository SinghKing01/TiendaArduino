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
//#include "CONS.h"

// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 9;

HIB hib;
MCP_CAN CAN(SPI_CS_PIN);
SO so;
Terminal term ;

/********************************************
        DECLARACIÓN DE ESTRUCTURAS
*********************************************/

  // Definición del struct usado para almacenar los datos recibidos por el UART
typedef struct 
{
      uint8_t id;   // Código del mensaje
      char msg[20]; // Contenido del mensaje
      uint8_t dlc;  // Longitud del mensaje que se enviará
} msgUART;

/********************************************
          DEFINICIÓN DE CONSTANTES
*********************************************/

// CONSTANTES DEL SISTEMA
  // Definición de prioridades
#define PRIO_TASK_CONTROLADOR 1
#define PRIO_TASK_CALCULAR_OXIGENO_ACTUAL 2
#define PRIO_TASK_VENTILACION 3
#define PRIO_TASK_ENVIAR_DATOS 3
#define PRIO_TASK_LUZ_OXIGENO 4
#define PRIO_TASK_RECIBIR_DATOS 4
#define PRIO_TASK_UART_IN 5
#define PRIO_TASK_UART_OUT 5
#define PRIO_TASK_LCD_O2_ACTUAL 5

  // Centinelas UART
#define CENTINELA_ID ';'
#define CENTINELA_MSG '/'

  // Identificadores del tipo de comando de UART
#define ID_CONTROLADOR 0
#define UART_LCD 1
#define UART_BUZZER 2
#define UART_MAX_OXIGEN 3
#define UART_UMBRAL_RIESGO 4
#define UART_UMBRAL_PELIGRO 5
#define UART_MAX_POTENCIA 6
#define UART_CODIGO_ERROR 99

  // Tipos de estados del sistema t
#define ESTADO_UNKNOWN 255
#define ESTADO_EXCELENTE 0
#define ESTADO_RIESGO 1
#define ESTADO_PELIGRO 2

  // Umbrales de oxigeno que definen el estado del sistema.
  // Actuan como constantes aunque pueden ser cambiadas por UART a petición del usuario.  
volatile uint16_t MAX_OXIGEN = 65532;
volatile uint16_t UMBRAL_RIESGO = 43688;
volatile uint16_t UMBRAL_PELIGRO = 21844;
const char nomsEstats[3][10] = {
  "Excelente",
  "Riesgo",
  "Peligro"
};

  // Máxima potencia que puede alcanzar el ventilador 
  // Actua como constantes aunque pueden ser cambiadas por UART a petición del usuario.
volatile uint8_t MAX_POTENCIA = 5;

/********************************************
 DECLARACIÓN DE VARIABLES GLOBALES COMPARTIDAS
*********************************************/
volatile uint8_t aforo_Actual = 0;                    // Total de personas en el interior de la tienda
volatile uint16_t o2_Generado = 0;                    // Cantidad de oxígeno generado por el sistema de ventilación
volatile float o2_Actual = MAX_OXIGEN;                // Cantidad de oxígeno actual en la sala
volatile uint8_t estado = ESTADO_UNKNOWN;           // Estado del nivel de oxígeno en el establecimiento
volatile msgUART mensaje_UART;                        // Estructura que almacena los datos recibidos por el UART
volatile uint8_t rx_msg;                              // Buffer del bus CAN


/********************************************
 DECLARACIÓN DE PRIMITIVAS DE SINCRONIZACIÓN
*********************************************/

  // Flags y máscaras
Flag fReciboCAN;                                                      // ISR CAN --> A_Recibir_Datos
const unsigned char maskReceived = 0x01;

Flag fEnvioCAN;                                                       // P_Controlador | A_UART_In --> A_Enviar_Datos
const unsigned char maskControlador = 0x01;
const unsigned char maskUART = 0x02;

Flag fLuzOxigeno;                                                     // P_Controlador --> A_Luz_Oxigeno
const unsigned char maskLuzOxigenoOn = 0x01;
const unsigned char maskLuzOxigenoOff = 0x02;

  // Semáforos
    // Semáforos mutex
Sem sAforo_Actual;                                    // Proteje la variable global "aforo_Actual"
Sem sO2_Generado;                                     // Proteje la variable global "o2_Generado"
Sem sO2_Actual;                                       // Proteje la variable global "o2_Actual"
Sem sEstado;                                          // Proteje la variable global "estado"
Sem sMensaje_UART;                                    // Proteje la variable global "mensaje_UART"
Sem sMAX_POTENCIA;                                    // Proteje la variable global "MAX_POTENCIA"
Sem sUMBRALES;                                        // Proteje las variables global "MAX_OXIGEN", "UMBRAL_RIESGO" y "UMBRAL_PELIGRO"

    // Semáforo débil
Sem sUART_OUT;                                        // P_Controlador --> P_UART_Out

  // MailBox
MBox mbPotenciaVentilador;                            // P_Controlador --(uint8_t)--> A_Ventilacion
MBox mbLCD_O2_Actual;                                 // P_Controlador --(string)--> P_LCD_O2_Actual



/********************************************
        DECLARACIÓN DE HOOK'S Y ISR's
*********************************************/

// Hook FOR TICKISR
void timer5Hook ()
{ 
  so.updateTime(); // Llama al SO para actualizar el tiempo (tick) y administrar las tareas
}

// ISR del bus CAN, al recibir información, la guarda en el buffer y despierta a la tarea de recepción
void CAN_ISR(){
  char auxSREG;
    // Guarda el estado del registro AVR
    auxSREG = SREG;

    // Si se ha producido una interrupción de recepción
    if(CAN.rxInterrupt()){
      // Lectura del mensaje recibido 
      CAN.readRxMsg();
      // Almacenamiento del contenido en el buffer "rx_msg"
      CAN.getRxMsgData((byte*) &rx_msg);

      // Despierta la tarea "A_Recibir_Datos" para procesar el mensaje
      so.setFlag(fReciboCAN, maskReceived);
    }

    // Carga de nuevo el estado del registro AVR
    SREG = auxSREG;
}


/********************************************
		IMPLEMENTACIÓN DE LAS TAREAS
*********************************************/

/*
 Activación: periodica (TICK)
 Rol: calcular el estado del sistema (estado) según el nivel de oxígeno en el local (o2_Actual). Determinar que tareas de control o aviso activar según el valor de este.
*/
void P_Controlador(){

  const unsigned char period = 8;                               // Se activará cada 8 ticks
  unsigned long nextActivationTick;
  nextActivationTick = so.getTick();                            // Obtención del tiempo actual
  uint8_t estadoTemp;                                           // Usada para determinar si el sistema cambio de estado respecto a la interación anterior
  char mensajeLCD[33];
  uint8_t potencia = 0;                                         // Nivel de potencia que se subministrarà al Ventilador
  char tempO2_Actual[8];                                        // Necesario para convertir 'O2_Actual' (float) a una cadena de caracteres
  float porcentajeO2;
  while(1){  
      // Determinar el estado del sistema a partir de la cantidad de oxígeno que hay en la sala
    so.waitSem(sO2_Actual);
    so.waitSem(sUMBRALES);

    if(o2_Actual > UMBRAL_RIESGO){
      estadoTemp = ESTADO_EXCELENTE;
    } else if(o2_Actual > UMBRAL_PELIGRO){
      estadoTemp = ESTADO_RIESGO;
    } else{
      estadoTemp = ESTADO_PELIGRO;
    }

    porcentajeO2 = o2_Actual/MAX_OXIGEN * 100;

    so.signalSem(sUMBRALES);
    
      // Si el estado es diferente con respecto a la iteración anterior se decidirá si activar las tareas pertinentes
    so.waitSem(sEstado);
    
    if(estadoTemp != estado){
        // Actualizar el estado solo cuando cambia
      estado = estadoTemp;
      if(estado == ESTADO_PELIGRO ){ 
        so.setFlag(fLuzOxigeno, maskLuzOxigenoOn);
      } else {
        so.setFlag(fLuzOxigeno, maskLuzOxigenoOff);
      }

      so.setFlag(fEnvioCAN, maskControlador);
    }

    dtostrf(porcentajeO2,6, 2, tempO2_Actual); 
    sprintf(mensajeLCD, "Estado %s /Oxigeno %s%%", nomsEstats[estado], tempO2_Actual);
    
	// Para calcular la potencia a assignar, se normalizará el valor de oxígeno actual entre [0, 1]
	// y se multiplicara por la potencia máxima, obteniendo un valor de potencia entre [0, MAX_POTENCIA]
    so.waitSem(sMAX_POTENCIA);
    potencia = (1 - ((o2_Actual  - 0) / (MAX_OXIGEN - 0))) * MAX_POTENCIA;
    so.signalSem(sMAX_POTENCIA);

    so.signalSem(sEstado);
    so.signalSem(sO2_Actual);
    
    so.signalMBox(mbLCD_O2_Actual, (byte*) mensajeLCD);
    so.signalMBox(mbPotenciaVentilador, (byte*) &potencia);
    so.signalSem(sUART_OUT);
    
    nextActivationTick = nextActivationTick + period;
    so.delayUntilTick(nextActivationTick);
  }
}


/*  
 Activación: periódica (TICK)
 Rol: calcular el nivel de oxígeno (o2_Actual) según las variables del sistema (aforo_Actual, o2_generado)
*/
void P_Calcular_Oxigeno_Actual(){

  const unsigned char period = 4;                               // Se activarà cada 4 ticks     
  unsigned long nextActivationTick;
  nextActivationTick = so.getTick();                            // Obtención del tiempo actual

  const uint16_t meanCons = 450;                                // Consumo medio de oxígeno por persona

  while(1){
    so.waitSem(sO2_Actual);
    so.waitSem(sO2_Generado);
    so.waitSem(sAforo_Actual);
    
    // Cada nivel de potencia suministra oxígeno para 3 personas
    // Se puede generar oxígeno hasta 18 personas, luego se consumirá más del producido.
    o2_Actual = o2_Actual + 0.4 * o2_Generado - 0.3 * meanCons * aforo_Actual;
    
    so.waitSem(sUMBRALES);
    if(o2_Actual > MAX_OXIGEN){
      o2_Actual = MAX_OXIGEN;
    } else if(o2_Actual < 0){
      o2_Actual = 0;
    }
    so.signalSem(sUMBRALES);
    
    so.signalSem(sAforo_Actual);
    so.signalSem(sO2_Generado);
    so.signalSem(sO2_Actual);

    nextActivationTick = nextActivationTick + period;
    so.delayUntilTick(nextActivationTick);
  }
}


/*
 Activación: MailBox (P_Controlador -> mbPotenciaVentilador)
 Rol: generar oxígeno en la tienda. La potencia del ventilador será indicada por el controlador, y representada por los LED's [1,5].
*/
void P_Ventilacion(){
  uint8_t rxPotencia;                                           // Potencia del ventilador determinada por el Controlador
  uint8_t * rxPotenciaMessage;
  const uint16_t o2_producido = 1000;                           // Oxígeno producido por cada nivel de potencia
  
  while(1){
    so.waitMBox(mbPotenciaVentilador, (byte**) &rxPotenciaMessage);
    rxPotencia = *rxPotenciaMessage;
    
      // Encender la cantidad de LEDs que simulan el nivel de potencia
    for(uint8_t i = 0; i < rxPotencia; i++){
      hib.ledOn(i);
    }

    so.waitSem(sMAX_POTENCIA);
      // Apagar los LEDs restantes
    for(uint8_t i = rxPotencia;i < MAX_POTENCIA; i++){
      hib.ledOff(i);
    }
    so.signalSem(sMAX_POTENCIA);

    so.waitSem(sO2_Generado);
    o2_Generado = o2_producido*rxPotencia;
    so.signalSem(sO2_Generado);
  }
}


/*
 Activación: Flag (P_Controlador | A_UART_In -> fEnvioCAN)
 Rol: informar del estado de oxígeno de la tienda y enviar comandos al microcontrolador 1
*/
void A_Enviar_Datos(){

  const unsigned char mask = (maskControlador | maskUART);      // Máscara para determinar quien la ha activado
  unsigned char flagValue;                                      // Valor del flag que se ha activado

  uint32_t tx_id;                                               // ID del mensaje que se enviará
  uint8_t tx_dlc;                                               // Tamaño del mensaje que se enviará
  uint8_t tx_Controlador;                                       // Estado que se enviará al controlador
  char tx_UART[8];                                              // Mensaje del UART que se enviará
  
  while(1)
  {
      // Espera hasta que el Flag sea activado por el Controlador o por UART_In
    so.waitFlag(fEnvioCAN, mask);
    flagValue = so.readFlag(fEnvioCAN); 
    so.clearFlag(fEnvioCAN, mask);

      // Según quien haya activado la tarea se decidirá como actuar 
    switch(flagValue)
    {
        // Si el controlador ha activado la tarea, se enviará el estado actual de la tienda por el bus CAN
      case maskControlador: 
        so.waitSem(sEstado);
        tx_Controlador = estado;
        so.signalSem(sEstado);
        tx_id = ID_CONTROLADOR;
        tx_dlc = sizeof(uint8_t);

        if (CAN.checkPendingTransmission() != CAN_TXPENDING)
        {
          CAN.sendMsgBufNonBlocking(tx_id, CAN_STDID, tx_dlc, (INT8U *) &tx_Controlador);
        }
      break;
      
        // Si UART_In ha activado la tarea, se enviará el contenido de la variable msgUART por el bus CAN
      case maskUART:
        so.waitSem(sMensaje_UART);
        tx_id = mensaje_UART.id;
        tx_dlc = mensaje_UART.dlc;
        strcpy(tx_UART, mensaje_UART.msg);
        so.signalSem(sMensaje_UART);

        if (CAN.checkPendingTransmission() != CAN_TXPENDING)
        {
          CAN.sendMsgBufNonBlocking(tx_id, CAN_STDID, tx_dlc, (INT8U *) &tx_UART);
        }
      break;

    }
  }
}


/*
 Activación: ISR (CAN_ISR -> fReciboCAN)
 Rol: recibir el aforo actual del microcontrolador 1 y actualizar el valor de la variable local
*/
void A_Recibir_Datos(){  
  while(1){
      // La ISR será quien activará la tarea
    so.waitFlag(fReciboCAN, maskReceived);
    so.clearFlag(fReciboCAN, maskReceived);
    
      // Actualizar el aforo según el valor recibido por el bus CAN
    so.waitSem(sAforo_Actual);
    aforo_Actual = rx_msg;
    so.signalSem(sAforo_Actual);
    
  }
}


/*
 Activación: Flag (P_Controlador -> fLuzOxigeno)
 Rol: encender o apagar la luz de emergencia según lo indique el controlador
*/
void A_Luz_Oxigeno(){ 
  
  unsigned char mask = (maskLuzOxigenoOn | maskLuzOxigenoOff);
  unsigned char flagValue;

  while(1)
  {
          // Espera a que alguna tarea active el flag con las máscaras
          //maskAlarm or masknNotAlarm        
    so.waitFlag(fLuzOxigeno, mask);
    
          // Obtencion del valor de la mascara
    flagValue = so.readFlag(fLuzOxigeno); 
       
          // Limpiar el flag
    so.clearFlag(fLuzOxigeno, mask);
    
        // Decidir si activar o no el LED de emergencia
    switch(flagValue)
    {
        case maskLuzOxigenoOn:
          hib.ledOn(5);
          break;
          
        case maskLuzOxigenoOff:
          hib.ledOff(5);
          break;
          
        default:
          break;
    }
  }
}

/*
Activación: Periódica (TICK)
Rol: gestionar la entrada de comandos por UART.
*/
void P_UART_In(){

  char buff_id[2];                                              // Buffer que contendrá la ID del comando introducido
  uint8_t id;
  char buff_msg[8];                                             // Buffer que contendrá la información del comando introducido
  char c;                                                       // Carácter de consulta por Polling
  uint8_t indx = 0;                                             // Indice que recorrerá los buffers previos

  const unsigned char period = 3;                               // Se activara cada 3 ticks
  unsigned long nextActivationTick;
  nextActivationTick = so.getTick();                            // Obtiene el tiempo actual

  while(1){
      // Lectura no bloqueante del carácter introducido en el UART
    c = term.getChar(false);
    
      // Determina si es un carácter válido 
    if(c != term.NO_CHAR_RX_UART){
      // Hasta que se lea el CENTINELA_ID (';') obtener el identificador del comando introducido
      while(c != CENTINELA_ID && indx < sizeof(buff_id)/sizeof(buff_id[0])){
        if(c != term.NO_CHAR_RX_UART){
          buff_id[indx] = c;
          indx = indx + 1;
        }
        c = term.getChar(false);
        nextActivationTick = nextActivationTick + period;
        so.delayUntilTick(nextActivationTick);
      }
  
        // Restear el índice
      indx = 0;
      c = term.getChar(false);
        // Haste que se lea el CENTINELA_MSG ('/') obtener el mensaje del UART
      while(c != CENTINELA_MSG && indx < sizeof(buff_msg)/sizeof(buff_msg[0])){
        if(c != term.NO_CHAR_RX_UART){
          buff_msg[indx] = c;
          indx = indx + 1;
        }
        c = term.getChar(false);
        nextActivationTick = nextActivationTick + period;
        so.delayUntilTick(nextActivationTick);
      }
  
        // Una vez se tenga el identificador y la información referente a este se procederá a determinar que desea ejecutar el usuario
      
      id = atoi(buff_id);
      switch(id){
       // El usuario desea mostrar en la pantalla un mensaje 
        case UART_LCD:
          so.waitSem(sMensaje_UART);

          for(uint8_t i = 0; i<indx; i++){
            mensaje_UART.msg[i] = buff_msg[i];
          }
          
          mensaje_UART.id = id;
          mensaje_UART.dlc = indx;
          
          so.signalSem(sMensaje_UART);
          so.setFlag(fEnvioCAN, maskUART);
        break;
  
        // El usuario desea realizar un simulacro de la alarma
        case UART_BUZZER:
          so.waitSem(sMensaje_UART);
          mensaje_UART.id = id;
          mensaje_UART.dlc = 0;
          so.signalSem(sMensaje_UART);
          so.setFlag(fEnvioCAN, maskUART);
        break;
  
        // El usuario desea cambiar el valor del umbral máximo de oxígeno (MAX_OXIGEN)
        case UART_MAX_OXIGEN:
          so.waitSem(sUMBRALES);
          sscanf(buff_msg, "%u", &MAX_OXIGEN);
          so.signalSem(sUMBRALES);
        break;
  
        // El usuario desea cambiar el valor del umbral de riesgo (UMBRAL_RIESGO)
        case UART_UMBRAL_RIESGO:
          so.waitSem(sUMBRALES);
          sscanf(buff_msg, "%u", &UMBRAL_RIESGO);
          so.signalSem(sUMBRALES);
        break;
  
        // El usuario desea cambiar el valor del umbral de peligro (UMBRAL_PELIGRO)
        case UART_UMBRAL_PELIGRO:
          so.waitSem(sUMBRALES);
          sscanf(buff_msg, "%u", &UMBRAL_PELIGRO);
          so.signalSem(sUMBRALES);
        break;
  
        // El usuario desea cambiar el valor de la potencia máxima del ventilador (MAX_POTENCIA)
        case UART_MAX_POTENCIA:
          so.waitSem(sMAX_POTENCIA);
          sscanf(buff_msg, "%u", &MAX_POTENCIA);
          so.signalSem(sMAX_POTENCIA);
        break;

        // En el caso que el usuario no haya introducido ningun código válido se indicará en la estructura 'mensaje_UART'
        default:
          so.waitSem(sMensaje_UART);
          mensaje_UART.id = UART_CODIGO_ERROR;
          mensaje_UART.dlc = 0;
          so.signalSem(sMensaje_UART);
      }   

      // Se limpian ambos buffers y se resetea el índice para la siguiente ejecución
      indx = 0;
      memset(buff_id, 0, sizeof buff_id);
      memset(buff_msg, 0, sizeof buff_msg);
    }
    
    nextActivationTick = nextActivationTick + period;
    so.delayUntilTick(nextActivationTick);
  }
}
 
  
/*
 Activación: semáforo (sUART_OUT)
 Rol: mostrar en el UART el valor de diversas variables globales del sistema
*/
void P_UART_Out(){

  char buff[120];
  char tempO2_Actual[10];
  while(1){
    so.waitSem(sUART_OUT);

    so.waitSem(sO2_Actual);
    so.waitSem(sEstado);
    so.waitSem(sO2_Generado);
    so.waitSem(sAforo_Actual);

    // Convertir float (o2_Actual) a String
    dtostrf(o2_Actual,10, 2, tempO2_Actual);
    sprintf(buff, "INFORMACIÓN DEL SISTEMA:\n\t- El estado es: %s\n\t- El aforo es: %u\n\t- O2 generado: %u\n\t- O2 Actual: %s", nomsEstats[estado], aforo_Actual, o2_Generado, tempO2_Actual);
      // Mostrar por el UART la información del sistema
    
    term.println(buff);

    so.signalSem(sAforo_Actual);
    so.signalSem(sO2_Generado);
    so.signalSem(sEstado);
    so.signalSem(sO2_Actual);

    term.waitTxComplete();
  }
}


/*
 Activación: MailBox (P_Controlador -> mbLCD_O2_Actual)
 Rol: mostrar a los clientes el nivel de oxígeno del establecimiento
*/
void P_LCD_O2_Actual(){

  // Parsear el string recibido en formato: "Frase_1/Frase_2"
  char rxLCD_Info[33];
  char * rxLCD_InfoMessage;
  char delim[] = "/";
  char * rxLCD_toPrint;
  

  while(1){
    so.waitMBox(mbLCD_O2_Actual, (byte**) &rxLCD_InfoMessage);
    hib.lcdClear();
    
    // Escritura Frase_1 en la primera línea del LCD
    strcpy(rxLCD_Info, rxLCD_InfoMessage);
    rxLCD_toPrint = strtok(rxLCD_Info, delim);
    hib.lcdPrint(rxLCD_toPrint);

    // Escritura Frase_2 en la segunda línea del LCD
    rxLCD_toPrint = strtok(NULL, delim);
    hib.lcdSetCursorSecondLine();
    hib.lcdPrint(rxLCD_toPrint);
  }
}


// SETUP
void setup() {  

  Serial.begin(115200); // SPEED
  term.begin(115200);   // Inicialización del Terminal
  term.clear();

    // Init can bus : baudrate = 500k, loopback mode, enable reception and transmission interrupts
  while (CAN.begin(CAN_500KBPS, MODE_NORMAL, true, false) != CAN_OK) {
    Serial.println("CAN BUS Shield init fail");
    Serial.println(" Init CAN BUS Shield again");
    delay(100);
  }

  hib.begin();        // Inicialización del HIB
  hib.lcdClear();
  
  attachInterrupt(0, CAN_ISR, FALLING);     // Añadir interrupción por recibo CAN
  so.begin();                               // Inicialización del sistema operativo
}


void loop() {
  // Inicializacion Flags
  fEnvioCAN = so.defFlag();
  fLuzOxigeno = so.defFlag();
  fReciboCAN = so.defFlag();
  
  // Inicializacion semáforos
    // Mutex
  sAforo_Actual = so.defSem(1);
  sO2_Generado = so.defSem(1);
  sO2_Actual = so.defSem(1);
  sEstado = so.defSem(1);
  sMensaje_UART = so.defSem(1);
  sMAX_POTENCIA = so.defSem(1);
  sUMBRALES = so.defSem(1);
    // Débil
  sUART_OUT = so.defSem(0);

  // Inicializacion MailBox
  mbPotenciaVentilador = so.defMBox();
  mbLCD_O2_Actual = so.defMBox();

  // Inicializacion de las tareas
  so.defTask(P_Controlador, PRIO_TASK_CONTROLADOR);
  so.defTask(P_Calcular_Oxigeno_Actual, PRIO_TASK_CALCULAR_OXIGENO_ACTUAL);
  so.defTask(P_Ventilacion, PRIO_TASK_VENTILACION);
  so.defTask(A_Enviar_Datos , PRIO_TASK_ENVIAR_DATOS);
  so.defTask(A_Luz_Oxigeno, PRIO_TASK_LUZ_OXIGENO);
  so.defTask(A_Recibir_Datos, PRIO_TASK_RECIBIR_DATOS);
  so.defTask(P_UART_In, PRIO_TASK_UART_IN);
  so.defTask(P_UART_Out, PRIO_TASK_UART_OUT);
  so.defTask(P_LCD_O2_Actual, PRIO_TASK_LCD_O2_ACTUAL);

  // Inicializacion timer
  hib.setUpTimer5(TIMER_TICKS_FOR_250ms, TIMER_PSCALER_FOR_250ms, timer5Hook);

  // Entrar en modo concurrente
  so.enterMultiTaskingEnvironment();
}
