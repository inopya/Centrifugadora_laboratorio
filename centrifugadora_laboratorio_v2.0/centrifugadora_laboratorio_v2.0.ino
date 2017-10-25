/*
 #       _\|/_   A ver..., ¿que tenemos por aqui?
 #       (O-O)
 # ---oOO-(_)-OOo---------------------------------

 ####################################################
 # ************************************************ #
 # *    Centrifugadora para laboratorio casero    * #
 # *                                              * #
 # *         Autor: Eulogio López Cayuela         * #
 # *       Versión 2.0     Fecha: 26/09/2017      * #
 # *                                              * #
 # ************************************************ #
 ####################################################



  NOTAS DE LA VERSION:
    Control de centrifugadora de laboratorio casero mediante ARDUINO UNO
    para actuar sobre el sistema se dispone de un potenciometro y un pulsador
    El potenciometro permite introducir valores en los campos corespondientes
    y el pulsador aceptar dichos valores
    Esta vesion tambien dispone de un tamper de seguridad que desconecta el servo
    en caso de que se abra la tapa de proceccion, avitandose asi posibles accidentes
    Permite asi mismo la apertura controla de dicha tapa durante la ejecucion de un trabajo
    haciendo una paus en el mismo uqe podemos reanudar despues de cerrar de nuevo


    Para los valores de tiempo dispomenos del rango entre 0 y 59 para cada uno de los campos --> HH:mm:ss
    que se modificaran actuando sobre el potenciometro.
    El valor de un campo queda aceptado al activar el pulsador, 'saltando' automaticamente al siguente campo.
    --- Podemos corregir un valor ya aceptado de la siguiente manera:
            En la posicion minima del potenciometro el valor que obtenemos es -1, que el programa  
            nos representa como'--'. Si pulsamos en ese estado el programa interpetra que queremos 
            corregir el valor anterior y nos mueve a dicho campo

    Otras explicaciones sobre el funcionamiento de funciones, en el codigo de dichas funciones


  #define interrupts() sei()
  #define noInterrupts() cli()
  
 */


/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//        SECCION PARA IMPORTAR BIBLIOTECAS
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/


//------------------------------------------------------
//Importacion de las biblioteca necesarias
//------------------------------------------------------
#include <Servo.h>                // utilizamos la libreria estandar para servos
#include <Wire.h>                 // Biblioteca para comunicaciones I2C
#include <LiquidCrystal_I2C.h>    // Biblioteca oara el control del LCD

#include <avr/interrupt.h>
#include <avr/io.h>

                                  // creamos un par de funciones para Humanizamos un poco el uso del timer2, 
                                  // Para el control de tiempos, las ISR relacionadas con el timer1 quedan descartadas
                                  // ya que entran en conflicto con el uso que de ellas hace la libreria servo.



/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//        SECCION DE DECLARACION DE CONSTANTES  Y  VARIABLES GLOBALES
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/
//------------------------------------------------------
//algunas definiciones personales para mi comodidad al escribir codigo
//------------------------------------------------------
#define AND &&
#define OR ||
#define NOT !
#define ANDbit &
#define ORbit |
#define XORbit ^
#define NOTbit ~

//------------------------------------------------------
//Otras definiciones para pines y variables
//------------------------------------------------------
#define LCD_AZUL_ADDR    0x27  // Direccion I2C de nuestro LCD color azul  <-- Este es el que vamos a usar
#define LCD_VERDE_ADDR   0x3F  // Direccion I2C de nuestro LCD color verde


//------------------------------------------------------
// Creamos las instancia de los objetos:
//------------------------------------------------------

//Creamos el objeto 'lcd' como una instancia del tipo "LiquidCrystal_I2C"
//                             addr, en,rw,rs,d4,d5,d6,d7,bl, blpol
LiquidCrystal_I2C lcd(LCD_AZUL_ADDR,  2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); 


// ****  Definiciones relacionadas con el LED RGB  ****
#define PIN_sensor_hall  2          // Patilla para el sensor hall

// ****  Definiciones relacionadas con el pulsador de seleccion  ****
#define PIN_PULSADOR_SELECCION  6   // Patilla para el sensor tactil

// ****  Definiciones relacionadas con el motor  ****
#define POTENCIOMETRO  A0           // potenciometro conectado en analogica 0
                                    // se usara para navegar por los menus y para regular la velocidad           

// ****  Definiciones relacionadas con el tamper de seguridad  ****
#define TAMPER  7                   // no se usa de momento. Ya que el tamper cortara la alimentacion
                                    // de formma fisica sin mediacion del microcontrolador


// ****  Definiciones relacionadas con el altavoz  ****
//activarAlarma(PIN_ZUMBADOR, FRECUENCIA, TIEMPO_SONIDO, TIEMPO_SILENCIO, CICLOS);

//     EJEMPLO --->  activarAlarma(PIN_ZUMBADOR, 1800, 200, 150, 1);
#define PIN_ZUMBADOR        3       //patilla para el altavocillo
#define FRECUENCIA       1800       //frecuencia (Hz)del tono. Entre 2000 y 2100, muy cabrones y buenos para ALARMA
#define TIEMPO_SONIDO     200       //durarion del tono (milisegundos)
#define TIEMPO_SILENCIO   150       //durarion del silencio (milisegundos)
//      CICLOS                      //numero de veces que se repite byte(1-255). con valor CERO no sonaria
//version humana ---> pitidos(numero_pitidos)

// ****  Definiciones relacionadas con el motor  ****
#define PIN_SERVO_CONTINUO   5        // Patilla para el servo de la centrifugadora

// ****  Declaraciones de los objetos tipo servo para todos los motores  ****
Servo SERVO_CONTINUO;                 // Definimos el objeto correspondiente al motor de la centrifugadora

unsigned long CONTADOR = 0;           // control de pulsos para el sensor HALL


boolean FLAG_punto_segundero = true;  // controla si deben o no mostrarse los puntos de separacion de los segundos
                                      // para generar el tipico parpadeo de los relojes que acompaña a los segundos

boolean FLAG_fin_contador = false;    // control de terminacion de proceso de centrifugado

boolean FLAG_update_reloj;            // indica si es el momento de refrescar el lcd para mostrar la hora
                                      // evitamos asi parpadeos innecesarios. La activa la ISR correspondiente 
                                      // al control del timer2 apra el control de 'medios segundos'

unsigned char medioSegundo = 0;       // control de medios segundos. Por una parte controla el parpadeo del segundero
                                      // y por otra, cada dos ciclos realiza el cambio de segundo y actualiza el reloj 

int horas = -1;                       // contiene el valor horario del temporizador (iniciados fuera de rango)
int minutos = -1;                     // contiene el valor de los minutos del temporizador
int segundos = -1;                    // contiene el valor de los segundos del temporizador

#define INTERVALO_TIMER2  497         // En teoria 500, pero dado que contamos ciclos del reloj interno del procesado
                                      // estos no son precisamente misisegundos reales :)
                                      // asi que este es un valor empirico que se ajusta bastante bien
                                      // consiguiendo un error aproximado de 2 segundos de adelando por 
                                      // cada hora temporizada

boolean pulsacion = false;            // Para el control del unico pulsador del que dispone el dispositivo
                                      // y que sirve tanto para aceptar en los menus como hacer una pausa
                                      // durante el proceso de centrifugado, lo que permite revisar el proceso
                                      // y añadir o quitar elementos

unsigned long TIEMPO_ACTUAL = 0;      // Utilizada en el control de tiempos par determinar las r.p.m.
unsigned long TIEMPO_ANTERIOR = 0;    // Utilizada en el control de tiempos par determinar las r.p.m.
unsigned int RPM = 0;                 // Variable usada para almacener el valor de revoluciones por minuto
                                      // usamos tipo uint para velocidad en los calculo, limitados a 65.500 rpm

unsigned long refresco_LCD = 0;       // control de tiempos que indica el proximo instante
                                      // para el refresco del numero de r.p.m. en el display.
                                      // Dado que el LCD se refresca cada medio segundo para generar el parpadeo
                                      // del segundero, es molesto e ilegible a veces el numero de r.p.m. 
                                      // ya que este no es 100% estable. Controlamos 

#define INTERVALO_REFRESCO_RPM 350000 // intervalo en microSegundos para refrescar el dato RPM

int RPM_auto = 0;           // para almaceanr la velocidad solicitada en modo auto

boolean FLAG_modo_auto = true;        //bandera para el control de la seleccion del tipo de funcionamiento
boolean FLAG_run_mode = true;

boolean FLAG_menu_principal = true;


/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//        SECCION OPERATIVA DEL PROGRAMA
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/


//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm 
//***************************************************************************************************
// FUNCION DE CONFIGURACION
//***************************************************************************************************
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm 

void setup() //inicializacion del sistema
{
  //Serial.begin(9600);         // inicializar el puerto serial para 'DEBUG'
  lcd.begin (16,2);             // Inicializar lcd 
  
  /* ------ MostarVersion ------------------------------------------ */
  lcd.clear();                  // Reset del display 
  lcd.setBacklight(true);       // Activamos la retroiluminacion
  lcd.setCursor(0, 0);          // posicionar cursor al inicio de la primera linea (0)
  //       ("0123456789ABCDEF") // guia apara escribir sin pasarnos del espacio disponible  :)
  lcd.print("Centrifugator 2K");
  lcd.setCursor(0, 1);          // posicionar cursor al inicio de la segunda linea (1)
  lcd.print(" v 2.1  -final- ");
  //delay(3500);                  // Pausa apra poder leer el mensaje
  //lcd.clear();                  // Reset del display 
  //pitidos(2);
  /* ------ FIN MostarVersion -------------------------------------- */  



  // *****  sensor magnetico para en cuenta revoluciones  *****
  pinMode(PIN_sensor_hall, INPUT);            // definimos el pin como una entrada
  
  
  // *****  motor de la centrifugadora  *****
  pinMode(PIN_SERVO_CONTINUO, OUTPUT);        // definimos el pin como una salida
  digitalWrite(PIN_SERVO_CONTINUO, LOW);      // ponemos dicha salida a CERO
  
  SERVO_CONTINUO.attach(PIN_SERVO_CONTINUO);  // indicamos al objeto servo en que patilal esta conectado
  
  SERVO_CONTINUO.writeMicroseconds(1000);     // escribimos un valor de 'duty cycle' que corresponde a PARADO
  delay(4000);  

  attachInterrupt(0, cuentaVueltas, RISING);  // tipos admitidos de interrupcion: LOW, CHANGE, FALLING, RISING 
                                              // asignacion de la funcion para atender la interrupcion INT0
                                              
  delay(250);                                 //Pausa inicial para dar tiempo a estabilizar el sistema
  
  pitidos(2);                                 //dos pitidos para avisar de que se ha inicializado todo
  lcd.clear();                                // Reset del display 
}



//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm 
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm 
//***************************************************************************************************
//  BUCLE PRINCIPAL DEL PROGRAMA   (SISTEMA VEGETATIVO)
//***************************************************************************************************
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm 
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm 

void loop()
{
  proceso_principal(); 
}



//========================================================
//  FUNCION PARA EL CONTROL DE MENU Y SELECCIONES
//========================================================

void proceso_principal()
{
  while(true){
    AtenderMenuPrincipal();
    FLAG_run_mode = true;
    FLAG_fin_contador = false;
    sei();
    
    
    if (FLAG_modo_auto == true AND FLAG_run_mode == true){
      Funcionamiento_AUTO();
    }
  
    if (FLAG_modo_auto == false  AND FLAG_run_mode == true){ 
      Funcionamiento_MANUAL();
    }
  }
}



/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//        SECCION PARA EL MODO DE OPERACION MANUAL
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

//========================================================
//  FUNCION PARA MODO MANUAL
//========================================================

void Funcionamiento_MANUAL()
{
  /*puesta acero de las variables apra el contador ascendente*/
  horas = 0;                      // contiene el valor horario del reloj
  minutos = 0;                    // contiene el valor de los minutos del reloj
  segundos = 0;                   // contiene el valor de los segundos del reloj


  lcd.clear();                    // reset del display
  lcd.setCursor(0, 0);
  //       ("0123456789ABCDEF"); 
  lcd.print("INICIANDO MANUAL");
  
  /* seleccion de velocidad mediante potenciometro */
  int potenciometro = analogRead(POTENCIOMETRO);   // leer potenciometro para ver si hay cambio de velocidad  

  /* 'acomodar' el valor del potenciometros al rango del duty cycle */
  int dutyCycle = map(potenciometro,0,1023,1000,2300);  
  
  /* bucle para acelerar suavemente hasta la 'posicion inicial del potenciometro' */
  for(int i=1000;i<dutyCycle;i+=10){
    SERVO_CONTINUO.writeMicroseconds(i);      // refrescamos el dato 'dutyCycle' al servo 
    delay(20);
  }

  lcd.clear();                    // reset del display
  start_Timer2();                 // activamos la interrupcion del timer02
  
  while (FLAG_run_mode == true){
   
    /* seleccion de velocidad mediante potenciometro */
    potenciometro = analogRead(POTENCIOMETRO);   // leer potenciometro para ver si hay cambio de velocidad  

    /* 'acomodar' el valor del potenciometros al rango del duty cycle */
    dutyCycle = map(potenciometro,0,1023,1000,2300);  

    /* lectura del pulsador */                                                          
    byte pulsacion = leer_Pulsador_Tactil();//digitalRead(PIN_PULSADOR_SELECCION); 
 
    /* enviar velocidad al servo */
    if(FLAG_fin_contador == false AND pulsacion == 0){
      SERVO_CONTINUO.writeMicroseconds(dutyCycle);      // refrescamos el dato 'dutyCycle' al servo  
    }
    else{
      SERVO_CONTINUO.writeMicroseconds(1000);           // indicar al servo que debe parar al terminarse el tiempo
//      stop_Timer2();                                    // paramos tambien el proceso de ISR de timer2
//      FLAG_update_reloj = false;                        // detener el refresco del reloj
    }
      
      
    /* Control de muestra y refresco del temporizador */
    if(FLAG_update_reloj == true){
      FLAG_update_reloj = false;
      refrescar_Reloj();
    }

    /* mostrar velocidad en el LCD */
    mostrar_Velocidad();
    
    /* condicion para una pausa en tiempo de ejecucion */ 
    if(FLAG_fin_contador == false AND pulsacion == 1){      //condicion de pausa en ejecucion
      activar_Pausa();                                      //llamada a rutina de pausa

      /*si volvemos aqui es por que se desactiva la pausa */
      /*en otros casos desde la rutina de pausa volvemos al menu principal por cancelacion del proceso */
      FLAG_update_reloj = true;          // rectivar el refresco del reloj
      start_Timer2();                    // Reanudamos la cuenta del tiempo activando la interrupcion del timer2
    }
  }  
  proceso_principal();
}




/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//    MODO DE OPERACION AUTOMATICO
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

//========================================================
//  FUNCION PARA MODO AUTOMATICO
//========================================================

void Funcionamiento_AUTO()
{
  /*
   * Mientras no se alcance el final de la cuenta atras
   * mantenemos el motor activo, vamos refrescando en pantalla el temporizador 
   * y el dato 'instantaneo' de las r.p.m.
   * Se nos permite cambair la velocidad de giro mientras esta funcionando.
   */

  programar_Modo_AUTO();
                                                  
  int dutyCycle = autoAjuste_velociad_v2();
  pitidos(1);

  delay(500);
  lcd.clear();
  
  start_Timer2();       // activar la interrupcion del timer2  

  while (FLAG_run_mode == true){   

    /* leer el pulsador de estado */
    byte pulsacion = leer_Pulsador_Tactil();//digitalRead(PIN_PULSADOR_SELECCION);      
    
    if(FLAG_fin_contador == false){
      
      /* enviar velocidad al servo */
      if(FLAG_fin_contador == false AND pulsacion == 0){
        SERVO_CONTINUO.writeMicroseconds(dutyCycle);      // refrescamos el dato 'dutyCycle' al servo  
      }
      else{
        SERVO_CONTINUO.writeMicroseconds(1000);           // indicar al servo que debe parar al terminarse el tiempo
//        stop_Timer2();                                    // paramos tambien el proceso de ISR de timer2
//        FLAG_update_reloj = false;                        // detener el refresco del reloj
      }

      /* Control de muestra y refresco de la cuenta atras */
      if(FLAG_update_reloj == true){
        FLAG_update_reloj = false;
        refrescar_Reloj();
      }
  
      /* mostrar velocidad en el LCD */
      mostrar_Velocidad();

      /* condicion para una pausa en tiempo de ejecucion */ 
      if(FLAG_fin_contador == false AND pulsacion == 1){      //condicion de pausa en ejecucion
        activar_Pausa();                                      //llamada a rutina de pausa
  
        /*si volvemos aqui es por que se desactiva la pausa */
        /*en otros casos desde la rutina de pausa volvemos al menu principal por cancelacion del proceso */
        FLAG_update_reloj = true;          // rectivar el refresco del reloj
        start_Timer2();                    // Reanudamos la cuenta del tiempo activando la interrupcion del timer2
      }

      /* condicion para terminar el ciclo de trabajo cuando acaba el tiempo */
      if(FLAG_fin_contador == true){
        SERVO_CONTINUO.writeMicroseconds(1000);
        pitidos(5);                                       // activamos una alarma para avisar
                                                          // al usuario de que ha terminado
        lcd.clear();
        lcd.setCursor(0, 0);
        //       ("0123456789ABCDEF"); 
        lcd.print("PROCESO ACABADO ");                // al tiempo que lo mostramos en  pantalla mediante un mensaje
        lcd.setCursor(0, 1);
        lcd.print("es seguro abrir ");
        playMusic();                                  // fanfarria de proceso finalizado. Reirse es bueno :)
        delay(2500);
        pulsacion = leer_Pulsador_Tactil();
        while (pulsacion == 0){ 
          pulsacion = leer_Pulsador_Tactil();
        }
        FLAG_run_mode = false;   //condicion para salir al menu principal
        delay(350);
        proceso_principal();
      }
    } 
  }
  proceso_principal();
}



/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//     FUNCIONES AUXILIARES
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

//========================================================
//  RELOJ EN EL LCD
//========================================================

void refrescar_Reloj()
{
  lcd.setCursor(0, 1);
  lcd.print("Time:  ");
  
  if(horas<10){
    lcd.print("0");   // para que todas las cantidades aparezcan siempre con dos cifras 
                      // y no tengamos un 'baile' de numeros
  }
  lcd.print(horas);
  lcd.print(":");
  if(minutos<10){
    lcd.print("0");
  }    
  lcd.print(minutos);
  
  //--- control de parpadeo de segundero
  if(FLAG_punto_segundero==true){
    lcd.print(":");
  }
  else {
    lcd.print(" "); 
  }
  //--- FIN control de parpadeo de segundero
  
  if(segundos<10){
    lcd.print("0");
  }
  lcd.print(segundos);lcd.print(" ");
}


//========================================================
//  VELOCIDAD EN EL LCD
//========================================================

void mostrar_Velocidad()
{
  /* mostrar datos de velociad en el LCD */
  lcd.setCursor(0, 0);
  lcd.print("rpm: ");
  lcd.setCursor(7, 0);
  unsigned long ahora_mismo = micros();
  if ((ahora_mismo - TIEMPO_ANTERIOR)> 650000){       // si ahce mas de 650 ms que tuvimos noticias del tacometro
    RPM = 0;                                          // por ultima vez, entendemos que estamos parados.
  }
  if(refresco_LCD <  ahora_mismo){                    // si ha pasado un cierto tiempo (INTERVALO_REFRESCO_RPM)
    lcd.print(RPM);lcd.print("    ");                 // procedemos a mostrar el nuevo valor de RPM
    refresco_LCD = micros() + INTERVALO_REFRESCO_RPM; // evitandonos asi molestos parpadeos den las cifras      
  }
}



//========================================================
//  GESTION DE LAS PAUSAS 
//========================================================

void activar_Pausa()
{
  pitidos(2); // 3 pitidos para indicar la pusa 
  lcd.clear();
  lcd.setCursor(0, 0);
  //       ("0123456789ABCDEF")
  lcd.print(" *** PAUSA ***  ");
  lcd.setCursor(0, 1);
  lcd.print("OK --> continuar");
  
  byte pulsacion = 0;
  while(pulsacion == 0){  //false
    pulsacion = leer_Pulsador_Tactil();//digitalRead(PIN_PULSADOR_SELECCION);  //esperamos la activacion del pulsdor para continuar
  }
  if (pulsacion == 2){
    FLAG_run_mode = false;   //condicion para abortar y salir al menu principal
    FLAG_fin_contador = true; //anulamos el proceso de contador al abortar
    proceso_principal();
  }
  pitidos(1); // 2 pitidos para indicar que continua
  delay(350); //para evitar rebotes
  /* salir de la pausa */ 
  lcd.clear();
}



/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//    TECLADO
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

//========================================================
//  FUNCION PARA CONTROLAR EL ESTADO DEL PULSADOR
//========================================================

byte leer_Pulsador_Tactil()
/*
 * Funcion para atender la pulsacion de la tecla OK/seleccion
 */
{
  boolean FLAG_pulsacion = false;
  unsigned long momento_pulsar_boton = 0;
  unsigned long momento_soltar_boton = 0;
  byte tipo_pulsacion = 0;   // no ha habido pulsacion 
    
  boolean pulsacion = digitalRead(PIN_PULSADOR_SELECCION);  // leemos el estado del pulsador tactil
  if (pulsacion==true){
    stop_Timer2();                                    // paramos tambien el proceso de ISR de timer2
    FLAG_update_reloj = false;                        // detener el refresco del reloj
    
    pitidos(1);  //1 pitido
    momento_pulsar_boton = millis();                        //'anotamos' el momento de la pulsacion
    delay(25);                                              //pausa para evitar rebotes
    while(pulsacion==true){
      pulsacion = digitalRead(PIN_PULSADOR_SELECCION);      // leemos el estado del pulsador tactil
    }
    momento_soltar_boton = millis();
    FLAG_pulsacion = true; 
  }
  
  unsigned long duracion_pulsacion = momento_soltar_boton - momento_pulsar_boton;
  //determinar la duracion de pulsacion para saber si es normal o larga

  if (FLAG_pulsacion==true){  //pulsacion corta
    if (duracion_pulsacion >= 450){  
      tipo_pulsacion = 2; //pulsacion larga
    }
    else{
      tipo_pulsacion = 1; //pulsacion normal
    }
  }
  return tipo_pulsacion;
}



/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//        MENU PRINCIPAL
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

//========================================================
//  SELECCIN DE MODO DE OPERACION
//========================================================

void AtenderMenuPrincipal()
  /*
      Menu principal del sistema en el que podemos elegir el modo de funcionamiento:
        MANUAL: Modo simple en el que comtrolamos la velocidad mediante el potenciometro
                y el tiempo es un contador ascendente que no para en ningun momento
                (Quizas estaria bien ponerle un limite de horas por si hay algun olvidadizo)
                Este modo permitira hacer una pausa en el proceso para revisar el trabajo
        AUTO:   Modo en el que se debe programar el tiempo de funcionamiento y la velocidad deseada
                Una vez iniciado el proceso solo se podran hacer pausas o cancelarlo,
                pero no actuar sobre la velocidad (el potenciometro estara desatendido)
                La velocidad se autoregulara para aproximarse lo mas posible a la pedida por el usuario
  */
{

  lcd.clear();                  // Reset del display 
  lcd.setBacklight(true);       // Activamos la retroiluminacion
  lcd.setCursor(0, 0);          // posicionar cursor al inicio de la primera linea (0)
  //       ("0123456789ABCDEF") // guia apara escribir sin pasarnos del espacio disponible  :)
  lcd.print("Selecciona  modo");
  lcd.setCursor(0, 1);          // posicionar cursor al inicio de la primera linea (1)
  lcd.print(" AUTO     MANUAL");


  byte posicion_cursor = 0;
  byte pulsacion = 0;
  unsigned long momento_pulsar_boton = 0;
  unsigned long momento_soltar_boton = 0; 
  boolean FLAG_menu_principal_activo = true;                  // bandera para control de seleccion de modo de operacion

  
  while(FLAG_menu_principal_activo == true){
    lcd.setCursor(posicion_cursor, 1);                        //posicion para escribir la marca de seleccion '*'
    lcd.print("*");

    pulsacion = leer_Pulsador_Tactil();                   // leemos el estado del pulsador tactil

    
    //si se produce una pulscion corta, cambiamos la posicion del cursor de seleccion
    if (pulsacion == 1){                    // pulsacion corta
      lcd.setCursor(posicion_cursor, 1);    // posicion para borrar la marca de seleccion '*'
      lcd.print(" ");
      if (posicion_cursor == 0){
        posicion_cursor = 9;                // nueva posicion par la amrca de seleccion
      }
      else{
        posicion_cursor = 0;                // nueva posicion par la amrca de seleccion
      }
    }
    
    //si se produce una pulscion larga, aceptar y salir
    if (pulsacion == 2){  //pulsacion larga
      pitidos(1);  //1+1 pitidos
      FLAG_menu_principal_activo = false; //cambio de menu
    }
  }

  //mostrar segundo menu para confirmar el tipo de seleccion
  lcd.clear();                  // Reset del display 
  lcd.setBacklight(true);       // Activamos la retroiluminacion
  lcd.setCursor(0, 0);          // posicionar cursor al inicio de la primera linea (0)
  //       ("0123456789ABCDEF") // guia apara escribir sin pasarnos del espacio disponible  :)
  lcd.print("MODO ");
  
  if (posicion_cursor == 0){
    FLAG_modo_auto = true;
    lcd.print(" AUTO");
  }
  else{
    FLAG_modo_auto = false;
    lcd.print(" MANUAL");
  } 
  lcd.setCursor(0, 1);          // posicionar cursor al inicio de la primera linea (1)  
  //       ("0123456789ABCDEF") // guia apara escribir sin pasarnos del espacio disponible  :)

  lcd.print("  --> click OK ");  

  pulsacion = leer_Pulsador_Tactil();               // leemos el estado del pulsador tactil
  while(pulsacion==0){
     pulsacion = leer_Pulsador_Tactil();            // leemos el estado del pulsador tactil
     delay(350); //pausa para evitar rebotes
  } 
}



/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//      MODO AUTO
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

//========================================================
// ASIGNAR VALORES AL MODO AUTO
//========================================================
void programar_Modo_AUTO()
/*
 * Generacion del menu para entrada de tiempos a programar asi como la velocidad aproximada 
 * con que se inia el proceso. Esa velocidad sera una estimacion que debe ser corregida luego 
 * en tiempo de ejecucion una vez que el sistema reporte datos reales de las r.p.m. 
 * Para los valores e los tiempo dispomenos del rango entre 0 y 59 para cada uno de los campos HH:mm:ss
 * El valor de un campo queda aceptado al activar el pulsador, 'saltando' automaticamente al siguente campo.
 * --- Podemos corregir un valor ya aceptado de la siguiente manera:
 *          En la posicion minima del potenciometro el valor que obtenemos es -1, que el programa  
 *          nos representa como'--'. Si pulsamos en ese estado el programa interpetra que queremos 
 *          corregir el valor anterior y nos mueve a dicho campo
 */
{
  lcd.clear();
  lcd.setCursor(0, 0);
  //       ("0123456789ABCDEF");
  lcd.print(">Select Run Time");
  boolean pulsacion = false;

  int REG_ESTADO = 1;                   // variable que indicara en que campo del menu de seleccion estamos
  
  boolean FLAG_intro_run_time = true;   // bandera para indicar que aun no hemos terminado de introducir el tiempo
                                        // en el temporizador.
                                        
  boolean FLAG_intro_velocidad = false; // bandera para control de seleccion de velocidad (aproximada) de inicio 

  horas = -1;       // valores iniciados fuera de rango, lo que ahce que se muestren como "--"
  minutos = -1;     // indicando que debemos introducir un valor valido
  segundos = -1;    //
       
  while (FLAG_intro_run_time == true){
    int valorPotenciometro = analogRead(POTENCIOMETRO);   // el potenciometro nos permitira ingresar valores 
                                                          // del rango -1 y 59. 
                                                          // aceptar el valor ('--') le indica a la rutina que deseamos 
                                                          // volver sobre el valor anterior para corregirlo
    int valor_mapeado = map(valorPotenciometro,5,1020, -4,60);
    if (valor_mapeado == -1){
      valor_mapeado=0;
    }
    if (valor_mapeado <= -2){
      valor_mapeado=-1;
    }     

    if (valor_mapeado > 59){
      valor_mapeado=59;
    }
    if(REG_ESTADO < 1){
      REG_ESTADO = 1;
    }
    
    if(REG_ESTADO == 1){
      horas = valor_mapeado;
    }
    if(REG_ESTADO == 2){
      minutos = valor_mapeado;
    }
    if(REG_ESTADO == 3){
      segundos = valor_mapeado;
    }       

    lcd.setCursor(5, 1);
    //--print horas
    if(horas == -1){
      lcd.print("--");
    }
    else{
      if(horas<10){
        lcd.print("0");
      }
      lcd.print(horas);
    }
    lcd.print(":");
    //--print minutos
    if(minutos == -1){
      lcd.print("--");
    }
    else{
      if(minutos<10){
        lcd.print("0");
      }
      lcd.print(minutos);
    }
    lcd.print(":");
    //--print segundos
    if(segundos == -1){
      lcd.print("--");
    }
    else{
      if(segundos<10){
        lcd.print("0");
      }
      lcd.print(segundos);
    }
    
    pulsacion = digitalRead(PIN_PULSADOR_SELECCION);  // leemos el estado del pulsador tactil

    if ((pulsacion==true) AND (FLAG_intro_run_time == true) AND (valor_mapeado >=0)){
      pitidos(1);  //1 pitido
      delay(350);         //para evitar rebotes
      REG_ESTADO++;       // si las condiciones son correctas se acepta el valor para ese campo y se salta al siguiente
      pulsacion = false;  //anulamos el estado de la pulsacion por precaucion
    }
    
    if ((pulsacion==true) AND (FLAG_intro_run_time == true) AND (valor_mapeado <0)){
      REG_ESTADO--;       // si el campo sobre el que se activa el pulsador contiene '--' (-1)
                          // se vuelve al campo anterior
    }

    if ((REG_ESTADO == 4) AND (segundos >=0)){
      //si se completan los tres primeros campos correspondientes al tiempo HH:mm:ss
      //se cambia el texto de la primera linea y se procede a solicitar una velocidad de inicio
      //si el valor de los segundos no es correcto se vuelve a la seleccion de tiempos
      // esto permite tambien que aun estando el tiempo correcto situemos el cursor al minimo
      // y volvamos sobre el valor de los segundos u otros campos que deseemos
      lcd.setCursor(0, 0);
      //       ("0123456789ABCDEF");
      lcd.print("Pulsa OK -> vel.");
    }
    else{
      lcd.setCursor(0, 0);
      lcd.print(">Select Run Time");
    }
            
    if (REG_ESTADO==5){                 // completado el proceso de introducion del tiempo a temporizar
      FLAG_intro_run_time = false;      // se desactiva la posibilidad de volver atras para modificar el tiempo
      FLAG_intro_velocidad = true;      // y se pasa a la seleccion del rango de velocidad de inicio
    } 
  }
  //---selecciond e la velocidad inicial
  while (FLAG_intro_velocidad == true){
    lcd.setCursor(0, 0);
    //       ("0123456789ABCDEF");
    lcd.print("selec. Velocidad");
    lcd.setCursor(12, 0);
    int valorPotenciometro = analogRead(POTENCIOMETRO);
    int valor_mapeado = map(valorPotenciometro,0,1023, 3500,10600);   //  3500,10600)

    lcd.setCursor(0, 1);
    lcd.print("  -->    ");
    
    if (valor_mapeado >9999){
      lcd.setCursor(7, 1);
    }
    else{
      lcd.setCursor(8, 1);
    }
    valor_mapeado = valor_mapeado /100;    // nos interesa mostar solo valores de 100 en 100 
    lcd.print(valor_mapeado);
    lcd.print("00       ");                 // añadimos los ceros que nos faltan

    pulsacion = digitalRead(PIN_PULSADOR_SELECCION);
    if (pulsacion==true){
      pitidos(1);  //1 pitido
      delay(350); //para evitar rebotes
      pulsacion = false;
      FLAG_intro_velocidad = false;    // una vez aceptada la velocidad no podemos hacer mas cambios
      RPM_auto = valor_mapeado * 100;   // almacenamos el valro de velocidad deseada apra el modo Auto
    }   
  }

  delay(800);
  lcd.setCursor(0, 0);
  //       ("0123456789ABCDEF");
  lcd.print(" OK para INICIAR");
  
    while (pulsacion == false){
    pulsacion = digitalRead(PIN_PULSADOR_SELECCION);
    if (pulsacion==true){
      pitidos(1); //1 pitido
      delay(350); //para evitar rebotes
      pulsacion = true;  //por seguridad, pero no hace falta reasignarla
    }   
  }    
  //fin del menu de seleccion de opciones

  return;
}



/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//        AUTOAJUSTE DE VELOCIDAD
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

//========================================================
// SEGUNDA version, reciclada de una anterior
//========================================================

int autoAjuste_velociad_v2()
{
  float rpm_solicitadas = RPM_auto;     //velocidad en rpm para la que deseamos conocer su valor PWM equivalente
  float tolerancia_admitida = 10;      //tolerancia admitida para la velocidad buscada (en rpm
  
  float rpm_reales= -1;           //almacena las rpm calculadas. Inicializada fuera de rango
  
  int velocidad = 1000;           //velocidad de partida para iniciar el test (parado)

  int incremento_velocidad = 40;   //valor de los primeros incrementos usados al buscar una velocidad optima
                                  //se auto modifican para ajustarse  100 (80 parece que no)
                                 
  int velocidad_optima;           //velocidad que mas se ajusta a la buscada (si no es posible hallarla) 
  
  boolean flag_exceso = false;    //se activara si al incrementar el intervalo de busqueda, excedemos el 
                                  //el valor de velocidad deseado
                                 
  int intentos_tolerables = 0;    //contador de aproximaciones al valor optimo dentro de la tolerancia. 
                                  //Nos ayuda a abandonar la
                                  //busqueda si no existe la posibilidad de encontrar el valor deseado

  float velocidad_min = 0;        //PWM minima para casos en que la aproximacion no es buena
  float velocidad_max = 0;        //PWM maxima para casos en que la aproximacion no es buena
  float rpm_min = 0;              //rpm minima para casos en que la aproximacion no es buena
  float rpm_max = 0;              //rpm maxima para casos en que la aproximacion no es buena
  int contador_oscilaciones = 0;  //contador de oscilaciones para casos en el valor exacto no es posible

  lcd.clear();

  lcd.setCursor(0, 0);
  //       ("0123456789ABCDEF");
  lcd.print("  AUTO AJUSTE   ");

  /* bucle para acelerar suavemente hasta una velocidad inicial de 1150 (en uS)  */
  while (velocidad < 1080){
    velocidad+=20;
    SERVO_CONTINUO.writeMicroseconds(velocidad);      // refrescamos el dato 'dutyCycle' al servo
    delay(15);
  }

  //comienza el bucle de busqueda del valor PWM  
  while(true){
    SERVO_CONTINUO.writeMicroseconds(velocidad);
    delay(150); //para que se estabilice en la velocidad probada
    rpm_reales = RPM;

    if (abs(rpm_reales - rpm_solicitadas) < tolerancia_admitida){     //valor casi exacto al buscado
      return velocidad;                                               //abandonamos el bucle de busqueda
      } 

    if (rpm_reales < rpm_solicitadas){        //velocidad buscada no alcanzada aun
      flag_exceso = false;                    //reset de la bandera de exceso
      velocidad_min = velocidad;
      rpm_min = rpm_reales;
      velocidad += incremento_velocidad;      //incrementamos la velocidad a probar
    }
    
    if (rpm_reales > rpm_solicitadas){  //velocidad buscada sobrepasada
      if (flag_exceso == false){        //acabamos de pasar de una inferior a un exceso     
        incremento_velocidad = (incremento_velocidad /2); //reducimos el valor de los incrementos 
                                                          //pues nos estamos acercando 
        contador_oscilaciones += 1;
      }

      velocidad_max = velocidad;  //guardamos la velocidad actual como velocidad maxima
      rpm_max = rpm_reales;       //guardamos las r.p.m. actual como las maximas
      flag_exceso = true;         //activamos al bandera que indica sobrepasar la velocidad deseada
 
      velocidad -= incremento_velocidad;  // reducimos la velocidad
    }
      
    if (contador_oscilaciones >2){
      if (rpm_reales > rpm_solicitadas){
        return velocidad_min;
      }
      return velocidad;
    }
  }
}



/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//      SONIDO
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

//========================================================
//  FUNCION BASE PARA GENERAR PITIDOS CON EL ZUMBADOR
//========================================================

void activarAlarma(byte pin_Zumbador, int frecuencia, int tiempoON, int tiempoOFF, byte repeticiones)
/*
 * Funcion para generar los pitidos de aviso y pulsacion de teclas
 */
{
  for (byte i=0; i<repeticiones; i++){
    tone(pin_Zumbador, frecuencia);
    delay (tiempoON);
    noTone(pin_Zumbador);
    delay (tiempoOFF);
  }
} 


//========================================================
//  FUNCION 'HUMANA' para generar PITIDOS
//========================================================

void pitidos(byte numero_pitidos)
{
  //activarAlarma(PIN_ZUMBADOR, FRECUENCIA, TIEMPO_SONIDO, TIEMPO_SILENCIO, numero_pitidos);
  for (byte i=0; i<numero_pitidos; i++){
    tone(PIN_ZUMBADOR, FRECUENCIA);
    delay (TIEMPO_SONIDO);
    noTone(PIN_ZUMBADOR);
    if(numero_pitidos>1){
      delay (TIEMPO_SILENCIO);
    }
  }
}



//========================================================
//  FUNCION PARA TOCAR LA FANFARRIA
//========================================================

void playMusic() 
/*
 * Fanfarria de cachondeo (la cancion de la guerra de las galaxias)
 * Recuperada de un montaje anterior 
 * Fue escrita por DiConCon a peticion mia.
 */
{
  // star-wars-intro
  // ===============

  int frecuencia_galactica[] = {
    0, 294, 294, 294, 392, 587, 523, 494, 440, 784, 587, 523, 494, 440, 
    784, 587, 523, 494, 523, 440, 0, 0, 294, 0, 330, 0, 523, 494, 440, 
    392, 392, 440, 494, 440, 330, 370, 0, 294, 0, 330, 0, 523, 494, 440, 
    392, 587, 440, 440, 0, 294, 330, 330, 523, 494, 440, 392, 392, 440, 
    494, 440, 330, 370, 587, 587, 784, 698, 622, 587, 523, 466, 440, 392, 
    587, 0, 294, 294, 294, 392, 587, 523, 494, 440, 784, 587, 523, 494, 
    440, 784, 587, 523, 494, 523, 440, 0, 587, 784 
    };

  int tiempo_galactico[] = {
    254, 166, 166, 166, 506, 1014, 166, 166, 166, 506, 1014, 166, 166, 166,
    506, 1014, 166, 166, 166, 1014, 126, 438, 438, 110, 878, 110, 218, 218,
    218, 218, 146, 146, 146, 326, 110, 438, 54, 438, 54, 878, 110, 218, 
    218, 218, 218, 326, 110, 882, 54, 438, 878, 218, 218, 218, 218, 218, 
    146, 146, 146, 326, 110, 438, 326, 110, 326, 110, 326, 110, 326, 110, 
    326, 110, 1322, 110, 166, 166, 166, 506, 1014, 166, 166, 166, 506, 
    1014, 166, 166, 166, 506, 1014, 166, 166, 166, 1014, 254, 506, 2030
    };

  for (int i=22; i<74;i++){ //usamos solo un trozo, para abreviar   //for (int i=0; i<96;i++){ 
    tone(PIN_ZUMBADOR, frecuencia_galactica[i], tiempo_galactico[i]);
    delay(tiempo_galactico[i]);
  }
}



/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//       ATENCION A INTERRUPCIONES  SOFTWARE
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

//========================================================
// INICIAR TIMER 2
//========================================================

void start_Timer2()
{
  //Setup Timer2 para que se dispare cada 500 ms
  TCCR2B = 0x00;        // deshabilita Timer2 mientras lo estamos ajustando
  TCNT2  = 3;           // reset del contador iniciandolo con el valor 3
  TIFR2  = 0x00;        //Timer2 INT Flag Reg: borrar la bandera de desbordamiento
  TIMSK2 = 0x01;        //Timer2 INT Reg: habilita la interrupcion de desbordamiento de Timer2
  TCCR2A = 0x00;        //Timer2 Control Reg A: Wave Gen Mode normal
  TCCR2B = 0x07;        //Timer2 Control Reg B: Timer Prescaler set to 512
}



//========================================================
// PARAR TIMER 2
//========================================================

void stop_Timer2()
{
 TCCR2B = 0x00;         // deshabilita Timer2 mientras lo estamos ajustando
 TIMSK2=0 ;             // Timer2 INT Reg: deshabilita la interrupcion de desbordamiento de Timer2
}



//========================================================
// RUTINA DE INTERRUPCION PARA DESBORDAMIENTO DE TIMER 2
//========================================================

unsigned int timer2_count = 0;   //contador de numero de interrupciones

ISR(TIMER2_OVF_vect) {
  timer2_count++;                 // Incremento del contador de numero de interrupciones
  if(timer2_count > 31){

    timer2_count = 0;             // reset del contador de numero de interrupciones
    if (FLAG_modo_auto == true){
      CuentaAtras_ISR();
    }
    else{
      Reloj_ISR();   
    }
    
  }
  TCNT2 = 3;                      // reset del contador iniciandolo con el valor 3
  TIFR2 = 0x00;                   // Timer2 INT Flag Reg: borrar la bandera de desbordamiento
}


//========================================================
// RELOJ CON INTERRUPCION SOFTWARE DEL TIMER02
//========================================================

void Reloj_ISR()
/*
 * Reloj relativametne preciso mediante software
 * y el uso de interrupciones intermas del timer02 
 * Podemos ponerlo en hora actuando sobre las variables globales 
 * 'horas', 'minutos' y 'segundos'  definidas al inicio del codigo
 * 
 * Usado para mostrar tiempo en pantalla unicamente como un contador creciente
 */
{
  medioSegundo ++;
  FLAG_update_reloj = true;
  if(medioSegundo == 2){
    segundos ++;
    if(segundos == 60){
      segundos = 0;
      minutos ++;
      if(minutos == 60){
        minutos = 0;
        horas ++;
        if(horas == 24){
          horas = 0;
        }
      }
    }
    medioSegundo = 0;  
  }
  FLAG_punto_segundero = !FLAG_punto_segundero;
}



//========================================================
// CUENTA ATRAS CON INTERRUPCION SOFTWARE DEL TIMER02
//========================================================

void CuentaAtras_ISR()
/*
 * Contador descendente mediante el uso de interrupciones intermas del timer02
 */
{
  if(horas==0 AND minutos==0 AND segundos==0){    // por si se queda activo el servicio de ISR, salimos de
                                                  // esta rutina si el tiepo ha llegado a cero
                                                  // evitando asi que se produzcan valores negativos que
                                                  // generen errroes de representacion el el LCD
                                                  
    FLAG_fin_contador = true;                     // Activamos la bandera de fin de temporizador
    FLAG_update_reloj = false;                    // y la correspondiente al refresco del reloj
    return;
  }

  medioSegundo ++;                                // contabilizamos medios segundos
  FLAG_update_reloj = true;                       // que sonlos que generan el refresco del segundero (:)
  if(medioSegundo == 2){                          // cada dos medios segundos, 
    segundos --;                                  // descontamos 1 segundo
    if(segundos == -1){                           // Despues del segundo CERO, viene el 59
      segundos = 59;
      minutos --;                                 // y con ello descontar 1 minuto
      if(minutos == -1){                          
        minutos = 59;                             // El paso de CERO minutos lleva al 59
        horas --;                                 // y con ello descontar 1 hora
      }
    }
    medioSegundo = 0;                             // cada dos medios segundos, reiniciamos el proceso 
  }
  FLAG_punto_segundero = !FLAG_punto_segundero;   // cada medio segundo se activa/desactiva la bandera 
                                                  // para la gestion y refresco del segundero en el LCD
}



/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//       ATENCION A INTERRUPCIONES  HARDWARE
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

//========================================================
// FUNCION PARA LA INTERRUPCION INT0 (PIN 2) ARDUINO UNO
//========================================================

void cuentaVueltas() 
/*
 * Tacometro mediante las lecturas de pulsos con un sensor Hall 
 * y el servicio de interrupciones hardware EXTERNAS de ARDUINO UNO
 * 
 * OJO!!!
 * 
 * Se obtiene 2 pulsos por cada giro del motor, ya que disponemos de dos imanes
 * situados simetricamente marcando 'el diametro del giro'. 
 * Ponemos 2 imanes para crear estabilidad.
 */

{

//  /* obtencion de r.p.m para un solopulso (mas rapido y mas impreciso si hay
//     varios pulsos por vuelta y los imanes no estan situados con precision )*/
//  TIEMPO_ACTUAL = micros();
//  RPM = (uS de un minuto) / (tiempo entre pulsos * nº de pulsos por vuelta);
//  RPM = 30000000/(TIEMPO_ACTUAL - TIEMPO_ANTERIOR);
//  if ((TIEMPO_ACTUAL - TIEMPO_ANTERIOR) > 400){
//    RPM = 0;
//  }
//  TIEMPO_ANTERIOR = TIEMPO_ACTUAL;



  /* obtencion de r.p.m para el promedio de 'n' pulsos (mas estable)*/
  TIEMPO_ACTUAL = micros();
  CONTADOR +=1;

  //lo ideal es un contador igual al numero de imanes
  if (CONTADOR==2){   
    CONTADOR = 0;
    //RPM = (uS de un minuto  * CONTADOR) / (tiempo del CONTADOR * nº de pulsos por vuelta);
    RPM = 60000000/(TIEMPO_ACTUAL - TIEMPO_ANTERIOR); 
    TIEMPO_ANTERIOR = TIEMPO_ACTUAL; 
  }
  
}


//*******************************************************
//                    FIN DE PROGRAMA
//*******************************************************

