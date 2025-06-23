#include <msp432p401r.h>
#include "msp.h"
#include "lib_PAE.h"
#include "msp.h"
#include <stdbool.h>
#include "lib_p4_robot.h"
typedef uint8_t byte;

#define TXD2_READY (UCA2IFG & UCTXIFG)

#define ESTADO_RECTO 0
#define ESTADO_PAREDDERECHA 1
#define ESTADO_PAREDIZQUIERDA 2
#define ESTADO_ROTAR 3

#define VELOCIDADESTANDAR 700       // PROBADO CON 500
#define ROTACION 200                // 200 ESTA BIEN




struct RxReturn leer_sensor();
void Activa_TimerA2_TimeOut();
void Desactiva_TimerA2_TimeOut();

typedef struct RxReturn{
    byte StatusPacket[16];    // CAMBIO : [9]
    byte timeout;
};

/* funcions per canviar el sentit de les comunicacions */
void Sentit_Dades_Rx(void)
{ //Configuració del Half Duplex dels motors: Recepció
    P3OUT &= ~BIT0; //El pin P3.0 (DIRECTION_PORT) el posem a 0 (Rx)
}
void Sentit_Dades_Tx(void)
{ //Configuració del Half Duplex dels motors: Transmissió
    P3OUT |= BIT0; //El pin P3.0 (DIRECTION_PORT) el posem a 1 (Tx)
}
/* funció TxUACx(byte): envia un byte de dades per la UART 0 */
void TxUACx(uint8_t bTxdData)
{
    while(!TXD2_READY); // Espera a que estigui preparat el buffer de transmissió
    UCA2TXBUF = bTxdData;
}


/**
 * Método con el que inicializamos UART
 */
void Init_UART(void)
{
    UCA2CTLW0 |= UCSWRST; //Fem un reset de la USCI, desactiva la USCI
    UCA2CTLW0 |= UCSSEL__SMCLK; //UCSYNC=0 mode asíncron
    UCA2MCTLW = UCOS16; // Necessitem sobre-mostreig => bit 0 = UCOS16 = 1
    //Ara fixarem el nostre prescaler. En aquest cas, el nostre UART és de 8MHz com hem vist anteriorment.
    //Com utilitzem un un SMCLK a una freqüència de 24MHz, l'hem de divir de tal manera d'aconseguir 8MHz.
    //Per tant, si 8MHz=(24MHz/prescaler), prescaler=(24MHz/8MHz)=3
    UCA2BRW = 3;

    //volem un baud rate de 500 kbps i fem sobre-mostreig de 16 mostres
    //el rellotge de la UART ha de ser de 8MHz (ja que 500000*16= 8MHz).
    UCA2MCTLW |= (0x00 << 8); //UCBRSx, part fractional del baud rate

    //activamos pines de la uart
    P3SEL0 |= BIT2 | BIT3; //I/O funció: P1.3 = UART0TX, P1.2 = UART0RX
    P3SEL1 &= ~ (BIT2 | BIT3);
    //activamos que sea de salida
    P3DIR |= BIT0;
    Sentit_Dades_Rx(); //ejecutamos metodo para poder cambiar el sentido del flujo de datos
    P3SEL0 |= BIT2 | BIT3; //I/O funció: P1.3 = UART0TX, P1.2 = UART0RX
    P3SEL1 &= ~ (BIT2 | BIT3);

    UCA2CTLW0 &= ~UCSWRST;
    EUSCI_A2->IFG &= ~EUSCI_A_IFG_RXIFG; // Clear eUSCI RX interrupt flag
    EUSCI_A2->IE |= EUSCI_A_IE_RXIE;

}

//TxPacket() 3 paràmetres: ID del Dynamixel, Mida dels paràmetres, Instruction byte. torna la mida del "Return packet"
byte TxPacket(byte bID, byte bParameterLength, byte bInstruction, byte Parametros[16])
{
    byte bCount,bCheckSum,bPacketLength;
    byte TxBuffer[32];


    char error[] = "adr. no permitida";
    if ((Parametros[0] < 6) && (bInstruction == 3)){//si se intenta escribir en una direccion <= 0x05,
        //emitir mensaje de error de direccion prohibida:
          halLcdPrintLine(error, 8, INVERT_TEXT);
          //y salir de la funcion sin mas:
          return 0;
    }



    Sentit_Dades_Tx(); //El pin P3.0 (DIRECTION_PORT) el posem a 1 (Transmetre)
    TxBuffer[0] = 0xff; //Primers 2 bytes que indiquen inici de trama FF, FF.
    TxBuffer[1] = 0xff;
    TxBuffer[2] = bID; //ID del mòdul al que volem enviar el missatge
    TxBuffer[3] = bParameterLength+2; //Length(Parameter,Instruction,Checksum)
    TxBuffer[4] = bInstruction; //Instrucció que enviem al Mòdul

    for(bCount = 0; bCount < bParameterLength; bCount++) //Comencem a generar la trama que hem d’enviar
    {
        TxBuffer[bCount+5] = Parametros[bCount];
    }
    bCheckSum = 0;
    bPacketLength = bParameterLength+4+2;
    for(bCount = 2; bCount < bPacketLength-1; bCount++) //Càlcul del checksum
    {
        bCheckSum += TxBuffer[bCount];
    }
    TxBuffer[bCount] = ~bCheckSum; //Escriu el Checksum (complement a 1)
    for(bCount = 0; bCount < bPacketLength; bCount++) //Aquest bucle és el que envia la trama al Mòdul Robot
    {
        TxUACx(TxBuffer[bCount]);
    }
    while((UCA2STATW & UCBUSY)); //Espera fins que s’ha transmès el últim byte
    Sentit_Dades_Rx(); //Posem la línia de dades en Rx perquè el mòdul Dynamixel envia resposta
    return(bPacketLength);
}

void init_interrupciones()
{
    // Configuracion al estilo MSP430 "clasico":
    // --> Enable Port 4 interrupt on the NVIC.
    // Segun el Datasheet (Tabla "6-39. NVIC Interrupts", apartado "6.7.2 Device-Level User Interrupts"),
    // la interrupcion del puerto 1 es la User ISR numero 35.
    // Segun el Technical Reference Manual, apartado "2.4.3 NVIC Registers",
    // hay 2 registros de habilitacion ISER0 y ISER1, cada uno para 32 interrupciones (0..31, y 32..63, resp.),
    // accesibles mediante la estructura NVIC->ISER[x], con x = 0 o x = 1.
    // Asimismo, hay 2 registros para deshabilitarlas: ICERx, y dos registros para limpiarlas: ICPRx.

    //Habilitamos el timer 0 (posicion 8 )
    NVIC->ICPR[0] |= BIT8; //Primero, me aseguro de que no quede ninguna interrupcion residual pendiente para este puerto,
    NVIC->ISER[0] |= BIT8; //y habilito las interrupciones del puerto

    //Habilitamos el timer 1 (posicion 10 )
    NVIC->ICPR[0] |= BIT(10); //Primero, me aseguro de que no quede ninguna interrupcion residual pendiente para este puerto,
    NVIC->ISER[0] |= BIT(10); //y habilito las interrupciones del puerto

    //Habilitamos el timer 2 (posicion 12 )
    NVIC->ICPR[0] |= BIT(12); //Primero, me aseguro de que no quede ninguna interrupcion residual pendiente para este puerto,
    NVIC->ISER[0] |= BIT(12); //y habilito las interrupciones del puerto

    //Habilitamos el uart (posicion 18 )
    NVIC->ICPR[0] |= BIT(18); //Primero, me aseguro de que no quede ninguna interrupcion residual pendiente para este puerto,
    NVIC->ISER[0] |= BIT(18); //y habilito las interrupciones del puerto

}

void init_timers(void){

   //Para testear los ejercicios, hemos comentado el codigo. Borrad y comentad para ir ejecutando las
    //partes que querais. Hemos dejado descomentado inicialmente la segunda parte

    /**
     *
     * PRIMERA PARTE DE LA PRÃ�CTICA
     *
    ***/


   //ConfiguraciÃ³n a 10Khz
   //Timer A1, used for robot
   //Divider = 1; CLK source is SMCLK; clear the counter; MODE is up

    TIMER_A1->CTL = TIMER_A_CTL_ID__1 | TIMER_A_CTL_SSEL__ACLK | TIMER_A_CTL_CLR
            | TIMER_A_CTL_MC__UP;

   //el clock utilizado es el SMCLK, el qual funciona a 24MHz. Nuestra base de tiempo en esta primera parte
   //es de 10Khz, es por eso que si fijamos el timer a 240, obtendremos esos 10Khz, ya que 24MHz/240 = 10Khz
   TIMER_A1->CCR[0] = 240;     // 10 kHz
   TIMER_A1->CCTL[0] |= TIMER_A_CCTLN_CCIE; //Interrupciones activadas en CCR0




   //Timer A2, para contar segundos

   //Divider = 1; CLK source is ACLK; clear the counter; MODE is up
   TIMER_A2->CTL = TIMER_A_CTL_ID__1 | TIMER_A_CTL_SSEL__ACLK | TIMER_A_CTL_CLR
              | TIMER_A_CTL_MC__UP;
   //el clock utilizado para esta segunda parte es el ACLK, el qual funciona a 32768 tics por segundo.
   //En este caso, al querer una freqüència de 1Hz, simplemente fijamos el timer A1 32768-1, ya que al dividir
   //esto nos daria 1Hz
   TIMER_A2->CCR[0] = 32768 - 1; // Configuración para 1 Hz
   TIMER_A2->CCTL[0] |= TIMER_A_CCTLN_CCIE; //Interrupciones activadas en CCR0


}

/**
 * main.c
 */

static uint8_t tiempoRotacion;


void main(void){
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer

    //iniciamos todos los metodos antes de iniciar el main
    init_ucs_24MHz();
    Init_UART();

    init_interrupciones();
    __enable_interrupts();
    init_timers();

    volatile uint32_t i;
    uint32_t temps;
    uint8_t linea = 0;
    struct RxReturn resulSensor;
    //uint8_t resulSensorInt;

    halLcdInit();

    char numeroDerecha[3];         // variable para tener valor del sensor como string
    char numeroMedio[3];           //
    char numeroIzquierda[3];       //
    char Derecha[8] = "DERECHA";   // fila 0
    char Medio[6] = "MEDIO";       // fila 3
    char Izquierda[10] = "IZQUIERDA";  // fila 6

    // CODIGO PARA IMPRIMIR NUESTROS NOMBRES

    char nombre[19] = "PAE Andres y Adria";

    halLcdClearScreen(0);

    halLcdPrintLine(Derecha, 0 ,0);  // Imprimir texto derecha
    halLcdPrintLine(Medio, 3 ,0);    // Imprimir texto medio
    halLcdPrintLine(Izquierda, 6 ,0);  // Imprimir texto izquierda

    halLcdPrintLine(nombre, 8 ,0);      // nombre

    move_up(VELOCIDADESTANDAR); //empezamos a movernos rectos
    //esta variable se encargarà de guardar el estado actual de movimiento de nuestro robot
    uint8_t estado = ESTADO_RECTO; //En el caso del comienzo, el estado inicial será el movimiento recto hacia delante

    //prueba de mover hacia delante, parar, mover hacia atras, parar, rotar a la derecha, parar, mover a la izquierda y parar
    while(1){

        resulSensor = leer_sensor();

        sprintf(numeroIzquierda, "%03d", resulSensor.StatusPacket[5]); // Convertir a cadena
        sprintf(numeroDerecha, "%03d", resulSensor.StatusPacket[7]); // Convertir a cadena
        sprintf(numeroMedio, "%03d", resulSensor.StatusPacket[6]); // Convertir a cadena

        halLcdPrintLine(numeroDerecha, 1 ,0);
        halLcdPrintLine(numeroMedio,4,0);
        halLcdPrintLine(numeroIzquierda,7,0);


        /* *****
         *
         * ESTADO INICIAL
         *
         * ******
         */
        if ( resulSensor.StatusPacket[6] > 30 && estado == ESTADO_RECTO ) {   //si el sensor frontal(en el estado recto) detecta una pared cerca

            stop();
            char estado1[15] = "Estado Inicial";
            halLcdPrintLine(estado1, 8 ,0);
            tiempoRotacion = 0; //variable con el que regularemos el tiempo de giro del robot

            Activa_TimerA1_TimeOut(); //activamos timer
            //para hacer el giro a la izquierda antes de que se choque con la pared, lo regularemos mediante dos condicionales dentro de un while
            do {

                move_up_custom(1000,200);   //empezamos a girar a la izquierda
                resulSensor = leer_sensor();

                sprintf(numeroIzquierda, "%03d", resulSensor.StatusPacket[5]); // Convertir a cadena
                sprintf(numeroDerecha, "%03d", resulSensor.StatusPacket[7]); // Convertir a cadena
                sprintf(numeroMedio, "%03d", resulSensor.StatusPacket[6]); // Convertir a cadena

                halLcdPrintLine(numeroDerecha, 1 ,0);
                halLcdPrintLine(numeroMedio,4,0);
                halLcdPrintLine(numeroIzquierda,7,0);

                if (resulSensor.StatusPacket[6] > 110) {  //si al estar girando vemos que se acerca demasiado a la pared, paramos
                                   break;
                               }
                //el tiempo de duración del giro estará regulado por un timer, el cual, si el tiempo de rotación es mayor que 1, paramos de girar
                if (tiempoRotacion > 1) {
                    break;
                }

            } while (resulSensor.StatusPacket[7] < 80); // esto se repetirá siempre que la distancia del sensor derecho a la pared sea menos de 80

            //Desactiva_TimerA1_TimeOut();

            //una vez hemos acabado el giro, seguiremos el camino yendo recto
            move_up(VELOCIDADESTANDAR);
            estado = ESTADO_PAREDDERECHA; //Ahora nuestro estado cambia a pared derecha
        }




        /* *****
         *
         * ESTADO PARED DERECHA
         *
         * ******
         */
        //SE VA A CHOCAR CONTRA LA PARED FRONTAL, ROTAMOS SU EJE HASTA QUE SE ALINIE EN PARALELO CON LA PARED Y SEGUIMOS RECTOS
        if ( resulSensor.StatusPacket[6] > 110 && estado == ESTADO_PAREDDERECHA) {   // SI SE VA A CHOCAR EN PARED DERECHA

            halLcdClearLine(8);
            char der1[14] = "derecha pared";
            halLcdPrintLine(der1, 8 ,0);

            stop(); //antes de rotar, nos detenemos
            move_rotation(900); //procedemos a rotar el robot

            sprintf(numeroIzquierda, "%03d", resulSensor.StatusPacket[5]); // Convertir a cadena
            sprintf(numeroDerecha, "%03d", resulSensor.StatusPacket[7]); // Convertir a cadena
            sprintf(numeroMedio, "%03d", resulSensor.StatusPacket[6]); // Convertir a cadena

            halLcdPrintLine(numeroDerecha, 1 ,0);
            halLcdPrintLine(numeroMedio,4,0);
            halLcdPrintLine(numeroIzquierda,7,0);

            do {
                //leemos los valores de los sensores y los imprimimos
                resulSensor = leer_sensor();

            } while (resulSensor.StatusPacket[6] != 0); //rotaremos hasta que el frontal detecte como lejos la pared

            //seguimos moviendonos hacia delante
            move_up(VELOCIDADESTANDAR);
        }



        //CASO DONDE NOS ALEJAMOS MUCHO DE LA PARED DERECHA, RECTIFICAMOS GIRANDO A LA DERECHA PARA VOLVERNOS A ACERCAR A ESTA
        if ( resulSensor.StatusPacket[7] == 0 && estado == ESTADO_PAREDDERECHA) {   // SI LA PARED DERECHA ESTA MUY LEJOS

            halLcdClearLine(8);
            char estado3[12] = "Pared Lejos";
            halLcdPrintLine(estado3, 8 ,0);
            //detenemos el movimiento
            stop();

            do {
                //empezamos a girar a la derecha
                move_up_custom(300,900);
                resulSensor = leer_sensor();

                sprintf(numeroIzquierda, "%03d", resulSensor.StatusPacket[5]); // Convertir a cadena
                sprintf(numeroDerecha, "%03d", resulSensor.StatusPacket[7]); // Convertir a cadena
                sprintf(numeroMedio, "%03d", resulSensor.StatusPacket[6]); // Convertir a cadena

                halLcdPrintLine(numeroDerecha, 1 ,0);
                halLcdPrintLine(numeroMedio,4,0);
                halLcdPrintLine(numeroIzquierda,7,0);

                if (resulSensor.StatusPacket[6] > 110) {  // si se va a comer una pared que salga asap
                    break;
                }

            } while (resulSensor.StatusPacket[7] < 90); //giraremos a la derecha siempre que la distancia del sensor derecho sea menos de 90

            move_up(VELOCIDADESTANDAR);

        }

        //CASO DONDE LA PARED DERECHA ESTA DEMASIADO CERCA, ROTAMOS EL EJE A LA IZQUIERDA Y SEGUIMOS RECTOS
        if ( resulSensor.StatusPacket[7] > 90 && estado == ESTADO_PAREDDERECHA) {   // SI LA DERECHA ESTA MUY CERCA

            halLcdClearLine(8);
            char estado4[12] = "Pared Cerca";
            halLcdPrintLine(estado4, 8 ,0);

            stop();

            do {

                move_rotation_left(900);  //empezamos a movernos a la izquierda
                resulSensor = leer_sensor();

                sprintf(numeroIzquierda, "%03d", resulSensor.StatusPacket[5]); // Convertir a cadena
                sprintf(numeroDerecha, "%03d", resulSensor.StatusPacket[7]); // Convertir a cadena
                sprintf(numeroMedio, "%03d", resulSensor.StatusPacket[6]); // Convertir a cadena

                halLcdPrintLine(numeroDerecha, 1 ,0);
                halLcdPrintLine(numeroMedio,4,0);
                halLcdPrintLine(numeroIzquierda,7,0);

                if (resulSensor.StatusPacket[6] > 110) {  //si al girar a la izquierda vemos que se va a chocar con una pared frontal, paramos
                    break;
                }

            } while (resulSensor.StatusPacket[7] > 90); //seguiremos girando siempre que la distancia con la pared derecha sea mayor que 90

            //una vez completado el giro, procedemos a ir rectos
            move_up(VELOCIDADESTANDAR);

        }
}}

static byte Byte_Recibido = 0;
static byte DatoLeido_UART;
static uint8_t tiempoGlobal;

void EUSCIA2_IRQHandler (void)
{ //interrupcion de recepcion en la UART A0

    EUSCI_A2->IFG &=~ EUSCI_A_IFG_RXIFG; // Clear interrupt
    UCA2IE &= ~UCRXIE; //Interrupciones desactivadas en RX
    DatoLeido_UART = UCA2RXBUF;
    Byte_Recibido=1;
    UCA2IE |= UCRXIE; //Interrupciones reactivadas en RX

}
void Activa_TimerA1_TimeOut(void){
    TIMER_A1->CCTL[0] |= TIMER_A_CCTLN_CCIE; //Interrupciones activadas en CCR0 ???
    TIMER_A1->CCR[0] = 0;
}
void Desactiva_TimerA1_TimeOut(void){
    TIMER_A1->CCTL[0] &= ~TIMER_A_CCTLN_CCIE;
}


void Activa_TimerA2_TimeOut(void){
    TIMER_A2->CCTL[0] |= TIMER_A_CCTLN_CCIE; //Interrupciones activadas en CCR0 ???
}
void Desactiva_TimerA2_TimeOut(void){
    TIMER_A2->CCTL[0] &= ~TIMER_A_CCTLN_CCIE;
}



/**
 * Metodo con el cual reiniciamos el valor del timer (tiempoGlobal) para el metodo recibir
 */
void Reset_Timeout(void){
    tiempoGlobal = 0;
}
/**
 * Metodo que comprueba que si el tiempo es mayor que el valor fijado, devuelve 1, si no 0.
 */
byte TimeOut(uint8_t tiempoMaximo){

    if (tiempoMaximo < tiempoGlobal){
        return 1;
    }

    return 0;

}

void TA0_0_IRQHandler (void){
    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG; //Hem de netejar el flag de la interrupció
    //Codi de la IRQ
}
/**
 * SUbrutina que cada vez que será llamada, aumenta un valor global
 */
void TA1_0_IRQHandler (void)
{
    TIMER_A1->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG; //Hem de netejar el flag de la interrupció
    //Codi de la IRQ
    tiempoGlobal++;

}
void TA2_0_IRQHandler (void)
{
    TIMER_A2->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG; //Hem de netejar el flag de la interrupció
    tiempoRotacion++;
}

struct RxReturn RxPacket(void) {
    struct RxReturn respuesta;

    byte bCount, bLenght, bChecksum;
    byte Rx_time_out=0;
    Sentit_Dades_Rx(); //Ponemos la linea half duplex en Rx
    Activa_TimerA1_TimeOut();
    for(bCount = 0; bCount < 4; bCount++) //bRxPacketLength; bCount++)
    {
        Reset_Timeout();
        Byte_Recibido=0; //No_se_ha_recibido_Byte();
        while (!Byte_Recibido) //Se_ha_recibido_Byte())
        {
            Rx_time_out=TimeOut(1000); // tiempo en decenas de microsegundos
            if (Rx_time_out)break;//sale del while
        }
        if (Rx_time_out)break;  //sale del for si ha habido Timeout
        //Si no, es que todo ha ido bien, y leemos un dato:
        respuesta.StatusPacket[bCount] = DatoLeido_UART; //Get_Byte_Leido_UART();
    }//fin del for

    if (!Rx_time_out) { // Continua llegint la resta de bytes del Status Packet

        // leemos los bytes siguientes, el numero de bytes correspondiendo a la length
        // que se recibe en la 3a posicion
        for(bLenght = 0; bLenght < respuesta.StatusPacket[3]; bLenght++) {
            Reset_Timeout();
            Byte_Recibido=0;
            while (!Byte_Recibido)
            {
                Rx_time_out=TimeOut(1000); // tiempo en decenas de microsegundos
                if (Rx_time_out)break;//sale del while
            }
            if (Rx_time_out)break;  //sale del for si ha habido Timeout
            //Si no, es que todo ha ido bien, y leemos un dato:
            respuesta.StatusPacket[bCount+bLenght] = DatoLeido_UART; //Get_Byte_Leido_UART();
        }//fin del for

        byte checkSum = 0;


        for(bCount = 2; bCount < 4+respuesta.StatusPacket[3]-1; bCount++) {
            checkSum += respuesta.StatusPacket[bCount];
        }

    }

    // Desactivamos el timer
    Desactiva_TimerA1_TimeOut();

    return respuesta;
}
/**
 * Método para poder mover el motor, donde cada parametro indica el movimiento a realizar
 */
void move_motor(byte id_motor, byte sentido, uint32_t velocidad){
    byte Parametros[3];
    Parametros[0]=32; //el 32 corresponde al move speed
    Parametros[1]=velocidad; //este parametro correspondrá a la velocidad pasada por nosotros
    Parametros[2]=((sentido<<2)|(velocidad>>8)); // como el registro moving speed H tiene el valor del Turn Direction y parte del speed
                                                // ejecutamos esta or para asignar cada valor donde corresponde
    TxPacket(id_motor,3,3,Parametros); //una vez tenemos los parametros, los enviamos. Al ser write, correspnde al 3

    RxPacket(); // CAMBIO
}
/**
 * Método para poder leer los valores del sensor
 */
struct RxReturn leer_sensor(){
    byte Parametros[2];
    Parametros[0]=26; //sensor left
    Parametros[1]=3; //numeros de sensores totaales
    TxPacket(100,2,2,Parametros); //los enviamos
    return RxPacket(); //recibimos la info y la devolvemos
}
