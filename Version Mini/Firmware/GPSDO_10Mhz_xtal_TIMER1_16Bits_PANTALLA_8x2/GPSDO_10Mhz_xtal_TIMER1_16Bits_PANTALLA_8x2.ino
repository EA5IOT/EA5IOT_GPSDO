// EA5IOT GPSDO SMT32f103C8T6 10Mhz OCXO FREQUENCY PWM CONTROL

// Librerias
#include <EEPROM.h>
#include "LiquidCrystal2.h"
//

// PATAS Microcontrolador
//      TX2         PA2															                            // TX RS232 PC (al RX del PC)
//		  RX2		      PA3															                            // RX RS232 PC (al TX del PC)
//      TX3         PB10															                          // TX GPS (al RX del GPS), cable verde
// 		  RX3		      PB11															                          // RX GPS (al TX del GPS), cable blanco

#define Timer1Ch1   PA8                                                         // Señal de 1PPS del gps (Activa cuando el GPS está enganchado), cable naranja
#define Timer1Ch2   PA9                                                         // Salida PWM Control de Frecuencia
#define Timer1Ch3   PA10															                          // Salida PWM contraste pantalla, cable verde pata 3 Pantalla

#define RS		      PB9															                            // Pin 4 pantalla, cable blanco
//#define RW              																                      // Pin 5 (a masa directamente, no se usa y así evitamos poner los conversores de 5 a 3,3V)
#define EN		      PB8															                            // Pin 6 pantalla, cable amarillo
#define DB7		      PB7															                            // Pin 14 pantalla, cable azul
#define DB6		      PB6															                            // Pin 13 pantalla, cable naranja
#define DB5		      PB5															                            // Pin 12 pantalla, cable marron

//#define DB4 	      PB15														                            // Pin 11 pantalla, cable violeta
#define DB4         PB4                                                         // Pin 11 pantalla, cable violeta

#define E1		      PA7															                            // Pin 1 encoder/pulsador
#define E2          PA6															                            // Pin 2 encoder/pulsador
#define EP          PA5															                            // Pin pulsador encoder/pulsador

#define LED         PC13                                                        // LED pata PC13

#define Filtro_Ruido 10                                                         // Milisegundos de retardo para filtrar los rebotes del pulsador/botones
//

// Variables y constantes
uint16_t Timer1MSB;                                                             // Para almacenar las veces que se desborda el TIMER3 y poder contar los pulsos totales contados entre periodos
uint16_t CaptureNew;
uint16_t CaptureAnt;
uint32_t CuentaTotal;
double FrecEstimada, FrecRequerida, FrecCentral;
int Offset;
double Pwm;
double Dif;
uint16_t MaxDes;
bool PrimerValor;
bool DatoNuevo;
int Contraste;
int Menu;
bool T1;
bool T2;
bool T3;
bool T4;
bool E1_Ant;
bool E2_Ant;
bool EP_Ant;
bool Derecha;
bool Izquierda;
bool Pulsador;
unsigned long DelayE1;
unsigned long DelayE2;
//

LiquidCrystal2 LCD(RS, EN, DB4, DB5, DB6, DB7);                                 // La libreria hay que modificarla porque al usar "dealay" interfiere en las rutinas del GPSDO

void setup()
{
  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);                                     // Se deshabilitan los pines JTAG, pero se permiten los pines para ST-LINK
  Cambiar_CPU_CLK();                                                            // Cambiamos el reloj del sistema a 100Mhz con cristal externo de 10Mhz
  
  Timer1MSB = 0;
  CaptureNew = 0,
  CaptureAnt = 0;
  CuentaTotal = 0;
  FrecEstimada = FrecRequerida = FrecCentral = 100000000.0;                     // Frecuencia central del PLL en Hz (100Mhz / 10 = 10Mhz)
  Dif = 0.0;
  MaxDes = 10000;                                                               // Maxima desviación permitida para tracking del filtro en Hz sobre 100Mhz
                                                                                // 10000Hz en 100Mhz = 500ppm
                                                                                // 1ppm = 1Hz en 1Mhz -> 10Hz en 10Mhz -> 100Hz en 100Mhz
                                                                                // +-250ppb el STP2145A, +-1,5ppm el CTI
                                                                                // +-1,5ppm = +-150Hz en 100Mhz
                                                                                // +-0,25ppm = +- 25Hz en 100Mhz
  PrimerValor = false;
  DatoNuevo = false;
  
  Pwm = 32768.0;                                                                // Iniciamos el PWM al 50%

  Menu = 0;
  E1_Ant = true;
  E2_Ant = true;
  EP_Ant = true;
  T1 = false;
  T2 = false;
  T3 = false;
  T4 = false;
  Derecha = false;
  Izquierda = false;
  Pulsador = false;
  DelayE1 = 0;
  DelayE2 = 0;

  pinMode(E1, INPUT_PULLUP);
  pinMode(E2, INPUT_PULLUP);
  pinMode(EP, INPUT_PULLUP);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);

  LeerEEPROM();                                                                 // Se lee el valor del contraste y el offset de frecuencia almacenados en la EEPROM

  LCD.begin(8, 2);
  delay(100);                                                                   // No se puede usar delay despues de iniciar los timers
  LCD.print(" EA5IOT");
  LCD.setCursor(0, 1);
  LCD.print(" GPSDO");
  delay(2000);

  Serial2.begin(82944);                                                         // PC serie, equivale a 115200 baudios, ya que el entorno se cree que tiene un reloj de 72Mhz
  Serial3.begin(3456);                                                          // GPS serie, equivale a 4800 baudios, ya que el entorno se cree que tiene un reloj de 72Mhz 
  
  IniciarContador();                                                            // Se inicializan los timers y se ponen en marcha los PWM
  CambiarContraste();                                                           // Se cambia el contraste al valor almacenado en la EEPROM despues de poner en marcha el PWM
}

void loop()
{
  ComprobarBotones();
  PintarMenu();
}

void LeerEEPROM()
{
  if (EEPROM.read(0) == 0xAA)
  {
    Contraste = EEPROM.read(1);
    Offset = EEPROM.read(2);
    FrecRequerida = FrecCentral + Offset;
  } else
    {
      EEPROM.write(0, 0xAA);
      EEPROM.write(1, 0);
      EEPROM.write(2, 0);
      Contraste = 0;
      Offset = 0;
    };
  
  return;
}

void PintarMenu()
{
  switch (Menu)
  {
    case 0:
      if (DatoNuevo)
      {
        LCD.setCursor(0, 0);
        LCD.print("ppb     ");
        LCD.setCursor(4, 0);
        LCD.print(Dif * 10.0, 1);                                               // Error en partes por billon de la frecuencia del oscilador
        LCD.setCursor(0, 1);
        LCD.print("pwm     ");
        LCD.setCursor(3, 1);
        LCD.print(uint16_t(Pwm));                                               // PWM aplicado para conseguir el minimo error posible
    
        /*                                                                      // Activar si se quiere enviar datos por el puerto serie2 TTL 3.3V
        Serial2.print("FL: ");
        Serial2.print(CuentaTotal1);
        Serial2.print(" Hz     FE: ");
        Serial2.print(" ");
        Serial2.print(FrecEstimada1, 3);
        Serial2.print(" Hz    ");
        Serial2.print("   Dif: ");
        Serial2.print(Dif1, 3);
        Serial2.print("   PWM: ");
        Serial2.println(Pwm, 3);
        */

        DatoNuevo = false;
        digitalWrite(LED, !digitalRead(LED));
      };

      if (Pulsador == true)
      {
        Menu = 1;
        Pulsador = Derecha = Izquierda = false;
      };
    break;

    case 1:
      LCD.setCursor(0, 0);
      LCD.print("CONTRAST");
      LCD.setCursor(0, 1);
      LCD.print(Contraste);
      LCD.print("       ");
      
      if (Derecha == true)
      {
        Menu = 2;
        Pulsador = Derecha = Izquierda = false;
      } else if (Izquierda == true)
        {
          Menu = 2;
          Pulsador = Derecha = Izquierda = false;
        } else if (Pulsador == true)
          {
            Menu = 10;
            Pulsador = Derecha = Izquierda = false;
          };
    break;

    case 2:
      LCD.setCursor(0, 0);
      LCD.print(" OFFSET ");
      LCD.setCursor(0, 1);
      LCD.print(double(Offset/10.0), 1);
      LCD.print("Hz     ");

      if (Derecha == true)
      {
        Menu = 1;
        Pulsador = Derecha = Izquierda = false;
      } else if (Izquierda == true)
        {
          Menu = 1;
          Pulsador = Derecha = Izquierda = false;
        } else if (Pulsador == true)
          {
            Menu = 20;
            Pulsador = Derecha = Izquierda = false;
          };      
    break;

    case 10:
      if (Derecha == true)
      {
        Contraste++;
        if (Contraste > 255) Contraste = 255;
        CambiarContraste();
        Pulsador = Derecha = Izquierda = false;
      } else if (Izquierda == true)
        {
          Contraste--;
          if (Contraste < 0) Contraste = 0;
          CambiarContraste();
          Pulsador = Derecha = Izquierda = false;
        } else if (Pulsador == true)
          {
            EEPROM.write(1, Contraste);
            Menu = 0;
            Pulsador = Derecha = Izquierda = false;
          };
          
      LCD.setCursor(0, 0);
      LCD.print("CONTRAST");
      LCD.setCursor(0, 1);
      LCD.print(Contraste);
      LCD.print("   ");
    break;

    case 20:
      if (Derecha == true)
      {
        Offset++;
        if (Offset > 1000) Offset = 1000;
        Pulsador = Derecha = Izquierda = false;
      } else if (Izquierda == true)
        {
          Offset--;
          if (Offset < -1000) Offset = -1000;
          Pulsador = Derecha = Izquierda = false;
        } else if (Pulsador == true)
          {
            EEPROM.write(2, Offset);
            FrecRequerida = FrecCentral + Offset;
            Menu = 0;
            Pulsador = Derecha = Izquierda = false;
          };

      LCD.setCursor(0, 0);
      LCD.print(" OFFSET ");
      LCD.setCursor(0, 1);
      LCD.print(double(Offset/10.0), 1);
      LCD.print("Hz     ");
    break;

    default:
    break;
  };
  
  return;
}

void Timer1UpdateInt(void)
{
  Timer1MSB++;                                                                  // Se cuentan los overflows del contador
  TIMER1->regs.gen->SR &= 0x1EFE;                                               // Se resetea la bandera de update
  
  return;
}

void Timer1InputCaptureInt(void)
{
  uint16_t temp;
  double k;
    
  CaptureNew = TIMER1->regs.gen->CCR1;                                          // Se almacena el valor del contador cuando se produce el pulso de 1pps
  TIMER1->regs.gen->SR &= 0x1EFD;                                               // Se resetea la bandera de input capture canal1
  temp = Timer1MSB;
  Timer1MSB = 0;                                                                // Se inicia el contador de desbordamientos del contador principal
      
  if (PrimerValor == false)
  {
    CaptureAnt = CaptureNew;
    PrimerValor = true;
  } else
  {
    CuentaTotal = (65536 * (temp - 1)) + (65536 - CaptureAnt) + CaptureNew;     // Se calculan los pulsos totales contados
    CaptureAnt = CaptureNew;

    if ((CuentaTotal > (FrecRequerida - MaxDes)) && (CuentaTotal < (FrecRequerida + MaxDes)))   // Se comprueba que no se haya leido mal la frecuencia del Xtal
    {
      k = abs(Dif) * 0.0018;
      k += 0.05;                                                                // Valor minimo de k
      if (k > 0.9) k = 0.9;                                                     // Valor maximo de k
      FrecEstimada += k * (CuentaTotal - FrecEstimada);                         // Se estima la Frecuencia actual del XTAL
      Dif = (FrecRequerida - FrecEstimada);                                     // Se calcula el error entre a frecuencia estimada y la frecuencia requerida
      Pwm += Dif * 50.0 * k;                                                    // Se varía el incremento del PWM en función de la cantidad de error, a mas error mayor variación

      if (Pwm > 65535.0) Pwm = 65535.0;
      if (Pwm < 0.0) Pwm = 0.0;
      TIMER1->regs.gen->CCR2 = uint16_t(Pwm);                                   // Se actualiza el valor del PWM para controlar la frecuencia del oscilador
    
      DatoNuevo = true;
    };
  };

  return;
}

void ComprobarBotones(void)
{
  bool tempE1 = digitalRead(E1);
  bool tempE2 = digitalRead(E2);
  bool tempEP = digitalRead(EP);
  bool t1_ant = T1;
  bool t2_ant = T2;
  bool t3_ant = T3;
  bool t4_ant = T4;

  // Derecha ---> T1>T3 > T2>T4
  // Izquierda -> T4>T2 > T3>T1

  if ((tempE1 == HIGH) && (E1_Ant == LOW) && ((millis() - DelayE1) > Filtro_Ruido))
  {
    T1 = true;
    DelayE1 = millis();
    if (t3_ant == true)
    {
      Izquierda = true;
      T1 = T2 = T3 = T4 = false;
    };
  } else
    if ((tempE1 == LOW) && (E1_Ant == HIGH) && ((millis() - DelayE1) > Filtro_Ruido))
    {
      T2 = true;
      DelayE1 = millis();
      if (t4_ant == true)
      {
        Izquierda = true;
        T1 = T2 = T3 = T4 = false;
      };
    };

  if ((tempE2 == HIGH) && (E2_Ant == LOW) && ((millis() - DelayE2) > Filtro_Ruido))
  {
    T3 = true;
    DelayE2 = millis();
    if (t1_ant == true)
    {
      Derecha = true;
      T1 = T2 = T3 = T4 = false;
    };    
  } else
    if ((tempE2 == LOW) && (E2_Ant == HIGH) && ((millis() - DelayE2) > Filtro_Ruido))
    {
      T4 = true;
      DelayE2 = millis();
      if (t2_ant == true)
      {
        Derecha = true;
        T1 = T2 = T3 = T4 = false;
      };      
    };

  if ((tempEP == LOW) && (EP_Ant == HIGH) && ((millis() - DelayE2) > Filtro_Ruido))
  {
    Pulsador = true;
  };

  E1_Ant = tempE1;
  E2_Ant = tempE2;
  EP_Ant = tempEP;
 
  return;
}

void CambiarContraste(void)
{
  TIMER1->regs.gen->CCR3 = Contraste * 256;	
	
  return;
}

void Cambiar_CPU_CLK(void)
{
  RCC_BASE->CR |= 0x1;                                                          // Activa el HSI
  while ((RCC_BASE->CR) & 0x2 == 0) {};                                         // Espera a que el HSI se ponga en marcha
  RCC_BASE->CFGR &= 0xFFFFFFFC;                                                 // Cambia al HSI como reloj del sistema
  while ((RCC_BASE->CFGR & 0xC) != 0) {};                                       // Espera a que el reloj del sistema sea el HSI
  RCC_BASE->CR &= 0xFFFEFFFF;                                                   // Desactiva el HSE
  while ((RCC_BASE->CR & 0x20000) != 0) {};                                     // Espera a que el HSE se pare
  RCC_BASE->AHBENR &= 0xFFFFEFFF;                                               // Desacta el reloj del USB antes de descactivar el PLL
  RCC_BASE->CR &= 0xFEFFFFFF;                                                   // Desactiva el PLL
  while ((RCC_BASE->CR & 0x2000000) != 0) {};                                   // Espera a que el PLL se pare
  RCC_BASE->CFGR &= 0xFFC3FFFF;                                                 // Pone los 0 en el PLL para multiplicar por 10
  RCC_BASE->CFGR |= 0x200000;                                                   // Pone los 1 en el PLL para multiplicar por 10
  RCC_BASE->CR |= 0x10000;                                                      // Activa el HSE
  while ((RCC_BASE->CR & 0x20000) == 0) {};                                     // Espera a que el HSE se ponga en marcha
  RCC_BASE->CR |= 0x1000000;                                                    // Activa el PLL
  while ((RCC_BASE->CR & 0x2000000) == 0) {};                                   // Espera a que el PLL esté enganchado
  RCC_BASE->CFGR |= 0x2;                                                        // Cambia al PLL como reloj del sistema
  while ((RCC_BASE->CFGR & 0x8) == 0) {};                                       // Espera a que el reloj del sistema sea el PLL 

  return;
}

void IniciarContador(void)
{ 
  // Deshabilitamos el TIMER1
  TIMER1->regs.gen->CR1   = 0x0000;                                             // Se deshabilita el contador

  // Input Compare en CH1 (1pps GPS)
  pinMode(Timer1Ch1, INPUT);
  TIMER1->regs.gen->CCMR1 = 0x0001;                                             // Input Capture Channel 1 mapped to TI1->PA8
  TIMER1->regs.gen->CCER  = 0x0001;                                             // Input Capture Enable, Timer1_Channel1 rising edge on TI1->PA8 para el 1PPS del GPS
  TIMER1->regs.gen->DIER  = 0x0007;                                             // Se activan las interrupciones de Update, Input Capture1 y Output Compare2
  TIMER1->regs.gen->PSC   = 0x0000;                                             // Hay que poner un cero en el preescaler del TIMER1 para que cuente a 100Mhz, porque las librerias lo arrancan a 2

  // PWM en CH2
  TIMER1->regs.gen->CCMR1 |= 0x6800;                                            // Output Compare CH2
  TIMER1->regs.gen->CCER  |= 0x0010;                                            // CH2 Output Compare ON - Polarity HIGH
  pinMode(Timer1Ch2, PWM);                                                      // No hay manera de acceder al registro BDTR para activar el OUTPUT COMPARE, asi hace lo mismo
  TIMER1->regs.gen->CCR2  = 0x8000;                                             // PWM al 50%

  GPIOA->regs->CRH &= 0xFFFFFF3F;                                             	// Se hace un and para poner los ceros
  GPIOA->regs->CRH |= 0x000000C0;                                               // Se hace un or para poner los unos, alternate function open drain PA9 para PWM de 5V    
  //GPIOA->regs->CRH   |= 0x00000080;                                           // Se hace un or para poner los unos, alternate function push-pull PA9 para PWM de 3,3V

  // PWM en CH3
  TIMER1->regs.gen->CCMR2 |= 0x0068;                                            // Output Compare CH3
  TIMER1->regs.gen->CCER  |= 0x0100;                                            // CH3 Output Compare ON - Polarity HIGH
  pinMode(Timer1Ch3, PWM);                                                      // No hay manera de acceder al registro BDTR para activar el OUTPUT COMPARE, asi hace lo mismo
  TIMER1->regs.gen->CCR3  = 0x8000;                                             // PWM al 50% 

  // Habilitamos el TIMER1
  TIMER1->regs.gen->CR1   = 0x0001;                                             // Se habilita el contador

  timer_attach_interrupt(TIMER1, TIMER_UPDATE_INTERRUPT, Timer1UpdateInt);      // Se declara la rutina de interrupción para el evento update del Timer1
  timer_attach_interrupt(TIMER1, TIMER_CC1_INTERRUPT, Timer1InputCaptureInt);   // Se declara la rutina de interrupción para el evento input capture del Timer1 en el canal1

  return;
}
