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
#define DB4 	      PB4 														                            // Pin 11 pantalla, cable violeta

#define E1		      PA7															                            // Pin 1 encoder/pulsador
#define E2          PA6															                            // Pin 2 encoder/pulsador
#define EP          PA5															                            // Pin pulsador encoder/pulsador

#define VIN         PA0                                                         // Entrada tensión de alimentación
#define OvenStatus  PA1                                                         // Entrada Oven Status

#define OutEN       PB0                                                         // Control de la señal de salida

#define LED         PC13                                                        // LED pata PC13
//

//
#define Memoria   (uint16_t*)0x1FFFF7E0                                         // Dirección de memeoria que contiene la cantidad de flash del micro dependiendo del modelo
//

// Variables y constantes
uint16_t Timer1MSB;                                                             // Para almacenar las veces que se desborda el TIMER3 y poder contar los pulsos totales contados entre periodos
uint16_t CaptureNew1;
uint16_t CaptureAnt1;
uint32_t CuentaTotal1;
double FrecEstimada1, FrecRequerida1;
double Pwm;
double Dif1;
double KF1;
uint16_t MaxDes1;
bool PrimerValor1;
bool DatoNuevo1;
uint8_t Contraste;
bool E1_Ant;
bool E2_Ant;
bool EP_Ant;
//

LiquidCrystal2 LCD(RS, EN, DB4, DB5, DB6, DB7);

void setup()
{
  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);                                     // Se deshabilitan los pines JTAG, pero se permiten los pines para ST-LINK
  Cambiar_CPU_CLK();                                                            // Cambiamos el reloj del sistema a 100Mhz con cristal externo de 10Mhz
  
  Timer1MSB = 0;
  CaptureNew1 = 0,
  CaptureAnt1 = 0;
  CuentaTotal1 = 0;
  FrecEstimada1 = FrecRequerida1 = 100000000.0;                                 // Frecuencia estimada del PLL en Hz (10Mhz * 10 = 100Mhz)
  Dif1 = 0.0;
  MaxDes1 = 150;                                                                // Maxima desviación permitida para tracking del filtro en Hz * 10 sobre 10Mhz
                                                                                // 1ppm = 1Hz en 1Mhz -> 10Hz en 10Mhz -> 100Hz en 100Mhz
                                                                                // +-250ppb el STP2145A, +-1,5ppm el CTI
                                                                                // +-1,5ppm = +-150Hz en 100Mhz
                                                                                // +-0,25ppm = +- 25Hz en 100Mhz
  KF1 = 1 / MaxDes1;
  PrimerValor1 = false;
  DatoNuevo1 = false;
  Pwm = 32768.0;                                                                // Iniciamos el PWM al 50%
  Contraste = EEPROM.read(0);
  E1_Ant = true;
  E2_Ant = true;
  EP_Ant = true;

  pinMode(VIN, INPUT);
  pinMode(OvenStatus, INPUT_PULLUP);
  pinMode(OutEN, OUTPUT);

  digitalWrite(OutEN, HIGH);                                                    // Se activa la señal de 10Mhz de salida

  pinMode(E1, INPUT_PULLUP);
  pinMode(E2, INPUT_PULLUP);
  pinMode(EP, INPUT_PULLUP);
  pinMode(LED, OUTPUT);
  
  digitalWrite(LED, HIGH);

  LCD.begin(16, 2);
  delay(100);
  LCD.print("EA5IOT");
  LCD.setCursor(0, 1);
  LCD.print("GPSDO");

  Serial2.begin(82944);                                                         // PC serie, equivale a 115200 baudios, ya que el entorno se cree que tiene un reloj de 72Mhz
  Serial3.begin(3456);                                                          // GPS serie, equivale a 4800 baudios, ya que el entorno se cree que tiene un reloj de 72Mhz 
  
  IniciarContador();
  CambiarContraste();
}

void loop()
{
  ComprobarBotones();
  if (DatoNuevo1)
  {
    digitalWrite(LED, !digitalRead(LED));
    LCD.setCursor(0, 0);
    LCD.print("        ");
    LCD.setCursor(0, 0);
    LCD.print(Dif1 * 10.0, 1);
    LCD.setCursor(0, 1);
    LCD.print("        ");
    LCD.setCursor(0, 1);
    LCD.print(uint16_t(Pwm));

    DatoNuevo1 = false;
  };
}

void Timer1UpdateInt(void)
{
  Timer1MSB++;                                                                  // Se cuentan los overflows del contador
  TIMER1->regs.gen->SR &= 0x1EFE;                                               // Se resetea la bandera de update
  
  return;
}

void Timer1InputCapture1(void)
{
  uint16_t temp;
  double signo;
    
  CaptureNew1 = TIMER1->regs.gen->CCR1;                                         // Se almacena el valor del contador cuando se produce el pulso de 1pps
  TIMER1->regs.gen->SR &= 0x1EFD;                                               // Se resetea la bandera de input capture canal1
  temp = Timer1MSB;
  Timer1MSB = 0;                                                                // Se inicia el contador de desbordamientos del contador principal
      
  if (PrimerValor1 == false)
  {
    CaptureAnt1 = CaptureNew1;
    PrimerValor1 = true;
  } else
  {
    CuentaTotal1 = (65536 * (temp - 1)) + (65536 - CaptureAnt1) + CaptureNew1;  // Se calculan los pulsos totales contados
    CaptureAnt1 = CaptureNew1;

    if ((CuentaTotal1 > (FrecRequerida1 - MaxDes1)) && (CuentaTotal1 < (FrecRequerida1 + MaxDes1)))  // Se comprueba que no se haya leido mal la frecuencia del Xtal
    {
      if (Dif1 != 0.0)
      {
        FrecEstimada1 += KF1 * Dif1 * (CuentaTotal1 - FrecEstimada1);           // Se estima la Frecuencia actual del XTAL, a mayor error mayor velocidad de control
      } else
      {
        FrecEstimada1 += KF1 * (CuentaTotal1 - FrecEstimada1);                  // Se estima la Frecuencia actual del XTAL
      };

      Dif1 = (FrecRequerida1 - FrecEstimada1);

      if (Dif1 >= 0.0) signo = 1.0; else signo = -1.0;

      Pwm += Dif1 * (Dif1 * 5.0) * signo;                                       // Se varía el incremento del PWM en función de la cantidad de error, a mas error mayor variación
      
      if (Pwm > 65535.0) Pwm = 65535.0;
      if (Pwm < 0.0) Pwm = 0.0;
      TIMER1->regs.gen->CCR2 = uint16_t(Pwm);                                   // Se actualiza el valor del PWM para controlar la frecuencia del oscilador
    
      DatoNuevo1 = true;
    };
  };

  return;
}

void Timer1OutputCompare2(void)
{
  TIMER1->regs.gen->SR &= 0x1EFB;                                               // Se resetea la bandera de output compare canal2

  return;
}

void ComprobarBotones(void)
{
  bool tempE1 = digitalRead(E1);
  bool tempE2 = digitalRead(E2);
  bool tempEP = digitalRead(EP);
  
  if ((E1_Ant == false) && (tempE1 == true))                                    // ¿Hay un flanco ascendente en el pulsador E1?
  {
    if (Contraste < 255) Contraste++;
    CambiarContraste();
  };

  if ((E2_Ant == false) && (tempE2 == true))                                    // ¿Hay un flanco ascendente en el pulsador E2?
  {
    if (Contraste > 0) Contraste--;
    CambiarContraste();
  };

  if ((EP_Ant == false) && (tempEP == true))                                    // ¿Hay un flanco ascendente en el pulsador EP?
  {
    EEPROM.write(0, Contraste);
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
  //GPIOA->regs->CRH |= 0x000000C0;                                             // Se hace un or para poner los unos, alternate function open drain PA9 para PWM de 5V    
  GPIOA->regs->CRH   |= 0x00000080;                                             // Se hace un or para poner los unos, alternate function push-pull PA9 para PWM de 3,3V

  // PWM en CH3
  TIMER1->regs.gen->CCMR2 |= 0x0068;                                            // Output Compare CH3
  TIMER1->regs.gen->CCER  |= 0x0100;                                            // CH3 Output Compare ON - Polarity HIGH
  pinMode(Timer1Ch3, PWM);                                                      // No hay manera de acceder al registro BDTR para activar el OUTPUT COMPARE, asi hace lo mismo
  TIMER1->regs.gen->CCR3  = 0x8000;                                             // PWM al 50% 

  // Habilitamos el TIMER1
  TIMER1->regs.gen->CR1   = 0x0001;                                             // Se habilita el contador

  timer_attach_interrupt(TIMER1, TIMER_UPDATE_INTERRUPT, Timer1UpdateInt);      // Se declara la rutina de interrupción para el evento update del Timer1
  timer_attach_interrupt(TIMER1, TIMER_CC1_INTERRUPT, Timer1InputCapture1);     // Se declara la rutina de interrupción para el evento input capture del Timer1 en el canal1
  timer_attach_interrupt(TIMER1, TIMER_CC2_INTERRUPT, Timer1OutputCompare2);    // Se declara la rutina de interrupción para el evento output compare del Timer1 en el canal2

  return;
}

