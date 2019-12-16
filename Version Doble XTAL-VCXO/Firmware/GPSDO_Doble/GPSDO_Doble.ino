// EA5IOT GPSDO DOBLE SMT32f103C8T6 XTAL FREQUENCY PWM CONTROL

/*
    Copyright (C) 2020

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

// PATAS Microcontrolador
#define Timer3Ch1  PB4                                                          // Señal directa del cristal_1
#define Timer3Ch2  PB5                                                          // Señal de 1PPS del gps (Activa cuando el GPS está enganchado)
#define Timer4Ch1  PB6                                                          // Señal directa del cristal_2
#define Timer4Ch2  PB7                                                          // Señal de 1PPS del gps (Activa cuando el GPS está enganchado)
#define Timer2Ch3  PB10                                                         // Salida PWM Control de Frecuencia1
#define Timer2Ch4  PB11                                                         // Salida PWM Control de Frecuencia2
#define LED        PC13                                                         // LED pata PC13
#define Apagado    HIGH
#define Encendido  LOW
//

// Variables
double FrecEstimada1, FrecRequerida1;
uint16_t Timer3MSB;                                                             // Para almacenar las veces que se desborda el TIMER3 y poder contar los pulsos totales contados entre periodos
uint16_t CaptureNew1;
uint16_t CaptureAnt1;
uint32_t CuentaTotal1;
double Dif1;
double Enganche1;
bool Disciplinado1;
double MaxDes1;
bool DatoNuevo1;
bool PrimerValor1;
double Pwm1;

double FrecEstimada2, FrecRequerida2;
uint16_t Timer4MSB;                                                             // Para almacenar las veces que se desborda el TIMER4 y poder contar los pulsos totales contados entre periodos
uint16_t CaptureNew2;
uint16_t CaptureAnt2;
uint32_t CuentaTotal2;
double Dif2;
double Enganche2;
bool Disciplinado2;
double MaxDes2;
bool PrimerValor2;
bool DatoNuevo2;
double Pwm2;

uint8_t ContadorLed;
//

void setup()
{
  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);                                     // Se deshabilitan los pines JTAG, pero se permiten los pines para ST-LINK
  Serial.begin(115200);

  Timer3MSB = 0;
  CaptureNew1 = 0,
  CaptureAnt1 = 0;
  CuentaTotal2 = 0;
  FrecEstimada1 = FrecRequerida1 = 28000000.0;                                  // Frecuencia estimada del Xtal en Hz
  Dif1 = 0.0;
  Enganche1 = 10.0;                                                             // Si el error de la frecuencia está por debajo de este valor se activa el LED para decir que el oscilador está disciplinado
  MaxDes1 = 10000.0;                                                            // Maxima desviación permitida para tracking del filtro en Hz
  PrimerValor1 = false;
  DatoNuevo1 = false;
  Pwm1 = 32768.0;
  Disciplinado1 = false;
    
  Timer4MSB = 0;
  CaptureNew2 = 0;
  CaptureAnt2 = 0;
  CuentaTotal2 = 0;
  FrecEstimada2 = FrecRequerida2 = 25000000.0;                                  // Frecuencia estimada del Xta2 en Hz
  Dif2 = 0.0;
  Enganche2 = 10.0;                                                             // Si el error de la frecuencia está por debajo de este valor se activa el LED para decir que el oscilador está disciplinado
  MaxDes2 = 10000.0;                                                            // Maxima desviación permitida para tracking del filtro en Hz
  PrimerValor2 = false;
  DatoNuevo2 = false;
  Pwm2 = 32768.0;
  Disciplinado2 = false;

  ContadorLed = 0;

  pinMode(Timer2Ch3, OUTPUT);
  pinMode(Timer2Ch4, OUTPUT);
  pinMode(Timer3Ch1, INPUT);
  pinMode(Timer3Ch2, INPUT);
  pinMode(Timer4Ch1, INPUT);
  pinMode(Timer4Ch2, INPUT);
  pinMode(LED, OUTPUT);

  IniciarPWM();
  IniciarContadores();
}

void loop()
{
  bool estado;
  unsigned long tiempo;
  
  if (DatoNuevo1)
  {
    if (Serial)
    {
      Serial.print("F1: ");
      Serial.print(CuentaTotal1);
      Serial.print(" E1: ");
      Serial.print(" ");
      Serial.println(FrecEstimada1, 3);
    };
    DatoNuevo1 = false;
    ContadorLed++;
  };
  
  if (DatoNuevo2)
  {
    if (Serial)
    {
      Serial.print("F2: ");
      Serial.print(CuentaTotal2);
      Serial.print(" E2: ");
      Serial.print(" ");
      Serial.println(FrecEstimada2, 3);
    };
    DatoNuevo2 = false;
    ContadorLed++;
  };

  if (ContadorLed > 1)
  {
    digitalWrite(LED, !digitalRead(LED));
    ContadorLed = 0;
  };

  estado = digitalRead(LED);
  
  if (Disciplinado1 = true)
  {
    digitalWrite(LED, Apagado);
    tiempo = millis();
    while((millis() - tiempo) <= 50) {};                                        // Esperamos 50ms
    digitalWrite(LED, Encendido);
    tiempo = millis();
    while((millis() - tiempo) <= 50) {};                                        // Esperamos 50ms
    digitalWrite(LED, Apagado);
    tiempo = millis();
    while((millis() - tiempo) <= 50) {};                                        // Esperamos 50ms    
    Disciplinado1 = false;   
  };
  
  if (Disciplinado2 = true)
  {
    digitalWrite(LED, Apagado);
    tiempo = millis();
    while((millis() - tiempo) <= 50) {};                                        // Esperamos 50ms
    digitalWrite(LED, Encendido);
    tiempo = millis();
    while((millis() - tiempo) <= 50) {};                                        // Esperamos 50ms
    digitalWrite(LED, Apagado);
    tiempo = millis();
    while((millis() - tiempo) <= 50) {};                                        // Esperamos 50ms
    digitalWrite(LED, Encendido);
    tiempo = millis();
    while((millis() - tiempo) <= 50) {};                                        // Esperamos 50ms
    digitalWrite(LED, Apagado);
    tiempo = millis();
    while((millis() - tiempo) <= 50) {};                                        // Esperamos 50ms     
    Disciplinado2 = false;
  };

  digitalWrite(LED, estado);
}

void IniciarPWM(void)
{
  //GPIOB->regs->CRH |= 0xFF00;                                                 // alternate function open drain PB10 y PB11 para poder generar un PWM de 5V
  GPIOB->regs->CRH |= 0xBB00;                                                   // alternate function push-pull PB10 y PB11 PWM de 3V
  
  TIMER2->regs.gen->CR1  = 0x0000;                                              // Se deshabilita el contador para programar los registros

  TIMER2->regs.gen->CCMR2 = 0x6868;                                             // Output Compare CH3 y CH4 (110 PWM_MODE_1)
  TIMER2->regs.gen->CCER  = 0x1100;                                             // CH4 Output Compare ON, CH3 Output Compare ON - Polarity HIGH

  TIMER2->regs.gen->CCR3 = 0x8000;                                              // PWM al 50%
  TIMER2->regs.gen->CCR4 = 0x8000;                                              // PWM al 50%
  
  TIMER2->regs.gen->CR1  = 0x0001;                                              // Se habilita el contador para que genere el PWM de Frecuencia1 y Frecuencia2
  
  return;
}

void IniciarContadores(void)                                                    // Se cuenta en el flanco ascendente del CLK y se captura el resto en el Comparador1 entrada2 --> Ve bien los 30Mhz
{
  AFIO_BASE->MAPR |= 0x0A00;                                                    // 10->Timer3 PB4 CH1 y PB5 CH2          10->Timer2 PB10 CH3 y PB11 CH4

  TIMER3->regs.gen->CR1   = 0x0000;                                             // Se deshabilita el contador
  TIMER3->regs.gen->CCMR1 = 0x0002;                                             // Input Capture Channel 1 mapped to TI2
  TIMER3->regs.gen->CCER  = 0x0001;                                             // Input Capture Enable, Timer1_Channel1 rising edge on TI2 pin Pata para el 1PPS del GPS
  TIMER3->regs.gen->SMCR  = 0x0040;                                             // TI1 Edge Detector is the trigger, Timer1_Channel1 as TI1 pin Pata para el reloj externo
  TIMER3->regs.gen->SMCR |= 0x0007;                                             // Ext. clk mode 1
  TIMER3->regs.gen->DIER  = 0x0003;                                             // Se activan las interrupciones de Update e Input Capture
  TIMER3->regs.gen->CR1   = 0x0001;                                             // Se habilita el contador
  
  TIMER4->regs.gen->CR1   = 0x0000;                                             // Se deshabilita el contador
  TIMER4->regs.gen->CCMR1 = 0x0002;                                             // Input Capture Channel 1 mapped to TI2
  TIMER4->regs.gen->CCER  = 0x0001;                                             // Input Capture Enable, Timer1_Channel1 rising edge on TI2 pin Pata para el 1PPS del GPS
  TIMER4->regs.gen->SMCR  = 0x0040;                                             // TI1 Edge Detector is the trigger, Timer1_Channel1 as TI1 pin Pata para el reloj externo
  TIMER4->regs.gen->SMCR |= 0x0007;                                             // Ext. clk mode 1
  TIMER4->regs.gen->DIER  = 0x0003;                                             // Se activan las interrupciones de Update e Input Capture
  TIMER4->regs.gen->CR1   = 0x0001;                                             // Se habilita el contador

  timer_attach_interrupt(TIMER3, TIMER_UPDATE_INTERRUPT, Timer3UpdateInt);      // Se declara la rutina de interrupción para el evento update del Timer3
  timer_attach_interrupt(TIMER3, TIMER_CC1_INTERRUPT, Timer3InputCaptureInt);   // Se declara la rutina de interrupción para el evento input capture del Timer3
  
  timer_attach_interrupt(TIMER4, TIMER_UPDATE_INTERRUPT, Timer4UpdateInt);      // Se declara la rutina de interrupción para el evento update del Timer4
  timer_attach_interrupt(TIMER4, TIMER_CC1_INTERRUPT, Timer4InputCaptureInt);   // Se declara la rutina de interrupción para el evento input capture del Timer4  
  
  return;
}

void Timer3UpdateInt(void)
{
  Timer3MSB++;                                                                  // Se cuentan los overflows del contador
  TIMER3->regs.gen->SR &= 0x1EFE;                                               // Se resetea la bandera de update  

  return;
}

void Timer3InputCaptureInt(void)
{
  uint16_t temp;
  double k;

  CaptureNew1 = TIMER3->regs.gen->CCR1;                                         // Se almacena el valor del contador cuando se produce el pulso de 1pps
  TIMER3->regs.gen->SR &= 0x1EFD;                                               // Se resetea la bandera de input capture
  temp = Timer3MSB;
  Timer3MSB = 0;                                                                // Se inicia el contador de desbordamientos del contador principal

  if (PrimerValor1 == false)
  {
	  CaptureAnt1 = CaptureNew1;
	  PrimerValor1 = true;
  } else
  {
	  CuentaTotal1 = (65536 * (temp - 1)) + (65536 - CaptureAnt1) + CaptureNew1;  // Se calculan los pulsos totales contados
	  CaptureAnt1 = CaptureNew1;
	  if ((CuentaTotal1 > (FrecEstimada1 - MaxDes1)) && (CuentaTotal1 < (FrecEstimada1 + MaxDes1)))		// Se comprueba que no se haya leido mal la frecuencia del Xtal
	  {
		  k = abs(Dif1) * 0.0018;
      k += 0.05;                                                                // Valor minimo de k
		  if (k > 0.9) k = 0.9;                                                     // Valor maximo de k
		  FrecEstimada1 += k * (CuentaTotal1 - FrecEstimada1);              			  // Se estima la Frecuencia actual del XTAL
		  Dif1 = FrecRequerida1 - FrecEstimada1;                                    // Diferencia en Hz de la desviación de frecuencia del Xtal con respecto a la frecuencia requerida
		  Pwm1 += Dif1 * 50.0 * k;
		  if (Pwm1 > 65535.0) Pwm1 = 65535.0;
		  if (Pwm1 < 0.0) Pwm1 = 0.0;
      TIMER2->regs.gen->CCR3 = uint16_t(Pwm1);
      DatoNuevo1 = true;
      if (abs(Dif1) < Enganche1) Disciplinado1 = true;
	  };
  };

  return;
}

void Timer4UpdateInt(void)
{
  Timer4MSB++;                                                                  // Se cuentan los overflows del contador
  TIMER4->regs.gen->SR &= 0x1EFE;                                               // Se resetea la bandera de update  

  return;
}

void Timer4InputCaptureInt(void)
{
  uint16_t temp;
  double k;

  temp = Timer4MSB;
  Timer4MSB = 0;                                                                // Se inicia el contador de desbordamientos del contador principal
  CaptureNew2 = TIMER4->regs.gen->CCR1;                                         // Se almacena el valor del contador cuando se produce el pulso de 1pps
  TIMER4->regs.gen->SR &= 0x1EFD;                                               // Se resetea la bandera de input capture

  CuentaTotal2 = (65536 * (temp - 1)) + (65536 - CaptureAnt2) + CaptureNew2;    // Se calculan los pulsos totales contados
  CaptureAnt2 = CaptureNew2;

  if (PrimerValor2 == false)
  {
    CaptureAnt2 = CaptureNew2;
    PrimerValor2 = true;
  } else
  {
    if ((CuentaTotal2 > (FrecEstimada2 - MaxDes2)) && (CuentaTotal2 < (FrecEstimada2 + MaxDes2)))  	// Se comprueba que no se haya leido mal la frecuencia del Xtal
    {
		  k = abs(Dif2) * 0.0018;
      k += 0.05;                                                                // Valor minimo de k
		  if (k > 0.9) k = 0.9;                                                     // Valor maximo de k
		  FrecEstimada2 += 0.5 * (CuentaTotal2 - FrecEstimada2);              			// Se estima la Frecuencia actual del XTAL
		  Dif2 = FrecRequerida2 - FrecEstimada2;                                    // Diferencia en Hz de la desviación de frecuencia del Xtal con respecto a la frecuencia requerida
		  Pwm2 += Dif2 * 50.0 * k;
		  if (Pwm2 > 65535.0) Pwm2 = 65535.0;
		  if (Pwm2 < 0.0) Pwm2 = 0.0;
		  TIMER2->regs.gen->CCR4 = uint16_t(Pwm2);
		  DatoNuevo2 = true;
      if (abs(Dif2) < Enganche2) Disciplinado2 = true;
    };
  };

  return;
}
