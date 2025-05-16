#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"
#include <math.h>

#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define STEPS_PER_ROTATION 32 // number of steps the motor takes for a full 360-degree rotation
uint16_t distanceBuffer[100000]; // array to store distance measurements from the sensor

#define MAXRETRIES              5           // number of receive attempts before giving up

void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;                                                                                      // activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;                                                                              // activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};                                                                                                       // ready?

  GPIO_PORTB_AFSEL_R |= 0x0C;                                                                                                                 // 3) enable alt funct on PB2,3       0b00001100
  GPIO_PORTB_ODR_R |= 0x08;                                                                                                                   // 4) enable open drain on PB3 only

  GPIO_PORTB_DEN_R |= 0x0C;                                                                                                                   // 5) enable digital I/O on PB2,3
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
  I2C0_MCR_R = I2C_MCR_MFE;                                                                                                 // 9) master function enable
  I2C0_MTPR_R = 0b0000000000000101000000000111011;                          // 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
}

void PortG_Init(void){
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
  while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
  GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0
  return;
}

void PortH_Init(void){
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;                                      // activate clock for Port H
  while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){};                 // allow time for clock to stabilize
  GPIO_PORTH_DIR_R |= 0x0F;                                                                       // configure Port H pins (PH0-PH3) as output
  GPIO_PORTH_AFSEL_R &= ~0x0F;                                                                  // disable alt funct on Port H pins (PH0-PH3)
  GPIO_PORTH_DEN_R |= 0x0F;                                                                     // enable digital I/O on Port H pins (PH0-PH3)
  GPIO_PORTH_AMSEL_R &= ~0x0F;                                                                  // disable analog functionality on Port H pins (PH0-PH3)    
  return;
}

void PortN_Init(void) {
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;
  while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R12) == 0){};
  GPIO_PORTN_DIR_R = 0b00000011;                                                                                      // PN1 and PN0 as outputs
  GPIO_PORTN_DEN_R = 0b00000011;                                                                                      // D1 is PN1 and D2 is PN0
  return;
}

void PortF_Init(void) {
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
  while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R5) == 0){};
  GPIO_PORTF_DIR_R = 0b00010001;                                                                                      // PF4 and PF0 as outputs
  GPIO_PORTF_DEN_R = 0b00010001;                                                                                      // D3 is PF4 and D4 is PF0
  return;
}

void PortJ_Init(void){
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;                                                  // Activate clock for Port J
  while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};                             // Allow time for clock to stabilize
  GPIO_PORTJ_DIR_R = 0b00000000;                                                                      // PJ1 and PJ0 as input
  GPIO_PORTJ_DEN_R = 0b00000011;                                                                // enable digital I/O on PJ1 and PJ0
  GPIO_PORTJ_PCTL_R &= ~0x000000FF;                                                                           // configure PJ1 and PJ0 as GPIO
  GPIO_PORTJ_AMSEL_R &= ~0b00000011;                                            // Disable analog functionality on PJ1 and PJ0        
  GPIO_PORTJ_PUR_R |= 0b00000011;                                                           // Enable weak pull up resistor on PJ1 and PJ0
  return;
}

void VL53L1X_XSHUT(void){
  GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
  GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
  FlashAllLEDs();
  SysTick_Wait10ms(10);
  GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
}

// Rotate the stepper motor one complete step in the specified direction (CW or CCW)
void RotateMotorOneSweep(int rotateCW) {
  uint32_t delayTime = 1;              // short delay between motor coil activations
  int stepsToTurn = 16;                // how many incremental steps to perform
  for (int s = 0; s < stepsToTurn; s++) {
    if (rotateCW) {
      GPIO_PORTH_DATA_R = 0b00000011;
      SysTick_Wait10ms(delayTime);
      GPIO_PORTH_DATA_R = 0b00000110;
      SysTick_Wait10ms(delayTime);
      GPIO_PORTH_DATA_R = 0b00001100;
      SysTick_Wait10ms(delayTime);
      GPIO_PORTH_DATA_R = 0b00001001;
      SysTick_Wait10ms(delayTime);
    } else {
      GPIO_PORTH_DATA_R = 0b00001001;
      SysTick_Wait10ms(delayTime);
      GPIO_PORTH_DATA_R = 0b00001100;
      SysTick_Wait10ms(delayTime);
      GPIO_PORTH_DATA_R = 0b00000110;
      SysTick_Wait10ms(delayTime);
      GPIO_PORTH_DATA_R = 0b00000011;
      SysTick_Wait10ms(delayTime);
    }
  }
}

uint16_t dev = 0x29;             //address of the ToF sensor as an I2C slave peripheral
int status = 0;

int main(void) {
  uint8_t byteData, sensorReady = 0, rawBytes[10] = {0xFF};
  uint16_t wordData;
  uint8_t rangeStatus;
  uint8_t tofDataReady;

  int totalLayers = 3;                  // how many times we scan vertically
  int rotateCW = 1;                     // flag to track motor rotation direction

  // initialize MCU peripherals and ports
  PLL_Init();
  SysTick_Init();
  onboardLEDs_Init();
  I2C_Init();
  UART_Init();
  PortH_Init();
  PortJ_Init();
  PortN_Init();

  // verify that ToF sensor is detected
  status = VL53L1X_GetSensorId(dev, &wordData);

  // wait until the sensor is fully booted
  while(sensorReady == 0){
    status = VL53L1X_BootState(dev, &sensorReady);
    SysTick_Wait10ms(10);
  }

  FlashAllLEDs();
  status = VL53L1X_ClearInterrupt(dev);
  status = VL53L1X_SensorInit(dev);
  Status_Check("SensorInit", status);
  status = VL53L1X_StartRanging(dev);

  while(1){
    // begin scan only when pushbutton is pressed
    if ((GPIO_PORTJ_DATA_R & 0b00000001) == 0) {
      while ((GPIO_PORTJ_DATA_R & 0b00000001) == 0){};  // wait for button release

      FlashLED1(1);  // light up to indicate scan started

      // perform scanning over multiple layers
      for (int layer = 0; layer < totalLayers; layer++) {
        UART_printf("A\n");     // notify PC to begin data capture
        FlashLED4(1);            // flash to show UART send

        for (int step = 0; step < STEPS_PER_ROTATION; step++) {
          tofDataReady = 0;
          while (tofDataReady == 0) {
            status = VL53L1X_CheckForDataReady(dev, &tofDataReady);
            VL53L1_WaitMs(dev, 4);
          }

          // store distance for current angle
          status = VL53L1X_GetDistance(dev, &distanceBuffer[step]);
          VL53L1X_ClearInterrupt(dev);

          // send index and distance over UART
          sprintf(printf_buffer, "%d,%d\n", step, distanceBuffer[step]);
          UART_printf(printf_buffer);

          FlashLED3(1);            // measurement indicator LED
          RotateMotorOneSweep(rotateCW);  // advance motor to next position
        }

        // re-initialize ranging for next layer
        VL53L1X_StopRanging(dev);
        VL53L1X_ClearInterrupt(dev);
        VL53L1X_StartRanging(dev);
        rotateCW = !rotateCW;              // alternate direction for motor
        UART_printf("scan_done\r\n");     // indicate layer is complete
        SysTick_Wait10ms(50);
      }
    }
  }
}
