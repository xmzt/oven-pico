# oven-pico

Raspberry pi pico controlled oven.
2x16 character display.
TMP101 i2c temperature sensor.
AC phase detect.
TRIAC control.

## TRIAC control circuitry

TRI0    = BTA16-600B T1,T2,G (Q1=T2+,G+ Q2=T2+,G- Q3=T2-,G- Q4=T2-,G+) Igt=10ma,max,Q1-Q3 Vgt=0.7 Vt=1.22
OPTO_EN = LTV-816S optocoupler, photodarlington would be better to replace OPTO_EN and Q0.
Q0      = SS8050 NPN (or equivalent), in darlington with OPTO_EN to boost output drive to TRIAC gate.
R_G     = 270 ohm resistor, controls TRI0.G current. 15mA = (5.0V - 0.7V) / 270.
R_EN_AK = 1k ohm resistor, opto LED drive current (4mA)

(HOT) - TRI0.T1 TRI0.T2 - LOAD - (NEU)
TRI0.G - R_G - OPTO_EN.C OPTO_EN.E - Q0.B
OPTO_EN.C - Q0.C
Q0.E - (HOT_M5)

(UC_5) - R_EN_AK - OPTO_EN.A OPTO_EN.K - (UC_0)

## AC phase detect circuitry

(HOT)    = AC hot
(HOT_M5) = AC hot minus 5
(NEU)    = AC neutral
(UC_5)    = microcontroller side 5V supply (isolated from AC)
(UC_0)    = microcontroller side 0V supply (isolated from AC)
(UC_ACX)  = microcontroller AC cross signal (isolated from AC)

DREC      = 1N5406 (overkill) diode, AC rectifying
OPTO_EN   = LTV-816S optocoupler, receive enable from microcontroller, most anything should work.
OPTO_TX   = LTV-816S optocoupler, transmit to microcontroller, desire low current capability. 
Q0        = TK10A50D NMOS, Vds > AC peak, current limit on enable, high voltage blocking on disable.
R_ILIM    = 1k ohm reistor, sets current limit to about 2mA = (5V - Q0.Vgt=3.0V) / R_ILIM.
R_EN_E    = 10k ohm resistor, sets current in OPTO_EN, any higher and OPTO_EN turn-off is delayed.
R_Q0_G    = 470 ohm resistor, gate resistor, might not even need it.
R_ACX     = 20k ohm resistor, high enough to get signal to swing between 0V and 5V. Too high and turn-off delayed. 
R_EN_AK   = 1k ohm resistor, opto LED drive current (4mA)

(NEU)- DREC.A DREC.K - OPTO_TX.A OPTO_TX.K - Q0.D Q0.S - R_ILIM - (HOT_M5)
(HOT) - OPTO_EN.C OPTO_EN.E - R_EN_E - (HOT_M5)
OPTO_EN.E - R_Q0_G - Q0.G
(UC_5) - OPTO_TX.C OPTO_TX.E - (UC_ACX) - R_ACX - (UC_0)
(UC_5) - R_EN_AK - OPTO_EN.A OPTO_EN.K - (UC_0)

## AC phase detect algorithm

The UC_ACX signal functions like a comparator NEU_X of NEU and some value X a bit above HOT_M5,
hi whenn NEU > X.

Timestamp T0 = rising edge of NEU_X.
Timestamp T1 = next falling edge of NEU_X.
Timestamp T2 = next rising edge of NEU_X (one full cycle since T0).
Timestamp T3 = predicted next falling edge of NEU_X after T2.
Timestamp T4 = predicted next zero cross after T3.

Signal considered valid when T3 is within a specified tolerance of actual measured value. If
valid , alarm for T4 is set.

PERIOD    = T2 - T0
PEAK_HI   = (T0 + T1) / 2
PEAK_LO   = (T1 + T2) / 2
ZERO      = (PEAK_HI + PEAK_LO) / 2 = (T0/2 + T1/2 + T1/2 + T2/2) / 2 = ((T0 + T2)/2 + T1)/2
PERIOD    = T2 - T0
T3        = PERIOD + T1
T4        = PERIOD + ZERO

Pico does not have functionality like an Atmel chip PWM to record timestamp upon a pin changing
value. How to get most accurate timestamp of an edge transistion on the Pico?

A. set a GPIO interrupt, read TIME[HL]R or TIMERAW[HL], subject to interrupt uncertainty.

B. set a DMA transfer of same TIME registers on GPIO pin change, the use DMA interrupt. DMA
   uncertainty probably less than interrupt uncertainty.

C. PWM can count level hi. Using 2 synchronized PWM counters of cmoplementary signals, one can
   count hi period, then lo period. A third sync'd PWM would trigger TRIAC.

D. PIO. Decrement counter until pin change, mov counter into fifo, interrupt, repeat. Is there a
   way to sync this counter with alarm / PWM? Or another PIO routine to set pin on alarm using
   sync'd counter.

For the purposes of AC detect for TRIAC triggering, any of these should be plenty accurate, but
in theory, D, with no uncertainty, should be possible.

## Notes

xosc - 12 MHz
clk_ref: 12 MHz, xosc/1
watchdog: 1 MHz, clk_ref/12
systick: disabled
pll_sys: refdiv=1 fbdiv=7d postdiv1=6 postdiv2=2, vco = 1500 MHz = (12MHZ / 1) * 125, 125 MHz = vco / (6 * 2)

ac detect
tick (timer + pwm oc) for phase out


