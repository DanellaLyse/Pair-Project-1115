"""
Raspberry Pi Pico – Two-Board PWM Monitor (easy version)

Goal:
Make one Pico create a PWM signal (like a blinking light but really fast), and have another Pico read that signal through a simple RC circuit to check if the PWM is doing what it’s supposed to.

Setup:
1. **TX Pico (Sender)**
   - Sends a PWM signal on pin GP16.
   - Talks to the other Pico using UART1 (serial) on pins GP8 (TX) and GP9 (RX).
   - Sends what duty cycle it is trying to make (like 25%, 50%, etc.).

2. **RX Pico (Receiver)**
   - Reads the smoothed (averaged) PWM signal using an analog pin (ADC0 on GP26).
   - Measures how close it is to the expected value.
   - Sends that number back to the TX Pico so both can compare results.

3. **RC Filter**
   - Connect PWM output → resistor (10kΩ) → capacitor (10µF) → ground.
   - Connect the midpoint (between resistor and capacitor) to ADC0 on the RX Pico.
   - This smooths out the PWM into a voltage between 0 and 3.3V.

4. **Connections Between Boards**
   - GP8 (TX1) of one Pico connects to GP9 (RX1) of the other Pico, and vice versa.
   - Connect both GNDs together.

What it does:
- The TX Pico changes the PWM duty cycle slowly (0%, 5%, 10%, etc.).
- It tells the RX Pico what duty cycle it’s sending.
- The RX Pico measures the actual average voltage and converts it back to %.
- RX sends the measured value back to TX.
- Both Picos print the results and difference to the USB console.

How to use:

- Flash MicroPython on both Picos.
- Copy this file to both boards.
- On one board, change `ROLE = "TX"`.
- On the other, change `ROLE = "RX"`.
- Open both serial consoles in Thonny or PuTTY to see results.

License: Free to use for school projects. Made for learning.
"""

# ==== SETTINGS ====
ROLE = "TX"  # Change to "RX" for the second Pico

PWM_OUT_PIN = 16     # PWM output pin
PWM_FREQ_HZ = 1000   # Frequency of PWM (1 kHz)
ADC_IN_PIN = 26      # Analog input pin (ADC0)
VREF = 3.3           # Reference voltage for Pico ADC

UART_ID = 1          # UART1
UART_BAUD = 115200
UART_TX_PIN = 8
UART_RX_PIN = 9

# TX duty cycle settings
TX_SWEEP = True      # If True, sweep from 0 to 100%
TX_FIXED_DUTY = 50.0 # Used only if TX_SWEEP = False
TX_STEP = 5.0        # Step size
TX_DWELL_MS = 800    # How long to wait before next step

from machine import Pin, PWM, ADC, UART
import time

# Helper functions
def clamp(x, low, high):
    return max(low, min(high, x))

# PWM setup class
class PwmSource:
    def __init__(self, pin, freq):
        self.pwm = PWM(Pin(pin))
        self.pwm.freq(freq)
    def set_duty(self, percent):
        percent = clamp(percent, 0, 100)
        self.pwm.duty_u16(int(percent / 100 * 65535))

# ADC reading class
class AdcReader:
    def __init__(self, pin, vref=3.3):
        self.adc = ADC(Pin(pin))
        self.vref = vref
    def read_percent(self):
        value = self.adc.read_u16() / 65535 * 100
        return clamp(value, 0, 100)

# UART link class
class SerialLink:
    def __init__(self, uart_id, baud, tx_pin, rx_pin):
        self.uart = UART(uart_id, baudrate=baud, tx=Pin(tx_pin), rx=Pin(rx_pin))
    def send(self, tag, value):
        msg = f"{tag},{value}\n".encode()
        self.uart.write(msg)
    def read(self):
        if self.uart.any():
            line = self.uart.readline()
            if line:
                try:
                    tag, value = line.decode().strip().split(',')
                    return tag, float(value)
                except:
                    return None
        return None

# TX behavior
def run_tx():
    print("Running TX (sender)...")
    pwm = PwmSource(PWM_OUT_PIN, PWM_FREQ_HZ)
    link = SerialLink(UART_ID, UART_BAUD, UART_TX_PIN, UART_RX_PIN)

    duty = TX_FIXED_DUTY
    step = TX_STEP if TX_SWEEP else 0
    pwm.set_duty(duty)
    link.send("SET", duty)

    last = time.ticks_ms()
    while True:
        now = time.ticks_ms()
        if TX_SWEEP and time.ticks_diff(now, last) > TX_DWELL_MS:
            duty += step
            if duty > 100 or duty < 0:
                step = -step
                duty = clamp(duty, 0, 100)
            pwm.set_duty(duty)
            link.send("SET", duty)
            print(f"[TX] Sent duty: {duty:.1f}%")
            last = now

        msg = link.read()
        if msg and msg[0] == "MEAS":
            meas = msg[1]
            print(f"[TX] Measured: {meas:.1f}% | Error: {meas - duty:.1f}%")
        time.sleep(0.1)

# RX behavior
def run_rx():
    print("Running RX (receiver)...")
    adc = AdcReader(ADC_IN_PIN, VREF)
    link = SerialLink(UART_ID, UART_BAUD, UART_TX_PIN, UART_RX_PIN)

    while True:
        msg = link.read()
        if msg and msg[0] == "SET":
            duty = msg[1]
            meas = adc.read_percent()
            link.send("MEAS", meas)
            print(f"[RX] Sent back: {meas:.1f}% (target {duty:.1f}%)")
        time.sleep(0.1)

# Main
if ROLE == "TX":
    run_tx()
else:
    run_rx()
