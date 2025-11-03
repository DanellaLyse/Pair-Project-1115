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
"""
Raspberry Pi Pico – Two-Board PWM Monitor (Full Duplex Version)
Both Picos send and receive at the same time over UART1 (GP8↔GP9).
"""

# ==== SETTINGS ====
ROLE = "TX"  # Change to "RX" for the second Pico

PWM_OUT_PIN = 16     # PWM output pin
PWM_FREQ_HZ = 1000   # PWM frequency (1 kHz)
ADC_IN_PIN = 26      # Analog input pin (ADC0)
VREF = 3.3           # Reference voltage

UART_ID = 1
UART_BAUD = 115200
UART_TX_PIN = 8
UART_RX_PIN = 9

TX_SWEEP = True
TX_FIXED_DUTY = 50.0
TX_STEP = 5.0
TX_DWELL_MS = 800

from machine import Pin, PWM, ADC, UART
import time

def clamp(x, low, high):
    return max(low, min(high, x))

# === PWM setup ===
class PwmSource:
    def __init__(self, pin, freq):
        self.pwm = PWM(Pin(pin))
        self.pwm.freq(freq)
    def set_duty(self, percent):
        percent = clamp(percent, 0, 100)
        self.pwm.duty_u16(int(percent / 100 * 65535))

# === ADC Reader ===
class AdcReader:
    def __init__(self, pin, vref=3.3):
        self.adc = ADC(Pin(pin))
        self.vref = vref
    def read_percent(self):
        val = self.adc.read_u16() / 65535 * 100
        return clamp(val, 0, 100)

# === UART Link ===
class SerialLink:
    def __init__(self, uart_id, baud, tx_pin, rx_pin):
        self.u = UART(uart_id, baudrate=baud, tx=Pin(tx_pin), rx=Pin(rx_pin))
    def send(self, tag, value):
        msg = f"{tag},{value}\n".encode()
        self.u.write(msg)
    def read(self):
        if self.u.any():
            line = self.u.readline()
            if line:
                try:
                    tag, value = line.decode().strip().split(',')
                    return tag, float(value)
                except:
                    pass
        return None

# === Main TX/RX Hybrid Function ===
def run_full_duplex():
    pwm = PwmSource(PWM_OUT_PIN, PWM_FREQ_HZ)
    adc = AdcReader(ADC_IN_PIN, VREF)
    link = SerialLink(UART_ID, UART_BAUD, UART_TX_PIN, UART_RX_PIN)
    led = Pin(25, Pin.OUT)

    duty = TX_FIXED_DUTY
    step = TX_STEP if TX_SWEEP else 0
    last_send = time.ticks_ms()

    print(f"Running {ROLE} (Full Duplex) on UART1 GP8/GP9…")

    while True:
        now = time.ticks_ms()

        # === Each side sends its own message every TX_DWELL_MS ===
        if time.ticks_diff(now, last_send) > TX_DWELL_MS:
            if ROLE == "TX":
                pwm.set_duty(duty)
                link.send("SET", duty)
                print(f"[TX] Sent duty: {duty:.1f}%")
                duty += step
                if duty > 100 or duty < 0:
                    step = -step
                    duty = clamp(duty, 0, 100)
            else:
                meas = adc.read_percent()
                link.send("MEAS", meas)
                print(f"[RX] Sent measured: {meas:.1f}%")
            led.toggle()
            last_send = now

        # === Both continuously listen for messages ===
        msg = link.read()
        if msg:
            tag, val = msg
            if ROLE == "TX" and tag == "MEAS":
                print(f"[TX] Received MEAS: {val:.1f}% | Error: {val - duty:.1f}%")
            elif ROLE == "RX" and tag == "SET":
                print(f"[RX] Received SET: {val:.1f}% (target duty)")
            else:
                print(f"[{ROLE}] Received {tag}: {val:.1f}%")

        time.sleep(0.05)

# === Main ===
run_full_duplex()
