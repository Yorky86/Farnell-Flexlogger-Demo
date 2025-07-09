from picozero import pico_led
from machine import Pin, ADC, PWM, UART
from time import sleep
import sys

analogPumpSP  = ADC(1)
analogValveSP = ADC(0)
board_v       = ADC(3)
temp_sensor   = ADC(4)

dout_Running   = Pin(18, Pin.OUT)
din_Run        = Pin(19, Pin.IN, Pin.PULL_DOWN)
pwm_PumpOut    = PWM(Pin(21, Pin.OUT), freq=100)
pwm_ValveOut   = PWM(Pin(20, Pin.OUT), freq=100)
dout_Heartbeat = Pin(22, Pin.OUT)
led_Onboard    = Pin(25, Pin.OUT)

Vmax = 3.3
doutFlag = 0



uart = UART(0)
uart.init(baudrate=9600, bits=8, parity=None, stop=1, tx=Pin(16), rx=Pin(17))

def read_int_temp():
    adc_value = temp_sensor.read_u16()
    voltage = adc_value * (3.3 / 65535.0)
    temp_celcius = 27 - (voltage - 0.706) / 0.001721
    return temp_celcius

def read_board_voltage():
    adc_value = board_v.read_u16()
    voltage = adc_value * (Vmax / 65535.0)
    return voltage

def read_pump_setpoint():
    adc_value = analogPumpSP.read_u16()
    setpoint = adc_value * (Vmax / 65535.0)
    return setpoint

def read_valve_setpoint():
    adc_value = analogValveSP.read_u16()
    setpoint = adc_value * (Vmax / 65535.0)
    return setpoint



while(1):
    #print(dinRun.value())
    print("\n")
    print("\n")
    print("\n")
    print("\n")
    temperature = read_int_temp()
    board_V = read_board_voltage()
    if dout_Heartbeat.value() == 1:
        doutFlag = 0
    else:
        doutFlag = 1
        
    dout_Heartbeat.value(doutFlag)
    led_Onboard.value(doutFlag)
    
    if din_Run.value():
        dout_Running.value(doutFlag)
        
        pumpSetpoint = int((read_pump_setpoint()/Vmax)*65535)
        valveSetpoint = int((read_valve_setpoint()/Vmax)*65535)
        
    else:
        #print("dinRun is False")
        pumpSetpoint = 0
        valveSetpoint = 0
        dout_Running.value(0)

    pwm_PumpOut.duty_u16(pumpSetpoint) 
    pwm_ValveOut.duty_u16(valveSetpoint)   
    
    write_buffer = f"Board temperature: {temperature}"
    uart.write(write_buffer)
    print(write_buffer)
        
    write_buffer  = f"Board voltage: {board_V}"
    uart.write(write_buffer)
    print(write_buffer)
        
    write_buffer = f"Pump Setpoint: {(pumpSetpoint/65535)*Vmax}" 
    uart.write(write_buffer)
    print(write_buffer)
        
    write_buffer = f"Valve Setpoint: {(valveSetpoint/65535)*Vmax}" 
    uart.write(write_buffer)
    print(write_buffer)
    
    sleep(0.5)

