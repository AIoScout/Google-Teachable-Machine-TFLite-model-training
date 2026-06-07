from machine import Pin, SDCard
import os, time

pwr = Pin(45, Pin.OUT)
pwr.value(0)      # 先试 0=ON
time.sleep_ms(200)