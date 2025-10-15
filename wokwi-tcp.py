Import("env")
import os

# Configura Wokwi para usar Serial over TCP en localhost:4000
os.environ["WOKWI_SERIAL_PORT"] = "tcp:127.0.0.1:4000"