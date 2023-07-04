import serial, atexit
from time import sleep
from typing import Union

from .profiling import time_this


global serial_link
serial_link = None


def link(device_path: str, baud_rate: int = 57600, timeout: int = 1) -> None:
    """
    Connect to the serial device hosted on **device_path** with a given baud_rate and timeout.
    Sleeps for 5 seconds to ensure correct connection.
    """

    global serial_link

    # Make sure we're not already connected
    unlink()

    print("Starting serial link")
    serial_link = serial.Serial(device_path, baud_rate, timeout=timeout)
    # Delayed to ensure link is correctly established
    countdown_duration = 5
    for i in range(countdown_duration):
        print("Starting in ", countdown_duration - i)
        sleep(1)

    print("Started")


def unlink() -> None:
    """
    Unlinks the currently connected serial device, if connected.
    """
    global serial_link
    if serial_link is not None and serial_link.is_open:
        serial_link.close()

    serial_link = None

# Make sure we disconnect before program exit
atexit.register(unlink)


@time_this
def send_data(turning_val: Union[str, int], speed_value: Union[str, int]) -> None:
    """
    Transmits turning and speed values to the connected microcontroller.
    Throws **serial.SerialException** if the serial connection is uninitialised or closed.
    """
    global serial_link

    if serial_link is None or not serial_link.is_open:
        raise serial.SerialException("Attempted to send data without a properly linked serial device")

    # print(turning_val, speed_value)
    serial_link.write(bytes(f"drc{turning_val}s{speed_value}t", "utf-8"))
    serial_link.flush()


