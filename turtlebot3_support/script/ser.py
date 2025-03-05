import serial

def send_command(ser, command, printout=False):
    """
    Send an AT command to the serial device and process the response.

    Parameters:
    ser (serial.Serial): The serial connection object.
    command (str): The AT command to send.
    printout (bool): Whether to print the response.

    Returns:
    tuple: A tuple of integers extracted from the response, or None if no valid response is found.
    """
    ser.flushInput()  # Clear input buffer
    ser.flushOutput()  # Clear output buffer

    ser.write(("AT"+command+"\r\n").encode())  # Send command

    raw = ser.read_until(b'OK\r\n')  # Read response until "OK"

    response = raw.decode(errors='ignore')  # Decode the response, ignoring errors

    if printout: # Print the response if requested
        print(">>>", response)

    # Process response
    for line in response.splitlines():
        line = line.strip()  # Remove leading/trailing whitespace
        if line.startswith(command):  # Check if the line starts with the command
            parts = line.split(":")[1].split(",")  # Get the part after `:`, then split by `,`
            numbers = tuple(int(p) for p in parts if p.strip().lstrip('-').isdigit())  # Convert valid numbers
            return numbers  # Return tuple with only valid integers
    return None  # Return None if no valid response is found

port = "/dev/ttyUSB4"  # Adjust for your setup
baudrate = 115200  # Match your module's speed

with serial.Serial(port, baudrate, timeout=0.3) as ser:
    
    import time
    start_time = time.time()  # Record the start time

    # Send AT commands and print the responses
    out = send_command(ser, "+CSQ")
    print("CSQ:", out)
    out = send_command(ser, "+QRSRP")
    print("RSRP:", out)
    out = send_command(ser, "+QRSRQ")
    print("RSRQ:", out)
    out = send_command(ser, "+QSINR")
    print("SINR:", out)

    end_time = time.time()  # Record the end time

    print("Time elapsed: ", end_time - start_time)  # Print the elapsed time

    # Some more AT command tests, printing the unprocessed responses
    send_command(ser, "+QNWCFG=\"lte_csi\"", True)
    send_command(ser, "+QNWCFG=\"nr5g_csi\"", True)
