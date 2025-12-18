

VALID_PREFIXES = {
    "Ch1:": 'x',  # Print Serial AUX
    "accX:": 'a', # Print Accelerometer
    "gyrX:": 'g', # Print Gyroscope
    "qW:": 'q',   # Print Quaternion
    "Roll:": 'e', # Print Euler angles in deg*100
    "Temp:": 't', # Print Temperature in degC*100
    "Walked": 'p' # Print Pedometer
}

def get_toggle_command(line: str) -> str | None:
    """
    Checks a line of serial output. 
    If it matches a known valid prefix, returns the char to toggle it on.
    Returns None if the line is clean (or unknown).
    """
    clean_line = line.strip()
    
    for prefix, cmd_char in VALID_PREFIXES.items():
        if clean_line.startswith(prefix):
            return cmd_char
            
    return None