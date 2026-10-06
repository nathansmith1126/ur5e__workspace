import socket  # Communicate with the Robotiq URCap over TCP.
import time  # Provide polling delays and movement deadlines.

ROBOT_IP = "172.22.22.2"  # IP address of the UR controller, not a separate gripper IP.
GRIPPER_ID = 3  # Select the gripper ID shown on the pendant.
SPEED = 64  # Requested speed on the raw 0–255 scale.
FORCE = 32  # Requested force on the raw 0–255 scale, not in newtons.


def receive(sock: socket.socket, *, ack: bool = False):  # Read either a command acknowledgement or a status line.
    """Assemble one URCap response, even when TCP splits it across packets.

    Args:
        sock (socket.socket): Connected TCP socket object shared by the test's
            command functions. Its configured timeout limits blocking reads.
        ack (bool): Keyword-only flag; True permits a three-byte 'ack' or 'nak'
            response without a newline. False expects a newline-terminated line.

    Local variables:
        data (bytearray): Mutable sequence of integer byte values that buffers
            the response as it arrives.
        chunk (bytes): Immutable sequence containing one received byte, or an
            empty sequence if the peer has closed the connection.

    Returns:
        str: ASCII response, such as 'ack', 'nak', or 'STA 3'. A line response
            has surrounding whitespace removed.

    Raises:
        ConnectionError: The peer closes the connection before a full response.
        RuntimeError: An unterminated response exceeds the buffer limit.
        OSError: A socket operation fails, including a socket timeout.
        UnicodeDecodeError: The response contains non-ASCII bytes.
    """
    data = bytearray()  # Accumulate response bytes until a complete message arrives.
    while True:  # Continue until the response terminator or an error is encountered.
        chunk = sock.recv(1)  # Read one byte, subject to the socket timeout.
        if not chunk:  # An empty read means the remote endpoint closed the connection.
            raise ConnectionError("Gripper connection closed")  # Abort on a lost connection.
        data.extend(chunk)  # Append the byte to the response buffer.
        if ack and bytes(data) in (b"ack", b"nak"):  # These acknowledgements have no newline.
            return data.decode("ascii")  # Return the accepted or rejected command response.
        if chunk == b"\n":  # GET responses end with a newline.
            return data.decode("ascii").strip()  # Return the response without surrounding whitespace.
        if len(data) > 256:  # Limit buffering if the service sends an unexpected response.
            raise RuntimeError("Unexpected response length")  # Stop rather than read indefinitely.


def set_values(sock: socket.socket, values: str):  # Send one or more variable assignments to the URCap.
    """Send a SET request and require acceptance before continuing the test.

    Args:
        sock (socket.socket): Connected TCP socket object used for both sending
            the command and receiving its acknowledgement.
        values (str): Space-separated variable/value pairs, such as 'SID 3' or
            'POS 125 SPE 64 FOR 32 GTO 1'. This is protocol text, not a Python
            dictionary; omit the leading SET and trailing newline.

    Local variables:
        reply (str): Decoded acknowledgement returned by receive(). Only 'ack'
            is accepted; it confirms command acceptance, not completed motion.

    Returns:
        None: Success is indicated by returning without an exception.

    Raises:
        RuntimeError: The service returns a response other than 'ack'.
        OSError: Sending or receiving fails. Other receive() errors propagate.
    """
    sock.sendall(f"SET {values}\n".encode("ascii"))  # Encode and transmit the complete SET command.
    reply = receive(sock, ack=True)  # Wait for the service to acknowledge the command.
    if reply != "ack":  # Only ack indicates that the service accepted the command.
        raise RuntimeError(f"Command rejected: {values!r}: {reply!r}")  # Include the failed command.


def get_value(sock: socket.socket, name: str):  # Read a named gripper variable as an integer.
    """Query one gripper variable and validate its numeric response.

    Args:
        sock (socket.socket): Connected TCP socket object with the intended
            gripper already selected through SET SID.
        name (str): Protocol variable name, such as 'STA', 'POS', or 'FLT'.

    Local variables:
        reply (str): Complete decoded response, for example 'POS 125'.
        fields (list[str]): Ordered list of whitespace-separated strings.
            Exactly two entries are expected: the variable name at index 0
            and its numeric value at index 1.

    Returns:
        int: Parsed base-10 value. Its units and meaning depend on name; POS
            is a raw position, while STA and FLT are status and fault codes.

    Raises:
        RuntimeError: The response has the wrong fields or variable name.
        ValueError: The value cannot be parsed as an integer, for example '?'.
        OSError: Communication fails. Other receive() errors propagate.
    """
    # Request the variable using the text protocol.
    sock.sendall(f"GET {name}\n".encode("ascii"))  
    
    # Read the newline-terminated response.
    reply = receive(sock)  
    
    # Separate the echoed variable name from its value.
    fields = reply.split() 
    if len(fields) != 2 or fields[0] != name:  # Verify that the response matches the request.
        raise RuntimeError(f"Unexpected response: {reply!r}")  # Reject malformed or mismatched replies.
    return int(fields[1])  # Convert the value; an unknown value such as '?' raises ValueError.


def move_gripper(sock: socket.socket, position: int, timeout: float = 15):
    """Command finger movement and poll until contact or travel completion.

    Args:
        sock (socket.socket): Connected TCP socket object for the selected,
            activated gripper; main() performs those setup checks.
        position (int): Requested raw finger position from 0 (open) to 255
            (closed), not a distance in millimeters. Not validated here.
        timeout (int | float): Polling deadline interval in seconds, default 15.
            Individual blocking socket calls can extend the elapsed time.

    Globals used:
        SPEED (int): Raw 0–255 speed setting included in the motion command.
        FORCE (int): Raw 0–255 force setting, not a measured force in newtons.

    Local variables:
        deadline (float): Absolute monotonic-clock time after which polling ends.
        fault (int): FLT code; zero means no reported fault.
        status (int): OBJ code; 0 means moving, 1/2 indicate contact, and 3
            indicates completed travel to the requested position.
        actual (int): Raw measured finger position read when movement ends.
        result (str): Human-readable outcome printed to the terminal.

    Returns:
        None: Prints the outcome and returns on either contact or completion.
            Contact does not abort the remaining demonstration sequence.

    Raises:
        RuntimeError: A command is rejected or a nonzero fault is reported.
        TimeoutError: Movement does not finish before the polling deadline.
        OSError: Socket communication fails. Parsing/helper errors propagate.

    Note:
        This function moves the gripper, not the arm. A timeout or exception
        does not issue a physical stop command or release the held object.
    """
    
    # Set motion parameters and start moving.
    set_values(sock, f"POS {position} SPE {SPEED} FOR {FORCE} GTO 1")  
    
    deadline = time.monotonic() + timeout  # Use a clock unaffected by wall-clock adjustments.

    while time.monotonic() < deadline:  # Bound polling time; individual socket reads can also take time.
        fault = get_value(sock, "FLT")  # Read the gripper's current fault code.
        if fault:  # A nonzero value indicates a reported fault.
            raise RuntimeError(f"Gripper fault: {fault}")  # End the test and report the code.

        if get_value(sock, "PRE") == position:  # Wait until the gripper echoes the requested target.
            status = get_value(sock, "OBJ")  # Read movement/contact status; zero means moving.
            # 0 - fingers moving 
            # 1 - fingers stopped against resistance while opening
            # 2 - fingers stopped against resistance while closing
            # 3 - fingers reached prescribed position
            if status in (1, 2, 3):  # Contact or completed travel ends this wait.
                actual = get_value(sock, "POS")  # Read the actual position, which may differ from the target.
                result = "object contact" if status in (1, 2) else "completed"  # Explain why motion ended.
                print(f"Target={position}, position={actual}: {result}")  # Report the movement outcome.
                return  # Return control to the open/close sequence.

        time.sleep(0.1)  # Space polling cycles apart to avoid continuously querying the controller.

    raise TimeoutError("Gripper movement timed out")  # Stop waiting; this does not issue a physical stop.


def main():  # Run an open, close, open demonstration; keep fingers and objects clear.
    """Connect, verify readiness, and execute the configured gripper test.

    Args:
        None.

    Globals used:
        ROBOT_IP (str): Dotted IPv4 address of the UR controller hosting the
            Robotiq service, not a separate network address for the gripper.
        GRIPPER_ID (int): Gripper device ID selected for this socket connection.

    Local variables and data structures:
        sock (socket.socket): TCP connection to (ROBOT_IP, 63352), a two-item
            (str, int) address tuple. The context manager closes it on exit.
        label (str): 'Open' or 'Close', unpacked from each test step for logging.
        position (int): Raw target unpacked from each test step and passed to
            move(). The current 'Close' step requests 125, not full closure.
        The loop iterates over a tuple of three (str, int) tuples:
            (('Open', 0), ('Close', 125), ('Open', 0)). Each pair describes one
            action. Socket timeouts are three seconds; pauses are one second.

    Returns:
        None: Runs the sequence and prints progress and movement outcomes.

    Raises:
        RuntimeError: The gripper is inactive, reports a fault, or rejects a
            command. Communication, parsing, and movement errors propagate,
            preventing subsequent test steps from running.

    Note:
        Activate the gripper from the pendant first. This test physically moves
        its fingers; keep the opening clear. Closing the socket is not a stop
        or release command. Importing this module does not run the test.
    """
    with socket.create_connection((ROBOT_IP, 63352), timeout=3) as sock:  # Connect to the URCap and close on exit.
        sock.settimeout(3)  # Allow up to three seconds for each blocking socket operation.
        set_values(sock, f"SID {GRIPPER_ID}")  # Select gripper 3 for this connection; this does not change its ID.

        if get_value(sock, "STA") != 3:  # Status 3 means activated, independently of the gripper ID.
            raise RuntimeError("Activate the gripper from the pendant first.")  # Require prior activation.
        if get_value(sock, "FLT") != 0:  # Confirm that no fault is reported before commanding motion.
            raise RuntimeError("Resolve the gripper fault before moving.")  # Abort if a fault is present.

        for label, position in (("Open", 0), ("Close", 255), ("Open", 0)):  # Use raw endpoint targets.
            print(label)  # Announce the next movement in the terminal.
            move(sock, position)  # Command the movement and wait for completion or contact.
            time.sleep(1)  # Pause for one second between movements.


if __name__ == "__main__":  # Run the demonstration only when executed directly, not when imported.
    main()  # Connect and execute the gripper sequence; no arm motion is commanded.
