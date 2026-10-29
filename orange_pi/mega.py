"""
RoboSort - Talk to the Arduino Mega by Hand (laptop + Orange Pi One)

The Pi has no Arduino Serial Monitor; this is one. It opens the Mega's
port, prints everything the Mega says, and sends whatever you type - the
same commands the firmware takes (arduino/main/main.ino):

    8 2 4 6 5   forward / backward / left / right / stop  (or FORWARD, BACK,
                LEFT, RIGHT, STOP)
    U D S R H   lift up / lift down / stretch out / retract / home
    7 9 0       camera 45 deg left / 45 deg right / straight ahead  (or
                LOOKLEFT, LOOKRIGHT, LOOKAHEAD)
    C O         conveyor on / off
    A           start driving on its own
    ?           command list + a STATUS line

Opening the port resets the Mega, which restarts its 10 s start-up
countdown. Unless --auto is given, this tool answers the countdown with
'5' so the rover stays put for a bench test. With --auto the countdown
runs out and the rover drives off.

Usage:
    python mega.py               # interactive console, rover held still
    python mega.py ?             # send '?', print the replies, exit
    python mega.py U D ?         # several commands, in order
    python mega.py --auto        # console, and let the rover drive after 10 s
"""

import argparse
import time

import serial_link
from config import CFG


def main():
    p = argparse.ArgumentParser(description="RoboSort Mega console")
    p.add_argument("commands", nargs="*",
                   help="send these, print the replies, exit (none = interactive)")
    p.add_argument("--port", default=CFG.serial_port)
    p.add_argument("--auto", action="store_true",
                   help="let the start-up countdown run - the rover WILL drive")
    p.add_argument("--wait", type=float, default=2.0,
                   help="seconds to show replies after each command (default 2)")
    args = p.parse_args()

    link = serial_link.SerialLink(args.port, CFG.baud_rate, CFG.serial_retries)
    try:
        if not args.auto:
            link.send("5", force=True)       # cancels the countdown: stay put
            time.sleep(0.5)

        if args.commands:
            for command in args.commands:
                link.send(command, force=True)
                time.sleep(args.wait)
            return

        print("Type a command and press Enter ('quit' or Ctrl+C to leave).")
        while True:
            try:
                line = input().strip()
            except EOFError:
                break
            if line.lower() in ("quit", "exit"):
                break
            if line:
                link.send(line, force=True)
    except KeyboardInterrupt:
        pass
    finally:
        link.close()


if __name__ == "__main__":
    main()
