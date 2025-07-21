import sys
import select
import tty
import termios
import time

def read_line_nonblocking():
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    input_str = ""
    try:
        print("Type something (press Enter to finish):")
        while True:
            if select.select([sys.stdin], [], [], 0)[0]:
                char = sys.stdin.read(1)
                if char == '\n':
                    break
                print(char, end='', flush=True)
                input_str += char
                
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    return input_str

result = read_line_nonblocking()
print(f"\nYou typed: {result}")
