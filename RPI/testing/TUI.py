import curses

def draw_menu(stdscr, fields, selected):
    stdscr.clear()
    curses.curs_set(0)
    h, w = stdscr.getmaxyx()

    for idx, (label, value) in enumerate(fields.items()):
        x = w//2 - 20
        y = h//2 - len(fields)//2 + idx
        if idx == selected:
            stdscr.attron(curses.A_REVERSE)
        stdscr.addstr(y, x, f"{label}: {value}")
        if idx == selected:
            stdscr.attroff(curses.A_REVERSE)

    stdscr.addstr(h-2, 2, "Use ↑↓ to navigate, Enter to edit, q to quit")
    stdscr.refresh()

def edit_field(stdscr, label, current_value):
    curses.curs_set(1)
    stdscr.clear()
    stdscr.addstr(0, 0, f"Enter new value for {label}: ")
    stdscr.addstr(1, 0, current_value)
    stdscr.refresh()

    win = curses.newwin(1, 40, 1, len(label) + 20)
    box = curses.textpad.Textbox(win)
    new_value = box.edit().strip()
    return new_value

def main(stdscr):
    import curses.textpad

    fields = {
        "Name": "John Doe",
        "Age": "25",
        "Email": "john@example.com"
    }

    selected = 0
    field_keys = list(fields.keys())

    while True:
        draw_menu(stdscr, fields, selected)
        key = stdscr.getch()

        if key == curses.KEY_UP and selected > 0:
            selected -= 1
        elif key == curses.KEY_DOWN and selected < len(fields) - 1:
            selected += 1
        elif key in (curses.KEY_ENTER, 10, 13):
            label = field_keys[selected]
            current = fields[label]
            new_val = edit_field(stdscr, label, current)
            fields[label] = new_val
        elif key == ord('q'):
            break

    stdscr.clear()
    stdscr.addstr(0, 0, "Final values:")
    for i, (k, v) in enumerate(fields.items()):
        stdscr.addstr(i+1, 2, f"{k}: {v}")
    stdscr.addstr(len(fields)+2, 0, "Press any key to exit.")
    stdscr.refresh()
    stdscr.getch()

curses.wrapper(main)
