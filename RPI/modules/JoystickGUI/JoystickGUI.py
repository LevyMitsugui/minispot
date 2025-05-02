import tkinter as tk
import math

# Configuration
OUTER_RADIUS = 100
STICK_RADIUS = 20
ROLL_MIN = -0.5
ROLL_MAX = 0.5
PITCH_MIN = -0.5
PITCH_MAX = 0.5
CANVAS_SIZE = OUTER_RADIUS * 2 + 20
X_MIN = -0.05
X_MAX = 0.05
Y_MIN = -0.05
Y_MAX = 0.05
SLIDER_STEPS = 100

class JoystickGUI:
    def __init__(self, root, on_change_only=False):
        
        self.root = root
        self.root.title("Joystick GUI")
        self.output_callback = None
        self.on_change_only = on_change_only

        self.prev_output = None

        self.canvas = tk.Canvas(root, width=CANVAS_SIZE, height=CANVAS_SIZE, bg="white")
        self.canvas.grid(row=0, column=0, padx=10, pady=10)

        self.center = CANVAS_SIZE // 2
        self.stick_pos = [self.center, self.center]

        # Draw outer boundary
        self.canvas.create_oval(
            self.center - OUTER_RADIUS, self.center - OUTER_RADIUS,
            self.center + OUTER_RADIUS, self.center + OUTER_RADIUS,
            outline="black"
        )

        # Draw inner stick
        self.stick = self.canvas.create_oval(0, 0, 0, 0, fill="blue")

        # Display roll and pitch
        self.label = tk.Label(root, text="Roll: 0.0  Pitch: 0.0", font=("Arial", 12))
        self.label.grid(row=1, column=0)

        # Degrees/radians toggle
        self.use_degrees = tk.BooleanVar(value=False)
        self.toggle = tk.Checkbutton(root, text="Show in Degrees", variable=self.use_degrees, command=self.update_angles)
        self.toggle.grid(row=2, column=0)

        # Horizontal slider (Y axis)
        self.snap_y = tk.BooleanVar(value=True)
        self.y_slider = tk.Scale(root, from_=0, to=SLIDER_STEPS, orient=tk.HORIZONTAL, length=200)
        self.y_slider.set(SLIDER_STEPS // 2)
        self.y_slider.grid(row=3, column=0)
        self.y_slider.bind("<ButtonRelease-1>", self.snap_back_y)
        self.y_snap_toggle = tk.Checkbutton(root, text="Snap Y to center", variable=self.snap_y)
        self.y_snap_toggle.grid(row=4, column=0)

        # Vertical slider (X axis)
        self.snap_x = tk.BooleanVar(value=True)
        self.x_slider = tk.Scale(root, from_=0, to=SLIDER_STEPS, orient=tk.VERTICAL, length=200)
        self.x_slider.set(SLIDER_STEPS // 2)
        self.x_slider.grid(row=0, column=1, rowspan=3)
        self.x_slider.bind("<ButtonRelease-1>", self.snap_back_x)
        self.x_snap_toggle = tk.Checkbutton(root, text="Snap X to center", variable=self.snap_x)
        self.x_snap_toggle.grid(row=3, column=1)

        # Internal storage
        self.x_value = 0.0
        self.y_value = 0.0

        # Now it's safe to draw the stick (which updates label)
        self.draw_stick(self.center, self.center)

        # Mouse bindings
        self.canvas.tag_bind(self.stick, '<Button-1>', self.start_drag)
        self.canvas.tag_bind(self.stick, '<B1-Motion>', self.drag)
        self.canvas.tag_bind(self.stick, '<ButtonRelease-1>', self.end_drag)
        self.dragging = False

        # Start update loop
        self.update_loop()

    def set_output_callback(self, callback_fn):
        self.output_callback = callback_fn

    def draw_stick(self, x, y):
        self.stick_pos = [x, y]
        self.canvas.coords(
            self.stick,
            x - STICK_RADIUS, y - STICK_RADIUS,
            x + STICK_RADIUS, y + STICK_RADIUS
        )
        self.update_angles()

    def start_drag(self, event):
        self.dragging = True

    def drag(self, event):
        if not self.dragging:
            return
        dx = event.x - self.center
        dy = event.y - self.center
        dist = math.hypot(dx, dy)
        if dist > OUTER_RADIUS:
            scale = OUTER_RADIUS / dist
            dx *= scale
            dy *= scale
        self.draw_stick(self.center + dx, self.center + dy)

    def end_drag(self, event):
        self.dragging = False
        self.draw_stick(self.center, self.center)

    def update_angles(self):
        dx = self.stick_pos[0] - self.center
        dy = self.stick_pos[1] - self.center

        norm_x = dx / OUTER_RADIUS  # range [-1, 1]
        norm_y = dy / OUTER_RADIUS  # range [-1, 1]

        roll = norm_x * (ROLL_MAX - ROLL_MIN) / 2
        pitch = norm_y * (PITCH_MAX - PITCH_MIN) / 2

        roll = max(min(roll, ROLL_MAX), ROLL_MIN)
        pitch = max(min(pitch, PITCH_MAX), PITCH_MIN)

        if self.use_degrees.get():
            roll_disp = math.degrees(roll)
            pitch_disp = math.degrees(pitch)
            self.label.config(text=f"Roll: {roll_disp:.1f}°  Pitch: {pitch_disp:.1f}°")
        else:
            self.label.config(text=f"Roll: {roll:.3f}  Pitch: {pitch:.3f}")

        self.roll = roll
        self.pitch = pitch

    def snap_back_y(self, event):
        if self.snap_y.get():
            self.y_slider.set(SLIDER_STEPS // 2)

    def snap_back_x(self, event):
        if self.snap_x.get():
            self.x_slider.set(SLIDER_STEPS // 2)

    def map_slider_to_range(self, value, min_val, max_val):
        center = SLIDER_STEPS // 2
        scaled = (center - value) / center  # [-1, 1]
        return min_val + (max_val - min_val) * (scaled + 1) / 2

    def update_loop(self):
        self.x_value = self.map_slider_to_range(self.x_slider.get(), X_MIN, X_MAX)
        self.y_value = self.map_slider_to_range(self.y_slider.get(), Y_MIN, Y_MAX)
        print(f"X: {self.x_value:.4f}, Y: {self.y_value:.4f}")

        self.x_slider.config(label=f"X Axis: {self.x_value:.4f} m")
        self.y_slider.config(label=f"Y Axis: {self.y_value:.4f} m")

        output = {
            'roll': self.roll,
            'pitch': self.pitch,
            'x': self.x_value,
            'y': self.y_value
        }

        if self.output_callback:
            if self.on_change_only:
                if output != self.prev_output:
                    self.output_callback(output)
                    self.prev_output = output.copy()
            else:
                self.output_callback(output)

        self.root.after(100, self.update_loop)