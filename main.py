import sys
import numpy as np
import tkinter as tk
import customtkinter as ctk
import serial
import matplotlib.pyplot as plt
from typing import Optional
import roboticstoolbox as rtb

# Constants
START_Q_DEG = [0, 0, 0, 0, 0] # q1-q5 at launch (degrees)
START_GRIP = 0 # 0 ° closed, 90 ° open
INCR_STEP = 1.0 # ° added / subtracted per tick
HOLD_MS = 100 # repeat interval while button is held (ms)

# Robot model
def build_robot_5dof():
    L1 = rtb.RevoluteDH(d=1.0, a=0.0, alpha=np.pi/2)
    L2 = rtb.RevoluteDH(d=0.0, a=3.0, alpha=0.0, offset=np.pi/2)
    L3 = rtb.RevoluteDH(d=0.0, a=2.0, alpha=0.0)
    L4 = rtb.RevoluteDH(d=-0.5, a=0.0, alpha=-np.pi/2, offset=-np.pi/2)
    L5 = rtb.RevoluteDH(d=1.0, a=0.0, alpha=0.0)

    return rtb.DHRobot([L1, L2, L3, L4, L5], name='5dof')

robot = build_robot_5dof() # global robot object

# Serial communication, a thin wrapper around pyserial to simplify connect/send/close.
class SerialLink:
    def __init__(self):
        self.ser: Optional[serial.Serial] = None

    def connect(self, port: str, baud: int = 115200) -> bool:
        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=1)
            return True
        except serial.SerialException as exc:
            tk.messagebox.showerror("Serial Error", f"Could not open {port}\n{exc}")
            return False

    def send(self, packet: str):
        if self.ser and self.ser.is_open:
            self.ser.write(packet.encode("utf-8"))
            self.ser.flush()
        else:
            tk.messagebox.showwarning("Serial Warning", "Serial port not connected!")

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

serial_link = SerialLink()

# GUI
ctk.set_appearance_mode("system")  # or 'dark' / 'light'
ctk.set_default_color_theme("blue")

class App(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("5-DoF Robot Control Panel")
        self.resizable(False, False)

        # state
        self.curr_q_deg: list[float] = START_Q_DEG.copy()  # live joint vector
        self.hold_active = False  # True while < or > is held
        self.hold_job: Optional[str] = None
        self.hold_params: tuple[int, int] = (0, 0)

        # plot the robot once
        self.backend = robot.plot(
            np.radians(self.curr_q_deg),
            backend="pyplot",
            block=False,
            limits=[-5, 5, -5, 5, -1, 9]
        )

        # build joint rows (labels, entries, Set buttons, current labels)
        self.entries: list[ctk.CTkEntry] = []
        self.curr_labels: list[ctk.CTkLabel] = []

        for i in range(5):  # q1-q5
            row = i
            ctk.CTkLabel(self, text=f"q{i + 1} (°)").grid(
                row=row, column=0, padx=6, pady=4, sticky="e"
            )

            ent = ctk.CTkEntry(self, width=70)
            ent.insert(0, str(START_Q_DEG[i]))
            ent.grid(row=row, column=1, padx=2, pady=3)
            self.entries.append(ent)

            btn_lt = ctk.CTkButton(self, text="<", width=26)
            btn_lt.grid(row=row, column=2, padx=2, pady=3)
            btn_lt.bind("<ButtonPress-1>", lambda e, j=i: self._hold_start(j, -1))
            btn_lt.bind("<ButtonRelease-1>", lambda e: self._hold_stop())

            btn_rt = ctk.CTkButton(self, text=">", width=26)
            btn_rt.grid(row=row, column=3, padx=2, pady=3)
            btn_rt.bind("<ButtonPress-1>", lambda e, j=i: self._hold_start(j, +1))
            btn_rt.bind("<ButtonRelease-1>", lambda e: self._hold_stop())

            ctk.CTkButton(
                self, text="Set", width=38,
                command=lambda j=i: self.set_joint(j)
            ).grid(row=row, column=4, padx=2, pady=3)

            lbl = ctk.CTkLabel(self, text=f"{START_Q_DEG[i]:.2f}", width=60, anchor="w")
            lbl.grid(row=row, column=5, padx=6, pady=3, sticky="w")
            self.curr_labels.append(lbl)

        # gripper switch (row 5)
        self.grip_switch = ctk.CTkSwitch(
            self, text="Gripper open", onvalue=90, offvalue=0
        )
        if START_GRIP == 90:
            self.grip_switch.select()
        self.grip_switch.grid(row=5, column=0, columnspan=3, pady=(4, 12))

        # serial-port frame (row 6)
        port_frame = ctk.CTkFrame(self)
        port_frame.grid(row=6, column=0, columnspan=6, sticky="we", pady=(0, 8))
        ctk.CTkLabel(port_frame, text="Serial port").pack(side="left", padx=(6, 2))
        self.port_entry = ctk.CTkEntry(port_frame, width=90)
        default_port = "COM3" if sys.platform.startswith("win") else "/dev/ttyUSB0"
        self.port_entry.insert(0, default_port)
        self.port_entry.pack(side="left", padx=4)
        self.connect_btn = ctk.CTkButton(
            port_frame, text="Connect", width=80, command=self.toggle_serial
        )
        self.connect_btn.pack(side="left", padx=6)

        # action buttons frame (row 7)
        btn_frame = ctk.CTkFrame(self, fg_color="transparent")
        btn_frame.grid(row=7, column=0, columnspan=6, pady=(0, 12))

        ctk.CTkButton(btn_frame, text="Send to Serial",
                      command=self.send_serial).pack(side="left", padx=6)

        self.auto_switch = ctk.CTkSwitch(
            btn_frame, text="Auto-send", command=self.toggle_auto_send
        )
        self.auto_switch.pack(side="left", padx=6)

    # GUI helpers
    def set_joint(self, index: int):
        try:
            val = float(self.entries[index].get())
        except ValueError:
            tk.messagebox.showerror("Input Error", "Enter a numeric angle.")
            return

        # 1) store & label
        self.curr_q_deg[index] = val
        self.curr_labels[index].configure(text=f"{val:.2f}")

        # 2) move robot & refresh plot
        robot.q = np.radians(self.curr_q_deg)
        self.backend.step(0.05)

    # Serial
    def toggle_serial(self):
        if serial_link.ser and serial_link.ser.is_open:
            serial_link.close()
            self.connect_btn.configure(text="Connect")
            self.auto_switch.deselect()
            self.auto_switch.configure(state="disabled")
            self.stop_auto_send()
        else:
            port = self.port_entry.get()
            if serial_link.connect(port):
                self.connect_btn.configure(text="Disconnect")
                self.auto_switch.configure(state="normal")

    def _packet(self) -> str:
        """Build the “[q1,…,q6]” string from current state."""
        q_send = self.curr_q_deg.copy()
        q_send.append(float(self.grip_switch.get()))
        return "[" + ", ".join(f"{ang:.2f}" for ang in q_send) + "]"

    def send_serial(self):
        """Send one packet, refresh labels/plot with entry values."""
        # Allow quick send of values typed but not 'Set'-clicked yet
        try:
            new_vals = [float(e.get()) for e in self.entries]
            self.curr_q_deg = new_vals
            for i, v in enumerate(new_vals):
                self.curr_labels[i].configure(text=f"{v:.2f}")
            robot.q = np.radians(self.curr_q_deg)
            self.backend.step(0.05)
        except ValueError:
            tk.messagebox.showerror("Input Error", "Enter numeric angles.")
            return

        serial_link.send(self._packet())

    # Auto-send
    def toggle_auto_send(self):
        if self.auto_switch.get():     # turned ON
            self.start_auto_send()
        else:                          # turned OFF
            self.stop_auto_send()

    def start_auto_send(self):
        if self.auto_job is None:
            self._auto_loop()          # kick off immediately

    def stop_auto_send(self):
        if self.auto_job is not None:
            self.after_cancel(self.auto_job)
            self.auto_job = None

    def _auto_loop(self):
        if not self.auto_switch.get():
            self.auto_job = None
            return

        serial_link.send(self._packet())
        self.auto_job = self.after(200, self._auto_loop)

    # increment & hold helpers
    def _increment_joint(self, idx: int, delta_sign: int):
        """Apply ±INCR_STEP to joint *idx* and refresh GUI + plot."""
        self.curr_q_deg[idx] += delta_sign * INCR_STEP
        val = self.curr_q_deg[idx]
        self.entries[idx].delete(0, tk.END)
        self.entries[idx].insert(0, f"{val:.2f}")
        self.curr_labels[idx].configure(text=f"{val:.2f}")
        robot.q = np.radians(self.curr_q_deg)
        self.backend.step(0.05)

    def _hold_start(self, idx: int, delta_sign: int):
        self.hold_active = True  # ← NEW
        self._increment_joint(idx, delta_sign)
        self.hold_params = (idx, delta_sign)
        self.hold_job = self.after(HOLD_MS, self._hold_repeat)

    def _hold_repeat(self):
        if not self.hold_active:  # ← NEW: stop loop
            self.hold_job = None
            return
        idx, delta_sign = self.hold_params
        self._increment_joint(idx, delta_sign)
        self.hold_job = self.after(HOLD_MS, self._hold_repeat)

    def _hold_stop(self):
        self.hold_active = False  # ← NEW
        if self.hold_job:
            self.after_cancel(self.hold_job)
            self.hold_job = None

if __name__ == "__main__":
    app = App()
    app.mainloop()