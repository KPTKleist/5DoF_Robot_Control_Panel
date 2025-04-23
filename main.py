import sys
import numpy as np
import tkinter as tk
import customtkinter as ctk
import serial
import matplotlib.pyplot as plt
from typing import Optional
import roboticstoolbox as rtb

# Robot model
def build_robot_5dof():
    L1 = rtb.RevoluteDH(d=1.0, a=0.0, alpha=np.pi/2)
    L2 = rtb.RevoluteDH(d=0.0, a=3.0, alpha=0.0)
    L3 = rtb.RevoluteDH(d=0.0, a=2.0, alpha=0.0)
    L4 = rtb.RevoluteDH(d=-0.5, a=0.0, alpha=-np.pi/2)
    L5 = rtb.RevoluteDH(d=1.0, a=0.0, alpha=0.0)

    robot = rtb.DHRobot([L1, L2, L3, L4, L5], name='5dof')
    return robot

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

        # Joint angle entries
        self.entries: list[ctk.CTkEntry] = []
        for i in range(1, 6):
            ctk.CTkLabel(self, text=f"q{i} (°)").grid(row=i-1, column=0, padx=8, pady=4, sticky="e")
            ent = ctk.CTkEntry(self, width=90)
            ent.insert(0, "0")
            ent.grid(row=i-1, column=1, padx=8, pady=4)
            self.entries.append(ent)

        # Gripper switch (q6)
        self.grip_switch = ctk.CTkSwitch(self, text="Gripper open", onvalue=90, offvalue=0)
        self.grip_switch.grid(row=5, columnspan=2, pady=(4, 12))

        # Compute FK button
        ctk.CTkButton(self, text="Compute FK", command=self.compute_fk).grid(row=6, columnspan=2, pady=(0, 12))

        # Serial port field + connect button
        port_frame = ctk.CTkFrame(self)
        port_frame.grid(row=7, columnspan=2, pady=(0, 8), sticky="we")
        ctk.CTkLabel(port_frame, text="Serial port").pack(side="left", padx=(6, 2))
        self.port_entry = ctk.CTkEntry(port_frame, width=90)
        self.port_entry.insert(0, "COM3" if sys.platform.startswith("win") else "/dev/ttyUSB0")
        self.port_entry.pack(side="left", padx=4)
        self.connect_btn = ctk.CTkButton(port_frame, text="Connect", width=80, command=self.toggle_serial)
        self.connect_btn.pack(side="left", padx=6)

        # Serial action buttons
        btn_frame = ctk.CTkFrame(self, fg_color="transparent")
        btn_frame.grid(row=8, columnspan=2, pady=(0, 12))
        ctk.CTkButton(btn_frame, text="Send to Serial", command=self.send_serial).pack(side="left", padx=6)

        # Auto-send switch
        self.auto_job = None
        self.auto_switch = ctk.CTkSwitch(
            btn_frame,
            text="Auto-send",
            command=self.toggle_auto_send  # callback defined next section
        )
        self.auto_switch.pack(side="left", padx=6)

        # Forward kinematic display
        self.fk_text = ctk.CTkTextbox(self, width=260, height=140, wrap="none", state="disabled")
        self.fk_text.grid(row=9, columnspan=2, padx=10, pady=(0, 12))

        # PyPlot backend
        self.backend: Optional[object] = None

        # Robot
        self.robot = build_robot_5dof()

    # GUI HELPERS
    def _read_angles_deg(self) -> list[float]:
        try:
            q_deg = [float(e.get()) for e in self.entries]
        except ValueError:
            tk.messagebox.showerror("Input Error", "Enter numeric angles only.")
            raise
        q_deg.append(float(self.grip_switch.get())) # q6
        return q_deg

    def compute_fk(self):
        q_deg = self._read_angles_deg()[:-1] # q1 ~ q5
        q_deg[1] = q_deg[1] + 90
        q_deg[3] = q_deg[3] - 90
        q_rad = np.radians(q_deg)

        if self.backend is None:
            # First click → create the window and keep the backend
            self.backend = self.robot.plot(
                q_rad,
                backend="pyplot",
                block=False  # keep GUI responsive
            )

        else:
            # Subsequent calls and update existing figure
            self.robot.q = q_rad
            self.backend.step(0.05)

        T = self.robot.fkine(q_rad)
        self.fk_text.configure(state="normal")
        self.fk_text.delete("1.0", tk.END)
        self.fk_text.insert(tk.END, np.array_str(T.A, precision=3, suppress_small=True))
        self.fk_text.configure(state="disabled")

    def send_serial(self):
        q_deg = self._read_angles_deg()
        packet = "[" + ", ".join(f"{ang:.2f}" for ang in q_deg) + "]"
        serial_link.send(packet)

    def toggle_serial(self):
        if serial_link.ser and serial_link.ser.is_open:
            serial_link.close()
            self.connect_btn.configure(text="Connect")
            self.auto_switch.configure(state="normal")
        else:
            port = self.port_entry.get()
            self.auto_switch.deselect()
            self.auto_switch.configure(state="disabled")
            self.stop_auto_send()
            if serial_link.connect(port):
                self.connect_btn.configure(text="Disconnect")

    # AUTO-SEND HELPERS
    # Called whenever the switch flips
    def toggle_auto_send(self):
        if self.auto_switch.get():  # switch just turned ON (value=1)
            self.start_auto_send()
        else:  # switch just turned OFF (value=0)
            self.stop_auto_send()

    # Begin the 200 ms repeating job
    def start_auto_send(self):
        if self.auto_job is None:  # prevent duplicates
            self._auto_loop()  # kick off immediately

    # Cancel the repeating job (if any)
    def stop_auto_send(self):
        if self.auto_job is not None:
            self.after_cancel(self.auto_job)
            self.auto_job = None

    def _auto_loop(self):
        if not self.auto_switch.get():  # switch flipped off mid-loop
            self.auto_job = None
            return

        # Build & send the packet (reuse existing helper)
        try:
            q_deg = self._read_angles_deg()
            packet = "[" + ", ".join(f"{ang:.2f}" for ang in q_deg) + "]"
            serial_link.send(packet)
        except Exception:
            # optional: show a warning or simply ignore bad input
            pass

        # Schedule the next tick in 200 ms (0.2 s)
        self.auto_job = self.after(200, self._auto_loop)

if __name__ == "__main__":
    app = App()
    app.mainloop()