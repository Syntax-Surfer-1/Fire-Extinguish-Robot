import customtkinter as ctk
from PIL import Image
import os
import serial
import serial.tools.list_ports
from tkinter import messagebox
import webbrowser
import sys

class SensorDashboard(ctk.CTk):
    def __init__(self):
        super().__init__()

        self.serial_connection = None
        self.toggle_state = False
        self.title("Robo Car Controller")
        self.geometry("1100x700")
        self.resizable(False, False)
        ctk.set_appearance_mode("dark")
        

        self.bg_color = "#1E1E2E"
        self.card_color = "#2E2E3E"
        self.accent_color = "#7F5AF0"
        self.button_hover = "#5E4AE3"
        self.text_white = "#FFFFFF"
        self.text_alert = "#FF4C4C"

        self.configure(bg=self.bg_color)
        self.bind_all("<Key>", self.key_pressed)
        self.create_custom_menu()

        ctk.CTkLabel(self, text="Robo Car Dashboard", font=("Arial Rounded MT Bold", 28),
                     text_color=self.accent_color).place(x=400, y=30)

        image_path = os.path.join(os.path.dirname(__file__), "hi.png")
        icon = None
        if os.path.exists(image_path):
            icon_img = Image.open(image_path)
            icon = ctk.CTkImage(light_image=icon_img, dark_image=icon_img, size=(60, 60))

        sensor_frame = ctk.CTkFrame(self, width=580, height=340, corner_radius=10,
                                    fg_color=self.card_color, border_color=self.accent_color, border_width=2)
        sensor_frame.place(x=40, y=80)

        ctk.CTkLabel(sensor_frame, text="Sensor Values", font=("Segoe UI", 16, "bold"),
                     text_color=self.accent_color).place(x=20, y=10)

        self.sensor_values = []
        sensor_labels = [
            "Ultrasonic (Left)", "Ultrasonic (Front)", "Ultrasonic (Right)",
            "IR (Left)", "IR (Front)", "IR (Right)"
        ]
        positions = [(20 + (i % 3) * 190, 50 if i < 3 else 180) for i in range(6)]

        for i in range(6):
            frame = ctk.CTkFrame(sensor_frame, width=160, height=120, corner_radius=10,
                                 fg_color=self.card_color, border_color=self.accent_color, border_width=2)
            frame.place(x=positions[i][0], y=positions[i][1])

            if icon:
                ctk.CTkLabel(frame, image=icon, text="").place(relx=0.5, rely=0.2, anchor='center')

            ctk.CTkLabel(frame, text=sensor_labels[i], font=("Segoe UI", 11),
                         text_color=self.text_white).place(relx=0.5, rely=0.6, anchor='center')

            value_label = ctk.CTkLabel(frame, text="-- cm", font=("Segoe UI", 15, "bold"),
                                       text_color=self.text_white)
            value_label.place(relx=0.5, rely=0.85, anchor='center')
            self.sensor_values.append(value_label)

        self.status_icon = ctk.CTkLabel(self, text="●", font=("Segoe UI", 18), text_color="#FF5F5F")
        self.status_icon.place(x=210, y=500)

        self.status_label = ctk.CTkLabel(self, text="Status: NOT CONNECTED", text_color="#FF5F5F",
                                         font=("Segoe UI", 13))
        self.status_label.place(x=230, y=500)

        self.bluetooth_dropdown = ctk.CTkComboBox(self, values=["Select Device"], width=180,
                                                  fg_color=self.card_color, button_color=self.accent_color,
                                                  dropdown_fg_color=self.card_color,
                                                  font=("Segoe UI", 12))
        self.bluetooth_dropdown.place(x=50, y=540)

        self.scan_btn = ctk.CTkButton(self, text="Scan", width=80, height=36, font=("Segoe UI", 12),
                                      fg_color=self.accent_color, hover_color=self.button_hover,
                                      text_color=self.text_white, cursor="hand2", command=self.scan_devices)
        self.scan_btn.place(x=240, y=540)

        self.connect_btn = ctk.CTkButton(self, text="CONNECT", width=120, height=36, font=("Segoe UI", 13),
                                         fg_color=self.accent_color, text_color=self.text_white,
                                         hover_color=self.button_hover, cursor="hand2",
                                         command=self.toggle_connection)
        self.connect_btn.place(x=330, y=540)

        self.auto_mode_switch = ctk.CTkSwitch(self, text="Manual Mode", text_color=self.text_white,
                                              switch_height=24, switch_width=44, fg_color=self.accent_color,
                                              progress_color="#FFFFFF", onvalue="off", offvalue="on",
                                              command=self.switch_toggled)
        self.auto_mode_switch.place(x=850, y=120)

        right_x = 880
        remote_y = 200
        size = 80

        directions = {
            "↑": (right_x, remote_y),
            "←": (right_x - 100, remote_y + 100),
            "⏹": (right_x, remote_y + 100),
            "→": (right_x + 100, remote_y + 100),
            "↓": (right_x, remote_y + 200),
        }

        for symbol, (x, y) in directions.items():
            ctk.CTkButton(self, text=symbol, width=size, height=size,
                          font=("Segoe UI", 24, "bold"), corner_radius=10,
                          fg_color=self.accent_color, text_color=self.text_white,
                          hover_color=self.button_hover, cursor="hand2",
                          command=lambda cmd=symbol: self.move_robot(cmd)).place(x=x, y=y)

        extra_y = remote_y + 310
        ctk.CTkButton(self, text="🚿 WATER", width=270, height=45,
                      font=("Segoe UI", 13, "bold"), fg_color=self.accent_color,
                      text_color=self.text_white, hover_color=self.button_hover,
                      cursor="hand2", command=lambda: self.send_command("W")).place(x=right_x - 100, y=extra_y)

        ctk.CTkButton(self, text="⚡ MAX SPEED", width=120, height=38,
                      font=("Segoe UI", 12), fg_color=self.accent_color,
                      text_color=self.text_white, hover_color=self.button_hover,
                      cursor="hand2", command=lambda: self.send_command("M")).place(x=right_x - 100, y=extra_y + 50)

        ctk.CTkButton(self, text="🐢 DEFAULT SPEED", width=140, height=38,
                      font=("Segoe UI", 12), fg_color=self.accent_color,
                      text_color=self.text_white, hover_color=self.button_hover,
                      cursor="hand2", command=lambda: self.send_command("D")).place(x=right_x + 30, y=extra_y + 50)

        self.blink_state = True
        self.status_blink = True
        self.update_sensor_values()
        self.blink_status_icon()


    def create_custom_menu(self):
        menu_frame = ctk.CTkFrame(self, fg_color="#242424", height=40)
        menu_frame.pack(fill="x", side="top")

        def open_github():
            webbrowser.open("https://github.com/Syntax-Surfer-1")

        def open_repo():
            webbrowser.open("https://github.com/Syntax-Surfer-1/Fire-Extinguish-Robot")

        def show_controls():
            controls_window = ctk.CTkToplevel(self)
            controls_window.title("Controls")
            ctk.CTkLabel(controls_window, text="WASD - Move\nEsc - Exit", text_color="white").pack(padx=20, pady=20)

        def show_about():
            messagebox.showinfo("About", "Created by Syntax-Surfer Using Python")

        def send_mail():
            webbrowser.open("mailto:yaxpatel6300@gmail.com")

        def exit_app():
            self.destroy()

        buttons = {
            "Info": open_repo,
            "GitHub": open_github,
            "Controls": show_controls,
            "About": show_about,
            "Help": send_mail,
            "Exit": exit_app
        }

        for name, command in buttons.items():
            btn = ctk.CTkButton(menu_frame, text=name, fg_color="#242424", hover_color="#2a2a3d",
                                text_color="white", corner_radius=0, width=60, command=command)
            btn.pack(side="left", padx=2, pady=5)

    def toggle_connection(self):
        if self.serial_connection is None:
            port = self.bluetooth_dropdown.get()
            if port and "Select" not in port and "No" not in port:
                try:
                    self.serial_connection = serial.Serial(port, 9600, timeout=1)
                    self.status_label.configure(text="Status: CONNECTED", text_color="#00FFAA")
                    self.status_icon.configure(text_color="#00FFAA")
                    self.connect_btn.configure(text="DISCONNECT")
                except Exception as e:
                    print(f"Connection failed: {e}")
            else:
                print("No valid port selected.")
        else:
            self.serial_connection.close()
            self.serial_connection = None
            self.status_label.configure(text="Status: NOT CONNECTED", text_color="#FF5F5F")
            self.status_icon.configure(text_color="#FF5F5F")
            self.connect_btn.configure(text="CONNECT")

    def send_command(self, command_char):
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.write(command_char.encode())
            print(f"Sent: {command_char}")
        else:
            print("Serial not connected.")

    def scan_devices(self):
        ports = serial.tools.list_ports.comports()
        available_ports = [port.device for port in ports]
        self.bluetooth_dropdown.configure(values=available_ports or ["No Devices Found"])
        self.bluetooth_dropdown.set("Select Device" if available_ports else "No Devices Found")

    def update_sensor_values(self):
        if self.serial_connection and self.serial_connection.in_waiting:
            try:
                raw = self.serial_connection.readline()
                line = raw.decode(errors='ignore').strip()
                print(f"Received: {line}")

                values = line.split(',')
                if len(values) in [3, 6]:
                    for i in range(3):
                        val = values[i].strip()
                        if val.isdigit():
                            dist = int(val)
                            self.sensor_values[i].configure(text=f"{dist} cm")
                            color = self.text_alert if (dist < 10 and self.blink_state) else self.text_white
                            self.sensor_values[i].configure(text_color=color)
                        else:
                            self.sensor_values[i].configure(text="Err", text_color="red")

                    if len(values) == 6:
                        for i in range(3, 6):
                            val = values[i].strip()
                            if val.isdigit():
                                state = int(val)
                                status = "Detected" if state == 1 else "Not Detected"
                                color = self.text_alert if (state == 1 and self.blink_state) else self.text_white
                                self.sensor_values[i].configure(text=status, text_color=color)
                            else:
                                self.sensor_values[i].configure(text="Err", text_color="red")
                    else:
                        for i in range(3, 6):
                            self.sensor_values[i].configure(text="N/A", text_color=self.text_white)
                else:
                    print("Invalid data length. Ignored.")
            except Exception as e:
                print(f"Error parsing: {e}")
        self.blink_state = not self.blink_state
        self.after(100, self.update_sensor_values)

    def blink_status_icon(self):
        if "CONNECTED" in self.status_label.cget("text"):
            self.status_icon.configure(text_color="#00FFAA" if self.status_blink else self.bg_color)
            self.status_blink = not self.status_blink
        else:
            self.status_icon.configure(text_color="#FF5F5F")
        self.after(600, self.blink_status_icon)

    def key_pressed(self, event):
        key = event.char.lower() if event.char else ""
        print(f"Key pressed: {repr(key)}")

        if key == "w":
            self.send_command('F')
        elif key == "s":
            self.send_command('B')
        elif key == "a":
            self.send_command('L')
        elif key == "d":
            self.send_command('R')
        elif key == "m":
            self.toggle_state = not self.toggle_state
            self.send_command('M' if self.toggle_state else 'A')
        elif key == "k":
            self.send_command('D')
        elif key == "i":
            self.send_command('W')
        elif key == "l":
            self.toggle_connection()
        elif key == "j":
            self.send_command('M')
        elif event.keysym == "space":
            self.send_command('S')

    def switch_toggled(self):
        mode = self.auto_mode_switch.get()
        print(f"Mode switched to: {mode}")
        self.send_command('A' if mode == 'off' else 'M')

    def move_robot(self, symbol):
        command_map = {
            "↑": "F",
            "↓": "B",
            "←": "L",
            "→": "R",
            "⏹": "S"
        }
        if symbol in command_map:
            self.send_command(command_map[symbol])

if __name__ == "__main__":
    app = SensorDashboard()
    app.mainloop() 
