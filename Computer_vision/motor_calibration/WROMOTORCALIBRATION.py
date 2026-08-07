import tkinter as tk
from tkinter import ttk
import serial
import serial.tools.list_ports

BAUD = 115200

ser = None

commands = [
    "ROTATE_TO",
    "SHOULDER_TO",
    "ELBOW_TO",
    "TILT_TO",
    "WRIST_TO",
    "GRIP"
]

slider_names = [
    "Base",
    "Shoulder",
    "Elbow",
    "Wrist Tilt",
    "Wrist Rotate",
    "Gripper"
]


# ------------------------------
# Serial Functions
# ------------------------------

def connect():
    global ser

    if ser and ser.is_open:
        return

    try:
        ser = serial.Serial(port_combo.get(), BAUD, timeout=1)
        status.config(text="Connected", fg="green")
    except Exception as e:
        status.config(text=str(e), fg="red")


def disconnect():
    global ser

    if ser:
        ser.close()

    status.config(text="Disconnected", fg="red")


def send(command):
    if ser and ser.is_open:
        ser.write((command + "\n").encode())


# ------------------------------
# Slider Callback
# ------------------------------

def slider_changed(index, value):

    angle = int(float(value))

    angle_labels[index]["text"] = str(angle)

    send(f"{commands[index]}:{angle}")


# ------------------------------
# Buttons
# ------------------------------

def home():

    homes = [90,90,90,90,90,30]

    for i,v in enumerate(homes):
        sliders[i].set(v)


def stop():
    send("STOP")


def ping():
    send("PING")


# ------------------------------
# GUI
# ------------------------------

root = tk.Tk()

root.title("Artefact Guardian Servo Tester")

root.geometry("600x550")

frame = tk.Frame(root)
frame.pack(pady=10)

ports = [p.device for p in serial.tools.list_ports.comports()]

port_combo = ttk.Combobox(frame, values=ports, width=15)

if ports:
    port_combo.current(0)

port_combo.grid(row=0,column=0,padx=5)

tk.Button(frame,text="Connect",command=connect).grid(row=0,column=1)

tk.Button(frame,text="Disconnect",command=disconnect).grid(row=0,column=2)

status = tk.Label(frame,text="Disconnected",fg="red")
status.grid(row=0,column=3,padx=10)

sliders=[]
angle_labels=[]

for i,name in enumerate(slider_names):

    tk.Label(root,text=name,font=("Arial",12,"bold")).pack()

    s=tk.Scale(root,
               from_=0,
               to=180,
               orient="horizontal",
               length=450,
               command=lambda value,i=i: slider_changed(i,value))

    s.set(90)

    if i==5:
        s.set(30)

    s.pack()

    sliders.append(s)

    l=tk.Label(root,text=str(s.get()),font=("Arial",10))
    l.pack()

    angle_labels.append(l)

button_frame=tk.Frame(root)
button_frame.pack(pady=15)

tk.Button(button_frame,
          text="HOME",
          width=12,
          command=home).grid(row=0,column=0,padx=5)

tk.Button(button_frame,
          text="STOP",
          width=12,
          bg="red",
          fg="white",
          command=stop).grid(row=0,column=1,padx=5)

tk.Button(button_frame,
          text="PING",
          width=12,
          command=ping).grid(row=0,column=2,padx=5)

root.mainloop()