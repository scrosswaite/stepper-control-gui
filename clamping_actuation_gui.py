import tkinter as tk
from tkinter import ttk
import serial
import time

# Adjust COM port for your system
arduino = serial.Serial(port='COM7', baudrate=9600, timeout=1)
time.sleep(2)

def send_command(cmd):
    arduino.write((cmd + "\n").encode())

def step_forward():
    steps = step_size.get()
    send_command(f"STEP {steps}")

def step_backward():
    steps = step_size.get()
    send_command(f"STEP {-steps}")

def stop_motor():
    send_command("STOP")

# GUI setup
root = tk.Tk()
root.title("Stepper Motor Control")
root.geometry("400x250")
root.configure(bg="#2e2e2e")

# Title
title = tk.Label(root, text="Stepper Motor Controller", font=("Arial", 14, "bold"), fg="white", bg="#2e2e2e")
title.pack(pady=10)

# Step size slider
slider_frame = tk.Frame(root, bg="#2e2e2e")
slider_frame.pack(pady=10)

tk.Label(slider_frame, text="Step Size:", fg="white", bg="#2e2e2e").pack(side=tk.LEFT, padx=5)

step_size = tk.IntVar(value=50)
slider = ttk.Scale(slider_frame, from_=1, to=500, orient="horizontal", variable=step_size, length=200)
slider.pack(side=tk.LEFT)

# Buttons
button_frame = tk.Frame(root, bg="#2e2e2e")
button_frame.pack(pady=20)

btn_back = tk.Button(button_frame, text="⬅ Step Back", width=12, command=step_backward, bg="#444", fg="white")
btn_back.grid(row=0, column=0, padx=10)

btn_forward = tk.Button(button_frame, text="Step Forward ➡", width=12, command=step_forward, bg="#444", fg="white")
btn_forward.grid(row=0, column=1, padx=10)

btn_stop = tk.Button(root, text="⏹ STOP", width=15, command=stop_motor, bg="red", fg="white")
btn_stop.pack(pady=10)

root.mainloop()
