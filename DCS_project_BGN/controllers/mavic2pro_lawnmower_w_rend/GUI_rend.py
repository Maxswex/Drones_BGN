import tkinter as tk
from tkinter import ttk
import json

# Function to read the JSON file and check the vector
def read_vector_from_json():
    try:
        with open("rend_states.json", "r") as file:
            data = json.load(file)
            # Check if the vector has four ones
            return data["rend_states"] == [0, 0, 0, 0]
    except FileNotFoundError:
        return False

# Function to update the state of the "Rejoin" button
def update_rejoin_button_state():
    is_vector_valid = read_vector_from_json()
    button_rejoin["state"] = tk.NORMAL if is_vector_valid else tk.DISABLED
    # Schedule the next update after 1000 milliseconds (1 second)
    root.after(1000, update_rejoin_button_state)

# Function to be called when the "Rejoin" button is clicked
def rejoin_button_click():
    # Set all elements of the vector to "1"
    new_vector = [1, 1, 1, 1]
    with open("rend_states.json", "w") as file:
        json.dump({"rend_states": new_vector}, file)

    # Update the state of the "Rejoin" button
    update_rejoin_button_state()
    message.set("Rejoin button clicked. Vector set to [1, 1, 1, 1]")

# Function to be called when a button is clicked
def button_click(direction):
    message.set(f"Button clicked: {direction}")

# Create the main application window
root = tk.Tk()
root.title("BGN Project GUI")

# Create a StringVar to hold the message
message = tk.StringVar()

# Create labels to display the message
message_label = tk.Label(root, textvariable=message)
message_label.pack()

# Create a frame to hold the buttons
button_frame = ttk.Frame(root)
button_frame.pack(pady=10)

# Create buttons for different directions
button_up = ttk.Button(button_frame, text="Up", command=lambda: button_click("Up"))
button_rejoin = ttk.Button(button_frame, text="Rejoin", command=rejoin_button_click)
button_down = ttk.Button(button_frame, text="Down", command=lambda: button_click("Down"))

# Pack the buttons into the frame with appropriate padding
button_up.grid(row=0, column=0, padx=5)
button_rejoin.grid(row=0, column=1, pady=5)
button_down.grid(row=0, column=2, pady=5)

# Check and update the state of the "Rejoin" button
update_rejoin_button_state()

# Start the Tkinter main loop
root.mainloop()
