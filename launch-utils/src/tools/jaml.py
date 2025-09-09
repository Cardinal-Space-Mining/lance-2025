import json
import yaml
import tkinter as tk
from tkinter import filedialog

root = tk.Tk()
root.withdraw()

yaml_file = filedialog.askopenfilename(
    title = 'Select a file',
    filetypes = [("Text files", "*.csv"), ("All files", "*.*")] )

print(f"Attempting to load file '{yaml_file}'")

try:
    with open(yaml_file, 'r') as f:
        yaml_data = yaml.safe_load(f)
except:
    raise RuntimeError(f"Failed to read file '{yaml_file}'")

print(json.dumps(yaml_data, indent=4))
