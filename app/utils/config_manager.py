import json
import os

class ConfigManager:
    def __init__(self, file_name = "config.json"):
        self.file_path = os.path.abspath(file_name)

    def load_settings(self):
        # load settings from the JSON file
        # return a dictionary with the settings, or an empty dict if file not found
        if not os.path.exists(self.file_path):
            return {}
        try:
            with open(self.file_path, 'r') as f:
                return json.load(f)
        except (json.JSONDecodeError, IOError):
        # return empty dict if file is corrupted or can't be read
                return {}
        
    def save_settings(self, settings_dict):
        """
        Saves the given dictionary of settings to the JSON file.
        """
        # --- DEBUGGING LINE ADDED ---
        print(f"Attempting to save config to: {self.file_path}")
        try:
            with open(self.file_path, 'w') as f:
                json.dump(settings_dict, f, indent=4)
            # --- DEBUGGING LINE ADDED ---
            print("Config file saved successfully.")
        except Exception as e:
            # Catch any potential error during file write
            print(f"Error saving config file: {e}")