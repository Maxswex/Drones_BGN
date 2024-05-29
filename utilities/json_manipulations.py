import json

def read_json(file):
    # Read the JSON file
    try:
        with open(file, 'r', encoding='utf-8') as json_file:
            try:
                msg = json.load(json_file)
            except json.JSONDecodeError:
                print("JSON file is empty or not properly formatted.")
                msg = {}
    except FileNotFoundError:
        print("JSON file not found.")
        msg = {}
    return msg

def write_json(msg, file, indentation=4):
    # Write the JSON file
    try:
        with open(file, 'w', encoding='utf-8') as json_file:
            json.dump(msg, json_file, indent=indentation, ensure_ascii=False)
    except FileNotFoundError:
        print("JSON file not found.")

def update_json(file, new_data):
    # Update the JSON file with new data
    try:
        with open(file, 'r+') as json_file:
            try:
                data = json.load(json_file)
            except json.JSONDecodeError:
                data = {}
            data.update(new_data)
            json_file.seek(0)
            json.dump(data, json_file, indent=4)
            json_file.truncate()
    except FileNotFoundError:
        print("JSON file not found.")

def delete_field_json(file, field):
    # Delete a field from the JSON file
    try:
        with open(file, 'r+') as json_file:
            try:
                data = json.load(json_file)
            except json.JSONDecodeError:
                print("JSON file is empty or not properly formatted.")
                return
            if field in data:
                del data[field]
                json_file.seek(0)
                json.dump(data, json_file, indent=4)
                json_file.truncate()
            else:
                print(f"Field '{field}' not found in the JSON file.")
    except FileNotFoundError:
        print("JSON file not found.")

def update_field_json(file, field, new_value):
    # Update a specific field in the JSON file
    try:
        with open(file, 'r+') as json_file:
            try:
                data = json.load(json_file)
            except json.JSONDecodeError:
                print("JSON file is empty or not properly formatted.")
                return
            data[field] = new_value
            json_file.seek(0)
            json.dump(data, json_file, indent=4)
            json_file.truncate()
    except FileNotFoundError:
        print("JSON file not found.")

def add_field_json(file, field, value):
    # Add a new field to the JSON file
    try:
        with open(file, 'r+') as json_file:
            try:
                data = json.load(json_file)
            except json.JSONDecodeError:
                data = {}
            data[field] = value
            json_file.seek(0)
            json.dump(data, json_file, indent=4)
            json_file.truncate()
    except FileNotFoundError:
        print("JSON file not found.")

