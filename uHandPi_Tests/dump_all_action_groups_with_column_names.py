import sqlite3
import os

# Function to dump ActionGroup table data to a text file
def dump_action_group_to_text(db_filename):
    try:
        # Connect to the SQLite database
        conn = sqlite3.connect(db_filename)
        cursor = conn.cursor()

        # Fetch column names from the ActionGroup table
        cursor.execute("PRAGMA table_info(ActionGroup)")
        column_names = [column[1] for column in cursor.fetchall()]

        # Fetch all records from the ActionGroup table
        cursor.execute("SELECT * FROM ActionGroup")
        rows = cursor.fetchall()

        # Close the database connection
        conn.close()

        # Create a text file with the same name as the database file
        txt_filename = os.path.splitext(db_filename)[0] + ".txt"
        with open(txt_filename, 'w') as txt_file:
            # Write column names as the first line
            txt_file.write(",".join(column_names) + "\n")
            
            # Write data rows
            for row in rows:
                txt_file.write(",".join(map(str, row)) + "\n")

        print(f"Data dumped to {txt_filename}")

    except sqlite3.Error as e:
        print(f"Error: {e}")
    except Exception as ex:
        print(f"Exception: {ex}")

# Directory containing .d6a files
directory = '/Users/rajasp/Personal/fll/uHand_Pi/uHand_Pi/ActionGroups/'

# Loop through files in the directory
for filename in os.listdir(directory):
    if filename.endswith(".d6a"):
        db_file_path = os.path.join(directory, filename)
        dump_action_group_to_text(db_file_path)
