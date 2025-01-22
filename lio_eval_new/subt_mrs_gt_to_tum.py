import csv

def convert_to_tum(input_csv, output_tum):
    """
    Converts a CSV file with timestamp (nsec), x, y, z, qx, qy, qz, qw to TUM format.

    Parameters:
        input_csv (str): Path to the input CSV file.
        output_tum (str): Path to the output TUM file.
    """
    try:
        with open(input_csv, 'r') as csv_file, open(output_tum, 'w') as tum_file:
            reader = csv.reader(csv_file)

            # Skip the header
            next(reader)

            for row in reader:
                timestamp_nsec = float(row[0])
                timestamp_sec = timestamp_nsec * 1e-9  # Convert nsec to sec

                x, y, z = row[1:4]
                qx, qy, qz, qw = row[4:]

                # Write to TUM file
                tum_file.write(f"{timestamp_sec:.9f} {x} {y} {z} {qx} {qy} {qz} {qw}\n")

        print(f"Conversion complete. Output saved to {output_tum}")
    except Exception as e:
        print(f"An error occurred: {e}")

# Example usage
input_csv_path = 'sensor fusion challenges/SubT_MRS_Hawkins_Multi_Floor_LegRobot/ground_truth_path.csv'
output_tum_path = 'sensor fusion challenges/SubT_MRS_Hawkins_Multi_Floor_LegRobot/tum_gt.csv'
convert_to_tum(input_csv_path, output_tum_path)

