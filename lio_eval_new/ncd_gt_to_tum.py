import csv

def convert_to_tum(input_csv, output_tum):
    """
    Converts a CSV file with the format #sec,nsec,x,y,z,qx,qy,qz,qw to TUM format.

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
                sec, nsec, x, y, z, qx, qy, qz, qw = row

                # Combine seconds and nanoseconds into a single timestamp
                timestamp = float(sec) + float(nsec) * 1e-9

                # Write to the TUM file
                tum_file.write(f"{timestamp:.9f} {x} {y} {z} {qx} {qy} {qz} {qw}\n")

        print(f"Conversion complete. Output saved to {output_tum}")
    except Exception as e:
        print(f"An error occurred: {e}")

# Example usage
convert_to_tum('05_quad_with_dynamics/ground_truth/registered_poses.csv', '05_quad_with_dynamics/gt.csv')

