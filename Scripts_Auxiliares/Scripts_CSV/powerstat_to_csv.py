#!/usr/bin/env python3
# filepath: powerstat_to_csv.py

import sys
import os
import re
import csv
from datetime import datetime

def parse_powerstat_log(filepath):
    """
    Parse a powerstat.log file and extract power measurement data
    """
    data = []
    header = []
    
    with open(filepath, 'r') as file:
        # Skip until we find data lines (they start with timestamps like 12:34:56)
        started = False
        for line in file:
            # Skip empty lines
            if not line.strip():
                continue
                
            # Look for the header line (contains "Time", "User", "Nice", etc.)
            if "Time" in line and "User" in line and "Nice" in line and "Sys" in line:
                header = line.strip().split()
                continue
                
            # Parse data lines that start with a timestamp
            if re.match(r'\d{2}:\d{2}:\d{2}', line):
                started = True
                values = line.strip().split()
                if len(values) > 1:  # Ensure we have actual data
                    data.append(values)
            
            # Stop when we hit the summary section
            if started and "--------" in line:
                break
                
    return header, data

def save_to_csv(header, data, output_path):
    """
    Save parsed data to a CSV file
    """
    with open(output_path, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        
        # Write the header
        if header:
            writer.writerow(header)
            
        # Write data rows
        for row in data:
            writer.writerow(row)

def process_file(input_path):
    """
    Process a single powerstat log file
    """
    print(f"Processing {input_path}...")
    
    # Create output filename
    base_name = os.path.basename(input_path)
    dir_name = os.path.dirname(input_path)
    output_file = os.path.splitext(base_name)[0] + ".csv"
    output_path = os.path.join(dir_name, output_file)
    
    # Parse and convert
    header, data = parse_powerstat_log(input_path)
    
    if not data:
        print(f"Error: No valid data found in {input_path}")
        return False
        
    save_to_csv(header, data, output_path)
    print(f"Successfully created {output_path}")
    return True

def main():
    """
    Main function to process powerstat log files
    """
    if len(sys.argv) < 2:
        print("Usage: python powerstat_to_csv.py file1.log [file2.log file3.log ...]")
        sys.exit(1)
    
    success_count = 0
    
    # Process each file provided as argument
    for filepath in sys.argv[1:]:
        if not os.path.exists(filepath):
            print(f"Error: File {filepath} does not exist")
            continue
            
        if process_file(filepath):
            success_count += 1
    
    print(f"\nProcessed {success_count} out of {len(sys.argv) - 1} files successfully")

if __name__ == "__main__":
    main()