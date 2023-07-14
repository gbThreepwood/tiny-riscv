# Convert a file in Intel HEX format to hexadecimal byte values
# which can be read by $readmemh() in Verilog.
#
# The output file should have 4 bytes (one word) per line encoded
# as ASCII hexadecimal values
#
# E.g.:
# DEADBEEF
# B16B00B5
#

import argparse
from intelhex import IntelHex

def read_file():
    parser = argparse.ArgumentParser()
    parser.parse_args()

    print("File")

def write_mem_file(mem_file_name, mem_data):
    with open(mem_file_name, "w") as file:
        file.write(mem_data)

if __name__ == '__main__':
    read_file()
