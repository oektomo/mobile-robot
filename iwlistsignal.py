#!/usr/bin/env python

import sys
def get_name(cell):
    return matching_line(cell,"ESSID:")[1:-1]

def get_channel(cell):
    return matching_line(cell,"Signal level=")

rules={"Name":get_name,
       "Channel":get_channel,
       }

def parse_cell(cell):
    """Applies the rules to the bunch of text describing a cell and returns the
    corresponding dictionary"""
    parsed_cell={}
    for key in rules:
        rule=rules[key]
        parsed_cell.update({key:rule(cell)})
    return parsed_cell

def main():
    """Pretty prints the output of iwlist scan into a table"""
    cells=[[]]
    parsed_cells=[]

    for line in sys.stdin:
        cell_line = match(line,"Cell ")
        if cell_line != None:
            cells.append([])
            line = cell_line[-27:]
        cells[-1].append(line.rstrip())

    cells=cells[1:]

    for cell in cells:
        parsed_cells.append(parse_cell(cell))

main()
