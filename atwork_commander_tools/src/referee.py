#! /usr/bin/python3
from argparse import ArgumentParser
from pathlib import Path

import rosbag
from atwork_commander_msgs.msg import Object


rule_book_type = {

    # atwork
    11: "F20_20_B",
    12: "F20_20_G",
    13: "S40_40_B",
    14: "S40_40_G",
    15: "M20_100" ,
    16: "M20",
    17: "M30",
    18: "R20",

    # advanced
    20: "Axis2"      ,
    21: "Bearing2"   ,
    22: "Housing"    ,
    23: "Motor2"     ,
    24: "Spacer"     ,
    25: "Screwdriver",
    26: "Wrench"     ,
    27: "Drill"      ,
    28: "AllenKey"   ,

    # rockin
    31: "BEARING_BOX"  ,
    32: "BEARING"      ,
    33: "AXIS"         ,
    34: "DISTANCE_TUBE",
    35: "MOTOR"        ,

    # container
    40: "CONTAINER_RED",
    41: "CONTAINER_BLUE",

    # cavity
    50: "F20_20_H" ,
    51: "F20_20_V" ,
    52: "F20_20_F" ,
    53: "S40_40_H" ,
    54: "S40_40_V" ,
    55: "S40_40_F" ,
    56: "M20_H"    ,
    57: "M20_V"    ,
    58: "M20_F"    ,
    59: "M20_100_H",
    60: "M20_100_V",
    61: "M20_100_F",
    62: "M30_H"    ,
    63: "M30_V"    ,
    64: "M30_F"    ,
    65: "R20_H"    ,
    66: "R20_V"    ,
    67: "R20_F"    ,
}

def parse_arguments():
    parser = ArgumentParser()
    parser.add_argument("bag", type=Path, help="Bag file to extract task from")
    return parser.parse_args()

def log(ws, obj, place = None):
    workstation = f"{ws} | {rule_book_type[obj]:<15}"

    if place:
        workstation += f" in {rule_book_type[place].replace('CONTAINER_', '')}"
    
    print(workstation)

def main(args):
    bag = rosbag.Bag(args.bag.expanduser())

    print("Start State: ")
    for topic, msg, t in bag.read_messages(topics=["/atwork_commander/task"]):
        for workstation in msg.arena_start_state:
            if workstation.objects:
                for object in workstation.objects:
                    if object.object < 40:
                        log(workstation.name, object.object)
        break

    print()
    print("End State: ")
    for topic, msg, t in bag.read_messages(topics=["/atwork_commander/task"]):
        # print(msg)
        for workstation in msg.arena_target_state:
            if workstation.objects:
                for object in workstation.objects:
                    if object.object < 40:
                        log(workstation.name, object.object, object.target)
        break


    bag.close()

if __name__ == "__main__":
    main(args = parse_arguments())