#! /usr/bin/python3

import sys
import argparse

import rospy
from atwork_commander_msgs.msg import Task, Workstation, Object

def generate_task(arena_num, atwork_replacement):

    if atwork_replacement == "None":
        atwork_source = False
        atwork_assembly = False
    if atwork_replacement == "Source":
        atwork_source = True
        atwork_assembly = False
    if atwork_replacement == "All":
        atwork_source = True
        atwork_assembly = True

    task = Task()

    gray_brick = Object()
    gray_brick.object = 101
    blue_brick = Object()
    blue_brick.object = 102
    assembled = Object()
    assembled.object = 103
    m20 = Object()
    m20.object = 16
    m20_100 = Object()
    m20_100.object = 15
    S40_40B = Object()
    S40_40B.object = 13

    arena_1_ws = ["WS01", "WS03", "WS04", "WS05"]
    arena_2_ws = ["WS10", "WS08", "WS07", "WS06"]

    arenas = [arena_1_ws, arena_2_ws]
    arena = arenas[arena_num-1]

    ws1_start = Workstation()
    ws1_start.type = "10"
    ws1_start.name = arena[0]
    ws1_start.objects = [
        m20 if atwork_source else gray_brick
    ]

    ws2_start = Workstation()
    ws2_start.type = "10"
    ws2_start.name = arena[1]
    ws2_start.objects = [
        m20_100 if atwork_source else blue_brick
    ]

    ws3_start = Workstation()
    ws3_start.type = "10"
    ws3_start.name = arena[2]
    ws3_start.objects = [
        S40_40B if atwork_assembly else assembled
    ]

    ws4_start = Workstation()
    ws4_start.type = "10"
    ws4_start.name = arena[3]
    ws4_start.objects = []

    ws1_end = Workstation()
    ws1_end.type = "10"
    ws1_end.name = arena[0]
    ws1_end.objects = []

    ws2_end = Workstation()
    ws2_end.type = "10"
    ws2_end.name = arena[1]
    ws2_end.objects = []

    ws3_end = Workstation()
    ws3_end.type = "10"
    ws3_end.name = arena[2]
    ws3_end.objects = [
        m20 if atwork_source else gray_brick,
        m20_100 if atwork_source else blue_brick
    ]

    ws4_end = Workstation()
    ws4_end.type = "10"
    ws4_end.name = arena[3]
    ws4_end.objects = [
        S40_40B if atwork_assembly else assembled
    ]

    task.arena_start_state = [
        ws1_start, ws2_start, ws3_start, ws4_start
    ]

    task.arena_target_state = [
        ws1_end, ws2_end, ws3_end, ws4_end
    ]

    return task


arg_parser = argparse.ArgumentParser(description="Fake task generator for SML workshop")
arg_parser.add_argument('arena_number', type=str, choices=["1", "2"],
                        help='Arena for task execution')
arg_parser.add_argument('replacement', type=str, choices=["None", "Source", "All"],
                        help='Replace objects with atwork objects')

def main():
    rospy.init_node('fake_task')
    print(rospy.myargv())
    args = arg_parser.parse_args(rospy.myargv()[1:])
    pub = rospy.Publisher("atwork_commander/internal/task", Task, queue_size=1)
    rate = rospy.Rate(10)
    task = generate_task(int(args.arena_number), args.replacement)
    while not rospy.is_shutdown():
        rate.sleep()
        pub.publish(task)


if __name__ == "__main__":
    main()
