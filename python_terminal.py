#!/usr/bin/env python

import rospy
import time
import subprocess
import os, sys, signal

def terminate_process_and_children(p):
    ps_command = subprocess.Popen("ps -o pid --ppid %d --noheaders" % p.pid, shell=True, stdout=subprocess.PIPE)
    ps_output = ps_command.stdout.read()
    retcode = ps_command.wait()
    assert retcode == 0, "ps command returned %d" % retcode
    for pid_str in ps_output.split("\n")[:-1]:
        os.kill(int(pid_str), signal.SIGINT)
    p.terminate()

if __name__ == '__main__':
    process = subprocess.Popen(['rosrun', 'kdarpa2022', 'record.sh'])
    time.sleep(3)
    terminate_process_and_children(process)