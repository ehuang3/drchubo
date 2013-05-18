#! /usr/bin/env python

import sys
import os
import copy

# debug toggle
debug = True

# Measures the statistics of gaits
class GaitStat():
    # Statistics dump file
    myfile = 'data/gait_stats.txt'
    # Config we are logging
    prev_cfg = {}
    curr_cfg = {}
    # Last keystroke
    prev_key = ''
    # State enums
    INIT = 1
    READY = 2
    LOG = 3
    state = INIT
    # Keyboard teleop names
    directions = {'u': "Turn Left Point", \
                  'i': "Forward", \
                  'o': "Turn Right Point", \
                  'j': "Side Left", \
                  'k': "Wonky Step", \
                  'l': "Side Right", \
                  'm': "Turn Left", \
                  ',': "Backward", \
                  '.': "Turn Right"}
    # Parameter names
    param_names = { "Forward Stride Length",
                    "Lateral Stride Length",
                    "Stride Duration",
                    "Walk Sequence Length",
                    "Stride Width",
                    "In Place Turn Size",
                    "Turn Radius" }
    
    def __init__(self):
        # Prepare the file path
        # print "__file__: %s" % __file__
        # print "sys.argv: %s" % sys.argv[0]

        a_f = os.path.abspath(__file__)
        a_s = os.path.abspath(sys.argv[0])

        # print "abs __file__: %s" % a_f
        # print "abs sys.argv: %s" % a_s

        __location__ = os.path.realpath( os.path.dirname(__file__) )
        __location__ = os.path.dirname( __location__ )
        
        self.myfile = os.path.join( __location__, self.myfile )

        # print "__location__: %s" % __location__
        print "logfile: %s" % self.myfile

        # Prepare state
        self.prev_key = 'm'
        self.prev_cfg = { "Forward Stride Length": {"value":0.4},
                          "Lateral Stride Length": {"value":0.15},
                          "Stride Duration": {"value":0.7},
                          "Walk Sequence Length": {"value":20},
                          "Stride Width": {"value":0.2},
                          "In Place Turn Size":{"value":0.1},
                          "Turn Radius": {"value":1} }

    def log_config(self, config):
        # Store this config to log
        self.curr_cfg = config

    def log_command(self, ch):
        ch = str(ch).lower()[0]
        fall = False

        # Transition on logging states
        if self.state == self.INIT:
            # We took some steps, get ready to log them
            if self.directions.has_key(ch):
                self.state = self.READY
        elif self.state == self.READY:
            # We took additional steps or reset
            # Log the previous steps
            if self.directions.has_key(ch):
                self.state = self.LOG
            elif ch == 'r':
                # We fell, but that's ok
                self.state = self.LOG
                fall = True
            elif ch == '!':
                # No fall, log anyways
                self.state = self.LOG
                fall = False
        # Do log
        if self.state == self.LOG:
            self._log(fall)

            self.state = self.READY
            # If we fell, wait for new steps to log
            if fall:
                self.state = self.INIT
            if ch == '!':
                self.stat = self.INIT # special case

        # Only log the previous _motion_ command
        if self.directions.has_key(ch):
            self.prev_key = ch
            self.prev_cfg = copy.deepcopy(self.curr_cfg)

    def log_header(self):
        header_str = ""
        # Gait parameters
        for p in self.param_names:
            header_str += p + ", "
        # Gait direction
        for i in self.directions.keys():
            header_str += self.directions[i] + ", "
        # Did we fall
        header_str += "Fall"
        
        return header_str

    def _log(self, fall):
        log_str = ""
        debug_str = "[GaitStat] Logging\n\t"
        # Gait parameters
        for p in self.param_names:
            debug_str += p + " := " + str(self.prev_cfg[p]["value"]) + "\n\t"
            log_str += str(self.prev_cfg[p]["value"]) + ", "
        # Gait direction
        for i in self.directions.keys():
            if self.prev_key == i:
                debug_str += self.directions[i] + " := " + str(1)
                log_str += str(1) + ", "
            else:
                log_str += str(0) + ", "
        # Did we fall
        if fall:
            debug_str += "\n\tFall := 1"
            log_str += str(1)
        else:
            log_str += str(0)

        if debug:
            print debug_str
            print log_str
        
        with open(self.myfile, 'a') as mystat:
            mystat.write(log_str + "\n")

if __name__ == '__main__':
    stat = GaitStat()
    
    stat.log_command('m')
    stat.log_command('r')
    stat.log_command('i')
    stat.log_command('i')

    print stat.log_header()
