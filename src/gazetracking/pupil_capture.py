#!/usr/bin/env python

import zmq
import time


class PupilCapture():
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    logger = None

    def setup(self, log):

        # setting IP
        self.socket.connect('tcp://127.0.0.1:50020')
        self.logger = log

        # Not sure this is necessary
        #socket.send('T 0.0') #set timebase to 0.0

        # Test connection speed
        t = time.time()
        self.socket.send('t') # ask for timestamp; ignoring for now
        self.socket.recv()
        delay = time.time()-t

        self.logger.info("Pupil remote controller connected, command time delay: " + str(delay))

    def start(self):
        self.socket.send('R') #start recording with specified name
        self.socket.recv()

    def stop(self):
        self.socket.send('r') # stop recording
        self.socket.recv()