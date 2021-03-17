import utime
# import logging
# import copy
import faulthandler
from datetime import datetime, timedelta

from DW1000 import DW1000
import DW1000Constants as C
import DW1000Register
import MAC
import config


class Node:
    """
    Super class for tag and anchor

    Attributes:
        dw1000 (DW1000): DW1000 device object
        timeout: Current time to check for timeouts
        timeout_old: Last timestamp for valid activity
        timeout_limit: Maximum time between timeout and timeout_old
        timeouts: Number of timeouts that occurred
        cb_rxfcg: Callback on good frame reception
        cb_txfrs: Callback after frame send
        cb_rxrfto: Callback after receiver timeout
        cb_rxerr: Callback after receiver error
        cb_irq_while: Callback at the beginning of the interruptCB while loop
        cb_reset: Callback if a timeout occurs
        enableRx: Enable receiver at end of interruptCB
        status: Temporary stores a copy of the status register
        message: Stores last message
        header: Storess header of last message
    """

    def __init__(self):
        self.dw1000 = None

        self.timeout = utime.ticks_us()  # Current time
        self.timeout_old = utime.ticks_us()  # Last valid timestamp
        self.timeout_limit = 500000  # Maximum time between timeout and timeout_old
        self.timeouts = 0  # Stores number of timeouts

        # Callbacks to be set by subclasses
        self.cb_rxfcg = lambda: None
        self.cb_txfrs = lambda: None
        self.cb_rxrfto = lambda: None
        self.cb_rxerr = lambda: None

        self.cb_irq_while = lambda: None
        self.cb_reset = lambda: None

        self.enableRx = False  # Enable receiver at end of interruptCB?

        # self.status = None # Store status register of dw1000

        self.message = None  # Store last message
        self.header = None  # Store header of last message

    def setup(self):
        """
        Node setup

        Normally called by setup function inside subclass.
        """
        self.dw1000 = DW1000(config.pin_cs, config.pin_rst, config.pin_irq)
        self.dw1000.begin()
        # logging.info("DW1000 initialized")

        self.dw1000.generalConfiguration(config.eid, config.pan, C.MODE_STANDARD)
        self.dw1000.setAntennaDelay(C.ANTENNA_DELAY_RASPI)
        # self.dw1000.interruptCallback = self.interruptCB

        # logging.info(self.dw1000.getDeviceInfoString())

    def run(self):
        """
        Loop function of nodes

        This function can be called after creation and call to setup().
        It runs the main loop and checks for inactivity of the DW1000.
        If a timeout occurs, a custom callback can be called.
        """
        self.dw1000.newReceive()
        self.dw1000.startReceive()

        while True:
            # Currently not using irq
            # self.interruptCB()

            self.timeout = utime.ticks_us()
            dt = self.timeout - self.timeout_old
            if dt > self.timeout_limit:
                print("Node - time error")
                self.cb_reset()
                self.timeout_old = self.timeout
                self.timeouts += 1

    def stop(self):
        """
        Stops the node
        """
        self.dw1000.stop()