"""@package tag
Tag part of SS-TWR system.

This module provides a tag class that ranges to some anchors.
The anchors are specified in the config module (config.py).
"""

# from http.server import HTTPServer, BaseHTTPRequestHandler
# import logging
# from threading import Thread

import DW1000Constants as C
import MAC
import config
import utime
import node

# from trilaterate import Trilaterator

page = (""
        "<!DOCTYPE html>"
        "<html>"
        "<head>"
        "<meta charset=\"UTF-8\">"
        "<meta http-equiv=\"refresh\" content=\"1\">"
        "</head>"
        "<body>"
        "<canvas id=\"uwbmap\" width=\"{}\" height=\"{}\"></canvas>"
        "<script>"
        "var c = document.getElementById(\"uwbmap\");"
        "var ctx = c.getContext(\"2d\");"
        "ctx.fillStyle = \"#000000\";"
        "ctx.fillRect({}, {}, 16, 16);"
        "ctx.fillRect({}, {}, 16, 16);"
        "ctx.fillRect({}, {}, 16, 16);"
        "ctx.fillRect({}, {}, 16, 16);"
        "ctx.fillStyle = \"#FF0000\";"
        "ctx.fillRect({}, {}, 16, 16);"
        "</script>"
        "</body>"
        "</html>"
        "")


def unixTimestamp():
    """
    Get a unix timestamp 

    The timestamp is returned as floating point with whole seconds before the comma.

    Returns:
        (float): Timestamp
    """
    return utime.time_ns()


class Tag(node.Node):
    """
    Tag class.

    Attributes:
        send: Number of send poll frames
        acked: Number of acked poll frames
        time_poll_send_ts: Timestamp of poll sending
        time_poll_recv_ts: Timestamp of poll receiving
        time_resp_send_ts: Timestamp of response sending
        time_resp_recv_ts: Timestamp of response receiving
        rxrfto_limit: Maximum number of receiver timeouts before a new poll frame is send
        rxrfto_count: Current number of receiver timeouts
        anchor_list: List of anchors used for ranging
        anchor_positions: List of anchor positions
        anchor_distances: Stores distances to anchors, cleared after each round
        anchor_idx: Index of current ranging anchor
        anchor_tries_limit: Maximum number of poll messages per anchor in one round
        anchor_tries: Current number of poll message to the current ranging anchor
        anchor_next: Flag signaling change to next anchor
        trilaterator: Trilaterator object for position calculation
        logfile: Logfile path
        http_thread: Thread handle for the web visualization server
        httpd: Web server
        http_position: Position that is published to the client
    """

    def __init__(self):
        super().__init__()

        # Statistics
        self.send = 0  # Number of send poll frames
        self.acked = 0  # Number of received acks for send polls

        self.time_poll_send_ts = None  # Timestamp of poll sending
        self.time_poll_recv_ts = None  # Timestamp of poll receiving
        self.time_resp_send_ts = None  # Timestamp of response sending
        self.time_resp_recv_ts = None  # Timestamp of response receiving

        self.rxrfto_limit = config.rxrfto_limit
        self.rxrfto_count = 0  # Current number of receive frame wait timeouts

        self.anchor_list = config.anchor_list  # list of anchors to range to
        self.anchor_positions = config.anchor_positions  # list of anchor positions
        self.anchor_distances = {}  # measured distances
        self.anchor_idx = 0  # current ranging anchor index
        self.anchor_tries_limit = config.tries_limit  # maximum number of poll message resends
        self.anchor_tries = 0  # current number of poll message sends
        self.anchor_next = False  # Indicate wanted change anchor_idx to next anchor_idx

        # self.trilaterator = Trilaterator() # Trilateror for position estimation

        # self.logfile = None  # logfile handle

        # Callbacks, see interruptCB
        self.cb_rxfcg = self.cb_rxfcg_
        self.cb_txfrs = self.cb_txfrs_
        self.cb_rxrfto = self.cb_rxrfto_
        self.cb_rxerr = self.cb_rxerr_

        self.cb_irq_while = self.updateAnchors
        self.cb_reset = self.cb_reset_

        self.http_thread = None
        self.httpd = None
        self.http_position = [0., 0., 0.]

    def setup(self):
        """ Tag setup 

        Call after creation of a tag. Set sysctrl and sysmask of DW1000.
        """
        super().setup()

        # self.logfile = open(config.logfile, "a")

        self.dw1000.syscfg.setBits(
            (C.DIS_STXP_BIT, C.FFEN_BIT, C.FFAA_BIT, C.FFAD_BIT, C.RXWTOE_BIT, C.AAT_BIT, C.RXAUTR_BIT), True)
        self.dw1000.writeRegister(self.dw1000.syscfg)

        # Set HSRBP to ICRBP for double buffering
        self.dw1000.disableDoubleBuffer()

        self.dw1000.setFrameWaitTimeout(40000)

        # Enable receiver buffer overrun detection, data frame receive, receive frame wait timeout
        self.dw1000.sysmask.clear()
        self.dw1000.sysmask.setBits((C.MRXOVRR_BIT, C.MRXFCG_BIT, C.MRXRFTO_BIT, C.MTXFRS_BIT), True)
        self.dw1000.writeRegister(self.dw1000.sysmask)

        self.dw1000.clearAllStatus()

      #  if config.webui_enable:
      #      self.http_thread = Thread(target=self.webserveFunc)
      #      self.http_thread.start()

    def stop(self):
        super().stop()



def main():
    tag = Tag()
    tag.setup()

    start = utime.time_ns()

    try:
        tag.run()
    except KeyboardInterrupt:
        tag.dw1000.stop()

    end = utime.time_ns()
    delta = end - start
    print('Delta', delta, '\n' 'Send', tag.send, '\n' 'Acked', tag.acked, '\n' 'Timeouts', tag.timeouts)


if __name__ == "__main__":
    main()
