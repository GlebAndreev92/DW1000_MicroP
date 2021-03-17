"""@package main
Main module responsible for startup.
This module reads the mode variable from config and
starts the responsible module.
It also sets the logging level for the application.
"""

import config
import tag

modes = {"tag": tag.main}

if __name__ == "__main__":

    try:
        modes[config.mode]()
    except KeyError:
        print("Unknown mode")