# cython: annotation_typing = False
import datetime
import logging
from logging import LogRecord

import colorlog
import requests

DISABLE_LIST = ["__main__"]


class SereactLogger(logging.Logger):
    def __init__(self, name: str = "msdirect_interface", disable_console_warnings: bool = False) -> None:
        super().__init__(name)

        # Set level to DEBUG to log all levels
        debugger_level = logging.DEBUG
        self.setLevel(debugger_level)

        # Create a file handler
        # file_handler = logging.FileHandler("sereact_system.log")
        # file_handler.setLevel(debugger_level)

        # Create a console handler
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.DEBUG)

        # Create a color formatter
        console_formatter = colorlog.ColoredFormatter('%(log_color)s%(levelname)s | %(process)s | %(name)s | %(asctime)s | %(message)s',
                                                      log_colors={
                                                          'DEBUG': 'cyan',
                                                          'INFO': 'green',
                                                          'WARNING': 'yellow',
                                                          'ERROR': 'red',
                                                      },
                                                      reset=True,
                                                      style='%',
                                                      datefmt='%Y-%m-%d %H:%M:%S'
                                                      )
        console_handler.setFormatter(console_formatter)
        self.addHandler(console_handler)

        # Suppress warning messages only for the console handler
        if disable_console_warnings:
            console_handler.addFilter(
                lambda record: record.levelno != logging.WARNING)

        # file_formatter = logging.Formatter(
        #     '%(levelname)s | %(process)s | %(name)s | %(asctime)s | %(message)s', datefmt='%Y-%m-%d %H:%M:%S')
        # file_handler.setFormatter(file_formatter)
        # self.addHandler(file_handler)

        if name in DISABLE_LIST:
            for handler in self.handlers:
                if isinstance(handler, logging.StreamHandler):
                    self.removeHandler(handler)