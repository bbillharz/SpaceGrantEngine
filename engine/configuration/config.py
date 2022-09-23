import os
import sys


class Config:
    """
    Class for loading configuration files and editing their values dynamically at runtime.
    """

    def __init__(self, config_file: str):
        assert config_file.endswith(".json") or config_file.endswith(".ini")

        self._filename = config_file
