"""
Compute a CRC-64 checksum of data.
"""
from typing import Optional

from .fastcrc import crc_64_ecma_182 as _crc_64_ecma_182
from .fastcrc import crc_64_go_iso as _crc_64_go_iso
from .fastcrc import crc_64_we as _crc_64_we
from .fastcrc import crc_64_xz as _crc_64_xz

__always_supported = ("ecma_182", "go_iso", "we", "xz")

algorithms_guaranteed = set(__always_supported)
algorithms_available = set(__always_supported)


def ecma_182(data: bytes, initial: Optional[int] = None) -> int:
    """
    Compute a CRC-64 checksum of data with the `ecma 182` algorithm.

    Algorithm parameters:
        - ploy: 0x42f0e1eba9ea3693
        - init: 0x0000000000000000
        - xorout: 0x0000000000000000
        - refin: False
        - refout: False

    :param bytes data: The data to be computed
    :param Optional[int] initial: The optional starting value of the checksum
    :return: The checksum
    :rtype: int
    :raises TypeError: if the data is not a bytes-like object
    """
    return _crc_64_ecma_182(data, initial)


def go_iso(data: bytes, initial: Optional[int] = None) -> int:
    """
    Compute a CRC-64 checksum of data with the `go iso` algorithm.

    Algorithm parameters:
        - ploy: 0x000000000000001b
        - init: 0xffffffffffffffff
        - xorout: 0xffffffffffffffff
        - refin: True
        - refout: True

    :param bytes data: The data to be computed
    :param Optional[int] initial: The optional starting value of the checksum
    :return: The checksum
    :rtype: int
    :raises TypeError: if the data is not a bytes-like object
    """
    return _crc_64_go_iso(data, initial)


def we(data: bytes, initial: Optional[int] = None) -> int:
    """
    Compute a CRC-64 checksum of data with the `we` algorithm.

    Algorithm parameters:
        - ploy: 0x42f0e1eba9ea3693
        - init: 0xffffffffffffffff
        - xorout: 0xffffffffffffffff
        - refin: False
        - refout: False

    :param bytes data: The data to be computed
    :param Optional[int] initial: The optional starting value of the checksum
    :return: The checksum
    :rtype: int
    :raises TypeError: if the data is not a bytes-like object
    """
    return _crc_64_we(data, initial)


def xz(data: bytes, initial: Optional[int] = None) -> int:
    """
    Compute a CRC-64 checksum of data with the `xz` algorithm.

    Algorithm parameters:
        - ploy: 0x42f0e1eba9ea3693
        - init: 0xffffffffffffffff
        - xorout: 0xffffffffffffffff
        - refin: True
        - refout: True

    :param bytes data: The data to be computed
    :param Optional[int] initial: The optional starting value of the checksum
    :return: The checksum
    :rtype: int
    :raises TypeError: if the data is not a bytes-like object
    """
    return _crc_64_xz(data, initial)
