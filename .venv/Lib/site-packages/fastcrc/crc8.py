"""
Compute a CRC-16 checksum of data.
"""
from typing import Optional

from .fastcrc import crc_8_autosar as _crc_8_autosar
from .fastcrc import crc_8_bluetooth as _crc_8_bluetooth
from .fastcrc import crc_8_cdma2000 as _crc_8_cdma2000
from .fastcrc import crc_8_darc as _crc_8_darc
from .fastcrc import crc_8_dvb_s2 as _crc_8_dvb_s2
from .fastcrc import crc_8_gsm_a as _crc_8_gsm_a
from .fastcrc import crc_8_gsm_b as _crc_8_gsm_b
from .fastcrc import crc_8_i_432_1 as _crc_8_i_432_1
from .fastcrc import crc_8_i_code as _crc_8_i_code
from .fastcrc import crc_8_lte as _crc_8_lte
from .fastcrc import crc_8_maxim_dow as _crc_8_maxim_dow
from .fastcrc import crc_8_mifare_mad as _crc_8_mifare_mad
from .fastcrc import crc_8_nrsc_5 as _crc_8_nrsc_5
from .fastcrc import crc_8_opensafety as _crc_8_opensafety
from .fastcrc import crc_8_rohc as _crc_8_rohc
from .fastcrc import crc_8_sae_j1850 as _crc_8_sae_j1850
from .fastcrc import crc_8_smbus as _crc_8_smbus
from .fastcrc import crc_8_tech_3250 as _crc_8_tech_3250
from .fastcrc import crc_8_wcdma as _crc_8_wcdma

__always_supported = (
    "autosar",
    "bluetooth",
    "cdma2000",
    "darc",
    "dvb_s2",
    "gsm_a",
    "gsm_b",
    "i_432_1",
    "i_code",
    "lte",
    "maxim_dow",
    "mifare_mad",
    "nrsc_5",
    "opensafety",
    "rohc",
    "sae_j1850",
    "smbus",
    "tech_3250",
    "wcdma",
)

algorithms_guaranteed = set(__always_supported)
algorithms_available = set(__always_supported)

def autosar(data: bytes, initial: Optional[int] = None) -> int:
    """
    Compute a CRC-8 checksum of data with the `autosar` algorithm.

    Algorithm parameters:
        - poly: 0x2f
        - init: 0xff
        - xorout: 0xff
        - refin: False
        - refout: False

    :param bytes data: The data to be computed
    :param Optional[int] initial: The optional starting value of the checksum
    :return: The checksum
    :rtype: int
    :raises TypeError: if the data is not a bytes-like object
    """
    return _crc_8_autosar(data, initial)


def bluetooth(data: bytes, initial: Optional[int] = None) -> int:
    """
    Compute a CRC-8 checksum of data with the `bluetooth` algorithm.

    Algorithm parameters:
        - poly: 0xa7
        - init: 0x00
        - xorout: 0x00
        - refin: True
        - refout: True

    :param bytes data: The data to be computed
    :param Optional[int] initial: The optional starting value of the checksum
    :return: The checksum
    :rtype: int
    :raises TypeError: if the data is not a bytes-like object
    """
    return _crc_8_bluetooth(data, initial)


def cdma2000(data: bytes, initial: Optional[int] = None) -> int:
    """
    Compute a CRC-8 checksum of data with the `cdma2000` algorithm.

    Algorithm parameters:
        - poly: 0x9b
        - init: 0xff
        - xorout: 0x00
        - refin: False
        - refout: False

    :param bytes data: The data to be computed
    :param Optional[int] initial: The optional starting value of the checksum
    :return: The checksum
    :rtype: int
    :raises TypeError: if the data is not a bytes-like object
    """
    return _crc_8_cdma2000(data, initial)


def darc(data: bytes, initial: Optional[int] = None) -> int:
    """
    Compute a CRC-8 checksum of data with the `darc` algorithm.

    Algorithm parameters:
        - poly: 0x39
        - init: 0x00
        - xorout: 0x00
        - refin: True
        - refout: True

    :param bytes data: The data to be computed
    :param Optional[int] initial: The optional starting value of the checksum
    :return: The checksum
    :rtype: int
    :raises TypeError: if the data is not a bytes-like object
    """
    return _crc_8_darc(data, initial)


def dvb_s2(data: bytes, initial: Optional[int] = None) -> int:
    """
    Compute a CRC-8 checksum of data with the `dvb_s2` algorithm.

    Algorithm parameters:
        - poly: 0xd5
        - init: 0x00
        - xorout: 0x00
        - refin: False
        - refout: False

    :param bytes data: The data to be computed
    :param Optional[int] initial: The optional starting value of the checksum
    :return: The checksum
    :rtype: int
    :raises TypeError: if the data is not a bytes-like object
    """
    return _crc_8_dvb_s2(data, initial)


def gsm_a(data: bytes, initial: Optional[int] = None) -> int:
    """
    Compute a CRC-8 checksum of data with the `gsm_a` algorithm.

    Algorithm parameters:
        - poly: 0x1d
        - init: 0x00
        - xorout: 0x00
        - refin: False
        - refout: False

    :param bytes data: The data to be computed
    :param Optional[int] initial: The optional starting value of the checksum
    :return: The checksum
    :rtype: int
    :raises TypeError: if the data is not a bytes-like object
    """
    return _crc_8_gsm_a(data, initial)


def gsm_b(data: bytes, initial: Optional[int] = None) -> int:
    """
    Compute a CRC-8 checksum of data with the `gsm_b` algorithm.

    Algorithm parameters:
        - poly: 0x49
        - init: 0x00
        - xorout: 0xff
        - refin: False
        - refout: False

    :param bytes data: The data to be computed
    :param Optional[int] initial: The optional starting value of the checksum
    :return: The checksum
    :rtype: int
    :raises TypeError: if the data is not a bytes-like object
    """
    return _crc_8_gsm_b(data, initial)


def i_432_1(data: bytes, initial: Optional[int] = None) -> int:
    """
    Compute a CRC-8 checksum of data with the `i_432_1` algorithm.

    Algorithm parameters:
        - poly: 0x07
        - init: 0x00
        - xorout: 0x55
        - refin: False
        - refout: False

    :param bytes data: The data to be computed
    :param Optional[int] initial: The optional starting value of the checksum
    :return: The checksum
    :rtype: int
    :raises TypeError: if the data is not a bytes-like object
    """
    return _crc_8_i_432_1(data, initial)


def i_code(data: bytes, initial: Optional[int] = None) -> int:
    """
    Compute a CRC-8 checksum of data with the `i_code` algorithm.

    Algorithm parameters:
        - poly: 0x1d
        - init: 0xfd
        - xorout: 0x00
        - refin: False
        - refout: False

    :param bytes data: The data to be computed
    :param Optional[int] initial: The optional starting value of the checksum
    :return: The checksum
    :rtype: int
    :raises TypeError: if the data is not a bytes-like object
    """
    return _crc_8_i_code(data, initial)


def lte(data: bytes, initial: Optional[int] = None) -> int:
    """
    Compute a CRC-8 checksum of data with the `lte` algorithm.

    Algorithm parameters:
        - poly: 0x9b
        - init: 0x00
        - xorout: 0x00
        - refin: False
        - refout: False

    :param bytes data: The data to be computed
    :param Optional[int] initial: The optional starting value of the checksum
    :return: The checksum
    :rtype: int
    :raises TypeError: if the data is not a bytes-like object
    """
    return _crc_8_lte(data, initial)


def maxim_dow(data: bytes, initial: Optional[int] = None) -> int:
    """
    Compute a CRC-8 checksum of data with the `maxim_dow` algorithm.

    Algorithm parameters:
        - poly: 0x31
        - init: 0x00
        - xorout: 0x00
        - refin: True
        - refout: True

    :param bytes data: The data to be computed
    :param Optional[int] initial: The optional starting value of the checksum
    :return: The checksum
    :rtype: int
    :raises TypeError: if the data is not a bytes-like object
    """
    return _crc_8_maxim_dow(data, initial)


def mifare_mad(data: bytes, initial: Optional[int] = None) -> int:
    """
    Compute a CRC-8 checksum of data with the `mifare_mad` algorithm.

    Algorithm parameters:
        - poly: 0x1d
        - init: 0xc7
        - xorout: 0x00
        - refin: False
        - refout: False

    :param bytes data: The data to be computed
    :param Optional[int] initial: The optional starting value of the checksum
    :return: The checksum
    :rtype: int
    :raises TypeError: if the data is not a bytes-like object
    """
    return _crc_8_mifare_mad(data, initial)


def nrsc_5(data: bytes, initial: Optional[int] = None) -> int:
    """
    Compute a CRC-8 checksum of data with the `nrsc_5` algorithm.

    Algorithm parameters:
        - poly: 0x31
        - init: 0xff
        - xorout: 0x00
        - refin: False
        - refout: False

    :param bytes data: The data to be computed
    :param Optional[int] initial: The optional starting value of the checksum
    :return: The checksum
    :rtype: int
    :raises TypeError: if the data is not a bytes-like object
    """
    return _crc_8_nrsc_5(data, initial)


def opensafety(data: bytes, initial: Optional[int] = None) -> int:
    """
    Compute a CRC-8 checksum of data with the `opensafety` algorithm.

    Algorithm parameters:
        - poly: 0x2f
        - init: 0x00
        - xorout: 0x00
        - refin: False
        - refout: False

    :param bytes data: The data to be computed
    :param Optional[int] initial: The optional starting value of the checksum
    :return: The checksum
    :rtype: int
    :raises TypeError: if the data is not a bytes-like object
    """
    return _crc_8_opensafety(data, initial)


def rohc(data: bytes, initial: Optional[int] = None) -> int:
    """
    Compute a CRC-8 checksum of data with the `rohc` algorithm.

    Algorithm parameters:
        - poly: 0x07
        - init: 0xff
        - xorout: 0x00
        - refin: True
        - refout: True

    :param bytes data: The data to be computed
    :param Optional[int] initial: The optional starting value of the checksum
    :return: The checksum
    :rtype: int
    :raises TypeError: if the data is not a bytes-like object
    """
    return _crc_8_rohc(data, initial)


def sae_j1850(data: bytes, initial: Optional[int] = None) -> int:
    """
    Compute a CRC-8 checksum of data with the `sae_j1850` algorithm.

    Algorithm parameters:
        - poly: 0x1d
        - init: 0xff
        - xorout: 0xff
        - refin: False
        - refout: False

    :param bytes data: The data to be computed
    :param Optional[int] initial: The optional starting value of the checksum
    :return: The checksum
    :rtype: int
    :raises TypeError: if the data is not a bytes-like object
    """
    return _crc_8_sae_j1850(data, initial)


def smbus(data: bytes, initial: Optional[int] = None) -> int:
    """
    Compute a CRC-8 checksum of data with the `smbus` algorithm.

    Algorithm parameters:
        - poly: 0x07
        - init: 0x00
        - xorout: 0x00
        - refin: False
        - refout: False

    :param bytes data: The data to be computed
    :param Optional[int] initial: The optional starting value of the checksum
    :return: The checksum
    :rtype: int
    :raises TypeError: if the data is not a bytes-like object
    """
    return _crc_8_smbus(data, initial)


def tech_3250(data: bytes, initial: Optional[int] = None) -> int:
    """
    Compute a CRC-8 checksum of data with the `tech_3250` algorithm.

    Algorithm parameters:
        - poly: 0x1d
        - init: 0xff
        - xorout: 0x00
        - refin: True
        - refout: True

    :param bytes data: The data to be computed
    :param Optional[int] initial: The optional starting value of the checksum
    :return: The checksum
    :rtype: int
    :raises TypeError: if the data is not a bytes-like object
    """
    return _crc_8_tech_3250(data, initial)


def wcdma(data: bytes, initial: Optional[int] = None) -> int:
    """
    Compute a CRC-8 checksum of data with the `wcdma` algorithm.

    Algorithm parameters:
        - poly: 0x9b
        - init: 0x00
        - xorout: 0x00
        - refin: True
        - refout: True

    :param bytes data: The data to be computed
    :param Optional[int] initial: The optional starting value of the checksum
    :return: The checksum
    :rtype: int
    :raises TypeError: if the data is not a bytes-like object
    """
    return _crc_8_wcdma(data, initial)