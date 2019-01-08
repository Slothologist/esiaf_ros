# ros imports
import esiaf_ros.msg as esiaf_msg

# util imports
from enum import Enum

# First some Enums

class Bitrate(Enum):
    """Enum that defines the available bitrates"""
    BIT_8 = 8
    BIT_16 = 16
    BIT_24 = 24
    BIT_32 = 32


class Endian(Enum):
    """Enum that defines the available endian types"""
    LE = 'LE'
    BE = 'BE'


class AudioRate(Enum):
    """Endian that defines the available Audiorates"""
    RATE_8000 = 8
    RATE_16000 = 16
    RATE_32000 = 32
    RATE_44100 = 44
    RATE_48000 = 48
    RATE_96000 = 96


# Then some utilities classes

class AudioFormat:
    rate = None
    bitrate = None
    channels = None
    endian = None

    def __init__(self,
                 rate,
                 bitrate,
                 channels,
                 endian):
        try:
            self.rate = AudioRate(rate)
            self.bitrate = Bitrate(bitrate)
            self.channels = channels
            self.endian = Endian(endian)
        except Exception as e:
            print('AudioFormat was created without proper parameters! ' + e.message)

    def to_ros(self):
        """
        Creates a representation of this AudioFormat in the form of a ros message object
        :return: a esiaf_ros.msg.AudioFormat object
        """
        rosified = esiaf_msg.AudioFormat()
        rosified.rate = self.rate.value
        rosified.bitrate = self.bitrate.value
        rosified.channels = self.channels
        rosified.endian = self.endian.value
        return rosified


class AudioTopicInfo:
    topic = None
    allowedFormats = None

    def __init__(self,
                 audioTopicInfo):
        self.topic = audioTopicInfo.topic
        self.allowedFormats = [AudioFormat(audioFormat.rate,
                                           audioFormat.bitrate,
                                           audioFormat.channels,
                                           audioFormat.endian)
                               for audioFormat in audioTopicInfo.allowedFormats]

    def to_ros(self):
        return None

