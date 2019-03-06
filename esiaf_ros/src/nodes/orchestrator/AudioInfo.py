# ros imports
import esiaf_ros.msg as esiaf_msg

# util imports
from enum import Enum

# First some Enums

class Bitrate(Enum):
    """Enum that defines the available bitrates"""
    BIT_INT_8_SIGNED = esiaf_msg.AudioTopicFormatConstants.BIT_INT_8_SIGNED
    BIT_INT_8_UNSIGNED = esiaf_msg.AudioTopicFormatConstants.BIT_INT_8_UNSIGNED
    BIT_INT_16_SIGNED = esiaf_msg.AudioTopicFormatConstants.BIT_INT_16_SIGNED
    BIT_INT_16_UNSIGNED = esiaf_msg.AudioTopicFormatConstants.BIT_INT_16_UNSIGNED
    BIT_INT_24_SIGNED = esiaf_msg.AudioTopicFormatConstants.BIT_INT_24_SIGNED
    BIT_INT_24_UNSIGNED = esiaf_msg.AudioTopicFormatConstants.BIT_INT_24_UNSIGNED
    BIT_INT_32_SIGNED = esiaf_msg.AudioTopicFormatConstants.BIT_INT_32_SIGNED
    BIT_INT_32_UNSIGNED = esiaf_msg.AudioTopicFormatConstants.BIT_INT_32_UNSIGNED
    BIT_FLOAT_32 = esiaf_msg.AudioTopicFormatConstants.BIT_FLOAT_32
    BIT_FLOAT_64 = esiaf_msg.AudioTopicFormatConstants.BIT_FLOAT_64


class Endian(Enum):
    """Enum that defines the available endian types"""
    LE = esiaf_msg.AudioTopicFormatConstants.LittleEndian
    BE = esiaf_msg.AudioTopicFormatConstants.BigEndian


class AudioRate(Enum):
    """Endian that defines the available Audiorates"""
    RATE_8000 = esiaf_msg.AudioTopicFormatConstants.RATE_8000
    RATE_16000 = esiaf_msg.AudioTopicFormatConstants.RATE_16000
    RATE_32000 = esiaf_msg.AudioTopicFormatConstants.RATE_32000
    RATE_44100 = esiaf_msg.AudioTopicFormatConstants.RATE_44100
    RATE_48000 = esiaf_msg.AudioTopicFormatConstants.RATE_48000
    RATE_96000 = esiaf_msg.AudioTopicFormatConstants.RATE_96000


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
    allowedFormat = None

    def __init__(self,
                 audioTopicInfo):
        self.topic = audioTopicInfo.topic
        self.allowedFormat = AudioFormat(audioTopicInfo.allowedFormat.rate,
                                           audioTopicInfo.allowedFormat.bitrate,
                                           audioTopicInfo.allowedFormat.channels,
                                           audioTopicInfo.allowedFormat.endian)

    def to_ros(self):
        rosified = esiaf_msg.AudioTopicInfo()
        rosified.topic = self.topic
        rosified.allowedFormat = self.allowedFormat.to_ros()
        return rosified

    def __str__(self):
        string = 'AudioTopicInfo[\n\t'
        string += 'topic: {}\n\t'.format(self.topic)
        string += 'allowedFormat: {}\n'.format(str(self.allowedFormat))
        string += ']'
        return string

