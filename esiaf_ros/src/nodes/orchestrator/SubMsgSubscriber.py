# ros imports
import rospy

# msg imports
from esiaf_ros.msg import *




class SubMsgSubscriber:
    DESIGNATION_DICT = {
        1: ('VAD', esiaf_ros.msg.VADInfo),
        2: ('SpeechRec', esiaf_ros.msg.SpeechInfo),
        3: ('SSL', esiaf_ros.msg.SSLInfo),
        4: ('Gender', esiaf_ros.msg.GenderInfo),
        5: ('Emotion', esiaf_ros.msg.EmotionInfo),
        6: ('VoiceId', esiaf_ros.msg.VoiceIdInfo)
    }

    subscriber = None

    last_msgs = []

    def __init__(self,
                 name,
                 designation):
        if 0 < designation < 7:
            self.subscriber = rospy.Subscriber(name + '/' + SubMsgSubscriber.DESIGNATION_DICT[designation][0],
                                           SubMsgSubscriber.DESIGNATION_DICT[designation][1],
                                           self.callback)

    def callback(self, msg):
        self.last_msgs.append(msg)