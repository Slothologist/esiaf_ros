# ros imports
import rospy

# audio info imports
from AudioInfo import *
from esiaf_ros.msg import ChangedConfig, DeterminedTopicConfig
from SubMsgSubscriber import SubMsgSubscriber


class Node:
    """
    Class representing a ROS Node inside the esiaf_ros pipeline.
    """

    name = None
    designation = None
    subMsgSubscriber = None
    configPublisher = None

    allowedTopicsIn = None
    allowedTopicsOut = None

    actualTopicsIn = None
    actualTopicsOut = None

    def __init__(self, nodeinfo):
        self.name = nodeinfo.name
        self.designation = nodeinfo.designation
        self.configPublisher = rospy.Publisher('/esiaf_ros/' + self.name + '/changedConfig', ChangedConfig, queue_size=10)

        self.allowedTopicsIn = []
        self.allowedTopicsOut = []

        self.actualTopicsIn = []
        self.actualTopicsOut = []

        for allowedTopicIn in nodeinfo.inputTopics:
            self.allowedTopicsIn.append(AudioTopicInfo(allowedTopicIn))

        for allowedTopicOut in nodeinfo.outputTopics:
            self.allowedTopicsOut.append(AudioTopicInfo(allowedTopicOut))

        self.subMsgSubscriber = SubMsgSubscriber(self.name, self.designation)


    def update_config(self):
        """
        Sends out this node-representations momentary, actual configuration to the real node.
        :return: None
        """
        config = ChangedConfig()

        config.inputTopics = []
        for topic in self.actualTopicsIn:
            determinedConfig = DeterminedTopicConfig()
            determinedConfig.topic = topic[0]
            determinedConfig.determinedFormat = topic[1].to_ros()
            config.inputTopics.append(determinedConfig)

        config.outputTopics = []
        for topic in self.actualTopicsOut:
            determinedConfig = DeterminedTopicConfig()
            determinedConfig.topic = topic[0]
            determinedConfig.determinedFormat = topic[1].to_ros()
            config.outputTopics.append(determinedConfig)

        self.configPublisher.publish(config)
        rospy.logdebug('Providing new config for "' + self.name + '": ' + str(config))

    def bury(self):
        """
        Basically a destructor for dead nodes. Will kill their publishers and subscribers.
        :return:
        """
        self.configPublisher.unregister()
        self.subMsgSubscriber.subscriber.unregister()
        del self
