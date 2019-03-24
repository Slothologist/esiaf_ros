# ros imports
import rosnode
import rospy

# esiaf imports
from .node import Node
from .AudioInfo import AudioFormat as AudioFormatInternal
from esiaf_ros.msg import *
from esiaf_ros.srv import *

# threading imports
import threading
import time
import signal
import sys


class Orchestrator:
    """
    Class representing a esiaf_ros orchestrator. It will handle orchestration between the esiaf_ros nodes.
    This includes registering of nodes, determining their audio in- and output formats and the points in the audio flow
    where resampling is necessary.
    """

    active_nodes = []
    active_nodes_lock = threading.Lock()
    stopping_signal = False

    registerService = None

    def __init__(self,
                 remove_dead_rate=0.2
                 ):
        """
        Will initialise an orchestrator according to the config file found under the given path.
        :param path_to_config: may be relative or absolute
        """
        self.registerService = rospy.Service('/esiaf_ros/orchestrator/register', RegisterNode, self.register_node)

        # define loop function for removing dead nodes and start a thread calling it every X seconds
        def dead_loop(orc_instance, stop):
            while not stop():
                Orchestrator.remove_dead_nodes(orc_instance)
                time.sleep(1 / remove_dead_rate)
            sys.exit(0)

        t = threading.Thread(target=dead_loop, args=(self, lambda: self.stopping_signal))
        t.start()

        # create a signal handler so that sigint does not take forever to escalate and eventually kill the orchestrator
        def signal_handler(sig, frame):
            rospy.loginfo('Got SIGINT, shutting down!')
            self.stopping_signal = True
            t.join()
            sys.exit(0)

        signal.signal(signal.SIGINT, signal_handler)

    def remove_dead_nodes(self):
        """
        This function will remove nodes from the orchestrators active_nodes list which have shut down.
        Should be invoked in a regular interval.
        :return: None
        """
        momentary_nodenames = rosnode.get_node_names()

        with self.active_nodes_lock:
            def check_node_ping(name):
                try:
                    return rosnode.rosnode_ping(name, 1)
                except:
                    return False

            still_active_nodes = [x for x in self.active_nodes if
                                  x.name in momentary_nodenames]
            dead_nodes = [x for x in self.active_nodes if
                          x.name not in momentary_nodenames or not check_node_ping(x.name)]
            crashed_nodes = [x for x in self.active_nodes if not check_node_ping(x.name)]

            if dead_nodes:
                rospy.logdebug('One or more esiaf nodes shut down!')
                for dead in dead_nodes:
                    dead.bury()
                for crashed in crashed_nodes:
                    if crashed in still_active_nodes:
                        still_active_nodes.remove(crashed)
                self.active_nodes = still_active_nodes
                self.calculate_audio_tree()

    def register_node(self, nodeinfo):
        """
        The callback function for the Orchestrators register subscription. Will register a node with this Orchestrator.
        :param nodeinfo: the ros message containing the registering node's information
        :return: 
        """
        new_node = Node(nodeinfo)
        rospy.loginfo('Registering new node: \n' + str(nodeinfo))

        with self.active_nodes_lock:
            self.active_nodes.append(new_node)
            self.calculate_audio_tree()

        return RegisterNodeResponse()

    def calculate_audio_tree(self):
        """
        Creates a new, optimized audio flow tree based on node preferences.
        The node's determined audio formats will be stored directly in them, no special tree datastructure is constructed.
        Assumes to already have the active_nodes_lock.
        :return: None
        """
        self.calculate_audio_tree_naive()

    def check_for_new_data(self):
        """
        Checks whether a new message can be send out to outside-of-pipeline subscribers
        :return: True if a new message can be send out, false otherwise
        """
        return self.check_for_new_data_naive()

    ####################################################################################
    ###
    ### temporary stuff for testing etc
    ###
    ####################################################################################

    def calculate_audio_tree_naive(self):
        basic_format = AudioFormatInternal(AudioTopicFormatConstants.RATE_48000,
                                           AudioTopicFormatConstants.BIT_INT_32_SIGNED,
                                           1,
                                           AudioTopicFormatConstants.LittleEndian)

        for node in self.active_nodes:
            rospy.loginfo("node in active nodes during calculation of audio tree \n" + str(node))
            node.actualTopicsIn = []
            node.actualTopicsOut = []
            for intopic in node.allowedTopicsIn:
                node.actualTopicsIn.append((intopic.topic, basic_format))
            for outtopic in node.allowedTopicsOut:
                node.actualTopicsOut.append((outtopic.topic, basic_format))

            rospy.loginfo('audio tree calc actualtopics IN')
            rospy.loginfo(str(node.actualTopicsIn))
            rospy.loginfo('audio tree calc actualtopics OUT')
            rospy.loginfo(str(node.actualTopicsOut))
            node.update_config()

    def check_for_new_data_naive(self):
        """
        naive method for mocking checks whether all
        :return:
        """
        with self.active_nodes_lock:
            return all([x.subMsgSubscriber.last_msgs for x in self.active_nodes])
