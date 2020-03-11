#!/usr/bin/env python

import rospy
import yaml
from std_msgs.msg import String
import importlib


# Command Samples
# rostopic pub /gm_start std_msgs/String "data: 'START'"


class GeneralManager:
    CONFIG_FOLDER = ''
    CURRENT_SCENARIO = None
    _scenarioMap = {}
    _currentScenario = None
    START_STATUS = "START"
    PENDING_STATUS = "PENDING"
    # FIXME ADD ROS PARAM
    _current_status = PENDING_STATUS

    def __init__(self, config):
        rospy.init_node('general_manager')
        self.configure(config)
        # self.waitDoorOpened()
        pass

    #######################################################################
    #######                Configure Current Node                    ######
    #######################################################################
    def configure(self, config):
        self._start_sub = rospy.Subscriber("gm_start", String, self.gmStartCallback)
        self._start_pub = rospy.Publisher("/gm_start", String, queue_size=1)

        # load face files form data directory
        self.CONFIG_FOLDER = config["GeneralManager"]["config_folder"]
        self.CURRENT_SCENARIO = config["GeneralManager"]["current_scenario"]

        rospy.loginfo("Param: config_folder:" + str(self.CONFIG_FOLDER))
        rospy.loginfo("Param: current_scenario:" + str(self.CURRENT_SCENARIO))
        rospy.loginfo('configure ok')
        scenario_config = None

        try:
            # mod = importlib.import_module(".scenario."+self.CURRENT_SCENARIO)
            print("scenario."+self.CURRENT_SCENARIO)
            module = importlib.import_module("scenario."+self.CURRENT_SCENARIO)
            self._current_scenario_class = getattr(module, self.CURRENT_SCENARIO)
        except ImportError as error:
            # Output expected ImportErrors.
            rospy.logerr(error.__class__.__name__ + ": " + error.message)
        except Exception as exception:
            rospy.logerr(exception)
            rospy.logerr(exception.__class__.__name__ + ": " + exception.message)




        try:
            scenario_config = self.loadConfig()
        except Exception as e:
            rospy.logwarn("Unable to load config file: %s" % e)

        try:
            self._currentScenario = self._current_scenario_class(scenario_config)
            self._currentScenario.initScenario()
        except Exception as e:
            rospy.logerr("Unable Load SCENARIO Object: %s" % e)

        # FOR DEBUG :
        self.execute()

    def execute(self):
        if self._currentScenario == None:
            rospy.logwarn("Current Scenario is not ready (maybe not currently loaded), unable to start scenario...")
            return
        else:
            if self._currentScenario.configurationReady:
                self._currentScenario.startScenario()
            else:
                rospy.logwarn("Current Scenario is not ready, wait minutes and try again...")

    #######################################################################
    #######                LOAD Scenario config.                     ######
    #######################################################################
    def loadConfig(self):
        scenarioFile = self.CONFIG_FOLDER + "/" + self.CURRENT_SCENARIO + "_SCENARIO.json"
        rospy.loginfo("Opening scenario configuration file: %s" % scenarioFile)
        with open(scenarioFile) as json_data:
            jsonContent = yaml.safe_load(json_data)
            rospy.loginfo("Scenario configuration file content: %s" % jsonContent)
            # jsonString=json.load(jsonContent)
            # jsonObject = json.load(jsonString, object_hook=lambda d: namedtuple('X', d.keys())(*d.values()))
            ##rospy.loginfo("Scenario configuraiton Object content: %s" % str(jsonObject))
            rospy.loginfo('name: ' + jsonContent['name'])
            rospy.loginfo('description: ' + jsonContent['description'])
            return jsonContent

    def gmStartCallback(self, msg):
        if self._current_status == self.START_STATUS:
            rospy.loginfo('SCENARIO IS ALREADY STARTED...')
            return
        rospy.loginfo('START CMD: ' + msg.data)
        if msg.data == self.START_STATUS:
            self._current_status = self.START_STATUS
            self.execute()
            self._current_status = self.PENDING_STATUS
            rospy.loginfo('-----------------------------------END CURRENT SCENARIO-----------------------')


if __name__ == '__main__':
    import os

    dirname = os.path.dirname(__file__)
    filepath = os.path.join(dirname, '../config/common_gm.yaml')


    with open(filepath) as data:
        gm_config = yaml.safe_load(data)

    gm = GeneralManager(gm_config)

    rospy.spin()

