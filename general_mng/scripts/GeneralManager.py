#!/usr/bin/env python
import rospy
import json
import yaml
from collections import namedtuple
from robocup_msgs.msg import gm_bus_msg
import threading
from std_msgs.msg import String

from scenario.CocktailPartyScenario import CocktailPartyScenario
from scenario.TestNavigScenario import TestNavigScenario
from scenario.DoorOpenAndNavigScenario import DoorOpenAndNavigScenario
from scenario.TestDialogueScenario import TestDialogueScenario
from scenario.TestCocktailPartyV1Scenario import TestCocktailPartyV1Scenario
from scenario.TestCocktailPartyV2Scenario import TestCocktailPartyV2Scenario
from scenario.TestCocktailPartyV3Scenario import TestCocktailPartyV3Scenario
from scenario.SPRV1Scenario import SPRV1Scenario
from scenario.HelpMeCarryV1Scenario import HelpMeCarryV1Scenario
from scenario.TestHoomanoScenario import TestHoomanoScenario
from scenario.InspectionScenario import InspectionScenario
from scenario.SPRV2Scenario import SPRV2Scenario
from scenario.GPRSV1Scenario import GPRSV1Scenario
from scenario.GPRSV2CPE1Scenario import GPRSV2CPE1Scenario
from scenario.Receptionist2019CPEScenario import Receptionist2019CPEScenario

from pepper_door_open_detector.srv import MinFrontValue


## COmmand Samples
# rostopic pub /gm_start std_msgs/String "data: 'START'"
#

class GeneralManager:
    CONFIG_FOLDER = ''
    CURRENT_SCENARIO = None
    _scenarioMap = {}
    _currentScenario = None
    START_STATUS = "START"
    PENDING_STATUS = "PENDING"
    # FIXME ADD ROS PARAM
    OPEN_DOOR_MIN_DISTANCE = 0.8
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
        currentConfig = None
        try:
            currentConfig = self.loadConfig()
        except Exception as e:
            rospy.logwarn("Unable to load config file: %s" % e)

        # self._scenarioMap["COCKTAIL_PARTY"]=CocktailPartyScenario(currentConfig)
        # self._scenarioMap["TEST_NAVIG"]=TestNavigScenario(currentConfig)
        # self._scenarioMap["DOOR_OPENED_NAVIG"]=DoorOpenAndNavigScenario(currentConfig)
        # self._scenarioMap["TEST_DIALOGUE"]=TestDialogueScenario(currentConfig)
        # self._scenarioMap["TEST_COCKTAIL_PARTY_V1"]=TestCocktailPartyV1Scenario(currentConfig)
        # self._scenarioMap["TEST_COCKTAIL_PARTY_V2"]=TestCocktailPartyV2Scenario(currentConfig)
        # self._scenarioMap["TEST_COCKTAIL_PARTY_V3"]=TestCocktailPartyV3Scenario(currentConfig)
        # self._scenarioMap["TEST_HOOMANO"]=TestHoomanoScenario(currentConfig)
        # self._scenarioMap["SPRV1"]=SPRV1Scenario(currentConfig)
        # self._scenarioMap["SPRV2"] = SPRV2Scenario(currentConfig)
        # self._scenarioMap["HELP_ME_CARRY"]=HelpMeCarryV1Scenario(currentConfig)
        # self._scenarioMap["INSPECTION"]=InspectionScenario(currentConfig)
        # self._scenarioMap["GPRSV1"] = GPRSV1Scenario(currentConfig)
        # self._scenarioMap["GPRSV2CPE"] = GPRSV2CPE1Scenario(currentConfig)
        self._scenarioMap["RECEPTIONIST_2019_CPE"] = Receptionist2019CPEScenario(currentConfig)

        try:
            self._currentScenario = self._scenarioMap[self.CURRENT_SCENARIO]
            self._currentScenario.initScenario()
        except Exception as e:
            rospy.logerr("Unable Load SCENARIO Object: %s" % e)

        # FOR DEBUG :
        # self.execute()

    def execute(self):
        if self._currentScenario == None:
            rospy.logwarn("Current Scenario is not ready (maybe not currently loaded), unable to start scenario...")
            return
        else:
            if self._currentScenario._configurationReady:
                self._currentScenario.startScenario()
            else:
                rospy.logwarn("Current Scenario is not ready, wait minutes and try again...")

    #######################################################################
    #######                LOAD Scenario config.                     ######
    #######################################################################
    def loadConfig(self):
        scenarioFile = self.CONFIG_FOLDER + "/" + self.CURRENT_SCENARIO + "_SCENARIO.json"
        rospy.loginfo("Opening scenario configuraiton file: %s" % scenarioFile)
        with open(scenarioFile) as json_data:
            jsonContent = yaml.safe_load(json_data)
            rospy.loginfo("Scenario configuraiton file content: %s" % jsonContent)
            # jsonString=json.load(jsonContent)
            # jsonObject = json.load(jsonString, object_hook=lambda d: namedtuple('X', d.keys())(*d.values()))
            ##rospy.loginfo("Scenario configuraiton Object content: %s" % str(jsonObject))
            rospy.loginfo('name: ' + jsonContent['name'])
            rospy.loginfo('description: ' + jsonContent['description'])
            return jsonContent

    def waitDoorOpened(self):
        ## wait for min front value service to detect door opening
        try:
            rospy.wait_for_service('/min_front_value_srv', 5)

            rospy.loginfo("end service min_front_value_srv wait time")
            self._getMinFrontDist = rospy.ServiceProxy('min_front_value_srv', MinFrontValue)
        except Exception as e:
            rospy.logerr("Service min_front_value_srv call failed: %s" % e)
            return

        ## check the min front value at a given frequency
        try:
            rate = rospy.Rate(2)  # 2hz
            current_distance = 0.0
            while not rospy.is_shutdown() and current_distance < self.OPEN_DOOR_MIN_DISTANCE:
                result = self._getMinFrontDist()
                current_distance = result.value
                rate.sleep()
            rospy.loginfo("************ DOOR IS OPENED, START SCENARIO ****************")
            self._start_pub.publish("START")
        except Exception as e:
            rospy.logwarn(" EXCEPTION ON THE DOOR OPEN SERVICE !!! e:" + str(e))

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

