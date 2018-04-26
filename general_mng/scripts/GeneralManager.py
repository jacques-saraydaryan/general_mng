#!/usr/bin/env python
import rospy
import json
from collections import namedtuple
from robocup_msgs.msg import gm_bus_msg
import threading
from std_msgs.msg import String
 
from scenario.CocktailPartyScenario import CocktailPartyScenario
from scenario.TestNavigScenario import TestNavigScenario

## COmmand Samples
# rostopic pub /gm_start std_msgs/String "data: 'START'"
#

class GeneralManager:
    CONFIG_FOLDER=''
    CURRENT_SCENARIO=None
    _scenarioMap={}
    _currentScenario=None
    START_STATUS="START"

    def __init__(self):
        rospy.init_node('general_manager')       
        self.configure()
        pass

    #######################################################################
    #######                Configure Current Node                    ######
    #######################################################################
    def configure(self):
        self._start_sub = rospy.Subscriber("gm_start", String, self.gmStartCallback)

        #load face files form data directory
        self.CONFIG_FOLDER=rospy.get_param('GeneralManager/config_folder')
        self.CURRENT_SCENARIO=rospy.get_param('GeneralManager/current_scenario')
       
        rospy.loginfo("Param: config_folder:"+str(self.CONFIG_FOLDER))
        rospy.loginfo("Param: current_scenario:"+str(self.CURRENT_SCENARIO))
        rospy.loginfo('configure ok')
        currentConfig=None
        try:
            currentConfig=self.loadConfig()
        except Exception as e:
             rospy.logwarn("Unable to load config file: %s" % e)
            

        self._scenarioMap["COCKTAIL_PARTY"]=CocktailPartyScenario(currentConfig)
        self._scenarioMap["TEST_NAVIG"]=TestNavigScenario(currentConfig)

        try:
            self._currentScenario=self._scenarioMap[self.CURRENT_SCENARIO]
        except Exception as e:
             rospy.logerr("Unable Load SCENARIO Object: %s" % e)
       

    def execute(self):
        if self._currentScenario == None:
            rospy.logwarn("Current Scenario is not ready (maybe not currently loaded), unable to start scenario...")
            return
        else:
            self._currentScenario.startScenario()

    #######################################################################
    #######                LOAD Scenario config.                     ######
    #######################################################################
    def loadConfig(self):
        scenarioFile= self.CONFIG_FOLDER+"/"+self.CURRENT_SCENARIO+"_SCENARIO.json"
        rospy.loginfo("Opening scenario configuraiton file: %s" % scenarioFile)
        with open(scenarioFile) as json_data:
            jsonContent = json.load(json_data)
            rospy.loginfo("Scenario configuraiton file content: %s" % jsonContent)
            jsonString=json.dumps(jsonContent)
            jsonObject = json.loads(jsonString, object_hook=lambda d: namedtuple('X', d.keys())(*d.values()))
            #rospy.loginfo("Scenario configuraiton Object content: %s" % str(jsonObject))
            rospy.loginfo('name: '+jsonObject.name)
            rospy.loginfo('description: '+jsonObject.description)
            return jsonObject


    def gmStartCallback(self,msg):
        rospy.loginfo('START CMD: '+msg.data)
        if msg.data == self.START_STATUS:
            self.execute()
 


if __name__ == '__main__':
    
    gm=GeneralManager()

    rospy.spin()

