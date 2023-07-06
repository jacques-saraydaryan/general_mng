__author__ = 'Jacques Saraydaryan'
import rospy
import os
import json
from copy import deepcopy
from AbstractScenario import AbstractScenario

from meta_lib.LTHriManager import LTHriManagerPalbator
from meta_lib.LTNavigation import LTNavigation
from meta_lib.LTPerception import LTPerception

class TestHRIScenarioV1(AbstractScenario):
    DEFAULT_TIMEOUT = 5.0
    NO_TIMEOUT = -1.0
    _severalActionPending = {}
    _oneActionPending = None

    def __init__(self,config,scenario_path_folder):
        self.configuration_ready = False
        self.init_scenario(config,scenario_path_folder)
        rospy.on_shutdown(self.onShutDown)
        

    def init_scenario(self, config,scenario_path_folder):

        self._scenario_path_folder = scenario_path_folder
        self._scenario=config
        self.current_dir = os.path.dirname(os.path.abspath(__file__))

        
        _name_action_server_HRI = self._scenario['parameters']['LTHri_action_server_name']
        self._nav_strategy = self._scenario['parameters']['nav_strategy_parameters']
        debug_variables = self._scenario['parameters']['debug_variables']
        
        self._lm_wrapper = LTHriManagerPalbator(_name_action_server_HRI)
        self._lt_navigation = LTNavigation()
        self._lt_perception = LTPerception()


        #####################  FOR DEBUG #####################
        #VARIABLES FOR DEBUG, DEFAULT -> TRUE. TO MODIFY, SEE JSON DATA SCENARIO FILE
        self.allow_perception = debug_variables['allow_perception']
        self.allow_navigation = debug_variables['allow_navigation']
        self.allow_highbehaviour = debug_variables['allow_highbehaviour']
        self.allow_motion = debug_variables['allow_motion']
        self.allow_simulation = debug_variables['allow_simulation']
        ####################################################

        self._locations = self._scenario['imports']['locations']
        self._objects = self._scenario['imports']['objects']
        self._videos = self._scenario['imports']['videos']      
        self._path_scenario_infos = self._scenario['variables']['scenarioInfos']
        self.reset_infos_JSON()
        
        rospy.loginfo("{class_name}: JSON FILES LOADED.".format(class_name=self.__class__.__name__))

        self.configuration_ready = True
        

    def start_scenario(self):
        """
        Runs the scenario according to the scenario JSON file
        """
        rospy.loginfo("""
        ######################################
        Starting the %s Scenario...
        ######################################
        """%str(self._scenario["name"]))

        # Wait door opening
        #result = self.lt_perception.wait_for_door_to_open()
        #self.print_result(result)

        self.manual_scenario()

    def manual_scenario(self):
        self.steps = deepcopy(self._scenario["steps"])

        rospy.loginfo("{class_name} : WAITING FOR HRI ACTION SERVER ACTIVATION".format(class_name=self.__class__.__name__))
        self._lm_wrapper.client_action_GmToHri.wait_for_server()

        self._lm_wrapper.restart_hri(self.NO_TIMEOUT)

        #Way Door Opened
        #result = self._lt_perception.wait_for_door_to_open()
        #self.print_result(result)

         # options (set of [value,media_src,media_type])
        result = self._lm_wrapper.generic_global("C1","C1 - Drink Selection",90,"Choose your favorite drink",
                        description="Choose your favorite drink",
                        type="question",
                        wait_answer=True,
                        need_confirmation=True,
                        need_validation=True,
                        options=[{'value': 'coca','media_src': 'https://www.lemoulindecaro.fr/wp-content/uploads/2020/05/coca.jpg', 'type':'drink','media_type': 'img'},{'value': 'water',  'type':'drink'},{'value': 'juice',  'type':'drink'}]
                        )

        result = self._lm_wrapper.generic_global("C1","C1 - Drink Selection",90,"Choose your favorite drink",
                        description="Choose your favorite drink",
                        type="question",
                        wait_answer=True,
                        need_confirmation=True,
                        need_validation=True,
                        options=[{'value': 'john', 'type':'person'},{'value': 'maria', 'type':'person'},{'value': 'jo', 'type':'person'},{'value': 'eric', 'type':'person'}]
                        )

        #rospy.loginfo("{class_name} : LOADING CONFIG FOR SCENARIO".format(class_name=self.__class__.__name__))
        self._lm_wrapper.timeboard_send_steps_list(self.steps, self._scenario["name"], self.NO_TIMEOUT)

        result = self._lm_wrapper.generic_global("A1","A11 - Visual wait info Name",30,"Visual wait info Speech",
                        description="Visual wait info Description",
                        wait_answer=False,
                        need_confirmation=False,
                        need_validation=False,
                        media_src="/img/hri/landscape1.gif",
                        media_type="img"
                        )

        rospy.sleep(3)

        # start navigation
        #result = self._lt_navigation.send_nav_order_to_pt("NP", "CRRCloseToGoal", -11.8, 2.45, 60.0)

        rospy.loginfo("{class_name} ACTION Wait confirmation")
        #self._lm_wrapper.timeboard_set_current_step("Beginning",self.NO_TIMEOUT)
        result = self._lm_wrapper.generic_global("A1","A12 -  Start Action",60,"Tell me when you are ready to continue",
                        description="Tell me when you are ready to continue",
                        wait_answer=True,
                        need_confirmation=True,
                        need_validation=True
                        )

        # start navigation
        #result = self._lt_navigation.send_nav_order_to_pt("NP", "CRRCloseToGoal", -11.9, -2.25, 60.0)
        #self.print_result(result)


        result = self._lm_wrapper.generic_global("A1","A13 - Start Action",60,"Tell me when you are ready to continue (tell me again)",
                        description="Tell me when you are ready to continue (tell me again)",
                        wait_answer=True,
                        need_confirmation=False,
                        need_validation=True
                        )

        #result = self._lt_navigation.send_nav_order_to_pt("NP", "CRRCloseToGoal", -11.8, 2.45, 60.0)
        
        result = self._lm_wrapper.generic_global("B1","B1 - Info 1",60,"Tell me when you are ready to continue 2",
                        description="Tell me when you are ready to continue 2",
                        wait_answer=True,
                        need_confirmation=False,
                        need_validation=True
                        )

        #result = self._lt_navigation.send_nav_order_to_pt("NP", "CRRCloseToGoal", -1.68, 0.314, 60.0)
        #Fake move simulation
        #rospy.sleep(10)

        # options (set of [value,media_src,media_type])
        result = self._lm_wrapper.generic_global("C1","C1 - Drink Selection",90,"Choose your favorite drink",
                        description="Choose your favorite drink",
                        type="question",
                        wait_answer=True,
                        need_confirmation=True,
                        need_validation=True,
                        options=[{'value': 'coca','media_src': 'https://www.lemoulindecaro.fr/wp-content/uploads/2020/05/coca.jpg', 'type':'drink','media_type': 'img'},{'value': 'water',  'type':'drink'}]
                        )
        try:
            data = result.payload['drink'][0]
        except KeyError:
            rospy.WARN("Unable to get name from result")
        self.print_result(result)

        rospy.loginfo("[GENERAL_MANAGER] Result after selection" + str(result))

        result = self._lm_wrapper.generic_global("D1","D1 - The End Name",30,"The End Speech",
                        description="The End Description",
                        wait_answer=False,
                        need_confirmation=False,
                        need_validation=False,
                        media_src="/img/hri/theend.gif",
                        media_type="img"
                        )

    def auto_scenarion(self):
        self.restart_order=False
        self.steps = deepcopy(self._scenario["steps"])

        rospy.loginfo("{class_name} : WAITING FOR ACTION SERVER ACTIVATION".format(class_name=self.__class__.__name__))
        self._lm_wrapper.client_action_GmToHri.wait_for_server()

        self._lm_wrapper.restart_hri(self.NO_TIMEOUT)

        rospy.loginfo("{class_name} : LOADING CONFIG FOR SCENARIO".format(class_name=self.__class__.__name__))
        self._lm_wrapper.timeboard_send_steps_list(self.steps, self._scenario["name"], self.NO_TIMEOUT)


        self.current_index_scenario=0
        self.scenario_end = False

        # while self.current_index_scenario<len(self.steps) and not rospy.is_shutdown():
        while self.scenario_end == False and not rospy.is_shutdown():

            rospy.loginfo("{class_name} : CURRENT STEP INDEX : ".format(class_name=self.__class__.__name__)+str(self.current_index_scenario))
            rospy.loginfo("{class_name}: NEW STEP".format(class_name=self.__class__.__name__))

            self.current_step=deepcopy(self.steps[self.current_index_scenario])

            if self.current_step['action']!="":
                result=self.action_parser(self.current_step['action'])
                
                rospy.loginfo("{class_name} : RESULT FROM PARSER ".format(class_name=self.__class__.__name__)+str(result))
            else:
                result=self._lm_wrapper.timeboard_set_current_step(self.current_index_scenario,self.NO_TIMEOUT)[1]
                rospy.loginfo("{class_name} : RESULT WITHOUT PARSER ".format(class_name=self.__class__.__name__)+str(result))
            
            if result is None:
                rospy.logwarn("{class_name} : ACTION ABORTED".format(class_name=self.__class__.__name__))
                break
            
            elif result != None:
                if 'result' in result and result['result']=="PREEMPTED": 
                    rospy.logwarn("{class_name} : ACTION ABORTED".format(class_name=self.__class__.__name__))
                    break
                
                if result['NextIndex'] != "":
                    self.current_index_scenario=deepcopy(result['NextIndex'])
                else:
                    self.scenario_end = True

                if 'saveData' in result:
                    self.store_infos(result['saveData'])

        rospy.logwarn("{class_name} : END OF SCENARIO ".format(class_name=self.__class__.__name__) + self._scenario["name"])


    def reset_infos_JSON(self):
        """
        Reset the JSON file which stores all the infos of current scenario.
        """
        with open(os.path.join(self._scenario_path_folder,self._path_scenario_infos),"w+") as f:
            data_JSON = {}
            json.dumps(data_JSON, f, indent=4)
            f.truncate()

    def onShutDown(self):
        rospy.loginfo(""" SHUTDOWN ASKED Clear action server before ending """)
        self._lm_wrapper.cancel_all_actions_HRI_mng
    