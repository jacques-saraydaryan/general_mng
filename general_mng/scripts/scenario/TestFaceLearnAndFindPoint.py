__author__ = 'Benoit Renault'
import rospy

from AbstractScenario import AbstractScenario
from AbstractScenarioAction import AbstractScenarioAction
from AbstractScenarioBus import AbstractScenarioBus
from AbstractScenarioService import AbstractScenarioService
from LocalManagerWrapper import LocalManagerWrapper

import json
import time
import math

import pdb


class TestFaceLearnAndFindPoint(AbstractScenario, AbstractScenarioBus,
                                AbstractScenarioAction, AbstractScenarioService):
    DEFAULT_TIMEOUT = 5.0
    NO_TIMEOUT = -1.0

    def __init__(self, config):
        AbstractScenarioBus.__init__(self, config)
        AbstractScenarioAction.__init__(self, config)
        # self._lm_wrapper = LocalManagerWrapper(config.ip_address, config.tcp_port, config.prefix)

        # TODO : Remove Hardocoded values and get them from config
        self._lm_wrapper = LocalManagerWrapper("192.168.42.189", 9559, "R2019")

        # with open(config.scenario_filepath) as data:
        ws = "/home/astro/catkin_robocup2019"
        with open("{0}/src/robocup-main/robocup_pepper-scenario_data_generator/jsons/receptionist/scenario.json".format(ws)) as data:
            self._scenario = json.load(data)

        with open("{0}/src/robocup-main/robocup_pepper-scenario_data_generator/jsons/drinks.json".format(ws)) as data:
            self._drinks = json.load(data)

        with open("{0}/src/robocup-main/robocup_pepper-scenario_data_generator/jsons/locations.json".format(ws)) as data:
            self._locations = json.load(data)

        with open("{0}/src/robocup-main/robocup_pepper-scenario_data_generator/jsons/people.json".format(ws)) as data:
            self._people = json.load(data)

        with open("{0}/src/robocup-main/robocup_pepper-scenario_data_generator/jsons/videos.json".format(ws)) as data:
            self._videos = json.load(data)

        # Scenario data

        # - Constants
        self._living_room = self.find_by_id(self._locations, "livingRoom")
        self._entrance = self.find_by_id(self._locations, "entrance")
        self.host_name = "John"
        self.host_age = 40
        self.host_drink = self.find_by_id(self._drinks, "coke")

        # - Variables
        self.guest_1_name = "Placeholder name"
        self.guest_2_name = "Placeholder name"
        self.guest_1_age = 1000
        self.guest_2_age = 1000
        self.guest_1_drink = "Placeholder drink"
        self.guest_2_drink = "Placeholder drink"

    def startScenario(self):
        rospy.loginfo("""
        ######################################
        Starting the {scenario_name} Scenario...
        ######################################
        """.format(scenario_name=self._scenario["name"]))

        steps = self._scenario["steps"]

        ###################################################################################################
        # Resete people database
        self.resetPeopleMetaInfoMap()

        # Start timeboard to follow scenario evolution on screen

        # Remember the dictionary that associates big steps to the array that was sent to the local manager
        step_id_to_index = self._lm_wrapper.timeboard_send_steps_list(
            steps, self._scenario["name"], self.NO_TIMEOUT)[1]
        self._lm_wrapper.timeboard_set_timer_state(True, self.NO_TIMEOUT)

        # Ask infos about first guest
        self._lm_wrapper.timeboard_set_current_step(step_id_to_index["AskInfoG1"], self.NO_TIMEOUT)

        # global_step_ask_info_g1_start_time = time.time()

        #Head's up
        self.moveheadPose(self.HEAD_PITCH_FOR_LOOK_AT_PEOPLE, self.HEAD_YAW_CENTER, True)

        #First Ask name
        askinfog1_ask_name_counter = 0
        askinfog1_ask_name_max_counts = 3
        askinfog1_ask_name = self.find_by_id(steps, "askinfog1_ask-name")
        askinfog1_confirm_name = self.find_by_id(steps, "askinfog1_confirm-name")
        while True:

            if askinfog1_ask_name_counter >= askinfog1_ask_name_max_counts:
                rospy.logwarn("Could not get name with confirmation !")
                # TODO : Do TTS action where robot says something like: Hmmm, I really can't understand what you say,
                #  I guess I will just call you {Last_Understood Name}
                break

            # - Ask name
            tentative_guest_1_name = self._lm_wrapper.ask_name(askinfog1_ask_name["speech"], self._people, self.NO_TIMEOUT)[1]

            # - Confirm name
            confirm_speech = askinfog1_confirm_name["speech"]
            confirm_speech["name"] = tentative_guest_1_name
            askinfog1_ask_name_confirmed = self._lm_wrapper.confirm(confirm_speech, self.NO_TIMEOUT)[1]
            if askinfog1_ask_name_confirmed:
                rospy.loginfo("Guest 1 got name {name} confirmed !".format(name=tentative_guest_1_name))
                self.guest_1_name = tentative_guest_1_name
                break

            askinfog1_ask_name_counter += 1

        #Learn face from name
        state_learnPeopleMeta, result_learnPeopleMeta = self.learnPeopleMetaFromImgTopic(self.guest_1_name, 10.0) #TODO whatif the face is not properly seen
        state_learnPeopleMeta, result_learnPeopleMeta = self.learnPeopleMetaFromImgTopic(self.guest_1_name, 10.0) #TODO whatif the face is not properly seen
        print result_learnPeopleMeta

        # Then ask drink
        askinfog1_ask_drink_counter = 0
        askinfog1_ask_drink_max_counts = 3
        askinfog1_ask_drink = self.find_by_id(steps, "askinfog1_ask-drink")
        askinfog1_confirm_drink = self.find_by_id(steps, "askinfog1_confirm-drink")
        while True:

            if askinfog1_ask_drink_counter >= askinfog1_ask_drink_max_counts:
                rospy.logwarn("Could not get drink with confirmation !")
                # TODO : Do TTS action where robot says something like: Hmmm, I really can't understand what you say,
                #  I guess I will just consider you like {Last_Understood Drink}
                break

            # - Ask drink
            ask_speech = askinfog1_ask_drink["speech"]
            ask_speech["name"] = self.guest_1_name
            tentative_guest_1_drink = self._lm_wrapper.ask_drink(ask_speech, self._drinks, self.NO_TIMEOUT)[1]

            # - Confirm drink
            confirm_speech = askinfog1_confirm_drink["speech"]
            confirm_speech["drink"] = tentative_guest_1_drink["name"]
            askinfog1_ask_drink_confirmed = self._lm_wrapper.confirm(confirm_speech, self.NO_TIMEOUT)[1]
            if askinfog1_ask_drink_confirmed:
                rospy.loginfo("Guest 1 got drink <{drink}> confirmed !".format(drink=tentative_guest_1_drink["name"]))
                self.guest_1_drink = tentative_guest_1_drink
                break

            askinfog1_ask_drink_counter += 1

        self._lm_wrapper.timeboard_send_step_done(step_id_to_index["AskInfoG1"], self.NO_TIMEOUT)

        # Point to and introduce both G1 and Host

        # self.moveheadPose(self.HEAD_PITCH_FOR_SPEECH_POSE, self.HEAD_YAW_CENTER, True)  # Reset Head position to talk
        self.moveheadPose(self.HEAD_PITCH_CENTER, self.HEAD_YAW_CENTER, True)  # Reset Head position to talk

        # TODO: Make sure the name associated with the image of the person is updated from new input when only general
        #  manager is restarted but not the face_recognition package
        # Turn around to introduce guests
        nb_people_here = 2
        nb_people_introduced = 0
        people_introduced = {}
        people_introduced[self.guest_1_name] = False
        people_introduced["Host"] = False
        for inc_angle in range(360/25):
            # Find people in the image
            state_getObject, result_getObject = self.getObjectInFrontRobot(["person"], False, 50.0)
            # Loop on people found
            if result_getObject is not None:
                if len(result_getObject.labelFound) > 0:
                    for i_person in range(result_getObject.labelFound[0]):
                        # Get name of the i_person-th closest people
                        state_getPeopleName, result_getPeopleName = self.getPeopleNameFromImgTopic(50.0)
                        # Loop on people recognized
                        i_people_to_introduce = -1
                        bounding_box_area_max = -1
                        if result_getPeopleName is not None:
                            print result_getPeopleName.peopleNames
                            print result_getPeopleName.peopleNamesScore
                            for i_people_name in range(len(result_getPeopleName.peopleNames)):
                                # Bounding box around the current person
                                box_x0 = result_getPeopleName.peopleMetaList.peopleList[i_people_name].details.boundingBox.points[0].x
                                box_x1 = result_getPeopleName.peopleMetaList.peopleList[i_people_name].details.boundingBox.points[1].x
                                box_y0 = result_getPeopleName.peopleMetaList.peopleList[i_people_name].details.boundingBox.points[0].y
                                box_y1 = result_getPeopleName.peopleMetaList.peopleList[i_people_name].details.boundingBox.points[1].y
                                # Relative size of the bounding box
                                bounding_box_area = abs(box_x0 - box_x1)*abs(box_y0 - box_y1)
                                if bounding_box_area > bounding_box_area_max:
                                    bounding_box_area_max = bounding_box_area
                                    i_people_to_introduce = i_people_name
                            # Introduce the person
                            if i_people_to_introduce >= 0:
                                if (result_getPeopleName.peopleNames[i_people_to_introduce] == self.guest_1_name) and (people_introduced[self.guest_1_name] == False):
                                    # Introduce first guest to John
                                    self._lm_wrapper.timeboard_set_current_step(self.find_by_id(steps, "IntroduceG1ToJohn"), self.NO_TIMEOUT)
                                    nb_people_introduced += 1
                                    # Point to guest 1
                                    # Say name and drink
                                    int_g1_john = self.find_by_id(steps, "introduceg1tojohn_say-name-and-drink")
                                    self._lm_wrapper.present_person(int_g1_john["speech"], self.guest_1_name, self.guest_1_drink,
                                                                    [self.host_name], self.NO_TIMEOUT)
                                    state_lookAtObject, result_lookAtObject = self.lookAtObject(["person"], i_person, False, False, True, 50.0)
                                    self._lm_wrapper.timeboard_send_step_done(self.find_by_id(steps, "IntroduceG1ToJohn"), self.NO_TIMEOUT)
                                    people_introduced[self.guest_1_name] = True
                                # TODO We find the host by leimination : maybe to improve upon
                                elif (result_getPeopleName.peopleNames[i_people_to_introduce] == "Unknown") and (people_introduced["Host"] == False):
                                    # Introduce John to first guest
                                    self._lm_wrapper.timeboard_set_current_step(self.find_by_id(steps, "IntroduceJohnToG1"), self.NO_TIMEOUT)
                                    nb_people_introduced += 1
                                    # Point to John
                                    # Say name and drink
                                    int_john_g1 = self.find_by_id(steps, "introducejohntog1_say-name-and-drink")
                                    self._lm_wrapper.present_person(int_john_g1["speech"], self.host_name, self.host_drink,
                                                                    [self.guest_1_name], self.NO_TIMEOUT)
                                    state_lookAtObject, result_lookAtObject = self.lookAtObject(["person"], i_person, False, False, True, 50.0)
                                    self._lm_wrapper.timeboard_send_step_done(self.find_by_id(steps, "IntroduceJohnToG1"), self.NO_TIMEOUT)
                                    people_introduced["Host"] = True
                                else:
                                    # TODO unknown name ?
                                    pass
            # Check if everyone has been introduced
            if nb_people_introduced < nb_people_here:
                # Turn a bit to find someone else
                self.moveTurn(25.0*math.pi/180.0)
                #print "I TURN !!!!"
            else:
                # End introducing
                break

        rospy.loginfo("""
                ######################################
                Finished executing the {scenario_name} Scenario...
                ######################################
                """.format(scenario_name=self._scenario["name"]))

    def gmBusListener(self, msg):
        if self._status == self.WAIT_ACTION_STATUS:
            self.checkActionStatus(msg)

    def initScenario(self):
        self._enableNavAction = True
        self._enableTtsAction = False
        self._enableDialogueAction = False
        self._enableAddInMemoryAction = False
        self._enableObjectDetectionMngAction = True
        self._enableLookAtObjectMngAction = True
        self._enableMultiplePeopleDetectionAction = False
        self._enableRequestToLocalManagerAction = True
        self._enableLearnPeopleMetaAction = True
        self._enableGetPeopleNameAction = True

        self._enableMoveHeadPoseService = True
        self._enableMoveTurnService = True
        self._enablePointAtService = True
        self._enableResetPersonMetaInfoMap = True

        AbstractScenarioAction.configure_intern(self)
        AbstractScenarioService.configure_intern(self)

    def simulate_ros_work(self, time_for_work, log_string):
        rospy.logwarn(log_string)
        rospy.logwarn("Waiting for {duration} seconds...".format(duration=time_for_work))
        time.sleep(time_for_work)

    def find_by_id(self, steps_array, step_id):
        step_index = self.find_index_by_id(steps_array, step_id)
        if step_index is None:
            return None
        else:
            return steps_array[step_index]

    def find_index_by_id(self, steps_array, step_id):
        for index in range(len(steps_array)):
            step = steps_array[index]
            if step["id"] == step_id:
                return index
        return None
