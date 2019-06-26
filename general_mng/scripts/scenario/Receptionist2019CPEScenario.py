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


class Receptionist2019CPEScenario(AbstractScenario, AbstractScenarioBus,
                                  AbstractScenarioAction, AbstractScenarioService):
    DEFAULT_TIMEOUT = 5.0
    NO_TIMEOUT = -1.0

    def __init__(self, config):
        AbstractScenarioBus.__init__(self, config)
        AbstractScenarioAction.__init__(self, config)
        # self._lm_wrapper = LocalManagerWrapper(config.ip_address, config.tcp_port, config.prefix)

        # TODO : Remove Hardocoded values and get them from config
        self._lm_wrapper = LocalManagerWrapper("pepper2", 9559, "R2019")

        # with open(config.scenario_filepath) as data:
        with open("/home/xia0ben/pepper_ws/src/robocup-main/robocup_pepper-scenario_data_generator/jsons/receptionist/scenario.json") as data:
            self._scenario = json.load(data)

        with open("/home/xia0ben/pepper_ws/src/robocup-main/robocup_pepper-scenario_data_generator/jsons/drinks.json") as data:
            self._drinks = json.load(data)

        with open("/home/xia0ben/pepper_ws/src/robocup-main/robocup_pepper-scenario_data_generator/jsons/locations.json") as data:
            self._locations = json.load(data)

        with open("/home/xia0ben/pepper_ws/src/robocup-main/robocup_pepper-scenario_data_generator/jsons/people.json") as data:
            self._people = json.load(data)

        with open("/home/xia0ben/pepper_ws/src/robocup-main/robocup_pepper-scenario_data_generator/jsons/videos.json") as data:
            self._videos = json.load(data)

        # Scenario data

        # - Constants
        self._living_room = self.find_by_id(self._locations, "livingRoom")
        self._entrance = self.find_by_id(self._locations, "entrance")
        self.host_name = "John"
        self.host_age = 40
        self.host_drink = self.find_by_id(self._drinks, "coke")["name"]

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
        # Start timeboard to follow scenario evolution on screen

        # Remember the dictionary that associates big steps to the array that was sent to the local manager
        step_id_to_index = self._lm_wrapper.timeboard_send_steps_list(
            steps, self._scenario["name"], self.NO_TIMEOUT)[1]
        self._lm_wrapper.timeboard_set_timer_state(True, self.NO_TIMEOUT)

        # scenario_start_time = time.time()

        ###################################################################################################

        # - Go to door
        # TODO Add scenario step !
        # self.moveheadPose(self.HEAD_PITCH_FOR_NAV_POSE, self.HEAD_YAW_CENTER, True)  # Reset Head position to navigate
        # self.sendNavOrderAction("NP", "CRRCloseToGoal", "GPRS_PEOPLE_ENTRANCE_It0", 50.0)

        ###################################################################################################

        # Find first guest
        self._lm_wrapper.timeboard_set_current_step(step_id_to_index["FindG1"], self.NO_TIMEOUT)
        self.moveheadPose(self.HEAD_PITCH_FOR_SPEECH_POSE, self.HEAD_YAW_CENTER, True)  # Reset Head position to talk

        global_step_find_g1_start_time = time.time()

        # - Wait
        findg1_wait = self.find_by_id(steps, "findg1_wait")
        self._lm_wrapper.wait(findg1_wait["speech"], findg1_wait["arguments"]["time"], findg1_wait["arguments"]["time"] + 2.0)

        # - Ask referee to open the door
        findg1_ask_referee = self.find_by_id(steps, "findg1_ask-referee-to-open-the-door")
        self._lm_wrapper.ask_open_door(findg1_ask_referee["speech"], self.NO_TIMEOUT)

        # - Wait2
        findg1_wait2 = self.find_by_id(steps, "findg1_wait2")
        self._lm_wrapper.wait(findg1_wait2["speech"], findg1_wait2["arguments"]["time"], findg1_wait2["arguments"]["time"] + 2.0)

        # - Detect human
        # findg1_detect_human = self.find_step(steps, "findg1_detect-human")
        # self._lm_wrapper.call_human(findg1_detect_human["speech"], 3.0, self.NO_TIMEOUT)
        # self.simulate_ros_work(1.0, "SIMULATING HUMAN DETECTION")

        self._lm_wrapper.timeboard_send_step_done(step_id_to_index["FindG1"], self.NO_TIMEOUT)

        ###################################################################################################

        # Ask infos about first guest
        self._lm_wrapper.timeboard_set_current_step(step_id_to_index["AskInfoG1"], self.NO_TIMEOUT)

        # global_step_ask_info_g1_start_time = time.time()

        #Head's up
        self.moveheadPose(self.HEAD_PITCH_FOR_LOOK_AT_PEOPLE, self.HEAD_YAW_CENTER, True)

        #First Ask name
        askinfog1_ask_name_max_counts = 3  # TODO Move this as config parameter
        askinfog1_ask_name = self.find_by_id(steps, "askinfog1_ask-name")
        askinfog1_confirm_name = self.find_by_id(steps, "askinfog1_confirm-name")
        self.guest_1_name = self.ask_name_and_confirm(
            askinfog1_ask_name_max_counts, askinfog1_ask_name, askinfog1_confirm_name)

        # Learn face from name
        # TODO whatif the face is not properly seen ? --> Make specific scenario view that sends feedback !
        # state_learnPeopleMeta, result_learnPeopleMeta = self.learnPeopleMetaFromImgTopic(self.guest_1_name, 10.0)

        # Then ask drink
        askinfog1_ask_drink_max_counts = 3  # TODO Move this as config parameter
        askinfog1_ask_drink = self.find_by_id(steps, "askinfog1_ask-drink")
        askinfog1_confirm_drink = self.find_by_id(steps, "askinfog1_confirm-drink")
        self.guest_1_drink = self.ask_drink_and_confirm(askinfog1_ask_drink_max_counts, askinfog1_ask_drink, askinfog1_confirm_drink)

        # - Ask age
        askinfog1_ask_age = self.find_by_id(steps, "askinfog1_ask-age")
        askinfog1_ask_age_speech = askinfog1_ask_age["speech"]
        askinfog1_ask_age_speech["name"] = self.guest_1_name
        self.guest_1_age = self._lm_wrapper.ask_age(askinfog1_ask_age_speech, self.NO_TIMEOUT)[1]

        self._lm_wrapper.timeboard_send_step_done(step_id_to_index["AskInfoG1"], self.NO_TIMEOUT)

        ###################################################################################################

        # Go to living room
        self._lm_wrapper.timeboard_set_current_step(step_id_to_index["GotoLR1"], self.NO_TIMEOUT)

        # global_step_go_to_lr1_start_time = time.time()

        # - Ask to follow
        gotolr1_ask_to_follow = self.find_by_id(steps, "gotolr1_ask-to-follow")
        gotolr1_ask_to_follow["speech"]["name"] = self.guest_1_name
        self._lm_wrapper.ask_to_follow(gotolr1_ask_to_follow["speech"], self._living_room, self.NO_TIMEOUT)

        # - Go to living room
        gotolr1_go_to_living_room = self.find_by_id(steps, "gotolr1_go-to-living-room")
        self._lm_wrapper.go_to(gotolr1_go_to_living_room["speech"], self._living_room, self.NO_TIMEOUT)
        self.moveheadPose(self.HEAD_PITCH_FOR_NAV_POSE, self.HEAD_YAW_CENTER, True)  # Reset Head position to navigate
        # self.sendNavOrderAction("NP", "CRRCloseToGoal", gotolr1_go_to_living_room["arguments"]["interestPoint"], 50.0)

        self._lm_wrapper.timeboard_send_step_done(step_id_to_index["GotoLR1"], self.NO_TIMEOUT)

        ###################################################################################################

        # Point to and introduce both G1 and Host
        #
        # self.moveheadPose(self.HEAD_PITCH_FOR_SPEECH_POSE, self.HEAD_YAW_CENTER, True)  # Reset Head position to talk
        #
        # # TODO: Make sure the name associated with the image of the person is updated from new input when only general
        # #  manager is restarted but not the face_recognition package
        # # Turn around to introduce guests
        # nb_people_here = 2
        # nb_people_introduced = 0
        # for inc_angle in range(360/25):
        #     # Find people in the image
        #     state_getObject, result_getObject = self.getObjectInFrontRobot("person", 15.0)
        #     # Loop on people found
        #     if result_getObject is not None:
        #         for i_person in range(len(result_getObject.labelList)):
        #             # Get name of the i_person-th closest people
        #             state_lookAtObject, result_lookAtObject = self.lookAtObject("person", i_person, True, False, True, 4.0)
        #             state_getPeopleName, result_getPeopleName = self.getPeopleNameFromImgTopic(15.0)
        #             # Loop on people recognized
        #             i_people_to_introduce = -1
        #             bounding_box_area_max = -1
        #             if result_getPeopleName is not None:
        #                 for i_people_name in range(len(result_getPeopleName.peopleNames)):
        #                     # Bounding box around the current person
        #                     box_x0 = result_getPeopleName.peopleMetaList.peopleList[i_people_name].details.boundingBox.points[0].x
        #                     box_x1 = result_getPeopleName.peopleMetaList.peopleList[i_people_name].details.boundingBox.points[1].x
        #                     box_y0 = result_getPeopleName.peopleMetaList.peopleList[i_people_name].details.boundingBox.points[0].y
        #                     box_y1 = result_getPeopleName.peopleMetaList.peopleList[i_people_name].details.boundingBox.points[1].y
        #                     # Relative size of the bounding box
        #                     bounding_box_area = abs(box_x0 - box_x1)*abs(box_y0 - box_y1)
        #                     if bounding_box_area > bounding_box_area_max:
        #                         bounding_box_area_max = bounding_box_area
        #                         i_people_to_introduce = i_people_name
        #                 # Introduce the person
        #                 if result_getPeopleName.peopleNames[i_people_to_introduce] == self.guest_1_name:
        #                     # Introduce first guest to John
        #                     self._lm_wrapper.timeboard_set_current_step(self.find_by_id["IntroduceG1ToJohn"], self.NO_TIMEOUT)
        #                     nb_people_introduced += 1
        #                     # Point to guest 1
        #                     # Say name and drink
        #                     int_g1_john = self.find_by_id(steps, "introduceg1tojohn_say-name-and-drink")
        #                     self._lm_wrapper.present_person(int_g1_john["speech"], self.guest_1_name, self.guest_1_drink,
        #                                                     [self.host_name], self.NO_TIMEOUT)
        #                     self._lm_wrapper.timeboard_send_step_done(self.find_by_id["IntroduceG1ToJohn"], self.NO_TIMEOUT)
        #                 # TODO We find the host by leimination : maybe to improve upon
        #                 elif result_getPeopleName.peopleNames[i_people_to_introduce] == "Unknown":
        #                     # Introduce John to first guest
        #                     self._lm_wrapper.timeboard_set_current_step(self.find_by_id["IntroduceJohnToG1"], self.NO_TIMEOUT)
        #                     nb_people_introduced += 1
        #                     # Point to John
        #                     # Say name and drink
        #                     int_john_g1 = self.find_by_id(steps, "introducejohntog1_say-name-and-drink")
        #                     self._lm_wrapper.present_person(int_john_g1["speech"], self.host_name, self.host_drink,
        #                                                     [self.guest_1_name], self.NO_TIMEOUT)
        #                     self._lm_wrapper.timeboard_send_step_done(self.find_by_id["IntroduceJohnToG1"], self.NO_TIMEOUT)
        #                 else:
        #                     # TODO unknown name ?
        #                     pass
        #     # Check if everyone has been introduced
        #     if nb_people_introduced < nb_people_here:
        #         # Turn a bit to find someone else
        #         self.moveTurn(25.0*math.pi/180.0)
        #     else:
        #         # End introducing
        #         break

        ###################################################################################################

        # Seat first guest
        self._lm_wrapper.timeboard_set_current_step(step_id_to_index["SeatG1"], self.NO_TIMEOUT)
        # global_step_seat_g1_start_time = time.time()

        # - Find empty seat
        self.simulate_ros_work(1.0, "SIMULATING FINDING AN EMPTY SEAT")

        # - Point to empty seat
        self.simulate_ros_work(1.0, "SIMULATING POINTING TO EMPTY SEAT")

        # - Tell first guest to seat
        seat_g1 = self.find_by_id(steps, "seatg1_tell-first-guest-to-seat")
        seat_g1["speech"]["name"] = self.guest_1_name
        self._lm_wrapper.seat_guest(seat_g1["speech"], self.NO_TIMEOUT)

        self._lm_wrapper.timeboard_send_step_done(step_id_to_index["SeatG1"], self.NO_TIMEOUT)

        ###################################################################################################

        # TODO If time too short, don't try to bring second guest ?

        # Go to door
        self._lm_wrapper.timeboard_set_current_step(step_id_to_index["GotoDoor1"], self.NO_TIMEOUT)
        # global_step_find_go_to_door1_start_time = time.time()

        # - Go to door
        gotodoor1 = self.find_by_id(steps, "gotodoor1_go-to-door")
        self._lm_wrapper.go_to(gotodoor1["speech"], self._entrance, self.NO_TIMEOUT)
        self.moveheadPose(self.HEAD_PITCH_FOR_NAV_POSE, self.HEAD_YAW_CENTER, True)  # Reset Head position to navigate
        # self.sendNavOrderAction("NP", "CRRCloseToGoal", gotodoor1["arguments"]["interestPoint"], 50.0)

        self._lm_wrapper.timeboard_send_step_done(step_id_to_index["GotoDoor1"], self.NO_TIMEOUT)

        # ###################################################################################################

        # Find second guest
        self._lm_wrapper.timeboard_set_current_step(step_id_to_index["FindG2"], self.NO_TIMEOUT)
        self.moveheadPose(self.HEAD_PITCH_FOR_SPEECH_POSE, self.HEAD_YAW_CENTER, True)  # Reset Head position to talk
        # global_step_find_g2_start_time = time.time()

        # - Wait
        findg2_wait = self.find_by_id(steps, "findg2_wait")
        self._lm_wrapper.wait(findg2_wait["speech"], findg2_wait["arguments"]["time"], findg2_wait["arguments"]["time"] + 2.0)

        # - Ask referee to open the door
        findg2_ask_referee = self.find_by_id(steps, "findg2_ask-referee-to-open-the-door")
        self._lm_wrapper.ask_open_door(findg2_ask_referee["speech"], self.NO_TIMEOUT)

        # - Wait2
        findg2_wait2 = self.find_by_id(steps, "findg2_wait2")
        self._lm_wrapper.wait(findg2_wait2["speech"], findg2_wait2["arguments"]["time"], findg2_wait2["arguments"]["time"] + 2.0)

        # - Detect human
        # findg2_detect_human = self.find_step(steps, "findg2_detect-human")
        # self._lm_wrapper.call_human(findg2_detect_human["speech"], 3.0, self.NO_TIMEOUT)
        # self.simulate_ros_work(1.0, "SIMULATING HUMAN DETECTION")

        self._lm_wrapper.timeboard_send_step_done(step_id_to_index["FindG2"], self.NO_TIMEOUT)

        # ###################################################################################################

        # Ask infos about second guest
        self._lm_wrapper.timeboard_set_current_step(step_id_to_index["AskInfoG2"], self.NO_TIMEOUT)
        # global_step_ask_info_g2_start_time = time.time()

        #Head's up
        self.moveheadPose(self.HEAD_PITCH_FOR_LOOK_AT_PEOPLE, self.HEAD_YAW_CENTER, True)

        #First Ask name
        askinfog2_ask_name_max_counts = 3  # TODO Move this as config parameter
        askinfog2_ask_name = self.find_by_id(steps, "askinfog2_ask-name")
        askinfog2_confirm_name = self.find_by_id(steps, "askinfog2_confirm-name")
        self.guest_2_name = self.ask_name_and_confirm(
            askinfog2_ask_name_max_counts, askinfog2_ask_name, askinfog2_confirm_name)

        # Learn face from name
        # TODO whatif the face is not properly seen ? --> Make specific scenario view that sends feedback !
        # state_learnPeopleMeta, result_learnPeopleMeta = self.learnPeopleMetaFromImgTopic(self.guest_2_name, 10.0)

        # Then ask drink
        askinfog2_ask_drink_max_counts = 3  # TODO Move this as config parameter
        askinfog2_ask_drink = self.find_by_id(steps, "askinfog2_ask-drink")
        askinfog2_confirm_drink = self.find_by_id(steps, "askinfog2_confirm-drink")
        self.guest_2_drink = self.ask_drink_and_confirm(askinfog2_ask_drink_max_counts, askinfog2_ask_drink, askinfog2_confirm_drink)

        # - Ask age
        askinfog2_ask_age = self.find_by_id(steps, "askinfog2_ask-age")
        askinfog2_ask_age_speech = askinfog2_ask_age["speech"]
        askinfog2_ask_age_speech["name"] = self.guest_2_name
        self.guest_2_age = self._lm_wrapper.ask_age(askinfog2_ask_age_speech, self.NO_TIMEOUT)[1]

        self._lm_wrapper.timeboard_send_step_done(step_id_to_index["AskInfoG2"], self.NO_TIMEOUT)

        # ###################################################################################################

        # Go to living room
        self._lm_wrapper.timeboard_set_current_step(step_id_to_index["GotoLR2"], self.NO_TIMEOUT)
        global_step_go_to_lr2_start_time = time.time()

        # - Ask to follow
        gotolr2_ask_to_follow = self.find_by_id(steps, "gotolr2_ask-to-follow")
        gotolr2_ask_to_follow["speech"]["name"] = self.guest_2_name
        self._lm_wrapper.ask_to_follow(gotolr2_ask_to_follow["speech"], self._living_room, self.NO_TIMEOUT)

        # - Go to living room
        gotolr2_go_to_living_room = self.find_by_id(steps, "gotolr2_go-to-living-room")
        self._lm_wrapper.go_to(gotolr2_go_to_living_room["speech"], self._living_room, self.NO_TIMEOUT)
        self.moveheadPose(self.HEAD_PITCH_FOR_NAV_POSE, self.HEAD_YAW_CENTER, True)  # Reset Head position to navigate
        # self.sendNavOrderAction("NP", "CRRCloseToGoal", gotolr2_go_to_living_room["arguments"]["interestPoint"], 50.0)

        self._lm_wrapper.timeboard_send_step_done(step_id_to_index["GotoLR2"], self.NO_TIMEOUT)

        # ###################################################################################################
        #
        # # Introduce second guest to others
        # self._lm_wrapper.timeboard_set_current_step(step_id_to_index["IntroduceG2ToOthers"], self.NO_TIMEOUT)
        #
        # self._lm_wrapper.timeboard_send_step_done(step_id_to_index["IntroduceG2ToOthers"], self.NO_TIMEOUT)
        #
        # ###################################################################################################
        #
        # # Introduce John to second guest
        # self._lm_wrapper.timeboard_set_current_step(step_id_to_index["IntroduceJohnToG2"], self.NO_TIMEOUT)

        # self._lm_wrapper.timeboard_send_step_done(step_id_to_index["IntroduceJohnToG2"], self.NO_TIMEOUT)
        #
        # ###################################################################################################
        #
        # # Introduce first guest to second guest
        # self._lm_wrapper.timeboard_set_current_step(step_id_to_index["IntroduceG1ToG2"], self.NO_TIMEOUT)
        #
        # self._lm_wrapper.timeboard_send_step_done(step_id_to_index["IntroduceG1ToG2"], self.NO_TIMEOUT)
        #
        # ###################################################################################################

        # Seat second guest
        self._lm_wrapper.timeboard_set_current_step(step_id_to_index["SeatG2"], self.NO_TIMEOUT)
        global_step_seat_g2_start_time = time.time()

        # - Find empty seat
        self.simulate_ros_work(1.0, "SIMULATING FINDING AN EMPTY SEAT")

        # - Point to empty seat
        self.simulate_ros_work(1.0, "SIMULATING POINTING TO EMPTY SEAT")

        # - Tell first guest to seat
        seat_g2 = self.find_by_id(steps, "seatg2_tell-first-guest-to-seat")
        seat_g2["speech"]["name"] = self.guest_2_name
        self._lm_wrapper.seat_guest(seat_g2["speech"], self.NO_TIMEOUT)

        self._lm_wrapper.timeboard_send_step_done(step_id_to_index["SeatG2"], self.NO_TIMEOUT)

        # ###################################################################################################

        # Finish Scenario
        self._lm_wrapper.timeboard_set_current_step(step_id_to_index["FinishScenario"], self.NO_TIMEOUT)

        # - Finish Scenario
        # self.send_action_to_local_manager("finishscenario_finish-scenario")

        self._lm_wrapper.timeboard_send_step_done(step_id_to_index["FinishScenario"], self.NO_TIMEOUT)

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
        self._enableObjectDetectionMngAction = False
        self._enableLookAtObjectMngAction = False
        self._enableMultiplePeopleDetectionAction = False
        self._enableRequestToLocalManagerAction = True
        self._enableLearnPeopleMetaAction = False
        self._enableGetPeopleNameAction = False

        self._enableMoveHeadPoseService = True

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

    def ask_name_and_confirm(self, ask_name_max_count, ask_step_data, confirm_step_data):
        ask_name_counter = 0
        while True:

            # - Ask name
            tentative_guest_name = self._lm_wrapper.ask_name(ask_step_data["speech"], self._people, self.NO_TIMEOUT)[1]

            # - Confirm name
            confirm_speech = confirm_step_data["speech"]
            confirm_speech["name"] = tentative_guest_name
            ask_name_confirmed = self._lm_wrapper.confirm(confirm_speech, self.NO_TIMEOUT)[1]
            if ask_name_confirmed:
                rospy.loginfo("Guest got name {name} confirmed !".format(name=tentative_guest_name))
                return tentative_guest_name

            ask_name_counter += 1

            if ask_name_counter >= ask_name_max_count:
                rospy.logwarn("Could not get name with confirmation !")
                # TODO : Do TTS action where robot says something like: Hmmm, I really can't understand what you say,
                #  I guess I will just call you {Last_Understood Name}
                return tentative_guest_name

    def ask_drink_and_confirm(self, ask_drink_max_count, ask_step_data, confirm_step_data):
        ask_drink_counter = 0
        while True:
            # - Ask drink
            ask_speech = ask_step_data["speech"]
            ask_speech["name"] = self.guest_1_name
            tentative_guest_drink = self._lm_wrapper.ask_drink(ask_speech, self._drinks, self.NO_TIMEOUT)[1]

            # - Confirm drink
            confirm_speech = confirm_step_data["speech"]
            confirm_speech["drink"] = tentative_guest_drink["name"]
            ask_drink_confirmed = self._lm_wrapper.confirm(confirm_speech, self.NO_TIMEOUT)[1]
            if ask_drink_confirmed:
                rospy.loginfo("Guest got drink <{drink}> confirmed !".format(drink=tentative_guest_drink["name"]))
                return tentative_guest_drink

            ask_drink_counter += 1

            if ask_drink_counter >= ask_drink_max_count:
                rospy.logwarn("Could not get drink with confirmation !")
                # TODO : Do TTS action where robot says something like: Hmmm, I really can't understand what you say,
                #  I guess I will just consider you like {Last_Understood Drink}
                return tentative_guest_drink
