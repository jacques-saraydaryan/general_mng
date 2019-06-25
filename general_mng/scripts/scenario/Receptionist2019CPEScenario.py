__author__ = 'Benoit Renault'
import rospy

from AbstractScenario import AbstractScenario
from AbstractScenarioAction import AbstractScenarioAction
from AbstractScenarioBus import AbstractScenarioBus
from AbstractScenarioService import AbstractScenarioService
from LocalManagerWrapper import LocalManagerWrapper

import json
import time


class Receptionist2019CPEScenario(AbstractScenario, AbstractScenarioBus,
                                  AbstractScenarioAction, AbstractScenarioService):
    DEFAULT_TIMEOUT = 5.0
    NO_TIMEOUT = -1.0

    def __init__(self, config):
        AbstractScenarioBus.__init__(self, config)
        AbstractScenarioAction.__init__(self, config)
        # self._lm_wrapper = LocalManagerWrapper(config.ip_address, config.tcp_port, config.prefix)

        # TODO : Remove Hardocoded values and get them from config
        self._lm_wrapper = LocalManagerWrapper("192.168.1.189", 9559, "R2019")

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
        self._living_room = self.find_location(self._locations, "livingRoom")

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

        self._lm_wrapper.timeboard_set_timer_state(True, self.NO_TIMEOUT)
        global_step_find_g1_start_time = time.time()

        # - Wait
        findg1_wait = self.find_step(steps, "findg1_wait")
        self._lm_wrapper.wait(findg1_wait["speech"], findg1_wait["arguments"]["time"], findg1_wait["arguments"]["time"] + 2.0)

        # - Ask referee to open the door
        findg1_ask_referee = self.find_step(steps, "findg1_ask-referee-to-open-the-door")
        self._lm_wrapper.ask_open_door(findg1_ask_referee["speech"], self.NO_TIMEOUT)

        # - Wait2
        findg1_wait2 = self.find_step(steps, "findg1_wait2")
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

        askinfog1_ask_name_counter = 0
        askinfog1_ask_name_max_counts = 3
        askinfog1_ask_name = self.find_step(steps, "askinfog1_ask-name")
        askinfog1_confirm_name = self.find_step(steps, "askinfog1_confirm-name")
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

        askinfog1_ask_drink_counter = 0
        askinfog1_ask_drink_max_counts = 3
        askinfog1_ask_drink = self.find_step(steps, "askinfog1_ask-drink")
        askinfog1_confirm_drink = self.find_step(steps, "askinfog1_confirm-drink")
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
            confirm_speech["drink"] = tentative_guest_1_drink
            askinfog1_ask_drink_confirmed = self._lm_wrapper.confirm(confirm_speech, self.NO_TIMEOUT)[1]
            if askinfog1_ask_drink_confirmed:
                rospy.loginfo("Guest 1 got drink {drink} confirmed !".format(drink=tentative_guest_1_drink))
                self.guest_1_drink = tentative_guest_1_drink
                break

            askinfog1_ask_drink_counter += 1

        # - Ask age
        askinfog1_ask_age = self.find_step(steps, "askinfog1_ask-age")
        askinfog1_ask_age_speech = askinfog1_ask_age["speech"]
        askinfog1_ask_age_speech["name"] = self.guest_1_name
        self.guest_1_age = self._lm_wrapper.ask_age(askinfog1_ask_age_speech, self.NO_TIMEOUT)[1]

        self._lm_wrapper.timeboard_send_step_done(step_id_to_index["AskInfoG1"], self.NO_TIMEOUT)

        ###################################################################################################

        # Go to living room
        self._lm_wrapper.timeboard_set_current_step(step_id_to_index["GotoLR1"], self.NO_TIMEOUT)

        # global_step_go_to_lr1_start_time = time.time()

        # - Ask to follow
        gotolr1_ask_to_follow = self.find_step(steps, "gotolr1_ask-to-follow")
        self._lm_wrapper.ask_to_follow(gotolr1_ask_to_follow["speech"], self._living_room, self.NO_TIMEOUT)

        # - Go to living room
        gotolr1_go_to_living_room = self.find_step(steps, "gotolr1_go-to-living-room")
        self._lm_wrapper.go_to(gotolr1_go_to_living_room["speech"], self._living_room, self.NO_TIMEOUT)
        self.moveheadPose(self.HEAD_PITCH_FOR_NAV_POSE, self.HEAD_YAW_CENTER, True)  # Reset Head position to navigate
        self.sendNavOrderAction("NP", "CRRCloseToGoal", gotolr1_go_to_living_room["arguments"]["interestPoint"], 50.0)

        self._lm_wrapper.timeboard_send_step_done(step_id_to_index["GotoLR1"], self.NO_TIMEOUT)

        ###################################################################################################

        # Introduce first guest to John
        self._lm_wrapper.timeboard_set_current_step(step_id_to_index["IntroduceG1ToJohn"], self.NO_TIMEOUT)

        self.moveheadPose(self.HEAD_PITCH_FOR_SPEECH_POSE, self.HEAD_YAW_CENTER, True)  # Reset Head position to talk
        # global_step_introduce_g1_to_john_start_time = time.time()

        # - Point to first guest
        self.send_action_to_local_manager("introduceg1tojohn_point-to-first-guest")
        self.simulate_ros_work(1.0, "SIMULATING POINTING TO FIRST GUEST ")

        # - Say name and drink
        self.send_action_to_local_manager("introduceg1tojohn_say-name-and-drink")

        self._lm_wrapper.timeboard_send_step_done(step_id_to_index["IntroduceG1ToJohn"], self.NO_TIMEOUT)

        ###################################################################################################

        # Introduce John to first guest
        self._lm_wrapper.timeboard_set_current_step(step_id_to_index["IntroduceJohnToG1"], self.NO_TIMEOUT)

        # global_step_introduce_john_to_g1_start_time = time.time()

        # - Point to John
        self.send_action_to_local_manager("introducejohntog1_point-to-john")
        self.simulate_ros_work(1.0, "SIMULATING POINTING TO JOHN ")

        # - Say name and drink
        self.send_action_to_local_manager("introducejohntog1_say-name-and-drink")

        self._lm_wrapper.timeboard_send_step_done(step_id_to_index["IntroduceJohnToG1"], self.NO_TIMEOUT)

        ###################################################################################################

        # Seat first guest
        self._lm_wrapper.timeboard_set_current_step(step_id_to_index["SeatG1"], self.NO_TIMEOUT)
        # global_step_seat_g1_start_time = time.time()

        # - Find empty seat
        self.send_action_to_local_manager("seatg1_find-empty-seat")
        self.simulate_ros_work(1.0, "SIMULATING FINDING AN EMPTY SEAT")

        # - Point to empty seat
        self.send_action_to_local_manager("seatg1_point-to-empty-seat")
        self.simulate_ros_work(1.0, "SIMULATING POINTING TO EMPTY SEAT")

        # - Tell first guest to seat
        self.send_action_to_local_manager("seatg1_tell-first-guest-to-seat")

        self._lm_wrapper.timeboard_send_step_done(step_id_to_index["SeatG1"], self.NO_TIMEOUT)

        ###################################################################################################

        # TODO If time too short, don't try to bring second guest ?

        # Go to door
        self._lm_wrapper.timeboard_set_current_step(step_id_to_index["GotoDoor1"], self.NO_TIMEOUT)
        # global_step_find_go_to_door1_start_time = time.time()

        # - Go to door
        self.send_action_to_local_manager("gotodoor1_go-to-door")
        self.moveheadPose(self.HEAD_PITCH_FOR_NAV_POSE, self.HEAD_YAW_CENTER, True)  # Reset Head position to navigate
        self.sendNavOrderAction("NP", "CRRCloseToGoal", self.find_in_steps_by_id(steps, "gotodoor1_go-to-door")["arguments"]["interestPoint"], 50.0)

        self._lm_wrapper.timeboard_send_step_done(step_id_to_index["GotoDoor1"], self.NO_TIMEOUT)

        ###################################################################################################

        # Find second guest
        self._lm_wrapper.timeboard_set_current_step(step_id_to_index["FindG2"], self.NO_TIMEOUT)
        self.moveheadPose(self.HEAD_PITCH_FOR_SPEECH_POSE, self.HEAD_YAW_CENTER, True)  # Reset Head position to talk
        # global_step_find_g2_start_time = time.time()

        # - Wait
        findg2_wait = self.find_step(steps, "findg2_wait")
        self.wait(findg2_wait["speech"], findg2_wait["arguments"]["time"], findg2_wait["arguments"]["time"] + 2.0)

        # - Ask referee to open the door
        self.send_action_to_local_manager("findg2_ask-referee-to-open-the-door")

        # - Wait2
        findg2_wait2 = self.find_step(steps, "findg2_wait2")
        self.wait(findg2_wait2["speech"], findg2_wait2["arguments"]["time"], findg2_wait2["arguments"]["time"] + 2.0)

        # - Detect human
        self.send_action_to_local_manager("findg2_detect-human")
        self.simulate_ros_work(1.0, "SIMULATING HUMAN DETECTION")

        self._lm_wrapper.timeboard_send_step_done(step_id_to_index["FindG2"], self.NO_TIMEOUT)

        ###################################################################################################

        # Ask infos about second guest
        self._lm_wrapper.timeboard_set_current_step(step_id_to_index["AskInfoG2"], self.NO_TIMEOUT)
        # global_step_ask_info_g2_start_time = time.time()

        askinfog2_ask_name_counter = 0
        askinfog2_ask_name_max_counts = 3
        askinfog2_ask_name_return_value = ""
        askinfog2_ask_name = self.find_step(steps, "askinfog2_ask-name")
        while askinfog2_ask_name_return_value != "ok":

            if askinfog2_ask_name_counter >= askinfog2_ask_name_max_counts:  # Reproduce bug of wrong text for ask drink that displays ask name : change >= to <
                rospy.logwarn("Could not get name with confirmation !")
                ## TODO : Do TTS action where robot says something like: Hmmm, I really can't understand what you say, I guess I will just call you {Last_Understood Name}
                break

            # - Ask name
            self._lm_wrapper.ask_name(askinfog2_ask_name["speech"], self._people, self.NO_TIMEOUT)

            # - Confirm name
            askinfog2_ask_name_return_value = self.send_action_to_local_manager("askinfog2_confirm-name")

            askinfog2_ask_name_counter += 1

        askinfog2_ask_drink_counter = 0
        askinfog2_ask_drink_max_counts = 3
        askinfog2_ask_drink_return_value = ""
        while askinfog2_ask_drink_return_value != "ok":

            if askinfog2_ask_drink_counter >= askinfog2_ask_drink_max_counts:  # Reproduce bug of wrong text for ask drink that displays ask name : change >= to <
                rospy.logwarn("Could not get drink with confirmation !")
                ## TODO : Do TTS action where robot says something like: Hmmm, I really can't understand what you say, I guess I will just consider you like {Last_Understood Drink}
                break

            # - Ask drink
            self.send_action_to_local_manager("askinfog2_ask-drink")

            # - Confirm drink
            askinfog2_ask_drink_return_value = self.send_action_to_local_manager("askinfog2_confirm-drink")

            askinfog2_ask_drink_counter += 1

        # - Ask age

        self.send_action_to_local_manager("askinfog2_ask-age")

        self._lm_wrapper.timeboard_send_step_done(step_id_to_index["AskInfoG2"], self.NO_TIMEOUT)

        ###################################################################################################

        # Go to living room
        self._lm_wrapper.timeboard_set_current_step(step_id_to_index["GotoLR2"], self.NO_TIMEOUT)
        global_step_go_to_lr2_start_time = time.time()

        # - Ask to follow
        self.send_action_to_local_manager("gotolr2_ask-to-follow")

        # - Go to living room
        self.send_action_to_local_manager("gotolr2_go-to-living-room")
        self.moveheadPose(self.HEAD_PITCH_FOR_NAV_POSE, self.HEAD_YAW_CENTER, True)  # Reset Head position to navigate
        self.sendNavOrderAction("NP", "CRRCloseToGoal", self.find_step(steps, "gotolr2_go-to-living-room")["arguments"]["interestPoint"], 50.0)

        self._lm_wrapper.timeboard_send_step_done(step_id_to_index["GotoLR2"], self.NO_TIMEOUT)

        ###################################################################################################

        # Introduce second guest to others
        self._lm_wrapper.timeboard_set_current_step(step_id_to_index["IntroduceG2ToOthers"], self.NO_TIMEOUT)
        self.moveheadPose(self.HEAD_PITCH_FOR_SPEECH_POSE, self.HEAD_YAW_CENTER, True)  # Reset Head position to talk
        global_step_introduce_g2_to_others_start_time = time.time()

        # - Point to second guest
        self.send_action_to_local_manager("introduceg2toothers_point-to-second-guest")
        self.simulate_ros_work(1.0, "SIMULATING POINTING TO SECOND GUEST ")

        # - Say name and drink
        self.send_action_to_local_manager("introduceg2toothers_say-name-and-drink")

        self._lm_wrapper.timeboard_send_step_done(step_id_to_index["IntroduceG2ToOthers"], self.NO_TIMEOUT)

        ###################################################################################################

        # Introduce John to second guest
        self._lm_wrapper.timeboard_set_current_step(step_id_to_index["IntroduceJohnToG2"], self.NO_TIMEOUT)
        global_step_introduce_john_to_g2_start_time = time.time()

        # - Point to John
        self.send_action_to_local_manager("introducejohntog2_point-to-john")
        self.simulate_ros_work(1.0, "SIMULATING POINTING TO JOHN ")

        # - Say name and drink
        self.send_action_to_local_manager("introducejohntog2_say-name-and-drink")

        self._lm_wrapper.timeboard_send_step_done(step_id_to_index["IntroduceJohnToG2"], self.NO_TIMEOUT)

        ###################################################################################################

        # Introduce first guest to second guest
        self._lm_wrapper.timeboard_set_current_step(step_id_to_index["IntroduceG1ToG2"], self.NO_TIMEOUT)
        global_step_introduce_g1_to_g2_start_time = time.time()

        # - Point to first guest
        self.send_action_to_local_manager("introduceg1tog2_point-to-first-guest")
        self.simulate_ros_work(1.0, "SIMULATING POINTING TO FIRST GUEST ")

        # - Say name and drink
        self.send_action_to_local_manager("introduceg1tog2_say-name-and-drink")

        self._lm_wrapper.timeboard_send_step_done(step_id_to_index["IntroduceG1ToG2"], self.NO_TIMEOUT)

        ###################################################################################################

        # Seat second guest
        self._lm_wrapper.timeboard_set_current_step(step_id_to_index["SeatG2"], self.NO_TIMEOUT)
        global_step_seat_g2_start_time = time.time()

        # - Find empty seat
        self.send_action_to_local_manager("seatg2_find-empty-seat")
        self.simulate_ros_work(1.0, "SIMULATING FINDING AN EMPTY SEAT")

        # - Point to empty seat
        self.send_action_to_local_manager("seatg2_point-to-empty-seat")
        self.simulate_ros_work(1.0, "SIMULATING POINTING TO EMPTY SEAT")

        # - Tell first guest to seat
        self.send_action_to_local_manager("seatg2_tell-first-guest-to-seat")

        self._lm_wrapper.timeboard_send_step_done(step_id_to_index["SeatG2"], self.NO_TIMEOUT)

        ###################################################################################################

        # Finish Scenario
        self._lm_wrapper.timeboard_set_current_step(step_id_to_index["FinishScenario"], self.NO_TIMEOUT)

        # - Finish Scenario
        self.send_action_to_local_manager("finishscenario_finish-scenario")

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

        self._enableMoveHeadPoseService = True

        AbstractScenarioAction.configure_intern(self)
        AbstractScenarioService.configure_intern(self)

    def simulate_ros_work(self, time_for_work, log_string):
        rospy.logwarn(log_string)
        rospy.logwarn("Waiting for {duration} seconds...".format(duration=time_for_work))
        time.sleep(time_for_work)

    def find_step(self, steps_array, step_id):
        step_index = self.find_step_index(steps_array, step_id)
        if step_index is None:
            return None
        else:
            return steps_array[step_index]

    def find_step_index(self, steps_array, step_id):
        for index in range(len(steps_array)):
            step = steps_array[index]
            if step["id"] == step_id:
                return index
        return None

    def find_location(self, locations_array, location_id):
        for location in locations_array:
            if location["id"] == location_id:
                return location
