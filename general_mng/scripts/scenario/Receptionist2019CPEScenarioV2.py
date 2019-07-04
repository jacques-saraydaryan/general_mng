__author__ = 'Benoit Renault'
import rospy
import collections
import os, sys

from AbstractScenario import AbstractScenario
from AbstractScenarioAction import AbstractScenarioAction
from AbstractScenarioBus import AbstractScenarioBus
from AbstractScenarioService import AbstractScenarioService
from LocalManagerWrapper import LocalManagerWrapper

import json
import time
import math
import Image


class Receptionist2019CPEScenarioV2(AbstractScenario, AbstractScenarioBus,
                                    AbstractScenarioAction, AbstractScenarioService):
    DEFAULT_TIMEOUT = 5.0
    NO_TIMEOUT = -1.0

    def __init__(self, config):
        AbstractScenarioBus.__init__(self, config)
        AbstractScenarioAction.__init__(self, config)

        # Node Configuration
        self.__configure()

        # Init wrapper
        self._lm_wrapper = LocalManagerWrapper(self.nao_ip, self.nao_port, self.lm_prefix)

        # with open(config.scenario_filepath) as data:
        with open("{0}/receptionistV2/scenario.json".format(self.jsons_data_folder)) as data:
            self._scenario = json.load(data)

        with open("{0}/drinks.json".format(self.jsons_data_folder)) as data:
            self._drinks = json.load(data)

        with open("{0}/locations.json".format(self.jsons_data_folder)) as data:
            self._locations = json.load(data)

        with open("{0}/people.json".format(self.jsons_data_folder)) as data:
            self._people = json.load(data)

        with open("{0}/videos.json".format(self.jsons_data_folder)) as data:
            self._videos = json.load(data)


        # Scenario data
        self.people_name_by_id = {}
        self.people_image_by_id = {}
        self.people_drink_by_id = {}
        self.people_age_by_id = {}
        self.steps = None

        # - Constants
        self._living_room = self.find_by_id(self._locations, "livingRoom")
        self._entrance = self.find_by_id(self._locations, "entrance")
        self.people_name_by_id[0] = {}
        self.people_name_by_id[0] = "John"
        self.people_image_by_id[0] = None
        self.people_age_by_id[0] = 23
        self.people_drink_by_id[0] = self.find_by_id(self._drinks, "coke")

        # Debug options
        self.allow_navigation = False
        self.recog_with_picture = False

    def __configure(self):
        """
        Get all parameters for the node.
        """
        self.nao_ip = rospy.get_param("nao_ip", "10.10.65.3")
        self.nao_port = rospy.get_param("nao_port", "9559")
        self.lm_prefix = rospy.get_param("prefix", "R2019")
        self.jsons_data_folder = rospy.get_param("jsons_data_folder", "/home/xia0ben/pepper_ws/src/robocup-main/robocup_pepper-scenario_data_generator/jsons")
        self.pictures_folder = rospy.get_param("pictures_folder", "/home/xia0ben/pepper_ws/data/pictures")


    def startScenario(self):
        rospy.loginfo("""
        ######################################
        Starting the {scenario_name} Scenario...
        ######################################
        """.format(scenario_name=self._scenario["name"]))

        self.steps = self._scenario["steps"]

        ###################################################################################################
        # Reset people database
        self.resetPeopleMetaInfoMap()

        # Learn John face
        picture_file_path = "{0}/John.png".format(self.pictures_folder)
        state_learnPeopleMeta, result_learnPeopleMeta = self.learnPeopleMetaFromImgPath( picture_file_path, self.people_name_by_id[0], 10.0)
        remote_file_path = "/home/nao/.local/share/PackageManager/apps/R2019/html/img/peoples/{0}.png".format(self.people_name_by_id[0])
        os.system('scp "{0}" "nao@{1}:{2}"'.format(picture_file_path, self.nao_ip, remote_file_path) )
        self.people_image_by_id[0] = "img/peoples/{0}.png".format(self.people_name_by_id[0])

        ##################################################################################################
        # Start timeboard to follow scenario evolution on screen

        # Remember the dictionary that associates big steps to the array that was sent to the local manager
        step_id_to_index = self._lm_wrapper.timeboard_send_steps_list(
            self.steps, self._scenario["name"], self.NO_TIMEOUT)[1]
        self._lm_wrapper.timeboard_set_timer_state(True, self.NO_TIMEOUT)

        scenario_start_time = time.time()

        ###################################################################################################

        # Go to door
        self._lm_wrapper.timeboard_set_current_step(step_id_to_index["GotoDoor0"], self.NO_TIMEOUT)

        # - Go to door
        gotodoor0_go_to = self.find_by_id(self.steps, "gotodoor0_go_to")
        self._lm_wrapper.go_to(gotodoor0_go_to["speech"], gotodoor0_go_to["arguments"]["location"], self.NO_TIMEOUT)
        self.moveheadPose(self.HEAD_PITCH_FOR_NAV_POSE, self.HEAD_YAW_CENTER, True)  # Reset Head position to navigate
        if self.allow_navigation: self.sendNavOrderAction("NP", "CRRCloseToGoal", "ENTRANCE_RECEPTIONIST_WAIT_GUEST_01", 120.0)

        self._lm_wrapper.timeboard_send_step_done(step_id_to_index["GotoDoor0"], self.NO_TIMEOUT)

        ###################################################################################################

        # Have first guest enter
        self._lm_wrapper.timeboard_set_current_step(step_id_to_index["EnterG1"], self.NO_TIMEOUT)

        # - Reset Head position to talk
        self.moveheadPose(self.HEAD_PITCH_FOR_SPEECH_POSE, self.HEAD_YAW_CENTER, True)

        # - Ask referee to open the door
        enterg1_ask_referee = self.find_by_id(self.steps, "enterg1_ask_referee")
        enterg1_confirm_referee = self.find_by_id(self.steps, "enterg1_confirm_referee")
        self.ask_validation_and_confirm(enterg1_ask_referee, enterg1_confirm_referee)

        # - Ask guest to enter
        enterg1_ask_guest = self.find_by_id(self.steps, "enterg1_ask_guest")
        enterg1_confirm_guest = self.find_by_id(self.steps, "enterg1_confirm_guest")
        self.ask_validation_and_confirm(enterg1_ask_guest, enterg1_confirm_guest)

        # - Detect human
        # findg1_detect_human = self.find_step(self.steps, "findg1_detect-human")
        # self._lm_wrapper.call_human(findg1_detect_human["speech"], 3.0, self.NO_TIMEOUT)
        # self.simulate_ros_work(1.0, "SIMULATING HUMAN DETECTION")

        self._lm_wrapper.timeboard_send_step_done(step_id_to_index["EnterG1"], self.NO_TIMEOUT)

        ###################################################################################################

        # Ask infos about first guest
        self._lm_wrapper.timeboard_set_current_step(step_id_to_index["AskInfoG1"], self.NO_TIMEOUT)

        # global_step_ask_info_g1_start_time = time.time()

        # - Ask name
        askinfog1_ask_name_max_counts = 3  # TODO Move this as config parameter
        askinfog1_ask_name = self.find_by_id(self.steps, "askinfog1_ask_name")
        askinfog1_confirm_name = self.find_by_id(self.steps, "askinfog1_confirm_name")
        self.people_name_by_id[1] = self.ask_name_and_confirm(askinfog1_ask_name_max_counts, askinfog1_ask_name, askinfog1_confirm_name)


        # - Take picture and send it to Pepper
        self.take_photo_and_confirm(guest_id_number=1)

        # - Ask drink
        askinfog1_ask_drink_max_counts = 3  # TODO Move this as config parameter
        askinfog1_ask_drink = self.find_by_id(self.steps, "askinfog1_ask_drink")
        askinfog1_confirm_drink = self.find_by_id(self.steps, "askinfog1_confirm_drink")
        self.people_drink_by_id[1] = self.ask_drink_and_confirm(
            askinfog1_ask_drink_max_counts, askinfog1_ask_drink, askinfog1_confirm_drink, self.people_name_by_id[1])

        # - Ask age
        askinfog1_ask_age_max_counts = 3  # TODO Move this as config parameter
        askinfog1_ask_age = self.find_by_id(self.steps, "askinfog1_ask_age")
        askinfog1_confirm_age = self.find_by_id(self.steps, "askinfog1_confirm_age")
        self.people_age_by_id[1] = self.ask_age_and_confirm(askinfog1_ask_age_max_counts, askinfog1_ask_age, askinfog1_confirm_age)

        self._lm_wrapper.timeboard_send_step_done(step_id_to_index["AskInfoG1"], self.NO_TIMEOUT)

        ###################################################################################################

        # Go to living room
        self._lm_wrapper.timeboard_set_current_step(step_id_to_index["GotoLR1"], self.NO_TIMEOUT)

        # global_step_go_to_lr1_start_time = time.time()

        # - Ask to follow
        gotolr1_ask_to_follow = self.find_by_id(self.steps, "gotolr1_ask_to_follow")
        gotolr1_ask_to_follow["speech"]["name"] = self.people_name_by_id[1]
        self._lm_wrapper.ask_to_follow(gotolr1_ask_to_follow["speech"], self._living_room, self.NO_TIMEOUT)

        # - Go to living room
        gotolr1_go_to = self.find_by_id(self.steps, "gotolr1_go_to")
        self._lm_wrapper.go_to(gotolr1_go_to["speech"], self._living_room, self.NO_TIMEOUT)
        self.moveheadPose(self.HEAD_PITCH_FOR_NAV_POSE, self.HEAD_YAW_CENTER, True)  # Reset Head position to navigate
        if self.allow_navigation: self.moveTurn(math.pi)
        if self.allow_navigation: self.sendNavOrderAction("NP", "CRRCloseToGoal", "ENTRANCE_TO_LIVINGROOM_02", 120.0)
        if self.allow_navigation: self.sendNavOrderAction("NP", "CRRCloseToGoal", "LIVINGROOM_SOFA_OBSERVATION_01", 120.0)

        self._lm_wrapper.timeboard_send_step_done(step_id_to_index["GotoLR1"], self.NO_TIMEOUT)

        ###################################################################################################

        # Introduce guest 1
        self._lm_wrapper.timeboard_set_current_step(step_id_to_index["IntroduceG1"], self.NO_TIMEOUT)

        # - introduceg1_say_intent
        introduceg1_say_intent = self.find_by_id(self.steps, "introduceg1_say_intent")
        self._lm_wrapper.generic(self.NO_TIMEOUT, introduceg1_say_intent["speech"])

        # - Find and introduce people to each others
        self.introduce_people_to_each_others("introduceg1_introduce_guest")

        self._lm_wrapper.timeboard_send_step_done(step_id_to_index["IntroduceG1"], self.NO_TIMEOUT)

        ###################################################################################################

        # Seat first guest
        self._lm_wrapper.timeboard_set_current_step(step_id_to_index["SeatG1"], self.NO_TIMEOUT)
        # global_step_seat_g1_start_time = time.time()

        # - Find empty seat
        self.find_an_empty_chair(1)

        self._lm_wrapper.timeboard_send_step_done(step_id_to_index["SeatG1"], self.NO_TIMEOUT)

        ###################################################################################################

        # TODO If time too short, don't try to bring second guest ?

        # Go to door
        self._lm_wrapper.timeboard_set_current_step(step_id_to_index["GotoDoor1"], self.NO_TIMEOUT)
        # global_step_find_go_to_door1_start_time = time.time()

        # - Go to door
        gotodoor1_go_to = self.find_by_id(self.steps, "gotodoor1_go_to")
        self._lm_wrapper.go_to(gotodoor1_go_to["speech"], self._entrance, self.NO_TIMEOUT)

        # - Reset Head position to navigate
        self.moveheadPose(self.HEAD_PITCH_FOR_NAV_POSE, self.HEAD_YAW_CENTER, True)

        # - Navigate
        if self.allow_navigation: self.sendNavOrderAction("NP", "CRRCloseToGoal", "LIVINGROOM_TO_ENTRANCE_01", 120.0)
        if self.allow_navigation: self.sendNavOrderAction("NP", "CRRCloseToGoal", "ENTRANCE_RECEPTIONIST_WAIT_GUEST_01", 120.0)

        self._lm_wrapper.timeboard_send_step_done(step_id_to_index["GotoDoor1"], self.NO_TIMEOUT)

        # ###################################################################################################

        # Find second guest
        self._lm_wrapper.timeboard_set_current_step(step_id_to_index["EnterG2"], self.NO_TIMEOUT)

        # - Reset Head position to talk
        self.moveheadPose(self.HEAD_PITCH_FOR_SPEECH_POSE, self.HEAD_YAW_CENTER, True)

        # - Ask referee to open the door
        enterg2_ask_referee = self.find_by_id(self.steps, "enterg2_ask_referee")
        enterg2_confirm_referee = self.find_by_id(self.steps, "enterg2_confirm_referee")
        self.ask_validation_and_confirm(enterg2_ask_referee, enterg2_confirm_referee)

        # - Ask guest to enter
        enterg2_ask_guest = self.find_by_id(self.steps, "enterg2_ask_guest")
        enterg2_confirm_guest = self.find_by_id(self.steps, "enterg2_confirm_guest")
        self.ask_validation_and_confirm(enterg2_ask_guest, enterg2_confirm_guest)

        # - Detect human
        # findg2_detect_human = self.find_step(self.steps, "findg2_detect-human")
        # self._lm_wrapper.call_human(findg2_detect_human["speech"], 3.0, self.NO_TIMEOUT)
        # self.simulate_ros_work(1.0, "SIMULATING HUMAN DETECTION")

        self._lm_wrapper.timeboard_send_step_done(step_id_to_index["EnterG2"], self.NO_TIMEOUT)

        # ###################################################################################################

        # Ask infos about second guest
        self._lm_wrapper.timeboard_set_current_step(step_id_to_index["AskInfoG2"], self.NO_TIMEOUT)
        # global_step_ask_info_g2_start_time = time.time()

        #Head's up
        self.moveheadPose(self.HEAD_PITCH_FOR_LOOK_AT_PEOPLE, self.HEAD_YAW_CENTER, True)

        #First Ask name
        askinfog2_ask_name_max_counts = 3  # TODO Move this as config parameter
        askinfog2_ask_name = self.find_by_id(self.steps, "askinfog2_ask_name")
        askinfog2_confirm_name = self.find_by_id(self.steps, "askinfog2_confirm_name")
        self.people_name_by_id[2] = self.ask_name_and_confirm(
            askinfog2_ask_name_max_counts, askinfog2_ask_name, askinfog2_confirm_name)

        #Take a picture and save it
        self.take_photo_and_confirm(guest_id_number=2)

        # Then ask drink
        askinfog2_ask_drink_max_counts = 3  # TODO Move this as config parameter
        askinfog2_ask_drink = self.find_by_id(self.steps, "askinfog2_ask_drink")
        askinfog2_confirm_drink = self.find_by_id(self.steps, "askinfog2_confirm_drink")
        self.people_drink_by_id[2] = self.ask_drink_and_confirm(
            askinfog2_ask_drink_max_counts, askinfog2_ask_drink, askinfog2_confirm_drink, self.people_name_by_id[2])

        # - Ask age
        askinfog2_ask_age_max_counts = 3  # TODO Move this as config parameter
        askinfog2_ask_age = self.find_by_id(self.steps, "askinfog2_ask_age")
        askinfog2_confirm_age = self.find_by_id(self.steps, "askinfog2_confirm_age")
        self.people_age_by_id[2] = self.ask_age_and_confirm(askinfog2_ask_age_max_counts, askinfog2_ask_age, askinfog2_confirm_age)

        self._lm_wrapper.timeboard_send_step_done(step_id_to_index["AskInfoG2"], self.NO_TIMEOUT)

        # ###################################################################################################

        # Go to living room
        self._lm_wrapper.timeboard_set_current_step(step_id_to_index["GotoLR2"], self.NO_TIMEOUT)

        # - Ask to follow
        gotolr2_ask_to_follow = self.find_by_id(self.steps, "gotolr2_ask_to_follow")
        gotolr2_ask_to_follow["speech"]["name"] = self.people_name_by_id[2]
        self._lm_wrapper.ask_to_follow(gotolr2_ask_to_follow["speech"], self._living_room, self.NO_TIMEOUT)

        # - Go to living room
        gotolr2_go_to = self.find_by_id(self.steps, "gotolr2_go_to")
        self._lm_wrapper.go_to(gotolr2_go_to["speech"], self._living_room, self.NO_TIMEOUT)
        self.moveheadPose(self.HEAD_PITCH_FOR_NAV_POSE, self.HEAD_YAW_CENTER, True)  # Reset Head position to navigate
        if self.allow_navigation: self.moveTurn(math.pi)
        if self.allow_navigation: self.sendNavOrderAction("NP", "CRRCloseToGoal", "ENTRANCE_TO_LIVINGROOM_02", 120.0)
        if self.allow_navigation: self.sendNavOrderAction("NP", "CRRCloseToGoal", "LIVINGROOM_SOFA_OBSERVATION_01", 120.0)

        self._lm_wrapper.timeboard_send_step_done(step_id_to_index["GotoLR2"], self.NO_TIMEOUT)

        ###################################################################################################

        # Introduce guest 2
        self._lm_wrapper.timeboard_set_current_step(step_id_to_index["IntroduceG2"], self.NO_TIMEOUT)

        # - introduceg1_say_intent
        introduceg2_say_intent = self.find_by_id(self.steps, "introduceg2_say_intent")
        self._lm_wrapper.generic(self.NO_TIMEOUT, introduceg2_say_intent["speech"])

        # - Find and introduce people to each others
        self.introduce_people_to_each_others("introduceg2_introduce_guest")

        self._lm_wrapper.timeboard_send_step_done(step_id_to_index["IntroduceG2"], self.NO_TIMEOUT)

        ##################################################################################################

        # Seat second guest
        self._lm_wrapper.timeboard_set_current_step(step_id_to_index["SeatG2"], self.NO_TIMEOUT)

        # - Find empty seat
        self.find_an_empty_chair(2)

        self._lm_wrapper.timeboard_send_step_done(step_id_to_index["SeatG2"], self.NO_TIMEOUT)

        # ###################################################################################################

        # Finish Scenario
        self._lm_wrapper.timeboard_set_current_step(step_id_to_index["FinishScenario"], self.NO_TIMEOUT)

        # - Finish Scenario
        self._lm_wrapper.generic(self.NO_TIMEOUT, {"said": "Well, I'm done !",
                                                   "title":"My scenario is finished !"})

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
        self._enableObjectDetectionMngAction = True
        self._enableLookAtObjectMngAction = True
        self._enableMultiplePeopleDetectionAction = False
        self._enableRequestToLocalManagerAction = True
        self._enableLearnPeopleMetaAction = True
        self._enableGetPeopleNameAction = True

        self._enableMoveHeadPoseService = True
        self._enableMoveTurnService = True
        self._enablePointAtService = True
        self._enableResetPersonMetaInfoMapService = True
        self._enableReleaseArmsService = True
        self._enableTakePictureService = True

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

    def take_photo_and_confirm(self, guest_id_number, nb_max_retries=3):
        nb_retries = 0
        while True:
            # - Head's up
            self.moveheadPose(self.HEAD_PITCH_FOR_LOOK_AT_PEOPLE, self.HEAD_YAW_CENTER, True)
            self._lm_wrapper.generic(self.NO_TIMEOUT,
                                     {"said": "Please look at me right in the eyes, and wait while I take a photo of you.",
                                      "title": "Please look at me right in the eyes, and wait while I take a photo of you... \n This may take a little while."})
            # TODO whatif the face is not properly seen ? --> Make specific scenario view that sends feedback !
            picture_file_path = "{0}/{1}.png".format(self.pictures_folder, self.people_name_by_id[1])
            self.takePictureAndSaveIt(picture_file_path)
            remote_file_path = ".local/share/PackageManager/apps/R2019/html/img/peoples/{0}_mod.png".format(
                self.people_name_by_id[1])
            picture_to_send_file_path = "{0}/{1}_mod.png".format(self.pictures_folder, self.people_name_by_id[1])
            relative_file_path = "img/peoples/{0}_mod.png".format(self.people_name_by_id[1])
            self.resize_img(picture_file_path, picture_to_send_file_path)
            scp_command = 'scp "{0}" "nao@{1}:{2}"'.format(picture_to_send_file_path, self.nao_ip, remote_file_path)
            os.system(scp_command)
            self._lm_wrapper.generic(self.NO_TIMEOUT,
                                     speech={
                                         "said": "This is the photograph I have taken.",
                                         "title": "This is the photograph I have taken."},
                                     image={"pathOnTablet": relative_file_path, "alternative": relative_file_path})
            time.sleep(3.0)
            # - Reset Head position to talk
            self.moveheadPose(self.HEAD_PITCH_FOR_SPEECH_POSE, self.HEAD_YAW_CENTER, True)
            ask_photo_confirmed = self._lm_wrapper.confirm(speech={
                                         "said": "Was it your face on the photograph?",
                                         "title": "Was it your face on the photograph?"}, timeout=self.NO_TIMEOUT)[1]

            if ask_photo_confirmed:
                rospy.loginfo("Guest photo confirmed !")
                self.people_image_by_id[guest_id_number] = "img/peoples/{0}.png".format(self.people_name_by_id[guest_id_number])
                # Learn face from name
                state_learnPeopleMeta, result_learnPeopleMeta = self.learnPeopleMetaFromImgPath(picture_file_path,
                                                                                                self.people_name_by_id[
                                                                                                    guest_id_number], 10.0)
                return

            if nb_max_retries >= nb_retries:
                rospy.logwarn("Could not get photo with confirmation, using whatever we have !")
                self.people_image_by_id[guest_id_number] = "img/peoples/{0}.png".format(self.people_name_by_id[guest_id_number])
                # Learn face from name
                state_learnPeopleMeta, result_learnPeopleMeta = self.learnPeopleMetaFromImgPath(picture_file_path,
                                                                                                self.people_name_by_id[
                                                                                                    guest_id_number], 10.0)
                return

            nb_retries += 1

    def resize_img(self, in_image_path, out_image_path):
        size = 512, 512
        if in_image_path != out_image_path:
            try:
                im = Image.open(in_image_path)
                im.thumbnail(size, Image.ANTIALIAS)
                im.save(out_image_path, "PNG")
            except IOError:
                print "cannot create thumbnail for '%s'" % in_image_path

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

    def ask_drink_and_confirm(self, ask_drink_max_count, ask_step_data, confirm_step_data, guest_name):
        ask_drink_counter = 0
        while True:
            # - Ask drink
            ask_speech = ask_step_data["speech"]
            ask_speech["name"] = guest_name
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

    def ask_age_and_confirm(self, ask_age_max_count, ask_step_data, confirm_step_data):
        ask_age_counter = 0
        while True:
            # - Ask age
            ask_speech = ask_step_data["speech"]
            tentative_guest_age = self._lm_wrapper.ask_age(ask_speech, self.NO_TIMEOUT)[1]

            # - Confirm age
            confirm_speech = confirm_step_data["speech"]
            confirm_speech["age"] = tentative_guest_age
            ask_age_confirmed = self._lm_wrapper.confirm(confirm_speech, self.NO_TIMEOUT)[1]
            if ask_age_confirmed:
                rospy.loginfo("Guest got age <{age}> confirmed !".format(age=tentative_guest_age))
                return tentative_guest_age

            ask_age_counter += 1

            if ask_age_counter >= ask_age_max_count:
                rospy.logwarn("Could not get age with confirmation !")
                # TODO : Do TTS action where robot says something like: Hmmm, I really can't understand what you say,
                #  I guess I will just consider you like {Last_Understood Age}
                return tentative_guest_age

    def ask_validation_and_confirm(self, ask_step_data, confirm_step_data):
        ask_validation_max_count = confirm_step_data["arguments"]["nb_max_retries"]
        ask_validation_counter = 0
        while True:
            ask_speech = ask_step_data["speech"]
            confirm_speech = confirm_step_data["speech"]

            if ask_validation_counter == 1:
                ask_speech["said"] = ask_speech["said2"]
                confirm_speech["said"] = confirm_speech["said2"]
            if ask_validation_counter == 2:
                # TODO SEND SIGNAL TO LOCAL MANAGER TO CUT VOCAL RECOGNITION
                ask_speech["said"] = ask_speech["said3"]
                ask_speech["description"] = ask_speech["description3"]
                confirm_speech["said"] = confirm_speech["said3"]

            # - Ask validation
            tentative_validation = self._lm_wrapper.ask_open_door(ask_speech, self.NO_TIMEOUT)

            # - Confirm validation
            ask_validation_confirmed = self._lm_wrapper.confirm(confirm_speech, self.NO_TIMEOUT)[1]
            if ask_validation_confirmed:
                rospy.loginfo("Validation confirmed !")
                return tentative_validation

            ask_validation_counter += 1

            if ask_validation_counter >= ask_validation_max_count:
                rospy.logwarn("Could not get validation with confirmation !")
                # TODO : Do TTS action where robot says something like: Hmmm, I really can't understand what you say,
                #  I guess I will just consider everything is fine.
                return tentative_validation

    def introduce_people_to_each_others_hardcoded(self):
        self.turn_to_interest_point("LIVINGROOM_GUEST_VAGUE_IT_01")
        self.pointAt()
        self._lm_wrapper.generic(self.NO_TIMEOUT,
                                 speech={
                                     "said": "This is {name}, their favorite drink is {drink}".format(name=name, drink=drink),
                                     "title": "This is the photograph I have taken."})
        self.turn_to_interest_point("LIVINGROOM_JUST_VAGUE_IT_01")


    def introduce_people_to_each_others(self, step_id):
        """
        Pepper turn on himself to find people and to introduce the to the group
        """
        # Intialize guests / host presentation
        nb_people_here = len(self.people_name_by_id.keys())
        nb_people_introduced = 0
        people_introduced = {}
        for name in self.people_name_by_id.values():
            people_introduced[name] = False
        # Input
        if self.recog_with_picture:
            # Picture file
            picture_file_path = "{0}/introducing.png".format(self.pictures_folder)
        else:
            # Remap darknet topic
            self.remap_topic("/pepper_robot/camera/front/image_raw", "/darknet/dummy/image")
        # Set head position
        self.moveheadPose(self.HEAD_PITCH_CENTER, self.HEAD_YAW_CENTER, True)
        # Indicator to set head position in the future
        # 0 : head is centered
        # 1 : need to head up
        # 2 : head is up
        person_might_be_standing = 0
        # Angle list to turn around to find guests
        # angle_list = [-25.0, 50.0, -75.0, 100.0, -125.0, 150.0, -175.0, 200.0, -225.0, 250.0, -275.0, 300.0, -325.0, 350.0]
        angle_list = [-25.0, 50.0, -75.0, 100.0, -125.0, 150.0, -175.0, -25.0, -25.0, -25.0, -25.0, -25.0, -25.0, -25.0]
        # Beware the infinity loop
        while True:
            if self.recog_with_picture:
                # Take a picture and save it (3 tries possible)
                for i in range(3):
                    ok_picture = self.takePictureAndSaveIt(picture_file_path)
                    if ok_picture == True:
                        break
                else:
                    rospy.logerr("Couldnot retrieve a picture from the robot. Failed to introduce people.")
                    break
                # Detect persons
                state_getObject, result_getObject = self.detectObjectsWithGivenSightFromImgPath(["person"], picture_file_path, 50.0)
            else:
                # Detect persons
                state_getObject, result_getObject = self.detectObjectsWithGivenSightFromImgTopic(["person"], 50.0)
            # Loop on people found
            if result_getObject is not None:
                if len(result_getObject.labelFound) > 0:
                    # Get people names
                    if self.recog_with_picture:
                        state_getPeopleName, result_getPeopleName = self.getPeopleNameFromImgPath(picture_file_path, 50.0)
                    else:
                        state_getPeopleName, result_getPeopleName = self.getPeopleNameFromImgTopic(50.0)
                    # If we recognize a face
                    if result_getPeopleName is not None:
                        if len(result_getPeopleName.peopleNames) > 0:
                            # Test
                            print result_getPeopleName.peopleNames
                            print result_getPeopleName.peopleNamesScore
                            # Compute people bounding box area
                            name_by_area = {}
                            for people, name in zip(result_getPeopleName.peopleMetaList.peopleList, result_getPeopleName.peopleNames):
                                box_x0 = people.details.boundingBox.points[0].x
                                box_x1 = people.details.boundingBox.points[1].x
                                box_y0 = people.details.boundingBox.points[0].y
                                box_y1 = people.details.boundingBox.points[1].y
                                box_area = abs(box_x0 - box_x1)*abs(box_y0 - box_y1)
                                name_by_area[box_area] = name
                            # Inverse sort of the people aera : closest people first
                            name_by_area_ordered = collections.OrderedDict(sorted(name_by_area.items(), reverse=True))
                            # Test all the names we found
                            for (name_of_people_found, i_name_of_people_found) in zip(name_by_area_ordered.values(), range(len(name_by_area_ordered.values()))):
                                # If the persone has not been recognize we jump to the next one
                                if name_of_people_found == "None":
                                    if person_might_be_standing == 0:
                                        person_might_be_standing = 1
                                    continue
                                # Checked if we find correspondences with any known name
                                for name_of_people_known in self.people_name_by_id.values():
                                    # First possibility : The person found is a guest
                                    if (   (name_of_people_found == name_of_people_known)
                                       and (people_introduced[name_of_people_known] == False)):
                                        # Point to Guest
                                        if self.recog_with_picture:
                                            state_lookAtObject, result_lookAtObject = self.lookAtObjectFromImgPath(["person"], picture_file_path, i_name_of_people_found, False, False, 2, 50.0)
                                        else:
                                            state_lookAtObject, result_lookAtObject = self.lookAtObjectFromImgTopic(["person"], i_name_of_people_found, False, False, 2, 50.0)
                                        # Introduce new_guest_to_john
                                        guest_id = self.people_name_by_id.keys()[self.people_name_by_id.values().index(name_of_people_known)]
                                        self.introduce_guest_to_others(step_id, guest_id)
                                        # Release arm
                                        self.releaseArms()
                                        # Update internal variables
                                        nb_people_introduced += 1
                                        people_introduced[name_of_people_known] = True
                                    # # Second possibility : The person found is the host
                                    # # TODO We find the host by elimination : maybe to improve upon
                                    # elif (   (name_of_people_found == "Unknown")
                                    #      and (people_introduced["John"] == False)):
                                    #     if self.people_image_by_id[0] is None:
                                    #         remote_file_path = "/home/nao/.local/share/PackageManager/apps/R2019/html/img/peoples/John.png"
                                    #         os.system('scp "{0}" "nao@{1}:{2}"'.format(picture_file_path, self.nao_ip, remote_file_path) )
                                    #         self.people_image_by_id[0] = "img/peoples/John.png"
                                    #     # Point to John
                                    #     state_lookAtObject, result_lookAtObject = self.lookAtObjectFromImgPath(["person"], picture_file_path, i_name_of_people_found, False, False, 2, 50.0)
                                    #     # Introduce John to new guest
                                    #     self.introduce_host_to_guests(step_id)
                                    #     # Release arm
                                    #     self.releaseArms()
                                    #     # Update internal variables
                                    #     nb_people_introduced += 1
                                    #     people_introduced["John"] = True
                                    #     # Ask age if we have'nt already got it.
                                    #     if self.people_age_by_id[0] is None:
                                    #         askhost_ask_age_max_counts = 3  # TODO Move this as config parameter
                                    #         askhost_ask_age = self.find_by_id(self.steps, "introduceg1_ask_host_age")
                                    #         askhost_confirm_age = self.find_by_id(self.steps, "introduceg1_confirm_host_age")
                                    #         self.people_age_by_id[0] = self.ask_age_and_confirm(askhost_ask_age_max_counts, askhost_ask_age, askhost_confirm_age)
                                    else:
                                        # Mismatch
                                        pass
            # Check if everyone has been introduced
            if nb_people_introduced < nb_people_here:
                # Set head position
                if person_might_be_standing == 1:
                    # Need to Head up
                    self.moveheadPose(self.HEAD_PITCH_FOR_LOOK_AT_PEOPLE, self.HEAD_YAW_CENTER, True)
                    person_might_be_standing = 2
                else:
                    # Reset Head position
                    if person_might_be_standing == 2:
                        self.moveheadPose(self.HEAD_PITCH_CENTER, self.HEAD_YAW_CENTER, True)
                    # Turn a bit to find someone else
                    if len(angle_list) > 0:
                        angle = angle_list.pop(0)
                        if self.allow_navigation: self.moveTurn(angle*math.pi/180.0)
                        #print "I TURN !!!!"
                    else:
                        break
            else:
                # End introducing
                break
        # Input
        if self.recog_with_picture == False:
            # Remap darknet topic
            self.unremap_topic("/pepper_robot/camera/front/image_raw", "/darknet/dummy/image")
        return

    def introduce_guest_to_others(self, step_id, guest_id):
        """
        Introduce the new guest to other guests
        """
        #
        introduceg_introduce_guest = self.find_by_id(self.steps, step_id)
        introduceg_introduce_guest["speech"]["who1_name"] = self.people_name_by_id[guest_id]
        introduceg_introduce_guest["speech"]["who1_drink"] = self.people_drink_by_id[guest_id]["name"]
        self._lm_wrapper.introduce(
            introduceg_introduce_guest["speech"],
            self.people_name_by_id[guest_id], self.people_drink_by_id[guest_id], self.people_image_by_id[guest_id], self.NO_TIMEOUT)

    def introduce_host_to_guests(self, step_id):
        """
        Introduce the host to a given guest
        """
        introduceh_introduce_guest = self.find_by_id(self.steps, step_id)
        introduceh_introduce_guest["speech"]["who1_name"] = self.people_name_by_id[0]
        introduceh_introduce_guest["speech"]["who1_drink"] = self.people_drink_by_id[0]["name"]
        self._lm_wrapper.introduce(
            introduceh_introduce_guest["speech"],
            self.people_name_by_id[0], self.people_drink_by_id[0], self.people_image_by_id[0], self.NO_TIMEOUT)

    def find_an_empty_chair(self, guest_id):
        """
        Find an empty chair for a guest
        """
        # Remap darknet topic
        self.remap_topic("/pepper_robot/camera/front/image_raw", "/darknet/dummy/image")
        # Tell guest we are looking for a chair
        seatg_find_empty_chair = self.find_by_id(self.steps, "seatg{0}_find_empty_seat".format(guest_id))
        self._lm_wrapper.generic(self.NO_TIMEOUT, seatg_find_empty_chair["speech"])
        # Set head position
        self.moveheadPose(self.HEAD_PITCH_FOR_LOOK_FOR_CHAIR, self.HEAD_YAW_CENTER, True)
        # Turn around to introduce guests
        # angle_list = [-25.0, 50.0, -75.0, 100.0, -125.0, 150.0, -175.0, 200.0, -225.0, 250.0, -275.0, 300.0, -325.0, 350.0]
        angle_list = [-25.0, 50.0, -75.0, 100.0, -125.0, 150.0, -175.0, -25.0, -25.0, -25.0, -25.0, -25.0, -25.0, -25.0]
        for angle in angle_list:
            # Find and point the chair in the image
            state_lookAtObject, result_lookAtObject = self.lookAtObjectFromImgTopic(["bench", "sofa"], 0, False, False, 2, 50.0)
            # Loop on people found
            if result_lookAtObject is not None:
                if result_lookAtObject.nb_label > 0:
                    self.sit_here_guest(guest_id)
                    self.releaseArms()
                    return
            # Turn a bit to find somewhere else
            if self.allow_navigation: self.moveTurn(angle*math.pi/180.0)
        # Remap darknet topic
        self.unremap_topic("/pepper_robot/camera/front/image_raw", "/darknet/dummy/image")
        return

    def sit_here_guest(self, guest_id):
        """
        Say to a guest to sit here
        """
        seat_g = self.find_by_id(self.steps, "seatg{0}_tell_guest_to_seat".format(guest_id))
        seat_g["speech"]["name"] = self.people_name_by_id[guest_id]
        self._lm_wrapper.seat_guest(seat_g["speech"], self.NO_TIMEOUT)
