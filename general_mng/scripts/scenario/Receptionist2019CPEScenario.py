__author__ = 'Benoit Renault'
import rospy

from AbstractScenario import AbstractScenario
from AbstractScenarioAction import AbstractScenarioAction
from AbstractScenarioBus import AbstractScenarioBus
from AbstractScenarioService import AbstractScenarioService

import json
import os
import threading
import time

import qi


class Receptionist2019CPEScenario(AbstractScenario, AbstractScenarioBus,
                                  AbstractScenarioAction, AbstractScenarioService):

    def __init__(self, config):
        AbstractScenarioBus.__init__(self, config)
        AbstractScenarioAction.__init__(self, config)

        self.url = "tcp://" + "192.168.1.189" + ":9559"
        self.session = qi.Session()
        self.session.connect(self.url)
        self.memory = self.session.service("ALMemory")

        self.api_folder = "/home/xia0ben/pepper_ws/src/robocup-main/robocup_pepper-hri_meta/robocup_pepper-hri_python/api"

        with open(os.path.join(self.api_folder, "common.json")) as common_api:
            with open(os.path.join(self.api_folder, "generalManagerToHRI.json")) as gmToHri:
                self.apis = {
                    "common": json.load(common_api),
                    "gmToHRI": json.load(gmToHri)
                }

        self.callback_ids = {}
        self.steps = []
        self.curr_step_ind = 0

        self.hri_work_return_string = ""

        receptionist_scenario_filepath = "/home/xia0ben/pepper_ws/src/robocup-main/robocup_pepper-scenario_data_generator/jsons/receptionist/scenario.json"
        with open(receptionist_scenario_filepath) as data:
            self.scenario = json.load(data)

        # TODO END

    def startScenario(self):
        rospy.loginfo("""
        ######################################
        Starting the {scenario_name} Scenario...
        ######################################
        """.format(scenario_name=self.scenario["name"]))

        self.steps = self.scenario["steps"]
        # Change the current scenario
        self.memory.raiseEvent(self.apis["gmToHRI"]["currentScenario"]["ALMemory"],
                               json.dumps({"scenario": self.scenario}))

        # Initialize step listener
        self.attach_event(self.apis["gmToHRI"]["actionComplete"]["ALMemory"], self.handle_hri_job_complete)

        # Start the local manager timer
        self.memory.raiseEvent(self.apis["gmToHRI"]["timerState"]["ALMemory"],
                               json.dumps({"state": self.apis["gmToHRI"]["timerState"]["state"]["on"]}))

        scenario_start_time = time.time()

        ###################################################################################################

        # Find first guest
        self.moveheadPose(self.HEAD_PITCH_FOR_SPEECH_POSE, self.HEAD_YAW_CENTER, False)  # Reset Head position to talk
        self.send_global_step_start_signal("FindG1")
        global_step_find_g1_start_time = time.time()

        ## Wait
        self.send_action_to_local_manager("findg1_wait")

        ## Ask referee to open the door
        self.send_action_to_local_manager("findg1_ask-referee-to-open-the-door")

        ## Wait2
        self.send_action_to_local_manager("findg1_wait2")

        ## Detect human
        self.send_action_to_local_manager("findg1_detect-human")
        self.simulate_ros_work(1.0, "SIMULATING HUMAN DETECTION")

        ###################################################################################################

        # Ask infos about first guest
        self.send_global_step_start_signal("AskInfoG1")
        global_step_ask_info_g1_start_time = time.time()

        askinfog1_ask_name_counter = 0
        askinfog1_ask_name_max_counts = 3
        askinfog1_ask_name_return_value = ""
        while askinfog1_ask_name_return_value != "ok":

            if askinfog1_ask_name_counter >= askinfog1_ask_name_max_counts: # Reproduce bug of wrong text for ask drink that displays ask name : change >= to <
                rospy.logwarn("Could not get name with confirmation !")
                ## TODO : Do TTS action where robot says something like: Hmmm, I really can't understand what you say, I guess I will just call you {Last_Understood Name}
                break

            ## Ask name
            self.send_action_to_local_manager("askinfog1_ask-name")

            ## Confirm name
            askinfog1_ask_name_return_value = self.send_action_to_local_manager("askinfog1_confirm-name")

            askinfog1_ask_name_counter += 1

        askinfog1_ask_drink_counter = 0
        askinfog1_ask_drink_max_counts = 3
        askinfog1_ask_drink_return_value = ""
        while askinfog1_ask_drink_return_value != "ok":

            if askinfog1_ask_drink_counter >= askinfog1_ask_drink_max_counts: # Reproduce bug of wrong text for ask drink that displays ask name : change >= to <
                rospy.logwarn("Could not get drink with confirmation !")
                ## TODO : Do TTS action where robot says something like: Hmmm, I really can't understand what you say, I guess I will just consider you like {Last_Understood Drink}
                break

            ## Ask drink
            self.send_action_to_local_manager("askinfog1_ask-drink")

            ## Confirm drink
            askinfog1_ask_drink_return_value = self.send_action_to_local_manager("askinfog1_confirm-drink")

            askinfog1_ask_drink_counter += 1

        ## Ask age
        self.send_action_to_local_manager("askinfog1_ask-age")

        ###################################################################################################

        # Go to living room
        self.send_global_step_start_signal("GotoLR1")
        global_step_go_to_lr1_start_time = time.time()

        ## Ask to follow
        self.send_action_to_local_manager("gotolr1_ask-to-follow")

        ## Go to living room
        self.send_action_to_local_manager("gotolr1_go-to-living-room")
        self.sendNavOrderAction("NP", "CRRCloseToGoal", self.find_in_steps_by_id(self.steps, "gotolr1_go-to-living-room")["arguments"]["interestPoint"], 50.0)

        ###################################################################################################

        # Introduce first guest to John
        self.moveheadPose(self.HEAD_PITCH_FOR_SPEECH_POSE, self.HEAD_YAW_CENTER, False)  # Reset Head position to talk
        self.send_global_step_start_signal("IntroduceG1ToJohn")
        global_step_introduce_g1_to_john_start_time = time.time()

        ## Point to first guest
        self.send_action_to_local_manager("introduceg1tojohn_point-to-first-guest")
        self.simulate_ros_work(1.0, "SIMULATING POINTING TO FIRST GUEST ")

        ## Say name and drink
        self.send_action_to_local_manager("introduceg1tojohn_say-name-and-drink")

        ###################################################################################################

        # Introduce John to first guest
        self.send_global_step_start_signal("IntroduceJohnToG1")
        global_step_introduce_john_to_g1_start_time = time.time()

        ## Point to John
        self.send_action_to_local_manager("introducejohntog1_point-to-john")
        self.simulate_ros_work(1.0, "SIMULATING POINTING TO JOHN ")

        ## Say name and drink
        self.send_action_to_local_manager("introducejohntog1_say-name-and-drink")

        ###################################################################################################

        # Seat first guest
        self.send_global_step_start_signal("SeatG1")
        global_step_seat_g1_start_time = time.time()

        ## Find empty seat
        self.send_action_to_local_manager("seatg1_find-empty-seat")
        self.simulate_ros_work(1.0, "SIMULATING FINDING AN EMPTY SEAT")

        ## Point to empty seat
        self.send_action_to_local_manager("seatg1_point-to-empty-seat")
        self.simulate_ros_work(1.0, "SIMULATING POINTING TO EMPTY SEAT")

        ## Tell first guest to seat
        self.send_action_to_local_manager("seatg1_tell-first-guest-to-seat")

        ###################################################################################################

        # TODO If time too short, don't try to bring second guest ?

        # Go to door
        self.send_global_step_start_signal("GotoDoor1")
        global_step_find_go_to_door1_start_time = time.time()

        ## Go to door
        self.send_action_to_local_manager("gotodoor1_go-to-door")
        self.sendNavOrderAction("NP", "CRRCloseToGoal", self.find_in_steps_by_id(self.steps, "gotodoor1_go-to-door")["arguments"]["interestPoint"], 50.0)

        ###################################################################################################

        # Find second guest
        self.moveheadPose(self.HEAD_PITCH_FOR_SPEECH_POSE, self.HEAD_YAW_CENTER, False)  # Reset Head position to talk
        self.send_global_step_start_signal("FindG2")
        global_step_find_g2_start_time = time.time()

        ## Wait
        self.send_action_to_local_manager("findg2_wait")

        ## Ask referee to open the door
        self.send_action_to_local_manager("findg2_ask-referee-to-open-the-door")

        ## Wait2
        self.send_action_to_local_manager("findg2_wait2")

        ## Detect human
        self.send_action_to_local_manager("findg2_detect-human")
        self.simulate_ros_work(1.0, "SIMULATING HUMAN DETECTION")

        ###################################################################################################

        # Ask infos about second guest
        self.send_global_step_start_signal("AskInfoG2")
        global_step_ask_info_g2_start_time = time.time()

        askinfog2_ask_name_counter = 0
        askinfog2_ask_name_max_counts = 3
        askinfog2_ask_name_return_value = ""
        while askinfog2_ask_name_return_value != "ok":

            if askinfog2_ask_name_counter >= askinfog2_ask_name_max_counts:  # Reproduce bug of wrong text for ask drink that displays ask name : change >= to <
                rospy.logwarn("Could not get name with confirmation !")
                ## TODO : Do TTS action where robot says something like: Hmmm, I really can't understand what you say, I guess I will just call you {Last_Understood Name}
                break

            ## Ask name
            self.send_action_to_local_manager("askinfog2_ask-name")

            ## Confirm name
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

            ## Ask drink
            self.send_action_to_local_manager("askinfog2_ask-drink")

            ## Confirm drink
            askinfog2_ask_drink_return_value = self.send_action_to_local_manager("askinfog2_confirm-drink")

            askinfog2_ask_drink_counter += 1

        ## Ask age
        self.send_action_to_local_manager("askinfog2_ask-age")

        ###################################################################################################

        # Go to living room
        self.send_global_step_start_signal("GotoLR2")
        global_step_go_to_lr2_start_time = time.time()

        ## Ask to follow
        self.send_action_to_local_manager("gotolr2_ask-to-follow")

        ## Go to living room
        self.send_action_to_local_manager("gotolr2_go-to-living-room")
        self.sendNavOrderAction("NP", "CRRCloseToGoal", self.find_in_steps_by_id(self.steps, "gotolr2_go-to-living-room")["arguments"]["interestPoint"], 50.0)

        ###################################################################################################

        # Introduce second guest to others
        self.moveheadPose(self.HEAD_PITCH_FOR_SPEECH_POSE, self.HEAD_YAW_CENTER, False)  # Reset Head position to talk
        self.send_global_step_start_signal("IntroduceG2ToOthers")
        global_step_introduce_g2_to_others_start_time = time.time()

        ## Point to second guest
        self.send_action_to_local_manager("introduceg2toothers_point-to-second-guest")
        self.simulate_ros_work(1.0, "SIMULATING POINTING TO SECOND GUEST ")

        ## Say name and drink
        self.send_action_to_local_manager("introduceg2toothers_say-name-and-drink")

        ###################################################################################################

        # Introduce John to second guest
        self.send_global_step_start_signal("IntroduceJohnToG2")
        global_step_introduce_john_to_g2_start_time = time.time()

        ## Point to John
        self.send_action_to_local_manager("introducejohntog2_point-to-john")
        self.simulate_ros_work(1.0, "SIMULATING POINTING TO JOHN ")

        ## Say name and drink
        self.send_action_to_local_manager("introducejohntog2_say-name-and-drink")

        ###################################################################################################

        # Introduce first guest to second guest
        self.send_global_step_start_signal("IntroduceG1ToG2")
        global_step_introduce_g1_to_g2_start_time = time.time()

        ## Point to first guest
        self.send_action_to_local_manager("introduceg1tog2_point-to-first-guest")
        self.simulate_ros_work(1.0, "SIMULATING POINTING TO FIRST GUEST ")

        ## Say name and drink
        self.send_action_to_local_manager("introduceg1tog2_say-name-and-drink")

        ###################################################################################################

        # Seat second guest
        self.send_global_step_start_signal("SeatG2")
        global_step_seat_g2_start_time = time.time()

        ## Find empty seat
        self.send_action_to_local_manager("seatg2_find-empty-seat")
        self.simulate_ros_work(1.0, "SIMULATING FINDING AN EMPTY SEAT")

        ## Point to empty seat
        self.send_action_to_local_manager("seatg2_point-to-empty-seat")
        self.simulate_ros_work(1.0, "SIMULATING POINTING TO EMPTY SEAT")

        ## Tell first guest to seat
        self.send_action_to_local_manager("seatg2_tell-first-guest-to-seat")

        ###################################################################################################

        # Finish Scenario
        self.send_global_step_start_signal("FinishScenario")

        ## Finish Scenario
        self.send_action_to_local_manager("finishscenario_finish-scenario")

        rospy.loginfo("""
                ######################################
                Finished executing the {scenario_name} Scenario...
                ######################################
                """.format(scenario_name=self.scenario["name"]))

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

        self._enableMoveHeadPoseService = True

        AbstractScenarioAction.configure_intern(self)
        AbstractScenarioService.configure_intern(self)

    def gm_heartbeat(self):
        while True:
            self.memory.raiseEvent(self.apis["common"]["generalManagerHeartbeat"]["ALMemory"],
                                   json.dumps({'time': time.time()}))
            time.sleep(1)

    def simulate_ros_work(self, time_for_work, log_string):
        rospy.logwarn(log_string)
        rospy.logwarn("Waiting for {duration} seconds...".format(duration=time_for_work))
        time.sleep(time_for_work)

    def attach_event(self, event_name, callback):
        try:
            self.callback_ids[event_name] = self.memory.subscriber(event_name)
            self.callback_ids[event_name].signal.connect(callback)
            rospy.loginfo("Attached event: {event_name} to: {callback_name}".format(
                event_name=event_name, callback_name=callback.__name__))
        except Exception as ex:
            rospy.logerr("Something went wrong while attaching event {event_name}: {exception}.".format(
                event_name=event_name, exception=str(ex)))

    def send_action_to_local_manager(self, action_id):
        action_data = self.find_in_steps_by_id(self.steps, action_id)
        args = action_data["arguments"]
        args["actionId"] = action_id

        rospy.loginfo("Send action to local manager with id: {action_id}. Arguments are: {args}".format(
            action_id=action_data["id"], args=str(args)))

        self.memory.raiseEvent(self.apis["gmToHRI"]["currentAction"]["ALMemory"], json.dumps(args))

        lock = threading.Lock()
        lock.acquire()
        is_work_done = bool(self.hri_work_return_string)
        hri_return_dict = {} if not is_work_done else json.loads(self.hri_work_return_string)
        lock.release()
        is_done_for_action_id = "ok" in hri_return_dict and hri_return_dict["ok"] == action_id
        is_error = "error" in hri_return_dict
        while not (is_work_done and (is_done_for_action_id or is_error)):
            time.sleep(0.1)

            lock = threading.Lock()
            lock.acquire()
            is_work_done = bool(self.hri_work_return_string)
            hri_return_dict = {} if not is_work_done else json.loads(self.hri_work_return_string)
            lock.release()
            is_done_for_action_id = "ok" in hri_return_dict and hri_return_dict["ok"] == action_id
            is_error = "error" in hri_return_dict

            if "ok" in hri_return_dict and hri_return_dict["ok"] != action_id:
                rospy.logerr("Received action complete signal for {received_id} but was expecting {expected_id}".format(
                    received_id=hri_return_dict["ok"], expected_id=action_id
                ))

        lock = threading.Lock()
        lock.acquire()
        self.hri_work_return_string = "" # Reset value
        lock.release()

        if "ok" in hri_return_dict:
            return "ok"
        elif "error" in hri_return_dict and hri_return_dict["error"] == "confirm":
            return "confirm"
        else:
            rospy.logerr("Local Manager returned unexpected error: {hri_work_return_string}".format(
                hri_work_return_string=str(hri_return_dict)))
            return "error"

    def send_global_step_start_signal(self, action_id):
        self.memory.raiseEvent(self.apis["gmToHRI"]["stepCompleted"]["ALMemory"], json.dumps({}))
        self.memory.raiseEvent(self.apis["gmToHRI"]["currentStep"]["ALMemory"], json.dumps({"actionId": action_id}))

    def handle_hri_job_complete(self, value):
        lock = threading.Lock()
        lock.acquire()
        self.hri_work_return_string = value
        lock.release()
        rospy.loginfo("HRI Callback completed with return value: {return_value}".format(return_value=value))

    def find_in_steps_by_id(self, steps_array, step_id):
        steps = [step for step in steps_array if step["id"] == step_id]
        return steps[0] if len(steps) == 1 else None
