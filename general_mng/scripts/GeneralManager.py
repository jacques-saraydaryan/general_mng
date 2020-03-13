#!/usr/bin/env python

import rospy
import yaml
import threading
from std_msgs.msg import String
import importlib
from os import walk, path


class GeneralManager:
    """
    Command Samples:
        rostopic pub /gm_start std_msgs/String "data: '<YOUR_SCENARIO_NAME (NOT FULL PATH, NO EXTENSION !)>'"
    """

    STARTED_STATUS = "START"
    PENDING_STATUS = "PENDING"

    _current_scenario = None
    # FIXME ADD ROS PARAM
    _current_status = PENDING_STATUS

    def __init__(self, config):
        rospy.init_node('general_manager')

        # load face files form data directory
        self.scenarios_data_folder = config["scenarios_data_folder"]
        self.default_scenario_name = '' if "default_scenario_name" not in config else config["default_scenario_name"]

        self._start_sub = rospy.Subscriber("/gm_start", String, self.gm_start_callback)

        if self.default_scenario_name != '':
            self._current_scenario = self.load_scenario(self.default_scenario_name)

        # self._current_scenario.start_scenario()  # UNCOMMENT FOR DEBUG PURPOSES

    def load_scenario(self, scenario_name):
        scenario_path = self.search_scenario_path_by_filename(scenario_name)

        if scenario_path is None:
            scenario_path = self.search_scenario_path_by_content(scenario_name)

        if scenario_path is None:
            rospy.logerr("Could not find a scenario data file in folder {0} matching name {1}.".format(
                self.scenarios_data_folder, scenario_name
            ))
            return None

        try:
            scenario_data = yaml.safe_load(scenario_path)

            if "imports" in scenario_data:
                for key, rel_path in scenario_data["imports"].items():
                    with open(path.join(scenario_path, rel_path)) as imported_json_file:
                        scenario_data["imports"][key] = yaml.safe_load(imported_json_file)

            try:
                scenario_module = importlib.import_module("scenario." + scenario_name)
                try:
                    scenario_class = getattr(scenario_module, scenario_name)
                    try:
                        return scenario_class(scenario_data, scenario_path)
                    except Exception as e:
                        rospy.logerr("Scenario data and class were loaded properly"
                                     "but scenario object could not be created.")
                        rospy.logerr(e.__class__.__name__ + ": " + e.message)
                except AttributeError as e:
                    rospy.logerr("Could not find a class in scenario python module with name " + scenario_name)
                    rospy.logerr(e.__class__.__name__ + ": " + e.message)
            except ImportError as e:
                rospy.logerr("Could not find a scenario python module with name " + scenario_name)
                rospy.logerr(e.__class__.__name__ + ": " + e.message)
        except yaml.YAMLError as e:
            rospy.logerr("File {0} is not properly formatted JSON or YAML. Please fix it or remove it from the folder.")
            rospy.logerr(e.__class__.__name__ + ": " + e.message)

    def execute_current_scenario(self):
        if self._current_scenario is None:
            rospy.logwarn("No scenario is currently loaded."
                          "Maybe wait a bit more to send a START command or load a new one.")
            return
        else:
            if self._current_scenario.configuration_ready:
                self._current_scenario.start_scenario()
            else:
                rospy.logwarn("Currently loaded scenario is not ready. Maybe wait a bit more to send a START command.")

    def gm_start_callback(self, msg):
        lock = threading.Lock()
        lock.acquire()
        if self._current_status == self.STARTED_STATUS:
            rospy.loginfo('SCENARIO IS ALREADY STARTED...')
            return
        self._current_status = self.STARTED_STATUS
        lock.release()

        rospy.loginfo('START CMD: ' + msg.data)
        if msg.data == 'START':
            self.execute_current_scenario()
        else:
            scenario_name = msg.data
            self._current_scenario = self.load_scenario(scenario_name)
            self.execute_current_scenario()
        self._current_status = self.PENDING_STATUS
        self._current_scenario = None
        rospy.loginfo('-----------------------------------END CURRENT SCENARIO-----------------------')

    def search_scenario_path_by_filename(self, scenario_name):
        for (dirpath, dirnames, filenames) in walk(self.scenarios_data_folder):
            for filename in filenames:
                if scenario_name == path.splitext(filename)[0]:
                    return path.join(dirpath, filename)
        return None

    def search_scenario_path_by_content(self, scenario_name):
        for (dirpath, dirnames, filenames) in walk(self.scenarios_data_folder):
            for filename in filenames:
                file_path = path.join(dirpath, filename)
                with open(file_path) as cur_file:
                    try:
                        file_data = yaml.safe_load(cur_file)
                        if 'name' in file_data and file_data['name'] == scenario_name:
                            return file_path
                    except yaml.YAMLError as e:
                        rospy.logerr("File {0} is not properly formatted JSON or YAML. Please fix it or remove it from the folder.".format(file_path))
                        rospy.logerr(e.__class__.__name__ + ": " + e.message)
        return None


if __name__ == '__main__':
    import os

    dirname = os.path.dirname(__file__)
    filepath = os.path.join(dirname, '../config/common_gm.yaml')

    with open(filepath) as data:
        gm_config = yaml.safe_load(data)

    gm = GeneralManager(gm_config)

    rospy.spin()
