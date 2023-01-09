# To Begin, run Anaconda prompt, navigate with change directory ($:cd ...) and type in:
# $: conda env list # to check if the environment is available
# $: activate OT2DropHandling  # to activate the environment
# $: jupyter-lab  # this runs a notebook on YOUR computer, not on OT-2
# Note,this assumes that the environment was created using:
# $: conda create --name OT2DropHandling --file OT2DropHandlingEnv.txt
# AND that the following packages added via pip
# $ pip install -U pyvisa
# $ pip install opentrons
# $ pip install ffmpy # to interact with camera on OT2
# you can check if the above three packages are installed with...
# $ pip list
# for detailed documentation, visit:
# https://docs.opentrons.com/v2/new_protocol_api.html
# current robot IP address: http://169.254.59.185:48888

# To Start in OT's jupyter notebook, open Opentrons app, then select
# Devices > Robot Settings >  Advanced > Jupyter Notebook > Launch
# Then, run the following code 4 lines of code
# import os  # to control the robot server
# import opentrons.execute  # to control robot execution
# os.system("systemctl stop opentrons-robot-server")  # stop connection to app
# protocol.set_rail_lights(True)  # check! only works if a single connection to robot
# Remember to run the following line at the end of experiment:
# os.system("systemctl start opentrons-robot-server")  # restart connection to app
# To close experiment, save notebook, then Shutdown kernel and Quit



# import json
import time
import math
from copy import deepcopy
from typing import List, Tuple
import datetime  # using time module
# from threading import Thread
# from typing import Tuple
from opentrons import protocol_api, types
from opentrons.protocol_api_experimental import Labware
from opentrons.protocol_api_experimental import pipette_context

metadata = {
    'apiLevel': '2.13',
    'protocolName': 'testing Time management during sample incubation',
    'description': '''Run by USC on Jan 09, 2023. ''',
    'author': 'Ulyana S. Cubeta',
    'rundate': '2023/01/09'
}

# GLOBAL variables, used throughout!
# if changing tips during load/mix/rinse, please ADJUST these

load_time_s = 30  # estimated time for 'load' action, in seconds
mix_time_s = 20  # estimated time for 'mix' action, in seconds
rinse_time_s = 60  # estimated time for 'rinse' or 'unload' action
start_time = 30  # seconds, when first action will run
load_mixes = 3  # number of times to mix when loading
# if incubation time exceeds this, will reload with fresh solution
max_time_before_evap_m = 45  # minutes


# MODIFY: move these class objects and functions to a different file and import
# define class objects
class ExperimentData:
    # need a way to save this metadata (MODIFY!!)
    def __init__(self):
        # Note: tuples() must be replaced, not edited in place
        self.exp_rate_fraction = 0.25  # % of default speeds (to reduce shaking of gantry)
        self.exp_name = "newExperiment"
        self.exp_date = 20221207  # yyyymmdd
        self.zero_timestmp = 1208125
        self.now_timestmp = 1208125

        # keeping track of numbers in experiment,
        # the following values are calculated in config_samples
        self.num_lg_tipracks = 0  # number of tiprack plates for large pipette
        self.num_sm_tipracks = 0  # number of tiprack plates for small pipette
        self.num_res_plates = 0  # number of plates holding reservoirs
        self.num_sam_plates = 0  # number of plates holding samples
        self.num_res_tot = 0  # total number of reservoir wells
        self.num_samples = 0  # total number of sample wells
        self.num_wells_sam_plates = (0,)  # list of sample wells on each plate
        self.num_wells_res_plates = (0,)  # list of reservoir wells on each plate
        self.res_plate_max_vol = []  # uL, int list determined by reservoir plate name

        # slots on the OT-2 (e.g. int 1 through 11)
        self.slots_sam_plates = ()  # well-plate slots, eg: (2, 3)
        self.slots_reservoirs = ()  # reservoir plate slots, eg: (1, 4)
        self.slots_tiprack_lg = ()  # large tip rack slots, eg: (10, 11)
        self.slots_tiprack_sm = ()  # small tip rack slots, eg: (10, 11)
        # labels for labware
        self.labels_res_plates = ('label',)
        self.labels_sam_plates = ('label',)
        # offset for each piece of labware
        self.slot_offsets_sam = ((0.0, 0.0, 0.0),)  # calibration offset (x,y,z)
        self.slot_offsets_res = ((0.0, 0.0, 0.0),)  # calibration offset (x,y,z)
        self.slot_offsets_lg_tips = ((0.0, 0.0, 0.0),)  # calibration offset (x,y,z)
        self.slot_offsets_sm_tips = ((0.0, 0.0, 0.0),)  # calibration offset (x,y,z)
        # index numbers, for plate indices, eg: (0,...10), and well indices, eg: (0,...,0,...)
        self.res_plate_indx_nums = (0,)  # zero-index of each res plate, start at 0
        self.res_well_indx_nums = (0,)  # zero-index of each well on res plate, start at 0
        self.sam_plate_indx_nums = (0,)  # zero-index of each sam plate, start at 0
        self.sam_well_indx_nums = (0,)  # zero-index of each well on sam plate, start at 0
        # locations, consisting of (slot_indx, well_indx)
        self.waste_res_loc = ((0, 0),)  # list of locations (slot_indx, well_indx)
        self.rinse_res_loc = ((0, 0),)  # list of locations (slot_indx, well_indx)
        self.sol_res_loc = ((0, 0),)  # list of locations (slot_indx, well_indx)
        # number of each set
        self.max_num_waste = 0
        self.max_num_rinse = 0
        self.max_num_solut = 0
        # currently, using these wells as:
        self.this_indx_waste = 0
        self.this_indx_rinse = 0
        self.this_indx_solut = 0

        # reservoir info
        self.res_plate_names = ('labware_name',)  # list of plate names
        # self.res_plate_labels = ('labware_label',)  # replicate of above variable
        self.res_starting_vols = [10000, 10000, 0]  # current volumes for reservoirs
        self.res_contents = ('sol1', 'sol2', 'waste')  # contents of the reservoirs
        self.res_start_concent = (0.0, 0.0, 0.0)  # uM (micromol/L)
        self.res_goal_concent = (0.0, 0.0, 0.0)  # uM (micromol/L)

        # sample info
        self.sam_plate_names = ('labware_name',)  # list of plate names
        # self.sam_plate_labels = ('labware_label',)  # replicate of above variable
        self.sam_names = ("sample1",)
        self.sam_well_names = ('A1',)
        self.sam_targ_incub_times_min = (10,)
        self.sam_inoculation_locations = ((0, 0),)  # list of locations (slot_indx, well_indx)
        # self.sample_targ_incub_gap_times_min = (5, )
        self.sam_targ_num_mixes = (1,)
        self.sam_targ_num_rinses = (1,)
        self.sam_indx_in_order = (0,)  # matches to sample_names
        self.sam_timing = (0,)  # points to each sample in order of events

        # custom 3-well 60mL reservoir plate with 'A1', 'A2', 'A3' wells,
        self.res3_60mL_name = 'usctrayvials_3_reservoir_60000ul'
        self.res3_60mL_max_vol = 60000  # uL (max volume of above reservoir plate)
        self.res3_60mL_label = 'res3_60mL_'
        # custom 4-well 32mL reservoir plate plate with 'A1', 'A2', 'A3', 'A4' wells
        self.res4_32mL_name = 'usctrayvials_4_reservoir_32000ul'
        self.res4_32mL_max_vol = 32000  # uL (max volume of above reservoir plate)
        self.res4_32mL_label = 'res4_32mL_'
        # custom 6-well 20mL reservoir plate with (A1-3, B1-3) wells
        self.res6_20mL_name = 'usctrayvials_6_reservoir_20000ul'
        self.res6_20mL_max_vol = 20000  # uL (max volume of above reservoir plate)
        self.res6_20mL_label = 'res6_20mL_'
        # custom 6-well 40mL reservoir plate with (A1-3, B1-3) wells
        self.res6_40mL_name = 'usctrayvials_6_reservoir_40000ul'
        self.res6_40mL_max_vol = 40000  # uL (max volume of above reservoir plate)
        self.res6_40mL_label = 'res6_40mL_'
        # custom 4-well sample wellplate with 'A1', 'A2', 'A3', 'A4' wells
        self.sam4p_400uL_name = 'usctrayfoam_4_wellplate_400ul'
        self.sam4p_400uL_max_vol = 400  # uL (max volume of sample well-plate)
        self.sam4p_400uL_label = 'Sam4Plate_'

        self.pipettes_in_use = 'large'  # 'large', 'small' or 'both'
        self.tip_rack_lg_name = 'opentrons_96_tiprack_1000ul'
        self.tip_rack_sm_name = 'opentrons_96_tiprack_20ul'
        self.pipette_lg_name = 'p1000_single_gen2'
        self.pipette_sm_name = 'p20_single_gen2'
        self.pipette_lg_loc = 'left'
        self.pipette_sm_loc = 'right'
        self.planned_sequence: list[ActClass] = []  # in seconds
        self.pln_seq_stamps: list[ActClass] = []  # in seconds
        # where each tuple is (sample_index, action, start_time_s, end_time_s)
        # eg: [(0, 'load', 10, 20), (0, 'mix', 110, 120), (0, 'mix', 210, 220), (0, 'rinse', 310, 320)]
        self.all_samples: list[list[SampleWellData]] = []
        self.waste_data: list[ResWell] = []
        self.rinse_data: list[ResWell] = []
        self.res_data: list[ResWell] = []
        self.res_data_locs: ((0, 0),)  # tuple of locations, same order as res_data

    def __repr__(self):
        this_string = "ExperimentData(" + str(self.exp_name) + ")"
        return this_string

    def __str__(self):
        this_string = "ExperimentData(" + str(self.exp_name) + ")"
        return this_string

    def find_max_res_vol(self, this_name: str):
        volume = 0
        if this_name == self.res3_60mL_name:
            volume = self.res3_60mL_max_vol
        elif this_name == self.res4_32mL_name:
            volume = self.res4_32mL_max_vol
        elif this_name == self.res6_20mL_name:
            volume = self.res6_20mL_max_vol
        elif this_name == self.res6_40mL_name:
            volume = self.res6_40mL_max_vol
        elif this_name == self.sam4p_400uL_name:
            volume = self.sam4p_400uL_max_vol
        return volume

    def find_label(self, this_name: str):
        this_label = 'no_label_'
        if this_name == self.res3_60mL_name:
            this_label = self.res3_60mL_label
        elif this_name == self.res4_32mL_name:
            this_label = self.res4_32mL_label
        elif this_name == self.res6_20mL_name:
            this_label = self.res6_20mL_label
        elif this_name == self.res6_40mL_name:
            this_label = self.res6_40mL_label
        elif this_name == self.sam4p_400uL_name:
            this_label = self.sam4p_400uL_label
        return this_label


class SampleWellData:
    # need a way to save this metadata (MODIFY!!)
    # may want to switch to getters/setters/properties
    def __init__(self):
        self.sample_name = "USC22Blnk1118a"  # name on the QR-coded label
        self.sam_indx = 0  # sample index from the original user_config_exp
        self.well_plate_slot = 3  # slot on on OT-2 (eg. num 1 through 11)
        self.plate_indx = 0  # plate index for this sample - for use with nested list of sample_set
        self.well_indx = 0  # well index for this sample on plate_indx plate, 0 to 3 for L to R
        self.loc_indx = (0, 0)  # (plate_indx, well_indx) # tuple has to be replaced, not edited
        self.sam_timing = 0  # where first sample we should load is 0, & last is (sam_num-1)
        # self.well_name = 'W1'  # name of the well # irrelevant?
        self.targ_incub_time_m = 1  # target time for incubation (in minutes)
        # self.targ_incub_gap_time_m = 1  # target time in between mixing
        self.targ_num_reload = 0  # if incubation time is long, will remove and reload incubation liquid
        self.targ_num_mixes = 1  # target num. of mixes during incubation
        self.targ_num_rinses = 3  # target num. of rinses after removing incub. liquid
        self.rinsed_num = 0  # how often has this sample been rinsed?
        self.mixed_num = 0  # how often has this sample been mixed?
        self.reloaded_num = 0  # how often has this sample been reloaded?
        self.solution_location = (0, 0)  # location for inoculation from exp.sol_res_loc (slot_indx, well_indx)
        self.solution_index = 0  # corresponding to index in exp.res_data
        self.incub_solution = "DI water"  # name of incubation solution
        self.incub_concen = 0  # uM (micromol/L) molarity of incubation solution
        self.incub_st_timestmp = 1208125  # start time collected @ time.perf_counter()
        self.incub_end_timestmp = 1208140  # end time collected @ time.perf_counter()
        self.incub_reload_timestmps = []  #  eg:[1208225, 1208325, ]  # reload time collected @ time.perf_counter()
        self.incub_mix_timestmps = []  # eg:[1208225, 1208325, ]  # mix time collected @ time.perf_counter()
        self.rinse_timestmps = []  # eg: [1208225, 1208325, ]  # rinse time collected @ time.perf_counter()
        self.incub_tot_time_s = 15  # integer, in seconds
        self.incub_mix_time_s = []  # eg: [100, 200, ]  # integer, in seconds
        self.targ_act_seq = List[ActClass,]  # in seconds
        self.pln_seq_stamps = List[ActClass,]  # in seconds
        # where each action is of Class ActClass(sample_index, action, timeline_in_seconds)

    # returns this when calling this object
    def __repr__(self):
        this_string = "SampleWell(" + str(self.sample_name) + ")"
        return this_string

    # returns this string when called via print(x)
    def __str__(self):
        this_string = "SampleWell(" + str(self.sample_name) + "," + \
                      str(self.loc_indx) + "," + str(self.loc_indx) + ")"
        return this_string


class ActClass:
    def __init__(self, sam_id: int, action: str, stamp: int):
        # initializing function, _attribute is a hidden attribute !
        self._sam_id = sam_id  # integer of samples indexed 0 to num_sam
        # in the order entered by user_config_exp
        self._action = action  # string ['load', 'unload', 'reload', 'mix', 'rinse' ]
        self._start_stamp = stamp  # integer of start timestamp (seconds) for the desired action
        self._end_stamp = self._calc_end()  # calculated, estimated end timestamp

    # returns this when calling this object
    def __repr__(self):
        this_string = "(" + str(self.sam_id) + ", '" + \
                      str(self.action) + "', " + str(self.start) + \
                      ", " + str(self.end) + ")"
        return this_string

    # returns this string when called via print(x)
    def __str__(self):
        this_string = "(" + str(self.sam_id) + ", '" + \
                      str(self.action) + "', " + str(self.start) + \
                      ", " + str(self.end) + ")"
        return this_string

    # setter method
    def change_start(self, new_start: int):
        self._start_stamp = new_start
        self._end_stamp = self._calc_end()

    # property/ getter methods
    @property
    def sam_id(self):
        return self._sam_id

    @property
    def action(self):
        return self._action

    @property
    def start(self):
        return self._start_stamp

    @property
    def end(self):
        return self._end_stamp

    @property
    def length(self):
        time2complete = self._end_stamp - self._start_stamp
        return time2complete

    # other functions
    def _calc_end(self):
        # MODIFY: check that global variables are defined.
        if self.action == 'load':
            prev_end_stamp = self._start_stamp + load_time_s
        elif self.action == 'mix':
            prev_end_stamp = self._start_stamp + mix_time_s
        elif self.action == 'rinse' or self.action == 'unload' or self.action == 'reload':
            prev_end_stamp = self._start_stamp + rinse_time_s
        else:
            prev_end_stamp = self._start_stamp
        return prev_end_stamp


class ResWell:
    def __init__(self):
        self.contents = ''  # solution name/chemical name
        self.concentration = 0.0  # uM (micromol/L) # used for dilutions
        self.curr_vol = 0  # uL
        self.max_vol = 10000  # uL
        self.plate_indx_num = 0
        self.well_indx_num = 0
        self.loc = (0, 0)

    # returns this when calling this object
    def __repr__(self):
        this_string = "( ResWell in loc: " + str(self.loc) + " w/ vol " + \
                      str(self.curr_vol) + " out of " + str(self.max_vol) + ")"
        return this_string

    # returns this string when called via print(x)
    def __str__(self):
        this_string = "( ResWell in loc: " + str(self.loc) + " w/ vol " + \
                      str(self.curr_vol) + " out of " + str(self.max_vol) + ")"
        return this_string


class StopExecution(Exception):
    def _render_traceback_(self):
        pass


# define functions on these class objects
def swap_actions(exp_sequence: List[ActClass], act_pos_this: int, act_pos_that: int):
    # swap the two actions in the list  (^ this_action, old_action)
    # print("Swapping these actions: ", exp_sequence[pos2], exp_sequence[pos1])
    num_actions = len(exp_sequence)
    if act_pos_this >= num_actions or act_pos_that >= num_actions:
        print("WARNING: Cannot swap actions, positions are out of index range.")
        return exp_sequence
    exp_sequence[act_pos_this], exp_sequence[act_pos_that] = exp_sequence[act_pos_that], exp_sequence[act_pos_this]
    # print("Swapping actions for the positions: ", act_pos_this, " and ", act_pos_that)  # debug
    # print("Now they're in order: ", exp_sequence[act_pos_this], exp_sequence[act_pos_that])  # debug
    return exp_sequence


def check_order_swap(exp_sequence: List[ActClass], sam_indx: Tuple[int], this_pos: int, old_pos: int):
    # if two actions are of the same type, check the order in the sam_indx
    old_action = exp_sequence[old_pos]
    this_action = exp_sequence[this_pos]
    # print("Checking if ", old_pos, ":", old_action, "should precede", this_pos, ":", this_action)  # debug
    old_action_sam_indx = sam_indx.index(old_action.sam_id)
    this_action_sam_indx = sam_indx.index(this_action.sam_id)
    # this means if two actions have the same index, they are probably in the correct order!
    if this_action_sam_indx < old_action_sam_indx:
        print(this_pos, ":", this_action, " should come before ", old_pos, ":", old_action, " Swapping them.")  # debug
        swap_actions(exp_sequence, this_pos, old_pos)
    return exp_sequence


def swap_time_w_gap(exp_sequence: List[ActClass], be_first_pos: int, be_second_pos: int, gap_time: int):
    # maybe should be an inner function for shift_timestamp
    # print("Swapping timestamps for the actions in positions: ", be_first_pos, " and ", be_second_pos)  # debug
    # print("Before swap:", exp_sequence[be_first_pos], " and ", exp_sequence[be_second_pos])
    be_first_time = exp_sequence[be_second_pos].start  # new time to start first action
    exp_sequence[be_second_pos].change_start(be_first_time + gap_time)  # new time for second action
    exp_sequence[be_first_pos].change_start(be_first_time)  # change timestamp for first action
    # print("After swap:", exp_sequence[be_first_pos], " and ", exp_sequence[be_second_pos])
    return exp_sequence


def shift_all_for_load(exp_sequence: List[ActClass], sam_index: int, shift_time: int):
    # make a list of all load actions that come after sam_index 'load'
    # shift all actions for samples in that list by a time shift_time
    # then sorts and prioritizes the action list
    # print("Shifting all for load of action:", sam_index)  # debug
    shift_for_load = False
    sams_2_shift = []
    for ix in range(len(exp_sequence)):
        # find all load actions that come after sam_index load
        this_action = exp_sequence[ix]
        if this_action.sam_id == sam_index:
            if this_action.action == 'load':
                shift_for_load = True
        if shift_for_load and this_action.action == 'load':
            sams_2_shift.append(this_action.sam_id)

    print("Shifting all actions for samples: ", sams_2_shift)  # debug
    for ix in range(len(exp_sequence)):
        # shift timestamp for all actions for samples in the list
        this_action = exp_sequence[ix]
        if this_action.sam_id in sams_2_shift:
            this_action.change_start(this_action.start + shift_time)

    # sort and prioritize the list again
    # exp_sequence = prioritize_sequence(exp_sequence)
    # print(exp_sequence)  # debug
    return exp_sequence


def shift_all_for_rinse(exp_sequence: List[ActClass], act_pos, shift_time):
    print("Shifting all rinse after step")
    this_indx = exp_sequence[act_pos].sam_id
    print("Shifting all rinse for samples ", this_indx)
    for ix in range(act_pos, len(exp_sequence)):
        this_action = exp_sequence[ix]
        if this_action.sam_id == this_indx and this_action.action == 'rinse':
            this_action.change_start(this_action.start + shift_time)
    return exp_sequence


def prioritize_sequence(in_seq: List[ActClass], sam_indx: Tuple[int]):
    # sort the list by timestamp, then iterate over the sorted list
    # if two timestamps overlap, rearrange the order in the following way:
    # prioritize in order (1) unload (2) mix (3) load (4) rinse
    # if two actions are the same (eg. both 'mix'),
    # use the sample load order to choose which goes first
    # The second loop modifies the timestamp so that exp_sequence.sort doesn't undo the work

    print("Running: prioritize_sequence(). Prioritizing and sorting list.")  # debug
    # print("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")  # debug
    exp_sequence = deepcopy(in_seq)
    # MODIFY: sometimes this sort swaps previously swapped actions, undoing work...
    exp_sequence.sort(key=lambda sort_action: sort_action.start)
    # print("Concatenated, sorted sequence is: ")  # debug
    # print(exp_sequence)  # debug
    # old_action = ActClass(0, 'none', 0)
    iterations = 0
    hall_pass = len(exp_sequence) * 3
    ix = 1
    while ix < len(exp_sequence):
        iterations += 1
        if iterations > hall_pass:
            print("WARNING: something is wrong, bailing out of prioritize_sequence while loop.")
            break  # emergency break out of while loop
        # iterate over list of actions, starting with the second action
        if ix == 0:
            ix = 1  # if accidentally went back to the first
        # print("p-loop, ix is now:", ix)  # debug
        this_action = deepcopy(exp_sequence[ix])
        old_action = deepcopy(exp_sequence[(ix - 1)])
        # if two timestamps overlap
        if this_action.start < old_action.end:
            # print("______________________________________________________________________________")  # debug
            # print(ix, ":", this_action, " starts at ", this_action.start,
            #      ", before ", (ix-1), ":", old_action, " ends at ", old_action.end)  # debug
            # Case: both are identical (0)
            if this_action.action == old_action.action:
                # print("Case: both are identical (0)")
                exp_sequence = check_order_swap(exp_sequence, sam_indx, ix, (ix - 1))
                if this_action.sam_id == exp_sequence[ix].sam_id and \
                        this_action.start == exp_sequence[ix].start:
                    # print("no change in exp_sequence")  # debug
                    # print("Actions are in the correct order.")  # debug
                    ix = ix + 1  # going forward one, so next_action can be checked
                else:
                    # print("changed exp_sequence")
                    ix = ix - 1  # going back one , to check that old_action moved correctly
            # Case: unload  (1)
            elif this_action.action == 'unload':
                # print("Case: unload  (1)")  # debug
                print(ix, ": ", this_action, " should come before ", (ix - 1), ": ", old_action,
                      " Swapping them.")  # debug
                exp_sequence = swap_actions(exp_sequence, ix, (ix - 1))
                ix = ix - 1  # going back one , to check that old_action moved correctly
            # Case: reload  (2)
            elif this_action.action == 'reload':
                # print("Case: reload  (2)")  # debug
                if old_action.action == 'unload' or old_action.action == 'reload':
                    # print("Actions are in the correct order.")  # debug
                    ix = ix + 1  # going forward one, so next_action can be checked
                    # pass  # do nothing, no swapping!
                else:
                    print(ix, ": ", this_action, " should come before ", (ix - 1), ": ", old_action,
                          " Swapping them.")  # debug
                    exp_sequence = swap_actions(exp_sequence, ix, (ix - 1))
                    ix = ix - 1  # going back one , to check that old_action moved correctly
            # Case: mix  (3)
            elif this_action.action == 'mix':
                # print("Case: mix  (3)")
                if old_action.action == 'load' or old_action.action == 'rinse':
                    print(ix, ": ", this_action, " should come before ", (ix - 1), ": ", old_action,
                          " Swapping them.")  # debug
                    exp_sequence = swap_actions(exp_sequence, ix, (ix - 1))
                    ix = ix - 1  # going back one , to check that old_action moved correctly
                else:
                    # print("Actions are in the correct order.")  # debug
                    ix = ix + 1  # going forward one, so next_action can be checked
                    # pass  # do nothing, no swapping!
            # Case: load  (4)
            elif this_action.action == 'load':
                # print("Case: load  (4)")
                if old_action.action == 'rinse':
                    print(ix, ": ", this_action, " should come before ", (ix - 1), ": ", old_action,
                          " Swapping them.")  # debug
                    exp_sequence = swap_actions(exp_sequence, ix, (ix - 1))
                    ix = ix - 1  # going back one , to check that old_action moved correctly
                else:
                    # print("Actions are in the correct order.")  # debug
                    ix = ix + 1  # going forward one, so next_action can be checked
                    # pass  # do nothing, no swapping!
            # Case: rinse  (5) or any others
            else:
                # print("Case: rinse  (5) or any others")
                # if adding other actions, will need to re-asses
                # print("Actions are in the correct order.")  # debug
                ix = ix + 1  # going forward one, so next_action can be checked
                # pass # do nothing, no swapping!
            # print("Changing ix to :", ix)  # debug
        else:
            ix = ix + 1  # going forward one, so next_action can be checked

    # print("Shifting start timestamps in case start times are equal to one another.")  # debug
    # print("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")  # debug
    iterations = 0
    hall_pass = len(exp_sequence) * 3
    ix = 1
    while ix < len(exp_sequence):
        iterations += 1
        if iterations > hall_pass:
            print("WARNING: something is wrong, bailing out of prioritize_sequence while loop.")
            break  # emergency break out of while loop
        # iterate over list of actions, starting with the second action
        if ix == 0:
            ix = 1  # if accidentally went back to the first
        # print("p-loop, ix is now:", ix)  # debug
        this_action = exp_sequence[ix]  # should be an alias, not a copy
        old_action = exp_sequence[(ix - 1)]  # should be an alias, not a copy
        # if two timestamps overlap
        if this_action.start <= old_action.start:
            # shifting all except 'load' or 'unload'
            if this_action.action == 'mix' or \
                    this_action.action == 'reload' or \
                    this_action.action == 'rinse':
                this_action.change_start(old_action.start + 10)
                # so that swap is not done redundantly, the time will be shifted again
                print("Shifted action: ", ix, this_action)
            ix = ix + 1  # going forward one, so next_action can be checked
        else:
            ix = ix + 1  # going forward one, so next_action can be checked
    # print("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")  # debug
    return exp_sequence


def shift_timestamp(in_seq: List[ActClass], sam_indx: Tuple[int]):
    # next, iterate over the sorted list and shift all except unload
    # shift the action if the previous one has a timestamp that
    # overlaps with the time needed to complete the previous action
    # prioritize in order (1) unload (2) mix (3) load (4) rinse
    # old_action = ActClass(0, 'none', 0)
    exp_sequence = deepcopy(in_seq)  # deep copy so that mix-ups can be interrupted
    print("Shifting timestamps for the list of ", len(exp_sequence), " actions.")  # debug
    # print("Sample index in order of inoculation", sam_indx)  # debug
    exp_sequence = prioritize_sequence(exp_sequence, sam_indx)
    # print("Concatenated, sorted, swapped sequence is: ")  # debug
    # print(exp_sequence)  # debug
    # print("=====================================================================================")  # debug
    # print("Starting the while loop in shift_timestamp()")  # debug

    iterations = 0
    hall_pass = len(exp_sequence) * 5
    ix = 1
    while ix < len(exp_sequence):
        iterations += 1
        if iterations > hall_pass:
            print("WARNING: something is wrong, bailing out of infinite while loop.")
            break  # emergency break out of while loop
        # iterate over list of actions, starting with the second action
        if ix == 0:
            ix = 1  # if accidentally went back to the first
        # print("ix is now:", ix)  # debug
        this_action = exp_sequence[ix]  # should be an alias, not a copy
        old_action = exp_sequence[(ix - 1)]  # should be an alias, not a copy
        # because the intent is to modify this_action or old_action
        # if this_action.start < old_action.end:
        #     print("=====================================================================================")  # debug
        #     print(ix, ":", this_action, " starts at ", this_action.start,
        #           "before ", old_action, " ends at ", old_action.end)  # debug
        #     print("OVERLAP! prioritizing sequence before changing timestamp")
        #     exp_sequence = prioritize_sequence(exp_sequence, sam_indx)  # checking the order
        #     # re-assign in case order changed
        #     this_action = exp_sequence[ix]  # should be an alias, not a copy
        #     old_action = exp_sequence[(ix - 1)]  # should be an alias, not a copy
        # Now, if timestamps overlap, modify exp_sequence timestamps!, then REPEAT ix
        if this_action.start < old_action.end:
            # print("=====================================================================================")  # debug
            # print(ix, ":", this_action, " starts at ", this_action.start,
            #      "before ", old_action, " ends at ", old_action.end)  # debug
            print("OVERLAP, changing timestamp. ix is now:", ix)  # debug
            # prioritize in order (1) unload (2) mix (3) load (4) rinse
            # Case: both are identical (0)
            if this_action.action == old_action.action:
                # print("Both actions are the same, checking order")
                exp_sequence = check_order_swap(exp_sequence, sam_indx, ix, (ix - 1))
                this_action = exp_sequence[ix]  # should be an alias, not a copy
                old_action = exp_sequence[(ix - 1)]  # should be an alias, not a copy
            # Case: unload  (1)
            if this_action.action == 'unload':
                print("Unload shouldn't move! Only another unload can overlap! ")  # debug
                # Case: unload (1) + old: unload (1)
                if old_action.action == 'unload':
                    print("Two unloads overlap! Check order:", sam_indx)  # debug
                    sam_id = this_action.sam_id  # moving this_action
                    shift_time = old_action.end - this_action.start
                    # print("Load needs to shift for all loads/steps after sam_id: ",
                    #      sam_id, " by time: ", shift_time)  # debug
                    exp_sequence = shift_all_for_load(exp_sequence, sam_id, shift_time)
                    print("WARNING: Now we have to take ix back to the beginning to resort!")
                    ix = 1
                else:
                    print("WARNING: Why didn't prioritize_sequence work?")
                    ix = 1
            # Case: reload (2)
            elif this_action.action == 'reload':
                # print("Reload can be moved earlier or later.")  # debug
                if old_action.action == 'unload' or old_action.action == 'reload':
                    this_action.change_start(old_action.end)  # new time to start mixing
                    print("Moving 'reload' to later:", this_action)  # debug
                else:
                    print("WARNING: old action should have been swapped with reload!", old_action)
                    ix = 1
            # Case: mix (2)
            elif this_action.action == 'mix':
                # print("Mix can be moved earlier or later.")  # debug
                if old_action.action == 'mix' or old_action.action == 'unload':
                    this_action.change_start(old_action.end)  # new time to start mixing
                    print("Moving 'mix' to later:", this_action)  # debug
                    # MODIFY: check that the mix will not come after unload for the same sample
                else:
                    print("WARNING: old action should have been swapped with mix!", old_action)
                    ix = 1
            # Case: load (3)
            elif this_action.action == 'load':
                # Case: load (3) + rinse (4)
                if old_action.action == 'rinse':
                    print("WARNING: rinse action should have been swapped with load!", old_action)
                    ix = 1
                # Case: load (3) + (unload (1) or mix(2) or load(3))
                else:
                    sam_id = this_action.sam_id
                    shift_time = old_action.end - this_action.start
                    # target load time should start at prev_end_stamp, so shift 'load' by the difference
                    # print("Load needs to shift for all loads/steps after sam_id: ",
                    #      sam_id, " by time: ", shift_time)  # debug
                    exp_sequence = shift_all_for_load(exp_sequence, sam_id, shift_time)
                    # print("Updated load action is, ", this_action)  # debug
            # Case: rinse (4)
            elif this_action.action == 'rinse':
                # MODIFY: shift ALL 'rinse' actions after this one for this sample.
                shift_time = old_action.end - this_action.start
                shift_all_for_rinse(exp_sequence, ix, shift_time)
                this_action.change_start(old_action.end)  # new time to start rinse
            else:
                print("WARNING: what else?")
                # ix = 1
            print("Re-prioritizing, since a change was likely")
            exp_sequence = prioritize_sequence(exp_sequence, sam_indx)
            ix = ix - 1  # going back one , to check that old_action moved correctly
            print("Changing ix to :", ix)  # debug
        else:
            ix = ix + 1  # going forward one, so next_action can be checked
    return exp_sequence


def find_gaps_compress_actions(in_seq: List[ActClass]):
    # Find the gaps in exp_sequence and compress when there are gaps
    # moving the mix/rinse forwards if they fit into gaps (no swapping)
    exp_sequence = deepcopy(in_seq)
    num_actions = len(exp_sequence)
    for ix in range(1, num_actions):
        this_action = exp_sequence[ix]  # should be an alias, not a copy
        if this_action.action == 'mix' or this_action.action == 'rinse':
            # only shifting mix and rinse actions, NOT load/unload/reload
            old_action = exp_sequence[(ix - 1)]  # should be an alias, not a copy
            if this_action.start > old_action.end:
                # print("Changing action: ", this_action)  # debug
                this_action.change_start(old_action.end)
    return exp_sequence


def swap_into_gaps(in_seq: List[ActClass]):
    # Check the gaps between two actions and see if the following
    # action can be swapped into the gap
    exp_sequence = deepcopy(in_seq)

    iterations = 0
    hall_pass = len(exp_sequence) * 5
    iter_ix = 2
    while iter_ix < len(exp_sequence):
        iterations += 1
        if iterations > hall_pass:
            print("WARNING: something is wrong, bailing out of while loop.")
            break  # emergency break out of while loop
        # iterate over list of actions, starting with the second action
        if iter_ix < 2:
            iter_ix = 2  # if accidentally went back to the first

        last_action = exp_sequence[(iter_ix - 1)]  # should be an alias, not a copy
        prev_action = exp_sequence[(iter_ix - 2)]  # should be an alias, not a copy
        this_action = exp_sequence[iter_ix]  # should be an alias, not a copy
        gap_between = last_action.start - prev_action.end

        # print("ix is now:", ix)  # debug
        loaded_samples = []  # need a new list since can go back to prev iteration
        unloaded_samples = []  # need a new list since can go back to prev iteration
        # START HERE: (I think I fixed the bug.)
        for subx in range(iter_ix - 1):
            sub_act = exp_sequence[subx]
            if sub_act.action == 'load':
                if sub_act.sam_id not in loaded_samples:
                    loaded_samples.append(sub_act.sam_id)
            elif sub_act.action == 'unload':
                if sub_act.sam_id not in unloaded_samples:
                    unloaded_samples.append(sub_act.sam_id)

        if this_action.action == 'mix':
            if gap_between >= mix_time_s and this_action.sam_id in loaded_samples:
                print("ix:", iter_ix, "Moving ", this_action, " between ", prev_action, " and ", last_action)  # debug
                print("Loaded samples: ", loaded_samples)  # debug
                this_action.change_start(prev_action.end)
                exp_sequence = swap_actions(exp_sequence, iter_ix, (iter_ix - 1))
                exp_sequence = find_gaps_compress_actions(exp_sequence)
                iter_ix = iter_ix - 2  # going back two, to check that this_action moved correctly
            else:
                iter_ix = iter_ix + 1  # iterating forward
        elif this_action.action == 'rinse':
            if gap_between >= rinse_time_s and this_action.sam_id in unloaded_samples:
                print("ix:", iter_ix, "Moving ", this_action, " between ", prev_action, " and ", last_action)
                print("Unloaded samples: ", unloaded_samples)  # debug
                this_action.change_start(prev_action.end)
                exp_sequence = swap_actions(exp_sequence, iter_ix, (iter_ix - 1))
                exp_sequence = find_gaps_compress_actions(exp_sequence)
                iter_ix = iter_ix - 2  # going back two, to check that this_action moved correctly
            else:
                iter_ix = iter_ix + 1  # iterating forward
        else:
            iter_ix = iter_ix + 1  # iterating forward

    return exp_sequence


def create_exp_sequence(exp: ExperimentData):
    # def create_exp_sequence(sample: SampleWell)
    # pass exp, exp_seq

    # get the time-ordered list of sample loading ( of sam indx)
    sam_indx_in_order = exp.sam_indx_in_order
    num_samples = exp.num_samples
    all_samples = exp.all_samples

    # first, concatenate action steps and add load time
    # noinspection PyTypeChecker
    exp_sequence: List[ActClass] = []
    this_action: ActClass
    time_in_seq = start_time
    for ij in range(num_samples):
        sample_index = sam_indx_in_order[ij]
        this_plate = exp.sam_plate_indx_nums[sample_index]
        this_well = exp.sam_well_indx_nums[sample_index]
        this_sample = all_samples[this_plate][this_well]
        sam_sequence = deepcopy(list(this_sample.targ_act_seq))  # needs to be a deepcopy,
        # not just a copy of pointers to objects that can be modified;
        # to preserve each sample's targ_act_seq tuple
        start_action = len(exp_sequence)
        exp_sequence = exp_sequence + sam_sequence  # concatenate the lists
        end_action = len(exp_sequence)
        # list of target action steps from sample
        for jx in range(start_action, end_action):
            this_action = exp_sequence[jx]  # not sure why this sets off type warnings
            new_start_time = this_action.start + time_in_seq
            # print("jx is: ", jx, "; this action is: ", this_action )  # debug
            this_action.change_start(new_start_time)
            # print("changed to: ", this_action)  # debug
        time_in_seq = time_in_seq + load_time_s  # shift the start time for next load by load_time

    # then, sort by the timestamp and prioritize the list by action type
    print("=====================================================================================")
    exp_sequence = prioritize_sequence(exp_sequence, sam_indx_in_order)
    print("Concatenated, sorted, swapped sequence is: ")  # debug
    print(exp_sequence)  # debug
    print("=====================================================================================")
    exp_sequence = shift_timestamp(exp_sequence, sam_indx_in_order)
    print("Concatenated, sorted, swapped, and shifted sequence is: ")  # debug
    print(exp_sequence)
    print("=====================================================================================")
    print("Double checking sequence:")
    exp_sequence = shift_timestamp(exp_sequence, sam_indx_in_order)
    print(exp_sequence)
    print("=====================================================================================")
    print("Compressing sequence")
    exp_sequence = find_gaps_compress_actions(exp_sequence)
    print(exp_sequence)
    print("=====================================================================================")
    print("Shifting into gaps")
    exp_sequence = swap_into_gaps(exp_sequence)
    print(exp_sequence)

    return exp_sequence


# used in config_samples
def give_list_order(some_list: Tuple[int]):
    # sort a tuple list (indexed 0 to len(some_list))
    # by the VALUES in the list, then return
    # the index values of the sorted list
    # eg: (5, 3, 2, 0, ...)
    temp_list = []
    ij: int
    for ij in range(len(some_list)):
        temp_list.append([some_list[ij], ij])
    temp_list.sort()  # sort by the first in tuple
    sort_index: List[int]
    sort_index = []
    for xit in temp_list:
        sort_index.append(xit[1])  # collect second item in tuple
    sorted_tuple: tuple[int]
    sorted_tuple = tuple(sort_index)
    print("Sorted list order is:", sorted_tuple)  # debug
    return sorted_tuple


# used after user_config_exp
def config_samples(exp: ExperimentData):
    # This function checks the number of plates & samples in configuration
    # both modifies exp variables and replaces all_samples in exp
    # (list of lists of objects of class SampleWell !)
    exp.num_samples = len(exp.sam_plate_indx_nums)
    exp.num_sam_plates = len(exp.slots_sam_plates)
    exp.num_lg_tipracks = len(exp.slots_tiprack_lg)
    exp.num_sm_tipracks = len(exp.slots_tiprack_sm)
    exp.num_res_plates = len(exp.slots_reservoirs)
    exp.num_res_tot = len(exp.res_plate_indx_nums)
    exp.max_num_rinse = len(exp.rinse_res_loc)
    exp.max_num_waste = len(exp.waste_res_loc)
    exp.max_num_solut = len(exp.sol_res_loc)
    max_incub = max(exp.sam_targ_incub_times_min)

    # check that the number of items in user_config_experiment is consistent
    num_sam = exp.num_samples
    if len(exp.sam_names) != num_sam or \
            len(exp.sam_targ_incub_times_min) != num_sam or \
            len(exp.sam_targ_num_rinses) != num_sam:
        f_out_string = "Something is off. Please check user_config_experiment()"
        print(f_out_string)  # debug for notebook
        # protocol.pause(f_out_string)  # debug for OT2 app
        # MODIFY: check that other parameters are numbered correctly!
    else:
        f_out_string = "The user_config_experiment() was written correctly. "
        print(f_out_string)  # debug for notebook
        # protocol.comment(f_out_string)  # debug for OT2 app

    # first, set up data for RESERVOIRS (solutions, waste, rinse)
    res_locations = []
    res_set = []  # list of ResWell objects (not nested)
    waste_set = []  # list of ResWell objects (not nested)
    rinse_set = []  # list of ResWell objects (not nested)
    num_plates = exp.num_res_plates
    plate_labels = ['label'] * num_plates
    num_wells_all_plates = [0] * num_plates  # placeholder for list of num_wells in each plate
    max_volumes = [0.0] * num_plates
    all_on_prev_plates = 0  # for sample_index from 0 to num_res-1
    for plate_index in range(num_plates):
        # for each plate in on the deck, indexed 0 to num_plates-1
        plate_name = exp.res_plate_names[plate_index]
        max_volumes[plate_index] = exp.find_max_res_vol(plate_name)
        plate_labels[plate_index] = exp.find_label(plate_name) + str(plate_index)
        num_wells_on_this_plate = 0
        for check_res in range(exp.num_res_tot):
            # not the fastest way, since we go through all samples num_plates times
            if exp.res_plate_indx_nums[check_res] == plate_index:
                num_wells_on_this_plate = num_wells_on_this_plate + 1
        # record the number of wells in each plate
        num_wells_all_plates[plate_index] = num_wells_on_this_plate
        f_out_string = "Plate " + str(plate_name) + " labeled " + str(plate_labels[plate_index]) \
                       + " indexed " + str(plate_index) + " has " + str(num_wells_on_this_plate) \
                       + " wells with max vol " + str(max_volumes[plate_index])  # debug
        print(f_out_string)  # debug
        for this_well_on_plate in range(num_wells_on_this_plate):
            # for each well on this plate
            this_res_indx = all_on_prev_plates + this_well_on_plate
            new_res = ResWell()
            pt_indx_num = exp.res_plate_indx_nums[this_res_indx]
            wl_indx_num = exp.res_well_indx_nums[this_res_indx]
            this_loc = (pt_indx_num, wl_indx_num)
            new_res.plate_indx_num = pt_indx_num
            new_res.well_indx_num = wl_indx_num
            new_res.loc = this_loc
            new_res.max_vol = max_volumes[plate_index]
            new_res.curr_vol = exp.res_starting_vols[this_res_indx]
            new_res.contents = exp.res_contents[this_res_indx]
            if this_loc in exp.waste_res_loc:
                waste_set.append(new_res)
            elif this_loc in exp.rinse_res_loc:
                rinse_set.append(new_res)
            elif this_loc in exp.sol_res_loc:
                res_set.append(new_res)  # this order may be out of sync with sol_res_loc
                res_locations.append(this_loc)  # list of the same order as res_set data
        all_on_prev_plates = all_on_prev_plates + num_wells_on_this_plate
    exp.num_wells_res_plates = tuple(num_wells_all_plates)  # record as a tuple, no mods
    exp.labels_res_plates = tuple(plate_labels)
    exp.res_data_locs = tuple(res_locations)
    exp.res_data = res_set
    exp.waste_data = waste_set
    exp.rinse_data = rinse_set

    # second, find the order of SAMPLE incubation:
    # sort by the length of incubation time to produce index of wells in time-order
    # then, find the order that the wells should be inoculated and then incubated
    # where each value exp.sam_timing[i] indicates WHEN that sample(exp.sample_names[i])
    # should be loaded, where a value of 0 is FIRST, & (sam_num-1) is LAST
    exp.sam_indx_in_order = give_list_order(exp.sam_targ_incub_times_min)  # inoculation order
    exp.sam_timing = give_list_order(exp.sam_indx_in_order)  # when to inoculate, sam-ordered!

    # lastly, set up data for SAMPLES
    # sort the samples by plate location and by well
    # pass the user input into the record of well info
    # list of plates [0 to num_plates-1] with
    # list of samples [0 to num_sam_in_plate-1] where each
    # object in the NESTED list is of class SampleWell objects
    sample_set = []  # all samples, NESTED list of lists of objects
    num_plates = exp.num_sam_plates
    plate_labels = ['label'] * num_plates
    num_wells_all_plates = [0] * num_plates  # placeholder for list of num_wells in each plate
    all_on_prev_plates = 0  # for sample_index from 0 to num_res-1
    for plate_index in range(num_plates):
        # for each plate in on the deck, indexed 0 to num_plates-1
        plate_name = exp.sam_plate_names[plate_index]
        plate_labels[plate_index] = exp.find_label(plate_name) + str(plate_index)
        num_wells_on_this_plate = 0
        for check_res in range(exp.num_samples):
            # not the fastest way, since we go through all samples num_plates times
            if exp.sam_plate_indx_nums[check_res] == plate_index:
                num_wells_on_this_plate = num_wells_on_this_plate + 1
        # record the number of wells in each plate
        num_wells_all_plates[plate_index] = num_wells_on_this_plate
        print("===========================================================================================")  # debug
        f_out_string = "Sample plate " + str(plate_index) \
                       + " labeled " + plate_labels[plate_index] \
                       + " with " + str(num_wells_on_this_plate) \
                       + " samples."  # debug
        print(f_out_string)  # debug
        # protocol.comment(f_out_string)  # debug
        plate_data_set = []  # each plate's list of SampleWell class
        for this_well_on_plate in range(num_wells_on_this_plate):
            # for each well on this plate
            plate_data_set.append(SampleWellData())  # appends object of SampleWell class to list
            sample = plate_data_set[this_well_on_plate]
            this_sam_indx = all_on_prev_plates + this_well_on_plate
            sample.sam_indx = this_sam_indx
            sample.sample_name = exp.sam_names[this_sam_indx]
            sample.sam_timing = exp.sam_timing[this_sam_indx]
            sample.plate_indx = exp.sam_plate_indx_nums[this_sam_indx]
            sample.well_plate_slot = exp.slots_sam_plates[plate_index]
            sample.well_indx = exp.sam_well_indx_nums[this_sam_indx]
            # sample.well_name = exp.sam_well_names[this_sam_indx]
            sample.targ_num_rinses = exp.sam_targ_num_rinses[this_sam_indx]
            sample.targ_num_mixes = exp.sam_targ_num_mixes[this_sam_indx]
            sample.targ_incub_time_m = exp.sam_targ_incub_times_min[this_sam_indx]
            if sample.targ_incub_time_m > max_time_before_evap_m:
                num_reload = int(sample.targ_incub_time_m/max_time_before_evap_m)
                sample.targ_num_reload = num_reload
                sample.targ_num_mixes = sample.targ_num_mixes - num_reload
                if sample.targ_num_mixes < 0:
                    sample.targ_num_mixes = 0
            sample.solution_location = exp.sam_inoculation_locations[this_sam_indx]
            sample.solution_index = res_locations.index(sample.solution_location)
            # sample.targ_incub_gap_time_m = exp.sample_targ_incub_gap_times_min[start_sam]
            sample.loc_indx = (sample.plate_indx, sample.well_indx)
            targ_inc_time_s = math.ceil(60 * sample.targ_incub_time_m)
            exp_sequence: List[ActClass] = []
            sam_timestamp = 0
            this_action = ActClass(this_sam_indx, 'load', sam_timestamp)
            exp_sequence.append(this_action)
            gap_time = math.ceil(targ_inc_time_s / (sample.targ_num_mixes + 1))
            for i in range(sample.targ_num_mixes):
                sam_timestamp = sam_timestamp + gap_time
                this_action = ActClass(this_sam_indx, 'mix', sam_timestamp)
                exp_sequence.append(this_action)
            # the timestamps for 'reload' will be resorted at the end
            for i in range(sample.targ_num_reload):
                gap_time = (i+1)*max_time_before_evap_m*60
                # START HERE!  check that 'reload' sorts correctly
                this_action = ActClass(this_sam_indx, 'reload', gap_time)
                exp_sequence.append(this_action)
            sam_timestamp = targ_inc_time_s
            this_action = ActClass(this_sam_indx, 'unload', sam_timestamp)
            exp_sequence.append(this_action)
            sam_timestamp = sam_timestamp + 3 * rinse_time_s + 60 * max_incub
            for i in range(sample.targ_num_rinses):
                this_action = ActClass(this_sam_indx, 'rinse', sam_timestamp)
                exp_sequence.append(this_action)
                sam_timestamp = sam_timestamp + 3 * rinse_time_s
            # this_action = ActClass(this_sam_indx, 'done', sam_timestamp)
            # exp_sequence.append(this_action)
            sample.targ_act_seq = exp_sequence
            # debug block...
            print("------------------------------------------------------------------------------------")  # debug
            f_out_string = "Sample name: " + str(sample.sample_name) + " indexed # " \
                           + str(this_sam_indx) + " on plate # " + str(plate_index) + " & well # " \
                           + str(this_well_on_plate) + " with sequence: "  # debug
            print(f_out_string)  # debug
            print(sample.targ_act_seq)  # debug
            # print(vars(sample))  # debug
        all_on_prev_plates = all_on_prev_plates + num_wells_on_this_plate
        sample_set.append(plate_data_set)
        for x in sample_set:
            for y in x:
                # print(y.sample_name)  # debug
                pass
    exp.num_wells_sam_plates = tuple(num_wells_all_plates)  # record as a tuple, no mods
    exp.labels_sam_plates = tuple(plate_labels)
    exp.all_samples = sample_set  # NESTED list of lists of objects of class SampleWell

    return exp


# have user fill out for each experiment
def user_config_exp():
    this_exp = ExperimentData()
    this_exp.exp_name = "TestingTimeManagement"  # update every run
    this_exp.exp_date = 20230103  # yyyymmdd # update every run

    this_exp.pipettes_in_use = 'large'  # 'large', 'small' or 'both'

    # list the reservoir wells in order plates 0 to (num_res_plates - 1)
    this_exp.slots_reservoirs = (1, 4, 5, 8, 7, 9)  # slots 1-11 on OT-2
    # xyz calibration offset for labware (in mm), check on OT-2 app
    this_exp.slot_offsets_res = ((-0.20, 2.10, -1.30),
                                 (-1.00, -1.00, -0.80),
                                 (-1.00, -1.00, -1.60),
                                 (0.00, -2.00, -1.70),
                                 (-1.00, 1.00, -0.70),
                                 (-1.00, 0.00, -1.10),)
    # reservoir labware, eg:  res3_60mL_name, res4_32mL_name, res6_20mL_name, res6_40mL_name
    this_exp.res_plate_names = (this_exp.res3_60mL_name,
                                this_exp.res3_60mL_name,
                                this_exp.res4_32mL_name,
                                this_exp.res4_32mL_name,
                                this_exp.res6_40mL_name,
                                this_exp.res6_40mL_name)  # choose one for each slot
    # List the indices of the reservoir plates (starting at zero for the first)
    # break to a new line to distinguish plates
    this_exp.res_plate_indx_nums = (0, 0, 0,
                                    1, 1, 1,
                                    2, 2, 2, 2,
                                    3, 3, 3, 3,
                                    4, 4, 4, 4, 4, 4,
                                    5, 5, 5, 5, 5, 5)  # zero-index of each plate, start at zero
    this_exp.res_well_indx_nums = (0, 1, 2,
                                   0, 1, 2,
                                   0, 1, 2, 3,
                                   0, 1, 2, 3,
                                   0, 1, 2, 3, 4, 5,
                                   0, 1, 2, 3, 4, 5)  # zero-index of each well on plate (L=0,...,R=3)

    # List the locations of waste reservoir well and rinse/dilution reservoir wells
    this_exp.waste_res_loc = ((0, 2), (1, 2))  # list of locations (slot_indx, well_indx) for waste
    this_exp.rinse_res_loc = ((0, 0), (1, 0))  # list of locations (slot_indx, well_indx) for DI rinse
    this_exp.sol_res_loc = ((0, 1),
                            (1, 1),
                            (2, 0), (2, 1), (2, 2), (2, 3),
                            (3, 0), (3, 1), (3, 2), (3, 3),
                            (4, 0), (4, 1), (4, 2), (4, 3), (4, 4), (4, 5),
                            (5, 0), (5, 1), (5, 2), (5, 3), (5, 4), (5, 5))  # loc list (slot_indx, well_indx)

    # starting reservoir volumes (1 mL = 1000 uL), list all: sol, rinse, waste
    this_exp.res_starting_vols = [50000, 50000, 0,
                                  50000, 50000, 0,
                                  30000, 30000, 30000, 30000,
                                  30000, 30000, 30000, 30000,
                                  40000, 40000, 40000, 40000, 40000, 40000,
                                  40000, 40000, 40000, 40000, 40000, 40000]
    # starting reservoir contents , list all: sol, rinse, waste
    this_exp.res_contents = ('DI_sol', 'Thiol_1_sol', 'Waste',
                             'DI_sol', 'Thiol_2_sol', 'Waste',
                             'Thiol_1_sol', 'Thiol_1_sol', 'Thiol_1_sol', 'Thiol_1_sol',
                             'Thiol_2_sol', 'Thiol_2_sol', 'Thiol_2_sol', 'Thiol_2_sol',
                             'Thiol_1_sol', 'Thiol_1_sol', 'Thiol_1_sol',
                             'Thiol_1_sol', 'Thiol_1_sol', 'Thiol_1_sol',
                             'Thiol_2_sol', 'Thiol_2_sol', 'Thiol_2_sol',
                             'Thiol_2_sol', 'Thiol_2_sol', 'Thiol_2_sol')  # contents of the reservoirs

    this_exp.res_start_concent = (0.0, 2000.0, 0.0,
                                  0.0, 2000.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,)  # uM (micromol/L)
    this_exp.res_goal_concent = (0.0, 2000.0, 0.0,
                                 0.0, 2000.0, 0.0,
                                 1600.0, 1000.0, 800.0, 600.0,
                                 1600.0, 1000.0, 800.0, 600.0,
                                 400.0, 300.0, 200.0, 100.0, 50.0, 10.0,
                                 400.0, 300.0, 200.0, 100.0, 50.0, 10.0)  # uM (micromol/L)



    # list of tipracks for pipettes (MODIFY if re-using/returning tips)
    this_exp.slots_tiprack_lg = (10, 11)  # slots 1-11 on OT-2
    # this_exp.slot_tip_rack_sm = (10, 11)  # slots 1-11 on OT-2
    this_exp.slot_offsets_lg_tips = ((-1.20, 1.50, 0.20),
                                     (-1.10, 1.50, -0.10))  # calibration offset for tips
    # this_exp.slot_offsets_sm_tips = ((-1.10, 1.50, -0.10),)  # calibration offset for tips

    # list of sam_plates corresponds to index of plates 0,1,2,... (num_plates-1)
    this_exp.slots_sam_plates = (2, 3, 6)  # slots 1-11 on OT-2 for the plates (indexed 0,1,2...)
    this_exp.slot_offsets_sam = ((0.00, 2.30, 0.10),
                                 (0.10, 1.40, 0.20),
                                 (-0.10, 0.80, 0.40),)  # calibration offset for labware
    # sample labware, eg:  sam4p_400uL_name
    this_exp.sam_plate_names = (this_exp.sam4p_400uL_name,
                                this_exp.sam4p_400uL_name,
                                this_exp.sam4p_400uL_name)  # choose one for each slot
    # the samples must be listed in order for the time being, modify code if out of order
    # sample index will be 0,1,2,...(num_samples-1), but order of inoculation will be calculated in config_samples
    this_exp.sam_names = ("USC22Au1221a", "USC22Au1221b", "USC22Au1221c",
                          "USC22Au1221d", "USC22Au1221e", "USC22Au1221f",
                          "USC22Au1221g", "USC22Au1221h")
    # zero-index of each sample's plate
    this_exp.sam_plate_indx_nums = (0, 0, 0,
                                    1, 1, 1,
                                    2, 2)  # start at zero
    # zero-index of each sample's well on plate
    this_exp.sam_well_indx_nums = (0, 1, 2,
                                   0, 1, 2,
                                   0, 1)  # (Left A1=0,...,Right A4=3)
    this_exp.sam_inoculation_locations = ((0, 1), (1, 1), (2, 0),
                                          (3, 0), (3, 1), (3, 2),
                                          (4, 0), (5, 0))  # in sample order
    this_exp.sam_targ_incub_times_min = (32, 24, 4,
                                         20, 8, 12,
                                         16, 28)
    # target incubation time (min) for each sample
    # this_exp.sample_targ_incub_gap_times_min = [1, 1, 1, 1, 1, 1, 1, 1]
    # MODIFY code to change when mixing happens
    this_exp.sam_targ_num_mixes = (3, 3, 3,
                                   3, 3, 3,
                                   3, 3)
    # num of times incubating solution is mixed during incubation
    this_exp.sam_targ_num_rinses = (3, 3, 3,
                                    3, 3, 3,
                                    3, 3)
    # num of times incubating solution is rinsed after incubation
    # the list of exp.num_wells_in_plate and other attributes will be filled in config_samples

    # When using jupiter notebook, please check that custom labware json
    # files have been uploaded to /labware for use with
    # 'usctrayvials_3_reservoir_60000ul.json' with max_vol 60mL
    # 'usctrayvials_4_reservoir_32000ul.json' with max_vol 32mL
    # 'usctrayvials_6_reservoir_20000ul.json' with max_vol 20mL
    # 'usctrayvials_6_reservoir_40000ul.json' with max_vol 40mL
    # 'usctrayfoam_4_wellplate_400ul.json' with max_vol 0.4mL
    # Additionally, check that the following hardware is in use:
    # large: 'p1000_single_gen2' on the 'left'
    # with tips: 'opentrons_96_tiprack_1000ul'
    # small: 'p20_single_gen2' on the 'right'
    # with tips: 'opentrons_96_tiprack_20ul'
    # if not, change ExperimentData class in file.

    return this_exp


def run(protocol: protocol_api.ProtocolContext):
    # define the protocol for the OT-2 to run

    def set_speeds(rate_change):
        # needs to be within run() to use with app protocol
        # can be defined outside run() in jupyter lab notebook
        f_out_string = "Adjusting gantry speeds to " + str(rate_change) + " frac of max."
        protocol.comment(f_out_string)  # debug
        print(f_out_string)  # debug
        protocol.max_speeds.update({
            'X': (400 * rate_change),
            'Y': (400 * rate_change),
            'Z': (100 * rate_change),
            'A': (100 * rate_change),
        })
        speed_max = max(protocol.max_speeds.values())
        for instr in protocol.loaded_instruments.values():
            instr.default_speed = speed_max

    def load_plates(slots, names, labels, offsets):
        plates: List[Labware] = []
        for xx in range(len(slots)):
            # eg: custom 3-well reservoir with 'A1', 'A2', 'A3' wells
            this_slot = slots[xx]
            this_name = names[xx]
            this_label = labels[xx]
            this_offset = offsets[xx]
            new_res = protocol.load_labware(this_name, this_slot, label=this_label)
            new_res.set_offset(this_offset[0], this_offset[1], this_offset[2])
            plates.append(new_res)  # each plate can be accessed using # this_plate =  sample_plates[i]
        return plates

    def make_res_array(loc_array):
        res_array = []
        for xx in range(len(loc_array)):
            this_loc = loc_array[xx]
            this_plate = this_loc[0]
            this_well = this_loc[1]
            new_res = reservoir_plates[this_plate].wells()[this_well]
            res_array.append(new_res)
        return res_array

    # set up experiment, exp is global to run()
    exp: ExperimentData = user_config_exp()
    exp = config_samples(exp)
    exp.planned_sequence = create_exp_sequence(exp)

    # MODIFY: "store" pipette tips in the tiprack to use for the same container, but
    # MODIFY: removing pipette tips requires re-homing so adjust time for each action

    # load pipettes and labware onto OT-2 deck
    pipette_sm: pipette_context  # define
    pipette_lg: pipette_context  # define
    if exp.pipettes_in_use == 'small' or exp.pipettes_in_use == 'both':
        tips_sm = []  # set of tip racks for this pipette size
        for xx in range(exp.num_lg_tipracks):
            rack_slot = exp.slots_tiprack_sm[xx]
            new_tiprack = protocol.load_labware(exp.tip_rack_sm_name, rack_slot)
            this_offset = exp.slot_offsets_sm_tips[xx]
            new_tiprack.set_offset(this_offset[0], this_offset[1], this_offset[2])
            tips_sm.append(new_tiprack)
        pipette_sm = protocol.load_instrument(exp.pipette_sm_name, exp.pipette_sm_loc, tip_racks=tips_sm)
    else:
        print("WARNING: No small pipette loaded. (Not needed for this script.)")

    if exp.pipettes_in_use == 'large' or exp.pipettes_in_use == 'both':
        tips_lg = []  # set of tip racks for this pipette size
        for xx in range(exp.num_lg_tipracks):
            rack_slot = exp.slots_tiprack_lg[xx]
            new_tiprack = protocol.load_labware(exp.tip_rack_lg_name, rack_slot)
            this_offset = exp.slot_offsets_lg_tips[xx]
            new_tiprack.set_offset(this_offset[0], this_offset[1], this_offset[2])
            tips_lg.append(new_tiprack)
        pipette_lg = protocol.load_instrument(exp.pipette_lg_name, exp.pipette_lg_loc, tip_racks=tips_lg)
    else:
        print("WARNING: without large pipette, cannot continue script. ")
        raise StopExecution

    reservoir_plates = load_plates(exp.slots_reservoirs, exp.res_plate_names,
                                   exp.labels_res_plates, exp.slot_offsets_res)

    rinse_res_arr = make_res_array(exp.rinse_res_loc)  # array of rinse well objects
    waste_res_arr = make_res_array(exp.waste_res_loc)  # array of waste well objects
    sol_res_arr = make_res_array(exp.sol_res_loc)  # array of solution well objects

    # START HERE: make these global variables?  modify with global keyword?
    # waste_data = exp.waste_data[exp.this_indx_waste]  # choose waste data_set (alias)
    # rinse_data = exp.rinse_data[exp.this_indx_rinse]  # choose rinse data_set (alias)
    # this_res_data = exp.res_data[exp.this_indx_solut]  # choose solution data_set (alias)
    # this_solution = sol_res[exp.this_indx_solut]  # Labware well object for protocol use
    # this_rinse = rinse_res[exp.this_indx_rinse]  # Labware well object for protocol use
    # this_waste = waste_res[exp.this_indx_waste]  # Labware well object for protocol use

    # volume of solution in each sample well, in uL
    well_volume = exp.sam4p_400uL_max_vol  # true volume of each well
    mix_volume = int(0.75 * well_volume)  # volume for mixing
    empty_volume = int(1.5 * well_volume)  # max volume to remove

    sample_plates = load_plates(exp.slots_sam_plates, exp.sam_plate_names,
                                exp.labels_sam_plates, exp.slot_offsets_sam)

    rate = exp.exp_rate_fraction
    set_speeds(rate)
    protocol.set_rail_lights(False)
    pipette_lg.home()

    # print("Solution index is now: ", exp.this_indx_solut)  # debug
    # print("Rinse index is now: ", exp.this_indx_rinse)  # debug
    # print("Waste index is now: ", exp.this_indx_waste)  # debug

    def check_res_empty(well_data):
        if well_data.curr_vol < 1000:
            print("This reservoir is empty.")
            print("Human input needed!")
            raise StopExecution

    def check_rinse_empty(well_data):
        well_data = exp.rinse_data[exp.this_indx_rinse]
        if well_data.curr_vol < 1000:
            print("This rinse reservoir is empty. Switching to next.")
            exp.this_indx_rinse = exp.this_indx_rinse + 1  # increment to next rinse index
            if exp.this_indx_rinse >= exp.max_num_rinse:
                print("We have run out of rinse solution!")
                print("Human input needed!")
                raise StopExecution
            # local variable, define in run() instead
            # rinse_well = rinse_res_arr[exp.this_indx_rinse]  # Labware object for protocol use
        return None

    def check_waste_full(well_data):
        if (well_data.curr_vol + 1000) > well_data.max_vol:
            print("This waste reservoir is full. Switching to next.")
            exp.this_indx_waste = exp.this_indx_waste + 1  # increment to next rinse index
            if exp.this_indx_waste >= exp.max_num_waste:
                print("No space left for waste collection!")
                print("Human input needed!")
                raise StopExecution
            # local variable, define in run() instead
            # waste_well = waste_res_arr[exp.this_indx_waste]  # Labware object for protocol use
        return None

    # MODIFY: add selection for tip position, and which pipette
    def fill_mix_well(this_well, this_res, res_data, this_waste, num_mix=1):
        # needs to within run() to use protocol & pipette
        f_out_string = "Filling well: " + str(this_well)  # debug
        # protocol.comment(f_out_string)  # debug
        print(f_out_string)  # debug

        # MODIFY: modify to use different tips with each sample
        # protocol to fill well from this_reservoir, into this_well, with 1+ mix, keeping the SAME TIP
        pipette_lg.transfer(well_volume, this_res, this_well, mix_after=(num_mix, mix_volume), new_tip='never')
        res_data.curr_vol = res_data.curr_vol - well_volume  # update reservoir volume
        # check_res_empty(res_data)  # checking well volume # res vs rinse!!!

        timestamp_now = math.ceil(time.perf_counter())  # get timestamp of when fill occurred
        pipette_lg.blow_out(location=this_waste.top())  # remove any extra liquid
        pipette_lg.move_to(this_waste.top())  # move pipette to the top of waste

        f_out_string = "Filled well: " + str(this_well) + " at timestamp " + str(timestamp_now)  # debug
        # protocol.comment(f_out_string)  # debug
        print(f_out_string)  # debug
        return timestamp_now

    def mix_well(this_well, this_waste, num_times):
        # uses the same tip, not keeping track of pipette tips
        f_out_string = "Mixing one well: " + this_well  # debug
        protocol.comment(f_out_string)  # debug
        print(f_out_string)

        # MODIFY:  swap tips
        pipette_lg.mix(num_times, mix_volume, this_well)  # mixes solution in this well, num_times

        # clean up
        pipette_lg.blow_out(location=this_waste.top())  # remove extra liquid
        pipette_lg.move_to(this_waste.top())  # after mixing, move pipette to the top of waste
        pipette_lg.touch_tip(this_waste)  # remove drops that may hang on pipette tip
        pipette_lg.move_to(this_waste.top())  # after shaking off drops, move pipette to the top of waste

        timestamp_now = math.ceil(time.perf_counter())  # get timestamp of when mix occurred
        return timestamp_now

    def empty_well(this_well, this_waste, waste_data):
        # instead of using pipette.transfer(), aspirate and
        # dispense (halfway up) with touch_tip and blow_out at out_res
        # empty volume slightly larger than fill volume so that all liq. is evacuated
        f_out_string = "Emptying one well: " + this_well  # debug
        protocol.comment(f_out_string)  # debug
        print(f_out_string)

        # separate aspirate and dispense to change rate/speed
        pipette_lg.aspirate(empty_volume, location=this_well.bottom(), rate=0.5)
        pipette_lg.dispense(empty_volume, location=this_waste.top(), rate=2.0)
        waste_data.curr_vol = waste_data.curr_vol + well_volume  # e.g. well 'A3' waste_res
        check_waste_full(waste_data)  # checking waste volume

        pipette_lg.blow_out(location=this_waste.top())  # remove extra liquid
        pipette_lg.touch_tip(this_waste)  # remove drops by touching tip to sides
        pipette_lg.move_to(this_waste.top())  # after shaking off drops, move pipette to the top of waste
        timestamp_now = math.ceil(time.perf_counter())  # get timestamp of when mix occurred
        return timestamp_now

    def rinse_well(this_well, this_waste, waste_data, in_res, in_res_data):
        pre_rinse_time = math.ceil(time.perf_counter())

        empty_well(this_well, this_waste, waste_data)
        fill_mix_well(this_well, in_res, in_res_data, this_waste, 1)
        check_rinse_empty(in_res_data)  # checking rinse well volume

        timestamp_now = math.ceil(time.perf_counter())
        rinsing_time = math.ceil(timestamp_now - pre_rinse_time)
        output_string = "Rinsing time for sample well " + str(this_well) + " (sec): " + str(rinsing_time)  # debug
        protocol.comment(output_string)  # debug
        return timestamp_now

    def swap_tips(pipette: pipette_context, tiprack: Labware, position: int):
        # swap tips for the pipette,
        # if using different tips for diff solutions
        # MODIFY, add pipette variable and tiprack variable
        pipette.return_tip(home_after=True)
        pipette.pick_up_tip(tiprack.wells()[position])
        # pipette.home()
        return None

    def run_sequence():
        # internal function, so dont need to pass exp, sample_plates, reservoirs, etc
        # run_sequence(exp: ExperimentData, plates_labware: List[Labware]):
        exp_sequence = deepcopy(exp.planned_sequence)
        num_actions = len(exp_sequence)
        all_samples = exp.all_samples  # data for samples (nested list of SampleWell type objects)
        # recall that protocol accessible labware object sample_plates is a list of sample plates!
        zero_timestmp = exp.zero_timestmp

        # shifting the timestamps to zero_timestmp (computer time)
        for act in range(num_actions - 1, -1, -1):
            print(act)
            new_time = exp_sequence[act].start + zero_timestmp
            exp_sequence[act].change_start(new_time)
            # pass

        exp.pln_seq_stamps = exp_sequence

        for ix in range(num_actions):
            # print("___________________________________________")
            # print("ix is now:", ix)  # debug
            this_action = exp_sequence[ix]
            sample_id = this_action.sam_id
            sam_plate_id = exp.sam_plate_indx_nums[sample_id]
            sam_well_id = exp.sam_well_indx_nums[sample_id]
            sam_data = all_samples[sam_plate_id][sam_well_id]  # choose sample data_set (alias)
            this_well = sample_plates[sam_plate_id].wells()[sam_well_id]  # Labware well object for protocol use

            waste_data = exp.waste_data[exp.this_indx_waste]  # choose waste data_set (alias)
            rinse_data = exp.rinse_data[exp.this_indx_rinse]  # choose rinse data_set (alias)
            this_rinse = rinse_res_arr[exp.this_indx_rinse]  # Labware well object for protocol use
            this_waste = waste_res_arr[exp.this_indx_waste]  # Labware well object for protocol use

            goal_time = this_action.start - 10  # start action within 10 seconds of start/end time
            action_type = this_action.action
            timestamp_now = math.ceil(time.perf_counter())  # get timestamp
            # Should experiment pause?
            if timestamp_now < goal_time:
                gap_time = goal_time - timestamp_now
                print("Delay time should be:", gap_time)
                protocol.delay(seconds=gap_time)  # delay protocol!
                # hold in place. pauses the notebook.
                # print("Delay is done") # debug

            # Case: unload  (1)
            if action_type == 'unload':
                print("Unloading sample #: ", sample_id)  # debug
                stamp = rinse_well(this_well, this_waste, waste_data, this_rinse, rinse_data)
                sam_data.incub_end_timestmp = stamp
            # Case: reload (2)
            elif action_type == 'reload':
                print("Reload sample #: ", sample_id)  # debug
                # MODIFY: choose/swap pipette tips
                exp.this_indx_solut = sam_data.solution_index  # change which solution is in use
                this_res_data = exp.res_data[exp.this_indx_solut]  # choose solution data_set (alias)
                this_solution = sol_res_arr[exp.this_indx_solut]  # Labware well object for protocol use
                stamp = fill_mix_well(this_well, this_solution, this_res_data, this_waste, load_mixes)
                check_res_empty(this_res_data)  # checking res-well volume
                sam_data.incub_reload_timestmps.append(stamp)
            # Case: mix (3)
            elif action_type == 'mix':
                print("Mixing sample #: ", sample_id)  # debug
                # MODIFY: choose/swap pipette tips
                stamp = mix_well(this_well, this_waste, 1)
                sam_data.incub_mix_timestmps.append(stamp)
            # Case: load (4)
            elif action_type == 'load':
                print("Loading sample #: ", sample_id)  # debug
                # MODIFY: choose/swap pipette tips
                exp.this_indx_solut = sam_data.solution_index  # change which solution is in use
                this_res_data = exp.res_data[exp.this_indx_solut]  # choose solution data_set (alias)
                this_solution = sol_res_arr[exp.this_indx_solut]  # Labware well object for protocol use
                stamp = fill_mix_well(this_well, this_solution, this_res_data, this_waste, load_mixes)
                check_res_empty(this_res_data)  # checking res-well volume
                sam_data.incub_st_timestmp = stamp
            # Case: rinse (5)
            elif action_type == 'rinse':
                print("Rinsing sample #: ", sample_id)
                stamp = rinse_well(this_well, this_waste, waste_data, this_rinse, rinse_data)
                sam_data.rinse_timestmps.append(stamp)

        print("Done with experiment actions. Updating sample information.")
        num_plates = exp.num_sam_plates
        for plate_index in range(num_plates):
            num_sam_in_plate = exp.num_wells_sam_plates[plate_index]
            for this_well_on_plate in range(num_sam_in_plate):
                sam_data = all_samples[plate_index][this_well_on_plate]  # alias for the sample data
                sam_data.mixed_num = len(sam_data.incub_mix_timestmps)
                sam_data.rinsed_num = len(sam_data.rinse_timestmps)
                sam_data.reloaded_num = len(sam_data.incub_reload_timestmps)
                sam_data.incub_tot_time_s = sam_data.incub_end_timestmp - sam_data.incub_st_timestmp
                prev_stamp = sam_data.incub_st_timestmp
                mix_time_gaps = []
                for this_mix in range(sam_data.mixed_num):
                    time_diff = sam_data.incub_mix_timestmps[this_mix] - prev_stamp
                    prev_stamp = sam_data.incub_mix_timestmps[this_mix]
                    mix_time_gaps.append(time_diff)
                sam_data.incub_mix_time_s = mix_time_gaps
        print("Done, print sample data?")
        # MODIFY: find a way to print sample data to file
        return None

    # pick up pipette tip
    protocol.set_rail_lights(True)
    pipette_lg.pick_up_tip()  # MODIFY: manage pipette tips - assign tips to each well?

    # start experimental timer
    exp.zero_timestmp = math.ceil(time.perf_counter())
    ct = datetime.datetime.now()
    out_string = "Starting Experimental Timer " + str(ct) + "; Timestamp: " + str(exp.zero_timestmp)  # debug
    protocol.comment(out_string)  # debug

    # debug code block
    out_string = "Tip loaded: " + str(pipette_lg.has_tip) + "; Lights On: " + str(protocol.rail_lights_on)  # debug
    protocol.comment(out_string)  # debug

    # run experimental sequence
    run_sequence()

    time_lapse = math.ceil(time.perf_counter() - exp.zero_timestmp)
    ct = datetime.datetime.now()
    out_string = "Ending Experimental Timer " + str(ct) + "TimeLapse (sec): " + str(time_lapse)  # debug
    protocol.comment(out_string)  # debug
    protocol.set_rail_lights(False)
    pipette_lg.return_tip()
    protocol.comment("Completed Experiment.")  # debug
