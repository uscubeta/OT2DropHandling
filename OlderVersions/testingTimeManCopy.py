# To Start, run Anaconda prompt, navigate with change directory ($:cd ...) and type in:
# $: conda env list # to check if the environment is available
# $: activate OT2DropHandling  # to activate the environment
# $: jupyter-lab
# this assumes that the environment was created using:
# $: conda create --name OT2DropHandling --file OT2DropHandlingEnv.txt
# AND that the following packages added via pip
# $ pip install -U pyvisa
# $ pip install opentrons
# $ pip install ffmpy # to interact with camera on OT2
# you can check if the above three packages are installed with...
# $ pip list
# for details, https://docs.opentrons.com/v2/new_protocol_api.html
# robot IP address: http://169.254.59.185:48888

import json
import time
import math
from copy import deepcopy
from typing import List
import datetime  # using time module
from threading import Thread
from typing import Tuple
from opentrons import protocol_api, types

metadata = {
    'apiLevel': '2.13',
    'protocolName': 'testing Time management during sample incubation',
    'description': '''Run by USC on Dec 15, 2022. ''',
    'author': 'Ulyana S. Cubeta',
    'rundate': '2022/12/15'
}
# global variables, used throughout!
# if changing tips during load/mix/rinse, please ADJUST these
load_time_s = 30  # estimated time for 'load' action, in seconds
mix_time_s = 20  # estimated time for 'mix' action, in seconds
rinse_time_s = 60  # estimated time for 'rinse' or 'unload' action


# define class objects
class ExperimentData:
    # need a way to save this metadata (MODIFY!!)
    def __init__(self):
        self.exp_rate_fraction = 0.25  # % of default speeds (to reduce shaking of gantry)
        self.exp_name = "newExperiment"
        self.exp_date = 20221207  # yyyymmdd
        self.zero_timestmp = 1208125
        self.now_timestmp = 1208125
        # tuples must be replaced, not edited in place
        self.sam_names = ("sample1",)
        self.sam_plate_indx_nums = (0,)
        self.sam_well_indx_nums = (0,)
        self.sam_well_names = ('A1',)
        self.sam_targ_incub_times_min = (10,)
        # self.sample_targ_incub_gap_times_min = (5, )
        self.sam_targ_num_mixes = (1,)
        self.sam_targ_num_rinses = (1,)
        self.sam_indx_in_order = (0,)  # matches to sample_names
        self.sam_timing = (0,)  # points to each sample in order of events
        self.sample_well_vol = 400
        self.rinse_res_num = 0  # 'A1'
        self.waste_res_num = 2  # 'A3'
        self.sol_res_num = 1  # 'A2'
        self.res_volumes = [10, 10, 0]  # vol of rinse,sol,& waste
        self.num_samples = 1  # calculated in config_samples
        self.num_plates = 1  # calculated in config_samples
        self.num_wells_in_plate = (2,)  # calculated in config_samples
        self.slots_plates = (2,)  # list of slots on OT-2 (eg. num 1 through 11)
        self.slot_tip_rack_lg = 11  # slot on OT-2 (MODIFY code for multiple tip racks)
        self.slot_tip_rack_sm = 10  # slot on OT-2 (MODIFY code for multiple tip racks)
        self.slot_reservoir = 1  # slot on OT-2 (MODIFY code for multiple reservoirs)
        self.slot_res_offset = (0.0, 0.0, 0.0)  # calibration offset (x,y,z)
        self.slot_tips_offset = (0.0, 0.0, 0.0)  # calibration offset (x,y,z)
        self.slot_plate_offsets = ((0.0, 0.0, 0.0),)  # calibration offset (x,y,z)
        self.reservoir_name = 'usctrayvials_3_reservoir_60000ul'
        self.plate_name = 'usctrayfoam_4_wellplate_400ul'
        self.tip_rack_lg_name = 'opentrons_96_tiprack_1000ul'
        self.tip_rack_sm_name = 'opentrons_96_tiprack_20ul'
        self.pipette_lg_name = 'p1000_single_gen2'
        self.pipette_sm_name = 'p20_single_gen2'
        self.pipette_lg_loc = 'left'
        self.pipette_sm_loc = 'right'
        self.planned_sequence = List[ActClass,]  # in seconds
        self.pln_seq_stamps = List[ActClass,]
        # where each tuple is (sample_index, action, start_time_s, end_time_s)
        # eg: [(0, 'load', 10, 20), (0, 'mix', 110, 120), (0, 'mix', 210, 220), (0, 'rinse', 310, 320)]

    def __repr__(self):
        this_string = "ExperimentData(" + str(self.exp_name) + ")"
        return this_string

    def __str__(self):
        this_string = "ExperimentData(" + str(self.exp_name) + ")"
        return this_string


class SampleWell:
    # need a way to save this metadata (MODIFY!!)
    def __init__(self):
        self.sample_name = "USC22Blnk1118a"  # name on the QR-coded label
        self.sam_indx = 0  # sample index from the original user_config_exp
        self.well_plate_slot = 3  # slot on on OT-2 (eg. num 1 through 11)
        self.plate_indx = 0  # plate index for this sample - for use with nested list of sample_set
        self.well_indx = 0  # well index for this sample on plate_indx plate, 0 to 3 for L to R
        self.loc_indx = (0, 0)  # (plate_indx, well_indx) # tuple has to be replaced, not edited
        self.sam_timing = 0  # where first sample we should load is 0, & last is (sam_num-1)
        self.well_name = 'W1'  # name of the well # irrelevant?
        self.targ_incub_time_m = 1  # target time for incubation (in minutes)
        # self.targ_incub_gap_time_m = 1  # target time in between mixing
        self.targ_num_mixes = 1  # target num. of mixes during incubation
        self.targ_num_rinses = 3  # target num. of rinses after removing incub. liquid
        self.rinsed_num = 0  # how often has this sample been rinsed?
        self.incub_solution = "DI water"  # name of incubation solution
        self.incub_concen = "0 M"  # molarity of incubation solution
        self.incub_st_timestmp = 1208125  # start time collected @ time.perf_counter()
        self.incub_end_timestmp = 1208140  # end time collected @ time.perf_counter()
        self.incub_mix_timestmps = [1208225, 1208325, ]  # mixe time collected @ time.perf_counter()
        self.rinse_timestmps = [1208225, 1208325, ]  # mixe time collected @ time.perf_counter()
        self.incub_tot_time_s = 15  # integer, in seconds
        self.incub_mix_time_s = [100, 200, ]  # integer, in seconds
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
        # intializing function, _attribute is a hidden attribute !
        self._sam_id = sam_id  # integer of samples indexed 0 to num_sam
        # in the order entered by user_config_exp
        self._action = action  # string ['load', 'unload', 'mix', 'rinse' ]
        self._start_stamp = stamp  # integer of start timestamp (seconds) for the desired action
        self._end_stamp = self._calc_end()  # calculated, estimated end timestamp

    # returns this when calling this object
    def __repr__(self):
        # MODIFY: check if using property works in this function
        this_string = "(" + str(self.sam_id) + ", '" + \
                      str(self.action) + "', " + str(self.start) + \
                      ", " + str(self.end) + ")"
        return this_string

    # returns this string when called via print(x)
    def __str__(self):
        # MODIFY: check if using property works in this function
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
        # MODIFY: check if using property works in this function
        if self.action == 'load':
            prev_end_stamp = self._start_stamp + load_time_s
        elif self.action == 'mix':
            prev_end_stamp = self._start_stamp + mix_time_s
        elif self.action == 'rinse' or self.action == 'unload':
            prev_end_stamp = self._start_stamp + rinse_time_s
        else:
            print("The following is not a known action:", self.action)
            prev_end_stamp = -1
        return prev_end_stamp


# define functions on these class objects
def give_list_order(some_list: tuple[int]):
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


def swap_actions(exp_sequence: List[ActClass], pos1: int, pos2: int):
    # swap the two actions in the list
    num_actions = len(exp_sequence)
    if pos1 >= num_actions or pos2 >= num_actions:
        print("WARNING: Cannot swap actions, positions are out of index range.")
        return exp_sequence
    exp_sequence[pos1], exp_sequence[pos2] = exp_sequence[pos2], exp_sequence[pos1]
    # print("Swapping actions for the positions: ", pos1, " and ", pos2)  # debug
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


def check_order_swap(exp_sequence: List[ActClass], sam_indx: tuple[int], this_pos: int, old_pos: int):
    # if two actions are of the same type, check the order in the sam_indx
    old_action = exp_sequence[old_pos]
    this_action = exp_sequence[this_pos]
    old_action_indx = sam_indx.index(old_action.sam_id)
    this_action_indx = sam_indx.index(this_action.sam_id)
    if this_action_indx < old_action_indx:
        swap_actions(exp_sequence, this_pos, old_pos)
    return exp_sequence


# have user fill out for each experiment
def user_config_exp():
    this_exp = ExperimentData()
    this_exp.exp_name = "TestingTimeManagement"  # change every run please
    this_exp.exp_date = 20221207  # yyyymmdd # update every run please
    this_exp.slot_reservoir = 1  # slots 1-11 on OT-2 (MODIFY code for multiple reservoirs)
    this_exp.slot_res_offset = (-0.20, 2.10, -1.30)  # calibration offset for labware
    this_exp.slot_tip_rack_lg = 4  # slots 1-11 on OT-2  (MODIFY code for multiple racks)
    this_exp.slot_tips_offset = (-1.10, 1.50, -0.10)  # calibration offset for labware
    # list of slots corresponds to index of plates 0,1,2,... (num_plates-1)
    this_exp.slots_plates = (2, 3, 5, 6)  # slots 1-11 on OT-2 for the plates (indexed 0,1,2...)
    this_exp.slot_plate_offsets = ((0.00, 2.30, 0.10),
                                   (0.10, 1.40, 0.20),
                                   (-0.30, 1.60, 0.40),
                                   (-0.10, 0.80, 0.40),)  # calibration offset for labware
    # the samples must be listed in order for the time being, modify code if out of order
    # sample index will be 0,1,2,...(num_samples-1), but order of inoculation will be calculated in config_samples
    this_exp.sam_names = ("USC22Blnk1128a", "USC22Blnk1128b", "USC22Blnk1128c",
                          "USC22Blnk1128d", "USC22Blnk1128e", "USC22Blnk1128f",
                          "USC22Blnk1128g", "USC22Blnk1128h")
    this_exp.sam_well_names = ('W1', 'W2', 'W3', 'W4', 'W5', 'W6', 'W7', 'W8')  # irrelevant?
    this_exp.sam_plate_indx_nums = (0, 0, 1, 1, 2, 2, 3, 3)  # zero-index of each plate, start at zero
    this_exp.sam_well_indx_nums = (0, 1, 0, 1, 0, 1, 0, 1)  # zero-index of each well on plate (L=0,...,R=3)
    this_exp.sam_targ_incub_times_min = (32, 24, 4, 20, 8, 12, 16, 28)
    # target incubation time (min) for each sample
    # this_exp.sample_targ_incub_gap_times_min = [1, 1, 1, 1, 1, 1, 1, 1]
    # MODIFY code to change when mixing happens
    this_exp.sam_targ_num_mixes = (3, 3, 3, 3, 3, 3, 3, 3)
    # num of times incubating solution is mixed during incubation
    this_exp.sam_targ_num_rinses = (3, 3, 3, 3, 3, 3, 3, 3)
    # num of times incubating solution is rinsed after incubation
    # the list of exp.num_wells_in_plate and other attributes will be filled in config_samples

    this_exp.rinse_res_num = 0  # 'A1'  (MODIFY code for multiple reservoirs)
    this_exp.sol_res_num = 1  # 'A2'  (MODIFY code for multiple reservoirs)
    this_exp.waste_res_num = 2  # 'A3'  (MODIFY code for multiple reservoirs)
    this_exp.res_volumes = [50000, 50000, 0]  # vol of rinse,sol,& waste, in uL (1 mL = 1000 uL)
    # Do not modify  unless changing pipettes or labware (if so, please MODIFY code!)
    # custom labware json MUST be uploaded to /labware for use with jupiter notebook
    # custom 3-well reservoir with 'A1', 'A2', 'A3' wells,
    this_exp.reservoir_name = 'usctrayvials_3_reservoir_60000ul'
    # custom 4-well well plate with 'A1', 'A2', 'A3', 'A4' wells
    this_exp.plate_name = 'usctrayfoam_4_wellplate_400ul'
    this_exp.sample_well_vol = 400  # uL, volume of each sample well
    this_exp.tip_rack_lg_name = 'opentrons_96_tiprack_1000ul'  # do not modify
    this_exp.pipette_lg_name = 'p1000_single_gen2'  # do not modify
    this_exp.pipette_lg_loc = 'left'  # install location of large pipette
    # this_exp.tip_rack_sm_name = 'opentrons_96_tiprack_20ul' # do not modify
    # this_exp.pipette_sm_name = 'p20_single_gen2' # do not modify
    # this_exp.pipette_sm_loc = 'right'  # install location of small pipette

    return this_exp


def run(protocol: protocol_api.ProtocolContext):
    # define the protocol for the OT-2 to run

    def set_speeds(rate_change):
        # needs to be within run() to use with app protocol
        # can be defined outside run() in jupyter lab notebook
        f_out_string = "Adjusting gantry speeds to " + str(rate_change) + "frac of max."
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

    # needs to be before exp is defined
    # MODIFY: move to outside run(), but how to
    # both change exp parameters and return set?
    def config_samples():
        # check the number of plates & samples in configuration
        # both modifies exp variables and returns sample_set
        # (list of lists of objects of class SampleWell !)
        # function needs to be within run() to use with app protocol
        exp.num_plates = len(exp.slots_plates)
        exp.num_samples = len(exp.sam_plate_indx_nums)

        # check that the number of items in user_config_experiment is consistent
        num_sam = exp.num_samples
        if len(exp.sam_names) != num_sam or len(exp.sam_well_names) != num_sam \
                or len(exp.sam_targ_incub_times_min) != num_sam or \
                len(exp.sam_targ_num_rinses) != num_sam:
            f_out_string = "Something is off. Please check user_config_experiment()"
            print(f_out_string)  # debug for notebook
            # protocol.pause(f_out_string)  # debug for OT2 app
        else:
            f_out_string = "The user_config_experiment() was written correctly. "
            print(f_out_string)  # debug for notebook
            # protocol.comment(f_out_string)  # debug for OT2 app

        # first, find the order of sample incubation:
        # sort by the length of incubation time to produce index of wells in time-order
        exp.sam_indx_in_order = give_list_order(exp.sam_targ_incub_times_min)  # inoculation order
        # then, find the order that the wells should be inoculated and then incubated
        # where each value exp.sam_timing[i] indicates WHEN that sample(exp.sample_names[i])
        # should be loaded, where a value of 0 is FIRST, & (sam_num-1) is LAST
        exp.sam_timing = give_list_order(exp.sam_indx_in_order)  # when to inoculate, sam-ordered!

        # sort the samples by plate location and by well
        # pass the user input into the record of well info
        sample_set = []  # all samples, will return
        # list of plates [0 to num_plates-1] with
        # list of samples [0 to num_sam_in_plate-1] where each
        # object in the NESTED list is of class SampleWell objects
        num_plates = exp.num_plates
        num_wells_in_plate = list(range(num_plates))  # placeholder for list of num_wells in each plate
        samples_on_prev_plates = 0  # for sample_index from 0 to num_sam-1
        for plate_index in range(num_plates):
            # for each plate in on the deck, indexed 0 to num_plates-1
            num_sam_in_plate = 0
            for check_sam in range(num_sam):
                # not the fastest way, since we go through all samples num_plates times
                if exp.sam_plate_indx_nums[check_sam] == plate_index:
                    num_sam_in_plate = num_sam_in_plate + 1
            # record the number of wells in each plate
            num_wells_in_plate[plate_index] = num_sam_in_plate
            f_out_string = "The number of samples in this plate " \
                           + str(plate_index) + " is " + str(num_sam_in_plate)  # debug
            print(f_out_string)  # debug
            # protocol.comment(f_out_string)  # debug
            plate_data_set = []  # each plate's list of SampleWell class
            for this_well_on_plate in range(num_sam_in_plate):
                # for each well on this plate
                plate_data_set.append(SampleWell())  # appends object of SampleWell class to list
                sample = plate_data_set[this_well_on_plate]
                this_sam_indx = samples_on_prev_plates + this_well_on_plate
                sample.sam_indx = this_sam_indx
                sample.sample_name = exp.sam_names[this_sam_indx]
                sample.sam_timing = exp.sam_timing[this_sam_indx]
                sample.plate_indx = exp.sam_plate_indx_nums[this_sam_indx]
                sample.well_plate_slot = exp.slots_plates[plate_index]
                sample.well_indx = exp.sam_well_indx_nums[this_sam_indx]
                sample.well_name = exp.sam_well_names[this_sam_indx]
                sample.targ_num_rinses = exp.sam_targ_num_rinses[this_sam_indx]
                sample.targ_num_mixes = exp.sam_targ_num_mixes[this_sam_indx]
                sample.targ_incub_time_m = exp.sam_targ_incub_times_min[this_sam_indx]
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
                sam_timestamp = targ_inc_time_s
                this_action = ActClass(this_sam_indx, 'unload', sam_timestamp)
                exp_sequence.append(this_action)
                sam_timestamp = sam_timestamp + rinse_time_s
                for i in range(sample.targ_num_rinses):
                    this_action = ActClass(this_sam_indx, 'rinse', sam_timestamp)
                    exp_sequence.append(this_action)
                    sam_timestamp = sam_timestamp + rinse_time_s
                sample.targ_act_seq = exp_sequence
                # debug block...
                print("Sample name: ", sample.sample_name, "; for well: ",
                      this_well_on_plate, "; for sample index: ",
                      this_sam_indx)  # debug
                print(sample.targ_act_seq)  # debug
                # print(vars(sample))  # debug
            samples_on_prev_plates = samples_on_prev_plates + num_sam_in_plate
            sample_set.append(plate_data_set)
            for x in sample_set:
                for y in x:
                    # print(y.sample_name)  # debug
                    pass
        exp.num_wells_in_plate = tuple(num_wells_in_plate)  # record as a tuple, no mods
        #  return list of lists of objects of class SampleWell
        return sample_set

    # set up experiment
    start_time = 30  # seconds
    exp = user_config_exp()
    all_samples = config_samples()

    # can be moved out of run(), fully encapsulated!

    def prioritize_sequence(exp_sequence: List[ActClass], sam_indx: tuple[int]):
        # sort the list by timestamp, then iterate over the sorted list
        # if two timestamps overlap, rearrange the order in the following way:
        # prioritize in order (1) unload (2) mix (3) load (4) rinse
        # does NOT change the TIMEstamp, just order!
        # if two actions are the same (eg. both 'mix'),
        # use the sample load order to choose which goes first

        print("Prioritizing and sorting list.")  # debug
        exp_sequence.sort(key=lambda sort_action: sort_action.start)
        # print("Concatenated, sorted sequence is: ")  # debug
        # print(exp_sequence)  # debug
        # old_action = ActClass(0, 'none', 0)
        for ix in range(1, len(exp_sequence)):
            # iterate over list of actions, starting with the second action
            old_action = exp_sequence[(ix - 1)]
            this_action = exp_sequence[ix]
            old_action_indx = sam_indx.index(old_action.sam_id)
            this_action_indx = sam_indx.index(this_action.sam_id)
            # if two timestamps overlap
            # MODIFY: need to check against sam_index when two actions are the same
            if this_action.start == old_action.start:
                # Case: unload  (1) vs all others
                if this_action.action == 'unload':
                    if old_action.action == 'unload':
                        if old_action_indx > this_action_indx:
                            #swap_time_w_gap(exp_sequence, ix, (ix - 1), rinse_time_s)
                            continue
                        else:
                            swap_actions(exp_sequence, ix, (ix - 1))
                            continue
                elif this_action.action == 'unload' and old_action.action != 'unload':
                    swap_actions(exp_sequence, ix, (ix - 1))
                    continue
                elif this_action.action == 'unload' and this_action.action == 'unload':
                    if old_action_indx > this_action_indx:
                        # old_action should come second
                        swap_time_w_gap(exp_sequence, ix, (ix - 1), rinse_time_s)
                elif this_action.action == 'mix' and old_action.action != 'unload' \
                        and old_action.action != 'mix':
                    swap_actions(exp_sequence, ix, (ix - 1))
                    continue
                elif this_action.action == 'load' and old_action.action != 'unload' and \
                        old_action.action != 'mix' and old_action.action != 'load':
                    swap_actions(exp_sequence, ix, (ix - 1))
                    continue
                else:
                    # print("Actions are in the correct order.")  # debug
                    pass
        return exp_sequence

    def shift_all_for_load(exp_sequence: List[ActClass], sam_index: int, shift_time: int):
        # make a list of all load actions that come after sam_index 'load'
        # shift all actions for samples in that list by a time shift_time
        # then sorts and prioritizes the action list
        print("Shifting all for load of action:", sam_index)  # debug
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

    def find_end_stamp(some_action: ActClass):
        # calculate when the some action will end
        if some_action.action == 'load':
            prev_end_stamp = some_action.start + load_time_s
        elif some_action.action == 'mix':
            prev_end_stamp = some_action.start + mix_time_s
        elif some_action.action == 'rinse' or some_action.action == 'unload':
            prev_end_stamp = some_action.start + rinse_time_s
        else:
            print("The following is not a known action", some_action)
            prev_end_stamp = -1
        return prev_end_stamp

    def shift_timestamp(exp_sequence: List[ActClass], sam_indx: tuple[int]):
        # next, iterate over the sorted list and shift all except unload
        # shift the action if the previous one has a timestamp that
        # overlaps with the time needed to complete the previous action
        # prioritize in order (1) unload (2) mix (3) load (4) rinse
        # old_action = ActClass(0, 'none', 0)
        print("Shifting timestamps for the list of actions.")
        ix = 1
        while ix < len(exp_sequence):
            # for ix in range(1, len(exp_sequence)): # changed to while loop so ix can be repeated
            # iterate over list of actions, starting with the second action
            old_action = exp_sequence[(ix - 1)]
            this_action = exp_sequence[ix]
            prev_end_stamp = find_end_stamp(old_action)  # calc when previous action will end
            print("ix is now:", ix)  # debug
            # if timestamps overlap, modify exp_sequence, then REPEAT ix
            if this_action.start < prev_end_stamp:
                print("=====================================================================================")  # debug
                print(ix, ":", this_action, " starts at ", this_action.start,
                      "before ", old_action, " ends at ", prev_end_stamp)  # debug
                # prioritize in order (1) unload (2) mix (3) load (4) rinse
                # Case: unload  (1)
                if this_action.action == 'unload':
                    print("Unload shouldn't move! So moving previous action to afterwards. ")  # debug
                    # Case: unload (1) + unload (1)
                    if old_action.action == 'unload':
                        print("Shoot, two unloads overlap! What to do?")
                        print("Maybe shift all actions for that unload?")
                    # Case: unload (1) + load (3)
                    elif old_action.action == 'load':
                        print("Previous action is load, so need to shift load")
                        sam_id = old_action.sam_id
                        shift_time = this_action.start + rinse_time_s - old_action.start
                        # target load time should start at prev_end_stamp, so shift 'load' by the difference
                        print("Load needs to shift for all loads/steps after sam_id: ",
                              sam_id, " by time: ", shift_time)  # debug
                        exp_sequence = shift_all_for_load(exp_sequence, sam_id, shift_time)
                    # Case: unload (1) + (mix (2) or rinse(4))
                    else:
                        old_action.change_start(this_action.start + rinse_time_s)
                        # exp_sequence = prioritize_sequence(exp_sequence)
                # Case: mix (2)
                elif this_action.action == 'mix':
                    print("Mix can be moved earlier or later.")  # debug
                    # MODIFY: if next action is also mix, the mixes start to pile up and be resorted!
                    # Case: mix (2) + unload (1)
                    if old_action.action == 'unload':
                        print("Previous action was 'unload', so moving 'mix' to later.")  # debug
                        this_action.change_start(old_action.start + rinse_time_s)  # new time to start mixing
                        # but this may mean the number of mixes will not be sufficient
                        # MODIFY: check that the mix will not come after unload for the same sample
                    # Case: mix (2) + mix (2)
                    elif old_action.action == 'mix':
                        print("Two mix actions overlap, moving second mix to later.")  # debug
                        this_action.change_start(old_action.start + mix_time_s)  # new time to start mixing
                    # Case: mix (2) + load (3)
                    elif old_action.action == 'load':
                        this_action.change_start(old_action.start)  # swapping time to mix, will sort during shift
                        print("Previous action was 'load', so moving 'mix' to earlier:", this_action)  # debug
                        # change the time on this mix action to the time that the old action starts
                        sam_id = old_action.sam_id  # the sample id the previous "load" action
                        exp_sequence = shift_all_for_load(exp_sequence, sam_id, mix_time_s)  # shifting by mix_time
                    # Case: mix (2) + rinse (4)
                    elif old_action.action == 'rinse':
                        print("Previous action was 'rinse', so swapping for 'mix' to come first.")  # debug
                        # change the time on this mix action to the time that the old action starts
                        new_time_to_mix = old_action.start  # new time to start mixing
                        old_action.change_start(new_time_to_mix + mix_time_s)  # new time to start rinse action
                        this_action.change_start(new_time_to_mix)  # change timestamp for mixing
                        # may be unnecessary since prioritize_sequence sorts by time anyway
                        swap_actions(exp_sequence, ix, (ix - 1))  # swap the two actions (this and previous)
                # Case: load (3)
                elif this_action.action == 'load':
                    # Case: load (3) + rinse (4)
                    if old_action.action == 'rinse':
                        # moving old_action's timestamp to later
                        old_action.change_start(this_action.start + load_time_s)
                        print("Shifting rinse action to later: ", old_action)  # debug
                    # Case: load (3) + load (3)
                    elif old_action.action == 'load':
                        old_action_indx = sam_indx.index(old_action.sam_id)
                        this_action_indx = sam_indx.index(this_action.sam_id)
                        if old_action_indx > this_action_indx:
                            # old_action should come second
                            swap_time_w_gap(this_action, old_action, load_time_s)
                    # Case: load (3) + (unload (1) or mix (2))
                    else:
                        sam_id = this_action.sam_id
                        shift_time = prev_end_stamp - this_action.start
                        # target load time should start at prev_end_stamp, so shift 'load' by the difference
                        print("Load needs to shift for all loads/steps after sam_id: ",
                              sam_id, " by time: ", shift_time)  # debug
                        exp_sequence = shift_all_for_load(exp_sequence, sam_id, shift_time)
                        print("Updated load action is, ", this_action)  # debug
                # Case: rinse (4)
                elif this_action.action == 'rinse':
                    # moving this_action's timestamp to later
                    # Case: rinse (4) + rinse (4)
                    if old_action.action == 'rinse':
                        old_action_indx = sam_indx.index(old_action.sam_id)
                        this_action_indx = sam_indx.index(this_action.sam_id)
                        if old_action_indx > this_action_indx:
                            # old_action should come second
                            swap_time_w_gap(this_action, old_action, rinse_time_s)
                        else:
                            this_action.change_start(prev_end_stamp)
                            # Case: rinse (4) + (unload (1) or mix (2) or load(3))
                    else:
                        this_action.change_start(prev_end_stamp)
                    print("Updated rinse action to later, ", this_action)  # debug
                else:
                    print()

                # reprioritizing, since a change was likely
                exp_sequence = prioritize_sequence(exp_sequence, sam_indx)
                ix = ix - 1  # going back one , to check that old_action moved correctly
                # MODIFY: this doesnt work in a for loop, maybe try a while loop
                print("Changing ix to :", ix)  # debug
            else:
                # iterate to the next ix
                ix = ix + 1
        return exp_sequence

    def create_exp_sequence():
        # def create_exp_sequence(sample: SampleWell)

        # get the time-ordered list of sample loading ( of sam indx)
        sam_indx_in_order = exp.sam_indx_in_order
        num_samples = exp.num_samples

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
        exp_sequence = prioritize_sequence(exp_sequence, sam_indx_in_order)
        print("Concatenated, sorted, swapped sequence is: ")  # debug
        print(exp_sequence)  # debug
        exp_sequence = shift_timestamp(exp_sequence, sam_indx_in_order)
        print(exp_sequence)

        # next, check if two timestamps overlap and
        # prioritize in order (1) unload (2) mix (3) load (4) rinse

        # action_tuple = (sample_index, 'load', time_pt)
        # exp_sequence.append(action_tuple)
        # time_pt = time_pt + load_time
        # targ_incub = targ_inc_times[sample_index] + time_pt
        # samp_unload_timept.append(targ_incub)

        for i in range(unload_next, num_samples):
            sample_index = sam_indx_in_order[i]
            action_tuple = (sample_index, 'mix', time_pt)
            exp_sequence.append(action_tuple)
            time_pt = time_pt + mix_time

            samp_gap_times.append(targ_gap_times[sample_index])
            if samp_unload_timept[unload_next] <= time_pt:
                print("yikes, time to unload before all are loaded")  # debug
                print("timept is ", time_pt)  # debug
                action_tuple = (sam_indx_in_order[unload_next], 'rinse', time_pt)
                exp_sequence.append(action_tuple)
                time_pt = time_pt + rinse_time
                unload_next = unload_next + 1

    # MODIFY: "store" pipette tips in the tiprack to use for the same container, but...
    # MODIFY: removing pipette tips can displace the motor and may require re-homing
    # load labware onto OT-2 deck
    tips_lg = protocol.load_labware(exp.tip_rack_lg_name, exp.slot_tip_rack_lg)
    pipette_lg = protocol.load_instrument(exp.pipette_lg_name, exp.pipette_lg_loc, tip_racks=[tips_lg])
    tips_lg.set_offset(exp.slot_tips_offset[0], exp.slot_tips_offset[1], exp.slot_tips_offset[2])

    # custom 3-well reservoir with 'A1', 'A2', 'A3' wells
    reservoirs = protocol.load_labware(exp.reservoir_name, exp.slot_reservoir)
    reservoirs.set_offset(exp.slot_res_offset[0], exp.slot_res_offset[1], exp.slot_res_offset[2])
    rinse_res = reservoirs.wells()[exp.rinse_res_num]
    waste_res = reservoirs.wells()[exp.waste_res_num]
    solution_res = reservoirs.wells()[exp.sol_res_num]
    well_volume = exp.sample_well_vol  # uL of solution in each sample well
    mix_volume = int(0.75 * well_volume)
    empty_volume = int(1.5 * well_volume)

    plate_set = []
    p_name = exp.plate_name
    # load labware plates to OT-2
    for p_indx in range(exp.num_plates):
        p_slot = exp.slots_plates[p_indx]
        p_label = "plate" + str(p_indx + 1)
        new_plate = protocol.load_labware(p_name, p_slot, label=p_label)
        this_offset = exp.slot_plate_offsets[p_indx]
        reservoirs.set_offset(this_offset[0], this_offset[1], this_offset[2])
        plate_set.append(new_plate)  # each plate can be accessed using # this_plate =  plate_set[i]

    rate = exp.exp_rate_fraction
    set_speeds(rate)
    protocol.set_rail_lights(False)
    pipette_lg.home()

    def load_wells(plate_index, fill_plate, this_res, res_num):
        # needs to within run() to use protocol & pipette
        com_out_string = "Filling wells in plate:" + str(fill_plate)  # debug
        protocol.comment(com_out_string)  # debug
        well_list = fill_plate.wells()  # need to link sample_set to wells...
        plate_data = all_samples[plate_index]
        exp_start_stamp = exp.zero_timestmp
        for well in well_list:
            fill_mix_well(well, this_res, res_num, 3)  # modify to capture timestamp

    def fill_mix_well(this_well, this_res, res_num, num_mix=1):
        # needs to within run() to use protocol & pipette
        out_string = "Filling well: " + str(this_well)  # debug
        protocol.comment(out_string)  # debug

        pipette_lg.transfer(well_volume, this_res, this_well, mix_after=(num_mix, mix_volume), new_tip='never')
        timestamp_now = math.ceil(time.perf_counter())
        pipette_lg.move_to(rinse_res.top())
        exp.res_volumes[res_num] = exp.res_volumes[res_num] - well_volume

        out_string = "Filled well: " + str(this_well) + " at timestamp " + str(timestamp_now)  # debug
        protocol.comment(out_string)  # debug
        return timestamp_now

    def mix_well(this_well):
        pipette_lg.mix(1, mix_volume, this_well)
        timestamp_now = math.ceil(time.perf_counter())
        pipette_lg.move_to(rinse_res.top())
        return timestamp_now

    def empty_well(this_well, out_res, out_num):
        # instead of using pipette.transfer(), aspirate and
        # dispense (halfway up) with touch_tip and blow_out at out_res
        # empty volume slightly larger than fill volume so that all liq. is evacuated
        output_string = "Emptying one well: " + this_well  # debug
        protocol.comment(output_string)  # debug
        pipette_lg.aspirate(empty_volume, location=this_well.bottom(), rate=0.5)
        # pipette.touch_tip(location=out_res)  # remove drops
        pipette_lg.dispense(empty_volume, location=out_res.bottom(40.0), rate=2.0)
        pipette_lg.blow_out(location=out_res.top())  # remove extra liquid
        pipette_lg.touch_tip()  # remove drops
        res_volumes[out_num] = res_volumes[out_num] + well_volume  # e.g. well 'A3' waste_res

    def rinse_well(this_well, out_res, out_res_num, in_res, in_res_num):
        pre_rinse_time = math.ceil(time.perf_counter())
        empty_well(this_well, out_res, out_res_num)
        fill_mix_well(this_well, in_res, in_res_num, 1)
        timestamp_now = math.ceil(time.perf_counter())
        rinsing_time = math.ceil(timestamp_now - pre_rinse_time)
        output_string = "Rinsing time for sample well " + str(this_well) + " (sec): " + str(rinsing_time)  # debug
        protocol.comment(output_string)  # debug
        return timestamp_now

    # start experimental timer
    exp.zero_timestmp = math.ceil(time.perf_counter())
    ct = datetime.datetime.now()
    out_string = "Starting Experimental Timer " + str(ct) + "; Timestamp: " + str(exp.zero_timestmp)  # debug
    protocol.comment(out_string)  # debug

    # pick up pipette tip
    protocol.set_rail_lights(True)
    pipette_lg.pick_up_tip()  # MODIFY: manage pipette tips - assign tips to each well?

    # debug code block
    out_string = "Tip loaded: " + str(pipette_lg.has_tip) + "; Lights On: " + str(protocol.rail_lights_on)  # debug
    protocol.comment(out_string)  # debug

    # fill wells, check timer
    for p_indx in range(exp.num_plates):
        plate = plate_set[p_indx]
        load_wells(p_indx, plate, solution_res, 1)
    pipette_lg.move_to(rinse_res.top())
    exp.now_timestmp = math.ceil(time.perf_counter())
    load_time_min = round(((exp.now_timestmp - exp.zero_timestmp) / 60), 2)  # debug
    out_string = "Time to load sample plate (min): " + str(load_time_min)  # debug
    protocol.comment(out_string)  # debug

    # target_timestamp = start_time + targ_inc_times_min * 60
    if timestamp_now < target_timestamp:
        gap_time_sec = target_timestamp - timestamp_now
        out_string = "Pausing for " + str(gap_time_sec) + " seconds"  # debug
        protocol.comment(out_string)  # debug
        protocol.delay(seconds=gap_time_sec)
    time_lapse = math.ceil(time.perf_counter() - exp.zero_timestmp)
    ct = datetime.datetime.now()
    out_string = "Ending Experimental Timer " + str(ct) + "TimeLapse (sec): " + str(time_lapse)  # debug
    protocol.comment(out_string)  # debug
    protocol.set_rail_lights(True)
    pipette_lg.return_tip()
    protocol.comment("Completed Experiment.")  # debug
