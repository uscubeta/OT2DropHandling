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
from typing import List, Tuple
import datetime  # using time module
# from threading import Thread
# from typing import Tuple
from opentrons import protocol_api, types
from opentrons.protocol_api_experimental import Labware
from opentrons.protocol_api_experimental import pipette_context

metadata = {
    'apiLevel': '2.13',
    'protocolName': 'Loading the deck to check offsets. Upload to OT2 app. '
                    'To test, replace user_config_exp with current version',
    'description': '''Run by USC on Jan 06, 2023. ''',
    'author': 'Ulyana S. Cubeta',
    'rundate (yyyy/mm/dd)': '2023/01/06'
}

# global variables, used throughout!
# if changing tips during load/mix/rinse, please ADJUST these
load_time_s = 30  # estimated time for 'load' action, in seconds
mix_time_s = 20  # estimated time for 'mix' action, in seconds
rinse_time_s = 60  # estimated time for 'rinse' or 'unload' action
start_time = 30  # seconds, when first action will run
load_mixes = 3


# define class objects (update if updated in main file)

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
        self.all_samples: list[list[SampleWell]] = []
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


class SampleWell:
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
        self.well_name = 'W1'  # name of the well # irrelevant?
        self.targ_incub_time_m = 1  # target time for incubation (in minutes)
        # self.targ_incub_gap_time_m = 1  # target time in between mixing
        self.targ_num_mixes = 1  # target num. of mixes during incubation
        self.targ_num_rinses = 3  # target num. of rinses after removing incub. liquid
        self.rinsed_num = 0  # how often has this sample been rinsed?
        self.mixed_num = 0  # how often has this sample been mixed?
        self.solution_location = (0, 0)  # location for inoculation from exp.sol_res_loc (slot_indx, well_indx)
        self.solution_index = 0  # corresponding to index in exp.res_data
        self.incub_solution = "DI water"  # name of incubation solution
        self.incub_concen = 0  # uM (micromol/L) molarity of incubation solution
        self.incub_st_timestmp = 1208125  # start time collected @ time.perf_counter()
        self.incub_end_timestmp = 1208140  # end time collected @ time.perf_counter()
        self.incub_mix_timestmps = []  # eg:[1208225, 1208325, ]  # mix time collected @ time.perf_counter()
        self.rinse_timestmps = []  # eg: [1208225, 1208325, ]  # mixe time collected @ time.perf_counter()
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
        self._action = action  # string ['load', 'unload', 'mix', 'rinse' , 'done']
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
        elif self.action == 'rinse' or self.action == 'unload':
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
        print("---------------------------------------------------------------")  # debug
        f_out_string = "Sample plate " + str(plate_index) \
                       + " labeled " + plate_labels[plate_index] \
                       + " with " + str(num_wells_on_this_plate) \
                       + " samples."  # debug
        print(f_out_string)  # debug
        # protocol.comment(f_out_string)  # debug
        plate_data_set = []  # each plate's list of SampleWell class
        for this_well_on_plate in range(num_wells_on_this_plate):
            # for each well on this plate
            plate_data_set.append(SampleWell())  # appends object of SampleWell class to list
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
    this_exp.slot_offsets_res = ((0.0, 0.0, 0.0),
                                 (0.0, 0.0, 0.0),
                                 (0.0, 0.0, 0.0),
                                 (0.0, 0.0, 0.0),
                                 (0.0, 0.0, 0.0),
                                 (0.0, 0.0, 0.0))
    # reservoir labware, eg:  res3_60mL_name, res4_32mL_name, res6_20mL_name, res6_40mL_name
    this_exp.res_plate_names = (this_exp.res3_60mL_name,
                                this_exp.res3_60mL_name,
                                this_exp.res4_32mL_name,
                                this_exp.res4_32mL_name,
                                this_exp.res6_40mL_name,
                                this_exp.res6_40mL_name)  # choose one for each slot

    # starting reservoir volumes (1 mL = 1000 uL): list,
    # can break to new line to distinguish plates
    this_exp.res_starting_vols = [50000, 50000, 0,
                                  50000, 50000, 0,
                                  30000, 30000, 30000, 30000,
                                  30000, 30000, 30000, 30000,
                                  40000, 40000, 40000, 40000, 40000, 40000,
                                  40000, 40000, 40000, 40000, 40000, 40000]
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
                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,)  # uM (micromol/L
    this_exp.res_goal_concent = (0.0, 2000.0, 0.0,
                                 0.0, 2000.0, 0.0,
                                 1600.0, 1000.0, 800.0, 600.0,
                                 1600.0, 1000.0, 800.0, 600.0,
                                 400.0, 300.0, 200.0, 100.0, 50.0, 10.0,
                                 400.0, 300.0, 200.0, 100.0, 50.0, 10.0)  # uM (micromol/L

    this_exp.waste_res_loc = ((0, 2), (1, 2))  # list of locations (slot_indx, well_indx) for waste
    this_exp.rinse_res_loc = ((0, 0), (1, 0))  # list of locations (slot_indx, well_indx) for DI rinse
    this_exp.sol_res_loc = ((0, 1),
                            (1, 1),
                            (2, 0), (2, 1), (2, 2), (2, 3),
                            (3, 0), (3, 1), (3, 2), (3, 3),
                            (4, 0), (4, 1), (4, 2), (4, 3), (4, 4), (4, 5),
                            (5, 0), (5, 1), (5, 2), (5, 3), (5, 4), (5, 5))  # loc list (slot_indx, well_indx)

    # list of tipracks for pipettes (MODIFY if re-using/returning tips)
    this_exp.slots_tiprack_lg = (10, 11)  # slots 1-11 on OT-2
    # this_exp.slot_tip_rack_sm = (10, 11)  # slots 1-11 on OT-2
    this_exp.slot_offsets_lg_tips = ((-1.10, 1.50, -0.10),
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
    # set up experiment
    exp: ExperimentData = user_config_exp()
    exp = config_samples(exp)

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

    # load labware plates on deck slots
    def load_plates(slots, names, labels, offsets):
        plates: List[Labware] = []
        for xx in range(len(slots)):
            # custom 3-well reservoir with 'A1', 'A2', 'A3' wells
            this_slot = slots[xx]
            this_name = names[xx]
            this_label = labels[xx]
            this_offset = offsets[xx]
            new_res = protocol.load_labware(this_name, this_slot, label=this_label)
            new_res.set_offset(this_offset[0], this_offset[1], this_offset[2])
            plates.append(new_res)  # each plate can be accessed using # this_plate =  plate_set[i]
        return plates

    reservoir_plates = load_plates(exp.slots_reservoirs, exp.res_plate_names,
                                   exp.labels_res_plates, exp.slot_offsets_res)

    sample_plates = load_plates(exp.slots_sam_plates, exp.sam_plate_names,
                                exp.labels_sam_plates, exp.slot_offsets_sam)

    def make_res_array(loc_array):
        res_array = []
        for xx in range(len(loc_array)):
            this_loc = loc_array[xx]
            this_plate = this_loc[0]
            this_well = this_loc[1]
            # make sure that reservoir_plates is a global variable
            new_res = reservoir_plates[this_plate].wells()[this_well]
            res_array.append(new_res)
        return res_array

    rinse_res = make_res_array(exp.rinse_res_loc)  # array of rinse well objects
    # waste_res = make_res_array(exp.waste_res_loc)  # array of waste well objects
    # sol_res = make_res_array(exp.sol_res_loc)  # array of solution well objects
    this_rinse = rinse_res[exp.this_indx_rinse]  # Labware well object for protocol use
    # this_waste = waste_res[exp.this_indx_waste]  # Labware well object for protocol use

    # begin experiment, set speeds and home
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

    rate = exp.exp_rate_fraction
    set_speeds(rate)
    protocol.set_rail_lights(False)
    pipette_lg.home()

    # pick up pipette tip
    protocol.set_rail_lights(True)
    pipette_lg.pick_up_tip()

    # some actions here....
    some_vol = 1000
    for plate in reservoir_plates:
        pipette_lg.transfer(some_vol, this_rinse, plate.wells(), new_tip='never')
    some_vol = 400
    for plate in sample_plates:
        pipette_lg.transfer(some_vol, this_rinse, plate.wells(), new_tip='never')

    # return pipette tip
    protocol.set_rail_lights(False)
    pipette_lg.return_tip()
    protocol.comment("Completed Experiment.")  # debug
