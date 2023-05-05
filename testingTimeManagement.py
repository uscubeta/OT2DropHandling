# To Begin, run Anaconda prompt, navigate with change directory ($:cd ...) and type in:
# $: conda env list # to check if the environment is available
# $: activate OT2DropHandling  # to activate the environment
# $: jupyter-lab  # this runs a notebook on YOUR computer, not on OT-2's
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
    'description': '''Run by USC on Jan 17, 2023. ''',
    'author': 'Ulyana S. Cubeta',
    'rundate': '2023/01/17'
}

# GLOBAL variables, used throughout!
DO_DEBUG = True


# MODIFY: move these class objects and functions to a different file and import
# define class objects
class StopExecution(Exception):
    def _render_traceback_(self):
        pass


class ExperimentData:
    # need a way to save this metadata (MODIFY!!)
    def __init__(self):
        # Note: tuples() must be replaced, not edited in place
        self.exp_rate_fraction = 0.25  # % of default speeds (to reduce shaking of gantry)
        self.exp_name = "newExperiment"
        self.exp_date = 20221207  # yyyymmdd
        self.zero_timestmp = 1208125
        self.now_timestmp = 1208125

        # The following values are calculated in config_samples:
        # number of racks of type: tipracks, reservoir, or sample
        self.num_lg_tipracks = 0  # number of tiprack plates for large pipette
        self.num_sm_tipracks = 0  # number of tiprack plates for small pipette
        self.num_res_plates = 0  # number of plates holding reservoirs
        self.num_sam_plates = 0  # number of plates holding samples
        # total number of tips, reservoirs, samples
        self.tot_num_lg_tips = 0  # total number of lg tips
        self.tot_num_sm_tips = 0  # total number of sm tips
        self.tot_num_sams = 0  # total number of sample wells
        self.tot_num_res = 0  # total number of reservoir wells
        # number of each set waste, rinse, and solution
        self.tot_num_waste = 0
        self.tot_num_rinse = 0
        self.tot_num_sol = 0
        # total number of actions sample and dilution planned and executed
        self.tot_num_sam_act = 0  # total number of sample actions (planned_sequence)
        self.tot_num_dil_act = 0  # total number of sample actions (planned_sequence)
        self.comp_sam_act = -1  # completed sample load/unload actions
        self.comp_dil_act = -1  # completed reservoir dilution actions

        # slots on the OT-2 (e.g. int 1 through 11)
        self.slots_sam_plates = (0,)  # well-plate sample slots, eg: (2, 3)
        self.slots_res_racks = (0,)  # reservoir plate slots, eg: (1, 4)
        self.slots_tiprack_lg = (0,)  # large tip rack slots, eg: (10, 11)
        self.slots_tiprack_sm = (0,)  # small tip rack slots, eg: (10, 11)

        # tip location tracking via (rack_num, well_id)
        self.available_tips_lg = ()  # list of locations for lg tips
        self.available_tips_sm = ()  # list of locations for sm tips
        self.which_tip_lg = 0  # index from available_tips_lg location list, used during planning
        self.which_tip_sm = 0  # index from available_tips_sm location list, used during planning
        self.cur_lg_tip = ()  # currently-loaded lg tip
        self.cur_sm_tip = ()  # currently-loaded sm tip
        self.tips_used = []  # list of tips used (wet) during solution handling
        # where 0 is A1, 1 is B1....8 is A2,... 88 is A12, ... etc to 95 for a full set of tips
        # and each tuple within the nest corresponds to each rack, with first value slot_num
        self.tips_in_lg_racks = ((10, [*range(0, 96, 1)]),)  # nested list of tips in rack(s)
        self.tips_in_sm_racks = ((10, [*range(0, 96, 1)]),)  # nested list of tips in rack(s)

        # reservoir labware, eg:  'res3_60mL', 'res4_32mL', 'res6_20mL', or 'res6_40mL'
        self.res_plate_names = ((1, 'label_name'),)  # tuple of tuples : (slot_#,'label')
        # sample labware, eg:  'sam4_400uL' or 'sam8_400uL'
        self.sam_plate_names = ((1, 'label_name'),)  # tuple of tuples : (slot_#,'label')

        # offset for each piece of labware (tuple of tuples - 4 values)
        self.offsets_sam_racks = ((1, 0.0, 0.0, 0.0),)  # calibration offset (slot_#, x,y,z)
        self.offsets_res_racks = ((1, 0.0, 0.0, 0.0),)  # calibration offset (slot_#, x,y,z)
        self.offsets_lg_tiprx = ((1, 0.0, 0.0, 0.0),)  # calibration offset (slot_#, x,y,z)
        self.offsets_sm_tiprx = ((1, 0.0, 0.0, 0.0),)  # calibration offset (slot_#, x,y,z)

        # locations tuple of tuples, consisting of (slot_num, well_indx)
        self.waste_res_locs = ((1, 0),)  # list of locations (slot_num, well_indx)
        self.rinse_res_locs = ((1, 0),)  # list of locations (slot_num, well_indx)
        self.sol_res_locs = ((1, 0),)  # list of locations (slot_num, well_indx)

        # location, start/end volumes (1mL=1000uL) & concentrations (uM=umol/L)
        # ((Slot_#, Well_#, 'res'), (start_vol_uL, start_conc_uM), (end_vol_uL, end_conc_uM), & 'contents')
        # location, start/end volumes (1mL=1000uL) & concentrations (uM=umol/L)
        # eg: zero-index of 6 well goes 0:A1, 1:B1, 2:A2, 3:B2, 4:A3, 5:B3
        self.input_res_data = (((1, 0, 'res'), (0, 0), (0, 0), 'DI_sol'),)  # user_config_exp
        # (sam_loc, (num_inoc,(inoc_sol_loc)), (targ_incub,num_mixes,num_rinses), 'sam_name')
        # sam_loc: (rack_num, well_id, 'sam')  # sample location - 'sam' should be last
        # inoc_sol_locs: (1, (rack_num, well_id, 'sol', vol_frac))  # inoculation solution loc
        # for multiple inoculation solutions, inoc_sol_loc = (num_sol, (sol_1),(sol_2),...)
        # targ_incub: number of minutes to incubate solution on sample
        # num_mixes: num of times incubating solution is mixed during incubation
        # num_rinses: num of times solution is rinsed after incubation
        self.input_sam_data = (((2, 0, 'sam'), (1, (5, 0, 'sol'),), (20, 3, 4), 'sam_name'),)  # user_config_exp
        self.incub_loc_order = ((1, 0),)  # inoculated sam order, by loc, longest incubation first
        self.max_incub_m = 1  # maximum incubation time for all samples

        # currently, using these wells as:
        self._cur_waste = None  # eg: (1, 2)  # updated, as dig_vol or curr_vol is exceeded
        self._cur_rinse = None  # eg: (1, 0)  # updated, as dig_vol or curr_vol is depleted

        self.do_dilutions = False
        self.start_dry = True
        self.store_dry = False
        self.incub_longest_first = True
        # maximum amount of time before evaporation of sample well
        self.max_time_before_evap_m = 60  # minutes
        self.content_types = ['type1', ]  # excludes waste/rinse
        self.num_cont_types = 0  # number of self.content_types
        self.rev_dil_order = []
        self.forward_dil_or = []
        # self.res_ids = []  # same order as res_data, correlates to the input (subset from zero to num_res_tot)

        # custom 3-well 60mL reservoir plate with 'A1', 'A2', 'A3' wells,
        self.res3_60mL_name = 'usctrayvials_3_reservoir_60000ul'
        self.res3_60mL_max_vol = 60000  # uL (max volume of above reservoir plate)
        self.res3_60mL_label = 'Res3_60mL_'  # 3-well 60mL res plate label
        # custom 4-well 32mL reservoir plate plate with 'A1', 'A2', 'A3', 'A4' wells
        self.res4_32mL_name = 'usctrayvials_4_reservoir_32000ul'
        self.res4_32mL_max_vol = 32000  # uL (max volume of above reservoir plate)
        self.res4_32mL_label = 'Res4_32mL_'  # 4-well 32mL res plate label
        # custom 6-well 20mL reservoir plate with (A1-3, B1-3) wells
        self.res6_20mL_name = 'usctrayvials_6_reservoir_20000ul'
        self.res6_20mL_max_vol = 20000  # uL (max volume of above reservoir plate)
        self.res6_20mL_label = 'Res6_20mL_'  # 6-well 20mL res plate label
        # custom 6-well 40mL reservoir plate with (A1-3, B1-3) wells
        self.res6_40mL_name = 'usctrayvials_6_reservoir_40000ul'
        self.res6_40mL_max_vol = 40000  # uL (max volume of above reservoir plate)
        self.res6_40mL_label = 'Res6_40mL_'  # 6-well 40mL res plate label
        # custom 4-well sample wellplate with 'A1', 'A2', 'A3', 'A4' wells
        self.sam4_400uL_name = 'usctrayfoam_4_wellplate_400ul'
        self.sam4_400uL_max_vol = 400  # uL (max volume of sample well-plate)
        self.sam4_400uL_label = 'Sam4Plate_'  # 4-well plate label
        # custom 8-well sample plate w/ 'A1','B1','A2','B2','A3','B3','A4','B4' wells
        self.sam8_400uL_name = 'usctrayfoam_8_wellplate_400ul'
        self.sam8_400uL_max_vol = 400  # uL (max volume of sample well-plate)
        self.sam8_400uL_label = 'Sam8Plate_'  # 8-well plate label

        # large: 1000uL (1mL) pipette can dispense from 100+/-2uL to 1000+/-7uL of solution
        # small: 20uL pipette can dispense from 1+/-0.15uL to 20+/-0.3uL of solution
        # not loaded but available: 300uL pipette dispensing 20 to 300uL
        self.pipettes_in_use = 'large'  # 'large', 'small' or 'both'
        self.tip_rack_lg_name = 'opentrons_96_tiprack_1000ul'
        self.tip_rack_sm_name = 'opentrons_96_tiprack_20ul'
        self.pipette_lg_name = 'p1000_single_gen2'
        self.pipette_sm_name = 'p20_single_gen2'
        self.pipette_lg_loc = 'left'  # pipette hardware mounted on the left
        self.pipette_sm_loc = 'right'  # pipette hardware mounted on the right

        self.planned_sequence: list[ActionInfo] = []  # in seconds
        self.pln_seq_stamps: list[ActionInfo] = []  # in seconds
        self.pln_dilut_seq: list[ActionInfo] = []  # in seconds
        # where each tuple is (sample_index, action, start_time_s, end_time_s)
        # eg: [(0, 'load', 10, 20), (0, 'mix', 110, 120), (0, 'mix', 210, 220), (0, 'rinse', 310, 320)]
        self.all_samples: list[list[SamWellData]] = []
        self.all_res_data: list[list[ResWellData]] = []
        self.res_plate_wells: list[RackData] = []
        self.sam_plate_wells: list[RackData] = []
        # self.res_data_locs: ((0, 0),)  # tuple of locations, same order as res_data

    def __repr__(self):
        this_string = "ExperimentData(" + str(self.exp_name) + ")"
        return this_string

    def __str__(self):
        this_string = "ExperimentData(" + str(self.exp_name) + ")"
        return this_string

    def find_sam_in_nest_list(self, loc: (int, int), sam_list=None):
        # for the sample with given location (loc) = (plate_indx, well_indx)
        # finds the nested indices (#,#) in the list, sam_list
        # if nested list is not specified, using self.all_samples
        if sam_list is None:
            sam_list = self.all_samples
        main_list_length = len(sam_list)
        for rack_indx in range(main_list_length):
            sub_list = sam_list[rack_indx]
            sub_list_length = len(sub_list)
            for well_indx in range(sub_list_length):
                sam_data = sub_list[well_indx]
                if sam_data.loc == loc:
                    indices = (rack_indx, well_indx)
                    return indices
        s_out = "Loc " + str(loc) + " is not in this nested list of samples!"
        raise ValueError(s_out)

    def find_res_in_nest_list(self, loc: (int, int), res_list=None):
        # for the reservoir with given location (loc) = (plate_indx, well_indx)
        # finds the nested indices (#,#) in the list, res_list
        # if nested list is not specified, using self.all_res_data
        if res_list is None:
            res_list = self.all_res_data
        main_list_length = len(res_list)
        for rack_indx in range(main_list_length):
            sub_list = res_list[rack_indx]
            sub_list_length = len(sub_list)
            for well_indx in range(sub_list_length):
                res_data = sub_list[well_indx]
                if res_data.loc == loc:
                    indices = (rack_indx, well_indx)
                    return indices
        s_out = "Loc " + str(loc) + " is not in this nested list of reservoirs!"
        raise ValueError(s_out)

    def find_rack_in_res_plates(self, slot_num: int, rack_list=None):
        # for rack with given slot number,
        # finds the index in the list of racks,
        # if list not specified, using self.res_plate_wells
        if rack_list is None:
            rack_list = self.res_plate_wells
        list_length = len(rack_list)
        for indx in range(list_length):
            rack = rack_list[indx]
            if rack.slot == slot_num:
                return indx
        s_out = "Slot # " + str(slot_num) + " is not in this list of reservoirs!"
        raise ValueError(s_out)

    def find_rack_in_sam_plates(self, slot_num: int, rack_list=None):
        # for rack with given slot number,
        # finds the index in the list of racks,
        # if list not specified, using self.sam_plate_wells
        if rack_list is None:
            rack_list = self.sam_plate_wells
        list_length = len(rack_list)
        for indx in range(list_length):
            rack = rack_list[indx]
            if rack.slot == slot_num:
                return indx
        s_out = "Slot # " + str(slot_num) + " is not in this list of reservoirs!"
        raise ValueError(s_out)

    def find_next_tip(self, which_pip: str):
        # finds the next tip to use in experiment planning
        # tracked via which_tip_lg and which_tip_sm
        if which_pip == 'small':
            self.cur_sm_tip = self.available_tips_sm[self.which_tip_sm]
            self.which_tip_sm += 1
            if self.which_tip_sm >= self.tot_num_sm_tips:
                str_out = "Small pipette has insufficient tips. " \
                          "Please load more, update user_config_exp() and restart."
                print(str_out)
                raise StopExecution
            new_tip = self.available_tips_sm[self.which_tip_sm]
        else:
            self.cur_lg_tip = self.available_tips_lg[self.which_tip_lg]
            self.which_tip_lg += 1
            if self.which_tip_lg >= self.tot_num_lg_tips:
                str_out = "Large pipette has insufficient tips. " \
                          "Please load more, update user_config_exp() and restart."
                print(str_out)
                raise StopExecution
            new_tip = self.available_tips_lg[self.which_tip_lg]
        return new_tip

    def give_rinse_loc(self, pull_vol: int = 0):
        if self._cur_rinse is None:
            # if current rinse not assigned, select the first location
            self._cur_rinse = self.rinse_res_locs[0]
            return self._cur_rinse  # return location
        res_loc = self._cur_rinse
        res_indx = self.find_res_in_nest_list(res_loc)  # find indices for sol, in all_res_data
        res = self.all_res_data[res_indx[0]][res_indx[1]]  # select ResWellData from all_res_data
        dig_vol = res.dig_vol
        goal_vol = dig_vol - pull_vol
        if goal_vol < 1000:
            # less than a mL remaining in Rinse with this step, switch to next Rinse res!
            print("Digital volume depleted, switching to next Rinse reservoir.")
            indx = self.rinse_res_locs.index(res_loc)
            indx += 1
            if indx >= self.tot_num_waste:
                str_out = "WARNING: experiment does not have sufficient waste containers. " \
                          "Load more and restart planning phase. "
                raise ValueError(str_out)
            self._cur_rinse = self.rinse_res_locs[indx]  # updating current rinse location
        return self._cur_rinse

    def give_waste_loc(self, add_vol: int = 0):
        if self._cur_waste is None:
            # if current waste not assigned, select the first location
            self._cur_waste = self.waste_res_locs[0]
            return self._cur_waste  # return location
        res_loc = self._cur_waste
        res_indx = self.find_res_in_nest_list(res_loc)  # find indices for sol, in all_res_data
        res = self.all_res_data[res_indx[0]][res_indx[1]]  # select ResWellData from all_res_data
        dig_vol = res.dig_vol
        goal_vol = dig_vol + add_vol
        max_vol = res.max_vol - 1000  # max vol less 1000uL
        if goal_vol > max_vol:
            # less than a mL of headspace remaining, switching to next Waste res!
            print("Digital volume exceeded, switching to next Waste reservoir.")
            indx = self.waste_res_locs.index(res_loc)
            indx += 1
            if indx >= self.tot_num_waste:
                str_out = "WARNING: experiment does not have sufficient waste containers. " \
                          "Load more and restart planning phase. "
                raise ValueError(str_out)
            self._cur_waste = self.waste_res_locs[indx]
        return self._cur_waste

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
        elif this_name == self.sam4_400uL_name:
            volume = self.sam4_400uL_max_vol
        elif this_name == self.sam8_400uL_name:
            volume = self.sam8_400uL_max_vol
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
        elif this_name == self.sam4_400uL_name:
            this_label = self.sam4_400uL_label
        elif this_name == self.sam8_400uL_name:
            this_label = self.sam8_400uL_label
        return this_label


class RackData:
    def __init__(self, slot_num: int, res_type: str):
        self.slot = slot_num  # slot_# on OT-2 Deck (slots 1-11)
        self.type = res_type  # 'res' or 'sam'
        self.offset = (0.0, 0.0, 0.0)
        self.name = ""
        self.label = ""
        self.max_vol = 0
        self.num_wells = 0
        self.well_ids = []

    # returns this when calling this object
    def __repr__(self):
        this_string = str(self.type) + " rack in slot #" + str(self.slot) + " named " + str(self.name) + \
                      ", labeled " + str(self.label) + " with " + str(self.num_wells) + \
                      " wells of max vol " + str(self.max_vol) + "uL, indexed " + str(self.well_ids)
        return this_string

    # returns this string when called via print(x)
    def __str__(self):
        this_string = str(self.type) + " rack in slot #" + str(self.slot) + " named " + str(self.name) + \
                      ", labeled " + str(self.label) + " with " + str(self.num_wells) + \
                      " wells of max vol " + str(self.max_vol) + "uL, indexed " + str(self.well_ids)
        return this_string


class ResWellData:
    def __init__(self):
        self.loc = (1, 0)  # (Slot_#, Well_#), location of this reservoir
        self.slot_num = 0  # Slot_# for this reservoir's rack, 1-11 on OT2 Deck
        self.well_id = 0  # Well_# for this reservoir, 0-(num_wells-1) on rack
        self.assigned_tip = (11, 0)  # (Tip_Slot_#, Tip_#), tip assigned to this reservoir to reduce contamination
        self.contents = ''  # solution name/chemical name for this reservoir
        self.curr_conc = 0  # uM (micromol/L) # used for dilutions  ('starting' and 'achieved' concentration)
        self.goal_conc = 0  # uM (micromol/L) # used for dilutions  (goal concentration)
        self.curr_vol = 0  # uL - 'starting' & 'current' volume (tracked during solution processing, not digital twin)
        self.goal_vol = 0  # uL - ending volume after dilutions, prior to use for sample inoculation
        self.dig_vol = 0  # uL - digital twin - keeping track of volume during planning stage, while writing actions
        self.max_vol = 10000  # uL, maximum volume for this reservoir well
        self.dilution_complete = True  # is dilution complete?  changed after physical action, not digital twin
        self.parent_loc = (0, 0)  # (Slot_#, Well_#) #  parent for dilution
        self.parent_conc = 0  # uM, parent concentration for dilution
        self.par_transf_vol = 0  # uL, transfer volume from parent to this child reservoir

    # returns this when calling this object
    def __repr__(self):
        this_string = "Res: " + str(self.loc) + "; tip: " + str(self.assigned_tip) + \
                      "; contents: " + str(self.contents) + "; max vol " + str(self.max_vol) + \
                      "; starting (vol,conc): (" + str(self.curr_vol) + ", " + str(self.curr_conc) + ")" + \
                      "; ending (vol,conc): (" + str(self.goal_vol) + ", " + str(self.goal_conc) + ")" + \
                      "; dil. complete: " + str(self.dilution_complete)
        return this_string

    # returns this string when called via print(x)
    def __str__(self):
        this_string = "Res: " + str(self.loc) + "; tip: " + str(self.assigned_tip) + \
                      "; contents: " + str(self.contents) + "; max vol " + str(self.max_vol) + \
                      "; starting (vol,conc): (" + str(self.curr_vol) + ", " + str(self.curr_conc) + ")" + \
                      "; ending (vol,conc): (" + str(self.goal_vol) + ", " + str(self.goal_conc) + ")" + \
                      "; dil. complete: " + str(self.dilution_complete)
        return this_string


class SamWellData:
    # need a way to save this metadata (MODIFY!!)
    # may want to switch to getters/setters/properties
    def __init__(self):
        self.sample_name = "USC22Blnk1118a"  # name on the QR-coded label of the sample
        self.loc = (3, 0)  # (plate_indx, well_indx) # tuple has to be replaced, not edited
        self.slot_num = 3  # slot on on OT-2 (eg. num 1 through 11)
        self.well_id = 0  # well index for this sample on plate_indx plate, 0 to 3 for L to R
        self.assigned_tip = (0, 0)  # (rack, well) - tip assigned to sample to reduce contamination

        self.targ_incub_time_m = 1  # int, target time for incubation (in minutes)
        self.targ_incub_time_s = 60  # int, target time for incubation (in seconds)
        self.targ_num_mixes = 1  # target num. of mixes during incubation
        self.targ_num_rinses = 3  # target num. of rinses after removing incub. liquid
        self.targ_num_reload = 0  # if incubation time is long, will remove and reload incubation liquid
        self.rinsed_num = 0  # how often has this sample been rinsed?
        self.mixed_num = 0  # how often has this sample been mixed?
        self.reloaded_num = 0  # how often has this sample been reloaded?

        self.max_vol = 400  # uL, volume of sample wells, in uL
        self.dig_vol = 0  # uL, digital twin - keeping track of volume during planning stage, while writing actions
        self.cur_vol = 0  # uL, volume tracked during solution handling
        self.start_dry = True
        self.store_dry = False

        self.sam_inoculation_timing = 0  # where first sample we should load is 0, & last is (sam_num-1)
        self.num_inoc_sol = 1  # number of inoculation solutions
        self.inoc_locs = ((5, 1),)  # solution locs and volume fractions
        self.inoc_fracs = (1.0,)  # volume fraction of each incubation solution
        self.incub_sols = ("DI water",)  # names of incubation solution
        self.incub_conc = (0,)  # uM (micromol/L) molarity of incubation solutions

        self.incub_st_timestmp = 1208125  # start time collected @ time.perf_counter()
        self.incub_end_timestmp = 1208140  # end time collected @ time.perf_counter()
        self.incub_reload_timestmps = []  # eg:[1208225, 1208325, ]  # reload time collected @ time.perf_counter()
        self.incub_mix_timestmps = []  # eg:[1208225, 1208325, ]  # mix time collected @ time.perf_counter()
        self.rinse_timestmps = []  # eg: [1208225, 1208325, ]  # rinse time collected @ time.perf_counter()
        self.incub_tot_time_s = 15  # integer, in seconds
        self.incub_mix_time_s = []  # eg: [100, 200, ]  # integer, in seconds
        self.incub_rinse_time_s = []  # eg: [100, 200, ]  # integer, in seconds
        self.targ_act_seq = List[ActionInfo]  # action list for each sample
        self.pln_seq_stamps = List[ActionInfo]  # action list with shifted timestamps
        # where each action is of the class:
        # ActClass(keeper_loc, 'action', start_time_s,
        #           parent_loc, targ_loc, transf_vol,
        #           tip_loc, num_mixes, is_complex)

    # returns this when calling this object
    def __repr__(self):
        this_string = "SampleWell(" + str(self.sample_name) + ")"
        return this_string

    # returns this string when called via print(x)
    def __str__(self):
        this_string = "SampleWell(" + str(self.sample_name) + "," + \
                      str(self.loc) + "," + str(self.loc) + ")"
        return this_string


class ActionInfo:
    def __init__(self, keeper: (int, int), sub_action: str, top_action: str, order_num: int, start_time_s: int,
                 parent=(1, 0), targ=(1, 0), vol=0, tip=(11, 0), num_mixes=3, is_complex=True):
        # par = from loc, targ = to loc, tip loc, vol, and is_complex are optional parameters
        # initializing function, _attribute is a hidden attribute!
        self._transf_time_s = 20  # est. time, in seconds, for 'transfer' SIMPLE action
        self._mix_time_s = 20  # est. time, in seconds, for 'mix' SIMPLE action
        self._load_time_s = 40  # est. time, in seconds, for 'load' COMPLEX action
        self._rinse_time_s = 60  # est. time (s) for 'reload' or 'rinse' or 'unload' COMPLEX actions
        self._keeper = keeper  # (rack_num, well_id)  # action belongs to 'keeper'
        # independent of from_loc and to_loc
        self._action = sub_action  # complex or simple action string
        # complex: ['load', 'reload', 'unload', 'rinse'] - multi step, multi-tips
        # simple:  [ 'mix', 'transf' ] - one step, one tip
        self._top_act = top_action  # complex action string
        # complex: ['load', 'reload', 'unload', 'rinse', 'dilution', 'only_mix']
        self._order_num = order_num  # order number used for sorting actions
        self._complex = is_complex  # set to True whe complex actions are expanded to simple ones
        # complex actions can be swapped, but simple are expanded and cannot be swapped
        self._start_stamp = start_time_s  # integer of start timestamp (seconds) for the desired action
        self._end_stamp = self._calc_end()  # calculated, estimated end timestamp
        self._time_stamp = (self._start_stamp, self._end_stamp)  # timestamp
        self._tip_loc = tip  # tip_loc (rack_slot,well_loc) - for SIMPLE actions
        self._from_loc = parent  # parent location, transfer 'from' (rack_slot,well_loc)
        self._targ_loc = targ  # target location, transfer 'to' (rack_slot,well_loc)
        self._transf_vol = vol  # uL target transfer amount from_loc to targ_loc
        self._num_mixes = num_mixes  # number of mixes for each SIMPLE 'mix' action,
        # or for the 'mix' in a COMPLEX action

    # returns this when calling this object
    def __repr__(self):
        this_string = "(" + str(self._time_stamp) + ", '" + \
                      str(self.action) + "', from:" + str(self._from_loc) + \
                      ", to:" + str(self._targ_loc) + ", vol:" + str(self._transf_vol) + \
                      ", tip: " + str(self._tip_loc) + ")"
        return this_string

    # returns this string when called via print(x)
    def __str__(self):
        this_string = "(" + str(self._time_stamp) + ", '" + \
                      str(self.action) + "', from:" + str(self._from_loc) + \
                      ", to:" + str(self._targ_loc) + ", vol:" + str(self._transf_vol) + \
                      ", tip: " + str(self._tip_loc) + ")"
        return this_string

    # setter methods
    def change_start(self, new_start: int):
        self._start_stamp = new_start
        self._end_stamp = self._calc_end()
        self._time_stamp = (self._start_stamp, self._end_stamp)

    def change_tip(self, set_tip: (int, int)):
        self._tip_loc = set_tip

    def set_targ(self, targ_loc: (int, int)):
        self._targ_loc = targ_loc

    def set_parent(self, par_loc: (int, int)):
        self._from_loc = par_loc

    def set_num_mixes(self, num_mixes: int):
        self._num_mixes = num_mixes

    def set_large_act(self, action: str):
        self._top_act = action

    # property/ getter methods
    @property
    def keeper(self):
        return self._keeper

    @property
    def action(self):
        return self._action

    @property
    def complex(self):
        return self._complex

    @property
    def start(self):
        return self._start_stamp

    @property
    def end(self):
        return self._end_stamp

    @property
    def stamp(self):
        return self._time_stamp

    @property
    def num_mixes(self):
        return self._num_mixes

    @property
    def par_loc(self):
        return self._from_loc

    @property
    def targ_loc(self):
        return self._targ_loc

    @property
    def tip_loc(self):
        return self._tip_loc

    @property
    def length(self):
        time2complete = self._end_stamp - self._start_stamp
        return time2complete

    # other functions
    def _calc_end(self):
        # MODIFY: check that global variables are defined.
        if self.action == 'mix':
            prev_end_stamp = self._start_stamp + self._mix_time_s
        elif self.action == 'transf':
            prev_end_stamp = self._start_stamp + self._transf_time_s
        elif self.action == 'load':
            prev_end_stamp = self._start_stamp + self._load_time_s
        else:
            prev_end_stamp = self._start_stamp + self._rinse_time_s
        return prev_end_stamp


# define functions on these class objects
def swap_actions(exp_sequence: List[ActionInfo], act_pos_this: int, act_pos_that: int):
    # swap the two actions in the list  (^ this_action, old_action)
    # print("Swapping these actions: ", exp_sequence[pos2], exp_sequence[pos1])
    num_actions = len(exp_sequence)
    if act_pos_this >= num_actions or act_pos_this < 0 \
            or act_pos_that >= num_actions or act_pos_that < 0:
        print("WARNING: Cannot swap actions, positions are out of index range.")
        return exp_sequence
    exp_sequence[act_pos_this], exp_sequence[act_pos_that] = exp_sequence[act_pos_that], exp_sequence[act_pos_this]
    # print("Swapping actions for the positions: ", act_pos_this, " and ", act_pos_that)  # debug
    # print("Now they're in order: ", exp_sequence[act_pos_this], exp_sequence[act_pos_that])  # debug
    return exp_sequence


def check_order_swap(exp_sequence: List[ActionInfo], sam_indx: Tuple[int], this_pos: int, old_pos: int):
    # if two actions are of the same type, check the order in the sam_indx
    old_action = exp_sequence[old_pos]
    this_action = exp_sequence[this_pos]
    # print("Checking if ", old_pos, ":", old_action, "should precede", this_pos, ":", this_action)  # debug
    old_action_sam_indx = sam_indx.index(old_action.keeper)
    this_action_sam_indx = sam_indx.index(this_action.keeper)
    # this means if two actions have the same index, they are probably in the correct order!
    if this_action_sam_indx < old_action_sam_indx:
        print(this_pos, ":", this_action, " should come before ", old_pos, ":", old_action, " Swapping them.")  # debug
        swap_actions(exp_sequence, this_pos, old_pos)
    return exp_sequence


def swap_time_w_gap(exp_sequence: List[ActionInfo], be_first_pos: int, be_second_pos: int, gap_time: int):
    # maybe should be an inner function for shift_timestamp
    # print("Swapping timestamps for the actions in positions: ", be_first_pos, " and ", be_second_pos)  # debug
    # print("Before swap:", exp_sequence[be_first_pos], " and ", exp_sequence[be_second_pos])
    be_first_time = exp_sequence[be_second_pos].start  # new time to start first action
    exp_sequence[be_second_pos].change_start(be_first_time + gap_time)  # new time for second action
    exp_sequence[be_first_pos].change_start(be_first_time)  # change timestamp for first action
    # print("After swap:", exp_sequence[be_first_pos], " and ", exp_sequence[be_second_pos])
    return exp_sequence


def shift_all_for_load(exp_sequence: List[ActionInfo], this_sam_id: int, shift_time: int):
    # make a list of all load actions that come after sam_index 'load'
    # shift all actions for samples in that list by a time shift_time
    # then sorts and prioritizes the action list
    # print("Shifting all for load of action:", sam_index)  # debug
    shift_for_load = False
    sams_2_shift = []  # list of samples that are loaded after this_sam_id
    for ix in range(len(exp_sequence)):
        # find all load actions that come after sam_index load
        this_action = exp_sequence[ix]
        if this_action.keeper == this_sam_id:
            if this_action.action == 'load':
                shift_for_load = True
        if shift_for_load and this_action.action == 'load':
            sams_2_shift.append(this_action.keeper)
            # list of samples that are loaded after this_sam_id

    print("Shifting all actions for samples: ", sams_2_shift)  # debug
    for ix in range(len(exp_sequence)):
        # shift timestamp for all actions for samples in the list
        this_action = exp_sequence[ix]
        if this_action.keeper in sams_2_shift:
            this_action.change_start(this_action.start + shift_time)

    # sort and prioritize the list again
    # exp_sequence = prioritize_sequence(exp_sequence)
    # print(exp_sequence)  # debug
    return exp_sequence


def shift_all_for_rinse(exp_sequence: List[ActionInfo], act_pos, shift_time):
    print("Shifting all rinse after step")
    this_indx = exp_sequence[act_pos].keeper
    print("Shifting all rinse for samples ", this_indx)
    for ix in range(act_pos, len(exp_sequence)):
        this_action = exp_sequence[ix]
        if this_action.keeper == this_indx and this_action.action == 'rinse':
            this_action.change_start(this_action.start + shift_time)
    return exp_sequence


def prioritize_sequence(in_seq: List[ActionInfo], sam_indx: Tuple[int]):
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
                if this_action.keeper == exp_sequence[ix].keeper and \
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


def find_when_action(action: str, exp_sequence: List[ActionInfo], act_pos: int):
    # find the index of the action 'action' that has the same
    # sample id as the action in act_pos

    # action = 'load', 'unload', etc...
    if act_pos >= len(exp_sequence):
        act_pos = len(exp_sequence) - 1  # position must be within sequence
    elif act_pos < 0:
        act_pos = 0  # position must be within sequence

    find_action = exp_sequence[act_pos]
    # print("This action is:", find_action) # debug
    find_action_id = find_action.keeper  # sample id of action # MODIFY: use sam Location instead?
    when_action = 0
    for ix in range(len(exp_sequence)):
        this_action = exp_sequence[ix]  # should be an alias, not a copy
        # MODIFY: use sam Location instead?
        if this_action.action == action and this_action.keeper == find_action_id:
            when_action = ix
            break  # action index found, break out of for loop
    # print("This action is correlated with:", exp_sequence[when_action])  # debug
    return when_action


def find_gap_array(exp_seq: List[ActionInfo], act_pos: int):
    # returns a list of gaps, in seconds, between the actions
    # in exp_sequence up to act_pos, including zeros

    # first, check that index is within range
    if act_pos >= len(exp_seq):
        act_pos = len(exp_seq) - 1  # position must be within sequence
    elif act_pos < 0:
        act_pos = 0  # position must be within sequence

    gap_list = []  # list of gaps, in seconds, between actions
    for subx in range(act_pos - 1):
        # checking actions from zero to act_pos
        sub_act = exp_seq[subx]  # first
        next_act = exp_seq[subx + 1]  # second
        next_gap = next_act.start - sub_act.end  # time between actions, in seconds
        gap_list.append(next_gap)  # list includes zeros

    # index for actions (eg __0___), gaps (eg 0 ), and gap-time (eg x0) shown below:
    # __0__  0  __1__  1  __2__  2  __3__  3  __4__  4  __5__  5  __6__  6  __7__ ...etc
    #       x0        x1        x2        x3        x4        x5        x6
    return gap_list  # list of gaps, in seconds, between actions up to (act_pos - 1)


def find_next_gap(exp_sequence: List[ActionInfo], from_pos: int, to_pos: int):
    # returns the index of the next available gap for action of interest @ to_pos
    # from the gap AFTER the load/unload action step (from_pos)

    # index for actions (eg __0___), gaps (eg 0 ), and gap-time (eg x0) shown below:
    # __0__  0  __1__  1  __2__  2  __3__  3  __4__  4  __5__  5  __6__  6  __7__ ...etc
    #       x0        x1        x2        x3        x4        x5        x6
    this_action = exp_sequence[to_pos]  # the action of interest, at index to_pos
    this_action_time = this_action.end - this_action.start  # time-length of action, in seconds
    gaps_before_this = find_gap_array(exp_sequence, to_pos)  # list of gaps, in seconds,
    # array labeled zero to (to_pos - 1)

    which_gap = to_pos - 1  # gap right before action of interest
    for ig in range(from_pos, len(gaps_before_this)):
        # indices of gaps from AFTER the from_pos action step to gap before (to_pos - 1) step
        gap = gaps_before_this[ig]  # gap-length in seconds
        if gap >= this_action_time:
            # if the gap-length is longer than the action time
            which_gap = ig  # select index for the gap, corresponding to step before
            break
    return which_gap  # returns index of first gap where action @ to_pos can fit, but after action @ from_pos


def shift_timestamp(in_seq: List[ActionInfo], sam_indx: Tuple[int]):
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
                    sam_id = this_action.keeper  # moving this_action
                    shift_time = old_action.end - this_action.start
                    # print("Load needs to shift for all loads/steps after sam_id: ",
                    #      sam_id, " by time: ", shift_time)  # debug
                    exp_sequence = shift_all_for_load(exp_sequence, sam_id, shift_time)
                    print("Now we have to take ix back to the beginning to resort!")
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
                if old_action.action == 'mix' or old_action.action == 'unload' or old_action.action == 'reload':
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
                    sam_id = this_action.keeper
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


def find_gaps_compress_actions(in_seq: List[ActionInfo]):
    # Find the gaps in exp_sequence and compress when there are gaps
    # moving all 'rinse' steps forward if they fit into gaps (no swapping)
    exp_sequence = deepcopy(in_seq)
    num_actions = len(exp_sequence)
    for ix in range(1, num_actions):
        # cycle through indices from 1 to (num_actions - 1)
        this_action = exp_sequence[ix]  # should be an alias, not a copy
        if this_action.action == 'rinse':
            # only shifting 'rinse' actions, NOT load/unload/reload/mix <- modified so mix not compressed
            old_action = exp_sequence[(ix - 1)]  # should be an alias, not a copy
            if this_action.start > old_action.end:
                # print("Changing action: ", this_action)  # debug
                this_action.change_start(old_action.end)  # moving action to start earlier
    return exp_sequence


def swap_into_gaps(in_seq: List[ActionInfo], sam_indx: Tuple[int]):
    # Check the gaps between two actions and see if
    # 'rinse' actions can be swapped into the gap
    exp_sequence = deepcopy(in_seq)  # not alias, deep copy
    iterations = 0  # tracking when to bail
    hall_pass = len(exp_sequence) * 5  # when to bail out of loop
    iter_ix = 2  # start at index 2 (third in sequence)
    while iter_ix < len(exp_sequence):
        # cycle through the action step in_seq
        iterations += 1  # track to bail
        if iterations > hall_pass:
            print("WARNING: something is wrong, bailing out of swap_into_gaps while loop.")
            break  # emergency break out of while loop
        # iterate over list of actions, starting with the second action
        if iter_ix < 2:
            iter_ix = 2  # if accidentally went back too far

        print("ix is now:", iter_ix)  # debug
        this_action = exp_sequence[iter_ix]  # should be an alias, not a copy
        which_gap = iter_ix - 1  # gap index prior to this action
        prev = which_gap  # previous action and gap

        # index for actions (eg __0___), gaps (eg 0 ), and gap-time (eg x0) shown below:
        # __0__  0  __1__  1  __2__  2  __3__  3  __4__  4  __5__  5  __6__  6  __7__ ...etc
        #       x0        x1        x2        x3        x4        x5        x6
        # if this_action.action == 'mix':
        #     # MODIFY: mix step should be spread out, not shuffled all together
        #     # 'mix' must come after 'load'
        #     when_loaded = find_when_action('load', exp_sequence, iter_ix)  # index when this sample was loaded
        #     which_gap = find_next_gap(exp_sequence, when_loaded, iter_ix)  # gap index for the first gap
        #     # between when sample was loaded and now, where action will fit
        if this_action.action == 'rinse':
            # 'rinse' must come after 'unload'
            when_unloaded = find_when_action('unload', exp_sequence, iter_ix)  # index when sample was unloaded
            which_gap = find_next_gap(exp_sequence, when_unloaded, iter_ix)  # gap index for the first gap
            # between when the sample was unloaded and now, where action will fit

        if which_gap < prev:
            # if the first gap where action fits comes before the gap prior to this step
            last_action = exp_sequence[which_gap]  # action before the gap
            print("ix:", iter_ix, "Moving ", this_action, " after ", last_action)  # debug
            this_action.change_start(last_action.end)
            exp_sequence = prioritize_sequence(exp_sequence, sam_indx)
            exp_sequence = find_gaps_compress_actions(exp_sequence)
            iter_ix = iter_ix - 2  # going back two, to check that this_action moved correctly
        else:
            iter_ix = iter_ix + 1  # iterating forward for 'load', 'reload' and 'unload'

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
    exp_sequence: List[ActionInfo] = []
    this_action: ActionInfo
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
    exp.num_actions = len(exp_sequence)

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
    exp_sequence = swap_into_gaps(exp_sequence, sam_indx_in_order)
    print(exp_sequence)
    print("=====================================================================================")
    print("Compressing sequence")
    exp_sequence = find_gaps_compress_actions(exp_sequence)
    print(exp_sequence)
    print("=====================================================================================")
    print("Triple checking sequence:")
    exp_sequence = shift_timestamp(exp_sequence, sam_indx_in_order)
    print(exp_sequence)

    return exp_sequence


# used in config_samples
# def give_incubation_order(incub_list: list[int, (int, int)]):
#     # sort a list of incubation values linked to sam_loc
#     # by the VALUES in the list, then return
#     # the index values of the sorted list
#     # eg: (5, 3, 2, 0, ...)
#     temp_list = []
#     ij: int
#     for ij in range(len(some_list)):
#         temp_list.append([some_list[ij], ij])
#     temp_list.sort()  # sort by the first in tuple
#     sort_index: List[int]
#     sort_index = []
#     for xit in temp_list:
#         sort_index.append(xit[1])  # collect second item in tuple
#     sorted_tuple: tuple[int]
#     sorted_tuple = tuple(sort_index)
#     print("Sorted list order is:", sorted_tuple)  # debug
#     return sorted_tuple


# recursive reverse the list function
def reverse_the_list(t):
    # condition checking
    if len(t) == 0:
        return t
    else:
        return (t[-1],) + reverse_the_list(t[:-1])


# keep external to classes
# def find_res_in_list(res_list: list[ResWellData], loc: (int, int)):
#     list_length = len(res_list)
#     for res_indx in range(list_length):
#         res_data = res_list[res_indx]
#         if res_data.loc == loc:
#             return res_indx
#     s_out = "Loc " + str(loc) + " is not in this list of reservoirs!"
#     raise ValueError(s_out)


# keep external to classes
# def find_sam_in_list(sam_list: list[SamWellData], loc: (int, int)):
#     list_length = len(sam_list)
#     for sam_indx in range(list_length):
#         sam_data = sam_list[sam_indx]
#         if sam_data.loc == loc:
#             return sam_indx
#     s_out = "Loc " + str(loc) + " is not in this list of samples!"
#     raise ValueError(s_out)


# Run first, after user_config_exp()
def calc_nums_exp(exp: ExperimentData):
    # Simplifies the data users are required to enter for each experiment.
    # Function derives totals from user_config_exp(). Checks that info entered correctly.
    # Calculates total number of important variables. Populates list of available pipette tips.
    # Collect info on res/sam racks into lists of RackData objects: res_plate_wells & sam_plate_wells
    # Then updates that data from info entered in user_config_exp():
    # (res) location, contents, starting/ending concentration/volume.
    # (sam) location, inoculation number,
    # inoculation locations (checked in function), & fractions (checks sum).
    # Identifies locations of solution wells and lists content types.
    # Determines if dilutions are needed. Prints info for user (debugging).

    print("===========================================================================================")  # debug
    # ToDo: Print info about slots in deck order: text picture?

    # calculate number of racks and plates
    exp.num_lg_tipracks = len(exp.slots_tiprack_lg)
    exp.num_sm_tipracks = len(exp.slots_tiprack_sm)
    exp.num_res_plates = len(exp.slots_res_racks)
    exp.num_sam_plates = len(exp.slots_sam_plates)

    # check that all slots are legal to the OT2Deck (1-11) and are not duplicated
    all_slots = list(exp.slots_res_racks) + list(exp.slots_sam_plates) + \
                list(exp.slots_tiprack_lg) + list(exp.slots_tiprack_sm)
    all_slots.sort()  # sort in ascending order
    for each in all_slots:
        if each < 1 or each > 11:
            # OT-2 Deck only supports slots 1-11
            s_out = "WARNING: Listed slot: " + str(each) \
                    + " is out of range! Check user_config_exp() and retry."
            print(s_out)
            raise StopExecution
        num_times = all_slots.count(each)
        if num_times > 1:
            # Slots can only be used once
            s_out = "WARNING: Listed slot: " + str(each) + " used more than once! Check user_config_exp and retry."
            print(s_out)
            raise StopExecution

    # check that user_config_exp was filled out correctly
    if len(exp.offsets_res_racks) != exp.num_res_plates \
            or len(exp.res_plate_names) != exp.num_res_plates:
        s_out = "Number of items in offsets_res_racks or num_res_plates is incorrect. Check user_config_exp and retry."
        print(s_out)
        raise StopExecution
    if len(exp.offsets_sam_racks) != exp.num_sam_plates \
            or len(exp.sam_plate_names) != exp.num_sam_plates:
        s_out = "Number of items in offsets_sam_racks or sam_plate_names is incorrect. Check user_config_exp and retry."
        print(s_out)
        raise StopExecution

    def find_slot_in_nestedlist_4rack(res_list, slot_number: int):
        # returns index of item for an item in a list such that:
        # res_list = ((slot_num1, some_val1,...),(slot_num2, some_val2,...),...)
        # where slot_num matches some_val in each sublist:
        for sub_list_indx in range(len(res_list)):
            sub_list = res_list[sub_list_indx]  # select sub_list
            if slot_number == sub_list[0]:  # if slot_num matches slot_number
                return sub_list_indx  # return index
        error_out = "Slot # " + str(slot_number) + " is not in the nested list."
        raise ValueError(error_out)  # if not found, return error

    def create_rack_list(slots, nested_offsets, nested_names):
        # collect data on res/sam racks into lists of RackData objects
        # make a list of RackData items for each res rack
        rack_list = []  # list of RackData objects
        for slot_number in slots:
            # go through list of reservoir rack slots from user_config_exp()
            new_item = RackData(slot_number, 'res')  # new rack data class instance for this slot
            index = find_slot_in_nestedlist_4rack(nested_offsets, slot_number)
            off_wrack = nested_offsets[index]  # select correct offset for this slot_num
            new_item.offset = (off_wrack[1], off_wrack[2], off_wrack[3])  # where zero is slot_num
            index = find_slot_in_nestedlist_4rack(nested_names, slot_number)
            plate_name = nested_names[index][1]  # select correct plate name for this slot_num
            new_item.name = plate_name  # assign plate name
            label = exp.find_label(plate_name)  # find the correct label for this rack
            res_vol = exp.find_max_res_vol(plate_name)  # find the correct max volume for wells in this rack
            new_item.label = str(label) + str(slot_number)  # assign label for this rack
            new_item.max_vol = res_vol  # assign max volume for wells in this rack
            # print(new_item)  # debug
            rack_list.append(new_item)  # append to list
        return rack_list

    # generate list of RackData objects for reservoir and sample racks
    exp.res_plate_wells = create_rack_list(exp.slots_res_racks, exp.offsets_res_racks, exp.res_plate_names)
    exp.sam_plate_wells = create_rack_list(exp.slots_sam_plates, exp.offsets_sam_racks, exp.sam_plate_names)

    # calculate totals for samples and reservoirs, as well as
    # total number of waste, rinse, and solution wells
    exp.tot_num_sams = len(exp.input_sam_data)
    exp.tot_num_res = len(exp.input_res_data)
    exp.tot_num_waste = len(exp.waste_res_locs)
    exp.tot_num_rinse = len(exp.rinse_res_locs)
    exp.tot_num_sol = exp.tot_num_res - exp.tot_num_waste - exp.tot_num_rinse

    # calculate number of tips available, print info
    if exp.pipettes_in_use == 'large' or exp.pipettes_in_use == 'both':
        num_tips = 0
        tip_locs = []
        for rk_indx in exp.tips_in_lg_racks:
            # ((slot_num, [list_of_well_indices]),...)
            slot_num = rk_indx[0]
            num_tips = num_tips + len(rk_indx[1])  # length of list
            for tip in rk_indx[1]:
                tip_loc = (slot_num, tip)
                tip_locs.append(tip_loc)
        exp.tot_num_lg_tips = num_tips  # total number of lg tips
        exp.available_tips_lg = tuple(tip_locs)  # convert list to immutable tuple
        exp.which_tip_lg = 0
        str_out = "Loading large pipette with " + str(exp.num_lg_tipracks) + \
                  " racks in slots: " + str(exp.slots_tiprack_lg) + \
                  " with total number of available tips:  " + str(exp.tot_num_lg_tips)
        print(str_out)
    else:
        str_out = "WARNING: without large pipette, cannot continue script."
        print(str_out)
        raise StopExecution
    if exp.pipettes_in_use == 'small' or exp.pipettes_in_use == 'both':
        num_tips = 0
        tip_locs = []
        for rk_indx in exp.tips_in_sm_racks:
            # ((slot_num, [list_of_well_indices]),...)
            slot_num = rk_indx[0]
            num_tips = num_tips + len(rk_indx[1])  # length of list
            for tip in rk_indx[1]:
                tip_loc = (slot_num, tip)  # location
                tip_locs.append(tip_loc)
        exp.tot_num_sm_tips = num_tips  # total number of sm tips
        exp.available_tips_sm = tuple(tip_locs)  # convert list to immutable tuple
        exp.which_tip_sm = 0
        str_out = "Loading small pipette with " + str(exp.num_sm_tipracks) + \
                  " racks in slots: " + str(exp.slots_tiprack_sm) + \
                  " with total number of available tips:  " + str(exp.tot_num_sm_tips)
        print(str_out)
    else:
        str_out = "WARNING: No small pipette loaded. (Not needed for this script.)"
        print(str_out)

    # from res_data input by user in user_config_exp(),
    # identify number of wells used in each rack,
    # list well-numbers, find locations for solution wells,
    # make a list of contents, determine if dilutions needed
    # each = ((Slot_#, Well_#, 'res'), (start_vol_uL, start_conc_uM),
    #           (end_vol_uL, end_conc_uM), & 'contents')
    # then modify: exp.res_plate_wells
    sol_res_locs = []  # solution reservoir locations
    start_conc = []  # list of starting concentrations
    end_conc = []  # list of ending concentrations
    content_types = []  # list of content types, excludes waste/rinse
    res_input = deepcopy(exp.input_res_data)  # res_data contents in user_config_exp()
    for in_res in res_input:
        # ((Slot_#, Well_#, 'res'), (start_vol_uL, start_conc_uM), (end_vol_uL, end_conc_uM), & 'contents')
        res_loc = in_res[0]  # (Slot_#, Well_#, 'res')
        starting = in_res[1]  # (start_vol_uL, start_conc_uM)
        ending = in_res[2]  # (end_vol_uL, end_conc_uM)
        content = in_res[3]  # res_data contents in user_config_exp()
        if res_loc[2] != 'res':
            print("Warning, first item of each res_data item must be (slot_num, well_id, 'res')")
        res_slot = res_loc[0]  # Slot_#
        res_well = res_loc[1]  # Well_#
        res_loc = (res_slot, res_well)  # (Slot_#, Well_#)
        indx = exp.find_rack_in_res_plates(res_slot)  # indx from list of RackData
        exp.res_plate_wells[indx].num_wells += 1  # update number of wells on this plate
        exp.res_plate_wells[indx].well_ids.append(res_well)  # update well index list
        if res_loc in exp.waste_res_locs or res_loc in exp.rinse_res_locs:
            pass  # skip the waste/rinse reservoirs
        else:
            sol_res_locs.append(res_loc)  # list of solution locations
            start_conc.append(starting[1])  # start concentration (not sorted)
            end_conc.append(ending[1])  # end concentration (not sorted)
            if content not in content_types:
                content_types.append(content)  # append to list of content types
    for rack in exp.res_plate_wells:
        rack.well_ids = tuple(rack.well_ids)  # convert list of well_ids in RackData object
    exp.sol_res_locs = tuple(sol_res_locs)  # immutable list of solution reservoir locations
    exp.content_types = tuple(content_types)  # immutable list of content types
    exp.num_cont_types = len(content_types)  # number of contents, excludes waste/rinse
    if start_conc != end_conc:
        print("Dilutions are needed.")  # debug
        exp.do_dilutions = True  # compare before/after lists (not sorted)

    # from sam_data, identify number of wells in each sam rack & list well-numbers
    # each = (sam_loc, (num_inoc,(inoc_sol_loc),...), (incub_info), 'sam_name')
    # then modify exp.sam_plate_wells RackData obj list
    sam_input = deepcopy(exp.input_sam_data)  # sam_data contents in user_config_exp()
    for in_sam in sam_input:
        # (sam_loc, (num_inoc,(inoc_sol_loc),...), (incub_info), 'sam_name')
        # sam_loc: (rack_num, well_id, 'sam')  # sample location - 'sam' should be last
        # inoc_sol_locs: (1, (rack_num, well_id, 'sol', vol_frac))  # inoculation solution loc
        # for multiple inoculation solutions, inoc_sol_loc = (num_sol, (sol_1),(sol_2),...)
        sam_loc = in_sam[0]  # sam_loc = (Slot_#, Well_#, 'sam')
        res_locs = in_sam[1]  # (num_sol, (rack_num, well_id, 'sol', vol_frac),...)
        if sam_loc[2] != 'sam':
            print("Warning, first item of each sam_data item must be (slot_num, well_id, 'sam')")
        sam_slot = sam_loc[0]  # rack slot num
        sam_well = sam_loc[1]  # well id
        sam_loc = (sam_slot, sam_well)  # location
        num_inoc_res = res_locs[0]  # number of inoculation solutions
        sum_frac = 0.0  # total fraction of all inoculation solutions
        # check inoculation reservoir locations
        for each_res in range(1, num_inoc_res + 1):
            res_loc = res_locs[each_res]  # for each inoc res loc listed in user_config_exp()
            res_slot = res_loc[0]  # Slot_#
            res_well = res_loc[1]  # Well_#
            sum_frac = sum_frac + res_loc[3]
            if res_loc[2] != 'sol':
                print("Warning, data in inoc_sol_locs of each sam_data item must be (slot_num, well_id, 'sol')")
            res_loc = (res_slot, res_well)  # reservoir location
            if res_loc not in exp.sol_res_locs:
                s_out = "Inoculation reservoir " + str(res_loc) \
                        + " for sample " + str(sam_loc) \
                        + " is invalid, check user_config_exp"
                print(s_out)
                raise StopExecution
        # check volume fraction
        if sum_frac > 1.0:
            s_out = "Volume fraction sum for sample " + str(sam_loc) \
                    + "exceeds 1.0, check user_config_exp"
            print(s_out)
            raise StopExecution
        indx = exp.find_rack_in_sam_plates(sam_slot)  # indx from list of RackData
        exp.sam_plate_wells[indx].num_wells += 1  # update number of wells on this plate
        exp.sam_plate_wells[indx].well_ids.append(sam_well)  # update well index list
    for rack in exp.sam_plate_wells:
        rack.well_ids = tuple(rack.well_ids)  # convert list of well_ids in RackData object

    # select starting wells
    waste_loc = exp.give_waste_loc()
    rinse_loc = exp.give_rinse_loc()

    # print summary for user/debugging:
    # print info about reservoirs
    for plate in exp.res_plate_wells:
        str_out = "Reservoir plate on slot " + str(plate.slot) + " labeled " + str(plate.label) \
                  + " has " + str(plate.num_wells) + " occupied wells with max vol " \
                  + str(plate.max_vol) + " uL "  # debug
        print(str_out)
    # print info about samples
    for plate in exp.sam_plate_wells:
        str_out = "Sample plate on slot " + str(plate.slot) + " labeled " \
                  + str(plate.label) + " with " + str(plate.num_wells) + " samples."  # debug
        print(str_out)
    print("===========================================================================================")  # debug
    return exp


# Run second, after calc_nums_exp()
def set_up_res_sam_data(exp: ExperimentData):
    # Populates the experiment's all_res_data and all_samples,
    # which are modifiable, nested list of ResWellData data objects.
    # Assigns locations, contents, starting & goal concentrations & volumes.
    # Assigns inoculation and incubation information to sample data.
    # Assigns tip locations to each reservoir and sample well.
    # Determines if dilution needed for each res. Finds max incubation time.
    # Determines the inoculation order for each sample based on incubation time

    which_pipette = 'large'  # using large pipette
    # select the first tip for the plan:
    next_tip_loc = exp.available_tips_lg[exp.which_tip_lg]  # first tip
    print("Selecting first ", which_pipette, " tip in loc: ", next_tip_loc)  # debug

    def find_loc_in_nested_well_list(well_list, find_loc: (int, int)):
        # returns index of item for an item in a list such that:
        # each_sublist = (rack_num, well_id, 'sam'), other attributes...
        # where find_loc matches (rack_num, well_id) in each_sublist
        for sub_list_indx in range(len(well_list)):
            sub_list = well_list[sub_list_indx]
            loc = sub_list[0]  # (Slot_#, Well_#, 'type')
            loc = (loc[0], loc[1])
            if find_loc == loc:
                return sub_list_indx
        s_out = "Location # " + str(find_loc) + " is not in nested list of reservoirs."
        raise ValueError(s_out)

    def list_actions_each_sam(sam: SamWellData):
        # todo: add res data so list_action can update dig_vol
        sam_loc = sam.loc  # (plate_indx, well_indx)
        sam_vol = sam.max_vol  # total volume in sample "well"
        sam_ord = sam.sam_inoculation_timing  # for sorting when consolidating actions
        sam_tip = sam.assigned_tip  # will be updated when complex step is split up
        incub_time = sam.targ_incub_time_s  # target incubation time in seconds
        num_inoc = sam.num_inoc_sol  # number of solutions used to inoculate sample
        num_reload = sam.targ_num_reload  # target number of reloads (based on incubation time)
        num_mix = sam.targ_num_mixes  # target number of mixes, spread evenly, from user_config_exp
        num_rinse = sam.targ_num_rinses  # target number of mixes, from user_config_exp
        is_complex = True  # all actions [load, reload, only_mix, unload, rinse]
        three_mixes = 0  # number of mixes for transfer action
        sam_sequence: List[ActionInfo] = []  # planned sequence of complex steps for this sample
        time_stmp = 0  # start time is zero for each sample, changed when consolidating actions

        # TODO: start here on 5/4/23

        # start by loading/inoculating the sample
        act_type = 'load'
        is_complex = True  # load is a complex action with 2 steps, 2 tips
        num_mixes = 3  # number of mixes AFTER load action, with sample tip
        for i in range(num_inoc):
            inoc_loc = sam.inoc_locs[i]
            inoc_frac = sam.inoc_fracs[i]
            inoc_vol = max_vol * inoc_frac
            new_action = ActionInfo(sam_loc, act_type, act_type,
                                    sam_ord, time_stmp,
                                    inoc_loc, sam_loc, inoc_vol,
                                    sam_tip, num_mixes, is_complex)
            time_stmp = new_action.end  # update the next start time
            sam_sequence.append(new_action)  # add to list of actions
        #
        # this_action.change_tip(sample.assigned_tip)  # load tip
        # # current_tip_loc = exp.find_next_tip(current_tip_loc)  # find next tip loc
        # sam_sequence.append(this_action)

        gap_time = math.ceil(incub_time / (num_mix + 1))
        for i in range(num_mix):
            time_stmp = time_stmp + gap_time
            this_action = ActionInfo(this_sam_indx, 'mix', time_stmp)
            this_action.change_tip(sample.assigned_tip)  # load tip
            sam_sequence.append(this_action)
        # the timestamps for 'reload' will be resorted at the end
        for i in range(sample.targ_num_reload):
            gap_time = (i + 1) * max_time_before_evap_m * 60
            this_action = ActionInfo(this_sam_indx, 'reload', gap_time)
            this_action.change_tip(sample.assigned_tip)  # load tip
            sam_sequence.append(this_action)
        time_stmp = targ_inc_time_s
        this_action = ActionInfo(this_sam_indx, 'unload', time_stmp)
        # sample.which_tip_loc = exp.find_next_tip(sample.which_tip_loc)  # find next tip loc
        add_tip = tip_ids[-1] + 1  # pick from exp.tips_in_racks instead?
        tip_ids.append(add_tip)  # new tip to unload
        this_action.change_tip(add_tip)
        sam_sequence.append(this_action)
        add_tip = tip_ids[-1] + 1  # pick from exp.tips_in_racks
        tip_ids.append(add_tip)  # new tip to rinse, every rinse
        time_stmp = time_stmp + 3 * rinse_time_s + 60 * max_incub
        for i in range(sample.targ_num_rinses):
            this_action = ActionInfo(this_sam_indx, 'rinse', time_stmp)
            this_action.change_tip(add_tip)
            sam_sequence.append(this_action)
            time_stmp = time_stmp + 3 * rinse_time_s
        # this_action = ActClass(this_sam_indx, 'done', sam_timestamp)
        # exp_sequence.append(this_action)
        sample.targ_act_seq = sam_sequence
        # debug block...
        print("------------------------------------------------------------------------------------")  # debug
        f_out_string = "Sample name: " + str(sample.sample_name) + " indexed # " \
                       + str(this_sam_indx) + " on plate # " + str(plate_index) + " & well # " \
                       + str(this_well_on_plate) + " with sequence: "  # debug

        return sam

    # TODO: start here on 4/26/23
    # first, set up reservoir data: exp.all_res_data
    all_res_input = deepcopy(exp.input_res_data)  # sample data inputted in user_config_exp
    res_racks = deepcopy(exp.res_plate_wells)  # rack data generated by calc_nums_exp
    res_data_set = []  # list of ResWellData objects (nested by rack), includes waste and rinse
    for rack in res_racks:
        # print(rack)  # debug
        slot_num = rack.slot  # rack slot number from OT2 Deck
        num_wells = rack.num_wells  # number of wells on this rack
        wells_ids = rack.well_ids  # well ids on this rack
        max_vol = rack.max_vol  # max volume for wells in this rack
        rack_set = []  # list of ResWell data objects for this rack
        for well_indx in range(num_wells):
            well_id = wells_ids[well_indx]  # choose well from wells in this rack
            res_loc = (slot_num, well_id)  # location of this reservoir (rack,well)
            res_data_indx = find_loc_in_nested_well_list(all_res_input, res_loc)
            # ((Slot_#, Well_#, 'res'), (start_vol_uL, start_conc_uM), (end_vol_uL, end_conc_uM), 'contents')
            in_res = all_res_input[res_data_indx]
            starting = in_res[1]  # (start_vol_uL, start_conc_uM)
            ending = in_res[2]  # (end_vol_uL, end_conc_uM)
            content = in_res[3]  # res_data contents
            new_res = ResWellData()  # new ResWell object
            new_res.slot_num = slot_num  # slot number for rack
            new_res.well_id = well_id  # well id on rack
            new_res.loc = res_loc  # (Slot_#, Well_#)
            new_res.max_vol = max_vol  # max volume for wells in this rack
            new_res.curr_vol = starting[0]  # start_vol_uL
            new_res.dig_vol = new_res.curr_vol  # start_vol_uL
            new_res.curr_conc = starting[1]  # start_conc_uM
            new_res.goal_vol = ending[0]  # goal end_vol_uL
            new_res.goal_conc = ending[1]  # goal end_conc_uM
            new_res.contents = content  # content
            if new_res.curr_conc < new_res.goal_conc:
                new_res.dilution_complete = False  # record if dilution to obtain res needed
            new_res.assigned_tip = next_tip_loc  # assign tip
            next_tip_loc = exp.find_next_tip(which_pipette)  # find next tip loc
            # print(new_res)  # debug
            rack_set.append(new_res)  # list of ResWell objects corresponding to this rack
        res_data_set.append(rack_set)  # nested list of ResWellData objects corresponding to all racks
    exp.all_res_data = res_data_set  # modifiable, nested list of ResWellData objects, grouped by racks

    # second, set up sample well data: exp.all_samples
    all_sam_input = deepcopy(exp.input_sam_data)  # sample data inputted in user_config_exp()
    sam_racks = deepcopy(exp.sam_plate_wells)  # sample rack data generated by calc_nums_exp(
    res_data = deepcopy(exp.all_res_data)  # not alias - needs to be copied back
    sam_data_set = []  # list of SamWellData objects (nested by rack)
    sam_incub_set = []  # list of [(sam_incub_time, (sam_loc)),...], not ordered or nested by rack
    max_incub_time = 0  # max incubation time for all samples
    for rack in sam_racks:
        print(rack)  # debug
        slot_num = rack.slot  # find the slot_num for this rack
        num_wells = rack.num_wells  # find number of wells in this rack
        wells_ids = rack.well_ids  # list of wells in this rack
        max_vol = rack.max_vol  # max vol for wells in this rack
        if exp.start_dry:
            start_vol = 0  # if samples start dry, volume zero
        else:
            start_vol = max_vol  # if samples start with solution in wells
        rack_set = []  # list of ResWell data objects for this rack
        for well_indx in range(num_wells):
            well_id = wells_ids[well_indx]  # choose well from wells in this rack
            sam_loc = (slot_num, well_id)  # location of this sample (rack,well)
            in_sam_data_indx = find_loc_in_nested_well_list(all_sam_input, sam_loc)  # find sample data by location
            in_sam = all_sam_input[in_sam_data_indx]  # select sample data (by location)
            # (sam_loc, (num_inoc,(inoc_sol_loc),...), (incub_info), 'sam_name')
            res_locs = in_sam[1]  # (num_sol, (rack_num, well_id, 'sol', vol_frac),...)
            incub_info = in_sam[2]  # (targ_incub_min,num_mixes,num_rinses)
            sam_name = in_sam[3]  # sample name
            new_sam = SamWellData()  # new data object for sample
            new_sam.sample_name = sam_name  # transfer sample name
            new_sam.slot_num = slot_num  # sample rack OT2 Deck slot
            new_sam.well_id = well_id  # well id on the rack, zero to num_wells-1
            new_sam.loc = sam_loc  # Sample location (rack_num, well_id)
            targ_incub_m = incub_info[0]  # targ_incub_min, integer preferred
            new_sam.targ_num_mixes = incub_info[1]  # target num_mixes
            new_sam.targ_num_rinses = incub_info[2]  # target num_rinses
            new_sam.targ_incub_time_m = targ_incub_m  # targ_incub_minutes, integer preferred
            new_sam.targ_incub_time_s = int(60 * targ_incub_m)  # targ_incub_seconds
            new_sam.max_vol = max_vol  # max volume for wells in sample rack
            new_sam.dig_vol = start_vol  # start volume for sample well
            new_sam.start_dry = exp.start_dry
            new_sam.store_dry = exp.store_dry
            num_inoc_res = res_locs[0]  # number of inoculation solutions
            new_sam.num_inoc_sol = num_inoc_res  # number of inoculation solutions
            inoc_locs = []  # inoculation locations for this sample
            inoc_fracs = []  # inoculation fractions for above locations
            inoc_contents = []  # inoculation solution contents
            inoc_concentration = []  # inoculation solution concentrations
            for each_res in range(1, num_inoc_res):
                res_loc = res_locs[each_res]  # (rack_num, well_id, 'sol', vol_frac)
                res_slot = res_loc[0]  # Slot_#
                res_well = res_loc[1]  # Well_#
                res_frac = res_loc[3]  # volume_fraction
                res_loc = (res_slot, res_well)  # (Slot_#, Well_#) # new variable
                res_indx = exp.find_res_in_nest_list(res_loc)  # find indices for inoc sol, in all_res_data
                inoc_res = res_data[res_indx[0]][res_indx[1]]  # select disputant ResWellData from all_res_data
                inoc_contents.append(inoc_res.contents)  # append to solution contents
                inoc_concentration.append(inoc_res.goal_conc)  # append to solution concentrations
                inoc_locs.append(res_loc)  # append to locations
                inoc_fracs.append(res_frac)  # append to fractions
            new_sam.inoc_locs = tuple(inoc_locs)  # inoculation locations for this sample
            new_sam.inoc_fracs = tuple(inoc_fracs)  # inoculation fractions for above locations
            new_sam.incub_sols = tuple(inoc_contents)  # inoculation solution set contents
            new_sam.incub_conc = tuple(inoc_concentration)  # inoculation solution concentrations
            new_sam.assigned_tip = next_tip_loc  # assign tip
            next_tip_loc = exp.find_next_tip(which_pipette)  # find next tip loc
            add_incub = (targ_incub_m, sam_loc)  # (incubation_time, (rack_num, well_id))
            # if target incubation time exceeds max time before evaporation
            if targ_incub_m > exp.max_time_before_evap_m:
                num_reload = int(targ_incub_m / exp.max_time_before_evap_m)  # num of reloads, min integer
                new_sam.targ_num_reload = num_reload  # num of times to reload
                new_sam.targ_num_mixes -= num_reload  # subtract from num mixes
            if new_sam.targ_num_mixes < 0:
                new_sam.targ_num_mixes = 0  # if num reload higher than num mixes
            if targ_incub_m > max_incub_time:
                max_incub_time = targ_incub_m  # find max incubation time
            sam_incub_set.append(add_incub)  # list of (incubation_time, (rack_num, well_id))
            exp_sequence: List[ActionInfo] = []
            sam_timestamp = 0

            rack_set.append(new_sam)  # list of ResWell objects corresponding to this rack
        sam_data_set.append(rack_set)  # nested list of SamWellData objects corresponding to all racks
    exp.all_samples = sam_data_set  # modifiable, nested list of SamWellData objects, grouped by racks

    # third, determine the inoculation order based on incubation time
    if exp.incub_longest_first:
        # each in set is (incubation_time, (rack_num, well_id))
        sam_incub_set.sort(reverse=True)  # order by incubation time, the longest first
    else:
        # each in set is (incubation_time, (rack_num, well_id))
        sam_incub_set.sort()  # order by incubation time, the shortest first
    incub_locs = []  # list of samples in order of inoculation
    for item in sam_incub_set:
        incub_locs.append(item[1])  # (0:incubation_time, 1:(rack_num, well_id))
    exp.incub_loc_order = tuple(incub_locs)  # make immutable
    exp.max_incub_m = max_incub_time  # record max incubation time

    # sort through inoculation order of the samples
    for inoc_index in range(len(exp.incub_loc_order)):
        # inoc_index from 0 to len - 1
        sam_loc = exp.incub_loc_order[inoc_index]  # sam loc
        # sam_indx = find_sam_in_nest_list(exp.all_samples, sam_loc)  # get sam index from loc
        sam_indx = exp.find_sam_in_nest_list(sam_loc)  # get sam index from loc
        sam_data = exp.all_samples[sam_indx[0]][sam_indx[1]]  # get sam data from index
        sam_data.sam_inoculation_timing = inoc_index  # modify inoculation timing to index
        # TODO: start here on 4/26/23

    # print debug info:
    # for rack in exp.all_res_data:
    #     for res in rack:
    #         print(res.loc, res.contents, res.assigned_tip)
    #
    # for rack in exp.all_samples:
    #     for sam in rack:
    #         print(sam.loc, sam.sample_name, sam.assigned_tip)

    return exp


# Run third, after set_up_res_sam_data()
def plan_dil_series(exp: ExperimentData):
    # plan dilution series:
    # double-checking that dilution is required
    if not exp.do_dilutions:
        print("Dilution already complete.")  # debug
        return exp

    def find_res_in_list(res_list: list[ResWellData], loc: (int, int)):
        list_length = len(res_list)
        for res_indx in range(list_length):
            res_data = res_list[res_indx]
            if res_data.loc == loc:
                return res_indx
        s_out = "Loc " + str(loc) + " is not in this list of reservoirs!"
        raise ValueError(s_out)

    def find_par_calc_tranf(dil_subset: list[ResWellData]):
        # Function finds dilution parent and transf_vol for list of ResWellData objects
        # with the same content. Warnings raised if errors encountered.
        # Returns sublist with goal-concentration in reverse order (highest to lowest)
        dil_subset.sort(key=lambda x: x.goal_conc, reverse=False)  # sort subset by goal_conc
        # sorted from the lowest goal concentration to the highest (reverse = False)
        # res_set_per_content.append(res_subset)
        content_type = dil_subset[0].contents  # type of contents
        num_subset = len(res_subset)  # length of the list
        # run check of subset and print info for user debugging
        print("--------------------------------------------------------------------------------")  # debug
        print("Identifying parent for each dilution child w content: ", content_type)  # debug
        print("id, loc,   cur_vol,  goal_vol,  goal_conc, curr_conc")  # debug
        for res_id in range(num_subset):
            child_res = res_subset[res_id]
            print(res_id, child_res.loc, "  ", child_res.curr_vol, "  ", child_res.goal_vol,
                  "  ", child_res.goal_conc, "  ", child_res.curr_conc)  # debug
            if child_res.contents != content_type:
                s_out = "Reservoir subset sent to find_par_calc_tranf is not all of content type: " + str(content_type)
                raise ValueError(s_out)  # double checks that the content is identical.

        # check each item on the list to see if dilution needed, recall that list is currently
        # sorted from the lowest goal concentration to the highest (reverse = False)
        # then find parent and transfer volume
        for res_id in range(num_subset):
            # for each in range(num_subset - 1):
            child_res = res_subset[res_id]
            print("____________________________________________________________________________")  # debug
            print("Evaluating", res_id, child_res.loc, "w goal concentration: ",
                  child_res.goal_conc, "uM and volume: ", child_res.goal_vol, "uL")  # debug
            if not child_res.dilution_complete:
                parent_id = res_id  # for first iteration, same as child
                parent_res = res_subset[parent_id]  # for 1st, same as child
                transf_vol = 1  # uL transferred from parent to res_child
                tran_mod = 1  # transf_vol mod 100 uL
                orig_vol = child_res.goal_vol  # w/o mods, goal volume for child_res
                loop_num = 0  # fail-safe to prevent infinite loop
                # when transfer volume is not divisible by 100 uL --> find parent, calc transf_vol
                while tran_mod > 0:
                    loop_num += 1  # fail-safe to prevent infinite loop
                    # recall res_subset sorted from the lowest goal concentration to highest
                    parent_id += 1  # selecting next higher concentration as parent
                    if loop_num > 1:  # skip first iteration
                        print("Pipette cannot transfer: ", tran_mod, "uL")  # debug
                    if parent_id >= num_subset or loop_num > 10:
                        # if parent id exceeds number of items in the subset
                        # or the number of times tried exceeds 10, warn user and quit
                        print("WARNING: Alternative dilution parent not available for reservoir in loc ", child_res.loc)
                        break
                    parent_res = res_subset[parent_id]  # selecting next higher concentration as parent
                    print("Choosing parent for dilution ", parent_id, parent_res.loc,
                          " with concentration ", parent_res.goal_conc, " uM")
                    # ratio of child to parent concentration
                    conc_ratio_chi_par = child_res.goal_conc / parent_res.goal_conc
                    transf_vol = int(conc_ratio_chi_par * child_res.goal_vol)  # calculate transfer volume
                    tran_mod = int(transf_vol % 100)  # round mod to 100 uL, due to 'large' pipette restrictions
                    # print("Transferring: ", transf_vol)  # debug
                    # if transfer volume is not divisible by 100 uL,
                    # and fewer than 3 loops have run for this child
                    if tran_mod > 0 and loop_num < 3:
                        transf_vol = int(transf_vol - tran_mod + 100)  # try increasing transf volume
                        new_goal_vol = int(transf_vol / conc_ratio_chi_par)  # calculate new goal volume for child
                        # and tran_mod % 10 == 0
                        # increase transfer volume,  mod to be divisible to 100
                        # print("Changing transf_vol to ", transf_vol,
                        #       ", with goal volume ", new_goal_vol) # debug
                        # if new goal volume is divisible by 100 uL,
                        # change the child reservoir goal vol and calculate again with the same parent
                        if new_goal_vol % 100 == 0:
                            child_res.goal_vol = new_goal_vol  # change goal vol
                            parent_id -= 1  # keep the same parent and try again
                            # if new goal is divisible by 100uL
                            # print("Changing", this_res.loc,
                            #       " with concentration: ", this_res.goal_conc,
                            #       " uM to goal volume ", this_res.goal_vol, "uL")  # debug
                        # else:
                        #     print("Volume incompatible with pipette")
                    # if transfer vol is not divisible
                    # and same parent tried more than 3 times
                    elif tran_mod > 0 and loop_num >= 3:
                        child_res.goal_vol = orig_vol  # returning to original volume if tried increasing thrice
                        # print("Changing", this_res.loc, " with concentration: ",
                        #       this_res.goal_conc, " uM BACK to goal volume ", this_res.goal_vol, "uL")  # debug
                # if parent id exists, increase parent's goal_volume by transfer amount
                if parent_id < (num_subset - 1):
                    parent_res.goal_vol = parent_res.goal_vol + transf_vol
                    if parent_res.goal_vol > parent_res.max_vol:
                        parent_res.goal_vol = parent_res.max_vol
                        print("WARNING: adjusting parent volume to max for loc ", parent_res.loc)
                    print("Changing parent", parent_res.loc, " volume to ", parent_res.goal_vol)  # debug
                child_res.parent_conc = parent_res.goal_conc  # update parent concentration for child reservoir
                child_res.parent_loc = parent_res.loc  # update parent location for child reservoir
                child_res.par_transf_vol = transf_vol  # update transfer volume for child reservoir
        res_subset.sort(key=lambda x: x.goal_conc, reverse=True)  # sort subset by goal_conc
        # sorted from the highest goal concentration to the lowest (reverse = True)
        # print info for user debugging
        print("____________________________________________________________________________")  # debug
        print("Updated subset of content type: ", content_type)  # debug
        print("id, loc, goal_conc, goal_vol, parent, p_conc, tf_vol")  # debug
        for res_id in range(num_subset):
            child_res = res_subset[res_id]
            print(res_id, child_res.loc, "  ", child_res.goal_conc, "  ", child_res.goal_vol,
                  "  ", child_res.parent_loc, "  ", child_res.parent_conc, "  ", child_res.par_transf_vol)  # debug
        print("--------------------------------------------------------------------------------")  # debug
        return dil_subset

    # res_data = exp.all_res_data  # alias - can be modified incorrectly
    # modifiable, nested list of ResWellData objects, grouped by racks
    res_data = deepcopy(exp.all_res_data)  # not alias - needs to be copied back
    num_types = exp.num_cont_types  # number of content types, excludes waste/rinse

    # first, sort reservoirs by content type and determine parent for dilution and transf volume
    res_set_per_content = []  # nested list of ResWellData objects, grouped by content type
    for type_indx in range(num_types):
        this_type = exp.content_types[type_indx]  # iterate through content types
        print("Content type: ", this_type)  # debug
        res_subset = []  # empty sublist of one content type
        for res_set_rack in res_data:
            for res in res_set_rack:
                check_contents = res.contents  # check contents
                if check_contents == this_type:
                    res_subset.append(res)  # add to res_subset
                    # print(check_contents, res.loc, res.dilution_complete)  # debug
        res_subset = find_par_calc_tranf(res_subset)  # finds dilution parents, transf_vol for this content list
        # returns sublist with goal-concentration in reverse order (highest to lowest)
        res_set_per_content.append(res_subset)

    # next, create action plan for dilutions
    res_timestamp = 0  # timestamps for use with actions
    zero_mixes = 0  # number of mixes for transfer action
    num_times_mixed = 10  # number of times a dilution is mixed
    action_set = []  # full action plan for dilutions
    dilution_num = 0  # dilution number, note-keeping for action infor
    for type_indx in range(num_types):
        this_type = exp.content_types[type_indx]  # select content type
        res_subset = res_set_per_content[type_indx]  # select res set for this content
        print("Content type: ", this_type)  # debug
        for res in res_subset:
            # recall list with goal-concentration (highest to lowest) order
            if not res.dilution_complete:
                # create actions for this reservoir, tracked by dilution_num:
                # (1) transf concentrated solution from parent to child
                # (2) transf DI dilution solution from parent to child
                # (3) mix solution in current reservoir
                print("Creating actions for: ", res.loc)  # debug
                sol_parent = res.parent_loc  # select concentrated parent
                par_indx = find_res_in_list(res_subset, sol_parent)  # index from subset list
                par_res = res_subset[par_indx]  # select reservoir
                par_tip = par_res.assigned_tip  # use tip of the 'from' reservoir
                sol_vol = res.par_transf_vol  # transfer volume from concentrated parent
                is_complex = False
                new_action = ActionInfo(res.loc, 'transf', 'dilution',
                                        dilution_num, res_timestamp,
                                        sol_parent, res.loc, sol_vol,
                                        par_tip, zero_mixes, is_complex)
                par_res.dig_vol = par_res.dig_vol - sol_vol  # update parent res volume
                res.dig_vol = res.dig_vol + sol_vol  # update current res volume
                res_timestamp = new_action.end  # update the next start time
                action_set.append(new_action)  # add to list of actions
                print(new_action)  # debug
                dil_vol = res.goal_vol - sol_vol  # calculate dilution volume
                dil_loc = exp.give_rinse_loc()  # location of dilution parent, updated internally based on dig_vol
                # dil_indx = find_res_in_nest_list(res_data, dil_loc)  # find indices for dilutant
                dil_indx = exp.find_res_in_nest_list(dil_loc)  # find indices for dilutant, in all_res_data
                dil_res = res_data[dil_indx[0]][dil_indx[1]]  # select disputant reservoir
                dil_tip = dil_res.assigned_tip  # tip of the dilution reservoir
                new_action = ActionInfo(res.loc, 'transf', 'dilution',
                                        dilution_num, res_timestamp,
                                        dil_loc, res.loc, dil_vol,
                                        dil_tip, zero_mixes, is_complex)
                res.dig_vol = res.dig_vol + dil_vol  # update current res volume
                dil_res.dig_vol = dil_res.dig_vol - dil_vol  # update disputant res volume
                res_timestamp = new_action.end  # update the next start time
                action_set.append(new_action)  # add to list of actions
                print(new_action)  # debug
                mix_vol = min(1000, int(0.5 * res.goal_vol))  # choose smaller volume
                res_tip = res.assigned_tip  # tip of the child reservoir
                new_action = ActionInfo(res.loc, 'mix', 'dilution',
                                        dilution_num, res_timestamp,
                                        res.loc, res.loc, mix_vol,
                                        res_tip, num_times_mixed, is_complex)
                res_timestamp = new_action.end  # update the next start time
                action_set.append(new_action)  # add to list of actions
                print(new_action)  # debug
                dilution_num += 1  # iterate dilution number

    exp.all_res_data = res_data  # copy res_data back to exp, since deepcopy was used
    exp.pln_dilut_seq = action_set  # copy dilution action set

    return exp


# used after user_config_exp
def config_samples(exp: ExperimentData):
    # This function checks the number of plates & samples in configuration
    # both modifies exp variables and replaces all_samples in exp
    # (list of lists of objects of class SampleWell !)
    # MODIFY description of components

    def spread_complex_action(all_acts: List[ActionInfo],
                              all_sam: List[List[SamWellData]],
                              all_res: List[List[ResWellData]]):
        action_set = []
        sam_timestamp = 0  # timestamps for use with actions
        # TODO: start here on 5/5/23
        for each_act in all_acts:
            if each_act.complex:
                # expand this action
                act_type = each_act.action
                if act_type == 'load':
                    print('decompressing action:', act_type)
                    sam_loc = each_act.keeper
                    res_loc = each_act.par_loc
                    is_complex = False
                    new_action = ActionInfo(sam_loc, 'transf', act_type, sam_timestamp,
                                            sol_parent, res.loc, sol_vol,
                                            par_tip, zero_mixes, is_complex)
                    new_action = ActionInfo(sam_loc, 'transf', act_type,
                                            dilution_num, sam_timestamp,
                                            sol_parent, res.loc, sol_vol,
                                            par_tip, zero_mixes, is_complex)
                elif act_type == 'reload':
                    print('decompressing action:', act_type)
                    sam_loc = each_act.keeper
                    res_loc = each_act.par_loc
                    waste_loc = each_act.targ_loc
                elif act_type == 'unload' or act_type == 'rinse':
                    print('decompressing action:', act_type)
                    sam_loc = each_act.keeper
                    res_loc = each_act.par_loc
                    waste_loc = each_act.targ_loc
                else:
                    print(act_type, "is not a complex action")
                    action_set.append(each_act)

            else:
                action_set.append(each_act)
        return action_set

    exp = calc_nums_exp(exp)  # first, calc the totals in exp, set up rack data, print rack info
    exp = set_up_res_sam_data(exp)  # second, set up data for reservoirs and sample wells,
    # also, obtain order of inoculation exp.incub_loc_order
    if exp.do_dilutions:
        print("Do dilutions")  # debug
        exp = plan_dil_series(exp)  # third, plan dilution series

    # TODO: start here on 5/4/23
    sam_data_set = deepcopy(exp.all_samples)
    incubation_order = exp.incub_loc_order

    # lastly, set up action list for samples
    # tip_ids = exp.tips_used  # list of tuples [(rack, well), ...]
    # sample_set = []  # all samples, NESTED list of lists of objects
    # num_plates = exp.num_sam_plates
    # plate_labels = ['label'] * num_plates
    # num_wells_all_plates = [0] * num_plates  # placeholder for list of num_wells in each plate
    # all_on_prev_plates = 0  # for sample_index from 0 to num_res-1
    # max_incub = max(exp.sam_targ_incub_times_min)  # find the maximum incubation time
    for plate_index in range(num_plates):
        # # for each plate in on the deck, indexed 0 to num_plates-1
        # plate_name = exp.sam_plate_names[plate_index]
        # plate_labels[plate_index] = exp.find_label(plate_name) + str(plate_index)
        # num_wells_on_this_plate = 0
        # for check_res in range(exp.num_samples):
        #     # not the fastest way, since we go through all samples num_plates times
        #     if exp.sam_plate_indx_nums[check_res] == plate_index:
        #         num_wells_on_this_plate = num_wells_on_this_plate + 1
        # # record the number of wells in each plate
        # num_wells_all_plates[plate_index] = num_wells_on_this_plate
        # print("===========================================================================================")  # debug
        # f_out_string = "Sample plate " + str(plate_index) \
        #                + " labeled " + plate_labels[plate_index] \
        #                + " with " + str(num_wells_on_this_plate) \
        #                + " samples."  # debug
        # print(f_out_string)  # debug
        # protocol.comment(f_out_string)  # debug
        plate_data_set = []  # each plate's list of SampleWell class
        for this_well_on_plate in range(num_wells_on_this_plate):
            # for each well on this plate
            # plate_data_set.append(SamWellData())  # appends object of SampleWell class to list
            # sample = plate_data_set[this_well_on_plate]
            # this_sam_indx = all_on_prev_plates + this_well_on_plate
            # sample.sam_indx = this_sam_indx  # not used anymore - replaced with indices (rack_id, well_id)
            # sample.sample_name = exp.sam_names[this_sam_indx]
            # sample.sam_inoculation_timing = exp.sam_timing[this_sam_indx]  # not used anymore replaced with zero to max from incub_loc_order
            # sample.plate_indx = exp.sam_plate_indx_nums[this_sam_indx]
            # sample.slot_num = exp.slots_sam_plates[plate_index]
            # sample.well_id = exp.sam_well_indx_nums[this_sam_indx]
            #
            # sample.targ_num_rinses = exp.sam_targ_num_rinses[this_sam_indx]
            # sample.targ_num_mixes = exp.sam_targ_num_mixes[this_sam_indx]
            # sample.targ_incub_time_m = exp.sam_targ_incub_times_min[this_sam_indx]
            # if sample.targ_incub_time_m > max_time_before_evap_m:
            #     num_reload = int(sample.targ_incub_time_m / max_time_before_evap_m)
            #     sample.targ_num_reload = num_reload
            #     sample.targ_num_mixes = sample.targ_num_mixes - num_reload
            #     if sample.targ_num_mixes < 0:
            #         sample.targ_num_mixes = 0

            # MODIFY THIS BEFORE RUNNING!
            # MODIFY to "locations" and fraction of total volume
            # sample.solution_location = exp.sam_inoculation_locations[this_sam_indx]  # using res_loc instead
            # MODIFY tp finding index with location data
            # parent_res = [res for res in res_subset if res.loc == this_res.parent_sol_loc]
            # sample.solution_index = 0  # res_locations.index(sample.solution_location)
            # this_res_data = exp.res_data[sample.solution_index]  # choose solution data_set (alias)
            # sample.assigned_tip = this_res_data.assigned_tip  # tuple (rack, well) - corresponds to res_well
            # sample.incub_solution = this_res_data.contents
            # sample.incub_concen = this_res_data.curr_conc

            # sample.targ_incub_gap_time_m = exp.sample_targ_incub_gap_times_min[start_sam]
            # sample.loc = (sample.plate_indx, sample.well_id)
            # targ_inc_time_s = math.ceil(60 * sample.targ_incub_time_m)
            # TODO: start here on 4/26/23
            # exp_sequence: List[ActionInfo] = []
            # sam_timestamp = 0
            # this_action = ActionInfo(this_sam_indx, 'load', sam_timestamp)
            # this_action.change_tip(sample.assigned_tip)  # load tip
            # # current_tip_loc = exp.find_next_tip(current_tip_loc)  # find next tip loc
            # exp_sequence.append(this_action)
            # gap_time = math.ceil(targ_inc_time_s / (sample.targ_num_mixes + 1))
            # for i in range(sample.targ_num_mixes):
            #     sam_timestamp = sam_timestamp + gap_time
            #     this_action = ActionInfo(this_sam_indx, 'mix', sam_timestamp)
            #     this_action.change_tip(sample.assigned_tip)  # load tip
            #     exp_sequence.append(this_action)
            # # the timestamps for 'reload' will be resorted at the end
            # for i in range(sample.targ_num_reload):
            #     gap_time = (i + 1) * max_time_before_evap_m * 60
            #     this_action = ActionInfo(this_sam_indx, 'reload', gap_time)
            #     this_action.change_tip(sample.assigned_tip)  # load tip
            #     exp_sequence.append(this_action)
            # sam_timestamp = targ_inc_time_s
            # this_action = ActionInfo(this_sam_indx, 'unload', sam_timestamp)
            # # sample.which_tip_loc = exp.find_next_tip(sample.which_tip_loc)  # find next tip loc
            # add_tip = tip_ids[-1] + 1  # pick from exp.tips_in_racks instead?
            # tip_ids.append(add_tip)  # new tip to unload
            # this_action.change_tip(add_tip)
            # exp_sequence.append(this_action)
            # add_tip = tip_ids[-1] + 1  # pick from exp.tips_in_racks
            # tip_ids.append(add_tip)  # new tip to rinse, every rinse
            # sam_timestamp = sam_timestamp + 3 * rinse_time_s + 60 * max_incub
            # for i in range(sample.targ_num_rinses):
            #     this_action = ActionInfo(this_sam_indx, 'rinse', sam_timestamp)
            #     this_action.change_tip(add_tip)
            #     exp_sequence.append(this_action)
            #     sam_timestamp = sam_timestamp + 3 * rinse_time_s
            # # this_action = ActClass(this_sam_indx, 'done', sam_timestamp)
            # # exp_sequence.append(this_action)
            # sample.targ_act_seq = exp_sequence
            # # debug block...
            # print("------------------------------------------------------------------------------------")  # debug
            # f_out_string = "Sample name: " + str(sample.sample_name) + " indexed # " \
            #                + str(this_sam_indx) + " on plate # " + str(plate_index) + " & well # " \
            #                + str(this_well_on_plate) + " with sequence: "  # debug
            # print(f_out_string)  # debug
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
    # Have user fill out the user_config_exp: five steps
    my_exp = ExperimentData()

    # First, select a name for the experiment and enter date, then
    # indicate if using both large or/and small pipette and then which
    # slots on the OT2 Deck will be used by the reservoir/sample/tip racks
    # ========================= OT-2 Opentrons Deck Slot numbers: ========================|
    # ___OT-2_Deck____  # locations (rack_slot_#, well_id)          |__eg__96-well_rack:__|
    # | 10 | 11 | Tr |  # load tips farther to the back of the deck |0:A1  8:A2 --> 88:A12|
    # |  7 |  8 |  9 |  # load res racks closer to the fan (left)   |  |    |         |   |
    # |  4 |  5 |  6 |  # load sam racks farther from  fan (right)  |  V    V         V   |
    # |  1 |  2 |  3 |  # run check_offsets to confirm xyz loc      |7:H1  15:H2--> 95:H12|
    # ====================================================================================|
    my_exp.exp_name = "TestingTimeManagement"  # update every run
    my_exp.exp_date = 20230103  # yyyymmdd # update every run
    my_exp.pipettes_in_use = 'large'  # 'large', 'small' or 'both'
    # Slot locations , leave blank if not using
    my_exp.slots_tiprack_sm = ()  # location of small tip racks
    my_exp.slots_tiprack_lg = (10, 11)  # locations of large tip racks (slots 1-11)
    my_exp.slots_res_racks = (1, 4, 5, 8, 7, 9)  # locs of reservoir rx (slots 1-11)
    my_exp.slots_sam_plates = (2, 3, 6)  # locations of sample racks (slots 1-11)

    # Second, indicate xyz calibration offset for each rack of labware on the deck,
    # to keep track of where each rack is placed, indicate slot_# as the first value
    # for offsets: (slot_#, x_val, y_val, z_val) w/ values in +/- #.# mm
    # confirm the offsets on the OT-2 app or with check_offsets() jupyter notebook
    # my_exp.offsets_sm_tiprx = ((10, -1.1, +1.5, -0.1),
    #                            (6,  -0.1, +0.8, +0.4),)  # SMALL pipette tips
    my_exp.offsets_lg_tiprx = ((10, -1.2, +1.5, +0.2),
                               (11, -1.1, +1.5, -0.1),)  # LARGE pipette tips
    my_exp.offsets_res_racks = ((1, -0.2, +2.1, -1.3),
                                (4, -1.0, -1.0, -0.8),
                                (5, -1.0, -1.0, -1.6),
                                (8, +0.0, -2.0, -1.7),
                                (7, -1.0, +1.0, -0.7),
                                (9, -1.0, +0.0, -1.1),)  # reservoir racks
    my_exp.offsets_sam_racks = ((2, +0.0, +2.3, +0.1),
                                (3, +0.1, +1.4, +0.2),
                                (6, +0.1, +1.0, +0.2),)  # sample racks

    # Third, indicate num of tips in each rack, res/sam plate names, and which are waste/rinse
    # For a full set of tips, use (slot_num, [*range(0, 96, 1)], modify '96' &
    # for incomplete set, w/o a full set of tips, innumerate list [0,1,...] or modify
    # range notation (eg missing 60-65) use (slot_num, [*range(0, 60, 1), *range(66, 96, 1)])
    my_exp.tips_in_sm_racks = []  # nested list, leave blank if not using
    my_exp.tips_in_lg_racks = [(10, [*range(0, 96, 1)]),
                               (11, [*range(0, 96, 1)]), ]  # nested list of tips in racks,
    # sample labware, eg:  'sam4_400uL' or 'sam8_400uL'
    my_exp.sam_plate_names = ((2, my_exp.sam4_400uL_name),
                              (3, my_exp.sam4_400uL_name),
                              (6, my_exp.sam4_400uL_name),)  # choose one for each slot
    # reservoir labware, eg:  'res3_60mL', 'res4_32mL', 'res6_20mL', or 'res6_40mL'
    my_exp.res_plate_names = ((1, my_exp.res3_60mL_name),
                              (4, my_exp.res3_60mL_name),
                              (5, my_exp.res4_32mL_name),
                              (8, my_exp.res4_32mL_name),
                              (7, my_exp.res6_20mL_name),
                              (9, my_exp.res6_20mL_name),)  # choose one for each slot
    # List the locations of waste reservoir wells and rinse/dilution reservoir wells:
    my_exp.waste_res_locs = ((1, 2), (4, 2),)  # location(s) (slot_num, well_indx) for 'Waste'
    my_exp.rinse_res_locs = ((1, 0), (4, 0),)  # location(s) (slot_num, well_indx) for 'DI_sol'
    # Start with samples wet (max_vol) or dry - w/o liquid
    my_exp.start_dry = False
    # Store samples wet (max_vol DI rinse) or dry - w/o liquid
    my_exp.store_dry = False
    # Indicate incubation order, with the longest incubation time first
    my_exp.incub_longest_first = True
    # Indicate how long before incubation liquid in
    # sample well will evaporate (and should be replaced)
    my_exp.max_time_before_evap_m = 60

    # Fourth, for each reservoir well, INCLUDING waste/rinse, indicate the following:
    # location, start/end volumes (1mL=1000uL) & concentrations (uM=umol/L)
    # ((Slot_#, Well_#, 'res'), (start_vol_uL, start_conc_uM), (end_vol_uL, end_conc_uM), 'contents')
    # eg: zero-index of 6 well goes 0:A1, 1:B1, 2:A2, 3:B2, 4:A3, 5:B3
    my_exp.input_res_data = (((1, 0, 'res'), (50000, 0), (0, 0), 'DI_sol'),
                             ((1, 1, 'res'), (50000, 2000), (0, 2000), 'Thiol_1_sol'),
                             ((1, 2, 'res'), (0, 0), (30000, 0), 'Waste'),
                             ((4, 0, 'res'), (50000, 0), (0, 0), 'DI_sol'),
                             ((4, 1, 'res'), (50000, 2000), (0, 2000), 'Thiol_2_sol'),
                             ((4, 2, 'res'), (0, 0), (30000, 0), 'Waste'),
                             ((5, 0, 'res'), (0, 0), (6000, 1600), 'Thiol_1_sol'),
                             ((5, 1, 'res'), (0, 0), (6000, 1000), 'Thiol_1_sol'),
                             ((5, 2, 'res'), (0, 0), (6000, 800), 'Thiol_1_sol'),
                             ((5, 3, 'res'), (0, 0), (6000, 600), 'Thiol_1_sol'),
                             ((8, 0, 'res'), (0, 0), (6000, 1600), 'Thiol_2_sol'),
                             ((8, 1, 'res'), (0, 0), (6000, 1000), 'Thiol_2_sol'),
                             ((8, 2, 'res'), (0, 0), (6000, 800), 'Thiol_2_sol'),
                             ((8, 3, 'res'), (0, 0), (6000, 600), 'Thiol_2_sol'),
                             ((7, 0, 'res'), (0, 0), (5000, 10), 'Thiol_1_sol'),
                             ((7, 1, 'res'), (0, 0), (5000, 50), 'Thiol_1_sol'),
                             ((7, 2, 'res'), (0, 0), (5000, 100), 'Thiol_1_sol'),
                             ((7, 3, 'res'), (0, 0), (5000, 200), 'Thiol_1_sol'),
                             ((7, 4, 'res'), (0, 0), (5000, 300), 'Thiol_1_sol'),
                             ((7, 5, 'res'), (0, 0), (5000, 400), 'Thiol_1_sol'),
                             ((9, 0, 'res'), (0, 0), (5000, 10), 'Thiol_2_sol'),
                             ((9, 1, 'res'), (0, 0), (5000, 50), 'Thiol_2_sol'),
                             ((9, 2, 'res'), (0, 0), (5000, 100), 'Thiol_2_sol'),
                             ((9, 3, 'res'), (0, 0), (5000, 200), 'Thiol_2_sol'),
                             ((9, 4, 'res'), (0, 0), (5000, 300), 'Thiol_2_sol'),
                             ((9, 5, 'res'), (0, 0), (5000, 400), 'Thiol_2_sol'),)

    # Fifth, for each sample on the sample plate, indicate:
    # (sam_loc, (num_inoc,(inoc_sol_loc)), (incub_time_min_int,num_mixes,num_rinses), 'sam_name')
    # sam_loc: (rack_num, well_id, 'sam')  # sample location - 'sam' should be last
    # inoc_sol_locs: (1, (rack_num, well_id, 'sol', vol_frac))  # inoculation solution loc
    # for multiple inoculation solutions, inoc_sol_loc = (num_sol, (sol_1),(sol_2),...)
    # incub_time_min_int: number of minutes to incubate solution on sample, integer preferred
    # num_mixes: num of times incubating solution is mixed during incubation
    # num_rinses: num of times solution is rinsed after incubation
    my_exp.input_sam_data = (((2, 0, 'sam'), (1, (5, 0, 'sol', 1.0),), (32, 3, 4), 'USC23Au0401a'),
                             ((2, 1, 'sam'), (1, (5, 1, 'sol', 1.0),), (24, 3, 4), 'USC23Au0401b'),
                             ((2, 2, 'sam'), (1, (5, 2, 'sol', 1.0),), (4, 3, 4), 'USC23Au0401c'),
                             ((2, 3, 'sam'), (1, (5, 3, 'sol', 1.0),), (20, 3, 4), 'USC23Au0401d'),
                             ((3, 0, 'sam'), (1, (8, 0, 'sol', 1.0),), (8, 3, 4), 'USC23Au0401e'),
                             ((3, 1, 'sam'), (1, (8, 1, 'sol', 1.0),), (12, 3, 4), 'USC23Au0401f'),
                             ((3, 2, 'sam'), (1, (8, 2, 'sol', 1.0),), (16, 3, 4), 'USC23Au0401g'),
                             ((3, 3, 'sam'), (1, (8, 3, 'sol', 1.0),), (28, 3, 4), 'USC23Au0401h'),)

    # Note: When using jupiter notebook, please check that custom
    # labware json files have been uploaded to /labware for use:
    # 'usctrayvials_3_reservoir_60000ul.json' with max_vol 60mL
    # 'usctrayvials_4_reservoir_32000ul.json' with max_vol 32mL
    # 'usctrayvials_6_reservoir_20000ul.json' with max_vol 20mL
    # 'usctrayvials_6_reservoir_40000ul.json' with max_vol 40mL
    # 'usctrayfoam_4_wellplate_400ul.json'    with max_vol .4mL
    # 'usctrayfoam_8_wellplate_400ul.json'    with max_vol .4mL
    # Additionally, check that the following hardware is in use:
    # large: 'p1000_single_gen2' on the 'left'
    # with tips: 'opentrons_96_tiprack_1000ul'
    # small: 'p20_single_gen2' on the 'right'
    # with tips: 'opentrons_96_tiprack_20ul'
    # if not, change ExperimentData class in file.

    return my_exp


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
            rack_offset = offsets[xx]
            new_rack = protocol.load_labware(this_name, this_slot, label=this_label)
            new_rack.set_offset(rack_offset[0], rack_offset[1], rack_offset[2])
            plates.append(new_rack)
            # each plate can be accessed using
            # this_plate =  sample_plates[i]  # NOT nested
        return plates

    def check_plate_offset(check_plate: Labware):
        first_col = check_plate.columns()[0]
        first_row = check_plate.rows()[0]
        num_cols = len(first_row)
        num_rows = len(first_col)
        last_row = check_plate.rows()[num_rows - 1]

        cornerTL = first_row[0]
        cornerTR = first_row[num_cols - 1]
        cornerBL = first_col[num_rows - 1]
        cornerBR = last_row[num_cols - 1]

        well = check_plate.wells()[0]
        pipette_lg.move_to(well.top())
        center_location = well.top()

        adjusted_location = center_location.move(types.Point(x=+1, y=+1, z=+1))
        pipette_lg.move_to(adjusted_location)
        return None

    def make_well_array(plates_labware: List[Labware],
                        loc_array: Tuple[Tuple[int, int]]):
        res_array = []  # array of wells (not nested)
        for xx in range(len(loc_array)):
            this_loc = loc_array[xx]  # tuple (rack,well)
            this_plate = this_loc[0]  # rack, indexed with 0,1,2, ...
            this_well = this_loc[1]  # well, indexed with 0,1,2, ..
            new_res = plates_labware[this_plate].wells()[this_well]
            res_array.append(new_res)  # individual wells (not nested array)
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
            this_offset = exp.offsets_lg_tiprx[xx]
            new_tiprack.set_offset(this_offset[0], this_offset[1], this_offset[2])
            tips_lg.append(new_tiprack)
        pipette_lg = protocol.load_instrument(exp.pipette_lg_name, exp.pipette_lg_loc, tip_racks=tips_lg)
    else:
        print("WARNING: without large pipette, cannot continue script. ")
        raise StopExecution

    tip_rack_lg = tips_lg[0]  # choose one of the racks
    reservoir_plates = load_plates(exp.slots_res_racks, exp.res_plate_names,
                                   exp.labels_res_plates, exp.offsets_res_racks)

    rinse_res_arr = make_well_array(reservoir_plates, exp.rinse_res_locs)  # array of rinse well objects (NOT nested)
    waste_res_arr = make_well_array(reservoir_plates, exp.waste_res_locs)  # array of waste well objects (NOT nested)
    sol_res_arr = make_well_array(reservoir_plates, exp.sol_res_locs)  # array of solution well objects (NOT nested)

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
                                exp.labels_sam_plates, exp.offsets_sam_racks)

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
        ## MODIFY: use subset of res_data?
        well_data = exp.rinse_data[exp._cur_rinse]
        if well_data.curr_vol < 1000:
            print("This rinse reservoir is empty. Switching to next.")
            exp._cur_rinse = exp._cur_rinse + 1  # increment to next rinse index
            # DOES THIS MODIFY THE GLOBAL VARIABLE?
            if exp._cur_rinse >= exp.max_num_rinse:
                print("We have run out of rinse solution!")
                print("Human input needed!")
                raise StopExecution
            # local variable, define in run() instead
            # rinse_well = rinse_res_arr[exp.this_indx_rinse]  # Labware object for protocol use
        return None

    def check_waste_full(well_data):
        if (well_data.curr_vol + 1000) > well_data.max_vol:
            print("This waste reservoir is full. Switching to next.")
            exp._cur_waste = exp._cur_waste + 1  # increment to next rinse index
            # DOES THIS MODIFY THE GLOBAL VARIABLE?
            if exp._cur_waste >= exp.max_num_waste:
                print("No space left for waste collection!")
                print("Human input needed!")
                raise StopExecution
            # local variable, define in run() instead
            # waste_well = waste_res_arr[exp.this_indx_waste]  # Labware object for protocol use
        return None

    # MODIFY: add selection for tip position, and which pipette
    def fill_mix_well(this_well, well_volume, this_res, res_data, this_waste, num_mix=1):
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

    def mix_well(this_well, this_waste, mix_volume, num_times):
        # uses the same tip, not keeping track of pipette tips
        f_out_string = "Mixing one well: " + str(this_well)  # debug
        protocol.comment(f_out_string)  # debug
        print(f_out_string)

        # MODIFY:  swap tips
        pipette_lg.mix(num_times, mix_volume, this_well)  # mixes solution in this well, num_times
        pipette_lg.blow_out(location=this_well.top())  # return extra liquid

        # clean up
        pipette_lg.move_to(this_waste.top())  # after mixing, move pipette to the top of waste
        pipette_lg.blow_out(location=this_waste.top())  # remove extra liquid
        pipette_lg.touch_tip(this_waste)  # remove drops that may hang on pipette tip
        pipette_lg.move_to(this_waste.top())  # after shaking off drops, move pipette to the top of waste

        timestamp_now = math.ceil(time.perf_counter())  # get timestamp of when mix occurred
        return timestamp_now

    def empty_well(this_well, this_waste, waste_data):
        # instead of using pipette.transfer(), aspirate and
        # dispense (halfway up) with touch_tip and blow_out at out_res
        # empty volume slightly larger than fill volume so that all liq. is evacuated
        f_out_string = "Emptying one well: " + str(this_well)  # debug
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
        fill_mix_well(this_well, well_volume, in_res, in_res_data, this_waste, 1)
        check_rinse_empty(in_res_data)  # checking rinse well volume

        timestamp_now = math.ceil(time.perf_counter())
        rinsing_time = math.ceil(timestamp_now - pre_rinse_time)
        output_string = "Rinsing time for sample well " + str(this_well) + " (sec): " + str(rinsing_time)  # debug
        protocol.comment(output_string)  # debug
        return timestamp_now

    def swap_tips(next_tip_loc: (int, int), which_pip: str):
        # swap tips for the pipette,
        # if using different tips for diff solutions
        rack_pos = next_tip_loc[0]
        which_tipwell = next_tip_loc[1]
        if which_pip == 'small':
            pipette = pipette_sm
            which_rack = tips_sm[rack_pos]
            max_racks = exp.num_sm_tipracks
            if which_tipwell not in exp.tips_in_sm_racks[rack_pos]:
                print("Not a valid position in this tiprack:", next_tip_loc)
                # raise StopExecution
        elif which_pip == 'large':
            pipette = pipette_lg
            which_rack = tips_lg[rack_pos]
            max_racks = exp.num_lg_tipracks
            if which_tipwell not in exp.tips_in_lg_racks[rack_pos]:
                print("Not a valid position in this tiprack:", next_tip_loc)
                # raise StopExecution
        else:
            print("Select pipette for swapping tips: 'small' or 'large'. Defaulting to 'large'")
            pipette = pipette_lg
            which_rack = tips_lg[rack_pos]
            max_racks = exp.num_lg_tipracks
            # raise StopExecution

        if rack_pos >= max_racks:
            print("Tiprack out of bounds!  Tiprack ", rack_pos, " is not loaded.")
            # raise StopExecution

        # LOCAL VARIABLE - how to change global variable instead?
        if pipette.has_tip:
            pipette.return_tip(home_after=True)  # return last tip to its rack  (not discarded)
        pipette.pick_up_tip(which_rack.wells()[which_tipwell])  # pick up selected tip from chosen rack
        pipette.home()  # homes pipette ONLY, NOT XYZ

        return None

    def run_sequence():
        # internal function, so dont need to pass exp, sample_plates, reservoirs, etc
        # run_sequence(exp: ExperimentData, plates_labware: List[Labware]):
        which_pip = exp.pipettes_in_use
        if which_pip == 'both':
            which_pip = 'large'
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
        which_tip = (0, 0)

        for ix in range(num_actions):
            # print("___________________________________________")
            # print("ix is now:", ix)  # debug
            this_action = exp_sequence[ix]
            sample_id = this_action.keeper

            sam_plate_id = exp.sam_plate_indx_nums[sample_id]
            sam_well_id = exp.sam_well_indx_nums[sample_id]
            sam_data = all_samples[sam_plate_id][sam_well_id]  # choose sample data_set (alias)
            this_well = sample_plates[sam_plate_id].wells()[sam_well_id]  # Labware well object for protocol use
            if which_tip != this_action.get_tip:
                which_tip = this_action.get_tip
                swap_tips(which_tip, which_pip)

            ## MODIFY: use subset of res_data?
            waste_data = exp.waste_data[exp._cur_waste]  # choose waste data_set (alias)
            rinse_data = exp.rinse_data[exp._cur_rinse]  # choose rinse data_set (alias)
            this_rinse = rinse_res_arr[exp._cur_rinse]  # Labware well object for protocol use
            this_waste = waste_res_arr[exp._cur_waste]  # Labware well object for protocol use

            goal_time = this_action.start - 10  # start action within 10 seconds of start/end time
            action_type = this_action.action
            timestamp_now = math.ceil(time.perf_counter())  # get timestamp
            # Should experiment pause?
            if timestamp_now < goal_time:
                gap_time = goal_time - timestamp_now
                print("Delay time should be:", gap_time)
                # MODIFY: pause notebook w/ time.sleep(), not robot
                # otherwise it's impossible to cancel
                # protocol.delay(seconds=gap_time)  # OT2-robot delay/sleep
                # robot stops listening to commands while in 'delay', no way to interrupt!
                print("Waiting ", gap_time, " seconds. To interrupt delay, press i,i.")
                time.sleep(gap_time)  # Sleep for 30 seconds
                # print("Done waiting ", gap_time, " seconds")
                # hold in place. pauses the notebook too.
                # print("Delay is done") # debug

            # Case: unload  (1)
            if action_type == 'unload':
                # empty and refill with rinse solution twice!
                print("Unloading sample #: ", sample_id)  # debug
                stamp = rinse_well(this_well, this_waste, waste_data, this_rinse, rinse_data)
                stamp = rinse_well(this_well, this_waste, waste_data, this_rinse, rinse_data)
                sam_data.incub_end_timestmp = stamp
            # Case: reload (2)
            elif action_type == 'reload':
                print("Reload sample #: ", sample_id)  # debug
                # MODIFY: choose/swap pipette tips
                exp.this_indx_solut = sam_data.solution_index  # change which solution is in use
                this_res_data = exp.input_res_data[exp.this_indx_solut]  # choose solution data_set (alias)
                this_solution = sol_res_arr[exp.this_indx_solut]  # Labware well object for protocol use
                stamp = fill_mix_well(this_well, well_volume, this_solution, this_res_data, this_waste, load_mixes)
                check_res_empty(this_res_data)  # checking res-well volume
                sam_data.incub_reload_timestmps.append(stamp)
            # Case: mix (3)
            elif action_type == 'mix':
                print("Mixing sample #: ", sample_id)  # debug
                # MODIFY: choose/swap pipette tips
                stamp = mix_well(this_well, this_waste, mix_volume, 1)
                sam_data.incub_mix_timestmps.append(stamp)
            # Case: load (4)
            elif action_type == 'load':
                print("Loading sample #: ", sample_id)  # debug
                # MODIFY: choose/swap pipette tips
                exp.this_indx_solut = sam_data.solution_index  # change which solution is in use
                this_res_data = exp.input_res_data[exp.this_indx_solut]  # choose solution data_set (alias)
                this_solution = sol_res_arr[exp.this_indx_solut]  # Labware well object for protocol use

                stamp = fill_mix_well(this_well, well_volume, this_solution, this_res_data, this_waste, load_mixes)
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
    protocol.set_rail_lights(True)  # turn on deck lights
    protocol.home()  # home all axis: XYZ & pipette
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
