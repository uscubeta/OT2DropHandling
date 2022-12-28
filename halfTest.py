# To Start, run Anaconda prompt, navigate with change directory ($:cd ...) and type in:
# $: conda env list # to check if the environment is available
# $: activate OT2DropHandling  # to activate the environment
# $: jupyter-lab
# this assumes that the environment was created using:
# $: conda create --name OT2DropHandling --file OT2DropHandlingEnv.txt
# AND that the following packages added via pip
# $ pip install -U pyvisa
# $ pip install opentrons
# you can check if the above two packages are installed with...
# $ pip list
# for details, https://docs.opentrons.com/v2/new_protocol_api.html
# robot IP address: http://169.254.59.185:48888

import json
import time
import math
import datetime  # using time module
from threading import Thread
from typing import Tuple
from opentrons import protocol_api, types

metadata = {
    'apiLevel': '2.13',
    'protocolName': 'testing Time management during sample incubation',
    'description': '''Run by USC on Nov 29, 2022. ''',
    'author': 'Ulyana S. Cubeta',
    'rundate': '2022/11/29'
}


class ExperimentData:
    def __init__(self):
        self.exp_rate_fraction = 0.25  # % of default speeds (to reduce shaking of gantry)
        self.experiment_name = "newExperiment"
        self.zero_timestmp = 1208125
        self.now_timestmp = 1208125
        self.num_samples = 1
        self.sample_names = ["sample1", ]
        self.sample_plate_nums = [1, ]
        self.sample_index_nums = [0, ]
        self.sample_well_names = ['A1', ]
        self.sample_targ_incub_times_min = [10, ]
        self.sample_targ_incub_gap_times_min = [5, ]
        self.sample_targ_num_rinses = [1, ]
        self.sample_well_vol = 400

        self.rinse_res_num = 0  # 'A1'
        self.waste_res_num = 2  # 'A3'
        self.sol_res_num = 1  # 'A2'
        self.res_volumes = [10, 10, 0]  # vol of rinse,sol,& waste

        self.num_plates = 1
        self.num_wells_in_plate = [2, ]
        self.slots_plates = [2, ]
        self.slot_tip_rack_lg = 11
        self.slot_tip_rack_sm = 10
        self.slot_reservoir = 1
        self.slot_res_offset = (0.0, 0.0, 0.0)  # calibration offset (x,y,z)
        self.slot_tips_offset = (0.0, 0.0, 0.0)  # calibration offset (x,y,z)
        self.slot_plate_offsets = [(0.0, 0.0, 0.0), ]  # calibration offset (x,y,z)
        self.reservoir_name = 'usctrayvials_3_reservoir_60000ul'
        self.plate_name = 'usctrayfoam_4_wellplate_400ul'
        self.tip_rack_lg_name = 'opentrons_96_tiprack_1000ul'
        self.tip_rack_sm_name = 'opentrons_96_tiprack_20ul'
        self.pipette_lg_name = 'p1000_single_gen2'
        self.pipette_sm_name = 'p20_single_gen2'
        self.pipette_lg_loc = 'left'
        self.pipette_sm_loc = 'right'
        self.load_time_s = 30  # estimated time for load function, seconds
        self.mix_time_s = 20  # estimated time for mix function, seconds
        self.rinse_time_s = 60  # estimated time for rinse function, seconds


class SampleWell:
    def __init__(self):
        self.sample_name = "USC22Blnk1118a"
        self.well_plate_slot = 3  # slots 1-11 on OT-2
        self.well_plate = 1  # plate1 to plate9 on OT-2
        self.well_index = 0  # indexing on the plate
        self.loc_indx = (0, 0)  # (well_plate, well_index)
        # tuple has to be replaced, not edited
        self.well_name = 'A1'  # name of the well from labware file
        self.targ_incub_time_m = 1  # target time for incubation
        self.targ_incub_gap_time_m = 1  # target time in between mixing
        self.targ_num_rinses = 3  # target num. of rinses after removing incub. liquid
        self.rinsed_num = 0  # how often has this sample been rinsed?
        self.incub_st_timestmp = 1208125  # start time collected @ time.perf_counter()
        self.incub_end_timestmp = 1208125  # end time collected @ time.perf_counter()
        self.incub_start_time_s = 10  # integer, in seconds
        self.incub_end_time_s = 25  # integer, in seconds
        self.incub_tot_time_s = 15  # integer, in seconds
        self.planned_sequence = [('load', 10), ('mix', 110), ('mix', 210), ('rinse', 310)]


def user_config_experiment():
    this_exp = ExperimentData()
    this_exp.experiment_name = "TestingTimeManagement"
    this_exp.slot_reservoir = 1  # slots 1-11 on OT-2
    this_exp.slot_res_offset = (-0.20, 2.10, -1.30)  # calibration offset
    this_exp.slot_tip_rack_lg = 4  # slots 1-11 on OT-2
    this_exp.slot_tips_offset = (-1.10, 1.50, -0.10)  # calibration offset
    this_exp.slots_plates = [2, 3, 5, 6]  # slots 1-11 on OT-2
    this_exp.slot_plate_offsets = [(0.00, 2.30, 0.10),
                                   (0.10, 1.40, 0.20),
                                   (-0.30, 1.60, 0.40),
                                   (-0.10, 0.80, 0.40), ]
    # list of slots corresponds to index of plates 0,1,2,...
    # the samples must be listed in order for the time being, adjust code if out of order
    this_exp.sample_plate_nums = [0, 0, 1, 1, 2, 2, 3, 3]  # zero-index of each plate, start at zero
    this_exp.sample_index_nums = [0, 1, 0, 1, 0, 1, 0, 1]  # zero-index of each well on plate, start at
    this_exp.sample_names = ["USC22Blnk1128a", "USC22Blnk1128b", "USC22Blnk1128c",
                             "USC22Blnk1128d", "USC22Blnk1128e", "USC22Blnk1128f",
                             "USC22Blnk1128g", "USC22Blnk1128h"]
    this_exp.sample_well_names = ['A1', 'A2', 'A3', 'A4', 'A5', 'A6', 'A7', 'A8']
    this_exp.sample_targ_incub_times_min = [2, 2, 2, 2, 2, 2, 2, 2]
    this_exp.sample_targ_incub_gap_times_min = [1, 1, 1, 1, 1, 1, 1, 1]
    this_exp.sample_targ_num_rinses = [3, 3, 3, 3, 3, 3, 3, 3, ]

    this_exp.rinse_res_num = 0  # 'A1'
    this_exp.sol_res_num = 1  # 'A2'
    this_exp.waste_res_num = 2  # 'A3'
    this_exp.res_volumes = [50000, 50000, 0]  # vol of rinse,sol,& waste, in uL (1 mL = 1000 uL)

    # custom 3-well reservoir with 'A1', 'A2', 'A3' wells
    this_exp.reservoir_name = 'usctrayvials_3_reservoir_60000ul'
    # custom 4-well well plate with 'A1', 'A2', 'A3', 'A4' wells
    this_exp.plate_name = 'usctrayfoam_4_wellplate_400ul'
    this_exp.tip_rack_name = 'opentrons_96_tiprack_1000ul'
    this_exp.pipette_lg_name = 'p1000_single_gen2'
    this_exp.pipette_lg_loc = 'left'  # location of lg pipette
    this_exp.sample_well_vol = 400  # uL, volume of each sample well

    this_exp.num_plates = len(this_exp.slots_plates)
    this_exp.num_samples = len(this_exp.sample_plate_nums)
    # the list of exp.num_wells_in_plate will be filled in config_samples
    return this_exp


def run(protocol: protocol_api.ProtocolContext):
    # define the protocol for the OT-2 to run
    def config_samples():
        # check the number of plates & samples in configuration
        exp.num_plates = len(exp.slots_plates)
        exp.num_samples = len(exp.sample_plate_nums)
        num_sam = exp.num_samples
        if len(exp.sample_names) != num_sam or len(exp.sample_well_names) != num_sam or len(
                exp.sample_targ_incub_times_min) != num_sam or len(
            exp.sample_targ_incub_gap_times_min) != num_sam or len(exp.sample_targ_num_rinses) != num_sam:
            # MODIFY: when moving this to jupyter notebook, can be more direct with output commands
            f_out_string = "Something is off. Please check user_config_experiment()"
            print(f_out_string)  # debug for notebook
            #protocol.pause(f_out_string)  # debug
        else:
            f_out_string = "The user_config_experiment() was written correctly. "
            print(f_out_string)  # debug for notebook
            #protocol.comment(f_out_string)  # debug

        sample_set = []
        num_plates = exp.num_plates
        exp.num_wells_in_plate = list(range(num_plates))  # placeholder for list of num_wells in each plate
        sample_indx = 0
        for plate_index in range(num_plates):
            plate_data_set = []
            num_sam_in_plate = 0

            for check_sam in range(num_sam):
                # not the fastest way, since we go through all samples multiple times
                if exp.sample_plate_nums[check_sam] == plate_index:
                    num_sam_in_plate = num_sam_in_plate + 1
            # record the number of wells in each plate
            exp.num_wells_in_plate[plate_index] = num_sam_in_plate
            f_out_string = "The number of samples in this plate " \
                           + str(plate_index) + " is " + str(num_sam_in_plate)  # debug
            print(f_out_string)  # debug
            protocol.comment(f_out_string)  # debug

            for this_well in range(num_sam_in_plate):
                start_sam = sample_indx + this_well
                plate_data_set.append(SampleWell())
                sample = plate_data_set[this_well]
                sample.sample_name = exp.sample_names[start_sam]
                sample.well_plate = exp.sample_plate_nums[start_sam]
                sample.well_plate_slot = exp.slots_plates[plate_index]
                sample.well_index = exp.sample_index_nums[start_sam]
                sample.well_name = exp.sample_well_names[start_sam]
                sample.targ_num_rinses = exp.sample_targ_num_rinses[start_sam]
                sample.targ_incub_time_m = exp.sample_targ_incub_times_min[start_sam]
                sample.targ_incub_gap_time_m = exp.sample_targ_incub_gap_times_min[start_sam]
                sample.loc_indx = (sample.well_plate, sample.well_index)
                # debug block...
                print("this_well: ", this_well, "; start_sam: ", start_sam)  # debug
                print("The sample name: ", sample.sample_name)  # debug
                print(vars(sample))  # debug
            sample_indx = sample_indx + num_sam_in_plate
            sample_set.append(plate_data_set)
        #
        return sample_set

    # set up experiment
    exp = user_config_experiment()
    all_samples = config_samples()

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
        plate_set.append(new_plate)
        # check if this works on the OT-2 - eg appending labware to list

    def set_speeds(rate_change):
        # needs to within run() to use protocol
        f_out_string = "Adjusting gantry speeds to " + str(rate_change) + "frac of max."
        protocol.comment(f_out_string)  # debug
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

    # target wells, check timer
    for p_indx in range(exp.num_plates):
        # iterate over all well-plates
        plate = plate_set[p_indx]
        num_wells = exp.num_wells_in_plate[p_indx]
        for w_indx in range(num_wells):
            # iterate over all wells in each plate
            well = plate.wells()[w_indx]
            this_sample = all_samples[p_indx][w_indx]
            pipette_lg.move_to(well.top())  # debug, check that each well registers.
            this_sample.incub_st_timestmp = math.ceil(time.perf_counter())
            this_sample.incub_start_time_s = this_sample.incub_st_timestmp - exp.zero_timestmp
    pipette_lg.move_to(rinse_res.top())
    exp.now_timestmp = math.ceil(time.perf_counter())
    load_time_min = round(((exp.now_timestmp - exp.zero_timestmp) / 60), 2)  # debug
    out_string = "Time to load sample plate (min): " + str(load_time_min)  # debug
    protocol.comment(out_string)  # debug

    # target wells, check timer
    for p_indx in range(exp.num_plates):
        # iterate over all well-plates
        plate = plate_set[p_indx]
        num_wells = exp.num_wells_in_plate[p_indx]
        for w_indx in range(num_wells):
            # iterate over all wells in each plate
            well = plate.wells()[w_indx]
            this_sample = all_samples[p_indx][w_indx]
            pipette_lg.move_to(well.top())  # debug, check that each well registers.
            this_sample.incub_end_timestmp = math.ceil(time.perf_counter())
            this_sample.incub_end_time_s = this_sample.incub_st_timestmp - exp.zero_timestmp
            this_sample.incub_tot_time_s = this_sample.incub_end_time_s - this_sample.incub_start_time_s

    time_lapse = math.ceil(time.perf_counter() - exp.zero_timestmp)
    ct = datetime.datetime.now()
    out_string = "Ending Experimental Timer " + str(ct) + "; TimeLapse (sec): " + str(time_lapse)  # debug
    protocol.comment(out_string)  # debug
    protocol.set_rail_lights(True)
    pipette_lg.return_tip()
    pipette_lg.drop_tip()
    pipette_lg.home()
    protocol.comment("Completed Experiment.")  # debug
