# for details, https://docs.opentrons.com/v2/new_protocol_api.html

import json
import time
import math
import datetime  # using time module
from threading import Thread
from typing import Tuple

from opentrons import protocol_api, types

RATE = 0.25  # % of default speeds (to reduce shaking of gantry)
well_volume = 400  # uL of solution in sample wells
mix_volume = int(0.75 * well_volume)
empty_volume = int(1.5 * well_volume)
start_volume = 50000  # uL of start solution (DI and thiol)
gap_times_s = [60, 120, 180, 300]  # gap times in seconds
targ_inc_times_min = 5  # target incubation time in minutes

metadata = {
    'apiLevel': '2.13',
    'protocolName': 'testing Time management during sample incubation',
    'description': '''Run by USC on Nov 10, 2022. ''',
    'author': 'Ulyana S. Cubeta',
    'rundate': '2022/11/02'
}


class SampleWell:
    def __init__(self):
        self.sample_name = "USC22Blnk1118a"
        self.well_plate = 1
        self.well_name = 'A1'
        self.targ_incub_time_m = 1
        self.targ_incub_gap_time_m = 1
        self.targ_num_rinses = 3
        self.rinsed_num = 0


def user_config_samples():
    num_samples = 4
    sample_names = ["USC22Blnk1118a", "USC22Blnk1118b", "USC22Blnk1118c", "USC22Blnk1118d"]
    sample_plates = [1, 1, 1, 1]
    sample_wells = ['A1', 'A2', 'A3', 'A4']
    sample_targ_incub_times_min = [6, 12, 24, 48]
    sample_targ_incub_gap_times_min = [3, 4, 5, 6]
    sample_targ_num_rinses = [9, 9, 9, 9]

    sample_set = []
    for i in range(num_samples):
        sample_set.append(SampleWell)
        sample = sample_set[i]
        sample.sample_name = sample_names[i]
        sample.well_plate = sample_plates[i]
        sample.well_name = sample_wells[i]
        sample.targ_num_rinses = sample_targ_num_rinses[i]
        sample.targ_incub_time_m = sample_targ_incub_times_min[i]
        sample.targ_incub_gap_time_m = sample_targ_incub_gap_times_min[i]

    return sample_set


def run(protocol: protocol_api.ProtocolContext):
    # load labware onto deck
    # find a way to "store" pipette tips in the tiprack to use for the same container
    tips = protocol.load_labware("opentrons_96_tiprack_1000ul", 5)
    pipette = protocol.load_instrument(
        "p1000_single_gen2", "left", tip_racks=[tips])

    # custom 3-well reservoir with 'A1', 'A2', 'A3' wells
    reservoirs = protocol.load_labware('usctrayvials_3_reservoir_60000ul', 1)
    res_volumes = [start_volume, start_volume, 0]
    rinse_res = reservoirs['A1']
    waste_res = reservoirs['A3']
    solution_res = reservoirs['A2']
    transfer_vol = 5000

    # custom 4-well well plate with 'A1', 'A2', 'A3', 'A4' wells
    #plate = protocol.load_labware('usctrayfoam_4_wellplate_400ul', 3)
    #all_samples = user_config_samples()

    def set_speeds(rate):
        protocol.max_speeds.update({
            'X': (400 * rate),
            'Y': (400 * rate),
            'Z': (100 * rate),
            'A': (100 * rate),
        })

        speed_max = max(protocol.max_speeds.values())

        for instr in protocol.loaded_instruments.values():
            instr.default_speed = speed_max

    set_speeds(RATE)
    protocol.set_rail_lights(False)
    pipette.home()

    # start experimental timer
    start_time = math.ceil(time.perf_counter())
    ct = datetime.datetime.now()
    out_string = "Starting Experimental Timer " + str(ct) + "Timestamp: " + str(start_time)  # debug
    protocol.comment(out_string)  # debug
    target_timestamp = start_time + targ_inc_times_min * 60

    has_tip_now = pipette.has_tip
    lights_on_now = protocol.rail_lights_on
    out_string = "Tip loaded: " + str(has_tip_now) + "Lights On: " + str(lights_on_now)
    protocol.comment(out_string)  # debug
    pipette.pick_up_tip()
    protocol.set_rail_lights(True)
    lights_on_now = protocol.rail_lights_on
    has_tip_now = pipette.has_tip
    out_string = "Tip loaded: " + str(has_tip_now) + "Lights On: " + str(lights_on_now)
    protocol.comment(out_string)  # debug

    pipette.transfer(transfer_vol, solution_res, waste_res, new_tip='never')
    pipette.move_to(rinse_res.top())
    protocol.set_rail_lights(False)  # debug
    protocol.comment("This comment comes after the transfer.")  # debug
    timestamp_now = math.ceil(time.perf_counter())
    if timestamp_now < target_timestamp:
        gap_time_sec = target_timestamp - timestamp_now
        out_string = "Pausing for " + str(gap_time_sec) + " seconds"  # debug
        protocol.comment(out_string)  # debug
        protocol.delay(seconds=gap_time_sec)
    time_lapse = math.ceil(time.perf_counter() - start_time)
    ct = datetime.datetime.now()
    out_string = "Ending Experimental Timer " + str(ct) + "TimeLapse (sec): " + str(time_lapse)  # debug
    protocol.comment(out_string)  # debug
    protocol.set_rail_lights(True)
    pipette.return_tip()
    protocol.comment("Completed Experiment.")  # debug
