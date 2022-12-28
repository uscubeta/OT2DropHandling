# for details, https://docs.opentrons.com/v2/writing.html#simulate-block
# for details, https://docs.opentrons.com/v2/tutorial.html#tutorial

import json
import time
import math
import datetime  # using time module
from opentrons import protocol_api, types
#from threading import Thread

RATE = 0.25  # % of default speeds (to reduce shaking of gantry)
well_volume = 400  # uL of solution in sample wells
mix_volume = int(0.75 * well_volume)
empty_volume = int(1.5 * well_volume)
start_volume = 50000  # uL of start solution (DI and thiol)
gap_times_s = [60, 120, 180, 300]  # gap times in seconds
incubation_times_min = [2, 6, 12, 30]  # incubation time in minutes

metadata = {
    'apiLevel': '2.13',
    'protocolName': 'Thiol Funtionalization of Au Working Electrodes',
    'description': '''Run by USC on Nov 1, 2022. This protocol is a series of 
          thiol incubations of varying times, mixing regimes, chemistry, and concentration.  
          Test run will be on 4 samples with 1000uM MCH (mercaptohexanol).''',
    'author': 'Ulyana S. Cubeta',
    'rundate': '2022/11/02'
}


def run(protocol: protocol_api.ProtocolContext):
    # load labware onto deck
    # find a way to "store" pipette tips in the tiprack to use for the same container
    tips = protocol.load_labware("opentrons_96_tiprack_1000ul", 5)
    pipette = protocol.load_instrument(
        "p1000_single_gen2", "left", tip_racks=[tips])
    # custom 4-well wel plate with 'A1', 'A2', 'A3', 'A4' wells
    plate = protocol.load_labware('usctrayfoam_4_wellplate_400ul', 3)

    # custom 3-well reservoir with 'A1', 'A2', 'A3' wells
    # DONE/MODIFY: reduce resevoir depth - tip bumps into irregularities
    reservoirs = protocol.load_labware('usctrayvials_3_reservoir_60000ul', 1)
    res_volumes = [start_volume, start_volume, 0]
    rinse_res = reservoirs['A1']
    waste_res = reservoirs['A3']
    thiol_res = reservoirs['A2']
    well_name = "A1"
    well_list = ['A1', 'A2', 'A3', 'A4']

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

    def rinsing_well(sample_well, num_rinses):
        # MODIFY: may need to mix incubating samples
        # at the same time as rinsing?
        # not sure how to split the tasks (treading?)
        pre_rinse_time = math.ceil(time.perf_counter())
        pipette.mix(num_rinses, mix_volume, plate[sample_well])
        vol_transf = well_volume * 1
        for i in range(num_rinses):
            empty_well(plate[sample_well], waste_res, 2)

            pipette.transfer(well_volume, rinse_res, plate[sample_well], mix_after=(num_rinses, mix_volume),
                             new_tip='never')
            res_volumes[0] = res_volumes[0] - vol_transf  # well 'A1' rinse_res
        rinsing_time = math.ceil(time.perf_counter() - pre_rinse_time)
        output_string = "Rinsing time for sample well " + sample_well + " (sec): " + str(rinsing_time)  # debug
        protocol.comment(output_string)  # debug

    def mixing_wells(sam_well_list, num_mix):
        pre_mix_time = math.ceil(time.perf_counter())
        for this_well in sam_well_list:
            pipette.mix(num_mix, mix_volume, plate[this_well])
        mixing_time = math.ceil(time.perf_counter() - pre_mix_time)
        out_string = "Time to mix  liquid on the sample plate (sec): " + str(mixing_time)  # debug
        protocol.comment(out_string)  # debug

    def incubate_wells(sam_well_list, targ_incubation_min, gap_time_s):
        # mixing incubation liquid
        target_incubation_time_s = targ_incubation_min * 60
        output_string = "Target incubation time (min): " + str(targ_incubation_min)  # debug
        protocol.comment(output_string)  # debug
        incubation_time_s = math.ceil(time.perf_counter() - start_time)
        incubation_time_min = round((incubation_time_s / 60), 2)  # debug
        if incubation_time_s < target_incubation_time_s:
            added_time = target_incubation_time_s - incubation_time_s
            num_intervals = int(added_time / gap_time_s)
            output_string = "Incubating longer, num_intervals: " + str(num_intervals)  # debug
            protocol.comment(output_string)  # debug
            for i in range(num_intervals):
                # MODIFY: may need to mix other wells in between delays
                output_string = "Interval: " + str(i)  # debug
                protocol.comment(output_string)  # debug
                pipette.move_to(waste_res.top())
                protocol.delay(seconds=gap_time_s)
                mixing_wells(sam_well_list, 2)
            incubation_time_min = round(((time.perf_counter() - start_time) / 60), 2)  # debug
        output_string = "Incubation time for the sample wells (min): " + str(incubation_time_min)  # debug
        protocol.comment(output_string)  # debug

    def empty_wells(this_plate, out_res, out_num):
        # protocol.comment("Emptying wells in plate.")  # debug
        num_wells = len(this_plate.wells())
        for well in range(num_wells):
            empty_well(this_plate.wells()[well], out_res, out_num)

    def empty_well(this_well, out_res, out_num):
        # instead of using pipette.transfer(), aspirate and
        # dispense (halfway up) with touch_tip and blow_out at out_res
        # empty volume slightly larger than fill volume so that all liq. is evacuated
        output_string = "Emptying one well: " + this_well  # debug
        protocol.comment(output_string)  # debug
        pipette.aspirate(empty_volume, location=this_well.bottom(), rate=0.5)
        pipette.touch_tip(location=out_res)  # remove drops
        pipette.dispense(empty_volume, location=out_res.bottom(40.0), rate=2.0)
        pipette.blow_out(location=out_res.top())  # remove extra liquid
        pipette.touch_tip()  # remove drops
        res_volumes[out_num] = res_volumes[out_num] + empty_volume  # e.g. well 'A3' waste_res

    def fill_wells(fill_plate, this_res, res_num):
        protocol.comment("Filling wells.")  # debug
        vol_transf = well_volume * 4
        pipette.transfer(well_volume, this_res, fill_plate.wells(), new_tip='never')
        res_volumes[res_num] = res_volumes[res_num] - vol_transf

    def fill_mix_wells(fill_plate, mix_times, this_res, res_num):
        protocol.comment("Filling and mixing wells.")  # debug
        vol_transf = well_volume * 4
        pipette.transfer(well_volume, this_res, fill_plate.wells(), mix_after=(mix_times, mix_volume), new_tip='never')
        res_volumes[res_num] = res_volumes[res_num] - vol_transf

    set_speeds(RATE)
    pipette.home()
    pipette.pick_up_tip()

    # first, rinse sample surfaces with DI water
    protocol.comment("Starting DI Rinse of surfaces.")  # debug
    fill_wells(plate, rinse_res, 0)
    empty_wells(plate, waste_res, 2)

    # start experimental timer and "load" sample plate
    start_time = math.ceil(time.perf_counter())
    ct = datetime.datetime.now()
    out_string = "Starting Experimental Timer " + str(ct) + "Timestamp: " + str(start_time)  # debug
    protocol.comment(out_string)  # debug
    fill_mix_wells(plate, 3, thiol_res, 1)  # well 'A2' thiol_res
    load_time_min = round(((time.perf_counter() - start_time) / 60), 2)  # debug
    out_string = "Time to load sample plate (min): " + str(load_time_min)  # debug
    protocol.comment(out_string)  # debug

    # incubating/mixing liquid in 4 wells
    well_list = ['A1', 'A2', 'A3', 'A4']
    incubate_wells(well_list, incubation_times_min[0], gap_times_s[0])
    rinsing_well('A1', 9)  # rinsing well after incubation

    # incubating/mixing liquid in 3 wells
    well_list = ['A2', 'A3', 'A4']
    incubate_wells(well_list, incubation_times_min[1], gap_times_s[1])
    rinsing_well('A2', 9)  # rinsing well after incubation

    # incubating/mixing liquid in 2 wells
    well_list = ['A3', 'A4']
    incubate_wells(well_list, incubation_times_min[2], gap_times_s[2])
    rinsing_well('A3', 9)  # rinsing well after incubation

    # incubating/mixing liquid in 1 well
    well_list = ['A4']
    incubate_wells(well_list, incubation_times_min[3], gap_times_s[3])
    rinsing_well('A4', 9)  # rinsing well after incubation

    pipette.return_tip()
    protocol.comment("Completed Experiment.")  # debug
