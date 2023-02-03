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
# from opentrons import protocol_api, types
# protocol = opentrons.execute.get_protocol_api('2.13')
# protocol.set_rail_lights(True)  # check! only works if a single connection to robot
# Remember to run the following line at the end of experiment:
# os.system("systemctl start opentrons-robot-server")  # restart connection to app
# To close experiment, save notebook, then Shutdown kernel and Quit

from opentrons import protocol_api

metadata = {
    'apiLevel': '2.13',
    'protocolName': 'Testing tip Switching',
    'description': '''Run by USC on Feb 02, 2023. ''',
    'author': 'Ulyana S. Cubeta',
    'rundate': '2023/02/02'
}

class ExperimentData:
    # need a way to save this metadata (MODIFY!!)
    def __init__(self):
        self.tip_rack_lg_name = 'opentrons_96_tiprack_1000ul'
        self.tip_rack_sm_name = 'opentrons_96_tiprack_20ul'
        self.pipette_lg_name = 'p1000_single_gen2'
        self.pipette_sm_name = 'p20_single_gen2'
        self.pipette_lg_loc = 'left'
        self.pipette_sm_loc = 'right'

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


def run(protocol: protocol_api.ProtocolContext):
    exp = ExperimentData()
    exp.slots_tiprack_sm = (9, 6)  # slots 1-11 on OT-2
    exp.slots_tiprack_lg = (7, 8)  # slots 1-11 on OT-2

    exp.slots_sam_plates = (2, 3)  # slots 1-11 on OT-2
    exp.slots_reservoirs = (1, 4, 5)  # slots 1-11 on OT-2
    exp.res_plate_names = (exp.res4_32mL_name,
                           exp.res6_40mL_name,
                           exp.res6_40mL_name)  # choose one for each slot
    exp.num_lg_tipracks = len(exp.slots_tiprack_lg)
    exp.num_sm_tipracks = len(exp.slots_tiprack_sm)
    exp.tips_in_lg_racks = [[*range(0, 96, 1)], [*range(0, 96, 1)]]  # nested list,
    exp.tips_in_sm_racks = [[*range(0, 96, 1)], [*range(0, 96, 1)]]  # nested list,
    # number of tips in each rack, where 0 is A1, 1 is B1....8 is A2,...etc to 85 for a full set of tips!
    # list of sam_plates corresponds to index of plates 0,1,2,... (num_plates-1)

    exp.slot_offsets_sm_tips = ((-0.30, 0.80, 0.30), (-0.30, 0.80, 0.30))  # calibration offset for tips
    exp.slot_offsets_lg_tips = ((-1.20, 1.70, -0.20), (-1.10, 1.50, -0.10))  # calibration offset for tips

    tips_sm = []  # set of tip racks for this pipette size
    rack_slot = exp.slots_tiprack_sm[0]
    this_offset = exp.slot_offsets_sm_tips[0]
    new_tiprack = protocol.load_labware(exp.tip_rack_sm_name, rack_slot)
    new_tiprack.set_offset(this_offset[0], this_offset[1], this_offset[2])
    tips_sm.append(new_tiprack)
    rack_slot = exp.slots_tiprack_sm[1]
    this_offset = exp.slot_offsets_sm_tips[1]
    new_tiprack = protocol.load_labware(exp.tip_rack_sm_name, rack_slot)
    new_tiprack.set_offset(this_offset[0], this_offset[1], this_offset[2])
    tips_sm.append(new_tiprack)
    pipette_sm = protocol.load_instrument(exp.pipette_sm_name, exp.pipette_sm_loc, tip_racks=tips_sm)

    tips_lg = []  # set of tip racks for this pipette size
    rack_slot = exp.slots_tiprack_lg[0]
    this_offset = exp.slot_offsets_lg_tips[0]
    new_tiprack = protocol.load_labware(exp.tip_rack_lg_name, rack_slot)
    new_tiprack.set_offset(this_offset[0], this_offset[1], this_offset[2])
    tips_lg.append(new_tiprack)
    rack_slot = exp.slots_tiprack_lg[1]
    this_offset = exp.slot_offsets_lg_tips[1]
    new_tiprack = protocol.load_labware(exp.tip_rack_lg_name, rack_slot)
    new_tiprack.set_offset(this_offset[0], this_offset[1], this_offset[2])
    tips_lg.append(new_tiprack)
    pipette_lg = protocol.load_instrument(exp.pipette_lg_name, exp.pipette_lg_loc, tip_racks=tips_lg)

    sam_plates = []
    this_name = exp.sam4p_400uL_name
    this_slot = exp.slots_sam_plates[0]
    new_res = protocol.load_labware(this_name, this_slot)
    sam_plates.append(new_res)
    this_slot = exp.slots_sam_plates[1]
    new_res = protocol.load_labware(this_name, this_slot)
    sam_plates.append(new_res)

    res_plates = []
    this_name = exp.res_plate_names[0]
    this_slot = exp.slots_reservoirs[0]
    new_res = protocol.load_labware(this_name, this_slot)
    res_plates.append(new_res)
    this_name = exp.res_plate_names[1]
    this_slot = exp.slots_reservoirs[1]
    new_res = protocol.load_labware(this_name, this_slot)
    res_plates.append(new_res)
    this_name = exp.res_plate_names[2]
    this_slot = exp.slots_reservoirs[2]
    new_res = protocol.load_labware(this_name, this_slot)
    res_plates.append(new_res)

    protocol.home()

    pipette_lg.pick_up_tip()
    pipette_sm.pick_up_tip()

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

    set_speeds(0.25)

    pipette_lg.home()
    pipette_sm.home()

    for res_plate in res_plates:
        print("Testing reservoir plate:", res_plate)
        for well in res_plate.wells():
            pipette_lg.move_to(well.top())

    for sam_plate in sam_plates:
        print("Testing sample plate:", sam_plate)
        for well in sam_plate.wells():
            pipette_lg.move_to(well.top())

    def swap_tips(position: (int, int), which_pip: str):
        # swap tips for the pipette,
        # if using different tips for diff solutions
        rack_pos = position[0]
        which_tipwell = position[1]
        if which_pip == 'small':
            pipette = pipette_sm
            which_rack = tips_sm[rack_pos]
            max_racks = exp.num_sm_tipracks
            if which_tipwell not in exp.tips_in_sm_racks[rack_pos]:
                print("Not a valid position in this tiprack:", position)
                # raise StopExecution
        elif which_pip == 'large':
            pipette = pipette_lg
            which_rack = tips_lg[rack_pos]
            max_racks = exp.num_lg_tipracks
            if which_tipwell not in exp.tips_in_lg_racks[rack_pos]:
                print("Not a valid position in this tiprack:", position)
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

    swap_tips((1, 0), 'large')
    swap_tips((0, 7), 'large')
    swap_tips((0, 88), 'large')
    swap_tips((0, 95), 'large')
    swap_tips((0, 0), 'large')

    swap_tips((1, 0), 'small')
    swap_tips((0, 7), 'small')
    swap_tips((0, 89), 'small')
    swap_tips((0, 0), 'small')

    pipette_lg.return_tip()
    pipette_sm.return_tip()



