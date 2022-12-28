# for details, https://docs.opentrons.com/v2/writing.html#simulate-block
# for details, https://docs.opentrons.com/v2/tutorial.html#tutorial

import json
from opentrons import protocol_api, types

RATE = 0.25  # % of default speeds

metadata = {
    'apiLevel': '2.13',
    'protocolName': 'OT2 Serial Dilution Tutorial',
    'description': '''Run by USC October 2022. This protocol is the
                   outcome of following the Python Protocol API Tutorial located at
                   https://docs.opentrons.com/v2/tutorial.html. It takes a
                   solution and progressively dilutes it by transferring it
                   stepwise across a 96-well plate.''',
    'author': 'Ulyana S. Cubeta',
    'rundate': '2022/10/28'
    }

def run(protocol: protocol_api.ProtocolContext):
    # load labware onto deck
    tipsLG = protocol.load_labware('opentrons_96_tiprack_1000ul', 11)
    tipsSM = protocol.load_labware('opentrons_96_tiprack_20ul', 10)
    # custom 3-well reservoir with 'A1', 'A2', 'A3' wells
    # reservoir = protocol.load_labware('usctrayvials_3_reservoir_60000ul', 1)
    reservoir = protocol.load_labware('nest_12_reservoir_15ml', 1)
    plate = protocol.load_labware('nest_96_wellplate_200ul_flat', 3)

    # load pipettes onto gantry
    pipetteLG = protocol.load_instrument(
        'p1000_single_gen2', 'left', tip_racks=[tipsLG])
    pipetteSM = protocol.load_instrument(
        'p20_single_gen2', 'right', tip_racks=[tipsSM])

    #pipetteLG.pick_up_tip()
    #pipetteSM.pick_up_tip()

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

    pipetteLG.transfer(120, reservoir['A1'], plate.wells())

    for i in range(8):
        # row (i) starts at row 0 to row 7 on a 96-well-plate
        # 12 columns numbered 0 to 11, ie  [0:12]
        volume = 10*(1+i)
        row = plate.rows()[i]
        pipetteSM.transfer(volume, reservoir['A2'], row[0], mix_after=(3, 20))
        pipetteSM.transfer(volume, row[:11], row[1:], mix_after=(3, 20))

    protocol.pause("If the serial dilution is complete, click 'resume.'")

    #pipetteLG.drop_tip()
    #pipetteSM.drop_tip()
