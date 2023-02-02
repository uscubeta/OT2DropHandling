from opentrons import protocol_api

metadata = {
    'apiLevel': '2.13',
    'protocolName': 'Just ejecting tips.',
    'description': '''Run by USC on Jan 30, 2023. Doesn't work if the OT2 doesn't know it has loaded tips, 
    if previous protocol was interrupted. ''',
    'author': 'Ulyana S. Cubeta',
    'rundate': '2023/01/30'
}


def run(protocol: protocol_api.ProtocolContext):
    # just loading tip racks and pipettes
    tips_sm = [protocol.load_labware('opentrons_96_tiprack_20ul', 9), ]
    pipette_sm = protocol.load_instrument('p20_single_gen2', 'right', tip_racks=tips_sm)
    tips_lg = [protocol.load_labware('opentrons_96_tiprack_1000ul', 7), ]
    pipette_lg = protocol.load_instrument('p1000_single_gen2', 'left', tip_racks=tips_lg)

    protocol.home()

    # pipette_lg.pick_up_tip()
    # pipette_sm.pick_up_tip()

    pipette_sm.drop_tip()
    pipette_lg.drop_tip()

    pipette_lg.home()
    pipette_sm.home()
