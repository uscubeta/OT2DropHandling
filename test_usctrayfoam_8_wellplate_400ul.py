import json
from opentrons import protocol_api, types

TEST_LABWARE_SLOT = '5'

RATE = 0.25  # % of default speeds

PIPETTE_MOUNT = 'left'
PIPETTE_NAME = 'p1000_single_gen2'

TIPRACK_SLOT = '11'
TIPRACK_LOADNAME = 'opentrons_96_tiprack_1000ul'
LABWARE_DEF_JSON = """{"ordering":[["A1","B1"],["A2","B2"],["A3","B3"],["A4","B4"]],"brand":{"brand":"USCtrayfoam","brandId":["FourSlideTrayFoam"]},"metadata":{"displayName":"USCtrayfoam 8 Well Plate 400 µL","displayCategory":"wellPlate","displayVolumeUnits":"µL","tags":[]},"dimensions":{"xDimension":127,"yDimension":85,"zDimension":19},"wells":{"A1":{"depth":1.2,"totalLiquidVolume":400,"shape":"circular","diameter":12.5,"x":18,"y":55,"z":17.8},"B1":{"depth":1.2,"totalLiquidVolume":400,"shape":"circular","diameter":12.5,"x":18,"y":30,"z":17.8},"A2":{"depth":1.2,"totalLiquidVolume":400,"shape":"circular","diameter":12.5,"x":48.4,"y":55,"z":17.8},"B2":{"depth":1.2,"totalLiquidVolume":400,"shape":"circular","diameter":12.5,"x":48.4,"y":30,"z":17.8},"A3":{"depth":1.2,"totalLiquidVolume":400,"shape":"circular","diameter":12.5,"x":78.8,"y":55,"z":17.8},"B3":{"depth":1.2,"totalLiquidVolume":400,"shape":"circular","diameter":12.5,"x":78.8,"y":30,"z":17.8},"A4":{"depth":1.2,"totalLiquidVolume":400,"shape":"circular","diameter":12.5,"x":109.2,"y":55,"z":17.8},"B4":{"depth":1.2,"totalLiquidVolume":400,"shape":"circular","diameter":12.5,"x":109.2,"y":30,"z":17.8}},"groups":[{"metadata":{"wellBottomShape":"flat"},"wells":["A1","B1","A2","B2","A3","B3","A4","B4"]}],"parameters":{"format":"irregular","quirks":[],"isTiprack":false,"isMagneticModuleCompatible":false,"loadName":"usctrayfoam_8_wellplate_400ul"},"namespace":"custom_beta","version":1,"schemaVersion":2,"cornerOffsetFromSlot":{"x":0,"y":0,"z":0}}"""
LABWARE_DEF = json.loads(LABWARE_DEF_JSON)
LABWARE_LABEL = LABWARE_DEF.get('metadata', {}).get(
    'displayName', 'test labware')
LABWARE_DIMENSIONS = LABWARE_DEF.get('wells', {}).get('A1', {}).get('yDimension')

metadata = {'apiLevel': '2.0'}


def run(protocol: protocol_api.ProtocolContext):
    tiprack = protocol.load_labware(TIPRACK_LOADNAME, TIPRACK_SLOT)
    pipette = protocol.load_instrument(
        PIPETTE_NAME, PIPETTE_MOUNT, tip_racks=[tiprack])

    test_labware = protocol.load_labware_from_definition(
        LABWARE_DEF,
        TEST_LABWARE_SLOT,
        LABWARE_LABEL,
    )

    num_cols = len(LABWARE_DEF.get('ordering', [[]]))
    num_rows = len(LABWARE_DEF.get('ordering', [[]])[0])
    total = num_cols * num_rows
    pipette.pick_up_tip()

    def set_speeds(rate):
        protocol.max_speeds.update({
            'X': (600 * rate),
            'Y': (400 * rate),
            'Z': (125 * rate),
            'A': (125 * rate),
        })

        speed_max = max(protocol.max_speeds.values())

        for instr in protocol.loaded_instruments.values():
            instr.default_speed = speed_max

    set_speeds(RATE)

    pipette.home()

    protocol.pause("Checking the center of each well. Click 'resume.'")
    for this_well in range(total):
        well = test_labware.well(this_well)
        pipette.move_to(well.top())
    protocol.pause("If the position is accurate click 'resume.'")

    for this_well in range(total):
        well = test_labware.well(this_well)
        all_4_edges = [
            [well._from_center_cartesian(x=-1, y=0, z=1), 'left'],
            [well._from_center_cartesian(x=1, y=0, z=1), 'right'],
            [well._from_center_cartesian(x=0, y=-1, z=1), 'front'],
            [well._from_center_cartesian(x=0, y=1, z=1), 'back']
        ]
        for edge_pos, edge_name in all_4_edges:
            set_speeds(RATE)
            edge_location = types.Location(point=edge_pos, labware=None)
            pipette.move_to(edge_location)
            protocol.pause("If the position is accurate click 'resume.'")

    set_speeds(1.0)
    pipette.return_tip()
