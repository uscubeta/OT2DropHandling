# for details, https://docs.opentrons.com/v2/writing.html#simulate-block
# for details, https://docs.opentrons.com/v2/tutorial.html#tutorial

import json
from opentrons import protocol_api, types

RATE = 0.25  # % of default speeds (to reduce shaking of gantry)
PIPETTE_MOUNT = 'left'
PIPETTE_NAME = 'p1000_single_gen2'
TIPRACK_LOADNAME = 'opentrons_96_tiprack_1000ul'

TIPRACK_SLOT = '5'
RESEVOIR_SLOT = '1'
WELLPLATE_SLOT = '2'

RESEVOIR_DEF_JSON = """{"ordering":[["A1"],["A2"],["A3"]],"brand":{"brand":"USCtrayvials","brandId":["ThreeJarTray"]},"metadata":{"displayName":"USCtrayvials 3 Reservoir 60000 µL","displayCategory":"reservoir","displayVolumeUnits":"µL","tags":[]},"dimensions":{"xDimension":127.4,"yDimension":85.4,"zDimension":88},"wells":{"A1":{"depth":80,"totalLiquidVolume":60000,"shape":"circular","diameter":28,"x":26.2,"y":59.2,"z":8},"A2":{"depth":80,"totalLiquidVolume":60000,"shape":"circular","diameter":28,"x":63.7,"y":26.2,"z":8},"A3":{"depth":80,"totalLiquidVolume":60000,"shape":"circular","diameter":28,"x":101.2,"y":59.2,"z":8}},"groups":[{"metadata":{"wellBottomShape":"flat"},"wells":["A1","A2","A3"]}],"parameters":{"format":"irregular","quirks":[],"isTiprack":false,"isMagneticModuleCompatible":false,"loadName":"usctrayvials_3_reservoir_60000ul"},"namespace":"custom_beta","version":1,"schemaVersion":2,"cornerOffsetFromSlot":{"x":0,"y":0,"z":0}}"""
RESEVOIR_DEF = json.loads(RESEVOIR_DEF_JSON)
RESEVOIR_LABEL = RESEVOIR_DEF.get('metadata', {}).get(
    'displayName', 'resevoir')

WELLPLATE_DEF_JSON = """{"ordering":[["A1"],["A2"],["A3"],["A4"]],"brand":{"brand":"USCtrayfoam","brandId":["FourSlideTrayFoam"]},"metadata":{"displayName":"USCtrayfoam 4 Well Plate 400 µL","displayCategory":"wellPlate","displayVolumeUnits":"µL","tags":[]},"dimensions":{"xDimension":127,"yDimension":85,"zDimension":19},"wells":{"A1":{"depth":1.2,"totalLiquidVolume":400,"shape":"circular","diameter":12.5,"x":18,"y":55,"z":17.8},"A2":{"depth":1.2,"totalLiquidVolume":400,"shape":"circular","diameter":12.5,"x":48.4,"y":55,"z":17.8},"A3":{"depth":1.2,"totalLiquidVolume":400,"shape":"circular","diameter":12.5,"x":78.8,"y":55,"z":17.8},"A4":{"depth":1.2,"totalLiquidVolume":400,"shape":"circular","diameter":12.5,"x":109.2,"y":55,"z":17.8}},"groups":[{"metadata":{"wellBottomShape":"flat"},"wells":["A1","A2","A3","A4"]}],"parameters":{"format":"irregular","quirks":[],"isTiprack":false,"isMagneticModuleCompatible":false,"loadName":"usctrayfoam_4_wellplate_400ul"},"namespace":"custom_beta","version":1,"schemaVersion":2,"cornerOffsetFromSlot":{"x":0,"y":0,"z":0}}"""
WELLPLATE_DEF = json.loads(WELLPLATE_DEF_JSON)
WELLPLATE_LABEL = WELLPLATE_DEF.get('metadata', {}).get(
    'displayName', 'wellplate')
LABWARE_DIMENSIONS = WELLPLATE_DEF.get('wells', {}).get('A1', {}).get('yDimension')

metadata = {'apiLevel': '2.13'}

def run(protocol: protocol_api.ProtocolContext):
    # load labware onto deck
    tiprack = protocol.load_labware(TIPRACK_LOADNAME, TIPRACK_SLOT)
    pipette = protocol.load_instrument(
        PIPETTE_NAME, PIPETTE_MOUNT, tip_racks=[tiprack])

    resevoir = protocol.load_labware_from_definition(
        RESEVOIR_DEF,
        RESEVOIR_SLOT,
        RESEVOIR_LABEL,
    )

    wellplate = protocol.load_labware_from_definition(
        WELLPLATE_DEF,
        WELLPLATE_SLOT,
        WELLPLATE_LABEL,
    )