
#%% 



#Initialise M1Pro
from  controllably.Move.Jointed.Dobot.m1pro import M1Pro

arm = M1Pro(host= '192.109.209.21',
    home_position= [[300,0,240],[-33,0,0]],
    calibrated_offset= [[-374,496.75,254.2],[-89.11611149,0,0]],
    scale= 1.0,
    # tool_offset= [[0,0,-232],[122.11611149,0,0]],
    safe_height= 240,
    verbose= True,
    simulation= False,
    saved_positions={
      'zone1_jar_tray_1_center': [[88.7,-29,60], [57,0,0]],
      'zone1_jar_tray_2_center': [[88.7,122,60], [57,0,0]],
      'zone1_jar_tray_4_center': [[-13.4,122,60], [57,0,0]],
      'zone2_jar_tray_center': [[636.4,246.7,60], [-33,0,0]]
      }
    )
arm.loadDeckFromFile('some_deck_file.json')
coordinates = (780,204.5,63)

arm_deck = arm.deck
zone_a = arm_deck.zones['zone_A'] # For platforms with diverse qubots zones are made for each of the modules 
zone_b = arm_deck.zones['zone_B']# These zones define specific way points to safely access this modules

formulations_ok_arm = zone_a.getSlot(5).loaded_labware
formulations_t_arm = zone_b.getSlot(1).loaded_labware

arm.setSpeedFactor(0.3)      # suitable speed: 30%
imaging_station = arm.deck.getSlot(1).loaded_labware.listWells()[0]

#%% [markdown]
"""
## 6. Mixing of formulations
The formulation plate is transferred to the magnetic stirring station for mixing.
The formulations are mixed using a magnetic stirrer for 10 minutes.
"""
#%% 

# Transfer formulations from zone 1 to mixer
arm.home()
arm.setHandedness(False)
arm.safeMoveTo(arm.position)
arm.enterZone('zone_A')
arm.safeMoveTo(formulations_ok_arm.fromTop([0,4,3]))
arm.setSpeedFactor(0.1)
arm.move('z',-4.5, speed_factor=0.01)
arm.setSpeedFactor(0.3)
arm.grab()

arm.setSpeedFactor(0.1)
arm.move('z',3.5, speed_factor=0.01)
arm.setSpeedFactor(0.3)
arm.exitZone()

arm.safeMoveTo(coordinates) 
arm.move('z',-3.5, speed_factor=0.01)
arm.drop()

#%% [markdown]
"""
## 7. Inspection of formulations
The formulations are transferred to the imaging station for inspection.
The formulations are inspected for stability and are recorded using a camera.
"""
#%% 

# This an example of using the arm for a more complicated movement.
# In this example the arm has grabbed a mtp tray holding 6 jars.
# The arm translates and rotates to specific poses so the jars
# are imaged at the same position and distance from a camera

from controllably.core.position import convert_to_position
import time

def inspect_tray(mover, camera, dwell_time:float = 3) -> str:
    """
    Capture images of the jars on the tray, and save the images.
    YUPI YEY EYE EY
    Args:
        mover (Callable): Mover object
        camera (Callable): Camera object
        dwell_time (float, optional): dwell time for image capture. Defaults to 3.

    Returns:
        str: folder name of where captured images are saved
    """
  
    a = 45              # deg
    x = 32.36           # mm
    y = 14.90           # mm
    t = dwell_time      # sec
    move_coords = [
        ((x/2,-y/2,0), (-a,0,0)),   # A1
        ((-x/2,x-y/2,0), (a,0,0)),         # A2
        ((-x/2,-y*1.5,0), (a,0,0)),        # A3
        ((0,0,0), (-180,0,0)),       # B1
        ((x/2,y*1.5,0), (-a,0,0)),       # B2
        ((x/2,-y*1.5,0), (-a,0,0))       # B3
    ]
    # move_pos = []
    # for coords in move_coords:
    #     move_pos.append(convert_to_position(coords))

    jars = ['A1','A2','A3','B1','B2','B3']
    
    camera.connect()
    time.sleep(10)
    mover_positions = []
    images = []
    for move,jar in zip(move_coords, jars):
        mover.rotateBy(move[1])
        if move[0] != (0,0,0):
            mover.moveBy(move[0])
       
        ret, frame = camera.getFrame(latest=True)
        images.append(frame)
    camera.disconnect()

    mover.moveBy(convert_to_position(((0,0,0),(88,0,0))))
    return images
# pick up tray from mixing station

arm.safeMoveTo(coordinates) 
arm.move('z',-3.5, speed_factor=0.01)

arm.grab()
arm.move('z',5, speed_factor=0.01)

# # bring tray to inspection position

arm.safeMoveTo(imaging_station.fromTop((-0.5,-20,12)))
arm.setHandedness(False)
arm.safeMoveTo(imaging_station.fromTop((-0.5,-20,12)))

arm.move('z',-126)

images = inspect_tray(arm, 'some camera')


# Transfer tray to pH adjustment station
arm.enterZone('zone_B')
arm.safeMoveTo(formulations_t_arm.fromTop((0,1.5,6.5))) ## Change if needed 
arm.move('z',-5.5, speed_factor=0.01)
time.sleep(3)
arm.drop()

arm.move('z',5, speed_factor=0.01)
arm.exitZone()


# # return arm to home
arm.home()



#