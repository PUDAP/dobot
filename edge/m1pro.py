from controllably.Move.Jointed.Dobot.m1pro import M1Pro

class M1ProTemp:
    def __init__(self, dobot_ip: str):
        self.arm = M1Pro(
            host=dobot_ip,
            home_position= [[300,0,240],[-33,0,0]],
            calibrated_offset= [[-374,496.75,254.2],[-89.11611149,0,0]],
            scale= 1.0,
            tool_offset= [[0,0,-232],[122.11611149,0,0]],
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
        
    def home(self):
        "homes the dobot arm"
        self.arm.home()
        
    def move(self, position: str):
        "moves the dobot arm to a given position"
        self.arm.move(position)