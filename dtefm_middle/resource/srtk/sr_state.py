
class SR_State_Robot():
    def __init__(self, th=0.0, ex=0.0, h1=90.0, h2=90.0, z=0.0):
        self.th = th
        self.ex = ex
        self.h1 = h1
        self.h2 = h2
        self.z = z
        self.pos = [self.th, self.ex, self.h1, self.h2, self.z]
        self.std_pos = [0.0, 0.0, 90.0, 90.0, 0.0]
        
    def set_pos(self, th, ex, h1, h2, z):
        self.th = th
        self.ex = ex
        self.h1 = h1
        self.h2 = h2
        self.z = z
        
class SR_State_PA():
    def __init__(self, th_pa=0.0):
        self.th_pa = th_pa
        
    def set_pos(self, th_pa):
        self.th_pa = th_pa
        
class SR_State():
    def __init__(self, th=0.0, ex=0.0, h1=90.0, h2=90.0, z=0.0, th_pa=0.0):
        self.robot = SR_State_Robot(th, ex, h1, h2, z)
        self.pa = SR_State_PA(th_pa)
        