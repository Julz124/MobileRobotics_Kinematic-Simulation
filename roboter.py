class Roboter:
    def __init__(self, l=0.6, h=0.2, r=0.1, l1=0.6, l2=0.4, 
                 beta_1_limits=(10, 170), beta_2_limits=(0, 360), 
                 init_x=2, init_y=1, init_theta=30, alpha_limits=(0, 360)):

        # Length of roboter
        self.l = l  
        # Height of roboter
        self.h = h  
        # Radius of roboter wheel
        self.r = r  
        # Length of first arm segment
        self.l1 = l1  
        # Length of second arm segment
        self.l2 = l2  
        # Angle limits for beta_1 (roboter joint 1)
        self.beta_1_limits = beta_1_limits  
        # Angle limits for beta_2 (roboter joint 2)
        self.beta_2_limits = beta_2_limits  
        # Initial X-Coordinate of roboter in Coordinate System {O} 
        self.init_x = init_x    
        # Initial Y-Coordinate of roboter in Coordinate System {O}
        self.init_y = init_y    
        # Initial theta (rotation) of roboter in Coordinate System {O}
        self.init_theta = init_theta        
        # Angle limits for alpha (arm rotation)
        self.alpha_limits = alpha_limits    