class Roboter:
    def __init__(self, l=0.6, h=0.2, r=0.1, l1=0.6, l2=0.4, beta_1_limits=(10, 170), beta_2_limits=(0, 360)):

        self.l = l  # Length of ???
        self.h = h  # Height of roboter to KS-R
        self.r = r  # Radius of roboter wheel
        self.l1 = l1  # Length of first arm segment
        self.l2 = l2  # Length of second arm segment
        self.beta_1_limits = beta_1_limits  # Angle limits for beta_1
        self.beta_2_limits = beta_2_limits  # Angle limits for beta_2