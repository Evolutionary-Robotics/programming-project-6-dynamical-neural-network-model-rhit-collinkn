
class LotkaVolterra():

    def __init__(self,a,b,c,d,x,y):
        # Model Parameters
        self.a = a  # alpha = reproduction rate of prey
        self.b = b  # beta = mortality rate of predator per prey
        self.c = c  # gamma = mortality rate of predator
        self.d = d  # delta = reproduction rate of predator per prey
        # Model Variables
        self.x = x  # Starting value of prey (rabbits)
        self.y = y  # Starting value of predator (foxes)

    def step(self, dt):
        # step 1
        dxdt = self.a*self.x - self.b*self.x*self.y    # Differential equation
        dydt = self.d*self.x*self.y - self.c*self.y
        # step 2
        self.x += dt * dxdt                             # Euler step
        self.y += dt * dydt
