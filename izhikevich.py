# izhikevich.py

class IzhikevichNetwork:
    def __init__(self, size):
        self.size = size
        self.voltages = [-65.0 for _ in range(size)]
        self.u = [0.0 for _ in range(size)]
        self.a = [0.02 for _ in range(size)]
        self.b = [0.2 for _ in range(size)]
        self.c = [-65.0 for _ in range(size)]
        self.d = [8.0 for _ in range(size)]
        self.weights = [[0.0 for _ in range(size)] for _ in range(size)]
        self.outputs = [0.0 for _ in range(size)]  

    def step(self, dt, inputs):
        dv = [0.0 for _ in range(self.size)]
        du = [0.0 for _ in range(self.size)]
        for i in range(self.size):
            total_input = inputs[i]
            for j in range(self.size):
                total_input += self.weights[i][j] * self.voltages[j]
            dv[i] = 0.04 * self.voltages[i]**2 + 5 * self.voltages[i] + 140 - self.u[i] + total_input
            du[i] = self.a[i] * (self.b[i] * self.voltages[i] - self.u[i])
        for i in range(self.size):
            self.voltages[i] += dt * dv[i]
            self.u[i] += dt * du[i]
            if self.voltages[i] >= 30:
                self.voltages[i] = self.c[i]
                self.u[i] += self.d[i]
        self.outputs = self.voltages.copy()
