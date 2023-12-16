

class DCMotor:

    def __init__(self, k, J, b):
        self.k = k
        self.J = J
        self.b = b
        self.angular_velocity = 0
        self.angle = 0

    def update(self, applied_voltage, time_step):
        acceleration = (applied_voltage - self.b * self.angular_velocity) / self.J
        self.angular_velocity += acceleration * time_step
        self.angle += self.angular_velocity * time_step
        return self.angle
