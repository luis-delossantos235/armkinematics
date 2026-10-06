import numpy as np
import math

MAX_STEP = 10

class JacobianController:
    def __init__(self, arm):
        # the controller should save a reference to the arm it's controlling
        self.arm = arm

    def control(self, target):
        print("Numero de articulaciones:", self.arm.get_num_joints())

        if self.arm.get_num_joints() == 2:
            self.control2J2D(target)

        elif self.arm.get_num_joints() == 3:
            self.control3J2D(target)

        elif self.arm.get_num_joints() == 6:
            self.control6J2D(target)

        else:
            raise Exception(
                "JacobianController.control(target): "
                "Can't control an arm with this amount joints."
            )

    def control2J2D(self, target):
        # the control method receives a target
        pass


    def control3J2D(self, target):
        # the control method receives a target
        pass

    # Código agregado
    def control6J2D(self, target):
        n = self.arm.get_num_joints()

        current = self.arm.endeffector()

        current_x = current.r * math.cos(current.a)
        current_y = current.r * math.sin(current.a)

        target_x = target.r * math.cos(target.a)
        target_y = target.r * math.sin(target.a)

        error = np.array([
            target_x - current_x,
            target_y - current_y
        ])


        J = np.zeros((2, n))

        for i in range(n):
            for j in range(i, n):

                angle = sum(self.arm.thetas[:j+1])

                J[0, i] += -self.arm.lengths[j] * math.sin(angle)
                J[1, i] +=  self.arm.lengths[j] * math.cos(angle)


        J_pinv = np.linalg.pinv(J)


        delta_theta = J_pinv @ error


        max_step_rad = math.radians(MAX_STEP)
        max_delta = np.max(np.abs(delta_theta))

        if max_delta > max_step_rad:
            delta_theta = delta_theta * (max_step_rad / max_delta)


        self.arm.move(delta_theta)
