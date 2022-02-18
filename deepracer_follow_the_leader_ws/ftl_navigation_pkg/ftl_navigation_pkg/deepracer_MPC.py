import cvxpy as cp
import numpy as np

class MPC():
    def __init__(self, v_f=1):
        # Define constants and matrices
        #   m - mass of car [kg]
        #   r - radius of car wheel [m]
        #   A - state matrix of dynamic model
        #   B - input matrix of dynamic model
        #   E - disturbance matrix of dynamic model
        #   v_f - velocity of front leader (either constant or measured each time) [m/s]
        #   R - input gain (DESIGN PARAMETER: increase to use smaller torques)
        #   alpha - input "jerk" (DESIGN PARAMETER: increase to smooth acceleration/decceleration but lower agility)
        #   N - MPC horizon (N*0.1 = number of seconds for horizon)
        #   v_low - minimum velocity constraint [m/s]
        #   v_high - maximum velocity constraint [m/s]
        #   d_min - minimum distance to front leader constraint [m]
        #   torque_low - minimum torque constraint [N*m]
        #   torque_high - maximum torque constraint [N*m]

        self.m = 1.55 
        self.r = 0.034 

        self.A = np.array([[1, -0.1],
                           [0, 1]])
        self.B = np.array([[-1/(200*self.m*self.r)],
                           [1/(10*self.m*self.r)]])
        self.E = np.array([[0.1],
                           [0]])
        self.v_f = v_f
        self.R = 0.75
        self.alpha = 0.1
        self.N = 15 

        # Define constraints
        self.v_low = 0
        self.v_high = 5
        self.d_min = 0.5
        self.torque_low = -0.0174
        self.torque_high = 0.0174

    def MPC_step(self, x_t, print_debug=False):
        # Solve optimization problem for one step of MPC. 
        # Inputs: 
        #   x_t - current state vector [distance to front leader, velocity]

        # Get number of states and inputs
        n = self.A.shape[1] #3 OR 2
        m = self.B.shape[1] #1

        # Create state and input optimization variables.
        x = cp.Variable((n, self.N+1))
        u = cp.Variable((m, self.N)) # u is weird since it is 1x50

        if print_debug:
            print(x.shape)
            print(u.shape)
            print((self.A@x[:, 1]).shape)
            print((self.B@u[:, 1]).shape)
            print((self.E*self.v_f).shape)

        cost = 0
        constraints = []
        # Form cost function objective and constraints over MPC horizon.
        # Current cost function wants ego vehicle to match front vehicle's speed
        # Deviation from v_f, input gain, input jerk.
        for t in range(self.N):
            cost += cp.sum_squares(x[1,t+1] - self.v_f) + cp.sum_squares(u[:, t])*self.R
            if t < self.N - 1:
                cost += self.alpha*cp.sum_squares(cp.abs(u[:, t+1] - u[:, t])) 

            constraints += [x[:,t+1] == self.A@x[:,t] + self.B@u[:, t] + cp.vec(self.E*self.v_f), # state dynamics model
                            self.v_low <= x[1,t+1],
                            x[1,t+1] <= self.v_high, # velocity constraint
                            self.d_min + 1*x[1,t+1] <= x[0,t+1], # distance from front leader constraint
                            self.torque_low <= u[:, t],
                            u[:, t] <= self.torque_high # torque constraint
                            ]

        constraints += [x[:,0] == cp.vec(x_t)]
        obj = cp.Minimize(cost)

        # Form and solve problem.
        prob = cp.Problem(obj, constraints)
        prob.solve()  # Returns the optimal value.

        if print_debug:
            print("status:", prob.status)
            print("optimal cost J:", prob.value)
            print("optimal state:", x.value, "\noptimal input:", u.value)

        return prob.status, x, u, prob.value


