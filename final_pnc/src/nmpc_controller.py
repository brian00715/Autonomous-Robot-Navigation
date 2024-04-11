"""
 # @ Author: Kuankuan Sima
 # @ Create Time: 2022-05-07 20:29:25
 # @ Modified time: 2022-05-08 21:43:56
 # @ Description: NMPC controller based on CasADi. 
 # Use dynamics model, 4th runge kutta integrator and direct multiple shooting method.
 # x: [x, y, theta]
 # u: [v, omega]
 """

import casadi as ca
import numpy as np


def weight_mat2str(mat):
    return np.array2string(mat, separator=", ", prefix=" " * 27, formatter={"all": lambda x: f"{x:.2f}"})


class NMPCC:
    """Nonlinear Model Predictive Controler"""

    def __init__(
        self,
        T=0.2,
        N=10,
        Q=np.diag([5, 5, 0]),
        R=np.diag([5, 1]),
        Qf=None,  # None,
        solver_params: dict = None,
        prob_params: dict = None,  # optimal control problem parameters
        integrator="euler",
    ):
        """

        Args:
            T (float, optional): Prediction time step.
            N (int, optional): Prediction length (steps).
            Q (np.ndarray, optional):  States weight matrix.
            R (np.ndarray, optional):  Input weight matrix.
            Qf (np.ndarray, optional): Final weight matrix.
            casadi_params (dict, optional): casadi solver parameters
            prob_params (dict, optional): optimal control problem parameters like mass, max_thrust, max_atti_ang, max_thrust_rate, max_atti_ang_rate
        """

        self.T = T  # time step (s)
        self.N = N  # horizon length
        self.integrator = integrator

        self.opti = ca.Opti()
        self.control_dim = prob_params["control_dim"]  # 2
        self.state_dim = prob_params["state_dim"]  # 3

        # create casadi parameters
        self.ca_params = {
            "Q": self.opti.parameter(self.state_dim, self.state_dim),
            "R": self.opti.parameter(self.control_dim, self.control_dim),
            "Qf": self.opti.parameter(self.state_dim, self.state_dim),
        }
        prob_params["Q"] = Q
        prob_params["R"] = R
        if Qf is None:
            prob_params["Qf"] = Q
        for param, value in prob_params.items():
            if isinstance(value, (int, float)):
                self.ca_params[param] = self.opti.parameter(1)
            if isinstance(value, np.ndarray):
                if value.ndim == 1:
                    self.ca_params[param] = self.opti.parameter(value.shape[0])
                elif value.ndim == 2:
                    self.ca_params[param] = self.opti.parameter(value.shape[0], value.shape[1])
        # set the parameter initial values
        for param in self.ca_params:
            self.opti.set_value(self.ca_params[param], prob_params[param])

        print("NMPCC Parameters".ljust(50, "-"))
        print("{:<25}: {}".format("T", T))
        print("{:<25}: {}".format("N", N))
        print("{:<25}: {}".format("init_pose", prob_params["init_pose"]))
        print("{:<25}: {}".format("max_vel (m/s)", prob_params["max_vel"]))
        print("{:<25}: {}".format("max_omega (rad/s)", prob_params["max_omega"]))
        print("{:<25}: {}".format("max_lin_acc (m/s^2)", prob_params["max_lin_acc"]))
        print("{:<25}: {}".format("max_ang_acc (rad/s^2)", prob_params["max_ang_acc"]))
        print("{:<25}: {}".format("Q", weight_mat2str(prob_params["Q"])))
        print("{:<25}: {}".format("Qf", weight_mat2str(prob_params["Qf"])))
        print("{:<25}: {}".format("R", weight_mat2str(prob_params["R"])))
        print("-" * 50)

        self.x0 = self.opti.parameter(1, self.state_dim)  # initial state
        # history states and controls
        self.x_opti = np.ones((self.N + 1, self.state_dim)) * prob_params["init_pose"].T
        self.u_opti = np.zeros((self.N, self.control_dim))

        # state variables
        self.var_states = self.opti.variable(self.N + 1, self.state_dim)
        # x = self.var_states[:, 0]
        # y = self.var_states[:, 1]
        # theta = self.var_states[:, 2]

        # control variables
        self.var_controls = self.opti.variable(self.N, self.control_dim)
        v = self.var_controls[:, 0]
        omega = self.var_controls[:, 1]

        self.u_ref = self.opti.parameter(self.N, self.control_dim)
        self.x_ref = self.opti.parameter(self.N + 1, self.state_dim)

        # dynamics differential equation
        self.dde = lambda x_, u_: ca.vertcat(
            *[
                np.cos(x_[2]) * u_[0] * self.T,
                np.sin(x_[2]) * u_[0] * self.T,
                u_[1] * self.T,
            ]
        )

        # cost function
        cost = 0
        # Q_time, R_time = [], []
        # Q1_scale = np.linspace(Q[0, 0], 0, self.N + 1)
        # Q2_scale = np.linspace(Q[1, 1], 0, self.N + 1)
        # for i in range(self.N + 1):
        #     Q_t = np.diag([Q1_scale[i], Q2_scale[i], Q[2, 2]])
        #     Q_time.append(Q_t)
        #     R_time.append(R)
        # Q_time, R_time = np.array(Q_time), np.array(R_time)
        # for i in range(-3, 0):  # # minimize the control input near the end
        #    R_time[i] *= 10
        for i in range(self.N + 1):
            state_error_ = self.var_states[i, :] - self.x_ref[i, :]
            if i < self.N:
                control_error_ = self.var_controls[i, :] - self.u_ref[i, :]
                cost = (
                    cost
                    # + ca.mtimes([state_error_, Q_time[i], state_error_.T])
                    # + ca.mtimes([control_error_, R_time[i], control_error_.T])
                    + ca.mtimes([state_error_, self.ca_params["Q"], state_error_.T])
                    + ca.mtimes([control_error_, self.ca_params["R"], control_error_.T])
                )
                # cost += self.var_controls[i, 0] * 2
            else:
                cost = cost + ca.mtimes([state_error_, self.ca_params["Qf"], state_error_.T])

        self.opti.minimize(cost)

        # constraints

        ## initial condition
        self.opti.subject_to(self.var_states[0, :] == self.x0)

        ## state space constraint
        max_lin_acc = self.ca_params["max_lin_acc"]
        max_ang_acc = self.ca_params["max_ang_acc"]
        for i in range(self.N):
            if self.integrator == "euler":
                x_next = self.var_states[i, :] + self.dde(self.var_states[i, :], self.var_controls[i, :]).T
            elif self.integrator == "rk4":
                x_next = self.runge_kutta(self.dde, self.T, self.var_states[i, :], self.var_controls[i, :].T)
            self.opti.subject_to(self.var_states[i + 1, :] == x_next)

            if i > 1 and 1:  # acceleration limit
                self.opti.subject_to(
                    self.opti.bounded(
                        -max_lin_acc, (self.var_states[i + 1, 0] - self.var_states[i, 0]) / self.T, max_lin_acc
                    )
                )
                self.opti.subject_to(
                    self.opti.bounded(
                        -max_lin_acc, (self.var_states[i + 1, 1] - self.var_states[i, 1]) / self.T, max_lin_acc
                    )
                )
                self.opti.subject_to(
                    self.opti.bounded(
                        -max_ang_acc, (self.var_states[i + 1, 2] - self.var_states[i, 2]) / self.T, max_ang_acc
                    )
                )

        ## input limits
        # self.opti.subject_to(self.opti.bounded(-self.ca_params["max_vel"], v, self.ca_params["max_vel"]))
        self.opti.subject_to(self.opti.bounded(0.2, v, self.ca_params["max_vel"]))
        self.opti.subject_to(self.opti.bounded(-self.ca_params["max_omega"], omega, self.ca_params["max_omega"]))
        self.opti.subject_to(v**2 + omega**2 <= 6**2)

        if solver_params is not None:
            self.opts_setting = solver_params
        else:
            self.opts_setting = {
                "ipopt.max_iter": 2000,
                "ipopt.print_level": 0,
                "print_time": 0,
                "ipopt.acceptable_tol": 1e-8,
                "ipopt.acceptable_obj_change_tol": 1e-6,
                # "expand":True
            }
        self.opti.solver("ipopt", self.opts_setting)

    def solve(self, x_curr: np.array, x_ref, u_ref, return_first_u=True):
        self.opti.set_value(self.x_ref, ca.DM(x_ref))
        self.opti.set_value(self.u_ref, ca.DM(u_ref))
        self.opti.set_value(self.x0, x_curr)

        # provide the initial guess of the optimization targets
        self.opti.set_initial(self.var_states, self.x_opti)
        self.opti.set_initial(self.var_controls, self.u_opti)

        # solve the problem
        sol = self.opti.solve()

        # obtain the control input
        self.u_opti = sol.value(self.var_controls)
        self.x_opti = sol.value(self.var_states)
        return self.u_opti[0, :] if return_first_u else self.u_opti, self.x_opti

    def set_param(self, param_name, value):
        if param_name in self.ca_params:
            self.opti.set_value(self.ca_params[param_name], value)
        else:
            print(f"Parameter {param_name} not found!")

    @staticmethod
    def runge_kutta(f, dt, x, u):
        k1 = f(x, u)
        k2 = f(x + dt / 2 * k1.T, u)
        k3 = f(x + dt / 2 * k2.T, u)
        k4 = f(x + dt * k3.T, u)
        new_x = x + dt / 6 * (k1.T + 2 * k2.T + 2 * k3.T + k4.T)
        return new_x


if __name__ == "__main__":
    mpc_controller = NMPCC(np.array([0, 0, 0, 0, 0, 0]), -1, 1, -1, 1, -np.pi / 6, np.pi / 6)
    x_ref = np.array([(0, 0, 2, 0, 0, 0)] * (mpc_controller.N + 1))
    u_ref = np.zeros((mpc_controller.N, 4))
    x_curr = np.array([0, 0, 0, 0, 0, 0])
    opt_u = mpc_controller.solve(x_curr, x_ref, u_ref)
    # print(f"opt_u: {opt_u[0]:.2f,opt_u[1]:.2f,opt_u[2]:.2f,opt_u[3]:.2f} pose: {x_curr}")
