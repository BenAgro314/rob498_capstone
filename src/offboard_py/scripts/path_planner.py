#%%
from utils import Colors
from ompl import util as ou
from typing import Union
from ompl import base as ob
import numpy as np
from ompl import geometric as og
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

class ValidityChecker(ob.StateValidityChecker):
    # Returns whether the given state's position overlaps the
    # circular obstacle

    def __init__(self, space, obstacle_points, obstacle_radius):
        super().__init__(space)
        # obstacles points (N, 3)
        self.obstacle_points = obstacle_points 
        self.obstacle_radius = obstacle_radius

    def isValid(self, state):
        if self.obstacle_points.shape[0] == 0:
            return True
        x, y, z = state[0], state[1], state[2]
        pt = np.array([x, y, z])
        dist = np.linalg.norm(pt - self.obstacle_points, axis = -1)
        return np.min(dist) > self.obstacle_radius

def state_to_q(state):
    """
    Parses ompl RealVectorStateSpae::StateType into a numpy array
    """
    q = []
    for i in range(3):
        q.append(state[i])
    return np.array(q)


def q_to_state(space, q):
    """
    Turns a numpy array in to a RealVectorStateSpae::StateType
    """
    state = ob.State(space)
    for i in range(len(q)):
        state[i] = q[i]
    return state

def find_traj(
    start: Union[np.array, list], # (3), np.array
    goal: Union[np.array, list], # (3), np.array
    obstacle_points: np.array, # (N, 3)
    obstacle_radius: float, # float
    limits: Union[np.array, list], # (3, 2), x, y, and z limits, low, high
):
    space = ob.RealVectorStateSpace(3)
    bounds = ob.RealVectorBounds(3)
    for i in range(3):
        bounds.setLow(i, limits[i][0])
        bounds.setHigh(i, limits[i][1])

    space.setBounds(bounds)
    si = ob.SpaceInformation(space)

    checker = ValidityChecker(si, obstacle_points, obstacle_radius=obstacle_radius)

    si.setStateValidityChecker(checker)
    #si.setStateValidityCheckingResolution(0.01) 
    si.setup()

    start = q_to_state(space, start)
    goal = q_to_state(space, goal)

    if not checker.isValid(start):
        print(f"{Colors.RED}INVALID OMPL START STATE{Colors.RESET}")
        return None
    if not checker.isValid(goal):
        print(f"{Colors.RED}INVALID OMPL GOAL STATE{Colors.RESET}")
        return None

    pdef = ob.ProblemDefinition(si)
    pdef.setStartAndGoalStates(start, goal)

    planner = og.LBKPIECE1(si)
    planner.setProblemDefinition(pdef)
    planner.setBorderFraction(0.1)
    planner.setup()

    solved = planner.solve(ob.CostConvergenceTerminationCondition(pdef))

    if not solved:
        print(f"{Colors.RED}COULD NOT FIND TRAJECTORY{Colors.RESET}")
        return None

    simplifier = og.PathSimplifier(si)
    path = pdef.getSolutionPath()
    simplifier.simplify(path, 10)

    res = np.array([state_to_q(state) for state in path.getStates()])
    return res



if __name__ == "__main__":
    limits = [
        [-3, 3], # x
        [-3, 3], # y
        [0, 3], # z
    ]
    obstacle_points = np.random.rand(4, 3)
    obstacle_radius = 0.3

    res = find_traj(
        start = [0, 0, 1.5],
        goal = [1, 1, 1],
        obstacle_points=obstacle_points,
        obstacle_radius=obstacle_radius,
        limits = limits,
    )

    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.plot3D(res[:, 0], res[:, 1], res[:, 2])

    for pt in obstacle_points:
        r = 0.15
        u, v = np.mgrid[0:2 * np.pi:30j, 0:np.pi:20j]
        x = pt[0] + r * np.cos(u) * np.sin(v)
        y = pt[1] + r * np.sin(u) * np.sin(v)
        z = pt[2] + r * np.cos(v)
        ax.plot_surface(x, y, z)# , cmap=plt.cm.YlGnBu_r)

    plt.savefig("out.png")