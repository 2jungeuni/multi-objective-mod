import pickle
import argparse
import numpy as np
import gurobipy as gp
from planner import *
from gurobipy import *
from sklearn.neighbors import BallTree

# status dictionary
status_dict = {1: "loaded",
               2: "optimal",
               3: "infeasible",
               4: "infeasible and unbounded",
               5: "unbounded",
               6: "cut off",
               7: "iteration limit",
               8: "node limit",
               9: "time limit",
               10: "solution limit",
               11: "interrupted",
               12: "numeric",
               13: "suboptimal",
               14: "in progress",
               15: "user objective limit",
               16: "work limit",
               17: "memory limit"}


def create_data_model(func, num_veh, pudo, alpha=1, beta=0, gamma=0):
    """Stores the data for the problem."""
    print(">>> Problem formulation")
    print(f"Alpha: {alpha} / Beta: {beta} / Gamma: {gamma}")
    print(f'Number of vehicle: {num_veh}')
    # planner
    planner = func(alpha, beta, gamma)

    data = {}
    data["pickups_deliveries"] = pudo
    data["num_vehicles"] = num_veh
    data["num_stops"] = len(data["pickups_deliveries"]) * 2
    data["cost"] = np.zeros((data["num_stops"], data["num_stops"]))
    data["stops"] = list(np.array(data["pickups_deliveries"]).reshape((data["num_stops"],)))
    data["indices_stops"] = {}
    for idx, stop in enumerate(data["stops"]):
        data["indices_stops"][idx+1] = stop

    print("Calculating cost ...")
    for i in range(data["num_stops"]):
        for j in range(data["num_stops"]):
            planner.init()
            data["cost"][i][j] = planner.astar(data["indices_stops"][i+1], data["indices_stops"][j+1])

    # set depot
    depot = 0
    data["depot"] = depot
    data["num_stops"] = data["num_stops"] + 1
    data["indices_stops"][0] = depot
    data["stops_indices"] = {v: k for k, v in data["indices_stops"].items()}

    # set penalties for not visiting some stations
    penalty = {}
    for stop in data["stops"]:
        penalty[data["stops_indices"][stop]] = -1000     # penalty
    penalty[0] = -1000                                   # depot penalty

    # arch's cost for a depot
    c_0 = np.zeros((1, data["cost"].shape[0]))
    data["cost"] = np.vstack((c_0, data["cost"]))
    c_0 = np.zeros((1, data["cost"].shape[0]))
    data["cost"] = np.concatenate((c_0.T, data["cost"]), axis=1)

    return data, penalty


def optimize(data, penalty):
    # define and initialize the optimal model
    m = gp.Model()      # minimize is default
    m.Params.outputFlag = False

    num_v = data["num_vehicles"]
    n = data["num_stops"]

    # re-definite distance matrix, t
    dist = {}
    for i, row in enumerate(data["cost"]):
        for j, elem in enumerate(row):
            for v in range(data["num_vehicles"]):
                if (i != j):
                    dist[(i, j, v)] = data["cost"][i][j]

    p = {}
    for p_key, p_val in penalty.items():
        for v in range(num_v):
            p[(p_key, v)] = -1e4

    # edge
    e_vars = m.addVars(dist.keys(), obj=dist, vtype=GRB.BINARY, name="e")
    # penalty
    p_vars = m.addVars(p.keys(), obj=p, vtype=GRB.BINARY, name="p")

    # Constraint 1: only one vehicle can visit one stop except for the depot.
    cons1 = m.addConstrs(p_vars.sum(data["stops_indices"][i], "*") <= 1 for i in data["stops"])
    # Constraint 2: visited node i must have an outgoing edge.
    cons2 = m.addConstrs(e_vars.sum(i, "*", v) == p_vars[(i, v)]
                         for i in list(data["indices_stops"].keys()) for v in range(num_v))
    # Constraint 3: visited node j must an ingoing edge.
    cons3 = m.addConstrs(e_vars.sum("*", j, v) == p_vars[(j, v)]
                         for j in list(data["indices_stops"].keys()) for v in range(num_v))
    # Constraint 4: considering the origin
    cons4_1 = m.addConstr(p_vars.sum(0, "*") == num_v)
    # Constraint 5: pickup-dropoff pairs
    cons5_1 = m.addConstrs(
        e_vars[(data["stops_indices"][pu], data["stops_indices"][do], v)] == p_vars[(data["stops_indices"][pu], v)]
        for pu, do in data["pickups_deliveries"] for v in range(num_v))
    cons5_2 = m.addConstrs(
        p_vars[(data["stops_indices"][pu], v)] == p_vars[(data["stops_indices"][do], v)]
        for pu, do in data["pickups_deliveries"] for v in range(num_v))

    def subtourlim(model, where):
        if where == GRB.Callback.MIPSOL:
            # make a list of edges selected in the solution
            vals = model.cbGetSolution(model._vars)
            selected = gp.tuplelist((i, j, k) for i, j, k in model._vars.keys() if vals[i, j, k] > 0.5)
            # find the shortest cycle in the selected edge list
            tour = subtour(selected)
            for v in range(num_v):
                if tour[v]:
                    for tv in tour[v]:
                        if len(tv) < n:
                            # add subtour elimination constraint for every pair of cities in tour
                            model.cbLazy(gp.quicksum(model._vars[i, j, v] for i, j in itertools.permutations(tv, 2))
                                         <= len(tv) - 1)

    def subtour(edges, exclude_depot=True):
        cycle = [[] for v in range(num_v)]

        for v in range(num_v):
            unvisited = list(np.arange(0, n))

            while unvisited:    # true if list is non-empty
                this_cycle = []
                neighbors = unvisited

                while neighbors:
                    current = neighbors[0]
                    this_cycle.append(current)
                    unvisited.remove(current)
                    neighbors = [j for i, j, k in edges.select(current, '*', '*') if (j in unvisited) and (k == v)]

                if len(this_cycle) > 1:
                    if exclude_depot:
                        if not (data["depot"] in this_cycle):
                            cycle[v].append(this_cycle)
        return cycle

    # optimize model
    m._vars = e_vars
    m._dvars = p_vars
    m.Params.lazyConstraints = 1
    m.optimize(subtourlim)

    # status
    print(">>> Result")
    print("Solver status: ", status_dict[m.status])

    e_vals = m.getAttr('x', e_vars)

    sol = {}
    for car in range(num_v):
        sol[car] = {}
        for i, j, k in e_vals.keys():
            if (e_vals[i, j, k] > 0.5) and (k == car):
                sol[k][i] = j

    routes = []

    for car in range(num_v):
        route = sol[car]
        i = 0
        path = []
        while True:
            i = route[i]
            if i == 0:
                break
            path.append(data["indices_stops"][i])
        routes.append(path)
    return routes




if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Write the number of vehicles, three weights (alpha, beta, gamma), and passengers requests.")
    parser.add_argument("-v", "--num_veh", type=int, required=True, help="number of vehicles")
    parser.add_argument("-w", "--weight", type=int, required=True, nargs="+", help="alpha, beta, gamma")
    parser.add_argument("-pudo", "--pudo", type=int, required=True, nargs="+", action="append", help="passengers requests")

    args = parser.parse_args()

    # make problem
    num_veh = args.num_veh     # number of vehicle
    alpha, beta, gamma = args.weight
    pickup_and_dropoff = args.pudo
    data, penalty = create_data_model(RoutingPlanner, num_veh, pickup_and_dropoff, alpha, beta, gamma)

    # solve
    routes = optimize(data, penalty)
    for car, route in enumerate(routes):
        print(f"car: {car} | route: {route}")