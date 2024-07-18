import pickle
import logging
import argparse
import numpy as np
import gurobipy as gp
from planner import *
from gurobipy import *
from tabulate import tabulate

# initialize the logger
logger = logging.getLogger("Multi objective mod system")
logger.setLevel(logging.DEBUG)

handler = logging.StreamHandler()
handler.setLevel(logging.DEBUG)

formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
handler.setFormatter(formatter)
logger.addHandler(handler)

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

def create_data_model(func, num_veh, pudo, alpha=1, beta=0, gamma=0, p=-1e3):
    """Stores the data for the problem."""
    logger.info("Problem formulation with alpha %s, beta $s, gamma %s", alpha, beta, gamma)

    # planner
    planner = func(alpha, beta, gamma)

    # data
    data = {}
    data["pickups_deliveries"] = pudo
    data["num_vehicles"] = num_veh
    data["num_stops"] = len(data["pickups_deliveries"]) * 2
    data["cost"] = np.zeros((data["num_stops"], data["num_stops"]))
    data["stops"] = list(np.array(data["pickups_deliveries"]).reshape((data["num_stops"],)))
    data["indices_stops"] = {}
    for idx, stop in enumerate(data["stops"]):
        data["indices_stops"][idx+1] = stop

    logger.info("Start calculating cost")
    for i in range(data["num_stops"]):
        for j in range(data["num_stops"]):
            planner.init()
            data["cost"][i][j] = planner.astar(data["indices_stops"][i+1], data["indices_stops"][j+1])
    logger.info("Complete cost calculation")

    # set depot (artificial location)
    depot = 0
    data["depot"] = depot
    data["num_stops"] = data["num_stops"] + 1