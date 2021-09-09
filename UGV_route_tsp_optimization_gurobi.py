import numpy as np
from gurobipy import *
import pandas as pd

model = Model('UGV_TSP')
x, t = {}, {}
M = 20

coords = np.array([[13200, 13200], [20592, 23020.8], [23628, 11748], [4752, 3537.6], [18849.6, 4224], [5121.6, 18585.6]])
X = []
Y = []
for i in range(len(coords)):
    X.append(coords[i][0])
for i in range(len(coords)):
    Y.append(coords[i][1])

distance_matrix = np.empty((len(coords), len(coords)))
for i in range(len(coords)):
    for j in range(len(coords)):
        x[i, j] = model.addVar(vtype=GRB.BINARY, name="x%d,%d" % (i, j))
        distance_matrix[i, j] = np.sqrt((X[i] - X[j]) ** 2 + (Y[i] - Y[j]) ** 2)
        if i == j:
            distance_matrix[i, j] = 100000
            continue
model.update()

for i in range(len(coords)):
    t[i] = model.addVar(lb=1, ub=len(coords), vtype=GRB.CONTINUOUS, name="y%d" % i)

for i in range(len(coords)):
    model.addConstr(quicksum(x[(i, j)] for j in range(len(coords))) == 1)
model.update()

for j in range(len(coords)):
    model.addConstr(quicksum(x[(i, j)] for i in range(len(coords))) == 1)
model.update()

for i in range(len(coords)):
    for j in range(len(coords)):
        if i != j and (i != 0 and j != 0):
            model.addConstr(t[j] >= t[i]+1 - M*(1 - x[(i, j)]))
model.update()

model.setObjective(quicksum(quicksum(x[(i, j)]*distance_matrix[(i, j)] for j in range(len(coords))) for i in range(len(coords))),GRB.MINIMIZE)
model.update()

model.Params.Method = 1
model.Params.IntFeasTol = 0.000000001
model.Params.NumericFocus = 3
model.Params.TimeLimit = 240  # seconds
model.optimize()

sol_x = model.getAttr('x', x)
sol_t = model.getAttr('x', t)
X = np.empty([len(coords), len(coords)])
T = np.empty([len(coords)])
for i in range(len(coords)):
    T[i] = int(sol_t[i])
    for j in range(len(coords)):
        X[i, j] = int(sol_x[i, j])

print('\nObjective is:', model.objVal)
