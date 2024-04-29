import numpy as np

w = np.array([
    [9.54750, -0.25780, -0.67200, 1],   # +X,0,0
    [-9.72430, -1.04040, -0.46490, 1],   # -X,0,0
    [-0.06040, 9.16040, 0.37330, 1],    # 0,-Y,0
    [0.02830, -10.01930, -0.57190, 1],   # 0,+Y,0
    [-0.43950, -0.63170, 9.39740, 1],   # 0,0,+Z
    [0.34940, -0.69220, -9.93090, 1]    # 0,0,-Z
])

Y = np.array([
    [9.81, 0, 0],   # +X,0,0
    [-9.81, 0, 0],  # -X,0,0
    [0, 9.81, 0],   # 0,+Y,0
    [0, -9.81, 0],  # 0,-Y,0
    [0, 0, 9.81],   # 0,0,+Z
    [0, 0, -9.81]   # 0,0,-Z
])

X = np.linalg.inv(w.T @ w) @ w.T @ Y
scale_factors = X[:3, :3]
biases = X[3, :3]

print(scale_factors, biases)
measurement = np.array([0.170000, -10.010000, -0.350000])
calibrated_measurement = scale_factors @ measurement + biases

print("measurement:", measurement)
print("calibrated measurement:", calibrated_measurement)
