# Plot the data recorded for comparison

import matplotlib.pyplot as plt

steps = 100
max_iter = 1000

lazy_sp_times = [4.989102125167847, 12.066282987594604, 10.153723001480103, 25.33744192123413, 23.824832916259766,
         46.098556995391846, 49.717559814453125, 54.00535202026367, 87.7180380821228, 112.72428107261658]

rrt_times = [17.252784967422485, 9.45022702217102, 32.16299104690552, 4.256010055541992, 35.861401081085205,
             29.13967204093933, 4.48254919052124, 15.407569885253906, 3.447671890258789, 49.855316162109375]

rrt_star_times = [17.527981996536255, 46.047972202301025, 118.98934412002563, 176.42872190475464, 272.69289898872375] + [0]*5

lazy_sp_edge_evals = [5.0, 9.0, 1.0, 10.0, 1.0, 1.0, 1.0, 1.0, 7.0, 35.0]

rrt_edge_evals = [100.0, 41.0, 158.0, 24.0, 158.0, 124.0, 24.0, 87.0, 14.0, 193.0]

rrt_star_edge_evals = [199.0, 290.0, 613.0, 940.0, 1276.0] + [0]*5

x_range = range(steps, max_iter+steps, steps)
plt.plot(x_range, lazy_sp_times, label="LazySP")
plt.plot(x_range, rrt_times, label="RRT")
plt.plot(x_range, rrt_star_times, label="RRT*")
plt.xlabel("max iterations")
plt.ylabel("time taken (seconds)")
plt.legend()
plt.show()


plt.plot(x_range, lazy_sp_edge_evals, label="LazySP")
plt.plot(x_range, rrt_edge_evals, label="RRT")
plt.plot(x_range, rrt_star_edge_evals, label="RRT*")
plt.xlabel("max iterations")
plt.ylabel("number of edge evaluations")
plt.legend()
plt.show()
