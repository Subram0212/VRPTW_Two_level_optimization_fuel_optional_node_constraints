# Solving Vehicle Routing Problem with time windows, fuel constraints and optional node constraints using OR-Tools’ constraint programming solver.

This project has the code for the simulation results produced in the IEEE paper titled “Cooperative Route planning of multiple fuel-constrained Unmanned Aerial Vehicles with recharging on an Unmanned Ground Vehicle”.

The two-level optimization code is as follows:

**Kmeans_clustering_4clusters:**  Using unsupervised Machine Learning (ML) algorithm to cluster the mission points on the area into 4 clusters. Each cluster has its own cluster centroid.

**IEEEpaper_UGV_route_optimization:**  Formulating a Traveling Salesman Problem (TSP) to find the optimal UGV path through the 4 centroid points obtained from K-means clustering.

**IEEEpaper_UAS_optimization_codes:**  Folder containing the UAV/UAS optimization codes. The ‘k’ in each file name represents the number of clusters. For example, k4 means solving the lower level UAV optimization using optimized UGV route obtained from 4 clusters. ‘V’ in the file name represents the number of UAVs.

**UGV_gurobi_tsp_k5:**  Formulating TSP problem for solving the optimal upper level UGV route using Gurobi Optimization.
