# 🚚 Vehicle Routing Problem (VRP) with Google OR-Tools

A collection of **customized Vehicle Routing Problem (VRP) models** using **Google OR-Tools** and **Python 3.x** for logistics, transportation, and delivery optimization.

> 📄 Detailed article on LinkedIn:  
> [Vehicle Routing Problem with Google OR-Tools Customized](https://www.linkedin.com/pulse/vehicle-routing-problem-google-or-tools-customized-hoai-dang-nguyen)

---

## 💡 Overview

This repository provides various VRP models with practical constraints and enhancements:
- Cost, distance, and vehicle minimization.
- Time windows, load/unload demands, depots, schedules, and more.
- Enhanced heuristics and metaheuristics.

---

## 🔧 Quick Setup

```bash
# Install OR-Tools
python -m pip install --upgrade --user ortools
```

# Models
1. Vehicle Route Problem basic (VRP)
2. Assignment Min Cost Flow
3. Capacity Vehicle Route Problem with Load and Unload Demand (CVRPLU)
4. Capacity Vehicle Route Problem Multiple Time Window (CVRPMTW)
5. Capacity Vehicle Route Problem Single Time Window (CVRPTW)
6. Capacity Vehicle Route Problem (CVRP)
7. Gateway Vehicle Route Problem Time Window (GVRPTW)
8. Pickup and Delivery demand Vehicle Route Problem (PADVRP) 
9. Reload Capacity Vehicle Route Problem Time Window (RCVRPTW)
10. Vehicle Route Problem Multiple Depot (VRPMD)
11. Vehicle Route Problem with Start End Point (VRPSE)
12. Vehicle Route Problem only One Way (VRPOW)
13. Vehicle Route Problem with Parallel Cheapest Insertion (Better result)
14. Vehicle Route Problem Researching
15. Vehicle Route Problem with Schedule
16. Several Solution
17. VRP with New Order

# Data Matrix 
> 💡 **Tip:**  
> It is recommended to prepare the data matrix (distance or duration) using the **[Google Distance Matrix API](https://developers.google.com/maps/documentation/distance-matrix)** for accurate and reliable results.

# Document:
A parallel insertion heuristic for vehicle routing with side constraints (M.W.P. Savelsbergh)
https://core.ac.uk/download/pdf/192270417.pdf

🔬 Advanced VRP Models for Research (Recommended)
VRPB - Vehicle Routing Problem with Backhauls

VRPBTW - VRP with Backhauls and Time Windows

VRPMSB - VRP with Mixed and Selective Backhauls

CVRPSD - CVRP with Stochastic Demands

HFVRP - Heterogeneous Fleet VRP

MOVRPTW - Multi-Objective VRP with Time Windows

RVRP - Rich VRP with multiple constraints

VRPSPDTW - VRP with Simultaneous Delivery, Pickup, and Time Windows

WVRP - Weighted VRP



# Other Vehicle Route Problem Models - Researching
1. Vehicle Routing Problem with Backhauls (VRPB)
- Algorithm: Greedy Randomized Adaptive Search Procedure (GRASP) + Ant colony optimization (ACO) + Permutation Flowshop Scheduling Problems (PFSP) Nawaz Enscore Ham (NEH)
- Object: Lowest cost of route and control quanlity of route

2. Vehicle Routing Problem with Backhauls (VRPB)
- Algorithm: Fuzzy Multiobjective Programe (FMOP)-VRPB: Clustering + Routing + Local search
- Object: Lowest total distance and maximum capacity

3. Capacitiated Vehicle Routing Problem (CVRP)
- Algorithm: Adaptive Multi Rate - Source Adaptive (AMR-SA) and AMR-SA-II
- Object: Lowest cost of route when the capacity of vehicle is limited

4. Capacitiated Vehicle Routing Problem (CVRP)
- Algorithm: Single Inlet and Multi Outlet Model (SMPN) + ACO
- Object: Improving search solution

5. Multi Depot Vehicle Routing Problem (MDVRP)
- Algorithm: Hybrid Mosquito Host-Seeking alogorithm (MHS + 3-opt LS)
- Object: Lowest cost of route, total distance and highest the number point of route


6. VRPB with Time Windowns (VRPBTW)
- Algorithm: Improved NNS+A-Interchage LS method
- Object: Lowest total distance

7. VRP with Mixed and Selective Backhauls (VRPMSB)
- Algorithm: Search Space-Based Multi-Objective Evolutionary Algorithm (SSMOEA)
- Object: Lowest cost of route and optimize return route

8. CVRP with Stochastic Demands (CVRPSD)
- Algorithm: Nearest Neighbor Searches (NNS) + Ant colony optimization algorithm (ACO) +2-opt (The main idea behind it is to take a route that crosses over itself and reorder it so that it does not)
- Object: Highest demand of route and estimate cost by random

9. Heterogeneous Fleet VRP (HFVRP)
- Algorithm: Clustering + CPLEX solver tools
- Object: Lowest cost of route and total used vehicle

10. Multi-Objective Vehicle Routing Problem with Time Windows (MOVRPTW)
- Algorithm: Local Search based multi objective optimization method	
- Object: Optimize total used vehicle with time window

11. Rich Vehicle Routing Problem (RVRP)	
- Algorithm: Hybrid multi-Objective Evolution Algorithm (HMOEA): Ondominated Sorting Genetic Algorithm II (NSGA-II) + Local Search Method (LS)	
- Object: Optimize total used vehicle and estimate risk cost of route

12. VRP with Simultaneous Delivery and Pickup and Time Windows (VRPSPDTW)	
- Algorithm: Multiobjective local search (MOLS) + Multiobjectvie memetic algorithm (MOMA)	
- Object: Lowest total used vehicle, total distance and picked demand by time window

13. Weighted VRP (WVRP)	
- Algorithm: Ant colony optimization algorithm (ACO) - MAX-MIN ant system (MMAS)	
- Object: Lowest total distance when lowest cost of route


------------------------------------------------------------------------------------------
Tks #skynet #googleai
