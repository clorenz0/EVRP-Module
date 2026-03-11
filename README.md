# ORTools
Experiment with OR-Tools to solve transport problems. OR-Tools, an optimization library by Google, offers algorithms for combinatorial optimization. "OR" signifies "Operations Research," focusing on analytical methods for optimal decisions in complex scenarios.

1. The "execute" folder inside "problems" contain the 6 problems:
   CVRP (Capacitated Vehicle Routing Problem)
   VRPTW (Vehicle Routing Problem with Time Windows)
   VRPPD (Vehicle Routing Problem with Pickup and Delivery)
   MDVRP (Vehicle Routing Problem with Multiple Depots)
   TSP (Traveling Salesman Problem)


2. The "solutions" folder inside "problem" contain the solutions according to the instances evaluated
 in the problem, with distance Euclidean, Manhattan, Haversine y Chebyshev.
 Example: ("solutions_vrptw" -> contains the results of the instances within 'vrptw_instances')


3. The "import_data" file inside "instance" contains the code to import data from previous problem entries
 for each of the problems to be solved so that ORTools processes it with the structure
 it accepts. 


4. This data that is imported is found in the "instances_data" folder separated by folders
 according to the problems. 


5. In the "predictions" folder, you will find everything related to the training and testing of the random forest 
 models for each VRP variant.


6. In the "distance" folder, you will find the necessary components to perform the calculations for the four types 
 of distances covered.


7. The "run.py" file contains the code to run the program and call ORTools solvers.