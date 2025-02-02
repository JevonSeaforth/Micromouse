# Micromouse
 
This project is simulates the popular Micromouse robotics competiton, where autonomous robots navigate through an unknown maze as quickly as possible. The project allows users to visualise and compare the performance of well known pathfinding algorithms A-star (A*) and Breadth-First Search (BFS). 

These algorithms are play a key role in artificial intelligence and robotics, enabling efficient navigation and decision-making in dynamic environments. The project offers users a real-time view of how these algorithms navigate through various mazes, providing valuable insights into their behavior and efficiency. 

## A* (A-star) search algorithm
A* search is an informed search method, which means it uses additional knowledge to make informed decisions about which path to explore. A* combines two factors to determine the best path: 

  g-value: The cost to reach the current node from the start point
  
  h-value (heuristic): An estimate of the cost from the current node to the goal

A* explores paths based on the sum of these two values (f = g + h), prioritizing paths that are likely to lead to the shortest possible route to the goal. By considering both the actual and estimated costs, A* tends to explore not only shorter paths, but also those that are more promising, potentially leading it closer to the goal. The most promising path is one that balances both reaching the goal quickly (heuristic value) and the cost incurred to get there (g-value). This makes it more likely to find the shortest path, while avoiding unnecessary exploration of less optimal paths. 

A* search is often faster and more efficient than Breadth-First search, particularly in more complex environments. 

## BFS (Breadth-First Search) algorithm
Breadth-First search is an uninformed search metod, meaning it explores the paths blindly, without using any additonal knowledge to prioritize nodes. BFS systematically explores all possible paths, starting from an intial node and moving outwards to neighbouring nodes. 

Method for expanding nodes: 

  Explore all neighbouring nodes: BFS starts from intial node and then moves to all of its neighbours. 
  Expanding each depth level: After exploring all nodes at a given depth, BFS moves to explore nodes at the  
  next depth level, iterating until the goal is found.

BFS guarantees that the shortest path in an unweight grid/graph because every node is explored before moving to the next depth level. 

## Future improvements
Looking forward, I plan to add the following features to the project: 

### Additional algorithms:
Implementing more algorithms would provide a wider perspective on the range of avaiable techniques and help users compare their performance. 

### Add a dynamic GUI: 
Integrating a GUI would enhance the user experience, making it easier to visualize and interact with the algorithm selection and maze navigation in real-time. 
