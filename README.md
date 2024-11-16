# A-Star

Coded on Nov 7, 2024.

- A modifiable implementation of the **A\*** algorithm where the diagonal and horizontal "costs" can be adjusted, along with the "weights" of the G_Cost and F_Cost, allowing for customization of the standard A* algorithm. Although written in C# for use with Unity, the variable names and AStarNode class can be easily modified to suit other applications.

## Motivation
- Initially, I was using a token-based pathfinding system (where tokens were dropped by object A and followed by object B). However, it was not flexible enough and led to many bugs. After researching, I discovered A*, which seemed like a smarter version of Dijkstra's algorithm because it uses a heuristic.
- Since I was already working with a grid-based system, A* seemed like a natural fit, so I learned how it works with the summation of G and H costs, and then implemented the code.

## Details
- The implementation includes a feature to mark tiles as "Untraversable," allowing for the creation of barriers that the pathfinding system will navigate around.
- This was designed for use on a square grid, though modifications can be made to support hexagonal grids or a polygon-based grid/mesh system.

## Developer Notes
- This was written for **BlackWinter**, a game that I am developing, which can be seen to a small extent here: https://github.com/Cimmerial/BlackWinter-Public.git 
- All the fancy comment blocks seem over the top here but are for visual clarity and organization within **BlackWinter**.
