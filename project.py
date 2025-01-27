import sys
import time
import os
import heapq
from queue import Queue
from collections import deque
import copy

"""
Note: use os.system('cls') is used, which is a Windows specific command - may cause issues on alternative Operating Systems
Note: printMaze has time.sleep(0.03) function to slow down execution for better readability. The number passed can be altered for slower or faster execution times
"""

# Available mazes to test
mazeFileOne = 'maze.txt'
mazeFileTwo = 'maze2.txt'
mazeFileThree = 'maze3.txt'

# List of permitted moves. 
permittedMoves = [
        (0, -4), # MOVE LEFT
        (0, 4),  # MOVE RIGHT
        (2, 0),  # MOVE DOWN
        (-2, 0)  # MOVE UP
    ]

# Main function
def main():
    handleTurn()

# Definition: Handles user input for selecting the maze configuration, selecting the algorithm, keeping track of algorithm times to completion, and printing results. 
# Parameters: No parameters. 
# Returns: No return - displays algorithm times to completion, and completed mazes. 
def handleTurn(): 
    print("Welcome to a learning tool for exploring path finding algorithms. Inspired by Micromouse, this tool will help visualize how a robot can find it's way from the starting point of a maze to the goal.\n")

    # Prompt user to select maze and assign selection to mazeSelection
    mazeSelection = input("Please begin by selecting a maze configuration: Maze 1, Maze 2, or Maze 3. Type 0 to cancel: ")
    
    # handle user input to select maze configuration
    if (int(mazeSelection) == 0):
        gameStop = True
        sys.exit()
    elif (int(mazeSelection) == 1):
        maze = getMaze(mazeFileOne)
    elif (int(mazeSelection) == 2):
        maze = getMaze(mazeFileTwo)
    elif (int(mazeSelection) == 3):
        maze = getMaze(mazeFileThree)
    
    # Declare variables to store time to complete search algorithms
    bfsTime = 0
    astarTime = 0
    
    # boolean to determine if user wants to end game
    gameStop = False

    # iterate until user selects to terminate game
    while(gameStop == False):
        # set start position, start position is the same for every maze 
        startPos = (31, 2)

        # prompt to select algorithm to use
        algorithmSelection = input("Enter a number to select an algorithm...\n1: A* search: \n2: Breadth-First search: \n3: Compare algorithm times: \n0: Stop program execution: \n")

        # handle user input for selecting algorithm

        # if user selects 0, end game 
        if (int(algorithmSelection) == 0):
            gameStop = True
            sys.exit()
        # start search with selected algorithm, measure time to complete search, and print completed maze and finish time. 
        elif (int(algorithmSelection) == 1):
            # for calculating time
            startTime = time.time()

            # calculate path
            path = astar(startPos, maze)

            # print path
            printMaze(maze, path)

            # Output the time to find path
            astarTime = (time.time() - startTime)
            print (f"A* search has finished, time to finish is: {astarTime: .3f}")
        # start search with selected algorithm, measure time to complete search, and print completed maze and finish time. 
        elif (int(algorithmSelection) == 2):        
            # for calculating time
            startTime = time.time()
            
            # calculate path
            path = bfs(startPos, maze)

            # print path
            printMaze(maze, path)

            # Output the time to find path
            bfsTime = (time.time() - startTime)
            print (f"Breadth-first search has finished, time to finish is: {bfsTime: .3f}")
        # Compare times of algorithms
        elif (int(algorithmSelection) == 3):
            print("\nDisplaying times: \n")

            if(astarTime == 0):
                print("A* search time not recorded.")
            else:
                print(f"A* search time: {astarTime: .3f} seconds") 

            if(bfsTime == 0):
               print("BFS search time not recorded.")
            else: 
                print(f"Breadth first search time: {bfsTime: .3f} seconds")

            print("\n")

# Definition: Takes a maze file, converts file into 2D array, and returns array.
# Parameters: takes a text file as input.
# Returns: an array with the converted maze.
def getMaze(mazeFile):
    # initialize empty array 
    parsedMaze = [] 

    # access maze file and read from file
    with open(mazeFile, 'r') as mazeFile:
        # access each row in text file, and then iterate through each element in row and add each element to mazeRow. After all the elements of the row have been 
        # added to mazeRow, append to parsedMaze array and continue to next row in file. 

        # iterate through each line of file
        for eachLine in mazeFile:
            mazeRow = []
            # iterate through each element in line
            for element in eachLine.strip():
                # add element to row
                mazeRow.append(element)

            # add row to parsedMaze
            parsedMaze.append(mazeRow)

    # return parsedMaze array
    return parsedMaze

# Definition: Handles printing maze and pathing.
# Parameters: takes maze array and path array.
# Returns: no return - prints maze and path.
def printMaze(maze, path):
    # if path is empty, don't print anything
    if path is None:
        return

    # create copy of maze so original remains unchanged
    mazeCopy = copy.deepcopy(maze)

    # sleep output for readability
    time.sleep(0.3)
    
    # clears console to improve readability 
    os.system("cls")

    # for each position in the path, check if position is valid
    # if position is invalid: continue to next position in iteration 
    # if position is valid: place an X to represent the path, and that the cell has been visited
    # Note: could replace with a function to check validity
    for position in path:
        if not isValid(position, maze):
            # Skip position and continue to next iteration if invalid position
            continue 
        # if valid, place an X
        mazeCopy[position[0]][position[1]] = 'χ'

     # If path is not empty, get last visited position in path and set it to ● character to represent current position of search
    if path != 0:
        # get last element of list
        lastPos = path[-1]
        # if last position is a valid position, place a character in that positiion to represent active search
        if isValid(lastPos, maze):
            mazeCopy[lastPos[0]][lastPos[1]] = '●'

    # iterate through maze and print maze characters together, row by row
    for row in mazeCopy:
        print((''.join(row)))

# Definition: Helper function for reversing and returning path from start to goal
# Parameters: takes the current position, the start position, and a dictionary that kept track of the path's parent and child nodes
# Returns: returns the path from the start to the goal
def returnPath(current, startPos, cameFromPath):
    # Initialize empty array
    path = []

    # add current to the path. Adds in reverse order from goal to start. 
    while current != startPos:
        # add current to the path. Adds in reverse order from goal to start. 
        path.append(current)

        # update current position by retreving parent position in came from dictionary.
        current = cameFromPath[current]

    # append start position to path list, because while loop will stop iterating 
    # right before start node, meaning it won't be added. 
    path.append(startPos)

    # reverse order of the postions in the path list so that they are from start to goal
    reversedPath = list(reversed(path))

    return reversedPath

# Definition: Calculates the Manhanttan Distance for A* search - algorithm for estimating distance to goal.
# Parameters: takes position of agent in maze as a tuple.
# Returns: integer sum of x and y coordinates.
def manhattan(position): 
    # return sum of x and y coordinates
    return position[0] + position[1]

# Definition: Checks if move violates any constraints of the map. Make sure it stays within map boundaries, doesn't jump over obstacles, and follows only unexplored paths.
# Parameters: takes the move to check, and the maze array.
# Returns: returns True if move is valid, else return false.
def isValid(nextPos, maze): 
    # Ensure next move is within the size / bounds of the maze
    if ( (nextPos[0] >= 0 and nextPos[1] >= 0) and (nextPos[0] < len(maze) and nextPos[1] < len(maze[0])) 
        # Ensure next position isn't a wall, ceiling, or path already travelled
        and (maze[nextPos[0]][nextPos[1]] != '|') and (maze[nextPos[0]][nextPos[1]] != '-') and (maze[nextPos[0]][nextPos[1]] != 'χ') ):
        # if it doesn't violate restrictions: return True
        return True
    # else restriction has been violated: return False
    else: 
        return False
    
# Definition: Check for ceiling within 1 position of current position. We initially check if move is only vertical (ensure column stays the same, and we check 1 row up
#             or 1 rows down. Ensures no ceilings are jumped over. Need this function because agent is designed to move two spaces up or down. Since isValid only checks for 
#             ceilings in the destination of the move, rather than the route to move; this function is required as an additional check.  
# Parameters: Takes the row coordinate, column coordinate, maze, and current position
# Returns: returns False, if no ceilings exists, and vertical move is permitted. Else returns True, signifying move isn't permitted
def checkForCeiling(rowCord, colCord, maze, currentPos):
    # colCord = col coordinate of current move
    # rowCord = row coordinate of current move

    # check if rowCord == 2 since permitted moves only allows for vertical moves up 2 or down 2
    if colCord == 0 and (rowCord == 2 or rowCord == -2):
        ceilingRow = currentPos[0] + int(rowCord / 2) # calculate which row (vertical direction for our purposes) the ceiling is in (up or down) - divides rowCord by 2 since rowCord is restricted to being either 2 
        # -2. To check for ceilings within +1 or -1, divide rowCord by 2.
        ceilingCol = currentPos[1] # keep column (horizontal direction for our purposes) fixed.

        # check if maze at row either up or down has a ceiling, if it does return true, else return false
        if maze[ceilingRow][ceilingCol] == '-':
            return True
        else:
            return False
    else:
        # if no ceiling detected, return False
        return False

# Definition: Check for walls within 2 position of current position. We initially check if move is only horizontal (ensure row stays the same, and we check 2 columns left
#             or 2 columns right. Ensures no walls are jumped over. Need this function because agent is designed to move two spaces left or right. Since isValid only checks for 
#             walls in the destination of the move, rather than the route to move; this function is required as an additional check.  
# Parameters: Takes the row coordinate, column coordinate, maze, and current position
# Returns: returns False, if no wall exists, and horizontal move is permitted. Else returns True, signifying move isn't permitted
def checkForWall(rowCord, colCord, maze, currentPos):
    # colCord = col coordinate of current move
    # rowCord = row coordinate of current move

    # check if colCord == 4 or -4 since permitted moves only allows for moves right or left by 4 or -4 
    if rowCord == 0 and (colCord == 4 or colCord == -4):
        ceilingRow = currentPos[0] # keep row (vertical direction for our purposes) fixed.
        ceilingCol = currentPos[1] + int(colCord / 2) # calculate which column (horizontal direction for our purposes) the ceiling is in (left or right) - divides colCord by 2 since colCord is restricted to being 
        # either 4 or -4. To check for walls within +2 or -2, divide colCord by 2. 

        # check if maze at column either left or right has a ceiling, if it does return true, else return false
        if maze[ceilingRow][ceilingCol] == '|':
            return True
        else: 
            return False
    else: 
        # if no wall detected, return false
        return False

# Definition: Applies the A* search algorithm to find the goal of a maze. 
# Parameters: Takes the starting position of the maze, represented as a coordinate tuple (x, y) and the maze represented as a 2d array
# Returns: returns a path from the start to the goal of the maze. Prints the maze at each iteration in the search algorithm's exploration of the maze.
def astar(startPos, maze):
    # Initialize empty array to represent frontier 
    # Empty array to store elements in form of tuples where each element is represented as position of node and cost/priority of node
    frontier = []

    # Initialize heapq to add node to frontier array with a cost and position. The lower
    # the value, i.e. cost, means it will be explored earlier. 
    heapq.heappush(frontier, (0, startPos))

    # Came from is a dictionary that keeps tracks of the path the mouse came from. 
    # Will store both the parent nodes and child nodes, where the child's position is the key, and the parent's position is the value in the key/value pair.
    # ex: (31, 2) : None
    #     (29, 2) : (31, 2)
    pathCameFrom = {}
    pathCameFrom[startPos] = None

    # Cost so far is a dictionary that keeps track of the current cost from the start position to the current position. 
    # Will store next position's coordinate as the key and the new cost to reach that position as the value. 
    # ex: (31, 2) : 0
    #     (29, 2) : 2
    # Used to calculate manhattan distance and in calculating optimal path. 
    costToReachPos = {}
    costToReachPos[startPos] = 0

    # Iterate while nodes are still in the frontier
    while frontier != 0:
        # Return the lowest cost node (highest priority node) to be explored next, and assign it to current variable.
        # heapq.heappop(frontier)[1] indexes to position in frontier, where heapq.heappop(frontier)[0] indexes to to the priority
        currentPos = heapq.heappop(frontier)[1]

        # Check current position in maze and see if the value contained in that position is the goal 'G'
        # If true, stop iterating and return path.
        if maze[currentPos[0]][currentPos[1]] == 'G': 
            break
        
        # Iterate through permitted moves
        for move in permittedMoves:
            # Calculate the next position by taking the current position and adding the x and y moves to that position. 
            nextPos = (currentPos[0] + move[0], currentPos[1] + move[1])
            
            # Check if next position is a valid move
            if isValid(nextPos, maze):
                # If the next position is a valid move, increase the cost of the current path.
                newCost = costToReachPos[currentPos] + 1
                
                # Verify that the selected move doesn't jump over a wall or ceiling in the maze.
                if checkForWall(move[0], move[1], maze, currentPos) or checkForCeiling(move[0], move[1], maze, currentPos):
                    continue
                
                # Check if next position hasn't been reached before or if the new cost of reaching the position is lower than the cost so far for the position
                if nextPos not in costToReachPos or newCost < costToReachPos[nextPos]:
                    # If position is valuable to explore: Update the cost so far, and push the next position onto the frontier of the search. 

                    # Update the new cost of reaching the next position to the the cost so far dictionary
                    costToReachPos[nextPos] = newCost

                    # Calculate priority of next position by calculating the sum of the Manhattan distance and new cost to reaching the position
                    priority = manhattan(nextPos) + newCost

                    # Add the next position and its priority to the frontier. Heapq push will ensure the elements are properly ordered in terms of their priority. 
                    heapq.heappush(frontier, (priority, nextPos))

                    # Update came from dictionary where the current position is recorded as the previous position of next position. The next position is the position currently being explored.
                    # Current represents the position from which the algorithm arrived at next position. Current is the parent of the next position of the path. 
                    # ex: if         current = (31, 2)
                    #     then next position = (29, 2)
                    # Meaning next position was reached by moving current to next position 
                    pathCameFrom[nextPos] = currentPos


        # get list of path positions from dictionary
        pathKey = [pos for pos in pathCameFrom]

        # print maze and pass it the path came from to all the positions that the mouse has explored 
        # printMaze(maze, list(pathCameFrom.keys()))
        printMaze(maze, pathKey)

        # Introduce a delay for visualization
        time.sleep(0.0)

    # Backtrack from goal to start to reconstruct path found by A* algorithm
    path = returnPath(currentPos, startPos, pathCameFrom)
    
    # return path to goal
    return path

# Definition: Applies the Breadth-first search algorithm to find the goal of the maze.
# Parameters: Takes the starting position of the maze, represented as a coordinate tuple (x, y) and the maze represented as a 2d array
# Returns: returns a path from the start to the goal of the maze. Prints the maze at each iteration in the search algorithm's exploration of the maze.
def bfs(start, maze):
    # Initialize reached as an array to represent the nodes we've reached throughout the search. 
    reachedCells = []

    # create a double ended queue for the nodes at the frontier 
    # deque allows quicker access to the queue's elements on each end
    frontierQueue = deque() 

    # Add starting position to the frontier
    frontierQueue.append(start) 
    # Add starting position to reached
    reachedCells.append(start)

    # Create a dictionary to keep track of parent positions
    parentChildPath = {}
    # Starting position doesn't have a parent, set to None
    parentChildPath[start] = None

    # loop through the frontier until no more nodes are at the frontier, i.e reached maxed depth
    while frontierQueue != 0:
        # let current be equal to the first node in the queue
        currentPos = frontierQueue.popleft()
   
        # Iterate through the permitted movements
        for move in permittedMoves:
            # A move in permittedMoves will be selected from which we can calculate the new position of the search 

            # Take current position and assign new position based on selected move. 
            # i.e. Takes current row and add row movement to current row
            # i.e. Takes current column and adds column movement to current column
            newPos = (currentPos[0] + move[0], currentPos[1] + move[1])

            # check if the move is valid, and that it doesn't hit a position that we've already reached
            if isValid(newPos, maze) and (newPos not in reachedCells):
                # if it's valid: calculate the actual movement of the move on both row and col

                # Determine actual number of rows and columns to move
                rowsToMove = newPos[0] - currentPos[0]
                colsToMove = newPos[1] - currentPos[1]

                # Verify that the selected move doesn't jump over a wall or ceiling in the maze.
                if not ( checkForWall(rowsToMove, colsToMove, maze, currentPos) ) and not ( checkForCeiling(rowsToMove, colsToMove, maze, currentPos) ):
                    # if it's valid:
                    # add the position to reached nodes
                    reachedCells.append(newPos)
                    # add position to frontier to continue exploration if needed
                    frontierQueue.append(newPos)
                    # let current be the parent of the new position 
                    parentChildPath[newPos] = currentPos
        
        # print the maze at each iteration in the while loop
        printMaze(maze, reachedCells)
        print("\n")

        # Introduce a delay for visualization
        time.sleep(0.0)

        # check if we've reached the goal 'G' at each iteration of the exploration. 
        if maze[currentPos[0]][currentPos[1]] == 'G':
            # Calculate path to goal, and return it
            path = returnPath(currentPos, start, parentChildPath)
            return path 

    # if depth is reached and destination is not reached, print no path
    print("No path")

if __name__ == '__main__': 
    main()