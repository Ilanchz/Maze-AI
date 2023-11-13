from tkinter import *
import heapq
from queue import PriorityQueue
import math

root=Tk()
root.title("Maze Animation")
root.geometry("800x800")
frame=Frame(root)
frame.grid(row=0,column=0,columnspan=3)
import time
from collections import deque
from tkinter import *
time_to_sleep=0.05

maze = [
['1', '0', '1', '0', '1', '1', '0', '1', '0', '1', '0', '1', '1', '1', '0', '0', '1', '0', '1', '0', '0', '1', '0', '1', '1', '1', '1', '1', '0', '0'],
['1', '0', '1', '1', '0', '1', '1', '1', '1', '1', '1', '0', '1', '1', '1', '1', '0', '1', '0', '0', '1', '0', '1', '1', '0', '1', '1', '0', '1', '1'],
['1', '1', '0', '0', '0', '1', '0', '1', '0', '0', '0', '0', '0', '0', '1', '0', '1', '0', '0', '0', '1', '1', '0', '1', '0', '1', '1', '0', '1', '1'],
['1', '0', '1', '0', '0', '1', '0', '0', '0', '0', '0', '0', '1', '0', '0', '0', '1', '0', '0', '1', '0', '0', '1', '0', '1', '0', '1', '1', '1', '0'],
['1', '0', '0', 'S', '1', '1', '1', '0', '1', '1', '1', '1', '1', '1', '1', '0', '0', '0', '1', '0', '1', '0', '0', '1', '1', '1', '0', '0', '1', '1'],
['1', '1', '1', '0', '0', '0', '0', '0', '1', '1', '0', '0', '1', '1', '1', '1', '1', '0', '0', '0', '1', '1', '0', '1', '0', '1', '1', '1', '0', '1'],
['0', '0', '0', '1', '0', '0', '0', '1', '1', '0', '0', '1', '1', '1', '1', '1', '1', '0', '0', '0', '1', '1', '1', '1', '1', '0', '0', '0', '0', '0'],
['1', '0', '0', '0', '0', '1', '1', '1', '1', '0', '0', '0', '1', '0', '0', '0', '0', '0', '1', '0', '0', '0', '0', '1', '0', '1', '1', '1', '0', '0'],
['1', '0', '0', '1', '0', '1', '0', '1', '0', '0', '0', '1', '1', '0', '1', '1', '1', '1', '0', '0', '0', '1', '0', '1', '1', '1', '0', '0', '0', '0'],
['0', '0', '0', '0', '0', '0', '1', '0', '0', '0', '0', '1', '1', '0', '0', '0', '0', '0', '0', '0', '1', '1', '0', '0', '0', '0', '0', '1', '0', '0'],
['1', '1', '1', '0', '0', '1', '1', '0', '0', '1', '0', '0', '1', '1', '0', '0', '0', '1', '1', '1', '1', '1', '1', '0', '1', '1', '1', '0', '1', '1'],
['0', '0', '1', '1', '0', '1', '0', '0', '1', '0', '0', '0', '0', '0', '1', '0', '0', '1', '0', '1', '0', '0', '1', '0', '0', '1', '0', '1', '0', '0'],
['0', '0', '1', '1', '0', '1', '1', '0', '0', '1', '1', '0', '0', '0', '0', '0', '1', '1', '1', '1', '0', '1', '1', '0', '1', '1', '0', '0', '1', '1'],
['0', '0', '0', '1', '0', '0', '0', '1', '0', '0', '0', '1', '0', '1', '1', '0', '1', '0', '0', '1', '1', '1', '0', '1', '0', '1', '0', '0', '1', '1'],
['1', '0', '1', '0', '1', '1', '0', '1', '1', '0', '0', '1', '0', '1', '0', '0', '0', '0', '1', '1', '0', '1', '1', '0', '1', '1', '0', '0', '0', '0'],
['0', '1', '0', '0', '1', '1', '0', '0', '1', '1', '0', '1', '1', '0', '1', '1', '0', '1', '0', '0', '0', '0', '0', '1', '1', '1', '1', '0', '1', '1'],
['0', '0', '1', '1', '1', '0', '0', '0', '1', '0', '0', '1', '1', '1', '0', '1', '0', '0', '0', '1', '1', '1', '0', '1', '0', '1', '1', '1', '1', '0'],
['0', '0', '1', '1', '0', '0', '0', '1', '1', '1', '0', '0', '0', '0', '1', '1', '1', '1', '1', '1', '1', '0', '0', '1', '0', '0', '0', '0', '0', '0'],
['0', '0', '0', '0', '0', '1', '1', '1', '1', '1', '1', '0', '0', '1', '1', '1', '0', '0', '1', '0', '0', '0', '1', '0', '0', '0', '1', '0', '0', '0'],
['1', '1', '0', '0', '0', '0', '1', '1', '1', '0', '1', '1', '1', '1', '0', '1', '1', '1', '0', '0', '0', '1', '0', '1', '1', '1', '0', '1', '1', '0'],
['1', '0', '0', '0', '0', '1', '0', '0', '0', '1', '0', '0', '1', '0', '1', '1', '0', '0', '1', '1', '0', '0', '1', '1', '1', '1', '1', '1', '0', '1'],
['0', '0', '0', '0', '1', '0', '1', '0', '0', '1', '0', '1', '1', '1', '1', '1', '1', '0', '1', '0', '0', '1', '0', '0', '0', '0', '0', '1', '0', '0'],
['0', '0', '0', '1', '1', '1', '0', '1', '0', '1', '0', '0', '1', '0', '1', '0', '1', '0', '1', '0', '1', '0', '0', '1', '1', '1', '0', '1', '0', '0'],
['1', '0', '0', '1', '1', '0', '0', '1', '0', '1', '1', '1', '1', '1', '0', '1', '1', '0', '1', '1', '1', '1', '1', '0', '0', '0', '0', '0', '1', '1'],
['1', '1', '0', '0', '0', '0', '1', '0', '1', '0', '0', '1', '0', '1', '1', '0', '0', '1', '1', '1', '0', '0', '0', '0', '1', '0', '1', '1', '0', '0'],
['1', '1', '1', '0', '1', '0', '0', '0', '0', '0', '0', '0', '0', '1', '1', '1', '1', '0', '0', '1', '1', '0', '1', '1', '1', '1', '0', '1', '1', '1'],
['1', '0', '1', '1', '1', '0', '0', '1', '0', '1', '1', '1', '0', '1', '0', '0', '0', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '1', '1'],
['0', '0', '0', '1', '0', '1', '0', '0', '0', '1', '0', '1', '0', '1', '1', '1', '0', '0', '0', '1', '0', '1', '0', '1', '0', '0', '0', '0', '0', '0'],
['1', '1', '0', '0', '1', '0', '1', '1', '1', '0', '1', '0', '0', '0', '0', '0', '0', '1', '0', '1', '0', '1', '0', '1', '0', '0', '0', '1', '0', 'E'],
['0', '0', '0', '1', '0', '1', '0', '1', '0', '1', '1', '0', '0', '1', '0', '0', '0', '0', '1', '1', '1', '0', '1', '1', '1', '1', '0', '0', '0', '1']
]
givenMaze=[
['1', '0', '1', '0', '1', '1', '0', '1', '0', '1', '0', '1', '1', '1', '0', '0', '1', '0', '1', '0', '0', '1', '0', '1', '1', '1', '1', '1', '0', '0'],
['1', '0', '1', '1', '0', '1', '1', '1', '1', '1', '1', '0', '1', '1', '1', '1', '0', '1', '0', '0', '1', '0', '1', '1', '0', '1', '1', '0', '1', '1'],
['1', '1', '0', '0', '0', '1', '0', '1', '0', '0', '0', '0', '0', '0', '1', '0', '1', '0', '0', '0', '1', '1', '0', '1', '0', '1', '1', '0', '1', '1'],
['1', '0', '1', '0', '0', '1', '0', '0', '0', '0', '0', '0', '1', '0', '0', '0', '1', '0', '0', '1', '0', '0', '1', '0', '1', '0', '1', '1', '1', '0'],
['1', '0', '0', 'S', '1', '1', '1', '0', '1', '1', '1', '1', '1', '1', '1', '0', '0', '0', '1', '0', '1', '0', '0', '1', '1', '1', '0', '0', '1', '1'],
['1', '1', '1', '0', '0', '0', '0', '0', '1', '1', '0', '0', '1', '1', '1', '1', '1', '0', '0', '0', '1', '1', '0', '1', '0', '1', '1', '1', '0', '1'],
['0', '0', '0', '1', '0', '0', '0', '1', '1', '0', '0', '1', '1', '1', '1', '1', '1', '0', '0', '0', '1', '1', '1', '1', '1', '0', '0', '0', '0', '0'],
['1', '0', '0', '0', '0', '1', '1', '1', '1', '0', '0', '0', '1', '0', '0', '0', '0', '0', '1', '0', '0', '0', '0', '1', '0', '1', '1', '1', '0', '0'],
['1', '0', '0', '1', '0', '1', '0', '1', '0', '0', '0', '1', '1', '0', '1', '1', '1', '1', '0', '0', '0', '1', '0', '1', '1', '1', '0', '0', '0', '0'],
['0', '0', '0', '0', '0', '0', '1', '0', '0', '0', '0', '1', '1', '0', '0', '0', '0', '0', '0', '0', '1', '1', '0', '0', '0', '0', '0', '1', '0', '0'],
['1', '1', '1', '0', '0', '1', '1', '0', '0', '1', '0', '0', '1', '1', '0', '0', '0', '1', '1', '1', '1', '1', '1', '0', '1', '1', '1', '0', '1', '1'],
['0', '0', '1', '1', '0', '1', '0', '0', '1', '0', '0', '0', '0', '0', '1', '0', '0', '1', '0', '1', '0', '0', '1', '0', '0', '1', '0', '1', '0', '0'],
['0', '0', '1', '1', '0', '1', '1', '0', '0', '1', '1', '0', '0', '0', '0', '0', '1', '1', '1', '1', '0', '1', '1', '0', '1', '1', '0', '0', '1', '1'],
['0', '0', '0', '1', '0', '0', '0', '1', '0', '0', '0', '1', '0', '1', '1', '0', '1', '0', '0', '1', '1', '1', '0', '1', '0', '1', '0', '0', '1', '1'],
['1', '0', '1', '0', '1', '1', '0', '1', '1', '0', '0', '1', '0', '1', '0', '0', '0', '0', '1', '1', '0', '1', '1', '0', '1', '1', '0', '0', '0', '0'],
['0', '1', '0', '0', '1', '1', '0', '0', '1', '1', '0', '1', '1', '0', '1', '1', '0', '1', '0', '0', '0', '0', '0', '1', '1', '1', '1', '0', '1', '1'],
['0', '0', '1', '1', '1', '0', '0', '0', '1', '0', '0', '1', '1', '1', '0', '1', '0', '0', '0', '1', '1', '1', '0', '1', '0', '1', '1', '1', '1', '0'],
['0', '0', '1', '1', '0', '0', '0', '1', '1', '1', '0', '0', '0', '0', '1', '1', '1', '1', '1', '1', '1', '0', '0', '1', '0', '0', '0', '0', '0', '0'],
['0', '0', '0', '0', '0', '1', '1', '1', '1', '1', '1', '0', '0', '1', '1', '1', '0', '0', '1', '0', '0', '0', '1', '0', '0', '0', '1', '0', '0', '0'],
['1', '1', '0', '0', '0', '0', '1', '1', '1', '0', '1', '1', '1', '1', '0', '1', '1', '1', '0', '0', '0', '1', '0', '1', '1', '1', '0', '1', '1', '0'],
['1', '0', '0', '0', '0', '1', '0', '0', '0', '1', '0', '0', '1', '0', '1', '1', '0', '0', '1', '1', '0', '0', '1', '1', '1', '1', '1', '1', '0', '1'],
['0', '0', '0', '0', '1', '0', '1', '0', '0', '1', '0', '1', '1', '1', '1', '1', '1', '0', '1', '0', '0', '1', '0', '0', '0', '0', '0', '1', '0', '0'],
['0', '0', '0', '1', '1', '1', '0', '1', '0', '1', '0', '0', '1', '0', '1', '0', '1', '0', '1', '0', '1', '0', '0', '1', '1', '1', '0', '1', '0', '0'],
['1', '0', '0', '1', '1', '0', '0', '1', '0', '1', '1', '1', '1', '1', '0', '1', '1', '0', '1', '1', '1', '1', '1', '0', '0', '0', '0', '0', '1', '1'],
['1', '1', '0', '0', '0', '0', '1', '0', '1', '0', '0', '1', '0', '1', '1', '0', '0', '1', '1', '1', '0', '0', '0', '0', '1', '0', '1', '1', '0', '0'],
['1', '1', '1', '0', '1', '0', '0', '0', '0', '0', '0', '0', '0', '1', '1', '1', '1', '0', '0', '1', '1', '0', '1', '1', '1', '1', '0', '1', '1', '1'],
['1', '0', '1', '1', '1', '0', '0', '1', '0', '1', '1', '1', '0', '1', '0', '0', '0', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '1', '1'],
['0', '0', '0', '1', '0', '1', '0', '0', '0', '1', '0', '1', '0', '1', '1', '1', '0', '0', '0', '1', '0', '1', '0', '1', '0', '0', '0', '0', '0', '0'],
['1', '1', '0', '0', '1', '0', '1', '1', '1', '0', '1', '0', '0', '0', '0', '0', '0', '1', '0', '1', '0', '1', '0', '1', '0', '0', '0', '1', '0', 'E'],
['0', '0', '0', '1', '0', '1', '0', '1', '0', '1', '1', '0', '0', '1', '0', '0', '0', '0', '1', '1', '1', '0', '1', '1', '1', '1', '0', '0', '0', '1']
]

def refreshMazeArray():
    import copy
    global maze
    maze = copy.deepcopy(givenMaze)

def makeMaze():
    global labels

# Define the dimensions of the grid
    m, n = len(maze),len(maze[0])

    # Create a 2D list to hold the labels
    labels = [[None for _ in range(n)] for _ in range(m)]

    # Create and arrange the labels in a grid
    for i in range(m):
        for j in range(n):
            label = Label(frame, text=f" ", width=4, height=2, bg='white',font="times 5 bold")
            label.grid(row=i, column=j)
            labels[i][j] = label

    # Change the color of a label to black Initalise Maze

    for i in range(len(maze)):
        for j in range(len(maze[i])):
            if maze[i][j]=="1":
                labels[i][j].config(bg="black")
            if maze[i][j]=="S":
                labels[i][j].config(bg="red")
                labels[i][j].config(text="S")
            if maze[i][j]=="E":
                labels[i][j].config(bg="green")
                labels[i][j].config(text="E")
            if maze[i][j]=="0":
                labels[i][j].config(bg="white")
        
makeMaze()
# Start the main event loop





def FloodButton():
    refreshMazeArray() 
    makeMaze()
    def flood_fill(maze, x, y, path):
        if x < 0 or x >= len(maze) or y < 0 or y >= len(maze[0]):
            return False

        if maze[x][y] == 'E':
            path.append((x, y))
            return True

        if maze[x][y] == '1' or maze[x][y] == 'V':
            return False

        maze[x][y] = 'V'  # Mark as visited
        labels[x][y].config(bg="blue")
        time.sleep(time_to_sleep)
        root.update()

        if (flood_fill(maze, x - 1, y, path) or
            flood_fill(maze, x + 1, y, path) or
            flood_fill(maze, x, y - 1, path) or
            flood_fill(maze, x, y + 1, path)):
            path.append((x, y))
            return True

        return False

    def find_path1(maze):
        for i in range(len(maze)):
            for j in range(len(maze[0])):
                if maze[i][j] == 'S':
                    path = []
                    if flood_fill(maze, i, j, path):
                        path.reverse()  # Reverse the path so it goes from start to end
                        return True, path
        return False, []
    
    start=time.time()
    found, path = find_path1(maze)
    if (found):
        end=time.time()
        print("Time Taken: ",end-start)
        print("Path found using Flood Fill DFS!")
        ShowPath(path)
    else:
        print("No path found!")

def BFSButton():
    refreshMazeArray()
    makeMaze()
    root.update()

    from collections import deque

    def bfs(maze, start, end):
        queue = deque()
        visited = set()
        predecessors = {}

        queue.append(start)
        visited.add(start)

        while queue:
            x, y = queue.popleft()
            labels[x][y].config(bg="blue")
            root.update()
            time.sleep(time_to_sleep)
            if (x, y) == end:
                path = []
                while (x, y) != start:
                    path.append((x, y))
                    x, y = predecessors[(x, y)]
                path.append(start)
                path.reverse()
                return True, path

            moves = [(1, 0), (-1, 0), (0, 1), (0, -1)]

            for dx, dy in moves:
                nx, ny = x + dx, y + dy

                if 0 <= nx < len(maze) and 0 <= ny < len(maze[0]) and (maze[nx][ny] == '0' or maze[nx][ny]=="E") and (nx, ny) not in visited:
                    queue.append((nx, ny))
                    visited.add((nx, ny))
                    predecessors[(nx, ny)] = (x, y)

        return False, []

    def find_path_bfs(maze):
        start = None
        end = None

        for i in range(len(maze)):
            for j in range(len(maze[0])):
                if maze[i][j] == 'S':
                    start = (i, j)
                if maze[i][j] == 'E':
                    end = (i, j)

        if start is None or end is None:
            return False

        return bfs(maze, start, end)



    start=time.time()

    for i in maze:
        print(i)
    found,path=find_path_bfs(maze)
    if found:
        end=time.time()
        print("Time Taken for BFS: ",end-start)
        print("Path found using BFS!")
        ShowPath(path)
    else:
        print("No path found using BFS!")




def heuristic(a, b):
    return abs((math.sqrt((a[0]-b[0])**2+(a[1]-b[1])**2)))                 #Euclidean distance
    #return abs(b[0] - a[0]) + abs(b[1] - a[1])                            #Manhattan distance 

def a_star_search(maze, start, goal):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    
    while not frontier.empty():
        current = frontier.get()

        labels[current[0]][current[-1]].config(bg="blue")
        root.update()

        time.sleep(time_to_sleep)
        if current == goal:
            break
        
        for next in neighbors(maze, current):
            new_cost = cost_so_far[current] + 1
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                came_from[next] = current
    
    return came_from, cost_so_far

def neighbors(maze, node):
    dirs = [[1, 0], [0, 1], [-1, 0], [0, -1]]
    result = []
    for dir in dirs:
        neighbor = (node[0] + dir[0], node[1] + dir[1])  # Use a tuple instead of a list
        if neighbor[0] >= 0 and neighbor[1] >= 0 and neighbor[0] < len(maze) and neighbor[1] < len(maze[0]):
            if maze[neighbor[0]][neighbor[1]] != '1':
                result.append(neighbor)
    return result

def A_StarButton():
    refreshMazeArray() 
    makeMaze()
    start = None
    goal = None
    
    for i in range(len(maze)):
        for j in range(len(maze[i])):
            if maze[i][j] == 'S':
                start = (i, j)  # Use a tuple instead of a list
            elif maze[i][j] == 'E':
                goal = (i, j)  # Use a tuple instead of a list
    
    came_from, cost_so_far = a_star_search(maze, start, goal)
    
    # Print the path
    current = goal
    path = []
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)
    
    path.reverse()

    ShowPath(path)
    
    '''
    
    for i in path:
        labels[i[0]][i[-1]].config(bg="blue")
        root.update()
        time.sleep(time_to_sleep)
    '''

def ShowPath(path):
    time.sleep(1)
    refreshMazeArray() 
    makeMaze()
    for i in path:
        labels[i[0]][i[-1]].config(bg="orange")
        root.update()
        time.sleep(time_to_sleep)



button1=Button(root,text="Flood It!",command=FloodButton,font="Rockwell 10 bold")
button1.grid(row=1,column=0)
button2=Button(root,text="BFS it!",command=BFSButton,font="Rockwell 10 bold")
button2.grid(row=1,column=1)
button3=Button(root,text="Clean Maze!",command=makeMaze,font="Rockwell 10 bold")
button3.grid(row=1,column=2)
button4 = Button(root, text="A* Star", command=A_StarButton, font="Rockwell 10 bold")
button4.grid(row=1, column=3)

root.mainloop()


