'''
Code for set 1: Dijkstra's algorithm and A*.
Lorenzo Shaikewitz, ME 133b
'''
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time

# Define the possible status levels for each state.
WALL      = 0
UNKNOWN   = 1
ONDECK    = 2
CURRENT   = 3
PROCESSED = 4
PATH = 5

display_rate = 20.0
debug = False

def showgrid_debug(state):
    if not debug:
        return
    # Grab the dimensions.
    M = np.size(state, axis=0)
    N = np.size(state, axis=1)

    # Close the old figure.
    plt.close()

    # Create the figure and axes.
    fig = plt.figure()
    ax  = plt.axes()

    # turn off the axis labels
    ax.axis('off')

    # Draw the grid, zorder 1 means draw after zorder 0 elements.
    for m in range(M+1):
        ax.axhline(m, lw=1, color='b', zorder=1)
    for n in range(N+1):
        ax.axvline(n, lw=1, color='b', zorder=1)

    # Create the color range.  There are clearly more elegant ways...
    color = np.ones((M,N,3))
    for m in range(M):
        for n in range(N):
            if   state[m,n] == WALL:
                color[m,n,0:3] = np.array([0.0, 0.0, 0.0])   # Black
            elif state[m,n] == UNKNOWN:
                color[m,n,0:3] = np.array([1.0, 1.0, 1.0])   # White
            elif state[m,n] == ONDECK:
                color[m,n,0:3] = np.array([0.0, 1.0, 0.0])   # Green
            elif state[m,n] == CURRENT:
                color[m,n,0:3] = np.array([0.1059, 0.4196, 0.0353])   # Dark Green
            elif state[m,n] == PROCESSED:
                color[m,n,0:3] = np.array([0.0, 0.0, 1.0])   # Blue
            elif state[m,n] == PATH:
                color[m,n,0:3] = np.array([1.0, 0.8745, 0.0510])   # Yellow!
            else:
                color[m,n,0:3] = np.array([1.0, 0.0, 0.0])   # Red

    # Draw the boxes
    ax.imshow(color, aspect='equal', interpolation='none',
              extent=[0, N, 0, M], zorder=0)

    # Force the figure to pop up.
    plt.pause(0.001)

######################################################################
#
#   showgrid(M,N)
#
#   Create a figure for an M (rows) x N (column) grid.  The X-axis
#   will be the columns (to the right) and the Y-axis will be the rows
#   (top downward).
#
def showgrid(state, ax):

    # Grab the dimensions.
    M = np.size(state, axis=0)
    N = np.size(state, axis=1)

    # Draw the grid, zorder 1 means draw after zorder 0 elements.
    # for m in range(M+1):
    #     ax.axhline(m, lw=1, color='b', zorder=1)
    # for n in range(N+1):
    #     ax.axvline(n, lw=1, color='b', zorder=1)

    # Create the color range.  There are clearly more elegant ways...
    color = np.ones((M,N,3))
    for m in range(M):
        for n in range(N):
            if   state[m,n] == WALL:
                color[m,n,0:3] = np.array([0.0, 0.0, 0.0])   # Black
            elif state[m,n] == UNKNOWN:
                color[m,n,0:3] = np.array([1.0, 1.0, 1.0])   # White
            elif state[m,n] == ONDECK:
                color[m,n,0:3] = np.array([0.0, 1.0, 0.0])   # Green
            elif state[m,n] == CURRENT:
                color[m,n,0:3] = np.array([0.1059, 0.4196, 0.0353])   # Dark Green
            elif state[m,n] == PROCESSED:
                color[m,n,0:3] = np.array([0.0, 0.0, 1.0])   # Blue
            elif state[m,n] == PATH:
                color[m,n,0:3] = np.array([1.0, 0.8745, 0.0510])   # Yellow!
            else:
                color[m,n,0:3] = np.array([1.0, 0.0, 0.0])   # Red

    # Draw the boxes
    axim = ax.imshow(color, aspect='equal', interpolation='none',
              extent=[0, N, 0, M], zorder=0)
    return [axim]

class Node:
    '''coords: (x, y) tuple that describes node position
       cost: cost to reach this node
       prev: node that led to this node.'''
    def __init__(self, coords, prev, cost = None):
        self.coords = coords
        self.prev = prev
        if cost is not None:
            self.cost = cost
        else:
            self.cost = self.prev.cost + 1.0

class Astar:
    '''
    empty_map: map will walls marked, in array form
    start: coords in array of starting position
    goal: coords in array of goal position
    cost_predictor: function to estimate cost to real goal from position
    '''
    def __init__(self, empty_map, start, goal, cost_predictor):
        self.map = empty_map.copy()

        self.start = start
        self.goal = goal
        self.cost_predictor = cost_predictor

        # setup visited and unvisited node lists
        self.visited = []
        self.queue = []
        # add start as the first node
        self.current_node = Node(self.start, None, 0.0)
        self.map[self.current_node.coords] = CURRENT

        self.process_animation = np.expand_dims(empty_map,axis=2)
        self.path_animation = np.expand_dims(empty_map,axis=2)

        nopath = False

        num_states = 0

        # repeat until we reach the goal or queue is empty
        while (self.current_node.coords != self.goal):
            num_states += 1
            self.add_neighbors()
            # make sure the queue still has stuff
            if (len(self.queue) == 0):
                # no path!
                print("No path from start to end found. :(")
                nopath = True
                break;
            else:
                # close out the current node
                self.map[self.current_node.coords] = PROCESSED
                self.visited.append(self.current_node)

                # sort the queue by cost + predicted cost to go to destination
                self.queue.sort(key=lambda x: (x.cost + cost_predictor(x.coords, self.goal)))
                # pull out the lowest cost
                self.current_node = self.queue.pop(0)
                self.map[self.current_node.coords] = CURRENT
            # show on plot
            showgrid_debug(self.map)
            self.process_animation = np.dstack((self.process_animation, self.map))

        # the goal has been reached, or we've hit the end and there's no path.
        if not nopath:
            print("Dist:\t" + str(self.current_node.cost))
            print("States:\t" + str(num_states))
            # animate path return
            n = self.current_node
            while n.coords != self.start:
                self.map[n.coords] = PATH
                showgrid_debug(self.map)
                self.path_animation = np.dstack((self.path_animation, self.map))
                n = n.prev
            self.map[n.coords] = PATH
            showgrid_debug(self.map)
            self.path_animation = np.dstack((self.path_animation, self.map))

    '''
    Explores all neighbors of current node, adding them to the queue if they
    are unvisited/unknown.
    '''
    def add_neighbors(self):
        # identify neighbors that haven't been visited/are not walls/out of array
        coords = self.current_node.coords
        check_increments = [(1,0), (0,1), (-1,0), (0,-1)]
        for c in check_increments:
            c = (coords[0] + c[0], coords[1] + c[1])
            # check if still in the matrix
            if not (c[0] >= self.map.shape[0]) or \
                   (c[1] >= self.map.shape[1]) or \
                   (c[0] <= 0) or \
                   (c[1] <= 0):
               # we're good to add spatially. Now check if wall or visited (or ondeck)
               if (self.map[c] == UNKNOWN):
                   self.queue.append(Node(c, self.current_node))
                   self.map[c] = ONDECK
               # for now we don't need to worry about on deck, since cost is always 1
               # so if it is already on deck, it will already be optimal cost

    def animate(self, t, ax):
        if (t >= self.process_animation.shape[2]):
            t = t - self.process_animation.shape[2]
            return showgrid(self.path_animation[:,:,t], ax)
        else:
            return showgrid(self.process_animation[:,:,t], ax)



######################################################################
#
#   Main Code
#
if __name__ == "__main__":
    # Define the grid with unknown states.
    M = 11
    N = 17

    state = np.ones((M,N)) * UNKNOWN

    # Populate the states.
    state[ 0,0:] = WALL
    state[-1,0:] = WALL
    state[0:, 0] = WALL
    state[0:,-1] = WALL

    state[3, 4:10] = WALL
    state[4,   10] = WALL
    state[5,   11] = WALL
    state[6,   12] = WALL
    state[7,   13] = WALL
    state[7:M,  7] = WALL

    # Update/show the grid and show the S/G states labelled.
    # Close the old figure.
    plt.close()
    # Create the figure and axes.
    fig = plt.figure()
    ax  = plt.axes()
    # turn off the axis labels
    ax.axis('off')

    # show the starting condition
    showgrid(state, ax)
    plt.pause(0.001)
    input('Hit return to begin!')

    # PART A: Classic Dijkstra
    print("Part a: Dijkstra")
    dij = Astar(state, (5,4), (5,12), lambda x,y: 0.0)

    process_frames = dij.process_animation.shape[2]
    path_frames = dij.path_animation.shape[2]
    total_frames = process_frames + path_frames
    ani = animation.FuncAnimation(fig, dij.animate, frames=total_frames, interval=display_rate, fargs=(ax,), blit = True, repeat=False)
    plt.show(block=False)
    input()
    # ani.save('dijkstra.gif', fps=int(total_frames*display_rate))


    # Create the figure and axes.
    fig = plt.figure()
    ax  = plt.axes()
    # turn off the axis labels
    ax.axis('off')

    showgrid(state, ax)
    plt.pause(0.001)
    # PART B: Manhattan A*
    print("Part b: Manhattan A*")
    man_a = Astar(state, (5,4), (5,12), lambda x,y: abs(x[0] - y[0]) + abs(x[1] - y[1]))

    process_frames = man_a.process_animation.shape[2]
    path_frames = man_a.path_animation.shape[2]
    total_frames = process_frames + path_frames
    ani = animation.FuncAnimation(fig, man_a.animate, frames=total_frames, interval=20, fargs=(ax,), blit = True, repeat=False)
    plt.show(block=False)
    input()
    # ani.save('astar_man.gif', fps=int(total_frames*display_rate))


    # Create the figure and axes.
    fig = plt.figure()
    ax  = plt.axes()
    # turn off the axis labels
    ax.axis('off')

    showgrid(state, ax)
    plt.pause(0.001)
    # PART C: Manhattan*2 A*
    print("Part c: 2*Manhattan A*")
    man2_a = Astar(state, (5,4), (5,12), lambda x,y: 2*(abs(x[0] - y[0]) + abs(x[1] - y[1])))

    process_frames = man2_a.process_animation.shape[2]
    path_frames = man2_a.path_animation.shape[2]
    total_frames = process_frames + path_frames
    ani = animation.FuncAnimation(fig, man2_a.animate, frames=total_frames, interval=20, fargs=(ax,), blit = True, repeat=False)
    plt.show(block=False)
    input()
    # ani.save('astar_man2.gif', fps=int(total_frames*display_rate))


    # Create the figure and axes.
    fig = plt.figure()
    ax  = plt.axes()
    # turn off the axis labels
    ax.axis('off')

    showgrid(state, ax)
    plt.pause(0.001)
    # PART D: 10*Manhattan A*
    print("Part d: 10*Manhattan A*")
    man10_a = Astar(state, (5,4), (5,12), lambda x,y: 10*(abs(x[0] - y[0]) + abs(x[1] - y[1])))

    process_frames = man10_a.process_animation.shape[2]
    path_frames = man10_a.path_animation.shape[2]
    total_frames = process_frames + path_frames
    ani = animation.FuncAnimation(fig, man10_a.animate, frames=total_frames, interval=20, fargs=(ax,), blit = True, repeat=False)
    plt.show(block=False)
    input()
    # ani.save('astar_man10.gif', fps=int(total_frames*display_rate))
