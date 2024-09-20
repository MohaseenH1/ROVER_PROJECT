import time
import serial
from queue import PriorityQueue

# Define the grid size and obstacles for A* algorithm
grid_size = 6
obstacles = [(3, 1), (1, 2), (4, 5), (4, 4), (0, 1), (3, 4)]
start = (5, 5)
end = (2, 2)

# Set up serial communication with Arduino
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
time.sleep(2)  # Allow time for the connection to establish

# A* Pathfinding algorithm
def heuristic(a, b):
    """Heuristic function to calculate the Manhattan distance."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star_search(start, goal):
    """Performs A* search to find the shortest path avoiding obstacles."""
    open_set = PriorityQueue()
    open_set.put((0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while not open_set.empty():
        _, current = open_set.get()

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]

        neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # Right, Down, Left, Up
        for dx, dy in neighbors:
            neighbor = (current[0] + dx, current[1] + dy)

            if 0 <= neighbor[0] < grid_size and 0 <= neighbor[1] < grid_size and neighbor not in obstacles:
                tentative_g_score = g_score[current] + 1

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    open_set.put((f_score[neighbor], neighbor))

    return []

def send_command(command):
    """Send movement command to Arduino."""
    ser.write(f"{command}\n".encode('utf-8'))
    time.sleep(0.1)

def move_to_next_step(current, next_step):
    """Send movement commands based on the next step."""
    dx = next_step[0] - current[0]
    dy = next_step[1] - current[1]

    if dx == 1:  # Move Down
        send_command('a')
    elif dx == -1:  # Move Up
        send_command('s')
    elif dy == 1:  # Move Right
        send_command('w')
    elif dy == -1:  # Move Left
        send_command('d')
    
    time.sleep(2)  # Assume 2 seconds to move between points
    send_command('q')  # Stop command

def autonomous_mode():
    """Run the rover in autonomous mode using A* algorithm."""
    path = a_star_search(start, end)

    if not path:
        print("No valid path found.")
        return

    print("Path found:", path)
    current_position = start

    for step in path[1:]:
        move_to_next_step(current_position, step)
        current_position = step

    print("Reached the destination.")

def manual_mode():
    """Run the rover in manual mode using keyboard commands."""
    print("Enter 'w (forward)', 's (right)', 'a (left)', 'd(backward)' for movement and 'q' to stop:")
    while True:
        command = input("Enter command: ")
        
        if command in ['w', 's', 'a', 'd', 'q']:
            ser.write(command.encode())  # Send the command to the Arduino
            if command == 'q':
                break
        else:
            print("Invalid command, please enter 'w', 's', 'a', 'd' or 'q'.")

def main():
    while True:
        mode = input("Enter '1' for Autonomous Mode, '2' for Manual Mode, or 'q' to Quit: ")
        
        if mode == '1':
            autonomous_mode()
        elif mode == '2':
            manual_mode()
        elif mode == 'q':
            print("Exiting the program.")
            break
        else:
            print("Invalid mode selected. Please enter '1', '2', or 'q'.")

    ser.close()  # Close the serial connection when done

if __name__ == "__main__":
    main()
