import queue
from scipy.spatial import distance

GOAL_1 = [0, 1, 2, 3, 4, 5, 6, 7, 8]
GOAL_2 = [1, 2, 3, 4, 5, 6, 7, 8, 0]
MOVES = {
    'up':    lambda i: i - 3 if i >= 3 else None,
    'down':  lambda i: i + 3 if i <= 5 else None,
    'left':  lambda i: i - 1 if i % 3 != 0 else None,
    'right': lambda i: i + 1 if i % 3 != 2 else None
}

def is_goal(state):
    return state == GOAL_1 or state == GOAL_2

def get_successors(state):
    index = state.index(0)
    successors = []
    for move, func in MOVES.items():
        new_index = func(index)
        if new_index is not None:
            new_state = state.copy()
            new_state[index], new_state[new_index] = new_state[new_index], new_state[index]
            successors.append((new_state, new_state[new_index]))
    return successors

def bfs(start):
    q = queue.Queue()
    visited = set()
    came_from = {}
    q.put(start)
    visited.add(tuple(start))

    while not q.empty():
        current = q.get()
        if is_goal(current):
            return reconstruct_path(came_from, current), len(visited)
        for next_state, moved_tile in get_successors(current):
            t_next = tuple(next_state)
            if t_next not in visited:
                visited.add(t_next)
                came_from[t_next] = current
                q.put(next_state)
    return [], len(visited)

def iddfs(start):
    total_visited = set()
    for depth in range(50):  # arbitrary depth limit
        path, visited = dls(start, depth)
        total_visited.update(visited)
        if path:
            return path, len(total_visited)
    return [], len(total_visited)

def dls(state, depth, visited=None, came_from=None):
    if visited is None:
        visited = set()
        came_from = {}
    visited.add(tuple(state))
    if is_goal(state):
        return reconstruct_path(came_from, state), visited
    if depth == 0:
        return None, visited
    result = None
    for next_state, moved_tile in get_successors(state):
        t_next = tuple(next_state)
        if t_next not in visited:
            came_from[t_next] = state
            sub_result, visited = dls(next_state, depth - 1, visited, came_from)
            if sub_result:
                result = sub_result
                break
    return result, visited

def gbfs(start):
    q = queue.PriorityQueue()
    visited = set()
    came_from = {}
    q.put((heuristic(start), start))
    visited.add(tuple(start))

    while not q.empty():
        _, current = q.get()
        if is_goal(current):
            return reconstruct_path(came_from, current), len(visited)
        for next_state, moved_tile in get_successors(current):
            t_next = tuple(next_state)
            if t_next not in visited:
                visited.add(t_next)
                came_from[t_next] = current
                q.put((heuristic(next_state), next_state))
    return [], len(visited)

def astar(start):
    q = queue.PriorityQueue()
    visited = set()
    came_from = {}
    g = {tuple(start): 0}
    q.put((heuristic(start), start))

    while not q.empty():
        _, current = q.get()
        t_curr = tuple(current)
        visited.add(t_curr)

        if is_goal(current):
            return reconstruct_path(came_from, current), len(visited)

        for next_state, moved_tile in get_successors(current):
            t_next = tuple(next_state)
            tentative_g = g[t_curr] + 1
            if t_next not in g or tentative_g < g[t_next]:
                came_from[t_next] = current
                g[t_next] = tentative_g
                f = tentative_g + heuristic(next_state)
                q.put((f, next_state))
    return [], len(visited)

def heuristic(state):
    return min(
        distance.euclidean(state, GOAL_1),
        distance.euclidean(state, GOAL_2)
    )

def reconstruct_path(came_from, current):
    path = [current]
    while tuple(current) in came_from:
        current = came_from[tuple(current)]
        path.append(current)
    path.reverse()
    return path

def print_results(name, path, nodes):
    print(f"\n{name} Results:")
    print(f"Nodes developed: {nodes}")
    if not path:
        print("No solution found – the algorithm did not reach a goal state within the allowed number of steps.")
    else:
        print("Steps:")
        for step in path:
            print(step)

def main():
    while True:
        inp = input("Enter a 9-digit number using each digit 0-8 exactly once (e.g., 724506831): ")
        if len(inp) != 9 or not inp.isdigit():
            print("Invalid input: Must contain exactly 9 digits.")
            continue
        digits = [int(ch) for ch in inp]
        if sorted(digits) != list(range(9)):
            print("Invalid input: Must contain each digit from 0 to 8 exactly once.")
            continue
        break

    start = digits
    results = []

    path, nodes = bfs(start)
    print_results("BFS", path, nodes)
    results.append(("BFS", nodes))

    path, nodes = iddfs(start)
    print_results("IDDFS", path, nodes)
    results.append(("IDDFS", nodes))

    path, nodes = gbfs(start)
    print_results("GBFS", path, nodes)
    results.append(("GBFS", nodes))

    path, nodes = astar(start)
    print_results("A*", path, nodes)
    results.append(("A*", nodes))

    print("\nSummary (most to least efficient):")
    for name, nodes in sorted(results, key=lambda x: x[1]):
        print(f"{name}: {nodes} nodes developed")

if __name__ == "__main__":
    main()
