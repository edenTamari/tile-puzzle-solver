import queue
import sys
from scipy.spatial import distance



GOAL_1 = [0, 1, 2, 3, 4, 5, 6, 7, 8]
GOAL_2 = [1, 2, 3, 4, 5, 6, 7, 8, 0]

UP = [3, 4, 5, 6, 7, 8]
DOWN = [0, 1, 2, 3, 4, 5]
LEFT = [1, 2, 4, 5, 7, 8]
RIGHT = [0, 1, 3, 4, 6, 7]

closed = []
squares = []
list_prev_step = list()
list_next_step = list()
squares2 =[]
tree = []
action_if_up =[]
action_if_down = []
action_if_left = []
action_if_right = []


def goal_check(situation):
    return situation == GOAL_1 or situation == GOAL_2


def get_step(step, status, index):
    action_step = status.copy()
    if step == 'up':
        squares.append(action_step[index - 3])
        action_step[index], action_step[index - 3] = (
            action_step[index - 3], action_step[index])
        return action_step, squares[-1]
    if step == 'down':
        squares.append(action_step[index + 3])
        action_step[index], action_step[index + 3] = (
            action_step[index + 3], action_step[index])
        return action_step, squares[-1]
    if step == 'left':
        squares.append(action_step[index - 1])
        action_step[index], action_step[index - 1] = (
            action_step[index - 1], action_step[index])
        return action_step, squares[-1]
    if step == 'right':
        squares.append(action_step[index + 1])
        action_step[index], action_step[index + 1] = (
            action_step[index + 1], action_step[index])
        return action_step, squares[-1]


def actions(status, frontier):
    global action_if_up, action_if_down, action_if_left, action_if_right
    if goal_check(status):
        return
    else:
        closed.append(status)

    index_of_space = status.index(0)
    if index_of_space in UP:
        action_if_up, moved_square = get_step('up', status, index_of_space)
        if goal_check(action_if_up):
            find_path_tree(status, action_if_up)
            return
        if action_if_up in closed:
            squares.pop(-1)
        elif action_if_up not in closed:
            frontier.put(action_if_up)

    if index_of_space in DOWN:
        action_if_down, moved_square = get_step('down', status, index_of_space)
        if goal_check(action_if_down):
            find_path_tree(status, action_if_down)
            return
        if action_if_down in closed:
            squares.pop(-1)
        elif action_if_down not in closed:
            frontier.put(action_if_down)

    if index_of_space in LEFT:
        action_if_left, moved_square = get_step('left', status, index_of_space)
        if goal_check(action_if_left):
            find_path_tree(status, action_if_left)
            return
        if action_if_left in closed:
            squares.pop(-1)
        elif action_if_left not in closed:
            frontier.put(action_if_left)

    if index_of_space in RIGHT:
        action_if_right, moved_square = get_step('right', status, index_of_space)
        if goal_check(action_if_right):
            find_path_tree(status, action_if_right)
            return
        if action_if_right in closed:
            squares.pop(-1)
        elif action_if_right not in closed:
            frontier.put(action_if_right)

    tree.append([status, [action_if_up, action_if_down, action_if_left, action_if_right]])
    return frontier


def depth_limited_dfs(status, depth, max_depth):
    if depth > max_depth:
        return None, 0

    if goal_check(status):
        return [status], 1

    nodes_count = 1
    for action in ['up', 'down', 'left', 'right']:
        index_of_space = status.index(0)
        if index_of_space in globals()[action.upper()]:
            next_step, moved_square = get_step(action, status, index_of_space)
            if next_step not in closed:
                result, count = depth_limited_dfs(next_step, depth + 1, max_depth)
                nodes_count += count
                if result is not None:
                    return [status] + result, nodes_count

    return None, nodes_count


def iterative_deepening_dfs(start, frontier):
    depth = 0
    while True:
        result, nodes_count = depth_limited_dfs(start, 0, depth)
        if result is not None:
            return result, nodes_count
        depth += 1

def minimum_dis_euc(status):
    minimum_res = 100
    while not status.empty():
        son = status.get()
        dis_goal_1 = distance.euclidean(GOAL_1, son)
        dis_goal_2 = distance.euclidean(GOAL_2, son)
        minimum = min(dis_goal_1, dis_goal_2)
        if minimum_res > minimum:
            minimum_res = minimum
            best_son = son
    return best_son

def minimum_dis_euc_astar(status, dis_so_far):
    global list_next_step
    global list_prev_step

    if len(list_next_step) == 0:
        list_prev_step.append([10, status.queue[0]])
    else:
        list_prev_step = list_next_step
        list_next_step.clear()
    list_prev_step.sort()
    minimum_res = 100
    while not status.empty():
        son = status.get()
        dis_goal_1 = distance.euclidean(GOAL_1, son)
        dis_goal_2 = distance.euclidean(GOAL_2, son)
        minimum = min(dis_goal_1 + dis_so_far, dis_goal_2 + dis_so_far)
        list_next_step.append([minimum, son])
        if minimum_res > minimum:
            minimum_res = minimum
            best_son = son
    list_next_step.sort()
    for item in list_prev_step:
        if item[0] < minimum_res and item[1] not in closed:
            minimum_res = item[0]
            best_son = item[1]
    list_prev_step.clear()
    return best_son

def update_squares(prev_step, next_step):
    for (i_prev, i_next) in zip(prev_step, next_step):
        if not i_prev == i_next and i_prev == 0 and not i_next == 0:
            squares2.append(i_next)
            return

def find_path_tree(father, goal):
    update_squares(father, goal)
    for node in reversed(tree):
        if father in node[1]:
            update_squares(node[0], father)
            father = node[0]

def main():
    global closed, squares, list_next_step, list_prev_step, squares2, tree, action_if_up, action_if_down, action_if_left, action_if_right
    frontier = queue.Queue()
    step_one = []
    inp = sys.argv
    inp = input("Enter board: ")
    for item in inp:
        if item.isalnum():
            step_one.append(int(item))

    print("BFS")
    actions(step_one, frontier)
    while not frontier.empty():
        next_step = frontier.get()
        if actions(next_step, frontier) is None:
            break

    print('The number of nodes developed by the BFS algorithm: ', len(closed))
    print('squares needed to move:', squares)

    print("IDDFS")
    closed.clear()
    squares.clear()
    iddfs_result, iddfs_nodes_count = iterative_deepening_dfs(step_one, frontier)
    print('The number of nodes developed by the IDDFS algorithm: i', iddfs_nodes_count)
    print('squares needed to move:', squares)


    print("GBFS")
    frontier = queue.Queue()
    frontier.put(step_one)
    squares2.clear()
    squares.clear()
    closed.clear()
    tree.clear()
    while not frontier.empty():
        next_step = minimum_dis_euc(frontier)
        if actions(next_step, frontier) is None:
            break
    print('The number of nodes developed by the GBFS algorithm: ', len(closed))
    print('squares needed to move:', squares)


    print("A*")
    squares2.clear()
    closed.clear()
    tree.clear()
    frontier = queue.Queue()
    frontier.put(step_one)
    prev_step = step_one
    node_deep = 0
    while not frontier.empty():
        next_step = minimum_dis_euc_astar(frontier, node_deep)
        node_deep += 1
        if actions(next_step, frontier) is None:
            break
    print('The number of nodes developed by the A* algorithm: ', len(closed))
    print('squares needed to move:', squares)


if __name__ == "__main__":
    squares = []
    main()
