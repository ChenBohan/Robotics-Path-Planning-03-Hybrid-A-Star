# Robotics-Path-Planning-03-Hybrid-A-Star
Udacity Self-Driving Car Engineer Nanodegree: Trajectroy Generation

## Definition

This implementation of the A* search algorithm uses the bicycle model.

The breadth first search algorithm which ``does not`` use any heuristics to improve its efficiency.

- ``State(x, y, theta, g, f)``: An object which stores ``x``, ``y`` coordinates, direction ``theta``, and current ``g`` and ``f`` values. 
- ``SPEED``: The speed of the vehicle used in the bicycle model.
- ``LENGTH``: The length of the vehicle used in the bicycle model.
- ``NUM_THETA_CELLS``: The number of cells a circle is divided into. This is used in keeping track of which States we have visited already.

## Expand

While there are still states to explore,

Sort the states by f-value and start search using the state with the lowest f-value.

The f-value improves search efficiency by indicating where to look first.

Check if the x and y coordinates are in the same grid cell as the goal.

Otherwise, expand the current state to get a list of possible next states.

```cpp
while(!opened.empty())
  {
    sort(opened.begin(), opened.end(), compare_maze_s);
    maze_s current = opened[0]; //grab first elment
    opened.erase(opened.begin()); //pop first element

    int x = current.x;
    int y = current.y;

    if(idx(x) == goal[0] && idx(y) == goal[1])
    {
      cout << "found path to goal in " << total_closed << " expansions" << endl;
      maze_path path;
      path.came_from = came_from;
      path.closed = closed;
      path.final = current;
      return path;

    }
    vector<maze_s> next_state = expand(current, goal);

    for(int i = 0; i < next_state.size(); i++)
    {
      int g2 = next_state[i].g;
      double x2 = next_state[i].x;
      double y2 = next_state[i].y;
      double theta2 = next_state[i].theta;

      if((x2 < 0 || x2 >= grid.size()) || (y2 < 0 || y2 >= grid[0].size()))
      {
        //invalid cell
        continue;
      }
      int stack2 = theta_to_stack_number(theta2);

      if(closed[stack2][idx(x2)][idx(y2)] == 0 && grid[idx(x2)][idx(y2)] == 0)
      {

        opened.push_back(next_state[i]);
        closed[stack2][idx(x2)][idx(y2)] = 1;
        came_from[stack2][idx(x2)][idx(y2)] = current;
        total_closed += 1;
      }
    }
  }
```

<img src="https://github.com/ChenBohan/Robotics-Path-Planning-04-Hybrid-A-Star/blob/master/readme_img/expand.png" width = "70%" height = "70%" div align=center />

```cpp
vector<HBF::maze_s> HBF::expand(HBF::maze_s state, vector<int> goal) {
  int g = state.g;
  double x = state.x;
  double y = state.y;
  double theta = state.theta;
    
  int g2 = g+1;
  vector<HBF::maze_s> next_states;
  for(double delta_i = -35; delta_i < 40; delta_i+=5)
  {

    double delta = M_PI / 180.0 * delta_i;
    double omega = SPEED / LENGTH * tan(delta);
    double theta2 = theta + omega;
    if(theta2 < 0)
    {
    	theta2 += 2*M_PI;
    }
    double x2 = x + SPEED * cos(theta);
    double y2 = y + SPEED * sin(theta);
    HBF::maze_s state2;
    state2.f = g2 + heuristic(x2, y2, goal);
    state2.g = g2;
    state2.x = x2;
    state2.y = y2;
    state2.theta = theta2;
    next_states.push_back(state2);

  }
  return next_states;
}
```
