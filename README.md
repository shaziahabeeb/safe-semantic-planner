# Safe Semantic Planner in a Finite Cartesian State Space

## PCCST503 – Machine Learning
### Assignment 1

## 1. Introduction

This project implements a Safe Semantic Planner for a finite Cartesian state space.

The planner finds a path from an initial state to a goal state while avoiding bad states. It considers transition cost, safety distance, reliability, and transition availability during planning.

## 2. Objectives

The planner aims to:

- Reach the goal state.
- Avoid all bad states.
- Minimize total transition cost.
- Maintain a safe distance from bad states.
- Maximize reliability.
- Handle dynamic changes in the environment.
- Replan when transitions or goals change.

## 3. State Representation

Each state is represented using:

- A unique state ID.
- A Cartesian embedding/vector.

Example:

```text
State 0 = (0, 0)
State 1 = (1, 0)
State 2 = (2, 0)

## 4. Transition Representation

Each directed transition contains:

Transition ID
Source state
Destination state
Cost
Safety information
Reliability
Availability flag
## 5. Planning Algorithm

The planner uses heuristic graph search to find a suitable path from the initial state to the goal.

The search considers:

Transition cost
Euclidean distance
Safety from bad states
Reliability
Transition availability

Bad states are never included in the final path.

## 6. Safety Computation

For every visited state, the Euclidean distance to the nearest bad state is calculated.

For two states:

d = sqrt((x1-x2)^2 + (y1-y2)^2)

The minimum distance along the complete path is used as the path's minimum safety distance.

## 7. Dynamic Environment

The planner supports changes such as:

Transition becoming unavailable.
Goal state changing.
Addition of new transitions.

After an update, the planner can execute the planning process again using the updated problem information.

## 8. Test Cases

The implementation contains six test cases:

Test Case 1 – Basic Reachability

Tests whether the planner can find a path from the source to the goal.

Test Case 2 – Bad State Avoidance

Tests whether the planner avoids a path containing a bad state.

Test Case 3 – Safety Margin

Tests the trade-off between path cost and safety distance.

Test Case 4 – Dynamic Transition

Tests replanning when an existing transition becomes unavailable.

Test Case 5 – Goal Update

Tests replanning when the goal state changes.

Test Case 6 – Transition Addition

Tests whether the planner can discover a newly added shortcut transition.

## 9. Evaluation Metrics

The following metrics are measured:

Goal success
Number of bad states visited
Total path cost
Minimum safety distance
Cumulative reliability
Number of explored states
Planning time
## 10. Implementation

The project is implemented in C++.

Compile
g++ main.cpp -o planner.exe
Run

On Windows PowerShell:

.\planner.exe
## 11. Expected Result

The planner should:

Successfully reach the goal whenever a valid path exists.
Never visit a bad state.
Select suitable paths considering cost and safety.
Adapt to changes in transitions and goals.
