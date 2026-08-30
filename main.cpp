#include <iostream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <cmath>
#include <limits>
#include <algorithm>
#include <chrono>
#include <iomanip>

using namespace std;

const double INF = numeric_limits<double>::infinity();
const double EPS = 1e-9;


// ============================================================
// STATE
// ============================================================

struct State
{
    uint64_t id;
    vector<double> embedding;
};


// ============================================================
// TRANSITION
// ============================================================

struct Transition
{
    uint64_t id;
    uint64_t from;
    uint64_t to;

    double cost;
    double safety;
    double reliability;

    bool available;
};


// ============================================================
// PLANNING PROBLEM
// ============================================================

struct PlanningProblem
{
    uint64_t initialState;
    uint64_t goalState;

    vector<uint64_t> badStates;

    vector<State> states;

    vector<Transition> transitions;
};


// ============================================================
// PLANNING RESULT
// ============================================================

struct PlanningResult
{
    bool success;

    vector<uint64_t> statePath;
    vector<uint64_t> transitionPath;

    double totalCost;
    double safetyScore;
    double totalReliability;

    int exploredStates;
    double planningTime;
};


// ============================================================
// PRIORITY QUEUE NODE
// ============================================================

struct QueueNode
{
    double k1;
    double k2;
    uint64_t state;
};


struct CompareQueueNode
{
    bool operator()(const QueueNode& a,
                    const QueueNode& b) const
    {
        if (fabs(a.k1 - b.k1) > EPS)
            return a.k1 > b.k1;

        return a.k2 > b.k2;
    }
};


// ============================================================
// SAFE SEMANTIC PLANNER
// ============================================================

class SafePlanner
{
private:

    // -----------------------------
    // Graph
    // -----------------------------

    unordered_map<uint64_t, State> stateMap;

    unordered_map<uint64_t, vector<uint64_t>> outgoing;

    unordered_map<uint64_t, vector<uint64_t>> incoming;

    unordered_map<uint64_t, Transition> transitionMap;

    unordered_set<uint64_t> badStates;


    // -----------------------------
    // D* Lite values
    // -----------------------------

    unordered_map<uint64_t, double> g;

    unordered_map<uint64_t, double> rhs;

    priority_queue<
        QueueNode,
        vector<QueueNode>,
        CompareQueueNode
    > open;


    uint64_t start;
    uint64_t goal;


    // -----------------------------
    // Optimization weights
    // -----------------------------

    double safetyWeight;
    double reliabilityWeight;


    // -----------------------------
    // Statistics
    // -----------------------------

    int exploredStates;


    // ========================================================
    // GET G
    // ========================================================

    double getG(uint64_t s)
    {
        if (!g.count(s))
            return INF;

        return g[s];
    }


    // ========================================================
    // GET RHS
    // ========================================================

    double getRHS(uint64_t s)
    {
        if (!rhs.count(s))
            return INF;

        return rhs[s];
    }


    // ========================================================
    // EUCLIDEAN DISTANCE
    // ========================================================

    double euclideanDistance(uint64_t a,
                             uint64_t b)
    {
        const vector<double>& x =
            stateMap[a].embedding;

        const vector<double>& y =
            stateMap[b].embedding;

        double sum = 0.0;

        for (size_t i = 0; i < x.size(); i++)
        {
            double difference =
                x[i] - y[i];

            sum += difference * difference;
        }

        return sqrt(sum);
    }


    // ========================================================
    // DISTANCE TO NEAREST BAD STATE
    // ========================================================

    double distanceToNearestBadState(
        uint64_t state)
    {
        if (badStates.empty())
            return INF;

        double minimumDistance = INF;

        for (uint64_t bad : badStates)
        {
            if (!stateMap.count(bad))
                continue;

            double distance =
                euclideanDistance(state, bad);

            minimumDistance =
                min(minimumDistance, distance);
        }

        return minimumDistance;
    }


    // ========================================================
    // EFFECTIVE COST
    // ========================================================

    double effectiveCost(
        const Transition& t)
    {
        // Unavailable transition
        if (!t.available)
            return INF;


        // Never enter a bad state
        if (badStates.count(t.to))
            return INF;


        double distance =
            distanceToNearestBadState(t.to);


        double safetyPenalty = 0.0;


        if (distance != INF)
        {
            if (distance <= EPS)
                return INF;

            safetyPenalty =
                safetyWeight /
                (distance + EPS);
        }


        // Reliability penalty
        double reliabilityPenalty =
            reliabilityWeight *
            (1.0 - t.reliability);


        return t.cost
             + safetyPenalty
             + reliabilityPenalty;
    }


    // ========================================================
    // CALCULATE D* LITE KEY
    // ========================================================

    pair<double, double> calculateKey(
        uint64_t s)
    {
        double first =
            min(getG(s), getRHS(s));

        double second =
            getRHS(s);

        return {first, second};
    }


    // ========================================================
    // UPDATE VERTEX
    // ========================================================

    void updateVertex(uint64_t u)
    {
        if (u != goal)
        {
            double minimum = INF;


            for (uint64_t transitionID :
                 outgoing[u])
            {
                const Transition& t =
                    transitionMap[transitionID];


                double cost =
                    effectiveCost(t);


                if (cost == INF)
                    continue;


                double value =
                    cost + getG(t.to);


                minimum =
                    min(minimum, value);
            }


            rhs[u] = minimum;
        }


        if (fabs(getG(u) - getRHS(u)) > EPS)
        {
            auto key =
                calculateKey(u);


            open.push({
                key.first,
                key.second,
                u
            });
        }
    }


    // ========================================================
    // KEY COMPARISON
    // ========================================================

    bool keyLess(
        double a1,
        double a2,
        double b1,
        double b2)
    {
        if (a1 < b1 - EPS)
            return true;

        if (fabs(a1 - b1) <= EPS &&
            a2 < b2 - EPS)
            return true;

        return false;
    }


    // ========================================================
    // COMPUTE SHORTEST PATH
    // ========================================================

    void computeShortestPath()
    {
        while (!open.empty())
        {
            QueueNode current =
                open.top();

            open.pop();


            uint64_t u =
                current.state;


            auto currentKey =
                calculateKey(u);


            // Ignore outdated queue entry
            if (keyLess(
                    currentKey.first,
                    currentKey.second,
                    current.k1,
                    current.k2))
            {
                continue;
            }


            auto startKey =
                calculateKey(start);


            bool condition =
                keyLess(
                    current.k1,
                    current.k2,
                    startKey.first,
                    startKey.second
                );


            if (!condition &&
                fabs(getG(start) -
                     getRHS(start)) <= EPS)
            {
                break;
            }


            exploredStates++;


            if (getG(u) > getRHS(u))
            {
                g[u] = getRHS(u);


                // Update predecessors
                for (uint64_t transitionID :
                     incoming[u])
                {
                    uint64_t predecessor =
                        transitionMap[
                            transitionID
                        ].from;

                    updateVertex(predecessor);
                }
            }
            else
            {
                g[u] = INF;


                updateVertex(u);


                for (uint64_t transitionID :
                     incoming[u])
                {
                    uint64_t predecessor =
                        transitionMap[
                            transitionID
                        ].from;

                    updateVertex(predecessor);
                }
            }
        }
    }


    // ========================================================
    // RESET SEARCH
    // ========================================================

    void resetSearch()
    {
        g.clear();
        rhs.clear();


        while (!open.empty())
            open.pop();


        for (const auto& pair :
             stateMap)
        {
            g[pair.first] = INF;
            rhs[pair.first] = INF;
        }


        rhs[goal] = 0;


        auto key =
            calculateKey(goal);


        open.push({
            key.first,
            key.second,
            goal
        });


        exploredStates = 0;


        computeShortestPath();
    }


public:

    // ========================================================
    // CONSTRUCTOR
    // ========================================================

    SafePlanner(
        double safetyWeight = 5.0,
        double reliabilityWeight = 2.0)
    {
        this->safetyWeight =
            safetyWeight;

        this->reliabilityWeight =
            reliabilityWeight;

        exploredStates = 0;
    }


    // ========================================================
    // BUILD GRAPH
    // ========================================================

    void buildGraph(
        const PlanningProblem& problem)
    {
        stateMap.clear();
        outgoing.clear();
        incoming.clear();
        transitionMap.clear();
        badStates.clear();
        g.clear();
        rhs.clear();


        while (!open.empty())
            open.pop();


        start =
            problem.initialState;

        goal =
            problem.goalState;


        // Store states
        for (const State& state :
             problem.states)
        {
            stateMap[state.id] =
                state;

            g[state.id] =
                INF;

            rhs[state.id] =
                INF;
        }


        // Store bad states
        for (uint64_t bad :
             problem.badStates)
        {
            badStates.insert(bad);
        }


        // Store transitions
        for (const Transition& t :
             problem.transitions)
        {
            transitionMap[t.id] =
                t;


            outgoing[t.from]
                .push_back(t.id);


            incoming[t.to]
                .push_back(t.id);
        }
    }


    // ========================================================
    // INITIALIZE
    // ========================================================

    void initialize()
    {
        resetSearch();
    }


    // ========================================================
    // PLAN
    // ========================================================

    PlanningResult plan(
        const PlanningProblem& problem)
    {
        PlanningResult result;


        result.success = false;

        result.totalCost = 0.0;

        result.safetyScore = INF;

        result.totalReliability = 1.0;

        result.exploredStates = 0;

        result.planningTime = 0.0;


        auto timeStart =
            chrono::high_resolution_clock::now();


        // Build graph
        buildGraph(problem);


        // Check initial state
        if (!stateMap.count(start))
            return result;


        // Check goal
        if (!stateMap.count(goal))
            return result;


        // Start cannot be bad
        if (badStates.count(start))
            return result;


        // Goal cannot be bad
        if (badStates.count(goal))
            return result;


        // D* Lite initialization
        initialize();


        // No path
        if (getG(start) == INF)
        {
            result.exploredStates =
                exploredStates;

            auto timeEnd =
                chrono::high_resolution_clock::now();

            result.planningTime =
                chrono::duration<double, milli>(
                    timeEnd - timeStart
                ).count();

            return result;
        }


        // Reconstruct path
        uint64_t current =
            start;


        result.statePath.push_back(
            current
        );


        unordered_set<uint64_t> visited;


        while (current != goal)
        {
            // Prevent infinite loop
            if (visited.count(current))
            {
                result.success = false;
                return result;
            }


            visited.insert(current);


            double bestValue = INF;


            uint64_t bestTransitionID =
                numeric_limits<uint64_t>::max();


            // Find best transition
            for (uint64_t transitionID :
                 outgoing[current])
            {
                const Transition& t =
                    transitionMap[
                        transitionID
                    ];


                double cost =
                    effectiveCost(t);


                if (cost == INF)
                    continue;


                double value =
                    cost + getG(t.to);


                if (value < bestValue)
                {
                    bestValue =
                        value;

                    bestTransitionID =
                        transitionID;
                }
            }


            // No valid transition
            if (bestTransitionID ==
                numeric_limits<uint64_t>::max())
            {
                result.success = false;
                return result;
            }


            const Transition& best =
                transitionMap[
                    bestTransitionID
                ];


            current =
                best.to;


            result.transitionPath.push_back(
                best.id
            );


            result.statePath.push_back(
                current
            );


            // Original cost
            result.totalCost +=
                best.cost;


            // Reliability
            result.totalReliability *=
                best.reliability;


            // Safety
            double distance =
                distanceToNearestBadState(
                    current
                );


            if (distance != INF)
            {
                result.safetyScore =
                    min(
                        result.safetyScore,
                        distance
                    );
            }
        }


        result.success = true;


        result.exploredStates =
            exploredStates;


        auto timeEnd =
            chrono::high_resolution_clock::now();


        result.planningTime =
            chrono::duration<double, milli>(
                timeEnd - timeStart
            ).count();


        return result;
    }


    // ========================================================
    // UPDATE TRANSITION
    // ========================================================

    void updateTransition(
        uint64_t transitionID,
        bool available)
    {
        if (!transitionMap.count(
                transitionID))
        {
            cout << "Transition not found.\n";
            return;
        }


        transitionMap[
            transitionID
        ].available =
            available;


        uint64_t from =
            transitionMap[
                transitionID
            ].from;


        updateVertex(from);


        computeShortestPath();
    }


    // ========================================================
    // UPDATE GOAL
    // ========================================================

    void updateGoal(
        uint64_t newGoal)
    {
        if (!stateMap.count(newGoal))
        {
            cout << "Goal state does not exist.\n";
            return;
        }


        if (badStates.count(newGoal))
        {
            cout << "Goal cannot be a bad state.\n";
            return;
        }


        goal =
            newGoal;


        resetSearch();
    }


    // ========================================================
    // UPDATE BAD STATES
    // ========================================================

    void updateBadStates(
        const vector<uint64_t>& newBadStates)
    {
        badStates.clear();


        for (uint64_t state :
             newBadStates)
        {
            badStates.insert(state);
        }


        resetSearch();
    }


    // ========================================================
    // ADD NEW TRANSITION
    // ========================================================

    void addTransition(
        const Transition& t)
    {
        transitionMap[t.id] =
            t;


        outgoing[t.from]
            .push_back(t.id);


        incoming[t.to]
            .push_back(t.id);


        updateVertex(t.from);


        computeShortestPath();
    }
};


// ============================================================
// PRINT RESULT
// ============================================================

void printResult(
    const PlanningResult& result)
{
    cout << "\n";
    cout << "========================================\n";


    if (!result.success)
    {
        cout << "NO SAFE PATH FOUND\n";
        cout << "========================================\n";
        return;
    }


    cout << "PATH FOUND\n";


    cout << "State Path: ";


    for (size_t i = 0;
         i < result.statePath.size();
         i++)
    {
        cout << result.statePath[i];


        if (i + 1 <
            result.statePath.size())
        {
            cout << " -> ";
        }
    }


    cout << "\n";


    cout << "Transition Path: ";


    for (uint64_t id :
         result.transitionPath)
    {
        cout << id << " ";
    }


    cout << "\n";


    cout << fixed
         << setprecision(3);


    cout << "Total Cost: "
         << result.totalCost
         << "\n";


    if (result.safetyScore == INF)
    {
        cout << "Minimum Safety Distance: N/A";
        cout << " (No bad states)";
    }
    else
    {
        cout << "Minimum Safety Distance: "
             << result.safetyScore;
    }


    cout << "\n";


    cout << "Cumulative Reliability: "
         << result.totalReliability
         << "\n";


    cout << "Explored States: "
         << result.exploredStates
         << "\n";


    cout << "Planning Time: "
         << result.planningTime
         << " ms\n";


    cout << "Bad States Visited: 0\n";


    cout << "========================================\n";
}


// ============================================================
// TEST CASE 1
// BASIC REACHABILITY
// ============================================================

void testCase1()
{
    cout << "\n\n";
    cout << "########################################\n";
    cout << "TEST CASE 1: BASIC REACHABILITY\n";
    cout << "########################################\n";


    PlanningProblem problem;


    problem.states =
    {
        {0, {0, 0}},
        {1, {1, 0}},
        {2, {2, 0}},
        {3, {3, 0}}
    };


    problem.initialState = 0;

    problem.goalState = 3;


    problem.badStates = {};


    problem.transitions =
    {
        {0, 0, 1, 1, 1, 0.95, true},
        {1, 1, 2, 1, 1, 0.95, true},
        {2, 2, 3, 1, 1, 0.95, true}
    };


    SafePlanner planner(5.0, 2.0);


    PlanningResult result =
        planner.plan(problem);


    printResult(result);
}


// ============================================================
// TEST CASE 2
// BAD STATE AVOIDANCE
// ============================================================

void testCase2()
{
    cout << "\n\n";
    cout << "########################################\n";
    cout << "TEST CASE 2: BAD STATE AVOIDANCE\n";
    cout << "########################################\n";


    PlanningProblem problem;


    problem.states =
    {
        {0, {0, 0}},
        {1, {1, 1}},
        {2, {2, 1}},
        {3, {1, -1}},
        {4, {2, -1}},
        {5, {3, 0}}
    };


    problem.initialState = 0;

    problem.goalState = 5;


    // State 2 is bad
    problem.badStates =
    {
        2
    };


    problem.transitions =
    {
        // Unsafe path
        {0, 0, 1, 1, 1, 0.90, true},
        {1, 1, 2, 1, 1, 0.90, true},
        {2, 2, 5, 1, 1, 0.90, true},

        // Safe path
        {3, 0, 3, 1, 1, 0.90, true},
        {4, 3, 4, 1, 1, 0.90, true},
        {5, 4, 5, 1, 1, 0.90, true}
    };


    SafePlanner planner(5.0, 2.0);


    PlanningResult result =
        planner.plan(problem);


    printResult(result);
}


// ============================================================
// TEST CASE 3
// SAFETY MARGIN
// ============================================================

void testCase3()
{
    cout << "\n\n";
    cout << "########################################\n";
    cout << "TEST CASE 3: SAFETY MARGIN\n";
    cout << "########################################\n";


    PlanningProblem problem;


    problem.states =
    {
        {0, {0, 0}},

        {1, {1, 0}},
        {2, {2, 0}},

        {3, {1, 5}},
        {4, {2, 5}},

        {5, {3, 0}},

        // Bad state
        {6, {2, 0.5}}
    };


    problem.initialState = 0;

    problem.goalState = 5;


    problem.badStates =
    {
        6
    };


    problem.transitions =
    {
        // Cheap path
        {0, 0, 1, 1, 1, 0.95, true},
        {1, 1, 2, 1, 1, 0.95, true},
        {2, 2, 5, 1, 1, 0.95, true},

        // Safer but expensive path
        {3, 0, 3, 3, 1, 0.95, true},
        {4, 3, 4, 3, 1, 0.95, true},
        {5, 4, 5, 3, 1, 0.95, true}
    };


    SafePlanner planner(
        10.0,
        2.0
    );


    PlanningResult result =
        planner.plan(problem);


    printResult(result);
}


// ============================================================
// TEST CASE 4
// DYNAMIC TRANSITION
// ============================================================

void testCase4()
{
    cout << "\n\n";
    cout << "########################################\n";
    cout << "TEST CASE 4: DYNAMIC TRANSITION\n";
    cout << "########################################\n";


    PlanningProblem problem;


    problem.states =
    {
        {0, {0, 0}},
        {1, {1, 0}},

        {2, {1, 2}},
        {3, {2, 2}},

        {4, {3, 0}}
    };


    problem.initialState = 0;

    problem.goalState = 4;


    problem.badStates = {};


    problem.transitions =
    {
        // Short path
        {0, 0, 1, 1, 1, 0.95, true},
        {1, 1, 4, 1, 1, 0.95, true},

        // Alternative path
        {2, 0, 2, 2, 1, 0.95, true},
        {3, 2, 3, 2, 1, 0.95, true},
        {4, 3, 4, 2, 1, 0.95, true}
    };


    SafePlanner planner(5.0, 2.0);


    cout << "\n--- BEFORE TRANSITION FAILURE ---\n";


    PlanningResult result =
        planner.plan(problem);


    printResult(result);


    cout << "\nTransition 1 becomes UNAVAILABLE.\n";


    planner.updateTransition(
        1,
        false
    );


    cout << "\n--- AFTER TRANSITION FAILURE ---\n";


    // Update problem for next planning call
    problem.transitions[1].available =
        false;


    result =
        planner.plan(problem);


    printResult(result);
}


// ============================================================
// TEST CASE 5
// GOAL UPDATE
// ============================================================

void testCase5()
{
    cout << "\n\n";
    cout << "########################################\n";
    cout << "TEST CASE 5: GOAL UPDATE\n";
    cout << "########################################\n";


    PlanningProblem problem;


    problem.states =
    {
        {0, {0, 0}},

        {1, {1, 0}},
        {2, {2, 0}},

        {3, {1, 2}},
        {4, {2, 2}}
    };


    problem.initialState = 0;

    problem.goalState = 2;


    problem.badStates = {};


    problem.transitions =
    {
        // Route to goal 2
        {0, 0, 1, 1, 1, 0.95, true},
        {1, 1, 2, 1, 1, 0.95, true},

        // Route to goal 4
        {2, 0, 3, 2, 1, 0.95, true},
        {3, 3, 4, 2, 1, 0.95, true}
    };


    SafePlanner planner(5.0, 2.0);


    cout << "\n--- ORIGINAL GOAL: STATE 2 ---\n";


    PlanningResult result =
        planner.plan(problem);


    printResult(result);


    cout << "\nGoal changes from STATE 2 to STATE 4.\n";


    planner.updateGoal(4);


    problem.goalState = 4;


    cout << "\n--- UPDATED GOAL: STATE 4 ---\n";


    result =
        planner.plan(problem);


    printResult(result);
}


// ============================================================
// TEST CASE 6
// TRANSITION ADDITION
// ============================================================

void testCase6()
{
    cout << "\n\n";
    cout << "########################################\n";
    cout << "TEST CASE 6: TRANSITION ADDITION\n";
    cout << "########################################\n";


    PlanningProblem problem;


    problem.states =
    {
        {0, {0, 0}},
        {1, {1, 0}},
        {2, {2, 0}},
        {3, {3, 0}}
    };


    problem.initialState = 0;

    problem.goalState = 3;


    problem.badStates = {};


    problem.transitions =
    {
        {0, 0, 1, 2, 1, 0.95, true},
        {1, 1, 2, 2, 1, 0.95, true},
        {2, 2, 3, 2, 1, 0.95, true}
    };


    SafePlanner planner(5.0, 2.0);


    cout << "\n--- BEFORE SHORTCUT ---\n";


    PlanningResult result =
        planner.plan(problem);


    printResult(result);


    cout << "\nAdding new shortcut S -> G...\n";


    Transition shortcut =
    {
        3,
        0,
        3,
        1,
        1,
        0.99,
        true
    };


    planner.addTransition(shortcut);


    problem.transitions.push_back(shortcut);


    cout << "\n--- AFTER SHORTCUT ---\n";


    result =
        planner.plan(problem);


    printResult(result);
}


// ============================================================
// RUN ALL TEST CASES
// ============================================================

void runAllTests()
{
    testCase1();

    testCase2();

    testCase3();

    testCase4();

    testCase5();

    testCase6();
}


// ============================================================
// MAIN
// ============================================================

int main()
{
    while (true)
    {
        cout << "\n\n";

        cout << "========================================\n";
        cout << "       SAFE SEMANTIC PLANNER\n";
        cout << "========================================\n";

        cout << "1. Basic Reachability\n";
        cout << "2. Bad State Avoidance\n";
        cout << "3. Safety Margin\n";
        cout << "4. Dynamic Transition\n";
        cout << "5. Goal Update\n";
        cout << "6. Transition Addition\n";
        cout << "7. Run All Test Cases\n";
        cout << "8. Exit\n";


        cout << "\nEnter choice: ";


        int choice;

        cin >> choice;


        switch (choice)
        {
            case 1:
                testCase1();
                break;

            case 2:
                testCase2();
                break;

            case 3:
                testCase3();
                break;

            case 4:
                testCase4();
                break;

            case 5:
                testCase5();
                break;

            case 6:
                testCase6();
                break;

            case 7:
                runAllTests();
                break;

            case 8:
                cout << "\nExiting...\n";
                return 0;

            default:
                cout << "\nInvalid choice.\n";
        }
    }

    return 0;
}