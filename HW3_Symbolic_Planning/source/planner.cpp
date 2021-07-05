/*
 *  Created on: Apr 19, 2020
 *      Author: Lin, Zhaozhi
 */

#include <vector>
#include <queue>
#include <chrono>

#include "symbolic.hpp"

const bool print_status = true;
const bool using_heuristics = true;

std::vector<std::vector<std::string>> generatePermutations(
        std::vector<std::string> &symbols, size_t permutation_length) {

    std::vector < std::vector < std::string >> permutations;
    if (symbols.size() < permutation_length) {
        return permutations;
    }
    std::function<void(std::vector<std::string>&, size_t)> depthFirstSearch =
            [&permutations, &permutation_length, &symbols, &depthFirstSearch](std::vector<std::string> &permutation, size_t index) {
        if (permutation.size() == permutation_length) {

            do {
                permutations.push_back(permutation);

            }while (std::next_permutation(permutation.begin(), permutation.end()));
            return;
        }
        if (index >= symbols.size()) {
            return;
        }
        while (index < symbols.size()) {
            permutation.push_back(symbols[index]);
            depthFirstSearch(permutation, ++index);
            permutation.pop_back();
        }

    };
    std::vector < std::string > permutation;
    depthFirstSearch(permutation, 0);
    return permutations;
}

std::vector<Action> generateActions(const std::unordered_set<Action> &actions,
        const std::unordered_set<std::string> &pool) {

    std::unordered_map<size_t, std::vector < std::vector<std::string>> > memo;
    std::vector < Action > grounded_actions;
    std::vector < std::string > symbols;
    for (const std::string &symbol : pool) {
        symbols.push_back(symbol);
    }
    std::sort(symbols.begin(), symbols.end());

    for (const Action &action : actions) {
        std::vector < std::string > args;
        for (const std::string &arg : action.getArgs()) {
            if (!pool.count(arg)) {
                args.push_back(arg);
            }
        }
        if (!memo.count(args.size())) {
            memo[args.size()] = generatePermutations(symbols, args.size());
        }

        auto permutations = memo[args.size()];
        std::unordered_map < std::string, std::string > value;
        for (const std::vector<std::string> &permutation : permutations) {
            value.clear();
            for (auto index = 0; index < args.size(); ++index) {
                value[args[index]] = permutation[index];
            }
            grounded_actions.push_back(Action(action, value));
        }
    }
    return grounded_actions;
}

std::list<Action> planner(World* world) {
    // this is where you insert your planner
    auto begin = std::chrono::high_resolution_clock::now();
    float epsilon = 1.2;
    std::vector < Action > actions = generateActions(world->getActions(),
            world->getSymbols());

    const State goal(world->getGoalConditions());
    const State start(world->getInitialConditions());

    std::unordered_map<State, float> hscore;
    std::unordered_map < State, size_t > gscore;
    std::unordered_map < State, State > predecessor; // Current state to previous state
    std::unordered_map < State, Action > edgeIn; // Map to the action resulting in current state

    gscore[start] = 0;
    hscore[goal] = 0;

    auto computeHeuristic =
            [&hscore, &goal](const State &state, bool informed) {
        if (!informed) {
            return 0.0f;
        }
        if (hscore.count(state)) {
            return hscore[state];
        }
        hscore[state] = 0;
        auto curr_conditions = state.getConditions();
        for (const Condition& condition : goal.getConditions()) {

            if (!curr_conditions.count(condition)) {
                ++hscore[state];
            }
        }
        return hscore[state];
    };
    auto compare =
            [&gscore, &computeHeuristic, &epsilon](const State &state1, const State &state2) {
        return epsilon * computeHeuristic(state1, using_heuristics) + gscore[state1] > epsilon * computeHeuristic(state2, using_heuristics) + gscore[state2];
    };

    std::priority_queue<State, std::vector<State>, decltype(compare)> open(
            compare);
    std::unordered_set < State > closed;
    open.push(start);
    std::list < Action > plan;
    State temp;

    while (!open.empty() && computeHeuristic(open.top(), true) > 0) {
        State curr = open.top();

        open.pop();
        if (closed.count(curr)) {
            continue;
        }
        for (const Action& action : actions) {
            if (curr.apply(action, temp)) {
                if (!closed.count(temp)
                        && (!gscore.count(temp)
                                || (gscore[temp] > gscore[curr] + 1))) {

                    gscore[temp] = gscore[curr] + 1;
                    predecessor[temp] = curr;
                    edgeIn[temp] = action;
                    open.push(temp);
                }
            }
        }
        closed.insert(curr);
    }
    if (!open.empty()) {
        for (State state = open.top(); edgeIn.count(state); state =
                predecessor[state]) {
            plan.push_front(edgeIn[state]);
        }
    }
    auto elapsed = std::chrono::duration_cast < std::chrono::seconds
            > (std::chrono::high_resolution_clock::now() - begin).count();
    std::cout << "Time taken: " << elapsed << " s" << std::endl;
    std::cout << "Number of steps: " << plan.size() << std::endl;
    std::cout << "Number of states expanded: " << gscore.size() << std::endl;
    return plan;
}

int main(int argc, char* argv[]) {
    // DO NOT CHANGE THIS FUNCTION
    char* filename = (char*) ("..\data\blocks_and_triangles.txt");
    if (argc > 1)
        filename = argv[1];

    std::cout << "Environment: " << filename << std::endl << std::endl;
    World* env = createWorld(filename);
    if (print_status) {
        std::cout << *env;
    }

    std::list < Action > actions = planner(env);

    std::cout << "\nPlan: " << std::endl;
    for (Action gac : actions) {
        std::cout << gac << std::endl;
    }
    system("pause");
    return EXIT_SUCCESS;
}
