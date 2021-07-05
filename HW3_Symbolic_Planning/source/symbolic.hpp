#pragma once

#ifndef SYMBOLIC_HPP_
#define SYMBOLIC_HPP_

#include <iostream>
#include <fstream>
#include <regex>
#include <unordered_set>
#include <list>
#include <unordered_map>
#include <algorithm>
#include <stdexcept>
#include <functional>

#define SYMBOLS 0
#define INITIAL 1
#define GOAL 2
#define ACTIONS 3
#define ACTION_DEFINITION 4
#define ACTION_PRECONDITION 5
#define ACTION_EFFECT 6

class Condition;
class Action;
class World;
class State;

class Condition {
private:
    std::string predicate;
    std::list<std::string> args;
    bool truth;

public:
    /** Constructor
     An all-args constructor.
     @param predicate the predicate of the condition, e.g At
     @param args the arguments of the condition
     @param truth whether the condition is positive or negative
     */
    Condition(std::string predicate, std::list<std::string> args, bool truth =
            true);
    std::string getPredicate(void) const;
    std::list<std::string> getArgs(void) const;
    bool getTruth(void) const;
    void setTruth(bool truth);
    friend std::ostream& operator<<(std::ostream& os,
            const Condition& condition);
    bool operator==(const Condition& rhs) const;
    std::string toString(void) const;
};

namespace std {
template<> struct hash<Condition> ;
template<> struct equal_to<Condition> ;

template<> struct equal_to<Condition> {
    bool operator()(const Condition& lhs, const Condition& rhs) const {
        return lhs == rhs;
    }
};

template<> struct hash<Condition> {
    size_t operator()(const Condition &condition) const {
        return hash<std::string> { }(condition.toString());
    }
};
}

class Action {
private:
    std::string name;
    std::list<std::string> args;
    std::unordered_set<Condition> preconditions;
    std::unordered_set<Condition> effects;

public:
    Action(void) = default;
    /** Constructor
     An all-args constructor.
     @param name the name of the action, e.g Move
     @param args the arguments of the action
     @param preconditions conditions that must be satisfied to perform the action
     @param effects conditions that are set after applying the action
     */
    Action(std::string name, std::list<std::string> args,
            std::unordered_set<Condition> &preconditions,
            std::unordered_set<Condition> &effects);
    /** Constructor
     Construct a grounded action from an abstract action.
     @param action an abstract action
     @param value mapping from variable name to value, e.g. [x=Block1, y=Block2, z=Table]
     */
    Action(const Action &action,
            std::unordered_map<std::string, std::string> &value);
    std::string getName(void) const;
    std::list<std::string> getArgs(void) const;
    std::unordered_set<Condition> getPreconditions(void) const;
    std::unordered_set<Condition> getEffects(void) const;
    bool operator==(const Action& rhs) const;
    friend std::ostream& operator<<(std::ostream& os, const Action& ac);
    std::string toString(void) const;
};

class State {
protected:
    std::unordered_set<Condition> conditions;
public:
    State(void) = default;
    State(const std::unordered_set<Condition>& conditions);
    std::unordered_set<Condition> getConditions(void) const;
    bool apply(const Action& action, State& after);
    State *apply(const Action& action);
    std::string toString(void) const;
    bool operator==(const State&) const;
};

namespace std {
template<> struct hash<Action> ;
template<> struct hash<State> ;
template<> struct hash<State*> ;

template<> struct equal_to<Action> ;
template<> struct equal_to<State> ;
template<> struct equal_to<State*> ;

template<>
struct equal_to<Action> {
    bool operator()(const Action& lhs, const Action& rhs) const {
        return lhs == rhs;
    }
};

template<>
struct hash<Action> {
    size_t operator()(const Action& ac) const {
        return hash<string> { }(ac.getName());
    }
};

template<>
struct equal_to<State> {
    bool operator()(const State& state1, const State& state2) const {
        return state1 == state2;
    }
};

template<>
struct hash<State> {
    size_t operator()(const State& state) const {
        return std::hash<std::string> { }(state.toString());
    }
};

template<>
struct equal_to<State*> {
    bool operator()(State *state1, State *state2) const {
        if (state1 == nullptr && state2 == nullptr) {
            return true;
        } else if (state1 == nullptr || state2 == nullptr) {
            return false;
        } else {
            return (*state1) == (*state2);
        }

    }
};

template<>
struct hash<State*> {
    size_t operator()(State *state) const {
        return std::hash<std::string> { }(state->toString());
    }
};
}

class World {
private:
    std::unordered_set<Condition> initialConditions;
    std::unordered_set<Condition> goalConditions;
    std::unordered_set<Action> actions;
    std::unordered_set<std::string> symbols;

public:
    void removeInitialCondition(Condition condition);
    void addInitialCondition(Condition condition);
    void addGoalCondition(Condition condition);
    void removeGoalCondition(Condition condition);
    void addSymbol(std::string symbol);
    void addSymbols(std::list<std::string> symbols);
    void addAction(Action action);

    Action getAction(std::string name);
    std::unordered_set<Action> getActions(void);
    std::unordered_set<std::string> getSymbols(void) const;
    std::unordered_set<Condition> getInitialConditions(void);
    std::unordered_set<Condition> getGoalConditions(void);

    friend std::ostream& operator<<(std::ostream& os, const World& w);
};

World* createWorld(char* filename);

#endif  /* SYMBOLIC_HPP_ */
