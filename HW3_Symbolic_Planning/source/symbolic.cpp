#include "symbolic.hpp"

#define SYMBOLS 0
#define INITIAL 1
#define GOAL 2
#define ACTIONS 3
#define ACTION_DEFINITION 4
#define ACTION_PRECONDITION 5
#define ACTION_EFFECT 6

Condition::Condition(std::string pred, std::list<std::string> args,
        bool truth) {
    this->predicate = pred;
    this->truth = truth;
    for (std::string arg : args) {
        this->args.push_back(arg);
    }
}

std::string Condition::getPredicate() const {
    return this->predicate;
}

std::list<std::string> Condition::getArgs() const {
    return this->args;
}

bool Condition::getTruth() const {
    return this->truth;
}

void Condition::setTruth(bool truth) {
    this->truth = truth;
}

std::ostream& operator<<(std::ostream& os, const Condition& condition) {
    os << condition.toString() << " ";
    return os;
}

bool Condition::operator==(const Condition& rhs) const // fixed
        {

    if (this->predicate != rhs.predicate
            || this->args.size() != rhs.args.size())
        return false;

    auto lhs_it = this->args.begin();
    auto rhs_it = rhs.args.begin();

    while (lhs_it != this->args.end() && rhs_it != rhs.args.end()) {
        if (*lhs_it != *rhs_it)
            return false;
        ++lhs_it;
        ++rhs_it;
    }

    if (this->truth != rhs.getTruth())
        return false;

    return true;
}

std::string Condition::toString() const {
    std::string temp = "";
    if (!this->truth)
        temp += "!";
    temp += this->predicate;
    temp += "(";
    for (std::string arg : this->args) {
        temp += arg + ",";
    }
    temp = temp.substr(0, temp.length() - 1);
    temp += ")";
    return temp;
}

Action::Action(std::string name, std::list<std::string> args,
        std::unordered_set<Condition>& preconditions,
        std::unordered_set<Condition>& effects) {
    this->name = name;
    for (std::string arg : args) {
        this->args.push_back(arg);
    }
    for (Condition precondition : preconditions) {
        this->preconditions.insert(precondition);
    }
    for (Condition effect : effects) {
        this->effects.insert(effect);
    }
}

Action::Action(const Action &action,
        std::unordered_map<std::string, std::string> &value) {

    this->name = action.getName();
    for (const std::string &arg : action.getArgs()) {
        if (value.count(arg)) {
            this->args.push_back(value[arg]);
        } else {
            this->args.push_back(arg);
        }

    }
    std::list < std::string > argValues;
    for (auto &precondition : action.getPreconditions()) {
        argValues.clear();
        for (const std::string &arg : precondition.getArgs()) {
            if (value.count(arg)) {
                argValues.push_back(value[arg]);
            } else {
                argValues.push_back(arg);
            }
        }
        this->preconditions.insert(
                Condition(precondition.getPredicate(), argValues,
                        precondition.getTruth()));
    }
    for (auto &effect : action.getEffects()) {
        argValues.clear();
        for (const std::string &arg : effect.getArgs()) {
            if (value.count(arg)) {
                argValues.push_back(value[arg]);
            } else {
                argValues.push_back(arg);
            }
        }
        this->effects.insert(
                Condition(effect.getPredicate(), argValues, effect.getTruth()));
    }
}

std::string Action::getName(void) const {
    return this->name;
}
std::list<std::string> Action::getArgs(void) const {
    return this->args;
}
std::unordered_set<Condition> Action::getPreconditions(void) const {
    return this->preconditions;
}
std::unordered_set<Condition> Action::getEffects(void) const {
    return this->effects;
}

bool Action::operator==(const Action& rhs) const {
    if (this->getName() != rhs.getName()
            || this->getArgs().size() != rhs.getArgs().size())
        return false;

    return true;
}

std::ostream& operator<<(std::ostream& os, const Action& action) {
    os << action.toString() << std::endl;
    os << "Precondition: ";
    for (Condition precond : action.getPreconditions())
        os << precond;
    os << std::endl;
    os << "Effect: ";
    for (Condition effect : action.getEffects())
        os << effect;
    os << std::endl;
    return os;
}

std::string Action::toString(void) const {
    std::string temp = "";
    temp += this->getName();
    temp += "(";
    for (std::string arg : this->getArgs()) {
        temp += arg + ",";
    }
    temp = temp.substr(0, temp.length() - 1);
    temp += ")";
    return temp;
}

void World::removeInitialCondition(Condition condition) {
    this->initialConditions.erase(condition);
}
void World::addInitialCondition(Condition condition) {
    this->initialConditions.insert(condition);
}
void World::addGoalCondition(Condition condition) {
    this->goalConditions.insert(condition);
}
void World::removeGoalCondition(Condition condition) {
    this->goalConditions.erase(condition);
}
void World::addSymbol(std::string symbol) {
    symbols.insert(symbol);
}
void World::addSymbols(std::list<std::string> symbols) {
    for (std::string symbol : symbols) {
        this->symbols.insert(symbol);
    }
}
void World::addAction(Action action) {
    this->actions.insert(action);
}

Action World::getAction(std::string name) {
    for (Action a : this->actions) {
        if (a.getName() == name)
            return a;
    }
    throw std::runtime_error("Action " + name + " not found!");
}
std::unordered_set<std::string> World::getSymbols(void) const {
    return this->symbols;
}

std::unordered_set<Action> World::getActions(void) {
    return this->actions;
}

std::ostream& operator<<(std::ostream& os, const World& world) {
    os << "***** Environment *****" << std::endl << std::endl;
    os << "Symbols: ";
    for (std::string symbol : world.getSymbols()) {
        os << symbol + ",";
    }
    os << std::endl;
    os << "Initial conditions: ";
    for (Condition condition : world.initialConditions) {
        os << condition;
    }
    os << std::endl;
    os << "Goal conditions: ";
    for (Condition goal : world.goalConditions) {
        os << goal;
    }

    os << std::endl;
    os << "Actions:" << std::endl;
    for (Action action : world.actions) {
        os << action << std::endl;
    }

    std::cout << "***** Environment Created! *****" << std::endl;
    return os;
}

std::unordered_set<Condition> World::getInitialConditions(void) {
    return this->initialConditions;
}

std::unordered_set<Condition> World::getGoalConditions(void) {
    return this->goalConditions;
}

static std::list<std::string> parseSymbols(std::string symbols_str) {
    std::list < std::string > symbols;
    size_t pos = 0;
    std::string delimiter = ",";
    while ((pos = symbols_str.find(delimiter)) != std::string::npos) {
        std::string symbol = symbols_str.substr(0, pos);
        symbols_str.erase(0, pos + delimiter.length());
        symbols.push_back(symbol);
    }
    symbols.push_back(symbols_str);
    return symbols;
}

World* createWorld(char* filename) {
    std::ifstream inputFile(filename);
    World* env = new World();
    std::regex symbolStateRegex("symbols:", std::regex::icase);
    std::regex symbolRegex("([a-zA-Z0-9_, ]+) *");
    std::regex initialConditionRegex("initialconditions:(.*)",
            std::regex::icase);
    std::regex conditionRegex(
            "(!?[A-Z][a-zA-Z_]*) *\\( *([a-zA-Z0-9_, ]+) *\\)");
    std::regex goalConditionRegex("goalconditions:(.*)", std::regex::icase);
    std::regex actionRegex("actions:", std::regex::icase);
    std::regex precondRegex("preconditions:(.*)", std::regex::icase);
    std::regex effectRegex("effects:(.*)", std::regex::icase);
    int parser = SYMBOLS;

    std::unordered_set < Condition > preconditions;
    std::unordered_set < Condition > effects;
    std::string actionName;
    std::string actionArgs;

    std::string line;
    if (inputFile.is_open()) {
        while (getline(inputFile, line)) {
            std::string::iterator endPos = remove(line.begin(), line.end(),
                    ' ');
            line.erase(endPos, line.end());

            if (line == "")
                continue;

            if (parser == SYMBOLS) {
                std::smatch results;
                if (regex_search(line, results, symbolStateRegex)) {
                    line = line.substr(8);
                    std::sregex_token_iterator iter(line.begin(), line.end(),
                            symbolRegex, 0);
                    std::sregex_token_iterator end;

                    env->addSymbols(parseSymbols(iter->str()));  // fixed

                    parser = INITIAL;
                } else {
                    std::cout << "Symbols are not specified correctly."
                            << std::endl;
                    throw;
                }
            } else if (parser == INITIAL) {
                const char* line_c = line.c_str();
                if (regex_match(line_c, initialConditionRegex)) {
                    const std::vector<int> submatches = { 1, 2 };
                    std::sregex_token_iterator itr(line.begin(), line.end(),
                            conditionRegex, submatches);
                    std::sregex_token_iterator end;

                    while (itr != end) {
                        // name
                        std::string predicate = itr->str();
                        itr++;
                        // args
                        std::string args = itr->str();
                        itr++;

                        if (predicate[0] == '!') {
                            env->removeInitialCondition(
                                    Condition(predicate.substr(1),
                                            parseSymbols(args)));
                        } else {
                            env->addInitialCondition(
                                    Condition(predicate, parseSymbols(args)));
                        }
                    }

                    parser = GOAL;
                } else {
                    std::cout << "Initial conditions not specified correctly."
                            << std::endl;
                    throw;
                }
            } else if (parser == GOAL) {
                const char* line_c = line.c_str();
                if (regex_match(line_c, goalConditionRegex)) {
                    const std::vector<int> submatches = { 1, 2 };
                    std::sregex_token_iterator iter(line.begin(), line.end(),
                            conditionRegex, submatches);
                    std::sregex_token_iterator end;

                    while (iter != end) {
                        // name
                        std::string predicate = iter->str();
                        iter++;
                        // args
                        std::string args = iter->str();
                        iter++;

                        if (predicate[0] == '!') {
                            env->removeGoalCondition(
                                    Condition(predicate.substr(1),
                                            parseSymbols(args)));
                        } else {
                            env->addGoalCondition(
                                    Condition(predicate, parseSymbols(args)));
                        }
                    }

                    parser = ACTIONS;
                } else {
                    std::cout << "Goal conditions not specified correctly."
                            << std::endl;
                    throw;
                }
            } else if (parser == ACTIONS) {
                const char* line_c = line.c_str();
                if (regex_match(line_c, actionRegex)) {
                    parser = ACTION_DEFINITION;
                } else {
                    std::cout << "Actions not specified correctly."
                            << std::endl;
                    throw;
                }
            } else if (parser == ACTION_DEFINITION) {
                const char* line_c = line.c_str();
                if (regex_match(line_c, conditionRegex)) {
                    const std::vector<int> submatches = { 1, 2 };
                    std::sregex_token_iterator iter(line.begin(), line.end(),
                            conditionRegex, submatches);
                    std::sregex_token_iterator end;
                    // name
                    actionName = iter->str();
                    iter++;
                    // args
                    actionArgs = iter->str();
                    iter++;

                    parser = ACTION_PRECONDITION;
                } else {
                    std::cout << "Action not specified correctly." << std::endl;
                    throw;
                }
            } else if (parser == ACTION_PRECONDITION) {
                const char* line_c = line.c_str();
                if (regex_match(line_c, precondRegex)) {
                    const std::vector<int> submatches = { 1, 2 };
                    std::sregex_token_iterator iter(line.begin(), line.end(),
                            conditionRegex, submatches);
                    std::sregex_token_iterator end;

                    while (iter != end) {
                        // name
                        std::string predicate = iter->str();
                        iter++;
                        // args
                        std::string args = iter->str();
                        iter++;

                        bool truth;

                        if (predicate[0] == '!') {
                            predicate = predicate.substr(1);
                            truth = false;
                        } else {
                            truth = true;
                        }

                        Condition precond(predicate, parseSymbols(args), truth);
                        preconditions.insert(precond);
                    }

                    parser = ACTION_EFFECT;
                } else {
                    std::cout << "Precondition not specified correctly."
                            << std::endl;
                    throw;
                }
            } else if (parser == ACTION_EFFECT) {
                const char* line_c = line.c_str();
                if (regex_match(line_c, effectRegex)) {
                    const std::vector<int> submatches = { 1, 2 };
                    std::sregex_token_iterator iter(line.begin(), line.end(),
                            conditionRegex, submatches);
                    std::sregex_token_iterator end;

                    while (iter != end) {
                        // name
                        std::string predicate = iter->str();
                        iter++;
                        // args
                        std::string args = iter->str();
                        iter++;

                        bool truth;

                        if (predicate[0] == '!') {
                            predicate = predicate.substr(1);
                            truth = false;
                        } else {
                            truth = true;
                        }

                        Condition effect(predicate, parseSymbols(args), truth);
                        effects.insert(effect);
                    }

                    env->addAction(
                            Action(actionName, parseSymbols(actionArgs),
                                    preconditions, effects));

                    preconditions.clear();
                    effects.clear();
                    parser = ACTION_DEFINITION;
                } else {
                    std::cout << "Effects not specified correctly."
                            << std::endl;
                    throw;
                }
            }
        }
        inputFile.close();
    }

    else
        std::cout << "Unable to open file";

    return env;
}

State::State(const std::unordered_set<Condition> &conditions) {
    this->conditions = conditions;
}

std::unordered_set<Condition> State::getConditions(void) const {
    return this->conditions;
}

State* State::apply(const Action &action) {
    auto preconditions = action.getPreconditions();

    for (Condition precondition : preconditions) {
        if (precondition.getTruth()) {
            if (!this->conditions.count(precondition)) {
                return nullptr;
            }
        } else {
            precondition.setTruth(true);
            if (this->conditions.count(precondition)) {
                precondition.setTruth(false);
                return nullptr;
            }
            precondition.setTruth(false);
        }
    }
    State *after = new State(this->conditions);

    auto effects = action.getEffects();
    for (auto effect : effects) {
        if (effect.getTruth()) {
            effect.setTruth(false);
            after->conditions.erase(effect);
            effect.setTruth(true);
            after->conditions.insert(effect);
        } else {
            effect.setTruth(true);
            after->conditions.erase(effect);
            effect.setTruth(false);
        }
    }
    return after;
}

bool State::apply(const Action &action, State &after) {
    auto preconditions = action.getPreconditions();

    for (Condition precondition : preconditions) {
        if (precondition.getTruth()) {
            if (!this->conditions.count(precondition)) {
                return false;
            }
        } else {
            precondition.setTruth(true);
            if (this->conditions.count(precondition)) {
                precondition.setTruth(false);
                return false;
            }
            precondition.setTruth(false);
        }
    }
    after.conditions = std::unordered_set < Condition
            > (this->conditions.begin(), this->conditions.end());
    auto effects = action.getEffects();
    for (auto effect : effects) {
        if (effect.getTruth()) {
            effect.setTruth(false);
            after.conditions.erase(effect);
            effect.setTruth(true);
            after.conditions.insert(effect);
        } else {
            effect.setTruth(true);
            after.conditions.erase(effect);
            effect.setTruth(false);
        }
    }
    return true;
}

std::string State::toString(void) const {
    std::string temp = "";
    if (this->conditions.empty()) {
        return temp;
    }
    std::vector < std::string > components;
    for (auto &condition : this->conditions) {
        components.push_back(condition.toString());
    }
    std::sort(components.begin(), components.end());

    while (components.size() > 1) {
        temp += components.back();
        components.pop_back();
        temp += " ^ ";
    }
    temp += components[0];
    return temp;
}

bool State::operator==(const State& other) const {
    return this->conditions == other.conditions;
}
