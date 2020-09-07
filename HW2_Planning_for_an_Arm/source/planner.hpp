/*
 * This file contains 4 classes, each of which implements
 * a samapling-based planning algorithm in robotics.
 *
 *  Created on: Feb 27, 2020
 *      Author: Lin, Zhaozhi
 */
#pragma once

#ifndef PLANNER_HPP_
#define PLANNER_HPP_

#include <iostream>
#include <algorithm>
#include <cfloat>
#include <ctime>
#include <limits>
#include <cmath>
#include <vector>
#include <utility>
#include <utility>
#include <unordered_map>
#include <unordered_set>
#include <iterator>
#include <functional>
#include <queue>
#include <numeric>

enum ExtensionStatus {
    INITED, ADVANCED, TRAPPED, REACHED, CONNECTED,
};

/**
 * Implementation of a rapidly exploring random tree
 *
 * The algorithm grows a tree rooted at the starting configuration by 
 * using random samples from the search space. As each sample is drawn, 
 * a connection is attempted between it and the nearest state in the tree.
 * If the connection is feasible, this results in the addition of the new
 * state to the tree.
 *
 * @tparam C the type of configuration
 */
template<typename C>
class RRTPlanner {

public:
    RRTPlanner(size_t numNodes, double tolerance, double goalSamplingRate,
            std::function<bool(C)> isValid,
            std::function<double(C, C)> computeDistance,
            std::function<C(void)> sample,
            std::function<C(C, C, double)> interpolate,
            std::function<void(C)> destroyConfig);

    virtual ~RRTPlanner(void);

    virtual std::deque<C> buildPath(C start, C goal, double extensionDistance,
            double resolution);

protected:
    const size_t numNodes;

    C start;
    C goal;
    double tolerance;
    float goalSamplingRate;

    std::unordered_map<C, double> cost;

    std::vector<C> configs;
    std::unordered_map<C, C> predecessor;

    virtual void reset(void);

    std::function<bool(C)> isValid;
    std::function<double(C, C)> computeDistance;
    std::function<C(void)> sample;
    std::function<C(C, C, double)> interpolate;
    std::function<void(C)> destroyConfig;

    std::pair<C, double> getNearest(C config);
    virtual ExtensionStatus extend(C configFrom, C configTo,
            double extensionDistance, double resolution);
};

template<typename C>
RRTPlanner<C>::RRTPlanner(size_t numNodes, double tolerance,
        double goalSamplingRate, std::function<bool(C)> isValid,
        std::function<double(C, C)> computeDistance,
        std::function<C(void)> sample,
        std::function<C(C, C, double)> interpolate,
        std::function<void(C)> destroyConfig) :
        numNodes(numNodes) {

    this->isValid = isValid;
    this->computeDistance = computeDistance;
    this->sample = sample;
    this->destroyConfig = destroyConfig;
    this->interpolate = interpolate;

    this->tolerance = tolerance;
    this->goalSamplingRate = goalSamplingRate;
}

template<typename C>
RRTPlanner<C>::~RRTPlanner(void) {
    this->reset();
}

template<typename C>
void RRTPlanner<C>::reset(void) {
    for (C config : this->configs) {
        if (config != start && config != goal) {
            this->destroyConfig(config);
        }
    }
    this->configs.clear();
    this->predecessor.clear();
    this->cost.clear();
}

template<typename C>
std::pair<C, double> RRTPlanner<C>::getNearest(C config) {
    double minDistance = DBL_MAX;
    std::pair<C, double> nearest;

    for (auto itr = this->configs.begin(); itr != this->configs.end(); ++itr) {
        if (*itr == config) {
            continue;
        }
        double distance = this->computeDistance(config, (*itr));
        if (distance < minDistance) {
            minDistance = distance;
            nearest.first = *itr;
        }
    }
    nearest.second = minDistance;
    return nearest;
}

template<typename C>
ExtensionStatus RRTPlanner<C>::extend(C configFrom, C configTo,
        double extensionDistance, double resolution) {

    double distance = this->computeDistance(configFrom, configTo);
    extensionDistance = std::min(extensionDistance, distance);
    auto numSamples = std::floor(extensionDistance / resolution);
    double stepSize = extensionDistance / numSamples;
    C interp;

    for (size_t i = 1; i <= numSamples; ++i) {
        interp = this->interpolate(configFrom, configTo,
                i * stepSize / distance);

        if (!this->isValid(interp)) {
            this->destroyConfig(interp);
            if (i <= 1) {
                return ExtensionStatus::TRAPPED;
            } else {
                interp = this->interpolate(configFrom, configTo,
                        (i - 1) * stepSize / distance);
                break;
            }
        } else if (i < numSamples) {
            this->destroyConfig(interp);
        }
    }
    this->configs.push_back(interp);

    this->predecessor[interp] = configFrom;
    this->cost[interp] = cost[configFrom]
            + this->computeDistance(configFrom, interp);

    if (this->configs[0] == this->start
            && this->computeDistance(interp, this->goal) < this->tolerance) {
        return ExtensionStatus::REACHED;
    } else {
        return ExtensionStatus::ADVANCED;
    }
}

template<typename C>
std::deque<C> RRTPlanner<C>::buildPath(C start, C goal,
        double extensionDistance, double resolution) {

    this->reset();
    std::deque<C> path;

    if (!this->isValid(start)) {
        std::cout << "Invalid start configuration." << std::endl;

        return path;
    }
    if (!this->isValid(goal)) {
        std::cout << "Invalid goal configuration." << std::endl;

        return path;
    }

    this->start = start;
    this->goal = goal;
    this->cost[start] = 0;

    this->configs.push_back(start);

    ExtensionStatus status = ExtensionStatus::INITED;

    while (status != ExtensionStatus::REACHED
            && this->configs.size() < this->numNodes) {
        if (static_cast<float>(rand()) / static_cast<float>(RAND_MAX)
                <= this->goalSamplingRate) {
            C configNearest = this->getNearest(goal).first;
            status = this->extend(configNearest, goal, extensionDistance,
                    resolution);
        } else {
            C configRand = this->sample();
            std::pair<C, double> nearest = this->getNearest(configRand);
            if (nearest.second < this->tolerance) {
                this->destroyConfig(configRand);
                continue;
            }
            C configNearest = nearest.first;
            status = this->extend(configNearest, configRand, extensionDistance,
                    resolution);
            this->destroyConfig(configRand);
        }
    }

    if (status != ExtensionStatus::REACHED) {
        std::cout << "Cannot find a path.\n" << std::endl;
        return path;
    }

    for (C config = this->getNearest(goal).first; config != start;
            config = this->predecessor[config]) {
        path.push_front(config);
    }
    path.push_front(start);

    std::cout << "Number of nodes: " << this->configs.size() << std::endl;
    std::cout << "Path cost: " << this->cost[path.back()] << std::endl;
    return path;
}

/**
 * Implementation of a bi-direcrional rapidly exploring random tree
 *
 * This is a derived class of RRTPlanner. The algorithm grows two trees
 * both from the start and goal configurations until they meet.
 *
 * @tparam C the type of configuration
 */
template<typename C>
class RRTConnectPlanner: public RRTPlanner<C> {

public:

    using RRTPlanner<C>::RRTPlanner;
    virtual ~RRTConnectPlanner(void);

    std::deque<C> buildPath(C start, C goal, double extensionDistance,
            double resolution) override;

protected:

    std::vector<C> configsAnother;
    void reset(void) override;
};

template<typename C>
RRTConnectPlanner<C>::~RRTConnectPlanner(void) {
    this->reset();
}

template<typename C>
void RRTConnectPlanner<C>::reset(void) {
    RRTPlanner<C>::reset();
    this->configsAnother.clear();
}

template<typename C>
std::deque<C> RRTConnectPlanner<C>::buildPath(C start, C goal,
        double extensionDistance, double resolution) {
    this->reset();
    std::deque<C> path;
    if (!this->isValid(start)) {
        std::cout << "Invalid start configuration." << std::endl;
        return path;
    }
    if (!this->isValid(goal)) {
        std::cout << "Invalid goal configuration." << std::endl;

        return path;
    }

    this->start = start;
    this->goal = goal;
    this->cost[start] = 0;
    this->cost[goal] = 0;

    this->configs.push_back(start);
    this->configsAnother.push_back(goal);

    ExtensionStatus status = ExtensionStatus::INITED;
    double pathCost = std::numeric_limits<double>::infinity();

    while (status != ExtensionStatus::CONNECTED
            && this->configs.size() + this->configsAnother.size()
                    < this->numNodes) {

        C configRand = this->sample();
        std::pair<C, double> nearest = this->getNearest(configRand);
        if (nearest.second < this->tolerance) {
            this->destroyConfig(configRand);
            continue;
        }
        C configNearest = nearest.first;
        status = this->extend(configNearest, configRand, extensionDistance,
                resolution);
        this->destroyConfig(configRand);

        if (status != ExtensionStatus::TRAPPED) {
            C configNew = this->configs.back();
            this->configsAnother.swap(this->configs);
            C configNearest = this->getNearest(configNew).first;
            status = this->extend(configNearest, configNew,
                    std::numeric_limits<double>::max(), resolution);
            if (this->computeDistance(configNew, this->configs.back())
                    < this->tolerance) {
                status = ExtensionStatus::CONNECTED;
                pathCost = this->cost[configNew]
                        + this->cost[this->configs.back()]
                        + this->computeDistance(configNew,
                                this->configs.back());
                if (this->configs[0] == start) {
                    for (C config = this->configs.back(); config != start;
                            config = this->predecessor[config]) {
                        path.push_front(config);
                    }
                    path.push_front(this->predecessor[path.back()]);
                    for (C config = configNew; config != goal;
                            config = this->predecessor[config]) {
                        path.push_back(config);
                    }
                    path.push_back(this->predecessor[path.back()]);
                } else {
                    for (C config = configNew; config != start;
                            config = this->predecessor[config]) {
                        path.push_front(config);
                    }
                    path.push_front(this->predecessor[path.back()]);
                    for (C config = this->configs.back(); config != goal;
                            config = this->predecessor[config]) {
                        path.push_back(config);
                    }
                    path.push_back(this->predecessor[path.back()]);
                }
            }
            this->configsAnother.swap(this->configs);
        }

        if (this->configs.size() > this->configsAnother.size()) {
            this->configs.swap(this->configsAnother);
        }

    }

    if (status != ExtensionStatus::CONNECTED) {
        std::cout << "Cannot find a path.\n" << std::endl;
    }

    std::cout << "Number of nodes: "
            << (this->configs.size() + this->configsAnother.size())
            << std::endl;
    std::cout << "Path cost: " << pathCost << std::endl;
    return path;
}

/**
 * Implementation of an RRT* algorithm
 *
 * The algorithm is a derived class of RRTPlanner. In each iteration,
 * it finds the optimal parent node within the area of a ball for the 
 * newly generated node. It also performs rewiring, which is to partially
 * reconstruct the tree within this radius of area to maintain the tree 
 * with minimal cost between tree connections.
 * Unlike basic RRT, this variant converges towards an optimal solution.
 *
 * @tparam C the type of configuration
 */
template<typename C>
class RRTStarPlanner: public RRTPlanner<C> {

public:

    using RRTPlanner<C>::RRTPlanner;
    virtual ~RRTStarPlanner(void);
    void setRadius(double radius);

    std::deque<C> buildPath(C start, C goal, double extensionDistance,
            double resolution) override;

protected:
    double radius;
    void rewire(void);
};

template<typename C>
RRTStarPlanner<C>::~RRTStarPlanner(void) {
    this->reset();
}

template<typename C>
void RRTStarPlanner<C>::setRadius(double radius) {
    this->radius = radius;
}

template<typename C>
void RRTStarPlanner<C>::rewire(void) {

    std::vector<size_t> indices;
    std::vector<double> distances;

    C configNew = this->configs.back();
    for (size_t i = 0; i < this->configs.size() - 1; ++i) {
        double distance = this->computeDistance(this->configs[i], configNew);
        if (distance <= this->radius) {
            size_t numSamples = 50;
            double stepSize = distance / numSamples;
            bool unblocked = true;

            for (size_t j = 1; j <= numSamples && unblocked; ++j) {
                C interp = this->interpolate(this->configs[i], configNew,
                        j * stepSize / distance);
                unblocked &= this->isValid(interp);
                this->destroyConfig(interp);
            }

            if (unblocked) {
                indices.push_back(i);
                distances.push_back(distance);
                if (this->cost[configNew]
                        > this->cost[this->configs[i]] + distance) {
                    this->cost[configNew] = this->cost[this->configs[i]]
                            + distance;
                    this->predecessor[configNew] = this->configs[i];
                }
            }
        }
    }
    for (size_t i = 0; i < indices.size(); ++i) {
        size_t index = indices[i];
        if (this->cost[this->configs[index]]
                > this->cost[configNew] + distances[i]) {
            this->cost[this->configs[index]] = this->cost[configNew]
                    + distances[i];
            this->predecessor[this->configs[index]] = configNew;
        }
    }
}

template<typename C>
std::deque<C> RRTStarPlanner<C>::buildPath(C start, C goal,
        double extensionDistance, double resolution) {

    this->reset();
    std::deque<C> path;

    if (!this->isValid(start)) {
        std::cout << "Invalid start configuration." << std::endl;

        return path;
    }
    if (!this->isValid(goal)) {
        std::cout << "Invalid goal configuration." << std::endl;
        return path;
    }

    this->start = start;
    this->goal = goal;

    this->configs.push_back(start);

    ExtensionStatus status = ExtensionStatus::INITED;

    while (status != ExtensionStatus::REACHED
            && this->configs.size() < this->numNodes) {
        if (static_cast<float>(rand()) / static_cast<float>(RAND_MAX)
                <= this->goalSamplingRate) {
            C configNearest = this->getNearest(goal).first;
            status = this->extend(configNearest, goal, extensionDistance,
                    resolution);
        } else {
            C configRand = this->sample();
            std::pair<C, double> nearest = this->getNearest(configRand);
            if (nearest.second < this->tolerance) {
                this->destroyConfig(configRand);
                continue;
            }
            C configNearest = nearest.first;
            status = this->extend(configNearest, configRand, extensionDistance,
                    resolution);
            this->destroyConfig(configRand);
        }
        if (status != ExtensionStatus::TRAPPED) {
            this->rewire();
        }

    }

    if (status != ExtensionStatus::REACHED) {
        std::cout << "Cannot find a path.\n" << std::endl;
        return path;
    }

    for (C config = this->getNearest(goal).first; config != start;
            config = this->predecessor[config]) {
        path.push_front(config);
    }
    path.push_front(start);

    std::cout << "Number of nodes: " << this->configs.size() << std::endl;
    std::cout << "Path cost: " << this->cost[path.back()] << std::endl;
    return path;
}

/**
 * Implementation of a probabilistic roadmap
 *
 * The algorithm takes random samples from the configuration space of the robot, tests
 * them for whether they are in the free space, and use a local planner to attempt to
 * connect these configurations to other nearby configurations. The starting and goal
 * configurations are added in, and a graph search algorithm is applied to the resulting
 * graph to determine a path between the starting and goal configurations.
 *
 * @tparam C the type of configuration
 */
template<typename C>
class PRMPlanner {
public:
    PRMPlanner(std::function<bool(C)> isValid,
            std::function<double(C, C)> computeDistance,
            std::function<C(void)> sample,
            std::function<C(C, C, double)> interpolate,
            std::function<void(C)> destroyConfig);
    ~PRMPlanner(void);
    void buildRoadmap(size_t numNodes, double radius, double resolution);
    virtual std::deque<C> buildPath(C start, C goal, double radius,
            double resolution);

protected:
    size_t numNodes;
    double radius;

    C start;
    C goal;
    std::vector<C> vertices;
    std::vector<std::vector<std::pair<size_t, double>>> adj;
    std::unordered_map<size_t, size_t> predecessor;
    std::vector<double> heuristics;

    std::function<bool(C)> isValid;
    std::function<double(C, C)> computeDistance;
    std::function<C(void)> sample;
    std::function<C(C, C, double)> interpolate;
    std::function<void(C)> destroyConfig;

    bool search(void);
    void reset(void);
};

template<typename C>
PRMPlanner<C>::PRMPlanner(std::function<bool(C)> isValid,
        std::function<double(C, C)> computeDistance,
        std::function<C(void)> sample,
        std::function<C(C, C, double)> interpolate,
        std::function<void(C)> destroyConfig) {

    this->isValid = isValid;
    this->computeDistance = computeDistance;
    this->sample = sample;
    this->destroyConfig = destroyConfig;
    this->interpolate = interpolate;
    this->numNodes = 0;
}

template<typename C>
PRMPlanner<C>::~PRMPlanner(void) {
    this->reset();
}

template<typename C>
void PRMPlanner<C>::reset(void) {
    this->heuristics.clear();
    for (size_t i = 0; i < this->numNodes; ++i) {
        this->destroyConfig(this->vertices[i]);
    }
    this->vertices.clear();
    this->adj.clear();
    this->predecessor.clear();
    this->numNodes = 0;
}

template<typename C>
void PRMPlanner<C>::buildRoadmap(size_t numNodes, double radius,
        double resolution) {

    this->reset();

    this->numNodes = numNodes;
    while (this->vertices.size() < numNodes) {
        C config = this->sample();
        if (this->isValid(config)) {
            this->vertices.push_back(config);
            this->adj.push_back(std::vector<std::pair<size_t, double>>());

        } else {
            this->destroyConfig(config);
        }
    }

    this->radius = radius;

    for (size_t i = 0; i < this->vertices.size(); ++i) {
        for (size_t j = i + 1; j < this->vertices.size(); ++j) {
            double distance = this->computeDistance(this->vertices[i],
                    this->vertices[j]);
            if (distance <= radius) {
                auto numSamples = std::floor(distance / resolution);
                double stepSize = distance / numSamples;
                bool flag = true;
                for (auto k = 1; k <= numSamples; ++k) {
                    C interp = this->interpolate(this->vertices[i],
                            this->vertices[j], k * stepSize / distance);
                    flag = this->isValid(interp);
                    this->destroyConfig(interp);
                    if (!flag) {
                        break;
                    }
                }

                if (flag) {
                    this->adj[i].push_back( { j, distance });
                    this->adj[j].push_back( { i, distance });
                }
            }
        }
    }
    std::cout << "Roadmap built." << std::endl;

}

template<typename C>
bool PRMPlanner<C>::search(void) {

    size_t numVertices = this->vertices.size();

    std::vector<double> fscores(numVertices,
            std::numeric_limits<double>::infinity());
    std::vector<double> gscores(numVertices,
            std::numeric_limits<double>::infinity());

    fscores[numVertices - 2] = 0.0;
    gscores[numVertices - 2] = 0.0;

    auto compare =
            [&fscores](size_t index1, size_t index2) {return fscores[index1] > fscores[index2];};
    std::priority_queue<size_t, std::vector<size_t>, decltype(compare)> open(
            compare);
    std::unordered_set<size_t> closed;

    open.push(numVertices - 2);

    while (!open.empty() && !closed.count(numVertices - 1)) {
        size_t curr = open.top();
        open.pop();
        if (closed.count(curr)) {
            continue;
        }

        for (std::pair<size_t, double> edge : this->adj[curr]) {
            size_t neighbor = edge.first;
            if (!closed.count(neighbor)
                    && gscores[neighbor] > gscores[curr] + edge.second) {
                gscores[neighbor] = gscores[curr] + edge.second;
                fscores[neighbor] = gscores[neighbor]
                        + this->heuristics[neighbor];

                this->predecessor[neighbor] = curr;
                open.push(neighbor);
            }
        }
        closed.insert(curr);
    }
    std::cout << "Path cost: " << gscores[numVertices - 1] << std::endl;
    return closed.count(numVertices - 1);
}

template<typename C>
std::deque<C> PRMPlanner<C>::buildPath(C start, C goal, double radius,
        double resolution) {

    if (goal != this->goal) {
        this->heuristics.clear();
    }

    std::deque<C> path;

    if (!this->isValid(start)) {
        std::cout << "Invalid start configuration." << std::endl;

        return path;
    }
    if (!this->isValid(goal)) {
        std::cout << "Invalid goal configuration." << std::endl;

        return path;
    }

    this->start = start;
    this->vertices.push_back(start);
    this->adj.push_back(std::vector<std::pair<size_t, double>>());

    for (size_t j = 0; j < this->vertices.size() - 1; ++j) {
        double distance = this->computeDistance(start, this->vertices[j]);
        if (distance <= radius) {
            auto numSamples = std::floor(distance / resolution);
            double stepSize = distance / numSamples;
            bool flag = true;
            for (auto i = 1; i <= numSamples; ++i) {
                C interp = this->interpolate(start, this->vertices[j],
                        i * stepSize / distance);
                flag = this->isValid(interp);
                this->destroyConfig(interp);
                if (!flag) {
                    break;
                }
            }

            if (flag) {
                this->adj.back().push_back( { j, distance });
                this->adj[j].push_back(
                        { this->vertices.size() - 1, distance });
            }
        }
    }

    this->goal = goal;
    this->vertices.push_back(goal);
    this->adj.push_back(std::vector<std::pair<size_t, double>>());
    for (size_t j = 0; j < this->vertices.size() - 1; ++j) {
        double distance = this->computeDistance(goal, this->vertices[j]);
        this->heuristics.push_back(distance);
        if (distance <= radius) {
            auto numSamples = std::floor(distance / resolution);
            double stepSize = distance / numSamples;
            bool flag = true;
            for (auto i = 1; i <= numSamples; ++i) {
                C interp = this->interpolate(this->vertices[j], goal,
                        i * stepSize / distance);
                flag = this->isValid(interp);
                this->destroyConfig(interp);
                if (!flag) {
                    break;
                }
            }

            if (flag) {
                this->adj.back().push_back( { j, distance });
                this->adj[j].push_back(
                        { this->vertices.size() - 1, distance });
            }
        }
    }
    this->heuristics.push_back(0.0);

    if (search()) {
        for (size_t i = this->vertices.size() - 1; this->predecessor.count(i);
                i = this->predecessor[i]) {
            path.push_front(this->vertices[i]);
        }
        path.push_front(start);
    }

    std::cout << "Number of nodes: " << this->vertices.size() << std::endl;
    return path;
}
#endif /* PLANNER_HPP_ */
