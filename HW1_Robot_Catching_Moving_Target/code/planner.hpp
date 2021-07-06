/*
 * planner.hpp
 *
 *  Created on: Feb 6, 2020
 *      Author: Lin, Zhaozhi
 */

#ifndef PLANNER_HPP_
#define PLANNER_HPP_

#include <iostream>
#include <functional>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

 /**
  * Implementation of A* search algorithm
  *
  * @tparam V the type of vertex
  */
template<typename V>
class AStarPlanner {
public:
    AStarPlanner(void);
    AStarPlanner(std::function<std::unordered_map<V, double>(V)>, std::function<double(V, V)>);
    ~AStarPlanner(void) = default;

    std::unordered_map<V, std::unordered_map<V, double>> getAdjancencyList(void);

    virtual double search(V, V);
    virtual std::deque<V> buildPath(V, V);

protected:
    V source;
    V goal;
    std::unordered_map<V, double> gscores;
    
    std::unordered_map<V, std::unordered_map<V, double>> adj;
    std::unordered_map<V, V> predecessor;

    std::priority_queue<V, std::vector<V>, std::function<bool(V, V)>> open;
    std::unordered_set<V> closed;

    std::function<std::unordered_map<V, double>(V)> getEdges;
    std::function<double(V, V)> heuristic;
};

template<typename V>
AStarPlanner<V>::AStarPlanner(void) {
    this->heuristic = [](V vertex, V goal) {return 0; };
    this->getEdges = [](V vertex) {return std::unordered_map<V, double>(); };
}

template<typename V>
AStarPlanner<V>::AStarPlanner(std::function<std::unordered_map<V, double>(V)> geEdges, std::function<double(V, V)> heuristic) {
    this->heuristic = heuristic;
    this->getEdges = getEdges;
}

template<typename V>
std::unordered_map<V, std::unordered_map<V, double>> AStarPlanner<V>::getAdjancencyList(void) {
    return this->adj;
}

template<typename V>
double AStarPlanner<V>::search(V source, V goal) {

    if (this->source != source) {
        this->gscores.clear();
        this->gscores[source] = 0;
    }

    if (this->closed.empty() || this->source != source || this->goal != goal) {
        this->source = source;
        this->goal = goal;
        std::function<bool(V, V)> compare =
            [this, &goal](V vertex1, V vertex2) {return this->gscores[vertex1] + this->heuristic(vertex1, goal) > this->gscores[vertex2] + this->heuristic(vertex2, goal); };
        this->open = std::priority_queue<V, std::vector<V>, std::function<bool(V, V)>>(
            compare);
        this->open.push(source);
        this->closed.clear();
    }

    while (!this->open.empty() && !closed.count(goal)) {
        V curr = this->open.top();
        this->open.pop();
        if (this->closed.count(curr)) {
            continue;
        }

        if (!this->adj.count(curr) && this->getEdges) {
            this->adj[curr] = this->getEdges(curr);
        }

        for (auto itr = this->adj[curr].begin(); itr != this->adj[curr].end(); ++itr) {
            V neighbor = itr->first;
            double weight = itr->second;

            if (!this->closed.count(neighbor)
                && (!this->gscores.count(neighbor)
                    || this->gscores[neighbor] > this->gscores[curr] + weight)) {
                this->gscores[neighbor] = this->gscores[curr] + weight;
                this->predecessor[neighbor] = curr;
                this->open.push(neighbor);
            }
        }
        this->closed.insert(curr);
    }
    return this->closed.count(goal) ? this->gscores[goal] : std::numeric_limits<double>::infinity();
}

template<typename V>
std::deque<V> AStarPlanner<V>::buildPath(V source, V goal) {
    std::deque<V> path;
    
    if (this->search(source, goal) < std::numeric_limits<double>::infinity()) {
        for (V vertex = goal; this->predecessor.count(vertex); vertex = this->predecessor[vertex]) {
            path.push_front(vertex);
        }
        path.push_front(source);
    }

    return path;
}

/**
 * Implementation of Dijkstra's algorithm
 *
 * This is a derived class of AStarPlanner. 
 *
 * @tparam V the type of vertex
 */
template<typename V>
class DijkstraPlanner :public AStarPlanner<V> {
public:
    DijkstraPlanner(std::function<std::unordered_map<V, double>(V)>);
    double search(V, V) override;
    virtual void search(V);
    double getG(V);
};

template<typename V>
DijkstraPlanner<V>::DijkstraPlanner(std::function<std::unordered_map<V, double>(V)> getEdges) {
    this->heuristic = [](V vertex, V goal) {return 0; };
    this->getEdges = getEdges;
}

template<typename V>
double DijkstraPlanner<V>::search(V source, V goal) {
    if (this->closed.empty() || this->source != source) {
        
        this->source = source;
        this->gscores.clear();
        this->gscores[source] = 0;
        std::function<bool(V, V)> compare =
            [this, goal](V vertex1, V vertex2) {return this->gscores[vertex1] > this->gscores[vertex2]; };
        this->open = std::priority_queue<V, std::vector<V>, std::function<bool(V, V)>>(
            compare);
        this->open.push(source);
        this->closed.clear();
    }

    while (!this->open.empty() && !this->closed.count(goal)) {
        V curr = this->open.top();
        //std::cout<<"Dubug " <<curr<<std::endl;
        this->open.pop();
        if (this->closed.count(curr)) {
            continue;
        }

        if (!this->adj.count(curr) && this->getEdges) {
            this->adj[curr] = this->getEdges(curr);
            //std::cout << "Dubug adj " << curr << std::endl;
        }
        for (auto itr = this->adj[curr].begin(); itr != this->adj[curr].end(); ++itr) {
            V neighbor = itr->first;
            double weight = itr->second;
            //std::cout << "Dubug neighbor " << neighbor << std::endl;
            if (!this->closed.count(neighbor)
                && (!this->gscores.count(neighbor)
                    || this->gscores[neighbor] > this->gscores[curr] + weight)) {
                this->gscores[neighbor] = this->gscores[curr] + weight;
                this->predecessor[neighbor] = curr;
                this->open.push(neighbor);
                
            }
        }
        this->closed.insert(curr);
    }
    return this->getG(goal);
}

template<typename V>
void DijkstraPlanner<V>::search(V source) {
    if (this->closed.empty() || this->source != source) {

        this->source = source;
        this->gscores.clear();
        this->gscores[source] = 0;
        std::function<bool(V, V)> compare =
            [this](V vertex1, V vertex2) {return this->gscores[vertex1] > this->gscores[vertex2]; };
        this->open = std::priority_queue<V, std::vector<V>, std::function<bool(V, V)>>(compare);
        this->open.push(source);
        this->closed.clear();
    }

    while (!this->open.empty()) {
        V curr = this->open.top();
        this->open.pop();
        if (this->closed.count(curr)) {
            continue;
        }

        if (!this->adj.count(curr) && this->getEdges) {
            this->adj[curr] = this->getEdges(curr);
        }

        for (auto itr = this->adj[curr].begin(); itr != this->adj[curr].end(); ++itr) {
            V neighbor = itr->first;
            double weight = itr->second;

            if (!this->closed.count(neighbor)
                && (!this->gscores.count(neighbor)
                    || this->gscores[neighbor] > this->gscores[curr] + weight)) {
                this->gscores[neighbor] = this->gscores[curr] + weight;
                this->predecessor[neighbor] = curr;
                this->open.push(neighbor);
            }
        }
        this->closed.insert(curr);
    }
}

template<typename V>
double DijkstraPlanner<V>::getG(V vertex)
{
    return this->closed.count(vertex) ? this->gscores[vertex] : std::numeric_limits<double>::infinity();
}

#endif /* PLANNER_HPP_ */
