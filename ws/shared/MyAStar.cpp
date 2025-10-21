#include "MyAStar.h"

// Implement the search method for the A* algorithm
MyAStarAlgo::GraphSearchResult MyAStarAlgo::search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) {
    // std::cout << "Starting A* Graph Search: Init --> goal | " << problem.init_node << " --> " << problem.goal_node << std::endl;
    
    std::unordered_map<amp::Node, std::pair<amp::Node, double>> open_set;
    open_set[problem.init_node] = std::pair<amp::Node, double>(-1,-1);
    std::unordered_map<amp::Node, std::pair<amp::Node, double>> closed_set;
    const amp::Graph<double>& graph = *problem.graph; 
    bool success = false;

    int round = 0;
    while(open_set.size() > 0){
        amp::Node n_best = 0;
        double f_best = -1;
        
        // LOG("round: " << round++);
        for(std::unordered_map<amp::Node, std::pair<amp::Node, double>>::iterator it = open_set.begin(); it != open_set.end(); ++it){
            if (it->second.second == -1)
            {
                it->second.second = backPointerLength(problem, closed_set, it->second.first, it->first);
            }
        }
        for(const auto& pair : open_set){
            if (pair.second.second + heuristic(pair.first) <= f_best || f_best == -1)
            {
                n_best = pair.first;
                f_best = pair.second.second + heuristic(pair.first);
            }
        }

        closed_set[n_best] = open_set[n_best];
        open_set.erase(n_best);
        // DEBUG("pass first for loop");
        // DEBUG("n_best is " << n_best);

        if(n_best == problem.goal_node)
        {
            success = true;
            break;
        }

        const std::vector<amp::Node>& children = graph.children(n_best);
        for(amp::Node child : children){
            if (closed_set.count(child) != 0)
            {
                continue;
            }
            if (open_set.count(child) == 0)
            {
                open_set[child] = std::pair<amp::Node, double>(n_best, closed_set[n_best].second + edgeLength(problem, n_best, child));
            }
            else if (closed_set[n_best].second + edgeLength(problem, n_best, child) < open_set[child].second)
            {
                open_set[child] = std::pair<amp::Node, double>(n_best, closed_set[n_best].second + edgeLength(problem, n_best, child));
            }
        }
    }

    // DEBUG("have result");
    GraphSearchResult result = {success, {}, 0.0}; // initialize the results object
    result.node_path.push_back(problem.goal_node);
    if(success){
        amp::Node previous_node = problem.goal_node;
        while(true){
            if (closed_set[previous_node].first != -1){
                result.path_cost += edgeLength(problem, closed_set[previous_node].first, previous_node);
                result.node_path.insert(result.node_path.begin(), closed_set[previous_node].first);
                previous_node = closed_set[previous_node].first;
                continue;
            }
            break;
        }
    }

    // result.print();
    return result;
}

double MyAStarAlgo::backPointerLength(const amp::ShortestPathProblem& problem, const std::unordered_map<amp::Node, std::pair<amp::Node, double>>& closed_set, amp::Node parent, amp::Node child){
    double length = 0;

    while(parent != -1){
        length += edgeLength(problem, parent, child);

        child = parent;
        parent = closed_set.at(parent).first;
    }
    return length;
}

double MyAStarAlgo::edgeLength(const amp::ShortestPathProblem& problem, amp::Node parent, amp::Node child){
    const amp::Graph<double>& graph = *problem.graph;
    const std::vector<amp::Node>& children = graph.children(parent);
    const std::vector<double>& edge = graph.outgoingEdges(parent);
    for (int i = 0; i < children.size(); i++)
    {
        if (child == children[i])
        {
            return edge[i];
        }
    }
    DEBUG("could not find matching edge");
    return 0;
}
