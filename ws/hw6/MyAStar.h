#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW6.h"
#include "tools/Logging.h"

class MyAStarAlgo : public amp::AStar {
    public:
        virtual GraphSearchResult search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) override;

    private:
        double backPointerLength(const amp::ShortestPathProblem& problem, const std::map<amp::Node, std::pair<amp::Node, double>>& closed_set, amp::Node parent, amp::Node child);
        double edgeLength(const amp::ShortestPathProblem&, amp::Node, amp::Node);
};