#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW8.h"

// This is file is loaded from the shared/ directory
// Overwrite with your MySamplingBasedPlanners.h and MySamplingBasedPlanners.cpp from hw7
#include "MySamplingBasedPlanners.h" 


class MyCentralPlanner : public amp::CentralizedMultiAgentRRT, public MyGenericRRT {
    public:
        MyCentralPlanner(double r_, double bias_, int max_it_): MyGenericRRT(bias_, max_it_, r_) {}

        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem_) override; 
};


class MyDecentralPlanner : public amp::DecentralizedMultiAgentRRT, public MyGenericRRT {
    public:
        MyDecentralPlanner(double r_, double bias_, int max_it_): MyGenericRRT(bias_, max_it_, r_) {}

        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem_) override;
};