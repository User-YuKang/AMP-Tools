#include "MyMultiAgentPlanners.h"

amp::MultiAgentPath2D MyCentralPlanner::plan(const amp::MultiAgentProblem2D& problem) {
    amp::MultiAgentPath2D path;
    MultiAgentDisk2DCollisionChecker collision_checker(problem);

    Eigen::VectorXd initial;
    initial.resize(problem.numAgents()*2);
    Eigen::VectorXd goal;
    goal.resize(problem.numAgents()*2);

    for(int i = 0; i < problem.numAgents(); i++){
        initial(i*2) = problem.agent_properties[i].q_init(0);
        initial(i*2+1) = problem.agent_properties[i].q_init(1);
        goal(i*2) = problem.agent_properties[i].q_goal(0);
        goal(i*2+1) = problem.agent_properties[i].q_goal(1);
    }
    amp::Path raw_path = planND(initial, goal, collision_checker);

    if (raw_path.waypoints.size() == 0){
        for (const amp::CircularAgentProperties& agent : problem.agent_properties) {
            amp::Path2D agent_path;
            agent_path.waypoints = {agent.q_init, agent.q_goal};
            path.agent_paths.push_back(agent_path);
        }
        return path;
    }

    // std::vector<amp::Path2D> agent_paths;
    for (const amp::CircularAgentProperties& agent : problem.agent_properties) {
        amp::Path2D agent_path;
        agent_path.waypoints = {agent.q_init};
        path.agent_paths.push_back(agent_path);
    }

    for (int i = 0; i < path.agent_paths.size(); i++){
        for(int j = 1; j<raw_path.waypoints.size(); j++){
            path.agent_paths[i].waypoints.push_back({Eigen::Vector2d(raw_path.waypoints[j](2*i), raw_path.waypoints[j](2*i+1))});
        }
    }

    return path;
}

amp::MultiAgentPath2D MyDecentralPlanner::plan(const amp::MultiAgentProblem2D& problem) {
    amp::MultiAgentPath2D path;

    for(int i = 0; i < problem.numAgents(); i++){
        MultiAgentDisk2DCollisionCheckerDecen collision_checker(problem,path);
        amp::Path2D agent_path;
        agent_path.waypoints = (planNDDecen(problem.agent_properties[i].q_init, problem.agent_properties[i].q_goal, collision_checker)).getWaypoints2D();
        path.agent_paths.push_back(agent_path);
    }

    // for (const amp::CircularAgentProperties& agent : problem.agent_properties) {
    //     amp::Path2D agent_path;
    //     agent_path.waypoints = {agent.q_init, agent.q_goal};
    //     path.agent_paths.push_back(agent_path);
    // }
    return path;
}