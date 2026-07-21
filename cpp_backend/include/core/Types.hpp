#pragma once

namespace uts //Urban Traffic Simulator
{
    using road_id_t = int; //Although road_id_t is an integer, it is used to represent the unique identifier of a road in the traffic simulation.
    using node_id_t = int; //Although node_id_t is an integer, it is used to represent the unique identifier of a node in the traffic simulation.

    constexpr road_id_t kInvalidRoadId = -1; //Defines a constant value representing an invalid road identifier. It is set to -1, indicating that the road ID is not valid or does not exist in the simulation.
    constexpr node_id_t kInvalidNodeId = -1; //Defines a constant value representing an invalid node identifier. It is set to -1, indicating that the node ID is not valid or does not exist in the simulation.
}