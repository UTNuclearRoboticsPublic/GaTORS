#include "gators/mcts.hpp"

MCTS::MCTS (Board board, agents::Party party, int player_turn, int duration, int candidates, int search_depth, float uct_c)
: manager_(GameManager(board, party, player_turn)), search_duration_ms_(duration), num_candidates_(candidates), search_depth_(search_depth), c_(uct_c)
{}

TurnSequence MCTS::search ()
{
    Ancestors candidates {generateCandidates()};

    int it {};
    std::chrono::milliseconds durationToRun(search_duration_ms_);
    std::chrono::steady_clock::time_point startTime = std::chrono::steady_clock::now();

    while (std::chrono::steady_clock::now() - startTime < durationToRun) {
        auto elapsedTime = std::chrono::steady_clock::now() - startTime;

        // choose leaf to visit
        int ancestor_id {findLeaf(candidates)};

        // construct a descendent
        GeneticLeaf descendent {GeneticLeaf(candidates[ancestor_id].game_state)};

        // simulate random game
        simulate(descendent);

        // backpropagate score and update num_episodes
        backpropagate(candidates[ancestor_id], descendent);

        it++;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // once time has expired, pick the best sampled move
    int apex_id {maxValuedLeaf(candidates)};

    return candidates[apex_id].turn;
}

void MCTS::printCandidates (Ancestors &candidates) 
{
    std::cout << "Candidates: " << std::endl;
    for (std::size_t i = 0; i < candidates.size(); i++) {
        std::cout << "\t" << candidates[i].value << ", " << candidates[i].episodes << std::endl;
    }
}

void MCTS::simulate (GeneticLeaf &node)
{
    node.game_state.playToDepth(search_depth_);
    // node.value = node.game_state.party_.players.at(node.game_state.party_.playing_order[node.game_state.player_turn_]).get_score() - calculateAverageValue (node.game_state); // from competitive framing
    node.value = node.game_state.party_.players.at(node.game_state.party_.playing_order[node.game_state.player_turn_]).get_score(); // for collaborative framing

}

int MCTS::maxValuedLeaf (Ancestors &candidates)
{
    int max_value_index {0};
    for (std::size_t i = 1; i < candidates.size(); i++) {
        if (candidates[i].value > candidates[max_value_index].value) {
            max_value_index = static_cast<int>(i);
        }
    }
    return max_value_index;
}

Ancestors MCTS::generateCandidates ()
{
    // generate candidate turns and leaves
    TurnOptions options {manager_.generateRandomTurns(num_candidates_)};

    Ancestors candidates;
    for (std::size_t i = 0; i < options.size(); i++) {
        candidates.push_back(generateCandidate(options[i]));
    }

    return candidates;
}

PrimalLeaf MCTS::generateCandidate (TurnSequence &turn)
{
    // create leaf
    PrimalLeaf candidate (manager_.board_, manager_.party_, manager_.player_turn_);
    
    // assign and play turn
    candidate.episodes = 1;
    candidate.turn = turn;
    candidate.game_state.playSequence(turn);

    // calculate value (from turn)
    candidate.value = candidate.game_state.party_.players.at(candidate.game_state.party_.playing_order[candidate.game_state.player_turn_]).get_score();

    return candidate;
}

int MCTS::upperConfidenceStrategy (Ancestors &candidates)
{
    int total_episodes {};
    for (std::size_t i = 0; i < candidates.size(); i++) {
        total_episodes += candidates[i].episodes;
    }

    // accumulate values for each descendent
    std::vector<double> values (candidates.size());
    for (std::size_t i = 0; i < candidates.size(); i++) {
        double explore {c_ * sqrt(log(total_episodes) / candidates[i].episodes)};
        double exploit {candidates[i].value / candidates[i].episodes};
        values[i] = explore + exploit;
    }

    // pick largest
    int max_value_index {0};
    for (std::size_t i = 1; i < values.size(); i++) {
        if (values[i] > values[max_value_index]) {
            max_value_index = static_cast<int>(i);
        }
    }

    // return the corresponding leaf
    return max_value_index;
}

int MCTS::findLeaf (Ancestors &candidates)
{
    return upperConfidenceStrategy(candidates);
}

void MCTS::backpropagate (PrimalLeaf &ancestor, GeneticLeaf &descendent)
{
    ancestor.episodes++;
    
    // ancestor.value += descendent.value; // simplest version for competitive framing

    // float average_value_difference {calculateAverageValue(descendent.game_state) - calculateAverageValue(ancestor.game_state)};
    // ancestor.value += average_value_difference; // more complex version for competitive framing
    
    ancestor.value += calculateCumulativeValue(descendent.game_state); // for collaborative framing
}

float MCTS::calculateCumulativeValue (GameManager &state)
{
    // loop through all players, sum their scores
    float total_value {};
    for (std::string player_id : state.party_.playing_order) {
        total_value += state.party_.players.at(player_id).get_score();
    }
    return total_value;
}

float MCTS::calculateAverageValue (GameManager &state)
{
    return calculateCumulativeValue(state) / state.party_.playing_order.size();
}
