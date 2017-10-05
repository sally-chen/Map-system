/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include <limits>
#include <chrono>
#include <cstdlib>      //rand
#include <algorithm>    // std::min
#include "StreetsDatabaseAPI.h"
#include "OSMDatabaseAPI.h"
#include <vector>
#include <queue>
#include <algorithm>    // std::reverse
#include <list>
#include <utility>      // std::pair
#include <unordered_map>
#include <unordered_set>
#include "m4.h"
#include "m3.h"
#include "m1.h"
#include "Node.h"
#include "Global.h"

extern Global *global;

struct smaller_f_score {

    bool operator()(Node*& node1, Node*& node2) const {
        return node1->f_score > node2->f_score;
    }
};

struct shorter_travel_time {
    bool operator()(std::pair<std::vector<unsigned>, double>& first, std::pair<std::vector<unsigned>, double>& second) const {
        return (first.second < second.second);
    }
};
using namespace std;
#define TIME_LIMIT 30 // m4 30 second time limit

double find_distance_between_two_intersection_ids(unsigned point1, unsigned point2);
bool check_path_legal(const std::vector<unsigned>& path, const std::vector<DeliveryInfo>& deliveries);
unsigned find_id(const std::vector<unsigned>& path, unsigned value);

void two_opt(std::vector<unsigned>& path, unsigned travel_time, const std::vector<DeliveryInfo>& deliveries, const double turn_penalty);

void populate_intersection_travel_times(std::vector<unsigned>& destinations_vec, std::unordered_set<unsigned>& destinations,
    std::unordered_map<unsigned,std::unordered_map<unsigned,std::pair<std::vector<unsigned>,double>>>& intersection_travel_times,
    const double turn_penalty);

void populate_destination(const std::vector<DeliveryInfo>& deliveries, 
                                        const std::vector<unsigned>& depots, std::unordered_set<unsigned>& destinations, std::vector<unsigned>& destinations_vec);

void find_path_to_all_package_locations(unsigned intersect_id_start, const std::unordered_set<unsigned>& destinations,
        std::unordered_map<unsigned,std::unordered_map<unsigned,std::pair<std::vector<unsigned>,double>>>& intersection_travel_times,
                                               const double turn_penalty);
void join_three_vectors(const std::vector<unsigned>& first_third,
        const std::vector<unsigned>& middle_third, const std::vector<unsigned>& last_third, std::vector<unsigned>temp_path);

double check_and_change_if_best_path(const std::vector<unsigned>&first_third,
        const std::vector<unsigned>&middle_third, const std::vector<unsigned>&last_third,
        std::vector<unsigned>& path, const std::vector<DeliveryInfo>& deliveries,
        const double turn_penalty, double current_best_time);

double find_travel_times_between_intersections(unsigned start, unsigned end, 
        std::unordered_map<unsigned,std::unordered_map<unsigned,std::pair<std::vector<unsigned>,double>>>& intersection_travel_times);

void find_path_between_intersections_from_hash(unsigned start, unsigned end, 
        std::unordered_map<unsigned,std::unordered_map<unsigned,std::pair<std::vector<unsigned>,double>>>& intersection_travel_times,
        std::vector<unsigned>& path);

bool if_path_exist(unsigned start, unsigned end, 
        std::unordered_map<unsigned,std::unordered_map<unsigned,std::pair<std::vector<unsigned>,double>>>& intersection_travel_times);

void multi_start(const std::vector<unsigned>& depots, const std::vector<DeliveryInfo>& deliveries, 
        std::unordered_map<unsigned,std::unordered_map<unsigned,std::pair<std::vector<unsigned>,double>>>& intersection_travel_times,
        std::vector<std::pair<std::vector<unsigned>, double>>& potential_path_multistart, std::vector<unsigned>& potential_path );

void greedy_find_path(unsigned depot_index, const std::vector<unsigned>& depots, const std::vector<DeliveryInfo>& deliveries, 
        std::unordered_map<unsigned,std::unordered_map<unsigned,std::pair<std::vector<unsigned>,double>>>& intersection_travel_times,
        std::vector<std::pair<std::vector<unsigned>, double>>& potential_path_multistart);

//boolean to check when the time runs out
bool time_out = false;
auto start_time = chrono::high_resolution_clock::now();


std::vector<unsigned> traveling_courier(const std::vector<DeliveryInfo>& deliveries,
        const std::vector<unsigned>& depots,
        const float turn_penalty) {
    
//    std::cout << "--------------new test-----------"<<std::endl;
    
    //path of street seg ids to be returned
    std::vector<unsigned> path;

    //return empty path if no deliveries or depots
    if (deliveries.empty() || depots.empty()) {
        return path;
    }

    //unordered set of pickup/ dropoff / depots locations od intersection id
    std::unordered_set<unsigned> destinations;
    std::vector<unsigned> destinations_vec;
    populate_destination(deliveries,depots,destinations,destinations_vec);
    
    //hash table (interx 1, hashtable (interx 2, pair(path of street seg from interx 1 to 2, travel time)))
    std::unordered_map<unsigned,std::unordered_map<unsigned,std::pair<std::vector<unsigned>,double>>> intersection_travel_times;
    populate_intersection_travel_times(destinations_vec,destinations,intersection_travel_times,turn_penalty);

    
    //vector of path, and corresponding travle time of this path with each start point a diff depot
    std::vector<std::pair<std::vector<unsigned>, double>> potential_path_multistart(depots.size());
    
    //path of intersection ids -- after multistart
    std::vector<unsigned> potential_path;
    
    multi_start(depots,deliveries, intersection_travel_times, potential_path_multistart, potential_path);
    
//    //old-------------------------------------------------------------
//    
//    //start point is any depot between 0 to num of depots -1 
//    unsigned start = depots[rand() % depots.size()];
//
//    
//    
//    potential_path.push_back(start);
//
//
//    //initialize the package check vector to 0
//    //0 -> package not picked up yet
//    //1 -> picked up
//    //2 -> dropped off
//    std::vector<unsigned> package_check(deliveries.size(), 0);
//
//    //count how many packages are dropped off
//    unsigned dropped_off_count = 0;
//
//    //variables for finding closest point
//    double min_distance = find_travel_times_between_intersections(start, deliveries[0].pickUp,intersection_travel_times);
//    double current_distance;
//    unsigned min_dis_intersection_id = deliveries[0].pickUp;
//    unsigned package_id = 0;
//
//    //intersection from which we check wheres closest legal point
//    unsigned current_intersection = start;
//
//    while (dropped_off_count < deliveries.size()) {
//       
//        //go through each delivery to find closest legal point
//        for (unsigned i = 0; i < deliveries.size(); i++) {
////           
//            if (package_check[i] == 0) {
////                
//                //not picked up yet, only check pick up location
//                current_distance = find_travel_times_between_intersections(current_intersection, deliveries[i].pickUp,intersection_travel_times);
//                
////                
//                if (current_distance < min_distance) {
//                    min_distance = current_distance;
//                    min_dis_intersection_id = deliveries[i].pickUp;
//                    package_id = i;
//                }
//            } else if (package_check[i] == 1) {
////                
//                //picked up yet, check drop off location
//                current_distance = find_travel_times_between_intersections(current_intersection, deliveries[i].dropOff,intersection_travel_times);
////                
//                if (current_distance < min_distance) {
//                    min_distance = current_distance;
//                    min_dis_intersection_id = deliveries[i].dropOff;
//                    package_id = i;
//                }
//
//            } else if (package_check[i] == 2) {
//                
////               
//                //already dropped off, directly go to next package
//                continue;
//            }
//        }
//
//        current_intersection = min_dis_intersection_id;
//        
//
//        //prevent duplicates
//        if(potential_path.back()!=current_intersection)
//            potential_path.push_back(current_intersection);
//        
//        //move on to next state for the package
//        package_check[package_id] = package_check[package_id] + 1;
//
//        if (package_check[package_id] == 2) {
//            
//            dropped_off_count++;
//        }
//     
//        min_distance = 100000000000000;
//    }
////    std::cout<<"get here 3--------"<<std::endl;
////  
//    //find closest depot to drop off the truck
//   
//    min_distance = find_travel_times_between_intersections(current_intersection, depots[0],intersection_travel_times);
//    unsigned id = 0;
//    unsigned closest_depot_id = depots[id];
//
//    for (unsigned i = 0; i < depots.size(); i++) {
//        if (find_travel_times_between_intersections(current_intersection, depots[i],intersection_travel_times) < min_distance) {
//            min_distance = find_travel_times_between_intersections(current_intersection, depots[i],intersection_travel_times);
//            closest_depot_id = depots[i];
//        }
//    }
//    std::vector<unsigned> maybe_empty;
//    find_path_between_intersections_from_hash(potential_path.back(), closest_depot_id,intersection_travel_times,maybe_empty);
//
//    while (maybe_empty.empty() && id < depots.size()) {
//        std::cout << "EMPTY D:" << std::endl;
//        id++;
//        closest_depot_id = depots[id];
//        find_path_between_intersections_from_hash(potential_path.back(), closest_depot_id,intersection_travel_times,maybe_empty);
//    }
//    
//    
//    potential_path.push_back(closest_depot_id);
//
//    
    //old-------------------------------------------------------------
    

    
    std::vector<unsigned> path_temp;
    for (unsigned i = 0; i < potential_path.size() - 1; ++i) {
        find_path_between_intersections_from_hash(potential_path[i], potential_path[i + 1],intersection_travel_times,path_temp);       
        path.insert(path.end(), path_temp.begin(), path_temp.end());
    }
    return path;
}

//Returns the distance between two intersection ids in meters

double find_distance_between_two_intersection_ids(unsigned point1, unsigned point2) {
    return find_distance_between_two_points(getIntersectionPosition(point1), getIntersectionPosition(point2));
}

bool check_path_legal(const std::vector<unsigned>& path, const std::vector<DeliveryInfo>& deliveries) {
    //iterate through path and adjust package_vector 
    unsigned num_packages_delivered = 0;
    for (unsigned i = 0; i < deliveries.size(); i++) {
        //find the path id of the pickup address
        //assume that pickup/ dropoff will be present
        unsigned pickup_id = find_id(path, deliveries[i].pickUp);
        unsigned dropoff_id = find_id(path, deliveries[i].dropOff);
        //check that package picked up before dropped off
        if (dropoff_id < pickup_id) {
            //            std::cout << "[pickup before dropoff]" << std::endl;
            //            std::cout << "dropoff" << dropoff_id << " pickup" << pickup_id << std::endl;
            return false;
        }
        num_packages_delivered++;

    }
    if (num_packages_delivered != deliveries.size()) {
        //        std::cout << "[too few]" << std::endl;
        return false;
    }
    return true;
}

unsigned find_id(const std::vector<unsigned>& path, unsigned value) {
    for (int i = path.size() - 1; i >= 0; i--) {
        if (path[i] == value) {
            return i;
        }
    }
}

//MAKE SURE TO PASS IN POTENTIAL PATH w/OUT DEPOTS! (like get rid of the first and last)

void two_opt(std::vector<unsigned>& path, unsigned travel_time, const std::vector<DeliveryInfo>& deliveries, const double turn_penalty) {
    //    std::cout<<"two-opt entered, path size"<<path.size() <<std::endl; 
    //    std::cout<<"path size third"<<path.size()*2/3 <<std::endl; 
    //split the current path into 3 segments 
    auto current_time = chrono::high_resolution_clock::now();
    auto wall_clock =
            chrono::duration_cast<chrono::duration<double>> (
            current_time - start_time);

    std::vector<unsigned>first_third;
    std::vector<unsigned>middle_third;
    std::vector<unsigned>last_third;
    double current_best_time = travel_time;
    unsigned i = 0;


    first_third.clear();
    middle_third.clear();
    last_third.clear();

    //divide up current path into three segments and create iterators to the breakpoints
    int first_break_index = path.size() / 3 + i;
    int second_break_index = path.size()*2 / 3 + i;

    std::vector<unsigned>::iterator first_break_it = path.begin() + first_break_index;
    std::vector<unsigned>::iterator second_break_it = path.begin() + second_break_index;

    //copy divided path into new vectors
    std::copy(path.begin(), first_break_it, std::back_inserter(first_third));
    std::copy(first_break_it + 1, second_break_it, std::back_inserter(middle_third));
    std::copy(second_break_it + 1, path.end(), std::back_inserter(last_third));

    //decrease the size of the first third for every iteration 
    while (i < path.size() / 3 && (first_third.size() > 1)&&(!time_out)) {

        //check the time
        if (wall_clock.count() > 0.9 * TIME_LIMIT) {
            time_out = true;
            return;
        }

        //rearrange the path 

        //    Path Options:
        //    ABC -> original
        current_best_time = check_and_change_if_best_path(first_third, middle_third,
                last_third, path, deliveries, turn_penalty, current_best_time);

        
        //check the time
        if (wall_clock.count() > 0.9 * TIME_LIMIT) {
            time_out = true;
            return;
        }
        //    ACB

        //check if the path is valid and if it is faster than the current best
        //if it is, change the path to this one 
        current_best_time = check_and_change_if_best_path(first_third, last_third,
                middle_third, path, deliveries, turn_penalty, current_best_time);

        //check the time
        if (wall_clock.count() > 0.9 * TIME_LIMIT) {
            time_out = true;
            return;
        }
        //    BAC
        current_best_time = check_and_change_if_best_path(middle_third, first_third,
                last_third, path, deliveries, turn_penalty, current_best_time);

        //check the time
        if (wall_clock.count() > 0.9 * TIME_LIMIT) {
            time_out = true;
            return;
        }
        //    BCA
        current_best_time = check_and_change_if_best_path(middle_third, last_third,
                first_third, path, deliveries, turn_penalty, current_best_time);

        //check the time
        if (wall_clock.count() > 0.9 * TIME_LIMIT) {
            time_out = true;
            return;
        }
        //    CAB

        current_best_time = check_and_change_if_best_path(last_third, first_third,
                middle_third, path, deliveries, turn_penalty, current_best_time);

        //check the time
        if (wall_clock.count() > 0.9 * TIME_LIMIT) {
            time_out = true;
            return;
        }
        //    CBA
        current_best_time = check_and_change_if_best_path(last_third, middle_third,
                first_third, path, deliveries, turn_penalty, current_best_time);

        //change the vector sizes for the next iteration: shorten the first third

        //add the last element of first third to middle third
        middle_third.insert(middle_third.begin(), first_third[first_third.size() - 1]);
        //shorten the first third
        first_third.pop_back();
        //add the last element of the middle to the last 
        last_third.insert(last_third.begin(), middle_third[middle_third.size() - 1]);
        //delete the last element in the middle third 
        middle_third.pop_back();

        i++;
    }

    //restore the original path vectors 
    std::copy(path.begin(), first_break_it, std::back_inserter(first_third));
    std::copy(first_break_it + 1, second_break_it, std::back_inserter(middle_third));
    std::copy(second_break_it + 1, path.end(), std::back_inserter(last_third));

    //decrease the size of the last third for every iteration 
    while (i < path.size() / 3 && (last_third.size() > 1)&&(!time_out)) {

        //check the time
        if (wall_clock.count() > 0.9 * TIME_LIMIT) {
            time_out = true;
            return;
        }

        //rearrange the path 

        //    Path Options:
        //    ABC -> original
        current_best_time = check_and_change_if_best_path(first_third, middle_third,
                last_third, path, deliveries, turn_penalty, current_best_time);
        
        //check the time
        if (wall_clock.count() > 0.9 * TIME_LIMIT) {
            time_out = true;
            return;
        }
        //    ACB

        //check if the path is valid and if it is faster than the current best
        //if it is, change the path to this one 
        current_best_time = check_and_change_if_best_path(first_third, last_third,
                middle_third, path, deliveries, turn_penalty, current_best_time);

        //check the time
        if (wall_clock.count() > 0.9 * TIME_LIMIT) {
            time_out = true;
            return;
        }
        //    BAC
        current_best_time = check_and_change_if_best_path(middle_third, first_third,
                last_third, path, deliveries, turn_penalty, current_best_time);

        //check the time
        if (wall_clock.count() > 0.9 * TIME_LIMIT) {
            time_out = true;
            return;
        }
        //    BCA
        current_best_time = check_and_change_if_best_path(middle_third, last_third,
                first_third, path, deliveries, turn_penalty, current_best_time);

        //check the time
        if (wall_clock.count() > 0.9 * TIME_LIMIT) {
            time_out = true;
            return;
        }
        //    CAB

        current_best_time = check_and_change_if_best_path(last_third, first_third,
                middle_third, path, deliveries, turn_penalty, current_best_time);

        //check the time
        if (wall_clock.count() > 0.9 * TIME_LIMIT) {
            time_out = true;
            return;
        }
        //    CBA
        current_best_time = check_and_change_if_best_path(last_third, middle_third,
                first_third, path, deliveries, turn_penalty, current_best_time);

        //change the vector sizes for the next iteration: shorten the last third

        //add the first element of the last third to the end of the middle third
        middle_third.push_back(last_third[0]);
        //delete the first element of the last third
        last_third.erase(last_third.begin());
        //add the first element of the middle third to the first third
        first_third.push_back(middle_third[0]);
        //delete the first element of the middle third
        middle_third.erase(middle_third.begin());

        i++;
    }
}

void join_three_vectors(const std::vector<unsigned>& first_third,
        const std::vector<unsigned>& middle_third, const std::vector<unsigned>& last_third, std::vector<unsigned>temp_path) {
    temp_path = first_third;
    temp_path.insert(temp_path.end(), middle_third.begin(), middle_third.end());
    temp_path.insert(temp_path.end(), last_third.begin(), last_third.end());
}

double check_and_change_if_best_path(const std::vector<unsigned>&first_third,
        const std::vector<unsigned>&middle_third, const std::vector<unsigned>&last_third,
        std::vector<unsigned>& path, const std::vector<DeliveryInfo>& deliveries,
        const double turn_penalty, double current_best_time) {

    //temporarily concatenate the 3 vectors 
    std::vector<unsigned>temp_path;
    join_three_vectors(first_third, middle_third, last_third, temp_path);
    //check if legal
    if (check_path_legal(temp_path, deliveries)) {
        //recalculate the travel time for this path
        std::cout << "[[[legal path found!]]]!!!!!!!!!!!!!!!!" << std::endl;

        //ADD DEPOTS TO START + END

        double test_time = compute_path_travel_time(temp_path, turn_penalty);
        if (test_time < current_best_time) {
            //if the time is less than the current best travel time, change the path to this one 
            path = temp_path;
            current_best_time = test_time;
        }


    }

    return current_best_time;

}
//load the above data structure by iterating through all the deliveries and depots

void populate_destination(const std::vector<DeliveryInfo>& deliveries, 
                                        const std::vector<unsigned>& depots, std::unordered_set<unsigned>& destinations, std::vector<unsigned>& destinations_vec){
    
    //populate the destination set with pickup / drop off location nodes
    for (unsigned i = 0; i < deliveries.size();i++) {
        if(destinations.find(deliveries[i].pickUp)==destinations.end()){
            destinations.insert(deliveries[i].pickUp);
            destinations_vec.push_back(deliveries[i].pickUp);
        }
        if(destinations.find(deliveries[i].dropOff)==destinations.end()){
            destinations.insert(deliveries[i].dropOff);
            destinations_vec.push_back(deliveries[i].dropOff);
        }
    }
    
    //populate the destination set with depots location nodes
    for (unsigned i = 0; i < depots.size();i++) {
        if(destinations.find(depots[i])==destinations.end()){
            destinations.insert(depots[i]);
            destinations_vec.push_back(depots[i]);
        }
    }  
}

void populate_intersection_travel_times(std::vector<unsigned>& destinations_vec, std::unordered_set<unsigned>& destinations,
    std::unordered_map<unsigned,std::unordered_map<unsigned,std::pair<std::vector<unsigned>,double>>>& intersection_travel_times,
    const double turn_penalty){  
    
    #pragma omp parallel for
    for(unsigned i = 0; i < destinations_vec.size(); i ++){
        std::unordered_map<unsigned,std::pair<std::vector<unsigned>,double>> destination_travel_time_pickup;
        std::pair <unsigned,std::unordered_map<unsigned,std::pair<std::vector<unsigned>,double>>> intersection_hash_pickup (destinations_vec[i], destination_travel_time_pickup);
        #pragma omp critical
        intersection_travel_times.insert(intersection_hash_pickup);
        
        
        find_path_to_all_package_locations(destinations_vec[i], destinations,intersection_travel_times,turn_penalty);
    
    }
}

void find_path_to_all_package_locations(unsigned intersect_id_start, const std::unordered_set<unsigned>& destinations,
        std::unordered_map<unsigned,std::unordered_map<unsigned,std::pair<std::vector<unsigned>,double>>>& intersection_travel_times,
                                               const double turn_penalty){
    
//        
    
    
    
    //path of street seg ids
    std::vector<unsigned> path;     
    std::vector<Node*> intersection_node;
    unsigned numIntersection = getNumberOfIntersections();
    intersection_node.resize(numIntersection);
    
    for(unsigned i = 0; i<numIntersection; i++){
        intersection_node[i] = new Node(i);
    }
    
    
    Node *start = NULL;    
    Node *current = NULL;
    Node *child = NULL;
    
    start = intersection_node[intersect_id_start];   

    // Define the open and the close list
    std::list<Node*>::iterator i;
    std::list<Node*> closedList;
    std::priority_queue<Node*,std::vector<Node*>,smaller_f_score> openList;        
    
    unsigned destination_found_counter = 0;
    unsigned iteration_counter = 0;
    // Add the start point to the openList
    openList.push(start);
    start->in_open_list = true;

    while (!openList.empty()){
        iteration_counter++;
        current = openList.top();
//        std::cout<<" [current] "<<current->intersection_id << " [iterated] " <<iteration_counter<<std::endl;

        // Remove the current point from the openList
        openList.pop();
        current->in_open_list = false;

        //check if equal to any node in the poi set
        auto it1 = destinations.find (current->intersection_id);

        if ( it1 != destinations.end() && (*it1) != intersect_id_start &&  current->in_closed_list == false){            
            
            
            destination_found_counter++;
            path.clear();
            
            Node* current_traceback = NULL;
            current_traceback = current;
            // Resolve the path starting from the end point
            while (current_traceback->hasParent() && current_traceback != start) {
                path.push_back(current_traceback->from_street_seg);
                current_traceback = current_traceback->getParent();
            }

           std::reverse(path.begin(),path.end()); 
           double travelTime = compute_path_travel_time(path,turn_penalty);
           
//           std::cout<<"[start] "<<intersect_id_start<<" [end] "<<(*it1)<< "[path length] "<<path.size()<<" [current id] "<<current->intersection_id<<std::endl;
           
           std::pair <std::vector<unsigned>,double> destination_path_travel_time = std::make_pair(path,travelTime);
           std::pair <unsigned, std::pair<std::vector<unsigned>,double>> destination_hash = std::make_pair((*it1), destination_path_travel_time);
           
           #pragma omp critical
           { 
                auto it2 = intersection_travel_times.find(intersect_id_start);         

                if(it2 != intersection_travel_times.end()){
                     it2->second.insert(destination_hash);   
                }
             }
           
        }
        
        // Add the current point to the closedList
        closedList.push_back(current);
        current->in_closed_list = true;

        //find adjacent intersections -- it automatically excludes the one-way street
        std::vector<unsigned> adjacent_intersections = find_adjacent_intersections(current->intersection_id);

        // Get all current's adjacent walkable points
        for (unsigned j = 0; j < adjacent_intersections.size(); j++) {

            child = intersection_node[adjacent_intersections[j]];

            //pass this neighbor if it is already in closed list (evaluated)
            //or is not reachable from current node (one way)
            if (child->in_closed_list)
                continue;

            if (child->in_open_list) {
                // If it has a wroste g score than the one that pass through the current point
                // then its path is improved when it's parent is the current point
                if (child->getGScore() > child->getGScore(current, turn_penalty, 0)) {
         
                    // Change its parent and g score
                    child->setParent(current);
                    child->computeScores(child,turn_penalty,1);
//                        
                    openList.push(child); 
                }
            }else{

                child->in_open_list = true;

                // Compute it's g, h and f score
                child->setParent(current);
                child->computeScores(child, turn_penalty, 1);

                //Add it to the openList with current point as parent
                openList.push(child);
            }
        }
    }

//    //reset closed and open list
//    for (std::list<Node*>::iterator n = closedList.begin(); n != closedList.end(); ++n){
//        (*n)->in_closed_list = false;
//    }
//    while(!openList.empty()){
//        openList.top()->in_open_list=false;
//        openList.pop();
//    }   
    for(unsigned i = 0; i < intersection_node.size();i++){
        delete intersection_node[i];
    }
    intersection_node.clear();
//    std::cout<<destination_found_counter<<std::endl;
}

double find_travel_times_between_intersections(unsigned start, unsigned end, 
        std::unordered_map<unsigned,std::unordered_map<unsigned,std::pair<std::vector<unsigned>,double>>>& intersection_travel_times){
    
    auto it2 = intersection_travel_times.find(start);
    if(it2 != intersection_travel_times.end()){
        auto it3 = it2->second.find(end);
        
        if(it3!=it2->second.end())
            return (it3->second).second;
        else{
            if( start!= end)
                std::cout<<" [start] "<<start<<" [end] "<<end<<" cannnot find end"<<std::endl;
            
            return 0;
        }
            
    }
    std::cout<<" [start] "<<start<<" [end] "<<end<<" cannnot find start"<<std::endl;
    return 0;

}

void find_path_between_intersections_from_hash(unsigned start, unsigned end, 
        std::unordered_map<unsigned,std::unordered_map<unsigned,std::pair<std::vector<unsigned>,double>>>& intersection_travel_times,
        std::vector<unsigned>& path){
    
    auto it2 = intersection_travel_times.find(start);
     if(it2 != intersection_travel_times.end()){
        auto it3 = it2->second.find(end);
        
        if(it3!=it2->second.end()){
            std::vector<unsigned> temp = (it3->second).first;
            path =  temp;
            return;
        }else{
            if( start!= end){
                std::cout<<" [start] "<<start<<" [end] "<<end<<" cannnot find end"<<std::endl;
                return;
            }else{
                return;
            }
            
        }
            
    }
    std::cout<<" [start] "<<start<<" [end] "<<end<<" cannnot find start"<<std::endl;
    return;


}

bool if_path_exist(unsigned start, unsigned end, 
        std::unordered_map<unsigned,std::unordered_map<unsigned,std::pair<std::vector<unsigned>,double>>>& intersection_travel_times){
    
    if(start == end)
        return true;
    
    auto it2 = intersection_travel_times.find(start);
    if(it2 != intersection_travel_times.end()){
        auto it3 = it2->second.find(end);        
        if(it3!=it2->second.end()){
            return true;
        }else{
            std::cout<<" [start] "<<start<<" [end] "<<end<<" cannnot find end"<<std::endl;
            return false;           
        }            
    }
    std::cout<<" [start] "<<start<<" [end] "<<end<<" cannnot find start"<<std::endl;
    return false;
}

void multi_start(const std::vector<unsigned>& depots, const std::vector<DeliveryInfo>& deliveries, 
        std::unordered_map<unsigned,std::unordered_map<unsigned,std::pair<std::vector<unsigned>,double>>>& intersection_travel_times,
        std::vector<std::pair<std::vector<unsigned>, double>>& potential_path_multistart, std::vector<unsigned>& potential_path ){
    
    #pragma omp parallel for
    for(unsigned i = 0; i < depots.size(); i++){
        //populate potential_path_multistart
        std::vector<unsigned> path_each;
        
        double travel_time = 0;
        potential_path_multistart[i]= std::make_pair(path_each,travel_time);

        greedy_find_path(i,depots,deliveries,intersection_travel_times,potential_path_multistart); 
    }
    //return path with shortest travel time
    potential_path = std::min_element(potential_path_multistart.begin(),potential_path_multistart.end(),shorter_travel_time())->first;

}

void greedy_find_path(unsigned depot_index, const std::vector<unsigned>& depots, const std::vector<DeliveryInfo>& deliveries, 
        std::unordered_map<unsigned,std::unordered_map<unsigned,std::pair<std::vector<unsigned>,double>>>& intersection_travel_times,
        std::vector<std::pair<std::vector<unsigned>, double>>& potential_path_multistart){

    potential_path_multistart[depot_index].first.push_back(depots[depot_index]);
    
    //initialize the package check vector to 0
    //0 -> package not picked up yet
    //1 -> picked up
    //2 -> dropped off
    std::vector<unsigned> package_check(deliveries.size(), 0);

    //count how many packages are dropped off
    unsigned dropped_off_count = 0;

    //variables for finding closest point
    double min_distance = std::numeric_limits<double>::max();
    double current_distance;
    unsigned min_dis_intersection_id = deliveries[0].pickUp;
    unsigned package_id = 0;
    
    double total_travel_time = 0;

    //intersection from which we check wheres closest legal point
    unsigned current_intersection = depots[depot_index];

    unsigned while_counter = 0;
    while (dropped_off_count < deliveries.size()) {
        while_counter++;
        //go through each delivery to find closest legal point
        for (unsigned i = 0; i < deliveries.size(); i++) {
//           
            if (package_check[i] == 0) {
//                
                //not picked up yet, only check pick up location
                current_distance = find_travel_times_between_intersections(current_intersection, deliveries[i].pickUp,intersection_travel_times);
                
//                
                if (current_distance < min_distance) {
                    min_distance = current_distance;
                    min_dis_intersection_id = deliveries[i].pickUp;
                    package_id = i;
                }
            } else if (package_check[i] == 1) {
//                
                //picked up yet, check drop off location
                current_distance = find_travel_times_between_intersections(current_intersection, deliveries[i].dropOff,intersection_travel_times);
//                
                if (current_distance < min_distance) {
                    min_distance = current_distance;
                    min_dis_intersection_id = deliveries[i].dropOff;
                    package_id = i;
                }

            } else if (package_check[i] == 2) {
                //already dropped off, directly go to next package
                continue;
            }
        }
        
        if(while_counter == 1 && min_distance == 0){
            std::cout<<"can not find path from depot " <<depots[depot_index]<<std::endl;
            potential_path_multistart[depot_index].second = std::numeric_limits<double>::max();
            return;
        }

        current_intersection = min_dis_intersection_id;        

        //prevent duplicates
        if(potential_path_multistart[depot_index].first.back()!=current_intersection)
            potential_path_multistart[depot_index].first.push_back(current_intersection);
        
        //move on to next state for the package
        package_check[package_id] = package_check[package_id] + 1;

        if (package_check[package_id] == 2) {            
            dropped_off_count++;
        }
        
        total_travel_time += min_distance;
        
        min_distance = std::numeric_limits<double>::max();            
    }
    
    //find closest depot to drop off the truck
   
    min_distance = std::numeric_limits<double>::max();
    unsigned id = 0;
    unsigned closest_depot_id = depots[id];

    for (unsigned i = 0; i < depots.size(); i++) {
        
        current_distance = find_travel_times_between_intersections(current_intersection, depots[i],intersection_travel_times);
        if (current_distance < min_distance && if_path_exist(current_intersection, depots[i],intersection_travel_times)) {
            min_distance = current_distance;
            closest_depot_id = depots[i];
        }
    }
    
     if(min_distance == std::numeric_limits<double>::max()){
        std::cout<<"end is not connected to any depot" <<std::endl;
        potential_path_multistart[depot_index].second = std::numeric_limits<double>::max();
        return;
    }
    total_travel_time += min_distance;   
    potential_path_multistart[depot_index].first.push_back(closest_depot_id);
    potential_path_multistart[depot_index].second = total_travel_time;



}
