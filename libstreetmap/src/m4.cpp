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
#include <fstream>
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
#define LARGE_NON_MAGIC_NUMBER 100000000

double find_distance_between_two_intersection_ids(unsigned point1, unsigned point2);

bool check_path_legal(std::vector<unsigned>& path, const std::vector<DeliveryInfo>& deliveries,
        std::unordered_map<unsigned, std::unordered_map<unsigned, std::pair<std::vector<unsigned>, double>>>& intersection_travel_times, double& travel_time,const std::vector<unsigned>& depots);

unsigned find_id(const std::vector<unsigned>& path, unsigned value);

void two_opt(std::vector<unsigned>& path, double travel_time, const std::vector<DeliveryInfo>& deliveries, const double turn_penalty,
        const std::vector<unsigned>& depots,
        std::unordered_map<unsigned, std::unordered_map<unsigned, std::pair<std::vector<unsigned>, double>>>& intersection_travel_times,
        bool time_out, auto start_time);

void populate_intersection_travel_times(std::vector<unsigned>& destinations_vec, std::unordered_set<unsigned>& destinations,
        std::unordered_map<unsigned, std::unordered_map<unsigned, std::pair<std::vector<unsigned>, double>>>& intersection_travel_times,
        const double turn_penalty);

void populate_destination(const std::vector<DeliveryInfo>& deliveries,
        const std::vector<unsigned>& depots, std::unordered_set<unsigned>& destinations, std::vector<unsigned>& destinations_vec);

void find_path_to_all_package_locations(unsigned intersect_id_start, const std::unordered_set<unsigned>& destinations,
        std::unordered_map<unsigned, std::unordered_map<unsigned, std::pair<std::vector<unsigned>, double>>>& intersection_travel_times,
        const double turn_penalty);
bool join_four_vectors(const std::vector<unsigned>& first_quarter,
        const std::vector<unsigned>& second_quarter, const std::vector<unsigned>& third_quarter, const std::vector<unsigned>last_quarter, std::vector<unsigned>& temp_path,
        std::unordered_map<unsigned, std::unordered_map<unsigned, std::pair<std::vector<unsigned>, double>>>& intersection_travel_times);

double check_and_change_if_best_path(const std::vector<unsigned>&first_quarter,
        const std::vector<unsigned>&second_quarter, const std::vector<unsigned>&third_quarter,
        const std::vector<unsigned>&last_quarter, std::vector<unsigned>& path, const std::vector<DeliveryInfo>& deliveries,
        const double turn_penalty, double current_best_time,
        const std::vector<unsigned>& depots,
        std::unordered_map<unsigned, std::unordered_map<unsigned, std::pair<std::vector<unsigned>, double>>>& intersection_travel_times);

double find_travel_times_between_intersections(unsigned start, unsigned end,
        std::unordered_map<unsigned, std::unordered_map<unsigned, std::pair<std::vector<unsigned>, double>>>& intersection_travel_times);

bool add_depots(std::vector<unsigned>& path, const std::vector<unsigned>& depots,
        std::unordered_map<unsigned, std::unordered_map<unsigned, std::pair<std::vector<unsigned>, double>>>& intersection_travel_times);

void find_path_between_intersections_from_hash(unsigned start, unsigned end,
        std::unordered_map<unsigned, std::unordered_map<unsigned, std::pair<std::vector<unsigned>, double>>>& intersection_travel_times,
        std::vector<unsigned>& path);

bool if_path_exist(unsigned start, unsigned end,
        std::unordered_map<unsigned, std::unordered_map<unsigned, std::pair<std::vector<unsigned>, double>>>& intersection_travel_times);

double multi_start(const std::vector<unsigned>& depots, const std::vector<DeliveryInfo>& deliveries,
        std::unordered_map<unsigned, std::unordered_map<unsigned, std::pair<std::vector<unsigned>, double>>>& intersection_travel_times,
        std::vector<std::pair<std::vector<unsigned>, double>>&potential_path_multistart, std::vector<unsigned>& potential_path, bool time_out, auto start_time,const float turn_penalty );

bool greedy_find_path(unsigned depot_index, const std::vector<unsigned>& depots, const std::vector<DeliveryInfo>& deliveries,
        std::unordered_map<unsigned, std::unordered_map<unsigned, std::pair<std::vector<unsigned>, double>>>& intersection_travel_times,
        std::vector<std::pair<std::vector<unsigned>, double>>&potential_path_multistart);

double remove_adjacent_intersection(std::vector<unsigned>& path,
        std::unordered_map<unsigned, std::unordered_map<unsigned, std::pair<std::vector<unsigned>, double>>>& intersection_travel_times);

std::vector<unsigned> traveling_courier(const std::vector<DeliveryInfo>& deliveries,
        const std::vector<unsigned>& depots,
        const float turn_penalty) {

    //boolean to check when the time runs out
    bool time_out = false;
    auto const start_time = chrono::high_resolution_clock::now();

//    std::cout << "--------------new test-----------" << std::endl;

    //path of street seg ids to be returned
    std::vector<unsigned> path;

    //return empty path if no deliveries or depots
    if (deliveries.empty() || depots.empty()) {
        return path;
    }

    //unordered set of pickup/ dropoff / depots locations od intersection id
    std::unordered_set<unsigned> destinations;
    std::vector<unsigned> destinations_vec;

    //    std::unordered_map<unsigned,std::vector<unsigned>> destination_delivery_id_hash;    
    populate_destination(deliveries, depots, destinations, destinations_vec);

    //hash table (interx 1, hashtable (interx 2, pair(path of street seg from interx 1 to 2, travel time)))
    std::unordered_map<unsigned, std::unordered_map<unsigned, std::pair < std::vector<unsigned>, double>>> intersection_travel_times;
    std::cout<<"before populate itt"<<std::endl;
    populate_intersection_travel_times(destinations_vec, destinations, intersection_travel_times, turn_penalty);
    std::cout<<"after populate itt"<<std::endl;

    //vector of path, and corresponding travle time of this path with each start point a diff depot
    std::vector<std::pair < std::vector<unsigned>, double>> potential_path_multistart(depots.size());

    //path of intersection ids -- after multistart
    std::vector<unsigned> potential_path;

//    double travel_time = multi_start(depots, deliveries, intersection_travel_times, potential_path_multistart, potential_path);
    double travel_time = multi_start(depots, deliveries, intersection_travel_times, potential_path_multistart, potential_path, time_out,start_time,turn_penalty);

    //    check_path_legal(potential_path, deliveries, intersection_travel_times);
    //    
    //    //depots removed
//        potential_path.erase(potential_path.begin());
//        potential_path.erase(potential_path.end()-1);
//    //    
//    two_opt(potential_path, travel_time, deliveries, turn_penalty, depots, intersection_travel_times, time_out, start_time);
//    //    //depots added
//        add_depots(potential_path, depots, intersection_travel_times);


    std::vector<unsigned> path_temp;
    for (unsigned i = 0; i < potential_path.size() - 1; ++i) {
//        std::cout << potential_path[i] << std::endl;
        find_path_between_intersections_from_hash(potential_path[i], potential_path[i + 1], intersection_travel_times, path_temp);
        path.insert(path.end(), path_temp.begin(), path_temp.end());
    }

    return path;
}

//Returns the distance between two intersection ids in meters

double find_distance_between_two_intersection_ids(unsigned point1, unsigned point2) {
    return find_distance_between_two_points(getIntersectionPosition(point1), getIntersectionPosition(point2));
}

bool check_path_legal(std::vector<unsigned>& path, const std::vector<DeliveryInfo>& deliveries,
        std::unordered_map<unsigned, std::unordered_map<unsigned, std::pair<std::vector<unsigned>, double>>>& intersection_travel_times, double& travel_time, const std::vector<unsigned>& depots) {
    //iterate through path and adjust package_vector 
    //    
    //    std::cout<<"path is: ";
    //    for (unsigned i = 0; i < path.size() - 1; i++) {
    //        std::cout<<path[i]<<" ";
    //    }
    //    std::cout<<" "<<std::endl;
    unsigned num_packages_delivered = 0;
    for (unsigned i = 0; i < deliveries.size(); i++) {
        //find the path id of the pickup address
        //assume that pickup/ dropoff will be present
        int pickup_id = find_id(path, deliveries[i].pickUp);
        if (pickup_id == -1) {
            std::cout << "NOT PICKED UP" << std::endl;
            return false;
        }
        int dropoff_id = find_id(path, deliveries[i].dropOff);
        if (dropoff_id == -1) {
            std::cout << "NOT DROPPED OFF" << std::endl;
            return false;
        }
        //check that package picked up before dropped off
        if (dropoff_id < pickup_id) {
            //                                        std::cout << "[pickup before dropoff]" << std::endl;
            //                        std::cout << "dropoff" << dropoff_id << " pickup" << pickup_id << std::endl;
            return false;
        }
        num_packages_delivered++;

    }
    if (num_packages_delivered != deliveries.size()) {
        std::cout << "[too few]" << std::endl;
        return false;
    }

    //check if a path exists between every intersection
    for (unsigned i = 0; i < path.size() - 1; i++) {
        if (!if_path_exist(path[i], path[i + 1], intersection_travel_times)) {
            std::cout << "[no path]" << std::endl;
            return false;
        }
    }
    if(!add_depots(path, depots, intersection_travel_times))
        return false;

    travel_time = remove_adjacent_intersection(path, intersection_travel_times);
    path.erase(path.begin());
    path.erase(path.end()-1);



    //    std::cout<< "new path is legal" <<std::endl;
    return true;
}

unsigned find_id(const std::vector<unsigned>& path, unsigned intersection_id) {
    for (int i = path.size() - 1; i >= 0; i--) {
        if (path[i] == intersection_id) {
            return i;
        }
    }
    return LARGE_NON_MAGIC_NUMBER;
}

//MAKE SURE TO PASS IN POTENTIAL PATH w/OUT DEPOTS! (like get rid of the first and last)

void two_opt(std::vector<unsigned>& path, double travel_time, const std::vector<DeliveryInfo>& deliveries, const double turn_penalty,
        const std::vector<unsigned>& depots,
        std::unordered_map<unsigned, std::unordered_map<unsigned, std::pair<std::vector<unsigned>, double>>>& intersection_travel_times,
        bool time_out, auto start_time) {
    //split the current path into 3 segments 
    auto current_time = chrono::high_resolution_clock::now();
    auto wall_clock = chrono::duration_cast<chrono::duration<double>> (current_time - start_time);

    std::vector<unsigned>first_quarter;
    std::vector<unsigned>second_quarter;
    std::vector<unsigned>third_quarter;
    std::vector<unsigned>last_quarter;

//    path.erase(path.begin());
//    path.erase(path.end()-1);

    double current_best_time = travel_time;
    unsigned i = path.size() / 2;
//    unsigned max_index_small = path.size()*3 / 4;
//    unsigned max_index_med = path.size()*10 / 11;
//    unsigned max_index_large = path.size()*19 / 20;
//    unsigned max_index;
    unsigned increment;
    if (path.size() <= 8){
        increment = 1;
    }else if (path.size()>6 && path.size() <= 30) {
        increment = 2;
    } else if (path.size() > 30 && path.size() <= 100) {
        increment = 3;
    } else {
        increment = 4;
    }

//    if (path.size() <= 30) {
//        max_index = max_index_small;
//    } else if (path.size() > 30 && path.size() < 100) {
//        max_index = max_index_med;
//    } else {
//        max_index = max_index_large;
//    }

    //decrease the size of the first third for every iteration 
//    while (i < max_index && (!time_out)) {
    while (i < path.size()-increment*2 && (!time_out)) {
        //        std::cout<<"a new two opt iteration entered"<<std::endl;
        if (i < 0) {
            std::cout << "i is less than 0 this loop works" << std::endl;
        }
        //        std::cout << "[line286 loop entered]" << std::endl;
        first_quarter.clear();
        second_quarter.clear();
        third_quarter.clear();
        last_quarter.clear();
        //remove start and end depots   
        //divide up current path into three segments and create iterators to the breakpoints
        unsigned first_break_index = i;
        //        unsigned second_break_index = path.size() - max_index + i;
        unsigned second_break_index = increment + i;
        unsigned third_break_index = increment*2 + i;

        for (unsigned j = 0; j < first_break_index; j++) {
            first_quarter.push_back(path[j]);
        }
        for (unsigned j = first_break_index; j < second_break_index; j++) {
            second_quarter.push_back(path[j]);
        }
        for (unsigned j = second_break_index; j < third_break_index; j++) {
            third_quarter.push_back(path[j]);
        }
        for (unsigned j = third_break_index; j < path.size(); j++) {
            last_quarter.push_back(path[j]);
        }
        if (first_quarter.size() + second_quarter.size() + third_quarter.size() + last_quarter.size() != path.size()) {
            std::cout << "---------------------not good--------------" << std::endl;
        }

        current_time = chrono::high_resolution_clock::now();
        wall_clock = chrono::duration_cast<chrono::duration<double>> (current_time - start_time);
        //check the time
        if (wall_clock.count() > 0.9 * TIME_LIMIT) {
            time_out = true;
            return;
        }

        //rearrange the path 

        //    Path Options:
        //    ABCD -> original
        current_best_time = check_and_change_if_best_path(first_quarter, second_quarter,
                third_quarter, last_quarter, path, deliveries, turn_penalty, current_best_time, depots, intersection_travel_times);

        current_time = chrono::high_resolution_clock::now();
        wall_clock = chrono::duration_cast<chrono::duration<double>> (current_time - start_time);
        //check the time
        if (wall_clock.count() > 0.95 * TIME_LIMIT) {
            time_out = true;
            return;
        }

        //ABDC 
        //check if the path is valid and if it is faster than the current best
        //if it is, change the path to this one 
        current_best_time = check_and_change_if_best_path(first_quarter, second_quarter, last_quarter,
                third_quarter, path, deliveries, turn_penalty, current_best_time, depots, intersection_travel_times);

        current_time = chrono::high_resolution_clock::now();
        wall_clock = chrono::duration_cast<chrono::duration<double>> (current_time - start_time);
        //check the time
        if (wall_clock.count() > 0.95* TIME_LIMIT) {
            time_out = true;
            return;
        }
        //ACBD 
        current_best_time = check_and_change_if_best_path(first_quarter, third_quarter,
                second_quarter, last_quarter, path, deliveries, turn_penalty, current_best_time, depots, intersection_travel_times);

        current_time = chrono::high_resolution_clock::now();
        wall_clock = chrono::duration_cast<chrono::duration<double>> (current_time - start_time);
        //check the time
        if (wall_clock.count() > 0.95 * TIME_LIMIT) {
            time_out = true;
            return;
        }
        //ACDB
        current_best_time = check_and_change_if_best_path(first_quarter, third_quarter, last_quarter, second_quarter,
                path, deliveries, turn_penalty, current_best_time, depots, intersection_travel_times);

        current_time = chrono::high_resolution_clock::now();
        wall_clock = chrono::duration_cast<chrono::duration<double>> (current_time - start_time);
        //check the time
        if (wall_clock.count() > 0.95 * TIME_LIMIT) {
            time_out = true;
            return;
        }
        //ADBC
        current_best_time = check_and_change_if_best_path(first_quarter, last_quarter,
                second_quarter, third_quarter, path, deliveries, turn_penalty, current_best_time, depots, intersection_travel_times);

        current_time = chrono::high_resolution_clock::now();
        wall_clock = chrono::duration_cast<chrono::duration<double>> (current_time - start_time);
        //check the time
        if (wall_clock.count() > 0.95 * TIME_LIMIT) {
            time_out = true;
            return;
        }
        //ADCB
        current_best_time = check_and_change_if_best_path(first_quarter, last_quarter,
                third_quarter, second_quarter, path, deliveries, turn_penalty, current_best_time, depots, intersection_travel_times);

        current_time = chrono::high_resolution_clock::now();
        wall_clock = chrono::duration_cast<chrono::duration<double>> (current_time - start_time);
        //check the time
        if (wall_clock.count() > 0.95 * TIME_LIMIT) {
            time_out = true;
            return;
        }
        //BCDA
        current_best_time = check_and_change_if_best_path(second_quarter, third_quarter,
                last_quarter, first_quarter, path, deliveries, turn_penalty, current_best_time, depots, intersection_travel_times);

        current_time = chrono::high_resolution_clock::now();
        wall_clock = chrono::duration_cast<chrono::duration<double>> (current_time - start_time);
        //check the time
        if (wall_clock.count() > 0.95 * TIME_LIMIT) {
            time_out = true;
            return;
        }
        //BCAD
        current_best_time = check_and_change_if_best_path(second_quarter, third_quarter, first_quarter,
                last_quarter, path, deliveries, turn_penalty, current_best_time, depots, intersection_travel_times);

        current_time = chrono::high_resolution_clock::now();
        wall_clock = chrono::duration_cast<chrono::duration<double>> (current_time - start_time);
        //check the time
        if (wall_clock.count() > 0.95 * TIME_LIMIT) {
            time_out = true;
            return;
        }
        //BDCA
        current_best_time = check_and_change_if_best_path(second_quarter, last_quarter,
                third_quarter, first_quarter, path, deliveries, turn_penalty, current_best_time, depots, intersection_travel_times);

        current_time = chrono::high_resolution_clock::now();
        wall_clock = chrono::duration_cast<chrono::duration<double>> (current_time - start_time);
        //check the time
        if (wall_clock.count() > 0.95 * TIME_LIMIT) {
            time_out = true;
            return;
        }
        //BDAC
        current_best_time = check_and_change_if_best_path(second_quarter, last_quarter,
                first_quarter, third_quarter, path, deliveries, turn_penalty, current_best_time, depots, intersection_travel_times);

        current_time = chrono::high_resolution_clock::now();
        wall_clock = chrono::duration_cast<chrono::duration<double>> (current_time - start_time);
        //check the time
        if (wall_clock.count() > 0.95 * TIME_LIMIT) {
            time_out = true;
            return;
        }
        //BACD
        current_best_time = check_and_change_if_best_path(second_quarter, first_quarter,
                third_quarter, last_quarter, path, deliveries, turn_penalty, current_best_time, depots, intersection_travel_times);

        current_time = chrono::high_resolution_clock::now();
        wall_clock = chrono::duration_cast<chrono::duration<double>> (current_time - start_time);
        //check the time
        if (wall_clock.count() > 0.95 * TIME_LIMIT) {
            time_out = true;
            return;
        }
        //BADC
        current_best_time = check_and_change_if_best_path(second_quarter, first_quarter,
                last_quarter, third_quarter, path, deliveries, turn_penalty, current_best_time, depots, intersection_travel_times);

        current_time = chrono::high_resolution_clock::now();
        wall_clock = chrono::duration_cast<chrono::duration<double>> (current_time - start_time);
        //check the time
        if (wall_clock.count() > 0.95 * TIME_LIMIT) {
            time_out = true;
            return;
        }
        //CDAB
        current_best_time = check_and_change_if_best_path(third_quarter, last_quarter, first_quarter, second_quarter,
                path, deliveries, turn_penalty, current_best_time, depots, intersection_travel_times);

        current_time = chrono::high_resolution_clock::now();
        wall_clock = chrono::duration_cast<chrono::duration<double>> (current_time - start_time);
        //check the time
        if (wall_clock.count() > 0.95 * TIME_LIMIT) {
            time_out = true;
            return;
        }
        //CDBA
        current_best_time = check_and_change_if_best_path(third_quarter, last_quarter, second_quarter, first_quarter,
                path, deliveries, turn_penalty, current_best_time, depots, intersection_travel_times);

        current_time = chrono::high_resolution_clock::now();
        wall_clock = chrono::duration_cast<chrono::duration<double>> (current_time - start_time);
        //check the time
        if (wall_clock.count() > 0.95 * TIME_LIMIT) {
            time_out = true;
            return;
        }
        //CADB
        current_best_time = check_and_change_if_best_path(third_quarter, first_quarter, last_quarter, second_quarter,
                path, deliveries, turn_penalty, current_best_time, depots, intersection_travel_times);

        current_time = chrono::high_resolution_clock::now();
        wall_clock = chrono::duration_cast<chrono::duration<double>> (current_time - start_time);
        //check the time
        if (wall_clock.count() > 0.95 * TIME_LIMIT) {
            time_out = true;
            return;
        }
        //CABD
        current_best_time = check_and_change_if_best_path(third_quarter, first_quarter, second_quarter, last_quarter,
                path, deliveries, turn_penalty, current_best_time, depots, intersection_travel_times);

        current_time = chrono::high_resolution_clock::now();
        wall_clock = chrono::duration_cast<chrono::duration<double>> (current_time - start_time);
        //check the time
        if (wall_clock.count() > 0.95 * TIME_LIMIT) {
            time_out = true;
            return;
        }
        //CBAD
        current_best_time = check_and_change_if_best_path(third_quarter, second_quarter, first_quarter, last_quarter,
                path, deliveries, turn_penalty, current_best_time, depots, intersection_travel_times);

        current_time = chrono::high_resolution_clock::now();
        wall_clock = chrono::duration_cast<chrono::duration<double>> (current_time - start_time);
        //check the time
        if (wall_clock.count() > 0.95 * TIME_LIMIT) {
            time_out = true;
            return;
        }
        //CBDA
        current_best_time = check_and_change_if_best_path(third_quarter, second_quarter, last_quarter, first_quarter,
                path, deliveries, turn_penalty, current_best_time, depots, intersection_travel_times);

        current_time = chrono::high_resolution_clock::now();
        wall_clock = chrono::duration_cast<chrono::duration<double>> (current_time - start_time);
        //check the time
        if (wall_clock.count() > 0.95 * TIME_LIMIT) {
            time_out = true;
            return;
        }
        //DABC
        current_best_time = check_and_change_if_best_path(last_quarter, first_quarter, second_quarter,
                third_quarter, path, deliveries, turn_penalty, current_best_time, depots, intersection_travel_times);

        current_time = chrono::high_resolution_clock::now();
        wall_clock = chrono::duration_cast<chrono::duration<double>> (current_time - start_time);
        //check the time
        if (wall_clock.count() > 0.95 * TIME_LIMIT) {
            time_out = true;
            return;
        }
        //DACB
        current_best_time = check_and_change_if_best_path(last_quarter, first_quarter, third_quarter, second_quarter,
                path, deliveries, turn_penalty, current_best_time, depots, intersection_travel_times);

        current_time = chrono::high_resolution_clock::now();
        wall_clock = chrono::duration_cast<chrono::duration<double>> (current_time - start_time);
        //check the time
        if (wall_clock.count() > 0.95 * TIME_LIMIT) {
            time_out = true;
            return;
        }
        //DCAB
        current_best_time = check_and_change_if_best_path(last_quarter, third_quarter, first_quarter, second_quarter,
                path, deliveries, turn_penalty, current_best_time, depots, intersection_travel_times);

        current_time = chrono::high_resolution_clock::now();
        wall_clock = chrono::duration_cast<chrono::duration<double>> (current_time - start_time);
        //check the time
        if (wall_clock.count() > 0.95 * TIME_LIMIT) {
            time_out = true;
            return;
        }
        //DCBA
        current_best_time = check_and_change_if_best_path(last_quarter, third_quarter, second_quarter, first_quarter,
                path, deliveries, turn_penalty, current_best_time, depots, intersection_travel_times);

        current_time = chrono::high_resolution_clock::now();
        wall_clock = chrono::duration_cast<chrono::duration<double>> (current_time - start_time);
        //check the time
        if (wall_clock.count() > 0.95 * TIME_LIMIT) {
            time_out = true;
            return;
        }
        //DBCA
        current_best_time = check_and_change_if_best_path(last_quarter, second_quarter, third_quarter, first_quarter,
                path, deliveries, turn_penalty, current_best_time, depots, intersection_travel_times);

        current_time = chrono::high_resolution_clock::now();
        wall_clock = chrono::duration_cast<chrono::duration<double>> (current_time - start_time);
        //check the time
        if (wall_clock.count() > 0.95 * TIME_LIMIT) {
            time_out = true;
            return;
        }
        //DBAC
        current_best_time = check_and_change_if_best_path(last_quarter, second_quarter, first_quarter, third_quarter,
                path, deliveries, turn_penalty, current_best_time, depots, intersection_travel_times);

        current_time = chrono::high_resolution_clock::now();
        wall_clock = chrono::duration_cast<chrono::duration<double>> (current_time - start_time);
        //check the time
        if (wall_clock.count() > 0.95 * TIME_LIMIT) {
            time_out = true;
            return;
        }

        i = i + increment;
        //        if (i >= max_index) {
        //            i = 2;
        //        }
        if (i >= path.size() - increment*2) {
            i = increment;
        }
//        path.erase(path.begin());
//        path.erase(--path.end());

    }
}

bool join_four_vectors(const std::vector<unsigned>& first_quarter,
        const std::vector<unsigned>& second_quarter, const std::vector<unsigned>& third_quarter, const std::vector<unsigned>last_quarter, std::vector<unsigned>& temp_path,
        std::unordered_map<unsigned, std::unordered_map<unsigned, std::pair<std::vector<unsigned>, double>>>& intersection_travel_times) {

    temp_path = first_quarter;
    //check if path between first_third.back(), and second_third(front)

    if (!if_path_exist(first_quarter.back(), second_quarter.front(), intersection_travel_times)) {
        return false;
    }
    temp_path.insert(temp_path.end(), second_quarter.begin(), second_quarter.end());

    if (!if_path_exist(second_quarter.back(), third_quarter.front(), intersection_travel_times)) {
        return false;
    }
    temp_path.insert(temp_path.end(), third_quarter.begin(), third_quarter.end());
    if (!if_path_exist(third_quarter.back(), last_quarter.front(), intersection_travel_times)) {
        return false;
    }
    temp_path.insert(temp_path.end(), last_quarter.begin(), last_quarter.end());
    return true;
}

double check_and_change_if_best_path(const std::vector<unsigned>&first_quarter,
        const std::vector<unsigned>&second_quarter, const std::vector<unsigned>&third_quarter,
        const std::vector<unsigned>&last_quarter, std::vector<unsigned>& path, const std::vector<DeliveryInfo>& deliveries,
        const double turn_penalty, double current_best_time,
        const std::vector<unsigned>& depots,
        std::unordered_map<unsigned, std::unordered_map<unsigned, std::pair<std::vector<unsigned>, double>>>& intersection_travel_times) {

    //temporarily concatenate the 3 vectors 
    std::vector<unsigned>temp_path;
    if (!join_four_vectors(first_quarter, second_quarter, third_quarter, last_quarter, temp_path, intersection_travel_times)) {
        return current_best_time;
    }

    double travel_time;
    //    std::cout << "[check path entered]" << std::endl;
    //check if legal

    bool calculate_if_legal = check_path_legal(temp_path, deliveries, intersection_travel_times, travel_time, depots);
    //    std::cout<<" [calculate_if_legal] "<<calculate_if_legal<<std::endl;
    if (calculate_if_legal) {
        //recalculate the travel time for this path
        //        std::cout << "[[[legal path found!]]]!!!!!!!!!!!!!!!!" << std::endl;

        //ADD DEPOTS TO START + END
        //        add_depots(temp_path, depots, intersection_travel_times);
        //        std::cout<<"legal loop entered_________" << std::endl; 
        if (travel_time < current_best_time) {
            //if the time is less than the current best travel time, change the path to this one 
            path = temp_path;
            current_best_time = travel_time;
            std::cout << "path time better, updated" << std::endl;
        }


    }

    return current_best_time;

}

bool add_depots(std::vector<unsigned>& path, const std::vector<unsigned>& depots,
        std::unordered_map<unsigned, std::unordered_map<unsigned, std::pair<std::vector<unsigned>, double>>>& intersection_travel_times) {
    //add the starting depot
    //find depot with smallest travel time from path[0]
    double min_travel_time_start = std::numeric_limits<double>::max();
    double min_travel_time_end = std::numeric_limits<double>::max();
    unsigned start_depot = 0;
    unsigned end_depot = 0;

    for (unsigned i = 0; i < depots.size(); i++) {

        double travel_time = find_travel_times_between_intersections(depots[i], path.front(), intersection_travel_times);
        if (if_path_exist(depots[i], path.front(), intersection_travel_times) && travel_time < min_travel_time_start) {
            start_depot = i;
            min_travel_time_start = travel_time;
        }
        double travel_time_end = find_travel_times_between_intersections(path.back(), depots[i], intersection_travel_times);
        if (if_path_exist(path.back(), depots[i], intersection_travel_times) && travel_time_end < min_travel_time_end) {
            end_depot = i;
            min_travel_time_end = travel_time_end;
        }
    }
    
    if(min_travel_time_start == std::numeric_limits<double>::max() || min_travel_time_end == std::numeric_limits<double>::max()){
        std::cout<<"start pr end not connected to any depot"<<std::endl;
        return false;
    }

    //insert into path
    std::vector<unsigned> front_path;
    front_path.push_back(depots[start_depot]);

    front_path.insert(front_path.end(), path.begin(), path.end());
    //    path.insert(--path.begin(), front_path.begin(), front_path.end());
    front_path.push_back(depots[end_depot]);
    path = front_path;
    return true;


}
//load the above data structure by iterating through all the deliveries and depots

void populate_destination(const std::vector<DeliveryInfo>& deliveries,
        const std::vector<unsigned>& depots, std::unordered_set<unsigned>& destinations, std::vector<unsigned>& destinations_vec) {

    //populate the destination set with pickup / drop off location nodes
    for (unsigned i = 0; i < deliveries.size(); i++) {
        if (destinations.find(deliveries[i].pickUp) == destinations.end()) {

            destinations.insert(deliveries[i].pickUp);
            destinations_vec.push_back(deliveries[i].pickUp);
        }
        if (destinations.find(deliveries[i].dropOff) == destinations.end()) {
            destinations.insert(deliveries[i].dropOff);
            destinations_vec.push_back(deliveries[i].dropOff);
        }
    }

    //populate the destination set with depots location nodes
    for (unsigned i = 0; i < depots.size(); i++) {
        if (destinations.find(depots[i]) == destinations.end()) {
            destinations.insert(depots[i]);
            destinations_vec.push_back(depots[i]);
        }
    }
}

void populate_intersection_travel_times(std::vector<unsigned>& destinations_vec, std::unordered_set<unsigned>& destinations,
        std::unordered_map<unsigned, std::unordered_map<unsigned, std::pair<std::vector<unsigned>, double>>>& intersection_travel_times,
        const double turn_penalty) {
    static std::ofstream perf("populate_intersection_travel_times.csv");
    auto const start_t = std::chrono::high_resolution_clock::now();
#pragma omp parallel for
    for (unsigned i = 0; i < destinations_vec.size(); i++) {
        std::unordered_map<unsigned, std::pair < std::vector<unsigned>, double>> destination_travel_time_pickup;
        std::pair <unsigned, std::unordered_map<unsigned, std::pair < std::vector<unsigned>, double>>> intersection_hash_pickup(destinations_vec[i], destination_travel_time_pickup);
#pragma omp critical
        intersection_travel_times.insert(intersection_hash_pickup);
        find_path_to_all_package_locations(destinations_vec[i], destinations, intersection_travel_times, turn_penalty);
    }
    
    auto current_time = std::chrono::high_resolution_clock::now();
    auto wall_clock = std::chrono::duration_cast<std::chrono::duration<double>> (current_time - start_t);
    std::cout<<"populate_intersection_travel_times, toronto, multithreading, "<<wall_clock.count()<<std::endl;
    perf << "populate_intersection_travel_times, toronto, multithreading, "<<wall_clock.count()<<std::endl;
}

void find_path_to_all_package_locations(unsigned intersect_id_start, const std::unordered_set<unsigned>& destinations,
        std::unordered_map<unsigned, std::unordered_map<unsigned, std::pair<std::vector<unsigned>, double>>>& intersection_travel_times,
        const double turn_penalty) {
    //path of street seg ids
    std::vector<unsigned> path;
    std::vector<Node*> intersection_node;
    unsigned numIntersection = getNumberOfIntersections();
    intersection_node.resize(numIntersection);

    for (unsigned i = 0; i < numIntersection; i++) {
        intersection_node[i] = new Node(i);
    }

    Node *start = NULL;
    Node *current = NULL;
    Node *child = NULL;

    start = intersection_node[intersect_id_start];

    // Define the open and the close list
    std::list<Node*> closedList;
    std::priority_queue<Node*, std::vector<Node*>, smaller_f_score> openList;

    unsigned destination_found_counter = 0;
    unsigned iteration_counter = 0;
    // Add the start point to the openList
    openList.push(start);
    start->in_open_list = true;

    while (!openList.empty()) {
        iteration_counter++;
        current = openList.top();
        //        std::cout<<" [current] "<<current->intersection_id << " [iterated] " <<iteration_counter<<std::endl;

        // Remove the current point from the openList
        openList.pop();
        current->in_open_list = false;

        //check if equal to any node in the poi set
        auto it1 = destinations.find(current->intersection_id);

        if (it1 != destinations.end() && (*it1) != intersect_id_start && current->in_closed_list == false) {


            destination_found_counter++;
            path.clear();

            Node* current_traceback = NULL;
            current_traceback = current;
            // Resolve the path starting from the end point
            while (current_traceback->hasParent() && current_traceback != start) {
                path.push_back(current_traceback->from_street_seg);
                current_traceback = current_traceback->getParent();
            }

            std::reverse(path.begin(), path.end());
            double travelTime = compute_path_travel_time(path, turn_penalty);

            //           std::cout<<"[start] "<<intersect_id_start<<" [end] "<<(*it1)<< "[path length] "<<path.size()<<" [current id] "<<current->intersection_id<<std::endl;

            std::pair <std::vector<unsigned>, double> destination_path_travel_time = std::make_pair(path, travelTime);
            std::pair <unsigned, std::pair < std::vector<unsigned>, double>> destination_hash = std::make_pair((*it1), destination_path_travel_time);

#pragma omp critical
            {
                auto it2 = intersection_travel_times.find(intersect_id_start);

                if (it2 != intersection_travel_times.end()) {
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
                    child->computeScores(child, turn_penalty, 1);
                    //                        
                    openList.push(child);
                }
            } else {

                child->in_open_list = true;

                // Compute it's g, h and f score
                child->setParent(current);
                child->computeScores(child, turn_penalty, 1);

                //Add it to the openList with current point as parent
                openList.push(child);
            }
        }
    }

    for (unsigned i = 0; i < intersection_node.size(); i++) {
        delete intersection_node[i];
    }
    intersection_node.clear();
}

double find_travel_times_between_intersections(unsigned start, unsigned end,
        std::unordered_map<unsigned, std::unordered_map<unsigned, std::pair<std::vector<unsigned>, double>>>& intersection_travel_times) {
    auto it2 = intersection_travel_times.find(start);
    if (it2 != intersection_travel_times.end()) {
        auto it3 = it2->second.find(end);

        if (it3 != it2->second.end())
            return (it3->second).second;
        else {
            if (start != end)
                std::cout << " [start] " << start << " [end] " << end << " cannot find end --- find travel times" << std::endl;

            return 0;
        }

    }
    std::cout << " [start] " << start << " [end] " << end << " cannnot find start --- find travel times" << std::endl;
    return 0;

}

void find_path_between_intersections_from_hash(unsigned start, unsigned end,
        std::unordered_map<unsigned, std::unordered_map<unsigned, std::pair<std::vector<unsigned>, double>>>& intersection_travel_times,
        std::vector<unsigned>& path) {

    auto it2 = intersection_travel_times.find(start);
    if (it2 != intersection_travel_times.end()) {
        auto it3 = it2->second.find(end);

        if (it3 != it2->second.end()) {
            std::vector<unsigned> temp = (it3->second).first;
            path = temp;
            return;
        } else {
            if (start != end)
                std::cout << " [start] " << start << " [end] " << end << " cannnot find end ---- find path" << std::endl;

            return;
        }

    }
    std::cout << " [start] " << start << " [end] " << end << " cannnot find start" << std::endl;
    return;


}

bool if_path_exist(unsigned start, unsigned end,
        std::unordered_map<unsigned, std::unordered_map<unsigned, std::pair<std::vector<unsigned>, double>>>& intersection_travel_times) {

    if (start == end)
        return true;

    auto it2 = intersection_travel_times.find(start);
    if (it2 != intersection_travel_times.end()) {
        auto it3 = it2->second.find(end);
        if (it3 != it2->second.end()) {
            return true;
        } else {
            std::cout << " [start] " << start << " [end] " << end << " cannnot find end" << std::endl;
            return false;
        }
    }
    std::cout << " [start] " << start << " [end] " << end << " cannnot find start" << std::endl;
    return false;
}

double multi_start(const std::vector<unsigned>& depots, const std::vector<DeliveryInfo>& deliveries,
        std::unordered_map<unsigned, std::unordered_map<unsigned, std::pair<std::vector<unsigned>, double>>>& intersection_travel_times,
        std::vector<std::pair<std::vector<unsigned>, double>>&potential_path_multistart, std::vector<unsigned>& potential_path, bool time_out, auto start_time, const float turn_penalty) {

#pragma omp parallel for
    for (unsigned i = 0; i < depots.size(); i++) {
        //populate potential_path_multistart
        std::vector<unsigned> path_each;

        double travel_time = 0;
        potential_path_multistart[i] = std::make_pair(path_each, travel_time);

        if(!greedy_find_path(i, depots, deliveries, intersection_travel_times, potential_path_multistart)){
            continue;
        }
        potential_path_multistart[i].first.erase(potential_path.begin());
        potential_path_multistart[i].first.erase(potential_path.end()-1);
    //    
        two_opt(potential_path_multistart[i].first, potential_path_multistart[i].second, deliveries, turn_penalty, depots, intersection_travel_times, time_out, start_time);
    //    //depots added
        add_depots(potential_path_multistart[i].first, depots, intersection_travel_times);
        
    }
    //return path with shortest travel time
    auto it = std::min_element(potential_path_multistart.begin(), potential_path_multistart.end(), shorter_travel_time());
    potential_path = it->first;
    return it -> second;
}

bool greedy_find_path(unsigned depot_index, const std::vector<unsigned>& depots, const std::vector<DeliveryInfo>& deliveries,
        std::unordered_map<unsigned, std::unordered_map<unsigned, std::pair<std::vector<unsigned>, double>>>& intersection_travel_times,
        std::vector<std::pair<std::vector<unsigned>, double>>&potential_path_multistart) {

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
                current_distance = find_travel_times_between_intersections(current_intersection, deliveries[i].pickUp, intersection_travel_times);

                //                
                if (current_distance < min_distance) {
                    min_distance = current_distance;
                    min_dis_intersection_id = deliveries[i].pickUp;
                    package_id = i;
                }
            } else if (package_check[i] == 1) {
                //                
                //picked up yet, check drop off location
                current_distance = find_travel_times_between_intersections(current_intersection, deliveries[i].dropOff, intersection_travel_times);
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

        if (while_counter == 1 && min_distance == 0) {
            std::cout << "can not find path from depot " << depots[depot_index] << std::endl;
            potential_path_multistart[depot_index].second = std::numeric_limits<double>::max();
            return false;
        }

        current_intersection = min_dis_intersection_id;

        //prevent duplicates
        if (potential_path_multistart[depot_index].first.back() != current_intersection)
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

        current_distance = find_travel_times_between_intersections(current_intersection, depots[i], intersection_travel_times);
        if (current_distance < min_distance && if_path_exist(current_intersection, depots[i], intersection_travel_times)) {
            min_distance = current_distance;
            closest_depot_id = depots[i];
        }
    }

    if (min_distance == std::numeric_limits<double>::max()) {
        std::cout << "end is not connected to any depot" << std::endl;
        potential_path_multistart[depot_index].second = std::numeric_limits<double>::max();
        return false;
    }
    total_travel_time += min_distance;
    potential_path_multistart[depot_index].first.push_back(closest_depot_id);
    potential_path_multistart[depot_index].second = total_travel_time;
    
    return true;
}

double remove_adjacent_intersection(std::vector<unsigned>& path,
        std::unordered_map<unsigned, std::unordered_map<unsigned, std::pair<std::vector<unsigned>, double>>>& intersection_travel_times) {
    double travel_time = 0;
    for (auto i = path.begin(); i != path.end() - 1; i++) {
        travel_time += find_travel_times_between_intersections(*i, *(i + 1), intersection_travel_times);
        if (*i == *(i + 1))
            path.erase(i);
    }
    return travel_time;
}
