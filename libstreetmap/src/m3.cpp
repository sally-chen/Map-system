/**
 reference code: http://xpac27.github.io/a-star-pathfinder-c%2B%2B-implementation.html
 */
#include "StreetsDatabaseAPI.h"
#include "OSMDatabaseAPI.h"
#include "Node.h"
#include "Global.h"
#include <algorithm>    // std::reverse
#include <list>
#include <chrono>
#include <queue>
#include <cassert>
#include <functional>
#include <vector>
#include <fstream>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <utility>      // std::pair
#include "m3.h"
#include "math.h"
#include "m4.h"

extern Global *global;
using namespace std;
struct smaller_f_score {

    bool operator()(Node*& node1, Node*& node2) const {
        return node1->f_score > node2->f_score;
    }
};

unsigned find_street_seg_from_intersections(unsigned intersection_id1, unsigned intersection_id2);


double compute_path_travel_time(const std::vector<unsigned>& path,
        const double turn_penalty) {
    double travel_time = 0.0;
    if (path.empty()) {
        std::cout << "path is empty" << std::endl;
        return 0.0;
    } else {
        for (unsigned i = 0; i < path.size() - 1; i++) {
            unsigned streetID_prev = getStreetSegmentInfo(path[i]).streetID;
            unsigned streetID_next = getStreetSegmentInfo(path[i + 1]).streetID;

            if (streetID_prev == streetID_next) {
                travel_time += find_street_segment_travel_time(path[i + 1]);
            } else {
                travel_time += find_street_segment_travel_time(path[i + 1]) + turn_penalty;
            }
        }
        return travel_time += find_street_segment_travel_time(path[0]);
    }
}

std::vector<unsigned> find_path_between_intersections(const unsigned intersect_id_start,
        const unsigned intersect_id_end,
        const double turn_penalty) {
    
    static std::ofstream perf("inter_inter.csv");
    auto const start_time = chrono::high_resolution_clock::now();

    //path of intersection id
    std::vector<unsigned> path;
    
    Node *start = NULL;
    Node *end = NULL;
    Node *current = NULL;
    Node *child = NULL;

    bool start_out_bound = false;
    bool end_out_bound = false;
    
    //check if provided intersection id is out of bound
    if(intersect_id_start > getNumberOfIntersections()-1){
        start_out_bound = true; 
        std::cout<<"start out of bound"<<std::endl;
        return path;

    }    
    
    if(intersect_id_end > getNumberOfIntersections()-1){
        end_out_bound = true; 
        std::cout<<"end out of bound"<<std::endl;
        return path;
    }
    //find start and end load
    start = global->intersection_node[intersect_id_start];
    end = global->intersection_node[intersect_id_end];
    
    if(!start_out_bound && !end_out_bound && !(start==end)){
        
        // Define the open and the close list
        std::list<Node*> closedList;
        std::priority_queue<Node*, std::vector<Node*>, smaller_f_score> openList;

        // Add the start point to the openList    
        openList.push(start);
        start->in_open_list = true;

        //look through the intersections that current intersection connects to
        while (current != end && !openList.empty()){
           
            //make current equal to the intersection with smallest f_score 
            current = openList.top();
       
            openList.pop();
            current->in_open_list = false;

            // Stop if we reached the end
            if (current == end)
                break;
            
            // Add the current point to the closedList
            closedList.push_back(current);
            current->in_closed_list = true;

            //find adjacent intersections -- it automatically excludes the one-way street
            std::vector<unsigned> adjacent_intersections = find_adjacent_intersections(current->intersection_id);
            
            // Get all current's adjacent walkable points
            for (unsigned j = 0; j < adjacent_intersections.size(); j++) {

                child = global->intersection_node[adjacent_intersections[j]];

                //pass this neighbor if it is already in closed list (evaluated)
                //or is not reachable from current node (one way)
                if (child->in_closed_list)
                    continue;


                if (child->in_open_list) {
                    // If it has a worse g score than the one that pass through the current point
                    // then its path is improved when it's parent is the current point
                    if (child->getGScore() > child->getGScore(current, turn_penalty, 0)) {
                        std::vector<Node*> temp;
                        
                        // Change its parent and g score
                        child->setParent(current);
                        child->computeScores(end, turn_penalty, 1);
                       
                        openList.push(child);                   
                       
                    }
                }else{

                    child->in_open_list = true;

                    // Compute it's g, h and f score
                    child->setParent(current);
                    child->computeScores(end, turn_penalty, 1);
                    //Add it to the openList with current point as parent
                    openList.push(child);
                        
                }
            }
        }

        for(unsigned j = 0; j < global->intersection_node.size();j++){
            global->intersection_node[j]->in_open_list = false;
            global->intersection_node[j]->in_closed_list = false;
        }
        
        //find the path in street seg
        while (current->hasParent() && current != start) {
           
            path.push_back(current->from_street_seg);
            current = current->getParent();
        }
        
        //reverse path so its in right order
        std::reverse(path.begin(),path.end());
     
    }
    
    auto current_time = chrono::high_resolution_clock::now();
    auto wall_clock = chrono::duration_cast<chrono::duration<double>> (current_time - start_time);
    
    std::cout<<"fins path between intersections"<<wall_clock.count()<<std::endl;
    
    perf << "interx_interx, london, "<<wall_clock.count()<<std::endl;
    return path;
}


std::vector<unsigned> find_path_to_point_of_interest(const unsigned intersect_id_start, 
                                               const std::string point_of_interest_name,
                                               const double turn_penalty){
    
    static std::ofstream perf("inter_poi.csv");
    auto start_time = chrono::high_resolution_clock::now();
    
    std::vector<unsigned> path;
     
     std::unordered_set<Node*> POI_end_nodes;
  
    //find range of iterators pointing to elements of matching values
    auto range = global->POI_name_inters.equal_range(point_of_interest_name);

    //if found, add the street ids of the given name to the found vector 
    for (std::unordered_map<std::string, Node*>::iterator iter = range.first; iter != range.second; iter++) {

        POI_end_nodes.insert(iter->second);
    }     

    //check if poi is found
    bool no_POI = false;
    if(POI_end_nodes.empty()){
        no_POI = true;
        std::cout<<"theres no poi with that name"<<std::endl;
        return path;
    }
  
    Node *start = NULL;
    
    Node *current = NULL;
    Node *child = NULL;

    bool start_out_bound = false;
   
    //check if out of bound
    if(intersect_id_start > getNumberOfIntersections()-1){
        start_out_bound = true; 
        std::cout<<"start out of bound"<<std::endl;
        return path;

    }
    start = global->intersection_node[intersect_id_start];
   
    
    if(!start_out_bound && !no_POI){

        // Define the open and the close list
        std::list<Node*>::iterator i;
        std::list<Node*> closedList;
        std::priority_queue<Node*,std::vector<Node*>,smaller_f_score> openList;        

        // Add the start point to the openList
        openList.push(start);
        start->in_open_list = true;
        
        bool found = false;

        while (!found && !openList.empty()){
            current = openList.top();
            
            // Remove the current point from the openList
            openList.pop();
            current->in_open_list = false;

            //check if equal to any node in the poi set
            std::unordered_set<Node*>::iterator it1 = POI_end_nodes.find (current);

            if ( it1 != POI_end_nodes.end() ){
                found = true;               
            }

            if (found)
                break;


            // Add the current point to the closedList
            closedList.push_back(current);
            current->in_closed_list = true;

            //find adjacent intersections -- it automatically excludes the one-way street
            std::vector<unsigned> adjacent_intersections = find_adjacent_intersections(current->intersection_id);

            // Get all current's adjacent walkable points
            for (unsigned j = 0; j < adjacent_intersections.size(); j++) {

                child = global->intersection_node[adjacent_intersections[j]];

                //pass this neighbor if it is already in closed list (evaluated)
                //or is not reachable from current node (one way)
                if (child->in_closed_list)
                    continue;

                if (child->in_open_list) {
                    // If it has a wroste g score than the one that pass through the current point
                    // then its path is improved when it's parent is the current point
                    if (child->getGScore() > child->getGScore(current, turn_penalty, 0)) {
                        std::vector<Node*> temp;
                       
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
        
        //reset closed and open list
        for (std::list<Node*>::iterator n = closedList.begin(); n != closedList.end(); ++n){
            (*n)->in_closed_list = false;
        }
        while(!openList.empty()){
            openList.top()->in_open_list=false;
            openList.pop();
        }

        // Resolve the path starting from the end point
        while (current->hasParent() && current != start) {
            path.push_back(current->from_street_seg);
            current = current->getParent();
        }
        
       std::reverse(path.begin(),path.end());
    }
    
    auto current_time = chrono::high_resolution_clock::now();
    auto wall_clock = chrono::duration_cast<chrono::duration<double>> (current_time - start_time);
    
    std::cout<<"fins path between intersection to POI"<<wall_clock.count()<<std::endl;
    
    perf << "inter_poi, london, "<<wall_clock.count()<<std::endl;
    return path;
}

void draw_directions_menu() {
    set_drawing_buffer(ON_SCREEN);

    set_coordinate_system(GL_SCREEN);
    settextrotation(0);
    //erase the search and mag glass
    setcolor(t_color(62, 134, 249));
    fillrect(45, 10, 315, 165);
    Surface direction_logo = load_png_from_file("directions_arrow.png");
    draw_surface(direction_logo, 52, 17);

    //draw the text bars
    setcolor(t_color(83, 148, 252));
    fillrect(95, 20, 310, 45);
    fillrect(95, 50, 310, 75);
    fillrect(95, 100, 310, 125);
    fillrect(95, 130, 310, 155);

    //place the arrow between the start and end points
    Surface down_arrow = load_png_from_file("arrow_down.png");
    draw_surface(down_arrow, 190, 80);

    //replace the text as the input changes 
    if (global->street_names.size() == 0) {
        setcolor(t_color(255, 255, 255));
        setfontsize(15);

        if (global->typing) {
            drawtext(100, 24, global->user_query, 305, 49, 0);
            drawtext(100, 54, "Start Point Street 2", 305, 79, 0);
            drawtext(100, 104, "End Point Street 1/POI ", 305, 129, 0);
            drawtext(100, 134, "End Point Street 2", 305, 159, 0);
        } else {
            //write the original text 
            drawtext(100, 24, "Start Point Street 1", 305, 49, 0);
            drawtext(100, 54, "Start Point Street 2", 305, 79, 0);
            drawtext(100, 104, "End Point Street 1/POI ", 305, 129, 0);
            drawtext(100, 134, "End Point Street 2", 305, 159, 0);
        }


    } else if (global->street_names.size() == 1) {

        setcolor(t_color(255, 255, 255));
        setfontsize(15);
        drawtext(100, 24, global->street_names[0], 305, 49, 0);

        if (global->typing) {
            drawtext(100, 54, global->user_query, 305, 79, 0);
            drawtext(100, 104, "End Point Street 1/POI ", 305, 129, 0);
            drawtext(100, 134, "End Point Street 2", 305, 159, 0);

        } else {
            drawtext(100, 54, "Start Point Street 2", 305, 79, 0);
            drawtext(100, 104, "End Point Street 1/POI ", 305, 129, 0);
            drawtext(100, 134, "End Point Street 2", 305, 159, 0);
        }

    } else if (global->street_names.size() == 2) {

        setcolor(t_color(255, 255, 255));
        setfontsize(15);
        drawtext(100, 24, global->street_names[0], 305, 49, 0);
        drawtext(100, 54, global->street_names[1], 305, 79, 0);
        if (global->typing) {
            drawtext(100, 104, global->user_query, 305, 129, 0);
            drawtext(100, 134, "End Point Street 2", 305, 159, 0);
        } else {
            drawtext(100, 104, "End Point Street 1/POI ", 305, 129, 0);
            drawtext(100, 134, "End Point Street 2", 305, 159, 0);
        }
    } else if (global->street_names.size() == 3) {

        setcolor(t_color(255, 255, 255));
        setfontsize(15);
        drawtext(100, 24, global->street_names[0], 305, 49, 0);
        drawtext(100, 54, global->street_names[1], 305, 79, 0);
        drawtext(100, 104, global->street_names[2], 305, 129, 0);

        if (global->typing) {
            drawtext(95, 134, global->user_query, 305, 159, 0);
        } else {
            drawtext(95, 134, "End Point Street 2", 305, 159, 0);

        }

    } else if (global->street_names.size() == 4) {

        setcolor(t_color(255, 255, 255));
        setfontsize(15);
        drawtext(100, 24, global->street_names[0], 305, 49, 0);
        drawtext(100, 54, global->street_names[1], 305, 79, 0);
        drawtext(100, 104, global->street_names[2], 305, 129, 0);
        drawtext(100, 134, global->street_names[3], 305, 159, 0);
        //perform mapping actions
        global->typing = false;
        global->directions_active = false;
    }


    set_coordinate_system(GL_WORLD);

    return;
}

void direction_menu_actions(float x, float y) {

    //check where the user clicked
    t_point clicked(xworld_to_scrn_fl(x), yworld_to_scrn_fl(y));

    if (global->directions_active && (clicked.x > 45 && clicked.x < 315 && clicked.y > 10 && clicked.y < 165)) {
        global->do_not_draw_pin = true;
    }

    if (clicked.x > 45 && clicked.x < 82 && clicked.y > 17 && clicked.y < 40) {
        //if they clicked on the menu button, drop the directions menu
        global-> directions_active = true;
        global->do_not_draw_pin = true;
        draw_directions_menu();
        //accept input
        //show results
        //display errors 
    } else {
        global->directions_active = false;
        return;
    }


}


