
#include "Node.h"
#include "Global.h"
extern Global *global;

Node::Node(unsigned _intersection_id) {
    intersection_id = _intersection_id;
    position = getIntersectionPosition(_intersection_id);
    h_score = g_score = f_score = 0;
    in_closed_list = false;
    in_open_list = false;
    parent = NULL;
}

Node* Node::getParent(){
    return parent;
}

void Node::setParent(Node *p){
    parent = p;
}

double Node::getGScore(Node *current, double turn_penalty, bool if_update)
{
    //find a vector of s_seg connected to current
    std::vector<unsigned> current_connected = find_intersection_street_segments(current->intersection_id);
    
    //find the street_seg between current and current's parent
    unsigned seg_current_to_child;
    
    //potential street seg choices
    std::vector<unsigned> seg_current_to_child_v;
    
    //find the effective seg
    for(unsigned i = 0; i < current_connected.size();i++){
        StreetSegmentInfo info = getStreetSegmentInfo(current_connected[i]);
        
        //check one way, if in right direction
        if(info.oneWay){
            if(info.from== current->intersection_id && info.to == intersection_id){
                
                    seg_current_to_child_v.push_back(current_connected[i]); 
                   
            }
        }else{
            if(info.from == intersection_id || info.to == intersection_id){
             
                    seg_current_to_child_v.push_back(current_connected[i]); 
                   
            } 
        }
    }
    
    //check which one is optimal when there are multiple choices
    if(seg_current_to_child_v.size()>1){
        
        double op_score, op_score_0, op_score_1;
        
        //if its the starting point
        if(current->parent==NULL){
            op_score_0 = find_street_segment_travel_time(seg_current_to_child_v[0]);
            op_score_1 = find_street_segment_travel_time(seg_current_to_child_v[1]);
        }else{  

            //1st element, check if theres  a turn
            if(getStreetSegmentInfo(current->from_street_seg).streetID == getStreetSegmentInfo(seg_current_to_child_v[0]).streetID){
                op_score_0 =  find_street_segment_travel_time(seg_current_to_child_v[0]);
            }else{
                op_score_0 =  turn_penalty + find_street_segment_travel_time(seg_current_to_child_v[0]);
            }    

            //second element check if theres  a turn
            if(getStreetSegmentInfo(current->from_street_seg).streetID == getStreetSegmentInfo(seg_current_to_child_v[1]).streetID){
                op_score_1 =  find_street_segment_travel_time(seg_current_to_child_v[1]);
            }else{
                op_score_1 =  turn_penalty + find_street_segment_travel_time(seg_current_to_child_v[1]);
            }
        }
        //take the element with the smallest score
        if(op_score_1 < op_score_0){
            seg_current_to_child =  seg_current_to_child_v[1];
            op_score = op_score_1;
        }else{
            seg_current_to_child = seg_current_to_child_v[0];
            op_score = op_score_0;
        }
        
        //when need to update g score
        if(if_update)
            from_street_seg = seg_current_to_child;
        
        return current->g_score  + op_score;
        
        
    }else{
        
        seg_current_to_child = seg_current_to_child_v[0];
    
        if(if_update)
            from_street_seg = seg_current_to_child;

        if(current->parent==NULL){
            return find_street_segment_travel_time(seg_current_to_child);
        }
        if(getStreetSegmentInfo(current->from_street_seg).streetID == getStreetSegmentInfo(seg_current_to_child).streetID){
            return current->g_score + find_street_segment_travel_time(seg_current_to_child);
        }else{
            return current->g_score + turn_penalty + find_street_segment_travel_time(seg_current_to_child);
        }    
    }
}

double Node::getHScore(Node *end)
{   
    //speed of high way 33m/s -- estimate travel time to destinationin seconds
    return find_distance_between_two_points(position,end->position)/global->max_speed;
}

double Node::getGScore()
{
    return g_score;
}

double Node::getHScore()
{
    return h_score;
}

double Node::getFScore()
{
    return f_score;
}

void Node::computeScores(Node *end, double turn_penalty,bool if_update)
{
    g_score = getGScore(parent,turn_penalty,if_update);
    h_score = getHScore(end);
    f_score = g_score + h_score;
}

bool Node::hasParent()
{
    return parent != NULL;
}