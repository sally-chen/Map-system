/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Node.h
 * Author: chenc166
 *
 * Created on March 11, 2017, 2:21 PM
 */



#ifndef NODE_H
#define NODE_H

#include "StreetsDatabaseAPI.h"
#include "OSMDatabaseAPI.h"
#include "m1.h"


class Node {
public:
    Node(unsigned intersection_id);
    unsigned intersection_id;
    LatLon position;
    double h_score;
    double g_score;
    double f_score;
    bool in_closed_list;
    bool in_open_list;
    Node* parent;
    unsigned from_street_seg;
    
    Node* getParent();
    void setParent(Node *p);
    void computeScores(Node *end, double turn_penalty,bool if_update);
    bool hasParent();
    double getFScore();
    double getGScore();
    double getHScore(Node* end);
    double getHScore();
    double getGScore(Node *current, double turn_penalty,bool if_update);
    
    
private:
    

};

#endif /* NODE_H */

