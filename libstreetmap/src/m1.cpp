/**********************************************************************/
/*Implements all the map operation functions using StreetDatabaseAPI and global class*/
/**********************************************************************/

#include "StreetsDatabaseAPI.h"
#include "OSMDatabaseAPI.h"
#include "math.h"
#include "Global.h"
#include "m2.h"
#include <algorithm>
#include <map>

using namespace std;

Global *global;

bool load_map(std::string map_path) {
    //load OSM data based on selected map in map path
    std::string osm_string = "osm.bin";
    std::string osm_map_temp = map_path;
    const std::string osm_map = osm_map_temp.replace(osm_map_temp.end()-11, osm_map_temp.end(), osm_string);
    
    
    
    //Load your map related data structures here
    if (loadStreetsDatabaseBIN(map_path)) {
        if(!loadOSMDatabaseBIN(osm_map))
        return false;

        //dynamically allocate the pointer to and Global object
        global = new Global;

    } else {
        std::cout << "map not loaded \n";
        return false;
    }

    return true;
}

void close_map() {

    //delete the allocated pointer to global object
    delete global;

    //close map
    clear_m2_global_var();
    closeStreetDatabase();
    closeOSMDatabase();
}

//Returns the street segments for the given intersection 

std::vector<unsigned> find_intersection_street_segments(unsigned intersection_id) {
    return global->intersection_street_segments[intersection_id];
}


//Returns the street names at the given intersection (includes duplicate street names in returned vector)

std::vector<std::string> find_intersection_street_names(unsigned intersection_id) {
    //close_map();
    std::vector<std::string> streetNames;
    std::vector<unsigned> streetSegments;
    streetSegments = global->intersection_street_segments[intersection_id];
    int i = 0;
    for (std::vector<unsigned>::iterator it = streetSegments.begin(); it < streetSegments.end(); it++) {
        i++;
        StreetIndex streetID = getStreetSegmentInfo(*it).streetID;
        std::string streetName = getStreetName(streetID);
        streetNames.push_back(streetName);
    }
    return streetNames;
}

//Returns the nearest point of interest to the given position

unsigned find_closest_point_of_interest(LatLon my_position) {

    //call find_closest_POI from the global class
    return global-> find_closest_POI(my_position);
}

//Returns the the nearest intersection to the given position

unsigned find_closest_intersection(LatLon my_position) {

    //call find_closest_INTERS from the global class
    return global->find_closest_INTERS(my_position);
}

//Checks whether two intersections are connected by a single street segment

bool are_directly_connected(unsigned intersection_id1, unsigned intersection_id2) {
    //for intersection1, look at the all the street segments connected to it
    //check the to/from intersections of those street segments and compare with intersection_id2
    for (unsigned i = 0; i < global->intersection_street_segments[intersection_id1].size(); i++) {
        //for each street segment, get info - check the intersection from/to
        StreetSegmentIndex street = global->intersection_street_segments[intersection_id1][i];

        StreetSegmentInfo streetInfo = getStreetSegmentInfo(street);

        //check that it matches intersection_id2
        if (streetInfo.from == intersection_id2 || streetInfo.to == intersection_id2) {
            return true;
        }
    }
    return false;
}

//Returns street id(s) for the given street name
//If no street with this name exists, returns a 0-length vector

std::vector<unsigned> find_street_ids_from_name(std::string street_name) {

    //create a vector to store all instances of the street name
    std::vector<unsigned> foundStreetIDs;

    //go through all the streets to see if they match the queried name
    //find range of iterators pointing to elements of matching values
    auto range = global->nameLookup.equal_range(street_name);

    //if found, add the street ids of the given name to the found vector 
    for (std::unordered_map<std::string, unsigned>::iterator iter = range.first; iter != range.second; iter++) {
        foundStreetIDs.push_back(iter->second);
    }

    return foundStreetIDs;
}



//Returns the distance between two coordinates in meters

double find_distance_between_two_points(LatLon point1, LatLon point2) {

    //convert the latitude and longitude to coordinates
    double x1, x2, y1, y2;
    x1 = point1.lon() * DEG_TO_RAD * cos((point1.lat() * DEG_TO_RAD + point2.lat() * DEG_TO_RAD) / 2);
    y1 = point1.lat() * DEG_TO_RAD;
    x2 = point2.lon() * DEG_TO_RAD * cos((point1.lat() * DEG_TO_RAD + point2.lat() * DEG_TO_RAD) / 2);
    y2 = point2.lat() * DEG_TO_RAD;
    //calculate the distance between the coordinates
    double distance = EARTH_RADIUS_IN_METERS * (sqrt(pow((y2 - y1), 2) + pow((x2 - x1), 2)));

    return distance;
}


//Returns the length of the given street segment in meters

double find_street_segment_length(unsigned street_segment_id) {
    return global-> find_street_segment_length(street_segment_id);
}


//Returns the length of the specified street in meters

double find_street_length(unsigned street_id) {
    return global->find_street_length(street_id);
}


//Returns the travel time to drive a street segment in seconds (time = distance/speed_limit)

double find_street_segment_travel_time(unsigned street_segment_id) {

    return global->travelTimes[street_segment_id];
}



//Finds intersections adjacent to given intersection

std::vector<unsigned> find_adjacent_intersections(unsigned intersection_id) {
    //create vector to store return values
    std::vector<unsigned> adjacent_intersections;
    for (unsigned i = 0; i < global->intersection_street_segments[intersection_id].size(); i++) {
        StreetSegmentInfo streetInfo = getStreetSegmentInfo(global->intersection_street_segments[intersection_id][i]);
        //compare values of to and from interesections to given intersection ids
        //store intersection id when matchh found
        if (streetInfo.from == intersection_id) {
            if (streetInfo.to != intersection_id) {
                //check for duplicates
                if (std::find(adjacent_intersections.begin(), adjacent_intersections.end(), streetInfo.to) == adjacent_intersections.end()) {
                    adjacent_intersections.push_back(streetInfo.to);
                }
            }
        }//consider when the street is one way (from -> to)
        else if (!streetInfo.oneWay && streetInfo.to == intersection_id) {
            if (std::find(adjacent_intersections.begin(), adjacent_intersections.end(), streetInfo.from) == adjacent_intersections.end()) {

                adjacent_intersections.push_back(streetInfo.from);
            }
        }
    }
    return adjacent_intersections;
}

//Returns all street segments for the given street

std::vector<unsigned> find_street_street_segments(unsigned street_id) {

    std::vector<unsigned> streetSegIndex;

    streetSegIndex = global->street_street_segments[street_id];

    return streetSegIndex;
}

//Returns all intersections along the a given street

std::vector<unsigned> find_all_street_intersections(unsigned street_id) {
    //initialize vector of street segments
    //initialize vector of intersection indices
    std::vector<unsigned> intersectionIndices;

    intersectionIndices = global->street_intersections[street_id];

    return intersectionIndices;
}

//Find the intersection points  between two streets

std::vector<unsigned> find_intersection_ids_from_street_names(std::string street_name1, std::string street_name2) {
    //find the streetID for street name_1
    std::vector<unsigned> streetIntersections1;
    std::vector<unsigned> streetIntersections2;

    //create temporary vectors to store all intersections
    std::vector<unsigned> tempIntersection1;
    std::vector<unsigned> tempIntersection2;

    //vector of intersection IDs to be returned
    std::vector<unsigned> intersectionIDs;

    //    //All the streetIDs associated with name 1 and name 2
    std::vector<unsigned> streetID_1 = find_street_ids_from_name(street_name1);
    std::vector<unsigned> streetID_2 = find_street_ids_from_name(street_name2);

    //create one large vector with all the street intersection IDs from street1
    for (unsigned i = 0; i < streetID_1.size(); i++) {
        tempIntersection1 = find_all_street_intersections(streetID_1[i]);
        streetIntersections1.insert(streetIntersections1.end(), tempIntersection1.begin(), tempIntersection1.end());
    }

    //create one large vector will all the street intersection IDs from street 2
    for (unsigned i = 0; i < streetID_2.size(); i++) {
        tempIntersection2 = find_all_street_intersections(streetID_2[i]);
        streetIntersections2.insert(streetIntersections2.end(), tempIntersection2.begin(), tempIntersection2.end());
    }

    //compare those two vectors using find
    for (unsigned i = 0; i < streetIntersections1.size(); i++) {
        if (std::find(streetIntersections2.begin(), streetIntersections2.end(), streetIntersections1[i]) != streetIntersections2.end()) {
            //check that intersection_id is unique
            if (std::find(intersectionIDs.begin(), intersectionIDs.end(), streetIntersections1[i]) == intersectionIDs.end()) {
                //push those results into a vector
                intersectionIDs.push_back(streetIntersections1[i]);
            }
        }
    }
    if(intersectionIDs.empty()){
        std::cout<<"They do not intersect, try again " <<std::endl;
    }else{
        std::cout<<"They intersect at: " <<std::endl;
    }
    return intersectionIDs;
}


