/**********************************************************************/
/* The global class contains all the data structures used in the map.
 * Constructor contains all load map functions.
 * The class also contains the implementation of the spatial functions.
 */
/**********************************************************************/
#include <string>
#include <vector>
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include "StreetsDatabaseAPI.h"
#include "OSMDatabaseAPI.h"
#include "graphics.h"
#include "math.h"
#include "m1.h"
#include "m2.h"
#include "Node.h"
#include "graphics_types.h"
#include <algorithm>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/foreach.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <thread>

#define EARTH_RADIUS_M 6372797.560856
#define D_TO_R 0.017453292519943295769236907684886
namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

#ifndef GLOBAL_H
#define GLOBAL_H


double find_spherical_dis(LatLon point1, LatLon point2);
typedef bg::model::point<double, 2, bg::cs::cartesian> point;
typedef bg::model::box<point> box;
typedef std::pair<box, unsigned> value;

struct StreetSeg {
    StreetIndex streetID;
    std::string streetName;
    bool isOneWay;
    std::string streetType;
    std::vector<t_point> points;

};

struct FeatureInfo {
    std::string name;
    FeatureType type;
    unsigned numPoint;
    t_point center;
    t_bound_box bounds;
    bool if_closed; //true- closed; false - open 
    t_point *point_positions; // declare a pointer of type int.

};

struct intersection_data {
    LatLon position;
    std::string name;

    
};

struct POI_data {
    t_point position;
    std::string name;
    std::string type;
    unsigned closest_intersection;
};

class Global {
public:
    //street - intersection - street seg
    std::vector<std::vector<unsigned>> intersection_street_segments;

    //street - (street seg, street seg info)
    std::vector<std::vector<unsigned>> street_street_segments;

    //street - intersection
    std::vector<std::vector<unsigned>> street_intersections;

    bgi::rtree< value, bgi::rstar<15> > rtree_POI;
    bgi::rtree< value, bgi::rstar<15> > rtree_INTERS;

    //create an unordered multimap of street names and street ids  
    std::unordered_multimap<std::string, unsigned> nameLookup;

    //Create a vector to store travel times for every street segement
    std::vector<double> travelTimes;

    //Create a vector of street segment structures
    std::vector<StreetSeg> street_segment_info;

    //Create a vector of features
    std::vector<FeatureInfo> features;

    //Create a vector of intersection data
    std::vector<intersection_data> intersections;

    //Create a vector of points of interest
    std::vector<POI_data> POIs;

    //create a map for OSMIDs and entities
    std::unordered_map<OSMID, const OSMEntity*> OSM_way_data;

    //Create a vector of nodes for A* pathfinding
    std::vector<Node*> intersection_node;
    
    //hash map for name - Node matching
    std::unordered_multimap<std::string, Node*> POI_name_inters;
   

    //Latitude and Longitude bounds of the map
    double avg_lat;
    double max_lat;
    double min_lat;
    double max_lon;
    double min_lon;
    double max_world_width; 
    float max_speed;

    //global variables for interactive search bar
    bool directions_active = false;
    std::vector<std::string> street_names;
    std::string user_query;
    bool typing = false;
    //Disables pin drop on mouse click
    bool do_not_draw_pin = false;
    //variable for text height in directions bar
    double y = 10; 

    //Functions to convert between Cartesian and LatLon
    double lon_to_x(double lon, double average_lat);
    double lat_to_y(double lat);
    float x_to_lon(float x, double average_lat);
    float y_to_lat(float y);
    t_point LatLon_to_tpoint(LatLon p);
    //******************//

    //Functions to load points of interest and intersections, as well as access that data
    void load_POI();
    void load_intersection_data();
    void load_travel_time_data(); 
    unsigned find_closest_INTERS(LatLon myPoint);
    std::vector<unsigned> find_closest_POI_withNUM(LatLon myPoint,int number);
    unsigned find_closest_POI(LatLon myPoint);
    //*******************************//

    double find_street_segment_length(unsigned street_segment_id);
    double find_street_length(unsigned street_id);

    //Functions to convert between screen and world coordinates for the graphics library
    t_point convert_world_to_screen(t_point world);
    t_point convert_screen_to_world(t_point screen);
    //****************************//

    //load intersection_node for pathfinding
    void load_node();


    Global();
    ~Global();
private:
    //Calculates the ratio between the visible world and visible screen
    t_point calculate_xyratio();


};


void load_m1_intersection_street_segments(Global* g);
void load_m1_street_intersections(Global* g);
void load_sseg_data(Global* g);
void load_feature(Global* g);
void insert_rtree(Global* g);


#endif /* GLOBAL_H */

