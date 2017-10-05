/**********************************************************************/
/*Implementation of the global class that contains all the data structures for the map*/
/**********************************************************************/

#include "Global.h"
#include <chrono>
#include <fstream>


//populate rtree for spatial tests
void insert_rtree(Global* g) {

    //populate rtree of point of interest
    for (unsigned i = 0; i < getNumberOfPointsOfInterest(); i++) {
        LatLon POIPosition = getPointOfInterestPosition(i);
        //make bounding box for each point
        box b(point(POIPosition.lat() - 0.000005, POIPosition.lon() - 0.000005), point(POIPosition.lat() + 0.000005, POIPosition.lon() + 0.000005));
        //insert in the tree
        g->rtree_POI.insert(std::make_pair(b, i));
    }

    //populate rtree of intersection
    for (unsigned i = 0; i < getNumberOfIntersections(); i++) {
        LatLon INTPosition = getIntersectionPosition(i);

        //make bounding box for each point 
        box c(point(INTPosition.lat() - 0.00000000005, INTPosition.lon() - 0.00000000005), point(INTPosition.lat() + 0.00000000005, INTPosition.lon() + 0.00000000005));
        //insert in the tree
        g->rtree_INTERS.insert(std::make_pair(c, i));
    }
    return;
}


unsigned Global::find_closest_POI(LatLon myPoint) {
    std::vector<value> result_n;
    double min_distance, current_distance;
    unsigned closest_POI_id;

    //query 10 nearest intersection and insert them into result_n
    rtree_POI.query(bgi::nearest(point(myPoint.lat(), myPoint.lon()), 100), std::back_inserter(result_n));

    //check empty to prevent seg fault
    if (result_n.empty()) {
        return 0;
    } else {

        //linear search through the result from the "find nearest 10 points query" to find the minimum distance and return the poi id
        for (unsigned i = 0; i < result_n.size(); i++) {
            LatLon POIPositionl = getPointOfInterestPosition(result_n[i].second);
            current_distance = find_spherical_dis(myPoint, POIPositionl);
            if ((i == 0) || (min_distance > current_distance)) {
                min_distance = current_distance;
                closest_POI_id = result_n[i].second;
            }
        }
        return closest_POI_id;
    }

}

std::vector<unsigned> Global::find_closest_POI_withNUM(LatLon myPoint,int number){
    std::vector<value> result_n;
    std::vector<unsigned> result;
    //query 10 nearest intersection and insert them into result_n
    rtree_POI.query(bgi::nearest(point(myPoint.lat(), myPoint.lon()), number), std::back_inserter(result_n));
    for(unsigned k = 0;k<result_n.size();k++){
        result.push_back(result_n[k].second);
    }
    
    return result;
}

unsigned Global::find_closest_INTERS(LatLon myPoint) {
    std::vector<value> result_n;
    double min_distance, current_distance;
    unsigned closest_intersection_id = 0;

    //query 10 nearest intersection and insert them into result_n
    rtree_INTERS.query(bgi::nearest(point(myPoint.lat(), myPoint.lon()), sqrt(getNumberOfIntersections())), std::back_inserter(result_n));

    //check empty to prevent seg fault
    if (result_n.empty()) {
        return 0;
    } else {

        //linear search through the result from the "find nearest 10 points query" to find the minimum distance and return the intersection id
        for (unsigned i = 0; i < result_n.size(); i++) {
            LatLon POIPositionl = getIntersectionPosition(result_n[i].second);
            current_distance = find_spherical_dis(myPoint, POIPositionl);
            if ((i == 0) || (min_distance > current_distance)) {
                min_distance = current_distance;
                closest_intersection_id = result_n[i].second;
            }
        }
        return closest_intersection_id;
    }
}

Global::Global() {
     static std::ofstream perf("load_map.csv");
    auto const start_time = std::chrono::high_resolution_clock::now();
    load_intersection_data();
   
    
    //split the loading functions into multiple threads
    std::thread t1(load_m1_intersection_street_segments,this);
    std::thread t2(insert_rtree,this);
    std::thread t3(load_sseg_data,this);
    std::thread t4(load_m1_street_intersections,this);
    std::thread t5(load_feature,this);    
    
    
    //combine the threads
    t1.join();
    t2.join();
    t3.join();
    t4.join();
    t5.join();
    
//    load_m1_intersection_street_segments(this);
//    insert_rtree(this);
//    load_sseg_data(this);
//    load_m1_street_intersections(this);
//    load_feature(this);       
    //load_feature();
    
    load_node();
    load_POI(); 
    
    load_travel_time_data();
    
    auto current_time = std::chrono::high_resolution_clock::now();
    auto wall_clock = std::chrono::duration_cast<std::chrono::duration<double>> (current_time - start_time);
    
    perf << "load_map, toronto, multithreading, "<<wall_clock.count()<<std::endl;
    
}

//load street seg datat structure from m1
void load_m1_intersection_street_segments(Global* g){

    unsigned numIntersection;
    unsigned numStreet;
    unsigned numStreetSeg_intersection;
   

    //populate intersection_street_segments
    numIntersection = getNumberOfIntersections();
    g->intersection_street_segments.resize(numIntersection);
    for (unsigned i = 0; i < numIntersection; i++) {
        numStreetSeg_intersection = getIntersectionStreetSegmentCount(/*IntersectionIndex intersectionIdx*/i);
        for (unsigned j = 0; j < numStreetSeg_intersection; j++) {
            //outside:  intersection_street_segments[i] : i = intersection id
            //inside each element is StreetSegmentIndex at this intersection
            g->intersection_street_segments[i].push_back(getIntersectionStreetSegment(i, j));
        }
    }

    numStreet = getNumberOfStreets();

    //populate the unordered multimap of names and street ids upon loading the map
    for (unsigned i = 0; i < numStreet; i++) {
        g->nameLookup.insert(make_pair(getStreetName(i), i));
    }    

}

//load street intersection data structure from m1
void load_m1_street_intersections(Global* g){
    unsigned numStreet = getNumberOfStreets();
    unsigned numStreetSeg = getNumberOfStreetSegments();
    
    g->street_street_segments.resize(numStreet);
    for (unsigned i = 0; i < numStreetSeg; i++) {
        StreetSegmentInfo ss_info = getStreetSegmentInfo(i);
        g->street_street_segments[ss_info.streetID].push_back(i);
    }
    //populate the street by intersections 2D vector
    //the outside vector are streetSegmentIds
    //the inside vectors are interseectionsIds
    g->street_intersections.resize(numStreet);

    for (unsigned i = 0; i < numStreet; i++) {
        for (unsigned j = 0; j < g->street_street_segments[i].size(); j++) {
            IntersectionIndex from = getStreetSegmentInfo(g->street_street_segments[i][j]).from;
            IntersectionIndex to = getStreetSegmentInfo(g->street_street_segments[i][j]).to;
            if (j == 0) {
                if (std::find(g->street_intersections[i].begin(), g->street_intersections[i].end(), from) == g->street_intersections[i].end())
                    g->street_intersections[i].push_back(from);
                if (std::find(g->street_intersections[i].begin(), g->street_intersections[i].end(), to) == g->street_intersections[i].end())
                    g->street_intersections[i].push_back(to);
            } else {
                if (std::find(g->street_intersections[i].begin(), g->street_intersections[i].end(), from) == g->street_intersections[i].end()) {
                    g->street_intersections[i].push_back(from);
                }
                if (std::find(g->street_intersections[i].begin(), g->street_intersections[i].end(), to) == g->street_intersections[i].end()) {
                    g->street_intersections[i].push_back(to);
                }
            }

        }
    }
}

//load street segment info vector
void load_sseg_data(Global* g){
    unsigned numStreetSeg;
    
    numStreetSeg = getNumberOfStreetSegments();
    
    //populate OSM_way_data to get tags
    for(unsigned i = 0; i < getNumberOfWays(); i++){
        const OSMWay* way = getWayByIndex(i);
        OSMID id = way->id();
        g->OSM_way_data.emplace(id,way);
    }
    
    
    //populate street_segment_info
    g->street_segment_info.resize(numStreetSeg);

    for(unsigned id = 0 ; id < numStreetSeg; id++){
        StreetSegmentInfo info = getStreetSegmentInfo(id);
        g->street_segment_info[id].streetID =  info.streetID;
        g->street_segment_info[id].streetName = getStreetName(info.streetID);
        g->street_segment_info[id].isOneWay = info.oneWay;
       
        //use OSM data to determine type of street
        //search data structure to get the OSM Entity
        //save the tags associated with the type of road
        OSMID wayOSM = info.wayOSMID;
        std::unordered_map<OSMID, const OSMEntity*>::iterator iter =  g->OSM_way_data.find(wayOSM);
        const OSMEntity* streetSegOSM = iter->second; 
        for(unsigned i=0; i < getTagCount(streetSegOSM); i++){
            std::pair<std::string,std::string> tagPair = getTagPair(streetSegOSM, i);
            if(tagPair.first == "highway"){
                g->street_segment_info[id].streetType = tagPair.second;
            }
        }
        
        //add the LatLon info for all the points the street segment hits
        t_point pointToInsert = g->LatLon_to_tpoint(getIntersectionPosition(info.from));
        g->street_segment_info[id].points.push_back(pointToInsert);
        for(unsigned j = 0; j < info.curvePointCount; j++){
            pointToInsert = g->LatLon_to_tpoint(getStreetSegmentCurvePoint(id,j));
            g->street_segment_info[id].points.push_back(pointToInsert);
        }
        pointToInsert = g->LatLon_to_tpoint(getIntersectionPosition(info.to));
        g->street_segment_info[id].points.push_back(pointToInsert);
        
    }

}


//clear the data structures

Global::~Global() {
    intersection_street_segments.clear();
    street_street_segments.clear();
    street_intersections.clear();
    nameLookup.clear();
    for(unsigned id = 0; id < features.size(); id++){
        delete [] features[id].point_positions;
    }
    features.clear();
    street_segment_info.clear();
    intersections.clear();
    POIs.clear();
    
    for(unsigned i = 0; i < intersection_node.size();i++){
        delete intersection_node[i];
    }
    intersection_node.clear();
    OSM_way_data.clear();
//    POI_name_inters_hash.clear();
    
    
}

//global function for finding spherical distance --used in the sorting function

double find_spherical_dis(LatLon point1, LatLon point2) {

    //convert the latitude and longitude to coordinates
    double x1, x2, y1, y2;
    x1 = point1.lon() * D_TO_R * cos((point1.lat() * D_TO_R + point2.lat() * D_TO_R) / 2);
    y1 = point1.lat() * D_TO_R;
    x2 = point2.lon() * D_TO_R * cos((point1.lat() * D_TO_R + point2.lat() * D_TO_R) / 2);
    y2 = point2.lat() * D_TO_R;
    //calculate the distance between the coordinates
    double distance = EARTH_RADIUS_M * (sqrt(pow((y2 - y1), 2) + pow((x2 - x1), 2)));

    return distance;
}

//populate the point of interest data structure
void Global::load_POI(){
    POIs.resize(getNumberOfPointsOfInterest());
    
   // POI_node.resize(getNumberOfPointsOfInterest());

    for(unsigned id = 0; id < getNumberOfPointsOfInterest();id++){
        POIs[id].position = LatLon_to_tpoint(getPointOfInterestPosition(id));
        POIs[id].name = getPointOfInterestName(id);
        POIs[id].type = getPointOfInterestType(id);
        POIs[id].closest_intersection = find_closest_INTERS(getPointOfInterestPosition(id));
        
        POI_name_inters.insert(std::make_pair(POIs[id].name, intersection_node[POIs[id].closest_intersection]));

    }
}

//populate features vector, calculate bounds and center for each feature for names
void load_feature(Global* g){
    g->features.resize(getNumberOfFeatures());

    for(unsigned id = 0; id < g->features.size(); id++){
        g->features[id].name = getFeatureName(id);
        g->features[id].type = getFeatureType(id);
        g->features[id].numPoint = getFeaturePointCount(id);
        LatLon first_point = getFeaturePoint(id, 0);
        LatLon last_point = getFeaturePoint(id, g->features[id].numPoint-1);
        g->features[id].if_closed = ((first_point.lat() == last_point.lat()) && (first_point.lon() == last_point.lon()))? true : false;
        g->features[id].point_positions = new t_point[g->features[id].numPoint]; // dynamically allocate memory using new
        LatLon p_ll;
        
        t_point max_bound = t_point(g->lon_to_x(getFeaturePoint(id, 0).lon(),g->avg_lat),g->lat_to_y(getFeaturePoint(id, 0).lat()));
        t_point min_bound = t_point(max_bound.x,max_bound.y);
        
        t_point center= t_point(0,0);
        for(unsigned i = 0; i < g->features[id].numPoint; i++){
            
            p_ll = getFeaturePoint(id, i);
            g->features[id].point_positions[i].x = g->lon_to_x(p_ll.lon(), g->avg_lat);
            g->features[id].point_positions[i].y = g->lat_to_y(p_ll.lat());
            
            if(g->features[id].if_closed){
                center.x += g->features[id].point_positions[i].x;
                center.y += g->features[id].point_positions[i].y;                
            }
            
            max_bound.x = std::max(g->features[id].point_positions[i].x,max_bound.x );
            max_bound.y = std::max(g->features[id].point_positions[i].y,max_bound.y);
            min_bound.x = std::min(g->features[id].point_positions[i].x,min_bound.x);
            min_bound.y = std::min(g->features[id].point_positions[i].y,min_bound.y);
        } 
        g->features[id].center.x = center.x/g->features[id].numPoint;
        g->features[id].center.y = center.y/g->features[id].numPoint;
        
        //t_bound_box(const t_point& bottomleft, const t_point& topright);
       g->features[id].bounds = t_bound_box(min_bound,max_bound);
    }

}
//populate vector of street segment ids (index) and travel times (elements)
void Global::load_travel_time_data(){
   unsigned numStreetSeg = getNumberOfStreetSegments();
   travelTimes.resize(numStreetSeg);
    max_speed = 0.0;
    for (unsigned i = 0; i < numStreetSeg; i++) {
        // get street segment info
        StreetSegmentInfo info;
        info = getStreetSegmentInfo(i);

        //get the street segment speed limit
        double speed = info.speedLimit;

        speed = speed * 1000 / 3600;
        if(i==0 || speed > max_speed)
            max_speed = speed;
        //get the street segment length
        double length = find_street_segment_length(i);

        //calculate time - length is in m
        double travelTime = length / speed;
        // store this result in the map
        travelTimes[i] = travelTime;

    } 
}

void Global::load_intersection_data(){
    unsigned numINtersection = getNumberOfIntersections();        
    intersections.resize(numINtersection);
    
    max_lat = getIntersectionPosition(0).lat();
    min_lat = max_lat;
    
    max_lon = getIntersectionPosition(0).lon();
    min_lon = max_lon;
    for(unsigned id = 0; id < numINtersection; ++id){
        LatLon lat_lon_position = getIntersectionPosition(id);
        intersections[id].position = lat_lon_position;
        intersections[id].name = getIntersectionName(id);
        
        max_lat = std::max(lat_lon_position.lat(),max_lat);
        min_lat = std::min(lat_lon_position.lat(),min_lat);
        max_lon = std::max(lat_lon_position.lon(),max_lon);
        min_lon = std::min(lat_lon_position.lon(),min_lon);
    }
    avg_lat = 0.5 * (max_lat + min_lat);
    max_world_width = (lon_to_x(max_lon, avg_lat) - lon_to_x(min_lon, avg_lat));
}

/**
 * If the part of the surface of the earth which you want to draw is relatively small, 
 * then you can use a very simple approximation. You can simply use the horizontal axis x
 *  to denote longitude λ, the vertical axis y to denote latitude φ. The ratio between these should not be 1:1, though. 
 * Instead you should use cos(φ0) as the aspect ratio, where φ0 denotes a latitude close to the center of your map. 
 * Furthermore, to convert from angles (measured in radians) to lengths, you multiply by the radius of the earth (which in this model is assumed to be a sphere).
 * x = r λ cos(φ0)
 * y = r φ 
 */

//latlon point to t_point
double Global::lon_to_x(double lon, double average_lat){
    return lon * DEG_TO_RAD * cos(average_lat*DEG_TO_RAD)*10000;
}


double Global::lat_to_y(double lat){
    return DEG_TO_RAD* lat*10000;

}

t_point Global::LatLon_to_tpoint(LatLon p){
    t_point new_point;
    new_point.x = lon_to_x(p.lon(), avg_lat);
    new_point.y = lat_to_y(p.lat());
    return new_point;
}

//t_point to latlon 
float Global::x_to_lon(float x, double average_lat){
    return x/(DEG_TO_RAD*cos(average_lat*DEG_TO_RAD)*10000);
}
float Global::y_to_lat(float num){
    return num/(DEG_TO_RAD*10000);
    
}

//Returns the length of the given street segment in meters

double Global::find_street_segment_length(unsigned street_segment_id) {
    
    
    double len = 0;
    //get street segment info
    StreetSegmentInfo info;
    info = getStreetSegmentInfo(street_segment_id);

    //get end points
    IntersectionIndex point1 = info.from;
    IntersectionIndex point2 = info.to;

    //convert end points to latlon
    LatLon end1 = getIntersectionPosition(point1);
    LatLon end2 = getIntersectionPosition(point2);

    //check for curve points
    unsigned curves = info.curvePointCount;

    //if curve points exist, find and save their coordinates
    if (curves > 0) {

        LatLon lastPoint = end1;

        for (unsigned i = 0; i < curves; i++) {
            //get location of the ith curve point

            LatLon curvePoint = getStreetSegmentCurvePoint(street_segment_id, i);

            //get length from start of segment to current curve point
            len += find_distance_between_two_points(lastPoint, curvePoint);
            lastPoint = curvePoint;
        }
        len += find_distance_between_two_points(lastPoint, end2);

    } else {

        len = find_distance_between_two_points(end1, end2);
    }

    return len;
}


//Returns the length of the specified street in meters

double Global::find_street_length(unsigned street_id) {
    //sum up the lengths of the street's segments 
    double totalLen = 0;

    std::vector<unsigned> streetSegments = find_street_street_segments(street_id);

    // find the length of each street segment
    for (unsigned i = 0; i < streetSegments.size(); i++) {
        //sum up these segments 
        totalLen += find_street_segment_length(streetSegments[i]);
    }
    streetSegments.clear();

    return totalLen;
}

/*Calculates the ratio between the visible world and the visible screen*/
t_point Global::calculate_xyratio(){
    t_bound_box visible_world_bounds = get_visible_world();
    t_bound_box visible_screen = get_visible_screen();
    t_point visible_world_span((visible_world_bounds.get_width()), (visible_world_bounds.get_height()));
    
    t_point visible_screen_span(visible_screen.get_width(), visible_screen.get_height());
    t_point ratio (visible_world_span.x/visible_screen_span.x, visible_world_span.y/visible_screen_span.y);
    return ratio;  
}

/*Converts coordinates in the visible world (cartesian) to screen (pixels)*/
t_point Global::convert_world_to_screen(t_point world){
    t_point xy_ratio = calculate_xyratio();
    t_point screen((world.x)/(xy_ratio.x), (world.y)/(xy_ratio.y));
    return screen;
    
}

/*Converts coordinates in pixels to world(cartesian)*/
t_point Global::convert_screen_to_world(t_point screen){
    t_point xy_ratio = calculate_xyratio();
    t_point world(screen.x * xy_ratio.x, screen.y * xy_ratio.y);
    return world;  
}

void Global::load_node(){
    unsigned numIntersection = getNumberOfIntersections();
    intersection_node.resize(numIntersection);
    
    for(unsigned i = 0; i<numIntersection; i++){
        intersection_node[i] = new Node(i);
    }
}

