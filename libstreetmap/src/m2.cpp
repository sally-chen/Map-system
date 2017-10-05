/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include "graphics.h"
#include "Global.h"
#include "math.h"
#include "StreetsDatabaseAPI.h"
#include "OSMDatabaseAPI.h"
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include "m3.h"

//#define turn_penalty 5


extern Global *global;
//global variables for deciding whether or not to draw features
bool draw_instructions_bool = false;
bool draw_pin_bool = false;
bool draw_path_bool = false;
//Enables the start/end intersection for a path
bool draw_path_end_bool = false;
//Disables path calculation between two intersections
bool path_already_calculated = false;
bool search_interaction = false;
//zoom level flag
bool half_world = false;
bool fourth_of_world = false;

//Turn this mode on for a intersection -> POI path search
//Keep off (false) otherwise
bool POI_mode = false;
std::vector<t_point> draw_pin_place;
//global variables for drawing path
std::vector<std::pair<unsigned, t_point>> draw_path_intersections;
std::vector<unsigned> path;
//global variables for user search query 

std::stringstream input_to_parse;

int turn_penalty;

void draw_screen();
void draw_streets();
void draw_features();
void draw_POI();
void draw_intersections();
void draw_search_bar();
void draw_scale();
bool print_intersection_data(unsigned intersectionID);
void print_highlight_nearest_intersection(float x, float y);
void find_intersection_button(void (*) (void));
void act_on_button_press(float x, float y, t_event_buttonPressed event);
void draw_pin();
void print_street_open_feature_name(t_point start, t_point end, std::string name, t_color color, bool isOneWay);
void draw_intersection_info(unsigned intersectionID);
void find_intersection_from_search();
void find_POI_by_name(std::string name);
void parse_input();
std::string remove_white_space(std::string name);

void search_bar_action(float x, float y);
void help_button_action(float x, float y);
void draw_instructions();
void act_on_key_press(char c, int keysym);
void draw_mag_glass(bool searching);
void path_find_button(void (*) (void));
void draw_path();
void draw_help();
void draw_path_ends();
void calculate_path();
void recalculate_endpoint();
void display_path_to_POI();
void display_path_between_intersections();
void print_directions();
std::string get_turn_direction(unsigned streetSeg1, unsigned streetSeg2);
void draw_directions_sidebar();
void display_text(std::string output);
void calculate_zoom_level();

void draw_map() {
    t_color grey (220, 220, 220);
    init_graphics("map", grey);
    set_visible_world(global->lon_to_x(global->min_lon, global->avg_lat), global->lat_to_y(global->min_lat), global->lon_to_x(global->max_lon, global->avg_lat), global->lat_to_y(global->max_lat));
    create_button("Window", "Find", find_intersection_button);
    create_button("Window", "Path Finder", path_find_button);
    set_keypress_input(true);
    event_loop(act_on_button_press, NULL, act_on_key_press, draw_screen);

}

void draw_screen() {
    static std::ofstream perf("draw_screen.csv");
    auto const start_time = std::chrono::high_resolution_clock::now();
    set_drawing_buffer(OFF_SCREEN);
    clearscreen();

    calculate_zoom_level();
    //    draw_intersections();
    draw_features();
    draw_streets();

    draw_POI();
    //Please don't change the order of pin/path
    draw_pin();
    draw_path();
    draw_help();
    draw_scale();

    //if you want it to be double buffered, stick above this line
    copy_off_screen_buffer_to_screen();

    draw_search_bar();
    auto current_time = std::chrono::high_resolution_clock::now();
    auto wall_clock = std::chrono::duration_cast<std::chrono::duration<double>> (current_time - start_time);
    std::cout<<"draw screen"<<wall_clock.count()<<std::endl;
    perf << "draw_screen, london, "<<wall_clock.count()<<std::endl;
    
}

void calculate_zoom_level() {
    float world_width = get_visible_world().get_width();
    if (world_width <= global->max_world_width / 2) {
        half_world = true;
        if (world_width <= global->max_world_width / 4) {
            fourth_of_world = true;
        } else
            fourth_of_world = false;
    } else
        half_world = false;
}


//draw the pin if location clicked

void draw_features() {

    t_color color, text_color;
    std::vector<FeatureInfo> rivers;
    for (unsigned id = 0; id < global->features.size(); id++) {
        if (global->features[id].type == Park || global->features[id].type == Island
                || global->features[id].type == Golfcourse || global->features[id].type == Greenspace || global->features[id].type == Shoreline) {
            color = t_color(203, 230, 163);
            text_color = t_color(113, 196, 108); //lightgreen
        } else if (global->features[id].type == Lake || global->features[id].type == River || global->features[id].type == Stream) {
            color = t_color(163, 204, 255);
            text_color = t_color(75, 139, 191); //light blue
        } else if (global->features[id].type == Building) {
            color = t_color(234, 234, 234);
            text_color = t_color(89, 189, 189); //light gray
        } else if (global->features[id].type == Beach) {
            color = t_color(240, 237, 229);
            text_color = t_color(230, 171, 124); //light brown
        } else
            color = t_color(255, 255, 255); //white

        if (global->features[id].if_closed) {
            setcolor(color);
            fillpoly(global->features[id].point_positions, global->features[id].numPoint - 1);
            setcolor(text_color);
            std::string seg_name = (global->features[id].name == "<unknown>" || global->features[id].name == "<noname>" || global->features[id].name == "<unspecified>") ? " " : global->features[id].name;
            setfontsize(10);
            drawtext(global->features[id].center,
                    seg_name, global->features[id].bounds);
        } else {
            rivers.push_back(global->features[id]);
        }

    }


    for (unsigned id = 0; id < rivers.size(); id++) {
        color = t_color(163, 204, 255);

        setlinewidth(2);
        for (unsigned i = 1; i < rivers[id].numPoint; i++) {
            setcolor(color);
            drawline(rivers[id].point_positions[i - 1], rivers[id].point_positions[i]);
            flushinput();
            print_street_open_feature_name(rivers[id].point_positions[i - 1], rivers[id].point_positions[i], rivers[id].name, t_color(75, 139, 191), false);

        }

    }



}

void draw_streets() {
    unsigned numStreetSeg = getNumberOfStreetSegments();
    for (unsigned id = 0; id < numStreetSeg; id++) {

        float linewidth = 0;
        t_color color;
        if (global->street_segment_info[id].streetType == "motorway" ||
                global->street_segment_info[id].streetType == "motorway_link") {
            linewidth = 3;
            color = t_color(249, 197, 92); //orange
        } else if (global->street_segment_info[id].streetType == "trunk" ||
                global->street_segment_info[id].streetType == "trunk_link") {
            linewidth = 4;
            color = t_color(255, 255, 255); //white
        } else if (global->street_segment_info[id].streetType == "primary" ||
                global->street_segment_info[id].streetType == "primary_link") {
            linewidth = 3;
            color = t_color(255, 255, 102);//yellow
        } else if (global->street_segment_info[id].streetType == "secondary" ||
                global->street_segment_info[id].streetType == "secondary_link") {
            linewidth = 3;
            color = t_color(255, 255, 179);//light yellow
        } else {
            //everything else
            if (half_world == false) {
                continue;
            }
            linewidth = 2;
            color = t_color(255, 255, 255);//white
        }

        for (unsigned curves = 1; curves < global->street_segment_info[id].points.size(); curves++) {


            setlinewidth(linewidth);
            setcolor(color);
            setlinestyle(SOLID, BUTT);

            drawline(global->street_segment_info[id].points[curves - 1], global->street_segment_info[id].points[curves]);
            print_street_open_feature_name(global->street_segment_info[id].points[curves - 1], global->street_segment_info[id].points[curves], global->street_segment_info[id].streetName, t_color(0, 0, 0), global->street_segment_info[id].isOneWay);

        }

    }


}

void draw_scale() {
    set_coordinate_system(GL_SCREEN);
    //draw the indicator image

    //find the relative quantities    
    t_bound_box current_view = get_visible_screen();

    double height = -1 * current_view.get_height();
    double width = current_view.get_width();

    t_color current;
    current = t_color(100, 100, 100); //grey
    setcolor(current);
    fillrect(width - 236, height - 20, width - 186, height - 15);
    fillrect(width - 186, height - 15, width - 136, height - 10);
    fillrect(width - 136, height - 20, width - 86, height - 15);
    //calculate the leftmost side
    //calculate the rightmost side
    //find the difference
    double scaleLength = 1;

    t_bound_box world = get_visible_world();
    t_bound_box screen = get_visible_screen();

    double world_width = world.get_width();
    double screen_width = screen.get_width();

    //find ratio of map to screen
    double ratio = world_width / screen_width;

    //find the relative distance of the 150 px long scale
    scaleLength = 150 * ratio * 0.777; //0.777 = magic lucky number scaling factor    

    //display the difference
    setfontsize(10);
    settextrotation(0);
    drawtext(width - 236, height - 30, "0", width - 226, height - 25);
    std::string letterAmount = std::to_string(scaleLength);

    drawtext(width - 86, height - 30, letterAmount + " km", width - 31, height - 25);

    set_coordinate_system(GL_WORLD);
}

void draw_help() {
    set_coordinate_system(GL_SCREEN);
    t_bound_box visible_screen_bound = get_visible_screen();
    t_point position = visible_screen_bound.top_right();
    //png is 20 x 20 - hence need offset of (-30, +10))
    position = t_point(position.x - 30, position.y + 10);
    Surface help = load_png_from_file("help.png");
    draw_surface(help, position);
    if (draw_instructions_bool == true) {
        draw_instructions();
    }

    set_coordinate_system(GL_WORLD);
}

void draw_instructions() {
    //draw instruction
    t_point offset(-409, 35);
    t_point position(get_visible_screen().top_right() + offset);
    draw_surface(load_png_from_file("UserInstructions.png"), position);
    //draw close button
    t_point offset_top(-5, 35);
    t_point position_top_right(get_visible_screen().top_right() + offset_top);
    t_point close(position_top_right.x - 16, position_top_right.y);
    draw_surface(load_png_from_file("close.png"), close);

}

//prints intersection data to message bar at bottom of screen
//returns false if intersection doesn't exist

bool print_intersection_data(unsigned intersectionID) {
    if (intersectionID < global->intersections.size()) {
        update_message(global->intersections[intersectionID].name);
        std::cout << global->intersections[intersectionID].name << std::endl;
        return true;
    }
    return false;
}

void act_on_button_press(float x, float y, t_event_buttonPressed) {

    //The boolean is used to disable pin drop + path calculation when search bar
    //or help button clicked
    global->do_not_draw_pin = false;
    help_button_action(x, y);
    //check if the mouse was clicked on the search bar
    search_bar_action(x, y);
    direction_menu_actions(x, y);

    //Function that drop pin - set do_not_draw_pin before this
    print_highlight_nearest_intersection(x, y);


}

void draw_pin() {
    if (draw_pin_bool == true) {
        set_coordinate_system(GL_WORLD);

        for (unsigned i = 0; i < draw_pin_place.size(); i++) {
            //offset point for accurate draw
            //pin size is 20 x 30 - want an offset of +10, +30
            t_point offset(-10, -30);
            offset = global->convert_screen_to_world(offset);
            t_point draw_pin_here = draw_pin_place[i] + offset;

            Surface pin = load_png_from_file("map_pointer2.png");
            draw_surface(pin, draw_pin_here);
        }
    }
}

void find_intersection_button(void (*) (void)) {
    std::vector<std::string> street_names_local(2);
    std::cout << "Enter street names > " << std::endl;
    getline(std::cin, street_names_local[0]);
    getline(std::cin, street_names_local[1]);
    std::vector<unsigned > intersection_ids = find_intersection_ids_from_street_names(street_names_local[0], street_names_local[1]);
    //empty vector of pin coordinates
    draw_pin_place.clear();
    draw_pin_bool = true;
    draw_path_bool = false;
    draw_path_end_bool = false;
    draw_path_intersections.clear();


    for (unsigned i = 0; i < intersection_ids.size(); i++) {

        std::cout << global->intersections[intersection_ids[i]].name << std::endl;
        print_intersection_data(intersection_ids[i]);

        //convert into t_point
        t_point intersectionXY = global->LatLon_to_tpoint(global->intersections[intersection_ids[i]].position);
        draw_pin_place.push_back(intersectionXY);

    }
    draw_screen();

}

void print_highlight_nearest_intersection(float x, float y) {

    if (global->do_not_draw_pin == true) {
        return;
    }
    //convert x,y to LatLon to use m1.cpp function
    LatLon pointClicked(global->y_to_lat(y), global->x_to_lon(x, global->avg_lat));

    unsigned intersectionID = find_closest_intersection(pointClicked);

    //call print intersection data function with returned intersection index
    print_intersection_data(intersectionID);

    //draw a pin at that intersection
    t_point intersectionXY = global->LatLon_to_tpoint(global->intersections[intersectionID].position);

    draw_path_end_bool = true;
    POI_mode = false; 
    path_already_calculated = false; 
    std::pair<unsigned, t_point> insert_point(intersectionID, intersectionXY);
    draw_path_intersections.push_back(insert_point);

    draw_screen();
}

void draw_search_bar() {
    set_drawing_buffer(ON_SCREEN);

    set_coordinate_system(GL_SCREEN);
    settextrotation(0);

    //draw the box
    t_color color;
    color = t_color(255, 255, 255);
    setcolor(color);
    fillrect(45, 10, 300, 47);

    //draw box details (menu bar)
    color = t_color(100, 100, 100);
    setcolor(color);

    fillrect(52, 17, 82, 20);
    fillrect(52, 27, 82, 30);
    fillrect(52, 37, 82, 40);

    //standard view:
    if (!search_interaction) {
        setfontsize(20);
        //needs to be left justified 
        color = t_color(175, 175, 175);
        setcolor(color);
        drawtext(110, 17, "Search", 290, 30, 0);
        draw_mag_glass(search_interaction);
    }//if user is typing in search box
    else if (search_interaction) {
        setfontsize(15);
        color = t_color(255, 255, 255);
        setcolor(color);
        fillrect(100, 27, 200, 47);
        //needs to be left justified 
        color = t_color(175, 175, 175);
        setcolor(color);
        drawtext(100, 25, global->user_query, 200, 30, 0);
        draw_mag_glass(search_interaction);
    }
    set_coordinate_system(GL_WORLD);
}

void search_bar_action(float x, float y) {

    //when the mouse is clicked on the search bar,
    //perform the following actions

    t_point clicked(xworld_to_scrn_fl(x), yworld_to_scrn_fl(y));

    if (clicked.x > 90 && clicked.x < 300 && clicked.y > 15 && clicked.y < 40) {

        search_interaction = true;
        global->do_not_draw_pin = true;
        draw_path_intersections.clear();

    }//when the mouse is clicked outside of the search bar, restore search bar
    else {

        search_interaction = false;
        draw_search_bar();
        draw_pin_bool = true;
        global->user_query.clear();
        global-> street_names.clear();
        global->user_query.clear();
        global->street_names.clear();


    }

}

void help_button_action(float x, float y) {
    t_point clicked(xworld_to_scrn_fl(x), yworld_to_scrn_fl(y));
    t_point screen_top_right(get_visible_screen().top_right());

    //close instructions box if cross is clicked
    if (draw_instructions_bool == true) {
        t_bound_box instruction_bounds(screen_top_right.x - 21, screen_top_right.y + 35,
                screen_top_right.x - 5, screen_top_right.y + 51);
        if (instruction_bounds.intersects(clicked) == true) {
            draw_instructions_bool = false;
            global->do_not_draw_pin = true;
        }
    }
    //open instructions box if help is clicked
    t_bound_box help_bounds(screen_top_right.x - 30, screen_top_right.y + 10,
            screen_top_right.x - 6, screen_top_right.y + 34);
    if (help_bounds.intersects(clicked) == true) {
        draw_instructions_bool = true;
        global->do_not_draw_pin = true;
    }
    draw_screen();

}

void act_on_key_press(char c, int keysym) {
    // function to handle keyboard press event, the ASCII character is returned
    // along with an extended code (keysym) on X11 to represent non-ASCII
    // characters like the arrow keys.
    if (search_interaction) {
#ifdef X11 // Extended keyboard codes only supported for X11 for now
        if (keysym == XK_BackSpace) {
            global->user_query = global->user_query.substr(0, global->user_query.size() - 1);
            draw_search_bar();
            return;
        } else if (keysym == XK_Return) {
            input_to_parse << global->user_query;
            parse_input();
            global->user_query.clear();
            draw_search_bar();
        }
        if (keysym != XK_Shift_L && keysym != XK_Shift_R && keysym != XK_Left &&
                keysym != XK_Right && keysym != XK_Up && keysym != XK_Down) {
            global->user_query += c;
            draw_search_bar();
        }
#endif
    } else if (global-> directions_active) {
        global->typing = true;
        //the user is entering directions, take in 4 lines of input
        //display this input 
#ifdef X11 // Extended keyboard codes only supported for X11 for now
        if (keysym == XK_BackSpace) {
            global->user_query = global->user_query.substr(0, global->user_query.size() - 1);
            draw_directions_menu();
            return;
        } else if (keysym == XK_Return) {
            input_to_parse << global->user_query;
            parse_input();
            global->user_query.clear();
            draw_directions_menu();
        }
        if (keysym != XK_Shift_L && keysym != XK_Shift_R && keysym != XK_Left &&
                keysym != XK_Right && keysym != XK_Up && keysym != XK_Down) {
            global->user_query += c;
            draw_directions_menu();
        }
#endif


    }
}

void draw_mag_glass(bool searching) {
    set_coordinate_system(GL_SCREEN);
    settextrotation(0);
    if (searching) {
        //draw a blue magnifying glass, as if an indication light is on 
        t_color current(140, 211, 255);
        setcolor(current);
        fillarc(280, 23, 9, 0, 360);
        setlinewidth(4);
        setlinestyle(SOLID, BUTT);
        drawline(280, 23, 265, 40);
        current = t_color(255, 255, 255);
        setcolor(current);
        fillarc(280, 23, 5, 0, 360);

    } else {
        //draw a grey magnifying glass, as if an indication light is off
        t_color current(204, 204, 204);
        setcolor(current);
        fillarc(280, 23, 9, 0, 360);
        setlinewidth(4);
        setlinestyle(SOLID, BUTT);
        drawline(280, 23, 265, 40);
        current = t_color(255, 255, 255);
        setcolor(current);
        fillarc(280, 23, 5, 0, 360);
    }
    set_coordinate_system(GL_WORLD);
}

void print_street_open_feature_name(t_point start, t_point end, std::string name, t_color color, bool isOneWay) {

    double centre_x = 0.5 * (end.x + start.x);
    double centre_y = 0.5 * (end.y + start.y);
    double diff_y = end.y - start.y;
    double diff_x = end.x - start.x;

    t_point centre = t_point(centre_x, centre_y);
    double angle = atan(diff_y / diff_x) / D_TO_R;
    double seg_length = sqrt(pow(diff_y, 2) + pow(diff_x, 2));
    std::string seg_name = (name == "<unknown>" || name == "<noname>") ? " " : name;
    setfontsize(10);
    setcolor(color);
    settextrotation(angle);

    drawtext(centre,
            seg_name,
            seg_length,
            seg_length);

    std::string arrow_right = "→";
    std::string arrow_left = "←";
    setfontsize(8);
    setcolor(128, 128, 128);
    //draw arrow on one way streets
    if (isOneWay) {

        if (diff_x < 0)
            drawtext(centre,
                arrow_left,
                seg_length,
                seg_length);
        else
            drawtext(centre,
                arrow_right,
                seg_length,
                seg_length);

    }

    settextrotation(0);
}

void draw_POI() {
    set_coordinate_system(GL_WORLD);
    if (fourth_of_world == true) {
        for (unsigned id = 0; id < global->POIs.size(); id++) {
            setcolor(175, 136, 100, 255 / 4);
            fillarc(global->POIs[id].position, 0.007, 0, 360);
            std::string POI_name = global->POIs[id].name;
            setfontsize(8);
            setcolor(t_color(175, 136, 100));
            t_point offset(0, -30);
            offset = global->convert_screen_to_world(offset);
            drawtext(t_point(global->POIs[id].position.x, global->POIs[id].position.y + offset.y),
                    POI_name,
                    0.05,
                    0.05);
            settextrotation(0);
        }
    }
}

void find_POI_by_name(std::string name) {
    draw_pin_place.clear();
    draw_pin_bool = true;
    for (unsigned i = 0; i < global->POIs.size(); i++) {
        if (name == global->POIs[i].name) {
            draw_pin_place.push_back(global->POIs[i].position);
        }
    }

    t_color current(230, 230, 230);
    setcolor(current);
    fillrect(45, 47, 300, 80);

    current = t_color(150, 150, 150);
    setcolor(current);
    drawtext(55, 55, name, 280, 75);

    draw_screen();
}

void find_intersection_from_search() {

    global->street_names[0] = remove_white_space(global->street_names[0]);
    global->street_names[1] = remove_white_space(global->street_names[1]);
    //find the intersection
    std::vector<unsigned > intersection_ids = find_intersection_ids_from_street_names(global->street_names[0], global->street_names[1]);
    //empty vector of pin coordinates
    draw_pin_place.clear();
    draw_path_end_bool = false;
    draw_path_intersections.clear();
    draw_pin_bool = true;

    for (unsigned i = 0; i < intersection_ids.size(); i++) {

        print_intersection_data(intersection_ids[i]);
        draw_intersection_info(intersection_ids[i]);

        //convert into t_point
        t_point intersectionXY = global->LatLon_to_tpoint(global->intersections[intersection_ids[i]].position);
        draw_pin_place.push_back(intersectionXY);

    }
    draw_screen();
}

void draw_intersection_info(unsigned intersectionID) {
    set_drawing_buffer(ON_SCREEN);

    if (intersectionID < global->intersections.size()) {
        update_message(global->intersections[intersectionID].name);
    }
}

void parse_input() {

    //parse the input string stream of the search bar
    //store the strings in the street name vector 

    global->street_names.push_back(global->user_query);

    //street_names vector now contains the user's arguments 
    if ((global->street_names.size() == 1) && search_interaction) {
        //otherwise save into one word 
        std::string pointOfInterest = global->street_names[0];
        find_POI_by_name(pointOfInterest);

    } else if (global->street_names.size() == 1) {
        //do nothing
    } else if ((global->street_names.size() == 2) && search_interaction) {
        //call the find function 
        find_intersection_from_search();
    } else if (global-> street_names.size() == 2) {
        //do nothing 
    } else if (global->street_names.size() == 3) {
        //user is finding path from an intersection to a POI
        display_path_to_POI();

    } else if (global->street_names.size() == 4) {
        //user is finding path from one intersection to a second intersection
        display_path_between_intersections();

    } else {
        //too many arguments
        global->street_names.clear();
    }

}

void display_path_to_POI() {

    path.clear();
    POI_mode = false;
    draw_path_intersections.clear();
    bool POI_found = false;

    //check if the third entry is a valid POI
    std::string pointOfInterest = remove_white_space(global->street_names[2]);

    //prepare to save the POI ID and position
    unsigned id = 0;
    t_point position;

    for (unsigned i = 0; i < global->POIs.size(); i++) {
        if (pointOfInterest == global->POIs[i].name) {

            POI_found = true;

            //save the POI ID and position 
            id = i;
            position = global->POIs[i].position;
            break;

        }
    }

    if (POI_found) {
        //check if the first two street names are in fact an intersection 
        std::vector<unsigned > intersection_id1 = find_intersection_ids_from_street_names(remove_white_space(global->street_names[0]), remove_white_space(global->street_names[1]));
        if (intersection_id1.size() == 0) {
            //the intersection does not exist
            //display an error message
            update_message("Error: The source point does not exist.");
            return;

        } else {
            //set this as the start point
            POI_mode = true;
            //draw the route to the POI

            //load start point intersection ID and coordinates into data structure
            t_point intersectionXY = global->LatLon_to_tpoint(global->intersections[intersection_id1[0]].position);
            std::pair<unsigned, t_point> point_to_insert(intersection_id1[0], intersectionXY);
            draw_path_intersections.push_back(point_to_insert);

            //load POI intersection ID and coordinates into data structure
            point_to_insert = std::make_pair(id, position);
            draw_path_intersections.push_back(point_to_insert);

            //Get ready to draw path between two intersections
            draw_path_end_bool = true;
            path_already_calculated = false;
            draw_screen();


        }
    } else {
        //the 3rd entry is not a valid POI
        //display an error message
        update_message("Error: The point of interest does not exist.");
        return;
    }

}

void display_path_between_intersections() {

    //check if the first 2 names are a valid intersection
    std::vector<unsigned > intersection_id1 = find_intersection_ids_from_street_names(remove_white_space(global->street_names[0]), remove_white_space(global->street_names[1]));
    if (intersection_id1.size() == 0) {
        //the intersection does not exist
        update_message("Error: The source point does not exist.");
        //display an error message
        return;
    } else {
        //check if the second 2 street names are a valid intersection
        std::vector<unsigned > intersection_id2 = find_intersection_ids_from_street_names(remove_white_space(global->street_names[2]), remove_white_space(global->street_names[3]));
        if (intersection_id2.size() == 0) {
            //the intersection does not exist
            //display an error message
            update_message("Error: The destination does not exist.");
            return;
        } else {
            //They are both valid points!! :D Draw the path between them! :)

            //load start point intersection ID and coordinates into data structure
            t_point intersectionXY = global->LatLon_to_tpoint(global->intersections[intersection_id1[0]].position);
            std::pair<unsigned, t_point> point_to_insert(intersection_id1[0], intersectionXY);
            draw_path_intersections.push_back(point_to_insert);

            //load end point intersection ID and coordinates into data structure
            intersectionXY = global->LatLon_to_tpoint(global->intersections[intersection_id2[0]].position);
            point_to_insert = std::make_pair(intersection_id2[0], intersectionXY);
            draw_path_intersections.push_back(point_to_insert);

            //Get ready to draw path between two intersections
            draw_path_end_bool = true;
            path_already_calculated = false;
            draw_screen();
        }
    }
}

std::string remove_white_space(std::string name) {

    char space = ' ';
    while (name[0] == space) {
        //remove preceding whitespace
        name = name.erase(0, 1);
    }

    while (name[name.size() - 1] == space) {
        //remove trailing whitespace
        name = name.substr(0, name.size() - 1);
    }

    while (!isalpha(name[0])) {
        //remove the last character 
        name = name.erase(0, 1);
    }

    while (!isalpha(name[name.size() - 1])) {
        //remove the last character 
        name = name.substr(0, name.size() - 1);
    }

    return name;
}

//asks user to find path between two intersections

void path_find_button(void (*) (void)) {
    path.clear();
    POI_mode = false;
    draw_path_intersections.clear();

       std::cout << "Enter first intersection - separate each street name by an enter >" << std::endl;

    std::vector<std::string> street_names_local(4);
    getline(std::cin, street_names_local[0]);
    getline(std::cin, street_names_local[1]);

    std::vector<unsigned > intersection_id1 = find_intersection_ids_from_street_names(street_names_local[0], street_names_local[1]);
    if (intersection_id1.size() == 0) {
        return;
    } else {
        print_intersection_data(intersection_id1[0]);
    }
    //load start point intersection ID and coordinates into data structure
    t_point intersectionXY = global->LatLon_to_tpoint(global->intersections[intersection_id1[0]].position);
    std::pair<unsigned, t_point> point_to_insert(intersection_id1[0], intersectionXY);
    draw_path_intersections.push_back(point_to_insert); 


    std::cout << "Enter second intersection - separate each street name by enter >" << std::endl;
    std::cout << "Or enter the name of a point of interest followed by enter>" << std::endl;
    getline(std::cin, street_names_local[2]);
    //check whether name is POI 
    for (unsigned i = 0; i < global->POIs.size(); i++) {
        if (street_names_local[2] == global->POIs[i].name) {
            //load end point intersection ID and coordinates into data structure
            point_to_insert = std::make_pair(i, global->POIs[i].position);
            draw_path_intersections.push_back(point_to_insert);
            POI_mode = true;
            std::cout << "Poi mode entered" << std::endl;
            break;
        }
    }
    if (POI_mode == false) {
        getline(std::cin, street_names_local[3]);
        //check that intersection exist

        std::vector<unsigned > intersection_id2 = find_intersection_ids_from_street_names(street_names_local[2], street_names_local[3]);
        if (intersection_id2.size() == 0) {
            std::cout << "Point of interest entered does not exits" << std::endl;
            return;
        } else {
            print_intersection_data(intersection_id2[0]);
        }
        //load end point intersection ID and coordinates into data structure
        intersectionXY = global->LatLon_to_tpoint(global->intersections[intersection_id2[0]].position);
        point_to_insert = std::make_pair(intersection_id2[0], intersectionXY);
        draw_path_intersections.push_back(point_to_insert);
    }

    //Get ready to draw path between two intersections
    draw_path_end_bool = true;
    path_already_calculated = false;


    draw_screen();
}


//Calculates path between intersections, intersection -> POI

void calculate_path() {
    path.clear();
    if (POI_mode == true) {
        path = find_path_to_point_of_interest(draw_path_intersections[0].first,
                getPointOfInterestName(draw_path_intersections[1].first), turn_penalty);
    } else {
        path = find_path_between_intersections(draw_path_intersections[0].first,
                draw_path_intersections[1].first, turn_penalty);

    }
    //check that path is non-zero
    if (path.size() == 0) {
        std::cout << "No route found" << std::endl;
        draw_path_bool = false;
        return;
    }
}

//print detailed directions to the terminal, based on the path found

void print_directions() {

    draw_directions_sidebar();
    std::string print_output = "Directions: ";
    display_text(print_output);

    //start with the first segment 
    //get the name of the current street 
    unsigned seg_count = 0;
    unsigned id = path[seg_count];
    std::string current_street = global->street_segment_info[id].streetName;
    //find the length of the street segment that needs to be traveled 
    double path_length = find_street_segment_length(id);

    //is the next segment still on the path?
    while ((seg_count + 1) < path.size()) {

        //is the next segment on the same street?
        if (global->street_segment_info[path[seg_count]].streetID == global->
                street_segment_info[path[seg_count + 1]].streetID) {
            //yes
            //add the next segment to the path
            path_length += find_street_segment_length(path[seg_count + 1]);

            if ((seg_count + 1) == (path.size() - 1)) {
                //at the end of the path
                //at the destination, break this loop
                seg_count = 100000000;

            } else {
                //not at the end of the path
                //check the next segment 
                seg_count++;
            }

        } else {
            //need to make a turn
            std::stringstream stream;
            stream << std::fixed << std::setprecision(2) << path_length;
            std::string s = stream.str();

            // std::string print_output = "Travel down " + current_street + " for " + std::to_string(trunc_double(path_length)) + " metres";
            print_output = "Travel along " + current_street + " for " + s + " metres";
            display_text(print_output);

            //get the name of the next street 
            std::string next_street = global->street_segment_info[path[seg_count + 1]].streetName;
            //determine whether to turn left or right 
            std::string direction = get_turn_direction(path[seg_count], path[seg_count + 1]);

            //turn onto this next street

            print_output = "Turn " + direction + "onto " + next_street;
            display_text(print_output);

            //set the next street to current street
            current_street = next_street;
            //reset the path length
            id = path[seg_count + 1];
            path_length = find_street_segment_length(id);
            //check the next path segment 
            seg_count++;
        }
    }
    //you have arrived at the destination!
    std::stringstream stream;
    stream << std::fixed << std::setprecision(2) << path_length;
    std::string s = stream.str();

    print_output = "Travel down " + current_street + " for " + s + " metres";
    display_text(print_output);
    print_output = "You have arrived at your destination!";
    display_text(print_output);
}

void draw_directions_sidebar() {
    set_coordinate_system(GL_SCREEN);

    setcolor(t_color(244, 245, 247, 90));

    //find the relative quantities    
    t_bound_box current_view = get_visible_screen();

    double height = -1 * current_view.get_height();
    double width = current_view.get_width();

    fillrect(width - 500, 0, width, height - 50);

    set_coordinate_system(GL_WORLD);
}

void display_text(std::string output) {
    set_coordinate_system(GL_SCREEN);

    //find the current screen width
    t_bound_box current_view = get_visible_screen();
    double width = current_view.get_width();
    double x = width - 490;

    if (output == "Directions: ") {
        global->y = 20;
        setfontsize(20);
    } else {
        setfontsize(10);
        global->y += 15;
    }
    setcolor(BLACK);
    drawtext(x, global->y, output, x - 5, global->y, 0);
    if (output == "Directions: ") {
        global->y += 25;
    }
    set_coordinate_system(GL_WORLD);
}

std::string get_turn_direction(unsigned streetSeg1, unsigned streetSeg2) {

    std::string turn = "\0";
    //find the start and end points of the street you are currently on 
    t_point start1 = global->street_segment_info[streetSeg1].points[0];
    unsigned last1 = global->street_segment_info[streetSeg1].points.size();
    t_point end1 = global->street_segment_info[streetSeg1].points[last1 - 1];

    //create a vector repping this street seg
    t_point Current_Vec(end1.x - start1.x, end1.y - start1.y);

    //find the start and end points of the next street
    t_point start2 = global->street_segment_info[streetSeg2].points[0];
    unsigned last2 = global->street_segment_info[streetSeg2].points.size();
    t_point end2 = global->street_segment_info[streetSeg2].points[last2 - 1];

    //create a vector repping this street seg
    t_point Next_Vec(end2.x - start2.x, end2.y - start2.y);

    //Calculate the cross product of Current_Vec X Next_Vec

    double cross_product = Current_Vec.x * Next_Vec.y - Next_Vec.x * Current_Vec.y;

    if (cross_product < 0) {
        turn = "right "; 
        return turn;
    } else if (cross_product > 0) {
        turn = "left ";
        return turn;
    } else {
        turn = "\0";
        return turn;
    }

}

//Draws path between two selected points when called
//Calls on function to draw end points

void draw_path() {

    draw_path_ends();
    if (draw_path_bool == true) {
        for (unsigned path_it = 0; path_it < path.size(); path_it++) {
            unsigned street_seg_id = path[path_it];

            for (unsigned curves = 1; curves < global->street_segment_info[street_seg_id].points.size(); curves++) {
                //Style of path drawn 
                setlinewidth(3);
                setcolor(t_color(255, 77, 77)); //red
                setlinestyle(SOLID, BUTT);

                drawline(global->street_segment_info[street_seg_id].points[curves - 1], global->street_segment_info[street_seg_id].points[curves]);

            }

        }
        print_directions();
        

    } else
        return;

}

//Draw the start and end pins of the map
//Calls function to calculate path

void draw_path_ends() {
    draw_path_bool = false;
    if (draw_path_end_bool == true && draw_path_intersections.size() > 0) {

        //Check that there are at most 2 elements - remove the first 2 otherwise
        if (draw_path_intersections.size() == 3) {
            std::pair<unsigned, t_point> valid_position = draw_path_intersections[2];
            draw_path_intersections.clear();
            draw_path_intersections.push_back(valid_position);
        }

        if (draw_path_intersections.size() == 2) {
            
            //calculate path if not already calculated for these two points
            if (path_already_calculated == false) {
                calculate_path();
                //for POIs, recalculate endpoint
                recalculate_endpoint();
                path_already_calculated = true;
            }
            //only set path to draw if a path exists
            if(path.size()==0){
                draw_path_bool = false; 
            }
            else{
            draw_path_bool = true;
            }
            //draw second pin at endpoint
            //flag size is 24x24
            t_point offset(-4, -24);
            offset = global->convert_screen_to_world(offset);
            t_point draw_pin_here = draw_path_intersections[1].second + offset;

            Surface pin = load_png_from_file("flag.png");
            draw_surface(pin, draw_pin_here);

        } else {
            path_already_calculated = false;
        }
        //anytime we draw a pin, the FIND pin should be disabled
        draw_pin_bool = false;
        draw_pin_place.clear();

        //draw first pin
        t_point offset(-10, -10);
        offset = global->convert_screen_to_world(offset);
        t_point draw_pin_here = draw_path_intersections[0].second + offset;

        Surface pin = load_png_from_file("poi.png");
        draw_surface(pin, draw_pin_here);
    }
}

//find coordinates of new POI end point, stick in draw_path_intersections[1].second

void recalculate_endpoint() {
    if (POI_mode == true) {
        unsigned intersection_id;
        //in case path size is 0, the endpoint is the start point
        if(path.size() == 0){
            draw_path_intersections[1] = draw_path_intersections[0];
            return; 
        }
        //In case the path size is 1
        if (path.size() == 1) {
            StreetSegmentInfo only_segment = getStreetSegmentInfo(path.front());
            if (only_segment.to == draw_path_intersections[0].first) {
                intersection_id = only_segment.from;
            } else {
                intersection_id = only_segment.to;
            }
        } else {
            StreetSegmentInfo last_segment = getStreetSegmentInfo(path.back());
            StreetSegmentInfo second_last_segment = getStreetSegmentInfo(*(path.end() - 2));

            if (last_segment.to == second_last_segment.to || last_segment.to == second_last_segment.from) {
                intersection_id = last_segment.from;
            } else
                intersection_id = last_segment.to;
        }
        //load new intersection as endpoint
        draw_path_intersections[1] = std::make_pair(intersection_id,
                global->LatLon_to_tpoint(global->intersections[intersection_id].position));
    }
}

//Clear all global variables in m2
void clear_m2_global_var() {
    draw_pin_place.clear();
    draw_path_intersections.clear();
    path.clear();
}
