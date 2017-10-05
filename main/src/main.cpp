#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include "m1.h"
#include "m2.h"
#include "m3.h"
#include "m4.h"
#include "graphics.h"

//change to max and min lat lon coords

std::string city_name(std::string city_name_input);

int main(int argc, char** argv) {

    std::string map_path;
    bool load_another_map = false;

    do {
        if (argc == 1) {
            //Ask user for which city they want to see
            std::cout << "Enter name of city" << std::endl;
            std::string city_name_string;
            getline(std::cin, city_name_string);
            map_path = city_name(city_name_string);


            //map_path = "/cad2/ece297s/public/maps/toronto_canada.streets.bin";
        } else if (argc == 2) {
            //Get the map from the command line
            map_path = argv[1];
        } else {
            //Invalid arguments
            std::cerr << "Usage: " << argv[0] << " [map_file_path]\n";
            std::cerr << "  If no map_file_path is provided a default map is loaded.\n";
            return 1;
        }

        //Load the map and related data structures
        bool load_success = load_map(map_path);
        if (!load_success) {
            std::cerr << "Failed to load map '" << map_path << "'\n";
            return 2;
        }

        std::cout << "Successfully loaded map '" << map_path << "'\n";
       
         //draw the map in the easy GL window 
        
//        std::vector<DeliveryInfo> deliveries;
//        std::vector<unsigned> depots;
//        std::vector<unsigned> result_path;
//        float turn_penalty;
//        deliveries = {DeliveryInfo(7552, 11395), DeliveryInfo(20877, 76067), DeliveryInfo(76915, 76067), DeliveryInfo(108204, 76067), DeliveryInfo(108204, 99189), DeliveryInfo(32523, 99189), DeliveryInfo(32523, 92242), DeliveryInfo(32523, 99189), DeliveryInfo(32523, 52119)};
//        depots = {32736, 27838, 28149};
//        turn_penalty = 15;
//        result_path = traveling_courier(deliveries, depots, turn_penalty);
        draw_map();
        
        //Clean-up the map related data structures
        std::cout << "Closing map\n";
        close_map();
        
        std::string yes_or_no;
        std::cout << "Do you want to see another map (y/n)?" << std::endl;

        if (!std::cin.eof()) {
            getline(std::cin, yes_or_no);
            if (yes_or_no == "y" || yes_or_no == "Y")
                load_another_map = true;
            else
                load_another_map = false; 
        }
    } while (load_another_map == 1);

    

    return 0;
}

std::string city_name(std::string city_name_input){
    if(city_name_input.find("Beijing")!= std::string::npos
            ||city_name_input.find("beijing")!= std::string::npos){
        return "/cad2/ece297s/public/maps/beijing_china.streets.bin";
    }
    if(city_name_input.find("Cairo")!= std::string::npos
            ||city_name_input.find("cairo")!= std::string::npos){
        return "/cad2/ece297s/public/maps/cairo_egypt.streets.bin";
    }
    if(city_name_input.find("Cape Town")!= std::string::npos
            ||city_name_input.find("cape town")!= std::string::npos){
        return "/cad2/ece297s/public/maps/cape-town_south-africa.streets.bin";
    }
    if(city_name_input.find("Golden Horseshoe")!= std::string::npos
            ||city_name_input.find("golden horseshoe")!= std::string::npos){
        return "/cad2/ece297s/public/maps/golden-horseshoe_canada.streets.bin";
    }
    if(city_name_input.find("hamilton")!= std::string::npos
            ||city_name_input.find("Hamilton")!= std::string::npos){
        return "/cad2/ece297s/public/maps/hamilton_canada.streets.bin";
    }
    if(city_name_input.find("Hong Kong")!= std::string::npos
            ||city_name_input.find("hong kong")!= std::string::npos){
        return "/cad2/ece297s/public/maps/hong-kong_china.streets.bin";
    }
    if(city_name_input.find("Iceland")!= std::string::npos
            ||city_name_input.find("iceland")!= std::string::npos){
        return "/cad2/ece297s/public/maps/iceland.streets.bin";
    }
    if(city_name_input.find("London")!= std::string::npos
            ||city_name_input.find("london")!= std::string::npos){
        return "/cad2/ece297s/public/maps/london_england.streets.bin";
    }
    if(city_name_input.find("Moscow")!= std::string::npos
            ||city_name_input.find("moscow")!= std::string::npos){
        return "/cad2/ece297s/public/maps/moscow_russia.streets.bin";
    }
    if(city_name_input.find("New Delhi")!= std::string::npos
            ||city_name_input.find("new delhi")!= std::string::npos){
        return "/cad2/ece297s/public/maps/new-delhi_india.streets.bin";
    }
    if(city_name_input.find("New York")!= std::string::npos
            ||city_name_input.find("new york")!= std::string::npos){
        return "/cad2/ece297s/public/maps/new-york_usa.streets.bin";
    }
    if(city_name_input.find("Rio")!= std::string::npos
            ||city_name_input.find("rio de janeiro")!= std::string::npos){
        return "/cad2/ece297s/public/maps/rio-de-janeiro_brazil.streets.bin";
    }
    if(city_name_input.find("Saint Helena")!= std::string::npos
            ||city_name_input.find("saint helena")!= std::string::npos){
        return "/cad2/ece297s/public/maps/saint-helena.streets.bin";
    }
    if(city_name_input.find("Singapore")!= std::string::npos
            ||city_name_input.find("singapore")!= std::string::npos){
        return "/cad2/ece297s/public/maps/singapore.streets.bin";
    }
    if(city_name_input.find("Sydney")!= std::string::npos
            ||city_name_input.find("sydney")!= std::string::npos){
        return "/cad2/ece297s/public/maps/sydney_australia.streets.bin";
    }
    if(city_name_input.find("Tehran")!= std::string::npos
            ||city_name_input.find("tehran")!= std::string::npos){
        return "/cad2/ece297s/public/maps/tehran_iran.streets.bin";
    }
    if(city_name_input.find("Tokyo")!= std::string::npos
            ||city_name_input.find("tokyo")!= std::string::npos){
        return "/cad2/ece297s/public/maps/tokyo_japan.streets.bin";
    }
    if(city_name_input.find("Toronto")!= std::string::npos
            ||city_name_input.find("toronto")!= std::string::npos){
        return "/cad2/ece297s/public/maps/toronto_canada.streets.bin";
    }
    else{
        std::cout<< "City map does not exist. Default set to Toronto" << std::endl;
        return "/cad2/ece297s/public/maps/toronto_canada.streets.bin";
    }
}
