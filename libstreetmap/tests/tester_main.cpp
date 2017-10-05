#include <iostream>
#include <unittest++/UnitTest++.h>
#include "StreetsDatabaseAPI.h"
#include "m1.h"
#include "m3.h"
/*
 * This is the main that drives running
 * unit tests.
 */
int main(int /*argc*/, char** /*argv*/) {
    //Run the unit tests
    
     bool load_success = load_map("/cad2/ece297s/public/maps/toronto_canada.streets.bin");
        if (!load_success) {
            std::cerr << "Failed to load map '" << "'\n";
            return 2;
        }

        std::cout << "Successfully loaded map '"  << "'\n";
    int num_failures = UnitTest::RunAllTests();

    return num_failures;
}
