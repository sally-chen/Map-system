///**********************************************************************/
////Tests the function find_street_ids_from_name in three tests
////Test 1 tests 3 scenarios of functionality; one id return, multiple id return, no id return
////Test 2 tests performance - 1 million calls should execute in < 250ms
////Test 3 tests functionality by comparing implemented function to naive implementation
///**********************************************************************/
//
//#include <algorithm>
//#include <unittest++/UnitTest++.h>
//#include <vector>
//#include "m1.h"
//#include "StreetsDatabaseAPI.h"
//
//
//
//
//struct MapFixture {
//
//    MapFixture() {
//        //Load the map
//        load_map("/cad2/ece297s/public/maps/toronto_canada.streets.bin");
//        maxTests = 1000000;
//        numStreets = getNumberOfStreets();
//        randNum = rand() % (numStreets + 1);
//    }
//
//    ~MapFixture() {
//        //Clean-up
//        close_map();
//    }
//    unsigned numStreets;
//    int maxTests;
//    unsigned randNum;
//    std::vector<unsigned> brute_ids_from_name(std::string street_name);
//};
//
//SUITE(find_street_ids_from_name){
//
//    TEST_FIXTURE(MapFixture, normal_functionality) {
//
//        std::string streetName;
//        std::vector<unsigned> expected;
//        std::vector<unsigned> actual;
//
//        //check for one return id
//        streetName = "Sesame Street";
//        expected = {12499};
//        actual = find_street_ids_from_name(streetName);
//        CHECK(expected == actual);
//
//
//        //check for multiple return ids
//        streetName = "College Street";
//        expected = {197, 13044};
//        actual = find_street_ids_from_name(streetName);
//        std::sort(actual.begin(), actual.end());
//        CHECK(expected == actual);
//
//        //check that it returns null
//        streetName = "DOESNT EXIST!!";
//        expected = {};
//        actual = find_street_ids_from_name(streetName);
//        CHECK(expected == actual);
//
//
//    }
//
//    TEST_FIXTURE(MapFixture, performance) {
//        //create vector to store all the test names
//        std::vector<std::string> testVector;
//
//        //fill vector with test street names
//        for (int i = 0; i < maxTests; i++) {
//            testVector.push_back(getStreetName(randNum));
//        }
//
//        //Time test for 250 ms
//        UNITTEST_TIME_CONSTRAINT(250);
//        for (int i = 0; i < maxTests; i++) {
//            find_street_ids_from_name(testVector[i]);
//
//        }
//
//
//    }
//    
//    TEST_FIXTURE(MapFixture, auto_gen_functionality){
//        std::string streetName;
//        std::vector<unsigned> expected;
//        std::vector<unsigned> actual;
//        
//        for(unsigned i = 0; i < numStreets; i++){
//            streetName = getStreetName(i);
//            expected = brute_ids_from_name(streetName);
//            std::sort(expected.begin(), expected.end());
//            actual = find_street_ids_from_name(streetName);
//            std::sort(actual.begin(), actual.end());
//            CHECK(expected == actual);
//        }
//    }
//
//}
//
////naive implementation for find_street_ids_from_name
////used in the auto-generated functionality test
//std::vector<unsigned> MapFixture::brute_ids_from_name(std::string street_name){
//    //create a vector for return values
//    std::vector<unsigned> return_street_ids;
//    //iterate through all the streets and compare names to input name
//    for(unsigned i = 0; i < numStreets; i++){
//        if(getStreetName(i) == street_name){
//            return_street_ids.push_back(i);
//        }
//    }
//    return return_street_ids;
//}
//
