#include "catch.hh"
#include "particle_filter.h"

TEST_CASE("TESTING SAMPLE CLASS"){
    ParticleFilter PF;
    SECTION("setting the str") {
        INFO("Using TestStr") // Only appears on a FAIL
        double sigma_pos [3] = {0.3, 0.3, 0.01};
        PF.init(0.0, 0.0, 0.0, sigma_pos);
        CAPTURE(PF.particles); // Displays this variable on a FAIL
        
        CHECK(PF.init() == "TestStr");
    }
    
    SECTION("test bigStr") {
        sc.setStr("TestStr");
        REQUIRE(sc.bigStr() == "TestStr : 7");
    }
    
    SECTION("Test doubles") {
        sc.setD(1);
        CHECK(sc.getD() == 1);
        sc.setD(1.0/3.0);
        CHECK(sc.getD() == Approx(0.33333)); // Nice
    }
}
