#include <string>
#include <cmath>

using namespace std;

void setup () {

}

void loop () {

}

class payload {
    private:
    float altitude, altitudeOffset, velocity;
    string flightstate;
    string command = "";
    int payload::sensorSetup() {
        return 1;
    }

    void payload::throwError() {

    }

    public:
    float payload::getPascals () {
        float pA = 0;

        return pA;
    }

    float payload::computeAltitudePreOffset () {
        float p = getPascals();
        float altitude = -1.0 * 7000.0  * log(p/101325.0);

        return altitude;
    }

    void payload::setAltitudeOffset () {
        altitudeOffset = computeAltitudePreOffset();
    }

    void payload::updateAltitude () {
        altitude = computeAltitudePreOffset() - altitudeOffset;
    }

    void payload::initialize () {
        flightstate = "standby";
        setAltitudeOffset();
    }

    void payload::releaseContainer () {

    }

    void payload::deployShield () {

    }

    void payload::deployChute () {

    }

    void payload::raiseFlag () {

    }

    void payload::signalGroundCrews () {

    }

    void stateMachineUpdate () {
        if (command == "arm" && flightstate == "standby") {
            flightstate = "armed";
        } else if (altitude > 500 && flightstate == "armed") {
            flightstate = "in_flight";
        } else if (altitude <= 500 && flightstate == "in_flight") {
            releaseContainer();
            deployShield();
            flightstate = "shielded";
        } else if (altitude <= 200 && flightstate == "shielded") {
            deployChute();
            flightstate = "landing";
        } else if (velocity <= 1 && flightstate == "landing") {
            raiseFlag();
            signalGroundCrews();
            flightstate = "landed";
        }
    }

};