#include "test_imu.h"
#include "algo/myocontrol.h"
#include "utils/check_ptr.h"
#include <filesystem>
#include "ui/visual/ledstrip.h"
#include "utils/log/log.h"
#include <iostream>
#include "components/internal/actuators/roboclaw/factory.h"

TestIMU::TestIMU(std::shared_ptr<SAM::Components> robot)
    : ThreadedLoop("Test RS232", 0.1)
    , _robot(robot)
{
    _menu->set_description("Test RS232");
    _menu->set_code("test"); /*
    try {
        _serial_port = RC::Factory::get("/dev/ttyAMA0", B115200);
        critical() << "Create test RS232";
    } catch (std::exception& e) {
        critical() << "Couldn't create test RS232 : (" << e.what() << ")";
    }
*/
}

TestIMU::~TestIMU()
{
    stop_and_join();
}

bool TestIMU::setup()
{
    _serial_port.open("/dev/ttyAMA0",B115200);
    std::cout << "bonjour" << std::endl;
    return true;
}

void TestIMU::loop(double dt, clock::time_point time)
{
    std::cout << "test" << std::endl;
    send();
}

void TestIMU::cleanup()
{
    std::cout << "au revoir" << std::endl;
}

void TestIMU::send()
{
    //_serial_port->take_ownership();
    auto data = _serial_port.read_all();
    for(auto c:data) {
        std::cout << (char)c << " (" << (int)c << ") ";
    }
    std::cout << std::endl;

    _serial_port.write("testjjhgkjB");

    //_serial_port->release_ownership();

}
