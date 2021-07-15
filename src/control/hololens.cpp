#include "hololens.h"

Hololens::Hololens(std::shared_ptr<SAM::Components> robot)
    : ThreadedLoop("hololens",0.01)
    , _robot(robot)
{
    _menu->set_description("Hololens receiver");
    _menu->set_code("hololens");

    int port = 7000;
    if (!_socket.bind("0.0.0.0", port)) {
        critical() << "Failed to bind on port" << port;
    }
}

Hololens::~Hololens()
{
    stop_and_join();
}

bool Hololens::setup()
{
    std::cout << "setup ok" << std::endl;
    return true;
}

void Hololens::loop(double, clock::time_point)
{
    char addressPattern[MAX_OSC_ADDRESS_PATTERN_LENGTH + 1];
    //std::cout << "test" << std::endl;
    std::vector<std::byte> buf;

    while (_socket.available()) {
        buf = _socket.receive();

//        for(unsigned int i=0; i< buf.size(); i++)
//            std::cout << static_cast<char>(buf[i]);
//        std::cout << std::endl;
//        std::cout << buf.size() << std::endl;
    }
    if (!buf.empty())
       buf.push_back(static_cast<std::byte>(0xC0));

    OscError oscError = OscErrorNone;
    for (unsigned int i = 0; i < buf.size(); i++) {
        oscError = _oscSlipDecoder.OscSlipProcessByte(static_cast<char>(buf[i]), &_oscMessage, &_oscTimeTag);
        if (oscError != OscErrorNone) {
            debug() << OscErrorGetMessage(oscError);
            return;
        } else if (static_cast<char>(buf[i]) == static_cast<char>(0xC0)) {
            _oscMessage.OscMessageGetAddressPattern(addressPattern);
            if (OscAddressMatch(addressPattern, "/Hololens")) {
                oscError = processData(_oscMessage);
            } else {
                debug() << "OSC address pattern not recognised : " << addressPattern;
            }
        }
    }
}

OscError Hololens::processData(OscMessage oscMessage)
{
    int datatest;

    OscError oscError;
    oscError = oscMessage.OscMessageGetArgumentAsInt32(&datatest);
    if (oscError != OscErrorNone) {
        return oscError;
    } else {
        std::cout << datatest << std::endl;
        switch (datatest) {
        case 1:
            _robot->joints.hand_quantum->makeContraction(QuantumHand::CO_CONTRACTION);
            break;
        case 2:
            _robot->joints.hand_quantum->makeContraction(QuantumHand::DOUBLE_CONTRACTION);
            break;
        case 3:
            _robot->joints.hand_quantum->makeContraction(QuantumHand::TRIPLE_CONTRACTION);
            break;
        default:
            break;
        }
    }
    return OscErrorNone;
}

void Hololens::cleanup()
{

}
