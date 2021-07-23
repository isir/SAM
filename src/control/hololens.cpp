#include "hololens.h"
#include <string>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>


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
    struct sockaddr addr;

    while (_socket.available()) {
        buf = _socket.receive(addr);

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
                oscError = processData(_oscMessage, addr);
                char ipstr[INET6_ADDRSTRLEN];
                std::cout << "from IP address " << inet_ntop(addr.sa_family,&((struct sockaddr_in *)&addr)->sin_addr, ipstr, sizeof ipstr) << std::endl;
            } else {
                debug() << "OSC address pattern not recognised : " << addressPattern;
            }
        }
    }
}

OscError Hololens::processData(OscMessage oscMessage, struct sockaddr addr)
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
            std::cout << "co contraction" << std::endl;
            sendIntMessage(1, addr);
            break;
        case 2:
            _robot->joints.hand_quantum->makeContraction(QuantumHand::DOUBLE_CONTRACTION);
            std::cout << "double contraction" << std::endl;
            sendIntMessage(2, addr);
            break;
        case 3:
            _robot->joints.hand_quantum->makeContraction(QuantumHand::TRIPLE_CONTRACTION);
            std::cout << "triple contraction" << std::endl;
            sendIntMessage(3, addr);
            break;
        default:
            break;
        }
    }
    return OscErrorNone;
}

OscError Hololens::sendIntMessage(int number, struct sockaddr addr) {
  OscError oscError = OscErrorNone;
  OscMessage oscMessage;
  oscMessage.OscMessageInitialise("/example");
  oscMessage.OscMessageAddInt32(number);
  oscError = sendOscContents(oscMessage, addr);
  return oscError;
}

OscError Hololens::sendOscContents(OscMessage oscMessage, struct sockaddr addr) {

/*  // Create OSC packet from OSC message or OSC bundle
  OscPacket oscPacket;
  if(oscPacket.OscPacketInitialiseFromContents(oscContents) != OscErrorNone) {
    return; // error: unable to create an OSC packet from the OSC contents
  }

  // Encode SLIP packet
  char slipPacket[MAX_OSC_PACKET_SIZE];
  size_t slipPacketSize;
  if(_oscSlipDecoder.OscSlipEncodePacket(&oscPacket, &slipPacketSize, slipPacket, sizeof (slipPacket)) != OscErrorNone) {
    return; // error: the encoded SLIP packet is too long for the size of slipPacket
  }

  // Send SLIP packet
  Serial.write((uint8_t*)slipPacket, slipPacketSize); // typecast from char* to uint8_t*
*/
  size_t message_size=0;
  char message[MAX_OSC_PACKET_SIZE];
  OscError oscError = OscErrorNone;
  oscError = oscMessage.OscMessageToCharArray(&message_size, message, MAX_OSC_PACKET_SIZE);
  std::cout << message_size << std::endl;
  std::cout << message << std::endl;
  if (oscError != OscErrorNone) {
      debug() << "Failed to send OSC message : " << OscErrorGetMessage(oscError);
  } else {
      ssize_t err = _socket.send(message, message_size, addr);
      std::cout << "envoi socket : " << errno << strerror(errno) << std::endl;
  }
  return oscError;
}

void Hololens::cleanup()
{

}
