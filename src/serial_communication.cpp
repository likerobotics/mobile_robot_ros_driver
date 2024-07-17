#include "serial_communication.h"

#include <unistd.h> // fixes "usleep" error

SerialCommunication::SerialCommunication(const std::string serialPort){
    portName = new char [serialPort.length()+1];
    strcpy(portName, serialPort.c_str());
    
}

SerialCommunication::~SerialCommunication(){
    this->stop();
}

void SerialCommunication::run()
{
    handleSerialThread = std::thread{&SerialCommunication::handleSerial, this};   
}


void SerialCommunication::stop()
{
    handleSerialThread.join();
}


void SerialCommunication::handleSerial()
{
    char n = '\n';
    int bytesSent = 0;
    int bytesRead = 0;

    const int bufferSize = (robotState.jointsPosition.rows()+robotState.jointsVelocity.rows() + robotState.odomPose.rows())*sizeof(float)+2;
    float stateVector[robotState.jointsPosition.rows()+robotState.jointsVelocity.rows() + robotState.odomPose.rows()] = {0};
    char buffer[bufferSize] = {0};
    uint8_t readChecksum;
    uint8_t calcChecksum;

    int fd = openSerialPort(portName);
    configSerialPort(fd);
    connectionEnstablished = false;
    while (1)
    {
        //tx
        {
            std::unique_lock<std::mutex> lock(mutexControl);
            calcChecksum = crc8((uint8_t *)linearVelocityControl.data(), linearVelocityControl.rows()*sizeof(float));
            bytesSent = 0;
            bytesSent += write(fd, linearVelocityControl.data(), linearVelocityControl.rows()*sizeof(float));
            bytesSent += write(fd, &calcChecksum, 1);
            bytesSent += write(fd, &n, 1);
        }

        //rx
        for (int i = 0; i < bufferSize; i++)
        {
            bytesRead = read(fd, buffer + i, 1);
        }

        if (buffer[bufferSize-1] == '\n')
        {
            memcpy(stateVector, buffer, bufferSize-2);
            readChecksum = buffer[bufferSize-2];
            calcChecksum = crc8((uint8_t *)stateVector, bufferSize-2);
            if (readChecksum == calcChecksum)
            { 
                connectionEnstablished = true;
                std::unique_lock<std::mutex> lock(mutexState);
                memcpy(robotState.jointsPosition.data(), stateVector, robotState.jointsPosition.rows()*sizeof(float));
                memcpy(robotState.jointsVelocity.data(), &(stateVector[robotState.jointsPosition.rows()]), robotState.jointsVelocity.rows()*sizeof(float));
                memcpy(robotState.odomPose.data(), &(stateVector[robotState.jointsPosition.rows() + robotState.jointsVelocity.rows()]), robotState.odomPose.rows()*sizeof(float));
                // printf("Got joint angles [%f, %f, %f, %f] \n", robotState.jointsPosition[0], robotState.jointsPosition[1], robotState.jointsPosition[2], robotState.jointsPosition[3]);
                // printf("Got joint velocities [%f, %f, %f, %f]\n", robotState.jointsVelocity[0], robotState.jointsVelocity[1], robotState.jointsVelocity[2], robotState.jointsVelocity[3]);
            }
        }
    }
}

int SerialCommunication::openSerialPort(const char *port)
{
    int fd;                                        // file descriptor for the port
    fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY); // Open for reading and writing, not as controlling terminal, no delay (no sleep - keep awake) (see Source 1)
    if (fd == -1)
    {
        perror("\nError: could not open specified serial port\n");
    }
    else
    {
        fcntl(fd, F_SETFL, 0); // Manipulates the file descriptor ands sets status FLAGS to 0 (see Source 4)
                                // Block (wait) until characters come in or interval timer expires
    }
    return (fd);
}

void SerialCommunication::configSerialPort(int fd)
{
    struct termios options;
    tcgetattr(fd, &options); // Fills the termios structure with the current serial port configuration

    // Change the current settings to new values

    // Set baud rate
    cfsetispeed(&options, BAUDRATE); // Input speed (rate)  -- Most systems don't support different input and output speeds so keep these the same for portability
    cfsetospeed(&options, BAUDRATE); // Output speed (rate)

    // Enable the receiver and set as local mode - CLOCAL & CREAD should always be enabled.
    // CLOCAL so that program does not become owner of the port
    // CREAD so that the serial interface will read incoming data
    options.c_cflag |= (CLOCAL | CREAD);

    // Set other options to match settings on Windows side of comm (No parity, 1 stop bit) - See Source 1
    options.c_cflag &= ~PARENB;                         // PARITY NOT ENABLED
    options.c_cflag &= ~CSTOPB;                         // CSTOPB = 2 stop bits (1 otherwise). Therefore ~CSTOPB means 1 stop bit
    options.c_cflag &= ~CSIZE;                          // Mask the character size to be in bits
    options.c_cflag |= CS8;                             // Use 8 bit data (per character)
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Set the port for RAW input with no echo or other input signals
    options.c_oflag &= ~OPOST;                          // Use RAW output mode
    options.c_cc[VMIN] = 0;                             // Min # of chars to read. When 0 VTIME specifies the wait time for every character
    options.c_cc[VTIME] = 1;                            // Time in 1/10ths of a second to wait for every character before timing out.
                                                        // If VTIME is set to 0 then reads will block forever (port will only read)
                                                        // 24 Second timeout
    // Apply the new options to the port
    tcsetattr(fd, TCSANOW, &options); // TCSANOW makes changes without waiting for data to complete
}

bool SerialCommunication::isConnectionEnstablished(){
    return connectionEnstablished;
}

uint8_t SerialCommunication::crc8(uint8_t *p, uint8_t len)
{
    uint16_t i;
    uint16_t crc = 0x0;

    while (len--)
    {
        i = (crc ^ *p++) & 0xFF;
        crc = (crc_table[i] ^ (crc << 8)) & 0xFF;
    }

    return crc & 0xFF;
}


