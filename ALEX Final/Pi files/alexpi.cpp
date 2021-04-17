#include <stdio.h>
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>
#include <stdint.h>
//#include "packet.h"
//#include "serial.h"
//#include "serialize.h"
//#include "constants.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "geometry_msgs/Twist.h"
#include <sstream>

#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <errno.h>

#define PORT_NAME			"/dev/ttyACM0"

#define BAUD_RATE			B9600

int exitFlag=0;
sem_t _xmitSema;
int16_t count = 0b0000000000000000;
//std_msgs::Int16 count;
float linX, angZ;
int current_time=0;
#define B(x) S_to_binary_(#x)

//serialize.h
#ifndef __SERIALIZE__
#define SERIALIZE

#include <stdlib.h>

#define PACKET_SIZE		140
#define MAX_DATA_SIZE				128

typedef enum
{
	PACKET_OK = 0,
	PACKET_BAD = 1,
	PACKET_CHECKSUM_BAD = 2,
	PACKET_INCOMPLETE = 3,
	PACKET_COMPLETE = 4
} TResult;

int serialize(char *buffer, void *dataStructure, size_t size);
TResult deserialize(const char *buffer, int len, void *output);

#endif

//serial.h
#ifndef __SERIAL__
#define __SERIAL__
#define MAX_BUFFER_LEN		1024

#include <termios.h>
void startSerial(const char *portName, int baudRate, int byteSize, char parity, int stopBits, int maxAttempts);

int serialRead(char *buffer);
void serialWrite(char *buffer, int len);

void endSerial();
#endif
//serial.cpp
static int _fd;
static struct termios _serOptions;

void startSerial(const char *portName, int baudRate, int byteSize, char parity, int stopBits, int maxAttempts)
{
	_fd = -1;

	int attempt=0;
	while(_fd < 0 && attempt < maxAttempts)
	{
	  printf("ATTEMPTING TO CONNECT TO SERIAL. ATTEMPT # %d of %d.\n", ++attempt, maxAttempts);
   	_fd=open(portName, O_RDWR | O_NOCTTY | O_NDELAY);

		if(_fd < 0)
		{
		  printf("FAILED. TRYING AGAIN IN 5 SECONDS\n");
		  sleep(5);
		}
	}
    
    if(_fd < 0)
		{
      perror("GIVING UP. Unable to open serial port.");
		}
    else
    {
        fcntl(_fd, F_SETFL, 0);
 
        tcgetattr(_fd, &_serOptions);
				cfmakeraw(&_serOptions);
        cfsetispeed(&_serOptions, baudRate);
        cfsetospeed(&_serOptions, baudRate);

        _serOptions.c_cflag|=(CLOCAL | CREAD);

        switch(parity)
        {
                case 'o':
                case 'O':
                _serOptions.c_cflag|=PARENB;
                _serOptions.c_cflag|=PARODD;
                _serOptions.c_iflag |= (INPCK | ISTRIP);
                break;

                case 'e':
                case 'E':
                _serOptions.c_cflag |= PARENB;
                _serOptions.c_cflag &= ~PARODD;
                _serOptions.c_iflag |= (INPCK | ISTRIP);
                break;
                
            default:
                _serOptions.c_cflag&= ~ PARENB;
                break;
            
        }
        
        if(stopBits==2)
            _serOptions.c_cflag |= CSTOPB;
        else
            _serOptions.c_cflag &= ~CSTOPB;
        
        _serOptions.c_cflag &= ~CSIZE;
        
        
        switch(byteSize)
        {
            case 5:
                _serOptions.c_cflag |= CS5;
                break;
                
            case 6:
                _serOptions.c_cflag|=CS6;
                break;
                
            case 7:
                _serOptions.c_cflag|=CS7;
                break;
                
            default:
                _serOptions.c_cflag|=CS8;
        }
    }
    
    // Disable hw flow control
    _serOptions.c_cflag &= ~CRTSCTS;
    
    // Disable sw flow control
    _serOptions.c_iflag &= ~(IXON | IXOFF | IXANY);
    
    
    // Clear canonical input mode
    _serOptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	
		/*
	// Set canonical input mode
	_serOptions.c_lflag |= ICANON; */

    // Set canonical output mode
   //_serOptions.c_oflag |= OPOST;

	//_serOptions.c_oflag &= ~OPOST;
	_serOptions.c_oflag &= OPOST;

    // Set the attributes
    tcsetattr(_fd, TCSANOW, &_serOptions);
}

int serialRead(char *buffer)
{
	ssize_t n=0;

	if(_fd >= 0)
		n = read(_fd, buffer, MAX_BUFFER_LEN);
	
	return n;
}

void serialWrite(char *buffer, int len)
{
	ssize_t n=-1;
	if(_fd >= 0)
	{
		n = write(_fd, (void *) buffer, len);
	}
}

void endSerial()
{
	if(_fd > 0)
		close(_fd);
}

//serialize.cpp
#define MAGIC_NUMBER				0xFCFDFEFF

/* Data size is 4 + 128 + 4 + 1 = 137 bytes. We pad to 140 bytes as this is the nearest divisible by 4 we have. So 
	 we add 3 bytes */

typedef struct comms
{
	uint32_t magic;
	uint32_t dataSize;
	char buffer[MAX_DATA_SIZE];
	unsigned char checksum;
	char dummy[3];
} TComms;

static char _privateBuffer[PACKET_SIZE];

static TResult assemble(char *outputBuffer, const char *inputBuffer, int len)
{
	// For copying to output buffer
	static int counter=0;

	// If there's leftover bytes from the next transmission
	static int leftoverFlag=0;
	static int leftoverCount=0;
	static char leftoverBuffer[PACKET_SIZE];

	int bytesLeft;
	int i;	

	// Copy in leftovers
	if(leftoverFlag)
	{
		int copyCount;
		if(leftoverCount <= PACKET_SIZE)
		{
			leftoverFlag=0;
			copyCount = leftoverCount;
		}
		else
			copyCount = PACKET_SIZE;

		leftoverCount -= copyCount;

		for(i=0; i<copyCount; i++)
		{
			outputBuffer[counter++] = leftoverBuffer[i];
		}
	}

	if(counter + len >= PACKET_SIZE)
	{
		bytesLeft = (PACKET_SIZE - counter);
		leftoverFlag=1;
		int bytesToCopy = len - bytesLeft;

		// Copy to leftover buffer
		for(i=0; i<bytesToCopy; i++)
		{
			leftoverBuffer[leftoverCount+i] = inputBuffer[bytesLeft + i];
		}
		leftoverCount += bytesToCopy;
	}
	else
		bytesLeft = len;

	for(i=0; i<bytesLeft; i++)
		outputBuffer[counter++] = inputBuffer[i];

	if(counter == PACKET_SIZE)
	{
		counter = 0;
		return PACKET_COMPLETE;
	}
	else
		return PACKET_INCOMPLETE;
}


TResult deserialize(const char *buffer, int len, void *output)
{
	TResult result = assemble(_privateBuffer, buffer, len);

	if(result == PACKET_COMPLETE)
	{
		// Extract out the comms packet
		TComms *comms = (TComms *) _privateBuffer;

		// Check that we have a valid packet
		if(comms->magic != MAGIC_NUMBER)
		{
			printf("BAD MAGIC NUMBER. EXPECTED %x GOT %x\n", MAGIC_NUMBER, comms->magic);
			return PACKET_BAD;
		}

		// Packet is valid. Now let's do the checksum
		unsigned char checksum = 0;

		unsigned int i;

		for(i=0; i<comms->dataSize; i++)
			checksum ^= comms->buffer[i];

		if(checksum != comms->checksum)
			return PACKET_CHECKSUM_BAD;
		else
		{
			memcpy(output, comms->buffer, comms->dataSize);
			return PACKET_OK;
		}
	}
	else
		return result;
}

int serialize(char *buffer, void *dataStructure, size_t size)
{
	TComms comms;

	// We use this to detect for malformed packets
	comms.magic = MAGIC_NUMBER;

	// Copy over the data structure
	memcpy(comms.buffer, dataStructure, size);

	// Now we take a checksum
	unsigned char checksum = 0;

	unsigned i;

	for(i=0; i<size; i++)
		checksum ^= comms.buffer[i];

	comms.checksum = checksum;
	comms.dataSize = size;

	memcpy(buffer, &comms, sizeof(TComms));

	return sizeof(TComms);
}


//packet.h
#ifndef __CONTROL_H__
#define __CONTROL_H__

#define MAX_STR_LEN   32
// This packet has 1 + 1 + 2 + 32 + 16 * 4 = 100 bytes
typedef struct
{

	char packetType;
	char command;
	char dummy[2]; // Padding to make up 4 bytes
	char data[MAX_STR_LEN]; // String data
	uint32_t params[16];
} TPacket;


#endif


//constants.h
#ifndef __CONSTANTS_INC__
#define __CONSTANTS_INC__

/* 
 *  This file containts all the packet types, commands
 *  and status constants
 *  
 */

// Packet types
typedef enum
{
  PACKET_TYPE_COMMAND = 0,
  PACKET_TYPE_RESPONSE = 1,
  PACKET_TYPE_ERROR = 2,
  PACKET_TYPE_MESSAGE = 3,
  PACKET_TYPE_HELLO = 4 
} TPacketType;

// Response types. This goes into the command field
typedef enum
{
  RESP_OK = 0,
  RESP_STATUS=1,
  RESP_BAD_PACKET = 2,
  RESP_BAD_CHECKSUM = 3,
  RESP_BAD_COMMAND = 4,
  RESP_BAD_RESPONSE = 5,
  RESP_COLOUR = 6, 
  RESP_ULTRASONIC = 7, //ultrasonic_dist
  RESP_IR = 8, //ir_dist
  RESP_WHEEL = 9 //wheel_dist
} TResponseType;


// Commands
// For direction commands, param[0] = distance in cm to move
// param[1] = speed{
typedef enum
{
  COMMAND_FORWARD = 0,
  COMMAND_REVERSE = 1,
  COMMAND_TURN_LEFT = 2,
  COMMAND_TURN_RIGHT = 3,
  COMMAND_STOP = 4,
  COMMAND_GET_STATS = 5,
  COMMAND_CLEAR_STATS = 6,
  COMMAND_COLOUR = 7,
  COMMAND_REV_LEFT = 8,
  COMMAND_REV_RIGHT = 9
} TCommandType;
#endif



void handleError(TResult error)
{
	switch(error)
	{
		case PACKET_BAD:
			printf("ERROR: Bad Magic Number\n");
			break;

		case PACKET_CHECKSUM_BAD:
			printf("ERROR: Bad checksum\n");
			break;

		default:
			printf("ERROR: UNKNOWN ERROR\n");
	}
}

void handleStatus(TPacket *packet)
{
	/*printf("\n ------- ALEX STATUS REPORT ------- \n\n");
	printf("Left Forward Ticks:\t\t%d\n", packet->params[0]);
	printf("Right Forward Ticks:\t\t%d\n", packet->params[1]);
	printf("Left Reverse Ticks:\t\t%d\n", packet->params[2]);
	printf("Right Reverse Ticks:\t\t%d\n", packet->params[3]);
	printf("Left Forward Ticks Turns:\t%d\n", packet->params[4]);
	printf("Right Forward Ticks Turns:\t%d\n", packet->params[5]);
	printf("Left Reverse Ticks Turns:\t%d\n", packet->params[6]);
	printf("Right Reverse Ticks Turns:\t%d\n", packet->params[7]);
	printf("Forward Distance:\t\t%d\n", packet->params[8]);
	printf("Reverse Distance:\t\t%d\n", packet->params[9]);
	printf("\n---------------------------------------\n\n");*/
}

void handleColour(TPacket *packet)
{


}

static inline unsigned long long S_to_binary_(const char *s)
{
    unsigned long long i = 0;
    while (*s) {
        i <<= 1;
        i += *s++ - '0';
    }
    return i;
}

void handleUltrasonicIR(TPacket *packet) {
	//printf("\n ------- ALEX ULTRASONIC DISTANCE REPORT ------- \n\n");
	//printf("Ultrasonic distance: %d\n", packet->params[0]);
	//S_to_binary_(packet->params[0]);
	//packet->params[0] = packet->params[0];
	uint32_t ultrasonic_dist = (packet->params[0]);//B in front
	//printf("Ultrasonic distance: %d\n", ultrasonic_dist);
	
	//printf("Distance:\t\t%d\n", (packet->params[0]));
	uint32_t ir_val=(packet->params[1]);
	uint32_t col = (packet->params[2]);//B in front
	ROS_INFO_STREAM("col:"<<(col));
	//count |= col;
	ROS_INFO_STREAM("IR:"<<(ir_val));
	count = (ir_val)|(ultrasonic_dist << 2)|col;
	ROS_INFO_STREAM("Dist:"<<(ultrasonic_dist));
	//count |= (ultrasonic_dist << 2);
	//printf("\n---------------------------------------\n\n");
}

void handleIR(TPacket *packet) {
	//printf("\n ------- ALEX IR OBSTACLES REPORT ------- \n\n");
	
	//ROS_INFO_STREAM("IR:"<<(packet->params[1]));
	//count |= (packet->params[1]);
	/*if (packet->params[0] == 0) {
		//printf("IR Left: too close\n"); 
		//count |= 0b0000100000000000;
		count |= (1<<12);
	}
	else {
		//printf("IR Left: no issues\n");
		//count &= 0b1111011111111111;
		count &= ~(1<<12); //0 at 11th, rest same
	}
	if (packet->params[1] == 0) {
		//printf("IR Right: too close\n"); 
		count |= (1<<10);
	}
	else {
		//printf("IR Right: no issues\n");
		//count &= 0b1110111111111111;
		count &= ~(1<<10); //0 at 11th, rest same
	}
	if (packet->params[2] == 0) {
		//printf("IR Rear: too close\n"); 
		count |= (1<<11); //0 at 11th, rest same
	}
	else {
		//printf("IR Rear: no issues\n");
		//count &= 0b1101111111111111;
		count &= ~(1<<11); //0 at 11th, rest same

	}
	if (packet->params[3] == 0) {
		//printf("IR Left: too close\n"); 
		//count |= 0b0000100000000000;
		count |= (1<<9);
	}
	else {
		//printf("IR Right: no issues\n");
		//count &= 0b1111011111111111;
		count &= ~(1<<9); //0 at 11th, rest same
	}

	if (packet->params[4] == 0) {
		//printf("IR Right: no issues\n");
		//count |= (1<<12);
		count |= (1<<8);
	}
	else {
		//printf("IR Right: too close\n"); 
		//count &= 0b1110111111111111;
		count &= ~(1<<8);
	}*/

	//printf("\n---------------------------------------\n\n");
}

void handleWheelDist(TPacket *packet) {
	/*printf("\n ------- ALEX WHEEL DISTANCE REPORT ------- \n\n");
	printf("Left wheel distance: %d cm\n", packet->params[0]);
	printf("Right wheel distance: %d cm\n", packet->params[1]);
	printf("\n---------------------------------------\n\n");*/
}

void handleResponse(TPacket *packet)
{
	// The response code is stored in command
	switch(packet->command)
	{
		case RESP_OK:
			printf("Command OK\n");
		break;

		case RESP_STATUS:
			handleStatus(packet);
		break;
		
		case RESP_COLOUR:
			handleColour(packet);
		break;
	
		case RESP_ULTRASONIC:
			handleUltrasonicIR(packet);
		break;

		case RESP_WHEEL:
			handleWheelDist(packet);
		break;

		default:
			printf("Arduino is confused\n");
	}
}

void handleErrorResponse(TPacket *packet)
{
	// The error code is returned in command
	switch(packet->command)
	{
		case RESP_BAD_PACKET:
			printf("Arduino received bad magic number\n");
		break;

		case RESP_BAD_CHECKSUM:
			printf("Arduino received bad checksum\n");
		break;

		case RESP_BAD_COMMAND:
			printf("Arduino received bad command\n");
		break;

		case RESP_BAD_RESPONSE:
			printf("Arduino received unexpected response\n");
		break;

		default:
			printf("Arduino reports a weird error\n");
	}
}

void handleMessage(TPacket *packet)
{
	//printf("%s\n", packet->data);
}

void handlePacket(TPacket *packet)
{
	switch(packet->packetType)
	{
		case PACKET_TYPE_COMMAND:
				// Only we send command packets, so ignore
			break;

		case PACKET_TYPE_RESPONSE:
				handleResponse(packet);
			break;

		case PACKET_TYPE_ERROR:
				handleErrorResponse(packet);
			break;

		case PACKET_TYPE_MESSAGE:
				handleMessage(packet);
			break;
	}
}

void sendPacket(TPacket *packet)
{
	char buffer[PACKET_SIZE];
	int len = serialize(buffer, packet, sizeof(TPacket));

	serialWrite(buffer, len);
	printf("command not ok 6");
}

void *receiveThread(void *p)
{
	char buffer[PACKET_SIZE];
	int len;
	TPacket packet;
	TResult result;
	int counter=0;

	while(1)
	{
		len = serialRead(buffer);
		counter+=len;
		if(len > 0)
		{
			result = deserialize(buffer, len, &packet);

			if(result == PACKET_OK)
			{
				counter=0;
				handlePacket(&packet);
			}
			else 
				if(result != PACKET_INCOMPLETE)
				{
					printf("PACKET ERROR\n");
					handleError(result);
				}
		}
	}
}

void flushInput()
{
	char c;

	while((c = getchar()) != '\n' && c != EOF);
}

void getParams(TPacket *commandPacket)
{
	//printf("Enter distance/angle in cm/degrees (e.g. 50) and power in %% (e.g. 75) separated by space.\n");
	//printf("E.g. 50 75 means go at 50 cm at 75%% power for forward/backward, or 50 degrees left or right turn at 75%%  power\n");
	//scanf("%d %d", &commandPacket->params[0], &commandPacket->params[1]);
	
	//commandPacket->params[0] = angZ; //CHANGE: ANGZ to speed percentage variable

	flushInput();
}

void sendCommand(char command)
{
	printf("command not ok 2");
	TPacket commandPacket;

	commandPacket.packetType = PACKET_TYPE_COMMAND;
	//commandPacket.params[0] = 1; //Ultrasonic & IR
	//commandPacket.params[1] = 0; //Colour 
	
	switch(command)
	{
		case 'f':
		case 'F':
			//getParams(&commandPacket);
			commandPacket.command = COMMAND_FORWARD;
			//printf("command not ok 3");
			sendPacket(&commandPacket);
			//printf("command not ok 4");
			break;

		case 'b':
		case 'B':
			//getParams(&commandPacket);
			commandPacket.command = COMMAND_REVERSE;
			sendPacket(&commandPacket);
			break;

		case 'l':
		case 'L':
			//getParams(&commandPacket);
			commandPacket.command = COMMAND_TURN_LEFT;
			sendPacket(&commandPacket);
			break;

		case 'r':
		case 'R':
			//getParams(&commandPacket);
			commandPacket.command = COMMAND_TURN_RIGHT;
			sendPacket(&commandPacket);
			break;

		case 's':
		case 'S':
			commandPacket.command = COMMAND_STOP;
			sendPacket(&commandPacket);
			//commandPacket.params[1] = 1; 
			break;
			
		case 'n':
		case 'N':
			commandPacket.command = COMMAND_REV_RIGHT;
			sendPacket(&commandPacket);
			break;
			
		case 'v':
		case 'V':
			commandPacket.command = COMMAND_REV_LEFT;
			sendPacket(&commandPacket);
			break;

		case 'c':
		case 'C':
			commandPacket.command = COMMAND_CLEAR_STATS;
			sendPacket(&commandPacket);
			break;

		case 'g':
		case 'G':
			commandPacket.command = COMMAND_GET_STATS;
			sendPacket(&commandPacket);
			break;
			
		case 'x':
		case 'X':
			commandPacket.command = COMMAND_COLOUR;
			sendPacket(&commandPacket);
			break;
			
		case 'q':
		case 'Q':
			exitFlag=1;
			break;

		default:
			printf("Bad command\n");

	}
	//printf("command not ok 5");
}

void data(const geometry_msgs::Twist& vel)//this is where to get cmd_vel msg
{
	char ch;
  linX = vel.linear.x;      angZ = vel.angular.z;
  //geometry_msgs::Twist new_vel=vel;
		if (linX == 0.5 && angZ == 0) {
			ch = 'f';
		}
		else if (linX == 0.5 && angZ == 1) {
			ch = 'l';
		}
		else if (linX == 0.5 && angZ == -1) {
			ch = 'r';
		}
		else if (linX == -0.5 && angZ == 0) {
			ch = 'b';
		} 
		else if (linX == 0 && angZ == 0) {
			ch = 's';
		} 
		else if (linX == -0.5 && angZ == 1) {
			ch = 'n';
		} 
		else if (linX == -0.5 && angZ == -1) {
			ch = 'v';
		} 
		else if (linX == 0 && angZ == 1) {
			ch = 'x';
		} 
		else if (linX == 0 && angZ == -1) {
			ch = 'g';
		}
		sendCommand(ch);
  
  ROS_INFO_STREAM("Subscriber velocities:"<<" linear="<<linX<<" angular="<<angZ);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "alexpi");
	
	 // Connect to the Arduino
	startSerial(PORT_NAME, BAUD_RATE, 8, 'N', 1, 5);

	// Sleep for two seconds
	printf("WAITING TWO SECONDS FOR ARDUINO TO REBOOT\n");
	sleep(2);
	printf("DONE\n");
	
	// Spawn receiver thread
	pthread_t recv;

	pthread_create(&recv, NULL, receiveThread, NULL);

	// Send a hello packet
	TPacket helloPacket;

	helloPacket.packetType = PACKET_TYPE_HELLO;
	sendPacket(&helloPacket);
	
	
	
	ros::NodeHandle n;//
	ros::Subscriber sub = n.subscribe("cmd_vel", 10, &data);//
	ros::Publisher chatter_pub = n.advertise<std_msgs::Int16>("chatter", 1);
	ros::Rate loop_rate(5);
	/*while(!exitFlag)
	{
		char ch;
		printf("Command (f=forward, b=reverse, l=turn left, r=turn right, s=stop, x=colour, c=clear stats, g=get stats q=exit)\n");
		scanf("%c", &ch);
		flushInput();
		sendCommand(ch);
		// Purge extraneous characters from input stream

	}*/
	
	while(ros::ok())
	{	
		std_msgs::Int16 msg;
		//if(time(
		//uint32_t ir_val=(packet->params[1]);0)-current_time>=5000)
		//{
		//current_time = 5000;
		
		//}
		msg.data = count;
		ROS_INFO("%i", msg.data); //broadcast the count value over ROS
		chatter_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
		//sendCommand('g');//send via a different package from move
		
	}
	


	
	
	
	
	ros::spin();
	//printf("Closing connection to Arduino.\n");
	//endSerial();
    return 0;
}
