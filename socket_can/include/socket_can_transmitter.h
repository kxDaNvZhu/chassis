#ifndef __TRANSMITTER_BASE_H__
#define __TRANSMITTER_BASE_H__

#include <sys/types.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <math.h>
#include <string>
#include <cstring>
#include <cstdio>

namespace control_can {
// struct can_frame CAN_frame;

#define CHASSIS_StaFb 0x0011
#define Ctrl 0x0010 


class TransmitterBase
{
public:
	TransmitterBase() {

	}
	virtual ~TransmitterBase() {

	}
	virtual bool InitDevice() = 0;
	virtual int ReadCanFrame(struct can_frame &frame) = 0;
	virtual int WriteCanFrame(const struct can_frame &frame) = 0;

};


// typedef struct SocketCanConfig
// {
// 	std::string can_name;
// 	SocketCanConfig() {
// 		can_name = "can0";
// 	}
// } SocketCanConfig;


class SocketcanTransmitter: public TransmitterBase
{
public:
	SocketcanTransmitter() ;
	~SocketcanTransmitter() {

	}

	bool InitDevice() override;
	int ReadCanFrame(struct can_frame &frame) override;
	int WriteCanFrame(const struct can_frame &frame) override;

private:
	int socketcan_fd_;
	const std::string interface = "can0";
};


}

#endif