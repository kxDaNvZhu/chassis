#include "include/socket_can_transmitter.h"
namespace control_can {


SocketcanTransmitter::SocketcanTransmitter() {

}

bool SocketcanTransmitter::InitDevice() {
	sockaddr_can addr;
	ifreq ifr;
	socketcan_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	printf("socketcan_fd_: %d \n", socketcan_fd_);
	if (socketcan_fd_ < 0)
	{
		perror("scoket_can--> socket:");
		exit(-1); //c++退出函数
	}
	strcpy(ifr.ifr_name, interface.c_str());
	ioctl(socketcan_fd_, SIOCGIFINDEX, &ifr);
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
	if (bind(socketcan_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
	{
		printf("bind err\n");

		perror("scoket_can--> bind:");
		// exit(-1);
	}
	printf("bind suc\n");
	struct can_filter rfilter[12];
	rfilter[0].can_id = CHASSIS_StaFb;
	rfilter[0].can_mask = CAN_SFF_MASK;
	rfilter[1].can_id = Ctrl;
	rfilter[1].can_mask = CAN_SFF_MASK;
	// setsockopt(socketcan_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
	// std::cout<<"set  can port ok."<<std::endl;
	return true;
}


int SocketcanTransmitter::ReadCanFrame(struct can_frame  &can_frame_rev) {
	can_frame frame;
	memset(&frame, 0, sizeof(frame));

	int nbytes = read(socketcan_fd_, &frame, sizeof(frame));
	// std::cout << "ReadCanFrame....."<<std::endl;

	if (nbytes < 0) {
		fprintf(stderr, "cannot read socketcan data\n");
		return -1;
	}
	if (nbytes < sizeof(struct can_frame)) {
		fprintf(stderr, "socketcan read: incomplete CAN frame\n");
		return -1;
	}

	can_frame_rev = frame;
	return 0;
}


int SocketcanTransmitter::WriteCanFrame(const struct can_frame& can_frame_send) {
	can_frame frame;
	memset(&frame, 0, sizeof(frame));
	
	frame = can_frame_send;

	int nbytes = write(socketcan_fd_, &frame, sizeof(frame));

	// std::cout<<"nbytes : "<<nbytes<<std::endl;
	if (nbytes < 0) return -1;
	return 0;
}

}//control_can
