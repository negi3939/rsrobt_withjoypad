#ifndef SERIAL_SETTING_H
#define SERIAL_SETTING_H

#define DEV_NAME "/dev/ttyACM0"
#define BAUD_RATE B115200
#define DATA_SIZE (4)
#define BUFF_SIZE (3*DATA_SIZE+2)
#define BUFR_SIZE (16000)

#endif

#ifndef SERIAL_SUB
#define SERIAL_SUB
void serial_init(int fd) {
	struct termios tio,tio_backup;
	ioctl(fd, TCGETS, &tio_backup);
	tio = tio_backup;
	tio.c_cflag = 0;
	tio.c_cflag |= CREAD; //
	tio.c_cflag |= CLOCAL; //
	tio.c_cflag |= CS8; //
	tio.c_cflag |= 0; // 
	tio.c_cflag |= 0; // 
	tio.c_cflag |= IGNBRK;
	tio.c_cflag |= IGNPAR;
	tio.c_cc[VTIME] = 0;
	tio.c_cc[VMIN] = DATA_SIZE;
	cfsetispeed( &tio, BAUD_RATE);
	cfsetospeed( &tio, BAUD_RATE);
	cfmakeraw(&tio); // 
	ioctl(fd, TCSANOW, &tio); // 
	tcflush(fd, TCIFLUSH);
	tcsetattr( fd, TCSANOW, &tio ); // 
	std::cout << "intialized fd:"<< fd << std::endl;
}

#endif