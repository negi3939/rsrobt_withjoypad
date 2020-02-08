#include <iostream>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

#include <linux/joystick.h>

#define JOYDEVNAME "/dev/input/js0"

#define JOYAXIS_NUM (4)

enum { JOYBTN_0=0, JOYBTN_1, JOYBTN_2, JOYBTN_3, 
       JOYBTN_4, JOYBTN_5, JOYBTN_6, JOYBTN_7, JOYBTN_NUM };

const uint8_t joybtn_map[JOYBTN_NUM] = { 2-1, 3-1, 1-1, 4-1, 10-1, 7-1, 8-1, 9-1};

struct Jostate{
  int16_t axis[JOYAXIS_NUM];
  uint8_t btn_state[JOYBTN_NUM];
  uint8_t btn_edge[JOYBTN_NUM];
};

struct Jostate joyState;

main()
{
  int joydev, size;
  struct js_event js;

  joydev=open(JOYDEVNAME, O_RDONLY);
  if (joydev <0) {
    perror("error in open("JOYDEVNAME"): ");
    exit(1);
  }


  while (1) {
/*
    char buf[8];
    size = read(joydev, buf, 8);
    time = (buf[0]<<24) + (buf[1]<<16) + (buf[2]<<8) + buf[3];
    value = buf[4]*256 + buf[5];
    type = buf[6];
    number = buf[7];
*/
    size = read(joydev, &js, sizeof(js));
    if (size != sizeof(js)) {
      perror("size error:");
      exit(1);
    }

    if(js.type & JS_EVENT_AXIS) {
      if (js.number < JOYAXIS_NUM) {
	if (js.number==1 || js.number==3) js.value *= -1;
	joyState.axis[js.number] = js.value;
#if 1
      } else {
	fprintf(stderr, "JS_EVENT_AXIS: js.number %d is out of range.\n",
		js.number);
#endif
      }

    } else if (js.type & JS_EVENT_BUTTON) {
      int btn;
      for(btn=0; btn<JOYBTN_NUM; btn++) {
	joyState.btn_edge[btn]=0;
	
	if (joybtn_map[btn] == js.number) {
	  if (joyState.btn_state[btn] < js.value) {
	    joyState.btn_edge[btn]=2; // leading edge
	  } else if (joyState.btn_state[btn] > js.value) {
	    joyState.btn_edge[btn]=1; // falling edg
	  }
	  joyState.btn_state[btn] = js.value;
	}
      }
    }

#if 1
    fprintf(stderr, "time=%d, value=%d, type=0x%02X, number=%d\n",
	    js.time, js.value&0xFFFF, js.type, js.number);
#endif

  }

  close(joydev);

}
