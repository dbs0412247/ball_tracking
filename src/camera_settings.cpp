#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <unistd.h>

v4l2_capability    cap;
/*
struct v4l2_queryctrl     queryctrl;

struct v4l2_querymenu     querymenu;
struct v4l2_capability    cap;

static void enumerate_menu(void)
{
  printf("  Menu items:\n");

  memset(&querymenu, 0, sizeof(querymenu));
  querymenu.id = queryctrl.id;

  for (querymenu.index = queryctrl.minimum;
       querymenu.index <= queryctrl.maximum;
       querymenu.index++) {
    if (0 == ioctl(fd, VIDIOC_QUERYMENU, &querymenu)) {
      printf("  %s\n", querymenu.name);
    }
  }
}*/

int main() {
  
  int fd = open("/dev/video1", O_RDWR);
  if (fd == -1) {
    //printf("Error openning device: %d", errno);   
    printf("Error openning device");    
  }
  ioctl(fd, VIDIOC_QUERYCAP, &cap);

  printf("");

  int status = close(fd);



/*
  memset(&queryctrl, 0, sizeof(queryctrl));

  for (queryctrl.id = V4L2_CID_BASE;
       queryctrl.id < V4L2_CID_LASTP1;
       queryctrl.id++) {
    if (0 == ioctl(fd, VIDIOC_QUERYCTRL, &queryctrl)) {
      if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
        continue;

      printf("Control %s\n", queryctrl.name);

      if (queryctrl.type == V4L2_CTRL_TYPE_MENU)
        enumerate_menu();
    } else {
      if (errno == EINVAL)
        continue;

      perror("VIDIOC_QUERYCTRL");
      exit(EXIT_FAILURE);
    }
  }

  for (queryctrl.id = V4L2_CID_PRIVATE_BASE;;
       queryctrl.id++) {
    if (0 == ioctl(fd, VIDIOC_QUERYCTRL, &queryctrl)) {
      if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
        continue;

      printf("Control %s\n", queryctrl.name);

      if (queryctrl.type == V4L2_CTRL_TYPE_MENU)
        enumerate_menu();
    } else {
      if (errno == EINVAL)
        break;

      perror("VIDIOC_QUERYCTRL");
      exit(EXIT_FAILURE);
    }
  }
*/


} // end main
 

