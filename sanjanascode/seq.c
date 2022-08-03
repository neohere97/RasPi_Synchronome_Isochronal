/*
 *
 *  Adapted by Sam Siewert for use with UVC web cameras and Bt878 frame
 *  grabber NTSC cameras to acquire digital video from a source,
 *  time-stamp each frame acquired, save to a PGM or PPM file.
 *
 *  The original code adapted was open source from V4L2 API and had the
 *  following use and incorporation policy:
 * 
 *  This program can be used and distributed without restrictions.
 *
 *      This program is provided with the V4L2 API
 * see http://linuxtv.org/docs.php for more information
 */

#define _GNU_SOURCE
 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <getopt.h>             /* getopt_long() */

#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <syslog.h>

#include <linux/videodev2.h>

#include <time.h>

#include <pthread.h>
#include <sched.h>
#include <semaphore.h>

#define USEC_PER_MSEC (1000)
#define NANOSEC_PER_SEC (1000000000)
#define NUM_CPU_CORES (1)
#define TRUE (1)
#define FALSE (0)

#define NUM_THREADS (4+1)

int abortTest=FALSE;
int abort_capture=FALSE, abort_diff=FALSE, abort_sel=FALSE, abort_wb=FALSE;
sem_t sem_capture, sem_diff, sem_sel, sem_wb;
struct timeval start_time_val;

pthread_mutex_t ring, ring_wb;
pthread_mutexattr_t rt_safe;

//FILE *fp_sequencer, *fp_capture, *fp_diff, *fp_sel, *fp_wb, *fp_change;

typedef struct
{
    int threadIdx;
    unsigned long long sequencePeriods;
} threadParams_t;

#define CLEAR(x) memset(&(x), 0, sizeof(x))
#define COLOR_CONVERT_RGB (1)
#define HRES 640
#define VRES 480
#define HRES_STR "640"
#define VRES_STR "480"
//#define HRES 320
//#define VRES 240
//#define HRES_STR "320"
//#define VRES_STR "240"

#define MAX_HRES (1920)
#define MAX_VRES (1080)

#define MAX_PIXEL_SIZE (3)
#define PIXEL_SIZE (3)

// Format is used by a number of functions, so made as a file global
static struct v4l2_format fmt;

enum io_method 
{
        IO_METHOD_READ,
        IO_METHOD_MMAP,
        IO_METHOD_USERPTR,
};

struct buffer 
{
        void   *start;
        size_t  length;
};

struct save_frame_t
{
    unsigned char   frame[HRES*VRES*PIXEL_SIZE];
    double diff;
    struct timespec time_stamp;
    char identifier_str[80];
};

struct ring_buffer_t
{
    unsigned int ring_size;

    int tail_idx;
    int head_idx;
    int count;
    struct save_frame_t save_frame[3*10];
};

void *sequencer(void *threadp);
void *frame_capture(void *threadp);
void *frame_diff(void *threadp);
void *frame_sel(void *threadp);
void *write_back(void *threadp);

struct ring_buffer_t ring_buffer, ring_buffer_wb;

unsigned char bigbuffer[(MAX_HRES*MAX_VRES*MAX_PIXEL_SIZE)];
unsigned char threshbuffer1[(MAX_HRES*MAX_VRES*MAX_PIXEL_SIZE)];
unsigned char threshbuffer2[(MAX_HRES*MAX_VRES*MAX_PIXEL_SIZE)];
unsigned char writebuffer[(MAX_HRES*MAX_VRES*MAX_PIXEL_SIZE)];
int size_bytes;

static char            *dev_name;
//static enum io_method   io = IO_METHOD_USERPTR;
//static enum io_method   io = IO_METHOD_READ;
static enum io_method   io = IO_METHOD_MMAP;
static int              fd = -1;
struct buffer          *buffers;
static unsigned int     n_buffers;
static int              out_buf;
static int              force_format=1;
static int              frame_count = (189);

static void errno_exit(const char *s)
{
        fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
        exit(EXIT_FAILURE);
}

static int xioctl(int fh, int request, void *arg)
{
        int r;

        do 
        {
            r = ioctl(fh, request, arg);

        } while (-1 == r && EINTR == errno);

        return r;
}

char ppm_header[]="P6\n#9999999999 sec 9999999999 msec \n"HRES_STR" "VRES_STR"\n255\n";
char ppm_dumpname[]="frames/test0000.ppm";

static void dump_ppm(const void *p, int size, unsigned int tag, struct timespec *time)
{
    int written, i, total, dumpfd;
   
    snprintf(&ppm_dumpname[11], 9, "%04d", tag);
    strncat(&ppm_dumpname[15], ".ppm", 5);
    dumpfd = open(ppm_dumpname, O_WRONLY | O_NONBLOCK | O_CREAT, 00666);

    snprintf(&ppm_header[4], 11, "%010d", (int)time->tv_sec);
    strncat(&ppm_header[14], " sec ", 5);
    snprintf(&ppm_header[19], 11, "%010d", (int)((time->tv_nsec)/1000000));
    strncat(&ppm_header[29], " msec \n"HRES_STR" "VRES_STR"\n255\n", 19);
    written=write(dumpfd, ppm_header, sizeof(ppm_header));

    total=0;

    do
    {
        written=write(dumpfd, p, size);
        total+=written;
    } while(total < size);

    printf("wrote %d bytes\n", total);

    close(dumpfd);
    
}


char pgm_header[]="P5\n#9999999999 sec 9999999999 msec \n"HRES_STR" "VRES_STR"\n255\n";
char pgm_dumpname[]="frames/test0000.pgm";

static void dump_pgm(const void *p, int size, unsigned int tag, struct timespec *time)
{
    int written, i, total, dumpfd;
   
    snprintf(&pgm_dumpname[11], 9, "%04d", tag);
    strncat(&pgm_dumpname[15], ".pgm", 5);
    dumpfd = open(pgm_dumpname, O_WRONLY | O_NONBLOCK | O_CREAT, 00666);

    snprintf(&pgm_header[4], 11, "%010d", (int)time->tv_sec);
    strncat(&pgm_header[14], " sec ", 5);
    snprintf(&pgm_header[19], 11, "%010d", (int)((time->tv_nsec)/1000000));
    strncat(&pgm_header[29], " msec \n"HRES_STR" "VRES_STR"\n255\n", 19);
    written=write(dumpfd, pgm_header, sizeof(pgm_header));

    total=0;

    do
    {
        written=write(dumpfd, p, size);
        total+=written;
    } while(total < size);

    printf("wrote %d bytes\n", total);

    close(dumpfd);
    
}


void yuv2rgb_float(float y, float u, float v, 
                   unsigned char *r, unsigned char *g, unsigned char *b)
{
    float r_temp, g_temp, b_temp;

    // R = 1.164(Y-16) + 1.1596(V-128)
    r_temp = 1.164*(y-16.0) + 1.1596*(v-128.0);  
    *r = r_temp > 255.0 ? 255 : (r_temp < 0.0 ? 0 : (unsigned char)r_temp);

    // G = 1.164(Y-16) - 0.813*(V-128) - 0.391*(U-128)
    g_temp = 1.164*(y-16.0) - 0.813*(v-128.0) - 0.391*(u-128.0);
    *g = g_temp > 255.0 ? 255 : (g_temp < 0.0 ? 0 : (unsigned char)g_temp);

    // B = 1.164*(Y-16) + 2.018*(U-128)
    b_temp = 1.164*(y-16.0) + 2.018*(u-128.0);
    *b = b_temp > 255.0 ? 255 : (b_temp < 0.0 ? 0 : (unsigned char)b_temp);
}


// This is probably the most acceptable conversion from camera YUYV to RGB
//
// Wikipedia has a good discussion on the details of various conversions and cites good references:
// http://en.wikipedia.org/wiki/YUV
//
// Also http://www.fourcc.org/yuv.php
//
// What's not clear without knowing more about the camera in question is how often U & V are sampled compared
// to Y.
//
// E.g. YUV444, which is equivalent to RGB, where both require 3 bytes for each pixel
//      YUV422, which we assume here, where there are 2 bytes for each pixel, with two Y samples for one U & V,
//              or as the name implies, 4Y and 2 UV pairs
//      YUV420, where for every 4 Ys, there is a single UV pair, 1.5 bytes for each pixel or 36 bytes for 24 pixels

void yuv2rgb(int y, int u, int v, unsigned char *r, unsigned char *g, unsigned char *b)
{
   int r1, g1, b1;

   // replaces floating point coefficients
   int c = y-16, d = u - 128, e = v - 128;       

   // Conversion that avoids floating point
   r1 = (298 * c           + 409 * e + 128) >> 8;
   g1 = (298 * c - 100 * d - 208 * e + 128) >> 8;
   b1 = (298 * c + 516 * d           + 128) >> 8;

   // Computed values may need clipping.
   if (r1 > 255) r1 = 255;
   if (g1 > 255) g1 = 255;
   if (b1 > 255) b1 = 255;

   if (r1 < 0) r1 = 0;
   if (g1 < 0) g1 = 0;
   if (b1 < 0) b1 = 0;

   *r = r1 ;
   *g = g1 ;
   *b = b1 ;
}


// always ignore first 8 frames
int framecnt=-30;

static void process_image(const void *p, int size)
{
    int i, newi, newsize=0;
    struct timespec frame_time;
    int y_temp, y2_temp, u_temp, v_temp;
    unsigned char *pptr = (unsigned char *)p;

    size_bytes = size;

    // record when process was called
    clock_gettime(CLOCK_REALTIME, &frame_time);    

    framecnt++;
    printf("frame %d: ", framecnt);

    // This just dumps the frame to a file now, but you could replace with whatever image
    // processing you wish.
    //

    if(fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_GREY)
    {
        printf("Dump graymap as-is size %d\n", size);
        dump_pgm(p, size, framecnt, &frame_time);
    }

    else if(fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV)
    {

#if defined(COLOR_CONVERT_RGB)
       
        // Pixels are YU and YV alternating, so YUYV which is 4 bytes
        // We want RGB, so RGBRGB which is 6 bytes
        //
        for(i=0, newi=0; i<size; i=i+4, newi=newi+6)
        {
            y_temp=(int)pptr[i]; u_temp=(int)pptr[i+1]; y2_temp=(int)pptr[i+2]; v_temp=(int)pptr[i+3];
            yuv2rgb(y_temp, u_temp, v_temp, &bigbuffer[newi], &bigbuffer[newi+1], &bigbuffer[newi+2]);
            yuv2rgb(y2_temp, u_temp, v_temp, &bigbuffer[newi+3], &bigbuffer[newi+4], &bigbuffer[newi+5]);
        }

        if(framecnt > -1) 
        {
            pthread_mutex_lock(&ring);
            memcpy((void *)&(ring_buffer.save_frame[ring_buffer.tail_idx].frame[0]), bigbuffer, ((size*6)/4));
            ring_buffer.tail_idx = (ring_buffer.tail_idx + 1) % ring_buffer.ring_size;
            ring_buffer.count = ring_buffer.count + 1;
            printf("Ring Count: %d\n", ring_buffer.count);
            pthread_mutex_unlock(&ring);
            
            //dump_ppm(bigbuffer, ((size*6)/4), framecnt, &frame_time);
            printf("Dump YUYV converted to RGB size %d\n", size);
        }
#else
       
        // Pixels are YU and YV alternating, so YUYV which is 4 bytes
        // We want Y, so YY which is 2 bytes
        //
        for(i=0, newi=0; i<size; i=i+4, newi=newi+2)
        {
            // Y1=first byte and Y2=third byte
            bigbuffer[newi]=pptr[i];
            bigbuffer[newi+1]=pptr[i+2];
        }

        //if(framecnt > -1)
        //{
        //    dump_pgm(bigbuffer, (size/2), framecnt, &frame_time);
        //    printf("Dump YUYV converted to YY size %d\n", size);
        //}
#endif

    }

    //else if(fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_RGB24)
    //{
    //    printf("Dump RGB as-is size %d\n", size);
    //    dump_ppm(p, size, framecnt, &frame_time);
    //}
    else
    {
        printf("ERROR - unknown dump format\n");
    }

    fflush(stderr);
    //fprintf(stderr, ".");
    fflush(stdout);
}


static int read_frame(void)
{
    struct v4l2_buffer buf;
    unsigned int i;

    switch (io)
    {

        case IO_METHOD_READ:
            if (-1 == read(fd, buffers[0].start, buffers[0].length))
            {
                switch (errno)
                {

                    case EAGAIN:
                        return 0;

                    case EIO:
                        /* Could ignore EIO, see spec. */

                        /* fall through */

                    default:
                        errno_exit("read");
                }
            }

            process_image(buffers[0].start, buffers[0].length);
            break;

        case IO_METHOD_MMAP:
            CLEAR(buf);

            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;

            if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf))
            {
                switch (errno)
                {
                    case EAGAIN:
                        return 0;

                    case EIO:
                        /* Could ignore EIO, but drivers should only set for serious errors, although some set for
                           non-fatal errors too.
                         */
                        return 0;


                    default:
                        printf("mmap failure\n");
                        errno_exit("VIDIOC_DQBUF");
                }
            }

            assert(buf.index < n_buffers);

            process_image(buffers[buf.index].start, buf.bytesused);

            if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                    errno_exit("VIDIOC_QBUF");
            break;

        case IO_METHOD_USERPTR:
            CLEAR(buf);

            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_USERPTR;

            if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf))
            {
                switch (errno)
                {
                    case EAGAIN:
                        return 0;

                    case EIO:
                        /* Could ignore EIO, see spec. */

                        /* fall through */

                    default:
                        errno_exit("VIDIOC_DQBUF");
                }
            }

            for (i = 0; i < n_buffers; ++i)
                    if (buf.m.userptr == (unsigned long)buffers[i].start
                        && buf.length == buffers[i].length)
                            break;

            assert(i < n_buffers);

            process_image((void *)buf.m.userptr, buf.bytesused);

            if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                    errno_exit("VIDIOC_QBUF");
            break;
    }

    //printf("R");
    return 1;
}


static void mainloop(void)
{
    unsigned int count;
    struct timespec read_delay;
    struct timespec time_error;

    // Replace this with a sequencer DELAY
    //
    // 250 million nsec is a 250 msec delay, for 4 fps
    // 1 sec for 1 fps
    //
    read_delay.tv_sec=0;
    read_delay.tv_nsec=10000000;

    count = frame_count;

    //while (count > 0)
    //{
        for (;;)
        {
            fd_set fds;
            struct timeval tv;
            int r;

            FD_ZERO(&fds);
            FD_SET(fd, &fds);

            /* Timeout. */
            tv.tv_sec = 2;
            tv.tv_usec = 0;

            r = select(fd + 1, &fds, NULL, NULL, &tv);

            if (-1 == r)
            {
                if (EINTR == errno)
                    continue;
                errno_exit("select");
            }

            if (0 == r)
            {
                fprintf(stderr, "select timeout\n");
                exit(EXIT_FAILURE);
            }

            if (read_frame())
            {
                if(nanosleep(&read_delay, &time_error) != 0)
                    perror("nanosleep");
                else
                    if(framecnt>-1)
                    {
                        /*pthread_mutex_lock(&ring);
                        memcpy((void *)&(ring_buffer.save_frame[ring_buffer.tail_idx].frame[0]), bigbuffer, ((size_bytes*6)/4));
                        ring_buffer.tail_idx = (ring_buffer.tail_idx + 1) % ring_buffer.ring_size;
                        ring_buffer.count = ring_buffer.count + 1;
                        pthread_mutex_unlock(&ring);*/
                        //process_image((void *)&(ring_buffer.save_frame[ring_buffer.head_idx].frame[0]), HRES*VRES*PIXEL_SIZE);
                        printf("time_error.tv_sec=%ld, time_error.tv_nsec=%ld\n", time_error.tv_sec, time_error.tv_nsec);
                    }

                //count--;
                break;
            }

            /* EAGAIN - continue select loop unless count done. */
            //if(count <= 0) break;
        }

        //if(count <= 0) break;
    //}
}

static void stop_capturing(void)
{
        enum v4l2_buf_type type;

        switch (io) {
        case IO_METHOD_READ:
                /* Nothing to do. */
                break;

        case IO_METHOD_MMAP:
        case IO_METHOD_USERPTR:
                type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type))
                        errno_exit("VIDIOC_STREAMOFF");
                break;
        }
}

static void start_capturing(void)
{
        unsigned int i;
        enum v4l2_buf_type type;

        switch (io) 
        {

        case IO_METHOD_READ:
                /* Nothing to do. */
                break;

        case IO_METHOD_MMAP:
                for (i = 0; i < n_buffers; ++i) 
                {
                        printf("allocated buffer %d\n", i);
                        struct v4l2_buffer buf;

                        CLEAR(buf);
                        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                        buf.memory = V4L2_MEMORY_MMAP;
                        buf.index = i;

                        if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                                errno_exit("VIDIOC_QBUF");
                }
                type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
                        errno_exit("VIDIOC_STREAMON");
                break;

        case IO_METHOD_USERPTR:
                for (i = 0; i < n_buffers; ++i) {
                        struct v4l2_buffer buf;

                        CLEAR(buf);
                        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                        buf.memory = V4L2_MEMORY_USERPTR;
                        buf.index = i;
                        buf.m.userptr = (unsigned long)buffers[i].start;
                        buf.length = buffers[i].length;

                        if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                                errno_exit("VIDIOC_QBUF");
                }
                type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
                        errno_exit("VIDIOC_STREAMON");
                break;
        }
}

static void uninit_device(void)
{
        unsigned int i;

        switch (io) {
        case IO_METHOD_READ:
                free(buffers[0].start);
                break;

        case IO_METHOD_MMAP:
                for (i = 0; i < n_buffers; ++i)
                        if (-1 == munmap(buffers[i].start, buffers[i].length))
                                errno_exit("munmap");
                break;

        case IO_METHOD_USERPTR:
                for (i = 0; i < n_buffers; ++i)
                        free(buffers[i].start);
                break;
        }

        free(buffers);
}

static void init_read(unsigned int buffer_size)
{
        buffers = calloc(1, sizeof(*buffers));

        if (!buffers) 
        {
                fprintf(stderr, "Out of memory\n");
                exit(EXIT_FAILURE);
        }

        buffers[0].length = buffer_size;
        buffers[0].start = malloc(buffer_size);

        if (!buffers[0].start) 
        {
                fprintf(stderr, "Out of memory\n");
                exit(EXIT_FAILURE);
        }
}

static void init_mmap(void)
{
        struct v4l2_requestbuffers req;

        CLEAR(req);

        req.count = 6;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;

        if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) 
        {
                if (EINVAL == errno) 
                {
                        fprintf(stderr, "%s does not support "
                                 "memory mapping\n", dev_name);
                        exit(EXIT_FAILURE);
                } else 
                {
                        errno_exit("VIDIOC_REQBUFS");
                }
        }

        if (req.count < 2) 
        {
                fprintf(stderr, "Insufficient buffer memory on %s\n", dev_name);
                exit(EXIT_FAILURE);
        }

        buffers = calloc(req.count, sizeof(*buffers));

        if (!buffers) 
        {
                fprintf(stderr, "Out of memory\n");
                exit(EXIT_FAILURE);
        }

        for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
                struct v4l2_buffer buf;

                CLEAR(buf);

                buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory      = V4L2_MEMORY_MMAP;
                buf.index       = n_buffers;

                if (-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
                        errno_exit("VIDIOC_QUERYBUF");

                buffers[n_buffers].length = buf.length;
                buffers[n_buffers].start =
                        mmap(NULL /* start anywhere */,
                              buf.length,
                              PROT_READ | PROT_WRITE /* required */,
                              MAP_SHARED /* recommended */,
                              fd, buf.m.offset);

                if (MAP_FAILED == buffers[n_buffers].start)
                        errno_exit("mmap");
        }
}

static void init_userp(unsigned int buffer_size)
{
        struct v4l2_requestbuffers req;

        CLEAR(req);

        req.count  = 4;
        req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_USERPTR;

        if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
                if (EINVAL == errno) {
                        fprintf(stderr, "%s does not support "
                                 "user pointer i/o\n", dev_name);
                        exit(EXIT_FAILURE);
                } else {
                        errno_exit("VIDIOC_REQBUFS");
                }
        }

        buffers = calloc(4, sizeof(*buffers));

        if (!buffers) {
                fprintf(stderr, "Out of memory\n");
                exit(EXIT_FAILURE);
        }

        for (n_buffers = 0; n_buffers < 4; ++n_buffers) {
                buffers[n_buffers].length = buffer_size;
                buffers[n_buffers].start = malloc(buffer_size);

                if (!buffers[n_buffers].start) {
                        fprintf(stderr, "Out of memory\n");
                        exit(EXIT_FAILURE);
                }
        }
}

static void init_device(void)
{
    struct v4l2_capability cap;
    struct v4l2_cropcap cropcap;
    struct v4l2_crop crop;
    unsigned int min;

    if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap))
    {
        if (EINVAL == errno) {
            fprintf(stderr, "%s is no V4L2 device\n",
                     dev_name);
            exit(EXIT_FAILURE);
        }
        else
        {
                errno_exit("VIDIOC_QUERYCAP");
        }
    }

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
    {
        fprintf(stderr, "%s is no video capture device\n",
                 dev_name);
        exit(EXIT_FAILURE);
    }

    switch (io)
    {
        case IO_METHOD_READ:
            if (!(cap.capabilities & V4L2_CAP_READWRITE))
            {
                fprintf(stderr, "%s does not support read i/o\n",
                         dev_name);
                exit(EXIT_FAILURE);
            }
            break;

        case IO_METHOD_MMAP:
        case IO_METHOD_USERPTR:
            if (!(cap.capabilities & V4L2_CAP_STREAMING))
            {
                fprintf(stderr, "%s does not support streaming i/o\n",
                         dev_name);
                exit(EXIT_FAILURE);
            }
            break;
    }


    /* Select video input, video standard and tune here. */


    CLEAR(cropcap);

    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (0 == xioctl(fd, VIDIOC_CROPCAP, &cropcap))
    {
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        crop.c = cropcap.defrect; /* reset to default */

        if (-1 == xioctl(fd, VIDIOC_S_CROP, &crop))
        {
            switch (errno)
            {
                case EINVAL:
                    /* Cropping not supported. */
                    break;
                default:
                    /* Errors ignored. */
                        break;
            }
        }

    }
    else
    {
        /* Errors ignored. */
    }


    CLEAR(fmt);

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (force_format)
    {
        printf("FORCING FORMAT\n");
        fmt.fmt.pix.width       = HRES;
        fmt.fmt.pix.height      = VRES;

        // Specify the Pixel Coding Formate here

        // This one works for Logitech C200
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;

        //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
        //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_VYUY;

        // Would be nice if camera supported
        //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;
        //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;

        //fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;
        fmt.fmt.pix.field       = V4L2_FIELD_NONE;

        if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
                errno_exit("VIDIOC_S_FMT");

        /* Note VIDIOC_S_FMT may change width and height. */
    }
    else
    {
        printf("ASSUMING FORMAT\n");
        /* Preserve original settings as set by v4l2-ctl for example */
        if (-1 == xioctl(fd, VIDIOC_G_FMT, &fmt))
                    errno_exit("VIDIOC_G_FMT");
    }

    /* Buggy driver paranoia. */
    min = fmt.fmt.pix.width * 2;
    if (fmt.fmt.pix.bytesperline < min)
            fmt.fmt.pix.bytesperline = min;
    min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
    if (fmt.fmt.pix.sizeimage < min)
            fmt.fmt.pix.sizeimage = min;

    switch (io)
    {
        case IO_METHOD_READ:
            init_read(fmt.fmt.pix.sizeimage);
            break;

        case IO_METHOD_MMAP:
            init_mmap();
            break;

        case IO_METHOD_USERPTR:
            init_userp(fmt.fmt.pix.sizeimage);
            break;
    }
}


static void close_device(void)
{
        if (-1 == close(fd))
                errno_exit("close");

        fd = -1;
}

static void open_device(void)
{
        struct stat st;

        if (-1 == stat(dev_name, &st)) {
                fprintf(stderr, "Cannot identify '%s': %d, %s\n",
                         dev_name, errno, strerror(errno));
                exit(EXIT_FAILURE);
        }

        if (!S_ISCHR(st.st_mode)) {
                fprintf(stderr, "%s is no device\n", dev_name);
                exit(EXIT_FAILURE);
        }

        fd = open(dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);

        if (-1 == fd) {
                fprintf(stderr, "Cannot open '%s': %d, %s\n",
                         dev_name, errno, strerror(errno));
                exit(EXIT_FAILURE);
        }
}

static void usage(FILE *fp, int argc, char **argv)
{
        fprintf(fp,
                 "Usage: %s [options]\n\n"
                 "Version 1.3\n"
                 "Options:\n"
                 "-d | --device name   Video device name [%s]\n"
                 "-h | --help          Print this message\n"
                 "-m | --mmap          Use memory mapped buffers [default]\n"
                 "-r | --read          Use read() calls\n"
                 "-u | --userp         Use application allocated buffers\n"
                 "-o | --output        Outputs stream to stdout\n"
                 "-f | --format        Force format to 640x480 GREY\n"
                 "-c | --count         Number of frames to grab [%i]\n"
                 "",
                 argv[0], dev_name, frame_count);
}

static const char short_options[] = "d:hmruofc:";

static const struct option
long_options[] = {
        { "device", required_argument, NULL, 'd' },
        { "help",   no_argument,       NULL, 'h' },
        { "mmap",   no_argument,       NULL, 'm' },
        { "read",   no_argument,       NULL, 'r' },
        { "userp",  no_argument,       NULL, 'u' },
        { "output", no_argument,       NULL, 'o' },
        { "format", no_argument,       NULL, 'f' },
        { "count",  required_argument, NULL, 'c' },
        { 0, 0, 0, 0 }
};

void *frame_capture(void* threadp)
{    
    struct timeval current_time_val;
    double current_time;
    unsigned long long S1Cnt=0;
    threadParams_t *threadParams = (threadParams_t *)threadp;
//    struct timespec start_time, stop_time;
//    double time_to_complete;

    gettimeofday(&current_time_val, (struct timezone *)0);
    syslog(LOG_CRIT, "Frame Capture thread @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
    printf("Frame Capture thread @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);

    while(!abort_capture)
    {
        //clock_gettime(CLOCK_MONOTONIC_RAW, &start_time);
        sem_wait(&sem_capture);
        mainloop();
        S1Cnt++;
        
//        clock_gettime(CLOCK_MONOTONIC_RAW, &stop_time);
//        time_to_complete = (double)(stop_time.tv_sec - start_time.tv_sec)*1000.0 + (double)((stop_time.tv_nsec - start_time.tv_nsec)/NANOSEC_PER_SEC)*1000.0;
//        fprintf(fp_capture, "%lf\n", time_to_complete);

        gettimeofday(&current_time_val, (struct timezone *)0);
        syslog(LOG_CRIT, "Frame Capture release %llu @ sec=%d, msec=%d\n", S1Cnt, (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
    }

    pthread_exit((void *)0);
}

void *frame_diff(void* threadp)
{
    int i, j;
    double num_squared_error = 0, denom_squared = 0;
    struct timeval current_time_val;
    double current_time;
    unsigned long long S2Cnt=0;
    threadParams_t *threadParams = (threadParams_t *)threadp;
    double diff_pixels=0;
//    struct timespec start_time, stop_time;
//    double time_to_complete;

    gettimeofday(&current_time_val, (struct timezone *)0);
    syslog(LOG_CRIT, "Time-stamp with Frame Differencing thread @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
    printf("Time-stamp with Frame Differencing thread @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);

    while(!abort_diff)
    {
        //clock_gettime(CLOCK_MONOTONIC_RAW, &start_time);
        sem_wait(&sem_diff);

        if(framecnt>-1)
        {   
            pthread_mutex_lock(&ring);
            for(i=ring_buffer.head_idx; i<ring_buffer.head_idx+ring_buffer.count; i++)
            {
                printf("Ring Count Count: %d\n", ring_buffer.count);
                memcpy(threshbuffer1, &(ring_buffer.save_frame[(i%ring_buffer.ring_size)].frame[0]), (size_bytes*6)/4);
                memcpy(threshbuffer2, &(ring_buffer.save_frame[((i+1)%ring_buffer.ring_size)].frame[0]), (size_bytes*6)/4);

                for(j = 0; j<(size_bytes*6)/4; j++)
                {
                    if(threshbuffer2[j]!=threshbuffer1[j])
                      diff_pixels++;
                    //num_squared_error += (threshbuffer2[j] - threshbuffer1[j])*(threshbuffer2[j] - threshbuffer1[j]);
                    //denom_squared += threshbuffer1[i];
                }
                //denom_squared = denom_squared*denom_squared;
                ring_buffer.save_frame[((i+1)%ring_buffer.ring_size)].diff = diff_pixels;
                printf("Diff: %lf\n", ring_buffer.save_frame[((i+1)%ring_buffer.ring_size)].diff);
                diff_pixels = 0;
            }
            pthread_mutex_unlock(&ring);
        }
        S2Cnt++;

//        clock_gettime(CLOCK_MONOTONIC_RAW, &stop_time);
//        time_to_complete = (double)(stop_time.tv_sec - start_time.tv_sec)*1000.0 + (double)((stop_time.tv_nsec - start_time.tv_nsec)/NANOSEC_PER_SEC)*1000.0;
//        fprintf(fp_diff, "%lf\n", time_to_complete);

        gettimeofday(&current_time_val, (struct timezone *)0);
        syslog(LOG_CRIT, "Time-stamp with Image Analysis release %llu @ sec=%d, msec=%d\n", S2Cnt, (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
    }

    pthread_exit((void *)0);    
}

void *frame_sel(void* threadp)
{
    
    int i, argmax=0;
    double max = 0;

    struct timeval current_time_val;
    double current_time;
    unsigned long long S2Cnt=0;
    threadParams_t *threadParams = (threadParams_t *)threadp;
//    struct timespec start_time, stop_time;
//    double time_to_complete;

    gettimeofday(&current_time_val, (struct timezone *)0);
    syslog(LOG_CRIT, "Time-stamp with Frame Selection thread @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
    printf("Time-stamp with Frame Selection thread @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);

    while(!abort_sel)
    {
        //clock_gettime(CLOCK_MONOTONIC_RAW, &start_time);
        sem_wait(&sem_sel);

        if(framecnt>-1 && ring_buffer.count>5)
        {
            pthread_mutex_lock(&ring);
            printf("Ring Count: %d\n", ring_buffer.count);
            for(i=ring_buffer.head_idx; i<ring_buffer.head_idx+ring_buffer.count; i++)
            {
                //fprintf(fp_change, "%lf\n", ring_buffer.save_frame[(i%ring_buffer.ring_size)].diff);
                if(max<ring_buffer.save_frame[(i%10)].diff)
                {
                    max=ring_buffer.save_frame[(i%10)].diff;
                    argmax = (i%10);
                }
                printf("%d\n", argmax);
            }
            printf("argmax: %d\n", argmax);
            

            pthread_mutex_lock(&ring_wb);
            memcpy((void* )&ring_buffer_wb.save_frame[ring_buffer_wb.tail_idx].frame[0], &ring_buffer.save_frame[(argmax+2)%10].frame[0], (size_bytes*6)/4);
            ring_buffer_wb.tail_idx = (ring_buffer_wb.tail_idx + 1)%ring_buffer_wb.ring_size;
            ring_buffer_wb.count++; 
            ring_buffer.count = 0;
            ring_buffer.tail_idx = 0;
            ring_buffer.head_idx = 0;
            pthread_mutex_unlock(&ring_wb);
            pthread_mutex_unlock(&ring);
        }
        S2Cnt++;

//        clock_gettime(CLOCK_MONOTONIC_RAW, &stop_time);
//        time_to_complete = (double)(stop_time.tv_sec - start_time.tv_sec)*1000.0 + (double)((stop_time.tv_nsec - start_time.tv_nsec)/NANOSEC_PER_SEC)*1000.0;
//        fprintf(fp_sel, "%lf\n", time_to_complete);

        gettimeofday(&current_time_val, (struct timezone *)0);
        syslog(LOG_CRIT, "Time-stamp with Image selection release %llu @ sec=%d, msec=%d\n", S2Cnt, (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
    }

    pthread_exit((void *)0);
}

void *write_back(void*threadp)
{

    int i=0;
    struct timeval current_time_val;
    double current_time;
    struct timespec fnow;
    unsigned long long S2Cnt=0;
    threadParams_t *threadParams = (threadParams_t *)threadp;
//    struct timespec start_time, stop_time;
//    double time_to_complete;
//    
    clock_gettime(CLOCK_MONOTONIC_RAW,&fnow);

    gettimeofday(&current_time_val, (struct timezone *)0);
    syslog(LOG_CRIT, "Time-stamp with write-back thread @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
    printf("Time-stamp with write-back thread @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);

    while(!abort_wb)
    {
        //clock_gettime(CLOCK_MONOTONIC_RAW, &start_time);
        sem_wait(&sem_wb);

        while(ring_buffer_wb.count)
        {
            pthread_mutex_lock(&ring_wb);
            memcpy((void* )&writebuffer, (void* )&ring_buffer_wb.save_frame[ring_buffer_wb.head_idx].frame[0], (size_bytes*6)/4);
            
            for(i=0; i<(size_bytes*6)/4; i++)
            {
                writebuffer[i] = 255 - writebuffer[i];
            }

            dump_ppm((void* )&writebuffer, (size_bytes*6)/4, framecnt,&fnow);
            ring_buffer_wb.head_idx = (ring_buffer_wb.head_idx + 1)%ring_buffer_wb.ring_size;
            ring_buffer_wb.count--;
            pthread_mutex_unlock(&ring_wb);
        }

        S2Cnt++;

//        clock_gettime(CLOCK_MONOTONIC_RAW, &stop_time);
//        time_to_complete = (double)(stop_time.tv_sec - start_time.tv_sec)*1000.0 + (double)((stop_time.tv_nsec - start_time.tv_nsec)/NANOSEC_PER_SEC)*1000.0;
//        fprintf(fp_wb, "%lf\n", time_to_complete);

        gettimeofday(&current_time_val, (struct timezone *)0);
        syslog(LOG_CRIT, "Time-stamp with write-back release %llu @ sec=%d, msec=%d\n", S2Cnt, (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
    }

    pthread_exit((void *)0);

}

void *sequencer(void* threadp)
{
    struct timeval current_time_val;
    struct timespec delay_time = {0,20000000}; // delay for 20 msec, 50 Hz
    struct timespec remaining_time;
    double current_time;
    double residual;
    int rc, delay_cnt=0;
    unsigned long long seqCnt=0;
    threadParams_t *threadParams = (threadParams_t *)threadp;
//    struct timespec start_time, stop_time;
//    double time_to_complete;

    gettimeofday(&current_time_val, (struct timezone *)0);
    syslog(LOG_CRIT, "Sequencer thread @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
    printf("Sequencer thread @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);

    do
    {
        //clock_gettime(CLOCK_MONOTONIC_RAW, &start_time);
        delay_cnt=0; residual=0.0;

        //gettimeofday(&current_time_val, (struct timezone *)0);
        //syslog(LOG_CRIT, "Sequencer thread prior to delay @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
        do
        {
            rc=nanosleep(&delay_time, &remaining_time);

            if(rc == EINTR)
            { 
                residual = remaining_time.tv_sec + ((double)remaining_time.tv_nsec / (double)NANOSEC_PER_SEC);

                if(residual > 0.0) printf("residual=%lf, sec=%d, nsec=%d\n", residual, (int)remaining_time.tv_sec, (int)remaining_time.tv_nsec);
 
                delay_cnt++;
            }
            else if(rc < 0)
            {
                perror("Sequencer nanosleep");
                exit(-1);
            }
           
        } while((residual > 0.0) && (delay_cnt < 100));

        seqCnt++;
        gettimeofday(&current_time_val, (struct timezone *)0);
        syslog(LOG_CRIT, "Sequencer cycle %llu @ sec=%d, msec=%d\n", seqCnt, (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);


        if(delay_cnt > 1) printf("Sequencer looping delay %d\n", delay_cnt);


        // Release each service at a sub-rate of the generic sequencer rate

        // Servcie_1 = RT_MAX-1	@ 3 Hz
        if((seqCnt % 5) == 0) sem_post(&sem_capture);

        // Service_2 = RT_MAX-2	@ 1 Hz
        if((seqCnt % 25) == 0) sem_post(&sem_diff);

        // Service_3 = RT_MAX-3	@ 0.5 Hz
        if((seqCnt % 50) == 0) sem_post(&sem_sel);

        // Service_4 = RT_MAX-2	@ 1 Hz
        if((seqCnt % 50) == 0) sem_post(&sem_wb);

        //gettimeofday(&current_time_val, (struct timezone *)0);
        //syslog(LOG_CRIT, "Sequencer release all sub-services @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
        
//        clock_gettime(CLOCK_MONOTONIC_RAW, &stop_time);
//        time_to_complete = (double)(stop_time.tv_sec - start_time.tv_sec)*1000.0 + (double)((stop_time.tv_nsec - start_time.tv_nsec)/NANOSEC_PER_SEC)*1000.0;
//        fprintf(fp_sequencer, "%lf\n", time_to_complete);

    } while(!abortTest && (seqCnt < threadParams->sequencePeriods));

    sem_post(&sem_capture); sem_post(&sem_diff); sem_post(&sem_sel);
    sem_post(&sem_wb);
    abort_capture=TRUE; abort_diff=TRUE; abort_sel=TRUE;
    abort_wb=TRUE;

    pthread_exit((void *)0);
}

int main(int argc, char **argv)
{

    pthread_mutex_init(&ring, NULL);
    pthread_mutex_init(&ring_wb, NULL);

    ring_buffer.tail_idx = 0; 
    ring_buffer.head_idx = 0; 
    ring_buffer.count = 0; 
    ring_buffer.ring_size = 30;
    ring_buffer_wb.tail_idx = 0; 
    ring_buffer_wb.head_idx = 0; 
    ring_buffer_wb.count = 0; 
    ring_buffer_wb.ring_size = 30;
    
//    fp_sequencer = fopen("sequencer_timing.txt", "w+");
//    fp_capture = fopen("capture_timing.txt", "w+");
//    fp_diff = fopen("diff_timing.txt", "w+");
//    fp_sel = fopen("sel_timing.txt", "w+");
//    fp_wb = fopen("wb_timing.txt", "w+");
//    fp_change = fopen("change.txt", "w+");
    
    if(argc > 1)
        dev_name = argv[1];
    else
        dev_name = "/dev/video0";

    for (;;)
    {
        int idx;
        int c;

        c = getopt_long(argc, argv,
                    short_options, long_options, &idx);

        if (-1 == c)
            break;

        switch (c)
        {
            case 0: /* getopt_long() flag */
                break;

            case 'd':
                dev_name = optarg;
                break;

            case 'h':
                usage(stdout, argc, argv);
                exit(EXIT_SUCCESS);

            case 'm':
                io = IO_METHOD_MMAP;
                break;

            case 'r':
                io = IO_METHOD_READ;
                break;

            case 'u':
                io = IO_METHOD_USERPTR;
                break;

            case 'o':
                out_buf++;
                break;

            case 'f':
                force_format++;
                break;

            case 'c':
                errno = 0;
                frame_count = strtol(optarg, NULL, 0);
                if (errno)
                        errno_exit(optarg);
                break;

            default:
                usage(stderr, argc, argv);
                exit(EXIT_FAILURE);
        }
    }

    // initialization of V4L2
    open_device();
    init_device();
    start_capturing();

    struct timeval current_time_val;
    int i, rc, scope;
    cpu_set_t threadcpu_seq, threadcpu_capture, threadcpu_diff, threadcpu_sel, threadcpu_wb;
    pthread_t threads[NUM_THREADS];
    threadParams_t threadParams[NUM_THREADS];
    pthread_attr_t rt_sched_attr[NUM_THREADS];
    int rt_max_prio, rt_min_prio;
    struct sched_param rt_param[NUM_THREADS];
    struct sched_param main_param;
    pthread_attr_t main_attr;
    pid_t mainpid;
    cpu_set_t allcpuset;

    printf("Starting Sequencer Demo\n");
    gettimeofday(&start_time_val, (struct timezone *)0);
    gettimeofday(&current_time_val, (struct timezone *)0);
    syslog(LOG_CRIT, "Sequencer @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);

    printf("System has %d processors configured and %d available.\n", get_nprocs_conf(), get_nprocs());

    CPU_ZERO(&allcpuset);

    for(i=0; i < NUM_CPU_CORES; i++)
        CPU_SET(i, &allcpuset);

    printf("Using CPUS=%d from total available.\n", CPU_COUNT(&allcpuset));


    // initialize the sequencer semaphores
    //
    if (sem_init (&sem_capture, 0, 0)) { printf ("Failed to initialize S1 semaphore\n"); exit (-1); }
    if (sem_init (&sem_diff, 0, 0)) { printf ("Failed to initialize S2 semaphore\n"); exit (-1); }
    if (sem_init (&sem_sel, 0, 0)) { printf ("Failed to initialize S3 semaphore\n"); exit (-1); }
    if (sem_init (&sem_wb, 0, 0)) { printf ("Failed to initialize S4 semaphore\n"); exit (-1); }

    mainpid=getpid();

    rt_max_prio = sched_get_priority_max(SCHED_FIFO);
    rt_min_prio = sched_get_priority_min(SCHED_FIFO);

    rc=sched_getparam(mainpid, &main_param);
    main_param.sched_priority=rt_max_prio;
    rc=sched_setscheduler(getpid(), SCHED_FIFO, &main_param);
    if(rc < 0) perror("main_param");
    //print_scheduler();


    pthread_attr_getscope(&main_attr, &scope);

    if(scope == PTHREAD_SCOPE_SYSTEM)
      printf("PTHREAD SCOPE SYSTEM\n");
    else if (scope == PTHREAD_SCOPE_PROCESS)
      printf("PTHREAD SCOPE PROCESS\n");
    else
      printf("PTHREAD SCOPE UNKNOWN\n");

    printf("rt_max_prio=%d\n", rt_max_prio);
    printf("rt_min_prio=%d\n", rt_min_prio);

    /*for(i=0; i < NUM_THREADS; i++)
    {

      CPU_ZERO(&threadcpu);
      CPU_SET(3, &threadcpu);

      rc=pthread_attr_init(&rt_sched_attr[i]);
      rc=pthread_attr_setinheritsched(&rt_sched_attr[i], PTHREAD_EXPLICIT_SCHED);
      rc=pthread_attr_setschedpolicy(&rt_sched_attr[i], SCHED_FIFO);
      //rc=pthread_attr_setaffinity_np(&rt_sched_attr[i], sizeof(cpu_set_t), &threadcpu);

      rt_param[i].sched_priority=rt_max_prio-i;
      pthread_attr_setschedparam(&rt_sched_attr[i], &rt_param[i]);

      threadParams[i].threadIdx=i;
    }
   
    printf("Service threads will run on %d CPU cores\n", CPU_COUNT(&threadcpu));*/

    // Create Service threads which will block awaiting release for:
    //

    // Servcie_1 = RT_MAX-1	@ 3 Hz
    //
    rt_param[1].sched_priority=rt_max_prio-1;
    CPU_ZERO(&threadcpu_capture);
    CPU_SET(3, &threadcpu_capture);

    rc=pthread_attr_init(&rt_sched_attr[1]);
    rc=pthread_attr_setinheritsched(&rt_sched_attr[1], PTHREAD_EXPLICIT_SCHED);
    rc=pthread_attr_setschedpolicy(&rt_sched_attr[1], SCHED_FIFO);
    rc=pthread_attr_setaffinity_np(&rt_sched_attr[1], sizeof(cpu_set_t), &threadcpu_capture);

    threadParams[1].threadIdx=1;
    pthread_attr_setschedparam(&rt_sched_attr[1], &rt_param[1]);
    rc=pthread_create(&threads[1],               // pointer to thread descriptor
                      &rt_sched_attr[1],         // use specific attributes
                      //(void *)0,               // default attributes
                      frame_capture,                 // thread function entry point
                      (void *)&(threadParams[1]) // parameters to pass in
                     );
    if(rc < 0)
        perror("pthread_create for service 1");
    else
        printf("pthread_create successful for service 1\n");


    // Service_2 = RT_MAX-2	@ 1 Hz
    //
    rt_param[2].sched_priority=rt_max_prio-2;
    CPU_ZERO(&threadcpu_diff);
    CPU_SET(2, &threadcpu_diff);

    rc=pthread_attr_init(&rt_sched_attr[2]);
    rc=pthread_attr_setinheritsched(&rt_sched_attr[2], PTHREAD_EXPLICIT_SCHED);
    rc=pthread_attr_setschedpolicy(&rt_sched_attr[2], SCHED_FIFO);
    rc=pthread_attr_setaffinity_np(&rt_sched_attr[2], sizeof(cpu_set_t), &threadcpu_diff);

    threadParams[2].threadIdx=2;
    pthread_attr_setschedparam(&rt_sched_attr[2], &rt_param[2]);
    rc=pthread_create(&threads[2], &rt_sched_attr[2], frame_diff, (void *)&(threadParams[2]));
    if(rc < 0)
        perror("pthread_create for service 2");
    else
        printf("pthread_create successful for service 2\n");


    // Service_3 = RT_MAX-3	@ 0.5 Hz
    //
    rt_param[3].sched_priority=rt_max_prio-3;
    CPU_ZERO(&threadcpu_sel);
    CPU_SET(2, &threadcpu_sel);

    rc=pthread_attr_init(&rt_sched_attr[3]);
    rc=pthread_attr_setinheritsched(&rt_sched_attr[3], PTHREAD_EXPLICIT_SCHED);
    rc=pthread_attr_setschedpolicy(&rt_sched_attr[3], SCHED_FIFO);
    rc=pthread_attr_setaffinity_np(&rt_sched_attr[3], sizeof(cpu_set_t), &threadcpu_sel);

    threadParams[3].threadIdx=3;
    pthread_attr_setschedparam(&rt_sched_attr[3], &rt_param[3]);
    rc=pthread_create(&threads[3], &rt_sched_attr[3], frame_sel, (void *)&(threadParams[3]));
    if(rc < 0)
        perror("pthread_create for service 3");
    else
        printf("pthread_create successful for service 3\n");


    // Service_4 = RT_MAX-2	@ 1 Hz
    //
    rt_param[4].sched_priority=rt_max_prio-4;
    CPU_ZERO(&threadcpu_wb);
    CPU_SET(1, &threadcpu_wb);

    rc=pthread_attr_init(&rt_sched_attr[4]);
    rc=pthread_attr_setinheritsched(&rt_sched_attr[4], PTHREAD_EXPLICIT_SCHED);
    rc=pthread_attr_setschedpolicy(&rt_sched_attr[4], SCHED_FIFO);
    rc=pthread_attr_setaffinity_np(&rt_sched_attr[4], sizeof(cpu_set_t), &threadcpu_wb);

    threadParams[4].threadIdx=4;
    pthread_attr_setschedparam(&rt_sched_attr[4], &rt_param[4]);
    rc=pthread_create(&threads[4], &rt_sched_attr[4], write_back, (void *)&(threadParams[4]));
    if(rc < 0)
        perror("pthread_create for service 4");
    else
        printf("pthread_create successful for service 4\n");


    // Wait for service threads to initialize and await relese by sequencer.
    //
    // Note that the sleep is not necessary of RT service threads are created wtih 
    // correct POSIX SCHED_FIFO priorities compared to non-RT priority of this main
    // program.
    //
    // usleep(1000000);
 
    // Create Sequencer thread, which like a cyclic executive, is highest prio
    printf("Start sequencer\n");
    threadParams[0].sequencePeriods=90150;

    // Sequencer = RT_MAX	@ 30 Hz
    //
    rt_param[0].sched_priority=rt_max_prio;
    CPU_ZERO(&threadcpu_seq);
    CPU_SET(0, &threadcpu_seq);

    rc=pthread_attr_init(&rt_sched_attr[0]);
    rc=pthread_attr_setinheritsched(&rt_sched_attr[0], PTHREAD_EXPLICIT_SCHED);
    rc=pthread_attr_setschedpolicy(&rt_sched_attr[0], SCHED_FIFO);
    rc=pthread_attr_setaffinity_np(&rt_sched_attr[0], sizeof(cpu_set_t), &threadcpu_seq);

    threadParams[0].threadIdx=0;
    pthread_attr_setschedparam(&rt_sched_attr[0], &rt_param[0]);
    rc=pthread_create(&threads[0], &rt_sched_attr[0], sequencer, (void *)&(threadParams[0]));
    if(rc < 0)
        perror("pthread_create for sequencer service 0");
    else
        printf("pthread_create successful for sequeencer service 0\n");


    for(i=0;i<NUM_THREADS;i++)
        pthread_join(threads[i], NULL);

    // shutdown of frame acquisition service
    stop_capturing();
    uninit_device();
    close_device();
    fprintf(stderr, "\n");
    
//    fclose(fp_capture);
//    fclose(fp_sequencer);
//    fclose(fp_diff);
//    fclose(fp_sel);
//    fclose(fp_wb);
//    fclose(fp_change);
    
    printf("\nTEST COMPLETE\n");
    return 0;
}
