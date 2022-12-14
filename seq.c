/***************************************************************************
 * RTES Final Project
 * Author: Starter code by Sam Siewert, Modifications + new code by Chinmay Shalawadi
 * Institution: University of Colorado Boulder
 * Mail id: chsh1552@colorado.edu
 * References: lecture slides, Starter Code & Walkthough videos
 ***************************************************************************/

#define _GNU_SOURCE
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <sys/stat.h>
#include <errno.h>
#include <time.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/utsname.h>
#include <unistd.h>
#include <sched.h>
#include <semaphore.h>
#include <stdio.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <syslog.h>
#include <linux/videodev2.h>


//Uncomment Bottom Line for 1Hz and Comment out for 10Hz
// #define ONEHZ 
#ifdef ONEHZ
#define ACQ_PERIOD 60
#define DUMP_PERIOD 133
#define SEL_PERIOD 89
#define SEQ_NANOSECONDS 16634666
#define NUM_STABLE_FRAMES 181
#define NUM_SKIPS 25
#else
#define ACQ_PERIOD 6
#define DUMP_PERIOD 13
#define SEL_PERIOD 8
#define SEQ_NANOSECONDS 16634000
#define NUM_STABLE_FRAMES 1801
#define NUM_SKIPS 40
#endif

#define NUM_PICTURES (NUM_SKIPS + NUM_STABLE_FRAMES)

//10Hz with transform needs finer tuned nanoseconds
#define TRANSFORM 0
// #define SEQ_NANOSECONDS 16633666
#define SEQ_SECONDS 0

//Global variables and Definitions
struct metaframe
{
    unsigned char frame_data[(1280 * 960)];
    double time_of_acq;
    unsigned int frame_num;
    struct timespec *frametime;
    unsigned int size;
};

//Write back buffer 
struct metaframe outbuffer[30];
//Acquisition Buffer
struct metaframe acqbuffer[90];

unsigned int out_buf_pending;
unsigned int out_buf_current;
unsigned int acq_buf_pending;
unsigned int acq_buf_current;
unsigned char abortTest;


sem_t semAcqPicture, semDumpPicture, semFrameSelector, seqBlocker;

pthread_t mainthread;
pthread_t startthread;
pthread_t acqthread, dumpthread, selthread;

pthread_attr_t fifo_sched_attr;
pthread_attr_t orig_sched_attr;
struct sched_param fifo_param;

#define SCHED_POLICY SCHED_FIFO

#define CLEAR(x) memset(&(x), 0, sizeof(x))
#define HRES 640
#define VRES 480
#define HRES_STR "640"
#define VRES_STR "480"

static struct v4l2_format fmt;

enum io_method
{
    IO_METHOD_READ,
    IO_METHOD_MMAP,
    IO_METHOD_USERPTR,
};

struct buffer
{
    void *start;
    size_t length;
};

static char *dev_name;
static enum io_method io = IO_METHOD_MMAP;
static int fd = -1;
struct buffer *buffers;
static unsigned int n_buffers;
static int out_buf;
static int force_format = 1;
static int frame_count = 30;

//Function Declarations
void *Sequencer(void *threadp);
static void start_capturing(void);
static void init_device(void);
static void open_device(void);
static void usage(FILE *fp, int argc, char **argv);
void *take_picture(void *threadp);
void *dump_thread(void *threadparams);
void *frame_selector(void *threadparams);
static void uninit_device(void);
static void stop_capturing(void);
static void close_device(void);

// -------------------------------------getTimeMsec-------------------------------------------------

double getTimeMsec(void)
{
    struct timespec event_ts = {0, 0};

    clock_gettime(CLOCK_REALTIME, &event_ts);
    return ((event_ts.tv_sec) * 1000.0) + ((event_ts.tv_nsec) / 1000000.0);
}

// -------------------------------------set_scheduler-------------------------------------------------

void set_scheduler(int cpu_id, int prio_offset)
{
    int max_prio, scope, rc, cpuidx;
    cpu_set_t cpuset;
    pthread_attr_init(&fifo_sched_attr);
    pthread_attr_setinheritsched(&fifo_sched_attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&fifo_sched_attr, SCHED_POLICY);
    CPU_ZERO(&cpuset);
    cpuidx = (cpu_id);
    CPU_SET(cpuidx, &cpuset);
    pthread_attr_setaffinity_np(&fifo_sched_attr, sizeof(cpu_set_t), &cpuset);

    max_prio = sched_get_priority_max(SCHED_POLICY);
    fifo_param.sched_priority = max_prio - prio_offset;

    if ((rc = sched_setscheduler(getpid(), SCHED_POLICY, &fifo_param)) < 0)
        perror("sched_setscheduler");

    pthread_attr_setschedparam(&fifo_sched_attr, &fifo_param);
}
// -------------------------------------main-------------------------------------------------

int main(int argc, char *argv[])
{

    if (argc > 1)
        dev_name = argv[1];
    else
        dev_name = "/dev/video0";

    open_device();
    init_device();
    start_capturing();

    int rc;
    int i, j;

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);

    // Initializing Globals
    abortTest = 0;
    out_buf_pending = 999;
    out_buf_current = 0;
    acq_buf_current = 0;
    acq_buf_pending = 999;


    mainthread = pthread_self();


    rc = pthread_getaffinity_np(mainthread, sizeof(cpu_set_t), &cpuset);
    if (rc != 0)
        perror("pthread_getaffinity_np");
    else
    {
        printf("main thread running on CPU=%d, CPUs =", sched_getcpu());

        for (j = 0; j < CPU_SETSIZE; j++)
            if (CPU_ISSET(j, &cpuset))
                printf(" %d", j);

        printf("\n");
    }

    //Starting Sequencer on core 1
    set_scheduler(1, 0);
    printf("Spawning Sequencer on Core 1, Max Priority \n\n");
    pthread_create(&startthread,
                   &fifo_sched_attr,
                   Sequencer,
                   (void *)0);

    //Starting Frame Acquisition on Core 2
    set_scheduler(2, 0);
    printf("Spawning Take Picture on Core 2, Max Priority \n\n");
    pthread_create(&acqthread,
                   &fifo_sched_attr,
                   take_picture,
                   (void *)0);

    //Starting Frame Selector on Core 3
    set_scheduler(3, 0);
    printf("Spawning Frame Selector on Core 3, Max Priority \n\n");
    pthread_create(&selthread,
                   &fifo_sched_attr,
                   frame_selector,
                   (void *)0);

    //Starting Frame Writeback on Core 3
    set_scheduler(3, 1);
    printf("Spawning Frame Writeback on Core 3, Lower Priority \n\n");

    pthread_create(&dumpthread,
                   &fifo_sched_attr,
                   dump_thread,
                   (void *)0);

    printf("\n\n Waiting for Trigger, Press any Key \n\n");
    getchar();

    //Releasing lock after trigger
    sem_post(&seqBlocker);
    printf("capturing..");


    pthread_join(acqthread, NULL);
    pthread_join(selthread, NULL);
    pthread_join(dumpthread, NULL);
    abortTest = 1;
    pthread_join(startthread, NULL);

    //Closing the Device
    stop_capturing();
    uninit_device();
    close_device();
}

// -------------------------------------errno_exit-------------------------------------------------

static void errno_exit(const char *s)
{
    fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
    exit(EXIT_FAILURE);
}
// -------------------------------------xioctl-------------------------------------------------

static int xioctl(int fh, int request, void *arg)
{
    int r;

    do
    {
        r = ioctl(fh, request, arg);

    } while (-1 == r && EINTR == errno);

    return r;
}

// -------------------------------------dump_pgm-------------------------------------------------

char pgm_header[] = "P5\n#9999999999 sec 9999999999 msec Linux raspberrypi 5.15.32-v7l+ #1538 SMP Thu Mar 31 19:39:41 BST 2022 armv7l GNU/Linux \n\n" HRES_STR " " VRES_STR "\n255\n";
char pgm_dumpname[] = "frames/test0000.pgm";

static void dump_pgm(const void *p, int size, unsigned int tag, struct timespec *time)
{

    int written, i, total, dumpfd;

    snprintf(&pgm_dumpname[11], 9, "%04d", tag - NUM_SKIPS);
    strncat(&pgm_dumpname[15], ".pgm", 5);
    dumpfd = open(pgm_dumpname, O_WRONLY | O_NONBLOCK | O_CREAT, 00666);

    snprintf(&pgm_header[4], 11, "%010d", (int)time->tv_sec);
    strncat(&pgm_header[14], " sec ", 5);
    snprintf(&pgm_header[19], 11, "%010d", (int)((time->tv_nsec) / 1000000));
    snprintf(&pgm_header[19], 11, "%010d", (int)((time->tv_nsec) / 1000000));
    strncat(&pgm_header[29], " msec \n" HRES_STR " " VRES_STR "\n255\n", 19);
    snprintf(&pgm_header[19], 11, "%010d", (int)((time->tv_nsec) / 1000000));

    written = write(dumpfd, pgm_header, sizeof(pgm_header));

    total = 0;

    do
    {
        written = write(dumpfd, p, size);
        total += written;
    } while (total < size);
    close(dumpfd);
}

// -------------------------------------process_image-------------------------------------------------

unsigned int framecnt = 0;
unsigned char bigbuffer[(1280 * 960)];

static void process_image(const void *p, int size)
{

    int i, newi, newsize = 0;
    struct timespec frame_time;
    int y_temp, y2_temp, u_temp, v_temp;
    unsigned char *pptr = (unsigned char *)p;

    clock_gettime(CLOCK_REALTIME, &frame_time);

    framecnt++;
    if (!(framecnt % 10))
        printf(".");

    if (framecnt > NUM_SKIPS)
    {
        if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_GREY)
        {
            printf("Dump graymap as-is size %d\n", size);
            dump_pgm(p, size, framecnt, &frame_time);
        }
        else if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV)
        {

            for (i = 0, newi = 0; i < size; i = i + 4, newi = newi + 2)
            {
                
                if (!TRANSFORM)
                {
                    acqbuffer[acq_buf_current].frame_data[newi] = pptr[i];
                    acqbuffer[acq_buf_current].frame_data[newi + 1] = pptr[i + 2];
                }
                else
                {
                    //If negative transform, subtract from 255 and then copy to buffer
                    acqbuffer[acq_buf_current].frame_data[newi] = 255 - pptr[i];
                    acqbuffer[acq_buf_current].frame_data[newi + 1] = 255 - pptr[i + 2];
                }
            }

            //Updating buffer control variables
            acqbuffer[acq_buf_current].size = (size / 2);
            acqbuffer[acq_buf_current].frametime = &frame_time;
            acqbuffer[acq_buf_current].frame_num = framecnt;

            if (acq_buf_pending == 999)
                acq_buf_pending = acq_buf_current;

            if (acq_buf_current < 89)
                acq_buf_current++;
            else
                acq_buf_current = 0;

        }
  
        else
        {
            printf("ERROR - unknown dump format\n");
        }
    }

    fflush(stderr);
    fflush(stdout);
}
// -------------------------------------read_frame-------------------------------------------------

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
            if (buf.m.userptr == (unsigned long)buffers[i].start && buf.length == buffers[i].length)
                break;

        assert(i < n_buffers);

        process_image((void *)buf.m.userptr, buf.bytesused);

        if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
            errno_exit("VIDIOC_QBUF");
        break;
    }

    // printf("R");

    return 1;
}
// -------------------------------------take_picture-------------------------------------------------

void *take_picture(void *threadp)
{
    unsigned int count;
    count = 0;
    double acqtime;
    while (count < NUM_PICTURES)
    {
        sem_wait(&semAcqPicture);   
        count++;
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
                break;
        }
 
    }
    printf("\nExiting Take Picture \n\n");
    pthread_exit((void *)0);
}
// -------------------------------------stop_capturing-------------------------------------------------

static void stop_capturing(void)
{
    enum v4l2_buf_type type;

    switch (io)
    {
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
// -------------------------------------start_capturing-------------------------------------------------

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
        for (i = 0; i < n_buffers; ++i)
        {
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
// -------------------------------------uninit_device-------------------------------------------------

static void uninit_device(void)
{
    unsigned int i;

    switch (io)
    {
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
// -------------------------------------init_read-------------------------------------------------

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
// -------------------------------------init_mmap-------------------------------------------------

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
                            "memory mapping\n",
                    dev_name);
            exit(EXIT_FAILURE);
        }
        else
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

    for (n_buffers = 0; n_buffers < req.count; ++n_buffers)
    {
        struct v4l2_buffer buf;

        CLEAR(buf);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = n_buffers;

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
// -------------------------------------init_userp-------------------------------------------------

static void init_userp(unsigned int buffer_size)
{
    struct v4l2_requestbuffers req;

    CLEAR(req);

    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_USERPTR;

    if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req))
    {
        if (EINVAL == errno)
        {
            fprintf(stderr, "%s does not support "
                            "user pointer i/o\n",
                    dev_name);
            exit(EXIT_FAILURE);
        }
        else
        {
            errno_exit("VIDIOC_REQBUFS");
        }
    }

    buffers = calloc(4, sizeof(*buffers));

    if (!buffers)
    {
        fprintf(stderr, "Out of memory\n");
        exit(EXIT_FAILURE);
    }

    for (n_buffers = 0; n_buffers < 4; ++n_buffers)
    {
        buffers[n_buffers].length = buffer_size;
        buffers[n_buffers].start = malloc(buffer_size);

        if (!buffers[n_buffers].start)
        {
            fprintf(stderr, "Out of memory\n");
            exit(EXIT_FAILURE);
        }
    }
}
// -------------------------------------init_device-------------------------------------------------

static void init_device(void)
{
    struct v4l2_capability cap;
    struct v4l2_cropcap cropcap;
    struct v4l2_crop crop;
    unsigned int min;

    if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap))
    {
        if (EINVAL == errno)
        {
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
        fmt.fmt.pix.width = HRES;
        fmt.fmt.pix.height = VRES;

        // Specify the Pixel Coding Formate here

        // This one work for Logitech C200
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;

        // fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
        // fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_VYUY;

        // Would be nice if camera supported
        // fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;
        // fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;

        // fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;
        fmt.fmt.pix.field = V4L2_FIELD_NONE;

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
// -------------------------------------close_device-------------------------------------------------

static void close_device(void)
{
    if (-1 == close(fd))
        errno_exit("close");

    fd = -1;
}
// -------------------------------------open_device-------------------------------------------------

static void open_device(void)
{
    struct stat st;

    if (-1 == stat(dev_name, &st))
    {
        fprintf(stderr, "Cannot identify '%s': %d, %s\n",
                dev_name, errno, strerror(errno));
        exit(EXIT_FAILURE);
    }

    if (!S_ISCHR(st.st_mode))
    {
        fprintf(stderr, "%s is no device\n", dev_name);
        exit(EXIT_FAILURE);
    }

    fd = open(dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);

    if (-1 == fd)
    {
        fprintf(stderr, "Cannot open '%s': %d, %s\n",
                dev_name, errno, strerror(errno));
        exit(EXIT_FAILURE);
    }
}
// -------------------------------------usage-------------------------------------------------

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
        {"device", required_argument, NULL, 'd'},
        {"help", no_argument, NULL, 'h'},
        {"mmap", no_argument, NULL, 'm'},
        {"read", no_argument, NULL, 'r'},
        {"userp", no_argument, NULL, 'u'},
        {"output", no_argument, NULL, 'o'},
        {"format", no_argument, NULL, 'f'},
        {"count", required_argument, NULL, 'c'},
        {0, 0, 0, 0}};
// -------------------------------------Sequencer-------------------------------------------------

void *Sequencer(void *threadp)
{
    struct timespec sleep_time;
    struct timespec time_error;

    sleep_time.tv_sec = SEQ_SECONDS;
    sleep_time.tv_nsec = SEQ_NANOSECONDS;

    double acq_time = getTimeMsec(), sel_time = getTimeMsec(), dump_time = getTimeMsec();
    int cnt_acq = 0;
    int frame_count = NUM_PICTURES, cnt_sel = 0, cnt_dump = 0;

    sem_wait(&seqBlocker);

    while (!abortTest)
    {
        cnt_acq++;
        cnt_sel++;
        cnt_dump++;

        //Sequencer
        if (cnt_acq == ACQ_PERIOD && frame_count > 0)
        {
            sem_post(&semAcqPicture);
            frame_count--;
            cnt_acq = 0;
        }

        //Frame Selector
        if (cnt_sel == SEL_PERIOD)
        {
            sem_post(&semFrameSelector);
            cnt_sel = 0;
        }

        //Frame Writeback
        if (cnt_dump == DUMP_PERIOD)
        {
            sem_post(&semDumpPicture);
            cnt_dump = 0;
        }

        nanosleep(&sleep_time, &time_error);
    }
    printf("Exiting Sequencer \n\n");
    pthread_exit((void *)0);
}
unsigned int dump_count = 0;
unsigned int sel_count = 0;
// -------------------------------------dump_thread-------------------------------------------------

void *dump_thread(void *threadparams)
{
    double acqtime;
    while (dump_count != NUM_STABLE_FRAMES)
    {
        sem_wait(&semDumpPicture);      
        while (out_buf_pending != out_buf_current && out_buf_pending != 999)
        {
            //Writing from outbuffer to filesystem
            dump_pgm(outbuffer[out_buf_pending].frame_data, outbuffer[out_buf_pending].size, outbuffer[out_buf_pending].frame_num, outbuffer[out_buf_pending].frametime);

            if (out_buf_pending == 29)
                out_buf_pending = 0;
            else
                out_buf_pending++;

            dump_count++;
        }        
    }

    printf("Exiting Dumper \n\n");
    pthread_exit((void *)0);
}
// -------------------------------------frame_selector-------------------------------------------------

void *frame_selector(void *threadparams)
{
    double acqtime;
    while (sel_count != NUM_STABLE_FRAMES)
    {
        sem_wait(&semFrameSelector);     
        //Copying the frame from acq buffer to outbuffer
        while (acq_buf_pending != acq_buf_current && acq_buf_pending != 999)
        {
            memcpy(&outbuffer[out_buf_current].frame_data, &acqbuffer[acq_buf_pending].frame_data, acqbuffer[acq_buf_pending].size);
            outbuffer[out_buf_current].size = acqbuffer[acq_buf_pending].size;
            outbuffer[out_buf_current].frametime = acqbuffer[acq_buf_pending].frametime;
            outbuffer[out_buf_current].frame_num = acqbuffer[acq_buf_pending].frame_num;

            //Updating the buffer control variables
            if (out_buf_pending == 999)
                out_buf_pending = out_buf_current;

            if (out_buf_current < 29)
                out_buf_current++;
            else
                out_buf_current = 0;

            if (acq_buf_pending == 89)
                acq_buf_pending = 0;
            else
                acq_buf_pending++;

            sel_count++;
        }        
    }
    printf("Exiting Frame Selector \n\n");
    pthread_exit((void *)0);
}

// -------------------------------------End-------------------------------------------------
