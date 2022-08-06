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

#define NUM_THREADS 64
#define NUM_CPUS 8
#define NUM_SKIPS 25
#define NUM_STABLE_FRAMES 181
#define NUM_PICTURES (NUM_SKIPS + NUM_STABLE_FRAMES)
#define ACQ_PERIOD 60
#define DUMP_PERIOD 71
#define TRANSFORM 0

#define SEQ_SECONDS 0
// #define SEQ_NANOSECONDS 8330000
#define SEQ_NANOSECONDS 16634600

struct metaframe
{
    unsigned char frame_data[(1280 * 960)];
    double time_of_acq;
    unsigned int frame_num;
    struct timespec *frametime;
    unsigned int size;
};

struct metaframe outbuffer[30];
struct metaframe acqbuffer[30];

unsigned int out_buf_pending;
unsigned int out_buf_current;

unsigned int acq_buf_pending;
unsigned int acq_buf_current;

unsigned char abortTest;

struct utsname sysname;

typedef struct
{
    int threadIdx;
} threadParams_t;

// POSIX thread declarations and scheduling attributes
//
sem_t semAcqPicture, semDumpPicture, semFrameSelector;

pthread_t threads[NUM_THREADS];
pthread_t mainthread;
pthread_t startthread;
pthread_t acqthread, dumpthread;
threadParams_t threadParams[NUM_THREADS];

pthread_attr_t fifo_sched_attr;
pthread_attr_t orig_sched_attr;
struct sched_param fifo_param;

#define SCHED_POLICY SCHED_FIFO

#define CLEAR(x) memset(&(x), 0, sizeof(x))
// #define COLOR_CONVERT_RGB
#define HRES 640
#define VRES 480
#define HRES_STR "640"
#define VRES_STR "480"

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
    void *start;
    size_t length;
};

static char *dev_name;
// static enum io_method   io = IO_METHOD_USERPTR;
// static enum io_method   io = IO_METHOD_READ;
static enum io_method io = IO_METHOD_MMAP;
static int fd = -1;
struct buffer *buffers;
static unsigned int n_buffers;
static int out_buf;
static int force_format = 1;
static int frame_count = 30;

void *Sequencer(void *threadp);
static void start_capturing(void);
static void init_device(void);
static void open_device(void);
static void usage(FILE *fp, int argc, char **argv);
void *take_picture(void *threadp);
void *dump_thread(void *threadparams);
// void *frame_selector(void *threadparams);

double getTimeMsec(void)
{
    struct timespec event_ts = {0, 0};

    clock_gettime(CLOCK_REALTIME, &event_ts);
    return ((event_ts.tv_sec) * 1000.0) + ((event_ts.tv_nsec) / 1000000.0);
}

void print_scheduler(void)
{
    int schedType = sched_getscheduler(getpid());

    switch (schedType)
    {
    case SCHED_FIFO:
        printf("Pthread policy is SCHED_FIFO\n");
        break;
    case SCHED_OTHER:
        printf("Pthread policy is SCHED_OTHER\n");
        break;
    case SCHED_RR:
        printf("Pthread policy is SCHED_RR\n");
        break;
    default:
        printf("Pthread policy is UNKNOWN\n");
    }
}

void set_scheduler(int cpu_id, int prio_offset)
{
    int max_prio, scope, rc, cpuidx;
    cpu_set_t cpuset;

    printf("INITIAL ");
    print_scheduler();

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

    printf("ADJUSTED ");
    print_scheduler();
}

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

    set_scheduler(1,0);

    CPU_ZERO(&cpuset);

    abortTest = 0;
    out_buf_pending = 99;
    out_buf_current = 0;

    uname(&sysname);

    // get affinity set for main thread
    mainthread = pthread_self();

    // Check the affinity mask assigned to the thread
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

    

    pthread_create(&startthread,     // pointer to thread descriptor
                   &fifo_sched_attr, // use FIFO RT max priority attributes
                   Sequencer,        // thread function entry point
                   (void *)0         // parameters to pass in
    );

    set_scheduler(2,0);

    pthread_create(&acqthread,       // pointer to thread descriptor
                   &fifo_sched_attr, // use FIFO RT max priority attributes
                   take_picture,     // thread function entry point
                   (void *)0         // parameters to pass in
    );
    set_scheduler(1,1);
    pthread_create(&dumpthread,      // pointer to thread descriptor
                   &fifo_sched_attr, // use FIFO RT max priority attributes
                   dump_thread,      // thread function entry point
                   (void *)0         // parameters to pass in
    );

    pthread_join(acqthread, NULL);
    pthread_join(dumpthread, NULL);
    abortTest = 1;
    pthread_join(startthread, NULL);
}

// ------------------------------SIMPLE_CAPTURE_CODE---------------------------------------------

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

char ppm_header[] = "P6\n#9999999999 sec 9999999999 msec \n" HRES_STR " " VRES_STR "\n255\n";
char ppm_dumpname[] = "frames/test0000.ppm";

static void dump_ppm(const void *p, int size, unsigned int tag, struct timespec *time)
{
    int written, i, total, dumpfd;

    snprintf(&ppm_dumpname[11], 9, "%04d", tag);
    strncat(&ppm_dumpname[15], ".ppm", 5);
    dumpfd = open(ppm_dumpname, O_WRONLY | O_NONBLOCK | O_CREAT, 00666);

    snprintf(&ppm_header[4], 11, "%010d", (int)time->tv_sec);
    strncat(&ppm_header[14], " sec ", 5);
    snprintf(&ppm_header[19], 11, "%010d", (int)((time->tv_nsec) / 1000000));
    strncat(&ppm_header[29], " msec \n" HRES_STR " " VRES_STR "\n255\n", 19);
    written = write(dumpfd, ppm_header, sizeof(ppm_header));

    total = 0;

    do
    {
        written = write(dumpfd, p, size);
        total += written;
    } while (total < size);

    printf("wrote %d bytes\n", total);

    close(dumpfd);
}

char pgm_header[] = "P5\n#9999999999 sec 9999999999 msec \n" HRES_STR " " VRES_STR "\n255\n";
char pgm_dumpname[] = "frames/test0000.pgm";
char uname_header[30];
char new_header[100];
static void dump_pgm(const void *p, int size, unsigned int tag, struct timespec *time)
{
    // double acq_inittime = getTimeMsec();
    int written, i, total, dumpfd;

    snprintf(&pgm_dumpname[11], 9, "%04d", tag - NUM_SKIPS);
    strncat(&pgm_dumpname[15], ".pgm", 5);
    dumpfd = open(pgm_dumpname, O_WRONLY | O_NONBLOCK | O_CREAT, 00666);

    snprintf(&pgm_header[4], 11, "%010d", (int)time->tv_sec);
    strncat(&pgm_header[14], " sec ", 5);
    snprintf(&pgm_header[19], 11, "%010d", (int)((time->tv_nsec) / 1000000));
    strncat(&pgm_header[29], " msec \n" HRES_STR " " VRES_STR "\n255\n", 19);
    snprintf(&pgm_header[19], 11, "%010d", (int)((time->tv_nsec) / 1000000));
    sprintf(&uname_header,"%S %S %S %S \n", sysname.sysname,sysname.nodename,sysname.release, sysname.machine);
    strcat(&new_header, &pgm_header);
    strcat(&new_header, &uname_header);

    written = write(dumpfd, pgm_header, sizeof(pgm_header));

    total = 0;

    do
    {
        written = write(dumpfd, p, size);
        total += written;
    } while (total < size);
    close(dumpfd);
    // syslog(LOG_CRIT, "time_ms,%f", getTimeMsec() - acq_inittime);
}

void yuv2rgb_float(float y, float u, float v,
                   unsigned char *r, unsigned char *g, unsigned char *b)
{
    float r_temp, g_temp, b_temp;

    // R = 1.164(Y-16) + 1.1596(V-128)
    r_temp = 1.164 * (y - 16.0) + 1.1596 * (v - 128.0);
    *r = r_temp > 255.0 ? 255 : (r_temp < 0.0 ? 0 : (unsigned char)r_temp);

    // G = 1.164(Y-16) - 0.813*(V-128) - 0.391*(U-128)
    g_temp = 1.164 * (y - 16.0) - 0.813 * (v - 128.0) - 0.391 * (u - 128.0);
    *g = g_temp > 255.0 ? 255 : (g_temp < 0.0 ? 0 : (unsigned char)g_temp);

    // B = 1.164*(Y-16) + 2.018*(U-128)
    b_temp = 1.164 * (y - 16.0) + 2.018 * (u - 128.0);
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
    int c = y - 16, d = u - 128, e = v - 128;

    // Conversion that avoids floating point
    r1 = (298 * c + 409 * e + 128) >> 8;
    g1 = (298 * c - 100 * d - 208 * e + 128) >> 8;
    b1 = (298 * c + 516 * d + 128) >> 8;

    // Computed values may need clipping.
    if (r1 > 255)
        r1 = 255;
    if (g1 > 255)
        g1 = 255;
    if (b1 > 255)
        b1 = 255;

    if (r1 < 0)
        r1 = 0;
    if (g1 < 0)
        g1 = 0;
    if (b1 < 0)
        b1 = 0;

    *r = r1;
    *g = g1;
    *b = b1;
}

unsigned int framecnt = 0;
unsigned char bigbuffer[(1280 * 960)];

static void process_image(const void *p, int size)
{

    int i, newi, newsize = 0;
    struct timespec frame_time;
    int y_temp, y2_temp, u_temp, v_temp;
    unsigned char *pptr = (unsigned char *)p;

    // record when process was called
    clock_gettime(CLOCK_REALTIME, &frame_time);

    framecnt++;
    printf("frame %d: ", framecnt);

    // This just dumps the frame to a file now, but you could replace with whatever image
    // processing you wish.
    //
    if (framecnt > 25)
    {
        if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_GREY)
        {
            printf("Dump graymap as-is size %d\n", size);
            dump_pgm(p, size, framecnt, &frame_time);
        }
        else if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV)
        {

            printf("Dump YUYV converted to YY size %d\n", size);

            // Pixels are YU and YV alternating, so YUYV which is 4 bytes
            // We want Y, so YY which is 2 bytes
            //
           for (i = 0, newi = 0; i < size; i = i + 4, newi = newi + 2)
            {
                // Y1=first byte and Y2=third byte
                if (!TRANSFORM)
                {
                    outbuffer[out_buf_current].frame_data[newi] = pptr[i];
                    outbuffer[out_buf_current].frame_data[newi + 1] = pptr[i + 2];
                }
                else
                {
                    outbuffer[out_buf_current].frame_data[newi] = 255 - pptr[i];
                    outbuffer[out_buf_current].frame_data[newi + 1] = 255 - pptr[i + 2];
                }
            }
            outbuffer[out_buf_current].size = (size / 2);
            outbuffer[out_buf_current].frametime = &frame_time;
            outbuffer[out_buf_current].frame_num = framecnt;

            if (out_buf_pending == 99)
                out_buf_pending = out_buf_current;

            if (out_buf_current < 29)
                out_buf_current++;
            else
                out_buf_current = 0;

            // dump_pgm(bigbuffer, (size / 2), framecnt, &frame_time);
        }
        else if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_RGB24)
        {
            printf("Dump RGB as-is size %d\n", size);
            dump_ppm(p, size, framecnt, &frame_time);
        }
        else
        {
            printf("ERROR - unknown dump format\n");
        }
    }

    fflush(stderr);
    // fprintf(stderr, ".");
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

void *take_picture(void *threadp)
{
    unsigned int count;
    count = 0;
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
    printf("Exiting Take Picture \n\n");
    pthread_exit((void *)0);
}

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

static void close_device(void)
{
    if (-1 == close(fd))
        errno_exit("close");

    fd = -1;
}

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

void *Sequencer(void *threadp)
{
    struct timespec sleep_time;
    struct timespec time_error;

    sleep_time.tv_sec = SEQ_SECONDS;
    sleep_time.tv_nsec = SEQ_NANOSECONDS;

    double acq_time = getTimeMsec(), sel_time = getTimeMsec(), dump_time = getTimeMsec();
    int cnt_acq = 0;
    int frame_count = NUM_PICTURES, cnt_sel = 0, cnt_dump = 0;

    printf("\n\n Waiting for Trigger, Press any Key \n\n");
    getchar();

    while (!abortTest)
    {
        cnt_acq++;
        cnt_sel++;
        cnt_dump++;

        if (cnt_acq == ACQ_PERIOD && frame_count > 0)
        {
            sem_post(&semAcqPicture);
            // syslog(LOG_CRIT,"This should be 32ms %f\n", getTimeMsec() - acq_time);
            // printf("This should be 32ms %f\n", getTimeMsec() - acq_time);
            acq_time = getTimeMsec();
            frame_count--;
            cnt_acq = 0;
        }

        // if (cnt_sel == 30)
        // {
        //     printf("This should be 500ms %f\n", getTimeMsec() - sel_time);
        //     sel_time = getTimeMsec();
        //     cnt_sel = 0;
        // }

        if (cnt_dump == DUMP_PERIOD)
        {
            sem_post(&semDumpPicture);
            // printf("This should be 2000ms %f\n", getTimeMsec() - dump_time);
            dump_time = getTimeMsec();
            cnt_dump = 0;
        }
        nanosleep(&sleep_time, &time_error);
    }
    printf("Exiting Sequencer \n\n");
    pthread_exit((void *)0);
}
unsigned int dump_count = 0;
void *dump_thread(void *threadparams)
{
    while (dump_count != NUM_STABLE_FRAMES)
    {        
        sem_wait(&semDumpPicture);
        while (out_buf_pending != out_buf_current && out_buf_pending != 99)
        {
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


// void *frame_selector(void *threadparams)
// {
//     while (dump_count != NUM_STABLE_FRAMES)
//     {        
//         sem_wait(&semFrameSelector);

//     }
//     printf("Exiting Frame Selector \n\n");
//     pthread_exit((void *)0);
// }