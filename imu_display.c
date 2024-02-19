#include <ncurses.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdint.h>

#define NUM_LINES           6
#define DELAY               500000  
#define BMX160_TIME_TICK_US 39
#define FRAME_SIZE          32

#define CONST_G                 9.80665f
#define CONST_RAW_SCALE         32768.0f
#define HALF_RANGE_ACC_G        2*CONST_G
#define HALF_RANGE_GYR_DPS      2000.0f

static inline float raw_to_float(int16_t raw, float half_range){
    return -half_range*(raw/CONST_RAW_SCALE);
} 

int main() {

    initscr();
    cbreak();
    noecho();
    curs_set(0);

    char *names[NUM_LINES] = {"ACCX: ","ACCY: ","ACCZ: ","GYRX: ","GYRY: ","GYRZ: "};
    char *units[NUM_LINES] = {"m/sec^2", "m/sec^2", "m/sec^2", "degrees/sec", "degrees/sec", "degrees/sec"};
    int values[NUM_LINES] = {0};

    const char *frame_path = "/sys/bus/i2c/devices/i2c-1/1-0068/raw_frame";
    int frame_f = open(frame_path, O_RDONLY);

    if(frame_f == -1){
        printf("ERROR: Cannot open file\n");
        return -1;
    }

    uint8_t frame_buf[FRAME_SIZE];
    struct bmx160_frame_t {
        int16_t gyrx, gyry, gyrz;
        int16_t accx, accy, accz;
        int32_t time;
    } *bmx160_frame = (struct bmx160_frame_t *)frame_buf;

    ssize_t ret;
    int err;

    while (1) {
        clear();

        if (lseek(frame_f, 0, SEEK_SET) == -1) {
            printw("ERROR: Can't seek to file start\n");
            break;
        }
        ret = read(frame_f, frame_buf, sizeof(uint8_t)*FRAME_SIZE);
        if(ret == -1){
            printw("ERROR: Unable to read file\n");
            break;
        }
        
        printw("%7s: %3.4f %s\n\n", names[0], raw_to_float(bmx160_frame->accx, HALF_RANGE_ACC_G), units[0]);
        printw("%7s: %3.4f %s\n\n", names[1], raw_to_float(bmx160_frame->accy, HALF_RANGE_ACC_G), units[1]);
        printw("%7s: %3.4f %s\n\n", names[2], raw_to_float(bmx160_frame->accz, HALF_RANGE_ACC_G) + 0.27495, units[2]);
        printw("%7s: %5.4f %s\n\n", names[3], raw_to_float(bmx160_frame->gyrx, HALF_RANGE_GYR_DPS), units[3]);
        printw("%7s: %5.4f %s\n\n", names[4], raw_to_float(bmx160_frame->gyry, HALF_RANGE_GYR_DPS), units[4]);
        printw("%7s: %5.4f %s\n\n", names[5], raw_to_float(bmx160_frame->gyrz, HALF_RANGE_GYR_DPS), units[5]);

        refresh();
        usleep(DELAY);
    }

    endwin();
    close(frame_f);
    return 0;
}
