#include <stdio.h>
#ifdef TEST_OLD_FSM
#include "../../Ant_FSM_20261028/line_control.h"
#include "../../Ant_FSM_20261028/control_config.h"
#define MODE_WAIT MODE_STOP
#else
unsigned char P1_0, P1_1;
static unsigned char raw_port;
static unsigned int reads, frames, phase, pulse_l, pulse_r;
#define main firmware_main
#include "../god.c"
#undef main
unsigned char TestReadP0(void) { reads++; return raw_port; }
void uart_Init(void) {}
void delay_nus(unsigned int n)
{
    if (!phase) pulse_l = n;
    else pulse_r = n;
    phase++;
}
void delay_nms(unsigned int n)
{
    if (n != FRAME_MS || phase != 2 || P1_0 || P1_1) {
        puts("Invalid pulse frame");
        frames = 10000;
    }
    phase = 0;
    frames++;
}
#endif
static unsigned int failures, cases;
static unsigned long checks;
#define EQ(x,y) do { long a_=(long)(x), b_=(long)(y); checks++; \
    if(a_!=b_){failures++;printf("FAIL line %d: %s = %ld expected %ld\n",__LINE__,#x,a_,b_);} }while(0)

static void ready(void)
{
    unsigned int i;
    ControlInit();
    for(i=0;i<14;i++) ControlStep(6);
}

static void normal_patterns(void)
{
    static const unsigned char expected[16] = {
        ACT_RIGHT,ACT_RIGHT,ACT_TRIM_RIGHT,ACT_RIGHT,
        ACT_TRIM_LEFT,ACT_TRIM_RIGHT,ACT_GO,ACT_RIGHT,
        ACT_LEFT,ACT_GO,ACT_TRIM_LEFT,ACT_TRIM_RIGHT,
        ACT_LEFT,ACT_TRIM_LEFT,ACT_LEFT,ACT_GO
    };
    unsigned int i;
    cases++;
    for(i=0;i<16;i++) { ready(); EQ(ControlStep((unsigned char)i),expected[i]); }
}

static void startup_waits_for_line(void)
{
    unsigned int i;
    cases++;
    ControlInit();
    for(i=0;i<100;i++) EQ(ControlStep(0),ACT_STOP);
    EQ(control.mode,MODE_WAIT);
    ControlStep(15); ControlStep(9); ControlStep(1); ControlStep(8);
    EQ(control.mode,MODE_WAIT);
    ControlStep(7); ControlStep(7);
    EQ(control.mode,MODE_TRACK);
}

static void wide_line_does_not_latch(void)
{
    unsigned int i;
    cases++;
    ready();
    for(i=0;i<200;i++) EQ(ControlStep(15),ACT_GO);
    EQ(control.mode,MODE_TRACK);
    ready();
    for(i=0;i<50;i++) {
        EQ(ControlStep(7),ACT_RIGHT);
        EQ(ControlStep(14),ACT_LEFT);
    }
    EQ(control.mode,MODE_TRACK);
    EQ(control.last_valid_dir,0);
}

static void bounded_search_and_auto_resume(void)
{
    unsigned int i;
    cases++;
    ready();
    for(i=0;i<200 && control.mode!=MODE_WAIT;i++) ControlStep(0);
    EQ(control.mode,MODE_WAIT);
    EQ(ControlStep(0),ACT_STOP);
    ControlStep(6);
    EQ(control.mode,MODE_WAIT);
    EQ(ControlStep(6),ACT_SLOW_GO);
    EQ(control.mode,MODE_TRACK);
    for(i=0;i<20;i++) ControlStep(6);
    EQ(ControlStep(6),ACT_GO);
}

static void turn_white_and_capture(void)
{
    unsigned int i;
    cases++;
    ready();
    ControlStep(8); ControlStep(8);
    EQ(control.mode,MODE_TURN);
    for(i=0;i<6;i++) EQ(ControlStep(0),ACT_LEFT);
    EQ(ControlStep(1),ACT_LEFT);
    EQ(control.turn_dir,-1);
    ControlStep(14); ControlStep(14);
    EQ(control.mode,MODE_TRACK);
    for(i=0;i<12;i++) ControlStep(14);
    EQ(control.episode_active,0);
}

static void no_false_single_capture(void)
{
    unsigned int i;
    cases++;
    ready();
    ControlStep(0);
    for(i=0;i<100 && control.mode!=MODE_WAIT;i++) {
        ControlStep((unsigned char)(i%2 ? 0 : 6));
        if(control.mode!=MODE_WAIT) EQ(control.mode,MODE_SEARCH);
    }
    EQ(control.mode,MODE_WAIT);
    ControlStep(6); ControlStep(0); ControlStep(6);
    EQ(control.mode,MODE_WAIT);
}

static void repeated_false_recovery_is_bounded(void)
{
    static const unsigned char sequence[]={0,6,6,15};
    unsigned int i;
    cases++;
    ready();
    for(i=0;i<EPISODE_MAX_FRAMES;i++) {
        ControlStep(sequence[i%4]);
        EQ(control.mode==MODE_WAIT,0);
    }
    EQ(control.episode_frames,EPISODE_MAX_FRAMES);
    EQ(ControlStep(0),ACT_STOP);
    EQ(control.mode,MODE_WAIT);
}

static void ambiguous_does_not_change_memory(void)
{
    static const unsigned char ambiguous[]={5,7,9,10,11,13,14,15};
    unsigned int i;
    cases++;
    for(i=0;i<sizeof(ambiguous);i++) {
        ready(); ControlStep(4); ControlStep(ambiguous[i]);
        EQ(control.last_valid_dir,-1);
    }
}

#ifndef TEST_OLD_FSM
static void exact_search_and_turn_boundaries(void)
{
    unsigned int i;
    cases++;
    ready(); ControlStep(4); /* reliable left history */
    for(i=0;i<20;i++) EQ(ControlStep(0),ACT_LEFT);
    for(i=0;i<40;i++) EQ(ControlStep(0),ACT_RIGHT);
    for(i=0;i<20;i++) EQ(ControlStep(0),ACT_LEFT);
    EQ(ControlStep(0),ACT_STOP); EQ(control.mode,MODE_WAIT);
    ready();
    EQ(ControlStep(1),ACT_RIGHT);
    EQ(ControlStep(6),ACT_RIGHT);
    EQ(ControlStep(6),ACT_RIGHT);
    EQ(control.mode,MODE_TURN); /* minimum three committed pulses */
    EQ(ControlStep(6),ACT_SLOW_GO);
    EQ(control.mode,MODE_TURN); /* only the first capture candidate */
    EQ(ControlStep(6),ACT_SLOW_GO);
    EQ(control.mode,MODE_TRACK);
    ready();
    ControlStep(8);
    for(i=1;i<TURN_MAX_FRAMES;i++) EQ(ControlStep(0),ACT_LEFT);
    EQ(control.mode,MODE_TURN);
    EQ(ControlStep(0),ACT_LEFT); EQ(control.mode,MODE_SEARCH);
}

static void wait_patterns_and_recovery_relapse(void)
{
    unsigned int m, i;
    unsigned char expected;
    cases++;
    for(m=0;m<16;m++) {
        ControlInit();
        expected=(m==2 || m==3 || m==4 || m==6 || m==7 || m==12 || m==14);
        ControlStep((unsigned char)m); EQ(control.mode,MODE_WAIT);
        ControlStep((unsigned char)m);
        EQ(control.mode,expected ? MODE_TRACK : MODE_WAIT);
    }
    ready(); ControlStep(0); ControlStep(6); ControlStep(6);
    EQ(control.mode,MODE_TRACK); EQ(control.episode_active,1);
    ControlStep(0);
    EQ(control.mode,MODE_SEARCH); EQ(control.episode_active,1);
    EQ(control.episode_frames,4);
    ControlStep(6); ControlStep(6);
    for(i=0;i<RECOVER_FRAMES-2;i++) {
        EQ(ControlStep(6),ACT_SLOW_GO); EQ(control.episode_active,1);
    }
    EQ(ControlStep(6),ACT_SLOW_GO); EQ(control.episode_active,0);
    EQ(ControlStep(6),ACT_GO);
}

static void hardware_boundary(void)
{
    unsigned int i;
    static const unsigned int expected[10][2]={
        {1535,1505},{1720,1320},{1570,1170},{1870,1470},{1070,1070},
        {1970,1970},{1120,1920},{1680,1360},{1600,1280},{1760,1440}
    };
    cases++;
    for(i=0;i<16;i++) {
        raw_port=(unsigned char)((i<<4)|15); reads=0;
        EQ(ReadLineMask(),i); EQ(reads,1);
    }
    for(i=0;i<10;i++) {
        frames=0; ApplyAction((unsigned char)i);
        EQ(frames,1); EQ(pulse_l,expected[i][0]); EQ(pulse_r,expected[i][1]);
    }
}
#endif

int main(void)
{
    normal_patterns();
    startup_waits_for_line();
    wide_line_does_not_latch();
    bounded_search_and_auto_resume();
    turn_white_and_capture();
    no_false_single_capture();
    repeated_false_recovery_is_bounded();
    ambiguous_does_not_change_memory();
#ifndef TEST_OLD_FSM
    exact_search_and_turn_boundaries();
    wait_patterns_and_recovery_relapse();
    hardware_boundary();
#endif
    printf("%u cases, %lu checks, %u failures\n",cases,checks,failures);
    return failures?1:0;
}
