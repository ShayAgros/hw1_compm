/* 046267 Computer Architecture - Spring 2017 - HW #1 */
/* This file should hold your implementation of the CPU pipeline core simulator */

#include "sim_api.h"

typedef enum {
	IF = 0,
	REG,
	EXE,
	MEM,
	WB
} machine_state;

int pc;
SIM_cmd p_letch[SIM_PIPELINE_DEPTH];

/*! SIM_CoreReset: Reset the processor core simulator machine to start new simulation
  Use this API to initialize the processor core simulator's data structures.
  The simulator machine must complete this call with these requirements met:
  - PC = 0  (entry point for a program is at address 0)
  - All the register file is cleared (all registers hold 0)
  - The value of IF is the instuction in address 0x0
  \returns 0 on success. <0 in case of initialization failure.
*/
int SIM_CoreReset(void) {
    pc=0;
    return 0;
}

void if_tick() {
    

    SIM_MemInstRead(pc,&p_letch[IF]);
    printf("command: %s src1: %d src2: %d\n",cmdStr[p_letch[IF].opcode],p_letch[IF].src1,p_letch[IF].src2);
    printf("dst: %d\n\n",p_letch[IF].dst);
    pc+=4;

}

void reg_tick() {
    // we only had one command thus far
    if(pc%4 < 1)
	return;
	// add implementation here
    return;
}

void exe_tick() {
    if(pc%4 < 2)
	return;
	// add implementation here
    return;
}

void mem_tick() {
    // we only had one command thus far
    if(pc%4 < 3)
	return;
	// add implementation here
    return;
}

void wb_tick() {
    // we only had one command thus far
    if(pc%4 < 4)
	return;
	// add implementation here
    return;
}

/*! SIM_CoreClkTick: Update the core simulator's state given one clock cycle.
  This function is expected to update the core pipeline given a clock cycle event.
*/
void SIM_CoreClkTick() {

    if_tick();
    reg_tick();
    exe_tick();
    mem_tick();
    wb_tick();

    pc+=4;
}

/*! SIM_CoreGetState: Return the current core (pipeline) internal state
    curState: The returned current pipeline state
    The function will return the state of the pipe at the end of a cycle
*/
void SIM_CoreGetState(SIM_coreState *curState) {
}

