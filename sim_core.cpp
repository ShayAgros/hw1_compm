/* 046267 Computer Architecture - Spring 2017 - HW #1 */
/* This file should hold your implementation of the CPU pipeline core simulator */

#include "sim_api.h"
#include <string.h>

#define DEBUG(msg,args...); printf(msg,args);
//#define DEBUG(msg,args...);

// declerations
void update_latch(SIM_cmd *cmd, pipeStage state);
void fetch_registers(SIM_cmd *cmd);

int pc;
int regFile[SIM_REGFILE_SIZE];
PipeStageState p_letch[SIM_PIPELINE_DEPTH];
int32_t wb_value[SIM_PIPELINE_DEPTH];
int stall_cycles;


// updates a pipe's latch.
// @cmd = the command to be processed by the stage
// @ state = the state whose letch is altered
void update_latch(SIM_cmd *cmd, pipeStage state) {
    memcpy(&p_letch[state].cmd,cmd,sizeof(SIM_cmd));
}

int32_t do_minus(int32_t src1, int32_t src2) {
    return src1 - src2;
}
int32_t do_plus(int32_t src1, int32_t src2) {
    return src1 + src2;
}

// TODO 1) add verification that r0 is no being assigned
//	2) add support for any kind of execution delay 
void do_arithmetic(SIM_cmd *cmd,int src1,int32_t src2) {
	int (*arithmetic_func)(int,int);
	
	if(cmd->opcode == CMD_ADD || cmd->opcode == CMD_ADDI)
	    arithmetic_func = &do_plus;  
	else
	    arithmetic_func = &do_minus;

	wb_value[MEMORY] = arithmetic_func((int32_t)src1,src2);

	DEBUG("EXECUTE:calculated value for register %d. Its new val wiil be %d."
		" arguments: arg1:%d arg2:%d\n",cmd->dst,wb_value[MEMORY],src1,src2);
}

// TODO 1) add forwarding support
void fetch_registers(SIM_cmd *cmd) {

    p_letch[EXECUTE].src1Val = regFile[cmd->src1];
    p_letch[EXECUTE].src2Val = (cmd->isSrc2Imm) ? cmd->src2 :
					regFile[cmd->src2];
    DEBUG("DECODE: fetched registers r1:%d and r2:%d with the "
	    "values of val1:%d val2:%d\n",cmd->src1,
	    				cmd->src2,
					p_letch[EXECUTE].src1Val,
					p_letch[EXECUTE].src2Val);

}

/*! SIM_CoreReset: Reset the processor core simulator machine to start new simulation
  Use this API to initialize the processor core simulator's data structures.
  The simulator machine must complete this call with these requirements met:
  - PC = 0  (entry point for a program is at address 0)
  - All the register file is cleared (all registers hold 0)
  - The value of IF is the instuction in address 0x0
  \returns 0 on success. <0 in case of initialization failure.
*/
// TODO reset all the variables you have
int SIM_CoreReset(void) {
    pc=0;
    return 0;
}

void if_tick() {
    SIM_cmd current_command;

    SIM_MemInstRead(pc,&current_command);

    DEBUG("IF_TICK:command: %s src1: %d src2: %d dst: %d\n\n",
	    cmdStr[current_command.opcode],
	    current_command.src1,
	    current_command.src2,
	    current_command.dst);

    update_latch(&current_command,DECODE);
}

void decode_tick() {
    // we only had one command thus far
    if ( p_letch[DECODE].cmd.opcode == CMD_NOP)
	return;
    SIM_cmd *cmd = &p_letch[DECODE].cmd;

    switch(cmd->opcode) {
	// commands that don't use registers
	case CMD_BR:
	case CMD_HALT:
	    break;
	// the rest of 'em
	default: 
	    fetch_registers(cmd);
	    break;
    }

    /* we zero to command to in case of stall */
    cmd->opcode = CMD_NOP;
    update_latch(cmd,EXECUTE);
    return;
}

void exe_tick() {
    if( p_letch[EXECUTE].cmd.opcode == CMD_NOP )
	return;

    SIM_cmd *cmd = &p_letch[EXECUTE].cmd;

    switch(cmd->opcode) {
	case CMD_ADD:
	case CMD_SUB:
	case CMD_ADDI:
	case CMD_SUBI:
	    int src1 = p_letch[EXECUTE].src1Val;
	    int32_t src2 = p_letch[EXECUTE].src2Val;
	    do_arithmetic(cmd,src1,src2);
	    break;
	/* nothing to do for the other command here */
    }

    /* we zero to command to in case of stall */
    cmd->opcode = CMD_NOP;
    update_latch(cmd,MEMORY);
    return;
}

void mem_tick() {

    if( p_letch[MEMORY].cmd.opcode == CMD_NOP )
	return;
    // add implementation here
    return;
}

void wb_tick() {

    // NOP is passing through
    if( p_letch[WRITEBACK].cmd.opcode == CMD_NOP )
	return;

    return;
}

/*! SIM_CoreClkTick: Update the core simulator's state given one clock cycle.
  This function is expected to update the core pipeline given a clock cycle event.
*/
void SIM_CoreClkTick() {

    // we execute the pipe "from end to beginning"
    wb_tick();
    mem_tick();
    exe_tick();

    // stall mechanism
    if(stall_cycles > 0){
	stall_cycles--;
	return;
    }

    decode_tick();
    if_tick();

    pc+=4;
}

/*! SIM_CoreGetState: Return the current core (pipeline) internal state
    curState: The returned current pipeline state
    The function will return the state of the pipe at the end of a cycle
*/
void SIM_CoreGetState(SIM_coreState *curState) {
}

