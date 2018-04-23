/* 046267 Computer Architecture - Spring 2017 - HW #1 */
/* This file should hold your implementation of the CPU pipeline core simulator */

#include "sim_api.h"
#include <string.h>

#define WAIT_CYCLE -1
#define DEBUG(msg,args...); printf(msg,args);
//#define DEBUG(msg,args...);

// declarations
void unstall_pipe();
void stall_pipe(pipeStage stage);
void update_latch(SIM_cmd *cmd, pipeStage state);
void fetch_registers(SIM_cmd *cmd);
void flush();

int branch_compare;
int pc;
int regFile[SIM_REGFILE_SIZE];
PipeStageState p_latch[SIM_PIPELINE_DEPTH];
int32_t wb_value[SIM_PIPELINE_DEPTH];
bool stalled[SIM_PIPELINE_DEPTH];


void unstall_pipe() {
	memset(stalled, 0, SIM_PIPELINE_DEPTH * sizeof(int));
}

void stall_pipe(pipeStage stage) {
	for (int i = 0; i <= stage; i++) {
		stalled[i] = true;
	}
}

void flush() {
	for (int i = 0; i <= EXECUTE; i++) {
		p_latch[i].cmd.opcode = CMD_NOP;
	}
}

// updates a pipe's latch.
// @cmd = the command to be processed by the stage
// @ state = the state whose latch is altered
void update_latch(SIM_cmd *cmd, pipeStage state) {
    memcpy(&p_latch[state].cmd,cmd,sizeof(SIM_cmd));
}

int32_t do_minus(int32_t src1, int32_t src2) {
    return src1 - src2;
}
int32_t do_plus(int32_t src1, int32_t src2) {
    return src1 + src2;
}

void do_arithmetic(SIM_cmd *cmd,int src1,int32_t src2) {
	int (*arithmetic_func)(int,int);
	
	if(cmd->opcode == CMD_SUB || cmd->opcode == CMD_SUBI)
		arithmetic_func = &do_minus; 
	else
		arithmetic_func = &do_plus;

	wb_value[MEMORY] = arithmetic_func((int32_t)src1,src2);

	DEBUG("EXECUTE:calculated value for register %d. Its new val wiil be %d."
		" arguments: arg1:%d arg2:%d\n",cmd->dst,wb_value[MEMORY],src1,src2);
}

// TODO 1) add forwarding support
void fetch_registers(SIM_cmd *cmd) {
	wb_value[EXECUTE] = regFile[cmd->dst];
    p_latch[EXECUTE].src1Val = regFile[cmd->src1];
    p_latch[EXECUTE].src2Val = (cmd->isSrc2Imm) ? cmd->src2 : 
		regFile[cmd->src2];
    DEBUG("DECODE: fetched registers r1:%d and r2:%d with the "
	    "values of val1:%d val2:%d\n",cmd->src1,
	    				cmd->src2,
					p_latch[EXECUTE].src1Val,
					p_latch[EXECUTE].src2Val);
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
	memset(regFile, 0, SIM_REGFILE_SIZE * sizeof(int));
	memset(stalled, 0, SIM_PIPELINE_DEPTH * sizeof(bool));
	memset(wb_value, 0, SIM_PIPELINE_DEPTH * sizeof(int32_t));
	memset(p_latch, 0, SIM_PIPELINE_DEPTH * sizeof(PipeStageState));
	branch_compare = 0;
    return 0;
}

void if_tick() {
	if (stalled[FETCH])
		return;

    SIM_cmd current_command;
    SIM_MemInstRead(pc,&current_command);

    DEBUG("IF_TICK:command: %s src1: %d src2: %d dst: %d\n\n",
	    cmdStr[current_command.opcode],
	    current_command.src1,
	    current_command.src2,
	    current_command.dst);

    update_latch(&current_command,DECODE);
	pc += 4;
}

void decode_tick() {
    if (p_latch[DECODE].cmd.opcode == CMD_NOP || stalled[DECODE])
		return;
    SIM_cmd *cmd = &p_latch[DECODE].cmd;

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

    update_latch(cmd,EXECUTE);
	/* We zero to command to in case of stall */
	cmd->opcode = CMD_NOP;
    return;
}

void exe_tick() {
    if (p_latch[EXECUTE].cmd.opcode == CMD_NOP || stalled[EXECUTE])
		return;

    SIM_cmd *cmd = &p_latch[EXECUTE].cmd;

	int src1;
	int32_t src2;

	//TODO: possible addition for branch
    switch(cmd->opcode) {
	case CMD_ADD:
	case CMD_SUB:
	case CMD_ADDI:
	case CMD_SUBI:
	case CMD_LOAD:
		src1 = p_latch[EXECUTE].src1Val;
	    src2 = p_latch[EXECUTE].src2Val;
	    do_arithmetic(cmd, src1, src2);
	    break;
	case CMD_STORE:
		src1 = wb_value[EXECUTE];
		src2 = p_latch[EXECUTE].src2Val;
		do_arithmetic(cmd, src1, src2);
	case CMD_BREQ:
	case CMD_BRNEQ:
		branch_compare = p_latch[EXECUTE].src1Val - p_latch[EXECUTE].src2Val;
	case CMD_BR:
		wb_value[MEMORY] = pc + wb_value[EXECUTE];
    }

    update_latch(cmd,MEMORY);
	/* we zero to command to in case of stall */
	cmd->opcode = CMD_NOP;
    return;
}

void mem_tick() {
    if (p_latch[MEMORY].cmd.opcode == CMD_NOP)
		return;

	SIM_cmd *cmd = &p_latch[MEMORY].cmd;
	uint32_t address = wb_value[MEMORY];
	int32_t rd_val;

	switch (cmd->opcode) {
	case CMD_LOAD:
		if (SIM_MemDataRead(address, &rd_val) == WAIT_CYCLE) {
			stall_pipe(EXECUTE);
			return;
		}
		else {
			unstall_pipe();
			wb_value[WRITEBACK] = rd_val;
		}
	case CMD_STORE:
		SIM_MemDataWrite(address, p_latch[MEMORY].src1Val);
	case CMD_BREQ:
		if (branch_compare == 0) {
			goto do_branch;
		}
	case CMD_BRNEQ:
		if (branch_compare != 0) {
			goto do_branch;
		}
	case CMD_BR:
	do_branch:
		flush();
		pc = wb_value[MEMORY];
	default:
		// pass the value onward
		wb_value[WRITEBACK] = wb_value[MEMORY];
	}
	update_latch(cmd, WRITEBACK);
	cmd->opcode = CMD_NOP;
    return;
}

void wb_tick() {
    if (p_latch[WRITEBACK].cmd.opcode == CMD_NOP)
		return;
	int dst;
	SIM_cmd *cmd = &p_latch[WRITEBACK].cmd;
	switch(cmd->opcode)
	case CMD_ADD:
	case CMD_ADDI:
	case CMD_SUB:
	case CMD_SUBI:
	case CMD_LOAD:
		dst = p_latch[WRITEBACK].cmd.dst;
		if (dst != 0)
			regFile[dst] = wb_value[WRITEBACK];
}

/*! SIM_CoreClkTick: Update the core simulator's state given one clock cycle.
  This function is expected to update the core pipeline given a clock cycle event.
*/
void SIM_CoreClkTick() {

    // we execute the pipe "from end to beginning"
    wb_tick();
    mem_tick();
    exe_tick();
    decode_tick();
    if_tick();
}

/*! SIM_CoreGetState: Return the current core (pipeline) internal state
    curState: The returned current pipeline state
    The function will return the state of the pipe at the end of a cycle
*/
void SIM_CoreGetState(SIM_coreState *curState) {
	curState->pc = pc;
	memcpy(curState->regFile, regFile, SIM_REGFILE_SIZE * sizeof(int32_t));
	memcpy(curState->pipeStageState, p_latch, SIM_PIPELINE_DEPTH * sizeof(PipeStageState));
	SIM_MemInstRead(pc, &curState->pipeStageState[FETCH].cmd);
}