/* 046267 Computer Architecture - Spring 2017 - HW #1 */
/* This file should hold your implementation of the CPU pipeline core simulator */

//	When running example2.img
//
// TODO: Bug
//
//	Simulation on cycle 7. The state is:
//	PC = 0xC
//	Register file:
//		R0 = 0x0 R1 = 0x0 R2 = 0x0 R3 = 0x0 R4 = 0x0 R5 = 0x0 R6 = 0x0 R7 = 0x0 R8 = 0x0 R9 = 0x0 R10 = 0x0 
//		R11 = 0x0 R12 = 0x0 R13 = 0x0 R14 = 0x0 R15 = 0x0 R16 = 0x0 R17 = 0x0 R18 = 0x0 R19 = 0x0 R20 = 0x0 
//		R21 = 0x0 R22 = 0x0 R23 = 0x0 R24 = 0x0 R25 = 0x0 R26 = 0x0 R27 = 0x0 R28 = 0x0 R29 = 0x0 R30 = 0x0 
//		R31 = 0x0
//	Command at each pipe stage:
//		IF : BREQ $1 , $2(=0x0) , $3(=0x0)
//		ID : LOAD $3 , $1(=0x0) , 12964(=0x0)
//		EXE : NOP $0 , $0(=0x0) , $0(=0x0)
//		MEM : LOAD $2 , $0(=0x0) , 12964(=0x32A4)
//		WB : LOAD $1 , $0(=0x0) , 12960(=0x32A0)
//	MEMORY: read stall
//	DECODE: fetched registers r1:1 and r2:12964 with the values of val1:0 val2:0
//
//
//	It shouldn't be able to proceed the DECODE function (because
//	WRITEBACK hasn't wrote it yet, and there should be data_hazard
//
//
//
//	TODO: another bug:
//	it looks like you can only branch forward. I can figure out
//	the correct address they want us to branch to (if we actually
//	add PC value, that the address can only grow, and we're not
//	gonna be stuck in an infinite loop. This is something that
//	need more clarification


#include "sim_api.h"
#include <string.h>

#define WAIT_CYCLE -1
#define DEBUG( ... ); printf( __VA_ARGS__ );
//#define DEBUG(msg,args...);

// declarations
void update_latch(SIM_cmd *cmd, pipeStage *state);
void fetch_registers(SIM_cmd *cmd);
void flush();
bool is_data_hazard(SIM_cmd *cmd);
void set_nop_opcode(PipeStageState *latch);
bool find_reg_in_latch(int reg,PipeStageState &latch);

// structs
typedef enum {
	ES_OKAY,
	ES_DELAY
} PipeExectutionState;

// global vars
bool do_jump;
int branch_compare;
int pc;
int regFile[SIM_REGFILE_SIZE];
PipeStageState p_latch[SIM_PIPELINE_DEPTH];
int32_t wb_value[SIM_PIPELINE_DEPTH];


void flush() {
    for (int i = 0; i <= EXECUTE; i++) {
	set_nop_opcode(&p_latch[i]);
    }
}

void set_nop_opcode(PipeStageState *latch) {
	memset(latch , 0, sizeof(PipeStageState));
}

// Checks weather the given letch is going to change
// this register (also verifies that the latch has
// a command that modifies anything)
bool find_reg_in_latch(int reg,PipeStageState *latch) {
    bool hazard = false;

    switch( latch->cmd.opcode ) {
    case CMD_ADD:
    case CMD_SUB:
    case CMD_ADDI:
    case CMD_SUBI:
    case CMD_LOAD:
	hazard |= (reg == latch->cmd.dst);
	break;
    }
    return hazard;
}

// TODO: check if its wouldn't be better to run for EXECUTE
// to WRITEBACK (in case of forwarding, you need the newest value)
// also, maybe there is a better way to implement this (though it does
// need to know A LOT to have a good answer 
/* the function checks for data hazard */
bool is_data_hazard(SIM_cmd *cmd) {

    SIM_cmd *exe;
    bool data_hazard = false;

    for (int i=(int)EXECUTE; i<= (int)WRITEBACK && !data_hazard ;i++) {
   	 switch (cmd->opcode) {
   	     case CMD_ADD:
   	     case CMD_ADDI:
   	     case CMD_SUB:
   	     case CMD_SUBI:
   	     case CMD_LOAD:
   	         data_hazard |= find_reg_in_latch(cmd->src1, &p_latch[i]);
		 if (!cmd->isSrc2Imm)
		     data_hazard |= find_reg_in_latch(cmd->src2, &p_latch[i]);
   	         break;
   	     case CMD_STORE:
   	         data_hazard |= find_reg_in_latch(cmd->dst, &p_latch[i]);
		 if (!cmd->isSrc2Imm)
		     data_hazard |= find_reg_in_latch(cmd->src2, &p_latch[i]);
		 break;
	     case CMD_BREQ:
	     case CMD_BRNEQ:
		 data_hazard |= find_reg_in_latch(cmd->src1, &p_latch[i]);
		 data_hazard |= find_reg_in_latch(cmd->src2, &p_latch[i]);
		 /* fall through */
	     case CMD_BR:
		 data_hazard |= find_reg_in_latch(cmd->dst, &p_latch[i]);
	 }
    }

    return data_hazard;
}

// updates a pipe's latch.
// @cmd = the command to be processed by the stage
// @ state = the state whose latch is altered
void update_latch(PipeStageState *dst_latch,PipeStageState *src_latch) {
    memcpy(dst_latch,src_latch,sizeof(PipeStageState));
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
    p_latch[DECODE].src1Val = regFile[cmd->src1];
    p_latch[DECODE].src2Val = (cmd->isSrc2Imm) ? cmd->src2 :
		regFile[cmd->src2];
    wb_value[EXECUTE] = regFile[cmd->dst];

    DEBUG("DECODE: fetch values src1:0x%X src2:0x%X dst:%X\n",p_latch[DECODE].src1Val,
	    						p_latch[DECODE].src2Val,
							wb_value[EXECUTE]);
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
    memset(wb_value, 0, SIM_PIPELINE_DEPTH * sizeof(int32_t));
    memset(p_latch, 0, SIM_PIPELINE_DEPTH * sizeof(PipeStageState));
    branch_compare = 0;
    return 0;
}

PipeExectutionState if_tick() {

    SIM_cmd *current_command = &p_latch[FETCH].cmd;
    SIM_MemInstRead(pc,current_command);

    DEBUG("FETCH: command: %s src1: %d src2: %d dst: %d\n\n",
	    cmdStr[current_command->opcode],
	    current_command->src1,
	    current_command->src2,
	    current_command->dst);

    update_latch(&p_latch[DECODE],&p_latch[FETCH]);
    pc += 4;
    return ES_OKAY;
}

// TODO: add support for forwarding here
// 	and fix the bug you have here (register file isn't updated
PipeExectutionState decode_tick() {
    if (p_latch[DECODE].cmd.opcode == CMD_NOP)
		return ES_OKAY;
    SIM_cmd *cmd = &p_latch[DECODE].cmd;

    switch(cmd->opcode) {
	// commands that don't use registers
	case CMD_HALT:
	    break;
	// the rest of 'em
	default:
	    if (is_data_hazard(cmd)) {
		DEBUG("DECODE: data hazard detected\n");
		return ES_DELAY;
	    }

	    fetch_registers(cmd);
	    break;
    }

    update_latch(&p_latch[EXECUTE],&p_latch[DECODE]);
    /* We zero to command to in case of stall */
    set_nop_opcode(&p_latch[DECODE]);
    return ES_OKAY;
}

PipeExectutionState exe_tick() {
    if (p_latch[EXECUTE].cmd.opcode == CMD_NOP )
		return ES_OKAY;

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
		break;
	case CMD_BREQ:
	case CMD_BRNEQ:
		branch_compare = p_latch[EXECUTE].src1Val - p_latch[EXECUTE].src2Val;
	case CMD_BR:
		wb_value[MEMORY] = pc + wb_value[EXECUTE];
    }

    update_latch(&p_latch[MEMORY],&p_latch[EXECUTE]);
    /* we zero to command to in case of stall */
    set_nop_opcode(&p_latch[EXECUTE]);
    return ES_OKAY;
}

PipeExectutionState mem_tick() {
	do_jump = false;
    if (p_latch[MEMORY].cmd.opcode == CMD_NOP)
		return ES_OKAY;

	SIM_cmd *cmd = &p_latch[MEMORY].cmd;
	uint32_t address = wb_value[MEMORY];
	int32_t rd_val;

	switch (cmd->opcode) {
	case CMD_LOAD:
		if (SIM_MemDataRead(address, &rd_val) == WAIT_CYCLE) {
			DEBUG("MEMORY: read stall\n");
			// in case the previous stage is NOP, there is
			// no point in stalling the pipe
			return (p_latch[EXECUTE].cmd.opcode == CMD_NOP) ? 
			    ES_OKAY : ES_DELAY;
		}
		DEBUG("MEMORY: value was read from address %X!\n",address);
		wb_value[WRITEBACK] = rd_val;
		break;
	case CMD_STORE:
		SIM_MemDataWrite(address, p_latch[MEMORY].src1Val);
		break;
	case CMD_BREQ:
		if (branch_compare == 0) {
			goto do_branch;
		}
		break;
	case CMD_BRNEQ:
		if (branch_compare != 0) {
			goto do_branch;
		}
		break;
	case CMD_BR:
	do_branch:
		do_jump = true;
		flush();
		// We use (value - 4) because the branch is calculated at EXECUTE stage,
		// while we need pc at DECODE STAGE
		pc = wb_value[MEMORY] - 4;
		DEBUG("MEMORY: branch taken to address 0x%X\n",pc);
		break;
	default:
		// pass the value onward
		wb_value[WRITEBACK] = wb_value[MEMORY];
	}
    	update_latch(&p_latch[WRITEBACK],&p_latch[MEMORY]);
	set_nop_opcode(&p_latch[MEMORY]);
	return ES_OKAY;
}

PipeExectutionState wb_tick() {
    if (p_latch[WRITEBACK].cmd.opcode == CMD_NOP)
		return ES_OKAY;
	int dst;
	SIM_cmd *cmd = &p_latch[WRITEBACK].cmd;
	switch(cmd->opcode) {
	case CMD_ADD:
	case CMD_ADDI:
	case CMD_SUB:
	case CMD_SUBI:
	case CMD_LOAD:
	    dst = p_latch[WRITEBACK].cmd.dst;
	    if (dst != 0)
		regFile[dst] = wb_value[WRITEBACK];
	}

	set_nop_opcode(&p_latch[WRITEBACK]);
	return ES_OKAY;
}

/*! SIM_CoreClkTick: Update the core simulator's state given one clock cycle.
  This function is expected to update the core pipeline given a clock cycle event.
*/
void SIM_CoreClkTick() {

    // we execute the pipe "from end to beginning"
    wb_tick();

    if(mem_tick() == ES_DELAY | do_jump)
	return;

    exe_tick();

    if(decode_tick() == ES_DELAY)
	return;
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
