/* 046267 Computer Architecture - Spring 2017 - HW #1 */
/* This file should hold your implementation of the CPU pipeline core simulator */

#include "sim_api.h"
#include <string.h>

#define WAIT_CYCLE -1
#define DEBUG( ... ); //printf( __VA_ARGS__ );
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
int wb_reg;
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

// Checks whether the given latch is going to change
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

// TODO: check if it wouldn't be better to run for EXECUTE
// to WRITEBACK (in case of forwarding, you need the newest value)
// also, maybe there is a better way to implement this (though it does
// need to know A LOT to have a good answer 

// TODO: added some lines to the cases, but didn't check to see if it's
// possible to improve switch layout.

/* the function checks for data hazards */
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
			 data_hazard |= (cmd->src1 == wb_reg);
   	         data_hazard |= find_reg_in_latch(cmd->src1, &p_latch[i]);
		 if (!cmd->isSrc2Imm)
			 data_hazard |= (cmd->src2 == wb_reg);
		     data_hazard |= find_reg_in_latch(cmd->src2, &p_latch[i]);
   	         break;
   	     case CMD_STORE:
			 data_hazard |= (cmd->src1 == wb_reg);
			 data_hazard |= (cmd->dst == wb_reg);
   	         data_hazard |= find_reg_in_latch(cmd->dst, &p_latch[i]);
			 data_hazard |= find_reg_in_latch(cmd->src1, &p_latch[i]);
		 if (!cmd->isSrc2Imm)
			 data_hazard |= (cmd->src2 == wb_reg);
		     data_hazard |= find_reg_in_latch(cmd->src2, &p_latch[i]);
		 break;
	     case CMD_BREQ:
	     case CMD_BRNEQ:
			 data_hazard |= (cmd->src1 == wb_reg);
			 data_hazard |= (cmd->src2 == wb_reg);
			 data_hazard |= find_reg_in_latch(cmd->src1, &p_latch[i]);
			 data_hazard |= find_reg_in_latch(cmd->src2, &p_latch[i]);
			 /* fall through */
		 case CMD_BR:
			 data_hazard |= (cmd->dst == wb_reg);
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

	DEBUG("EXECUTE:calculated value for register %d. Its new val will be %d."
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

	// TODO: those two lines contradict themselves
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
    /* We zero command in case of stall */
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
			return ES_DELAY;

			/* 
			   According to sample output, next code segment is not correct.
			   Stalls should stop *ALL* stages of the pipe (ex. WB).
			*/

			/*
			// in case the previous stage is NOP, there is
			// no point in stalling the pipe
			return (p_latch[EXECUTE].cmd.opcode == CMD_NOP) ? 
				ES_OKAY : ES_DELAY;
			*/
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
	wb_reg = -1; // to prevent wierd hazard with r0
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
		wb_reg = dst;
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

	// These don't actually have any meaning other 
	// than to compare to the TA's output
	SIM_cmd *current_command = &p_latch[FETCH].cmd;
	p_latch[DECODE].src1Val = regFile[current_command->src1];
	p_latch[DECODE].src2Val = (current_command->isSrc2Imm) ? current_command->src2 :
		regFile[current_command->src2];

	memcpy(curState->regFile, regFile, SIM_REGFILE_SIZE * sizeof(int32_t));
	memcpy(curState->pipeStageState, p_latch, SIM_PIPELINE_DEPTH * sizeof(PipeStageState));
	SIM_MemInstRead(pc, &curState->pipeStageState[FETCH].cmd);
}
