/* 046267 Computer Architecture - Spring 2017 - HW #1 */
/* This file should hold your implementation of the CPU pipeline core simulator */

#include "sim_api.h"
#include <string.h>

#define WAIT_CYCLE -1
#define AFTER_WB (WRITEBACK + 1)
#define DEBUG( ... );  //printf( __VA_ARGS__ );
#define DEBUG2( ... ); //printf( __VA_ARGS__ );
// declarations
void update_latch(PipeStageState *dst_latch,PipeStageState *src_latch);
void fetch_registers(SIM_cmd *cmd);
void flush();
bool is_data_hazard(SIM_cmd *cmd);
void set_nop_opcode(PipeStageState *latch);
bool find_reg_in_latch(int reg, PipeStageState *latch);
void update_src_regs_w_split_regs(SIM_cmd *cmd,int reg_val);
int is_contained_in_latch(PipeStageState *state,int reg_num);
void find_reg_and_replace(int reg_num,int *reg_val, pipeStage stage);

// structs
typedef enum {
	ES_OKAY,
	ES_DELAY
} PipeExectutionState;

// global vars
int branch_compare;
int pc;
int regFile[SIM_REGFILE_SIZE];
// We add an extra stage becuase when we reach decode we need
// information that WB no longer has.
PipeStageState p_latch[SIM_PIPELINE_DEPTH + 1];
int32_t wb_value[SIM_PIPELINE_DEPTH + 1];


void flush() {
	for (int i = 0; i <= EXECUTE; i++) {
		set_nop_opcode(&p_latch[i]);
	}
}

void set_nop_opcode(PipeStageState *latch) {
	memset(latch, 0, sizeof(PipeStageState));
}

void update_src_regs_w_split_regs(SIM_cmd *cmd,int reg_val) {

	switch(cmd->opcode) {
	case CMD_ADD:
	case CMD_ADDI:
	case CMD_SUB:
	case CMD_SUBI:
	case CMD_LOAD:
	    regFile[cmd->dst]=reg_val;
	}
}

// Checks whether the given latch is going to change
// this register (also verifies that the latch has
// a command that modifies anything)
bool find_reg_in_latch(int reg, PipeStageState *latch) {
	bool hazard = false;

	switch (latch->cmd.opcode) {
	case CMD_ADD:
	case CMD_SUB:
	case CMD_ADDI:
	case CMD_SUBI:
	case CMD_LOAD:
		hazard |= (reg!=0 && reg == latch->cmd.dst);
		break;
	}
	return hazard;
}

/* the function checks for data hazards */
bool is_data_hazard(SIM_cmd *cmd) {

	bool data_hazard = false;
	int stage;

	/*
	*****   COMMANDS THAT CANNOT BE AVOIDED *****
	--- LOAD at EXECUTE

	*****   COMMANDS THAT SPLIT_REG CORRECTS *******
	--- Any command at WRITEBACK

	*****   COMMANDS THAT FORWARD CORRECTS ******
	--- As far as I could find, anything else really...
	*/

	// When we reach HDU, all information already passed 1 cycle.
	for (int i= (int)EXECUTE; i <= (int)AFTER_WB && !data_hazard; i++) {
		switch (cmd->opcode) {
		case CMD_ADD:
		case CMD_ADDI:
		case CMD_SUB:
		case CMD_SUBI:
		case CMD_LOAD:
			data_hazard |= find_reg_in_latch(cmd->src1, &p_latch[i]);
			if (!cmd->isSrc2Imm)
				data_hazard |= find_reg_in_latch(cmd->src2, &p_latch[i]);

			if(pc == 0xC && i==WRITEBACK) {
			    DEBUG("The data hazard value is: %d\n",data_hazard);
			    DEBUG("Checked data hazard for registers number %d and %d\n",cmd->src1,cmd->src2);
			    DEBUG("Content of WB latch is: src1: %d src2: %d dst: %d\n",p_latch[i].cmd.src1,
				    							p_latch[i].cmd.src2,
											p_latch[i].cmd.dst);
			}
			break;
		case CMD_STORE:
			data_hazard |= find_reg_in_latch(cmd->dst, &p_latch[i]);
			data_hazard |= find_reg_in_latch(cmd->src1, &p_latch[i]);
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
		stage = i;
	}

	// stage is (+1) because of implementation...
	if (split_regfile && stage == (int)AFTER_WB && data_hazard) {
		DEBUG2("\nHazard reg is %d", hazard_reg);
		DEBUG2("\nReg that was splited is %d", wb_value[AFTER_WB]);
		data_hazard = false;
	}

	if (forwarding && stage == MEMORY && data_hazard &&
		p_latch[MEMORY].cmd.opcode == CMD_LOAD) {}
	else if(forwarding && data_hazard) {
		DEBUG2("\nForwarding fixes the hazard.\n");
		data_hazard = false;
	}

    return data_hazard;
}


void find_reg_and_replace(int reg_num,int *reg_val, pipeStage stage) {

	switch (p_latch[stage].cmd.opcode) {
	case CMD_ADD:
	case CMD_SUB:
	case CMD_ADDI:
	case CMD_SUBI:
	case CMD_LOAD:
	    if(reg_num == p_latch[stage].cmd.dst) {
		*reg_val = wb_value[stage];
	    	DEBUG("DECODE: found collision in stage: %d with reg_num: %d\n",stage,
		    p_latch[stage].cmd.dst);
	    }
	    break;
	}

}

void perform_forwarding() {
	SIM_cmd *cmd = &p_latch[EXECUTE].cmd;
	int *src1Val,*src2Val, *dst_val;
	int src1_num,src2_num,dst_num;

	src1_num = p_latch[EXECUTE].cmd.src1;
	src2_num = p_latch[EXECUTE].cmd.src2;
	dst_num = p_latch[EXECUTE].cmd.dst;

	src1Val = &p_latch[EXECUTE].src1Val;
	src2Val = &p_latch[EXECUTE].src2Val;
	dst_val = &wb_value[EXECUTE];


	for (int i= (int)WRITEBACK; i >= (int)MEMORY; i--) {
	    switch (cmd->opcode) {
		case CMD_ADD:
		case CMD_ADDI:
		case CMD_SUB:
		case CMD_SUBI:
		case CMD_LOAD:
		    find_reg_and_replace(src1_num,src1Val, (pipeStage)i);
		    if (!cmd->isSrc2Imm)
			find_reg_and_replace(src2_num,src2Val, (pipeStage)i);

		    break;
		case CMD_STORE:
		    find_reg_and_replace(dst_num,dst_val, (pipeStage)i);
		    find_reg_and_replace(src1_num,src1Val, (pipeStage)i);
		    if (!cmd->isSrc2Imm)
			find_reg_and_replace(src2_num,src2Val, (pipeStage)i);
		    break;
		case CMD_BREQ:
		case CMD_BRNEQ:
		    find_reg_and_replace(src1_num,src1Val, (pipeStage)i);
		    find_reg_and_replace(src2_num,src2Val, (pipeStage)i);
		    /* fall through */
		case CMD_BR:
		    find_reg_and_replace(dst_num,dst_val, (pipeStage)i);
		}
	}
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

void fetch_registers(SIM_cmd *cmd) {


	p_latch[DECODE].src1Val = regFile[cmd->src1];

	p_latch[DECODE].src2Val = (cmd->isSrc2Imm) ? cmd->src2 :
		    regFile[cmd->src2];

	wb_value[EXECUTE] = regFile[cmd->dst];

	DEBUG("DECODE: fetch values src1:0x%X src2:0x%X dst:%X\n", p_latch[DECODE].src1Val,
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
	memset(wb_value, 0, (SIM_PIPELINE_DEPTH + 1) * sizeof(int32_t));
	memset(p_latch, 0, (SIM_PIPELINE_DEPTH + 1) * sizeof(PipeStageState));
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
	wb_value[DECODE] = 0;

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

	if (forwarding) 
	    perform_forwarding();

	set_nop_opcode(&p_latch[DECODE]);
	return ES_OKAY;
}

PipeExectutionState exe_tick() {
	if (p_latch[EXECUTE].cmd.opcode == CMD_NOP )
		return ES_OKAY;

	SIM_cmd *cmd = &p_latch[EXECUTE].cmd;

	int src1;
	int32_t src2;

	switch (cmd->opcode) {
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
		break;
	}

	update_latch(&p_latch[MEMORY],&p_latch[EXECUTE]);
	/* we zero command in case of stall */
	set_nop_opcode(&p_latch[EXECUTE]);
	return ES_OKAY;
}

PipeExectutionState mem_tick() {
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
		flush();
		// We use (value - 4) because the branch is calculated at EXECUTE stage,
		// while we need pc at the DECODE stage
		pc = wb_value[MEMORY] - 4;
		DEBUG("MEMORY: branch taken to address 0x%X\n",pc);

		update_latch(&p_latch[WRITEBACK],&p_latch[MEMORY]);
		set_nop_opcode(&p_latch[MEMORY]);
		return ES_DELAY;
		break;
	default:
		// pass the value onward
		wb_value[WRITEBACK] = wb_value[MEMORY];
	}

	// if split_reg the MEM_STAGE updates DECODE latch
	if(split_regfile)
	    update_src_regs_w_split_regs(cmd,wb_value[WRITEBACK]);

	update_latch(&p_latch[WRITEBACK],&p_latch[MEMORY]);

	set_nop_opcode(&p_latch[MEMORY]);
	return ES_OKAY;
}

PipeExectutionState wb_tick() {

	int dst = -1;
	SIM_cmd *cmd = &p_latch[WRITEBACK].cmd;
	switch(cmd->opcode) {
	case CMD_ADD:
	case CMD_ADDI:
	case CMD_SUB:
	case CMD_SUBI:
	case CMD_LOAD:
	    dst = p_latch[WRITEBACK].cmd.dst;
	    if (dst != 0) {
		regFile[dst] = wb_value[WRITEBACK];
	    }
	}

	wb_value[AFTER_WB] = dst;
	update_latch(&p_latch[AFTER_WB], &p_latch[WRITEBACK]);

	set_nop_opcode(&p_latch[WRITEBACK]);
	return ES_OKAY;
}

/*! SIM_CoreClkTick: Update the core simulator's state given one clock cycle.
  This function is expected to update the core pipeline given a clock cycle event.
*/
void SIM_CoreClkTick() {
	// we execute the pipe "from end to beginning"

	wb_tick();

	if(mem_tick() == ES_DELAY)
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
