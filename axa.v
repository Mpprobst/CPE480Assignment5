// Basic bit definitions
`define DATA                     [15:0]
`define ADDRESS                  [15:0]
`define SIZE                        [65535:0]
`define WORD                   [15:0]
`define WHIGH                  [15:8]
`define WLOW                   [7:0]
`define INSTRUCTION    [15:0]
`define OP                           [15:10]
`define OP8                        [15:12]
`define OPPUSH                [14]
`define OP8IMM                              [15]
`define SRCTYPE                               [9:8]
`define DESTREG                              [3:0]
`define SRCREG                 [7:4]
`define SRCREGMSB        [7]
`define SRC8                       [11:4]
`define SRC8MSB              [11]
`define STATE                    [6:0]
`define REGS                      [15:0]
`define OPERATION_BITS              [6:0]
`define REGSIZE                [15:0]
`define USIZE [15:0]

//Op values
`define OPsys                                                                    6'b000000
`define OPcom                                                                  6'b000001
`define OPadd                                                                   6'b000010
`define OPsub                                                                   6'b000011
`define OPxor                                                                    6'b000100
`define OPex                                                                      6'b000101
`define OProl                                                                     6'b000110
`define OPbzjz                                                                   6'b001000
`define OPbnzjnz                                                              6'b001001
`define OPbnjn                                                                 6'b001010
`define OPbnnjnn                                                            6'b001011
`define OPjerr                                                                   6'b001110
`define OPfail                                                                     6'b001111
`define OPland                                                                  6'b010000
`define OPshr                                                                    6'b010001
`define OPor                                                                      6'b010010
`define OPand                                                                   6'b010011
`define OPdup                                                                   6'b010100
`define OPxhi                                                                     6'b100000
`define OPxlo                                                                     6'b101000
`define OPlhi                                                                      6'b110000
`define OPllo                                                                      6'b111000
`define OPnop                                                                   6'b111100

// Checks
//            8-bit
`define OPxhiCheck                                                        4'b1000
`define OPxloCheck                                                        4'b1010
`define OPlhiCheck                                                          4'b1100
`define OPlloCheck                                                          4'b1110

//            SrcType
`define SrcTypeRegister                                                2'b00
`define SrcTypeI4                                                             2'b01
`define SrcTypeMem                                                      2'b10
`define SrcTypeI4Undo                                                  2'b11

//State values
`define Start                                                                       7'b1000000
`define Decode                                                                 7'b1100000
`define Decode2                                                               7'b1100001
`define DecodeI8                                                              7'b1100010
`define Nop                                                                        7'b1000010
`define SrcType                                                                 7'b1001000
`define SrcRegister                                                          7'b1001001
`define SrcI4                                                                       7'b1001010
`define SrcI8                                                                       7'b1001011
`define SrcMem                                                                7'b1001100
`define Done 						6'b111101

//`define NOP         	  16'b0

`define LINEADDR 				[15:0]
`define LINE 						[15:0]
`define LINES 					[65535:0]
`define MEMDELAY 				4

// Cache values
`define CACHE_LINES			[7:0]
`define LINE_SIZE				[35:0]
`define LINE_INIT				[35]
`define TRAN						[34]
`define USED						[33]
`define DIRTY						[32]
`define LINE_MEMORY			[31:16]
`define LINE_VALUE			[15:0]

// cache states
`define CACHE_STANDBY			4'b0000		// cache do nothing
`define CHECK_HIT					4'b0001		// is there a hit in the cache
`define HIT								4'b0010		// there is a hit, return value to core
`define MISS							4'b0011		// find open cache line
`define REWIND						4'b0100		// check if used bits are 1, if all are, reset them to 0
`define READ							4'b0101		// fill an empty cache line from slowmem
`define FLUSH							4'b0111 	// if no empty lines, flush one


//******************************************************
//*						     	  	SLOWMEM16			  							 *
//******************************************************

module slowmem16(rdy, rdata, addr, wdata, wtoo, strobe, clk);
output reg rdy = 0;
output reg `LINE rdata;

input `LINEADDR addr;
input `LINE wdata;
input wtoo, strobe, clk;
reg [7:0] busy = 0;
reg `LINEADDR maddr;
reg mwtoo;
reg `LINE mwdata;
reg `LINE m `LINES;

initial begin
	$readmemh1(m); //Data
end

always @(posedge clk) begin
  if (busy == 1) begin
    // complete request
    rdata <= m[maddr];
    if (mwtoo) m[maddr] <= mwdata;
    busy <= 0;
    rdy <= 1;
  end else if (busy > 1) begin
    // still waiting
    busy <= busy - 1;
  end else if (strobe) begin
    // idle and new request
    rdata <= 16'hxxxx;
    maddr <= addr;
    mwdata <= wdata;
    mwtoo <= wtoo;
    busy <= `MEMDELAY;
    rdy <= 0;
  end else begin rdy <= 0; end
end
endmodule

//******************************************************
//*										  	ARBITER											 *
//******************************************************

module arbiter(mem_out, val_out, wdata, wtoo, addr, mem_rdy, mem_strobe, mem_in, rdy, val_in, rdata, write, clk);

output reg `DATA val_out, wdata;
output reg `ADDRESS mem_out;
output reg `ADDRESS addr;									// mem_out tells the caches which memory has been modified in memory so their values can be updated
output reg mem_rdy, mem_strobe, wtoo;								// strobe = 0 when transaction is occuring

input `DATA val_in, rdata;
input `ADDRESS mem_in;
input rdy, write, clk;

reg arb_rdy = 0;

always @(posedge clk) begin

// also check here if in a transaction
if (mem_in !== 16'bx) begin
	arb_rdy = 1;
end else begin
	arb_rdy = 0;
end

if (arb_rdy) begin
	$display("arbiter ready");
	mem_strobe <= 1;
	wtoo <= write;																// write is 1 if core is doing a write operation, wtoo tells slowmem to write a value in memory. if 0, read value.
	addr <= mem_in;																// mem_in is memory address from core that is being accessed in slowmem
	wdata <= val_in;															// val_in is the new value sent from updated cache to be written in slowmem
end else begin
// do nothing
	mem_strobe <= 0;
end

mem_rdy <= rdy;																// mem_rdy tells core that the slowmem has completed its operation

if (rdy) begin
	mem_out <= addr;														// address to be updated in cache is the same as the address modified in slowmem
	val_out <= rdata;															// val_out is value received from slowmem that is being sent to the cache
	arb_rdy <= 0;																	// arbiter is done, so now it can wait for another transaction
	mem_strobe <= 0;
	$display("mem is rdy");
end

end

endmodule

//******************************************************
//*												CORE							 					 *
//******************************************************

module core(halt, val_in, mem_in, write, mem_rdy, val_out, mem_out, reset, clk);
output reg halt, write;
output reg `DATA val_in;
output reg `ADDRESS mem_in;

input mem_rdy, reset, clk;
input `DATA val_out;
input `ADDRESS mem_out;

reg `DATA reglist `REGSIZE;  //register file
reg `DATA datamem `SIZE;  //data memory
reg `INSTRUCTION instrmem `SIZE;  //instruction memory
reg `DATA pc,tpc;
reg `DATA passreg;   //This is the temp register to hold the source NOTE: src is used in stage 3, is this needed?
reg `INSTRUCTION ir0, ir1, ir2, ir3, ir_stalled; //instruction registers for each stage
reg jump;  //is jump or not
reg branch; //is branch or not
reg land;
reg `ADDRESS target;
reg wait1;    //check to make sure stage 2 is caught up
reg `STATE s, sLA;
reg `OP op4; // opcode for stage 4
reg `DATA des,des1, src,src1,src2, res;

reg `DATA usp;  //This is how we will index through undo buffer
reg `DATA u `USIZE;  //undo stack

reg query_cache;				// when query_cache = 1, the core is requesting a value from cache

// cache registers

reg `LINE_SIZE cache_data `CACHE_LINES;
reg signed [3:0] hit, replace;
wire mem_rdy;
wire `DATA rdata, wdata;
reg strobe;

wire `DATA cache_val;   // value returned from the cache
reg `DATA cache_mem; // memory address to search for in the cache.
wire cache_ready;

reg [3:0] cache_state;

always @(reset) begin
                halt = 0;
                pc = 0;	// set to 0x8000 for core 2
                usp=0;
                ir1= `OPnop;
                ir2= `OPnop;
                ir3= `OPnop;
								ir_stalled = `OPnop;
                op4 = `OPnop;
                des = 0;
                src = 0;
                jump=0;
                branch=0;
                land=0;
                res = 0;
								query_cache = 0;
								cache_state = `CACHE_STANDBY;
//Setting initial values
                $readmemh0(reglist); //Registers
                $readmemh2(instrmem); //Instructions
end

function setsdes;
                input `INSTRUCTION inst;
                setsdes = (((inst`OP >= `OPadd) && (inst `OP <= `OProl)) ||
                                                                ((inst`OP >= `OPshr) && (inst `OP <= `OPdup)) ||
                                                                (inst `OP == `OPxhi) ||
                                                                (inst `OP == `OPxlo) ||
                                                                (inst `OP == `OPlhi) ||
                                                                (inst `OP == `OPllo)
                                                                );
endfunction
function usesdes;
                input `INSTRUCTION inst;
                usesdes = (((inst`OP >= `OPadd) && (inst `OP <= `OProl)) ||
                                                                ((inst`OP >= `OPshr) && (inst `OP <= `OPdup)) ||
                                                                ((inst`OP >= `OPbzjz) && (inst `OP <= `OPbnnjnn)) ||
                                                                (inst `OP == `OPxhi) ||
                                                                (inst `OP == `OPxlo)
                                                                );
endfunction

function usessrc;
                input `INSTRUCTION inst;
                usessrc = ((((inst`OP >= `OPadd) && (inst `OP <= `OProl)) ||
                                                                ((inst`OP >= `OPshr) && (inst `OP <= `OPdup))) && (inst `SRCTYPE == `SrcRegister));
endfunction


//start pipeline
assign pendpc = (setsdes(ir1) || setsdes(ir2) || setsdes(ir3));


//Stage1: Fetch
always @(posedge clk) begin
                if(jump) begin
                                tpc = target;
                                jump <=0;
                end else if(branch) begin
                                tpc = pc + src-1;
                                branch <= 0;
                end else begin
                                tpc = pc;
                end

                if(land) begin
                                u[usp] =tpc;
                                usp = usp+1;
                                land =0;
                end


                if(wait1) begin
                                pc<= tpc;
                end else begin
                                if(pendpc) begin
                                                ir1 <= `OPnop;
                                                pc <= tpc;
                end else begin
									if (!query_cache) begin
                                ir0 = instrmem[tpc];
                                ir1<= ir0;
                                pc <= tpc+1;
									end else begin
										ir0 <= `OPnop;
										ir1 <= `OPnop;
									end
                end
end


end //always block

//stage2: register read
always @(posedge clk) begin
                if(query_cache || branch || jump || (ir1 != `OPnop) && setsdes(ir2) && ((usesdes(ir1) && (ir1 `DESTREG == ir2 `DESTREG)) || (usessrc(ir1) && (ir1 `SRCREG == ir2 `DESTREG)))) begin
																wait1 = 1;
																//$display("wait bc query = %d branch = %d", query_cache, branch);
                                ir2 <= `OPnop;
                end else begin
                                wait1 = 0;
                                des1 <= reglist[ir1 `DESTREG];
                                if(ir1 `OP8IMM == 1'b0) begin
                                                case(ir1 `SRCTYPE)
                                                                `SrcTypeRegister: begin src2 <= reglist[ir1 `SRCREG]; end
                                                                `SrcTypeI4Undo: begin src2 <= ir1 `SRCREG; end
                                                                `SrcTypeI4: begin src2 <= ir1 `SRCREG; end
                                                                default: begin end
                                                endcase
                                end else begin
                                                src2 <= ir1 `SRC8;
                                end
                                if(ir1`OPPUSH) begin
                                                //NEEDS TO PUSH des TO UNDO BUFFER
                                                u[usp] = ir1 `DESTREG;
                                                usp = usp+1;
                                end
                                ir2 <= ir1;
                end
end

//stage3: Data memory
always @(posedge clk) begin //should handle selection of source?
                if(ir2 == `OPnop) begin
									if (!query_cache) begin
																	ir3 <= `OPnop;
											end

                end else if (!jump) begin
                              	if(ir2 `SRCTYPE == `SrcTypeMem) begin								// if the src is a memory address, get it from cache.
																	if (!query_cache) begin
	                                  	// check this PE's cache
	                                  	cache_mem <= ir2 `SRCREG;                       // send new memory address to cache
																			$display("cache queried");
																			write <= 0;																			// write is only 1 when flushing a dirty line, or during an ex instruction
	                          					query_cache = 1;
																			if (branch || jump) begin
																				ir_stalled <= `OPnop;
																			end else begin
																				ir_stalled <= ir2;
																			end
																			ir3 <= `OPnop;
	  																	// check the other PE's cache
																	end else begin
																		//ir3 <= `OPnop;
																	end
                                end else begin
                                	src <= src2;
																	des<=des1;
								                	ir3 <= ir2;
                                end
                end else begin
									ir3 <= `OPnop;
								end
end

// stage4: execute and write
always @(posedge clk) begin
                if (ir3 == `OPnop) begin
                                jump <= 0;
																branch <= 0;
                end else if (!jump && !branch) begin
                                op4 = ir3 `OP;

                                case(op4)

                                `OPxlo: begin res = {des`WHIGH, src`WLOW ^ des`WLOW}; end
                                `OPxhi: begin res = {src`WLOW ^ des`WHIGH  , des`WLOW}; end
                                `OPllo: begin res = {{8{src[7]}}, src}; op4 <=`OPnop; end
                                `OPlhi: begin res = {src, 8'b0}; end
                                `OPand: begin res = des & src; $display("ADD: %d + %d = %d", des, src, res); end
                                `OPor:  begin res = des | src; $display("OR: %d + %d = %d", des, src, res); end
                                `OPxor: begin res = des ^ src; $display("XOR: %d + %d = %d", des, src, res); end
                                `OPadd: begin res = des + src; $display("ADD: %d + %d = %d", des, src, res); end
                                `OPsub: begin res = des - src; $display("SUB: %d - %d = %d", des, src, res); end
                                `OProl: begin res = (des << src) | (des >> (16 - src)); end
                                `OPshr: begin res = des >> src; end
                                `OPbzjz: begin $display("is %d zero?", des); if(des==0) begin $display("yes");
                                                if(ir3 `SRCTYPE == 2'b01) begin
                                                                branch<=1;
                                                end else begin
                                                                jump<=1;
                                                                target<= src;
                                                end
                                                end
                                end

                                `OPbnzjnz: begin if(des!=0) begin
                                                if(ir3 `SRCTYPE == 2'b01) begin
                                                                branch<=1;
                                                end else begin
                                                                jump<=1;
                                                                target<=src;
                                                end
                                                end
                                end

                                `OPbnjn: begin if(des[15]==1) begin
                                                if(ir3 `SRCTYPE == 2'b01) begin
                                                                branch<=1;
                                                end else begin
                                                                jump<=1;
                                                                target<=src;
                                                end
                                                end
                                end

                                `OPbnnjnn: begin if(des[15]==0) begin
                                                if(ir3 `SRCTYPE == 2'b01) begin
                                                                branch<=1;
                                                end else begin
                                                                jump<=1;
                                                                target<=src;
                                                end
                                                end
                                end

                                `OPdup: begin res = src; end
                                `OPex: begin res <= src; datamem[reglist[ir3 `SRCREG]] <= des; end
                                `OPfail: begin if (!jump && !branch) begin // fail after a branch still gets executed. this prevents the fail in those cases
                                                halt <= 1;
                        end
                        end
                                `OPsys: begin if (!jump && !branch) begin // sys after a branch still gets executed. this prevents the fail in those cases
                        $display("sys call");
                                                halt <= 1;
                        end
                        end
                 default: begin
                                                halt <= 1;
                end
                                endcase


                                if(setsdes(ir3) && !jump && !branch) begin // check if we are ready to set the des
                                                reglist[ir3 `DESTREG] = res;
                                                jump <= 0;
                                end
                end
								else begin
									ir3 <= `OPnop;
								end
end //  always

// cache block


always @(posedge clk) begin

													// CACHE LOGIC

																	case (cache_state)

																	// Cache is standing by until the processor needs a value from it.
																	// When cache is no longer standing by, it immediately checks if it has the desired mem address
																		`CACHE_STANDBY: begin
																		$display("standby");
																			if (query_cache && cache_mem !== 16'bx) begin

																				$display("looking for address: %d ", cache_mem);
																				// check if any line in the cache is the designated memory address, then store cache line index in hit
																				if (cache_data[0]`LINE_MEMORY == cache_mem) begin hit <= 0; $display("hit on 0"); end else
																				if (cache_data[1]`LINE_MEMORY == cache_mem) begin hit <= 1; $display("hit on 1"); end else
																				if (cache_data[2]`LINE_MEMORY == cache_mem) begin hit <= 2; $display("hit on 2"); end else
																				if (cache_data[3]`LINE_MEMORY == cache_mem) begin hit <= 3; $display("hit on 3"); end else
																				if (cache_data[4]`LINE_MEMORY == cache_mem) begin hit <= 4; $display("hit on 4"); end else
																				if (cache_data[5]`LINE_MEMORY == cache_mem) begin hit <= 5; $display("hit on 5"); end else
																				if (cache_data[6]`LINE_MEMORY == cache_mem) begin hit <= 6; $display("hit on 6"); end else
																				if (cache_data[7]`LINE_MEMORY == cache_mem) begin hit <= 7; $display("hit on 7"); end else begin hit <= -1; $display("no hit"); end
																				cache_state <= `CHECK_HIT;
																			end

																		end

																	// Check the value of the hit register, then handle it appropriately
																		`CHECK_HIT: begin

																			$display("checking hit. hit = %d", hit);

																			if(hit>=0) begin
																				$display("hit");
																	// tells arbiter to not read a value from memory
																				mem_in = 16'bx;
																				cache_data[hit]`USED = 1;
																				cache_data[hit]`LINE_INIT = 1;
																				src <= cache_data[hit]`LINE_VALUE;

																				// reset all used bits to 0 if all are 1
																				if (cache_data[0]`USED == 0) begin end else
																				if (cache_data[1]`USED == 0) begin end else
																				if (cache_data[2]`USED == 0) begin end else
																				if (cache_data[3]`USED == 0) begin end else
																				if (cache_data[4]`USED == 0) begin end else
																				if (cache_data[5]`USED == 0) begin end else
																				if (cache_data[6]`USED == 0) begin end else
																				if (cache_data[7]`USED == 0) begin end
																				else begin
																					$display("rewind");

																					cache_data[0]`USED <= 0;
																					cache_data[1]`USED <= 0;
																					cache_data[2]`USED <= 0;
																					cache_data[3]`USED <= 0;
																					cache_data[4]`USED <= 0;
																					cache_data[5]`USED <= 0;
																					cache_data[6]`USED <= 0;
																					cache_data[7]`USED <= 0;
																				end
																				des <= des1;
																				ir3 <= ir_stalled;
																				src <= val_out;
																				query_cache <= 0;
																				cache_state <= `CACHE_STANDBY;
																				$display("memory: %d, value found: %d on line %d", cache_data[hit]`LINE_MEMORY, cache_data[hit]`LINE_VALUE, hit);

																			end else begin
																				$display("miss");
																				// find an empty cache line, or go to flush state if all used.
																				mem_in = cache_mem;
																				if (cache_data[0]`LINE_INIT == 0 || cache_data[0]`LINE_INIT === 1'bx) begin replace = 0; end else
																				if (cache_data[1]`LINE_INIT == 0 || cache_data[1]`LINE_INIT === 1'bx) begin replace = 1; end else
																				if (cache_data[2]`LINE_INIT == 0 || cache_data[2]`LINE_INIT === 1'bx) begin replace = 2; end else
																				if (cache_data[3]`LINE_INIT == 0 || cache_data[3]`LINE_INIT === 1'bx) begin replace = 3; end else
																				if (cache_data[4]`LINE_INIT == 0 || cache_data[4]`LINE_INIT === 1'bx) begin replace = 4; end else
																				if (cache_data[5]`LINE_INIT == 0 || cache_data[5]`LINE_INIT === 1'bx) begin replace = 5; end else
																				if (cache_data[6]`LINE_INIT == 0 || cache_data[6]`LINE_INIT === 1'bx) begin replace = 6; end else
																				if (cache_data[7]`LINE_INIT == 0 || cache_data[7]`LINE_INIT === 1'bx) begin replace = 7; end else begin replace = -1; end

																				if (replace < 0) begin
																					write <= 1;
																					// if there is no empty line, find one to flush
																					// check if all cache lines have been recently used
																					// if flushed line is dirty, write it to memory. TODO
																					$display("flush");
																					if (cache_data[0]`USED == 0) begin replace = 0; end else
																					if (cache_data[1]`USED == 0) begin replace = 1; end else
																					if (cache_data[2]`USED == 0) begin replace = 2; end else
																					if (cache_data[3]`USED == 0) begin replace = 3; end else
																					if (cache_data[4]`USED == 0) begin replace = 4; end else
																					if (cache_data[5]`USED == 0) begin replace = 5; end else
																					if (cache_data[6]`USED == 0) begin replace = 6; end else
																					if (cache_data[7]`USED == 0) begin replace = 7; end else begin replace = -1; end
																					val_in	<= cache_data[replace]`LINE_VALUE;
																				end

																					cache_state <= `READ;
																			end
																		end

																	// Cache reads a value from slow memory
																		`READ: begin
																			cache_mem <= 16'bx;
																			mem_in <= 16'bx;		// we found what we were looking for so now the arbiter needs to be told to stop
																			if (mem_rdy) begin
																				$display("read");
																				if (replace >= 0) begin
																					cache_data[replace]`USED = 1;
																					cache_data[replace]`LINE_INIT = 1;
																					cache_data[replace]`LINE_MEMORY = mem_out;
																					cache_data[replace]`LINE_VALUE = val_out;
																					des <= des1;
																					if (ir_stalled != `OPnop) begin
																						ir3 <= ir_stalled;
																					end
																					src <= val_out;
																					query_cache <= 0;
																					cache_state <= `CACHE_STANDBY;

																					$display("line %d gets memory location: %d with value: %d", replace, cache_data[replace]`LINE_MEMORY, cache_data[replace]`LINE_VALUE);

																				end else begin
																					$display("ERROR: trying to access cache line with negative index");
																				end
																			end
																			else begin
																				cache_state <= `READ;
																			end
																		end

																		endcase // cache

																		// if we have a jump or branch, we need to clear out queued instructions
																						if (jump || branch) begin
																							ir_stalled <= `OPnop;
																						end
	end

endmodule // core

module testbench;
wire halt, write, wtoo, mem_strobe;
wire `DATA val_in;
wire `ADDRESS mem_in, addr;
wire mem_rdy, rdy;
wire `DATA val_out, rdata;
wire `ADDRESS mem_out;
reg reset = 0;
reg clk = 0;

core core1(halt, val_in, mem_in, write, mem_rdy, val_out, mem_out, reset, clk);
arbiter a(mem_out, val_out, wdata, wtoo, addr, mem_rdy, mem_strobe, mem_in, rdy, val_in, rdata, write, clk);
slowmem16 memory(rdy, rdata, addr, wdata, wtoo, mem_strobe, clk);

initial begin
                $dumpfile;
                $dumpvars(0, core1.ir0, core1.ir1, core1.ir2, core1.ir3, core1.ir_stalled, core1.query_cache, core1.branch);
                #10 reset = 1;
                #10 reset = 0;
                while (!halt) begin
                                #10 clk = 1;
                                #10 clk = 0;
                end
                $finish;
end
endmodule
//test
