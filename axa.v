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

`define NOP         	  16'b0

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
`define FIND_HIT					4'b0001		// is there a hit in the cache
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
		$display("in slowmem");
    rdata <= m[maddr];
    if (mwtoo) m[maddr] <= mwdata;
    busy <= 0;
    rdy <= 1;
  end else if (busy > 1) begin
    // still waiting
    busy <= busy - 1;
  end else if (strobe) begin
    // idle and new request
		$display("addr = %d", addr);
    rdata <= 16'hxxxx;
    maddr <= addr;
    mwdata <= wdata;
    mwtoo <= wtoo;
    busy <= `MEMDELAY;
    rdy <= 0;
  end
end
endmodule

//******************************************************
//*										  	ARBITOR											 *
//******************************************************

module arbitor(mem_out, val_out, wdata, wtoo, address, mem_rdy, mem_strobe, mem_in, rdy, val_in, rdata, write, clk);

output reg `DATA val_out, wdata;
output reg `ADDRESS mem_out;
output reg `ADDRESS address;									// mem_out tells the caches which memory has been modified in memory so their values can be updated
output reg mem_rdy, mem_strobe, wtoo;								// strobe = 0 when transaction is occuring

input `DATA val_in, rdata;
input `ADDRESS mem_in;
input rdy, write, clk;

reg arb_rdy = 0;

always @(posedge clk) begin
// also check here if in a transaction
if (mem_in !== 16'bx) begin
	$display("arbitor is ready, mem_in = %d", mem_in);
	arb_rdy <= 1;
end

if (arb_rdy) begin
	wtoo <= write;																// write is 1 if core is doing a write operation, wtoo tells slowmem to write a value in memory. if 0, read value.
	mem_strobe <= 1;
	assign address = mem_in;																// mem_in is memory address from core that is being accessed in slowmem
	$display("in arbitor addr = %d", address);
	wdata <= val_in;															// val_in is the new value sent from updated cache to be written in slowmem
	mem_out <= mem_in;														// address to be updated in cache is the same as the address modified in slowmem
	val_out <= rdata;															// val_out is value received from slowmem that is being sent to the cache
	mem_rdy <= rdy;																// mem_rdy tells core that the slowmem has completed its operation
	if (mem_rdy) begin $display("val_out = %d", val_out); end
	arb_rdy <= 0;																	// arbiter is done, so now it can wait for another transaction
end else begin
// do nothing
	mem_strobe <= 0;
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
reg `INSTRUCTION ir0, ir1, ir2, ir3; //instruction registers for each stage
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
                ir1= `NOP;
                ir2= `NOP;
                ir3= `NOP;
                op4 = `NOP;
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
                                tpc= target;
                                jump <=0;
                end else if(branch) begin
                                tpc= pc + src-1;
                                branch<=0;
                end else begin
                                tpc=pc;
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
                                                ir1 <= `NOP;
                                                pc <= tpc;
                end else begin
                                ir0 = instrmem[tpc];
                                ir1<= ir0;
                                pc<= tpc+1;
                end
end


end //always block

//stage2: register read
always @(posedge clk) begin
                if((ir1 != `NOP) && setsdes(ir2) && ((usesdes(ir1) && (ir1 `DESTREG == ir2 `DESTREG)) || (usessrc(ir1) && (ir1 `SRCREG == ir2 `DESTREG)))) begin
                                wait1 = 1;
                                ir2 <= `NOP;
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
                if(ir2 == `NOP) begin
                                ir3 <= `NOP;
                end else begin
                                if(ir2 `SRCTYPE == `SrcTypeMem) begin
                                                // check this PE's cache
                                                cache_mem <= ir2 `SRCREG;                        // send new memory address to cache
                                                query_cache <= 1;
                                                // check the other PE's cache
                                                // get value from slowmem
                                                //src <= datamem[ir2 `SRCREG];
                                end else begin
                                                src <= src2;
                                end
																if (mem_rdy) begin
																	$display("done waiting");
	                                des<=des1;
	                                ir3 <= ir2;
																end else begin $display("waiting on memory"); end
                end
end

// stage4: execute and write
always @(posedge clk) begin
                if (ir3 == `NOP) begin
                                jump <= 0;
                end else begin
                                op4 = ir3 `OP;
                //des = ir3`DESTREG;
                                //src <= src1;
                                case(op4)

                                `OPxlo: begin res = {des`WHIGH, src`WLOW ^ des`WLOW}; end
                                `OPxhi: begin res = {src`WLOW ^ des`WHIGH  , des`WLOW}; end
                                `OPllo: begin res = {{8{src[7]}}, src}; op4 <=`OPnop; end
                                `OPlhi: begin res = {src, 8'b0}; end
                                `OPand: begin res = des & src; end
                                `OPor:   begin res = des | src; end
                                `OPxor: begin res = des ^ src; end
                                `OPadd: begin res = des + src; end
                                `OPsub: begin res = des - src; end
                                `OProl: begin res = (des << src) |(des >> (16 - src)); end
                                `OPshr: begin res = des >> src; end
                                `OPbzjz: begin if(des==0) begin
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
end //  always

// cache block

always @(posedge clk) begin
case (cache_state)
	`CACHE_STANDBY: begin
		if (query_cache) begin cache_state <= `FIND_HIT;
			$display("cache_mem = %d", cache_mem);
		end

	end
	`FIND_HIT: begin
		// check if any line in the cache is the designated memory address, then store cache line index in hit
		if (cache_data[0]`LINE_MEMORY == cache_mem) begin hit <= 0; end else
		if (cache_data[1]`LINE_MEMORY == cache_mem) begin hit <= 1; end else
		if (cache_data[2]`LINE_MEMORY == cache_mem) begin hit <= 2; end else
		if (cache_data[3]`LINE_MEMORY == cache_mem) begin hit <= 3; end else
		if (cache_data[4]`LINE_MEMORY == cache_mem) begin hit <= 4; end else
		if (cache_data[5]`LINE_MEMORY == cache_mem) begin hit <= 5; end else
		if (cache_data[6]`LINE_MEMORY == cache_mem) begin hit <= 6; end else
		if (cache_data[7]`LINE_MEMORY == cache_mem) begin hit <= 7; end else begin hit <= -1; end

		if(hit>=0) begin
			cache_state <= `HIT;
		end else begin
			cache_state <= `MISS;
		end
	end

	`HIT: begin
		$display("Hit");
		$display("value found: %d on line %d", cache_data[hit]`LINE_VALUE, hit);
		// tells arbitor to not read a value from memory
		cache_data[hit]`USED <= 1;
		cache_data[hit]`LINE_INIT <= 1;
		src <= cache_data[hit]`LINE_VALUE;
		cache_state <= `REWIND;
	end

	`MISS: begin
		$display("miss");
		// find an empty cache line, or go to flush state if all used.
		mem_in <= cache_mem;
		$display("cache[0]LINE_INIT = %d",cache_data[0]`LINE_INIT );
		if (cache_data[0]`LINE_INIT == 0 || cache_data[0]`LINE_INIT === 1'bx) begin replace = 0; $display("replace = %d", replace ); end else
		if (cache_data[1]`LINE_INIT == 0 || cache_data[1]`LINE_INIT === 1'bx) begin replace = 1; end else
		if (cache_data[2]`LINE_INIT == 0 || cache_data[2]`LINE_INIT === 1'bx) begin replace = 2; end else
		if (cache_data[3]`LINE_INIT == 0 || cache_data[3]`LINE_INIT === 1'bx) begin replace = 3; end else
		if (cache_data[4]`LINE_INIT == 0 || cache_data[4]`LINE_INIT === 1'bx) begin replace = 4; end else
		if (cache_data[5]`LINE_INIT == 0 || cache_data[5]`LINE_INIT === 1'bx) begin replace = 5; end else
		if (cache_data[6]`LINE_INIT == 0 || cache_data[6]`LINE_INIT === 1'bx) begin replace = 6; end else
		if (cache_data[7]`LINE_INIT == 0 || cache_data[7]`LINE_INIT === 1'bx) begin replace = 7; end else begin replace = -1; end

		$display("in miss, replace = %d", replace);
		if (replace >= 0) begin cache_state <= `READ; end
		else begin cache_state <= `FLUSH; end
		end

	`FLUSH: begin
		// if there is no empty line, find one to flush
		// check if all cache lines have been recently used
		// if flushed line is dirty, write it to memory.
		if (cache_data[0]`USED == 0) begin replace = 0; end else
		if (cache_data[1]`USED == 0) begin replace = 1; end else
		if (cache_data[2]`USED == 0) begin replace = 2; end else
		if (cache_data[3]`USED == 0) begin replace = 3; end else
		if (cache_data[4]`USED == 0) begin replace = 4; end else
		if (cache_data[5]`USED == 0) begin replace = 5; end else
		if (cache_data[6]`USED == 0) begin replace = 6; end else
		if (cache_data[7]`USED == 0) begin replace = 7; end else begin replace = -1; end

		cache_state <= `READ;
	end

	`READ: begin
		if (mem_rdy) begin
			$display("in read, mem_out = %d, val_out = %d, cacheline = %d", mem_out, val_out, replace);

			cache_data[replace]`USED <= 1;
			cache_data[replace]`LINE_INIT <= 1;
			cache_data[replace]`LINE_MEMORY <= mem_out;
			cache_data[replace]`LINE_VALUE <= val_out;
			cache_state <= `CACHE_STANDBY;
		end
		else begin
			cache_state <= `READ;
		end
	end

	`REWIND: begin
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
			cache_data[0]`USED <= 0;
			cache_data[1]`USED <= 0;
			cache_data[2]`USED <= 0;
			cache_data[3]`USED <= 0;
			cache_data[4]`USED <= 0;
			cache_data[5]`USED <= 0;
			cache_data[6]`USED <= 0;
			cache_data[7]`USED <= 0;
		end

		cache_state <= `CACHE_STANDBY;
	end

	endcase
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
arbitor a(mem_out, val_out, wdata, wtoo, addr, mem_rdy, mem_strobe, mem_in, rdy, val_in, rdata, write, clk);
slowmem16 memory(rdy, rdata, addr, wdata, wtoo, mem_strobe, clk);

initial begin
                $dumpfile;
                $dumpvars(0, core1.ir0);
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
