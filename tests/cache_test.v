// Cache definitions
`define DIRTYBIT        [1]
`define LRUBIT           [0]
`define TAG             [14]
`define CACHE_LINES   [15:0]
`define CACHEVALUE      [17:2]
`define MEMADDR      [33:18]

module cache(out , mem_in, write, reset, clk);
output [15:0] out;
input [15:0] mem_in;
input write, reset, clk;


reg [15:0] mem [7:0];
reg [33:0] cache_data [7:0];
reg `CACHE_LINES hit;
wire [3:0] replaceline;                          //to store cache line number we plan to replace
reg [7:0] used;

always@(reset) begin
$readmemh0(mem);
end


always@(posedge clk) begin
cache_data[0]`MEMADDR <= mem[0];

if (write) begin
        if (cache_data[0]`DIRTYBIT) begin
        // TODO: commit cache line to memory
        end
        // TODO: do for each cache line
end

// check if any line in the cache is the designated memory address
hit[0] <= (cache_data[0]`MEMADDR & mem_in) ? 1 : 0;
hit[1] <= (cache_data[1]`MEMADDR & mem_in) ? 1 : 0;
hit[2] <= (cache_data[2]`MEMADDR & mem_in) ? 1 : 0;
hit[3] <= (cache_data[3]`MEMADDR & mem_in) ? 1 : 0;
hit[4] <= (cache_data[4]`MEMADDR & mem_in) ? 1 : 0;
hit[5] <= (cache_data[5]`MEMADDR & mem_in) ? 1 : 0;
hit[6] <= (cache_data[6]`MEMADDR & mem_in) ? 1 : 0;
hit[7] <= (cache_data[7]`MEMADDR & mem_in) ? 1 : 0;

if (|hit) begin
// send found value to the PE
$display("hit");
used <= used | hit;                    //OR used register with hit to update Recently used cache line
used <= (&used) ? 0 : used;      //checks to see if cachedata is 11111111 os so set all bits back to zero, if not do nothing

end else begin
// find value in slowmem (TODO check the other cache for value)
$display("miss");
end
end

endmodule // cache

// TEST BENCH

module testbench;
reg clk = 0;
reg reset = 0;
reg write = 0;
reg [15:0] mem_in;
wire [15:0] result;
reg [15:0] memory [7:0];
reg count = 0;

cache c(result, mem_in, write, reset, clk);

//module cache(out , mem_in, write, reset, clk);

initial begin
$readmemh0(memory, reset);
#10 reset = 1;
#10 reset = 0;
 repeat (8) begin #1
#10 clk = 1;
#10 clk = 0;
mem_in <= memory[count];
end
$finish;
end

endmodule // testbench