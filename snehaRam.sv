module alu(
    mode,r1,r2,r3,addr,PC
);

input logic signed [18:0] r3,r2,addr;
input [4:0] mode;
output logic signed [18:0] r1,PC; // 19 bit architecture

reg [9:0] stack [18:0];
reg [18:0] sp = 0; // setting the default value as 0, initially
reg [9:0] memory [18:0];

always @(*) begin
    case(mode)
        5'd0 : r1 = ADD(r2,r3);
        5'd1 : r1 = SUB(r2,r3);
        5'd2 : r1 = MUL(r2,r3);
        5'd3 : r1 = DIV(r2,r3);
        5'd4 : r1 = INC(r1);
        5'd5 : r1 = DEC(r1);

        5'd6 : r1 = AND(r2,r3);
        5'd7 : r1 = OR(r2,r3);
        5'd8 : r1 = XOR(r2,r3);
        5'd9 : r1 = NOT(r1);

        5'd10 : JMP_ADDR;
        5'd11 : r1 = BEQ(r2,addr);
        5'd12 : r1 = BNE(r2,addr);
        5'd13 : CALL_addr;
        5'd14 : RET;

        5'd15 : LD(addr);
        5'd16 : ST(addr,r1);

        

    endcase
end
// alu
function logic signed [18:0] ADD(input [18:0] x,y);
    return x+y;
endfunction

function logic signed [18:0] SUB(input [18:0] x,y);
return x-y;
endfunction

function logic signed [18:0] MUL(input [18:0] x,y);
return x*y;
endfunction

function logic signed [18:0] DIV(input [18:0] x,y);
return x/y;
endfunction

function logic signed [18:0] INC(input [18:0] x);
return x+1;
endfunction

function logic signed [18:0] DEC(input [18:0] x);
return x-1;
endfunction

// logical instructions
function logic [18:0] AND(input [18:0] x,y);
return x&y;
endfunction

function logic [18:0] OR(input [18:0] x,y);
return x|y;
endfunction

function logic [18:0] XOR(input [18:0] x,y);
return x^y;
endfunction

function logic [18:0] NOT(input [18:0] x);
return !x;
endfunction

// control flow instructions
function void JMP_ADDR();
PC = addr;
endfunction

function logic [18:0] BEQ(input [18:0] x,y);
    if(r1 == r2) return addr;
    else return 0;
endfunction

function logic [18:0] BNE(input [18:0] x,y);
if(r1 != r2) return addr;
    else return 0;
endfunction

function void CALL_addr();
stack[sp] = PC+1;
sp = sp - 1;
PC = addr;
endfunction

function void RET();
    sp = sp+1;
    PC = stack[sp];
endfunction

// memory access instructions
function void LD(input [18:0] x);
r1 = memory[x];
endfunction

function void ST(input [18:0] addr,x);
memory[addr] = x;
endfunction

// custom instructions


endmodule

module tb;
logic [18:0] r1,r2,r3,addr,PC;
logic [4:0] mode;

alu dut(.mode(mode),.r1(r1),.r2(r2),.r3(r3),.addr(addr),.PC(PC));

initial begin
    {r2,r3,addr} = 0;
    #5;
mode = 1;
#5;
mode = 5;
end
initial begin
     $monitor("mode = %0d, r1 = %0d, r2 = %0d, r3 = %0d, addr = %0d, pc = %0d",mode,r1,r2,r3,addr,PC);
#350 $finish();
end

endmodule