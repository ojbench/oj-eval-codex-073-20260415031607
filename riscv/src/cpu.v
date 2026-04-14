// RISCV32I CPU top module
// Simple in-order, single-issue RV32I CPU with byte-wide memory port

module cpu(
  input  wire                 clk_in,            // system clock signal
  input  wire                 rst_in,            // reset signal
  input  wire                 rdy_in,            // ready signal, pause cpu when low

  input  wire [ 7:0]          mem_din,           // data input bus
  output wire [ 7:0]          mem_dout,          // data output bus
  output wire [31:0]          mem_a,             // address bus (only 17:0 is used)
  output wire                 mem_wr,            // write/read signal (1 for write)
  
  input  wire                 io_buffer_full,    // 1 if uart buffer is full
  
  output wire [31:0]          dbgreg_dout        // cpu register output (debugging demo)
);

// Registers and state
reg [31:0] pc;
reg [31:0] ir;
reg [31:0] regs[31:0];

wire [4:0]  rd_id  = ir[11:7];
wire [4:0]  rs1_id = ir[19:15];
wire [4:0]  rs2_id = ir[24:20];
wire [31:0] rs1 = regs[rs1_id];
wire [31:0] rs2 = regs[rs2_id];

// Memory interface registers
reg [31:0] mem_a_r;
reg [7:0]  mem_dout_r;
reg        mem_wr_r;
assign mem_a    = mem_a_r;
assign mem_dout = mem_dout_r;
assign mem_wr   = mem_wr_r;

// Debug register output (demo)
assign dbgreg_dout = regs[10]; // x10 (a0)

// Immediates
wire [31:0] imm_i = {{20{ir[31]}}, ir[31:20]};
wire [31:0] imm_s = {{20{ir[31]}}, ir[31:25], ir[11:7]};
wire [31:0] imm_b = {{19{ir[31]}}, ir[31], ir[7], ir[30:25], ir[11:8], 1'b0};
wire [31:0] imm_u = {ir[31:12], 12'b0};
wire [31:0] imm_j = {{11{ir[31]}}, ir[31], ir[19:12], ir[20], ir[30:21], 1'b0};

// FSM states
localparam S_RESET      = 0,
           S_FETCH0     = 1,
           S_FETCH1     = 2,
           S_FETCH2     = 3,
           S_FETCH3     = 4,
           S_FETCH4     = 5,
           S_DECODE     = 6,
           S_LD_ADDR    = 7,
           S_LD_B0      = 8,
           S_LD_B1      = 9,
           S_LD_B2      = 10,
           S_LD_B3      = 11,
           S_ST_B0      = 12,
           S_ST_W0      = 13,
           S_ST_W1      = 14,
           S_ST_W2      = 15,
           S_ST_W3      = 16,
           S_COMMIT     = 17;

reg [4:0]  state;
reg        write_rd;
reg [31:0] write_data;
reg [31:0] next_pc;
reg        branch_taken;
reg        br_cond;
reg [31:0] eff_addr;
reg [31:0] ld_data;
reg [1:0]  ld_size;        // 0=byte,1=half,2=word
reg        ld_unsigned;

integer i;

// Keep x0 zero
always @(*) begin
  // nothing combinational here for regs
end

always @(posedge clk_in) begin
  if (rst_in) begin
    pc <= 32'h00000000;
    state <= S_RESET;
    mem_wr_r <= 1'b0;
    mem_a_r <= 32'b0;
    mem_dout_r <= 8'b0;
    for (i=0; i<32; i=i+1) regs[i] <= 32'b0;
  end else if (!rdy_in) begin
    // stall: hold state and outputs
  end else begin
    regs[0] <= 32'b0; // hardwire x0
    case (state)
      S_RESET: begin
        // Initiate fetch of first instruction
        mem_wr_r <= 1'b0;
        mem_a_r <= pc; // set address, data available next cycle
        state <= S_FETCH1;
      end
      S_FETCH0: begin
        // Not used (kept for readability)
        mem_wr_r <= 1'b0;
        mem_a_r <= pc;
        state <= S_FETCH1;
      end
      S_FETCH1: begin
        // read byte 0
        ir[7:0] <= mem_din;
        mem_a_r <= pc + 1;
        state <= S_FETCH2;
      end
      S_FETCH2: begin
        // read byte 1
        ir[15:8] <= mem_din;
        mem_a_r <= pc + 2;
        state <= S_FETCH3;
      end
      S_FETCH3: begin
        // read byte 2
        ir[23:16] <= mem_din;
        mem_a_r <= pc + 3;
        state <= S_FETCH4;
      end
      S_FETCH4: begin
        // read byte 3
        ir[31:24] <= mem_din;
        state <= S_DECODE;
      end

      S_DECODE: begin
        // Defaults
        write_rd <= 1'b0;
        branch_taken <= 1'b0;
        next_pc <= pc + 4;

        case (ir[6:0])
          7'b0110111: begin // LUI
            write_rd <= 1'b1; write_data <= imm_u; state <= S_COMMIT;
          end
          7'b0010111: begin // AUIPC
            write_rd <= 1'b1; write_data <= pc + imm_u; state <= S_COMMIT;
          end
          7'b1101111: begin // JAL
            write_rd <= 1'b1; write_data <= pc + 4; next_pc <= pc + imm_j; branch_taken <= 1'b1; state <= S_COMMIT;
          end
          7'b1100111: begin // JALR
            write_rd <= 1'b1; write_data <= pc + 4; next_pc <= (rs1 + imm_i) & ~32'b1; branch_taken <= 1'b1; state <= S_COMMIT;
          end
          7'b1100011: begin // Branch
            case (ir[14:12])
              3'b000: br_cond = (rs1 == rs2);                     // BEQ
              3'b001: br_cond = (rs1 != rs2);                     // BNE
              3'b100: br_cond = ($signed(rs1) <  $signed(rs2));   // BLT
              3'b101: br_cond = ($signed(rs1) >= $signed(rs2));   // BGE
              3'b110: br_cond = (rs1 < rs2);                      // BLTU
              3'b111: br_cond = (rs1 >= rs2);                     // BGEU
            endcase
            // Choose next pc based on br_cond
            next_pc <= (br_cond ? (pc + imm_b) : (pc + 32'd4));
            state <= S_COMMIT; // no rd write
          end
          7'b0000011: begin // Loads
            eff_addr <= rs1 + imm_i;
            ld_unsigned <= (ir[14:12]==3'b100)||(ir[14:12]==3'b101); // LBU/LHU
            case (ir[14:12])
              3'b000,3'b100: ld_size <= 2'd0; // LB/LBU
              3'b001,3'b101: ld_size <= 2'd1; // LH/LHU
              default:        ld_size <= 2'd2; // LW
            endcase
            mem_wr_r <= 1'b0;
            mem_a_r <= rs1 + imm_i; // request first byte
            state <= S_LD_B0;
          end
          7'b0100011: begin // Stores
            eff_addr <= rs1 + imm_s;
            // Byte 0 write
            mem_a_r <= rs1 + imm_s;
            mem_dout_r <= rs2[7:0];
            mem_wr_r <= 1'b1;
            case (ir[14:12])
              3'b000: state <= S_ST_W0; // SB (only one byte)
              3'b001: state <= S_ST_W1; // SH (two bytes)
              default: state <= S_ST_W3; // SW (four bytes)
            endcase
          end
          7'b0010011: begin // ALU immediate
            case (ir[14:12])
              3'b000: write_data <= rs1 + imm_i;                         // ADDI
              3'b010: write_data <= ($signed(rs1) <  $signed(imm_i))?1:0; // SLTI
              3'b011: write_data <= (rs1 < imm_i)?1:0;                    // SLTIU
              3'b100: write_data <= rs1 ^ imm_i;                          // XORI
              3'b110: write_data <= rs1 | imm_i;                          // ORI
              3'b111: write_data <= rs1 & imm_i;                          // ANDI
              3'b001: write_data <= rs1 << ir[24:20];                     // SLLI
              3'b101: begin
                if (ir[30]) write_data <= $signed(rs1) >>> ir[24:20];     // SRAI
                else        write_data <= rs1 >> ir[24:20];               // SRLI
              end
            endcase
            write_rd <= 1'b1; state <= S_COMMIT;
          end
          7'b0110011: begin // ALU reg
            case ({ir[31:25], ir[14:12]})
              {7'b0000000,3'b000}: write_data <= rs1 + rs2;               // ADD
              {7'b0100000,3'b000}: write_data <= rs1 - rs2;               // SUB
              {7'b0000000,3'b001}: write_data <= rs1 << rs2[4:0];         // SLL
              {7'b0000000,3'b010}: write_data <= ($signed(rs1) < $signed(rs2))?1:0; // SLT
              {7'b0000000,3'b011}: write_data <= (rs1 < rs2)?1:0;         // SLTU
              {7'b0000000,3'b100}: write_data <= rs1 ^ rs2;               // XOR
              {7'b0000000,3'b101}: write_data <= rs1 >> rs2[4:0];         // SRL
              {7'b0100000,3'b101}: write_data <= $signed(rs1) >>> rs2[4:0]; // SRA
              {7'b0000000,3'b110}: write_data <= rs1 | rs2;               // OR
              {7'b0000000,3'b111}: write_data <= rs1 & rs2;               // AND
              default: write_data <= 32'b0;
            endcase
            write_rd <= 1'b1; state <= S_COMMIT;
          end
          default: begin
            // Unsupported: treat as NOP
            state <= S_COMMIT;
          end
        endcase
      end

      // Load sequence (little endian, synchronous read -> data valid next cycle)
      S_LD_B0: begin
        ld_data[7:0] <= mem_din;
        if (ld_size==2'd0) begin
          state <= S_COMMIT;
        end else begin
          mem_a_r <= eff_addr + 1;
          state <= S_LD_B1;
        end
      end
      S_LD_B1: begin
        ld_data[15:8] <= mem_din;
        if (ld_size==2'd1) begin
          state <= S_COMMIT;
        end else begin
          mem_a_r <= eff_addr + 2;
          state <= S_LD_B2;
        end
      end
      S_LD_B2: begin
        ld_data[23:16] <= mem_din;
        mem_a_r <= eff_addr + 3;
        state <= S_LD_B3;
      end
      S_LD_B3: begin
        ld_data[31:24] <= mem_din;
        state <= S_COMMIT;
      end

      // Store sequence: each byte write is one cycle with mem_wr high
      S_ST_W0: begin
        // finalize byte0 write
        mem_wr_r <= 1'b0;
        // SB completes here
        state <= S_COMMIT;
      end
      S_ST_W1: begin
        // wrote byte0 previously, now write byte1 for SH
        mem_wr_r <= 1'b0;
        mem_a_r <= eff_addr + 1;
        mem_dout_r <= rs2[15:8];
        mem_wr_r <= 1'b1;
        state <= S_ST_W2;
      end
      S_ST_W2: begin
        mem_wr_r <= 1'b0;
        // For SH, we are done
        if (ir[14:12]==3'b001) begin
          state <= S_COMMIT;
        end else begin
          // For SW, write remaining bytes
          mem_a_r <= eff_addr + 2; mem_dout_r <= rs2[23:16]; mem_wr_r <= 1'b1; state <= S_ST_W3;
        end
      end
      S_ST_W3: begin
        mem_wr_r <= 1'b0;
        if (ir[14:12]==3'b001) begin
          state <= S_COMMIT;
        end else begin
          mem_a_r <= eff_addr + 3; mem_dout_r <= rs2[31:24]; mem_wr_r <= 1'b1; state <= S_ST_B0;
        end
      end
      S_ST_B0: begin
        mem_wr_r <= 1'b0;
        state <= S_COMMIT;
      end

      S_COMMIT: begin
        // Finalize load data if needed and write back
        if (ir[6:0]==7'b0000011) begin
          case (ld_size)
            2'd0: write_data <= ld_unsigned ? {24'b0, ld_data[7:0]} : {{24{ld_data[7]}}, ld_data[7:0]};
            2'd1: write_data <= ld_unsigned ? {16'b0, ld_data[15:0]} : {{16{ld_data[15]}}, ld_data[15:0]};
            default: write_data <= ld_data;
          endcase
          write_rd <= 1'b1;
        end
        if (write_rd && rd_id!=5'd0) begin
          regs[rd_id] <= write_data;
        end
        // Update PC and start next fetch
        pc <= next_pc;
        mem_wr_r <= 1'b0;
        mem_a_r <= next_pc;
        state <= S_FETCH1;
      end
    endcase
  end
end

endmodule
