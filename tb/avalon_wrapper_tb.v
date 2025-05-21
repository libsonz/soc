//----------------------------------------------------------------------------
// Testbench for matrix_multiplier_avalon_wrapper (using tasks for readability)
//----------------------------------------------------------------------------
`timescale 1ns / 1ps

module avalon_wrapper_tb;

   // Parameters (must match the avalon_wrapper module)
   parameter DATA_WIDTH = 16;
   parameter M = 4;
   parameter K = 4;
   parameter N = 4;
   parameter N_BANKS = 4;
   parameter PE_ROWS = M;
   parameter PE_COLS = N;
   parameter ID_WIDTH = 3; // For address lines (0-7 -> 3 bits)

   // Testbench signals (corresponding to avalon_wrapper ports)
   reg       clk;
   reg       reset_n;
   reg [ID_WIDTH-1:0] address;
   reg                chipselect;
   reg                read;
   reg                write;
   reg [N_BANKS * DATA_WIDTH - 1:0] writedata; // Adjusted to match wrapper's input port width
   wire [DATA_WIDTH * 2 + ((K > 1) ? $clog2(K) : 1):0] readdata; // Matches wrapper's output port width
   wire                                                waitrequest;

   // Local parameters for address map (for clarity in testbench)
   localparam                                          ADDR_CONTROL = 3'd0;
   localparam                                          ADDR_STATUS = 3'd1;
   localparam                                          ADDR_C_ADDR = 3'd2;
   localparam                                          ADDR_C_DATA = 3'd3;
   localparam                                          ADDR_A_ADDR = 3'd4;
   localparam                                          ADDR_A_DATA = 3'd5;
   localparam                                          ADDR_B_ADDR = 3'd6;
   localparam                                          ADDR_B_DATA = 3'd7;

   // Instantiate the avalon_wrapper
   avalon_wrapper
     #(
       .DATA_WIDTH (DATA_WIDTH),
       .M          (M),
       .K          (K),
       .N          (N),
       .N_BANKS    (N_BANKS),
       .PE_ROWS    (PE_ROWS),
       .PE_COLS    (PE_COLS),
       .ID_WIDTH   (ID_WIDTH)
       )
   dut (
        .clk          (clk),
        .reset_n      (reset_n),
        .address      (address),
        .chipselect   (chipselect),
        .read         (read),
        .write        (write),
        .writedata    (writedata),
        .readdata     (readdata),
        .waitrequest  (waitrequest)
        );


   // Clock generation
   initial begin
      clk = 0;
      forever #5 clk = ~clk; // 10ns period (100 MHz)
   end

   // Tasks for Avalon MM transactions
   //----------------------------------------------------------------------------
   task avalon_write;
      input [ID_WIDTH-1:0] addr;
      input [N_BANKS * DATA_WIDTH - 1:0] data;
      begin
         @(posedge clk); // Align to positive clock edge
         chipselect = 1'b1;
         write = 1'b1;
         read = 1'b0;
         address = addr;
         writedata = data;
         // Wait for waitrequest to go low (if asserted)
         while (waitrequest) @(posedge clk);
         @(posedge clk); // Hold transaction for one cycle
         chipselect = 1'b0;
         write = 1'b0;
         address = 'b0;
         writedata = 'b0;
      end
   endtask

   task avalon_read;
      input [ID_WIDTH-1:0] addr;
      output [DATA_WIDTH * 2 + ((K > 1) ? $clog2(K) : 1):0] data; // Output matches readdata width
      begin
         @(posedge clk); // Align to positive clock edge
         chipselect = 1'b1;
         read = 1'b1;
         write = 1'b0;
         address = addr;
         // Wait for waitrequest to go low (if asserted)
         while (waitrequest) @(posedge clk);
         @(posedge clk); // Read data is typically valid on the next cycle
         data = readdata; // Capture the read data
         chipselect = 1'b0;
         read = 1'b0;
         address = 'b0;
      end
   endtask
   //----------------------------------------------------------------------------

   // Test sequence
   initial
     begin : inital

        reg [DATA_WIDTH * 2 + ((K > 1) ? $clog2(K) : 1):0] temp_read_data;
        // Dump waves for viewing (if using a simulator like Icarus Verilog or Questa/ModelSim)
        $dumpfile("avalon_wrapper_tb.vcd");
        $dumpvars(0, avalon_wrapper_tb);

        // Initialize Avalon signals
        chipselect = 1'b0;
        read = 1'b0;
        write = 1'b0;
        address = 'b0;
        writedata = 'b0;

        // Apply reset
        reset_n = 1'b0;
        #40; // Hold reset for 20ns (2 clock cycles)
        reset_n = 1'b1;
        #40; // Wait a bit after reset release

        $display("--- Start Test Sequence ---");


        // Test 5: Write A BRAM Load Address
        // 1100 1000 0100 0000 -> C840
        $display("Time %0t: Writing 0x1234 to A BRAM Load Address (Addr %0d).", $time, ADDR_A_ADDR);
        avalon_write(ADDR_A_ADDR, 32'h0000C840); // Placeholder, adjust width if needed
        #10;

        // Test 6: Write A BRAM Load Data (this should pulse a_we_reg and a_en_reg)
        $display("Time %0t: Writing 0xABCD to A BRAM Load Data (Addr %0d).", $time, ADDR_A_DATA);
        avalon_write(ADDR_A_DATA, {N_BANKS * DATA_WIDTH {1'b1}} & 32'hABCD); // Broadcast data
        #10;

        // Test 7: Write B BRAM Load Address
        $display("Time %0t: Writing 0x5678 to B BRAM Load Address (Addr %0d).", $time, ADDR_B_ADDR);
        avalon_write(ADDR_B_ADDR, 32'h0000C840); // Placeholder, adjust width if needed
        #10;

        // Test 8: Write B BRAM Load Data (this should pulse b_we_reg and b_en_reg)
        $display("Time %0t: Writing 0xEF01 to B BRAM Load Data (Addr %0d).", $time, ADDR_B_DATA);
        avalon_write(ADDR_B_DATA, {N_BANKS * DATA_WIDTH {1'b1}} & 32'hEF01); // Broadcast data
        #10;

        // Test 1: Write to Control Register (Start Multiplication)
        $display("Time %0t: Writing 0x1 to Control Register (Addr %0d) to start multiplication.", $time, ADDR_CONTROL);
        avalon_write(ADDR_CONTROL, 3); // bit 0 = 1 (start_mult)

        // Wait for multiplication to complete
        $display("Time %0t: Waiting for mult_done...", $time);
        @(posedge dut.top_mult_done);
        $display("Time %0t: mult_done asserted!", $time);
        #10; // Give it one more cycle

        // Test 2: Read Status Register (mult_done)
        $display("Time %0t: Reading Status Register (Addr %0d).", $time, ADDR_STATUS);
        avalon_read(ADDR_STATUS, temp_read_data);
        $display("Time %0t: Read mult_done status: %h (Expected: 1)", $time, temp_read_data[0]);
        #10;

        // Test 3: Write C BRAM Read Address
        $display("Time %0t: Writing 0xAB to C BRAM Read Address (Addr %0d).", $time, ADDR_C_ADDR);
        avalon_write(ADDR_C_ADDR, 16'h00AB); // Assuming ADDR_WIDTH_C is less than DATA_WIDTH
        #10;

        // Test 4: Read C BRAM Data
        $display("Time %0t: Reading C BRAM Data (Addr %0d).", $time, ADDR_C_DATA);
        avalon_read(ADDR_C_DATA, temp_read_data);
        $display("Time %0t: Read C BRAM Data: %h (Expected: 0xAB + 100 = 0x117)", $time, temp_read_data);
        #10;

        $display("--- End Test Sequence ---");
        #100; // Final delay
        $finish; // End simulation
     end

endmodule
