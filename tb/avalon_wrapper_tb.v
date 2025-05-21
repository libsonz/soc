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
   // NEW: byteenable testbench signal
   reg [(N_BANKS * DATA_WIDTH)/8 - 1:0] byteenable;
   reg [N_BANKS * DATA_WIDTH - 1:0]     temp_read_data;
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

   // Derive DATA_IN_WIDTH for byteenable sizing
   localparam                                          DATA_IN_WIDTH = N_BANKS * DATA_WIDTH;
   localparam                                          BYTE_EN_FULL = {DATA_IN_WIDTH/8 {1'b1}};

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
        .clk         (clk),
        .reset_n     (reset_n),
        .address     (address),
        .chipselect  (chipselect),
        .read        (read),
        .write       (write),
        .writedata   (writedata),
        .byteenable  (byteenable), // Connect new signal
        .readdata    (readdata),
        .waitrequest (waitrequest)
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
      input [DATA_IN_WIDTH - 1:0] data;
      input [DATA_IN_WIDTH/8 - 1:0] byte_en_in; // NEW: Input for byteenable
      begin
         @(posedge clk); // Align to positive clock edge
         chipselect = 1'b1;
         write = 1'b1;
         read = 1'b0;
         address = addr;
         writedata = data;
         byteenable = byte_en_in; // Drive the byteenable signal
         // Wait for waitrequest to go low (if asserted)
         while (waitrequest) @(posedge clk);
         @(posedge clk); // Hold transaction for one cycle
         chipselect = 1'b0;
         write = 1'b0;
         address = 'b0;
         writedata = 'b0;
         byteenable = '0; // Deassert byteenable after transaction
      end
   endtask

   task avalon_read;
      input [ID_WIDTH-1:0] addr;
      output [DATA_IN_WIDTH-1:0] data; // Output matches readdata width
      begin
         @(posedge clk); // Align to positive clock edge
         chipselect = 1'b1;
         read = 1'b1;
         write = 1'b0;
         address = addr;
         // For read, byteenable should be 'all ones' or ignored by slave
         byteenable = {DATA_IN_WIDTH/8 {1'b1}};
         // Wait for waitrequest to go low (if asserted)
         while (waitrequest) @(posedge clk);
         @(posedge clk); // Read data is typically valid on the next cycle
         data = readdata; // Capture the read data
         chipselect = 1'b0;
         read = 1'b0;
         address = 'b0;
         byteenable = '0; // Deassert byteenable
      end
   endtask
   //----------------------------------------------------------------------------

   // Test sequence
   initial
     begin

        // Initialize Avalon signals
        chipselect = 1'b0;
        read = 1'b0;
        write = 1'b0;
        address = 'b0;
        writedata = 'b0;
        byteenable = '0; // Initialize byteenable

        // Apply reset
        reset_n = 1'b0;
        #40; // Hold reset for 20ns (2 clock cycles)
        reset_n = 1'b1;
        #40; // Wait a bit after reset release

        $display("--- Start Test Sequence ---");

        // Test 1: Write to Control Register (Start Multiplication)
        $display("Time %0t: Writing 0x1 to Control Register (Addr %0d) to start multiplication.", $time, ADDR_CONTROL);
        avalon_write(ADDR_CONTROL, 3, BYTE_EN_FULL);
        @(posedge clk); // Give the system a chance to register the write

        // Wait for multiplication to complete
        $display("Time %0t: Waiting for mult_done...", $time);
        @(posedge dut.top_inst.mult_done);
        $display("Time %0t: mult_done asserted!", $time);
        #10; // Give it one more cycle

        // Test 2: Read Status Register (mult_done)
        $display("Time %0t: Reading Status Register (Addr %0d).", $time, ADDR_STATUS);
        avalon_read(ADDR_STATUS, temp_read_data);
        $display("Time %0t: Read mult_done status: %h (Expected: 1)", $time, temp_read_data[0]);
        #10;

        // Test 3: Write C BRAM Read Address
        $display("Time %0t: Writing 0xAB to C BRAM Read Address (Addr %0d).", $time, ADDR_C_ADDR);
        avalon_write(ADDR_C_ADDR, 16'h00AB, BYTE_EN_FULL);
        #10;

        // Test 4: Read C BRAM Data
        $display("Time %0t: Reading C BRAM Data (Addr %0d).", $time, ADDR_C_DATA);
        avalon_read(ADDR_C_DATA, temp_read_data);
        $display("Time %0t: Read C BRAM Data: %h (Expected: 0xAB + 100 = 0x117)", $time, temp_read_data);
        #10;

        // Test 5: Write A BRAM Load Address
        $display("Time %0t: Writing 0x1234 to A BRAM Load Address (Addr %0d).", $time, ADDR_A_ADDR);
        avalon_write(ADDR_A_ADDR, 32'h00001234, BYTE_EN_FULL);
        #10;

        // Test 6: Write A BRAM Load Data (full write)
        $display("Time %0t: Full write 0xABCD to A BRAM Load Data (Addr %0d).", $time, ADDR_A_DATA);
        avalon_write(ADDR_A_DATA, {DATA_IN_WIDTH {1'b1}} & 32'hABCD, BYTE_EN_FULL);
        #10;

        /*
        // Test 7: Partial Write to A BRAM Load Data - only the first byte of the first element
        // Assuming DATA_WIDTH = 16, N_BANKS = 3, so DATA_IN_WIDTH = 48, byteenable_tb width = 6
        // The first byte of the first bank corresponds to byteenable[0] and bits [7:0] of writedata.
        // If the full `a_data_reg` was previously 0x...ABCD... (from previous write),
        // and we write 0x11 to only byte 0, then `a_data_reg` should become 0x...ABC**11**.
        $display("Time %0t: Partial write to A BRAM Data (Addr %0d), only byte 0 with 0x11", $time, ADDR_A_DATA);
        avalon_write(ADDR_A_DATA, {{DATA_IN_WIDTH-8}{1'b0}, 8'h11}, 1'b1 << 0);
        #10;

        // Test 8: Partial Write to A BRAM Load Data - only the 4th byte (byte 3 of first bank)
        // If DATA_WIDTH = 16 (2 bytes per bank), and N_BANKS = 3.
        // byteenable[0] -> Bank 0, Byte 0 (bits 7:0)
        // byteenable[1] -> Bank 0, Byte 1 (bits 15:8)
        // byteenable[2] -> Bank 1, Byte 0 (bits 23:16)
        // byteenable[3] -> Bank 1, Byte 1 (bits 31:24)
        // byteenable[4] -> Bank 2, Byte 0 (bits 39:32)
        // byteenable[5] -> Bank 2, Byte 1 (bits 47:40)
        $display("Time %0t: Partial write to A BRAM Data (Addr %0d), only byte %0d with 0x22", $time, ADDR_A_DATA, (DATA_WIDTH/8) * 1 + 0);
        avalon_write(ADDR_A_DATA, {{DATA_IN_WIDTH-((DATA_WIDTH/8)*8)-8}{1'b0}, 8'h22, {(DATA_WIDTH/8)*8}{1'b0}}, 1'b1 << (DATA_WIDTH/8)); // Byte 1 of 1st bank
        #10;

        // Reset A_data_reg for next test
        $display("Time %0t: Full write 0xDEADBEEF to A BRAM Load Data (Addr %0d).", $time, ADDR_A_DATA);
        avalon_write(ADDR_A_DATA, {DATA_IN_WIDTH {1'b1}} & 32'hDEADBEEF, BYTE_EN_FULL);
        #10;

        // Test 9: Write B BRAM Load Address
        $display("Time %0t: Writing 0x5678 to B BRAM Load Address (Addr %0d).", $time, ADDR_B_ADDR);
        avalon_write(ADDR_B_ADDR, 32'h00005678, BYTE_EN_FULL);
        #10;

        // Test 10: Write B BRAM Load Data (full write)
        $display("Time %0t: Full write 0xEF01 to B BRAM Load Data (Addr %0d).", $time, ADDR_B_DATA);
        avalon_write(ADDR_B_DATA, {DATA_IN_WIDTH {1'b1}} & 32'hEF01, BYTE_EN_FULL);
        #10;

        // Test 11: Partial Write to B BRAM Load Data - only the last byte
        // If DATA_IN_WIDTH=48, this is byteenable[5]
        $display("Time %0t: Partial write to B BRAM Data (Addr %0d), only last byte with 0x33", $time, ADDR_B_DATA);
        avalon_write(ADDR_B_DATA, {{DATA_IN_WIDTH-8}{1'b0}, 8'h33}, 1'b1 << (DATA_IN_WIDTH/8 - 1));
        #10;
        */

        $display("--- End Test Sequence ---");
        #100; // Final delay
        $finish; // End simulation
     end

endmodule
