//----------------------------------------------------------------------------
// Module: matrix_multiplier_top_tb
// Description: Testbench for the matrix_multiplier_top module.
//              Loads matrices using the shared Port A, triggers the
//              multiplication, and verifies results.
//----------------------------------------------------------------------------
`timescale 1ns/1ps
module top_tb;

   // Parameters - Must match the top-level module instantiation
   parameter DATA_WIDTH = 16; // Data width of matrix elements A and B
   parameter M = 4;           // Number of rows in Matrix A and C
   parameter K = 4;           // Number of columns in Matrix A and rows in Matrix B
   parameter N = 4;           // Number of columns in Matrix B and C
   parameter N_BANKS = 4;     // Number of BRAM banks for Matrix A and B

   // Parameters for the 2D PE Array dimensions (Must match top-level module)
   parameter PE_ROWS = M;     // Number of PE rows = M
   parameter PE_COLS = N;     // Number of PE columns = N
   parameter N_PE = PE_ROWS * PE_COLS; // Total number of PEs

   // Derived parameters (matching top-level module, used here for sizing)
   // Ensure dimensions are positive to avoid $clog2(0) issues
   parameter ADDR_WIDTH_A = ($clog2(N_BANKS) + ((M/N_BANKS * K > 0) ? $clog2(M/N_BANKS * K) : 1));
   parameter ADDR_WIDTH_B = ($clog2(N_BANKS) + ((K * N/N_BANKS > 0) ? $clog2(K * N/N_BANKS) : 1));
   parameter ADDR_WIDTH_A_BANK = (M/N_BANKS * K > 0) ? $clog2(M/N_BANKS * K) : 1;
   parameter ADDR_WIDTH_B_BANK = (K * N/N_BANKS > 0) ? $clog2(K * N/N_BANKS) : 1;
   parameter ADDR_WIDTH_C = (M * N > 0) ? $clog2(M * N) : 1;
   // Accumulator width: DATA_WIDTH*2 for product + $clog2(K) for K additions
   parameter ADDR_WIDTH_BANK = $clog2(N_BANKS);
   parameter ACC_WIDTH = DATA_WIDTH * 2 + ((K > 1) ? $clog2(K) : 1);

   // Calculated Sizes for BRAMs and Matrices (matching datapath)
   parameter A_BANK_SIZE = (M/N_BANKS) * K; // Size of each A BRAM bank
   parameter B_BANK_SIZE = K * (N/N_BANKS); // Size of each B BRAM bank
   parameter C_BRAM_SIZE = M * N;           // Size of the C BRAM


   // Testbench Control Parameters
   parameter NUM_TEST_CASES = 100; // How many test case directories to read (test_000 to test_099)
   // !! IMPORTANT: Update this path to where your test case directories are located !!
   // Use a reg array for the base path
   parameter [8*100-1:0] TEST_CASE_DIR_BASE = "/home/lamar/Documents/git/matrix-multiplier/testcases"; // Base directory for test cases (Max 100 chars)
   parameter             MAX_FILENAME_LEN = 150; // Maximum length for generated filenames


   // Testbench Signals (Inputs to Top Module - Declared as regs)
   reg                   clk;         // Clock signal
   reg                   rst_n;       // Asynchronous active-low reset
   reg                   start_mult;  // Start signal to initiate multiplication

   reg                   en_a_brams_in;
   reg [N_BANKS * ($clog2(N_BANKS) + ((M/N_BANKS * K > 0) ? $clog2(M/N_BANKS * K) : 1)) - 1:0] addr_a_brams_in;
   reg                                                                                         we_a_brams_in;
   reg [N_BANKS * DATA_WIDTH - 1:0]                                                            din_a_brams_in;

   reg                                                                                         en_b_brams_in;
   reg [N_BANKS * ($clog2(N_BANKS) + ((K * N/N_BANKS > 0) ? $clog2(K * N/N_BANKS) : 1)) - 1:0] addr_b_brams_in;
   reg                                                                                         we_b_brams_in;
   reg [N_BANKS * DATA_WIDTH - 1:0]                                                            din_b_brams_in;
   // Data input for writing to B banks (Port A)


   // External C BRAM Read Interface (for testbench verification)
   reg                                     read_en_c;   // External read enable for C BRAM Port B
   reg [ADDR_WIDTH_C-1:0]                  read_addr_c; // External read address for C BRAM Port B

   // Testbench Signals (Outputs from Top Module - Declared as wires)
   wire                                    mult_done;   // Signal indicating multiplication is complete
   wire [ACC_WIDTH-1:0]                    dout_c;      // Data output from C BRAM


   // **Wires to read BRAM outputs hierarchically for verification**
   // These wires are connected to the internal BRAM dout_a ports via a generate block
   wire [DATA_WIDTH-1:0]                   tb_bram_a_dout [N_BANKS-1:0];
   wire [DATA_WIDTH-1:0]                   tb_bram_b_dout [N_BANKS-1:0];

   // Internal testbench arrays to hold matrix data and expected results
   reg [DATA_WIDTH-1:0]                    matrix_A [M-1:0][K-1:0];
   reg [DATA_WIDTH-1:0]                    matrix_B [K-1:0][N-1:0];
   reg [ACC_WIDTH-1:0]                     expected_C [M-1:0][N-1:0];
   reg [ACC_WIDTH-1:0]                     actual_C [M-1:0][N-1:0]; // Flattened array to store read results

   reg [N_BANKS * DATA_WIDTH-1:0]          partitioned_matrix_A [N_BANKS-1:0];
   reg [N_BANKS * DATA_WIDTH-1:0]          partitioned_matrix_B [N_BANKS-1:0];

   // Internal testbench arrays to hold matrix data and expected results
   reg [DATA_WIDTH-1:0]                    testbench_A [0:M-1][0:K-1];
   reg [DATA_WIDTH-1:0]                    testbench_B [0:K-1][0:N-1];

   // Internal variables for loops and test counters
   integer                                 i, j, k; // Loop variables
   integer                                 test_case; // Current test case number (0 to NUM_TEST_CASES-1)
   integer                                 pass_count; // Counter for passed test cases
   integer                                 fail_count; // Counter for failed test cases
   integer                                 total_errors; // Total element mismatches across all test cases


   // Clock Generation
   always #5 clk = ~clk; // 10ns clock period (adjust as needed)

   // Instantiate the Top-Level Matrix Multiplier module
   top
     #(
       .DATA_WIDTH (DATA_WIDTH),
       .M          (M),
       .K          (K),
       .N          (N),
       .N_BANKS    (N_BANKS),
       .PE_ROWS    (PE_ROWS),
       .PE_COLS    (PE_COLS)
       )
   dut (
        .clk                                                    (clk),
        .rst_n                                                  (rst_n),
        .start_mult                                             (start_mult),
        .mult_done                                              (mult_done),

        // **Connected to Testbench BRAM Load/Execution Signals (Port A)**
        .en_a_brams_in                                          (en_a_brams_in),
        .addr_a_brams_in                                        (addr_a_brams_in),
        .we_a_brams_in                                          (we_a_brams_in),
        .din_a_brams_in                                         (din_a_brams_in),
        .en_b_brams_in                                          (en_b_brams_in),
        .addr_b_brams_in                                        (addr_b_brams_in),
        .we_b_brams_in                                          (we_b_brams_in),
        .din_b_brams_in                                         (din_b_brams_in),

        .read_en_c                                              (read_en_c),
        .read_addr_c                                            (read_addr_c),
        .dout_c                                                 (dout_c)
        );

   /*
   // **Generate block to connect testbench wires to internal BRAM outputs for verification**
   // This uses constant indices within the generate loop.
   genvar verify_gen_idx;
   generate
      for (verify_gen_idx = 0; verify_gen_idx < N_BANKS; verify_gen_idx = verify_gen_idx + 1)
        begin : tb_bram_readers
           // Connect testbench wires to the dout_a ports of the internal BRAM instances
           // These hierarchical references use the constant generate variable verify_gen_idx
           assign tb_bram_a_dout[verify_gen_idx] = dut.datapath_inst.a_bram_gen[verify_gen_idx].a_bram_inst.dout_a;
           assign tb_bram_b_dout[verify_gen_idx] = dut.datapath_inst.b_bram_gen[verify_gen_idx].b_bram_inst.dout_a;
        end
   endgenerate
    */

   // ----------------------------------------------------------------------------------- //
   // Task to read A and B matrices from external text files
   // ----------------------------------------------------------------------------------- //
   task read_matrices_and_expected_C;
      input integer test_num;
      // Use reg arrays for filenames instead of string
      reg [8*MAX_FILENAME_LEN-1:0] dir_path;
      reg [8*MAX_FILENAME_LEN-1:0] a_filename;
      reg [8*MAX_FILENAME_LEN-1:0] b_filename;
      reg [8*MAX_FILENAME_LEN-1:0] c_filename;
      integer                      matrix_row; // Declare variables at start of task
      integer                      matrix_col; // Declare variables at start of task
      reg [DATA_WIDTH-1:0]         read_value_data; // Declare variables at start of task
      reg [ACC_WIDTH-1:0]          read_value_acc; // Declare variables at start of task
      integer                      scan_ret; // Declare variables at start of task
      integer                      file_handle; // Declare variables at start of task

      begin // Start of task body
         // Construct filenames using $sformatf
         $sformat(dir_path, "%0s/test_%0d", TEST_CASE_DIR_BASE, test_num);
         $sformat(a_filename, "%0s/matrix_A.txt", dir_path);
         $sformat(b_filename, "%0s/matrix_B.txt", dir_path);
         $sformat(c_filename, "%0s/expected_C.txt", dir_path);

         $display("Reading test case %0d: %s, %s, and %s", test_num, a_filename, b_filename, c_filename);

         // Read A matrix (assuming hexadecimal values in file)
         file_handle = $fopen(a_filename, "r"); // Open file for reading
         if (file_handle == 0)
           begin
              $error("Could not open A matrix file: %s", a_filename);
              $finish; // Abort simulation on error
           end
         else
           begin
              for (matrix_row = 0; matrix_row < M; matrix_row = matrix_row + 1)
                begin
                   for (matrix_col = 0; matrix_col < K; matrix_col = matrix_col + 1)
                     begin
                        scan_ret = $fscanf(file_handle, "%h", read_value_data);
                        if (scan_ret != 1)
                          begin
                             $error("Error reading A matrix file %s at row %0d, col %0d", a_filename, matrix_row, matrix_col);
                             $fclose(file_handle);
                             $finish;
                          end
                        testbench_A[matrix_row][matrix_col] = read_value_data; // Store in testbench array
                     end
                end
              $fclose(file_handle); // Close file
           end // else: !if(file_handle == 0)

         // Read B matrix (assuming hexadecimal values in file)
         file_handle = $fopen(b_filename, "r"); // Open file for reading
         if (file_handle == 0)
           begin
              $error("Could not open B matrix file: %s", b_filename);
              $finish;
           end
         else
           begin
              for (matrix_row = 0; matrix_row < K; matrix_row = matrix_row + 1) begin // B has K rows
                 for (matrix_col = 0; matrix_col < N; matrix_col = matrix_col + 1) begin // B has N columns
                    scan_ret = $fscanf(file_handle, "%h", read_value_data);
                    if (scan_ret != 1) begin
                       $error("Error reading B matrix file %s at row %0d, col %0d", b_filename, matrix_row, matrix_col);
                       $fclose(file_handle);
                       $finish;
                    end
                    testbench_B[matrix_row][matrix_col] = read_value_data; // Store in testbench array
                 end
              end
              $fclose(file_handle); // Close file
           end // else: !if(file_handle == 0)

         // Read expected C matrix (assuming hexadecimal values in file)
         file_handle = $fopen(c_filename, "r"); // Open file for reading
         if (file_handle == 0)
           begin
              $error("Could not open expected C matrix file: %s", c_filename);
              $finish;
           end
         else
           begin
              for (matrix_row = 0; matrix_row < M; matrix_row = matrix_row + 1) begin
                 for (matrix_col = 0; matrix_col < N; matrix_col = matrix_col + 1) begin
                    scan_ret = $fscanf(file_handle, "%h", read_value_acc);
                    if (scan_ret != 1) begin
                       $error("Error reading expected C matrix file %s at row %0d, col %0d", c_filename, matrix_row, matrix_col);
                       $fclose(file_handle);
                       $finish;
                    end
                    expected_C[matrix_row][matrix_col] = read_value_acc; // Store in testbench array
                 end
              end
              $fclose(file_handle); // Close file
           end // else: !if(file_handle == 0)

         $display("Matrices and expected C read successfully.");
      end
   endtask

   //--------------------------------------------------------------------------
   // Matrix Partition
   //--------------------------------------------------------------------------
   task matrix_partition;
      begin : matrix_partition
         integer r, c;
         reg [K * DATA_WIDTH - 1:0] d_out_a[N_BANKS-1:0];
         reg [K * DATA_WIDTH - 1:0] d_out_b[N_BANKS-1:0];

         // Matrix A Partition
         for (c = 0; c < K; c = c + 1)
           begin
              for (r = 0; r < M; r = r + 1)
                begin
                   d_out_a[c][(r * DATA_WIDTH) +: DATA_WIDTH] = testbench_A[r][c];
                end
              partitioned_matrix_A[c] = d_out_a[c];
           end

         // Matrix B Parition
         for (r = 0; r < K; r = r + 1)
           begin
              for (c = 0; c < N; c = c + 1)
                begin
                   d_out_b[r][(c * DATA_WIDTH) +: DATA_WIDTH]  = testbench_B[r][c];
                end
              partitioned_matrix_B[r] = d_out_b[r];
           end
         $display("@%0t Matrix partition ended! ", $time);
      end
   endtask


   //--------------------------------------------------------------------------
   // Load matrices into bram
   //--------------------------------------------------------------------------

   // Task to load sample matrices A and B into BRAMs via Port A
   task load_matrices_into_brams;
      begin: load_matrices_into_brams
         integer r, c, b; // Row, Column, Bank loop variables

         $display("@%0t: Loading DUT BRAMs using shared Port A...", $time);

         // Ensure controller is not active and read port is disabled during load
         start_mult = 0; // Ensure controller is not driving Port A
         read_en_c = 0;
         @(posedge clk); #1; // Wait a cycle for signals to settle

         // Load Matrix A
         en_a_brams_in = 1;
         we_a_brams_in = 1;
         for (r = 0; r < M; r = r + 1)
           begin // Iterate through rows
              din_a_brams_in = partitioned_matrix_A[r];
              for (c = 0; c < K; c = c + 1)
                begin // Iterate through columns
                   // Calculate combined address for A[r][c] in A_BRAM[r % N_BANKS]
                   // Address within bank: (r / N_BANKS) * K + c
                   // Combined address: {r % N_BANKS, (r / N_BANKS) * K + c}

                   $display("  Loading A[%0d][%0d] (Bank %0d, Addr %0d) with %h", r, c, r, c, partitioned_matrix_A[r][(c * DATA_WIDTH) +: DATA_WIDTH]);
                   // Address for A
                   // addr in bank
                   addr_a_brams_in[c * ADDR_WIDTH_A + ADDR_WIDTH_A_BANK - 1 -: ADDR_WIDTH_A_BANK] = r;

                   // bank idx
                   addr_a_brams_in[c * ADDR_WIDTH_A + ADDR_WIDTH_A - 1 -: ADDR_WIDTH_BANK] = c;
                end // for (c = 0; c < K; c = c + 1)
              @(posedge clk) #1;
           end // for (r = 0; r < M; r = r + 1)

         @(posedge clk)
           begin
              en_a_brams_in = 0; // Deassert enable and write after loading
              we_a_brams_in = 0;
           end

         // Load Matrix B
         en_b_brams_in = 1;
         we_b_brams_in = 1;
         for (c = 0; c < N; c = c + 1)
           begin // Iterate through rows
              din_b_brams_in = partitioned_matrix_B[c];
              for (r = 0; r < K; r = r + 1)
                begin // Iterate through columns
                   // Calculate combined address for B[r][c] in B_BRAM[c % N_BANKS]
                   // Address within bank: r * (N / N_BANKS) + c / N_BANKS
                   // Combined address: {c % N_BANKS, r * (N / N_BANKS) + c / N_BANKS}

                   $display("  Loading B[%0d][%0d] (Bank %0d, Addr %0d) with %h", r, c, c, r, partitioned_matrix_B[c][(r * DATA_WIDTH) +: DATA_WIDTH]);
                   // Address for B
                   // addr in bank
                   addr_b_brams_in[r * ADDR_WIDTH_A + ADDR_WIDTH_B_BANK - 1 -:ADDR_WIDTH_B_BANK] = c;

                   // bank idx
                   addr_b_brams_in[r * ADDR_WIDTH_A + ADDR_WIDTH_A - 1 -: ADDR_WIDTH_BANK] = r;
                end // for (c = 0; c < N; c = c + 1)
              @(posedge clk) #1;
           end // for (r = 0; r < K; r = r + 1
         @(posedge clk)
           begin
              en_b_brams_in = 0; // Deassert enable and write after loading
              we_b_brams_in = 0;
           end
      end
   endtask

   /*
   // --------------------------------------------------------------------------
   // Task to verify the contents of the loaded A and B BRAMs
   // Reads BRAM outputs via hierarchical references using a
   // testbench-local generate block.
   // --------------------------------------------------------------------------
   task verify_loaded_brams;
      integer matrix_row;
      integer matrix_col;
      integer bank_a_idx;
      integer addr_a;
      integer bank_b_idx;
      integer addr_b;
      integer errors;
      reg [DATA_WIDTH-1:0] read_data_a; // Data read from A BRAM
      reg [DATA_WIDTH-1:0] read_data_b; // Data read from B BRAM


      begin
         $display("@%0t: Verifying loaded BRAM contents...", $time);
         errors = 0;

         // Ensure controller is not active and read port is disabled during verification
         start_mult = 0;
         read_en_c = 0;
         // Also ensure BRAM write enables are low
         we_a_brams_in = 'b0;
         we_b_brams_in = 'b0;
         @(posedge clk); #1; // Wait a cycle for signals to settle

         // Verify A BRAMs
         $display("  Verifying A BRAMs...");
         for (matrix_row = 0; matrix_row < M; matrix_row = matrix_row + 1) begin
            for (matrix_col = 0; matrix_col < K; matrix_col = matrix_col + 1) begin
               bank_a_idx = matrix_row % N_BANKS;
               addr_a = (matrix_row / N_BANKS) * K + matrix_col;

               if (addr_a < (M/N_BANKS * K)) begin // Check bounds
                  // Drive the shared Port A read signals
                  en_a_brams_in[bank_a_idx] = 1;
                  addr_a_brams_in[(bank_a_idx * ADDR_WIDTH_A_BANK) +: ADDR_WIDTH_A_BANK] = addr_a;
                  // Keep write enable low
                  we_a_brams_in[bank_a_idx] = 0;

                  @(posedge clk); #1; // Apply address and enable, wait for clock edge

                  // Read data from the testbench wire connected by the generate block
                  // Use the procedural variable bank_a_idx to select the correct testbench wire
                  read_data_a = tb_bram_a_dout[bank_a_idx];

                  // Compare with testbench array
                  if (read_data_a !== testbench_A[matrix_row][matrix_col]) begin
                     $display("  A BRAM Verify FAIL: Bank %0d, Addr %0h (Matrix A[%0d][%0d]). Read %h, Expected %h",
                              bank_a_idx, addr_a, matrix_row, matrix_col, read_data_a, testbench_A[matrix_row][matrix_col]);
                     errors = errors + 1;
                  end

                  // Deassert enable for this bank
                  en_a_brams_in[bank_a_idx] = 0;

                  @(posedge clk); // Wait for the next clock cycle
               end
            end
         end
         // Ensure all A BRAM enables are low after verification
         en_a_brams_in = 'b0;


         $display("  Verifying B BRAMs...");
         // Verify B BRAMs
         for (matrix_row = 0; matrix_row < K; matrix_row = matrix_row + 1) begin // Matrix B has K rows
            for (matrix_col = 0; matrix_col < N; matrix_col = matrix_col + 1) begin // Matrix col has N columns
               bank_b_idx = matrix_col % N_BANKS;
               addr_b = matrix_row * (N / N_BANKS) + matrix_col / N_BANKS;

               if (addr_b < (K * N/N_BANKS)) begin // Check bounds
                  // Drive the shared Port A read signals
                  en_b_brams_in[bank_b_idx] = 1;
                  addr_b_brams_in[(bank_b_idx * ADDR_WIDTH_B_BANK)+:ADDR_WIDTH_B_BANK] = addr_b;
                  // Keep write enable low
                  we_b_brams_in[bank_b_idx] = 0;

                  @(posedge clk); #1; // Apply address and enable, wait for clock edge

                  // Read data from the testbench wire connected by the generate block
                  // Use the procedural variable bank_b_idx to select the correct testbench wire
                  read_data_b = tb_bram_b_dout[bank_b_idx];

                  // Compare with testbench array
                  if (read_data_b !== testbench_B[matrix_row][matrix_col]) begin
                     $display("  B BRAM Verify FAIL: Bank %0d, Addr %0h (Matrix B[%0d][%0d]). Read %h, Expected %h",
                              bank_b_idx, addr_b, matrix_row, matrix_col, read_data_b, testbench_B[matrix_row][matrix_col]);
                     errors = errors + 1;
                  end

                  // Deassert enable for this bank
                  en_b_brams_in[bank_b_idx] = 0;

                  @(posedge clk); // Wait for the next clock cycle
               end
            end
         end
         // Ensure all B BRAM enables are low after verification
         en_b_brams_in = 'b0;


         if (errors == 0)
           begin
              $display("@%0t: BRAM contents verified successfully.", $time);
           end
         else
           begin
              $error("@%0t: BRAM contents verification failed with %0d errors.", $time, errors);
           end
         $display("--------------------------------------");
         #10; // Wait a bit
      end
   endtask
    */


   // --------------------------------------------------------------------------
   // Task to run the multiplication sequence via the controller
   // --------------------------------------------------------------------------
   task run_multiplication;
      begin
         $display("@%0t: Asserting start_mult to trigger controller...", $time);
         // The controller will now take over driving the Port A signals
         start_mult = 1;
         @(posedge mult_done); // Wait for the controller to signal completion
         $display("@%0t: Controller signalled mult_done high.", $time);
         start_mult = 0; // Deassert start after completion (controller releases Port A)
         #100; // Wait a bit after completion before reading
      end
   endtask


   // ---------------------- //
   // Task to verify results //
   // ---------------------- //
   task verify_results;
      begin : verify_results
         integer row_v; // Declare variables at start of task
         integer col_v; // Declare variables at start of task
         integer element_errors; // Declare variables at start of task
         real    error_percentage; // Declare variables at start of task

         begin // Start of task body
            $display("Verifying results for test case %0d...", test_case);
            element_errors = 0; // Reset error count for this test case

            // The actual_C matrix is assumed to be populated by the read_actual_c task
            // Compare the actual result with the calculated expected result
            read_en_c = 1;
            for (row_v = 0; row_v < M; row_v = row_v + 1)
              begin // Loop through rows of C
                 for (col_v = 0; col_v < N; col_v = col_v + 1)
                   begin // Loop through columns of C
                      read_addr_c = row_v * M + col_v;
                      @(posedge clk); #1;
                      actual_C[row_v][col_v] = dout_c;
                      if (actual_C[row_v][col_v] !== expected_C[row_v][col_v])
                        begin
                           $display("Test Case %0d FAIL: C[%0d][%0d] mismatch! Actual %h, Expected %h",
                                    test_case, row_v, col_v, actual_C[row_v][col_v], expected_C[row_v][col_v]);
                           element_errors = element_errors + 1;
                        end // else begin
                      else
                        begin
                           $display("Test Case %0d PASS: C[%0d][%0d] = %h",
                                    test_case, row_v, col_v, actual_C[row_v][col_v]);
                        end // else: !if(actual_C[row_v][col_v] !== expected_C[row_v][col_v])
                      // $display("  C[%0d][%0d] matches: %h", row_v, col_v, actual_C[row_v][col_v]); // Uncomment for successful matches
                      // end
                   end
              end // for (row_v = 0; row_v < M; row_v = row_v + 1)
            if(element_errors == 0)
              begin
                 pass_count = pass_count + 1;
              end
            else
              begin
                 fail_count = fail_count + 1;
                 total_errors = total_errors + element_errors;
              end
            read_en_c = 0;
         end
      end

   endtask


   // --------------------------------------------------------------------------
   // --- Main Initial Block ---
   // --------------------------------------------------------------------------
   initial
     begin
        // Setup waveform dumping for debugging
        $dumpfile("top_tb.vcd");
        $dumpvars(0, top_tb); // Dump all signals in the testbench module

        // Initialize all testbench inputs to a known state at time 0
        clk = 0;
        rst_n = 0; // Start with reset asserted
        start_mult = 0;
        read_en_c = 0;
        read_addr_c = 0;

        // Initialize shared Port A signals
        en_a_brams_in = 'b0;
        addr_a_brams_in = 'b0;
        we_a_brams_in = 'b0;
        din_a_brams_in = 'b0;
        en_b_brams_in = 'b0;
        addr_b_brams_in = 'b0;
        we_b_brams_in = 'b0;
        din_b_brams_in = 'b0;


        // Initialize test counters
        pass_count = 0;
        fail_count = 0;
        total_errors = 0;

        // Wait for initial setup time
        #100;

        // // comment /////////////////////////////////////////////////////////////// Apply Reset ---
        $display("\n--- Applying Reset ---");
        rst_n = 0; // Assert reset
        #100; // Hold reset for 100 time units
        rst_n = 1; // Release reset
        #100; // Wait for reset to propagate
        $display("Reset complete.");


      for (test_case = 0; test_case < NUM_TEST_CASES; test_case = test_case + 1)
        begin

           $display("\n===================================================");
           $display("@%0t Starting Test Case %0d of %0d",$time, test_case, NUM_TEST_CASES);
           $display("===================================================");
           read_matrices_and_expected_C(test_case);


           $display("\n===================================================");
           $display("@%0t Matrix Partition", $time);
           $display("===================================================");
           matrix_partition();

           // Load Sample Matrices A and B into BRAMs via Port A
           // This simulates an external loader or DMA writing to the BRAMs.
           $display("@%0t Loading matrices A and B into BRAMs via Port A...", $time);
           load_matrices_into_brams();
           $display("Matrix loading complete.");

           // Simulate Controller Signals to Execute Multiplication
           $display("@%0t Simulating controller signals to execute multiplication...", $time);
           run_multiplication();
           $display("Multiplication execution sequence complete.");

           // Read Result from C BRAM via Port B and Verify
           $display("@%0t Reading result from C BRAM via Port B and verifying...", $time);
           verify_results();
           $display("Result verification complete.");

           #100;
        end // for (test_case = 0; test_case < NUM_TEST_CASES; test_case = test_case + 1)

      $display("--------------------------------------------------");
      $display(" Pass cases : %d", pass_count);
      $display(" Fail cases : %d", fail_count);
        $display(" Testbench FAILED with %0d errors!", total_errors);
        $display("--------------------------------------------------");

        #100; // Wait before finishing
        $display("@%0t: Testbench simulation finished.", $time);
        $finish; // End simulation

     end // initial begin

endmodule
