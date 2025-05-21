//----------------------------------------------------------------------------
// Module: datapath_tb
// Description: Testbench for the updated datapath module.
//              Loads sample matrices into A and B BRAMs via Port A,
//              simulates controller signals to perform multiplication,
//              and reads the result from the C BRAM Port B.
//
// Assumptions:
// - Assumes the presence of 'bram.v', 'pe_no_fifo.v', 'multiplier_carrysave.v',
//   'full_adder.v', 'multiplier_adder.v' in the simulation environment.
// - Uses parameters matching the datapath module.
// - Loads hardcoded sample matrices A and B.
// - Checks the final result against a hardcoded expected matrix C.
//----------------------------------------------------------------------------
`timescale 1ns / 1ps

module datapath_tb;

   //--------------------------------------------------------------------------
   // Parameters
   // Must match the datapath module under test
   // Using M=3, K=3, N=3, N_BANKS=3 as in your provided code.
   parameter DATA_WIDTH = 16;
   parameter M = 4;
   parameter K = 4;
   parameter N = 4;
   parameter N_BANKS = 4;
   parameter PE_ROWS = M;
   parameter PE_COLS = N;

   // Derived Parameters (matching datapath)
   parameter ADDR_WIDTH_A = $clog2(N_BANKS) + ((M/N_BANKS * K > 0) ? $clog2(M/N_BANKS * K) : 1);
   parameter ADDR_WIDTH_B = $clog2(N_BANKS) + ((K * N/N_BANKS > 0) ? $clog2(K * N/N_BANKS) : 1);
   parameter ADDR_WIDTH_C = (M * N > 0) ? $clog2(M * N) : 1;
   parameter ACC_WIDTH_PE = DATA_WIDTH * 2 + ((K > 1) ? $clog2(K) : 1);
   parameter N_PE = PE_ROWS * PE_COLS;
   parameter ADDR_WIDTH_PE_IDX = ($clog2(N_PE > 0 ? N_PE : 1) > 0) ? $clog2(N_PE > 0 ? N_PE : 1) : 1;

   parameter BANK_IDX_WIDTH = $clog2(N_BANKS);
   parameter ADDR_IN_BANK_WIDTH = ((M/N_BANKS * K > 0) ? $clog2(M/N_BANKS * K) : 1);

   // Clock Period
   parameter CLK_PERIOD = 10; // 10 ns period = 100 MHz

   // PE Pipeline Latency (from pe_valid_in_in to pe_output_valid high)
   // This should match the PE module's actual latency. Assuming 3 stages.
   parameter PE_ACC_LATENCY = 3;

   //--------------------------------------------------------------------------
   // Signals
   // Connect these to the ports of the datapath instance
   reg       clk;
   reg       clr_n;

   reg [$clog2(K > 0 ? K : 1)-1:0] k_idx_in;

   reg                             en_a_brams_in;
   reg [N_BANKS * ($clog2(N_BANKS) + ((M/N_BANKS * K > 0) ? $clog2(M/N_BANKS * K) : 1)) - 1:0] addr_a_brams_in;
   reg                                                                                         we_a_brams_in;
   reg [N_BANKS * DATA_WIDTH-1:0]                                                              din_a_brams_in;

   reg                                                                                         en_b_brams_in;
   reg [N_BANKS * ($clog2(N_BANKS) + ((K * N/N_BANKS > 0) ? $clog2(K * N/N_BANKS) : 1)) - 1:0] addr_b_brams_in;
   reg                                                                                         we_b_brams_in;
   reg [N_BANKS * DATA_WIDTH-1:0]                                                              din_b_brams_in;

   reg                                                                                         en_c_bram_in;
   reg                                                                                         we_c_bram_in;
   reg [ADDR_WIDTH_C-1:0]                                                                      addr_c_bram_in;
   reg [ADDR_WIDTH_PE_IDX-1:0]                                                                 pe_write_idx_in;

   reg                                                                                         pe_start_in;
   reg                                                                                         pe_valid_in_in;
   reg                                                                                         pe_last_in;

   reg                                                                                         pe_output_capture_en;
   reg                                                                                         pe_output_buffer_reset;

   wire [(PE_ROWS * PE_COLS * ACC_WIDTH_PE)-1:0]                                               pe_c_out_out;
   wire [(PE_ROWS * PE_COLS > 0 ? PE_ROWS * PE_COLS : 1)-1:0]                                  pe_outputs_valid_out;
   wire                                                                                        pe_output_buffer_valid_out;

   reg                                                                                         read_en_c;
   reg [ADDR_WIDTH_C-1:0]                                                                      read_addr_c;
   wire [ACC_WIDTH_PE-1:0]                                                                     dout_c;

   //--------------------------------------------------------------------------
   // Internal variables for file handling, loops, and address calculation
   integer                                                                                     i, j, k; // Loop variables
   integer                                                                                     test_case; // Current test case number (0 to NUM_TEST_CASES-1)
   integer                                                                                     pass_count; // Counter for passed test cases
   integer                                                                                     fail_count; // Counter for failed test cases
   integer                                                                                     total_errors; // Total element mismatches across all test cases
   integer                                                                                     errors;

   // Sample Input Matrices (M x K and K x N) and Expected Output Matrix (M x N)
   // Using simple integer values for demonstration.
   // For real testing, load from files using $readmemh or similar.
   reg [DATA_WIDTH-1:0]                                                                        matrix_A [M-1:0][K-1:0];
   reg [DATA_WIDTH-1:0]                                                                        matrix_B [K-1:0][N-1:0];
   reg [ACC_WIDTH_PE-1:0]                                                                      expected_C [M-1:0][N-1:0];
   reg [ACC_WIDTH_PE-1:0]                                                                      actual_C [M-1:0][N-1:0]; // Flattened array to store read results

   reg [N_BANKS * DATA_WIDTH-1:0]                                                              partitioned_matrix_A [N_BANKS-1:0];
   reg [N_BANKS * DATA_WIDTH-1:0]                                                              partitioned_matrix_B [N_BANKS-1:0];

   // Internal testbench arrays to hold matrix data and expected results
   reg [DATA_WIDTH-1:0]                                                                        testbench_A [0:M-1][0:K-1];
   reg [DATA_WIDTH-1:0]                                                                        testbench_B [0:K-1][0:N-1];

   // This stores the actual result read from the DUT's C BRAM

   // Testbench Control Parameters
   parameter                                                                                   NUM_TEST_CASES = 100; // How many test case directories to read (test_000 to test_099)
   // !! IMPORTANT: Update this path to where your test case directories are located !!
   // Use a reg array for the base path
   parameter [8*100-1:0]                                                                       TEST_CASE_DIR_BASE = "/home/lamar/Documents/git/matrix-multiplier/testcases"; // Base directory for test cases (Max 100 chars)
   parameter                                                                                   MAX_FILENAME_LEN = 150; // Maximum length for generated filenames


   //--------------------------------------------------------------------------
   // Instantiate the Unit Under Test (UUT)
   datapath #(.DATA_WIDTH (DATA_WIDTH), .M (M), .K (K), .N (N), .N_BANKS (N_BANKS), .PE_ROWS (PE_ROWS), .PE_COLS (PE_COLS))
   uut (
        .clk                        (clk),
        .clr_n                      (clr_n),

        .k_idx_in                   (k_idx_in),
        .en_a_brams_in              (en_a_brams_in),
        .addr_a_brams_in            (addr_a_brams_in),
        .we_a_brams_in              (we_a_brams_in),
        .din_a_brams_in             (din_a_brams_in),
        .en_b_brams_in              (en_b_brams_in),
        .addr_b_brams_in            (addr_b_brams_in),
        .we_b_brams_in              (we_b_brams_in),
        .din_b_brams_in             (din_b_brams_in),
        .en_c_bram_in               (en_c_bram_in),
        .we_c_bram_in               (we_c_bram_in),
        .addr_c_bram_in             (addr_c_bram_in),
        .pe_write_idx_in            (pe_write_idx_in),
        .pe_start_in                (pe_start_in),
        .pe_valid_in_in             (pe_valid_in_in),
        .pe_last_in                 (pe_last_in),
        .pe_output_capture_en       (pe_output_capture_en),
        .pe_output_buffer_reset     (pe_output_buffer_reset),

        .pe_c_out_out               (pe_c_out_out),
        .pe_outputs_valid_out       (pe_outputs_valid_out),
        .pe_output_buffer_valid_out (pe_output_buffer_valid_out),

        .read_en_c                  (read_en_c),
        .read_addr_c                (read_addr_c),
        .dout_c                     (dout_c)
        );

   //--------------------------------------------------------------------------
   // Clock Generation
   always begin
      #((CLK_PERIOD/2)) clk = ~clk;
   end

   //--------------------------------------------------------------------------
   // Test Sequence
   initial begin
      // Initialize signals
      clk = 0;
      clr_n = 0; // Assert reset
      k_idx_in = 0;
      en_a_brams_in = 0;
      addr_a_brams_in = 0;
      we_a_brams_in = 0;
      din_a_brams_in = 0;
      en_b_brams_in = 0;
      addr_b_brams_in = 0;
      we_b_brams_in = 0;
      din_b_brams_in = 0;
      en_c_bram_in = 0;
      we_c_bram_in = 0;
      addr_c_bram_in = 0;
      pe_write_idx_in = 0;
      pe_start_in = 0;
      pe_valid_in_in = 0;
      pe_last_in = 0;
      pe_output_capture_en = 0;
      pe_output_buffer_reset = 0;
      read_en_c = 0;
      read_addr_c = 0;
      errors = 0;
      fail_count = 0;
      pass_count = 0;

      $display("--------------------------------------------------");
      $display(" Starting Datapath Testbench ");
      $display(" Parameters: M=%0d, K=%0d, N=%0d, N_BANKS=%0d", M, K, N, N_BANKS);
      $display(" ADDR_WIDTH_A=%0d, ADDR_WIDTH_B=%0d, ADDR_WIDTH_C=%0d, ACC_WIDTH_PE=%0d", ADDR_WIDTH_A, ADDR_WIDTH_B, ADDR_WIDTH_C, ACC_WIDTH_PE);
      $display("--------------------------------------------------");

      // Apply reset for a few clock cycles
      #(CLK_PERIOD * 2) clr_n = 1; // Deassert reset
      // ----------------------------------------------------------------------------------- //
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
           execute_multiplication();
           $display("Multiplication execution sequence complete.");

           // Read Result from C BRAM via Port B and Verify
           $display("@%0t Reading result from C BRAM via Port B and verifying...", $time);
           verify_results();
           $display("Result verification complete.");
           #(CLK_PERIOD);

        end // for (test_case = 0; test_case < NUM_TEST_CASES; test_case = test_case + 1)

      $display("--------------------------------------------------");
      $display(" Pass cases : %d", pass_count);
      $display(" Fail cases : %d", fail_count);
      $display(" Testbench FAILED with %0d errors!", total_errors);
      $display("--------------------------------------------------");

      $finish; // End simulation
   end // initial begin


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
      reg [ACC_WIDTH_PE-1:0]       read_value_acc; // Declare variables at start of task
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
                   addr_a_brams_in[c * ADDR_WIDTH_A + ADDR_IN_BANK_WIDTH - 1 -: ADDR_IN_BANK_WIDTH] = r;

                   // bank idx
                   addr_a_brams_in[c * ADDR_WIDTH_A + ADDR_WIDTH_A - 1 -: BANK_IDX_WIDTH] = c;
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
                   addr_b_brams_in[r * ADDR_WIDTH_A + ADDR_IN_BANK_WIDTH - 1 -: ADDR_IN_BANK_WIDTH] = c;

                   // bank idx
                   addr_b_brams_in[r * ADDR_WIDTH_A + ADDR_WIDTH_A - 1 -: BANK_IDX_WIDTH] = r;
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


   // Verify BRAM after loading
   task verify_brams_loading;
      begin

      end
   endtask


   // Task to simulate controller signals for multiplication execution
   task execute_multiplication;
      begin : execute_multiplication
         integer bank_idx;
         // Reset PE output buffer
         @(posedge clk) pe_output_buffer_reset = 1;
         @(posedge clk) pe_output_buffer_reset = 0;
         $display("@%0t: Starting input feeding sequence...", $time);

         // --- Pre-fetch BRAM data for k_step = 0 ---
         // Set BRAM addresses and enables for the first input cycle (k_step = 0)
         k_idx_in = 0; // Set index for the data being requested
         en_a_brams_in = 1;
         en_b_brams_in = 1;
         we_a_brams_in = 0;
         we_b_brams_in = 0;

         for (bank_idx = 0; bank_idx < N_BANKS; bank_idx = bank_idx + 1)
           begin
              // Address for A
              // addr in bank
              addr_a_brams_in[bank_idx * ADDR_WIDTH_A + ADDR_IN_BANK_WIDTH - 1 -: ADDR_IN_BANK_WIDTH] = 0;

              // bank idx
              addr_a_brams_in[bank_idx * ADDR_WIDTH_A + ADDR_WIDTH_A - 1 -: BANK_IDX_WIDTH] = bank_idx;

              // Address for B
              // addr in bank
              addr_b_brams_in[bank_idx * ADDR_WIDTH_B + ADDR_IN_BANK_WIDTH - 1 -: ADDR_IN_BANK_WIDTH] = 0;

              // bank idx
              addr_b_brams_in[bank_idx * ADDR_WIDTH_B + ADDR_WIDTH_B - 1 -: BANK_IDX_WIDTH] = bank_idx;
           end
         @(posedge clk); #1;

         $display("@%0t: Pre-fetching BRAM data for k_step = 0. Addresses set.", $time);
         @(posedge clk); #1; // Wait one cycle. BRAM read for k_step=0 is initiated.
         $display("@%0t: BRAM read for k_step = 0 initiated. Data will be ready next cycle.", $time);


         // Simulate accumulation steps (K cycles)
         for (k = 0; k < K; k = k + 1)
           begin
              begin
                 k_idx_in = k;
                 pe_valid_in_in = 1;
                 pe_start_in = (k == 0); // Start pulse on the first step
                 pe_last_in = (k == K - 1); // Last pulse on the final step
                 if(k < K - 1)
                   begin
                      for (bank_idx = 0; bank_idx < N_BANKS; bank_idx = bank_idx + 1)
                        begin
                           // Address for A
                           // addr in bank
                           addr_a_brams_in[bank_idx * ADDR_WIDTH_A + ADDR_IN_BANK_WIDTH - 1 -: ADDR_IN_BANK_WIDTH] = k + 1;

                           // bank idx
                           addr_a_brams_in[bank_idx * ADDR_WIDTH_A + ADDR_WIDTH_A - 1 -: BANK_IDX_WIDTH] = bank_idx;

                           // Address for B
                           // addr in bank
                           addr_b_brams_in[bank_idx * ADDR_WIDTH_B + ADDR_IN_BANK_WIDTH - 1 -: ADDR_IN_BANK_WIDTH] = k + 1;

                           // bank idx
                           addr_b_brams_in[bank_idx * ADDR_WIDTH_B + ADDR_WIDTH_B - 1 -: BANK_IDX_WIDTH] = bank_idx;
                        end
                      @(posedge clk);
                      #1;
                      $display("  Accumulation step %0d: pe_start_in=%0b, pe_valid_in_in=%0b, pe_last_in=%0b", k, pe_start_in, pe_valid_in_in, pe_last_in);
                   end // if (k < K - 1)
              end
           end




         // Deassert PE input control signals after accumulation cycles
         @(posedge clk) begin
            pe_valid_in_in = 0;
            pe_start_in = 0;
            pe_last_in = 0;
            en_a_brams_in = 0; // Deassert BRAM enables
            en_b_brams_in = 0;
         end

         // Wait for PEs to finish and output valid data
         // This depends on PE_ACC_LATENCY
         $display("  Waiting for PE outputs to become valid (Latency = %0d cycles)...", PE_ACC_LATENCY);
         repeat (PE_ACC_LATENCY) @(posedge clk);

         // Capture PE outputs into the buffer
         $display("  Capturing PE outputs into buffer...");
         @(posedge clk) pe_output_capture_en = 1;
         @(posedge clk) pe_output_capture_en = 0;

         // Wait for buffer valid signal (should be high after capture)
         @(posedge clk);
         if (pe_output_buffer_valid_out ) begin
            $display("  PE output buffer valid.");
         end else begin
            $display("  Error: PE output buffer not valid after capture!");
            total_errors = total_errors + 1;

         end

         // Simulate writing from buffer to C BRAM
         $display("  Writing from buffer to C BRAM...");
         for (i = 0; i < PE_ROWS * PE_COLS; i = i + 1) begin
            @(posedge clk) begin
               pe_write_idx_in = i;
               en_c_bram_in = 1;
               we_c_bram_in = 1;
               addr_c_bram_in = i; // Flattened address for C BRAM
               // Data input to C BRAM is automatically connected from the buffer in datapath
               $display("  Writing buffer element %0d to C BRAM address %0d", i, i);
            end
         end
         @(posedge clk) begin
            en_c_bram_in = 0; // Deassert C BRAM write signals
            we_c_bram_in = 0;
         end

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
                           fail_count = fail_count + 1;
                        end // else begin
                      else
                        begin
                           $display("Test Case %0d PASS: C[%0d][%0d] = %h",
                                    test_case, row_v, col_v, actual_C[row_v][col_v]);
                           pass_count = pass_count + 1;
                        end // else: !if(actual_C[row_v][col_v] !== expected_C[row_v][col_v])
                      // $display("  C[%0d][%0d] matches: %h", row_v, col_v, actual_C[row_v][col_v]); // Uncomment for successful matches
                      // end
                   end
              end // for (row_v = 0; row_v < M; row_v = row_v + 1)
            read_en_c = 0;
         end
      end

   endtask


endmodule
