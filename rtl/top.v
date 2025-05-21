//----------------------------------------------------------------------------
// Module: matrix_multiplier_top
// Description: Top-level module connecting the datapath2 and matrix_controller.
//              Provides the main interface for the matrix multiplication system.
//              **Uses ONLY Port A for A and B BRAMs (for both loading and execution).**
//              The external system/testbench must drive the A/B BRAM Port A
//              inputs for loading when start_mult is low.
//----------------------------------------------------------------------------
module top
  #(
    parameter DATA_WIDTH = 16, // Data width of matrix elements A and B
    parameter M = 3,           // Number of rows in Matrix A and C
    parameter K = 3,           // Number of columns in Matrix A and rows in Matrix B
    parameter N = 3,           // Number of columns in Matrix B and C
    parameter N_BANKS = 3,     // Number of BRAM banks for Matrix A and B

    // Parameters for the 2D PE Array dimensions (Must match datapath/controller)
    parameter PE_ROWS = M,     // Number of PE rows = M
    parameter PE_COLS = N      // Number of PE columns = N
    )
   (
    input wire                                                                                         clk,             // Clock signal
    input wire                                                                                         rst_n,           // Asynchronous active-low reset

    // External Control Input
    input wire                                                                                         start_mult,      // Start signal to initiate multiplication

    // External Status Output
    output wire                                                                                        mult_done,       // Signal indicating multiplication is complete

    input wire                                                                                         en_a_brams_in,   // Enable for A banks (Port A)
    input wire [N_BANKS * ($clog2(N_BANKS) + ((M/N_BANKS * K > 0) ? $clog2(M/N_BANKS * K) : 1)) - 1:0] addr_a_brams_in, // Address for A banks (Port A)
    input wire                                                                                         we_a_brams_in,   // Write enable for A banks (Port A)
    input wire [N_BANKS * DATA_WIDTH - 1:0]                                                            din_a_brams_in,  // Data input for writing to A banks (Port A)

    input wire                                                                                         en_b_brams_in,   // Enable for B banks (Port A)
    input wire [N_BANKS * ($clog2(N_BANKS) + ((K * N/N_BANKS > 0) ? $clog2(K * N/N_BANKS) : 1)) - 1:0] addr_b_brams_in, // Address for B banks (Port A)
    input wire                                                                                         we_b_brams_in,   // Write enable for B banks (Port A)
    input wire [N_BANKS * DATA_WIDTH - 1:0]                                                            din_b_brams_in,  // Data input for writing to B banks (Port A)


    // External C BRAM Read Interface (for reading the final result)
    input wire                                                                                         read_en_c,       // External read enable for C BRAM Port B
    input wire [((M * N > 0) ? $clog2(M * N) : 1)-1:0]                                                 read_addr_c,     // External read address for C BRAM Port B
    output wire [(DATA_WIDTH * 2 + ((K > 1) ? $clog2(K) : 1))-1:0]                                     dout_c           // Data output from C BRAM
    );

   // Derived parameters (matching sub-modules)

   parameter ADDR_WIDTH_A = ($clog2(N_BANKS) + ((M/N_BANKS * K > 0) ? $clog2(M/N_BANKS * K) : 1));
   parameter ADDR_WIDTH_B = ($clog2(N_BANKS) + ((K * N/N_BANKS > 0) ? $clog2(K * N/N_BANKS) : 1));
   parameter ADDR_WIDTH_A_BANK = (M/N_BANKS * K > 0) ? $clog2(M/N_BANKS * K) : 1;
   parameter ADDR_WIDTH_B_BANK = (K * N/N_BANKS > 0) ? $clog2(K * N/N_BANKS) : 1;
   parameter ADDR_WIDTH_C = (M * N > 0) ? $clog2(M * N) : 1;
   parameter ACC_WIDTH_PE = DATA_WIDTH * 2 + ((K > 1) ? $clog2(K) : 1); // PE accumulator width must match
   parameter N_PE = PE_ROWS * PE_COLS; // Total number of PEs
   parameter ADDR_WIDTH_BANK = $clog2(N_BANKS); // Width of the bank index in the new address format

   // Internal Wires to connect Controller and Datapath
   // These wires carry the control signals from the controller to the datapath
   wire [$clog2(K)-1:0] k_idx_in;
   wire                 en_c_bram_in;
   wire                 we_c_bram_in;
   wire [ADDR_WIDTH_C-1:0] addr_c_bram_in;
   wire [$clog2(N_PE)-1:0] pe_write_idx_in;
   wire                    pe_start_in;
   wire                    pe_valid_in_in;
   wire                    pe_last_in;
   wire                    pe_output_capture_en;
   wire                    pe_output_buffer_reset;

   // Internal Wires to connect Datapath Status to Controller
   wire [(PE_ROWS * PE_COLS)-1:0] pe_outputs_valid_out;
   wire                           pe_output_buffer_valid_out;

   // Internal Wires to connect Controller Outputs to Datapath Inputs (for Port A)
   // These are the signals the controller *wants* to drive Port A with during execution.
   // The top module will need to select between these and the external inputs.
   wire [N_BANKS * ADDR_WIDTH_A - 1:0] ctrl_addr_a_brams;
   wire                                ctrl_en_a_brams;
   wire                                ctrl_we_a_brams;
   wire [N_BANKS * DATA_WIDTH - 1:0]   ctrl_din_a_brams;

   wire                                ctrl_en_b_brams;
   wire [N_BANKS * ADDR_WIDTH_B - 1:0] ctrl_addr_b_brams;
   wire                                ctrl_we_b_brams;
   wire [N_BANKS * DATA_WIDTH - 1:0]   ctrl_din_b_brams;


   // Logic to select between external loading inputs and controller execution inputs for Datapath Port A
   // This selection is based on the start_mult signal.
   // When start_mult is high, the controller drives the datapath's Port A inputs.
   // When start_mult is low, the external inputs drive the datapath's Port A inputs (for loading).
   wire                                datapath_en_a_brams;
   wire [N_BANKS * ADDR_WIDTH_A - 1:0] datapath_addr_a_brams;
   wire                                datapath_we_a_brams;
   wire [N_BANKS * DATA_WIDTH - 1:0]   datapath_din_a_brams;

   wire                                datapath_en_b_brams;
   wire [N_BANKS * ADDR_WIDTH_B - 1:0] datapath_addr_b_brams;
   wire                                datapath_we_b_brams;
   wire [N_BANKS * DATA_WIDTH - 1:0]   datapath_din_b_brams;

   assign datapath_en_a_brams = start_mult ? ctrl_en_a_brams : en_a_brams_in;
   assign datapath_addr_a_brams = start_mult ? ctrl_addr_a_brams : addr_a_brams_in;
   assign datapath_we_a_brams = start_mult ? ctrl_we_a_brams : we_a_brams_in;
   assign datapath_din_a_brams = start_mult ? ctrl_din_a_brams : din_a_brams_in;

   assign datapath_en_b_brams = start_mult ? ctrl_en_b_brams : en_b_brams_in;
   assign datapath_addr_b_brams = start_mult ? ctrl_addr_b_brams : addr_b_brams_in;
   assign datapath_we_b_brams = start_mult ? ctrl_we_b_brams : we_b_brams_in;
   assign datapath_din_b_brams = start_mult ? ctrl_din_b_brams : din_b_brams_in;


   // Instantiate the Datapath module
   datapath
     #(
       .DATA_WIDTH (DATA_WIDTH),
       .M          (M),
       .K          (K),
       .N          (N),
       .N_BANKS    (N_BANKS),
       .PE_ROWS    (PE_ROWS),
       .PE_COLS    (PE_COLS)
       )
   datapath_inst (
                  .clk                                (clk),
                  .clr_n                              (rst_n), // Connect top-level reset to datapath reset

                  // Connected to the selection logic (driven by external or controller)
                  .en_a_brams_in                      (datapath_en_a_brams),
                  .addr_a_brams_in                    (datapath_addr_a_brams),
                  .we_a_brams_in                      (datapath_we_a_brams),
                  .din_a_brams_in                     (datapath_din_a_brams),
                  .en_b_brams_in                      (datapath_en_b_brams),
                  .addr_b_brams_in                    (datapath_addr_b_brams),
                  .we_b_brams_in                      (datapath_we_b_brams),
                  .din_b_brams_in                     (datapath_din_b_brams),


                  // Connected to Controller Outputs  (Specific to Execution Flow)
                  .k_idx_in                           (k_idx_in),
                  .en_c_bram_in                       (en_c_bram_in),
                  .we_c_bram_in                       (we_c_bram_in),
                  .addr_c_bram_in                     (addr_c_bram_in),
                  .pe_write_idx_in                    (pe_write_idx_in),
                  .pe_start_in                        (pe_start_in),
                  .pe_valid_in_in                     (pe_valid_in_in),
                  .pe_last_in                         (pe_last_in),
                  .pe_output_capture_en               (pe_output_capture_en),
                  .pe_output_buffer_reset             (pe_output_buffer_reset),

                  // Connected to Controller Inputs   (Internal Wires)
                  .pe_outputs_valid_out               (pe_outputs_valid_out),
                  .pe_output_buffer_valid_out         (pe_output_buffer_valid_out),

                  // Connected to Top-Level Ports     (External C BRAM Read Interface)
                  .read_en_c                          (read_en_c),
                  .read_addr_c                        (read_addr_c),
                  .dout_c                             (dout_c) // Connects directly to top-level output
                  );

   // Instantiate the Controller module
   controller
     #(
       .DATA_WIDTH (DATA_WIDTH),
       .M          (M),
       .K          (K),
       .N          (N),
       .N_BANKS    (N_BANKS),
       .PE_ROWS    (PE_ROWS),
       .PE_COLS    (PE_COLS)
       )
   controller_inst (
                    .clk                             (clk),
                    .rst_n                           (rst_n), // Connect top-level reset to controller reset
                    .start_mult                      (start_mult), // Connect to top-level start signal

                    // Connected to Datapath Outputs (Internal Wires)
                    .pe_outputs_valid_out            (pe_outputs_valid_out),
                    .pe_output_buffer_valid_out      (pe_output_buffer_valid_out),

                    // Connected to Internal Wires   (Controller Outputs that feed the selection logic)
                    .k_idx_in                        (k_idx_in),
                    .en_a_brams_in                   (ctrl_en_a_brams), // Controller drives these wires
                    .addr_a_brams_in                 (ctrl_addr_a_brams), // Controller drives these wires
                    .we_a_brams_in                   (ctrl_we_a_brams), // Controller drives these wires
                    .en_b_brams_in                   (ctrl_en_b_brams), // Controller drives these wires
                    .addr_b_brams_in                 (ctrl_addr_b_brams), // Controller drives these wires
                    .we_b_brams_in                   (ctrl_we_b_brams), // Controller drives these wires
                    .en_c_bram_in                    (en_c_bram_in),
                    .we_c_bram_in                    (we_c_bram_in),
                    .addr_c_bram_in                  (addr_c_bram_in),
                    .pe_write_idx_in                 (pe_write_idx_in),
                    .pe_start_in                     (pe_start_in),
                    .pe_valid_in_in                  (pe_valid_in_in),
                    .pe_last_in                      (pe_last_in),
                    .pe_output_capture_en            (pe_output_capture_en),
                    .pe_output_buffer_reset          (pe_output_buffer_reset),

                    // Connected to Top-Level Output
                    .mult_done                       (mult_done) // Connects directly to top-level output
                    );

endmodule
