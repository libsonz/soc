`timescale 1ns/1ps
module controller
  #(
    parameter DATA_WIDTH = 16, // Data width of matrix elements A and B
    parameter M = 3,           // Number of rows in Matrix A and C
    parameter K = 3,           // Number of columns in Matrix A and rows in Matrix B
    parameter N = 3,           // Number of columns in Matrix B and C
    parameter N_BANKS = 3,     // Number of BRAM banks for Matrix A and B

    // Parameters for the 2D PE Array dimensions (Must match datapath)
    parameter PE_ROWS = M,     // Number of PE rows = M
    parameter PE_COLS = N      // Number of PE columns = N
    )
   (
    input wire                                                                                         clk,                        // Clock signal
    input wire                                                                                         rst_n,                      // Asynchronous active-low reset (connect to datapath clr_n)
    input wire                                                                                         start_mult,                 // Start signal from external system

    // Status Inputs from Datapath
    input wire [(PE_ROWS * PE_COLS)-1:0]                                                               pe_outputs_valid_out,       // Flattened PE output_valid signals
    input wire                                                                                         pe_output_buffer_valid_out, // Flag indicating valid data in the buffer

    // Control Outputs to Datapath
    output reg [$clog2(K)-1:0]                                                                         k_idx_in,                   // Current index for accumulation (0 to K-1)

    output reg                                                                                         en_a_brams_in,              // Enable for A banks
    output reg [N_BANKS * ($clog2(N_BANKS) + ((M/N_BANKS * K > 0) ? $clog2(M/N_BANKS * K) : 1)) - 1:0] addr_a_brams_in,            // Address for A banks
    output reg                                                                                         we_a_brams_in,              // Write enable for A banks (kept low during mult execution)

    output reg                                                                                         en_b_brams_in,              // Enable for B banks
    output reg [N_BANKS * ($clog2(N_BANKS) + ((K * N/N_BANKS > 0) ? $clog2(K * N/N_BANKS) : 1)) - 1:0] addr_b_brams_in,            // Address for B banks
    output reg                                                                                         we_b_brams_in,              // Write enable for B banks (kept low during mult execution)

    output reg                                                                                         en_c_bram_in,               // Enable for writing to C BRAM
    output reg                                                                                         we_c_bram_in,               // Write enable for C BRAM
    output reg [((M * N > 0) ? $clog2(M * N) : 1)-1:0]                                                 addr_c_bram_in,             // Address for writing to C BRAM
    output reg [$clog2(PE_ROWS*PE_COLS)-1:0]                                                           pe_write_idx_in,            // Index for writing PE outputs from buffer (0 to PE_ROWS*PE_COLS-1)

    output reg                                                                                         pe_start_in,                // Start signal for PEs (initialize accumulation)
    output reg                                                                                         pe_valid_in_in,             // Valid input signal for PEs
    output reg                                                                                         pe_last_in,                 // Last input signal for PEs

    output reg                                                                                         pe_output_capture_en,       // Enable to capture PE outputs into buffer
    output reg                                                                                         pe_output_buffer_reset,     // Reset the PE output buffer

    // Status Output to External System
    output reg                                                                                         mult_done                   // Signal indicating multiplication is complete
    );

   parameter ADDR_WIDTH_A = ($clog2(N_BANKS) + ((M/N_BANKS * K > 0) ? $clog2(M/N_BANKS * K) : 1));
   parameter ADDR_WIDTH_B = ($clog2(N_BANKS) + ((K * N/N_BANKS > 0) ? $clog2(K * N/N_BANKS) : 1));
   parameter ADDR_WIDTH_A_BANK = (M/N_BANKS * K > 0) ? $clog2(M/N_BANKS * K) : 1;
   parameter ADDR_WIDTH_B_BANK = (K * N/N_BANKS > 0) ? $clog2(K * N/N_BANKS) : 1;
   parameter ADDR_WIDTH_C = (M * N > 0) ? $clog2(M * N) : 1;
   parameter ACC_WIDTH_PE = DATA_WIDTH * 2 + ((K > 1) ? $clog2(K) : 1); // PE accumulator width must match
   parameter ADDR_WIDTH_BANK = $clog2(N_BANKS); // Width of the bank index in the new address format

   // PE pipeline latency from input registration to output_valid high
   localparam PE_ACC_LATENCY = 3;

   // State Machine Definition using localparam
   localparam [3:0] // Adjust width based on the number of states (8 states -> 4 bits needed)
                    IDLE             = 4'd0, // Waiting for start_mult
                    RESET_BUFFER     = 4'd1, // Resetting the PE output buffer
                    PRE_FETCH_BRAM   = 4'd2, // Initiate BRAM read for k_step = 0
                    ACCUMULATE       = 4'd3, // Feeding inputs to PEs for K cycles
                    WAIT_PE_DONE     = 4'd4, // Waiting for PEs to signal valid outputs
                    CAPTURE_OUTPUT   = 4'd5, // Pulsing capture enable
                    WRITE_C_BRAM     = 4'd6, // Writing captured outputs to C BRAM
                    DONE             = 4'd7; // Multiplication complete


   reg [3:0]        current_state, next_state; // State registers

   // Internal Registers
   reg [$clog2(K):0] k_step_cnt; // Counter for accumulation steps (0 to K)
   reg [$clog2(PE_ROWS*PE_COLS):0] write_c_cnt; // Counter for writing to C BRAM (0 to PE_ROWS*PE_COLS)
   integer                         bank_idx; // Loop variable for address calculation


   // State Transition Logic (Synchronous)
   always @(posedge clk or negedge rst_n)
     begin
        if (!rst_n)
          begin
             current_state <= IDLE;
          end
        else
          begin
             current_state <= next_state;
          end
     end

   // Next State Logic (Combinational) and Output Logic (Combinational/Synchronous)
   always @(*)
     begin
        // Default values for outputs to avoid latches
        next_state = current_state;
        k_idx_in = k_step_cnt; // k_idx_in tracks the current step being fed
        en_a_brams_in = 'b0;
        addr_a_brams_in = 'b0;
        we_a_brams_in = 'b0; // Keep write enables low during execution
        en_b_brams_in = 'b0;
        addr_b_brams_in = 'b0;
        we_b_brams_in = 'b0; // Keep write enables low during execution
        en_c_bram_in = 1'b0;
        we_c_bram_in = 1'b0;
        addr_c_bram_in = 'b0;
        pe_write_idx_in = write_c_cnt; // pe_write_idx_in tracks the current element being written
        pe_start_in = 1'b0;
        pe_valid_in_in = 1'b0;
        pe_last_in = 1'b0;
        pe_output_capture_en = 1'b0;
        pe_output_buffer_reset = 1'b0;
        mult_done = 1'b0;

        case (current_state)
          IDLE: begin
             if (start_mult) begin
                next_state = RESET_BUFFER;
             end
          end

          RESET_BUFFER: begin
             pe_output_buffer_reset = 1'b1; // Assert reset for one cycle
             next_state = PRE_FETCH_BRAM; // Transition to pre-fetch state
          end

          PRE_FETCH_BRAM: begin
             pe_output_buffer_reset = 1'b0; // Deassert reset

             // Initiate BRAM read for k_step = 0
             // Set BRAM addresses and enables for the first input cycle (k_step = 0)
             en_a_brams_in = 1'b1;
             we_a_brams_in = 1'b0;
             en_b_brams_in = 1'b1;
             we_b_brams_in = 1'b0;

             for (bank_idx = 0; bank_idx < N_BANKS; bank_idx = bank_idx + 1)
               begin
                  // Address for A
                  // addr in bank
                  addr_a_brams_in[bank_idx * ADDR_WIDTH_A + ADDR_WIDTH_A_BANK - 1 -: ADDR_WIDTH_A_BANK] = 0;

                  // bank idx
                  addr_a_brams_in[bank_idx * ADDR_WIDTH_A + ADDR_WIDTH_A - 1 -: ADDR_WIDTH_BANK] = bank_idx;

                  // Address for B
                  // addr in bank
                  addr_b_brams_in[bank_idx * ADDR_WIDTH_B + ADDR_WIDTH_B_BANK - 1 -: ADDR_WIDTH_B_BANK] = 0;

                  // bank idx
                  addr_b_brams_in[bank_idx * ADDR_WIDTH_B + ADDR_WIDTH_B - 1 -: ADDR_WIDTH_BANK] = bank_idx;
               end // for (bank_idx = 0; bank_idx < N_BANKS; bank_idx = bank_idx + 1)
             #1;

             next_state = ACCUMULATE; // Transition to accumulate after one cycle (BRAM read initiated)
          end

          ACCUMULATE: begin
             pe_output_buffer_reset = 1'b0; // Keep reset deasserted

             // Drive PE control signals for the current k_step_cnt
             pe_valid_in_in = 1'b1;
             pe_start_in = (k_step_cnt == 0); // Start only on the first step (when k_step_cnt is 0)
             pe_last_in = (k_step_cnt == K - 1); // Last only on the final step

             // Drive BRAM Read Addresses and Enables for the *next* k_step_cnt + 1
             // Data for k_step_cnt is available from BRAMs from the previous cycle's address.
             // Now set addresses for the next cycle (k_step_cnt + 1).
             if (k_step_cnt < K - 1)
               begin // Only set addresses for the next step if it exists
                  en_a_brams_in = 1'b1;
                  en_b_brams_in = 1'b1;

                  for (bank_idx = 0; bank_idx < N_BANKS; bank_idx = bank_idx + 1)
                    begin
                       // Address for A
                       // addr in bank
                       addr_a_brams_in[bank_idx * ADDR_WIDTH_A + ADDR_WIDTH_A_BANK - 1 -: ADDR_WIDTH_A_BANK] = k_step_cnt + 1;

                       // bank idx
                       addr_a_brams_in[bank_idx * ADDR_WIDTH_A + ADDR_WIDTH_A - 1 -: ADDR_WIDTH_BANK] = bank_idx;

                       // Address for B
                       // addr in bank
                       addr_b_brams_in[bank_idx * ADDR_WIDTH_B + ADDR_WIDTH_B_BANK - 1 -: ADDR_WIDTH_B_BANK] = k_step_cnt + 1;

                       // bank idx
                       addr_b_brams_in[bank_idx * ADDR_WIDTH_B + ADDR_WIDTH_B - 1 -: ADDR_WIDTH_BANK] = bank_idx;
                    end
                  #1;
               end
             else
               begin
                  // On the last accumulation step (k_step_cnt == K-1), deassert BRAM enables
                  en_a_brams_in = 'b0;
                  en_b_brams_in = 'b0;
               end


             if (k_step_cnt == K - 1)
               begin
                  // Finished feeding the last input (k_step = K-1)
                next_state = WAIT_PE_DONE;
               end
             else
               begin
                  next_state = ACCUMULATE; // Stay in accumulate for K cycles
               end
          end

          WAIT_PE_DONE: begin
             // Wait until all PE outputs are valid
             // Deassert all PE control signals
             pe_valid_in_in = 1'b0;
             pe_start_in = 1'b0;
             pe_last_in = 1'b0;
             en_a_brams_in = 'b0; // Ensure BRAMs are disabled
             en_b_brams_in = 'b0;

             if (pe_outputs_valid_out == {(PE_ROWS * PE_COLS){1'b1}}) begin
                next_state = CAPTURE_OUTPUT;
             end else begin
                next_state = WAIT_PE_DONE;
             end
          end

          CAPTURE_OUTPUT: begin
             pe_output_capture_en = 1'b1; // Pulse capture enable for one cycle
             next_state = WRITE_C_BRAM;
          end

          WRITE_C_BRAM: begin
             pe_output_capture_en = 1'b0; // Deassert capture enable

             en_c_bram_in = 1'b1;
             we_c_bram_in = 1'b1;
             addr_c_bram_in = write_c_cnt; // Write to flattened address

             if (write_c_cnt == (PE_ROWS * PE_COLS) - 1) begin
                // Finished writing the last element
                next_state = DONE;
             end else begin
                next_state = WRITE_C_BRAM; // Write all elements
             end
          end

          DONE: begin
             mult_done = 1'b1; // Signal completion
             // Deassert C BRAM write signals
             en_c_bram_in = 1'b0;
             we_c_bram_in = 1'b0;

             if (!start_mult) begin // Go back to IDLE if start is deasserted
                next_state = IDLE;
             end else begin
                next_state = DONE; // Stay in DONE until start is deasserted
             end
          end

          default: begin // Should not happen
             next_state = IDLE;
          end
        endcase
     end

   // Counters (Synchronous)
   always @(posedge clk or negedge rst_n) begin
      if (!rst_n) begin
         k_step_cnt <= 0;
         write_c_cnt <= 0;
      end else begin
         case (current_state)
           ACCUMULATE: begin
              // Increment k_step_cnt for each accumulation cycle
              if (k_step_cnt < K) begin
                 k_step_cnt <= k_step_cnt + 1;
              end
           end
           WRITE_C_BRAM: begin
              // Increment write_c_cnt for each C BRAM write cycle
              if (write_c_cnt < (PE_ROWS * PE_COLS)) begin
                 write_c_cnt <= write_c_cnt + 1;
              end
           end
           RESET_BUFFER: begin
              // Reset counters when starting a new multiplication
              k_step_cnt <= 0;
              write_c_cnt <= 0;
           end
           DONE: begin
              // Reset counters when going back to IDLE
              if (next_state == IDLE) begin
                 k_step_cnt <= 0;
                 write_c_cnt <= 0;
              end
           end
           default: begin
              // Counters hold their value in other states
           end
         endcase
      end
   end

endmodule
