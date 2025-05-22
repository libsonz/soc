//----------------------------------------------------------------------------
// Module: matrix_multiplier_avalon_wrapper
// Description: Avalon Memory-Mapped Slave wrapper for the matrix multiplier IP.
//              Instantiates the user-provided 'top' module and connects
//              the Nios II processor (via Avalon bus) to its control, status,
//              and BRAM interfaces.
//
// Register Map:
// Address 0 (Write): Control Register
//   [0]: start_mult (pulse high to start multiplication)
//   [1]: reset (pulse low to assert asynchronous rst_n)
// Address 1 (Read): Status Register
//   [0]: mult_done
// Address 2 (Write): C BRAM Read Address
//   [ADDR_WIDTH_C-1:0]: read_addr_c (Address in flattened C BRAM)
// Address 3 (Read): C BRAM Read Data
//   [ACC_WIDTH-1:0]: dout_c
// Address 4 (Write): A BRAM Load Address (Broadcast to all banks)
//   [ADDR_WIDTH_A_BANK-1:0]: Address to load into A BRAMs via Port A
// Address 5 (Write): A BRAM Load Data (Broadcast to all banks)
//   [DATA_WIDTH-1:0]: Data to load into A BRAMs via Port A (Writing asserts en_a/we_a)
// Address 6 (Write): B BRAM Load Address (Broadcast to all banks)
//   [ADDR_WIDTH_B_BANK-1:0]: Address to load into B BRAMs via Port A
// Address 7 (Write): B BRAM Load Data (Broadcast to all banks)
//   [DATA_WIDTH-1:0]: Data to load into B BRAMs via Port A (Writing asserts en_b/we_b)
//
// Assumptions:
// - Assumes DATA_WIDTH, M, K, N, N_BANKS, PE_ROWS, PE_COLS are parameters
//   passed down from the top level or defined here.
// - Assumes Avalon data width matches N_BANKS * DATA_WIDTH.
// - The 'top' module handles the multiplexing of Port A inputs between
//   external loading (when start_mult is low) and internal controller
//   execution (when start_mult is high).
//----------------------------------------------------------------------------
module avalon_wrapper
  #(
    parameter DATA_WIDTH = 16,
    parameter M = 3,
    parameter K = 3,
    parameter N = 3,
    parameter N_BANKS = 3,
    parameter PE_ROWS = M,
    parameter PE_COLS = N,
    // ID_WIDTH needs to be wide enough for all defined addresses (0-7 -> 8 addresses -> 3 bits)
    parameter ID_WIDTH = 3
    )
   (
    // Avalon MM Slave Ports
    input wire                                  clk,
    input wire                                  reset_n,    // Asynchronous active-low reset (connect to rst_n)
    input wire [ID_WIDTH-1:0]                   address,
    input wire                                  chipselect,
    input wire                                  read,
    input wire                                  write,
    input wire [N_BANKS * DATA_WIDTH - 1:0]     writedata,
    // NEW: Add byteenable input
    input wire [(N_BANKS * DATA_WIDTH)/8 - 1:0] byteenable,
    // output reg [DATA_WIDTH * 2 + ((K > 1) ? $clog2(K) : 1):0] readdata,
    output reg [N_BANKS * DATA_WIDTH - 1:0]     readdata,
    output wire                                 waitrequest // Simple waitrequest (high when busy)
    );

   // Derived Parameters (matching top module/datapath/controller)
   // Hardcoded address widths to avoid $clog2 synthesis issues if necessary
   localparam DATA_IN_WIDTH = N_BANKS * DATA_WIDTH;
   localparam ADDR_WIDTH_A = $clog2(N_BANKS) + ((M/N_BANKS * K) > 0 ? $clog2(M/N_BANKS * K) : 1); // Corrected ternary operator parentheses
   localparam ADDR_WIDTH_B = $clog2(N_BANKS) + ((K * N/N_BANKS) > 0 ? $clog2(K * N/N_BANKS) : 1); // Corrected ternary operator parentheses
   localparam ADDR_WIDTH_C = (M * N > 0) ? $clog2(M * N) : 1;
   localparam ACC_WIDTH_PE = DATA_WIDTH * 2 + ((K > 1) ? $clog2(K) : 1); // PE accumulator width must match
   localparam N_PE = PE_ROWS * PE_COLS; // Total number of PEs


   // Internal registers to hold control values written by Nios II
   reg [ADDR_WIDTH_C-1:0] c_addr_reg; // Register for C BRAM read address
   reg                    start_mult_reg;
   reg                    clrn_reg; // Register to pulse the reset signal

   // Internal registers for A and B BRAM loading via Nios II (connected to top-level Port A inputs)
   // These registers capture the address and data written by Nios II.
   reg [N_BANKS * ($clog2(N_BANKS) + ((M/N_BANKS * K > 0) ? $clog2(M/N_BANKS * K) : 1)) - 1:0] a_addr_reg; // Address for A banks (broadcast)
   reg [DATA_IN_WIDTH-1:0]                                                                     a_data_reg; // Data for A banks (broadcast)
   reg                                                                                         a_en_reg; // Enable/Write Enable pulse for A banks
   reg                                                                                         a_we_reg;


   reg [N_BANKS * ($clog2(N_BANKS) + ((K * N/N_BANKS > 0) ? $clog2(K * N/N_BANKS) : 1)) - 1:0] b_addr_reg; // Address for A banks (broadcast)
   reg [DATA_IN_WIDTH-1:0]                                                                     b_data_reg; // Data for A banks (broadcast)
   reg                                                                                         b_en_reg; // Enable/Write Enable pulse for A banks
   reg                                                                                         b_we_reg;

   // Wires to connect to the top instance
   wire                             top_mult_done;
   wire [ACC_WIDTH_PE-1:0]          top_dout_c;

   // Loop variable for byteenable logic
   integer                          i;


   // Instantiate the user-provided 'top' module
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
   top_inst (
             .clk                                (clk),
             .rst_n                              (clrn_reg), // Connect Avalon reset to top-level reset

             // External Control Input           (from Avalon)
             .start_mult                         (start_mult_reg), // Connect to internal start_mult register

             // External Status Output           (to Avalon)
             .mult_done                          (top_mult_done), // Connect to internal wire

             // External A and B BRAM Interfaces (Port A - Driven by Avalon during load)
             // The 'top' module's internal logic selects between these and controller signals.
             .en_a_brams_in                      (a_en_reg), // Connect to generated load signals
             .addr_a_brams_in                    (a_addr_reg), // Connect to generated load signals
             .we_a_brams_in                      (a_we_reg), // Connect to generated load signals
             .din_a_brams_in                     (a_data_reg), // Connect to generated load signals

             .en_b_brams_in                      (b_en_reg), // Connect to generated load signals
             .addr_b_brams_in                    (b_addr_reg), // Connect to generated load signals
             .we_b_brams_in                      (b_we_reg), // Connect to generated load signals
             .din_b_brams_in                     (b_data_reg), // Connect to generated load signals


             // External C BRAM Read Interface   (from/to Avalon)
             .read_en_c                          (read && chipselect && (address == 8'd3)), // Enable C BRAM read when Nios II reads C address or data
             .read_addr_c                        (c_addr_reg), // Connect to internal read address register
             .dout_c                             (top_dout_c) // Connect to internal wire
             );



   // ------------------------------------------------------------------------- //
   // Logic to handle Avalon transactions and map to internal signals/registers //
   // ------------------------------------------------------------------------- //
   always @(posedge clk or negedge reset_n)
     begin
        if (!reset_n)
          begin
             start_mult_reg <= 1'b0;
             clrn_reg <= 1'b0; // Deassert reset pulse
             c_addr_reg <= 'b0;
             a_addr_reg <= 'b0;
             a_data_reg <= 'b0;
             a_we_reg <= 'b0; // Initialize pulse register
             a_en_reg <= 'b0; // Initialize pulse register
             b_addr_reg <= 'b0;
             b_data_reg <= 'b0;
             b_we_reg <= 'b0; // Initialize pulse register
             b_en_reg <= 'b0; // Initialize pulse register
          end
        else
          begin
             // Deassert pulse signals by default
             start_mult_reg <= 1'b0;
             clrn_reg <= 1'b1;
             a_we_reg <= 'b0; // Deassert pulse
             a_en_reg <= 'b0; // Initialize pulse register
             b_we_reg <= 'b0; // Deassert pulse
             b_en_reg <= 'b0; // Initialize pulse register


             if (chipselect && write)
               begin
                  // Write transactions
                  case (address)
                    8'h00:
                      begin // Control Register
                         start_mult_reg <= writedata[0]; // Assuming start_mult is bit 0 (pulse)
                         clrn_reg <= writedata[1]; // Assuming reset pulse is bit 1 (pulse)
                      end
                    8'h10:
                      begin // C BRAM Read Address Register (Nios II writes the address it wants to read from C)
                         c_addr_reg <= writedata[ADDR_WIDTH_C-1:0]; // Capture the address to read from C BRAM
                      end
                    8'h20:
                      begin // A BRAM Load Address Register (Nios II writes the address for A BRAM via Port A)
                         a_en_reg <= 1;
                         a_addr_reg <= writedata[N_BANKS * ($clog2(N_BANKS) + ((M/N_BANKS * K > 0) ? $clog2(M/N_BANKS * K) : 1)) - 1:0]; // Capture the address
                      end
                    8'h28:
                      begin // A BRAM Load Data Register (Nios II writes the data for A BRAM via Port A)
                         a_we_reg <= 1; // Pulse high to trigger A BRAM write via Port A for all banks
                         a_en_reg <= 1; // Pulse high to trigger A BRAM write via Port A for all banks
                         // NEW: Apply byte enables for a_data_reg
                         for (i = 0; i < DATA_IN_WIDTH / 8; i = i + 1) begin
                            if (byteenable[i]) begin
                               a_data_reg[(i*8) +: 8] <= writedata[(i*8) +: 8];
                            end
                         end
                      end
                    8'h30:
                      begin // B BRAM Load Address Register (Nios II writes the address for B BRAM via Port A)
                         b_en_reg <= 1;
                         b_addr_reg <= writedata[N_BANKS * ($clog2(N_BANKS) + ((K * N/N_BANKS > 0) ? $clog2(K * N/N_BANKS) : 1)) - 1:0]; // Capture the addressb_addr_reg <= writedata[ADDR_WIDTH_B-1:0]; // Capture the address
                      end
                    8'h38:
                      begin // B BRAM Load Data Register (Nios II writes the data for B BRAM via Port A)
                         b_we_reg <= 1; // Pulse high to trigger B BRAM write via Port A for all banks
                         b_en_reg <= 1; // Pulse high to trigger B BRAM write via Port A for all banks
                         // NEW: Apply byte enables for b_data_reg
                         for (i = 0; i < DATA_IN_WIDTH / 8; i = i + 1) begin
                            if (byteenable[i]) begin
                               b_data_reg[(i*8) +: 8] <= writedata[(i*8) +: 8];
                            end
                         end
                      end
                    default:
                      begin
                         // Ignore writes to undefined addresses
                      end
                  endcase
               end // if (chipselect && write)
             else if (chipselect && read)
               begin
                  case (address)
                    8'h08:
                      begin
                         // For status register (mult_done), place it at bit 0 and pad with zeros
                         readdata <= {ACC_WIDTH_PE{1'b0}} | top_mult_done;
                      end
                    8'h10:
                      begin
                         // For read_addr_c, pad with zeros
                         readdata <= {ACC_WIDTH_PE{1'b0}} | c_addr_reg;
                      end
                    8'h18:
                      begin
                         readdata <= top_dout_c;
                      end
                    default:
                      begin
                         readdata <= {ACC_WIDTH_PE{1'bx}}; // Return 'X' for undefined reads
                      end
                  endcase // case (address)
               end // if (chipselect && read)
          end // else: !if(!reset_n)

     end // always @ (posedge clk or negedge reset_n)

   assign waitrequest = chipselect && (read || write) &&
                        ((start_mult_reg || !top_mult_done) || // Busy during execution
                         (write && (address == 8'h28 || address == 8'd38))); // Busy during load data write


endmodule
