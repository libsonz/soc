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
// - Assumes Avalon data width matches DATA_WIDTH.
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
    input wire                                                 clk,
    input wire                                                 reset_n,    // Asynchronous active-low reset (connect to rst_n)
    input wire [ID_WIDTH-1:0]                                  address,
    input wire                                                 chipselect,
    input wire                                                 read,
    input wire                                                 write,
    // Assuming Avalon data width matches DATA_WIDTH for simplicity.
    // If ACC_WIDTH_PE is wider, you might need wider Avalon ports or multiple reads/writes.
    input wire [DATA_WIDTH-1:0]                                writedata,
    output wire [DATA_WIDTH * 2 + ((K > 1) ? $clog2(K) : 1):0] readdata,
    output wire                                                waitrequest // Simple waitrequest (high when busy)
    );

   // Derived Parameters (matching top module/datapath/controller)
   // Hardcoded address widths to avoid $clog2 synthesis issues if necessary
   parameter ADDR_WIDTH_A = $clog2(N_BANKS) + (M/N_BANKS * K > 0) ? $clog2(M/N_BANKS * K) : 1;
   parameter ADDR_WIDTH_B = $clog2(N_BANKS) + (K * N/N_BANKS > 0) ? $clog2(K * N/N_BANKS) : 1;
   parameter ADDR_WIDTH_C = (M * N > 0) ? $clog2(M * N) : 1;
   parameter ACC_WIDTH_PE = DATA_WIDTH * 2 + ((K > 1) ? $clog2(K) : 1); // PE accumulator width must match
   parameter N_PE = PE_ROWS * PE_COLS; // Total number of PEs


   // Internal registers to hold control values written by Nios II
   reg       start_mult_reg;
   reg       reset_reg; // Register to pulse the reset signal
   reg [ADDR_WIDTH_C-1:0] c_addr_reg; // Register for C BRAM read address

   // Internal registers for A and B BRAM loading via Nios II (connected to top-level Port A inputs)
   // These registers capture the address and data written by Nios II.
   reg [ADDR_WIDTH_A-1:0] a_addr_reg; // Address for A banks (broadcast)
   reg [DATA_WIDTH-1:0]   a_data_reg; // Data for A banks (broadcast)
   reg                    a_en_reg; // Enable/Write Enable pulse for A banks
   reg                    a_we_reg;


   reg [ADDR_WIDTH_B-1:0] b_addr_reg; // Address for A banks (broadcast)
   reg [DATA_WIDTH-1:0]   b_data_reg; // Data for A banks (broadcast)
   reg                    b_en_reg; // Enable/Write Enable pulse for A banks
   reg                    b_we_reg;

   // Wires to connect to the top instance
   wire                   top_mult_done;
   wire [ACC_WIDTH_PE-1:0] top_dout_c;

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
             .rst_n                              (reset_n), // Connect Avalon reset to top-level reset

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
             .read_en_c                          (read && chipselect && (address == 8'd2 || address == 8'd3)), // Enable C BRAM read when Nios II reads C address or data
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
             reset_reg <= 1'b0; // Deassert reset pulse
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
             reset_reg <= 1'b0;
             a_we_reg <= 'b0; // Deassert pulse
             a_en_reg <= 'b0; // Initialize pulse register
             b_we_reg <= 'b0; // Deassert pulse
             b_en_reg <= 'b0; // Initialize pulse register


             if (chipselect && write)
               begin
                  // Write transactions
                  case (address)
                    8'd0: begin // Control Register
                       start_mult_reg <= writedata[0]; // Assuming start_mult is bit 0 (pulse)
                       reset_reg <= writedata[1]; // Assuming reset pulse is bit 1 (pulse)
                    end
                    8'd2: begin // C BRAM Read Address Register (Nios II writes the address it wants to read from C)
                       c_addr_reg <= writedata[ADDR_WIDTH_C-1:0]; // Capture the address to read from C BRAM
                    end
                    8'd4: begin // A BRAM Load Address Register (Nios II writes the address for A BRAM via Port A)
                       a_en_reg <= 1;
                       a_addr_reg <= writedata[ADDR_WIDTH_A-1:0]; // Capture the address
                    end
                    8'd5: begin // A BRAM Load Data Register (Nios II writes the data for A BRAM via Port A)
                       a_data_reg <= writedata[DATA_WIDTH-1:0]; // Capture the data
                       a_we_reg <= 1; // Pulse high to trigger A BRAM write via Port A for all banks
                       a_en_reg <= 1; // Pulse high to trigger A BRAM write via Port A for all banks
                    end
                    8'd6: begin // B BRAM Load Address Register (Nios II writes the address for B BRAM via Port A)
                       b_en_reg <= 1;
                       b_addr_reg <= writedata[ADDR_WIDTH_B-1:0]; // Capture the address
                    end
                    8'd7: begin // B BRAM Load Data Register (Nios II writes the data for B BRAM via Port A)
                       b_data_reg <= writedata[DATA_WIDTH-1:0]; // Capture the data
                       b_we_reg <= 1; // Pulse high to trigger B BRAM write via Port A for all banks
                       b_en_reg <= 1; // Pulse high to trigger B BRAM write via Port A for all banks
                    end
                    default: begin
                       // Ignore writes to undefined addresses
                    end
                  endcase
               end
          end
     end

   // Logic for read transactions
   // readdata is typically combinational based on address and chipselect/read
   // Need to handle ACC_WIDTH_PE possibly being wider than DATA_WIDTH for dout_c
   assign readdata = chipselect && read ?
                     (address == 8'd1 ? {{(ACC_WIDTH_PE-1){1'b0}}, top_mult_done} : // Status Register (mult_done on bit 0)
                      address == 8'd3 ? top_dout_c[ACC_WIDTH_PE-1:0] : // C BRAM Read Data (output lower bits)
                      {DATA_WIDTH{1'b0}}) : // Default to 0 for other addresses or when not reading
                     {DATA_WIDTH{1'b0}}; // Output 0 when not selected or not reading

   // Simple waitrequest: high if chipselect is asserted AND (read or write) is asserted
   // AND the top module is busy (mult_done is low and start_mult_reg was high)
   // Or if Nios II is writing to the load data registers (addresses 5 or 7),
   // as these writes take one cycle.
   assign waitrequest = chipselect && (read || write) &&
                        ((start_mult_reg || !top_mult_done) || // Busy during execution
                         (write && (address == 8'd5 || address == 8'd7))); // Busy during load data write


endmodule
