# **Avalon Wrapper and C Software for Matrix Multiplier IP**

This guide explains how to create an Avalon Memory-Mapped (MM) Slave wrapper for your matrix multiplier IP and write C software to interact with it from a Nios II processor in an Altera DE2 SoC.

## **1\. Understanding the Avalon Memory-Mapped (MM) Slave Interface**

The Avalon MM interface is a set of signals that allow a master component (like the Nios II CPU) to read from and write to registers or memory within a slave component (your matrix multiplier IP).  
Key Avalon MM Slave signals include:

* clk: Clock signal.  
* reset\_n: Asynchronous active-low reset.  
* address: Specifies the address of the register or memory location being accessed within the slave. The width depends on the number of accessible locations.  
* chipselect: Asserted when the slave is selected for a transaction.  
* read: Asserted by the master for a read transaction.  
* write: Asserted by the master for a write transaction.  
* writedata: Data being written from the master to the slave during a write transaction. The width matches the data width of the interface.  
* readdata: Data being read from the slave back to the master during a read transaction. The width matches the data width of the interface.  
* waitrequest (Optional but Recommended): Asserted by the slave to pause the master if the slave is not ready to complete the transaction immediately.

Your Avalon wrapper module will have these Avalon signals as its ports and translate them into the control and data signals required by your core datapath module.

## **2\. Defining Memory-Mapped Registers**

Before writing the wrapper, you need to decide which signals from your datapath and matrix\_controller modules the Nios II processor needs to access. These will become your memory-mapped registers.  
Consider the functionality you need to expose:

* **Control Registers (Writeable by Nios II):**  
  * start\_mult: To initiate a matrix multiplication.  
  * Maybe registers to load matrix A and B data (if you want to load them via the Nios II instead of the testbench). This would require exposing BRAM write ports through the wrapper.  
  * rst\_n (or a synchronous reset signal): To reset the IP.  
* **Status Registers (Readable by Nios II):**  
  * mult\_done: To check if the multiplication is complete.  
  * pe\_output\_buffer\_valid\_out: To check if the PE output buffer contains valid data ready to be read.  
  * Maybe registers to read the final result from the C BRAM (if you want to read them via the Nios II). This would require exposing the C BRAM read port through the wrapper.  
* **Data Ports (Writeable/Readable by Nios II):**  
  * If loading/reading matrices via Nios II, you'll need data input/output ports mapped to registers.

Let's assume a simple set of registers for control and status:

* Address 0: Control Register (Write)  
  * Bit 0: start\_mult  
  * Bit 1: reset (synchronous reset for the IP)  
* Address 1: Status Register (Read)  
  * Bit 0: mult\_done  
  * Bit 1: pe\_output\_buffer\_valid\_out  
* Address 2: C BRAM Read Address (Write)  
  * Bits \[ADDR\_WIDTH\_C-1:0\]: read\_addr\_c  
* Address 3: C BRAM Read Data (Read)  
  * Bits \[ACC\_WIDTH\_PE-1:0\]: dout\_c

You would also need a mechanism to load A and B matrices if not pre-loaded. This could involve dedicated write addresses for A and B BRAMs, potentially using the same Port A interface as the controller during execution, but controlled by the Nios II processor when start\_mult is low. For simplicity in this example, we'll focus on the execution phase control and result reading.

## **3\. Creating the Avalon Wrapper (Conceptual Verilog)**

This is a conceptual example. You'll need to adapt it to your specific datapath and controller module ports and the registers you defined.  
//----------------------------------------------------------------------------  
// Module: matrix\_multiplier\_avalon\_wrapper  
// Description: Avalon Memory-Mapped Slave wrapper for the matrix multiplier IP.  
//              Connects the Nios II processor (via Avalon bus) to the datapath  
//              and controller modules.  
//  
// Assumptions:  
// \- Instantiates the 'datapath' and 'matrix\_controller' modules.  
// \- Exposes control and status signals as memory-mapped registers.  
// \- Assumes a simple register map (defined below).  
// \- Assumes DATA\_WIDTH, M, K, N, N\_BANKS, PE\_ROWS, PE\_COLS are parameters  
//   passed down from the top level or defined here.  
//----------------------------------------------------------------------------  
module matrix\_multiplier\_avalon\_wrapper  
\#(  
  parameter DATA\_WIDTH \= 16,  
  parameter M \= 8,  
  parameter K \= 8,  
  parameter N \= 8,  
  parameter N\_BANKS \= 8,  
  parameter PE\_ROWS \= M,  
  parameter PE\_COLS \= N,  
  parameter ID\_WIDTH \= 3 // Width of the Avalon address bus (e.g., 3 bits for 8 addresses)  
)  
(  
  // Avalon MM Slave Ports  
  input wire                      clk,  
  input wire                      reset\_n, // Asynchronous active-low reset (connect to rst\_n)  
  input wire \[ID\_WIDTH-1:0\]       address,  
  input wire                      chipselect,  
  input wire                      read,  
  input wire                      write,  
  input wire \[DATA\_WIDTH-1:0\]     writedata, // Assuming Avalon data width matches DATA\_WIDTH for simplicity  
  output wire \[DATA\_WIDTH-1:0\]    readdata,  
  output wire                     waitrequest // Simple waitrequest (high when busy)  
);

  // Derived Parameters (matching datapath/controller)  
  parameter ADDR\_WIDTH\_A\_BANK \= (M/N\_BANKS \* K \> 0\) ? $clog2(M/N\_BANKS \* K) : 1;  
  parameter ADDR\_WIDTH\_B\_BANK \= (K \* N/N\_BANKS \> 0\) ? $clog2(K \* N/N\_BANKS) : 1;  
  parameter ADDR\_WIDTH\_C \= (M \* N \> 0\) ? $clog2(M \* N) : 1;  
  parameter ACC\_WIDTH\_PE \= DATA\_WIDTH \* 2 \+ ((K \> 1\) ? $clog2(K) : 1);  
  parameter N\_PE \= PE\_ROWS \* PE\_COLS;

  // Internal signals for connecting to datapath and controller  
  wire \[$clog2(K \> 0 ? K : 1)-1:0\]     k\_idx\_in;  
  wire \[N\_BANKS-1:0\]                   en\_a\_brams\_in;  
  wire \[N\_BANKS \* ADDR\_WIDTH\_A\_BANK \- 1:0\] addr\_a\_brams\_in;  
  wire \[N\_BANKS-1:0\]                   we\_a\_brams\_in;  
  wire \[N\_BANKS \* DATA\_WIDTH \- 1:0\]    din\_a\_brams\_in;  
  wire \[N\_BANKS-1:0\]                   en\_b\_brams\_in;  
  wire \[N\_BANKS \* ADDR\_WIDTH\_B\_BANK \- 1:0\] addr\_b\_brams\_in;  
  wire \[N\_BANKS-1:0\]                   we\_b\_brams\_in;  
  wire \[N\_BANKS \* DATA\_WIDTH \- 1:0\]    din\_b\_brams\_in;  
  wire                                 en\_c\_bram\_in;  
  wire                                 we\_c\_bram\_in;  
  wire \[ADDR\_WIDTH\_C-1:0\]              addr\_c\_bram\_in;  
  wire \[$clog2(N\_PE \> 0 ? N\_PE : 1)-1:0\] pe\_write\_idx\_in;  
  wire                                 pe\_start\_in;  
  wire                                 pe\_valid\_in\_in;  
  wire                                 pe\_last\_in;  
  wire                                 pe\_output\_capture\_en;  
  wire                                 pe\_output\_buffer\_reset;

  // Status signals from datapath to controller  
  wire \[(PE\_ROWS \* PE\_COLS \> 0 ? PE\_ROWS \* PE\_COLS : 1)-1:0\] pe\_outputs\_valid\_out;  
  wire                                 pe\_output\_buffer\_valid\_out;  
  wire \[(PE\_ROWS \* PE\_COLS \* ACC\_WIDTH\_PE)-1:0\] pe\_c\_out\_out; // Flattened PE outputs before buffer  
  wire \[ACC\_WIDTH\_PE-1:0\]              dout\_c; // Output from C BRAM read port

  // Status signal from controller  
  wire                                 mult\_done;

  // Internal registers to hold control and status values accessible by Nios II  
  reg                                  start\_mult\_reg;  
  reg                                  sync\_reset\_reg; // Synchronous reset derived from Avalon reset  
  reg \[ADDR\_WIDTH\_C-1:0\]               c\_read\_addr\_reg; // Register for C BRAM read address

  // Internal logic for waitrequest (simple example: busy when not in IDLE or DONE)  
  // You'll need to connect to the controller's state or a busy signal  
  wire controller\_busy; // Assume controller has a 'busy' output (high when not IDLE or DONE)  
  assign waitrequest \= chipselect && (read || write) && controller\_busy; // Assert waitrequest if selected and controller is busy

  // Instantiate the Controller  
  matrix\_controller  
  \#(  
    .DATA\_WIDTH(DATA\_WIDTH),  
    .M(M),  
    .K(K),  
    .N(N),  
    .N\_BANKS(N\_BANKS),  
    .PE\_ROWS(PE\_ROWS),  
    .PE\_COLS(PE\_COLS)  
  )  
  controller\_inst (  
    .clk(clk),  
    .rst\_n(reset\_n), // Connect Avalon reset to controller reset  
    .start\_mult(start\_mult\_reg), // Connect to internal start\_mult register

    // Status Inputs from Datapath  
    .pe\_outputs\_valid\_out(pe\_outputs\_valid\_out),  
    .pe\_output\_buffer\_valid\_out(pe\_output\_buffer\_valid\_out),

    // Control Outputs to Datapath  
    .k\_idx\_in(k\_idx\_in),  
    .en\_a\_brams\_in(en\_a\_brams\_in),  
    .addr\_a\_brams\_in(addr\_a\_brams\_in),  
    .we\_a\_brams\_in(we\_a\_brams\_in),  
    .din\_a\_brams\_in(din\_a\_brams\_in),  
    .en\_b\_brams\_in(en\_b\_brams\_in),  
    .addr\_b\_brams\_in(addr\_b\_brams\_in),  
    .we\_b\_brams\_in(we\_b\_brams\_in),  
    .din\_b\_brams\_in(din\_b\_brams\_in),  
    .en\_c\_bram\_in(en\_c\_bram\_in),  
    .we\_c\_bram\_in(we\_c\_bram\_in),  
    .addr\_c\_bram\_in(addr\_c\_bram\_in),  
    .pe\_write\_idx\_in(pe\_write\_idx\_in),  
    .pe\_start\_in(pe\_start\_in),  
    .pe\_valid\_in\_in(pe\_valid\_in\_in),  
    .pe\_last\_in(pe\_last\_in),  
    .pe\_output\_capture\_en(pe\_output\_capture\_en),  
    .pe\_output\_buffer\_reset(pe\_output\_buffer\_reset),

    // Status Output to External System (connect to internal wire)  
    .mult\_done(mult\_done)  
    // Assuming controller has a 'busy' output  
    // .busy(controller\_busy)  
  );

  // Instantiate the Datapath  
  datapath  
  \#(  
    .DATA\_WIDTH(DATA\_WIDTH),  
    .M(M),  
    .K(K),  
    .N(N),  
    .N\_BANKS(N\_BANKS),  
    .PE\_ROWS(PE\_ROWS),  
    .PE\_COLS(PE\_COLS)  
  )  
  datapath\_inst (  
    .clk(clk),  
    .clr\_n(reset\_n), // Connect Avalon reset to datapath reset

    // Control Inputs from Controller (connected to controller outputs)  
    .k\_idx\_in(k\_idx\_in),  
    .en\_a\_brams\_in(en\_a\_brams\_in),  
    .addr\_a\_brams\_in(addr\_a\_brams\_in),  
    .we\_a\_brams\_in(we\_a\_brams\_in),  
    .din\_a\_brams\_in(din\_a\_brams\_in),  
    .en\_b\_brams\_in(en\_b\_brams\_in),  
    .addr\_b\_brams\_in(addr\_b\_brams\_in),  
    .we\_b\_brams\_in(we\_b\_brams\_in),  
    .din\_b\_brams\_in(din\_b\_brams\_in),  
    .en\_c\_bram\_in(en\_c\_bram\_in),  
    .we\_c\_bram\_in(we\_c\_bram\_in),  
    .addr\_c\_bram\_in(addr\_c\_bram\_in),  
    .pe\_write\_idx\_in(pe\_write\_idx\_in),  
    .pe\_start\_in(pe\_start\_in),  
    .pe\_valid\_in\_in(pe\_valid\_in\_in),  
    .pe\_last\_in(pe\_last\_in),  
    .pe\_output\_capture\_en(pe\_output\_capture\_en),  
    .pe\_output\_buffer\_reset(pe\_output\_buffer\_reset),

    // Status Outputs to Controller  
    .pe\_outputs\_valid\_out(pe\_outputs\_valid\_out),  
    .pe\_output\_buffer\_valid\_out(pe\_output\_buffer\_valid\_out),

    // External C BRAM Read Interface (connected to internal register and output)  
    .read\_en\_c(read), // Enable C BRAM read when Nios II reads from C data address  
    .read\_addr\_c(c\_read\_addr\_reg), // Connect to internal read address register  
    .dout\_c(dout\_c) // Connect to internal wire  
  );

  // Logic to handle Avalon transactions and map to internal signals/registers  
  always @(posedge clk or negedge reset\_n) begin  
    if (\!reset\_n) begin  
      start\_mult\_reg \<= 1'b0;  
      sync\_reset\_reg \<= 1'b1; // Assert synchronous reset on power-up/asynchronous reset  
      c\_read\_addr\_reg \<= 'b0;  
    end else begin  
      sync\_reset\_reg \<= 1'b0; // Deassert synchronous reset after one cycle

      if (chipselect && write) begin  
        // Write transactions  
        case (address)  
          8'd0: begin // Control Register  
            start\_mult\_reg \<= writedata\[0\]; // Assuming start\_mult is bit 0  
            // Optionally handle synchronous reset from Nios II here  
            // sync\_reset\_reg \<= writedata\[1\]; // Assuming reset is bit 1  
          end  
          8'd2: begin // C BRAM Read Address Register  
            c\_read\_addr\_reg \<= writedata\[ADDR\_WIDTH\_C-1:0\]; // Capture the address to read from C BRAM  
          end  
          // Add cases for loading A and B matrices if needed  
          default: begin  
            // Ignore writes to undefined addresses  
          end  
        endcase  
      end else begin  
         // Deassert start\_mult\_reg if start\_mult input from Nios II is pulse-like  
         // If Nios II provides a level signal, this might need different logic.  
         // Assuming Nios II pulses start\_mult high for one cycle:  
         start\_mult\_reg \<= 1'b0;  
      end  
    end  
  end

  // Logic for read transactions  
  // readdata is typically combinational based on address and chipselect/read  
  assign readdata \= chipselect && read ?  
                    (address \== 8'd1 ? {{(DATA\_WIDTH-2){1'b0}}, pe\_output\_buffer\_valid\_out, mult\_done} : // Status Register  
                     address \== 8'd3 ? dout\_c\[DATA\_WIDTH-1:0\] : // C BRAM Read Data (assuming DATA\_WIDTH matches Avalon)  
                     {DATA\_WIDTH{1'b0}}) : // Default to 0 for other addresses or when not reading  
                    {DATA\_WIDTH{1'b0}}; // Output 0 when not selected or not reading

  // Simple busy signal (replace with actual controller busy signal if available)  
  assign controller\_busy \= \!(controller\_inst.current\_state \== controller\_inst.IDLE || controller\_inst.current\_state \== controller\_inst.DONE);

endmodule // matrix\_multiplier\_avalon\_wrapper  
