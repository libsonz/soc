/*
 * source.c
 *
 *  Created on: May 22, 2025
 *      Author: admin
 */

#include <stdio.h>     // For printf, standard input/output functions
#include <stdint.h> // For uint16_t, uint8_t, etc.
#include <inttypes.h> // For PRIu64, PRIx64, etc.
#include <system.h>    // Contains base addresses for Qsys components (like MATRIX_MULTIPLIER_WRAPPER_BASE)
#include <alt_types.h> // Contains Altera-specific data types like alt_u16, alt_u32, alt_u64

// --- Hardware Parameters & Memory Map ---
// These defines must match your Qsys/Platform Designer setup and Verilog implementation
#define DATA_WIDTH 16 // Width of one element in your matrices (16 bits)
#define M 4           // Matrix A rows / Matrix C rows
#define K 4           // Matrix A columns / Matrix B rows
#define N 4           // Matrix B columns / Matrix C columns
#define N_BANKS 4     // Number of 16-bit elements loaded in parallel (64-bit writedata / 16-bit element = 4)
#define MATRIX_DIM 4

#define BRAM_DATA_WIDTH (N_BANKS * DATA_WIDTH) // This evaluates to 64 bits (4 * 16)


#define BRAM_DATA_WIDTH (N_BANKS * DATA_WIDTH) // This evaluates to 64 bits (4 * 16)
#define ELEMENT_INDEX_BITS 2
#define ADDR_BITS 4

#define MM_BASE MM_IP_0_BASE // Base address of your custom hardware block

// Memory-Mapped Register Offsets (relative to MM_BASE)
// Your Verilog hardware must decode the Avalon 'address' signal to match these.
#define ADDR_CONTROL_OFFSET         0x00 // 64-bit control register (e.g., bit 0 for start)
#define ADDR_STATUS_OFFSET          0x08 // 64-bit status register (e.g., bit 0 for done)

#define ADDR_C_ADDR_OFFSET          0x10 // 64-bit register to specify address for C matrix BRAM read
#define ADDR_C_DATA_OFFSET          0x18 // 64-bit register to read C matrix elements (result of 16-bit * 16-bit is 32-bit)

#define ADDR_A_ADDR_OFFSET          0x20 // 64-bit register to specify address for A matrix BRAM write
#define ADDR_A_DATA_OFFSET          0x28 // 64-bit register for A matrix data (offset 0x10 + 8 bytes for 64-bit alignment)

#define ADDR_B_ADDR_OFFSET          0x30 // 64-bit register to specify address for B matrix BRAM write
#define ADDR_B_DATA_OFFSET          0x38 // 64-bit register for B matrix data (offset 0x20 + 8 bytes

// --- Sample Matrices (using alt_u16 for 16-bit elements) ---
alt_u16 matrix_A[M][K] = {
    { 1,  2,  3,  4},
    { 5,  6,  7,  8},
    { 9, 10, 11, 12},
    {13, 14, 15, 16}
};

alt_u16 matrix_B[K][N] = {
    {16, 15, 14, 13},
    {12, 11, 10,  9},
    { 8,  7,  6,  5},
    { 4,  3,  2,  1}
};


uint64_t compose_bram_address(uint8_t bank_idx, uint8_t element_idx) 
{
    uint64_t address = 0;
    // Set Element Index bits
    address |= element_idx;
    // Set Bank Index bits
    address |= bank_idx << ELEMENT_INDEX_BITS;

    // Higher 12 bits are 0 in this composed address, as per our assumption.
    return address;
}


void load_matrix_A_column(volatile uint64_t *addr,volatile uint64_t *data, uint16_t matrix_A[MATRIX_DIM][MATRIX_DIM], uint8_t col_idx) 
{   
    uint64_t data_buffer = 0; // Buffer to hold the data for the current column
    uint64_t addr_buffer = 0;

    if (col_idx >= MATRIX_DIM) {
        printf("Error: Column index %u out of bounds for Matrix A.\n", col_idx);
        return;
    }

    printf("Loading column %u of Matrix A into BRAM...\n", col_idx);
    for (uint8_t row_idx = 0; row_idx < MATRIX_DIM; row_idx++) {
        uint64_t data = matrix_A[row_idx][col_idx];

        uint64_t bram_address_to_write = compose_bram_address(row_idx, col_idx);
    
        addr_buffer |= (uint64_t)(bram_address_to_write) << (row_idx * ADDR_BITS);
        data_buffer |= (uint64_t)(data) << (row_idx * 16); // Shift the data to the correct position


    }   

    *(addr) = addr_buffer;
    *(data) = data_buffer;       

    printf(" Bram address : %" PRIu64 "\n", *addr);
    printf(" Bram data : %" PRIu64 "\n", *data);
    printf("Finish loading col %u of Matrix A into BRAM...\n", col_idx);
    printf("------------------------------------------------------------- \n");
}


// --- Function to load a specific column of Matrix A into BRAM ---
// Each A[r][col_idx] goes into BRAM[col_idx][r]
void load_matrix_B_row(volatile uint64_t *addr, volatile uint64_t *data, uint16_t matrix_A[MATRIX_DIM][MATRIX_DIM], uint8_t row_idx) 
{   
    uint64_t data_buffer = 0; // Buffer to hold the data for the current column
    uint64_t addr_buffer = 0;

    if (row_idx >= MATRIX_DIM) {
        printf("Error: Column index %u out of bounds for Matrix A.\n", row_idx);
        return;
    }

    printf("Loading row %u of Matrix B into BRAM...\n", row_idx);
    for (uint8_t col_idx = 0; col_idx < MATRIX_DIM; col_idx++) {
        uint64_t data = matrix_A[row_idx][col_idx];

        uint64_t bram_address_to_write = compose_bram_address(col_idx, row_idx);
    
        addr_buffer |= (uint64_t)(bram_address_to_write) << (col_idx * ADDR_BITS);
        data_buffer |= (uint64_t)(data) << (col_idx * 16); // Shift the data to the correct position


    }   

    *addr = addr_buffer;
    *data = data_buffer;       

    printf(" Bram address : %" PRIu64 "\n", *addr);
    printf(" Bram data : %" PRIu64 "\n", *data);
    printf(" Finish loading row %u of Matrix B into BRAM...\n", row_idx);
    printf("------------------------------------------------------------- \n");


}
//--- main Function ---
int main() {
    volatile uint64_t *addr_reg_ptr_A = (volatile uint64_t *)(MM_BASE + ADDR_A_ADDR_OFFSET);
    volatile uint64_t *data_reg_ptr_A = (volatile uint64_t *)(MM_BASE + ADDR_A_DATA_OFFSET); 
    volatile uint64_t *addr_reg_ptr_B = (volatile uint64_t *)(MM_BASE + ADDR_B_ADDR_OFFSET);
    volatile uint64_t *data_reg_ptr_B = (volatile uint64_t *)(MM_BASE + ADDR_B_DATA_OFFSET); 
    printf("Nios II 4x4 Matrix Multiplier Control Software Starting...\n");
    printf("------------------------------------------------------------- \n");
    printf("Loading matrix A...\n");
    for (int col = 0; col < K; col++) 
    {
        load_matrix_A_column(addr_reg_ptr_A, data_reg_ptr_A, matrix_A, col);
    }
    printf("--------------------------------------------------------------\n");
    printf("Loading matrix B...\n");

    for (int row = 0; row < K; row++) 
    {
        load_matrix_B_row(addr_reg_ptr_B, data_reg_ptr_B, matrix_B, row);
    }
    // Declare pointers for control and status registers
    volatile alt_u32 *control_reg_ptr = (volatile alt_u32 *)(MM_BASE + ADDR_CONTROL_OFFSET);
    volatile alt_u32 *status_reg_ptr = (volatile alt_u32 *)(MM_BASE + ADDR_STATUS_OFFSET);

    printf("\nStarting multiplication...\n");
    // Write to control register using direct pointer
    *control_reg_ptr = 0x3;

    printf("Waiting for multiplication to finish...\n");
    alt_u32 status_reg;
    do {
        // Read from status register using direct pointer
        status_reg = *status_reg_ptr;
    } while (!(status_reg & 0x1));
    printf("Multiplication finished!\n");

    printf("\nReading results from C matrix BRAM...\n");
    // Declare pointers for C matrix access
    volatile alt_u32 *c_addr_reg_ptr = (volatile alt_u32 *)(MM_BASE + ADDR_C_ADDR_OFFSET);
    // Assuming C matrix elements are read as 32-bit values (product of 16-bit)
    volatile alt_u32 *c_data_reg_ptr = (volatile alt_u32 *)(MM_BASE + ADDR_C_DATA_OFFSET);

    int row;
    int col;
    for (row = 0; row < M; row++) {
        for (col = 0; col < N; col++) {
            alt_u32 c_bram_addr = row * N + col;

            // Write to C address register using direct pointer
            *c_addr_reg_ptr = c_bram_addr;
            // Read from C data register using direct pointer
            alt_u32 result_val = *c_data_reg_ptr;

            printf("C[%d][%d] = %u (0x%X)\n", row, col, (unsigned int)result_val, (unsigned int)result_val);
        }
    }

    printf("\nNios II application finished.\n");
    return 0;
}

