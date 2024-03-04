`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/29/2024 08:17:46 PM
// Design Name: 
// Module Name: noc2aximst
// Project Name: Improving Memory Access Latency 
// Target Devices: None
// Tool Versions: 
// Description: 
// 
// Dependencies: ESP Main Repo
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

`define cacheline 8				// Global parameter: Burst length when CPU is AHB (Leon3)
`define ARCH_BITS 32				// Global parameter: AXI crossbar width
`define AXIDW	  32				// Global parameter: AXI data channel width
`define GLOB_PHYS_ADDR_BITS 32			// Global parameter: AXI address channel width
`define PREAMBLE_WIDTH 2			// Global parameter: Preamble of all flits is 2 bits wide
`define NOC_FLIT_SIZE 34			// Global parameter: Flit width = ARCH_BITS + 2
`define MSG_TYPE_WIDTH 5			// Global parameter: Message width within the header flit

// Burst Types
parameter [1:0] XBURST_FIXED = 2'b00;
parameter [1:0] XBURST_INCR  = 2'b01;
parameter [1:0] XBURST_WRAP  = 2'b10;

// Response Types
parameter [1:0] XRESP_OKAY   = 2'b00;
parameter [1:0] XRESP_EXOKAY = 2'b01;
parameter [1:0] XRESP_SLVERR = 2'b10;
parameter [1:0] XRESP_DECERR = 2'b11;


typedef struct {

 	logic [      `NOC_FLIT_SIZE-1 : 0] flit;
	logic [	    `MSG_TYPE_WIDTH-1 : 0] msg;
	logic [	      		    2 : 0] state;
	logic [			    7 : 0] count;
	
	logic	     			   ar_valid;
	logic [`GLOB_PHYS_ADDR_BITS-1 : 0] ar_addr;
	logic [			    7 : 0] ar_len;
	logic [			    2 : 0] ar_size;
	logic [			    2 : 0] ar_prot;

	logic	     			   aw_valid;
	logic [`GLOB_PHYS_ADDR_BITS-1 : 0] aw_addr;
	logic [			    7 : 0] aw_len;
	logic [			    2 : 0] aw_size;
	logic [			    2 : 0] aw_prot;

	logic		     r_ready;

	logic		     w_valid;
	logic [`AXIDW-1 : 0] w_data;
	logic [   AWI-1 : 0] w_strb;
	logic 		     w_last;

	logic		     b_ready;


} state_struct;

module noc2aximst 

    #(
    	parameter tech          = 0,
	parameter mst_index     = 0,		// Master index: each Noc2AXI master is given a different value
    	parameter axitran       = 0,		// axitran = 1: CPU is AXI (Ariane), axitran = 0: CPU is AHB (Leon3)
    	parameter eth_dma       = 0,
    	parameter narrow_noc    = 0,
    	parameter cacheline	= 8)		// Burst length when the CPU is AHB (Leon3)

    (

	input logic 	  ACLK,
	input logic 	  ARESETn,
	input logic [2:0] local_y,		// Coordinates of memory tile (source tile) used when data are sent to the NoC
	input logic [2:0] local_x,	

	/* AXI Interface Requirements */

	/* Read Address Channel */
	
	output logic				 AR_ID;
	input  logic	 			 AR_READY;
	output logic	     			 AR_VALID;
	output logic [`GLOB_PHYS_ADDR_BITS-1 : 0] AR_ADDR;
	output logic [			  7 : 0] AR_LEN;
	output logic [			  2 : 0] AR_SIZE;
	output logic    		  1 : 0] AR_BURST;
	output logic 				 AR_LOCK;
	//AWCACHE
	output logic [			  2 : 0] AR_PROT;
	//ARQOS;
	//AWREGION
	//AWUSER
	

	/* Read Data Channel	*/
	input  logic 		   R_ID;
	input  logic		   R_VALID;
	output logic		   R_READY;
	input  logic [`AXIDW-1 : 0] R_DATA;
	input  logic [      1 : 0] R_RESP;
	input  logic 		   R_LAST;
	//RUSER;

	/* Write Address Channel */
	
	output logic 				 AW_ID;
	input  logic				 AW_READY;
	output logic				 AW_VALID;
	output logic [`GLOB_PHYS_ADDR_BITS-1 : 0] AW_ADDR;	
	output logic [			  7 : 0] AW_LEN;
	output logic [			  2 : 0] AW_SIZE;
	output logic		 	  1 : 0] AW_BURST;
	output logic 				 AW_LOCK;
	//AWCACHE
	output logic [			  2 : 0] AW_PROT;
	//AWQOS;
	//AWREGION
	//AWUSER
	
	/* Write Data Channel */

	input  logic		   W_READY;
	output logic		   W_VALID;
	output logic [`AXIDW-1 : 0] W_DATA;
	output logic [AWI-1   : 0] W_STRB;
	output logic 		   W_LAST;
	//WUSER

	/*Write Response Channel */
	
	input  logic       B_ID;
	input  logic	   B_VALID;
	output logic	   B_READY;
	input  logic [1:0] B_RESP;
	// BUSER


	/*	NoC Interface	*/
	output logic 	                        coherence_req_rdreq;
	input  logic	[`NOC_FLIT_SIZE-1 : 0]	coherence_req_data_out;
	input  logic			    	        coherence_req_empty;

	output logic				            coherence_rsp_snd_wrreq;
	output logic	[`NOC_FLIT_SIZE-1 : 0]  coherence_rsp_snd_data_in;
	input  logic				            coherence_rsp_snd_full;
	

    );
endmodule

	localparam AWI = `AXIDW / 8;

	// No support for atomic operations
    assign AW_LOCK = 1'b0;
	assign AR_LOCK = 1'b0;

	// AxSIZE: Specifies the number of bytes to transfer in each data transfer (beat)
	// The processor can request 1 byte, 2 bytes, 4 bytes or 8 bytes, which means that transfers only up to 64 bits are supported
	assign AR_SIZE[2] = 1'b0;
	assign AW_SIZE[2] = 1'b0;

	// AxBURST: Specifies the burst type. Since we access sequential memory set to INCR
	assign AR_BURST = XBURST_INCR;
	assign AW_BURST = XBURST_INCR;


	// AxPROT: Specifies the access permissions to protect against illegal transactions


	// AxID: Transaction Identifiers
	// Data transfers with the same ID must be returned in order
	// For masters: transaction ID field can be up to 4 bits. If master supports only a single ordered interface: ID can be tied to a constant value
	assign AR_ID = mst_index;
	assign AW_ID = mst_index;

	// AxQOS: Additional signaling set to default value '0000'
	//assign AR_QOS = 4'b0000;
	//assign AW_QOS = 4'b0000;

	// TODO: AxLAST and AxRESP signals can be omitted

	


	logic [2:0] current_state, next_state;
	parameter receive_header = 2'b00;


	logic [`PREAMBLE_WIDTH-1:0] preamble;
	logic sample_header;

	always @(*) begin 
		
		preamble = coherence_req_data_out[`NOC_FLIT_SIZE-1:`NOC_FLIT_SIZE-`PREAMBLE_WIDTH];
		sample_header = 1'b0;
    		narrow_coherence_req_rdreq	 = 1'b0;
    		narrow_coherence_rsp_snd_data_in = 0;
    		narrow_coherence_rsp_snd_wrreq   = 1'b0;
		
		case (current_state)
		
			receive_header
        endcase





	
	end

	always @(posedge ACLK) begin
		if (~ARESETn) begin
			AR_VALID <= 0;	
			AW_VALID <= 0;
			W_VALID  <= 0;
			
		end else 
			current_state <= next_state;
			AR_VALID <= next_state.ar_valid;	

	end
