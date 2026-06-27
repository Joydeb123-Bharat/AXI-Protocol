`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 27.06.2026 20:47:51
// Design Name: 
// Module Name: axi4_lite_slave
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module axi4_lite_slave #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32
)(
    // Global Signals
    input  logic                      ACLK,
    input  logic                      ARESETn, // Active-low reset

    // Write Address Channel (AW)
    input  logic [ADDR_WIDTH-1:0]     AWADDR,
    input  logic [2:0]                AWPROT,  // Protection attributes
    input  logic                      AWVALID,
    output logic                      AWREADY,

    // Write Data Channel (W)
    input  logic [DATA_WIDTH-1:0]     WDATA,
    input  logic [(DATA_WIDTH/8)-1:0] WSTRB,   // Byte strobes
    input  logic                      WVALID,
    output logic                      WREADY,

    // Write Response Channel (B)
    output logic [1:0]                BRESP,   // 2b response (OKAY, SLVERR, etc.)
    output logic                      BVALID,
    input  logic                      BREADY,
    
    // Read Address Channel (AR)
    input  logic [ADDR_WIDTH-1:0]     ARADDR,
    input  logic [2:0]                ARPROT,
    input  logic                      ARVALID,
    output logic                      ARREADY,

    // Read Data Channel (R)
    output logic [DATA_WIDTH-1:0]     RDATA,
    output logic [1:0]                RRESP,
    output logic                      RVALID,
    input  logic                      RREADY
);

    // INTERNAL REGISTERS AND FLAGS
    
    // Memory/Register file 
    logic [(DATA_WIDTH/8)-1:0][7:0] slv_reg [0:3];

    // AXI4-Lite requires order independence for AW and W channels.
    logic aw_pend; 
    logic w_pend;

    // Latched Write Address and Write Data
    logic [ADDR_WIDTH-1:0] latched_awaddr;
    logic [DATA_WIDTH-1:0] latched_wdata;
    logic [(DATA_WIDTH/8)-1:0] latched_wstrb;
    
    // Latched Read Address and Read Data
    logic [ADDR_WIDTH-1:0] latched_araddr;
    logic                  ar_pend;
    
    // For the AW channel 
    always_ff@(posedge ACLK)
    begin
        if(!ARESETn)
        begin
            aw_pend <= '0;
            latched_awaddr <= '0;
            AWREADY <= '0;
        end
        else
        begin
            if(AWVALID && !AWREADY && !aw_pend)
            begin
                AWREADY <= 1'b1;
            end
            else
            begin
                AWREADY <= 1'b0;  
            end
            
            if(AWVALID && AWREADY)
            begin
                aw_pend <= 1'b1;
                latched_awaddr <= AWADDR;   
            end
            else if(BREADY && BVALID)
                aw_pend <= 1'b0;
        end
    end
    // For the W channel 
    logic awaddr_invalid;
    assign awaddr_invalid = (latched_awaddr[31:4] != 0 || latched_awaddr[1:0] != 0 || latched_awaddr[3:2] >= 4);
    
    always_ff@(posedge ACLK)
    begin
        if(!ARESETn)
        begin
            WREADY <= '0;
            w_pend <=  '0;
            latched_wdata <= '0;
            latched_wstrb <= '0;
        end
        else
        begin
            if(!w_pend && WVALID)
            begin
                latched_wdata <= WDATA;
                latched_wstrb <= WSTRB;
                w_pend <= 1'b1;
            end
            else if(BREADY && BVALID)
                w_pend <= 1'b0;
            WREADY <= (WVALID && !w_pend);
            
        end
    end
    
    // Execution block and B channel
    always_ff@(posedge ACLK)
    begin
        if(!ARESETn)
        begin
            BRESP <= '0;
            BVALID <= '0;
            for (int i = 0; i < 4; i++)
                slv_reg[i] <= '0;
        end
        else
        begin
            if(aw_pend && w_pend && !BVALID) 
            begin
                // Only write if the address is valid!
                if (!awaddr_invalid) 
                begin
                    for (int b = 0; b < DATA_WIDTH/8; b++) 
                    begin
                        if (latched_wstrb[b])
                            slv_reg[latched_awaddr[3:2]][b] <= latched_wdata[(8*b)+: 8];
                    end
                end
                 
                BRESP <= awaddr_invalid ? 2'b10 : 2'b00;
                BVALID <= 1'b1;
            end 
            else if(BREADY && BVALID)
            begin
                BVALID <= 1'b0;
            end
        end
    end
    
    //For the Read Address (AR) Channel
    always_ff@(posedge ACLK)
    begin
        if(!ARESETn)
        begin
            latched_araddr <= '0;
            ar_pend <= '0;
            ARREADY <= '0;
        end
        else
        begin
            if(ARREADY && ARVALID) 
            begin                                   
                ARREADY       <= 1'b0;
                ar_pend       <= 1'b1;
                latched_araddr <= ARADDR;
            end 
            else if(RVALID && RREADY)
                ar_pend <= 1'b0;
            else if(!ar_pend && ARVALID)
                ARREADY <= 1'b1;
            else
                ARREADY <= 1'b0;        
        end
    end
    
    // For the Read Channel
    logic araddr_invalid;
    assign araddr_invalid = (latched_araddr[31:4] != 0 || latched_araddr[1:0] != 0 || latched_araddr[3:2] >= 4);
    
    always_ff@(posedge ACLK)
    begin
        if(!ARESETn)
        begin
            RDATA <= '0;
            RRESP <= '0;
            RVALID <= '0;
        end
        else
        begin
            if(ar_pend && !RVALID)
            begin
                RVALID <= 1'b1;
                RDATA  <= araddr_invalid ? '0 : slv_reg[latched_araddr[3:2]];   
                RRESP <= araddr_invalid ? 2'b10 : 2'b00;
            end
            else if(RVALID && RREADY)
            begin
                RVALID <= 1'b0;
            end
        end
    end
endmodule