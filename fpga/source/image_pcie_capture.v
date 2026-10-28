`timescale 1ns / 1ps

`define UD #1
`define CMOS_1      
`define CMOS_2      
module image_pcie_capture #(

   parameter MEM_ROW_WIDTH        = 15         ,

   parameter MEM_COLUMN_WIDTH     = 10         ,

   parameter MEM_BANK_WIDTH       = 3          ,

  parameter MEM_DQ_WIDTH          = 16         ,

  parameter MEM_DQS_WIDTH         = 2

)(
    input                                free_clk                  ,
    input                                board_rst_n               ,

    //DDR3 interface
    output                               mem_cs_n                  ,  
    output                               mem_rst_n                 ,
    output                               mem_ck                    ,
    output                               mem_ck_n                  ,
    output                               mem_cke                   ,
    output                               mem_ras_n                 ,
    output                               mem_cas_n                 ,
    output                               mem_we_n                  ,
    output                               mem_odt                   ,
    output      [MEM_ROW_WIDTH-1:0]      mem_a                     ,
    output      [MEM_BANK_WIDTH-1:0]     mem_ba                    ,
    inout       [MEM_DQ_WIDTH/8-1:0]     mem_dqs                   ,
    inout       [MEM_DQ_WIDTH/8-1:0]     mem_dqs_n                 ,
    inout       [MEM_DQ_WIDTH-1:0]       mem_dq                    ,
    output      [MEM_DQ_WIDTH/8-1:0]     mem_dm                    ,

    // PCIe interface
    input					             ref_clk_p                 ,		    
	input					             ref_clk_n                 ,		    
	input					             perst_n                   ,		    
	input		[1:0]		             rxn                       ,			
	input		[1:0]		             rxp                       ,			
	output wire	[1:0]		             txn                       ,			
	output wire	[1:0]		             txp                       ,			

//OV5640
    output  [2:0]                        cmos_init_done             ,
    //coms1	        
    inout                                cmos1_scl                  ,
    inout                                cmos1_sda                  ,
    input                                cmos1_vsync                ,
    input                                cmos1_href                 ,
    input                                cmos1_pclk                 ,
    input   [7:0]                        cmos1_data                 ,
    output                               cmos1_reset                ,
    //coms2     
    inout                                cmos2_scl                  ,
    inout                                cmos2_sda                  ,
    input                                cmos2_vsync                ,
    input                                cmos2_href                 ,
    input                                cmos2_pclk                 ,
    input   [7:0]                        cmos2_data                 ,
    output                               cmos2_reset                
);

parameter CTRL_ADDR_WIDTH = MEM_ROW_WIDTH + MEM_BANK_WIDTH + MEM_COLUMN_WIDTH;
parameter TH_1S = 27'd33000000;
parameter REM_DQS_WIDTH = 9 - MEM_DQS_WIDTH;


reg                         heart_beat_led             ;
reg                         pclk_led                   ;
reg                         ref_led                    ;

wire                        ddrphy_cpd_lock            ;
wire                        ddr_init_done              ;
wire                        pll_lock                   ;
wire                        core_clk                   ;
wire [CTRL_ADDR_WIDTH-1:0]  axi_awaddr                 ;
wire                        axi_awuser_ap              ;
wire [3:0]                  axi_awuser_id              ;
wire [3:0]                  axi_awlen                  ;
wire                        axi_awready                ;
wire                        axi_awvalid                ;
wire [MEM_DQ_WIDTH*8-1:0]   axi_wdata                  ;
wire [MEM_DQ_WIDTH*8/8-1:0] axi_wstrb                  ;
wire                        axi_wready                 ;
wire [3:0]                  axi_wusero_id              ;
wire                        axi_wusero_last            ;
wire [CTRL_ADDR_WIDTH-1:0]  axi_araddr                 ;
wire                        axi_aruser_ap              ;
wire [3:0]                  axi_aruser_id              ;
wire [3:0]                  axi_arlen                  ;
wire                        axi_arready                ;
wire                        axi_arvalid                ;
wire [MEM_DQ_WIDTH*8-1:0]   axi_rdata  /* synthesis syn_keep = 1 */;
wire                        axi_rvalid /* synthesis syn_keep = 1 */;
wire [3:0]                  axi_rid                    ;
wire                        axi_rlast                  ;
wire                        resetn                     ;
reg  [26:0]                 cnt                        ;
wire [7:0]                  err_cnt                    ;
wire                        free_clk_g                 ;

//cmos
reg  [15:0]                 rstn_1ms                   ;
wire                        cmos_scl                   ;
wire                        cmos_sda                   ;
wire                        cmos_vsync                 ;
wire                        cmos_href                  ;
wire                        cmos_pclk                  ;
wire   [7:0]                cmos_data                  ;
wire                        cmos_reset                 ;
wire                        initial_en                 ;
wire                        initial_en_1               ;
wire[15:0]                  cmos1_d_16bit              /*synthesis PAP_MARK_DEBUG="1"*/;
wire                        cmos1_href_16bit           /*synthesis PAP_MARK_DEBUG="1"*/;
reg [7:0]                   cmos1_d_d0                 /*synthesis PAP_MARK_DEBUG="1"*/;
reg                         cmos1_href_d0              /*synthesis PAP_MARK_DEBUG="1"*/;
reg                         cmos1_vsync_d0             /*synthesis PAP_MARK_DEBUG="1"*/;
wire                        cmos1_pclk_16bit           /*synthesis PAP_MARK_DEBUG="1"*/;
wire[15:0]                  cmos2_d_16bit              /*synthesis PAP_MARK_DEBUG="1"*/;
wire                        cmos2_href_16bit           /*synthesis PAP_MARK_DEBUG="1"*/;
reg [7:0]                   cmos2_d_d0                 /*synthesis PAP_MARK_DEBUG="1"*/;
reg                         cmos2_href_d0              /*synthesis PAP_MARK_DEBUG="1"*/;
reg                         cmos2_vsync_d0             /*synthesis PAP_MARK_DEBUG="1"*/;
wire                        cmos2_pclk_16bit           /*synthesis PAP_MARK_DEBUG="1"*/;

wire[15:0]                  o_rgb565                   ;
wire                        pclk_in_test               ;    
wire                        lane_pixel_clk             ;
wire                        vs_in_test                 ;
wire                        de_in_test                 ;
wire[15:0]                  i_rgb565                   ;
wire                        pclk_in_test_2             ;    
wire                        vs_in_test_2               ;
wire                        de_in_test_2               ;
wire[15:0]                  i_rgb565_2                 ;

// lane result path
wire                        lane_enable_125            ;
wire                        lane_irq_en_125            ;
wire                        lane_soft_reset_125        ;
wire [10:0]                 lane_threshold_125         ;
wire [10:0]                 lane_expected_left_125     ;
wire [10:0]                 lane_expected_right_125    ;
wire [10:0]                 lane_window_125            ;
wire [31:0]                 lane_result_seq_125        ;

// decision output registers
wire                        decision_led_l_125         ;
wire                        decision_led_r_125         ;
wire                        decision_led_m_125         ;
wire                        decision_alarm_125         ;
wire [15:0]                 decision_frame_echo_125    ;
wire [4:0]                  decision_key_events_125    ;

wire                        lane_enable_pixel          ;
wire [10:0]                 lane_threshold_pixel       ;
wire [10:0]                 lane_expected_left_pixel   ;
wire [10:0]                 lane_expected_right_pixel  ;
wire [10:0]                 lane_window_pixel          ;
wire                        lane_pixel_ready           ;
wire                        lane_pixel_rst_n           ;

wire                        lane_commit_125            ;
wire [31:0]                 lane_frame_id_125          ;
wire [1:0]                  lane_left_state_125        ;
wire [1:0]                  lane_right_state_125       ;
wire signed [31:0]          lane_left_a_q8_125         ;
wire signed [31:0]          lane_right_a_q8_125        ;
wire signed [31:0]          lane_left_b_q8_125         ;
wire signed [31:0]          lane_right_b_q8_125        ;
wire                        lane_left_fit_valid_125    ;
wire                        lane_right_fit_valid_125   ;
wire [5:0]                  lane_left_point_count_125  ;
wire [5:0]                  lane_right_point_count_125 ;
wire                        lane_left_fit_raw_125      ;
wire                        lane_right_fit_raw_125     ;
wire [11:0]                 lane_left_max_error_125    ;
wire [11:0]                 lane_right_max_error_125   ;
wire                        lane_busy_125              ;
wire [31:0]                 lane_dbg_125               ;

//pll
wire                        lock                       ;
wire                        clk_10m                    ;
wire                        clk_25m                    ;
wire                        clk_50m                    ;

localparam DEVICE_TYPE = 3'b000;			
localparam AXIS_SLAVE_NUM = 3;				

// Test unit mode signals
wire			pcie_cfg_ctrl_en;			
wire			axis_master_tready_cfg;		

wire			cfg_axis_slave0_tvalid;		
wire	[127:0]	cfg_axis_slave0_tdata;		
wire			cfg_axis_slave0_tlast;		
wire			cfg_axis_slave0_tuser;		

// For mux
wire			axis_master_tready_mem;		
wire			axis_master_tvalid_mem;		
wire	[127:0]	axis_master_tdata_mem;		
wire	[3:0]	axis_master_tkeep_mem;		
											
wire			axis_master_tlast_mem;		
wire	[7:0]	axis_master_tuser_mem;		

wire			cross_4kb_boundary;			

wire			dma_axis_slave0_tvalid;		
wire	[127:0]	dma_axis_slave0_tdata;		
wire			dma_axis_slave0_tlast;		
wire			dma_axis_slave0_tuser;		

// Reset debounce and sync
wire			sync_button_rst_n; 			
wire			ref_core_rst_n;	
wire            sync_perst_n;			
wire			s_pclk_rstn;				

// Internal signal
wire			pclk_div2/*synthesis PAP_MARK_DEBUG="1"*/;  	
wire			pclk/*synthesis PAP_MARK_DEBUG="1"*/;			
wire			ref_clk; 					
wire			core_rst_n;					

wire			axis_master_tvalid;
wire			axis_master_tready;
wire	[127:0]	axis_master_tdata;
wire	[3:0]	axis_master_tkeep;
wire			axis_master_tlast;
wire	[7:0]	axis_master_tuser;

// AXI4-Stream slave 0 interface
wire			axis_slave0_tready;
wire			axis_slave0_tvalid;
wire	[127:0]	axis_slave0_tdata;
wire			axis_slave0_tlast;
wire			axis_slave0_tuser;
// AXI4-Stream slave 1 interface
wire			axis_slave1_tready;
wire			axis_slave1_tvalid;
wire	[127:0]	axis_slave1_tdata;
wire			axis_slave1_tlast;
wire			axis_slave1_tuser;
// AXI4-Stream slave 2 interface
wire			axis_slave2_tready;
wire			axis_slave2_tvalid;
wire	[127:0]	axis_slave2_tdata;
wire			axis_slave2_tlast;
wire			axis_slave2_tuser;

wire	[7:0]	cfg_pbus_num;			
wire	[4:0]	cfg_pbus_dev_num; 		
wire	[2:0]	cfg_max_rd_req_size;	
wire	[2:0]	cfg_max_payload_size;	
wire			cfg_rcb;				

wire			cfg_ido_req_en;			
wire			cfg_ido_cpl_en;			
wire	[7:0]	xadm_ph_cdts;			
wire	[11:0]	xadm_pd_cdts;			
wire	[7:0]	xadm_nph_cdts;			
wire	[11:0]	xadm_npd_cdts;			
wire	[7:0]	xadm_cplh_cdts;			
wire	[11:0]	xadm_cpld_cdts;			

wire	[4:0]	smlh_ltssm_state/*synthesis PAP_MARK_DEBUG="1"*/;

// Led lights up signal
reg		[22:0]	ref_led_cnt;		
reg		[26:0]	pclk_led_cnt;		
wire			smlh_link_up; 	
wire			rdlh_link_up/*synthesis PAP_MARK_DEBUG="1"*/; 	

// Uart to APB 32bits
wire			uart_p_sel;			
wire	[3:0]	uart_p_strb;		
wire	[15:0]	uart_p_addr;		
wire	[31:0]	uart_p_wdata;		
wire			uart_p_ce;			
wire			uart_p_we;			
wire			uart_p_rdy;			
wire	[31:0]	uart_p_rdata;		

// APB signal
wire	[3:0]	p_strb; 			
wire	[15:0]	p_addr; 			
wire	[31:0]	p_wdata; 			
wire			p_ce; 				
wire			p_we; 				


wire			p_sel_pcie;			
wire			p_sel_cfg;			
wire			p_sel_dma;			

wire	[31:0]	p_rdata_pcie;		
wire	[31:0]	p_rdata_cfg;		
wire	[31:0]	p_rdata_dma;		

wire			p_rdy_pcie;			
wire			p_rdy_cfg;			
wire			p_rdy_dma;			

assign cfg_ido_req_en	=	1'b0;	
assign cfg_ido_cpl_en	=	1'b0;	
assign xadm_ph_cdts		=	8'b0;	
assign xadm_pd_cdts		=	12'b0;	
assign xadm_nph_cdts	=	8'b0;	
assign xadm_npd_cdts	=	12'b0;	
assign xadm_cplh_cdts	=	8'b0;	
assign xadm_cpld_cdts	=	12'b0;	





//axi
wire          ch0_wframe_data_valid      ;
wire [15:0]   ch0_wframe_data            ;
reg           ch0_rframe_req;
wire          ch0_rframe_req_ack;
wire          ch0_rframe_data_en;
wire  [127:0] ch0_rframe_data;
wire          ch0_rframe_data_valid;

wire          ch1_wframe_data_valid      ;
wire [15:0]   ch1_wframe_data            ;
reg           ch1_rframe_req;
wire          ch1_rframe_req_ack;
wire          ch1_rframe_data_en;
wire  [127:0] ch1_rframe_data;
wire          ch1_rframe_data_valid;

//dma
wire            o_dma_write_data_req;
wire [11:0]     o_dma_write_addr ;
wire [127:0]    i_dma_write_data;

wire            cmos2_pclk_bufg;

reg             cmos1_vsync_d1;
reg             cmos1_vsync_d2;
reg             cmos1_href_d1;
reg             cmos1_href_d2;
reg   [7:0]     cmos1_data_d1;
reg   [7:0]     cmos1_data_d2;

reg             cmos2_vsync_d1;
reg             cmos2_vsync_d2;
reg             cmos2_href_d1;
reg             cmos2_href_d2;
reg   [7:0]     cmos2_data_d1;
reg   [7:0]     cmos2_data_d2;

wire            line_full_flag_1;
wire            line_full_flag_2;

wire                        ch0_wframe_data_valid_raw;
wire [15:0]                 ch0_wframe_data_raw;

always @(posedge cmos1_pclk)begin
    cmos1_vsync_d1 <= cmos1_vsync;
    cmos1_vsync_d2 <= cmos1_vsync_d1;
    cmos1_href_d1 <= cmos1_href;
    cmos1_href_d2  <= cmos1_href_d1;
    cmos1_data_d1 <= cmos1_data;
    cmos1_data_d2 <= cmos1_data_d1;
end

always @(posedge cmos2_pclk)begin
    cmos2_vsync_d1 <= cmos2_vsync;
    cmos2_vsync_d2 <= cmos2_vsync_d1;
    cmos2_href_d1 <= cmos2_href;
    cmos2_href_d2  <= cmos2_href_d1;
    cmos2_data_d1 <= cmos2_data;
    cmos2_data_d2 <= cmos2_data_d1;
end

//*==============================================================================
//PLL
//*==============================================================================
pll pll_inst (
  .clkout0(clk_10m),    
  .clkout1(clk_25m),    
  .clkout2(clk_50m),    
  .lock(lock),          
  .clkin1(free_clk)       
);

//*==============================================================================
//配置cmos
//*==============================================================================
power_on_delay	power_on_delay_inst(
    .clk_50M                 (clk_50m        ),
    .reset_n                 (ddr_init_done  ),	
    .camera1_rstn            (cmos1_reset    ),
    .camera2_rstn            (cmos2_reset    ),	
    .camera_pwnd             (               ),
    .initial_en              (initial_en     ) 		
);

//CMOS1 Camera 
reg_config	coms1_reg_config(
    .clk_25M                 (clk_50m            ),
    .camera_rstn             (cmos1_reset        ),
    .initial_en              (initial_en         ),		
    .i2c_sclk                (cmos1_scl          ),
    .i2c_sdat                (cmos1_sda          ),
    .reg_conf_done           (cmos_init_done[0]  ),
    .reg_index               (                   ),
    .clock_20k               (                   ) 
);

//CMOS2 Camera 
reg_config	coms2_reg_config(
    .clk_25M                 (clk_50m            ),
    .camera_rstn             (cmos2_reset        ),
    .initial_en              (initial_en         ),		
    .i2c_sclk                (cmos2_scl          ),
    .i2c_sdat                (cmos2_sda          ),
    .reg_conf_done           (cmos_init_done[1]  ),
    .reg_index               (                   ),
    .clock_20k               (                   ) 
);

//===============================================================================
//CMOS 8bit转16bit
//===============================================================================
//CMOS1
always@(posedge cmos1_pclk)
    begin
        cmos1_d_d0        <= cmos1_data_d2    ;
        cmos1_href_d0     <= cmos1_href_d2    ;
        cmos1_vsync_d0    <= cmos1_vsync_d2   ;
    end

cmos_8_16bit cmos1_8_16bit(
.pclk           (cmos1_pclk       ),
.rst_n          (cmos_init_done[0]),
.pdata_i        (cmos1_d_d0       ),
.de_i           (cmos1_href_d0    ),
.vs_i           (cmos1_vsync_d0    ),

.pixel_clk      (cmos1_pclk_16bit ),
.pdata_o        (cmos1_d_16bit    ),
.de_o           (cmos1_href_16bit ) 
);

//CMOS2
always@(posedge cmos2_pclk)
    begin
        cmos2_d_d0        <= cmos2_data_d2    ;
        cmos2_href_d0     <= cmos2_href_d2    ;
        cmos2_vsync_d0    <= cmos2_vsync_d2   ;
    end

cmos_8_16bit cmos2_8_16bit(
.pclk           (cmos2_pclk       ),
.rst_n          (cmos_init_done[1]),
.pdata_i        (cmos2_d_d0       ),
.de_i           (cmos2_href_d0    ),
.vs_i           (cmos2_vsync_d0    ),

.pixel_clk      (cmos2_pclk_16bit ),
.pdata_o        (cmos2_d_16bit    ),
.de_o           (cmos2_href_16bit ) 
);

//===============================================================================
// 输出视频选择
//===============================================================================
assign     pclk_in_test    =    cmos1_pclk_16bit    ;
// Isolate the lane path from the high-fanout image clock. Drive the
// global clock-buffer enable explicitly so the lane clock is not gated.
// 2026-10-10: lane tapped cmos1 here until it was proven (occlusion test:
// covering the stereo module never perturbed a single published bit) that
// the cmos1 path delivers constant data to the lane while the same pixels
// are live in the app path; the tap now follows cmos2.
GTP_CLKBUFG u_lane_pixel_clkbufg (
    .CLKOUT                (lane_pixel_clk           ),
    .CLKIN                 (pclk_in_test_2            )
);
assign     vs_in_test      =    cmos1_vsync_d0      ;
assign     de_in_test      =    cmos1_href_16bit    ;

// (The temporary 2026-10-10 edge/run activity export was retired after the
// lane pipeline was verified: the right-b register carries the real
// coefficient again; those counters stay internal to lane_core.)
assign     i_rgb565        =    {cmos1_d_16bit[4:0],cmos1_d_16bit[10:5],cmos1_d_16bit[15:11]};

assign lane_pixel_ready = lock && cmos_init_done[0];
// Production mode: lane pixel path always enabled and reset only by the
// global core reset (the camera-ready handshake gating used before is not
// required in the integrated build).
assign lane_enable_pixel = 1'b1;
assign lane_pixel_rst_n = core_rst_n;

lane_config_sync u_lane_config_sync (
    .src_clk                (pclk_div2                 ),
    .src_rst_n              (core_rst_n                ),
    .enable_i               (lane_enable_125           ),
    .threshold_i            (lane_threshold_125        ),
    .expected_left_i        (lane_expected_left_125    ),
    .expected_right_i       (lane_expected_right_125   ),
    .window_i               (lane_window_125           ),
    .dst_clk                (lane_pixel_clk            ),
    .dst_rst_n              (lane_pixel_ready         ),
    .enable_o               (                          ),
    .threshold_o            (lane_threshold_pixel      ),
    .expected_left_o        (lane_expected_left_pixel  ),
    .expected_right_o       (lane_expected_right_pixel ),
    .window_o               (lane_window_pixel         )
);

// Side path tap: i_rgb565 is the R/B-swapped RGB565 stream before image_reshape.
lane_capture_path u_lane_capture_path (
    .pixel_clk              (lane_pixel_clk            ),
    .pixel_rst_n            (lane_pixel_rst_n          ),
    .vsync_i                (vs_in_test_2              ),
    .de_i                   (de_in_test_2              ),
    .rgb565_i               (i_rgb565_2                ),
    .threshold_i            (lane_threshold_pixel      ),
    .expected_left_i        (lane_expected_left_pixel  ),
    .expected_right_i       (lane_expected_right_pixel ),
    .window_i               (lane_window_pixel         ),
    .dst_clk                (pclk_div2                 ),
    .dst_rst_n              (core_rst_n                ),
    .dst_commit_o           (lane_commit_125           ),
    .dst_frame_id_o         (lane_frame_id_125         ),
    .dst_left_state_o       (lane_left_state_125       ),
    .dst_right_state_o      (lane_right_state_125      ),
    .dst_left_a_q8_o        (lane_left_a_q8_125        ),
    .dst_right_a_q8_o       (lane_right_a_q8_125       ),
    .dst_left_b_q8_o        (lane_left_b_q8_125        ),
    .dst_right_b_q8_o       (lane_right_b_q8_125       ),
    .dst_left_fit_valid_o   (lane_left_fit_valid_125   ),
    .dst_right_fit_valid_o  (lane_right_fit_valid_125  ),
    .debug_left_point_count_o(lane_left_point_count_125),
    .debug_right_point_count_o(lane_right_point_count_125),
    .debug_left_fit_valid_raw_o(lane_left_fit_raw_125),
    .debug_right_fit_valid_raw_o(lane_right_fit_raw_125),
    .debug_left_max_error_o (lane_left_max_error_125   ),
    .debug_right_max_error_o(lane_right_max_error_125  ),
    .debug_stream_word_o     (lane_dbg_125            ),
    .dst_busy_o             (lane_busy_125             )
);

assign     pclk_in_test_2  =    cmos2_pclk_16bit    ;
assign     vs_in_test_2    =    cmos2_vsync_d0      ;
assign     de_in_test_2    =    cmos2_href_16bit    ;
assign     i_rgb565_2      =    {cmos2_d_16bit[4:0],cmos2_d_16bit[10:5],cmos2_d_16bit[15:11]};

//*==============================================================================
//ddr3 IP例化
//*==============================================================================
always@(posedge core_clk or negedge ddr_init_done)
begin
   if (!ddr_init_done)
      cnt <= 27'd0;
   else if ( cnt >= TH_1S )
      cnt <= 27'd0;
   else
      cnt <= cnt + 27'd1;
end

always @(posedge core_clk or negedge ddr_init_done)
begin
   if (!ddr_init_done)
      heart_beat_led <= 1'd1;
   else if ( cnt >= TH_1S )
      heart_beat_led <= ~heart_beat_led;
end
ddr3 #(
    .MEM_ROW_WIDTH              (MEM_ROW_WIDTH                ),
    .MEM_COLUMN_WIDTH           (MEM_COLUMN_WIDTH             ),
    .MEM_BANK_WIDTH             (MEM_BANK_WIDTH               ),
    .MEM_DQ_WIDTH               (MEM_DQ_WIDTH                 ),
    .MEM_DM_WIDTH               (MEM_DQS_WIDTH                ),
    .MEM_DQS_WIDTH              (MEM_DQS_WIDTH                ),
    .CTRL_ADDR_WIDTH            (CTRL_ADDR_WIDTH              )
  )I_ips_ddr_top(
    .ref_clk                    (free_clk                     ),
    .resetn                     (board_rst_n                  ),
    .core_clk                   (core_clk                     ),
    .pll_lock                   (pll_lock                     ),
    .phy_pll_lock               (phy_pll_lock                 ),
    .gpll_lock                  (gpll_lock                    ),
    .rst_gpll_lock              (rst_gpll_lock                ),
    .ddrphy_cpd_lock            (ddrphy_cpd_lock              ),
    .ddr_init_done              (ddr_init_done                ),

    .axi_awaddr                 (axi_awaddr                   ),
    .axi_awuser_ap              (axi_awuser_ap                ),
    .axi_awuser_id              (axi_awuser_id                ),
    .axi_awlen                  (axi_awlen                    ),
    .axi_awready                (axi_awready                  ),
    .axi_awvalid                (axi_awvalid                  ),

    .axi_wdata                  (axi_wdata                    ),
    .axi_wstrb                  (axi_wstrb                    ),
    .axi_wready                 (axi_wready                   ),
    .axi_wusero_id              (axi_wusero_id                ),
    .axi_wusero_last            (axi_wusero_last              ),

    .axi_araddr                 (axi_araddr                   ),
    .axi_aruser_ap              (axi_aruser_ap                ),
    .axi_aruser_id              (axi_aruser_id                ),
    .axi_arlen                  (axi_arlen                    ),
    .axi_arready                (axi_arready                  ),
    .axi_arvalid                (axi_arvalid                  ),

    .axi_rdata                  (axi_rdata                    ),
    .axi_rid                    (axi_rid                      ),
    .axi_rlast                  (axi_rlast                    ),
    .axi_rvalid                 (axi_rvalid                   ),

    .apb_clk                    (1'b0                         ),
    .apb_rst_n                  (1'b0                         ),
    .apb_sel                    (1'b0                         ),
    .apb_enable                 (1'b0                         ),
    .apb_addr                   (8'd0                         ),
    .apb_write                  (1'b0                         ),
    .apb_ready                  (                             ),
    .apb_wdata                  (16'd0                        ),
    .apb_rdata                  (                             ),


    .mem_cs_n                   (mem_cs_n                     ),

    .mem_rst_n                  (mem_rst_n                    ),
    .mem_ck                     (mem_ck                       ),
    .mem_ck_n                   (mem_ck_n                     ),
    .mem_cke                    (mem_cke                      ),
    .mem_ras_n                  (mem_ras_n                    ),
    .mem_cas_n                  (mem_cas_n                    ),
    .mem_we_n                   (mem_we_n                     ),
    .mem_odt                    (mem_odt                      ),
    .mem_a                      (mem_a                        ),
    .mem_ba                     (mem_ba                       ),
    .mem_dqs                    (mem_dqs                      ),
    .mem_dqs_n                  (mem_dqs_n                    ),
    .mem_dq                     (mem_dq                       ),
    .mem_dm                     (mem_dm                       ),

    .dbg_gate_start             (1'b0                         ),
    .dbg_cpd_start              (1'b0                         ),
    .dbg_ddrphy_rst_n           (1'b1                         ),
    .dbg_gpll_scan_rst          (1'b0                         ),

    .samp_position_dyn_adj      (1'b0                         ),
    .init_samp_position_even    (16'd0                        ),
    .init_samp_position_odd     (16'd0                        ),

    .wrcal_position_dyn_adj     (1'b0                         ),
    .init_wrcal_position        (16'd0                        ),

    .force_read_clk_ctrl        (1'b0                         ),
    .init_slip_step             (8'd0                         ),
    .init_read_clk_ctrl         (6'd0                         ),

    .debug_calib_ctrl           (                             ),
    .dbg_dll_upd_state          (                             ),
    .dbg_slice_status           (                             ),
    .dbg_slice_state            (                             ),
    .debug_data                 (                             ),
    .debug_gpll_dps_phase       (                             ),

    .dbg_rst_dps_state          (                             ),
    .dbg_tran_err_rst_cnt       (                             ),
    .dbg_ddrphy_init_fail       (                             ),

    .debug_cpd_offset_adj       (1'b0                         ),
    .debug_cpd_offset_dir       (1'b0                         ),
    .debug_cpd_offset           (10'd0                        ),
    .debug_dps_cnt_dir0         (                             ),
    .debug_dps_cnt_dir1         (                             ),

    .ck_dly_en                  (1'b0                         ),
    .init_ck_dly_step           (8'd0                         ),
    .ck_dly_set_bin             (                             ),

    .align_error                (                             ),
    .debug_rst_state            (                             ),
    .debug_cpd_state            (                             )

  );


//*==============================================================================
// image_reshape
//*==============================================================================
image_reshape ch0_image_reshape(
    .clk                    (pclk_in_test               ),  
    .rst_n                  (lock   && ddr_init_done    ),

    .img_vs                 (vs_in_test                 ),
    .img_data_valid         (de_in_test                 ),     
    .img_data               (i_rgb565                   ),  
   
    .img_data_valid_out     (ch0_wframe_data_valid_raw  ),
    .img_data_out           (ch0_wframe_data_raw        )
);

p3_osd_overlay #(
    .INIT_FILE              ("../source/osd/p3_osd_font.mem")
) u_p3_osd_overlay (
    .clk_pixel              (pclk_in_test                 ),
    .rst_n                  (lock && ddr_init_done       ),
    .vs_i                   (vs_in_test                  ),
    .valid_i                (ch0_wframe_data_valid_raw   ),
    .rgb565_i               (ch0_wframe_data_raw         ),
    .led_l_i                (decision_led_l_125          ),
    .led_m_i                (decision_led_m_125          ),
    .led_r_i                (decision_led_r_125          ),
    .alarm_i                (decision_alarm_125          ),
    .valid_o                (ch0_wframe_data_valid       ),
    .rgb565_o               (ch0_wframe_data             )
);

image_reshape ch1_image_reshape(
    .clk                    (pclk_in_test_2             ),  
    .rst_n                  (lock   && ddr_init_done    ),

    .img_vs                 (vs_in_test_2               ),
    .img_data_valid         (de_in_test_2               ),     
    .img_data               (i_rgb565_2                 ),  
   
    .img_data_valid_out     (ch1_wframe_data_valid      ),
    .img_data_out           (ch1_wframe_data            )
);

pcie_img_select pcie_img_select_inst(
    .clk                         (pclk_div2                                 ),
    .rst_n                       (core_rst_n                                ),
    
    .dma_sim_vs                  (ch0_rframe_req                            ),
    .line_full_flag              (line_full_flag_1 && line_full_flag_2      ),                   

    .ch0_data_req                (ch0_rframe_data_en                        ),
    .ch0_data                    (ch0_rframe_data                           ),
    .ch1_data_req                (ch1_rframe_data_en                        ),
    .ch1_data                    (ch1_rframe_data                           ),


    .dma_wr_data_req             (o_dma_write_data_req                      ),
    .dma_wr_data                 (i_dma_write_data                          ) 
);
//*==============================================================================
//axi控制器例化
//*==============================================================================
axi_ctrl_256_burst axi_ctrl_inst
(
	  .ARESETN                     (ddr_init_done                             ),
	  .ACLK                        (core_clk                                  ),
	  .M_AXI_AWID                  (axi_awuser_id                             ),
	  .M_AXI_AWADDR                (axi_awaddr                                ),
	  .M_AXI_AWLEN                 (axi_awlen                                 ),
	  .M_AXI_AWSIZE                (                                          ),
	  .M_AXI_AWBURST               (                                          ),
	  .M_AXI_AWLOCK                (                                          ),
	  .M_AXI_AWCACHE               (                                          ),
	  .M_AXI_AWPROT                (                                          ),
	  .M_AXI_AWQOS                 (                                          ),
	  .M_AXI_AWUSER                (                                          ),
	  .M_AXI_AWVALID               (axi_awvalid                               ),
	  .M_AXI_AWREADY               (axi_awready                               ),
	  .M_AXI_WDATA                 (axi_wdata                                 ),
	  .M_AXI_WSTRB                 (axi_wstrb                                 ),
	  .M_AXI_WLAST                 (                                          ),
	  .M_AXI_WUSER                 (                                          ),
	  .M_AXI_WVALID                (                                          ),
	  .M_AXI_WREADY                (axi_wready                                ),
	  .M_AXI_BID                   (0                                         ),
	  .M_AXI_BRESP                 (0                                         ),
	  .M_AXI_BUSER                 (0                                         ),
      .M_AXI_BVALID                (1'b1                                      ),

	  .M_AXI_BREADY                (                                          ),
	  .M_AXI_ARID                  (axi_aruser_id                             ),
	  .M_AXI_ARADDR                (axi_araddr                                ),
	  .M_AXI_ARLEN                 (axi_arlen                                 ),
	  .M_AXI_ARSIZE                (                                          ),
	  .M_AXI_ARBURST               (                                          ),
	  .M_AXI_ARLOCK                (                                          ),
	  .M_AXI_ARCACHE               (                                          ),
	  .M_AXI_ARPROT                (                                          ),
	  .M_AXI_ARQOS                 (                                          ),
	  .M_AXI_ARUSER                (                                          ),
	  .M_AXI_ARVALID               (axi_arvalid                               ),
	  .M_AXI_ARREADY               (axi_arready                               ),
	  .M_AXI_RID                   (axi_rid                                   ),
	  .M_AXI_RDATA                 (axi_rdata                                 ),
	  .M_AXI_RRESP                 (0                                         ),
	  .M_AXI_RLAST                 (axi_rlast                                 ),
	  .M_AXI_RUSER                 (0                                         ),
	  .M_AXI_RVALID                (axi_rvalid                                ),
	  .M_AXI_RREADY                (                                          ),  

      // key
      .key                         ({1'b1,3'b111,4'b0000}                     ),

      // 通道0
      .ch0_wframe_pclk             (pclk_in_test                              ),
      .ch0_wframe_rst_n            (lock && ddr_init_done                     ),
      .ch0_wframe_vsync            (vs_in_test                                ),
      .ch0_wframe_data_valid       (ch0_wframe_data_valid                     ),         
      .ch0_wframe_data             (ch0_wframe_data                           ),

      .ch0_rframe_pclk             (pclk_div2                                 ),   
      .ch0_rframe_rst_n            (ddr_init_done                             ), 
      .ch0_rframe_vsync            (ch0_rframe_req                            ),
      .ch0_rframe_req              (ch0_rframe_req                            ),
      .ch0_rframe_req_ack          (ch0_rframe_req_ack                        ),
      .ch0_rframe_data_en          (ch0_rframe_data_en                        ),
      .ch0_rframe_data             (ch0_rframe_data                           ),      
      .ch0_rframe_data_valid       (                                          ),
      .ch0_read_line_full          (line_full_flag_1                          ),

      // 通道1
      .ch1_wframe_pclk             (pclk_in_test_2                            ),
      .ch1_wframe_rst_n            (lock && ddr_init_done                     ),
      .ch1_wframe_vsync            (vs_in_test_2                              ),
      .ch1_wframe_data_valid       (ch1_wframe_data_valid                     ),          
      .ch1_wframe_data             (ch1_wframe_data                           ),      

      .ch1_rframe_pclk             (pclk_div2                                 ),   
      .ch1_rframe_rst_n            (ddr_init_done                             ), 
      .ch1_rframe_vsync            (ch0_rframe_req                            ),
      .ch1_rframe_req              (ch0_rframe_req                            ),
      .ch1_rframe_req_ack          (                                          ),
      .ch1_rframe_data_en          (ch1_rframe_data_en                        ),
      .ch1_rframe_data             (ch1_rframe_data                           ),      
      .ch1_rframe_data_valid       (                                          ),
      .ch1_read_line_full          (line_full_flag_2                          )
);


//*==============================================================================
// pcie
//*==============================================================================
hsst_rst_cross_sync_v1_0 #(
    `ifdef IPS2L_PCIE_SPEEDUP_SIM
    .RST_CNTR_VALUE     (16'h10             )
    `else
    .RST_CNTR_VALUE     (16'hC000           )
    `endif
)
u_refclk_buttonrstn_debounce(
    .clk                (ref_clk            ),
    .rstn_in            (board_rst_n        ),
    .rstn_out           (sync_button_rst_n  )
);

hsst_rst_cross_sync_v1_0 #(
    `ifdef IPS2L_PCIE_SPEEDUP_SIM
    .RST_CNTR_VALUE     (16'h10             )
    `else
    .RST_CNTR_VALUE     (16'hC000           )
    `endif
)
u_refclk_perstn_debounce(
    .clk                (ref_clk            ),
    .rstn_in            (perst_n            ),
    .rstn_out           (sync_perst_n       )
);

hsst_rst_sync_v1_0  u_ref_core_rstn_sync    (
    .clk                (ref_clk            ),
    .rst_n              (core_rst_n         ),
    .sig_async          (1'b1               ),
    .sig_synced         (ref_core_rst_n     )
);

hsst_rst_sync_v1_0  u_pclk_core_rstn_sync   (
    .clk                (pclk               ),
    .rst_n              (core_rst_n         ),
    .sig_async          (1'b1               ),
    .sig_synced         (s_pclk_rstn        )
);

always @(posedge ref_clk or negedge sync_perst_n) begin
	if (!sync_perst_n) begin
		ref_led_cnt <= 23'd0;
		ref_led <= 1'b1;
	end else if (smlh_link_up & rdlh_link_up) begin
		ref_led_cnt <= ref_led_cnt + 23'd1;
		if(&ref_led_cnt)
			ref_led <= ~ref_led;
	end
end

always @(posedge pclk or negedge s_pclk_rstn) begin
	if (!s_pclk_rstn) begin
		pclk_led_cnt <= 27'd0;
		pclk_led <= 1'b1;
	end else if (smlh_link_up & rdlh_link_up) begin
		pclk_led_cnt <= pclk_led_cnt + 27'd1;
		if(&pclk_led_cnt)
			pclk_led <= ~pclk_led;
	end
end


//===========================================================================
//pcie dma
//===========================================================================
ips2l_pcie_dma #(
	.DEVICE_TYPE			(DEVICE_TYPE),
	.AXIS_SLAVE_NUM			(AXIS_SLAVE_NUM)
) u_ips2l_pcie_dma (
	.clk					(pclk_div2),				
	.rst_n					(core_rst_n),				

	.i_cfg_pbus_num			(cfg_pbus_num),				
	.i_cfg_pbus_dev_num		(cfg_pbus_dev_num),			
	.i_cfg_max_rd_req_size	(cfg_max_rd_req_size),		
	.i_cfg_max_payload_size	(cfg_max_payload_size),		

	.i_axis_master_tvld		(axis_master_tvalid_mem),	
	.o_axis_master_trdy		(axis_master_tready_mem),	
	.i_axis_master_tdata	(axis_master_tdata_mem),	
	.i_axis_master_tkeep	(axis_master_tkeep_mem),	
														
	.i_axis_master_tlast	(axis_master_tlast_mem),	
	.i_axis_master_tuser	(axis_master_tuser_mem),	

	.i_axis_slave0_trdy		(axis_slave0_tready),		
	.o_axis_slave0_tvld		(dma_axis_slave0_tvalid),	
	.o_axis_slave0_tdata	(dma_axis_slave0_tdata),	
	.o_axis_slave0_tlast	(dma_axis_slave0_tlast),	
	.o_axis_slave0_tuser	(dma_axis_slave0_tuser),	

	.i_axis_slave1_trdy		(axis_slave1_tready),		
	.o_axis_slave1_tvld		(axis_slave1_tvalid),		
	.o_axis_slave1_tdata	(axis_slave1_tdata),		
	.o_axis_slave1_tlast	(axis_slave1_tlast),		
	.o_axis_slave1_tuser	(axis_slave1_tuser),		

	.i_axis_slave2_trdy		(axis_slave2_tready),		
	.o_axis_slave2_tvld		(axis_slave2_tvalid),		
	.o_axis_slave2_tdata	(axis_slave2_tdata),		
	.o_axis_slave2_tlast	(axis_slave2_tlast),		
	.o_axis_slave2_tuser	(axis_slave2_tuser),		

	.i_cfg_ido_req_en		(cfg_ido_req_en),			
	.i_cfg_ido_cpl_en		(cfg_ido_cpl_en),			
	.i_xadm_ph_cdts			(xadm_ph_cdts),				
	.i_xadm_pd_cdts			(xadm_pd_cdts),				
	.i_xadm_nph_cdts		(xadm_nph_cdts),			
	.i_xadm_npd_cdts		(xadm_npd_cdts),			
	.i_xadm_cplh_cdts		(xadm_cplh_cdts),			
	.i_xadm_cpld_cdts		(xadm_cpld_cdts),			

	.i_apb_psel				(p_sel_dma),				
	.i_apb_paddr			(p_addr[8:0]),				
	.i_apb_pwdata			(p_wdata),					
	.i_apb_pstrb			(p_strb),					
	.i_apb_pwrite			(p_we),						
	.i_apb_penable			(p_ce),						
	.o_apb_prdy				(p_rdy_dma),				
	.o_apb_prdata			(p_rdata_dma),				

	.i_lane_commit			(lane_commit_125),		
	.i_lane_frame_id		(lane_frame_id_125),		
	.i_lane_left_state		(lane_left_state_125),	
	.i_lane_right_state		(lane_right_state_125),	
	.i_lane_left_a_q8		(lane_left_a_q8_125),
	.i_lane_right_a_q8		(lane_right_a_q8_125),
	.i_lane_left_b_q8		(lane_left_b_q8_125),
	.i_lane_right_b_q8		(lane_right_b_q8_125),
	.i_lane_left_fit_valid	(lane_left_fit_valid_125),	
	.i_lane_right_fit_valid	(lane_right_fit_valid_125),	
	.i_lane_left_point_count(lane_left_point_count_125),
	.i_lane_right_point_count(lane_right_point_count_125),
	.i_lane_left_fit_raw	(lane_left_fit_raw_125),
	.i_lane_right_fit_raw	(lane_right_fit_raw_125),
	.i_lane_left_max_error	(lane_left_max_error_125),
	.i_lane_right_max_error	(lane_right_max_error_125),
	.i_lane_busy			(lane_busy_125),
	.i_lane_protocol_error	(1'b0),
	.i_lane_dbg				(lane_dbg_125),
	.o_lane_enable			(lane_enable_125),		
	.o_lane_irq_en			(lane_irq_en_125),		
	.o_lane_soft_reset		(lane_soft_reset_125),	
	.o_lane_threshold		(lane_threshold_125),	
	.o_lane_expected_left	(lane_expected_left_125),	
	.o_lane_expected_right	(lane_expected_right_125),	
	.o_lane_window			(lane_window_125),		
	.o_lane_result_seq		(lane_result_seq_125),	

	.i_key0_n				(1'b1),					
	.i_key1_n				(1'b1),					
	.o_decision_led_l		(decision_led_l_125),	
	.o_decision_led_r		(decision_led_r_125),	
	.o_decision_led_m		(decision_led_m_125),	
	.o_decision_alarm		(decision_alarm_125),	
	.o_decision_frame_echo	(decision_frame_echo_125),	
	.o_decision_key_events	(decision_key_events_125),	

	.o_cross_4kb_boundary	(cross_4kb_boundary),		
    .o_dma_write_data_req   (o_dma_write_data_req  ),
    .o_dma_write_addr       (o_dma_write_addr      ),
    .i_dma_write_data       (i_dma_write_data      )
);



assign p_rdy_cfg               = 1'b0;
assign p_rdata_cfg             = 32'b0;

assign axis_slave0_tvalid      = dma_axis_slave0_tvalid;
assign axis_slave0_tlast       = dma_axis_slave0_tlast;
assign axis_slave0_tuser       = dma_axis_slave0_tuser;
assign axis_slave0_tdata       = dma_axis_slave0_tdata;

assign axis_master_tvalid_mem  = axis_master_tvalid;
assign axis_master_tdata_mem   = axis_master_tdata;
assign axis_master_tkeep_mem   = axis_master_tkeep;
assign axis_master_tlast_mem   = axis_master_tlast;
assign axis_master_tuser_mem   = axis_master_tuser;

assign axis_master_tready      = axis_master_tready_mem;



pcie_test u_ips2l_pcie_wrap (
	.button_rst_n				(sync_button_rst_n),
	.power_up_rst_n				(sync_perst_n),
	.perst_n					(sync_perst_n),

	.pclk						(pclk),					
	.pclk_div2					(pclk_div2),			
	.ref_clk					(ref_clk),				
	.ref_clk_n					(ref_clk_n),			
	.ref_clk_p					(ref_clk_p),			
	.core_rst_n					(core_rst_n),			

	.p_sel						(p_sel_pcie),			
	.p_strb						(uart_p_strb),			
	.p_addr						(uart_p_addr),			
	.p_wdata					(uart_p_wdata),			
	.p_ce						(uart_p_ce),			
	.p_we						(uart_p_we),			
	.p_rdy						(p_rdy_pcie),			
	.p_rdata					(p_rdata_pcie),			

	.rxn						(rxn),					
	.rxp						(rxp),					
	.txn						(txn),					
	.txp						(txp),					
	.pcs_nearend_loop			({4{1'b0}}),			
	.pma_nearend_ploop			({4{1'b0}}),			
	.pma_nearend_sloop			({4{1'b0}}),			

	.axis_master_tvalid			(axis_master_tvalid),	
	.axis_master_tready			(axis_master_tready),	
	.axis_master_tdata			(axis_master_tdata),	
	.axis_master_tkeep			(axis_master_tkeep),	
														
	.axis_master_tlast			(axis_master_tlast),	
	.axis_master_tuser			(axis_master_tuser),	

	.axis_slave0_tready			(axis_slave0_tready),	
	.axis_slave0_tvalid			(axis_slave0_tvalid),	
	.axis_slave0_tdata			(axis_slave0_tdata),	
	.axis_slave0_tlast			(axis_slave0_tlast),	
	.axis_slave0_tuser			(axis_slave0_tuser),	

	.axis_slave1_tready			(axis_slave1_tready),	
	.axis_slave1_tvalid			(axis_slave1_tvalid),	
	.axis_slave1_tdata			(axis_slave1_tdata),	
	.axis_slave1_tlast			(axis_slave1_tlast),	
	.axis_slave1_tuser			(axis_slave1_tuser),	

	.axis_slave2_tready			(axis_slave2_tready),	
	.axis_slave2_tvalid			(axis_slave2_tvalid),	
	.axis_slave2_tdata			(axis_slave2_tdata),	
	.axis_slave2_tlast			(axis_slave2_tlast),	
	.axis_slave2_tuser			(axis_slave2_tuser),	

	.pm_xtlh_block_tlp			(),						

	.cfg_send_cor_err_mux		(),						
	.cfg_send_nf_err_mux		(),						
	.cfg_send_f_err_mux			(),						
	.cfg_sys_err_rc				(),						
	.cfg_aer_rc_err_mux			(),						

	.radm_cpl_timeout			(),						

	.cfg_max_rd_req_size		(cfg_max_rd_req_size),	
	.cfg_bus_master_en			(),						
	.cfg_max_payload_size		(cfg_max_payload_size),	
	.cfg_ext_tag_en				(),						
	.cfg_rcb					(cfg_rcb),				
	.cfg_mem_space_en			(),						
	.cfg_pm_no_soft_rst			(),						
	.cfg_crs_sw_vis_en			(),						
	.cfg_no_snoop_en			(),						
	.cfg_relax_order_en			(),						
	.cfg_tph_req_en				(),						
	.cfg_pf_tph_st_mode			(),						
	.rbar_ctrl_update			(),						
	.cfg_atomic_req_en			(),						

	.cfg_pbus_num				(cfg_pbus_num),			
	.cfg_pbus_dev_num			(cfg_pbus_dev_num),		

	.radm_idle					(),						
	.radm_q_not_empty			(),						
	.radm_qoverflow				(),						
	.diag_ctrl_bus				(2'b0),					
	.cfg_link_auto_bw_mux		(),						
	.cfg_bw_mgt_mux				(),						
	.cfg_pme_mux				(),						
	.app_ras_des_sd_hold_ltssm	(1'b0),					
	.app_ras_des_tba_ctrl		(2'b0),					

	.dyn_debug_info_sel			(4'b0),					
	.debug_info_mux				(),

	.smlh_link_up				(smlh_link_up),			
	.rdlh_link_up				(rdlh_link_up),			
	.smlh_ltssm_state			(smlh_ltssm_state)
);



//=======================
// A frame is 1280 x 360 RGB565 pixels. The BAR2 read path transfers eight
// 16-bit pixels per 128-bit beat, so a frame request must follow 57,600 actual
// DMA read beats. The old 0x9f -> 0xa0 address-transition counter depended on
// an unreliable address pattern and did not represent a complete frame.
localparam [15:0] FRAME_DMA_BEATS = 16'd57600;
reg [15:0] dma_beat_cnt;
wire dma_frame_complete = o_dma_write_data_req &&
                           (dma_beat_cnt == FRAME_DMA_BEATS - 1'b1);

always @(posedge pclk_div2) begin
    if (!core_rst_n) begin
        dma_beat_cnt <= 16'd0;
    end else if (o_dma_write_data_req) begin
        if (dma_beat_cnt == FRAME_DMA_BEATS - 1'b1)
            dma_beat_cnt <= 16'd0;
        else
            dma_beat_cnt <= dma_beat_cnt + 1'b1;
    end
end

always @(posedge pclk_div2) begin
    if (!core_rst_n) begin
        ch0_rframe_req <= 1'b0;
    end else if (dma_frame_complete) begin
        ch0_rframe_req <= 1'b1;
    end else if (ch0_rframe_req_ack) begin
        ch0_rframe_req <= 1'b0;
    end
end






endmodule
