//////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2019 PANGO MICROSYSTEMS, INC
// ALL RIGHTS REVERVED.
//
// THE SOURCE CODE CONTAINED HEREIN IS PROPRIETARY TO PANGO MICROSYSTEMS, INC.
// IT SHALL NOT BE REPRODUCED OR DISCLOSED IN WHOLE OR IN PART OR USED BY
// PARTIES WITHOUT WRITTEN AUTHORIZATION FROM THE OWNER.
//
//////////////////////////////////////////////////////////////////////////////
//
// Library:
// Filename:ips2l_pcie_dma_rx_top.v
//////////////////////////////////////////////////////////////////////////////
module ips2l_pcie_dma_rx_top #(
    parameter                           DEVICE_TYPE = 3'd0      ,   //3'd0:EP,3'd1:Legacy EP,3'd4:RC
    parameter                           ADDR_WIDTH  = 4'd9
)(
    input                               clk                     ,   //gen1:62.5MHz,gen2:125MHz
    input                               rst_n                   ,
    input           [2:0]               i_cfg_max_rd_req_size   ,
    //**********************************************************************
    //AXIS master interface
    input                               i_axis_master_tvld      ,
    output  wire                        o_axis_master_trdy      ,
    input           [127:0]             i_axis_master_tdata     ,
    input           [3:0]               i_axis_master_tkeep     ,
    input                               i_axis_master_tlast     ,
    input           [7:0]               i_axis_master_tuser     ,
    output  wire    [2:0]               o_trgt1_radm_pkt_halt   ,
//    input           [5:0]               i_radm_grant_tlp_type   ,
    //**********************************************************************
    //bar0 rd interface
    input                               i_bar0_rd_clk_en        ,
    input           [ADDR_WIDTH-1:0]    i_bar0_rd_addr          ,
    output  wire    [127:0]             o_bar0_rd_data          ,
    input           [31:0]              i_abi_done_count        ,

    //lane result snapshot and BAR0 register window
    input                               i_lane_commit           ,
    input           [31:0]              i_lane_frame_id         ,
    input           [1:0]               i_lane_left_state       ,
    input           [1:0]               i_lane_right_state      ,
    input  signed   [31:0]              i_lane_left_a_q8        ,
    input  signed   [31:0]              i_lane_right_a_q8       ,
    input  signed   [31:0]              i_lane_left_b_q8        ,
    input  signed   [31:0]              i_lane_right_b_q8       ,
    input                               i_lane_left_fit_valid   ,
    input                               i_lane_right_fit_valid  ,
    input           [5:0]               i_lane_left_point_count ,
    input           [5:0]               i_lane_right_point_count,
    input                               i_lane_left_fit_raw     ,
    input                               i_lane_right_fit_raw    ,
    input           [11:0]              i_lane_left_max_error   ,
    input           [11:0]              i_lane_right_max_error  ,
    input                               i_lane_busy             ,
    input                               i_lane_protocol_error   ,
    input       [31:0]                  i_lane_dbg              ,
    output  wire                        o_lane_enable           ,
    output  wire                        o_lane_irq_en           ,
    output  wire                        o_lane_soft_reset       ,
    output  wire    [10:0]              o_lane_threshold        ,
    output  wire    [10:0]              o_lane_expected_left    ,
    output  wire    [10:0]              o_lane_expected_right   ,
    output  wire    [10:0]              o_lane_window           ,
    output  wire    [31:0]              o_lane_result_seq       ,

    //decision output and key registers at BAR0+0x200
    input                               i_key0_n                ,
    input                               i_key1_n                ,
    output  wire                        o_decision_led_l        ,
    output  wire                        o_decision_led_r        ,
    output  wire                        o_decision_led_m        ,
    output  wire                        o_decision_alarm        ,
    output  wire    [15:0]              o_decision_frame_echo   ,
    output  wire    [4:0]               o_decision_key_events   ,

    //bar2 rd interface
    input                               i_bar2_rd_clk_en        ,
    input           [ADDR_WIDTH-1:0]    i_bar2_rd_addr          ,
    output  wire    [127:0]             o_bar2_rd_data          ,

    //bar1 wr interface
    output  reg                         o_bar1_wr_en            ,
    output  reg     [ADDR_WIDTH-1:0]    o_bar1_wr_addr          ,
    output  reg     [127:0]             o_bar1_wr_data          ,
    output  reg     [15:0]              o_bar1_wr_byte_en       ,
    //**********************************************************************
    //to tx top
    //req rcv
    output  wire    [2:0]               o_mrd_tc                ,
    output  wire    [2:0]               o_mrd_attr              ,
    output  wire    [9:0]               o_mrd_length            ,
    output  wire    [15:0]              o_mrd_id                ,
    output  wire    [7:0]               o_mrd_tag               ,
    output  wire    [63:0]              o_mrd_addr              ,

    output  wire                        o_cpld_req_vld          ,
    input                               i_cpld_req_rdy          ,
    input                               i_cpld_tx_rdy           ,
    //cpld rcv
    output wire                         o_cpld_rcv              ,
    output wire     [7:0]               o_cpld_tag              ,
    input                               i_tag_full              ,
    //rst tlp cnt
    output wire     [63:0]              o_dma_check_result      ,
    input                               i_tx_restart

    //output  wire    [42:0]              o_dbg_bus_rx_ctrl       ,
    //output  wire    [69:0]              o_dbg_tlp_rcv_cnt
);

wire                        mwr_wr_start;
wire    [9:0]               mwr_length;
wire    [7:0]               mwr_dwbe;
wire    [127:0]             mwr_data;
wire    [3:0]               mwr_dw_vld;
wire    [63:0]              mwr_addr;

wire                        cpld_wr_start;
wire    [9:0]               cpld_length;
wire    [6:0]               cpld_low_addr;
wire    [11:0]              cpld_byte_cnt;
wire    [127:0]             cpld_data;
wire    [3:0]               cpld_dw_vld;
wire                        multicpld_flag;

wire    [1:0]               bar_hit;

wire                        lane_wr_hit;
wire                        lane_rd_hit;
wire    [127:0]             lane_rd_data;
wire                        decision_wr_hit;
wire                        decision_rd_hit;
wire    [127:0]             decision_rd_data;
wire    [127:0]             bar0_rd_data_ram;

// PANG completion-status ABI used by the vendor host driver.
// Host reads of 0x1F0/0x1F4/0x1F8 arrive one 128-bit word later at 0x20.
wire                        abi_rd_hit = (i_bar0_rd_addr == 9'h20);
// The driver only consumes word0 (completion count) and word1 (PANG
// signature). Reuse the two otherwise-unused upper words for a compact
// lane snapshot:
//   0x1F8 = LANE_SEQ (increments once per lane commit)
//   0x1FC = {flags[7:0], FRAME_ID[23:0]}
// flags = {enable, protocol_error, right_fit, left_fit,
//          right_state[1:0], left_state[1:0]}
wire    [7:0]               lane_abi_flags =
    {o_lane_enable, i_lane_protocol_error,
     i_lane_right_fit_valid, i_lane_left_fit_valid,
     i_lane_right_state, i_lane_left_state};
wire    [31:0]              lane_abi_packed =
    {lane_abi_flags, i_lane_frame_id[23:0]};
// Latch the packed lane snapshot on each lane commit so the ARM reader
// always sees an atomic word pair, and give it a per-commit sequence
// number to detect a stale/frozen lane datapath (fail-closed on ARM).
reg     [31:0]              lane_abi_seq_q;
reg     [31:0]              lane_abi_packed_q;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        lane_abi_seq_q    <= 32'd0;
        lane_abi_packed_q <= 32'd0;
    end else if (i_lane_commit) begin
        lane_abi_seq_q    <= lane_abi_seq_q + 32'd1;
        lane_abi_packed_q <= lane_abi_packed;
    end
end
wire    [127:0]             abi_rd_data =
    {lane_abi_packed_q, lane_abi_seq_q, 32'h50414E47, i_abi_done_count};

//mwr wr
wire                        mwr_wr_en;
wire    [ADDR_WIDTH-1:0]    mwr_wr_addr;
wire    [127:0]             mwr_wr_data;
wire    [15:0]              mwr_wr_be;
wire    [1:0]               mwr_wr_bar_hit/*synthesis PAP_MARK_DEBUG="1"*/;
//cpld wr
wire                        cpld_wr_en;
wire    [ADDR_WIDTH-1:0]    cpld_wr_addr;
wire    [127:0]             cpld_wr_data;
wire    [15:0]              cpld_wr_be;
wire    [1:0]               cpld_wr_bar_hit;
//bar0 wr interface
reg                         bar0_wr_en/*synthesis PAP_MARK_DEBUG="1"*/;
reg     [ADDR_WIDTH-1:0]    bar0_wr_addr;
reg     [127:0]             bar0_wr_data/*synthesis PAP_MARK_DEBUG="1"*/;
reg     [15:0]              bar0_wr_byte_en;
//bar2 wr interface
wire                        bar2_wr_en;
wire    [ADDR_WIDTH-1:0]    bar2_wr_addr;
wire    [127:0]             bar2_wr_data;
wire    [15:0]              bar2_wr_byte_en;

ips2l_pcie_dma_tlp_rcv #(
    .DEVICE_TYPE            (DEVICE_TYPE            )
)
u_ips2l_pcie_dma_tlp_rcv
(
    .clk                    (clk                    ),  //gen1:62.5MHz,gen2:125MHz
    .rst_n                  (rst_n                  ),

    //**********************************************************************
    //AXIS master interface
    .i_axis_master_tvld     (i_axis_master_tvld     ),
    .o_axis_master_trdy     (o_axis_master_trdy     ),
    .i_axis_master_tdata    (i_axis_master_tdata    ),
    .i_axis_master_tkeep    (i_axis_master_tkeep    ),
    .i_axis_master_tlast    (i_axis_master_tlast    ),
    .i_axis_master_tuser    (i_axis_master_tuser    ),
    .o_trgt1_radm_pkt_halt  (o_trgt1_radm_pkt_halt  ),
//    .i_radm_grant_tlp_type  (i_radm_grant_tlp_type  ),

    //**********************************************************************
    //to mwr write control
    .o_mwr_wr_start         (mwr_wr_start           ),
    .o_mwr_length           (mwr_length             ),
    .o_mwr_dwbe             (mwr_dwbe               ),
    .o_mwr_data             (mwr_data               ),
    .o_mwr_dw_vld           (mwr_dw_vld             ),
    .o_mwr_addr             (mwr_addr               ),
    //to cpld write control
    .o_cpld_wr_start        (cpld_wr_start          ),
    .o_cpld_length          (cpld_length            ),
    .o_cpld_low_addr        (cpld_low_addr          ),
    .o_cpld_byte_cnt        (cpld_byte_cnt          ),
    .o_cpld_data            (cpld_data              ),
    .o_cpld_dw_vld          (cpld_dw_vld            ),
    .o_multicpld_flag       (multicpld_flag         ),
    //write bar hit
    .o_bar_hit              (bar_hit                ),
    //**********************************************************************
    //to tx top
    //req rcv
    .o_mrd_tc               (o_mrd_tc               ),
    .o_mrd_attr             (o_mrd_attr             ),
    .o_mrd_length           (o_mrd_length           ),
    .o_mrd_id               (o_mrd_id               ),
    .o_mrd_tag              (o_mrd_tag              ),
    .o_mrd_addr             (o_mrd_addr             ),

    .o_cpld_req_vld         (o_cpld_req_vld         ),
    .i_cpld_req_rdy         (i_cpld_req_rdy         ),
    .i_cpld_tx_rdy          (i_cpld_tx_rdy          ),
    //cpld rcv
    .o_cpld_rcv             (o_cpld_rcv             ),
    .o_cpld_tag             (o_cpld_tag             ),
    .i_tag_full             (i_tag_full             ),
    //rst tlp cnt
    .o_dma_check_result     (o_dma_check_result     ),
    .i_tx_restart           (i_tx_restart           )
    //.o_dbg_bus              (o_dbg_bus_rx_ctrl      ),
    //.o_dbg_tlp_rcv_cnt      (o_dbg_tlp_rcv_cnt      )
);


ips2l_pcie_dma_rx_cpld_wr_ctrl #(
    .ADDR_WIDTH             (ADDR_WIDTH             )
)
u_cpld_wr_ctrl
(
    .clk                    (clk                    ),  //gen1:62.5MHz,gen2:125MHz
    .rst_n                  (rst_n                  ),
    .i_cfg_max_rd_req_size  (i_cfg_max_rd_req_size  ),  //input [2:0]
    //****************************************
    .i_cpld_wr_start        (cpld_wr_start          ),
    .i_cpld_length          (cpld_length            ),
    .i_cpld_low_addr        (cpld_low_addr          ),
    .i_cpld_byte_cnt        (cpld_byte_cnt          ),
    .i_cpld_data            (cpld_data              ),
    .i_cpld_dw_vld          (cpld_dw_vld            ),
    .i_cpld_tag             (o_cpld_tag             ),
    .i_bar_hit              (bar_hit                ),
    .i_multicpld_flag       (multicpld_flag         ),
    //****************************************
    .o_cpld_wr_en           (cpld_wr_en             ),
    .o_cpld_wr_addr         (cpld_wr_addr           ),
    .o_cpld_wr_data         (cpld_wr_data           ),
    .o_cpld_wr_be           (cpld_wr_be             ),
    .o_cpld_wr_bar_hit      (cpld_wr_bar_hit        )
);

ips2l_pcie_dma_rx_mwr_wr_ctrl #(
    .ADDR_WIDTH             (ADDR_WIDTH             )
)
u_mwr_wr_ctrl
(
    .clk                    (clk                    ),  //gen1:62.5MHz,gen2:125MHz
    .rst_n                  (rst_n                  ),
    //****************************************
    .i_mwr_wr_start         (mwr_wr_start           ),
    .i_mwr_length           (mwr_length             ),
    .i_mwr_dwbe             (mwr_dwbe               ),
    .i_mwr_data             (mwr_data               ),
    .i_mwr_dw_vld           (mwr_dw_vld             ),
    .i_mwr_addr             (mwr_addr               ),
    .i_bar_hit              (bar_hit                ),
    //****************************************
    .o_mwr_wr_en            (mwr_wr_en              ),
    .o_mwr_wr_addr          (mwr_wr_addr            ),
    .o_mwr_wr_data          (mwr_wr_data            ),
    .o_mwr_wr_be            (mwr_wr_be              ),
    .o_mwr_wr_bar_hit       (mwr_wr_bar_hit         )
);

// Lane result registers occupy BAR0 aligned words 0x1C0/0x1D0/0x1E0.
lane_result_bar0 u_lane_result_bar0 (
    .clk                    (clk                    ),
    .rst_n                  (rst_n                  ),
    .commit_i               (i_lane_commit           ),
    .commit_frame_id_i      (i_lane_frame_id         ),
    .commit_left_state_i    (i_lane_left_state       ),
    .commit_right_state_i   (i_lane_right_state      ),
    .commit_left_a_q8_i     (i_lane_left_a_q8        ),
    .commit_right_a_q8_i    (i_lane_right_a_q8       ),
    .commit_left_b_q8_i     (i_lane_left_b_q8        ),
    .commit_right_b_q8_i    (i_lane_right_b_q8       ),
    .commit_left_fit_valid_i(i_lane_left_fit_valid   ),
    .commit_right_fit_valid_i(i_lane_right_fit_valid ),
    .commit_left_point_count_i(i_lane_left_point_count),
    .commit_right_point_count_i(i_lane_right_point_count),
    .commit_left_fit_valid_raw_i(i_lane_left_fit_raw ),
    .commit_right_fit_valid_raw_i(i_lane_right_fit_raw),
    .commit_left_max_error_i(i_lane_left_max_error   ),
    .commit_right_max_error_i(i_lane_right_max_error ),
    .busy_i                 (i_lane_busy             ),
    .protocol_error_i       (i_lane_protocol_error   ),
    .commit_dbg_i           (i_lane_dbg              ),
    .lane_enable_o          (o_lane_enable           ),
    .irq_en_o               (o_lane_irq_en           ),
    .lane_soft_reset_o      (o_lane_soft_reset       ),
    .threshold_o            (o_lane_threshold        ),
    .expected_left_o        (o_lane_expected_left    ),
    .expected_right_o       (o_lane_expected_right   ),
    .window_o               (o_lane_window           ),
    .result_seq_o           (o_lane_result_seq       ),
    .bar0_wr_en_i           (mwr_wr_en && (mwr_wr_bar_hit == 2'b00)),
    .bar0_wr_addr_i         (mwr_wr_addr[8:0]        ),
    .bar0_wr_data_i         (mwr_wr_data             ),
    .bar0_wr_be_i           (mwr_wr_be               ),
    .bar0_rd_addr_i         (i_bar0_rd_addr[8:0]     ),
    .write_hit_o            (lane_wr_hit             ),
    .read_hit_o             (lane_rd_hit             ),
    .read_data_o            (lane_rd_data            )
);

p3_decision_bar0 u_p3_decision_bar0 (
    .clk                    (clk                    ),
    .rst_n                  (rst_n                  ),
    .key0_n                 (i_key0_n               ),
    .key1_n                 (i_key1_n               ),
    .led_l_o                (o_decision_led_l       ),
    .led_r_o                (o_decision_led_r       ),
    .led_m_o                (o_decision_led_m       ),
    .alarm_o                (o_decision_alarm       ),
    .frame_echo_o           (o_decision_frame_echo  ),
    .key_events_o           (o_decision_key_events  ),
    .bar0_wr_en_i           (mwr_wr_en && (mwr_wr_bar_hit == 2'b00)),
    .bar0_wr_addr_i         (mwr_wr_addr[8:0]        ),
    .bar0_wr_data_i         (mwr_wr_data             ),
    .bar0_wr_be_i           (mwr_wr_be               ),
    .bar0_rd_addr_i         (i_bar0_rd_addr[8:0]     ),
    .write_hit_o            (decision_wr_hit         ),
    .read_hit_o             (decision_rd_hit         ),
    .read_data_o            (decision_rd_data        )
);

//bar0 interface
always@(*)
begin
    if(mwr_wr_bar_hit == 2'b0 && !lane_wr_hit && !decision_wr_hit)
    begin
        bar0_wr_en       = mwr_wr_en;
        bar0_wr_addr     = mwr_wr_addr;
        bar0_wr_data     = mwr_wr_data;
        bar0_wr_byte_en  = mwr_wr_be;
    end
    else
    begin
        bar0_wr_en       = 1'b0;
        bar0_wr_addr     = {ADDR_WIDTH{1'b0}};
        bar0_wr_data     = 128'b0;
        bar0_wr_byte_en  = 16'b0;
    end
end
     
//bar1 interface 
always@(*)
begin
    if(bar_hit == 2'b01 && (DEVICE_TYPE == 3'b000 || DEVICE_TYPE == 3'b001))
    begin
        o_bar1_wr_en       = 1'b1;
        o_bar1_wr_addr     = mwr_addr[ADDR_WIDTH-1:0];
        o_bar1_wr_data     = mwr_data;
        o_bar1_wr_byte_en  = {{4{mwr_dwbe[3]}},{4{mwr_dwbe[2]}},{4{mwr_dwbe[1]}},{4{mwr_dwbe[0]}}};
    end
    else
    begin
        o_bar1_wr_en       = 1'b0;
        o_bar1_wr_addr     = {ADDR_WIDTH{1'b0}};
        o_bar1_wr_data     = 128'b0;
        o_bar1_wr_byte_en  = 16'b0;
    end
end

assign bar2_wr_en       = cpld_wr_en;
assign bar2_wr_addr     = cpld_wr_addr;
assign bar2_wr_data     = cpld_wr_data;
assign bar2_wr_byte_en  = cpld_wr_be;

ips2l_pcie_dma_ram ips2l_pcie_dma_bar0 (
    .wr_data            (bar0_wr_data               ),  // input [127:0]
    .wr_addr            (bar0_wr_addr               ),  // input [8:0]
    .wr_en              (bar0_wr_en                 ),  // input
    .wr_byte_en         (bar0_wr_byte_en            ),  // input [15:0]
    .wr_clk             (clk                        ),  // input
    .wr_rst             (~rst_n                     ),  // input
    .rd_addr            (i_bar0_rd_addr             ),  // input [8:0]
    .rd_data            (bar0_rd_data_ram           ),  // output [127:0]
    .rd_clk             (clk                        ),  // input
    .rd_clk_en          (i_bar0_rd_clk_en           ),  // input
    .rd_rst             (~rst_n                     )   // input
);

assign o_bar0_rd_data = abi_rd_hit       ? abi_rd_data       :
                        decision_rd_hit ? decision_rd_data :
                        lane_rd_hit     ? lane_rd_data     :
                                          bar0_rd_data_ram;

ips2l_pcie_dma_ram ips2l_pcie_dma_bar2 (
    .wr_data            (bar2_wr_data               ),  // input [127:0]
    .wr_addr            (bar2_wr_addr               ),  // input [8:0]
    .wr_en              (bar2_wr_en                 ),  // input
    .wr_byte_en         (bar2_wr_byte_en            ),  // input [15:0]
    .wr_clk             (clk                        ),  // input
    .wr_rst             (~rst_n                     ),  // input
    .rd_addr            (i_bar2_rd_addr             ),  // input [8:0]
    .rd_data            (o_bar2_rd_data             ),  // output [127:0]
    .rd_clk             (clk                        ),  // input
    .rd_clk_en          (i_bar2_rd_clk_en           ),  // input
    .rd_rst             (~rst_n                     )   // input
);

endmodule
