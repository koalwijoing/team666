module cmos_write_req_gen(
	input              rst,
	input              pclk,
	input              cmos_vsync,
	output reg         write_req,
	output reg[1:0]    write_addr_index,
	output reg[1:0]    read_addr_index,
	input              write_frame_done_toggle,
	input              write_req_ack
);

reg cmos_vsync_d0;
reg cmos_vsync_d1;

reg write_done_meta;
reg write_done_sync;
reg write_done_sync_d;
reg [1:0] pending_read_addr_index;

// Fallback so the read path can never dead-lock if the frame-done toggle
// never arrives: commit the pending read index after ~8 ms at 42 MHz anyway.
reg [19:0] vsync_wait_cnt;
reg        pending_commit_timeout;

wire write_done_pulse = write_done_sync ^ write_done_sync_d;

always@(posedge pclk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		cmos_vsync_d0 <= 1'b0;
		cmos_vsync_d1 <= 1'b0;
	end
	else
	begin
		cmos_vsync_d0 <= cmos_vsync;
		cmos_vsync_d1 <= cmos_vsync_d0;
	end
end

// Synchronize the frame-complete toggle from the DDR/memory clock domain.
always@(posedge pclk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		write_done_meta <= 1'b0;
		write_done_sync <= 1'b0;
		write_done_sync_d <= 1'b0;
	end
	else
	begin
		write_done_meta <= write_frame_done_toggle;
		write_done_sync <= write_done_meta;
		write_done_sync_d <= write_done_sync;
	end
end

always@(posedge pclk or posedge rst)
begin
	if(rst == 1'b1)
		write_req <= 1'b0;
	else if(cmos_vsync_d0 == 1'b1 && cmos_vsync_d1 == 1'b0)
		write_req <= 1'b1;
	else if(write_req_ack == 1'b1)
		write_req <= 1'b0;
end

always@(posedge pclk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		write_addr_index <= 2'b0;
		pending_read_addr_index <= 2'b0;
	end
	else if(cmos_vsync_d0 == 1'b1 && cmos_vsync_d1 == 1'b0)
	begin
		write_addr_index <= write_addr_index + 2'd1;
		pending_read_addr_index <= write_addr_index;
	end
end

always@(posedge pclk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		vsync_wait_cnt <= 20'd0;
		pending_commit_timeout <= 1'b0;
	end
	else if(cmos_vsync_d0 == 1'b1 && cmos_vsync_d1 == 1'b0)
	begin
		vsync_wait_cnt <= 20'd0;
		pending_commit_timeout <= 1'b0;
	end
	else if(write_done_pulse == 1'b1)
	begin
		vsync_wait_cnt <= 20'd0;
		pending_commit_timeout <= 1'b0;
	end
	else if(vsync_wait_cnt >= 20'd336000)
		pending_commit_timeout <= 1'b1;
	else
		vsync_wait_cnt <= vsync_wait_cnt + 20'd1;
end

always@(posedge pclk or posedge rst)
begin
	if(rst == 1'b1)
		read_addr_index <= 2'b0;
	else if(write_done_pulse == 1'b1 || pending_commit_timeout == 1'b1)
		read_addr_index <= pending_read_addr_index;
end

endmodule
