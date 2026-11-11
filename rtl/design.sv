// 8b/10b Line Encoder (IBM Standard Partitioning: 5b/6b + 3b/4b)
module encode_8b10b (
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] data_in,
    input  wire       is_k,        // 1 for special control char (e.g., K28.5), 0 for data
    output reg  [9:0] data_out,
    output reg        disparity    // 0 = negative/neutral, 1 = positive
);
    wire a = data_in[0];
    wire b = data_in[1];
    wire c = data_in[2];
    wire d = data_in[3];
    wire e = data_in[4];
    wire f = data_in[5];
    wire g = data_in[6];
    wire h = data_in[7];

    reg [5:0] sub_5b6b;
    reg [3:0] sub_3b4b;
    reg       disp_after_6b;

    // 5b/6b Encoding Logic
    always @(*) begin
        if (is_k && data_in == 8'hBC) begin // K28.5 special character
            sub_5b6b = disparity ? 6'b001111 : 6'b110000;
            disp_after_6b = ~disparity;
        end else begin
            case (data_in[4:0])
                5'd0:  sub_5b6b = disparity ? 6'b010011 : 6'b101100;
                5'd1:  sub_5b6b = disparity ? 6'b100011 : 6'b011100;
                5'd2:  sub_5b6b = disparity ? 6'b010010 : 6'b101101;
                5'd3:  sub_5b6b = 6'b110001;
                5'd4:  sub_5b6b = disparity ? 6'b001010 : 6'b110101;
                5'd5:  sub_5b6b = 6'b101001;
                5'd6:  sub_5b6b = 6'b011001;
                5'd7:  sub_5b6b = disparity ? 6'b000111 : 6'b111000;
                5'd8:  sub_5b6b = disparity ? 6'b000110 : 6'b111001;
                5'd9:  sub_5b6b = 6'b100101;
                5'd10: sub_5b6b = 6'b010101;
                5'd11: sub_5b6b = 6'b110100;
                5'd12: sub_5b6b = 6'b001101;
                5'd13: sub_5b6b = 6'b101100;
                5'd14: sub_5b6b = 6'b011100;
                5'd15: sub_5b6b = disparity ? 6'b010111 : 6'b101000;
                default: sub_5b6b = disparity ? 6'b011011 : 6'b100100;
            endcase
            disp_after_6b = disparity ^ (^sub_5b6b);
        end
    end

    // 3b/4b Encoding Logic
    always @(*) begin
        if (is_k && data_in == 8'hBC) begin
            sub_3b4b = disp_after_6b ? 6'b0101 : 6'b1010;
        end else begin
            case (data_in[7:5])
                3'd0: sub_3b4b = disp_after_6b ? 4'b0100 : 4'b1011;
                3'd1: sub_3b4b = 4'b1001;
                3'd2: sub_3b4b = 4'b0101;
                3'd3: sub_3b4b = disp_after_6b ? 4'b0011 : 4'b1100;
                3'd4: sub_3b4b = disp_after_6b ? 4'b0010 : 4'b1101;
                3'd5: sub_3b4b = 4'b1010;
                3'd6: sub_3b4b = 4'b0110;
                3'd7: sub_3b4b = disp_after_6b ? 4'b0001 : 4'b1110;
                default: sub_3b4b = 4'b1010;
            endcase
        end
    end

    // Sequential Register & Running Disparity Update
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            data_out  <= 10'd0;
            disparity <= 1'b0;
        end else begin
            data_out  <= {sub_3b4b, sub_5b6b};
            disparity <= disp_after_6b ^ (^sub_3b4b);
        end
    end
endmodule

// 8b/10b Line Decoder
module decode_8b10b (
    input  wire       clk,
    input  wire       rst_n,
    input  wire [9:0] data_in,
    output reg  [7:0] data_out,
    output reg        is_k_out,
    output reg        code_err
);
    wire [5:0] sub_6b = data_in[5:0];
    wire [3:0] sub_4b = data_in[9:6];

    reg [4:0] dec_5b;
    reg [2:0] dec_3b;
    reg       err_6b, err_4b, k_det;

    // 6b to 5b Decode
    always @(*) begin
        err_6b = 1'b0;
        k_det  = 1'b0;
        case (sub_6b)
            6'b001111, 6'b110000: begin dec_5b = 5'h1C; k_det = 1'b1; end // K28 prefix
            6'b010011, 6'b101100: dec_5b = 5'd0;
            6'b100011, 6'b011100: dec_5b = 5'd1;
            6'b010010, 6'b101101: dec_5b = 5'd2;
            6'b110001:            dec_5b = 5'd3;
            6'b001010, 6'b110101: dec_5b = 5'd4;
            6'b101001:            dec_5b = 5'd5;
            6'b011001:            dec_5b = 5'd6;
            6'b000111, 6'b111000: dec_5b = 5'd7;
            6'b000110, 6'b111001: dec_5b = 5'd8;
            6'b100101:            dec_5b = 5'd9;
            6'b010101:            dec_5b = 5'd10;
            6'b110100:            dec_5b = 5'd11;
            6'b001101:            dec_5b = 5'd12;
            6'b101100:            dec_5b = 5'd13;
            6'b011100:            dec_5b = 5'd14;
            6'b010111, 6'b101000: dec_5b = 5'd15;
            default: begin dec_5b = 5'd0; err_6b = 1'b1; end
        endcase
    end

    // 4b to 3b Decode
    always @(*) begin
        err_4b = 1'b0;
        case (sub_4b)
            4'b0100, 4'b1011: dec_3b = 3'd0;
            4'b1001:          dec_3b = 3'd1;
            4'b0101:          dec_3b = 3'd2;
            4'b0011, 4'b1100: dec_3b = 3'd3;
            4'b0010, 4'b1101: dec_3b = 3'd4;
            4'b1010:          dec_3b = 3'd5;
            4'b0110:          dec_3b = 3'd6;
            4'b0001, 4'b1110: dec_3b = 3'd7;
            default: begin dec_3b = 3'd0; err_4b = 1'b1; end
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            data_out <= 8'd0;
            is_k_out <= 1'b0;
            code_err <= 1'b0;
        end else begin
            data_out <= {dec_3b, dec_5b};
            is_k_out <= k_det;
            code_err <= err_6b | err_4b;
        end
    end
endmodule

// Serializer / Deserializer Loopback Subsystem
module transceiver_8b10b_top (
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] tx_data,
    input  wire       tx_isk,
    input  wire       inject_err, // Injects a bit error to test decoder validation
    output wire [7:0] rx_data,
    output wire       rx_isk,
    output wire       rx_err,
    output wire       tx_disp
);
    wire [9:0] encoded_data;
    reg  [9:0] channel_reg;

    encode_8b10b enc_inst (
        .clk(clk),
        .rst_n(rst_n),
        .data_in(tx_data),
        .is_k(tx_isk),
        .data_out(encoded_data),
        .disparity(tx_disp)
    );

    // Simulated 1-cycle Serial Channel
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            channel_reg <= 10'd0;
        else if (inject_err)
            channel_reg <= encoded_data ^ 10'b0000000001; // Corrupt LSB
        else
            channel_reg <= encoded_data;
    end

    decode_8b10b dec_inst (
        .clk(clk),
        .rst_n(rst_n),
        .data_in(channel_reg),
        .data_out(rx_data),
        .is_k_out(rx_isk),
        .code_err(rx_err)
    );
endmodule