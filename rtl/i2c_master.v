`timescale 1ns / 1ps
//==============================================================================
// Модуль  : i2c_master
// Опис    : I2C Master (режим ЗАПИСУ): START -> ADDR+W -> ACK -> DATA -> ACK -> STOP
// Мова    : Verilog-2001
// Тактова : CLK_FREQ = 50 МГц, швидкість шини I2C_FREQ = 100 кГц (Standard-mode)
//
// Часова схема. Один біт I2C ділиться на 4 фази (чверті періоду SCL).
// Строб tick надходить кожні TICK_DIV = CLK_FREQ / (I2C_FREQ * 4) = 125 тактів clk.
//   фаза 0 : SCL = 0, Master виставляє біт на SDA (SDA змінюється лише при SCL = 0)
//   фаза 1 : SCL = 1
//   фаза 2 : SCL = 1, біт стабільний (Slave його зчитує; у станах ACK Master читає SDA)
//   фаза 3 : SCL = 0, перехід до наступного біта / стану
//
// Лінія SDA - з відкритим стоком: Master або тягне її в '0', або відпускає ('Z').
// Рівень '1' формує зовнішній підтягувальний резистор (pull-up).
//==============================================================================
module i2c_master #(
    parameter CLK_FREQ = 50_000_000,   // системна частота, Гц
    parameter I2C_FREQ = 100_000       // частота шини SCL, Гц
)(
    input  wire       clk,        // системний такт 50 МГц
    input  wire       rst,        // синхронне скидання, активний рівень '1'
    input  wire       start,      // імпульс запуску передачі (приймається в стані IDLE)
    input  wire [6:0] addr,       // 7-бітна адреса Slave-пристрою
    input  wire [7:0] data_in,    // байт даних для передачі
    output reg        busy,       // '1' - модуль зайнятий передачею
    output reg        ack_error,  // '1' - Slave не підтвердив (NACK) адресу або дані
    output reg        scl,        // лінія тактування шини I2C
    inout  wire       sda         // двонапрямлена лінія даних I2C
);

    //--------------------------------------------------------------------------
    // Константи та коди станів автомата
    //--------------------------------------------------------------------------
    localparam integer TICK_DIV = CLK_FREQ / (I2C_FREQ * 4);   // = 125

    // Префікс S_ потрібен, щоб імена станів не збігалися з портами (start, addr)
    localparam [2:0] S_IDLE     = 3'd0,   // спокій: SCL = 1, SDA = 1
                     S_START    = 3'd1,   // умова START
                     S_ADDR     = 3'd2,   // передача 7 біт адреси + біт R/W
                     S_ACK_ADDR = 3'd3,   // підтвердження адреси від Slave
                     S_DATA     = 3'd4,   // передача 8 біт даних
                     S_ACK_DATA = 3'd5,   // підтвердження даних від Slave
                     S_STOP     = 3'd6;   // умова STOP

    //--------------------------------------------------------------------------
    // Внутрішні регістри
    //--------------------------------------------------------------------------
    reg [2:0] state;      // поточний стан FSM
    reg [1:0] phase;      // номер фази (0..3) всередині одного біта
    reg [7:0] div_cnt;    // лічильник-дільник тактової частоти (0 .. TICK_DIV-1)
    reg [2:0] bit_cnt;    // індекс біта, що передається (7 -> 0, старший біт першим)
    reg [7:0] tx_shift;   // байт, що передається: спочатку {addr, R/W=0}, потім data_in
    reg       sda_low;    // '1' - Master тягне SDA в '0'; '0' - SDA відпущено (Z)

    // Строб тактування: один імпульс кожні TICK_DIV тактів clk
    wire tick = (div_cnt == TICK_DIV - 1);

    // Тристабільний вихід SDA (відкритий стік): або '0', або високий імпеданс
    assign sda = sda_low ? 1'b0 : 1'bz;

    //--------------------------------------------------------------------------
    // Основний синхронний процес: дільник частоти + FSM
    //--------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            state     <= S_IDLE;
            phase     <= 2'd0;
            div_cnt   <= 8'd0;
            bit_cnt   <= 3'd7;
            tx_shift  <= 8'd0;
            sda_low   <= 1'b0;
            scl       <= 1'b1;
            busy      <= 1'b0;
            ack_error <= 1'b0;
        end else begin
            // Вільнобіжний лічильник-дільник (генерує строб tick)
            if (tick) div_cnt <= 8'd0;
            else      div_cnt <= div_cnt + 8'd1;

            case (state)
                //--------------------------------------------------------------
                // IDLE: шина вільна, чекаємо start
                //--------------------------------------------------------------
                S_IDLE: begin
                    scl     <= 1'b1;
                    sda_low <= 1'b0;
                    busy    <= 1'b0;
                    phase   <= 2'd0;
                    if (start) begin
                        tx_shift  <= {addr, 1'b0};   // 7 біт адреси + R/W = 0 (запис)
                        busy      <= 1'b1;
                        ack_error <= 1'b0;
                        div_cnt   <= 8'd0;           // синхронізуємо дільник зі стартом
                        state     <= S_START;
                    end
                end

                //--------------------------------------------------------------
                // START: SDA падає 1->0 при SCL = 1
                //--------------------------------------------------------------
                S_START: if (tick) begin
                    case (phase)
                        2'd0: begin scl <= 1'b1; sda_low <= 1'b0; phase <= 2'd1; end
                        2'd1: begin sda_low <= 1'b1;              phase <= 2'd2; end // START
                        2'd2: begin                               phase <= 2'd3; end
                        2'd3: begin
                            scl     <= 1'b0;
                            bit_cnt <= 3'd7;
                            phase   <= 2'd0;
                            state   <= S_ADDR;
                        end
                    endcase
                end

                //--------------------------------------------------------------
                // ADDR / DATA: побітова передача (старший біт першим)
                //--------------------------------------------------------------
                S_ADDR, S_DATA: if (tick) begin
                    case (phase)
                        2'd0: begin
                            scl     <= 1'b0;
                            sda_low <= ~tx_shift[bit_cnt];   // біт '0' -> тягнемо SDA вниз
                            phase   <= 2'd1;
                        end
                        2'd1: begin scl <= 1'b1; phase <= 2'd2; end
                        2'd2: begin              phase <= 2'd3; end
                        2'd3: begin
                            scl   <= 1'b0;
                            phase <= 2'd0;
                            if (bit_cnt == 3'd0)
                                state <= (state == S_ADDR) ? S_ACK_ADDR : S_ACK_DATA;
                            else
                                bit_cnt <= bit_cnt - 3'd1;
                        end
                    endcase
                end

                //--------------------------------------------------------------
                // ACK_ADDR / ACK_DATA: звільняємо SDA, Slave відповідає '0' = ACK
                //--------------------------------------------------------------
                S_ACK_ADDR, S_ACK_DATA: if (tick) begin
                    case (phase)
                        2'd0: begin
                            scl     <= 1'b0;
                            sda_low <= 1'b0;                 // відпускаємо SDA (Z)
                            phase   <= 2'd1;
                        end
                        2'd1: begin scl <= 1'b1; phase <= 2'd2; end
                        2'd2: begin
                            if (sda != 1'b0) ack_error <= 1'b1;   // немає '0' -> NACK
                            phase <= 2'd3;
                        end
                        2'd3: begin
                            scl   <= 1'b0;
                            phase <= 2'd0;
                            if (state == S_ACK_ADDR && !ack_error) begin
                                tx_shift <= data_in;         // адресу підтверджено -> шлемо дані
                                bit_cnt  <= 3'd7;
                                state    <= S_DATA;
                            end else begin
                                state    <= S_STOP;          // NACK або дані передано
                            end
                        end
                    endcase
                end

                //--------------------------------------------------------------
                // STOP: SDA росте 0->1 при SCL = 1
                //--------------------------------------------------------------
                S_STOP: if (tick) begin
                    case (phase)
                        2'd0: begin scl <= 1'b0; sda_low <= 1'b1; phase <= 2'd1; end
                        2'd1: begin scl <= 1'b1;                  phase <= 2'd2; end
                        2'd2: begin sda_low <= 1'b0;              phase <= 2'd3; end // STOP
                        2'd3: begin
                            phase <= 2'd0;
                            busy  <= 1'b0;
                            state <= S_IDLE;
                        end
                    endcase
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
