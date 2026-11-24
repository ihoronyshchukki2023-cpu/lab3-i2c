`timescale 1ns / 1ps
//==============================================================================
// Тестбенч : tb_i2c_master
// Перевіряє: модуль i2c_master (I2C Master, режим запису)
//
// Тест 1: Slave з адресою 0x50 підтверджує адресу, Master передає дані 0x35.
//         Очікується: адреса 0x50 + R/W=0, дані 0x35, ack_error = 0.
// Тест 2: Master звертається до адреси 0x51, якої на шині немає (NACK).
//         Очікується: ack_error = 1, дані не передаються, шина звільняється (STOP).
// Додатково перевіряється період SCL = 10 мкс (100 кГц).
//==============================================================================
module tb_i2c_master;

    //--------------------------------------------------------------------------
    // Параметри та сигнали
    //--------------------------------------------------------------------------
    localparam [6:0] SLAVE_ADDR = 7'h50;   // адреса віртуального Slave-пристрою

    reg        clk       = 1'b0;           // системний такт 50 МГц
    reg        rst       = 1'b1;           // скидання (активний рівень '1')
    reg        start     = 1'b0;           // запуск передачі
    reg  [6:0] addr      = 7'h50;          // адреса, яку передає Master
    reg  [7:0] data_in   = 8'h35;          // дані, які передає Master
    wire       busy;
    wire       ack_error;
    wire       scl;
    tri1       sda;                        // tri1 = лінія з вбудованим pull-up ('1' коли ніхто не тягне)

    //--------------------------------------------------------------------------
    // Перевіряємий модуль (UUT)
    //--------------------------------------------------------------------------
    i2c_master #(
        .CLK_FREQ (50_000_000),
        .I2C_FREQ (100_000)
    ) uut (
        .clk       (clk),
        .rst       (rst),
        .start     (start),
        .addr      (addr),
        .data_in   (data_in),
        .busy      (busy),
        .ack_error (ack_error),
        .scl       (scl),
        .sda       (sda)
    );

    // Генератор тактового сигналу: період 20 нс (50 МГц)
    always #10 clk = ~clk;

    //--------------------------------------------------------------------------
    // Модель Slave-пристрою (приймач): розпізнає START/STOP, приймає байти,
    // відповідає ACK на 9-му такті, якщо адреса збігається з SLAVE_ADDR
    //--------------------------------------------------------------------------
    reg       slave_sda_low = 1'b0;        // '1' - Slave тягне SDA в '0'
    reg       rx_active     = 1'b0;        // 1 між START і STOP
    reg       rx_is_addr    = 1'b1;        // 1 - зараз приймається байт адреси
    reg [3:0] rx_cnt        = 4'd0;        // лічильник прийнятих бітів (0..8), 9 = такт ACK
    reg [7:0] rx_shift      = 8'd0;        // зсувний регістр приймача
    reg       ack_now       = 1'b0;

    reg [7:0] rx_addr_byte  = 8'd0;        // прийнятий байт адреси (7 біт адреси + R/W)
    reg [7:0] rx_data_byte  = 8'd0;        // прийнятий байт даних
    integer   data_bytes_rx = 0;           // скільки байтів даних прийнято

    assign sda = slave_sda_low ? 1'b0 : 1'bz;

    // Умова START: SDA падає при SCL = 1
    always @(negedge sda)
        if (scl === 1'b1) begin
            rx_active  <= 1'b1;
            rx_cnt     <= 4'd0;
            rx_is_addr <= 1'b1;
        end

    // Умова STOP: SDA росте при SCL = 1
    always @(posedge sda)
        if (scl === 1'b1) rx_active <= 1'b0;

    // Прийом біта по фронту SCL (старший біт першим)
    always @(posedge scl)
        if (rx_active && rx_cnt < 4'd8) begin
            rx_shift <= {rx_shift[6:0], sda};
            rx_cnt   <= rx_cnt + 4'd1;
        end

    // По спаду SCL: після 8-го біта виставляємо ACK, після 9-го такту - звільняємо SDA
    always @(negedge scl)
        if (rx_active) begin
            if (rx_cnt == 4'd8) begin
                if (rx_is_addr) begin
                    rx_addr_byte <= rx_shift;
                    ack_now = (rx_shift[7:1] == SLAVE_ADDR);   // ACK лише для своєї адреси
                end else begin
                    rx_data_byte  <= rx_shift;
                    data_bytes_rx <= data_bytes_rx + 1;
                    ack_now = 1'b1;
                end
                slave_sda_low <= ack_now;
                rx_cnt        <= 4'd9;
            end else if (rx_cnt == 4'd9) begin
                slave_sda_low <= 1'b0;
                rx_cnt        <= 4'd0;
                rx_is_addr    <= 1'b0;                         // далі приймаємо байти даних
            end
        end

    //--------------------------------------------------------------------------
    // Контроль періоду SCL (має бути 10 мкс = 10000 нс)
    //--------------------------------------------------------------------------
    realtime t_last        = 0.0;
    realtime period        = 0.0;
    integer  scl_edges     = 0;
    integer  period_errors = 0;

    always @(posedge busy) scl_edges = 0;     // нова передача - починаємо рахунок заново

    always @(posedge scl)
        if (busy) begin
            if (scl_edges > 0) begin
                period = $realtime - t_last;
                if (period < 9999.0 || period > 10001.0) begin
                    period_errors = period_errors + 1;
                    $display("[%0t] ERROR: SCL period = %0f ns", $time, period);
                end
            end
            t_last    = $realtime;
            scl_edges = scl_edges + 1;
        end

    //--------------------------------------------------------------------------
    // Задача: одна сесія передачі (імпульс start 40 нс і очікування завершення)
    //--------------------------------------------------------------------------
    task do_transfer;
        begin
            @(negedge clk);
            start = 1'b1;
            repeat (2) @(negedge clk);        // start тримається 40 нс
            start = 1'b0;
            wait (busy == 1'b1);              // чекаємо початку обміну
            wait (busy == 1'b0);              // чекаємо завершення (після STOP)
            #1000;                            // невелика пауза для наочності осцилограм
        end
    endtask

    //--------------------------------------------------------------------------
    // Сценарій тестування
    //--------------------------------------------------------------------------
    integer errors = 0;

    initial begin
        $timeformat(-9, 3, " ns", 0);          // час у консолі - в наносекундах

        // Скидання
        rst = 1'b1;
        #100;
        rst = 1'b0;
        #100;

        //----- Тест 1: коректна передача 0x50 / 0x35 -----
        $display("[%0t] Test 1: addr 0x50, data 0x35 (ACK expected)", $time);
        addr    = 7'h50;
        data_in = 8'h35;
        do_transfer;

        if (rx_addr_byte !== {SLAVE_ADDR, 1'b0}) begin
            errors = errors + 1;
            $display("ERROR: received address byte = %b, expected %b", rx_addr_byte, {SLAVE_ADDR, 1'b0});
        end
        if (rx_data_byte !== 8'h35) begin
            errors = errors + 1;
            $display("ERROR: received data = 0x%h, expected 0x35", rx_data_byte);
        end
        if (ack_error !== 1'b0) begin
            errors = errors + 1;
            $display("ERROR: ack_error = %b, expected 0", ack_error);
        end
        if (errors == 0)
            $display("[%0t] Test 1 PASSED: slave received addr 0x%h (R/W=%b) and data 0x%h",
                     $time, rx_addr_byte[7:1], rx_addr_byte[0], rx_data_byte);

        //----- Тест 2: адреса 0x51 - Slave не відповідає (NACK) -----
        $display("[%0t] Test 2: addr 0x51 (NACK and ack_error = 1 expected)", $time);
        addr    = 7'h51;
        data_in = 8'hAA;
        do_transfer;

        if (ack_error !== 1'b1) begin
            errors = errors + 1;
            $display("ERROR: ack_error = %b, expected 1", ack_error);
        end
        if (data_bytes_rx !== 1) begin
            errors = errors + 1;
            $display("ERROR: data must not be sent after NACK (data bytes received: %0d)", data_bytes_rx);
        end
        if (ack_error === 1'b1 && data_bytes_rx === 1)
            $display("[%0t] Test 2 PASSED: NACK detected, no data sent", $time);

        //----- Підсумок -----
        errors = errors + period_errors;
        if (errors == 0)
            $display("=== ALL TESTS PASSED (SCL period = 10 us / 100 kHz) ===");
        else
            $display("=== TESTS FAILED, errors: %0d ===", errors);

        $finish;
    end

    // Захист від зависання симуляції
    initial begin
        #2_000_000;                           // 2 мс
        $display("ERROR: simulation timeout");
        $finish;
    end

endmodule
