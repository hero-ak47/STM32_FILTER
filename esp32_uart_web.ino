#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

// =====================================================
// WIFI
// =====================================================

const char* ssid     = "B13 309A_4G";
const char* password = "hust@b13";


// =====================================================
// SERVER
// =====================================================

WebServer server(80);
WebSocketsServer webSocket(81);


// =====================================================
// UART STM32
//
// STM32 PA9  TX  --->  ESP32 GPIO16 RX
// STM32 GND      --->  ESP32 GND
//
// GPIO17 TX của ESP32 không cần dùng nếu chỉ nhận dữ liệu.
// =====================================================

#define STM_BAUD 460800

#define STM_RX_PIN 16
#define STM_TX_PIN 17


// =====================================================
// FRAME
//
// Header:
//      AA 55
//
// Data:
//      64 sample
//
// Mỗi sample:
//      Raw      = int16 = 2 byte
//      Filtered = int16 = 2 byte
//
// Tổng:
//      2 + 64 * 4 = 258 byte
// =====================================================

#define FRAME_SAMPLES 64
#define FRAME_BYTES   (2 + FRAME_SAMPLES * 4)


// =====================================================
// UART RECEIVE BUFFER
// =====================================================

uint8_t rxTemp[FRAME_BYTES];

size_t rxCount = 0;


// =====================================================
// HTML
// =====================================================

const char index_html[] PROGMEM = R"HTML(

<!DOCTYPE html>

<html>

<head>

<meta charset="utf-8">

<title>STM32 ADC Waveform</title>

<script src="https://cdnjs.cloudflare.com/ajax/libs/Chart.js/3.9.1/chart.min.js"></script>

<style>

body {
    font-family: sans-serif;
    background: #111;
    color: #eee;
    text-align: center;
    margin: 0;
    padding: 8px;
}

h2 {
    margin: 15px;
}

canvas {
    background: #1e1e1e;
    border-radius: 8px;
    margin-top: 10px;
}

#status {
    font-size: 18px;
    margin-top: 15px;
}

</style>

</head>


<body>


<h2>STM32 ADC Waveform (Raw vs Filtered)</h2>


<canvas
    id="chartRaw"
    width="1100"
    height="250">
</canvas>


<canvas
    id="chartFilt"
    width="1100"
    height="250">
</canvas>


<p id="status">
    Đang kết nối...
</p>


<script>


// =====================================================
// CONFIG
// =====================================================

const MAX_POINTS = 500;

const FRAME_SAMPLES = 64;

const FRAME_BYTES =
    2 + FRAME_SAMPLES * 4;


// =====================================================
// DATA
// =====================================================

let rawData = [];

let filtData = [];

let labels = [];

let sampleIndex = 0;

let pendingUpdate = false;


// =====================================================
// RAW CHART
// =====================================================

const ctxRaw =
    document
    .getElementById('chartRaw')
    .getContext('2d');


const chartRaw = new Chart(
    ctxRaw,
    {

        type: 'line',

        data:
        {
            labels: labels,

            datasets:
            [
                {
                    label: 'Raw',

                    data: rawData,

                    borderColor: '#e74c3c',

                    borderWidth: 2,

                    pointRadius: 0,

                    tension: 0
                }
            ]
        },


        options:
        {
            animation: false,

            responsive: false,

            scales:
            {
                x:
                {
                    display: false
                },

                y:
                {
                    min : 1900,
                    max : 2200
                }
            }
        }

    }
);


// =====================================================
// FILTERED CHART
// =====================================================

const ctxFilt =
    document
    .getElementById('chartFilt')
    .getContext('2d');


const chartFilt = new Chart(
    ctxFilt,
    {

        type: 'line',

        data:
        {
            labels: labels,

            datasets:
            [
                {
                    label: 'Filtered (LPF)',

                    data: filtData,

                    borderColor: '#2ecc71',

                    borderWidth: 2,

                    pointRadius: 0,

                    tension: 0
                }
            ]
        },


        options:
        {
            animation: false,

            responsive: false,

            scales:
            {
                x:
                {
                    display: false
                },

                y:
                {
                    min: 1900,
                    max: 2200
                }
            }
        }

    }
);


// =====================================================
// WEBSOCKET
// =====================================================

const ws =
    new WebSocket(
        'ws://' + location.hostname + ':81/'
    );


ws.binaryType = 'arraybuffer';


// =====================================================
// WEBSOCKET CONNECTED
// =====================================================

ws.onopen = function()
{

    document
        .getElementById('status')
        .innerText =
        "Đã kết nối, đang nhận dữ liệu...";

};


// =====================================================
// WEBSOCKET CLOSED
// =====================================================

ws.onclose = function()
{

    document
        .getElementById('status')
        .innerText =
        "Mất kết nối WebSocket.";

};


// =====================================================
// WEBSOCKET ERROR
// =====================================================

ws.onerror = function()
{

    document
        .getElementById('status')
        .innerText =
        "Lỗi WebSocket.";

};


// =====================================================
// RECEIVE FRAME
// =====================================================

ws.onmessage = function(evt)
{

    // -------------------------------------------------
    // Chuyển ArrayBuffer thành Uint8Array
    // -------------------------------------------------

    const data =
        new Uint8Array(evt.data);


    // -------------------------------------------------
    // DEBUG
    // -------------------------------------------------

    console.log(
        "WS frame:",
        data.length,
        "bytes"
    );


    // -------------------------------------------------
    // Kiểm tra kích thước
    // -------------------------------------------------

    if (data.length !== FRAME_BYTES)
    {

        console.log(
            "Sai kích thước frame:",
            data.length,
            "byte"
        );

        return;
    }


    // -------------------------------------------------
    // Kiểm tra Header
    // -------------------------------------------------

    if (
        data[0] !== 0xAA ||
        data[1] !== 0x55
    )
    {

        console.log(
            "SAI HEADER:",
            data[0].toString(16),
            data[1].toString(16)
        );

        return;
    }


    // -------------------------------------------------
    // Data bắt đầu từ byte thứ 2
    // -------------------------------------------------

    const view =
        new DataView(
            data.buffer,
            data.byteOffset + 2,
            FRAME_SAMPLES * 4
        );


    // -------------------------------------------------
    // Đọc 64 sample
    // -------------------------------------------------

    for (
        let i = 0;
        i < FRAME_SAMPLES;
        i++
    )
    {

        // ---------------------------------------------
        // Raw
        // ---------------------------------------------

        const raw =
            view.getInt16(
                i * 4,
                true
            );


        // ---------------------------------------------
        // Filtered
        // ---------------------------------------------

        const filt =
            view.getInt16(
                i * 4 + 2,
                true
            );


        // ---------------------------------------------
        // Lưu dữ liệu
        // ---------------------------------------------

        rawData.push(raw);

        filtData.push(filt);

        labels.push(sampleIndex++);


        // ---------------------------------------------
        // Giới hạn 500 điểm
        // ---------------------------------------------

        if (
            rawData.length > MAX_POINTS
        )
        {

            rawData.shift();

            filtData.shift();

            labels.shift();

        }

    }


    // =================================================
    // UPDATE CHART
    // =================================================

    if (!pendingUpdate)
    {

        pendingUpdate = true;


        requestAnimationFrame(
            function()
            {

                chartRaw.update();

                chartFilt.update();

                pendingUpdate = false;

            }
        );

    }

};


</script>


</body>

</html>

)HTML";


// =====================================================
// HTTP ROOT
// =====================================================

void handleRoot()
{
    server.send_P(
        200,
        "text/html",
        index_html
    );
}


// =====================================================
// WEBSOCKET EVENT
// =====================================================

void webSocketEvent(
    uint8_t num,
    WStype_t type,
    uint8_t *payload,
    size_t length
)
{

    if (type == WStype_CONNECTED)
    {

        Serial.printf(
            "Browser #%u da ket noi WebSocket\n",
            num
        );

    }


    else if (type == WStype_DISCONNECTED)
    {

        Serial.printf(
            "Browser #%u ngat ket noi WebSocket\n",
            num
        );

    }

}


// =====================================================
// SETUP
// =====================================================

void setup()
{

    // =================================================
    // SERIAL DEBUG
    // =================================================

    Serial.begin(115200);

    delay(500);

    Serial.println();

    Serial.println(
        "======================================"
    );

    Serial.println(
        "ESP32 STM32 ADC RECEIVER"
    );

    Serial.println(
        "======================================"
    );


    // =================================================
    // UART2
    // =================================================

    Serial2.begin(
        STM_BAUD,
        SERIAL_8N1,
        STM_RX_PIN,
        STM_TX_PIN
    );


    Serial.println(
        "UART2 da khoi dong"
    );

    Serial.printf(
        "Baud = %d\n",
        STM_BAUD
    );

    Serial.printf(
        "RX = GPIO%d\n",
        STM_RX_PIN
    );

    Serial.printf(
        "TX = GPIO%d\n",
        STM_TX_PIN
    );


    // =================================================
    // WIFI
    // =================================================

    WiFi.begin(
        ssid,
        password
    );


    Serial.print(
        "Dang ket noi WiFi"
    );


    while (
        WiFi.status() != WL_CONNECTED
    )
    {

        delay(300);

        Serial.print(".");

    }


    Serial.println();

    Serial.println(
        "WiFi connected"
    );


    Serial.print(
        "IP address: "
    );

    Serial.println(
        WiFi.localIP()
    );


    // =================================================
    // HTTP SERVER
    // =================================================

    server.on(
        "/",
        handleRoot
    );


    server.begin();


    Serial.println(
        "HTTP server started"
    );


    // =================================================
    // WEBSOCKET SERVER
    // =================================================

    webSocket.begin();


    webSocket.onEvent(
        webSocketEvent
    );


    Serial.println(
        "WebSocket server started"
    );


    Serial.println(
        "======================================"
    );

}


// =====================================================
// LOOP
// =====================================================

void loop()
{

    // =================================================
    // XỬ LÝ HTTP
    // =================================================

    server.handleClient();


    // =================================================
    // XỬ LÝ WEBSOCKET
    // =================================================

    webSocket.loop();


    // =================================================
    // NHẬN UART STM32
    // =================================================

    while (
        Serial2.available()
    )
    {

        uint8_t b =
            Serial2.read();


        // =============================================
        // STATE 0
        //
        // Chờ byte AA
        // =============================================

        if (rxCount == 0)
        {

            if (b == 0xAA)
            {

                rxTemp[0] = 0xAA;

                rxCount = 1;

            }

            continue;
        }


        // =============================================
        // STATE 1
        //
        // Đã nhận AA
        // Chờ 55
        // =============================================

        if (rxCount == 1)
        {

            if (b == 0x55)
            {

                rxTemp[1] = 0x55;

                rxCount = 2;

            }

            else
            {

                // -------------------------------------
                // Header sai
                // -------------------------------------

                rxCount = 0;


                // -------------------------------------
                // Nếu byte hiện tại vẫn là AA
                // thì dùng nó làm byte đầu header
                // mới
                // -------------------------------------

                if (b == 0xAA)
                {

                    rxTemp[0] = 0xAA;

                    rxCount = 1;

                }

            }

            continue;
        }


        // =============================================
        // STATE 2
        //
        // Đã nhận AA 55
        // Nhận data
        // =============================================

        rxTemp[rxCount++] = b;


        // =============================================
        // ĐỦ FRAME
        // =============================================

        if (rxCount == FRAME_BYTES)
        {

            // -----------------------------------------
            // Debug
            // -----------------------------------------

            static uint32_t frameCount = 0;

            frameCount++;


            Serial.printf(
                "FRAME %lu received - %u bytes\n",
                frameCount,
                FRAME_BYTES
            );


            // -----------------------------------------
            // In một số byte đầu
            // -----------------------------------------

            Serial.printf(
                "HEADER: %02X %02X\n",
                rxTemp[0],
                rxTemp[1]
            );


            // -----------------------------------------
            // Gửi frame qua WebSocket
            // -----------------------------------------

            webSocket.broadcastBIN(
                rxTemp,
                FRAME_BYTES
            );


            // -----------------------------------------
            // Reset
            // -----------------------------------------

            rxCount = 0;

        }

    }

}
