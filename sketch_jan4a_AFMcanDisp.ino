/**
 * @file sketch_jan4a_AFMcanDisp.ino
 * @brief AFM CAN Display - M5Stack Core2 + CAN Unit (U085)
 * @date 2026/01/04
 * 
 * CANバスからAFM信号を受信し、HUD風UIでリアルタイム表示
 * 
 * Hardware: M5Stack Core2 + CAN Unit (U085)
 * Libraries: M5Unified, M5GFX
 * CAN Driver: ESP32 TWAI (built-in)
 */

#include <M5Unified.h>
#include <M5GFX.h>
#include <driver/twai.h>
#include "can_signals.h"

// ============================================================================
// 設定定数
// ============================================================================
// M5Stack Core2 ポート設定
// PORT-A (赤): G32/G33 (I2C) - Grove端子が本体側面
// PORT-B (黒): G26/G36 (DAC/ADC)
// PORT-C (青): G13/G14 (UART)
// ※CAN Unitの接続ポートに合わせて変更してください

// PORT-A使用時 (本体側面のGrove端子)
#define CAN_TX_PIN          GPIO_NUM_32
#define CAN_RX_PIN          GPIO_NUM_33

// PORT-B使用時 (M5GO Bottom等)
// #define CAN_TX_PIN          GPIO_NUM_26
// #define CAN_RX_PIN          GPIO_NUM_36

// PORT-C使用時
// #define CAN_TX_PIN          GPIO_NUM_14
// #define CAN_RX_PIN          GPIO_NUM_13

#define SCREEN_WIDTH        320
#define SCREEN_HEIGHT       240

// UI更新間隔 (約30fps)
#define UI_UPDATE_INTERVAL_MS   33

// 信号タイムアウト (ms) - この時間受信がないと無効表示
#define SIGNAL_TIMEOUT_MS   3000

// デバッグ出力の有効/無効 (コメントアウトで無効化)
//#define DEBUG_CAN_OUTPUT

// ============================================================================
// HUD カラー定義 (RGB565)
// ============================================================================
#define HUD_COLOR_BG        TFT_BLACK
#define HUD_COLOR_PRIMARY   TFT_GREEN       // メイン表示色 (緑)
#define HUD_COLOR_CYAN      TFT_CYAN        // シアン
#define HUD_COLOR_WARNING   TFT_YELLOW      // 警告色
#define HUD_COLOR_ERROR     TFT_RED         // 異常色
#define HUD_COLOR_DIM       0x03E0          // 暗めの緑 (無効時)
#define HUD_COLOR_FRAME     0x07E0          // 枠線色

// ============================================================================
// グローバル変数
// ============================================================================
M5GFX display;
M5Canvas canvas(&display);  // ダブルバッファ用スプライト

CANSignals_t canSignals;

uint32_t lastUIUpdate = 0;
bool canInitialized = false;

// ============================================================================
// 関数プロトタイプ
// ============================================================================
void initCAN();
void initDisplay();
void processCANMessages();
void updateDisplay();
void drawHUDPanel(int x, int y, int w, int h, const char* label, 
                  float value, const char* unit, float minVal, float maxVal, 
                  bool valid, uint32_t lastUpdate);
void drawHUDFrame(int x, int y, int w, int h);
void drawBarGraph(int x, int y, int w, int h, float value, float minVal, float maxVal, bool valid);
uint16_t getValueColor(float value, float minVal, float maxVal, bool valid);

// ============================================================================
// セットアップ
// ============================================================================
void setup() {
    // M5Stack初期化
    auto cfg = M5.config();
    M5.begin(cfg);
    
    Serial.begin(115200);
    Serial.println("AFM CAN Display - Starting...");
    
    // CAN信号構造体初期化
    initCANSignals(&canSignals);
    
    // ディスプレイ初期化
    initDisplay();
    
    // CAN初期化
    initCAN();
    
    Serial.println("Initialization complete. Waiting for CAN data...");
}

// ============================================================================
// メインループ
// ============================================================================
void loop() {
    M5.update();
    
    // CAN受信処理
    processCANMessages();
    
    // UI更新 (30fps制限)
    uint32_t now = millis();
    if (now - lastUIUpdate >= UI_UPDATE_INTERVAL_MS) {
        lastUIUpdate = now;
        updateDisplay();
    }
}

// ============================================================================
// CAN初期化 (TWAI Driver)
// ============================================================================
void initCAN() {
    // TWAI設定 - 500kbps
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    g_config.rx_queue_len = 10;
    g_config.tx_queue_len = 5;
    
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    
    // TWAIドライバのインストール
    esp_err_t result = twai_driver_install(&g_config, &t_config, &f_config);
    if (result == ESP_OK) {
        Serial.println("TWAI driver installed");
    } else {
        Serial.printf("TWAI driver install failed: %d\n", result);
        return;
    }
    
    // TWAIドライバの開始
    result = twai_start();
    if (result == ESP_OK) {
        Serial.println("TWAI driver started");
        canInitialized = true;
    } else {
        Serial.printf("TWAI driver start failed: %d\n", result);
    }
    
    Serial.println("CAN initialized: 500kbps");
}

// ============================================================================
// ディスプレイ初期化
// ============================================================================
void initDisplay() {
    display.begin();
    display.setRotation(1);  // 横向き
    display.fillScreen(HUD_COLOR_BG);
    
    // ダブルバッファ用キャンバス作成
    canvas.createSprite(SCREEN_WIDTH, SCREEN_HEIGHT);
    canvas.setTextDatum(MC_DATUM);  // 中央揃え
    
    Serial.println("Display initialized");
}

// ============================================================================
// CAN受信処理 (TWAI Driver)
// ============================================================================
void processCANMessages() {
    if (!canInitialized) {
        return;
    }
    
    twai_message_t rx_message;
    
    // キューにあるメッセージを全て処理（最大10件/ループ）
    int processedCount = 0;
    const int maxProcessPerLoop = 10;
    
    while (processedCount < maxProcessPerLoop) {
        // ノンブロッキングで取得（タイムアウト0）
        esp_err_t result = twai_receive(&rx_message, 0);
        
        if (result != ESP_OK) {
            // メッセージなし or エラー
            if (result != ESP_ERR_TIMEOUT) {
#ifdef DEBUG_CAN_OUTPUT
                static uint32_t lastErrPrint = 0;
                if (millis() - lastErrPrint > 5000) {
                    Serial.printf("TWAI receive error: %d\n", result);
                    twai_status_info_t status;
                    if (twai_get_status_info(&status) == ESP_OK) {
                        Serial.printf("  State: %d, TX_err: %d, RX_err: %d\n",
                                      status.state, status.tx_error_counter, status.rx_error_counter);
                    }
                    lastErrPrint = millis();
                }
#endif
            }
            break;  // キューが空になったらループ終了
        }
        
        processedCount++;

#ifdef DEBUG_CAN_OUTPUT
        // デバッグ出力
        Serial.printf("CAN RX: ID=0x%03X DLC=%d Data=", 
                      rx_message.identifier, rx_message.data_length_code);
        for (int i = 0; i < rx_message.data_length_code; i++) {
            Serial.printf("%02X ", rx_message.data[i]);
        }
        Serial.println();
#endif
        
        // Extended Frame / RTR は除外
        if (rx_message.extd || rx_message.rtr) {
            continue;
        }
        
        switch (rx_message.identifier) {
            case CAN_ID_AFMCONV1:
                decodeAFMConv1(rx_message.data, &canSignals);
#ifdef DEBUG_CAN_OUTPUT
                Serial.println("  -> AFMConv1 decoded");
#endif
                break;
                
            case CAN_ID_AFMCONV2:
                decodeAFMconv2(rx_message.data, &canSignals);
#ifdef DEBUG_CAN_OUTPUT
                Serial.println("  -> AFMconv2 decoded");
#endif
                break;
                
            default:
                // 未知のメッセージID
                break;
        }
    }
}

// ============================================================================
// ディスプレイ更新
// ============================================================================
void updateDisplay() {
    uint32_t now = millis();
    
    // 背景クリア
    canvas.fillSprite(HUD_COLOR_BG);
    
    // タイトル
    canvas.setTextColor(HUD_COLOR_CYAN);
    canvas.setFont(&fonts::Font2);
    canvas.setTextDatum(TC_DATUM);
    canvas.drawString("AFM CAN DISPLAY", SCREEN_WIDTH / 2, 2);
    
    // 水平線（タイトル下）
    canvas.drawFastHLine(10, 18, SCREEN_WIDTH - 20, HUD_COLOR_DIM);
    
    // パネルサイズ計算 (2x2グリッド)
    const int panelW = 155;
    const int panelH = 100;
    const int startX = 5;
    const int startY = 24;
    const int gapX = 5;
    const int gapY = 8;
    
    // 信号有効性チェック（タイムアウト判定）
    bool afmFreqValid = canSignals.afm_freq_valid && 
                        (now - canSignals.last_update_afmconv1 < SIGNAL_TIMEOUT_MS);
    bool thaValid = canSignals.tha_valid && 
                    (now - canSignals.last_update_afmconv1 < SIGNAL_TIMEOUT_MS);
    bool rawGaValid = canSignals.raw_ga_valid && 
                      (now - canSignals.last_update_afmconv1 < SIGNAL_TIMEOUT_MS);
    bool mcuTempValid = canSignals.mcu_temp_valid && 
                        (now - canSignals.last_update_afmconv2 < SIGNAL_TIMEOUT_MS);
    
    // 左上: AFM周波数
    drawHUDPanel(startX, startY, panelW, panelH,
                 "AFM FREQ", canSignals.afm_freq, "Hz",
                 AFM_FREQ_MIN, AFM_FREQ_MAX, afmFreqValid,
                 canSignals.last_update_afmconv1);
    
    // 右上: 空気量 (Raw)
    drawHUDPanel(startX + panelW + gapX, startY, panelW, panelH,
                 "AIR FLOW", canSignals.raw_ga, "g/s",
                 RAW_GA_MIN, RAW_GA_MAX, rawGaValid,
                 canSignals.last_update_afmconv1);
    
    // 左下: 吸気温度
    drawHUDPanel(startX, startY + panelH + gapY, panelW, panelH,
                 "INTAKE TEMP", canSignals.tha, "C",
                 THA_MIN, THA_MAX, thaValid,
                 canSignals.last_update_afmconv1);
    
    // 右下: MCU温度
    drawHUDPanel(startX + panelW + gapX, startY + panelH + gapY, panelW, panelH,
                 "MCU TEMP", canSignals.mcu_temp, "C",
                 MCUTEMP_MIN, MCUTEMP_MAX, mcuTempValid,
                 canSignals.last_update_afmconv2);
    
    // スプライトをディスプレイに転送
    canvas.pushSprite(0, 0);
}

// ============================================================================
// HUDパネル描画
// ============================================================================
void drawHUDPanel(int x, int y, int w, int h, const char* label,
                  float value, const char* unit, float minVal, float maxVal,
                  bool valid, uint32_t lastUpdate) {
    
    // HUD風フレーム描画
    drawHUDFrame(x, y, w, h);
    
    // ラベル
    canvas.setFont(&fonts::Font2);
    canvas.setTextDatum(TL_DATUM);
    canvas.setTextColor(valid ? HUD_COLOR_CYAN : HUD_COLOR_DIM);
    canvas.drawString(label, x + 5, y + 3);
    
    // 値の色を決定
    uint16_t valueColor = getValueColor(value, minVal, maxVal, valid);
    
    // メイン数値表示
    canvas.setFont(&fonts::Font7);
    canvas.setTextDatum(MC_DATUM);
    canvas.setTextColor(valueColor);
    
    char valueStr[16];
    if (valid) {
        // 整数か小数かで表示形式を変更
        if (value >= 1000) {
            snprintf(valueStr, sizeof(valueStr), "%.0f", value);
        } else if (value >= 100) {
            snprintf(valueStr, sizeof(valueStr), "%.1f", value);
        } else {
            snprintf(valueStr, sizeof(valueStr), "%.1f", value);
        }
    } else {
        strcpy(valueStr, "----");
    }
    canvas.drawString(valueStr, x + w/2, y + h/2 - 5);
    
    // 単位表示
    canvas.setFont(&fonts::Font2);
    canvas.setTextDatum(BR_DATUM);
    canvas.setTextColor(valid ? HUD_COLOR_PRIMARY : HUD_COLOR_DIM);
    canvas.drawString(unit, x + w - 5, y + h - 18);
    
    // バーグラフ
    drawBarGraph(x + 5, y + h - 15, w - 10, 10, value, minVal, maxVal, valid);
}

// ============================================================================
// HUD風フレーム描画
// ============================================================================
void drawHUDFrame(int x, int y, int w, int h) {
    uint16_t frameColor = HUD_COLOR_DIM;
    
    // コーナーサイズ
    const int corner = 8;
    
    // 四隅のL字型コーナー
    // 左上
    canvas.drawFastHLine(x, y, corner, frameColor);
    canvas.drawFastVLine(x, y, corner, frameColor);
    
    // 右上
    canvas.drawFastHLine(x + w - corner, y, corner, frameColor);
    canvas.drawFastVLine(x + w - 1, y, corner, frameColor);
    
    // 左下
    canvas.drawFastHLine(x, y + h - 1, corner, frameColor);
    canvas.drawFastVLine(x, y + h - corner, corner, frameColor);
    
    // 右下
    canvas.drawFastHLine(x + w - corner, y + h - 1, corner, frameColor);
    canvas.drawFastVLine(x + w - 1, y + h - corner, corner, frameColor);
}

// ============================================================================
// バーグラフ描画
// ============================================================================
void drawBarGraph(int x, int y, int w, int h, float value, float minVal, float maxVal, bool valid) {
    // 枠線
    canvas.drawRect(x, y, w, h, HUD_COLOR_DIM);
    
    if (!valid) {
        return;
    }
    
    // 値を0-100%に正規化
    float normalized = (value - minVal) / (maxVal - minVal);
    normalized = constrain(normalized, 0.0f, 1.0f);
    
    // バーの幅を計算
    int barWidth = (int)(normalized * (w - 2));
    
    // 色を決定
    uint16_t barColor = HUD_COLOR_PRIMARY;
    if (normalized > 0.9f) {
        barColor = HUD_COLOR_ERROR;
    } else if (normalized > 0.75f) {
        barColor = HUD_COLOR_WARNING;
    }
    
    // バーを描画
    if (barWidth > 0) {
        canvas.fillRect(x + 1, y + 1, barWidth, h - 2, barColor);
    }
}

// ============================================================================
// 値に応じた色を取得
// ============================================================================
uint16_t getValueColor(float value, float minVal, float maxVal, bool valid) {
    if (!valid) {
        return HUD_COLOR_DIM;
    }
    
    // 正規化
    float normalized = (value - minVal) / (maxVal - minVal);
    
    // 範囲外チェック
    if (normalized < 0.0f || normalized > 1.0f) {
        return HUD_COLOR_ERROR;
    }
    
    // 高い値は警告色
    if (normalized > 0.9f) {
        return HUD_COLOR_ERROR;
    } else if (normalized > 0.75f) {
        return HUD_COLOR_WARNING;
    }
    
    return HUD_COLOR_PRIMARY;
}
