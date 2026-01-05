/**
 * @file can_signals.h
 * @brief CAN信号定義ヘッダ - mx5_afmconv.dbc より生成
 * @date 2026/01/04
 */

#ifndef CAN_SIGNALS_H
#define CAN_SIGNALS_H

#include <stdint.h>

// ============================================================================
// CAN メッセージID定義
// ============================================================================
#define CAN_ID_AFMCONV1     0x001   // AFM主要データ
#define CAN_ID_AFMCONV2     0x002   // 補助データ

// ============================================================================
// シグナル スケーリング定義
// ============================================================================

// AFM_Freq: Factor=1, Offset=0
#define AFM_FREQ_FACTOR     1.0f
#define AFM_FREQ_OFFSET     0.0f
#define AFM_FREQ_MIN        1500.0f
#define AFM_FREQ_MAX        20000.0f
#define AFM_FREQ_UNIT       "Hz"

// THA: Factor=0.1, Offset=-30
#define THA_FACTOR          0.1f
#define THA_OFFSET          (-30.0f)
#define THA_MIN             (-20.0f)
#define THA_MAX             120.0f
#define THA_UNIT            "C"

// raw_Ga: Factor=0.1, Offset=0
#define RAW_GA_FACTOR       0.1f
#define RAW_GA_OFFSET       0.0f
#define RAW_GA_MIN          0.0f
#define RAW_GA_MAX          200.0f
#define RAW_GA_UNIT         "g/s"

// MCUtemp: Factor=0.1, Offset=-30
#define MCUTEMP_FACTOR      0.1f
#define MCUTEMP_OFFSET      (-30.0f)
#define MCUTEMP_MIN         (-20.0f)
#define MCUTEMP_MAX         120.0f
#define MCUTEMP_UNIT        "C"

// ============================================================================
// シグナルデータ構造体
// ============================================================================
typedef struct {
    float afm_freq;     // AFM出力周波数 [Hz]
    float tha;          // 吸気温度 [℃]
    float raw_ga;       // 空気量(Raw) [g/s]
    float mcu_temp;     // MCU温度 [℃]
    
    bool afm_freq_valid;
    bool tha_valid;
    bool raw_ga_valid;
    bool mcu_temp_valid;
    
    uint32_t last_update_afmconv1;  // 最終受信時刻 AFMConv1
    uint32_t last_update_afmconv2;  // 最終受信時刻 AFMconv2
} CANSignals_t;

// ============================================================================
// Big Endian 16bit Signed デコード関数
// バイト配置: [MSB][LSB] (Motorola byte order)
// ============================================================================
inline int16_t decodeSignedBE16(const uint8_t* data, int byteOffset) {
    return (int16_t)((data[byteOffset] << 8) | data[byteOffset + 1]);
}

// ============================================================================
// シグナルデコード関数
// ============================================================================

/**
 * @brief AFMConv1メッセージをデコード
 * @param data CANフレームデータ (8バイト)
 * @param signals デコード結果格納先
 */
inline void decodeAFMConv1(const uint8_t* data, CANSignals_t* signals) {
    // AFM_Freq: Byte[0:1], Factor=1, Offset=0
    int16_t raw_afm_freq = decodeSignedBE16(data, 0);
    signals->afm_freq = (float)raw_afm_freq * AFM_FREQ_FACTOR + AFM_FREQ_OFFSET;
    signals->afm_freq_valid = true;
    
    // THA: Byte[2:3], Factor=0.1, Offset=-30
    int16_t raw_tha = decodeSignedBE16(data, 2);
    signals->tha = (float)raw_tha * THA_FACTOR + THA_OFFSET;
    signals->tha_valid = true;
    
    // raw_Ga: Byte[4:5], Factor=0.1, Offset=0
    int16_t raw_raw_ga = decodeSignedBE16(data, 4);
    signals->raw_ga = (float)raw_raw_ga * RAW_GA_FACTOR + RAW_GA_OFFSET;
    signals->raw_ga_valid = true;
    
    // comp_Ga: Byte[6:7] - 表示対象外のためスキップ
    
    signals->last_update_afmconv1 = millis();
}

/**
 * @brief AFMconv2メッセージをデコード
 * @param data CANフレームデータ (8バイト)
 * @param signals デコード結果格納先
 */
inline void decodeAFMconv2(const uint8_t* data, CANSignals_t* signals) {
    // THA_comp: Byte[0:1] - 表示対象外のためスキップ
    // VR1_comp: Byte[2:3] - 表示対象外のためスキップ
    // AFMoutV:  Byte[4:5] - 表示対象外のためスキップ
    
    // MCUtemp: Byte[6:7], Factor=0.1, Offset=-30
    int16_t raw_mcu_temp = decodeSignedBE16(data, 6);
    signals->mcu_temp = (float)raw_mcu_temp * MCUTEMP_FACTOR + MCUTEMP_OFFSET;
    signals->mcu_temp_valid = true;
    
    signals->last_update_afmconv2 = millis();
}

/**
 * @brief シグナル構造体の初期化
 * @param signals 初期化対象
 */
inline void initCANSignals(CANSignals_t* signals) {
    signals->afm_freq = 0.0f;
    signals->tha = 0.0f;
    signals->raw_ga = 0.0f;
    signals->mcu_temp = 0.0f;
    
    signals->afm_freq_valid = false;
    signals->tha_valid = false;
    signals->raw_ga_valid = false;
    signals->mcu_temp_valid = false;
    
    signals->last_update_afmconv1 = 0;
    signals->last_update_afmconv2 = 0;
}

#endif // CAN_SIGNALS_H
