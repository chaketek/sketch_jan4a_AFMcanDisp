/**
 * @file kalman_filter.h
 * @brief 1次カルマンフィルタ実装
 * @date 2026/01/12
 * 
 * 空気量信号(raw_Ga)の脈動ノイズ除去用
 * 設計条件:
 *   - サンプルレート: 5ms (200Hz)
 *   - 遅延目標: 数十ms以下
 *   - 用途: 管内脈動のキャンセル
 */

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

// ============================================================================
// 1次カルマンフィルタ構造体
// ============================================================================
// 状態方程式:  x[k] = x[k-1] + w  (定常モデル)
// 観測方程式:  z[k] = x[k] + v
// w ~ N(0, Q), v ~ N(0, R)
// ============================================================================

typedef struct {
    // カルマンフィルタ状態
    float x;        // 推定状態 (フィルタ出力)
    float P;        // 推定誤差共分散
    
    // フィルタパラメータ
    float Q;        // プロセスノイズ共分散 (小さい→スムーズ)
    float R;        // 観測ノイズ共分散 (大きい→スムーズ)
    
    // 統計情報
    bool initialized;
    uint32_t sampleCount;
} KalmanFilter_t;

// ============================================================================
// 推奨パラメータプリセット
// ============================================================================
// 脈動キャンセル用 (応答やや遅め、スムージング強め)
// 遅延: 約20-30ms (4-6サンプル相当の位相遅れ)
#define KALMAN_PRESET_PULSATION_Q   0.01f
#define KALMAN_PRESET_PULSATION_R   0.5f

// 高応答用 (応答速め、ノイズやや残る)
// 遅延: 約10-15ms (2-3サンプル相当の位相遅れ)
#define KALMAN_PRESET_FAST_Q        0.1f
#define KALMAN_PRESET_FAST_R        0.2f

// バランス型 (中間設定)
// 遅延: 約15-20ms (3-4サンプル相当の位相遅れ)
#define KALMAN_PRESET_BALANCED_Q    0.05f
#define KALMAN_PRESET_BALANCED_R    0.3f

// ============================================================================
// 関数プロトタイプ
// ============================================================================

/**
 * @brief カルマンフィルタの初期化
 * @param kf フィルタ構造体ポインタ
 * @param Q プロセスノイズ共分散 (推奨: 0.01〜0.1)
 * @param R 観測ノイズ共分散 (推奨: 0.1〜1.0)
 */
inline void kalmanInit(KalmanFilter_t* kf, float Q, float R) {
    kf->x = 0.0f;
    kf->P = 1.0f;   // 初期推定誤差 (大きめに設定)
    kf->Q = Q;
    kf->R = R;
    kf->initialized = false;
    kf->sampleCount = 0;
}

/**
 * @brief カルマンフィルタの初期化（プリセット版：脈動キャンセル用）
 * @param kf フィルタ構造体ポインタ
 */
inline void kalmanInitPulsation(KalmanFilter_t* kf) {
    kalmanInit(kf, KALMAN_PRESET_PULSATION_Q, KALMAN_PRESET_PULSATION_R);
}

/**
 * @brief カルマンフィルタの初期化（プリセット版：高応答用）
 * @param kf フィルタ構造体ポインタ
 */
inline void kalmanInitFast(KalmanFilter_t* kf) {
    kalmanInit(kf, KALMAN_PRESET_FAST_Q, KALMAN_PRESET_FAST_R);
}

/**
 * @brief カルマンフィルタの初期化（プリセット版：バランス型）
 * @param kf フィルタ構造体ポインタ
 */
inline void kalmanInitBalanced(KalmanFilter_t* kf) {
    kalmanInit(kf, KALMAN_PRESET_BALANCED_Q, KALMAN_PRESET_BALANCED_R);
}

/**
 * @brief カルマンフィルタ更新 (メイン処理)
 * @param kf フィルタ構造体ポインタ
 * @param measurement 観測値 (生の測定値)
 * @return float フィルタ適用後の推定値
 * 
 * 処理時間: 数μs (ESP32@240MHz)
 * 遅延: パラメータ依存、概ね2〜6サンプル (10〜30ms @5msサンプリング)
 */
inline float kalmanUpdate(KalmanFilter_t* kf, float measurement) {
    // 初回は観測値で初期化
    if (!kf->initialized) {
        kf->x = measurement;
        kf->P = 1.0f;
        kf->initialized = true;
        kf->sampleCount = 1;
        return kf->x;
    }
    
    kf->sampleCount++;
    
    // === 予測ステップ (Predict) ===
    // x_pred = x (定常モデルなので状態は変化しない)
    // P_pred = P + Q
    float P_pred = kf->P + kf->Q;
    
    // === 更新ステップ (Update) ===
    // カルマンゲイン: K = P_pred / (P_pred + R)
    float K = P_pred / (P_pred + kf->R);
    
    // 状態更新: x = x_pred + K * (z - x_pred)
    // イノベーション (観測残差): y = z - x_pred
    float innovation = measurement - kf->x;
    kf->x = kf->x + K * innovation;
    
    // 誤差共分散更新: P = (1 - K) * P_pred
    kf->P = (1.0f - K) * P_pred;
    
    return kf->x;
}

/**
 * @brief フィルタのリセット
 * @param kf フィルタ構造体ポインタ
 */
inline void kalmanReset(KalmanFilter_t* kf) {
    float Q = kf->Q;
    float R = kf->R;
    kalmanInit(kf, Q, R);
}

/**
 * @brief パラメータの動的変更
 * @param kf フィルタ構造体ポインタ
 * @param Q 新しいプロセスノイズ共分散
 * @param R 新しい観測ノイズ共分散
 * 
 * Note: フィルタ状態はリセットされない（連続動作）
 */
inline void kalmanSetParams(KalmanFilter_t* kf, float Q, float R) {
    kf->Q = Q;
    kf->R = R;
}

/**
 * @brief 現在のカルマンゲインを取得（デバッグ用）
 * @param kf フィルタ構造体ポインタ
 * @return float 現在のカルマンゲイン (0〜1)
 */
inline float kalmanGetGain(const KalmanFilter_t* kf) {
    float P_pred = kf->P + kf->Q;
    return P_pred / (P_pred + kf->R);
}

#endif // KALMAN_FILTER_H
