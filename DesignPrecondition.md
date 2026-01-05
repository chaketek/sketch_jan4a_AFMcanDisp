# DesignPrecondition.md
## AFM CAN Display - 設計前提条件書

**プロジェクト名**: AFM CAN Display  
**作成日**: 2026年1月4日  
**バージョン**: 1.3

---

## 1. システム概要

### 1.1 目的
CANバスから受信された信号を可読性の高いUIでリアルタイムに表示するシステム。
車両のエアフローメータ（AFM）から送信されるCAN信号を解析し、視認性の高いグラフィカルインターフェースで表示する。

### 1.2 表示対象信号
| 信号名 | CAN信号名 | 説明 | 範囲 | 単位 |
|--------|-----------|------|------|------|
| AFM出力周波数 | AFM_Freq | エアフロメータの出力周波数 | 2000 ~ 20000 | Hz |
| 空気量(Raw) | raw_Ga | 吸入空気量（生値） | 0 ~ 200 | g/s |
| 吸気温度 | THA | 吸気温度センサ値 | -20.0 ~ 120.0 | ℃ |
| MCU温度 | MCUtemp | マイコン内部温度 | -20.0 ~ 120.0 | ℃ |

---

## 2. ハードウェア構成

### 2.1 使用デバイス
| コンポーネント | 型番 | 説明 |
|----------------|------|------|
| メインユニット | M5Stack Core2 | ESP32ベースの開発ボード (320x240 IPS LCD搭載) |
| CANインターフェース | M5Stack CAN Unit (U085) | MCP2515ベースCANトランシーバユニット |

### 2.2 接続構成
```
[CANバス] <---> [CAN Unit U085] <---> [M5Stack Core2]
                    |                      |
               GPIO32 (TX)            Grove Port A (本体側面)
               GPIO33 (RX)
```

### 2.3 GPIO ピンアサイン

M5Stack Core2のGroveポートは複数存在し、接続位置によってGPIOが異なる。

| ポート | 位置 | GPIO (TX/RX) | 用途 |
|--------|------|--------------|------|
| **PORT-A (赤)** | 本体側面 | **G32 / G33** | I2C ※実際に使用 |
| PORT-B (黒) | M5GO Bottom等 | G26 / G36 | DAC/ADC |
| PORT-C (青) | - | G14 / G13 | UART |

> **Note**: 本プロジェクトではPORT-A (G32/G33) を使用。M5Stack初代のサンプルコード(G26/G36)とは異なるため注意。

| 機能 | GPIO | 備考 |
|------|------|------|
| CAN TX | GPIO 32 | PORT-A 使用時 |
| CAN RX | GPIO 33 | PORT-A 使用時 |

---

## 3. ソフトウェア構成

### 3.1 開発環境
| 項目 | 詳細 |
|------|------|
| IDE | Arduino IDE |
| ボードマネージャ | M5Stack Board Manager v2.1.3以上 |
| ターゲットボード | M5Stack Core2 |

### 3.2 依存ライブラリ
| ライブラリ名 | バージョン | 用途 | リポジトリ |
|--------------|------------|------|------------|
| M5Unified | ≥ 0.2.2 | M5Stack統合ライブラリ | https://github.com/m5stack/M5Unified |
| M5GFX | ≥ 0.2.3 | 高速グラフィックス描画 | https://github.com/m5stack/M5GFX |
| ESP32 TWAI Driver | (ESP-IDF内蔵) | CAN通信ドライバ | ESP32 Arduino Core標準 |

> **Note**: ESP32-Arduino-CANライブラリはESP32 Arduino Core 3.x系と互換性がないため、ESP-IDF内蔵のTWAI (Two-Wire Automotive Interface) ドライバを使用します。

### 3.3 UIフレームワーク
- **M5GFX** を使用して高速なダブルバッファリング描画を実現
- `M5Canvas` スプライトを活用したちらつきのない画面更新
- リアルタイム性を重視し、描画処理を最適化

---

## 4. CAN通信仕様

### 4.1 通信パラメータ
| パラメータ | 値 |
|------------|-----|
| ボーレート | 500 kbps |
| フレームタイプ | Standard Frame (11bit ID) |
| 受信キューサイズ | 10 (推奨) |

### 4.2 初期化コード例 (TWAI Driver)
```cpp
#include <driver/twai.h>

// PORT-A使用時 (本体側面のGrove端子)
#define CAN_TX_PIN  GPIO_NUM_32
#define CAN_RX_PIN  GPIO_NUM_33

void initCAN() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    g_config.rx_queue_len = 10;
    
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    
    twai_driver_install(&g_config, &t_config, &f_config);
    twai_start();
}
```

### 4.3 DBCファイル参照
- DBCファイル: `mx5_afmconv.dbc` （リポジトリに追加済み）
- DBCファイルに基づいてメッセージID、シグナル定義、スケーリング係数を実装

### 4.4 CANメッセージ定義（mx5_afmconv.dbc より）

#### メッセージ一覧
| メッセージ名 | CAN ID | DLC | 説明 |
|--------------|--------|-----|------|
| AFMConv1 | 0x001 | 8 | AFM主要データ（周波数、空気量、吸気温） |
| AFMconv2 | 0x002 | 8 | 補助データ（MCU温度、出力電圧、補正係数） |

#### シグナル詳細定義

**AFMConv1 (CAN ID: 0x001)**
| シグナル名 | スタートビット | 長さ | バイトオーダー | 符号 | Factor | Offset | Min | Max | 単位 | 表示対象 |
|------------|----------------|------|----------------|------|--------|--------|-----|-----|------|----------|
| AFM_Freq | 7 | 16 | Big Endian (Motorola) | Signed | 1 | 0 | -32768 | 32767 | Hz | ✓ |
| THA | 23 | 16 | Big Endian (Motorola) | Signed | 0.1 | -30 | -3306.8 | 3246.7 | ℃ | ✓ |
| raw_Ga | 39 | 16 | Big Endian (Motorola) | Signed | 0.1 | 0 | -3276.8 | 3276.7 | g/s | ✓ |
| comp_Ga | 55 | 16 | Big Endian (Motorola) | Signed | 0.1 | 0 | -3276.8 | 3276.7 | g/s | - |

**AFMconv2 (CAN ID: 0x002)**
| シグナル名 | スタートビット | 長さ | バイトオーダー | 符号 | Factor | Offset | Min | Max | 単位 | 表示対象 |
|------------|----------------|------|----------------|------|--------|--------|-----|-----|------|----------|
| THA_comp | 7 | 16 | Big Endian (Motorola) | Signed | 0.001 | 0 | -32.768 | 32.767 | - | - |
| VR1_comp | 23 | 16 | Big Endian (Motorola) | Signed | 0.001 | 0 | -32.768 | 32.767 | - | - |
| AFMoutV | 39 | 16 | Big Endian (Motorola) | Signed | 0.001 | 0 | -32.768 | 32.767 | - | - |
| MCUtemp | 55 | 16 | Big Endian (Motorola) | Signed | 0.1 | -30 | -3306.8 | 3246.7 | ℃ | ✓ |

#### シグナルデコード計算式
```
物理値 = (Raw値 × Factor) + Offset
```

| シグナル | 計算式 |
|----------|--------|
| AFM_Freq | `raw × 1 + 0` = raw（そのまま） |
| THA | `raw × 0.1 + (-30)` = raw × 0.1 - 30 |
| raw_Ga | `raw × 0.1 + 0` = raw × 0.1 |
| MCUtemp | `raw × 0.1 + (-30)` = raw × 0.1 - 30 |

### 4.5 バイト配置図

**AFMConv1 (0x001):**
```
Byte:    [0]     [1]     [2]     [3]     [4]     [5]     [6]     [7]
Bit:   7    0  15   8  23  16  31  24  39  32  47  40  55  48  63  56
       |AFM_Freq |  |  THA    |  |  raw_Ga  |  |  comp_Ga  |
       (16bit)      (16bit)      (16bit)      (16bit)
```

**AFMconv2 (0x002):**
```
Byte:    [0]     [1]     [2]     [3]     [4]     [5]     [6]     [7]
Bit:   7    0  15   8  23  16  31  24  39  32  47  40  55  48  63  56
       |THA_comp |  |VR1_comp |  | AFMoutV  |  | MCUtemp   |
       (16bit)      (16bit)      (16bit)      (16bit)
```

### 4.6 DBCファイル統合時の確認事項
DBCファイル追加時に以下の情報を確認・実装に反映する:
- [x] メッセージID (CAN ID) → 0x001, 0x002
- [x] 各シグナルのスタートビット、長さ → 上記表参照
- [x] バイトオーダー (Intel/Motorola) → Big Endian (Motorola)
- [x] スケーリング係数 (Factor, Offset) → 上記表参照
- [x] 値の範囲 (Min, Max) → 上記表参照
- [x] 物理単位 → Hz, g/s, ℃

---

## 5. UI設計要件

### 5.1 デザインコンセプト
- **航空機HUD（ヘッドアップディスプレイ）スタイル**を採用
- 視認性を最優先とした設計思想
- 高コントラスト配色（黒背景 + 緑/シアン系発光色）
- シンプルで直感的な情報表示
- 必要な情報のみを大きく明瞭に表示

### 5.2 画面レイアウト
- **解像度**: 320 x 240 ピクセル (M5Stack Core2 LCD)
- **画面構成**: 4つの信号を同時に視認可能なレイアウト（2x2グリッド推奨）
- **配色**: 
  - 背景: 黒 (#000000)
  - メイン表示色: 緑 (#00FF00) または シアン (#00FFFF)
  - 警告色: 黄 (#FFFF00)
  - 異常色: 赤 (#FF0000)

### 5.3 表示要素
| 要素 | 説明 |
|------|------|
| 信号名ラベル | 各信号の名称表示 |
| 現在値 | 大きなフォントで現在値を表示（HUDスタイル） |
| 単位表示 | Hz, g/sec 等の単位 |
| バーグラフ/メーター | 視覚的な値表示（HUD風ラダー/スケール表示） |
| 更新タイムスタンプ | 最終受信時刻（オプション） |
| 境界線/枠 | HUD風の角型枠線で情報をグルーピング |

### 5.4 HUDデザインガイドライン
- **フォント**: 等幅フォント推奨、数値は大きく明瞭に
- **線の太さ**: 細い線（1-2px）でシャープな印象
- **アニメーション**: 最小限に抑え、情報の即時認識を優先
- **レイアウト**: 左右対称または整列された配置
- **数値表示**: 固定幅フォーマット（桁揃え）で視線移動を最小化

### 5.5 更新要件
| 項目 | 目標値 |
|------|--------|
| 画面更新レート | ≥ 30 fps |
| CAN受信遅延 | ≤ 10 ms |
| 描画遅延 | ≤ 33 ms |

---

## 6. ファイル構成

```
sketch_jan4a_AFMcanDisp/
├── sketch_jan4a_AFMcanDisp.ino   # メインスケッチ
├── DesignPrecondition.md         # 本設計前提条件書
├── mx5_afmconv.dbc               # DBCファイル（CAN信号定義）
├── can_signals.h                 # CAN信号定義ヘッダ
└── ui_display.h                  # UI描画関連（オプション）
```

---

## 7. 参考リソース

### 7.1 公式ドキュメント・サンプルコード
- **CAN Unit サンプルコード**: https://github.com/m5stack/M5Stack/tree/master/examples/Unit/CAN
- **M5Stack Core2 製品ページ**: https://docs.m5stack.com/en/core/core2
- **CAN Unit U085**: https://docs.m5stack.com/en/unit/can

### 7.2 ライブラリドキュメント
- **M5GFX**: https://github.com/m5stack/M5GFX
- **M5Unified**: https://github.com/m5stack/M5Unified
- **ESP32-Arduino-CAN**: https://github.com/miwagner/ESP32-Arduino-CAN

---

## 8. 制約事項・注意点

### 8.1 ハードウェア制約
- CAN Unitは終端抵抗を内蔵していない場合があるため、バス終端処理を確認すること
- **M5Stack Core2のPORT-A (G32/G33)** を使用すること（初代M5Stackとはピン配置が異なる）
- M5Stack初代のサンプルコード (G26/G36) をそのまま使用するとCAN通信が動作しない

### 8.2 ソフトウェア制約
- ESP32のCAN機能はハードウェアCANコントローラ (TWAI) を使用
- **ESP32-Arduino-CANライブラリはESP32 Arduino Core 3.x系と非互換**（`esp_intr.h`が存在しない）
- ESP-IDF内蔵の `driver/twai.h` を直接使用すること
- 受信キューのオーバーフローに注意（高頻度メッセージ時）

### 8.4 リアルタイム性に関する注意
- CAN受信処理では `twai_receive()` のタイムアウトを0に設定し、ノンブロッキングで処理すること
- 1回のループで複数メッセージを処理する実装を推奨（キュー滞留による遅延防止）
- デバッグ用シリアル出力は本番運用時に無効化すること（`#define DEBUG_CAN_OUTPUT` をコメントアウト）

### 8.3 通信上の注意
- 500kbpsでの通信安定性を確認すること
- バス上に他のノードが存在しない場合はACKエラーが発生する可能性あり

---

## 9. 今後の作業

1. [x] DBCファイルをリポジトリに追加
2. [x] DBCファイルを解析し、シグナル定義を文書化
3. [x] `can_signals.h` ヘッダファイル作成
4. [x] UI詳細設計・モックアップ作成
5. [x] メインコード実装
6. [x] 実機テスト・デバッグ
7. [x] デバッグ出力のコンパイルスイッチ化
8. [x] CAN受信遅延の改善（バッチ処理化）
9. [ ] 追加機能の検討（ボタン操作、画面切替等）

---

## 変更履歴

| 日付 | バージョン | 変更内容 | 担当 |
|------|------------|----------|------|
| 2026/01/04 | 1.0 | 初版作成 | - |
| 2026/01/04 | 1.1 | DBCファイル解析結果追加、表示信号を4つに拡張 | - |
| 2026/01/05 | 1.2 | GPIO設定修正(PORT-A: G32/G33)、実装完了ステータス更新 | - |
| 2026/01/05 | 1.3 | CAN受信遅延改善、デバッグスイッチ追加、リアルタイム性注意事項追加 | - |
