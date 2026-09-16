# 使用教學（新手版）

這份文件教你從零開始，用 `doc_translator` 翻譯一份 PDF 或 Word 文件。
技術細節、設計原理請看同目錄的 `README.md`；這裡只講「怎麼用」。

---

## 第一步：安裝

打開終端機，切到專案目錄，跑一次就好：

```bash
cd jetbot_ros
pip install -r tools/doc_translator/requirements.txt
```

---

## 第二步：設定 API 金鑰

需要一把 Anthropic 的 API 金鑰（在 [console.anthropic.com](https://console.anthropic.com) 申請）。

```bash
export ANTHROPIC_API_KEY=sk-ant-你的金鑰
```

這行只在目前的終端機視窗有效。想每次開機都自動生效，可以把這行加進
`~/.bashrc` 或 `~/.zshrc`。

---

## 第三步：翻譯一份文件

最簡單的用法：

```bash
python -m tools.doc_translator 你的文件.pdf
```

跑起來後，程式會先問你兩個問題：

```
目標語言 [繁體中文]：
```
→ 直接按 Enter 用預設值（繁體中文），或輸入你要的語言，例如「英文」「日文」。

```
翻譯的特殊要求（語氣、術語、讀者對象、格式慣例…）
直接按 Enter 可略過，略過時由 Claude 自行判斷最合適的譯法。
輸入完成後，空一行結束：
```
→ 這裡可以打字告訴它「這份是技術手冊，產品名不要翻」之類的要求，
  可以打好幾行，打完空一行按 Enter 結束。
  什麼都不寫、直接空一行按 Enter 也可以，AI 會自己判斷。

接著它就會開始翻譯，過程中會印出進度，最後給你一份花費摘要。

翻好的檔案會存在原檔旁邊，檔名自動加上語言代碼，例如：

```
你的文件.pdf  →  你的文件.zh-TW.pdf
```

---

## 常見用法範例

**指定翻成英文：**
```bash
python -m tools.doc_translator 手冊.pdf -t 英文
```

**把特殊要求直接寫在指令裡（就不會互動詢問）：**
```bash
python -m tools.doc_translator 手冊.pdf -y -t 英文 \
    -i "保持技術手冊口吻，產品名稱不要翻譯"
```

**一次翻多份文件：**
```bash
python -m tools.doc_translator 手冊1.pdf 手冊2.docx 手冊3.pdf -o 翻譯結果/
```

**只想知道大概要花多少錢，還不想真的翻：**
```bash
python -m tools.doc_translator 手冊.pdf --dry-run
```
這個指令不會呼叫翻譯，只會統計文件有多少內容、預估花費多少美金。

**圖片裡的文字不想被改動（保留原圖）：**
```bash
python -m tools.doc_translator 手冊.pdf --no-images
```

---

## 執行後看到什麼

翻譯完成會印出類似這樣的摘要：

```
========================================================
翻譯完成
  ✓ 手冊.pdf → 手冊.zh-TW.pdf
      文字片段 42/42、圖片 3/3 張重繪
--------------------------------------------------------
API 呼叫 6 次｜輸入 8,120 tokens（快取命中 0）｜輸出 6,340 tokens
估計花費：約 US$0.1985（依 claude-opus-5 的公開價格，未計入快取折扣）
========================================================
```

- 「文字片段 42/42」：文件裡有 42 段文字，全部翻譯成功。
- 「圖片 3/3 張重繪」：3 張圖片裡的文字都成功換成譯文並畫回原圖。
- 最後一行是這次翻譯大約花了多少錢。

---

## 常見問題

**Q：翻譯到一半我按了 Ctrl-C 中斷，重跑會重新算錢嗎？**
不會。已經翻好的片段會存在本機的快取裡，重新執行同一份文件、同樣的
語言與要求時，這些片段會直接沿用，不會再花錢重翻。

**Q：只支援 PDF 跟 Word 嗎？**
目前是的（`.pdf` 與 `.docx`）。

**Q：圖片裡的文字翻譯效果不好怎麼辦？**
可以先跑：
```bash
python -m tools.doc_translator 手冊.pdf --image-debug-dir 檢查用
```
它會在「檢查用」資料夾裡存一份紅框標示圖，讓你看出 AI 找到的文字位置準不準。
如果效果真的不理想，可以加 `--no-images` 讓圖片維持原樣、只翻文字。

**Q：想每次都用同一組特殊要求跟術語表，不想每次手動輸入？**
可以做一份設定檔，參考 `tools/doc_translator/example.config.yaml`，
然後：
```bash
python -m tools.doc_translator 手冊.pdf --config 我的設定.yaml -y
```

**Q：更多參數說明？**
```bash
python -m tools.doc_translator --help
```
