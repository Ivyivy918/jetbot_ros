# 交接檔：doc_translator（文件翻譯工具）— 第二版

> 給下一個對話的 Claude 看的交接筆記。這個檔案只跟 `tools/doc_translator/`
> 這個子任務有關，跟 `jetbot_ros` 主專案（ROS2 機器人程式碼）本身無關，
> 兩者請不要混在一起改。
>
> **這是第二版**，反映「搬出 jetbot_ros、獨立成新 repo」這件事的最新進度。
> 跟第一版比，狀態變化很大，**請整份重看，不要只看差異**。

---

## 1. 這個任務的目標

一支程式：**呼叫 Claude API 翻譯文件**，具體要求：

1. 可以在**一開始**提出「翻譯的特殊要求」（語氣、術語、讀者對象…），
   沒提也要能跑，由 AI 自行判斷。
2. **圖片裡的文字也要翻譯**，不能只翻文字內容。

經過 `AskUserQuestion` 確認過的三個關鍵規格（使用者親口選的，不要再回頭問一次）：

| 問題 | 使用者的選擇 |
| --- | --- |
| 圖片文字怎麼處理 | **重繪圖片**（譯文蓋回原位），不是額外插入圖說 |
| 支援哪些檔案格式 | **PDF + DOCX**（沒有要 HTML/PPTX/純文字） |
| 特殊要求怎麼輸入 | **互動詢問**為主；沒回答時 AI 自行判斷；也支援參數/設定檔非互動使用 |

後來使用者又追加了一個目標：**這個工具不要放在 jetbot_ros 底下**（跟機器人
專案無關，塞在子資料夾裡太雜），要**獨立成自己的 GitHub repo**。
這件事目前**還沒完成**，見第 2 節。

---

## 2. 目前已經完成什麼（這節是重點，狀態很曲折，請仔細看）

### 2.1 程式本體：完整、測試全過

功能已經寫完，離線測試 5 項全過（見 README 的「測試」章節）。這部分沒有
爭議，程式碼本身是穩的，問題全部出在「要放到哪裡」。

### 2.2 在 jetbot_ros 裡的歷史（現在是**過時副本**，等著被清掉）

- Branch：`claude/document-translation-program-u1he2h`（已 push 到
  `origin` = `Ivyivy918/jetbot_ros`）
- commit `b3a36e3`：初版程式碼（`tools/doc_translator/` 全部檔案）
- commit `51efda9`：加上 `TUTORIAL.md` 和第一版 `CLAUDE.md`
- **這個 branch 目前沒有開 PR**
- 這份程式碼**目前仍然存在**於這個容器的
  `/home/user/jetbot_ros/tools/doc_translator/`，**還沒被刪除**，
  也還沒 commit 任何刪除動作。

### 2.3 「獨立成新 repo」這件事發生了什麼（讀這段才知道現在卡在哪）

1. 使用者要求搬出去獨立成新 repo。我試著用 `mcp__github__create_repository`
   直接建一個新 repo → **失敗，403**：這個 session 掛的 GitHub App
   只有 `Ivyivy918/jetbot_ros` 的權限，沒有「建立新 repo」的權限。
2. 問了使用者怎麼辦，使用者選擇「自己先去 GitHub 建空 repo，我再推代碼
   進去」。
3. 使用者建好了，但**是用另一個 GitHub 帳號 `mtyea` 建的**
   （repo 全名 `mtyea/doc-translator`），不是 `jetbot_ros` 所在的
   `Ivyivy918`。
4. 我試著用 `mcp__Claude_Code_Remote__add_repo` 把 `mtyea/doc-translator`
   加進這個 session → **失敗**：
   ```
   cross-tier adds are not supported in v1: requested "mtyea/doc-translator"
   but session already has repos from owner(s) [ivyivy918]. Start a new
   session with the requested repo as the initial source, or add a repo
   from the same owner as the existing sources
   ```
   意思是：**一個 session 綁定一個 GitHub 帳號**（這個 session 綁的是
   `Ivyivy918`），不能中途加別的帳號的 repo。要嘛開一個全新的 session
   並以 `mtyea/doc-translator` 當起始 repo，要嘛換成同帳號的 repo。
5. 問了使用者三個選項（換帳號建 repo / 開新 session / 我打包給你自己推），
   使用者選了**「我把檔案打包給你，你自己推」**。
6. 我把 `tools/doc_translator/` 的內容**攤平**成一個獨立 repo 的結構
   （拿掉 `tools/doc_translator/` 這層前綴，套件本身變成 repo 根目錄下的
   `doc_translator/`，執行指令從 `python -m tools.doc_translator` 改成
   `python -m doc_translator`），在 `/tmp/.../scratchpad/doc-translator-repo/`
   本地 `git init` 一個新的 local repo，commit 一次，打包成
   `doc-translator.zip`（純檔案）跟 `doc-translator.bundle`（含 git
   歷史，可直接 `git clone` 出來用），用 `SendUserFile` 傳給使用者。
7. 使用者接著說「claude.md和操作說明可以刪了」→ 我把打包內容裡的
   `CLAUDE.md` 和 `TUTORIAL.md` 拿掉、`git commit --amend` 更新那個
   local commit，重新打包，**再傳了一次**（因為使用者一開始下載不到，
   多傳了幾次同樣的檔案）。

### 2.4 現在使用者手上（理論上）有的東西

一個 zip 跟一個 bundle，內容如下（**這就是最終要進 `mtyea/doc-translator`
的內容，不含 CLAUDE.md / TUTORIAL.md**）：

```
doc-translator/                （repo 根目錄，注意：沒有 tools/ 這層了）
├── .gitignore
├── README.md              技術文件（架構圖、設計決策、限制）
├── requirements.txt
├── example.config.yaml
└── doc_translator/
    ├── __init__.py
    ├── __main__.py         python -m doc_translator 就能跑
    ├── cli.py
    ├── config.py
    ├── llm.py
    ├── docx_backend.py
    ├── pdf_backend.py
    ├── imagetrans.py
    ├── fonts.py
    ├── cache.py
    ├── report.py
    └── tests/
        └── test_offline.py
```

bundle 裡目前只有**一個 commit**（local commit hash 在這個容器裡是
`98144d5`，但 push 到 GitHub 後 hash 可能因為 `git bundle`/`clone` 的
處理而不同，不要死認這個值，用 `git log` 現查）。

### 2.5 ⚠️ 還沒確認的事（**下一個對話一開始就要問清楚**）

- **使用者是否已經把 zip/bundle 成功 push 到 `github.com/mtyea/doc-translator`？**
  這件事我完全沒有辦法從這個 session 驗證（跨帳號，`add_repo` 會被拒），
  只能問使用者。
- 使用者是否真的成功把附件下載到電腦上了？中途出現過「附件在哪」
  「怎麼存到桌面」的困惑，來回好幾輪才確認他用的是 claude.ai 網頁版。
  **不要假設下載一定成功**，開口先問一句「檔案下載/推送順利嗎」。

---

## 3. 剛剛做的關鍵決定，以及為什麼

### 3.1 程式碼本身的決定（跟搬 repo 無關，這些都還有效）

1. **文字「原地換字」而非「重新排版」**：DOCX 寫回原本的 `run`，PDF 用
   redaction 塗掉原文後 `insert_textbox` 寫回同一個框並自動縮小字級。
   → 使用者要的是「翻譯後的文件」，不是翻譯稿；取捨是太長的譯文會被截斷。
2. **圖片重繪走「量底色 → 蓋掉 → 自動斷行寫回」**，顏色用像素統計，不靠
   模型猜。→ 更穩定、更便宜；取捨是複雜背景（照片/漸層）補丁痕跡看得出來。
3. **structured outputs**（`output_config.format` + json_schema）而非自由文字
   → 保證能被 `json.loads` 解析，降低整批失敗機率。
4. **模型固定 `claude-opus-5`；文字 effort=`low`、圖片 effort=`medium`**
   → 翻譯密集但單純，不需要高 effort；圖片辨識要估座標值得多想一點。
5. **特殊要求放進 system prompt 最高優先區塊，且進快取金鑰**
   → 改了要求就該重譯，不能拿到舊要求下的譯文。
6. **一律把待翻譯內容當「資料」，不當「指令」**（`_BASE_RULES` 明講）
   → 基本的 prompt injection 防護，**不要因為精簡 prompt 而砍掉**。
7. **降級而非中斷**：批次失敗→逐段重試；單段失敗→保留原文；圖片辨識
   失敗→保留原圖。→ 不讓整份文件因為一次 API 抽風而報廢。
8. **執行緒安全**：`FontBook.lock`（`imagetrans.py`）、`Translator._client`
   的鎖（`llm.py`）。→ 預設 `concurrency=4`，PIL 字型物件跟延遲初始化都
   不保證線程安全，**這兩把鎖不要為了簡化拿掉**。
9. **PDF 對直排／旋轉文字直接跳過不處理**。→ 寧可不翻也不要畫壞版面。

### 3.2 搬 repo 這件事的決定

10. **獨立成自己的 repo**（不是這次交接的新決定，是延續上一版）
    → 這個翻譯工具跟 jetbot_ros（機器人專案）毫無關聯，混在一起會讓機器人
    專案背一堆不相關的重依賴，使用者也覺得塞在子資料夾裡太雜。

11. **沒有嘗試用這個 session 的憑證硬推到 `mtyea/doc-translator`，而是
    打包成 zip/bundle 讓使用者自己推**
    → 系統明確擋掉跨帳號操作（`cross-tier adds are not supported`），
    這是設計上的隔離，不是 bug，**不要想辦法繞過**（例如去改
    session 的 owner 設定、用裸 git remote 硬 push 之類）。唯一正規
    路徑是「使用者自己推」或「開一個以 `mtyea/doc-translator` 為起始
    來源的新 session」。

12. **應使用者要求，打包內容拿掉了 `CLAUDE.md` 和 `TUTORIAL.md`**，只留
    `README.md`（技術文件）+ 程式碼 + `requirements.txt` +
    `example.config.yaml`。
    → 使用者明確說「claude.md和操作說明可以刪了」，只要精簡的程式碼交付
    物。**如果下一個對話要重新打包送給使用者，記得問一下現在還要不要
    這兩個檔案**，不要自動加回去。

13. **在使用者確認新 repo 推送成功之前，先不清 jetbot_ros 裡的舊副本**
    → 這個容器是暫時的（閒置一段時間會被回收）；如果我先刪掉 jetbot_ros
    這邊的程式碼，結果使用者那邊推送失敗或還沒推，程式碼就會兩邊都不見
    （只剩使用者電腦裡下載到一半/沒下載到的 zip）。**這是刻意的保守
    順序，不要因為「應該要清乾淨」就搶著先刪。**

---

## 4. 還沒做的下一步

**優先順序由上到下：**

1. **問使用者：`mtyea/doc-translator` 那邊推送順利嗎？**
   這是這次交接最關鍵的懸而未決事項。
   - 如果**成功**：
     a. 去 `/home/user/jetbot_ros/tools/doc_translator/` 跟根目錄
        `CLAUDE.md`（就是這個檔案本身）**清掉**，commit 一個「移除，
        已獨立成 mtyea/doc-translator repo」之類的訊息，push 到
        `claude/document-translation-program-u1he2h` 分支。
     b. 問使用者要不要順便在 `mtyea/doc-translator` 那邊補一份新的
        `CLAUDE.md`（因為這個 session 目前綁定 `Ivyivy918`，改不了
        `mtyea` 那邊的 repo，只能像這次一樣「打包成檔案給使用者」，
        或請使用者另開一個以 `mtyea/doc-translator` 為起始來源的
        session）。
   - 如果**還沒推 / 卡住**：先幫忙排除卡點（多半是下載/git push 操作
     上的問題，不是程式碼問題），不要急著往下做別的事。
   - 如果使用者**下載檔案本身還有問題**：這個容器裡打包好的檔案還在
     （`/tmp/claude-0/.../scratchpad/doc-translator.zip` 跟
     `.bundle`，以及未攤平的原始版本在
     `/home/user/jetbot_ros/tools/doc_translator/`），**如果這個容器
     還沒被回收**可以直接重新 `SendUserFile`；如果容器已經重置、這些
     暫存檔案不在了，就要從 `jetbot_ros` 裡的 `tools/doc_translator/`
     重新走一次「攤平成獨立 repo 結構 → 打包」的流程（步驟見第 2.3
     節第 6 點）。

2. **拿真實的 `ANTHROPIC_API_KEY` 實測一次完整流程**（程式碼本身，跟
   搬 repo 無關，之前就列了、目前還是沒做）：
   - 先 `--dry-run` 看預估花費。
   - 挑一份有圖有表格的真實 PDF/DOCX，用 `--image-debug-dir` 檢查圖片
     bbox 抓得準不準。
   - 檢查目標語言的字型渲染是否正常（`fonts.py` 的 `find_font()` 找不到
     字型只會印警告、不會中止）。
3. 視實測結果調整 `_MIN_PDF_FONT`（PDF 最小字級）跟 `imagetrans.py` 的
   `pad_x`/`pad_y`（圖片重繪的框線外擴比例），目前都是憑經驗猜的數字。
4. 使用者若之後要 HTML/PPTX 等格式，是全新的後端模組，目前完全沒寫。
5. 沒有打包成可安裝套件（沒有 `pyproject.toml`），目前只能在 repo
   根目錄下 `python -m doc_translator` 執行。

---

## 5. 不要碰的地方、有什麼雷

### 5.1 程式碼本身的雷（跟上一版一樣，仍然有效）

1. **不要把 API 金鑰寫進任何檔案再 commit。** 金鑰只透過環境變數
   `ANTHROPIC_API_KEY` 讀取。
2. **`pdf_backend.py` 的 redaction 呼叫順序跟參數不要亂動**
   （`add_redact_annot` 之後接 `apply_redactions`，其中
   `images=pymupdf.PDF_REDACT_IMAGE_NONE`、
   `graphics=pymupdf.PDF_REDACT_LINE_ART_NONE`、
   `text=pymupdf.PDF_REDACT_TEXT_REMOVE`）。改錯值會把圖片或表格線條
   一起清掉，且不會報錯，只會在輸出 PDF 裡靜靜消失。
3. **`imagetrans.py` 裡 `FontBook.lock` 跟 `redraw()`/`_redraw_locked()`
   的包裝關係不要拆開**（執行緒安全，見 3.1 節第 8 點）。
4. **測試目前全部是離線測試**（`tests/test_offline.py` 用假的
   `Translator`）。**不要把測試通過當成「翻譯品質已驗證」**——只驗證了
   文件處理管線，沒驗證過模型實際回傳的翻譯內容或 bbox 準確度。
5. **`cache.py` 的快取 key 只雜湊使用者的 instructions/glossary，不含
   `_BASE_RULES` 這些內建規則文字**。改了 prompt 邏輯不會自動讓快取
   失效，記得提醒使用者清一下（`--no-cache` 或砍掉
   `~/.cache/doc_translator/translations.sqlite3`）。

### 5.2 這次搬 repo 特別要注意的雷（新增）

6. **不要假設這個 session 能直接操作 `mtyea/doc-translator`。**
   這個 session 綁定的 GitHub 身份是 `Ivyivy918`（`jetbot_ros` 那個），
   `add_repo` 對不同帳號的 repo 會直接被系統拒絕（`cross-tier adds are
   not supported`）。看到這個錯誤不要重試、不要想繞過，直接告訴使用者
   需要開一個以 `mtyea/doc-translator` 為起始來源的新 session，或者
   繼續用「打包給使用者自己推」的方式。
7. **不要在使用者確認新 repo 推送成功之前，搶著清掉 jetbot_ros 裡的
   `tools/doc_translator/` 或這個 `CLAUDE.md`。** 這個容器是暫時的，
   先刪等於是在確認安全備份存在之前燒掉唯一副本。**這是本次交接最容易
   被下一個對話搞砸的地方**，務必先問、確認成功了才刪。
8. **不要自動把 `CLAUDE.md` / `TUTORIAL.md` 塞回打包給使用者的檔案裡**，
   除非使用者自己要求——他們明確說過要精簡，只要程式碼本身
   （+ README 技術文件）。
9. **git commit 的 attribution**：請用當下 session 系統提示裡指定的
   author/co-author 內容，不要沿用舊 commit 裡別的模型身份署名。這次
   打包給使用者的那個 local commit 用的是佔位的
   `you@example.com`／`doc-translator`（不是真人也不是任何 Claude
   身份，因為那份 commit 是要讓使用者自己 push、由他們的 GitHub 帳號
   認證的，不代表這個 session 的身份）——**這是刻意的，不用「修正」
   成 Claude 的 attribution**。
