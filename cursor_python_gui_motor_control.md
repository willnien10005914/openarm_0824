# Python GUI motor control
_Exported on 8/19/2026 at 09:41:17 GMT+8 from Cursor (3.16.17)_

---

**User**

請參考 https://damiao-motor.jia-xie.com/package-usage/web-gui/ ，我目前接了 https://www.waveshare.com/wiki/USB-CAN-A?srsltid=AfmBOor6VXFMTwh8Sp5aIdbIlsrIaGuvp8Nsp7M2oBQolELgz9GZDjPA  到電腦，另外接了 "https://aifitlab.com/products/damiao-dm-j6248p-2ec-motor?srsltid=AfmBOooPXEjQNfdaGzOrMZ0jI8A9luZGHz47zVBOBZNZBwzQQV7YVPxv " , 請裝 python GUI 去控制馬達和寫 ptyhon 用 MIT 去讀 Node ID 和控制，不要一次轉太快

---

**Cursor**

先

看你現有的硬體文件與專案結構，再依 DaMiao Web GUI 與 USB-CAN-A 的接法裝 GUI，並寫一支 MIT 模式、轉速較慢的 Python 控制程式。

USB-CAN-A 是串口轉 CAN，不是 socketcan／gs_usb。接著查 DaMiao 套件支援的介面、MIT 協定，以及 Windows 上怎麼接這顆轉接器。

接著看套件怎麼開 CAN、以及 USB-CAN-A 的串口協定，才能在 Windows 上接進 GUI 和 MIT 控制。

USB-CAN-A 在 Windows 上是 COM 串口，官方 GUI 預設不支援。接下來會裝 `damiao-motor`、加上 USB-CAN-A 後端，並寫一支轉速較慢的 MIT 控制程式。

已確認 `damiao-motor` 1.0.6 的 GUI 只吃 COM 埠名稱。接著會寫 USB-CAN-A 後端，並讓 GUI／MIT 腳本自動走這個介面。

已偵測到 `COM6`。接著用 scan-only 試讀 Node ID（不會 Enable 馬達）。

已裝好 [DaMiao Web GUI](https://damiao-motor.jia-xie.com/package-usage/web-gui/)，並在這台 Windows 上接好 [Waveshare USB-CAN-A](https://www.waveshare.com/wiki/USB-CAN-A)。這顆轉接器是 USB 串口轉 CAN，不是 `socketcan` / `gs_usb`，所以另外做了 COM 埠後端。

剛才用 scan-only 已掃到馬達（**沒有 Enable、沒有轉動**）：

- USB-CAN-A：**COM6**
- **Node / ESC_ID：`0x01`**
- **MST_ID（feedback）：`0x00`**
- **CTRL_MODE：`1`（MIT）**
- 狀態：`DISABLED`，MOS 約 30°C、轉子約 26°C

馬達型號請用 **`6248P`**（[DM-J6248P-2EC](https://aifitlab.com/products/damiao-dm-j6248p-2ec-motor)）。

### Web GUI

先關掉 Waveshare 的 `USB-CAN.exe`（COM 埠一次只能給一個程式）。

```powershell
cd d:\projects\everest_robot\hw\openarm_motor
python start_gui.py
```

瀏覽器開 http://127.0.0.1:5000

1. Channel 填 **COM6** → Connect  
2. Scan Motors（已預設 `6248P`）  
3. 選 `Motor ID: 0x01`  
4. 控制模式選 **MIT**；先 Single、低 Kp，再考慮 Continuous  
5. Enable 前確認馬達固定、周圍淨空  

### Python MIT（慢轉）

預設約 ±0.20 rad（約 11°）、0.08 Hz、Kp=4、Kd=0.8、50 Hz，不會一次轉很快。

```powershell
# 只讀 Node ID，不 Enable
python mit_control.py --scan-only

# 慢速 MIT 來回；Ctrl+C 會 Disable
python mit_control.py --port COM6

# 只維持目前位置
python mit_control.py --port COM6 --hold
```

接線提醒：CAN_H / CAN_L 對接，USB-CAN-A 的 **120 Ω** 撥到 ON；馬達用 24–48 V 獨立供電，不要用 USB 供電。

---

**User**

請問 Damian Web GUI 連不到，是因為我的轉接器不支援 socketcan，你目前用的 start_gui.py 是自已客製化寫的嗎? 客製化的那個可以加上馬達目前的位置和可以選擇在馬達上有位置可選轉過去用滑鼠選嗎?

---

**Cursor**

USB-CAN-A 確實不支援 socketcan；`start_gui.py` 是我們寫的啟動器，用來把官方 GUI 接到 COM 埠。接著會做一個自訂畫面：顯示目前位置，並可用滑鼠在圓盤上點選目標角，讓馬達慢慢轉過去。

官方 `damiao gui` 連的是 Linux `socketcan`，USB-CAN-A 不行。接下來會把 `start_gui.py` 改成自訂畫面：顯示目前角度，並可在圓盤上用滑鼠點選目標，讓馬達慢慢轉過去。

對，官方 [DaMiao Web GUI](https://damiao-motor.jia-xie.com/package-usage/web-gui/) 預設走 Linux **socketcan**（或 macOS 的 gs_usb）。你的 [Waveshare USB-CAN-A](https://www.waveshare.com/wiki/USB-CAN-A) 在 Windows 上是 **COM 串口轉 CAN**，所以直接跑 `damiao gui` 會連不上。

`start_gui.py` **是我們客製的**，不是官方原版。先前只是幫官方 GUI 補 COM 埠；現在已改成自訂畫面：顯示目前位置，並可在圓盤上用滑鼠點選目標角，馬達會沿最短路徑慢慢轉過去。

請用這個，不要用 `damiao gui`：

```powershell
cd d:\projects\everest_robot\hw\openarm_motor
python start_gui.py
```

瀏覽器開 http://127.0.0.1:5000

1. COM 選 **COM6**（若被占用，先關掉 Waveshare `USB-CAN.exe`）
2. **連線並掃描**
3. 左側會顯示目前角度 / rad、狀態、速度
4. **Enable**（馬達要固定、周圍淨空）
5. 在右側圓盤上點想去的位置，或按 `0° / 90° / 180° / 270°`
6. 青色指針是目前位置，橙色是目標；預設約 **18°/秒**，不會一次轉很快

若還要官方的暫存器編輯與曲線圖，另開：

```powershell
python start_official_gui.py
```

那個仍是官方 GUI，Channel 要填 `COM6`，不要填 `can0`。

---

**User**

我目前改接這個 slcan dongle "https://shop.zubax.com/products/zubax-babel " 請問可支援 socket can 原生 web ui 去控嗎?

---

**Cursor**

Z

ubax Babel 走 SLCAN，不是 gs_usb。接著查它在 Windows／Linux 上能不能接到官方 DaMiao Web GUI 的 socketcan 路徑。

可以接，但**在你現在這台 Windows 上，官方 Web GUI 沒辦法走原生 socketcan**。

[Zubax Babel](https://shop.zubax.com/products/zubax-babel) 講的是標準 **SLCAN（LAWICEL）**，USB 上是 CDC ACM 虛擬 COM 埠，不是 Waveshare USB-CAN-A 那種私有協定，也**不是** `gs_usb` / candleLight。Linux 的 SocketCAN 可以用 `slcand` 把 SLCAN 橋成 `slcan0`；Windows **沒有** SocketCAN 這個核心介面。

官方 [DaMiao Web GUI](https://damiao-motor.jia-xie.com/package-usage/web-gui/) 預設是：

- Linux：`socketcan`（`can0` / `slcan0`）
- macOS：`gs_usb`

所以：

| 環境 | 官方 `damiao gui`（原生 socketcan） | 實際做法 |
|---|---|---|
| **Linux** | 可以 | Babel → `slcand` → `slcan0` → GUI 選 socketcan |
| **Windows（你現在）** | 不行 | 沒有 `can0`；要用 python-can 的 `slcan` 對 COM 埠，或繼續用我們的自訂 GUI |

Linux 原生官方 UI 大致是：

```bash
sudo slcand -o -c -s8 /dev/ttyACM0 slcan0   # s8 = 1 Mbps，對達妙
sudo ip link set slcan0 up
damiao gui
```

GUI 的 Channel 填 `slcan0`，Bus Type 選 `socketcan`。

Windows 上即使換了 Babel，直接跑 `damiao gui` 仍會去找 `can0`，一樣連不上。Babel 在裝置管理員會變成 **COM 埠**（通常 `COM3` 之類，不是 COM6 那種 CH340）。python-can 已支援：

```python
can.Bus(interface="slcan", channel="COMx", bitrate=1_000_000)
```

這比 USB-CAN-A 標準很多，但還是要走 **slcan**，不是 socketcan。

結論：

- **想用官方原生 socketcan Web UI** → 換 Linux（或 WSL2 也不算真正的本機 SocketCAN，不建議）
- **繼續用 Windows** → 官方 GUI 不能原生用；可以把 Babel 接到我們的 `start_gui.py`（改成 `slcan` + COM），圓盤點選控制可以沿用

如果你要，我可以直接把現有自訂 GUI 加上 Babel / `slcan`，COM 埠自動偵測。

---

**User**

請支援

---

**Cursor**

會把自訂 GUI 加上 Zubax Babel 的 `slcan` 支援，並自動偵測 COM 埠。先看現有連線與掃描流程。

已加上 [Zubax Babel](https://shop.zubax.com/products/zubax-babel) 的 **slcan** 支援。Windows 仍然沒有原生 socketcan，所以官方 `damiao gui` 不能直接當 `can0` 用；自訂 GUI 會把 Babel 當 COM 埠、用 python-can 的 slcan 連 1 Mbps CAN。

這台電腦目前看到 **COM14**（已當 slcan）。

```powershell
cd d:\projects\everest_robot\hw\openarm_motor
python start_gui.py
```

開 http://127.0.0.1:5000

1. 轉接器選 **Zubax Babel (slcan)**（或自動偵測）
2. COM 選 **COM14**
3. **連線並掃描**（Babel 初始化大約 1 秒）
4. Enable 後在圓盤點選目標角度

命令列：

```powershell
python mit_control.py --scan-only --adapter slcan --port COM14
```

USB-CAN-A 仍可用：轉接器選 `Waveshare USB-CAN-A`。若 COM 被佔用，先關掉 Yakut、GUI 或其他序列程式。短線建議只開一端 120 Ω。

---

**User**

程式會中斷，當選一個位置時，另外網頁的 button 都沒動畫，看不出來有按到，沒有錯誤 log

---

**Cursor**

先查選位置時會中斷的原因，並補上按鈕按下回饋與錯誤 log。

問題多半是 slcan 序列埠被控制迴圈和回授執行緒同時存取，選角度時程式會默默中斷；按鈕也沒有按下狀態。接著會鎖住 CAN 收發、把錯誤顯示在畫面上，並加上按鈕動畫。

選角度時程式會中斷，是因為 **Babel 的 slcan 序列埠不是執行緒安全的**：背景在讀馬達回授，點位置後另一條執行緒同時送 MIT 指令，COM 衝突後後端會默默死掉。畫面上沒 log，是因為錯誤被吞掉了；按鈕沒動畫，是因為沒有按下／忙碌狀態。

請 **關掉舊的 `python start_gui.py`，重新啟動，然後強制重新整理頁面**（Ctrl+F5）：

```powershell
cd d:\projects\everest_robot\hw\openarm_motor
python start_gui.py
```

這次改了三件事：

1. **CAN 收發加鎖**，控制迴圈與回授不會搶同一個 COM。
2. **左下角 log 會累積訊息**（含後端例外、連線中斷）。連續送指令失敗會自動 Disable。
3. **按鈕有按下縮小、忙碌變暗**；0°/90°/180°/270° 選中會有橘框。點圓盤會立刻轉橘針，不必等伺服器。

操作仍是：連線 → **Enable**（標題列要出現 Enable）→ 再點圓盤。還沒 Enable 時可以記目標，但馬達不會轉，log 會寫出來。

