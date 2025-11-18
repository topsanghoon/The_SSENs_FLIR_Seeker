using GMap.NET;
using GMap.NET.MapProviders;
using GMap.NET.WindowsPresentation;
using Gst;
using Gst.App;
using Gst.Base;
using Gst.Video;
using OpenCvSharp;
using OpenCvSharp.Extensions;
using SQLitePCL;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing.Imaging;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Reflection.Emit;
using System.Text;
using System.Threading;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Forms;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Media.Media3D;
using System.Windows.Shapes;
using System.Windows.Threading;
using GMainContext = GLib.MainContext;
using Media = System.Windows.Media;
using SysDateTime = System.DateTime;
// === 충돌 방지 별칭들 ===
using Task = System.Threading.Tasks.Task;
using WpfApp = System.Windows.Application;
using WpfPoint = System.Windows.Point;

namespace TheSSENS
{
    /// <summary>
    /// [파일 분리] MainWindow의 partial class입니다.
    /// UDP 통신 (수신/송신), 메타데이터 파싱 로직을 담당합니다.
    /// </summary>
    public partial class MainWindow : System.Windows.Window
    {


        // ===== UDP 통신 =====
        private const string CMD_REMOTE_IP = "192.168.0.101";
        private const int CMD_REMOTE_PORT = 5000;
        private const int META_LOCAL_PORT = 5001;

        // [NRT 수정] Nullable(?) 추가
        private UdpClient? _cmdTx;
        private IPEndPoint? _cmdRemoteEp;
        private int level = 119;

        private UdpClient? _metaRx;
        private CancellationTokenSource? _metaCts;
        private long _lastMetaUiTickMs = 0;

        // 아르코 id
        public uint now_id = 0;

        /// <summary>
        /// UDP 송신 클라이언트를 초기화합니다.
        /// </summary>
        public void InitializeUdp()
        {
            _cmdTx = new UdpClient();
            _cmdRemoteEp = new IPEndPoint(IPAddress.Parse(CMD_REMOTE_IP), CMD_REMOTE_PORT);
            _metaCts = new CancellationTokenSource();
        }

        // ==== MetaWire v1 파서 유틸 ====
        private static uint ReadU32LE(byte[] b, int off)
        {
            return (uint)(b[off] | (b[off + 1] << 8) | (b[off + 2] << 16) | (b[off + 3] << 24));
        }
        private static ulong ReadU64LE(byte[] b, int off)
        {
            uint lo = ReadU32LE(b, off);
            uint hi = ReadU32LE(b, off + 4);
            return ((ulong)hi << 32) | lo;
        }
        private static float ReadF32LE(byte[] b, int off)
        {
            uint u = ReadU32LE(b, off);
            return BitConverter.Int32BitsToSingle(unchecked((int)u));
        }

        private void ParseAndLogMeta(byte[] buf, int n)
        {
            if (n < 4) { AppendLog("[META] packet too short"); return; }

            byte ver = buf[0];
            byte type = buf[1];
            if (ver != 1) { AppendLog($"[META] unknown ver={ver}"); return; }

            int p = 4;
            int remain = n - p;

            switch (type)
            {
                case 0x04: // HB
                    {
                        if (remain < 8) { AppendLog("[META] HB too short"); break; }
                        ulong ts = ReadU64LE(buf, p);
                        AppendLog($"[META][HB] ts={ts}");
                        _hbClock.Restart();

                        if (!_isHbConnected)
                        {
                            AppendLog("[SYSTEM] HB 수신 재개됨.");

                        }

                        _isHbConnected = true;
                        UpdateConnectionStatus(_isHbConnected);
                        break;
                    }
                case 0x03: // CTRL
                    {
                        if (remain < 12) { AppendLog("[META] CTRL too short"); break; }
                        ulong ts = ReadU64LE(buf, p + 0);
                        int val = (int)ReadU32LE(buf, p + 8);
                        AppendLog($"[META][CTRL] ts={ts} val={val}");



                        switch (val)
                        {
                            case 8001:
                                {
                                    EnableFlyStatusMid();
                                    DisableFlyStatusEnd();

                                    Dispatcher.Invoke(() =>
                                    {
                                        if (GuidanceModeText != null)
                                            GuidanceModeText.Text = "중기유도";
                                    });

                                    break;
                                }
                            case 9001:
                                {
                                    FinalHoming(10.0f);
                                    DisableFlyStatusMid();
                                    EnableFlyStatusEnd();

                                    Dispatcher.Invoke(() =>
                                    {
                                        if (GuidanceModeText != null)
                                            GuidanceModeText.Text = "종말유도";
                                    });

                                    break;
                                }
                            case 119:
                                {
                                    break;
                                }
                            default:
                                UpdateMotorValues((val).ToString(), (val).ToString(), (-val).ToString(), (-val).ToString());
                                UpdateScrollBars((double)(50 + val), (double)(50 - val));
                                break;

                        }
                        //if (val == 9001)
                        //{
                        //    DisableFlyStatusMid();
                        //    EnableFlyStatusEnd();

                        //}
                        //else if (val == 8001)
                        //{
                        //    EnableFlyStatusMid();
                        //    DisableFlyStatusEnd();
                        //}
                        //else
                        //{
                        //    // 3. UI 업데이트 함수 호출 (MainWindow.xaml.cs에 있음)
                        //    UpdateMotorValues(val.ToString(), val.ToString(), val.ToString(), val.ToString());
                        //    UpdateScrollBars((double)(50 - val), (double)(50 + val));
                        //}
                        break;
                    }
                case 0x01: // TRACK
                    {
                        if (remain < 32) { AppendLog("[META] TRACK too short"); break; }

                        uint seq = ReadU32LE(buf, p + 0);
                        ulong ts = ReadU64LE(buf, p + 4);
                        int off = 12;
                        if (remain >= 36) off += 4;

                        if (remain < off + 20) { AppendLog("[META] TRACK floats short"); break; }

                        float x = ReadF32LE(buf, p + off + 0);
                        float y = ReadF32LE(buf, p + off + 4);
                        float w = ReadF32LE(buf, p + off + 8);
                        float h = ReadF32LE(buf, p + off + 12);
                        float score = ReadF32LE(buf, p + off + 16);

                        AppendLog($"[META][TRACK] seq={seq} ts={ts} box=({x:F1},{y:F1},{w:F1},{h:F1}) score={score:F3}");
                        break;
                    }
                case 0x02: // ARUCO
                    {
                        if (remain < 10 + 4 + 16) { AppendLog("[META] ARUCO too short"); break; }

                        ulong ts = ReadU64LE(buf, p + 0);
                        uint id = ReadU32LE(buf, p + 10);
                        int off = 14;

                        if (remain >= 10 + 4 + 4 + 16) off += 4;

                        if (remain < off + 16) { AppendLog("[META] ARUCO floats short"); break; }

                        float x = ReadF32LE(buf, p + off + 0);
                        float y = ReadF32LE(buf, p + off + 4);
                        float w = ReadF32LE(buf, p + off + 8);
                        float h = ReadF32LE(buf, p + off + 12);

                        now_id = id;
                        ApplyExternalRange(OutRange(h)); // 약 35(최대) 이하 부터는 탐지 못함

                        AppendLog($"[META][ARUCO] id={id} ts={ts} box=({x:F1},{y:F1},{w:F1},{h})");
                        break;
                    }

                // [추가] 텔레메트리 패킷 (0x05) 처리
                case 0x05: // STATUS / TELEMETRY
                    {
                        // 4x uint32 (모터값) + 2x float (스크롤값 0-100) = 16 + 8 = 24 바이트
                        if (remain < 24) { AppendLog("[META] STATUS packet too short"); break; }

                        // 1. 모터 값 4개 읽기 (uint로 가정)
                        uint m1 = ReadU32LE(buf, p + 0);
                        uint m2 = ReadU32LE(buf, p + 4);
                        uint m3 = ReadU32LE(buf, p + 8);
                        uint m4 = ReadU32LE(buf, p + 12);

                        // 2. 스크롤 값 2개 읽기 (float 0~100 범위로 가정)
                        float s1 = ReadF32LE(buf, p + 16);
                        float s2 = ReadF32LE(buf, p + 20);

                        break;
                    }

                default:
                    AppendLog($"[META] unknown type=0x{type:X2}");
                    break;
            }
        }
        private float OutRange(float h)
        {
            float d = (float)((239.016f / (9.0f * h)) * 180.0f);
            return d; // "남은 거리" [cm]
        }

        private void ParseAndHandleMeta_NoUi(byte[] buf, int n)
        {
            try
            {
                int idx = 0;
                byte ver = buf[idx++]; byte type = buf[idx++];
                if (ver != 1) return;

                int p = 4; // V1 파서(p=4)와 동일하게 오프셋 시작
                int remain = n - p;

                // ===== TRACK (0x01) =====
                if (type == 0x01)
                {
                    // V2의 상세한 패킷 길이/오프셋 대신 V1의 파서(ParseAndLogMeta) 로직을 따름
                    if (remain < 32) return;

                    int off = 12;
                    if (remain >= 36) off += 4; // V1과 동일한 오프셋 로직
                    if (remain < off + 20) return;

                    _bx = ReadF32LE(buf, p + off + 0); // volatile 필드 업데이트
                    _by = ReadF32LE(buf, p + off + 4);
                    _bw = ReadF32LE(buf, p + off + 8);
                    _bh = ReadF32LE(buf, p + off + 12);
                    // float score = ReadF32LE(buf, p + off + 16);

                    _hasBox = true; // volatile 플래그
                    _trkClock.Restart(); // 1단계에서 추가한 스톱워치
                    return;
                }

                // ===== ARUCO (0x02) =====
                if (type == 0x02)
                {
                    // 최소 길이 검사 (기본 헤더 + 타임스탬프 + ID + FLAGS까지)
                    if (n < idx + 4 + 8 + 4 + 4) return;

                    idx += 4; // version, type 다음 reserved 건너뛰기

                    /*ulong ts =*/
                    ReadU64LE(buf, idx); idx += 8; // 타임스탬프
                    uint id = ReadU32LE(buf, idx); idx += 4; // ID

                    // ⭐ 핵심: FLAGS 필드 읽기
                    uint flags = ReadU32LE(buf, idx); idx += 4;

                    bool hasBox = (flags & 1u) != 0;
                    bool hasCorners = (flags & 2u) != 0;

                    float bx = 0, by = 0, bw = 0, bh = 0;

                    // 1. Box 정보가 있다면 읽기
                    if (hasBox)
                    {
                        if (n < idx + 16) return; // Box 데이터 길이 체크
                        bx = ReadF32LE(buf, idx); idx += 4; by = ReadF32LE(buf, idx); idx += 4;
                        bw = ReadF32LE(buf, idx); idx += 4; bh = ReadF32LE(buf, idx); idx += 4;
                    }

                    // 2. Corners 정보가 있다면 읽기
                    var cx = new float[4]; var cy = new float[4];
                    if (hasCorners)
                    {
                        if (n < idx + 32) return; // Corners 데이터 길이 체크 (4 * 2 * float)
                        for (int k = 0; k < 4; k++)
                        {
                            cx[k] = ReadF32LE(buf, idx); idx += 4;
                            cy[k] = ReadF32LE(buf, idx); idx += 4;
                        }
                    }

                    // 3. UI 필드 업데이트
                    if (hasCorners)
                    {
                        // 코너 좌표가 있으면 코너 정보를 우선 사용
                        _arucoPts[0] = new WpfPoint(cx[0], cy[0]);
                        _arucoPts[1] = new WpfPoint(cx[1], cy[1]);
                        _arucoPts[2] = new WpfPoint(cx[2], cy[2]);
                        _arucoPts[3] = new WpfPoint(cx[3], cy[3]);
                        _arucoHas = true;
                    }
                    else if (hasBox)
                    {
                        // 코너는 없고 박스만 있으면 직사각형으로 변환하여 사용
                        _arucoPts[0] = new WpfPoint(bx, by);
                        _arucoPts[1] = new WpfPoint(bx + bw, by);
                        _arucoPts[2] = new WpfPoint(bx + bw, by + bh);
                        _arucoPts[3] = new WpfPoint(bx, by + bh);
                        _arucoHas = true;
                    }
                    else
                    {
                        // 박스나 코너 정보가 없으면 그리지 않음
                        _arucoHas = false;
                    }

                    _arucoId = id;
                    _arucoClock.Restart();
                    return;
                }

                //if (type == 0x02)
                //{
                //    if (remain < 10 + 4 + 16) return;

                //    uint id = ReadU32LE(buf, p + 10);
                //    int off = 14; 

                //    if (remain >= 10 + 4 + 4 + 16) off += 4;
                //    if (remain < off + 16) return;

                //    float x = ReadF32LE(buf, p + off + 0);
                //    float y = ReadF32LE(buf, p + off + 4);
                //    float w = ReadF32LE(buf, p + off + 8);
                //    float h = ReadF32LE(buf, p + off + 12);

                //    // V2의 ArUco 파서 로직 (hasCorners 대신 hasBox 사용)
                //    // (V1에는 4개 코너 좌표가 아닌 박스 좌표만 있었으므로 V1을 따름)
                //    _arucoPts[0] = new WpfPoint(x, y);
                //    _arucoPts[1] = new WpfPoint(x + w, y);
                //    _arucoPts[2] = new WpfPoint(x + w, y + h);
                //    _arucoPts[3] = new WpfPoint(x, y + h);
                //    _arucoHas = true;
                //    _arucoId = id;
                //    _arucoClock.Restart();
                //    return;
                //}
            }
            catch (Exception)
            {
                // 백그라운드 파싱 오류는 무시 (로그 스팸 방지)
            }
        }

        // ====== 바이너리 클릭 송신 유틸 ======
        private void SendClickBinary(float x, float y)
        {
            // [NRT 수정] _cmdTx, _cmdRemoteEp null 체크
            if (_cmdTx == null || _cmdRemoteEp == null) return;

            byte[] pkt = new byte[9];
            pkt[0] = 0; // CLICK

            int xi = BitConverter.SingleToInt32Bits(x);
            int yi = BitConverter.SingleToInt32Bits(y);

            byte[] xb = BitConverter.GetBytes(IPAddress.HostToNetworkOrder(xi));
            byte[] yb = BitConverter.GetBytes(IPAddress.HostToNetworkOrder(yi));

            System.Buffer.BlockCopy(xb, 0, pkt, 1, 4);
            System.Buffer.BlockCopy(yb, 0, pkt, 5, 4);

            _cmdTx.Send(pkt, pkt.Length, _cmdRemoteEp);
            AppendLog($"[TX BIN→{_cmdRemoteEp.Address}:{_cmdRemoteEp.Port}] CLICK x={x:F1}, y={y:F1}");
        }


        // ====== 텍스트 송신 유틸 ======
        private void SendCmd(string msg)
        {
            // [NRT 수정] _cmdTx, _cmdRemoteEp null 체크
            if (_cmdTx == null || _cmdRemoteEp == null) return;
            try
            {
                //var data = Encoding.UTF8.GetBytes(msg);
                //_cmdTx.Send(data, data.Length, _cmdRemoteEp);
                //AppendLog($"[TX→{_cmdRemoteEp.Address}:{_cmdRemoteEp.Port}] {msg}");

                msg = (level >= 0) ? $"SD {level}" : "SD";
                var data = Encoding.UTF8.GetBytes(msg);
                _cmdTx.Send(data, data.Length, _cmdRemoteEp);
                AppendLog($"[TX→{_cmdRemoteEp.Address}:{_cmdRemoteEp.Port}] {msg}");
            }
            catch (Exception ex)
            {
                AppendLog($"[TX-ERR] {ex.Message}");
            }
        }

        // ====== HEX 문자열 유틸 ======
        // [NRT 수정] b를 nullable로
        private static string HexHead(byte[]? b, int n = 16)
        {
            // [NRT 수정] b null 체크
            if (b == null) return "(null)";
            int k = Math.Min(b.Length, n);
            var sb = new StringBuilder(k * 3);
            for (int i = 0; i < k; i++) sb.AppendFormat("{0:X2} ", b[i]);
            if (b.Length > k) sb.Append("...");
            return sb.ToString();
        }

        // ====== 메타 수신 루프 ======
        private void StartMetaReceiver()
        {
            try
            {
                _metaRx = new UdpClient(AddressFamily.InterNetwork);
                _metaRx.Client.ExclusiveAddressUse = false;
                _metaRx.Client.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.ReuseAddress, true);
                _metaRx.Client.Bind(new IPEndPoint(IPAddress.Any, META_LOCAL_PORT));
                AppendLog($"[META] 수신 대기 시작 :{META_LOCAL_PORT}/udp");
            }
            catch (Exception ex)
            {
                AppendLog($"[META-ERR] 소켓 오픈 실패: {ex.Message}");
                return;
            }

            _metaCts = _metaCts ?? new CancellationTokenSource();

            Task.Run(async () =>
            {
                // [NRT 수정] _metaCts null 체크 (이미 위에서 null 아님 보장됨)
                try
                {
                    while (!_metaCts.IsCancellationRequested)
                    {
                        UdpReceiveResult res;
                        try
                        {
                            // [NRT 수정] _metaRx null 체크
                            if (_metaRx == null) break;
                            res = await _metaRx.ReceiveAsync().ConfigureAwait(false);
                        }
                        catch (ObjectDisposedException) { break; }
                        catch (Exception ex)
                        {
                            AppendLog($"[META-ERR] ReceiveAsync Exception: {ex.Message}");
                            break;
                        }

                        var buf = res.Buffer;
                        int n = buf?.Length ?? 0;

                        if (n >= 4 && buf != null && buf[0] == 0x01 && (buf[1] >= 0x01 && buf[1] <= 0x02))
                        {
                            // 1. (60fps) GDI+ 렌더링을 위해 좌표 파싱 (V2 원본 파서)
                            ParseAndHandleMeta_NoUi(buf, n);

                            if (buf[1] == 0x01)
                            {
                                //videoPanelLeft?.Invalidate();
                            }
                            if (buf[1] == 0x02)
                            {
                                // videoPanelRight?.Invalidate();
                            }
                        }
                        // --- [V2 병합 끝] ---

                        await Dispatcher.InvokeAsync(() =>
                        {
                            AppendLog($"[META][RX] {res.RemoteEndPoint.Address}:..."); // 매번 로그 (이건 괜찮음)

                            if (n >= 4 && buf != null && buf[0] == 0x01 && (buf[1] >= 0x01 && buf[1] <= 0x05))
                            {
                                byte type = buf[1];

                                // [V2 최적화] 0x01, 0x02, 0x03, 0x04는 로그만 찍음
                                if (type >= 0x01 && type <= 0x04)
                                {
                                    ParseAndLogMeta(buf, n); // 로그만 찍는 것은 UI 부담 적음
                                }
                                // [V2 최적화] 0x05 (모터/스크롤 UI)는 200ms에 한 번만 처리
                                else if (type == 0x05)
                                {
                                    long now = NowMs(); // (MainWindow.xaml.cs에 추가한 함수)
                                    if (now - Interlocked.Read(ref _lastMetaUiTickMs) > 200)
                                    {
                                        Interlocked.Exchange(ref _lastMetaUiTickMs, now);
                                        ParseAndLogMeta(buf, n); // 200ms에 한 번만 UI 업데이트
                                    }
                                }
                            }

                            else
                            {
                                try
                                {
                                    // [NRT 수정] buf null 체크
                                    if (buf != null)
                                    {
                                        string txt = Encoding.UTF8.GetString(buf);
                                        if (!string.IsNullOrWhiteSpace(txt))
                                            AppendLog($"[META][TEXT?] {txt.Trim()}");
                                    }
                                }
                                catch { /* ignore */ }
                            }
                        });
                    }
                }
                finally
                {
                }
            }, _metaCts.Token);
        }
    }
}