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
using System.Text;
using System.Threading;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Forms;
using System.Windows.Input;
using System.Windows.Media.Imaging;
using System.Windows.Media.Media3D;
using System.Windows.Threading;
using GMainContext = GLib.MainContext;
using Media = System.Windows.Media;
using SysDateTime = System.DateTime;
// === 충돌 방지 별칭들 ===
using Task = System.Threading.Tasks.Task;
using WpfApp = System.Windows.Application;
using WpfPoint = System.Windows.Point;

// V2(Paint 이벤트)에서 사용하는 GDI+ 관련 using
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Drawing.Text;

namespace TheSSENS
{
    // UI 이벤트 핸들러(버튼 클릭, 키 입력), 창 관리, 오버레이 로직
    public partial class MainWindow : System.Windows.Window
    {
        public int temp = 0;
        private double _testRemain = -1;

        // GrapicOverlayWindow가 Window_ContentRendered에서 생성되므로 nullable(?)로 변경합니다.
        private GrapicOverlayWindow? _leftImageOverlay;
        private GrapicOverlayWindow? _rightImageOverlay;

        // 신호 수신 여부를 저장할 플래그
        private bool _isLeftSignalActive = false;
        private bool _isRightSignalActive = false;

        // 신호 감지용 타이머 및 상수
        private const int SignalTimeoutMs = 2000; // 신호 타임아웃 시간 (2초)
        private DispatcherTimer? _leftSignalTimer; // 왼쪽 스트림 감시용 타이머
        private DispatcherTimer? _rightSignalTimer; // 오른쪽 스트림 감시용 타이머

        private bool _isLeftOverlayActive = false;
        private bool _isRightOverlayActive = false;

        // ===== 시나리오 번호 =====
        private int _selectedScenarioId = 0;

        // ===== GStreamer =====
        // (관련 변수 및 초기화 코드는 MainWindow.GStreamer.cs로)
        private DispatcherTimer? _timer; // UpdateMotorOutputs용 타이머는 UI용이므로 여기에 둡니다.

        private GLib.MainLoop? _glibLoop;
        private Thread? _glibThread;

        // ===== UDP 통신 =====
        // (관련 변수 및 함수는 MainWindow.UDP.cs로)
        private Random _rand = new Random();

        // [NRT 수정] Nullable(?) 추가
        private SubOverlayWindow? _currentOverlay = null;
        private bool desMode = false;

        // IR(Left) 스트림 관련
        private const int IR_WIDTH = 80;
        private const int IR_HEIGHT = 60;
        private readonly object _irBmpLock = new();
        private System.Drawing.Bitmap? _irLastBmp; // GDI+ 렌더링용 IR 비트맵 캐시

        // EO(Right) 스트림 관련
        private readonly object _eoBmpLock = new();
        private System.Drawing.Bitmap? _eoLastBmp; // GDI+ 렌더링용 EO 비트맵 캐시
        private int _eoSrcW = 640; // EO 원본 가로 해상도 (메타데이터 기준)
        private int _eoSrcH = 480; // EO 원본 세로 해상도 (메타데이터 기준)

        // 메타데이터 오버레이용 변수 (UDP 스레드에서 쓰고 Paint 스레드에서 읽음)

        // IR 추적 박스 (0x01)
        private volatile bool _hasBox;
        private volatile float _bx, _by, _bw, _bh;
        private readonly Stopwatch _trkClock = new Stopwatch(); // 박스 자동 숨김용
        private readonly Stopwatch _arucoClock = new Stopwatch();
        private DispatcherTimer? _hideTimer; // 박스 자동 숨김용 타이머

        private readonly Stopwatch _hbClock = new Stopwatch();
        private bool _isHbConnected = false;

        // EO ArUco 마커 (0x02)
        private volatile bool _arucoHas = false;
        private WpfPoint[] _arucoPts = new WpfPoint[4]; // 원본(EO) 기준 좌표
        private uint _arucoId = 0;

        private readonly StringBuilder _logBuf = new();
        private long _lastFlushMs = 0;

        // [FPS 측정용 변수 추가]
        private int _framesLeft = 0;
        private int _framesRight = 0;
        private double _fpsLeft = 0.0;
        private double _fpsRight = 0.0;
        private long _lastTimeLeft = 0;
        private long _lastTimeRight = 0;
        private static long NowMs() => Stopwatch.GetTimestamp() * 1000 / Stopwatch.Frequency;

        // ===== 실행 초기값 =====
        public MainWindow()
        {
            InitializeComponent();

            // 지도를 다운받을 때, 보안 방식을 선언
            System.Net.ServicePointManager.SecurityProtocol |= System.Net.SecurityProtocolType.Tls12;

            // 지도 다운로드
            GMaps.Instance.Mode = AccessMode.ServerAndCache;
            try
            {
                // 지도 형식을 선택합니다.
                // 위성 지도 혹은 일반 지도를 선택 할 수 있다.
                // GoogleMapProvider : 일반 지도
                // GoogleTerrainMapProvider : 지형도
                // GoogleSatelliteMapProvider : 위성지도
                // GoogleHybridMapProvider : 하이브리드

                MapControl.MapProvider = GMap.NET.MapProviders.GoogleTerrainMapProvider.Instance;
            }
            catch
            {
                // 지도 로딩 실패시, 지도 형식을 선택 할 수 있다.
                MapControl.MapProvider = GMap.NET.MapProviders.GoogleSatelliteMapProvider.Instance;
            }

            MapControl.Position = new PointLatLng(_missileLat, _missileLng); // _missileLat 등은 Map.cs에 있음

            // 지도 줌 설정값
            MapControl.Zoom = 8;
            MapControl.MinZoom = 8;
            MapControl.MaxZoom = 8;
            MapControl.ShowCenter = false;
            MapControl.CanDragMap = true;

            // _mapUpdateTimer 초기화는 MainWindow.Map.cs로

            // 모터 출력 타이머
            _timer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(500) };
            _timer.Tick += UpdateMotorOutputs;
            _timer.Start();

            // _glibTimer 초기화는 MainWindow.GStreamer.cs로 이동

            // _simulationTimer 초기화는 MainWindow.Map.cs로

            // 왼쪽 신호 타임아웃용 타이머 초기화
            _leftSignalTimer = new DispatcherTimer();
            _leftSignalTimer.Interval = TimeSpan.FromMilliseconds(SignalTimeoutMs);
            _leftSignalTimer.Tick += OnLeftSignalTimeout; // 이 함수는 GStreamer.cs에 추가해야 함

            // 오른쪽 신호 타임아웃용 타이머 초기화
            _rightSignalTimer = new DispatcherTimer();
            _rightSignalTimer.Interval = TimeSpan.FromMilliseconds(SignalTimeoutMs);
            _rightSignalTimer.Tick += OnRightSignalTimeout; // 이 함수는 GStreamer.cs에 추가해야 함

            // IR 추적 박스 및 HB 타임아웃 감지 타이머
            _hideTimer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(150) };
            _hideTimer.Tick += (s, e) => {
                // 1. IR Track 
                if (_trkClock.IsRunning && _trkClock.ElapsedMilliseconds > 800)
                {
                    _hasBox = false; _trkClock.Stop();
                    videoPanelLeft?.Invalidate();
                }

                // 2. ArUco 
                if (_arucoClock.IsRunning && _arucoClock.ElapsedMilliseconds > 500)
                {
                    _arucoHas = false;
                    _arucoClock.Stop();
                    videoPanelRight?.Invalidate();
                }

                // 3. HB
                if (_isHbConnected && _hbClock.ElapsedMilliseconds > SignalTimeoutMs)
                {
                    _isHbConnected = false; // 플래그를 false로
                    UpdateConnectionStatus(_isHbConnected);
                    _hbClock.Stop(); // 스톱워치 중지
                    DisableFlyStatusMid();
                    DisableFlyStatusEnd();
                    //AppendLog("[SYSTEM] HB 수신 중단 (Timeout)");
                }
            };
        }

        private void ZoomInMap()
        {
            MapControl.Zoom = 8;
            MapControl.MinZoom = 8;
            MapControl.MaxZoom = 8;
            MapControl.ShowCenter = false;
            MapControl.CanDragMap = true;
        }
        private void ZoomOutMap()
        {
            MapControl.Zoom = 8;
            MapControl.MinZoom = 8;
            MapControl.MaxZoom = 8;
            MapControl.ShowCenter = false;
            MapControl.CanDragMap = true;
        }
        // ===== 실시간 초기화 =====
        private void Window_ContentRendered(object sender, EventArgs e)
        {
            try
            {
                // WinForms Paint 이벤트 핸들러 연결
                videoPanelLeft.Paint += VideoPanelLeft_Paint;
                videoPanelRight.Paint += VideoPanelRight_Paint;

                Gst.Application.Init();

                _glibThread = new Thread(() =>
                {
                    _glibLoop = new GLib.MainLoop();
                    _glibLoop.Run();
                })
                {
                    IsBackground = true,
                    Name = "GLibMainLoop"
                };
                _glibThread.Start();

                // GStreamer/Map/UDP 초기화 함수 호출 (각 partial 파일에 정의되어 있음)
                InitializeGStreamerLeft();  // GStreamer.cs
                InitializeGStreamerRight(); // GStreamer.cs

                InitializeUdp(); // UDP.cs

                InitializeMapTimers();   // Map.cs
                CreateMissileMarker();   // Map.cs
                UpdateMapCenter();       // Map.cs
                InitializeWaypointImages(); // Map.cs

                _mapUpdateTimer?.Start(); // [NRT 수정] ? 추가

                // [추가] 3. 시나리오 UI를 초기 상태(선택)로 설정
                SetScenarioUIState(false, null);

                _hideTimer?.Start();

                _leftSignalTimer?.Start();
                _rightSignalTimer?.Start();
            }
            catch (Exception ex)
            {
                AppendLog($"[FATAL] Init 에러: {ex}");
            }

            ShowImageOverlays();

            StartMetaReceiver(); // UDP.cs

            // 메인 윈도우가 움직이거나 크기가 조절될 때 오버레이도 따라가도록 이벤트 연결
            this.LocationChanged += (s, ev) => UpdateOverlayPositions();
            this.SizeChanged += (s, ev) => UpdateOverlayPositions();
        }

        // ===== 화면 ui 오버레이 기능 =====
        private void ShowImageOverlays()
        {
            // [NRT 수정] PresentationSource가 null일 수 있으므로 체크
            var source = PresentationSource.FromVisual(this);
            if (source?.CompositionTarget == null)
            {
                AppendLog("[FATAL] PresentationSource is null. 영상 오버레이 실패");
                return;
            }
            var transform = source.CompositionTarget.TransformToDevice;

            // --- 왼쪽 오버레이
            var targetHostLeft = videoHostLeft;
            var screenPosLeft = targetHostLeft.PointToScreen(new System.Windows.Point(0, 0));

            _leftImageOverlay = new GrapicOverlayWindow
            {
                Left = screenPosLeft.X / transform.M11,
                Top = screenPosLeft.Y / transform.M22,
                Width = targetHostLeft.ActualWidth,
                Height = targetHostLeft.ActualHeight,
                Owner = this,
                IsEoOnlyMode = false
            };
            _leftImageOverlay.Show();

            // --- 오른쪽 오버레이
            var targetHostRight = videoHostRight;
            var screenPosRight = targetHostRight.PointToScreen(new System.Windows.Point(0, 0));

            _rightImageOverlay = new GrapicOverlayWindow
            {
                Left = screenPosRight.X / transform.M11,
                Top = screenPosRight.Y / transform.M22,
                Width = targetHostRight.ActualWidth,
                Height = targetHostRight.ActualHeight,
                Owner = this,
                IsEoOnlyMode = true // [추가] 오른쪽 패널은 EO 전용 모드
            };
            _rightImageOverlay.Show(); // <-- 오른쪽 창 띄우기
        }
        // ===== 화면 ui 오버레이 위치 업데이트 기능 =====
        private void UpdateOverlayPositions()
        {
            // [NRT 수정] PresentationSource가 null일 수 있으므로 체크
            var source = PresentationSource.FromVisual(this);
            if (source?.CompositionTarget == null) return;
            var transform = source.CompositionTarget.TransformToDevice;

            // --- 왼쪽 업데이트
            if (_leftImageOverlay != null)
            {
                var targetHost = videoHostLeft;
                var screenPos = targetHost.PointToScreen(new System.Windows.Point(0, 0));

                _leftImageOverlay.Left = screenPos.X / transform.M11;
                _leftImageOverlay.Top = screenPos.Y / transform.M22;
                _leftImageOverlay.Width = targetHost.ActualWidth;
                _leftImageOverlay.Height = targetHost.ActualHeight;
            }

            // --- 오른쪽 업데이트
            if (_rightImageOverlay != null)
            {
                var targetHost = videoHostRight;
                var screenPos = targetHost.PointToScreen(new System.Windows.Point(0, 0));

                _rightImageOverlay.Left = screenPos.X / transform.M11;
                _rightImageOverlay.Top = screenPos.Y / transform.M22;
                _rightImageOverlay.Width = targetHost.ActualWidth;
                _rightImageOverlay.Height = targetHost.ActualHeight;
            }
        }


        private void VideoPanelLeft_Paint(object? sender, PaintEventArgs e)
        {


            var g = e.Graphics;
            g.SmoothingMode = SmoothingMode.AntiAlias;
            g.InterpolationMode = InterpolationMode.HighQualityBicubic;
            g.PixelOffsetMode = PixelOffsetMode.Half;
            g.CompositingQuality = CompositingQuality.HighQuality;

            Bitmap? bmp = null;
            lock (_irBmpLock) { if (_irLastBmp != null) bmp = (Bitmap)_irLastBmp.Clone(); }
            if (bmp == null) return;

            var dest = new Rectangle(0, 0, videoPanelLeft.Width, videoPanelLeft.Height);
            g.DrawImage(bmp, dest, new Rectangle(0, 0, bmp.Width, bmp.Height), GraphicsUnit.Pixel);
            bmp.Dispose();

            //// [FPS 텍스트 그리기]
            //using (var font = new Font("Consolas", 12, System.Drawing.FontStyle.Bold))
            //using (var brush = new SolidBrush(Color.LimeGreen)) // 잘 보이게 라임색
            //{
            //    string fpsText = $"IR FPS: {_fpsLeft:F1}"; // 소수점 1자리까지 표시
            //    g.DrawString(fpsText, font, brush, new PointF(5, 5));
            //}

            if (_hasBox)
            {
                float sx = (float)videoPanelLeft.Width / IR_WIDTH;
                float sy = (float)videoPanelLeft.Height / IR_HEIGHT;

                // --- 1. 기존 (바깥쪽) 박스 좌표 계산 ---
                float rx = _bx * sx;
                float ry = _by * sy;
                float rw = _bw * sx;
                float rh = _bh * sy;

                // --- 2. [신규] 중심이 같고 크기가 절반인 (안쪽) 박스 좌표 계산 ---
                float new_rw = rw / 2.0f; // 너비 절반
                float new_rh = rh / 2.0f; // 높이 절반
                float new_rx = rx + (rw / 4.0f); // (바깥쪽 너비의 1/4) 만큼 x 이동
                float new_ry = ry + (rh / 4.0f); // (바깥쪽 높이의 1/4) 만큼 y 이동
                using var pen = new Pen(Color.CornflowerBlue, 5.0f) { LineJoin = LineJoin.Round };
                using var path = new GraphicsPath();

                // --- 3. 두 개의 박스를 path에 추가 ---
                //path.AddRectangle(new RectangleF(rx, ry, rw, rh)); // 기존 바깥쪽 박스
                path.AddRectangle(new RectangleF(new_rx, new_ry, new_rw, new_rh)); // [신규] 안쪽 박스


                g.DrawPath(pen, path);

            }
        }


        private void VideoPanelRight_Paint(object? sender, PaintEventArgs e)
        {
            // ── 0) 품질 옵션
            var g = e.Graphics;
            g.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.AntiAlias;
            g.InterpolationMode = System.Drawing.Drawing2D.InterpolationMode.HighQualityBicubic;
            g.PixelOffsetMode = System.Drawing.Drawing2D.PixelOffsetMode.Half;
            g.CompositingQuality = System.Drawing.Drawing2D.CompositingQuality.HighQuality;
            g.TextRenderingHint = System.Drawing.Text.TextRenderingHint.ClearTypeGridFit;

            // ── 1) 비트맵 꺼내기
            Bitmap? bmp = null;
            lock (_eoBmpLock) { if (_eoLastBmp != null) bmp = (Bitmap)_eoLastBmp.Clone(); }
            if (bmp == null) return;

            int panelW = videoPanelRight.Width;
            int panelH = videoPanelRight.Height;

            // ── 2) 영상 그리기 (비율 유지 + 레터박스 보정)
            //    패널 비율과 원본 비율이 다르면 중앙 정렬 + 여백(ox, oy) 발생
            bool PRESERVE_ASPECT = true;  // 필요시 false로 바꾸면 패널 전체에 강제 스트레치
            Rectangle dest;
            float ox = 0f, oy = 0f, drawW = panelW, drawH = panelH;

            if (PRESERVE_ASPECT && _eoSrcW > 0 && _eoSrcH > 0)
            {
                float srcAspect = (float)_eoSrcW / _eoSrcH;
                float dstAspect = (float)panelW / panelH;

                if (dstAspect > srcAspect)
                {
                    // 패널이 더 가로로 넓음 → 세로 기준 스케일, 좌우 레터박스
                    drawH = panelH;
                    drawW = drawH * srcAspect;
                    ox = (panelW - drawW) * 0.5f;
                    oy = 0f;
                }
                else
                {
                    // 패널이 더 세로로 큼 → 가로 기준 스케일, 상하 레터박스
                    drawW = panelW;
                    drawH = drawW / srcAspect;
                    ox = 0f;
                    oy = (panelH - drawH) * 0.5f;
                }
            }

            dest = new Rectangle((int)Math.Round(ox), (int)Math.Round(oy),
                                 (int)Math.Round(drawW), (int)Math.Round(drawH));

            g.DrawImage(
                bmp,
                dest,
                new Rectangle(0, 0, bmp.Width, bmp.Height),
                GraphicsUnit.Pixel
            );
            bmp.Dispose();

            //// [FPS 텍스트 그리기]
            //using (var font = new Font("Consolas", 12, System.Drawing.FontStyle.Bold))
            //using (var brush = new SolidBrush(Color.LimeGreen))
            //{
            //    string fpsText = $"EO FPS: {_fpsRight:F1}";
            //    g.DrawString(fpsText, font, brush, new PointF(5, 5));
            //}

            if (_arucoHas && _eoSrcW > 0 && _eoSrcH > 0)
            {
                // dest는 레터박스가 적용된 비디오 영역 (Rectangle)
                float sx = (float)dest.Width / _eoSrcW;
                float sy = (float)dest.Height / _eoSrcH;

                System.Drawing.PointF P(int i) =>
                    new System.Drawing.PointF(
                        ox + (float)_arucoPts[i].X * sx,
                        oy + (float)_arucoPts[i].Y * sy
                    );

                using var pen = new System.Drawing.Pen(System.Drawing.Color.Yellow, 2f)
                {
                    LineJoin = System.Drawing.Drawing2D.LineJoin.Round,
                    StartCap = System.Drawing.Drawing2D.LineCap.Round,
                    EndCap = System.Drawing.Drawing2D.LineCap.Round
                };
                using var red = new System.Drawing.SolidBrush(System.Drawing.Color.Yellow);
                using var limeBrush = new System.Drawing.SolidBrush(System.Drawing.Color.Yellow);
                using var font = new System.Drawing.Font("Consolas", 10f, System.Drawing.FontStyle.Regular);

                var p0 = P(0); var p1 = P(1); var p2 = P(2); var p3 = P(3);

                g.DrawLines(pen, new[] { p0, p1, p2, p3, p0 });

                void Dot(System.Drawing.PointF p)
                {
                    const float r = 3f;
                    g.FillEllipse(red, p.X - r, p.Y - r, 2 * r, 2 * r);
                }
                Dot(p0); Dot(p1); Dot(p2); Dot(p3);

                var labelPos = new System.Drawing.PointF(p0.X, Math.Max(0, p0.Y - 16));
                // g.DrawString($"id={_arucoId}", font, limeBrush, labelPos);
                g.DrawString($"", font, limeBrush, labelPos);
            }
        }

        // ===== 종료 정리 =====
        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {

            _hideTimer?.Stop();

            try { _glibLoop?.Quit(); } catch { }
            try { _glibThread?.Join(200); } catch { }

            _leftSignalTimer?.Stop();
            _rightSignalTimer?.Stop();

            _mapUpdateTimer?.Stop();
            _timer?.Stop();
            _simulationTimer?.Stop();

            _pipelineLeft?.SetState(State.Null);
            _pipelineLeft?.Dispose();
            _pipelineRight?.SetState(State.Null);
            _pipelineRight?.Dispose();

            _metaCts?.Cancel();
            _metaRx?.Close(); _metaRx?.Dispose();
            _cmdTx?.Close(); _cmdTx?.Dispose();

            _leftImageOverlay?.Close();
            _rightImageOverlay?.Close();

            // GDI+ 비트맵 리소스 해제
            lock (_irBmpLock)
            {
                _irLastBmp?.Dispose();
                _irLastBmp = null;
            }
            lock (_eoBmpLock)
            {
                _eoLastBmp?.Dispose();
                _eoLastBmp = null;
            }
        }



        // ====== UI 모터 샘플 ======
        private void UpdateMotorOutputs(object? sender, EventArgs e)
        {

        }

        // ===== 자폭 버튼 =====
        private void ST_Click(object sender, RoutedEventArgs e)
        {
            // [수정] 1. 토글 스위치(ArmSelfDestructToggle)가 켜져 있는지 확인
            if (ArmSelfDestructToggle.IsChecked == true)
            {
                SendCmd("SELF_DESTRUCT"); // (UDP.cs에 정의)
                AppendLog($"[{SysDateTime.Now:HH:mm:ss}] 자폭 실행");
                DisableFlyStatusEnd();

                // [추가] 사용 후 즉시 토글을 끔 (안전을 위해)
                ArmSelfDestructToggle.IsChecked = false;
                StopSimulation(); // (Map.cs에 정의)
            }
            else
            {
                AppendLog("[WARN] 자폭 실패: 안전 스위치(OFF)가 잠겨있습니다.");
            }
        }

        // ===== 목표 지정 버튼 =====
        private void DES_Click(object sender, RoutedEventArgs e)
        {
            if (desMode)
            {
                desMode = false;
                _currentOverlay?.Close();
                _currentOverlay = null;
                //AppendLog($"[{SysDateTime.Now:HH:mm:ss}] [DES] 비활성화");
                DES.Style = (Style)FindResource("ClickableButtonStyle");
                return;
            }

            desMode = true;

            DES.Style = (Style)FindResource("ClickableButtonActiveStyle");

            var targetHost = videoHostLeft;
            var screenPos = targetHost.PointToScreen(new WpfPoint(0, 0));
            var source = PresentationSource.FromVisual(this);
            // [NRT 수정] source null 체크
            if (source?.CompositionTarget == null) return;
            var transform = source.CompositionTarget.TransformToDevice;

            //AppendLog($"[{SysDateTime.Now:HH:mm:ss}] [DES] 활성화");

            _currentOverlay = new SubOverlayWindow
            {
                Left = screenPos.X / transform.M11,
                Top = screenPos.Y / transform.M22,
                Width = targetHost.ActualWidth,
                Height = targetHost.ActualHeight,
                Owner = this
            };

            _currentOverlay.OnCanvasClick += (pt) =>
            {
                // 1. =========[수정] 충돌 방지를 위해 try-catch 블록 추가 =========
                try
                {
                    // [NRT 수정] _currentOverlay null 체크
                    if (_currentOverlay == null) return;

                    // 2. =========[수정] 0으로 나누기 오류 방지 =========
                    // 오버레이 창의 크기가 0이면 계산을 중단합니다.
                    if (_currentOverlay.Width == 0 || _currentOverlay.Height == 0)
                    {
                        AppendLog("[ERROR] [DES] 오버레이 크기가 0입니다. 클릭을 처리할 수 없습니다.");
                        return;
                    }

                    double nx = pt.X / _currentOverlay.Width;
                    double ny = pt.Y / _currentOverlay.Height;

                    int ix = (int)Math.Round(nx * (IR_WIDTH - 1));
                    int iy = (int)Math.Round(ny * (IR_HEIGHT - 1));
                    ix = Math.Max(0, Math.Min(IR_WIDTH - 1, ix));
                    iy = Math.Max(0, Math.Min(IR_HEIGHT - 1, iy));

                    // 3. =========[수정] 로그 주석 해제 (작동 확인용) =========
                    AppendLog($"[{System.DateTime.Now:HH:mm:ss}] ({ix}, {iy}) 위치의 표적 추적을 시도합니다");

                    // SendClickBinary (UDP.cs) 또는 FindResource 에서 오류가 발생할 가능성이 높습니다.
                    SendClickBinary((float)ix, (float)iy); // (UDP.cs에 정의)

                    desMode = false;
                    _currentOverlay.Close();
                    _currentOverlay = null;

                    DES.Style = (Style)FindResource("ClickableButtonStyle");
                }
                // 1. =========[수정] 예외 발생 시 로그 기록 =========
                catch (Exception ex)
                {
                    // 오류가 발생해도 프로그램이 튕기지 않고, 여기에 로그를 남깁니다.
                    AppendLog($"[FATAL] [DES] 클릭 처리 중 심각한 오류 발생: {ex.Message}");
                    AppendLog($"[FATAL] StackTrace: {ex.StackTrace}"); // 오류의 상세 위치

                    // 오류 발생 시에도 안전하게 모드 해제
                    try
                    {
                        desMode = false;
                        _currentOverlay?.Close();
                        _currentOverlay = null;
                        DES.Style = (Style)FindResource("ClickableButtonStyle");
                    }
                    catch { /* 무시 */ }
                }
            };

            _currentOverlay.Closed += (s, ev) =>
            {
                if (desMode)
                {
                    desMode = false;
                    //AppendLog($"[{SysDateTime.Now:HH:mm:ss}] [DES] 비활성화 (창 닫힘)");
                }
                _currentOverlay = null;
            };

            _currentOverlay.Show();
        }

        // ===== 필터 버튼 =====
        private void SD_Click(object sender, RoutedEventArgs e)
        {
            AppendLog($"[{System.DateTime.Now:HH:mm:ss}] 필터 전환");
            temp++;
            int newIndex = temp % 4;
            if (_isLeftSignalActive) _leftImageOverlay?.UpdateImage(newIndex);
            if (_isRightSignalActive) _rightImageOverlay?.UpdateImage(newIndex);
        }

        // ====== 로그 출력 ======
        private void AppendLog(string text)
        {
            lock (_logBuf) { _logBuf.AppendLine(text); }

            long now = NowMs();
            if (now - _lastFlushMs < 100) return; // 100ms 쓰로틀링 (이것도 아주 중요함)
            _lastFlushMs = now;

            _ = Dispatcher.BeginInvoke(new Action(() =>
            {
                string dump;
                lock (_logBuf) { dump = _logBuf.ToString(); _logBuf.Clear(); }

                // 1. 텍스트 추가
                LogBox.Text += dump;

                // 2. [핵심] 길이 제한 (메모리 폭주 방지)
                const int MaxLogLen = 3000; // 약 3000자 정도만 유지
                if (LogBox.Text.Length > MaxLogLen)
                {
                    // 뒤에서부터 MaxLogLen 만큼만 남기고 자름
                    LogBox.Text = LogBox.Text.Substring(LogBox.Text.Length - MaxLogLen);
                }

                LogScrollViewer.ScrollToEnd();
            }), DispatcherPriority.Background);
        }
        private void ThrustBar_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e) { }

        // ====== [추가] 시나리오 버튼 클릭 이벤트 ======
        private void Scenario1_Click(object sender, RoutedEventArgs e)
        {
            _selectedScenarioId = 1;
            StartSimulation(_selectedScenarioId);
            SetScenarioUIState(true, "시나리오 1 시작 대기");
            StartMissionButton.Visibility = Visibility.Visible;
            AbortMissionButton.Visibility = Visibility.Collapsed;
        }

        private void Scenario2_Click(object sender, RoutedEventArgs e)
        {
            _selectedScenarioId = 2;
            StartSimulation(_selectedScenarioId);
            SetScenarioUIState(true, "시나리오 2 시작 대기");
            StartMissionButton.Visibility = Visibility.Visible;
            AbortMissionButton.Visibility = Visibility.Collapsed;

        }

        private void Scenario3_Click(object sender, RoutedEventArgs e)
        {
            _selectedScenarioId = 3;
            StartSimulation(_selectedScenarioId);
            SetScenarioUIState(true, "시나리오 3 시작 대기");
            StartMissionButton.Visibility = Visibility.Visible;
            AbortMissionButton.Visibility = Visibility.Collapsed;
        }

        private void AbortMission_Click(object sender, RoutedEventArgs e)
        {
            AppendLog("[WARN] 임무 수동 종료.");
            ZoomInMap();
            StopSimulation(); // 3. Map.cs의 시뮬레이션 종료
            // SetScenarioUIState(false, null); // [주석] StopSimulation이 내부적으로 SetScenarioUIState를 호출합니다.
        }

        private void StartMission_Click(object sender, RoutedEventArgs e)
        {
            ExecuteSimulationStart(); // 분리했던 '시작' 로직 호출
            ZoomInMap();
            // 버튼 상태 변경 (시작 -> 종료)
            StartMissionButton.Visibility = Visibility.Collapsed;
            AbortMissionButton.Visibility = Visibility.Visible;

            if (MissionDetailsText != null)
            {
                MissionDetailsText.Text = $"시나리오 {_selectedScenarioId} 진행 중...";
            }
        }
        public void SetScenarioUIState(bool isRunning, string missionText)
        {
            if (isRunning)
            {
                ZoomInMap();
                ScenarioSelectionGrid.Visibility = Visibility.Collapsed;
                ScenarioInProgressGrid.Visibility = Visibility.Visible;
                MissionDetailsText.Text = missionText;
            }
            else
            {
                ZoomOutMap();
                ScenarioSelectionGrid.Visibility = Visibility.Visible;
                ScenarioInProgressGrid.Visibility = Visibility.Collapsed;
                _selectedScenarioId = 0;

                StartMissionButton.Visibility = Visibility.Visible;
                AbortMissionButton.Visibility = Visibility.Collapsed;
            }
        }

        // ====== [추가] 4. 통신 상태 UI 제어 함수 ======

        /// <summary>
        /// (4) (모든 스레드에서 호출 가능) 좌측 상단의 통신 상태 표시기를 업데이트합니다.
        /// </summary>
        /// <param name="status">표시할 상태 (Good, Warning, Disconnected)</param>
        public void UpdateConnectionStatus(bool status)
        {
            // GStreamer나 UDP 스레드 등 다른 스레드에서 호출될 수 있으므로
            // 반드시 Dispatcher.Invoke를 사용하여 UI 스레드에서 실행합니다.
            // status = 0(통신 끊김) 1(불안정) 2(안정)

            Dispatcher.Invoke(() =>
            {
                if (StatusIndicatorCircle_1 == null || StatusIndicatorText_1 == null) return;

                if (status)
                {
                    StatusIndicatorCircle_1.Fill = Media.Brushes.GreenYellow;
                    //StatusIndicatorText_1.Text = "안정";
                }
                else
                {
                    StatusIndicatorCircle_1.Fill = Media.Brushes.Red;
                    //StatusIndicatorText_1.Text = "연결 끊김";
                }
                //switch (status) {
                //    case 2:
                //        StatusIndicatorCircle_1.Fill = Media.Brushes.GreenYellow;
                //        StatusIndicatorText_1.Text = "안정";
                //        break;
                //    case 1:
                //        StatusIndicatorCircle_1.Fill = Media.Brushes.Orange;
                //        StatusIndicatorText_1.Text = "불안정";
                //        break;

                //    case 0:
                //        StatusIndicatorCircle_1.Fill = Media.Brushes.Red;
                //        StatusIndicatorText_1.Text = "연결 끊김";
                //        break;
                //    default:
                //        StatusIndicatorCircle_1.Fill = Media.Brushes.Red;
                //        StatusIndicatorText_1.Text = "통신 인자 전달 실패";
                //        break;
                //}
            });
        }

        public void DisableFlyStatusMid()
        {
            Dispatcher.Invoke(() =>
            {
                if (FlyStatusIndicatorCircle_Mid == null || FlyStatusIndicatorText_Mid == null) return;
                FlyStatusIndicatorCircle_Mid.Fill = Media.Brushes.Red;

            });
        }
        public void EnableFlyStatusMid()
        {
            Dispatcher.Invoke(() =>
            {
                if (FlyStatusIndicatorCircle_Mid == null || FlyStatusIndicatorText_Mid == null) return;
                FlyStatusIndicatorCircle_Mid.Fill = Media.Brushes.GreenYellow;

            });
        }
        public void DisableFlyStatusEnd()
        {
            Dispatcher.Invoke(() =>
            {
                if (FlyStatusIndicatorCircle_End == null || FlyStatusIndicatorText_End == null) return;
                FlyStatusIndicatorCircle_End.Fill = Media.Brushes.Red;

            });
        }
        public void EnableFlyStatusEnd()
        {
            Dispatcher.Invoke(() =>
            {
                if (FlyStatusIndicatorCircle_End == null || FlyStatusIndicatorText_End == null) return;
                FlyStatusIndicatorCircle_End.Fill = Media.Brushes.GreenYellow;

            });
        }

        // ====== [추가] 실시간 UI 업데이트 함수 (모터/스크롤) ======

        /// <summary>
        /// (모든 스레드에서 호출 가능) 4개의 모터 텍스트 값을 실시간으로 업데이트합니다.
        /// (예: Zynq 통신 스레드에서 UpdateMotorValues("531", "50", "49", "51"); 호출)
        /// </summary>
        public void UpdateMotorValues(string m1, string m2, string m3, string m4)
        {
            // 다른 스레드(UDP, GStreamer 등)에서 호출될 수 있으므로,
            // UI 스레드에서 실행되도록 Dispatcher.Invoke를 사용합니다.
            Dispatcher.Invoke(() =>
            {
                MotorValue1.Text = m1;
                MotorValue2.Text = m2;
                MotorValue3.Text = m3;
                MotorValue4.Text = m4;
            });
        }

        /// <summary>
        /// (모든 스레드에서 호출 가능) 2개의 스크롤 바 값을 실시간으로 업데이트합니다. (값 범위: 0-100)
        /// (예: Zynq 통신 스레드에서 UpdateScrollBars(75.5, 30.0); 호출)
        /// </summary>
        public void UpdateScrollBars(double scroll1, double scroll2)
        {
            Dispatcher.Invoke(() =>
            {
                Scroll_1Bar.Value = scroll1; // XAML의 이름(Scroll_1Bar)을 따라야 함
                Scroll_2Bar.Value = scroll2; // XAML의 이름(Scroll_2Bar)을 따라야 함
            });
        }

        // ===== 키 입력 처리 =====
        private void Window_KeyDown(object sender, System.Windows.Input.KeyEventArgs e)
        {
            bool positionChanged = false;

            if (_simulationRunning)
            {
                // (시뮬레이션 중에는 WASD 안 먹힘)
            }
            else
            {
                switch (e.Key)
                {
                    case Key.W:
                        _missileLat += _movementStep;
                        positionChanged = true;
                        break;
                    case Key.S:
                        _missileLat -= _movementStep;
                        positionChanged = true;
                        break;
                    case Key.A:
                        _missileLng -= _movementStep;
                        positionChanged = true;
                        break;
                    case Key.D:
                        _missileLng += _movementStep;
                        positionChanged = true;
                        break;
                }
            }

            if (e.Key == Key.Q)
            {
                if (!_simulationRunning)
                {
                    Scenario1_Click(sender, e); // (Map.cs에 정의)
                }
                else
                {
                    AbortMission_Click(sender, e); // (Map.cs에 정의)
                }
            }

            if (positionChanged)
            {
                UpdateMissilePosition(_missileLat, _missileLng); // (Map.cs에 정의)
                UpdateMapCenter(); // (Map.cs에 정의)
                AppendLog($"[DEBUG] 방향 : {e.Key}, 위치 : ({_missileLat:F4}, {_missileLng:F4})");
            }
        }
    }
}