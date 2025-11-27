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
using System.Text.RegularExpressions;
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
using SWC = System.Windows.Controls;
using SysDateTime = System.DateTime;
// === 충돌 방지 별칭들 ===
using Task = System.Threading.Tasks.Task;
using WF = System.Windows.Forms;
using WpfPoint = System.Windows.Point;

namespace TheSSENS
{
    /// <summary>
    /// 지도(GMap.NET) + 웨이포인트 + 외부 "남은 거리" 비율 반영 로직
    ///  - 자동 이동 제거 (외부 값만으로 위치 갱신)
    ///  - 남은거리=0 → 다음 웨이포인트로 자동 전환
    ///  - 스톱워치/HUD 타이머는 유지(시작/정지)
    /// </summary>
    public partial class MainWindow : System.Windows.Window
    {
        // 아르코 id 관련 신호
        private bool arriveflag = false;
        private uint prv_id = 0;
        private float Max_h = 100;
        private const double TerminalThreshold = 100;   // 45 아래는 고정 속도 구간

        private bool _terminalPhaseActive = false;      // 종말 구간(<=35) 진입 여부
        private double _virtualRemaining = 0.0;         // 종말 구간에서 사용하는 가상 남은거리

        private double _terminalDurationSec = 3.0;      // 35 → 0 까지 걸리는 시간 (튜닝용)
        private SysDateTime _terminalStartTime;         // 종말 구간 시작 시각
        private PointLatLng _terminalFromPos;           // 종말 구간 시작 좌표(거리=35)
        private PointLatLng _terminalToPos;             // 종말 구간 목표 좌표(WP)
        private uint _terminalId;                       // 종말 구간 시작 시점의 now_id

        string baseDir = AppDomain.CurrentDomain.BaseDirectory;
        // ===== 지도/마커 =====
        private GMapMarker? _missileMarker;
        private GMapRoute? _flightPathLine;
        private DispatcherTimer? _mapUpdateTimer; // (필요 시 사용할 여지만 남김)

        // ===== 위치 =====
        private readonly PointLatLng _manualStartPosition = new PointLatLng(36.1, 127.8); // 수동 기본 위치
        private double _initialMissileLat = 36.1; // 시나리오 시작 위치
        private double _initialMissileLng = 127.8;
        private double _missileLat = 36.1;
        private double _missileLng = 127.8;

        // ===== 웨이포인트/아이콘 =====
        private List<PointLatLng>? _waypoints;               // 웨이포인트 좌표
        private List<GMapMarker>? _waypointMarkers;         // 웨이포인트 마커
        private List<BitmapImage>? _waypointOriginalImages;  // 숫자 아이콘(컬러)
        private List<BitmapSource>? _waypointGrayImages;     // 숫자 아이콘(회색)
        private List<GMapMarker>? _enemyAAMarkers;          // 방공망 아이콘

        private BitmapImage? _finalTargetIdleImg;    // 표적 아이콘
        private BitmapImage? _finalTargetArrivedImg; // 파괴된 표적 아이콘

        private int _currentWaypointIndex = 0;               // 현재 목표 WP 인덱스 (0..N-1)

        // ===== 외부 "남은 거리" 스케일 (각 leg의 총 외부값)
        //  - leg: [Start->WP1], [WP1->WP2], ... 총 웨이포인트 수와 동일
        private List<double> _legExternalTotal = new();

        // ===== 타이머/보조 =====
        private Stopwatch? _missionStopwatch;   // 경과 시간 기록
        private DispatcherTimer? _simulationTimer; // HUD 갱신용
        private bool _simulationRunning = false;   // (다른 partial에서 조회)
        private const double _movementStep = 0.01; // (다른 partial에서 조회)

        // === 시나리오별 외부 스케일만 유지 (필요한 값만 남김) ===
        private static readonly Dictionary<int, double[]> SCENARIO_EXT_TOTAL = new()
        {
            // 예: WP가 3개면 leg=3개 (Start->1, 1->2, 2->3)
            { 1, new double[] { 100, 100, 100, 100 } },
            { 2, new double[] { 100, 100, 100, 100 } },
            { 3, new double[] { 100, 100, 100 } },
        };

        // ===== 유틸 =====
        private static BitmapSource? ToGray(BitmapSource? src)
        {
            if (src == null) return null;
            var gray = new FormatConvertedBitmap();
            gray.BeginInit();
            gray.Source = src;
            gray.DestinationFormat = PixelFormats.Gray8;
            gray.EndInit();
            gray.Freeze();
            return gray;
        }

        // ===== 초기화 =====
        public void InitializeMapTimers()
        {
            _missionStopwatch = new Stopwatch();

            // HUD(경과시간)만 갱신 — 이동 로직 없음
            _simulationTimer = new DispatcherTimer
            {
                Interval = TimeSpan.FromMilliseconds(100)
            };
            _simulationTimer.Tick += OnSimulationLoopTick;
        }

        // ===== 미사일 마커 생성 =====
        private void CreateMissileMarker()
        {
            var initialPosition = new PointLatLng(_missileLat, _missileLng);
            _missileMarker = new GMapMarker(initialPosition);

            string iconPath = baseDir + "/Images/missile_icon.png";
            try
            {
                if (File.Exists(iconPath))
                {
                    var iconSource = new BitmapImage(new global::System.Uri(iconPath));
                    iconSource.Freeze();

                    var img = new System.Windows.Controls.Image
                    {
                        Width = 50,
                        Height = 50,
                        Stretch = Media.Stretch.Uniform,
                        Source = iconSource,
                        RenderTransformOrigin = new System.Windows.Point(0.5, 0.5),
                    };

                    var textBorder = new Border
                    {
                        Background = new SolidColorBrush(Media.Color.FromArgb(100, 0, 0, 0)),
                        CornerRadius = new CornerRadius(2),
                        Padding = new Thickness(1, 1, 1, 1),
                        Margin = new Thickness(0, 1, 0, 0)
                    };
                    var text = new TextBlock
                    {
                        Text = "AGM-850",
                        Foreground = Media.Brushes.White,
                        FontFamily = new Media.FontFamily("Consolas"),
                        FontSize = 12
                    };
                    textBorder.Child = text;

                    var stack = new StackPanel
                    {
                        Orientation = SWC.Orientation.Vertical,
                        Children = { img, textBorder }
                    };

                    _missileMarker.Shape = stack;
                    stack.Loaded += (s, e) =>
                    {
                        if (_missileMarker != null)
                            _missileMarker.Offset = new System.Windows.Point(-stack.ActualWidth / 2, -stack.ActualHeight / 2);
                    };
                }
                else
                {
                    _missileMarker.Shape = new TextBlock
                    {
                        Text = "AGM-850",
                        FontSize = 24,
                        Foreground = Media.Brushes.White,
                        ToolTip = "Icon file missing"
                    };
                }
            }
            catch (Exception ex)
            {
                AppendLog($"[ERROR] 미사일 아이콘 로드 실패: {ex.Message}");
                _missileMarker.Shape = new TextBlock
                {
                    Text = "x",
                    FontSize = 24,
                    Foreground = Media.Brushes.Black,
                    ToolTip = $"Icon load failed: {ex.Message}"
                };
            }

            MapControl.Markers.Add(_missileMarker);
            //AppendLog($"[INFO] 미사일 아이콘 배치: ({_missileLat:F4}, {_missileLng:F4})");
        }

        // ===== 지도 카메라 =====
        private void UpdateMapCenter()
        {
            if (_missileMarker != null)
                MapControl.Position = _missileMarker.Position;
        }

        // ===== 아이콘 위치 갱신 =====
        public void UpdateMissilePosition(double newLat, double newLng)
        {
            if (_missileMarker == null) return;
            _missileMarker.Position = new PointLatLng(newLat, newLng);
        }

        // ===== 웨이포인트 이미지/마커 =====
        private void InitializeWaypointImages()
        {
            _waypointOriginalImages = new List<BitmapImage>();
            _waypointGrayImages = new List<BitmapSource?>();

            string wp1 = baseDir + "/Images/circle_num_1.png";
            string wp2 = baseDir + "/Images/circle_num_2.png";
            string wp3 = baseDir + "/Images/circle_num_3.png";
            string wp4 = baseDir + "/Images/circle_num_4.png";
            string wp5 = baseDir + "/Images/circle_num_5.png";

            string g1 = baseDir + "/Images/circle_num_1_gray.png";
            string g2 = baseDir + "/Images/circle_num_2_gray.png";
            string g3 = baseDir + "/Images/circle_num_3_gray.png";
            string g4 = baseDir + "/Images/circle_num_4_gray.png";
            string g5 = baseDir + "/Images/circle_num_5_gray.png";

            string finalIdlePath = baseDir + "/Images/Hostile_Supply.png";
            string finalArrivedPath = baseDir + "/Images/Hostile_Supply_destory.png";

            _finalTargetIdleImg = LoadBitmap(finalIdlePath);
            _finalTargetArrivedImg = LoadBitmap(finalArrivedPath);

            string[] color = { wp1, wp2, wp3, wp4, wp5 };
            string[] gray = { g1, g2, g3, g4, g5 };


            for (int i = 0; i < color.Length; i++)
            {
                try
                {
                    if (!File.Exists(color[i]))
                        throw new FileNotFoundException($"Waypoint icon not found: {color[i]}");

                    var bmp = new BitmapImage(new global::System.Uri(color[i], UriKind.Absolute));
                    bmp.Freeze();
                    _waypointOriginalImages.Add(bmp);

                    BitmapSource? gsrc = null;
                    if (File.Exists(gray[i]))
                    {
                        var gimg = new BitmapImage(new global::System.Uri(gray[i], UriKind.Absolute));
                        gimg.Freeze();
                        gsrc = gimg;
                    }
                    else
                    {
                        gsrc = ToGray(bmp);
                    }
                    _waypointGrayImages.Add(gsrc!);
                }
                catch (Exception ex)
                {
                    AppendLog($"[ERROR] 웨이포인트 아이콘 로드 실패: {ex.Message}");
                }
            }

            _flightPathLine = null;
        }

        private static BitmapImage? LoadBitmap(string absPath)
        {
            try
            {
                if (!File.Exists(absPath)) return null;
                var bmp = new BitmapImage(new global::System.Uri(absPath, UriKind.Absolute));
                bmp.Freeze();
                return bmp;
            }
            catch { return null; }
        }

        private void InitializeWaypointMarkers()
        {
            if (_waypoints == null || _waypointOriginalImages == null) return;

            _waypointMarkers = new List<GMapMarker>();
            for (int i = 0; i < _waypoints.Count; i++)
            {
                var marker = new GMapMarker(_waypoints[i]);

                // 기본은 숫자 아이콘
                BitmapSource bmp = _waypointOriginalImages[Math.Min(i, _waypointOriginalImages.Count - 1)];

                // === [추가] 마지막 표적이면 대기(진행중) 아이콘으로 대체 ===
                bool isFinal = (i == _waypoints.Count - 1);
                if (isFinal && _finalTargetIdleImg != null)
                    bmp = _finalTargetIdleImg;

                var img = new System.Windows.Controls.Image
                {
                    Width = 40,
                    Height = 40,
                    Source = bmp,
                    ToolTip = isFinal ? "최종 표적" : $"Waypoint {i + 1}"
                };

                marker.Shape = img;
                marker.Offset = new WpfPoint(-20, -20);
                _waypointMarkers.Add(marker);
                MapControl.Markers.Add(marker);
            }

            if (_enemyAAMarkers != null)
                foreach (var aa in _enemyAAMarkers) MapControl.Markers.Add(aa);
        }

        private GMapMarker CreateAAMarker(PointLatLng pos, string name)
        {
            var marker = new GMapMarker(pos);
            string aaIcon = baseDir + "/Images/AA_Icon.png";

            try
            {
                var bmp = new BitmapImage(new global::System.Uri(aaIcon, UriKind.Absolute));
                bmp.Freeze();
                marker.Shape = new System.Windows.Controls.Image
                {
                    Width = 40,
                    Height = 40,
                    Source = bmp,
                    ToolTip = name
                };
                marker.Offset = new WpfPoint(0, 0);
            }
            catch
            {
                marker.Shape = new TextBlock
                {
                    Text = "x",
                    Foreground = Media.Brushes.Red,
                    Background = Media.Brushes.Black,
                    FontWeight = FontWeights.Bold,
                    FontSize = 16,
                    ToolTip = $"{name} (아이콘 없음)"
                };
                marker.Offset = new WpfPoint(0, 0);
            }
            return marker;
        }

        private void ClearScenarioMarkers()
        {
            if (_waypointMarkers != null)
            {
                foreach (var m in _waypointMarkers) MapControl.Markers.Remove(m);
                _waypointMarkers.Clear();
            }
            if (_enemyAAMarkers != null)
            {
                foreach (var m in _enemyAAMarkers) MapControl.Markers.Remove(m);
                _enemyAAMarkers.Clear();
            }
        }

        // ===== 시나리오 로드 =====
        private void LoadScenario(int scenarioId)
        {
            ClearScenarioMarkers();

            _waypoints = new List<PointLatLng>();
            _enemyAAMarkers = new List<GMapMarker>();
            _currentWaypointIndex = 0;

            switch (scenarioId)
            {
                case 1:

                    _initialMissileLat = 37.02278; _initialMissileLng = 126.38667;
                    _waypoints.Add(ParseDmsPoint("37°17'31\"N 126°05'31\"E"));
                    _waypoints.Add(ParseDmsPoint("37°39'56\"N 125°42'03\"E"));
                    _waypoints.Add(ParseDmsPoint("37°57'29\"N 126°02'41\"E"));
                    _waypoints.Add(ParseDmsPoint("37°59'51\"N 126°27'06\"E"));
                    _enemyAAMarkers.Add(CreateAAMarker(new PointLatLng(37.8475, 126.03611), "SA-3"));
                    break;

                case 2:
                    _initialMissileLat = 38.25722; _initialMissileLng = 128.48222;
                    _waypoints.Add(ParseDmsPoint("38°41'55\"N 129°16'54\"E"));
                    _waypoints.Add(ParseDmsPoint("39°21'27\"N 129°06'15\"E"));
                    _waypoints.Add(ParseDmsPoint("40°04'32\"N 128°34'44\"E"));
                    _waypoints.Add(ParseDmsPoint("40°13'00\"N 128°18'56\"E"));
                    break;

                case 3:
                    _initialMissileLat = 38.16583; _initialMissileLng = 127.15972;
                    _waypoints.Add(ParseDmsPoint("38°11'16\"N 126°08'30\"E"));
                    _waypoints.Add(ParseDmsPoint("38°30'34\"N 126°51'04\"E"));
                    _waypoints.Add(ParseDmsPoint("38°56'58\"N 125°58'33\"E"));
                    _enemyAAMarkers.Add(CreateAAMarker(new PointLatLng(38.7, 127), "SA-3"));
                    _enemyAAMarkers.Add(CreateAAMarker(new PointLatLng(38.9, 126), "SA-2"));
                    break;
            }

            // 미사일 위치 초기화
            _missileLat = _initialMissileLat;
            _missileLng = _initialMissileLng;
            UpdateMissilePosition(_missileLat, _missileLng);
            UpdateMapCenter();

            InitializeWaypointMarkers();

            // 외부 스케일 테이블 주입
            _legExternalTotal = new List<double>(
                SCENARIO_EXT_TOTAL.TryGetValue(scenarioId, out var ext)
                ? ext
                : global::System.Array.Empty<double>());

            // leg 수 보정 (부족분은 1로 채움)
            int legCount = (_waypoints?.Count ?? 0);
            while (_legExternalTotal.Count < legCount) _legExternalTotal.Add(1);
            if (_legExternalTotal.Count > legCount) _legExternalTotal.RemoveRange(legCount, _legExternalTotal.Count - legCount);

            //AppendLog($"[INFO] 시나리오 {scenarioId} 로드: legs={legCount}, 외부총길이=[{string.Join(",", _legExternalTotal)}]");

            UpdateFlightPathLine(); // 초기 경로(아이콘→현재 목표)
        }

        // ===== GPS 좌표계 분석 함수 =====
        public static (double lat, double lng) ParseDmsToLatLng(string dmsPair)
        {
            var parts = dmsPair.Split(
                new[] { ' ' },
                StringSplitOptions.RemoveEmptyEntries
            );

            if (parts.Length != 2)
                throw new ArgumentException($"DMS 형식 오류: {dmsPair}");

            double lat = DmsToDecimal(parts[0]);
            double lng = DmsToDecimal(parts[1]);

            return (lat, lng);
        }

        // ===== GPS 좌표계 -> 위도경도 함수 =====
        private static double DmsToDecimal(string dms)
        {
            dms = dms.Trim();

            var match = Regex.Match(
                dms,
                @"(?<deg>\d+)[°º]\s*(?<min>\d+)'?\s*(?<sec>\d+(\.\d+)?)""?\s*(?<dir>[NSEW])",
                RegexOptions.IgnoreCase
            );

            if (!match.Success)
                throw new ArgumentException($"잘못된 DMS 문자열: {dms}");

            double deg = double.Parse(match.Groups["deg"].Value);
            double min = double.Parse(match.Groups["min"].Value);
            double sec = double.Parse(match.Groups["sec"].Value);
            char dir = char.ToUpper(match.Groups["dir"].Value[0]);

            double value = deg + min / 60.0 + sec / 3600.0;

            if (dir == 'S' || dir == 'W')
                value *= -1;

            return value;
        }

        public static PointLatLng ParseDmsPoint(string dmsPair)
        {
            var (lat, lng) = ParseDmsToLatLng(dmsPair);
            return new PointLatLng(lat, lng);
        }


        // ===== 좌표 생성 =====
        private PointLatLng GetLegStartPoint(int legIndex)
        {
            if (legIndex == 0) return new PointLatLng(_initialMissileLat, _initialMissileLng);
            return _waypoints![legIndex - 1];
        }

        // ===== 외부 남은거리 반영 (핵심) =====
        /// <summary>
        /// 외부에서 들어온 "현재 레그의 남은 거리(외부단위)"로 아이콘 위치를 직선 보간.
        /// 남은거리 <= 0 → 웨이포인트 도달 처리.
        /// </summary>
        public void ApplyExternalRange(float externalRemaining)
        {
            if (_waypoints == null || _missileMarker == null) return;
            if (_currentWaypointIndex >= _waypoints.Count) return;
            if (arriveflag && prv_id == now_id) return;

            // ID 바뀌면 Max_h 갱신 + 종말 구간 강제 종료
            if (prv_id != now_id)
            {
                Max_h = externalRemaining;
                _terminalPhaseActive = false;
            }

            arriveflag = false;
            prv_id = now_id;

            int leg = _currentWaypointIndex;
            double extTotal = Math.Max(1e-6, Max_h);

            // 이미 종말 구간(35~0) 진행 중이면 외부값은 무시
            if (_terminalPhaseActive)
            {
                // 실제 위치 갱신은 타이머에서 UpdateTerminalApproach()가 한다.
                return;
            }

            //  35 이하 → 종말 구간 시작 트리거 후 종료
            if (externalRemaining <= TerminalThreshold)
            {
                StartTerminalApproach(extTotal);
                return;
            }

            // ===== 여기부터는 기존 "정상 구간(>35)" 보간 로직 =====
            double r = externalRemaining / extTotal;
            if (double.IsNaN(r) || double.IsInfinity(r)) r = 1.0;
            r = Math.Clamp(r, 0.0, 1.0);
            double p = 1.0 - r; // 0.0=출발, 1.0=도착

            var from = GetLegStartPoint(leg);
            var to = _waypoints[leg];

            double newLat = from.Lat + (to.Lat - from.Lat) * p;
            double newLng = from.Lng + (to.Lng - from.Lng) * p;

            UpdateMissilePosition(newLat, newLng);
            UpdateMapCenter();
            UpdateFlightPathLine();
        }

        // ===== 마지막 자동 유도 ui =====
        public void FinalHoming(double finaltime)
        {
            if (_waypoints == null || _waypoints.Count == 0) return;
            if (_missileMarker == null) return;

            // 목표는 "마지막 웨이포인트"
            int finalIndex = _waypoints.Count - 1;
            var finalTarget = _waypoints[finalIndex];

            var fromPos = _missileMarker.Position;

            _currentWaypointIndex = finalIndex;

            _terminalFromPos = fromPos;
            _terminalToPos = finalTarget;
            _terminalStartTime = SysDateTime.Now;
            _terminalDurationSec = finaltime;
            _terminalId = now_id;
            _terminalPhaseActive = true;

            UpdateMissilePosition(fromPos.Lat, fromPos.Lng);
            UpdateMapCenter();
            UpdateFlightPathLine();
        }

        /// <summary>
        /// 종말 구간(거리 35 → 0) 시간 기반 보간 시작.
        /// extTotal: 현재 leg의 전체 외부 거리(Max_h 기반)
        /// </summary>
        private void StartTerminalApproach(double extTotal)
        {
            if (_waypoints == null) return;
            if (_currentWaypointIndex >= _waypoints.Count) return;

            int leg = _currentWaypointIndex;

            var from = GetLegStartPoint(leg);
            var to = _waypoints[leg];

            // "남은거리 = 35" 시점의 진행비율 p35 계산
            double r35 = TerminalThreshold / extTotal;     // 남은비율
            r35 = Math.Clamp(r35, 0.0, 1.0);
            double p35 = 1.0 - r35;                        // 진행비율

            double lat35 = from.Lat + (to.Lat - from.Lat) * p35;
            double lng35 = from.Lng + (to.Lng - from.Lng) * p35;

            // 종말 구간 시작/끝 좌표 저장
            _terminalFromPos = new PointLatLng(lat35, lng35);
            _terminalToPos = to;
            _terminalStartTime = SysDateTime.Now;
            _terminalId = now_id;          // 이때의 ID 기억
            _terminalPhaseActive = true;

            // 시작할 때 바로 "거리 35 지점"으로 스냅
            UpdateMissilePosition(lat35, lng35);
            UpdateMapCenter();
            UpdateFlightPathLine();
        }

        /// <summary>
        /// 종말 구간(35~0)을 시간 기준으로 진행.
        /// - _terminalDurationSec 동안 From → To 직선 보간
        /// - now_id가 바뀌면 중간에 즉시 탈출
        /// - 끝까지 가면 도착 처리 + 다음 웨이포인트 전환
        /// </summary>
        private void UpdateTerminalApproach()
        {
            if (!_terminalPhaseActive) return;
            if (_waypoints == null) return;
            if (_currentWaypointIndex >= _waypoints.Count) return;

            // 새 ID가 잡히면 즉시 탈출
            if (now_id != _terminalId)
            {
                _terminalPhaseActive = false;
                return;
            }

            double elapsedSec = (SysDateTime.Now - _terminalStartTime).TotalSeconds;
            double t = elapsedSec / _terminalDurationSec;  // 0 ~ 1

            if (t >= 1.0)
            {
                // 0까지 도달 → 웨이포인트 도착 처리
                UpdateMissilePosition(_terminalToPos.Lat, _terminalToPos.Lng);
                UpdateMapCenter();
                UpdateFlightPathLine();

                arriveflag = true;
                _terminalPhaseActive = false;

                MarkWaypointPassedAndAdvance();

                return;
            }

            t = Math.Clamp(t, 0.0, 1.0);

            double newLat = _terminalFromPos.Lat + (_terminalToPos.Lat - _terminalFromPos.Lat) * t;
            double newLng = _terminalFromPos.Lng + (_terminalToPos.Lng - _terminalFromPos.Lng) * t;

            UpdateMissilePosition(newLat, newLng);
            UpdateMapCenter();
            UpdateFlightPathLine();
        }


        /// <summary>
        /// 현재 목표 WP를 회색으로 바꾸고 도달 시간 표시 → 다음 WP로 전환.
        /// </summary>
        private void MarkWaypointPassedAndAdvance()
        {
            if (_waypointMarkers == null || _waypointMarkers.Count == 0) return;
            if (_currentWaypointIndex >= _waypointMarkers.Count) return;

            bool isFinal = (_currentWaypointIndex == _waypointMarkers.Count - 1);
            string ts = (_missionStopwatch != null) ? _missionStopwatch.Elapsed.ToString(@"mm\:ss\:ff") : "--:--:--";

            if (isFinal)
            {
                BitmapSource arrived = _finalTargetArrivedImg
                                       ?? (_waypointGrayImages != null && _waypointGrayImages.Count > 0
                                           ? _waypointGrayImages[Math.Min(_currentWaypointIndex, _waypointGrayImages.Count - 1)]
                                           : (_waypointOriginalImages != null ? _waypointOriginalImages[Math.Min(_currentWaypointIndex, _waypointOriginalImages.Count - 1)] : null));

                var img = new System.Windows.Controls.Image
                {
                    Width = 40,
                    Height = 40,
                    Source = arrived,
                    ToolTip = $"최종 표적 도착 ({ts})"
                };

                var timeText = new TextBlock
                {
                    Text = ts,
                    Foreground = Media.Brushes.Black,
                    Background = new SolidColorBrush(Media.Color.FromArgb(180, 255, 255, 255)),
                    Padding = new Thickness(3, 0, 3, 0),
                    Margin = new Thickness(5, 0, 0, 0),
                    VerticalAlignment = VerticalAlignment.Center,
                    FontSize = 10
                };

                var panel = new StackPanel { Orientation = SWC.Orientation.Horizontal };
                panel.Children.Add(img);
                panel.Children.Add(timeText);

                _waypointMarkers[_currentWaypointIndex].Shape = panel;
                _waypointMarkers[_currentWaypointIndex].Offset = new WpfPoint(-20, -20);
            }
            else
            {
                BitmapSource? iconGray =
                     (_waypointGrayImages != null && _waypointGrayImages.Count > 0)
                     ? _waypointGrayImages[Math.Min(_currentWaypointIndex, _waypointGrayImages.Count - 1)]
                     : ToGray((_waypointOriginalImages != null && _waypointOriginalImages.Count > 0)
                              ? _waypointOriginalImages[Math.Min(_currentWaypointIndex, _waypointOriginalImages.Count - 1)]
                              : null);

                var img = new System.Windows.Controls.Image
                {
                    Width = 40,
                    Height = 40,
                    Source = iconGray,
                    ToolTip = $"Waypoint {_currentWaypointIndex + 1} (Reached: {ts})"
                };
                var timeText = new TextBlock
                {
                    Text = ts,
                    Foreground = Media.Brushes.Black,
                    Background = new SolidColorBrush(Media.Color.FromArgb(180, 255, 255, 255)),
                    Padding = new Thickness(3, 0, 3, 0),
                    Margin = new Thickness(5, 0, 0, 0),
                    VerticalAlignment = VerticalAlignment.Center,
                    FontSize = 10
                };
                var panel = new StackPanel { Orientation = SWC.Orientation.Horizontal };
                panel.Children.Add(img);
                panel.Children.Add(timeText);

                _waypointMarkers[_currentWaypointIndex].Shape = panel;
                _waypointMarkers[_currentWaypointIndex].Offset = new WpfPoint(-20, -20);
            }

            _currentWaypointIndex++;

            if (_currentWaypointIndex >= (_waypoints?.Count ?? 0))
            {
                UpdateFlightPathLine();
                AppendLog("[INFO] 최종 웨이포인트 도달. 임무 대기 상태.");
            }
            else
            {
                UpdateFlightPathLine();
                if (finalentry)
                {
                    FinalHoming(15.0f);
                    
                    return;
                }
                AppendLog($"[INFO] 웨이포인트 {_currentWaypointIndex} → {_currentWaypointIndex + 1} 전환.");
            }
        }


        // ===== 경로선(아이콘→현재 목표) =====
        private void UpdateFlightPathLine()
        {
            if (MapControl == null) return;

            if (_flightPathLine != null)
                MapControl.Markers.Remove(_flightPathLine);

            var points = new List<PointLatLng>();
            if (_missileMarker != null && _waypoints != null && _currentWaypointIndex < _waypoints.Count)
            {
                points.Add(_missileMarker.Position);
                points.Add(_waypoints[_currentWaypointIndex]);
            }

            _flightPathLine = new GMapRoute(points);
            _flightPathLine.Shape = new System.Windows.Shapes.Path
            {
                Stroke = Media.Brushes.Red,
                StrokeThickness = 2,
                Opacity = 0.7,
            };
            MapControl.Markers.Add(_flightPathLine);
        }

        // ===== 시뮬레이션 API (타이머/HUD만) =====
        public void ExecuteSimulationStart()
        {
            _simulationRunning = true;
            _missionStopwatch?.Restart();
            _simulationTimer?.Start();
            //AppendLog("[INFO] 임무 시간 측정 시작.");
        }

        private void OnSimulationLoopTick(object? sender, EventArgs e)
        {
            // 35~0 종말 구간 진행
            UpdateTerminalApproach();
            if (MissionTimeText != null && _missionStopwatch != null)
            {
                MissionTimeText.Text = _missionStopwatch.Elapsed.ToString(@"mm\:ss\:ff");
            }
        }

        private void StartSimulation(int scenarioId)
        {
            LoadScenario(scenarioId);

            // 아이콘을 시작 위치로 복귀
            _missileLat = _initialMissileLat;
            _missileLng = _initialMissileLng;
            UpdateMissilePosition(_missileLat, _missileLng);
            UpdateMapCenter();

            // WP 아이콘을 컬러로 초기화
            if (_waypointMarkers != null && _waypointOriginalImages != null)
            {
                for (int i = 0; i < _waypointMarkers.Count; i++)
                {
                    bool isFinal = (i == _waypointMarkers.Count - 1);
                    BitmapSource src = isFinal && _finalTargetIdleImg != null
                        ? _finalTargetIdleImg
                        : _waypointOriginalImages[Math.Min(i, _waypointOriginalImages.Count - 1)];

                    var original = new System.Windows.Controls.Image
                    {
                        Width = 40,
                        Height = 40,
                        Source = src,
                        ToolTip = isFinal ? "최종 표적" : $"Waypoint {i + 1}"
                    };
                    _waypointMarkers[i].Shape = original;
                    _waypointMarkers[i].Offset = new WpfPoint(-20, -20);
                }
            }

            UpdateFlightPathLine();
            //AppendLog("[INFO] 시나리오 초기화 완료. 외부 남은거리 입력 대기.");
        }

        private void StopSimulation()
        {
            _simulationRunning = false;
            _simulationTimer?.Stop();
            _missionStopwatch?.Stop();

            if (_flightPathLine != null)
            {
                MapControl.Markers.Remove(_flightPathLine);
                _flightPathLine = null;
            }

            ClearScenarioMarkers();

            // 수동 조작 기본 위치로 복귀
            _missileLat = _manualStartPosition.Lat;
            _missileLng = _manualStartPosition.Lng;
            UpdateMissilePosition(_missileLat, _missileLng);
            UpdateMapCenter();

            Dispatcher.Invoke(() => SetScenarioUIState(false, null));
            //AppendLog("[INFO] 임무 종료. 초기 화면으로 복귀.");
        }
    }
}