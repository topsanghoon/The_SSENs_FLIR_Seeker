#define USE_GRAY8_PIPELINE

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
using System.Drawing; // GDI+ Bitmap을 위해 추가
using System.IO;
using System.Net;
using System.Net.Sockets;
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
    /// GStreamer 파이프라인 (Left/Right), 비디오 처리, 신호 감지 로직을 담당합니다.
    /// </summary>
    public partial class MainWindow : System.Windows.Window
    {
        // ===== GStreamer =====
        // [NRT 수정] Nullable(?) 추가
        private Pipeline? _pipelineLeft;
        private Pipeline? _pipelineRight;
        private DispatcherTimer _glibTimer;
        // private DispatcherTimer _timer; // _timer는 UI용이므로 MainWindow.xaml.cs에 있음

        /// <summary>
        /// GStreamer GLib 메인 루프용 타이머를 초기화합니다.
        /// </summary>

        // ====== GStreamer Left (IR 16-bit) ======
        private void InitializeGStreamerLeft()
        {
            try
            {
#if USE_GRAY8_PIPELINE
                // 8비트
                //string pipelineStr =
                //   "udpsrc port=5002 caps=\"video/x-raw,format=GRAY8,width=80,height=60,framerate=9/1\" ! appsink name=sinkLeft";

                string pipelineStr =
                    "udpsrc port=5002 do-timestamp=true ! " +
                    "queue max-size-buffers=1 leaky=downstream ! " +
                    "jpegparse ! jpegdec ! videoconvert ! videoscale ! " +
                    "video/x-raw,format=GRAY8,width=80,height=60 ! appsink name=sinkLeft";

                //string pipelineStr =
                //    "udpsrc port=5002 do-timestamp=true ! " +
                //    "queue max-size-buffers=1 leaky=downstream ! " +
                //    "vp8dec ! videoconvert ! videoscale ! " +
                //    "video/x-raw,format=I420,width=80,height=60 ! appsink name=sinkLeft";


                //string pipelineStr = @"
                //    udpsrc port=5002 caps=""application/x-rtp, media=video, clock-rate=90000, encoding-name=VP8"" ! 
                //    rtpjitterbuffer latency=100 ! 
                //    rtpvp8depay ! 
                //    vp8dec ! 
                //    videoconvert ! 
                //    video/x-raw,format=BGRx ! 
                //    appsink name=sinkLeft sync=false async=false";


#else
                // 16비트
                string pipelineStr =
                   "udpsrc port=5002 caps=\"video/x-raw,format=GRAY16_LE,width=80,height=60,framerate=9/1\" ! appsink name=sinkLeft";
#endif
                _pipelineLeft = (Pipeline)Parse.Launch(pipelineStr);

                var bus = _pipelineLeft.Bus;
                bus.AddSignalWatch();
                bus.Message += OnLeftBusMessage;

                // [NRT 수정] _pipelineLeft null 체크
                if (_pipelineLeft == null)
                {
                    AppendLog("[ERROR] 왼쪽 파이프라인 생성 실패");
                    return;
                }

                var sinkElement = _pipelineLeft.GetByName("sinkLeft");
                if (sinkElement == null)
                {
                    AppendLog("[ERROR] 왼쪽 AppSink를 찾을 수 없음");
                    return;
                }

                var appSink = new AppSink(sinkElement.Handle)
                {
                    EmitSignals = true,
                    MaxBuffers = 1,
                    Drop = true
                };
                appSink.NewSample += OnNewSampleLeft;

                var ret = _pipelineLeft.SetState(State.Playing); // [V2] 이름 변경
                if (ret == StateChangeReturn.Failure)
                {
                    AppendLog("[ERROR] 왼쪽(IR) 파이프라인 시작 실패");
                    _pipelineLeft.Dispose();
                    _pipelineLeft = null;
                    return;
                }
                AppendLog($"[{System.DateTime.Now:HH:mm:ss}] 적외선 영상 스트림 수신 준비 완료");
            }
            catch (Exception ex)
            {
                AppendLog($"[FATAL] Left GST error: {ex.Message}");
            }
        }

        private void OnNewSampleLeft(object sender, NewSampleArgs args)
        {
            // [FPS 계산 로직 시작]
            _framesLeft++;
            long now = System.Environment.TickCount64; // 현재 시간 (ms)
            if (now - _lastTimeLeft >= 1000) // 1초가 지났으면
            {
                _fpsLeft = _framesLeft * 1000.0 / (now - _lastTimeLeft);
                _framesLeft = 0;
                _lastTimeLeft = now;
            }
            // [FPS 계산 로직 끝]

            _isLeftSignalActive = true; // 1. "지난 2초간 신호가 1번이라도 들어왔음" 플래그 ON

            if (!_isLeftOverlayActive) // 2. "그런데 오버레이가 'NoSig' 상태였나?"
            {
                _isLeftOverlayActive = true; // 3. 오버레이 상태를 "Active"로 변경

                // 4. "Active" 상태로 1번만 PNG 업데이트
                System.Windows.Application.Current.Dispatcher.InvokeAsync(() =>
                {
                    int currentIndex = temp % 4;
                    _leftImageOverlay?.UpdateImage(currentIndex);
                    //AppendLog($"[INFO] Left signal detected. Overlay set to 'Frame_{currentIndex + 1}'.");
                });
            }
            // (성능 최적화: 타이머 리셋용 Invoke는 제거된 상태 유지)

            // (이하 V2 GDI+ 영상 렌더링 로직은 동일)
            var appSink = sender as AppSink;
            if (appSink == null) return;

            using (var sample = appSink.PullSample())
            {
                if (sample == null) return;
                using (var buffer = sample.Buffer)
                using (var caps = sample.Caps)
                {
                    var videoInfo = new VideoInfo();
                    if (!videoInfo.FromCaps(caps)) return;
                    buffer.Map(out MapInfo info, MapFlags.Read);
                    try
                    {
#if USE_GRAY8_PIPELINE
                        // ====== 8비트 처리 ======
                        using (var tempMat = Mat.FromPixelData(videoInfo.Height, videoInfo.Width, MatType.CV_8UC1, info.DataPtr))
                        using (var mat8 = tempMat.Clone())
                        {
                            int filter = temp % 4;
                            // 8비트 Mat을 필터 함수로 바로 전달
                            using Mat renderedBase = BuildBaseIrMat(mat8, filter);

                            // ... (Bitmap 변환 및 렌더링 로직은 동일) ...
                            using var bmp = BitmapConverter.ToBitmap(renderedBase);
                            lock (_irBmpLock)
                            {
                                _irLastBmp?.Dispose();
                                _irLastBmp = (System.Drawing.Bitmap)bmp.Clone();
                            }
                            videoPanelLeft?.Invalidate();
                        }
#else
                        // ====== 16비트 처리 ======
                        using (var tempMat = Mat.FromPixelData(videoInfo.Height, videoInfo.Width, MatType.CV_16UC1, info.DataPtr))
                        using (var mat16 = tempMat.Clone())
                        {
                            using var normalized = new Mat();
                            Normalize16To8(mat16, normalized); // 16->8 변환
                            int filter = temp % 4;
                            using Mat renderedBase = BuildBaseIrMat(normalized, filter);

                            // ... (Bitmap 변환 및 렌더링 로직은 동일) ...
                            using var bmp = BitmapConverter.ToBitmap(renderedBase);
                            lock (_irBmpLock)
                            {
                                _irLastBmp?.Dispose();
                                _irLastBmp = (System.Drawing.Bitmap)bmp.Clone();
                            }
                            videoPanelLeft?.Invalidate();
                        }
#endif
                    }
                    catch (Exception ex)
                    {
                        Dispatcher.InvokeAsync(() => AppendLog($"[IR-ERR] {ex.Message}"));
                    }
                    finally
                    {
                        buffer.Unmap(info);
                    }
                }
            }
        }

#if true

#if !USE_GRAY8_PIPELINE
        private static void Normalize16To8(Mat mat16, Mat dst8)
        {
            double minVal = 0, maxVal = 10000;
            double scale = 255.0 / (maxVal - minVal);
            double delta = -minVal * scale;

            using var tmp8 = new Mat();
            mat16.ConvertTo(tmp8, MatType.CV_8U, scale, delta);
            tmp8.CopyTo(dst8);
        }
#endif
        private static Mat BuildBaseIrMat(Mat normalized8u, int filterMode)
        {
            switch (filterMode)
            {
                case 0:
                    return normalized8u.Clone(); // Gray
                case 1:
                    var color = new Mat();
                    Cv2.ApplyColorMap(normalized8u, color, ColormapTypes.Inferno);
                    return color; // Inferno
                case 2:
                    using (var eq = new Mat())
                    {
                        Cv2.EqualizeHist(normalized8u, eq);
                        var color2 = new Mat();
                        Cv2.ApplyColorMap(eq, color2, ColormapTypes.Inferno);
                        return color2; // Equalized Inferno

                        //Cv2.EqualizeHist(normalized8u, eq);
                        //var color2 = new Mat();
                        //Cv2.CvtColor(eq, color2, ColorConversionCodes.GRAY2BGR);
                        //return color2; // Equalized Inferno
                    }
                case 3:
                    using (var edges = new Mat())
                    {
                        Cv2.Canny(normalized8u, edges, 50, 150);
                        var bgr = new Mat();
                        Cv2.CvtColor(edges, bgr, ColorConversionCodes.GRAY2BGR);
                        return bgr; // Canny
                    }
                default:
                    return normalized8u.Clone();
            }
        }


#else

#if !USE_GRAY8_PIPELINE
        private static void Normalize16To8(Mat mat16, Mat dst8)
        {
            double minVal = 7000, maxVal = 10000;
            double scale = 255.0 / (maxVal - minVal);
            double delta = -minVal * scale;

            using var tmp8 = new Mat();
            mat16.ConvertTo(tmp8, MatType.CV_8U, scale, delta);
            tmp8.CopyTo(dst8);
        }
#endif
        // [V2에서 추가] 필터 적용 (수정됨)
        private static Mat BuildBaseIrMat(Mat normalized8u, int filterMode)
        {
            // 1. (공통) 8000-9000으로 정규화된 8비트 이미지를 이퀄라이즈합니다.
            // 이 'eq' 이미지가 모든 필터의 기본 소스가 됩니다.
            using (var eq = new Mat())
            {
                Cv2.EqualizeHist(normalized8u, eq);

                // 2. 필터 모드에 따라 분기합니다.
                switch (filterMode)
                {
                    case 0:
                        // Equalized Gray
                        return eq.Clone();

                    case 1:
                        // Equalized Inferno
                        var color = new Mat();
                        Cv2.ApplyColorMap(eq, color, ColormapTypes.Inferno);
                        return color;

                    case 2:
                        // Equalized Inferno + Threshold (150-255)
                        using (var threshMask = new Mat())
                        using (var colorMap = new Mat())
                        {
                            // 2a. 이퀄라이즈된 이미지(eq)에서 150~255 범위만 마스크로 만듭니다.
                            Cv2.Threshold(eq, threshMask, 150, 255, ThresholdTypes.Binary);

                            // 2b. 이퀄라이즈된 이미지(eq)에 컬러맵을 적용합니다.
                            Cv2.ApplyColorMap(eq, colorMap, ColormapTypes.Inferno);

                            // 2c. 검은색 배경에 (2b) 컬러맵을 (2a) 마스크로 씌웁니다.
                            var result = new Mat(eq.Size(), MatType.CV_8UC3, Scalar.Black);
                            colorMap.CopyTo(result, threshMask);
                            return result;
                        }

                    case 3:
                        // Equalized Canny
                        using (var edges = new Mat())
                        {
                            Cv2.Canny(eq, edges, 50, 150);
                            var bgr = new Mat();
                            Cv2.CvtColor(edges, bgr, ColorConversionCodes.GRAY2BGR);
                            return bgr;
                        }

                    default:
                        // 기본값 (Equalized Gray)
                        return eq.Clone();
                }
            }
        }
#endif

        // [V1 코드 교체] GStreamer Right (EO JPEG → AppSink)
        private void InitializeGStreamerRight()
        {
            try
            {
                // [V2] V1의 VP8 파이프라인 대신 JPEG 파이프라인 사용
                string pipelineStr =
                    "udpsrc port=5003 ! jpegdec ! videoconvert ! " +
                    "video/x-raw,format=BGR,width=320,height=240 ! appsink name=sinkRight";


                //string pipelineStr =
                //    "udpsrc port=5003 caps=\"application/x-rtp, media=video, encoding-name=VP8, payload=96\" ! rtpvp8depay ! vp8dec ! videoconvert ! " +
                //    "appsink name=sinkRight";

                //string pipelineStr =
                //    "udpsrc port=5003 ! " +
                //    "vp8dec ! videoconvert ! " +
                //    "video/x-raw,format=I420 ! " +
                //    "appsink name=sinkRight";



                _pipelineRight = (Pipeline)Parse.Launch(pipelineStr);

                var bus = _pipelineRight.Bus;
                bus.AddSignalWatch();
                bus.Message += OnRightBusMessage;

                if (_pipelineRight == null) // [V1] NRT 체크 유지
                {
                    AppendLog("[ERROR] 오른쪽 파이프라인 생성 실패");
                    return;
                }

                var sinkElement = _pipelineRight.GetByName("sinkRight"); // [V1] 이름 유지
                if (sinkElement == null)
                {
                    AppendLog("[ERROR] 오른쪽 AppSink를 찾을 수 없음");
                    return;
                }

                var appSink = new AppSink(sinkElement.Handle)
                {
                    EmitSignals = true,
                    MaxBuffers = 1,
                    Drop = true
                };
                appSink.NewSample += OnNewSampleRight; // EO 프레임 콜백 (아래 신규 추가)

                var result = _pipelineRight.SetState(State.Playing);
                if (result == StateChangeReturn.Failure)
                {
                    AppendLog("[ERROR] 오른쪽(EO) 파이프라인 시작 실패");
                    _pipelineRight?.Dispose();
                    _pipelineRight = null;
                    return;
                }
                AppendLog($"[{System.DateTime.Now:HH:mm:ss}] 가시광 영상 스트림 수신 준비 완료");
            }
            catch (Exception ex)
            {
                AppendLog($"[FATAL] Right GST error: {ex.Message}");
            }
        }

        private void OnNewSampleRight(object? sender, NewSampleArgs args)
        {
            // [FPS 계산 로직 시작]
            _framesRight++;
            long now = System.Environment.TickCount64;
            if (now - _lastTimeRight >= 1000)
            {
                _fpsRight = _framesRight * 1000.0 / (now - _lastTimeRight);
                _framesRight = 0;
                _lastTimeRight = now;
            }
            // [FPS 계산 로직 끝]

            _isRightSignalActive = true; // 1. "신호 들어옴" 플래그 ON

            if (!_isRightOverlayActive) // 2. "오버레이가 'NoSig' 상태였나?"
            {
                _isRightOverlayActive = true; // 3. 오버레이 상태를 "Active"로 변경

                // 4. "Active" 상태로 1번만 PNG 업데이트
                System.Windows.Application.Current.Dispatcher.InvokeAsync(() =>
                {
                    int currentIndex = temp % 4;
                    _rightImageOverlay?.UpdateImage(currentIndex);
                    //AppendLog($"[INFO] Right signal detected (AppSink). Overlay set to 'Frame_{currentIndex + 1}'.");
                });
            }
            // (성능 최적화: 타이머 리셋용 Invoke는 제거된 상태 유지)

            // (이하 V2 GDI+ 영상 렌더링 로직은 동일)
            var appSink = sender as AppSink;
            if (appSink == null) return;

            using (var sample = appSink.PullSample())
            {
                if (sample == null) return;
                using (var buffer = sample.Buffer)
                using (var caps = sample.Caps)
                {
                    var vi = new VideoInfo();
                    if (!vi.FromCaps(caps)) return;
                    _eoSrcW = (int)vi.Width;
                    _eoSrcH = (int)vi.Height;
                    buffer.Map(out MapInfo info, MapFlags.Read);
                    try
                    {
                        using (var tmp = Mat.FromPixelData(vi.Height, vi.Width, MatType.CV_8UC3, info.DataPtr))
                        using (var cloned = tmp.Clone())
                        {
                            using var bmp = BitmapConverter.ToBitmap(cloned);
                            lock (_eoBmpLock)
                            {
                                _eoLastBmp?.Dispose();
                                _eoLastBmp = (System.Drawing.Bitmap)bmp.Clone();
                            }
                        }
                        videoPanelRight?.Invalidate();
                    }
                    catch (Exception ex)
                    {
                        Dispatcher.InvokeAsync(() => AppendLog($"[EO-ERR] {ex.Message}"));
                    }
                    finally
                    {
                        buffer.Unmap(info);
                    }
                }
            }
        }

        private void OnLeftSignalTimeout(object? sender, EventArgs e)
        {
            if (_isLeftSignalActive)
            {
                // 신호가 1번이라도 들어왔었음 (정상)
                _isLeftSignalActive = false; // 다음 2초를 위해 체크 플래그 리셋
            }
            else
            {
                // 지난 2초간 신호가 1번도 안 들어옴 (신호 끊김)
                if (_isLeftOverlayActive) // "그런데 오버레이가 'Active' 상태였나?"
                {
                    _isLeftOverlayActive = false; // 1. 오버레이 상태를 "Lost"로 변경
                                                  // 2. "Lost" 상태로 1번만 PNG 업데이트
                    _leftImageOverlay?.UpdateImage(-1);
                    //AppendLog("[WARN] Left signal lost (timeout). Overlay set to 'No Signal'.");
                }
            }
            _leftSignalTimer?.Start(); // 다음 2초 체크 시작
        }

        // [수정] OnLeftBusMessage: 'OverlayActive' 플래그를 확인
        private void OnLeftBusMessage(object o, MessageArgs args)
        {
            var msg = args.Message;
            if (msg.Type == MessageType.Eos || msg.Type == MessageType.Error)
            {
                System.Windows.Application.Current.Dispatcher.Invoke(() =>
                {
                    _leftSignalTimer?.Stop(); // 타이머 완전 정지
                    if (_isLeftOverlayActive) // "그런데 오버레이가 'Active' 상태였나?"
                    {
                        _isLeftOverlayActive = false; // 1. 오버레이 상태를 "Lost"로 변경
                        _isLeftSignalActive = false;
                        // 2. "Lost" 상태로 1번만 PNG 업데이트
                        _leftImageOverlay?.UpdateImage(-1);
                        //AppendLog("[WARN] Left signal lost (EOS/Error). Overlay set to 'No Signal'.");
                    }
                });
            }
        }

        // [수정] OnRightSignalTimeout: 'OverlayActive' 플래그를 확인
        private void OnRightSignalTimeout(object? sender, EventArgs e)
        {
            if (_isRightSignalActive)
            {
                // 신호가 1번이라도 들어왔었음 (정상)
                _isRightSignalActive = false; // 다음 2초를 위해 체크 플래그 리셋
            }
            else
            {
                // 지난 2초간 신호가 1번도 안 들어옴 (신호 끊김)
                if (_isRightOverlayActive) // "그런데 오버레이가 'Active' 상태였나?"
                {
                    _isRightOverlayActive = false; // 1. 오버레이 상태를 "Lost"로 변경
                                                   // 2. "Lost" 상태로 1번만 PNG 업데이트
                    System.Windows.Application.Current.Dispatcher.Invoke(() =>
                    {
                        _rightImageOverlay?.UpdateImage(-1);
                        //AppendLog("[WARN] Right signal lost (timeout). Overlay set to 'No Signal'.");
                    });
                }
            }
            _rightSignalTimer?.Start(); // 다음 2초 체크 시작
        }

        // [수정] OnRightBusMessage: 'OverlayActive' 플래그를 확인
        private void OnRightBusMessage(object o, MessageArgs args)
        {
            var msg = args.Message;

            switch (msg.Type)
            {
                case MessageType.Eos:
                case MessageType.Error:
                    System.Windows.Application.Current.Dispatcher.Invoke(() =>
                    {
                        _rightSignalTimer?.Stop(); // 타이머 완전 정지
                        if (_isRightOverlayActive) // "그런데 오버레이가 'Active' 상태였나?"
                        {
                            _isRightOverlayActive = false; // 1. 오버레이 상태를 "Lost"로 변경
                            _isRightSignalActive = false;
                            // 2. "Lost" 상태로 1번만 PNG 업데이트
                            _rightImageOverlay?.UpdateImage(-1);
                            //AppendLog("[WARN] Right signal lost (EOS/Error). Overlay set to 'No Signal'.");
                        }
                    });
                    break;
            }
        }

    }
}