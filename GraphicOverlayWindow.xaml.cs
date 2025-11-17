using System;
// --- [신규] GDI+ 및 WPF 컨트롤/별칭(Alias)을 위한 using ---
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Drawing.Text;
using System.IO;
using System.Windows;
using System.Windows.Controls;      // Canvas, TextBlock
using System.Windows.Forms;         // PaintEventArgs
using System.Windows.Media.Imaging; // BitmapImage를 위해 추가
using System.Windows.Shapes;        // Rectangle, Polygon
using WpfPoint = System.Windows.Point; // 'WpfPoint' 별칭 추가
// --- [신규 끝] ---

namespace TheSSENS
{
    public partial class GrapicOverlayWindow : Window
    {
        // 1. 이미지를 여러 개 보관하도록 배열로 변경 (V1 기존 필드)
        private static readonly BitmapImage _noSignalImage;
        private static readonly BitmapImage[] _signalImages;

        // --- [신규] EO 전용 이미지 추가 --- (V1 기존 필드)
        private static readonly BitmapImage _signalImageEO;
        // --- [끝] ---

        // [신규] MainWindow에서 설정할 모드 플래그
        public bool IsEoOnlyMode { get; set; } = false;

        // --- [신규] V2 GDI+ 오버레이용 필드 ---
        private volatile bool _irHasBox = false;
        private volatile float _irBx, _irBy, _irBw, _irBh;

        private volatile bool _eoHasAruco = false;
        private volatile uint _eoArucoId = 0;
        private WpfPoint[] _eoArucoPts = new WpfPoint[4];

        // [신규] MainWindow에서 받아올 해상도 값 (GDI+ 스케일링용)
        private volatile int _irWidth = 80; // 기본값
        private volatile int _irHeight = 60;
        private volatile int _eoWidth = 640;
        private volatile int _eoHeight = 480;
        // --- [신규 끝] ---


        // 정적 생성자(static constructor): (V1 기존 로직)
        static GrapicOverlayWindow()
        {
            try
            {
                string baseDir = AppDomain.CurrentDomain.BaseDirectory;
                string path_nosig = baseDir + "/Images/Frame_nosig.png";
                string path_1 = baseDir + "/Images/Frame_1.png";
                string path_2 = baseDir + "/Images/Frame_2.png";
                string path_3 = baseDir + "/Images/Frame_3.png";
                string path_4 = baseDir + "/Images/Frame_4.png";
                string path_EO = baseDir + "/Images/Frame_EO.png";
                // 2. '신호 없음' 이미지 로드
                _noSignalImage = new BitmapImage(new Uri(path_nosig));
                _noSignalImage.Freeze();

                // 3. '신호 있음' 이미지 4개를 배열로 로드 (0, 1, 2, 3)
                _signalImages = new BitmapImage[4]; // 4개 공간 생성

                // selectNum 0, 1, 2, 3에 맞춰 순서대로 로드
                _signalImages[0] = new BitmapImage(new Uri(path_1));
                _signalImages[1] = new BitmapImage(new Uri(path_2));
                _signalImages[2] = new BitmapImage(new Uri(path_3));
                _signalImages[3] = new BitmapImage(new Uri(path_4));

                // 모든 이미지 동결 (Freeze)
                foreach (var img in _signalImages)
                {
                    img.Freeze();
                }

                // --- [신규] EO 이미지 로드 및 동결 ---
                _signalImageEO = new BitmapImage(new Uri(path_EO));
                _signalImageEO.Freeze();
                // --- [끝] ---
            }
            catch (Exception ex)
            {
                System.Windows.MessageBox.Show($"오버레이 이미지 로드 실패: {ex.Message}\nFrame_1~4.png, Frame_EO.png 파일이 모두 있는지 확인하세요.");
            }
        }

        public GrapicOverlayWindow()
        {
            InitializeComponent();
            if (_noSignalImage != null)
            {
                OverlayImage.Source = _noSignalImage;
            }
        }

        /// <summary>
        /// (V1) PNG 필터 이미지를 업데이트합니다.
        /// </summary>
        public void UpdateImage(int signalIndex)
        {
            Dispatcher.Invoke(() =>
            {
                if (signalIndex == -1) // 1. 신호 없음
                {
                    if (_noSignalImage != null) OverlayImage.Source = _noSignalImage;
                }
                else if (IsEoOnlyMode) // 2. 신호 있음 + "EO 전용 모드" (오른쪽 패널)
                {
                    if (_signalImageEO != null) OverlayImage.Source = _signalImageEO;
                }
                else if (signalIndex >= 0 && signalIndex < _signalImages.Length && _signalImages[signalIndex] != null)
                {
                    OverlayImage.Source = _signalImages[signalIndex];
                }
                else // 4. 예외 처리 (잘못된 인덱스 등)
                {
                    if (_noSignalImage != null) OverlayImage.Source = _noSignalImage;
                }
            });
        }
    }
}