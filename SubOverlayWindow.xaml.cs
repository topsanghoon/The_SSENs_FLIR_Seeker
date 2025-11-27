using System;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;

namespace TheSSENS
{
    public partial class SubOverlayWindow : Window
    {
        public event Action<System.Windows.Point> OnCanvasClick;

        public SubOverlayWindow()
        {
            InitializeComponent();
        }

        private void OverlayCanvas_MouseMove(object sender, System.Windows.Input.MouseEventArgs e)
        {
            if (TargetBox == null) return;

            var pos = e.GetPosition(OverlayCanvas);

            double halfW = TargetBox.Width / 2.0;
            double halfH = TargetBox.Height / 2.0;

            Canvas.SetLeft(TargetBox, pos.X - halfW);
            Canvas.SetTop(TargetBox, pos.Y - halfH);

            if (TargetBox.Visibility != Visibility.Visible)
                TargetBox.Visibility = Visibility.Visible;
        }

        private void OverlayCanvas_MouseLeftButtonDown(object sender, System.Windows.Input.MouseButtonEventArgs e)
        {
            System.Windows.Point pos = e.GetPosition(OverlayCanvas);
            OnCanvasClick?.Invoke(pos);
            this.Close();
        }
    }
}