using System;
using System.Windows;
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

        private void OverlayCanvas_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            System.Windows.Point pos = e.GetPosition(OverlayCanvas);
            OnCanvasClick?.Invoke(pos);
            this.Close();
        }
    }
}
