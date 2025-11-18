using System.Windows.Forms;

namespace TheSSENS
{
    public class DbPanel : Panel
    {
        public DbPanel()
        {
            // GDI+ Paint 이벤트를 사용하기 위한 스타일 설정
            SetStyle(ControlStyles.AllPaintingInWmPaint |
                     ControlStyles.OptimizedDoubleBuffer |
                     ControlStyles.UserPaint, true);
            DoubleBuffered = true;
            ResizeRedraw = true;
        }
    }
}