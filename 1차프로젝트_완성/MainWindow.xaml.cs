using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using Microsoft.Kinect;
using System.Runtime.InteropServices;
using System.Threading;
using System.Windows.Threading;
using Microsoft.Kinect.Toolkit;
using Microsoft.Kinect.Toolkit.FaceTracking;
using System.Speech.Synthesis;
using Point = System.Windows.Point;

namespace _1차프로젝트_완성
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        
        SpeechSynthesizer speach = new SpeechSynthesizer();

        public MainWindow()
        {
            #region
            m_rect = new Rectangle[col + 1, row + 1];
            rank = new makeDepthRank[col * row + 1];
            point = new rect_infomaion[col + 1, row + 1];
            QueueSize = 90;

            rect_Shift = new CriticalShell_Shift[QueueSize];

            Frame_cnt = 0;
            rect_cnt = new int[4, 4];

            #endregion

            InitializeComponent();
            InitializeNui();
            int cnt = 0;

            #region 초기화

            Image_critical.Visibility = Visibility.Collapsed;

            for (int i = 0; i < QueueSize; i++)
                rect_Shift[i] = new CriticalShell_Shift(3, 3);

            for (int i = 0; i < col; i++)
                for (int j = 0; j < row; j++)
                {

                    cnt++;
                    m_rect[i, j] = new Rectangle();
                    m_rect[i, j].StrokeThickness = 5;
                    m_rect[i, j].Visibility = Visibility.Collapsed;
                    Canvas1.Children.Add(m_rect[i, j]);
                }
            #endregion
        }

        KinectSensor nui = null;
        KinectSensor nui2 = null;
        void InitializeNui()
        {

            nui = KinectSensor.KinectSensors[1];
            nui2 = KinectSensor.KinectSensors[0];
            
            nui.DepthStream.Enable(DepthImageFormat.Resolution320x240Fps30);
            nui.DepthFrameReady += new EventHandler<DepthImageFrameReadyEventArgs>(nui_DepthFrameReady);
   //         nui.SkeletonStream.Enable();

            nui2.ColorStream.Enable();
  //          nui2.ColorFrameReady += new EventHandler<ColorImageFrameReadyEventArgs>(nui_ColorFrameReady);

            nui2.DepthStream.Enable();
            nui2.SkeletonStream.Enable();
            nui2.SkeletonStream.TrackingMode = SkeletonTrackingMode.Seated;
            nui2.AllFramesReady += new EventHandler<AllFramesReadyEventArgs>(nui_AllFramesReady);

            nui.Start();
            nui2.Start();
        }

        #region 얼굴

        void nui_ColorFrameReady(object sender, ColorImageFrameReadyEventArgs e)
        {
            ColorImageFrame ImageParam = e.OpenColorImageFrame();

            if (ImageParam == null) return;

            byte[] ImageBits = new byte[ImageParam.PixelDataLength];
            ImageParam.CopyPixelDataTo(ImageBits);

            BitmapSource src = null;
            src = BitmapSource.Create(ImageParam.Width, ImageParam.Height,
                                        96, 96, PixelFormats.Bgr32, null,
                                        ImageBits,
                                        ImageParam.Width * ImageParam.BytesPerPixel);
            image2.Source = src;
        }

        public Skeleton[] skeletonData;
        private readonly Dictionary<int, SkeletonFaceTracker> trackedSkeletons = new Dictionary<int, SkeletonFaceTracker>();

        protected override void OnRender(DrawingContext drawingContext)
        {
            base.OnRender(drawingContext);
            foreach (SkeletonFaceTracker faceInformation in this.trackedSkeletons.Values)
                faceInformation.DrawFaceModel(drawingContext); //그리기 불러온 곳
            
        }

        private void RemoveTracker(int trackingId) // trackedSkeletons에 들오온 것을 지워주는 작업
        {
            this.trackedSkeletons[trackingId].Dispose();
            this.trackedSkeletons.Remove(trackingId);
        }

        private void ResetFaceTracking() // 들어온 것 초기화
        {
            foreach (int trackingId in new List<int>(this.trackedSkeletons.Keys))
            {
                this.RemoveTracker(trackingId);
            }
        }


        private ColorImageFormat colorImageFormat = ColorImageFormat.Undefined;
        private DepthImageFormat depthImageFormat = DepthImageFormat.Undefined;

        private static FaceTriangle[] faceTriangles;

        private class SkeletonFaceTracker : IDisposable
        {
            private EnumIndexableCollection<FeaturePoint, PointF> facePoints;
            private FaceTracker faceTracker;
            private bool lastFaceTrackSucceeded;
            private SkeletonTrackingState skeletonTrackingState;
            public int LastTrackedFrame { get; set; }

            public void Dispose()
            {
                if (this.faceTracker != null)
                {
                    this.faceTracker.Dispose();
                    this.faceTracker = null;
                }
            }


            //스켈레톤과 합쳐서 머리의 중심점과 함께 나타내자 // 레지스트리로 넘기는 것도 하나의 방법
            public void DrawFaceModel(DrawingContext drawingContext)  // face 그리기 코 끝에 점을 따라 다니게 하기를 하자
            {
                if (!this.lastFaceTrackSucceeded || this.skeletonTrackingState != SkeletonTrackingState.Tracked)
                {
                    return;
                }

                var faceModelPts = new List<System.Windows.Point>();
                var faceModel = new List<FaceModelTriangle>();

                //얼굴의 120개의 점을 찍는다.
                int tt = 3;
                for (int i = 0; i < this.facePoints.Count; i++)
                {
                    faceModelPts.Add(new System.Windows.Point(this.facePoints[i].X + 0.5f, this.facePoints[i].Y + 0.5f));
                    
                }



                var faceModelGroup = new GeometryGroup();
                for (int i = 0; i < faceModel.Count; i++)
                {
                    var faceTriangle = new GeometryGroup();
                    faceTriangle.Children.Add(new LineGeometry(faceModel[i].P1, faceModel[i].P2));
                    faceTriangle.Children.Add(new LineGeometry(faceModel[i].P2, faceModel[i].P3));
                    faceTriangle.Children.Add(new LineGeometry(faceModel[i].P3, faceModel[i].P1));
                    faceModelGroup.Children.Add(faceTriangle);
                }

                drawingContext.DrawGeometry(Brushes.LightYellow, new Pen(Brushes.LightYellow, 1.0), faceModelGroup);

            }

            /// <summary>
            /// Updates the face tracking information for this skeleton
            /// </summary>
            internal void OnFrameReady(KinectSensor kinectSensor, ColorImageFormat colorImageFormat, byte[] colorImage, DepthImageFormat depthImageFormat, short[] depthImage, Skeleton skeletonOfInterest)
            {
                this.skeletonTrackingState = skeletonOfInterest.TrackingState;

                if (this.skeletonTrackingState != SkeletonTrackingState.Tracked)
                {
                    // nothing to do with an untracked skeleton.
                    return;
                }

                if (this.faceTracker == null)
                {
                    try
                    {
                        this.faceTracker = new FaceTracker(kinectSensor);
                    }
                    catch (InvalidOperationException)
                    {
                        // During some shutdown scenarios the FaceTracker
                        // is unable to be instantiated.  Catch that exception
                        // and don't track a face.
                        this.faceTracker = null;
                    }
                }

                if (this.faceTracker != null)
                {
                    FaceTrackFrame frame = this.faceTracker.Track(
                        colorImageFormat, colorImage, depthImageFormat, depthImage, skeletonOfInterest);

                    this.lastFaceTrackSucceeded = frame.TrackSuccessful;
                    if (this.lastFaceTrackSucceeded)
                    {
                        if (faceTriangles == null)
                        {
                            // only need to get this once.  It doesn't change.
                            faceTriangles = frame.GetTriangles();
                        }

                        this.facePoints = frame.GetProjected3DShape();


                        double dbX = facePoints[5].X;
                        double dbY = facePoints[5].Y;

                        App thisApp = App.Current as App;
                        thisApp.m_dbX = dbX;
                        thisApp.m_dbY = dbY;
                    }
                }
            }



            private struct FaceModelTriangle
            {
                public System.Windows.Point P1;
                public System.Windows.Point P2;
                public System.Windows.Point P3;
            }
        }

        private short[] depthImage;
        private byte[] colorImage;

        void nui_AllFramesReady(object sender, AllFramesReadyEventArgs e)
        {
            ColorImageFrame cf = e.OpenColorImageFrame();
            DepthImageFrame df = e.OpenDepthImageFrame();
            SkeletonFrame sf = e.OpenSkeletonFrame();

            //            Skeleton[] skeletonData = new Skeleton[sf.SkeletonArrayLength];

            if (cf == null || df == null || sf == null)
            {
                return;
            }


            byte[] ImageBits = new byte[cf.PixelDataLength];
            cf.CopyPixelDataTo(ImageBits);

            BitmapSource src = null;
            src = BitmapSource.Create(cf.Width, cf.Height,
                                        96, 96, PixelFormats.Bgr32, null,
                                        ImageBits,
                                        cf.Width * cf.BytesPerPixel);
            image2.Source = src;



            // Check for image format changes.  The FaceTracker doesn't
            // deal with that so we need to reset.
            if (this.depthImageFormat != df.Format)
            {
                this.ResetFaceTracking();
                this.depthImage = null;
                this.depthImageFormat = df.Format;
            }

            if (this.colorImageFormat != cf.Format)
            {
                this.ResetFaceTracking();
                this.colorImage = null;
                this.colorImageFormat = cf.Format;
            }

            // Create any buffers to store copies of the data we work with
            if (this.depthImage == null)
            {
                this.depthImage = new short[df.PixelDataLength];
            }

            if (this.colorImage == null)
            {
                this.colorImage = new byte[cf.PixelDataLength];
            }

            // Get the skeleton information
            if (this.skeletonData == null || this.skeletonData.Length != sf.SkeletonArrayLength)
            {
                this.skeletonData = new Skeleton[sf.SkeletonArrayLength];
            }

            cf.CopyPixelDataTo(this.colorImage);
            df.CopyPixelDataTo(this.depthImage);
            sf.CopySkeletonDataTo(this.skeletonData);

            foreach (Skeleton skeleton in this.skeletonData)
            {
                if (skeleton.TrackingState == SkeletonTrackingState.Tracked
                    || skeleton.TrackingState == SkeletonTrackingState.PositionOnly)
                {
                    // We want keep a record of any skeleton, tracked or untracked.
                    if (!this.trackedSkeletons.ContainsKey(skeleton.TrackingId))
                    {
                        this.trackedSkeletons.Add(skeleton.TrackingId, new SkeletonFaceTracker());
                    }

                    // Give each tracker the upated frame.
                    SkeletonFaceTracker skeletonFaceTracker;
                    if (this.trackedSkeletons.TryGetValue(skeleton.TrackingId, out skeletonFaceTracker))
                    {
                        skeletonFaceTracker.OnFrameReady(nui2, colorImageFormat, colorImage, depthImageFormat, depthImage, skeleton);
                        skeletonFaceTracker.LastTrackedFrame = sf.FrameNumber;
                    }
                }
            }



            using (DepthImageFrame depthImageFrame = e.OpenDepthImageFrame())
            {
                if (depthImageFrame != null)
                {
                    foreach (Skeleton sd in skeletonData)
                    {
                        if (sd.TrackingState == SkeletonTrackingState.Tracked)
                        {
                            Joint joint = sd.Joints[JointType.Head];

                            DepthImagePoint depthPoint;

                            //                            CoordinateMapper coordinateMapper = new CoordinateMapper(nui);
                            //                            depthPoint = coordinateMapper.MapSkeletonPointToDepthPoint(joint.Position, DepthImageFormat.Resolution320x240Fps30);

                            depthPoint = depthImageFrame.MapFromSkeletonPoint(joint.Position);

                            System.Windows.Point point = new System.Windows.Point((int)(image2.ActualWidth * depthPoint.X
                                                               / depthImageFrame.Width),
                                                    (int)(image2.ActualHeight * depthPoint.Y
                                                               / depthImageFrame.Height));


                            Canvas.SetLeft(ellipse1, (point.X) - ellipse1.Width);
                            Canvas.SetTop(ellipse1, (point.Y) - ellipse1.Height);

                            App thisApp = App.Current as App;

                            Canvas.SetLeft(rect2, thisApp.m_dbX - rect2.Width);
                            Canvas.SetTop(rect2, thisApp.m_dbY - rect2.Height);

                            double GapX, GapY;
                            GapX = point.X - (thisApp.m_dbX - 2) ;
                            GapY = point.Y - (thisApp.m_dbY - 2) ;

                            int siteX = 999, siteY = 999;

                            if (GapX < 30 && GapX > -30)
                                siteX = 1;
                            else if (GapX >= 30)
                                siteX = 0;
                            else if (GapY <= -30)
                                siteX = 2;

                            if (GapY >= -40)
                                siteY = 0;
                            else if (GapY < -40 && GapY > -60)
                                siteY = 1;
                            else if (GapY <= -60)
                                siteY = 2;

                            int site;
                            site = siteX + (siteY * 3);
                            if (site == 0)
                                text2.Text = "좌상";
                            else if (site == 1)
                                text2.Text = "상";
                            else if (site == 2)
                                text2.Text = "우상";
                            else if (site == 3)
                                text2.Text = "좌";
                            else if (site == 4)
                                text2.Text = "정";
                            else if (site == 5)
                                text2.Text = "우";
                            else if (site == 6)
                                text2.Text = "좌하";
                            else if (site == 7)
                                text2.Text = "하";
                            else if (site == 8)
                                text2.Text = "우하";

                            thisApp.nowsite = site;
                            /*
                             
                        rect4.X = facePoints[i].X - 2;
                        rect4.Y = facePoints[i].Y - 2;
                        rect4.Width = 4;
                        rect4.Height = 4;
                             */
                        }
                    }
                }
            }
        }
#endregion 



        #region 장애물소스

        #region struct사용

        public struct rect_infomaion  //조그만 사각형(위험물 위치 표시) 정보들
        {
            public int min_X;           // 사각형 중 가장 왼쪽의 픽셀 좌표
            public int min_Y;           // 사각형 중 가장 위쪽의 픽셀 좌표
            public int max_X;           // 
            public int max_Y;
            public int sGape;
            public int bGape;
        }

        rect_infomaion[,] point;

        public struct makeDepthRank
        {
            public int i, j, Gape;
        }
        makeDepthRank[] rank;



        #endregion

        #region 쉬프트 (원형큐 알고리즘에서 따옴)(3초 중 2초 이상 장애물이 있을 시 경고표시 판단)

        public struct CriticalShell_Shift
        {
            public int[,] lastinfo_int; //지금까지 몇 초간 위험물이 있었던 큰 셀 판단

            public CriticalShell_Shift(int row_shell, int col_shell)
            {
                lastinfo_int = new int[row_shell, col_shell];        //지금까지 몇 초간 위험물이 있었던 큰 셀 판단

                for (int i = 0; i < row_shell; i++)
                    for (int j = 0; j < col_shell; j++)
                        lastinfo_int[i, j] = 0;

            }

        }
        #endregion 쉬프트

        #region 장애물판단


        CriticalShell_Shift[] rect_Shift;
        int rear = 0;
        int QueueSize;

        #endregion

        #region 객체들(그리고 정보)

        Rectangle[,] m_rect;    //조그만 사각형(위험물 표시)
        int[,] rect_cnt;        // 커다란 셀(전체 크기의 3분의 1 씩 나눈 것들)에서 조그만 사각형 개수
        int Frame_cnt;          // 현재 프레임이 몇번째인지 알아보고 1초마다 비프음 소리를 낼지를 판단.
        int col = 12, row = 12;  //조그만 사각형의 가로와 세로개수
        bool myFlag = true;     //조그만 사각형의 색깔을 어떤 기준으로 나타낼 것인지(true면 사각형 크기에 따라 색깔을 다르게, false면 사각형안의 거리값의 차이에 따라 색깔을 다르게)

        #endregion 객체들

        #region 소리 준비

        
        int big_rect = 10;  //위험물이 가장 많은 위치에 따라서 소리가 남(위험물이 온 위치는 가장 많은 위험물로 인식된 곳에서 온다고 생각하였기 때문)

        #endregion


        void nui_DepthFrameReady(object sender, DepthImageFrameReadyEventArgs e)
        {
            bool critical_flag = false;
            DepthImageFrame ImageParam = e.OpenDepthImageFrame();

            if (ImageParam == null) return;

            short[] ImageBits = new short[ImageParam.PixelDataLength];
            ImageParam.CopyPixelDataTo(ImageBits);

            WriteableBitmap wb = new WriteableBitmap(ImageParam.Width,
                                                     ImageParam.Height,
                                                     96, 96,
                                                     PixelFormats.Bgr32, null);
            wb.WritePixels(new Int32Rect(0, 0,
                                            ImageParam.Width,
                                            ImageParam.Height),
                            GetRGB(ImageParam, ImageBits, nui.DepthStream),
                            ImageParam.Width * 4,
                            0);

            image1.Source = wb;

            #region 셀(큰 사각형) 처리

            int cnt = 0;

            for (int i = 0; i < col; i++)
                for (int j = 0; j < row; j++)
                {
                    if (!(point[i, j].max_X > 0 && point[i, j].min_X < 99999))
                    {
                        m_rect[i, j].Visibility = Visibility.Collapsed;
                        continue;
                    }

                    #region 작은 사각형 처리
                    m_rect[i, j].Width = (point[i, j].max_X - point[i, j].min_X )* 2;
                    m_rect[i, j].Height = (point[i, j].max_Y - point[i, j].min_Y) * 2;

                    float Per; // 사각형 크기의 비율
                    Per = (float)((m_rect[i, j].Width * m_rect[i, j].Height) / (ImageParam.Width * ImageParam.Height / col / row));

                    if (myFlag == true)
                        m_rect[i, j].Stroke = new SolidColorBrush(Color.FromRgb((byte)(255 * Per), (byte)(255 - (255 * Per)), 0));

                    m_rect[i, j].Visibility = Visibility.Visible;
                    Canvas.SetLeft(m_rect[i, j], (double)point[i, j].min_X * 2);
                    Canvas.SetTop(m_rect[i, j], (double)point[i, j].min_Y * 2);

                    #endregion 사각형 그리기

                    #region 깊이 차이

                    rect_cnt[i / 4, i / 4]++;
                    critical_flag = true;

                    rank[cnt].Gape = point[i, j].bGape - point[i, j].sGape;

                    rank[cnt].i = i;
                    rank[cnt].j = j;
                    cnt++;

                    #endregion 깊이 차이

                }

            #region 이전 상황을 점검하여 3초 간 중 2초 이상 위험물이 있었던 부분에 표시

            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    if (rect_cnt[i, j] > 0)
                        rect_Shift[rear].lastinfo_int[i, j] = 1;
                    else
                        rect_Shift[rear].lastinfo_int[i, j] = 0;

                    int second_cnt = rect_Shift[rear].lastinfo_int[i, j];

                    for (int last_seconds = 1; last_seconds < QueueSize; last_seconds++)
                    {
                        int rear_now = rear - last_seconds;
                        if (rear_now < 0)
                            rear_now = QueueSize + rear_now;

                        second_cnt += rect_Shift[rear_now].lastinfo_int[i, j];

                    }

                    if (second_cnt >= 60)
                    {
                        visible_secondCritical(i, j);

                    }
                    else if (second_cnt < 60)
                    {
                        Collapsed_secondCritical(i, j);
                    }
                }
            }

            rear++;
            rear %= QueueSize; // 90프레임(3초)만 검사할 것이니 QueueSize의 나머지만 사용

            #endregion 이전 상황을 점검하여 3초 간 중 2초 이상 위험물이 있었던 부분에 표시

            #region 가장 작은 사각형이 많은 곳의 셀(큰 사각형) 경고 표시

            if (critical_flag)
            {

                int rt_x = 0, rt_y = 0;

                for (int i = 0; i < 3; i++)
                    for (int j = 0; j < 3; j++)
                        if (rect_cnt[rt_x, rt_y] < rect_cnt[i, j])
                        {
                            rt_x = i;
                            rt_y = j;
                        }

                big_rect = (rt_x * 3) + rt_y;

                Frame_cnt++;

                App thisApp = App.Current as App;

                if (big_rect != thisApp.nowsite)
                {
                    if (Frame_cnt >= 30) // 1초 가 지날때마다 어떤 소리를 낼 것인지 검사
                    {
                        Frame_cnt = 0;



                        Thread speachThread = new Thread(new ThreadStart(callBeep));
                        speachThread.Start();

                    }


                    if (big_rect == 0)
                        text1.Text = "왼측 상단을 보세요";
                    else if (big_rect == 1)
                        text1.Text = "상단을 보세요";
                    else if (big_rect == 2)
                        text1.Text = "우측 상단을 보세요";
                    else if (big_rect == 3)
                        text1.Text = "좌측을 보세요";
                    else if (big_rect == 4)
                        text1.Text = "정면을 보세요";
                    else if (big_rect == 5)
                        text1.Text = "우측을 보세요";
                    else if (big_rect == 6)
                        text1.Text = "좌측 하단을 보세요";
                    else if (big_rect == 7)
                        text1.Text = "하단을 보세요";
                    else if (big_rect == 8)
                        text1.Text = "우측 하단을 보세요";

                    Image_critical.Visibility = Visibility.Visible;
                    Canvas.SetLeft(Image_critical, (double)(Canvas1.Width / 3 * rt_y));
                    Canvas.SetTop(Image_critical, (double)(Canvas1.Height / 3 * rt_x));
                }
                else
                    text1.Text = "조심하세요";
            }
            else
            {
                Image_critical.Visibility = System.Windows.Visibility.Collapsed;
                text1.Text = "조심하세요.";
            }

            #endregion 가장 작은 사각형이 많은 곳의 셀(큰 사각형) 경고 위치

            //거리값 차이에 따른 작은 사각형 색깔
            if (myFlag == false)
                for (int i = 0; i < row * col; i++)
                    m_rect[rank[i].i, rank[i].j].Stroke = new SolidColorBrush(Color.FromRgb(0, (byte)((255 / cnt * i)), 255));

            #endregion 셀 정보

        }

        void visible_secondCritical(int i, int j) // 9개의 셀(큰사각형) 에서 경고 표시를 낼 것을 나타냄
        {
            int myShell = (i * 3) + j;

            switch (myShell)
            {

                case 0:
                    Image_LongCritical1.Visibility = Visibility.Visible;
                    break;
                case 1:
                    Image_LongCritical2.Visibility = Visibility.Visible;
                    break;
                case 2:
                    Image_LongCritical3.Visibility = Visibility.Visible;
                    break;
                case 3:
                    Image_LongCritical4.Visibility = Visibility.Visible;
                    break;
                case 4:
                    Image_LongCritical5.Visibility = Visibility.Visible;
                    break;
                case 5:
                    Image_LongCritical6.Visibility = Visibility.Visible;
                    break;
                case 6:
                    Image_LongCritical7.Visibility = Visibility.Visible;
                    break;
                case 7:
                    Image_LongCritical8.Visibility = Visibility.Visible;
                    break;
                case 8:
                    Image_LongCritical9.Visibility = Visibility.Visible;
                    break;
                default:
                    return;
            }

        }
        void Collapsed_secondCritical(int i, int j) // 9개의 셀(큰 사각형) 중에서 경고가 사라질 것을 찾아 숨김
        {
            int myShell = (i * 3) + j;

            switch (myShell)
            {

                case 0:
                    Image_LongCritical1.Visibility = Visibility.Collapsed;
                    break;
                case 1:
                    Image_LongCritical2.Visibility = Visibility.Collapsed;
                    break;
                case 2:
                    Image_LongCritical3.Visibility = Visibility.Collapsed;
                    break;
                case 3:
                    Image_LongCritical4.Visibility = Visibility.Collapsed;
                    break;
                case 4:
                    Image_LongCritical5.Visibility = Visibility.Collapsed;
                    break;
                case 5:
                    Image_LongCritical6.Visibility = Visibility.Collapsed;
                    break;
                case 6:
                    Image_LongCritical7.Visibility = Visibility.Collapsed;
                    break;
                case 7:
                    Image_LongCritical8.Visibility = Visibility.Collapsed;
                    break;
                case 8:
                    Image_LongCritical9.Visibility = Visibility.Collapsed;
                    break;
                default:
                    return;
            }
        }

        void SetRGB(byte[] nPlayers, int nPos, byte r, byte g, byte b)
        {
            nPlayers[nPos + 2] = r;
            nPlayers[nPos + 1] = g;
            nPlayers[nPos + 0] = b;
        }

        byte[] GetRGB(DepthImageFrame PImage, short[] depthFrame, DepthImageStream depthStream)
        {

            byte[] rgbs = new byte[PImage.Width * PImage.Height * 4];

            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    rect_cnt[i, j] = 0;

            //작은 사각형 값의 초기화
            for (int i = 0; i < col; i++)
            {
                for (int j = 0; j < row; j++)
                {
                    point[i, j].min_X = int.MaxValue;
                    point[i, j].min_Y = int.MaxValue;
                    point[i, j].max_X = int.MinValue;
                    point[i, j].max_Y = int.MinValue;
                    point[i, j].sGape = int.MaxValue;
                    point[i, j].bGape = int.MinValue;

                }
            }


            for (int i16 = 0, i32 = 0; i16 < depthFrame.Length && i32 < rgbs.Length; i16++, i32 += 4)
            {
                int nDistance = depthFrame[i16] >>
                                DepthImageFrame.PlayerIndexBitmaskWidth;

                int nX = i16 % PImage.Width;
                int nY = i16 / PImage.Width;

                int i = (nY - 1) / (PImage.Height / col);
                int j = (nX - 1) / (PImage.Width / row);


                int nColor = 0xFF * nDistance / 4000;

                if (nDistance < 1000 && nDistance >= 0)
                {
                    SetRGB(rgbs, i32, 0xff, 0, 0);
                    point[i, j].min_X = Math.Min(point[i, j].min_X, nX); //작은 사각형 정보 입력
                    point[i, j].min_Y = Math.Min(point[i, j].min_Y, nY);
                    point[i, j].max_X = Math.Max(point[i, j].max_X, nX);
                    point[i, j].max_Y = Math.Max(point[i, j].max_Y, nY);
                    point[i, j].sGape = Math.Min(point[i, j].sGape, nDistance);
                    point[i, j].bGape = Math.Max(point[i, j].bGape, nDistance);

                    rect_cnt[i / 4, j / 4]++;

                }
                else if (nDistance < 2000 && nDistance > 1000)
                {
                    SetRGB(rgbs, i32, 0xFF, 0xFF, 0);
                }
                else
                    SetRGB(rgbs, i32, (byte)nColor, (byte)nColor, (byte)nColor);

            }

            return rgbs;
        }

        public void callBeep() //소리 출력
        {

            switch (big_rect) //현재 가장 많은 작은 사각형이 있는 위치에서 소리를 냄
            {
                    
                case 0:
                    speach.SpeakAsync("좌상");
                    break;
                case 1:
                    speach.SpeakAsync("상");
                    break;
                case 2:
                    speach.SpeakAsync("우상");
                    break;
                case 3:
                    speach.SpeakAsync("좌");
                    break;
                case 4:
                    speach.SpeakAsync("정면");
                    break;
                case 5:
                    speach.SpeakAsync("우");
                    break;
                case 6:
                    speach.SpeakAsync("좌하");
                    break;
                case 7:
                    speach.SpeakAsync("하");
                    break;
                case 8:
                    speach.SpeakAsync("우하");
                    break;
                case 9:
                    speach.SpeakAsync("조심하십시오");
                    break;
                default:
                    return;
            }
        }

        void sort(int n) // 거리차에 따른 정렬
        {
            for (int i = 0; i < n; i++)
                for (int j = i + 1; j < n; j++)
                    if (rank[i].Gape > rank[j].Gape)
                    {
                        makeDepthRank c = new makeDepthRank();
                        c = rank[i];
                        rank[i] = rank[j];
                        rank[j] = c;
                    }
        }

        private void button1_Click(object sender, RoutedEventArgs e) //버튼이 눌러지면 작은 사각형 표시 방법 변환
        {
            if (myFlag)
            {
                myFlag = false;
                button1.Content = "각 셀의 거리값 차이에 대한 색깔";
            }
            else
            {
                myFlag = true;
                button1.Content = "비율에 대한 색깔";
            }
        }

    }
        #endregion 장애물소스
}