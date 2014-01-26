using System;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using Microsoft.Kinect;
using System.Windows.Shapes;
using System.Diagnostics;
using System.Collections.Generic;
using System.Globalization;
using System.Timers;
using System.Windows.Threading;

namespace SkeletonTracking
{
    /// <summary>
    /// MainWindow.xaml の相互作用ロジック
    /// </summary>
    public partial class MainWindow : Window
    {
        readonly int Bgr32BytesPerPixel = PixelFormats.Bgr32.BitsPerPixel / 8;
        /*コード追加*********************/
        LinkedList<Point> Points = new LinkedList<Point>();//X,Y座標のみ
        LinkedList<SkeletonPoint> PPositions = new LinkedList<SkeletonPoint>();//キャプチャユーザーの右手のposition．一時保存用
        LinkedList<SkeletonPoint> Vertex = new LinkedList<SkeletonPoint>(); //オブジェクト認識した結果の頂点とか
        LinkedList<SkeletonPoint> PPositions1 = new LinkedList<SkeletonPoint>();//キャプチャしたプレイヤー１の右手のposition
        LinkedList<SkeletonPoint> PPositions2 = new LinkedList<SkeletonPoint>();//キャプチャしたプレイヤー2の右手のposition
        LinkedList<ColorImagePoint> BPositions1;
        LinkedList<ColorImagePoint> BPositions2;
        bool is_player1_turn = true;
        bool is_moving = false;
        int move_type = 0;
        int moving_count = 0;

        int player1_hp = 100; //player1用の体力 TODO intでいいのかな？
        int player2_hp = 100;
        bool t_flag = false;
        DispatcherTimer dispatcherTimer;

        /********************************/
        /**onizawa**/
        static int TOTAL_SAMPLE = 64;
        static double CIRCLE_THRESHOLD = 0.8;//0.4; //円かどうか判断するための閾値
        static double VERTEX_THRESHOLD = 0.5; //多角形の頂点を判定するための閾値
        static int VERTEX_THRESHOLD_COUNT = 3; //多角形の頂点を判定する際、何点連続していたら閾値を超えたと判定するか
        /**onizawaEND**/

        public MainWindow()
        {
            try
            {
                InitializeComponent();

                // Kinectが接続されているかどうかを確認する
                if (KinectSensor.KinectSensors.Count == 0)
                {
                    throw new Exception("Kinectを接続してください");
                }

                // Kinectの動作を開始する
                StartKinect(KinectSensor.KinectSensors[0]);
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
                Close();
            }
        }

        /// <summary>
        /// Kinectの動作を開始する
        /// </summary>
        /// <param name="kinect"></param>
        private void StartKinect(KinectSensor kinect)
        {
            // RGBカメラを有効にして、フレーム更新イベントを登録する
            kinect.ColorStream.Enable();
            kinect.ColorFrameReady +=
              new EventHandler<ColorImageFrameReadyEventArgs>(kinect_ColorFrameReady);

            // 距離カメラを有効にして、フレーム更新イベントを登録する
            kinect.DepthStream.Enable();
            kinect.DepthFrameReady +=
              new EventHandler<DepthImageFrameReadyEventArgs>(kinect_DepthFrameReady);

            // スケルトンを有効にして、フレーム更新イベントを登録する
            kinect.SkeletonFrameReady +=
              new EventHandler<SkeletonFrameReadyEventArgs>(kinect_SkeletonFrameReady);
            kinect.SkeletonStream.Enable();

            // Kinectの動作を開始する
            kinect.Start();

            // defaultモードとnearモードの切り替え
            comboBoxRange.Items.Clear();
            foreach (var range in Enum.GetValues(typeof(DepthRange)))
            {
                comboBoxRange.Items.Add(range.ToString());
            }

            comboBoxRange.SelectedIndex = 0;
        }

        /// <summary>
        /// Kinectの動作を停止する
        /// </summary>
        /// <param name="kinect"></param>
        private void StopKinect(KinectSensor kinect)
        {
            if (kinect != null)
            {
                if (kinect.IsRunning)
                {
                    // フレーム更新イベントを削除する
                    kinect.ColorFrameReady -= kinect_ColorFrameReady;
                    kinect.DepthFrameReady -= kinect_DepthFrameReady;
                    kinect.SkeletonFrameReady -= kinect_SkeletonFrameReady;

                    // Kinectの停止と、ネイティブリソースを解放する
                    kinect.Stop();
                    kinect.Dispose();

                    imageRgb.Source = null;
                    imageDepth.Source = null;
                }
            }
        }

        /// <summary>
        /// RGBカメラのフレーム更新イベント
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        void kinect_ColorFrameReady(object sender, ColorImageFrameReadyEventArgs e)
        {
            try
            {
                // RGBカメラのフレームデータを取得する
                using (ColorImageFrame colorFrame = e.OpenColorImageFrame())
                {
                    if (colorFrame != null)
                    {
                        // RGBカメラのピクセルデータを取得する
                        byte[] colorPixel = new byte[colorFrame.PixelDataLength];
                        colorFrame.CopyPixelDataTo(colorPixel);

                        // ピクセルデータをビットマップに変換する
                        imageRgb.Source = BitmapSource.Create(colorFrame.Width, colorFrame.Height, 96, 96,
                            PixelFormats.Bgr32, null, colorPixel, colorFrame.Width * colorFrame.BytesPerPixel);
                    }
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
        }

        /// <summary>
        /// 距離カメラのフレーム更新イベント
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        void kinect_DepthFrameReady(object sender, DepthImageFrameReadyEventArgs e)
        {

            try
            {
                // センサーのインスタンスを取得する
                KinectSensor kinect = sender as KinectSensor;
                if (kinect == null)
                {
                    return;
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
        }

        /// <summary>
        /// スケルトンのフレーム更新イベント
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        void kinect_SkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            bool tekitounaflag = false;
            try
            {
                // Kinectのインスタンスを取得する
                KinectSensor kinect = sender as KinectSensor;
                if (kinect == null)
                {
                    return;
                }

                // スケルトンのフレームを取得する
                int playerId = 1;
                using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame())
                {
                    if (skeletonFrame != null)
                    {
                        // 最初に誰かが対象になった時
                        DrawSkeleton(kinect, skeletonFrame, playerId);
                    }
                }

            }
            catch (Exception ex)
            {
                
                MessageBox.Show(ex.ToString()); //onizawa編集
            }
        }

        /// <summary>
        /// スケルトンを描画する
        /// </summary>
        /// <param name="kinect"></param>
        /// <param name="skeletonFrame"></param>
        private void DrawSkeleton(KinectSensor kinect, SkeletonFrame skeletonFrame, int playerId)
        {
            // スケルトンのデータを取得する
            Skeleton[] skeletons = new Skeleton[skeletonFrame.SkeletonArrayLength];
            skeletonFrame.CopySkeletonDataTo(skeletons);

            canvasSkeleton.Children.Clear();
            // トラッキングされているスケルトンのジョイントを描画する
            foreach (Skeleton skeleton in skeletons)
            {
                // スケルトンがトラッキング状態(デフォルトモード)の場合は、ジョイントを描画する
                if (skeleton.TrackingState == SkeletonTrackingState.Tracked)
                {
                    //特定の部分のみの描画の準備(右手のjointについて)
                    if (skeleton.Joints[JointType.Head].TrackingState != JointTrackingState.NotTracked)
                    {
                        DrawEllipse(kinect, skeleton.Joints[JointType.HandRight].Position, playerId);
                        //Debug.WriteLine(skeleton.Joints[JointType.HandLeft].Position.Y);        //コンソールに左手の座標を表示(正なら描写のjoint取得)
                        if (skeleton.Joints[JointType.HandLeft].Position.Y > 0)                 //左手を挙げた時にif文に入る
                        {
                            //wasPlayerId1Tracked = true;
                            if (Vertex != null) Vertex.Clear();
                            //Debug.WriteLine("OK!!!!!!");
                            //リスト作成箇所！！！！
                            Point point = new Point(skeleton.Joints[JointType.HandRight].Position.X, skeleton.Joints[JointType.HandRight].Position.Y);
                            Points.AddLast(point);//鬼沢に渡す情報
                            PPositions.AddLast(skeleton.Joints[JointType.HandRight].Position);//ユーザの右手のキャプチャしたPosiotion情報
                        }
                        
                        else if (skeleton.Joints[JointType.HandLeft].Position.Y < 0)  //onizawa
                        {
                            //if (PPositions.First == PPositions.Last) PPositions.Clear();
                            if (PPositions.Count > 5 && PPositions1.Count == 0)
                            {
                                foreach (SkeletonPoint skeletonpoint in PPositions)
                                {
                                    PPositions1.AddLast(skeletonpoint);
                                }
                                PPositions.Clear();
                            }
                            if (PPositions.Count > 5 && PPositions1.Count != 0 && PPositions2.Count == 0)
                            {
                                foreach (SkeletonPoint skeletonpoint in PPositions)
                                {
                                    PPositions2.AddLast(skeletonpoint);
                                }
                                PPositions.Clear();
                            }
                        }
                    }


                    //キャプチャしたjointを描画

                    if (PPositions1.Count > 5 && PPositions2.Count == 0)
                    {
                        foreach (SkeletonPoint skeletonpoint in PPositions1)
                        {
                            DrawEllipse(kinect, skeletonpoint, 1);
                        }
                    }

                    /* 一時的にコメントアウト
                    if (PPositions2.Count > 5)
                    {
                        foreach (SkeletonPoint skeletonpoint in PPositions2)
                        {
                            DrawEllipse(kinect, skeletonpoint, 2);
                        }
                    }
                     * */
                    // ユーザaとユーザbがそれぞれ書き終わったら，ステータス表示
                    // というかBattleStageに入るべきか
                    if (PPositions1.Count > 5 && PPositions2.Count > 5)
                    {
                        //textblock1.Text = "100";
                        //textblock2.Text = "100";
                        DrawBattleStage(kinect);
                        DrawBattleCharacters();
                        //tekitounaflag = true;
                        //return true;
                    }
                }
            }
        }

        /**onizawa**/
        private ObjectInfo analyzePoints(LinkedList<Point> Points)
        {
            int n = Points.Count;
            if (n < 1) return new ObjectInfo(-1, null); 

            Point[] points = new Point[n];
            int indexForNextLoop=0;//下のループのためだけの変数
            foreach (Point p in Points) {
                points[indexForNextLoop] = new Point(p.x+2,p.y+2); // 全ての座標を正にするために、＋２．返す時は、ObjectInfo内で-2してから返す
                indexForNextLoop++;
            }

            //点のresample (論文を参考に)
            double I = pathLength(points, n)/(TOTAL_SAMPLE-1);
            double D = 0;
            Point[] new_points = new Point[TOTAL_SAMPLE];
            new_points[0] = points[0];
            int new_points_i = 1;

            for (int i=1; i<n; i++)
            {
                double d = distance(points[i-1], points[i]);
                if ((D + d) >= I)
                {
                    double x = points[i - 1].x + ((I - D) / d) * (points[i].x - points[i - 1].x);
                    double y = points[i - 1].y + ((I - D) / d) * (points[i].y - points[i - 1].y);
                    Point q = new Point((float)x, (float)y);
                    new_points[new_points_i] = q;
                    new_points_i++;
                    points[i-1] = q;
                    i--;
                    D = 0;
                }
                else
                {
                    D += d;
                }
            }
            points = new_points;
            for (int i = TOTAL_SAMPLE - 1; i > 0; i--) //応急処置
            {
                if (points[i] != null)
                {
                    n = i + 1;
                    break;
                }
            }
            //resampleここまで

            Point mp = meanPoint(points, n);
            Debug.WriteLine("mean point x:"+mp.x+" y:"+mp.y);

            double[] ds = new double[n]; //distances
            double dmax = 0.0;
            double dmin = Double.MaxValue;
            double[] vs = new double[n - 1]; //velocities
            Boolean[] vp = new Boolean[n - 1]; // velocity is plus


            ds[0] = distance(mp, points[0]);
            for (int i = 1; i < n; i++)
            {
                //Debug.WriteLine("point"+i+" x:" + points[i].x + " y:" + points[i].y);
                ds[i] = distance(mp, points[i]);
                //Debug.WriteLine(ds[i]);
                if (ds[i] > dmax) dmax = ds[i];
                if (ds[i] < dmin) dmin = ds[i];
                vs[i - 1] = ds[i] - ds[i - 1];
                vp[i - 1] = vs[i - 1] > 0; //TODO: 最初動いていない時はfalseでよいのか？
            }
            Debug.WriteLine("dmax=" + dmax);
            Debug.WriteLine("dmin=" + dmin);

            double circle_rate = ((dmax - dmin)/dmin);
            Debug.WriteLine("circle_rate:"+circle_rate);

            if ( circle_rate < CIRCLE_THRESHOLD)
            {
                int type = 0;
                Point[] vertex = { mp, points[0] };
                Debug.WriteLine("円");

                return new ObjectInfo(type, vertex);
            }
            else
            {
                Point[] vertex = getVertex(points, ds, dmax, dmin, n);
                int type = vertex.Length;
                if (type < 2) return new ObjectInfo(-1, null);
                Debug.WriteLine(type + "角形");

                return new ObjectInfo(type, vertex);
            }
        }

        //TODO 要検討
        private Point[] getVertex(Point[] points, double[] ds, double dmax, double dmin,int n)
        {
            double dm = (dmax + dmin) / 2;
            dmax = (dmax - dmax) * VERTEX_THRESHOLD + dm;
            dmin = dm - (dm - dmin) * VERTEX_THRESHOLD;
            int count = 0; //何点連続でdmaxかdminを超えたか
            int[] os = new int[n]; //OverState 各dsがdmaxを超えている状態なら1,dminを超えている状態なら-1,それ以外は0
            int pos = 0; //現在のOverState

            Func<double, int> OverState = d =>
                {
                    if (d > dmax) return 1;
                    if (d < dmin) return -1;
                    return 0;
                };

            for (int i = 0; i < n; i++)
            {
                os[i] = OverState(ds[i]);
            }

            //os=0がVERTEX_THRESHOLD_COUNT回以上連続しているところを探す
            int over_count=0;
            for (int i = 0; i < n; i++)
            {
                if (os[i] == 0)
                {
                    count++;
                    if (count >= VERTEX_THRESHOLD_COUNT)
                    {
                        over_count = i - VERTEX_THRESHOLD_COUNT + 1;
                    }
                }
                else
                {
                    count = 0;
                }
            }

            //最初にVERTEX_THRESHOLD_COUNT回以上os=0となるようにする。
            Point[] tmpP = new Point[n];
            double[] tmpd = new double[n];
            int[] tmpo = new int[n];
            for (int i = 0; i < n - over_count; i++) {
                tmpP[i] = points[i + over_count];
                tmpd[i] = ds[i + over_count];
                tmpo[i] = os[i + over_count];
            }
            for (int i = 0; i < over_count; i++) 
            {
                tmpP[i+n-over_count] = points[i];
                tmpd[i + n - over_count] = ds[i];
                tmpo[i + n - over_count] = os[i];
            }
            points = tmpP;
            ds = tmpd;


            count = 0;
            int start = 0;
            LinkedList<int> vertex_points = new LinkedList<int>(); //pointsの何個目の要素が頂点か
            for (int i = 0; i < n; i++) 
            {
                if (pos != OverState(ds[i]))
                {
                    count++;
                    if (count >= VERTEX_THRESHOLD_COUNT) 
                    {
                        if (pos == 1)
                        {
                            int max_n = getMax(ds, start, i + 1 - VERTEX_THRESHOLD_COUNT);
                            vertex_points.AddLast(max_n);
                        }

                        pos = OverState(ds[i]);
                        start = i + 1 - VERTEX_THRESHOLD_COUNT;
                    }
                }
                else
                {
                    count = 0;
                }
            }

            Point[] vertex = new Point[vertex_points.Count];

            int index = 0;
            foreach (int p_num in vertex_points)
            {
                vertex[index] = points[p_num];
                index++;
            }

            return vertex;
        }

        private int getMax(double[] ds, int start, int end)
        {
            double max = 0;
            int max_n = start;
            int n = end - start + 1;
            for (int i = 0; i < n; i++)
            {
                if (max < ds[start + i])
                {
                    max = ds[start + i];
                    max_n = start + i;
                }
            }

            return max_n;
        }

        private double mean(double[] d, int n)
        {
            double mean = 0;
            for (int i = 0; i < n; i++)
            {
                mean += d[i];
            }

            return mean / n;
        }

        private Point meanPoint(Point[] points, int n)
        {
            Point mp = new Point(0, 0);

            for (int i = 0; i < n; i++)
            {
                try//応急処置
                {
                    mp.x += points[i].x;
                    mp.y += points[i].y;
                }
                catch (Exception e) 
                {
                    Debug.WriteLine("error at meanPoint");
                    Debug.WriteLine("i="+i);
                }
            }

            mp.x /= n;
            mp.y /= n;

            return mp;
        }

        private double pathLength(Point[] points, int n)
        {
            double d = 0;

            for (int i = 1; i < n; i++)
            {
                d += distance(points[i-1], points[i]);
            }

            return d;
        }

        private double distance(Point p1, Point p2)
        {
            float dx = p1.x-p2.x;
            float dy = p1.y-p2.y;
            double d = Math.Sqrt(
                        Math.Pow(dx, 2) + Math.Pow(dy, 2)
                       );
            return d;
        }

        private Point[] loadPoints()
        {
            throw new NotImplementedException();
        }

    /**onizawaEND**/

        /// <summary>
        /// ジョイントの円を描く
        /// </summary>
        /// <param name="kinect"></param>
        /// <param name="position"></param>
        private void DrawEllipse(KinectSensor kinect, SkeletonPoint position, int playerId)
        {
            const int R = 5;

            // スケルトンの座標を、RGBカメラの座標に変換する
            ColorImagePoint point = kinect.MapSkeletonPointToColor(position, kinect.ColorStream.Format);

            // 座標を画面のサイズに変換する
            point.X = (int)ScaleTo(point.X, kinect.ColorStream.FrameWidth, canvasSkeleton.Width);
            point.Y = (int)ScaleTo(point.Y, kinect.ColorStream.FrameHeight, canvasSkeleton.Height);

            // 円を描く playerId == 1;
            if (playerId == 1)
            {
                canvasSkeleton.Children.Add(new Ellipse()
                {
                    Fill = new SolidColorBrush(Colors.Red),
                    Margin = new Thickness(point.X - R, point.Y - R, 0, 0),
                    Width = R * 2,
                    Height = R * 2,
                });
            }
            else
            {
                canvasSkeleton.Children.Add(new Ellipse()
                {
                    Fill = new SolidColorBrush(Colors.Green),
                    Margin = new Thickness(point.X - R, point.Y - R, 0, 0),
                    Width = R * 2,
                    Height = R * 2,
                });
            }
        }
        private void DrawEllipse(ColorImagePoint point, int playerId)
        {
            const int R = 5;

            // 円を描く playerId == 1;
            if (playerId == 1)
            {
                canvasSkeleton.Children.Add(new Ellipse()
                {
                    Fill = new SolidColorBrush(Colors.Red),
                    Margin = new Thickness(point.X - R, point.Y - R, 0, 0),
                    Width = R * 2,
                    Height = R * 2,
                });
            }
            else
            {
                canvasSkeleton.Children.Add(new Ellipse()
                {
                    Fill = new SolidColorBrush(Colors.Green),
                    Margin = new Thickness(point.X - R, point.Y - R, 0, 0),
                    Width = R * 2,
                    Height = R * 2,
                });
            }
        }

        private void DrawBattleStage(KinectSensor kinect)
        {
            if (t_flag)
            {
                return;
            }
            t_flag = true;
            //StopKinect(kinect);
            dispatcherTimer = new DispatcherTimer(DispatcherPriority.Normal);
            dispatcherTimer.Interval = TimeSpan.FromMilliseconds(10);
            
            BPositions1 = toBattlePosition(PPositions1, 1, kinect);
            BPositions2 = toBattlePosition(PPositions2, 2, kinect);
            //StopKinect(kinect);
            dispatcherTimer.Tick += dispatcherTimer_Tick;
            dispatcherTimer.Start();


            //textblock1.Text = (player1_hp--).ToString();
            //System.Threading.Thread.Sleep(1000);

            //TODO ダメージ表示処理など入れる
        }
        private void dispatcherTimer_Tick(object sender, EventArgs e)
        {
            if (!is_moving)
            {
                move_type = new System.Random().Next(3);
                is_moving = true;
            }
            if (is_moving)
            {
                move();
            }

        }

        private void move()
        {
            if (moving_count <= 50)
            {
                move_character(1, 0);
            }
            else if (moving_count <= 60)
            {
                int dy = (0 - ((moving_count - 50) / 2));
                move_character(5, -dy*dy);
            }
            else if (moving_count <= 70)
            {
                int dy = (0 + ((moving_count - 60) / 2));
                move_character(5, dy*dy);
            }
            else if (moving_count <= 120)
            {
                move_character(-3, 0);
            }
            else 
            {
                is_moving = false;
                is_player1_turn = !is_player1_turn;
                moving_count = 0;
                return;
            }

            if (moving_count == 0) textblock1.Text = "1";
            if (moving_count == 50) textblock1.Text = "2";
            if (moving_count == 60) textblock1.Text = "3";
            if (moving_count == 70) textblock1.Text = "4";
            if (moving_count == 120) textblock1.Text = "5";

            moving_count++;
        }

        private void move_character(int dx, int dy)
        {
            LinkedList<ColorImagePoint> newPosition = new LinkedList<ColorImagePoint>();
            LinkedList<ColorImagePoint> rawPosition;
            if (is_player1_turn)
            {
                rawPosition = BPositions1;
            }
            else
            {
                rawPosition = BPositions2;
                dx = -dx;
            }

            foreach (ColorImagePoint point in rawPosition)
            {
                ColorImagePoint newPoint = new ColorImagePoint();
                newPoint.X = point.X + dx;
                newPoint.Y = point.Y + dy;
                newPosition.AddLast(newPoint);
            }
            if (is_player1_turn)
            {
                BPositions1 = newPosition;
            }
            else
            {
                BPositions2 = newPosition;
            }
        }

        private void DrawBattleCharacters()
        {
            foreach (ColorImagePoint skeletonpoint in BPositions1)
            {
                DrawEllipse(skeletonpoint, 1);
            }
            foreach (ColorImagePoint skeletonpoint in BPositions2)
            {
                DrawEllipse(skeletonpoint, 2);
            }
        }

        private LinkedList<ColorImagePoint> toBattlePosition(LinkedList<SkeletonPoint> PPositions, int field, KinectSensor kinect)
        {
            LinkedList<ColorImagePoint> BattlePositions = new LinkedList<ColorImagePoint>();

            //座標の変換と、最大最小のXYの取得
            int minX = 1000;
            int maxX = 0;
            int minY = 1000;
            int maxY = 0;
            foreach (SkeletonPoint position in PPositions)
            {
                ColorImagePoint point = kinect.MapSkeletonPointToColor(position, kinect.ColorStream.Format);
                // 座標を画面のサイズに変換する
                point.X = (int)ScaleTo(point.X, kinect.ColorStream.FrameWidth, canvasSkeleton.Width);
                point.Y = (int)ScaleTo(point.Y, kinect.ColorStream.FrameHeight, canvasSkeleton.Height);
                //Console.WriteLine("X: {0}, Y: {1}  ({2})", point.X, point.Y, field);

                if (point.X < minX) minX = point.X;
                if (point.X > maxX) maxX = point.X;
                if (point.Y < minY) minY = point.Y;
                if (point.Y > maxY) maxY = point.Y;

                BattlePositions.AddLast(point);
            }

            //一番左上の座標がbase座標になるよう移動
            int baseX = -40 + 300 * (field - 1);
            int baseY = 20;
            int diffX = minX - baseX;
            int diffY = minY - baseY;
            LinkedList<ColorImagePoint> movedBattlePositions = new LinkedList<ColorImagePoint>();
            foreach (ColorImagePoint point in BattlePositions)
            {
                ColorImagePoint movedPoint = new ColorImagePoint();
                movedPoint.X = point.X - diffX;
                movedPoint.Y = point.Y - diffY;
                movedBattlePositions.AddLast(movedPoint);
            }

            //maxWidth, maxHeight内に収まるよう、スケールする
            //また、センターによせる
            int maxWidth = 200;
            int maxHeight = 180;
            int width = maxX - minX;
            int height = maxY - minY;
            double ratioX = 1.0;
            double ratioY = 1.0;
            int cx = 0;
            int cy = 0;

            if (width > maxWidth)
            {
                ratioX = (double)maxWidth / width;
            }
            else
            {
                cx = (maxWidth - width) / 2;
            }
            if (height > maxHeight)
            {
                ratioY = (double)maxHeight / height;
            }
            else
            {
                cy = (maxHeight - height) / 2;
            }

            LinkedList<ColorImagePoint> scaledBattlePositions = new LinkedList<ColorImagePoint>();
                foreach (ColorImagePoint point in movedBattlePositions)
                {
                    ColorImagePoint scaledPoint = new ColorImagePoint();
                    scaledPoint.X = (int)((point.X - baseX) * ratioX) + baseX + cx;
                    scaledPoint.Y = (int)((point.Y - baseY) * ratioY) + baseY + cy;
                    scaledBattlePositions.AddLast(scaledPoint);
                }

            return scaledBattlePositions;
           
        }

        /// <summary>
        /// スケールを変換する
        /// </summary>
        /// <param name="value"></param>
        /// <param name="source"></param>
        /// <param name="dest"></param>
        /// <returns></returns>
        double ScaleTo(double value, double source, double dest)
        {
            return (value * dest) / source;
        }

        /// <summary>
        /// 距離カメラの通常/近接モード変更イベント
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void comboBoxRange_SelectionChanged(object sender,
          System.Windows.Controls.SelectionChangedEventArgs e)
        {
            try
            {
                KinectSensor.KinectSensors[0].DepthStream.Range = (DepthRange)comboBoxRange.SelectedIndex;
            }
            catch (Exception)
            {
                comboBoxRange.SelectedIndex = 0;
            }
        }

        /// <summary>
        /// Windowが閉じられるときのイベント
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            StopKinect(KinectSensor.KinectSensors[0]);
        }

    }
    //鬼沢に渡すjointのX,Y座標
    class Point
    {
        public float x;
        public float y;
        public Point(float x, float y)
        {
            this.x = x;
            this.y = y;
        }
    }


    /**onizawa**/
    class ObjectInfo
    {
        public int type; //エラーの場合-1,円なら0, それ以外なら頂点の数（線なら2）
        public Point[] vertex; //円ならvertex[0]が中心点で、vertex[1]が円上の点の一つ
        //円以外なら各頂点の座標

        public ObjectInfo(int type, Point[] vertex_in)
        {
            if (type > -1) 
            {
                int n = vertex_in.Length;
                for (int i = 0; i < n; i++) //最初に全ての点を+2したので元に戻す
                {
                    vertex_in[i] = new Point(vertex_in[i].x - 2, vertex_in[i].y - 2);
                }
            }

            this.type = type;
            this.vertex = vertex_in;
            //if (type < 0) Debug.WriteLine("ObjectInfo: エラー");
        }
    }
    /**onizawaEND**/
}
