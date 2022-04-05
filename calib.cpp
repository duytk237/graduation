#include "calib.h"

Calib::Calib(QObject *parent)
    :QThread { parent }
{

}


using namespace cv;
using namespace dnn;
using namespace std;


Mat cameraMatrix = (Mat_<double>(3,3) <<   709.610699,        0, 309.137357,
                                                  0,  945.254236,  209.405394,
                                                  0,        0,        1);
Mat new_cameraMatrix = (Mat_<double>(3,3) <<   714.6498,        0, 311.36056,
                                                    0,  955.9724,  205.6297,
                                                    0,        0,        1);

Mat distCoeffs = (Mat_<double>(1,5) <<   -1.061549e-01, 2.486004e+00, -1.159369e-02, 7.784208e-03, -8.79829e+00);
UMat U_cameraMatrix = cameraMatrix.getUMat(ACCESS_READ);
UMat U_distCoeffs = distCoeffs.getUMat(ACCESS_READ);
UMat U_newCameraMatrix = new_cameraMatrix.getUMat(ACCESS_READ);


float z_cam = 290 ;//z camera = 287 mm
float scale = (16.0/44.685);

//Set limit of x,y,theta limit
double speed_conveyor = 37.5;                 //(mm/s)
int x_limit = 1;                            //(pixel)
int y_limit = int(speed_conveyor*2/37.5);     //(pixel)
int theta_limit = 2;                        //degree

vector<KalmanFilter> KF;
vector<Mat> state;
vector<Mat> meas;

void Transfer_CorRobot(float x_cam, float y_cam, float z_cam,
                      float &x_robot, float &y_robot, float &z_robot);
void Set_KalmanFilter(vector<KalmanFilter>& KF, vector<Mat>& state, vector<Mat>& meas);

void Calib::run()
{
    current_point = 0;
   while (true)
   {
       if(count_delay>=20000)
       {
           count_delay = 0;
           //Time per frame
           precTick = ticks;
           ticks = (double) cv::getTickCount();
           dT = (ticks - precTick) / cv::getTickFrequency(); //seconds
           //Predict with Kalman Filter
           for (size_t i = 0; i < Trans_buffer.size(); ++i)
           {
               if(Trans_buffer[i][5] != 0)   // if Flag Already Set the Initialization == true
               {
                   // >>>> Matrix A
                   //KF[i].transitionMatrix.at<float>(2) = dT;
                   KF[i].transitionMatrix.at<float>(9) = dT;
                   state[i] = KF[i].predict();
                   //cout << "X_predict = " << state[i].at<float>(0) << "Y_predict = " << state[i].at<float>(1)<< endl;
                   //Update Trans_buffer
                   Trans_buffer[i][1] = state[i].at<float>(0);
                   Trans_buffer[i][2] = state[i].at<float>(1);
                   Trans_buffer[i][4] = state[i].at<float>(4);
                   Trans_buffer[i][0] = round(state[i].at<float>(5));
                   //qDebug()<<"X_pre"<<i<<"= "<<Trans_buffer[i][1]<<"Y_pre = "<<Trans_buffer[i][2]<<state[i].at<float>(3)<<state[i].at<float>(2)<<endl;
               }
           }

           for (size_t i = 0; i < Trans_buffer.size(); ++i)
           {
               if (Trans_buffer[i][2]< 62 && Trans_buffer[i][2] > 58)
               {
                  //qDebug()<<pre_X << endl;
                  //qDebug()<<Trans_buffer[i][2] << endl;
//                  if(!((pre_X - 0.5) < Trans_buffer[i][1] && Trans_buffer[i][1] < (pre_X + 0.5)&& (pre_id==Trans_buffer[i][0])))
                  if (Trans_buffer[i][7] == 0 && Trans_buffer[i][1] < 355)
                  {
                       //Add data
                      Ready = true;
                      vector<double> buffer_tmp;
                      buffer_tmp.push_back(Trans_buffer[i][0]);
                      buffer_tmp.push_back(Trans_buffer[i][1]);
                      buffer_tmp.push_back(Trans_buffer[i][2]);
                      buffer_tmp.push_back(Trans_buffer[i][3]);
                      buffer_tmp.push_back(Trans_buffer[i][4]);
                      Send_buffer.push_back(buffer_tmp);
//                      pre_id = Trans_buffer[i][0];
//                      pre_X = Trans_buffer[i][1];
                      Trans_buffer[i][7] = 2222;
                      qDebug()<<"Current point = " << current_point <<"; V_y =  " <<state[i].at<float>(3)<<endl;
                      current_point--;

                  }
               }

           }


       }
       else count_delay ++;
       if (Set==true)
        {
           Set = false;
           std::vector<std::vector<double>> copy_buffer;
           if (!buffer.empty())
           {
               if(buffer.size() > Trans_buffer.size())      //detect if there is new point
               {
                   size_t m = current_point;
                   for(size_t t = 1; t <= buffer.size() - m; t++)
                   {
                       current_point++;                     //Add current point
                       // Trans_buffer
                       vector<double> buffer_tmp;
                       buffer_tmp.push_back(2222);          //0 Id
                       buffer_tmp.push_back(340);           //1 x
                       buffer_tmp.push_back(140);           //2 y
                       buffer_tmp.push_back(2222);          //3 z
                       buffer_tmp.push_back(2222);          //4 theta
                       buffer_tmp.push_back(0);             //5 Flag Already Set the Initialization
                       buffer_tmp.push_back(0);             //6 LIFE_TIME
                       buffer_tmp.push_back(0);             //7 Had sent
                       Trans_buffer.push_back(buffer_tmp);
                       Set_KalmanFilter(KF, state, meas);   // Create KF
                   }
               }               
               for (size_t i = 0; i < Trans_buffer.size(); ++i)
               {
                   copy_buffer.push_back(Trans_buffer.at(i));
               }
               for (size_t i = 0; i < buffer.size(); ++i)
               {
                   if (!(buffer[i][1]==2222 && buffer[i][2]==2222 && buffer[i][3]==2222) && !copy_buffer.empty()
                          && (-85 < buffer[i][1]) && (buffer[i][1] < 130) )

                   {
                       //Set distort center point
                       Mat_<Point2f> center_org(1,1), center_undis(1,1);
                       center_org(0) = Point2f(buffer[i][1]+320,buffer[i][2]+240);
                       //undistort
                       undistortPoints(center_org, center_undis, U_cameraMatrix, U_distCoeffs, Mat::eye(3,3,CV_64F), U_newCameraMatrix);
                       //qDebug()<<"X_org = "<<buffer[i][1]<<"Y_org = "<<buffer[i][2]<<endl;
                       //qDebug()<<"X_new = "<<center_undis[0]->x-320<<"Y_new = "<<center_undis[0]->y-240<<endl;
                       //cout<<center_undis<<endl;
                       //Transfer to Coordinate of Robot
                       //with z = 290 mm


                       float x_robot, y_robot, z_robot = 0;
                       float x_cam = (center_undis(0).x-320)*scale;
                       float y_cam = (center_undis(0).y-240)*scale;

                       //float x_cam = (buffer[i][1])*scale;
                       //float y_cam = (buffer[i][2])*scale;
                       Transfer_CorRobot(x_cam,y_cam,z_cam,
                                         x_robot,y_robot,z_robot);

                       //qDebug()<<"X_calib = "<<x_robot<< " Y_calib = "<< y_robot<<endl;
                       // Calculate delta
                       vector<float> delta_buffer;
                       for(size_t j = 0; j < copy_buffer.size(); j++)
                       {
                           float R = sqrt((x_robot - copy_buffer[j][1])*(x_robot - copy_buffer[j][1])
                                   + (y_robot - copy_buffer[j][2])*(y_robot - copy_buffer[j][2]));
                           delta_buffer.push_back(R);                           
                       }
                       int min_element_index = min_element(delta_buffer.begin(), delta_buffer.end()) - delta_buffer.begin() ;  //get the smallest index
                       copy_buffer[min_element_index][1] = 2222; //remove the used data by attach to it
                       copy_buffer[min_element_index][2] = 2222; //a dummy value, this index will be ignored
                                                                 //for the next loop because the large delta,
                                                                 //this prevents duplicate data for one index

                        if (Trans_buffer[min_element_index][7] == 0)
                        {
                           // Add in measurement matrix of Kalman Filter
                           meas[min_element_index].at<float>(0) = x_robot;
                           meas[min_element_index].at<float>(1) = y_robot;
                           meas[min_element_index].at<float>(2) = float(buffer[i][3]);
                           meas[min_element_index].at<float>(3) = float(buffer[i][0]);
                           // Set the Initialization for Kalman Filter
                           if (Trans_buffer[min_element_index][5] == 0) // if Flag Already Set the Initialization == false
                           {
                               Trans_buffer[min_element_index][5] = 1;
                               setIdentity(KF[min_element_index].errorCovPost);
                               state[min_element_index].at<float>(0) = meas[min_element_index].at<float>(0);
                               state[min_element_index].at<float>(1) = meas[min_element_index].at<float>(1);
                               state[min_element_index].at<float>(2) = 0;
                               state[min_element_index].at<float>(3) = -1;
                               state[min_element_index].at<float>(4) = meas[min_element_index].at<float>(2);
                               state[min_element_index].at<float>(5) = meas[min_element_index].at<float>(3);
                               // <<<< Initialization
                               KF[min_element_index].statePost = state[min_element_index];
                               // Trans_buffer
                               Trans_buffer[min_element_index][0] = buffer[i][0];
                               Trans_buffer[min_element_index][1] = x_robot;
                               Trans_buffer[min_element_index][2] = y_robot;
                               Trans_buffer[min_element_index][3] = z_robot;
                               Trans_buffer[min_element_index][4] = buffer[i][3];
                           }
                           else
                           {
                               KF[min_element_index].correct(meas[min_element_index]);
                           }
                        }

                   }

               }

           }
           for (size_t i = 0; i < Trans_buffer.size(); ++i)          //Check LIFE_CYCLE
           {
               if (copy_buffer.empty())
               {
                   Trans_buffer[i][6] ++;
               }
               else if(copy_buffer[i][1] == 2222 && copy_buffer[i][2] == 2222 && Trans_buffer[i][7] == 0)
               {
                   Trans_buffer[i][6] = 0;
               }
               else Trans_buffer[i][6] ++;

               if(Trans_buffer[i][6] >= 150 || Trans_buffer[i][2] <= 50)      // = 65
               {
                   Trans_buffer.erase(Trans_buffer.begin() + i);
                   KF.erase(KF.begin() + i);
                   state.erase(state.begin() + i);
                   meas.erase(meas.begin() + i);
                   break;

               }

           }
           if (Trans_buffer.empty() && current_point > 0)
           {
               current_point = 0;
           }

//           if (!Trans_buffer.empty())
//           {
//              emit newPixmapCaptured();
//           }
       }

       if (Ready)
       {
          Ready = false;
          emit newPixmapCaptured();
       }
   }
}



void Transfer_CorRobot(float x_cam, float y_cam, float z_cam,
                  float &x_robot, float &y_robot, float &z_robot)
{
//Tranfer Matrix
float A[4][4] =
  {
    {     -1,     0,    0,      332.3 },                // 1: 336.3
    {     0,     1,     0,      126.7 },                //126.7 _ 2: 135.7 _ 3:118.7 (focus)
    {     0,     0,     -1,     416.5 },
    {     0,     0,     0,          1 },
  };
 // Position of Object in Camera Coordinate
float B[4][1]=
  {
    {x_cam},
    {y_cam},
    {z_cam},
    {1}
  };
// Multiply 2 Matrix : C = A*B
  float C[4][1] ;
  int i,j,k;
  for(i=0;i<4;i++)
    for(j=0;j<1;j++)
    {
      C[i][j]=0;
      for(k=0;k<4;k++)
      {
        C[i][j]+=A[i][k]*B[k][j];
      }
    }
  //Object Position in Robot Coordinate
  x_robot = C[0][0];
  y_robot = C[1][0];
  z_robot = C[2][0];
}

void Set_KalmanFilter(vector<KalmanFilter>& KF, vector<Mat>& state, vector<Mat>& meas)
{
    //Set Kalman Filter
    KF.push_back(KalmanFilter(6, 4, 0, CV_32F));
    state.push_back(Mat(6, 1, CV_32F));  // [x,y,v_x,v_y,theta,id]
    meas.push_back(Mat(4, 1, CV_32F));   // [z_x,z_y,z_theta,z_id]
    int index = int(KF.size()) -1 ;
    // Transition State Matrix A
    // Note: set dT at each processing step!
        // [ 1 0 dT 0  0  0 ]
        // [ 0 1 0  dT 0  0 ]
        // [ 0 0 1  0  0  0 ]
        // [ 0 0 0  1  0  0 ]
        // [ 0 0 0  0  1  0 ]
        // [ 0 0 0  0  0  1 ]
    setIdentity(KF[index].transitionMatrix);
    // Measure Matrix H
    // [ 1 0 0 0 0 0 ]
    // [ 0 1 0 0 0 0 ]
    // [ 0 0 0 0 1 0 ]
    // [ 0 0 0 0 0 1 ]
    //setIdentity(KF[index].measurementMatrix);
    KF[index].measurementMatrix.at<float>(0) = 1;
    KF[index].measurementMatrix.at<float>(7) = 1;
    KF[index].measurementMatrix.at<float>(16) = 1;
    KF[index].measurementMatrix.at<float>(23) = 1;
    // Process Noise Covariance Matrix Q
    setIdentity(KF[index].processNoiseCov, cv::Scalar::all(1e-0));
    // Measures Noise Covariance Matrix R
    KF[index].measurementNoiseCov.at<float>(0) = 50*1e-0;
    KF[index].measurementNoiseCov.at<float>(5) = 1*1e-1;// The smaller scale value, the faster it change
    KF[index].measurementNoiseCov.at<float>(10) = 50*1e-0;
    KF[index].measurementNoiseCov.at<float>(15) = 50*1e-0;
}
