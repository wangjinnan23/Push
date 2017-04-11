/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <mutex>

#include<Eigen/Dense>

namespace ORB_SLAM2
{


MapDrawer::MapDrawer(Map* pMap, const string &strSettingPath):mpMap(pMap)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
    mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
    mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
    mPointSize = fSettings["Viewer.PointSize"];
    mCameraSize = fSettings["Viewer.CameraSize"];
    mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];

}

void MapDrawer::DrawMapPoints()
{
    const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
    const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();

    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);
    cout<<"vpMPs.size() is "<<vpMPs.size()<<endl;
    float xmin=0.0,xmax=0.0,ymin=0,ymax=0,zmin=0,zmax=0;
    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        if(xmin>pos.at<float>(0))
            xmin=pos.at<float>(0);
        if(xmax<pos.at<float>(0))
            xmax=pos.at<float>(0);
        if(ymin>pos.at<float>(1))
            ymin=pos.at<float>(1);
        if(ymax<pos.at<float>(1))
            ymax=pos.at<float>(1);
        if(zmin>pos.at<float>(2))
            zmin=pos.at<float>(2);
        if(zmax<pos.at<float>(2))
            zmax=pos.at<float>(2);
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
    }
    cout<<"xmin is "<<(float)xmin<<", xmax is "<<(float)xmax<<endl;
    cout<<"ymin is "<<(float)ymin<<", ymax is "<<(float)ymax<<endl;
    cout<<"zmin is "<<(float)zmin<<", zmax is "<<(float)zmax<<endl;
    glEnd();

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);

    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));

    }

    glEnd();
}
void MapDrawer::DrawDenseMapPoints()
{
    char *argv[5];
    argv[1]="/home/archer/GitHub/SLAM/ORB_SLAM/2/Demo/ORB_SLAM2/Vocabulary/ORBvoc.txt";
    argv[2]="/home/archer/GitHub/SLAM/ORB_SLAM/2/Demo/ORB_SLAM2/Examples/RGB-D/TUM1.yaml";
    argv[3]="/home/archer/GitHub/SLAM/ORB_SLAM/2/Demo/ORB_SLAM2/data/rgbd_dataset_freiburg1_room";
    argv[4]="/home/archer/GitHub/SLAM/ORB_SLAM/2/Demo/ORB_SLAM2/data/rgbd_dataset_freiburg1_room/associate.txt";
    ifstream fin("./pose.txt");
    if (!fin)
    {
        cerr<<"请在有pose.txt的目录下运行此程序"<<endl;
        return ;
    }
    glPointSize(mPointSize);
    glBegin(GL_POINTS);

for(int h=1;h<=5;h++)
{
    char rgb_path[]="/home/archer/GitHub/SLAM/ORB_SLAM/2/Demo/ORB_SLAM2/data/rgbd_dataset_freiburg1_room/Test/color/%d.png";
    char dep_path[]="/home/archer/GitHub/SLAM/ORB_SLAM/2/Demo/ORB_SLAM2/data/rgbd_dataset_freiburg1_room/Test/depth/%d.pgm";
    sprintf(rgb_path,rgb_path,h);
    sprintf(dep_path,dep_path,h);
    cout<<rgb_path<<endl;
    cout<<dep_path<<endl;
    cv::Mat imRGB = cv::imread(rgb_path,CV_LOAD_IMAGE_UNCHANGED);
    cv::Mat imD = cv::imread(dep_path,CV_LOAD_IMAGE_UNCHANGED);

    float xmin=0.0,xmax=0.0,ymin=0,ymax=0,zmin=0,zmax=0;

    double data[7] = {0};
    for ( auto& d:data )
    {
        fin>>d;
    }
    cout<<"!!!"<<data[0]<<endl;


    for(size_t i=0;i<imD.rows;i++)
    {
        for(size_t j=0;j<imD.cols; j++)
        {
            cv::Mat pos(1,3,CV_64FC1,cv::Scalar::all(0));
            pos.at<float>(2)=(float)imD.at<uchar>(i,j)/100;
            pos.at<float>(0)=(float)(i-318.643040)*pos.at<float>(2)/517.306408;
            pos.at<float>(1)=(float)(j-255.313989)*pos.at<float>(2)/516.469215;
            Eigen::Vector3d point;
            point[2]=pos.at<float>(2);
            point[1]=pos.at<float>(1);
            point[0]=pos.at<float>(0);
            Eigen::Quaterniond q( data[6], data[3], data[4], data[5] );
            Eigen::Isometry3d T(q);

            T.pretranslate( Eigen::Vector3d( data[0], data[1], data[2] ));
            Eigen::Vector3d pointWorld = T*point;
            pos.at<float>(2)=pointWorld[2];
            pos.at<float>(1)=pointWorld[1];
            pos.at<float>(0)=pointWorld[0];
            if(xmin>pos.at<float>(0))
                xmin=pos.at<float>(0);
            if(xmax<pos.at<float>(0))
                xmax=pos.at<float>(0);
            if(ymin>pos.at<float>(1))
                ymin=pos.at<float>(1);
            if(ymax<pos.at<float>(1))
                ymax=pos.at<float>(1);
            if(zmin>pos.at<float>(2))
                zmin=pos.at<float>(2);
            if(zmax<pos.at<float>(2))
                zmax=pos.at<float>(2);
            glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
            glColor3f((float)imRGB.at<cv::Vec3b>(i,j)[0]/255,(float)imRGB.at<cv::Vec3b>(i,j)[1]/255,(float)imRGB.at<cv::Vec3b>(i,j)[2]/255);
        }
        cout<<"xmin is "<<(float)xmin<<", xmax is "<<(float)xmax<<endl;
        cout<<"ymin is "<<(float)ymin<<", ymax is "<<(float)ymax<<endl;
        cout<<"zmin is "<<(float)zmin<<", zmax is "<<(float)zmax<<endl;

    }

    glEnd();
}
/*
    cv::Mat imRGB1 = cv::imread("/home/archer/GitHub/SLAM/ORB_SLAM/2/Demo/ORB_SLAM2/data/rgbd_dataset_freiburg1_room/Test/color/1.png",CV_LOAD_IMAGE_UNCHANGED);
    cv::Mat imD1 = cv::imread("/home/archer/GitHub/SLAM/ORB_SLAM/2/Demo/ORB_SLAM2/data/rgbd_dataset_freiburg1_room/Test/depth/1.png",CV_LOAD_IMAGE_UNCHANGED);

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    for(size_t i=0;i<imD1.rows;i++)
    {
        for(size_t j=0;j<imD1.cols; j++)
        {
            glVertex3f((float)(i-318.643040)/517.306408,(float)(j-255.313989)/516.469215,(float)(imD1.at<uchar>(i,j)/40));
            glColor3f((float)imRGB1.at<cv::Vec3b>(i,j)[0]/255,(float)imRGB1.at<cv::Vec3b>(i,j)[1]/255,(float)imRGB1.at<cv::Vec3b>(i,j)[2]/255);
        }
    }
    glEnd();
*/

}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph)
{
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;

    const vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();

    if(bDrawKF)
    {
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];
            cv::Mat Twc = pKF->GetPoseInverse().t();

            glPushMatrix();

            glMultMatrixf(Twc.ptr<GLfloat>(0));

            glLineWidth(mKeyFrameLineWidth);
            glColor3f(0.0f,0.0f,1.0f);
            glBegin(GL_LINES);
            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);

            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();
        }
    }

    if(bDrawGraph)
    {
        glLineWidth(mGraphLineWidth);
        glColor4f(0.0f,1.0f,0.0f,0.6f);
        glBegin(GL_LINES);

        for(size_t i=0; i<vpKFs.size(); i++)
        {
            // Covisibility Graph
            const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
            cv::Mat Ow = vpKFs[i]->GetCameraCenter();
            if(!vCovKFs.empty())
            {
                for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
                {
                    if((*vit)->mnId<vpKFs[i]->mnId)
                        continue;
                    cv::Mat Ow2 = (*vit)->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                    glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
                }
            }

            // Spanning tree
            KeyFrame* pParent = vpKFs[i]->GetParent();
            if(pParent)
            {
                cv::Mat Owp = pParent->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
            }

            // Loops
            set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
            for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
            {
                if((*sit)->mnId<vpKFs[i]->mnId)
                    continue;
                cv::Mat Owl = (*sit)->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owl.at<float>(0),Owl.at<float>(1),Owl.at<float>(2));
            }
        }

        glEnd();
    }
}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}


void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
{
    if(!mCameraPose.empty())
    {
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        {
            unique_lock<mutex> lock(mMutexCamera);
            Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
        }

        M.m[0] = Rwc.at<float>(0,0);
        M.m[1] = Rwc.at<float>(1,0);
        M.m[2] = Rwc.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<float>(0,1);
        M.m[5] = Rwc.at<float>(1,1);
        M.m[6] = Rwc.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<float>(0,2);
        M.m[9] = Rwc.at<float>(1,2);
        M.m[10] = Rwc.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15]  = 1.0;
    }
    else
        M.SetIdentity();
}

} //namespace ORB_SLAM
