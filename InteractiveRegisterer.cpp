#include "InteractiveRegisterer.h"

#include <opencv2/core/eigen.hpp>
#include "conversion.h"


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;

InteractiveRegisterer::InteractiveRegisterer()
:
	corresps_ (new pcl::Correspondences), cloud_left_ (new PointCloud), cloud_right_ (new PointCloud), lefties_ (new PointCloud), righties_ (new PointCloud), aligned_cloud_left_ (new PointCloud), aligned_cloud_right_ (new PointCloud)
{
    lefties_->height = 1;
    lefties_->width = 0; // we will increment in width
    lefties_->points.resize( lefties_->height * lefties_->width );
        
    righties_->height = 1;
    righties_->width = 0;
    righties_->points.resize( righties_->height * righties_->width );
            
    must_translate_ = false;
    must_align_ = false;
}

InteractiveRegisterer::InteractiveRegisterer(const InteractiveRegisterer& other)
{
    *this = other;
}

InteractiveRegisterer::~InteractiveRegisterer()
{
    
}

InteractiveRegisterer& InteractiveRegisterer::operator=(const InteractiveRegisterer& other)
{
    if (this != &other)
    {
        cloud_left_     = other.cloud_left_;
        cloud_right_    = other.cloud_right_;
        
        lefties_        = other.lefties_;
        righties_       = other.righties_;
        lefties_idx_    = other.lefties_idx_;
        righties_idx_   = other.righties_idx_;
        
        reallocate_points_  = other.reallocate_points_;
        num_points_         = other.num_points_;
        
        must_translate_ = other.must_translate_;
        must_align_     = other.must_align_;
        
        corresps_ = other.corresps_;
        
        m_tLeft             = other.m_tLeft;
        m_tRight            = other.m_tRight;
        m_Transformation    = other.m_Transformation;
        m_InverseTransformation    = other.m_InverseTransformation;
        
        aligned_cloud_left_     = other.aligned_cloud_left_;
        aligned_cloud_right_    = other.aligned_cloud_right_;
    }
    
    return *this;
}

void InteractiveRegisterer::setInputFrames(ColorFrame cFrameA, ColorFrame cFrameB, DepthFrame dFrameA, DepthFrame dFrameB)
{
    m_ColorFrameA = cFrameA;
    m_ColorFrameB = cFrameB;
    m_DepthFrameA = dFrameA;
    m_DepthFrameB = dFrameB;
}

void InteractiveRegisterer::interact()
{
    m_pViz = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer ("PCL OpenNI cloud"));
    m_pViz->registerMouseCallback (&InteractiveRegisterer::mouseCallback, *this);
    m_pViz->registerKeyboardCallback(&InteractiveRegisterer::keyboardCallback, *this);
    m_pViz->registerPointPickingCallback (&InteractiveRegisterer::ppCallback, *this);
    
    m_pViz->createViewPort(0, 0, 0.5f, 1.0f, viewport_left_);
    m_pViz->createViewPort(0.5f, 0, 1.f, 1.0f, viewport_right_);
    
    //m_pViz->addCoordinateSystem(0.1, 0, 0, 0, "cs1", viewport_left_);
    //m_pViz->addCoordinateSystem(0.1, 0, 0, 0, "cs2", viewport_right_);
    
    m_pViz->setSize(1280, 480);
    
    setDefaultCamera(m_pViz, viewport_left_);
    setDefaultCamera(m_pViz, viewport_right_);
    
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_left	(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_right (new pcl::PointCloud<pcl::PointXYZRGB>);

	m_DepthFrameA.getColoredPointCloud(m_ColorFrameA, *cloud_left);
	m_DepthFrameB.getColoredPointCloud(m_ColorFrameB, *cloud_right);
    
    pcl::copyPointCloud(*cloud_left, *cloud_left_);
    pcl::copyPointCloud(*cloud_right, *cloud_right_);

    m_pViz->addPointCloud (cloud_left, "Left cloud", viewport_left_);
    m_pViz->addPointCloud (cloud_right, "Right cloud", viewport_right_);

//    while (!must_translate_)
//    {
//        m_pViz->spinOnce(100);
//    }
//    stop(*m_pViz);
    
    //find_transformation(ref_points_src, ref_points_tgt, m_Transformation);
}


void InteractiveRegisterer::computeTransformation()
{
    find_transformation(lefties_, righties_, m_Transformation);
    m_InverseTransformation = m_Transformation.inverse();
    
    registration(cloud_left_, cloud_right_,
                 *aligned_cloud_left_, *aligned_cloud_right_);
}

void InteractiveRegisterer::computeFineTransformation()
{
    PointCloud downsampled_aligned_cloud_left, downsampled_aligned_cloud_right;
    // TODO: implementation
}

void InteractiveRegisterer::setNumPoints(int num_points)
{
	num_points_ = num_points;
}

pair<PointCloudPtr,PointCloudPtr> InteractiveRegisterer::getRegisteredClouds()
{
    return pair<PointCloudPtr,PointCloudPtr>(aligned_cloud_left_, aligned_cloud_right_);
}

void InteractiveRegisterer::translate(const PointCloudPtr cloud, PointT t, 
	PointCloud& cloud_ctr)
{
    cloud_ctr.width  = cloud->width;
    cloud_ctr.height = cloud->height;
    cloud_ctr.resize( cloud_ctr.width * cloud_ctr.height );
        
    for (int i = 0; i < cloud->points.size(); i++)
    {
        cloud_ctr.points[i].x = cloud->points[i].x - t.x;
        cloud_ctr.points[i].y = cloud->points[i].y - t.y;
        cloud_ctr.points[i].z = cloud->points[i].z - t.z;
    }
}


void InteractiveRegisterer::translate(const PointCloudPtr cloud, Eigen::Vector4f t, 
	PointCloud& cloud_ctr)
{
    cloud_ctr.width  = cloud->width;
    cloud_ctr.height = cloud->height;
    cloud_ctr.resize( cloud_ctr.width * cloud_ctr.height );
        
    for (int i = 0; i < cloud->points.size(); i++)
    {
        cloud_ctr.points[i].x = cloud->points[i].x - t.x();
        cloud_ctr.points[i].y = cloud->points[i].y - t.y();
        cloud_ctr.points[i].z = cloud->points[i].z - t.z();
    }
}


void InteractiveRegisterer::keyboardCallback(const pcl::visualization::KeyboardEvent& keyboard_event, void*)
{
    cout << "the key \'" << keyboard_event.getKeyCode() << "\' (" << keyboard_event.getKeyCode() << ") was";
    if (keyboard_event.getKeyCode ())
        cout << "the key \'" << keyboard_event.getKeyCode() << "\' (" << keyboard_event.getKeyCode() << ") was";
    else
        cout << "the special key \'" << keyboard_event.getKeySym() << "\' was";
    if (keyboard_event.keyDown())
        cout << " pressed" << endl;
    else
        cout << " released" << endl;
}


void InteractiveRegisterer::mouseCallback(const pcl::visualization::MouseEvent& mouse_event, void*)
{
    m_MouseX = mouse_event.getX();
    m_MouseY = mouse_event.getY();
    
//    if (mouse_event.getType() == pcl::visualization::MouseEvent::MouseButtonPress && mouse_event.getButton() == pcl::visualization::MouseEvent::LeftButton)
//    {
//    }
    
    if (mouse_event.getType() == pcl::visualization::MouseEvent::MouseButtonPress && mouse_event.getButton() == pcl::visualization::MouseEvent::RightButton)
    {
        int x, y, z;
        int view = (m_MouseX < 640) ? 0 : 1;
        std::stringstream ss_sphere, ss_line;

        if (view == 0 && lefties_->points.size() > 0)
        {
            x = m_MouseX;
            y = m_MouseY;
            z = cloud_left_->at(x,y).z;
            
            float distance;
            int idx = -1;
            for (int i = 0; i < lefties_->points.size() && idx < 0; i++)
            {
                distance = sqrtf(pow(x - lefties_->points[i].x, 2)
                                 + pow(y - lefties_->points[i].y, 2)
                                 + pow(z - lefties_->points[i].z, 2));
                if (distance < 0.1) idx = i;
            }
            ss_sphere << "left sphere " << idx;
            m_pViz->removeShape(ss_sphere.str());
            
            lefties_->points.erase(lefties_->points.begin() + idx);
            lefties_->width--; // add a point in a row
            
            lefties_idx_.erase(lefties_idx_.begin() + idx);
            
            std::cout << "removed left" << std::endl;
            
        }
        else if (view == 1 && righties_->points.size() > 0)
        {
            x = m_MouseX - 640;
            y = m_MouseY;
            z = cloud_left_->at(x,y).z;
            
            float distance;
            int idx = -1;
            for (int i = 0; i < righties_->points.size() && idx < 0; i++)
            {
                distance = sqrtf(pow(x - righties_->points[i].x, 2)
                                 + pow(y - righties_->points[i].y, 2)
                                 + pow(z - righties_->points[i].z, 2));
                if (distance < 0.1) idx = i;
            }
            ss_sphere << "right sphere " << idx;
            m_pViz->removeShape(ss_sphere.str());
            
            righties_->points.erase(righties_->points.begin() + idx);
            righties_->width--; // add a point in a row
            
            righties_idx_.erase(righties_idx_.begin() + idx);
            
            std::cout << "removed right" << std::endl;
        }
    }
}


void InteractiveRegisterer::ppCallback(const pcl::visualization::PointPickingEvent& pointpicking_event, void*)
{       
    // Error handling
    if (pointpicking_event.getPointIndex () == -1) return;
        
    float x, y, z;
    pointpicking_event.getPoint(x,y,z);
        
    cout << "left + shift button pressed @ " << x << " , " << y << " , " << z << endl;
    
    int view = (m_MouseX < 640) ? 0 : 1;

    // If not all points picked up in both sides, continue
    std::stringstream ss_sphere, ss_line;

    if (view == 0)
    {
        float distance;
        int idx = -1;
        for (int i = 0; i < lefties_->points.size() && idx < 0; i++)
        {
            distance = sqrtf(pow(x - lefties_->points[i].x, 2)
                             + pow(y - lefties_->points[i].y, 2)
                             + pow(z - lefties_->points[i].z, 2));
            if (distance < 0.1) idx = i;
        }
        
        if (idx >=0)
        {
            ss_sphere << "left sphere " << idx;
            m_pViz->removeShape(ss_sphere.str());
            
            lefties_->points.erase(lefties_->points.begin() + idx);
            lefties_->width--; // add a point in a row
            
            lefties_idx_.erase(lefties_idx_.begin() + idx);
            
            std::cout << "removed left" << std::endl;
        }
        else if (lefties_->points.size() < num_points_)
        {
            ss_sphere << "left sphere " << lefties_->points.size();
        
            if (lefties_->empty()) m_tLeft << x, y, z, 1;
            
            int idx = lefties_->points.size();
            m_pViz->addSphere(PointT(x,y,z), 0.03, colors[idx][0], colors[idx][1], colors[idx][2], ss_sphere.str(), viewport_left_);
                
            lefties_->points.push_back(PointT(x,y,z));
            lefties_->width++; // add a point in a row
                
            lefties_idx_.push_back(pointpicking_event.getPointIndex());
            
            std::cout << "added left" << std::endl;
        }
    }
    else if (view == 1)
    {
        float distance;
        int idx = -1;
        for (int i = 0; i < righties_->points.size() && idx < 0; i++)
        {
            distance = sqrtf(pow(x - righties_->points[i].x, 2)
                             + pow(y - righties_->points[i].y, 2)
                             + pow(z - righties_->points[i].z, 2));
            if (distance < 0.1) idx = i;
        }
        
        if (idx >= 0)
        {
            ss_sphere << "right sphere " << idx;
            m_pViz->removeShape(ss_sphere.str());
            
            righties_->points.erase(righties_->points.begin() + idx);
            righties_->width--; // add a point in a row
            
            righties_idx_.erase(righties_idx_.begin() + idx);
            
            std::cout << "removed right" << std::endl;
        }
        else if (righties_->points.size() < num_points_)
        {
            ss_sphere << "right sphere " << righties_->points.size();
            
            if (righties_->empty()) m_tRight << x, y, z, 1;                
            
            int idx = righties_->points.size();
            m_pViz->addSphere(PointT(x,y,z), 0.03, colors[idx][0], colors[idx][1], colors[idx][2], ss_sphere.str(), viewport_right_);
                
            righties_->points.push_back(PointT(x,y,z));
            righties_->width++;
                
            righties_idx_.push_back(pointpicking_event.getPointIndex());
            
            std::cout << "added right" << std::endl;
        }
    }
    
    if (!must_translate_ && lefties_->points.size() == num_points_ && righties_->points.size() == num_points_)
    {
        //pcl::PCDWriter writer;
        //writer.write("lefties.pcd", *lefties_);
        //writer.write("righties.pcd", *righties_);
        //    
        //cv::FileStorage fs;
        //fs.open("indices.yml", cv::FileStorage::WRITE);
        //fs << "lefties_idx_" << lefties_idx_;
        //fs << "righties_idx_" << righties_idx_;
            
        must_translate_ = true;
        //std::cout << "All " << num_points_ << " have been defined." << std::endl;
    }
}

void InteractiveRegisterer::setDefaultCamera(pcl::visualization::PCLVisualizer::Ptr viewer, int viewport)
{
    std::vector<pcl::visualization::Camera> cameras;
    viewer->getCameras(cameras);
        
    // Default parameters
    cameras[viewport].pos[2] = 2.0;
    cameras[viewport].focal[2] = 1.0;
    cameras[viewport_right_].view[1] = -1;
        
    m_pViz->setCameraParameters(cameras[viewport],viewport);
    m_pViz->updateCamera();
}


void InteractiveRegisterer::stop(pcl::visualization::PCLVisualizer & viewer)
{
    while (!viewer.wasStopped ())
    {
        viewer.close();
    }
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Find the transformation needed to align two sets of 3-D points (minimum 3)
* \param src the source points
* \param tgt the target points
* \param transformation the resultant transform between source and target
*/
void InteractiveRegisterer::find_transformation(const PointCloudPtr ref_points_src, const PointCloudPtr ref_points_tgt, Eigen::Matrix4f & transformation)
{
    Eigen::Vector4f centroid_src, centroid_tgt;
    pcl::compute3DCentroid(*ref_points_src, centroid_src);
    pcl::compute3DCentroid(*ref_points_tgt, centroid_tgt);
        
    PointCloudPtr centered_points_src (new PointCloud);
    PointCloudPtr centered_points_tgt (new PointCloud);
        
    translate(ref_points_src, centroid_src, *centered_points_src);
    translate(ref_points_tgt, centroid_tgt, *centered_points_tgt);
        
    // To opencv
        
    int n = 0;
        
    if (ref_points_src->width < ref_points_tgt->width)
    {
        n = ref_points_src->width;
    }
    else
    {
        n = ref_points_tgt->width;
    }
        
    if (n < 3)
    {
        return; // error
    }
        
    // Fill the matrices
    
    cv::Mat matrix_src (3, n, CV_32F);
    cv::Mat matrix_tgt (3, n, CV_32F);
                
    for (int i = 0; i < n; i++)
    {
        matrix_src.at<float>(0,i) = centered_points_src->points[i].x;
        matrix_src.at<float>(1,i) = centered_points_src->points[i].y;
        matrix_src.at<float>(2,i) = centered_points_src->points[i].z;

        matrix_tgt.at<float>(0,i) = centered_points_tgt->points[i].x;
        matrix_tgt.at<float>(1,i) = centered_points_tgt->points[i].y;
        matrix_tgt.at<float>(2,i) = centered_points_tgt->points[i].z;
    }
        
    cv::Mat h = cv::Mat::zeros(3, 3, CV_32F);
        
    for (int i = 0; i < n; i++)
    {
        cv::Mat aux;
        cv::transpose(matrix_tgt.col(i), aux);
            
        h = h + (matrix_src.col(i) * aux);
            
        aux.release();
    }
        
    cout << h << endl;
        
    cv::SVD svd;
    cv::Mat s, u, vt;
    svd.compute(h, s, u, vt);
        
    cv::Mat v, ut;
    cv::transpose(vt, v);
    cv::transpose(u, ut);
    vt.release();
        
    std::cout << v << std::endl;
    std::cout << (cv::determinant(v) < 0) << std::endl;
    if (cv::determinant(v) < 0)
    {
        v.col(2) = v.col(2) * (-1);
    }
    std::cout << v << std::endl;
    std::cout << (cv::determinant(v) < 0) << std::endl;
        
    cv::Mat r;
    r = v * ut;
        
    std::cout << r << std::endl;
    std::cout << (cv::determinant(r) < 0) << std::endl;
    if (cv::determinant(r) < 0)
    {
        r.col(2) = r.col(2) * (-1);
    }
    std::cout << r << std::endl;
    std::cout << (cv::determinant(r) < 0) << std::endl;

    transformation <<
        r.at<float>(0,0), r.at<float>(0,1), r.at<float>(0,2),  0,
        r.at<float>(1,0), r.at<float>(1,1), r.at<float>(1,2),  0,
        r.at<float>(2,0), r.at<float>(2,1), r.at<float>(2,2),  0,
                       0,                0,                0,  1;
        
    std::cout << transformation << std::endl;
}

//
//void InteractiveRegisterer::align(
//    const PointCloudPtr cloud_src, const PointCloudPtr cloud_tgt,
//	PointCloudPtr ref_points_src, PointCloudPtr ref_points_tgt, 
//	PointCloudPtr cloud_src_aligned, PointCloudPtr cloud_tgt_aligned)
//{
//    Eigen::Vector4f centroid_cloud_src, centroid_cloud_tgt;
//    pcl::compute3DCentroid(*ref_points_src, centroid_cloud_src);
//    pcl::compute3DCentroid(*ref_points_tgt, centroid_cloud_tgt);
//        
//    PointCloudPtr pCloudSrcCentered (new PointCloud);
//    translate(cloud_src, centroid_cloud_src, *pCloudSrcCentered);
//    translate(cloud_tgt, centroid_cloud_tgt, *cloud_tgt_aligned);
//        
//    pcl::transformPointCloud(*pCloudSrcCentered, *cloud_src_aligned, m_Transformation);
//}

//void InteractiveRegisterer::align(DepthFrame dFrameA, DepthFrame dFrameB) // public
//{        
//	PointCloudPtr cloud_left_	(new PointCloud);
//	PointCloudPtr cloud_right_ (new PointCloud);
//
//	dFrameA.getPointCloud(*cloud_left_);
//	dFrameB.getPointCloud(*cloud_right_);
////
////    m_pViz->addPointCloud (cloud_left_, "Left cloud", viewport_left_);
////    m_pViz->addPointCloud (cloud_right_, "Right cloud", viewport_right_);
////        
////    while (!must_translate_)
////    {
////        m_pViz->spinOnce(100);
////    }
////    stop(*m_pViz);
////
//    PointCloudPtr cloud_src_aligned (new PointCloud), cloud_tgt_aligned (new PointCloud);
//    align(cloud_left_, cloud_right_, lefties_, righties_, cloud_src_aligned, cloud_tgt_aligned);
//        
//    // Visualize and that shit
//	visualizeRegistration(cloud_src_aligned, cloud_tgt_aligned);
//}


void InteractiveRegisterer::saveTransformation(std::string filePath)
{
	cv::Mat tLeftMat, tRightMat, transfMat;
	cv::eigen2cv(m_tLeft, tLeftMat);
	cv::eigen2cv(m_tRight, tRightMat);
	cv::eigen2cv(m_Transformation, transfMat);

	cv::FileStorage fs (filePath + "transformation.yml", cv::FileStorage::WRITE);
	fs << "tLeftMat" << tLeftMat;
	fs << "tRightMat" << tRightMat;
	fs << "transfMat" << transfMat;

	fs.release();
    
    pcl::PCDWriter pcdwriter;
    pcdwriter.write(filePath + "registeredA.pcd", *aligned_cloud_left_);
    pcdwriter.write(filePath + "registeredB.pcd", *aligned_cloud_right_);
}


bool InteractiveRegisterer::loadTransformation(std::string filePath)
{
	cv::Mat tLeftMat, tRightMat, transfMat;
	cv::FileStorage fs (filePath + "transformation.yml", cv::FileStorage::READ);
    
    if (!fs.isOpened())
        return false;
    
	fs["tLeftMat"] >> tLeftMat;
	fs["tRightMat"] >> tRightMat;
	fs["transfMat"] >> transfMat;

	fs.release();

	if (transfMat.rows > 0 && transfMat.cols > 0)
	{
		cv::cv2eigen(transfMat, m_Transformation);
        m_InverseTransformation = m_Transformation.inverse();
		cv::cv2eigen(tLeftMat, m_tLeft);
		cv::cv2eigen(tRightMat, m_tRight);
        
        pcl::PCDReader pcdreader;
        pcdreader.read(filePath + "registeredA.pcd", *aligned_cloud_left_);
        pcdreader.read(filePath + "registeredB.pcd", *aligned_cloud_right_);
        
		return true;
	}
	else
	{
		return false;
	}
}

PointT InteractiveRegisterer::registration(PointT point, int viewpoint)
{
    PointT regPoint;
    
    deregistration(point, regPoint, viewpoint);
    
    return regPoint;
}

void InteractiveRegisterer::registration(PointT point, PointT& regPoint, int viewpoint)
{
    PointCloudPtr pCloud (new PointCloud);
    PointCloudPtr pRegCloud (new PointCloud);
    
    pCloud->push_back(point);
    pCloud->height = pCloud->width = 1;
    
    registration(pCloud, *pRegCloud, viewpoint);
    
    regPoint = pRegCloud->points[0];
}

void InteractiveRegisterer::registration(PointT pointA, PointT pointB, PointT& regPointA, PointT& regPointB)
{
    PointCloudPtr pCloudA (new PointCloud);
    PointCloudPtr pCloudB (new PointCloud);
    PointCloudPtr pRegCloudA (new PointCloud);
    PointCloudPtr pRegCloudB (new PointCloud);
    
    pCloudA->push_back(pointA);
    pCloudA->height = pCloudA->width = 1;
    pCloudB->push_back(pointB);
    pCloudB->height = pCloudB->width = 1;

    registration(pCloudA, pCloudB, *pRegCloudA, *pRegCloudB);
    
    regPointA = pRegCloudA->points[0];
    regPointB = pRegCloudB->points[0];
}

void InteractiveRegisterer::registration(PointCloudPtr pCloudA, PointCloudPtr pCloudB, PointCloud& regCloudA, PointCloud& regCloudB)
{
    registration(pCloudA, regCloudA, 0);
    registration(pCloudB, regCloudB, 1);
}

void InteractiveRegisterer::registration(PointCloudPtr pCloud, PointCloud& regCloud, int viewpoint)
{
    if (viewpoint == 0)
    {
        PointCloudPtr pCtrCloud (new PointCloud);
        translate(pCloud, m_tLeft, *pCtrCloud);
        pcl::transformPointCloud(*pCtrCloud, regCloud, m_Transformation);
    }
    else
    {
        translate(pCloud, m_tRight, regCloud);
    }
}

void InteractiveRegisterer::registration(vector<PointCloudPtr> pCloudsA, vector<PointCloudPtr> pCloudsB, vector<PointCloudPtr>& pRegCloudsA, vector<PointCloudPtr>& pRegCloudsB)
{
    for (int i = 0; i < pRegCloudsA.size(); i++)
    {
        PointCloudPtr pRegCloudA (new PointCloud);
        registration(pCloudsA[i], *pRegCloudA, 0);
        
        pCloudsA.push_back(pRegCloudA);
    }
    
    for (int i = 0; i < pRegCloudsB.size(); i++)
    {
        PointCloudPtr pRegCloudB (new PointCloud);
        registration(pCloudsB[i], *pRegCloudB, 0);
        
        pCloudsA.push_back(pRegCloudB);
    }
}

PointT InteractiveRegisterer::deregistration(PointT regPoint, int viewpoint)
{
    PointT point;
    
    deregistration(regPoint, point, viewpoint);
    
    return point;
}

void InteractiveRegisterer::deregistration(PointT regPoint, PointT& point, int viewpoint)
{
    PointCloudPtr pRegCloud (new PointCloud);
    PointCloudPtr pCloud (new PointCloud);
    
    pRegCloud->push_back(regPoint);
    pRegCloud->height = pRegCloud->width = 1;
    
    deregistration(pRegCloud, *pCloud, viewpoint);
    
    point = pCloud->points[0];
}

void InteractiveRegisterer::deregistration(PointT regPointA, PointT regPointB, PointT& pointA, PointT& pointB)
{
    PointCloudPtr pRegCloudA (new PointCloud);
    PointCloudPtr pRegCloudB (new PointCloud);
    PointCloudPtr pCloudA (new PointCloud);
    PointCloudPtr pCloudB (new PointCloud);
    
    pRegCloudA->push_back(regPointA);
    pRegCloudA->height = pRegCloudA->width = 1;
    pRegCloudB->push_back(regPointB);
    pRegCloudB->height = pRegCloudB->width = 1;

    deregistration(pRegCloudA, pRegCloudB, *pCloudA, *pCloudB);
    
    pointA = pCloudA->points[0];
    pointB = pCloudB->points[0];
}

void InteractiveRegisterer::deregistration(PointCloudPtr pRegCloudA, PointCloudPtr pRegCloudB, PointCloud& cloudA, PointCloud& cloudB)
{
    if (!pRegCloudA->empty()) deregistration(pRegCloudA, cloudA, 0);
    if (!pRegCloudB->empty()) deregistration(pRegCloudB, cloudB, 1);
}

void InteractiveRegisterer::deregistration(PointCloudPtr pRegCloud, PointCloud& cloud, int viewpoint)
{
    if (viewpoint == 0)
    {
        PointCloudPtr pCtrCloud (new PointCloud);
        pcl::transformPointCloud(*pRegCloud, *pCtrCloud, m_InverseTransformation);
        translate(pCtrCloud, -m_tLeft,  cloud);
    }
    else
    {
        translate(pRegCloud, -m_tRight, cloud);
    }
}

void InteractiveRegisterer::deregistration(vector<PointCloudPtr> pRegCloudsA, vector<PointCloudPtr> pRegCloudsB, vector<PointCloudPtr>& pCloudsA, vector<PointCloudPtr>& pCloudsB)
{
    for (int i = 0; i < pRegCloudsA.size(); i++)
    {
        PointCloudPtr pCloudA (new PointCloud);
        deregistration(pRegCloudsA[i], *pCloudA, 0);
        
        pCloudsA.push_back(pCloudA);
    }
    
    for (int i = 0; i < pRegCloudsB.size(); i++)
    {
        PointCloudPtr pCloudB (new PointCloud);
        deregistration(pRegCloudsB[i], *pCloudB, 1);
        
        pCloudsB.push_back(pCloudB);
    }
}

void InteractiveRegisterer::registration(DepthFrame frameA, DepthFrame frameB,
	PointCloud& regCloudA, PointCloud& regCloudB, 
	bool bBackgroundPoints, bool bUserPoints)
{    
	PointCloudPtr pCloudA (new PointCloud);
	PointCloudPtr pCloudB (new PointCloud);

	if (bBackgroundPoints && bUserPoints)
	{
		frameA.getPointCloud(*pCloudA);
		frameB.getPointCloud(*pCloudB);
	}
	else if (bBackgroundPoints && !bUserPoints)
	{
		frameA.getUserFreePointCloud(*pCloudA);
		frameB.getUserFreePointCloud(*pCloudB);
	}
	else if (!bBackgroundPoints && bUserPoints)
	{
		frameA.getForegroundPointCloud(*pCloudA);
		frameA.getForegroundPointCloud(*pCloudB);
	}
	else
	{
		frameA.getForegroundUserFreePointCloud(*pCloudA);
		frameB.getForegroundUserFreePointCloud(*pCloudB);
	}

//    PointCloudPtr pCtrCloudA (new PointCloud);
//    translate(pCloudA, m_tLeft, *pCtrCloudA);
//    translate(pCloudB, m_tRight, regCloudB);
//        
//    pcl::transformPointCloud(*pCtrCloudA, regCloudA, m_Transformation);
    
    registration(pCloudA, pCloudB, regCloudA, regCloudB);
}


void InteractiveRegisterer::visualizeRegistration(PointCloudPtr cloudA, PointCloudPtr cloudB)
{
    pcl::visualization::PCLVisualizer::Ptr pViz (new pcl::visualization::PCLVisualizer);
	visualizeRegistration(pViz, cloudA, cloudB);
}

void InteractiveRegisterer::visualizeRegistration(pcl::visualization::PCLVisualizer::Ptr pViz, PointCloudPtr cloudA, PointCloudPtr cloudB)
{
	pViz->removePointCloud("cloud left");
    pViz->removePointCloud("cloud right");

    pViz->addPointCloud (cloudA, "cloud left");
    pViz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 0, 0, "cloud left");
    pViz->addPointCloud (cloudB, "cloud right");
    pViz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0.5, 0, "cloud right");
    
    pViz->spin();
}

void InteractiveRegisterer::visualizeRegistration(DepthFrame dFrameA, DepthFrame dFrameB)
{
	PointCloudPtr pCloudA (new PointCloud);
	PointCloudPtr pCloudB (new PointCloud);
    
	registration(dFrameA, dFrameB, *pCloudA, *pCloudB);
    
    visualizeRegistration(pCloudA, pCloudB);
}

PointT InteractiveRegisterer::getLeftRefPoint()
{
    return EigenToPointXYZ(m_tLeft);
}

PointT InteractiveRegisterer::getRightRefPoint()
{
    return EigenToPointXYZ(m_tRight);
}

