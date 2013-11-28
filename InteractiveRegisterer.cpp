#include "InteractiveRegisterer.h"

#include <opencv2/core/eigen.hpp>


InteractiveRegisterer::InteractiveRegisterer()
:
	corresps_ (new pcl::Correspondences), lefties_ (new pcl::PointCloud<pcl::PointXYZ> ()), righties_ (new pcl::PointCloud<pcl::PointXYZ> ())
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


void InteractiveRegisterer::interact()
{
    cloud_viewer_ = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer ("PCL OpenNI cloud"));
    cloud_viewer_->registerMouseCallback (&InteractiveRegisterer::mouseCallback, *this);
    cloud_viewer_->registerKeyboardCallback(&InteractiveRegisterer::keyboardCallback, *this);
    cloud_viewer_->registerPointPickingCallback (&InteractiveRegisterer::ppCallback, *this);
    
    cloud_viewer_->createViewPort(0, 0, 0.5f, 1.0f, viewport_left_);
    cloud_viewer_->createViewPort(0.5f, 0, 1.f, 1.0f, viewport_right_);
    
    cloud_viewer_->addCoordinateSystem(0.1, 0, 0, 0, viewport_left_);
    cloud_viewer_->addCoordinateSystem(0.1, 0, 0, 0, viewport_right_);
    
    cloud_viewer_->setSize(1280, 480);
    
    setDefaultCamera(cloud_viewer_, viewport_left_);
    setDefaultCamera(cloud_viewer_, viewport_right_);
}


void InteractiveRegisterer::setNumPoints(int num_points)
{
	num_points_ = num_points;
}


void InteractiveRegisterer::translate(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ t, 
	pcl::PointCloud<pcl::PointXYZ>& cloud_ctr)
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


void InteractiveRegisterer::translate(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Vector4f t, 
	pcl::PointCloud<pcl::PointXYZ>& cloud_ctr)
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
    if (mouse_event.getType() == pcl::visualization::MouseEvent::MouseButtonPress && mouse_event.getButton() == pcl::visualization::MouseEvent::LeftButton)
    {
        cout << "left button pressed @ " << mouse_event.getX () << " , " << mouse_event.getY () << endl;
    }
        
    if (mouse_event.getType() == pcl::visualization::MouseEvent::MouseButtonPress && mouse_event.getButton() == pcl::visualization::MouseEvent::RightButton && must_translate_)
    {
        cout << "right button pressed" << std::endl;
        must_align_ = true;
    }
}


void InteractiveRegisterer::ppCallback(const pcl::visualization::PointPickingEvent& pointpicking_event, void*)
{       
    // Error handling
    if (pointpicking_event.getPointIndex () == -1) return;
        
    float x, y, z;
    pointpicking_event.getPoint(x,y,z);
        
    cout << "left + shift button pressed @ " << x << " , " << y << " , " << z << endl;
        
        
    // If not all points picked up in both sides, continue.
    if (righties_->points.size() < num_points_)
    {
        std::stringstream ss_sphere, ss_line;
            
        if (lefties_->points.size() <= righties_->points.size())
        {
            std::cout << "added left" << std::endl;

			if (lefties_->empty()) m_tLeft << x, y, z, 1;	
                
            ss_sphere << "sphere " << lefties_->points.size() << " left";
                
            int idx = lefties_->points.size();
            cloud_viewer_->addSphere(pcl::PointXYZ(x,y,z), 0.01, colors[idx][0], colors[idx][1], colors[idx][2], ss_sphere.str(), viewport_left_);
                
            lefties_->points.push_back(pcl::PointXYZ(x,y,z));
            lefties_->width ++; // add a point in a row
                
            lefties_idx_.push_back(pointpicking_event.getPointIndex());
        }
        else
        {
            std::cout << "added right" << std::endl;

			if (righties_->empty()) m_tRight << x, y, z, 1;                
            ss_sphere << "sphere " << lefties_->points.size() << " right";
                
            int idx = righties_->points.size();
            cloud_viewer_->addSphere(pcl::PointXYZ(x,y,z), 0.01, colors[idx][0], colors[idx][1], colors[idx][2], ss_sphere.str(), viewport_right_);
                
            righties_->points.push_back(pcl::PointXYZ(x,y,z));
            righties_->width ++;
                
            righties_idx_.push_back(pointpicking_event.getPointIndex());
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
    cameras[viewport].pos[2] = -1.0;
    cameras[viewport].focal[2] = 1.0;
    cameras[viewport_right_].view[1] = -1;
        
    cloud_viewer_->setCameraParameters(cameras[viewport],viewport);
    cloud_viewer_->updateCamera();
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
void InteractiveRegisterer::find_transformation(const pcl::PointCloud<pcl::PointXYZ>::Ptr ref_points_src, const pcl::PointCloud<pcl::PointXYZ>::Ptr ref_points_tgt, Eigen::Matrix4f & transformation)
{
    Eigen::Vector4f centroid_src, centroid_tgt;
    pcl::compute3DCentroid(*ref_points_src, centroid_src);
    pcl::compute3DCentroid(*ref_points_tgt, centroid_tgt);
        
    pcl::PointCloud<pcl::PointXYZ>::Ptr centered_points_src (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr centered_points_tgt (new pcl::PointCloud<pcl::PointXYZ>);
        
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


void InteractiveRegisterer::align(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr ref_points_src, pcl::PointCloud<pcl::PointXYZ>::Ptr ref_points_tgt, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src_aligned, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt_aligned)
{
    find_transformation(ref_points_src, ref_points_tgt, m_Transformation);
        
    Eigen::Vector4f centroid_cloud_src, centroid_cloud_tgt;
    pcl::compute3DCentroid(*ref_points_src, centroid_cloud_src);
    pcl::compute3DCentroid(*ref_points_tgt, centroid_cloud_tgt);
        
    pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudSrcCentered (new pcl::PointCloud<pcl::PointXYZ>);
    translate(cloud_src, centroid_cloud_src, *pCloudSrcCentered);
    translate(cloud_tgt, centroid_cloud_tgt, *cloud_tgt_aligned);
        
    pcl::transformPointCloud(*pCloudSrcCentered, *cloud_src_aligned, m_Transformation);
}

void InteractiveRegisterer::computeTransformation(DepthFrame& dFrameA, DepthFrame& dFrameB)
{        
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_left_	(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_right_ (new pcl::PointCloud<pcl::PointXYZ>);

	dFrameA.getPointCloud(*cloud_left_);
	dFrameB.getPointCloud(*cloud_right_);

    cloud_viewer_->addPointCloud (cloud_left_, "Left cloud", viewport_left_);
    cloud_viewer_->addPointCloud (cloud_right_, "Right cloud", viewport_right_);
        
    while (!must_translate_)
    {
        cloud_viewer_->spinOnce(100);
    }
    stop(*cloud_viewer_);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src_aligned (new pcl::PointCloud<pcl::PointXYZ>), cloud_tgt_aligned (new pcl::PointCloud<pcl::PointXYZ>);
    align(cloud_left_, cloud_right_, lefties_, righties_, cloud_src_aligned, cloud_tgt_aligned);
        
    // Visualize and that shit
	visualizeRegistration(cloud_src_aligned, cloud_tgt_aligned);
}


void InteractiveRegisterer::saveTransformation(const char* filePath)
{
	cv::Mat tLeftMat, tRightMat, transfMat;
	cv::eigen2cv(m_tLeft, tLeftMat);
	cv::eigen2cv(m_tRight, tRightMat);
	cv::eigen2cv(m_Transformation, transfMat);

	cv::FileStorage fs (filePath, cv::FileStorage::WRITE);
	fs << "tLeftMat" << tLeftMat;
	fs << "tRightMat" << tRightMat;
	fs << "transfMat" << transfMat;

	fs.release();
}


bool InteractiveRegisterer::loadTransformation(const char* filePath)
{
	cv::Mat tLeftMat, tRightMat, transfMat;
	cv::FileStorage fs (filePath, cv::FileStorage::READ);

	fs["tLeftMat"] >> tLeftMat;
	fs["tRightMat"] >> tRightMat;
	fs["transfMat"] >> transfMat;

	fs.release();

	if (transfMat.rows > 0 && transfMat.cols > 0)
	{
		cv::cv2eigen(transfMat, m_Transformation);
		cv::cv2eigen(tLeftMat, m_tLeft);
		cv::cv2eigen(tRightMat, m_tRight);
		return true;
	}
	else
	{
		return false;
	}
}


void InteractiveRegisterer::getRegisteredClouds(DepthFrame frameA, DepthFrame frameB,
	pcl::PointCloud<pcl::PointXYZ>& regCloudA, pcl::PointCloud<pcl::PointXYZ>& regCloudB, 
	bool bBackgroundPoints, bool bUserPoints)
{    
	pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudA (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudB (new pcl::PointCloud<pcl::PointXYZ>);

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

    pcl::PointCloud<pcl::PointXYZ>::Ptr pCtrCloudA (new pcl::PointCloud<pcl::PointXYZ>);
    translate(pCloudA, m_tLeft, *pCtrCloudA);
    translate(pCloudB, m_tRight, regCloudB);
        
    pcl::transformPointCloud(*pCtrCloudA, regCloudA, m_Transformation);
}


void InteractiveRegisterer::visualizeRegistration(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB)
{
	pcl::visualization::PCLVisualizer::Ptr pViz (new pcl::visualization::PCLVisualizer);
    pViz->setWindowName("Fusion viewer");
        
    pViz->addCoordinateSystem(0.2, 0, 0, 0);
        
    pViz->addPointCloud (cloudA, "cloud left");
    pViz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud left");
    pViz->addPointCloud (cloudB, "cloud right");
    pViz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud right");
        
    while (!pViz->wasStopped())
    {
        pViz->spinOnce(200);
    }
}


void InteractiveRegisterer::visualizeRegistration(pcl::visualization::PCLVisualizer::Ptr pViz,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB)
{
	pViz->removePointCloud("cloud left");
    pViz->removePointCloud("cloud right");

    pViz->addPointCloud (cloudA, "cloud left");
    pViz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 0, 0, "cloud left");
    pViz->addPointCloud (cloudB, "cloud right");
    pViz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0.5, 0, "cloud right");
}


void InteractiveRegisterer::visualizeRegistration(DepthFrame dFrameA, DepthFrame dFrameB)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudA (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudB (new pcl::PointCloud<pcl::PointXYZ>);

	getRegisteredClouds(dFrameA, dFrameB, *pCloudA, *pCloudB);

	pcl::visualization::PCLVisualizer::Ptr pViz (new pcl::visualization::PCLVisualizer);
    pViz->setWindowName("Fusion viewer");
        
    pViz->addCoordinateSystem(0.2, 0, 0, 0);
        
    pViz->addPointCloud (pCloudA, "cloud left");
    pViz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud left");
    pViz->addPointCloud (pCloudB, "cloud right");
    pViz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud right");
        
    while (!pViz->wasStopped())
    {
        pViz->spinOnce(200);
    }
}